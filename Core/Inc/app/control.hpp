// SPDX-License-Identifier: proprietary
//
// control.hpp -- the pure ECU control core: the start / ready-to-drive FSM and
// the APPS/torque/plausibility computation. HAL-free and RTOS-free (it only
// takes a snapshot struct + a millisecond tick), so it compiles and runs in a
// host unit test with no mocks -- the AMS state_machine.hpp pattern.
//
// It is a non-blocking re-expression of the legacy VCU pre-jarama behaviour
// (its blocking startup sequence + the per-tick inverter switch + setTorque()).
// Two deliberate deviations from that legacy, both toward its documented
// intent (flagged inline in control.cpp):
//   1. the low-cell-voltage derate is APPLIED (legacy computed it but sent the
//      undated value);
//   2. the T.11.8.9 APPS-disagreement cut honours the 100 ms persistence window
//      (legacy had it commented out -> instant trip).
//
// What is NOT here: reading the ADC/GPIO (io_signals), the CAN wire encoding,
// and the inverter unit-map + two's-complement + E2E framing (the inverter
// adapter). The core emits a torque PERCENTAGE and an
// abstract inverter MODE; the firmware layer encodes them.

#ifndef ECU_CONTROL_HPP_
#define ECU_CONTROL_HPP_

#include <cstdint>

#include "app/as_buzzer.hpp"
#include "app/cell_derate.hpp"
#include "app/motor_thermal.hpp"
#include "app/pack_thermal.hpp"
#include "app/ecu_config.hpp"
#include "app/pedal_cal.hpp"
#include "app/power_limit.hpp"

namespace ecu {

// Abstract inverter command. The numeric values are the NX/EMC App_State_Req
// mode words (EMC_RX_SETPOINT_1 / 0x360); the inverter adapter writes them
// (with E2E) -- the core only decides which one.
enum class InvMode : uint8_t {
    Off            = 0x01,
    Ready          = 0x04,  // drive standby -> ready
    TorqueEnable   = 0x06,
    Fault          = 0x13,  // soft fault (App_State 10) reset
    HardFaultReset = 0x0D,  // hard fault (App_State 11) recovery -> clears the latch
};

// The start / ready-to-drive FSM.
enum class CtrlState : uint8_t {
    WaitInvVdcConfig = 0,  // wait for the inverter to report its DC-bus config (0x466)
    Precharge,             // stream 0x100; wait for AMS ok_precharge (0x020)
    WaitStartBrake,        // wait for START button + brake pressed
    R2dDelay,              // drive the RTDS buzzer for R2dSoundMs
    WaitInvStandby,        // command Ready; wait for inverter to reach ready
    Active,                // runtime torque
    AmsError,              // AMS latched Error -> inhibited, no precharge retry
};

// One 10 ms snapshot. Filled by control_task from io_signals (pedals/brake/
// button) + the vehicle_service snapshot (inverter + AMS). All HAL-free PODs.
struct CtrlInputs {
    // pedals / brake (raw 12-bit ADC) + debounced START button
    uint16_t apps1_raw = 0;
    uint16_t apps2_raw = 0;
    uint16_t brake_raw = 0;
    bool     start_button = false;
    // inverter feedback (FDCAN1 / EMC)
    bool     inv_present = false;        // any inverter frame seen recently
    bool     inv_vconfig_ready = false;  // 0x466 DC-bus config frame seen
    uint8_t  inv_state = 0;              // App_State_App (0x461); >=10 = fault
    uint16_t inv_dc_bus_V = 0;           // 0x466 DCBus_Voltage_V
    // AMS / ACU (FDCAN2)
    bool     ams_fresh = false;          // 0x020/0x4A0 recently received
    bool     ok_precharge = false;       // 0x020
    bool     ams_error = false;          // 0x4A0 fsm_state == AmsFsmError
    uint16_t v_cell_min_mV = 0;          // 0x12C / 0x4A0
    // Pack current, for the IR compensation that keeps the low-cell derate from
    // reacting to acceleration sag (cell_derate.hpp). Deciamps, + = discharge.
    // Freshness is tracked SEPARATELY from ams_fresh: 0x135 is its own frame at
    // its own cadence, and a bus-level liveness flag would let a dead current
    // signal keep compensating off a frozen value.
    int16_t  current_accu_dA = 0;        // 0x135
    bool     current_fresh = false;      // 0x135 recently received
    // Motor temperatures (0x464, raw bytes; degC = raw - 50) for the thermal
    // cap. Own freshness for the same reason as the current: the inverter can
    // keep talking on 0x461/0x463/0x466 while 0x464 alone dies, and a frozen
    // temperature reads as a healthy one.
    uint8_t  inv_temp_motor1_raw = 0;    // 0x464
    uint8_t  inv_temp_motor2_raw = 0;    // 0x464
    bool     inv_temps_fresh = false;    // 0x464 recently received
    // Accumulator per-module maxima (0x136/0x137, signed degC, no offset) plus
    // the AMS's own view of which modules are reporting (0x4A0 byte2). Own
    // freshness again -- and here it is the ONLY guard against an
    // uninitialised state, because 0 degC is a real pack temperature.
    int16_t  tmax_module[PackModuleCount] = {};
    uint8_t  module_online_mask = 0;     // 0x4A0 byte2
    bool     ams_status_fresh = false;   // 0x4A0 fresh -> the mask is trustworthy
    bool     pack_temps_fresh = false;   // 0x136/0x137 recently received
    // uDV / autonomous (FDCAN2). The DV ready-to-drive gate is
    // dv_r2d_req && brake_raw > BrakeDvHardRaw -- the EBS holds HARD braking
    // and the ECU verifies it on its own brake sensor; no start button in DV.
    bool     dv_r2d_req = false;         // 0x510 byte0 != 0 AND fresh (UdvR2dStaleMs)
    bool     dv_fresh = false;           // 0x507 stream fresh (UdvCmdStaleMs)
    uint8_t  dv_torque_pct = 0;          // 0x507 conditioned (clamped/NaN-rejected) 0..100
    // AS state from 0x50A, with its OWN freshness. Both are needed: losing the
    // frame while the uDV was DRIVING or READY is itself the emergency, so the
    // buzzer has to see the difference between "reported OFF" and "stopped
    // reporting". See as_buzzer.hpp.
    uint8_t  as_status = 0;              // 0x50A byte0, meaningful only when fresh
    bool     as_fresh  = false;          // 0x50A within UdvAsStaleMs
    // MECHANICAL motor speed (0x463 erpm / MotorPolePairs). Feeds the EV 2.2.1
    // power envelope. Feed-forward only: the cap reads this, it does not drive
    // it on the timescale of a control tick, so no loop is closed.
    int32_t  motor_rpm_mech = 0;
    // Pedal calibration, carried IN rather than read from a global so step()
    // stays a pure function of its arguments and the SIL suite can vary it.
    // Falls back to the ecu_config.hpp defaults when nothing is stored.
    PedalCal cal{};
};

struct CtrlOutput {
    CtrlState state = CtrlState::WaitInvVdcConfig;
    uint8_t   torque_pct = 0;    // 0..100, commanded (non-zero only in Active)
    InvMode   inv_mode = InvMode::Off;
    // Fault-recovery FOLLOW-UP mode words: extra App_State_Req values the task
    // sends on 0x360 AFTER inv_mode, within the SAME 10 ms cycle (n = 0 on the
    // normal path, so nothing changes when the inverter is healthy).
    //
    // WHY: the W90 manual 9.3 states that going to OFF is what restarts a FAULT
    // -- OFF is the only fault-clearing path the vendor documents (0x0D/0x13 do
    // not even appear in its App_State_Req list). The IFS07 VCU, which DID
    // recover without a power cycle, sent exactly this burst because its
    // App_State switch fell through with no breaks: soft fault -> 0x13, 0x0D,
    // 0x01; hard fault -> 0x0D, 0x01. Commanding the reset word ALONE (what we
    // did before) means the OFF that actually clears the fault is never sent,
    // so the inverter stays faulted and WaitInvStandby hangs forever.
    InvMode   inv_mode_follow[2] = { InvMode::Off, InvMode::Off };
    uint8_t   inv_mode_follow_n = 0;
    // Assert Flt_Clear (0x360 byte 2 bit 7) on the LAST follow-up word. Set only
    // while the inverter reports a fault, so the bit is PULSED, not held: the
    // primary word and any earlier follow word leave it clear, giving the
    // inverter a fresh rising edge every 10 ms cycle rather than a stuck-high
    // level (which an edge-triggered clear would see exactly once). The mode
    // words alone do not shift a LATCHED fault -- dem_present = 0, condition
    // already gone, inverter still parked in SoftFault(10).
    bool      inv_flt_clear = false;
    // MIND THE DIFFERENCE BETWEEN THESE THREE FLAGS.
    //
    // power_capped means the envelope ACTUALLY CLIPPED the request this tick --
    // it is set only when the demand exceeded the cap. Reported on 0x700, so a
    // driver complaining of "no power at the end of the straight" can be
    // answered from a capture instead of a guess.
    bool      power_capped = false;
    // The two thermal flags mean something WEAKER: the ceiling is below 100 %,
    // whether or not it clipped anything. At 60 % pedal with a 70 % motor cap,
    // thermal_capped is 1 and the cap is not costing you a single Nm.
    //
    // So when you are chasing a slow car, do NOT read these as "this is the one
    // limiting me". Compare torque_pct (0x700) against the published ceilings --
    // cap_pct (0x709), thermal_cap_pct (0x706), pack_cap_pct (0x70A) -- and the
    // ceiling equal to torque_pct is the binding one.
    //
    // Motor thermal ceiling is below full (including the unknown-sensor case).
    // Reported on 0x706 next to the temperatures that caused it.
    bool      thermal_capped = false;
    // Accumulator thermal ceiling is below full. Reported on 0x70A.
    bool      pack_thermal_capped = false;
    // Wrapping count of times Active dropped back to WaitInvStandby because the
    // inverter left the drive. Transient by nature -- the inverter
    // recovers and the FSM climbs again within a couple of ticks -- so without a
    // counter the event is invisible after the fact. On 0x708.
    uint8_t   inv_redrive_count = 0;
    // Commanded SHAFT torque in Nm (positive = forward). Reported on 0x700
    // torque_cmd, which was hardcoded to 0 before this.
    int16_t   torque_nm = 0;
    // Drive the RTDS buzzer. Two sources: the R2D tone in R2dDelay, and the AS
    // Emergency pattern, which OVERRIDES it -- a driverless emergency landing
    // during an R2D tone must not be masked by it.
    bool      rtds_on = false;
    // The AS Emergency tone is what is driving rtds_on this tick. Separated so
    // the pit-diag stream can tell an emergency from a normal R2D chirp.
    bool      as_buzzer_active = false;
    // The running tone was started by 0x50A going SILENT, not by an EMERGENCY
    // frame -- i.e. the uDV is gone rather than merely unhappy.
    bool      as_buzzer_from_stale = false;
    bool      ok_to_drive = false;
    // plausibility verdicts (driven into 0x700.flags for pit-diag / Block F)
    bool      t11_8_9 = false;   // APPS disagreement past the 100 ms window
    // DV drive latched this cycle (the 0x510+EBS gate fired) -> torque source
    // is the uDV 0x507 command; drives the 0x511 R2D-confirm emission.
    bool      dv_mode = false;
};

// Holds the FSM state + the stateful plausibility latches across 10 ms steps.
// Pure logic, no HAL/RTOS -> host-unit-testable.
class Controller {
public:
    CtrlOutput step(const CtrlInputs& in, uint32_t now_ms) noexcept;
    CtrlState  state() const noexcept { return state_; }

    // The low-cell estimator's working state from the last step(): the raw
    // reading, the IR correction applied, the resulting OCV estimate and the
    // cap. Published on pit-diag 0x709 -- R_cell is commissioned by watching
    // est_ocv_mV stay flat through an acceleration run, which is only possible
    // if the intermediate values leave the core.
    const CellDerateState& cell_derate() const noexcept { return cell_derate_; }

    // Motor thermal cap working state from the last step(): the filtered
    // temperature, per-sensor validity and the resulting cap. Published on
    // 0x706 so a thermal limit is visible rather than inferred.
    const MotorThermalState& motor_thermal() const noexcept { return motor_thermal_; }

    // Accumulator thermal cap working state from the last step(). Published on
    // 0x70A -- which modules were actually used is the part worth seeing, since
    // a silently-excluded module is how a hot pack goes unnoticed.
    const PackThermalState& pack_thermal() const noexcept { return pack_thermal_; }

private:
    void enter_(CtrlState s, uint32_t now_ms) noexcept;

    CtrlState state_ = CtrlState::WaitInvVdcConfig;
    uint32_t  state_entry_ms_ = 0;
    bool      apps_disagree_active_ = false;
    uint32_t  apps_disagree_since_ms_ = 0;
    // The mode decision: latched by WHICH trigger fired at WaitStartBrake
    // (manual start+brake vs DV 0x510+EBS-brake); cleared on any exit from the
    // drive ladder (enter_ to a pre-R2D state or AmsError). Never swaps live.
    bool      dv_latched_ = false;
    uint8_t   inv_redrive_count_ = 0;
    // IR-compensated low-cell estimator (cell_derate.hpp). Holds filter state,
    // so it lives with the FSM's other history rather than being rebuilt per
    // tick -- a filter reconstructed every call is just a passthrough.
    AsBuzzer        as_buzzer_{};
    CellDerate      cell_{};
    CellDerateState cell_derate_{};
    MotorThermal      thermal_{};
    MotorThermalState motor_thermal_{};
    PackThermal       pack_{};
    PackThermalState  pack_thermal_{};
};

// APPS travel %, clamped 0..100. Exposed for the pit-diag adapter (0x701) and
// unit tests.
uint8_t apps_pct(uint16_t raw, uint16_t adc_min, uint16_t adc_max) noexcept;

}  // namespace ecu

#endif  // ECU_CONTROL_HPP_
