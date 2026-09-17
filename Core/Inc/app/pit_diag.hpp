// SPDX-License-Identifier: proprietary
//
// Pit-diag frame builders: thin adapters that turn the live system state into
// the 0x700-0x705 / 0x7E1 CanFrames (the ECU's CAN-only observability stream,
// gated by 0x7E0). Pure (no HAL) -- the scaling lives here, the wire layout in
// the DSL .def files. CanTxTask/DiagTask call these; the gate + cadence live
// there.

#pragma once

#include "app/can_frame.hpp"
#include "app/control.hpp"
#include "app/cal_session.hpp"
#include "app/pedal_cal.hpp"
#include "app/error_latch.hpp"
#include "app/io_signals.hpp"
#include "app/reset_cause.hpp"
#include "app/vehicle_service.hpp"

#include <cstdint>

namespace ecu {

// Runtime health gathered by DiagTask for the 0x704 frame.
struct HealthMetrics {
    std::uint16_t free_heap     = 0;
    std::uint16_t min_free_heap = 0;
    std::uint8_t  task_ran_mask = 0;
    ResetCause    reset_cause   = ResetCause::Unknown;
    std::uint8_t  uptime_s      = 0;
    FaultCode     last_fault    = FaultCode::None;
};

class PitDiag {
public:
    static CanFrame build_status(const CtrlOutput& c, const VehicleState& v,
                                 bool start_button) noexcept;   // 0x700
    static CanFrame build_dv(const CtrlOutput& c, const CtrlInputs& in,
                             const VehicleState& v) noexcept;   // 0x707
    // 0x709 -- the low-cell derate estimator laid open (raw vs IR-compensated
    // vs filtered). This is the ONLY way CellIrMilliOhm gets commissioned: it
    // is right when est_ocv_mV stays flat through an acceleration run while
    // raw_mV sags. Takes the state straight off the controller, which already
    // computed it this tick.
    static CanFrame build_cell(const CellDerateState& s) noexcept;
    // 0x70A -- the accumulator thermal cap. valid_mask is the reason this frame
    // exists: the per-module temperatures are already on the dash, but which
    // ones the cap actually USED is not visible anywhere else.
    static CanFrame build_pack_temp(const PackThermalState& s) noexcept;
    // 0x70B / 0x70C -- the inverter's own FOC feedback and its torque ceiling.
    // Everything here was already arriving on FDCAN1 (the filter accepts every
    // standard ID) and was simply never decoded, which left "is the INVERTER
    // limiting us?" unanswerable from the car.
    static CanFrame build_inv_foc(const VehicleState& v, std::uint32_t now_ms) noexcept;
    static CanFrame build_inv_torque(const VehicleState& v,
                                     std::int16_t torque_req_nm) noexcept;
    // 0x70D -- commanded shaft power next to the inverter's measured AC power
    // and the DC bus. This is the frame that retires DrivetrainEffPct as a
    // guess: it is the entire compliance margin of the EV 2.2.1 envelope and it
    // has never been measured.
    static CanFrame build_power(const CtrlOutput& c, const VehicleState& v) noexcept;
    static CanFrame build_pedals(const IoInputs& io,
                                 const PedalCal& cal) noexcept;  // 0x701
    // 0x702. inv_mode_cmd is the App_State_Req the ECU is COMMANDING on 0x360
    // this cycle (CtrlOutput::inv_mode) -- everything else in the frame is what
    // the inverter REPORTS. Passed in rather than mirrored through a global: the
    // caller (control_task) already has the CtrlOutput in hand at this point.
    static CanFrame build_inverter(const VehicleState& v,
                                   std::uint8_t inv_mode_cmd) noexcept;
    // 0x706 -- the four inverter temperatures PLUS the motor thermal cap they
    // produced. The cap comes from the controller rather than being recomputed
    // here: motor_temp_used_degC is filtered and sensor-validated, so a second
    // implementation would drift from the one that actually limits torque.
    static CanFrame build_inverter_temps(const VehicleState& v,
                                         const MotorThermalState& th) noexcept;
    // 0x708 -- the inverter's L1 (PwrStg) / L2 (EMCtrl) fault-layer bitmasks.
    // L3 (DEM) alone cannot say whether a latched fault is held up by a live
    // hardware condition underneath.
    // now_ms is needed for inv_state_age_ms (freshness of 0x461).
    static CanFrame build_inv_faults(const VehicleState& v,
                                    const CtrlOutput& c,
                                    std::uint32_t now_ms) noexcept;  // 0x708
    static CanFrame build_fwinfo() noexcept;                    // 0x703
    static CanFrame build_health(const HealthMetrics& m) noexcept;  // 0x704
    static CanFrame build_brake(const IoInputs& io,
                                const PedalCal& cal) noexcept;   // 0x705
    static CanFrame build_ack(bool enabled) noexcept;           // 0x7E1
    // Calibration session. cal_load mirrors the boot-time outcome so the
    // calibration view does not have to also watch 0x704.
    static CanFrame build_cal_status(const CalSessionOutput& o,
                                     std::uint8_t last_cmd,
                                     std::uint8_t cal_load) noexcept;   // 0x7E3
    static CanFrame build_cal_apps(const PedalCal& c) noexcept;         // 0x7E4
    static CanFrame build_cal_brake(const PedalCal& c) noexcept;        // 0x7E5
};

}  // namespace ecu
