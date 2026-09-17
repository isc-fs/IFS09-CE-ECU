// SPDX-License-Identifier: proprietary
//
// ControlTask: the one realtime task and sole safety actor (the AMS MainTask
// pattern). Every 10 ms it reads the local IO, snapshots the vehicle service,
// steps the pure controller, drives the outputs (RTDS + status LEDs), emits the
// 0x100 heartbeat in EVERY state, optionally the pit-diag stream, and refreshes
// the IWDG. It is the only IWDG kicker -- if it wedges, the dog fires and the
// bootloader catches the reset.

#include "app/app_tasks.h"

#include "app/app_globals.h"
#include "app/can_tx.hpp"
#include "app/control.hpp"
#include "app/discharge.hpp"
#include "app/ecu_config.hpp"
#include "app/inverter.hpp"
#include "app/io_signals.hpp"
#include "app/cal_session.hpp"
#include "app/pedal_cal_flash.hpp"
#include "app/pedal_cal_nvm.hpp"
#include "app/pit_diag.hpp"
#include "app/udv_tx.hpp"
#include "app/vehicle_service.hpp"
#include "app/watchdog.hpp"

#include "can/can_codecs.hpp"

#include "cmsis_os2.h"
#include "main.h"
#include "fdcan.h"      // hfdcan1 -- FDCAN1 TX-health debug probe (ECU_DEBUG_INV_BRIDGE)

using namespace ecu;

extern "C" void ecu_control_task_run(void *argument) {
    (void)argument;

    Controller ctrl;
    Discharge  discharge;
    IoSignals  io;
    // The ACTIVE pedal calibration, loaded ONCE at task start from the
    // bootloader NVM sector. load_cal_from_nvm never fails: absent,
    // torn, unknown-version or invalid records all return the compile-time
    // defaults, because refusing to drive over a bad calibration would be worse
    // than running the values the car shipped with. The outcome is latched into
    // a global so the diagnostic stream can announce a silent fallback -- an
    // operator MUST be able to tell "my calibration is live" from "the ECU
    // quietly ignored it".
    const CalLoadResult cal_load =
        load_cal_from_nvm(reinterpret_cast<const void*>(CalNvmBase), CalNvmSize);
    // NOT const: a valid commit applies immediately (apply-on-commit). That is
    // safe because a session only opens while the vehicle is quiescent, so the
    // calibration never changes under a moving car. It also means the
    // operator's read-back shows what they just committed, instead of the old
    // values plus "now power-cycle".
    PedalCal   cal = cal_load.cal;
    CalSession cal_session;
    uint32_t   last_cal_status = 0;
    g_cal_load_status  = static_cast<std::uint8_t>(cal_load.status);
    g_cal_load_flags   = cal_load.flags;
    auto&      vs       = VehicleService::instance();
    uint32_t   last_pit = 0;
    uint32_t   last_udv = 0;
    uint32_t   tick     = osKernelGetTickCount();

    for (;;) {
        ++g_task_step[ECU_TASK_CONTROL];
        const uint32_t now = osKernelGetTickCount();

        // --- inputs ---
        IoInputs in{};
        io.read(in);
        const VehicleState veh = vs.snapshot();

        CtrlInputs ci{};
        ci.cal = cal;
        ci.apps1_raw         = in.apps1_raw;
        ci.apps2_raw         = in.apps2_raw;
        ci.brake_raw         = in.brake_raw;
        ci.start_button      = in.start_button;
        // Bench stub (config::StubNoInverter, ecu_config.hpp — folds away when false):
        // no inverter on FDCAN1, so bypass BOTH inverter gates (0x466 vconfig + Ready
        // state) and the FSM walks to Active on its own for the R2D/RTDS sequence +
        // APPS sweep. Ready(4) < fault(10) -> stays in TorqueEnable. DISABLES the
        // inverter handshake -- NEVER true for flight.
        if constexpr (config::StubNoInverter) {
            ci.inv_present       = true;
            ci.inv_vconfig_ready = true;
            ci.inv_state         = config::InvReadyState;
            ci.inv_dc_bus_V      = veh.inv_dc_bus_V;
        } else {
            ci.inv_present       = VehicleService::is_fresh(now, veh.last_inv_tick, config::InvStaleMs);
            ci.inv_vconfig_ready = (veh.last_vconfig_tick != 0u);
            ci.inv_state         = veh.inv_state;
            ci.inv_dc_bus_V      = veh.inv_dc_bus_V;
        }
        // Bench stub (config::StubNoAms): no AMS on the bus (inverter on bench PSUs)
        // -> assume precharge complete + AMS healthy so the FSM can reach Active.
        // WaitInvVdcConfig still gates on the inverter's 0x466 DC-bus report, so it
        // won't arm into a dead bus. DISABLES the AMS safety gate -- NEVER for flight.
        if constexpr (config::StubNoAms) {
            ci.ams_fresh         = true;
            ci.ok_precharge      = true;
            ci.ams_error         = false;
            ci.v_cell_min_mV     = config::CellVDefaultMv;   // healthy default -> no low-cell derate
            ci.current_accu_dA   = 0;
            ci.current_fresh     = false;                    // nothing to compensate on a bench PSU
            // No AMS -> no pack temperatures. Report a plausible cool pack so the
            // bench does not sit at the unknown-sensor cap; this whole branch is
            // already "pretend the AMS is healthy", and a stub that trips a
            // thermal limit would just be confusing.
            for (std::size_t m = 0; m < PackModuleCount; ++m) ci.tmax_module[m] = 25;
            ci.module_online_mask = 0x1Fu;
            ci.ams_status_fresh   = true;
            ci.pack_temps_fresh   = true;
        } else {
            const bool ams_fresh = VehicleService::is_fresh(now, veh.last_ams_tick, config::AmsStaleMs);
            ci.ams_fresh         = ams_fresh;
            ci.ok_precharge      = veh.ok_precharge && ams_fresh;   // stale AMS -> not ok (fail-safe)
            ci.ams_error         = (veh.ams_fsm_state == config::AmsFsmError) && ams_fresh;
            ci.v_cell_min_mV     = veh.v_cell_min_mV;
            // Own freshness window: 0x135 is 50 ms cyclic and feeds the derate's
            // IR compensation, which must stop the moment the current signal
            // does -- not when the whole AMS block goes quiet.
            ci.current_accu_dA   = veh.current_accu_dA;
            ci.current_fresh     = VehicleService::is_fresh(now, veh.last_currents_tick,
                                                            config::AcuCurrentsStaleMs);
            // Pack thermal cap: per-module maxima on their own window, plus the
            // AMS's own online mask (only trusted while 0x4A0 is fresh).
            for (std::size_t m = 0; m < PackModuleCount; ++m) {
                ci.tmax_module[m] = veh.tmax_module[m];
            }
            ci.module_online_mask = veh.module_online_mask;
            // 0x4A0's OWN tick, not the bus-level ams_fresh that ten AMS frames
            // refresh. If 0x4A0 never arrives at all the mask stays at its zero
            // initialiser -- "no module reporting" -- and trusting it would skip
            // every module and park the pack cap at its unknown value for the
            // whole run.
            ci.ams_status_fresh   = VehicleService::is_fresh(now, veh.last_ams_status_tick,
                                                             config::AmsStatusStaleMs);
            // BOTH temperature frames, not either. They carry different modules
            // (0x136 = 0-2, 0x137 = 3-4); with one shared tick, losing 0x137
            // froze modules 3-4 forever while this still read fresh.
            ci.pack_temps_fresh   = VehicleService::is_fresh(now, veh.last_tmax_a_tick,
                                                             config::AcuTmaxStaleMs) &&
                                    VehicleService::is_fresh(now, veh.last_tmax_b_tick,
                                                             config::AcuTmaxStaleMs);
        }
        // uDV / autonomous: the DV R2D request (0x510, value AND fresh)
        // and the conditioned 0x507 accel command with its own freshness --
        // stale command => the core commands torque 0 (never APPS fallback).
        ci.dv_r2d_req    = (veh.udv_r2d_request != 0u) &&
                           VehicleService::is_fresh(now, veh.last_udv_r2d_tick, config::UdvR2dStaleMs);
        ci.dv_fresh      = VehicleService::is_fresh(now, veh.last_udv_cmd_tick, config::UdvCmdStaleMs);
        ci.dv_torque_pct = VehicleService::condition_udv_torque(veh.udv_torque_cmd);
        // AS state for the emergency buzzer, on 0x50A's OWN tick. Not the
        // 0x507/0x510 freshness: a live torque stream from a uDV whose AS
        // monitor has died would mask exactly the case the fail-safe exists
        // for. The status byte is passed through raw -- as_buzzer decides what
        // it means, and it needs to distinguish "reported OFF" from "stopped
        // reporting", which it can only do with the flag alongside.
        ci.as_status     = veh.udv_as_status;
        ci.as_fresh      = VehicleService::is_fresh(now, veh.last_udv_as_tick,
                                                    config::UdvAsStaleMs);
        // 0x463 reports ELECTRICAL rpm; the envelope needs MECHANICAL.
        //
        // GATED, like every other safety input on this page. 0x463 is the only
        // signal feeding the 80 kW envelope, and last_inv_tick is stamped by
        // 0x461/0x463/0x464/0x466 alike -- so 0x463 alone can die while the
        // inverter still looks perfectly healthy. A frozen rpm then feeds the
        // envelope forever, and if it froze at 0 (the uninitialised value)
        // power_cap_pct returns 100 % and the only power limiter in the vehicle
        // is silently gone, with power_capped correctly reading 0 the whole
        // time. Stale -> assume max rpm, which caps hard instead.
        ci.motor_rpm_mech = VehicleService::is_fresh(now, veh.last_inv_s4_tick,
                                                     config::InvFeedbackStaleMs)
                            ? veh.inv_rpm / config::MotorPolePairs
                            : config::MotorRpmStaleAssumed;
        // Motor temperatures for the thermal cap, on their OWN freshness window
        // (0x464 can die while the rest of the inverter keeps talking, and a
        // frozen temperature reads as a healthy one).
        ci.inv_temp_motor1_raw = veh.inv_temp_motor1;
        ci.inv_temp_motor2_raw = veh.inv_temp_motor2;
        ci.inv_temps_fresh     = VehicleService::is_fresh(now, veh.last_inv_temps_tick,
                                                          config::InvTempsStaleMs);

        // --- step the pure controller ---
        CtrlOutput out = ctrl.step(ci, now);
        // Bring-up torque cap (config::TorqueCap; 100 = no cap). Clamps the
        // commanded torque for on-stands / freewheel testing -- MUST be 100 for flight.
        if (out.torque_pct > config::TorqueCap) {
            out.torque_pct = config::TorqueCap;
        }
        g_last_torque_pct = out.torque_pct;
        g_last_ctrl_state = static_cast<std::uint8_t>(out.state);
        g_last_apps1_raw     = in.apps1_raw;
        g_last_apps2_raw     = in.apps2_raw;
        g_last_brake_raw     = in.brake_raw;
        g_last_start_button  = in.start_button ? 1u : 0u;
        g_last_t11_8_9       = out.t11_8_9  ? 1u : 0u;

        // --- calibration session ------------------------------------
        // Entry is gated on a genuinely quiescent car: TS down, not in the
        // drive state, no torque commanded and the motor stopped. All four,
        // because a pedal calibration that changed under load could command
        // torque the driver did not ask for.
        {
            CalSessionInputs cs{};
            cs.vehicle_safe = !ci.ok_precharge &&
                              out.state != CtrlState::Active &&
                              out.torque_pct == 0u &&
                              veh.inv_rpm == 0;
            cs.apps1_raw = in.apps1_raw;
            cs.apps2_raw = in.apps2_raw;
            cs.brake_raw = in.brake_raw;
            cs.now_ms    = now;

            bool emit = false;
            CalSessionOutput cso{};
            std::uint8_t last_cmd = 0;

            if (g_cal_cmd_pending != 0u) {
                last_cmd = g_cal_cmd;
                cso = cal_session.handle(static_cast<CalCmd>(g_cal_cmd), g_cal_arg,
                                         g_cal_guard, cs, cal);
                g_cal_cmd_pending = 0u;
                emit = true;

                if (cso.commit_requested) {
                    // APPLY FIRST, then persist. The operator is standing at the
                    // car and will verify by sweeping the pedals, so the live
                    // values must be the ones they just committed. Applying
                    // before the write also means a flash failure leaves the
                    // calibration usable for this session rather than losing
                    // the operator's work outright -- they are told it did not
                    // persist and can retry.
                    cal = cso.to_commit;
                    const CalWrite w = persist_cal(cso.to_commit);
                    if (w == CalWrite::Ok) {
                        // Durable. Reflect it in the boot-status field too, so
                        // 0x704 stops advertising whatever the last BOOT found
                        // and starts telling the truth about what is stored.
                        g_cal_load_status = static_cast<std::uint8_t>(CalLoad::Loaded);
                        cso.state = CalSessionState::Committed;
                    } else {
                        cal_session.note_persist_failed();
                        cso.state  = CalSessionState::Error;
                        cso.result = CalResult::NvmWriteFailed;
                    }
                }
            } else if (cal_session.tick(cs)) {
                emit = true;                       // session timed out
                cso.state = cal_session.state();
                cso.result = CalResult::NotInSession;
            } else if (cal_session.state() != CalSessionState::Idle &&
                       static_cast<uint32_t>(now - last_cal_status) >= config::PitDiagStreamMs) {
                emit = true;                       // heartbeat while a session is open
                cso.state = cal_session.state();
                cso.captured_mask = cal_session.captured_mask();
            }

            if (emit) {
                last_cal_status = now;
                can_tx_post(PitDiag::build_cal_status(cso, last_cmd, g_cal_load_status));
                if (cso.emit_values) {
                    can_tx_post(PitDiag::build_cal_apps(cso.values));
                    can_tx_post(PitDiag::build_cal_brake(cso.values));
                }
            }
        }

        // --- ECU-held DC-link discharge -----------------------------
        // The SDC still drives the discharge relay exactly as before. PB6 only
        // interrupts its COIL, so the ECU can add a reason to discharge and can
        // never take one away.
        //
        // The AMS decides WHEN, on 0x021: its FSM in Start, TSMS on, and the
        // link still charged -- conditions only it can see. The ECU owns the
        // HOLD, and releases on its own voltage measurement rather than on the
        // request going away, so one lost frame mid-discharge cannot re-strand
        // the link.
        const bool dc_bus_valid =
            VehicleService::is_fresh(now, veh.last_vconfig_tick, config::InvDcBusStaleMs);
        DischargeInputs di{};
        // The AMS's two observations, gated on 0x021's own freshness. The ECU
        // supplies the third term (a charged link it can see) inside update().
        const bool interlock_fresh =
            VehicleService::is_fresh(now, veh.last_discharge_req_tick,
                                     config::DischargeReqStaleMs);
        di.fsm_in_start = (veh.ams_fsm_in_start != 0u) && interlock_fresh;
        di.tsms         = (veh.ams_tsms != 0u) && interlock_fresh;
        di.dc_bus_V     = veh.inv_dc_bus_V;
        di.dc_bus_valid = dc_bus_valid;
        di.now_ms       = now;
        const DischargeState dis = discharge.update(di);
        // ACTIVE-HIGH = secure = open the coil path = force the bleed on.
        // Reset state is high-Z, which the hardware must pull to PERMIT: a
        // floating pin during bootloader + init must not command a discharge
        // into a link nobody has looked at. Confirm the pull with electronics.
        HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin,
                          dis.secure ? GPIO_PIN_SET : GPIO_PIN_RESET);
        g_discharge_engaged = dis.engaged ? 1u : 0u;
        g_discharge_fault   = dis.fault ? 1u : 0u;

        // --- 0x100 heartbeat: EVERY state, every cycle (the AMS VcuStale contract) ---
        {
            VCU_heartbeat_t hb{};
            // NOT veh.inv_dc_bus_V directly. That field is a held 0x466 reading
            // with no expiry, and this frame goes out every 10 ms regardless.
            // A silent inverter therefore left us broadcasting pack voltage
            // forever, and the AMS's staleness check still saw a healthy frame.
            hb.dc_bus_voltage = VehicleService::heartbeat_dc_bus_V(veh, now);
            hb.dc_bus_valid   = dc_bus_valid ? 1u : 0u;
            hb.discharge_engaged = dis.engaged ? 1u : 0u;
            uint8_t b[VCU_heartbeat_DLC];
            encode_VCU_heartbeat(hb, b);
            CanFrame f{};
            f.bus = static_cast<uint8_t>(CanBus::Acu);
            f.id  = VCU_heartbeat_ID;
            f.dlc = VCU_heartbeat_DLC;
            for (unsigned i = 0; i < VCU_heartbeat_DLC; ++i) f.data[i] = b[i];
            can_tx_post(f);
        }

        // --- uDV autonomous contract, ACU bus, UNGATED (not pit-diag) ---
        // 0x506 motor rpm every cycle (10 ms -- the uDV SLAM/odometry input);
        // 0x504 TS-active + 0x505 brake-over-limit at 100 ms. ts_active is the
        // FSM's own ok_precharge view (stub-consistent); the brake verdict uses
        // the SAME threshold that will gate the DV R2D entry.
        can_tx_post(UdvTx::build_motor_rpm(veh.inv_rpm));
        if (static_cast<uint32_t>(now - last_udv) >= config::UdvTxPeriodMs) {
            last_udv = now;
            can_tx_post(UdvTx::build_ts_active(ci.ok_precharge));
            can_tx_post(UdvTx::build_brake_over_limit(in.brake_raw > ci.cal.brake_dv_hard));
            // 0x511 R2D confirm: repeated at 100 ms (loss-robust) with the DV
            // latch as the value -- uDV keys on byte0 != 0.
            can_tx_post(UdvTx::build_r2d_confirm(out.dv_mode));
        }

        // --- inverter setpoints (FDCAN1), EVERY cycle: PLAIN 0x360 mode + 0x362
        //     torque (bytes 0-1 = 0, no E2E -- byte-for-byte as the IFS07 VCU, which
        //     blasted these continuously regardless of HV). Off pre-R2D, Ready at
        //     WaitInvStandby, TorqueEnable in Active. ---
        const CanFrame sp_mode = Inverter::build_setpoint_mode(out.inv_mode);
        const CanFrame sp_tq   = Inverter::build_setpoint_torque(out.torque_pct);
        can_tx_post(sp_mode);
        // Fault-recovery burst: the reset word alone does not clear a latched
        // inverter fault -- the follow-up Off(0x01) does (manual 9.3, and the
        // IFS07 VCU that recovered without a power cycle sent the same
        // sequence). Non-empty ONLY while inv_state is 10/11, so the healthy
        // path still emits exactly one 0x360 per cycle.
        // Flt_Clear rides the LAST follow word only -> the bit pulses low-low-high
        // each cycle, so an edge-triggered clear sees a fresh rising edge at 100 Hz
        // instead of one stuck-high level.
        for (uint8_t i = 0; i < out.inv_mode_follow_n; ++i) {
            const bool last = (i + 1u == out.inv_mode_follow_n);
            can_tx_post(Inverter::build_setpoint_mode(out.inv_mode_follow[i],
                                                      last && out.inv_flt_clear));
        }
        can_tx_post(sp_tq);
#if defined(ECU_DEBUG_INV_BRIDGE)
        // DEBUG: mirror our OWN setpoints onto FDCAN2 (0x560/0x562). NEVER flight.
        {
            CanFrame d = sp_mode; d.bus = static_cast<uint8_t>(CanBus::Acu); d.id = 0x560u; can_tx_post(d);
            CanFrame e = sp_tq;   e.bus = static_cast<uint8_t>(CanBus::Acu); e.id = 0x562u; can_tx_post(e);
        }
#endif

        // --- outputs: RTDS + status LEDs ---
        HAL_GPIO_WritePin(RTDS_GPIO_Port, RTDS_Pin,
                          out.rtds_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin,
                          (out.state == CtrlState::AmsError) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(ERR_STATUS_GPIO_Port, ERR_STATUS_Pin,
                          (out.state == CtrlState::AmsError) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        // --- pit-diag app-state stream @100 ms (gated by 0x7E0) ---
        if (g_pit_diag_enabled &&
            static_cast<uint32_t>(now - last_pit) >= config::PitDiagStreamMs) {
            last_pit = now;
            can_tx_post(PitDiag::build_status(out, veh, in.start_button));
            can_tx_post(PitDiag::build_dv(out, ci, veh));   // 0x707 DV/autonomy
            can_tx_post(PitDiag::build_cell(ctrl.cell_derate()));  // 0x709 low-cell estimator
            can_tx_post(PitDiag::build_pack_temp(ctrl.pack_thermal()));  // 0x70A pack thermal cap
            can_tx_post(PitDiag::build_inv_foc(veh, now));               // 0x70B inverter FOC feedback
            // Same wire value we put on 0x362 this cycle, so ask/ceiling/
            // delivered can be read off one frame without correlating.
            can_tx_post(PitDiag::build_inv_torque(
                veh, Inverter::torque_to_nm_req(out.torque_pct)));       // 0x70C
            can_tx_post(PitDiag::build_power(out, veh));                 // 0x70D shaft/AC/DC
            can_tx_post(PitDiag::build_pedals(in, ci.cal));
            can_tx_post(PitDiag::build_inverter(veh, static_cast<std::uint8_t>(out.inv_mode)));
            can_tx_post(PitDiag::build_inverter_temps(veh, ctrl.motor_thermal()));
            can_tx_post(PitDiag::build_inv_faults(veh, out, now));  // 0x708 L1/L2 + cmd + 0x461 freshness
            can_tx_post(PitDiag::build_fwinfo());
            can_tx_post(PitDiag::build_brake(in, ci.cal));
#if defined(ECU_DEBUG_INV_BRIDGE)
            // DEBUG: FDCAN1 (inverter bus) TX health. 0x57F =
            // [TxErrCnt, RxErrCnt, LastErrorCode, flags(b0=busoff,b1=errpassive,b2=warn), activity].
            // LEC 3 = ACK error (no node ACKed our TX -> inverter not receiving / bus issue);
            // LEC 0 or 7 = no error (TX is ACKed -> inverter IS receiving -> rejecting on its E2E).
            {
                FDCAN_ErrorCountersTypeDef ec{};
                FDCAN_ProtocolStatusTypeDef ps{};
                HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
                HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
                CanFrame g{};
                g.bus = static_cast<uint8_t>(CanBus::Acu);
                g.id  = 0x57Fu;
                g.dlc = 5;
                g.data[0] = static_cast<uint8_t>(ec.TxErrorCnt);
                g.data[1] = static_cast<uint8_t>(ec.RxErrorCnt);
                g.data[2] = static_cast<uint8_t>(ps.LastErrorCode);
                g.data[3] = static_cast<uint8_t>((ps.BusOff ? 1u : 0u) |
                                                 (ps.ErrorPassive ? 2u : 0u) |
                                                 (ps.Warning ? 4u : 0u));
                g.data[4] = static_cast<uint8_t>(ps.Activity);
                can_tx_post(g);
            }
#endif
        }

        // --- IWDG (sole kicker) ---
        Watchdog::refresh();

        tick += config::ControlPeriodMs;
        osDelayUntil(tick);
    }
}
