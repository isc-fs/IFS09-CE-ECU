// SPDX-License-Identifier: proprietary
//
// The pure ECU control core. See control.hpp for the contract. No HAL/RTOS
// includes here -- this translation unit is compiled as-is into the host
// unit-test build.

#include "app/control.hpp"

namespace ecu {

using namespace config;

uint8_t apps_pct(uint16_t raw, uint16_t adc_min, uint16_t adc_max) noexcept {
    if (adc_max <= adc_min) return 0;          // mis-calibration guard
    if (raw <= adc_min) return 0;
    if (raw >= adc_max) return 100;
    return static_cast<uint8_t>(
        (static_cast<uint32_t>(raw - adc_min) * 100u) / (adc_max - adc_min));
}

void Controller::enter_(CtrlState s, uint32_t now_ms) noexcept {
    state_ = s;
    state_entry_ms_ = now_ms;
    // The DV latch lives for one drive cycle: any exit from the drive ladder
    // (back to a pre-R2D state, or the AmsError inhibit) clears it. The next
    // R2D entry re-decides the mode from whichever trigger fires.
    if (s < CtrlState::R2dDelay || s == CtrlState::AmsError) {
        dv_latched_ = false;
    }
}

CtrlOutput Controller::step(const CtrlInputs& in, uint32_t now_ms) noexcept {
    // ---- pedal % + torque + plausibility (computed every tick, even before
    //      Active, so the latches track and pit-diag shows live verdicts) ----
    const uint8_t a1 = apps_pct(in.apps1_raw, in.cal.apps1_min, in.cal.apps1_max);
    const uint8_t a2 = apps_pct(in.apps2_raw, in.cal.apps2_min, in.cal.apps2_max);

    uint8_t torque = 0;
    if (a1 > AppsAgreementPct && a2 > AppsAgreementPct) {
        torque = static_cast<uint8_t>((static_cast<uint16_t>(a1) + a2) / 2u);
    }
    if (torque < DeadbandLowPct) torque = 0;
    else if (torque > DeadbandHighPct) torque = 100;

    // The EV.2.3 brake+throttle plausibility cut USED TO BE HERE. It was deleted
    // in FS-Rules 2024 and is gone from the firmware with it.
    //
    // It was a latching torque cut: brake above cal.brake_pressed with demand
    // over 25 % zeroed torque until the driver fully lifted. Two reasons not to
    // keep it as an optional safety net once the rule stopped requiring it --
    // it tripped on brake_pressed, which is still COMMISSION-tagged and has
    // never been measured, and being a latch, a spurious trip took drive away
    // mid-corner until a full lift. An unnecessary cut on an unverified
    // threshold is a hazard of its own.
    //
    // T.11.8.9 below is UNAFFECTED -- APPS-disagreement is a different rule and
    // is still required.

    // T.11.8.9 APPS disagreement, honouring the 100 ms persistence window.
    const int diff = static_cast<int>(a1) - static_cast<int>(a2);
    const bool disagree = (diff < 0 ? -diff : diff) > static_cast<int>(AppsDisagreePct);
    if (disagree) {
        if (!apps_disagree_active_) {
            apps_disagree_active_ = true;
            apps_disagree_since_ms_ = now_ms;
        }
    } else {
        apps_disagree_active_ = false;
    }
    const bool t11 = apps_disagree_active_ &&
                     static_cast<uint32_t>(now_ms - apps_disagree_since_ms_) >= AppsDisagreePersistMs;

    if (t11) torque = 0;

    // DV torque source: when the DV drive is latched the pedals are NOT
    // the torque source -- the conditioned uDV 0x507 command is, and a stale
    // command stream means torque 0, NEVER a fall-back to APPS (no driver is
    // seated). T.11.8.9 is a driver-pedal rule and does not gate the DV
    // command. Its latch keeps computing above (pedals idle; the pit-diag
    // verdict stays live) but the zeroing applies to the pedal torque only.
    // The cell-voltage derate below still applies -- pack protection is
    // mode-independent.
    if (dv_latched_) {
        torque = in.dv_fresh ? in.dv_torque_pct : 0;
    }

    // Low-cell-voltage derate (applied -- see note above). Runs every tick,
    // including before Active, so the filter is already settled on the real
    // pack voltage by the time torque is first commanded rather than converging
    // through the first second of the run.
    //
    // The input is an ESTIMATED open-circuit voltage, not the loaded reading:
    // sag under acceleration is ohmic and transient, and derating on it makes
    // the derate a function of throttle instead of state of charge. See
    // cell_derate.hpp.
    CellDerateInputs cdi{};
    cdi.v_cell_min_mV = in.v_cell_min_mV;
    cdi.v_fresh       = in.ams_fresh;
    cdi.current_dA    = in.current_accu_dA;
    cdi.i_fresh       = in.current_fresh;
    cell_derate_ = cell_.update(cdi);

    // All four limits are CAPS, not gains: everything under the ceiling passes
    // through untouched and only the top is clipped. Order is therefore
    // immaterial. See cell_derate.hpp for why capping beats scaling.
    if (torque > cell_derate_.cap_pct) torque = cell_derate_.cap_pct;

    // Motor thermal cap. Runs every tick so the filter is settled before
    // torque is ever commanded. Losing the sensors does NOT mean no limit; see
    // motor_thermal.hpp.
    MotorThermalInputs mti{};
    mti.temp_motor1_raw = in.inv_temp_motor1_raw;
    mti.temp_motor2_raw = in.inv_temp_motor2_raw;
    mti.fresh           = in.inv_temps_fresh;
    motor_thermal_ = thermal_.update(mti);
    if (torque > motor_thermal_.cap_pct) torque = motor_thermal_.cap_pct;

    // Accumulator thermal cap. Same shape as the motor cap; the pack has
    // minutes of thermal mass rather than a lap's, so once this engages it stays
    // engaged for the session. That is correct, and it is why it annunciates.
    PackThermalInputs pti{};
    for (std::size_t i = 0; i < PackModuleCount; ++i) pti.tmax_module[i] = in.tmax_module[i];
    pti.module_online_mask = in.module_online_mask;
    pti.mask_valid         = in.ams_status_fresh;
    pti.temps_fresh        = in.pack_temps_fresh;
    pack_thermal_ = pack_.update(pti);
    if (torque > pack_thermal_.cap_pct) torque = pack_thermal_.cap_pct;

    // EV 2.2.1 tractive-power envelope. LAST, so nothing downstream can
    // put torque back above it, and feed-forward from measured speed so it
    // closes no loop. Before this, nothing in the vehicle enforced the 80 kW
    // limit and the map commanded roughly twice it over most of the speed
    // range. Inert below ~2721 mech rpm, so normal cornering is untouched.
    //
    // Order among the four limiters is immaterial: cell, motor, pack and power
    // are all min() caps, so the lowest wins whatever sequence they run in. That
    // was NOT true while the cell derate multiplied -- a gain had to come first
    // or it would have scaled the caps themselves.
    bool power_capped = false;
    {
        const uint8_t cap = power_cap_pct(in.motor_rpm_mech);
        if (torque > cap) { torque = cap; power_capped = true; }
    }

    // ---- FSM: decide transitions FIRST, then derive outputs from the
    //      resulting state, so the emitted output always matches the state we
    //      report (a Moore machine -- no one-tick lag on entry actions). ----

    // AMS latched Error overrides every state: inhibit, do not retry precharge.
    if (in.ams_error && state_ != CtrlState::AmsError) enter_(CtrlState::AmsError, now_ms);

    switch (state_) {
    case CtrlState::WaitInvVdcConfig:
        if (in.inv_vconfig_ready) enter_(CtrlState::Precharge, now_ms);
        break;
    case CtrlState::Precharge:
        // Gate on the AMS verdict (0x020) ONLY -- the AMS owns precharge now
        // (0x600 retired). On timeout, restart the wait window (retry).
        if (in.ok_precharge) {
            enter_(CtrlState::WaitStartBrake, now_ms);
        } else if (static_cast<uint32_t>(now_ms - state_entry_ms_) >= PrechargeTimeoutMs) {
            enter_(CtrlState::Precharge, now_ms);
        }
        break;
    case CtrlState::WaitStartBrake:
        // The trigger IS the mode decision: whichever gate fires latches
        // the mode for this drive cycle. Manual = seated driver (START + brake
        // past the arm threshold). DV = the uDV R2D request (0x510, fresh)
        // WHILE the EBS holds hard braking, verified on our own brake sensor
        // (brake_raw > BrakeDvHardRaw) -- no start button in DV. The two are
        // physically exclusive (driver seated vs ASMS on / AS mission running).
        if (in.start_button && in.brake_raw > in.cal.brake_arm) {
            enter_(CtrlState::R2dDelay, now_ms);
        } else if (in.dv_r2d_req && in.brake_raw > in.cal.brake_dv_hard) {
            enter_(CtrlState::R2dDelay, now_ms);
            dv_latched_ = true;   // after enter_ (which clears it for pre-R2D targets)
        }
        break;
    case CtrlState::R2dDelay:
        if (static_cast<uint32_t>(now_ms - state_entry_ms_) >= R2dSoundMs) {
            enter_(CtrlState::WaitInvStandby, now_ms);
        }
        break;
    case CtrlState::WaitInvStandby:
        if (in.inv_state == InvReadyState) enter_(CtrlState::Active, now_ms);
        break;
    case CtrlState::Active:
        // AMS opened the contactors (ok_precharge fell) -> re-arm.
        if (!in.ok_precharge) {
            enter_(CtrlState::Precharge, now_ms);
        } else if (in.inv_state != InvReadyState &&
                   in.inv_state != InvTorqueEnableState) {
            // THE INVERTER LEFT THE DRIVE WHILE THE TS STAYED UP.
            //
            // Observed with an overspeed: the inverter trips, parks itself in
            // Standby(3) and never comes back. Standby is NOT a fault state, so
            // the reactive block below does not fire; and Standby < SoftFault,
            // so the output switch happily kept commanding TorqueEnable(0x06)
            // at 100 Hz. But this A16 config climbs Standby -> Ready(0x04) ->
            // TorqueEnable; it does not jump straight to TorqueEnable. The one
            // state that knows how to run that climb is WaitInvStandby, which we
            // had already left -- so the ECU sat in Active shouting a word the
            // inverter would not act on, forever. TS never dropped, so the
            // ok_precharge exit above never fired either, and R2D is only
            // reachable from WaitStartBrake: the driver could not re-arm at all
            // without a full LV power cycle.
            //
            // Testing membership of the DRIVE states rather than for Standby
            // specifically, because the same trap is waiting in Shutdown(13) and
            // in whatever state a cleared fault lands in. "Is it faulted?" and
            // "is it still in the drive?" are different questions and only the
            // first one was being asked.
            //
            // Faults (10/11) route here too, which is an improvement: the
            // reactive block runs in ANY drive state, so the recovery burst
            // still goes out, and afterwards the climb is driven from the state
            // that owns it instead of from Active where nothing did.
            //
            // NO RTDS on the way back: R2dDelay is skipped, so the buzzer does
            // not re-sound. That is deliberate -- the RTDS marks the driver's
            // R2D, not an inverter hiccup.
            //
            // Drive resumes as soon as the inverter reaches Ready again.
            // Team decision: recovery speed over a re-arm gate. The
            // consequence to know about is that if the driver still has the
            // throttle down when the inverter recovers, torque returns with no
            // driver action. If that ever proves too abrupt, the fix is a latch
            // holding cmd_torque at 0 until apps falls below the deadband --
            // NOT reinstating a full R2D, which would make every lifted-wheel
            // overspeed a stop-and-rearm.
            enter_(CtrlState::WaitInvStandby, now_ms);
            ++inv_redrive_count_;   // wraps; visible on 0x708
        }
        break;
    case CtrlState::AmsError:
        if (!in.ams_error) enter_(CtrlState::WaitInvVdcConfig, now_ms);
        break;
    }

    InvMode mode = InvMode::Off;
    // Follow-up mode words for the fault burst below (see CtrlOutput).
    InvMode follow[2] = { InvMode::Off, InvMode::Off };
    uint8_t follow_n = 0;
    bool    flt_clear = false;
    bool    rtds = false;
    bool    drive = false;
    uint8_t cmd_torque = 0;

    switch (state_) {
    case CtrlState::WaitInvVdcConfig:
    case CtrlState::Precharge:
    case CtrlState::WaitStartBrake:
    case CtrlState::AmsError:
        mode = InvMode::Off;
        break;
    case CtrlState::R2dDelay:
        mode = InvMode::Off;
        rtds = true;                        // drive the RTDS buzzer
        break;
    case CtrlState::WaitInvStandby:
        // Climb to Ready. From Standby(3) that is a direct Ready(0x04); from
        // Off(0)/Shutdown(13) it is NOT -- this A16 config will not take Ready
        // from those states.
        //
        // BENCH EVIDENCE (on stands at 355 V):
        // parked in WaitInvStandby with inv_state=13 Shutdown, commanding
        // Ready(0x04) at 100 Hz, L1/L2 fault layers CLEAN (PwrStg 0x001 alive,
        // EMCtrl 0x01 init_ok) and the DEM only latched history -- and the
        // inverter never moved. Nothing was blocking it; it simply does not
        // accept Ready from Shutdown.
        //
        // NOT the same as sending Off INSTEAD of Ready. That variant was tried
        // and reverted: without a following Ready the inverter cannot climb at
        // all. The IFS07 VCU -- the only configuration known to
        // have recovered without a power cycle -- sent BOTH in one pass, via the
        // fall-through in its App_State switch (pre-jarama main.c:1788-1811):
        //     case 13 -> 0x01                (Shutdown: Off only)
        //     case 0  -> 0x01 then 0x04      (Off: Off THEN Ready, same pass)
        // That is what is reproduced here, using the same follow-word mechanism
        // as the fault burst. Ready is ALWAYS still sent for state 0; the Off
        // merely precedes it.
        //
        // Note the manual's 9.1 diagram shows OFF --(READY)--> READY as one
        // direct transition -- but its App_State_Req enum (1..5) does not match
        // this A16 config at all, so trust the bench for numbering and the
        // diagram for topology only.
        //
        // A latched fault (10/11) is still overridden to its reset word by the
        // reactive block below. Reaching Ready(4) advances to Active (above).
        if (in.inv_state == InvShutdownState) {
            mode = InvMode::Off;            // 0x01 -- legacy case 13
        } else if (in.inv_state == InvOffState) {
            mode      = InvMode::Off;       // 0x01 -- legacy case 0 ...
            follow[0] = InvMode::Ready;     // 0x04 -- ... falling through to case 3
            follow_n  = 1;
        } else {
            mode = InvMode::Ready;          // 0x04 -- standby(3) -> ready
        }
        break;
    case CtrlState::Active:
        // Healthy inverter -> drive. A faulted inverter (>= soft fault) gets its
        // recovery mode + no torque from the reactive block after this switch.
        if (in.inv_state < InvSoftFaultState) {
            mode = InvMode::TorqueEnable;
            drive = true;
            cmd_torque = torque;
        }
        break;
    }

    // Inverter fault recovery -- reactive, in ANY drive state (not just Active).
    // A faulted inverter ignores Off/Ready; it clears only when commanded its
    // recovery mode word. Mirrors the legacy VCU's per-state inverter switch
    // (pre-jarama main.c:1912/1956): soft fault (10) -> Fault (0x13), hard fault
    // (11) -> HardFaultReset (0x0D). The inverter can boot LATCHED in hard fault
    // before we ever reach Active, so this must run pre-Active or the FSM stalls
    // at WaitInvStandby forever waiting for a ready state that never comes. Torque
    // is already 0 outside the Active healthy path, so this never drives a fault.
    // Not applied in AmsError (that state inhibits -- Off is the safe command).
    //
    // The reset word is followed, IN THE SAME CYCLE, by Off(0x01) -- see
    // CtrlOutput::inv_mode_follow. Sending only the reset word leaves the fault
    // standing: manual 9.3 says going to OFF is what restarts a FAULT, and the
    // IFS07 VCU's fall-through switch always ended on 0x01. A bench
    // capture caught exactly this -- inverter latched SoftFault(10) with the DC
    // bus at 355 V while the ECU commanded 0x13 forever and it never cleared.
    if (state_ != CtrlState::AmsError) {
        if (in.inv_state == InvHardFaultState) {
            mode      = InvMode::HardFaultReset;   // 0x0D
            follow[0] = InvMode::Off;              // 0x01 -- the documented clear
            follow_n  = 1;
            flt_clear = true;                      // + Flt_Clear on that Off
        } else if (in.inv_state == InvSoftFaultState) {
            mode      = InvMode::Fault;            // 0x13
            follow[0] = InvMode::HardFaultReset;   // 0x0D
            follow[1] = InvMode::Off;              // 0x01 -- the documented clear
            follow_n  = 2;
            flt_clear = true;                      // + Flt_Clear on that Off
        }
    }

    CtrlOutput out{};
    out.state       = state_;
    out.torque_pct  = cmd_torque;
    out.inv_mode    = mode;
    out.inv_mode_follow[0]  = follow[0];
    out.inv_mode_follow[1]  = follow[1];
    out.inv_mode_follow_n   = follow_n;
    out.inv_flt_clear       = flt_clear;
    out.power_capped   = power_capped;
    out.thermal_capped      = motor_thermal_.capped;
    out.pack_thermal_capped = pack_thermal_.capped;
    out.inv_redrive_count   = inv_redrive_count_;
    out.torque_nm    = torque_pct_to_nm(cmd_torque);
    // AS Emergency tone. Runs in EVERY state, not just the DV ones: the uDV can
    // latch Emergency from its watchdog path with the car anywhere in the
    // ladder, and a car sitting in Precharge with a dead autonomous system
    // still has to make the noise.
    const AsBuzzerState as = as_buzzer_.update({in.as_status, in.as_fresh, now_ms});
    // OR, with the emergency winning. An emergency arriving during an R2D chirp
    // must not be swallowed by it, and the two are audibly different anyway --
    // 2 s continuous versus 10 s of 150 ms pulses.
    out.rtds_on               = rtds || as.sounding;
    out.as_buzzer_active      = as.active;
    out.as_buzzer_from_stale  = as.from_stale;
    out.ok_to_drive = drive;
    out.t11_8_9     = t11;
    out.dv_mode     = dv_latched_;
    return out;
}

}  // namespace ecu
