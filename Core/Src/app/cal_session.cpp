// SPDX-License-Identifier: proprietary
//
// The calibration session state machine. Pure logic -- see the header.

#include "app/cal_session.hpp"

#include "app/control.hpp"        // apps_pct -- the same mapping the FSM uses
#include "app/pedal_cal_nvm.hpp"   // cal_crc32 -- the canonical serialisation the CRC covers

namespace ecu {

namespace {

std::uint8_t abs_diff(std::uint8_t a, std::uint8_t b) noexcept {
    return (a > b) ? static_cast<std::uint8_t>(a - b) : static_cast<std::uint8_t>(b - a);
}

}  // namespace

void CalSession::reset_() noexcept {
    state_ = CalSessionState::Idle;
    staged_ = PedalCal{};
    mask_ = 0;
    mid_apps1_ = 0;
    mid_apps2_ = 0;
}

std::uint8_t CalSession::validate_staged_() const noexcept {
    std::uint8_t flags = validate_cal(staged_);

    // The mid-travel agreement check. validate_cal() cannot do this -- it only
    // sees endpoints, and both channels normalise to 0..100 % by construction,
    // so they always agree there however badly they diverge in between. This is
    // the only place the T.11.8.9 failure mode is actually measured.
    if ((mask_ & cal_point_bit::AppsMid) != 0u) {
        const std::uint8_t p1 = apps_pct(mid_apps1_, staged_.apps1_min, staged_.apps1_max);
        const std::uint8_t p2 = apps_pct(mid_apps2_, staged_.apps2_min, staged_.apps2_max);
        if (abs_diff(p1, p2) > CalMidTravelTolerancePct) {
            flags |= cal_flag::AppsSpanMismatch;
        }
    }
    return flags;
}

bool CalSession::tick(const CalSessionInputs& in) noexcept {
    if (state_ == CalSessionState::Idle) return false;
    if (static_cast<std::uint32_t>(in.now_ms - last_cmd_ms_) < CalSessionTimeoutMs) return false;
    // Abandoned. Discard everything staged -- a half-captured calibration must
    // never be recoverable by a client that reconnects and blindly commits.
    reset_();
    last_result_ = CalResult::NotInSession;
    return true;
}

void CalSession::note_persist_failed() noexcept {
    state_ = CalSessionState::Error;
    last_result_ = CalResult::NvmWriteFailed;
}

CalSessionOutput CalSession::handle(CalCmd cmd, std::uint8_t arg, std::uint32_t guard,
                                    const CalSessionInputs& in,
                                    const PedalCal& active) noexcept {
    last_cmd_ms_ = in.now_ms;

    CalSessionOutput out{};
    out.result = CalResult::Ok;

    const bool in_session = (state_ != CalSessionState::Idle);

    switch (cmd) {
    case CalCmd::Poll:
        break;   // report current state, change nothing

    case CalCmd::ReadStored:
        // Deliberately allowed with no session: reading what is in force is not
        // a privileged operation, and the pit tool shows it before the operator
        // decides to calibrate at all.
        out.emit_values = true;
        out.values = active;
        break;

    case CalCmd::Enter:
        if (guard != CalGuardMagic) { out.result = CalResult::BadGuard; break; }
        if (!in.vehicle_safe)       { out.result = CalResult::VehicleNotSafe; break; }
        reset_();
        // Start from what is currently in force, not from compile-time
        // defaults: an operator recalibrating only the brake must not silently
        // revert someone else's APPS work.
        staged_ = active;
        state_  = CalSessionState::Active;
        break;

    case CalCmd::Capture: {
        if (!in_session)      { out.result = CalResult::NotInSession; break; }
        if (!in.vehicle_safe) { out.result = CalResult::VehicleNotSafe; reset_(); break; }
        switch (static_cast<CalPoint>(arg)) {
        case CalPoint::AppsRest:
            staged_.apps1_min = in.apps1_raw; staged_.apps2_min = in.apps2_raw;
            mask_ |= cal_point_bit::AppsRest; break;
        case CalPoint::AppsFull:
            staged_.apps1_max = in.apps1_raw; staged_.apps2_max = in.apps2_raw;
            mask_ |= cal_point_bit::AppsFull; break;
        case CalPoint::AppsMid:
            mid_apps1_ = in.apps1_raw; mid_apps2_ = in.apps2_raw;
            mask_ |= cal_point_bit::AppsMid; break;
        case CalPoint::BrakeRest:
            staged_.brake_rest = in.brake_raw;
            mask_ |= cal_point_bit::BrakeRest; break;
        case CalPoint::BrakePressed:
            staged_.brake_pressed = in.brake_raw;
            mask_ |= cal_point_bit::BrakePressed; break;
        default:
            out.result = CalResult::UnknownCmd; break;
        }
        // Re-derive the two intermediate thresholds from the measured span
        // whenever it moves, instead of leaving inherited IFS06 placeholders
        // sitting inside a freshly measured range. arm at 10 % of travel (just
        // clear of the released-pedal noise floor), dv_hard at 60 % (the EBS
        // holds well past that). Both stay operator-visible on 0x7E5 before any
        // commit, and the ordering rule in validate_cal() still gates them.
        if ((mask_ & cal_point_bit::BrakeRest) && (mask_ & cal_point_bit::BrakePressed) &&
            staged_.brake_pressed > staged_.brake_rest) {
            const std::uint32_t span = static_cast<std::uint32_t>(staged_.brake_pressed - staged_.brake_rest);
            staged_.brake_arm     = static_cast<std::uint16_t>(staged_.brake_rest + span * 10u / 100u);
            staged_.brake_dv_hard = static_cast<std::uint16_t>(staged_.brake_rest + span * 60u / 100u);
        }
        break;
    }

    case CalCmd::ReadStaged:
        if (!in_session) { out.result = CalResult::NotInSession; break; }
        out.emit_values = true;
        out.values = staged_;
        break;

    case CalCmd::Commit: {
        if (!in_session)      { out.result = CalResult::NotInSession; break; }
        if (!in.vehicle_safe) { out.result = CalResult::VehicleNotSafe; reset_(); break; }
        if ((mask_ & cal_point_bit::All) != cal_point_bit::All) {
            out.result = CalResult::MissingPoints; break;
        }
        // The guard field carries a CRC-32 of the staged set on COMMIT. A
        // client that has drifted out of sync cannot commit values it does not
        // hold. Computed by the caller (it owns the serialisation) and passed
        // through, so this stays free of encoding concerns.
        if (guard != cal_crc32(staged_)) {
            out.result = CalResult::ValidationFailed;
            break;
        }
        const std::uint8_t flags = validate_staged_();
        if (flags != 0u) {
            out.result = CalResult::ValidationFailed;
            out.validation_flags = flags;
            state_ = CalSessionState::Error;
            break;
        }
        state_ = CalSessionState::Committing;
        out.commit_requested = true;
        out.to_commit = staged_;
        break;
    }

    case CalCmd::Abort:
        reset_();
        break;

    case CalCmd::ResetDefaults:
        if (guard != CalGuardMagic) { out.result = CalResult::BadGuard; break; }
        if (!in.vehicle_safe)       { out.result = CalResult::VehicleNotSafe; break; }
        // Stage the compile-time defaults; the operator still has to COMMIT.
        // Restoring is destructive enough that it should not also be silent.
        staged_ = PedalCal{};
        mask_   = cal_point_bit::All;   // nothing left to capture
        state_  = CalSessionState::Active;
        break;

    default:
        out.result = CalResult::UnknownCmd;
        break;
    }

    last_result_ = out.result;
    out.state = state_;
    out.captured_mask = mask_;
    return out;
}

}  // namespace ecu
