// SPDX-License-Identifier: proprietary
//
// cal_session.hpp -- the operator calibration session state machine.
//
// PURE. No HAL, no CAN, no flash: it takes a decoded command plus a snapshot of
// the live pedal readings and returns what to do. The task layer decodes 0x7E2
// into it and encodes its output onto 0x7E3/0x7E4/0x7E5, and performs the
// persistence it asks for. That split is what lets the SIL suite drive every
// branch -- including the ones that only occur when an operator does something
// wrong, which are the ones that matter.
//
// APPLY-ON-COMMIT. A valid commit takes effect on the live calibration
// immediately, not at the next reset. That is safe by construction here: a
// session can only be open while the vehicle is quiescent (see CalSessionInputs
// ::vehicle_safe), so no calibration ever changes under a moving car. It also
// makes the read-back verification meaningful -- the operator sweeps the pedals
// and sees the calibration they just committed, rather than the old one plus a
// note to power-cycle, which people skip.

#ifndef CAL_SESSION_HPP_
#define CAL_SESSION_HPP_

#include <cstdint>

#include "app/pedal_cal.hpp"

namespace ecu {

enum class CalCmd : std::uint8_t {
    Poll = 0, Enter = 1, Capture = 2, ReadStored = 3,
    ReadStaged = 4, Commit = 5, Abort = 6, ResetDefaults = 7,
};

enum class CalPoint : std::uint8_t {
    AppsRest = 1, AppsFull = 2, BrakeRest = 3, BrakePressed = 4, AppsMid = 5,
};

enum class CalSessionState : std::uint8_t {
    Idle = 0, Active = 1, Validated = 2, Committing = 3, Committed = 4, Error = 5,
};

enum class CalResult : std::uint8_t {
    Ok = 0, BadGuard = 1, NotInSession = 2, VehicleNotSafe = 3,
    SampleUnstable = 4, MissingPoints = 5, ValidationFailed = 6,
    NvmWriteFailed = 7, UnknownCmd = 8,
};

// captured_mask bits, in CalPoint order (bit = point - 1).
namespace cal_point_bit {
inline constexpr std::uint8_t AppsRest     = 1u << 0;
inline constexpr std::uint8_t AppsFull     = 1u << 1;
inline constexpr std::uint8_t BrakeRest    = 1u << 2;
inline constexpr std::uint8_t BrakePressed = 1u << 3;
inline constexpr std::uint8_t AppsMid      = 1u << 4;
inline constexpr std::uint8_t All          = 0x1Fu;
}  // namespace cal_point_bit

// Opening a session needs this constant, so a stray frame on a bus shared with
// the AMS and the uDV cannot start one. COMMIT does NOT use it -- it carries a
// CRC of the staged set instead, which is both the guard and the consistency
// check.
inline constexpr std::uint32_t CalGuardMagic = 0xCA11B0DEu;

// A session with no traffic for this long is abandoned: an operator who
// unplugs mid-way must not leave the ECU holding a half-captured calibration.
inline constexpr std::uint32_t CalSessionTimeoutMs = 30000;

// How far the two APPS channels may disagree at the mid-travel point. This is
// the ONLY check that actually verifies T.11.8.9 tracking -- the endpoints
// cannot, because both channels normalise to 0..100 % by construction. Held
// tighter than AppsDisagreePct so a calibration cannot be committed that sits
// right on the edge of tripping the runtime cut.
inline constexpr std::uint8_t CalMidTravelTolerancePct = 5;

struct CalSessionInputs {
    bool          vehicle_safe = false;  // TS down, not driving, no torque, no rpm
    std::uint16_t apps1_raw = 0;
    std::uint16_t apps2_raw = 0;
    std::uint16_t brake_raw = 0;
    std::uint32_t now_ms = 0;
};

struct CalSessionOutput {
    CalSessionState state  = CalSessionState::Idle;
    CalResult       result = CalResult::Ok;
    std::uint8_t    captured_mask = 0;
    std::uint8_t    validation_flags = 0;
    bool            emit_values = false;        // send 0x7E4 + 0x7E5
    PedalCal        values{};                   // what those frames carry
    // Set on a valid COMMIT: the task must apply this to the live calibration
    // AND persist it. Persistence failure is reported back via note_persist_failed.
    bool            commit_requested = false;
    PedalCal        to_commit{};
};

class CalSession {
public:
    // Handle one decoded 0x7E2. `active` is the calibration currently in force.
    CalSessionOutput handle(CalCmd cmd, std::uint8_t arg, std::uint32_t guard,
                            const CalSessionInputs& in, const PedalCal& active) noexcept;

    // Call every control cycle. Closes an abandoned session; otherwise inert.
    // Returns true if the session just timed out (the caller may want to emit).
    bool tick(const CalSessionInputs& in) noexcept;

    // The task calls this when the persistence it was asked for did not stick.
    // The calibration is already live for this power cycle; the operator has to
    // know it will not survive a reset.
    void note_persist_failed() noexcept;

    CalSessionState state() const noexcept { return state_; }
    std::uint8_t    captured_mask() const noexcept { return mask_; }

private:
    void reset_() noexcept;
    std::uint8_t validate_staged_() const noexcept;

    CalSessionState state_ = CalSessionState::Idle;
    PedalCal        staged_{};
    std::uint8_t    mask_ = 0;
    std::uint32_t   last_cmd_ms_ = 0;
    CalResult       last_result_ = CalResult::Ok;
    // mid-travel samples, kept out of PedalCal because they are evidence for a
    // validation rule rather than part of the calibration itself
    std::uint16_t   mid_apps1_ = 0;
    std::uint16_t   mid_apps2_ = 0;
};

}  // namespace ecu

#endif  // CAL_SESSION_HPP_
