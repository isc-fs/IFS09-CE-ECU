// SPDX-License-Identifier: proprietary
//
// AS Emergency buzzer waveform. See the header for why the ECU owns the tone
// and the uDV owns only the trigger.

#include "app/as_buzzer.hpp"

namespace ecu {

using namespace config;

void AsBuzzer::trigger_(std::uint32_t now_ms, bool stale) noexcept {
    // Restarts the window on a fresh edge even if a tone is already running:
    // a second emergency is a second emergency.
    active_        = true;
    from_stale_    = stale;
    started_at_ms_ = now_ms;
}

AsBuzzerState AsBuzzer::update(const AsBuzzerInputs& in) noexcept {
    AsBuzzerState st{};

    if (in.as_fresh) {
        // Frames are back, so re-arm the silence fail-safe for next time.
        stale_fired_ = false;

        const bool emergency     = (in.as_status == AsStatusEmergency);
        const bool was_emergency = seen_ && (last_status_ == AsStatusEmergency);

        // Rising edge. `seen_` being false counts as an edge, so a uDV that is
        // already in Emergency when we first hear from it still sounds.
        if (emergency && !was_emergency) trigger_(in.now_ms, false);

        last_status_ = in.as_status;
        seen_        = true;
    } else {
        // Silence mid-mission is itself an emergency. Gated on having actually
        // seen the uDV driving: a manual car never sends this frame and must
        // not buzz for its absence.
        const bool mid_mission = seen_ && (last_status_ == AsStatusDriving ||
                                           last_status_ == AsStatusReady);
        if (mid_mission && !stale_fired_) {
            trigger_(in.now_ms, true);
            stale_fired_ = true;
        }
    }

    if (active_) {
        // Unsigned subtraction, so the 32-bit tick wrap costs nothing.
        const std::uint32_t elapsed = in.now_ms - started_at_ms_;
        if (elapsed >= AsEmergencySoundMs) {
            active_ = false;
        } else {
            // 150 ms on, 150 ms off -> 3.3 Hz at 50 % duty, matching the ASSI
            // blue flash. Same rate, not phase-locked: two MCUs, two clocks.
            st.sounding = ((elapsed / AsBuzzerHalfPeriodMs) % 2u) == 0u;
        }
    }

    st.active     = active_;
    st.from_stale = active_ && from_stale_;
    return st;
}

}  // namespace ecu
