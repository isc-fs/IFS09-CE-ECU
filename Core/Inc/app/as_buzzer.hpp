// SPDX-License-Identifier: proprietary
//
// as_buzzer.hpp -- the AS Emergency acoustic signal on the RTDS buzzer.
//
// WHAT THE RULE WANTS. FSG requires AS Emergency to be indicated acoustically:
// an intermittent sound, 50 % duty in the 1-5 Hz band, for 10 s, at the same
// time as the ASSI blue flash. The DV stack has no buzzer of its own, so it
// borrows the RTDS one.
//
// WHY THE WAVEFORM LIVES HERE AND THE TRIGGER DOES NOT. The uDV owns the
// trigger: only its AS state machine knows it is in Emergency, and it publishes
// that on 0x50A. The ECU owns the waveform. If the uDV drove each edge over
// CAN, a bus hiccup would silence a safety signal mid-tone; generating it here
// means one frame starts a tone that then runs on our own clock.
//
// EDGE-TRIGGERED, NOT LEVEL. AS Emergency latches on the uDV until the ASMS is
// switched off, which is routinely longer than 10 s. The rule caps the SOUND at
// 10 s while the light keeps flashing. So the tone starts on the rising edge of
// EMERGENCY, runs its 10 s, and stops -- even though the uDV is still reporting
// EMERGENCY the whole time. Driving it from the level would sound until the
// marshals arrived.
//
// An emergency present on the very first frame counts as an edge. Arriving
// already in Emergency is an emergency; there is no earlier state to rise from,
// and staying silent because we missed the transition would be the wrong call.
//
// THE STALE FAIL-SAFE. A uDV that goes quiet mid-mission is itself an
// emergency, and it cannot tell us so -- that is what "quiet" means. So losing
// 0x50A while the last thing we saw was DRIVING or READY sounds the tone too.
//
// It is gated on having seen those states for a reason: a manual car with no
// uDV fitted never sends 0x50A at all, and must not buzz for it. The fail-safe
// fires once per silence, not once per tick, and re-arms when frames return.
//
// Pure: no HAL, no clock of its own, the caller passes now_ms. The SIL drives
// it directly, which is the only way to test a ten-second tone in under a
// second.

#ifndef AS_BUZZER_HPP_
#define AS_BUZZER_HPP_

#include <cstdint>

#include "app/ecu_config.hpp"

namespace ecu {

struct AsBuzzerInputs {
    std::uint8_t  as_status = 0;      // 0x50A byte 0; meaningful only when fresh
    bool          as_fresh  = false;  // 0x50A seen within config::UdvAsStaleMs
    std::uint32_t now_ms    = 0;
};

struct AsBuzzerState {
    // Drive the buzzer THIS tick. Already toggled -- the caller writes it
    // straight to the pin and does no timing of its own.
    bool sounding = false;
    // The 10 s window is running. Published on the pit-diag stream so a bench
    // can tell "the tone fired and finished" from "the tone never fired".
    bool active   = false;
    // The running tone was started by silence rather than by an EMERGENCY
    // frame. Worth separating: it means the uDV is gone, not just unhappy.
    bool from_stale = false;
};

class AsBuzzer {
public:
    AsBuzzerState update(const AsBuzzerInputs& in) noexcept;

private:
    void trigger_(std::uint32_t now_ms, bool stale) noexcept;

    std::uint8_t  last_status_   = 0;
    bool          seen_          = false;  // a fresh frame has arrived at least once
    bool          active_        = false;
    bool          from_stale_    = false;
    bool          stale_fired_   = false;  // one tone per silence, not one per tick
    std::uint32_t started_at_ms_ = 0;
};

}  // namespace ecu

#endif  // AS_BUZZER_HPP_
