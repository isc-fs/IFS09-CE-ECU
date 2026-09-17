// SPDX-License-Identifier: proprietary
//
// derate_ramp.hpp -- the falling linear ramp both thermal caps use.
//
// 100 % at or below `start`, `floor_pct` at or above `limit`, linear between.
// Shared because the motor cap and the pack cap are the same curve with
// different constants, and two hand-written copies of a safety ramp is one copy
// too many.
//
// NOT used by the low-cell derate: that one rises with voltage rather than
// falling with it, so folding it in here would mean a direction flag on a
// function whose whole value is being obvious at a glance.

#ifndef DERATE_RAMP_HPP_
#define DERATE_RAMP_HPP_

#include <cstdint>

namespace ecu {

// Precondition: start < limit (every caller static_asserts it on its own
// constants, so this is not re-checked at runtime in the control path).
[[nodiscard]] constexpr std::uint8_t falling_ramp_pct(std::int32_t x,
                                                      std::int32_t start,
                                                      std::int32_t limit,
                                                      std::uint8_t floor_pct) noexcept {
    if (x <= start) return 100u;
    if (x >= limit) return floor_pct;
    const std::int32_t span  = limit - start;
    const std::int32_t below = limit - x;
    return static_cast<std::uint8_t>(
        static_cast<std::int32_t>(floor_pct) +
        (100 - static_cast<std::int32_t>(floor_pct)) * below / span);
}

}  // namespace ecu

#endif  // DERATE_RAMP_HPP_
