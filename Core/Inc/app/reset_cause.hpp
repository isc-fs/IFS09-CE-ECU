// SPDX-License-Identifier: proprietary
//
// Classifies the cause of the last reset from the RCC reset-status register,
// read + cleared once early in boot. Feeds the 0x704 health frame's
// `reset_cause` so an IWDG reset (the TX-dead class of failure mode) is visible over
// CAN. Values match the pit-diag enum (0 unknown ... 4 IWDG ...).

#pragma once

#include <cstdint>

namespace ecu {

enum class ResetCause : std::uint8_t {
    Unknown  = 0,
    PowerOn  = 1,   // POR / BOR
    Pin      = 2,   // NRST
    Software = 3,   // NVIC_SystemReset
    Iwdg     = 4,
    Wwdg     = 5,
    LowPower = 6,
};

class ResetInfo {
public:
    // Read RCC->RSR, classify (specific causes before PIN -- the H7 sets
    // PINRSTF on almost every reset), then clear the flags. Call once at boot.
    static ResetCause read_and_clear() noexcept;
};

}  // namespace ecu
