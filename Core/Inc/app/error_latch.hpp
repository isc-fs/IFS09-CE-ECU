// SPDX-License-Identifier: proprietary
//
// Sticky last-fault sentinel in an RTC backup register (BKP1R). The fault /
// stack-overflow / malloc-failed hooks stamp a code here just before they spin
// into the watchdog reset; DiagTask reads it after the reset for the 0x704
// health frame's `last_fault`, so an otherwise-invisible crash names itself
// over CAN (the car has no UART/SWD). Survives warm resets (sw/IWDG/pin) but
// not a power-cycle (no VBAT on this carrier) -- accepted, same as the AMS.
//
// A magic tag in the high 16 bits distinguishes a real latched fault from
// cold-boot garbage in the backup register.

#pragma once

#include <cstdint>

namespace ecu {

enum class FaultCode : std::uint8_t {
    None          = 0x00,
    HardFault     = 0xF1,
    MemManage     = 0xF2,
    BusFault      = 0xF3,
    UsageFault    = 0xF4,
    StackOverflow = 0xF5,
    MallocFailed  = 0xF6,
    AssertFailed  = 0xF7,
};

class FaultLatch {
public:
    static constexpr std::uint32_t Tag = 0xFA170000u;  // "valid fault" tag

    // Pure encode/decode (host-testable, no HAL).
    static constexpr std::uint32_t encode(FaultCode c) noexcept {
        return Tag | static_cast<std::uint32_t>(c);
    }
    static constexpr FaultCode decode(std::uint32_t raw) noexcept {
        return ((raw & 0xFFFF0000u) == Tag)
                   ? static_cast<FaultCode>(raw & 0xFFu)
                   : FaultCode::None;            // cold-boot garbage / cleared
    }

    static void                init() noexcept;          // one-shot backup-domain unlock
    static void                set(FaultCode code) noexcept;
    [[nodiscard]] static FaultCode get() noexcept;       // for 0x704 last_fault
    static void                clear() noexcept;          // clean boot / ECU_HIL_CLEAR_ERROR_LATCH
};

// C-linkage for the CubeMX freertos.c fault/overflow/malloc hooks (no C++ there).
extern "C" void ecu_fault_latch_set_c(std::uint8_t code);

}  // namespace ecu
