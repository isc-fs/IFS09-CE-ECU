// SPDX-License-Identifier: proprietary
//
// The one part of calibration persistence that touches hardware: programming a
// single flash word into the bootloader's NVM sector.
//
// Deliberately thin, and deliberately NOT in the SIL target. Everything that
// can be got wrong -- where the entry goes, what sequence number it needs, what
// the 32 bytes contain -- is pure and lives in pedal_cal_nvm.cpp where the SIL
// suite pins it against the bootloader's own rules. What is left here is an
// unlock, one HAL call, a lock and a read-back.
//
// APPEND ONLY. We never erase and never compact; see pedal_cal_nvm.hpp for why
// (watchdog budget, corruption risk, single-bank stall). A full region is
// reported as CalWrite::Full and left for a bootloader-side compaction.

#include "app/pedal_cal_flash.hpp"

#include "app/pedal_cal_nvm.hpp"
#include "app/watchdog.hpp"

#include "main.h"   // HAL, HAL_FLASH_*

namespace ecu {

CalWrite persist_cal(const PedalCal& c) noexcept {
    std::uint32_t offset = 0;
    std::uint32_t seq    = 0;
    if (!find_cal_write_slot(reinterpret_cast<const void*>(CalNvmBase), CalNvmSize,
                             &offset, &seq)) {
        return CalWrite::Full;
    }

    // HAL_FLASH_Program takes the ADDRESS of the source for FLASHWORD writes and
    // moves 256 bits at once, so the buffer must be a full entry and aligned.
    alignas(32) std::uint8_t entry[CalNvmEntrySize];
    build_cal_entry(c, seq, entry);

    const std::uint32_t addr = CalNvmBase + offset;

    // Kick the dog on both sides. One flash word is ~100 us against a 500 ms
    // IWDG budget, so this is not load-bearing -- it is here so that if anyone
    // ever widens this function into something slower, the failure is a missed
    // refresh they have to think about rather than a reset on the car.
    Watchdog::refresh();

    if (HAL_FLASH_Unlock() != HAL_OK) return CalWrite::HardwareFail;
    const HAL_StatusTypeDef st =
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr,
                          reinterpret_cast<std::uintptr_t>(entry));
    HAL_FLASH_Lock();

    Watchdog::refresh();

    if (st != HAL_OK) {
        // The bootloader falls back to compaction here (a slot with stray bits
        // is permanently un-programmable). We cannot: compaction erases the
        // whole sector. find_cal_write_slot already refuses a dirty slot, so
        // reaching this means a genuine flash fault -- report it and stop.
        return CalWrite::HardwareFail;
    }

    // Read back through the memory-mapped view. Flash that programmed without
    // an error but did not take is the failure mode worth catching: the
    // operator would otherwise be told the calibration is saved.
    const auto* written = reinterpret_cast<const std::uint8_t*>(addr);
    for (std::uint32_t i = 0; i < CalNvmEntrySize; ++i) {
        if (written[i] != entry[i]) return CalWrite::VerifyFail;
    }
    return CalWrite::Ok;
}

}  // namespace ecu
