// SPDX-License-Identifier: proprietary
//
// pedal_cal_flash.hpp -- the hardware half of calibration persistence.
// Separated from pedal_cal_nvm so the pure logic stays in the SIL build and
// only the unavoidable HAL call sits outside it.

#ifndef PEDAL_CAL_FLASH_HPP_
#define PEDAL_CAL_FLASH_HPP_

#include "app/pedal_cal.hpp"
#include "app/pedal_cal_nvm.hpp"   // CalWrite

namespace ecu {

// Append the calibration to the bootloader NVM sector as one flash word.
//
// Blocks for roughly 100 us (single 256-bit program on a single-bank H733, so
// instruction fetch stalls too). That is 0.02 % of the 500 ms IWDG budget and
// 1 % of one ControlTask cycle, which is why this can be called straight from
// ControlTask instead of needing a worker task. It must stay that cheap: never
// add an erase or a compaction here.
[[nodiscard]] CalWrite persist_cal(const PedalCal& c) noexcept;

}  // namespace ecu

#endif  // PEDAL_CAL_FLASH_HPP_
