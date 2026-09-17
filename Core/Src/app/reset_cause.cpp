// SPDX-License-Identifier: proprietary

#include "app/reset_cause.hpp"

#include "main.h"   // HAL + RCC

namespace ecu {

ResetCause ResetInfo::read_and_clear() noexcept {
    const std::uint32_t rsr = RCC->RSR;

    // Specific causes first: the H7 latches PINRSTF on nearly every reset, so
    // checking it last keeps the real cause (IWDG/software/...) from being
    // masked.
    ResetCause c = ResetCause::Unknown;
    if      (rsr & RCC_RSR_LPWRRSTF)                    c = ResetCause::LowPower;
    else if (rsr & RCC_RSR_WWDG1RSTF)                   c = ResetCause::Wwdg;
    else if (rsr & RCC_RSR_IWDG1RSTF)                   c = ResetCause::Iwdg;
    else if (rsr & RCC_RSR_SFTRSTF)                     c = ResetCause::Software;
    else if (rsr & RCC_RSR_PINRSTF)                     c = ResetCause::Pin;
    else if (rsr & (RCC_RSR_PORRSTF | RCC_RSR_BORRSTF)) c = ResetCause::PowerOn;

    __HAL_RCC_CLEAR_RESET_FLAGS();
    return c;
}

}  // namespace ecu
