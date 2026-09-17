// SPDX-License-Identifier: proprietary

#include "app/watchdog.hpp"

#include "iwdg.h"   // hiwdg1
#include "main.h"   // HAL

namespace ecu {

void Watchdog::refresh() noexcept {
    HAL_IWDG_Refresh(&hiwdg1);
}

extern "C" void ecu_watchdog_refresh_c(void) {
    HAL_IWDG_Refresh(&hiwdg1);
}

}  // namespace ecu
