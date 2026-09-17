// SPDX-License-Identifier: proprietary
//
// Thin wrapper over the IWDG. The dog (IWDG1, ~500 ms, started pre-scheduler by
// CubeMX's MX_IWDG1_Init) is refreshed by exactly one owner -- ControlTask, the
// realtime task -- on both its clean and fault paths, so a latched ECU stays
// alive long enough to emit diagnostics instead of reset-looping. If ControlTask
// itself wedges, the dog is NOT refreshed -> hardware reset -> the BL catches it.

#pragma once

namespace ecu {

class Watchdog {
public:
    static void refresh() noexcept;   // HAL_IWDG_Refresh(&hiwdg1)
};

// C-linkage convenience for any C caller.
extern "C" void ecu_watchdog_refresh_c(void);

}  // namespace ecu
