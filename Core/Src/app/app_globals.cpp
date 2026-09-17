// SPDX-License-Identifier: proprietary

#include "app/app_globals.h"

extern "C" {
volatile uint32_t g_task_step[ECU_TASK_COUNT] = {};
volatile uint8_t  g_pit_diag_enabled = 0;
volatile uint32_t g_can_tx_dropped = 0u;
volatile uint8_t  g_cal_cmd_pending = 0u;
volatile uint8_t  g_cal_cmd = 0u;
volatile uint8_t  g_cal_arg = 0u;
volatile uint32_t g_cal_guard = 0u;
volatile uint8_t  g_cal_load_status = 0u;   /* ecu::CalLoad::Defaults */
volatile uint8_t  g_cal_load_flags  = 0u;
uint32_t g_last_torque_pct = 0;
uint8_t  g_last_ctrl_state = 0;
volatile uint32_t g_boot_trigger_refused = 0;
uint16_t g_last_apps1_raw = 0;
uint16_t g_last_apps2_raw = 0;
uint16_t g_last_brake_raw = 0;
uint8_t  g_last_start_button = 0;
uint8_t  g_last_t11_8_9 = 0;
uint8_t  g_discharge_engaged = 0;
uint8_t  g_discharge_fault   = 0;
}

extern "C" void ecu_app_globals_init(void) {
    for (int i = 0; i < ECU_TASK_COUNT; ++i) {
        g_task_step[i] = 0u;
    }
}
