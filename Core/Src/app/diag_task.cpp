// SPDX-License-Identifier: proprietary
//
// DiagTask: emits the 0x704 firmware-health frame once a second. Runs at low
// priority and -- crucially -- SEPARATELY from ControlTask, so it survives a
// ControlTask stall (the exact class of failure it exists to diagnose). The
// 0x704 frame is always-on (ungated) so reset_cause + task liveness are visible
// over CAN immediately on boot, with no UART/SWD.

#include "app/app_tasks.h"

#include "app/app_globals.h"
#include "app/can_tx.hpp"
#include "app/ecu_config.hpp"
#include "app/error_latch.hpp"
#include "app/pit_diag.hpp"
#include "app/reset_cause.hpp"

#include "cmsis_os2.h"
#include "main.h"

#include "FreeRTOS.h"   // xPortGetFreeHeapSize

using namespace ecu;

extern "C" void ecu_diag_task_run(void *argument) {
    (void)argument;

    // Captured once: the cause of THIS boot, and the fault latched in backup RAM
    // before the previous reset (0 if a clean boot). Both reported every frame.
    const ResetCause boot_cause = ResetInfo::read_and_clear();
    const FaultCode  last_fault = FaultLatch::get();
    const uint32_t   boot_tick  = osKernelGetTickCount();

    uint32_t min_heap            = 0xFFFFFFFFu;
    uint32_t prev_step[ECU_TASK_COUNT] = {};
    uint32_t tick                = osKernelGetTickCount();

    for (;;) {
        ++g_task_step[ECU_TASK_DIAG];

        const uint32_t heap = static_cast<uint32_t>(xPortGetFreeHeapSize());
        if (heap < min_heap) min_heap = heap;

        // Per-task liveness: bit set = the task's step counter advanced since the
        // last health frame. A cyclic task whose bit stays clear has stalled.
        uint8_t mask = 0;
        for (int i = 0; i < ECU_TASK_COUNT; ++i) {
            if (g_task_step[i] != prev_step[i]) mask |= static_cast<uint8_t>(1u << i);
            prev_step[i] = g_task_step[i];
        }

        HealthMetrics m{};
        m.free_heap     = static_cast<uint16_t>(heap     > 0xFFFFu ? 0xFFFFu : heap);
        m.min_free_heap = static_cast<uint16_t>(min_heap > 0xFFFFu ? 0xFFFFu : min_heap);
        m.task_ran_mask = mask;
        m.reset_cause   = boot_cause;
        m.uptime_s      = static_cast<uint8_t>(((osKernelGetTickCount() - boot_tick) / 1000u) & 0xFFu);
        m.last_fault    = last_fault;
        can_tx_post(PitDiag::build_health(m));

        tick += config::DiagPeriodMs;
        osDelayUntil(tick);
    }
}
