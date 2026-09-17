// SPDX-License-Identifier: proprietary
//
// App_InitTask: runs once at high priority after the scheduler starts, brings
// up the two FDCAN instances, then self-deletes. The bring-up is the heart of
// the FDCAN bring-up fix:
//   1. The non-overlapping MessageRAM offsets (FDCAN1=0 / FDCAN2=387 words) are
//      set at the single MX_FDCAN1/2_Init in fdcan.c -- FDCAN2 no longer shares
//      SRAMCAN with FDCAN1 (the overlap was what corrupted TX). NOT re-applied
//      here: a DeInit/Init dance gates the SHARED FDCAN kernel clock and faults.
//   2. Each bus is started INDEPENDENTLY and success is gated on FDCAN2 alone
//      -- a dead/unpeered FDCAN1 (the inverter bus) must never take down the
//      heartbeat on FDCAN2 (the shared bus the AMS watches). We deliberately do
//      NOT Error_Handler() on a bus start failure: ControlTask keeps running +
//      kicking the IWDG, and the result is visible over CAN (g_fdcan*_result).
//
// Pre-scheduler init (IWDG, GPIO) stays in main.c's USER CODE.

#include "app/app_tasks.h"

#include "app/ecu_config.hpp"
#include "app/error_latch.hpp"

#include "cmsis_os2.h"
#include "main.h"

#include <cstdint>

extern "C" {
extern FDCAN_HandleTypeDef hfdcan1;   // INV  (FDCAN1)
extern FDCAN_HandleTypeDef hfdcan2;   // ACU  (FDCAN2, shared + flash)
extern FDCAN_HandleTypeDef hfdcan3;   // DASH (FDCAN3, TX-only)
}

// Bring-up milestones + per-instance results, surfaced on the diag stream so an
// engineer on the bus sees how far init got even if a bus never starts.
//   progress: 1 latch-init, 2 latch-clear(HIL), 3 FDCAN2 re-init, 4 FDCAN2 up,
//             5 FDCAN1 up, 6 done
extern "C" {
volatile std::uint8_t  g_app_init_progress    = 0;
volatile std::uint32_t g_fdcan1_start_result  = 0xFFFFFFFFu;
volatile std::uint32_t g_fdcan2_start_result  = 0xFFFFFFFFu;
}

namespace {

// Standard-accept filter (both buses are 11-bit), RX FIFO0 notification, start.
HAL_StatusTypeDef bring_up(FDCAN_HandleTypeDef* h) noexcept {
    HAL_FDCAN_ConfigGlobalFilter(h,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,  // unmatched std -> FIFO0
                                 FDCAN_REJECT,              // reject all extended
                                 FDCAN_REJECT_REMOTE,
                                 FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    return HAL_FDCAN_Start(h);
}

}  // namespace

extern "C" void ecu_app_init_task_run(void *argument) {
    (void)argument;

    ecu::FaultLatch::init();
    g_app_init_progress = 1u;

#if defined(ECU_HIL_CLEAR_ERROR_LATCH)
    // Bench-only: wipe a stale fault latched in a previous iteration so the chip
    // doesn't come up trapped (the bench has no out-of-band clear). NEVER in a
    // flight image -- a real latched fault must survive a reboot.
    ecu::FaultLatch::clear();
    g_app_init_progress = 2u;
#endif

    // FDCAN MessageRAM offsets (FDCAN1=0 / FDCAN2=387 words) are applied at the
    // single MX_FDCAN_Init in fdcan.c -- we do NOT DeInit/Init here (that gates
    // the shared FDCAN kernel clock and faults).
    g_app_init_progress = 3u;

    // Resilient bring-up. FDCAN2 first (the load-bearing bus); FDCAN1 is
    // best-effort. Neither failure Error_Handler()s the ECU.
    g_fdcan2_start_result = static_cast<std::uint32_t>(bring_up(&hfdcan2));
    g_app_init_progress   = 4u;
    g_fdcan1_start_result = static_cast<std::uint32_t>(bring_up(&hfdcan1));
    g_app_init_progress   = 5u;

    // FDCAN3 (Dash) is TX-only -- a plain Start, no RX filter/notification.
    HAL_FDCAN_Start(&hfdcan3);

    g_app_init_progress = 6u;
    osThreadExit();   // single-shot: free the TCB + stack
}
