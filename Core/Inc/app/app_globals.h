/* SPDX-License-Identifier: proprietary
 *
 * C-visible globals shared between the CubeMX freertos.c (which creates the
 * tasks + queues) and the C++ app. Kept in a C header so freertos.c can call
 * ecu_app_globals_init() from its USER CODE.
 */

#ifndef ECU_APP_GLOBALS_H
#define ECU_APP_GLOBALS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Task indices for the 0x704 task_ran_mask liveness bitmap. */
enum {
    ECU_TASK_CONTROL = 0,
    ECU_TASK_CAN_RX  = 1,
    ECU_TASK_CAN_TX  = 2,
    ECU_TASK_TELEMETRY = 3,
    ECU_TASK_DIAG    = 4,
    ECU_TASK_COUNT   = 5,
};

/* Free-running per-task step counters: each task bumps its own every loop.
 * DiagTask snapshots them once per health frame and reports, per task, whether
 * the counter advanced (bit set = the task ran). A cyclic task whose bit stays
 * clear has stalled -- the exact #48-class symptom we need visible over CAN. */
extern volatile uint32_t g_task_step[ECU_TASK_COUNT];

/* Pit-diag app-state stream (0x700-0x705) gate: set by CanRxTask on a 0x7E0
 * enable command (magic 0xDEADBEEF), read by ControlTask. The 0x704 health
 * frame is deliberately ALWAYS-ON (ungated) -- it's the diagnostic lifeline. */
extern volatile uint8_t g_pit_diag_enabled;
extern uint32_t g_last_torque_pct;
extern uint8_t  g_last_ctrl_state;

/* Pedals/plausibility, mirrored from ControlTask's realtime-local IoInputs +
 * CtrlOutput so TelemetryTask (200 ms, non-realtime) can publish them on
 * FDCAN3 (0x511, 0x510 flags) without pulling the pedal path off its inline
 * read-once-per-10ms-cycle design (see io_signals.hpp). Display-only: the
 * safety decision is made in ControlTask itself, this is just a mirror. */
extern uint16_t g_last_apps1_raw;
extern uint16_t g_last_apps2_raw;
extern uint16_t g_last_brake_raw;
extern uint8_t  g_last_start_button;
extern uint8_t  g_last_t11_8_9;
// #198 DC-link discharge, mirrored for DiagTask/telemetry. engaged is the
// COMMANDED state (no auxiliary-contact readback is wired); fault means the
// hold timed out without the link falling.
extern uint8_t  g_discharge_engaged;
extern uint8_t  g_discharge_fault;

/* Count of frames dropped by can_tx_post() because the TX queue was full (its
 * osMessageQueuePut timed out at 0). Free-running, never reset. 0 on a healthy
 * bus. Surfaced as the sticky `tx_dropped` bit on 0x700 -- if it is ever nonzero
 * a safety-cyclic (0x100/0x504/...) may have been silently dropped. Also readable
 * over SWD for the exact count. */
extern volatile uint32_t g_can_tx_dropped;
// Reboot triggers REFUSED because the car was in the drive ladder. Surfaced on
// the ungated 0x704 so "my flash did nothing" is answerable from the bus: a
// refusal that is invisible is indistinguishable from a dead node.
extern volatile uint32_t g_boot_trigger_refused;

/* Outcome of loading the pedal calibration from NVM at boot (#169).
 * status is ecu::CalLoad; flags is the cal_flag:: bitmask when the stored
 * record was found but rejected. Surfaced on the diagnostic stream so a silent
 * fallback to compile-time defaults is visible to the operator -- a
 * calibration that was quietly ignored is indistinguishable from one that
 * applied, and that is exactly the confusion this must prevent. */
/* One-deep mailbox for an inbound 0x7E2 calibration command (#169).
 * CanRxTask decodes and posts; ControlTask consumes and clears. Single
 * producer, single consumer, and commands are human-paced, so a deeper queue
 * would be ceremony -- a command arriving while one is still pending is
 * dropped and the client simply does not see a reply, which its POLL recovers.
 * ControlTask owns the session because it is the task that holds the live
 * calibration and the live pedal readings a CAPTURE has to sample. */
extern volatile uint8_t  g_cal_cmd_pending;
extern volatile uint8_t  g_cal_cmd;
extern volatile uint8_t  g_cal_arg;
extern volatile uint32_t g_cal_guard;

extern volatile uint8_t  g_cal_load_status;
extern volatile uint8_t  g_cal_load_flags;

/* Called once from freertos.c USER CODE after osKernelInitialize(), before the
 * scheduler starts. Currently just zeroes the step counters; a hook point for
 * any app-side RTOS object the .ioc can't carry. */
void ecu_app_globals_init(void);

/* C-linkage app shim (defined in error_latch.cpp) so the CubeMX freertos.c
 * stack-overflow / malloc-failed hooks can latch a fault sentinel before they
 * spin into the watchdog reset. code = FaultCode (0xF5 stack-overflow, 0xF6
 * malloc-failed). */
void ecu_fault_latch_set_c(uint8_t code);

#ifdef __cplusplus
}
#endif

#endif /* ECU_APP_GLOBALS_H */
