/* SPDX-License-Identifier: proprietary
 *
 * extern "C" trampoline declarations for the FreeRTOS task entry points. Each
 * CubeMX-generated StartXxxTask (in freertos.c) calls the matching ecu_*_task_run
 * from its USER CODE block; the bodies live in the per-task Core/Src/app/ files.
 * This header is the bridge freertos.c (C) includes to reach the C++ app.
 */

#ifndef ECU_APP_TASKS_H
#define ECU_APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

void ecu_app_init_task_run(void *argument);   /* one-shot: peripheral bring-up */
void ecu_control_task_run(void *argument);    /* realtime 10 ms: the safety actor */
void ecu_can_rx_task_run(void *argument);     /* drain can_rx_queue -> services */
void ecu_can_tx_task_run(void *argument);     /* drain can_tx_queue -> HAL TX */
void ecu_telemetry_task_run(void *argument);  /* 200 ms: FDCAN3 + nRF24 */
void ecu_diag_task_run(void *argument);       /* 1 Hz: 0x704 health */
void ecu_gps_task_run(void *argument);        /* 20 ms: NMEA drain, 0x508/0x509 */

#ifdef __cplusplus
}
#endif

#endif /* ECU_APP_TASKS_H */
