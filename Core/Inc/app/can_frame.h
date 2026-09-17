/* SPDX-License-Identifier: proprietary
 *
 * C-visible POD shape for the CAN frame carried through the FreeRTOS message
 * queues (can_rx_queue, can_tx_queue). freertos.c / main.c (CubeMX-generated,
 * C) need to see sizeof(CanFrame) when calling osMessageQueueNew(); the C++
 * side (can_frame.hpp) wraps this same layout in the ecu:: namespace.
 *
 * Keep this in sync with Core/Inc/app/can_frame.hpp.
 */

#ifndef ECU_CAN_FRAME_H
#define ECU_CAN_FRAME_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ECU_CAN_FRAME_MAX_DATA 8u

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  extended;       /* bool: 1 if 29-bit id (the ECU contract is all 11-bit) */
    uint8_t  fd;             /* reserved, always 0 (classic CAN) */
    uint8_t  bus;            /* CanBus: 0 = INV (FDCAN1), 1 = ACU (FDCAN2), 2 = DASH (FDCAN3) */
    uint32_t timestamp_ms;
    uint8_t  data[ECU_CAN_FRAME_MAX_DATA];
} CanFrame;

#ifndef __cplusplus
_Static_assert(sizeof(CanFrame) <= 32, "CanFrame must fit a 32-byte queue slot");
#else
static_assert(sizeof(CanFrame) <= 32, "CanFrame must fit a 32-byte queue slot");
#endif

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* ECU_CAN_FRAME_H */
