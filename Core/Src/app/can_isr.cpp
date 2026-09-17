// SPDX-License-Identifier: proprietary
//
// Single strong override of the HAL's weak HAL_FDCAN_RxFifo0Callback. Fires for
// BOTH FDCAN1 (INV) and FDCAN2 (ACU); tags the frame with its bus and pushes it
// onto can_rx_queue for CanRxTask to dispatch. Non-blocking (ISR context): a
// full queue drops the frame and bumps a counter.

#include "app/app_globals.h"
#include "app/can_frame.h"

#include "cmsis_os2.h"
#include "main.h"

extern "C" {

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern osMessageQueueId_t  can_rx_queueHandle;

// ISR-side drop counter (queue full / pre-scheduler). Surfaced on a diag frame.
volatile uint32_t g_can_rx_isr_drop = 0;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) {
        return;
    }
    if (can_rx_queueHandle == NULL) {
        return;  // pre-scheduler / shutdown -- nowhere to land the frame.
    }

    uint8_t bus;
    if      (hfdcan == &hfdcan1) bus = 0u;  // CanBus::Inv
    else if (hfdcan == &hfdcan2) bus = 1u;  // CanBus::Acu
    else                         return;

    FDCAN_RxHeaderTypeDef rxh;
    uint8_t rxbuf[8] = {0};

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, rxbuf) != HAL_OK) {
            break;
        }

        CanFrame frame   = {0};
        frame.id         = rxh.Identifier;
        // DataLength is the extracted DLC code (0..15); for classic <=8 frames
        // the low nibble is the byte count. (& 0x0F is version-robust.)
        frame.dlc        = (uint8_t)(rxh.DataLength & 0x0F);
        frame.extended   = (rxh.IdType == FDCAN_EXTENDED_ID) ? 1u : 0u;
        frame.fd         = (rxh.FDFormat == FDCAN_FD_CAN) ? 1u : 0u;
        frame.bus        = bus;
        frame.timestamp_ms = osKernelGetTickCount();

        for (uint8_t i = 0; i < 8u && i < frame.dlc; ++i) {
            frame.data[i] = rxbuf[i];
        }

        if (osMessageQueuePut(can_rx_queueHandle, &frame, 0u, 0u) != osOK) {
            ++g_can_rx_isr_drop;
        }
    }
}

}  // extern "C"
