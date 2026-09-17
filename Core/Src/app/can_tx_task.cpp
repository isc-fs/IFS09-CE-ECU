// SPDX-License-Identifier: proprietary
//
// CanTxTask: the single point where the app touches the FDCAN TX hardware. It
// drains can_tx_queue and hands each frame to HAL_FDCAN_AddMessageToTxFifoQ on
// the bus its `bus` field names. can_tx_post() is the producer side that every
// other task uses, so nothing else blocks on the TX FIFO.

#include "app/can_tx.hpp"

#include "app/app_globals.h"
#include "app/app_tasks.h"
#include "app/can_frame.hpp"

#include "cmsis_os2.h"
#include "main.h"

extern "C" {
extern FDCAN_HandleTypeDef hfdcan1;          // INV
extern FDCAN_HandleTypeDef hfdcan2;          // ACU / shared
extern FDCAN_HandleTypeDef hfdcan3;          // DASH (FDCAN3)
extern osMessageQueueId_t  can_tx_queueHandle;
}

namespace {

// dlc (0..8 bytes) -> the HAL's coded DataLength. NOT dlc<<16 (a classic bug);
// use the HAL macros.
const uint32_t kDlcCode[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
    FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
    FDCAN_DLC_BYTES_8,
};

void hal_send(const ecu::CanFrame& f) {
    FDCAN_HandleTypeDef* h = &hfdcan2;                                        // ACU (default)
    if (f.bus == static_cast<uint8_t>(ecu::CanBus::Inv))       h = &hfdcan1;  // INV
    else if (f.bus == static_cast<uint8_t>(ecu::CanBus::Dash)) h = &hfdcan3;  // DASH (FDCAN3)

    FDCAN_TxHeaderTypeDef tx = {};
    tx.Identifier          = f.id;
    tx.IdType              = f.extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = kDlcCode[f.dlc <= 8u ? f.dlc : 8u];
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    // Fire-and-forget: drop on a full TX FIFO (32-deep; the heartbeat + pit-diag
    // rates can't overflow it). A failed add is not fatal.
    HAL_FDCAN_AddMessageToTxFifoQ(h, &tx, const_cast<uint8_t*>(f.data));
}

}  // namespace

namespace ecu {

bool can_tx_post(const CanFrame& f) noexcept {
    if (can_tx_queueHandle == NULL) {
        ++g_can_tx_dropped;
        return false;
    }
    // Non-blocking (timeout 0): a full queue DROPS the frame rather than stalling
    // the poster. Count the drop so a silently-lost safety cyclic (0x100/0x504/...)
    // is visible (sticky tx_dropped bit on 0x700; exact count over SWD).
    if (osMessageQueuePut(can_tx_queueHandle, &f, 0u, 0u) == osOK) {
        return true;
    }
    ++g_can_tx_dropped;
    return false;
}

}  // namespace ecu

extern "C" void ecu_can_tx_task_run(void *argument) {
    (void)argument;
    ecu::CanFrame f;
    for (;;) {
        if (osMessageQueueGet(can_tx_queueHandle, &f, NULL, osWaitForever) == osOK) {
            ++g_task_step[ECU_TASK_CAN_TX];
            hal_send(f);
        }
    }
}
