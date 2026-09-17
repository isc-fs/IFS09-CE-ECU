/**
 * hal_impl.c - Mock HAL/FDCAN para build SIL
 *
 * Implementa un "CAN fisico" simulado por bus:
 * - RX FIFO por handle FDCAN para alimentar Can_ISR_PushRxFifo0()
 * - TX FIFO por handle FDCAN para observar lo que emite CanTx_SendHal()
 */

#include "main.h"
#include "sil_hal_mocks.h"
#include <stdio.h>
#include <string.h>

#define SIL_FDCAN_QUEUE_CAPACITY 64u

typedef struct {
    FDCAN_RxHeaderTypeDef rx_header;
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t data[8];
} sil_fdcan_frame_t;

typedef struct {
    sil_fdcan_frame_t items[SIL_FDCAN_QUEUE_CAPACITY];
    uint32_t head;
    uint32_t tail;
    uint32_t count;
} sil_fdcan_queue_t;

typedef struct {
    sil_fdcan_queue_t rx;
    sil_fdcan_queue_t tx;
} sil_fdcan_bus_t;

/* -------------------------------------------------------------------------
   Handles FDCAN globales (extern en can.c, definidos aqui en SIL)
   ---------------------------------------------------------------------- */
FDCAN_HandleTypeDef hfdcan1 = { .Instance = 0x40006400UL };
FDCAN_HandleTypeDef hfdcan2 = { .Instance = 0x40006800UL };
FDCAN_HandleTypeDef hfdcan3 = { .Instance = 0x40006C00UL };

static sil_fdcan_bus_t s_bus_inv;
static sil_fdcan_bus_t s_bus_acu;
static sil_fdcan_bus_t s_bus_dash;

static sil_fdcan_bus_t *bus_from_handle(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1) return &s_bus_inv;
    if (hfdcan == &hfdcan2) return &s_bus_acu;
    if (hfdcan == &hfdcan3) return &s_bus_dash;
    return NULL;
}

static uint32_t dlc_to_hal(uint8_t dlc)
{
    switch (dlc) {
        case 0u: return FDCAN_DLC_BYTES_0;
        case 1u: return FDCAN_DLC_BYTES_1;
        case 2u: return FDCAN_DLC_BYTES_2;
        case 3u: return FDCAN_DLC_BYTES_3;
        case 4u: return FDCAN_DLC_BYTES_4;
        case 5u: return FDCAN_DLC_BYTES_5;
        case 6u: return FDCAN_DLC_BYTES_6;
        case 7u: return FDCAN_DLC_BYTES_7;
        default: return FDCAN_DLC_BYTES_8;
    }
}

static void queue_reset(sil_fdcan_queue_t *q)
{
    if (!q) return;
    memset(q, 0, sizeof(*q));
}

static HAL_StatusTypeDef queue_push_rx(sil_fdcan_queue_t *q,
                                       const FDCAN_RxHeaderTypeDef *header,
                                       const uint8_t *data)
{
    sil_fdcan_frame_t *dst;

    if (!q || !header || q->count >= SIL_FDCAN_QUEUE_CAPACITY) return HAL_ERROR;

    dst = &q->items[q->tail];
    memset(dst, 0, sizeof(*dst));
    dst->rx_header = *header;
    if (data) memcpy(dst->data, data, 8);

    q->tail = (q->tail + 1u) % SIL_FDCAN_QUEUE_CAPACITY;
    q->count++;
    return HAL_OK;
}

static HAL_StatusTypeDef queue_pop_rx(sil_fdcan_queue_t *q,
                                      FDCAN_RxHeaderTypeDef *header,
                                      uint8_t *data)
{
    sil_fdcan_frame_t *src;

    if (!q || q->count == 0u) return HAL_ERROR;

    src = &q->items[q->head];
    if (header) *header = src->rx_header;
    if (data) memcpy(data, src->data, 8);

    q->head = (q->head + 1u) % SIL_FDCAN_QUEUE_CAPACITY;
    q->count--;
    return HAL_OK;
}

static HAL_StatusTypeDef queue_push_tx(sil_fdcan_queue_t *q,
                                       const FDCAN_TxHeaderTypeDef *header,
                                       const uint8_t *data)
{
    sil_fdcan_frame_t *dst;

    if (!q || !header || q->count >= SIL_FDCAN_QUEUE_CAPACITY) return HAL_ERROR;

    dst = &q->items[q->tail];
    memset(dst, 0, sizeof(*dst));
    dst->tx_header = *header;
    if (data) memcpy(dst->data, data, 8);

    q->tail = (q->tail + 1u) % SIL_FDCAN_QUEUE_CAPACITY;
    q->count++;
    return HAL_OK;
}

static HAL_StatusTypeDef queue_pop_tx(sil_fdcan_queue_t *q,
                                      FDCAN_TxHeaderTypeDef *header,
                                      uint8_t *data)
{
    sil_fdcan_frame_t *src;

    if (!q || q->count == 0u) return HAL_ERROR;

    src = &q->items[q->head];
    if (header) *header = src->tx_header;
    if (data) memcpy(data, src->data, 8);

    q->head = (q->head + 1u) % SIL_FDCAN_QUEUE_CAPACITY;
    q->count--;
    return HAL_OK;
}

void SIL_FDCAN_Reset(void)
{
    queue_reset(&s_bus_inv.rx);
    queue_reset(&s_bus_inv.tx);
    queue_reset(&s_bus_acu.rx);
    queue_reset(&s_bus_acu.tx);
    queue_reset(&s_bus_dash.rx);
    queue_reset(&s_bus_dash.tx);
}

HAL_StatusTypeDef SIL_FDCAN_InjectRxFrame(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t id,
                                          uint32_t id_type,
                                          const uint8_t *data,
                                          uint8_t dlc)
{
    sil_fdcan_bus_t *bus = bus_from_handle(hfdcan);
    FDCAN_RxHeaderTypeDef rxh;
    uint8_t payload[8] = {0};

    if (!bus) return HAL_ERROR;

    memset(&rxh, 0, sizeof(rxh));
    rxh.Identifier = id;
    rxh.IdType = id_type;
    rxh.RxFrameType = FDCAN_DATA_FRAME;
    rxh.DataLength = dlc_to_hal(dlc);
    rxh.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    rxh.BitRateSwitch = FDCAN_BRS_OFF;
    rxh.FDFormat = FDCAN_CLASSIC_CAN;

    if (data && dlc) memcpy(payload, data, dlc > 8u ? 8u : dlc);
    return queue_push_rx(&bus->rx, &rxh, payload);
}

HAL_StatusTypeDef SIL_FDCAN_PopTxFrame(FDCAN_HandleTypeDef *hfdcan,
                                       FDCAN_TxHeaderTypeDef *pTxHeader,
                                       uint8_t *pTxData)
{
    sil_fdcan_bus_t *bus = bus_from_handle(hfdcan);
    if (!bus) return HAL_ERROR;
    return queue_pop_tx(&bus->tx, pTxHeader, pTxData);
}

uint32_t SIL_FDCAN_GetTxCount(FDCAN_HandleTypeDef *hfdcan)
{
    sil_fdcan_bus_t *bus = bus_from_handle(hfdcan);
    return bus ? bus->tx.count : 0u;
}

/* -------------------------------------------------------------------------
   Stubs HAL FDCAN conectados a las colas mock
   ---------------------------------------------------------------------- */
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
                                                FDCAN_TxHeaderTypeDef *pTxHeader,
                                                uint8_t *pTxData)
{
    sil_fdcan_bus_t *bus = bus_from_handle(hfdcan);
    uint8_t payload[8] = {0};

    if (!bus || !pTxHeader) return HAL_ERROR;
    if (pTxData) memcpy(payload, pTxData, 8);

    return queue_push_tx(&bus->tx, pTxHeader, payload);
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan,
                                         uint32_t RxLocation,
                                         FDCAN_RxHeaderTypeDef *pRxHeader,
                                         uint8_t *pRxData)
{
    sil_fdcan_bus_t *bus = bus_from_handle(hfdcan);
    (void)RxLocation;

    if (!bus) return HAL_ERROR;
    return queue_pop_rx(&bus->rx, pRxHeader, pRxData);
}

/* -------------------------------------------------------------------------
   Error handler  (en STM32 entra en loop infinito; en SIL solo imprime)
   ---------------------------------------------------------------------- */
void Error_Handler(void)
{
    fprintf(stderr, "[SIL][ERROR] Error_Handler() llamado\n");
}
