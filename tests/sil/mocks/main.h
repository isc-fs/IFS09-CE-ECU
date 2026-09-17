/**
 * main.h  –  Mock mínimo para build SIL (host PC)
 *
 * Este fichero sobreescribe Core/Inc/main.h en el build SIL porque el
 * directorio mocks/ va PRIMERO en el include path. Proporciona solo los
 * tipos HAL/FDCAN que necesitan can.c y otros módulos de aplicación.
 *
 * NO incluye ninguna cabecera STM32 ni CMSIS de hardware.
 */

#ifndef __MAIN_SIL_H
#define __MAIN_SIL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
   HAL base
   ---------------------------------------------------------------------- */
typedef enum {
    HAL_OK      = 0x00U,
    HAL_ERROR   = 0x01U,
    HAL_BUSY    = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

/* -------------------------------------------------------------------------
   FDCAN – constantes de tipo/frame
   ---------------------------------------------------------------------- */
#define FDCAN_STANDARD_ID       0x00000000U
#define FDCAN_EXTENDED_ID       0x00000001U

#define FDCAN_DATA_FRAME        0x00000000U
#define FDCAN_REMOTE_FRAME      0x00000001U

/* DLC → (dlc_bytes << 16), igual que en la HAL real del STM32H7 */
#define FDCAN_DLC_BYTES_0       0x00000000U
#define FDCAN_DLC_BYTES_1       0x00010000U
#define FDCAN_DLC_BYTES_2       0x00020000U
#define FDCAN_DLC_BYTES_3       0x00030000U
#define FDCAN_DLC_BYTES_4       0x00040000U
#define FDCAN_DLC_BYTES_5       0x00050000U
#define FDCAN_DLC_BYTES_6       0x00060000U
#define FDCAN_DLC_BYTES_7       0x00070000U
#define FDCAN_DLC_BYTES_8       0x00080000U

#define FDCAN_ESI_ACTIVE        0x00000000U
#define FDCAN_ESI_PASSIVE       0x00000001U

#define FDCAN_BRS_OFF           0x00000000U
#define FDCAN_BRS_ON            0x00000001U

#define FDCAN_CLASSIC_CAN       0x00000000U
#define FDCAN_FD_CAN            0x00000001U

#define FDCAN_NO_TX_EVENTS      0x00000000U

#define FDCAN_RX_FIFO0          0x00000001U
#define FDCAN_RX_FIFO1          0x00000002U

/* -------------------------------------------------------------------------
   FDCAN – estructuras de cabecera TX/RX (solo campos usados en can.c)
   ---------------------------------------------------------------------- */
typedef struct {
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t TxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t TxEventFifoControl;
    uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct {
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t RxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t RxTimestamp;
    uint32_t FilterIndex;
    uint32_t IsFilterMatchingFrame;
} FDCAN_RxHeaderTypeDef;

/* Handle FDCAN mínimo (can.c sólo accede al puntero de handle) */
typedef struct {
    uint32_t Instance;   /* placeholder */
} FDCAN_HandleTypeDef;

/* -------------------------------------------------------------------------
   Funciones HAL FDCAN (stubs – retornan HAL_ERROR en SIL)
   ---------------------------------------------------------------------- */
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
                                                  FDCAN_TxHeaderTypeDef *pTxHeader,
                                                  uint8_t *pTxData);

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t RxLocation,
                                          FDCAN_RxHeaderTypeDef *pRxHeader,
                                          uint8_t *pRxData);

/* -------------------------------------------------------------------------
   Error handler (stub)
   ---------------------------------------------------------------------- */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_SIL_H */
