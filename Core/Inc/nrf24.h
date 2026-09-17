#ifndef __NRF24_H
#define __NRF24_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

typedef struct
{
  uint8_t nopStatus;
  uint8_t status;
  uint8_t writeStatus;
  uint8_t config;
  uint8_t rfChannelBefore;
  uint8_t rfChannelAfter;
  GPIO_PinState irqState;
  HAL_StatusTypeDef halStatus;
} NRF24_TestResult;

#define NRF24_MAX_PAYLOAD_SIZE    32U

void NRF24_BusInit(void);
void NRF24_SetCE(uint8_t high);
void NRF24_SetCSN(uint8_t high);
GPIO_PinState NRF24_GetIrqState(void);
HAL_StatusTypeDef NRF24_ReadStatusNop(uint8_t *statusValue);
HAL_StatusTypeDef NRF24_ReadRawNop4(uint8_t raw[4]);
HAL_StatusTypeDef NRF24_ReadRegister(uint8_t reg, uint8_t *value, uint8_t *statusValue);
HAL_StatusTypeDef NRF24_ReadRegisterMulti(uint8_t reg, uint8_t *values, uint8_t length, uint8_t *statusValue);
HAL_StatusTypeDef NRF24_WriteRegister(uint8_t reg, uint8_t value, uint8_t *statusValue);
HAL_StatusTypeDef NRF24_WriteRegisterMulti(uint8_t reg, const uint8_t *values, uint8_t length, uint8_t *statusValue);
HAL_StatusTypeDef NRF24_ApplyDefaultConfig(void);
HAL_StatusTypeDef NRF24_RunSelfTest(NRF24_TestResult *result);
void NRF24_DiagnoseSerial(UART_HandleTypeDef *huart);
HAL_StatusTypeDef NRF24_EnterRxMode(void);
HAL_StatusTypeDef NRF24_SendPayload(const uint8_t *payload, uint8_t length, uint32_t timeoutMs);
HAL_StatusTypeDef NRF24_ReadRxPayload(uint8_t *payload, uint8_t length);
HAL_StatusTypeDef NRF24_ClearIrqFlags(uint8_t irqMask);
HAL_StatusTypeDef NRF24_FlushRx(void);
HAL_StatusTypeDef NRF24_FlushTx(void);

#ifdef __cplusplus
}
#endif

#endif /* __NRF24_H */
