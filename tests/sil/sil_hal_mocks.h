/**
 * sil_hal_mocks.h
 * Mock HAL drivers for SIL environment
 */

#ifndef SIL_HAL_MOCKS_H
#define SIL_HAL_MOCKS_H

#include <stdint.h>
#include "main.h"

/* Initialize all HAL mocks */
void SIL_HAL_Init(void);

/* ADC mocking */
uint16_t SIL_ADC_Read(uint32_t channel);
void SIL_ADC_SetValue(uint32_t channel, uint16_t value);

/* CAN mocking */
void SIL_CAN_Init(void);
void SIL_CAN_SendFrame(uint32_t id, const uint8_t *data, uint8_t dlc);
int SIL_CAN_ReceiveFrame(uint32_t *id, uint8_t *data, uint8_t *dlc);
void SIL_FDCAN_Reset(void);
HAL_StatusTypeDef SIL_FDCAN_InjectRxFrame(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t id,
                                          uint32_t id_type,
                                          const uint8_t *data,
                                          uint8_t dlc);
HAL_StatusTypeDef SIL_FDCAN_PopTxFrame(FDCAN_HandleTypeDef *hfdcan,
                                       FDCAN_TxHeaderTypeDef *pTxHeader,
                                       uint8_t *pTxData);
uint32_t SIL_FDCAN_GetTxCount(FDCAN_HandleTypeDef *hfdcan);

/* GPIO mocking */
void SIL_GPIO_Write(uint32_t pin, int state);
int SIL_GPIO_Read(uint32_t pin);

/* USART mocking */
void SIL_USART_Init(void);
void SIL_USART_Send(const uint8_t *data, uint16_t size);
int SIL_USART_Receive(uint8_t *data, uint16_t size);

/* Timer mocking */
uint32_t SIL_GetTickMs(void);

#endif /* SIL_HAL_MOCKS_H */
