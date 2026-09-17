/**
 * sil_hal_mocks.c
 * Implementation of mock HAL drivers
 */

#include "sil_hal_mocks.h"
#include "can.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ===== ADC simulation ===== */
static uint16_t adc_values[16] = {0};

void SIL_ADC_SetValue(uint32_t channel, uint16_t value)
{
    if (channel < 16) {
        adc_values[channel] = value;
        printf("[ADC] Channel %u = %u (0x%04x)\n", channel, value, value);
    }
}

uint16_t SIL_ADC_Read(uint32_t channel)
{
    if (channel < 16) {
        return adc_values[channel];
    }
    return 0;
}

/* ===== CAN simulation ===== */
/* NOTE: SIL_CAN_Init is defined in sil_can_simulator.c */
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

static FDCAN_HandleTypeDef *guess_fdcan_from_id(uint32_t id)
{
    if (id == 0x020u || id == 0x12Cu || id == 0x4A0u) return &hfdcan2;
    return &hfdcan1;
}

void SIL_CAN_SendFrame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    FDCAN_HandleTypeDef *hfdcan = guess_fdcan_from_id(id);

    if (SIL_FDCAN_InjectRxFrame(hfdcan, id, FDCAN_STANDARD_ID, data, dlc) != HAL_OK) {
        printf("[CAN] RX fifo full, dropping frame 0x%03x\n", id);
        return;
    }

    Can_ISR_PushRxFifo0(hfdcan);
    printf("[CAN] RX->ISR: ID=0x%03x DLC=%u\n", id, dlc);
}

int SIL_CAN_ReceiveFrame(uint32_t *id, uint8_t *data, uint8_t *dlc)
{
    FDCAN_TxHeaderTypeDef txh;
    uint8_t payload[8] = {0};
    FDCAN_HandleTypeDef *buses[] = {&hfdcan1, &hfdcan2, &hfdcan3};

    for (size_t i = 0; i < 3; i++) {
        if (SIL_FDCAN_PopTxFrame(buses[i], &txh, payload) == HAL_OK) {
            if (id)  *id  = txh.Identifier;
            if (dlc) *dlc = (uint8_t)((txh.DataLength >> 16) & 0xFu);
            if (data) memcpy(data, payload, 8);
            return 1;
        }
    }

    return 0;  /* No frame available */
}

/* ===== GPIO simulation ===== */
static uint32_t gpio_states[32] = {0};

void SIL_GPIO_Write(uint32_t pin, int state)
{
    if (pin < 32) {
        gpio_states[pin] = state ? 1 : 0;
        printf("[GPIO] Pin %u = %d\n", pin, state);
    }
}

int SIL_GPIO_Read(uint32_t pin)
{
    if (pin < 32) {
        return gpio_states[pin];
    }
    return 0;
}

/* ===== USART simulation ===== */
#define SIL_USART_BUFFER_SIZE 256
static uint8_t usart_tx_buffer[SIL_USART_BUFFER_SIZE] = {0};
static uint8_t usart_rx_buffer[SIL_USART_BUFFER_SIZE] = {0};
static uint32_t usart_tx_idx = 0, usart_rx_idx = 0;

void SIL_USART_Init(void)
{
    printf("[USART] USART SIL initialized\n");
}

void SIL_USART_Send(const uint8_t *data, uint16_t size)
{
    if (usart_tx_idx + size > SIL_USART_BUFFER_SIZE) {
        printf("[USART] TX buffer overflow\n");
        return;
    }
    
    memcpy(&usart_tx_buffer[usart_tx_idx], data, size);
    usart_tx_idx += size;
    printf("[USART] TX: %u bytes\n", size);
}

int SIL_USART_Receive(uint8_t *data, uint16_t size)
{
    if (usart_rx_idx < SIL_USART_BUFFER_SIZE && data) {
        uint16_t available = SIL_USART_BUFFER_SIZE - usart_rx_idx;
        uint16_t to_read = size < available ? size : available;
        memcpy(data, &usart_rx_buffer[usart_rx_idx], to_read);
        usart_rx_idx += to_read;
        return to_read;
    }
    return 0;
}

/* ===== Timer ===== */
extern uint32_t sil_get_time_ms(void);

uint32_t SIL_GetTickMs(void)
{
    return sil_get_time_ms();
}

/* ===== Global initialization ===== */
void SIL_HAL_Init(void)
{
    printf("[HAL] Initializing all SIL HAL mocks\n");
    
    memset(adc_values, 0, sizeof(adc_values));
    memset(gpio_states, 0, sizeof(gpio_states));
    memset(usart_tx_buffer, 0, sizeof(usart_tx_buffer));
    memset(usart_rx_buffer, 0, sizeof(usart_rx_buffer));
    SIL_FDCAN_Reset();
    
    SIL_USART_Init();
    SIL_CAN_Init();
    
    printf("[HAL] All SIL HAL mocks initialized\n");
}
