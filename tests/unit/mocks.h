#ifndef TEST_MOCKS_H
#define TEST_MOCKS_H

#include <stdint.h>
#include <string.h>
#include "app_state.h"
#include "can.h"

/*
 * Mock/Stub layer para tests unit en host.
 * Reemplaza dependencias de RTOS, HAL, ISRs.
 */

/* ===== Mocks de Input ===== */

/**
 * Crea input nominal (sin throttle, sin freno, todo en range)
 */
app_inputs_t mock_input_nominal(void);

/**
 * Crea input con freno activado
 */
app_inputs_t mock_input_brake_engaged(void);

/**
 * Crea input con throttle al máximo
 */
app_inputs_t mock_input_throttle_max(void);

/**
 * Crea input con throttle a 50%
 */
app_inputs_t mock_input_throttle_50pct(void);

/**
 * Crea input con inversor en estado inválido
 */
app_inputs_t mock_input_inverter_fault(void);

/**
 * Crea input con tensión DC fuera de rango
 */
app_inputs_t mock_input_vdc_oob(void);

/* ===== Mocks de CAN ===== */

/**
 * Crea un frame CAN sintético con ID y payload dados
 */
can_msg_t mock_can_frame(uint32_t id, const uint8_t data[8]);

/**
 * Frame de ACK de precarga (ID 0x20)
 */
can_msg_t mock_can_precarga_ack(uint8_t ack);

/**
 * Frame legado TX_STATE_7 del inversor (ID 0x466, Vdc en bytes 2..3 LE)
 */
can_msg_t mock_can_dc_bus_voltage(uint16_t voltage_raw);

/**
 * Frame de tensión mínima celda (ID 0x12C)
 */
can_msg_t mock_can_cell_min_voltage(uint16_t voltage_raw);

/**
 * Frame legado TX_STATE_2 del inversor (estado en byte4, error en byte2).
 */
can_msg_t mock_can_inv_state(uint8_t state, uint8_t error_code);

/**
 * Frame legado TX_STATE_5 con temperaturas del inversor.
 */
can_msg_t mock_can_inv_temps(uint8_t motor, uint8_t igbt, uint8_t air);

/* ===== Mocks de RTOS ===== */

/**
 * Mock del tick del kernel RTOS
 */
extern uint32_t mock_kernel_tick_ms;

/**
 * Mock de osKernelGetTickCount() para tests
 */
uint32_t osKernelGetTickCount(void);

/**
 * Avanzar el tick simulado N ms
 */
void mock_tick_advance(uint32_t ms);

/**
 * Resetear el tick a 0
 */
void mock_tick_reset(void);

/* ===== Helpers para Assertion ===== */

/**
 * Verificar que dos frames CAN son idénticos
 */
int mock_can_frames_equal(const can_msg_t *a, const can_msg_t *b);

/**
 * Verificar que dos inputs son iguales (comparación de campos clave)
 */
int mock_app_inputs_equal(const app_inputs_t *a, const app_inputs_t *b);

#endif /* TEST_MOCKS_H */
