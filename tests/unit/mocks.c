#include "mocks.h"
#include <string.h>
#include <stdlib.h>

/* ===== Global state para mocks ===== */
uint32_t mock_kernel_tick_ms = 0;

/* ===== Mock Inputs ===== */

app_inputs_t mock_input_nominal(void)
{
    app_inputs_t in = {
        .s1_aceleracion = 2100,
        .s2_aceleracion = 1960,
        .s_freno = 2000,
        .boton_arranque = 0,
        .inv_state = 0x04,
        .inv_dc_bus_voltage = 400,
        .inv_vdc_ready = 1,
        .inv_motor_temp = 20,
        .inv_igbt_temp = 25,
        .inv_air_temp = 30,
        .inv_rpm = 0,
        .inv_speed_actual = 0,
        .inv_current_actual = 0,
        .inv_error = 0,
        .v_celda_min = 3600,
        .ok_precarga = 1,
        .flag_EV_2_3 = 0,
        .flag_T11_8_9 = 0,
        .torque_total = 0
    };
    return in;
}

app_inputs_t mock_input_brake_engaged(void)
{
    app_inputs_t in = mock_input_nominal();
    in.s_freno = 4500;
    return in;
}

app_inputs_t mock_input_throttle_max(void)
{
    app_inputs_t in = mock_input_nominal();
    in.s1_aceleracion = 2950;
    in.s2_aceleracion = 2570;
    return in;
}

app_inputs_t mock_input_throttle_50pct(void)
{
    app_inputs_t in = mock_input_nominal();
    in.s1_aceleracion = 2500;
    in.s2_aceleracion = 2243;
    return in;
}

app_inputs_t mock_input_inverter_fault(void)
{
    app_inputs_t in = mock_input_nominal();
    in.inv_state = 10;
    in.inv_error = 1;
    return in;
}

app_inputs_t mock_input_vdc_oob(void)
{
    app_inputs_t in = mock_input_nominal();
    in.inv_dc_bus_voltage = 0xFFFF;
    return in;
}

/* ===== Mock CAN Frames ===== */

can_msg_t mock_can_frame(uint32_t id, const uint8_t data[8])
{
    can_msg_t m = {
        .bus = CAN_BUS_INV,
        .id = id,
        .dlc = 8,
        .ide = 0
    };
    if (data) {
        memcpy(m.data, data, 8);
    } else {
        memset(m.data, 0, 8);
    }
    return m;
}

can_msg_t mock_can_precarga_ack(uint8_t ack)
{
    can_msg_t m = {0};
    m.bus = CAN_BUS_ACU;
    m.id = 0x20;
    m.dlc = 8;
    m.data[0] = ack;
    return m;
}

can_msg_t mock_can_dc_bus_voltage(uint16_t voltage_raw)
{
    can_msg_t m = {0};
    m.bus = CAN_BUS_INV;
    m.id = 0x466;
    m.dlc = 6;
    m.data[2] = (uint8_t)(voltage_raw & 0xFF);
    m.data[3] = (uint8_t)((voltage_raw >> 8) & 0xFF);
    return m;
}

can_msg_t mock_can_cell_min_voltage(uint16_t voltage_raw)
{
    can_msg_t m = {0};
    m.bus = CAN_BUS_ACU;
    m.id = 0x12C;
    m.dlc = 8;
    m.data[0] = (uint8_t)((voltage_raw >> 8) & 0xFF);
    m.data[1] = (uint8_t)(voltage_raw & 0xFF);
    return m;
}

can_msg_t mock_can_inv_state(uint8_t state, uint8_t error_code)
{
    can_msg_t m = {0};
    m.bus = CAN_BUS_INV;
    m.id = 0x461;
    m.dlc = 8;
    m.data[2] = error_code;
    m.data[4] = (uint8_t)(state & 0x0F);
    return m;
}

can_msg_t mock_can_inv_temps(uint8_t motor, uint8_t igbt, uint8_t air)
{
    can_msg_t m = {0};
    m.bus = CAN_BUS_INV;
    m.id = 0x464;
    m.dlc = 8;
    m.data[0] = motor;
    m.data[1] = igbt;
    m.data[2] = air;
    return m;
}

/* ===== Mock RTOS ===== */

uint32_t osKernelGetTickCount(void)
{
    return mock_kernel_tick_ms;
}

void mock_tick_advance(uint32_t ms)
{
    mock_kernel_tick_ms += ms;
}

void mock_tick_reset(void)
{
    mock_kernel_tick_ms = 0;
}

/* ===== Helpers para Assertion ===== */

int mock_can_frames_equal(const can_msg_t *a, const can_msg_t *b)
{
    if (!a || !b) return 0;

    return (a->id == b->id &&
            a->bus == b->bus &&
            a->dlc == b->dlc &&
            a->ide == b->ide &&
            memcmp(a->data, b->data, 8) == 0);
}

int mock_app_inputs_equal(const app_inputs_t *a, const app_inputs_t *b)
{
    if (!a || !b) return 0;

    return (a->s1_aceleracion == b->s1_aceleracion &&
            a->s2_aceleracion == b->s2_aceleracion &&
            a->s_freno == b->s_freno &&
            a->inv_state == b->inv_state &&
            a->inv_dc_bus_voltage == b->inv_dc_bus_voltage &&
            a->torque_total == b->torque_total);
}

/* ===== Mock stubs para no-op RTOS calls ===== */

osMutexId_t g_inMutex = NULL;
osMessageQueueId_t canRxQueueHandle = NULL;
osMessageQueueId_t canTxQueueHandle = NULL;
osMessageQueueId_t telemetryEventQueueHandle = NULL;
osMessageQueueId_t telemetryRadioQueueHandle = NULL;
osMessageQueueId_t telemetrySdQueueHandle = NULL;
osMessageQueueId_t telemetryDashQueueHandle = NULL;

/* Stubs de app_state.c */
app_inputs_t g_in = {0};

void AppState_Init(void)
{
    memset(&g_in, 0, sizeof(g_in));
    g_in.v_celda_min = 3600u;
}

osStatus_t osMessageQueuePut(osMessageQueueId_t mq_id, const void *msg_ptr,
                             uint8_t msg_prio, uint32_t timeout)
{
    (void)mq_id;
    (void)msg_ptr;
    (void)msg_prio;
    (void)timeout;
    return osErrorResource;
}

osStatus_t osMessageQueueReset(osMessageQueueId_t mq_id)
{
    (void)mq_id;
    return osOK;
}
