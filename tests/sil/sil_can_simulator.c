/**
 * sil_can_simulator.c
 * CAN message simulator - injects simulated CAN messages into the app
 */

#include "sil_can_simulator.h"
#include "sil_hal_mocks.h"
#include <stdio.h>
#include <string.h>

static struct {
    uint8_t throttle;
    uint8_t brake;
    uint8_t start_button;
    uint8_t inverter_state;
    uint16_t dc_voltage;
    uint32_t last_update_ms;
} sim_state = {
    .throttle = 0,
    .brake = 0,
    .start_button = 0,
    .inverter_state = 0x03,  /* STANDBY */
    .dc_voltage = 400,
    .last_update_ms = 0
};

static uint16_t sil_apps1_raw_from_pct(uint8_t throttle_pct)
{
    return (uint16_t)(2050u + ((uint32_t)throttle_pct * 900u) / 100u);
}

static uint16_t sil_apps2_raw_from_pct(uint8_t throttle_pct)
{
    return (uint16_t)(1915u + ((uint32_t)throttle_pct * 655u) / 100u);
}

static uint16_t sil_brake_raw_from_pct(uint8_t brake_pct)
{
    if (brake_pct == 0u) {
        return 0u;
    }

    return (uint16_t)(2000u + ((uint32_t)brake_pct * 2000u) / 100u);
}

void SIL_CAN_Init(void)
{
    sim_state.throttle = 0;
    sim_state.brake = 0;
    sim_state.start_button = 0;
    sim_state.inverter_state = 0x03;
    sim_state.dc_voltage = 400;
    sim_state.last_update_ms = 0;
    printf("[CAN-SIM] CAN Simulator initialized\n");
}

void SIL_CAN_InjectThrottle(uint8_t throttle_pct)
{
    if (throttle_pct > 100) throttle_pct = 100;
    sim_state.throttle = throttle_pct;
    printf("[CAN-SIM] Throttle injected: %u%%\n", throttle_pct);
}

void SIL_CAN_InjectBrake(uint8_t brake_pct)
{
    if (brake_pct > 100) brake_pct = 100;
    sim_state.brake = brake_pct;
    printf("[CAN-SIM] Brake injected: %u%%\n", brake_pct);
}

void SIL_CAN_InjectInverterState(uint8_t state)
{
    sim_state.inverter_state = state;
    printf("[CAN-SIM] Inverter state injected: 0x%02x\n", state);
}

void SIL_CAN_InjectDCVoltage(uint16_t voltage_v)
{
    sim_state.dc_voltage = voltage_v;
    printf("[CAN-SIM] DC voltage injected: %u V\n", voltage_v);
}

void SIL_IO_SetStartButton(uint8_t pressed)
{
    sim_state.start_button = pressed ? 1u : 0u;
    printf("[IO-SIM] Start button: %s\n", sim_state.start_button ? "ON" : "OFF");
}

uint8_t SIL_IO_GetStartButton(void)
{
    return sim_state.start_button;
}

uint16_t SIL_IO_GetApps1Raw(void)
{
    return sil_apps1_raw_from_pct(sim_state.throttle);
}

uint16_t SIL_IO_GetApps2Raw(void)
{
    return sil_apps2_raw_from_pct(sim_state.throttle);
}

uint16_t SIL_IO_GetBrakeRaw(void)
{
    return sil_brake_raw_from_pct(sim_state.brake);
}

void SIL_CAN_Process(void)
{
    /* Periodically inject CAN messages based on current simulated state */

    uint32_t current_time = SIL_GetTickMs();
    
    /* Send precharge ACK (ID 0x20) every 100ms */
    if ((current_time - sim_state.last_update_ms) >= 100) {
        uint8_t data[8] = {0x01, 0, 0, 0, 0, 0, 0, 0};  /* AMS v1.2+: ACK=1 => precarga OK */

        SIL_CAN_SendFrame(0x20, data, 8);

        /* AMS status 0x4A0: byte0 = FSM state (3 = Run -- not Start(0)/Error(5)),
         * byte3 = session id. The ECU's precharge gate now requires a fresh,
         * running AMS status alongside the 0x020 precharge ACK, so the simulated
         * AMS must stream it like real hardware does. */
        {
            uint8_t ams[8] = {3u, 0, 0, 1u, 0, 0, 0, 0};
            SIL_CAN_SendFrame(0x4A0, ams, 8);
        }

        /* Send inverter DC bus voltage feedback (legacy TX_STATE_7 / 0x466). */
        data[2] = (uint8_t)(sim_state.dc_voltage & 0xFF);
        data[3] = (uint8_t)((sim_state.dc_voltage >> 8) & 0xFF);
        SIL_CAN_SendFrame(0x466, data, 6);

        /* Inverter state feedback (legacy TX_STATE_2). */
        memset(data, 0, sizeof(data));
        data[4] = (uint8_t)(sim_state.inverter_state & 0x0Fu);
        SIL_CAN_SendFrame(0x461, data, 8);
        
        sim_state.last_update_ms = current_time;
    }
}
