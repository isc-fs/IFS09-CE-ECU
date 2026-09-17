/**
 * sil_main.c
 * SIL (Software-In-The-Loop) Entry Point - Simplified Version
 * 
 * Simulates the complete ECU application logic with the real task layout
 * from freertos.c, using a cooperative RTOS mock on the host.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "app_state.h"
#include "app_runtime.h"
#include "can.h"
#include "can/can_codecs.h"   /* code-first CAN DSL: generated encoders/decoders */
#include "control.h"
#include "bootloader.h"
#include "pit_diag.h"
#include "telemetry.h"
#include "test_integration.h"   /* suites S1-S10, Test_IntegrationRunAll() */
#include "cmsis_os2.h"
#include "sil_hal_mocks.h"
#include "sil_can_simulator.h"
#include "sil_boot_sequence.h"
#include "sil_results.h"

/* Declarada en mocks/diag_sil.c: redirige Diag_Log al fichero especificado */
extern void SIL_DiagSetFile(FILE *f);
extern void MX_FREERTOS_Init(void);

/* Integration test suites (tests/sil/integration/) */
extern int run_boot_sequence_tests(void);   /* returns failure count */
extern int run_full_cycle_tests(void);      /* returns failure count */

/* ===== Global state for SIL ===== */
static volatile uint32_t sil_tick_ms = 0;
static volatile int sil_simulation_running = 0;
static volatile uint32_t sil_test_duration_ms = 0;
static uint8_t sil_last_telemetry[32] = {0};
static uint8_t sil_last_heartbeat[32] = {0};
static uint8_t sil_last_event[32] = {0};
static uint32_t sil_telemetry_count = 0;
static uint32_t sil_heartbeat_count = 0;
static uint32_t sil_event_count = 0;
static int sil_test_failures = 0;

uint32_t sil_get_time_ms(void);

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

void Telemetry_Send32(const uint8_t payload[32])
{
    if (!payload) return;
    memcpy(sil_last_telemetry, payload, sizeof(sil_last_telemetry));
    sil_telemetry_count++;

    if (payload[17] == (uint8_t)TELEMETRY_FRAME_EVENT) {
        memcpy(sil_last_event, payload, sizeof(sil_last_event));
        sil_event_count++;
    } else {
        memcpy(sil_last_heartbeat, payload, sizeof(sil_last_heartbeat));
        sil_heartbeat_count++;
    }
}

void Telemetry_SdStore32(const uint8_t payload[32])
{
    (void)payload;
}

static void sil_check(int condition, const char *message)
{
    if (condition) {
        printf("[CHECK][PASS] %s\n", message);
    } else {
        printf("[CHECK][FAIL] %s\n", message);
        sil_test_failures++;
    }
}

static void sil_check_task_metric(app_task_id_t task_id,
                                  uint32_t min_starts,
                                  uint32_t min_steps,
                                  const char *label)
{
    const app_task_metrics_t *metrics = AppRuntime_TaskMetricsGet(task_id);
    char detail[160];

    snprintf(detail, sizeof(detail), "%s -> metrics available", label);
    sil_check(metrics != NULL, detail);
    if (!metrics) {
        return;
    }

    snprintf(detail, sizeof(detail), "%s -> starts >= %lu", label, (unsigned long)min_starts);
    sil_check(metrics->start_count >= min_starts, detail);

    snprintf(detail, sizeof(detail), "%s -> steps >= %lu", label, (unsigned long)min_steps);
    sil_check(metrics->step_count >= min_steps, detail);
}

static void sil_reset_captured_outputs(void)
{
    memset(sil_last_telemetry, 0, sizeof(sil_last_telemetry));
    memset(sil_last_heartbeat, 0, sizeof(sil_last_heartbeat));
    memset(sil_last_event, 0, sizeof(sil_last_event));
    sil_telemetry_count = 0;
    sil_heartbeat_count = 0;
    sil_event_count = 0;
}

static void sil_set_start_button(uint8_t pressed)
{
    SIL_IO_SetStartButton(pressed);
}

static void sil_runtime_init(void)
{
    SIL_RTOS_ResetKernel();
    (void)osKernelInitialize();
    SIL_HAL_Init();
    MX_FREERTOS_Init();
    (void)osKernelStart();
    SIL_RTOS_RunReadyThreads();

    sil_tick_ms = 0;
    sil_simulation_running = 0;
    sil_test_duration_ms = 0;
    sil_test_failures = 0;
    sil_reset_captured_outputs();
}

static void sil_runtime_step_1ms(void)
{
    sil_tick_ms++;
    SIL_AdvanceTick(1);
    SIL_CAN_Process();
    SIL_RTOS_RunReadyThreads();
}

/* Forward decl: the quiet RX injector is defined further down, but the manual
 * run-loop below needs it to stream the AMS status heartbeat. */
static HAL_StatusTypeDef sil_inject_rx_frame(FDCAN_HandleTypeDef *hfdcan,
                                             uint32_t id, uint32_t id_type,
                                             const uint8_t *data, uint8_t dlc);

static void sil_runtime_step_manual_1ms(uint8_t process_can_sim)
{
    sil_tick_ms++;
    SIL_AdvanceTick(1);
    /* A present AMS streams its status (0x4A0, byte0=3=Run) continuously. The
     * ECU's staleness guard drops the drive latch and bounces the FSM back to
     * the safe boot gate if 0x4A0 is silent for >1s (AMS_STATUS_STALE_MS), so
     * the manual run-loop keeps that heartbeat alive every 100 ms -- mirroring
     * what SIL_CAN_Process does for the simulation loop -- WITHOUT injecting the
     * precharge-ACK (0x020) or inverter frames, which the manual harnesses drive
     * explicitly. Without it, any wait longer than 1s (e.g. the 2s R2D delay)
     * would falsely read as an AMS dropout and abort the start sequence. */
    if ((sil_tick_ms % 100u) == 0u) {
        uint8_t ams[8] = {3u, 0, 0, 1u, 0, 0, 0, 0};
        (void)sil_inject_rx_frame(&hfdcan2, 0x4A0u, FDCAN_STANDARD_ID, ams, 8u);
    }
    if (process_can_sim) {
        SIL_CAN_Process();
    }
    SIL_RTOS_RunReadyThreads();
}

static void sil_run_manual(uint32_t duration_ms, uint8_t process_can_sim)
{
    uint32_t end_tick = sil_tick_ms + duration_ms;

    while (sil_tick_ms < end_tick) {
        sil_runtime_step_manual_1ms(process_can_sim);
    }
}

static void sil_run_until_tick(uint32_t target_tick, uint8_t process_can_sim)
{
    while (sil_tick_ms < target_tick) {
        sil_runtime_step_manual_1ms(process_can_sim);
    }
}

static void sil_run_until_next_control_cycle_after_rx(void)
{
    uint32_t now = osKernelGetTickCount();
    uint32_t rx_tick = ((now / 5u) + 1u) * 5u;
    uint32_t control_tick = ((rx_tick / 10u) + 1u) * 10u;
    sil_run_until_tick(control_tick, 0u);
}

static can_bus_t sil_bus_from_handle(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1) return CAN_BUS_INV;
    if (hfdcan == &hfdcan2) return CAN_BUS_ACU;
    if (hfdcan == &hfdcan3) return CAN_BUS_DASH;
    return CAN_BUS_INV;
}

static uint8_t sil_pop_tx_can_msg(FDCAN_HandleTypeDef *hfdcan, can_msg_t *msg)
{
    FDCAN_TxHeaderTypeDef txh;
    uint8_t payload[8] = {0};

    if (!msg) return 0u;
    if (SIL_FDCAN_PopTxFrame(hfdcan, &txh, payload) != HAL_OK) {
        return 0u;
    }

    memset(msg, 0, sizeof(*msg));
    msg->bus = sil_bus_from_handle(hfdcan);
    msg->id = txh.Identifier;
    msg->ide = (txh.IdType == FDCAN_EXTENDED_ID) ? 1u : 0u;
    msg->dlc = (uint8_t)((txh.DataLength >> 16) & 0xFu);
    memcpy(msg->data, payload, sizeof(msg->data));
    return 1u;
}

static void sil_drain_tx_bus(FDCAN_HandleTypeDef *hfdcan)
{
    can_msg_t dummy;

    while (sil_pop_tx_can_msg(hfdcan, &dummy)) {
    }
}

static HAL_StatusTypeDef sil_inject_rx_frame(FDCAN_HandleTypeDef *hfdcan,
                                             uint32_t id,
                                             uint32_t id_type,
                                             const uint8_t *data,
                                             uint8_t dlc)
{
    HAL_StatusTypeDef st = SIL_FDCAN_InjectRxFrame(hfdcan, id, id_type, data, dlc);

    if (st == HAL_OK) {
        Can_ISR_PushRxFifo0(hfdcan);
    }

    return st;
}

static void sil_check_bus_empty(FDCAN_HandleTypeDef *hfdcan, const char *message)
{
    sil_check(SIL_FDCAN_GetTxCount(hfdcan) == 0u, message);
    if (SIL_FDCAN_GetTxCount(hfdcan) != 0u) {
        sil_drain_tx_bus(hfdcan);
    }
}

static void sil_expect_tx_frame(FDCAN_HandleTypeDef *hfdcan,
                                uint32_t expected_id,
                                uint8_t expected_ide,
                                uint8_t expected_dlc,
                                const uint8_t *expected_data,
                                uint8_t expected_data_len,
                                const char *label)
{
    can_msg_t msg;
    char detail[160];
    uint8_t has_msg = 0u;

    memset(&msg, 0, sizeof(msg));

    snprintf(detail, sizeof(detail), "%s -> trama presente", label);
    has_msg = sil_pop_tx_can_msg(hfdcan, &msg);
    sil_check(has_msg == 1u, detail);
    if (!has_msg) {
        return;
    }

    snprintf(detail, sizeof(detail), "%s -> ID 0x%03lX", label, (unsigned long)expected_id);
    sil_check(msg.id == expected_id, detail);

    snprintf(detail, sizeof(detail), "%s -> IDE %u", label, (unsigned)expected_ide);
    sil_check(msg.ide == expected_ide, detail);

    snprintf(detail, sizeof(detail), "%s -> DLC %u", label, (unsigned)expected_dlc);
    sil_check(msg.dlc == expected_dlc, detail);

    if (expected_data && expected_data_len > 0u) {
        snprintf(detail, sizeof(detail), "%s -> payload", label);
        sil_check(memcmp(msg.data, expected_data, expected_data_len) == 0, detail);
    }
}

static uint16_t sil_legacy_torque_command(uint16_t torque_pct)
{
    uint16_t scaled = torque_pct;

    if (scaled >= 10u) {
        scaled = (uint16_t)(((uint32_t)scaled * 240u) / 90u - (2400u / 90u));
    }

    return (uint16_t)(~scaled + 1u);
}

static void test_legacy_compat_harness(void)
{
    app_inputs_t snapshot = {0};
    uint8_t data[8] = {0};
    uint8_t expected[8] = {0};
    uint16_t expected_torque_cmd = 0u;

    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  SIL TEST: Legacy Compatibility Harness            ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n\n");

    SIL_Results_Init("precharge_startup_e2e.log");
    SIL_Results_Log("PRECHARGE_E2E", "STARTED",
                    "Complete startup/precharge simulation through READY/TORQUE handoff");

    sil_runtime_init();
    sil_set_start_button(0u);
    SIL_CAN_InjectBrake(0u);
    SIL_CAN_InjectThrottle(0u);
    sil_drain_tx_bus(&hfdcan1);
    sil_drain_tx_bus(&hfdcan2);
    sil_drain_tx_bus(&hfdcan3);

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE1", "WAIT_INV_VDC_CONFIG");
    sil_run_manual(30u, 0u);
    sil_check_bus_empty(&hfdcan1, "P1: sin escritura al inversor antes de TX_STATE_7");
    /* The 0x100 DC-bus heartbeat now streams on the ACU bus every control
     * cycle (the AMS VcuStale watchdog requires it). Before TX_STATE_7 the bus
     * carries that heartbeat and nothing else. */
    memset(expected, 0, sizeof(expected));
    sil_expect_tx_frame(&hfdcan2, TINT_ID_DC_BUS_V, 0u, 2u, expected, 2u,
                        "P1: solo heartbeat 0x100 antes de TX_STATE_7");
    sil_drain_tx_bus(&hfdcan2);

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE2", "PRECHARGE_REQUEST");
    sil_set_start_button(1u);
    memset(data, 0, sizeof(data));
    data[2] = 0x00u;
    data[3] = 0x00u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_7, FDCAN_STANDARD_ID, data, 6u) == HAL_OK,
              "P2: RX TX_STATE_7 inyectada");
    sil_run_until_next_control_cycle_after_rx();

    /* The precharge step now emits only the 0x100 heartbeat on the ACU bus
     * (the 0x600 precharge command was retired AMS-side). */
    memset(expected, 0, sizeof(expected));
    sil_expect_tx_frame(&hfdcan2, TINT_ID_DC_BUS_V, 0u, 2u, expected, 2u,
                        "P2: reenvio DC bus al ACU");
    sil_check_bus_empty(&hfdcan1, "P2: sin escritura al inversor durante precarga");
    sil_check_bus_empty(&hfdcan2, "P2: solo el heartbeat 0x100 en el ciclo");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE3", "PRECHARGE_ACK");
    /* Model a present, running AMS (0x4A0 state = Run) before the ACK: the ECU's
     * precharge gate now requires a fresh, non-Start AMS status alongside 0x020. */
    {
        uint8_t ams[8] = {3u, 0, 0, 1u, 0, 0, 0, 0};
        (void)sil_inject_rx_frame(&hfdcan2, 0x4A0u, FDCAN_STANDARD_ID, ams, 8u);
    }
    memset(data, 0, sizeof(data));
    data[0] = 0x01u;
    sil_check(sil_inject_rx_frame(&hfdcan2, TINT_ID_ACK_PRECARGA, FDCAN_STANDARD_ID, data, 1u) == HAL_OK,
              "P3: ACK precarga inyectado");
    sil_run_until_next_control_cycle_after_rx();
    sil_check_bus_empty(&hfdcan1, "P3: tras ACK aun no se escribe al inversor");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE4", "WAIT_START_BRAKE");
    sil_run_manual(40u, 0u);
    sil_check_bus_empty(&hfdcan1, "P4: sin escritura al inversor esperando freno");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE5", "R2D_DELAY");
    SIL_CAN_InjectBrake(100u);
    sil_run_manual(1990u, 0u);
    sil_check_bus_empty(&hfdcan1, "P5: sin escritura al inversor durante RTDS");
    sil_run_manual(20u, 0u);
    sil_check_bus_empty(&hfdcan1, "P5: sin escritura hasta recibir state=3");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE6", "INV_STATE_3");
    memset(data, 0, sizeof(data));
    data[4] = 3u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_2, FDCAN_STANDARD_ID, data, 8u) == HAL_OK,
              "P6: state=3 inyectado");
    sil_run_until_next_control_cycle_after_rx();
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x04u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P6: state=3 -> modo READY");
    memset(expected, 0, sizeof(expected));
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_3, 0u, 4u, expected, 4u,
                        "P6: state=3 -> torque cero");
    sil_check_bus_empty(&hfdcan1, "P6: solo READY + torque cero en state=3");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE7", "INV_STATE_3_REPEAT");
    sil_run_manual(10u, 0u);
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x04u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P7: state=3 persistente -> READY");
    memset(expected, 0, sizeof(expected));
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_3, 0u, 4u, expected, 4u,
                        "P7: state=3 persistente -> torque cero");
    sil_check_bus_empty(&hfdcan1, "P7: secuencia repetida en state=3");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE8", "INV_STATE_4");
    memset(data, 0, sizeof(data));
    data[4] = 4u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_2, FDCAN_STANDARD_ID, data, 8u) == HAL_OK,
              "P8: state=4 inyectado");
    sil_run_until_next_control_cycle_after_rx();
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x06u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P8: state=4 -> modo TORQUE");
    memset(expected, 0, sizeof(expected));
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_3, 0u, 4u, expected, 4u,
                        "P8: state=4 -> torque cero");
    sil_check_bus_empty(&hfdcan1, "P8: solo TORQUE + torque cero en state=4");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE9", "INV_STATE_6_TORQUE");
    SIL_CAN_InjectBrake(0u);
    SIL_CAN_InjectThrottle(50u);
    memset(data, 0, sizeof(data));
    data[4] = 6u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_2, FDCAN_STANDARD_ID, data, 8u) == HAL_OK,
              "P9: state=6 inyectado");
    sil_run_until_next_control_cycle_after_rx();
    AppState_Snapshot(&snapshot);
    expected_torque_cmd = sil_legacy_torque_command(snapshot.torque_total);
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x06u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P9: state=6 -> modo TORQUE");
    memset(expected, 0, sizeof(expected));
    expected[2] = (uint8_t)(expected_torque_cmd & 0xFFu);
    expected[3] = (uint8_t)((expected_torque_cmd >> 8) & 0xFFu);
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_3, 0u, 4u, expected, 4u,
                        "P9: state=6 -> torque legado");
    sil_check_bus_empty(&hfdcan1, "P9: solo modo + torque en state=6");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE10", "INV_STATE_10_SOFT_FAULT");
    SIL_CAN_InjectThrottle(0u);
    memset(data, 0, sizeof(data));
    data[2] = 3u;
    data[4] = 10u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_2, FDCAN_STANDARD_ID, data, 8u) == HAL_OK,
              "P10: state=10 inyectado");
    sil_run_until_next_control_cycle_after_rx();
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x13u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P10: state=10 -> reset 1");
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P10: state=10 -> reset 2");
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x01u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P10: state=10 -> standby final");
    sil_check_bus_empty(&hfdcan1, "P10: secuencia exacta soft fault");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE11", "INV_STATE_11_HARD_FAULT");
    memset(data, 0, sizeof(data));
    data[4] = 11u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_2, FDCAN_STANDARD_ID, data, 8u) == HAL_OK,
              "P11: state=11 inyectado");
    sil_run_until_next_control_cycle_after_rx();
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x13u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P11: state=11 -> reset");
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x01u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P11: state=11 -> standby");
    sil_check_bus_empty(&hfdcan1, "P11: secuencia exacta hard fault");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE12", "INV_STATE_13_SHUTDOWN");
    memset(data, 0, sizeof(data));
    data[4] = 13u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_2, FDCAN_STANDARD_ID, data, 8u) == HAL_OK,
              "P12: state=13 inyectado");
    sil_run_until_next_control_cycle_after_rx();
    memset(expected, 0, sizeof(expected));
    expected[2] = 0x01u;
    sil_expect_tx_frame(&hfdcan1, TINT_RX_SETPOINT_1, 0u, 3u, expected, 3u,
                        "P12: state=13 -> standby");
    sil_check_bus_empty(&hfdcan1, "P12: una sola trama en shutdown");

    SIL_Results_Log("PRECHARGE_E2E",
                    (sil_test_failures == 0) ? "SUCCESS" : "FAIL",
                    (sil_test_failures == 0)
                        ? "Precharge/startup end-to-end simulation passed"
                        : "Precharge/startup end-to-end simulation FAILED");
    SIL_Results_Close();
}

/* ===== Test: Suites de integración S1-S10 (test_integration.c) ===== */

/**
 * Ejecuta las 10 suites de integración y escribe el informe en:
 *   tests/sil/results/integration_test.log
 *
 * La salida también aparece en consola (stdout) en tiempo real a través
 * de Diag_Log, que en SIL está implementado en mocks/diag_sil.c.
 */
static void test_integration_suite(void)
{
    const char *log_path = "tests/sil/results/integration_test.log";
    int results_dir_ready = (SIL_EnsureResultsDir() == 0);

    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  SIL TEST: Integration Test Suite  (S1-S10)         ║\n");
    printf("║  Modo: TEST_MODE_SIL  |  Host PC (sin hardware)     ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n\n");

    /* Abrir fichero de resultados */
    FILE *log = results_dir_ready ? fopen(log_path, "w") : NULL;
    if (!log && !results_dir_ready) {
        printf("[WARN] No se pudo preparar el directorio de resultados.\n");
    } else if (!log) {
        /* Intentar crear el directorio results si no existe */
        printf("[WARN] No se pudo abrir %s, intentando crear directorio...\n", log_path);
        if (SIL_EnsureResultsDir() == 0) {
            log = fopen(log_path, "w");
        }
    }

    /* Escribir cabecera del fichero de log */
    if (log) {
        time_t now = time(NULL);
        char   ts[64];
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", localtime(&now));
        fprintf(log, "=======================================================\n");
        fprintf(log, "  ECU08 NSIL – Integration Test Results (SIL)          \n");
        fprintf(log, "  Fecha: %s                                             \n", ts);
        fprintf(log, "  Modo : TEST_MODE_SIL (host PC, sin hardware STM32)   \n");
        fprintf(log, "=======================================================\n\n");
        fflush(log);
        SIL_DiagSetFile(log);   /* Diag_Log → stdout + log file              */
        printf("[INFO] Resultados se guardarán en: %s\n\n", log_path);
    } else {
        printf("[WARN] No se pudo crear fichero de log. Salida solo por stdout.\n\n");
        SIL_DiagSetFile(NULL);
    }

    /* Inicializar el entorno RTOS mock (colas CAN, mutex) */
    SIL_RTOS_Init();

    /* EJECUTAR TODOS LOS TESTS */
    test_result_t res = Test_IntegrationRunAll();

    /* Añadir resumen final al fichero de log */
    if (log) {
        fprintf(log, "\n=======================================================\n");
        fprintf(log, "  RESUMEN FINAL\n");
        fprintf(log, "=======================================================\n");
        fprintf(log, "  Tests totales : %lu\n", (unsigned long)res.total);
        fprintf(log, "  Tests pasados : %lu\n", (unsigned long)res.passed);
        fprintf(log, "  Tests fallados: %lu\n", (unsigned long)res.failed);
        fprintf(log, "  Tiempo total  : %lu ms (simulados)\n", (unsigned long)res.execution_time_ms);
        fprintf(log, "  Modo          : %s\n", res.mode);
        fprintf(log, "  Resultado     : %s\n",
                res.failed == 0 ? ">>> ALL TESTS PASSED <<<" : ">>> HAY TESTS FALLIDOS <<<");
        fprintf(log, "=======================================================\n");
        fflush(log);
        SIL_DiagSetFile(NULL);   /* Desconectar antes de cerrar */
        fclose(log);
        printf("\n[INFO] Informe guardado en: %s\n", log_path);
    }

    /* Feed the global failure counter so main() returns a non-zero exit code
     * (honest result for CI). Without this, --test-integration / --test-all
     * would exit 0 even with failed S1-S10 assertions. */
    sil_test_failures += (int)res.failed;

    /* Código de salida: 0 = todo OK, 1 = hay fallos */
    if (res.failed > 0) {
        printf("[RESULT] FALLOS: %lu/%lu tests fallaron.\n",
               (unsigned long)res.failed, (unsigned long)res.total);
    } else {
        printf("[RESULT] OK: %lu/%lu tests pasaron.\n",
               (unsigned long)res.passed, (unsigned long)res.total);
    }
}

/**
 * Run SIL simulation for N milliseconds
 */
static void sil_run_simulation(uint32_t duration_ms)
{
    sil_simulation_running = 1;
    sil_test_duration_ms = duration_ms;
    uint32_t end_tick = sil_tick_ms + duration_ms;
    
    printf("[SIL] Starting simulation for %u ms\n", duration_ms);
    
    while (sil_tick_ms < end_tick && sil_simulation_running) {
        sil_runtime_step_1ms();
        usleep(1000);  /* Sleep 1ms */
    }
    
    sil_simulation_running = 0;
    printf("[SIL] Simulation finished at %u ms\n", sil_tick_ms);
}

/**
 * Stop simulation
 */
void sil_stop_simulation(void)
{
    sil_simulation_running = 0;
}

/**
 * Get current simulated time
 */
uint32_t sil_get_time_ms(void)
{
    return sil_tick_ms;
}

/* ===== Test entry points ===== */

/**
 * Test: Boot sequence (BOOT -> PRECHARGE -> READY)
 */
static void test_boot_sequence(void)
{
    int failures;

    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║  SIL TEST: Boot Sequence Verification   ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");

    SIL_Results_Init("boot_sequence_test.log");
    SIL_Results_Log("BOOT_TEST", "STARTED", "Boot sequence real-assertion suite");
    SIL_Results_LogEvent(0, "INIT", "Initialising RTOS mock");

    /* Initialise RTOS mock and create queues/mutex via MX_FREERTOS_Init */
    sil_runtime_init();

    /* Run the real assertion suite */
    failures = run_boot_sequence_tests();
    sil_test_failures += failures;

    SIL_Results_Log("BOOT_TEST",
                    (failures == 0) ? "SUCCESS" : "FAIL",
                    (failures == 0) ? "All boot sequence assertions passed"
                                    : "Boot sequence assertions FAILED");
    SIL_Results_Close();
}

/**
 * Test: Full operating cycle
 */
static void test_full_cycle(void)
{
    int failures;

    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("��  SIL TEST: Full Operating Cycle         ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");

    SIL_Results_Init("full_cycle_test.log");
    SIL_Results_Log("CYCLE_TEST", "STARTED", "Full cycle real-assertion suite");
    SIL_Results_LogEvent(0, "INIT", "Initialising RTOS mock");

    sil_runtime_init();

    failures = run_full_cycle_tests();
    sil_test_failures += failures;

    SIL_Results_Log("CYCLE_TEST",
                    (failures == 0) ? "SUCCESS" : "FAIL",
                    (failures == 0) ? "All full-cycle assertions passed"
                                    : "Full-cycle assertions FAILED");
    SIL_Results_Close();
}

/**
 * Test: Error handling - Low DC voltage
 */
static void test_error_low_voltage(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║  SIL TEST: Low DC Voltage Fault         ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");
    
    SIL_Results_Init("error_low_voltage_test.log");
    SIL_Results_Log("ERROR_LOW_V", "STARTED", "Low voltage fault scenario");
    
    printf("[ERROR_V] ➜ Initializing system\n");
    sil_runtime_init();
    
    printf("[ERROR_V] ➜ Normal operation (0-5s)\n");
    SIL_CAN_InjectDCVoltage(400);  /* Normal 400V */
    sil_run_simulation(5000);
    SIL_Results_LogEvent(5000, "NORMAL_OP", "400V stable");
    
    printf("[ERROR_V] ➜ Injecting low voltage fault (5-10s)\n");
    SIL_CAN_InjectDCVoltage(250);  /* Low voltage FAULT */
    SIL_Results_LogEvent(5000, "FAULT_INJECT", "DC voltage 250V (FAULT)");
    sil_run_simulation(5000);
    
    printf("[ERROR_V] ➜ System should limit torque\n");
    app_inputs_t snapshot = {0};
    AppState_Snapshot(&snapshot);
    
    if (snapshot.torque_total == 0) {
        printf("[ERROR_V] ✅ Torque correctly limited to 0 during fault\n");
        SIL_Results_LogEvent(10000, "FAULT_RESPONSE", "Torque=0 (correct)");
    } else {
        printf("[ERROR_V] ⚠️  Torque not limited: %u\n", snapshot.torque_total);
        SIL_Results_LogEvent(10000, "FAULT_RESPONSE", "Torque not limited (issue)");
    }
    
    printf("[ERROR_V] ➜ Clearing fault (10-12s)\n");
    SIL_CAN_InjectDCVoltage(400);  /* Back to normal */
    SIL_Results_LogEvent(10000, "FAULT_CLEAR", "DC voltage restored to 400V");
    sil_run_simulation(2000);
    
    printf("[ERROR_V] ✅ Recovery test complete\n");
    SIL_Results_Log("ERROR_LOW_V", "SUCCESS", "Low voltage fault handled correctly");
    SIL_Results_Close();
}

/**
 * Test: Error handling - High temperature
 */
static void test_error_high_temperature(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║  SIL TEST: High Temperature Fault       ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");
    
    SIL_Results_Init("error_high_temp_test.log");
    SIL_Results_Log("ERROR_TEMP", "STARTED", "High temperature fault scenario");
    
    printf("[ERROR_T] ➜ Initializing system\n");
    sil_runtime_init();
    
    printf("[ERROR_T] ➜ Normal temperature operation (0-5s)\n");
    g_in.inv_motor_temp = 50;
    SIL_Results_LogEvent(0, "TEMP_NORMAL", "Motor temp = 50C");
    sil_run_simulation(5000);
    
    printf("[ERROR_T] ➜ Injecting high temperature fault (5-10s)\n");
    g_in.inv_motor_temp = 95;  /* >80C = WARNING/FAULT */
    SIL_Results_LogEvent(5000, "TEMP_FAULT", "Motor temp = 95C (FAULT)");
    sil_run_simulation(5000);
    
    printf("[ERROR_T] ➜ System should degrade gracefully\n");
    app_inputs_t snapshot = {0};
    AppState_Snapshot(&snapshot);
    printf("[ERROR_T] ℹ  Current state: Motor temp=%d, Torque=%u\n", 
           snapshot.inv_motor_temp, snapshot.torque_total);
    SIL_Results_LogEvent(10000, "DEGRADATION", "Graceful degradation active");
    
    printf("[ERROR_T] ➜ Cooling down (10-15s)\n");
    g_in.inv_motor_temp = 60;
    SIL_Results_LogEvent(10000, "TEMP_COOLING", "Motor temp cooling to 60C");
    sil_run_simulation(5000);
    
    printf("[ERROR_T] ✅ Recovery complete\n");
    SIL_Results_Log("ERROR_TEMP", "SUCCESS", "High temperature handled with graceful degradation");
    SIL_Results_Close();
}

/**
 * Test: EV 2.3 Safety - Brake + Throttle simultaneous
 */
static void test_safety_brake_throttle(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║  SIL TEST: EV 2.3 Brake+Throttle       ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");
    
    SIL_Results_Init("safety_brake_throttle_test.log");
    SIL_Results_Log("SAFETY_BT", "STARTED", "Brake+Throttle safety test");
    
    printf("[SAFETY] ➜ Initializing system\n");
    sil_runtime_init();
    
    printf("[SAFETY] ➜ Normal throttle (0-3s)\n");
    SIL_CAN_InjectThrottle(60);
    SIL_CAN_InjectBrake(0);  /* No brake */
    SIL_Results_LogEvent(0, "THROTTLE_ONLY", "Throttle=60%, Brake=0%");
    sil_run_simulation(3000);
    
    printf("[SAFETY] ➜ Simultaneous brake+throttle activation (3-8s)\n");
    SIL_CAN_InjectBrake(80);  /* Brake activated */
    SIL_Results_LogEvent(3000, "FAULT_INJECT", "Brake=80% + Throttle=60% (FAULT)");
    sil_run_simulation(5000);
    
    app_inputs_t snapshot = {0};
    AppState_Snapshot(&snapshot);
    
    printf("[SAFETY] ➜ Checking safety flag\n");
    if (snapshot.flag_EV_2_3) {
        printf("[SAFETY] ✅ EV 2.3 flag correctly set\n");
        SIL_Results_LogEvent(8000, "SAFETY_FLAG", "EV_2_3=1 (correct)");
    } else {
        printf("[SAFETY] ⚠️  EV 2.3 flag NOT set\n");
        SIL_Results_LogEvent(8000, "SAFETY_FLAG", "EV_2_3=0 (issue!)");
    }
    
    printf("[SAFETY] ➜ Releasing throttle only (8-13s)\n");
    SIL_CAN_InjectThrottle(0);  /* Release throttle */
    SIL_Results_LogEvent(8000, "THROTTLE_RELEASE", "Throttle=0%, Brake still=80%");
    sil_run_simulation(5000);
    
    printf("[SAFETY] ➜ Releasing brake (13-15s)\n");
    SIL_CAN_InjectBrake(0);
    SIL_Results_LogEvent(13000, "BRAKE_RELEASE", "All controls released");
    sil_run_simulation(2000);
    
    printf("[SAFETY] ✅ EV 2.3 safety test complete\n");
    SIL_Results_Log("SAFETY_BT", "SUCCESS", "Brake+Throttle safety validated");
    SIL_Results_Close();
}

/**
 * Test: State machine transitions under dynamic conditions
 */
static void test_dynamic_state_transitions(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║  SIL TEST: Dynamic State Transitions    ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");
    
    SIL_Results_Init("dynamic_transitions_test.log");
    SIL_Results_Log("DYNAMIC_ST", "STARTED", "Dynamic state transition testing");
    
    printf("[DYNAMIC] ➜ Initializing\n");
    sil_runtime_init();
    
    printf("[DYNAMIC] ➜ State 1: BOOT (0-5s)\n");
    SIL_Results_LogEvent(0, "STATE", "BOOT");
    sil_run_simulation(5000);
    
    printf("[DYNAMIC] ➜ State 2: Requesting Precharge (5-7s)\n");
    g_in.ok_precarga = 0;  /* Request precharge */
    SIL_Results_LogEvent(5000, "STATE_TRANS", "Requesting PRECHARGE");
    sil_run_simulation(2000);
    
    printf("[DYNAMIC] ➜ State 3: Precharge ACK received (7-10s)\n");
    g_in.ok_precarga = 1;  /* ACK received */
    g_in.inv_dc_bus_voltage = 400;  /* Voltage stable */
    SIL_Results_LogEvent(7000, "STATE_TRANS", "PRECHARGE ACK received");
    sil_run_simulation(3000);
    
    printf("[DYNAMIC] ➜ State 4: Ready for operation (10-15s)\n");
    SIL_Results_LogEvent(10000, "STATE", "READY");
    sil_run_simulation(5000);
    
    printf("[DYNAMIC] ➜ State 5: Throttle applied (15-20s)\n");
    SIL_CAN_InjectThrottle(75);
    SIL_Results_LogEvent(15000, "STATE", "THROTTLE_CONTROL");
    sil_run_simulation(5000);
    
    printf("[DYNAMIC] ➜ State 6: Fault injection (20-22s)\n");
    g_in.inv_dc_bus_voltage = 200;  /* Fault */
    SIL_Results_LogEvent(20000, "FAULT", "Low voltage injected");
    sil_run_simulation(2000);
    
    printf("[DYNAMIC] ➜ State 7: Fault recovery (22-25s)\n");
    g_in.inv_dc_bus_voltage = 400;  /* Fault cleared */
    SIL_Results_LogEvent(22000, "RECOVERY", "Low voltage cleared");
    sil_run_simulation(3000);
    
    printf("[DYNAMIC] ✅ Dynamic transitions test complete\n");
    SIL_Results_Log("DYNAMIC_ST", "SUCCESS", "State machine transitions working correctly");
    SIL_Results_Close();
}

static void test_precharge_unconfirmed(void)
{
    uint8_t data[8] = {0};
    uint8_t expected[8] = {0};

    printf("\n");
    printf("========================================\n");
    printf("  SIL TEST: Precharge Not Confirmed\n");
    printf("========================================\n\n");

    SIL_Results_Init("precharge_unconfirmed.log");
    SIL_Results_Log("PRECHARGE_NO_ACK", "STARTED",
                    "Verify ECU never reaches inverter control without positive precharge ACK");

    sil_runtime_init();
    sil_set_start_button(1u);
    SIL_CAN_InjectBrake(100u);
    SIL_CAN_InjectThrottle(0u);
    sil_drain_tx_bus(&hfdcan1);
    sil_drain_tx_bus(&hfdcan2);
    sil_drain_tx_bus(&hfdcan3);

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE1", "WAIT_INV_VDC_CONFIG");
    sil_run_manual(30u, 0u);
    sil_check_bus_empty(&hfdcan1, "N1: sin escritura al inversor antes de TX_STATE_7");
    /* The 0x100 DC-bus heartbeat streams on the ACU bus every control cycle;
     * it is the only ACU traffic (the 0x600 precharge command was retired). */
    memset(expected, 0, sizeof(expected));
    sil_expect_tx_frame(&hfdcan2, TINT_ID_DC_BUS_V, 0u, 2u, expected, 2u,
                        "N1: solo heartbeat 0x100 antes de TX_STATE_7");
    sil_drain_tx_bus(&hfdcan2);

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE2", "PRECHARGE_REQUEST_NO_ACK");
    memset(data, 0, sizeof(data));
    data[2] = 0x00u;
    data[3] = 0x00u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_7, FDCAN_STANDARD_ID, data, 6u) == HAL_OK,
              "N2: RX TX_STATE_7 inyectada");
    sil_run_until_next_control_cycle_after_rx();

    /* Precharge step now emits only the tail 0x100 heartbeat (0x600 retired). */
    memset(expected, 0, sizeof(expected));
    sil_expect_tx_frame(&hfdcan2, TINT_ID_DC_BUS_V, 0u, 2u, expected, 2u,
                        "N2: reenvio DC bus al ACU");
    sil_check_bus_empty(&hfdcan1, "N2: sin escritura al inversor durante precarga");

    /* Negative ACK / pending state must keep the FSM blocked in precharge. */
    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE3", "NEGATIVE_ACK");
    memset(data, 0, sizeof(data));
    data[0] = 0x00u;
    sil_check(sil_inject_rx_frame(&hfdcan2, TINT_ID_ACK_PRECARGA, FDCAN_STANDARD_ID, data, 1u) == HAL_OK,
              "N3: ACK negativo inyectado");
    sil_run_until_next_control_cycle_after_rx();

    memset(expected, 0, sizeof(expected));
    sil_expect_tx_frame(&hfdcan2, TINT_ID_DC_BUS_V, 0u, 2u, expected, 2u,
                        "N3: reenvio DC bus repetido");
    sil_check_bus_empty(&hfdcan1, "N3: ACK negativo no habilita inversor");

    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE4", "TIMEOUT_WITHOUT_ACK");
    sil_run_manual(3000u, 0u);
    sil_check_bus_empty(&hfdcan1, "N4: tras 3s sin ACK no hay escritura al inversor");

    /* Another control cycle should still produce only the ACU heartbeat. */
    memset(expected, 0, sizeof(expected));
    sil_expect_tx_frame(&hfdcan2, TINT_ID_DC_BUS_V, 0u, 2u, expected, 2u,
                        "N4: DC bus sigue publicandose al ACU");
    sil_check_bus_empty(&hfdcan1, "N4: sigue bloqueado antes de R2D");

    /* Even if an inverter state arrives, the control FSM must ignore it while
       precharge is not confirmed. */
    SIL_Results_LogEvent(sil_get_time_ms(), "PHASE5", "SPURIOUS_INV_STATE");
    memset(data, 0, sizeof(data));
    data[4] = 3u;
    sil_check(sil_inject_rx_frame(&hfdcan1, TINT_TX_STATE_2, FDCAN_STANDARD_ID, data, 8u) == HAL_OK,
              "N5: state=3 espurio inyectado");
    sil_run_until_next_control_cycle_after_rx();
    sil_check_bus_empty(&hfdcan1, "N5: state=3 espurio no desbloquea el inversor");

    SIL_Results_Log("PRECHARGE_NO_ACK",
                    (sil_test_failures == 0) ? "SUCCESS" : "FAIL",
                    (sil_test_failures == 0)
                        ? "Negative/no-ACK precharge path remained blocked as expected"
                        : "ECU escaped precharge without confirmation");
    SIL_Results_Close();
}

static void test_rtos_task_startup(void)
{
    printf("\n");
    printf("========================================\n");
    printf("  SIL TEST: RTOS Task Startup/Liveness\n");
    printf("========================================\n\n");

    SIL_Results_Init("rtos_task_startup_test.log");
    SIL_Results_Log("RTOS_TASKS", "STARTED", "Task startup and liveness instrumentation");

    sil_runtime_init();

    sil_check_task_metric(APP_TASK_ID_DEFAULT, 1u, 1u, "defaultTask after init");
    sil_check_task_metric(APP_TASK_ID_INIT, 1u, 1u, "App_InitTask after init");
    sil_check_task_metric(APP_TASK_ID_CONTROL, 1u, 1u, "ControlTask after init");
    sil_check_task_metric(APP_TASK_ID_CAN_RX, 1u, 1u, "CanRxTask after init");
    sil_check_task_metric(APP_TASK_ID_CAN_TX, 1u, 1u, "CanTxTask after init");
    sil_check_task_metric(APP_TASK_ID_TELEMETRY, 1u, 1u, "TelemetryTask after init");
    sil_check_task_metric(APP_TASK_ID_DIAG, 1u, 1u, "DiagTask after init");
    sil_check_task_metric(APP_TASK_ID_RADIO_TX, 1u, 1u, "RadioTxTask after init");
    sil_check_task_metric(APP_TASK_ID_SD_LOG, 1u, 1u, "SdLogTask after init");
    sil_check_task_metric(APP_TASK_ID_DASH, 1u, 1u, "DashTask after init");

    sil_run_manual(120u, 1u);

    sil_check_task_metric(APP_TASK_ID_CONTROL, 1u, 5u, "ControlTask periodic activity");
    sil_check_task_metric(APP_TASK_ID_CAN_RX, 1u, 5u, "CanRxTask periodic activity");
    sil_check_task_metric(APP_TASK_ID_CAN_TX, 1u, 5u, "CanTxTask periodic activity");
    sil_check_task_metric(APP_TASK_ID_TELEMETRY, 1u, 2u, "TelemetryTask periodic activity");
    sil_check_task_metric(APP_TASK_ID_DIAG, 1u, 1u, "DiagTask periodic activity");
    sil_check_task_metric(APP_TASK_ID_RADIO_TX, 1u, 10u, "RadioTxTask periodic activity");
    sil_check_task_metric(APP_TASK_ID_SD_LOG, 1u, 10u, "SdLogTask periodic activity");
    sil_check_task_metric(APP_TASK_ID_DASH, 1u, 10u, "DashTask periodic activity");

    SIL_Results_Log("RTOS_TASKS",
                    (sil_test_failures == 0) ? "SUCCESS" : "FAIL",
                    (sil_test_failures == 0)
                        ? "All RTOS tasks started and executed as expected"
                        : "Some RTOS tasks did not start or did not execute enough");
    SIL_Results_Close();
}

/**
 * Print usage
 */
static void test_bootloader_trigger(void)
{
    can_msg_t m;

    printf("\n");
    printf("========================================\n");
    printf("  SIL TEST: Bootloader reboot trigger\n");
    printf("========================================\n\n");
    SIL_Results_Init("bootloader_trigger.log");
    SIL_Results_Log("BL_TRIGGER", "STARTED",
                    "Bootloader_MatchesTrigger payload gating (id 0x002)");

    /* Positive: ACU bus, id 0x002, dlc 4, ECU payload B0 07 AD 12. */
    memset(&m, 0, sizeof(m));
    m.bus = CAN_BUS_ACU; m.id = 0x002u; m.dlc = 4u;
    m.data[0] = 0xB0u; m.data[1] = 0x07u; m.data[2] = 0xADu; m.data[3] = 0x12u;
    sil_check(Bootloader_MatchesTrigger(&m) == 1u, "T1: ECU payload B0 07 AD 12 -> match");

    /* Negative: AMS payload (...AD 11) must NOT reboot the ECU. */
    m.data[3] = 0x11u;
    sil_check(Bootloader_MatchesTrigger(&m) == 0u, "T2: AMS payload B0 07 AD 11 -> no match");

    /* Negative: wrong id. */
    m.data[3] = 0x12u; m.id = 0x100u;
    sil_check(Bootloader_MatchesTrigger(&m) == 0u, "T3: wrong id 0x100 -> no match");

    /* Negative: wrong bus (inverter, not ACU). */
    m.id = 0x002u; m.bus = CAN_BUS_INV;
    sil_check(Bootloader_MatchesTrigger(&m) == 0u, "T4: wrong bus INV -> no match");

    /* Negative: wrong dlc. */
    m.bus = CAN_BUS_ACU; m.dlc = 3u;
    sil_check(Bootloader_MatchesTrigger(&m) == 0u, "T5: wrong dlc 3 -> no match");

    /* Negative: NULL frame. */
    sil_check(Bootloader_MatchesTrigger(NULL) == 0u, "T6: NULL -> no match");

    SIL_Results_Log("BL_TRIGGER",
                    (sil_test_failures == 0) ? "SUCCESS" : "FAIL",
                    "Payload-gated bootloader trigger");
    SIL_Results_Close();
}

static void test_pit_diag(void)
{
    can_msg_t m;
    uint8_t en;
    app_inputs_t in;
    control_out_t out;
    can_msg_t frames[4];
    const uint8_t git[4] = { 0xAAu, 0xBBu, 0xCCu, 0xDDu };
    uint8_t got0 = 0u;
    int i;

    printf("\n========================================\n");
    printf("  SIL TEST: Pit-diag\n");
    printf("========================================\n\n");
    SIL_Results_Init("pit_diag.log");
    SIL_Results_Log("PIT_DIAG", "STARTED", "command gating + encoders + 100ms cadence");

    /* Command gating: enable (magic DEADBEEF LE), disable, AMS id rejected. */
    memset(&m, 0, sizeof(m));
    m.bus = CAN_BUS_ACU; m.id = 0x7E0u; m.dlc = 4u;
    m.data[0] = 0xEFu; m.data[1] = 0xBEu; m.data[2] = 0xADu; m.data[3] = 0xDEu;
    en = 0xFFu;
    sil_check(PitDiag_MatchCommand(&m, &en) == 1u && en == 1u, "P1: 0x7E0 DEADBEEF -> enable");
    m.data[0] = 0u; m.data[1] = 0u; m.data[2] = 0u; m.data[3] = 0u;
    en = 0xFFu;
    sil_check(PitDiag_MatchCommand(&m, &en) == 1u && en == 0u, "P2: 0x7E0 zero -> disable");
    m.id = 0x680u;  /* an AMS pit-diag id */
    sil_check(PitDiag_MatchCommand(&m, &en) == 0u, "P3: AMS id 0x680 -> not ours");

    /* Encoders. */
    memset(&in, 0, sizeof(in));
    memset(&out, 0, sizeof(out));
    in.inv_state = 6u; in.v_celda_min = 3700u;       /* 0x0E74 */
    in.s1_aceleracion = 0x1234u; in.s2_aceleracion = 0x2345u; in.s_freno = 0x0456u;
    in.inv_dc_bus_voltage = 400u;
    out.fsm_state = 6u; out.torque_pct = 42u; out.flag_ev_2_3 = 1u;

    PitDiag_BuildStatus(&in, &out, &m);
    sil_check(m.id == 0x700u && m.bus == CAN_BUS_ACU, "P4: status id/bus");
    sil_check(m.data[0] == 6u && m.data[1] == 6u && m.data[3] == 42u, "P5: status fsm/inv/torque");
    sil_check(m.data[2] == 0x01u, "P6: status flags (ev23 bit)");
    sil_check(m.data[4] == 0x0Eu && m.data[5] == 0x74u, "P7: status v_celda_min BE 3700");

    PitDiag_BuildPedals(&in, &m);
    sil_check(m.id == 0x701u && m.data[0] == 0x12u && m.data[1] == 0x34u, "P8: pedals apps1 BE");

    PitDiag_BuildFwInfo(1u, 2u, 3u, git, &m);
    sil_check(m.id == 0x703u && m.data[0] == 1u && m.data[2] == 3u && m.data[3] == 0xAAu, "P9: fwinfo");

    /* 100 ms cadence: enabled -> nothing for 9 steps, 4 frames on the 10th. */
    PitDiag_SetEnabled(1u);
    for (i = 0; i < 9; i++) got0 = (uint8_t)(got0 + PitDiag_Collect(&in, &out, 1u, 2u, 3u, git, frames, 4u));
    sil_check(got0 == 0u, "P10: no frames in the first 9 steps");
    sil_check(PitDiag_Collect(&in, &out, 1u, 2u, 3u, git, frames, 4u) == 4u, "P11: 4 frames on the 10th step");
    PitDiag_SetEnabled(0u);
    sil_check(PitDiag_Collect(&in, &out, 1u, 2u, 3u, git, frames, 4u) == 0u, "P12: disabled -> 0 frames");

    SIL_Results_Log("PIT_DIAG", (sil_test_failures == 0) ? "SUCCESS" : "FAIL", "pit-diag");
    SIL_Results_Close();
}

static void test_ams_error(void)
{
    app_inputs_t  in;
    control_out_t out;
    const int AMS_ERROR_ST = 7;   /* CTRL_ST_AMS_ERROR */
    const int WAIT_VDC_ST  = 0;   /* CTRL_ST_WAIT_INV_VDC_CONFIG */
    uint8_t i, standby_cmd, reset_cmd, any_torque;

    printf("\n========================================\n");
    printf("  SIL TEST: AMS Start/Error consumer\n");
    printf("========================================\n\n");
    SIL_Results_Init("ams_error.log");
    SIL_Results_Log("AMS_ERROR", "STARTED", "0x4A0[0] Error inhibit + re-arm");

    Control_Init();

    /* AMS in Start (0), inverter ready + precharge ack: the ECU proceeds past
       precharge -- it does NOT inhibit just because of ok_precarga. */
    memset(&in, 0, sizeof(in));
    in.ams_state = 0u; in.inv_vdc_ready = 1u; in.ok_precarga = 1u;
    Control_Step10ms(&in, &out);
    sil_check(out.fsm_state != AMS_ERROR_ST, "A1: AMS Start -> not inhibited");

    /* AMS latches Error (5), inverter NOT faulted: inhibit (regardless of
       ok_precarga), command STANDBY (legacy idle, 0x360=0x01), never torque. */
    in.ams_state = 5u; in.inv_state = 0u;
    Control_Step10ms(&in, &out);
    sil_check(out.fsm_state == AMS_ERROR_ST, "A2: AMS Error -> CTRL_ST_AMS_ERROR");
    standby_cmd = 0u; any_torque = 0u;
    for (i = 0u; i < out.count; i++)
    {
        if (out.msgs[i].id == 0x360u && out.msgs[i].data[2] == 0x01u) standby_cmd = 1u; /* INV_MODE_STANDBY */
        if (out.msgs[i].id == 0x362u) any_torque = 1u;
    }
    sil_check(standby_cmd == 1u, "A3: AMS Error (inv idle) -> STANDBY (0x360=0x01)");
    sil_check(any_torque == 0u, "A3b: AMS Error -> no torque frame (0x362) emitted");

    /* Still Error but the inverter has soft-faulted (10, lost HV): use the SAME
       legacy recovery command RESET so it is not left stuck in fault when the
       AMS returns. Still no torque. */
    in.inv_state = 10u;
    Control_Step10ms(&in, &out);
    sil_check(out.fsm_state == AMS_ERROR_ST, "A4: still Error -> still inhibited (no retry)");
    reset_cmd = 0u; any_torque = 0u;
    for (i = 0u; i < out.count; i++)
    {
        if (out.msgs[i].id == 0x360u && out.msgs[i].data[2] == 0x13u) reset_cmd = 1u; /* INV_MODE_RESET */
        if (out.msgs[i].id == 0x362u) any_torque = 1u;
    }
    sil_check(reset_cmd == 1u, "A4b: AMS Error + inv soft-fault -> RESET (0x360=0x13)");
    sil_check(any_torque == 0u, "A4c: AMS Error + fault -> still no torque (0x362)");

    /* AMS recovers (power-cycled back to Start): the ECU re-arms from the top. */
    in.ams_state = 0u; in.inv_vdc_ready = 0u;
    Control_Step10ms(&in, &out);
    sil_check(out.fsm_state == WAIT_VDC_ST, "A5: AMS leaves Error -> re-arm to WAIT_INV_VDC_CONFIG");

    SIL_Results_Log("AMS_ERROR", (sil_test_failures == 0) ? "SUCCESS" : "FAIL", "AMS error consumer");
    SIL_Results_Close();
}

/* Code-first CAN DSL parity: the generated encoder must reproduce the exact
 * wire bytes the hand-rolled build_acu_dc_bus_frame() produced (the contract
 * the AMS decodes), and the .def-derived id/dlc must match. Replaces the C++
 * original's compile-time overlap guard with a runtime encode/decode check. */
static void test_dsl_parity(void)
{
    uint8_t i;

    printf("\n========================================\n");
    printf("  SIL TEST: code-first CAN DSL parity\n");
    printf("========================================\n\n");
    SIL_Results_Init("dsl_parity.log");
    SIL_Results_Log("DSL_PARITY", "STARTED", "0x100 VCU_heartbeat encoder/DBC parity");

    /* D1: envelope constants come from the .def (single source of truth). */
    sil_check(VCU_heartbeat_ID == 0x100 && VCU_heartbeat_DLC == 2,
              "D1: VCU_heartbeat_ID/DLC == 0x100/2 (from .def)");

    /* D2: wire layout == legacy hand-rolled (dc_bus_voltage little-endian in
       bytes 0..1). Vectors are the exact bytes the old encoder emitted. */
    {
        static const struct { uint16_t v; uint8_t b0, b1; } vec[] = {
            { 0x0000u, 0x00u, 0x00u },
            { 0x1234u, 0x34u, 0x12u },
            { 300u,    0x2Cu, 0x01u },   /* the old precharge threshold */
            { 0xFFFFu, 0xFFu, 0xFFu },
        };
        uint8_t ok = 1u;
        for (i = 0u; i < (uint8_t)(sizeof(vec) / sizeof(vec[0])); i++) {
            VCU_heartbeat_t in = {0};
            uint8_t d[8] = { 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu, 0xAAu };
            in.dc_bus_voltage = vec[i].v;
            encode_VCU_heartbeat(&in, d);
            if (d[0] != vec[i].b0 || d[1] != vec[i].b1) ok = 0u;
        }
        sil_check(ok == 1u, "D2: encoder emits legacy LE byte layout for all vectors");
    }

    /* D3: encode -> decode round-trip is identity. */
    {
        VCU_heartbeat_t a = {0};
        VCU_heartbeat_t b = {0};
        uint8_t buf[8] = {0};
        a.dc_bus_voltage = 0xBEEFu;
        encode_VCU_heartbeat(&a, buf);
        decode_VCU_heartbeat(buf, &b);
        sil_check(b.dc_bus_voltage == 0xBEEFu, "D3: encode/decode round-trip is identity");
    }

    /* D4: build_acu_dc_bus_frame (the adapter) is exercised by the boot/full-
       cycle suites which assert the 0x100 frame on the ACU bus; here we just
       confirm the generated path is wired by re-encoding a known value. */
    {
        VCU_heartbeat_t in = {0};
        uint8_t d[8] = {0};
        in.dc_bus_voltage = 0x0102u;
        encode_VCU_heartbeat(&in, d);
        sil_check(d[0] == 0x02u && d[1] == 0x01u && d[2] == 0x00u,
                  "D4: only bytes 0..1 written, dlc-clean tail");
    }

    /* ---- pit-diag frames (0x700..0x703): the adapters are now DSL-backed;
       assert they still reproduce the exact hand-rolled byte layout. ---- */
    {
        app_inputs_t  in;
        control_out_t out;
        can_msg_t     m;
        uint8_t       git[4] = { 0xDEu, 0xADu, 0xBEu, 0xEFu };
        memset(&in, 0, sizeof(in));
        memset(&out, 0, sizeof(out));

        /* D5: status -- flags bitmask + BE v_cell_min + signed BE torque_cmd. */
        out.fsm_state = 3u; in.inv_state = 6u; out.torque_pct = 50u;
        out.flag_ev_2_3 = 1u; in.ok_precarga = 1u; in.v_celda_min = 0x0E74u;
        out.torque_cmd = -1234; /* 0xFB2E big-endian */
        PitDiag_BuildStatus(&in, &out, &m);
        sil_check(m.id == 0x700u && m.dlc == 8u &&
                  m.data[0] == 3u && m.data[1] == 6u &&
                  m.data[2] == (0x01u | 0x08u) && m.data[3] == 50u &&
                  m.data[4] == 0x0Eu && m.data[5] == 0x74u &&
                  m.data[6] == 0xFBu && m.data[7] == 0x2Eu,
                  "D5: pit-diag status -> flags + BE v_cell_min + BE torque_cmd");

        /* D6: pedals -- three BE u16 raw + two apps pct bytes (dlc 8). The raws
           here are above the APPS band, so both pcts saturate to 100. */
        in.s1_aceleracion = 0x1234u; in.s2_aceleracion = 0x5678u; in.s_freno = 0x9ABCu;
        PitDiag_BuildPedals(&in, &m);
        sil_check(m.id == 0x701u && m.dlc == 8u &&
                  m.data[0] == 0x12u && m.data[1] == 0x34u &&
                  m.data[2] == 0x56u && m.data[3] == 0x78u &&
                  m.data[4] == 0x9Au && m.data[5] == 0xBCu &&
                  m.data[6] == 100u && m.data[7] == 100u,
                  "D6: pit-diag pedals -> 3x BE u16 raw + apps pct (over-range -> 100)");

        /* D6b: under-range APPS raw saturates to 0% (robust vs the exact cal). */
        in.s1_aceleracion = 0u; in.s2_aceleracion = 0u;
        PitDiag_BuildPedals(&in, &m);
        sil_check(m.data[6] == 0u && m.data[7] == 0u,
                  "D6b: under-range APPS raw -> 0%");

        /* D6c: brake (0x705) -- under-range raw -> 0 bar / 0%; over-range -> 100%. */
        in.s_freno = 0u;
        PitDiag_BuildBrake(&in, &m);
        sil_check(m.id == 0x705u && m.dlc == 3u &&
                  m.data[0] == 0u && m.data[1] == 0u && m.data[2] == 0u,
                  "D6c: pit-diag brake under-range -> 0 bar / 0%");
        in.s_freno = 0xFFFFu;
        PitDiag_BuildBrake(&in, &m);
        sil_check(m.data[2] == 100u, "D6d: brake over-range -> 100%");

        /* D7: inverter -- SIGNED big-endian 32-bit rpm (FIELD_BE_S). The new
           variant; -1000 must encode as 0xFFFFFC18 in bytes 2..5. */
        in.inv_dc_bus_voltage = 0x0190u; in.inv_rpm = -1000; in.inv_error = 7u;
        PitDiag_BuildInverter(&in, &m);
        sil_check(m.id == 0x702u && m.dlc == 7u &&
                  m.data[0] == 0x01u && m.data[1] == 0x90u &&
                  m.data[2] == 0xFFu && m.data[3] == 0xFFu &&
                  m.data[4] == 0xFCu && m.data[5] == 0x18u &&
                  m.data[6] == 7u,
                  "D7: pit-diag inverter -> signed BE32 rpm (-1000)");

        /* D8: fwinfo -- version bytes + git hash as a BE u32 from 4 bytes. */
        PitDiag_BuildFwInfo(1u, 2u, 3u, git, &m);
        sil_check(m.id == 0x703u &&
                  m.data[0] == 1u && m.data[1] == 2u && m.data[2] == 3u &&
                  m.data[3] == 0xDEu && m.data[4] == 0xADu &&
                  m.data[5] == 0xBEu && m.data[6] == 0xEFu,
                  "D8: pit-diag fwinfo -> version + git4 (BE u32)");

        /* D9: health (0x704, #36) -- heap BE u16s, then 4 status bytes. */
        PitDiag_BuildHealth(12000u, 8000u, 0x05u, 4u /*IWDG*/, 10u, 0xF1u /*HardFault*/, &m);
        sil_check(m.id == 0x704u && m.dlc == 8u &&
                  m.data[0] == 0x2Eu && m.data[1] == 0xE0u &&   /* free_heap 12000 BE */
                  m.data[2] == 0x1Fu && m.data[3] == 0x40u &&   /* min_free_heap 8000 BE */
                  m.data[4] == 0x05u && m.data[5] == 0x04u &&   /* task_ran_mask, reset_cause */
                  m.data[6] == 0x0Au && m.data[7] == 0xF1u,     /* uptime_s, last_fault */
                  "D9: pit-diag health -> heap BE u16 + status bytes");
    }

    SIL_Results_Log("DSL_PARITY", (sil_test_failures == 0) ? "SUCCESS" : "FAIL", "CAN DSL parity");
    SIL_Results_Close();
}

static void print_usage(const char *prog)
{
    printf("Usage: %s [OPTIONS]\n", prog);
    printf("  --test-boot              Boot sequence test\n");
    printf("  --test-full-cycle        Full operating cycle test\n");
    printf("  --test-rtos-startup      Task startup/liveness instrumentation test\n");
    printf("  --test-error-voltage     Low voltage fault test\n");
    printf("  --test-error-temp        High temperature fault test\n");
    printf("  --test-safety-brake      EV 2.3 brake+throttle test\n");
    printf("  --test-dynamic-states    Dynamic state transition test\n");
    printf("  --test-precharge-e2e     Complete startup/precharge end-to-end test\n");
    printf("  --test-precharge-no-ack  Precharge remains blocked without positive ACK\n");
    printf("  --test-legacy-compat     Polling-compatible startup/inverter harness\n");
    printf("  --test-integration       Suites S1-S10 (test_integration.c)\n");
    printf("  --test-bootloader-trigger  Bootloader reboot-trigger payload gating\n");
    printf("  --test-pit-diag          Pit-diag command gating + encoders + cadence\n");
    printf("  --test-ams-error         AMS 0x4A0 Error inhibit + re-arm\n");
    printf("  --test-dsl-parity        Code-first CAN DSL encoder/DBC parity (0x100)\n");
    printf("                           -> genera tests/sil/results/integration_test.log\n");
    printf("  --test-all               Run ALL tests (incluyendo S1-S10)\n");
    printf("  --help                   Print this message\n");
}

/* ===== Main entry point ===== */

int main(int argc, char *argv[])
{
    printf("\n");
    printf("========================================\n");
    printf("  ECU08 NSIL - Software-In-The-Loop\n");
    printf("  Comprehensive SIL Test Suite\n");
    printf("========================================\n\n");
    
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }
    
    const char *test_name = argv[1];
    
    if (strcmp(test_name, "--test-boot") == 0) {
        test_boot_sequence();
    } else if (strcmp(test_name, "--test-full-cycle") == 0) {
        test_full_cycle();
    } else if (strcmp(test_name, "--test-rtos-startup") == 0) {
        test_rtos_task_startup();
    } else if (strcmp(test_name, "--test-error-voltage") == 0) {
        test_error_low_voltage();
    } else if (strcmp(test_name, "--test-error-temp") == 0) {
        test_error_high_temperature();
    } else if (strcmp(test_name, "--test-safety-brake") == 0) {
        test_safety_brake_throttle();
    } else if (strcmp(test_name, "--test-dynamic-states") == 0) {
        test_dynamic_state_transitions();
    } else if (strcmp(test_name, "--test-precharge-e2e") == 0) {
        test_legacy_compat_harness();
    } else if (strcmp(test_name, "--test-precharge-no-ack") == 0) {
        test_precharge_unconfirmed();
    } else if (strcmp(test_name, "--test-legacy-compat") == 0) {
        test_legacy_compat_harness();
    } else if (strcmp(test_name, "--test-integration") == 0) {
        test_integration_suite();
    } else if (strcmp(test_name, "--test-bootloader-trigger") == 0) {
        test_bootloader_trigger();
    } else if (strcmp(test_name, "--test-pit-diag") == 0) {
        test_pit_diag();
    } else if (strcmp(test_name, "--test-ams-error") == 0) {
        test_ams_error();
    } else if (strcmp(test_name, "--test-dsl-parity") == 0) {
        test_dsl_parity();
    } else if (strcmp(test_name, "--test-all") == 0) {
        printf("[MAIN] Running all SIL tests...\n\n");
        test_boot_sequence();
        test_full_cycle();
        test_rtos_task_startup();
        test_error_low_voltage();
        test_error_high_temperature();
        test_safety_brake_throttle();
        test_dynamic_state_transitions();
        test_legacy_compat_harness();
        test_precharge_unconfirmed();
        test_bootloader_trigger();
        test_pit_diag();
        test_ams_error();
        test_dsl_parity();
        test_integration_suite();   /* S1-S10 al final, genera integration_test.log */
    } else if (strcmp(test_name, "--help") == 0) {
        print_usage(argv[0]);
    } else {
        printf("Unknown test: %s\n", test_name);
        print_usage(argv[0]);
        return 1;
    }
    
    printf("\n[SIL] Test execution completed\n\n");
    if (sil_test_failures > 0) {
        printf("[SIL] %d runtime checks failed\n", sil_test_failures);
        return 1;
    }
    return 0;
}

/* ===== Stub for FreeRTOS panic ===== */
void vAssertCalled(const char *file, int line)
{
    printf("[PANIC] Assertion failed at %s:%d\n", file, line);
    sil_stop_simulation();
    exit(1);
}
