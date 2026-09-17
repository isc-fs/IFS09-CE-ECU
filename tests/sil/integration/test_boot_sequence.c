/**
 * test_boot_sequence.c
 * SIL test: Boot sequence verification — real assertions
 *
 * Drives the control state machine step-by-step and verifies the exact CAN
 * frames emitted at each transition.  Every test function returns 1 on pass,
 * 0 on first assertion failure.
 *
 * Protocol constants verified against can.c / control.c:
 *   ID_ACK_PRECARGA   = 0x020  (ACU bus, data[0]==1 → ok_precarga=1)
 *   TX_STATE_2        = 0x461  (INV bus, data[4]&0x0F = inv_state)
 *   TX_STATE_7        = 0x466  (INV bus, dlc=6, data[2:3] LE = dc_bus_voltage)
 *   RX_SETPOINT_1     = 0x360  (ECU→INV, dlc=3, data[2]=mode, ide=0)
 *   RX_SETPOINT_3     = 0x362  (ECU→INV, dlc=4, data[2:3] LE = legacy torque)
 *   ID_DC_BUS_VOLTAGE = 0x100  (ECU→ACU, dlc=2, ide=0)
 *
 * Precharge complete: ok_precarga (AMS 0x020) ONLY. The ECU no longer
 *   completes precharge on a DC-bus voltage threshold; the AMS owns that
 *   decision (closes AIR+ at >=95% of the measured pack, then sets 0x020).
 *
 * Thresholds (control.c):
 *   UMBRAL_FRENO_ARRANQUE  = 900   (s_freno > 900 for R2D)
 *   R2D_DELAY              = 2000 ms
 */

#include <stdio.h>
#include <string.h>

#include "app_state.h"
#include "control.h"
#include "can.h"
#include "cmsis_os2.h"
#include "sil_hal_mocks.h"

/* ── External handles / queues (defined in hal_impl.c / freertos.c) ───── */
extern FDCAN_HandleTypeDef  hfdcan1;   /* CAN_BUS_INV  */
extern FDCAN_HandleTypeDef  hfdcan2;   /* CAN_BUS_ACU  */
extern FDCAN_HandleTypeDef  hfdcan3;   /* CAN_BUS_DASH */
extern osMessageQueueId_t   canRxQueueHandle;
extern osMessageQueueId_t   canTxQueueHandle;

/* ── Protocol IDs (mirrors can.c / control.c defines) ─────────────────── */
#define BT_TX_STATE_2        0x461u
#define BT_TX_STATE_7        0x466u
#define BT_ID_ACK_PRECARGA   0x020u
#define BT_RX_SETPOINT_1     0x360u
#define BT_RX_SETPOINT_3     0x362u
#define BT_ID_DC_BUS_V       0x100u

/* Inverter mode bytes sent in SETPOINT_1.data[2] */
#define BT_MODE_STANDBY  0x01u
#define BT_MODE_READY    0x04u
#define BT_MODE_TORQUE   0x06u
#define BT_MODE_RESET    0x13u

/* ── Assertion counters ────────────────────────────────────────────────── */
static int bt_total  = 0;
static int bt_passed = 0;
static int bt_failed = 0;

#define BT_ASSERT_TRUE(cond, name) \
  do { bt_total++; \
       if (!(cond)) { \
           printf("[FAIL] BOOT :: %s\n", (name)); \
           bt_failed++; return 0; \
       } \
       printf("[PASS] BOOT :: %s\n", (name)); bt_passed++; \
  } while (0)

#define BT_ASSERT_EQ(actual, expected, name) \
  do { bt_total++; \
       if ((uint32_t)(actual) != (uint32_t)(expected)) { \
           printf("[FAIL] BOOT :: %s  got=%lu  exp=%lu\n", (name), \
                  (unsigned long)(uint32_t)(actual), \
                  (unsigned long)(uint32_t)(expected)); \
           bt_failed++; return 0; \
       } \
       printf("[PASS] BOOT :: %s\n", (name)); bt_passed++; \
  } while (0)

#define BT_ASSERT_NE(actual, unexpected, name) \
  do { bt_total++; \
       if ((uint32_t)(actual) == (uint32_t)(unexpected)) { \
           printf("[FAIL] BOOT :: %s  got=%lu (should differ)\n", (name), \
                  (unsigned long)(uint32_t)(actual)); \
           bt_failed++; return 0; \
       } \
       printf("[PASS] BOOT :: %s\n", (name)); bt_passed++; \
  } while (0)

#define BT_ASSERT_RANGE(val, lo, hi, name) \
  do { bt_total++; \
       int32_t _v  = (int32_t)(val); \
       int32_t _lo = (int32_t)(lo); \
       int32_t _hi = (int32_t)(hi); \
       if (_v < _lo || _v > _hi) { \
           printf("[FAIL] BOOT :: %s  got=%ld  range=[%ld,%ld]\n", \
                  (name), (long)_v, (long)_lo, (long)_hi); \
           bt_failed++; return 0; \
       } \
       printf("[PASS] BOOT :: %s\n", (name)); bt_passed++; \
  } while (0)

/* ── Helper: write to g_in under mutex ────────────────────────────────── */
#define BT_SET_GIN(field, value) \
  do { if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever); \
       g_in.field = (value); \
       if (g_inMutex) osMutexRelease(g_inMutex); } while (0)

/* ── Helper: inject CAN frame and pump immediately into g_in ───────────── */
static int bt_inject(FDCAN_HandleTypeDef *h, uint32_t id,
                     const uint8_t *data, uint8_t dlc)
{
    can_qitem16_t qi;
    can_msg_t     msg;

    if (SIL_FDCAN_InjectRxFrame(h, id, FDCAN_STANDARD_ID, data, dlc) != HAL_OK)
        return 0;
    Can_ISR_PushRxFifo0(h);

    while (osMessageQueueGet(canRxQueueHandle, &qi, NULL, 0) == osOK) {
        CAN_Unpack16(&qi, &msg);
        if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever);
        CanRx_ParseAndUpdate(&msg, &g_in);
        if (g_inMutex) osMutexRelease(g_inMutex);
    }
    return 1;
}

/* ── Helper: flush control output frames into the HAL TX FIFO ─────────── */
static void bt_flush(const control_out_t *out)
{
    uint8_t i;
    for (i = 0; i < out->count; i++)
        CanTx_SendHal(&out->msgs[i]);
}

/* ── Helper: pop one TX frame from a bus; returns 1 if frame present ───── */
static int bt_pop(FDCAN_HandleTypeDef *h, can_msg_t *msg)
{
    FDCAN_TxHeaderTypeDef txh;
    uint8_t d[8] = {0};

    if (SIL_FDCAN_PopTxFrame(h, &txh, d) != HAL_OK) return 0;
    memset(msg, 0, sizeof(*msg));
    msg->id  = txh.Identifier;
    msg->ide = (txh.IdType == FDCAN_EXTENDED_ID) ? 1u : 0u;
    msg->dlc = (uint8_t)((txh.DataLength >> 16) & 0xFu);
    memcpy(msg->data, d, 8);
    return 1;
}

/* ── Helper: drain all TX frames from a bus ─────────────────────────────── */
static void bt_drain(FDCAN_HandleTypeDef *h)
{
    can_msg_t dummy;
    while (bt_pop(h, &dummy)) {}
}

/* ── Helper: clean state before each test ─────────────────────────────── */
static void bt_reset(void)
{
    can_qitem16_t tmp;

    AppState_Init();
    Control_Init();
    SIL_FDCAN_Reset();

    if (canRxQueueHandle)
        while (osMessageQueueGet(canRxQueueHandle, &tmp, NULL, 0) == osOK) {}
    if (canTxQueueHandle)
        while (osMessageQueueGet(canTxQueueHandle,  &tmp, NULL, 0) == osOK) {}

    bt_drain(&hfdcan1);
    bt_drain(&hfdcan2);
    bt_drain(&hfdcan3);

    /* Model a present, running AMS (0x4A0 byte0=3=Run) as the bench default: the
     * precharge gate now requires a fresh, non-Start AMS status alongside the
     * 0x020 ACK, and the staleness guard bounces the FSM back to the boot gate
     * without it. Tests that exercise a different AMS state (e.g. Error=5) just
     * re-inject 0x4A0 to override this default. */
    {
        uint8_t ams[8] = {3u, 0, 0, 1u, 0, 0, 0, 0};
        bt_inject(&hfdcan2, 0x4A0u, ams, 8u);
    }
}

/* ============================================================================
   T1 – Full boot sequence timing
   Verifies each state transition emits the correct CAN frames:
     WAIT_INV_VDC_CONFIG → BOOT/WAIT_PRECHARGE_ACK → WAIT_START_BRAKE
       → R2D_DELAY → WAIT_INV_STANDBY → ACTIVE
   ========================================================================== */
static int test_boot_sequence_timing(void)
{
    app_inputs_t  snap;
    control_out_t out;
    can_msg_t     tx;
    uint8_t       data[8];

    printf("\n[BOOT] T1: full boot sequence timing\n");
    bt_reset();

    /* T1.1: Before VDC config the ECU streams only the 0x100 DC-bus heartbeat
     * (now emitted every control step so the AMS VcuStale watchdog stays fed);
     * no inverter traffic and no precharge command yet. */
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    bt_flush(&out);
    BT_ASSERT_EQ(SIL_FDCAN_GetTxCount(&hfdcan1), 0u,
                 "T1.1_no_inv_tx_before_vdc_config");
    BT_ASSERT_TRUE(bt_pop(&hfdcan2, &tx),
                   "T1.1_acu_heartbeat_before_vdc_config");
    BT_ASSERT_EQ(tx.id, BT_ID_DC_BUS_V, "T1.1_acu_heartbeat_id");
    bt_drain(&hfdcan2);

    /* T1.2: TX_STATE_7 sets inv_vdc_ready=1; dc_bus=0 (no auto-precharge).
     *       ECU should emit the DC bus report frame to ACU. */
    memset(data, 0, sizeof(data));
    BT_ASSERT_TRUE(bt_inject(&hfdcan1, BT_TX_STATE_7, data, 6u),
                   "T1.2_tx_state_7_inject_ok");
    AppState_Snapshot(&snap);
    BT_ASSERT_EQ(snap.inv_vdc_ready, 1u, "T1.2_inv_vdc_ready_set");

    Control_Step10ms(&snap, &out);   /* WAIT_VDC_CONFIG→BOOT→WAIT_PRECHARGE_ACK */
    bt_flush(&out);

    BT_ASSERT_TRUE(bt_pop(&hfdcan2, &tx),    "T1.2_dc_bus_frame_emitted_to_acu");
    BT_ASSERT_EQ(tx.id,  BT_ID_DC_BUS_V,    "T1.2_dc_bus_frame_id");
    BT_ASSERT_EQ(tx.ide, 0u,                "T1.2_dc_bus_frame_ide_standard");
    BT_ASSERT_EQ(tx.dlc, 2u,                "T1.2_dc_bus_frame_dlc");
    BT_ASSERT_EQ(SIL_FDCAN_GetTxCount(&hfdcan1), 0u,
                 "T1.2_no_inv_tx_during_precharge_wait");

    /* T1.3: Precharge ACK (data[0]=1) → ok_precarga=1 */
    memset(data, 0, sizeof(data));
    data[0] = 1u;
    BT_ASSERT_TRUE(bt_inject(&hfdcan2, BT_ID_ACK_PRECARGA, data, 1u),
                   "T1.3_ack_inject_ok");
    AppState_Snapshot(&snap);
    BT_ASSERT_EQ(snap.ok_precarga, 1u, "T1.3_ok_precarga_set_after_ack");

    /* T1.4: With ok_precarga=1, WAIT_PRECHARGE_ACK advances to WAIT_START_BRAKE */
    Control_Step10ms(&snap, &out);
    bt_drain(&hfdcan1);
    bt_drain(&hfdcan2);

    /* T1.5: Button + brake → R2D_DELAY; rtds_active must be asserted */
    BT_SET_GIN(boton_arranque, 1u);
    BT_SET_GIN(s_freno, 1000u);   /* 1000 > UMBRAL_FRENO_ARRANQUE(900) */
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    BT_ASSERT_EQ(out.rtds_active, 1u, "T1.5_rtds_active_on_r2d_entry");

    /* T1.6: At 1500 ms (< 2000 ms R2D timeout) still in R2D_DELAY */
    SIL_AdvanceTick(1500u);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    BT_ASSERT_EQ(out.rtds_active, 1u, "T1.6_rtds_active_before_2s");
    bt_flush(&out);
    BT_ASSERT_EQ(SIL_FDCAN_GetTxCount(&hfdcan1), 0u,
                 "T1.6_no_inv_cmd_during_r2d");

    /* T1.7: After > 2000 ms total R2D_DELAY exits to WAIT_INV_STANDBY;
     *       no INV frames until inv_state=3 is received. */
    SIL_AdvanceTick(600u);   /* cumulative > 2000 ms */
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    bt_flush(&out);
    BT_ASSERT_EQ(SIL_FDCAN_GetTxCount(&hfdcan1), 0u,
                 "T1.7_no_inv_cmd_in_wait_standby");

    /* T1.8: inv_state=3 → enters ACTIVE, emits READY(0x04) + zero torque */
    memset(data, 0, sizeof(data));
    data[4] = 3u;
    BT_ASSERT_TRUE(bt_inject(&hfdcan1, BT_TX_STATE_2, data, 8u),
                   "T1.8_inv_state3_inject");
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    bt_flush(&out);

    BT_ASSERT_TRUE(bt_pop(&hfdcan1, &tx),    "T1.8_setpoint1_ready_present");
    BT_ASSERT_EQ(tx.id,        BT_RX_SETPOINT_1, "T1.8_setpoint1_id");
    BT_ASSERT_EQ(tx.dlc,       3u,               "T1.8_setpoint1_dlc");
    BT_ASSERT_EQ(tx.data[2],   BT_MODE_READY,    "T1.8_setpoint1_mode_ready_0x04");

    BT_ASSERT_TRUE(bt_pop(&hfdcan1, &tx),    "T1.8_setpoint3_zero_torque_present");
    BT_ASSERT_EQ(tx.id,        BT_RX_SETPOINT_3, "T1.8_setpoint3_id");
    BT_ASSERT_EQ(tx.dlc,       4u,               "T1.8_setpoint3_dlc");
    BT_ASSERT_EQ(tx.data[2],   0u,               "T1.8_zero_torque_byte2");
    BT_ASSERT_EQ(tx.data[3],   0u,               "T1.8_zero_torque_byte3");

    BT_ASSERT_EQ(SIL_FDCAN_GetTxCount(&hfdcan1), 0u,
                 "T1.8_no_extra_frames_after_state3");

    return 1;
}

/* ============================================================================
   T2 – Precharge ACK handling
   Verifies the parser correctly sets ok_precarga based on data[0], and that
   frames on the wrong bus are ignored.
   ========================================================================== */
static int test_precharge_ack(void)
{
    app_inputs_t snap;
    uint8_t      data[8];

    printf("\n[BOOT] T2: precharge ACK handling\n");
    bt_reset();

    /* T2.1: Positive ACK data[0]=1 → ok_precarga=1 */
    memset(data, 0, sizeof(data));
    data[0] = 1u;
    BT_ASSERT_TRUE(bt_inject(&hfdcan2, BT_ID_ACK_PRECARGA, data, 1u),
                   "T2.1_positive_ack_inject_ok");
    AppState_Snapshot(&snap);
    BT_ASSERT_EQ(snap.ok_precarga, 1u, "T2.1_ok_precarga_set_to_1");

    /* T2.2: Negative ACK data[0]=0 → ok_precarga=0 */
    data[0] = 0u;
    BT_ASSERT_TRUE(bt_inject(&hfdcan2, BT_ID_ACK_PRECARGA, data, 1u),
                   "T2.2_negative_ack_inject_ok");
    AppState_Snapshot(&snap);
    BT_ASSERT_EQ(snap.ok_precarga, 0u, "T2.2_ok_precarga_cleared_by_nack");

    /* T2.3: Same frame on INV bus must be ignored (parser filters by bus) */
    data[0] = 1u;
    BT_ASSERT_TRUE(bt_inject(&hfdcan1, BT_ID_ACK_PRECARGA, data, 1u),
                   "T2.3_wrong_bus_inject_ok");
    AppState_Snapshot(&snap);
    BT_ASSERT_EQ(snap.ok_precarga, 0u,
                 "T2.3_ok_precarga_unchanged_on_wrong_bus");

    /* T2.4: Re-confirm positive ACK after negative */
    data[0] = 1u;
    bt_inject(&hfdcan2, BT_ID_ACK_PRECARGA, data, 1u);
    AppState_Snapshot(&snap);
    BT_ASSERT_EQ(snap.ok_precarga, 1u, "T2.4_ok_precarga_restored");

    return 1;
}

/* ============================================================================
   T3 – Inverter state machine output
   After driving to ACTIVE, verifies the exact frames emitted for each
   inverter state: state=3 (ready), state=4 (idle), state=10 (soft fault),
   state=11 (hard fault).
   ========================================================================== */
static int test_inverter_states(void)
{
    app_inputs_t  snap;
    control_out_t out;
    can_msg_t     tx;
    uint8_t       data[8];

    printf("\n[BOOT] T3: inverter state machine output\n");
    bt_reset();

    /* ---- Drive to ACTIVE state ---- */
    memset(data, 0, sizeof(data));
    bt_inject(&hfdcan1, BT_TX_STATE_7, data, 6u);   /* vdc_ready=1, dc_bus=0 */
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    bt_drain(&hfdcan2);

    memset(data, 0, sizeof(data));
    data[0] = 1u;                                    /* ok_precarga=1 */
    bt_inject(&hfdcan2, BT_ID_ACK_PRECARGA, data, 1u);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    bt_drain(&hfdcan1); bt_drain(&hfdcan2);

    BT_SET_GIN(boton_arranque, 1u);
    BT_SET_GIN(s_freno, 1000u);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);                   /* → R2D_DELAY */

    SIL_AdvanceTick(2001u);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);                   /* → WAIT_INV_STANDBY */
    bt_drain(&hfdcan1); bt_drain(&hfdcan2);

    /* T3.1: inv_state=3 → ACTIVE: SETPOINT_1(READY=0x04) + SETPOINT_3(zero) */
    memset(data, 0, sizeof(data));
    data[4] = 3u;
    bt_inject(&hfdcan1, BT_TX_STATE_2, data, 8u);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    bt_flush(&out);

    BT_ASSERT_TRUE(bt_pop(&hfdcan1, &tx),            "T3.1_state3_setpoint1_present");
    BT_ASSERT_EQ(tx.id,      BT_RX_SETPOINT_1,       "T3.1_state3_setpoint1_id");
    BT_ASSERT_EQ(tx.data[2], BT_MODE_READY,           "T3.1_state3_mode_ready");
    BT_ASSERT_TRUE(bt_pop(&hfdcan1, &tx),            "T3.1_state3_setpoint3_present");
    BT_ASSERT_EQ(tx.id,      BT_RX_SETPOINT_3,       "T3.1_state3_setpoint3_id");
    BT_ASSERT_EQ(tx.data[2], 0u,                     "T3.1_state3_zero_torque");
    BT_ASSERT_EQ(SIL_FDCAN_GetTxCount(&hfdcan1), 0u, "T3.1_state3_no_extra_frames");

    /* T3.2: inv_state=4 → TORQUE mode(0x06) + zero torque */
    memset(data, 0, sizeof(data));
    data[4] = 4u;
    bt_inject(&hfdcan1, BT_TX_STATE_2, data, 8u);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    bt_flush(&out);

    BT_ASSERT_TRUE(bt_pop(&hfdcan1, &tx),            "T3.2_state4_setpoint1_present");
    BT_ASSERT_EQ(tx.data[2], BT_MODE_TORQUE,         "T3.2_state4_mode_torque");
    BT_ASSERT_TRUE(bt_pop(&hfdcan1, &tx),            "T3.2_state4_setpoint3_present");
    BT_ASSERT_EQ(tx.id,      BT_RX_SETPOINT_3,       "T3.2_state4_setpoint3_id");
    BT_ASSERT_EQ(tx.data[2], 0u,                     "T3.2_state4_zero_torque");
    BT_ASSERT_EQ(SIL_FDCAN_GetTxCount(&hfdcan1), 0u, "T3.2_state4_no_extra_frames");

    /* T3.3: Soft fault state=10 → RESET(0x13) + RESET(0x13) + STANDBY(0x01) */
    memset(data, 0, sizeof(data));
    data[4] = 10u;
    data[2] = 0x03u;   /* inv_error payload */
    bt_inject(&hfdcan1, BT_TX_STATE_2, data, 8u);
    AppState_Snapshot(&snap);
    BT_ASSERT_EQ(snap.inv_state,  10u, "T3.3_inv_state10_parsed");
    BT_ASSERT_EQ(snap.inv_error, 0x03u, "T3.3_inv_error_parsed");

    Control_Step10ms(&snap, &out);
    bt_flush(&out);

    BT_ASSERT_TRUE(bt_pop(&hfdcan1, &tx),            "T3.3_state10_reset1_present");
    BT_ASSERT_EQ(tx.data[2], BT_MODE_RESET,          "T3.3_state10_reset1_mode");
    BT_ASSERT_TRUE(bt_pop(&hfdcan1, &tx),            "T3.3_state10_reset2_present");
    BT_ASSERT_EQ(tx.data[2], BT_MODE_RESET,          "T3.3_state10_reset2_mode");
    BT_ASSERT_TRUE(bt_pop(&hfdcan1, &tx),            "T3.3_state10_standby_present");
    BT_ASSERT_EQ(tx.data[2], BT_MODE_STANDBY,        "T3.3_state10_standby_mode");
    BT_ASSERT_EQ(SIL_FDCAN_GetTxCount(&hfdcan1), 0u, "T3.3_state10_no_extra_frames");

    /* T3.4: Hard fault state=11 → RESET(0x13) + STANDBY(0x01) */
    memset(data, 0, sizeof(data));
    data[4] = 11u;
    bt_inject(&hfdcan1, BT_TX_STATE_2, data, 8u);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    bt_flush(&out);

    BT_ASSERT_TRUE(bt_pop(&hfdcan1, &tx),            "T3.4_state11_reset_present");
    BT_ASSERT_EQ(tx.data[2], BT_MODE_RESET,          "T3.4_state11_reset_mode");
    BT_ASSERT_TRUE(bt_pop(&hfdcan1, &tx),            "T3.4_state11_standby_present");
    BT_ASSERT_EQ(tx.data[2], BT_MODE_STANDBY,        "T3.4_state11_standby_mode");
    BT_ASSERT_EQ(SIL_FDCAN_GetTxCount(&hfdcan1), 0u, "T3.4_state11_no_extra_frames");

    return 1;
}

/* ============================================================================
   T4 – Precharge gates on the AMS ACK (0x020), never on DC bus voltage
   The precharge-complete decision belongs to the AMS: it closes AIR+ at >=95%
   of the measured pack and then publishes ok_precarga on 0x020. The ECU must
   NOT complete precharge on any DC-bus voltage on its own — not even a high
   one. (TX_STATE_7 / 0x466 little-endian parsing is still exercised here.)
   ========================================================================== */
static int test_dc_voltage_handling(void)
{
    app_inputs_t  snap;
    control_out_t out;
    uint8_t       data[8];

    printf("\n[BOOT] T4: precharge gates on AMS ACK, not DC bus voltage\n");

    /* T4.1: a HIGH bus (400 V) WITHOUT an ACK must NOT complete precharge.
     *       Button+brake must not advance to R2D, and no INV frames go out. */
    bt_reset();
    memset(data, 0, sizeof(data));
    /* 400 = 0x0190 → LE: data[2]=0x90, data[3]=0x01 */
    data[2] = 0x90u;
    data[3] = 0x01u;
    bt_inject(&hfdcan1, BT_TX_STATE_7, data, 6u);
    AppState_Snapshot(&snap);
    BT_ASSERT_EQ(snap.inv_dc_bus_voltage, 400u, "T4.1_dc_bus_400_parsed_le");
    BT_ASSERT_EQ(snap.inv_vdc_ready,      1u,   "T4.1_vdc_ready_set");

    Control_Step10ms(&snap, &out);   /* BOOT → WAIT_PRECHARGE_ACK (no ACK yet) */
    bt_drain(&hfdcan2);              /* discard the DC bus heartbeat */

    BT_SET_GIN(boton_arranque, 1u);
    BT_SET_GIN(s_freno, 1000u);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);   /* stays in WAIT_PRECHARGE_ACK: 400 V alone is not enough */
    bt_flush(&out);
    BT_ASSERT_EQ(SIL_FDCAN_GetTxCount(&hfdcan1), 0u,
                 "T4.1_no_inv_tx_at_400v_without_ack");
    BT_ASSERT_EQ(out.rtds_active, 0u,
                 "T4.1_no_rtds_at_400v_without_ack");

    /* T4.2: the AMS ACK (0x020 data[0]=1) arrives → precharge completes and the
     *       already-held button+brake advance to R2D. */
    memset(data, 0, sizeof(data));
    data[0] = 1u;
    bt_inject(&hfdcan2, BT_ID_ACK_PRECARGA, data, 1u);
    AppState_Snapshot(&snap);
    BT_ASSERT_EQ(snap.ok_precarga, 1u, "T4.2_ok_precarga_set_after_ack");

    Control_Step10ms(&snap, &out);   /* WAIT_PRECHARGE_ACK → WAIT_START_BRAKE → R2D_DELAY */
    BT_ASSERT_EQ(out.rtds_active, 1u,
                 "T4.2_rtds_active_after_ack");

    /* T4.3: a LOW bus (200 V) WITH the ACK still completes — proving the gate
     *       is the ACK alone, independent of any voltage threshold. */
    bt_reset();
    memset(data, 0, sizeof(data));
    /* 200 = 0x00C8 → LE: data[2]=0xC8, data[3]=0x00 */
    data[2] = 0xC8u;
    data[3] = 0x00u;
    bt_inject(&hfdcan1, BT_TX_STATE_7, data, 6u);
    memset(data, 0, sizeof(data));
    data[0] = 1u;
    bt_inject(&hfdcan2, BT_ID_ACK_PRECARGA, data, 1u);
    BT_SET_GIN(boton_arranque, 1u);
    BT_SET_GIN(s_freno, 1000u);
    AppState_Snapshot(&snap);
    BT_ASSERT_EQ(snap.inv_dc_bus_voltage, 200u, "T4.3_dc_bus_200_parsed_le");
    BT_ASSERT_EQ(snap.ok_precarga,        1u,   "T4.3_ok_precarga_set");

    Control_Step10ms(&snap, &out);   /* ACK completes precharge even at 200 V → R2D */
    BT_ASSERT_EQ(out.rtds_active, 1u,
                 "T4.3_rtds_active_low_bus_with_ack");

    return 1;
}

/* ============================================================================
   Entry point
   ========================================================================== */
int run_boot_sequence_tests(void)
{
    printf("\n");
    printf("╔════════════════════════════════════════════════════════╗\n");
    printf("║  SIL: BOOT SEQUENCE TEST SUITE  (real assertions)     ║\n");
    printf("╚════════════════════════════════════════════════════════╝\n");

    bt_total = bt_passed = bt_failed = 0;

    test_boot_sequence_timing();
    test_precharge_ack();
    test_inverter_states();
    test_dc_voltage_handling();

    printf("\n[BOOT] %d/%d passed  |  %d failed  |  %s\n",
           bt_passed, bt_total, bt_failed,
           bt_failed == 0 ? "ALL PASS" : "FAILURES DETECTED");

    return bt_failed;   /* 0 = all pass */
}
