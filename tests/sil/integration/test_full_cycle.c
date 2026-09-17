/**
 * test_full_cycle.c
 * SIL test: Full operating cycle — real assertions
 *
 * Covers throttle-to-torque mapping, EV 2.3 brake safety, telemetry
 * serialization, inverter communication format, dynamic state sequencing,
 * and cell-voltage torque limiting.
 *
 * ADC calibration (from test_integration.h, aligned with Control_ComputeTorque):
 *   s1_pct = saturate((raw - 2050.0) / 9.0)
 *   s2_pct = saturate((raw - 1915.0) / 6.55)
 *   torque  = (s1_pct + s2_pct) / 2  if both > 8%, else 0
 *   if torque < 10 → 0 ; if torque > 90 → 100
 *
 * Key ADC values:
 *   S1:  0% = 2050, 50% = 2500, 100% = 2950
 *   S2:  0% = 1915, 50% = 2243, 100% = 2570
 *   Freno inactive = 0, active = 3500 (> UMBRAL_FRENO_APPS = 3000)
 *
 * Torque limiter (control.c apply_vmin_torque_limit):
 *   v_celda_min >= VMIN_FULL_TORQUE (3500) → no limit
 *   v_celda_min <= VMIN_MIN_TORQUE  (2800) → torque × 0.05
 *   2800 < v_celda_min < 3500       → proportional
 *
 * Inverter mode bytes sent in SETPOINT_1.data[2]:
 *   STANDBY = 0x01, READY = 0x04, TORQUE = 0x06, RESET = 0x13
 */

#include <stdio.h>
#include <string.h>

#include "app_state.h"
#include "control.h"
#include "can.h"
#include "telemetry.h"
#include "cmsis_os2.h"
#include "sil_hal_mocks.h"

/* ── External handles / queues ─────────────────────────────────────────── */
extern FDCAN_HandleTypeDef  hfdcan1;
extern FDCAN_HandleTypeDef  hfdcan2;
extern FDCAN_HandleTypeDef  hfdcan3;
extern osMessageQueueId_t   canRxQueueHandle;
extern osMessageQueueId_t   canTxQueueHandle;

/* ── Protocol IDs ──────────────────────────────────────────────────────── */
#define FC_TX_STATE_2        0x461u
#define FC_TX_STATE_7        0x466u
#define FC_ID_ACK_PRECARGA   0x020u
#define FC_RX_SETPOINT_1     0x360u
#define FC_RX_SETPOINT_3     0x362u
#define FC_MODE_TORQUE       0x06u

/* ── Calibrated ADC constants ──────────────────────────────────────────── */
#define FC_S1_0PCT    2050u
#define FC_S1_50PCT   2500u
#define FC_S1_100PCT  2950u
#define FC_S2_0PCT    1915u
#define FC_S2_50PCT   2243u
#define FC_S2_100PCT  2570u
#define FC_FRENO_OFF  0u
#define FC_FRENO_ON   3500u   /* > UMBRAL_FRENO_APPS(3000) */

/* ── Assertion counters ────────────────────────────────────────────────── */
static int fc_total  = 0;
static int fc_passed = 0;
static int fc_failed = 0;

#define FC_ASSERT_TRUE(cond, name) \
  do { fc_total++; \
       if (!(cond)) { \
           printf("[FAIL] CYCLE :: %s\n", (name)); \
           fc_failed++; return 0; \
       } \
       printf("[PASS] CYCLE :: %s\n", (name)); fc_passed++; \
  } while (0)

#define FC_ASSERT_EQ(actual, expected, name) \
  do { fc_total++; \
       if ((uint32_t)(actual) != (uint32_t)(expected)) { \
           printf("[FAIL] CYCLE :: %s  got=%lu  exp=%lu\n", (name), \
                  (unsigned long)(uint32_t)(actual), \
                  (unsigned long)(uint32_t)(expected)); \
           fc_failed++; return 0; \
       } \
       printf("[PASS] CYCLE :: %s\n", (name)); fc_passed++; \
  } while (0)

#define FC_ASSERT_NE(actual, unexpected, name) \
  do { fc_total++; \
       if ((uint32_t)(actual) == (uint32_t)(unexpected)) { \
           printf("[FAIL] CYCLE :: %s  got=%lu (should differ)\n", (name), \
                  (unsigned long)(uint32_t)(actual)); \
           fc_failed++; return 0; \
       } \
       printf("[PASS] CYCLE :: %s\n", (name)); fc_passed++; \
  } while (0)

#define FC_ASSERT_RANGE(val, lo, hi, name) \
  do { fc_total++; \
       int32_t _v  = (int32_t)(val); \
       int32_t _lo = (int32_t)(lo); \
       int32_t _hi = (int32_t)(hi); \
       if (_v < _lo || _v > _hi) { \
           printf("[FAIL] CYCLE :: %s  got=%ld  range=[%ld,%ld]\n", \
                  (name), (long)_v, (long)_lo, (long)_hi); \
           fc_failed++; return 0; \
       } \
       printf("[PASS] CYCLE :: %s\n", (name)); fc_passed++; \
  } while (0)

/* ── Helper: write to g_in under mutex ────────────────────────────────── */
#define FC_SET_GIN(field, value) \
  do { if (g_inMutex) osMutexAcquire(g_inMutex, osWaitForever); \
       g_in.field = (value); \
       if (g_inMutex) osMutexRelease(g_inMutex); } while (0)

/* ── Helper: inject CAN frame and pump immediately into g_in ───────────── */
static int fc_inject(FDCAN_HandleTypeDef *h, uint32_t id,
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

/* ── Helper: flush control output into HAL TX FIFO ────────────────────── */
static void fc_flush(const control_out_t *out)
{
    uint8_t i;
    for (i = 0; i < out->count; i++)
        CanTx_SendHal(&out->msgs[i]);
}

/* ── Helper: pop one TX frame ──────────────────────────────────────────── */
static int fc_pop(FDCAN_HandleTypeDef *h, can_msg_t *msg)
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
static void fc_drain(FDCAN_HandleTypeDef *h)
{
    can_msg_t dummy;
    while (fc_pop(h, &dummy)) {}
}

/* ── Helper: clean state ───────────────────────────────────────────────── */
static void fc_reset(void)
{
    can_qitem16_t tmp;

    AppState_Init();
    Control_Init();
    SIL_FDCAN_Reset();

    if (canRxQueueHandle)
        while (osMessageQueueGet(canRxQueueHandle, &tmp, NULL, 0) == osOK) {}
    if (canTxQueueHandle)
        while (osMessageQueueGet(canTxQueueHandle,  &tmp, NULL, 0) == osOK) {}

    fc_drain(&hfdcan1);
    fc_drain(&hfdcan2);
    fc_drain(&hfdcan3);
}

/* ── Helper: drive state machine from init to CTRL_ST_ACTIVE ──────────── *
 *  After this call:
 *    - s_state = CTRL_ST_ACTIVE, s_flag_r2d = 1, s_ev23_latched = 0
 *    - TX FIFOs and queues are drained
 */
static void fc_drive_to_active(void)
{
    app_inputs_t  snap;
    control_out_t out;
    uint8_t       data[8];

    fc_reset();

    /* 1. VDC config: inv_vdc_ready=1, dc_bus=0 (no auto-precharge). Also model a
     *    present, running AMS (0x4A0 byte0=3=Run): the precharge gate now
     *    requires a fresh, non-Start AMS status alongside the 0x020 ACK, and the
     *    staleness guard bounces the FSM back to the boot gate without it. */
    memset(data, 0, sizeof(data));
    fc_inject(&hfdcan1, FC_TX_STATE_7, data, 6u);
    { uint8_t ams[8] = {3u, 0, 0, 1u, 0, 0, 0, 0};
      fc_inject(&hfdcan2, 0x4A0u, ams, 8u); }
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);   /* WAIT_VDC→BOOT→WAIT_PRECHARGE_ACK */
    fc_drain(&hfdcan2);

    /* 2. Precharge ACK: data[0]=1 → ok_precarga=1 */
    memset(data, 0, sizeof(data));
    data[0] = 1u;
    fc_inject(&hfdcan2, FC_ID_ACK_PRECARGA, data, 1u);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);   /* WAIT_PRECHARGE_ACK → WAIT_START_BRAKE */
    fc_drain(&hfdcan1); fc_drain(&hfdcan2);

    /* 3. Button + brake (freno < UMBRAL_FRENO_APPS → no EV2.3 risk) → R2D */
    FC_SET_GIN(boton_arranque, 1u);
    FC_SET_GIN(s_freno, 1000u);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);   /* → R2D_DELAY */

    /* 4. Advance past the 2000 ms R2D delay */
    SIL_AdvanceTick(2001u);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);   /* → WAIT_INV_STANDBY */
    fc_drain(&hfdcan1); fc_drain(&hfdcan2);

    /* 5. inv_state=3 → enters ACTIVE */
    memset(data, 0, sizeof(data));
    data[4] = 3u;
    fc_inject(&hfdcan1, FC_TX_STATE_2, data, 8u);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    fc_flush(&out);
    fc_drain(&hfdcan1);   /* discard READY + zero-torque frames */

    /* Release button/brake for clean throttle tests */
    FC_SET_GIN(boton_arranque, 0u);
    FC_SET_GIN(s_freno, 0u);
}

/* ============================================================================
   T1 – Throttle-to-torque mapping (Control_Step10ms path)
   System is in ACTIVE with inv_state=6, v_celda_min=3500 (no limit).
   Verifies out.torque_pct for 0%, 50%, and 100% ADC inputs.
   ========================================================================== */
static int test_throttle_mapping(void)
{
    app_inputs_t  snap;
    control_out_t out;
    uint8_t       data[8];

    printf("\n[CYCLE] T1: throttle-to-torque mapping\n");
    fc_drive_to_active();

    /* Transition to inv_state=6 (torque mode) */
    memset(data, 0, sizeof(data));
    data[4] = 6u;
    fc_inject(&hfdcan1, FC_TX_STATE_2, data, 8u);

    /* Set v_celda_min=3500 (>= VMIN_FULL_TORQUE) so limiter is inactive */
    FC_SET_GIN(v_celda_min, 3500u);

    /* T1.1: 0% throttle → torque_pct = 0 */
    FC_SET_GIN(s1_aceleracion, FC_S1_0PCT);
    FC_SET_GIN(s2_aceleracion, FC_S2_0PCT);
    FC_SET_GIN(s_freno,        FC_FRENO_OFF);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    FC_ASSERT_EQ(out.torque_pct, 0u, "T1.1_zero_throttle_zero_torque");
    fc_flush(&out); fc_drain(&hfdcan1);

    /* T1.2: 50% throttle → torque_pct in [45, 55] */
    FC_SET_GIN(s1_aceleracion, FC_S1_50PCT);
    FC_SET_GIN(s2_aceleracion, FC_S2_50PCT);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    FC_ASSERT_RANGE(out.torque_pct, 45u, 55u, "T1.2_50pct_throttle_torque_range");
    fc_flush(&out); fc_drain(&hfdcan1);

    /* T1.3: 100% throttle → torque_pct = 100 (> 90 saturates to 100) */
    FC_SET_GIN(s1_aceleracion, FC_S1_100PCT);
    FC_SET_GIN(s2_aceleracion, FC_S2_100PCT);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    FC_ASSERT_RANGE(out.torque_pct, 95u, 100u, "T1.3_100pct_throttle_torque_range");
    fc_flush(&out); fc_drain(&hfdcan1);

    return 1;
}

/* ============================================================================
   T2 – EV 2.3 brake safety (Control_ComputeTorque direct path)
   Uses Control_ComputeTorque so the test is independent of state-machine
   position and runs quickly.
   ========================================================================== */
static int test_brake_safety_ev23(void)
{
    app_inputs_t in;
    uint8_t      ev23, t11;
    uint16_t     torque;

    printf("\n[CYCLE] T2: EV 2.3 brake safety\n");

    /* ---- Scenario A: throttle + brake > 25% → EV2.3 must latch ---- */
    Control_Init();   /* resets s_ev23_latched */
    memset(&in, 0, sizeof(in));
    /* s1≈77%, s2≈77% → raw torque≈77; freno=3500 > UMBRAL_FRENO_APPS(3000) */
    in.s1_aceleracion = 2750u;
    in.s2_aceleracion = 2420u;
    in.s_freno        = FC_FRENO_ON;
    in.v_celda_min    = 3500u;   /* no cell-voltage limiting */

    torque = Control_ComputeTorque(&in, &ev23, &t11);
    FC_ASSERT_EQ(ev23,   1u, "T2.A_ev23_latched_throttle_plus_brake");
    FC_ASSERT_EQ(torque, 0u, "T2.A_torque_zero_when_ev23_latched");

    /* ---- Scenario B: latch persists even when brake is released ---- */
    /* Brake released but throttle still high → torque still limited to 0 */
    in.s_freno = FC_FRENO_OFF;
    torque = Control_ComputeTorque(&in, &ev23, &t11);
    FC_ASSERT_EQ(ev23,   1u, "T2.B_ev23_persists_after_brake_release");
    FC_ASSERT_EQ(torque, 0u, "T2.B_torque_still_zero_with_latched_ev23");

    /* ---- Scenario C: latch clears only when throttle drops AND freno low ---- */
    /* Condition to unlatch: s_freno < 3000 AND torque (from ADC) < 5 */
    in.s1_aceleracion = FC_S1_0PCT;   /* 0% ADC */
    in.s2_aceleracion = FC_S2_0PCT;
    in.s_freno        = FC_FRENO_OFF;
    torque = Control_ComputeTorque(&in, &ev23, &t11);
    FC_ASSERT_EQ(ev23,   0u, "T2.C_ev23_cleared_after_full_release");
    FC_ASSERT_EQ(torque, 0u, "T2.C_torque_zero_at_0pct_throttle");

    /* ---- Scenario D: throttle only (no brake) → no latch ---- */
    Control_Init();
    memset(&in, 0, sizeof(in));
    in.s1_aceleracion = 2750u;
    in.s2_aceleracion = 2420u;
    in.s_freno        = FC_FRENO_OFF;   /* no brake */
    in.v_celda_min    = 3500u;

    torque = Control_ComputeTorque(&in, &ev23, &t11);
    FC_ASSERT_EQ(ev23, 0u, "T2.D_no_ev23_latch_without_brake");
    FC_ASSERT_NE(torque, 0u, "T2.D_torque_nonzero_throttle_only");

    /* ---- Scenario E: brake only (no throttle) → no latch ---- */
    Control_Init();
    memset(&in, 0, sizeof(in));
    in.s1_aceleracion = FC_S1_0PCT;
    in.s2_aceleracion = FC_S2_0PCT;
    in.s_freno        = FC_FRENO_ON;
    in.v_celda_min    = 3500u;

    torque = Control_ComputeTorque(&in, &ev23, &t11);
    FC_ASSERT_EQ(ev23,   0u, "T2.E_no_ev23_latch_brake_only");
    FC_ASSERT_EQ(torque, 0u, "T2.E_torque_zero_brake_only_0pct_throttle");

    return 1;
}

/* ============================================================================
   T3 – Telemetry serialization (Telemetry_Build32 / Telemetry_SerializeFrame)
   Verifies byte layout from telemetry.c telemetry_pack_snapshot_compat:
     out32[0]    = inv_state
     out32[1]    = torque_total & 0xFF
     out32[2:3]  = inv_dc_bus_voltage  (LE)
     out32[6:7]  = s1_aceleracion      (LE)
     out32[10:11]= s_freno             (LE)
     out32[12]   = flag_EV_2_3
     out32[14]   = ok_precarga
   And Telemetry_SerializeFrame:
     out32[17]   = frame kind (1=HEARTBEAT, 2=EVENT)
   ========================================================================== */
static int test_telemetry_output(void)
{
    app_inputs_t     in;
    uint8_t          buf32[32];
    telemetry_frame_t frame;

    printf("\n[CYCLE] T3: telemetry serialization\n");
    memset(&in, 0, sizeof(in));

    /* Known values */
    in.inv_state         = 6u;
    in.torque_total      = 50u;
    in.inv_dc_bus_voltage = 400u;   /* 0x0190 → LE: 0x90, 0x01 */
    in.s1_aceleracion    = 0x1234u; /* LE: 0x34, 0x12 */
    in.s_freno           = 0x0A0Bu; /* LE: 0x0B, 0x0A */
    in.flag_EV_2_3       = 1u;
    in.ok_precarga       = 1u;
    in.v_celda_min       = 3500u;   /* LE: 0xAC, 0x0D */

    /* T3.1: Telemetry_Build32 (snapshot-compatible layout) */
    Telemetry_Build32(&in, buf32);
    FC_ASSERT_EQ(buf32[0],  6u,    "T3.1_byte0_inv_state");
    FC_ASSERT_EQ(buf32[1],  50u,   "T3.1_byte1_torque_total");
    /* inv_dc_bus_voltage=400=0x0190 LE */
    FC_ASSERT_EQ(buf32[2],  0x90u, "T3.1_byte2_dc_bus_lo");
    FC_ASSERT_EQ(buf32[3],  0x01u, "T3.1_byte3_dc_bus_hi");
    /* v_celda_min=3500=0x0DAC LE */
    FC_ASSERT_EQ(buf32[4],  0xACu, "T3.1_byte4_vcelda_lo");
    FC_ASSERT_EQ(buf32[5],  0x0Du, "T3.1_byte5_vcelda_hi");
    /* s1_aceleracion=0x1234 LE */
    FC_ASSERT_EQ(buf32[6],  0x34u, "T3.1_byte6_s1_lo");
    FC_ASSERT_EQ(buf32[7],  0x12u, "T3.1_byte7_s1_hi");
    /* s_freno=0x0A0B LE */
    FC_ASSERT_EQ(buf32[10], 0x0Bu, "T3.1_byte10_freno_lo");
    FC_ASSERT_EQ(buf32[11], 0x0Au, "T3.1_byte11_freno_hi");
    FC_ASSERT_EQ(buf32[12], 1u,    "T3.1_byte12_flag_ev23");
    FC_ASSERT_EQ(buf32[14], 1u,    "T3.1_byte14_ok_precarga");

    /* T3.2: SerializeFrame for HEARTBEAT (kind=1) */
    Telemetry_BuildFrame(&in, NULL, &frame);   /* event=NULL → HEARTBEAT */
    FC_ASSERT_EQ(frame.kind, TELEMETRY_FRAME_HEARTBEAT, "T3.2_frame_kind_heartbeat");

    memset(buf32, 0, sizeof(buf32));
    Telemetry_SerializeFrame(&frame, buf32);
    FC_ASSERT_EQ(buf32[16], 1u, "T3.2_byte16_sentinel");
    FC_ASSERT_EQ(buf32[17], (uint8_t)TELEMETRY_FRAME_HEARTBEAT,
                 "T3.2_byte17_frame_kind_heartbeat");

    /* T3.3: SerializeFrame for EVENT (kind=2) */
    telemetry_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.type                = TELEMETRY_EVENT_FAULT;
    ev.previous_inv_state  = 4u;
    ev.current_inv_state   = 10u;
    ev.inv_error           = 0x05u;

    Telemetry_BuildFrame(&in, &ev, &frame);
    FC_ASSERT_EQ(frame.kind, TELEMETRY_FRAME_EVENT, "T3.3_frame_kind_event");

    memset(buf32, 0, sizeof(buf32));
    Telemetry_SerializeFrame(&frame, buf32);
    FC_ASSERT_EQ(buf32[17], (uint8_t)TELEMETRY_FRAME_EVENT,
                 "T3.3_byte17_frame_kind_event");
    FC_ASSERT_EQ(buf32[18], (uint8_t)TELEMETRY_EVENT_FAULT,
                 "T3.3_byte18_event_type_fault");
    FC_ASSERT_EQ(buf32[19], 4u,    "T3.3_byte19_prev_inv_state");
    FC_ASSERT_EQ(buf32[20], 10u,   "T3.3_byte20_curr_inv_state");
    FC_ASSERT_EQ(buf32[21], 0x05u, "T3.3_byte21_inv_error");

    return 1;
}

/* ============================================================================
   T4 – Inverter communication format (CAN frames from Control_Step10ms)
   Drives to ACTIVE with inv_state=6 and 50% throttle, then verifies:
     SETPOINT_1  id=0x360, dlc=3, ide=0, data[2]=0x06 (TORQUE mode)
     SETPOINT_3  id=0x362, dlc=4, ide=0, data[2:3] = legacy torque LE
   ========================================================================== */
static int test_inverter_communication(void)
{
    app_inputs_t  snap;
    control_out_t out;
    can_msg_t     tx;
    uint8_t       data[8];
    uint16_t      exp_torque_pct;
    uint16_t      exp_legacy;
    uint16_t      scaled;
    uint8_t       ev23, t11;

    printf("\n[CYCLE] T4: inverter communication format\n");
    fc_drive_to_active();

    /* Set inv_state=6 (torque mode), 50% throttle, no cell-voltage limiting */
    memset(data, 0, sizeof(data));
    data[4] = 6u;
    fc_inject(&hfdcan1, FC_TX_STATE_2, data, 8u);

    FC_SET_GIN(s1_aceleracion, FC_S1_50PCT);
    FC_SET_GIN(s2_aceleracion, FC_S2_50PCT);
    FC_SET_GIN(s_freno,        FC_FRENO_OFF);
    FC_SET_GIN(v_celda_min,    3500u);

    AppState_Snapshot(&snap);

    /* Compute expected torque independently to avoid hardcoding */
    exp_torque_pct = Control_ComputeTorque(&snap, &ev23, &t11);
    FC_ASSERT_RANGE(exp_torque_pct, 45u, 55u,
                    "T4_precondition_50pct_torque_range");

    /* legacy_torque = ~scaled + 1 where scaled = (pct*240/90) - (2400/90) */
    scaled = exp_torque_pct;
    if (scaled >= 10u)
        scaled = (uint16_t)(((uint32_t)scaled * 240u) / 90u - (2400u / 90u));
    exp_legacy = (uint16_t)(~scaled + 1u);

    /* Run the step and flush to HAL TX FIFO */
    Control_Step10ms(&snap, &out);
    fc_flush(&out);

    /* Verify SETPOINT_1: mode frame */
    FC_ASSERT_TRUE(fc_pop(&hfdcan1, &tx),         "T4_setpoint1_present");
    FC_ASSERT_EQ(tx.id,      FC_RX_SETPOINT_1,    "T4_setpoint1_id_0x360");
    FC_ASSERT_EQ(tx.dlc,     3u,                  "T4_setpoint1_dlc_3");
    FC_ASSERT_EQ(tx.ide,     0u,                  "T4_setpoint1_standard_id");
    FC_ASSERT_EQ(tx.data[2], FC_MODE_TORQUE,       "T4_setpoint1_mode_torque_0x06");

    /* Verify SETPOINT_3: torque command — byte order must be little-endian */
    FC_ASSERT_TRUE(fc_pop(&hfdcan1, &tx),         "T4_setpoint3_present");
    FC_ASSERT_EQ(tx.id,  FC_RX_SETPOINT_3,        "T4_setpoint3_id_0x362");
    FC_ASSERT_EQ(tx.dlc, 4u,                      "T4_setpoint3_dlc_4");
    FC_ASSERT_EQ(tx.ide, 0u,                      "T4_setpoint3_standard_id");
    FC_ASSERT_EQ(tx.data[2], (uint8_t)(exp_legacy & 0xFFu),
                 "T4_setpoint3_torque_lo_byte");
    FC_ASSERT_EQ(tx.data[3], (uint8_t)((exp_legacy >> 8) & 0xFFu),
                 "T4_setpoint3_torque_hi_byte");

    FC_ASSERT_EQ(SIL_FDCAN_GetTxCount(&hfdcan1), 0u,
                 "T4_no_extra_frames_after_torque_cmd");

    return 1;
}

/* ============================================================================
   T5 – Dynamic state machine: throttle sequence
   Verifies torque responds correctly to:
     0% → 100% → brake (EV2.3 latch) → full release (unlatch)
   ========================================================================== */
static int test_dynamic_state_machine(void)
{
    app_inputs_t  snap;
    control_out_t out;
    uint8_t       data[8];

    printf("\n[CYCLE] T5: dynamic state machine sequence\n");
    fc_drive_to_active();

    /* Set inv_state=6, no cell-voltage limiting */
    memset(data, 0, sizeof(data));
    data[4] = 6u;
    fc_inject(&hfdcan1, FC_TX_STATE_2, data, 8u);
    FC_SET_GIN(v_celda_min, 3500u);

    /* Step 1: 0% throttle, no brake */
    FC_SET_GIN(s1_aceleracion, FC_S1_0PCT);
    FC_SET_GIN(s2_aceleracion, FC_S2_0PCT);
    FC_SET_GIN(s_freno,        FC_FRENO_OFF);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    FC_ASSERT_EQ(out.torque_pct,    0u, "T5.1_zero_throttle_zero_torque");
    FC_ASSERT_EQ(out.flag_ev_2_3,   0u, "T5.1_ev23_clear_at_zero");
    fc_flush(&out); fc_drain(&hfdcan1);

    /* Step 2: 100% throttle, no brake */
    FC_SET_GIN(s1_aceleracion, FC_S1_100PCT);
    FC_SET_GIN(s2_aceleracion, FC_S2_100PCT);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    FC_ASSERT_RANGE(out.torque_pct, 95u, 100u, "T5.2_100pct_throttle_max_torque");
    FC_ASSERT_EQ(out.flag_ev_2_3,   0u,        "T5.2_ev23_clear_at_100pct_no_brake");
    fc_flush(&out); fc_drain(&hfdcan1);

    /* Step 3: 100% throttle + hard brake → EV2.3 must latch */
    FC_SET_GIN(s_freno, FC_FRENO_ON);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    FC_ASSERT_EQ(out.flag_ev_2_3, 1u, "T5.3_ev23_latched_throttle_plus_brake");
    FC_ASSERT_EQ(out.torque_pct,  0u, "T5.3_torque_zero_ev23_active");
    fc_flush(&out); fc_drain(&hfdcan1);

    /* Step 4: Release all controls → EV2.3 must unlatch (freno<3000, ADC→0<5%) */
    FC_SET_GIN(s1_aceleracion, FC_S1_0PCT);
    FC_SET_GIN(s2_aceleracion, FC_S2_0PCT);
    FC_SET_GIN(s_freno,        FC_FRENO_OFF);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    FC_ASSERT_EQ(out.flag_ev_2_3, 0u, "T5.4_ev23_cleared_after_full_release");
    FC_ASSERT_EQ(out.torque_pct,  0u, "T5.4_torque_zero_after_release");
    fc_flush(&out); fc_drain(&hfdcan1);

    /* Step 5: Torque is available again after unlatch */
    FC_SET_GIN(s1_aceleracion, FC_S1_50PCT);
    FC_SET_GIN(s2_aceleracion, FC_S2_50PCT);
    AppState_Snapshot(&snap);
    Control_Step10ms(&snap, &out);
    FC_ASSERT_RANGE(out.torque_pct, 45u, 55u, "T5.5_50pct_available_after_unlatch");
    FC_ASSERT_EQ(out.flag_ev_2_3,  0u,        "T5.5_ev23_still_clear");
    /* Sanity: torque always in 0..100 */
    FC_ASSERT_RANGE(out.torque_pct, 0u, 100u, "T5.5_torque_in_valid_range");

    return 1;
}

/* ============================================================================
   T6 – Cell voltage torque limiter (apply_vmin_torque_limit)
   Verifies the three operating regions:
     v_celda_min >= 3500  → full torque (no limit)
     2800 < v_celda < 3500 → proportional limit
     v_celda_min <= 2800  → torque × 0.05 (minimum 5%)
   ========================================================================== */
static int test_cell_voltage_limits(void)
{
    app_inputs_t in;
    uint8_t      ev23, t11;
    uint16_t     torque_full, torque_low, torque_min;

    printf("\n[CYCLE] T6: cell voltage torque limiter\n");
    Control_Init();   /* reset EV2.3 latch */

    /* Common 50% throttle inputs */
    memset(&in, 0, sizeof(in));
    in.s1_aceleracion = FC_S1_50PCT;
    in.s2_aceleracion = FC_S2_50PCT;
    in.s_freno        = FC_FRENO_OFF;

    /* T6.1: Full voltage → no limiting, torque ≈ 50 */
    in.v_celda_min = 3500u;
    torque_full = Control_ComputeTorque(&in, &ev23, &t11);
    FC_ASSERT_RANGE(torque_full, 45u, 55u,
                    "T6.1_full_voltage_torque_50pct_range");

    /* T6.2: Critically low voltage (≤2800) → torque × 0.05 ≈ 2-3 */
    Control_Init();
    in.v_celda_min = 2700u;   /* below VMIN_MIN_TORQUE=2800 → factor 0.05 */
    torque_low = Control_ComputeTorque(&in, &ev23, &t11);
    FC_ASSERT_RANGE(torque_low, 0u, 5u,
                    "T6.2_critical_voltage_torque_severely_limited");
    FC_ASSERT_TRUE(torque_low < torque_full,
                   "T6.2_low_voltage_less_than_full_voltage");

    /* T6.3: Proportional region (2800 < v < 3500) → torque is between
     *       the two extremes */
    Control_Init();
    in.v_celda_min = 3150u;   /* midpoint → roughly 50% of full torque */
    torque_min = Control_ComputeTorque(&in, &ev23, &t11);
    FC_ASSERT_TRUE(torque_min > torque_low,
                   "T6.3_proportional_voltage_greater_than_critical");
    FC_ASSERT_TRUE(torque_min < torque_full,
                   "T6.3_proportional_voltage_less_than_full");
    FC_ASSERT_RANGE(torque_min, 0u, 100u,
                    "T6.3_proportional_torque_in_valid_range");

    /* T6.4: Zero torque from ADC is never modified by limiter (stays 0) */
    Control_Init();
    in.s1_aceleracion = FC_S1_0PCT;
    in.s2_aceleracion = FC_S2_0PCT;
    in.v_celda_min    = 2700u;
    uint16_t t_at_zero = Control_ComputeTorque(&in, &ev23, &t11);
    FC_ASSERT_EQ(t_at_zero, 0u,
                 "T6.4_zero_adc_torque_stays_zero_regardless_of_vcelda");

    return 1;
}

/* ============================================================================
   Entry point
   ========================================================================== */
int run_full_cycle_tests(void)
{
    printf("\n");
    printf("╔════════════════════════════════════════════════════════╗\n");
    printf("║  SIL: FULL CYCLE TEST SUITE  (real assertions)        ║\n");
    printf("╚════════════════════════════════════════════════════════╝\n");

    fc_total = fc_passed = fc_failed = 0;

    test_throttle_mapping();
    test_brake_safety_ev23();
    test_telemetry_output();
    test_inverter_communication();
    test_dynamic_state_machine();
    test_cell_voltage_limits();

    printf("\n[CYCLE] %d/%d passed  |  %d failed  |  %s\n",
           fc_passed, fc_total, fc_failed,
           fc_failed == 0 ? "ALL PASS" : "FAILURES DETECTED");

    return fc_failed;   /* 0 = all pass */
}
