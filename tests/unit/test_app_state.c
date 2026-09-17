#include "unity.h"
#include "app_state.h"
#include "mocks.h"
#include <string.h>

TEST_GROUP(AppState);

void setUp(void)
{
    /* Reset global state */
    mock_tick_reset();
    AppState_Init();
}

void tearDown(void)
{
    /* Cleanup */
}

/**
 * UT-100: AppState_Init() initializes correctly
 */
TEST(AppState, init_sets_default_values)
{
    /* Arrange */
    AppState_Init();
    app_inputs_t snapshot = {0};
    
    /* Act */
    AppState_Snapshot(&snapshot);
    
    /* Assert: defaults aligned with legacy startup assumptions */
    TEST_ASSERT_EQUAL_INT(0, snapshot.s1_aceleracion);
    TEST_ASSERT_EQUAL_INT(0, snapshot.s2_aceleracion);
    TEST_ASSERT_EQUAL_INT(0, snapshot.s_freno);
    TEST_ASSERT_EQUAL_INT(0, snapshot.boton_arranque);
    TEST_ASSERT_EQUAL_INT(0, snapshot.inv_state);
    TEST_ASSERT_EQUAL_INT(0, snapshot.ok_precarga);
    TEST_ASSERT_EQUAL_INT(0, snapshot.flag_EV_2_3);
    TEST_ASSERT_EQUAL_INT(0, snapshot.flag_T11_8_9);
    TEST_ASSERT_EQUAL_INT(3600, snapshot.v_celda_min);
}

/**
 * UT-101: AppState_Snapshot() retrieves current state
 */
TEST(AppState, snapshot_retrieves_state)
{
    /* Arrange */
    AppState_Init();
    g_in.s1_aceleracion = 2000;
    g_in.s2_aceleracion = 2050;
    g_in.inv_dc_bus_voltage = 400;
    g_in.ok_precarga = 1;
    
    app_inputs_t snapshot = {0};
    
    /* Act */
    AppState_Snapshot(&snapshot);
    
    /* Assert */
    TEST_ASSERT_EQUAL_INT(2000, snapshot.s1_aceleracion);
    TEST_ASSERT_EQUAL_INT(2050, snapshot.s2_aceleracion);
    TEST_ASSERT_EQUAL_INT(400, snapshot.inv_dc_bus_voltage);
    TEST_ASSERT_EQUAL_INT(1, snapshot.ok_precarga);
}

/**
 * UT-102: AppState_Snapshot() handles NULL gracefully
 */
TEST(AppState, snapshot_null_handling)
{
    /* Arrange */
    AppState_Init();
    
    /* Act - Should not crash */
    AppState_Snapshot(NULL);
    
    /* Assert - No crash is success */
    TEST_ASSERT_EQUAL_INT(0, 0);
}

/**
 * UT-103: State updates are isolated between snapshots
 */
TEST(AppState, state_isolation_between_snapshots)
{
    /* Arrange */
    AppState_Init();
    app_inputs_t snap1 = {0};
    app_inputs_t snap2 = {0};
    
    /* Act (1st snapshot) */
    g_in.torque_total = 50;
    AppState_Snapshot(&snap1);
    
    /* Act (modify state) */
    g_in.torque_total = 100;
    
    /* Act (2nd snapshot) */
    AppState_Snapshot(&snap2);
    
    /* Assert: snapshots should show different values */
    TEST_ASSERT_EQUAL_INT(50, snap1.torque_total);
    TEST_ASSERT_EQUAL_INT(100, snap2.torque_total);
}

/**
 * UT-104: Multiple state field updates consistency
 */
TEST(AppState, consistent_multi_field_update)
{
    /* Arrange */
    AppState_Init();
    app_inputs_t snapshot = {0};
    
    /* Act - Simulate multiple sensor reads */
    g_in.s1_aceleracion = 2100;
    g_in.s2_aceleracion = 2200;
    g_in.s_freno = 1500;
    g_in.inv_dc_bus_voltage = 420;
    g_in.inv_motor_temp = 45;
    g_in.inv_igbt_temp = 40;
    
    AppState_Snapshot(&snapshot);
    
    /* Assert: all fields read consistently */
    TEST_ASSERT_EQUAL_INT(2100, snapshot.s1_aceleracion);
    TEST_ASSERT_EQUAL_INT(2200, snapshot.s2_aceleracion);
    TEST_ASSERT_EQUAL_INT(1500, snapshot.s_freno);
    TEST_ASSERT_EQUAL_INT(420, snapshot.inv_dc_bus_voltage);
    TEST_ASSERT_EQUAL_INT(45, snapshot.inv_motor_temp);
    TEST_ASSERT_EQUAL_INT(40, snapshot.inv_igbt_temp);
}

/**
 * UT-105: Flag updates (EV 2.3 and T11.8.9)
 */
TEST(AppState, safety_flags_update)
{
    /* Arrange */
    AppState_Init();
    app_inputs_t snapshot = {0};
    
    /* Act - Set safety flags */
    g_in.flag_EV_2_3 = 1;      /* EV 2.3 violation detected */
    g_in.flag_T11_8_9 = 1;     /* T11.8.9 violation detected */
    
    AppState_Snapshot(&snapshot);
    
    /* Assert */
    TEST_ASSERT_EQUAL_INT(1, snapshot.flag_EV_2_3);
    TEST_ASSERT_EQUAL_INT(1, snapshot.flag_T11_8_9);
}

/**
 * UT-106: Inverter state tracking
 */
TEST(AppState, inverter_state_tracking)
{
    /* Arrange */
    AppState_Init();
    app_inputs_t snapshot = {0};
    
    /* Act - Simulate inverter state changes */
    uint8_t states[] = {0, 1, 2, 3, 4};  /* Different inverter states */
    
    for (int i = 0; i < 5; i++) {
        g_in.inv_state = states[i];
        AppState_Snapshot(&snapshot);
        
        /* Assert */
        TEST_ASSERT_EQUAL_INT(states[i], snapshot.inv_state);
    }
}

/**
 * UT-107: Temperature boundary conditions
 */
TEST(AppState, temperature_boundary_conditions)
{
    /* Arrange */
    AppState_Init();
    app_inputs_t snapshot = {0};
    
    /* Act - Test temperature extremes */
    struct {
        int16_t motor_temp;
        int16_t igbt_temp;
        int16_t air_temp;
        const char *desc;
    } cases[] = {
        {-40, -40, -40, "Cold"},
        {0, 0, 0, "Zero"},
        {55, 50, 45, "Normal"},
        {100, 90, 85, "Hot"},
        {127, 127, 127, "Max positive"},
        {-127, -127, -127, "Max negative"}
    };
    
    for (int i = 0; i < 6; i++) {
        g_in.inv_motor_temp = cases[i].motor_temp;
        g_in.inv_igbt_temp = cases[i].igbt_temp;
        g_in.inv_air_temp = cases[i].air_temp;
        
        AppState_Snapshot(&snapshot);
        
        TEST_ASSERT_EQUAL_INT(cases[i].motor_temp, snapshot.inv_motor_temp);
        TEST_ASSERT_EQUAL_INT(cases[i].igbt_temp, snapshot.inv_igbt_temp);
        TEST_ASSERT_EQUAL_INT(cases[i].air_temp, snapshot.inv_air_temp);
    }
}

/**
 * UT-108: RPM range validation
 */
TEST(AppState, rpm_range_tracking)
{
    /* Arrange */
    AppState_Init();
    app_inputs_t snapshot = {0};
    
    /* Act - Simulate RPM changes */
    uint16_t rpms[] = {0, 2000, 5000, 8000, 12000};
    
    for (int i = 0; i < 5; i++) {
        g_in.inv_rpm = rpms[i];
        AppState_Snapshot(&snapshot);
        
        /* Assert */
        TEST_ASSERT_EQUAL_INT(rpms[i], snapshot.inv_rpm);
    }
}

/**
 * UT-109: Torque accumulation tracking
 */
TEST(AppState, torque_accumulation)
{
    /* Arrange */
    AppState_Init();
    app_inputs_t snapshot = {0};
    
    /* Act - Simulate torque changes */
    g_in.torque_total = 0;
    AppState_Snapshot(&snapshot);
    TEST_ASSERT_EQUAL_INT(0, snapshot.torque_total);
    
    g_in.torque_total = 50;
    AppState_Snapshot(&snapshot);
    TEST_ASSERT_EQUAL_INT(50, snapshot.torque_total);
    
    g_in.torque_total = 100;
    AppState_Snapshot(&snapshot);
    TEST_ASSERT_EQUAL_INT(100, snapshot.torque_total);
    
    /* Assert - Multiple reads in sequence */
    TEST_ASSERT_EQUAL_INT(100, snapshot.torque_total);
}

/**
 * UT-110: All fields synchronized in snapshot
 */
TEST(AppState, all_fields_synchronized)
{
    /* Arrange */
    AppState_Init();
    app_inputs_t snapshot = {0};
    
    /* Act - Set complex state */
    app_inputs_t expected = {
        .s1_aceleracion = 2500,
        .s2_aceleracion = 2550,
        .s_freno = 2000,
        .boton_arranque = 1,
        .inv_state = 3,
        .inv_dc_bus_voltage = 410,
        .inv_motor_temp = 52,
        .inv_igbt_temp = 48,
        .inv_air_temp = 35,
        .inv_rpm = 9500,
        .v_celda_min = 3000,
        .ok_precarga = 1,
        .flag_EV_2_3 = 0,
        .flag_T11_8_9 = 0,
        .torque_total = 75
    };
    
    memcpy(&g_in, &expected, sizeof(app_inputs_t));
    AppState_Snapshot(&snapshot);
    
    /* Assert - Full struct consistency */
    TEST_ASSERT_EQUAL_INT(expected.s1_aceleracion, snapshot.s1_aceleracion);
    TEST_ASSERT_EQUAL_INT(expected.s2_aceleracion, snapshot.s2_aceleracion);
    TEST_ASSERT_EQUAL_INT(expected.s_freno, snapshot.s_freno);
    TEST_ASSERT_EQUAL_INT(expected.boton_arranque, snapshot.boton_arranque);
    TEST_ASSERT_EQUAL_INT(expected.inv_state, snapshot.inv_state);
    TEST_ASSERT_EQUAL_INT(expected.inv_dc_bus_voltage, snapshot.inv_dc_bus_voltage);
    TEST_ASSERT_EQUAL_INT(expected.inv_motor_temp, snapshot.inv_motor_temp);
    TEST_ASSERT_EQUAL_INT(expected.inv_igbt_temp, snapshot.inv_igbt_temp);
    TEST_ASSERT_EQUAL_INT(expected.inv_air_temp, snapshot.inv_air_temp);
    TEST_ASSERT_EQUAL_INT(expected.inv_rpm, snapshot.inv_rpm);
    TEST_ASSERT_EQUAL_INT(expected.v_celda_min, snapshot.v_celda_min);
    TEST_ASSERT_EQUAL_INT(expected.ok_precarga, snapshot.ok_precarga);
    TEST_ASSERT_EQUAL_INT(expected.flag_EV_2_3, snapshot.flag_EV_2_3);
    TEST_ASSERT_EQUAL_INT(expected.flag_T11_8_9, snapshot.flag_T11_8_9);
    TEST_ASSERT_EQUAL_INT(expected.torque_total, snapshot.torque_total);
}
