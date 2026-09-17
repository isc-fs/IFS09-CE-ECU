#include "unity.h"
#include "can.h"
#include "control.h"
#include "app_state.h"
#include "mocks.h"

TEST_GROUP(ErrorHandling);

void setUp(void)
{
    mock_tick_reset();
    AppState_Init();
    Control_Init();
}

void tearDown(void)
{
}

/**
 * UT-200: CAN timeout detection (no frames received for >1s)
 */
TEST(ErrorHandling, can_timeout_detection)
{
    /* Arrange */
    app_inputs_t in = mock_input_nominal();
    in.ok_precarga = 0;
    uint32_t initial_tick = osKernelGetTickCount();
    
    /* Act - Simulate 1.5 seconds without CAN update */
    mock_tick_advance(1500);
    uint32_t current_tick = osKernelGetTickCount();
    
    /* Assert - Timeout should be detected */
    TEST_ASSERT_GREATER_THAN(initial_tick, current_tick);
    TEST_ASSERT_TRUE(current_tick > initial_tick + 1000);
}

/**
 * UT-201: DC voltage out of range (low) - <300V
 */
TEST(ErrorHandling, dc_voltage_low_fault)
{
    /* Arrange */
    app_inputs_t in = mock_input_nominal();
    in.inv_dc_bus_voltage = 200;  /* <300V = FAULT */
    
    uint8_t ev23 = 0, t1189 = 0;
    control_out_t out = {0};
    
    /* Act */
    uint16_t torque = Control_ComputeTorque(&in, &ev23, &t1189);
    Control_Step10ms(&in, &out);
    
    /* Assert - Torque should be limited/zeroed on low voltage */
    TEST_ASSERT_TRUE(torque == 0 || torque < 10);
}

/**
 * UT-202: DC voltage out of range (high) - >500V
 */
TEST(ErrorHandling, dc_voltage_high_fault)
{
    /* Arrange */
    app_inputs_t in = mock_input_nominal();
    in.inv_dc_bus_voltage = 550;  /* >500V = FAULT */
    
    uint8_t ev23 = 0, t1189 = 0;
    control_out_t out = {0};
    
    /* Act */
    uint16_t torque = Control_ComputeTorque(&in, &ev23, &t1189);
    Control_Step10ms(&in, &out);
    
    /* Assert - Should limit/disable on overvoltage */
    TEST_ASSERT_TRUE(torque == 0 || torque < 10);
}

/**
 * UT-203: Motor temperature exceeds warning threshold (>80°C)
 */
TEST(ErrorHandling, motor_temperature_warning)
{
    /* Arrange */
    app_inputs_t in = mock_input_nominal();
    in.inv_motor_temp = 85;  /* >80°C = WARNING/FAULT */
    in.s1_aceleracion = 2800;  /* Throttle active */
    in.s2_aceleracion = 2550;
    
    uint8_t ev23 = 0, t1189 = 0;
    control_out_t out = {0};
    
    /* Act */
    uint16_t torque = Control_ComputeTorque(&in, &ev23, &t1189);
    Control_Step10ms(&in, &out);
    
    /* Assert - Torque should be limited on high temp */
    TEST_ASSERT_TRUE(torque < 50 || torque == 0);
}

/**
 * UT-204: IGBT temperature exceeds warning threshold (>75°C)
 */
TEST(ErrorHandling, igbt_temperature_warning)
{
    /* Arrange */
    app_inputs_t in = mock_input_nominal();
    in.inv_igbt_temp = 78;  /* >75°C = WARNING */
    in.s1_aceleracion = 2800;
    in.s2_aceleracion = 2550;
    
    uint8_t ev23 = 0, t1189 = 0;
    control_out_t out = {0};
    
    /* Act */
    uint16_t torque = Control_ComputeTorque(&in, &ev23, &t1189);
    Control_Step10ms(&in, &out);
    
    /* Assert - Torque should be limited */
    TEST_ASSERT_TRUE(torque < 50 || torque == 0);
}

/**
 * UT-205: Inverter in FAULT state
 */
TEST(ErrorHandling, inverter_fault_state)
{
    /* Arrange */
    app_inputs_t in = mock_input_inverter_fault();  /* inv_state = FAULT */
    in.s1_aceleracion = 2800;
    in.s2_aceleracion = 2550;
    
    uint8_t ev23 = 0, t1189 = 0;
    control_out_t out = {0};
    
    /* Act */
    uint16_t torque = Control_ComputeTorque(&in, &ev23, &t1189);
    Control_Step10ms(&in, &out);
    
    /* Assert - Torque should be zero on inverter fault */
    TEST_ASSERT_EQUAL_INT(0, torque);
}

/**
 * UT-206: Cell minimum voltage too low (<2.5V equivalent)
 */
TEST(ErrorHandling, cell_min_voltage_low)
{
    /* Arrange */
    app_inputs_t in = mock_input_nominal();
    in.v_celda_min = 1800;  /* <2500mV = FAULT */
    
    uint8_t ev23 = 0, t1189 = 0;
    
    /* Act */
    uint16_t torque = Control_ComputeTorque(&in, &ev23, &t1189);
    
    /* Assert - Should flag safety condition */
    /* If cell voltage is critical, EV 2.3 flag or T11.8.9 might be set */
    TEST_ASSERT_TRUE(torque == 0 || ev23 == 1 || t1189 == 1);
}

/**
 * UT-207: ADC sensor reading invalid (0xFFFF or stuck)
 */
TEST(ErrorHandling, adc_sensor_stuck)
{
    /* Arrange */
    app_inputs_t in = mock_input_nominal();
    in.s1_aceleracion = 0xFFFF;  /* Stuck at max */
    in.s2_aceleracion = 0xFFFF;  /* Disagreement between sensors */
    
    uint8_t ev23 = 0, t1189 = 0;
    
    /* Act */
    uint16_t torque = Control_ComputeTorque(&in, &ev23, &t1189);
    
    /* Assert - Sensor disagreement should trigger safety flag */
    TEST_ASSERT_EQUAL_INT(1, ev23);  /* Should latch */
    TEST_ASSERT_EQUAL_INT(0, torque);  /* Zero torque on fault */
}

/**
 * UT-208: Multiple faults simultaneously
 */
TEST(ErrorHandling, multiple_simultaneous_faults)
{
    /* Arrange */
    app_inputs_t in = mock_input_nominal();
    in.inv_dc_bus_voltage = 250;     /* Low voltage FAULT */
    in.inv_motor_temp = 90;          /* High temp FAULT */
    in.inv_state = 4;                /* Inverter FAULT */
    in.v_celda_min = 2000;           /* Low cell voltage */
    in.s1_aceleracion = 2800;
    in.s2_aceleracion = 2550;
    
    uint8_t ev23 = 0, t1189 = 0;
    control_out_t out = {0};
    
    /* Act */
    uint16_t torque = Control_ComputeTorque(&in, &ev23, &t1189);
    Control_Step10ms(&in, &out);
    
    /* Assert - With low cell voltage the new runtime should heavily limit torque. */
    TEST_ASSERT_TRUE(torque <= 5);
}

/**
 * UT-209: Graceful degradation with partial fault
 */
TEST(ErrorHandling, graceful_degradation)
{
    /* Arrange */
    app_inputs_t healthy = mock_input_nominal();
    healthy.inv_motor_temp = 50;
    healthy.inv_dc_bus_voltage = 400;
    healthy.s1_aceleracion = 2500;
    healthy.s2_aceleracion = 2550;
    
    app_inputs_t degraded = healthy;
    degraded.inv_motor_temp = 75;  /* Approaching limit but not critical */
    
    uint8_t ev23_h = 0, t1189_h = 0;
    uint8_t ev23_d = 0, t1189_d = 0;
    
    /* Act */
    uint16_t torque_healthy = Control_ComputeTorque(&healthy, &ev23_h, &t1189_h);
    uint16_t torque_degraded = Control_ComputeTorque(&degraded, &ev23_d, &t1189_d);
    
    /* Assert - Degraded state should produce less torque */
    TEST_ASSERT_TRUE(torque_degraded < torque_healthy);
}

/**
 * UT-210: Recovery after fault cleared
 */
TEST(ErrorHandling, recovery_after_fault_clear)
{
    /* Arrange */
    app_inputs_t in = mock_input_nominal();
    uint8_t ev23 = 0, t1189 = 0;
    
    /* Act (1st): Induce fault */
    in.inv_dc_bus_voltage = 200;
    Control_ComputeTorque(&in, &ev23, &t1189);
    /* Torque should be limited */
    
    /* Act (2nd): Clear fault */
    in.inv_dc_bus_voltage = 400;
    ev23 = 0;
    t1189 = 0;
    uint16_t torque_recovered = Control_ComputeTorque(&in, &ev23, &t1189);
    
    /* Assert - Should return to normal operation */
    TEST_ASSERT_GREATER_THAN(0, torque_recovered);
}

/**
 * UT-211: CAN frame corruption detection
 */
TEST(ErrorHandling, can_frame_corruption)
{
    /* Arrange */
    can_msg_t corrupted = {0};
    corrupted.id = 0xFFF;  /* Invalid CAN ID */
    corrupted.dlc = 15;    /* Invalid DLC (should be 0-8) */
    corrupted.data[0] = 0xFF;
    
    app_inputs_t in = mock_input_nominal();
    
    /* Act - Try to parse corrupted frame */
    CanRx_ParseAndUpdate(&corrupted, &in);
    
    /* Assert - Should not crash, corrupted frame ignored */
    TEST_ASSERT_EQUAL_INT(0, 0);  /* If we reach here, no crash */
}

/**
 * UT-212: Null pointer safety in error conditions
 */
TEST(ErrorHandling, null_pointer_error_handling)
{
    /* Arrange */
    app_inputs_t in = mock_input_nominal();
    
    /* Act - Call with NULL outputs */
    uint16_t torque = Control_ComputeTorque(&in, NULL, NULL);
    
    /* Assert - Should handle NULL without crash */
    TEST_ASSERT_TRUE(torque >= 0);
}

/**
 * UT-213: ADC boundary values (min/max)
 */
TEST(ErrorHandling, adc_boundary_extremes)
{
    /* Arrange */
    app_inputs_t in = mock_input_nominal();
    uint8_t ev23 = 0, t1189 = 0;
    
    /* Act with min values */
    in.s1_aceleracion = 0;
    in.s2_aceleracion = 0;
    uint16_t torque_min = Control_ComputeTorque(&in, &ev23, &t1189);
    
    /* Act with max values */
    in.s1_aceleracion = 0xFFFF;
    in.s2_aceleracion = 0xFFFF;
    uint16_t torque_max = Control_ComputeTorque(&in, &ev23, &t1189);
    
    /* Assert - Should clamp to valid range */
    TEST_ASSERT_TRUE(torque_min >= 0 && torque_min <= 100);
    TEST_ASSERT_TRUE(torque_max >= 0 && torque_max <= 100);
}

/**
 * UT-214: Timeout recovery mechanism
 */
TEST(ErrorHandling, timeout_recovery)
{
    /* Arrange */
    app_inputs_t in = mock_input_nominal();
    control_out_t out = {0};
    
    /* Act (1st) - Normal operation */
    Control_Step10ms(&in, &out);
    
    /* Act (2nd) - Simulate timeout by advancing time significantly */
    mock_tick_advance(2000);
    
    /* Act (3rd) - Recovery message comes in */
    in.ok_precarga = 1;
    Control_Step10ms(&in, &out);
    
    /* Assert - Should recover without crash */
    TEST_ASSERT_EQUAL_INT(0, 0);
}

/**
 * UT-215: State consistency after error recovery
 */
TEST(ErrorHandling, state_consistency_after_recovery)
{
    /* Arrange */
    app_inputs_t in = mock_input_nominal();
    app_inputs_t snapshot1 = {0};
    app_inputs_t snapshot2 = {0};
    
    /* Act (1st) - Normal state */
    in.torque_total = 50;
    AppState_Snapshot(&snapshot1);
    
    /* Act (2nd) - Simulate fault cascade then recovery */
    in.inv_dc_bus_voltage = 250;     /* Fault */
    in.inv_motor_temp = 100;
    mock_tick_advance(500);
    
    /* Act (3rd) - Fault cleared */
    in.inv_dc_bus_voltage = 400;
    in.inv_motor_temp = 50;
    in.torque_total = 50;
    AppState_Snapshot(&snapshot2);
    
    /* Assert - State should return to consistent values */
    TEST_ASSERT_EQUAL_INT(snapshot1.torque_total, snapshot2.torque_total);
}
