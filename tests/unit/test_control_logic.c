#include "unity.h"
#include "control.h"
#include "mocks.h"

TEST_GROUP(ControlLogic);

void setUp(void)
{
    mock_tick_reset();
    Control_Init();
}

void tearDown(void)
{
}

TEST(ControlLogic, compute_torque_nominal_zero)
{
    app_inputs_t in = mock_input_nominal();
    uint8_t ev23 = 0, t1189 = 0;

    uint16_t torque = Control_ComputeTorque(&in, &ev23, &t1189);

    TEST_ASSERT_EQUAL_INT(0, torque);
    TEST_ASSERT_EQUAL_INT(0, ev23);
    TEST_ASSERT_EQUAL_INT(0, t1189);
}

TEST(ControlLogic, compute_torque_half_throttle_legacy_points)
{
    app_inputs_t in = mock_input_throttle_50pct();
    uint8_t ev23 = 0, t1189 = 0;

    uint16_t torque = Control_ComputeTorque(&in, &ev23, &t1189);

    TEST_ASSERT_INT_WITHIN(5, 50, torque);
    TEST_ASSERT_EQUAL_INT(0, ev23);
    TEST_ASSERT_EQUAL_INT(0, t1189);
}

TEST(ControlLogic, compute_torque_max_legacy_points)
{
    app_inputs_t in = mock_input_throttle_max();
    uint8_t ev23 = 0, t1189 = 0;

    uint16_t torque = Control_ComputeTorque(&in, &ev23, &t1189);

    TEST_ASSERT_EQUAL_INT(100, torque);
    TEST_ASSERT_EQUAL_INT(0, ev23);
    TEST_ASSERT_EQUAL_INT(0, t1189);
}

TEST(ControlLogic, compute_torque_limited_by_vmin_linear_region)
{
    app_inputs_t in = mock_input_throttle_50pct();
    uint8_t ev23 = 0, t1189 = 0;

    in.v_celda_min = 3000;

    TEST_ASSERT_EQUAL_INT(16, Control_ComputeTorque(&in, &ev23, &t1189));
    TEST_ASSERT_EQUAL_INT(0, ev23);
    TEST_ASSERT_EQUAL_INT(0, t1189);
}

TEST(ControlLogic, compute_torque_limited_by_vmin_critical_region)
{
    app_inputs_t in = mock_input_throttle_50pct();
    uint8_t ev23 = 0, t1189 = 0;

    in.v_celda_min = 2500;

    TEST_ASSERT_EQUAL_INT(2, Control_ComputeTorque(&in, &ev23, &t1189));
    TEST_ASSERT_EQUAL_INT(0, ev23);
    TEST_ASSERT_EQUAL_INT(0, t1189);
}

TEST(ControlLogic, ev23_brake_throttle_engage_latch)
{
    app_inputs_t in = mock_input_nominal();
    in.s_freno = 4500;
    in.s1_aceleracion = 2800;
    in.s2_aceleracion = 2500;
    uint8_t ev23 = 0, t1189 = 0;

    uint16_t torque = Control_ComputeTorque(&in, &ev23, &t1189);

    TEST_ASSERT_EQUAL_INT(0, torque);
    TEST_ASSERT_EQUAL_INT(1, ev23);
}

TEST(ControlLogic, ev23_brake_throttle_release_latch)
{
    app_inputs_t in = mock_input_nominal();
    in.s_freno = 4500;
    in.s1_aceleracion = 2800;
    in.s2_aceleracion = 2500;
    uint8_t ev23 = 0, t1189 = 0;

    Control_ComputeTorque(&in, &ev23, &t1189);
    TEST_ASSERT_EQUAL_INT(1, ev23);

    in.s1_aceleracion = 2100;
    in.s2_aceleracion = 1960;
    ev23 = 0;
    t1189 = 0;
    TEST_ASSERT_EQUAL_INT(0, Control_ComputeTorque(&in, &ev23, &t1189));
    TEST_ASSERT_EQUAL_INT(1, ev23);

    in.s_freno = 2000;
    ev23 = 0;
    t1189 = 0;
    TEST_ASSERT_EQUAL_INT(0, Control_ComputeTorque(&in, &ev23, &t1189));
    TEST_ASSERT_EQUAL_INT(0, ev23);
}

TEST(ControlLogic, t1189_sensor_mismatch_zeroes_torque)
{
    app_inputs_t in = mock_input_nominal();
    in.s1_aceleracion = 2950;
    in.s2_aceleracion = 2243;
    uint8_t ev23 = 0, t1189 = 0;

    uint16_t torque = Control_ComputeTorque(&in, &ev23, &t1189);

    TEST_ASSERT_EQUAL_INT(1, t1189);
    TEST_ASSERT_EQUAL_INT(0, torque);
}

TEST(ControlLogic, control_step_boot_to_precharge)
{
    app_inputs_t in = mock_input_nominal();
    in.ok_precarga = 0;
    control_out_t out = {0};

    Control_Step10ms(&in, &out);

    TEST_ASSERT_EQUAL_INT(1, out.count);
    TEST_ASSERT_EQUAL_HEX32(0x100u, out.msgs[0].id);
    TEST_ASSERT_EQUAL_INT(CAN_BUS_ACU, out.msgs[0].bus);
    TEST_ASSERT_EQUAL_INT(0, out.msgs[0].ide);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(in.inv_dc_bus_voltage & 0xFFu), out.msgs[0].data[0]);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)((in.inv_dc_bus_voltage >> 8) & 0xFFu), out.msgs[0].data[1]);
}

TEST(ControlLogic, control_step_waits_for_vdc_config_before_precharge)
{
    app_inputs_t in = mock_input_nominal();
    control_out_t out = {0};

    in.inv_vdc_ready = 0;
    in.ok_precarga = 0;

    Control_Step10ms(&in, &out);

    TEST_ASSERT_EQUAL_INT(0, out.count);
    TEST_ASSERT_EQUAL_INT(0, out.torque_pct);
}

TEST(ControlLogic, control_step_precharge_emits_only_heartbeat)
{
    app_inputs_t in = mock_input_nominal();
    control_out_t out = {0};

    in.ok_precarga = 0;
    in.boton_arranque = 1;

    Control_Step10ms(&in, &out);

    /* The 0x600 precharge command was retired AMS-side (the AMS self-triggers
       precharge from its TSMS/DASH_CHG inputs). Holding the start button during
       precharge therefore emits only the 0x100 DC-bus heartbeat. */
    TEST_ASSERT_EQUAL_INT(1, out.count);
    TEST_ASSERT_EQUAL_HEX32(0x100u, out.msgs[0].id);
    TEST_ASSERT_EQUAL_INT(CAN_BUS_ACU, out.msgs[0].bus);
    TEST_ASSERT_EQUAL_INT(0, out.msgs[0].ide);
}

TEST(ControlLogic, control_step_waits_for_inverter_standby_before_runtime_commands)
{
    app_inputs_t in = mock_input_nominal();
    control_out_t out = {0};

    in.ok_precarga = 1;
    in.boton_arranque = 1;
    in.s_freno = 901;
    in.inv_state = 4;

    Control_Step10ms(&in, &out);
    Control_Step10ms(&in, &out);
    mock_tick_advance(2000);
    Control_Step10ms(&in, &out);
    Control_Step10ms(&in, &out);

    TEST_ASSERT_EQUAL_INT(0, out.count);
    TEST_ASSERT_EQUAL_INT(0, out.torque_pct);
}

TEST(ControlLogic, control_step_enables_rtds_during_r2d_delay)
{
    app_inputs_t in = mock_input_nominal();
    control_out_t out = {0};

    in.ok_precarga = 1;
    in.boton_arranque = 1;
    in.s_freno = 901;

    Control_Step10ms(&in, &out);
    Control_Step10ms(&in, &out);
    TEST_ASSERT_EQUAL_INT(1, out.rtds_active);

    mock_tick_advance(1500);
    Control_Step10ms(&in, &out);
    TEST_ASSERT_EQUAL_INT(1, out.rtds_active);

    mock_tick_advance(600);
    Control_Step10ms(&in, &out);
    TEST_ASSERT_EQUAL_INT(0, out.rtds_active);
}

TEST(ControlLogic, control_step_run_emits_legacy_inverter_frames)
{
    app_inputs_t in = mock_input_throttle_50pct();
    control_out_t out = {0};

    in.ok_precarga = 1;
    in.boton_arranque = 1;
    in.s_freno = 901;
    in.inv_state = 0;

    Control_Step10ms(&in, &out);
    Control_Step10ms(&in, &out);
    mock_tick_advance(2000);
    Control_Step10ms(&in, &out);

    in.inv_state = 3;
    Control_Step10ms(&in, &out);
    TEST_ASSERT_EQUAL_INT(2, out.count);
    TEST_ASSERT_EQUAL_HEX32(0x360u, out.msgs[0].id);
    TEST_ASSERT_EQUAL_HEX8(0x04u, out.msgs[0].data[2]);
    TEST_ASSERT_EQUAL_HEX32(0x362u, out.msgs[1].id);
    TEST_ASSERT_EQUAL_INT(0, out.torque_pct);

    in.inv_state = 4;
    Control_Step10ms(&in, &out);
    TEST_ASSERT_EQUAL_INT(2, out.count);
    TEST_ASSERT_EQUAL_HEX32(0x360u, out.msgs[0].id);
    TEST_ASSERT_EQUAL_HEX8(0x06u, out.msgs[0].data[2]);
    TEST_ASSERT_EQUAL_HEX32(0x362u, out.msgs[1].id);
    TEST_ASSERT_EQUAL_INT(0, out.torque_pct);

    in.inv_state = 6;
    in.s_freno = 2000;
    Control_Step10ms(&in, &out);

    TEST_ASSERT_EQUAL_INT(2, out.count);
    TEST_ASSERT_EQUAL_HEX32(0x360u, out.msgs[0].id);
    TEST_ASSERT_EQUAL_HEX8(0x06u, out.msgs[0].data[2]);
    TEST_ASSERT_EQUAL_HEX32(0x362u, out.msgs[1].id);
    TEST_ASSERT_EQUAL_INT(50, out.torque_pct);
}

TEST(ControlLogic, compute_torque_null_input)
{
    uint8_t ev23 = 0, t1189 = 0;

    TEST_ASSERT_EQUAL_INT(0, Control_ComputeTorque(NULL, &ev23, &t1189));
}

TEST(ControlLogic, control_step_null_output)
{
    app_inputs_t in = mock_input_nominal();

    Control_Step10ms(&in, NULL);

    TEST_ASSERT_EQUAL_INT(0, 0);
}

TEST(ControlLogic, compute_torque_adc_min)
{
    app_inputs_t in = mock_input_nominal();
    uint8_t ev23 = 0, t1189 = 0;

    in.s1_aceleracion = 0;
    in.s2_aceleracion = 0;

    TEST_ASSERT_EQUAL_INT(0, Control_ComputeTorque(&in, &ev23, &t1189));
}

TEST(ControlLogic, compute_torque_adc_max)
{
    app_inputs_t in = mock_input_nominal();
    uint8_t ev23 = 0, t1189 = 0;

    in.s1_aceleracion = 0xFFFF;
    in.s2_aceleracion = 0xFFFF;

    TEST_ASSERT_LESS_OR_EQUAL(100, Control_ComputeTorque(&in, &ev23, &t1189));
}

TEST(ControlLogic, ev23_no_latch_low_throttle)
{
    app_inputs_t in = mock_input_nominal();
    in.s_freno = 4500;
    in.s1_aceleracion = 2200;
    in.s2_aceleracion = 2050;
    uint8_t ev23 = 0, t1189 = 0;

    uint16_t torque = Control_ComputeTorque(&in, &ev23, &t1189);

    TEST_ASSERT_GREATER_THAN(0, torque);
    TEST_ASSERT_EQUAL_INT(0, ev23);
    TEST_ASSERT_EQUAL_INT(0, t1189);
}

TEST_GROUP_RUNNER(ControlLogic)
{
    RUN_TEST_CASE(ControlLogic, compute_torque_nominal_zero);
    RUN_TEST_CASE(ControlLogic, compute_torque_half_throttle_legacy_points);
    RUN_TEST_CASE(ControlLogic, compute_torque_max_legacy_points);
    RUN_TEST_CASE(ControlLogic, compute_torque_limited_by_vmin_linear_region);
    RUN_TEST_CASE(ControlLogic, compute_torque_limited_by_vmin_critical_region);
    RUN_TEST_CASE(ControlLogic, ev23_brake_throttle_engage_latch);
    RUN_TEST_CASE(ControlLogic, ev23_brake_throttle_release_latch);
    RUN_TEST_CASE(ControlLogic, t1189_sensor_mismatch_zeroes_torque);
    RUN_TEST_CASE(ControlLogic, control_step_boot_to_precharge);
    RUN_TEST_CASE(ControlLogic, control_step_waits_for_vdc_config_before_precharge);
    RUN_TEST_CASE(ControlLogic, control_step_precharge_emits_only_heartbeat);
    RUN_TEST_CASE(ControlLogic, control_step_waits_for_inverter_standby_before_runtime_commands);
    RUN_TEST_CASE(ControlLogic, control_step_enables_rtds_during_r2d_delay);
    RUN_TEST_CASE(ControlLogic, control_step_run_emits_legacy_inverter_frames);
    RUN_TEST_CASE(ControlLogic, compute_torque_null_input);
    RUN_TEST_CASE(ControlLogic, control_step_null_output);
    RUN_TEST_CASE(ControlLogic, compute_torque_adc_min);
    RUN_TEST_CASE(ControlLogic, compute_torque_adc_max);
    RUN_TEST_CASE(ControlLogic, ev23_no_latch_low_throttle);
}
