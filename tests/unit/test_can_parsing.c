#include "unity.h"
#include "can.h"
#include "mocks.h"

TEST_GROUP(CAN_Parsing);
TEST_GROUP(CAN_Packing);

void setUp(void)
{
    mock_tick_reset();
    memset(&g_in, 0, sizeof(g_in));
}

void tearDown(void)
{
}

TEST(CAN_Parsing, parse_precarga_ack_one_means_ok)
{
    can_msg_t frame = mock_can_precarga_ack(0x01);
    app_inputs_t st = {0};

    CanRx_ParseAndUpdate(&frame, &st);

    TEST_ASSERT_EQUAL_INT(1, st.ok_precarga);
}

TEST(CAN_Parsing, parse_precarga_ack_zero_means_pending)
{
    can_msg_t frame = mock_can_precarga_ack(0x00);
    app_inputs_t st = {0};

    CanRx_ParseAndUpdate(&frame, &st);

    TEST_ASSERT_EQUAL_INT(0, st.ok_precarga);
}

TEST(CAN_Parsing, parse_inverter_dc_bus_voltage_frame)
{
    can_msg_t frame = mock_can_dc_bus_voltage(0x0190);
    app_inputs_t st = {0};

    CanRx_ParseAndUpdate(&frame, &st);

    TEST_ASSERT_EQUAL_INT(0x0190, st.inv_dc_bus_voltage);
    TEST_ASSERT_EQUAL_INT(1, st.inv_vdc_ready);
}

TEST(CAN_Parsing, ignore_valid_id_on_wrong_bus)
{
    can_msg_t frame = mock_can_cell_min_voltage(3700);
    app_inputs_t st = {0};

    frame.bus = CAN_BUS_INV;
    CanRx_ParseAndUpdate(&frame, &st);

    TEST_ASSERT_EQUAL_INT(0, st.v_celda_min);
}

TEST(CAN_Parsing, ignore_dash_apps_frame_in_production)
{
    uint8_t data[8] = {0x09, 0xC4, 0x08, 0xC3, 0, 0, 0, 0};
    can_msg_t frame = mock_can_frame(0x101, data);
    app_inputs_t st = {0};
    frame.bus = CAN_BUS_DASH;
    frame.dlc = 4;

    CanRx_ParseAndUpdate(&frame, &st);

    TEST_ASSERT_EQUAL_INT(0, st.s1_aceleracion);
    TEST_ASSERT_EQUAL_INT(0, st.s2_aceleracion);
}

TEST(CAN_Parsing, ignore_dash_aux_apps_frame_in_production)
{
    uint8_t data[8] = {0xA8, 0x07, 0, 0, 0, 0, 0, 0};
    can_msg_t frame = mock_can_frame(0x102, data);
    app_inputs_t st = {0};
    frame.bus = CAN_BUS_DASH;
    frame.dlc = 2;

    CanRx_ParseAndUpdate(&frame, &st);

    TEST_ASSERT_EQUAL_INT(0, st.s2_aceleracion);
}

TEST(CAN_Parsing, ignore_dash_brake_frame_in_production)
{
    uint8_t data[8] = {0xAC, 0x0D, 0, 0, 0, 0, 0, 0};
    can_msg_t frame = mock_can_frame(0x103, data);
    app_inputs_t st = {0};
    frame.bus = CAN_BUS_DASH;
    frame.dlc = 2;

    CanRx_ParseAndUpdate(&frame, &st);

    TEST_ASSERT_EQUAL_INT(0, st.s_freno);
}

TEST(CAN_Parsing, parse_legacy_cell_min_voltage_big_endian)
{
    can_msg_t frame = mock_can_cell_min_voltage(3700);
    app_inputs_t st = {0};

    CanRx_ParseAndUpdate(&frame, &st);

    TEST_ASSERT_EQUAL_INT(3700, st.v_celda_min);
}

TEST(CAN_Parsing, parse_legacy_inverter_state_and_error)
{
    can_msg_t frame = mock_can_inv_state(10, 3);
    app_inputs_t st = {0};

    CanRx_ParseAndUpdate(&frame, &st);

    TEST_ASSERT_EQUAL_INT(10, st.inv_state);
    TEST_ASSERT_EQUAL_INT(3, st.inv_error);
}

TEST(CAN_Parsing, parse_legacy_inverter_temps)
{
    can_msg_t frame = mock_can_inv_temps(55, 48, 36);
    app_inputs_t st = {0};

    CanRx_ParseAndUpdate(&frame, &st);

    TEST_ASSERT_EQUAL_INT(55, st.inv_motor_temp);
    TEST_ASSERT_EQUAL_INT(48, st.inv_igbt_temp);
    TEST_ASSERT_EQUAL_INT(36, st.inv_air_temp);
}

TEST(CAN_Parsing, parse_ams_module_temps_as_signed_big_endian)
{
    can_msg_t frame = {0};
    app_inputs_t st = {0};

    frame.bus = CAN_BUS_ACU;
    frame.id = 0x136u;
    frame.dlc = 6u;
    frame.data[0] = 0xFFu; frame.data[1] = 0xFBu; /* -5 */
    frame.data[2] = 0x00u; frame.data[3] = 0x19u; /* 25 */
    frame.data[4] = 0x00u; frame.data[5] = 0x00u; /* 0 */

    CanRx_ParseAndUpdate(&frame, &st);

    TEST_ASSERT_EQUAL_INT(-5, st.temp_max_modulo[0]);
    TEST_ASSERT_EQUAL_INT(25, st.temp_max_modulo[1]);
    TEST_ASSERT_EQUAL_INT(0, st.temp_max_modulo[2]);
}

TEST(CAN_Parsing, parse_ams_dcdc_temp_sentinel_as_signed)
{
    can_msg_t frame = {0};
    app_inputs_t st = {0};

    frame.bus = CAN_BUS_ACU;
    frame.id = 0x137u;
    frame.dlc = 6u;
    frame.data[0] = 0x00u; frame.data[1] = 0x1Eu; /* 30 */
    frame.data[2] = 0xFFu; frame.data[3] = 0xF6u; /* -10 */
    frame.data[4] = 0x80u; frame.data[5] = 0x00u; /* -32768 sentinel */

    CanRx_ParseAndUpdate(&frame, &st);

    TEST_ASSERT_EQUAL_INT(30, st.temp_max_modulo[3]);
    TEST_ASSERT_EQUAL_INT(-10, st.temp_max_modulo[4]);
    TEST_ASSERT_EQUAL_INT(-32768, st.temp_dcdc);
}

TEST(CAN_Parsing, unknown_id_no_crash)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    can_msg_t frame = mock_can_frame(0xDEAD, data);
    app_inputs_t st = {
        .inv_dc_bus_voltage = 400
    };

    CanRx_ParseAndUpdate(&frame, &st);

    TEST_ASSERT_EQUAL_INT(400, st.inv_dc_bus_voltage);
}

TEST(CAN_Parsing, multiple_frames_sequential)
{
    app_inputs_t st = {0};

    can_msg_t f1 = mock_can_precarga_ack(0x01);
    CanRx_ParseAndUpdate(&f1, &st);
    TEST_ASSERT_EQUAL_INT(1, st.ok_precarga);

    can_msg_t f2 = mock_can_cell_min_voltage(3600);
    CanRx_ParseAndUpdate(&f2, &st);
    TEST_ASSERT_EQUAL_INT(3600, st.v_celda_min);

    can_msg_t f3 = mock_can_inv_state(4, 0);
    CanRx_ParseAndUpdate(&f3, &st);
    TEST_ASSERT_EQUAL_INT(4, st.inv_state);
}

TEST(CAN_Packing, roundtrip_pack_unpack)
{
    can_msg_t original = {
        .bus = CAN_BUS_INV,
        .id = 0x123,
        .dlc = 8,
        .ide = 0,
        .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
    };
    can_qitem16_t packed;
    can_msg_t unpacked;

    CAN_Pack16(&original, &packed);
    CAN_Unpack16(&packed, &unpacked);

    TEST_ASSERT_EQUAL_INT(original.id, unpacked.id);
    TEST_ASSERT_EQUAL_INT(original.dlc, unpacked.dlc);
    TEST_ASSERT_EQUAL_INT(original.bus, unpacked.bus);
    TEST_ASSERT_EQUAL_INT(original.ide, unpacked.ide);
    TEST_ASSERT_EQUAL_MEMORY(original.data, unpacked.data, 8);
}

TEST(CAN_Packing, roundtrip_dlc_4)
{
    can_msg_t original = {
        .bus = CAN_BUS_INV,
        .id = 0x200,
        .dlc = 4,
        .ide = 0,
        .data = {0xAA, 0xBB, 0xCC, 0xDD, 0x00, 0x00, 0x00, 0x00}
    };
    can_qitem16_t packed;
    can_msg_t unpacked;

    CAN_Pack16(&original, &packed);
    CAN_Unpack16(&packed, &unpacked);

    TEST_ASSERT_EQUAL_INT(4, unpacked.dlc);
    TEST_ASSERT_EQUAL_INT(0xAA, unpacked.data[0]);
    TEST_ASSERT_EQUAL_INT(0xBB, unpacked.data[1]);
}

TEST_GROUP_RUNNER(CAN_Parsing)
{
    RUN_TEST_CASE(CAN_Parsing, parse_precarga_ack_one_means_ok);
    RUN_TEST_CASE(CAN_Parsing, parse_precarga_ack_zero_means_pending);
    RUN_TEST_CASE(CAN_Parsing, parse_inverter_dc_bus_voltage_frame);
    RUN_TEST_CASE(CAN_Parsing, ignore_valid_id_on_wrong_bus);
    RUN_TEST_CASE(CAN_Parsing, ignore_dash_apps_frame_in_production);
    RUN_TEST_CASE(CAN_Parsing, ignore_dash_aux_apps_frame_in_production);
    RUN_TEST_CASE(CAN_Parsing, ignore_dash_brake_frame_in_production);
    RUN_TEST_CASE(CAN_Parsing, parse_legacy_cell_min_voltage_big_endian);
    RUN_TEST_CASE(CAN_Parsing, parse_legacy_inverter_state_and_error);
    RUN_TEST_CASE(CAN_Parsing, parse_legacy_inverter_temps);
    RUN_TEST_CASE(CAN_Parsing, parse_ams_module_temps_as_signed_big_endian);
    RUN_TEST_CASE(CAN_Parsing, parse_ams_dcdc_temp_sentinel_as_signed);
    RUN_TEST_CASE(CAN_Parsing, unknown_id_no_crash);
    RUN_TEST_CASE(CAN_Packing, roundtrip_pack_unpack);
    RUN_TEST_CASE(CAN_Packing, roundtrip_dlc_4);
    RUN_TEST_CASE(CAN_Parsing, multiple_frames_sequential);
}

void setUp_packing(void)
{
}

void tearDown_packing(void)
{
}

TEST_GROUP_RUNNER(CAN_Packing)
{
    RUN_TEST_CASE(CAN_Packing, roundtrip_pack_unpack);
    RUN_TEST_CASE(CAN_Packing, roundtrip_dlc_4);
}
