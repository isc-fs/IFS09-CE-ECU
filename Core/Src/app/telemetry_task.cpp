// SPDX-License-Identifier: proprietary

#include "app/app_globals.h"
#include "app/app_tasks.h"
#include "app/can_tx.hpp"
#include "app/ecu_config.hpp"   // config::StubTelemetryDummy (bench dummy-telemetry toggle)
#include "app/gps_service.hpp"
#include "app/radio_snapshot.hpp"
#include "app/vehicle_service.hpp"

#include "telemetry.h"

#include "cmsis_os2.h"
#include "nrf24.h"   // bit-bang nRF24 driver (NRF24_BusInit / ApplyDefaultConfig / SendPayload)

#include <cstdint>
#include <cstring>

using namespace ecu;

// Telemetry_Send32 shim: telemetry.h declares it (used by send_radio_snapshot);
// the bit-bang driver exposes NRF24_SendPayload. One 32-byte nRF24 payload.
extern "C" void Telemetry_Send32(const uint8_t payload[32]) {
    (void)NRF24_SendPayload(payload, 32u, 10u);
}

// Radio telemetry = the v2 "fragmented snapshot" (radio_snapshot.hpp): one
// 102-byte snapshot in 5 nRF24 fragments per 200 ms cycle, matched byte-for-byte
// to the live ground-station parser (IFS08-TE feat/receptor_08 ISC_RTT_serial.py
// _decode_snapshot). This replaced the older RF_FAST/RF_SLOW multi-kind protocol
// the ground station no longer parses. The FDCAN3 dash frames (send_dashboard)
// are a separate wire contract and are unchanged.
namespace {

constexpr uint32_t PeriodMs = 200u;

// Inter-fragment radio pacing: gap between the 5 nRF24 packets of one snapshot so
// the ground station can flush each over serial before the next lands (reception
// fix ported from MrAndy5's feat/1). 5 x this stays well under PeriodMs.
constexpr uint32_t RadioFragmentPaceMs = 5u;

void put_u16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

void put_u32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

void send_dash(uint32_t id, const uint8_t* data, uint8_t dlc) {
    CanFrame f{};
    f.bus = static_cast<uint8_t>(CanBus::Dash);
    f.id = id;
    f.dlc = dlc;
    for (uint8_t i = 0; i < dlc && i < CanFrameMaxData; ++i) f.data[i] = data[i];
    can_tx_post(f);
}

void send_dashboard(const VehicleState& v, uint16_t seq) {
    uint8_t d[8] = {};

    // 0x510 - status. Bits/bytes match docs/CAN3_MAP.md + DASH's
    // display_telemetry_can_config.c exactly (byte2 bit0=RESERVED, bit1=T11.8.9;
    // byte4=start button raw from ControlTask, mirrored via g_last_*).
    d[0] = v.inv_state;
    d[1] = static_cast<uint8_t>(g_last_torque_pct);
    // bit0 is RESERVED, always 0: it carried EV.2.3, deleted in FS-Rules 2024
    // along with the cut. NOT reclaimed and the layout NOT shifted -- the dash
    // decoder is owned by another team and a renumbered bit misparses silently.
    d[2] = static_cast<uint8_t>(g_last_t11_8_9 ? 0x02u : 0u);
    d[3] = v.ok_precharge ? 1u : 0u;
    d[4] = g_last_start_button;
    put_u16(&d[6], seq);
    send_dash(0x510u, d, 8u);

    // 0x511 - pedales/freno (ADC raw, real -- mirrored from ControlTask's IoInputs).
    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], g_last_apps1_raw);
    put_u16(&d[2], g_last_apps2_raw);
    put_u16(&d[4], g_last_brake_raw);
    send_dash(0x511u, d, 6u);

    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], v.inv_dc_bus_V);
    put_u16(&d[2], v.v_cell_min_mV);
    d[4] = v.inv_error;
    d[5] = v.last_vconfig_tick ? 1u : 0u;
    send_dash(0x512u, d, 6u);

    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], v.inv_temp_motor1);
    put_u16(&d[2], v.inv_temp_pwrstg);
    put_u16(&d[4], v.inv_temp_board);
    send_dash(0x513u, d, 6u);

    std::memset(d, 0, sizeof(d));
    put_u32(d, static_cast<uint32_t>(v.inv_rpm));
    send_dash(0x514u, d, 4u);

    // 0x515/0x516 - inverter speed/current "actual". PLACEHOLDER (0): the
    // inverter frames VehicleService decodes today (0x461/0x463/0x464/0x466)
    // don't carry these; no real source exists in this firmware yet.
    std::memset(d, 0, sizeof(d));
    send_dash(0x515u, d, 4u);
    send_dash(0x516u, d, 4u);

    std::memset(d, 0, sizeof(d));
    d[0] = g_last_ctrl_state;
    d[1] = v.ams_fsm_state;
    send_dash(0x517u, d, 2u);

    // 0x518 - AMS soc (PLACEHOLDER: no estimator, deferred on the AMS side
    // too -- see IFS08-CE-AMS acu_tx_encoders.hpp) + corrientes/temp_dcdc
    // (real, from ACU_currents 0x135 / ACU_tmax_module_b 0x137).
    std::memset(d, 0, sizeof(d));
    put_u16(&d[1], static_cast<uint16_t>(v.current_accu_dA));
    put_u16(&d[3], static_cast<uint16_t>(v.current_dcdc_dA));
    put_u16(&d[5], static_cast<uint16_t>(v.tmax_dcdc));
    send_dash(0x518u, d, 7u);

    // 0x519/0x51A - GPS. Still zero, but NOT for lack of a driver: gps_task.cpp
    // reads an MTK3339 on USART10, and GpsService already feeds 0x508/0x509 and
    // the radio snapshot below. All that is missing is wiring GpsService into
    // these two dash frames. See docs/CAN3_MAP.md.
    std::memset(d, 0, sizeof(d));
    send_dash(0x519u, d, 8u);
    send_dash(0x51Au, d, 8u);

    // 0x51B - gps_longitude still zero (same wiring gap as 0x519); tick_ms is
    // real (RTOS tick).
    std::memset(d, 0, sizeof(d));
    put_u32(&d[4], osKernelGetTickCount());
    send_dash(0x51Bu, d, 8u);

    // 0x51C/0x51D - AMS per-module vmin (real, from ACU_vmin_module_a/b
    // 0x131/0x132).
    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], v.vmin_module[0]);
    put_u16(&d[2], v.vmin_module[1]);
    put_u16(&d[4], v.vmin_module[2]);
    send_dash(0x51Cu, d, 6u);

    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], v.vmin_module[3]);
    put_u16(&d[2], v.vmin_module[4]);
    send_dash(0x51Du, d, 4u);

    // 0x51E/0x51F - AMS per-module vmax (real, from ACU_vmax_module_a/b
    // 0x133/0x134).
    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], v.vmax_module[0]);
    put_u16(&d[2], v.vmax_module[1]);
    put_u16(&d[4], v.vmax_module[2]);
    send_dash(0x51Eu, d, 6u);

    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], v.vmax_module[3]);
    put_u16(&d[2], v.vmax_module[4]);
    send_dash(0x51Fu, d, 4u);

    // 0x520/0x521 - AMS per-module tmax + temp_dcdc (real, from
    // ACU_tmax_module_a/b 0x136/0x137).
    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], static_cast<uint16_t>(v.tmax_module[0]));
    put_u16(&d[2], static_cast<uint16_t>(v.tmax_module[1]));
    put_u16(&d[4], static_cast<uint16_t>(v.tmax_module[2]));
    send_dash(0x520u, d, 6u);

    std::memset(d, 0, sizeof(d));
    put_u16(&d[0], static_cast<uint16_t>(v.tmax_module[3]));
    put_u16(&d[2], static_cast<uint16_t>(v.tmax_module[4]));
    put_u16(&d[4], static_cast<uint16_t>(v.tmax_dcdc));
    send_dash(0x521u, d, 6u);
}

// Populate the snapshot inputs from the shared vehicle state + the ControlTask
// g_last_* mirrors, serialize the 102-byte v2 snapshot, and send it as five
// nRF24 fragments. The byte layout is owned by radio_snapshot.cpp (matched to
// the ground-station parser); this is only the data plumbing.
void send_radio_snapshot(const VehicleState& v, uint16_t seq, uint32_t tick_ms) {
    RadioSnapshotInputs in{};
    in.tick_ms       = tick_ms;
    in.seq           = seq;
    in.start_button  = g_last_start_button;
    in.apps1_raw     = g_last_apps1_raw;
    in.apps2_raw     = g_last_apps2_raw;
    in.brake_raw     = g_last_brake_raw;
    in.torque_pct    = static_cast<uint8_t>(g_last_torque_pct);
    in.t11_8_9       = g_last_t11_8_9;
    in.state         = g_last_ctrl_state;
    in.ok_precharge  = v.ok_precharge ? 1u : 0u;
    in.ams_fsm_state = v.ams_fsm_state;
    in.v_cell_min_mV = v.v_cell_min_mV;
    in.soc           = 0u;  // PLACEHOLDER: AMS has no SoC estimator yet.
    for (unsigned i = 0; i < 5; ++i) {
        in.vmin_module[i] = v.vmin_module[i];
        in.vmax_module[i] = v.vmax_module[i];
        in.tmax_module[i] = v.tmax_module[i];
    }
    in.current_accu_dA    = v.current_accu_dA;
    in.current_dcdc_dA    = v.current_dcdc_dA;
    in.tmax_dcdc          = v.tmax_dcdc;
    in.inv_state          = v.inv_state;
    in.inv_vconfig_active = v.last_vconfig_tick ? 1u : 0u;
    in.inv_error          = v.inv_error;
    in.inv_dc_bus_V       = v.inv_dc_bus_V;
    in.inv_temp_motor1    = v.inv_temp_motor1;
    in.inv_temp_pwrstg    = v.inv_temp_pwrstg;
    in.inv_temp_board     = v.inv_temp_board;
    in.inv_rpm            = v.inv_rpm;
    in.inv_speed_actual   = 0;  // PLACEHOLDER: no inverter speed/current source.
    in.inv_current_actual = 0;

    // GPS (0x508/0x509 also carry this; here it rides the radio to the pit).
    // Straight from GpsService -- NOT from VehicleService: the GPS is a local
    // UART peripheral, not something we receive over CAN.
    const GpsState gps      = GpsService::instance().snapshot();
    in.gps_lat_deg1e7       = gps.fix.lat_deg1e7;
    in.gps_lon_deg1e7       = gps.fix.lon_deg1e7;
    in.gps_speed_kmh_x100   = static_cast<uint16_t>(
        gps.fix.speed_kmh_x100 > 0xFFFFu ? 0xFFFFu : gps.fix.speed_kmh_x100);
    in.gps_course_deg_x100  = static_cast<uint16_t>(
        gps.fix.course_deg_x100 > 0xFFFFu ? 0xFFFFu : gps.fix.course_deg_x100);
    in.gps_sats             = gps.fix.sats;
    in.gps_has_fix          = gps.fix.has_fix ? 1u : 0u;

    uint8_t snap[kRadioSnapshotWireSize];
    serialize_radio_snapshot(snap, in);
    for (uint8_t frag = 0; frag < kRadioSnapshotFragments; ++frag) {
        uint8_t p[kRadioFragmentSize];
        build_radio_fragment(p, snap, seq, frag);
        Telemetry_Send32(p);
#if !defined(SIL_BUILD)
        // Pace the fragments (~5 ms) so the ground station flushes each 32-byte
        // packet over its serial link before the next arrives. Without this the 5
        // back-to-back sends overrun the receiver and the snapshot is dropped --
        // this is the reception fix from MrAndy5's feat/1, adapted to our
        // direct-send path (the legacy branch paced in a separate radio-TX task).
        osDelay(RadioFragmentPaceMs);
#endif
    }
}

// Bench only (config::StubTelemetryDummy): fill the VehicleState with a
// deterministic synthetic SWEEP keyed on the frame `seq`, so the radio snapshot
// + FDCAN3 dash show MOVING, plausible values with no live AMS/inverter on the
// bus. Covers the CAN-sourced fields only; pedals/torque/flags stay real
// (ControlTask's g_last_* mirrors). [[maybe_unused]] -- discarded when the
// toggle is false, so a flight build never references it.
[[maybe_unused]] void make_dummy_vehicle_state(VehicleState& v, uint16_t seq) {
    const uint16_t s = seq;
    // --- inverter (FDCAN1 0x461/0x463/0x464/0x466) ---
    v.inv_state         = 4u;                                       // Ready
    v.inv_error         = 0u;
    v.inv_rpm           = static_cast<int32_t>((s % 160u) * 50u);   // 0..7950 erpm ramp
    v.inv_dc_bus_V      = static_cast<uint16_t>(380u + (s % 40u));  // ~380..419 V
    v.inv_temp_board    = static_cast<uint8_t>(30u + (s % 30u));    // raw byte (-50 -> degC)
    v.inv_temp_pwrstg   = static_cast<uint8_t>(35u + (s % 30u));
    v.inv_temp_motor1   = static_cast<uint8_t>(40u + (s % 30u));
    v.inv_temp_motor2   = static_cast<uint8_t>(42u + (s % 30u));
    v.last_vconfig_tick = 1u;                                       // -> vconfig_active = 1
    // --- AMS / ACU (FDCAN2) ---
    v.ok_precharge  = true;
    v.ams_fsm_state = 2u;                                           // any non-Error state
    v.v_cell_min_mV = static_cast<uint16_t>(3200u + (s % 600u));    // 3.20..3.80 V
    for (unsigned i = 0; i < 5u; ++i) {
        v.vmin_module[i] = static_cast<uint16_t>(3200u + i * 10u + (s % 200u));
        v.vmax_module[i] = static_cast<uint16_t>(3600u + i * 10u + (s % 200u));
        v.tmax_module[i] = static_cast<int16_t>(25 + static_cast<int>(i) * 2 +
                                                static_cast<int>(s % 20u));
    }
    v.current_accu_dA = static_cast<int16_t>(static_cast<int>(s % 200u) - 100);  // -10.0..+9.9 A
    v.current_dcdc_dA = static_cast<int16_t>(static_cast<int>(s % 100u) - 50);
    v.tmax_dcdc       = static_cast<int16_t>(40 + static_cast<int>(s % 20u));
}

}  // namespace

#if defined(SIL_BUILD)
namespace ecu {

uint32_t telemetry_period_ms_for_test() {
    return PeriodMs;
}

void telemetry_emit_for_test(const VehicleState& v, uint16_t seq) {
    send_dashboard(v, seq);
    send_radio_snapshot(v, seq, osKernelGetTickCount());
}

}  // namespace ecu
#endif

extern "C" void ecu_telemetry_task_run(void *argument) {
    (void)argument;
    uint16_t seq = 0;
    uint32_t tick = osKernelGetTickCount();
    auto& vs = VehicleService::instance();

    // Bring up the nRF24 on the bit-bang SPI transport (claims PA5/6/7 as GPIO,
    // overriding the CubeMX SPI1 AF config -- SPI1 hardware can't read MISO on
    // this board). Radio is idle until this runs.
    NRF24_BusInit();
    (void)NRF24_ApplyDefaultConfig();

    for (;;) {
        ++g_task_step[ECU_TASK_TELEMETRY];
        ++seq;
        // Live snapshot, or (bench) a synthetic sweep so the ground station / dash
        // can be validated with no live AMS/inverter on the bus. The toggle folds
        // away at compile time -- a flight build only ever takes the snapshot.
        VehicleState v;   // default-inits to zero
        if constexpr (config::StubTelemetryDummy) {
            make_dummy_vehicle_state(v, seq);
        } else {
            v = vs.snapshot();
        }
        send_dashboard(v, seq);
        send_radio_snapshot(v, seq, osKernelGetTickCount());

        tick += PeriodMs;
        osDelayUntil(tick);
    }
}
