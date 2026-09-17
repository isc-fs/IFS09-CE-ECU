// SPDX-License-Identifier: proprietary

#include "app/pit_diag.hpp"

#include "app/app_globals.h"   // g_can_tx_dropped (tx_dropped sticky bit)
#include "app/ecu_config.hpp"

#include "can/can_codecs.hpp"

#include <cstdint>

// firmware_info.cpp getters (no shared header -- only this TU needs them).
extern "C" {
std::uint8_t        ecu_fw_version_major(void);
std::uint8_t        ecu_fw_version_minor(void);
std::uint8_t        ecu_fw_version_patch(void);
const std::uint8_t* ecu_git_hash(void);
}

namespace ecu {

namespace {

// Wrap an encoded payload into an ACU-bus CanFrame. The DSL encoder fills a
// DLC-sized buffer; copy it into the 8-byte transport.
template <unsigned Dlc>
CanFrame make_acu(std::uint32_t id, const std::uint8_t (&buf)[Dlc]) noexcept {
    CanFrame f{};
    f.bus = static_cast<std::uint8_t>(CanBus::Acu);
    f.id  = id;
    f.dlc = static_cast<std::uint8_t>(Dlc);
    for (unsigned i = 0; i < Dlc; ++i) f.data[i] = buf[i];
    return f;
}

}  // namespace

CanFrame PitDiag::build_status(const CtrlOutput& c, const VehicleState& v,
                               bool start_button) noexcept {
    PitDiag_status_t s{};
    s.fsm_state = static_cast<std::uint8_t>(c.state);
    s.inv_state = v.inv_state;
    s.t11_8_9      = c.t11_8_9 ? 1u : 0u;
    s.rtds_active  = c.rtds_on ? 1u : 0u;
    s.ok_precharge = v.ok_precharge ? 1u : 0u;
    s.start_button = start_button ? 1u : 0u;
    s.dv_mode      = c.dv_mode ? 1u : 0u;   // DV drive latched this cycle
    s.tx_dropped   = (g_can_tx_dropped != 0u) ? 1u : 0u;  // any TX-queue drop since boot
    s.power_capped = c.power_capped ? 1u : 0u;            // EV 2.2.1 envelope active
    s.torque_pct    = c.torque_pct;
    s.v_cell_min_mV = v.v_cell_min_mV;
    s.torque_cmd    = c.torque_nm;   // real commanded shaft Nm (was hardcoded 0)
    std::uint8_t b[PitDiag_status_DLC];
    encode_PitDiag_status(s, b);
    return make_acu(PitDiag_status_ID, b);
}

CanFrame PitDiag::build_dv(const CtrlOutput& c, const CtrlInputs& in,
                           const VehicleState& v) noexcept {
    PitDiag_dv_t d{};
    d.dv_r2d_req       = in.dv_r2d_req ? 1u : 0u;                            // uDV 0x510 set+fresh
    d.dv_cmd_fresh     = in.dv_fresh ? 1u : 0u;                              // uDV 0x507 stream fresh
    d.ts_active        = in.ok_precharge ? 1u : 0u;                         // TX 0x504 view
    d.brake_over_limit = (in.brake_raw > in.cal.brake_dv_hard) ? 1u : 0u;  // TX 0x505 verdict
    d.r2d_confirm      = c.dv_mode ? 1u : 0u;                               // TX 0x511 (== latched)
    d.dv_torque_pct    = in.dv_torque_pct;                                  // conditioned 0x507
    d.motor_rpm_mech   = static_cast<std::int16_t>(v.inv_rpm / config::MotorPolePairs);
    // AS Emergency buzzer. as_status is the RAW byte, published even when
    // stale, so a bench can see what the uDV last said before it went quiet --
    // which is precisely the state the fail-safe acts on.
    d.as_emergency     = c.as_buzzer_active ? 1u : 0u;
    d.as_from_stale    = c.as_buzzer_from_stale ? 1u : 0u;
    d.as_fresh         = in.as_fresh ? 1u : 0u;
    d.as_status        = in.as_status;
    std::uint8_t b[PitDiag_dv_DLC];
    encode_PitDiag_dv(d, b);
    return make_acu(PitDiag_dv_ID, b);
}

CanFrame PitDiag::build_cell(const CellDerateState& s) noexcept {
    PitDiag_cell_t d{};
    d.raw_mV      = s.raw_mV;
    d.est_ocv_mV  = s.est_ocv_mV;
    d.comp_mV     = s.comp_mV;
    d.cap_pct     = s.cap_pct;
    d.compensated = s.compensated ? 1u : 0u;
    d.raw_floor   = s.raw_floor ? 1u : 0u;
    d.capped      = s.capped ? 1u : 0u;
    std::uint8_t b[PitDiag_cell_DLC];
    encode_PitDiag_cell(d, b);
    return make_acu(PitDiag_cell_ID, b);
}

CanFrame PitDiag::build_pack_temp(const PackThermalState& s) noexcept {
    PitDiag_pack_temp_t d{};
    d.pack_temp_used_degC = s.temp_degC;
    d.pack_temp_raw_degC  = s.raw_max_degC;
    d.pack_cap_pct        = s.cap_pct;
    d.mod0_used    = (s.valid_mask & 0x01u) ? 1u : 0u;
    d.mod1_used    = (s.valid_mask & 0x02u) ? 1u : 0u;
    d.mod2_used    = (s.valid_mask & 0x04u) ? 1u : 0u;
    d.mod3_used    = (s.valid_mask & 0x08u) ? 1u : 0u;
    d.mod4_used    = (s.valid_mask & 0x10u) ? 1u : 0u;
    d.pack_unknown = s.unknown ? 1u : 0u;
    d.pack_capped  = s.capped  ? 1u : 0u;
    std::uint8_t b[PitDiag_pack_temp_DLC];
    encode_PitDiag_pack_temp(d, b);
    return make_acu(PitDiag_pack_temp_ID, b);
}

CanFrame PitDiag::build_inv_foc(const VehicleState& v, std::uint32_t now_ms) noexcept {
    PitDiag_inv_foc_t d{};
    d.current_d_A  = v.inv_current_d_raw;
    d.current_q_A  = v.inv_current_q_raw;
    d.volt_modulus = v.inv_volt_modulus;
    d.ctrl_mode    = v.inv_ctrl_mode;
    d.ctrl_type    = v.inv_ctrl_type;
    d.cmd_src      = v.inv_cmd_src;
    // Per-frame freshness. The shared inverter tick cannot say that 0x467 in
    // particular went quiet, and a frozen ceiling reads as a healthy one.
    d.s4_fresh = VehicleService::is_fresh(now_ms, v.last_inv_s4_tick, config::InvFeedbackStaleMs) ? 1u : 0u;
    d.s6_fresh = VehicleService::is_fresh(now_ms, v.last_inv_s6_tick, config::InvFeedbackStaleMs) ? 1u : 0u;
    d.s8_fresh = VehicleService::is_fresh(now_ms, v.last_inv_s8_tick, config::InvFeedbackStaleMs) ? 1u : 0u;
    d.s9_fresh = VehicleService::is_fresh(now_ms, v.last_inv_s9_tick, config::InvFeedbackStaleMs) ? 1u : 0u;
    std::uint8_t b[PitDiag_inv_foc_DLC];
    encode_PitDiag_inv_foc(d, b);
    return make_acu(PitDiag_inv_foc_ID, b);
}

CanFrame PitDiag::build_inv_torque(const VehicleState& v,
                                   std::int16_t torque_req_nm) noexcept {
    PitDiag_inv_torque_t d{};
    // The WIRE value from the same builder that produced 0x362, not a
    // re-derivation -- so this is directly comparable with what the inverter
    // reports back, sign and all.
    d.torque_req      = torque_req_nm;
    d.torque_max_feas = v.inv_torque_max_feas;   // RAW: unit unresolved, see the .def
    d.torque_est      = v.inv_torque_est_nm;
    d.setpoint_q      = v.inv_setpoint_q_raw;
    std::uint8_t b[PitDiag_inv_torque_DLC];
    encode_PitDiag_inv_torque(d, b);
    return make_acu(PitDiag_inv_torque_ID, b);
}

CanFrame PitDiag::build_power(const CtrlOutput& c, const VehicleState& v) noexcept {
    PitDiag_power_t d{};
    // Commanded MECHANICAL power: P = T * omega, omega = rpm * 2*pi/60. Integer
    // via 2*pi/60 ~= 0.10472 -> *1047/10000. Reported in 10 W units.
    const std::int32_t rpm_mech = v.inv_rpm / config::MotorPolePairs;
    const std::int32_t shaft_W  =
        static_cast<std::int32_t>(c.torque_nm) * rpm_mech * 1047 / 10000;
    auto to_daW = [](std::int32_t w) -> std::int16_t {
        std::int32_t v10 = w / 10;
        if (v10 >  32767) v10 =  32767;
        if (v10 < -32768) v10 = -32768;
        return static_cast<std::int16_t>(v10);
    };
    d.shaft_power  = to_daW(shaft_W);
    d.ac_power     = to_daW(v.inv_ac_power_W);
    d.dc_bus_V     = v.inv_dc_bus_V;
    d.accu_current = v.current_accu_dA;   // AMS 0x135, the OTHER bus
    std::uint8_t b[PitDiag_power_DLC];
    encode_PitDiag_power(d, b);
    return make_acu(PitDiag_power_ID, b);
}

CanFrame PitDiag::build_pedals(const IoInputs& io, const PedalCal& cal) noexcept {
    PitDiag_pedals_t p{};
    p.apps1_raw = io.apps1_raw;
    p.apps2_raw = io.apps2_raw;
    p.brake_raw = io.brake_raw;
    p.apps1_pct = apps_pct(io.apps1_raw, cal.apps1_min, cal.apps1_max);
    p.apps2_pct = apps_pct(io.apps2_raw, cal.apps2_min, cal.apps2_max);
    std::uint8_t b[PitDiag_pedals_DLC];
    encode_PitDiag_pedals(p, b);
    return make_acu(PitDiag_pedals_ID, b);
}

CanFrame PitDiag::build_inverter(const VehicleState& v,
                                 std::uint8_t inv_mode_cmd) noexcept {
    PitDiag_inverter_t inv{};
    inv.dc_bus_voltage = v.inv_dc_bus_V;
    // 0x463 reports ELECTRICAL rpm; diag shows MECHANICAL shaft rpm, same
    // convention as the uDV 0x506 feed (erpm / MotorPolePairs, 10).
    inv.inv_rpm        = v.inv_rpm / config::MotorPolePairs;
    inv.inv_error      = v.inv_error;
    inv.dem_present    = v.inv_dem_present ? 1u : 0u;   // active fault vs latched history
    // 7-bit field; every InvMode fits (max 0x13 = 19). Mask defensively so a
    // bad value can never bleed into dem_present's bit.
    inv.inv_mode_cmd   = static_cast<std::uint8_t>(inv_mode_cmd & 0x7Fu);
    std::uint8_t b[PitDiag_inverter_DLC];
    encode_PitDiag_inverter(inv, b);
    return make_acu(PitDiag_inverter_ID, b);
}

CanFrame PitDiag::build_inverter_temps(const VehicleState& v,
                                       const MotorThermalState& th) noexcept {
    PitDiag_inverter_temps_t t{};
    t.temp_board_degC  = v.inv_temp_board;   // raw bytes; the DBC -50 offset -> degC
    t.temp_pwrstg_degC = v.inv_temp_pwrstg;
    t.temp_motor1_degC = v.inv_temp_motor1;
    t.temp_motor2_degC = v.inv_temp_motor2;
    // Re-encode with the same -50 offset the DBC will undo. Clamped rather than
    // wrapped so an absurd temperature stays absurd on the wire.
    {
        std::int32_t enc = static_cast<std::int32_t>(th.temp_degC) - config::MotorTempRawOffsetDegC;
        if (enc < 0)   enc = 0;
        if (enc > 255) enc = 255;
        t.motor_temp_used_degC = static_cast<std::uint8_t>(enc);
    }
    t.thermal_cap_pct  = th.cap_pct;
    t.temp_s1_valid    = th.s1_valid ? 1u : 0u;
    t.temp_s2_valid    = th.s2_valid ? 1u : 0u;
    t.temp_unknown     = th.unknown  ? 1u : 0u;
    t.thermal_capped   = th.capped   ? 1u : 0u;
    std::uint8_t b[PitDiag_inverter_temps_DLC];
    encode_PitDiag_inverter_temps(t, b);
    return make_acu(PitDiag_inverter_temps_ID, b);
}

CanFrame PitDiag::build_inv_faults(const VehicleState& v,
                                   const CtrlOutput& c,
                                   std::uint32_t now_ms) noexcept {
    PitDiag_inv_faults_t f{};
    const std::uint16_t p = v.inv_pwrstg_bits;   // L1, 9 bits
    const std::uint8_t  e = v.inv_emctrl_bits;   // L2, 8 bits
    f.pwrstg_alive          = (p >> 0) & 1u;
    f.pwrstg_enable         = (p >> 1) & 1u;
    f.pwrstg_uvlo           = (p >> 2) & 1u;
    f.pwrstg_desat          = (p >> 3) & 1u;
    f.pwrstg_dt_violation   = (p >> 4) & 1u;
    f.pwrstg_hvil_open      = (p >> 5) & 1u;
    f.pwrstg_ocp            = (p >> 6) & 1u;
    f.pwrstg_ovp_th1        = (p >> 7) & 1u;
    f.pwrstg_ovp_th2        = (p >> 8) & 1u;
    f.emctrl_init_ok        = (e >> 0) & 1u;
    f.emctrl_posfb          = (e >> 1) & 1u;
    f.emctrl_asc            = (e >> 2) & 1u;
    f.emctrl_curr_imbalance = (e >> 3) & 1u;
    f.emctrl_pwrstg_fault   = (e >> 4) & 1u;
    f.emctrl_curr_derating  = (e >> 5) & 1u;
    f.emctrl_loop_delocked  = (e >> 6) & 1u;
    f.emctrl_phcurr_acq     = (e >> 7) & 1u;
    // commanded side -- what the ECU decided to emit on FDCAN1 this cycle
    f.cmd_follow_n          = c.inv_mode_follow_n;
    f.cmd_flt_clear         = c.inv_flt_clear ? 1u : 0u;
    // Freshness of the 0x461 we are steering on. Saturate at 255 ms -- past
    // that the exact number stops mattering, it is simply far too stale.
    const std::uint32_t age = static_cast<std::uint32_t>(now_ms - v.last_inv_state_tick);
    f.inv_state_age_ms      = (age > 255u) ? 255u : static_cast<std::uint8_t>(age);
    f.inv_state_seq         = v.inv_state_seq;
    f.inv_redrive_count     = c.inv_redrive_count;   // Active -> WaitInvStandby fallbacks
    std::uint8_t b[PitDiag_inv_faults_DLC];
    encode_PitDiag_inv_faults(f, b);
    return make_acu(PitDiag_inv_faults_ID, b);
}

CanFrame PitDiag::build_fwinfo() noexcept {
    PitDiag_fwinfo_t fw{};
    fw.fw_major = ecu_fw_version_major();
    fw.fw_minor = ecu_fw_version_minor();
    fw.fw_patch = ecu_fw_version_patch();
    const std::uint8_t* g = ecu_git_hash();
    fw.git_hash = (static_cast<std::uint32_t>(g[0]) << 24) |
                  (static_cast<std::uint32_t>(g[1]) << 16) |
                  (static_cast<std::uint32_t>(g[2]) << 8) |
                   static_cast<std::uint32_t>(g[3]);
    std::uint8_t b[PitDiag_fwinfo_DLC];
    encode_PitDiag_fwinfo(fw, b);
    return make_acu(PitDiag_fwinfo_ID, b);
}

CanFrame PitDiag::build_health(const HealthMetrics& m) noexcept {
    PitDiag_health_t h{};
    h.free_heap     = m.free_heap;
    h.min_free_heap = m.min_free_heap;
    // split the liveness mask into 1-bit DBC signals (EcuTaskId bit order)
    h.task_control   = (m.task_ran_mask >> 0) & 1u;
    h.task_can_rx    = (m.task_ran_mask >> 1) & 1u;
    h.task_can_tx    = (m.task_ran_mask >> 2) & 1u;
    h.task_telemetry = (m.task_ran_mask >> 3) & 1u;   // EcuTaskId TELEMETRY=3
    h.task_diag      = (m.task_ran_mask >> 4) & 1u;   // EcuTaskId DIAG=4
    // Bench stub announce: mirror the compile-time ecu_config toggles onto
    // the bus so a bring-up image can't pass for a flight one. ALL ZERO on flight.
    // stub_no_ams is the load-bearing one -- it forces ok_precharge, which is what
    // 0x504 VCU_ts_active reports to the uDV, so set => TS-active here is FAKE.
    // Boot-time calibration outcome -- see the .def for why it is here.
    h.cal_status       = static_cast<std::uint8_t>(g_cal_load_status & 0x03u);
    // Sticky: any refusal since boot. See the .def for why it is one bit and
    // why it belongs on the ungated frame.
    h.boot_refused     = (g_boot_trigger_refused != 0u) ? 1u : 0u;
    h.stub_no_ams      = config::StubNoAms ? 1u : 0u;
    h.stub_no_inverter = config::StubNoInverter ? 1u : 0u;
    h.stub_start       = config::StubStart ? 1u : 0u;
    // stub_brake: restored to byte5 b3 (StubBrakeRaw != 0 injects a fake
    // brake_raw). Disables no safety gate -- also visible on 0x701 -- but carried
    // here so the flight-vs-bench announce on the ungated 0x704 is complete again.
    h.stub_brake       = (config::StubBrakeRaw != 0) ? 1u : 0u;
    // stub_torque_cap: the bring-up torque clamp. Announced here because it is
    // applied in control_task AFTER Controller::step(), so it trips none of the
    // derate `capped` flags -- a build limited to 80 % presented as a derate
    // that no derate could account for. A flight build reports 0.
    h.stub_torque_cap  = (config::TorqueCap < 100) ? 1u : 0u;
    h.reset_cause   = static_cast<std::uint8_t>(m.reset_cause);
    h.uptime_s      = m.uptime_s;
    h.last_fault    = static_cast<std::uint8_t>(m.last_fault);
    std::uint8_t b[PitDiag_health_DLC];
    encode_PitDiag_health(h, b);
    return make_acu(PitDiag_health_ID, b);
}

CanFrame PitDiag::build_brake(const IoInputs& io, const PedalCal& cal) noexcept {
    PitDiag_brake_t br{};
    // Real pressure from the EPT1400 transfer function, using an ABSOLUTE map
    // -- no calibration involved. Returns 0 while the sensor's full-scale range
    // is unset: reporting an invented pressure would be worse than none.
    br.brake_pressure = brake_pressure_dbar(io.brake_raw);
    // Percentage now uses the CALIBRATED span once a rest point exists, and
    // falls back to the legacy full-range scaling until then (see brake_pct).
    br.brake_pct = brake_pct(io.brake_raw, cal);
    std::uint8_t b[PitDiag_brake_DLC];
    encode_PitDiag_brake(br, b);
    return make_acu(PitDiag_brake_ID, b);
}

CanFrame PitDiag::build_cal_status(const CalSessionOutput& o, std::uint8_t last_cmd,
                                   std::uint8_t cal_load) noexcept {
    PitCal_status_t st{};
    st.session_state    = static_cast<std::uint8_t>(o.state);
    st.last_cmd         = last_cmd;
    st.result           = static_cast<std::uint8_t>(o.result);
    st.captured_mask    = o.captured_mask;
    st.validation_flags = o.validation_flags;
    st.cal_load         = cal_load;
    std::uint8_t b[PitCal_status_DLC];
    encode_PitCal_status(st, b);
    return make_acu(PitCal_status_ID, b);
}

CanFrame PitDiag::build_cal_apps(const PedalCal& c) noexcept {
    PitCal_apps_t a{};
    a.apps1_min = c.apps1_min; a.apps1_max = c.apps1_max;
    a.apps2_min = c.apps2_min; a.apps2_max = c.apps2_max;
    std::uint8_t b[PitCal_apps_DLC];
    encode_PitCal_apps(a, b);
    return make_acu(PitCal_apps_ID, b);
}

CanFrame PitDiag::build_cal_brake(const PedalCal& c) noexcept {
    PitCal_brake_t k{};
    k.brake_rest = c.brake_rest; k.brake_arm = c.brake_arm;
    k.brake_dv_hard = c.brake_dv_hard; k.brake_pressed = c.brake_pressed;
    std::uint8_t b[PitCal_brake_DLC];
    encode_PitCal_brake(k, b);
    return make_acu(PitCal_brake_ID, b);
}

CanFrame PitDiag::build_ack(bool enabled) noexcept {
    PitDiag_ack_t a{};
    a.enabled = enabled ? 1u : 0u;
    std::uint8_t b[PitDiag_ack_DLC];
    encode_PitDiag_ack(a, b);
    return make_acu(PitDiag_ack_ID, b);
}

}  // namespace ecu
