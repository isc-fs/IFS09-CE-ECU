// SPDX-License-Identifier: proprietary

#include "app/vehicle_service.hpp"

#include "app/ecu_config.hpp"

#include <cstdint>

namespace {

// Generic BE16 reads, shared by the per-module AMS frames (0x131-0x137):
// unlike the single-field frames above, these carry 2-3 same-shaped fields
// each, so a shared helper beats repeating the shift-pattern per field.
std::uint16_t be16(const std::uint8_t* d) noexcept {
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(d[0]) << 8) | d[1]);
}
std::int16_t be16_s(const std::uint8_t* d) noexcept {
    return static_cast<std::int16_t>(be16(d));
}

// The EMC inverter frames are LITTLE-endian throughout (DBC @1), unlike the
// AMS block above. Separate helpers rather than a flag, so a call site cannot
// silently pick the wrong endianness.
std::int16_t le16_s(const std::uint8_t* d) noexcept {
    return static_cast<std::int16_t>(
        static_cast<std::uint16_t>(d[0] | (static_cast<std::uint16_t>(d[1]) << 8)));
}

}  // namespace

namespace ecu {

VehicleService& VehicleService::instance() noexcept {
    static VehicleService Instance;
    return Instance;
}

// ---- pure decoders (layouts mirror the .def files) -------------------------

bool VehicleService::decode_ok_precharge(const std::uint8_t* d) noexcept {
    return d[0] != 0u;
}

std::uint16_t VehicleService::decode_v_cell_min(const std::uint8_t* d) noexcept {
    return static_cast<std::uint16_t>(                       // BE16 @ bytes 0-1
        (static_cast<std::uint16_t>(d[0]) << 8) | d[1]);
}

std::uint8_t VehicleService::decode_ams_fsm_state(const std::uint8_t* d) noexcept {
    return d[0];
}

std::uint16_t VehicleService::decode_ams_min_cell(const std::uint8_t* d) noexcept {
    return static_cast<std::uint16_t>(                       // BE16 @ bytes 4-5
        (static_cast<std::uint16_t>(d[4]) << 8) | d[5]);
}

std::uint8_t VehicleService::decode_inv_state(const std::uint8_t* d) noexcept {
    return static_cast<std::uint8_t>(d[4] & 0x7Fu);          // App_State_App @ bit 32, 7 bits
}

std::uint16_t VehicleService::decode_inv_dc_bus_V(const std::uint8_t* d) noexcept {
    // DCBus_Voltage_V: 10-bit little-endian starting at frame bit 16.
    return static_cast<std::uint16_t>(
        d[2] | ((static_cast<std::uint16_t>(d[3]) & 0x03u) << 8));
}

std::int32_t VehicleService::decode_inv_ac_power_W(const std::uint8_t* d) noexcept {
    // ACBus_Power_W: 26|16@1- in the vendor DBC. Little-endian and NOT
    // byte-aligned, so the 16 bits straddle three bytes:
    //   bits 26-31 -> byte3 bits 2-7   (the 6 LSBs)
    //   bits 32-39 -> byte4            (the next 8)
    //   bits 40-41 -> byte5 bits 0-1   (the top 2)
    std::uint32_t raw = (static_cast<std::uint32_t>(d[3]) >> 2) |
                        (static_cast<std::uint32_t>(d[4]) << 6) |
                        ((static_cast<std::uint32_t>(d[5]) & 0x03u) << 14);
    if (raw & 0x8000u) raw |= 0xFFFF0000u;          // sign-extend bit 15
    const std::int32_t counts = static_cast<std::int32_t>(raw);
    // 32.767 W/LSB, folded in here so no caller carries the scale. Integer:
    // multiply by 32767 then divide by 1000. Worst case |counts| = 32768, so the
    // product is ~1.07e9 -- inside int32 (2.1e9), but only just, which is why
    // this is done in one place with a comment rather than at each call site.
    return counts * config::InvAcPowerScaleMilli / 1000;
}

std::int32_t VehicleService::decode_udv_torque_cmd(const std::uint8_t* d) noexcept {
    // 0x507: torque command as an INTEGER percent, int32 little-endian on the
    // wire (agreed with the DV team; replaced the original float32 idea).
    const std::uint32_t u = static_cast<std::uint32_t>(d[0]) |
                            (static_cast<std::uint32_t>(d[1]) << 8) |
                            (static_cast<std::uint32_t>(d[2]) << 16) |
                            (static_cast<std::uint32_t>(d[3]) << 24);
    return static_cast<std::int32_t>(u);
}

std::int32_t VehicleService::decode_inv_rpm(const std::uint8_t* d) noexcept {
    // EMachine_Speed_erpm: 20-bit SIGNED, little-endian, frame bit 44 -- the LSB
    // sits at byte5 bit4, so the 20 bits span d5[4:7] | d6 | d7. Sign-extend from
    // bit 19. (DBC-correct; the bit-40 layout the HIL sim used reads wrong.)
    std::uint32_t raw = (static_cast<std::uint32_t>(d[5]) >> 4) |
                        (static_cast<std::uint32_t>(d[6]) << 4) |
                        (static_cast<std::uint32_t>(d[7]) << 12);
    if (raw & 0x80000u) raw |= 0xFFF00000u;             // sign-extend bit 19
    return static_cast<std::int32_t>(raw);
}

std::uint8_t VehicleService::condition_udv_torque(std::int32_t cmd) noexcept {
    // Fail-safe conditioning of the autonomous torque command: the wire
    // value is an integer percent; anything outside 0..100 is not a sane
    // command. Negative -> 0 (regen/reverse is NOT in the contract -- an
    // explicit extension if ever wanted), overrange clamps.
    if (cmd < 0)   return 0u;
    if (cmd > 100) return 100u;
    return static_cast<std::uint8_t>(cmd);
}

std::uint16_t VehicleService::heartbeat_dc_bus_V(const VehicleState& v,
                                                 std::uint32_t now) noexcept {
    // See the header for why stale publishes 0 rather than the held value, and
    // why 0 is the right direction for the AMS precharge criterion but NOT for
    // the DC-link discharge gate.
    if (!is_fresh(now, v.last_vconfig_tick, config::InvDcBusStaleMs)) return 0u;
    return v.inv_dc_bus_V;
}

// ---- freshness -------------------------------------------------------------

bool VehicleService::is_fresh(std::uint32_t now, std::uint32_t last,
                              std::uint32_t window) noexcept {
    if (last == 0u) return false;
    const std::uint32_t age = (now >= last) ? (now - last) : 0u;
    return age <= window;
}

// ---- single-writer update --------------------------------------------------

bool VehicleService::update_from_frame(const CanFrame& f) noexcept {
    // --- inverter bus (FDCAN1) ---
    if (f.bus == static_cast<std::uint8_t>(CanBus::Inv)) {
        if (f.id == config::InvRxStateId) {                 // 0x461
            if (f.dlc < 5) return false;
            state_.inv_state     = decode_inv_state(f.data);
            state_.inv_error     = f.data[2];               // DEM_Code low byte
            // DEM_Present (byte3 bit7): fault active NOW vs latched history. The
            // NX boots latched (DEM_Code set, DEM_Present clear) -- this bit is
            // what tells the two apart. dlc>=5 guaranteed by the guard above.
            state_.inv_dem_present = (f.data[3] & 0x80u) != 0u;
            // L2/L1 fault layers (DBC EMCtrl_FOC_BitState 39|8@1+, PwrStg_
            // BitState 47|9@1+) straddle bytes 4-6, so they need the full DLC 7.
            // Guarded separately from the dlc>=5 check above: a short frame must
            // still yield inv_state/DEM rather than dropping the whole update.
            if (f.dlc >= 7) {
                state_.inv_emctrl_bits = static_cast<std::uint8_t>(
                    (f.data[4] >> 7) | static_cast<std::uint8_t>((f.data[5] & 0x7Fu) << 1));
                state_.inv_pwrstg_bits = static_cast<std::uint16_t>(
                    (f.data[5] >> 7) | (static_cast<std::uint16_t>(f.data[6]) << 1));
            }
            state_.last_inv_state_tick = f.timestamp_ms;    // 0x461 only
            ++state_.inv_state_seq;                         // wraps; delta = arrivals
            state_.last_inv_tick = f.timestamp_ms;
            return true;
        }
        if (f.id == config::InvRxRpmId) {                   // 0x463
            if (f.dlc < 8) return false;
            state_.inv_rpm       = decode_inv_rpm(f.data);
            // FOC (field-oriented control) feedback, which shares this frame.
            // Current_Q_A is the torque-producing axis: if Q current plateaus
            // while our request keeps climbing, the INVERTER is what is
            // limiting us.
            state_.inv_current_d_raw = le16_s(&f.data[0]);   // Current_D_A
            state_.inv_current_q_raw = le16_s(&f.data[2]);   // Current_Q_A
            // Volt_Modulus_permil: 12 bits at frame bit 32 -> byte4 | byte5[0:3].
            state_.inv_volt_modulus  = static_cast<std::uint16_t>(
                f.data[4] | (static_cast<std::uint16_t>(f.data[5] & 0x0Fu) << 8));
            state_.last_inv_tick    = f.timestamp_ms;
            state_.last_inv_s4_tick = f.timestamp_ms;
            return true;
        }
        if (f.id == config::InvRxTempId) {                  // 0x464
            if (f.dlc < 4) return false;
            state_.inv_temp_board  = f.data[0];
            state_.inv_temp_pwrstg = f.data[1];
            state_.inv_temp_motor1 = f.data[2];
            state_.inv_temp_motor2 = f.data[3];
            state_.last_inv_tick       = f.timestamp_ms;
            state_.last_inv_temps_tick = f.timestamp_ms;   // per-signal: see the header
            return true;
        }
        if (f.id == config::InvRxCtrlModeId) {              // 0x465 EMC_TX_STATE_6
            if (f.dlc < 2) return false;
            // Which control law the inverter is running and where it is taking
            // commands from. If Ctrl_Mode/Ctrl_Type are not what we assume, the
            // Torque_Nm_Req on 0x362 may not even be the active setpoint.
            state_.inv_cmd_src   = static_cast<std::uint8_t>(f.data[0] & 0x0Fu);
            state_.inv_ctrl_type = static_cast<std::uint8_t>(f.data[0] >> 4);
            state_.inv_ctrl_mode = static_cast<std::uint8_t>(f.data[1] & 0x0Fu);
            state_.last_inv_tick    = f.timestamp_ms;
            state_.last_inv_s6_tick = f.timestamp_ms;
            return true;
        }
        if (f.id == config::InvRxTorqueLimId) {             // 0x467 EMC_TX_STATE_8
            if (f.dlc < 6) return false;
            // Torque_Max_Feas: the inverter stating its OWN ceiling. This is the
            // single most direct answer to "is the inverter limiting us?" and it
            // has been broadcast, undecoded, the whole time.
            state_.inv_torque_max_feas = le16_s(&f.data[0]);
            state_.inv_setpoint_q_raw  = le16_s(&f.data[4]);   // Setpoint_App_Q_A
            state_.last_inv_tick    = f.timestamp_ms;
            state_.last_inv_s8_tick = f.timestamp_ms;
            return true;
        }
        if (f.id == config::InvRxTorqueEstId) {             // 0x468 EMC_TX_STATE_9
            if (f.dlc < 4) return false;
            // Torque_Est_Nm @ frame bit 16 -- bytes 0-1 are the E2E header.
            state_.inv_torque_est_nm = le16_s(&f.data[2]);
            state_.last_inv_tick    = f.timestamp_ms;
            state_.last_inv_s9_tick = f.timestamp_ms;
            return true;
        }
        if (f.id == config::InvRxDcBusId) {                 // 0x466
            if (f.dlc < 4) return false;
            state_.inv_dc_bus_V      = decode_inv_dc_bus_V(f.data);
            // ACBus_Power_W needs the full DLC 6. Guarded SEPARATELY from the
            // dlc >= 4 above, same pattern as 0x461's fault layers: a short
            // frame must still yield the DC-bus voltage (which gates Precharge)
            // rather than dropping the whole update.
            if (f.dlc >= 6) {
                state_.inv_ac_power_W = decode_inv_ac_power_W(f.data);
            }
            state_.last_vconfig_tick = f.timestamp_ms;
            state_.last_inv_tick     = f.timestamp_ms;
            return true;
        }
        return false;
    }

    // --- AMS / ACU bus (FDCAN2) ---
    if (f.bus == static_cast<std::uint8_t>(CanBus::Acu)) {
        if (f.id == config::AcuOkPrechargeId) {             // 0x020
            state_.ok_precharge  = decode_ok_precharge(f.data);
            state_.last_ams_tick = f.timestamp_ms;
            return true;
        }
        if (f.id == config::AcuDischargeInterlockId) {      // 0x021
            if (f.dlc < 1) return false;
            // Bit 0 fsm_in_start, bit 1 tsms -- mirrors the AMS's own .def.
            state_.ams_fsm_in_start        = static_cast<std::uint8_t>(f.data[0] & 0x01u);
            state_.ams_tsms                = static_cast<std::uint8_t>((f.data[0] >> 1) & 0x01u);
            state_.last_discharge_req_tick = f.timestamp_ms;
            state_.last_ams_tick           = f.timestamp_ms;
            return true;
        }
        if (f.id == config::AcuVCellMinId) {                // 0x12C
            if (f.dlc < 2) return false;
            state_.v_cell_min_mV = decode_v_cell_min(f.data);
            state_.last_ams_tick = f.timestamp_ms;
            return true;
        }
        if (f.id == config::AmsStatusId) {                  // 0x4A0
            if (f.dlc < 8) return false;
            state_.ams_fsm_state      = decode_ams_fsm_state(f.data);
            state_.v_cell_min_mV      = decode_ams_min_cell(f.data);
            state_.module_online_mask = f.data[2];
            state_.last_ams_status_tick = f.timestamp_ms;   // 0x4A0 only
            state_.last_ams_tick = f.timestamp_ms;
            return true;
        }
        // --- uDV / autonomous. Deliberately does NOT touch last_ams_tick:
        //     uDV traffic must not keep the AMS freshness (ok_precharge) alive.
        if (f.id == config::UdvTorqueCmdId) {               // 0x507
            if (f.dlc < 4) return false;
            state_.udv_torque_cmd    = decode_udv_torque_cmd(f.data);
            state_.last_udv_cmd_tick = f.timestamp_ms;
            return true;
        }
        if (f.id == config::UdvR2dRequestId) {              // 0x510
            if (f.dlc < 1) return false;
            state_.udv_r2d_request   = f.data[0];
            state_.last_udv_r2d_tick = f.timestamp_ms;
            return true;
        }
        if (f.id == config::UdvAsStatusId) {                // 0x50A
            if (f.dlc < 1) return false;
            state_.udv_as_status    = f.data[0];
            state_.last_udv_as_tick = f.timestamp_ms;
            return true;
        }
        // --- AMS per-module telemetry (0x131-0x137, decoded for radio/dash) ---
        if (f.id == config::AcuVminModuleAId) {             // 0x131
            if (f.dlc < 6) return false;
            state_.vmin_module[0] = be16(&f.data[0]);
            state_.vmin_module[1] = be16(&f.data[2]);
            state_.vmin_module[2] = be16(&f.data[4]);
            state_.last_ams_tick  = f.timestamp_ms;
            return true;
        }
        if (f.id == config::AcuVminModuleBId) {             // 0x132
            if (f.dlc < 4) return false;
            state_.vmin_module[3] = be16(&f.data[0]);
            state_.vmin_module[4] = be16(&f.data[2]);
            state_.last_ams_tick  = f.timestamp_ms;
            return true;
        }
        if (f.id == config::AcuVmaxModuleAId) {             // 0x133
            if (f.dlc < 6) return false;
            state_.vmax_module[0] = be16(&f.data[0]);
            state_.vmax_module[1] = be16(&f.data[2]);
            state_.vmax_module[2] = be16(&f.data[4]);
            state_.last_ams_tick  = f.timestamp_ms;
            return true;
        }
        if (f.id == config::AcuVmaxModuleBId) {             // 0x134
            if (f.dlc < 4) return false;
            state_.vmax_module[3] = be16(&f.data[0]);
            state_.vmax_module[4] = be16(&f.data[2]);
            state_.last_ams_tick  = f.timestamp_ms;
            return true;
        }
        if (f.id == config::AcuCurrentsId) {                // 0x135
            if (f.dlc < 4) return false;
            state_.current_accu_dA = be16_s(&f.data[0]);
            state_.current_dcdc_dA = be16_s(&f.data[2]);
            state_.last_ams_tick      = f.timestamp_ms;
            state_.last_currents_tick = f.timestamp_ms;   // per-signal: see the header
            return true;
        }
        if (f.id == config::AcuTmaxModuleAId) {             // 0x136
            if (f.dlc < 6) return false;
            state_.tmax_module[0] = be16_s(&f.data[0]);
            state_.tmax_module[1] = be16_s(&f.data[2]);
            state_.tmax_module[2] = be16_s(&f.data[4]);
            state_.last_ams_tick  = f.timestamp_ms;
            state_.last_tmax_a_tick = f.timestamp_ms;   // 0x136 only -- see the header
            return true;
        }
        if (f.id == config::AcuTmaxModuleBId) {             // 0x137
            if (f.dlc < 6) return false;
            state_.tmax_module[3] = be16_s(&f.data[0]);
            state_.tmax_module[4] = be16_s(&f.data[2]);
            state_.tmax_dcdc      = be16_s(&f.data[4]);
            state_.last_ams_tick  = f.timestamp_ms;
            state_.last_tmax_b_tick = f.timestamp_ms;   // 0x137 only -- see the header
            return true;
        }
        return false;
    }
    return false;
}

VehicleState VehicleService::snapshot() const noexcept {
    return state_;
}

}  // namespace ecu
