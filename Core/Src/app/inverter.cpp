// SPDX-License-Identifier: proprietary

#include "app/inverter.hpp"

#include "app/ecu_config.hpp"

namespace ecu {

using namespace config;

std::int16_t Inverter::torque_to_nm_req(std::uint8_t torque_pct) noexcept {
    if (torque_pct < DeadbandLowPct) return 0;          // below the deadband -> no torque
    // Map: pct*Mul/Div - Bias/Div, re-based so DeadbandLowPct(5) -> 0 Nm and
    // 100 -> 240 Nm. The bias is what puts zero exactly at the deadband edge.
    const std::int32_t mapped =
        static_cast<std::int32_t>(torque_pct) * InvTorqueMapMul / InvTorqueMapDiv
        - InvTorqueMapBias / InvTorqueMapDiv;
    // NEGATE -- a MECHANICAL constraint of the motor (its mounting): forward
    // drive is a NEGATIVE Torque_Nm_Req. Do NOT remove; the car would drive the
    // wrong way. (Matches the original VCU two's-complement step.)
    return static_cast<std::int16_t>(-mapped);
}

CanFrame Inverter::build_setpoint_mode(InvMode mode, bool flt_clear) noexcept {
    CanFrame f{};                                       // zero-inits all bytes
    f.bus  = static_cast<std::uint8_t>(CanBus::Inv);
    f.id   = InvTxSetpointModeId;                       // 0x360
    f.dlc  = InvTxSetpointModeDlc;                      // 3
    // bytes 0-1 stay 0 -- the original VCU sent the setpoint as {0, 0, mode}.
    // byte 2 packs App_State_Req (bits 0-6) with Flt_Clear (bit 7).
    f.data[2] = static_cast<std::uint8_t>(
        static_cast<std::uint8_t>(mode) | (flt_clear ? InvFltClearBit : 0u));
    return f;
}

CanFrame Inverter::build_setpoint_torque(std::uint8_t torque_pct) noexcept {
    const std::uint16_t nm = static_cast<std::uint16_t>(torque_to_nm_req(torque_pct));
    CanFrame f{};
    f.bus  = static_cast<std::uint8_t>(CanBus::Inv);
    f.id   = InvTxSetpointTorqueId;                     // 0x362
    f.dlc  = InvTxSetpointTorqueDlc;                    // 4
    // bytes 0-1 stay 0 -- {0, 0, torque_lo, torque_hi}, as the original VCU.
    f.data[2] = static_cast<std::uint8_t>(nm & 0xFFu);         // Torque_Nm_Req low (LE)
    f.data[3] = static_cast<std::uint8_t>((nm >> 8) & 0xFFu);  // high
    return f;
}

}  // namespace ecu
