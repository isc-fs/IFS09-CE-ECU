// SPDX-License-Identifier: proprietary
//
// NX/EMC inverter TX adapter: builds the 0x360 (mode) + 0x362 (torque) setpoints
// on FDCAN1. IDs, mode words, byte layout and the torque map are matched
// byte-for-byte to the original VCU (IFS08-CE/VCU pre-jarama) -- including its
// E2E bytes, which it sent as 0x00 (the inverter takes the setpoint without a
// CRC). The control core produces an InvMode + a torque %; this layer applies
// the inverter unit map and the mechanical sign negation.

#ifndef ECU_INVERTER_HPP_
#define ECU_INVERTER_HPP_

#include <cstdint>

#include "app/can_frame.hpp"
#include "app/control.hpp"   // InvMode

namespace ecu {

class Inverter {
public:
    // 0x360 EMC_RX_SETPOINT_1: {0, 0, App_State_Req}. App_State_Req @ byte2 is
    // the InvMode mode word. Bytes 0-1 are 0 (as the original VCU sent them).
    //
    // flt_clear sets byte 2 bit 7 (Flt_Clear in the vendor DBC), the inverter's
    // explicit fault-clear request. It shares byte 2 with App_State_Req (bits
    // 0-6), so the two are sent together. Default false -- the healthy path is
    // byte-for-byte unchanged.
    static CanFrame build_setpoint_mode(InvMode mode, bool flt_clear = false) noexcept;

    // 0x362 EMC_RX_SETPOINT_3: {0, 0, torque_lo, torque_hi}. Torque_Nm_Req is a
    // signed 16-bit LE at bytes 2-3. Bytes 0-1 are 0.
    static CanFrame build_setpoint_torque(std::uint8_t torque_pct) noexcept;

    // torque % -> Torque_Nm_Req. Maps pct (>=10) to the inverter scale, then
    // NEGATES it: the motor's mechanical mounting makes forward drive a NEGATIVE
    // setpoint (verified against the original VCU). Exposed for SIL.
    static std::int16_t torque_to_nm_req(std::uint8_t torque_pct) noexcept;
};

}  // namespace ecu

#endif  // ECU_INVERTER_HPP_
