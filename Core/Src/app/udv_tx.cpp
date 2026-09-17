// SPDX-License-Identifier: proprietary

#include "app/udv_tx.hpp"

#include "app/ecu_config.hpp"

#include "can/can_codecs.hpp"

namespace ecu {

namespace {

// Wrap an encoded payload into an ACU-bus CanFrame (the pit_diag helper).
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

CanFrame UdvTx::build_ts_active(bool ts_active) noexcept {
    VCU_ts_active_t t{};
    t.ts_active = ts_active ? 1u : 0u;
    std::uint8_t b[VCU_ts_active_DLC];
    encode_VCU_ts_active(t, b);
    return make_acu(VCU_ts_active_ID, b);
}

CanFrame UdvTx::build_brake_over_limit(bool over_limit) noexcept {
    VCU_brake_over_limit_t v{};
    v.brake_over_limit = over_limit ? 1u : 0u;
    std::uint8_t b[VCU_brake_over_limit_DLC];
    encode_VCU_brake_over_limit(v, b);
    return make_acu(VCU_brake_over_limit_ID, b);
}

CanFrame UdvTx::build_motor_rpm(std::int32_t erpm) noexcept {
    // The inverter reports ELECTRICAL rpm (EMachine_Speed_erpm); uDV's SLAM
    // wants the MECHANICAL shaft speed. Convert here -- the pole-pair count is
    // motor knowledge the ECU owns; uDV never needs it. Integer division
    // truncates toward zero for both signs (C++), fine at 1-rpm resolution.
    VCU_motor_rpm_t m{};
    m.motor_rpm = erpm / config::MotorPolePairs;
    std::uint8_t b[VCU_motor_rpm_DLC];
    encode_VCU_motor_rpm(m, b);
    return make_acu(VCU_motor_rpm_ID, b);
}

CanFrame UdvTx::build_r2d_confirm(bool confirmed) noexcept {
    VCU_r2d_confirm_t c{};
    c.r2d_confirm = confirmed ? 1u : 0u;
    std::uint8_t b[VCU_r2d_confirm_DLC];
    encode_VCU_r2d_confirm(c, b);
    return make_acu(VCU_r2d_confirm_ID, b);
}

}  // namespace ecu
