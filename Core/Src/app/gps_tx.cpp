// SPDX-License-Identifier: proprietary

#include "app/gps_tx.hpp"

#include "can/can_codecs.hpp"

namespace ecu {

namespace {

// Wrap an encoded payload into an ACU-bus CanFrame (mirrors udv_tx.cpp).
template <unsigned Dlc>
CanFrame make_acu(std::uint32_t id, const std::uint8_t (&buf)[Dlc]) noexcept {
    CanFrame f{};
    f.bus = static_cast<std::uint8_t>(CanBus::Acu);
    f.id  = id;
    f.dlc = static_cast<std::uint8_t>(Dlc);
    for (unsigned i = 0; i < Dlc; ++i) f.data[i] = buf[i];
    return f;
}

// Saturate a scaled-integer field into the u16 the wire carries. Speed and
// course cannot legitimately overflow (655 km/h / 655 deg), but a garbled
// sentence must not wrap into a plausible-looking small value.
std::uint16_t sat_u16(std::uint32_t v) noexcept {
    return static_cast<std::uint16_t>(v > 0xFFFFu ? 0xFFFFu : v);
}

}  // namespace

CanFrame GpsTx::build_position(const GpsFix& fix) noexcept {
    VCU_gps_position_t p{};
    p.latitude  = fix.lat_deg1e7;
    p.longitude = fix.lon_deg1e7;
    std::uint8_t b[VCU_gps_position_DLC];
    encode_VCU_gps_position(p, b);
    return make_acu(VCU_gps_position_ID, b);
}

CanFrame GpsTx::build_status(const GpsFix& fix, std::uint32_t sentences) noexcept {
    VCU_gps_status_t s{};
    s.speed_kmh  = sat_u16(fix.speed_kmh_x100);
    s.course_deg = sat_u16(fix.course_deg_x100);
    s.sats       = fix.sats;
    s.has_fix    = fix.has_fix ? 1u : 0u;
    // Free-running liveness counter -- wraps at 16 bits, which is fine: the
    // consumer only cares that it CHANGES.
    s.nmea_count = static_cast<std::uint16_t>(sentences & 0xFFFFu);
    std::uint8_t b[VCU_gps_status_DLC];
    encode_VCU_gps_status(s, b);
    return make_acu(VCU_gps_status_ID, b);
}

}  // namespace ecu
