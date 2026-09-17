// SPDX-License-Identifier: proprietary
//
// The EV 2.2.1 tractive-power envelope. Pure integer maths -- see the header
// for why this is feed-forward and why it must land before the low-cell derate
// is touched.

#include "app/power_limit.hpp"

namespace ecu {

using namespace config;

std::uint8_t power_cap_pct(std::int32_t rpm_mech) noexcept {
    // Magnitude only: the envelope applies whichever way the shaft turns.
    const std::uint32_t rpm = static_cast<std::uint32_t>(rpm_mech < 0 ? -rpm_mech : rpm_mech);
    if (rpm == 0u) return 100u;                  // stationary: P = T * 0 = 0

    // T_max = PowerCapK / rpm, with PowerCapK folding in the limit, the
    // assumed efficiency and the 60/(2*pi) rev/min -> rad/s conversion.
    const std::uint32_t t_max_nm = PowerCapK / rpm;
    if (t_max_nm >= InvTorqueFullScaleNm) return 100u;   // envelope not binding

    // Invert the torque map to get a percentage:
    //   Nm  = pct*Mul/Div - Bias/Div      (inverter.cpp)
    //   pct = (Nm*Div + Bias) / Mul
    const std::uint32_t pct =
        (t_max_nm * static_cast<std::uint32_t>(InvTorqueMapDiv) +
         static_cast<std::uint32_t>(InvTorqueMapBias)) /
        static_cast<std::uint32_t>(InvTorqueMapMul);
    return (pct > 100u) ? 100u : static_cast<std::uint8_t>(pct);
}

std::int16_t torque_pct_to_nm(std::uint8_t pct) noexcept {
    // Same map as Inverter::torque_to_nm_req, but WITHOUT the mechanical
    // negation -- this is for reporting, where a positive number meaning
    // "forward drive" is what a human wants to read.
    if (pct < DeadbandLowPct) return 0;
    return static_cast<std::int16_t>(
        static_cast<std::int32_t>(pct) * InvTorqueMapMul / InvTorqueMapDiv
        - InvTorqueMapBias / InvTorqueMapDiv);
}

}  // namespace ecu
