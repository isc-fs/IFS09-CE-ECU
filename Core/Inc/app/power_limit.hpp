// SPDX-License-Identifier: proprietary
//
// power_limit.hpp -- the FS-Rules EV 2.2.1 tractive-power envelope.
//
// FS-Rules 2026 EV 2.2.1: "The TS power at the outlet of the TSAC must not
// exceed 80 kW." D 10.4.1 judges it on a 500 ms moving average taken by the
// officials' data logger. Before this, NOTHING in the vehicle enforced it --
// not the ECU, not the inverter, not the AMS -- and the torque map commands
// roughly twice the legal power over most of the usable speed range.
//
// FEED-FORWARD, NOT FEEDBACK. The cap is computed from motor speed alone:
//
//     P = T * omega   ->   T_max(omega) = P_limit * eta / omega
//
// It reads a signal the cap does not influence on the timescale that matters
// (speed is set by vehicle inertia, not by this tick's torque), so unlike the
// low-cell derate it closes no loop and cannot oscillate. That property is the
// whole reason this lands FIRST: the low-cell derate is currently the only
// thing limiting power, and every proposed fix to it RELEASES power, so the
// envelope has to exist before that derate is touched.
//
// Integer-only and pure, so the control core stays host-testable and free of
// floating point in the 10 ms path.
//
// WHAT THIS IS NOT. It is an open-loop envelope derived from commanded torque
// and measured speed, using an ASSUMED drivetrain efficiency. It is not a
// measurement of delivered power and it does not replace measuring the real
// 500 ms average on the car. Treat it as
// "cannot obviously exceed the limit", not as proof of compliance.

#ifndef POWER_LIMIT_HPP_
#define POWER_LIMIT_HPP_

#include <cstdint>

#include "app/ecu_config.hpp"

namespace ecu {

// Torque cap in percent (0..100) for a given MECHANICAL motor speed.
//
// Returns 100 below the speed at which full torque would breach the envelope,
// so the cap is inert at low speed and in normal cornering -- it only engages
// where the physics say it must. Sign of rpm is ignored: the envelope applies
// to power magnitude in either direction of rotation.
//
// At the shipped constants (76 kW, 90 % assumed efficiency, 240 Nm full scale)
// the cap starts biting at roughly 2721 mechanical rpm.
[[nodiscard]] std::uint8_t power_cap_pct(std::int32_t rpm_mech) noexcept;

// Commanded shaft torque in Nm for a torque percentage, using the same map the
// inverter adapter uses. Exposed so the diagnostic stream can report what is
// actually being commanded rather than a percentage -- 0x700 torque_cmd was
// hardcoded to 0 before this.
[[nodiscard]] std::int16_t torque_pct_to_nm(std::uint8_t pct) noexcept;

}  // namespace ecu

#endif  // POWER_LIMIT_HPP_
