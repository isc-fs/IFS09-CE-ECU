// SPDX-License-Identifier: proprietary
//
// pedal_cal.hpp -- the pedal calibration set, as RUNTIME data.
//
// These eight values are not constants, so an operator can
// calibrate over CAN from the pit tool instead of editing a header, rebuilding
// and reflashing. That matters because a calibration only a firmware engineer
// can perform is a calibration that does not get performed.
//
// HAL-free and dependency-free on purpose: the pure control core includes this,
// so it must stay host-compilable for SIL. The defaults in ecu_config.hpp apply
// when no valid record is stored.
//
// NOT calibratable and deliberately still compile-time: the deadbands, the
// FSAE plausibility percentages (AppsDisagree*) and every timing. Those
// are design decisions, not per-car measurements, and an operator must not be
// able to move them.

#ifndef PEDAL_CAL_HPP_
#define PEDAL_CAL_HPP_

#include <cstdint>

#include "app/ecu_config.hpp"

namespace ecu {

// The full calibration record. Field order is the on-the-wire order for
// 0x7E4 (the four APPS values) and 0x7E5 (the four brake values).
struct PedalCal {
    // --- APPS: raw ADC at rest and at the mechanical stop, per channel ---
    std::uint16_t apps1_min = config::Apps1AdcMin;
    std::uint16_t apps1_max = config::Apps1AdcMax;
    std::uint16_t apps2_min = config::Apps2AdcMin;
    std::uint16_t apps2_max = config::Apps2AdcMax;
    // --- Brake: the span, plus the three thresholds derived from it ---
    // brake_rest is 0 in the defaults because it has never been measured; 0
    // means "span unknown" and callers fall back to their previous behaviour.
    std::uint16_t brake_rest    = config::BrakeRestRaw;
    std::uint16_t brake_arm     = config::BrakeArmRaw;
    std::uint16_t brake_dv_hard = config::BrakeDvHardRaw;
    std::uint16_t brake_pressed = config::BrakePressedRaw;
};

// Validation flag bits, mirrored onto 0x7E3 byte 4 so the pit tool can tell the
// operator which rule failed. 0 = the candidate calibration is acceptable.
namespace cal_flag {
inline constexpr std::uint8_t Apps1SpanTooSmall = 1u << 0;
inline constexpr std::uint8_t Apps2SpanTooSmall = 1u << 1;
inline constexpr std::uint8_t AppsSpanMismatch  = 1u << 2;
inline constexpr std::uint8_t AppsNotMonotonic  = 1u << 3;
inline constexpr std::uint8_t BrakeSpanTooSmall = 1u << 4;
inline constexpr std::uint8_t BrakeOrder        = 1u << 5;
inline constexpr std::uint8_t OutOfAdcRange     = 1u << 6;
}  // namespace cal_flag

// Minimum usable travel, in raw 12-bit counts. Today's calibration spans 860
// (APPS1) and 680 (APPS2); anything under a few hundred counts means the pedal
// was not swept properly or a sensor is not moving, and would turn ADC noise
// into torque.
inline constexpr std::uint16_t CalMinAppsSpan  = 300;
inline constexpr std::uint16_t CalMinBrakeSpan = 300;
// The two APPS channels are different sensors and legitimately have different
// spans (860 vs 680 today, a ratio of 1.26). A much larger disparity means one
// channel was mis-captured. Expressed as a percentage of the LARGER span.
inline constexpr std::uint16_t CalMaxAppsSpanRatioPct = 200;
inline constexpr std::uint16_t CalAdcMax = 4095;   // 12-bit

// Validate a candidate calibration. Returns a bitmask of cal_flag::*; 0 means
// acceptable. Pure -- no HAL, no state -- so the SIL suite covers it directly
// and the same function gates both the CAN commit path and boot-time loading.
//
// LIMITATION, deliberate and documented: rest+full endpoints alone CANNOT
// detect the failure T.11.8.9 exists to catch. Both channels are normalised to
// 0..100 % by construction, so they agree at the endpoints no matter how badly
// they diverge in between -- mid-travel divergence comes from sensor
// non-linearity, which endpoint capture cannot see. AppsSpanMismatch is a weak
// proxy. Catching the real thing needs either a mid-travel capture point or a
// post-commit verification sweep.
[[nodiscard]] std::uint8_t validate_cal(const PedalCal& c) noexcept;

// Brake travel as a percentage, 0..100.
//
// Uses the CALIBRATED span (brake_rest..brake_pressed) when a rest point has
// been captured. Until then brake_rest is 0 and this falls back to the legacy
// full-12-bit-range scaling, which is what shipped before and is why a released
// brake reads about 14 % in the pit tool today (raw ~560 of 4095). That number
// cannot be fixed without measuring rest -- scaling to a span nobody has
// measured would just be a different wrong answer -- so it stays until an
// operator captures BRAKE_REST.
[[nodiscard]] std::uint8_t brake_pct(std::uint16_t raw, const PedalCal& c) noexcept;

// ---- Brake pressure, EPT1400 -> bar -----------------------------------------
//
// Sensor: Variohm EuroSensor EPT1400 (docs/EPT1400_pressure_sensor.pdf).
// Ratiometric, 0.5 V at zero pressure, 4.5 V at full scale, linear between.
//
// Board conditioning ahead of the ADC (schematic): R8 1k in series, R9 2k to
// ground, a 3V3 TVS clamp and a 10 pF cap. So the ADC sees the sensor through a
// resistive divider of
//
//     k = R9 / (R8 + R9) = 2 / 3
//
// Because the divider AND Vref are known, pressure is an ABSOLUTE map from raw
// counts -- it needs no calibration and no brake_rest anchor:
//
//     counts(V) = V * k / Vref * 4095
//     0.5 V -> 414 counts      4.5 V -> 3723 counts
//     P = Pmax * (counts - 414) / (3723 - 414)
//
// Note 3723 is comfortably inside the 12-bit range, so the top of the sensor's
// travel does NOT clip.
inline constexpr std::uint32_t BrakeDivR8Ohm = 1000;   // series
inline constexpr std::uint32_t BrakeDivR9Ohm = 2000;   // shunt
// Derived from the divider, Vref and the sensor's two defined points. Kept as
// explicit constants so the arithmetic below stays integer, with a SIL test that
// re-derives them from the resistor values and fails if the board changes and
// these do not.
inline constexpr std::uint16_t BrakeCountsAtZeroBar = 414;
inline constexpr std::uint16_t BrakeCountsAtFullBar = 3723;

// Brake pressure in DECI-BAR (0.1 bar, matching the 0x705 signal scale).
//
// Returns 0 when the sensor's full-scale range is unknown
// (config::BrakeSensorFullScaleBar == 0) -- reporting an invented pressure
// would be worse than reporting none, since 0x705 is what a scrutineer or the
// dashboard would believe. That range comes from the order code and is the only
// value the maths cannot derive.
[[nodiscard]] std::uint16_t brake_pressure_dbar(std::uint16_t raw) noexcept;

}  // namespace ecu

#endif  // PEDAL_CAL_HPP_
