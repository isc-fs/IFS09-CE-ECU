// SPDX-License-Identifier: proprietary
//
// Pure calibration validation. No HAL, no state -- compiled into the SIL build
// and used by BOTH the CAN commit path and boot-time loading, so a record that
// would be rejected over CAN can never be accepted from storage either.

#include "app/pedal_cal.hpp"

namespace ecu {

namespace {

// Span with underflow protection: an inverted calibration (max <= min) reports
// 0 here and is caught separately by AppsNotMonotonic.
std::uint16_t span_of(std::uint16_t lo, std::uint16_t hi) noexcept {
    return (hi > lo) ? static_cast<std::uint16_t>(hi - lo) : 0u;
}

}  // namespace

std::uint8_t validate_cal(const PedalCal& c) noexcept {
    std::uint8_t flags = 0;

    // Every value must be a real 12-bit ADC reading. Catches a truncated or
    // garbage record before any of the arithmetic below runs on it.
    if (c.apps1_min > CalAdcMax || c.apps1_max > CalAdcMax ||
        c.apps2_min > CalAdcMax || c.apps2_max > CalAdcMax ||
        c.brake_rest > CalAdcMax || c.brake_arm > CalAdcMax ||
        c.brake_dv_hard > CalAdcMax || c.brake_pressed > CalAdcMax) {
        flags |= cal_flag::OutOfAdcRange;
    }

    // Monotonic: rest strictly below full travel, both channels. An inverted
    // pair would make apps_pct() divide the wrong way round.
    if (c.apps1_max <= c.apps1_min || c.apps2_max <= c.apps2_min) {
        flags |= cal_flag::AppsNotMonotonic;
    }

    const std::uint16_t s1 = span_of(c.apps1_min, c.apps1_max);
    const std::uint16_t s2 = span_of(c.apps2_min, c.apps2_max);
    if (s1 < CalMinAppsSpan) flags |= cal_flag::Apps1SpanTooSmall;
    if (s2 < CalMinAppsSpan) flags |= cal_flag::Apps2SpanTooSmall;

    // The channels may differ (different sensors) but not wildly -- a large
    // disparity means one of them was captured at the wrong pedal position.
    // Only meaningful once both spans are individually sane.
    if (s1 >= CalMinAppsSpan && s2 >= CalMinAppsSpan) {
        const std::uint32_t big   = (s1 > s2) ? s1 : s2;
        const std::uint32_t small = (s1 > s2) ? s2 : s1;
        if (small * CalMaxAppsSpanRatioPct < big * 100u) {
            flags |= cal_flag::AppsSpanMismatch;
        }
    }

    // Brake span. brake_rest == 0 means "never measured" (the default), which
    // is not an error in itself -- but then the span cannot be checked, so the
    // pressed threshold alone has to carry it.
    if (c.brake_rest != 0u) {
        if (span_of(c.brake_rest, c.brake_pressed) < CalMinBrakeSpan) {
            flags |= cal_flag::BrakeSpanTooSmall;
        }
    }

    // Threshold ordering. This is load-bearing: brake_arm gates R2D,
    // brake_dv_hard gates the driverless R2D and the 0x505 verdict sent to the
    // uDV, and brake_pressed is the top of the brake_pct scale. Out of order, the
    // driverless gate could fire before the brake is meaningfully applied.
    if (!(c.brake_arm < c.brake_dv_hard && c.brake_dv_hard < c.brake_pressed)) {
        flags |= cal_flag::BrakeOrder;
    }
    // Rest must sit below the arm threshold or a released pedal reads as armed.
    if (c.brake_rest != 0u && c.brake_rest >= c.brake_arm) {
        flags |= cal_flag::BrakeOrder;
    }

    return flags;
}

std::uint8_t brake_pct(std::uint16_t raw, const PedalCal& c) noexcept {
    // Calibrated: straight span map, same shape as apps_pct.
    if (c.brake_rest != 0u && c.brake_pressed > c.brake_rest) {
        if (raw <= c.brake_rest) return 0u;
        if (raw >= c.brake_pressed) return 100u;
        return static_cast<std::uint8_t>(
            (static_cast<std::uint32_t>(raw - c.brake_rest) * 100u) /
            static_cast<std::uint32_t>(c.brake_pressed - c.brake_rest));
    }
    // Uncalibrated fallback: legacy full-range scaling, bug and all, so this
    // change is a no-op until a rest point exists.
    if (raw >= CalAdcMax) return 100u;
    return static_cast<std::uint8_t>(
        (static_cast<std::uint32_t>(raw) * 100u) / CalAdcMax);
}

std::uint16_t brake_pressure_dbar(std::uint16_t raw) noexcept {
    if (config::BrakeSensorFullScaleBar == 0u) return 0u;   // range unknown
    if (raw <= BrakeCountsAtZeroBar) return 0u;             // at or below 0 bar

    // P_dbar = Pmax * 10 * (raw - zero) / (full - zero). Multiply first;
    // worst case 250 * 10 * 3681 = 9.2e6, well inside uint32.
    constexpr std::uint32_t span = BrakeCountsAtFullBar - BrakeCountsAtZeroBar;
    const std::uint32_t num = static_cast<std::uint32_t>(config::BrakeSensorFullScaleBar) * 10u *
                              static_cast<std::uint32_t>(raw - BrakeCountsAtZeroBar);
    const std::uint32_t dbar = num / span;
    return (dbar > 0xFFFFu) ? 0xFFFFu : static_cast<std::uint16_t>(dbar);
}

}  // namespace ecu
