// SPDX-License-Identifier: proprietary
//
// Local I/O the ECU reads directly: the two APPS channels + brake on ADC3, and
// the START button GPIO. Read once per control cycle by ControlTask (the
// pedals are safety inputs, so they're read inline in the realtime task rather
// than published through a cross-task service -- no staleness on the torque
// path). The HAL reads live in io_signals.cpp; the debounce is pure + inline so
// the host test exercises it.

#pragma once

#include "app/ecu_config.hpp"

#include <cstdint>

namespace ecu {

struct IoInputs {
    std::uint16_t apps1_raw    = 0;   // PF8 / ADC3 IN7
    std::uint16_t apps2_raw    = 0;   // PF9 / ADC3 IN2
    std::uint16_t brake_raw    = 0;   // PF7 / ADC3 IN3
    bool          start_button = false;  // debounced
};

class IoSignals {
public:
    // Read ADC3 (brake + both APPS) and the debounced START GPIO into `out`.
    // HAL-coupled -> firmware build only (defined in io_signals.cpp).
    void read(IoInputs& out) noexcept;

    // Integrator debounce: the level must hold for StartDebounceSamples
    // consecutive reads before it flips. Pure (no HAL) -> host-testable.
    bool debounce_start(bool raw) noexcept {
        if (raw == start_stable_) {
            start_count_ = 0;
            return start_stable_;
        }
        if (++start_count_ >= config::StartDebounceSamples) {
            start_stable_ = raw;
            start_count_  = 0;
        }
        return start_stable_;
    }

private:
    std::uint8_t start_count_  = 0;
    bool         start_stable_ = false;
};

}  // namespace ecu
