// SPDX-License-Identifier: proprietary
//
// pack_thermal.hpp -- accumulator over-temperature torque cap.
//
// Same shape as the motor cap (motor_thermal.hpp) and for the same reason: a
// falling cap that reaches its floor AT the limit, so the pack never gets there,
// and a min() ceiling rather than a gain so the pedal keeps its resolution below
// the cap.
//
// WHAT IS DIFFERENT FROM THE MOTOR, and why this is not the same file:
//
//  - THE SENSORS. Five per-module maxima on 0x136/0x137, signed degC directly
//    (no -50 offset, unlike the inverter's byte encoding). tmax_dcdc rides along
//    on 0x137 and is deliberately IGNORED: the AMS documents it as a sentinel
//    until a real DCDC sensor is wired, and derating the car on a placeholder
//    would be worse than not derating at all.
//
//  - VALIDITY COMES FROM THE AMS. Which modules are actually reporting is
//    published as module_online_mask on 0x4A0, so this consults that rather than
//    guessing from the values. An offline module's tmax is whatever was last in
//    the slot, and stale-warm is as misleading as stale-cold.
//
//  - 0 degC IS A REAL TEMPERATURE HERE. The motor cap could reject raw 0 as
//    implausible because it decoded to -50; a pack module genuinely can sit at
//    0 degC on a cold morning. So the uninitialised-VehicleState case is caught
//    by FRESHNESS alone, which makes the 0x136/0x137 staleness window the whole
//    of the fail-safe rather than a backstop to a sentinel check.
//
//  - IT DOES NOT RECOVER WITHIN A RUN. A motor cools in a lap; an accumulator
//    has minutes of thermal mass. Once this engages it stays engaged for the
//    rest of the session, which is correct and worth knowing before someone
//    reports the car "went slow and stayed slow".
//
// Losing every module is NOT "no limit", for the same reason as the motor: an
// unmonitored pack at full power is the failure the module exists to prevent.

#ifndef PACK_THERMAL_HPP_
#define PACK_THERMAL_HPP_

#include <cstddef>
#include <cstdint>

#include "app/ecu_config.hpp"

namespace ecu {

inline constexpr std::size_t PackModuleCount = 5;

struct PackThermalInputs {
    std::int16_t tmax_module[PackModuleCount] = {};  // 0x136/0x137, degC (signed, no offset)
    std::uint8_t module_online_mask = 0;             // 0x4A0 byte 2, bit N = module N reporting
    bool         mask_valid = false;                 // 0x4A0 fresh -> trust the mask
    bool         temps_fresh = false;                // 0x136/0x137 recently received
};

struct PackThermalState {
    std::int16_t temp_degC     = 0;      // filtered hottest valid module -- what the cap uses
    std::int16_t raw_max_degC  = 0;      // unfiltered
    std::uint8_t valid_mask    = 0;      // which modules were actually used
    std::uint8_t cap_pct       = 100;
    bool         unknown       = false;  // no usable module -> PackTempUnknownCapPct
    bool         capped        = false;
};

class PackThermal {
public:
    PackThermalState update(const PackThermalInputs& in) noexcept;
    // NOT CURRENTLY CALLED: update() already unseeds itself on a stale or
    // unusable input. Kept for tests.
    void reset() noexcept { seeded_ = false; }

private:
    std::int32_t     filt_q16_ = 0;   // degC * 65536 (see motor_thermal.cpp on why not q8)
    bool             seeded_   = false;
    PackThermalState last_{};
};

[[nodiscard]] std::uint8_t pack_thermal_cap_pct(std::int16_t temp_degC) noexcept;

// A module temperature worth acting on: inside the plausible band. 0 degC passes
// -- it is a real reading here, which is exactly why freshness rather than
// value-sniffing has to carry the uninitialised case.
[[nodiscard]] bool pack_temp_plausible(std::int16_t degC) noexcept;

}  // namespace ecu

#endif  // PACK_THERMAL_HPP_
