// SPDX-License-Identifier: proprietary
//
// motor_thermal.hpp -- motor over-temperature torque limiting.
//
// The motor limit is MotorTempLimitDegC (110 degC). A limit is not a trip point:
// reaching it and then cutting is too late, because the winding is already
// there. So the cap starts backing torque off at MotorTempDerateStartDegC
// (90 degC) and reaches its floor AT the limit, which is what keeps the motor
// from getting there in the first place.
//
// A CAP, NOT A GAIN. torque = min(torque, cap), so everything below the cap
// passes through untouched and the driver keeps full pedal resolution over the
// range that is still allowed. Every torque limiter in the core works this way,
// which is also why their order does not matter: the lowest ceiling wins.
//
// Heat goes as torque squared, so the floor does not need to be near zero to
// work: at MotorTempFloorPct = 20 % the motor generates ~4 % of the heat it does
// at full torque, which cools it under any realistic load while still leaving
// enough drive to get the car off track under its own power. A thermal limiter
// that strands the car has traded one failure for another.
//
// SENSOR FAILURE IS THE HARD PART, and the two failure modes point in opposite
// directions:
//
//   0xFF is the inverter's "sensor disconnected" sentinel. Decoded naively with
//   the -50 offset it reads 205 degC and would slam the cap to the floor -- a
//   disconnected wire would look identical to a cooking motor.
//
//   Raw 0 decodes to -50 degC, and 0 is also what an untouched VehicleState
//   holds before any 0x464 has ever arrived. Naively that reads "very cold", so
//   a motor with no temperature reporting at all would run at full power
//   forever. That is the dangerous one, and it is the default state at boot.
//
// Both are rejected explicitly, and losing every sensor is NOT treated as
// "no derate". With no usable reading the cap sits at MotorTempUnknownCapPct --
// enough to drive, not enough to cook a motor nobody is watching. Annunciated on
// 0x706 so it is visible rather than inferred from the car feeling slow.

#ifndef MOTOR_THERMAL_HPP_
#define MOTOR_THERMAL_HPP_

#include <cstdint>

#include "app/ecu_config.hpp"

namespace ecu {

struct MotorThermalInputs {
    std::uint8_t temp_motor1_raw = 0;   // 0x464 byte 2, raw (degC = raw - 50)
    std::uint8_t temp_motor2_raw = 0;   // 0x464 byte 3, raw
    bool         fresh           = false;  // 0x464 recently received (its OWN window)
};

struct MotorThermalState {
    std::int16_t temp_degC     = 0;      // filtered, the value the cap is computed from
    std::int16_t raw_max_degC  = 0;      // unfiltered max of the valid sensors
    std::uint8_t cap_pct       = 100;
    bool         s1_valid      = false;
    bool         s2_valid      = false;
    bool         unknown       = false;  // no usable sensor -> MotorTempUnknownCapPct
    bool         capped        = false;  // the cap is actually limiting this tick
};

class MotorThermal {
public:
    // One control tick. Returns the whole state so the caller can apply the cap
    // and publish the inputs that produced it.
    MotorThermalState update(const MotorThermalInputs& in) noexcept;

    // NOT CURRENTLY CALLED: update() already unseeds itself on a stale or
    // unusable input. Kept for tests.
    void reset() noexcept { seeded_ = false; }

private:
    std::int32_t      filt_q16_ = 0;   // degC * 65536 -- see the .cpp for why not 256
    bool              seeded_  = false;
    MotorThermalState last_{};
};

// degC -> percent cap. Free function so the SIL pins the curve without driving
// the sensor-validity logic through it.
[[nodiscard]] std::uint8_t motor_thermal_cap_pct(std::int16_t temp_degC) noexcept;

// True when a raw 0x464 temperature byte carries a usable reading: not the
// disconnected sentinel, and not implausibly cold (which catches an untouched
// VehicleState as well as a shorted sensor).
[[nodiscard]] bool motor_temp_raw_valid(std::uint8_t raw) noexcept;

}  // namespace ecu

#endif  // MOTOR_THERMAL_HPP_
