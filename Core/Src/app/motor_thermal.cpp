// SPDX-License-Identifier: proprietary
//
// Motor over-temperature torque cap. See the header for the failure-mode
// reasoning -- that is the part of this that matters.

#include "app/motor_thermal.hpp"

#include "app/derate_ramp.hpp"

namespace ecu {

using namespace config;

namespace {

constexpr std::int16_t decode_degC(std::uint8_t raw) noexcept {
    return static_cast<std::int16_t>(static_cast<std::int16_t>(raw) + MotorTempRawOffsetDegC);
}

}  // namespace

bool motor_temp_raw_valid(std::uint8_t raw) noexcept {
    if (raw == MotorTempDisconnectedRaw) return false;      // inverter sentinel
    // Implausibly cold. Catches a shorted sensor AND -- the reason this exists
    // -- an untouched VehicleState, whose zeroed bytes decode to -50 degC and
    // would otherwise read as a very cold, very healthy motor.
    return decode_degC(raw) >= MotorTempMinPlausibleDegC;
}

std::uint8_t motor_thermal_cap_pct(std::int16_t temp_degC) noexcept {
    return falling_ramp_pct(temp_degC, MotorTempDerateStartDegC,
                            MotorTempLimitDegC, MotorTempFloorPct);
}

MotorThermalState MotorThermal::update(const MotorThermalInputs& in) noexcept {
    MotorThermalState st{};

    st.s1_valid = in.fresh && motor_temp_raw_valid(in.temp_motor1_raw);
    st.s2_valid = in.fresh && motor_temp_raw_valid(in.temp_motor2_raw);

    if (!st.s1_valid && !st.s2_valid) {
        // No usable reading. NOT "no derate" -- see the header. Hold a cap that
        // is drivable but cannot cook a motor nobody can see.
        seeded_      = false;
        st.unknown   = true;
        // NEVER RELAX -- same reasoning as pack_thermal.cpp. A motor genuinely
        // at 105 degC, correctly capped to 40 %, must not jump to 60 % because
        // 0x464 went quiet. Ratchets down only.
        st.cap_pct   = (MotorTempUnknownCapPct < last_.cap_pct)
                       ? MotorTempUnknownCapPct : last_.cap_pct;
        st.capped    = (st.cap_pct < 100u);
        st.temp_degC = 0;
        last_ = st;
        return st;
    }

    // Hottest of whatever is still reporting. One dead sensor must not hide a
    // hot motor on the other, so this is a max over the VALID ones only rather
    // than over both.
    std::int16_t hottest;
    if (st.s1_valid && st.s2_valid) {
        const std::int16_t t1 = decode_degC(in.temp_motor1_raw);
        const std::int16_t t2 = decode_degC(in.temp_motor2_raw);
        hottest = (t1 > t2) ? t1 : t2;
    } else {
        hottest = decode_degC(st.s1_valid ? in.temp_motor1_raw : in.temp_motor2_raw);
    }
    st.raw_max_degC = hottest;

    // The sensor is 1 degC per count and the ramp is
    // (100 - MotorTempFloorPct) / (MotorTempLimitDegC - MotorTempDerateStartDegC)
    // = 4 points of cap per degC, so a reading dithering between two counts
    // would step torque by 4 points at 100 Hz. Temperature is slow enough that a ~2.5 s filter costs nothing in
    // response and removes the dither entirely. Seeded on the first valid
    // sample -- ramping up from 0 degC would mean no protection for the first
    // few seconds after the sensor appears.
    //
    // 16 fractional bits, NOT 8. With 8 the increment (err >> 8) truncates to
    // zero as soon as the residual drops below 1 degC, so the filter stalls up
    // to a whole degree short of the real temperature -- a thermal limiter that
    // permanently under-reads. At the limit that would hold the cap 4 points
    // high -- 24 % instead of 20 %. At 16 bits one degC is 65536 counts and the increment
    // stays 256, so it converges exactly. Worst case 205 degC * 65536 = 13.4e6,
    // comfortably inside int32.
    const std::int32_t target = static_cast<std::int32_t>(hottest) << 16;
    if (!seeded_) {
        filt_q16_ = target;
        seeded_   = true;
    } else {
        filt_q16_ += (target - filt_q16_) >> MotorTempFilterShift;
    }
    // ROUNDED, not truncated. An EMA approaches its target asymptotically from
    // below, so it sits at 74.99 forever when the motor is at 75 -- and a plain
    // >> floors that to 74, one degree cool, permanently. At 8 points of cap per
    // degC that is a systematic under-protection, in the one direction that
    // matters for a thermal limit.
    st.temp_degC = static_cast<std::int16_t>((filt_q16_ + 32768) >> 16);

    st.cap_pct = motor_thermal_cap_pct(st.temp_degC);
    st.capped  = (st.cap_pct < 100u);

    last_ = st;
    return st;
}

}  // namespace ecu
