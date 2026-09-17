// SPDX-License-Identifier: proprietary
//
// Accumulator over-temperature torque cap. See the header for what differs from
// the motor cap -- the sensor validity rules are the substance here.

#include "app/pack_thermal.hpp"

#include "app/derate_ramp.hpp"

namespace ecu {

using namespace config;

bool pack_temp_plausible(std::int16_t degC) noexcept {
    return degC >= PackTempMinPlausibleDegC && degC <= PackTempMaxPlausibleDegC;
}

std::uint8_t pack_thermal_cap_pct(std::int16_t temp_degC) noexcept {
    return falling_ramp_pct(temp_degC, PackTempDerateStartDegC,
                            PackTempLimitDegC, PackTempFloorPct);
}

PackThermalState PackThermal::update(const PackThermalInputs& in) noexcept {
    PackThermalState st{};

    std::int16_t hottest = 0;
    bool any = false;

    if (in.temps_fresh) {
        for (std::size_t i = 0; i < PackModuleCount; ++i) {
            // The AMS knows which modules are talking; ask it rather than infer.
            // An offline module's slot still holds whatever arrived last, and
            // stale-warm misleads exactly as much as stale-cold.
            //
            // When the mask itself is unavailable (0x4A0 not fresh) fall back to
            // plausibility alone rather than treating everything as offline: a
            // missing mask must not manufacture a thermal fault out of five
            // perfectly good temperature readings.
            if (in.mask_valid &&
                (in.module_online_mask & static_cast<std::uint8_t>(1u << i)) == 0u) {
                continue;
            }
            const std::int16_t t = in.tmax_module[i];
            if (!pack_temp_plausible(t)) continue;
            st.valid_mask = static_cast<std::uint8_t>(st.valid_mask | (1u << i));
            if (!any || t > hottest) { hottest = t; any = true; }
        }
    }

    if (!any) {
        // Nothing usable. NOT "no limit" -- an unmonitored pack at full power is
        // the failure this exists to prevent. Note this is reached by STALENESS
        // in the uninitialised case, not by a value check: 0 degC is a real pack
        // temperature and cannot be rejected the way the motor's -50 could.
        seeded_      = false;
        st.unknown   = true;
        // NEVER RELAX. Losing sight of the pack must not hand the car more
        // torque than it had while the pack was visible: a pack genuinely at
        // 48 degC, correctly capped to 36 %, would otherwise JUMP to 60 % the
        // moment 0x136/0x137 went quiet -- 78 % more torque on a pack nobody can
        // see any more. Ratchets down only; a real reading restores the ramp.
        st.cap_pct   = (PackTempUnknownCapPct < last_.cap_pct)
                       ? PackTempUnknownCapPct : last_.cap_pct;
        st.capped    = (st.cap_pct < 100u);
        st.temp_degC = 0;
        last_ = st;
        return st;
    }

    st.raw_max_degC = hottest;

    // q16 and a ROUNDED readout, both for the reasons spelled out in
    // motor_thermal.cpp: q8 stalls the filter a whole degree short, and a
    // truncating readout leaves an EMA reading one degree cool forever. Both
    // biases point the same dangerous way -- pack cooler than it is.
    const std::int32_t target = static_cast<std::int32_t>(hottest) << 16;
    if (!seeded_) {
        filt_q16_ = target;
        seeded_   = true;
    } else {
        filt_q16_ += (target - filt_q16_) >> PackTempFilterShift;
    }
    st.temp_degC = static_cast<std::int16_t>((filt_q16_ + 32768) >> 16);

    st.cap_pct = pack_thermal_cap_pct(st.temp_degC);
    st.capped  = (st.cap_pct < 100u);

    last_ = st;
    return st;
}

}  // namespace ecu
