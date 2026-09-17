// SPDX-License-Identifier: proprietary
//
// IR-compensated low-cell derate. See the header for why the compensation is
// the mechanism and the filter is only a trim.

#include "app/cell_derate.hpp"

namespace ecu {

using namespace config;

std::uint8_t cell_derate_pct(std::uint16_t v_mV) noexcept {
    if (v_mV >= CellVDerateKneeMv) return 100u;
    if (v_mV <= CellVDerateFloorMv) return CellVDerateFloorPct;
    const std::uint32_t span  = CellVDerateKneeMv - CellVDerateFloorMv;
    const std::uint32_t above = static_cast<std::uint32_t>(v_mV - CellVDerateFloorMv);
    return static_cast<std::uint8_t>(CellVDerateFloorPct +
                                     (100u - CellVDerateFloorPct) * above / span);
}

CellDerateState CellDerate::update(const CellDerateInputs& in) noexcept {
    CellDerateState st{};

    // No voltage at all -> assume healthy and say nothing. The AMS-stale case is
    // already handled upstream by ok_precharge going false (control_task), which
    // drops the car out of the drive ladder entirely; derating on a made-up
    // voltage on top of that would only obscure the real fault.
    if (!in.v_fresh || in.v_cell_min_mV == 0u) {
        seeded_ = false;
        st.est_ocv_mV = CellVDefaultMv;
        st.raw_mV     = in.v_cell_min_mV;
        st.cap_pct    = 100u;
        st.capped     = false;
        last_ = st;
        return st;
    }

    st.raw_mV = in.v_cell_min_mV;

    // ---- 1. add the ohmic drop back ---------------------------------------
    // I[A] * R[mOhm] is millivolts directly, so with current in deciamps the
    // whole thing is (dA * mOhm) / 10 and never leaves integer arithmetic.
    //
    // Signed on purpose. Under regen the current is negative and the correction
    // pushes the estimate DOWN -- correct (a cell being charged reads above its
    // OCV) and conservative for a derate, which is the direction we want when
    // the two disagree.
    std::int32_t comp = 0;
    if (in.i_fresh && CellIrMilliOhm > 0u) {
        comp = (static_cast<std::int32_t>(in.current_dA) *
                static_cast<std::int32_t>(CellIrMilliOhm)) / 10;
        if (comp >  static_cast<std::int32_t>(CellIrCompMaxMv)) comp =  CellIrCompMaxMv;
        if (comp < -static_cast<std::int32_t>(CellIrCompMaxMv)) comp = -CellIrCompMaxMv;
        st.compensated = true;
    }
    st.comp_mV = static_cast<std::int16_t>(comp);

    std::int32_t est = static_cast<std::int32_t>(in.v_cell_min_mV) + comp;
    if (est < 0) est = 0;
    if (est > 65535) est = 65535;

    // ---- 2. trim filter ----------------------------------------------------
    // Seeded on the first sample rather than started from zero: a filter
    // climbing from 0 mV would spend its first second reporting an empty pack
    // and derate the car to the floor every time the AMS link comes up.
    const std::int32_t target = est << 8;
    if (!seeded_) {
        filt_q8_ = target;
        seeded_  = true;
    } else {
        filt_q8_ += (target - filt_q8_) >> CellVFilterShift;
    }
    st.est_ocv_mV = static_cast<std::uint16_t>(filt_q8_ >> 8);

    // ---- 3. raw-voltage backstop ------------------------------------------
    // Compensation only ever makes the pack look better, so a current sensor
    // reading high could hold the derate off indefinitely. This is the one path
    // that ignores the estimate: if the cell is ACTUALLY at or below the floor
    // on the wire, derate, and let the driver feel it rather than let the AMS
    // open the AIRs mid-corner.
    if (in.v_cell_min_mV <= CellVRawFloorMv) {
        st.raw_floor = true;
        st.cap_pct   = CellVDerateFloorPct;
    } else {
        st.cap_pct = cell_derate_pct(st.est_ocv_mV);
    }
    st.capped = (st.cap_pct < 100u);

    last_ = st;
    return st;
}

}  // namespace ecu
