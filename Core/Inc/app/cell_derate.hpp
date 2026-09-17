// SPDX-License-Identifier: proprietary
//
// cell_derate.hpp -- turning the AMS minimum cell voltage into something worth
// derating on.
//
// A CAP, NOT A GAIN: the result is a torque CEILING, torque = min(torque, cap).
// Only the peak needs limiting -- the pack can deliver a 30 % request even when
// it cannot deliver 100 % -- so scaling the whole pedal would cost the driver
// resolution across the entire travel to bound the top of it. Capping leaves
// everything below the ceiling untouched, and matches how the thermal and power
// limiters behave.
//
// THE PROBLEM. A cell's terminal voltage under load is not a measure of how
// empty it is; it is mostly a measure of how hard you are accelerating. Pack
// current flows through every series element, so each one sags by I * R_cell.
// At 250 A and 1.5 mOhm that is 375 mV -- a cell resting at a perfectly healthy
// 3.1 V reads 2725 mV at the end of the straight and recovers the instant the
// driver lifts.
//
// Derating on that raw number does two bad things:
//
//   1. It fires on healthy packs. The derate becomes a function of throttle,
//      not of state of charge, and it engages hardest exactly where the car
//      needs the power.
//   2. It closes a loop with no damping. The sag is CAUSED by the current the
//      derate is controlling: cut torque -> current falls -> voltage recovers
//      -> derate lifts -> torque returns -> sag again. That is the ~2.5 Hz
//      limit cycle, and because pack current goes as T * omega the loop gain
//      grows linearly with rpm -- worst at the top of the straight.
//
// THE FIX. Add the ohmic drop back before deciding anything:
//
//      v_ocv_est = v_cell_min + I_pack * R_cell
//
// which is flat through an acceleration by construction -- the sag and the
// compensation move together and cancel. What is left is the slow decline that
// actually means the pack is emptying. No filter can do this job on its own: a
// straight lasts several seconds, so any time constant long enough to reject it
// is also long enough to miss a real collapse.
//
// A modest low-pass then sits on TOP of the compensation, for the part the
// ohmic model does not capture (concentration polarisation, AMS quantisation,
// one-frame noise). It is a trim, not the mechanism.
//
// FAIL-SAFE DIRECTION. Compensation makes the pack look HEALTHIER, so a wrong
// current reading is dangerous in a way a wrong voltage reading is not. Three
// guards, all of which fail towards derating sooner:
//   - stale 0x135 -> no compensation at all, derate on the raw loaded voltage
//   - the compensation is clamped to CellIrCompMaxMv
//   - a raw loaded voltage at/under CellVRawFloorMv goes straight to the floor
//     derate whatever the compensation says
//
// COMMISSIONING. R_cell is the one number that has to come from the car. It is
// measurable without a dyno: enable the pit-diag stream, drive one acceleration
// run, and plot est_ocv_mV (0x709) against time. Too low and the trace still
// dips under load; too high and it humps upward. Flat means correct.

#ifndef CELL_DERATE_HPP_
#define CELL_DERATE_HPP_

#include <cstdint>

#include "app/ecu_config.hpp"

namespace ecu {

struct CellDerateInputs {
    std::uint16_t v_cell_min_mV = 0;      // 0x12C, raw loaded minimum cell
    bool          v_fresh       = false;  // 0x12C recently received
    std::int16_t  current_dA    = 0;      // 0x135 current_accu_dA, + = discharge
    bool          i_fresh       = false;  // 0x135 recently received
};

struct CellDerateState {
    std::uint16_t est_ocv_mV   = 0;      // compensated + filtered -- what the derate uses
    std::uint16_t raw_mV       = 0;      // straight off 0x12C, for comparison on 0x709
    std::int16_t  comp_mV      = 0;      // how much IR compensation was applied
    std::uint8_t  cap_pct      = 100;    // torque CEILING in percent, 0..100
    bool          capped       = false;  // the ceiling is below full this tick
    bool          compensated  = false;  // false -> running on raw voltage (stale current)
    bool          raw_floor    = false;  // the raw-voltage backstop fired
};

// The estimator + the derate curve. Holds filter state, so it is a small object
// rather than a free function; still pure (no HAL, no clock of its own -- the
// caller owns the tick), so the SIL drives it directly.
class CellDerate {
public:
    // One control tick. Returns the full state so the caller can both apply the
    // cap and publish the intermediate values -- an estimator you cannot see is
    // an estimator you cannot commission.
    CellDerateState update(const CellDerateInputs& in) noexcept;

    // Drop the filter history, so a filter carrying seconds-old state across a
    // dropout cannot ramp through voltages the pack was never at.
    //
    // NOT CURRENTLY CALLED from anywhere: update() already unseeds itself when
    // the input goes stale or unusable, which covers the dropout case. Kept for
    // tests and for a caller that needs to force it.
    void reset() noexcept { seeded_ = false; }

    [[nodiscard]] std::uint16_t est_ocv_mV() const noexcept { return last_.est_ocv_mV; }

private:
    std::int32_t    filt_q8_ = 0;      // estimate in mV * 256
    bool            seeded_  = false;
    CellDerateState last_{};
};

// The derate curve on its own: mV -> percent cap. Free function so the SIL can
// pin the curve independently of the estimator that feeds it.
[[nodiscard]] std::uint8_t cell_derate_pct(std::uint16_t v_mV) noexcept;

}  // namespace ecu

#endif  // CELL_DERATE_HPP_
