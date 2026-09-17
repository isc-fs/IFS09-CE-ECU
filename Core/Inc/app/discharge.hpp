// SPDX-License-Identifier: proprietary
//
// discharge.hpp -- ECU-held DC-link discharge.
//
// THE FAILURE THIS PREVENTS. The discharge relay follows the shutdown circuit
// and nothing else: SDC open -> coil de-energised -> its NC contact closes ->
// bleed across the link. That is fail-safe and it stays as it is. What it
// cannot do is FINISH. Cycle the SDC inside the discharge time -- an e-stop
// tapped and released, a connector bounce, a driver resetting immediately --
// and the coil re-energises, the contact opens, and the bleed stops part-way.
// The link is left at an intermediate voltage with nothing draining it.
//
// If that residual sits above 95 % of pack, the AMS's precharge-complete
// criterion (dc_bus >= 95 % of pack) is already true before precharge starts.
// The resistor never carries meaningful current, and the check whose purpose is
// to prove the resistor and contactor work is satisfied by leftover charge --
// a dead precharge path passes its own self-test.
//
// TOPOLOGY. A normally-closed, ECU-driven relay sits in series with the
// discharge relay COIL, not in the bleed path:
//
//      coil energised  =  (SDC closed)  AND  (ECU permits)
//      discharging     =  (SDC open)    OR   (ECU secures)
//
// So the ECU can only ever ADD a reason to discharge; it is physically
// incapable of defeating the SDC. That is what makes adding a component to this
// path defensible. (A parallel driver would fight the SDC for the coil; a relay
// in series with the BLEED could interrupt a discharge, which is a rules
// problem.)
//
// WHO DECIDES. Split, because neither side can see everything. The AMS
// publishes two raw observations on 0x021 -- never a pre-computed request -- and
// the ECU supplies the third term, which is a measurement only it has:
//
//      secure = fsm_in_start AND tsms AND (our dc_bus > DischargeReleaseV)
//               \_________ 0x021 _________/  \____ our measurement ____/
//
// All three terms are load-bearing:
//   - fsm_in_start: TSMS-on with a charged link is also true in normal driving,
//     so without it this would bleed a live tractive system at speed. In Start
//     the AIRs are commanded open, so the same reading uniquely means "the SDC
//     was cycled and the discharge did not finish". We know neither FSM state
//     nor TSMS ourselves.
//   - dc_bus: without it, entering Start with an already-drained link and a
//     stale 0x466 secures on every entry and holds to the timeout, because the
//     release path needs a valid reading and never gets one.
//
// This is a LEVEL condition, not an edge: a stranded link holds the observations
// true for as long as it is stranded, so nothing has to catch a fast SDC
// transient.
//
// WHAT THE ECU ADDS: the hold. Latch on the observations, release only on our
// own measurement -- so one lost CAN frame mid-discharge cannot abort a bleed
// and re-strand the link. That asymmetry is the entire point of doing this here
// rather than in the AMS.
//
// RELEASE AT DischargeReleaseV, well below the AMS's own 60 V gate, so their
// re-arm is satisfied before we let go. Note this may be below what the inverter
// can report; if 0x466 stops updating on the way down, the release can never be
// confirmed and every discharge ends in the timeout instead of completing.
//
// TIMEOUT. If we secure and the link does not fall -- bleed resistor open, sense
// fault, or a measurement that stops short -- an indefinite hold would leave a
// car that never arms with nothing saying why. So give up after
// DischargeTimeoutMs, report a fault, and stop securing. Releasing is safe: the
// AMS gates on its own voltage too, so it simply will not arm, now with a reason
// attached.
//
// BOOT reports "cannot confirm" rather than forcing a discharge. Defaulting to
// engaged would be dangerous here: a watchdog reset mid-drive would boot with
// the TS live and the AIRs closed, and securing would put a transient-duty
// resistor across a live pack. Instead dc_bus_valid reads 0 until 0x466 is
// fresh, and the AMS treats "cannot confirm" as "do not arm".
//
// NO AUXILIARY CONTACT READBACK -- a team decision with a real cost. The bit on
// 0x100 reports what we COMMANDED, not what the relay did, so two failures are
// undetectable from the ECU:
//   1. Coil-interrupt relay stuck open (driver shorted on, contact welded). We
//      release the command and report engaged = 0 while the bleed is still
//      connected, and the AMS closes an AIR into a transient-duty resistor.
//   2. A discharge the SDC started on its own. The bit means "the ECU is
//      commanding one", never "the bleed is connected" -- and the AMS gate is
//      written against the second statement.
// Wiring an auxiliary pole later is one input and one line, and buys back (1).

#ifndef DISCHARGE_HPP_
#define DISCHARGE_HPP_

#include <cstdint>

#include "app/ecu_config.hpp"

namespace ecu {

struct DischargeInputs {
    // The caller conditions the 0x021 bits on that frame's freshness.
    bool          fsm_in_start   = false;  // 0x021 bit 0: AIRs and precharge all open
    bool          tsms           = false;  // 0x021 bit 1: SDC complete -> bleed disconnected
    std::uint16_t dc_bus_V       = 0;      // 0x466, relayed
    bool          dc_bus_valid   = false;  // 0x466 fresh; a held reading is not a measurement
    std::uint32_t now_ms         = 0;
};

struct DischargeState {
    bool secure  = false;  // drive the coil-interrupt relay: true = force a discharge
    // Goes on 0x100. Mirrors `secure`, kept separate so wiring an auxiliary
    // contact later changes one assignment rather than every call site.
    bool engaged = false;
    // Secured longer than DischargeTimeoutMs without the link falling.
    // Sticky until the AMS withdraws its observations.
    bool fault   = false;
};

class Discharge {
public:
    DischargeState update(const DischargeInputs& in) noexcept;

private:
    bool          secured_       = false;
    bool          fault_         = false;
    std::uint32_t secured_at_ms_ = 0;
};

}  // namespace ecu

#endif  // DISCHARGE_HPP_
