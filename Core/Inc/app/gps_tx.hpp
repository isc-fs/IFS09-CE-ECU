// SPDX-License-Identifier: proprietary
//
// GPS -> CAN frame builders (0x508 position / 0x509 status), ACU bus.
// Pure like udv_tx: no HAL, no RTOS, so the wire layout is host-testable.

#ifndef ECU_GPS_TX_HPP_
#define ECU_GPS_TX_HPP_

#include "app/can_frame.hpp"
#include "app/gps_nmea.hpp"

#include <cstdint>

namespace ecu {

struct GpsTx {
    // 0x508 -- lat/lon as degrees*1e7, s32 LE. Carries the LAST VALID position
    // when the fix is lost; consumers gate on 0x509.has_fix.
    static CanFrame build_position(const GpsFix& fix) noexcept;

    // 0x509 -- speed (km/h*100), course (deg*100), sats, has_fix, plus the
    // parsed-sentence counter as a wiring/liveness signal.
    static CanFrame build_status(const GpsFix& fix, std::uint32_t sentences) noexcept;
};

}  // namespace ecu

#endif  // ECU_GPS_TX_HPP_
