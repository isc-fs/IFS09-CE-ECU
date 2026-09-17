// SPDX-License-Identifier: proprietary
//
// gps_nmea -- NMEA 0183 parser for the MTK3339 GPS on USART10 (PG11 RX /
// PG12 TX, 9600 8N1). Position + speed over ground for telemetry, the pit
// tool and the uDV.
//
// Ported from the bench-proven GPS_TEST driver (IFS08_PRIVATE/GPS_TEST,
// Core/Src/gps_mtk3339.c): the sentence parsing is kept algorithmically
// IDENTICAL -- integer-only, no atof/float anywhere -- because that version
// was validated against a live module. What changed is the TRANSPORT: the
// test driver pulled bytes with a blocking HAL_UART_Receive, which would
// stall an RTOS task. Here the parser is a byte-fed state machine (feed())
// with no HAL and no blocking at all; gps_task.cpp drains the DMA ring and
// pushes bytes in.
//
// Being HAL-free/RTOS-free (like control.cpp / radio_snapshot.cpp) makes the
// risky part -- the sentence parsing -- directly host-testable in SIL against
// real captured sentences.
//
// Sentences consumed:
//   RMC (GPRMC/GNRMC) -- fix status, lat, lon, speed over ground, course
//   GGA (GPGGA/GNGGA) -- fix quality + satellites in view
// Both are matched on the 3-letter type so any talker ID (GP/GN/GL/...) works.

#ifndef ECU_GPS_NMEA_HPP_
#define ECU_GPS_NMEA_HPP_

#include <cstdint>

namespace ecu {

// Longest NMEA sentence is 82 chars incl. CRLF; 128 leaves headroom.
inline constexpr unsigned kGpsLineMax = 128u;

// One parsed GPS solution. Scaled integers only -- no float on the wire or in
// the firmware. Speed is km/h (the car's unit), converted from the NMEA knots.
struct GpsFix {
    bool          has_fix         = false;  // RMC status 'A' / GGA quality != 0
    std::uint8_t  sats            = 0;      // GGA satellites in view
    std::int32_t  lat_deg1e7      = 0;      // degrees * 1e7, +N / -S
    std::int32_t  lon_deg1e7      = 0;      // degrees * 1e7, +E / -W
    std::uint32_t speed_kmh_x100  = 0;      // km/h * 100 (converted from knots)
    std::uint32_t course_deg_x100 = 0;      // course over ground, deg * 100
};

// NMEA knots*100 -> km/h*100. 1 knot = 1.852 km/h exactly, so the conversion
// is the exact integer ratio 1852/1000 with round-to-nearest. Exposed for SIL.
std::uint32_t gps_knots_x100_to_kmh_x100(std::uint32_t knots_x100) noexcept;

// Parse ONE complete NMEA sentence (leading '$', checksum optional -- it is
// stripped, not verified: the UART framing plus the sentence-type match is the
// integrity check the bench version relied on). Updates the matching fields of
// `fix` and returns true if the sentence was a recognised RMC/GGA.
//
// Pure + reentrant -- this is the SIL entry point.
bool gps_parse_sentence(const char* line, GpsFix& fix) noexcept;

// Byte-fed line assembler + parser. No HAL, no blocking, no allocation.
class GpsNmea {
public:
    // Feed one received byte. Returns true when that byte completed a sentence
    // that parsed as a recognised RMC/GGA (i.e. fix() just changed).
    // CR is ignored; LF terminates. An over-long line is dropped, not truncated.
    bool feed(char c) noexcept;

    const GpsFix& fix() const noexcept { return fix_; }

    // Sentences successfully parsed since boot -- liveness for pit-diag: a GPS
    // that is wired but has no sky still increments this (RMC with status 'V').
    std::uint32_t sentences() const noexcept { return sentences_; }

private:
    char          line_[kGpsLineMax] = {};
    std::uint16_t len_               = 0;
    bool          overflow_          = false;  // drop the rest of an over-long line
    GpsFix        fix_{};
    std::uint32_t sentences_         = 0;
};

}  // namespace ecu

#endif  // ECU_GPS_NMEA_HPP_
