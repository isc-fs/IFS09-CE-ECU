// SPDX-License-Identifier: proprietary
//
// See gps_nmea.hpp. Parsing ported from the bench-proven GPS_TEST driver
// (gps_mtk3339.c) with the integer-only maths preserved verbatim; only the
// transport changed (byte-fed, non-blocking) and knots -> km/h added.

#include "app/gps_nmea.hpp"

namespace ecu {

namespace {

// --- tiny integer-only token helpers (no strtok/atof/snprintf) --------------

// Copy comma-separated token #idx (0-based) of `s` into tmp[]. Empty -> "".
const char* token(const char* s, char* tmp, unsigned tmp_sz, int idx) noexcept {
    unsigned cur = 0;
    const char* p = s;
    const char* start = s;
    for (;;) {
        if (*p == ',' || *p == '\0') {
            if (cur == static_cast<unsigned>(idx)) {
                unsigned n = static_cast<unsigned>(p - start);
                if (n >= tmp_sz) n = tmp_sz - 1u;
                for (unsigned i = 0; i < n; ++i) tmp[i] = start[i];
                tmp[n] = '\0';
                return tmp;
            }
            if (*p == '\0') break;
            ++cur;
            ++p;
            start = p;
            continue;
        }
        ++p;
    }
    tmp[0] = '\0';
    return tmp;
}

bool parse_u32(const char* t, std::uint32_t* out) noexcept {
    if (!t || !*t) return false;
    std::uint32_t v = 0;
    for (; *t; ++t) {
        if (*t < '0' || *t > '9') return false;
        v = v * 10u + static_cast<std::uint32_t>(*t - '0');
    }
    *out = v;
    return true;
}

// "12.34" -> 1234 (two decimals kept, extra digits discarded).
bool parse_u32_x100(const char* t, std::uint32_t* out_x100) noexcept {
    if (!t || !*t) return false;
    std::uint32_t ip = 0, fp = 0, fp_mul = 1;
    bool seen_dot = false;
    for (; *t; ++t) {
        if (*t == '.') {
            if (seen_dot) return false;
            seen_dot = true;
            continue;
        }
        if (*t < '0' || *t > '9') return false;
        const std::uint32_t d = static_cast<std::uint32_t>(*t - '0');
        if (!seen_dot) {
            ip = ip * 10u + d;
        } else if (fp_mul < 100u) {
            fp = fp * 10u + d;
            fp_mul *= 10u;
        }
    }
    while (fp_mul < 100u) { fp *= 10u; fp_mul *= 10u; }
    *out_x100 = ip * 100u + fp;
    return true;
}

// NMEA "ddmm.mmmm" + hemisphere -> degrees * 1e7. lat has 2 degree digits,
// lon has 3. degrees = dd + mm.mmmm/60, all in integer arithmetic.
bool parse_latlon_deg1e7(const char* ddmm, char hemi, bool is_lon,
                         std::int32_t* out_deg1e7) noexcept {
    if (!ddmm || !*ddmm) return false;

    const char* dot = nullptr;
    for (const char* p = ddmm; *p; ++p) {
        if (*p == '.') { dot = p; break; }
    }
    if (!dot) return false;

    const int deg_digits = is_lon ? 3 : 2;
    const int pre = static_cast<int>(dot - ddmm);
    if (pre < (deg_digits + 2)) return false;      // need dd(d) + mm before '.'

    std::int32_t deg = 0;
    for (int i = 0; i < deg_digits; ++i) {
        const char c = ddmm[i];
        if (c < '0' || c > '9') return false;
        deg = deg * 10 + (c - '0');
    }

    std::uint32_t min_ip = 0;
    for (int i = 0; i < 2; ++i) {
        const char c = ddmm[deg_digits + i];
        if (c < '0' || c > '9') return false;
        min_ip = min_ip * 10u + static_cast<std::uint32_t>(c - '0');
    }

    std::uint32_t min_fp = 0, fp_mul = 1;
    for (const char* p = dot + 1; *p && fp_mul < 10000u; ++p) {
        if (*p < '0' || *p > '9') return false;
        min_fp = min_fp * 10u + static_cast<std::uint32_t>(*p - '0');
        fp_mul *= 10u;
    }
    while (fp_mul < 10000u) { min_fp *= 10u; fp_mul *= 10u; }

    const std::uint32_t minutes_x1e4 = min_ip * 10000u + min_fp;

    // deg1e7 = deg*1e7 + minutes_x1e4 * 1e7 / (60 * 1e4)
    std::int64_t deg1e7 = static_cast<std::int64_t>(deg) * 10000000LL +
                          static_cast<std::int64_t>(minutes_x1e4) * 10000000LL /
                              (60LL * 10000LL);

    if (hemi == 'S' || hemi == 'W') deg1e7 = -deg1e7;
    *out_deg1e7 = static_cast<std::int32_t>(deg1e7);
    return true;
}

// Does `body` carry NMEA sentence type `type` (3 chars) within the talker-ID
// prefix? e.g. body "GPRMC,..." / "GNRMC,..." both match type "RMC".
bool sentence_is(const char* body, const char* type) noexcept {
    // The type sits right after a 2-char talker ID for standard sentences, but
    // accept it anywhere in the first 4 chars so odd/truncated talkers still
    // decode (same tolerance the bench driver had).
    for (int off = 0; off <= 4; ++off) {
        if (body[off] == '\0') return false;
        int i = 0;
        for (; i < 3; ++i) {
            if (body[off + i] != type[i]) break;
        }
        if (i == 3 && body[off + 3] == ',') return true;
    }
    return false;
}

}  // namespace

std::uint32_t gps_knots_x100_to_kmh_x100(std::uint32_t knots_x100) noexcept {
    // 1 kn = 1.852 km/h exactly -> *1852/1000, round to nearest.
    // Max sane input (a few hundred knots) is nowhere near overflowing u64.
    const std::uint64_t v = static_cast<std::uint64_t>(knots_x100) * 1852ULL;
    return static_cast<std::uint32_t>((v + 500ULL) / 1000ULL);
}

bool gps_parse_sentence(const char* line, GpsFix& fix) noexcept {
    if (!line || line[0] != '$') return false;

    // Copy locally and strip the "*CS" checksum tail so tokenising is simple.
    char buf[kGpsLineMax];
    unsigned n = 0;
    for (; n < (kGpsLineMax - 1u) && line[n] != '\0' && line[n] != '*'; ++n) {
        buf[n] = line[n];
    }
    buf[n] = '\0';

    const char* body = buf + 1;   // skip '$'

    if (sentence_is(body, "RMC")) {
        // $xxRMC,time,status,lat,N,lon,E,sog_knots,cog,date,...
        char t[24];

        fix.has_fix = (token(buf, t, sizeof(t), 2)[0] == 'A');

        char lat_tok[24], lat_hemi[4], lon_tok[24], lon_hemi[4];
        token(buf, lat_tok,  sizeof(lat_tok),  3);
        token(buf, lat_hemi, sizeof(lat_hemi), 4);
        token(buf, lon_tok,  sizeof(lon_tok),  5);
        token(buf, lon_hemi, sizeof(lon_hemi), 6);

        // Only trust a position when the sentence reports a valid fix -- an
        // unfixed RMC carries empty/garbage lat/lon fields.
        if (fix.has_fix) {
            (void)parse_latlon_deg1e7(lat_tok, lat_hemi[0], false, &fix.lat_deg1e7);
            (void)parse_latlon_deg1e7(lon_tok, lon_hemi[0], true,  &fix.lon_deg1e7);
        }

        char sog_tok[24], cog_tok[24];
        token(buf, sog_tok, sizeof(sog_tok), 7);
        token(buf, cog_tok, sizeof(cog_tok), 8);

        std::uint32_t sog_x100 = 0, cog_x100 = 0;
        if (parse_u32_x100(sog_tok, &sog_x100)) {
            fix.speed_kmh_x100 = gps_knots_x100_to_kmh_x100(sog_x100);
        }
        if (parse_u32_x100(cog_tok, &cog_x100)) {
            fix.course_deg_x100 = cog_x100;
        }
        return true;
    }

    if (sentence_is(body, "GGA")) {
        // $xxGGA,time,lat,N,lon,E,fix_quality,sats,hdop,...
        char fixq_tok[8], sats_tok[16];
        token(buf, fixq_tok, sizeof(fixq_tok), 6);
        token(buf, sats_tok, sizeof(sats_tok), 7);

        std::uint32_t fixq = 0, sats = 0;
        if (parse_u32(fixq_tok, &fixq)) fix.has_fix = (fixq != 0u);
        if (parse_u32(sats_tok, &sats)) {
            if (sats > 255u) sats = 255u;
            fix.sats = static_cast<std::uint8_t>(sats);
        }
        return true;
    }

    return false;
}

bool GpsNmea::feed(char c) noexcept {
    if (c == '\r') return false;                    // ignore CR

    if (c != '\n') {
        if (len_ < (kGpsLineMax - 1u)) {
            line_[len_++] = c;
        } else {
            overflow_ = true;                       // drop this line entirely
        }
        return false;
    }

    // LF -- terminate the line.
    line_[len_] = '\0';
    const std::uint16_t had = len_;
    len_ = 0;

    if (overflow_) {                                // truncated garbage, discard
        overflow_ = false;
        return false;
    }
    if (had == 0u) return false;

    if (gps_parse_sentence(line_, fix_)) {
        ++sentences_;
        return true;
    }
    return false;
}

}  // namespace ecu
