// SPDX-License-Identifier: proprietary
//
// Read-only parser for the bootloader NVM calibration record. See the header
// for why the app parses the format itself and why that is acceptable for reads
// but NOT for writes.

#include "app/pedal_cal_nvm.hpp"

namespace ecu {

namespace {

// The bootloader's 32-byte entry, read field by field rather than by casting a
// struct over flash: this build's packing/alignment must not silently decide
// how another project's on-flash format is interpreted.
//   0..1  magic (LE)   2..3 key (LE)   4 len   5 reserved_a
//   6..7  reserved_b   8..11 seq (LE)  12..31  value[20]
std::uint16_t rd_u16le(const std::uint8_t* p) noexcept {
    return static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8));
}
std::uint32_t rd_u32le(const std::uint8_t* p) noexcept {
    return static_cast<std::uint32_t>(p[0]) |
           (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) |
           (static_cast<std::uint32_t>(p[3]) << 24);
}
std::uint16_t rd_u16be(const std::uint8_t* p) noexcept {
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(p[0]) << 8) | p[1]);
}

}  // namespace

void encode_cal_record(const PedalCal& c, std::uint8_t out[CalRecordLen]) noexcept {
    out[0] = CalRecordVersion;
    out[1] = 0;
    const std::uint16_t v[8] = {
        c.apps1_min, c.apps1_max, c.apps2_min, c.apps2_max,
        c.brake_rest, c.brake_arm, c.brake_dv_hard, c.brake_pressed,
    };
    for (std::size_t i = 0; i < 8; ++i) {
        out[2 + i * 2]     = static_cast<std::uint8_t>(v[i] >> 8);
        out[2 + i * 2 + 1] = static_cast<std::uint8_t>(v[i] & 0xFFu);
    }
}

std::uint32_t cal_crc32(const PedalCal& c) noexcept {
    std::uint8_t rec[CalRecordLen];
    encode_cal_record(c, rec);
    // Bitwise rather than table-driven: 18 bytes is 144 iterations, which is
    // nothing next to 1 KB of table in a flash-constrained image.
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < CalRecordLen; ++i) {
        crc ^= rec[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

CalLoadResult load_cal_from_nvm(const void* nvm_base, std::uint32_t nvm_size) noexcept {
    CalLoadResult r{};                      // defaults, status = Defaults

    if (nvm_base == nullptr || nvm_size < CalNvmEntrySize) return r;

    const auto* base = static_cast<const std::uint8_t*>(nvm_base);
    const std::uint32_t n_entries = nvm_size / CalNvmEntrySize;

    // Highest seq wins, exactly as the bootloader resolves it. Scan the whole
    // region rather than stopping at the first hit: the store is append-only,
    // so a newer record for the same key sits FURTHER in, and an early exit
    // would load a stale calibration.
    const std::uint8_t* best = nullptr;
    std::uint32_t best_seq = 0;
    bool found = false;

    for (std::uint32_t i = 0; i < n_entries; ++i) {
        const std::uint8_t* e = base + static_cast<std::size_t>(i) * CalNvmEntrySize;
        if (rd_u16le(e) != CalNvmMagic)  continue;   // erased, torn or stale
        if (rd_u16le(e + 2) != CalNvmKey) continue;  // someone else's key
        const std::uint8_t len = e[4];
        if (len == 0u) {                             // tombstone: calibration deleted
            const std::uint32_t seq = rd_u32le(e + 8);
            if (!found || seq >= best_seq) { found = true; best_seq = seq; best = nullptr; }
            continue;
        }
        if (len < CalRecordLen) continue;            // truncated -- cannot be ours
        const std::uint32_t seq = rd_u32le(e + 8);
        if (!found || seq >= best_seq) { found = true; best_seq = seq; best = e + 12; }
    }

    if (best == nullptr) return r;                   // nothing stored, or tombstoned

    if (best[0] != CalRecordVersion) {
        r.status = CalLoad::BadVersion;
        return r;                                    // defaults, and say why
    }

    PedalCal c{};
    c.apps1_min     = rd_u16be(best + 2);
    c.apps1_max     = rd_u16be(best + 4);
    c.apps2_min     = rd_u16be(best + 6);
    c.apps2_max     = rd_u16be(best + 8);
    c.brake_rest    = rd_u16be(best + 10);
    c.brake_arm     = rd_u16be(best + 12);
    c.brake_dv_hard = rd_u16be(best + 14);
    c.brake_pressed = rd_u16be(best + 16);

    // The SAME validation that gates the CAN commit path. A record that could
    // not have been committed over CAN must not be accepted from storage
    // either -- otherwise corrupted flash becomes a way around the safety rules.
    const std::uint8_t flags = validate_cal(c);
    if (flags != 0u) {
        r.status = CalLoad::Invalid;
        r.flags  = flags;
        return r;                                    // defaults
    }

    r.cal    = c;
    r.status = CalLoad::Loaded;
    return r;
}

bool find_cal_write_slot(const void* nvm_base, std::uint32_t nvm_size,
                         std::uint32_t* out_offset, std::uint32_t* out_seq) noexcept {
    if (nvm_base == nullptr || out_offset == nullptr || out_seq == nullptr) return false;
    if (nvm_size < CalNvmEntrySize) return false;

    const auto* base = static_cast<const std::uint8_t*>(nvm_base);
    const std::uint32_t n = nvm_size / CalNvmEntrySize;

    // Append point = ONE PAST THE LAST MAGIC ENTRY, which is exactly what
    // bl_nvm_init does (bootloader Core/Src/bl_nvm.c, bl_nvm_init). NOT the
    // first erased slot: the bootloader tolerates stray non-magic patterns
    // mid-sector precisely so that a torn write cannot make a later append
    // land BEFORE live entries and shadow them. Matching its rule is what
    // keeps the two implementations interchangeable.
    //
    // seq is GLOBAL across every key (bl_nvm_write does `entry.seq =
    // ++g_max_seq`), so ours must beat the highest in the store, not just the
    // highest of our own key.
    std::uint32_t last_live = 0;
    bool any_live = false;
    std::uint32_t max_seq = 0;

    for (std::uint32_t i = 0; i < n; ++i) {
        const std::uint8_t* e = base + static_cast<std::size_t>(i) * CalNvmEntrySize;
        if (rd_u16le(e) != CalNvmMagic) continue;
        any_live = true;
        last_live = i;
        const std::uint32_t seq = rd_u32le(e + 8);
        if (seq > max_seq) max_seq = seq;
    }

    const std::uint32_t slot = any_live ? (last_live + 1u) : 0u;
    if (slot >= n) return false;                 // region full -> caller refuses

    // The bootloader programs blind here and falls back to compaction if the
    // slot rejects the write. We cannot compact, so check first: flash bits only
    // go 1 -> 0, and a slot with any stray bit is permanently un-programmable
    // until the sector is erased. Better a clean "Full" than a hardware error.
    const std::uint8_t* dst = base + static_cast<std::size_t>(slot) * CalNvmEntrySize;
    for (std::uint32_t b = 0; b < CalNvmEntrySize; ++b) {
        if (dst[b] != 0xFFu) return false;
    }

    *out_offset = slot * CalNvmEntrySize;
    *out_seq    = max_seq + 1u;
    return true;
}

void build_cal_entry(const PedalCal& c, std::uint32_t seq,
                     std::uint8_t out[CalNvmEntrySize]) noexcept {
    // Zero-filled, matching the bootloader's `bl_nvm_entry_t entry = { 0 }`
    // so an app-written entry is byte-identical in shape to a BL-written one.
    for (std::uint32_t i = 0; i < CalNvmEntrySize; ++i) out[i] = 0x00u;
    out[0] = static_cast<std::uint8_t>(CalNvmMagic & 0xFFu);
    out[1] = static_cast<std::uint8_t>(CalNvmMagic >> 8);
    out[2] = static_cast<std::uint8_t>(CalNvmKey & 0xFFu);
    out[3] = static_cast<std::uint8_t>(CalNvmKey >> 8);
    out[4] = static_cast<std::uint8_t>(CalRecordLen);
    out[5] = 0; out[6] = 0; out[7] = 0;
    out[8]  = static_cast<std::uint8_t>(seq & 0xFFu);
    out[9]  = static_cast<std::uint8_t>((seq >> 8) & 0xFFu);
    out[10] = static_cast<std::uint8_t>((seq >> 16) & 0xFFu);
    out[11] = static_cast<std::uint8_t>((seq >> 24) & 0xFFu);
    std::uint8_t rec[CalRecordLen];
    encode_cal_record(c, rec);
    for (std::size_t i = 0; i < CalRecordLen; ++i) out[12 + i] = rec[i];
}

}  // namespace ecu
