// SPDX-License-Identifier: proprietary

#include "app/radio_snapshot.hpp"

#include <cstring>

namespace ecu {

namespace {

inline void put_le16(std::uint8_t* p, std::uint16_t v) noexcept {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
}
inline void put_le32(std::uint8_t* p, std::uint32_t v) noexcept {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
    p[2] = static_cast<std::uint8_t>(v >> 16);
    p[3] = static_cast<std::uint8_t>(v >> 24);
}
// Signed helpers write the raw two's-complement bits LE; the parser reads them
// back with '<h' / '<i' (signed), so no reinterpretation is needed here.
inline void put_i16(std::uint8_t* p, std::int16_t v) noexcept {
    put_le16(p, static_cast<std::uint16_t>(v));
}
inline void put_i32(std::uint8_t* p, std::int32_t v) noexcept {
    put_le32(p, static_cast<std::uint32_t>(v));
}

}  // namespace

void serialize_radio_snapshot(std::uint8_t out[kRadioSnapshotWireSize],
                              const RadioSnapshotInputs& in) noexcept {
    std::memset(out, 0, kRadioSnapshotWireSize);

    put_le32(&out[0],  in.tick_ms);
    put_le16(&out[4],  in.seq);
    out[6] = in.start_button;
    put_le16(&out[7],  in.apps1_raw);
    put_le16(&out[9],  in.apps2_raw);
    put_le16(&out[11], in.brake_raw);
    put_le16(&out[13], in.torque_pct);                   // u8 zero-extended
    out[15] = 0;   // reserved (was ev_2_3)
    out[16] = in.t11_8_9;
    out[17] = in.state;
    out[18] = in.ok_precharge;
    out[19] = in.ams_fsm_state;
    put_le16(&out[20], in.v_cell_min_mV);
    out[22] = in.soc;
    for (unsigned i = 0; i < 5; ++i) put_le16(&out[23 + 2 * i], in.vmin_module[i]);
    for (unsigned i = 0; i < 5; ++i) put_le16(&out[33 + 2 * i], in.vmax_module[i]);
    put_i16(&out[43], in.current_accu_dA);
    put_i16(&out[45], in.current_dcdc_dA);
    put_i16(&out[47], in.tmax_dcdc);
    for (unsigned i = 0; i < 5; ++i) put_i16(&out[49 + 2 * i], in.tmax_module[i]);
    out[59] = in.inv_state;
    out[60] = in.inv_vconfig_active;
    out[61] = in.inv_error;
    put_le16(&out[62], in.inv_dc_bus_V);
    put_le16(&out[64], in.inv_temp_motor1);
    put_le16(&out[66], in.inv_temp_pwrstg);
    put_le16(&out[68], in.inv_temp_board);
    put_i32(&out[70], in.inv_rpm);
    put_i32(&out[74], in.inv_speed_actual);
    put_i32(&out[78], in.inv_current_actual);
    // --- GPS (formerly reserved tail; wire size unchanged at 102 bytes) ---
    put_i32(&out[82],  in.gps_lat_deg1e7);
    put_i32(&out[86],  in.gps_lon_deg1e7);
    put_le16(&out[90], in.gps_speed_kmh_x100);
    put_le16(&out[92], in.gps_course_deg_x100);
    out[94] = in.gps_sats;
    out[95] = in.gps_has_fix;
    // [96..101] reserved (already zero).
}

void build_radio_fragment(std::uint8_t out[kRadioFragmentSize],
                          const std::uint8_t snapshot[kRadioSnapshotWireSize],
                          std::uint16_t seq, std::uint8_t frag_idx) noexcept {
    std::memset(out, 0, kRadioFragmentSize);
    out[0] = kRadioMagic;
    out[1] = kRadioVersionSnapshot;
    out[2] = frag_idx;
    out[3] = static_cast<std::uint8_t>(kRadioSnapshotFragments);
    put_le16(&out[4], seq);
    out[6] = kRadioKindSnapshot;
    out[7] = 0u;  // reserved

    const unsigned off = static_cast<unsigned>(frag_idx) * kRadioFragPayloadSize;
    if (off < kRadioSnapshotWireSize) {
        unsigned n = kRadioSnapshotWireSize - off;
        if (n > kRadioFragPayloadSize) n = kRadioFragPayloadSize;
        std::memcpy(&out[8], &snapshot[off], n);  // remainder stays zero-padded
    }
}

}  // namespace ecu
