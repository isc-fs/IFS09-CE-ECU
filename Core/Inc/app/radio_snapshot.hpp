// SPDX-License-Identifier: proprietary
//
// radio_snapshot -- the v2 "fragmented snapshot" nRF24 telemetry wire format.
//
// The ground station (IFS08-TE, branch feat/receptor_08,
// ISC_REAL_TIME_25/ISC_RTT_serial.py) reassembles five 32-byte nRF24 payloads
// into one 102-byte snapshot and decodes it in _decode_snapshot(). THAT parser
// is the byte-authority; this serializer is written to match its documented
// offsets exactly, and a host round-trip test feeds our bytes back through the
// real parser to prove it.
//
// Pure + HAL/RTOS-free (like control.cpp / inverter.cpp) so the byte contract
// is host-testable in isolation. The telemetry task fills RadioSnapshotInputs
// from VehicleState + the ControlTask g_last_* mirrors and does the framing.
//
// On-air fragment (per 32-byte payload):
//   [0] magic 0xEC · [1] version 0x03 · [2] frag_idx 0..4 · [3] frag_tot 5
//   [4..5] seq u16 LE · [6] kind 0x06 (snapshot) · [7] reserved
//   [8..31] 24-byte slice: snapshot[frag_idx*24 .. +24)   (last frag zero-padded)

#pragma once

#include <cstdint>

namespace ecu {

inline constexpr unsigned     kRadioSnapshotWireSize  = 102u;  // kRadioSnapshotWireSize
inline constexpr unsigned     kRadioFragPayloadSize   = 24u;   // bytes 8..31 of a fragment
inline constexpr unsigned     kRadioSnapshotFragments = 5u;    // ceil(102/24)
inline constexpr unsigned     kRadioFragmentSize      = 32u;   // full nRF24 payload
inline constexpr std::uint8_t kRadioMagic             = 0xECu;
inline constexpr std::uint8_t kRadioVersionSnapshot   = 0x03u;
inline constexpr std::uint8_t kRadioKindSnapshot      = 0x06u;

// Everything that goes into the 102-byte snapshot, as a plain POD. Placeholders
// (soc, inv_speed_actual, inv_current_actual) have no source in this firmware
// yet -- pass 0; the ground station parses the slot regardless.
struct RadioSnapshotInputs {
    std::uint32_t tick_ms            = 0;   // [0..3]
    std::uint16_t seq                = 0;   // [4..5]
    // --- control (ControlTask g_last_* mirrors) ---
    std::uint8_t  start_button       = 0;   // [6]
    std::uint16_t apps1_raw          = 0;   // [7..8]
    std::uint16_t apps2_raw          = 0;   // [9..10]
    std::uint16_t brake_raw          = 0;   // [11..12]
    std::uint8_t  torque_pct         = 0;   // [13..14] (u8 zero-extended)
    // [15] RESERVED, always 0. Was ev_2_3 until EV.2.3 was deleted in
    // FS-Rules 2024. The byte stays so [16] onward keep their offsets -- the
    // ground station decodes this by fixed offset.
    std::uint8_t  reserved_15        = 0;   // [15]
    std::uint8_t  t11_8_9            = 0;   // [16]
    std::uint8_t  state              = 0;   // [17] ECU FSM state
    std::uint8_t  ok_precharge       = 0;   // [18]
    // --- AMS ---
    std::uint8_t  ams_fsm_state      = 0;   // [19]
    std::uint16_t v_cell_min_mV      = 0;   // [20..21]
    std::uint8_t  soc                = 0;   // [22] PLACEHOLDER (no estimator)
    std::uint16_t vmin_module[5]     = {};  // [23..32]
    std::uint16_t vmax_module[5]     = {};  // [33..42]
    std::int16_t  current_accu_dA    = 0;   // [43..44]
    std::int16_t  current_dcdc_dA    = 0;   // [45..46]
    std::int16_t  tmax_dcdc          = 0;   // [47..48]
    std::int16_t  tmax_module[5]     = {};  // [49..58]
    // --- inverter ---
    std::uint8_t  inv_state          = 0;   // [59]
    std::uint8_t  inv_vconfig_active = 0;   // [60] (last_vconfig_tick != 0)
    std::uint8_t  inv_error          = 0;   // [61]
    std::uint16_t inv_dc_bus_V       = 0;   // [62..63]
    std::uint16_t inv_temp_motor1    = 0;   // [64..65]
    std::uint16_t inv_temp_pwrstg    = 0;   // [66..67]
    std::uint16_t inv_temp_board     = 0;   // [68..69]
    std::int32_t  inv_rpm            = 0;   // [70..73]
    std::int32_t  inv_speed_actual   = 0;   // [74..77] PLACEHOLDER (no source)
    std::int32_t  inv_current_actual = 0;   // [78..81] PLACEHOLDER (no source)
    // --- GPS (MTK3339 / USART10, see gps_nmea.hpp). Occupies the reserved tail,
    //     so the wire size stays 102 bytes and a ground station that predates
    //     these fields keeps working -- it just ignores them. The TE-side
    //     _decode_snapshot() must be extended to SHOW them. ---
    std::int32_t  gps_lat_deg1e7     = 0;   // [82..85]  degrees * 1e7, +N/-S
    std::int32_t  gps_lon_deg1e7     = 0;   // [86..89]  degrees * 1e7, +E/-W
    std::uint16_t gps_speed_kmh_x100 = 0;   // [90..91]  km/h * 100
    std::uint16_t gps_course_deg_x100= 0;   // [92..93]  deg * 100
    std::uint8_t  gps_sats           = 0;   // [94]
    std::uint8_t  gps_has_fix        = 0;   // [95]  gate lat/lon/speed on this
    // [96..101] reserved / zero
};

// Fill `out` (102 bytes) with the little-endian snapshot per _decode_snapshot().
void serialize_radio_snapshot(std::uint8_t out[kRadioSnapshotWireSize],
                              const RadioSnapshotInputs& in) noexcept;

// Fill `out` (32 bytes) with fragment `frag_idx` (0..4): the v2 header + the
// 24-byte slice snapshot[frag_idx*24 .. +24), zero-padded on the last fragment.
void build_radio_fragment(std::uint8_t out[kRadioFragmentSize],
                          const std::uint8_t snapshot[kRadioSnapshotWireSize],
                          std::uint16_t seq, std::uint8_t frag_idx) noexcept;

}  // namespace ecu
