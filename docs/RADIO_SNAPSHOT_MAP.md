# Radio telemetry — v2 fragmented snapshot (nRF24)

**Status: current.** Supersedes the RF_FAST/RF_SLOW multi-kind protocol in
[RADIO_MAP.md](historical/RADIO_MAP.md) (the ground station no longer parses it — see
[RADIO_TELEMETRY_FAILURE_ANALYSIS.md](historical/RADIO_TELEMETRY_FAILURE_ANALYSIS.md) for
the mismatch that this replaces).

## Source of truth

The **ground station** owns this wire contract, not the firmware:
`IFS08-TE` (branch **`feat/receptor_08`**) →
`ISC_REAL_TIME_25/ISC_RTT_serial.py` → `_decode_snapshot()`. The firmware
serializer (`Core/Src/app/radio_snapshot.cpp`) is written to match it exactly
and is verified two ways:

- **SIL** (`tests/sil/sil_control_tests.cpp::test_radio_snapshot`) — every
  offset, signedness, and the 5-fragment reassembly.
- **Round-trip** (`tests/sil/radio_snapshot_roundtrip.py`) — the firmware's
  bytes are decoded by a verbatim copy of the real `_decode_snapshot` and every
  field asserted.

## RF link config (nRF24, must match the receiver)

`PIPE "ECU01" (45 43 55 30 31) · channel 76 · 1 Mbps · AUTO_ACK off · CRC-8 (1 byte, CRCO sin poner) · PA_MAX`.

> The pipe address is the 5 ASCII bytes `ECU01`, **not** the nRF24 library default
> `0xE7E7E7E7E7`. Source of truth: `g_nrf24Address` in
> [`Core/Src/nrf24.c`](../Core/Src/nrf24.c) (line 47), written to both `RX_ADDR_P0`
> and `TX_ADDR` in `NRF24_ApplyDefaultConfig()`. A receiver left on the library
> default hears **nothing**, with no CAN symptom and no error on the ECU side.

Path: STM32 → nRF24 → Arduino Nano → USB-serial (`AA 55 20 <32B> <XOR>`) → PC.

## On-air fragment (each 32-byte nRF24 payload)

| Byte | Field | Value |
|---|---|---|
| 0 | magic | `0xEC` |
| 1 | version | `0x03` |
| 2 | frag_idx | 0..4 |
| 3 | frag_tot | 5 |
| 4..5 | seq | u16 LE |
| 6 | kind | `0x06` (`kRadioKindSnapshot`) |
| 7 | reserved | 0 |
| 8..31 | data | 24-byte slice: `snapshot[frag_idx*24 .. +24)` (last frag zero-padded) |

Five fragments reassemble the **102-byte** snapshot (`5 × 24 = 120 ≥ 102`).

## 102-byte snapshot layout (little-endian)

| Offset | Field | Type | Firmware source |
|---|---|---|---|
| 0..3 | tick_ms | u32 | `osKernelGetTickCount()` |
| 4..5 | seq | u16 | task counter |
| 6 | start_button | u8 | `g_last_start_button` |
| 7..8 | apps1_raw | u16 | `g_last_apps1_raw` |
| 9..10 | apps2_raw | u16 | `g_last_apps2_raw` |
| 11..12 | brake_raw | u16 | `g_last_brake_raw` |
| 13..14 | torque_pct | u16 | `g_last_torque_pct` (u8 zero-ext) |
| 15 | **reservado** | u8 | siempre `0` — era `ev_2_3`; EV.2.3 se eliminó en FS-Rules 2024 (#177). El byte se mantiene para que 16+ conserven su offset |
| 16 | t11_8_9 | u8 | `g_last_t11_8_9` |
| 17 | state | u8 | `g_last_ctrl_state` (ECU FSM) |
| 18 | ok_precharge | u8 | `veh.ok_precharge` |
| 19 | ams_fsm_state | u8 | `veh.ams_fsm_state` |
| 20..21 | v_cell_min_mV | u16 | `veh.v_cell_min_mV` |
| 22 | **soc** | u8 | **PLACEHOLDER 0** — no AMS estimator |
| 23..32 | vmin_modulo[0..4] | 5×u16 | `veh.vmin_module` (0x131/0x132) |
| 33..42 | vmax_modulo[0..4] | 5×u16 | `veh.vmax_module` (0x133/0x134) |
| 43..44 | corriente_accu | i16 | `veh.current_accu_dA` (0x135) |
| 45..46 | corriente_dcdc | i16 | `veh.current_dcdc_dA` (0x135) |
| 47..48 | temp_dcdc | i16 | `veh.tmax_dcdc` (0x137) |
| 49..58 | temp_max_modulo[0..4] | 5×i16 | `veh.tmax_module` (0x136/0x137) |
| 59 | inv_state | u8 | `veh.inv_state` (0x461) |
| 60 | inv_vconfig_active | u8 | `veh.last_vconfig_tick != 0` |
| 61 | inv_error | u8 | `veh.inv_error` (0x461) |
| 62..63 | inv_dc_bus_V | u16 | `veh.inv_dc_bus_V` (0x466) |
| 64..65 | inv_temp_motor1 | u16 | `veh.inv_temp_motor1` (0x464) |
| 66..67 | inv_temp_pwrstg | u16 | `veh.inv_temp_pwrstg` (0x464) |
| 68..69 | inv_temp_board | u16 | `veh.inv_temp_board` (0x464) |
| 70..73 | inv_rpm | i32 | `veh.inv_rpm` (0x463, erpm — electrical) |
| 74..77 | **inv_speed_actual** | i32 | **PLACEHOLDER 0** — no source decoded |
| 78..81 | **inv_current_actual** | i32 | **PLACEHOLDER 0** — no source decoded |
| 82..85 | gps_lat_deg1e7 | i32 | `GpsService` — degrees * 1e7, +N / -S |
| 86..89 | gps_lon_deg1e7 | i32 | `GpsService` — degrees * 1e7, +E / -W |
| 90..91 | gps_speed_kmh_x100 | u16 | km/h * 100 (converted from NMEA knots) |
| 92..93 | gps_course_deg_x100 | u16 | course over ground, deg * 100 |
| 94 | gps_sats | u8 | satellites in view (GGA) |
| 95 | gps_has_fix | u8 | 1 = valid fix — **gate the four fields above on this** |
| 96..101 | reserved | 6 B | 0 |

> **GPS is live since #147.** The MTK3339 on USART10 fills what used to be the
> reserved tail. The wire size is UNCHANGED at 102 bytes, so an un-updated
> ground station keeps working — it simply ignores 82..95. Source is
> `GpsService` (a local UART peripheral), NOT `VehicleService`.
>
> **Always gate on `gps_has_fix`.** Position and speed carry the LAST VALID
> values when the fix drops; they do not zero out, so an ungated display shows
> a stale position as if it were live.
>
> Ground-station decode is tracked in isc-fs/IFS08-TE#1 — until that lands the
> bytes are transmitted but not shown.

## Cadence

One snapshot (5 fragments) per 200 ms `TelemetryTask` cycle, alongside the
FDCAN3 dash frames (`send_dashboard`, a separate wire contract — see
[CAN3_MAP.md](CAN3_MAP.md)). Revisit if nRF24 airtime becomes a constraint.
