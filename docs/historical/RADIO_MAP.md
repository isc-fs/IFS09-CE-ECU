# Radio Map

> ⚠️ **SUPERSEDED (2026-07-05).** This documents the old RF_FAST/RF_SLOW
> multi-kind protocol (version `0x02`), which the ground station **no longer
> parses**. The current radio contract is the v2 fragmented snapshot —
> see **[RADIO_SNAPSHOT_MAP.md](../RADIO_SNAPSHOT_MAP.md)**. Kept for history.

Contrato actual de radio entre la ECU y `IFS08-TE-main`.

La ECU no envia floats ni IDs `0x600/0x610/...` por `nRF24`.
Envia paquetes binarios de `32` bytes con cabecera fija y payload
troceado por cadencia.

Importante: el layout de bytes de este documento esta verificado contra el
parser real y vigente en `IFS08-TE-main/ISC_REAL_TIME_25/ISC_RTT_serial.py`
(`parse_radio_v2_frame()`), no solo contra el firmware. Si cualquiera de los
dos lados cambia el layout, hay que actualizar el otro.

Como en `docs/CAN3_MAP.md`: los campos marcados **PLACEHOLDER** se mandan en
`0` porque el dato no existe todavia en la ECU.

Origen en codigo:

- serializacion radio: `Core/Src/app/telemetry_task.cpp`
- transporte nRF24: `Core/Src/nrf24.c`
- parser receptor (ground truth): `IFS08-TE-main/ISC_REAL_TIME_25/ISC_RTT_serial.py` (`parse_radio_v2_frame`)

## Cabecera comun

Todos los paquetes radio ocupan `32` bytes.

| Byte | Campo | Valor |
|---|---|---|
| 0 | `magic` | `0xEC` |
| 1 | `version` | `0x02` |
| 2 | `fragment_index` | fragmento actual |
| 3 | `fragment_count` | numero total de fragmentos del frame |
| 4-5 | `sequence` LE | secuencia del frame |
| 6 | `kind` | `3=RF_FAST`, `4=RF_SLOW` |
| 7 | reservado | `0` |

Bytes `8..31`: payload del fragmento.

## Cadencia

- `RF_FAST` y `RF_SLOW` se publican juntos, cada `200 ms` (mismo ciclo de
  `TelemetryTask`) -- no hay sub-tasa de 500 ms para `RF_SLOW` como en el
  contrato legacy original (misma simplificacion ya aplicada a CAN3).
- `RF_EVENT`: no se emite todavia (requiere logica de deteccion de
  transicion de estado/error, no solo repaquetado de un snapshot).

## RF_FAST

Fragmentos: `2`

### Fragmento 0

| Byte | Campo | Origen actual |
|---|---|---|
| 8 | `ecu_fsm_state` | `g_last_ctrl_state` |
| 9 | `inv_state` | `VehicleState.inv_state` |
| 10 | `ams_state` | `VehicleState.ams_fsm_state` |
| 11 | `boton_arranque` | `g_last_start_button` |
| 12 | `ok_precarga` | `VehicleState.ok_precharge` |
| 13 | `flag_ev_2_3` | `g_last_ev_2_3` |
| 14 | `flag_t11_8_9` | `g_last_t11_8_9` |
| 15 | `inv_vdc_ready` | `VehicleState.last_vconfig_tick != 0` |
| 16 | `inv_error` | `VehicleState.inv_error` |
| 17-18 | `torque_total` LE | `g_last_torque_pct` |
| 19-20 | `inv_dc_bus_voltage` LE | `VehicleState.inv_dc_bus_V` |
| 21-22 | `v_celda_min` LE | `VehicleState.v_cell_min_mV` |
| 23-24 | `s1_aceleracion` LE | `g_last_apps1_raw` |
| 25-26 | `s2_aceleracion` LE | `g_last_apps2_raw` |
| 27-28 | `s_freno` LE | `g_last_brake_raw` |
| 29-31 | reservado | `0` |

### Fragmento 1

| Bytes | Campo | Origen actual |
|---|---|---|
| 8-9 | `inv_motor_temp` LE | `VehicleState.inv_temp_motor1` |
| 10-11 | `inv_igbt_temp` LE | `VehicleState.inv_temp_pwrstg` |
| 12-13 | `inv_air_temp` LE | `VehicleState.inv_temp_board` |
| 14-17 | `inv_rpm` LE | `VehicleState.inv_rpm` |
| 18-21 | `inv_speed_actual` LE | **PLACEHOLDER `0`** -- ver nota `0x465` abajo |
| 22-25 | `inv_current_actual` LE | **PLACEHOLDER `0`** -- ver nota `0x465` abajo |
| 26-31 | reservado | `0` |

## RF_SLOW

Fragmentos: `5`

### Fragmento 0

| Bytes | Campo | Origen actual |
|---|---|---|
| 8 | `soc` | **PLACEHOLDER `0`** (AMS sin estimador, ver `docs/CAN3_MAP.md`) |
| 9-10 | `corriente_accu` LE | `VehicleState.current_accu_dA` (0x135) |
| 11-12 | `corriente_dcdc` LE | `VehicleState.current_dcdc_dA` (0x135) |
| 13-14 | `temp_dcdc` LE | `VehicleState.tmax_dcdc` (0x137, stub del lado AMS) |
| 15-16 | `gps_speed` LE | **PLACEHOLDER `0`** (sin driver GPS) |
| 17-18 | `gps_course_deg` LE | **PLACEHOLDER `0`** |
| 19-22 | `gps_altitude` LE | **PLACEHOLDER `0`** |
| 23-26 | `tick_ms` LE | `osKernelGetTickCount()` (real) |
| 27 | `gps_fix_type` | **PLACEHOLDER `0`** |
| 28 | `gps_sat_count` | **PLACEHOLDER `0`** |
| 29-30 | `gps_hdop` LE | **PLACEHOLDER `0`** |

### Fragmento 1

| Bytes | Campo | Origen actual |
|---|---|---|
| 8-11 | `gps_latitude` LE | **PLACEHOLDER `0`** |
| 12-15 | `gps_longitude` LE | **PLACEHOLDER `0`** |

### Fragmento 2

| Bytes | Campo | Origen actual |
|---|---|---|
| 8-9 | `vmin_modulo[0]` LE | `VehicleState.vmin_module[0]` (0x131) |
| 10-11 | `vmin_modulo[1]` LE | `VehicleState.vmin_module[1]` (0x131) |
| 12-13 | `vmin_modulo[2]` LE | `VehicleState.vmin_module[2]` (0x131) |
| 14-15 | `vmin_modulo[3]` LE | `VehicleState.vmin_module[3]` (0x132) |
| 16-17 | `vmin_modulo[4]` LE | `VehicleState.vmin_module[4]` (0x132) |

### Fragmento 3

| Bytes | Campo | Origen actual |
|---|---|---|
| 8-9 | `vmax_modulo[0]` LE | `VehicleState.vmax_module[0]` (0x133) |
| 10-11 | `vmax_modulo[1]` LE | `VehicleState.vmax_module[1]` (0x133) |
| 12-13 | `vmax_modulo[2]` LE | `VehicleState.vmax_module[2]` (0x133) |
| 14-15 | `vmax_modulo[3]` LE | `VehicleState.vmax_module[3]` (0x134) |
| 16-17 | `vmax_modulo[4]` LE | `VehicleState.vmax_module[4]` (0x134) |

### Fragmento 4

| Bytes | Campo | Origen actual |
|---|---|---|
| 8-9 | `temp_max_modulo[0]` LE | `VehicleState.tmax_module[0]` (0x136) |
| 10-11 | `temp_max_modulo[1]` LE | `VehicleState.tmax_module[1]` (0x136) |
| 12-13 | `temp_max_modulo[2]` LE | `VehicleState.tmax_module[2]` (0x136) |
| 14-15 | `temp_max_modulo[3]` LE | `VehicleState.tmax_module[3]` (0x137) |
| 16-17 | `temp_max_modulo[4]` LE | `VehicleState.tmax_module[4]` (0x137) |

## RF_EVENT

No emitido todavia por `TelemetryTask` (requiere detectar transiciones de
estado/error, no solo repaquetar un snapshot periodico). El parser en
`IFS08-TE-main` ya lo soporta:

Fragmentos: `1`

| Byte | Campo |
|---|---|
| 8 | `inv_state` |
| 9 | `flag_ev_2_3` |
| 10 | `flag_t11_8_9` |
| 11 | `inv_error` |
| 12 | `previous_inv_state` |
| 13 | `current_inv_state` |
| 14 | `event_inv_error` |
| 15 | `ok_precarga` |
| 16 | `inv_vdc_ready` |
| 17-18 | `torque_total` LE |
| 19-20 | `inv_dc_bus_voltage` LE |
| 21-22 | `v_celda_min` LE |
| 23-26 | `event_tick_ms` LE |
| 27 | `ecu_fsm_state` |
| 28 | `ams_state` |

## Pendiente

- **`inv_speed_actual`/`inv_current_actual`** (RF_FAST frag1): el ID real del
  inversor (`0x465 TX_STATE_6`) nunca tuvo un mapeo de bytes confirmado --
  incluso el firmware legacy (`main_polling.c`) lo dejaba con comentarios
  `// REPLACE with real byte mapping`. No wired hasta confirmar el layout
  contra el DBC real del inversor NX/EMC.
- **GPS y SOC**: mismo motivo que en `docs/CAN3_MAP.md` -- no hay driver GPS
  ni estimador de SOC en el AMS.
- **RF_EVENT**: pendiente de implementar la deteccion de transicion de
  estado/error en `TelemetryTask` (guardar `inv_state`/`inv_error` previos y
  disparar un frame extra al cambiar).
