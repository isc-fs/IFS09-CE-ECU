# Dash Variables

> **⚠️ This is a WISHLIST, not a contract.** It records variables the team would
> *like* on the dashboard. It does **not** describe what the ECU actually sends.
>
> The live dashboard contract — the 18 frames `0x510`–`0x521` on FDCAN3, with
> real byte offsets that another team's decoder depends on — is
> [`CAN3_MAP.md`](CAN3_MAP.md). If the two disagree, `CAN3_MAP.md` and
> `Core/Src/app/telemetry_task.cpp` are right and this file is aspiration.

Objetivo de variables que queremos poder mostrar en el dashboard.

Importante:

- este documento describe el objetivo funcional del dash
- no describe necesariamente lo que hoy sale por `CAN3`
- el contrato CAN3 actual sigue estando en [CAN3_MAP.md](./CAN3_MAP.md)

## Metadatos

| Variable | Tipo | Descripción |
| --- | --- | --- |
| `tick_ms` | `uint32_t` | Tick del sistema al construir el snapshot |
| `sequence` | `uint16_t` | Contador secuencial de telemetría |

## ECU

| Variable | Tipo | Origen actual | Descripción |
| --- | --- | --- | --- |
| `ecu.fsm_state` | `uint8_t` | `g_in.fsm_state` | Estado interno de la máquina de estados ECU |
| `ecu.boton_arranque` | `uint8_t` | `g_in.boton_arranque` | Estado del botón de arranque |
| `ecu.s1_aceleracion` | `uint16_t` | `g_in.s1_aceleracion` | Pedal acelerador 1 |
| `ecu.s2_aceleracion` | `uint16_t` | `g_in.s2_aceleracion` | Pedal acelerador 2 |
| `ecu.s_freno` | `uint16_t` | `g_in.s_freno` | Sensor de freno |
| `ecu.torque_total` | `uint16_t` | `g_in.torque_total` | Par calculado por control |
| ~~`ecu.flag_ev_2_3`~~ | — | — | **ELIMINADA**: EV.2.3 se borro de FS-Rules 2024 y el corte con ella. El bit 0 de `fault_bits` queda RESERVADO en 0; la posicion NO se renumero. |
| `ecu.flag_t11_8_9` | `uint8_t` | `g_in.flag_T11_8_9` | Flag de plausibilidad T11.8.9 |

## AMS

| Variable | Tipo | Origen actual | Descripción |
| --- | --- | --- | --- |
| `ams.ok_precarga` | `uint8_t` | `g_in.ok_precarga` | ACK de precarga |
| `ams.ams_state` | `uint8_t` | `g_in.ams_state` | Estado reportado por AMS |
| `ams.v_celda_min` | `uint16_t` | `g_in.v_celda_min` | Voltaje mínimo de celda |
| `ams.soc` | `uint8_t` | `g_in.soc` | Estado de carga |
| `ams.vmin_modulo[0..4]` | `uint16_t[5]` | `g_in.vmin_modulo[0..4]` | Voltajes minimos por modulo |
| `ams.vmax_modulo[0..4]` | `uint16_t[5]` | `g_in.vmax_modulo[0..4]` | Voltajes maximos por modulo |
| `ams.corriente_accu` | `int16_t` | `g_in.corriente_accu` | Corriente del acumulador |
| `ams.corriente_dcdc` | `int16_t` | `g_in.corriente_dcdc` | Corriente del DC/DC |
| `ams.temp_dcdc` | `int16_t` | `g_in.temp_dcdc` | Temperatura del DC/DC |
| `ams.temp_max_modulo[0..4]` | `int16_t[5]` | `g_in.temp_max_modulo[0..4]` | Temperaturas maximas por modulo |

## Inverter

| Variable | Tipo | Origen actual | Descripción |
| --- | --- | --- | --- |
| `inverter.inv_state` | `uint8_t` | `g_in.inv_state` | Estado del inversor |
| `inverter.inv_vdc_ready` | `uint8_t` | `g_in.inv_vdc_ready` | Bandera `vdc_ready` |
| `inverter.inv_error` | `uint8_t` | `g_in.inv_error` | Bandera de error del inversor |
| `inverter.inv_dc_bus_voltage` | `uint16_t` | `g_in.inv_dc_bus_voltage` | Tensión de bus DC |
| `inverter.inv_motor_temp` | `int16_t` | `g_in.inv_motor_temp` | Temperatura de motor |
| `inverter.inv_igbt_temp` | `int16_t` | `g_in.inv_igbt_temp` | Temperatura IGBT |
| `inverter.inv_air_temp` | `int16_t` | `g_in.inv_air_temp` | Temperatura de aire |
| `inverter.inv_rpm` | `int32_t` | `g_in.inv_rpm` | RPM del inversor/motor |
| `inverter.inv_speed_actual` | `int32_t` | `g_in.inv_speed_actual` | Velocidad actual |
| `inverter.inv_current_actual` | `int32_t` | `g_in.inv_current_actual` | Corriente actual |

## GPS

| Variable | Tipo | Origen actual | Descripción |
| --- | --- | --- | --- |
| `gps.speed` | `uint16_t` | `g_in.gps_speed` | Velocidad GPS |
| `gps.course_deg` | `uint16_t` | `g_in.gps_course_deg` | Rumbo |
| `gps.altitude` | `int32_t` | `g_in.gps_altitude` | Altitud |
| `gps.fix_type` | `uint8_t` | `g_in.gps_fix_type` | Tipo de fix |
| `gps.sat_count` | `uint8_t` | `g_in.gps_sat_count` | Número de satélites |
| `gps.hdop` | `uint16_t` | `g_in.gps_hdop` | HDOP |
| `gps.latitude` | `int32_t` | `g_in.gps_latitude` | Latitud |
| `gps.longitude` | `int32_t` | `g_in.gps_longitude` | Longitud |

## Estados clave

Estos tres estados son especialmente útiles para el dashboard:

- `ecu.fsm_state`: qué está intentando hacer la ECU
- `ams.ams_state`: en qué estado está el AMS
- `inverter.inv_state`: en qué estado está el inversor
