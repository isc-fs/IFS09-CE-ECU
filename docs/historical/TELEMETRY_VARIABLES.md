# Telemetry Variables

Inventario actual de variables del `telemetry_snapshot_t`.

La referencia buena para telemetria ya no es `app_inputs_t` completa, sino
`telemetry_snapshot_t`, que es lo que construye `Telemetry_BuildSnapshot(...)`
y luego consumen `dash`, `radio` y `sd`.

## Metadatos

| Variable | Tipo | Descripcion |
| --- | --- | --- |
| `tick_ms` | `uint32_t` | Tick del sistema al construir el snapshot |
| `sequence` | `uint16_t` | Contador secuencial de telemetria |

## ECU

| Variable | Tipo | Origen actual | Descripcion |
| --- | --- | --- | --- |
| `ecu.boton_arranque` | `uint8_t` | `g_in.boton_arranque` | Estado del boton de arranque |
| `ecu.s1_aceleracion` | `uint16_t` | `g_in.s1_aceleracion` | Pedal acelerador 1 |
| `ecu.s2_aceleracion` | `uint16_t` | `g_in.s2_aceleracion` | Pedal acelerador 2 |
| `ecu.s_freno` | `uint16_t` | `g_in.s_freno` | Sensor de freno |
| `ecu.torque_total` | `uint16_t` | `g_in.torque_total` | Par calculado por control |
| `ecu.flag_ev_2_3` | `uint8_t` | `g_in.flag_EV_2_3` | Flag de seguridad EV 2.3 |
| `ecu.flag_t11_8_9` | `uint8_t` | `g_in.flag_T11_8_9` | Flag de plausibilidad T11.8.9 |
| `ecu.fsm_state` | `uint8_t` | `g_in.fsm_state` | Estado interno de la maquina de estados ECU |

## AMS

| Variable | Tipo | Origen actual | Descripcion |
| --- | --- | --- | --- |
| `ams.ok_precarga` | `uint8_t` | `g_in.ok_precarga` | ACK de precarga |
| `ams.ams_state` | `uint8_t` | `g_in.ams_state` | Estado reportado por AMS |
| `ams.v_celda_min` | `uint16_t` | `g_in.v_celda_min` | Voltaje minimo de celda |
| `ams.soc` | `uint8_t` | `g_in.soc` | Estado de carga |
| `ams.vmin_modulo[0..4]` | `uint16_t[5]` | `g_in.vmin_modulo[0..4]` | Voltajes minimos por modulo |
| `ams.vmax_modulo[0..4]` | `uint16_t[5]` | `g_in.vmax_modulo[0..4]` | Voltajes maximos por modulo |
| `ams.corriente_accu` | `int16_t` | `g_in.corriente_accu` | Corriente del acumulador |
| `ams.corriente_dcdc` | `int16_t` | `g_in.corriente_dcdc` | Corriente del DC/DC |
| `ams.temp_dcdc` | `int16_t` | `g_in.temp_dcdc` | Temperatura del DC/DC |
| `ams.temp_max_modulo[0..4]` | `int16_t[5]` | `g_in.temp_max_modulo[0..4]` | Temperaturas maximas por modulo |

## Inverter

| Variable | Tipo | Origen actual | Descripcion |
| --- | --- | --- | --- |
| `inverter.inv_state` | `uint8_t` | `g_in.inv_state` | Estado del inversor |
| `inverter.inv_vdc_ready` | `uint8_t` | `g_in.inv_vdc_ready` | Bandera `vdc_ready` |
| `inverter.inv_error` | `uint8_t` | `g_in.inv_error` | Bandera de error del inversor |
| `inverter.inv_dc_bus_voltage` | `uint16_t` | `g_in.inv_dc_bus_voltage` | Tension de bus DC |
| `inverter.inv_motor_temp` | `int16_t` | `g_in.inv_motor_temp` | Temperatura de motor |
| `inverter.inv_igbt_temp` | `int16_t` | `g_in.inv_igbt_temp` | Temperatura IGBT |
| `inverter.inv_air_temp` | `int16_t` | `g_in.inv_air_temp` | Temperatura de aire |
| `inverter.inv_rpm` | `int32_t` | `g_in.inv_rpm` | RPM del inversor/motor |
| `inverter.inv_speed_actual` | `int32_t` | `g_in.inv_speed_actual` | Velocidad actual |
| `inverter.inv_current_actual` | `int32_t` | `g_in.inv_current_actual` | Corriente actual |

## GPS

| Variable | Tipo | Origen actual | Descripcion |
| --- | --- | --- | --- |
| `gps.speed` | `uint16_t` | `g_in.gps_speed` | Velocidad GPS |
| `gps.course_deg` | `uint16_t` | `g_in.gps_course_deg` | Rumbo |
| `gps.altitude` | `int32_t` | `g_in.gps_altitude` | Altitud |
| `gps.fix_type` | `uint8_t` | `g_in.gps_fix_type` | Tipo de fix |
| `gps.sat_count` | `uint8_t` | `g_in.gps_sat_count` | Numero de satelites |
| `gps.hdop` | `uint16_t` | `g_in.gps_hdop` | HDOP |
| `gps.latitude` | `int32_t` | `g_in.gps_latitude` | Latitud |
| `gps.longitude` | `int32_t` | `g_in.gps_longitude` | Longitud |

## Fuera del snapshot

Estas variables existen en `app_inputs_t` pero ahora mismo no entran en
`telemetry_snapshot_t`:

| Variable | Motivo |
| --- | --- |
| `last_precharge_ack_tick` | Estado interno/temporal |
| `last_ams_status_tick` | Estado interno/temporal |
| `ams_session_id` | Estado interno/contrato AMS, fuera del snapshot objetivo |
| `ams_session_valid` | Estado interno/contrato AMS, fuera del snapshot objetivo |
| `imu_accel_xyz[3]` | Fuera del snapshot objetivo actual |
| `imu_gyro_xyz[3]` | Fuera del snapshot objetivo actual |
| `imu_roll_deg` | Fuera del snapshot objetivo actual |
| `imu_pitch_deg` | Fuera del snapshot objetivo actual |
| `imu_yaw_deg` | Fuera del snapshot objetivo actual |

## Nota

- `telemetry_snapshot_t` es la base interna que ahora queremos transmitir.
- `CAN3_MAP.md` sigue describiendo solo el subconjunto actual de CAN3.
