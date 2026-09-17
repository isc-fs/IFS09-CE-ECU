# CAN3 Map

Mapa de las tramas que la ECU publica hacia el dashboard por `CAN_BUS_DASH`
(`FDCAN3`).

Importante:

- `TelemetryTask` arma las 18 tramas (`0x510..0x521`) que espera
  `IFS08-CE-DASH` (`display_telemetry_can_config.c`), pero **no todos los
  campos tienen fuente real hoy en el firmware de la ECU**.
- Los campos marcados **PLACEHOLDER** se mandan en `0` porque el dato no
  existe todavia en la ECU (no se decodifica ese CAN RX del inversor/AMS, o
  no hay driver GPS). No confundir un `0` de placeholder con una lectura
  real de cero.

Origen en codigo:

- snapshot de vehiculo: [Core/Src/app/vehicle_service.cpp](../Core/Src/app/vehicle_service.cpp)
- pedales/plausibilidad (mirror de ControlTask): [Core/Inc/app/app_globals.h](../Core/Inc/app/app_globals.h)
- publicacion CAN3: [Core/Src/app/telemetry_task.cpp](../Core/Src/app/telemetry_task.cpp)

## Flujo

1. `TelemetryTask` hace snapshot de `VehicleService` (+ lee los `g_last_*`
   que `ControlTask` mirror-ea cada 10 ms: pedales, boton, flag T11.8.9)
2. construye las 18 tramas CAN3 de dashboard
3. las encola en `can_tx_queue` con `can_tx_post()`
4. `CanTxTask` las transmite por `FDCAN3`

Cadencia:

- todas las tramas (`0x510..0x521`) se publican cada `200 ms`, en el mismo
  ciclo de `TelemetryTask` -- no hay sub-tasa de 500 ms para el grupo AMS/GPS
  como en el contrato legacy (no hay `DashTask` ni cola de telemetria propia
  en el firmware actual)

## IDs

### `0x510` - estado general

Bus: `FDCAN3`
DLC: `8`

| Byte | Campo | Origen actual |
|---|---|---|
| 0 | `inv_state` | `VehicleState.inv_state` |
| 1 | `torque_pct` | `g_last_torque_pct` |
| 2 | `fault_bits` | bit0 **RESERVADO** (siempre 0), bit1 `g_last_t11_8_9` (mirror de `CtrlOutput.t11_8_9`) |
| 3 | `ok_precarga` | `VehicleState.ok_precharge` |
| 4 | `boton_arranque` | `g_last_start_button` (mirror de `IoInputs.start_button`) |
| 5 | reservado | `0` |
| 6-7 | `sequence` LE | secuencia de `TelemetryTask` |

`fault_bits`:

- bit 0: **RESERVADO — siempre 0.** Llevaba `EV.2.3` (freno+acelerador
  implausible). La regla se eliminó en FS-Rules 2024 y el corte con ella (#177).
  El bit **no se ha reutilizado ni se ha desplazado el layout**: el decoder del
  dashboard es de otro equipo y un bit renumerado se malinterpreta en silencio.
- bit 1: `T11.8.9` (disagreement APPS1/APPS2 > 100 ms) — **sin cambios**, sigue
  siendo obligatoria

### `0x511` - pedales y freno

Bus: `FDCAN3`
DLC: `6`

Real (ADC crudo, mirror de `IoInputs` leido por `ControlTask` cada 10 ms).

| Bytes | Campo | Origen actual |
|---|---|---|
| 0-1 | `s1_aceleracion` LE | `g_last_apps1_raw` |
| 2-3 | `s2_aceleracion` LE | `g_last_apps2_raw` |
| 4-5 | `s_freno` LE | `g_last_brake_raw` |

### `0x512` - tensiones y estado inversor

Bus: `FDCAN3`
DLC: `6`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0-1 | `inv_dc_bus_voltage` LE | `VehicleState.inv_dc_bus_V` |
| 2-3 | `v_celda_min` LE | `VehicleState.v_cell_min_mV` |
| 4 | `inv_error` | `VehicleState.inv_error` |
| 5 | `inv_vdc_ready` | `VehicleState.last_vconfig_tick != 0` |

### `0x513` - temperaturas inversor

Bus: `FDCAN3`
DLC: `6`

| Bytes | Campo | Origen actual |
|---|---|---|
| 0-1 | `inv_motor_temp` LE | `VehicleState.inv_temp_motor1` |
| 2-3 | `inv_igbt_temp` LE | `VehicleState.inv_temp_pwrstg` |
| 4-5 | `inv_air_temp` LE | `VehicleState.inv_temp_board` |

### `0x514` - rpm inversor

Bus: `FDCAN3`
DLC: `4`

| Bytes | Campo | Origen actual |
|---|---|---|
| 0-3 | `inv_rpm` LE | `VehicleState.inv_rpm` |

### `0x515` - velocidad inversor

Bus: `FDCAN3`
DLC: `4`

**PLACEHOLDER (`0`)**: el inversor no reporta esta senal en ninguno de los
frames que `VehicleService` decodifica hoy (`0x461/0x463/0x464/0x466`).

| Bytes | Campo | Origen actual |
|---|---|---|
| 0-3 | `inv_speed_actual` LE | placeholder `0` |

### `0x516` - corriente inversor

Bus: `FDCAN3`
DLC: `4`

**PLACEHOLDER (`0`)**: mismo motivo que `0x515`.

| Bytes | Campo | Origen actual |
|---|---|---|
| 0-3 | `inv_current_actual` LE | placeholder `0` |

### `0x517` - estados ECU y AMS

Bus: `FDCAN3`
DLC: `2`

| Byte | Campo | Origen actual |
|---|---|---|
| 0 | `ecu_fsm_state` | `g_last_ctrl_state` |
| 1 | `ams_state` | `VehicleState.ams_fsm_state` |

### `0x518` - AMS lento

Bus: `FDCAN3`
DLC: `7`

`soc` sigue en placeholder (el AMS no tiene estimador de SOC, ver mas abajo);
`corriente_accu`/`corriente_dcdc`/`temp_dcdc` son reales, decodificados de
`ACU_currents` (0x135) y `ACU_tmax_module_b` (0x137) en el bus ACU.

| Bytes | Campo | Origen actual |
|---|---|---|
| 0 | `soc` | placeholder `0` (AMS: sin estimador, ver `IFS08-CE-AMS acu_tx_encoders.hpp`) |
| 1-2 | `corriente_accu` LE | `VehicleState.current_accu_dA` (0x135, deciamps, tal cual lo manda el AMS) |
| 3-4 | `corriente_dcdc` LE | `VehicleState.current_dcdc_dA` (0x135, deciamps) |
| 5-6 | `temp_dcdc` LE | `VehicleState.tmax_dcdc` (0x137; el AMS lo manda como stub hasta que exista sensor DCDC real) |

### `0x519` - GPS A

Bus: `FDCAN3`
DLC: `8`

**PLACEHOLDER (`0`)**: el driver GPS **si existe** (`gps_task.cpp`, MTK3339 en
USART10) y ya publica `0x508`/`0x509` en el bus ACU mas los bytes 82..95 del
snapshot de radio. Lo que falta es solo cablear `GpsService` a estas tramas de
dashboard en `telemetry_task.cpp`.

| Bytes | Campo | Origen actual |
|---|---|---|
| 0-1 | `gps_speed` LE | placeholder `0` |
| 2-3 | `gps_course_deg` LE | placeholder `0` |
| 4-7 | `gps_altitude` LE | placeholder `0` |

### `0x51A` - GPS B

Bus: `FDCAN3`
DLC: `8`

**PLACEHOLDER (`0`)**: mismo motivo que `0x519`.

| Bytes | Campo | Origen actual |
|---|---|---|
| 0 | `gps_fix_type` | placeholder `0` |
| 1 | `gps_sat_count` | placeholder `0` |
| 2-3 | `gps_hdop` LE | placeholder `0` |
| 4-7 | `gps_latitude` LE | placeholder `0` |

### `0x51B` - GPS C + tiempo

Bus: `FDCAN3`
DLC: `8`

`gps_longitude` es placeholder; `tick_ms` es real.

| Bytes | Campo | Origen actual |
|---|---|---|
| 0-3 | `gps_longitude` LE | placeholder `0` |
| 4-7 | `tick_ms` LE | `osKernelGetTickCount()` |

### `0x51C` - AMS vmin modulos 0..2

Bus: `FDCAN3`
DLC: `6`

Real: decodificado de `ACU_vmin_module_a` (0x131, BE en el bus ACU) y
reenviado LE para el dash.

| Bytes | Campo | Origen actual |
|---|---|---|
| 0-1 | `vmin_modulo[0]` LE | `VehicleState.vmin_module[0]` (0x131) |
| 2-3 | `vmin_modulo[1]` LE | `VehicleState.vmin_module[1]` (0x131) |
| 4-5 | `vmin_modulo[2]` LE | `VehicleState.vmin_module[2]` (0x131) |

### `0x51D` - AMS vmin modulos 3..4

Bus: `FDCAN3`
DLC: `4`

Real: decodificado de `ACU_vmin_module_b` (0x132).

| Bytes | Campo | Origen actual |
|---|---|---|
| 0-1 | `vmin_modulo[3]` LE | `VehicleState.vmin_module[3]` (0x132) |
| 2-3 | `vmin_modulo[4]` LE | `VehicleState.vmin_module[4]` (0x132) |

### `0x51E` - AMS vmax modulos 0..2

Bus: `FDCAN3`
DLC: `6`

Real: decodificado de `ACU_vmax_module_a` (0x133).

| Bytes | Campo | Origen actual |
|---|---|---|
| 0-1 | `vmax_modulo[0]` LE | `VehicleState.vmax_module[0]` (0x133) |
| 2-3 | `vmax_modulo[1]` LE | `VehicleState.vmax_module[1]` (0x133) |
| 4-5 | `vmax_modulo[2]` LE | `VehicleState.vmax_module[2]` (0x133) |

### `0x51F` - AMS vmax modulos 3..4

Bus: `FDCAN3`
DLC: `4`

Real: decodificado de `ACU_vmax_module_b` (0x134).

| Bytes | Campo | Origen actual |
|---|---|---|
| 0-1 | `vmax_modulo[3]` LE | `VehicleState.vmax_module[3]` (0x134) |
| 2-3 | `vmax_modulo[4]` LE | `VehicleState.vmax_module[4]` (0x134) |

### `0x520` - AMS temperaturas maximas 0..2

Bus: `FDCAN3`
DLC: `6`

Real: decodificado de `ACU_tmax_module_a` (0x136).

| Bytes | Campo | Origen actual |
|---|---|---|
| 0-1 | `temp_max_modulo[0]` LE | `VehicleState.tmax_module[0]` (0x136) |
| 2-3 | `temp_max_modulo[1]` LE | `VehicleState.tmax_module[1]` (0x136) |
| 4-5 | `temp_max_modulo[2]` LE | `VehicleState.tmax_module[2]` (0x136) |

### `0x521` - AMS temperaturas maximas 3..4 + DC/DC

Bus: `FDCAN3`
DLC: `6`

Real: decodificado de `ACU_tmax_module_b` (0x137). `temp_dcdc` es un stub del
lado del AMS hasta que exista un sensor DCDC real -- la ECU reenvia lo que
sea que el AMS mande.

| Bytes | Campo | Origen actual |
|---|---|---|
| 0-1 | `temp_max_modulo[3]` LE | `VehicleState.tmax_module[3]` (0x137) |
| 2-3 | `temp_max_modulo[4]` LE | `VehicleState.tmax_module[4]` (0x137) |
| 4-5 | `temp_dcdc` LE | `VehicleState.tmax_dcdc` (0x137, stub AMS) |

## Pendiente para cerrar los placeholders restantes

- **Velocidad/corriente inversor (`0x515/0x516`)**: agregar el/los frame(s)
  RX del inversor que traigan esas senales (si el protocolo NX/EMC los
  expone) y decodificarlos en `vehicle_service.cpp`.
- **SOC (`0x518` byte 0)**: no depende de la ECU -- el AMS no tiene estimador
  de SOC todavia (`IFS08-CE-AMS acu_tx_encoders.hpp`: "0x130 SoC % -- DEFERRED,
  no estimator"). Va a seguir en `0` hasta que el AMS lo implemente.
- **GPS (`0x519..0x51B` salvo `tick_ms`)**: el driver y el parser ya estan; solo
  falta leer `GpsService::instance().snapshot()` en `telemetry_task.cpp` y
  rellenar estas tramas.
