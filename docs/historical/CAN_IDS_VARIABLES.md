# ECU dev clone - mapa CAN actual

> ⚠️ **HISTÓRICO (17-jun-2026) — NO es el mapa CAN vigente.** Cita como "referencia de
> verdad" `Core/Src/control.c`, `Core/Src/can.c` y `Core/Inc/app_state.h`, **borrados en el
> rewrite a C++17**. No incluye el contrato uDV (`0x504`–`0x511`), las tramas por módulo de
> la AMS (`0x131`–`0x137`), `0x706`/`0x707`, ni el bus de dashboard FDCAN3.
>
> **Fuente de verdad actual:** el DSL code-first `Core/Inc/can/messages/*.def` →
> [`dbc/ecu.dbc`](../dbc/ecu.dbc) (lo regenera el bot *dbcinator* en cada PR, así que nunca
> diverge). Resumen legible en [`../CLAUDE.md`](../../CLAUDE.md); dashboard en
> [`CAN3_MAP.md`](../CAN3_MAP.md). Se conserva sólo por historia.

Fecha: 17-jun-2026

## 1. Alcance

Este documento describe lo que hace hoy el `dev clone` de la ECU.
La referencia de verdad es el codigo en:

- `Core/Src/control.c`
- `Core/Src/can.c`
- `Core/Inc/can/messages/vcu_heartbeat.def`
- `Core/Inc/app_state.h`

La documentacion anterior mezclaba comportamiento legacy con el actual. En
particular, el comando de precarga `0x600` ya no forma parte del contrato
vigente ECU <-> AMS.

## 2. Resumen del contrato ECU <-> AMS actual

La ECU ya no ordena la precarga por CAN. El flujo vigente es:

1. La ECU publica continuamente `0x100` con `inv_dc_bus_voltage`.
2. El AMS decide la precarga con su propia logica interna.
3. El AMS publica `0x020[0]` cuando la precarga ha terminado.
4. El AMS publica `0x4A0[0]` con su estado FSM.
5. La ECU usa `0x020` para avanzar y `0x4A0` para distinguir `Start` de
   `Error`.

Consecuencia importante:

- `0x600` esta retirado en `dev`.
- La ECU espera `0x020`.
- Si `0x4A0[0] == 5`, la ECU entra en `CTRL_ST_AMS_ERROR` y no rearma hasta
  que el AMS salga de `Error`.

## 3. CAN critico

### 3.1 ECU -> inversor

#### ID `0x360` - `RX_SETPOINT_1`

- Bus: `CAN_BUS_INV` (`FDCAN1`)
- Direccion: ECU -> inversor
- DLC: `3`
- Uso: seleccion de modo del inversor

| Byte | Contenido | Significado |
|---|---|---|
| `data[0]` | `0x00` | no usado |
| `data[1]` | `0x00` | no usado |
| `data[2]` | modo | `INV_MODE_STANDBY`, `INV_MODE_READY`, `INV_MODE_TORQUE`, `INV_MODE_RESET` |

#### ID `0x362` - `RX_SETPOINT_3`

- Bus: `CAN_BUS_INV` (`FDCAN1`)
- Direccion: ECU -> inversor
- DLC: `4`
- Uso: consigna legacy de par

| Byte | Contenido | Significado |
|---|---|---|
| `data[0]` | `0x00` | no usado |
| `data[1]` | `0x00` | no usado |
| `data[2]` | byte bajo | `legacy_torque & 0xFF` |
| `data[3]` | byte alto | `(legacy_torque >> 8) & 0xFF` |

Nota: `legacy_torque` sale de `torque_pct_to_legacy_command()`.

### 3.2 ECU -> AMS

#### ID `0x100` - `VCU_heartbeat`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Direccion: ECU -> AMS
- DLC: `2`
- IDE: estandar de 11 bits
- Fuente de verdad: `Core/Inc/can/messages/vcu_heartbeat.def`
- Uso: heartbeat de tension de bus DC

| Byte | Contenido | Significado |
|---|---|---|
| `data[0]` | byte bajo | `inv_dc_bus_voltage` |
| `data[1]` | byte alto | `inv_dc_bus_voltage` |

Notas:

- El valor se codifica en little-endian.
- La ECU lo emite en cada `Control_Step10ms()`, en todos los estados.
- Este frame ya no se usa solo para precarga; es un heartbeat permanente hacia
  el AMS.

#### ID `0x600` - retirado

El comando de precarga `0x600` ya no se emite en la rama `dev`.

Estado actual:

- No se construye en `Core/Src/control.c`.
- La ECU no ordena la precarga por CAN.
- La precarga queda gobernada por el AMS a partir de sus entradas propias y del
  heartbeat `0x100`.

### 3.3 Inversor -> ECU

#### ID `0x461` - `TX_STATE_2`

- Bus: `CAN_BUS_INV` (`FDCAN1`)
- Direccion: inversor -> ECU
- Uso: estado principal del inversor

| Byte | Contenido | Significado |
|---|---|---|
| `data[2]` | error | `inv_error` cuando `inv_state == 10` o `11` |
| `data[4]` nibble bajo | estado | `inv_state` |

#### ID `0x463` - `TX_STATE_4`

- Bus: `CAN_BUS_INV` (`FDCAN1`)
- Direccion: inversor -> ECU
- DLC esperado: `8`
- Uso: rpm del inversor

| Byte | Contenido | Significado |
|---|---|---|
| `data[5]` | byte bajo | `inv_rpm` |
| `data[6]` | byte medio | `inv_rpm` |
| `data[7]` nibble bajo | nibble alto | `inv_rpm` |

Nota: el firmware recompone un entero con signo de 20 bits.

#### ID `0x464` - `TX_STATE_5`

- Bus: `CAN_BUS_INV` (`FDCAN1`)
- Direccion: inversor -> ECU
- DLC esperado: `8`
- Uso: temperaturas del inversor

| Byte | Contenido | Significado |
|---|---|---|
| `data[0]` | temperatura | `inv_motor_temp` |
| `data[1]` | temperatura | `inv_igbt_temp` |
| `data[2]` | temperatura | `inv_air_temp` |

#### ID `0x465` - `TX_STATE_6`

- Bus: `CAN_BUS_INV` (`FDCAN1`)
- Direccion: inversor -> ECU
- DLC esperado: `8`
- Uso: velocidad y corriente medidas

| Byte | Contenido | Significado |
|---|---|---|
| `data[2]` | byte bajo | `inv_speed_actual` |
| `data[3]` | byte alto | `inv_speed_actual` |
| `data[4]` | byte bajo | `inv_current_actual` |
| `data[5]` | byte alto | `inv_current_actual` |

Nota: ambos campos se leen en little-endian de 16 bits y se guardan como
`int32_t`.

#### ID `0x466` - `TX_STATE_7`

- Bus: `CAN_BUS_INV` (`FDCAN1`)
- Direccion: inversor -> ECU
- DLC esperado: `6`
- Uso: tension de bus DC y marca de VDC listo

| Byte | Contenido | Significado |
|---|---|---|
| `data[2]` | byte bajo | `inv_dc_bus_voltage` |
| `data[3]` | byte alto | `inv_dc_bus_voltage` |

Ademas, al recibir esta trama valida:

- `inv_vdc_ready = 1`

### 3.4 AMS -> ECU

#### ID `0x020` - `ID_ACK_PRECARGA`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Direccion: AMS -> ECU
- Uso: confirmacion de precarga completada

| Byte | Contenido | Significado |
|---|---|---|
| `data[0]` | `0` | `ok_precarga = 0` |
| `data[0]` | distinto de `0` | `ok_precarga = 1` |

Notas:

- La ECU usa este bit como criterio de `precharge_complete()`.
- La ECU ya no aplica la heuristica antigua de tension fija para asumir que la
  precarga ha terminado.

#### ID `0x4A0` - `ID_AMS_STATUS`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Direccion: AMS -> ECU
- Uso: estado FSM del AMS

| Byte | Contenido | Significado |
|---|---|---|
| `data[0]` | estado | `ams_state` |

Mapa consumido por la ECU:

- `0` = `Start`
- `1` = `Precharge`
- `2` = `Transition`
- `3` = `Run`
- `4` = `Charge`
- `5` = `Error`

Nota importante:

- `0x020` por si solo no distingue entre "todavia no ha terminado" y "AMS en
  error".
- Por eso `0x4A0[0]` forma parte del contrato operativo actual.

#### ID `0x12C` - `ID_V_CELDA_MIN`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Direccion: AMS -> ECU
- Uso: tension minima de celda

| Byte | Contenido | Significado |
|---|---|---|
| `data[0]` | byte alto | `v_celda_min` |
| `data[1]` | byte bajo | `v_celda_min` |

Nota: la ECU la interpreta en big-endian.

#### ID `0x130` - `ID_ACU_SOC`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Direccion: AMS -> ECU
- Uso: estado de carga

| Byte | Contenido | Significado |
|---|---|---|
| `data[0]` | SOC | `soc` |

#### ID `0x131` - `ID_ACU_VMIN_012`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Direccion: AMS -> ECU
- DLC esperado: `>= 6`
- Uso: tension minima de modulos 0, 1 y 2

| Bytes | Significado |
|---|---|
| `data[0..1]` | `vmin_modulo[0]` big-endian |
| `data[2..3]` | `vmin_modulo[1]` big-endian |
| `data[4..5]` | `vmin_modulo[2]` big-endian |

#### ID `0x132` - `ID_ACU_VMIN_34`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Direccion: AMS -> ECU
- DLC esperado: `>= 4`
- Uso: tension minima de modulos 3 y 4

| Bytes | Significado |
|---|---|
| `data[0..1]` | `vmin_modulo[3]` big-endian |
| `data[2..3]` | `vmin_modulo[4]` big-endian |

#### ID `0x133` - `ID_ACU_VMAX_012`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Direccion: AMS -> ECU
- DLC esperado: `>= 6`
- Uso: tension maxima de modulos 0, 1 y 2

| Bytes | Significado |
|---|---|
| `data[0..1]` | `vmax_modulo[0]` big-endian |
| `data[2..3]` | `vmax_modulo[1]` big-endian |
| `data[4..5]` | `vmax_modulo[2]` big-endian |

#### ID `0x134` - `ID_ACU_VMAX_34`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Direccion: AMS -> ECU
- DLC esperado: `>= 4`
- Uso: tension maxima de modulos 3 y 4

| Bytes | Significado |
|---|---|
| `data[0..1]` | `vmax_modulo[3]` big-endian |
| `data[2..3]` | `vmax_modulo[4]` big-endian |

#### ID `0x135` - `ID_ACU_CURR_PACK_DCDC`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Direccion: AMS -> ECU
- DLC esperado: `>= 4`
- Uso: corriente del acumulador y del DCDC

| Bytes | Significado |
|---|---|
| `data[0..1]` | `corriente_accu` signed big-endian |
| `data[2..3]` | `corriente_dcdc` signed big-endian |

#### ID `0x136` - `ID_ACU_TMAX_012`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Direccion: AMS -> ECU
- DLC esperado: `>= 6`
- Uso: temperatura maxima de modulos 0, 1 y 2

| Bytes | Significado |
|---|---|
| `data[0..1]` | `temp_max_modulo[0]` signed big-endian |
| `data[2..3]` | `temp_max_modulo[1]` signed big-endian |
| `data[4..5]` | `temp_max_modulo[2]` signed big-endian |

#### ID `0x137` - `ID_ACU_TMAX_34_DCDC`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Direccion: AMS -> ECU
- DLC esperado: `>= 6`
- Uso: temperatura maxima de modulos 3 y 4 y temperatura DCDC

| Bytes | Significado |
|---|---|
| `data[0..1]` | `temp_max_modulo[3]` signed big-endian |
| `data[2..3]` | `temp_max_modulo[4]` signed big-endian |
| `data[4..5]` | `temp_dcdc` signed big-endian |

## 4. Estado de control relevante para el contrato con AMS

La FSM de control de la ECU tiene estos estados internos:

- `CTRL_ST_WAIT_INV_VDC_CONFIG`
- `CTRL_ST_BOOT`
- `CTRL_ST_WAIT_PRECHARGE_ACK`
- `CTRL_ST_WAIT_START_BRAKE`
- `CTRL_ST_R2D_DELAY`
- `CTRL_ST_WAIT_INV_STANDBY`
- `CTRL_ST_ACTIVE`
- `CTRL_ST_AMS_ERROR`

Comportamiento relevante:

- La ECU espera a ver `TX_STATE_7` del inversor antes de salir de
  `CTRL_ST_WAIT_INV_VDC_CONFIG`.
- En `CTRL_ST_BOOT` y `CTRL_ST_WAIT_PRECHARGE_ACK`, la ECU no manda comando de
  precarga; solo mantiene `0x100` y espera `0x020`.
- Si la precarga no termina en `PRECHARGE_TIMEOUT_MS = 10000`, vuelve a
  `CTRL_ST_BOOT`.
- Si `ams_state == 5`, entra en `CTRL_ST_AMS_ERROR`, inhibe drive y no reintenta
  precarga hasta que el AMS salga de `Error`.

## 5. CAN dash

Estas tramas se generan en `DashTask` a partir de `telemetry_frame_t`.
La salida al dash va por `CAN_BUS_DASH` (`FDCAN3`).

### ID `0x510`

- DLC: `8`
- Uso: estado general, flags y secuencia

| Byte | Significado |
|---|---|
| `data[0]` | `inv_state` |
| `data[1]` | `torque_total` truncado a `uint8_t` |
| `data[2]` | flags: `bit0=flag_EV_2_3`, `bit1=flag_T11_8_9`, `bit2=inv_error!=0` |
| `data[3]` | `ok_precarga` |
| `data[4]` | `boton_arranque` |
| `data[5]` | `frame->kind` |
| `data[6]` | `sequence` byte bajo |
| `data[7]` | `sequence` byte alto |

### ID `0x511`

- DLC: `6`
- Uso: entradas del conductor

| Bytes | Significado |
|---|---|
| `data[0..1]` | `s1_aceleracion` little-endian |
| `data[2..3]` | `s2_aceleracion` little-endian |
| `data[4..5]` | `s_freno` little-endian |

### ID `0x512`

- DLC: `6`
- Uso: variables electricas y de estado

| Bytes | Significado |
|---|---|
| `data[0..1]` | `inv_dc_bus_voltage` little-endian |
| `data[2..3]` | `v_celda_min` little-endian |
| `data[4]` | `inv_error` |
| `data[5]` | `inv_vdc_ready` |

### ID `0x513`

- DLC: `6`
- Uso: temperaturas del inversor

| Bytes | Significado |
|---|---|
| `data[0..1]` | `inv_motor_temp` little-endian |
| `data[2..3]` | `inv_igbt_temp` little-endian |
| `data[4..5]` | `inv_air_temp` little-endian |

### ID `0x514`

- DLC: `4`
- Uso: `inv_rpm` completo

### ID `0x515`

- DLC: `4`
- Uso: `inv_speed_actual` completo

### ID `0x516`

- DLC: `4`
- Uso: `inv_current_actual` completo

## 6. Snapshot interno (`app_inputs_t`)

El snapshot interno compartido por la aplicacion contiene, entre otros, estos
campos relevantes para ECU <-> AMS:

- `inv_dc_bus_voltage`
- `inv_vdc_ready`
- `v_celda_min`
- `ok_precarga`
- `ams_state`
- `soc`
- `vmin_modulo[5]`
- `vmax_modulo[5]`
- `corriente_accu`
- `corriente_dcdc`
- `temp_dcdc`
- `temp_max_modulo[5]`

Definicion: `Core/Inc/app_state.h`

## 7. Notas de mantenimiento

Si cambia el contrato ECU <-> AMS, hay que revisar como minimo:

- `Core/Src/control.c`
- `Core/Src/can.c`
- `Core/Inc/can/messages/vcu_heartbeat.def`
- este documento

En particular, no reintroducir `0x600` en la documentacion salvo que vuelva a
existir en codigo.
