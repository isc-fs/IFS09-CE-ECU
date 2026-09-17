# ECU <-> ACU/AMS - mensajes actuales

> ⚠️ **HISTÓRICO (17-jun-2026) — NO describe el `dev` actual.** Cita como fuentes
> `Core/Src/control.c`, `Core/Src/can.c` y `Core/Inc/app_state.h`, **borrados en el rewrite
> a C++17**, y afirma que la ECU parsea `0x130`, cosa que no ocurre en ningún sitio de
> `Core/`. También enlaza un `docs/CAN_MAP.md` que nunca llegó a existir.
>
> **Fuente de verdad actual:** `Core/Inc/can/messages/*.def` → [`dbc/ecu.dbc`](../dbc/ecu.dbc).
> El contrato ECU↔AMS vigente (heartbeat `0x100`, `0x020`, `0x12C`, `0x4A0`, `0x131`–`0x137`)
> está resumido en [`../CLAUDE.md`](../../CLAUDE.md). Se conserva sólo por historia.

Fecha: 17-jun-2026

## 1. Alcance

Este documento resume solo los mensajes entre la ECU y el ACU/AMS en el estado
actual de `dev clone`.

Referencias de codigo:

- ECU: `Core/Src/control.c`
- ECU: `Core/Src/can.c`
- ECU: `Core/Inc/can/messages/vcu_heartbeat.def`
- AMS: `Core/Inc/app/ams_config.hpp`
- AMS: `Core/Inc/app/state_machine.hpp`
- AMS: `docs/CAN_MAP.md`

Nota de nomenclatura:

- En la ECU aparecen nombres legacy como `ACU`.
- En el firmware nuevo del acumulador la logica se describe como `AMS`.
- En este documento se usa `ACU/AMS` para evitar ambiguedad.

## 2. Resumen funcional

La ECU ya no manda una orden de precarga dedicada.

El contrato operativo actual es:

1. La ECU transmite continuamente `0x100` con la tension de bus DC.
2. El ACU/AMS decide internamente la precarga.
3. El ACU/AMS publica `0x020` cuando la precarga ya esta completada.
4. El ACU/AMS publica `0x4A0[0]` con su estado FSM.
5. La ECU usa `0x020` para avanzar y `0x4A0` para distinguir `Start` de `Error`.

Consecuencia:

- `0x600` ya no forma parte del contrato actual.

## 3. ECU -> ACU/AMS

### ID `0x100` - `VCU_heartbeat`

- Bus ECU: `CAN_BUS_ACU` (`FDCAN2`)
- Bus AMS: `FDCAN1`
- Direccion: ECU -> ACU/AMS
- Tipo: estandar 11-bit
- DLC: `2`
- Periodo observado en ECU: cada `Control_Step10ms()`
- Fuente de verdad ECU: `Core/Inc/can/messages/vcu_heartbeat.def`

Payload:

| Byte | Campo | Formato |
|---|---|---|
| `data[0]` | `dc_bus_voltage` LSB | little-endian |
| `data[1]` | `dc_bus_voltage` MSB | little-endian |

Semantica:

- La ECU envia `inv_dc_bus_voltage`.
- El ACU/AMS usa este mensaje como heartbeat del vehiculo.
- En modo coche, la frescura de este mensaje participa en:
  - bloqueo `Car` frente a `Charger`
  - deteccion de `VcuStale`
  - criterio de fin de precarga por tension de bus

Observaciones:

- Si este mensaje desaparece mientras el AMS exige VCU presente, el AMS puede
  latchear `Error`.
- Este mensaje se envia en todos los estados de la FSM de control de la ECU.

### ID `0x600` - retirado

Estado actual:

- La ECU no envia `0x600`.
- La precarga ya no se arranca por una orden CAN desde la ECU.

## 4. ACU/AMS -> ECU

### ID `0x020` - `ok_precharge`

- Direccion: ACU/AMS -> ECU
- Tipo: estandar 11-bit
- DLC: `1`
- Fuente AMS: `AcuTxOkPrechargeId = 0x020`

Payload:

| Byte | Campo | Formato |
|---|---|---|
| `data[0]` | `ok_precharge` | `0` o `1` |

Semantica:

- `1`: el ACU/AMS considera completada la precarga.
- `0`: la precarga no esta completada.

Uso en ECU:

- Se parsea en `Core/Src/can.c` y actualiza `ok_precarga`.
- `precharge_complete()` en `Core/Src/control.c` depende de este valor.

Nota:

- La ECU actual no usa freshness explicita para este mensaje.

### ID `0x4A0` - `AMS status`

- Direccion: ACU/AMS -> ECU
- Tipo: estandar 11-bit
- DLC: `8` en la telemetria AMS, aunque la ECU solo consume `byte 0`
- Fuente AMS: `AmsTelemStatusId = 0x4A0`

Payload relevante para ECU:

| Byte | Campo | Formato |
|---|---|---|
| `data[0]` | `ams_state` | `uint8_t` |

Mapa de estados consumido por ECU:

- `0` = `Start`
- `1` = `Precharge`
- `2` = `Transition`
- `3` = `Run`
- `4` = `Charge`
- `5` = `Error`

Uso en ECU:

- Se parsea en `Core/Src/can.c` y actualiza `ams_state`.
- Si `ams_state == 5`, la ECU entra en `CTRL_ST_AMS_ERROR`.
- Mientras el AMS siga en `Error`, la ECU no rearma drive.

Nota:

- Este mensaje es el que permite a la ECU distinguir:
  - `Start` rearmable
  - `Error` latcheado

### ID `0x12C` - `v_celda_min`

- Direccion: ACU/AMS -> ECU
- Tipo: estandar 11-bit
- DLC: `2`

Payload:

| Byte | Campo | Formato |
|---|---|---|
| `data[0]` | byte alto | big-endian |
| `data[1]` | byte bajo | big-endian |

Semantica:

- Tension minima de celda.

Uso en ECU:

- Se guarda en `v_celda_min`.
- Se usa para limitar par en `apply_vmin_torque_limit()`.

### ID `0x130` - `soc`

- Direccion: ACU/AMS -> ECU
- Tipo: estandar 11-bit
- DLC: `1`

Uso en ECU:

- `data[0] -> soc`

Nota:

- En AMS figura como ID reservada o diferida en parte de la documentacion.
- La ECU actual si tiene parser para ella.

### IDs `0x131` y `0x132` - `vmin_modulo`

- Direccion: ACU/AMS -> ECU
- Tipo: estandar 11-bit

`0x131`:

| Bytes | Campo |
|---|---|
| `data[0..1]` | `vmin_modulo[0]` big-endian |
| `data[2..3]` | `vmin_modulo[1]` big-endian |
| `data[4..5]` | `vmin_modulo[2]` big-endian |

`0x132`:

| Bytes | Campo |
|---|---|
| `data[0..1]` | `vmin_modulo[3]` big-endian |
| `data[2..3]` | `vmin_modulo[4]` big-endian |

### IDs `0x133` y `0x134` - `vmax_modulo`

- Direccion: ACU/AMS -> ECU
- Tipo: estandar 11-bit

`0x133`:

| Bytes | Campo |
|---|---|
| `data[0..1]` | `vmax_modulo[0]` big-endian |
| `data[2..3]` | `vmax_modulo[1]` big-endian |
| `data[4..5]` | `vmax_modulo[2]` big-endian |

`0x134`:

| Bytes | Campo |
|---|---|
| `data[0..1]` | `vmax_modulo[3]` big-endian |
| `data[2..3]` | `vmax_modulo[4]` big-endian |

### ID `0x135` - corrientes

- Direccion: ACU/AMS -> ECU
- Tipo: estandar 11-bit
- DLC esperado: `>= 4`

Payload:

| Bytes | Campo |
|---|---|
| `data[0..1]` | `corriente_accu` signed big-endian |
| `data[2..3]` | `corriente_dcdc` signed big-endian |

### IDs `0x136` y `0x137` - temperaturas

- Direccion: ACU/AMS -> ECU
- Tipo: estandar 11-bit

`0x136`:

| Bytes | Campo |
|---|---|
| `data[0..1]` | `temp_max_modulo[0]` signed big-endian |
| `data[2..3]` | `temp_max_modulo[1]` signed big-endian |
| `data[4..5]` | `temp_max_modulo[2]` signed big-endian |

`0x137`:

| Bytes | Campo |
|---|---|
| `data[0..1]` | `temp_max_modulo[3]` signed big-endian |
| `data[2..3]` | `temp_max_modulo[4]` signed big-endian |
| `data[4..5]` | `temp_dcdc` signed big-endian |

## 5. Como usa la ECU estos mensajes

Secuencia simplificada:

1. La ECU espera `inv_vdc_ready` del inversor.
2. La ECU entra en `BOOT`.
3. La ECU mantiene `0x100` al ACU/AMS.
4. La ECU espera `0x020 == 1`.
5. Si recibe `0x4A0 == Error`, salta a `CTRL_ST_AMS_ERROR`.
6. Si recibe `0x020 == 1`, pasa a esperar `Start + freno`.

Timeout relevante ECU:

- `PRECHARGE_TIMEOUT_MS = 10000`

## 6. Debilidades actuales del contrato

Las principales debilidades que se ven hoy son:

- La ECU no publica un comando de sesion o rearme al ACU/AMS.
- La ECU no usa freshness explicita para `0x020` ni para `0x4A0`.
- No hay `boot counter` ni `session id` compartido.
- Un rearranque unilateral puede dejar durante un tiempo valores viejos en la
  ECU hasta que lleguen nuevas tramas.

## 7. Resumen corto

Mensajes minimos realmente criticos para el arranque ECU <-> ACU/AMS:

- ECU -> ACU/AMS:
  - `0x100`

- ACU/AMS -> ECU:
  - `0x020`
  - `0x4A0`

Mensajes adicionales de supervision/telemetria:

- `0x12C`
- `0x130`
- `0x131`
- `0x132`
- `0x133`
- `0x134`
- `0x135`
- `0x136`
- `0x137`
