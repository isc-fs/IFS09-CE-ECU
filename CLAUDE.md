# CLAUDE.md — IFS09-CE-ECU

Guía de contexto para sesiones futuras. Leer antes de tocar código.

> **New to this repo? Start here, in this order.** Docs are being migrated to
> English; the code, every `.def` and the generated DBC are already English.
>
> | | |
> |---|---|
> | 1 | This file — architecture, the FSM, the CAN contract, and the traps |
> | 2 | [`docs/flashing.md`](docs/flashing.md) — build, flash, **and un-brick**. Read the recovery section BEFORE you need it |
> | 3 | [`docs/derating.md`](docs/derating.md) — the four torque caps and how they compose |
> | 4 | [`docs/commissioning.md`](docs/commissioning.md) — on-car bring-up and every unmeasured constant |
>
> **The four things most likely to cost you a weekend, all of them deliberate
> and none of them bugs:**
>
> 1. **Bench stubs are config values, not build flags.** `TorqueCap`,
>    `StubNoAms`, `StubNoInverter`, `StubStart`, `StubBrakeRaw` in
>    `ecu_config.hpp`. A flight build has `TorqueCap = 100` and the rest
>    `false`. All five are announced on the **ungated `0x704`** — check the bus,
>    not the source tree.
> 2. **The torque sent to the inverter is NEGATED.** Mechanical constraint of
>    the motor mounting (`inverter.cpp`). Remove it and the car drives backwards.
> 3. **PA5/PA6/PA7 are nRF24 bit-bang, not SPI1**, and **PB6 is the DC-link
>    discharge**, not a spare GPIO. See [`docs/PINES_RUTEADOS_IOC.md`](docs/PINES_RUTEADOS_IOC.md).
> 4. **MessageRAM offsets must be re-applied after every CubeMX regen** — see
>    the warning in the CAN section below.
>
> **Anything under [`docs/historical/`](docs/historical/) describes firmware that
> no longer exists.** Do not cite it.

---

## Qué es este proyecto

ECU de competición FSAE (fórmula SAE eléctrico) sobre **STM32H733ZG**. Es una app
**FreeRTOS lean en C++17**: un núcleo de control puro y host-testable (`ecu::Controller`)
rodeado de una fina capa de tareas RTOS y drivers HAL. Enlaza **sobre el
stm32-can-bootloader** (app en `0x08020000`), así que toda la recuperación y el
flasheo van por CAN.

**Rama principal:** `dev` (protegida: PR + checks de firmware/SIL).
**Plataforma:** STM32H733ZG · HAL FW_H7 · CubeMX (`ECU.ioc`).

---

## Arquitectura en una línea

```
FDCAN RX ISR → can_rx_queue → CanRxTask → VehicleService (estado RX compartido)
                                               │ (snapshot)
io_signals (ADC3 + GPIO) → ControlTask 10 ms → ecu::Controller::step()
                                               │
                          salidas: RTDS/LEDs · setpoints inversor · can_tx_post()
                                               ↓
                              can_tx_queue → CanTxTask → FDCAN TX (FDCAN1 INV / FDCAN2 ACU)
```

- **`ecu::Controller`** (núcleo puro, sin HAL): FSM de arranque + torque + cortes de
  plausibilidad FSAE. Compila en host → SIL.
- **VehicleService**: estado recibido por CAN. `CanRxTask` es su **único escritor**;
  `ControlTask` lo *snapshot*-ea cada ciclo. Sustituye al viejo `g_in` con mutex.
- **`can_tx_post()`**: cola productora; `CanTxTask` es el **único** punto que toca el
  TX del FDCAN, así que ninguna otra tarea bloquea en el FIFO de transmisión.

### Tareas RTOS (`Core/Src/freertos.c` + `Core/Src/app/*.cpp`)

| Tarea          | Prioridad     | Período | Rol |
|----------------|---------------|---------|-----|
| `App_InitTask` | High (one-shot)| —      | Levanta FDCAN1/2 (filtros + notif. RX) y arranca FDCAN3 (sólo TX), luego se autodestruye. Corazón del fix #48. |
| `ControlTask`  | **Realtime**  | 10 ms   | Único actor de seguridad. Lee IO, snapshot, `step()`, salidas, `0x100` **en todos los estados**, stream pit-diag opcional, **único que patea el IWDG**. |
| `CanRxTask`    | AboveNormal   | ~RX     | Drena `can_rx_queue`. Despacha: trigger BL → cmd pit-diag → VehicleService. |
| `CanTxTask`    | AboveNormal   | ~TX     | Drena `can_tx_queue` → `HAL_FDCAN_AddMessageToTxFifoQ`. |
| `DiagTask`     | Low           | 1000 ms | Emite `0x704` (health) **aparte de ControlTask**, para sobrevivir a un cuelgue de éste. |
| `TelemetryTask`| BelowNormal   | 200 ms  | Snapshot → **dashboard por FDCAN3** (18 tramas, ver [`docs/CAN3_MAP.md`](docs/CAN3_MAP.md)) + **snapshot de radio nRF24** (102 B en 5 fragmentos, ver [`docs/RADIO_SNAPSHOT_MAP.md`](docs/RADIO_SNAPSHOT_MAP.md)). |
| `defaultTask`  | Low           | idle    | Task de CubeMX; no hace trabajo de aplicación. |
| `GpsTask`      | Low           | 20 ms   | Drena el anillo NMEA (ISR USART10), parsea y publica `0x508`/`0x509` a 5 Hz + `GpsService`. Creada en los **bloques USER CODE** de `freertos.c` (sobrevive a un regen de CubeMX). |

---

## FSM de arranque (`ecu::CtrlState`)

> ⚠️ **Este bloque es un ÍNDICE, no una copia de la lógica.** Las transiciones, las
> condiciones exactas y las palabras de modo que se mandan al inversor viven en
> [`Core/Src/app/control.cpp`](Core/Src/app/control.cpp) (`Controller::step()`:
> primero el `switch` de transiciones, luego el de salidas), con el razonamiento y
> las referencias a issues en comentarios **ahí mismo, junto al código**.
>
> Antes esto duplicaba esa lógica línea a línea y **derivó dos veces en una sola
> semana**: lo corregí en #163 y volvió a quedar obsoleto en cuanto #168 cambió la
> subida del inversor. Un texto que hay que actualizar a mano cada vez que cambia
> un `if` no se actualiza. Si necesitas el detalle, lee el código. Si cambias el
> código, **no** hace falta volver aquí — sólo si añades o quitas un estado.

Estados, en orden de arranque:

| Estado | Para qué |
|---|---|
| `WaitInvVdcConfig` | espera a que el inversor reporte su configuración de bus DC |
| `Precharge` | espera el veredicto de precarga de la AMS; reintenta por timeout |
| `WaitStartBrake` | espera el trigger de R2D (manual o DV) |
| `R2dDelay` | hace sonar el RTDS antes de habilitar par |
| `WaitInvStandby` | sube el inversor hasta Ready |
| `Active` | par en marcha |
| `AmsError` | inhibición; se entra desde CUALQUIER estado |

### Invariantes que el `switch` no cuenta por sí solo

Esto sí vive aquí porque son **decisiones**, no mecánica — y no se leen de un
vistazo en el código:

- **El trigger ES la decisión de modo (#17).** En `WaitStartBrake`, el gate que
  dispare (manual = botón + freno; DV = `0x510` fresco + freno EBS verificado en
  nuestro propio sensor) latchea el modo para **todo el ciclo de marcha**. Manual
  tiene precedencia. No se cambia de modo en caliente.
- **En DV el par NUNCA cae de vuelta a las APPS.** La fuente es el `0x507`
  condicionado; si el stream está stale el par es 0. No hay piloto sentado.
- **`AmsError` distingue el Start re-armable de la AMS del Error latcheado**
  (`0x4A0` byte0 == 5): en Error la ECU **inhibe** en vez de reintentar precarga,
  y ahí se suprime también la recuperación de fallo del inversor.
- **La recuperación de fallo del inversor corre ANTES de `Active`, no sólo dentro.**
  El inversor puede arrancar ya latcheado en hard fault; si sólo se intentara
  desde `Active`, la FSM se quedaría esperando para siempre en `WaitInvStandby`.
- **`Active` exige que el inversor SIGA en la marcha, no sólo que no esté en
  fallo (#191).** Si `inv_state` sale de `Ready(4)`/`TorqueEnable(6)` con la TS
  todavía arriba, se vuelve a `WaitInvStandby` para re-escalar. Lo encontró un
  overspeed: el inversor se aparca en `Standby(3)`, que **no es estado de fallo**,
  así que la ráfaga de recuperación no disparaba, y `Standby < SoftFault` hacía
  que se siguiera mandando `TorqueEnable` a 100 Hz para siempre. Como la TS nunca
  caía, la única salida de `Active` (`ok_precharge`) tampoco. Sin power-cycle de
  LV no había forma de re-armar. Se cuenta en `0x708 inv_redrive_count`.
  **La marcha se reanuda en cuanto vuelve a `Ready`, sin R2D nuevo** — decisión
  del equipo (2026-08-02): prima la recuperación rápida.
- **El par va NEGADO** hacia el inversor: restricción mecánica del montaje del
  motor, no un bug (ver `inverter.cpp`).
- **La calibración de pedales es RUNTIME** (#169): los umbrales de APPS y freno
  llegan por `CtrlInputs::cal`, no de `ecu_config.hpp`. Los valores de ese header
  son sólo los defaults de fábrica.

---

## Buses CAN

| Bus       | FDCAN  | Rol |
|-----------|--------|-----|
| **INV**   | FDCAN1 | Inversor NX/EMC (IDs estándar). RX 0x461/0x463/0x464/0x465/0x466/0x467/0x468 · TX 0x360/0x362. |
| **ACU**   | FDCAN2 | AMS + Pit-Tool + uDV (compartido). RX 0x020/0x021/0x12C/0x4A0/0x131-0x137/0x507/0x50A/0x510/0x002/0x7E0/0x7E2 · TX 0x100 + bloque uDV + stream pit-diag. |
| **DASH**  | FDCAN3 | Dashboard, **sólo TX**. 18 tramas `0x510..0x521` desde `TelemetryTask` — ver [`docs/CAN3_MAP.md`](docs/CAN3_MAP.md). |

El `0x600` está **retirado** (la AMS auto-dispara precarga).

> ⚠️ **Offsets de MessageRAM — re-aplicar tras CADA regen de CubeMX.** Los tres FDCAN
> comparten la SRAMCAN y **no hay campo en la GUI**, así que un regen los pone todos a 0 y
> se solapan (ésta fue la raíz del TX-dead #48). Ventanas actuales, en `MX_FDCAN1/2/3_Init`
> de `Core/Src/fdcan.c`:
>
> | Periférico | `MessageRAMOffset` | Ventana |
> |------------|--------------------|---------|
> | FDCAN1     | `0`                | `[0, 387)`   |
> | FDCAN2     | `387`              | `[387, 582)` |
> | FDCAN3     | `582`              | `[582, 710)` |
>
> Si sólo se restaura el 387 y se olvida el 582, FDCAN3 vuelve a solapar FDCAN1 y corrompe
> el bus del inversor **y** el heartbeat `0x100` que la AMS vigila a 200 ms para mantener
> los AIRs cerrados.

### Contrato de mensajes (generado del DSL)

**TX propias de la ECU**

- **`0x100` VCU_heartbeat** (DLC **3**) → bus ACU, cada ciclo (10 ms), **en todos los
  estados**. Contrato cargado ECU↔AMS: la AMS arma un watchdog `VcuStale` (200 ms) en Car
  mode y abre los AIRs si para.
  - `dc_bus_voltage` (LE u16, V) — **publica 0 si `0x466` está stale**, no el último
    valor. La trama sale cada 10 ms pase lo que pase, así que la TRAMA siempre está
    fresca mientras el VALOR era una lectura retransmitida sin caducidad: un inversor
    callado dejaba emitiendo tensión de pack a 100 Hz para siempre y el check de
    frescura de la AMS no puede verlo. Congelado alto satisface su criterio de
    precarga (≥95 % del pack) → **una precarga muerta pasaría su propio autotest** (#198).
  - `discharge_engaged` (bit 16) — la ECU está manteniendo el bleed conectado. **Estado
    COMANDADO**: no hay readback de contacto auxiliar, así que no detecta un relé
    atascado abierto. Ver `discharge.hpp`.
  - `dc_bus_valid` (bit 17) — `dc_bus_voltage` es una medida ACTUAL. 0 = no usar el
    valor en NINGUNA dirección.
- **Setpoints inversor** (`inverter.cpp` / `ecu_config.hpp`, aún **fuera del DSL** por E2E):
  - `0x360` EMC_RX_SETPOINT_1 — mode word (`App_State_Req` @ byte2, DLC 3).
  - `0x362` EMC_RX_SETPOINT_3 — `Torque_Nm_Req` (s16 LE @ bytes 2-3, DLC 4).
  - Se envían **planos** (bytes 0-1 = 0, sin E2E, byte-por-byte como la VCU original). El
    par va **NEGADO**: restricción mecánica del motor (drive adelante = par negativo), **no
    opcional**.

- **GPS** (MTK3339 en `USART10`, 9600 8N1, PG11 RX / PG12 TX) → bus ACU, **sin gate**, 5 Hz:
  - `0x508` `VCU_gps_position` — lat/lon como grados×1e7, **s32 LE** (DBC con escala 1e-7).
  - `0x509` `VCU_gps_status` — velocidad **km/h**×100 (convertida de nudos NMEA, ×1.852),
    rumbo deg×100, satélites, `has_fix` y `nmea_count` (contador de sentencias = **liveness**:
    si no sube, el módulo/cableado está muerto; si sube con `has_fix=0`, sólo falta cielo).
  - Va **ungated** porque lo consume el uDV para localización (igual que `0x504/0x505/0x506`);
    la pit-tool y MingoCAN lo ven gratis (mismo bus + DBC generado). **Gatea siempre por `has_fix`**:
    `0x508` mantiene la última posición válida cuando se pierde el fix.
  - El parser (`gps_nmea.cpp`) es puro y host-testable → SIL `--test-gps`.

**Stream pit-diag** (bus ACU, gated por `0x7E0` = `DEADBEEF`, deshabilita con `0`, ACK `0x7E1`):

| ID    | Frame            | Contenido |
|-------|------------------|-----------|
| 0x700 | PitDiag_status   | FSM · inv_state · flags de control (bits) · torque · v_cell_min |
| 0x701 | PitDiag_pedals   | APPS1/2 raw+%, brake raw |
| 0x702 | PitDiag_inverter | DC-bus V · rpm (signed) · error |
| 0x703 | PitDiag_fwinfo   | semver + 4 bytes de git hash |
| 0x704 | PitDiag_health   | heap · liveness por tarea (bits) · reset_cause · uptime · last_fault · **anuncio de stubs** (`stub_no_ams`/`no_inverter`/`start`/`brake`/**`torque_cap`**) |
| 0x705 | PitDiag_brake    | `brake_pressure` en bar (mapa ABSOLUTO del EPT1400, sin calibración — `brake_pressure_dbar()`) + `brake_pct` (escalado full-ADC hasta que `BrakeRestRaw` != 0, así que marca ~13 % en reposo) |
| 0x706 | PitDiag_inverter_temps | temps board / power-stage / motor1 / motor2 (byte crudo −50 = °C; 0xFF = sensor desconectado) **+ el cap térmico del motor** (#177): `motor_temp_used_degC` (el sensor válido más caliente, filtrado), `thermal_cap_pct`, y `temp_s1_valid` / `temp_s2_valid` / `temp_unknown` / `thermal_capped`. `temp_unknown=1` = ningún sensor usable → cap fijo `MotorTempUnknownCapPct`; **raw 0 decodifica a −50 °C, así que "parece frío" es el estado por defecto al arrancar** |
| 0x707 | PitDiag_dv       | `dv_r2d_req` · `brake_over_limit` · `r2d_confirm` · torque uDV — diagnóstico del modo driverless |
| 0x7E2 | PitCal_cmd (RX)  | Sesión de calibración de pedales: comando + punto de captura + guard `0xCA11B0DE` (CRC-32 del set en `COMMIT`) |
| 0x7E3 | PitCal_status    | Estado de sesión · último cmd · result · máscara de capturas · flags de validación |
| 0x7E4 / 0x7E5 | PitCal_apps / PitCal_brake | Lectura de los valores (almacenados o staged) tras `READ_STORED` / `READ_STAGED` |
| 0x70B | PitDiag_inv_foc | Realimentación FOC del inversor, de `0x463`+`0x465`: `current_d_A`/`current_q_A` (LSB 1/32 A), `volt_modulus` (per-mille) y `ctrl_mode`/`ctrl_type`/`cmd_src` + frescura por trama. **`current_q_A` es el eje que produce par**: si se aplana mientras `torque_req` sigue subiendo, el que limita es el INVERSOR. **`volt_modulus` cerca de 1000 = techo de tensión** (debilitamiento de campo): no hay más corriente disponible a esa velocidad y ningún cambio en la ECU lo levanta |
| 0x70C | PitDiag_inv_torque | **Pedido / techo / entregado en una sola trama**: `torque_req` (lo que ponemos en `0x362`, negativo = marcha adelante), `torque_max_feas` (`0x467`, el inversor declarando su propio techo — ⚠ unidad SIN confirmar, el DBC del fabricante pone "Ndm") y `torque_est` (`0x468`). Si `torque_max_feas` < `torque_req`, limita el inversor |
| 0x70D | PitDiag_power | **Lo que retira `DrivetrainEffPct` como suposición**: `shaft_power` (mecánica COMANDADA) junto a `ac_power` (medida propia del inversor, `ACBus_Power_W` de `0x466` — llegaba y se tiraba) + `dc_bus_V` y `accu_current` (`0x135`, el OTRO bus). De una captura a fondo justo por debajo del codo: rendimiento total = `shaft_power / (dc_bus_V × accu_current)`, y ése es el valor que debe llevar `DrivetrainEffPct` |
| 0x70A | PitDiag_pack_temp | Cap térmico del acumulador (#177): `pack_temp_used_degC` (módulo válido más caliente, filtrado) · `pack_cap_pct` · `mod0..4_used` · `pack_unknown` / `pack_capped`. **`mod*_used` es el motivo de la trama**: un módulo excluido (offline según `module_online_mask`, o lectura fuera de banda) desaparece del máximo en silencio, y así es como un pack caliente pasa desapercibido. Ojo: **0 °C es una temperatura real de pack**, así que sólo la frescura de `0x136`/`0x137` protege del estado sin inicializar |
| 0x709 | PitDiag_cell | Estimador del **cap** por celda baja (`torque = min(torque, cap_pct)`, ya no multiplica — #177): `raw_mV` (0x12C crudo) vs `est_ocv_mV` (compensado por IR + filtrado) vs `comp_mV` (la corrección aplicada) + `cap_pct` y los flags `compensated` / `raw_floor`. **Es el instrumento con el que se calibra `CellIrMilliOhm`** — ver [`docs/commissioning.md`](docs/commissioning.md) |
| 0x708 | PitDiag_inv_faults | Capas de fallo **L1 `PwrStg`** (9 bits) y **L2 `EMCtrl`** (8 bits) del 0x461, con nombre bit a bit + el lado **comandado** (`cmd_follow_n`, `cmd_flt_clear`). El DEM (L3) por sí solo no distingue un latch limpiable de una condición L1/L2 viva que lo sostiene (#148) |

> **`0x704` se emite siempre (ungated)** desde `DiagTask`, fuera del gate `0x7E0`, para que
> `reset_cause` y la liveness por tarea sean visibles en CAN nada más arrancar y sobrevivan
> a un cuelgue de ControlTask. Paridad con la AMS `0x4A2/0x4A3`.

**RX consumidas** (el sender es dueño del layout)

- `0x020` ACU_ok_precharge — precarga OK / HV viva (la FSM gatea aquí).
- `0x12C` ACU_v_cell_min — mín. tensión de celda (mV, BE) → derate de torque por celda baja.
- `0x4A0` AMS_status — `fsm_state` (5=Error) / `ams_ok` → distingue Start vs Error latcheado.
- `0x461`/`0x463`/`0x464`/`0x465`/`0x466`/`0x467`/`0x468` — inversor: App_State / rpm /
  temps / modo de control / DC-bus V + potencia AC / techo de par / par estimado.
- `0x131`–`0x137` — AMS por módulo (vmin/vmax/tmax, corrientes) → radio + dashboard.
- `0x002` BL_boot_trigger — magic `0xB007AD12` → escribe magic en RTC backup + reset.
- `0x7E0` PitDiag_cmd — magic `0xDEADBEEF` enable / `0` disable.

**Contrato uDV / driverless (#17, bus ACU)** — el ECU es el que manda en el par:

| ID | Dir | Contenido |
|----|-----|-----------|
| `0x507` | RX | `UDV_torque_cmd` — par como **entero %** (s32 LE). Se condiciona a 0..100 (negativo → 0: no hay regen en el contrato); stale → 0, **nunca** fallback a APPS. |
| `0x50A` | RX | `UDV_as_status` — estado del sistema autónomo (~10 Hz). `byte0 == 0x01 EMERGENCY` dispara el zumbador de AS Emergency en el RTDS: 3,3 Hz / 50 % durante 10 s, por FLANCO (el uDV mantiene Emergency latcheado mucho más de 10 s; la regla limita el SONIDO, no la luz). Perder la trama con el uDV en `DRIVING`/`READY` también suena — un uDV mudo a media misión es una emergencia. Ver `as_buzzer.hpp`. |
| `0x510` | RX | `UDV_r2d_request` — petición R2D autónoma (sólo vale con freno EBS duro verificado en nuestro propio sensor). |
| `0x504` | TX | `VCU_ts_active` — vista viva de `ok_precharge`, cada 100 ms (el uDV expira a 400 ms). |
| `0x505` | TX | `VCU_brake_over_limit` — verdicto del ECU sobre el freno. |
| `0x506` | TX | `VCU_motor_rpm` — rpm **mecánicas** = erpm / 10 (s32 LE). |
| `0x511` | TX | `VCU_r2d_confirm` — confirmación de R2D en modo DV. |

> ⚠️ `0x510`/`0x511` **se reutilizan** en FDCAN3 con otro significado (status/pedales del
> dashboard). Los IDs son por bus: el DBC no lleva calificador de bus, así que cargar
> `ecu.dbc` contra una captura de FDCAN3 decodifica basura. Las 18 tramas de dashboard
> están **fuera del DSL** — ver [`docs/CAN3_MAP.md`](docs/CAN3_MAP.md).

---

## CAN DSL code-first (fuente de verdad)

El mapa CAN se define una sola vez en un DSL de macros-X y de ahí se generan structs,
encoders/decoders, descriptores y el DBC:

- **`Core/Inc/can/messages/*.def`** — un frame por archivo (`CAN_MSG` / `FIELD_LE` /
  `FIELD_BE` / `FIELD_BE_S` / `FIELD_LE_BITS` para bits con nombre / `CAN_VAL` para tablas
  enum). `all_messages.inc` los registra (añadir un frame = una línea + su `.def`).
- **`Core/Inc/can/can_dsl.hpp` + `can_codecs.hpp`** — el motor de macros; expande el `.inc`
  en varias pasadas (struct / encode / decode / descriptor / guarda de solape). De aquí
  salen `ecu::<Msg>_ID` / `_DLC`.
- **`tools/dbc_dump.cpp` → `docs/dbc/ecu.dbc`** — el DBC generado, con tablas `VAL_` de enum
  (`CAN_VAL`, gated por `ECU_DSL_VALUES_PASS`) y señales de bit con nombre (`FIELD_LE_BITS`).
  El bot **dbcinator** (`.github/workflows/dbc-bot.yml`) lo regenera y fuerza-pushea en cada
  PR, así que el DBC nunca diverge del DSL.

```bash
# regenerar el DBC a mano:
c++ -std=c++17 -I Core/Inc tools/dbc_dump.cpp -o /tmp/dbc_dump && /tmp/dbc_dump
```

---

## Constantes que requieren calibración en banco

Todas viven en **`Core/Inc/app/ecu_config.hpp`** (HAL-free, host-testable). Las que siguen
tagueadas `COMMISSION` son placeholders heredados de la VCU legacy (placa IFS06) y **deben
re-medirse en el coche montado con sensores reales antes de cualquier marcha**.

> ⚠️ **No dupliques los valores aquí.** `ecu_config.hpp` es la única fuente de verdad — este
> archivo ya se quedó desactualizado una vez citándolos. Estado a día de hoy:
> **APPS1/APPS2 ya calibrados** en banco (2026-06-22) y **`BrakeArmRaw` calibrado en coche**
> (2026-06-27); **`BrakePressedRaw` (recorrido total de freno) y `BrakeDvHardRaw` (R2D driverless) siguen
> `COMMISSION`** — sin calibrar. Ver [`docs/commissioning.md`](docs/commissioning.md).

### Calibración de pedales en RUNTIME (#169)

Los siete valores de pedales (`Apps1/2AdcMin/Max`, `BrakeRest/Arm/DvHard/PressedRaw`)
**ya no son constexpr**: viven en `ecu::PedalCal` y se cargan al arrancar desde el
sector NVM del bootloader (sector 7, `0x080E0000` — **fuera de la región de
aplicación**, así que sobreviven a un reflash). Los valores de `ecu_config.hpp`
son sólo los DEFAULTS.

- Se pasan al núcleo puro por `CtrlInputs::cal`, no por un global, para que
  `Controller::step()` siga siendo función pura de sus argumentos y el SIL pueda
  variarlos por test.
- `validate_cal()` (puro, en el SIL) gatea **tanto** el commit por CAN **como** la
  carga desde flash: un registro que no se podría haber comiteado tampoco se
  acepta desde almacenamiento.
- Registro ausente, roto, de versión desconocida o inválido → **defaults**, y se
  anuncia en `0x704` `cal_status` (**ungated**). Una calibración ignorada en
  silencio es indistinguible de una aplicada; eso es lo que evita ese campo.
- Escritura **sólo append**, nunca borrado ni compactación: programar una flash
  word son ~100 µs (0,02 % del presupuesto IWDG de 500 ms); un borrado de sector
  serían cientos de ms. Si el sector se llena se **rechaza** — compactar es
  trabajo del bootloader al arrancar. No implementar la compactación es también
  lo que garantiza que no podemos corromper las claves del propio BL.
- Un commit válido se aplica **en caliente** (apply-on-commit); es seguro porque
  la sesión sólo abre con el coche quieto (TS abajo, fuera de `Active`, par 0,
  rpm 0).
- ⚠️ Las constantes del bootloader están **duplicadas** en `pedal_cal_nvm.hpp`
  (no hay header compartido). Ver el aviso en ese fichero y diffear contra
  `bl_memmap.h` / `bl_nvm.h` en cualquier subida del bootloader.

**Procedimiento:** leer `apps1_raw` / `apps2_raw` / `brake_raw` del stream pit-diag `0x701`
(o por SWD) en reposo y a fondo. El umbral de armado de freno debe ser ≈10 % del recorrido.
`pct = clamp((raw - min) * 100 / (max - min), 0, 100)`. Procedimiento completo, y los
**stubs de banco** (`StubBrakeRaw` / `StubStart` / `StubNoAms` / `StubNoInverter` /
`StubTelemetryDummy` / `TorqueCap`), en [`docs/commissioning.md`](docs/commissioning.md).

---

## Arranque y layout de flash (fix #48 — handoff del bootloader)

El stm32-can-bootloader entrega la app con **IRQs enmascaradas** (`__disable_irq`) y SysTick
parado (su `Bootloader_JumpToApplication`, su issue #59). La app debe rearmar todo:

1. **`main.c` USER CODE BEGIN 1** — lo PRIMERO: limpiar SysTick pendiente y `__enable_irq()`.
   Sin esto `HAL_GetTick()` queda congelado, las esperas de HAL giran para siempre y el IWDG
   heredado resetea antes del init (era el crash de boot intermitente — confirmado por SWD).
2. **SysInit** — refresca el IWDG heredado y fuerza-resetea el periférico FDCAN para
   configurarlo desde un estado limpio (la BL lo deja arrancado).
3. **USER CODE 2** — habilita el reloj del RTC (`__HAL_RCC_RTC_ENABLE`) para que `BKPxR`
   (fault latch + boot magic) no haga bus-fault tras un reset de backup domain.

**Flash** (`STM32H733ZGTX_FLASH.ld`): app en `0x08020000` (768K); registro firmware-info
fijado en `0x08020400`. Node id de la ECU = `0x01` (multi-nodo BL: ECU 0x01 / AMS 0x02 /
uDV 0x03).

---

## Cómo compilar y correr SIL

```bash
# SIL host (núcleo de control puro). Unity offline → deshabilitar unit tests:
cmake -S . -B build-sil -DBUILD_SIL_TESTS=ON -DBUILD_UNIT_TESTS=OFF
cmake --build build-sil
ctest --test-dir build-sil --output-on-failure        # o:
./build-sil/tests/sil/ecu09_sil --test-all
```

El target SIL `ecu09_sil` define `SIL_BUILD=1` y compila **17 unidades**:
`sil_control_tests.cpp` + `Core/Src/app/{control,as_buzzer,cell_derate,discharge,motor_thermal,
pack_thermal,pedal_cal,power_limit,pedal_cal_nvm,cal_session,inverter,udv_tx,
radio_snapshot,vehicle_service,gps_nmea,gps_tx}.cpp`. **Sin HAL, sin FreeRTOS, sin mocks** — esos ficheros no incluyen nada
del HAL y `Controller::step()` recibe `now_ms` como argumento, así que el test es
determinista. No cubre: capa de tareas, HAL/periféricos, `io_signals`, `pit_diag` (#9) ni el
transporte nRF24 — eso es banco / HIL.

`-DBUILD_UNIT_TESTS=OFF` **no es opcional**: `tests/unit/` sigue listando
`Core/Src/{can,control,telemetry,app_state}.c`, borrados en el rewrite a C++, y además
descarga Unity por red. No compila.

El firmware ARM se compila desde `firmware/CMakeLists.txt`, que hace **`file(GLOB)`** de
`Core/Src/*.c` y `Core/Src/app/*.cpp` (sólo hay que re-lanzar el configure si un regen añade
ficheros).

---

## Archivos clave

| Archivo | Qué hace |
|---------|----------|
| `Core/Src/app/control.cpp` · `Core/Inc/app/control.hpp` | Núcleo `ecu::Controller`: FSM + torque + plausibilidad. Sin HAL. |
| `Core/Inc/app/ecu_config.hpp` | **Todas** las constantes tunables. Calibración `COMMISSION` aquí. |
| `Core/Inc/app/cell_derate.hpp` · `Core/Src/app/cell_derate.cpp` | Cap por celda baja sobre **OCV estimado** (compensación IR). Puro, en el SIL. Ver [`docs/derating.md`](docs/derating.md). |
| `Core/Inc/app/motor_thermal.hpp` · `.cpp` | Cap térmico de motor (EMRAX 228). Puro. |
| `Core/Inc/app/pack_thermal.hpp` · `.cpp` | Cap térmico de acumulador (por módulo, con `module_online_mask`). Puro. |
| `Core/Inc/app/power_limit.hpp` · `.cpp` | Envolvente EV 2.2.1. Feed-forward, entero. Puro. |
| `Core/Inc/app/derate_ramp.hpp` | La rampa descendente que comparten los dos caps térmicos. |
| `Core/Inc/app/as_buzzer.hpp` · `Core/Src/app/as_buzzer.cpp` | Zumbador de AS Emergency sobre el RTDS (0x50A). Flanco + ventana de 10 s + onda de 150 ms. Puro, en el SIL. |
| `Core/Inc/app/discharge.hpp` · `Core/Src/app/discharge.cpp` | Descarga del DC-link mantenida por la ECU (#198). Latch sobre `0x021`, suelta con **medida propia**. Puro. |
| `Core/Src/app/control_task.cpp` | Tarea realtime 10 ms. `0x100` en todo estado + único kicker IWDG. |
| `Core/Src/app/can_rx_task.cpp` | Drena RX, despacha, único escritor de VehicleService. |
| `Core/Src/app/can_tx_task.cpp` | Único punto TX del FDCAN; selecciona bus por `frame.bus`. |
| `Core/Src/app/diag_task.cpp` | `0x704` health cada 1 s, separado de control. |
| `Core/Src/app/app_init_task.cpp` | Bring-up FDCAN1/2 + arranque de FDCAN3, one-shot (fix #48). |
| `Core/Inc/app/pedal_cal.hpp` · `Core/Src/app/pedal_cal.cpp` | `PedalCal` runtime + `validate_cal()` + `brake_pct()`. Puro, en el SIL. |
| `Core/Src/app/pedal_cal_nvm.cpp` | Parseo del registro en el sector NVM del BL + helpers de escritura (slot, seq, entry). Puro. |
| `Core/Src/app/pedal_cal_flash.cpp` | La única parte con HAL: programa una flash word. Fuera del SIL. |
| `Core/Src/app/cal_session.cpp` | Máquina de estados de la sesión de calibración (0x7E2). Pura, en el SIL. |
| `Core/Src/app/io_signals.cpp` | ADC3 (freno, APPS1/2) + GPIO (botón, RTDS, LEDs). |
| `Core/Src/app/inverter.cpp` | Adaptador inversor NX/EMC: setpoints 0x360/0x362, decode 0x461/63/66. |
| `Core/Src/app/vehicle_service.cpp` | Estado RX compartido (snapshot por control). |
| `Core/Src/app/telemetry_task.cpp` | 200 ms: dashboard por FDCAN3 (18 tramas) + snapshot de radio nRF24. |
| `Core/Src/app/radio_snapshot.cpp` · `Core/Src/nrf24.c` | Serializado de 102 B + fragmentación · driver nRF24 (**SPI bit-bang** PA5/6/7, ver aviso abajo). |
| `Core/Src/app/udv_tx.cpp` | Builders del contrato uDV (0x504/0x505/0x506/0x511). |
| `Core/Src/app/gps_nmea.cpp` · `Core/Inc/app/gps_nmea.hpp` | Parser NMEA (RMC/GGA) del MTK3339. **Puro, sin HAL, sólo enteros** → SIL. Portado del banco `IFS08_PRIVATE/GPS_TEST`. |
| `Core/Src/app/gps_task.cpp` · `gps_service.cpp` · `gps_tx.cpp` | Tarea GPS (ISR USART10 → anillo SPSC → parser), snapshot compartido y builders `0x508`/`0x509`. |
| `Core/Src/app/{bootloader,error_latch,reset_cause,firmware_info,pit_diag,watchdog}.cpp` | Trigger BL (0x002), latch de fault en BKPxR, reset cause, fwinfo (0x703), builders pit-diag, helpers IWDG. |
| `Core/Inc/can/messages/*.def` + `all_messages.inc` | DSL CAN (fuente de verdad). |
| `Core/Src/app/app_globals.cpp` · `Core/Inc/app/app_globals.h` | Espejos `g_last_*` que la telemetría y el dashboard leen sin tocar el core. Incluye el anuncio de stubs de `0x704`. |
| `Core/Src/app/can_isr.cpp` | ISR RX de los tres FDCAN → `can_rx_queue`. Único productor. |
| `Core/Src/{main,fdcan,freertos}.c` | Handoff BL (#48), MX FDCAN + offset 387w, attrs de tareas. |
| `docs/dbc/ecu.dbc` | DBC generado del DSL (dbcinator lo mantiene). |
| `docs/commissioning.md` | **Runbook operativo**: calibración APPS/freno + stubs de banco. Léelo antes de tocar el coche. |
| `docs/CAN3_MAP.md` · `docs/RADIO_SNAPSHOT_MAP.md` | Contrato del dashboard (FDCAN3) · contrato de radio (nRF24, 102 B). |
| `docs/PINES_RUTEADOS_IOC.md` | Pines ruteados (⚠️ PA5/6/7 aparecen como SPI1; en realidad los usa el nRF24 por bit-bang). |

> ⚠️ **nRF24 = SPI bit-bang, no SPI1.** El SPI1 hardware lee MISO clavado a 0xFF en esta
> placa, así que `nrf24.c` mueve PA5/PA6/PA7 como GPIO. `MX_SPI1_Init()` **sigue
> ejecutándose** en `main.c` antes del kernel; la radio funciona sólo porque
> `NRF24_BusInit()` reclama los pines después. No "arregles" esto pasando el driver a
> `hspi1` — ya se intentó y la radio queda muda.

> **Histórico — NO describen el `dev` actual:** los documentos archivados en
> [`docs/historical/`](docs/historical/README.md) describen el firmware en C anterior al
> rewrite (citan `control.c` / `can.c` / `app_state.c`, que ya no existen). Ese README los
> enumera uno a uno y da la redirección al documento vigente de cada uno. Para el contrato
> CAN actual: `Core/Inc/can/messages/*.def` → [`docs/dbc/ecu.dbc`](docs/dbc/ecu.dbc).
