# Informe técnico — Lógica completa de la ECU (ECU08 NSIL)

> ⚠️ **HISTÓRICO (23-feb-2026) — NO describe el firmware actual.** Este informe documenta
> la app en **C anterior al rewrite a C++17**: `Core/Src/control.c`, `app_state.c`, `g_in` +
> mutex, la FSM de 6 estados `BOOT..RUN`, torque por `0x181` y pedales por CAN
> `0x101-0x103`. **Ninguno de esos ficheros existe ya.**
>
> ⚠️ **El veredicto de validación "SIL: 93/93 PASS" es de aquella suite, ya borrada** — no
> es evidencia válida del firmware que va en el coche. Igualmente, el "riesgo abierto" que
> lista al final (`flag_T11_8_9` como placeholder) **es falso en `dev`**: T.11.8.9 está
> implementado con su ventana de 100 ms y cubierto por SIL.
>
> Fuentes vigentes: [`../CLAUDE.md`](../../CLAUDE.md) (arquitectura + FSM),
> `Core/Src/app/control.cpp` (núcleo real), `Core/Inc/can/messages/*.def` →
> [`dbc/ecu.dbc`](../dbc/ecu.dbc) (contrato CAN). Se conserva sólo por historia.

Fecha: 23-feb-2026  
Estado de validación SIL: 93/93 tests PASS *(de la suite pre-rewrite, ya eliminada)*

---

## 1) Objetivo del sistema

La ECU implementa control de torque para vehículo eléctrico con:

- adquisición de entradas (sensores + CAN),
- lógica de seguridad (EV2.3),
- máquina de estados de arranque,
- generación de comandos CAN al inversor,
- telemetría y diagnóstico.

El sistema está diseñado sobre FreeRTOS (CMSIS-RTOS v2) y separa lógica de aplicación de drivers HAL/FDCAN.

---

## 2) Componentes principales y responsabilidad

## 2.1 Estado global de aplicación

Archivo: `Core/Src/app_state.c` + `Core/Inc/app_state.h`

`g_in` (tipo `app_inputs_t`) contiene el estado compartido:

- entradas de pedal y freno (`s1_aceleracion`, `s2_aceleracion`, `s_freno`, `boton_arranque`),
- feedback del inversor (`inv_state`, `inv_dc_bus_voltage`, temperaturas, rpm),
- batería (`v_celda_min`, `ok_precarga`),
- flags de seguridad (`flag_EV_2_3`, `flag_T11_8_9`),
- derivados (`torque_total`).

Acceso concurrente protegido por `g_inMutex`.

Operaciones:

- `AppState_Init()`: limpia todo el estado.
- `AppState_Snapshot(out)`: copia atómica del estado bajo mutex.

## 2.2 Lógica de control

Archivo: `Core/Src/control.c` + `Core/Inc/control.h`

Funciones clave:

- `Control_Init()`: reinicia la FSM de arranque.
- `Control_ComputeTorque(in, flag_ev, flag_t11)`: calcula torque 0..100%.
- `Control_Step10ms(in, out)`: avanza FSM y genera tramas CAN de control.

## 2.3 Capa CAN

Archivo: `Core/Src/can.c` + `Core/Inc/can.h`

Funciones clave:

- `CAN_Pack16()` / `CAN_Unpack16()`: serialización a item de cola de 16 bytes.
- `CanRx_ParseAndUpdate()`: parsea IDs CAN y actualiza estado.
- `CanTx_SendHal()`: punto único de envío HAL FDCAN.
- `Can_ISR_PushRxFifo0()`: helper ISR para encolar RX.

## 2.4 Telemetría y diagnóstico

Archivos: `Core/Src/telemetry.c`, `Core/Src/diag.c`

- `Telemetry_Build32()`: empaqueta 32 bytes de estado.
- `Telemetry_Send32()`: weak, transporte final dependiente del proyecto.
- `Diag_Report()`: métricas de colas y heap.
- `Diag_Log()`: weak en firmware base (en SIL se sobreescribe por mock).

## 2.5 Orquestación RTOS

Archivo: `Core/Src/freertos.c`

- creación de `g_inMutex`, colas RX/TX,
- creación de tareas:
  - `App_InitTask`,
  - `ControlTask`,
  - `CanRxTask`,
  - `CanTxTask`,
  - `TelemetryTask`,
  - `DiagTask`,
  - `IntegrationTestTask`.

---

## 3) Lógica de control funcional

## 3.1 Cálculo de torque

Calibración de sensores:

- $s1\_pct = \frac{s1\_{raw} - 2050}{9.0}$
- $s2\_pct = \frac{s2\_{raw} - 1915}{6.55}$

Saturación de cada canal a [0, 100].

Regla de activación:

- Si ambos canales > 8%, el torque es el promedio.
- Si no, torque = 0.

Post-procesado:

- Si torque < 10 → 0.
- Si torque > 90 → 100.

## 3.2 Seguridad EV2.3 (latch freno + acelerador)

`lat_ev23` (estático interno) se activa cuando:

- `s_freno > 3000` y `torque > 25`.

Se libera solo cuando:

- `s_freno < 3000` y `torque < 5`.

Mientras `lat_ev23 = 1`, torque forzado a 0.

## 3.3 Máquina de estados de arranque

Estados:

1. `BOOT`
2. `WAIT_PRECHARGE_ACK`
3. `WAIT_START_BRAKE`
4. `R2D_DELAY`
5. `READY`
6. `RUN`

Transiciones clave:

- `BOOT` → `WAIT_PRECHARGE_ACK` si no hay precarga.
- `WAIT_PRECHARGE_ACK` → `WAIT_START_BRAKE` con `ok_precarga=1`.
- `WAIT_START_BRAKE` → `R2D_DELAY` con botón + freno.
- `R2D_DELAY` → `READY` tras 2000 ms.
- `READY` → `RUN` al siguiente ciclo.

Importante: el comando de torque CAN y `out.torque_pct` se publican únicamente en `RUN`.

---

## 4) Mapa lógico CAN

IDs parseados en RX:

- `0x020`: ACK precarga (`ok_precarga`)
- `0x100`: tensión DC bus inversor
- `0x101`: S1 acelerador (little-endian)
- `0x102`: S2 acelerador (little-endian)
- `0x103`: freno (little-endian)
- `0x12C`: tensión mínima de celda
- `0x461`..`0x466`: estado inversor (`inv_state`)

TX control:

- `0x181`: comando de torque al inversor (bus INV)

Buses:

- INV = FDCAN1
- ACU = FDCAN2
- DASH = FDCAN3

---

## 5) Flujo de datos extremo a extremo

1. ISR FDCAN recibe frame y lo mete en cola RX.
2. `CanRxTask` saca cola RX, desempaqueta y parsea.
3. Estado compartido `g_in` refleja entradas nuevas.
4. `ControlTask` toma snapshot de `g_in` cada 10 ms.
5. `Control_Step10ms` calcula salida y genera frames de control.
6. Frames de control se encolan en TX.
7. `CanTxTask` transmite por HAL FDCAN.
8. `TelemetryTask` empaqueta y publica estado para monitorización.

---

## 6) Temporización de tareas (diseño observado)

En la configuración activa de `freertos.c`:

- `ControlTask`: bucle con `osDelay(10)` (100 Hz nominal).
- `CanRxTask`: polling cada 5 ms.
- `CanTxTask`: vaciado de cola y `osDelay(20)`.
- `TelemetryTask`: cada 100 ms.
- `DiagTask`: actividad mínima.
- `IntegrationTestTask`: ejecuta suite y luego se suspende.

Notas:

- Para real-time estricto, `osDelayUntil()` suele ser preferible para control periódico.
- Polling RX/TX puede aumentar latencia bajo carga.

---

## 7) Estado de validación

## 7.1 SIL (software-in-the-loop)

- Build SIL: OK
- Ejecución suite integración: OK
- Resultado: 93/93 PASS, 10/10 suites

Cobertura funcional de integración:

- sincronización/mutex,
- FSM arranque,
- parseo CAN,
- pack/unpack TX,
- lógica torque,
- seguridad EV2.3,
- pipeline completo,
- concurrencia,
- estrés/límites.

## 7.2 HIL (hardware-in-the-loop)

Se añadió checklist HIL en el documento de integración (`INTEGRATION_TESTS_EXPLAINED.md`) para cerrar validación en hardware real: tiempos, colas bajo carga y FDCAN físico.

---

## 8) Riesgos técnicos actuales (priorizados)

1. Riesgo de divergencia de implementación de tareas (`freertos.c` vs `app_tasks.c`) si ambas rutas se mantienen en paralelo.
2. `flag_T11_8_9` está como placeholder (sin plausibilidad completa S1/S2).
3. Telemetría usa `torque_total`; conviene asegurar publicación consistente de derivados en estado global.
4. Polling no bloqueante en RX/TX puede degradar latencia con carga CAN alta.

---

## 9) Conclusión técnica

La lógica de ECU está bien estructurada y funcionalmente validada en SIL con cobertura amplia de integración. El comportamiento crítico de seguridad (EV2.3) y de secuencia de arranque está implementado y probado.

Conclusión operativa:

- **SIL:** apto.
- **HIL:** apto condicional a ejecutar checklist de tiempos reales, carga de colas y validación de FDCAN físico.

Recomendación inmediata: cerrar campaña HIL y consolidar una única ruta de tareas para eliminar ambigüedad operativa.