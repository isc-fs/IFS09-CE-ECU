# Tests de Integración ECU08 NSIL — Explicación completa

> ⚠️ **HISTÓRICO (23-feb-2026) — describe una suite que ya NO existe.** Este documento
> explica el **antiguo harness SIL con RTOS mockeado** (`sil_main.c`, `sil_can_simulator.c`,
> `tests/sil/mocks/`, `g_in`, `cmsis_os2_impl.c`). Esos ficheros siguen en el árbol pero
> **no los compila ningún CMakeLists** — son inertes.
>
> La SIL actual es `tests/sil/sil_control_tests.cpp` → target `ecu08_sil`: 6 unidades de
> traducción, **sin mocks, sin FreeRTOS, sin HAL**. Ver la sección *"Cómo compilar y correr
> SIL"* de [`../CLAUDE.md`](../../CLAUDE.md).
>
> ⚠️ Además, los valores que cita están desfasados: usa `s_freno > 3000` como umbral de
> armado de R2D cuando hoy es `BrakeArmRaw = 750`, y su checklist de HIL (§8) manda trabajar
> contra USART10 y FDCAN3 de una forma que ya no aplica. Se conserva sólo por historia.

> **Resultado última ejecución:** 93/93 tests PASS · 10/10 suites OK · Modo SIL · 23-feb-2026
> *(de la suite pre-rewrite, ya eliminada — no es evidencia del firmware actual)*

---

## 1. ¿Son tests unitarios o de integración?

**Son tests de integración, no unitarios.**

| Característica | Test unitario | Estos tests |
|---|---|---|
| Módulo bajo test | 1 función aislada | Varios módulos encadenados |
| Dependencias | Mocks de todo | APIs reales de RTOS y CAN |
| Estado compartido | Ninguno | `g_in`, colas CAN, mutex, FSM |
| FreeRTOS activo | No | **Sí** (mutex, colas, `osDelay`) |
| Secuencias de arranque | No | **Sí** (S3 recorre 6 estados) |
| Pipeline sensor→control→CAN | No | **Sí** (S8 lo ejecuta completo) |

Cada suite llama a **múltiples módulos reales** en cadena:  
`CanRx_ParseAndUpdate` → `AppState_Snapshot` → `Control_Step10ms` / `Control_ComputeTorque` → `CAN_Pack16` / `Telemetry_Build32`.

---

## 2. Infraestructura FreeRTOS usada por los tests

Los tests usan directamente la API de CMSIS-RTOS v2, la misma que usa el firmware:

```
osMessageQueuePut / osMessageQueueGet   → colas canRxQueueHandle / canTxQueueHandle
osMutexAcquire / osMutexRelease         → g_inMutex (protege g_in)
osDelay(ms)                             → avanza el tick (real en HIL, simulado en SIL)
osKernelGetTickCount()                  → medición de tiempo de ejecución (S6.7, S10.4)
```

En **modo SIL** (PC) estas llamadas van a `tests/sil/mocks/cmsis_os2_impl.c`, que implementa:
- Colas con ring-buffer en heap
- Mutex no-op (proceso single-thread)
- Tick simulado que avanza con cada `osDelay`

En **modo HIL** (STM32H733) son las llamadas reales de FreeRTOS.

---

## 3. Lógica de control — `control.c`

### 3.1 FSM de arranque (6 estados)

`Control_Step10ms` implementa una máquina de estados cooperativa que impide enviar torque hasta que el vehículo haya completado la secuencia de arranque de la normativa EV:

```
BOOT
  │  (primera llamada)
  ▼
WAIT_PRECHARGE_ACK  ←── espera ok_precarga = 1  (ACK de la ACU por CAN 0x020)
  │
  ▼
WAIT_START_BRAKE    ←── espera boton_arranque=1 AND s_freno > 3000
  │
  ▼
R2D_DELAY           ←── espera 2000 ms (Ready-to-Drive delay, normativa EV)
  │
  ▼
READY               ←── estado transitorio (1 ciclo)
  │
  ▼
RUN                 ←── ÚNICO estado que genera tramas CAN y asigna torque_pct
```

> **Bug corregido en esta sesión:** en la versión anterior `out->torque_pct = torque` se
> asignaba *antes* del switch, enviando torque incluso en estado BOOT.  
> Ahora la asignación está **dentro** del case `CTRL_ST_RUN`.

### 3.2 Cálculo de torque — `Control_ComputeTorque`

Función independiente del estado de la FSM. Recibe `app_inputs_t` y devuelve un porcentaje 0-100:

```
s1_pct = (s1_aceleracion − 2050) / 9.0      [raw ADC 12 bits, offset calibrado]
s2_pct = (s2_aceleracion − 1915) / 6.55

    ADC raw    s1_pct   s2_pct
    ─────────  ──────   ──────
     2050       0 %      —
     1915        —       0 %
     2275      25 %      —
     2079        —      25 %
     2500      50 %      —
     2243        —      50 %
     2950     100 %      —
     2570        —     100 %

torque = promedio(s1_pct, s2_pct)   si ambos > 8 %
torque = 0                          si alguno ≤ 8 %  (zona muerta)
torque = 0    si torque < 10        (hysteresis baja)
torque = 100  si torque > 90        (clamp alto)
```

### 3.3 Regla de seguridad EV2.3 (latch freno+acelerador)

```
SI  s_freno > 3000  AND  torque > 25 %
    → lat_ev23 = 1   (LATCH activado → torque = 0)

lat_ev23 se libera SOLO cuando:
    s_freno < 3000  AND  torque < 5 %   (ambas condiciones simultáneas)
```

El flag `lat_ev23` es `static` dentro de `Control_ComputeTorque`. Por eso cada suite que lo usa llama primero a `Control_Init()` para resetearlo.

---

## 4. Las 10 suites — qué prueban

### S1 — Mutex / AppState (9 tests)

Verifica que la sincronización del estado compartido es correcta.

| Test | Qué comprueba |
|---|---|
| `1.1_mutex_handle_valid` | `g_inMutex` existe antes de ejecutar ningún test |
| `1.2_init_*` | `AppState_Init()` pone todos los campos a cero |
| `1.3_torque_valid_range` | El snapshot devuelve `torque_total` entre 0 y 100 |
| `1.4_snap_consistency` | Tres snapshots consecutivos con `osDelay(2)` devuelven el mismo valor |
| `1.5_write_read_*` | Escritura bajo mutex → lectura coherente por `AppState_Snapshot` |

**Relación con FreeRTOS:** usa `osMutexAcquire/Release` y `osDelay` reales.

---

### S2 — FSM Inversor BAMOCAR (7 tests)

Verifica que los mensajes CAN del inversor se parsean y se reflejan en `inv_state`.

| Test | ID CAN | Estado inversor esperado |
|---|---|---|
| `2.1_inv_state_initial_zero` | — | 0 (recién inicializado) |
| `2.2_inv_state_2` | 0x461 | byte0 = 0x02 |
| `2.3_inv_state_4` | 0x463 | byte0 = 0x04 |
| `2.4_inv_state_5_ready` | 0x464 | byte0 = 0x05 |
| `2.5_inv_state_6_run` | 0x465 | byte0 = 0x06 |
| `2.6_inv_state_7_fault` | 0x466 | byte0 = 0x07 |
| `2.7_dc_bus_voltage_400V` | `TINT_ID_DC_BUS_V` | little-endian 0x0190 = 400 V |

Cada test inyecta un mensaje a `CanRx_ParseAndUpdate` bajo mutex y verifica el resultado en `AppState_Snapshot`.

---

### S3 — Secuencia de arranque (9 tests)

Recorre la FSM de arranque completa de principio a fin, verificando que **no se envía torque** en ningún estado anterior a RUN.

```
BOOT         → S3.1: torque_pct=0, count=0
    ↓ ACK precarga (CAN 0x020)
WAIT_PRECHRG → S3.2: ok_precarga=1 en el snapshot
    ↓ llamada sin botón
WAIT_START_B → S3.3: torque_pct=0
    ↓ botón + freno ON
R2D_DELAY    → S3.3b: torque_pct=0
    ↓ osDelay(2100ms)
READY        → 1ª llamada: transición silenciosa
    ↓
RUN          → S3.4: count ∈ [1,8], torque_pct ∈ [0,100]
             → S3.5: msgs[0].id=0x181, msgs[0].bus=CAN_BUS_INV
```

**Por qué son necesarias 3 llamadas para pasar de R2D_DELAY a RUN:**  
`R2D_DELAY→READY` (una llamada), `READY→RUN` (otra llamada transitoria), `RUN` genera la trama (tercera llamada).

---

### S4 — CAN RX parsing (6 tests)

Verifica que `CanRx_ParseAndUpdate` decodifica correctamente cada ID del protocolo:

| Test | ID CAN | Campo actualizado | Valor |
|---|---|---|---|
| `4.1_ack_precarga` | 0x020 | `ok_precarga` | 1 |
| `4.2_s1_acel_parsed` | 0x101 | `s1_aceleracion` | 2500 (50%) |
| `4.3_s2_acel_parsed` | 0x102 | `s2_aceleracion` | 2243 (50%) |
| `4.4_s_freno_parsed` | 0x103 | `s_freno` | 3500 |
| `4.5_v_celda_min_parsed` | 0x12C | `v_celda_min` | 3700 |
| `4.6_unknown_id_no_corruption` | 0xFFFF | (ninguno) | `ok_precarga` sigue siendo 1 |

El último test verifica **robustez ante IDs no reconocidos**: el estado previo no se corrompe.

---

### S5 — CAN TX pack/unpack (23 tests)

Verifica la serialización de mensajes CAN entre la representación `can_msg_t` y el formato de cola `can_qitem16_t`.

| Test group | Qué verifica |
|---|---|
| `5.1_*_roundtrip` | `CAN_Pack16` → `CAN_Unpack16` conserva id, dlc, bus, data[0] |
| `5.2_rx_queue_*` | Cola RX: put → get devuelve los mismos 4 words |
| `5.3_tx_queue_*` | Cola TX: put → get devuelve los mismos 4 words |
| `5.4_fifo_ordering` | 3 mensajes distintos se recuperan **en orden FIFO** |
| `5.5_all_8_bytes_ok` | DLC=8: todos los bytes se preservan tras Pack/Unpack |

Son 23 tests porque `5.4` y `5.5` se ejecutan en bucle (3 y 8 iteraciones).

---

### S6 — Lógica de control / ADC reales (8 tests)

Prueba `Control_ComputeTorque` directamente con valores ADC calibrados del sensor real (no valores arbitrarios).

| Test | s1 raw | s2 raw | Torque esperado |
|---|---|---|---|
| `6.1_zero_throttle_zero_torque` | 2050 (0%) | 1915 (0%) | = 0 |
| `6.2_25pct_throttle` | 2275 (25%) | 2079 (25%) | ∈ [1, 35] |
| `6.3_50pct_throttle` | 2500 (50%) | 2243 (50%) | ∈ [25, 70] |
| `6.4_100pct_throttle` | 2950 (100%) | 2570 (100%) | ∈ [80, 100] |
| `6.5_output_count_valid` | — | — | count ∈ [0, 8] |
| `6.6_compute_torque_range` | 0 / 2950 | 0 / 2570 | ∈ [0, 100] |
| `6.7_step10ms_timing` | — | — | elapsed < 5 ms |

> **S6.1-6.4 usan `Control_ComputeTorque` directamente**, no `Control_Step10ms`.  
> Esto es correcto porque la conversión ADC→torque es independiente del estado de la FSM de arranque.

---

### S7 — Seguridad EV2.3 + APPS (10 tests)

Verifica la regla de seguridad de fórmula eléctrica que prohíbe acelerar mientras se frena.

| Test | Condición | Resultado esperado |
|---|---|---|
| `7.1_normal_throttle_no_inhibit` | 50% acelerador, freno OFF | torque > 0, ev23 = 0 |
| `7.2_brake_accel_torque_zero` | 50% acelerador, freno ON (3500) | torque = 0, ev23 = 1 (latch) |
| `7.3_ev23_latch_persists` | Freno OFF, pero 25% acelerador aún | torque = 0, ev23 = 1 (no libera) |
| `7.4_ev23_latch_released` | Freno OFF + 0% acelerador | ev23 = 0 (liberado) |
| `7.5_full_throttle_no_brake` | 100% acelerador, freno OFF | torque ∈ [80,100], ev23 = 0 |
| `7.6_null_input_safe` | `Control_ComputeTorque(NULL, ...)` | torque = 0 (no crash) |

S7.3 es el test más importante de seguridad: demuestra que **liberar el freno no es suficiente** para desactivar el latch; hace falta que el acelerador también baje por debajo del 5%.

---

### S8 — Pipeline completo sensor→CAN→telemetría (9 tests)

Ejecuta el pipeline completo de una iteración del bucle de control:

```
CAN RX (sensor s1)
    → CanRx_ParseAndUpdate → g_in actualizado bajo mutex
    → AppState_Snapshot → in (copia local)
    → Control_Step10ms → out (torque_pct + tramas CAN)
    → CAN_Pack16 → osMessageQueuePut (cola TX)
    → Telemetry_Build32 → buffer de 32 bytes
```

| Test | Qué verifica |
|---|---|
| `8.2_control_output_valid` | `out.torque_pct` ∈ [0, 100] |
| `8.3_no_frames_no_enqueue` | Si no hay tramas, la cola TX queda vacía |
| `8.4_telemetry_s1_byte6` | Byte 6 del payload = LSB de `s1_aceleracion` |
| `8.5_telemetry_not_garbage` | Los primeros 16 bytes no son todos 0xFF |
| `8.6_multi_cycle` | 5 ciclos consecutivos con `osDelay(10)` sin corrupción |

---

### S9 — Concurrencia FreeRTOS (6 tests)

Verifica que el sistema no tiene race conditions ni deadlocks en escenarios de acceso concurrente.

| Test | Escenario |
|---|---|
| `9.1_rapid_snapshots_no_corruption` | 50 snapshots sin `osDelay` → `torque_total` siempre ≤ 100 |
| `9.2_concurrent_control_steps_valid` | 20 steps con `osDelay(1)` entre ellos → `torque_pct` y `count` válidos |
| `9.3_queue_integrity` | Producir 10 items → consumir todos → integridad del tag 0xAA00 |
| `9.3_queue_count` | Los 10 items se consumen exactamente |
| `9.4_rx_queue_isolated` | Item en RX queue no aparece en TX queue |
| `9.4_tx_queue_isolated` | Item en TX queue no aparece en RX queue |

> En SIL estos tests prueban la implementación del mock de colas.  
> En HIL prueban el FreeRTOS real con las colas del firmware.

---

### S10 — Estrés y casos límite (6 tests)

| Test | Escenario | Criterio |
|---|---|---|
| `10.1_100_cycles_torque_bounded` | 100 ciclos con 6 valores ADC distintos (incl. 0 y 4095) | `torque_pct` siempre ≤ 100 |
| `10.2_queue_bounded_fill` | Intentar meter 140 items en cola de 128 | Éxitos ∈ [64, 128] |
| `10.3_extreme_adc_values_safe` | ADC = 0 y ADC = 4095 (límites de 12 bits) | No cuelga, torque ≤ 100 |
| `10.4_1000ms_loop_no_overflow` | Bucle 1000 ms a 10 ms/ciclo | No overflow, ≥ 50 iteraciones |
| `10.4_loop_count_reasonable` | Mismo bucle | Iteraciones ∈ [50, 200] |
| `10.5_snapshot_null_safe` | `AppState_Snapshot(NULL)` | No crash |

---

## 5. Dónde y cómo se ejecutan

### Modo SIL (Software-In-The-Loop) — en PC

```
freertos.c → StartIntegrationTestTask()
               └─ osDelay(200)
               └─ Test_IntegrationRunAll()
                    └─ [10 suites]
                    └─ Diag_Log → stdout + tests/sil/results/integration_test.log
```

La tarea FreeRTOS en SIL es el hilo `main` del ejecutable. El mutex es no-op y el tick avanza con `osDelay`.

### Modo HIL (Hardware-In-The-Loop) — en STM32H733

```
FreeRTOS scheduler
    ├─ CanRxTask   (recibe mensajes FDCAN1/2/3, escribe g_in bajo mutex)
    ├─ ControlTask (lee snapshot, ejecuta Control_Step10ms, encola TX)
    ├─ CanTxTask   (saca de cola TX, envía por FDCAN)
    └─ IntegrationTestTask  ← Test_IntegrationRunAll()
                                └─ Diag_Log → USART10 @ 115200 baud
```

### Comando de compilación y ejecución SIL

```powershell
$base = "...\ECU08 NSIL"; $sil = "$base\tests\sil"
gcc -DSIL_BUILD=1 -DTEST_MODE_SIL=1 `
    -I"$sil\mocks" -I"$sil" -I"$base\Core\Inc" `
    [fuentes ...] -o "$sil\build\ecu08_sil.exe" -lm

.\build\ecu08_sil.exe --test-integration
```

---

## 6. Resultado de la última ejecución

```
====================================================
  INFORME FINAL - ECU08 NSIL INTEGRATION TESTS
====================================================
  Modo ejecucion : SIL
  Suites totales : 10
  Suites OK      : 10
  Suites FAIL    : 0
  Tests totales  : 93
  Tests pasados  : 93
  Tests fallados : 0
  Tiempo total   : 3324 ms
----------------------------------------------------
  RESULTADO: >>> ALL TESTS PASSED <<< (100%)
  Estado   : SISTEMA LISTO PARA HARDWARE
====================================================
```

---

## 7. Bugs reales encontrados durante la sesión de tests

| # | Bug | Módulo | Efecto | Fix |
|---|---|---|---|---|
| 1 | `out->torque_pct` se asignaba antes del switch de estados | `control.c` | Se enviaba torque en estado BOOT (sin precarga) | Mover asignación solo al case `CTRL_ST_RUN` |
| 2 | `g_inMutex` nunca inicializado en `freertos.c` | `freertos.c` | NULL dereference en la primera adquisición | `g_inMutex = osMutexNew(NULL)` en bloque RTOS_MUTEX |
| 3 | Valores ADC de test arbitrarios (1024, 2048, 4095) | `test_integration.c` | Los tests nunca generaban torque real | Reemplazados por valores calibrados del sensor |
| 4 | `lat_ev23` estático contaminaba suites consecutivas | `test_integration.c` | Falsos negativos en S7 | `Control_Init()` al inicio de cada suite relevante |
| 5 | `Diag_Log` era `__attribute__((weak))` → no-op | `diag.c` (SIL) | Resultados no aparecían en ningún sitio | Creado `mocks/diag_sil.c` con implementación real |

---

## 8. Checklist HIL (Hardware-In-The-Loop) — Cierre de integridad E2E

> Objetivo: validar que lo que pasó en SIL (93/93) también funciona en hardware real con tiempos reales, buses FDCAN reales y carga de colas real.

### 8.1 Preparación de banco

- [ ] MCU STM32H733 conectada por ST-Link
- [ ] Fuente estable y masa común entre nodos CAN
- [ ] Terminación CAN correcta (120Ω en extremos)
- [ ] Transceptor CAN activo en FDCAN1/FDCAN2/FDCAN3
- [ ] UART/USART10 monitorizada a 115200 8N1
- [ ] Firmware compilado y grabado desde STM32CubeIDE
- [ ] Logging habilitado (`Diag_Log`) sin saturación de UART

**Criterio PASS:** sistema inicia sin hardfault y muestra arranque por UART.

---

### 8.2 Prueba de tiempos reales (Control 10 ms + jitter)

#### Procedimiento
1. Instrumentar inicio/fin de `ControlTask` con pin GPIO o marca temporal por UART.
2. Medir período durante 5 minutos.
3. Calcular período medio, jitter pico-pico y peor caso.

#### Umbrales de aceptación
- Período objetivo: $T=10\,ms$
- Error medio permitido: $|\overline{T}-10| \le 0.5\,ms$
- Jitter pico-pico: $\le 1.0\,ms$
- Peor ciclo: $\le 12\,ms$

**Criterio PASS:** todos los umbrales dentro de límite.

---

### 8.3 Prueba de colas bajo carga (Rx/Tx)

#### Procedimiento
1. Inyectar ráfagas CAN simultáneas en FDCAN1/2/3 (al menos 2x tasa nominal).
2. Mantener 120 s de carga continua.
3. Registrar en tiempo real:
   - `osMessageQueueGetCount(canRxQueueHandle)`
   - `osMessageQueueGetCount(canTxQueueHandle)`
   - contador de drops (si existe)
4. Verificar que `CanRxTask` y `CanTxTask` siguen vivos (heartbeat por UART).

#### Umbrales de aceptación
- Overflow de colas: 0 eventos
- Bloqueo de tareas: 0 eventos
- Tramas críticas perdidas (ACK precarga, estado inversor): 0

**Criterio PASS:** no hay pérdida crítica ni bloqueo bajo carga.

---

### 8.4 Prueba funcional FDCAN en hardware

#### Casos mínimos
- [ ] ACK precarga (ID `0x020`) llega y pone `ok_precarga=1`
- [ ] Estados inversor (`0x461..0x466`) actualizan `inv_state`
- [ ] Sensores (`0x101`, `0x102`, `0x103`) se parsean correctamente (little-endian)
- [ ] En estado `RUN` se transmite comando torque por ID `0x181`
- [ ] Antes de `RUN`, torque y tramas de mando se mantienen inhibidos

**Criterio PASS:** secuencia BOOT→RUN correcta y mapeo CAN correcto en ambos sentidos.

---

### 8.5 Prueba de seguridad EV2.3 en hardware

#### Procedimiento
1. Forzar acelerador >25% y freno activo (`s_freno>3000`).
2. Verificar `torque=0` y latch EV2.3 activo.
3. Soltar solo freno y mantener acelerador >5%.
4. Confirmar que el latch persiste.
5. Bajar acelerador <5% con freno suelto.
6. Confirmar liberación del latch.

**Criterio PASS:** comportamiento exacto del latch según regla EV2.3.

---

### 8.6 Prueba E2E prolongada (soak test)

#### Procedimiento
- Ejecutar 30 minutos en lazo cerrado con tráfico CAN nominal.
- Cada 5 min registrar:
  - estado FSM (`BOOT/PRECHARGE/READY/RUN`)
  - latencia Rx→Control→Tx
  - uso de colas Rx/Tx
  - eventos de error HAL/FDCAN

#### Umbrales de aceptación
- Resets espontáneos: 0
- HardFault: 0
- Deadlock: 0
- Desincronización FSM: 0

**Criterio PASS:** operación estable durante toda la ventana.

---

### 8.7 Matriz de cierre HIL

| Área | Métrica | Objetivo | Resultado | Estado |
|---|---|---|---|---|
| Tiempo de control | Período medio | 10 ms ± 0.5 ms | ____ | ☐ PASS / ☐ FAIL |
| Tiempo de control | Jitter p-p | ≤ 1.0 ms | ____ | ☐ PASS / ☐ FAIL |
| Colas RTOS | Overflow Rx/Tx | 0 | ____ | ☐ PASS / ☐ FAIL |
| FDCAN RX | Parseo IDs críticos | 100% | ____ | ☐ PASS / ☐ FAIL |
| FDCAN TX | Comando torque en RUN | 100% | ____ | ☐ PASS / ☐ FAIL |
| Seguridad EV2.3 | Latch/Release correcto | 100% | ____ | ☐ PASS / ☐ FAIL |
| Estabilidad | HardFault/Reset | 0 | ____ | ☐ PASS / ☐ FAIL |

---

### 8.8 Criterio final de liberación

El sistema queda **Apto para pruebas dinámicas de vehículo** solo si:

1. Todas las filas de la matriz HIL están en PASS.
2. Se conserva el resultado SIL 93/93 PASS.
3. No aparecen nuevos warnings críticos en compilación firmware.

Si falla cualquier punto: estado **Condicional** y se bloquea release a pista.
