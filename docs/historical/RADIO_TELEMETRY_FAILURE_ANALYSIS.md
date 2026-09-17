# Radio Telemetry Failure Analysis

> ⚠️ **HISTÓRICO (2026-06-18).** Informe de incidencia sobre ficheros que ya no existen
> (`telemetry.c`, `app_tasks.c`, `build_firmware.ps1`) y sobre el predecesor de
> `StubTelemetryDummy` con su nombre de macro antiguo. Sus enlaces "origen en código"
> apuntan a rutas Windows de otra máquina.
>
> El enlace de radio vigente (nRF24, snapshot v2 de 102 B) está en
> [`RADIO_SNAPSHOT_MAP.md`](../RADIO_SNAPSHOT_MAP.md). Se conserva sólo por historia.

Fecha: 2026-06-18

## Resumen

Durante la validacion de la telemetria entre `IFS08-CE-ECU` y `IFS08-TE-main`, TE recibia tramas con:

- `magic = EC`
- `kind = 1`
- `fragment_count = 3`

Ejemplos observados por UART en STM:

```text
NRF24 TX ok #70 magic=EC frag=0/3 seq=63 kind=1
NRF24 TX ok #80 magic=EC frag=2/3 seq=70 kind=1
NRF24 TX ok #100 magic=EC frag=0/3 seq=90 kind=1
```

Esto no coincide con el contrato actual del firmware fuente revisado en este repositorio.

## Comportamiento esperado segun el codigo actual

En el codigo actual de `IFS08-CE-ECU`:

- `1 = TELEMETRY_FRAME_DASH`
- `3 = TELEMETRY_FRAME_RF_FAST`
- `4 = TELEMETRY_FRAME_RF_SLOW`
- `5 = TELEMETRY_FRAME_RF_EVENT`

Referencia:

- [telemetry.h](../../Core/Inc/telemetry.h)

La telemetria por radio deberia salir asi:

- `RF_FAST` -> `kind=3`, `fragment_count=2`
- `RF_SLOW` -> `kind=4`, `fragment_count=5`
- `RF_EVENT` -> `kind=5`, `fragment_count=1`

Referencias:

- `freertos.c`
- `freertos.c`
- `freertos.c`
- `telemetry.c`

## Problema observado

La placa estaba emitiendo:

- `kind=1`
- `frag=.../3`

Eso implica que el firmware ejecutado en la placa no seguia el comportamiento esperado del codigo actual.

En otras palabras:

- la cabecera parecia nueva (`magic=EC`)
- pero el `kind` y el numero de fragmentos no eran compatibles con la implementacion actual del repositorio

## Analisis realizado

### 1. El parser de TE no era la causa principal

TE estaba recibiendo exactamente lo que salia por radio. El parser no inventaba `kind=1`; simplemente lo estaba leyendo.

Archivo revisado:

- `ISC_RTT_serial.py` (in the separate `IFS08-TE` repo, `ISC_REAL_TIME_25/`)

### 2. El firmware actual del STM no deberia emitir `kind=1` por radio

La ruta principal de telemetria en `freertos.c` construye:

- `TELEMETRY_FRAME_RF_FAST`
- `TELEMETRY_FRAME_RF_SLOW`
- `TELEMETRY_FRAME_RF_EVENT`

No construye `TELEMETRY_FRAME_DASH` para radio.

### 3. Se encontro una segunda implementacion legacy de telemetria

En `app_tasks.c` habia una ruta antigua que hacia:

```c
Telemetry_BuildFrame(&in_snap, NULL, &frame);
```

Como `Telemetry_BuildFrame(..., NULL, ...)` genera `TELEMETRY_FRAME_DASH`, esa ruta podia terminar mandando `kind=1` por radio si se compilaba o ejecutaba por error.

Referencia:

- `telemetry.c`

### 4. La telemetria estaba forzada a datos dummy

Esto no explica `kind=1`, pero si afecta a la validacion funcional:

- `TELEMETRY_DUMMY_SNAPSHOT` estaba activado
- el snapshot real se sobreescribia con datos sinteticos

Referencia:

- `telemetry.c`
- `telemetry.c`

El usuario confirmo que los datos dummy deben mantenerse por ahora.

## Correccion aplicada

Se corrigio `app_tasks.c` para que su ruta legacy use la misma separacion que `freertos.c`:

- `DASH` solo a dashboard
- `RF_FAST` a radio
- `SD` a SD
- `RF_SLOW` cada 5 ciclos a radio

Con esto, aunque esa implementacion secundaria se use, ya no deberia emitir `kind=1` por radio.

Archivo modificado:

- `app_tasks.c`

## Conclusion tecnica

La causa mas probable del fallo observado es una de estas dos:

1. La placa no estaba ejecutando el binario nuevo compilado desde este repositorio.
2. La placa estaba ejecutando una variante antigua/hibrida donde la telemetria de radio seguia usando `kind=1` y `fragment_count=3`.

Despues de corregir `app_tasks.c`, si la UART sigue mostrando:

- `kind=1`
- `frag=.../3`

entonces la conclusion es que el binario cargado en la placa no corresponde al firmware actual generado en:

- [build-fw/ECU08.elf](../../build-fw/ECU08.elf)

## Estado actual

- Ruta de build limpia y estable: [build-fw](../../build-fw)
- Binario actual: [ECU08.elf](../../build-fw/ECU08.elf)
- Script de compilacion estable: `build_firmware.ps1`

## Siguiente paso recomendado

Flashear explicitamente `build-fw/ECU08.elf` y volver a comprobar UART.

Si tras el flasheo siguen apareciendo tramas con `kind=1 frag=0/3`, hay que asumir que:

- no se ha flasheado la placa correcta, o
- la UART observada no pertenece al firmware recien cargado.
