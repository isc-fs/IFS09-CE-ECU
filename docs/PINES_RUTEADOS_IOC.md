# Mapa de pines ECU08

Mapa de referencia entre:

- pin del conector de la PCB inferior,
- senal funcional en placa,
- pin fisico del STM32,
- y nombre/configuracion observados en `ECU.ioc`.

Este documento no es solo un volcado de CubeMX. Tambien fija la asociacion
funcional que vamos a usar en el proyecto, aunque en el `.ioc` algunos pines
aparezcan con alias genericos como `D1`, `D2`, `A1` o `A2`.

## Fuentes usadas

- `ECU.ioc`
- `Core/Inc/main.h`
- `Core/Src/gpio.c`
- descripcion de ruteo micro -> placa facilitada por el usuario

## Regla de lectura

- La columna "Senal funcional" representa la funcion real en placa.
- La columna "Label CubeMX" refleja el `GPIO_Label` del `.ioc`, cuando existe.
- La columna "Signal `.ioc`" refleja la senal exacta asignada en CubeMX.
- Si una senal funcional no coincide con el nombre de CubeMX, prevalece este
  documento como mapa de asociacion funcional para firmware y migracion.

## Lado izquierdo del conector

| Pin | Senal funcional | Tipo | Pin STM32 | Label CubeMX | Signal `.ioc` | Modo `.ioc` | Estado |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | GND | Alimentacion | - | - | - | - | OK |
| 2 | GND | Alimentacion | - | - | - | - | OK |
| 3 | S_TEMP_REFRI | Funcional | PD5 | - | - | - | ⚠ No aparece en `ECU.ioc` |
| 4 | GPS_TX | Funcional | PG11 | - | `USART10_RX` | `Asynchronous` | OK |
| 5 | GPS_RX | Funcional | PG12 | - | `USART10_TX` | `Asynchronous` | OK |
| 6 | RTDS | Funcional | PB4 | `RTDS` | `GPIO_Output` | - | OK |
| 7 | START_FIL | Funcional | PB5 | `START` | `GPIO_Input` | - | OK |
| 8 | **DC-LINK DISCHARGE** | **EN USO (#198)** | PB6 | `D3` | `GPIO_Output` | - | ⚠ **NO REUTILIZAR** |
| 9 | GPIO4 | Generico | PB7 | `D4` | `GPIO_Output` | - | Libre (candidato a readback auxiliar del relé de descarga, #198) |
| 10 | GPIO5 | Generico | PB8 | `D5` | `GPIO_Output` | - | OK |
| 11 | GPIO6 | Generico | PB9 | `D6` | `GPIO_Output` | - | OK |
| 12 | S_BRAKE_FIL | Funcional | PF7 | `S_BRAKE` | `ADC3_INP3` | `IN3-Single-Ended` | OK |
| 13 | APPS_1 | Funcional | PF8 | `APPS_1` | `SharedAnalog_PF8` = `ADC3_INP7` | - | OK |
| 14 | APPS_2 | Funcional | PF9 | `APPS_2` | `ADC3_INP2` | `IN2-Single-Ended` | OK |
| 15 | GPIO10 | Generico | PF10 | `A4` | `ADC3_INP6` | `IN6-Single-Ended` | Libre **y con ADC** — candidato a sensado propio del DC-link (#198/#177) |
| 16 | GPIO11 | Generico | PC0 | `A5` | `ADCx_INP10` | - | OK |
| 17 | GPIO12 | Generico | PC1 | `A6` | `ADCx_INP11` | - | OK |
| 18 | GPIO13 | Generico | PC2 | - | - | - | No aparece en `.ioc` |

## Lado derecho del conector

| Pin | Senal funcional | Tipo | Pin STM32 | Label CubeMX | Signal `.ioc` | Modo `.ioc` | Estado |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 19 | GND | Alimentacion | - | - | - | - | OK |
| 20 | GND | Alimentacion | - | - | - | - | OK |
| 21 | CAN_L_INV | Funcional | PD0 | - | `FDCAN1_RX` | `FDCAN_Activate` | OK |
| 22 | CAN_H_INV | Funcional | PD1 | - | `FDCAN1_TX` | `FDCAN_Activate` | OK |
| 23 | CAN_H_TEL | Funcional | PB13 | - | `FDCAN2_TX` | `FDCAN_Activate` | OK |
| 24 | CAN_L_TEL | Funcional | PB12 | - | `FDCAN2_RX` | `FDCAN_Activate` | OK |
| 25 | CAN_H_SENS | Funcional | PG9 | - | `FDCAN3_TX` | `FDCAN_Activate` | OK |
| 26 | CAN_L_SENS | Funcional | PG10 | - | `FDCAN3_RX` | `FDCAN_Activate` | OK |
| 27 | NRF_CS | Funcional | PB0 | `NRF24_CS` | `GPIO_Output` | - | OK |
| 28 | NRF24_CE | Funcional | PC5 | `NRF24_CE` | `GPIO_Output` | - | OK |
| 29 | NRF24_IRQ | Funcional | PC4 | `NRF24_IRQ` | `GPIO_Input` | - | OK |
| 30 | SPI1_MOSI | Funcional | PA7 | - | `SPI1_MOSI` | `Full_Duplex_Master` | OK |
| 31 | SPI1_MISO | Funcional | PA6 | - | `SPI1_MISO` | `Full_Duplex_Master` | OK |
| 32 | SPI1_SCK | Funcional | PA5 | - | `SPI1_SCK` | `Full_Duplex_Master` | OK |
| 33 | +3V3 | Alimentacion | - | - | - | - | OK |
| 34 | +3V3 | Alimentacion | - | - | - | - | OK |
| 35 | +5V | Alimentacion | - | - | - | - | OK |
| 36 | +5V | Alimentacion | - | - | - | - | OK |

## Asociaciones funcionales clave

Estas asociaciones son las que conviene usar cuando hablemos de firmware:

- `RTDS` -> `PB4` -> label CubeMX `RTDS`
- `START_FIL` -> `PB5` -> label CubeMX `START` (entrada, no salida)
- `S_BRAKE_FIL` -> `PF7` -> label CubeMX `S_BRAKE`
- `APPS_1` -> `PF8` -> label CubeMX `APPS_1`
- `APPS_2` -> `PF9` -> label CubeMX `APPS_2`
- `GPIO10` -> `PF10` -> label CubeMX `A4`
- `GPIO11` -> `PC0` -> label CubeMX `A5`
- `GPIO12` -> `PC1` -> label CubeMX `A6`

## Hallazgos

### 1. `PC2 / GPIO13` no esta configurado en el `.ioc`

- No aparece en `ECU.ioc`.
- No aparece definido en `Core/Inc/main.h`.
- No aparece inicializado en `Core/Src/gpio.c`.

Implicacion:

- A dia de hoy no hay configuracion firmware asociada a esa linea.

### 2. `PF8 / APPS_1` esta en modo analogico compartido

En el `.ioc`:

- `PF8.Signal = SharedAnalog_PF8`
- `SH.SharedAnalog_PF8.0 = ADC3_INN3`
- `SH.SharedAnalog_PF8.1 = ADC3_INP7,IN7-Single-Ended`

Implicacion:

- El pin si esta asociado a recursos ADC en CubeMX, pero no aparece con un
  `Signal` simple del tipo `ADC3_INPx`.
- Conviene revisar en firmware si la configuracion runtime de ADC realmente
  esta usando ese canal para `APPS_1`.

### 3. Convencion UART

El naming de placa queda asi:

- `GPS_TX` -> `PG11` -> `USART10_RX`
- `GPS_RX` -> `PG12` -> `USART10_TX`

Esto es electricamente coherente y corresponde al cruce habitual TX/RX entre
dispositivos.

### 4. Diferencia entre nombre funcional y nombre CubeMX

En el proyecto actual, CubeMX usa etiquetas genericas:

- `D3..D6` para PB6..PB9 (PB4 y PB5 llevan sus nombres funcionales `RTDS` y `START`)
- `A4..A6` para PF10, PC0, PC1 (PF7/PF8/PF9 llevan `S_BRAKE`, `APPS_1`, `APPS_2`)

Para la migracion y el control conviene pensar en sus nombres funcionales de
placa (`RTDS`, `START_FIL`, `S_BRAKE_FIL`, `APPS_1`, `APPS_2`, etc.), no en
los alias genericos.

## Resumen operativo

El ruteo funcional relevante queda fijado asi:

- `RTDS` en `PB4`
- `START_FIL` en `PB5`
- `S_BRAKE_FIL` en `PF7`
- `APPS_1` en `PF8`
- `APPS_2` en `PF9`
- `CAN_INV` en `PD0/PD1`
- `CAN_TEL` en `PB12/PB13`
- `CAN_SENS` en `PG10/PG9`
- `NRF24` en `PB0`, `PC5`, `PC4`
- `SPI1` en `PA5`, `PA6`, `PA7`

## Pendiente de confirmacion final

- `PC2 / GPIO13`
- ~~uso ADC efectivo de `PF8 / APPS_1` en firmware~~ — **resuelto**: `io_signals.cpp`
  lee `apps1_raw` con `read_adc3(ADC_CHANNEL_7)`, que corresponde a
  `SH.SharedAnalog_PF8.1=ADC3_INP7` en `ECU.ioc`.


---

## ⚠ Pines que NO son genéricos aunque lo parezcan

Esta tabla se lee para buscar un pin libre. Antes de reutilizar cualquiera,
comprobar aquí:

| Pin | Señal | Por qué no se puede tocar |
|---|---|---|
| **PB6** (`D3`) | Interrupción de bobina del relé de descarga del DC-link (#198) | Sale a un **NPN → relé NC en serie con la bobina del relé de descarga**. `HIGH = descargar`. Reutilizarlo o dejarlo flotante rompe la descarga del bus DC: o impide una descarga que debe completarse, o conecta la resistencia de bleed (régimen transitorio) sobre un pack vivo. Lógica en [`Core/Inc/app/discharge.hpp`](../Core/Inc/app/discharge.hpp). **Requiere pull-down externo en la base** (ya presente en la placa nueva): el pin es alta impedancia desde el reset hasta `MX_GPIO_Init`, y el bootloader corre antes. |
| **PA5 / PA6 / PA7** | nRF24 por **bit-bang**, NO SPI1 | El `.ioc` los declara SPI1 y `MX_SPI1_Init()` se ejecuta, pero el driver los mueve como GPIO. El SPI1 hardware lee MISO clavado a 0xFF en esta placa. No "arreglarlo" pasando el driver a `hspi1` — ya se intentó y la radio queda muda. |
| **PB5** (`D2`) | `START_FIL` (botón de arranque) | Gatea el R2D manual. |
| **PB4** (`D1`) | `RTDS` | Zumbador reglamentario. |
