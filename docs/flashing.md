# Flashing and recovery

**If your ECU will not boot, jump to [Recovery — unbricking an ECU](#recovery--unbricking-an-ecu).**

This is the document for getting firmware *onto* the board. [`commissioning.md`](commissioning.md)
is the on-car runbook and assumes you already have a working, flashed ECU.

---

## What you are flashing onto

The ECU links **on top of** [`stm32-can-bootloader`](https://github.com/isc-fs/stm32-can-bootloader).
There is no SWD in the normal workflow — everything goes over CAN.

| region | contents |
|---|---|
| sector 0 | the bootloader. **Not touched by an app flash.** |
| sectors 1–6, from `0x08020000` | the application. This is what you flash. |
| sector 7, `0x080E0000` | bootloader NVM — **and your pedal calibration** (#169) |

Two consequences worth internalising:

- **A reflash does not erase your calibration.** It lives in sector 7, outside the app region.
- **A reflash does not remove the bootloader**, which is why a bad app image is recoverable.

Node IDs on the bus: **ECU `0x01`**, AMS `0x02`, uDV `0x03`.

---

## Build a flight image

```bash
cmake -S firmware -B build-fw && cmake --build build-fw
```

**`cmake -S firmware`, not `cmake -S .`** — the repo-root `CMakeLists.txt` is the *host SIL*
project, not the firmware.

> ⚠️ **`firmware/CMakeLists.txt` globs `Core/Src/*.c` and `Core/Src/app/*.cpp` at CONFIGURE
> time.** If you add a new source file and only run `cmake --build`, it will not be compiled
> and you get a confusing **link** error (`undefined reference to ...`) rather than a missing
> file. Re-run the configure step whenever a file is added.

Then check the layout before you write anything:

```bash
python3 scripts/check_flash_layout.py build-fw/ECU08.elf
```

This is what stops an image that has grown into sector 0 or sector 7 — such an image
flashes cleanly and then never boots.

### ⚠️ Check the AMS branch before you flash

The ECU and the AMS share CAN contracts, and **the two repos can be out of step**.
Right now:

| | `0x100` | `0x021` |
|---|---|---|
| AMS **`dev`** | DLC 3, decodes `discharge_engaged` | present |
| AMS **`main`** | **DLC 2** | **absent** |
| ECU `dev` | sends DLC 3 | decodes it |

An ECU built from `dev` sends a 3-byte `0x100`. An AMS built from **`main`** has
never seen byte 2. Their `dev` decoder handles the short frame deliberately
(`if dlc >= 3`), so the pairing degrades safely in that direction — but the
reverse has not been verified, and neither has `main`'s decoder.

**As of the 2026 season close, the car runs AMS `dev`** — so the pairing is
consistent and byte 2 of `0x100` is actually decoded. Treat that as the last
known state, not as a standing guarantee: the hazard is *release ordering*. When
the AMS merges `dev` into `main`, the ECU has to go with it, or an ECU on `dev`
meets an AMS on `main` and byte 2 silently disappears.

**Before a session, confirm which AMS branch is on the car**, and prefer flashing
both from `dev` or both from `main`. To check the shared frames yourself with
`IFS08-CE-AMS` checked out alongside:

```bash
python3 scripts/check_ams_contract.py
```

It compares every frame both repos touch and fails on any conflicting bit layout.
This exists because the ECU once decoded `0x021` bit 0 as a completely different
signal from the one the AMS was sending there — see the header of that script.

### Before every flight build, confirm the bench values are off

These are **config toggles, not build flags**, so nothing warns you:

| constant | flight value |
|---|---|
| `TorqueCap` | `100` |
| `StubNoAms`, `StubNoInverter`, `StubStart` | `false` |
| `StubBrakeRaw` | `0` |

All five are announced on the **ungated** `0x704` (`stub_no_ams`, `stub_no_inverter`,
`stub_start`, `stub_brake`, `stub_torque_cap`). **A flight build reports all five zero** —
check that on the bus after flashing rather than trusting the source tree, since these are
routinely edited locally and easy to commit by accident.

---

## Flash

PCAN on the **ACU bus (FDCAN2)**. All three buses are classic CAN at **500 kbit/s**.

```bash
arm-none-eabi-objcopy -O binary build-fw/ECU08.elf ECU08.bin
```

Then flash with `can-flasher` / MingoCAN to node `0x01`, app base `0x08020000`.

> On macOS the mac-can `libPCBUSB.dylib` must be in `/usr/local/lib`. The GUI loads it
> in-process; the CLI does not.

> The flasher extension's **default `buildCommand` is wrong for this repo** — build with the
> command above rather than letting the extension do it.

**Never power-cut mid-write.**

### The pre-flight bootloader check

`commissioning.md` tells you three separate times to "confirm BL recovery first" without
saying how. This is how, and it takes ten seconds:

With the **app running**, send the trigger on the ACU bus:

```
ID 0x002, DLC 4, payload  B0 07 AD 12
```

The ECU writes the magic to RTC backup and resets into the bootloader. Confirm the
bootloader announces itself, then reset back.

**If it does not respond, STOP.** You have no recovery path and must not start a write.

---

## Recovery — unbricking an ECU

A **bricked** ECU — bad image, power cut mid-write, app that hard-faults before it reaches
`CanRxTask` — is recoverable, because the bootloader lives in sector 0 and an app flash
never touches it.

> ### ⚠️ Do NOT copy the `send-raw` example from `can-flasher/docs/USAGE.md`
>
> That document shows:
> ```
> can-flasher send-raw 0x002 B0 07 AD 11
> ```
> **`AD 11` is the AMS magic. The ECU's is `AD 12`.**
>
> On our shared ACU bus, `AD 11` reboots the **AMS** into *its* bootloader — which opens
> every relay on the way — and does nothing at all to the ECU. You will drop the pack and
> the ECU will still be bricked.
>
> | node | trigger payload |
> |---|---|
> | **ECU** | `B0 07 AD 12` |
> | AMS | `B0 07 AD 11` |
>
> Source of truth: `Core/Inc/app/ecu_config.hpp` `BlBootTriggerPayload`.

### If the app still runs enough to serve CAN

Send `0x002` / `B0 07 AD 12` as above, then flash normally.

### If the app does not run at all

The bootloader gets control at every reset before the app. Power-cycle the ECU and start
the flash immediately — the bootloader's own CAN window is open before the app is entered,
so the flasher can catch it there. Repeat if you miss the window.

### If that fails too

SWD is the last resort and is **not** part of the normal workflow. The app base is
`0x08020000`; do not erase sector 0 (you would lose the bootloader and with it every
CAN-based recovery) and do not erase sector 7 (you would lose the pedal calibration).

### Diagnosing rather than guessing

`0x704 PitDiag_health` is **ungated** — it streams from `DiagTask` from boot without arming
pit-diag. `reset_cause` on it distinguishes a power-on from an **IWDG** reset, which is the
difference between "it never started" and "it started and then wedged". Check that before
assuming the image is bad.

---

## First five minutes when the car misbehaves

In order, because each rules out a class of problem:

1. **`0x704` (ungated).** Task liveness bits, `reset_cause`, and the five stub bits. If a
   stub bit is set, stop here — you are running a bench image.
2. **`0x700`.** `fsm_state` and `inv_state` side by side. A mismatched pair is the signature
   of most drive problems (see the FSM invariants in [`../CLAUDE.md`](../CLAUDE.md)).
3. **Arm pit-diag** (`0x7E0` = `DE AD BE EF`) and read the limiter frames — see
   [`derating.md`](derating.md) for which cap is which and what each flag means.
4. **`0x708`** if the inverter will not come up: L1/L2 fault layers plus `inv_redrive_count`.

Disable pit-diag with `0x7E0` = `0`.
