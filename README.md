![ISC Logo](http://iscracingteam.com/wp-content/uploads/2022/03/Picture5.jpg)

# IFS09 — ECU

Firmware for the **ECU** of the IFS09, the ISC Racing Team's Formula Student
electric car. It reads the driver's pedals, decides how much torque the
car is allowed to make, and talks to everything else on the car over CAN.

**STM32H733ZG · FreeRTOS · C++17.** The image links at `0x08020000` and is
flashed over CAN through [`stm32-can-bootloader`](https://github.com/isc-fs/stm32-can-bootloader)
— there is no SWD in the normal workflow.

---

## Season 9 — Starting point

IFS09 opens from the **IFS08 season-end baseline (v2.1.1)**. The codebase is
a direct migration: all firmware, tests, tooling, and CI/CD pipelines have
been carried over from [`IFS08-CE-ECU`](https://github.com/isc-fs/IFS08-CE-ECU)
unchanged. The control core is proven and has driven the car.

**What this means for you:**

- Everything in [`HANDOVER.md`](HANDOVER.md) describes the state the code was
  in at the end of IFS08. Read it before opening any issue or starting any feature.
- The three open threads from IFS08 (`#132`, `#127`, `#212`) are the natural
  first candidates for IFS09 work.
- Version numbering resets: IFS09 starts at **v0.1.0**. The first release that
  drives the IFS09 car will be v1.0.0.

> **Repository secrets to configure (team admin):** `AMS_REPO_TOKEN`,
> `HIL`, `DBCINATOR_APP_ID`, `DBCINATOR_PRIVATE_KEY`. These are required for
> the cross-repo AMS contract check, HIL test dispatch, and the DBC bot
> respectively. Without them those CI jobs skip or fail loudly — they do not
> silently pass. See the workflow files for details.

---

## Start here

| If you are… | Read |
|---|---|
| **New to the team** | This page, then [`docs/derating.md`](docs/derating.md) — it is short and explains the thing people ask about most. |
| **About to flash a board** | [`docs/flashing.md`](docs/flashing.md) — including the recovery section, *before* you need it. |
| **Bringing the car up** | [`docs/commissioning.md`](docs/commissioning.md) — the on-car runbook. |
| **Changing code** | [`CLAUDE.md`](CLAUDE.md) — the architecture reference and the conventions. |
| **Chasing "why is the car slow?"** | [`docs/derating.md`](docs/derating.md), then the pit-diag frames it names. |
| **Picking this up from IFS08** | [`HANDOVER.md`](HANDOVER.md) — where it was left, what is half-finished, and the traps that are not in the code. |

Anything in [`docs/historical/`](docs/historical/README.md) describes the **old
C firmware** and is kept only so old decisions stay readable. Do not trust it
against the current code.

---

## What the ECU actually does

Pedals in, torque out, with four independent limits on the way:

```
APPS1 + APPS2 ──▶ plausibility (T.11.8.9) ──▶ torque map ──┐
                                                            │
                    ┌───────────────────────────────────────┘
                    ▼
        min( cell voltage cap, motor thermal cap,
             pack thermal cap, EV 2.2.1 power cap )
                    │
                    ▼
             inverter, over FDCAN1
```

Every limiter is a **cap**, never a gain — `torque = min(torque, cap)`. A gain
rescales the whole pedal to bound a peak and costs the driver resolution
everywhere; a cap clips only the top and leaves the rest of the travel alone.
Because they are all caps, the order they are applied in does not matter.

[`docs/derating.md`](docs/derating.md) explains all four and which one to
suspect when the car feels down on power. The thresholds live in
[`Core/Inc/app/ecu_config.hpp`](Core/Inc/app/ecu_config.hpp) and are
deliberately **not** repeated in the docs — a document goes stale the moment it
copies a number.

### The three buses

| Bus | Peer | Traffic |
|---|---|---|
| **FDCAN1** | Inverter (EPowerLabs W90 / EMC150) | torque command out, state and FOC telemetry in |
| **FDCAN2** | AMS, pit tool, driverless uDV | the safety contracts — heartbeat, precharge, discharge interlock |
| **FDCAN3** | Dashboard | TX only |

All three at 500 kbit/s. The wire format is **code-first**: the `.def` files in
`Core/Inc/can/messages/` are the source of truth, and
[`docs/dbc/ecu.dbc`](docs/dbc/ecu.dbc) is generated from them. Never hand-edit
the DBC — CI fails the build if it drifts.

---

## Build and test

```bash
cmake -S . -B build-sil -DBUILD_SIL_TESTS=ON -DBUILD_UNIT_TESTS=OFF
cmake --build build-sil -j8
./build-sil/tests/sil/ecu09_sil --test-all
```

The SIL suite is the regression gate — the control core is pure, host-testable
C++ with no HAL and no RTOS, so it runs anywhere with a compiler. If you change
behaviour, a test should change with it.

```bash
cmake -S firmware -B build-fw && cmake --build build-fw -j8
python3 scripts/check_flash_layout.py build-fw/ECU09.elf
```

> `firmware/CMakeLists.txt` globs its sources **at configure time**. Add a
> `.cpp` and the build will fail to link until you re-run the `cmake -S` step,
> not just `--build`.

The layout check is not optional: an image that grows into sector 0 or sector 7
overwrites the bootloader or its NVM, flashes cleanly, and then never boots.

---

## How we work

`main` and `dev` are both protected. All work happens on a branch off `dev` and
arrives through a pull request with green CI.

```bash
git checkout dev && git pull
git checkout -b fix/short-descriptive-slug
```

Branch names take a **descriptive slug** — `fix/cell-derate-above-ams-trip`,
not `fix/7`. The tracking-issue bot still looks for a legacy leading number and
will warn when it does not find one; that warning is expected noise.

`dev → main` is a **release**, not a routine merge, and is gated on validation.
Full detail in [`docs/REPOSITORY_WORKFLOW.md`](docs/REPOSITORY_WORKFLOW.md).

**Two rules that are easy to get wrong and expensive to miss:**

- **`TorqueCap` must be `100` and every `Stub*` must be `false` in anything
  committed.** They exist for bench work on stands. All five are announced on
  the ungated `0x704`, so check the bus before you chase a car that feels slow.
- **Never hand-edit CubeMX-owned files outside a `USER CODE BEGIN/END` block.**
  A regeneration wipes anything outside them, and it has silently reset the
  FDCAN MessageRAM offsets more than once — which kills CAN TX outright.

---

## Honest gaps

Constants tagged `COMMISSION` in `ecu_config.hpp` have **never been measured on
this car**. They are placeholders with plausible values, not settings.
[`docs/commissioning.md`](docs/commissioning.md) is the procedure for turning
each one into a measurement, and most of them fall out of a single acceleration
run with the pit-diag stream enabled.

This is a convention worth keeping: throughout these docs, what is *measured*,
what is *assumed*, and what is *untested* are stated separately. A number
nobody has checked is far more dangerous when it is written down with
confidence, because the next person has no reason to doubt it. If you measure
one, delete the tag and say where the number came from. If you find a sentence
here that the code contradicts, **the code wins and the sentence is a bug** —
fix it in the same PR.

---

## For whoever picks this up next

You have inherited a car that mostly works and a pile of things nobody got to.
That is the normal state of a Formula Student project, and it is not a
criticism of anyone who came before you — every team hands over mid-sentence.

The firmware will let you do almost anything. The pack will not. When those two
disagree, the pack is right.

> *"For a successful technology, reality must take precedence over public
> relations, for Nature cannot be fooled."*
>
> — Richard P. Feynman, *Report of the Presidential Commission on the Space
> Shuttle Challenger Accident*, Appendix F (1986)

Go faster than we did.

---

*ISC Racing Team — IFS09 Control Electronics*