# Handover — state of play at season end

Written **2026-08-15**, after the IFS08 season. This is the companion to the
[`README`](README.md): the README tells you how the ECU works and how to work on
it; **this file tells you where it was left, what is half-finished, and the
things that are not written down anywhere else.**

It is a *snapshot*. The issue tracker, `git log`, and the code are the live
truth — where this disagrees with them, they win. Read it once for orientation,
then trust the tracker. Nothing here needs to stay accurate forever; delete a
section once it stops being true.

> Read the [`README`](README.md) first. If you only read one thing, read its
> "How we work" and "Honest gaps" sections — they will save you the two mistakes
> that cost us the most time.

---

## 1. Where the code is

- **`dev` is the working branch, `main` is releases.** Both protected; work on a
  branch, arrive by PR with green CI. Latest release **v2.1.0**.
- The firmware **works and has driven the car.** The control core is covered by
  a host SIL suite (`--test-all`) that is the regression gate — if you change
  behaviour and no test changes, you probably broke the contract silently.
- Four torque limits (cell voltage, motor thermal, pack thermal, EV.2.2.1 power)
  are all live; see [`docs/derating.md`](docs/derating.md). DC-link discharge is
  ECU-held and reports to the AMS. GPS, FOC decode, and the AS-emergency buzzer
  all landed this season.

## 2. Open threads (honest status — the tracker titles lie a little)

| # | Title says | Actually |
|---|---|---|
| **#132** | "Priority TX path for safety cyclics" | **Real, undecided.** The pit-diag/telemetry flood *can* in principle starve the safety cyclics (`0x100`/`0x504`/`0x505`/`0x511`) on a full TX queue. `0x700.tx_dropped` already reports when a frame is lost. Nobody decided between a priority queue and a second FIFO. The right first move is to **measure** whether it actually happens under a realistic bus load before building anything. |
| **#127** | "confirm 0x504 ts_active semantics" | **Mostly stale, needs a joint sit-down with the DV team.** The uDV was moving tractive-system detection off the TSMS pin onto our `0x504` frame. Whether that migration finished is a uDV question — check with them before touching `0x504`. |
| **#212** | "HIL discharge interlock end to end" | **Belongs to the HIL repo** ([`IFS08_HIL`](https://github.com/isc-fs/IFS08_HIL)), not this one. The ECU firmware side of the discharge interlock (#198 et al.) is done; this is about exercising it on the rig against the AMS. Keep ECU firmware and HIL as separate workstreams. |

**#148 (TS-off inverter recovery) is CLOSED and solved** — see §3, it is the most
useful war story in this repo.

## 3. War stories — read these before you "fix" something that looks wrong

The code has comments that look over-explained. They are load-bearing. Three
things were expensive to learn; do not re-learn them.

- **The inverter will not accept `Ready` from `Shutdown` (#148).** After a
  tractive-system deactivation the EPowerLabs W90 (A16 config) drops to
  `App_State 13 (Shutdown)` and *ignores* a `Ready(0x04)` command — it is not
  faulted, it simply will not take that transition. The fix (#168) sends the
  legacy IFS07 **dual word**: `Off(0x01)` **then** `Ready(0x04)` in one pass for
  state 0, `Off` alone for state 13. Bench-proven on stands @355 V. The manual's
  §9.1 diagram says `OFF→READY` is one direct transition; **it is wrong for this
  config** — trust the bench for the App_State numbers, the diagram for topology
  only. `control.cpp`'s `WaitInvStandby` carries the full dated evidence. **Do
  not simplify that block.**
- **The FDCAN MessageRAM offsets are not optional and CubeMX keeps deleting
  them.** All three FDCAN instances share one 10 KB SRAMCAN; if their windows
  overlap they clobber each other and **CAN TX dies** with no other symptom
  (this was defect #48). There is no CubeMX GUI field for it, so every `.ioc`
  regeneration resets them to 0. `Core/Src/fdcan.c` sets `0 / 387 / 582` by hand,
  and CI (`scripts/check_fdcan_ram.py`) fails the build on overlap. After **any**
  regen, check that file.
- **The nRF24 radio is bit-banged on purpose.** The SPI1 peripheral reads MISO
  stuck-high on this board; a software bit-bang on PA5/6/7 reads the radio fine.
  SPI1 was removed. If telemetry silently dies, it is the radio wiring or the
  bit-bang timing, not a missing peripheral.

## 4. Deferred / placeholder — things that are wired to zero, not broken

These send valid frames carrying `0`. A consumer sees a real signal at the right
offset; the *source* just does not exist yet. Grep `PLACEHOLDER` in `Core/`.

- **State of Charge (`soc`)** — no estimator, on this side *or* the AMS side.
  Shows 0 on the radio snapshot `[22]` and dash `0x518`.
- **Inverter speed / current "actual"** (`0x515`/`0x516`, snapshot `[74..81]`) —
  the inverter frames we decode today do not carry them.
- **GPS on the dashboard** (`0x519`/`0x51A`/`0x51B`) — this is the frustrating
  one: the GPS *driver works* (`gps_task.cpp` reads an MTK3339 on USART10,
  `GpsService` already feeds `0x508`/`0x509` and the radio). All that is missing
  is copying `GpsService` into those three dash frames. Low effort, high visible
  payoff. See [`docs/CAN3_MAP.md`](docs/CAN3_MAP.md).
- **Inverter E2E** (`all_messages.inc` `TODO(inverter)`) — the NX/EMC frames are
  AUTOSAR E2E Profile 1 protected (CRC8 + rolling counter). We send them **plain
  zeros** and it works, exactly as the old VCU did — so the inverter evidently
  does not enforce E2E on RX. If you ever see E2E-related rejects, that assumption
  is where to look.

## 5. Who this ECU talks to (and where their code lives)

The safety-relevant contracts are shared with other teams' repos. A wire change
here is a change to their world too — coordinate, don't just ship.

| Peer | Bus | Repo | Contract |
|---|---|---|---|
| **AMS** | FDCAN2 | [`IFS08-CE-AMS`](https://github.com/isc-fs/IFS08-CE-AMS) | `0x100` heartbeat (they open the AIRs if it stops), `0x020` precharge, `0x021` discharge interlock, `0x4A0` status |
| **Driverless (uDV)** | FDCAN2 | [`IFS08-DV-uDV`](https://github.com/isc-fs/IFS08-DV-uDV) | `0x504/0x505/0x506/0x511` out, `0x507/0x510/0x50A` in, `0x508/0x509` GPS |
| **Dashboard** | FDCAN3 | (dash team) | `0x510–0x521`, TX only, [`docs/CAN3_MAP.md`](docs/CAN3_MAP.md) |
| **Ground station** | nRF24 | [`IFS08-TE`](https://github.com/isc-fs/IFS08-TE) | 102-byte radio snapshot, [`docs/RADIO_SNAPSHOT_MAP.md`](docs/RADIO_SNAPSHOT_MAP.md) — **byte-exact**, they decode by fixed offset |
| **Pit tool** | FDCAN2 | [`MingoCAN`](https://github.com/isc-fs/MingoCAN) | reads `ecu.dbc`; arm the pit-diag stream with `0x7E0 = DEADBEEF` |
| **Bootloader** | FDCAN2 | [`stm32-can-bootloader`](https://github.com/isc-fs/stm32-can-bootloader) | app @ `0x08020000`, `0x002` reboot trigger, node id `0x01` |
| **HIL rig** | — | [`IFS08_HIL`](https://github.com/isc-fs/IFS08_HIL) | separate workstream; flag bench items there, don't edit it from here |

The radio snapshot has a hard-won rule: **never renumber a byte offset** — a
field deleted by rules change (e.g. `[15]`, the old EV.2.3 bit) stays as a
reserved zero rather than shifting everything after it, because the ground
station and dash decode by fixed offset and a shift misparses silently.

## 6. Process traps that are not in the code

- **The tracker bot warns that your branch has "no number."** Expected noise —
  we moved to descriptive slugs (`fix/cell-derate-above-ams-trip`). Ignore it.
- **A PR/commit body that says "does not fix #N" will still CLOSE #N on merge.**
  GitHub's closing-keyword handling is negation-blind, and on `main` (the default
  branch) GitHub does it natively — we cannot intercept that. `#148` was
  auto-closed this way three separate times. Our `close-on-dev-merge.yml` guards
  the `dev` path; for `main`, just **don't write a bare "fix #N" after a
  negation** in anything that will reach a release commit.
- **`gh ... --body "..."` mangles backticks and apostrophes** (the shell runs
  backticked text). Use `gh ... --body-file` for anything with `` ` `` or `'`.
- **Bench values leak onto `dev`.** `StubBrakeRaw`/`TorqueCap` get changed for
  on-stands testing and left uncommitted. Before you branch,
  `git checkout -- Core/Inc/app/ecu_config.hpp` if it is dirty. `dev` must always
  ship `TorqueCap=100`, `StubBrakeRaw=0`, every `Stub*=false`. All are announced
  on the ungated `0x704` — check the bus before chasing a car that feels slow.

## 7. If you have a free afternoon

Ranked by payoff-to-effort, all off-car:

1. **Wire GPS into the dash frames** (§4) — the driver already works; this is
   plumbing and the track map lights up.
2. **Extend the ground station to show the GPS it already receives** — see
   [`IFS08-TE`](https://github.com/isc-fs/IFS08-TE); the bytes are on the air at
   snapshot `[82..95]`, the receiver just ignores them.
3. **Measure the safety-cyclic starvation (#132)** before deciding if it needs a
   fix at all. `0x700.tx_dropped` already tells you.

---

*The README says it best: the firmware will let you do almost anything, the pack
will not, and when they disagree the pack is right. Everything in here is a
detail on top of that. Go faster than we did.*
