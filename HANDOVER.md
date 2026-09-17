# Handover — IFS09 opening state

Written **2026-09-17**, at the start of the IFS09 season. IFS09 opens from the
**IFS08 season-end baseline (v2.1.1)**. The full IFS08 handover notes — which
describe every open thread, war story, placeholder, and process trap in detail —
live at [`HANDOVER_IFS08.md`](docs/historical/HANDOVER_IFS08.md) (archived).
This document supersedes them for the IFS09 team.

Read the [`README`](README.md) first. Then read the archived IFS08 handover for
context on what you inherited.

---

## 1. Where the code is

- **`dev` is the working branch, `main` is releases.** Both protected; work on a
  branch, arrive by PR with green CI. IFS09 opens at **v0.1.0** (version counter
  reset; IFS08 ended at v2.1.1).
- The firmware **works and has driven the IFS08 car.** The control core is covered
  by a host SIL suite (`--test-all`) that is the regression gate.
- Four torque limits (cell voltage, motor thermal, pack thermal, EV.2.2.1 power)
  are all live; see [`docs/derating.md`](docs/derating.md). DC-link discharge,
  GPS, FOC decode, and the AS-emergency buzzer all landed in IFS08.

---

## 2. Inherited open threads (from IFS08)

These were open at IFS08 season end. Decide which to carry forward.

| # (IFS08) | Title | Status at handover |
|---|---|---|
| **#132** | Priority TX path for safety cyclics | Real, undecided. Measure `0x700.tx_dropped` first before building anything. |
| **#127** | Confirm 0x504 ts_active semantics | Needs a joint sit-down with the DV team. Check whether the uDV migration finished before touching `0x504`. |
| **#212** | HIL discharge interlock end-to-end | Belongs to the HIL repo (`IFS_HIL`), not this one. ECU firmware side is done. |

Open new IFS09 issues for any you pick up, so the IFS09 tracker is the live truth.

---

## 3. War stories — inherited from IFS08

Do not re-learn these.

- **The inverter will not accept `Ready` from `Shutdown` (#148 IFS08).** After a
  tractive-system deactivation the EPowerLabs W90 (A16 config) drops to
  `App_State 13 (Shutdown)` and *ignores* a `Ready(0x04)` command. The fix sends
  the legacy dual word: `Off(0x01)` **then** `Ready(0x04)` in one pass for state 0,
  `Off` alone for state 13. **Do not simplify that block in `control.cpp`.**
- **The FDCAN MessageRAM offsets are not optional and CubeMX keeps deleting them.**
  All three FDCAN instances share one 10 KB SRAMCAN; overlap kills CAN TX with no
  other symptom. `Core/Src/fdcan.c` sets `0 / 387 / 582` by hand. CI
  (`scripts/check_fdcan_ram.py`) fails the build on overlap. After **any** regen, check.
- **The nRF24 radio is bit-banged on purpose.** SPI1 reads MISO stuck-high on this
  board. If telemetry dies, check the bit-bang wiring (PA5/PA6/PA7), not a missing
  peripheral.

---

## 4. Placeholders — wired to zero, not broken

Grep `PLACEHOLDER` in `Core/`. These send valid frames with `0`:

- **State of Charge (`soc`)** — no estimator yet.
- **Inverter speed / current "actual"** (`0x515`/`0x516`, snapshot `[74..81]`) — frames we decode don't carry them.
- **GPS on the dashboard** (`0x519`/`0x51A`/`0x51B`) — GPS driver works; just needs wiring into the dash frames. Low effort, high visible payoff.
- **Inverter E2E** — sent as plain zeros. The inverter does not enforce E2E on RX.

---

## 5. Peer repos (IFS09 equivalents — update as repos are created)

| Peer | Bus | Repo | Contract |
|---|---|---|---|
| **AMS** | FDCAN2 | [`IFS09-CE-AMS`](https://github.com/isc-fs/IFS09-CE-AMS) | `0x100` heartbeat, `0x020` precharge, `0x021` discharge interlock, `0x4A0` status |
| **Driverless (uDV)** | FDCAN2 | [`IFS09-DV-uDV`](https://github.com/isc-fs/IFS09-DV-uDV) *(update if renamed)* | `0x504/0x505/0x506/0x511` out, `0x507/0x510/0x50A` in, `0x508/0x509` GPS |
| **Dashboard** | FDCAN3 | (dash team) | `0x510–0x521`, TX only, [`docs/CAN3_MAP.md`](docs/CAN3_MAP.md) |
| **Ground station** | nRF24 | [`IFS09-TE`](https://github.com/isc-fs/IFS09-TE) *(update if renamed)* | 102-byte radio snapshot, byte-exact fixed offsets |
| **Pit tool** | FDCAN2 | [`MingoCAN`](https://github.com/isc-fs/MingoCAN) | reads `ecu.dbc` |
| **Bootloader** | FDCAN2 | [`stm32-can-bootloader`](https://github.com/isc-fs/stm32-can-bootloader) | app @ `0x08020000`, node id `0x01` |
| **HIL rig** | — | [`IFS_HIL`](https://github.com/isc-fs/IFS_HIL) | shared with IFS08; flag bench items there |

---

## 6. Repository setup (admin checklist)

On the IFS09 GitHub repo, a team admin must configure:

- [ ] Branch protection on `main` (require PR + CI green, no force-push)
- [ ] Branch protection on `dev` (same)
- [ ] Secret `AMS_REPO_TOKEN` — PAT with read access to `isc-fs/IFS09-CE-AMS`
- [ ] Secret `HIL` — fine-grained PAT scoped to `isc-fs/IFS_HIL` with `Actions: read and write`
- [ ] Secret `DBCINATOR_APP_ID` — GitHub App ID for the DBC bot
- [ ] Secret `DBCINATOR_PRIVATE_KEY` — private key for the DBC bot
- [ ] Update `dbc-bot.yml` to point at `isc-fs/IFS09-DBCinator` once it exists

---

## 7. If you have a free afternoon (inherited ranked list)

1. **Wire GPS into the dash frames** — driver works; plumbing missing.
2. **Extend the ground station to show GPS** — bytes are on the air at snapshot `[82..95]`, receiver ignores them.
3. **Measure the safety-cyclic starvation (#132)** — `0x700.tx_dropped` tells you.

---

*The README says it best: the firmware will let you do almost anything, the pack
will not, and when they disagree the pack is right. Go faster than we did.*