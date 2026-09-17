# Torque limiting: the four caps

Every limiter in this ECU is a **`min()` ceiling** on commanded torque. This document
explains how they compose and which one to suspect when the car is slow. The thresholds
themselves live in [`Core/Inc/app/ecu_config.hpp`](../Core/Inc/app/ecu_config.hpp) and are
**not repeated here** — this file goes stale the moment it copies a number.

The reasoning behind each individual cap is in its own header, at length. Start here to
know *which* header to open.

## The chain

```
pedals -> deadband -> T.11.8.9 cut -> DV source override
       -> cell voltage cap        cell_derate.hpp
       -> motor thermal cap       motor_thermal.hpp
       -> pack thermal cap        pack_thermal.hpp
       -> EV 2.2.1 power envelope power_limit.hpp
       -> TorqueCap (bench stub, control_task.cpp)
       -> Nm, NEGATED, onto 0x362 inverter.cpp
```

**Order is immaterial** among the four caps: they are all ceilings, so the lowest wins
whatever sequence they run in. That was *not* true until #177 — the cell derate used to
multiply demand, and a gain has to come first or it scales the caps themselves. Do not
reintroduce a multiplier.

## Why caps and not gains

A gain rescales the whole pedal: at a 68 % factor a 30 % request becomes 20 %, even though
30 % was never the problem. A cap leaves everything below the ceiling untouched, so the
driver keeps full resolution over the range still allowed. This distinction cost a rewrite
(#177) — it is the single most important idea in this subsystem.

## Which cap is which

| cap | driven by | annunciated on | header |
|---|---|---|---|
| Cell voltage | AMS `0x12C` min cell, **IR-compensated to OCV** using `0x135` current | `0x709` `capped` | `cell_derate.hpp` |
| Motor thermal | inverter `0x464` motor temps (EMRAX 228) | `0x706` `thermal_capped` | `motor_thermal.hpp` |
| Pack thermal | AMS `0x136`/`0x137` per-module maxima | `0x70A` `pack_capped` | `pack_thermal.hpp` |
| Power envelope | `0x463` rpm, feed-forward | `0x700` `power_capped` | `power_limit.hpp` |
| `TorqueCap` | nothing — a bench stub (a config value, applied at runtime) | `0x704` `stub_torque_cap` (**ungated**) | — |

> ⚠️ **The `capped` flags do not all mean the same thing.** `power_capped` is set
> only when the envelope *actually clipped* the request. The other three mean
> only that the ceiling is below 100 %, whether or not it bound — at 60 % pedal
> with a 70 % motor cap, `thermal_capped` reads 1 and is costing you nothing.
>
> **To find the binding cap:** read `torque_pct` on `0x700`, then compare it
> against the published ceilings — `cap_pct` (`0x709`), `thermal_cap_pct`
> (`0x706`), `pack_cap_pct` (`0x70A`). The ceiling equal to `torque_pct` is the
> one limiting you. The power envelope publishes no percentage, so if none of
> the three matches, it is the envelope: check `power_capped` on `0x700`.

**Check `0x704` first.** `TorqueCap` is applied *after* the pure core, so it trips none of
the `capped` flags — a bench image looks exactly like a derate that no derate explains.

## Three ideas that recur in all four

**1. A cap must never RELAX when a sensor is lost.** Losing sight of the pack must not hand
the car more torque than it had while the pack was visible. Both thermal caps ratchet down
only. This was a real bug (a hot pack jumped 78 Nm → 139 Nm when its frames went quiet).

**2. Per-signal freshness, never bus-level.** `last_ams_tick` and `last_inv_tick` are
stamped by *every* frame on their bus, so they cannot tell you that one frame died — and a
frozen reading always looks healthy. Five separate bugs of this exact shape have been fixed
(`0x135`, `0x464`, `0x136`/`0x137`, the FOC frames, `0x463`). **If you add a consumer of a
CAN signal, give it its own timestamp.**

**3. A floor must command real torque.** The percentage scale is *not* linear to 240 Nm —
`InvTorqueMap*` is re-based so `DeadbandLowPct` maps to exactly 0 Nm. A floor written as
"5 %" commanded *nothing*, while reading as a limp-home. Assert on the **Nm**, not the
percent. `ecu_config.hpp` asserts the floor sits above `DeadbandLowPct`; the
Nm assertions themselves live in `tests/sil/sil_control_tests.cpp`.

## Constants that were never measured

Several thresholds are placeholders. `ecu_config.hpp` tags each one and gives the procedure;
the ones whose absence changes behaviour most:

- **`CellIrMilliOhm`** — one acceleration run, plot `est_ocv_mV` against `raw_mV` on `0x709`. Flat is correct.
- **`DrivetrainEffPct`** — the *entire* EV 2.2.1 compliance margin. `0x70D` puts commanded shaft power next to the inverter's own AC measurement and the DC bus, so one capture at steady full throttle gives the real number.
- **`AmsCellUnderVoltageMv`** — mirrored from the AMS, and the cell thresholds are
  derived from it. See below.
- **`PackTempLimitDegC`** — note the AMS gates its own cell-temperature faults behind
  `TempFaultsTrusted`, which is `false` on a flight build, so **this ECU cap is
  currently the only pack over-temperature response in the car**. It is not a soft
  pre-limit under a real one.

## ⚠️ The AMS trip point is fixed, and the derate is derived from it

The AMS opens the AIRs when the **raw loaded** minimum cell falls below its
`CellUnderVoltageMv`. That value is theirs to set, not ours — but it is **not
fixed**: it is still tagged `COMMISSION` in their config, so it will move when
they measure it, and every threshold derived from it moves with it.

This ECU derates on **IR-compensated OCV**, which under load is always *higher*
than the loaded reading they trip on. So any threshold at or below their trip
**can never engage** — the AIRs open while the ECU is still commanding 100 %.

That was live: knee and floor were 2800/2500 against a 2800 mV trip, so the
entire ramp sat underneath it and the derate was decorative. At 230 A and 1 mΩ a
loaded 2799 mV reads as 3029 mV here.

So the thresholds are **derived, not chosen** (`ecu_config.hpp`):

```
floor = AMS trip + margin
knee  = floor + I_peak x CellIrMilliOhm
```

giving the invariant worth remembering: **at the knee, full torque pulls the
loaded cell down to exactly the floor.** Below the knee the derate cuts torque,
which cuts current, which is what actually protects the loaded voltage.

Deriving it also means **commissioning `CellIrMilliOhm` moves the knee
automatically**. Hand-picked numbers would silently stop being correct the moment
that value changed — which is precisely how the first version broke.

`static_assert`s enforce that the floor and the raw backstop both sit above the
AMS trip. Nothing complained about the old values because nothing checked the
relationship.

**`IFS08-CE-AMS/Core/Inc/app/ams_config.hpp` is a sibling checkout — read it, and
run `scripts/check_ams_contract.py` before any PR that touches a shared frame.**

⚠️ That script compares **CAN frame layout only** — DLC, start bit, length, byte
order, signedness. It does **not** compare `AmsCellUnderVoltageMv` against the
AMS's `CellUnderVoltageMv`. The one number whose drift caused this bug is guarded
by nothing but the note above it. Diff it by hand on any AMS bump.
