#!/usr/bin/env python3
"""Diff the CAN frames the ECU and the AMS BOTH touch.

WHY THIS EXISTS. In August 2026 the ECU shipped a decoder for 0x021 that read
bit 0 as a "discharge request" while the AMS was sending `fsm_in_start` there,
and ignored bit 1 (`tsms`) entirely. Both repos agreed on the ID. Both had
careful comments. Neither had any way to notice they disagreed about the fields,
and it reached `dev`.

Frame freshness, DLC and bit offsets are all checkable mechanically from the two
generated DBCs, so there is no excuse for finding this by hand again.

WHAT IT CHECKS, for every ID present in both DBCs:
  - DLC matches
  - every signal the OTHER side defines exists here at the same start bit,
    length, byte order and signedness

Signal NAMES are allowed to differ (the ECU calls it `dc_bus_voltage`, the AMS
calls it `dc_bus_V`) -- only the wire layout has to agree. Extra signals on
either side are fine, as long as they do not collide.

USAGE
    python3 scripts/check_ams_contract.py [path/to/ams.dbc]

Defaults to ../IFS09-CE-AMS/docs/dbc/ams.dbc. SKIPS (exit 0) if the AMS repo is
not checked out, so it is safe in CI where only this repo is cloned -- it is a
local pre-flight check, not a gate that fails on someone else's absence.
"""
import re, sys, os

SIG = re.compile(r'^\s*SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@(\d)([+-])')
MSG = re.compile(r'^BO_\s+(\d+)\s+(\w+)\s*:\s*(\d+)\s+(\S+)')

def parse(path):
    frames, cur = {}, None
    for line in open(path, encoding='utf-8', errors='replace'):
        m = MSG.match(line)
        if m:
            fid, name, dlc = int(m.group(1)), m.group(2), int(m.group(3))
            cur = {'name': name, 'dlc': dlc, 'sender': m.group(4), 'sigs': {}}
            frames[fid] = cur
            continue
        s = SIG.match(line)
        if s and cur is not None:
            # key on LAYOUT, not name -- the two repos name signals differently
            cur['sigs'][(int(s.group(2)), int(s.group(3)), s.group(4), s.group(5))] = s.group(1)
    return frames

def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ecu_dbc = os.path.join(here, 'docs', 'dbc', 'ecu.dbc')
    ams_dbc = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(here), 'IFS09-CE-AMS', 'docs', 'dbc', 'ams.dbc')

    if not os.path.exists(ams_dbc):
        print(f"SKIP: AMS DBC not found at {ams_dbc}")
        print("      (clone isc-fs/IFS09-CE-AMS alongside this repo to enable this check)")
        return 0
    if not os.path.exists(ecu_dbc):
        print(f"FAIL: {ecu_dbc} missing -- regenerate with tools/dbc_dump.cpp")
        return 1

    ecu, ams = parse(ecu_dbc), parse(ams_dbc)
    shared = sorted(set(ecu) & set(ams))
    if not shared:
        print("FAIL: no shared frame IDs -- one of the DBCs is probably not what you think")
        return 1

    errors, warnings = [], []
    for fid in shared:
        e, a = ecu[fid], ams[fid]
        tag = f"0x{fid:03X} (ECU {e['name']} / AMS {a['name']})"
        # Who OWNS the frame decides which direction is dangerous. The AMS DBC's
        # sender field is authoritative: they name themselves on frames they send.
        ams_sends = (a['sender'].upper() == 'AMS')

        if e['dlc'] != a['dlc']:
            errors.append(f"{tag}: DLC differs -- ECU {e['dlc']}, AMS {a['dlc']}")

        # >>> THE RULE THAT WOULD HAVE CAUGHT THE 0x021 BUG. <<<
        # On a frame the AMS SENDS, we are the consumer. Every signal they put on
        # the wire must have a matching layout on our side -- a missing one means
        # we are either ignoring data they expect us to act on, or (as in August
        # 2026) we have mapped those bits to something else entirely. That is an
        # ERROR, not a version skew.
        for key, nm in a['sigs'].items():
            if key not in e['sigs']:
                where = f"{key[0]}|{key[1]}@{key[2]}{key[3]}"
                msg = f"{tag}: AMS sends '{nm}' @ {where} and the ECU has no signal there"
                if ams_sends:
                    errors.append(msg + " -- we are the RECEIVER and are not decoding it")
                else:
                    warnings.append(msg)

        # On a frame WE send, the AMS not decoding something is usually just one
        # repo running ahead. Warn.
        for key, nm in e['sigs'].items():
            if key not in a['sigs']:
                where = f"{key[0]}|{key[1]}@{key[2]}{key[3]}"
                msg = f"{tag}: ECU sends '{nm}' @ {where} and the AMS has no signal there"
                if ams_sends:
                    errors.append(msg + " -- we decode a field they do not send")
                else:
                    warnings.append(msg + " (they do not consume it -- fine if deliberate)")

        # Same bits, different NAME. Sometimes benign (dc_bus_voltage vs
        # dc_bus_V) but it is exactly how the 0x021 misread hid in plain sight,
        # so it is always worth a human look.
        for key, nm in e['sigs'].items():
            other = a['sigs'].get(key)
            if other and other != nm:
                warnings.append(f"{tag}: same bits {key[0]}|{key[1]}, different names -- "
                                f"ECU '{nm}' vs AMS '{other}'. Confirm they mean the SAME thing.")

    print(f"Compared {len(shared)} shared frame(s): " +
          ", ".join(f"0x{f:03X}" for f in shared))

    if warnings:
        print("\nWARNINGS -- look, but not necessarily wrong:")
        for w in warnings:
            print(f"  ~ {w}")

    if errors:
        print("\nCONTRACT MISMATCH:")
        for p_ in errors:
            print(f"  - {p_}")
        print("\nReconcile the .def files in BOTH repos -- never edit the generated DBC.")
        return 1

    print("\nPASS: every frame the AMS sends is fully decoded by the ECU.")
    return 0

if __name__ == '__main__':
    sys.exit(main())
