#!/usr/bin/env python3
"""Radio v2 snapshot round-trip: prove the firmware serializer produces bytes
the LIVE ground station decodes correctly.

Runs the SIL binary with --dump-radio (which serializes radio_test_inputs()
via the real firmware serialize_radio_snapshot) and feeds the 102 bytes through
`_decode_snapshot()` -- copied VERBATIM from the ground-station parser
IFS08-TE (branch feat/receptor_08) ISC_REAL_TIME_25/ISC_RTT_serial.py -- then
asserts every field matches the known inputs. If either side's byte layout
drifts, this fails.

Usage:  python3 radio_snapshot_roundtrip.py [path/to/ecu08_sil]
"""
import struct
import subprocess
import sys
from pathlib import Path

# ============ VERBATIM from IFS08-TE feat/receptor_08 ISC_RTT_serial.py ============
SNAPSHOT_SIZE = 102  # kRadioSnapshotWireSize

def _u16(data, offset): return struct.unpack_from('<H', data, offset)[0]
def _i16(data, offset): return struct.unpack_from('<h', data, offset)[0]
def _i32(data, offset): return struct.unpack_from('<i', data, offset)[0]
def _u32(data, offset): return struct.unpack_from('<I', data, offset)[0]

def _decode_snapshot(data: bytes) -> dict:
    if len(data) < SNAPSHOT_SIZE:
        return {}
    return {
        'tick_ms':             _u32(data, 0),
        'seq':                 _u16(data, 4),
        'start_button':        data[6],
        'apps1_raw':           _u16(data, 7),
        'apps2_raw':           _u16(data, 9),
        'brake_raw':           _u16(data, 11),
        'torque_pct':          _u16(data, 13),
        'reserved_15':         data[15],
        't11_8_9':             data[16],
        'state':               data[17],
        'ok_precharge':        data[18],
        'ams_fsm_state':       data[19],
        'v_cell_min_mV':       _u16(data, 20),
        'soc':                 data[22],
        'vmin_modulo':         [_u16(data, 23 + 2 * i) for i in range(5)],
        'vmax_modulo':         [_u16(data, 33 + 2 * i) for i in range(5)],
        'corriente_accu':      _i16(data, 43),
        'corriente_dcdc':      _i16(data, 45),
        'temp_dcdc':           _i16(data, 47),
        'temp_max_modulo':     [_i16(data, 49 + 2 * i) for i in range(5)],
        'inv_state':           data[59],
        'last_vconfig_tick':   data[60],
        'inv_error':           data[61],
        'inv_dc_bus_V':        _u16(data, 62),
        'inv_temp_motor1':     _u16(data, 64),
        'inv_temp_pwrstg':     _u16(data, 66),
        'inv_temp_board':      _u16(data, 68),
        'inv_rpm':             _i32(data, 70),
        'inv_speed_actual':    _i32(data, 74),
        'inv_current_actual':  _i32(data, 78),
    }
# ==================================================================================

# Expected == radio_test_inputs() in sil_control_tests.cpp.
EXPECTED = {
    'tick_ms': 0x11223344, 'seq': 0xABCD, 'start_button': 1,
    'apps1_raw': 2500, 'apps2_raw': 2400, 'brake_raw': 1234, 'torque_pct': 77,
    'reserved_15': 0, 't11_8_9': 1, 'state': 5, 'ok_precharge': 1, 'ams_fsm_state': 3,
    'v_cell_min_mV': 3650, 'soc': 87,
    'vmin_modulo': [3600, 3601, 3602, 3603, 3604],
    'vmax_modulo': [3700, 3701, 3702, 3703, 3704],
    'corriente_accu': -421, 'corriente_dcdc': 55, 'temp_dcdc': 38,
    'temp_max_modulo': [30, 31, 32, 33, 34],
    'inv_state': 6, 'last_vconfig_tick': 1, 'inv_error': 0, 'inv_dc_bus_V': 550,
    'inv_temp_motor1': 72, 'inv_temp_pwrstg': 68, 'inv_temp_board': 55,
    'inv_rpm': -12345, 'inv_speed_actual': 0, 'inv_current_actual': 0,
}


def main() -> int:
    sil = sys.argv[1] if len(sys.argv) > 1 else str(
        Path(__file__).resolve().parents[2] / "build-sil/tests/sil/ecu08_sil")
    out = subprocess.run([sil, "--dump-radio"], capture_output=True, text=True, check=True).stdout
    hexline = next(l.strip() for l in out.splitlines() if len(l.strip()) == 2 * SNAPSHOT_SIZE)
    data = bytes.fromhex(hexline)

    decoded = _decode_snapshot(data)
    fails = [(k, EXPECTED[k], decoded.get(k)) for k in EXPECTED if decoded.get(k) != EXPECTED[k]]
    if fails:
        print("ROUND-TRIP FAILED — firmware bytes decode wrong on the real parser:")
        for k, exp, got in fails:
            print(f"  {k}: expected {exp}, parser decoded {got}")
        return 1
    print(f"ROUND-TRIP OK — all {len(EXPECTED)} fields decode correctly through the "
          f"real ground-station _decode_snapshot ({SNAPSHOT_SIZE}-byte v2 snapshot).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
