#!/usr/bin/env python3
"""Guard the FDCAN MessageRAM layout in Core/Src/fdcan.c.

WHY THIS EXISTS
---------------
The three FDCAN instances share one 10 KB SRAMCAN. Each instance's window
starts at its `MessageRAMOffset` and runs for as many words as its filter /
FIFO configuration needs. If two windows OVERLAP the instances silently
clobber each other's message RAM and transmit dies -- that was the root cause
of #48 ("TX-dead"), which cost days to find.

CubeMX has NO GUI field for MessageRAMOffset, so every single regeneration
resets all three to 0 -- i.e. every regen re-introduces #48. It has already
happened at least three times (most recently the SPI1 removal, #136/#152).
Relying on someone remembering to re-apply it by hand has not worked, so this
check fails the build instead.

WHAT IT CHECKS
--------------
It does NOT hard-code 0/387/582. It parses the actual Init fields, computes
each instance's true footprint from them, and verifies the windows are
non-overlapping, ordered, and inside SRAMCAN. That way it also catches a
FIFO-depth change that would silently make a hand-copied offset wrong --
which a magic-number check would happily wave through.

Word costs (STM32H7 RM0433 "Message RAM" / HAL FDCAN_Init):
  standard filter  1 word    extended filter  2 words
  Tx event FIFO    2 words   each Rx/Tx element  2 header words + data/4

Usage:  python3 scripts/check_fdcan_ram.py [path/to/fdcan.c]
Exit 0 = layout sound, 1 = overlap / overflow / unparseable.
"""

import re
import sys

# STM32H733 SRAMCAN is 10 KB shared across the three FDCAN instances.
SRAMCAN_WORDS = 2560

WORDS_PER_STD_FILTER = 1
WORDS_PER_EXT_FILTER = 2
WORDS_PER_TX_EVENT = 2
ELEMENT_HEADER_WORDS = 2


def data_bytes(sym: str) -> int:
    """FDCAN_DATA_BYTES_n -> n (payload bytes per element)."""
    m = re.search(r"FDCAN_DATA_BYTES_(\d+)", sym)
    if not m:
        raise ValueError(f"unrecognised element size: {sym!r}")
    return int(m.group(1))


def element_words(size_sym: str) -> int:
    return ELEMENT_HEADER_WORDS + data_bytes(size_sym) // 4


def parse(src: str, inst: str) -> dict:
    """Pull hfdcan<inst>.Init.<field> assignments out of the generated C."""
    out = {}
    for m in re.finditer(
        rf"hfdcan{inst}\.Init\.(\w+)\s*=\s*([^;]+);", src
    ):
        out[m.group(1)] = m.group(2).strip()
    return out


def footprint(f: dict) -> int:
    """Words of MessageRAM this instance actually occupies."""

    def n(key: str) -> int:
        return int(f.get(key, "0"))

    words = n("StdFiltersNbr") * WORDS_PER_STD_FILTER
    words += n("ExtFiltersNbr") * WORDS_PER_EXT_FILTER
    words += n("RxFifo0ElmtsNbr") * element_words(f.get("RxFifo0ElmtSize", "FDCAN_DATA_BYTES_8"))
    words += n("RxFifo1ElmtsNbr") * element_words(f.get("RxFifo1ElmtSize", "FDCAN_DATA_BYTES_8"))
    words += n("RxBuffersNbr") * element_words(f.get("RxBufferSize", "FDCAN_DATA_BYTES_8"))
    words += n("TxEventsNbr") * WORDS_PER_TX_EVENT
    tx_elt = element_words(f.get("TxElmtSize", "FDCAN_DATA_BYTES_8"))
    words += (n("TxBuffersNbr") + n("TxFifoQueueElmtsNbr")) * tx_elt
    return words


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else "Core/Src/fdcan.c"
    try:
        src = open(path, encoding="utf-8", errors="replace").read()
    except OSError as e:
        print(f"::error::cannot read {path}: {e}")
        return 1

    windows = []
    for inst in ("1", "2", "3"):
        f = parse(src, inst)
        if not f:
            print(f"FDCAN{inst}: not configured in {path}, skipping.")
            continue
        if "MessageRAMOffset" not in f:
            print(f"::error::FDCAN{inst} has no MessageRAMOffset assignment in {path}")
            return 1
        try:
            off = int(f["MessageRAMOffset"])
            size = footprint(f)
        except ValueError as e:
            print(f"::error::FDCAN{inst}: {e}")
            return 1
        windows.append((inst, off, size))

    if not windows:
        print(f"::error::no FDCAN instances parsed from {path} -- has the file moved?")
        return 1

    print(f"FDCAN MessageRAM layout ({path}), SRAMCAN = {SRAMCAN_WORDS} words:")
    for inst, off, size in windows:
        print(f"  FDCAN{inst}: offset {off:5d}  size {size:4d} words -> [{off}, {off + size})")

    ok = True

    # Overlap is the #48 failure: two instances sharing words.
    ordered = sorted(windows, key=lambda w: w[1])
    for (ai, ao, asz), (bi, bo, _bsz) in zip(ordered, ordered[1:]):
        if ao + asz > bo:
            print(
                f"::error::FDCAN{ai} [{ao}, {ao + asz}) OVERLAPS FDCAN{bi} at {bo} "
                f"-- this is the #48 TX-dead defect. A CubeMX regen resets "
                f"MessageRAMOffset to 0; re-apply the offsets in {path}. "
                f"FDCAN{bi} must start at or after {ao + asz}."
            )
            ok = False

    # Overflowing SRAMCAN corrupts whatever follows.
    for inst, off, size in windows:
        if off + size > SRAMCAN_WORDS:
            print(
                f"::error::FDCAN{inst} ends at {off + size} words, past SRAMCAN "
                f"({SRAMCAN_WORDS}). Shrink a FIFO or an offset."
            )
            ok = False

    if ok:
        used = max(o + s for _, o, s in windows)
        print(f"OK: no overlap, top word {used}/{SRAMCAN_WORDS} used.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
