#!/usr/bin/env python3
"""
Audi TT 8N Magneti Marelli / Jaeger M73 cluster dump tool (68HC912DG128).

Works with two dump formats:
  * 64 KB "direct" reads of the whole CPU address space (VAG K+CAN Commander)
  * 2 KB EEPROM-only reads of 0x3800-0x3FFF (kw1281test DumpMarelliMem 14336 2048)

All addresses printed/accepted by this tool are CPU addresses (i.e. the EEPROM
always lives at 0x3800-0x3FFF), regardless of which file format is loaded.

No external dependencies. Python 3.8+.

Sub-commands:
  info            decode part number, odometer ring, immo/key data, unknown fields
  verify          check structural invariants (complement ring, key copies ...)
  extract         64 KB -> 2 KB EEPROM image
  inject          2 KB EEPROM image -> into a 64 KB template
  diff            byte-level diff of two dumps (any format mix)
  hexdump         dump a range
  strings         printable ASCII strings
  set-odometer    rewrite the odometer ring buffer (+ inverted copy)
  set-vin         write a VIN into the 920 (Immo 3) VIN slots
  set-bytes       raw patch at an address
  checksum-scan   brute-force search for 8/16-bit checksums (sanity check)
"""
from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from functools import reduce
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

# --------------------------------------------------------------------------
# Memory layout (CPU addresses). Confidence levels:
#   [V] verified on both dumps in this repo
#   [P] plausible / README interpretation, not independently confirmed
#   [?] unknown purpose, listed so modifications can be reviewed
# --------------------------------------------------------------------------
EEPROM_START = 0x3800
EEPROM_SIZE = 0x0800
EEPROM_END = EEPROM_START + EEPROM_SIZE  # exclusive
FULL_SIZE = 0x10000

RAM_END = 0x3800          # 0x0000-0x37FF: working RAM / loader (volatile)   [V]
FLASH_START = 0x4000      # 0x4000-0xFFFF: internal flash (firmware)         [V]

# Odometer ring buffer                                                   [V]
ODO_ENTRIES = 36
ODO_ENTRY_LEN = 4
ODO_RING_INV = 0x3899     # 36 x 4 bytes, bitwise NOT of the plain ring
ODO_RING = 0x3929         # 36 x 4 bytes, big-endian km, step 2 km, circular
ODO_STEP = 2
ODO_RING_LEN = ODO_ENTRIES * ODO_ENTRY_LEN  # 0x90
assert ODO_RING_INV + ODO_RING_LEN == ODO_RING

PART_NUMBER = (0x3A29, 0x3A38)   # "8N1919860C  D29" / "8N2920950A  D04"  [V]
SERIAL_A = (0x384A, 0x3855)      # "ZAJUK3CD011" / "RALMLL5C003"            [?]

# Immo / PIN area. The README interprets 0x3881.. as PIN + key count for
# the 919 (Immo 2). In the 920 dump the same bytes are shifted by one.
IMMO_AREA = (0x3880, 0x3890)                                         # [P]
UNKNOWN_COUNTERS = (0x3894, 0x3899)  # "03 8e ff ff ff" / "03 6e ff 00 64" [?]

# Key / transponder blocks, three redundant copies (stride 41 / 59 bytes)
# 919: 16 bytes = 2 x 8-byte key records; the byte before each copy is 0x02
#      (= number of keys?) for copies 2/3, but "02 bc" precedes copy 1.
KEY_BLOCKS_919 = [(0x3A3A, 16), (0x3A63, 16), (0x3A8C, 16)]          # [V] copies identical
KEY_BLOCKS_920 = [(0x3B4B, 35), (0x3B86, 35), (0x3BC1, 35)]          # [V] copies identical
KEY_PREFIX_LEN = 2  # bytes shown before each copy for context

# VIN slots (920 / Immo 3 only; the 919 dump has 0xFF here)             [V] present
VIN_SLOTS = [0x3BFD, 0x3C18]   # 17 ASCII chars each; 0x3BFD held "XXXXXXXXXXXXXXXXX",
                               # 0x3C18 held the donor car's VIN. Which one the
                               # firmware/ECU compares against is not verified.
VIN_LEN = 17

# Flash strings (64 KB dumps only)
FLASH_SW_VERSION = (0xC003, 0xC014)   # "V7.66ML   EP_2002"              [V]


# --------------------------------------------------------------------------
# Dump container
# --------------------------------------------------------------------------
@dataclass
class Dump:
    data: bytearray
    base: int          # CPU address of data[0]
    path: str = ""

    @classmethod
    def load(cls, path: str, base: Optional[int] = None) -> "Dump":
        raw = bytearray(Path(path).read_bytes())
        if base is None:
            if len(raw) == FULL_SIZE:
                base = 0
            elif len(raw) == EEPROM_SIZE:
                base = EEPROM_START
            else:
                sys.exit(f"{path}: unrecognised size {len(raw)} bytes; "
                         f"pass --base to say which CPU address offset 0 maps to")
        return cls(raw, base, path)

    @property
    def end(self) -> int:
        return self.base + len(self.data)

    @property
    def kind(self) -> str:
        if self.base == 0 and len(self.data) == FULL_SIZE:
            return "full 64 KB address-space dump"
        if self.base == EEPROM_START and len(self.data) == EEPROM_SIZE:
            return "2 KB EEPROM-only dump"
        return f"custom window 0x{self.base:04X}-0x{self.end - 1:04X}"

    def has(self, start: int, end: int) -> bool:
        return self.base <= start and end <= self.end

    def __getitem__(self, addr):
        if isinstance(addr, slice):
            s = addr.start if addr.start is not None else self.base
            e = addr.stop if addr.stop is not None else self.end
            if not self.has(s, e):
                raise IndexError(f"range 0x{s:04X}-0x{e:04X} not in dump")
            return bytes(self.data[s - self.base:e - self.base])
        if not self.has(addr, addr + 1):
            raise IndexError(f"address 0x{addr:04X} not in dump")
        return self.data[addr - self.base]

    def write(self, addr: int, payload: bytes) -> None:
        if not self.has(addr, addr + len(payload)):
            raise IndexError(f"range 0x{addr:04X}+{len(payload)} not in dump")
        self.data[addr - self.base:addr - self.base + len(payload)] = payload

    def eeprom(self) -> bytes:
        return self[EEPROM_START:EEPROM_END]

    def save(self, path: str) -> None:
        Path(path).write_bytes(bytes(self.data))


# --------------------------------------------------------------------------
# Decoding helpers
# --------------------------------------------------------------------------
def hexdump(data: bytes, base: int, width: int = 16) -> str:
    out = []
    for off in range(0, len(data), width):
        row = data[off:off + width]
        hexpart = " ".join(f"{b:02x}" for b in row)
        asc = "".join(chr(b) if 32 <= b < 127 else "." for b in row)
        out.append(f"{base + off:05X}: {hexpart:<{width * 3 - 1}}  {asc}")
    return "\n".join(out)


def ascii_or_hex(b: bytes) -> str:
    if all(32 <= x < 127 for x in b):
        return repr(b.decode("ascii"))
    return b.hex(" ")


def read_ring(d: Dump) -> Tuple[List[int], List[int]]:
    plain = [int.from_bytes(d[ODO_RING + 4 * i:ODO_RING + 4 * i + 4], "big")
             for i in range(ODO_ENTRIES)]
    inv = [int.from_bytes(d[ODO_RING_INV + 4 * i:ODO_RING_INV + 4 * i + 4], "big")
           for i in range(ODO_ENTRIES)]
    return plain, inv


@dataclass
class RingInfo:
    plain: List[int]
    inv: List[int]
    newest_index: int
    km: int
    complement_ok: bool
    sequence_ok: bool
    problems: List[str]


def analyse_ring(d: Dump) -> RingInfo:
    plain, inv = read_ring(d)
    problems: List[str] = []
    comp_ok = all((p ^ 0xFFFFFFFF) == q for p, q in zip(plain, inv))
    if not comp_ok:
        bad = [i for i, (p, q) in enumerate(zip(plain, inv)) if (p ^ 0xFFFFFFFF) != q]
        problems.append(f"inverted copy mismatch at ring index(es) {bad}")
    if any(p == 0xFFFFFFFF for p in plain):
        problems.append("ring contains erased (0xFFFFFFFF) entries")
    km = max(plain)
    newest = plain.index(km)
    # expected: 36 consecutive values with step 2, newest at `newest`, oldest right after
    expected = sorted(range(km - ODO_STEP * (ODO_ENTRIES - 1), km + 1, ODO_STEP))
    seq_ok = sorted(plain) == expected
    if not seq_ok:
        problems.append("ring values are not 36 consecutive values with step 2")
    else:
        # check circular order
        order_ok = all(plain[(newest - k) % ODO_ENTRIES] == km - ODO_STEP * k
                       for k in range(ODO_ENTRIES))
        if not order_ok:
            problems.append("ring values are not in circular order")
            seq_ok = False
    return RingInfo(plain, inv, newest, km, comp_ok, seq_ok, problems)


def build_ring(km: int, newest_index: int) -> Tuple[bytes, bytes]:
    """Return (plain_ring_bytes, inverted_ring_bytes) for a given km value."""
    if not (0 <= km <= 0xFFFFFF):
        raise ValueError("km out of range (0..16777215)")
    if km - ODO_STEP * (ODO_ENTRIES - 1) < 0:
        raise ValueError(f"km must be >= {ODO_STEP * (ODO_ENTRIES - 1)} to fill the ring")
    vals = [0] * ODO_ENTRIES
    for k in range(ODO_ENTRIES):
        vals[(newest_index - k) % ODO_ENTRIES] = km - ODO_STEP * k
    plain = b"".join(v.to_bytes(4, "big") for v in vals)
    inv = bytes(b ^ 0xFF for b in plain)
    return plain, inv


def generation(d: Dump) -> str:
    """'919' (non-CAN / Immo2), '920' (CAN / Immo3) or 'unknown'."""
    try:
        pn = d[PART_NUMBER[0]:PART_NUMBER[1]].decode("ascii", "replace")
    except IndexError:
        return "unknown"
    m = re.match(r"8N[12](919|920)", pn)
    return m.group(1) if m else "unknown"


def find_strings(data: bytes, base: int, minlen: int = 6) -> Iterable[Tuple[int, str]]:
    for m in re.finditer(rb"[\x20-\x7e]{%d,}" % minlen, data):
        yield base + m.start(), m.group().decode("ascii")


def find_immo_ids(data: bytes, base: int) -> List[Tuple[int, str]]:
    # VAG immobiliser ID: 3 letters + 11 alphanumerics, e.g. AUZ5Z0B3245205,
    # not part of a longer alphanumeric run (which would be a VIN).
    return [(base + m.start(), m.group().decode())
            for m in re.finditer(rb"(?<![0-9A-Z])[A-Z]{3}[0-9A-Z]{11}(?![0-9A-Z])", data)]


def find_vins(data: bytes, base: int) -> List[Tuple[int, str]]:
    # 17 chars, VIN alphabet (no I/O/Q) or the all-X placeholder
    return [(base + m.start(), m.group().decode())
            for m in re.finditer(rb"(?<![0-9A-Z])(?:[A-HJ-NPR-Z0-9]{17}|X{17})(?![0-9A-Z])", data)]


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------
def cmd_info(args) -> int:
    d = Dump.load(args.dump, args.base)
    gen = generation(d)
    print(f"File      : {d.path}")
    print(f"Format    : {d.kind}")
    print(f"Generation: {gen}  "
          f"({'non-CAN, Immo 2' if gen == '919' else 'CAN, Immo 3' if gen == '920' else '?'})")
    print()

    print("== Identification (EEPROM) ==")
    pn = d[PART_NUMBER[0]:PART_NUMBER[1]]
    print(f"  0x{PART_NUMBER[0]:04X} part number / index : {ascii_or_hex(pn)}")
    sa = d[SERIAL_A[0]:SERIAL_A[1]]
    print(f"  0x{SERIAL_A[0]:04X} serial-like string  : {ascii_or_hex(sa)}   [purpose unknown]")
    ids = find_immo_ids(d.eeprom(), EEPROM_START)
    if ids:
        for addr, s in ids:
            print(f"  0x{addr:04X} immobiliser ID      : {s}")
    else:
        print("  immobiliser ID      : not present as ASCII in EEPROM (normal for 919/Immo 2)")
    vins = find_vins(d.eeprom(), EEPROM_START)
    for addr, s in vins:
        tag = "placeholder" if set(s) == {"X"} else "VIN (donor vehicle if cluster is used)"
        print(f"  0x{addr:04X} VIN slot            : {s}   [{tag}]")
    if not vins and gen == "920":
        print("  VIN slots           : empty")
    if d.has(FLASH_SW_VERSION[0], FLASH_SW_VERSION[1]):
        sw = d[FLASH_SW_VERSION[0]:FLASH_SW_VERSION[1]]
        print(f"  0x{FLASH_SW_VERSION[0]:04X} firmware version    : {ascii_or_hex(sw)}   [flash]")
        for addr, s in find_strings(d[FLASH_START:FULL_SIZE], FLASH_START, 8):
            if "KOMBI" in s or "Wegfahrs" in s:
                print(f"  0x{addr:04X} flash text          : {s.strip()!r}")
    print()

    print("== Odometer ring buffer ==")
    r = analyse_ring(d)
    print(f"  plain ring   : 0x{ODO_RING:04X}-0x{ODO_RING + ODO_RING_LEN - 1:04X}  "
          f"({ODO_ENTRIES} x {ODO_ENTRY_LEN} bytes, big-endian km, step {ODO_STEP})")
    print(f"  inverted ring: 0x{ODO_RING_INV:04X}-0x{ODO_RING_INV + ODO_RING_LEN - 1:04X}  "
          f"(bitwise NOT) -> {'OK' if r.complement_ok else 'MISMATCH'}")
    print(f"  sequence     : {'OK (36 consecutive, circular)' if r.sequence_ok else 'IRREGULAR'}")
    print(f"  newest entry : index {r.newest_index} @ 0x{ODO_RING + 4 * r.newest_index:04X}")
    print(f"  ODOMETER     : {r.km:,} km  (0x{r.km:06X})   "
          f"= {r.km / 1.609344:,.1f} mi")
    print(f"  oldest entry : {min(r.plain):,} km")
    print(f"  parity       : {'even' if r.km % 2 == 0 else 'odd'} values")
    for p in r.problems:
        print(f"  ! {p}")
    if d.has(0, RAM_END):
        needle = r.km.to_bytes(3, "big")
        hits = [i for i in range(0, RAM_END - 3) if d.data[i:i + 3] == needle]
        print(f"  RAM copies of current km (volatile): "
              f"{', '.join(f'0x{h:04X}' for h in hits) or 'none'}")
    print()

    print("== Immobiliser / PIN area (interpretation NOT verified) ==")
    print(hexdump(d[IMMO_AREA[0]:IMMO_AREA[1]], IMMO_AREA[0]))
    b = d[0x3881:0x3886]
    if gen == "919":
        pin = int.from_bytes(b[0:2], "big")
        print(f"  README reading: PIN = {pin:05d} (0x{pin:04X} @0x3881), key count @0x3884 = {b[3]}")
    elif gen == "920":
        pin = int.from_bytes(d[0x3882:0x3884], "big")
        print(f"  Same structure shifted by 1 would give: PIN = {pin:05d} (0x{pin:04X} @0x3882)")
        print("  -> confirm against what VAG Commander / VCDS report before trusting this.")
    print()

    print("== Key / transponder blocks (3 redundant copies) ==")
    blocks = KEY_BLOCKS_919 if gen == "919" else KEY_BLOCKS_920 if gen == "920" else []
    copies = []
    for addr, ln in blocks:
        blk = d[addr:addr + ln]
        copies.append(blk)
        prefix = d[addr - KEY_PREFIX_LEN:addr]
        state = "  (erased)" if all(x == 0xFF for x in blk) else ""
        print(f"  0x{addr:04X}: [{prefix.hex(' ')}] {blk.hex(' ')}{state}")
    if copies:
        print(f"  all three copies identical: {copies[0] == copies[1] == copies[2]}")
    if gen == "919" and copies and not all(x == 0xFF for x in copies[0]):
        print(f"  decode guess: key1={copies[0][:8].hex(' ')}  key2={copies[0][8:].hex(' ')}"
              f"  (RAM holds key1 byte-reversed)")
    elif gen == "920" and copies:
        if not all(x == 0xFF for x in copies[0]):
            c = copies[0]
            print(f"  decode guess: count={c[0]}  ?={c[1:3].hex(' ')}  "
                  f"id={c[3:17].decode('ascii', 'replace')}  tail={c[17:27].hex(' ')}  "
                  f"key1={c[27:31].hex(' ')} key2={c[31:35].hex(' ')}")
    print()

    print("== Unknown fields near the ring (left untouched by set-odometer) ==")
    print(hexdump(d[UNKNOWN_COUNTERS[0]:UNKNOWN_COUNTERS[1]], UNKNOWN_COUNTERS[0]))
    print(hexdump(d[0x3828:0x3838], 0x3828))
    print(hexdump(d[0x3860:0x386C], 0x3860))
    return 0


def cmd_verify(args) -> int:
    d = Dump.load(args.dump, args.base)
    problems: List[str] = []
    r = analyse_ring(d)
    problems += r.problems
    gen = generation(d)
    if gen == "unknown":
        problems.append("part number at 0x3A29 not recognised")
    blocks = KEY_BLOCKS_919 if gen == "919" else KEY_BLOCKS_920 if gen == "920" else []
    copies = [d[a:a + n] for a, n in blocks]
    if copies and not (copies[0] == copies[1] == copies[2]):
        problems.append(f"{gen} key block copies differ")
    ee = d.eeprom()
    if all(x == 0xFF for x in ee):
        problems.append("EEPROM region is completely erased")
    if d.has(0, FULL_SIZE):
        rv = int.from_bytes(d[0xFFFE:0x10000], "big")
        if not (FLASH_START <= rv < FULL_SIZE):
            problems.append(f"reset vector 0x{rv:04X} does not point into flash")
    if problems:
        print(f"{d.path}: {len(problems)} problem(s)")
        for p in problems:
            print("  !", p)
        return 1
    print(f"{d.path}: OK  ({gen}, {r.km:,} km, ring and key copies consistent)")
    return 0


def cmd_extract(args) -> int:
    d = Dump.load(args.dump, args.base)
    if not d.has(EEPROM_START, EEPROM_END):
        sys.exit("input does not cover 0x3800-0x3FFF")
    Path(args.out).write_bytes(d.eeprom())
    print(f"wrote {EEPROM_SIZE} bytes (CPU 0x{EEPROM_START:04X}-0x{EEPROM_END - 1:04X}) -> {args.out}")
    return 0


def cmd_inject(args) -> int:
    tpl = Dump.load(args.template, None)
    ee = Dump.load(args.eeprom, args.base)
    if not tpl.has(EEPROM_START, EEPROM_END):
        sys.exit("template does not cover 0x3800-0x3FFF")
    tpl.write(EEPROM_START, ee.eeprom())
    tpl.save(args.out)
    print(f"injected EEPROM from {ee.path} into {tpl.path} -> {args.out}")
    return 0


def cmd_diff(args) -> int:
    a = Dump.load(args.a, args.base_a)
    b = Dump.load(args.b, args.base_b)
    lo, hi = max(a.base, b.base), min(a.end, b.end)
    if args.eeprom_only:
        lo, hi = max(lo, EEPROM_START), min(hi, EEPROM_END)
    if lo >= hi:
        sys.exit("dumps do not overlap")
    diffs = [i for i in range(lo, hi) if a[i] != b[i]]
    print(f"comparing CPU 0x{lo:04X}-0x{hi - 1:04X}: {len(diffs)} differing byte(s)")
    regions = [("RAM/loader", 0, RAM_END), ("EEPROM", EEPROM_START, EEPROM_END),
               ("flash", FLASH_START, FULL_SIZE)]
    for name, s, e in regions:
        n = sum(1 for i in diffs if s <= i < e)
        if lo < e and hi > s:
            print(f"  {name:<11}: {n}")
    # group into runs
    runs: List[Tuple[int, int]] = []
    for i in diffs:
        if runs and i == runs[-1][1] + 1:
            runs[-1] = (runs[-1][0], i)
        else:
            runs.append((i, i))
    shown = 0
    for s, e in runs:
        if shown >= args.max_runs:
            print(f"  ... {len(runs) - shown} more run(s)")
            break
        n = e - s + 1
        if n <= 16:
            print(f"  0x{s:04X}+{n:<3} {a[s:e + 1].hex(' ')}  ->  {b[s:e + 1].hex(' ')}")
        else:
            print(f"  0x{s:04X}+{n:<3} {a[s:s + 8].hex(' ')} ..  ->  {b[s:s + 8].hex(' ')} ..")
        shown += 1
    return 1 if diffs else 0


def cmd_hexdump(args) -> int:
    d = Dump.load(args.dump, args.base)
    s = args.start if args.start is not None else (EEPROM_START if d.base == 0 else d.base)
    e = args.end if args.end is not None else (EEPROM_END if s == EEPROM_START else d.end)
    print(hexdump(d[s:e], s))
    return 0


def cmd_strings(args) -> int:
    d = Dump.load(args.dump, args.base)
    for addr, s in find_strings(bytes(d.data), d.base, args.min):
        print(f"0x{addr:04X}: {s}")
    return 0


def cmd_set_odometer(args) -> int:
    d = Dump.load(args.dump, args.base)
    before = analyse_ring(d)
    if args.from_dump:
        src = Dump.load(args.from_dump, None)
        km = analyse_ring(src).km
        print(f"source {src.path}: {km:,} km")
    else:
        km = args.km
    if km is None:
        sys.exit("give --km N or --from-dump FILE")
    if km % 2 != before.km % 2:
        if args.match_parity:
            km += 1
            print(f"adjusted to {km:,} km to keep the ring parity of this cluster")
        else:
            print(f"note: ring parity changes ({'even' if before.km % 2 == 0 else 'odd'} -> "
                  f"{'even' if km % 2 == 0 else 'odd'}); both parities occur in real "
                  f"dumps, use --match-parity to keep the existing one")
    plain, inv = build_ring(km, before.newest_index)
    d.write(ODO_RING, plain)
    d.write(ODO_RING_INV, inv)
    after = analyse_ring(d)
    assert after.km == km and after.complement_ok and after.sequence_ok
    if args.out == d.path and not args.in_place:
        sys.exit("refusing to overwrite the input; pass --in-place or a different --out")
    d.save(args.out)
    changed = sum(1 for i in range(ODO_RING_INV, ODO_RING + ODO_RING_LEN)
                  if Dump.load(args.dump, args.base)[i] != d[i])
    print(f"{before.km:,} km -> {after.km:,} km  (newest entry kept at ring index "
          f"{before.newest_index}, {changed} bytes changed in 0x{ODO_RING_INV:04X}-"
          f"0x{ODO_RING + ODO_RING_LEN - 1:04X})")
    print(f"wrote {args.out}")
    if d.has(0, RAM_END):
        print("note: RAM copies in 0x0000-0x37FF were not touched (volatile, rebuilt by firmware)")
    return 0


def cmd_set_vin(args) -> int:
    d = Dump.load(args.dump, args.base)
    vin = args.vin.strip().upper()
    if len(vin) != VIN_LEN or not re.fullmatch(r"[A-HJ-NPR-Z0-9]{17}", vin):
        sys.exit("VIN must be 17 characters, letters/digits without I, O, Q")
    slots = [args.slot] if args.slot is not None else VIN_SLOTS
    for addr in slots:
        old = d[addr:addr + VIN_LEN]
        if not (all(x == 0xFF for x in old) or re.fullmatch(rb"[A-Z0-9]{17}", old)):
            if not args.force:
                sys.exit(f"0x{addr:04X} does not look like a VIN slot ({old.hex(' ')}); use --force")
        d.write(addr, vin.encode("ascii"))
        print(f"0x{addr:04X}: {ascii_or_hex(old)} -> {vin!r}")
    if args.out == d.path and not args.in_place:
        sys.exit("refusing to overwrite the input; pass --in-place or a different --out")
    d.save(args.out)
    print(f"wrote {args.out}")
    return 0


def cmd_set_bytes(args) -> int:
    d = Dump.load(args.dump, args.base)
    payload = bytes.fromhex(args.hex.replace(" ", ""))
    old = d[args.at:args.at + len(payload)]
    d.write(args.at, payload)
    if args.out == d.path and not args.in_place:
        sys.exit("refusing to overwrite the input; pass --in-place or a different --out")
    d.save(args.out)
    region = ("EEPROM" if EEPROM_START <= args.at < EEPROM_END
              else "RAM (volatile!)" if args.at < RAM_END else "flash")
    print(f"0x{args.at:04X} [{region}]: {old.hex(' ')} -> {payload.hex(' ')}  -> {args.out}")
    return 0


def _crc8(poly: int, init: int):
    tbl = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        tbl.append(c)

    def f(data: bytes) -> int:
        c = init
        for x in data:
            c = tbl[c ^ x]
        return c
    return f


def _crc16(poly: int, init: int, refin: bool):
    tbl = []
    for i in range(256):
        if refin:
            c = i
            for _ in range(8):
                c = (c >> 1) ^ poly if c & 1 else c >> 1
        else:
            c = i << 8
            for _ in range(8):
                c = ((c << 1) ^ poly) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        tbl.append(c)

    def f(data: bytes) -> int:
        c = init
        for x in data:
            c = (tbl[(c ^ x) & 0xFF] ^ (c >> 8)) if refin else (tbl[((c >> 8) ^ x) & 0xFF] ^ ((c << 8) & 0xFFFF))
        return c & 0xFFFF
    return f


def cmd_checksum_scan(args) -> int:
    """Look for a byte/word adjacent to a range that equals sum/xor/CRC of it.
    With two dumps given, only candidates valid in BOTH are reported, which
    filters out most coincidences."""
    dumps = [Dump.load(p, None) for p in args.dumps]
    algos = {
        "sum8": lambda r: sum(r) & 0xFF,
        "neg8": lambda r: (-sum(r)) & 0xFF,
        "xor8": lambda r: reduce(lambda x, y: x ^ y, r, 0),
        "crc8-07": _crc8(0x07, 0x00),
        "crc8-1d": _crc8(0x1D, 0xFF),
        "sum16": lambda r: sum(r) & 0xFFFF,
        "crc16-ccitt": _crc16(0x1021, 0xFFFF, False),
        "crc16-modbus": _crc16(0xA001, 0xFFFF, True),
    }

    def informative(r: bytes) -> bool:
        return sum(1 for x in r if x not in (0, 0xFF)) >= max(3, len(r) // 2)

    in_ring = lambda c: ODO_RING_INV <= c < ODO_RING + ODO_RING_LEN  # noqa: E731
    hits = []
    for L in range(args.min_len, args.max_len + 1):
        for s in range(EEPROM_START, EEPROM_END - L):
            rs = [d[s:s + L] for d in dumps]
            if not all(informative(r) for r in rs):
                continue
            for name, f in algos.items():
                w = 2 if "16" in name else 1
                vals = [f(r) for r in rs]
                if any(v in (0, (1 << (8 * w)) - 1) for v in vals):
                    continue
                for c in (s - w, s + L):
                    if c < EEPROM_START or c + w > EEPROM_END:
                        continue
                    if not args.include_ring and (in_ring(c) or in_ring(s)):
                        continue
                    for en in (("big", "little") if w == 2 else ("big",)):
                        if all(int.from_bytes(d[c:c + w], en) == v for d, v in zip(dumps, vals)):
                            hits.append((name, en, s, s + L, c))
    print(f"{len(hits)} candidate(s) (ranges inside the odometer ring are "
          f"{'included' if args.include_ring else 'excluded: they are artefacts of the complement pairs'})")
    for name, en, s, e, c in hits:
        print(f"  {name:<12} {en:<6} over 0x{s:04X}-0x{e - 1:04X} stored at 0x{c:04X}")
    return 0


# --------------------------------------------------------------------------
def auto_int(s: str) -> int:
    return int(s, 0)


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    def add_base(sp):
        sp.add_argument("--base", type=auto_int, default=None,
                        help="CPU address of file offset 0 (auto: 64KB->0, 2KB->0x3800)")

    s = sub.add_parser("info", help="decode a dump"); s.add_argument("dump"); add_base(s)
    s.set_defaults(fn=cmd_info)

    s = sub.add_parser("verify", help="check structural invariants"); s.add_argument("dump"); add_base(s)
    s.set_defaults(fn=cmd_verify)

    s = sub.add_parser("extract", help="64KB dump -> 2KB EEPROM image")
    s.add_argument("dump"); s.add_argument("out"); add_base(s)
    s.set_defaults(fn=cmd_extract)

    s = sub.add_parser("inject", help="2KB EEPROM image -> 64KB template")
    s.add_argument("template", help="64KB dump providing RAM/flash content")
    s.add_argument("eeprom", help="2KB (or 64KB) dump providing the EEPROM content")
    s.add_argument("out"); add_base(s)
    s.set_defaults(fn=cmd_inject)

    s = sub.add_parser("diff", help="compare two dumps")
    s.add_argument("a"); s.add_argument("b")
    s.add_argument("--base-a", type=auto_int); s.add_argument("--base-b", type=auto_int)
    s.add_argument("--eeprom-only", action="store_true")
    s.add_argument("--max-runs", type=int, default=200)
    s.set_defaults(fn=cmd_diff)

    s = sub.add_parser("hexdump", help="hex dump a range (default: EEPROM)")
    s.add_argument("dump"); s.add_argument("--start", type=auto_int); s.add_argument("--end", type=auto_int)
    add_base(s); s.set_defaults(fn=cmd_hexdump)

    s = sub.add_parser("strings", help="list ASCII strings")
    s.add_argument("dump"); s.add_argument("--min", type=int, default=6); add_base(s)
    s.set_defaults(fn=cmd_strings)

    s = sub.add_parser("set-odometer", help="rewrite the odometer ring buffer")
    s.add_argument("dump"); s.add_argument("--out", required=True)
    s.add_argument("--km", type=int); s.add_argument("--from-dump", help="copy km from another dump")
    s.add_argument("--match-parity", action="store_true", help="round up to keep existing even/odd parity")
    s.add_argument("--in-place", action="store_true"); add_base(s)
    s.set_defaults(fn=cmd_set_odometer)

    s = sub.add_parser("set-vin", help="write a VIN into the 920 VIN slot(s)")
    s.add_argument("dump"); s.add_argument("--out", required=True); s.add_argument("--vin", required=True)
    s.add_argument("--slot", type=auto_int, help="only this slot (default: all known slots)")
    s.add_argument("--force", action="store_true"); s.add_argument("--in-place", action="store_true")
    add_base(s); s.set_defaults(fn=cmd_set_vin)

    s = sub.add_parser("set-bytes", help="raw patch")
    s.add_argument("dump"); s.add_argument("--out", required=True)
    s.add_argument("--at", type=auto_int, required=True); s.add_argument("--hex", required=True)
    s.add_argument("--in-place", action="store_true"); add_base(s)
    s.set_defaults(fn=cmd_set_bytes)

    s = sub.add_parser("checksum-scan", help="search for checksums (slow, pure python)")
    s.add_argument("dumps", nargs="+")
    s.add_argument("--min-len", type=int, default=3); s.add_argument("--max-len", type=int, default=128)
    s.add_argument("--include-ring", action="store_true")
    s.set_defaults(fn=cmd_checksum_scan)

    args = p.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
