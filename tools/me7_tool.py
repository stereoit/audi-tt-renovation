#!/usr/bin/env python3
"""
Bosch ME7.5 (C167, 512 KB 29F400) flash image tool - Audi TT 8N 1.8T APX.

Verified against the images in ECU/ and cross-checked with ME7Sum (nyetwurk).
No external dependencies. Python 3.8+.

Address conventions
  * All addresses printed/accepted are FILE OFFSETS (0x00000-0x7FFFF).
  * The C167 sees the flash at 0x800000 + offset ("C167 address").
  * The calibration pages 0x10000-0x17FFF are addressed by code through
    DPP0 (=0x204) / DPP1 (=0x205) as 16-bit 0x0000-0x7FFF ("DPP address").

Layout (512 KB VAG ME7.5)
  0x00000-0x0FFFF  boot / loader / ROMSYS header (@0x8000)
  0x10000-0x1FFFF  calibration (maps, curves, codewords, ID strings)
  0x1FC00-0x1FE1F  multipoint checksum table (34 x 16 bytes)
  0x20000-0x7FFFF  program code; main-program checksum pair @0x7FFE0

Sub-commands
  info        ID strings, DPP setup, checksum summary, and current overrun/SAI
              settings read from the XDF in ECU/Profiles (--xdf, --no-xdf, --no-settings)
  checksums   verify every checksum (ROMSYS, CRC32 zones, background,
              main program, multipoint); --fix writes a corrected copy
  fix         shorthand for `checksums --fix`
  diff        byte diff of two images, each run annotated (checksum vs. data)
  apply-diff  port the (non-checksum) changes BASE->MOD onto TARGET, fix sums
  patch       write raw bytes at an offset (checksums fixed afterwards)
  where       what covers a given offset (region, checksums, DPP address)
  regions     print the region / checksum map
  xref        find code references to a calibration byte/word/table
  maps        list map & curve candidates (pointer-referenced or scanned)
  map         print one table (header auto-parsed, or --nx/--ny)
  hexdump     dump a range
  strings     printable ASCII strings
"""
from __future__ import annotations

import argparse
import hashlib
import re
import struct
import sys
import zlib
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

BASE = 0x800000               # C167 address of file offset 0
FLASH_512K = 0x80000
CAL_START, CAL_END = 0x10000, 0x20000     # calibration, end exclusive
ROMSYS = 0x8000               # ROMSYS descriptor
MASK32 = 0xFFFFFFFF

# --------------------------------------------------------------------------
# image container / low level helpers
# --------------------------------------------------------------------------


@dataclass
class Image:
    data: bytearray
    path: str = ""

    @classmethod
    def load(cls, path: str) -> "Image":
        raw = bytearray(Path(path).read_bytes())
        if len(raw) not in (FLASH_512K, 2 * FLASH_512K):
            sys.exit(f"{path}: {len(raw)} bytes - expected a 512 KB (or 1 MB) ME7 image")
        if len(raw) != FLASH_512K:
            print(f"warning: {path} is not 512 KB; this tool is only verified on 512 KB ME7.5", file=sys.stderr)
        return cls(raw, path)

    def __len__(self) -> int:
        return len(self.data)

    def u8(self, o: int) -> int:
        return self.data[o]

    def u16(self, o: int) -> int:
        return struct.unpack_from("<H", self.data, o)[0]

    def u32(self, o: int) -> int:
        return struct.unpack_from("<I", self.data, o)[0]

    def w32(self, o: int, v: int) -> None:
        struct.pack_into("<I", self.data, o, v & MASK32)

    def md5(self) -> str:
        return hashlib.md5(self.data).hexdigest()

    def save(self, path: str, overwrite_ok: bool = False) -> None:
        p = Path(path)
        if p.exists() and not overwrite_ok and p.resolve() == Path(self.path).resolve():
            sys.exit(f"refusing to overwrite the input file {path}; pass --in-place")
        p.write_bytes(self.data)


def parse_int(s: str) -> int:
    return int(s, 0)


def hexaddr(o: int) -> str:
    return f"0x{o:05X}"


def masked_find(data: bytes, needle: Sequence[int], mask: Sequence[int],
                start: int = 0, end: Optional[int] = None, step: int = 2) -> List[int]:
    """Return every offset (aligned to `step`) where data & mask == needle & mask."""
    end = len(data) if end is None else end
    n = len(needle)
    # quick pre-filter on the first fully-masked byte
    first = next((i for i, m in enumerate(mask) if m == 0xFF), None)
    hits = []
    for i in range(start, end - n + 1, step):
        if first is not None and data[i + first] != needle[first]:
            continue
        for j in range(n):
            if (data[i + j] ^ needle[j]) & mask[j]:
                break
        else:
            hits.append(i)
    return hits


def wordsum16(data: bytes, start: int, end_inclusive: int) -> int:
    """Sum of little-endian 16-bit words over [start, end], mod 2^32 (Bosch CalcChecksumBlk16)."""
    n = (end_inclusive - start + 1) // 2
    return sum(struct.unpack_from(f"<{n}H", data, start)) & MASK32


def crc32(data: bytes, start: int, end_inclusive: int) -> int:
    return zlib.crc32(bytes(data[start:end_inclusive + 1])) & MASK32


# --------------------------------------------------------------------------
# checksum model
# --------------------------------------------------------------------------


@dataclass
class Check:
    group: str                    # romsys / crc / bg / main / mp
    name: str
    ranges: List[Tuple[int, int]]  # inclusive file-offset ranges that are summed
    store: int                    # file offset of the stored value
    has_inv: bool                 # stored as (value, ~value) pair
    stored: int = 0
    stored_inv: int = 0
    calc: int = 0
    boot: bool = False            # bootrom multipoint entry (reported, never corrected)
    note: str = ""

    @property
    def inside(self) -> bool:
        width = 8 if self.has_inv else 4
        return any(self.store + width > s and self.store <= e for s, e in self.ranges)

    @property
    def inv_ok(self) -> bool:
        return (not self.has_inv) or self.stored_inv == (~self.stored & MASK32)

    @property
    def ok(self) -> bool:
        return self.stored == self.calc and self.inv_ok

    def span(self) -> str:
        return ", ".join(f"{hexaddr(s)}-{hexaddr(e)}" for s, e in self.ranges)


class ChecksumMap:
    """Locate every checksum structure of a VAG ME7.5 512 KB image (port of ME7Sum logic)."""

    def __init__(self, img: Image):
        self.img = img
        self.problems: List[str] = []
        self.checks: List[Check] = []
        self.crc_blocks: List[Tuple[int, int, int]] = []     # (start, end, store)
        self.main_table: Optional[int] = None
        self.mp_table: Optional[int] = None
        self.mp_count = 0
        self._discover()

    # -- discovery ---------------------------------------------------------
    def _discover(self) -> None:
        self.checks = []
        self.checks += self._romsys()
        self.checks += self._main_crcs()
        self.checks += self._background()
        self.checks += self._main_program()
        self.checks += self._multipoint()
        self.recompute()

    def _romsys(self) -> List[Check]:
        d = self.img
        if len(d) < ROMSYS + 0x40:
            return []
        sections_off = d.u32(ROMSYS + 0x08)
        checksums_off = d.u32(ROMSYS + 0x0C)
        all_param_p = d.u32(ROMSYS + 0x10)
        flash_top = d.u32(ROMSYS + 0x14)
        ap_start, ap_end = d.u32(ROMSYS + 0x30), d.u32(ROMSYS + 0x34)
        if sections_off != 0x20 or checksums_off != 0x38 or flash_top != BASE + len(d) - 1:
            self.problems.append(f"ROMSYS header @{hexaddr(ROMSYS)} not recognised "
                                 f"(sections={sections_off:#x} checksums={checksums_off:#x} top={flash_top:#x})")
            return []
        out = [Check("romsys", "ROMSYS startup  word[0x8000]+word[0xFFFE]",
                     [(0x8000, 0x8001), (0xFFFE, 0xFFFF)], ROMSYS + 0x38, False)]
        if BASE <= all_param_p < BASE + len(d) and BASE <= ap_start < ap_end < BASE + len(d):
            out.append(Check("romsys", "ROMSYS param page  word[start]+word[end]",
                             [(ap_start - BASE, ap_start - BASE + 1), (ap_end - BASE, ap_end - BASE + 1)],
                             all_param_p - BASE, False))
        else:
            self.problems.append("ROMSYS param-page pointer out of range")
        # program pages: first+last word of every 8 KB page in 0x0-0xFFFF and 0x20000-top
        prog_end = flash_top - BASE
        ranges = []
        for lo, hi in ((0x0, 0xFFFF), (0x20000, prog_end)):
            for a in range(lo, hi, 0x2000):
                ranges += [(a, a + 1), (a + 0x1FFE, a + 0x1FFF)]
        out.append(Check("romsys", "ROMSYS program pages  8k first+last words", ranges, ROMSYS + 0x3C, False))
        return out

    def _main_crcs(self) -> List[Check]:
        d = self.img.data
        # mov r8,#lo ; mov r9,#seg ; mov r4,<mem>
        n0 = [0xE6, 0xF8, 0, 0, 0xE6, 0xF9, 0x80, 0x00, 0xF2, 0xF4]
        m0 = [0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0xF0, 0xFF, 0xFF, 0xFF]
        # addc r9,r11 ; mov r4,#lo ; mov r5,#seg   (block end)
        n1 = [0x10, 0x9B, 0xE6, 0xF4, 0, 0, 0xE6, 0xF5, 0x80, 0x00]
        m1 = [0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0xF0, 0xFF]
        # mov <mem>,r5 ; mov r4,#lo ; mov r5,#seg ; calls   (stored CRC address)
        n2 = [0xF6, 0xF5, 0, 0, 0xE6, 0xF4, 0, 0, 0xE6, 0xF5, 0x80, 0x00, 0xDA, 0x00]
        m2 = [0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0x01, 0, 0xFF, 0xFF, 0xF0, 0xFF, 0xFF, 0xFF]

        def val(o: int, lo_idx: int, seg_idx: int) -> int:
            lo = struct.unpack_from("<H", d, o + 2 * lo_idx)[0]
            seg = struct.unpack_from("<H", d, o + 2 * seg_idx)[0]
            return ((seg << 16) | lo) - BASE

        starts = [val(o, 1, 3) for o in masked_find(d, n0, m0)]
        ends = [val(o, 2, 4) for o in masked_find(d, n1, m1)]
        stores = [val(o, 3, 5) for o in masked_find(d, n2, m2)]
        blocks = list(zip(starts, ends)) if starts and len(starts) == len(ends) else self._indirect_crc_blocks()
        if not (1 <= len(blocks) <= 4 and len(blocks) == len(stores)):
            self.problems.append(f"main data CRC blocks not found (blocks={len(blocks)} "
                                 f"stores={len(stores)}); CRC32 zones skipped")
            return []
        self.crc_blocks = [(s, e, st) for (s, e), st in zip(blocks, stores)]
        return [Check("crc", f"CRC32 zone {i + 1}", [(s, e)], st, False)
                for i, (s, e, st) in enumerate(self.crc_blocks)]

    def _indirect_crc_blocks(self) -> List[Tuple[int, int]]:
        """Blocks described relative to a pointer in r8/r9 (later ME7.5 code, this ROM):
             [add r8,#lo] addc r9,#seg ; cmp r9,#seg ; jmpr NE ; cmp r8,#(end-gap-1) ;
             mov r4,#gap ; ... ; mov r5,#end(exclusive)"""
        d = self.img.data
        n = [0x16, 0xF9, 0, 0, 0x46, 0xF9, 0, 0, 0x3D, 0x02,
             0x46, 0xF8, 0, 0, 0x9D, 0x05, 0xE6, 0xF4, 0, 0,
             0xC4, 0x40, 0x02, 0x00, 0x0D, 0x0A, 0xE7, 0xF8, 0, 0,
             0xF7, 0xF8, 0, 0xF9, 0xF0, 0x48, 0xE6, 0xF5, 0, 0]
        m = [0xFF if x else 0 for x in n]
        m[2] = m[3] = m[6] = m[7] = m[12] = m[13] = m[18] = m[19] = m[28] = m[29] = m[32] = m[38] = m[39] = 0
        u16 = lambda o: struct.unpack_from("<H", d, o)[0]
        out = []
        for i in masked_find(d, n, m):
            if i >= 4 and d[i - 4] == 0x06 and d[i - 3] == 0xF8:
                start_lo = u16(i - 2)
            elif i >= 2 and d[i - 2] == 0x08 and d[i - 1] == 0x80:
                start_lo = 0
            else:
                continue
            seg, seg2, cmpv, gap, end_lo = u16(i + 2), u16(i + 6), u16(i + 12), u16(i + 18), u16(i + 38)
            if seg != seg2 or end_lo <= start_lo or (end_lo - gap - 1) & 0xFFFF != cmpv:
                continue
            fs = ((seg << 16) | start_lo) - BASE
            fe = ((seg << 16) | end_lo) - 1 - BASE
            if 0 <= fs <= fe < len(d):
                out.append((fs, fe))
        return out

    def _descriptor(self, o: int) -> Tuple[int, int, int, int]:
        return struct.unpack_from("<IIII", self.img.data, o)

    def _desc_ok(self, o: int, allow_boot: bool = False) -> bool:
        if o < 0 or o + 16 > len(self.img):
            return False
        s, e, v, iv = self._descriptor(o)
        if v != (~iv & MASK32):
            return False
        if s < BASE or e < BASE:
            if not allow_boot:
                return False
            return s < e < len(self.img)
        return BASE <= s < e < BASE + len(self.img) and (e - s + 1) % 2 == 0

    def _background(self) -> List[Check]:
        """Descriptors ending just before the 'outer' 0x24000 block; anchor = block that
        covers 0x14000 up to (and including) its own descriptor."""
        d = self.img
        anchor = None
        for o in range(CAL_START, min(len(d), CAL_END) - 16, 2):
            s, e, v, iv = self._descriptor(o)
            if s == BASE + 0x14000 and e == BASE + o + 15:
                if anchor is not None:
                    return []
                anchor = o
        if anchor is None:
            return []
        offs = [anchor]
        o = anchor
        while len(offs) < 4 and o >= CAL_START + 16 and self._desc_ok(o - 16):
            s, e, _, _ = self._descriptor(o - 16)
            if e - s + 1 > 64 * 1024:
                break
            o -= 16
            offs.append(o)
        offs.reverse()
        out = []
        for i, o in enumerate(offs):
            s, e, v, iv = self._descriptor(o)
            out.append(Check("bg", f"background block {i + 1}  <{hexaddr(o)}>",
                             [(s - BASE, e - BASE)], o + 8, True))
        return out

    def _main_program(self) -> List[Check]:
        d = self.img
        needle = struct.pack("<IIII", BASE, BASE + 0xFBFF, BASE + 0x20000, BASE + 0xF00FF)
        mask = struct.pack("<IIII", MASK32, MASK32, MASK32, 0xFFF700FF)
        hits = masked_find(d.data, needle, mask)
        final = len(d) - 0x20
        if len(hits) != 1:
            self.problems.append(f"main program checksum range table not found ({len(hits)} candidates)")
            return []
        if d.u32(final) != (~d.u32(final + 4) & MASK32):
            self.problems.append(f"main program checksum pair @{hexaddr(final)} is not (v, ~v); "
                                 f"will be rebuilt by --fix")
        self.main_table = hits[0]
        s0, e0, s1, e1 = struct.unpack_from("<IIII", d.data, hits[0])
        return [Check("main", f"main program checksum  (table @{hexaddr(hits[0])})",
                      [(s0 - BASE, e0 - BASE), (s1 - BASE, e1 - BASE)], final, True)]

    def _multipoint(self) -> List[Check]:
        d = self.img
        needle = struct.pack("<II", BASE, BASE + 0x3FFF)
        hits = [o for o in masked_find(d.data, needle, b"\xff" * 8, start=CAL_START)
                if self._desc_ok(o)]
        if len(hits) != 1:
            self.problems.append(f"multipoint table not found ({len(hits)} candidates)")
            return []
        first = hits[0]
        while self._desc_ok(first - 16, allow_boot=True):
            first -= 16
        self.mp_table = first
        out = []
        o = first
        while o + 16 <= len(d):
            s, e, v, iv = self._descriptor(o)
            if s == MASK32:
                break
            if not self._desc_ok(o, allow_boot=True):
                break
            boot = s < BASE
            base = 0 if boot else BASE
            out.append(Check("mp", f"multipoint {len(out) + 1:2d}  <{hexaddr(o)}>",
                             [(s - base, e - base)], o + 8, True, boot=boot,
                             note="boot" if boot else ""))
            o += 16
        self.mp_count = len(out)
        return out

    # -- evaluation --------------------------------------------------------
    def _calc(self, c: Check) -> int:
        d = self.img
        if c.group == "crc":
            s, e = c.ranges[0]
            return crc32(d.data, s, e)
        if c.group == "romsys":
            return sum(d.u16(s) for s, _ in c.ranges) & MASK32
        if c.has_inv and c.inside and not c.inv_ok:
            # pre-correct the inverse so (v + ~v) inside the range is constant
            saved = d.u32(c.store + 4)
            d.w32(c.store + 4, ~d.u32(c.store))
            try:
                return sum(wordsum16(d.data, s, e) for s, e in c.ranges) & MASK32
            finally:
                d.w32(c.store + 4, saved)
        return sum(wordsum16(d.data, s, e) for s, e in c.ranges) & MASK32

    def recompute(self) -> None:
        for c in self.checks:
            c.stored = self.img.u32(c.store)
            c.stored_inv = self.img.u32(c.store + 4) if c.has_inv else 0
            c.calc = self._calc(c)

    def errors(self) -> List[Check]:
        return [c for c in self.checks if not c.ok and not c.boot]

    def fix(self, max_passes: int = 6) -> int:
        """Correct every stored checksum; iterate because code-area checksums depend on
        calibration CRCs etc. Returns number of values rewritten."""
        written = 0
        for _ in range(max_passes):
            self.recompute()
            bad = self.errors()
            if not bad:
                return written
            for c in bad:
                self.img.w32(c.store, c.calc)
                if c.has_inv:
                    self.img.w32(c.store + 4, ~c.calc)
                written += 1
        self.recompute()
        if self.errors():
            raise RuntimeError("checksums did not converge - image structure not understood")
        return written

    def covering(self, off: int) -> List[Check]:
        return [c for c in self.checks if any(s <= off <= e for s, e in c.ranges)]

    def store_ranges(self) -> List[Tuple[int, int, str]]:
        out = []
        for c in self.checks:
            w = 8 if c.has_inv else 4
            out.append((c.store, c.store + w - 1, c.name))
        return out

    def stored_at(self, off: int) -> Optional[Check]:
        for c in self.checks:
            w = 8 if c.has_inv else 4
            if c.store <= off < c.store + w:
                return c
        return None


def print_checks(cm: ChecksumMap, verbose: bool = True) -> int:
    nerr = 0
    group_titles = {"romsys": "ROMSYS", "crc": "Main data CRC32 (calibration)",
                    "bg": "Background checksums", "main": "Main program checksum",
                    "mp": "Multipoint checksums"}
    last = None
    for c in cm.checks:
        if c.group != last:
            print(f"\n[{group_titles[c.group]}]")
            last = c.group
        status = "OK" if c.ok else ("boot (not corrected)" if c.boot else "** NOT OK **")
        if not c.ok and not c.boot:
            nerr += 1
        flags = " (i)" if c.inside else ""
        if verbose or not c.ok or c.group != "mp":
            span = c.span() if c.group != "romsys" or len(c.ranges) <= 2 else f"{len(c.ranges)} words"
            inv = f" inv={c.stored_inv:08X}" if c.has_inv and not c.inv_ok else ""
            print(f"  {c.name:<44s} {span:<24s} @{hexaddr(c.store)} "
                  f"stored={c.stored:08X} calc={c.calc:08X}{inv}{flags}  {status}")
    if not verbose:
        mp = [c for c in cm.checks if c.group == "mp"]
        if mp:
            print(f"  ({len(mp)} multipoint blocks, {sum(1 for c in mp if c.ok)} OK)")
    for p in cm.problems:
        print(f"  ! {p}")
    return nerr


# --------------------------------------------------------------------------
# identification
# --------------------------------------------------------------------------

RE_PART = re.compile(rb"[0-9][A-Z0-9][A-Z0-9]9[0-9]{5}[A-Z]{0,2} {0,3}")
RE_EPK = re.compile(rb"[0-9]{2}/[0-9]/ME7\.[0-9.]+/[0-9]+/[0-9.]+//[A-Za-z0-9]+/[A-Za-z0-9]+//")
RE_HW = re.compile(rb"(026[0-9]{7})(1[0-9]{9})")


def ecu_ids(img: Image) -> Dict[str, Tuple[int, str]]:
    d = bytes(img.data[CAL_START:CAL_END])
    out: Dict[str, Tuple[int, str]] = {}
    m = RE_PART.search(d)
    if m:
        o = CAL_START + m.start()
        out["part_number"] = (o, img.data[o:o + 12].decode("ascii", "replace").strip())
        out["engine_id"] = (o + 12, img.data[o + 12:o + 32].decode("ascii", "replace").strip())
        out["sw_version"] = (o + 32, img.data[o + 32:o + 36].decode("ascii", "replace").strip())
    m = RE_EPK.search(d)
    if m:
        out["epk"] = (CAL_START + m.start(), m.group().decode())
    m = RE_HW.search(d)
    if m:
        out["bosch_hw"] = (CAL_START + m.start(1), m.group(1).decode())
        out["bosch_sw"] = (CAL_START + m.start(2), m.group(2).decode())
    return out


def dpp_setup(img: Image) -> Dict[int, int]:
    """Most frequent `MOV DPPn,#imm` immediates in the code area."""
    out = {}
    code = bytes(img.data)
    for n in range(4):
        c: Counter = Counter()
        for m in re.finditer(re.escape(bytes([0xE6, n])) + b"..", code, re.S):
            v = struct.unpack_from("<H", code, m.start() + 2)[0]
            if v < 0x400:
                c[v] += 1
        if c:
            out[n] = c.most_common(1)[0][0]
    return out


def dpp_address(off: int, dpp: Dict[int, int]) -> Optional[Tuple[int, int]]:
    """(dpp_number, 16-bit address) under which `off` is reachable, or None."""
    page = (BASE + off) >> 14
    for n, p in dpp.items():
        if p == page:
            return n, (n << 14) | (off & 0x3FFF)
    return None


def region_name(off: int) -> str:
    if off < 0x10000:
        return "boot/loader"
    if off < CAL_END:
        return "calibration"
    return "program code"


# --------------------------------------------------------------------------
# diff
# --------------------------------------------------------------------------


def diff_runs(a: bytes, b: bytes) -> List[Tuple[int, int]]:
    runs: List[Tuple[int, int]] = []
    start = None
    for i in range(min(len(a), len(b))):
        if a[i] != b[i]:
            if start is None:
                start = i
        elif start is not None:
            runs.append((start, i - 1))
            start = None
    if start is not None:
        runs.append((start, min(len(a), len(b)) - 1))
    return runs


def classify_run(cm: ChecksumMap, s: int, e: int) -> str:
    """'checksum: <name>' if the run lies entirely inside stored checksum values."""
    names = []
    o = s
    while o <= e:
        c = cm.stored_at(o)
        if c is None:
            return region_name(s)
        names.append(c.name)
        o = c.store + (8 if c.has_inv else 4)
    return "checksum: " + "; ".join(dict.fromkeys(names))


# --------------------------------------------------------------------------
# code cross references
# --------------------------------------------------------------------------

# opcode -> (mnemonic, operand style). Style 'mem' = `op Fn addr16` (reg form),
# 'idx' = `op nm addr16` with Rn destination/source and Rm index register,
# 'imm' = `op Fn imm16` (address used as pointer).
MEM_OPS = {
    0x02: "ADD Rw,mem", 0x03: "ADDB Rb,mem", 0x12: "ADDC Rw,mem", 0x13: "ADDCB Rb,mem",
    0x22: "SUB Rw,mem", 0x23: "SUBB Rb,mem", 0x32: "SUBC Rw,mem", 0x33: "SUBCB Rb,mem",
    0x42: "CMP Rw,mem", 0x43: "CMPB Rb,mem", 0x52: "XOR Rw,mem", 0x53: "XORB Rb,mem",
    0x62: "AND Rw,mem", 0x63: "ANDB Rb,mem", 0x72: "OR Rw,mem", 0x73: "ORB Rb,mem",
    0xC2: "MOVBZ Rw,mem", 0xD2: "MOVBS Rw,mem", 0xF2: "MOV Rw,mem", 0xF3: "MOVB Rb,mem",
    0x04: "ADD mem,Rw", 0x05: "ADDB mem,Rb", 0x24: "SUB mem,Rw", 0x25: "SUBB mem,Rb",
    0x64: "AND mem,Rw", 0x65: "ANDB mem,Rb", 0x74: "OR mem,Rw", 0x75: "ORB mem,Rb",
    0x54: "XOR mem,Rw", 0x55: "XORB mem,Rb", 0xF6: "MOV mem,Rw", 0xF7: "MOVB mem,Rb",
    0x84: "MOV [Rw],mem", 0x94: "MOVB [Rw],mem", 0xC5: "MOVBZ mem,Rb", 0xD5: "MOVBS mem,Rb",
}
IDX_OPS = {0xD4: "MOV Rw,[Rw+#d16]", 0xC4: "MOV [Rw+#d16],Rw",
           0xF4: "MOVB Rb,[Rw+#d16]", 0xE4: "MOVB [Rw+#d16],Rb"}
IMM_OPS = {0xE6: "MOV Rw,#imm16", 0x46: "CMP Rw,#imm16", 0x06: "ADD Rw,#imm16",
           0x26: "SUB Rw,#imm16", 0x66: "AND Rw,#imm16", 0x76: "OR Rw,#imm16"}


def regname(nib: int, byte: bool) -> str:
    if byte:
        return f"R{'L' if nib % 2 == 0 else 'H'}{nib // 2}"
    return f"R{nib}"


def find_xrefs(img: Image, off: int, dpp: Dict[int, int]) -> List[str]:
    res = dpp_address(off, dpp)
    if res is None:
        return []
    _, a16 = res
    pat = struct.pack("<H", a16)
    d = img.data
    hits = []
    i = 0
    while True:
        i = d.find(pat, i)
        if i < 0:
            break
        o = i - 2
        if o >= 0 and i % 2 == 0 and not (CAL_START <= o < CAL_END):
            op, rb = d[o], d[o + 1]
            if op in MEM_OPS and rb >> 4 == 0xF:
                byte = "B" in MEM_OPS[op].split()[0]
                hits.append(f"@{hexaddr(o)}  {MEM_OPS[op]:<16s} reg={regname(rb & 0xF, byte)}")
            elif op in IDX_OPS:
                byte = op in (0xF4, 0xE4)
                hits.append(f"@{hexaddr(o)}  {IDX_OPS[op]:<16s} reg={regname(rb >> 4, byte)} index=R{rb & 0xF}")
            elif op in IMM_OPS and rb >> 4 == 0xF:
                hits.append(f"@{hexaddr(o)}  {IMM_OPS[op]:<16s} R{rb & 0xF}  (pointer to table)")
        i += 1
    return hits


def nearest_table_base(img: Image, off: int, dpp: Dict[int, int], max_back: int = 64) -> Optional[Tuple[int, List[str]]]:
    """For an unreferenced byte, walk backwards to the nearest offset that code references
    (the table/array base it most likely belongs to)."""
    for k in range(1, max_back + 1):
        hits = find_xrefs(img, off - k, dpp)
        if hits:
            return off - k, hits
    return None


# --------------------------------------------------------------------------
# maps - derived from lookup call sites
#
# ME7 passes table pointers in R12..R15 right before a CALLS to a lookup routine.
# Layouts seen in this ROM (counts are exact element counts):
#   group axes   R13 -> [nx][x..][ny][y..]      R12 -> raw data nx*ny   (8/16-bit)
#   embedded 2D  R12 -> [nx][ny][x..][y..][data]                        (8-bit)
#   curve / axis R12 -> [n][x..][data] / [n][x..]
# --------------------------------------------------------------------------


@dataclass
class CallSite:
    offset: int                       # offset of the CALLS instruction
    routine: int                      # linear target address (seg<<16|off) - BASE if in flash
    regs: Dict[int, Tuple[str, int]]  # reg -> ('ptr', file offset) | ('mem', addr16) | ('imm', value)


@dataclass
class MapCandidate:
    data: int                   # file offset of the data cells
    width: int                  # 1 or 2 bytes per cell
    nx: int
    ny: int                     # 0 for a curve
    x_axis: List[int]
    y_axis: List[int]
    axes: Optional[int] = None  # offset of the axes block (None if embedded/raw)
    layout: str = "raw"
    routine: Optional[int] = None
    refs: int = 0

    @property
    def cells(self) -> int:
        return self.nx * max(self.ny, 1)

    @property
    def size(self) -> int:
        return self.cells * self.width

    @property
    def kind(self) -> str:
        return f"{'map' if self.ny else 'curve'}{8 * self.width}"


def monotonic(vals: Sequence[int]) -> bool:
    return len(vals) >= 2 and (all(a < b for a, b in zip(vals, vals[1:]))
                               or all(a > b for a, b in zip(vals, vals[1:])))


def rd_fn(img: Image, width: int):
    d = img.data
    return (lambda p: d[p]) if width == 1 else (lambda p: struct.unpack_from("<H", d, p)[0])


def read_axis(img: Image, o: int, width: int, min_n: int = 2, max_n: int = 32) -> Optional[List[int]]:
    """[n][values] at o -> values if n in range and strictly monotonic."""
    rd = rd_fn(img, width)
    if o + width > len(img):
        return None
    n = rd(o)
    if not (min_n <= n <= max_n) or o + width * (1 + n) > len(img):
        return None
    vals = [rd(o + width * (1 + i)) for i in range(n)]
    return vals if monotonic(vals) else None


def dpp_to_offset(a16: int, dpp: Dict[int, int]) -> Optional[int]:
    n = a16 >> 14
    if n in dpp:
        off = ((dpp[n] << 14) | (a16 & 0x3FFF)) - BASE
        if 0 <= off < FLASH_512K * 2:
            return off
    return None


def find_call_sites(img: Image, dpp: Dict[int, int]) -> List[CallSite]:
    """Every CALLS preceded (within 8 four-byte instructions) by loads into R12..R15."""
    d = img.data
    sites = []
    for m in re.finditer(rb"\xDA[\x00\x80-\x8F]..", bytes(d), re.S):
        o = m.start()
        if o % 2 or CAL_START <= o < CAL_END:
            continue
        seg, off = d[o + 1], struct.unpack_from("<H", d, o + 2)[0]
        routine = ((seg << 16) | off)
        routine = routine - BASE if routine >= BASE else routine
        regs: Dict[int, Tuple[str, int]] = {}
        p = o
        for _ in range(10):
            # 4-byte loads into a register (reg form 0xFn in the second byte)
            if p - 4 >= 0 and d[p - 3] >> 4 == 0xF and d[p - 4] in (0xE6, 0xF2, 0xC2, 0xD2, 0xF3):
                p -= 4
                op, reg, val = d[p], d[p + 1] & 0xF, struct.unpack_from("<H", d, p + 2)[0]
                if reg < 12:
                    continue
                if op == 0xE6:                       # MOV Rn,#imm16
                    po = dpp_to_offset(val, dpp)
                    regs.setdefault(reg, ("ptr", po) if po is not None and CAL_START <= po < CAL_END
                                    else ("imm", val))
                else:                                # MOV/MOVBZ/MOVBS/MOVB Rn,mem
                    regs.setdefault(reg, ("mem", val))
            # 4-byte indexed loads MOV(B) Rn,[Rm+#d16]
            elif p - 4 >= 0 and d[p - 4] in (0xD4, 0xF4):
                p -= 4
                reg = d[p + 1] >> 4
                if reg >= 12:
                    regs.setdefault(reg, ("mem", struct.unpack_from("<H", d, p + 2)[0]))
            # 2-byte register moves / short immediates: MOV(B) Rn,Rm, MOVBZ/MOVBS Rn,Rm, MOV Rn,#data4
            elif p - 2 >= 0 and d[p - 2] in (0xF0, 0xF1, 0xC0, 0xD0, 0xE0, 0xE1):
                p -= 2
                reg = d[p + 1] >> 4
                if reg >= 12:
                    regs.setdefault(reg, ("reg", d[p + 1] & 0xF))
            else:
                break
        if 12 in regs:
            sites.append(CallSite(o, routine, regs))
    return sites


def guess_width(img: Image, data: int, cells: int) -> int:
    """8- vs 16-bit cells: in little-endian 16-bit data the high bytes (odd offsets) are much
    smoother than the low bytes; in 8-bit data both parities are equally rough."""
    d = img.data
    blk = d[data:data + 2 * cells]
    if len(blk) < 8:
        return 1
    even = [blk[i] for i in range(0, len(blk) - 1, 2)]
    odd = [blk[i] for i in range(1, len(blk), 2)]
    rough = lambda v: sum(abs(a - b) for a, b in zip(v, v[1:])) / max(len(v) - 1, 1)
    re_, ro = rough(even), rough(odd)
    return 2 if ro < 0.35 * re_ and re_ > 4 else 1


def axes_block(img: Image, o: int, width: int) -> Optional[Tuple[List[int], List[int]]]:
    xs = read_axis(img, o, width)
    if xs is None:
        return None
    ys = read_axis(img, o + width * (1 + len(xs)), width)
    return xs, (ys or [])


def embedded_2d(img: Image, o: int) -> Optional[Tuple[List[int], List[int]]]:
    d = img.data
    nx, ny = d[o], d[o + 1]
    if not (2 <= nx <= 32 and 2 <= ny <= 32) or o + 2 + nx + ny + nx * ny > len(d):
        return None
    xs, ys = list(d[o + 2:o + 2 + nx]), list(d[o + 2 + nx:o + 2 + nx + ny])
    return (xs, ys) if monotonic(xs) and monotonic(ys) else None


def find_maps(img: Image, dpp: Dict[int, int]) -> Tuple[List[MapCandidate], List[CallSite]]:
    sites = find_call_sites(img, dpp)
    found: Dict[int, MapCandidate] = {}
    for s in sites:
        r12 = s.regs.get(12)
        if not r12 or r12[0] != "ptr":
            continue
        data = r12[1]
        r13 = s.regs.get(13)
        cand: Optional[MapCandidate] = None
        if r13 and r13[0] == "ptr":
            # group axes: R13 -> axes, R12 -> data. Cell width from the gap to the next pointer.
            for width in (1, 2):
                ab = axes_block(img, r13[1], width)
                if ab:
                    xs, ys = ab
                    cells = len(xs) * max(len(ys), 1)
                    cw = guess_width(img, data, cells)
                    cand = MapCandidate(data, cw, len(xs), len(ys), xs, ys, axes=r13[1], layout="group")
                    break
        else:
            e2 = embedded_2d(img, data)
            if e2:
                xs, ys = e2
                cand = MapCandidate(data + 2 + len(xs) + len(ys), 1, len(xs), len(ys), xs, ys,
                                    axes=data, layout="embedded")
            else:
                for width in (1, 2):
                    xs = read_axis(img, data, width)
                    if xs:
                        cand = MapCandidate(data + width * (1 + len(xs)), width, len(xs), 0, xs, [],
                                            axes=data, layout="curve/axis")
                        break
        if cand:
            cand.routine = s.routine
            key = cand.data
            if key in found:
                found[key].refs += 1
            else:
                cand.refs = 1
                found[key] = cand
    return sorted(found.values(), key=lambda c: c.data), sites


def fmt_cell(v: int, width: int, signed: bool, factor: float, offset: float) -> str:
    if signed:
        v = v - (1 << (8 * width)) if v >= 1 << (8 * width - 1) else v
    if factor == 1.0 and offset == 0.0:
        return f"{v:{3 if width == 1 else 5}d}"
    return f"{v * factor + offset:8.3f}"


def print_map(img: Image, c: MapCandidate, signed: bool, factor: float, offset: float,
              xf: float = 1.0, yf: float = 1.0) -> None:
    w = c.width
    rd = rd_fn(img, w)
    ax = f" axes@{hexaddr(c.axes)}" if c.axes is not None else ""
    rt = f" routine=0x{c.routine:05X}" if c.routine is not None else ""
    print(f"{c.kind} data@{hexaddr(c.data)} nx={c.nx} ny={c.ny or '-'} size={c.size} layout={c.layout}{ax}{rt}")
    cw = 4 if (factor == 1.0 and offset == 0.0 and w == 1) else (6 if factor == 1.0 and offset == 0.0 else 9)
    xs = [f"{x * xf:{cw}.{0 if xf == 1 else 2}f}" if xf != 1 else f"{x:{cw}d}" for x in c.x_axis]
    if not c.ny:
        print("  x: " + " ".join(xs))
        print("  y: " + " ".join(fmt_cell(rd(c.data + w * i), w, signed, factor, offset).rjust(cw)
                                 for i in range(c.nx)))
        return
    print(" " * 9 + " ".join(xs))
    for j in range(c.ny):
        y = c.y_axis[j]
        ylab = f"{y * yf:8.2f}" if yf != 1 else f"{y:8d}"
        row = [fmt_cell(rd(c.data + w * (j * c.nx + i)), w, signed, factor, offset).rjust(cw)
               for i in range(c.nx)]
        print(f"{ylab} " + " ".join(row))


# --------------------------------------------------------------------------
# XDF definitions (TunerPro) - optional, used to name and scale tables
# --------------------------------------------------------------------------

RE_XDF_TABLE = re.compile(r"<XDFTABLE\b[^>]*>(.*?)</XDFTABLE>", re.S)
RE_XDF_AXIS = re.compile(r"<XDFAXIS\b([^>]*)>(.*?)</XDFAXIS>", re.S)
RE_XDF_EMB = re.compile(r"<EMBEDDEDDATA([^>]*?)/?>")
RE_XDF_ATTR = re.compile(r'(\w+)="([^"]*)"')
RE_XDF_MATH = re.compile(r'<MATH\s+equation="([^"]*)"')
RE_LINEAR = re.compile(r"^(?:(?P<a>-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\*)?X(?:\+(?P<b>-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?))?$")


@dataclass
class XdfAxis:
    address: Optional[int]
    count: int
    bits: int
    rows: int
    cols: int
    factor: float
    offset: float
    units: str
    labels: List[str] = field(default_factory=list)


@dataclass
class XdfTable:
    title: str
    description: str
    category: str
    address: int
    rows: int
    cols: int
    bits: int
    signed: bool
    factor: float
    offset: float
    units: str
    x: Optional[XdfAxis] = None
    y: Optional[XdfAxis] = None
    raw_equation: str = ""


def _xdf_text(s: str, tag: str) -> str:
    m = re.search(rf"<{tag}>(.*?)</{tag}>", s, re.S)
    if not m:
        return ""
    return re.sub(r"&#(\d+);", lambda k: chr(int(k.group(1))), m.group(1)).replace("&lt;", "<").replace("&gt;", ">").replace("&amp;", "&").strip()


def _xdf_int(s: str) -> int:
    return int(s, 0)


def xdf_linear(equation: str) -> Optional[Tuple[float, float]]:
    """Parse 'a*X+b' style TunerPro equations into (factor, offset); None if not linear."""
    eq = re.sub(r"\s+", "", equation or "X")
    m = RE_LINEAR.match(eq)
    if not m:
        return None
    return float(m.group("a") or 1.0), float(m.group("b") or 0.0)


def _xdf_axis_from_block(attrs: str, body: str) -> Optional[XdfAxis]:
    emb = RE_XDF_EMB.search(body)
    ea = dict(RE_XDF_ATTR.findall(emb.group(1))) if emb else {}
    addr = _xdf_int(ea["mmedaddress"]) if "mmedaddress" in ea else None
    eq = RE_XDF_MATH.search(body)
    lin = xdf_linear(eq.group(1)) if eq else (1.0, 0.0)
    if lin is None:
        lin = (1.0, 0.0)
    count_m = re.search(r"<indexcount>(\d+)</indexcount>", body)
    labels = re.findall(r'<LABEL index="\d+" value="([^"]*)"', body)
    rows = _xdf_int(ea["mmedrowcount"]) if "mmedrowcount" in ea else 1
    cols = _xdf_int(ea["mmedcolcount"]) if "mmedcolcount" in ea else 1
    count = int(count_m.group(1)) if count_m else rows * cols
    return XdfAxis(addr, count, int(ea.get("mmedelementsizebits", "8")), rows, cols,
                   lin[0], lin[1], _xdf_text(body, "units"), [l for l in labels if l])


RE_XDF_CONST = re.compile(r"<XDFCONSTANT\b[^>]*>(.*?)</XDFCONSTANT>", re.S)


def _load_xdf_constants(text: str) -> List[XdfTable]:
    out: List[XdfTable] = []
    for m in RE_XDF_CONST.finditer(text):
        body = m.group(1)
        emb = RE_XDF_EMB.search(body)
        if not emb:
            continue
        ea = dict(RE_XDF_ATTR.findall(emb.group(1)))
        if "mmedaddress" not in ea:
            continue
        eq = RE_XDF_MATH.search(body)
        raw_eq = eq.group(1) if eq else "X"
        lin = xdf_linear(raw_eq) or (1.0, 0.0)
        out.append(XdfTable(
            title=_xdf_text(body, "title"), description=_xdf_text(body, "description"),
            category="", address=_xdf_int(ea["mmedaddress"]), rows=1, cols=1,
            bits=int(ea.get("mmedelementsizebits", "8")), signed=False,
            factor=lin[0], offset=lin[1], units=_xdf_text(body, "units"), raw_equation=raw_eq))
    return out


def load_xdf(path: str) -> List[XdfTable]:
    text = Path(path).read_text(encoding="latin-1")
    tables: List[XdfTable] = _load_xdf_constants(text)
    for m in RE_XDF_TABLE.finditer(text):
        body = m.group(1)
        title = _xdf_text(body, "title")
        desc = _xdf_text(body, "description")
        cat_m = re.search(r'<CATEGORYMEM[^>]*category="(\d+)"', body)
        axes = {}
        z = None
        for am in RE_XDF_AXIS.finditer(body):
            aid = dict(RE_XDF_ATTR.findall(am.group(1))).get("id", "")
            if aid == "z":
                z = am.group(2)
            elif aid in ("x", "y"):
                axes[aid] = _xdf_axis_from_block(am.group(1), am.group(2))
        if z is None:
            continue
        emb = RE_XDF_EMB.search(z)
        if not emb:
            continue
        ea = dict(RE_XDF_ATTR.findall(emb.group(1)))
        if "mmedaddress" not in ea:
            continue
        eq = RE_XDF_MATH.search(z)
        raw_eq = eq.group(1) if eq else "X"
        lin = xdf_linear(raw_eq) or (1.0, 0.0)
        rows = _xdf_int(ea["mmedrowcount"]) if "mmedrowcount" in ea else 1
        cols = _xdf_int(ea["mmedcolcount"]) if "mmedcolcount" in ea else 1
        flags = _xdf_int(ea["mmedtypeflags"]) if "mmedtypeflags" in ea else 0
        tables.append(XdfTable(
            title=title, description=desc, category=cat_m.group(1) if cat_m else "",
            address=_xdf_int(ea["mmedaddress"]), rows=rows, cols=cols,
            bits=int(ea.get("mmedelementsizebits", "8")), signed=bool(flags & 0x01),
            factor=lin[0], offset=lin[1], units=_xdf_text(z, "units"),
            x=axes.get("x"), y=axes.get("y"), raw_equation=raw_eq))
    return tables


def find_xdf(img_path: str, part: Optional[str]) -> Optional[str]:
    """Default definition lookup: ECU/Profiles/*.xdf next to the image; prefer the one named after the part."""
    folder = Path(img_path).resolve().parent / "Profiles"
    if not folder.is_dir():
        return None
    cands = sorted(folder.glob("*.xdf"))
    if part:
        for c in cands:
            if c.stem.upper().startswith(part.upper()):
                return str(c)
    return str(cands[0]) if cands else None


def xdf_by_title(tables: Sequence[XdfTable]) -> Dict[str, XdfTable]:
    return {t.title: t for t in tables}


def read_cells(img: Image, addr: int, n: int, bits: int, signed: bool, factor: float, offset: float) -> List[float]:
    w = max(1, bits // 8)
    out: List[float] = []
    for i in range(n):
        o = addr + i * w
        if o + w > len(img.data):
            break
        v = int.from_bytes(img.data[o:o + w], "little")
        if signed and v >= 1 << (8 * w - 1):
            v -= 1 << (8 * w)
        out.append(v * factor + offset)
    return out


# Overrun fuel cut (Schubabschaltung) and related settings. Titles are the
# XDF table names; anything missing from the definition is reported, not guessed.
SETTINGS_OVERRUN = [
    "TKATSA", "TATMSA", "DNSLL", "DNVSA", "NLRSHB", "KFTVSA", "TVSATM",
    "RLMNSAN", "REDMXSA", "TDMFBSA", "NGDNSA", "VSAA", "DVSAVO", "DVSAVU",
]

# Secondary-air (SAI) related bit array, located by address from the README
# (0x14EE4, 7 bytes). Known patch states are matched by content, not by name.
SAI_ARRAY_ADDR = 0x14EE4
SAI_ARRAY_LEN = 7
SAI_STOCK = bytes.fromhex("ffff00c0fff833")
SAI_OFF_TAIL = bytes.fromhex("fb3f")   # elements 5-6 after the XTuner SAI-off patch


def sai_state(img: Image) -> str:
    cells = bytes(img.data[SAI_ARRAY_ADDR:SAI_ARRAY_ADDR + SAI_ARRAY_LEN])
    if cells == SAI_STOCK:
        return "stock"
    if cells[5:7] == SAI_OFF_TAIL and cells[:5] == SAI_STOCK[:5]:
        return "SAI-off patch applied (matches XTuner signature)"
    return "modified/unknown"


LABELS_EN_PATH = Path(__file__).resolve().parent / "xdf_labels_en.json"


def load_labels_en(path: Path = LABELS_EN_PATH) -> Tuple[Dict[str, str], Dict[str, str]]:
    """(title -> English label, German unit -> English unit) from xdf_labels_en.json."""
    import json
    if not path.exists():
        return {}, {}
    data = json.loads(path.read_text(encoding="utf-8"))
    labels = {k: v["en"] for k, v in data.get("labels", {}).items() if v.get("en")}
    return labels, dict(data.get("units", {}))


def label_en(t: XdfTable, labels: Dict[str, str]) -> str:
    """English label for a table; falls back to the German XDF description if untranslated."""
    return labels.get(t.title, t.description)


def unit_en(u: str, units: Dict[str, str]) -> str:
    return units.get(u, u)


def print_xdf_table(img: Image, t: XdfTable, labels: Optional[Dict[str, str]] = None,
                    units: Optional[Dict[str, str]] = None) -> None:
    """Print a table with its XDF axes, values scaled by the XDF equation."""
    if labels is None or units is None:
        labels, units = load_labels_en()
    text = label_en(t, labels)
    u = unit_en(t.units, units)
    n = t.rows * t.cols
    vals = read_cells(img, t.address, n, t.bits, t.signed, t.factor, t.offset)
    if t.x is None and t.y is None and n == 1:
        w = max(1, t.bits // 8)
        raw = int.from_bytes(img.data[t.address:t.address + w], "little")
        print(f"{t.title:<8s} @{hexaddr(t.address)} = {vals[0]:g} {u}  (raw 0x{raw:0{2 * w}X})  [{text}]")
        return
    print(f"{t.title}  @{hexaddr(t.address)}  {t.rows}x{t.cols}  {t.bits}-bit  units={u or '-'}  [{text}]")
    if t.rows == 1:
        xs = _axis_values(img, t.x, t.cols) if t.x else []
        if xs and len(xs) == len(vals):
            print("  " + "  ".join(f"{x:>7.1f}" for x in xs))
        print("  " + "  ".join(f"{v:>7.2f}" for v in vals))
        return
    xs = _axis_values(img, t.x, t.cols) if t.x else [None] * t.cols
    ys = _axis_values(img, t.y, t.rows) if t.y else [None] * t.rows
    head = " " * 10 + " ".join((f"{x:>7.1f}" if x is not None else f"{'col'+str(i):>7}") for i, x in enumerate(xs))
    print(head)
    for r in range(t.rows):
        ylab = f"{ys[r]:>8.1f}" if ys[r] is not None else f"{'row'+str(r):>8}"
        row = vals[r * t.cols:(r + 1) * t.cols]
        print(f"{ylab} " + " ".join(f"{v:>7.2f}" for v in row))


def _axis_values(img: Image, ax: Optional[XdfAxis], n: int) -> List[float]:
    if ax is None:
        return []
    if ax.address is None:
        return [float(i) for i in range(n)]
    return read_cells(img, ax.address, min(n, ax.count), ax.bits, False, ax.factor, ax.offset)


def print_settings(img: Image, tables: Sequence[XdfTable], listed: Sequence[str] = SETTINGS_OVERRUN) -> None:
    by = xdf_by_title(tables)
    labels, units = load_labels_en()
    print(f"\nsettings  : overrun / SAI-related values (from XDF, read-only; labels in English)")
    print(f"sai state : {sai_state(img)}")
    print(f"            ESKONF @{hexaddr(SAI_ARRAY_ADDR)} = " + " ".join(f"{b:02x}" for b in img.data[SAI_ARRAY_ADDR:SAI_ARRAY_ADDR + SAI_ARRAY_LEN]))
    missing = []
    for title in listed:
        t = by.get(title)
        if t is None:
            missing.append(title)
            continue
        print()
        print_xdf_table(img, t, labels, units)
    if missing:
        print(f"\nnot in definition: {', '.join(missing)}")


# --------------------------------------------------------------------------
# commands
# --------------------------------------------------------------------------


def cmd_info(a: argparse.Namespace) -> int:
    img = Image.load(a.file)
    ids = ecu_ids(img)
    print(f"file      : {img.path}  ({len(img)} bytes, md5 {img.md5()})")
    for k, (o, v) in ids.items():
        print(f"{k:<10s}: {v!r:<40s} @{hexaddr(o)}")
    dpp = dpp_setup(img)
    print("dpp       : " + "  ".join(f"DPP{n}=0x{p:03X} (C167 0x{p << 14:06X})" for n, p in sorted(dpp.items())))
    cm = ChecksumMap(img)
    nerr = print_checks(cm, verbose=False)
    print(f"\nchecksums : {len(cm.checks)} found, {nerr} error(s)")
    if not a.no_settings:
        part = ids["part_number"][1] if "part_number" in ids else None
        xdf_path = a.xdf or ("" if a.no_xdf else find_xdf(a.file, part) or "")
        if xdf_path:
            tables = load_xdf(xdf_path)
            print(f"\ndefinition: {xdf_path} ({len(tables)} tables)")
            stem = Path(xdf_path).stem.upper()
            if part and not stem.startswith(part.upper()):
                print(f"warning   : definition name '{Path(xdf_path).name}' does not match part {part}")
            print_settings(img, tables)
        else:
            print("\nsettings  : no XDF definition found (use --xdf PATH)")
    return 1 if nerr else 0


def cmd_checksums(a: argparse.Namespace) -> int:
    img = Image.load(a.file)
    cm = ChecksumMap(img)
    nerr = print_checks(cm, verbose=True)
    print(f"\n{len(cm.checks)} checksums, {nerr} error(s)")
    if a.fix:
        if not a.out and not a.in_place:
            sys.exit("--fix needs --out FILE (or --in-place)")
        n = cm.fix()
        print(f"corrected {n} stored value(s)")
        img.save(a.out or a.file, overwrite_ok=a.in_place)
        print(f"wrote {a.out or a.file}  md5 {img.md5()}")
        return 0
    return 1 if nerr else 0


def cmd_fix(a: argparse.Namespace) -> int:
    a.fix = True
    return cmd_checksums(a)


def cmd_diff(a: argparse.Namespace) -> int:
    x, y = Image.load(a.a), Image.load(a.b)
    print(f"A: {x.path}  md5 {x.md5()}")
    print(f"B: {y.path}  md5 {y.md5()}")
    runs = diff_runs(x.data, y.data)
    if not runs:
        print("identical")
        return 0
    cm = ChecksumMap(x)
    total = sum(e - s + 1 for s, e in runs)
    print(f"{len(runs)} run(s), {total} byte(s) differ\n")
    real = 0
    for s, e in runs:
        kind = classify_run(cm, s, e)
        if not kind.startswith("checksum"):
            real += 1
        n = e - s + 1
        if n <= a.max_bytes:
            av, bv = x.data[s:e + 1].hex(" "), y.data[s:e + 1].hex(" ")
            print(f"{hexaddr(s)}-{hexaddr(e)} [{n:5d}] {kind}\n    A: {av}\n    B: {bv}")
        else:
            print(f"{hexaddr(s)}-{hexaddr(e)} [{n:5d}] {kind}  (use hexdump to view)")
        if a.xref and kind == "calibration":
            dpp = dpp_setup(x)
            for o in range(s, e + 1):
                hits = find_xrefs(x, o, dpp)
                for h in hits:
                    print(f"      {hexaddr(o)} <- {h}")
                if not hits:
                    nb = nearest_table_base(x, o, dpp)
                    if nb:
                        print(f"      {hexaddr(o)} no direct reference; nearest referenced base "
                              f"{hexaddr(nb[0])} (+{o - nb[0]}), {len(nb[1])} ref(s) e.g. {nb[1][0]}")
    print(f"\n{real} run(s) are data changes, {len(runs) - real} are checksum corrections")
    return 0


def cmd_apply_diff(a: argparse.Namespace) -> int:
    base, mod, tgt = Image.load(a.base), Image.load(a.mod), Image.load(a.target)
    cm_base = ChecksumMap(base)
    runs = diff_runs(base.data, mod.data)
    applied, skipped, conflicts = 0, 0, []
    for s, e in runs:
        kind = classify_run(cm_base, s, e)
        if kind.startswith("checksum") and not a.keep_checksums:
            skipped += 1
            continue
        if tgt.data[s:e + 1] != base.data[s:e + 1]:
            conflicts.append((s, e))
            if not a.force:
                continue
        tgt.data[s:e + 1] = mod.data[s:e + 1]
        applied += 1
        print(f"applied {hexaddr(s)}-{hexaddr(e)} [{e - s + 1}] {kind}: "
              f"{base.data[s:e + 1].hex(' ')} -> {mod.data[s:e + 1].hex(' ')}")
    for s, e in conflicts:
        print(f"{'FORCED' if a.force else 'CONFLICT (skipped)'} {hexaddr(s)}-{hexaddr(e)}: "
              f"target differs from base here ({tgt.data[s:e + 1].hex(' ')} vs {base.data[s:e + 1].hex(' ')})")
    print(f"{applied} run(s) applied, {skipped} checksum run(s) skipped, {len(conflicts)} conflict(s)")
    if conflicts and not a.force:
        sys.exit("aborting because of conflicts (use --force to overwrite)")
    if not a.no_fix:
        n = ChecksumMap(tgt).fix()
        print(f"checksums: corrected {n} stored value(s)")
    tgt.save(a.out)
    print(f"wrote {a.out}  md5 {tgt.md5()}")
    return 0


def cmd_patch(a: argparse.Namespace) -> int:
    img = Image.load(a.file)
    blob = bytes.fromhex(a.hex.replace(" ", ""))
    if a.at + len(blob) > len(img):
        sys.exit("patch runs past end of image")
    old = bytes(img.data[a.at:a.at + len(blob)])
    img.data[a.at:a.at + len(blob)] = blob
    print(f"{hexaddr(a.at)}: {old.hex(' ')} -> {blob.hex(' ')}  ({region_name(a.at)})")
    if not a.no_fix:
        n = ChecksumMap(img).fix()
        print(f"checksums: corrected {n} stored value(s)")
    img.save(a.out, overwrite_ok=a.in_place)
    print(f"wrote {a.out}  md5 {img.md5()}")
    return 0


def cmd_where(a: argparse.Namespace) -> int:
    img = Image.load(a.file)
    cm = ChecksumMap(img)
    dpp = dpp_setup(img)
    for off in a.addr:
        print(f"{hexaddr(off)}  C167 0x{BASE + off:06X}  region={region_name(off)}  byte=0x{img.u8(off):02X}")
        r = dpp_address(off, dpp)
        if r:
            print(f"    DPP{r[0]} address 0x{r[1]:04X}")
        st = cm.stored_at(off)
        if st:
            print(f"    stored checksum value: {st.name}")
        for c in cm.covering(off):
            print(f"    covered by: {c.name}")
        if cm.mp_table and cm.mp_table <= off < cm.mp_table + 16 * cm.mp_count:
            print("    inside multipoint descriptor table")
    return 0


def cmd_regions(a: argparse.Namespace) -> int:
    img = Image.load(a.file)
    cm = ChecksumMap(img)
    print("fixed layout")
    print("  0x00000-0x0FFFF  boot / loader (ROMSYS header @0x08000)")
    print(f"  {hexaddr(CAL_START)}-{hexaddr(CAL_END - 1)}  calibration")
    print(f"  {hexaddr(CAL_END)}-{hexaddr(len(img) - 1)}  program code")
    print("\nchecksum coverage")
    for c in cm.checks:
        if c.group == "mp" and not a.all:
            continue
        print(f"  {c.name:<44s} {c.span() if len(c.ranges) <= 2 else str(len(c.ranges)) + ' words':<24s} -> @{hexaddr(c.store)}")
    if not a.all:
        print(f"  multipoint: {cm.mp_count} blocks, table @{hexaddr(cm.mp_table or 0)} (use --all)")
    # uncovered holes inside the calibration
    print("\ncalibration bytes NOT covered by any CRC32 zone:")
    covered = [(s, e) for s, e, _ in cm.crc_blocks]
    cur = CAL_START
    for s, e in sorted(covered):
        if s > cur:
            print(f"  {hexaddr(cur)}-{hexaddr(s - 1)}  ({s - cur} bytes)")
        cur = e + 1
    if cur < CAL_END:
        print(f"  {hexaddr(cur)}-{hexaddr(CAL_END - 1)}  ({CAL_END - cur} bytes)")
    return 0


def cmd_xref(a: argparse.Namespace) -> int:
    img = Image.load(a.file)
    dpp = dpp_setup(img)
    end = a.end if a.end is not None else a.addr
    for off in range(a.addr, end + 1):
        hits = find_xrefs(img, off, dpp)
        r = dpp_address(off, dpp)
        if hits or not a.quiet:
            lab = f"DPP{r[0]} 0x{r[1]:04X}" if r else "not DPP-addressable"
            print(f"{hexaddr(off)} (= {lab}, byte 0x{img.u8(off):02X}): {len(hits)} ref(s)")
        for h in hits:
            print(f"    {h}")
        if not hits and r and not a.quiet:
            nb = nearest_table_base(img, off, dpp)
            if nb:
                print(f"    no direct reference; nearest referenced base {hexaddr(nb[0])} (+{off - nb[0]})")
    return 0


def cmd_maps(a: argparse.Namespace) -> int:
    img = Image.load(a.file)
    dpp = dpp_setup(img)
    cands, sites = find_maps(img, dpp)
    if a.min_cells:
        cands = [c for c in cands if c.cells >= a.min_cells]
    if a.routine is not None:
        cands = [c for c in cands if c.routine == a.routine]
    by_routine = Counter(s.routine for s in sites)
    print(f"{len(sites)} lookup call sites, {len(by_routine)} routines, {len(cands)} table(s) listed")
    if a.routines:
        for r, n in by_routine.most_common():
            kinds = Counter(c.layout for c in cands if c.routine == r)
            print(f"  routine 0x{r:05X}: {n:3d} calls  {dict(kinds)}")
        return 0
    for c in cands:
        dims = f"{c.nx}x{c.ny}" if c.ny else f"{c.nx}"
        xs = " ".join(f"{v}" for v in c.x_axis[:6]) + (" .." if c.nx > 6 else "")
        ax = f"axes@{hexaddr(c.axes)}" if c.axes is not None else " " * 12
        print(f"  data@{hexaddr(c.data)} {c.kind:<7s} {dims:<6s} {ax} {c.layout:<10s} "
              f"rt=0x{c.routine:05X} refs={c.refs:<2d} x=[{xs}]")
    return 0


def cmd_map(a: argparse.Namespace) -> int:
    img = Image.load(a.file)
    width = 2 if a.word else 1
    c: Optional[MapCandidate] = None
    if a.nx:                                   # fully manual: addr = data start
        xs, ys = list(range(a.nx)), list(range(a.ny or 0))
        if a.axes is not None:
            ab = axes_block(img, a.axes, width)
            if ab:
                xs, ys = ab[0], ab[1] if a.ny else []
        c = MapCandidate(a.addr, width, a.nx, a.ny or 0, xs, ys, axes=a.axes, layout="manual")
    elif a.axes is not None:                   # data at addr, axes block given
        ab = axes_block(img, a.axes, width)
        if not ab:
            sys.exit(f"no [n][values] axes block at {hexaddr(a.axes)}")
        c = MapCandidate(a.addr, width, len(ab[0]), len(ab[1]), ab[0], ab[1], axes=a.axes, layout="group")
    else:                                      # resolve through the lookup call sites
        dpp = dpp_setup(img)
        cands, _ = find_maps(img, dpp)
        for k in cands:
            if k.data == a.addr or k.axes == a.addr:
                c = k
                break
        if c is None:
            e2 = embedded_2d(img, a.addr)
            if e2:
                c = MapCandidate(a.addr + 2 + len(e2[0]) + len(e2[1]), 1, len(e2[0]), len(e2[1]), e2[0], e2[1],
                                 axes=a.addr, layout="embedded")
        if c is None:
            xs = read_axis(img, a.addr, width)
            if xs:
                c = MapCandidate(a.addr + width * (1 + len(xs)), width, len(xs), 0, xs, [],
                                 axes=a.addr, layout="curve/axis")
        if c is None:
            sys.exit("not a known table; give --nx/--ny (data at addr) or --axes AXES_ADDR")
        if a.word and c.width == 1:
            c.width = 2
    print_map(img, c, a.signed, a.factor, a.offset, a.xf, a.yf)
    return 0


def cmd_hexdump(a: argparse.Namespace) -> int:
    img = Image.load(a.file)
    end = min(a.end, len(img))
    for o in range(a.start, end, 16):
        row = img.data[o:min(o + 16, end)]
        asc = "".join(chr(c) if 32 <= c < 127 else "." for c in row)
        print(f"{o:05X}: {row.hex(' '):<47s}  {asc}")
    return 0


def cmd_strings(a: argparse.Namespace) -> int:
    img = Image.load(a.file)
    for m in re.finditer(rb"[\x20-\x7e]{%d,}" % a.min, bytes(img.data)):
        print(f"{m.start():05X}: {m.group().decode()}")
    return 0


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("info", help="identify image, checksums and current overrun/SAI settings")
    s.add_argument("file")
    s.add_argument("--xdf", help="TunerPro definition to name/scale tables (default: ECU/Profiles/<part>*.xdf)")
    s.add_argument("--no-xdf", action="store_true", help="skip definition lookup")
    s.add_argument("--no-settings", action="store_true", help="omit the settings section")
    s.set_defaults(fn=cmd_info)

    for name, fn in (("checksums", cmd_checksums), ("fix", cmd_fix)):
        s = sub.add_parser(name, help="verify (and with --fix correct) every checksum")
        s.add_argument("file")
        s.add_argument("--fix", action="store_true", default=(name == "fix"))
        s.add_argument("--out")
        s.add_argument("--in-place", action="store_true")
        s.set_defaults(fn=fn)

    s = sub.add_parser("diff", help="annotated byte diff of two images")
    s.add_argument("a")
    s.add_argument("b")
    s.add_argument("--max-bytes", type=int, default=64, help="print hex for runs up to this length")
    s.add_argument("--xref", action="store_true", help="list code references for changed calibration bytes")
    s.set_defaults(fn=cmd_diff)

    s = sub.add_parser("apply-diff", help="port the changes BASE->MOD onto TARGET")
    s.add_argument("base")
    s.add_argument("mod")
    s.add_argument("target")
    s.add_argument("--out", required=True)
    s.add_argument("--keep-checksums", action="store_true", help="also copy stored checksum bytes")
    s.add_argument("--force", action="store_true", help="apply even where TARGET already differs from BASE")
    s.add_argument("--no-fix", action="store_true")
    s.set_defaults(fn=cmd_apply_diff)

    s = sub.add_parser("patch", help="write raw bytes")
    s.add_argument("file")
    s.add_argument("--out", required=True)
    s.add_argument("--at", type=parse_int, required=True)
    s.add_argument("--hex", required=True)
    s.add_argument("--no-fix", action="store_true")
    s.add_argument("--in-place", action="store_true")
    s.set_defaults(fn=cmd_patch)

    s = sub.add_parser("where", help="describe offset(s)")
    s.add_argument("file")
    s.add_argument("addr", type=parse_int, nargs="+")
    s.set_defaults(fn=cmd_where)

    s = sub.add_parser("regions", help="region and checksum map")
    s.add_argument("file")
    s.add_argument("--all", action="store_true")
    s.set_defaults(fn=cmd_regions)

    s = sub.add_parser("xref", help="code references to calibration offset(s)")
    s.add_argument("file")
    s.add_argument("addr", type=parse_int)
    s.add_argument("--end", type=parse_int)
    s.add_argument("--quiet", action="store_true", help="only print offsets that have references")
    s.set_defaults(fn=cmd_xref)

    s = sub.add_parser("maps", help="list map/curve candidates")
    s.add_argument("file")
    s.add_argument("--min-cells", type=int, default=0, help="only tables with at least this many cells")
    s.add_argument("--routine", type=parse_int, help="only tables used by this lookup routine")
    s.add_argument("--routines", action="store_true", help="summarise lookup routines instead")
    s.set_defaults(fn=cmd_maps)

    s = sub.add_parser("map", help="print a table")
    s.add_argument("file")
    s.add_argument("addr", type=parse_int)
    s.add_argument("--nx", type=int, help="manual: addr is the data start with nx columns")
    s.add_argument("--ny", type=int)
    s.add_argument("--axes", type=parse_int, help="offset of a [nx][x..][ny][y..] axes block")
    s.add_argument("--word", action="store_true", help="16-bit cells/axes")
    s.add_argument("--signed", action="store_true")
    s.add_argument("--factor", type=float, default=1.0)
    s.add_argument("--offset", type=float, default=0.0)
    s.add_argument("--xf", type=float, default=1.0, help="x axis factor")
    s.add_argument("--yf", type=float, default=1.0, help="y axis factor")
    s.set_defaults(fn=cmd_map)

    s = sub.add_parser("hexdump")
    s.add_argument("file")
    s.add_argument("--start", type=parse_int, default=0)
    s.add_argument("--end", type=parse_int, default=0x100)
    s.set_defaults(fn=cmd_hexdump)

    s = sub.add_parser("strings")
    s.add_argument("file")
    s.add_argument("--min", type=int, default=6)
    s.set_defaults(fn=cmd_strings)
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    a = build_parser().parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
