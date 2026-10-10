#!/usr/bin/env python3
"""Regression tests for eeprom_tool.py against the dumps in EEPROM/.
Run:  python3 -m unittest tools/test_eeprom_tool.py -v
"""
import io
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import eeprom_tool as et  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
D919 = ROOT / "EEPROM" / "TT_919_orig_252622km.bin"
D920 = ROOT / "EEPROM" / "GB_920_dump.bin"


def run(*argv) -> str:
    buf = io.StringIO()
    with redirect_stdout(buf):
        rc = et.main([str(a) for a in argv])
    return rc, buf.getvalue()


class Decode(unittest.TestCase):
    def test_919(self):
        d = et.Dump.load(str(D919))
        self.assertEqual(et.generation(d), "919")
        r = et.analyse_ring(d)
        self.assertEqual(r.km, 252622)
        self.assertTrue(r.complement_ok and r.sequence_ok, r.problems)
        self.assertEqual(r.newest_index, 11)

    def test_920(self):
        d = et.Dump.load(str(D920))
        self.assertEqual(et.generation(d), "920")
        r = et.analyse_ring(d)
        self.assertEqual(r.km, 233031)
        self.assertTrue(r.complement_ok and r.sequence_ok, r.problems)
        self.assertEqual([v for _, v in et.find_immo_ids(d.eeprom(), et.EEPROM_START)],
                         ["AUZ5Z0B3245205"] * 3)
        self.assertEqual([a for a, _ in et.find_vins(d.eeprom(), et.EEPROM_START)], et.VIN_SLOTS)

    def test_verify_both_ok(self):
        for p in (D919, D920):
            rc, out = run("verify", p)
            self.assertEqual(rc, 0, out)


class Modify(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.w = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_extract_inject_roundtrip(self):
        ee = self.w / "ee.bin"
        rt = self.w / "rt.bin"
        self.assertEqual(run("extract", D920, ee)[0], 0)
        self.assertEqual(ee.stat().st_size, 2048)
        self.assertEqual(et.Dump.load(str(ee)).kind, "2 KB EEPROM-only dump")
        self.assertEqual(run("inject", D920, ee, rt)[0], 0)
        self.assertEqual(rt.read_bytes(), D920.read_bytes())

    def test_set_odometer_only_touches_ring(self):
        out = self.w / "o.bin"
        rc, _ = run("set-odometer", D920, "--out", out, "--from-dump", D919)
        self.assertEqual(rc, 0)
        a, b = D920.read_bytes(), out.read_bytes()
        changed = {i for i in range(len(a)) if a[i] != b[i]}
        lo, hi = et.ODO_RING_INV, et.ODO_RING + et.ODO_RING_LEN
        self.assertTrue(all(lo <= i < hi for i in changed))
        d = et.Dump.load(str(out))
        r = et.analyse_ring(d)
        self.assertEqual(r.km, 252622)
        self.assertEqual(r.newest_index, 6)  # ring pointer preserved
        self.assertTrue(r.complement_ok and r.sequence_ok)
        self.assertEqual(run("verify", out)[0], 0)

    def test_set_odometer_on_2k_with_parity(self):
        ee = self.w / "ee.bin"
        run("extract", D920, ee)
        out = self.w / "o.bin"
        rc, _ = run("set-odometer", ee, "--out", out, "--km", "100000", "--match-parity")
        self.assertEqual(rc, 0)
        self.assertEqual(et.analyse_ring(et.Dump.load(str(out))).km, 100001)

    def test_set_vin(self):
        out = self.w / "v.bin"
        rc, _ = run("set-vin", D920, "--out", out, "--vin", "TRUZZZ8N1X1000001")
        self.assertEqual(rc, 0)
        d = et.Dump.load(str(out))
        for s in et.VIN_SLOTS:
            self.assertEqual(d[s:s + 17], b"TRUZZZ8N1X1000001")

    def test_refuse_overwrite(self):
        src = self.w / "src.bin"
        src.write_bytes(D919.read_bytes())
        with self.assertRaises(SystemExit):
            run("set-odometer", src, "--out", src, "--km", "1000")

    def test_build_ring_roundtrip(self):
        for km, idx in ((252622, 11), (233031, 6), (70, 0)):
            plain, inv = et.build_ring(km, idx)
            self.assertEqual(bytes(b ^ 0xFF for b in plain), inv)
            vals = [int.from_bytes(plain[4 * i:4 * i + 4], "big") for i in range(36)]
            self.assertEqual(max(vals), km)
            self.assertEqual(vals.index(km), idx)


if __name__ == "__main__":
    unittest.main()
