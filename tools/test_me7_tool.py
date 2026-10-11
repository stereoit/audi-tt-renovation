#!/usr/bin/env python3
"""Regression tests for me7_tool.py against the images in ECU/.

Run:  python3 -m unittest tools/test_me7_tool.py -v
"""
import hashlib
import io
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import me7_tool as T  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
ORIG = ROOT / "ECU" / "8N0906018AE 0001.Bin"
SAIOFF = ROOT / "ECU" / "8N0906018AE_0001_MOD_XTUNER_SAIOFF_done.Bin"
ORIG_MD5 = "7602ca25143f717429c5bd8d151a1361"
SAIOFF_MD5 = "f0cabb1d52be19665bfbf3b26e7934af"


def run(*argv):
    buf = io.StringIO()
    with redirect_stdout(buf):
        rc = T.main([str(a) for a in argv])
    return rc, buf.getvalue()


@unittest.skipUnless(ORIG.exists() and SAIOFF.exists(), "ECU images missing")
class Identify(unittest.TestCase):
    def test_ids(self):
        img = T.Image.load(str(ORIG))
        ids = {k: v for k, (_, v) in T.ecu_ids(img).items()}
        self.assertEqual(ids["part_number"], "8N0906018AE")
        self.assertEqual(ids["sw_version"], "0001")
        self.assertEqual(ids["bosch_hw"], "0261207030")
        self.assertEqual(ids["bosch_sw"], "1037354178")
        self.assertTrue(ids["epk"].startswith("36/1/ME7.5/5/4018.10"))

    def test_dpp(self):
        dpp = T.dpp_setup(T.Image.load(str(ORIG)))
        self.assertEqual(dpp[0], 0x204)
        self.assertEqual(dpp[1], 0x205)
        self.assertEqual(T.dpp_address(0x14EE9, dpp), (1, 0x4EE9))
        self.assertEqual(T.dpp_address(0x10A00, dpp), (0, 0x0A00))
        self.assertIsNone(T.dpp_address(0x30000, dpp))


@unittest.skipUnless(ORIG.exists() and SAIOFF.exists(), "ECU images missing")
class Checksums(unittest.TestCase):
    def test_structures_found(self):
        cm = T.ChecksumMap(T.Image.load(str(ORIG)))
        self.assertEqual(cm.problems, [])
        self.assertEqual(len(cm.checks), 43)
        self.assertEqual(cm.mp_table, 0x1FC00)
        self.assertEqual(cm.mp_count, 34)
        self.assertEqual(cm.main_table, 0x1C3D8)
        self.assertEqual(cm.crc_blocks, [(0x10000, 0x13FFF, 0x67058),
                                         (0x14300, 0x17F67, 0x6705E),
                                         (0x18191, 0x1FBFF, 0x67064)])

    def test_both_images_clean(self):
        for p in (ORIG, SAIOFF):
            cm = T.ChecksumMap(T.Image.load(str(p)))
            self.assertEqual(cm.errors(), [], p.name)

    def test_values_match_me7sum_reference(self):
        cm = T.ChecksumMap(T.Image.load(str(ORIG)))
        by = {c.store: c for c in cm.checks}
        self.assertEqual(by[0x08038].calc, 0x0000ECCF)
        self.assertEqual(by[0x1A8AE].calc, 0x0000971D)
        self.assertEqual(by[0x0803C].calc, 0x003F6C38)
        self.assertEqual(by[0x6705E].calc, 0x8A3A0620)
        self.assertEqual(by[0x1422C].calc, 0x0AE6D1AE)
        self.assertEqual(by[0x7FFE0].calc, 0xF868A288)
        self.assertEqual(by[0x1FC78].calc, 0x1016C318)

    def test_fix_reproduces_xtuner_file(self):
        img = T.Image.load(str(ORIG))
        img.data[0x14EE9:0x14EEB] = bytes.fromhex("fb3f")
        n = T.ChecksumMap(img).fix()
        self.assertEqual(n, 4)  # mp8, mp28, crc zone 2, main program
        self.assertEqual(img.md5(), SAIOFF_MD5)

    def test_fix_converges_on_multi_region_corruption(self):
        img = T.Image.load(str(ORIG))
        for off, v in [(0x9000, 0x12), (0x8001, 0x34), (0x10100, 0x56), (0x14100, 0x78),
                       (0x14250, 0x9A), (0x15000, 0xBC), (0x19000, 0xF0), (0x25000, 0x11),
                       (0x66000, 0x22), (0x7F000, 0x33), (0x21FFE, 0x55)]:
            img.data[off] = v
        cm = T.ChecksumMap(img)
        self.assertGreater(len(cm.errors()), 10)
        cm.fix()
        self.assertEqual(T.ChecksumMap(img).errors(), [])

    def test_inside_checksum_precorrection(self):
        # corrupt the inverse of the main program pair (which lies inside its own range)
        img = T.Image.load(str(ORIG))
        img.w32(0x7FFE4, 0)
        cm = T.ChecksumMap(img)
        cm.fix()  # main pair + multipoint 34 (which covers the pair) need two passes
        self.assertEqual(img.md5(), ORIG_MD5)


@unittest.skipUnless(ORIG.exists() and SAIOFF.exists(), "ECU images missing")
class DiffAndPatch(unittest.TestCase):
    def test_diff_classifies_runs(self):
        a, b = T.Image.load(str(ORIG)), T.Image.load(str(SAIOFF))
        runs = T.diff_runs(a.data, b.data)
        self.assertEqual(len(runs), 8)
        cm = T.ChecksumMap(a)
        kinds = [T.classify_run(cm, s, e) for s, e in runs]
        self.assertEqual(kinds[0], "calibration")
        self.assertTrue(all(k.startswith("checksum") for k in kinds[1:]))
        self.assertEqual(runs[0], (0x14EE9, 0x14EEA))

    def test_patch_cli(self):
        with tempfile.TemporaryDirectory() as d:
            out = os.path.join(d, "out.bin")
            rc, _ = run("patch", ORIG, "--out", out, "--at", "0x14EE9", "--hex", "fb 3f")
            self.assertEqual(rc, 0)
            self.assertEqual(hashlib.md5(Path(out).read_bytes()).hexdigest(), SAIOFF_MD5)

    def test_apply_diff_ports_change(self):
        with tempfile.TemporaryDirectory() as d:
            out = os.path.join(d, "out.bin")
            rc, _ = run("apply-diff", ORIG, SAIOFF, ORIG, "--out", out)
            self.assertEqual(rc, 0)
            self.assertEqual(hashlib.md5(Path(out).read_bytes()).hexdigest(), SAIOFF_MD5)

    def test_apply_diff_conflict_aborts(self):
        with tempfile.TemporaryDirectory() as d:
            tgt = os.path.join(d, "tgt.bin")
            img = T.Image.load(str(ORIG))
            img.data[0x14EE9] = 0x00
            T.ChecksumMap(img).fix()
            Path(tgt).write_bytes(img.data)
            with self.assertRaises(SystemExit):
                run("apply-diff", ORIG, SAIOFF, tgt, "--out", os.path.join(d, "o.bin"))

    def test_refuses_to_overwrite_input(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "x.bin")
            Path(p).write_bytes(ORIG.read_bytes())
            with self.assertRaises(SystemExit):
                run("patch", p, "--out", p, "--at", "0x14EE9", "--hex", "fb")

    def test_where_and_xref(self):
        rc, out = run("where", ORIG, "0x14EE9")
        self.assertIn("CRC32 zone 2", out)
        self.assertIn("multipoint  8", out)
        self.assertIn("DPP1 address 0x4EE9", out)
        img = T.Image.load(str(ORIG))
        dpp = T.dpp_setup(img)
        self.assertEqual(T.find_xrefs(img, 0x14EE9, dpp), [])
        base, hits = T.nearest_table_base(img, 0x14EE9, dpp)
        self.assertEqual(base, 0x14EE4)
        self.assertEqual(len(hits), 7)


@unittest.skipUnless(ORIG.exists(), "ECU images missing")
class Maps(unittest.TestCase):
    def test_known_tables(self):
        img = T.Image.load(str(ORIG))
        cands, sites = T.find_maps(img, T.dpp_setup(img))
        by = {c.data: c for c in cands}
        m = by[0x14F09]                      # embedded 14x14 next to the SAI edit
        self.assertEqual((m.layout, m.nx, m.ny, m.axes, m.width), ("embedded", 14, 14, 0x14EEB, 1))
        g = by[0x15DD4]                      # group-axes 12x8 (axes @0x14450)
        self.assertEqual((g.layout, g.nx, g.ny, g.axes), ("group", 12, 8, 0x14450))
        self.assertGreater(len(sites), 1000)

    def test_map_cli(self):
        rc, out = run("map", ORIG, "0x14EEB")
        self.assertEqual(rc, 0)
        self.assertIn("map8 data@0x14F09 nx=14 ny=14", out)


XDF = ROOT / "ECU" / "Profiles" / "8N0906018AE.xdf"


class XdfEquations(unittest.TestCase):
    def test_linear_forms(self):
        self.assertEqual(T.xdf_linear("5.000000 * X+ -50.000000"), (5.0, -50.0))
        self.assertEqual(T.xdf_linear("X"), (1.0, 0.0))
        self.assertEqual(T.xdf_linear("40.000000 * X"), (40.0, 0.0))
        self.assertIsNone(T.xdf_linear("X * X"))

    def test_synthetic_definition(self):
        xml = (
            '<XDFFORMAT version="1.60"><XDFHEADER></XDFHEADER>\n'
            '<XDFCONSTANT uniqueid="0x0"><title>TST1</title><description>scalar</description>'
            '<EMBEDDEDDATA mmedaddress="0x10" mmedelementsizebits="8" />'
            '<units>Grad C</units><MATH equation="5.000000 * X+ -50.000000"><VAR id="X" /></MATH></XDFCONSTANT>\n'
            '<XDFTABLE uniqueid="0x0"><title>TST2</title><description>table</description>'
            '<XDFAXIS id="x"><EMBEDDEDDATA mmedaddress="0x20" mmedelementsizebits="8" />'
            '<units>Upm</units><MATH equation="40.000000 * X"><VAR id="X" /></MATH></XDFAXIS>'
            '<XDFAXIS id="z"><EMBEDDEDDATA mmedaddress="0x30" mmedelementsizebits="8" mmedrowcount="1" mmedcolcount="2" />'
            '<units>s</units><MATH equation="0.01 * X"><VAR id="X" /></MATH></XDFAXIS></XDFTABLE>\n'
            '</XDFFORMAT>\n'
        )
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "t.xdf"
            p.write_text(xml, encoding="latin-1")
            by = T.xdf_by_title(T.load_xdf(str(p)))
        self.assertEqual(by["TST1"].address, 0x10)
        self.assertEqual((by["TST1"].factor, by["TST1"].offset), (5.0, -50.0))
        t2 = by["TST2"]
        self.assertEqual((t2.address, t2.rows, t2.cols), (0x30, 1, 2))
        self.assertEqual(t2.x.address, 0x20)


@unittest.skipUnless(ORIG.exists() and SAIOFF.exists() and XDF.exists(), "ECU images or XDF missing")
class OverrunSettings(unittest.TestCase):
    def setUp(self):
        self.tables = T.load_xdf(str(XDF))
        self.by = T.xdf_by_title(self.tables)

    def test_overrun_tables_located(self):
        self.assertEqual(self.by["TKATSA"].address, 0x15450)
        self.assertEqual(self.by["DNSLL"].address, 0x1544B)
        self.assertEqual(self.by["KFTVSA"].address, 0x19465)
        self.assertEqual((self.by["KFTVSA"].rows, self.by["KFTVSA"].cols), (5, 8))
        self.assertEqual(self.by["RLMNSAN"].address, 0x186F2)

    def test_sai_state(self):
        self.assertEqual(T.sai_state(T.Image.load(str(ORIG))), "stock")
        self.assertTrue(T.sai_state(T.Image.load(str(SAIOFF))).startswith("SAI-off patch"))

    def test_sai_patch_leaves_overrun_settings_alone(self):
        a = T.Image.load(str(ORIG))
        b = T.Image.load(str(SAIOFF))
        for name in ("TKATSA", "DNSLL", "DNVSA", "NLRSHB", "KFTVSA", "RLMNSAN"):
            t = self.by[name]
            n = t.rows * t.cols * max(1, t.bits // 8)
            self.assertEqual(a.data[t.address:t.address + n], b.data[t.address:t.address + n], name)

    def test_info_prints_settings(self):
        rc, out = run("info", ORIG, "--xdf", XDF)
        self.assertEqual(rc, 0)
        self.assertIn("sai state : stock", out)
        self.assertIn("NLRSHB   @0x15CAC = 3520 rpm", out)
        self.assertIn("Overrun cut-off lock time after end of start", out)
        self.assertNotIn("Sperrzeit", out)
        self.assertIn("RLMNSAN  @0x186F2", out)

    def test_every_listed_setting_has_english_label(self):
        labels, units = T.load_labels_en()
        for title in T.SETTINGS_OVERRUN:
            self.assertIn(title, labels, title)
            self.assertNotIn("ß", labels[title])

    def test_info_no_settings(self):
        rc, out = run("info", ORIG, "--no-settings")
        self.assertEqual(rc, 0)
        self.assertNotIn("sai state", out)

    def test_info_finds_definition_in_profiles(self):
        rc, out = run("info", ORIG)
        self.assertIn("definition:", out)
        self.assertNotIn("does not match", out)


if __name__ == "__main__":
    unittest.main()
