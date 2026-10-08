"""Tests for ice40_release.py: python3 -m unittest tools/test_ice40_release.py (make -C ice40 releasetooltest)"""

import argparse
import contextlib
import io
import os
import pathlib
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ice40_release as rel  # noqa: E402

FLAGS = dict(synth_flags="-abc9 -dsp -spram -top top", pnr_flags="--up5k --threads 1",
             defines_loop="-DVERSION_V2", defines_deep="-DVERSION_V2 -DUSE_DEEP_REPLAY")


class Ice40ReleaseTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = pathlib.Path(self.tmp.name)
        ice40 = root / "ice40"
        (ice40 / "src").mkdir(parents=True)
        (ice40 / "synth").mkdir()
        (ice40 / "src" / "top_v2.v").write_text("engine_block #(.GATEWARE_VERSION(8'd47)) e ();\n")
        (ice40 / "src" / "a.vh").write_text("`define A 1\n")
        (ice40 / "vbench_pod.pcf").write_text("set_io clk48 35\n")
        (ice40 / "clocks.py").write_text("ctx.addClock('clk48', 48)\n")
        (ice40 / "synth" / "sb_mac16_keep.v").write_text("module SB_MAC16_KEEP; endmodule\n")
        (ice40 / "synth" / "check_dsp.py").write_text("\n")
        self.saved = (rel.ROOT, rel.ICE40, rel.REL, rel.MANIFEST)
        rel.ROOT, rel.ICE40 = root, ice40
        rel.REL = ice40 / "release"
        rel.MANIFEST = rel.REL / "MANIFEST"
        # images built after the sources
        past = time.time() - 100
        for f in rel.build_inputs():
            os.utime(f, (past, past))
        for kind in rel.IMAGES:
            img = rel.image_path(kind)
            img.write_bytes(kind.encode() * 64)
            (img.parent / (img.name + ".gwversion")).write_text("47\n")
            os.utime(img, (past + 50, past + 50))

    def tearDown(self):
        rel.ROOT, rel.ICE40, rel.REL, rel.MANIFEST = self.saved
        self.tmp.cleanup()

    def args(self, **kw):
        a = dict(hw_verified="bench 1/1", seed_loop="26", seed_deep="57", **FLAGS)
        a.update(kw)
        return argparse.Namespace(**a)

    def promote(self, **kw):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            rel.cmd_promote(self.args(**kw))
        return out.getvalue()

    def test_promote_records_toolchain_and_flags(self):
        self.promote()
        m = rel.read_manifest()
        self.assertEqual(m["synth_flags"], FLAGS["synth_flags"])
        self.assertEqual(m["pnr_flags"], FLAGS["pnr_flags"])
        self.assertEqual(m["loop.defines"], FLAGS["defines_loop"])
        self.assertEqual(m["deep.defines"], FLAGS["defines_deep"])
        self.assertIn("nextpnr_revision", m)
        self.assertEqual(rel.check(), [])
        self.assertEqual(rel.flag_notes(m, self.args()), [])

    def test_promote_refuses_an_image_older_than_a_source(self):
        src = rel.ICE40 / "src" / "a.vh"
        src.write_text("`define A 2\n")          # edited after the images were built
        with self.assertRaises(SystemExit) as e:
            self.promote()
        self.assertIn("older than ice40/src/a.vh", str(e.exception))
        self.assertFalse(rel.MANIFEST.exists())

    def test_promote_refuses_an_image_older_than_a_synth_helper(self):
        os.utime(rel.ICE40 / "synth" / "check_dsp.py")
        with self.assertRaises(SystemExit) as e:
            self.promote()
        self.assertIn("check_dsp.py", str(e.exception))

    def test_check_notes_moved_flags_but_passes(self):
        self.promote()
        m = rel.read_manifest()
        notes = rel.flag_notes(m, self.args(synth_flags="-noabc9 -top top"))
        self.assertEqual(len(notes), 1)
        self.assertIn("synth_flags is now '-noabc9 -top top'", notes[0])
        self.assertEqual(rel.check(), [])

    def test_check_accepts_a_manifest_without_the_new_fields(self):
        self.promote()
        kept = [ln for ln in rel.MANIFEST.read_text().splitlines()
                if not ln.startswith(("synth_flags=", "pnr_flags=", "nextpnr_revision=", "loop.defines=",
                                      "deep.defines="))]
        rel.MANIFEST.write_text("\n".join(kept) + "\n")
        self.assertEqual(rel.check(), [])
        notes = rel.flag_notes(rel.read_manifest(), self.args())
        self.assertIn("the MANIFEST predates recording synth_flags", notes)

    def test_nextpnr_revision_falls_back_to_the_homebrew_version(self):
        bindir = pathlib.Path(self.tmp.name) / "Cellar" / "nextpnr-ice40" / "0.10" / "bin"
        bindir.mkdir(parents=True)
        exe = bindir / "nextpnr-ice40"
        exe.write_text('#!/bin/sh\necho \'"nextpnr-ice40" -- Next Generation Place and Route (Version )\' >&2\n')
        exe.chmod(0o755)
        self.assertRegex(rel.nextpnr_revision(str(exe)), r"^homebrew nextpnr-ice40 0\.10 \(sha256 [0-9a-f]{16}\)$")
        exe.write_text('#!/bin/sh\necho \'"nextpnr-ice40" -- Next Generation Place and Route (Version nextpnr-0.9-12-gabcdef)\'\n')
        self.assertRegex(rel.nextpnr_revision(str(exe)), r"^nextpnr-0\.9-12-gabcdef \(sha256 ")


if __name__ == "__main__":
    unittest.main()
