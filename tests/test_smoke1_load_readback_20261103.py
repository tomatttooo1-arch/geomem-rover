"""SMOKE1 (2026-11-03): a bitstream load is judged by the BUILD_ID read back from sysinfo.

BOARD3 found that "xmutil loadapp" right after copying an app directory printed "load Error: -1" with exit code 0
while the PL kept the previous bitstream, and board_bringup.sh step 1 still printed PASS (it only looked for the app
name in listapps). Checked here:

* kv_start.sh (the car's KV start): readback equal => one load, "FPGA_LOAD ok"; readback wrong on the first load and
  right on the second => unload + load again, then the same server start; wrong every time / unreadable => three
  loads, a loud FPGA_LOAD_MISMATCH and the server still starts (owner 10-26: warn, no new refusal); a non-zero xmutil
  exit no longer stops the start. Positive control: the pre-SMOKE1 kv_start.sh under "always wrong" loads once and
  says nothing.
* board_bringup.sh step 1: same readback through the package's own m7pl_test info; expected id from M7PL_BUILD_ID or
  SELFCHECK.txt; wrong after the retries => FAIL / STOP at step 1. Without an expected id it falls back to the old
  listapps judgment, which passes a wrong bitstream (the blind spot, shown on purpose).
* census: every source line that calls "xmutil loadapp" is in a known file; kv_start.sh and board_bringup.sh are the
  car start chain and the package bring-up, the rest are dig/demo tools that already read the BUILD_ID back or are
  historical. A new caller makes this test fail so that it gets the readback too.
"""
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests"))

KV_START = ROOT / "ui/geomem/tools/competition/kv_start.sh"
BRINGUP = ROOT / "hw/m7/pl/tools/board_bringup.sh"
OLD_ID = "0x03c087c9"     # v3fc2: what the PL held when BOARD3's load failed


def kv_run(profile, readback, old_script=None):
    """readback: 'want' | 'second' | 'old' | 'empty' (shell lines over the harness' calls.log)."""
    from test_competition_kv_service import KvStartTests
    t = KvStartTests("test_no_argument_is_old_a2_command")
    orig = subprocess.run
    seen = {}

    def run(cmd, *a, **kw):
        if "kv_start.sh" in " ".join(map(str, cmd)):
            repo = Path(cmd[2])
            log = repo.parent / "calls.log"
            fw = Path(kw["env"]["FPGA_FW_DIR"])
            for app in ("m7_pl_m2_166", "m7_pl_m2"):
                (fw / app).mkdir(parents=True, exist_ok=True)
            n = 'n=$(grep -c "xmutil loadapp" "%s" 2>/dev/null || true); ' % log
            line = {
                "want": 'printf "%s\\n" "$FPGA_BUILD_ID"',
                "second": n + '[ "$n" -ge 2 ] && echo "$FPGA_BUILD_ID" || echo ' + OLD_ID,
                "old": "echo " + OLD_ID,
                "empty": "true",
            }[readback]
            kw["env"] = dict(kw["env"], GEOMEM_KV_BUILD_ID_CMD=line, GEOMEM_KV_LOAD_WAIT_S="0",
                             GEOMEM_KV_READ_WAIT_S="0")
            if old_script is not None:
                cmd = [cmd[0], str(old_script)] + list(cmd[2:])
            seen["cmd"] = cmd
        return orig(cmd, *a, **kw)
    subprocess.run = run
    try:
        r, calls, old = t.run_start(*([profile] if profile else []))
    finally:
        subprocess.run = orig
    t.doCleanups()
    return r, calls, old


class KvStartReadback(unittest.TestCase):
    def test_match_first_time_is_one_load(self):
        r, calls, old = kv_run("m2_166", "want")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(calls, ["xmutil unloadapp", "xmutil loadapp m7_pl_m2_166", "kv_board " + old + " --rolling pl"])
        self.assertIn("FPGA_LOAD ok app=m7_pl_m2_166 build_id=0x07da3d4a loads=1", r.stdout)
        self.assertNotIn("WARNING: after load", r.stderr)
        self.assertNotIn("FPGA_LOAD_MISMATCH", r.stdout + r.stderr)

    def test_old_bitstream_after_first_load_reloads(self):
        r, calls, old = kv_run("m2_166", "second")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(calls, ["xmutil unloadapp", "xmutil loadapp m7_pl_m2_166"] * 2
                         + ["kv_board " + old + " --rolling pl"])
        self.assertIn("reads BUILD_ID %s, not 0x07da3d4a" % OLD_ID, r.stderr)
        self.assertIn("FPGA_LOAD ok app=m7_pl_m2_166 build_id=0x07da3d4a loads=2", r.stdout)

    def test_always_wrong_warns_and_still_starts(self):
        for rb, shown in (("old", OLD_ID), ("empty", "unreadable")):
            with self.subTest(rb=rb):
                r, calls, old = kv_run("m2", rb)
                self.assertEqual(r.returncode, 0, r.stderr)
                self.assertEqual(calls, ["xmutil unloadapp", "xmutil loadapp m7_pl_m2"] * 3
                                 + ["kv_board " + old + " --rolling pl"])
                line = "FPGA_LOAD_MISMATCH app=m7_pl_m2 expected=0x07da3d4a read=%s loads=3" % shown
                self.assertIn(line, r.stdout)
                self.assertIn(line, r.stderr)

    def test_a2_default_checks_its_own_id(self):
        r, calls, old = kv_run(None, "want")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("FPGA_LOAD ok app=m7_pl_merge_a2 build_id=0x00da5041 loads=1", r.stdout)

    def test_nonzero_xmutil_is_not_fatal(self):
        from test_competition_kv_service import KvStartTests
        orig = KvStartTests.run_start

        def patched(self, *extra, **kw):
            o = subprocess.run

            def run(cmd, *a, **k):
                if "kv_start.sh" in " ".join(map(str, cmd)):
                    xm = Path(cmd[2]).parent / "bin/xmutil"
                    xm.write_text(xm.read_text() + "exit 1\n")
                return o(cmd, *a, **k)
            subprocess.run = run
            try:
                return orig(self, *extra, **kw)
            finally:
                subprocess.run = o
        KvStartTests.run_start = patched
        try:
            r, calls, old = kv_run("m2_166", "want")
        finally:
            KvStartTests.run_start = orig
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("xmutil loadapp m7_pl_m2_166 rc=1", r.stderr)
        self.assertIn("FPGA_LOAD ok", r.stdout)
        self.assertEqual(calls[-1], "kv_board " + old + " --rolling pl")

    def test_positive_control_old_script_is_blind(self):
        with tempfile.TemporaryDirectory() as d:
            oldsh = Path(d) / "kv_start_pre.sh"
            src = subprocess.run(["git", "-C", str(ROOT), "show",
                                  "nat-smoke1-pre-20261103:ui/geomem/tools/competition/kv_start.sh"],
                                 stdout=subprocess.PIPE, universal_newlines=True)
            if src.returncode != 0:
                self.skipTest("tag nat-smoke1-pre-20261103 not available")
            oldsh.write_text(src.stdout)
            r, calls, old = kv_run("m2", "old", old_script=oldsh)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(calls, ["xmutil unloadapp", "xmutil loadapp m7_pl_m2", "kv_board " + old + " --rolling pl"])
        self.assertNotIn("FPGA_LOAD", r.stdout + r.stderr)


class BringupStep1(unittest.TestCase):
    APP = "m7_pl_m2_166"

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        t = Path(self.tmp.name)
        self.pkg, self.bin, self.fw = t / "pkg", t / "bin", t / "fw"
        self.log = t / "xm.log"
        (self.pkg / self.APP).mkdir(parents=True)
        (self.pkg / self.APP / "x.bit.bin").write_text("bit")
        (self.pkg / "board_bringup.sh").write_text(BRINGUP.read_text())
        self.bin.mkdir()
        self.fake(self.bin / "xmutil", 'echo "xmutil $*" >> %s\n[ "$1" = listapps ] && echo "%s XRT_FLAT -1"\n'
                  'exit 0' % (self.log, self.APP))
        self.fake(self.bin / "id", 'echo 0')
        self.fake(self.bin / "dmesg", 'exit 0')
        self.fake(self.pkg / "m7pl_test", 'n=$(grep -c "xmutil loadapp" %s 2>/dev/null || true)\n'
                  'id=$(cat %s/readback 2>/dev/null)\n'
                  'case "$id" in second) [ "$n" -ge 2 ] && id=0x07da3d4a || id=%s ;; esac\n'
                  '[ -n "$id" ] || exit 1\n'
                  'echo "sysinfo MAGIC=0x4d375049 VERSION=0x1 BUILD_ID=$id BUILD_DATE=0x20261103 CLK_HZ=100000000 '
                  'SLOT_MAP=0x263"\necho "PASS sysinfo"' % (self.log, t, OLD_ID))

    def fake(self, p, body):
        p.write_text("#!/bin/sh\n" + body + "\n")
        p.chmod(0o755)

    def run_step1(self, readback, selfcheck=True, env=None):
        t = Path(self.tmp.name)
        (t / "readback").write_text(readback)
        if selfcheck:
            (self.pkg / "SELFCHECK.txt").write_text(
                "x\nexpected sysinfo BUILD_ID on the board: 0x07da3d4a (source commit 7da3d4a)\n")
        sums = subprocess.run("find . -type f ! -name SHA256SUMS ! -path './bringup-*' | sort | xargs sha256sum", shell=True,
                              cwd=str(self.pkg), stdout=subprocess.PIPE, universal_newlines=True).stdout
        (self.pkg / "SHA256SUMS").write_text(sums)
        e = dict(os.environ, PATH="%s:%s" % (self.bin, os.environ["PATH"]), M7PL_APP=self.APP,
                 M7PL_FW_DIR=str(self.fw), M7PL_LOAD_WAIT_S="0", **(env or {}))
        r = subprocess.run(["bash", "./board_bringup.sh", "1"], cwd=str(self.pkg), env=e, stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE, universal_newlines=True, timeout=60)
        loads = self.log.read_text().count("xmutil loadapp") if self.log.exists() else 0
        return r, loads

    def test_match_first_time(self):
        r, loads = self.run_step1("0x07da3d4a")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("PASS xmutil loadapp m7_pl_m2_166: sysinfo BUILD_ID 0x07da3d4a (load 1)", r.stdout)
        self.assertEqual(loads, 1)
        self.assertTrue((self.fw / self.APP / "x.bit.bin").is_file())

    def test_second_load_works(self):
        r, loads = self.run_step1("second")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("WARN load 1 of m7_pl_m2_166: sysinfo BUILD_ID %s" % OLD_ID, r.stdout)
        self.assertIn("(load 2)", r.stdout)
        self.assertEqual(loads, 2)

    def test_wrong_after_retries_fails(self):
        for rb in (OLD_ID, ""):
            with self.subTest(rb=rb):
                if self.log.exists():
                    self.log.unlink()
                r, loads = self.run_step1(rb)
                self.assertEqual(r.returncode, 1, r.stdout)
                self.assertIn("FAIL xmutil loadapp m7_pl_m2_166: sysinfo BUILD_ID %s after 3 loads"
                              % (rb or "unreadable"), r.stdout)
                self.assertIn("STOP at step 1", r.stdout)
                self.assertEqual(loads, 3)

    def test_env_overrides_selfcheck(self):
        r, loads = self.run_step1(OLD_ID, env={"M7PL_BUILD_ID": OLD_ID})
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertIn("sysinfo BUILD_ID %s (load 1)" % OLD_ID, r.stdout)

    def test_without_expected_id_is_the_old_blind_check(self):
        # positive control for the blind spot: no expected id => listapps only => a wrong bitstream passes
        r, loads = self.run_step1(OLD_ID, selfcheck=False)
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertIn("WARN expected BUILD_ID unknown", r.stdout)
        self.assertIn("PASS xmutil loadapp m7_pl_m2_166 (listapps only)", r.stdout)
        self.assertEqual(loads, 1)


class LoadappCensus(unittest.TestCase):
    KNOWN = {
        # car start chain + package bring-up: BUILD_ID readback (this change)
        "ui/geomem/tools/competition/kv_start.sh", "hw/m7/pl/tools/board_bringup.sh",
        # bench / dig / demo tools: read the BUILD_ID back themselves (kv_board.sh check / m7pl_test info) or
        # historical one-off sessions; not in the car start chain
        "tools/car_run/bench_line4.sh", "tools/car_run/halfrate_dig/hr_session.sh", "tools/demo_shot5/kv_shot5.sh",
        "tools/hb_gap_dig/shm8g/shm8g_session.sh", "tools/hb_gap_dig/t1/t1_session.sh",
        "tools/hb_gap_dig/t2/t2_session.sh", "tools/demo_shot5/shot5_run.py",
        "tools/kv_link/ar_screen.sh",   # AR screen bring-up tool (10-15), not in the car start chain
        # mentions only (overlay comment, an advice string, a docstring)
        "hw/m7/pl/tools/m7pl_gen.py", "tools/kv_link/kv_server.py", "tools/m7_rolling_pl/replay_trace.py",
    }

    def test_every_loadapp_caller_is_known(self):
        out = subprocess.run(["git", "-C", str(ROOT), "grep", "-n", "xmutil loadapp", "--", "*.sh", "*.py",
                              ":!tests/*", ":!docs/*"], stdout=subprocess.PIPE, universal_newlines=True).stdout
        files = set()
        for line in out.splitlines():
            f, _n, text = line.split(":", 2)
            s = text.strip()
            if s.startswith("#") or "(xmutil loadapp)" in s:   # comments, car_up's advice text
                continue
            files.add(f)
        self.assertEqual(files - self.KNOWN, set(),
                         "new xmutil loadapp caller(s): add the BUILD_ID readback, then list them here")
        self.assertIn("ui/geomem/tools/competition/kv_start.sh", files)
        self.assertIn("hw/m7/pl/tools/board_bringup.sh", files)


if __name__ == "__main__":
    unittest.main()
