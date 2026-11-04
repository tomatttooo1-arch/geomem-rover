"""Integration v7 (2026-10-12): record-write / gc switches reach both programs; CPU frequency in sys.csv.

1. CLI: runner and producer --record-write default async, sync accepted, other values refused;
   --gc default|freeze (default default).
2. nano_v3.sh accepts RECORD_WRITE= / GC_MODE= (stub docker, fake inner: the values arrive) and still
   refuses unknown names; nano_chain_inner_v2.sh turns them into the SAME flags on the runner command
   line and in the producer's PRODUCER_EXTRA (executed, not grepped); empty = no flag (program default).
3. session_v3.sh v7 mode: three formal runs new / sync control / new with identical env apart from
   RECORD_WRITE; INTEG_W default unchanged; bash -n.
4. tf_probe sys.csv: one freq_khz_cpuN column per CPU with cpufreq (fake sysfs root), values read per
   sample, unreadable -> empty; summarize_chain_v2 does not count them as processes.
Python 3.8+.
"""
import csv
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import threading
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

import test_integ_v5_source_scripts as v5  # noqa: E402
from test_runner_tf_qos import parsed_args  # noqa: E402

SESSION = ROOT / "docs/design/integ_car_chain_v3_20261011/session_v3.sh"
INNER = ROOT / "tools/integ_car_chain/nano_chain_inner_v2.sh"
BASE = ["--socket", "/tmp/x.sock", "--pose-source", "reference", "--sensor-source", "replay"]
FAKE_INNER = 'echo "INNER RECORD_WRITE=<${RECORD_WRITE-UNSET}> GC_MODE=<${GC_MODE-UNSET}> STALL_DIAG=<${STALL_DIAG-UNSET}>"\n'


class Cli(unittest.TestCase):
    def test_runner(self):
        a = parsed_args(BASE)
        self.assertEqual((a.record_write, a.gc), ("async", "freeze"))     # 10-12: freeze is the CLI default
        a = parsed_args(BASE + ["--record-write", "sync", "--gc", "default"])
        self.assertEqual((a.record_write, a.gc), ("sync", "default"))
        with self.assertRaises(SystemExit):
            parsed_args(BASE + ["--record-write", "never"])

    def test_producer(self):
        from host.ros_geo_producer import parse_args
        a, _ = parse_args(["--data-source", "replay", "--backend", "cpu", "--ground-z-m", "0"])
        self.assertEqual((a.record_write, a.gc), ("async", "freeze"))
        a, _ = parse_args(["--data-source", "replay", "--backend", "cpu", "--ground-z-m", "0",
                           "--record-write", "sync", "--gc", "default"])
        self.assertEqual((a.record_write, a.gc), ("sync", "default"))
        with self.assertRaises(SystemExit):
            parse_args(["--data-source", "replay", "--backend", "cpu", "--ground-z-m", "0", "--gc", "off"])


class BoardPassThrough(unittest.TestCase):
    setUp = v5.BoardScriptPassThrough.setUp
    tearDown = v5.BoardScriptPassThrough.tearDown
    _script = v5.BoardScriptPassThrough._script
    _run = v5.BoardScriptPassThrough._run
    _inner = v5.BoardScriptPassThrough._inner

    def test_nano_v3_record_write_and_gc(self):
        (self.tmp / "repo/tools/integ_car_chain/nano_chain_inner_v2.sh").write_text(FAKE_INNER)
        s = self._script(v5.NANO_V3.read_text(), "nano_v3.sh")
        rc, out = self._run(s, ["chain", "t1", "10", "5", "-1", "1", "1", "1", "",
                                "STALL_DIAG=1 RECORD_WRITE=sync GC_MODE=freeze"])
        self.assertEqual(rc, 0, out)
        self.assertEqual(self._inner(out), "INNER RECORD_WRITE=<sync> GC_MODE=<freeze> STALL_DIAG=<1>")
        rc, out = self._run(s, ["chain", "t2", "10", "5", "-1", "1", "1", "1", "", "RECORD_MODE=sync"])
        self.assertEqual(rc, 2, out)
        self.assertIn("bad env RECORD_MODE=sync", out)

    def inner_flags(self, env):
        """Execute nano_chain_inner_v2.sh's RW_ARGS lines and return (RW_ARGS, PRODUCER_EXTRA)."""
        text = INNER.read_text()
        lines = [ln for ln in text.splitlines() if "RW_ARGS" in ln and not ln.lstrip().startswith("#")]
        block = [ln for ln in lines if not ln.lstrip().startswith("python3") and "--geometry-source" not in ln]
        self.assertEqual(len(block), 3, block)
        script = "PRODUCER_EXTRA='--tf-reliable'\n" + "\n".join(block) + '\necho "RW=<$RW_ARGS> PE=<$PRODUCER_EXTRA>"\n'
        r = subprocess.run(["bash", "-c", script], stdout=subprocess.PIPE, env=dict(os.environ, **env), check=True)
        return r.stdout.decode().strip()

    def test_inner_builds_the_same_flags_for_both(self):
        self.assertEqual(self.inner_flags({"RECORD_WRITE": "sync", "GC_MODE": "freeze"}),
                         "RW=<--record-write sync --gc freeze> PE=<--tf-reliable --record-write sync --gc freeze>")
        env = dict((k, v) for k, v in os.environ.items() if k not in ("RECORD_WRITE", "GC_MODE"))
        r = subprocess.run(["bash", "-c", "\n".join(
            ["unset RECORD_WRITE GC_MODE", "PRODUCER_EXTRA=''"]
            + [ln for ln in INNER.read_text().splitlines() if ln.startswith(("RW_ARGS=", "[ -n \"${GC_MODE", "PRODUCER_EXTRA=\"$PRODUCER"))]
            + ['echo "RW=<$RW_ARGS> PE=<$PRODUCER_EXTRA>"'])], stdout=subprocess.PIPE, env=env, check=True)
        self.assertEqual(r.stdout.decode().strip(), "RW=<> PE=< >")
        text = INNER.read_text()
        self.assertIn("--geometry-fallback none $RT_DIAG $RW_ARGS $RUNTIME_EXTRA", text)
        # the producer gets PRODUCER_EXTRA from rig_inner.sh
        self.assertIn('PRODUCER_EXTRA="$PRODUCER_EXTRA"', text)
        self.assertIn("${PRODUCER_EXTRA:-}", (ROOT / "tools/car_geo_producer/rig_inner.sh").read_text())


class Session(unittest.TestCase):
    def test_v7_mode(self):
        src = SESSION.read_text()
        self.assertIn('case "$MODE" in v4|v5|v6|v7)', src)
        self.assertIn("W=${INTEG_W:-${FPGA_SRC}/runs/integ-car-chain-20261011-worktree}", src)
        arms = [ln.strip() for ln in src.splitlines() if ln.strip().startswith("formal _v7")]
        self.assertEqual(arms, ['formal _v7a1 "" "STALL_DIAG=1 RECORD_WRITE=async$GCE"',
                                'formal _v7s "" "STALL_DIAG=1 RECORD_WRITE=sync GC_MODE=default"',
                                'formal _v7a2 "" "STALL_DIAG=1 RECORD_WRITE=async$GCE"'])
        # the older modes are untouched
        self.assertIn('elif [ "$MODE" = v5 ] || [ "$MODE" = v6 ]; then', src)
        r = subprocess.run(["bash", "-n", str(SESSION)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.assertEqual(r.returncode, 0, r.stdout)


class CpuFreq(unittest.TestCase):
    def test_columns_and_values(self):
        from tools.integ_car_chain import tf_probe
        root = Path(tempfile.mkdtemp(prefix="sysfs_"))
        for n, f in ((0, "1728000"), (1, "729600"), (2, None), (10, "115200")):
            d = root / ("cpu%d" % n) / "cpufreq"
            d.mkdir(parents=True)
            if f is not None:
                (d / "scaling_cur_freq").write_text(f + "\n")
        (root / "cpufreq").mkdir()                  # not a cpuN directory
        (root / "cpu3").mkdir()                     # offline / no cpufreq
        paths = tf_probe.cpu_freq_paths(str(root))
        self.assertEqual([c for c, _ in paths], ["freq_khz_cpu0", "freq_khz_cpu1", "freq_khz_cpu10"])
        (root / "cpu1/cpufreq/scaling_cur_freq").write_text("garbage")
        out = Path(tempfile.mkdtemp(prefix="sys_")) / "sys.csv"
        stop = threading.Event()
        th = threading.Thread(target=tf_probe.sampler, args=(str(out), {"me": os.getpid()}, 0.01, stop),
                              kwargs={"freq_paths": paths})
        th.start()
        threading.Event().wait(0.1)
        stop.set()
        th.join(2)
        rows = list(csv.DictReader(open(str(out))))
        self.assertGreaterEqual(len(rows), 2)
        self.assertEqual(list(rows[0])[-4:], ["me", "freq_khz_cpu0", "freq_khz_cpu1", "freq_khz_cpu10"])
        self.assertEqual((rows[0]["freq_khz_cpu0"], rows[0]["freq_khz_cpu1"], rows[0]["freq_khz_cpu10"]),
                         ("1728000", "", "115200"))
        self.assertTrue(rows[0]["me"].isdigit())

    def test_summarize_skips_freq_columns(self):
        src = (ROOT / "tools/integ_car_chain/summarize_chain_v2.py").read_text()
        self.assertIn('k.startswith("freq_khz_")', src)


if __name__ == "__main__":
    unittest.main()
