"""AR helper waits after a signal, and the core resync (relay 5, 2026-10-16).

docs/design/AR_COLOUR_MAIN_20261016.md 11.5 / 12: SIGUSR1 (dump request) interrupted the bare
poll() on the AR core's interrupt; the render was declared failed while the core still ran, and
from then on each completion was taken for the next render's (~47 us "done", half-written
pictures) until exit. tools/kv_link/ar_helper/m7ar_wait.h holds the fix (wait_readable,
resync_core, run_core); m7ar_wait_selftest.cpp drives it on the host:
  * wait_readable: signal at 20 ms + data at 100 ms -> 1 at >= 100 ms, eintr >= 1; signal at 20
    ms + 50 ms deadline -> 0 at >= 50 ms (remaining time recomputed); positive control: a bare
    poll returns -1/EINTR at < 50 ms;
  * a simulated HLS core with UIO irq semantics, five cases (calm, SIGUSR1 during the wait, a
    real timeout, a core that never goes idle, an interrupt that lands after the drain), new
    algorithm vs the 9c2c6c0 run() (positive control, must desync in the signal and timeout
    cases and write registers while the core is busy in the timeout / stuck cases).
Also: the helper (host build) still builds with the header and its mock renders are unchanged
(tests/test_kv_ar.py, test_kv_ar_colour.py, test_kv_ar_replay.py run the helper end to end).

Run: PYTHONHASHSEED=0 PYTHONPATH=. python3 -m unittest discover -s tests -p test_ar_helper_wait.py
(also /usr/bin/python3.8).
"""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / "tools" / "kv_link" / "ar_helper"


class Selftest:
    _rows = None

    @classmethod
    def rows(cls):
        if cls._rows is None:
            cls._tmp = tempfile.TemporaryDirectory(prefix="test_ar_helper_wait_")
            exe = Path(cls._tmp.name) / "wst"
            r = subprocess.run(["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-pthread",
                                "-I", str(HELPER), "-o", str(exe), str(HELPER / "m7ar_wait_selftest.cpp")],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            if r.returncode != 0:
                raise RuntimeError("selftest build failed:\n%s" % r.stdout.decode()[-3000:])
            out = subprocess.run([str(exe)], stdout=subprocess.PIPE, timeout=60, check=True).stdout.decode()
            cls._rows = [json.loads(l) for l in out.splitlines() if l.strip()]
        return cls._rows

    @classmethod
    def get(cls, case, algo=None):
        hits = [r for r in cls.rows() if r["case"] == case and r.get("algo") == algo]
        assert len(hits) == 1, (case, algo, hits)
        return hits[0]


class WaitReadable(unittest.TestCase):
    def test_signal_does_not_end_the_wait(self):
        r = Selftest.get("wait_signal_then_data")
        self.assertEqual(r["ret"], 1)
        self.assertGreaterEqual(r["eintr"], 1)
        self.assertGreaterEqual(r["ms"], 99.0)

    def test_remaining_time_recomputed(self):
        r = Selftest.get("wait_signal_deadline")
        self.assertEqual(r["ret"], 0)
        self.assertGreaterEqual(r["eintr"], 1)
        self.assertGreaterEqual(r["ms"], 50.0)
        self.assertLess(r["ms"], 90.0)

    def test_positive_control_bare_poll_returns_early(self):
        r = Selftest.get("bare_poll_control")
        self.assertEqual(r["ret"], -1)
        self.assertEqual(r["eintr"], 1)
        self.assertLess(r["ms"], 50.0)


class SimulatedCore(unittest.TestCase):
    def test_calm_both_agree(self):
        for algo in ("old", "new"):
            r = Selftest.get("calm", algo)
            self.assertEqual((r["ok"], r["desync"], r["fast_ok"]), (20, 0, 0), r)
        self.assertEqual(Selftest.get("calm", "new")["resync_renders"], 0)

    def test_signal_during_wait(self):
        old, new = Selftest.get("eintr", "old"), Selftest.get("eintr", "new")
        # positive control: the old run desyncs for the rest of the run (done in < 1 ms)
        self.assertGreaterEqual(old["desync"], 10, old)
        self.assertGreaterEqual(old["fast_ok"], 10, old)
        self.assertEqual(old["fail"], 1, old)
        # new: the signal is counted, the wait goes on, nothing fails, nothing desyncs
        self.assertEqual((new["ok"], new["fail"], new["desync"], new["fast_ok"]), (20, 0, 0, 0), new)
        self.assertGreaterEqual(new["eintr"], 1, new)

    def test_real_timeout_recovers(self):
        old, new = Selftest.get("timeout", "old"), Selftest.get("timeout", "new")
        self.assertGreaterEqual(old["desync"], 10, old)
        self.assertGreater(old["writes_while_busy"], 0, old)
        self.assertEqual((new["fail"], new["desync"], new["writes_while_busy"]), (1, 0, 0), new)
        self.assertEqual(new["resync_renders"], 1, new)
        self.assertTrue(new["resync_bits"] & 1, new)   # waited for idle

    def test_never_idle_skips_without_register_writes(self):
        old, new = Selftest.get("stuck", "old"), Selftest.get("stuck", "new")
        self.assertGreater(old["writes_while_busy"], 0, old)
        self.assertGreaterEqual(old["desync"], 10, old)
        self.assertEqual((new["skipped"], new["desync"], new["writes_while_busy"]), (1, 0, 0), new)
        self.assertTrue(new["resync_bits"] & 16, new)
        self.assertEqual(new["ok"], 18, new)

    def test_late_interrupt_is_not_taken_for_done(self):
        new = Selftest.get("late_irq", "new")
        self.assertEqual(new["desync"], 0, new)
        self.assertEqual(new["fail"], 1, new)
        # the late event was either consumed inside the wait or drained by the resync
        self.assertTrue(new["stale_irq"] >= 1 or (new["resync_bits"] & 8), new)


class HelperBuild(unittest.TestCase):
    def test_helper_uses_the_header(self):
        src = (HELPER / "m7ar_present.cpp").read_text()
        self.assertIn('#include "m7ar_wait.h"', src)
        self.assertIn("m7ar::run_core(", src)
        self.assertIn("m7ar::resync_core(", src)
        self.assertIn("m7ar::wait_readable(fd, deadline, eintr)", src)
        self.assertNotIn("poll(&p, 1, static_cast<int>(timeout_s * 1000))", src)
        mk = (HELPER / "Makefile").read_text()
        self.assertIn("m7ar_wait.h", mk)


if __name__ == "__main__":
    unittest.main()
