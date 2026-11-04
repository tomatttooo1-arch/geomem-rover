"""SW_M1 (nationals 2026-11-03): the PL ROI driver waits for its interrupt (kv_server --roi-wait irq).

Known answers: with wait="irq" the rows equal the CPU reference and the poll driver row for row (synchronous and
late-finishing mock core); the default wait="poll" writes exactly the registers it wrote before SW_M1 (register
write sequence recorded against the pre-change class from git when the tag is present, else against the fixed
expectation: no GIE/IER/ISR traffic) and describes itself with the old keys.
Failure shapes: a lost interrupt (the line never fires) is rescued within one slice per call (irq_rescued, rows
still equal); a stale event in the line is a spurious wake (irq_spurious), the rows still equal; a hung core raises
RoiDeviceError at timeout_s and leaves GIE/IER 0. Positive control: the row_shift mutant is caught in irq mode.
End to end: kv_server --roi-core pl --device mock --roi-concurrent 1 --roi-wait irq (+ a 3 ms late core) = CPU rows.
Run: PYTHONPATH=. python3 -m unittest discover -s tests -p test_swm1_roi_irq_20261103.py"""
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = Path(HERE).parent
sys.path.insert(0, HERE)
sys.path.insert(0, str(ROOT))
import test_kv_roi_pl as K  # noqa: E402
from host.kv_link import CpuRoiCore  # noqa: E402
from host.kv_roi_pl import ROI_REGS, MockRoiWindow, PlRoiCore, RoiDeviceError  # noqa: E402
from host.m7_pl_device import MockPlDevice  # noqa: E402

PRE_TAG = "nat-swm1-pre-20261103"


class RecWindow(MockRoiWindow):
    """Records every register write (offset, value) in order."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.seq = []

    def wr(self, off, value):
        self.seq.append((off, int(value) & 0xFFFFFFFF))
        super().wr(off, value)


def core(wait="poll", window_cls=MockRoiWindow, core_cls=PlRoiCore, **wkw):
    env = K.Env.get()
    dev = MockPlDevice()
    win = window_cls(dev._memory, env.native.lib, **wkw)
    kw = {} if wait == "poll" else {"wait": "irq", "irq": win.irq_line(), "irq_slice_s": 0.01}
    return core_cls(win, dev, **kw), win


def scenes(n, seed=20261103):
    rng = np.random.default_rng(seed)
    for _ in range(n):
        h, w = int(rng.integers(8, 80)), int(rng.integers(8, 120))
        depth, local, cfg = K.scene(rng, h, w, int(rng.integers(1, 33)), margin=float(rng.choice([.04, .5])))
        yield depth, K.roi_dict(local, cfg)


class IrqEqualsPoll(unittest.TestCase):
    def test_rows_equal_cpu_and_poll(self):
        cpu = CpuRoiCore(K.Env.get().obj_lib)
        poll, _ = core()
        for delay in (0.0, 0.003):
            irq, win = core("irq", delay_s=delay)
            rows = 0
            for i, (depth, roi) in enumerate(scenes(60)):
                a, b, c = cpu(depth, roi), poll(depth, roi), irq(depth, roi)
                self.assertEqual((a["status"], a["rows"]), (c["status"], c["rows"]), (delay, i))
                self.assertEqual((b["status"], b["rows"]), (c["status"], c["rows"]), (delay, i))
                rows += len(c["rows"])
            self.assertGreater(rows, 500)
            self.assertEqual(irq.counts["irq_wakeups"], 60, dict(irq.counts))
            self.assertEqual(irq.counts.get("irq_rescued", 0), 0)
            self.assertEqual((win.words[ROI_REGS["GIE"]], win.words[ROI_REGS["IER"]]), (0, 0))
            d = irq.describe()
            self.assertEqual((d["wait"], d["irq"]["arms"]), ("irq", 60))

    def test_positive_control_row_shift_in_irq_mode(self):
        cpu = CpuRoiCore(K.Env.get().obj_lib)
        irq, _ = core("irq", mutant="row_shift")
        diff = sum(cpu(d, r)["rows"] != irq(d, r)["rows"] for d, r in scenes(20))
        self.assertGreater(diff, 10)


class PollUnchanged(unittest.TestCase):
    def test_default_register_traffic_and_describe(self):
        poll, win = core(window_cls=RecWindow)
        for depth, roi in scenes(10):
            poll(depth, roi)
        offs = {o for o, _ in win.seq}
        self.assertFalse(offs & {ROI_REGS["GIE"], ROI_REGS["IER"], ROI_REGS["ISR"]})
        self.assertEqual(set(poll.describe()), {"core", "detail", "buffer_pa", "timeout_s", "counts"})
        self.assertEqual(win.line, None)
        have = subprocess.run(["git", "-C", str(ROOT), "show", PRE_TAG + ":host/kv_roi_pl.py"],
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        if have.returncode != 0:
            return
        with tempfile.TemporaryDirectory() as t:                 # the pre-change driver on the same window
            p = os.path.join(t, "kv_roi_pl_pre.py")
            open(p, "wb").write(have.stdout)
            spec = importlib.util.spec_from_file_location("kv_roi_pl_pre", p)
            pre = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(pre)
            old, wold = core(window_cls=RecWindow, core_cls=pre.PlRoiCore)
            new, wnew = core(window_cls=RecWindow)
            for depth, roi in scenes(10, seed=5):
                self.assertEqual(old(depth, roi)["rows"], new(depth, roi)["rows"])
            self.assertEqual(wold.seq, wnew.seq)
            self.assertEqual(old.counts, new.counts)


class IrqFailureShapes(unittest.TestCase):
    def test_lost_interrupt_is_rescued_within_a_slice(self):
        cpu = CpuRoiCore(K.Env.get().obj_lib)
        irq, _ = core("irq", irq_lost=True)
        t0 = time.monotonic()
        for depth, roi in scenes(5):
            self.assertEqual(cpu(depth, roi)["rows"], irq(depth, roi)["rows"])
        dt = time.monotonic() - t0
        self.assertEqual(irq.counts["irq_rescued"], 5)
        self.assertEqual(irq.counts.get("irq_wakeups", 0), 0)
        self.assertLess(dt, 5 * 0.01 + 1.0)                      # one slice each, not timeout_s (0.5 s)
        self.assertGreaterEqual(dt, 5 * 0.009)

    def test_stale_event_is_a_spurious_wake(self):
        cpu = CpuRoiCore(K.Env.get().obj_lib)
        irq, win = core("irq", delay_s=0.003)
        depth, roi = next(scenes(1))
        win.line.armed = True
        win.line.fire()                                           # an event nobody consumed
        self.assertEqual(cpu(depth, roi)["rows"], irq(depth, roi)["rows"])
        self.assertEqual(irq.counts["irq_spurious"], 1)
        self.assertEqual(irq.counts["irq_wakeups"], 2)

    def test_hung_core_times_out_and_disables(self):
        env = K.Env.get()
        dev = MockPlDevice()
        win = MockRoiWindow(dev._memory, env.native.lib, hang=True)
        irq = PlRoiCore(win, dev, timeout_s=0.05, wait="irq", irq=win.irq_line(), irq_slice_s=0.01)
        depth, roi = next(scenes(1))
        with self.assertRaises(RoiDeviceError):
            irq(depth, roi)
        self.assertEqual(irq.counts["device_timeout"], 1)
        self.assertEqual((win.words[ROI_REGS["GIE"]], win.words[ROI_REGS["IER"]]), (0, 0))

    def test_bad_arguments(self):
        env = K.Env.get()
        dev = MockPlDevice()
        win = MockRoiWindow(dev._memory, env.native.lib)
        with self.assertRaises(ValueError):
            PlRoiCore(win, dev, wait="irq")
        with self.assertRaises(ValueError):
            PlRoiCore(win, dev, wait="spin")


class ServerIrq(unittest.TestCase):
    def test_concurrent_irq_end_to_end(self):
        runs = {}
        for name, extra in (("poll", []), ("irq", ["--roi-wait", "irq"]),
                            ("irq_late", ["--roi-wait", "irq", "--roi-mock-delay-s", "0.003"]),
                            ("irq_lost", ["--roi-wait", "irq", "--roi-mock-irq-lost"])):
            t = K.ServerEndToEndTests("test_pl_mock_end_to_end_and_mutants")
            with tempfile.TemporaryDirectory() as d:
                orig = subprocess.Popen

                def popen(cmd, *a, **kw):
                    if "kv_server.py" in " ".join(map(str, cmd)):
                        cmd = list(cmd) + ["--roi-concurrent", "1", "--out", d] + extra
                    return orig(cmd, *a, **kw)
                subprocess.Popen = popen
                try:
                    s = t._run(None)
                finally:
                    subprocess.Popen = orig
                srv = json.load(open(os.path.join(d, "summary.json")))
            runs[name] = (s, srv)
        for name, (s, srv) in runs.items():
            self.assertTrue(s["roi"]["compare"]["equal"], name)
            self.assertTrue(s["roi"]["compare"]["control_caught"], name)
            self.assertEqual(s["roi"]["frames_ok"], s["answered"], name)
        rc = {n: json.dumps(srv, sort_keys=True) for n, (_, srv) in runs.items()}
        self.assertNotIn('"wait": "irq"', rc["poll"])
        for n in ("irq", "irq_late", "irq_lost"):
            self.assertIn('"wait": "irq"', rc[n], n)
        self.assertIn('"irq_rescued"', rc["irq_lost"])
        self.assertNotIn('"irq_rescued"', rc["irq"])


if __name__ == "__main__":
    unittest.main()
