"""Runner stall diagnostics (host/runner_stall_diag.py, ros_runtime --stall-diag).

1. Module: inject spec parsing; a section that sleeps is attributed to that section only
   (free sleep: wall without CPU; hold: the GIL watchdog also fires); poll gaps; /tf arrival
   gaps; acquisition waits/drops; the file is created new (never overwritten).
2. ros_runtime on the M5 software replay with --stall-diag:
   * off (default): the event stream equals the cpu baseline (the diagnostics are opt-in);
   * on, no injection: the event stream is still identical to the run without it, and every
     processed frame has one frame_timing row with every poll section;
   * on, positive control memory:1:80:free / geometry:1:80:hold: every frame row puts >= 80 ms
     in that section and < 80 ms in every other one; hold also shows GIL watchdog overshoots.
3. Frameless polls (10-12): sections marked while no frame is open (permit, /fpga/memory publish)
   give a poll_sections row when >= 50 ms; every injection writes an inject row (in a frame or
   not); the module before (4e722be) wrote neither for the same calls; stall_report counts both.
Python 3.8+.
"""

import json
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from host import runner_stall_diag as rsd  # noqa: E402


def rows_of(path, kind=None):
    with open(path) as stream:
        rows = [json.loads(l) for l in stream if l.strip()]
    return [r for r in rows if kind is None or r.get("event") == kind]


class Module(unittest.TestCase):
    def tmp(self):
        return Path(tempfile.mkdtemp(prefix="rsd_")) / "diag.jsonl"

    def test_parse_inject(self):
        self.assertIsNone(rsd.parse_inject(None))
        self.assertEqual(rsd.parse_inject("memory:5:250"), ("memory", 5, 250.0, "free"))
        self.assertEqual(rsd.parse_inject("permit:1:10:hold"), ("permit", 1, 10.0, "hold"))
        for bad in ("nowhere:1:10", "memory:0:10", "memory:1:0", "memory:1:10:spin", "memory:1"):
            with self.assertRaises(ValueError):
                rsd.parse_inject(bad)

    def test_new_file_only(self):
        p = self.tmp()
        d = rsd.RunnerStallDiag(p, watchdog=False)
        d.close()
        with self.assertRaises(FileExistsError):
            rsd.RunnerStallDiag(p, watchdog=False)

    def frame(self, fid="rgb-1"):
        return types.SimpleNamespace(frame_id=fid)

    def test_section_attribution_free_and_hold(self):
        p = self.tmp()
        d = rsd.RunnerStallDiag(p, gil_period_s=0.01, gil_min_ms=30.0)
        for mode in ("free", "hold"):
            f = self.frame("rgb-" + mode)
            d.frame_begin(f)
            d.mark("result")
            (rsd.free_gil_sleep if mode == "free" else rsd.hold_gil_sleep)(120)
            d.mark("memory")
            d.mark("record_publish")
            d.frame_end(tracks=3, result="processed")
        d.close()
        timing = rows_of(p, "frame_timing")
        self.assertEqual(len(timing), 2)
        for row in timing:
            s = row["sections_ms"]
            self.assertGreaterEqual(s["memory"], 115)
            self.assertLess(max(v for k, v in s.items() if k != "memory"), 20)
            self.assertLess(row["sections_cpu_ms"]["memory"], 30)       # a sleep: wall without CPU
        gil = rows_of(p, "gil_stall")
        # hold: the watchdog thread could not run for ~120 ms; free: it could
        self.assertTrue(any(g["late_ms"] >= 80 for g in gil), gil)
        free_t = [r for r in timing if r["frame_id"] == "rgb-free"][0]["end_raw_ns"]
        self.assertFalse([g for g in gil if g["end_raw_ns"] <= free_t and g["late_ms"] >= 80])

    def test_inject_every(self):
        p = self.tmp()
        d = rsd.RunnerStallDiag(p, inject="gate:3:20", watchdog=False)
        slept = [d.maybe_inject("gate") for _ in range(7)] + [d.maybe_inject("memory")]
        d.close()
        self.assertEqual(slept, [0.0, 0.0, 20.0, 0.0, 0.0, 20.0, 0.0, 0.0])
        self.assertEqual(rows_of(p, "stall_diag_summary")[0]["injected"], {"gate": 2})

    def test_poll_gap_tf_gap_and_acquisition(self):
        p = self.tmp()
        clock = [10 ** 12]
        d = rsd.RunnerStallDiag(p, clock=lambda: clock[0], watchdog=False)
        d.poll_enter()
        clock[0] += 100 * 10 ** 6
        d.poll_enter()
        clock[0] += 350 * 10 ** 6
        d.poll_enter()
        hdr = lambda ns: types.SimpleNamespace(stamp=types.SimpleNamespace(sec=ns // 10 ** 9, nanosec=ns % 10 ** 9))
        tf = lambda ns: types.SimpleNamespace(transforms=[types.SimpleNamespace(header=hdr(ns))])
        d.on_tf(tf(5 * 10 ** 9), clock[0])
        d.on_tf(tf(5 * 10 ** 9 + 20 * 10 ** 6), clock[0] + 20 * 10 ** 6)
        d.on_tf(tf(5 * 10 ** 9 + 40 * 10 ** 6), clock[0] + 420 * 10 ** 6)
        d.acquisition_wait(6 * 10 ** 9, clock[0], "base_camera_static_tf", "ExtrapolationException", 5 * 10 ** 9)
        d.acquisition_wait(6 * 10 ** 9, clock[0], "base_camera_static_tf", "ExtrapolationException", 5 * 10 ** 9 + 10 ** 8)
        clock[0] += 500 * 10 ** 6
        d.acquisition_dropped(6 * 10 ** 9, clock[0] - 500 * 10 ** 6, "wait_expired")
        d.close()
        gaps = rows_of(p, "poll_gap")
        self.assertEqual([round(g["gap_ms"]) for g in gaps], [350])
        self.assertEqual([round(g["gap_ms"]) for g in rows_of(p, "tf_arrival_gap")], [400])
        drop = rows_of(p, "acquisition_dropped")[0]
        self.assertEqual((drop["tries"], drop["max_tf_lag_ms"], round(drop["since_rx_ms"])), (2, 1000.0, 500))
        self.assertEqual(drop["branches"], {"base_camera_static_tf:ExtrapolationException": 2})


class FramelessPoll(unittest.TestCase):
    """10-12: a poll without a frame (permit + throttled /fpga/memory publish in poll's finally path)
    used to drop its marks (mark() needed an open frame): an injected memory_publish stall there was
    visible only as a poll gap. Now: a poll_sections row (>= 50 ms) and an inject row per injection."""

    def tmp(self):
        return Path(tempfile.mkdtemp(prefix="rsd_fp_")) / "diag.jsonl"

    @staticmethod
    def frameless_poll(d, clock, permit_ms, publish_ms):
        d.poll_enter()
        clock[0] += 5 * 10 ** 6             # prepare_images etc., not a section
        d.skip()                            # poll's finally path: skip() before the permit
        clock[0] += permit_ms * 10 ** 6
        d.mark("permit")
        clock[0] += 1 * 10 ** 6
        d.mark("memory_feed")
        d.maybe_inject("memory_publish")
        clock[0] += publish_ms * 10 ** 6
        d.mark("memory_publish")

    def run_module(self, module, inject=None):
        p = self.tmp()
        clock = [10 ** 12]
        d = module.RunnerStallDiag(p, clock=lambda: clock[0], watchdog=False, inject=inject)
        self.frameless_poll(d, clock, 1, 70)        # slow publish, no frame
        self.frameless_poll(d, clock, 1, 10)        # fast: no row
        d.poll_enter()                              # a poll with a frame: sections go to the frame row
        d.frame_begin(types.SimpleNamespace(frame_id="rgb-f"))
        clock[0] += 3 * 10 ** 6
        d.mark("result")
        d.skip()
        d.maybe_inject("memory_publish")
        clock[0] += 80 * 10 ** 6
        d.mark("memory_publish")
        d.frame_end(result="processed")
        d.poll_enter()
        d.close()
        d.flush(force=True)
        return p

    def test_frameless_sections_row(self):
        p = self.run_module(rsd)
        rows = rows_of(p, "poll_sections")
        self.assertEqual(len(rows), 1, rows)
        self.assertEqual(rows[0]["sections_ms"], {"permit": 1.0, "memory_feed": 1.0, "memory_publish": 70.0})
        timing = rows_of(p, "frame_timing")
        self.assertEqual(len(timing), 1)
        self.assertEqual(timing[0]["sections_ms"], {"result": 3.0, "memory_publish": 80.0})
        self.assertEqual(rows_of(p, "stall_diag_summary")[0]["counts"]["poll_sections"], 1)

    def test_inject_rows_in_and_out_of_frame(self):
        p = self.run_module(rsd, inject="memory_publish:1:5:free")
        inj = rows_of(p, "inject")
        self.assertEqual([r["in_frame"] for r in inj], [None, None, "rgb-f"])
        self.assertTrue(all(r["section"] == "memory_publish" and r["ms"] == 5.0 for r in inj))
        self.assertEqual(rows_of(p, "stall_diag_summary")[0]["injected"], {"memory_publish": 3})

    def test_old_module_was_blind(self):
        # the module before this change (4e722be): same calls, no poll_sections and no inject rows
        import subprocess
        import importlib.util
        try:
            src = subprocess.run(["git", "-C", str(ROOT), "show", "4e722be:host/runner_stall_diag.py"],
                                 stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=True,
                                 universal_newlines=True).stdout
        except (subprocess.CalledProcessError, OSError):
            self.skipTest("git history not available")
        spec = importlib.util.spec_from_loader("rsd_old_4e722be", loader=None)
        old = importlib.util.module_from_spec(spec)
        exec(compile(src, "rsd_old_4e722be", "exec"), old.__dict__)
        p = self.run_module(old, inject="memory_publish:1:5:free")
        self.assertEqual(rows_of(p, "poll_sections"), [])
        self.assertEqual(rows_of(p, "inject"), [])
        self.assertEqual(rows_of(p, "stall_diag_summary")[0]["injected"], {"memory_publish": 3})

    def test_report_counts_frameless_rows(self):
        from tools.runner_age_gate import stall_report
        chain = Path(tempfile.mkdtemp(prefix="rsd_rep_"))
        (chain / "runtime").mkdir()
        p = chain / "runtime" / "stall_diag.jsonl"
        clock = [10 ** 12]
        d = rsd.RunnerStallDiag(p, clock=lambda: clock[0], watchdog=False, inject="memory_publish:1:5:hold")
        self.frameless_poll(d, clock, 1, 250)
        d.poll_enter()
        d.close()
        d.flush(force=True)
        inj = rows_of(p, "inject")[0]
        with open(p, "a") as f:             # a GIL watchdog stall over the injection
            f.write(json.dumps({"event": "gil_stall", "late_ms": 240.0, "start_raw_ns": inj["start_raw_ns"],
                                "end_raw_ns": inj["start_raw_ns"] + 250 * 10 ** 6}) + "\n")
        chk = stall_report.report(str(chain))["inject_check"]
        self.assertEqual((chk["inject_rows"], chk["inject_frameless"], chk["rows_frameless_poll"],
                          chk["of_which_only_that_section"], chk["inject_seen_by_gil_watchdog"]), (1, 1, 1, 1, 1))
        rep = stall_report.report(str(chain))
        self.assertEqual([(r["section"], r["frame_id"]) for r in rep["long_sections_ge_100ms"]],
                         [("memory_publish", None)])


class RuntimeWiring(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import test_car_geometry_kv as cg
        cls.cg = cg
        if cg.baseline_source() is None:
            raise unittest.SkipTest("git history not available")
        cls.src = (ROOT / "host" / "ros_runtime.py").read_text(encoding="utf-8")
        cls.before, _ = cg.capture(cg.baseline_source(), "rsd_cpu_baseline")
        cls.plain, _ = cg.capture(cls.src, "rsd_cpu_plain", {"geometry_source": "cpu"})

    def diag_capture(self, name, inject=None):
        path = Path(tempfile.mkdtemp(prefix="rsd_rt_"))
        counter = [0]
        import test_region_source_none_regression as reg
        original = reg.make_loader
        from unittest.mock import patch

        def make_loader(src, module, extra_args=None):
            def load_runtime(world, dpath):
                counter[0] += 1
                extra = dict(extra_args or {}, stall_diag=path / ("diag_%02d.jsonl" % counter[0]),
                             stall_diag_inject=inject)
                return original(src, module, extra)(world, dpath)
            return load_runtime
        with patch.object(reg, "make_loader", make_loader):
            streams, ok = reg.capture(self.src, name, {"geometry_source": "cpu"})
        files = sorted(path.glob("diag_*.jsonl"))
        self.assertEqual(len(files), counter[0])
        return streams, files

    def test_default_off_unchanged(self):
        self.assertEqual(self.cg.diff_names(self.before, self.plain, self.cg.strict), [])

    def test_on_same_events_and_one_row_per_frame(self):
        streams, files = self.diag_capture("rsd_cpu_on")
        self.assertEqual(self.cg.diff_names(self.plain, streams, self.cg.strict), [])
        processed = sum(len(self.cg.status_rows({k: v}, "frame_processed")) for k, v in streams.items())
        timing = [r for f in files for r in rows_of(f, "frame_timing")]
        self.assertEqual(len([r for r in timing if r["result"] == "processed"]), processed)
        for r in timing:
            if r["result"] == "processed":
                self.assertTrue({"result", "process", "geometry", "gate", "memory", "stats", "record_dumps",
                                 "record_write", "record_publish", "task", "submit", "permit"}
                                <= set(r["sections_ms"]), r["sections_ms"])
                self.assertEqual((r["gate_verdict"], r["age_basis"]), ("pass", "runner_arrival"))
                self.assertIsNotNone(r["spans_ms"]["picked_to_gate"])
        for f in files:
            summary = rows_of(f, "stall_diag_summary")
            self.assertEqual(len(summary), 1)
            self.assertEqual(summary[0]["injected"], {})

    def assert_injected(self, files, section, ms):
        timing = [r for f in files for r in rows_of(f, "frame_timing") if r["result"] == "processed"]
        self.assertGreater(len(timing), 10)
        for r in timing:
            s = r["sections_ms"]
            self.assertGreaterEqual(s[section], ms, s)
            self.assertLess(max(v for k, v in s.items() if k != section), ms, s)
        injected = sum(rows_of(f, "stall_diag_summary")[0]["injected"].get(section, 0) for f in files)
        self.assertEqual(injected, len(timing))
        return timing

    def test_positive_control_free(self):
        streams, files = self.diag_capture("rsd_cpu_mem", "memory:1:80:free")
        timing = self.assert_injected(files, "memory", 80)
        self.assertTrue(all(r["sections_cpu_ms"]["memory"] < 60 for r in timing))
        self.assertEqual(self.cg.diff_names(self.plain, streams, self.cg.strict), [])

    def test_positive_control_hold_before_gate(self):
        # hold: GIL held; a section before the gate also ages the geometry it is about to use
        streams, files = self.diag_capture("rsd_cpu_geo", "geometry:1:80:hold")
        timing = self.assert_injected(files, "geometry", 80)
        self.assertTrue(all(r["spans_ms"]["picked_to_gate"] >= 80 for r in timing))
        gil = [r for f in files for r in rows_of(f, "gil_stall")]
        self.assertGreaterEqual(len([g for g in gil if g["late_ms"] >= 40]), len(timing) // 2)


if __name__ == "__main__":
    unittest.main()
