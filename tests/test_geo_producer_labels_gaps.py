"""Producer AR labels and frame-gap causes (2026-10-11 integration fixes).

1. AR labels: the producer sends with every ROI batch the class name of the memory track
   behind each row, in row order (10-11 board smoke: the producer sent none, the AR boxes read
   R+row). Three tracks with distinct labels; the labels must follow plan.row_map, not the
   memory list order (variant: memory order -> caught when the plan reorders rows by priority).
   roi_labels=False sends none; a track without a label -> None (shown as R+row by the KV260).
   Through the real RemotePlFrameBackend.set_next_roi: wire names, cut at 8 bytes.
2. FrameGapTracker: startup (no good frame yet) vs running gaps; one event per gap start and
   one when frames flow again (geometry_ready / geometry_resumed with missed count and gap_ms);
   counters per reason and phase.
Python 3.8+, no ROS.
"""
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host.geo_producer import FrameGapTracker, roi_row_labels  # noqa: E402


class _Backend(object):
    backend_name = "remote_pl:fake"
    accepts_capture_identity = True

    def __init__(self, cpu):
        self.cpu, self.calls, self.lib = cpu, [], types.SimpleNamespace(last=None)
        self._next_roi = self._next_codes = self._next_labels = None
        self.last_roi = None

    def set_next_roi(self, roi, labels=None):
        self._next_roi, self._next_labels = roi, labels

    def set_next_codes(self, codes):
        self._next_codes = codes

    def frame_from_depth(self, *, identity=None, **kw):
        self.calls.append({"roi": self._next_roi, "labels": self._next_labels})
        self._next_roi = self._next_codes = self._next_labels = None
        return self.cpu.frame_from_depth(**kw)


def _scene():
    from tools.car_geo_producer.common import base_camera, odom_base
    sys.path.insert(0, str(ROOT / "robot_sim" / "tests"))
    import rolling_nav2_smoke as smoke
    depth, _, _ = smoke.render((-0.6, 0.0, 0.0), smoke.WALLS + [smoke.BOX_A])
    codes = np.where(np.isfinite(depth), np.rint(depth / 0.001), 0).astype(np.uint16)
    return codes, smoke.intrinsics(), odom_base(0) @ base_camera()


def _memory(target="bottle"):
    tracks = []
    for i, (lab, y) in enumerate((("cup", -0.12), ("toothbrush", 0.0), ("bottle", 0.12))):
        tracks.append({"id": "t%d" % i, "label": lab, "bounds_m": [[0.55, y - 0.04, 0.0], [0.65, y + 0.04, 0.2]],
                       "last_seen": 1.0 + i, "position_m": [0.60, y, 0.1]})
    tracks.append({"id": "t_nolabel", "bounds_m": [[0.85, -0.05, 0.0], [0.95, 0.05, 0.2]],
                   "last_seen": 0.5, "position_m": [0.9, 0.0, 0.1]})
    return {"runtime_boot_id": "b", "revision": 3, "world_frame": "map", "target_label": target, "tracks": tracks}


class ProducerLabels(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from host.m7_backend import M7BackendSelector
        from host.m7_frame_native import compile_frame_native
        from host.m7_rolling import compile_rolling
        tmp = tempfile.mkdtemp(prefix="geoprod_lab_")
        cls.sel = M7BackendSelector(cpu_frame_library=compile_frame_native(os.path.join(tmp, "f.so")))
        cls.tmp, cls.compile_rolling = tmp, staticmethod(compile_rolling)
        cls.codes, cls.K, cls.Toc = _scene()

    def run_one(self, mem, roi_labels=True):
        from host.geo_producer import GeoProducer
        now = [10 ** 12]
        backend = _Backend(self.sel.frame_backend("cpu"))
        roll = self.sel.rolling_backend("cpu", cpu_library=self.compile_rolling(
            tempfile.mkdtemp(prefix="r", dir=self.tmp) + "/r.so"))
        prod = GeoProducer(backend, roll, depth_unit_m=0.001, clock=lambda: now[0], roi_labels=roi_labels)
        prod.process(self.codes, self.K, depth_stamp_ns=1, acquired_ns=now[0] - 1_000_000,
                     T_odom_camera=self.Toc, robot_position_m=(0., 0., 0.), calibration_id="c",
                     T_world_camera=self.Toc, memory_msg=mem)
        self.assertEqual(len(backend.calls), 1)
        return backend.calls[0], prod

    def test_labels_follow_rows(self):
        mem = _memory()
        call, prod = self.run_one(mem)
        rows = call["roi"].rows
        self.assertGreaterEqual(len(rows), 3)
        from host.geometry_region_glue import plan_frame
        by_id = {t["id"]: t.get("label") for t in mem["tracks"]}
        plan = plan_frame(mem, self.codes.shape, self.K, self.Toc, prod.cfg)
        expect = [by_id[plan.row_map[r.row_id]] for r in plan.batch.rows]
        self.assertEqual(call["labels"], expect)
        self.assertEqual(prod.counts["roi_labels_sent"], 1)
        # rows are in priority order (target first), so memory list order differs -> a variant
        # labelling by list position is caught
        naive = [t.get("label") for t in mem["tracks"]][:len(rows)]
        self.assertNotEqual(naive, call["labels"])
        self.assertEqual(call["labels"][0], "bottle")                     # target first
        if len(rows) == 4:
            self.assertIn(None, call["labels"])

    def test_roi_labels_off(self):
        call, prod = self.run_one(_memory(), roi_labels=False)
        self.assertIsNotNone(call["roi"])
        self.assertIsNone(call["labels"])
        self.assertEqual(prod.counts["roi_labels_sent"], 0)

    def test_real_backend_wire_names(self):
        from host.kv_link import RemotePlFrameBackend
        from host.geometry_region_glue import plan_frame
        mem = _memory()
        from perception.fpga_replay.core import Config
        plan = plan_frame(mem, self.codes.shape, self.K, self.Toc, Config())
        be = RemotePlFrameBackend.__new__(RemotePlFrameBackend)
        import collections
        be.counts = collections.Counter()
        be.set_next_roi(plan.batch, labels=roi_row_labels(mem, plan))
        names = dict(zip([mem["tracks"][i]["label"] for i in range(3)], [b"cup", b"toothbru", b"bottle"]))
        got = be._next_labels
        for lab, wire in names.items():
            self.assertIn(wire, got)
        self.assertEqual(be.counts["label:cut_to_8"], 1)


class Gaps(unittest.TestCase):
    def test_startup_then_running(self):
        g = FrameGapTracker()
        self.assertEqual(g.state(), "startup")
        e = g.missing("tf_missing", 100)
        self.assertEqual((e["event"], e["phase"], e["reason"]), ("geometry_gap", "startup", "tf_missing"))
        self.assertIsNone(g.missing("tf_missing", 200))
        self.assertEqual(g.state(), "gap_startup:tf_missing")
        e = g.ok(1_000_100)
        self.assertEqual((e["event"], e["phase"], e["missed"], e["gap_ms"]), ("geometry_ready", "startup", 2, 1.0))
        self.assertIsNone(g.ok(2_000_000))
        self.assertEqual(g.state(), "ready")
        e = g.missing("tf_missing", 3_000_000, tf_arrivals_1s=0)
        self.assertEqual((e["phase"], e["tf_arrivals_1s"]), ("running", 0))
        for t in range(3):
            g.missing("tf_missing", 3_000_001 + t)
        e = g.ok(2_403_000_000)
        self.assertEqual((e["event"], e["phase"], e["missed"]), ("geometry_resumed", "running", 4))
        self.assertAlmostEqual(e["gap_ms"], 2400.0)
        self.assertEqual(dict(g.counts), {"tf_missing:startup": 2, "gaps:startup": 1,
                                          "tf_missing:running": 4, "gaps:running": 1})

    def test_ready_without_gap(self):
        g = FrameGapTracker()
        e = g.ok(5)
        self.assertEqual(e, {"event": "geometry_ready", "source": "geo_producer", "raw_ns": 5})


if __name__ == "__main__":
    unittest.main()
