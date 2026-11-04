"""Car geometry producer (host/geo_producer.py, host/geometry_topic.py, ros_runtime kv_topic).

Docs: docs/design/CAR_GEO_PRODUCER_RESULTS_20261011.md.

1. Producer equivalence on a rendered depth sequence (synthetic prepared set, the smoke's
   scene renderer, mm codes) driven along the replay trajectory: GeoProducer through kv-link
   to a separate-process fake KV (g++ frame core + C ROI core) equals the smoke's CPU producer
   delta for delta and grid for grid (through RollingLayerBridge), and every ROI row equals the
   local C reference. Positive controls: pose one frame late, grid in the camera frame, clock id
   "mono" -- each must be caught (grid differs, or the bridge refuses clock_mismatch).
2. GeoProducer never leaves a frame's ROI batch or codes for the next frame (a frame the
   pipeline drops before the backend), and withdraws the ROI answer of an expired frame.
3. check_block_height known answers (smoke mount, recommended mount, too-low block).
4. GeometryFrameCache: exact / nearest / out of tolerance, refusals (failure, foreign runtime,
   world frame, pose disagreement), memory order and uncovered tracks.
5. ros_runtime --geometry-source kv_topic on the M5 software replay with an emulated producer
   (glue plan_frame + the C ROI reference on the frame's own depth, published before the
   detections are handed over): stream equals the cpu stream after removing the rows kv_topic
   adds; shadow check equal on every frame. Positive control: the emulated producer answers on
   the previous frame's depth -> differs. No producer: bounded wait, then geometry_unavailable
   (no_geometry_frame) and fallback none / cpu as configured. Argument conflicts refused.
6. The rolling bridge and both replay producers use host.fpga_clock (no "mono" left).
Python 3.8+.
"""

import json
import math
import os
from pathlib import Path
import queue
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from host.geo_producer import check_block_height  # noqa: E402
from host.geometry_topic import GeometryFrameCache  # noqa: E402


def synthetic_prep(path, frames=24):
    """A prepared set rendered by the smoke's scene (64x48, mm codes) from poses along x."""
    sys.path.insert(0, str(ROOT / "robot_sim" / "tests"))
    import rolling_nav2_smoke as smoke
    os.makedirs(path)
    walls = smoke.WALLS + [smoke.BOX_A, smoke.BOX_B]
    codes = []
    for k in range(frames):
        depth, _, _ = smoke.render((-0.6 + 0.03 * k, 0.02 * math.sin(k / 3.0), 0.1 * math.sin(k / 5.0)), walls)
        c = np.where(np.isfinite(depth), np.rint(depth / 0.001), 0).astype("<u2")
        codes.append(c)
    arr = np.stack(codes)
    arr.tofile(os.path.join(path, "depth_u16.bin"))
    meta = {"width": smoke.IMG_W, "height": smoke.IMG_H, "frames": frames, "depth_file": "depth_u16.bin",
            "depth_unit_m": 0.001, "K": smoke.intrinsics().tolist(), "calibration_id": "synthetic-smoke"}
    with open(os.path.join(path, "meta.json"), "w") as fp:
        json.dump(meta, fp)
    return path


class ProducerEquivalence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="geoprod_test_")
        cls.prep = synthetic_prep(os.path.join(cls.tmp, "prep"))
        from tools.car_geo_producer import equivalence
        cls.eq = equivalence
        cls.results = {}
        for variant in ("none", "pose_lag1", "camera_frame", "clock_mono"):
            out = os.path.join(cls.tmp, "out_" + variant)
            rc = equivalence.main(["--prep", cls.prep, "--out", out, "--fake", "cpu", "--boxes", "8",
                                   "--variant", variant, "--timeout-s", "2.0"])
            with open(os.path.join(out, "summary.json")) as fp:
                cls.results[variant] = (rc, json.load(fp))

    def test_same_input_equal(self):
        rc, s = self.results["none"]
        t = s["tally"]
        sys.stderr.write("\n[producer-eq] frames %d deltas %d grids %d roi rows %d/%d changes %s grid %s\n"
                         % (t["frames"], t["delta_equal"], t["grid_equal"], t["roi_rows_equal"], t["roi_rows"],
                            t.get("subj_changes"), t.get("final_ref_grid")))
        self.assertEqual(rc, 0)
        self.assertTrue(s["all_equal"])
        self.assertEqual(t["delta_equal"], t["frames"])
        self.assertEqual(t["subj_fused"], t["frames"])
        self.assertGreater(t["subj_changes"], 100)
        self.assertGreater(t["final_ref_grid"]["lethal"], 0)
        self.assertGreater(t["final_ref_grid"]["free"], 0)
        self.assertEqual(t["bridge_subj"], {"applied": t["frames"]})

    def test_positive_controls_caught(self):
        for variant in ("pose_lag1", "camera_frame", "clock_mono"):
            rc, s = self.results[variant]
            self.assertEqual(rc, 0, variant)
            self.assertTrue(s["caught"], variant)
            self.assertFalse(s["all_equal"], variant)
        self.assertEqual(self.results["clock_mono"][1]["tally"]["bridge_subj"],
                         {"clock_mismatch": self.results["clock_mono"][1]["tally"]["subj_fused"]})


class _RecordingBackend(object):
    """Frame backend double with the remote-only hooks; records what each call carried."""
    backend_name = "remote_pl:fake"
    accepts_capture_identity = True

    def __init__(self, cpu):
        self.cpu, self.calls, self.lib = cpu, [], types.SimpleNamespace(last=None)
        self._next_roi = self._next_codes = None
        self.last_roi = None

    def set_next_roi(self, roi, labels=None):
        self._next_roi = roi
        self._next_labels = labels

    def set_next_codes(self, codes):
        self._next_codes = codes

    def frame_from_depth(self, *, identity=None, codes=None, **kw):
        # codes: passed by M7Pipeline for an all-invalid frame (10-26 #2 blank request); the CPU core takes none
        roi, nxt = self._next_roi, self._next_codes
        self._next_roi = self._next_codes = None
        self.calls.append({"frame_id": identity.frame_id, "roi": roi, "codes": nxt if codes is None else codes})
        self.last_roi = None
        return self.cpu.frame_from_depth(**kw)


class ProducerHygiene(unittest.TestCase):
    def make(self, clock):
        from host.geo_producer import GeoProducer
        from host.m7_backend import M7BackendSelector
        from host.m7_frame_native import compile_frame_native
        from host.m7_rolling import compile_rolling
        tmp = tempfile.mkdtemp(prefix="geoprod_hyg_")
        sel = M7BackendSelector(cpu_frame_library=compile_frame_native(os.path.join(tmp, "f.so")))
        backend = _RecordingBackend(sel.frame_backend("cpu"))
        roll = sel.rolling_backend("cpu", cpu_library=compile_rolling(os.path.join(tmp, "r.so")))
        return GeoProducer(backend, roll, depth_unit_m=0.001, clock=clock), backend

    def test_no_residue_after_dropped_frame_and_expired_roi_withdrawn(self):
        from tools.car_geo_producer.common import base_camera, odom_base
        sys.path.insert(0, str(ROOT / "robot_sim" / "tests"))
        import rolling_nav2_smoke as smoke
        now = [10 ** 12]
        prod, backend = self.make(lambda: now[0])
        depth, _, _ = smoke.render((-0.6, 0.0, 0.0), smoke.WALLS + [smoke.BOX_A])
        codes = np.where(np.isfinite(depth), np.rint(depth / 0.001), 0).astype(np.uint16)
        K = smoke.intrinsics()
        Toc = odom_base(0) @ base_camera()
        mem = {"runtime_boot_id": "b", "revision": 1, "world_frame": "map", "tracks": [
            {"id": "t1", "label": "cup", "bounds_m": [[0.55, -0.25, 0.0], [0.75, 0.05, 0.35]],
             "last_seen": 1.0, "position_m": [0.65, -0.1, 0.17]}]}
        kw = dict(T_odom_camera=Toc, robot_position_m=(0., 0., 0.), calibration_id="c", T_world_camera=Toc,
                  memory_msg=mem)
        # frame 1: 300 ms old at the geometry step -> dropped before the backend
        r1 = prod.process(codes, K, depth_stamp_ns=1, acquired_ns=now[0] - 300_000_000, **kw)
        self.assertEqual(r1["nav_reason"], "expired_frame")
        self.assertEqual(backend.calls, [])
        self.assertEqual(r1["body"]["failure"], "expired_output")
        self.assertIsNone(backend._next_roi)
        self.assertIsNone(backend._next_codes)
        # frame 2: fresh -> the backend sees exactly this frame's codes
        codes2 = codes.copy()
        codes2[0, 0] = 777
        now[0] += 100_000_000
        r2 = prod.process(codes2, K, depth_stamp_ns=2, acquired_ns=now[0] - 1_000_000, **kw)
        self.assertEqual(len(backend.calls), 1)
        self.assertEqual(int(backend.calls[0]["codes"][0, 0]), 777)
        self.assertIsNotNone(backend.calls[0]["roi"])
        self.assertEqual(r2["nav_reason"], "geometry_ready")
        self.assertEqual(r2["body"]["producer"]["depth_stamp_ns"], 2)
        # 10-27 console first-person boxes scale roi_xyxy by the depth pixel frame
        self.assertEqual(r2["body"]["producer"]["depth_wh"], [codes.shape[1], codes.shape[0]])

    def test_frame_age_cannot_be_relaxed(self):
        from host.geo_producer import GeoProducer
        with self.assertRaises(ValueError):
            GeoProducer(None, None, depth_unit_m=0.001, max_frame_age_ns=250_000_000)


class BlockHeight(unittest.TestCase):
    def test_known_answers(self):
        # float64 floor like FollowCameraGrid.cell_of: 0.30 / 0.05 = 5.999... -> camera cell 5
        ok, info = check_block_height(0.30, 0.0)                 # smoke mount: block -0.10..0.40
        self.assertTrue(ok)
        self.assertAlmostEqual(info["block_bottom_m"], -0.10)
        ok, info = check_block_height(0.36, 0.0)                 # cell 7: block bottom on the ground
        self.assertFalse(ok)
        self.assertEqual(info["need_height_cell_at_least"], 8)
        ok, info = check_block_height(0.36, 0.0, height_cell=8)
        self.assertTrue(ok)
        self.assertAlmostEqual(info["block_bottom_m"], -0.05)
        ok, _ = check_block_height(0.60, 0.0, height_cell=7)     # too high for the block
        self.assertFalse(ok)


def _body(stamp, *, boot="b1", world="map", failure=None, verdicts=(), T=None, fid=1):
    from host.region_verdicts import build_message
    body = build_message(depth_stamp_ns=stamp, capture_nano_ns=5, kv_frame_id=fid, world_frame=world,
                         T_world_camera=np.eye(4) if T is None else T, pose_valid=True, runtime_boot_id=boot,
                         memory_revision=3, verdicts=verdicts, failure=failure, roi_core="pl")
    body["producer"] = {"depth_stamp_ns": stamp, "acquired_raw_ns": 1000, "frame_id": fid}
    return json.dumps(body)


class _Mem(object):
    def __init__(self, ids):
        self.tracks = {i: None for i in ids}


class CacheTests(unittest.TestCase):
    V = [{"track_id": "b", "geometry_evidence": "occluded", "code": 3, "roi_xyxy": [0, 0, 1, 1], "counts": None},
         {"track_id": "a", "geometry_evidence": "out_of_view", "code": None, "roi_xyxy": None, "counts": None},
         {"track_id": "gone", "geometry_evidence": "free", "code": 2, "roi_xyxy": [0, 0, 1, 1], "counts": None}]

    def test_match_and_evidence_order(self):
        c = GeometryFrameCache("b1", "map", tolerance_ns=50_000_000)
        c.on_message(_body(1_000_000_000, verdicts=self.V), 2000)
        ev, reason, info = c.lookup(1_000_000_000, _Mem(["a", "b", "c"]), np.eye(4), now_raw_ns=3000)
        self.assertIsNone(reason)
        self.assertEqual(list(ev.items()), [("a", "out_of_view"), ("b", "occluded")])
        self.assertEqual(info["match"], "exact")
        self.assertEqual(info["uncovered"], 1)
        self.assertEqual(info["frame_age_ms"], 2000 / 1e6)
        ev, reason, info = c.lookup(1_030_000_000, _Mem(["a"]), np.eye(4), now_raw_ns=3000)
        self.assertEqual(info["match"], "nearest")
        self.assertAlmostEqual(info["match_dt_ms"], -30.0)
        ev, reason, _ = c.lookup(1_060_000_000, _Mem(["a"]), np.eye(4), now_raw_ns=3000)
        self.assertIsNone(ev)
        self.assertEqual(reason, "no_geometry_frame")

    def test_refusals(self):
        c = GeometryFrameCache("b1", "map", tolerance_ns=0)
        c.on_message(_body(1, failure="expired_output"), 0)
        c.on_message(_body(2, boot="other"), 0)
        c.on_message(_body(3, world="odom"), 0)
        T = np.eye(4)
        T[0, 3] = 0.05
        c.on_message(_body(4, T=T), 0)
        self.assertFalse(c.on_message("{not json", 0))
        m = _Mem(["a"])
        self.assertEqual(c.lookup(1, m, np.eye(4), now_raw_ns=0)[1], "expired_output")
        self.assertEqual(c.lookup(2, m, np.eye(4), now_raw_ns=0)[1], "foreign_runtime")
        self.assertEqual(c.lookup(3, m, np.eye(4), now_raw_ns=0)[1], "world_frame_mismatch")
        self.assertEqual(c.lookup(4, m, np.eye(4), now_raw_ns=0)[1], "pose_disagrees")
        T[0, 3] = 0.01
        c.on_message(_body(5, T=T), 0)
        self.assertIsNone(c.lookup(5, m, np.eye(4), now_raw_ns=0)[1])
        self.assertEqual(c.counts["malformed"], 1)

    def test_bounded(self):
        c = GeometryFrameCache("b1", "map", tolerance_ns=0, capacity=4)
        for s in range(10):
            c.on_message(_body(s), 0)
        self.assertEqual(sorted(c.bodies), [6, 7, 8, 9])


# ---------------------------------------------------------------- 5. runtime kv_topic
def emulated_body(node, frame, depth, n):
    """What the producer publishes for this frame, computed from the frame's own depth and the
    runtime's current memory with the producer's glue (plan_frame / verdict_message) and the
    local C ROI reference in place of the KV260."""
    from host.geometry_region_glue import plan_frame, verdict_message
    from host.memory_publish import track_rows
    from host.fpga_clock import now_ns
    from perception.fpga_replay.core import Config
    cfg = Config()
    mem = {"runtime_boot_id": node.runtime_boot_id, "revision": n, "world_frame": "map", "target_label": None,
           "tracks": track_rows(node.memory)}
    plan = plan_frame(mem, frame.depth.shape, frame.K, frame.T_world_camera, cfg)
    outcome = None
    if plan.batch is not None:
        rows = _ROI.evaluate_boxes(np.asarray(depth, dtype=np.float32),
                                   [(r.row_id, r.xyxy, r.near_z, r.far_z) for r in plan.batch.rows], cfg)
        outcome = types.SimpleNamespace(ok=True, rows=rows, roi_core="cpu-emulated", times_ns=None,
                                        roi_cycles=None, reason=None)
    stamp = int(round((frame.depth_timestamp if frame.depth_timestamp is not None else frame.timestamp) * 1e9))
    body = verdict_message(plan, outcome, memory_msg=mem, depth_stamp_ns=stamp, capture_nano_ns=now_ns(),
                           kv_frame_id=n, world_frame="map", T_world_camera=frame.T_world_camera,
                           pose_valid=True, cfg=cfg)
    body["producer"] = {"depth_stamp_ns": stamp, "acquired_raw_ns": now_ns(), "frame_id": n}
    return json.dumps(body)


_ROI = None
LATE_STATE = []


def capture_topic(source, name, extra, producer="same"):
    """capture() of the region-source regression, with a producer emulation hooked in front
    of the detections queue: 'same' (this frame's depth), 'prev' (previous frame's depth),
    None (no producer)."""
    global _ROI
    import test_region_source_none_regression as reg
    from host.m7_object_native import NativeObjectEvidence, compile_object_native
    if _ROI is None:
        _ROI = NativeObjectEvidence(compile_object_native(os.path.join(tempfile.mkdtemp(), "o.so")))
    original = reg.make_loader

    def make_loader(src, module, extra_args=None):
        load = original(src, module, extra_args)

        def load_runtime(world, path):
            node = load(world, path)
            if producer is None:
                return node
            state = {"n": 0, "prev": None}

            class Inject(queue.Queue):
                def put(self, item, *a, **k):
                    frame, _ = item
                    state["n"] += 1
                    depth = frame.depth if producer != "prev" or state["prev"] is None else state["prev"]
                    state["prev"] = frame.depth
                    msg = types.SimpleNamespace(data=emulated_body(node, frame, depth, state["n"]))
                    if producer == "late":
                        state["late"] = msg      # delivered after the poll that pops this frame
                    else:
                        node.on_geometry_frame(msg)
                    return queue.Queue.put(self, item, *a, **k)
            node.results = Inject(maxsize=1)
            if producer == "late":
                publish_memory = node.publish_memory

                def deliver_after_poll():
                    publish_memory()
                    msg, state["late"] = state.get("late"), None
                    if msg is not None:
                        state["held_before"] = state.get("held_before", 0) + (node.kv_hold is not None)
                        node.on_geometry_frame(msg)
                node.publish_memory = deliver_after_poll
                LATE_STATE.append(state)
            return node
        return load_runtime
    with patch.object(reg, "make_loader", make_loader):
        return reg.capture(source, name, extra)


def topic_args(**over):
    a = dict(geometry_source="kv_topic", geometry_fallback="none", kv_server=None, kv_expect_build_id=None,
             kv_depth_unit_m=None, kv_grid=None, kv_shadow_check=True, region_source="none",
             max_region_age_s=1.0, geometry_topic_tolerance_s=0.05, geometry_topic_wait_s=0.0)
    a.update(over)
    return a


class RuntimeKvTopic(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import test_car_geometry_kv as cg
        cls.cg = cg
        if cg.baseline_source() is None:
            raise unittest.SkipTest("git history with %s not available" % cg.BASELINE)
        cls.src = (ROOT / "host" / "ros_runtime.py").read_text(encoding="utf-8")
        cls.before, _ = cg.capture(cg.baseline_source(), "ros_runtime_topic_baseline")
        cls.topic, cls.topic_ok = capture_topic(cls.src, "ros_runtime_topic", topic_args())

    def test_stream_equals_cpu_and_shadow_equal(self):
        cg = self.cg
        self.assertTrue(all(ok for ok, _ in self.topic_ok.values()), self.topic_ok)
        self.assertEqual(cg.diff_names(self.before, self.topic, cg.kv_normalise), [])
        processed = cg.status_rows(self.topic, "frame_processed")
        self.assertGreater(len(processed), 10)
        self.assertTrue(all(p["geometry"]["result"] == "fused" and p["geometry"]["match"] == "exact"
                            for p in processed))
        closes = [c for c in cg.status_rows(self.topic, "geometry_close") if c.get("source") == "kv_topic"]
        frames = sum(c["shadow"]["frames"] for c in closes)
        self.assertEqual(frames, sum(c["shadow"]["equal"] for c in closes))
        boxes = sum(p["geometry"]["boxes"] for p in processed)
        sys.stderr.write("\n[kv_topic-m5] %d frames, shadow equal %d/%d, checked boxes %d\n"
                         % (len(processed), frames, frames, boxes))
        self.assertEqual(frames, len(processed))
        self.assertGreater(boxes, 0)

    def test_positive_control_previous_depth_differs(self):
        streams, _ = capture_topic(self.src, "ros_runtime_topic_prev", topic_args(kv_shadow_check=False),
                                   producer="prev")
        self.assertNotEqual(self.cg.diff_names(self.before, streams, self.cg.kv_normalise), [])

    def test_no_producer_waits_then_unavailable(self):
        cg = self.cg
        t0 = time.monotonic()
        streams, _ = capture_topic(self.src, "ros_runtime_topic_none",
                                   topic_args(kv_shadow_check=False, geometry_topic_wait_s=0.0), producer=None)
        rows = cg.status_rows(streams, "geometry_unavailable")
        self.assertTrue(rows and all(r["reason"] == "no_geometry_frame" for r in rows))
        self.assertIn("scenario_occlusion_changes_view", cg.diff_names(self.before, streams, cg.kv_normalise))
        fb, _ = capture_topic(self.src, "ros_runtime_topic_none_cpu",
                              topic_args(kv_shadow_check=False, geometry_fallback="cpu"), producer=None)
        self.assertEqual(cg.diff_names(self.before, fb, cg.kv_normalise), [])
        self.assertEqual(len(cg.status_rows(fb, "geometry_fallback_cpu")), len(cg.status_rows(fb, "frame_processed")))
        self.assertLess(time.monotonic() - t0, 600)

    def test_bounded_wait_holds_then_releases(self):
        # the body arrives only after the poll that popped the detections: that poll holds the
        # frame (wait 10 s, never reached), the body's arrival processes it with an exact match
        cg = self.cg
        del LATE_STATE[:]
        streams, ok = capture_topic(self.src, "ros_runtime_topic_late",
                                    topic_args(kv_shadow_check=False, geometry_topic_wait_s=10.0),
                                    producer="late")
        processed = cg.status_rows(streams, "frame_processed")
        self.assertEqual(len(processed), len(cg.status_rows(self.before, "frame_processed")))
        self.assertTrue(all(p["geometry"]["result"] == "fused" and p["geometry"]["match"] == "exact"
                            and p["geometry"]["waited_ms"] > 0 for p in processed))
        held = sum(st.get("held_before", 0) for st in LATE_STATE)
        self.assertEqual(held, len(processed))
        self.assertTrue(all(o for o, _ in ok.values()), ok)

    def test_argument_conflicts(self):
        import host.ros_runtime as rr
        for extra in (["--region-source", "kv"], ["--kv-server", "1.2.3.4:5"]):
            with self.assertRaises(SystemExit):
                with patch("sys.stderr"):
                    rr.main(["--socket", "x", "--pose-source", "reference", "--sensor-source", "hardware",
                             "--geometry-source", "kv_topic"] + extra)


class ClockUnification(unittest.TestCase):
    def test_no_mono_clock_left(self):
        from host import fpga_clock
        src = (ROOT / "host" / "ros_rolling_nav.py").read_text()
        self.assertTrue("from host.fpga_clock import CLOCK_ID, now_ns" in src)
        self.assertFalse('CLOCK_ID = "mono"' in src or "def now_ns" in src)
        for rel in ("robot_sim/tests/rolling_nav2_smoke.py", "tools/nano_partition/n5_map_replay.py"):
            text = (ROOT / rel).read_text()
            self.assertFalse('"mono"' in text or "return time.monotonic_ns()" in text, rel)
        self.assertEqual(fpga_clock.CLOCK_ID, "nano_monotonic_raw_ns")


if __name__ == "__main__":
    unittest.main()
