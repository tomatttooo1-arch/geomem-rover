"""P3 fixed-camera chain (tools/nano_loop): synthetic frames through the real M7 CPU reference.

Covers the detector-mask decode, original-frame binding of a late detection, the M4 document,
and the J4 replay check including its deliberately wrong variant.
"""
import tempfile
import unittest
from pathlib import Path

import numpy as np

from host.m7_frame_native import FrameNativeM7, compile_frame_native
from host.m7_object_native import NativeObjectEvidence, compile_object_native
from perception.offline_memory import OfflineMemory
from tools.nano_detector.nano_detector import Letterbox, rle_encode
from tools.nano_loop.chain import FixedCameraChain, calibration_record, depth_mm_to_m, mask_full
from tools.nano_loop.replay import MODES, run_scenario
from tools.nano_loop.query import summarize

W, H = 640, 480
K = [[517.0, 0.0, 320.0], [0.0, 517.0, 240.0], [0.0, 0.0, 1.0]]
BOX = (280, 200, 360, 300)          # object pixels, original frame
T0 = 1_790_000_000_000_000_000
PERIOD = 66_666_666


def detection(box=BOX, name="bottle", score=0.9):
    lb = Letterbox(W, H, 640)
    s = 4  # prototype stride, network pixels (r = 1 for 640x480)
    x0, y0 = box[0] // s, (box[1] + lb.top) // s
    x1, y1 = box[2] // s, (box[3] + lb.top) // s
    cells = np.ones((y1 - y0, x1 - x0), dtype=bool)
    return {"cls": 39, "name": name, "score": score, "box": [float(v) for v in box],
            "mask": {"stride_px": 4.0, "box_cells": [x0, y0, x1, y1], "shape": list(cells.shape),
                     "rle": rle_encode(cells), "area_px": int(cells.sum()) * 16}}


def scene(object_mm=500, background_mm=1000):
    d = np.full((H, W), background_mm, dtype=np.uint16)
    d[BOX[1]:BOX[3], BOX[0]:BOX[2]] = object_mm
    return d


class NanoLoopChainTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        root = Path(cls.tmp.name)
        cls.fb = FrameNativeM7(compile_frame_native(root / "frame.so"))
        cls.ob = NativeObjectEvidence(compile_object_native(root / "object.so"))
        cls.calib = calibration_record("TESTSERIAL", "1.0", K, W, H)
        cls.lb = Letterbox(W, H, 640)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def chain(self):
        return FixedCameraChain(self.fb, self.ob, self.calib, "unit", letterbox=self.lb,
                                prediction_set="unit_set", width=W, height=H)

    def test_depth_conversion_marks_zero_missing(self):
        d = depth_mm_to_m(np.array([[0, 1000]], dtype=np.uint16))
        self.assertTrue(np.isnan(d[0, 0]))
        self.assertAlmostEqual(float(d[0, 1]), 1.0, places=6)
        self.assertEqual(float(depth_mm_to_m(np.array([[0]], dtype=np.uint16), 9.9)[0, 0]), np.float32(9.9))

    def test_mask_decode_lands_on_original_pixels(self):
        m = mask_full(detection(), self.lb, W, H)
        ys, xs = np.nonzero(m)
        self.assertEqual((xs.min(), ys.min(), xs.max(), ys.max()), (BOX[0], BOX[1], BOX[2] - 1, BOX[3] - 1))

    def test_late_detection_uses_original_frame_depth(self):
        c = self.chain()
        c.ingest_frame(0, T0, scene(500), T0 + 1_000_000, rgb_sha256="a" * 64)
        # a newer frame arrives with the object region at a different depth before the detection returns
        c.ingest_frame(2, T0 + PERIOD, scene(800), T0 + PERIOD + 1_000_000, rgb_sha256="b" * 64)
        res, doc = c.consume_detections(0, [detection()], T0 + PERIOD + 5_000_000)
        self.assertTrue(res["late"])
        row = res["instances"][0]
        self.assertEqual(row["candidate"], "support_present")
        self.assertAlmostEqual(row["position_m"][2], 0.5, places=3)   # frame 0 depth, not frame 2
        self.assertIsNotNone(doc)
        mem = OfflineMemory()
        mem.import_document(doc)
        out = summarize(mem, "bottle")
        self.assertEqual(len(out["candidates"]), 1)
        self.assertEqual(out["scope"], "offline_candidates_not_current_presence_or_navigation")

    def test_detection_after_cache_window_is_rejected_not_rebound(self):
        c = self.chain()
        for k in range(6):
            c.ingest_frame(2 * k, T0 + k * PERIOD, scene(500), T0 + k * PERIOD + 1_000_000,
                           rgb_sha256="%064x" % k)
        res, doc = c.consume_detections(0, [detection()], T0 + 5 * PERIOD + 2_000_000)
        self.assertEqual(res["instances"][0]["candidate"], "rejected")
        self.assertIn(res["instances"][0]["reason"], ("frame_not_cached", "expired_frame"))
        self.assertIsNone(doc)

    def frames(self, n=14):
        return [{"color_index": 2 * k, "acquired_ns": T0 + k * PERIOD, "rgb_sha256": "%064x" % (k + 1),
                 "depth": scene(500)} for k in range(n)]

    def test_replay_invalid_depth_never_removes_and_positive_control_does(self):
        frames = self.frames()
        dets = {f["color_index"]: [detection()] for f in frames}
        results = {m: run_scenario(frames, dets, self.chain, m, (7, 12)) for m in MODES}
        for mode in ("zero_all", "zero_region", "occlude_region"):
            self.assertEqual(results[mode]["check"], "PASS", results[mode])
            self.assertEqual(results[mode]["free_space_conflict_frames"], 0)
        self.assertEqual(results["zero_all"]["geometry_invalid_frames"], 5)
        self.assertEqual(results["zero_region"]["outcomes"],
                         {"indeterminate:invalid_historical_region_depth": 5})
        self.assertEqual(results["occlude_region"]["outcomes"],
                         {"indeterminate:historical_region_occluded": 5})
        self.assertEqual(results["remove_object"]["check"], "PASS")
        self.assertEqual(results["remove_object"]["free_space_conflict_frames"], 5)

    def test_replay_refuses_target_whose_region_never_reaches_depth(self):
        # an object at the image edge: its historical region projects out of view, so the
        # removal check would be vacuous; the replay must refuse instead of reporting PASS
        edge = (0, 200, 60, 300)
        frames = self.frames()
        for f in frames:
            f["depth"] = np.full((H, W), 1000, dtype=np.uint16)
            f["depth"][edge[1]:edge[3], edge[0]:edge[2]] = 500
        dets = {f["color_index"]: [detection(box=edge)] for f in frames}
        with self.assertRaises(RuntimeError):
            run_scenario(frames, dets, self.chain, "zero_region", (7, 12))

    def test_wrong_variant_invalid_as_free_is_caught(self):
        frames = self.frames()
        dets = {f["color_index"]: [detection()] for f in frames}
        for mode in ("zero_all", "zero_region"):
            r = run_scenario(frames, dets, self.chain, mode, (7, 12), wrong=True)
            self.assertEqual(r["check"], "FAIL", r)
            self.assertGreater(r["free_space_conflict_frames"], 0)


if __name__ == "__main__":
    unittest.main()
