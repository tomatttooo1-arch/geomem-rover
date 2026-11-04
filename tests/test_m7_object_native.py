"""Object kernel versus old Python evidence and asynchronous original-frame cache."""
import ctypes
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

import numpy as np

from host.m7_native import NativeM7, compile_native
from host.m7_object_native import NativeObjectEvidence, compile_object_native
from host.m7_pipeline import (CaptureIdentity, DepthFrame, GridSpec, M7Pipeline,
                              PoseAtCapture, SemanticPacket)
from perception.fpga_replay.core import Config, backproject


ROOT = Path(__file__).resolve().parents[1]
K = np.array([[12., 0., 12.], [0., 12., 12.], [0., 0., 1.]])
T = np.eye(4)
BOUNDS = np.array([[-.25, -.25, 1.5], [.25, .25, 2.5]])
GRID = GridSpec((-2., -2., -1.), .25, (16, 16, 20))
ROI = (10, 10, 14, 14)


def scene(kind):
    depth = np.full((25, 25), 3., np.float32)
    mask = np.zeros((25, 25), np.bool_)
    if kind == "present":
        depth[10:15, 10:15] = 2.
        mask[10:15, 10:15] = True
    elif kind == "occluded":
        depth[10:15, 10:15] = 1.
    elif kind == "invalid":
        depth[10:15, 10:15] = np.nan
    elif kind == "removed":
        pass
    else:
        raise ValueError(kind)
    return depth, mask


def frame(seq, depth):
    stamp = 1_000_000_000 + seq * 10_000_000
    identity = CaptureIdentity("stream-a", seq, stamp, "camera-clock", "K-25-v1",
                               "odom", "epoch-1")
    pose = PoseAtCapture(stamp, "camera-clock", "odom", "epoch-1", True, T.copy())
    return DepthFrame(identity, depth.copy(), K.copy(), pose)


def packet(result, f, mask, object_id="cup"):
    return SemanticPacket(f.identity, result.navigation.frame_token,
                          object_id, mask, BOUNDS.copy())


class M7ObjectNativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="m7-object-")
        cls.addClassCleanup(cls.tmp.cleanup)
        path = Path(cls.tmp.name)
        cls.object_native = NativeObjectEvidence(compile_object_native(path / "object.so"))
        cls.voxel_native = NativeM7(compile_native(path / "voxel.so"))

    def pipeline(self, object_backend=None):
        return M7Pipeline(self.voxel_native, GRID, object_backend=object_backend)

    def geometry(self, engine, f):
        return engine.ingest_geometry(f, now_ns=f.identity.acquired_ns + 1,
                                      now_clock_id=f.identity.clock_id)

    def semantic(self, engine, pkt, now_frame):
        return engine.ingest_semantic(pkt, now_ns=now_frame.identity.acquired_ns + 2,
                                      now_clock_id=now_frame.identity.clock_id)

    def test_mask_points_match_existing_double_backproject_and_cpu_median(self):
        depth, mask = scene("present")
        t = np.eye(4)
        angle = .2
        t[:3, :3] = [[np.cos(angle), 0, np.sin(angle)], [0, 1, 0],
                     [-np.sin(angle), 0, np.cos(angle)]]
        t[:3, 3] = [.3, -.2, .1]
        raw = self.object_native.evaluate(depth, mask, K, t)
        self.assertEqual(raw.candidate, "support_present")
        self.assertEqual(raw.counts["input_pixels"], 625)
        self.assertEqual(raw.counts["mask_pixels"], 25)
        self.assertEqual(raw.counts["projected_valid_points"], 25)
        v, u = np.where(mask)
        reference = backproject(np.column_stack((u, v)), depth[mask], K, t)
        np.testing.assert_allclose(raw.points_local_m, reference, rtol=0, atol=2e-15)
        np.testing.assert_allclose(np.median(raw.points_local_m, axis=0),
                                   np.median(reference, axis=0), rtol=0, atol=2e-15)

    def test_counts_and_states_match_old_python_for_fixed_cases(self):
        for kind, expected, reason in (("present", "support_present", "matched_mask_depth"),
                                       ("removed", "free_space_conflict", "conservative_free_depth"),
                                       ("occluded", "indeterminate", "historical_region_occluded"),
                                       ("invalid", "indeterminate", "invalid_historical_region_depth")):
            with self.subTest(kind=kind):
                depth, mask = scene(kind)
                old = self.pipeline()
                new = self.pipeline(self.object_native)
                f = frame(1, depth)
                old_g, new_g = self.geometry(old, f), self.geometry(new, f)
                old_ev = self.semantic(old, packet(old_g, f, mask), f)
                new_ev = self.semantic(new, packet(new_g, f, mask), f)
                self.assertEqual(new_ev.payload["candidate"], expected)
                self.assertEqual(new_ev.payload["candidate"], old_ev.payload["candidate"])
                self.assertEqual(new_ev.reason, reason)
                self.assertEqual(new_ev.reason, old_ev.reason)
                self.assertEqual(new_ev.payload.get("region_valid_pixels"),
                                 old_ev.payload.get("region_valid_pixels"))
                self.assertEqual(new_ev.payload.get("mask_valid_pixels"),
                                 old_ev.payload.get("mask_valid_pixels"))
                if kind == "present":
                    np.testing.assert_allclose(new_ev.payload["position_local_m"],
                                               old_ev.payload["position_local_m"], atol=2e-15)
                    self.assertEqual(new_ev.payload["position_summary_method"],
                                     "cpu_median_from_kernel_points")

    def test_missing_mask_uses_no_free_even_when_roi_is_clear(self):
        depth, _ = scene("removed")
        raw = self.object_native.evaluate(depth, None, K, T,
                                           roi=ROI, near_z=1.5, far_z=2.5)
        self.assertEqual(raw.counts["roi_behind_pixels"], 25)
        self.assertEqual(raw.candidate, "indeterminate")
        engine = self.pipeline(self.object_native)
        f = frame(1, depth)
        g = self.geometry(engine, f)
        ev = self.semantic(engine, packet(g, f, None), f)
        self.assertEqual(ev.payload["candidate"], "indeterminate")
        self.assertEqual(ev.reason, "missing_mask")

    def test_history_association_and_out_of_view_stay_on_host(self):
        depth, mask = scene("present")
        engine = self.pipeline(self.object_native)
        f = frame(1, depth)
        geometry = self.geometry(engine, f)
        distant = np.array([[1., 1., 1.], [1.2, 1.2, 1.2]])
        pkt = SemanticPacket(f.identity, geometry.navigation.frame_token,
                             "cup", mask, distant)
        evidence = self.semantic(engine, pkt, f)
        self.assertEqual(evidence.payload["candidate"], "indeterminate")
        self.assertEqual(evidence.reason, "mask_not_in_historical_bounds")
        self.assertEqual(evidence.payload["position_summary_method"],
                         "cpu_median_from_kernel_points")
        removed, negative = scene("removed")
        e2 = self.pipeline(self.object_native)
        f2 = frame(1, removed)
        geometry2 = self.geometry(e2, f2)
        far_bounds = np.array([[20., 20., 1.5], [21., 21., 2.5]])
        out = self.semantic(e2, SemanticPacket(f2.identity,
             geometry2.navigation.frame_token, "cup", negative, far_bounds), f2)
        self.assertEqual(out.payload["candidate"], "indeterminate")
        self.assertEqual(out.reason, "historical_region_out_of_view")

    def test_roi_threshold_boundaries_and_more_conservative_config(self):
        for nfree, expected in ((22, "indeterminate"), (23, "free_space_conflict")):
            with self.subTest(nfree=nfree):
                depth, mask = scene("removed")
                depth[10:15, 10:15].flat[nfree:] = 2.
                raw = self.object_native.evaluate(depth, mask, K, T, roi=ROI,
                          near_z=1.5, far_z=2.5)
                self.assertEqual(raw.counts["roi_behind_pixels"], nfree)
                self.assertEqual(raw.candidate, expected)
                strict = Config(free_fraction=.96)
                strict_raw = self.object_native.evaluate(depth, mask, K, T, roi=ROI,
                             near_z=1.5, far_z=2.5, cfg=strict)
                self.assertEqual(strict_raw.candidate, "indeterminate")
        for invalid_z in (np.nan, np.float32(.1), np.float32(10.)):
            depth, mask = scene("removed")
            depth[10:15, 10:15].flat[:8] = invalid_z
            raw = self.object_native.evaluate(depth, mask, K, T, roi=ROI,
                      near_z=1.5, far_z=2.5)
            self.assertEqual(raw.counts["roi_valid_pixels"], 17)
            self.assertEqual(raw.candidate, "indeterminate")

    def test_float32_validity_and_front_back_adjacent_ulp(self):
        depth = np.full((4, 4), np.nan, np.float32)
        mask = np.ones((4, 4), np.bool_)
        values = np.array([np.float32(.1), np.nextafter(np.float32(.1),
                           np.float32(np.inf)), np.float32(10.),
                           np.nextafter(np.float32(10.), np.float32(0.))])
        depth.flat[:4] = values
        raw = self.object_native.evaluate(depth, mask, K, T, roi=(0, 0, 3, 3),
                                           near_z=1.5, far_z=2.5)
        self.assertEqual(raw.counts["projected_valid_points"], 2)
        self.assertEqual(raw.counts["roi_valid_pixels"], 2)
        self.assertEqual(raw.candidate, "indeterminate")

        front = np.float32(1.5 - .04)
        back = np.float32(2.5 + .04)
        depths = np.array([np.nextafter(front, np.float32(0)), front,
                           np.nextafter(front, np.float32(np.inf)),
                           np.nextafter(back, np.float32(0)), back,
                           np.nextafter(back, np.float32(np.inf))], np.float32)
        sample = np.tile(depths, 3)[:16].reshape(4, 4)
        raw = self.object_native.evaluate(sample, np.zeros_like(sample, np.bool_), K, T,
                                           roi=(0, 0, 3, 3), near_z=1.5, far_z=2.5)
        good = np.isfinite(sample) & (sample > .1) & (sample < 10)
        self.assertEqual(raw.counts["roi_front_pixels"],
                         int(np.count_nonzero(sample[good] < 1.5 - .04)))
        self.assertEqual(raw.counts["roi_behind_pixels"],
                         int(np.count_nonzero(sample[good] > 2.5 + .04)))

    def test_bad_mask_roi_capacity_and_float64_reject_without_result(self):
        depth, mask = scene("removed")
        with self.assertRaises(ValueError):
            self.object_native.evaluate(depth.astype(np.float64), mask, K, T)
        with self.assertRaises(ValueError):
            self.object_native.evaluate(depth, mask[:24], K, T)
        with self.assertRaises(ValueError):
            self.object_native.evaluate(depth, mask, K, T, roi=(10, 10, 12, 14),
                                        near_z=1.5, far_z=2.5)
        with self.assertRaises(ValueError):
            self.object_native.evaluate(np.zeros((1, 1281), np.float32), None, K, T)
        bad_mask = mask.astype(np.uint8)
        bad_mask.flat[-1] = 2
        with self.assertRaises(ValueError):
            self.object_native.evaluate(depth, bad_mask, K, T)
        with self.assertRaises(ValueError):
            self.object_native.evaluate(depth, mask, K, T,
                roi=ROI, near_z=1.5, far_z=2.5, cfg=Config(free_fraction=.5))
        with self.assertRaises(ValueError):
            self.object_native.evaluate(depth, mask, K, T,
                cfg=Config(min_pixels=2 ** 32 + 9))

    def test_raw_c_abi_rejection_preserves_output_sentinels(self):
        depth, mask = scene("removed")
        mask = mask.astype(np.uint8)
        mask.flat[-1] = 2
        rectangle = np.array(ROI, dtype=np.uint32)
        inv = np.ascontiguousarray(np.linalg.inv(K), dtype=np.float64)
        t = np.ascontiguousarray(T[:3], dtype=np.float64)
        xyz = np.full(3 * 1280 * 800, 777., dtype=np.float64)
        counts = np.full(8, 777, dtype=np.uint32)
        status = np.array([99], dtype=np.uint32)
        self.object_native.lib.fpga_m7_object_evidence(depth, mask, 25, 25, 1, 1,
            rectangle, 1.5, 2.5, .04, inv, t, 9, .7, .9, xyz, counts, status)
        self.assertEqual(int(status[0]), 2)
        self.assertTrue(np.all(counts == 777))
        self.assertEqual(xyz[0], 777.)
        rectangle[:] = [10, 10, 12, 14]
        mask.flat[-1] = 0
        self.object_native.lib.fpga_m7_object_evidence(depth, mask, 25, 25, 1, 1,
            rectangle, 1.5, 2.5, .04, inv, t, 9, .7, .9, xyz, counts, status)
        self.assertEqual(int(status[0]), 3)
        self.assertTrue(np.all(counts == 777))
        self.assertEqual(xyz[0], 777.)

    def test_maximum_frame_and_invalid_returns_do_not_make_free(self):
        depth = np.full((800, 1280), np.nan, np.float32)
        raw = self.object_native.evaluate(depth, None, K, T,
            roi=(0, 0, 1279, 799), near_z=1.5, far_z=2.5)
        self.assertEqual(raw.counts["input_pixels"], 1_024_000)
        self.assertEqual(raw.counts["roi_pixels"], 1_024_000)
        self.assertEqual(raw.counts["roi_valid_pixels"], 0)
        self.assertEqual(raw.candidate, "indeterminate")
        depth[-1, -1] = 2.
        mask = np.zeros_like(depth, dtype=np.bool_)
        mask[-1, -1] = True
        last = self.object_native.evaluate(depth, mask, K, T)
        self.assertEqual(last.counts["input_pixels"], 1_024_000)
        self.assertEqual(last.counts["mask_pixels"], 1)
        self.assertEqual(last.counts["projected_valid_points"], 1)
        self.assertEqual(last.candidate, "indeterminate")

    def test_late_original_frame_semantic_and_wrong_frame_rejection(self):
        engine = self.pipeline(self.object_native)
        old_depth, mask = scene("present")
        old = frame(1, old_depth)
        original = self.geometry(engine, old)
        new_depth, negative = scene("removed")
        newer = frame(2, new_depth)
        current = self.geometry(engine, newer)
        old.depth_m[:] = 3.  # cached copy must remain the original depth.
        late = self.semantic(engine, packet(original, old, mask), newer)
        self.assertTrue(late.valid)
        self.assertEqual(late.payload["candidate"], "support_present")
        self.assertEqual(late.payload["position_local_m"][2], 2.)
        cleared = self.semantic(engine, packet(current, newer, negative), newer)
        self.assertEqual(cleared.payload["candidate"], "free_space_conflict")
        wrong = self.semantic(engine, packet(original, newer, mask, "other"), newer)
        self.assertFalse(wrong.valid)
        engine.reset()
        self.assertFalse(self.semantic(engine, packet(original, old, mask, "other"), newer).valid)

    def test_object_backend_failure_is_invalid_not_python_fallback(self):
        class Broken:
            def candidate(self, *args):
                raise RuntimeError("deliberate error")
        depth, mask = scene("present")
        f = frame(1, depth)
        engine = self.pipeline(Broken())
        geometry = self.geometry(engine, f)
        failed = self.semantic(engine, packet(geometry, f, mask), f)
        self.assertFalse(failed.valid)
        self.assertEqual(failed.reason, "object_backend_failure:RuntimeError")
        engine.object_backend = self.object_native
        recovered = self.semantic(engine, packet(geometry, f, mask), f)
        self.assertTrue(recovered.valid)
        self.assertEqual(recovered.payload["candidate"], "support_present")
        f64 = frame(2, depth.astype(np.float64))
        geometry2 = self.geometry(engine, f64)
        rejected = self.semantic(engine, packet(geometry2, f64, mask), f64)
        self.assertFalse(rejected.valid)
        self.assertEqual(rejected.reason, "object_backend_failure:ValueError")

    def test_backend_cannot_override_reserved_consumer_fields(self):
        class MaliciousDetails:
            def candidate(self, *args):
                return "indeterminate", "fake", {"candidate": "free_space_conflict",
                                                  "object_id": "wrong", "association": "trusted"}
        depth, mask = scene("present")
        f = frame(1, depth)
        engine = self.pipeline(MaliciousDetails())
        geometry = self.geometry(engine, f)
        rejected = self.semantic(engine, packet(geometry, f, mask), f)
        self.assertFalse(rejected.valid)
        self.assertEqual(rejected.reason, "object_backend_failure:ValueError")
        self.assertEqual(rejected.payload, {})

    def test_standalone_cpp_testbench(self):
        compiler = shutil.which("g++")
        binary = Path(self.tmp.name) / "object-testbench"
        subprocess.run([compiler, "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-ffp-contract=off", str(ROOT / "hw/m7/object_evidence.cpp"),
                        str(ROOT / "hw/m7/object_testbench.cpp"), "-o", str(binary)],
                       check=True, capture_output=True, text=True)
        output = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
        self.assertIn("M7 object evidence C++ testbench PASS", output.stdout)


if __name__ == "__main__":
    unittest.main()
