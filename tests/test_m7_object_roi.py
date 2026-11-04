"""Mask-free multi-box region check: kernel vs independent golden vs core.py.

docs/design/OBJECT_ROI_SPEC_20261011.md section 4.1: core.py compares float32
depth with np.float64 front/back scalars. NumPy 1.x (value-based casting)
compares in float32, the kernel's reading; NumPy 2.x (NEP 50) compares in
float64. Both readings are reported here; no threshold is tuned to agree.
"""
import ctypes
from dataclasses import replace
from pathlib import Path
import shutil
import tempfile
import unittest

import numpy as np

from host.m7_object_sizes import CLASS_SIZE_M, synthetic_bounds
from host.m7_object_native import (NativeObjectEvidence, ROI_MAX_BOXES,
                                   compile_object_native)
from perception.fpga_replay.core import Config, Frame, Memory, Track, compute_geometry


NUMPY2 = int(np.__version__.split(".")[0]) >= 2
F32_TENTH = float(np.float32(0.1))  # validity is float32 in both NumPy versions


def golden_box(depth, roi, near, far, cfg, reading):
    """Per-pixel Python loop; reading 'single' or 'double' for front/back."""
    x0, y0, x1, y1 = roi
    front = near - cfg.depth_margin_m
    back = far + cfg.depth_margin_m
    if reading == "single":
        front, back = float(np.float32(front)), float(np.float32(back))
    pixels = valid = ahead = behind = 0
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            pixels += 1
            z = float(depth[y][x])
            if not (z == z) or not (z > F32_TENTH and z < 10.0):
                continue
            valid += 1
            ahead += z < front
            behind += z > back
    if valid < cfg.min_pixels or valid / pixels < cfg.min_valid_fraction:
        reason = "invalid_depth"
    elif ahead / valid >= .5:
        reason = "occluded"
    elif behind / valid >= cfg.free_fraction:
        reason = "free"
    else:
        reason = "surface_or_uncertain"
    return (pixels, valid, ahead, behind), reason


def core_frame(depth, k, pose, stamp=1.0, name="f"):
    h, w = depth.shape
    return Frame(name, stamp, np.zeros((h, w, 3), np.uint8), depth, k, pose)


def core_memory(bounds_list):
    memory = Memory()
    for i, bounds in enumerate(bounds_list):
        key = "object-%04d" % (i + 1)
        lower, upper = np.asarray(bounds[0], float), np.asarray(bounds[1], float)
        memory.tracks[key] = Track(key, "cup", "red", (lower + upper) / 2,
                                   lower.copy(), upper.copy(), 0.)
    return memory


def first_float32_rounding(direction):
    """Smallest near (>1.5) whose float32(near +/- .04) rounds the given way."""
    for step in range(1, 100000):
        near = 1.5 + step * 1e-4
        if direction == "down":
            exact = near - .04
            if float(np.float32(exact)) < exact:
                return near, float(np.float32(exact))
        else:
            exact = near + .04
            if float(np.float32(exact)) > exact:
                return near, float(np.float32(exact))
    raise AssertionError("no rounding example found")


class ObjectRoiBatchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="m7_object_roi_")
        cls.native = NativeObjectEvidence(compile_object_native(Path(cls.tmp) / "object.so"))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def counts(self, row):
        return (row["roi_pixels"], row["roi_valid_pixels"],
                row["roi_front_pixels"], row["roi_behind_pixels"])

    def test_random_scenes_match_single_precision_golden(self):
        rng = np.random.default_rng(20261011)
        cfg = Config()
        double_differs = 0
        for trial in range(60):
            h, w = int(rng.integers(20, 50)), int(rng.integers(20, 60))
            levels = np.array([.05, .5, 1., 1.86, 2., 2.14, 3., 9.99, 12.], np.float32)
            depth = rng.choice(levels, size=(h, w)).astype(np.float32)
            depth += rng.normal(0, .02, size=(h, w)).astype(np.float32)
            depth[rng.random((h, w)) < .1] = np.nan
            depth[rng.random((h, w)) < .05] = 0.
            boxes = []
            for b in range(int(rng.integers(1, ROI_MAX_BOXES + 1))):
                if rng.random() < .15:
                    boxes.append((b, None, 0., 0.)); continue
                x0 = int(rng.integers(0, w - 4)); y0 = int(rng.integers(0, h - 4))
                x1 = int(rng.integers(x0 + 3, w)); y1 = int(rng.integers(y0 + 3, h))
                near = float(rng.uniform(.5, 3.)); far = near + float(rng.uniform(0, .5))
                boxes.append((b, (x0, y0, x1, y1), near, far))
            rows = self.native.evaluate_boxes(depth, boxes, cfg)
            for (b, roi, near, far), row in zip(boxes, rows):
                self.assertEqual(row["object_id"], b)
                if roi is None:
                    self.assertEqual(self.counts(row) + (row["enabled"], row["verdict_code"]),
                                     (0, 0, 0, 0, 0, 0))
                    continue
                single, single_reason = golden_box(depth, roi, near, far, cfg, "single")
                double, _ = golden_box(depth, roi, near, far, cfg, "double")
                self.assertEqual(self.counts(row), single, (trial, b))
                double_differs += single != double
                code = {"occluded": 3, "free": 2}.get(single_reason, 0)
                self.assertEqual(row["verdict_code"], code)
        # Random noise essentially never lands exactly on a rounded threshold.
        self.assertEqual(double_differs, 0)

    def test_boundary_pixel_splits_readings_and_core_numpy_versions(self):
        cfg = Config()
        near, front32 = first_float32_rounding("down")
        far_b, back32 = first_float32_rounding("up")
        k = np.array([[10., 0., 0.], [0., 10., 0.], [0., 0., 1.]])
        # Occlusion edge: 6x4 ROI, 11 clear front pixels + 1 at front32.
        depth = np.full((20, 30), np.float32(near + .05), np.float32)
        depth[2:4, 2:8] = 1.
        depth[3, 7] = np.float32(front32)
        # Free edge: 5x5 ROI, 22 clear behind + 1 at back32 + 2 on surface.
        depth[10:15, 10:15] = 5.
        depth[10, 10] = depth[10, 11] = np.float32(far_b - .01)
        depth[10, 12] = np.float32(back32)
        occ = ((2, 2, 7, 5), near, near + .1)
        free = ((10, 10, 14, 14), far_b - .1, far_b)
        rows = self.native.evaluate_boxes(depth, [(0,) + occ, (1,) + free], cfg)
        expect = {}
        for name, (roi, n, f), row in (("occ", occ, rows[0]), ("free", free, rows[1])):
            single, single_reason = golden_box(depth, roi, n, f, cfg, "single")
            double, double_reason = golden_box(depth, roi, n, f, cfg, "double")
            self.assertEqual(self.counts(row), single)
            expect[name] = (single, single_reason, double, double_reason)
        self.assertEqual(expect["occ"][0][2], 11)
        self.assertEqual(expect["occ"][2][2], 12)
        self.assertEqual((expect["occ"][1], expect["occ"][3]),
                         ("surface_or_uncertain", "occluded"))
        self.assertEqual(expect["free"][0][3], 22)
        self.assertEqual(expect["free"][2][3], 23)
        self.assertEqual((expect["free"][1], expect["free"][3]),
                         ("surface_or_uncertain", "free"))
        self.assertEqual([r["verdict_code"] for r in rows], [0, 0])
        # core.py on exactly these ROIs: identity pose, fx=fy=10, cx=cy=0.
        # Lower x/y chosen so floor(10*x/far) = x0, upper so ceil(10*x/near) = x1.
        def bounds_for(roi, n, f):
            x0, y0, x1, y1 = roi
            return np.array([[(x0 + .5) * f / 10, (y0 + .5) * f / 10, n],
                             [(x1 - .5) * n / 10, (y1 - .5) * n / 10, f]])
        memory = core_memory([bounds_for(*occ), bounds_for(*free)])
        core = compute_geometry(core_frame(depth, k, np.eye(4)), memory, cfg)
        ours = self.native.check_boxes(depth, k, np.eye(4),
                                       [("a", bounds_for(*occ)), ("b", bounds_for(*free))], cfg)
        self.assertEqual([d["roi_xyxy"] for _, _, d in ours], [list(occ[0]), list(free[0])])
        self.assertEqual([e for _, e, _ in ours], ["surface_or_uncertain"] * 2)
        if NUMPY2:
            # NumPy 2: float64 comparison = the double reading.
            self.assertEqual(list(core.values()), ["occluded", "free"])
        else:
            # NumPy 1.x: value-based casting compares in float32 = the kernel.
            self.assertEqual(list(core.values()), ["surface_or_uncertain"] * 2)
        print("\n[boundary] numpy %s core.py=%s kernel=%s front32=%r back32=%r"
              % (np.__version__, list(core.values()), [e for _, e, _ in ours],
                 front32, back32))

    def test_core_compute_geometry_agreement(self):
        rng = np.random.default_rng(7)
        cfg = Config()
        k = np.array([[50., 0., 32.], [0., 50., 24.], [0., 0., 1.]])
        tally = {}
        mismatches = []
        for trial in range(40):
            angle = float(rng.uniform(-.2, .2))
            c, s = np.cos(angle), np.sin(angle)
            pose = np.eye(4)
            pose[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
            pose[:3, 3] = rng.uniform(-.2, .2, 3)
            depth = np.full((48, 64), np.float32(rng.uniform(2.5, 5.)), np.float32)
            bounds_list = []
            for b in range(int(rng.integers(1, 40))):
                center_cam = np.array([rng.uniform(-1.2, 1.2), rng.uniform(-.9, .9),
                                       rng.uniform(.3, 3.)])
                half = rng.uniform(.02, .25, 3)
                center = pose[:3, :3] @ center_cam + pose[:3, 3]
                bounds_list.append(np.array([center - half, center + half]))
                # Paint something for this box: surface, occluder or nothing.
                u = int(k[0, 0] * center_cam[0] / center_cam[2] + k[0, 2])
                v = int(k[1, 1] * center_cam[1] / center_cam[2] + k[1, 2])
                r = int(max(1, 40 * half[0] / center_cam[2]))
                kind = rng.integers(0, 4)
                z = (center_cam[2] if kind == 0 else center_cam[2] - .5 if kind == 1 else
                     np.nan if kind == 3 else None)
                if z is not None and not z <= .2:
                    depth[max(0, v - r):v + r, max(0, u - r):u + r] = np.float32(z)
            depth[rng.random(depth.shape) < .08] = np.nan
            frame = core_frame(depth, k, pose, stamp=float(trial + 1), name="t%d" % trial)
            memory = core_memory(bounds_list)
            core = compute_geometry(frame, memory, cfg)
            entries = [("object-%04d" % (i + 1), bd) for i, bd in enumerate(bounds_list)]
            ours = self.native.check_boxes(depth, k, pose, entries, cfg)
            for (key, bd), (verdict, evidence, details) in zip(entries, ours):
                tally[evidence] = tally.get(evidence, 0) + 1
                if core[key] != evidence:
                    mismatches.append((trial, key, core[key], evidence))
        self.assertEqual(mismatches, [])
        print("\n[core.py agreement] numpy %s boxes=%d %s" % (np.__version__, sum(tally.values()),
                                                          sorted(tally.items())))
        # Every core.py outcome is exercised.
        for reason in ("out_of_view", "too_small", "invalid_depth", "occluded",
                       "free", "surface_or_uncertain"):
            self.assertGreater(tally.get(reason, 0), 0, (reason, tally))

    def test_host_capacity_constant_matches_header(self):
        header = (Path(__file__).resolve().parents[1] / "hw/m7/object_evidence.hpp").read_text()
        self.assertIn("FPGA_M7_ROI_MAX_BOXES = %d;" % ROI_MAX_BOXES, header)

    def test_batches_larger_than_capacity_split(self):
        cfg = Config()
        k = np.array([[50., 0., 32.], [0., 50., 24.], [0., 0., 1.]])
        depth = np.full((48, 64), np.float32(4.), np.float32)
        entries = [("o%d" % i, np.array([[-.2, -.2, 1.9 + i * .01], [.2, .2, 2.1 + i * .01]]))
                   for i in range(ROI_MAX_BOXES + 9)]
        ours = self.native.check_boxes(depth, k, np.eye(4), entries, cfg)
        self.assertEqual(len(ours), ROI_MAX_BOXES + 9)
        self.assertEqual({e for _, e, _ in ours}, {"free"})
        self.assertEqual([d["object_id"] for _, _, d in ours], [e[0] for e in entries])

    def test_oversized_table_box_shrunk_does_not_call_present_object_free(self):
        """Owner rule (10-11): boxes may be too small, never too large.

        The real cup is 30% smaller than its table entry. The object is
        rendered as its front face at 2 m against a 3 m wall.
        """
        cfg = Config()
        fx = 125.
        k = np.array([[fx, 0., 80.], [0., fx, 60.], [0., 0., 1.]])
        (w_t, d_t, h_t), _ = CLASS_SIZE_M["cup"]
        real = np.array([min(w_t, d_t), h_t, min(w_t, d_t)]) / 1.3  # camera y is up/down
        center = np.array([0., 0., 2.])
        depth = np.full((120, 160), np.float32(3.), np.float32)
        z_front = center[2] - real[2] / 2
        u0, u1 = (fx * (center[0] + np.array([-1, 1]) * real[0] / 2) / z_front + 80)
        v0, v1 = (fx * (center[1] + np.array([-1, 1]) * real[1] / 2) / z_front + 60)
        depth[int(np.ceil(v0)):int(np.floor(v1)) + 1,
              int(np.ceil(u0)):int(np.floor(u1)) + 1] = np.float32(z_front)
        verdicts = {}
        for name, bounds in (
                ("shrink_0.8", synthetic_bounds("cup", center, 1)),
                ("shrink_1.0", synthetic_bounds("cup", center, 1, shrink=1.)),
                ("box_4x_real", np.array([center - 2 * real, center + 2 * real]))):
            (verdict, evidence, details), = self.native.check_boxes(
                depth, k, np.eye(4), [(name, bounds)], cfg)
            row = details["kernel_row"]
            verdicts[name] = (evidence, row["roi_behind_pixels"], row["roi_valid_pixels"])
        print("\n[oversize] %s" % verdicts)
        self.assertEqual(verdicts["shrink_0.8"][0], "surface_or_uncertain")
        self.assertNotEqual(verdicts["shrink_1.0"][0], "free")
        # The danger direction is real: a box far larger than the object
        # sees mostly wall and calls a present object free.
        self.assertEqual(verdicts["box_4x_real"][0], "free")
        with self.assertRaises(ValueError):
            synthetic_bounds("cup", center, 1, shrink=1.2)
        with self.assertRaises(ValueError):
            synthetic_bounds("cup", center, 3)
        flat = synthetic_bounds("cell phone", center, 1)
        self.assertAlmostEqual(flat[1, 0] - flat[0, 0], .007 * .8)

    def test_rejections_leave_outputs_untouched(self):
        depth = np.full((20, 20), np.float32(3.), np.float32)
        with self.assertRaises(ValueError):
            self.native.evaluate_boxes(depth, [], Config())
        with self.assertRaises(ValueError):
            self.native.evaluate_boxes(depth, [(i, None, 0., 0.)
                                               for i in range(ROI_MAX_BOXES + 1)])
        with self.assertRaises(ValueError):
            self.native.evaluate_boxes(depth.astype(np.float64), [(0, None, 0., 0.)])
        with self.assertRaises(ValueError):
            self.native.evaluate_boxes(depth, [(0, (2, 2, 4, 6), 1., 2.)])
        with self.assertRaises(ValueError):
            self.native.evaluate_boxes(depth, [(0, (2, 2, 6, 6), 2., 1.)])
        with self.assertRaises(ValueError):
            self.native.evaluate_boxes(depth, [(0, (2, 2, 6, 6), 1., 2.)],
                                       Config(free_fraction=.8))
        # Raw ABI: count N+1 is rejected by the kernel itself, result intact.
        lib = self.native.lib
        n = ROI_MAX_BOXES
        result = np.full(7 * n, 0xFFFFFFFF, np.uint32)
        status = np.zeros(1, np.uint32)
        lib.fpga_m7_object_roi_batch(
            np.ascontiguousarray(depth), 20, 20, n + 1, np.zeros(n, np.uint32),
            np.zeros(4 * n, np.uint32), np.zeros(n), np.zeros(n), np.zeros(n, np.uint32),
            .04, 9, .7, .9, result, status)
        self.assertEqual(int(status[0]), 3)
        self.assertTrue(np.all(result == 0xFFFFFFFF))
        enabled = np.zeros(n, np.uint32); enabled[0] = 2
        lib.fpga_m7_object_roi_batch(
            np.ascontiguousarray(depth), 20, 20, 1, enabled,
            np.zeros(4 * n, np.uint32), np.zeros(n), np.zeros(n), np.zeros(n, np.uint32),
            .04, 9, .7, .9, result, status)
        self.assertEqual(int(status[0]), 3)
        self.assertTrue(np.all(result == 0xFFFFFFFF))
        self.assertIsInstance(lib.fpga_m7_object_roi_batch, ctypes._CFuncPtr)


def _pipeline_frame(seq, depth):
    from host.m7_pipeline import CaptureIdentity, DepthFrame, PoseAtCapture
    stamp = 1_000_000_000 + seq * 10_000_000
    identity = CaptureIdentity("stream-r", seq, stamp, "clk", "K-25", "odom", "epoch-1")
    pose = PoseAtCapture(stamp, "clk", "odom", "epoch-1", True, np.eye(4))
    k = np.array([[12., 0., 12.], [0., 12., 12.], [0., 0., 1.]])
    return DepthFrame(identity, depth.copy(), k, pose)


class RegionPipelineAndMemoryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from host.m7_native import NativeM7, compile_native
        cls.tmp = tempfile.mkdtemp(prefix="m7_roi_pipe_")
        cls.region = NativeObjectEvidence(compile_object_native(Path(cls.tmp) / "object.so"))
        cls.voxel = NativeM7(compile_native(Path(cls.tmp) / "voxel.so"))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    BOUNDS = np.array([[-.25, -.25, 1.5], [.25, .25, 2.5]])

    def engine(self, region=True, **kw):
        from host.m7_pipeline import GridSpec, M7Pipeline
        return M7Pipeline(self.voxel, GridSpec((-2., -2., -1.), .25, (16, 16, 20)),
                          region_backend=self.region if region else None, **kw)

    def check(self, engine, f, entries, token=None, now=None):
        from host.m7_pipeline import RegionCheckPacket
        g = engine.ingest_geometry(f, now_ns=f.identity.acquired_ns + 1,
                                   now_clock_id="clk")
        return g, engine.ingest_region_checks(
            RegionCheckPacket(f.identity, token or g.navigation.frame_token, entries),
            now_ns=(now or f.identity.acquired_ns) + 2, now_clock_id="clk")

    def scenes(self):
        removed = np.full((25, 25), 3., np.float32)
        occluded = removed.copy(); occluded[10:15, 10:15] = 1.
        present = removed.copy(); present[10:15, 10:15] = 2.
        return {"removed": removed, "occluded": occluded, "present": present}

    def test_requires_explicit_backend_and_yields_region_channel(self):
        with self.assertRaisesRegex(ValueError, "explicit region_backend"):
            self.check(self.engine(region=False), _pipeline_frame(1, self.scenes()["removed"]),
                       (("cup", self.BOUNDS),))
        for name, verdict, evidence in (("removed", "free_space_conflict", "free"),
                                        ("occluded", "occluded", "occluded"),
                                        ("present", "indeterminate", "surface_or_uncertain")):
            engine = self.engine()
            _, (out,) = self.check(engine, _pipeline_frame(1, self.scenes()[name]),
                                   (("cup", self.BOUNDS),))
            self.assertTrue(out.valid)
            self.assertEqual((out.channel, out.payload["verdict"],
                              out.payload["geometry_evidence"]), ("region", verdict, evidence))
            self.assertNotIn("candidate", out.payload)
            self.assertEqual(out.payload["roi_xyxy"], [10, 10, 14, 14])

    def test_tokens_duplicates_capacity_and_revocation(self):
        engine = self.engine(max_objects_per_frame=2)
        f = _pipeline_frame(1, self.scenes()["removed"])
        _, outs = self.check(engine, f, (("a", self.BOUNDS), ("a", self.BOUNDS),
                                         ("b", self.BOUNDS), ("c", self.BOUNDS)))
        self.assertEqual([(o.valid, o.reason) for o in outs],
                         [(True, "free"), (False, "duplicate_object_frame"),
                          (True, "free"), (False, "object_capacity_exceeded")])
        self.assertTrue(all(o.channel == "region" for o in outs))
        checked = engine.validate_output(outs[0], now_ns=f.identity.acquired_ns + 5,
                                         clock_id="clk")
        self.assertTrue(checked.valid)
        engine.reset()
        checked = engine.validate_output(outs[0], now_ns=f.identity.acquired_ns + 5,
                                         clock_id="clk")
        self.assertFalse(checked.valid)
        engine2 = self.engine()
        _, bad = self.check(engine2, f, (("a", self.BOUNDS), ("b", self.BOUNDS)),
                            token="forged-token")
        self.assertEqual([(o.valid, o.reason, o.channel, o.payload["object_id"]) for o in bad],
                         [(False, "frame_token_mismatch", "region", "a"),
                          (False, "frame_token_mismatch", "region", "b")])
        # Mask path and region path each admit the same object once per frame.
        from host.m7_pipeline import SemanticPacket
        engine3 = self.engine()
        g, (out,) = self.check(engine3, f, (("a", self.BOUNDS),))
        sem = engine3.ingest_semantic(
            SemanticPacket(f.identity, g.navigation.frame_token, "a",
                           np.zeros((25, 25), np.bool_), self.BOUNDS),
            now_ns=f.identity.acquired_ns + 3, now_clock_id="clk")
        self.assertTrue(out.valid and sem.valid)
        self.assertEqual(sem.channel, "memory")

    def _track_memory(self, state, negatives=0):
        memory = core_memory([np.array([[-.25, -.25, 1.5], [.25, .25, 2.5]])])
        t = memory.tracks["object-0001"]
        t.state = state
        for i in range(negatives):
            t.negative_poses.append((-10. - i, np.eye(4)))
        return memory

    def _core_frame(self, name, stamp, translation=(0., 0., 0.)):
        pose = np.eye(4); pose[:3, 3] = translation
        depth = np.full((25, 25), 3., np.float32)
        k = np.array([[12., 0., 12.], [0., 12., 12.], [0., 0., 1.]])
        return core_frame(depth, k, pose, stamp=stamp, name=name)

    def test_geometry_only_entry_equals_update_memory_for_unaffected_states(self):
        evidences = ("occluded", "free", "surface_or_uncertain", "invalid_depth", "out_of_view")
        for state in ("historical_memory", "position_invalid"):
            for negatives in (0, 1):
                for ev in evidences:
                    a, b = self._track_memory(state, negatives), self._track_memory(state, negatives)
                    a.update_memory([], {"object-0001": ev}, self._core_frame("f1", 1.))
                    b.update_geometry_only({"object-0001": ev}, self._core_frame("f1", 1.))
                    ta, tb = a.tracks["object-0001"], b.tracks["object-0001"]
                    self.assertEqual((ta.state, ta.reason, len(ta.negative_poses)),
                                     (tb.state, tb.reason, len(tb.negative_poses)),
                                     (state, negatives, ev))

    def test_geometry_only_start_state_table(self):
        """Spec s.6 table: no leading downgrade in the geometry-only entry."""
        cases = {
            ("current_confirmed", "occluded"): ("temporarily_occluded", "temporarily_occluded"),
            ("current_confirmed", "free"): ("historical_memory", "current_confirmed"),
            ("current_confirmed", "surface_or_uncertain"): ("historical_memory", "current_confirmed"),
            ("temporarily_occluded", "surface_or_uncertain"):
                ("historical_memory", "temporarily_occluded"),
            ("temporarily_occluded", "free"): ("historical_memory", "temporarily_occluded"),
        }
        for (state, ev), (old_state, new_state) in cases.items():
            a, b = self._track_memory(state), self._track_memory(state)
            a.update_memory([], {"object-0001": ev}, self._core_frame("f1", 1.))
            b.update_geometry_only({"object-0001": ev}, self._core_frame("f1", 1.))
            self.assertEqual((a.tracks["object-0001"].state, b.tracks["object-0001"].state),
                             (old_state, new_state), (state, ev))
        # Two independent free views invalidate a confirmed position.
        m = self._track_memory("current_confirmed")
        m.cfg = replace(m.cfg, existence_v3=False)   # 10-27: legacy two-view rule (existence_v3=False); v3 counterpart in tests/test_memory_existence_20261027.py
        m.update_geometry_only({"object-0001": "free"}, self._core_frame("g1", 1.))
        m.update_geometry_only({"object-0001": "free"}, self._core_frame("g2", 1.1))
        self.assertEqual(m.tracks["object-0001"].state, "current_confirmed")  # too soon
        m.update_geometry_only({"object-0001": "free"}, self._core_frame("g3", 2., (.5, 0, 0)))
        self.assertEqual(m.tracks["object-0001"].state, "position_invalid")

    def test_geometry_clock_is_separate_from_detector_clock(self):
        m = self._track_memory("historical_memory")
        m.update_geometry_only({"object-0001": "surface_or_uncertain"}, self._core_frame("f5", 5.))
        # A detector batch for the same acquisition frame still goes through.
        out = m.update_memory([], {"object-0001": "surface_or_uncertain"},
                              self._core_frame("f5", 5.))
        self.assertIsNone(out["ignored"])
        self.assertEqual(m.update_geometry_only({}, self._core_frame("f5", 5.))["ignored"],
                         "duplicate_frame")
        with self.assertRaisesRegex(ValueError, "out-of-order geometry"):
            m.update_geometry_only({}, self._core_frame("f4", 4.))
        with self.assertRaisesRegex(ValueError, "unknown tracks"):
            m.update_geometry_only({"object-9999": "free"}, self._core_frame("f6", 6.))
        self.assertEqual(m.seen_frames, {"f5"})
        self.assertEqual(m.last_timestamp, 5.)

    def test_region_outputs_feed_geometry_only_memory(self):
        engine = self.engine()
        memory = self._track_memory("current_confirmed")
        memory.cfg = replace(memory.cfg, existence_v3=False)   # 10-27: legacy two-view rule (existence_v3=False); v3 counterpart in tests/test_memory_existence_20261027.py
        for seq, x in ((1, 0.), (2, .5)):
            f = _pipeline_frame(seq, self.scenes()["removed"])
            _, outs = self.check(engine, f, (("object-0001", self.BOUNDS),))
            evidence = {o.payload["object_id"]: o.payload["geometry_evidence"]
                        for o in outs if o.valid}
            memory.update_geometry_only(evidence, self._core_frame("p%d" % seq, float(seq), (x, 0, 0)))
        self.assertEqual(memory.tracks["object-0001"].state, "position_invalid")


if __name__ == "__main__":
    unittest.main()
