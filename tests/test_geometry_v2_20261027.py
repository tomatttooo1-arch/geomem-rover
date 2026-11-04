"""Geometry v2 (Config.geometry_v2, docs/design/GEOMETRY_V2_20261027.md): synthetic RGB-D cases.

Camera 0.39 m above the floor, pitched 7 deg down (the car's mount), 640x480, f = 380 px. Objects are
axis-aligned boxes ray-cast into an optical-Z depth image (the 10-27 diagnosis renderer). Covers: floor
bottle taken away (still / moving) -> v2 free where v1 is not; object present at 0.6-2 m and several
heights -> v2 never free; bottle 2 cm from a wall (recorded as measured); object on a cabinet; a hand in
front -> occluded; a transparent bottle -> the see-through filter stops free; half out of the image ->
never free; CPU words == KV words (the C reference of the region core, through the producer's plan).
"""
import dataclasses
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from perception.fpga_replay import core, geometry_v2  # noqa: E402
from perception.fpga_replay.see_through import SeeThroughFilter  # noqa: E402

W, H = 640, 480
K = np.array([[380., 0, 320], [0, 380., 240], [0, 0, 1]])
CFG1 = core.Config(**core.UNGATED_CREATION)
CFG2 = dataclasses.replace(CFG1, geometry_v2=True)


def pose(x=0.0, y=0.0, h=0.39, pitch_deg=7.0):
    """world x forward, y left, z up; camera optical z forward, x right, y down."""
    p = np.radians(pitch_deg)
    r0 = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    T = np.eye(4); T[:3, :3] = ry @ r0; T[:3, 3] = [x, y, h]
    return T


def render(T, boxes=(), wall_x=None):
    v, u = np.mgrid[0:H, 0:W]
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, float)], -1)
    d = rays @ T[:3, :3].T; o = T[:3, 3]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(d[..., 2] < -1e-6, -o[2] / d[..., 2], np.inf)
        if wall_x is not None:
            t = np.minimum(t, np.where(d[..., 0] > 1e-6, (wall_x - o[0]) / d[..., 0], np.inf))
        for lo, hi in boxes:
            t1 = (lo - o) / d; t2 = (hi - o) / d
            tn = np.nanmax(np.minimum(t1, t2), -1); tf = np.nanmin(np.maximum(t1, t2), -1)
            t = np.where((tn <= tf) & (tn > 0) & (tn < t), tn, t)
    z = t.astype(float); z[~np.isfinite(z)] = np.nan   # ray z-component is 1: optical Z = t
    return z


def frame(i, T, depth):
    f = core.Frame("f%d" % i, i * 0.133, np.zeros((H, W, 3), np.uint8), depth, K, T)
    f.ground_z_m = 0.0
    return f


def bottle(x, y=0.0, base=0.0, r=0.035, height=0.20):
    return np.array([x - r, y - r, base]), np.array([x + r, y + r, base + height])


def detection(f, box):
    lo, hi = box; T = f.T_world_camera
    cam = (np.array([[a, b, c] for a in (lo[0], hi[0]) for b in (lo[1], hi[1]) for c in (lo[2], hi[2])])
           - T[:3, 3]) @ T[:3, :3]
    uv = cam @ K.T; uv = uv[:, :2] / uv[:, 2:]
    return core.Detection("bottle", 0.9, (*uv.min(0), *uv.max(0)), frame_id=f.frame_id, timestamp=f.timestamp)


def remembered(box, T, cfg=CFG2, scene=(), wall=None):
    """Memory with one bottle track built from 3 frames at pose T."""
    m = core.Memory("geometry", cfg)
    for i in range(3):
        f = frame(i, T, render(T, list(scene) + [box], wall))
        obs = core.process_frame(f, [detection(f, box)], cfg)
        m.update_memory(obs, {}, f)
    assert len(m.tracks) == 1, m.rejected
    return m, next(iter(m.tracks.values()))


def words(m, tid, T, boxes=(), wall=None, i=50):
    f = frame(i, T, render(T, boxes, wall))
    return core.compute_geometry(f, m, CFG1)[tid], core.compute_geometry(f, m, CFG2)[tid]


class TestRemoved(unittest.TestCase):
    def test_floor_bottle_removed_car_still(self):
        T = pose()
        m, t = remembered(bottle(1.0), T)
        v1, v2 = words(m, t.id, T)
        self.assertEqual(v1, "surface_or_uncertain")      # the 10-27 diagnosis (R2)
        self.assertEqual(v2, "free")

    def test_floor_bottle_removed_car_moving(self):
        m, t = remembered(bottle(1.0), pose())
        got = [words(m, t.id, pose(x=0.02 * k), i=50 + k) for k in range(15)]   # creeping to 0.72 m
        self.assertTrue(all(v2 == "free" for _, v2 in got), got)
        self.assertTrue(all(v1 != "free" for v1, _ in got), got)

    def test_removed_various_ranges_floor(self):
        for x in (0.7, 1.0, 1.3, 1.6, 2.0):
            m, t = remembered(bottle(x), pose())
            self.assertEqual(words(m, t.id, pose())[1], "free", x)

    def test_cabinet_top_object_removed(self):
        cabinet = (np.array([1.1, -0.4, 0.0]), np.array([1.7, 0.4, 0.45]))
        b = bottle(1.25, base=0.45)
        m, t = remembered(b, pose(), scene=[cabinet], wall=3.0)
        v1, v2 = words(m, t.id, pose(), boxes=[cabinet], wall=3.0)
        self.assertEqual(v2, "free", (v1, v2))
        self.assertNotEqual(words(m, t.id, pose(), boxes=[cabinet, b], wall=3.0)[1], "free")


class TestPresent(unittest.TestCase):
    def test_present_never_free(self):
        seen = 0
        for x in (0.6, 0.8, 1.0, 1.3, 1.6, 2.0):
            for base, wall in ((0.0, None), (0.0, 2.6), (0.15, 2.6), (0.30, 2.6)):
                box = bottle(x, base=base)
                scene = [] if base == 0 else [(np.array([x - .1, -.1, 0.]), np.array([x + .1, .1, base]))]
                try:
                    m, t = remembered(box, pose(), scene=scene, wall=wall)
                except AssertionError:
                    continue                                 # not creatable from this view (clipped)
                for k in range(5):                           # small pose jitter: 1 cm, 0.5 deg
                    T = pose(x=0.01 * (k - 2), pitch_deg=7.0 + 0.5 * (k - 2))
                    self.assertNotEqual(words(m, t.id, T, boxes=scene + [box], wall=wall, i=60 + k)[1], "free",
                                        (x, base, wall, k))
                    seen += 1
        self.assertGreater(seen, 60)

    def test_occluded_by_hand(self):
        m, t = remembered(bottle(1.2), pose())
        hand = (np.array([0.55, -0.12, 0.0]), np.array([0.62, 0.12, 0.40]))
        self.assertEqual(words(m, t.id, pose(), boxes=[hand])[1], "occluded")             # bottle gone
        self.assertEqual(words(m, t.id, pose(), boxes=[hand, bottle(1.2)])[1], "occluded")

    def test_half_out_of_view_never_free(self):
        m, t = remembered(bottle(1.0), pose())
        for y in (0.6, 0.72, 0.78, 0.82, -0.78):           # camera slides sideways: bottle at / over the image edge
            v1, v2 = words(m, t.id, pose(y=y))
            if v2 == "free":
                # only allowed when the whole padded box is still inside the image
                T = pose(y=y)
                reason, *_ = geometry_v2.plan_one(T, K, W, H, t.lower, t.upper, t.position, 0.0, CFG2)
                self.assertIsNone(reason)
                cam = (np.array([[a, b, c] for a in (t.lower[0], t.upper[0]) for b in (t.lower[1], t.upper[1])
                                 for c in (t.lower[2], t.upper[2])]) - T[:3, 3]) @ T[:3, :3]
                uv = cam @ K.T; uv = uv[:, :2] / uv[:, 2:]
                self.assertTrue((uv[:, 0] >= 0).all() and (uv[:, 0] < W).all())
            else:
                self.assertIn(v2, ("out_of_view", "surface_or_uncertain", "too_small"))
        self.assertEqual(words(m, t.id, pose(y=0.78))[1], "out_of_view")   # bottle half out of the image


class TestWall(unittest.TestCase):
    def test_against_wall_record(self):
        """Bottle 2 cm in front of a wall: recorded, not required (DIAG R2 says v1 cannot)."""
        out = {}
        for x in (0.7, 1.0, 1.4):
            m, t = remembered(bottle(x), pose(), wall=x + 0.035 + 0.02)
            out[x] = words(m, t.id, pose(), wall=x + 0.055)
            # present must never be free
            self.assertNotEqual(words(m, t.id, pose(), boxes=[bottle(x)], wall=x + 0.055)[1], "free")
        sys.stderr.write("\n[geometry v2] wall 2 cm, removed (v1, v2) by range: %r\n" % out)
        self.assertTrue(all(v1 != "free" for v1, _ in out.values()))


class TestSeeThrough(unittest.TestCase):
    def test_transparent_bottle_filter(self):
        T = pose()
        box = bottle(1.0)
        m, t = remembered(box, T)            # the track came from frames where its label/cap gave depth
        filt = SeeThroughFilter(CFG2)
        results = []
        for k in range(6):
            f = frame(100 + k, T, render(T))                 # now the depth sees the floor through it
            geo = core.compute_geometry(f, m, CFG2)
            self.assertEqual(geo[t.id], "free")              # raw v2: fooled
            detected = k < 3                                  # detector sees it the first 3 frames only
            # the detector's observation (its depth would be the floor's; the filter only needs label + position)
            obs = ([core.Observation("bottle", 0.9, t.position.copy(), t.lower, t.upper, "black", 1.0, f.frame_id,
                                     f.timestamp, (0, 0, 1, 1), "detector")] if detected else [])
            out = filt.apply(geo, obs, m)
            results.append(out[t.id])
        self.assertTrue(all(w != "free" for w in results), results)
        self.assertIn(t.id, filt.see_through)
        self.assertEqual(filt.counts["conflicts"], 3)

    def test_filter_passthrough_when_v1(self):
        # 10-27 candidate merge: pass-through needs existence_v3 off too (v1 + existence_v3 filters, see below)
        filt = SeeThroughFilter(dataclasses.replace(CFG1, existence_v3=False))
        geo = {"a": "free"}
        self.assertIs(filt.apply(geo, [], None), geo)
        for cfg in (dataclasses.replace(CFG1, see_through_filter=False),
                    dataclasses.replace(CFG2, see_through_filter=False)):
            self.assertIs(SeeThroughFilter(cfg).apply(geo, [], None), geo)

    def test_filter_runs_for_v1_with_existence(self):
        # 10-27 candidate merge: existence_v3 on -> the filter also judges v1 words
        self.assertTrue(CFG1.existence_v3 and not CFG1.geometry_v2)
        T = pose(); box = bottle(1.0)
        m, t = remembered(box, T, cfg=CFG1)
        filt = SeeThroughFilter(CFG1)
        obs = [core.Observation("bottle", 0.9, t.position.copy(), t.lower, t.upper, "black", 1.0, "x", 0.0,
                                (0, 0, 1, 1), "detector")]
        for k in range(2):
            self.assertEqual(filt.apply({t.id: "free"}, obs, m)[t.id], "surface_or_uncertain")
        self.assertEqual(filt.apply({t.id: "free"}, [], m)[t.id], "surface_or_uncertain")   # 2 conflicts: blocked
        self.assertTrue(filt.blocked(m, t.id))

    def test_one_conflict_then_real_removal_still_free(self):
        T = pose(); box = bottle(1.0)
        m, t = remembered(box, T)
        filt = SeeThroughFilter(CFG2)
        f = frame(200, T, render(T))
        geo = core.compute_geometry(f, m, CFG2)
        self.assertEqual(filt.apply(geo, [], m)[t.id], "free")        # no detection: free passes
        other = core.process_frame(frame(201, T, render(T, [bottle(1.0, y=0.6)])),
                                   [detection(frame(201, T, None), bottle(1.0, y=0.6))], CFG2)
        self.assertEqual(filt.apply(geo, other, m)[t.id], "free")     # a bottle 0.6 m away is another object
        self.assertEqual(filt.counts["conflicts"], 0)


class TestPlanContract(unittest.TestCase):
    def test_near_far_always_kernel_valid(self):
        rng = np.random.default_rng(3)
        for _ in range(500):
            e = float(rng.uniform(0.11, 6.0))
            front, behind = geometry_v2.thresholds(e, CFG2)
            near = max(front + CFG2.depth_margin_m, geometry_v2.NEAR_Z_MIN)
            far = max(behind - CFG2.depth_margin_m, near)
            self.assertTrue(near > 0.1 and far >= near)
            self.assertLessEqual(np.float32(near - CFG2.depth_margin_m), np.float32(far + CFG2.depth_margin_m))
            self.assertAlmostEqual(far + CFG2.depth_margin_m, behind, places=12)

    def test_default_off(self):
        self.assertFalse(core.Config().geometry_v2)

    def test_memory_cfg_drives_default_call(self):
        T = pose(); m, t = remembered(bottle(1.0), T)
        f = frame(300, T, render(T))
        self.assertEqual(core.compute_geometry(f, m)[t.id], "free")              # memory cfg is v2
        m1, t1 = remembered(bottle(1.0), T, cfg=CFG1)
        self.assertEqual(core.compute_geometry(f, m1)[t1.id], "surface_or_uncertain")


@unittest.skipIf(shutil.which("g++") is None, "g++ needed for the region-core C reference")
class TestCpuKvSameWords(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from host.m7_object_native import NativeObjectEvidence, compile_object_native
        cls.tmp = tempfile.mkdtemp()
        cls.nat = NativeObjectEvidence(compile_object_native(Path(cls.tmp) / "object.so"))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def kv_words(self, m, f):
        """The car's KV path: /fpga/memory body -> plan_frame -> region core rows -> evidence_word."""
        from host.geometry_region_glue import evidence_word, plan_frame
        msg = {"revision": 1, "world_frame": "map", "runtime_boot_id": "x",
               "tracks": [dict(t.as_dict()) for t in m.tracks.values()]}
        depth32 = np.ascontiguousarray(np.nan_to_num(f.depth, nan=0.0), dtype=np.float32)
        plan = plan_frame(msg, depth32.shape, f.K, f.T_world_camera, CFG2, ground_z_m=0.0)
        out = dict(plan.host_resolved)
        if plan.batch is not None:
            rows = self.nat.evaluate_boxes(depth32, [(r.row_id, r.xyxy, r.near_z, r.far_z)
                                                     for r in plan.batch.rows], CFG2)
            for r, row in zip(plan.batch.rows, rows):
                out[plan.row_map[r.row_id]] = evidence_word(row, CFG2)
        # check_boxes (the local reference path) must agree too
        entries = [(t.id, np.array([t.lower, t.upper]), t.position) for t in m.tracks.values()]
        for (tid, _, _), (_, word, _) in zip(entries, self.nat.check_boxes(depth32, f.K, f.T_world_camera,
                                                                         entries, CFG2, ground_z_m=0.0)):
            self.assertEqual(word, out[tid])
        return out

    def test_same_words_many_scenes(self):
        cases = 0
        hand = (np.array([0.55, -0.12, 0.0]), np.array([0.62, 0.12, 0.40]))
        for x in (0.7, 1.0, 1.5, 2.0):
            m, t = remembered(bottle(x), pose())
            for scene, wall in (([], None), ([bottle(x)], None), ([hand], None), ([], x + 0.055),
                                ([bottle(x)], 2.6)):
                for T in (pose(), pose(x=0.1), pose(y=0.35), pose(y=0.7), pose(pitch_deg=9)):
                    f = frame(400 + cases, T, render(T, scene, wall))
                    f.depth = np.nan_to_num(f.depth, nan=0.0).astype(np.float32).astype(np.float64)
                    cpu = core.compute_geometry(f, m, CFG2)
                    self.assertEqual(cpu, self.kv_words(m, f), (x, len(scene), wall))
                    cases += 1
        self.assertEqual(cases, 100)


if __name__ == "__main__":
    unittest.main()
