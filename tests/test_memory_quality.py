"""Memory quality (10-16): admission gates, same-frame dedupe, drift-scaled radius.

docs/design/MEMORY_QUALITY_20261016.md. Two parts:

* unit tests on hand-made observations (ground gate, size gate, clipped boxes,
  same-frame dedupe, radius growth/cap, pose-sigma branch, memory_quality=False);
* the compact real-car stream (tests/data/memq_realcar_20261016_compact.json,
  10-16 first real run, frames without observations and <2 cm/<1 deg steps dropped)
  replayed through Config(memory_quality=False) (positive control: must reproduce
  the run's failures) and the default Config (criteria C1-C4 of the design doc).

The fixture rounds positions to 4 decimals, so the positive control is checked
against the logged final tracks at 1e-3 m; the bit-exact legacy == car-code check
is the full-stream harness of the design doc (section 6), not this test.
Compaction drops ~25 % of the accumulated turn (oscillation below 1 deg), so the
new arm's radius on the fixture is a lower bound of the full stream's; assertions
here are about bottles, bins and z, never the total track count.
"""
import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np

from perception.fpga_replay import memory_gates
from perception.fpga_replay.core import Config as _Config, Frame, Memory, Observation, UNGATED_CREATION, process_frame
from perception.fpga_replay.synthetic import scene


def Config(**kw):
    """core.Config with the D-044 creation gate off (core.UNGATED_CREATION). The expectations in this file
    were written for single-sighting creation; the gate has its own tests (test_memory_creation_gate.py)."""
    return _Config(**dict(UNGATED_CREATION, **kw))


ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests/data/memq_realcar_20261016_compact.json"
K = np.array([[500., 0, 320], [0, 500, 240], [0, 0, 1]])
W, H = 640, 480


def load_replay_stream():
    spec = importlib.util.spec_from_file_location(
        "memq_replay_stream", ROOT / "tools/memory_quality/replay_stream.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["memq_replay_stream"] = module
    spec.loader.exec_module(module)
    return module


# Radius tests start a second track just outside the association radius; with identity_v2 (10-27) a
# never-co-visible second track inside the exclusion zone is the same object, so they use the legacy
# association. identity_v2 itself: tests/test_object_identity_20261027.py.
LEGACY_IDENTITY = Config(identity_v2=False)


class Driver:
    """Feeds hand-made frames/observations to one Memory with increasing timestamps."""

    def __init__(self, cfg=Config()):
        self.memory, self.i = Memory("geometry", cfg), 0

    def frame(self, x=0.0, yaw=0.0, ground=None, sigma=None, pose_valid=True):
        self.i += 1
        T = np.eye(4)
        c, s = np.cos(yaw), np.sin(yaw)
        T[:2, :2] = [[c, -s], [s, c]]
        T[0, 3] = x
        f = Frame("q-%04d" % self.i, float(self.i), np.zeros((2, 2, 3), np.uint8),
                  np.full((2, 2), np.nan, np.float32), K, T, pose_valid, "test", "test")
        if ground is not None:
            f.ground_z_m = ground
        if sigma is not None:
            f.pose_sigma_m = sigma
        return f

    def obs(self, f, label, p, score=0.9, extent=(0.06, 0.2), truncated=False, bbox=None):
        p = np.array(p, dtype=float)
        if bbox is None:
            # 10-27: distinct world positions give distinct image boxes (identity_v2 drops a same-class box
            # overlapping another by IoU > 0.3 as the same object); 3 cm apart still overlap.
            u = 100 + 100 * (p[0] - 1) + 200 * p[1]
            bbox = (u, 100, u + 30, 200)
        return Observation(label, score, p, p - .03, p + .03, "red", 1.0, f.frame_id, f.timestamp,
                           tuple(bbox), "test", extent_m=extent, truncated=truncated)

    def step(self, specs=(), **frame_kw):
        """specs: list of (label, position, kwargs); returns update_memory's result."""
        f = self.frame(**frame_kw)
        return self.memory.update_memory([self.obs(f, lab, p, **kw) for lab, p, kw in specs], {}, f)

    def travel(self, metres, steps=20, x0=0.0):
        """Move the camera along x in ``steps`` frames without observations; returns the end x."""
        for k in range(1, steps + 1):
            self.step(x=x0 + metres * k / steps)
        return x0 + metres

    def bottles(self):
        return [t for t in self.memory.tracks.values() if t.label == "bottle"]


class GateTests(unittest.TestCase):
    def test_below_ground_dropped_with_tolerance(self):
        d = Driver()
        r = d.step([("bottle", (1, 0, -0.02), {}), ("cup", (1, 1, -0.04), {})])
        self.assertEqual([t.label for t in d.memory.tracks.values()], ["bottle"])
        self.assertEqual([x["reason"] for x in r["rejected"]], ["below_ground"])
        self.assertEqual(d.memory.rejected, {"below_ground": 1})

    def test_frame_ground_overrides_config(self):
        d = Driver(Config(ground_z_m=0.0))
        d.step([("bottle", (1, 0, -0.05), {})], ground=-0.1)       # 0.05 above the frame's ground
        self.assertEqual(len(d.bottles()), 1)
        d.step([("cup", (2, 0, 0.05), {})], ground=0.2)           # 0.15 below the frame's ground
        self.assertEqual(d.memory.rejected, {"below_ground": 1})
        d = Driver(Config(ground_z_m=0.2))
        d.step([("cup", (2, 0, 0.05), {})])                       # no frame value: Config ground
        self.assertEqual(d.memory.rejected, {"below_ground": 1})

    def test_size_gate_class_table(self):
        d = Driver()
        limit_w = memory_gates.CLASS_MAX_APPARENT_M["bottle"][0][0] * Config().size_margin
        d.step([("bottle", (1, 0, .1), {"extent": (limit_w + .01, .2)}),
                ("bottle", (2, 0, .1), {"extent": (limit_w - .01, .2)}),
                ("couch", (3, 0, .1), {"extent": (2.0, 2.0)})])      # unlisted class: no size gate
        self.assertNotIn("couch", memory_gates.CLASS_MAX_APPARENT_M)
        self.assertEqual(sorted(t.label for t in d.memory.tracks.values()), ["bottle", "couch"])
        self.assertEqual(d.memory.rejected, {"size_exceeds_class": 1})

    def test_clipped_box_size_is_lower_bound_still_gated(self):
        d = Driver()
        d.step([("bottle", (1, 0, .1), {"extent": (.30, .30), "truncated": True})])
        self.assertEqual(d.memory.rejected, {"size_exceeds_class": 1})

    def test_clipped_small_class_updates_but_never_creates(self):
        d = Driver()
        d.step([("bottle", (1, 0, .1), {"truncated": True})])
        self.assertEqual(d.bottles(), [])
        self.assertEqual(d.memory.rejected, {"clipped_box_cannot_create": 1})
        d.step([("bottle", (1, 0, .1), {})])
        d.step([("bottle", (1.05, 0, .1), {"truncated": True})])
        (t,) = d.bottles()
        self.assertEqual(t.state, "current_confirmed")
        # 10-27: the clipped box updated the track (it is its newest observation); the position is fused
        self.assertTrue(np.allclose(t.history[-1]["position_m"], (1.05, 0, .1)))
        self.assertTrue(1.0 < t.position[0] < 1.05)

    def test_clipped_large_class_may_create(self):
        d = Driver()
        d.step([("backpack", (1, 0, .2), {"truncated": True, "extent": (.4, .5)})])
        self.assertEqual([t.label for t in d.memory.tracks.values()], ["backpack"])

    def test_same_frame_duplicates_keep_complete_box(self):
        d = Driver()
        r = d.step([("bottle", (1, 0, .1), {"truncated": True, "score": .95}),
                    ("bottle", (1.03, 0, .1), {"score": .5}),
                    ("bottle", (1.5, 0, .1), {})])               # another bottle, kept
        self.assertEqual(len(d.bottles()), 2)
        self.assertEqual(r["ambiguous"], [])
        self.assertEqual(d.memory.rejected, {"same_frame_duplicate": 1})
        self.assertTrue(any(np.allclose(t.position, (1.03, 0, .1)) for t in d.bottles()))

    def test_is_truncated_and_apparent_extent(self):
        self.assertTrue(memory_gates.is_truncated((0, 100, 50, 200), (H, W), 2))
        self.assertTrue(memory_gates.is_truncated((100, 100, 638, 200), (H, W), 2))
        self.assertFalse(memory_gates.is_truncated((3, 3, 636, 476), (H, W), 2))
        w, h = memory_gates.apparent_extent((100, 100, 150, 300), 2.0, K)
        self.assertAlmostEqual(w, 0.2); self.assertAlmostEqual(h, 0.8)

    def test_process_frame_fills_extent_and_truncation(self):
        f, dets = scene(0)
        (o,) = process_frame(f, dets)
        self.assertFalse(o.truncated)
        x1, y1, x2, y2 = dets[0].bbox
        self.assertAlmostEqual(o.extent_m[0], (x2 - x1) * 2.0 / f.K[0, 0], places=6)
        self.assertLess(o.extent_m[0], memory_gates.CLASS_MAX_APPARENT_M["cup"][0][0] * Config().size_margin)


class RadiusTests(unittest.TestCase):
    def test_stationary_radius_is_fixed(self):
        d = Driver(LEGACY_IDENTITY)
        d.step([("bottle", (1, 0, .1), {})])
        d.step([("bottle", (1.26, 0, .1), {})])
        self.assertEqual(len(d.bottles()), 2)

    def test_radius_grows_with_path(self):
        cfg = LEGACY_IDENTITY
        d = Driver(cfg)
        d.step([("bottle", (1, 0, .1), {})])
        x = d.travel(5.0)                                          # r = 0.25 + 0.03 * 5 = 0.40
        self.assertAlmostEqual(d.memory.odometer[0], 5.0)
        d.step([("bottle", (1.38, 0, .1), {})], x=x)
        (t,) = d.bottles()
        self.assertTrue(np.allclose(t.position, (1.38, 0, .1)))
        # after the update the radius is back at 0.25 for this track
        d.step([("bottle", (1.38 + .30, 0, .1), {})], x=x)
        self.assertEqual(len(d.bottles()), 2)

    def test_radius_grows_with_turn(self):
        d = Driver()
        d.step([("bottle", (1, 0, .1), {})])
        for k in range(1, 41):                                     # 4 rad back and forth: + 0.20
            d.step(yaw=0.1 if k % 2 else 0.0)
        self.assertAlmostEqual(d.memory.odometer[1], 4.0, places=6)
        d.step([("bottle", (1.43, 0, .1), {})])
        self.assertEqual(len(d.bottles()), 1)

    def test_radius_capped(self):
        cfg = Config()
        d = Driver(cfg)
        d.step([("bottle", (1, 0, .1), {})])
        x = d.travel(100.0, steps=50)                              # uncapped would be 3.25 m
        d.step([("bottle", (1 + cfg.association_max_m + .05, 0, .1), {})], x=x)
        self.assertEqual(len(d.bottles()), 2)
        d = Driver(cfg)
        d.step([("bottle", (1, 0, .1), {})])
        x = d.travel(100.0, steps=50)
        d.step([("bottle", (1 + cfg.association_max_m - .05, 0, .1), {})], x=x)
        self.assertEqual(len(d.bottles()), 1)
        self.assertEqual(memory_gates.association_radius((0, 0), None, (1e3, 1e3), None, cfg),
                         cfg.association_max_m)

    def test_pose_sigma_branch_replaces_odometry_terms(self):
        # no motion: odometry branch gives 0.25; sigma 0.1 on both sides gives 0.25 + 2 * 0.1414 = 0.533
        d = Driver(LEGACY_IDENTITY)
        d.step([("bottle", (1, 0, .1), {})], sigma=0.1)
        d.step([("bottle", (1.5, 0, .1), {})], sigma=0.1)
        self.assertEqual(len(d.bottles()), 1)
        d = Driver(LEGACY_IDENTITY)
        d.step([("bottle", (1, 0, .1), {})], sigma=0.1)
        d.step([("bottle", (1.6, 0, .1), {})], sigma=0.1)
        self.assertEqual(len(d.bottles()), 2)
        d = Driver(LEGACY_IDENTITY)                                               # sigma only on one side: odometry branch
        d.step([("bottle", (1, 0, .1), {})])
        d.step([("bottle", (1.5, 0, .1), {})], sigma=0.1)
        self.assertEqual(len(d.bottles()), 2)

    def test_invalid_pose_frames_do_not_advance_odometer(self):
        d = Driver()
        d.step(x=0.0)
        d.step(x=5.0, pose_valid=False)
        self.assertEqual(d.memory.odometer, (0.0, 0.0))

    def test_mutually_exclusive_rule_kept(self):
        d = Driver(LEGACY_IDENTITY)
        d.step([("bottle", (1, 0, .1), {}), ("bottle", (1.4, 0, .1), {})])
        x = d.travel(5.0)                                          # r = 0.40 for both tracks
        r = d.step([("bottle", (1.2, 0, .1), {})], x=x)            # inside both radii
        self.assertEqual(len(d.bottles()), 2)
        self.assertEqual(len(r["ambiguous"]), 1)


class LegacyBehaviourTests(unittest.TestCase):
    def test_memory_quality_false_disables_gates_and_growth(self):
        d = Driver(Config(memory_quality=False))
        d.step([("bottle", (1, 0, -0.5), {}),                       # below ground
                ("bottle", (2, 0, .1), {"extent": (1.0, 1.0)}),       # oversize
                ("bottle", (3, 0, .1), {"truncated": True})])         # clipped
        self.assertEqual(len(d.bottles()), 3)
        x = d.travel(10.0)
        d.step([("bottle", (3.26, 0, .1), {})], x=x)                 # beyond fixed 0.25 despite travel
        self.assertEqual(len(d.bottles()), 4)
        self.assertEqual(d.memory.rejected, {})
        self.assertEqual(d.memory.odometer, (0.0, 0.0))
        r = d.step([("bottle", (5, 0, .1), {}), ("bottle", (5.03, 0, .1), {})])   # no dedupe: twins
        self.assertEqual(len(d.bottles()), 6)
        self.assertEqual(r["rejected"], [])

    def test_default_and_legacy_agree_on_synthetic_scene(self):
        from perception.fpga_replay.core import compute_geometry
        snaps = []
        for cfg in (Config(), Config(memory_quality=False)):
            m = Memory("geometry", cfg)
            for i in range(4):
                f, dets = scene(i)
                m.update_memory(process_frame(f, dets, cfg), compute_geometry(f, m), f)
            snaps.append(m.snapshot())
        self.assertEqual(snaps[0], snaps[1])


class RealCarStreamTests(unittest.TestCase):
    """C1-C4 (design doc section 5) on the compact fixture, with the legacy arm as positive control."""

    @classmethod
    def setUpClass(cls):
        from perception.fpga_replay import core
        rs = cls.rs = load_replay_stream()
        K_, T_bc, image, frames, cls.doc = rs.load_stream(FIXTURE)
        points = {"find1": 1792092486.9923933, "find2": 1792092813.9154556, "kidnap": 1792092965.0}
        cls.cfg = Config()
        cls.arms = {
            "legacy": rs.run_arm(core, Config(memory_quality=False), frames, K_, T_bc, image, points, True),
            "new": rs.run_arm(core, cls.cfg, frames, K_, T_bc, image, points, True),
        }
        cls.ev = rs.evaluate(cls.arms, points["kidnap"], points["find1"], cls.cfg.below_ground_tol_m)

    def test_fixture_is_compact_schema(self):
        self.assertEqual(self.doc["schema"], "fpga.memq.realcar_compact.v1")
        self.assertLess(FIXTURE.stat().st_size, 1 << 20)

    def test_positive_control_reproduces_logged_run(self):
        end = {(t["id"], t["label"]): t["position_m"] for t in self.arms["legacy"][1]["end"]}
        logged = {(t["id"], t["label"]): t["position_m"] for t in self.doc["logged_final_tracks"]}
        self.assertEqual(end.keys(), logged.keys())
        self.assertEqual(len(logged), 30)
        for k in logged:
            self.assertTrue(np.allclose(end[k], logged[k], atol=1e-3, rtol=0), k)
        old = self.ev["legacy"]
        self.assertEqual(len(old["bottle_tracks_find1"]), 9)
        self.assertEqual(len(self.doc["ambiguous_find_events"][0]["candidates"]), 9)
        self.assertEqual(old["identity_full_run"]["bins"],
                         ["object-0012", "object-0013", "object-0014", "object-0015"])
        self.assertEqual(old["applied_below_zero"], 26)
        self.assertGreater(old["applied_below_tol"], 0)
        for phys in ("milk", "nongfu", "redcap"):
            self.assertGreater(len(old["identity_before_kidnap"][phys]), 1, phys)

    def test_c1_first_find_candidates_and_position(self):
        c = self.rs.criteria(dict(self.ev, main=self.ev["legacy"]), self.cfg.below_ground_tol_m)
        self.assertEqual(c["C1_find1_bottle_candidates_le_3"]["value"], 3)
        self.assertTrue(c["C1_find1_bottle_candidates_le_3"]["pass"])
        self.assertTrue(c["C1b_both_found_bottles_among_candidates"]["pass"])
        self.assertTrue(c["C1c_position_not_worse"]["pass"], c["C1c_position_not_worse"])

    def test_c2_bins_never_stored(self):
        self.assertEqual(self.ev["new"]["identity_full_run"]["bins"], [])

    def test_c3_one_track_per_bottle_before_kidnap(self):
        new = self.ev["new"]
        for phys in ("milk", "nongfu", "redcap"):
            self.assertEqual(len(new["identity_before_kidnap"][phys]), 1, phys)
        self.assertEqual(new["mixed_tracks_before_kidnap"], {})

    def test_c4_nothing_below_ground_stored(self):
        self.assertEqual(self.ev["new"]["applied_below_tol"], 0)
        self.assertEqual(self.ev["new"]["applied_below_zero"], 0)

    def test_rejection_counts(self):
        # 10-27 identity_v2: observations farther than 2 m update but do not start tracks, beyond 3 m dropped;
        # clipped boxes that no longer match a (fused) track are counted. The 10-16 counts stay for the legacy
        # association arm. 10-28 move_v2: a far reading no longer confirms (no drift-gate reset), so more of the
        # clipped / > 2 m readings land in a track's gate as far readings, counted as too weak to move it
        # (clipped 35 -> 17, too far 509 -> 27, + 485); end tracks unchanged (4; the 3 "near" relocations are
        # now far-association moves).
        self.assertEqual(self.ev["new"]["rejected"], {"size_exceeds_class": 29, "clipped_box_cannot_create": 17,
                                                      "beyond_range": 1, "too_far_to_create": 27,
                                                      "far_reading_too_weak_to_move": 485,
                                                      "same_frame_duplicate": 1, "below_ground": 25})
        rs, frames = self.rs, self.rs.load_stream(FIXTURE)[3]
        K_, T_bc, image = self.rs.load_stream(FIXTURE)[:3]
        q = rs.run_arm(__import__("perception.fpga_replay.core", fromlist=["core"]), Config(identity_v2=False),
                       frames, K_, T_bc, image, {}, True)[0]
        self.assertEqual(q.rejected, {"size_exceeds_class": 29, "clipped_box_cannot_create": 3,
                                      "same_frame_duplicate": 1, "below_ground": 25})


if __name__ == "__main__":
    unittest.main()
