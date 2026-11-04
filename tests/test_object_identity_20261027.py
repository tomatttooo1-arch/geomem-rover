"""Object identity (Config.identity_v2, 2026-10-27): docs/design/OBJECT_IDENTITY_20261027.md.

Scenarios are the failure modes seen in the 10-26 real-car logs, fed through the car's default Config
(creation gate on: 3 hits over 2 s), at 5 Hz like the board:
  * one transparent bottle at 2.07 m, every 4th depth reading 0.24 m too far (run 150138: bottle 15 -> 16)
  * one bottle at 3.3-3.6 m with 0.2-0.3 m along-ray jitter (run 152301: bottle 4 -> 11)
and the cases the rules must not break: two real bottles side by side, a bottle moved while unseen,
old duplicates loaded from a store, the task's target, numbered queries.
"""
import math
import tempfile
import unittest
from dataclasses import replace

import numpy as np

from host import memory_store as ms
from perception.fpga_replay.core import Config, Frame, Memory, Observation, process_frame, requested_number

K = np.array([[517., 0, 320], [0, 517., 240], [0, 0, 1]])


class Car:
    """Camera at the origin looking along +x (world), 5 Hz frames, observations with range_m."""

    def __init__(self, cfg=Config()):
        self.memory, self.t, self.i = Memory("geometry", cfg), 1000.0, 0

    def frame(self, x=0.0):
        self.i += 1; self.t += 0.2
        T = np.eye(4)
        T[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]       # optical z = world x
        T[0, 3] = x
        f = Frame("f-%05d" % self.i, self.t, np.zeros((2, 2, 3), np.uint8), np.full((2, 2), np.nan, np.float32),
                  K, T, True, "test", "test")
        f.ground_z_m = 0.0
        return f

    def obs(self, f, p, x=0.0, label="bottle", score=0.8):
        p = np.array(p, dtype=float)
        rng = float(p[0] - x)
        u = 320 - 517 * p[1] / rng
        return Observation(label, score, p, p - .03, p + .03, "red", 1.0, f.frame_id, f.timestamp,
                           (u - 10, 200, u + 10, 260), "test", extent_m=(0.06, 0.2), truncated=False, range_m=rng)

    def step(self, points, x=0.0):
        f = self.frame(x)
        return self.memory.update_memory([self.obs(f, p, x) for p in points], {}, f)

    def bottles(self):
        return sorted((t for t in self.memory.tracks.values() if t.label == "bottle"), key=lambda t: t.id)


class LogFailureModes(unittest.TestCase):
    def test_see_through_depth_readings_do_not_split_one_bottle(self):
        car = Car()
        for k in range(60):                     # 12 s at 1.9 m (tracks start within 2 m), every 4th reading +0.30 m
            far = k % 4 == 3
            car.step([(1.9 + (0.30 if far else 0.0) + 0.01 * math.sin(k), 0.0, 0.11)])
        (t,) = car.bottles()
        self.assertLess(abs(t.position[0] - 1.9), 0.03)
        self.assertEqual(t.number, 1)

    def test_legacy_association_splits_the_same_stream(self):
        # positive control: the 10-26 behaviour (identity_v2=False) makes a second track from the far readings
        car = Car(Config(identity_v2=False))
        for k in range(60):
            car.step([(1.9 + (0.30 if k % 4 == 3 else 0.0), 0.0, 0.11)])
        self.assertGreater(len(car.bottles()), 1)

    def test_far_jitter_creates_nothing_then_one_track_close_up(self):
        car = Car()
        rng = np.random.default_rng(0)
        for k in range(40):                                       # 2.4-2.9 m away: too far to start a track
            car.step([(2.65 + rng.uniform(-0.25, 0.25), 0.6, 0.11)])
        self.assertEqual(car.bottles(), [])
        self.assertGreater(car.memory.rejected.get("too_far_to_create", 0), 0)
        for k in range(40):                                       # drive to 1.6 m: one track
            car.step([(2.65 + rng.uniform(-0.03, 0.03), 0.6, 0.11)], x=1.05)
        self.assertEqual(len(car.bottles()), 1)
        for k in range(20):                                       # back out: far readings update, never split
            car.step([(2.65 + rng.uniform(-0.25, 0.25), 0.6, 0.11)], x=0.0)
        self.assertEqual(len(car.bottles()), 1)


class RealInstances(unittest.TestCase):
    def test_two_bottles_side_by_side_stay_two_with_fixed_numbers(self):
        car = Car(Config(one_per_class=False))   # 10-31: two real bottles (one_per_class would not make a second)
        for k in range(30):
            car.step([(1.5, 0.0, 0.11), (1.5, 0.3, 0.11)])
        a, b = car.bottles()
        self.assertEqual((a.number, b.number), (1, 2))
        self.assertIn(b.id, a.covis)
        for k in range(30):                                       # only one in view for a while: no merge
            car.step([(1.5, 0.0, 0.11)])
        self.assertEqual([t.id for t in car.bottles()], [a.id, b.id])
        names = {t.id: t.number for t in car.bottles()}
        self.assertEqual(names, {a.id: 1, b.id: 2})

    def test_second_bottle_appearing_next_to_a_visible_one(self):
        car = Car(Config(one_per_class=False))   # 10-31: two real bottles (one_per_class would not make a second)
        for k in range(20):
            car.step([(1.5, 0.0, 0.11)])
        for k in range(20):
            car.step([(1.5, 0.0, 0.11), (1.6, 0.35, 0.11)])
        self.assertEqual(len(car.bottles()), 2)

    def test_bottle_moved_while_unseen_keeps_its_identity(self):
        car = Car()
        for k in range(20):
            car.step([(1.5, 0.0, 0.11)])
        (t,) = car.bottles()
        for k in range(5):
            car.step([])
        for k in range(20):                                       # moved 0.45 m: same track follows
            car.step([(1.5, 0.45, 0.11)])
        (u,) = car.bottles()
        self.assertEqual((u.id, u.number), (t.id, 1))
        self.assertLess(abs(u.position[1] - 0.45), 0.03)
        self.assertEqual(car.memory.identity_stats["relocated"], 1)

    def test_outlier_candidate_beside_a_seen_track_is_dropped(self):
        car = Car()
        for k in range(20):
            car.step([(1.5, 0.0, 0.11)])
        for k in range(30):                                       # alternate true / 0.4 m off readings
            car.step([(1.5, 0.0, 0.11) if k % 2 else (1.9, 0.0, 0.11)])
        self.assertEqual(len(car.bottles()), 1)
        self.assertGreater(car.memory.identity_stats["suppressed_near"], 0)


class MergeAndNames(unittest.TestCase):
    def twins(self):
        car = Car(replace(Config(), identity_v2=False))           # make old-style twins, then switch on v2
        for k in range(20):
            car.step([(1.5, 0.0, 0.11)])
        for k in range(20):
            car.step([(1.8, 0.0, 0.11)])
        self.assertEqual(len(car.bottles()), 2)
        car.memory.cfg = Config()
        return car

    def test_old_duplicates_merge_into_the_smaller_number(self):
        car = self.twins()
        a, b = car.bottles()
        r = car.step([(1.8, 0.0, 0.11)])
        self.assertEqual([t.id for t in car.bottles()], [a.id])
        self.assertEqual(r["merged"][0], {"kept": a.id, "merged": b.id, "distance_m": r["merged"][0]["distance_m"]})
        self.assertEqual(car.memory.resolve(b.id), a.id)
        self.assertIn(b.id, car.bottles()[0].aliases)

    def test_task_target_survives_a_merge(self):
        car = self.twins()
        a, b = car.bottles()
        car.memory.keep_ids = {b.id}
        car.step([(1.8, 0.0, 0.11)])
        self.assertEqual([t.id for t in car.bottles()], [b.id])

    def test_numbered_query(self):
        car = Car(Config(one_per_class=False))   # 10-31: two real bottles (one_per_class would not make a second)
        for k in range(30):
            car.step([(1.5, 0.0, 0.11), (1.5, 0.3, 0.11)])
        self.assertEqual([r["number"] for r in car.memory.query_memory("找瓶子2")], [2])
        self.assertEqual([r["number"] for r in car.memory.query_memory("二号瓶子")], [2])
        self.assertEqual(sorted(r["number"] for r in car.memory.query_memory("帮我找一个瓶子")), [1, 2])
        self.assertEqual(car.memory.query_memory("瓶子7"), [])
        self.assertEqual((requested_number("瓶子 #12"), requested_number("找瓶子")), (12, None))

    def test_store_round_trip_keeps_numbers_and_covisibility(self):
        car = Car(Config(one_per_class=False))   # 10-31: two real bottles (one_per_class would not make a second)
        for k in range(30):
            car.step([(1.5, 0.0, 0.11), (1.5, 0.3, 0.11)])
        doc = ms.dump(car.memory, complete=True, runtime_boot_id="b", world_frame="map", pose_source="estimated",
                      sensor_source="test", map_identity_doc=None)
        m = Memory("geometry", Config())
        ms.restore(doc, m)
        self.assertEqual({t.id: (t.number, sorted(t.covis)) for t in m.tracks.values()},
                         {t.id: (t.number, sorted(t.covis)) for t in car.memory.tracks.values()})
        self.assertEqual(m._new_id(), "object-0003")


class NearSurfaceDepth(unittest.TestCase):
    def frame(self):
        depth = np.full((480, 640), 3.0, np.float32)
        depth[150:330, 300:340] = 2.30                            # wall seen through the bottle
        depth[200:250, 300:340] = 2.05                            # label band: the bottle's real surface
        T = np.eye(4)
        f = Frame("d-1", 1.0, np.zeros((480, 640, 3), np.uint8), depth, K, T, True, "test", "test")
        from perception.fpga_replay.core import Detection
        return f, [Detection("bottle", 0.8, (300., 150., 340., 330.), frame_id="d-1", timestamp=1.0)]

    def test_nearest_significant_surface_not_the_wall_behind(self):
        f, d = self.frame()
        (o,) = process_frame(f, d)
        self.assertAlmostEqual(float(o.position[2]), 2.05, delta=0.03)
        self.assertAlmostEqual(o.range_m, float(o.position[2]), places=6)
        (legacy,) = process_frame(f, d, Config(identity_v2=False))
        self.assertAlmostEqual(float(legacy.position[2]), 2.30, delta=0.03)


class RevisitDriftAfterAlignment(unittest.TestCase):
    """10-26 run 150138, bottle 13 / 26: the same bottle seen again minutes later through 0.9 m of odometry drift
    becomes a second track. At the mapping stop host/map_align moves each sample by its own node's correction; the
    two land together and the next frame merges them into the smaller number."""

    def test_loop_closure_correction_then_merge(self):
        from host import map_align
        car = Car(Config(one_per_class=False))   # 10-31: the drifted twin is what this tests (one_per_class would not make a second)
        for k in range(15):
            car.step([(1.5, 0.0, 0.11)])
        car.t += 300.0                                             # minutes later, 0.9 m of drift
        for k in range(15):
            car.step([(1.5, 0.9, 0.11)])
        a, b = car.bottles()
        self.assertEqual((a.number, b.number), (1, 2))
        eye = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        fix = [0.0, -0.9, 0.0, 0.0, 0.0, 0.0, 1.0]
        align = {"map_odom": fix, "nodes": [[1000.0] + eye, [1004.0] + eye, [car.t - 3.0] + fix, [car.t] + fix]}
        map_align.apply_memory(car.memory, align)
        r = car.step([])
        (t,) = car.bottles()
        self.assertEqual((t.id, t.number), (a.id, 1))
        self.assertEqual(r["merged"][0]["merged"], b.id)
        self.assertLess(abs(t.position[1]), 0.03)


if __name__ == "__main__":
    unittest.main()
