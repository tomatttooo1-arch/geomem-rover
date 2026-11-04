"""10-28 move by far association (Config.move_v2), found/approach at the observed place, tighter creation.

Car run 20261028-164456-live: a banana moved 0.64 m was seen once at the new place through a drift-grown gate,
taken as a confirmation of the old place (fusion dropped the reading), the find drove to the empty old place,
later readings started a second banana, and a box glued to the top image edge made a false third one.
"""
import math
import unittest
from dataclasses import replace

import numpy as np

from host.ros_runtime import approach_goal
from host.task_control import PoseGoal, TaskController
from perception.fpga_replay import memory_gates
from perception.fpga_replay.core import Config, Frame, Memory, Observation

K = np.array([[517., 0, 320], [0, 517., 240], [0, 0, 1]])


class Car:
    """Camera looking along +x (world) from (0, y), 5 Hz; ``drive`` adds odometer path without observations."""

    def __init__(self, cfg=Config()):
        self.memory, self.t, self.i, self.y = Memory("geometry", cfg), 1000.0, 0, 0.0

    def frame(self):
        self.i += 1; self.t += 0.2
        T = np.eye(4)
        T[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        T[1, 3] = self.y
        f = Frame("f-%05d" % self.i, self.t, np.zeros((2, 2, 3), np.uint8), np.full((2, 2), np.nan, np.float32),
                  K, T, True, "test", "test")
        f.ground_z_m = 0.0
        return f

    def obs(self, f, p, label="banana", score=0.8, truncated=False):
        p = np.array(p, dtype=float)
        rng = float(p[0])
        u = 320 - 517 * (p[1] - self.y) / rng
        return Observation(label, score, p, p - .03, p + .03, "yellow", 1.0, f.frame_id, f.timestamp,
                           (u - 10, 200, u + 10, 230), "test", extent_m=(0.15, 0.05), truncated=truncated,
                           range_m=rng)

    def step(self, points=(), **kw):
        f = self.frame()
        out = self.memory.update_memory([self.obs(f, p, **kw) for p in points], {}, f)
        return f, out

    def drive(self, metres):
        """Odometer path (side to side, 0.5 m per frame): the drift gate grows to its 0.8 m cap by ~15 m."""
        for _ in range(int(metres / 0.5)):
            self.y = 0.5 if self.y == 0.0 else 0.0
            self.step()
        self.y = 0.0

    def bananas(self):
        return sorted((t for t in self.memory.tracks.values() if t.label == "banana"), key=lambda t: t.id)


OLD, NEW = (1.5, 0.0, 0.03), (1.5, 0.62, 0.03)


def seeded(cfg=Config()):
    car = Car(cfg)
    for _ in range(15):                     # 3 s in full view: one track, number 1
        car.step([OLD])
    car.drive(16)
    (t,) = car.bananas()
    return car, t


class FarAssociationIsAMove(unittest.TestCase):
    def test_one_far_reading_does_not_confirm_the_old_place(self):
        car, t = seeded()
        odo, existence, last = t.odo, t.existence, t.last_seen
        f, _ = car.step([NEW])
        self.assertLess(np.linalg.norm(t.position - OLD), 0.02)          # not moved by one reading
        self.assertNotEqual(t.state, "current_confirmed")                # and not confirmed there either
        self.assertEqual((t.odo, t.existence, t.last_seen), (odo, existence, last))
        self.assertEqual(t.observed["frame_id"], f.frame_id)
        self.assertTrue(t.observed["far"] and not t.observed["weak"])
        (row,) = car.memory.query_memory("香蕉", after=f.timestamp - 0.1)
        self.assertFalse(row["confirmed_after_query"])
        self.assertTrue(row["observed_after_query"])
        self.assertAlmostEqual(row["observed"]["position_m"][1], 0.62)

    def test_three_far_readings_move_the_track_keeping_id_and_number(self):
        car, t = seeded()
        for _ in range(3):
            _, out = car.step([NEW])
        (moved,) = car.bananas()                                         # no second banana
        self.assertEqual((moved.id, moved.number), (t.id, 1))
        self.assertLess(np.linalg.norm(moved.position - NEW), 0.02)
        self.assertEqual(moved.state, "current_confirmed")
        self.assertTrue(all(abs(s[1][1] - 0.62) < 0.01 for s in moved.samples))   # old place's samples dropped
        self.assertEqual(out["relocated"][0]["via"], "far_association")
        for _ in range(10):                                              # the new place now confirms normally
            car.step([NEW])
        self.assertEqual(len(car.bananas()), 1)

    def test_old_place_still_seen_means_no_move(self):
        car, t = seeded()
        for k in range(12):                                              # see-through style: every 3rd reading far
            car.step([NEW if k % 3 == 0 else OLD])
        (kept,) = car.bananas()
        self.assertLess(np.linalg.norm(kept.position - OLD), 0.02)
        self.assertEqual(car.memory.identity_stats.get("relocated_far", 0), 0)

    def test_one_outlier_then_old_place_again(self):
        car, t = seeded()
        car.step([NEW])
        for _ in range(5):
            car.step([OLD])
        (kept,) = car.bananas()
        self.assertLess(np.linalg.norm(kept.position - OLD), 0.02)
        self.assertEqual(kept.state, "current_confirmed")

    def test_weak_far_readings_move_nothing(self):
        car, t = seeded()
        for _ in range(5):
            car.step([NEW], truncated=True)                              # clipped box: may not start, may not move
        for _ in range(5):
            car.step([NEW], score=0.3)                                   # below the banana creation score
        (kept,) = car.bananas()
        self.assertLess(np.linalg.norm(kept.position - OLD), 0.02)
        self.assertTrue(t.observed["weak"])
        self.assertGreater(car.memory.rejected.get("far_reading_too_weak_to_move", 0), 0)

    def test_near_readings_still_confirm(self):
        car, t = seeded()
        car.step([(1.55, 0.05, 0.03)])                                   # 7 cm off: an ordinary confirmation
        self.assertEqual(t.state, "current_confirmed")
        self.assertFalse(t.observed["far"])

    def test_switch_off_is_the_old_behaviour(self):
        car, t = seeded(replace(Config(), move_v2=False))
        car.step([NEW])
        self.assertEqual(t.state, "current_confirmed")                   # the 10-28 car bug: far read confirmed
        self.assertLess(np.linalg.norm(t.position - OLD), 0.02)


class TighterCreation(unittest.TestCase):
    def test_clipped_box_of_an_unlisted_class_cannot_create(self):
        o = Observation("banana", 0.8, np.zeros(3), np.zeros(3), np.zeros(3), "y", 1.0, "f", 0.0, (10, 0, 90, 120),
                        "t", truncated=memory_gates.is_truncated((10, 0, 90, 120), (480, 640), 2.0))
        self.assertTrue(o.truncated)                                     # the top edge counts
        self.assertFalse(memory_gates.may_create(o))
        self.assertFalse(memory_gates.may_create(replace(o, label="bottle")))
        self.assertTrue(memory_gates.may_create(replace(o, label="chair")))   # large classes still may
        self.assertTrue(memory_gates.may_create(replace(o, truncated=False)))

    def test_top_edge_false_banana_never_starts(self):
        car = Car()
        for _ in range(30):
            car.step([(1.8, -0.3, 0.46)], truncated=True, score=0.3)
        self.assertEqual(car.bananas(), [])

    def test_banana_needs_04_to_start_but_not_to_update(self):
        car = Car()
        for _ in range(20):
            car.step([OLD], score=0.35)
        self.assertEqual(car.bananas(), [])
        self.assertGreater(car.memory.rejected.get("score_below_create", 0), 0)
        for _ in range(15):
            car.step([OLD], score=0.6)
        (t,) = car.bananas()
        f, _ = car.step([OLD], score=0.3)                                # a low-score update still confirms
        self.assertEqual((t.state, t.last_seen_frame), ("current_confirmed", f.frame_id))
        bottle = Car()
        for _ in range(15):
            bottle.step([OLD], label="bottle", score=0.35)               # other classes: unchanged
        self.assertEqual(len(bottle.memory.tracks), 1)


class FoundDrivesToTheObservation(unittest.TestCase):
    def test_seen_en_route_far_from_memory_approaches_the_reading(self):
        car, t = seeded()
        task = TaskController(car.memory, lambda row: [PoseGoal(0.5, 0.0, 0.0)], ())
        f0 = car.frame()
        task.start("香蕉", f0.timestamp, f0)
        self.assertEqual(task.state, "navigating")
        f, _ = car.step([NEW])
        task.observe(f, f.timestamp, robot_xy=(0.8, 0.4))
        self.assertEqual((task.state, task.reason, task.target_id), ("found", "seen_en_route", t.id))
        self.assertEqual(task.found_row["approach_source"], "observation")
        self.assertAlmostEqual(task.found_row["approach_position_m"][1], 0.62)
        self.assertEqual(task.events[-1]["approach_source"], "observation")
        self.assertAlmostEqual(task.events[-1]["memory_position_m"][1], 0.0, places=2)
        wb = np.array([[1, 0, 0, 0.4], [0, 1, 0, 0.62], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        bc = np.eye(4); bc[0, 3] = 0.1
        pose, info = approach_goal(wb, task.found_row, bc)
        self.assertEqual(info["goal_source"], "observation")
        self.assertAlmostEqual(info["target_m"][1], 0.62)
        self.assertAlmostEqual(pose.y, 0.62, places=3)                   # drives at the banana, not at y = 0

    def test_weak_far_reading_is_not_a_find(self):
        car, t = seeded()
        task = TaskController(car.memory, lambda row: [PoseGoal(0.5, 0.0, 0.0)], ())
        f0 = car.frame()
        task.start("香蕉", f0.timestamp, f0)
        f, _ = car.step([NEW], truncated=True)
        task.observe(f, f.timestamp, robot_xy=(0.8, 0.4))
        self.assertEqual(task.state, "navigating")

    def test_near_find_keeps_the_memory_place(self):
        car, t = seeded()
        task = TaskController(car.memory, lambda row: [PoseGoal(0.5, 0.0, 0.0)], ())
        f0 = car.frame()
        task.start("香蕉", f0.timestamp, f0)
        f, _ = car.step([OLD])
        task.observe(f, f.timestamp, robot_xy=(0.8, 0.0))
        self.assertEqual((task.state, task.found_row["approach_source"]), ("found", "memory"))
        _, info = approach_goal(np.array([[1, 0, 0, 0.4], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float),
                                task.found_row, np.eye(4))
        self.assertEqual(info["goal_source"], "memory")


if __name__ == "__main__":
    unittest.main()
