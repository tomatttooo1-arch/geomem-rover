"""10-31 one object per class (Config.one_per_class, owner: at the defence every class is on the field once).

Car 10-31: a banana moved beyond the exclusion zone before its old place was judged empty became "banana 2".
With one_per_class on, a promoted same-class candidate takes the existing track over (id and number kept, place
= the new one); a candidate co-visible with the track, or one while the track is still seen at its old place,
is dropped. Off = the rules before (a second banana).
"""
import unittest
from dataclasses import replace

import numpy as np

from perception.fpga_replay.core import Config, Frame, Memory, Observation

K = np.array([[517., 0, 320], [0, 517., 240], [0, 0, 1]])
OLD = (1.5, 0.0, 0.03)
FAR = (1.5, 2.0, 0.03)          # 2 m from OLD: seen with the car shifted to y = 2 (OLD then out of view)
SIDE = (1.9, 0.9, 0.03)         # ~0.98 m from OLD, beyond the 0.6 m exclusion zone, seen from y = 0


class Car:
    """Camera looking along +x (world) from (0, y), 5 Hz (as tests/test_memory_move_20261028.py)."""

    def __init__(self, cfg):
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

    def obs(self, f, p, label):
        p = np.array(p, dtype=float)
        rng = float(p[0])
        u = 320 - 517 * (p[1] - self.y) / rng
        return Observation(label, 0.8, p, p - .03, p + .03, "yellow", 1.0, f.frame_id, f.timestamp,
                           (u - 10, 200, u + 10, 230), "test", extent_m=(0.15, 0.05), range_m=rng)

    def step(self, points=(), label="banana"):
        f = self.frame()
        pts = [(p, label) if not isinstance(p[0], tuple) else p for p in points]
        out = self.memory.update_memory([self.obs(f, p, lab) for p, lab in pts], {}, f)
        return f, out

    def of(self, label):
        return sorted((t for t in self.memory.tracks.values() if t.label == label), key=lambda t: t.id)


def seeded(**kw):
    car = Car(replace(Config(), **kw))
    for _ in range(15):                     # 3 s in full view: one banana, number 1
        car.step([OLD])
    (t,) = car.of("banana")
    return car, t


def moved_far(car, frames=15):
    car.y = 2.0                             # drive beside the new place; the old one leaves the view
    outs = [car.step([FAR])[1] for _ in range(frames)]
    return outs


class CarRun20261031(unittest.TestCase):
    """The car case itself (run 20261031-163027-live): banana 1 last seen ~175 s before banana 2 appeared
    1.07 m away; banana 1 was never judged empty (historical_memory to the end) -> a second banana."""

    def run_case(self, on):
        car, t = seeded(one_per_class=on)
        car.t += 175.0                      # out of sight for 175 s, old place never revisited
        car.y = 1.07                        # new place 1.07 m from OLD; OLD out of view from here
        for _ in range(15):
            car.step([(1.5, 1.07, 0.03)])
        return t, car.of("banana")

    def test_on_one_banana(self):
        t, bananas = self.run_case(True)
        self.assertEqual([b.id for b in bananas], [t.id])
        self.assertEqual(bananas[0].number, 1)
        self.assertLess(np.linalg.norm(bananas[0].position - np.array([1.5, 1.07, 0.03])), 0.1)

    def test_off_reproduces_banana_two(self):
        _, bananas = self.run_case(False)
        self.assertEqual([b.number for b in bananas], [1, 2])


class OnePerClass(unittest.TestCase):
    def test_a_move_beyond_exclusion_keeps_id_and_number(self):
        car, t = seeded()
        self.assertNotEqual(t.state, "position_invalid")
        outs = moved_far(car)
        (b,) = car.of("banana")                                          # no "banana 2"
        self.assertEqual((b.id, b.number), (t.id, 1))
        self.assertLess(np.linalg.norm(b.position - FAR), 0.05)
        self.assertEqual(car.memory.identity_stats.get("relocated_one_per_class"), 1)
        vias = [r["via"] for o in outs for r in o.get("relocated", [])]
        self.assertIn("one_per_class", vias)

    def test_a_also_without_existence_v3(self):
        car, t = seeded(existence_v3=False)
        moved_far(car)
        (b,) = car.of("banana")
        self.assertEqual((b.id, b.number), (t.id, 1))
        self.assertLess(np.linalg.norm(b.position - FAR), 0.05)

    def test_b_off_is_the_old_behaviour_a_second_banana(self):
        car, t = seeded(one_per_class=False)
        moved_far(car)
        bs = car.of("banana")
        self.assertEqual(len(bs), 2)
        self.assertEqual(bs[0].id, t.id)
        self.assertLess(np.linalg.norm(bs[0].position - OLD), 0.05)
        self.assertEqual(sorted(x.number for x in bs), [1, 2])

    def test_c_old_place_still_seen_drops_the_candidate(self):
        car, t = seeded()
        rejected = []
        for k in range(30):                                              # alternate frames: old place / elsewhere
            _, out = car.step([OLD if k % 2 else SIDE])
            rejected += out.get("rejected") or []
        (b,) = car.of("banana")
        self.assertEqual((b.id, b.number), (t.id, 1))
        self.assertLess(np.linalg.norm(b.position - OLD), 0.05)          # not moved
        self.assertEqual(car.memory.identity_stats.get("relocated_one_per_class", 0), 0)
        self.assertIn("one_per_class_existing_still_seen", [r["reason"] for r in rejected])

    def test_c_off_creates_a_second(self):                               # the same input made two before
        car, t = seeded(one_per_class=False)
        for k in range(30):
            car.step([OLD if k % 2 else SIDE])
        self.assertEqual(len(car.of("banana")), 2)

    def test_d_two_bananas_in_one_frame_never_make_a_second(self):
        car, t = seeded()
        rejected = []
        for _ in range(20):
            _, out = car.step([OLD, SIDE])
            rejected += out.get("rejected") or []
        (b,) = car.of("banana")
        self.assertEqual((b.id, b.number), (t.id, 1))
        self.assertLess(np.linalg.norm(b.position - OLD), 0.05)
        self.assertIn("one_per_class_covisible", [r["reason"] for r in rejected])

    def test_d_off_two_in_one_frame_make_two(self):
        car, t = seeded(one_per_class=False)
        for _ in range(20):
            car.step([OLD, SIDE])
        self.assertEqual(len(car.of("banana")), 2)

    def test_e_other_classes_unaffected(self):
        car, t = seeded()
        car.y = 2.0
        for _ in range(15):
            car.step([FAR], label="cup")
        (b,) = car.of("banana")
        self.assertLess(np.linalg.norm(b.position - OLD), 0.05)
        (c,) = car.of("cup")
        self.assertEqual(c.number, 1)
        self.assertLess(np.linalg.norm(c.position - FAR), 0.05)
        self.assertEqual(car.memory.identity_stats.get("relocated_one_per_class", 0), 0)

    def test_runtime_switch(self):
        import argparse
        from host.ros_runtime import memory_config as build
        self.assertTrue(build(argparse.Namespace()).one_per_class)
        self.assertTrue(build(argparse.Namespace(one_per_class="on")).one_per_class)
        self.assertFalse(build(argparse.Namespace(one_per_class="off")).one_per_class)


if __name__ == "__main__":
    unittest.main()
