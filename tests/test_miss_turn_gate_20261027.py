"""10-27 turn gate (Config.miss_turn_gate; docs/design/MEMORY_EXISTENCE_20261027.md "转弯漏检"): a missed detection
is no existence evidence while the camera turns faster than miss_max_turn_dps (and miss_turn_hold_s after).

Rendered cases reuse the car-like camera and the real process_frame / compute_geometry / update_memory chain of
tests/test_memory_existence_20261027.py.
"""
import math
from pathlib import Path
import subprocess
import sys
import types
import unittest

from perception.fpga_replay import core
from perception.fpga_replay.core import Config, UNGATED_CREATION

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_memory_existence_20261027 import DT, Rig, bottle, canon, pose, script, state  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
BASE_COMMIT = "6d76a3d"            # memory-sim branch before the turn gate


def tri(i, dps, amp):
    """Yaw (deg) of frame i for a triangle sweep -amp..+amp at dps deg/s (starts at 0, rising)."""
    period = 4.0 * amp / dps
    t = (i * DT) % period
    q = t * dps
    if q <= amp:
        return q
    if q <= 3 * amp:
        return 2 * amp - q
    return q - 4 * amp


def sweep(rig, seconds, dps, amp, objects, cond=None, **kw):
    """Step a yaw sweep in place; with cond, seconds until cond(memory) holds (None: never)."""
    start = rig.now
    for k in range(int(round(seconds / DT))):
        rig.step(pose(yaw_deg=tri(k, dps, amp)), objects, **kw)
        if cond is not None and cond(rig.m):
            return rig.now - start
    return None


def seeded(cfg=None):
    rig = Rig(cfg)
    rig.run(3.0, pose(), {"a": bottle(1.0, 0.)})
    (tid,) = rig.m.tracks
    return rig, tid


def invalid(tid):
    return lambda m: state(m, tid) == "position_invalid"


class TurnGateTests(unittest.TestCase):
    def test_1_fast_turn_misses_in_full_view_do_not_invalidate(self):
        # 40 deg/s sweep +-20 deg, bottle (1 m, floor) fully in view, never detected for 2 s
        obj = {"a": bottle(1.0, 0.)}
        rig, tid = seeded()
        took = sweep(rig, 2.0, 40.0, 20.0, obj, invalid(tid), detect=set())
        words = {o["geometry"].get(tid) for o in rig.log[-15:]}
        self.assertIn("surface_or_uncertain", words)              # seen (no out_of_view): misses would count
        self.assertNotIn("out_of_view", words)
        self.assertIsNone(took)
        # only the sweep's first frame (yaw 0, still at rest) is a counted miss: 3.5 + 0.5 ln(0.1) = 2.35
        self.assertAlmostEqual(rig.m.tracks[tid].existence, 3.5 + 0.5 * math.log(0.1), 6)
        rates, skips = [], []
        r2 = Rig(); r2.run(3.0, pose(), obj)
        for k in range(1, int(round(2.0 / DT))):
            r2.step(pose(yaw_deg=tri(k, 40.0, 20.0)), obj, detect=set())
            rates.append(r2.m.turn_dps); skips.append(r2.m.turn_skip_miss)
        self.assertTrue(all(skips))                                # turnarounds are covered by the hold
        # positive control: gate off, same sequence -> invalid (misses only, as when parked)
        off, tid_off = seeded(Config(miss_turn_gate=False))
        took_off = sweep(off, 2.0, 40.0, 20.0, obj, invalid(tid_off), detect=set())
        self.assertIsNotNone(took_off)
        self.assertEqual(off.m.tracks[tid_off].reason, "not_detected")
        print("\n[turn 1] 40 deg/s, 2 s misses in view: gate on never invalid (L=%.2f, rate %.1f..%.1f deg/s); "
              "gate off invalid after %.2f s" % (rig.m.tracks[tid].existence, min(r for r in rates if r),
                                                 max(r for r in rates if r), took_off))

    def test_2_parked_removal_times_unchanged(self):
        for name, stand, wall in (("free+miss", .45, 2.5), ("miss only", 0.0, 3.5)):
            with self.subTest(name):
                took = {}
                for gate in (True, False):
                    rig = Rig(Config(miss_turn_gate=gate))
                    rig.run(3.0, pose(), {"a": bottle(1.0, 0., stand)}, wall_x=None if stand else 3.5)
                    (tid,) = rig.m.tracks
                    took[gate] = rig.until(5.0, pose(), {}, invalid(tid), wall_x=wall)
                    self.assertFalse(rig.m.turn_skip_miss)
                self.assertIsNotNone(took[True])
                self.assertEqual(took[True], took[False])
                print("\n[turn 2] parked removal, %s: invalid after %.2f s (gate on) = %.2f s (gate off)"
                      % (name, took[True], took[False]))

    def test_3_slow_turn_removal_still_invalidates(self):
        took = {}
        for gate in (True, False):
            rig, tid = seeded(Config(miss_turn_gate=gate))
            took[gate] = sweep(rig, 5.0, 10.0, 10.0, {}, invalid(tid))
            self.assertLessEqual(rig.m.turn_dps, 10.5)
            self.assertFalse(rig.m.turn_skip_miss)
        self.assertIsNotNone(took[True])
        self.assertEqual(took[True], took[False])
        print("\n[turn 3] 10 deg/s sweep, bottle taken: invalid after %.2f s (gate on) = %.2f s (gate off)"
              % (took[True], took[False]))

    def test_hold_and_gap(self):
        m = core.Memory("geometry", Config())
        f = lambda i, t, yaw: core.Frame("g%d" % i, t, None, None, None, pose(yaw_deg=yaw), True, "s", "s", t)
        self.assertFalse(m._turn_frame(f(0, 10.0, 0.), True)); self.assertIsNone(m.turn_dps)
        self.assertTrue(m._turn_frame(f(1, 10.1, 5.), True)); self.assertAlmostEqual(m.turn_dps, 50.0, 3)
        self.assertTrue(m._turn_frame(f(2, 10.3, 5.), True)); self.assertLess(m.turn_dps, 1e-3)   # hold 0.3 s
        self.assertFalse(m._turn_frame(f(3, 10.45, 5.), True))                                   # hold over
        self.assertFalse(m._turn_frame(f(4, 11.6, 60.), True)); self.assertIsNone(m.turn_dps)     # gap > 1 s
        self.assertFalse(m._turn_frame(f(5, 11.7, 60.), False)); self.assertIsNone(m.turn_dps)    # invalid pose


def load_base_core():
    try:
        src = subprocess.check_output(["git", "show", "%s:perception/fpga_replay/core.py" % BASE_COMMIT],
                                      cwd=str(ROOT), stderr=subprocess.DEVNULL).decode("utf-8")
    except (OSError, subprocess.CalledProcessError):
        return None
    mod = types.ModuleType("core_base_6d76a3d")
    mod.__file__ = str(ROOT / "perception" / "fpga_replay" / "core.py")
    sys.modules[mod.__name__] = mod
    exec(compile(src, mod.__file__, "exec"), mod.__dict__)
    return mod


def turning_script(rig):
    script(rig)
    obj = {"a": bottle(1.0, 0.)}
    rig.run(3.0, pose(), obj)
    sweep(rig, 2.0, 40.0, 20.0, obj, detect=set())
    sweep(rig, 2.0, 40.0, 20.0, {})
    sweep(rig, 3.0, 10.0, 10.0, {})


class LegacyEquivalenceTests(unittest.TestCase):
    def test_4_gate_off_is_bit_identical_to_6d76a3d(self):
        base = load_base_core()
        if base is None:
            self.skipTest("git history with %s not available" % BASE_COMMIT)
        for extra in ({}, UNGATED_CREATION):
            with self.subTest(extra=sorted(extra)):
                old = Rig(base.Config(**extra), base)
                new = Rig(Config(miss_turn_gate=False, **extra))
                turning_script(old); turning_script(new)
                self.assertGreater(len(old.log), 150)
                self.assertEqual(canon(new), canon(old))
                on = Rig(Config(**extra)); turning_script(on)       # positive control: the gate changes the log
                self.assertNotEqual(canon(on), canon(old))


class RuntimeRowTests(unittest.TestCase):
    def test_miss_turn_row(self):
        from host.ros_runtime import miss_turn_row
        m = core.Memory("geometry", Config())
        self.assertEqual(miss_turn_row(m), [None, 0])
        m.turn_dps, m.turn_skip_miss = 41.234, True
        self.assertEqual(miss_turn_row(m), [41.2, 1])
        self.assertIsNone(miss_turn_row(core.Memory("geometry", Config(existence_v3=False))))


if __name__ == "__main__":
    unittest.main()
