"""10-27 night: fast removal (Config.miss_reliable) + the map shows a removal at once (memory_publish / hub).

Owner: an object taken away in front of the parked car left memory too slowly. Car log 20261027-205546: key and
drink can (class factor 0.5) took 9.6 s / 8.1 s. Rendered cases reuse the car-like camera of
test_memory_existence_20261027 through process_frame / compute_geometry / update_memory.
"""
from dataclasses import replace
import json
import subprocess
import sys
import types
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_memory_existence_20261027 import (DT, Rig, bottle, canon, feed, pose, score_memory,  # noqa: E402
                                            script, state)
from perception.fpga_replay.core import Config, UNGATED_CREATION  # noqa: E402
from host.memory_publish import MemoryFeed  # noqa: E402
from host.hub.hub_core import HubCore, NS  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
BASE_COMMIT = "036f82a"            # the car version of 10-27 evening (existence score, no fast removal)
MS = NS // 1000


def labelled(label, x=1.0):
    _, lo, hi = bottle(x, 0.)
    return (label, lo, hi)


def removal_time(cfg, label):
    T = pose()
    rig = Rig(cfg)
    rig.run(5.0, T, {"a": labelled(label)})
    (tid,) = rig.m.tracks
    took = rig.until(15.0, T, {}, lambda m: state(m, tid) == "position_invalid")
    return took, rig.m.tracks[tid]


class RenderedFastRemovalTests(unittest.TestCase):
    def test_parked_floor_object_taken_away(self):
        rows = []
        for label in ("bottle", "drink can"):
            off, _ = removal_time(Config(miss_reliable=False), label)
            on, t = removal_time(Config(), label)
            rows.append((label, off, on))
            self.assertIsNotNone(on)
            self.assertEqual(t.reason, "not_detected")
            self.assertLessEqual(on, off)
        # class factor 0.5 (drink can, key, mouse, cup ...): 15 windows before, 5 now
        (_, b_off, b_on), (_, c_off, c_on) = rows
        self.assertGreater(c_off, 4.0)
        self.assertLessEqual(c_on, 2.0)
        self.assertLessEqual(b_on, 2.0)
        for label, off, on in rows:
            print("\n[fast] %s floor 1 m, parked: %.2f s -> %.2f s" % (label, off, on))

    def test_low_pd_class_unchanged(self):
        # a phone (P_D 0.15) seen in every frame and then gone: misses still only prime the score (miss_floor)
        T = pose()
        for cfg in (Config(), Config(miss_reliable=False)):
            rig = Rig(cfg)
            rig.run(5.0, T, {"a": labelled("cell phone")})
            (tid,) = rig.m.tracks
            rig.run(30.0, T, {})
            self.assertNotEqual(state(rig.m, tid), "position_invalid")
            self.assertAlmostEqual(rig.m.tracks[tid].existence, cfg.miss_floor)


class ScoreFastRemovalTests(unittest.TestCase):
    def hits(self, m, t, n, start=0):
        for i in range(start, start + n):
            m._ex_hit(t, 2000.0 + i * 0.3)
        return start + n

    def test_reliable_needs_enough_windows_and_rate(self):
        m, t = score_memory("drink can")
        self.hits(m, t, 3)
        self.assertIsNone(t.ex_rate)                     # 3 windows < miss_reliable_min_windows
        self.hits(m, t, 1, start=3)
        self.assertEqual(t.ex_rate, 1.0)

    def test_old_detection_votes_normally(self):
        # a track last detected long ago (> miss_reliable_recent_s) gets the plain class vote again
        m, t = score_memory("drink can")
        cfg = m.cfg
        p0 = m._ex_window(1000.0)
        t.ex_seen = [(p0 - k, 1) for k in range(6)]
        t.ex_rate, t.ex_last_hit = 1.0, p0
        t.ex_window, t.ex_miss_pd = p0 + 1, 0.5
        self.assertTrue(m._reliable_miss(t))
        t.ex_window = p0 + int(cfg.miss_reliable_recent_s / cfg.existence_window_s) + 1
        self.assertFalse(m._reliable_miss(t))

    def test_switch_off_counts_like_before(self):
        m, t = score_memory("drink can", cfg=Config(miss_reliable=False))
        feed(m, "surface_or_uncertain", 2.0)
        self.assertAlmostEqual(t.existence, 3.5 + 6 * 0.5 * np.log(0.5), places=6)


def load_base_core():
    try:
        src = subprocess.check_output(["git", "show", "%s:perception/fpga_replay/core.py" % BASE_COMMIT],
                                      cwd=str(ROOT), stderr=subprocess.DEVNULL).decode("utf-8")
    except (OSError, subprocess.CalledProcessError):
        return None
    mod = types.ModuleType("core_base_036f82a")
    mod.__file__ = str(ROOT / "perception" / "fpga_replay" / "core.py")
    sys.modules[mod.__name__] = mod
    exec(compile(src, mod.__file__, "exec"), mod.__dict__)
    return mod


def removal_script(rig):
    T = pose()
    rig.run(5.0, T, {"a": labelled("drink can")})
    rig.run(6.0, T, {})
    rig.run(2.0, T, {"a": labelled("drink can")})
    rig.run(1.0, T, {})
    rig.run(1.0, T, {"a": labelled("drink can")})
    rig.run(6.0, T, {})


class LegacyEquivalenceTests(unittest.TestCase):
    def test_switch_off_is_bit_identical_to_036f82a(self):
        base = load_base_core()
        if base is None:
            self.skipTest("git history with %s not available" % BASE_COMMIT)
        for extra in ({}, UNGATED_CREATION):
            for scr in (script, removal_script):
                with self.subTest(extra=sorted(extra), script=scr.__name__):
                    old = Rig(base.Config(**extra), base)
                    new = Rig(Config(miss_reliable=False, **extra))
                    scr(old); scr(new)
                    self.assertGreater(len(old.log), 100)
                    self.assertEqual(canon(new), canon(old))
        on, old = Rig(Config()), Rig(base.Config(), base)
        removal_script(on); removal_script(old)
        self.assertNotEqual(canon(on), canon(old))       # positive control


class _Mem(object):
    """Stand-in Memory for MemoryFeed: tracks with a state."""
    class _T(object):
        def __init__(self, tid, st):
            self.id, self.label, self.color, self.state, self.reason = tid, "bottle", "red", st, ""
            self.position = np.zeros(3); self.lower = np.zeros(3); self.upper = np.zeros(3)
            self.last_seen, self.last_seen_frame, self.negative_poses, self.history = 0.0, "", [], []

    def __init__(self, states):
        self.tracks = {k: self._T(k, v) for k, v in states.items()}
        self.cfg = Config()
        self.strategy = "geometry"


class DisplayPushTests(unittest.TestCase):
    def test_feed_publishes_removal_inside_the_window(self):
        feed_ = MemoryFeed("boot", "map", "test")
        self.assertIsNotNone(feed_.step(_Mem({"a": "current_confirmed"}), 0.0, 1.0))
        # an ordinary change inside 0.5 s is still held
        self.assertIsNone(feed_.step(_Mem({"a": "historical_memory"}), 0.1, 1.1))
        # leaving the map is not
        msg = feed_.step(_Mem({"a": "position_invalid"}), 0.2, 1.2)
        self.assertIsNotNone(msg)
        self.assertEqual(msg["tracks"][0]["state"], "position_invalid")
        self.assertEqual(feed_.counts["shown_changed_now"], 1)
        # coming back is not held either
        self.assertIsNotNone(feed_.step(_Mem({"a": "current_confirmed"}), 0.3, 1.3))

    def test_hub_sends_removal_on_the_next_tick(self):
        def msg(rev, st):
            return json.dumps({"v": 1, "runtime_boot_id": "b", "revision": rev, "stamp_ros_s": 1.0,
                               "world_frame": "map", "tracks": [{"id": "a", "label": "bottle", "state": st,
                                                                 "position_m": [0, 0, 0]}]})
        core = HubCore("live")
        core.feed_memory(msg(1, "current_confirmed"), 0)
        self.assertIsNotNone(core.memory_body(10 * MS))
        core.feed_memory(msg(2, "historical_memory"), 20 * MS)
        self.assertIsNone(core.memory_body(100 * MS))            # ordinary change: < 0.5 s since last
        core.feed_memory(msg(3, "position_invalid"), 150 * MS)
        body = core.memory_body(200 * MS)
        self.assertIsNotNone(body)
        self.assertEqual(body["revision"], 3)


if __name__ == "__main__":
    unittest.main()
