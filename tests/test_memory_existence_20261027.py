"""10-27 existence score + keep-the-number-on-a-move (Config.existence_v3; docs/design/MEMORY_EXISTENCE_20261027.md).

Rendered cases use a car-like camera (0.39 m high, pitched down 7 deg, 7.5 Hz = 0.134 s frames; render idea
from the 10-27 diagnosis notes) through the real process_frame / compute_geometry / update_memory chain with
the default Config (creation gate on). Score-only cases feed geometry words directly.
"""
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import subprocess
import types
import sys
import unittest

import numpy as np

from perception.fpga_replay import core
from perception.fpga_replay.core import Config, Frame, Memory, Track, UNGATED_CREATION

ROOT = Path(__file__).resolve().parents[1]
BASE_COMMIT = "0f377be"            # comp-integ-20261026 before existence_v3 (car version 10-27)

H, W = 200, 320
K = np.array([[225., 0., 160.], [0., 225., 100.], [0., 0., 1.]])
DT = 0.134
T0 = 1000.0


def pose(x=0., y=0., yaw_deg=0., h=.39, pitch_deg=7.):
    p, a = np.radians(pitch_deg), np.radians(yaw_deg)
    r0 = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)      # optical axes in world (x fwd, z up)
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    T = np.eye(4); T[:3, :3] = rz @ ry @ r0; T[:3, 3] = [x, y, h]
    return T


def bottle(x, y, z0=0.0):
    return ("bottle", np.array([x - .035, y - .035, z0]), np.array([x + .035, y + .035, z0 + .20]))


def render(T, boxes, wall_x=3.5):
    v, u = np.mgrid[0:H, 0:W]
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones((H, W))], -1)
    d = rays @ T[:3, :3].T
    o = T[:3, 3]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(d[..., 2] < -1e-6, -o[2] / d[..., 2], np.inf)                      # floor
        if wall_x is not None:
            t = np.minimum(t, np.where(d[..., 0] > 1e-6, (wall_x - o[0]) / d[..., 0], np.inf))
        for lo, hi in boxes:
            t1, t2 = (lo - o) / d, (hi - o) / d
            tn = np.nanmax(np.minimum(t1, t2), -1); tf = np.nanmin(np.maximum(t1, t2), -1)
            hit = (tn <= tf) & (tn > 0)
            t = np.where(hit & (tn < t), tn, t)
    t[~np.isfinite(t)] = np.nan
    return t.astype(np.float32)                 # ray z-component is 1: t is the optical-axis depth


def project(T, lo, hi):
    c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    cam = (c - T[:3, 3]) @ T[:3, :3]
    if (cam[:, 2] <= .1).any():
        return None
    uv = cam @ K.T; uv = uv[:, :2] / uv[:, 2:]
    x1, y1 = uv.min(0); x2, y2 = uv.max(0)
    if x1 < 1 or y1 < 1 or x2 > W - 2 or y2 > H - 2:
        return None
    return (float(x1), float(y1), float(x2), float(y2))


class Rig:
    """One memory driven frame by frame through a given core module (current or the base commit's)."""

    def __init__(self, cfg=None, mod=core):
        self.mod = mod
        self.cfg = cfg if cfg is not None else mod.Config()
        self.m = mod.Memory("geometry", self.cfg)
        self.i = 0
        self.log = []

    @property
    def now(self):
        return T0 + self.i * DT

    def step(self, T, objects, detect=None, wall_x=3.5, geometry_only=False):
        mod = self.mod
        stamp = self.now
        f = mod.Frame("f%05d" % self.i, stamp, np.zeros((H, W, 3), np.uint8), render(T, [(lo, hi) for _, lo, hi in
                                                                                           objects.values()], wall_x),
                      K, T, True, "synthetic", "synthetic", stamp)
        self.i += 1
        if geometry_only:
            out = self.m.update_geometry_only(mod.compute_geometry(f, self.m, self.cfg), f)
            self.log.append(out)
            return out
        dets = []
        for name, (label, lo, hi) in objects.items():
            if detect is not None and name not in detect:
                continue
            bb = project(T, lo, hi)
            if bb is not None:
                dets.append(mod.Detection(label, .9, bb, "synthetic", f.frame_id, f.timestamp))
        obs = mod.process_frame(f, dets, self.cfg)
        out = self.m.update_memory(obs, mod.compute_geometry(f, self.m, self.cfg), f)
        self.log.append(out)
        return out

    def run(self, seconds, T, objects, **kw):
        outs = []
        for _ in range(int(round(seconds / DT))):
            outs.append(self.step(T, objects, **kw))
        return outs

    def until(self, seconds, T, objects, cond, **kw):
        """Step until cond(memory) holds; seconds of acquisition time it took, or None."""
        start = self.now
        for _ in range(int(round(seconds / DT))):
            self.step(T, objects, **kw)
            if cond(self.m):
                return self.now - start      # frames stepped x DT, from the first changed frame
        return None


def state(m, tid):
    return m.tracks[tid].state


class RenderedExistenceTests(unittest.TestCase):
    def seeded(self, objects, T, seconds=3.0, cfg=None, **kw):
        rig = Rig(cfg)
        rig.run(seconds, T, objects, **kw)
        return rig

    def test_1_parked_car_object_taken_in_front_free_and_miss(self):
        # bottle on a stand 0.45 m up, wall 2.5 m behind: removal gives free + miss at 1 m
        T = pose()
        rig = self.seeded({"a": bottle(1.0, 0., .45)}, T, wall_x=None)
        (tid,) = rig.m.tracks
        self.assertEqual(rig.m.tracks[tid].existence, 3.5)
        took = rig.until(5.0, T, {}, lambda m: state(m, tid) == "position_invalid", wall_x=2.5)
        words = {o["geometry"].get(tid) for o in rig.log[-3:]}
        self.assertIn("free", words)
        self.assertIsNotNone(took)
        self.assertLessEqual(took, 2.0)
        self.assertEqual(rig.m.tracks[tid].reason, "free")
        print("\n[1] free+miss, parked: position_invalid after %.2f s" % took)

    def test_2_floor_bottle_misses_only(self):
        T = pose()
        rig = self.seeded({"a": bottle(1.0, 0.)}, T)
        (tid,) = rig.m.tracks
        took = rig.until(5.0, T, {}, lambda m: state(m, tid) == "position_invalid")
        self.assertNotIn("free", {o["geometry"].get(tid) for o in rig.log[-5:]})   # the floor keeps it from free
        self.assertIsNotNone(took)
        self.assertLessEqual(took, 2.0)
        self.assertEqual(rig.m.tracks[tid].reason, "not_detected")
        print("\n[2] misses only (floor bottle 1 m), parked: position_invalid after %.2f s" % took)

    def test_6_back_in_place_recovers_same_id(self):
        T = pose()
        obj = {"a": bottle(1.0, 0.)}
        rig = self.seeded(obj, T)
        (tid,) = rig.m.tracks
        rig.until(5.0, T, {}, lambda m: state(m, tid) == "position_invalid")
        rig.step(T, obj)
        self.assertEqual(list(rig.m.tracks), [tid])
        self.assertEqual(state(rig.m, tid), "current_confirmed")
        rig.run(1.0, T, obj)
        self.assertEqual(state(rig.m, tid), "current_confirmed")
        self.assertGreater(rig.m.tracks[tid].existence, 1.0)

    def test_detection_frame_that_closes_a_falling_window_stays_confirmed(self):
        # 10-27 host whole-flow simulation (memory-sim): a detection arriving in the frame that closes a miss
        # window whose vote crosses invalid_below left the track position_invalid IN the detection frame
        # (_ex_hit rolled the window after _touch had set current_confirmed), reason "not_detected" with a box
        # on it; it stayed so until the next detected frame.
        T = pose()
        obj = {"a": bottle(1.0, 0.)}
        rig = self.seeded(obj, T)
        (tid,) = rig.m.tracks
        t = rig.m.tracks[tid]
        w = rig.m._ex_window
        while not (t.existence is not None and t.existence <= -1.0 and t.ex_miss_pd is not None):
            rig.step(T, {})
            self.assertNotEqual(state(rig.m, tid), "position_invalid")
        while w(rig.now) == t.ex_window:          # finish this (missing) window without a box
            rig.step(T, {})
        self.assertNotEqual(state(rig.m, tid), "position_invalid")
        rig.step(T, obj)                          # first frame of the next window: the box is back
        self.assertEqual(t.last_seen_frame, "f%05d" % (rig.i - 1))
        self.assertEqual(state(rig.m, tid), "current_confirmed")
        self.assertEqual(t.reason, "new_visual_observation")

    def test_7a_new_place_seen_first_then_old_place_found_empty_merges(self):
        old, new = bottle(1.0, 0.), bottle(1.0, 1.0)             # moved 1.0 m
        rig = self.seeded({"a": old}, pose(), cfg=Config(one_per_class=False))   # 10-31: two-track scenario (one_per_class would take over at once)
        (tid,) = rig.m.tracks
        number = rig.m.tracks[tid].number
        rig.run(3.0, pose(yaw_deg=45.), {"a": new})              # only the new place in view: a second track
        self.assertEqual(len(rig.m.tracks), 2)
        (other,) = set(rig.m.tracks) - {tid}
        rig.until(5.0, pose(), {"a": new}, lambda m: other not in m.tracks)   # back to the old place: empty
        self.assertEqual(list(rig.m.tracks), [tid])
        t = rig.m.tracks[tid]
        self.assertEqual(t.number, number)
        self.assertLess(np.linalg.norm(t.position[:2] - [1.0, 1.0]), .1)      # at the new place, not averaged
        self.assertEqual(rig.m.resolve(other), tid)
        self.assertNotEqual(t.state, "position_invalid")
        rows = [r for o in rig.log for r in o.get("merged", ())]
        self.assertEqual(rows[-1]["reason"], "relocated")

    def test_7b_old_place_found_empty_first_then_new_place_takes_over(self):
        old, new = bottle(1.0, 0.), bottle(1.0, 1.0)
        rig = self.seeded({"a": old}, pose())
        (tid,) = rig.m.tracks
        number = rig.m.tracks[tid].number
        rig.until(5.0, pose(), {}, lambda m: state(m, tid) == "position_invalid")
        rig.run(3.0, pose(yaw_deg=45.), {"a": new})
        self.assertEqual(list(rig.m.tracks), [tid])
        t = rig.m.tracks[tid]
        self.assertEqual((t.number, t.state), (number, "current_confirmed"))
        self.assertLess(np.linalg.norm(t.position[:2] - [1.0, 1.0]), .1)
        rows = [r for o in rig.log for r in o.get("relocated", ())]
        self.assertEqual([(r["track_id"], r["via"]) for r in rows], [(tid, "position_invalid")])

    def test_8_co_visible_twin_neither_merged_nor_taken_over(self):
        a, b = bottle(1.0, -.3), bottle(1.0, .3)
        T = pose()
        rig = self.seeded({"a": a}, T, cfg=Config(one_per_class=False))   # 10-31: two real bottles
        (ta,) = rig.m.tracks
        rig.run(3.0, T, {"a": a, "b": b})                        # B appears later, seen together with A
        (tb,) = set(rig.m.tracks) - {ta}
        self.assertIn(tb, rig.m.tracks[ta].covis)
        rig.until(5.0, T, {"b": b}, lambda m: state(m, ta) == "position_invalid")
        rig.run(2.0, T, {"b": b})
        self.assertEqual(sorted(rig.m.tracks), sorted([ta, tb]))
        self.assertEqual(state(rig.m, ta), "position_invalid")
        self.assertEqual(state(rig.m, tb), "current_confirmed")
        self.assertLess(np.linalg.norm(rig.m.tracks[tb].position[:2] - [1.0, .3]), .1)

    def test_9_keep_ids_never_merged_away(self):
        old, new = bottle(1.0, 0.), bottle(1.0, 1.0)
        rig = self.seeded({"a": old}, pose(), cfg=Config(one_per_class=False))   # 10-31: two-track scenario (one_per_class would take over at once)
        (tid,) = rig.m.tracks
        rig.run(3.0, pose(yaw_deg=45.), {"a": new})
        (other,) = set(rig.m.tracks) - {tid}
        rig.m.keep_ids = {other}                                 # e.g. the task's target is the new track
        rig.until(5.0, pose(), {"a": new}, lambda m: tid not in m.tracks or state(m, tid) == "position_invalid")
        self.assertEqual(list(rig.m.tracks), [other])
        self.assertEqual(rig.m.resolve(tid), other)
        self.assertLess(np.linalg.norm(rig.m.tracks[other].position[:2] - [1.0, 1.0]), .1)

    def test_geometry_only_entry_counts_free(self):
        T = pose()
        rig = self.seeded({"a": bottle(1.0, 0., .45)}, T, wall_x=None)
        (tid,) = rig.m.tracks
        start = rig.now
        for _ in range(30):
            rig.step(T, {}, wall_x=2.5, geometry_only=True)
            if state(rig.m, tid) == "position_invalid":
                break
        self.assertEqual(state(rig.m, tid), "position_invalid")
        self.assertLessEqual(rig.now - start, 2.0)


def score_memory(label="bottle", x=1.0, existence=3.5, cfg=None):
    m = Memory("geometry", cfg or Config())
    p = np.array([x, 0., .1])
    t = Track("object-0001", label, "red", p, p - .05, p + .05, T0, created_at=T0, number=1)
    t.existence = existence
    m.tracks[t.id] = t
    return m, t


def flat_frame(i, x_cam=0.0):
    T = pose(x=x_cam)
    return Frame("s%06d" % i, T0 + 1 + i * DT, np.zeros((4, 4, 3), np.uint8), np.ones((4, 4), np.float32),
                 np.array([[2., 0, 2], [0, 2., 2], [0, 0, 1]]), T)


def feed(m, word, seconds, start=0, x_cam=0.0):
    n = int(round(seconds / DT))
    for i in range(start, start + n):
        m.update_memory([], {"object-0001": word}, flat_frame(i, x_cam))
    return start + n


class ScoreTests(unittest.TestCase):
    def test_3_low_detection_class_misses_alone_do_not_invalidate_in_3_min(self):
        m, t = score_memory("cell phone")
        feed(m, "surface_or_uncertain", 180.0)
        self.assertEqual(t.state, "historical_memory")
        self.assertAlmostEqual(t.existence, Config().miss_floor)
        # the plan's pure sum would have dropped it: P_D 0.15 -> 0.5 ln 0.85 per window
        m2, t2 = score_memory("cell phone", cfg=Config(miss_decisive_pd=0.0))
        feed(m2, "surface_or_uncertain", 180.0)
        self.assertEqual(t2.state, "position_invalid")
        # a free vote still decides at once from the floor
        feed(m, "free", 0.7, start=5000)
        self.assertEqual(t.state, "position_invalid")

    def test_4_far_misses_do_not_count(self):
        m, t = score_memory("bottle", x=2.0 + .4)                # optical depth ~2 m from the camera
        feed(m, "surface_or_uncertain", 60.0)
        self.assertEqual((t.state, t.existence), ("historical_memory", 3.5))

    def test_5_occluded_out_of_view_and_others_never_invalidate_nor_reset(self):
        for word in ("occluded", "out_of_view", "too_small", "invalid_depth", "unknown"):
            m, t = score_memory(existence=-1.0)
            feed(m, word, 120.0)
            self.assertNotEqual(t.state, "position_invalid", word)
            self.assertEqual(t.existence, -1.0, word)
        m, t = score_memory()
        n = feed(m, "occluded", 10.0)
        self.assertEqual(t.state, "temporarily_occluded")
        # surface_or_uncertain no longer erases negative evidence: it is a miss vote here (1 m bottle)
        feed(m, "surface_or_uncertain", 0.6, start=n)
        self.assertLess(t.existence, 3.5)

    def test_same_class_box_in_gate_blocks_negative_votes(self):
        m, t = score_memory()
        for i in range(60):
            f = flat_frame(i)
            # a same-class box in the track's gate that the admission gates reject (below ground): not an
            # update of the track, but the object may be there -- neither free nor miss may vote
            p = t.position + [0., 0., -.2]
            o = core.Observation("bottle", .9, p, p - .05, p + .05, "red", 1., f.frame_id, f.timestamp, (0, 0, 1, 1),
                                 "detector", range_m=1.0)
            out = m.update_memory([o], {"object-0001": "free"}, f)
            self.assertEqual(out["rejected"][0]["reason"], "below_ground")
        self.assertEqual((t.state, t.existence), ("historical_memory", 3.5))
        feed(m, "free", 1.5, start=100)                          # control: without the box it empties
        self.assertEqual(t.state, "position_invalid")

    def test_window_votes_once(self):
        cfg = Config()
        m, t = score_memory(existence=0.0)
        feed(m, "free", 0.0 + 3 * DT)                           # frames inside one or two windows
        expected_votes = len({int(math.floor((T0 + 1 + i * DT) / cfg.existence_window_s)) for i in range(3)}) - 1
        per_vote = cfg.existence_free + cfg.miss_weight * math.log(1 - cfg.miss_pd_cap)
        self.assertAlmostEqual(t.existence, max(cfg.existence_min, expected_votes * per_vote))

    def test_snapshot_carries_existence(self):
        m, t = score_memory(existence=1.23456)
        self.assertEqual(m.snapshot()[0]["existence"], 1.235)
        m_off = Memory("geometry", Config(existence_v3=False))
        p = np.array([1., 0., .1])
        m_off.tracks["object-0001"] = Track("object-0001", "bottle", "red", p, p - .05, p + .05, T0)
        self.assertNotIn("existence", m_off.snapshot()[0])


def load_base_core():
    try:
        src = subprocess.check_output(["git", "show", "%s:perception/fpga_replay/core.py" % BASE_COMMIT],
                                      cwd=str(ROOT), stderr=subprocess.DEVNULL).decode("utf-8")
    except (OSError, subprocess.CalledProcessError):
        return None
    mod = types.ModuleType("core_base_0f377be")
    mod.__file__ = str(ROOT / "perception" / "fpga_replay" / "core.py")
    sys.modules[mod.__name__] = mod
    exec(compile(src, mod.__file__, "exec"), mod.__dict__)
    return mod


def script(rig):
    """A sequence touching every branch the switch guards: creation, removal, move both orders, covis twin,
    geometry-only entry, occlusion by the other object."""
    a, b, moved = bottle(1.0, -.3), bottle(1.0, .3), bottle(1.0, 1.0)
    rig.run(3.0, pose(), {"a": a})
    rig.run(2.0, pose(), {"a": a, "b": b})
    rig.run(2.5, pose(), {"b": b})
    rig.run(1.0, pose(x=.12), {"b": b})
    rig.run(1.0, pose(x=.25), {"b": b}, geometry_only=True)
    rig.run(3.0, pose(yaw_deg=45.), {"b": moved})
    rig.run(3.0, pose(), {"b": moved})
    rig.run(1.0, pose(x=.12), {"b": moved, "a": a}, detect={"b"})
    rig.run(1.0, pose(x=.3), {}, wall_x=2.0)


def canon(rig):
    return json.dumps({"log": rig.log, "merged": rig.m.merged, "stats": rig.m.identity_stats,
                       "rejected": rig.m.rejected, "candidates": rig.m.candidate_stats},
                      sort_keys=True, default=lambda v: v.tolist() if hasattr(v, "tolist") else repr(v))


class LegacyEquivalenceTests(unittest.TestCase):
    def test_10_switch_off_is_bit_identical_to_the_base_commit(self):
        base = load_base_core()
        if base is None:
            self.skipTest("git history with %s not available" % BASE_COMMIT)
        for extra in ({}, UNGATED_CREATION):
            with self.subTest(extra=sorted(extra)):
                old = Rig(base.Config(**extra), base)
                new = Rig(Config(existence_v3=False, one_per_class=False, **extra))   # 10-31: both switches off
                script(old); script(new)
                self.assertGreater(len(old.log), 100)
                self.assertEqual(canon(new), canon(old))
                # positive control: with the switch on the same script gives a different log
                on = Rig(Config(**extra)); script(on)
                self.assertNotEqual(canon(on), canon(old))


class StoreRoundTripTests(unittest.TestCase):
    def doc(self, m):
        from host import memory_store as ms
        d = ms.dump(m, complete=True, runtime_boot_id="boot", world_frame="map", pose_source="reference",
                    sensor_source="synthetic", map_identity_doc={})
        return json.loads(json.dumps(d, allow_nan=False))

    def test_11_dump_validate_restore_keeps_existence(self):
        from host import memory_store as ms
        m, t = score_memory(existence=-0.4567)
        p = np.array([2., 1., .1])
        u = Track("object-0002", "cup", "red", p, p - .05, p + .05, T0, state="position_invalid", number=1)
        m.tracks[u.id] = u                                       # no score yet: restored from its state
        d = self.doc(m)
        self.assertEqual(d["tracks"][0]["existence"], -0.457)
        self.assertNotIn("existence", d["tracks"][1])
        ms.validate_document(d)
        r = Memory("geometry", Config())
        ms.restore(d, r)
        self.assertEqual(r.tracks["object-0001"].existence, -0.457)
        self.assertEqual(r.tracks["object-0002"].existence, Config().invalid_below)
        self.assertEqual(self.doc(r)["tracks"][0]["existence"], -0.457)
        bad = json.loads(json.dumps(d)); bad["tracks"][0]["existence"] = "x"
        with self.assertRaises(ms.MemoryStoreError):
            ms.validate_document(bad)
        off = Memory("geometry", Config(existence_v3=False))
        ms.restore(d, off)
        self.assertIsNone(off.tracks["object-0001"].existence)
        self.assertNotIn("existence", off.snapshot()[0])


class TaskWithMoveTests(unittest.TestCase):
    def test_invalid_target_found_elsewhere_keeps_its_id(self):
        """Behaviour change of step 2 (v3): the replacement found in search is the target itself (same id),
        found by fresh_matching_observation instead of search_single_candidate / user selection."""
        from host.task_control import PoseGoal, TaskController
        from perception.fpga_replay.core import compute_geometry, process_frame
        from perception.fpga_replay.synthetic import scene
        m = Memory("geometry", replace(Config(), **UNGATED_CREATION))

        def update(pair):
            frame, dets = pair
            m.update_memory(process_frame(frame, dets), compute_geometry(frame, m), frame)
            return frame
        update(scene(0))
        m.tracks["object-0001"].state = "position_invalid"
        task = TaskController(m, lambda row: [PoseGoal(0, 0, 0)], [PoseGoal(1, 0, 0), PoseGoal(2, 0, 0)])

        @dataclass
        class Stamp:
            frame_id: str
            timestamp: float
            pose_valid: bool = True
        command = task.start("找红色杯子", 0, Stamp("initial", 0))[0]
        task.navigation_result(command.command_id, True, .5)
        frame, dets = scene(1)
        frame.T_world_camera[0, 3] = .8
        update((frame, dets))
        self.assertEqual(list(m.tracks), ["object-0001"])
        task.observe(frame, 1)
        self.assertEqual((task.state, task.reason, task.target_id), ("found", "fresh_matching_observation",
                                                                     "object-0001"))


if __name__ == "__main__":
    unittest.main()
