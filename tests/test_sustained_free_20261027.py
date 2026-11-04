"""10-27 sustained-free rule of the see-through filter (owner approved; Config.sustained_free_*,
see_through.SustainedFree; docs/design/MEMORY_EXISTENCE_20261027.md section 6.11).

Same rendered car camera as tests/test_memory_existence_20261027.py; each frame goes process_frame ->
compute_geometry -> SeeThroughFilter.apply(now=frame time) -> update_memory, the order of host/ros_runtime.py.
"""
from dataclasses import replace
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_memory_existence_20261027 as ex  # noqa: E402
from test_memory_candidate_20261027 import phone  # noqa: E402

from perception.fpga_replay import core  # noqa: E402
from perception.fpga_replay.see_through import SeeThroughFilter  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = "fd3eda3"     # the candidate head: its see_through.py has the permanent block


def base_filter_class():
    """SeeThroughFilter as it was on the candidate branch (git show), or None outside a git checkout."""
    try:
        src = subprocess.run(["git", "-C", REPO, "show", BASE + ":perception/fpga_replay/see_through.py"],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        return None
    path = os.path.join(tempfile.mkdtemp(), "see_through_base.py")
    with open(path, "wb") as fh:
        fh.write(src)
    spec = importlib.util.spec_from_file_location("see_through_base", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.SeeThroughFilter


class TimedRig(ex.Rig):
    """ex.Rig with the runtime's filter, given the frame time (pass_now=False: the old call without it)."""

    def __init__(self, cfg, filt=None, pass_now=True):
        ex.Rig.__init__(self, cfg)
        self.filt = filt if filt is not None else SeeThroughFilter(self.cfg)
        self.pass_now = pass_now
        self.force_free = False          # this frame: every track's raw word is free
        self.words = []                  # (raw, used) per frame

    def step(self, T, objects, detect=None, wall_x=3.5, geometry_only=False):
        mod = self.mod
        stamp = self.now
        f = mod.Frame("f%05d" % self.i, stamp, np.zeros((ex.H, ex.W, 3), np.uint8),
                      ex.render(T, [(lo, hi) for _, lo, hi in objects.values()], wall_x),
                      ex.K, T, True, "synthetic", "synthetic", stamp)
        self.i += 1
        dets = []
        for name, (label, lo, hi) in objects.items():
            if detect is not None and name not in detect:
                continue
            bb = ex.project(T, lo, hi)
            if bb is not None:
                dets.append(mod.Detection(label, .9, bb, "synthetic", f.frame_id, f.timestamp))
        obs = mod.process_frame(f, dets, self.cfg)
        geo = mod.compute_geometry(f, self.m, self.cfg)
        if self.force_free:
            geo = {tid: "free" for tid in geo}
        used = self.filt.apply(geo, obs, self.m, now=stamp) if self.pass_now else self.filt.apply(geo, obs, self.m)
        self.words.append((dict(geo), dict(used)))
        out = self.m.update_memory(obs, used, f)
        self.log.append(out)
        return out


def block_phone(rig, T, obj):
    """Phone in place and detected; two frames where v1 also says free -> the filter blocks its free."""
    rig.run(3.0, T, obj)
    (tid,) = rig.m.tracks
    k = 0
    while not rig.filt.blocked(rig.m, tid):
        rig.force_free = True
        rig.step(T, obj)
        rig.force_free = False
        k += 1
        assert k < 10
    rig.run(1.0, T, obj)                 # seen again, no free
    return tid


class SustainedFreeTests(unittest.TestCase):
    def taken_away(self, cfg, seconds=30.0):
        """Blocked phone (P_D class 0.15) taken away in front of the parked car; v1 then reports its place free on
        every frame (what a correct geometry says). Returns (s from removal to position_invalid or None, reason)."""
        T = ex.pose()
        obj = {"p": phone(0.9, 0.)}
        rig = TimedRig(cfg)
        tid = block_phone(rig, T, obj)
        self.assertEqual(ex.state(rig.m, tid), "current_confirmed")
        rig.force_free = True
        took = rig.until(seconds, T, {}, lambda m: ex.state(m, tid) == "position_invalid")
        return took, rig.m.tracks[tid].reason, rig

    def test_blocked_phone_taken_away_falls_after_about_5_s(self):
        took, reason, rig = self.taken_away(core.Config())
        off, _, _ = self.taken_away(replace(core.Config(), sustained_free_override=False))
        print("\n[sustained] blocked phone taken away: invalid after %r s (reason %s); override off: %r"
              % (took, reason, off))
        self.assertIsNotNone(took)
        self.assertGreaterEqual(took, 5.0)                # never before the 5 s window is full
        self.assertLessEqual(took, 7.0)                   # 5 s + the score's few free votes
        self.assertEqual(reason, "free")
        self.assertGreater(rig.filt.counts["sustained_free"], 0)
        self.assertIsNone(off)                            # positive control: the permanent block never lets go

    def intermittent(self, cfg, gap_s, seconds=180.0):
        """Phone stays; detected 0.5 s, then gap_s undetected with v1 falsely saying free on every frame, repeated.
        Returns (invalidations, blocked)."""
        T = ex.pose()
        obj = {"p": phone(0.9, 0.)}
        rig = TimedRig(cfg)
        tid = block_phone(rig, T, obj)
        period = 0.5 + gap_s
        flips, prev = 0, ex.state(rig.m, tid)
        for k in range(int(round(seconds / ex.DT))):
            seen = (k * ex.DT) % period < 0.5
            rig.force_free = not seen
            rig.step(T, obj, detect=None if seen else ())
            st = ex.state(rig.m, tid)
            if st == "position_invalid" and prev != "position_invalid":
                flips += 1
            prev = st
        return flips, rig.filt.blocked(rig.m, tid), rig.filt.counts["sustained_free"]

    def test_intermittent_false_free_up_to_3_8_s_never_invalid_in_3_min(self):
        on = self.intermittent(core.Config(), 3.8)
        ctrl = self.intermittent(core.Config(), 6.0, seconds=60.0)
        print("\n[sustained] phone stays, false free runs of 3.8 s between detections, 180 s: (flips, blocked, "
              "sustained passes) %r; control with 6.0 s runs, 60 s: %r" % (on, ctrl))
        self.assertEqual(on[0], 0)
        self.assertTrue(on[1])
        self.assertEqual(on[2], 0)
        self.assertGreater(ctrl[0], 0)                   # positive control: runs longer than the window do count

    def test_too_few_in_view_windows_or_mixed_words_do_not_fire(self):
        cfg = core.Config()
        from perception.fpga_replay.see_through import SustainedFree
        sf = SustainedFree(cfg)
        # 5 in-view windows only (others out of view), all free, no detection
        for k in range(40):
            t = 100.0 + k * 0.134
            sf.observe("a", t, "free" if k < 12 else "out_of_view", False)
        self.assertFalse(sf.holds("a"))
        # many windows, but only 70 % free (rest surface_or_uncertain)
        sf = SustainedFree(cfg)
        for k in range(40):
            t = 100.0 + k * 0.3
            sf.observe("b", t, "free" if k % 10 < 7 else "surface_or_uncertain", False)
        self.assertFalse(sf.holds("b"))
        # 80 % free -> holds; one detection in the last 5 s -> not
        sf = SustainedFree(cfg)
        for k in range(40):
            sf.observe("c", 100.0 + k * 0.3, "free" if k % 5 else "surface_or_uncertain", False)
        self.assertTrue(sf.holds("c"))
        sf.observe("c", 112.1, "free", True)
        self.assertFalse(sf.holds("c"))

    def test_switch_off_is_bit_identical_to_the_permanent_block(self):
        # sustained_free_override=False (frame time passed) and the old call without a frame time, against the
        # candidate's own see_through.py (git show fd3eda3): same words and same memory output, frame by frame
        Base = base_filter_class()
        if Base is None:
            self.skipTest("no git checkout")
        T = ex.pose()
        obj = {"p": phone(0.9, 0.)}
        cfg_off = replace(core.Config(), sustained_free_override=False)
        rigs = [TimedRig(cfg_off), TimedRig(core.Config(), pass_now=False), TimedRig(core.Config(), filt=Base(core.Config()),
                                                                                       pass_now=False)]
        for rig in rigs:
            block_phone(rig, T, obj)
            rig.force_free = True
            rig.run(20.0, T, {})
            rig.force_free = False
            rig.run(5.0, T, obj)

        def dump(rig):
            return json.dumps([rig.words, rig.log], sort_keys=True, default=str)
        self.assertEqual(dump(rigs[0]), dump(rigs[2]))
        self.assertEqual(dump(rigs[1]), dump(rigs[2]))
        on = TimedRig(core.Config())                      # positive control: on, the output differs
        block_phone(on, T, obj)
        on.force_free = True
        on.run(20.0, T, {})
        on.force_free = False
        on.run(5.0, T, obj)
        self.assertNotEqual(dump(on), dump(rigs[2]))

    def test_filtered_free_still_starts_the_task_search(self):
        # The filter turns a blocked track's free into surface_or_uncertain; task_control's "not seen at the last
        # location" (5 frames with the box in view and no confirmation) must still fire on it.
        from host.task_control import TaskController
        T = ex.pose()
        obj = {"p": phone(0.9, 0.)}
        rig = TimedRig(replace(core.Config(), sustained_free_override=False))
        tid = block_phone(rig, T, obj)
        task = TaskController.__new__(TaskController)     # only the counter under test
        from host.task_control import TaskConfig
        task.config = TaskConfig()
        task._not_seen = 0
        rig.force_free = True
        fired = None
        for k in range(10):
            rig.step(T, {})
            raw, used = rig.words[-1]
            self.assertEqual((raw[tid], used[tid]), ("free", "surface_or_uncertain"))
            row = next(r for r in rig.m.snapshot() if r["id"] == tid)
            self.assertEqual(row["reason"], "surface_or_uncertain")
            if task._count_not_seen(row):
                fired = k + 1
                break
        self.assertEqual(fired, TaskConfig().not_seen_frames)
        self.assertNotEqual(ex.state(rig.m, tid), "position_invalid")   # the search starts before any fall


class ReplaySustainedTests(unittest.TestCase):
    def test_replay_tool_variant(self):
        # existence_log_replay: the "+filter+sustained" variant lets a blocked, really removed object fall; the
        # "+filter" variant (permanent block) does not
        from host.ros_runtime import track_ev_row
        from tools.memory_quality import existence_log_replay as rep
        T = ex.pose()
        obj = {"p": phone(0.9, 0.)}
        rig = TimedRig(core.Config())
        tid = block_phone(rig, T, obj)
        last, rows = {}, []
        for k in range(int(round(20.0 / ex.DT))):
            gone = k * ex.DT >= 2.0
            rig.force_free = gone
            f_id = "f%05d" % rig.i
            stamp = rig.now
            rig.step(T, {} if gone else obj)
            raw, used = rig.words[-1]
            ev = track_ev_row(rig.m, rig.filt, raw, used, last)
            rows.append({"event": "frame_processed", "frame_id": f_id, "acquisition_timestamp": stamp,
                         "detections": [], "tracks": rig.m.snapshot(), "track_ev": ev})
        # the log starts after the block: seed the replay's conflicts with two detected-and-free rows
        seed = []
        for j in range(2):
            seed.append({"event": "frame_processed", "frame_id": "s%d" % j,
                         "acquisition_timestamp": rows[0]["acquisition_timestamp"] - 1.0 + 0.134 * j,
                         "detections": [], "tracks": [dict(rows[0]["tracks"][0], last_seen_frame="s%d" % j)],
                         "track_ev": {tid: ["f", "s", 3.5, 1]}})
        with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as fh:
            for r in seed + rows:
                fh.write(json.dumps(r) + "\n")
        try:
            blocked = rep.summary(fh.name, "worst+filter")
            sustained = rep.summary(fh.name, "worst+filter+sustained")
        finally:
            os.unlink(fh.name)
        print("\n[sustained] replay 20 s, phone removed at 2 s: worst+filter invalidated %d, +sustained %d"
              % (blocked["invalidated"], sustained["invalidated"]))
        self.assertEqual(blocked["invalidated"], 0)
        self.assertEqual(sustained["invalidated"], 1)
        self.assertEqual(ex.state(rig.m, tid), "position_invalid")      # the runtime path agrees


if __name__ == "__main__":
    unittest.main()
