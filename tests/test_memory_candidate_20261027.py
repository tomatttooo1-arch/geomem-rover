"""10-27 candidate merge: existence_v3 + geometry v2 + see-through filter together
(docs/design/MEMORY_EXISTENCE_20261027.md, section "候选合并（10-27）").

Same rendered car camera as tests/test_memory_existence_20261027.py (0.39 m high, 7 deg down, 0.134 s
frames, default Config with the creation gate on); each frame goes process_frame -> compute_geometry ->
SeeThroughFilter.apply -> update_memory, the order of host/ros_runtime.py.
"""
from dataclasses import replace
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_memory_existence_20261027 as ex  # noqa: E402

from perception.fpga_replay import core  # noqa: E402
from perception.fpga_replay.see_through import SeeThroughFilter  # noqa: E402


def phone(x, y):
    """A flat small object (phone lying in a case, 3.5 cm high): v1 cannot judge it reliably."""
    return ("cell phone", np.array([x - .075, y - .04, 0.0]), np.array([x + .075, y + .04, .035]))


class FilteredRig(ex.Rig):
    """ex.Rig with the runtime's see-through filter and an optional geometry override per frame."""

    def __init__(self, cfg):
        ex.Rig.__init__(self, cfg)
        self.filt = SeeThroughFilter(self.cfg)
        self.force_free = False          # this frame: every track's raw word is free (a v1 false report)

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
        out = self.m.update_memory(obs, self.filt.apply(geo, obs, self.m), f)
        self.log.append(out)
        return out


V2 = replace(core.Config(), geometry_v2=True)


class CandidateTests(unittest.TestCase):
    def test_v2_floor_bottle_taken_parked(self):
        # existence_v3 + geometry v2 (+ filter): floor bottle 1 m, taken in front of a parked car
        T = ex.pose()
        out = {}
        for name, cfg in (("v2", V2), ("v1", core.Config())):
            rig = FilteredRig(cfg)
            rig.run(3.0, T, {"a": ex.bottle(1.0, 0.)})
            (tid,) = rig.m.tracks
            took = rig.until(5.0, T, {}, lambda m: ex.state(m, tid) == "position_invalid")
            out[name] = (took, rig.m.tracks[tid].reason, {o["geometry"].get(tid) for o in rig.log[-3:]})
        print("\n[candidate] floor bottle 1 m taken, parked: %r" % out)
        took, reason, words = out["v2"]
        self.assertIsNotNone(took)
        self.assertLessEqual(took, 2.0)
        self.assertEqual(reason, "free")                 # v2 judges the floor object's place empty
        self.assertIn("free", words)
        self.assertLess(took, out["v1"][0])              # v1 needs the misses alone

    def flat_sequence(self, cfg, seconds=180.0):
        """A phone stays in place; it is detected 2.5 s of every 4 s (P_D class 0.15 in the score), and v1
        geometry falsely says free on every 3rd frame while it is detected and on every frame while it is
        not. Returns (invalidations, first invalidation s, blocked by the filter)."""
        T = ex.pose()
        obj = {"p": phone(0.9, 0.)}
        rig = FilteredRig(cfg)
        rig.run(3.0, T, obj)
        (tid,) = rig.m.tracks
        start = rig.now
        flips, first, prev = 0, None, ex.state(rig.m, tid)
        for k in range(int(round(seconds / ex.DT))):
            t = k * ex.DT % 4.0
            seen = t < 2.5
            rig.force_free = (k % 3 == 0) if seen else True
            rig.step(T, obj, detect=None if seen else ())
            st = ex.state(rig.m, tid)
            if st == "position_invalid" and prev != "position_invalid":
                flips += 1
                if first is None:
                    first = rig.now - start
            prev = st
        self.assertEqual(list(rig.m.tracks), [tid])
        return flips, first, rig.filt.blocked(rig.m, tid)

    def test_v1_flat_object_false_free_filter(self):
        on = self.flat_sequence(core.Config())
        off = self.flat_sequence(replace(core.Config(), see_through_filter=False))
        print("\n[candidate] phone, v1 false free, 180 s: filter on (flips, first s, blocked) %r, off %r" % (on, off))
        self.assertEqual(on[0], 0)                       # never invalid in 3 minutes
        self.assertTrue(on[2])
        self.assertGreater(off[0], 0)                    # positive control: the same sequence fails without it
        self.assertFalse(off[2])

    def test_track_ev_log_and_replay(self):
        # frame_processed.track_ev keeps the detected track's raw word, and the log replay's +filter variant
        # uses it (the pre-candidate logs could not: their detected rows only say new_visual_observation)
        import json, tempfile
        from host.ros_runtime import track_ev_row
        from tools.memory_quality import existence_log_replay as rep
        T = ex.pose()
        obj = {"p": phone(0.9, 0.)}
        rig = FilteredRig(core.Config())
        rig.run(3.0, T, obj)
        (tid,) = rig.m.tracks
        last, rows = {}, []
        for k in range(int(round(40.0 / ex.DT))):
            seen = k * ex.DT % 4.0 < 2.5
            rig.force_free = (k % 3 == 0) if seen else True
            f_id = "f%05d" % rig.i
            raw_box = {}
            orig = rig.filt.apply

            def spy(geo, obs, m, _orig=orig):
                raw_box["raw"] = geo
                raw_box["used"] = _orig(geo, obs, m)
                return raw_box["used"]
            rig.filt.apply = spy
            rig.step(T, obj, detect=None if seen else ())
            rig.filt.apply = orig
            ev = track_ev_row(rig.m, rig.filt, raw_box["raw"], raw_box["used"], last)
            rows.append({"event": "frame_processed", "frame_id": f_id, "acquisition_timestamp": rig.now - ex.DT,
                         "detections": [], "tracks": rig.m.snapshot(), "track_ev": ev})
        self.assertTrue(any(r["track_ev"].get(tid, ["-"])[0] == "f" and r["tracks"][0]["last_seen_frame"] == r["frame_id"]
                            for r in rows))                   # a detected frame's raw free is in the log
        self.assertEqual(rows[-1]["track_ev"][tid][3], 1)     # blocked flag
        self.assertLess(len(json.dumps(rows[-1]["track_ev"])), 60)
        with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as fh:
            for r in rows:
                fh.write(json.dumps(r) + "\n")
        try:
            self.assertTrue(rep.has_track_ev(fh.name))
            plain = rep.summary(fh.name, "worst")
            filt = rep.summary(fh.name, "worst+filter")
        finally:
            os.unlink(fh.name)
        print("\n[candidate] track_ev replay 40 s: worst invalidated %d, worst+filter %d"
              % (plain["invalidated"], filt["invalidated"]))
        self.assertGreater(plain["invalidated"], 0)
        self.assertEqual(filt["invalidated"], 0)

    def test_existence_off_filter_off(self):
        # existence_v3=False and geometry_v2=False: the filter never changes a word (0f377be behaviour)
        filt = SeeThroughFilter(replace(core.Config(), existence_v3=False))
        self.assertFalse(filt.active())
        geo = {"x": "free"}
        self.assertIs(filt.apply(geo, [object()], None), geo)


if __name__ == "__main__":
    unittest.main()
