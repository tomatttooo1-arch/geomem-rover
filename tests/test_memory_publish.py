"""/fpga/memory publish (spec D.1) and /fpga/region_verdicts merge (spec D.2/D.3, OR s.6)."""

import copy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import unittest

import numpy as np

from host.memory_publish import MemoryFeed, canonical, digest, track_rows
from host.region_verdicts import RegionMerger, build_message
from perception.fpga_replay.core import Memory, Track

sys.path.insert(0, str(Path(__file__).resolve().parent))

BOOT, WORLD = "rt-boot", "map"


def track(tid, label="bottle", x=1.0, last_seen=1.0, state="historical_memory"):
    p = np.array([x, 0.0, 0.5])
    t = Track(tid, label, "red", p, p - 0.05, p + 0.05, last_seen, state=state)
    t.history.append({"frame_id": "f%d" % int(last_seen * 10), "timestamp": last_seen, "bbox": [1, 2, 3, 4],
                      "score": 0.9, "big": list(range(50))})
    return t


def memory_with(*tracks):
    m = Memory("geometry")
    for t in tracks:
        m.tracks[t.id] = t
    return m


def revision_rule_violations(messages):
    """Revision changes iff (digest, target_label) changes between consecutive publishes."""
    bad = []
    for a, b in zip(messages, messages[1:]):
        changed = (a["digest"], a["target_label"]) != (b["digest"], b["target_label"])
        step = b["revision"] - a["revision"]
        if step != (1 if changed else 0):
            bad.append((a["revision"], b["revision"], changed))
    return bad


class HeartbeatBumpsRevision(MemoryFeed):
    """Variant: a heartbeat also increments revision (forbidden by D.1)."""

    def step(self, *a, **kw):
        out = MemoryFeed.step(self, *a, **kw)
        if out is not None and self.counts["heartbeat"] > getattr(self, "_hb", 0):
            self._hb = self.counts["heartbeat"]
            self.revision += 1
            out = dict(out, revision=self.revision)
        return out


class MemoryFeedTests(unittest.TestCase):
    def run_timeline(self, feed_cls=MemoryFeed):
        m = memory_with(track("object-0001"), track("object-0002", x=2.0))
        feed = feed_cls(BOOT, WORLD, "simulation")
        out = []

        def step(t, label=None):
            msg = feed.step(m, t, 100.0 + t, target_label=label)
            if msg is not None:
                out.append((t, msg))
            return msg

        step(0.0)                                   # first publish, revision 0
        step(0.3)                                   # nothing changed, inside heartbeat -> None
        step(1.0)                                   # heartbeat, same revision
        m.tracks["object-0001"].state = "temporarily_occluded"
        step(1.2)                                   # change -> revision 1
        # 10-27 fast removal: a track turning position_invalid is now published at once (test_memory_fast_removal);
        # an ordinary change (here temporarily_occluded) keeps the 0.5 s merge window this timeline checks
        m.tracks["object-0002"].state = "temporarily_occluded"
        step(1.4)                                   # change held inside the 0.5 s window
        m.tracks["object-0002"].reason = "free"
        step(1.5)                                   # still held, merged
        step(1.7)                                   # window over -> ONE publish, revision 2
        step(2.7)                                   # heartbeat
        step(2.8, label="bottle")                   # target label is content -> revision 3
        return feed, out

    def test_revision_plus_one_exactly_on_change_heartbeat_keeps_it(self):
        feed, out = self.run_timeline()
        self.assertEqual([(t, msg["revision"]) for t, msg in out],
                         [(0.0, 0), (1.0, 0), (1.2, 1), (1.7, 2), (2.7, 2), (2.8, 3)])
        self.assertEqual(revision_rule_violations([msg for _, msg in out]), [])
        self.assertEqual((feed.counts["changed"], feed.counts["heartbeat"], feed.counts["held_in_window"]),
                         (4, 2, 2))

    def test_positive_control_heartbeat_bumping_revision_is_caught(self):
        _, out = self.run_timeline(HeartbeatBumpsRevision)
        self.assertTrue(revision_rule_violations([msg for _, msg in out]))

    def test_no_history_only_history_len_and_last(self):
        _, out = self.run_timeline()
        for _, msg in out:
            for row in msg["tracks"]:
                self.assertNotIn("history", row)
                self.assertEqual(row["history_len"], 1)
                self.assertEqual(set(row["history_last"]), {"frame_id", "timestamp", "bbox", "score"})
            json.dumps(msg, allow_nan=False)
        self.assertEqual(out[0][1]["runtime_boot_id"], BOOT)
        self.assertEqual(out[0][1]["world_frame"], WORLD)

    def test_digest_is_stable_and_is_the_canonical_hash(self):
        a = memory_with(track("object-0001"), track("object-0002", x=2.0))
        b = memory_with(track("object-0001"), track("object-0002", x=2.0))
        ra, rb = track_rows(a), track_rows(b)
        self.assertEqual(digest(ra), digest(rb))
        self.assertEqual(digest(ra), hashlib.sha256(canonical(ra)).hexdigest())
        m1 = MemoryFeed(BOOT, WORLD, "simulation").step(a, 0.0, 1.0)
        m2 = MemoryFeed("other-boot", WORLD, "simulation").step(b, 5.0, 9.0)
        self.assertEqual(m1["digest"], m2["digest"])                 # content only, not boot/stamp
        b.tracks["object-0002"].position = b.tracks["object-0002"].position + 0.001
        self.assertNotEqual(digest(track_rows(b)), digest(ra))


# ---------------------------------------------------------------------------- D.3 merge
def pose(x=0.0, yaw_deg=0.0):
    T = np.eye(4)
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    T[:2, :2] = [[c, -s], [s, c]]
    T[0, 3] = x
    return T


def message(stamp_s, verdicts, T=None, boot=BOOT, world=WORLD, failure=None, revision=1):
    return build_message(depth_stamp_ns=int(round(stamp_s * 1e9)), capture_nano_ns=1, kv_frame_id=7,
                         world_frame=world, T_world_camera=pose() if T is None else T, pose_valid=True,
                         runtime_boot_id=boot, memory_revision=revision,
                         verdicts=[{"track_id": tid, "geometry_evidence": w, "code": 3 if w == "occluded" else 0}
                                   for tid, w in verdicts], failure=failure)


def fingerprint(m):
    return json.dumps({"tracks": [dict(t.as_dict(), history=len(t.history),
                                       neg=[(s, p.tolist()) for s, p in t.negative_poses])
                                  for t in m.tracks.values()],
                       "last_geo": m.last_geometry_timestamp, "seen": sorted(m.geometry_seen_frames)},
                      sort_keys=True, default=float)


class RegionMergeTransitionTests(unittest.TestCase):
    def setUp(self):
        self.m = memory_with(track("object-0001"), track("object-0002", x=2.0, state="current_confirmed"))
        self.r = RegionMerger(BOOT, WORLD, max_age_s=1.0)

    def apply(self, stamp, verdicts, now=None, **kw):
        return self.r.apply(self.m, json.dumps(message(stamp, verdicts, **kw)), stamp + 0.1 if now is None else now)

    def state(self, tid):
        return self.m.tracks[tid].state

    def test_occluded_becomes_temporarily_occluded(self):
        ev = self.apply(10.0, [("object-0001", "occluded")])
        self.assertEqual((ev["event"], ev["applied"]), ("region_applied", 1))
        self.assertEqual(self.state("object-0001"), "temporarily_occluded")

    def test_two_independent_free_views_make_position_invalid(self):
        self.m.cfg = replace(self.m.cfg, existence_v3=False)   # 10-27: legacy two-view rule (existence_v3=False); v3 counterpart in tests/test_memory_existence_20261027.py
        self.apply(10.0, [("object-0001", "free")], T=pose(0.0))
        self.assertEqual(self.state("object-0001"), "historical_memory")
        self.apply(10.5, [("object-0001", "free")], T=pose(0.2))      # 0.5 s and 0.2 m apart
        self.assertEqual(self.state("object-0001"), "position_invalid")

    def test_two_free_views_from_the_same_pose_are_not_independent(self):
        self.m.cfg = replace(self.m.cfg, existence_v3=False)   # 10-27: legacy two-view rule (existence_v3=False); v3 counterpart in tests/test_memory_existence_20261027.py
        self.apply(10.0, [("object-0001", "free")], T=pose(0.0))
        self.apply(10.5, [("object-0001", "free")], T=pose(0.01, 1.0))
        self.assertEqual(self.state("object-0001"), "historical_memory")
        self.assertEqual(len(self.m.tracks["object-0001"].negative_poses), 1)

    def test_current_confirmed_holds_under_one_free_and_surface(self):
        self.apply(10.0, [("object-0002", "free")])
        self.assertEqual(self.state("object-0002"), "current_confirmed")
        self.apply(10.5, [("object-0002", "surface_or_uncertain")], T=pose(0.3))
        self.assertEqual(self.state("object-0002"), "current_confirmed")
        self.assertEqual(self.m.tracks["object-0002"].negative_poses, [])

    def test_six_rejections_leave_memory_untouched(self):
        self.apply(10.0, [("object-0001", "free")])                   # last_geometry_timestamp = 10.0
        before = fingerprint(self.m)
        cases = {
            "malformed": lambda: self.r.apply(self.m, "{not json", 11.0),
            "foreign_runtime": lambda: self.apply(10.6, [("object-0001", "occluded")], boot="other"),
            "world_frame_mismatch": lambda: self.apply(10.6, [("object-0001", "occluded")], world="odom"),
            "region_future": lambda: self.apply(10.6, [("object-0001", "occluded")], now=10.5),
            "region_stale": lambda: self.apply(10.6, [("object-0001", "occluded")], now=11.7),
            "region_out_of_order": lambda: self.apply(9.9, [("object-0001", "occluded")]),
        }
        for reason, call in cases.items():
            ev = call()
            self.assertEqual((ev["event"], ev["reason"], ev["applied"]), ("region_rejected", reason, 0), reason)
            self.assertEqual(fingerprint(self.m), before, reason)
            self.assertEqual(self.r.counts["region_rejected:" + reason], 1)
        ev = self.r.apply(self.m, json.dumps(message(10.6, [], failure="region_backend_failure:no_roi_block")), 10.7)
        self.assertEqual((ev["event"], ev["reason"]), ("region_failure", "region_backend_failure:no_roi_block"))
        self.assertEqual(fingerprint(self.m), before)

    def test_positive_control_fingerprint_sees_an_applied_verdict(self):
        before = fingerprint(self.m)
        self.apply(10.0, [("object-0001", "occluded")])
        self.assertNotEqual(fingerprint(self.m), before)

    def test_unknown_track_and_newer_visual_observation_are_skipped(self):
        self.m.tracks["object-0002"].last_seen = 20.0
        ev = self.apply(10.0, [("object-9999", "occluded"), ("object-0002", "occluded"),
                               ("object-0001", "occluded")])
        self.assertEqual(ev["applied"], 1)
        self.assertEqual(ev["skipped"], {"unknown_track": 1, "newer_visual_observation": 1})
        self.assertEqual(self.state("object-0002"), "current_confirmed")
        self.assertEqual(self.r.last_region["object-0002"]["skip_reason"], "newer_visual_observation")

    def test_bad_pose_does_not_crash(self):
        msg = message(10.0, [("object-0001", "free")])
        msg["T_world_camera"] = [float("nan")] * 16
        ev = self.r.apply(self.m, msg, 10.1)
        self.assertIn(ev["event"], ("region_rejected", "region_applied"))


class KvOnlyRuntimeTests(unittest.TestCase):
    """--region-source kv_only: the runtime's own geometry is replaced by {} (D.3)."""

    def test_kv_only_equals_geometry_dropped_and_differs_from_none(self):
        import test_region_source_none_regression as nr
        if nr.baseline_source() is None:
            self.skipTest("git history not available")
        src = (nr.ROOT / "host" / "ros_runtime.py").read_text(encoding="utf-8")
        none, ok_none = nr.capture(src, "rt_none")
        kv_only, ok_kv = nr.capture(src, "rt_kv_only", {"region_source": "kv_only"})
        dropped, _ = nr.capture(src.replace(nr.GEOMETRY_CALL, "{}"), "rt_dropped")
        # Without KV verdicts the runtime has no geometry: exactly the scenarios whose own checks
        # need free/occluded evidence fail; all others pass.
        self.assertTrue(all(ok for ok, _ in ok_none.values()), ok_none)
        self.assertEqual(sorted(k for k, (ok, _) in ok_kv.items() if not ok),
                         ["empty_old_position_new_id_selection", "occlusion_changes_view"])

        def strip(streams):          # region_source is echoed in the startup/record rows
            out = {}
            for k, rows in streams.items():
                out[k] = [dict(r, payload={kk: vv for kk, vv in r["payload"].items() if kk != "region_source"})
                          if isinstance(r.get("payload"), dict) else r for r in rows
                          if not (r.get("event") == "subscribe" and r.get("topic") == "/fpga/region_verdicts")]
            return out
        self.assertEqual(nr.differences(strip(dropped), strip(kv_only)), [])
        self.assertIn("scenario_occlusion_changes_view", [d[0] for d in nr.differences(none, kv_only)])


if __name__ == "__main__":
    unittest.main()
