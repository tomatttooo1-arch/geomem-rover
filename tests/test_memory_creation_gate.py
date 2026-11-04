"""D-044 revised (2026-10-21) memory creation gate (core.Config.create_* / candidate_ttl_s).

An observation that matches no track becomes a candidate first; candidates are not tracks, so they
are not in the snapshot, the memory feed (kiosk / UI), the PL ROI plan or the memory store. A
candidate is promoted after create_min_hits sightings in distinct frames spanning >=
create_min_span_s seconds and dropped when unseen for candidate_ttl_s. Updates of existing tracks
are not gated. Defaults (S, N, T, TTL) = (0, 3, 2 s, 3 s), runs/coco-open-20261021-results README.
"""
from dataclasses import replace
from types import SimpleNamespace
import unittest

import numpy as np

from host import geometry_region_glue as glue
from host import memory_store
from host.memory_publish import MemoryFeed
from host.ros_runtime import memory_config
from perception.fpga_replay.core import UNGATED_CREATION, Config, Frame, Memory, Observation

K = np.array([[500., 0, 320], [0, 500, 240], [0, 0, 1]])


class Driver:
    """Hand-made frames at chosen times (seconds) with world-frame observations."""

    def __init__(self, cfg=Config()):
        self.memory, self.i = Memory("geometry", cfg), 0

    def frame(self, t):
        self.i += 1
        return Frame("g-%04d" % self.i, float(t), np.zeros((2, 2, 3), np.uint8),
                     np.full((2, 2), np.nan, np.float32), K, np.eye(4), True, "test", "test")

    @staticmethod
    def obs(f, label, p, score=0.9):
        p = np.array(p, dtype=float)
        return Observation(label, score, p, p - .03, p + .03, "red", 1.0, f.frame_id, f.timestamp,
                           (100, 100, 130, 200), "test", extent_m=(0.06, 0.2))

    def step(self, t, specs=()):
        """specs: (label, position[, score]); returns update_memory's result."""
        f = self.frame(t)
        return self.memory.update_memory([self.obs(f, *s) for s in specs], {}, f)

    def labels(self):
        return sorted(t.label for t in self.memory.tracks.values())


BOTTLE = ("bottle", (1.0, 0.0, 0.1))


class DefaultsTests(unittest.TestCase):
    def test_config_defaults_are_the_selected_gate(self):
        c = Config()
        self.assertEqual((c.create_min_score, c.create_min_hits, c.create_min_span_s, c.candidate_ttl_s),
                         (0.0, 3, 2.0, 3.0))

    def test_runner_memory_config_gate_on_by_default(self):
        self.assertEqual(memory_config(SimpleNamespace()), Config())
        self.assertEqual(memory_config(SimpleNamespace(memory_creation_gate="on")), Config())
        off = memory_config(SimpleNamespace(memory_creation_gate="off"))
        self.assertEqual(off, replace(Config(), **UNGATED_CREATION))
        self.assertFalse(Memory("geometry", off)._create_gated())
        self.assertTrue(Memory("geometry", Config())._create_gated())

    def test_runner_cli_default_on(self):
        import host.ros_runtime as rr
        with open(rr.__file__, encoding="utf8") as stream:
            src = stream.read()
        self.assertIn('"--memory-creation-gate", choices=("on", "off"), default="on"', src)
        self.assertIn("Memory(cfg=memory_config(args))", src)


class CandidateInvisibleTests(unittest.TestCase):
    def test_candidate_not_in_snapshot_feed_plan_or_store(self):
        d = Driver()
        r1 = d.step(0.0, [BOTTLE])
        r2 = d.step(1.0, [BOTTLE])
        m = d.memory
        self.assertEqual(m.tracks, {})
        self.assertEqual(r1["tracks"], [])
        self.assertEqual(r2["tracks"], [])
        self.assertEqual([c["hits"] for c in r1["candidates"] + r2["candidates"]], [1, 2])
        self.assertEqual(len(m.candidates), 1)
        self.assertEqual(m.snapshot(), [])
        self.assertEqual(m.query_memory("找瓶子"), [])
        msg = MemoryFeed("boot", "map", "test").step(m, 10.0, 1.0)
        self.assertEqual(msg["tracks"], [])                       # kiosk / operator UI input
        plan = glue.plan_frame(msg, (480, 640), K, np.eye(4), Config())
        self.assertIsNone(plan.batch)                             # nothing sent to the PL
        doc = memory_store.dump(m, complete=True, runtime_boot_id="b", world_frame="map", pose_source="estimated",
                                sensor_source="test", map_identity_doc=None)
        self.assertEqual(doc["tracks"], [])                       # nothing persisted

    def test_promoted_track_reaches_feed_and_plan(self):
        d = Driver()
        for t in (0.0, 1.0, 2.0):
            d.step(t, [BOTTLE])
        msg = MemoryFeed("boot", "map", "test").step(d.memory, 10.0, 2.0)
        self.assertEqual([t["label"] for t in msg["tracks"]], ["bottle"])
        T = np.eye(4)
        T[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]           # optical z along world +x, track in view
        plan = glue.plan_frame(msg, (480, 640), K, T, Config())
        self.assertEqual(len(plan.batch.rows), 1)


class PromotionTests(unittest.TestCase):
    def test_needs_three_frames_spanning_two_seconds(self):
        d = Driver()
        d.step(0.0, [BOTTLE]); d.step(1.0, [BOTTLE])
        self.assertEqual(d.labels(), [])
        d.step(2.0, [BOTTLE])
        self.assertEqual(d.labels(), ["bottle"])
        t = next(iter(d.memory.tracks.values()))
        self.assertEqual(t.created_at, 2.0)
        self.assertEqual(t.state, "current_confirmed")
        self.assertEqual(d.memory.candidates, {})
        self.assertEqual(d.memory.candidate_stats, {"started": 1, "promoted": 1, "expired": 0})

    def test_three_frames_within_two_seconds_wait_for_span(self):
        d = Driver()
        for t in (0.0, 0.1, 0.2, 0.3, 1.9):                         # board rate: hits come fast
            d.step(t, [BOTTLE])
        self.assertEqual(d.labels(), [])
        d.step(2.0, [BOTTLE])
        self.assertEqual(d.labels(), ["bottle"])
        self.assertEqual(next(iter(d.memory.tracks.values())).created_at, 2.0)

    def test_span_without_enough_hits_waits(self):
        d = Driver(replace(Config(), candidate_ttl_s=10.0))
        d.step(0.0, [BOTTLE]); d.step(5.0, [BOTTLE])
        self.assertEqual(d.labels(), [])
        d.step(6.0, [BOTTLE])
        self.assertEqual(d.labels(), ["bottle"])

    def test_history_keeps_every_candidate_sighting(self):
        d = Driver()
        for t in (0.0, 1.0, 2.0):
            d.step(t, [BOTTLE])
        h = next(iter(d.memory.tracks.values())).history
        self.assertEqual([row["timestamp"] for row in h], [0.0, 1.0, 2.0])
        self.assertEqual(len({row["frame_id"] for row in h}), 3)

    def test_same_frame_second_sighting_adds_no_hit_and_no_history(self):
        # Two same-label observations 0.20 m apart: kept apart by the same-frame dedupe, both within
        # the candidate association radius. Only one hit per frame per candidate.
        d = Driver()
        near = ("bottle", (1.2, 0.0, 0.1))
        r = d.step(0.0, [BOTTLE, near])
        self.assertEqual(len(d.memory.candidates), 1, r)
        self.assertEqual(len(next(iter(d.memory.candidates.values())).history), 1)
        d.step(1.0, [BOTTLE, near])
        self.assertEqual(len(next(iter(d.memory.candidates.values())).history), 2)
        d.step(2.0, [BOTTLE, near])
        bottles = [t for t in d.memory.tracks.values()]
        self.assertEqual(len(bottles), 1)
        h = bottles[0].history
        self.assertEqual([row["timestamp"] for row in h], [0.0, 1.0, 2.0])
        self.assertEqual([row["position_m"][0] for row in h], [1.0, 1.0, 1.0])

    def test_labels_do_not_share_a_candidate(self):
        d = Driver()
        d.step(0.0, [BOTTLE]); d.step(1.0, [("cup", BOTTLE[1])]); d.step(2.0, [BOTTLE])
        self.assertEqual(d.labels(), [])
        self.assertEqual(sorted(c.label for c in d.memory.candidates.values()), ["bottle", "cup"])


class ExpiryTests(unittest.TestCase):
    def test_candidate_unseen_for_ttl_dropped_and_restarts(self):
        d = Driver()
        d.step(0.0, [BOTTLE]); d.step(1.0, [BOTTLE])
        d.step(4.5)                                                 # 3.5 s unseen > TTL 3 s
        self.assertEqual(d.memory.candidates, {})
        self.assertEqual(d.memory.candidate_stats["expired"], 1)
        d.step(5.0, [BOTTLE])
        self.assertEqual(d.labels(), [])
        self.assertEqual(len(next(iter(d.memory.candidates.values())).history), 1)

    def test_gap_within_ttl_keeps_candidate(self):
        d = Driver()
        d.step(0.0, [BOTTLE]); d.step(1.0, [BOTTLE]); d.step(3.9); d.step(4.0, [BOTTLE])
        self.assertEqual(d.labels(), ["bottle"])


class ScoreTests(unittest.TestCase):
    def test_score_below_create_rejected(self):
        d = Driver(replace(Config(), create_min_score=0.5))
        r = d.step(0.0, [("bottle", BOTTLE[1], 0.4)])
        self.assertEqual([x["reason"] for x in r["rejected"]], ["score_below_create"])
        self.assertEqual(d.memory.candidates, {})
        self.assertEqual(d.memory.rejected, {"score_below_create": 1})


class ExistingTrackTests(unittest.TestCase):
    def test_existing_track_updates_immediately_and_low_score_ok(self):
        d = Driver(replace(Config(), create_min_score=0.5))
        for t in (0.0, 1.0, 2.0):
            d.step(t, [BOTTLE])
        track = next(iter(d.memory.tracks.values()))
        r = d.step(2.5, [("bottle", (1.1, 0.0, 0.1), 0.3)])       # one sighting, score under S
        self.assertEqual(r["rejected"], [])
        self.assertEqual(r["candidates"], [])
        self.assertEqual(track.last_seen, 2.5)
        # 10-27 identity_v2: the sighting is the track's newest observation; its position is the fused one
        self.assertAlmostEqual(float(track.history[-1]["position_m"][0]), 1.1)
        self.assertTrue(1.0 <= float(track.position[0]) <= 1.1)
        self.assertEqual(len(track.history), 4)

    def test_loaded_track_updates_without_gate(self):
        src = Driver(Config(**UNGATED_CREATION))
        src.step(0.0, [BOTTLE])
        doc = memory_store.dump(src.memory, complete=True, runtime_boot_id="b", world_frame="map",
                                pose_source="estimated", sensor_source="test", map_identity_doc=None)
        d = Driver()
        memory_store.restore(doc, d.memory)
        d.step(10.0, [BOTTLE])
        track = next(iter(d.memory.tracks.values()))
        self.assertEqual(track.last_seen, 10.0)
        self.assertEqual(d.memory.candidates, {})


class BypassTests(unittest.TestCase):
    def test_ungated_and_memory_quality_off_create_on_first_sighting(self):
        for cfg in (Config(**UNGATED_CREATION), replace(Config(), memory_quality=False)):
            d = Driver(cfg)
            r = d.step(0.0, [BOTTLE])
            self.assertEqual(d.labels(), ["bottle"], cfg)
            self.assertNotIn("candidates", r)
            self.assertEqual(d.memory.candidates, {})


class StoreTests(unittest.TestCase):
    def dump(self, memory):
        return memory_store.dump(memory, complete=True, runtime_boot_id="b", world_frame="map",
                                 pose_source="estimated", sensor_source="test", map_identity_doc=None)

    def test_dump_has_gate_keys_and_query_classes(self):
        cfg = self.dump(Memory("geometry", Config()))["memory"]["cfg"]
        for k in ("create_min_score", "create_min_hits", "create_min_span_s", "candidate_ttl_s"):
            self.assertIn(k, cfg)
        self.assertEqual(cfg["query_classes"], list(Config().query_classes))

    def test_pre_gate_file_loads_without_cfg_differs(self):
        doc = self.dump(Memory("geometry", Config()))
        for k in ("create_min_score", "create_min_hits", "create_min_span_s", "candidate_ttl_s", "query_classes"):
            del doc["memory"]["cfg"][k]
        info = memory_store.restore(doc, Memory("geometry", Config()))
        self.assertEqual(info["cfg_differs"], [])

    def test_changed_gate_reported_in_cfg_differs(self):
        doc = self.dump(Memory("geometry", Config()))
        info = memory_store.restore(doc, Memory("geometry", Config(**UNGATED_CREATION)))
        self.assertEqual(info["cfg_differs"], ["create_min_hits", "create_min_span_s"])


if __name__ == "__main__":
    unittest.main()
