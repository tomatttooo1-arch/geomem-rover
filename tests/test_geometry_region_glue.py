"""D.2 glue with package 1's real plan_boxes / memory_priority / check_roi_answer / CpuRoiCore.

Same-input consistency: for one depth frame and one /fpga/memory snapshot, the words the
glue publishes from the KV CPU core's answer equal NativeObjectEvidence.check_boxes and the
pure-Python compute_geometry on the same input, for every track (host-resolved and kernel).
"""

import collections
import json
import shutil
import tempfile
from pathlib import Path
import unittest

import numpy as np

from host import geometry_region_glue as glue
from host.kv_link import CpuRoiCore, RoiOutcome, check_roi_answer
from host.m7_object_native import NativeObjectEvidence, compile_object_native
from host.region_verdicts import RegionMerger, validate_message
from perception.fpga_replay.core import Config, Frame, Memory, Track, compute_geometry

BOOT, WORLD = "rt-boot", "map"
H, W = 48, 64
K = np.array([[50., 0., 32.], [0., 50., 24.], [0., 0., 1.]])


def scene():
    """Wall at 3 m; a near plate in front of the 'occluded' box; NaNs over 'nodepth'."""
    depth = np.full((H, W), 3.0, np.float32)
    depth[14:34, 44:60] = 1.0                      # occluder for box at x=+0.6..., z=2.5
    depth[2:12, 2:16] = np.nan                     # invalid depth patch (upper-left)
    tracks = {
        "surface": ((0.0, 0.0, 3.0), 0.1, "cup", 5.0),
        "occluded": ((0.6, 0.0, 2.5), 0.1, "bottle", 6.0),
        "free": ((-0.6, 0.0, 1.5), 0.1, "bottle", 7.0),
        "behind": ((0.0, 0.0, -2.0), 0.1, "cup", 8.0),
        "tiny": ((0.0, 0.3, 6.0), 0.005, "cup", 9.0),
        "nodepth": ((-1.15, -0.65, 2.5), 0.08, "cup", 4.0),
    }
    return depth, tracks


def memory_msg(tracks, revision=7, target=None, world=WORLD):
    rows = []
    for tid, (c, half, label, seen) in tracks.items():
        c = np.array(c)
        rows.append({"id": tid, "label": label, "position_m": c.tolist(),
                     "bounds_m": [(c - half).tolist(), (c + half).tolist()], "last_seen": seen,
                     "state": "historical_memory"})
    return {"v": 1, "runtime_boot_id": BOOT, "revision": revision, "world_frame": world,
            "target_label": target, "tracks": rows}


def kv_answer(core, depth, batch):
    """What kv_server --roi-core cpu returns for this batch (wire rows), as check_roi_answer reads it."""
    rows = []
    for r in batch.rows:
        rows.append((r.row_id, 1) + tuple(r.xyxy) + (r.near_z, r.far_z) if r.enabled and r.xyxy is not None
                    else (r.row_id, 0, 0, 0, 0, 0, 0., 0.))
    cfg = batch.cfg
    out = core(depth, {"memory_revision": batch.memory_revision, "rows": rows,
                       "cfg": {"depth_margin_m": cfg.depth_margin_m, "min_pixels": cfg.min_pixels,
                               "min_valid_fraction": cfg.min_valid_fraction, "free_fraction": cfg.free_fraction}})
    return {"memory_revision": batch.memory_revision, "roi_status": out["status"], "rows": out["rows"],
            "roi_core": "cpu", "times_ns": {"prep": 1, "device": 2, "readback": 3, "total": 6}, "roi_cycles": None}


@unittest.skipIf(shutil.which("g++") is None, "g++ needed for the object-evidence reference")
class SameInputTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        lib = compile_object_native(Path(cls.tmp) / "object.so")
        cls.native = NativeObjectEvidence(lib)
        cls.core = CpuRoiCore(lib)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def glue_words(self, depth, msg, cfg=Config(), word=None):
        plan = glue.plan_frame(msg, depth.shape, K, np.eye(4), cfg)
        outcome = None                          # every track resolved on the host: nothing sent
        if plan.batch is not None:
            outcome = check_roi_answer(plan.batch, kv_answer(self.core, depth, plan.batch))
            self.assertTrue(outcome.ok, outcome.reason)
        if word is not None:
            orig = glue.evidence_word
            glue.evidence_word = word
        try:
            out = glue.verdict_message(plan, outcome, memory_msg=msg, depth_stamp_ns=10 ** 10,
                                       capture_nano_ns=5, kv_frame_id=3, world_frame=WORLD,
                                       T_world_camera=np.eye(4), pose_valid=True, cfg=cfg)
        finally:
            if word is not None:
                glue.evidence_word = orig
        return plan, out, {v["track_id"]: v["geometry_evidence"] for v in out["verdicts"]}

    def references(self, depth, msg, cfg=Config()):
        entries = [(t["id"], t["bounds_m"]) for t in msg["tracks"]]
        native = {e[0]: r[1] for e, r in zip(entries, self.native.check_boxes(depth, K, np.eye(4), entries, cfg))}
        mem = Memory("geometry")
        for t in msg["tracks"]:
            lo, hi = np.array(t["bounds_m"][0]), np.array(t["bounds_m"][1])
            mem.tracks[t["id"]] = Track(t["id"], t["label"], "red", (lo + hi) / 2, lo, hi, t["last_seen"])
        frame = Frame("f", 10.0, np.zeros((H, W, 3), np.uint8), depth.copy(), K, np.eye(4), True)
        python = compute_geometry(frame, mem, cfg)
        return native, python

    def test_glue_words_equal_check_boxes_and_compute_geometry(self):
        depth, tracks = scene()
        msg = memory_msg(tracks)
        plan, out, words = self.glue_words(depth, msg)
        native, python = self.references(depth, msg)
        self.assertEqual(words, native)
        self.assertEqual(words, python)
        # the scene exercises every word the core vocabulary has, host-resolved and kernel
        self.assertEqual(sorted(set(words.values())),
                         sorted(["surface_or_uncertain", "occluded", "free", "out_of_view", "too_small",
                                 "invalid_depth"]))
        self.assertEqual(plan.batch.memory_revision, 7)
        self.assertEqual(out["memory_revision"], 7)
        validate_message(out)
        host = {v["track_id"] for v in out["verdicts"] if v["code"] is None}
        self.assertEqual(host, {"behind", "tiny"})

    def test_random_scenes_same_input_consistent(self):
        rng = np.random.default_rng(20261011)
        n = 0
        for trial in range(25):
            depth = rng.uniform(.5, 4., size=(H, W)).astype(np.float32)
            depth[rng.random((H, W)) < .1] = np.nan
            tracks = {}
            for i in range(int(rng.integers(3, 40))):
                c = (float(rng.uniform(-1.5, 1.5)), float(rng.uniform(-1., 1.)), float(rng.uniform(-1., 5.)))
                tracks["t%02d" % i] = (c, float(rng.uniform(.01, .3)), "cup", float(rng.uniform(0, 9)))
            msg = memory_msg(tracks)
            plan, out, words = self.glue_words(depth, msg)
            native, python = self.references(depth, msg)
            got = dict(words)
            for u in out["unchecked"]:
                self.assertEqual(u["reason"], "overflow")
                got.pop(u["track_id"], None)
                native.pop(u["track_id"], None)
                python.pop(u["track_id"], None)
            self.assertEqual(got, native, trial)
            self.assertEqual(got, python, trial)
            n += len(got)
        self.assertGreater(n, 200)

    def test_positive_control_word_map_without_invalid_depth_branch_disagrees(self):
        depth, tracks = scene()
        msg = memory_msg(tracks)

        def careless(row, cfg):
            return {3: "occluded", 2: "free"}.get(row["verdict_code"], "surface_or_uncertain")
        _, _, words = self.glue_words(depth, msg, word=careless)
        native, _ = self.references(depth, msg)
        self.assertNotEqual(words, native)
        self.assertEqual(words["nodepth"], "surface_or_uncertain")

    def test_message_feeds_the_runtime_merge(self):
        depth, tracks = scene()
        msg = memory_msg(tracks)
        _, out, _ = self.glue_words(depth, msg)
        mem = Memory("geometry", Config(existence_v3=False))   # 10-27: legacy negative_views count checked below
        for t in msg["tracks"]:
            lo, hi = np.array(t["bounds_m"][0]), np.array(t["bounds_m"][1])
            mem.tracks[t["id"]] = Track(t["id"], t["label"], "red", (lo + hi) / 2, lo, hi, 1.0,
                                        state="historical_memory")
        ev = RegionMerger(BOOT, WORLD).apply(mem, json.dumps(out), 10.1)
        self.assertEqual((ev["event"], ev["applied"]), ("region_applied", 6))
        self.assertEqual(mem.tracks["occluded"].state, "temporarily_occluded")
        self.assertEqual(len(mem.tracks["free"].negative_poses), 1)
        # 10-27 existence_v3 (default): the same message opens a free vote in the track's window
        v3 = Memory("geometry")
        for t in msg["tracks"]:
            lo, hi = np.array(t["bounds_m"][0]), np.array(t["bounds_m"][1])
            v3.tracks[t["id"]] = Track(t["id"], t["label"], "red", (lo + hi) / 2, lo, hi, 1.0,
                                       state="historical_memory")
        RegionMerger(BOOT, WORLD).apply(v3, json.dumps(out), 10.1)
        self.assertEqual((v3.tracks["free"].ex_free, v3.tracks["occluded"].state), (True, "temporarily_occluded"))


class PlanAndWatchTests(unittest.TestCase):
    def test_priority_target_label_then_newest_and_overflow_listed(self):
        tracks = {}
        for i in range(40):
            tracks["t%02d" % i] = ((-0.5 + 0.025 * i, 0.0, 2.0), 0.05, "bottle" if i % 4 == 0 else "cup", float(i))
        msg = memory_msg(tracks, target="bottle")
        plan = glue.plan_frame(msg, (H, W), K, np.eye(4), Config())
        self.assertEqual(len(plan.batch.rows), 32)
        order = [plan.row_map[r.row_id] for r in plan.batch.rows]
        bottles = ["t%02d" % i for i in range(39, -1, -1) if i % 4 == 0]
        self.assertEqual(order[:10], bottles)                          # target class first, newest first
        self.assertEqual(order[10:12], ["t39", "t38"])
        self.assertEqual(len(plan.overflow), 8)
        out = glue.verdict_message(plan, RoiOutcome(False, "region_backend_failure:no_roi_block", [], "", {}, None),
                                   memory_msg=msg, depth_stamp_ns=1, capture_nano_ns=1, kv_frame_id=1,
                                   world_frame=WORLD, T_world_camera=np.eye(4), pose_valid=True, cfg=Config())
        self.assertEqual((out["failure"], out["verdicts"], out["unchecked"]),
                         ("region_backend_failure:no_roi_block", [], []))

    def test_memory_watch_skips_and_counts(self):
        w = glue.MemoryWatch(WORLD)
        self.assertEqual(w.usable(0.0), (None, "no_memory"))
        self.assertTrue(w.on_memory(json.dumps(memory_msg({}, world="odom")), 1.0))
        self.assertEqual(w.usable(1.0), (None, "world_frame_mismatch"))
        self.assertTrue(w.on_memory(json.dumps(memory_msg({}, revision=8)), 2.0))
        self.assertEqual(w.usable(6.9)[0]["revision"], 8)
        self.assertEqual(w.usable(7.1), (None, "memory_heartbeat_lost"))
        self.assertFalse(w.on_memory("{oops", 7.2))
        self.assertEqual(w.usable(7.3)[1], "memory_heartbeat_lost")    # the bad message replaced nothing
        counts = collections.Counter()
        self.assertIsNone(glue.plan_frame(memory_msg({}), (H, W), K, None, Config(), counts=counts))
        self.assertEqual(dict(w.counts), {"roi_skipped:no_memory": 1, "roi_skipped:world_frame_mismatch": 1,
                                          "memory_received": 2, "roi_skipped:memory_heartbeat_lost": 2,
                                          "memory_malformed": 1})
        self.assertEqual(dict(counts), {"roi_skipped:no_world_pose": 1})

    def test_frame_stats_known_answer(self):
        st = glue.frame_stats(kv_frame_id=4, capture_nano_ns=10 ** 9, done_nano_ns=10 ** 9 + 700 * 10 ** 6,
                              result="fused", mailbox_wait_ms=4.0,
                              client_last={"rtt_s": 0.65, "kv_ns": {"core_device": 604 * 10 ** 6, "recv": 10 ** 6}},
                              client_counts={"answered": 3})
        self.assertEqual((st["frame_age_ms"], st["channel_ms"], st["rtt_ms"]), (700.0, 92.0, 650.0))
        self.assertEqual(st["kv_times_ms"], {"core_device": 604.0, "recv": 1.0})
        json.dumps(st, allow_nan=False)


if __name__ == "__main__":
    unittest.main()
