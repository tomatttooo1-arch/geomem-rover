import copy
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np
from dataclasses import replace
from perception.fpga_replay.core import (Config as _Config, Memory as _Memory, UNGATED_CREATION, backproject,
                                        compute_geometry, process_frame)
from perception.fpga_replay.io import load_frame, sha256, quaternion_matrix, interpolate_pose
from perception.fpga_replay.synthetic import scene, scenarios


def Memory(strategy="geometry", cfg=None):
    """core.Memory with the D-044 creation gate off (core.UNGATED_CREATION). The expectations in this file
    were written for single-sighting creation; the gate has its own tests (test_memory_creation_gate.py)."""
    return _Memory(strategy, replace(_Config() if cfg is None else cfg, **UNGATED_CREATION))


def step(memory, pair):
    frame, boxes = pair
    return memory.update_memory(process_frame(frame, boxes), compute_geometry(frame, memory), frame)


class ReplayTests(unittest.TestCase):
    def seeded(self, strategy="geometry", legacy=False):
        m = Memory(strategy, _Config(existence_v3=False) if legacy else None); step(m, scene(0)); return m

    def test_backproject_known_identity_and_rotated_camera(self):
        K = np.array([[100., 0, 50], [0, 100, 50], [0, 0, 1]])
        uv, z = np.array([[50, 50], [100, 50]]), np.array([2., 2.])
        np.testing.assert_allclose(backproject(uv, z, K, np.eye(4)), [[0, 0, 2], [1, 0, 2]])
        T = np.eye(4); T[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]; T[:3, 3] = [1, 2, 3]
        np.testing.assert_allclose(backproject(uv, z, K, T), [[1, 2, 5], [1, 3, 5]])

    def test_depth_position_and_invalid_center_pixel(self):
        f, d = scene(0); f.depth[60, 80] = np.nan
        o = process_frame(f, d)
        self.assertEqual(len(o), 1)
        np.testing.assert_allclose(o[0].position, [0, 0, 2], atol=.02)
        self.assertEqual(o[0].color, "red")

    def test_out_of_view_occlusion_and_miss_have_different_evidence(self):
        for kind, expected in [("out_of_view", "out_of_view"), ("occluded", "occluded"),
                               ("present", "surface_or_uncertain"), ("removed", "free")]:
            m = self.seeded(); f, _ = scene(1, kind)
            self.assertEqual(compute_geometry(f, m)["object-0001"], expected)

    def test_scenarios_against_expected_truth(self):
        expected = {"out_of_view": "historical_memory", "occluded": "temporarily_occluded",
                    "removed": "position_invalid", "invalid_depth": "historical_memory",
                    "invalid_pose": "historical_memory", "detector_miss": "historical_memory",
                    "repeat_view": "historical_memory", "reconfirm": "current_confirmed"}
        for name, frames in scenarios():
            with self.subTest(name=name):
                m = Memory(cfg=_Config(existence_v3=False))   # 10-27: legacy two-view rule (existence_v3=False); v3 counterpart in tests/test_memory_existence_20261027.py
                for frame in frames: step(m, frame)
                self.assertEqual(m.snapshot()[0]["state"], expected[name])

    def test_baselines_expose_opposing_failure_modes(self):
        static, miss, geometric = self.seeded("static"), self.seeded("miss"), self.seeded()
        for i in (1, 2):
            step(static, scene(i, "removed", camera_x=.12*(i-1)))
            step(miss, scene(i, "occluded", camera_x=.12*(i-1)))
            step(geometric, scene(i, "occluded", camera_x=.12*(i-1)))
        self.assertEqual(static.snapshot()[0]["state"], "historical_memory")
        self.assertEqual(miss.snapshot()[0]["state"], "position_invalid")
        self.assertEqual(geometric.snapshot()[0]["state"], "temporarily_occluded")

    def test_duplicate_and_stationary_frames_do_not_accumulate(self):
        m = self.seeded(legacy=True); pair = scene(1, "removed")   # 10-27: legacy two-view rule (existence_v3=False); v3 counterpart in tests/test_memory_existence_20261027.py
        step(m, pair); before = m.snapshot()
        self.assertEqual(step(m, pair)["ignored"], "duplicate_frame")
        self.assertEqual(m.snapshot(), before)
        step(m, scene(2, "removed"))
        self.assertEqual(m.snapshot()[0]["negative_views"], 1)

    def test_invalid_pose_cannot_create_or_invalidate(self):
        m = self.seeded(); f, d = scene(1); f.pose_valid = False
        self.assertEqual(process_frame(f, d), [])
        step(m, (f, d))
        self.assertEqual(m.snapshot()[0]["negative_views"], 0)

    def test_surface_resets_previous_free_evidence(self):
        m = self.seeded(); step(m, scene(1, "removed")); step(m, scene(2, suppress=True))
        self.assertEqual(m.snapshot()[0]["negative_views"], 0)

    def test_new_evidence_required_after_query(self):
        m = self.seeded()
        self.assertFalse(m.query_memory("找红色杯子", after=0)[0]["confirmed_after_query"])
        step(m, scene(1, "out_of_view"))
        self.assertFalse(m.query_memory("找红色杯子", after=0)[0]["confirmed_after_query"])
        step(m, scene(2, camera_x=.12))
        self.assertTrue(m.query_memory("找红色杯子", after=0)[0]["confirmed_after_query"])
        self.assertEqual(len(m.tracks), 1)
        self.assertEqual(len(m.query_memory("找蓝色杯子")), 1)       # 10-22: colour words do not filter
        self.assertEqual(m.query_memory("找大象"), [])

    def test_chinese_backpack_alias_does_not_also_match_book(self):
        m = self.seeded()
        book = m.tracks["object-0001"]
        book.label = "book"
        backpack = copy.deepcopy(book)
        backpack.id = "object-0002"
        backpack.label = "backpack"
        backpack.position[0] += 1
        m.tracks[backpack.id] = backpack

        self.assertEqual([row["id"] for row in m.query_memory("找书包")], ["object-0002"])
        self.assertEqual([row["id"] for row in m.query_memory("find red book")], ["object-0001"])

    def test_ambiguous_instances_not_arbitrarily_merged(self):
        # legacy association (identity_v2=False): one box near two stored instances stays ambiguous
        m = Memory(cfg=_Config(identity_v2=False)); step(m, scene(0)); twin = copy.deepcopy(m.tracks["object-0001"])
        twin.id = "object-0002"; twin.position[0] += .1; m.tracks[twin.id] = twin
        result = step(m, scene(1))
        self.assertEqual(len(m.tracks), 2)
        self.assertEqual(len(result["ambiguous"]), 1)
        self.assertTrue(all(t["state"] == "historical_memory" for t in m.snapshot()))

    def test_identity_v2_merges_twins_only_when_never_seen_together(self):
        # 10-27: a twin 0.1 m away that was never seen in the same frame is a duplicate -> merged into the
        # smaller number; one that was seen together with it (co-visible) is another object and stays.
        m = self.seeded(); twin = copy.deepcopy(m.tracks["object-0001"])
        twin.id, twin.number = "object-0002", 2; twin.position[0] += .1; m.tracks[twin.id] = twin
        result = step(m, scene(1))
        self.assertEqual(sorted(m.tracks), ["object-0001"])
        self.assertEqual(result["merged"][0]["merged"], "object-0002")
        self.assertEqual(m.resolve("object-0002"), "object-0001")
        m = self.seeded(); twin = copy.deepcopy(m.tracks["object-0001"])
        twin.id, twin.number = "object-0002", 2; twin.position[0] += .1; m.tracks[twin.id] = twin
        twin.covis.add("object-0001"); m.tracks["object-0001"].covis.add("object-0002")
        step(m, scene(1))
        self.assertEqual(sorted(m.tracks), ["object-0001", "object-0002"])

    def test_changed_units_wrong_frame_and_improper_pose_rejected(self):
        f, d = scene(0); f.depth *= 5000
        with self.assertRaisesRegex(ValueError, "units"): process_frame(f, d)
        f, d = scene(0); f.depth_timestamp = 1
        with self.assertRaisesRegex(ValueError, "timestamp"): process_frame(f, d)
        f, d = scene(0); f.T_world_camera[0, 0] = -1
        with self.assertRaisesRegex(ValueError, "orthogonal"): process_frame(f, d)
        f, d = scene(0); o = process_frame(f, d); g, _ = scene(1)
        with self.assertRaisesRegex(ValueError, "different"): process_frame(g, d)
        with self.assertRaisesRegex(ValueError, "different"): Memory().update_memory(o, {}, g)

    def test_out_of_order_timestamp(self):
        m = self.seeded(); f, d = scene(1); f.timestamp = -.5; f.depth_timestamp = -.5
        with self.assertRaisesRegex(ValueError, "out-of-order"): step(m, (f, []))

    def test_pose_interpolation_translation_rotation_and_gap(self):
        rows = [["0", "0", "0", "0", "0", "0", "0", "1"],
                [".02", "2", "0", "0", "0", "0", "1", "0"]]
        T, valid = interpolate_pose(.01, rows, np.array([0, .02]))
        self.assertTrue(valid); np.testing.assert_allclose(T[:3, 3], [1, 0, 0])
        np.testing.assert_allclose(T[:3, :3], [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-12)
        self.assertFalse(interpolate_pose(.1, rows, np.array([0, .2]))[1])
        exact, valid = interpolate_pose(0, rows, np.array([0, .02]))
        self.assertTrue(valid); np.testing.assert_allclose(exact, np.eye(4))
        with self.assertRaises(ValueError): quaternion_matrix([0, 0, 0, 0])

    def test_canonical_adapter_png_scale_checksum_and_range(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td); f, _ = scene(0)
            cv2.imwrite(str(root/"rgb.png"), cv2.cvtColor(f.rgb, cv2.COLOR_RGB2BGR))
            cv2.imwrite(str(root/"depth.png"), (f.depth*5000).astype(np.uint16))
            rec = {"rgb": "rgb.png", "depth": "depth.png", "rgb_sha256": sha256(root/"rgb.png"),
                   "depth_sha256": sha256(root/"depth.png"), "depth_scale": 5000,
                   "depth_kind": "optical_z", "frame_id": "0", "timestamp": 0, "depth_timestamp": 0,
                   "K": f.K.tolist(), "T_world_camera": f.T_world_camera.tolist(), "pose_valid": True,
                   "pose_source": "test", "calibration_id": "test"}
            loaded = load_frame(rec, root)
            np.testing.assert_allclose(loaded.depth, f.depth)
            rec["depth_kind"] = "ray_range"
            with self.assertRaisesRegex(ValueError, "ray range"): load_frame(rec, root)
            rec["depth_kind"] = "optical_z"; rec["depth_sha256"] = "wrong"
            with self.assertRaisesRegex(ValueError, "checksum"): load_frame(rec, root)


if __name__ == "__main__":
    unittest.main()
