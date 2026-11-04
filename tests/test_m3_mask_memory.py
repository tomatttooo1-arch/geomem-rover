"""Cross-module calibration: known optical geometry and distinct source failures."""
from copy import deepcopy
import hashlib
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from perception.offline_memory import OfflineMemory
from perception.fpga_replay.core import Frame, backproject
from perception.fpga_replay.io import sha256
from tools.m3_mask_memory import (CALIBRATION, MAP_ID, MAP_VERSION,
                                  OPTICAL_FRAME, POSE_SOURCE, frozen_manifest,
                                  manifest_file, mask_geometry, path_within,
                                  read_instance_mask, tum_record)


ROOT = Path(__file__).resolve().parents[1]


class MaskMemoryCalibration(unittest.TestCase):
    def fixture(self, depth=None):
        depth = np.full((6, 6), 2., dtype=np.float32) if depth is None else depth
        K = np.array([[2, 0, 2], [0, 2, 2], [0, 0, 1]], dtype=float)
        T = np.array([[0, -1, 0, 1], [1, 0, 0, 3], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        frame = Frame("tum-rgb:1.000000", 1., np.zeros((6, 6, 3), np.uint8),
                      depth, K, T, True, POSE_SOURCE, CALIBRATION, 1.0)
        frame.validate()
        mask = np.zeros((6, 6), bool)
        mask[1:5, 1:5] = True
        return frame, mask

    def m4_document(self, frame, geometry):
        evidence = {"path": str(Path(__file__).resolve()), "sha256": sha256(__file__)}
        document = {"schema": "dev3.m4.input.v1",
                    "maps": [{"map_id": MAP_ID, "map_version": MAP_VERSION,
                              "world_frame": "TUM_mocap_world", "units": "m",
                              "pose_source": POSE_SOURCE, "source_kind": "TUM_mocap_reference",
                              "source_ref": evidence, "calibration_id": CALIBRATION}],
                    "poses": [{"map_id": MAP_ID, "keyframe_id": frame.frame_id, "version": "tum-mocap-v1",
                               "frame_id": OPTICAL_FRAME, "T_map_local": frame.T_world_camera.tolist(),
                               "source": POSE_SOURCE, "evidence_refs": [evidence]}],
                    "batches": [{"map_id": MAP_ID, "map_version": MAP_VERSION,
                                 "acquisition": {"id": frame.frame_id, "source_id": "synthetic-test-acquisition",
                                                 "image_sha256": hashlib.sha256(b"calibration rgb").hexdigest(),
                                                 "timestamp": 1., "time_source": "fixture_rgb_table_row",
                                                 "clock_domain": "fixture_clock", "calibration_id": CALIBRATION},
                                 "prediction_set": "synthetic-known-answer", "status": "ok", "error": None,
                                 "observations": [{"instance_id": 1, "instance_kind": "fixture_instance",
                                                   "label": "cup", "score": .8, "source": "fixture_mask",
                                                   "geometry": geometry, "evidence_refs": [evidence]}]}]}
        return document

    def test_known_local_geometry_rotated_world_and_m4_candidate(self):
        frame, mask = self.fixture()
        geometry = mask_geometry(mask, frame, True)
        np.testing.assert_allclose(geometry["position_local_m"], [.5, .5, 2], atol=1e-12)
        self.assertEqual(geometry["valid_fraction"], 1.)
        document = self.m4_document(frame, geometry)
        memory = OfflineMemory()
        memory.import_document(document)
        cup = memory.query("cup")["objects"][0]
        self.assertEqual(cup["state"], "candidate")
        np.testing.assert_allclose(cup["position_m"], [.5, 3.5, 2], atol=1e-12)
        self.assertEqual(memory.import_document(document)["batches"][0]["status"], "duplicate")

    def test_depth_filtering_is_per_mask_pixel_and_all_invalid_is_not_object(self):
        frame, mask = self.fixture()
        frame.depth[1, 1] = np.nan
        frame.depth[0, 0] = 9.  # Outside the mask: must not contaminate a summary.
        valid_geometry = mask_geometry(mask, frame, True)
        self.assertEqual(valid_geometry["mask_pixels"], 16)
        self.assertEqual(valid_geometry["valid_depth_pixels"], 15)
        self.assertEqual(valid_geometry["valid_fraction"], 15/16)
        v, u = np.where(mask & np.isfinite(frame.depth))
        expected = np.median(backproject(np.c_[u, v], frame.depth[v, u], frame.K, np.eye(4)), axis=0)
        np.testing.assert_allclose(valid_geometry["position_local_m"], expected)
        frame.depth[mask] = np.nan
        invalid = mask_geometry(mask, frame, True)
        self.assertEqual(invalid["reason"], "no_valid_depth")
        self.assertEqual(invalid["valid_depth_pixels"], 0)
        self.assertNotIn("position_local_m", invalid)
        invalid_memory = OfflineMemory()
        invalid_memory.import_document(self.m4_document(frame, invalid))
        self.assertEqual(invalid_memory.query("cup")["objects"], [])
        invalid_raw = next(iter(invalid_memory.data["observations"].values()))
        self.assertEqual(invalid_raw["raw"]["geometry"]["reason"], "no_valid_depth")
        self.assertEqual(invalid_raw["geometry_status"], "invalid_geometry:no_valid_depth")
        v, u = np.where(mask)
        frame.depth[v[:10], u[:10]] = 2.  # 10 valid of 16; M7 threshold 9, M4 fraction threshold .7.
        low_quality = mask_geometry(mask, frame, True)
        self.assertTrue(low_quality["valid"])
        self.assertEqual(low_quality["valid_fraction"], 10/16)
        quality_memory = OfflineMemory()
        quality_memory.import_document(self.m4_document(frame, low_quality))
        self.assertEqual(quality_memory.query("cup")["objects"], [])
        quality_raw = next(iter(quality_memory.data["observations"].values()))
        self.assertEqual(quality_raw["geometry_status"], "insufficient_geometry_quality")
        self.assertIn("position_local_m", quality_raw["raw"]["geometry"])
        self.assertEqual(mask_geometry(mask, None, True)["reason"], "missing_aligned_TUM_depth")
        self.assertEqual(mask_geometry(mask, frame, False)["reason"], "missing_TUM_mocap_reference_pose")

    def test_frozen_manifest_rejects_wrong_frame_or_same_size_mask_swap(self):
        (ROOT / "runs").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT / "runs") as temp:
            base = Path(temp)
            frame_a = base / "m3-batch-20260916/batch/image-001/masks"
            frame_b = base / "m3-batch-20260916/batch/image-002/masks"
            frame_a.mkdir(parents=True)
            frame_b.mkdir(parents=True)
            one, two = np.zeros((6, 6), np.uint8), np.zeros((6, 6), np.uint8)
            one[0:3, 0:3] = 255
            two[3:6, 3:6] = 255
            first, second = frame_a / "instance-001.png", frame_b / "instance-001.png"
            cv2.imwrite(str(first), one)
            cv2.imwrite(str(second), two)
            manifest = base / "MANIFEST-m3-batch-20260916.sha256"
            relative_first = "batch/image-001/masks/instance-001.png"
            relative_second = "batch/image-002/masks/instance-001.png"
            manifest.write_text(f"{sha256(first)}  ./{relative_first}\n"
                                f"{sha256(second)}  ./{relative_second}\n")
            entries = frozen_manifest(manifest, sha256(manifest))
            det = {"instance_id": 1, "mask_path": "masks/instance-001.png",
                   "mask_shape_hw": [6, 6], "foreground_pixels": 9, "bbox_xyxy": [0, 0, 5, 5]}
            mask, _ = read_instance_mask(det, 1, "batch/image-001", base / "m3-batch-20260916",
                                         entries, (6, 6))
            self.assertEqual(int(mask.sum()), 9)
            wrong_owner = deepcopy(det)
            wrong_owner["instance_id"] = 2
            with self.assertRaisesRegex(ValueError, "ownership"):
                read_instance_mask(wrong_owner, 1, "batch/image-001", base / "m3-batch-20260916",
                                   entries, (6, 6))
            wrong_dimensions = deepcopy(det)
            wrong_dimensions["mask_shape_hw"] = [6, 5]
            with self.assertRaisesRegex(ValueError, "raster"):
                read_instance_mask(wrong_dimensions, 1, "batch/image-001", base / "m3-batch-20260916",
                                   entries, (6, 6))
            first.write_bytes(second.read_bytes())  # both masks still 6x6, 9 foreground pixels
            with self.assertRaisesRegex(ValueError, "SHA256"):
                read_instance_mask(det, 1, "batch/image-001", base / "m3-batch-20260916",
                                   entries, (6, 6))
            with self.assertRaisesRegex(ValueError, "SHA256"):
                frozen_manifest(manifest, sha256(manifest))

    def test_rgb_table_frame_binding_and_missing_depth(self):
        (ROOT / "runs").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT / "runs") as temp:
            data = Path(temp)
            (data / "rgb").mkdir()
            rgb = data / "rgb/1.000000.png"
            cv2.imwrite(str(rgb), np.zeros((6, 6, 3), np.uint8))
            (data / "rgb.txt").write_text("1.000000 rgb/1.000000.png\n")
            image = {"source_path": str(rgb), "sha256": sha256(rgb),
                     "frame_id": "sha256:" + sha256(rgb), "acquisition_timestamp": 1.,
                     "timestamp_source": {"path": str(data / "rgb.txt"),
                                          "sha256": sha256(data / "rgb.txt"),
                                          "row": "1.000000 rgb/1.000000.png"}}
            selected = {"source_path": str(rgb), "sha256": sha256(rgb),
                        "timestamp_text": "1.000000"}
            truth = [["1.000000", "0", "0", "0", "0", "0", "0", "1"]]
            record, pose_valid = tum_record(image, selected, data,
                                             {"rgb/1.000000.png": "1.000000"},
                                             [["1.500000", "depth/1.500000.png"]],
                                             np.array([1.5]), truth, np.array([1.]), set())
            self.assertTrue(pose_valid)
            self.assertIsNone(record["depth"])
            swapped = deepcopy(image)
            swapped["timestamp_source"]["row"] = "2.000000 rgb/1.000000.png"
            with self.assertRaisesRegex(ValueError, "timestamp source"):
                tum_record(swapped, selected, data, {"rgb/1.000000.png": "1.000000"},
                           [["1.500000", "depth/1.500000.png"]], np.array([1.5]),
                           truth, np.array([1.]), set())
            moved = deepcopy(image)
            moved["acquisition_timestamp"] = 1.2
            with self.assertRaisesRegex(ValueError, "acquisition time"):
                tum_record(moved, selected, data, {"rgb/1.000000.png": "1.000000"},
                           [["1.500000", "depth/1.500000.png"]], np.array([1.5]),
                           truth, np.array([1.]), set())



class PathEscapeGuards(unittest.TestCase):
    """Paired controls for the frozen-source path guards.

    Each guard gets a legal path that must pass and an escaping path that must
    be refused.  These also pin the 3.8 floor: ``Path.is_relative_to`` only
    exists from 3.9, so on the declared reference interpreter the old spelling
    did not weaken these guards, it raised ``AttributeError`` and took the
    whole tool down.  Every case below therefore has to run clean on 3.8.
    """

    def test_path_within_agrees_with_the_stdlib_method_it_replaces(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp).resolve()
            (base / "root/inside").mkdir(parents=True)
            (base / "sibling").mkdir()
            (base / "root/link").symlink_to(base / "sibling", target_is_directory=True)
            root = base / "root"
            legal = [(root / "inside", root), (root / "inside/file.png", root),
                     (root, root), (root / "link/file.png", root),
                     (Path("a/b"), Path("a"))]
            escaping = [(base / "sibling", root), (base / "sibling/file.png", root),
                        ((root / ".." / "sibling").resolve(), root),
                        ((root / "link/file.png").resolve(), root),
                        (root, root / "inside"), (Path("a/b"), Path("/a"))]
            for path, other in legal:
                self.assertTrue(path_within(path, other), (path, other))
            for path, other in escaping:
                self.assertFalse(path_within(path, other), (path, other))
            if hasattr(Path("/"), "is_relative_to"):  # 3.9+: differential control
                for path, other in legal + escaping:
                    self.assertEqual(path_within(path, other),
                                     Path(path).is_relative_to(other), (path, other))

    def manifest(self, base, rows):
        path = base / "MANIFEST-m3-batch-20260916.sha256"
        path.write_text("".join("%s  ./%s\n" % row for row in rows))
        return path

    def test_frozen_manifest_refuses_a_row_pointing_outside_the_batch_root(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp).resolve()
            pinned = base / "m3-batch-20260916/batch/image-001/masks/instance-001.png"
            pinned.parent.mkdir(parents=True)
            cv2.imwrite(str(pinned), np.zeros((6, 6), np.uint8))
            inside = "batch/image-001/masks/instance-001.png"
            legal = self.manifest(base, [(sha256(pinned), inside)])
            self.assertEqual(frozen_manifest(legal, sha256(legal)),
                             {inside: sha256(pinned)})
            # The escaping row carries the true digest of the file it reaches,
            # so only the containment check can reject it.
            escape = base / "outside.png"
            cv2.imwrite(str(escape), np.full((6, 6), 255, np.uint8))
            unsafe = self.manifest(base, [(sha256(escape), "../outside.png")])
            self.assertEqual(sha256((base / "m3-batch-20260916/../outside.png").resolve()),
                             sha256(escape))
            with self.assertRaisesRegex(ValueError, "unsafe or duplicate"):
                frozen_manifest(unsafe, sha256(unsafe))

    def test_manifest_file_refuses_a_pinned_relative_path_that_escapes(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp).resolve() / "m3-batch-20260916"
            root.mkdir()
            digest = "0" * 64
            inside = "batch/image-001/masks/instance-001.png"
            self.assertEqual(manifest_file(root, {inside: digest}, inside), root / inside)
            with self.assertRaisesRegex(ValueError, "escapes frozen source"):
                manifest_file(root, {"../outside.png": digest}, "../outside.png")

    def tum_fixture(self, base, depth_relative):
        data = base / "data"
        (data / "rgb").mkdir(parents=True)
        (data / "depth").mkdir()
        rgb = data / "rgb/1.000000.png"
        cv2.imwrite(str(rgb), np.zeros((6, 6, 3), np.uint8))
        (data / "rgb.txt").write_text("1.000000 rgb/1.000000.png\n")
        target = (data / depth_relative)
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(target), np.full((6, 6), 1000, np.uint16))
        image = {"source_path": str(rgb), "sha256": sha256(rgb),
                 "frame_id": "sha256:" + sha256(rgb), "acquisition_timestamp": 1.,
                 "timestamp_source": {"path": str(data / "rgb.txt"),
                                      "sha256": sha256(data / "rgb.txt"),
                                      "row": "1.000000 rgb/1.000000.png"}}
        selected = {"source_path": str(rgb), "sha256": sha256(rgb),
                    "timestamp_text": "1.000000"}
        return data, image, selected

    def call(self, data, image, selected, depth_relative):
        return tum_record(image, selected, data, {"rgb/1.000000.png": "1.000000"},
                          [["1.010000", depth_relative]], np.array([1.01]),
                          [["1.000000", "0", "0", "0", "0", "0", "0", "1"]],
                          np.array([1.]), set())

    def test_tum_record_refuses_a_depth_row_that_escapes_the_dataset(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp).resolve()
            inside = "depth/1.010000.png"
            data, image, selected = self.tum_fixture(base, inside)
            record, _ = self.call(data, image, selected, inside)
            self.assertEqual(record["depth"], inside)
            self.assertEqual(record["depth_sha256"], sha256(data / inside))
            # "depth/../../escape.png" passes the depth/ prefix check and names a
            # readable file, so only the containment check stands between the
            # tool and hashing a file outside the frozen dataset.
            escape = base / "escape.png"
            cv2.imwrite(str(escape), np.full((6, 6), 2000, np.uint16))
            outside = "depth/../../escape.png"
            self.assertTrue(outside.startswith("depth/"))
            self.assertEqual(sha256((data / outside).resolve()), sha256(escape))
            with self.assertRaisesRegex(ValueError, "invalid depth table path"):
                self.call(data, image, selected, outside)

    def test_tum_record_refuses_an_rgb_source_outside_the_dataset(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp).resolve()
            inside = "depth/1.010000.png"
            data, image, selected = self.tum_fixture(base, inside)
            self.assertIsNotNone(self.call(data, image, selected, inside)[0])
            stray = base / "stray/rgb/1.000000.png"
            stray.parent.mkdir(parents=True)
            stray.write_bytes((data / "rgb/1.000000.png").read_bytes())
            moved = deepcopy(image)
            moved["source_path"] = str(stray)
            moved_selection = deepcopy(selected)
            moved_selection["source_path"] = str(stray)
            with self.assertRaisesRegex(ValueError, "outside the TUM RGB directory"):
                self.call(data, moved, moved_selection, inside)


if __name__ == "__main__":
    unittest.main()
