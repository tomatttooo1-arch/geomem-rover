import json
from contextlib import closing
import hashlib
from pathlib import Path
import sqlite3
import tempfile
import unittest

import numpy as np

from tools.rtabmap_reload_coordinate_evaluate import (
    OPTICAL_IN_TRACKED,
    alignment_matrix,
    audit_database,
    fixed_pose_errors,
    first_three,
    freeze_reference,
    identity_sha256,
    rotation_error_degrees,
    score_records,
    verify_approval_poses,
)
from host.localization_gate import pose_matrix
from tools.rtabmap_reload_coordinate_prepare import validate_calibration


CALIBRATION = """%YAML:1.0
---
camera_name: rtabmap_calib
image_width: 640
image_height: 480
camera_matrix:
   rows: 3
   cols: 3
   data: [ 517.3, 0., 318.6, 0., 516.5, 255.3, 0., 0., 1. ]
local_transform:
   rows: 3
   cols: 4
   data: [ 0., 0., 1., 0., -1., 0., 0., 0., 0., -1., 0., 0. ]
"""


class ReloadCoordinateEvaluationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.calibration = self.root / "rtabmap_calib.yaml"
        self.calibration.write_text(CALIBRATION)
        self.calibration_hash = hashlib.sha256(self.calibration.read_bytes()).hexdigest()
        self.calibration_validation = validate_calibration(self.calibration)

    def manifest(self, role, start, stop, count=10):
        rows = []
        for index in range(start, stop):
            timestamp = f"{100 + index}.000000"
            rows.append({
                "original_index": index,
                "timestamp": timestamp,
                "depth_timestamp": f"{100 + index}.010000",
                "rgb_sync": {"source": f"/raw/rgb/{index}.png",
                             "source_sha256": f"rgb-source-{index}",
                             "input_sha256": f"rgb-input-{index}"},
                "depth_sync": {"source": f"/raw/depth/{index}.png",
                               "source_sha256": f"depth-source-{index}",
                               "input_sha256": f"depth-input-{index}"},
            })
        result = {"schema": 1, "role": role, "split_index": 6, "split_ratio": .6,
                  "original_range": [start, stop], "source_manifest_sha256": "source",
                  "groundtruth_available_to_slam": False, "pairs": rows,
                  "calibration": {"source": str(self.calibration),
                                  "source_sha256": self.calibration_hash,
                                  "input_sha256": self.calibration_hash,
                                  "validation": self.calibration_validation}}
        result["pairs_identity_sha256"] = identity_sha256(result)
        return result

    def database(self, name="map.db", nodes=(), nonzero_ground_truth=False, extra_table=None):
        path = self.root / name
        with closing(sqlite3.connect(path)) as connection:
            connection.execute("CREATE TABLE Node(id INTEGER, stamp FLOAT, ground_truth_pose BLOB)")
            connection.execute("CREATE TABLE Data(id INTEGER)")
            connection.execute("CREATE TABLE Link(id INTEGER)")
            connection.execute("CREATE TABLE Word(id INTEGER)")
            connection.execute("CREATE TABLE Feature(id INTEGER)")
            for number, stamp in enumerate(nodes, 1):
                blob = (b"\1" + b"\0" * 47) if nonzero_ground_truth else b"\0" * 48
                connection.execute("INSERT INTO Node VALUES(?,?,?)", (number, stamp, blob))
                connection.execute("INSERT INTO Data VALUES(?)", (number,))
            if extra_table:
                connection.execute(f"INSERT INTO {extra_table} VALUES(1)")
            connection.commit()
        return path

    @staticmethod
    def write_tum(path, rows):
        np.savetxt(path, np.asarray(rows, dtype=float), fmt="%.9f")
        return path

    def write_manifests(self):
        mapping, query = self.manifest("mapping", 0, 6), self.manifest("query", 6, 10)
        mapping_path, query_path = self.root / "mapping/INPUT.json", self.root / "query/INPUT.json"
        mapping_path.parent.mkdir()
        query_path.parent.mkdir()
        mapping_path.write_text(json.dumps(mapping))
        query_path.write_text(json.dumps(query))
        (mapping_path.parent / "rtabmap_calib.yaml").write_text(CALIBRATION)
        (query_path.parent / "rtabmap_calib.yaml").write_text(CALIBRATION)
        return mapping_path, query_path

    def test_nontrivial_fixed_se3_scores_position_rotation_and_quaternion_sign(self):
        estimated = np.array([
            [100, 0, 0, 0, 0, 0, 0, 1],
            [101, 1, 0, 0, 0, 0, 0, 1],
            [102, 0, 1, 0, 0, 0, 0, 1],
        ], dtype=float)
        q = np.sqrt(.5)
        truth = np.array([
            [100, 3, -2, 1, 0, 0, q, q],
            [101, 3, -1, 1, 0, 0, -q, -q],
            [102, 2, -2, 1, 0, 0, q, q],
        ], dtype=float)
        alignment = {"alignment_rotation": [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
                     "alignment_translation_m": [3, -2, 1]}
        records = fixed_pose_errors(estimated, truth, alignment)
        self.assertEqual(len(records), 3)
        self.assertLess(max(row["translation_error_m"] for row in records), 1e-12)
        self.assertLess(max(row["rotation_error_deg"] for row in records), 1e-10)
        wrong_translation = dict(alignment, alignment_translation_m=[4, -2, 1])
        self.assertAlmostEqual(fixed_pose_errors(estimated, truth, wrong_translation)[0]["translation_error_m"], 1)
        wrong_rotation = {"alignment_rotation": np.eye(3).tolist(),
                          "alignment_translation_m": [3, -2, 1]}
        self.assertAlmostEqual(fixed_pose_errors(estimated, truth, wrong_rotation)[0]["rotation_error_deg"], 90)
        invalid = estimated.copy()
        invalid[0, 7] = 2
        with self.assertRaisesRegex(ValueError, "non-unit"):
            fixed_pose_errors(invalid, truth, alignment)

    def test_database_zero_placeholder_allowed_nonzero_detected(self):
        zero = self.database(nodes=(100, 101, 102))
        result = audit_database(zero)
        self.assertEqual(result["ground_truth_pose_blobs"]["all_zero"], 3)
        self.assertEqual(result["ground_truth_pose_blobs"]["nonzero"], 0)
        nonzero = self.database("nonzero.db", nodes=(100,), nonzero_ground_truth=True)
        with self.assertRaisesRegex(ValueError, "nonzero ground-truth"):
            audit_database(nonzero)

    def test_empty_control_requires_all_map_content_tables_empty(self):
        empty = self.database("empty.db")
        self.assertEqual(audit_database(empty, require_nonempty=False)["table_counts"]["Node"], 0)
        word = self.database("word.db", extra_table="Word")
        with self.assertRaisesRegex(ValueError, "map content"):
            audit_database(word, require_nonempty=False)

    def test_freeze_rejects_export_pose_not_from_database(self):
        mapping, query = self.write_manifests()
        database = self.database(nodes=(100, 101, 102))
        poses = self.write_tum(self.root / "poses.txt", [
            [100, 0, 0, 0, 0, 0, 0, 1], [101, 1, 0, 0, 0, 0, 0, 1],
            [102, 0, 1, 0, 0, 0, 0, 1], [106, 0, 0, 1, 0, 0, 0, 1]])
        truth = self.write_tum(self.root / "truth.txt", [
            [100, 0, 0, 0, 0, 0, 0, 1], [101, 1, 0, 0, 0, 0, 0, 1],
            [102, 0, 1, 0, 0, 0, 0, 1], [106, 0, 0, 1, 0, 0, 0, 1]])
        with self.assertRaisesRegex(ValueError, "exactly cover"):
            freeze_reference(poses, truth, database, mapping, query, "commit", self.root / "reference.json")

    def test_freeze_accepts_mapping_only_pose_identity(self):
        mapping, query = self.write_manifests()
        database = self.database(nodes=(100, 101, 102))
        poses = self.write_tum(self.root / "poses.txt", [
            [100, 0, 0, 0, 0, 0, 0, 1], [101, 1, 0, 0, 0, 0, 0, 1],
            [102, 0, 1, 0, 0, 0, 0, 1]])
        truth = self.write_tum(self.root / "truth.txt", [
            [100, 0, 0, 0, 0, 0, 0, 1], [101, 1, 0, 0, 0, 0, 0, 1],
            [102, 0, 1, 0, 0, 0, 0, 1]])
        reference = freeze_reference(poses, truth, database, mapping, query,
                                     "commit", self.root / "reference.json")
        identity = reference["mapping_pose_check"]["database_pose_stamp_identity"]
        self.assertEqual(identity["matches_within_2us"], 3)
        self.assertEqual(reference["alignment"]["alignment_covariance_rank"], 2)

    @staticmethod
    def header(stamp, frame):
        sec, nanosec = divmod(stamp, 10**9)
        return {"stamp": {"sec": sec, "nanosec": nanosec}, "frame_id": frame}

    def events(self, query, link=1, localization_offset_ns=0):
        covariance = np.eye(6).flatten().tolist()
        events = []
        for local_index, pair in enumerate(query["pairs"]):
            stamp = int(float(pair["timestamp"]) * 1e9)
            shifted = stamp + localization_offset_ns
            pose = {"position": {"x": float(local_index), "y": 0., "z": 0.},
                    "orientation": {"x": .5, "y": -.5, "z": .5, "w": .5}}
            events.extend([
                {"event": "publish_begin", "index": local_index, "stamp_ns": stamp,
                 "timestamp": pair["timestamp"],
                 "source_rgb_sha256": pair["rgb_sync"]["source_sha256"],
                 "source_depth_sha256": pair["depth_sync"]["source_sha256"],
                 "input_sha256": {"rgb_sync": pair["rgb_sync"]["input_sha256"],
                                  "depth_sync": pair["depth_sync"]["input_sha256"]}},
                {"event": "odom", "stamp_ns": stamp, "header": self.header(stamp, "odom"),
                 "child_frame_id": "camera_link", "pose": {"pose": pose, "covariance": covariance}},
                {"event": "odom_info", "stamp_ns": stamp, "header": self.header(stamp, "odom"),
                 "lost": False, "covariance": covariance},
                {"event": "info", "stamp_ns": stamp, "header": self.header(stamp, "map"),
                 "loop_closure_id": link, "proximity_detection_id": 0},
                {"event": "localization_pose", "stamp_ns": shifted,
                 "header": self.header(shifted, "map"), "pose": {"pose": pose, "covariance": covariance}},
            ])
        return events

    def test_query_uses_local_indices_and_prior_membership(self):
        query = self.manifest("query", 6, 10)
        truth = np.array([[100 + index, index - 6, 0, 0, 0, 0, 0, 1]
                          for index in range(6, 10)], dtype=float)
        reference = {"alignment": {"alignment_rotation": np.eye(3).tolist(),
                                   "alignment_translation_m": [0, 0, 0]}}
        records, unknown, _ = score_records(self.events(query), query, truth, reference, {1}, {})
        self.assertFalse(any(unknown.values()))
        self.assertEqual([row["original_index"] for row in records], [6, 7, 8, 9])
        self.assertTrue(all(row["healthy_prior_localization"] for row in records))
        self.assertLess(max(row["rotation_error_deg"] for row in records), 1e-10)
        wrong, _, _ = score_records(self.events(query, link=99), query, truth, reference, {1}, {})
        self.assertTrue(all(row["raw_positive_link"] for row in wrong))
        self.assertFalse(any(row["prior_link"] for row in wrong))
        self.assertFalse(any(row["healthy_prior_localization"] for row in wrong))

    def test_noncommuting_query_rotation_chain_order(self):
        query = self.manifest("query", 6, 10)
        events = self.events(query)
        root = np.sqrt(.5)
        map_camera_q = {"x": root, "y": 0., "z": 0., "w": root}
        for row in events:
            if row["event"] == "localization_pose":
                row["pose"]["pose"]["orientation"] = map_camera_q
        truth = np.array([[100 + index, 3, -2 + index - 6, 1, -.5, .5, .5, .5]
                          for index in range(6, 10)], dtype=float)
        reference = {"alignment": {
            "alignment_rotation": [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
            "alignment_translation_m": [3, -2, 1],
        }}
        records, _, streams = score_records(events, query, truth, reference, {1}, {})
        self.assertLess(max(row["translation_error_m"] for row in records), 1e-12)
        self.assertLess(max(row["rotation_error_deg"] for row in records), 1e-10)
        first = streams["localization_pose"][records[0]["stamp_ns"]]
        wrong_order = pose_matrix(first["pose"]["pose"]) @ alignment_matrix(reference["alignment"]) @ OPTICAL_IN_TRACKED
        expected = {"position": {"x": 3., "y": -2., "z": 1.},
                    "orientation": {"x": -.5, "y": .5, "z": .5, "w": .5}}
        self.assertGreater(rotation_error_degrees(pose_matrix(expected), wrong_order), 1.)

    def test_wrong_stamp_duplicate_and_approval_mismatch_detected(self):
        query = self.manifest("query", 6, 10)
        truth = np.array([[100 + index, index - 6, 0, 0, 0, 0, 0, 1]
                          for index in range(6, 10)], dtype=float)
        reference = {"alignment": {"alignment_rotation": np.eye(3).tolist(),
                                   "alignment_translation_m": [0, 0, 0]}}
        records, unknown, streams = score_records(
            self.events(query, localization_offset_ns=100_000_000), query, truth, reference, {1}, {})
        self.assertTrue(unknown["localization_pose"])
        self.assertFalse(any(row["numeric_pose"] for row in records))
        duplicate = self.events(query)
        duplicate.append(dict(duplicate[0]))
        with self.assertRaisesRegex(ValueError, "duplicate publication"):
            score_records(duplicate, query, truth, reference, {1}, {})
        stamp = records[0]["stamp_ns"]
        with self.assertRaisesRegex(ValueError, "differs"):
            verify_approval_poses({stamp: {"stamp_ns": stamp, "pose": {"bad": True}}},
                                  streams["localization_pose"], [stamp])

    def test_no_output_and_missing_frame_break_three_frame_streak(self):
        query = self.manifest("query", 6, 10)
        truth = np.array([[100 + index, index - 6, 0, 0, 0, 0, 0, 1]
                          for index in range(6, 10)], dtype=float)
        reference = {"alignment": {"alignment_rotation": np.eye(3).tolist(),
                                   "alignment_translation_m": [0, 0, 0]}}
        events = [row for row in self.events(query) if row["event"] != "localization_pose"]
        records, _, _ = score_records(events, query, truth, reference, {1}, {})
        self.assertFalse(any(row["numeric_pose"] for row in records))
        self.assertIsNone(first_three(records))
        records, _, _ = score_records(self.events(query), query, truth, reference, {1}, {})
        records[1]["published"] = False
        self.assertIsNone(first_three(records))


if __name__ == "__main__":
    unittest.main()
