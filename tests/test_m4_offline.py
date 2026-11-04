"""Synthetic logic tests; no real sensor/recognition/SLAM accuracy claims.

Identity answers occur only in assertions. Normal association sees only the
same provenance, category, geometry and acquisition metadata as the file API.
"""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

from perception.offline_memory import OfflineMemory


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = {"path": str(Path(__file__).resolve()),
            "sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


def matrix(rotation=None, translation=(0, 0, 0)):
    value = np.eye(4)
    if rotation is not None:
        value[:3, :3] = rotation
    value[:3, 3] = translation
    return value.tolist()


def pose(anchor, map_id="synthetic-map", version="pose-v1", transform=None):
    return {"map_id": map_id, "keyframe_id": anchor, "version": version,
            "frame_id": "synthetic_camera_optical", "T_map_local": transform or matrix(),
            "source": "synthetic_exact_pose", "evidence_refs": [deepcopy(EVIDENCE)]}


def observation(instance=1, position=(1, 0, 2), label="cup", anchor="capture-a", **changes):
    row = {"instance_id": instance, "instance_kind": "synthetic_single_frame_index",
           "label": label, "score": .6, "source": "synthetic_detector_n",
           "geometry": {"valid": True, "reason": None, "position_local_m": list(position),
                        "keyframe_id": anchor, "pose_version": "pose-v1",
                        "frame_id": "synthetic_camera_optical", "units": "m",
                        "summary_method": "explicit_synthetic_point", "valid_fraction": .95},
           "evidence_refs": [deepcopy(EVIDENCE)]}
    row.update(changes)
    return row


def document(capture="capture-a", points=None, timestamp=10., map_id="synthetic-map",
             source_id="synthetic-recording", prediction="synthetic-model-n", anchor=None,
             map_version="map-v1"):
    anchor = anchor or capture
    if points is None:
        points = [observation(anchor=anchor)]
    return {
        "schema": "dev3.m4.input.v1",
        "maps": [{"map_id": map_id, "map_version": map_version,
                  "world_frame": "synthetic_world", "units": "m",
                  "pose_source": "synthetic_exact_pose", "source_kind": "synthetic",
                  "source_ref": deepcopy(EVIDENCE), "calibration_id": "synthetic-calibration"}],
        "poses": [pose(anchor, map_id)],
        "batches": [{"map_id": map_id, "map_version": map_version,
                     "acquisition": {"id": capture, "source_id": source_id,
                         "image_sha256": hashlib.sha256(("synthetic image fixture:" + capture).encode()).hexdigest(),
                         "timestamp": timestamp, "time_source": None if timestamp is None else "synthetic_fixture_clock",
                         "clock_domain": "synthetic_clock", "calibration_id": "synthetic-calibration"},
                     "prediction_set": prediction, "status": "ok", "error": None,
                     "observations": points}],
    }


def revision(updates, revision_id="revision-1", from_version="map-v1", to_version="map-v2",
             unchanged=(), map_id="synthetic-map"):
    return {"schema": "dev3.m4.revision.v1", "revision_id": revision_id, "map_id": map_id,
            "from_version": from_version, "to_version": to_version,
            "reason": "explicit synthetic absolute pose correction; not a real loop closure",
            "poses": updates, "unchanged_keyframes": list(unchanged)}


class OfflineMemoryTests(unittest.TestCase):
    def test_late_unknown_pose_cannot_be_promoted_from_old_map_metadata(self):
        memory = OfflineMemory()
        original = document()
        memory.import_document(original)
        memory.revise(revision([pose("capture-a", version="pose-v2", transform=matrix(translation=(2, 0, 0)))]))
        late = deepcopy(original)
        late["poses"] = [pose("late-old-anchor")]
        late["batches"] = []
        before = deepcopy(memory.data)
        with self.assertRaisesRegex(ValueError, "current map"):
            memory.import_document(late)
        self.assertEqual(memory.data, before)

    def test_intermediate_revision_input_and_known_pose_remain_idempotent(self):
        memory = OfflineMemory()
        memory.import_document(document())
        p2 = pose("capture-a", version="pose-v2", transform=matrix(translation=(1, 0, 0)))
        memory.revise(revision([p2]))
        obs = observation(anchor="capture-a")
        obs["geometry"]["pose_version"] = "pose-v2"
        middle = document("capture-b", [obs], 11, anchor="capture-a", map_version="map-v2")
        middle["poses"] = []
        memory.import_document(middle)
        memory.revise(revision([pose("capture-a", version="pose-v3", transform=matrix(translation=(2, 0, 0)))],
                               "revision-2", "map-v2", "map-v3"))
        before = deepcopy(memory.data)
        report = memory.import_document(middle)
        self.assertEqual(report["batches"][0]["status"], "duplicate")
        self.assertEqual(memory.data, before)
        # Metadata can include a previously registered pose version verbatim.
        middle["poses"] = [p2]
        self.assertEqual(memory.import_document(middle)["batches"][0]["status"], "duplicate")
        self.assertEqual(memory.data, before)

    def test_malformed_json_shapes_raise_explicit_input_errors(self):
        for value in ([], None, "wrong"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    OfflineMemory().import_document(value)
                with self.assertRaises(ValueError):
                    OfflineMemory().revise(value)
        bad = document()
        bad["batches"][0]["observations"][0]["evidence_refs"] = [None]
        with self.assertRaises(ValueError):
            OfflineMemory().import_document(bad)

    def test_two_compatible_acquisitions_associate_without_claiming_motion_or_truth(self):
        memory = OfflineMemory()
        memory.import_document(document())
        report = memory.import_document(document("capture-b", [observation(position=(1.1, 0, 2), anchor="capture-b")], 11))
        rows = memory.query("  CUP  ")["objects"]
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["physical_observation_count"], 2)
        self.assertEqual(rows[0]["state"], "candidate")
        self.assertEqual(report["batches"][0]["decisions"][0]["decision"], "associated_mutual_unique")
        self.assertEqual(len(rows[0]["observation_ids"]), 2)
        np.testing.assert_allclose(rows[0]["position_m"], [1.1, 0, 2])

    def test_same_frame_nearby_instances_stay_separate_and_query_returns_all(self):
        memory = OfflineMemory()
        memory.import_document(document(points=[observation(), observation(2, (1.1, 0, 2))]))
        rows = memory.query("cup")["objects"]
        self.assertEqual(len(rows), 2)
        self.assertEqual(len({row["id"] for row in rows}), 2)
        self.assertEqual([row["physical_observation_count"] for row in rows], [1, 1])
        report = memory.import_document(document("capture-b", [observation(position=(1.05, 0, 2), anchor="capture-b")], 11))
        self.assertEqual(report["batches"][0]["decisions"][0]["decision"], "ambiguous_association")
        self.assertEqual(len(memory.query("cup")["objects"]), 2)
        self.assertEqual([r["physical_observation_count"] for r in memory.query("cup")["objects"]], [1, 1])

    def test_two_new_observations_competing_for_one_old_object_are_both_pending(self):
        memory = OfflineMemory()
        memory.import_document(document())
        report = memory.import_document(document("capture-b", [
            observation(position=(1.02, 0, 2), anchor="capture-b"),
            observation(2, (1.1, 0, 2), anchor="capture-b")], 11))
        self.assertEqual([d["decision"] for d in report["batches"][0]["decisions"]],
                         ["ambiguous_association", "ambiguous_association"])
        self.assertEqual(memory.query("cup")["objects"][0]["physical_observation_count"], 1)

    def test_instance_id_reuse_does_not_define_identity(self):
        memory = OfflineMemory()
        memory.import_document(document())
        memory.import_document(document("capture-b", [observation(1, (5, 0, 2), anchor="capture-b")], 11))
        self.assertEqual(len(memory.query("cup")["objects"]), 2)

    def test_different_map_or_source_never_merges_on_distance(self):
        for change in ({"map_id": "different-map"}, {"source_id": "different-recording"}):
            with self.subTest(change=change):
                memory = OfflineMemory()
                memory.import_document(document())
                memory.import_document(document("capture-b", [observation(anchor="capture-b")], 11, **change))
                self.assertEqual(len(memory.query("cup")["objects"]), 2)

    def test_exact_repeat_and_renamed_replay_do_not_add_evidence(self):
        memory = OfflineMemory()
        original = document()
        memory.import_document(original)
        before = deepcopy(memory.data)
        self.assertEqual(memory.import_document(original)["batches"][0]["status"], "duplicate")
        self.assertEqual(memory.data, before)
        renamed = deepcopy(original)
        renamed["batches"][0]["acquisition"]["id"] = "same-image-another-replay-name"
        self.assertEqual(memory.import_document(renamed)["batches"][0]["status"], "duplicate")
        self.assertEqual(memory.data, before)

    def test_same_image_model_variants_preserve_predictions_without_independent_count(self):
        memory = OfflineMemory()
        original = document()
        memory.import_document(original)
        second = deepcopy(original)
        second["batches"][0]["prediction_set"] = "synthetic-model-s"
        second["batches"][0]["observations"][0]["instance_id"] = 7
        second["batches"][0]["observations"][0]["score"] = .99
        second["batches"][0]["observations"][0]["source"] = "synthetic_detector_s"
        memory.import_document(second)
        result = memory.query("cup")
        self.assertEqual(len(result["objects"]), 1)
        self.assertEqual(result["objects"][0]["physical_observation_count"], 1)
        self.assertEqual(len(memory.data["observations"]), 2)
        self.assertEqual(result["pending_observations"][0]["decision"], "alternate_prediction_same_physical_capture")

    def test_changed_prediction_under_identical_identity_is_rejected_atomically(self):
        memory = OfflineMemory()
        original = document()
        memory.import_document(original)
        before = deepcopy(memory.data)
        changed = deepcopy(original)
        changed["batches"][0]["observations"][0]["geometry"]["position_local_m"][0] = 5
        with self.assertRaises(ValueError):
            memory.import_document(changed)
        self.assertEqual(memory.data, before)

    def test_invalid_geometry_and_low_quality_keep_evidence_without_origin_object(self):
        for geometry in ({"valid": False, "reason": "missing_depth"},
                         dict(observation()["geometry"], valid_fraction=.1)):
            with self.subTest(geometry=geometry):
                memory = OfflineMemory()
                memory.import_document(document(points=[observation(geometry=geometry)]))
                result = memory.query("cup")
                self.assertEqual(result["objects"], [])
                self.assertEqual(len(result["pending_observations"]), 1)
                self.assertIsNone(result["pending_observations"][0]["position_m"])
                self.assertEqual(result["pending_observations"][0]["raw"]["geometry"], geometry)

    def test_missing_source_and_wrong_units_are_rejected_without_partial_mutation(self):
        for kind in ("source", "units", "unknown_map", "calibration"):
            with self.subTest(kind=kind):
                memory = OfflineMemory()
                memory.import_document(document())
                before = deepcopy(memory.data)
                broken = document("capture-b", [observation(anchor="capture-b")], 11)
                batch = broken["batches"][0]
                if kind == "source":
                    del batch["observations"][0]["source"]
                elif kind == "units":
                    batch["observations"][0]["geometry"]["units"] = "mm"
                elif kind == "unknown_map":
                    batch["map_id"] = "not-registered"
                else:
                    batch["acquisition"]["calibration_id"] = "another-calibration"
                with self.assertRaises(ValueError):
                    memory.import_document(broken)
                self.assertEqual(memory.data, before)

    def test_missing_or_incompatible_pose_stays_pending_not_identity_transform(self):
        for kind in ("missing", "wrong_frame", "wrong_version"):
            with self.subTest(kind=kind):
                memory = OfflineMemory()
                doc = document()
                if kind == "missing":
                    doc["poses"] = []
                elif kind == "wrong_frame":
                    doc["batches"][0]["observations"][0]["geometry"]["frame_id"] = "base_link"
                else:
                    doc["batches"][0]["observations"][0]["geometry"]["pose_version"] = "unknown-version"
                memory.import_document(doc)
                result = memory.query()
                self.assertEqual(result["objects"], [])
                self.assertIsNone(result["pending_observations"][0]["position_m"])

    def test_unknown_and_out_of_order_time_preserve_newer_position_and_last_seen(self):
        memory = OfflineMemory()
        memory.import_document(document(timestamp=20))
        memory.import_document(document("capture-older", [observation(position=(1.1, 0, 2), anchor="capture-older")], 10))
        memory.import_document(document("capture-unknown", [observation(position=(1.2, 0, 2), anchor="capture-unknown")], None))
        row = memory.query("cup")["objects"][0]
        self.assertEqual(row["last_seen"], 20)
        self.assertEqual(row["last_seen_frame"], "capture-a")
        np.testing.assert_allclose(row["position_m"], [1, 0, 2])
        self.assertEqual(row["physical_observation_count"], 3)
        raw = [o for o in memory.data["observations"].values() if o["batch"]["acquisition"]["id"] == "capture-unknown"][0]
        self.assertIsNone(raw["batch"]["acquisition"]["timestamp"])
        self.assertIsNone(raw["batch"]["acquisition"]["time_source"])
        only_unknown = OfflineMemory()
        only_unknown.import_document(document(timestamp=None))
        self.assertIsNone(only_unknown.query()["objects"][0]["last_seen"])

    def test_category_conflicts_do_not_become_truth_by_score_or_repetition(self):
        memory = OfflineMemory()
        memory.import_document(document())
        for i in range(3):
            capture = f"conflicting-capture-{i}"
            memory.import_document(document(capture, [observation(label="bottle", anchor=capture, score=.999)], 11 + i))
        cup = memory.query("cup")["objects"][0]
        self.assertEqual(cup["label"], "cup")
        self.assertEqual(cup["physical_observation_count"], 1)
        self.assertEqual(cup["state"], "needs_review")
        self.assertEqual(memory.query("bottle")["objects"], [])
        self.assertEqual(len(memory.query("bottle")["pending_observations"]), 3)
        same_frame = OfflineMemory()
        same_frame.import_document(document(points=[observation(), observation(2, label="bottle", score=.999)]))
        self.assertEqual(same_frame.query()["objects"], [])
        self.assertEqual(len(same_frame.query()["pending_observations"]), 2)

    def test_empty_invalid_geometry_and_execution_failure_do_not_delete_objects(self):
        memory = OfflineMemory()
        memory.import_document(document())
        empty = memory.import_document(document("empty", [], 100))
        invalid = memory.import_document(document("invalid", [observation(geometry={"valid": False, "reason": "localization_invalid"})], 101))
        failed_doc = document("failed", [], 102)
        failed_doc["batches"][0].update(status="failed", error="synthetic execution failure")
        failed = memory.import_document(failed_doc)
        self.assertEqual(empty["batches"][0]["status"], "empty")
        self.assertEqual(invalid["batches"][0]["status"], "ok")
        self.assertEqual(failed["batches"][0]["status"], "failed")
        row = memory.query("cup")["objects"][0]
        self.assertEqual(row["last_seen"], 10)
        self.assertEqual(row["physical_observation_count"], 1)
        self.assertEqual(row["state"], "candidate")

    def test_absolute_rotation_translation_revisions_keep_ids_raw_and_provenance(self):
        memory = OfflineMemory()
        original = document()
        memory.import_document(original)
        before = memory.query()["objects"][0]
        raw = {k: deepcopy(v["raw"]) for k, v in memory.data["observations"].items()}
        first = revision([pose("capture-a", version="pose-v2", transform=matrix(
            [[0, -1, 0], [1, 0, 0], [0, 0, 1]], (10, 20, 30)))])
        memory.revise(first)
        after = memory.query()["objects"][0]
        np.testing.assert_allclose(after["position_m"], [10, 21, 32])
        self.assertEqual(after["id"], before["id"])
        self.assertEqual(after["evidence_refs"], before["evidence_refs"])
        self.assertEqual(after["map_version"], "map-v2")
        second = revision([pose("capture-a", version="pose-v3", transform=matrix(
            [[-1, 0, 0], [0, -1, 0], [0, 0, 1]], (3, 4, 5)))],
            "revision-2", "map-v2", "map-v3")
        memory.revise(second)
        np.testing.assert_allclose(memory.query()["objects"][0]["position_m"], [2, 4, 7])
        self.assertEqual({k: v["raw"] for k, v in memory.data["observations"].items()}, raw)
        snapshot = deepcopy(memory.data)
        self.assertEqual(memory.revise(second)["status"], "duplicate_revision")
        self.assertEqual(memory.import_document(original)["batches"][0]["status"], "duplicate")
        self.assertEqual(memory.data, snapshot)

    def test_new_observation_can_use_current_known_pose_version_after_revision(self):
        memory = OfflineMemory()
        memory.import_document(document())
        memory.revise(revision([pose("capture-a", version="pose-v2", transform=matrix(translation=(5, 0, 0)))]))
        fresh = observation(position=(1.1, 0, 2), anchor="capture-a")
        fresh["geometry"]["pose_version"] = "pose-v2"
        doc = document("capture-b", [fresh], 11, anchor="capture-a", map_version="map-v2")
        doc["poses"] = []
        memory.import_document(doc)
        rows = memory.query("cup")["objects"]
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["physical_observation_count"], 2)
        np.testing.assert_allclose(rows[0]["position_m"], [6.1, 0, 2])

    def test_partial_revision_marks_missing_anchor_pending_and_keeps_unaffected_object(self):
        memory = OfflineMemory()
        memory.import_document(document())
        memory.import_document(document("capture-b", [observation(position=(1.05, 0, 2), anchor="capture-b")], 11))
        memory.import_document(document("capture-c", [observation(position=(5, 0, 2), anchor="capture-c")], 12))
        ids = [row["id"] for row in memory.query()["objects"]]
        memory.revise(revision([pose("capture-a", version="pose-v2", transform=matrix(translation=(1, 0, 0)))], unchanged=["capture-c"]))
        rows = memory.query()["objects"]
        self.assertEqual([row["id"] for row in rows], ids)
        self.assertEqual(rows[0]["state"], "pending_revision")
        self.assertIsNone(rows[0]["position_m"])
        np.testing.assert_allclose(rows[1]["position_m"], [5, 0, 2])
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "memory.json"
            memory.save(path)
            self.assertEqual(OfflineMemory.load(path).query(), memory.query())

    def test_unanchored_map_summary_cannot_claim_synced_after_revision(self):
        memory = OfflineMemory()
        point = observation()
        point["geometry"] = {"valid": True, "reason": None, "position_m": [1, 0, 2],
                             "frame_id": "synthetic_world", "units": "m",
                             "summary_method": "explicit_synthetic_world_point", "valid_fraction": .9}
        doc = document(points=[point])
        doc["poses"] = []
        memory.import_document(doc)
        memory.revise(revision([]))
        row = memory.query()["objects"][0]
        self.assertIsNone(row["position_m"])
        self.assertEqual(row["state"], "pending_revision")

    def test_invalid_revision_is_transactional_and_stale_version_rejected(self):
        memory = OfflineMemory()
        memory.import_document(document())
        before = deepcopy(memory.data)
        invalid_rotation = matrix()
        invalid_rotation[0][0] = -1
        with self.assertRaises(ValueError):
            memory.revise(revision([pose("capture-a", version="pose-v2", transform=invalid_rotation)]))
        self.assertEqual(memory.data, before)
        with self.assertRaises(ValueError):
            memory.revise(revision([], from_version="stale-version"))
        self.assertEqual(memory.data, before)

    def test_reload_repeat_and_repeat_after_revision_preserve_store(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "memory.json"
            original = document()
            memory = OfflineMemory()
            memory.import_document(original)
            memory.save(path)
            loaded = OfflineMemory.load(path)
            before = loaded.query()
            loaded.import_document(original)
            self.assertEqual(loaded.query(), before)
            loaded.revise(revision([pose("capture-a", version="pose-v2", transform=matrix(translation=(1, 2, 3)))]))
            loaded.save(path)
            reloaded = OfflineMemory.load(path)
            before = reloaded.query()
            reloaded.import_document(original)
            self.assertEqual(reloaded.query(), before)
            self.assertEqual(before["objects"][0]["physical_observation_count"], 1)

    def test_atomic_replace_failure_preserves_previous_valid_store(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "memory.json"
            memory = OfflineMemory()
            memory.import_document(document())
            memory.save(path)
            before = path.read_bytes()
            memory.import_document(document("capture-b", [observation(anchor="capture-b")], 11))
            with mock.patch("perception.offline_memory.os.replace", side_effect=OSError("synthetic write failure")):
                with self.assertRaises(OSError):
                    memory.save(path)
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(OfflineMemory.load(path).query()["objects"][0]["physical_observation_count"], 1)

    def test_stale_writer_cannot_overwrite_newer_store(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "memory.json"
            memory = OfflineMemory()
            memory.import_document(document())
            memory.save(path)
            writer_a, writer_b = OfflineMemory.load(path), OfflineMemory.load(path)
            writer_a.import_document(document("capture-b", [observation(anchor="capture-b")], 11))
            writer_a.save(path)
            before = path.read_bytes()
            writer_b.import_document(document("capture-c", [observation(anchor="capture-c")], 12))
            with self.assertRaises(ValueError):
                writer_b.save(path)
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(OfflineMemory.load(path).query()["objects"][0]["physical_observation_count"], 2)

    def test_corrupt_store_is_error_not_empty_query(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "memory.json"
            memory = OfflineMemory()
            memory.import_document(document())
            memory.save(path)
            self.assertEqual(memory.query("unseen class")["objects"], [])
            wrapper = json.loads(path.read_text())
            wrapper["data"]["objects"]["object-0001"]["position_m"][0] = 99
            path.write_text(json.dumps(wrapper))
            with self.assertRaises(ValueError):
                OfflineMemory.load(path)

    def test_cli_new_process_import_query_revise_repeat_and_storage_errors(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            store, inputs, changes = root / "memory.json", root / "input.json", root / "revision.json"
            inputs.write_text(json.dumps(document()))
            changes.write_text(json.dumps(revision([pose("capture-a", version="pose-v2", transform=matrix(
                [[0, -1, 0], [1, 0, 0], [0, 0, 1]], (10, 20, 30)))])))

            def call(command, *extra, success=True):
                env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
                run = subprocess.run([sys.executable, "-B", "-m", "host.offline_memory", command,
                                      "--store", str(store), *map(str, extra)], cwd=ROOT, env=env,
                                     text=True, capture_output=True, timeout=30)
                if success:
                    self.assertEqual(run.returncode, 0, run.stderr)
                    result = json.loads(run.stdout)
                    self.assertTrue(result["ok"])
                else:
                    self.assertNotEqual(run.returncode, 0)
                    result = json.loads(run.stderr)
                    self.assertFalse(result["ok"])
                return result

            imported = call("import", "--input", inputs)
            queried = call("query", "--category", "cup")
            self.assertNotEqual(imported["process_id"], queried["process_id"])
            self.assertNotEqual(queried["process_id"], os.getpid())
            object_id = queried["objects"][0]["id"]
            revised = call("revise", "--input", changes)
            after = call("query", "--category", "cup")
            self.assertNotEqual(revised["process_id"], after["process_id"])
            self.assertEqual(after["objects"][0]["id"], object_id)
            np.testing.assert_allclose(after["objects"][0]["position_m"], [10, 21, 32])
            self.assertEqual(call("import", "--input", inputs)["batches"][0]["status"], "duplicate")
            self.assertEqual(call("query", "--category", "cup")["objects"][0]["physical_observation_count"], 1)
            self.assertEqual(call("query", "--category", "unknown")["objects"], [])
            store.write_text("{broken json")
            failure = call("query", "--category", "cup", success=False)
            self.assertIn("error", failure)


if __name__ == "__main__":
    unittest.main()
