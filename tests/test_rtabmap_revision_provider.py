import copy
import math
import unittest

import numpy as np

from host.rtabmap_revision_provider import (
    BINDING_SCHEMA, SNAPSHOT_SCHEMA, anchor_from_binding,
    revision_from_snapshots, seal_binding, seal_snapshot,
    snapshot_transition, validate_binding, validate_bindings,
    validate_snapshot,
)
from perception.offline_memory import OfflineMemory
from tools.rtabmap_memory_anchor_probe import rotation_diagnostics


SHA = {
    "db": "1" * 64, "poses": "2" * 64, "manifest": "3" * 64,
    "calibration": "4" * 64, "rgb": "5" * 64, "depth": "6" * 64,
    "bindings": "7" * 64, "next_db": "8" * 64, "next_poses": "9" * 64,
}


def ref(role, sha=None, suffix=None):
    return {"role": role, "path": "/evidence/%s%s" % (role, suffix or ""),
            "sha256": sha or SHA.get(role, "a" * 64)}


def transform(axis=(0.0, 0.0, 1.0), degrees=0.0, translation=(0.0, 0.0, 0.0)):
    axis = np.asarray(axis, dtype=float); axis /= np.linalg.norm(axis)
    angle = math.radians(degrees)
    cross = np.array([[0.0, -axis[2], axis[1]],
                      [axis[2], 0.0, -axis[0]],
                      [-axis[1], axis[0], 0.0]])
    rotation = np.eye(3) + math.sin(angle) * cross + (1.0 - math.cos(angle)) * (cross @ cross)
    value = np.eye(4); value[:3, :3] = rotation; value[:3, 3] = translation
    return value.tolist()


LOCAL = transform((1, 1, 0), 37, (0.31, -0.22, 0.17))


def snapshot(nodes=None, *, parent=None, sequence=1, kind="full", missing=(),
             lineage="room-lineage", provider="mapping-session-a", evidence_suffix=""):
    if nodes is None:
        nodes = [
            (1, 1001, transform((0, 0, 1), 29, (1.2, -0.4, 0.3))),
            (2, 1002, transform((0, 1, 0), -17, (-0.2, 0.8, 1.1))),
            (3, 1003, transform((1, 0, 0), 11, (0.5, 0.6, -0.3))),
        ]
    raw = {
        "schema": SNAPSHOT_SCHEMA,
        "map_lineage_id": lineage,
        "parent_snapshot_id": parent,
        "sequence": sequence,
        "provider_instance_id": provider,
        "provider_version": "rtabmap-0.23.7",
        "snapshot_kind": kind,
        "gate_map_id": SHA["db"] if sequence == 1 else SHA["next_db"],
        "world_frame": "map",
        "node_frame": "camera_link",
        "calibration": {
            "calibration_id": SHA["calibration"],
            "image_size": [640, 480],
            "camera_matrix": [[517.3, 0.0, 318.6], [0.0, 516.5, 255.3], [0.0, 0.0, 1.0]],
            "distortion_model": "pre_registered_zero_distortion",
            "distortion_coefficients": [],
            "local_frame_id": "camera_optical_frame",
            "T_node_local": copy.deepcopy(LOCAL),
        },
        "nodes": [{"node_id": node_id, "stamp_ns": stamp, "T_map_node": pose}
                  for node_id, stamp, pose in nodes],
        "explicit_missing_node_ids": list(missing),
        "evidence_refs": [
            ref("rtabmap_database", SHA["db"] if sequence == 1 else SHA["next_db"], evidence_suffix),
            ref("optimized_camera_poses", SHA["poses"] if sequence == 1 else SHA["next_poses"], evidence_suffix),
            ref("capture_manifest", SHA["manifest"], evidence_suffix),
            ref("calibration", SHA["calibration"], evidence_suffix),
        ],
    }
    return seal_snapshot(raw)


def child(old, nodes=None, *, kind="full", missing=(), evidence_suffix="-v2"):
    if nodes is None:
        nodes = [(row["node_id"], row["stamp_ns"], copy.deepcopy(row["T_map_node"]))
                 for row in old["nodes"]]
    return snapshot(nodes, parent=old["snapshot_id"], sequence=old["sequence"] + 1,
                    kind=kind, missing=missing, lineage=old["map_lineage_id"],
                    provider=old["provider_instance_id"], evidence_suffix=evidence_suffix)


def binding(initial, node_id, pose_epoch="capture-epoch-1"):
    node = next(row for row in initial["nodes"] if row["node_id"] == node_id)
    raw = {
        "schema": BINDING_SCHEMA,
        "keyframe_id": "capture-%d" % node_id,
        "source_id": "recording-a",
        "image_sha256": SHA["rgb"][:-1] + str(node_id),
        "depth_sha256": SHA["depth"][:-1] + str(node_id),
        "rgb_stamp_ns": node["stamp_ns"],
        "depth_stamp_ns": node["stamp_ns"] + 10,
        "pose_epoch": pose_epoch,
        "map_lineage_id": initial["map_lineage_id"],
        "initial_snapshot_id": initial["snapshot_id"],
        "gate_map_id": initial["gate_map_id"],
        "provider_node_id": node_id,
        "provider_node_stamp_ns": node["stamp_ns"],
        "node_frame": initial["node_frame"],
        "local_frame_id": initial["calibration"]["local_frame_id"],
        "calibration_id": initial["calibration"]["calibration_id"],
        "image_size": copy.deepcopy(initial["calibration"]["image_size"]),
        "camera_matrix": copy.deepcopy(initial["calibration"]["camera_matrix"]),
        "distortion_model": initial["calibration"]["distortion_model"],
        "distortion_coefficients": copy.deepcopy(initial["calibration"]["distortion_coefficients"]),
        "T_node_local": copy.deepcopy(initial["calibration"]["T_node_local"]),
        "evidence_refs": [ref("capture_manifest", SHA["manifest"]),
                          ref("rgb_source", SHA["rgb"][:-1] + str(node_id), "-%d" % node_id),
                          ref("depth_source", SHA["depth"][:-1] + str(node_id), "-%d" % node_id),
                          ref("calibration", SHA["calibration"]),
                          ref("rtabmap_database", SHA["db"])],
    }
    return seal_binding(raw)


BINDING_REF = ref("capture_bindings", SHA["bindings"])


def memory_from(initial, bindings):
    anchors = [anchor_from_binding(initial, row, BINDING_REF) for row in bindings]
    maps = [{"map_id": initial["map_lineage_id"], "map_version": initial["snapshot_id"],
             "world_frame": initial["world_frame"], "units": "m",
             "pose_source": "rtabmap_absolute_node_pose",
             "source_kind": "rtabmap_provider_node_binding",
             "source_ref": anchors[0]["source_ref"],
             "calibration_id": initial["calibration"]["calibration_id"]}]
    poses = [{"map_id": a["map_id"], "keyframe_id": b["keyframe_id"],
              "version": a["pose_version"], "frame_id": a["local_frame_id"],
              "T_map_local": a["T_map_local"], "source": a["pose_source"],
              "evidence_refs": a["evidence_refs"]}
             for a, b in zip(anchors, bindings)]
    memory = OfflineMemory()
    memory.import_document({"schema": "dev3.m4.input.v1", "maps": maps,
                            "poses": poses, "batches": []})
    return memory


class RtabmapRevisionProviderTests(unittest.TestCase):
    def test_rotation_metric_projects_small_float_orthogonality_error(self):
        exact = np.eye(4)
        noisy = np.eye(4); noisy[0, 0] += 1e-8
        measured = rotation_diagnostics(noisy, exact)
        self.assertLess(measured["projected_rotation_error_deg"], 1e-8)
        self.assertNotEqual(measured["raw_trace_statistic_not_angle"], 1.0)
        self.assertGreater(measured["actual_orthogonality_fro"], 0.0)

    def test_rotation_metric_detects_known_angles_and_rejects_bad_matrices(self):
        identity = np.eye(4)
        for degrees in (0.01, 90.0):
            known = np.asarray(transform((0, 0, 1), degrees))
            with self.subTest(degrees=degrees):
                self.assertAlmostEqual(
                    rotation_diagnostics(identity, known)["projected_rotation_error_deg"],
                    degrees, places=8)
        reflection = np.eye(4); reflection[0, 0] = -1
        scaled = np.eye(4); scaled[0, 0] = 1.01
        for bad in (reflection, scaled):
            with self.assertRaisesRegex(ValueError, "reflective or too far"):
                rotation_diagnostics(bad, identity)

    def test_noncommuting_static_anchor_uses_map_node_times_node_local(self):
        first = snapshot()
        bound = binding(first, 1)
        anchor = anchor_from_binding(first, bound, BINDING_REF)
        node = np.asarray(first["nodes"][0]["T_map_node"])
        local = np.asarray(LOCAL)
        self.assertTrue(np.array_equal(anchor["T_map_local"], (node @ local).tolist()))
        self.assertFalse(np.allclose(anchor["T_map_local"], local @ node))
        self.assertEqual(anchor["map_id"], "room-lineage")
        self.assertEqual(anchor["map_version"], first["snapshot_id"])
        self.assertTrue(any(r["sha256"] == SHA["bindings"]
                            for r in anchor["evidence_refs"]))

    def test_snapshot_duplicate_and_ordered_parent_chain(self):
        first = snapshot()
        self.assertEqual(snapshot_transition(first, copy.deepcopy(first))["status"],
                         "duplicate_snapshot")
        second = child(first)
        self.assertEqual(snapshot_transition(first, second)["status"], "advance")
        for mutation, pattern in (
                (("parent_snapshot_id", "f" * 64), "parent"),
                (("sequence", 3), "sequence"),
                (("provider_instance_id", "other-provider"), "provider_instance_id")):
            raw = copy.deepcopy(second); raw.pop("snapshot_id"); raw[mutation[0]] = mutation[1]
            bad = seal_snapshot(raw)
            with self.assertRaisesRegex(ValueError, pattern):
                snapshot_transition(first, bad)

    def test_snapshot_id_collision_and_cross_lineage_fail(self):
        first = snapshot()
        duplicate_stamp = copy.deepcopy(first)
        duplicate_stamp["nodes"][0]["stamp_ns"] = duplicate_stamp["nodes"][1]["stamp_ns"]
        with self.assertRaisesRegex(ValueError, "node stamps must be unique"):
            validate_snapshot(duplicate_stamp)
        tampered = copy.deepcopy(first); tampered["nodes"][0]["T_map_node"][0][3] += 0.01
        with self.assertRaisesRegex(ValueError, "snapshot_id"):
            validate_snapshot(tampered)
        other = snapshot(lineage="another-lineage")
        raw = copy.deepcopy(other); raw.pop("snapshot_id")
        raw["parent_snapshot_id"] = first["snapshot_id"]; raw["sequence"] = 2
        other = seal_snapshot(raw)
        with self.assertRaisesRegex(ValueError, "map_lineage_id"):
            snapshot_transition(first, other)

    def test_binding_requires_exact_node_stamp_calibration_and_evidence(self):
        first = snapshot(); good = binding(first, 1)
        validate_binding(good, first)
        cases = []
        bad = copy.deepcopy(good); bad.pop("binding_id"); bad["provider_node_stamp_ns"] += 1
        cases.append((seal_binding(bad), "stamp"))
        bad = copy.deepcopy(good); bad.pop("binding_id"); bad["camera_matrix"][0][0] += 1
        cases.append((seal_binding(bad), "calibration"))
        bad = copy.deepcopy(good); bad.pop("binding_id"); bad["T_node_local"][0][3] += 1
        cases.append((seal_binding(bad), "calibration"))
        bad = copy.deepcopy(good); bad.pop("binding_id"); bad["evidence_refs"] = []
        cases.append((seal_binding(bad), "evidence"))
        for value, pattern in cases:
            with self.subTest(pattern=pattern):
                with self.assertRaisesRegex(ValueError, pattern):
                    validate_binding(value, first)

    def test_declared_content_hashes_must_match_evidence(self):
        first = snapshot()
        raw = copy.deepcopy(first); raw.pop("snapshot_id")
        next(ref for ref in raw["evidence_refs"]
             if ref["role"] == "rtabmap_database")["sha256"] = "f" * 64
        with self.assertRaisesRegex(ValueError, "gate_map_id"):
            seal_snapshot(raw)
        good = binding(first, 1)
        raw = copy.deepcopy(good); raw.pop("binding_id")
        next(ref for ref in raw["evidence_refs"]
             if ref["role"] == "rgb_source")["sha256"] = "e" * 64
        with self.assertRaisesRegex(ValueError, "image_sha256"):
            validate_binding(seal_binding(raw), first)

    def test_bindings_are_one_to_one(self):
        first = snapshot(); one = binding(first, 1); two = binding(first, 2)
        duplicate = copy.deepcopy(two); duplicate.pop("binding_id")
        duplicate["keyframe_id"] = one["keyframe_id"]
        duplicate = seal_binding(duplicate)
        with self.assertRaisesRegex(ValueError, "keyframe.*one-to-one"):
            validate_bindings([one, duplicate], first)

    def test_gate_epoch_changes_binding_not_snapshot_version(self):
        first = snapshot()
        a = binding(first, 1, "runtime-epoch-a")
        b = binding(first, 1, "runtime-epoch-b")
        self.assertNotEqual(a["binding_id"], b["binding_id"])
        self.assertEqual(anchor_from_binding(first, a, BINDING_REF)["map_version"],
                         anchor_from_binding(first, b, BINDING_REF)["map_version"])

    def test_updated_unchanged_missing_are_closed_and_apply_to_m4(self):
        first = snapshot(); bindings = [binding(first, i) for i in (1, 2, 3)]
        memory = memory_from(first, bindings)
        changed = copy.deepcopy(first["nodes"][0]["T_map_node"])
        changed[0][3] += 0.25
        second = child(first, [(1, 1001, changed),
                               (2, 1002, copy.deepcopy(first["nodes"][1]["T_map_node"]))],
                       missing=(3,))
        built = revision_from_snapshots(memory, first, first, second, bindings, BINDING_REF)
        self.assertEqual(built["audit"], {"updated": ["capture-1"],
                         "unchanged": ["capture-2"], "missing": ["capture-3"],
                         "from_sequence": 1, "to_sequence": 2})
        self.assertEqual(memory.revise(built["revision"])["status"], "revised")
        by_key = {row["original"]["keyframe_id"]: row for row in memory.data["poses"].values()}
        self.assertFalse(by_key["capture-1"]["pending"])
        self.assertFalse(by_key["capture-2"]["pending"])
        self.assertTrue(by_key["capture-3"]["pending"])
        self.assertEqual(memory.data["maps"]["room-lineage"]["map_version"], second["snapshot_id"])

    def test_partial_omission_is_not_implicit_missing(self):
        first = snapshot(); bindings = [binding(first, i) for i in (1, 2)]
        memory = memory_from(first, bindings)
        second = child(first, [(1, 1001, copy.deepcopy(first["nodes"][0]["T_map_node"]))],
                       kind="partial")
        with self.assertRaisesRegex(ValueError, "without explicit missing"):
            revision_from_snapshots(memory, first, first, second, bindings, BINDING_REF)

    def test_new_snapshot_with_identical_poses_advances_as_unchanged(self):
        first = snapshot(); bindings = [binding(first, 1)]
        memory = memory_from(first, bindings); second = child(first)
        built = revision_from_snapshots(memory, first, first, second, bindings, BINDING_REF)
        self.assertEqual(built["audit"]["updated"], [])
        self.assertEqual(built["audit"]["unchanged"], ["capture-1"])
        self.assertNotEqual(built["revision"]["from_version"], built["revision"]["to_version"])
        memory.revise(built["revision"])
        self.assertEqual(memory.data["maps"]["room-lineage"]["map_version"], second["snapshot_id"])

    def test_unchanged_then_updated_uses_trusted_per_anchor_pose_history(self):
        first = snapshot(); bound = binding(first, 1); memory = memory_from(first, [bound])
        second = child(first)
        unchanged = revision_from_snapshots(memory, first, first, second, [bound], BINDING_REF)
        memory.revise(unchanged["revision"])
        stored = next(iter(memory.data["poses"].values()))
        self.assertEqual(stored["current"]["version"],
                         "rtabmap:%s:node:1" % first["snapshot_id"])
        nodes = [(row["node_id"], row["stamp_ns"], copy.deepcopy(row["T_map_node"]))
                 for row in second["nodes"]]
        nodes[0][2][1][3] += 0.125
        third = child(second, nodes, evidence_suffix="-v3")
        updated = revision_from_snapshots(memory, first, second, third, [bound], BINDING_REF,
                                          snapshot_history=[first, second])
        self.assertEqual(updated["audit"]["updated"], ["capture-1"])
        memory.revise(updated["revision"])
        self.assertEqual(next(iter(memory.data["poses"].values()))["current"]["version"],
                         "rtabmap:%s:node:1" % third["snapshot_id"])

    def test_same_node_id_cannot_change_immutable_capture_stamp(self):
        first = snapshot(); bound = binding(first, 1); memory = memory_from(first, [bound])
        nodes = [(row["node_id"], row["stamp_ns"], copy.deepcopy(row["T_map_node"]))
                 for row in first["nodes"]]
        nodes[0] = (1, 9999, nodes[0][2])
        second = child(first, nodes)
        with self.assertRaisesRegex(ValueError, "different stamp"):
            revision_from_snapshots(memory, first, first, second, [bound], BINDING_REF)

    def test_tiny_pose_change_is_updated_without_semantic_tolerance(self):
        first = snapshot(); bound = binding(first, 1); memory = memory_from(first, [bound])
        changed = copy.deepcopy(first["nodes"][0]["T_map_node"]); changed[0][3] += 1e-12
        nodes = [(1, 1001, changed), (2, 1002, first["nodes"][1]["T_map_node"]),
                 (3, 1003, first["nodes"][2]["T_map_node"])]
        second = child(first, nodes)
        built = revision_from_snapshots(memory, first, first, second, [bound], BINDING_REF)
        self.assertEqual(built["audit"]["updated"], ["capture-1"])

    def test_wrong_old_pose_or_missing_binding_evidence_fails_before_revision(self):
        first = snapshot(); bound = binding(first, 1); memory = memory_from(first, [bound])
        second = child(first)
        pose = next(iter(memory.data["poses"].values()))["current"]
        pose["T_map_local"][0][3] += 1
        with self.assertRaisesRegex(ValueError, "current pose"):
            revision_from_snapshots(memory, first, first, second, [bound], BINDING_REF)
        memory = memory_from(first, [bound])
        pose = next(iter(memory.data["poses"].values()))["current"]
        pose["evidence_refs"] = [r for r in pose["evidence_refs"]
                                  if r["sha256"] != SHA["bindings"]]
        with self.assertRaisesRegex(ValueError, "current pose"):
            revision_from_snapshots(memory, first, first, second, [bound], BINDING_REF)

    def test_unbound_old_pose_cannot_receive_provider_revision(self):
        first = snapshot(); bound = binding(first, 1); memory = memory_from(first, [bound])
        extra = copy.deepcopy(next(iter(memory.data["poses"].values()))["original"])
        extra["keyframe_id"] = "old-mocap-or-synthetic"
        extra["version"] = "unbound-version"
        extra["source"] = "not_rtabmap_provider"
        key = json_key = '["room-lineage","old-mocap-or-synthetic"]'
        memory.data["poses"][key] = {"original": extra, "current": copy.deepcopy(extra),
                                     "versions": {extra["version"]: copy.deepcopy(extra)},
                                     "pending": False}
        with self.assertRaisesRegex(ValueError, "closed set"):
            revision_from_snapshots(memory, first, first, child(first), [bound], BINDING_REF)
        self.assertEqual(json_key, key)  # keep the canonical key visible in this calibration

    def test_missing_anchor_restore_requires_explicit_pose_history(self):
        first = snapshot(); bindings = [binding(first, i) for i in (1, 2)]
        memory = memory_from(first, bindings)
        second = child(first, [(2, 1002, copy.deepcopy(first["nodes"][1]["T_map_node"]))],
                       missing=(1,))
        missing = revision_from_snapshots(memory, first, first, second, bindings, BINDING_REF)
        memory.revise(missing["revision"])
        restored_nodes = [(row["node_id"], row["stamp_ns"], copy.deepcopy(row["T_map_node"]))
                          for row in first["nodes"]]
        third = child(second, restored_nodes, evidence_suffix="-v3")
        with self.assertRaisesRegex(ValueError, "restore_requires_explicit_pose_history"):
            revision_from_snapshots(memory, first, second, third, bindings, BINDING_REF,
                                    snapshot_history=[first, second])

    def test_duplicate_snapshot_does_not_emit_revision(self):
        first = snapshot(); bound = binding(first, 1); memory = memory_from(first, [bound])
        result = revision_from_snapshots(memory, first, first, copy.deepcopy(first),
                                         [bound], BINDING_REF)
        self.assertEqual(result["status"], "duplicate_snapshot")
        self.assertIsNone(result["revision"])


    def test_pending_anchor_is_refused_when_from_snapshot_still_has_the_node(self):
        """Isolate the `pending is False` guard from its downstream twin.

        test_missing_anchor_restore_requires_explicit_pose_history reaches the
        *other* check that raises this same message: there the node sits in
        from_snapshot's explicit_missing_node_ids, so that check refuses the
        call whether or not the pending check exists. Here from_snapshot still
        carries the node, so only `stored["pending"] is False` stands in the
        way of silently revising an anchor another revision left pending.
        """
        first = snapshot(); bindings = [binding(first, i) for i in (1, 2)]
        memory = memory_from(first, bindings)
        nodes = {row["node_id"]: copy.deepcopy(row["T_map_node"]) for row in first["nodes"]}
        moved = copy.deepcopy(nodes[2]); moved[0][3] += 0.25
        second = child(first, [(1, 1001, copy.deepcopy(nodes[1])), (2, 1002, moved),
                               (3, 1003, copy.deepcopy(nodes[3]))])
        built = revision_from_snapshots(memory, first, first, second, bindings, BINDING_REF)
        self.assertEqual(built["audit"]["updated"], ["capture-2"])
        self.assertEqual(built["audit"]["unchanged"], ["capture-1"])
        # Another M4 revision producer advances the same map without declaring
        # capture-1 unchanged; offline_memory.revise then marks it pending.
        other = copy.deepcopy(built["revision"])
        other["revision_id"] = "other-producer-revision-1"
        other["unchanged_keyframes"] = []
        self.assertEqual(memory.revise(other)["status"], "revised")
        rows = {row["original"]["keyframe_id"]: row for row in memory.data["poses"].values()}
        self.assertTrue(rows["capture-1"]["pending"])
        self.assertFalse(rows["capture-2"]["pending"])
        # from_snapshot lists node 1 as present and not missing, so the
        # downstream guard cannot fire for capture-1.
        self.assertNotIn(1, second["explicit_missing_node_ids"])
        self.assertIn(1, [row["node_id"] for row in second["nodes"]])
        second_nodes = {row["node_id"]: copy.deepcopy(row["T_map_node"])
                        for row in second["nodes"]}
        shifted = copy.deepcopy(second_nodes[1]); shifted[1][3] += 0.5
        third = child(second, [(1, 1001, shifted), (2, 1002, copy.deepcopy(second_nodes[2])),
                               (3, 1003, copy.deepcopy(second_nodes[3]))],
                      evidence_suffix="-v3")
        with self.assertRaisesRegex(ValueError, "restore_requires_explicit_pose_history"):
            revision_from_snapshots(memory, first, second, third, bindings, BINDING_REF,
                                    snapshot_history=[first, second])


if __name__ == "__main__":
    unittest.main()
