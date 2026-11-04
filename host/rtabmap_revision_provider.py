"""Fail-closed RTAB-Map snapshot and M4 historical-anchor contract.

This module is deliberately independent of ROS and of the localization lease.
It binds captures to persistent provider nodes, emits an existing M7-to-M4
anchor, and converts an ordered pair of provider snapshots to the existing
``dev3.m4.revision.v1`` document.  It never treats a gate epoch, a current
``map->odom`` transform, or a missing node in a partial snapshot as a revision.
"""

from copy import deepcopy
import hashlib
import json
import math

import numpy as np


SNAPSHOT_SCHEMA = "fpga.rtabmap.snapshot.v1"
BINDING_SCHEMA = "fpga.rtabmap.capture-binding.v1"
REVISION_SCHEMA = "dev3.m4.revision.v1"


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False)


def digest(value):
    return hashlib.sha256(canonical(value).encode("utf8")).hexdigest()


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _text(value, name):
    _require(isinstance(value, str) and bool(value.strip()),
             "%s must be a nonempty string" % name)
    return value


def _sha(value, name):
    _text(value, name)
    _require(len(value) == 64 and all(c in "0123456789abcdef" for c in value),
             "%s must be a lowercase SHA256" % name)
    return value


def _integer(value, name, minimum=0):
    _require(type(value) is int and value >= minimum,
             "%s must be an integer >= %d" % (name, minimum))
    return value


def _rigid(value, name):
    try:
        matrix = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        raise ValueError("%s must be a finite 4x4 rigid transform" % name)
    _require(matrix.shape == (4, 4) and np.isfinite(matrix).all(),
             "%s must be a finite 4x4 rigid transform" % name)
    rotation = matrix[:3, :3]
    _require(np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
             and np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
             and np.isclose(np.linalg.det(rotation), 1, atol=1e-6, rtol=0),
             "%s must be a proper rigid transform" % name)
    return matrix


def _camera_matrix(value):
    try:
        matrix = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        raise ValueError("camera_matrix must be finite 3x3")
    _require(matrix.shape == (3, 3) and np.isfinite(matrix).all()
             and matrix[0, 0] > 0 and matrix[1, 1] > 0
             and np.array_equal(matrix[2], [0, 0, 1]),
             "camera_matrix must be a finite pinhole 3x3 matrix")
    return matrix


def _references(value, name="evidence_refs"):
    _require(isinstance(value, list) and bool(value),
             "%s must be a nonempty list" % name)
    result = []
    for index, ref in enumerate(value):
        _require(isinstance(ref, dict), "%s[%d] must be an object" % (name, index))
        _text(ref.get("role"), "%s[%d].role" % (name, index))
        path = _text(ref.get("path"), "%s[%d].path" % (name, index))
        _require(path.startswith("/"), "%s[%d].path must be absolute" % (name, index))
        _sha(ref.get("sha256"), "%s[%d].sha256" % (name, index))
        result.append(deepcopy(ref))
    _require(len({(r["role"], r["path"], r["sha256"]) for r in result}) == len(result),
             "%s contains duplicate references" % name)
    return result


def _calibration(value):
    _require(isinstance(value, dict), "calibration must be an object")
    _sha(value.get("calibration_id"), "calibration_id")
    size = value.get("image_size")
    _require(isinstance(size, list) and len(size) == 2
             and all(type(x) is int and x > 0 for x in size),
             "image_size must contain positive integer width and height")
    _camera_matrix(value.get("camera_matrix"))
    _text(value.get("distortion_model"), "distortion_model")
    coefficients = value.get("distortion_coefficients")
    _require(isinstance(coefficients, list)
             and all(not isinstance(x, bool) and isinstance(x, (int, float))
                     and math.isfinite(x) for x in coefficients),
             "distortion_coefficients must be finite numbers")
    _text(value.get("local_frame_id"), "local_frame_id")
    _rigid(value.get("T_node_local"), "T_node_local")
    return deepcopy(value)


def _snapshot_payload(snapshot):
    payload = deepcopy(snapshot)
    payload.pop("snapshot_id", None)
    return payload


def seal_snapshot(snapshot):
    """Return a validated snapshot with a content-derived ``snapshot_id``."""
    value = _snapshot_payload(snapshot)
    value["snapshot_id"] = digest(value)
    validate_snapshot(value)
    return value


def validate_snapshot(snapshot):
    _require(isinstance(snapshot, dict) and snapshot.get("schema") == SNAPSHOT_SCHEMA,
             "unsupported RTAB-Map snapshot schema")
    for key in ("map_lineage_id", "provider_instance_id", "provider_version",
                "world_frame", "node_frame"):
        _text(snapshot.get(key), key)
    _sha(snapshot.get("snapshot_id"), "snapshot_id")
    _sha(snapshot.get("gate_map_id"), "gate_map_id")
    parent = snapshot.get("parent_snapshot_id")
    if parent is not None:
        _sha(parent, "parent_snapshot_id")
    sequence = _integer(snapshot.get("sequence"), "sequence", 1)
    _require((sequence == 1) == (parent is None),
             "only sequence 1 may have a null parent_snapshot_id")
    _require(snapshot.get("snapshot_kind") in ("full", "partial"),
             "snapshot_kind must be full or partial")
    calibration = _calibration(snapshot.get("calibration"))
    nodes = snapshot.get("nodes")
    _require(isinstance(nodes, list) and bool(nodes), "snapshot nodes must be nonempty")
    node_ids, stamps = [], []
    for index, node in enumerate(nodes):
        _require(isinstance(node, dict), "node must be an object")
        node_ids.append(_integer(node.get("node_id"), "node_id", 1))
        stamps.append(_integer(node.get("stamp_ns"), "node.stamp_ns", 1))
        _rigid(node.get("T_map_node"), "node.T_map_node")
        _require(set(node) == {"node_id", "stamp_ns", "T_map_node"},
                 "node contains unsupported or missing fields")
    _require(node_ids == sorted(node_ids) and len(set(node_ids)) == len(node_ids),
             "node IDs must be unique and sorted")
    _require(len(set(stamps)) == len(stamps), "node stamps must be unique")
    missing = snapshot.get("explicit_missing_node_ids")
    _require(isinstance(missing, list)
             and all(type(x) is int and x > 0 for x in missing)
             and missing == sorted(missing) and len(set(missing)) == len(missing),
             "explicit_missing_node_ids must be unique sorted positive integers")
    _require(not set(missing) & set(node_ids),
             "present and explicitly missing node IDs overlap")
    refs = _references(snapshot.get("evidence_refs"))
    by_role = {ref["role"]: ref for ref in refs}
    _require(len(by_role) == len(refs), "snapshot evidence roles must be unique")
    _require(by_role.get("rtabmap_database", {}).get("sha256") == snapshot["gate_map_id"],
             "rtabmap_database evidence does not match gate_map_id")
    _require(by_role.get("calibration", {}).get("sha256") == calibration["calibration_id"],
             "calibration evidence does not match calibration_id")
    _require(snapshot["snapshot_id"] == digest(_snapshot_payload(snapshot)),
             "snapshot_id does not match canonical snapshot content")
    return deepcopy(snapshot)


def _nodes(snapshot):
    return {row["node_id"]: row for row in snapshot["nodes"]}


def snapshot_transition(old_snapshot, new_snapshot):
    """Validate ordering and return ``duplicate_snapshot`` or ``advance``."""
    old = validate_snapshot(old_snapshot)
    new = validate_snapshot(new_snapshot)
    if old["snapshot_id"] == new["snapshot_id"]:
        _require(canonical(old) == canonical(new), "snapshot ID collision")
        return {"status": "duplicate_snapshot", "from_snapshot_id": old["snapshot_id"],
                "to_snapshot_id": new["snapshot_id"]}
    for key in ("map_lineage_id", "provider_instance_id", "provider_version",
                "world_frame", "node_frame", "calibration"):
        _require(canonical(old[key]) == canonical(new[key]),
                 "snapshot transition changed %s" % key)
    _require(new["parent_snapshot_id"] == old["snapshot_id"],
             "snapshot parent is not the current snapshot")
    _require(new["sequence"] == old["sequence"] + 1,
             "snapshot sequence is not the next value")
    return {"status": "advance", "from_snapshot_id": old["snapshot_id"],
            "to_snapshot_id": new["snapshot_id"]}


def _binding_payload(binding):
    payload = deepcopy(binding)
    payload.pop("binding_id", None)
    return payload


def seal_binding(binding):
    value = _binding_payload(binding)
    value["binding_id"] = digest(value)
    return value


def validate_binding(binding, initial_snapshot):
    snapshot = validate_snapshot(initial_snapshot)
    _require(isinstance(binding, dict) and binding.get("schema") == BINDING_SCHEMA,
             "unsupported capture binding schema")
    _sha(binding.get("binding_id"), "binding_id")
    for key in ("keyframe_id", "source_id", "pose_epoch", "map_lineage_id",
                "initial_snapshot_id", "gate_map_id", "node_frame", "local_frame_id",
                "calibration_id"):
        _text(binding.get(key), key)
    _sha(binding.get("image_sha256"), "image_sha256")
    _sha(binding.get("depth_sha256"), "depth_sha256")
    node_id = _integer(binding.get("provider_node_id"), "provider_node_id", 1)
    node_stamp = _integer(binding.get("provider_node_stamp_ns"),
                          "provider_node_stamp_ns", 1)
    rgb_stamp = _integer(binding.get("rgb_stamp_ns"), "rgb_stamp_ns", 1)
    _integer(binding.get("depth_stamp_ns"), "depth_stamp_ns", 1)
    _require(node_stamp == rgb_stamp,
             "provider node stamp must equal the bound RGB capture stamp")
    _camera_matrix(binding.get("camera_matrix"))
    size = binding.get("image_size")
    _require(isinstance(size, list) and len(size) == 2
             and all(type(x) is int and x > 0 for x in size),
             "binding image_size must contain positive integer width and height")
    _text(binding.get("distortion_model"), "binding.distortion_model")
    coefficients = binding.get("distortion_coefficients")
    _require(isinstance(coefficients, list)
             and all(not isinstance(x, bool) and isinstance(x, (int, float))
                     and math.isfinite(x) for x in coefficients),
             "binding distortion_coefficients must be finite numbers")
    _rigid(binding.get("T_node_local"), "binding.T_node_local")
    refs = _references(binding.get("evidence_refs"))
    by_role = {ref["role"]: ref for ref in refs}
    _require(len(by_role) == len(refs), "binding evidence roles must be unique")
    _require(by_role.get("rgb_source", {}).get("sha256") == binding["image_sha256"],
             "RGB evidence does not match image_sha256")
    _require(by_role.get("depth_source", {}).get("sha256") == binding["depth_sha256"],
             "depth evidence does not match depth_sha256")
    _require(by_role.get("calibration", {}).get("sha256") == binding["calibration_id"],
             "calibration evidence does not match calibration_id")
    _require(by_role.get("rtabmap_database", {}).get("sha256") == binding["gate_map_id"],
             "database evidence does not match gate_map_id")
    _require(binding["binding_id"] == digest(_binding_payload(binding)),
             "binding_id does not match canonical binding content")
    _require(binding["map_lineage_id"] == snapshot["map_lineage_id"]
             and binding["initial_snapshot_id"] == snapshot["snapshot_id"]
             and binding["gate_map_id"] == snapshot["gate_map_id"],
             "binding map identity does not match its initial snapshot")
    calibration = snapshot["calibration"]
    _require(binding["node_frame"] == snapshot["node_frame"]
             and binding["local_frame_id"] == calibration["local_frame_id"]
             and binding["calibration_id"] == calibration["calibration_id"]
             and canonical(binding["image_size"]) == canonical(calibration["image_size"])
             and canonical(binding["camera_matrix"]) == canonical(calibration["camera_matrix"])
             and binding["distortion_model"] == calibration["distortion_model"]
             and canonical(binding["distortion_coefficients"])
                 == canonical(calibration["distortion_coefficients"])
             and canonical(binding["T_node_local"]) == canonical(calibration["T_node_local"]),
             "binding frame or calibration does not match its initial snapshot")
    node = _nodes(snapshot).get(node_id)
    _require(node is not None and node["stamp_ns"] == node_stamp,
             "binding provider node or stamp is absent from initial snapshot")
    return deepcopy(binding)


def validate_bindings(bindings, initial_snapshot):
    _require(isinstance(bindings, list) and bool(bindings), "bindings must be nonempty")
    checked = [validate_binding(row, initial_snapshot) for row in bindings]
    for name, values in (
            ("keyframe", [x["keyframe_id"] for x in checked]),
            ("provider node", [x["provider_node_id"] for x in checked]),
            ("RGB stamp", [x["rgb_stamp_ns"] for x in checked])):
        _require(len(values) == len(set(values)), "%s bindings are not one-to-one" % name)
    return checked


def _pose_version(snapshot_id, node_id):
    return "rtabmap:%s:node:%d" % (snapshot_id, node_id)


def _binding_ref_present(pose, binding_document_ref):
    expected = (binding_document_ref["path"], binding_document_ref["sha256"])
    return any((ref.get("path"), ref.get("sha256")) == expected
               for ref in pose.get("evidence_refs", []))


def anchor_from_binding(snapshot, binding, binding_document_ref):
    """Generate the anchor dictionary consumed by ``build_m4_document``.

    This pure function cannot read ``binding_document_ref.path``.  Its caller
    must hash the immutable binding document bytes first; the real-file probe
    does that after writing ``bindings.json`` and before calling this function.
    """
    snapshot = validate_snapshot(snapshot)
    binding = validate_binding(binding, snapshot)
    binding_ref = _references([binding_document_ref], "binding_document_ref")[0]
    _require(binding_ref["role"] == "capture_bindings",
             "binding_document_ref must have capture_bindings role")
    node = _nodes(snapshot)[binding["provider_node_id"]]
    matrix = (_rigid(node["T_map_node"], "T_map_node")
              @ _rigid(binding["T_node_local"], "T_node_local"))
    database_ref = next((deepcopy(ref) for ref in snapshot["evidence_refs"]
                         if ref["role"] == "rtabmap_database"), None)
    _require(database_ref is not None, "snapshot lacks rtabmap_database evidence")
    evidence = deepcopy(snapshot["evidence_refs"]) + deepcopy(binding["evidence_refs"]) + [binding_ref]
    unique = []
    seen = set()
    for ref in evidence:
        key = (ref["role"], ref["path"], ref["sha256"])
        if key not in seen:
            seen.add(key); unique.append(ref)
    return {
        "map_id": snapshot["map_lineage_id"],
        "map_version": snapshot["snapshot_id"],
        "world_frame": snapshot["world_frame"],
        "pose_source": "rtabmap_absolute_node_pose",
        "source_kind": "rtabmap_provider_node_binding",
        "pose_version": _pose_version(snapshot["snapshot_id"], binding["provider_node_id"]),
        "local_frame_id": binding["local_frame_id"],
        "pose_epoch": binding["pose_epoch"],
        "calibration_id": binding["calibration_id"],
        "T_map_local": matrix.tolist(),
        "source_ref": database_ref,
        "evidence_refs": unique,
        "provider_binding": {
            "binding_id": binding["binding_id"],
            "initial_snapshot_id": binding["initial_snapshot_id"],
            "provider_node_id": binding["provider_node_id"],
            "provider_node_stamp_ns": binding["provider_node_stamp_ns"],
        },
    }


def _memory_data(memory):
    data = memory.data if hasattr(memory, "data") else memory
    _require(isinstance(data, dict), "memory store data required")
    for key in ("maps", "poses"):
        _require(isinstance(data.get(key), dict), "memory store lacks %s" % key)
    return data


def _validated_history(initial, old, snapshot_history):
    if initial["snapshot_id"] == old["snapshot_id"]:
        supplied = [initial] if snapshot_history is None else snapshot_history
    else:
        _require(snapshot_history is not None,
                 "complete snapshot history is required after the initial version")
        supplied = snapshot_history
    _require(isinstance(supplied, list) and bool(supplied),
             "snapshot history must be a nonempty list")
    history = [validate_snapshot(row) for row in supplied]
    _require(history[0]["snapshot_id"] == initial["snapshot_id"]
             and history[-1]["snapshot_id"] == old["snapshot_id"],
             "snapshot history does not span initial through current")
    _require(len({row["snapshot_id"] for row in history}) == len(history),
             "snapshot history repeats a version")
    for previous, current in zip(history, history[1:]):
        _require(snapshot_transition(previous, current)["status"] == "advance",
                 "snapshot history contains a non-advance transition")
    return history


def revision_from_snapshots(memory, initial_snapshot, from_snapshot, to_snapshot,
                            bindings, binding_document_ref, snapshot_history=None):
    """Build, but do not apply, an exhaustive M4 revision.

    Version 1 intentionally rejects restoration of an anchor already marked
    pending by an earlier missing-node revision.  Restoration needs an explicit
    pose history contract and is not inferred from a node reappearing.
    """
    initial = validate_snapshot(initial_snapshot)
    old = validate_snapshot(from_snapshot)
    new = validate_snapshot(to_snapshot)
    transition = snapshot_transition(old, new)
    if transition["status"] == "duplicate_snapshot":
        return {"status": "duplicate_snapshot", "revision": None,
                "audit": {"updated": [], "unchanged": [], "missing": []}}
    _require(initial["map_lineage_id"] == old["map_lineage_id"],
             "initial and current snapshots use different lineages")
    history = _validated_history(initial, old, snapshot_history)
    binding_ref = _references([binding_document_ref], "binding_document_ref")[0]
    _require(binding_ref["role"] == "capture_bindings",
             "binding_document_ref must have capture_bindings role")
    bindings = validate_bindings(bindings, initial)
    data = _memory_data(memory)
    lineage = old["map_lineage_id"]
    map_row = data["maps"].get(lineage)
    _require(isinstance(map_row, dict), "memory has no provider lineage map")
    _require(map_row.get("map_id") == lineage
             and map_row.get("map_version") == old["snapshot_id"]
             and map_row.get("world_frame") == old["world_frame"]
             and map_row.get("calibration_id") == old["calibration"]["calibration_id"],
             "memory map metadata does not match from snapshot")
    poses = {row["original"]["keyframe_id"]: row for row in data["poses"].values()
             if row.get("original", {}).get("map_id") == lineage}
    by_key = {row["keyframe_id"]: row for row in bindings}
    _require(set(poses) == set(by_key),
             "known memory anchors and provider bindings are not a closed set")
    old_nodes, new_nodes = _nodes(old), _nodes(new)
    updates, unchanged, missing = [], [], []
    for keyframe_id in sorted(poses):
        stored, binding = poses[keyframe_id], by_key[keyframe_id]
        _require(stored.get("pending") is False,
                 "restore_requires_explicit_pose_history")
        current = stored.get("current", {})
        node_id = binding["provider_node_id"]
        old_node = old_nodes.get(node_id)
        _require(old_node is not None and node_id not in old["explicit_missing_node_ids"],
                 "restore_requires_explicit_pose_history")
        _require(old_node["stamp_ns"] == binding["provider_node_stamp_ns"],
                 "from snapshot reused provider node ID with a different stamp")
        expected_old = (_rigid(old_node["T_map_node"], "old.T_map_node")
                        @ _rigid(binding["T_node_local"], "T_node_local")).tolist()
        original = stored.get("original", {})
        initial_node = _nodes(initial).get(node_id)
        expected_initial = (_rigid(initial_node["T_map_node"], "initial.T_map_node")
                            @ _rigid(binding["T_node_local"], "T_node_local")).tolist()
        _require(original.get("map_id") == lineage
                 and original.get("keyframe_id") == keyframe_id
                 and original.get("frame_id") == binding["local_frame_id"]
                 and original.get("version") == _pose_version(initial["snapshot_id"], node_id)
                 and canonical(original.get("T_map_local")) == canonical(expected_initial)
                 and _binding_ref_present(original, binding_ref),
                 "memory original pose does not match the immutable initial binding")
        version_snapshots = {row["snapshot_id"]: row for row in history}
        valid_versions = {_pose_version(snapshot_id, node_id): snapshot_id
                          for snapshot_id in version_snapshots}
        current_snapshot_id = valid_versions.get(current.get("version"))
        _require(current_snapshot_id is not None,
                 "memory current pose version is outside the trusted snapshot history")
        version_node = _nodes(version_snapshots[current_snapshot_id]).get(node_id)
        _require(version_node is not None
                 and version_node["stamp_ns"] == binding["provider_node_stamp_ns"],
                 "pose-version snapshot lacks the immutable bound node")
        expected_version_pose = (_rigid(version_node["T_map_node"], "version.T_map_node")
                                 @ _rigid(binding["T_node_local"], "T_node_local")).tolist()
        _require(current.get("map_id") == lineage
                 and current.get("keyframe_id") == keyframe_id
                 and current.get("frame_id") == binding["local_frame_id"]
                 and canonical(current.get("T_map_local")) == canonical(expected_old)
                 and canonical(current.get("T_map_local")) == canonical(expected_version_pose)
                 and _binding_ref_present(current, binding_ref),
                 "memory current pose does not match the bound from-snapshot pose")
        if node_id in new["explicit_missing_node_ids"]:
            missing.append(keyframe_id)
            continue
        new_node = new_nodes.get(node_id)
        _require(new_node is not None,
                 "snapshot omitted a bound node without explicit missing declaration")
        _require(new_node["stamp_ns"] == binding["provider_node_stamp_ns"],
                 "to snapshot reused provider node ID with a different stamp")
        if canonical(new_node["T_map_node"]) == canonical(old_node["T_map_node"]):
            unchanged.append(keyframe_id)
            continue
        matrix = (_rigid(new_node["T_map_node"], "new.T_map_node")
                  @ _rigid(binding["T_node_local"], "T_node_local"))
        updates.append({
            "map_id": lineage,
            "keyframe_id": keyframe_id,
            "version": _pose_version(new["snapshot_id"], node_id),
            "frame_id": binding["local_frame_id"],
            "T_map_local": matrix.tolist(),
            "source": "rtabmap_absolute_node_pose_revision",
            "evidence_refs": deepcopy(new["evidence_refs"]) + [binding_ref],
        })
    classified = set(unchanged) | set(missing) | {row["keyframe_id"] for row in updates}
    _require(classified == set(poses)
             and len(classified) == len(unchanged) + len(missing) + len(updates),
             "updated/unchanged/missing classification is not closed and disjoint")
    identity = {"map_lineage_id": lineage, "from": old["snapshot_id"],
                "to": new["snapshot_id"], "updated": [x["keyframe_id"] for x in updates],
                "unchanged": unchanged, "missing": missing}
    revision = {
        "schema": REVISION_SCHEMA,
        "revision_id": "rtabmap-" + digest(identity),
        "map_id": lineage,
        "from_version": old["snapshot_id"],
        "to_version": new["snapshot_id"],
        "reason": "ordered RTAB-Map absolute node-pose snapshot transition",
        "poses": updates,
        "unchanged_keyframes": unchanged,
    }
    return {"status": "revision", "revision": revision,
            "audit": {"updated": [x["keyframe_id"] for x in updates],
                      "unchanged": unchanged, "missing": missing,
                      "from_sequence": old["sequence"], "to_sequence": new["sequence"]}}
