#!/usr/bin/env python3
"""Build provenance-complete static M4 anchors from one frozen RTAB-Map map.

The probe is read-only with respect to every input.  It verifies all database
nodes against the mapping capture manifest, calibration and independently
exported camera poses.  It emits no object observations and no dynamic map
revision.
"""

import argparse
from copy import deepcopy
from decimal import Decimal
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import sqlite3
import struct
import subprocess
import sys
import time

import numpy as np

from host.rtabmap_revision_provider import (
    BINDING_SCHEMA, SNAPSHOT_SCHEMA, anchor_from_binding, seal_binding,
    seal_snapshot, snapshot_transition,
)


TRANSLATION_LIMIT_M = 1e-5
ROTATION_LIMIT_DEG = 1e-3
ROTATION_ORTHOGONALITY_LIMIT = 1e-6


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def stamp_ns(value):
    return int(Decimal(str(value)) * Decimal(1_000_000_000))


def matrix4(value):
    value = np.asarray(value, dtype=float)
    if value.shape == (3, 4):
        result = np.eye(4); result[:3] = value; value = result
    require(value.shape == (4, 4) and np.isfinite(value).all(), "invalid transform shape")
    return value


def quaternion_matrix(values):
    x, y, z, w = np.asarray(values, dtype=float)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    require(math.isfinite(norm) and norm > 0, "invalid export quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    result = np.eye(4)
    result[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    return result


def _project_rotation(value, name):
    rotation = np.asarray(value, dtype=float)
    require(rotation.shape == (3, 3) and np.isfinite(rotation).all(),
            "%s rotation must be finite 3x3" % name)
    orthogonality_fro = float(np.linalg.norm(rotation.T @ rotation - np.eye(3), ord="fro"))
    determinant = float(np.linalg.det(rotation))
    require(orthogonality_fro <= ROTATION_ORTHOGONALITY_LIMIT
            and determinant > 0 and abs(determinant - 1.0) <= ROTATION_ORTHOGONALITY_LIMIT,
            "%s rotation is reflective or too far from SO(3)" % name)
    left, _, right = np.linalg.svd(rotation)
    projected = left @ right
    require(np.linalg.det(projected) > 0, "%s SVD projection is reflective" % name)
    return projected, orthogonality_fro, determinant


def rotation_diagnostics(a, b):
    """Compare near-SO(3) rotations after projecting float serialization noise.

    The raw trace statistic is retained for audit, but it is not an angle when
    either input is non-orthogonal.  The reported angle is defined only on the
    two SVD-projected proper rotations.
    """
    raw_a = np.asarray(a, dtype=float)[:3, :3]
    raw_b = np.asarray(b, dtype=float)[:3, :3]
    projected_a, orthogonality_a, determinant_a = _project_rotation(raw_a, "actual")
    projected_b, orthogonality_b, determinant_b = _project_rotation(raw_b, "expected")
    raw_trace_statistic = float((np.trace(raw_a.T @ raw_b) - 1.0) / 2.0)
    relative = projected_a.T @ projected_b
    cosine = max(-1.0, min(1.0, float((np.trace(relative) - 1.0) / 2.0)))
    return {"projected_rotation_error_deg": math.degrees(math.acos(cosine)),
            "raw_trace_statistic_not_angle": raw_trace_statistic,
            "actual_orthogonality_fro": orthogonality_a,
            "expected_orthogonality_fro": orthogonality_b,
            "actual_determinant": determinant_a,
            "expected_determinant": determinant_b}


def evidence(role, path):
    path = Path(path).resolve()
    return {"role": role, "path": str(path), "sha256": sha256(path)}


def checked_input(path, expected, name):
    path = Path(path).resolve()
    require(path.is_file(), "%s is missing" % name)
    observed = sha256(path)
    require(observed == expected, "%s SHA256 mismatch" % name)
    return path


def load_calibration(path):
    import cv2
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    require(storage.isOpened(), "cannot open calibration")
    try:
        def read_matrix(name):
            node = storage.getNode(name)
            rows = int(node.getNode("rows").real())
            columns = int(node.getNode("cols").real())
            data = node.getNode("data")
            values = [data.at(i).real() for i in range(data.size())]
            require(rows > 0 and columns > 0 and len(values) == rows * columns,
                    "malformed calibration %s" % name)
            return np.asarray(values, dtype=float).reshape(rows, columns)
        result = {
            "camera_name": storage.getNode("camera_name").string(),
            "image_size": [int(storage.getNode("image_width").real()),
                           int(storage.getNode("image_height").real())],
            "camera_matrix": read_matrix("camera_matrix"),
            "T_node_local": matrix4(read_matrix("local_transform")),
        }
    finally:
        storage.release()
    require(result["camera_name"] and all(x > 0 for x in result["image_size"]),
            "invalid calibration identity or size")
    return result


def load_nodes(database):
    connection = sqlite3.connect("file:%s?mode=ro" % database, uri=True)
    try:
        rows = connection.execute("SELECT id, stamp, pose FROM Node ORDER BY id").fetchall()
    finally:
        connection.close()
    require(rows, "RTAB-Map database has no nodes")
    result = []
    for node_id, stamp, blob in rows:
        require(type(node_id) is int and node_id > 0 and isinstance(blob, bytes) and len(blob) == 48,
                "invalid RTAB-Map Node row")
        values = struct.unpack("<12f", blob)
        result.append({"node_id": node_id, "stamp_ns": stamp_ns(stamp),
                       "T_map_node": matrix4(np.asarray(values).reshape(3, 4)).tolist()})
    require([x["node_id"] for x in result] == sorted({x["node_id"] for x in result}),
            "database node IDs are duplicate or unordered")
    return result


def load_export(path):
    result = {}
    for line in Path(path).read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        values = list(map(float, line.split()))
        require(len(values) == 8, "malformed exported pose row")
        matrix = quaternion_matrix(values[4:8]); matrix[:3, 3] = values[1:4]
        stamp = stamp_ns(line.split()[0])
        require(stamp not in result, "duplicate exported pose stamp")
        result[stamp] = matrix
    require(result, "exported pose file is empty")
    return result


def load_manifest(path, calibration_path, calibration_hash):
    manifest = json.loads(Path(path).read_text())
    require(manifest.get("schema") == 1 and manifest.get("role") == "mapping",
            "manifest is not the frozen mapping input")
    calibration = manifest.get("calibration", {})
    require(calibration.get("input_sha256") == calibration_hash
            and calibration.get("source_sha256") == calibration_hash,
            "manifest calibration identity mismatch")
    pairs = manifest.get("pairs")
    require(isinstance(pairs, list) and pairs, "mapping manifest has no pairs")
    by_stamp = {}
    for row in pairs:
        stamp = stamp_ns(row["timestamp"])
        require(stamp not in by_stamp, "duplicate mapping capture stamp")
        by_stamp[stamp] = row
    require(str(Path(calibration_path).resolve()).endswith("rtabmap_calib.yaml"),
            "unexpected calibration filename")
    return manifest, by_stamp


def make_binding(snapshot, node, row, manifest_path, calibration_path, pose_epoch):
    rgb = Path(row["rgb_sync"]["source"]).resolve()
    depth = Path(row["depth_sync"]["source"]).resolve()
    require(sha256(rgb) == row["rgb_sync"]["source_sha256"]
            == row["rgb_sync"]["input_sha256"], "RGB source/input hash mismatch")
    require(sha256(depth) == row["depth_sync"]["source_sha256"]
            == row["depth_sync"]["input_sha256"], "depth source/input hash mismatch")
    calibration = snapshot["calibration"]
    value = {
        "schema": BINDING_SCHEMA,
        "keyframe_id": "tum-rgb:%s" % row["timestamp"],
        "source_id": "TUM_freiburg1_desk_mapping_split",
        "image_sha256": row["rgb_sync"]["input_sha256"],
        "depth_sha256": row["depth_sync"]["input_sha256"],
        "rgb_stamp_ns": stamp_ns(row["timestamp"]),
        "depth_stamp_ns": stamp_ns(row["depth_timestamp"]),
        "pose_epoch": pose_epoch,
        "map_lineage_id": snapshot["map_lineage_id"],
        "initial_snapshot_id": snapshot["snapshot_id"],
        "gate_map_id": snapshot["gate_map_id"],
        "provider_node_id": node["node_id"],
        "provider_node_stamp_ns": node["stamp_ns"],
        "node_frame": snapshot["node_frame"],
        "local_frame_id": calibration["local_frame_id"],
        "calibration_id": calibration["calibration_id"],
        "image_size": deepcopy(calibration["image_size"]),
        "camera_matrix": deepcopy(calibration["camera_matrix"]),
        "distortion_model": calibration["distortion_model"],
        "distortion_coefficients": deepcopy(calibration["distortion_coefficients"]),
        "T_node_local": deepcopy(calibration["T_node_local"]),
        "evidence_refs": [
            evidence("capture_manifest", manifest_path),
            evidence("rgb_source", rgb), evidence("depth_source", depth),
            evidence("calibration", calibration_path),
            next(deepcopy(r) for r in snapshot["evidence_refs"]
                 if r["role"] == "rtabmap_database"),
        ],
    }
    return seal_binding(value)


def negative_controls(snapshot, binding, binding_ref):
    controls = []
    for name, mutate in (
            ("wrong_node_stamp", lambda x: x.update(provider_node_stamp_ns=x["provider_node_stamp_ns"] + 1)),
            ("wrong_intrinsics", lambda x: x["camera_matrix"][0].__setitem__(0, x["camera_matrix"][0][0] + 1)),
            ("wrong_lineage", lambda x: x.update(map_lineage_id="wrong-lineage"))):
        candidate = deepcopy(binding); candidate.pop("binding_id")
        mutate(candidate); candidate = seal_binding(candidate)
        try:
            anchor_from_binding(snapshot, candidate, binding_ref)
        except ValueError as exc:
            controls.append({"name": name, "rejected": True, "reason": str(exc)})
        else:
            controls.append({"name": name, "rejected": False, "reason": None})
    require(all(row["rejected"] for row in controls), "negative control was accepted")
    return controls


def run(args):
    started = time.time()
    checkout = Path(__file__).resolve().parents[1]
    source_commit = subprocess.run(["git", "rev-parse", args.source_commit + "^{commit}"],
                                   cwd=checkout, check=True, text=True,
                                   stdout=subprocess.PIPE).stdout.strip()
    require(source_commit == args.source_commit,
            "source_commit must be a full commit SHA")
    source_files = {}
    for relative in ("host/rtabmap_revision_provider.py",
                     "tools/rtabmap_memory_anchor_probe.py"):
        current = (checkout / relative).read_bytes()
        frozen = subprocess.run(["git", "show", "%s:%s" % (source_commit, relative)],
                                cwd=checkout, check=True,
                                stdout=subprocess.PIPE).stdout
        source_files[relative] = {"sha256": hashlib.sha256(current).hexdigest(),
                                  "fixed_sha256": hashlib.sha256(frozen).hexdigest(),
                                  "matches_source_commit": current == frozen}
    require(all(row["matches_source_commit"] for row in source_files.values()),
            "checked-out provider files do not match source_commit")
    checked_out_head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=checkout,
                                      check=True, text=True,
                                      stdout=subprocess.PIPE).stdout.strip()
    git_status = subprocess.run(["git", "status", "--porcelain=v1", "-uall"],
                                cwd=checkout, check=True, text=True,
                                stdout=subprocess.PIPE).stdout.splitlines()
    output = args.output.resolve()
    require(not output.exists(), "output directory already exists")
    database = checked_input(args.database, args.expected_database_sha256, "database")
    poses_path = checked_input(args.poses, args.expected_poses_sha256, "exported poses")
    manifest_path = checked_input(args.manifest, args.expected_manifest_sha256, "manifest")
    calibration_path = checked_input(args.calibration, args.expected_calibration_sha256, "calibration")
    calibration_raw = load_calibration(calibration_path)
    manifest, pairs = load_manifest(manifest_path, calibration_path,
                                    args.expected_calibration_sha256)
    validation = manifest["calibration"].get("validation", {})
    require(validation.get("image_size") == calibration_raw["image_size"]
            and validation.get("camera_matrix") == calibration_raw["camera_matrix"].tolist()
            and validation.get("local_transform") == calibration_raw["T_node_local"][:3].tolist(),
            "manifest calibration values differ from calibration file")
    nodes = load_nodes(database)
    require(set(row["stamp_ns"] for row in nodes).issubset(pairs),
            "database node stamp is absent from mapping captures")
    refs = [evidence("rtabmap_database", database),
            evidence("optimized_camera_poses", poses_path),
            evidence("capture_manifest", manifest_path),
            evidence("calibration", calibration_path)]
    raw_snapshot = {
        "schema": SNAPSHOT_SCHEMA,
        "map_lineage_id": args.lineage_id,
        "parent_snapshot_id": None,
        "sequence": 1,
        "provider_instance_id": args.provider_instance_id,
        "provider_version": args.provider_version,
        "snapshot_kind": "full",
        "gate_map_id": args.expected_database_sha256,
        "world_frame": "map",
        "node_frame": "camera_link",
        "calibration": {
            "calibration_id": args.expected_calibration_sha256,
            "image_size": calibration_raw["image_size"],
            "camera_matrix": calibration_raw["camera_matrix"].tolist(),
            "distortion_model": "pre_registered_no_coefficients_in_calibration",
            "distortion_coefficients": [],
            "local_frame_id": "camera_optical_frame",
            "T_node_local": calibration_raw["T_node_local"].tolist(),
        },
        "nodes": nodes,
        "explicit_missing_node_ids": [],
        "evidence_refs": refs,
    }
    snapshot = seal_snapshot(raw_snapshot)
    bindings = [make_binding(snapshot, node, pairs[node["stamp_ns"]], manifest_path,
                             calibration_path, args.pose_epoch) for node in nodes]
    output.mkdir(parents=True)
    snapshot_path = output / "snapshot.json"
    snapshot_path.write_text(json.dumps(snapshot, indent=2, allow_nan=False) + "\n")
    bindings_doc = {"schema": "fpga.rtabmap.capture-bindings.v1",
                    "snapshot_id": snapshot["snapshot_id"], "bindings": bindings}
    bindings_path = output / "bindings.json"
    bindings_path.write_text(json.dumps(bindings_doc, indent=2, allow_nan=False) + "\n")
    binding_ref = evidence("capture_bindings", bindings_path)
    anchors = [{"keyframe_id": binding["keyframe_id"],
                "provider_node_id": binding["provider_node_id"],
                "anchor": anchor_from_binding(snapshot, binding, binding_ref)}
               for binding in bindings]
    anchors_path = output / "anchors.json"
    anchors_path.write_text(json.dumps({"schema": "fpga.rtabmap.static-anchors.v1",
                                        "snapshot_id": snapshot["snapshot_id"],
                                        "binding_ref": binding_ref,
                                        "anchors": anchors}, indent=2, allow_nan=False) + "\n")
    exported = load_export(poses_path)
    require(set(exported) == {x["stamp_ns"] for x in nodes},
            "exported pose stamps do not equal database node stamps")
    residuals = []
    by_node = {row["provider_node_id"]: row for row in anchors}
    for node in nodes:
        actual = np.asarray(by_node[node["node_id"]]["anchor"]["T_map_local"])
        expected = exported[node["stamp_ns"]]
        rotation = rotation_diagnostics(actual, expected)
        residuals.append({"provider_node_id": node["node_id"], "stamp_ns": node["stamp_ns"],
                          "translation_m": float(np.linalg.norm(actual[:3, 3] - expected[:3, 3])),
                          **rotation})
    max_translation = max(x["translation_m"] for x in residuals)
    max_rotation = max(x["projected_rotation_error_deg"] for x in residuals)
    require(max_translation <= TRANSLATION_LIMIT_M and max_rotation <= ROTATION_LIMIT_DEG,
            "static anchor differs from frozen export beyond preregistered limits")
    controls = negative_controls(snapshot, bindings[0], binding_ref)
    duplicate = snapshot_transition(snapshot, deepcopy(snapshot))
    require(duplicate["status"] == "duplicate_snapshot", "static duplicate was not idempotent")
    summary = {
        "schema": 1,
        "scope": "real recorded RTAB-Map static anchors; no dynamic revision or objects",
        "source_commit": args.source_commit,
        "snapshot_id": snapshot["snapshot_id"],
        "map_lineage_id": snapshot["map_lineage_id"],
        "gate_map_id": snapshot["gate_map_id"],
        "snapshot_kind": snapshot["snapshot_kind"],
        "nodes": len(nodes), "bindings": len(bindings), "anchors": len(anchors),
        "capture_stamps_one_to_one": len({x["stamp_ns"] for x in nodes}) == len(nodes),
        "export_stamps_equal_nodes": True,
        "translation_limit_m": TRANSLATION_LIMIT_M,
        "rotation_limit_deg": ROTATION_LIMIT_DEG,
        "max_translation_residual_m": max_translation,
        "max_rotation_residual_deg": max_rotation,
        "rotation_metric": "angle between SVD-projected proper rotations",
        "rotation_orthogonality_limit_fro": ROTATION_ORTHOGONALITY_LIMIT,
        "max_actual_rotation_orthogonality_fro":
            max(x["actual_orthogonality_fro"] for x in residuals),
        "max_expected_rotation_orthogonality_fro":
            max(x["expected_orthogonality_fro"] for x in residuals),
        "raw_trace_statistic_not_angle_range": [
            min(x["raw_trace_statistic_not_angle"] for x in residuals),
            max(x["raw_trace_statistic_not_angle"] for x in residuals)],
        "negative_controls": controls,
        "duplicate_snapshot_status": duplicate["status"],
        "dynamic_revision_produced": False,
        "all_checks_passed": True,
        "residuals": residuals,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    import cv2
    run_info = {
        "schema": 1, "source_commit": source_commit,
        "checked_out_head": checked_out_head, "source_files": source_files,
        "argv": sys.argv,
        "started_unix_s": started, "finished_unix_s": time.time(),
        "python_executable": sys.executable, "python": platform.python_version(),
        "numpy": np.__version__, "opencv": cv2.__version__,
        "sqlite": sqlite3.sqlite_version, "pid": os.getpid(),
        "git_status": git_status,
        "inputs": {ref["role"]: {"path": ref["path"], "sha256": ref["sha256"]} for ref in refs},
        "provider_source_sha256": sha256(Path(__file__)),
    }
    (output / "run-info.json").write_text(json.dumps(run_info, indent=2, allow_nan=False) + "\n")
    files = ["snapshot.json", "bindings.json", "anchors.json", "summary.json", "run-info.json"]
    with (output / "MANIFEST.sha256").open("x") as stream:
        for name in files:
            stream.write("%s  %s\n" % (sha256(output / name), name))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", required=True, type=Path)
    parser.add_argument("--poses", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--calibration", required=True, type=Path)
    parser.add_argument("--expected-database-sha256", required=True)
    parser.add_argument("--expected-poses-sha256", required=True)
    parser.add_argument("--expected-manifest-sha256", required=True)
    parser.add_argument("--expected-calibration-sha256", required=True)
    parser.add_argument("--lineage-id", required=True)
    parser.add_argument("--provider-instance-id", required=True)
    parser.add_argument("--provider-version", default="rtabmap-0.23.7")
    parser.add_argument("--pose-epoch", default="static-anchor-probe-not-gate-revision")
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        result = run(args)
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError,
            sqlite3.DatabaseError, struct.error) as exc:
        parser.exit(2, "static anchor probe failed: %s\n" % exc)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
