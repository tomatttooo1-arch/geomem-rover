#!/usr/bin/env python3
"""Freeze a mapping-only SE3 reference and score a disjoint reload query."""

import argparse
from collections import Counter
from contextlib import closing
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path
import sqlite3

import numpy as np

from host.localization_gate import covariance_valid, pose_matrix, pose_valid, stamp_ns
from tools.rtabmap_eval import associate, evaluate, read_tum


OPTICAL_IN_TRACKED = pose_matrix({
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"x": -0.5, "y": 0.5, "z": -0.5, "w": 0.5},
})


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_json(path):
    return json.loads(Path(path).read_text())


def load_jsonl(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def manifest_stamp_ns(row):
    return int(Decimal(str(row["timestamp"])) * 1_000_000_000)


def manifest_identity(manifest):
    return [{
        "original_index": row["original_index"],
        "timestamp": row["timestamp"],
        "depth_timestamp": row["depth_timestamp"],
        "rgb_source": row["rgb_sync"]["source"],
        "depth_source": row["depth_sync"]["source"],
        "rgb_sha256": row["rgb_sync"]["input_sha256"],
        "depth_sha256": row["depth_sync"]["input_sha256"],
    } for row in manifest["pairs"]]


def identity_sha256(manifest):
    payload = json.dumps(manifest_identity(manifest), sort_keys=True,
                         separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


def validate_segment(manifest, role):
    if manifest.get("schema") != 1 or manifest.get("role") != role:
        raise ValueError(f"invalid {role} manifest identity")
    rows = manifest.get("pairs")
    start, stop = manifest.get("original_range", [None, None])
    if not isinstance(rows, list) or [row.get("original_index") for row in rows] != list(range(start, stop)):
        raise ValueError(f"non-contiguous {role} original indices")
    stamps = [manifest_stamp_ns(row) for row in rows]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError(f"non-increasing {role} timestamps")
    if identity_sha256(manifest) != manifest.get("pairs_identity_sha256"):
        raise ValueError(f"{role} pair identity mismatch")
    if manifest.get("groundtruth_available_to_slam") is not False:
        raise ValueError(f"{role} manifest does not exclude truth")
    calibration = manifest.get("calibration", {})
    if (calibration.get("source_sha256") != calibration.get("input_sha256")
            or len(calibration.get("input_sha256", "")) != 64):
        raise ValueError(f"{role} calibration identity is missing")
    return rows, stamps


def audit_split(mapping_manifest, query_manifest):
    mapping_rows, mapping_stamps = validate_segment(mapping_manifest, "mapping")
    query_rows, query_stamps = validate_segment(query_manifest, "query")
    split = mapping_manifest["split_index"]
    count = len(mapping_rows) + len(query_rows)
    if (split != int(count * 0.6) or mapping_manifest["original_range"] != [0, split]
            or query_manifest["original_range"] != [split, count]
            or query_manifest["split_index"] != split
            or mapping_manifest["source_manifest_sha256"] != query_manifest["source_manifest_sha256"]
            or mapping_manifest["calibration"] != query_manifest["calibration"]):
        raise ValueError("manifests do not implement the fixed 60/40 split")
    if mapping_stamps[-1] >= query_stamps[0]:
        raise ValueError("RGB time ranges overlap")
    mapping_depth = [int(Decimal(str(row["depth_timestamp"])) * 1_000_000_000)
                     for row in mapping_rows]
    query_depth = [int(Decimal(str(row["depth_timestamp"])) * 1_000_000_000)
                   for row in query_rows]
    if max(mapping_depth) >= min(query_depth):
        raise ValueError("depth time ranges overlap")
    overlaps = {}
    for folder in ("rgb_sync", "depth_sync"):
        left = {row[folder]["source"] for row in mapping_rows}
        right = {row[folder]["source"] for row in query_rows}
        overlaps[folder + "_source_paths"] = sorted(left & right)
    left_pairs = {(r["rgb_sync"]["input_sha256"], r["depth_sync"]["input_sha256"])
                  for r in mapping_rows}
    right_pairs = {(r["rgb_sync"]["input_sha256"], r["depth_sync"]["input_sha256"])
                   for r in query_rows}
    overlaps["paired_hashes"] = sorted(left_pairs & right_pairs)
    if any(overlaps.values()):
        raise ValueError("mapping/query input leakage detected")
    return {
        "source_pairs": count,
        "split_index": split,
        "mapping_pairs": len(mapping_rows),
        "query_pairs": len(query_rows),
        "mapping_original_range": [0, split],
        "query_original_range": [split, count],
        "mapping_timestamp_ns_range": [mapping_stamps[0], mapping_stamps[-1]],
        "query_timestamp_ns_range": [query_stamps[0], query_stamps[-1]],
        "rgb_boundary_gap_s": (query_stamps[0] - mapping_stamps[-1]) / 1e9,
        "overlaps": overlaps,
        "calibration_sha256": mapping_manifest["calibration"]["input_sha256"],
    }


def audit_calibration_files(mapping_manifest_path, query_manifest_path,
                            mapping_manifest, query_manifest):
    from tools.rtabmap_reload_coordinate_prepare import validate_calibration
    paths = [Path(mapping_manifest_path).parent / "rtabmap_calib.yaml",
             Path(query_manifest_path).parent / "rtabmap_calib.yaml"]
    expected = mapping_manifest["calibration"]["input_sha256"]
    hashes = [sha256(path) for path in paths]
    if hashes != [expected, expected] or query_manifest["calibration"]["input_sha256"] != expected:
        raise ValueError("mapping/query calibration file hash mismatch")
    validations = [validate_calibration(path) for path in paths]
    if validations[0] != validations[1] or validations[0] != mapping_manifest["calibration"]["validation"]:
        raise ValueError("mapping/query calibration values mismatch")
    return {"sha256": expected, "mapping_path": str(paths[0].resolve()),
            "query_path": str(paths[1].resolve()), "validation": validations[0]}


def sqlite_counts(database):
    uri = Path(database).resolve().as_uri() + "?mode=ro&immutable=1"
    with closing(sqlite3.connect(uri, uri=True)) as connection:
        tables = {row[0] for row in connection.execute(
            "SELECT name FROM sqlite_master WHERE type='table'")}
        required = {"Node", "Data", "Link", "Word", "Feature"}
        if not required <= tables:
            raise ValueError("database missing Node/Data tables")
        integrity = [row[0] for row in connection.execute("PRAGMA integrity_check")]
        if integrity != ["ok"]:
            raise ValueError("database integrity check failed")
        node_columns = {row[1] for row in connection.execute("PRAGMA table_info(Node)")}
        if not {"id", "stamp", "ground_truth_pose"} <= node_columns:
            raise ValueError("unexpected Node schema")
        nodes = list(connection.execute("SELECT id, stamp, ground_truth_pose FROM Node ORDER BY id"))
        counts = {table: int(connection.execute(f'SELECT COUNT(*) FROM "{table}"').fetchone()[0])
                  for table in ("Node", "Data", "Link", "Word", "Feature") if table in tables}
    blobs = [row[2] for row in nodes]
    blob_audit = {
        "null": sum(blob is None for blob in blobs),
        "all_zero": sum(blob is not None and not any(bytes(blob)) for blob in blobs),
        "nonzero": sum(blob is not None and any(bytes(blob)) for blob in blobs),
        "sizes": dict(sorted(Counter(len(bytes(blob)) for blob in blobs if blob is not None).items())),
    }
    return nodes, counts, blob_audit


def audit_database(database, mapping_manifest=None, query_manifest=None, require_nonempty=True):
    nodes, counts, blob_audit = sqlite_counts(database)
    if require_nonempty and not nodes:
        raise ValueError("mapping database is empty")
    if not require_nonempty and nodes:
        raise ValueError("empty control database contains nodes")
    if not require_nonempty and any(counts[table] for table in ("Node", "Data", "Link", "Word", "Feature")):
        raise ValueError("empty control database contains map content")
    if blob_audit["nonzero"]:
        raise ValueError("nonzero ground-truth pose blob in online database")
    result = {
        "database_sha256": sha256(database),
        "table_counts": counts,
        "prior_node_ids": [row[0] for row in nodes],
        "node_stamps_s": [float(row[1]) for row in nodes],
        "ground_truth_pose_blobs": blob_audit,
    }
    if nodes and mapping_manifest is not None and query_manifest is not None:
        node_times = np.asarray([float(row[1]) for row in nodes])
        map_times = np.asarray([float(row["timestamp"]) for row in mapping_manifest["pairs"]])
        query_times = np.asarray([float(row["timestamp"]) for row in query_manifest["pairs"]])
        mapping_matches = associate(node_times, map_times, 0.000002)
        query_matches = associate(node_times, query_times, 0.000002)
        if len(mapping_matches) != len(nodes) or query_matches:
            raise ValueError("database node stamp is outside mapping-only input")
        differences = [abs(node_times[i] - map_times[j]) for i, j in mapping_matches]
        result["node_stamp_audit"] = {
            "nodes": len(nodes),
            "mapping_matches": len(mapping_matches),
            "query_matches": len(query_matches),
            "mapping_match_max_difference_s": max(differences) if differences else None,
            "min_s": float(node_times.min()),
            "max_s": float(node_times.max()),
        }
    return result


def row_pose_matrix(row):
    q = np.asarray(row[4:8], dtype=float)
    if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1.0) > 0.001:
        raise ValueError("non-unit TUM quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ]
    matrix[:3, 3] = row[1:4]
    return matrix


def rotation_error_degrees(expected, observed):
    error = expected[:3, :3].T @ observed[:3, :3]
    return math.degrees(math.acos(float(np.clip((np.trace(error) - 1) / 2, -1, 1))))


def alignment_matrix(alignment):
    rotation = np.asarray(alignment["alignment_rotation"], dtype=float)
    translation = np.asarray(alignment["alignment_translation_m"], dtype=float)
    if (rotation.shape != (3, 3) or translation.shape != (3,)
            or not np.isfinite(rotation).all() or not np.isfinite(translation).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or abs(np.linalg.det(rotation) - 1) > 1e-6):
        raise ValueError("invalid fixed SE3 alignment")
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def fixed_pose_errors(estimated, truth, alignment, optical_in_tracked=None):
    matches = associate(estimated[:, 0], truth[:, 0], 0.02)
    transform = alignment_matrix(alignment)
    records = []
    for estimate_index, truth_index in matches:
        predicted = transform @ row_pose_matrix(estimated[estimate_index])
        if optical_in_tracked is not None:
            predicted = predicted @ optical_in_tracked
        expected = row_pose_matrix(truth[truth_index])
        records.append({
            "estimate_index": estimate_index,
            "truth_index": truth_index,
            "time_difference_s": abs(float(estimated[estimate_index, 0] - truth[truth_index, 0])),
            "translation_error_m": float(np.linalg.norm(predicted[:3, 3] - expected[:3, 3])),
            "rotation_error_deg": rotation_error_degrees(expected, predicted),
        })
    return records


def distribution(values):
    if not values:
        return {"samples": 0, "rmse": None, "median": None, "p90": None, "max": None}
    array = np.asarray(values, dtype=float)
    return {
        "samples": len(array),
        "rmse": float(np.sqrt(np.mean(array ** 2))),
        "median": float(np.median(array)),
        "p90": float(np.quantile(array, 0.9)),
        "max": float(array.max()),
    }


def freeze_reference(poses_path, truth_path, database_path, mapping_manifest_path,
                     query_manifest_path, code_commit, output_path):
    mapping_manifest, query_manifest = load_json(mapping_manifest_path), load_json(query_manifest_path)
    split = audit_split(mapping_manifest, query_manifest)
    calibration = audit_calibration_files(mapping_manifest_path, query_manifest_path,
                                          mapping_manifest, query_manifest)
    database = audit_database(database_path, mapping_manifest, query_manifest, True)
    poses, truth = read_tum(poses_path), read_tum(truth_path)
    node_times = np.asarray(database["node_stamps_s"], dtype=float)
    pose_node_matches = associate(poses[:, 0], node_times, 0.000002)
    if len(pose_node_matches) != len(poses) or len(pose_node_matches) != len(node_times):
        raise ValueError("exported camera poses do not exactly cover mapping database nodes")
    pose_node_differences = [abs(float(poses[i, 0] - node_times[j]))
                             for i, j in pose_node_matches]
    alignment = evaluate(poses_path, truth_path, 0.02)
    if alignment["matched_samples"] != len(poses):
        raise ValueError("not every mapping pose has a ground-truth association")
    if alignment["matched_samples"] < 3 or alignment["alignment_covariance_rank"] < 2:
        raise ValueError("mapping alignment is underconstrained")
    mapping_errors = fixed_pose_errors(poses, truth, alignment)
    reference = {
        "schema": 1,
        "protocol": "mapping-only frozen SE3; query trajectory never fitted",
        "code_commit": code_commit,
        "split": split,
        "calibration": calibration,
        "mapping_manifest": {"path": str(Path(mapping_manifest_path).resolve()),
                             "sha256": sha256(mapping_manifest_path)},
        "query_manifest": {"path": str(Path(query_manifest_path).resolve()),
                           "sha256": sha256(query_manifest_path)},
        "mapping_database": {"path": str(Path(database_path).resolve()),
                             "sha256": sha256(database_path)},
        "mapping_camera_poses": {"path": str(Path(poses_path).resolve()),
                                 "sha256": sha256(poses_path)},
        "ground_truth": {"path": str(Path(truth_path).resolve()), "sha256": sha256(truth_path)},
        "database_audit": database,
        "alignment": alignment,
        "mapping_pose_check": {
            "database_pose_stamp_identity": {
                "database_nodes": len(node_times),
                "exported_poses": len(poses),
                "matches_within_2us": len(pose_node_matches),
                "max_difference_s": max(pose_node_differences) if pose_node_differences else None,
            },
            "matches": len(mapping_errors),
            "translation_m": distribution([row["translation_error_m"] for row in mapping_errors]),
            "rotation_deg": distribution([row["rotation_error_deg"] for row in mapping_errors]),
            "records": mapping_errors,
        },
    }
    with Path(output_path).open("x") as stream:
        json.dump(reference, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return reference


def unique_messages(events, kind):
    rows = [row for row in events if row.get("event") == kind]
    counts = Counter(row.get("stamp_ns") for row in rows)
    duplicates = {stamp: count for stamp, count in counts.items() if count > 1}
    if duplicates:
        raise ValueError(f"duplicate {kind} stamps: {duplicates}")
    return {row["stamp_ns"]: row for row in rows}


def header_valid(row, frame, child=None):
    try:
        if stamp_ns(row) != row["stamp_ns"] or row["header"]["frame_id"] != frame:
            return False
        return child is None or row.get("child_frame_id") == child
    except (KeyError, TypeError, ValueError):
        return False


def approved_poses(gate_events, map_id):
    approved = []
    for event in gate_events:
        if event.get("map_id") != map_id:
            raise ValueError("gate event map identity mismatch")
        approved.extend(event.get("approved", []))
    counts = Counter(row.get("stamp_ns") for row in approved)
    if any(count > 1 for count in counts.values()):
        raise ValueError("duplicate approved stamp")
    return {row["stamp_ns"]: row for row in approved}


def score_records(events, query_manifest, truth, reference, prior_ids, approvals):
    publications = [row for row in events if row.get("event") == "publish_begin"]
    index_counts = Counter(row.get("index") for row in publications)
    stamp_counts = Counter(row.get("stamp_ns") for row in publications)
    if any(count > 1 for count in index_counts.values()) or any(count > 1 for count in stamp_counts.values()):
        raise ValueError("duplicate publication index or stamp")
    indices = [row["index"] for row in publications]
    stamps = [row["stamp_ns"] for row in publications]
    if any(b <= a for a, b in zip(indices, indices[1:])) or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("publications are not strictly ordered")
    pairs = query_manifest["pairs"]
    publications_by_index = {}
    for row in publications:
        index = row["index"]
        if type(index) is not int or not 0 <= index < len(pairs):
            raise ValueError("publication index outside query manifest")
        expected = pairs[index]
        if (row["stamp_ns"] != manifest_stamp_ns(expected)
                or row.get("timestamp") != expected["timestamp"]
                or row.get("source_rgb_sha256") != expected["rgb_sync"]["source_sha256"]
                or row.get("source_depth_sha256") != expected["depth_sync"]["source_sha256"]
                or row.get("input_sha256", {}).get("rgb_sync") != expected["rgb_sync"]["input_sha256"]
                or row.get("input_sha256", {}).get("depth_sync") != expected["depth_sync"]["input_sha256"]):
            raise ValueError(f"publication/query identity mismatch at local index {index}")
        publications_by_index[index] = row
    streams = {kind: unique_messages(events, kind)
               for kind in ("odom", "odom_info", "info", "localization_pose")}
    published_stamps = set(stamps)
    unknown = {kind: sorted(set(rows) - published_stamps) for kind, rows in streams.items()}
    local_rows = list(streams["localization_pose"].values())
    matches = associate(np.asarray([row["stamp_ns"] / 1e9 for row in local_rows]), truth[:, 0], 0.02)
    truth_by_stamp = {local_rows[i]["stamp_ns"]: truth[j] for i, j in matches}
    fixed = alignment_matrix(reference["alignment"])
    records = []
    for index, pair in enumerate(pairs):
        publication = publications_by_index.get(index)
        stamp = manifest_stamp_ns(pair)
        odom = streams["odom"].get(stamp)
        odom_info = streams["odom_info"].get(stamp)
        info = streams["info"].get(stamp)
        localization = streams["localization_pose"].get(stamp)
        links = [] if info is None else [info.get("loop_closure_id"), info.get("proximity_detection_id")]
        prior_link = any(type(link) is int and link in prior_ids for link in links)
        raw_positive_link = any(type(link) is int and link > 0 for link in links)
        odom_good = bool(odom and header_valid(odom, "odom", "camera_link")
                         and pose_valid(odom.get("pose", {}).get("pose", {}))
                         and covariance_valid(odom.get("pose", {}).get("covariance", [])))
        odom_info_good = bool(odom_info and header_valid(odom_info, "odom")
                              and odom_info.get("lost") is False
                              and covariance_valid(odom_info.get("covariance", [])))
        info_good = bool(info and header_valid(info, "map") and prior_link)
        localization_good = bool(localization and header_valid(localization, "map")
                                 and pose_valid(localization.get("pose", {}).get("pose", {}))
                                 and covariance_valid(localization.get("pose", {}).get("covariance", [])))
        translation_error = rotation_error = truth_dt = None
        if localization and pose_valid(localization.get("pose", {}).get("pose", {})) and stamp in truth_by_stamp:
            predicted = fixed @ pose_matrix(localization["pose"]["pose"]) @ OPTICAL_IN_TRACKED
            expected = row_pose_matrix(truth_by_stamp[stamp])
            translation_error = float(np.linalg.norm(predicted[:3, 3] - expected[:3, 3]))
            rotation_error = rotation_error_degrees(expected, predicted)
            truth_dt = abs(stamp / 1e9 - float(truth_by_stamp[stamp][0]))
        records.append({
            "local_index": index,
            "original_index": pair["original_index"],
            "stamp_ns": stamp,
            "published": publication is not None,
            "odom": odom is not None,
            "odom_info": odom_info is not None,
            "info": info is not None,
            "localization_pose": localization is not None,
            "raw_positive_link": raw_positive_link,
            "prior_link": prior_link,
            "numeric_pose": translation_error is not None,
            "healthy_prior_localization": bool(publication is not None and odom_good and odom_info_good
                                                and info_good and localization_good),
            "approved": stamp in approvals,
            "truth_time_difference_s": truth_dt,
            "translation_error_m": translation_error,
            "rotation_error_deg": rotation_error,
        })
    return records, unknown, streams


def metric_group(records, predicate):
    selected = [row for row in records if predicate(row)]
    scored = [row for row in selected if row["translation_error_m"] is not None]
    return {
        "selected": len(selected),
        "scored": len(scored),
        "unscored": len(selected) - len(scored),
        "translation_m": distribution([row["translation_error_m"] for row in scored]),
        "rotation_deg": distribution([row["rotation_error_deg"] for row in scored]),
        "local_indices": [row["local_index"] for row in scored],
        "original_indices": [row["original_index"] for row in scored],
    }


def first_three(records, require_approved=False):
    streak = []
    for row in records:
        good = (row["published"] and row["healthy_prior_localization"]
                and row["translation_error_m"] is not None
                and row["translation_error_m"] <= 0.15
                and (not require_approved or row["approved"]))
        streak = streak + [row] if good else []
        if len(streak) == 3:
            return {
                "local_indices": [item["local_index"] for item in streak],
                "original_indices": [item["original_index"] for item in streak],
            }
    return None


def verify_approval_poses(approvals, localizations, published_stamps):
    if set(approvals) - set(published_stamps):
        raise ValueError("approval stamp was not published")
    for stamp, approved in approvals.items():
        localization = localizations.get(stamp)
        if localization is None or approved.get("pose") != localization.get("pose"):
            raise ValueError("approved pose differs from raw localization pose")


def score_run(run_path, truth_path, reference_path, mapping_manifest_path,
              query_manifest_path, output_path, mode, expected_database_sha256=None):
    run_path = Path(run_path)
    reference = load_json(reference_path)
    mapping_manifest, query_manifest = load_json(mapping_manifest_path), load_json(query_manifest_path)
    split = audit_split(mapping_manifest, query_manifest)
    calibration = audit_calibration_files(mapping_manifest_path, query_manifest_path,
                                          mapping_manifest, query_manifest)
    if calibration["sha256"] != reference["calibration"]["sha256"]:
        raise ValueError("reference calibration hash mismatch")
    for label, path in (("mapping_manifest", mapping_manifest_path),
                        ("query_manifest", query_manifest_path), ("ground_truth", truth_path)):
        if sha256(path) != reference[label]["sha256"]:
            raise ValueError(f"reference {label} hash mismatch")
    truth = read_tum(truth_path)
    database_path = run_path / "map.db"
    runner = load_json(run_path / "runner-result.json")
    outer_command = load_json(run_path.parent / "docker-command.json")
    runner_host_identity = outer_command.get("host_identity", {})
    if (runner_host_identity.get("query_manifest_sha256") != sha256(query_manifest_path)
            or runner_host_identity.get("calibration_sha256") != reference["calibration"]["sha256"]
            or runner_host_identity.get("reference_identity_sha256") != sha256(reference_path)):
        raise ValueError("query runner host identity is not bound to reference/input/calibration")
    if mode == "normal":
        if sha256(database_path) != reference["mapping_database"]["sha256"]:
            raise ValueError("normal reload database/reference mismatch")
        if (runner.get("input_database_sha256") != reference["mapping_database"]["sha256"]
                or runner.get("database_sha256_after") != reference["mapping_database"]["sha256"]
                or runner.get("readonly_database_unchanged") is not True):
            raise ValueError("normal runner did not preserve the pinned database")
        database = audit_database(database_path, mapping_manifest, query_manifest, True)
    else:
        if not expected_database_sha256 or runner.get("input_database_sha256") != expected_database_sha256:
            raise ValueError("empty control source database identity mismatch")
        database = audit_database(database_path, require_nonempty=False)
    prior_ids = set(database["prior_node_ids"])
    events = load_jsonl(run_path / "observations.jsonl")
    gate_path = run_path / "gate-events.jsonl"
    if mode == "normal":
        if not gate_path.exists():
            raise ValueError("normal run is missing gate evidence")
        gate_events = load_jsonl(gate_path)
        approvals = approved_poses(gate_events, reference["mapping_database"]["sha256"])
    else:
        if gate_path.exists():
            raise ValueError("empty control must not start the prior gate")
        approvals = {}
        gate_events = []
    records, unknown, streams = score_records(events, query_manifest, truth, reference,
                                              prior_ids, approvals)
    if any(unknown.values()):
        raise ValueError(f"message stamp without a published query frame: {unknown}")
    raw_positive = sum(row["raw_positive_link"] for row in records)
    if mode == "empty" and raw_positive:
        raise ValueError("empty control emitted positive map links")
    verify_approval_poses(approvals, streams["localization_pose"],
                          [row["stamp_ns"] for row in records if row["published"]])
    publications = sum(row["published"] for row in records)
    approved_outside_published = [row["local_index"] for row in records
                                  if row["approved"] and not row["published"]]
    approved_outside_health = [row["local_index"] for row in records
                               if row["approved"] and not row["healthy_prior_localization"]]
    complete_events = [row for row in events if row.get("event") == "complete"]
    result = {
        "schema": 1,
        "protocol": "fixed mapping-only SE3; query GT one-to-one; map-camera_link times fixed camera-optical extrinsic",
        "mode": mode,
        "reference": {"path": str(Path(reference_path).resolve()), "sha256": sha256(reference_path)},
        "split": split,
        "database_audit": database,
        "runner_result": runner,
        "runner_host_identity": runner_host_identity,
        "denominators": {
            "expected_query_frames": len(records),
            "published": publications,
            "missing_publication_local_indices": [row["local_index"] for row in records if not row["published"]],
            "odom_messages": len(streams["odom"]),
            "odom_info_messages": len(streams["odom_info"]),
            "info_messages": len(streams["info"]),
            "localization_pose_messages": len(streams["localization_pose"]),
            "numeric_pose_with_truth": sum(row["numeric_pose"] for row in records),
            "healthy_prior_localization": sum(row["healthy_prior_localization"] for row in records),
            "raw_positive_link_frames": raw_positive,
            "prior_member_link_frames": sum(row["prior_link"] for row in records),
            "gate_started": gate_path.exists(),
            "gate_acquisitions": sum(row.get("event") == "acquisition" for row in gate_events),
            "gate_approved": len(approvals),
            "approved_outside_published_local_indices": approved_outside_published,
            "approved_outside_healthy_local_indices": approved_outside_health,
            "unknown_message_stamps": unknown,
        },
        "scores": {
            "raw_localization_outputs": metric_group(records, lambda row: row["localization_pose"]),
            "all_numeric_poses": metric_group(records, lambda row: row["numeric_pose"]),
            "healthy_prior_localizations": metric_group(records, lambda row: row["healthy_prior_localization"]),
            "gate_approved": metric_group(records, lambda row: row["approved"]),
        },
        "first_three_healthy_within_0_15m": first_three(records),
        "first_three_approved_within_0_15m": first_three(records, True),
        "completion": {
            "complete_events": complete_events,
            "one_complete_event": len(complete_events) == 1,
            "runner_replay_exit_zero": runner.get("replay_exit_code") == 0,
            "all_owned_processes_exited": runner.get("all_owned_processes_exited") is True,
            "all_query_frames_published": publications == len(records),
        },
        "records": records,
        "limitations": "Same recording, disjoint times. No rotation pass threshold. Not vehicle, ground, Nav2, or real-time acceptance.",
    }
    with Path(output_path).open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    freeze = sub.add_parser("freeze")
    freeze.add_argument("--poses", type=Path, required=True)
    freeze.add_argument("--truth", type=Path, required=True)
    freeze.add_argument("--database", type=Path, required=True)
    freeze.add_argument("--mapping-manifest", type=Path, required=True)
    freeze.add_argument("--query-manifest", type=Path, required=True)
    freeze.add_argument("--code-commit", required=True)
    freeze.add_argument("--output", type=Path, required=True)
    score = sub.add_parser("score")
    score.add_argument("--run", type=Path, required=True)
    score.add_argument("--truth", type=Path, required=True)
    score.add_argument("--reference", type=Path, required=True)
    score.add_argument("--mapping-manifest", type=Path, required=True)
    score.add_argument("--query-manifest", type=Path, required=True)
    score.add_argument("--output", type=Path, required=True)
    score.add_argument("--mode", choices=("normal", "empty"), required=True)
    score.add_argument("--expected-database-sha256")
    args = parser.parse_args()
    try:
        if args.command == "freeze":
            result = freeze_reference(args.poses, args.truth, args.database,
                args.mapping_manifest, args.query_manifest, args.code_commit, args.output)
            summary = {"database_sha256": result["mapping_database"]["sha256"],
                       "mapping_nodes": result["database_audit"]["table_counts"]["Node"],
                       "alignment": result["alignment"],
                       "mapping_pose_check": result["mapping_pose_check"]}
        else:
            result = score_run(args.run, args.truth, args.reference, args.mapping_manifest,
                args.query_manifest, args.output, args.mode, args.expected_database_sha256)
            summary = {"mode": result["mode"], "denominators": result["denominators"],
                       "scores": result["scores"],
                       "first_three_healthy_within_0_15m": result["first_three_healthy_within_0_15m"],
                       "first_three_approved_within_0_15m": result["first_three_approved_within_0_15m"]}
    except (OSError, ValueError, KeyError, TypeError, sqlite3.Error, np.linalg.LinAlgError,
            json.JSONDecodeError) as exc:
        parser.exit(2, f"reload coordinate evaluation failed: {exc}\n")
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
