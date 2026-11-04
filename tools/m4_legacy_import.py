"""Export saved TUM replay geometry to the private M4 offline input draft.

No inference, depth backprojection or old object identity is reused.  The local
anchor is the inverse-original-pose image of a SAVED world-coordinate summary,
not a newly computed camera-coordinate median or a RTAB-Map keyframe.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile

import numpy as np

from perception.fpga_replay.io import interpolate_pose, read_table, sha256


POSE_SOURCE = "TUM_mocap_interpolated_not_SLAM"
CALIBRATION = "TUM_registered_ROS_defaults_no_extra_depth_correction"
DETECTOR = "torchvision_ssdlite320_mobilenet_v3_large_COCO_V1"
OPTICAL_FRAME = "camera_optical_frame"


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _path_within(path, root):
    """``PurePath.is_relative_to`` for Python 3.8, the declared reference
    interpreter; the standard-library method only exists from Python 3.9.

    This is the CPython 3.9 implementation, so the semantics are identical: a
    purely lexical parts-prefix test that resolves no symlink and collapses no
    ``..``.  The caller resolves both sides first, as it must with the
    standard-library method.
    """
    try:
        Path(path).relative_to(root)
    except ValueError:
        return False
    return True


def _number(value, name):
    _require(not isinstance(value, bool) and isinstance(value, (int, float))
             and math.isfinite(value), f"{name}: finite number required")
    return float(value)


def _reference(path, selector=None):
    result = {"path": str(path.resolve()), "sha256": sha256(path)}
    if selector is not None:
        result["selector"] = selector
    return result


def _jsonl(path):
    rows = []
    for line_no, text in enumerate(path.read_text(encoding="utf8").splitlines(), 1):
        if not text.strip():
            continue
        try:
            row = json.loads(text, parse_constant=lambda value: (_ for _ in ()).throw(
                ValueError(f"non-finite JSON constant: {value}")))
        except ValueError as error:
            raise ValueError(f"{path}:{line_no}: {error}") from error
        _require(isinstance(row, dict), f"{path}:{line_no}: object required")
        rows.append((line_no, row))
    _require(bool(rows), f"empty replay file: {path}")
    return rows


def _table_index(path):
    rows, stamps = read_table(path)
    by_path = {}
    for row in rows:
        _require(len(row) == 2, f"invalid image table row in {path}")
        _require(row[1] not in by_path, f"duplicate image path in {path}")
        by_path[row[1]] = row
    return by_path, rows, stamps


def _image_reference(data, record, field, table):
    relative = record.get(field)
    _require(isinstance(relative, str) and relative in table,
             f"{field}: missing image or table provenance")
    path = (data / relative).resolve()
    _require(_path_within(path, data), f"{field}: image escapes dataset root")
    ref = _reference(path)
    _require(record.get(field + "_sha256") == ref["sha256"],
             f"{field}: original image checksum mismatch")
    timestamp = record["timestamp" if field == "rgb" else "depth_timestamp"]
    _require(_number(timestamp, field + " timestamp") == float(table[relative][0]),
             f"{field}: acquisition timestamp differs from source table")
    return ref


def _pose(record, truth_rows, truth_times):
    matrix = np.asarray(record.get("T_world_camera"), dtype=float)
    _require(matrix.shape == (4, 4) and np.isfinite(matrix).all(), "invalid saved pose")
    _require(np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-10), "invalid pose last row")
    rotation = matrix[:3, :3]
    _require(np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
             and np.isclose(np.linalg.det(rotation), 1, atol=1e-6), "invalid pose rotation")
    _require(type(record.get("pose_valid")) is bool, "pose_valid must be boolean")
    expected, valid = interpolate_pose(record["timestamp"], truth_rows, truth_times)
    _require(record["pose_valid"] == valid, "saved pose validity differs from groundtruth")
    if valid:
        _require(np.allclose(matrix, expected, atol=1e-9, rtol=0),
                 "saved pose differs from declared mocap interpolation")
    return matrix, valid


def build_document(replay, data):
    """Read, verify and adapt existing files; do not write or perform inference."""
    replay, data = Path(replay).resolve(), Path(data).resolve()
    input_path, events_path = replay / "input.jsonl", replay / "events.jsonl"
    input_rows, events = _jsonl(input_path), _jsonl(events_path)
    rgb_table, _, _ = _table_index(data / "rgb.txt")
    depth_table, _, _ = _table_index(data / "depth.txt")
    truth_rows, truth_times = read_table(data / "groundtruth.txt")
    refs = {name: _reference(path) for name, path in {
        "input": input_path, "events": events_path, "rgb_table": data / "rgb.txt",
        "depth_table": data / "depth.txt", "groundtruth": data / "groundtruth.txt",
        "run": replay / "run.json", "manifest": replay / "manifest.json",
    }.items()}
    original_run = json.loads((replay / "run.json").read_text())
    original_manifest = json.loads((replay / "manifest.json").read_text())
    for filename, reference_name in (("input.jsonl", "input"), ("events.jsonl", "events"), ("run.json", "run")):
        _require(original_manifest.get(filename) == refs[reference_name]["sha256"],
                 f"{filename}: saved artifact manifest mismatch")
    _require(original_run.get("mode") == "tum" and original_run.get("git_dirty") is False,
             "legacy run must declare frozen TUM software provenance")
    _require(isinstance(original_run.get("git_commit"), str) and len(original_run["git_commit"]) == 40,
             "legacy source commit required")
    identity = ":".join(refs[name]["sha256"] for name in
                        ("rgb_table", "depth_table", "groundtruth"))
    source_id = "tum_dataset:" + hashlib.sha256(identity.encode("ascii")).hexdigest()
    map_id = "tum_mocap:" + refs["groundtruth"]["sha256"]
    by_frame = {}
    for line, record in input_rows:
        frame_id = record.get("frame_id")
        _require(isinstance(frame_id, str) and frame_id, "missing acquisition frame ID")
        _require(frame_id not in by_frame, "duplicate input acquisition frame ID")
        by_frame[frame_id] = (line, record)
    _require(len(events) == len(input_rows), "input/event frame count mismatch")
    poses, batches, frame_reports, seen_frames = [], [], [], set()
    for event_line, event in events:
        frame_id = event.get("frame_id")
        _require(frame_id in by_frame and frame_id not in seen_frames,
                 "unknown or duplicate replay event acquisition")
        seen_frames.add(frame_id)
        input_line, record = by_frame[frame_id]
        timestamp = _number(record.get("timestamp"), "acquisition timestamp")
        _require(frame_id == rgb_table.get(record.get("rgb"), [None])[0],
                 "acquisition ID differs from TUM rgb table")
        _require(event.get("timestamp") == timestamp, "event/input timestamp mismatch")
        _require(record.get("pose_source") == event.get("pose_source") == POSE_SOURCE,
                 "undeclared or incompatible pose provenance")
        _require(record.get("calibration_id") == event.get("calibration_id") == CALIBRATION,
                 "undeclared or incompatible calibration provenance")
        _require(record.get("depth_kind") == "optical_z" and record.get("depth_scale") == 5000,
                 "unsupported legacy depth encoding or units")
        intrinsics = np.asarray(record.get("K"), dtype=float)
        _require(intrinsics.shape == (3, 3) and np.array_equal(intrinsics,
                 [[525, 0, 319.5], [0, 525, 239.5], [0, 0, 1]]),
                 "calibration label does not match saved legacy intrinsics")
        rgb_ref = _image_reference(data, record, "rgb", rgb_table)
        depth_ref = _image_reference(data, record, "depth", depth_table)
        _require(abs(timestamp - record["depth_timestamp"]) <= .02,
                 "RGB/depth acquisition mismatch exceeds original 20 ms policy")
        matrix, pose_valid = _pose(record, truth_rows, truth_times)
        acquisition_id = source_id + ":" + frame_id
        input_ref = dict(refs["input"], selector=f"line:{input_line}")
        event_ref = dict(refs["events"], selector=f"line:{event_line}")
        if pose_valid:
            poses.append({"map_id": map_id, "keyframe_id": acquisition_id,
                          "version": "original", "frame_id": OPTICAL_FRAME,
                          "T_map_local": matrix.tolist(), "source": POSE_SOURCE,
                          "evidence_refs": [input_ref, refs["groundtruth"]]})
        detections = event.get("detections")
        _require(isinstance(detections, list), "missing saved detections")
        for detection in detections:
            _require(detection.get("frame_id") == frame_id
                     and detection.get("timestamp") == timestamp
                     and detection.get("source") == DETECTOR,
                     "detection frame/time/model provenance mismatch")
            score = _number(detection.get("score"), "detector score")
            _require(0 <= score <= 1, "detector score out of range")
            bbox = np.asarray(detection.get("bbox"), dtype=float)
            _require(bbox.shape == (4,) and np.isfinite(bbox).all()
                     and bbox[2] > bbox[0] and bbox[3] > bbox[1], "invalid detection bbox")
            _require(isinstance(detection.get("label"), str) and detection["label"],
                     "missing detector label")
        strategy = event.get("strategies", {}).get("geometry")
        _require(isinstance(strategy, dict) and isinstance(strategy.get("tracks"), list),
                 "missing saved geometry strategy")
        _require(strategy.get("ignored") is None, "ignored replay event cannot supply new evidence")
        count = event.get("observation_count")
        _require(type(count) is int and 0 <= count <= len(detections),
                 "invalid saved observation count")
        _require(pose_valid or count == 0, "observations carry invalid original pose")
        candidates, unmatched, ambiguous = {}, 0, 0
        for track_index, track in enumerate(strategy["tracks"]):
            _require(isinstance(track.get("history"), list), "missing track history")
            for history_index, history in enumerate(track["history"]):
                if history.get("frame_id") != frame_id:
                    continue
                _require(history.get("timestamp") == timestamp, "history timestamp mismatch")
                matches = [index for index, detection in enumerate(detections)
                           if detection["bbox"] == history.get("bbox")
                           and detection["source"] == history.get("source")
                           and detection["label"] == track.get("label")
                           and detection["score"] == history.get("score")]
                if not matches:
                    unmatched += 1
                elif len(matches) != 1:
                    ambiguous += 1
                else:
                    candidates.setdefault(matches[0], []).append(
                        (history, track_index, history_index))
        observations = []
        for index, rows in sorted(candidates.items()):
            if len(rows) != 1:
                ambiguous += len(rows)
                continue
            history, track_index, history_index = rows[0]
            position = np.asarray(history.get("position_m"), dtype=float)
            _require(position.shape == (3,) and np.isfinite(position).all(),
                     "invalid saved world-coordinate summary")
            fraction = _number(history.get("valid_depth_fraction"), "valid depth fraction")
            _require(0 <= fraction <= 1, "valid depth fraction out of range")
            local = matrix[:3, :3].T @ (position - matrix[:3, 3])
            detection = detections[index]
            selector = (f"line:{event_line}/strategies/geometry/tracks/{track_index}"
                        f"/history/{history_index}")
            observations.append({
                "instance_id": index + 1, "instance_kind": "legacy_detection_index",
                "label": detection["label"], "score": detection["score"], "source": DETECTOR,
                "geometry": {"valid": True, "reason": None,
                    "position_local_m": local.tolist(), "keyframe_id": acquisition_id,
                    "pose_version": "original", "frame_id": OPTICAL_FRAME, "units": "m",
                    "summary_method": "inverse_original_pose_of_saved_world_median",
                    "valid_fraction": fraction},
                "evidence_refs": [dict(refs["events"], selector=selector), input_ref,
                                  rgb_ref, depth_ref, refs["rgb_table"], refs["depth_table"],
                                  refs["groundtruth"], refs["run"], refs["manifest"]],
            })
        _require(len(observations) <= count, "more recovered histories than original observations")
        batches.append({
            "map_id": map_id, "map_version": "original",
            "acquisition": {"id": acquisition_id, "source_id": source_id,
                "image_sha256": rgb_ref["sha256"], "timestamp": timestamp,
                "time_source": "TUM rgb.txt acquisition timestamp",
                "clock_domain": source_id, "calibration_id": CALIBRATION},
            "prediction_set": "legacy:" + DETECTOR,
            "status": "ok", "error": None, "observations": observations,
        })
        frame_reports.append({
            "acquisition_id": acquisition_id, "original_pose_valid": pose_valid,
            "saved_detections": len(detections), "saved_geometry_observations": count,
            "exported_observations": len(observations),
            "not_recoverable_observations": count - len(observations),
            "history_without_unique_detection": unmatched + ambiguous,
            "ambiguous_history_rows": ambiguous,
            "legacy_association_ambiguous": len(strategy.get("ambiguous", [])),
            "evidence_refs": [event_ref, input_ref],
        })
    return {
        "schema": "dev3.m4.input.v1",
        "maps": [{"map_id": map_id, "map_version": "original",
            "world_frame": "tum_mocap_world", "units": "m", "pose_source": POSE_SOURCE,
            "source_kind": "public_dataset", "source_ref": refs["groundtruth"],
            "calibration_id": CALIBRATION}],
        "poses": poses, "batches": batches,
        "adapter_report": {
            "source_files": refs, "source_id": source_id,
            "original_source_commit": original_run["git_commit"],
            "original_geometry_config": original_run.get("config"),
            "input_frames": len(input_rows), "frames": frame_reports,
            "exported_observations": sum(row["exported_observations"] for row in frame_reports),
            "not_recoverable_observations": sum(row["not_recoverable_observations"] for row in frame_reports),
            "scope": "public TUM RGB-D; saved legacy SSDLite geometry; mocap pose, not SLAM or M3 YOLO",
            "anchor_kind": "acquisition_frame_anchor_not_RTAB_Map_keyframe",
            "position_method": "inverse original mocap pose of saved world componentwise median; no point-set recomputation",
            "limitations": [
                "Only histories retained by the old geometry strategy are recoverable; missing or ambiguous observations are counted.",
                "Old track identity, state, color and association decisions are not inputs to new association.",
                "No new detection, depth backprojection, map construction, geometry accuracy or semantic truth validation.",
                "Original ROS-default intrinsics are retained; no mixing with RTAB-Map intrinsics or estimated maps.",
                "Coordinate names and this schema are private offline adapter conventions pending team agreement.",
            ],
        },
    }


def write_new(path, document):
    """Publish one complete file atomically, refusing any existing destination."""
    path = Path(path)
    payload = (json.dumps(document, ensure_ascii=False, allow_nan=False, indent=2) + "\n").encode("utf8")
    with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".m4-import-", delete=False) as stream:
        temporary = Path(stream.name)
        try:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
            os.link(temporary, path)
        finally:
            temporary.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True, help="new JSON file; parent must exist")
    args = parser.parse_args()
    try:
        document = build_document(args.replay, args.data)
        write_new(args.out, document)
    except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
        print(json.dumps({"ok": False, "error": str(error)}, ensure_ascii=False), file=sys.stderr)
        return 1
    report = document["adapter_report"]
    print(json.dumps({"ok": True, "out": str(args.out.resolve()), "frames": len(document["batches"]),
                      "exported_observations": report["exported_observations"],
                      "not_recoverable_observations": report["not_recoverable_observations"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
