"""Frozen M3 instance masks + registered TUM depth -> dev3 M4 offline input.

Only the pinned 2026-09-16 n-seg batch is accepted. No detector is run, and
TUM motion-capture poses are reference coordinates, not estimated localization.
"""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

import cv2
import numpy as np

from perception.fpga_replay.core import Config, backproject
from perception.fpga_replay.io import interpolate_pose, load_frame, read_table, sha256


MANIFEST_SHA256 = "86cf65e5d3a8c483d45abefe1478986530185456a836342bd58674810e2342b7"
INPUTS_SHA256 = "3e8f3c96a3e8e92cb9e8a96fa2ced1b995faeb8bfce6d76fe15a2326621f49b5"
M3_COMMIT = "c895e5af2be2421a014ced6a9443ea29cf82858b"
MODEL_SHA256 = "a7cd8f929e1903d78a12a48efecab430209f18dc46cb96c3599a5980c63c423c"
MAP_ID = "TUM_freiburg1_desk_mocap_reference"
MAP_VERSION = "m3_20260916_tum_mocap_v1"
OPTICAL_FRAME = "tum_rgb_camera_optical"
POSE_SOURCE = "TUM_mocap_interpolated_not_SLAM"
CALIBRATION = "TUM_registered_ROS_defaults_no_extra_depth_correction"
K = [[525, 0, 319.5], [0, 525, 239.5], [0, 0, 1]]


def require(ok, message):
    if not ok:
        raise ValueError(message)


def path_within(path, root):
    """``PurePath.is_relative_to`` for Python 3.8, the declared reference
    interpreter; the standard-library method only exists from Python 3.9.

    This is the CPython 3.9 implementation of ``is_relative_to``, so it keeps
    the exact semantics: a purely lexical parts-prefix test that touches no
    filesystem, does not resolve symlinks and does not collapse ``..``, and
    that answers ``False`` (rather than raising) for a relative/absolute mix.
    Callers that need symlinks and ``..`` resolved must ``resolve()`` both
    sides first, exactly as they must with the standard-library method.
    """
    try:
        Path(path).relative_to(root)
    except ValueError:
        return False
    return True


def reference(path, selector=None):
    ref = {"path": str(path.resolve()), "sha256": sha256(path)}
    if selector is not None:
        ref["selector"] = selector
    return ref


def frozen_manifest(path, expected_sha=MANIFEST_SHA256):
    """The pinned hash authenticates each result AND its exact mask path."""
    path = Path(path).resolve()
    require(sha256(path) == expected_sha, "frozen M3 manifest SHA256 mismatch")
    root = path.parent / "m3-batch-20260916"
    entries = {}
    for line in path.read_text(encoding="utf8").splitlines():
        if line.startswith("#") or not line.strip():
            continue
        match = re.fullmatch(r"([0-9a-f]{64})  \./([^\s]+)", line)
        require(match is not None, "invalid frozen M3 manifest row")
        digest, relative = match.groups()
        target = (root / relative).resolve()
        require(path_within(target, root) and relative not in entries,
                "unsafe or duplicate frozen M3 manifest path")
        require(sha256(target) == digest, f"frozen M3 file SHA256 mismatch: {relative}")
        entries[relative] = digest
    require(bool(entries), "empty frozen M3 manifest")
    return entries


def manifest_file(root, entries, relative):
    require(relative in entries, f"file not pinned in M3 manifest: {relative}")
    path = (root / relative).resolve()
    require(path_within(path, root), "M3 path escapes frozen source")
    return path


def table(path):
    rows, stamps = read_table(path)
    require(all(len(row) == 2 for row in rows), f"invalid TUM image table: {path}")
    require(len({row[1] for row in rows}) == len(rows), f"duplicate TUM table path: {path}")
    return {row[1]: row[0] for row in rows}, rows, stamps


def tum_record(image, selected, data, rgb_table, depth_rows, depth_times,
               truth_rows, truth_times, used_depth):
    rgb_path = Path(image["source_path"]).resolve()
    require(path_within(rgb_path, data) and rgb_path.parent == data / "rgb",
            "M3 RGB source is outside the TUM RGB directory")
    relative = rgb_path.relative_to(data).as_posix()
    stamp_text = selected["timestamp_text"]
    require(relative in rgb_table and rgb_table[relative] == stamp_text,
            "M3 RGB image / TUM rgb.txt acquisition row mismatch")
    require(Path(selected["source_path"]).resolve() == rgb_path
            and selected["sha256"] == image["sha256"] == sha256(rgb_path),
            "M3 input selection / RGB content mismatch")
    require(image["frame_id"] == "sha256:" + image["sha256"],
            "M3 content identifier does not match image hash")
    stamp = float(stamp_text)
    require(image["acquisition_timestamp"] == stamp, "M3 acquisition time mismatch")
    source = image["timestamp_source"]
    require(Path(source["path"]).resolve() == data / "rgb.txt"
            and source["sha256"] == sha256(data / "rgb.txt")
            and source["row"] == f"{stamp_text} {relative}",
            "M3 acquisition timestamp source mismatch")
    index = int(np.argmin(np.abs(depth_times - stamp)))
    has_depth = index not in used_depth and abs(depth_times[index] - stamp) <= 0.02
    if has_depth:
        used_depth.add(index)
    depth_path = depth_rows[index][1] if has_depth else None
    if has_depth:
        require(depth_path.startswith("depth/")
                and path_within((data / depth_path).resolve(), data),
                "invalid depth table path")
    truth, pose_valid = interpolate_pose(stamp, truth_rows, truth_times)
    record = {"frame_id": "tum-rgb:" + stamp_text, "timestamp": stamp,
              "depth_timestamp": float(depth_times[index]) if has_depth else None, "rgb": relative,
              "depth": depth_path, "depth_scale": 5000.0,
              "depth_kind": "optical_z", "K": K,
              "T_world_camera": truth.tolist(), "pose_valid": pose_valid,
              "pose_source": POSE_SOURCE, "calibration_id": CALIBRATION,
              "rgb_sha256": image["sha256"],
              "depth_sha256": sha256(data / depth_path) if has_depth else None}
    # A missing pose is never exported: io.interpolate_pose's identity return
    # is only an internal validity sentinel, never a fabricated world pose.
    return record, pose_valid


def mask_geometry(mask, frame, pose_valid):
    pixels = int(np.count_nonzero(mask))
    if frame is None:
        return {"valid": False, "reason": "missing_aligned_TUM_depth",
                "mask_pixels": pixels, "valid_depth_pixels": 0}
    if not pose_valid:
        return {"valid": False, "reason": "missing_TUM_mocap_reference_pose",
                "mask_pixels": pixels, "valid_depth_pixels": 0}
    depth = frame.depth[mask]
    valid = np.isfinite(depth) & (depth > 0.1) & (depth < 10)
    count = int(valid.sum())
    fraction = count / pixels
    if count < Config().min_pixels:
        return {"valid": False,
                "reason": "no_valid_depth" if count == 0 else "insufficient_valid_depth_pixels",
                "mask_pixels": pixels, "valid_depth_pixels": count,
                "valid_fraction": fraction}
    v, u = np.where(mask)
    # backproject's identity maps optical rays to the SAME local camera frame.
    # The only world transform is the separately sourced TUM mocap pose below.
    points = backproject(np.c_[u[valid], v[valid]], depth[valid], frame.K, np.eye(4))
    return {"valid": True, "reason": None,
            "position_local_m": np.median(points, axis=0).tolist(),
            "keyframe_id": frame.frame_id, "pose_version": "tum-mocap-v1",
            "frame_id": OPTICAL_FRAME, "units": "m",
            "summary_method": "median_of_valid_instance_mask_backprojected_local_points",
            "valid_fraction": fraction, "mask_pixels": pixels,
            "valid_depth_pixels": count}


def read_instance_mask(det, ordinal, directory, root, entries, expected_shape):
    instance = det["instance_id"]
    relative = det["mask_path"]
    require(instance == ordinal and det.get("library_index", ordinal - 1) == ordinal - 1
            and relative == f"masks/instance-{ordinal:03d}.png",
            "M3 result instance/mask ownership mismatch")
    mask_path = manifest_file(root, entries, f"{directory}/{relative}")
    # Pinning the exact bytes catches a same-size / same-pixel-count swap.
    require(sha256(mask_path) == entries[f"{directory}/{relative}"],
            "M3 mask SHA256 does not match its frame/instance manifest entry")
    mask = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
    require(mask is not None and mask.dtype == np.uint8
            and mask.shape == expected_shape == tuple(det["mask_shape_hw"]),
            "M3 mask/depth raster or dtype mismatch")
    require(np.isin(mask, [0, 255]).all() and np.count_nonzero(mask) == det["foreground_pixels"]
            and det["foreground_pixels"] > 0, "M3 binary mask/pixel count mismatch")
    x1, y1, x2, y2 = det["bbox_xyxy"]
    ys, xs = np.where(mask == 255)
    require(xs.min() >= x1 - 1 and xs.max() <= x2 + 1
            and ys.min() >= y1 - 1 and ys.max() <= y2 + 1,
            "M3 mask outside paired bounding box")
    return mask == 255, mask_path


def build_document(batch, manifest, data, *, expected_manifest_sha=MANIFEST_SHA256,
                   expected_inputs_sha=INPUTS_SHA256, expected_commit=M3_COMMIT,
                   expected_model_sha=MODEL_SHA256):
    batch, manifest, data = Path(batch).resolve(), Path(manifest).resolve(), Path(data).resolve()
    require(batch == manifest.parent / "m3-batch-20260916" / "batch",
            "M3 batch must belong to frozen manifest root")
    entries = frozen_manifest(manifest, expected_manifest_sha)
    info_path = manifest_file(batch.parent, entries, "batch/batch_info.json")
    info = json.loads(info_path.read_text())
    selected_path = manifest_file(batch.parent, entries, "batch/inputs.json")
    require(sha256(selected_path) == expected_inputs_sha == info["manifest_sha256"],
            "frozen M3 input selection SHA256 mismatch")
    selection = json.loads(selected_path.read_text())
    require(sha256(data / "rgb.txt") == selection["tum_rgb_table_sha256"],
            "TUM RGB table SHA256 mismatch")
    require(info["input_count"] == len(selection["inputs"]),
            "M3 batch/input count mismatch")
    rgb_table, _, _ = table(data / "rgb.txt")
    _, depth_rows, depth_times = table(data / "depth.txt")
    truth_rows, truth_times = read_table(data / "groundtruth.txt")
    require(all(len(row) == 8 for row in truth_rows), "invalid TUM mocap pose table")
    refs = [reference(manifest), reference(selected_path), reference(info_path),
            *(reference(data / name) for name in ("rgb.txt", "depth.txt", "groundtruth.txt"))]
    map_row = {"map_id": MAP_ID, "map_version": MAP_VERSION, "world_frame": "TUM_mocap_world",
               "units": "m", "pose_source": POSE_SOURCE, "source_kind": "TUM_mocap_reference",
               "source_ref": refs[-1], "calibration_id": CALIBRATION}
    poses, batches, inventory = [], [], []
    used_depth = set()
    for number, selected in enumerate(selection["inputs"], 1):
        directory = f"batch/image-{number:03d}"
        result_path = manifest_file(batch.parent, entries, f"{directory}/result.json")
        run_path = manifest_file(batch.parent, entries, f"{directory}/run_info.json")
        result, run = json.loads(result_path.read_text()), json.loads(run_path.read_text())
        require(run["code_commit"] == expected_commit
                and run["model"]["sha256"] == expected_model_sha
                and run["parameters"]["device"] == "cpu", "M3 code/model/run configuration mismatch")
        image = result["image"]
        record, pose_valid = tum_record(image, selected, data, rgb_table,
                                        depth_rows, depth_times, truth_rows, truth_times, used_depth)
        frame = None
        if record["depth"] is not None:
            raw_depth = cv2.imread(str(data / record["depth"]), cv2.IMREAD_UNCHANGED)
            require(raw_depth is not None and raw_depth.dtype == np.uint16,
                    "TUM depth must be registered uint16 PNG with /5000 metres")
            frame = load_frame(record, data)
            require([frame.depth.shape[1], frame.depth.shape[0]] == [image["width"], image["height"]],
                    "RGB/mask/depth alignment or dimensions mismatch")
        if pose_valid:
            poses.append({"map_id": MAP_ID, "keyframe_id": record["frame_id"],
                          "version": "tum-mocap-v1", "frame_id": OPTICAL_FRAME,
                          "T_map_local": record["T_world_camera"], "source": POSE_SOURCE,
                          "evidence_refs": [refs[-1], reference(data / "rgb.txt", image["timestamp_source"]["row"])]})
        acquire = {"id": record["frame_id"], "source_id": "TUM_freiburg1_desk_original_recording",
                   "image_sha256": image["sha256"], "timestamp": record["timestamp"],
                   "time_source": "TUM_rgb.txt:" + image["timestamp_source"]["row"],
                   "clock_domain": "TUM_original_RGB_timestamp_seconds",
                   "calibration_id": CALIBRATION}
        evidence = [reference(result_path), reference(run_path), reference(data / record["rgb"]),
                    *([reference(data / record["depth"])] if frame is not None else []), *refs]
        observations = []
        for ordinal, det in enumerate(result.get("detections", []), 1):
            instance = det["instance_id"]
            foreground, mask_path = read_instance_mask(det, ordinal, directory, batch.parent,
                                                        entries, (image["height"], image["width"]))
            require(frame is None or foreground.shape == frame.depth.shape,
                    "M3 mask/depth alignment mismatch")
            observations.append({"instance_id": instance, "instance_kind": "M3_single_frame_instance_index",
                                 "label": det["class_name"].lower().strip(),
                                 "score": det["confidence"], "source": "frozen_M3_yolov8n_seg",
                                 "geometry": mask_geometry(foreground, frame, pose_valid),
                                 "evidence_refs": [*evidence, reference(mask_path, f"instance_id:{instance}")]})
        status = "ok" if result["ok"] else "failed"
        require(status == "ok" or not observations, "failed M3 result carries usable instances")
        batches.append({"map_id": MAP_ID, "map_version": MAP_VERSION,
                        "acquisition": acquire, "prediction_set": "M3_c895e5a_yolov8n_seg_a7cd8f9",
                        "status": status,
                        "error": None if status == "ok" else json.dumps(result.get("error"), sort_keys=True),
                        "observations": observations})
        inventory.append({"acquisition_id": record["frame_id"],
                          "image_content_sha256": image["sha256"],
                          "frame_id": OPTICAL_FRAME, "rgb": reference(data / record["rgb"]),
                          "depth": reference(data / record["depth"]) if frame is not None else None,
                          "depth_time": record["depth_timestamp"], "rgb_time": record["timestamp"],
                          "reference_pose_valid": pose_valid,
                          "result": reference(result_path), "run_info": reference(run_path),
                          "masks": [o["evidence_refs"][-1] for o in observations]})
    return {"schema": "dev3.m4.input.v1", "maps": [map_row], "poses": poses, "batches": batches}, inventory


def write_new(path, value):
    with path.open("x", encoding="utf8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def run(args):
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    write_new(out / "run-info.json", {"source_commit": commit, "python": sys.version,
              "argv": sys.argv, "batch": str(args.batch.resolve()),
              "manifest": str(args.manifest.resolve()), "data": str(args.data.resolve()),
              "threads": {key: os.environ.get(key) for key in
                          ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")}})
    try:
        require(not subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True).strip(),
                "commit source and tests before running the integration")
        document, inventory = build_document(args.batch, args.manifest, args.data)
        write_new(out / "m4-input.json", document)
        write_new(out / "input-inventory.json", {"code_commit": commit,
                 "m3_manifest": reference(args.manifest), "m3_inputs": reference(args.batch / "inputs.json"),
                 "tum_tables": [reference(args.data / table_name)
                                for table_name in ("rgb.txt", "depth.txt", "groundtruth.txt")],
                 "frames": inventory})
        steps = []
        def invoke(stage, command, *params):
            argv = [sys.executable, "-B", "-m", "host.offline_memory", command,
                    "--store", str(out / "memory.json"), *map(str, params),
                    "--out", str(out / f"{stage}.json")]
            process = subprocess.run(argv, cwd=root, text=True, capture_output=True,
                                     env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
            (out / f"{stage}.stdout.txt").write_text(process.stdout)
            (out / f"{stage}.stderr.txt").write_text(process.stderr)
            steps.append({"stage": stage, "argv": argv, "returncode": process.returncode})
            write_new(out / f"{stage}.command.json", steps[-1])
            require(process.returncode == 0, f"M4 {stage} failed: {process.stderr}")
            return json.loads((out / f"{stage}.json").read_text())
        imported = invoke("import", "import", "--input", out / "m4-input.json")
        queried = invoke("reload-query", "query")
        require(imported["process_id"] != queried["process_id"], "M4 reload must use a new process")
        store_before = sha256(out / "memory.json")
        duplicate = invoke("repeat", "import", "--input", out / "m4-input.json")
        require(all(row["status"] == "duplicate" for row in duplicate["batches"])
                and sha256(out / "memory.json") == store_before,
                "M4 repeat changed memory or accepted duplicate evidence")
        write_new(out / "commands.json", steps)
        invalid = Counter(o["geometry"].get("reason") for b in document["batches"]
                          for o in b["observations"] if not o["geometry"]["valid"])
        decisions = Counter(d["decision"] for batch_report in imported["batches"]
                            for d in batch_report.get("decisions", []))
        summary = {"ok": True, "source_commit": commit, "rgb_prediction_frames": len(document["batches"]),
                   "depth_associated_frames": sum(row["depth"] is not None for row in inventory),
                   "valid_reference_pose_frames": sum(row["reference_pose_valid"] for row in inventory),
                   "m3_instances": sum(len(b["observations"]) for b in document["batches"]),
                   "mask_geometry_has_local_position": sum(o["geometry"]["valid"] for b in document["batches"]
                                         for o in b["observations"]),
                   "geometry_invalid_reasons": dict(invalid),
                   "m4_decisions": dict(decisions),
                   "m3_normal_empty_frames": sum(b["status"] == "ok" and not b["observations"]
                                                 for b in document["batches"]),
                   "m3_failed_frames": sum(b["status"] == "failed" for b in document["batches"]),
                   "m4_objects": len(queried["objects"]),
                   "m4_pending": len(queried["pending_observations"]),
                   "import_process": imported["process_id"], "reload_process": queried["process_id"],
                   "repeat_unchanged": True, "pose_source": POSE_SOURCE,
                   "scope": "TUM mocap reference, M3 frozen predictions; no live localization or confirmed objects"}
        write_new(out / "summary.json", summary)
        digest_lines = [f"{sha256(p)}  {p.name}" for p in sorted(out.iterdir()) if p.is_file()]
        (out / "MANIFEST.sha256").write_text("\n".join(digest_lines) + "\n", encoding="utf8")
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    except Exception as exc:
        write_new(out / "error.json", {"ok": False, "error": type(exc).__name__, "reason": str(exc)})
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", type=Path, required=True, help="frozen M3 .../batch")
    parser.add_argument("--manifest", type=Path, required=True, help="frozen M3 SHA256 manifest")
    parser.add_argument("--data", type=Path, required=True, help="original TUM desk dataset")
    parser.add_argument("--out", type=Path, required=True, help="new output directory; never overwritten")
    args = parser.parse_args(argv)
    try:
        run(args)
    except (ValueError, OSError, KeyError, TypeError, RuntimeError) as exc:
        print(json.dumps({"ok": False, "error": type(exc).__name__, "reason": str(exc)}), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
