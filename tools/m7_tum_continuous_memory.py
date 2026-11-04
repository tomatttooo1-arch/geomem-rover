"""Eight adjacent real TUM RGB-D frames with one frozen, delayed M3 mask batch.

This is a CPU interface replay. TUM motion capture is a reference pose, not
estimated localization; other seven frames have no detector output. No model,
ROS, device, board, navigation or motor controller is started.
"""

import argparse
from dataclasses import asdict
from decimal import Decimal
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

import cv2
import numpy as np

from host.m7_memory import build_m4_document
from host.m7_native import NativeM7, compile_native
from host.m7_object_native import NativeObjectEvidence, compile_object_native
from host.m7_pipeline import (CaptureIdentity, DepthFrame, GridSpec, M7Pipeline,
                              PoseAtCapture, SemanticPacket)
from perception.offline_memory import OfflineMemory
from perception.fpga_replay.io import interpolate_pose, read_table, sha256
from tools.m3_mask_memory import (CALIBRATION, K, M3_COMMIT, MANIFEST_SHA256,
                                  MAP_ID, MAP_VERSION, MODEL_SHA256, POSE_SOURCE,
                                  frozen_manifest, manifest_file, path_within,
                                  read_instance_mask)


ROOT = Path(__file__).resolve().parents[1]
SOURCE_FILES = ["tools/m7_tum_continuous_memory.py", "tools/m3_mask_memory.py",
                "host/m7_memory.py", "host/m7_pipeline.py", "host/m7_native.py",
                "hw/m7/ray_flags.cpp", "hw/geometry/geometry.cpp",
                "perception/offline_memory.py", "host/m7_object_native.py",
                "hw/m7/object_evidence.cpp"]
PREDICTION_SET = "M3_c895e5a_yolov8n_seg_a7cd8f9"
CLOCK = "TUM_original_RGB_timestamp_ns"
STREAM = "TUM_freiburg1_desk_selected_window_20260921"
LOCAL_FRAME = "TUM_mocap_world"
POSE_EPOCH = "TUM_mocap_reference_window_v1"
POSE_VERSION = "TUM_mocap_reference_map_identity_v1"


def require(condition, message):
    if not condition:
        raise ValueError(message)


def write_new(path, value):
    with path.open("x", encoding="utf8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def reference(path, selector=None):
    result = {"path": str(path.resolve()), "sha256": sha256(path)}
    if selector is not None:
        result["selector"] = selector
    return result


def source_clean():
    command = ["git", "diff", "--quiet", "HEAD", "--", *SOURCE_FILES]
    require(subprocess.run(command, cwd=ROOT).returncode == 0,
            "tracked replay source differs from fixed HEAD")


def verify_frozen_rgb(image, selected, data, stamp_text):
    """Bind the original RGB bytes and timestamp row to frozen M3 metadata."""
    rgb_path = Path(image["source_path"]).resolve()
    require(rgb_path == Path(selected["source_path"]).resolve() and
            path_within(rgb_path, data) and rgb_path.parent == data / "rgb",
            "M3 selected RGB path mismatch")
    actual_sha = sha256(rgb_path)
    require(actual_sha == image["sha256"] == selected["sha256"] and
            image["frame_id"] == "sha256:" + actual_sha,
            "original RGB bytes do not match frozen M3 content identity")
    source = image["timestamp_source"]
    relative = rgb_path.relative_to(data).as_posix()
    require(Path(source["path"]).resolve() == data / "rgb.txt" and
            source["sha256"] == sha256(data / "rgb.txt") and
            source["row"] == stamp_text + " " + relative,
            "M3 RGB acquisition row does not match original TUM table")
    return rgb_path


def run_cli(output, name, command):
    process = subprocess.run(command, cwd=ROOT, text=True, capture_output=True,
                             env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
    (output / (name + ".stdout.txt")).write_text(process.stdout)
    (output / (name + ".stderr.txt")).write_text(process.stderr)
    write_new(output / (name + ".command.json"),
              {"argv": command, "returncode": process.returncode})
    require(process.returncode == 0, name + " M4 command failed: " + process.stderr)
    return json.loads((output / (name + ".json")).read_text())


def run(args):
    data = args.data.resolve()
    batch = args.batch.resolve()
    manifest = args.manifest.resolve()
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=False)
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                     text=True).strip()
    write_new(output / "run-info.json",
              {"source_commit": commit, "argv": sys.argv,
               "python": platform.python_version(),
               "source_sha256": {name: sha256(ROOT / name) for name in SOURCE_FILES},
               "scope": "eight real adjacent TUM RGB-D frames; TUM mocap reference, one frozen M3 prediction batch; CPU only"})
    try:
        source_clean()
        require(sha256(manifest) == MANIFEST_SHA256, "frozen M3 manifest mismatch")
        entries = frozen_manifest(manifest)
        require(batch == manifest.parent / "m3-batch-20260916" / "batch",
                "batch is outside frozen M3 manifest root")
        selected_path = manifest_file(batch.parent, entries, "batch/inputs.json")
        selected = json.loads(selected_path.read_text())["inputs"][1]  # pinned M3 image 002
        target_stamp = selected["timestamp_text"]
        result_path = manifest_file(batch.parent, entries, "batch/image-002/result.json")
        run_path = manifest_file(batch.parent, entries, "batch/image-002/run_info.json")
        result = json.loads(result_path.read_text())
        run_info = json.loads(run_path.read_text())
        require(result["ok"] and run_info["code_commit"] == M3_COMMIT and
                run_info["model"]["sha256"] == MODEL_SHA256,
                "M3 run status, code or model differs from frozen source")
        image = result["image"]
        require(image["acquisition_timestamp"] == float(target_stamp),
                "M3 result does not match selected RGB capture")
        target_rgb_path = verify_frozen_rgb(image, selected, data, target_stamp)
        rgb_rows, _ = read_table(data / "rgb.txt")
        depth_rows, depth_times = read_table(data / "depth.txt")
        truth_rows, truth_times = read_table(data / "groundtruth.txt")
        matches = [i for i, row in enumerate(rgb_rows)
                   if row[0] == target_stamp and (data / row[1]).resolve() == target_rgb_path]
        require(len(matches) == 1, "M3 target is not uniquely present in original TUM rgb.txt")
        target_index = matches[0]
        window = rgb_rows[target_index - 3:target_index + 5]
        require(len(window) == 8 and window[3][0] == target_stamp,
                "fixed eight-frame contiguous window unavailable")
        used_depth = set()
        associated = []
        for row in window:
            stamp = float(row[0])
            depth_index = int(np.argmin(np.abs(depth_times - stamp)))
            pose, pose_valid = interpolate_pose(stamp, truth_rows, truth_times, max_gap=.05)
            require(depth_index not in used_depth and abs(depth_times[depth_index] - stamp) <= .02
                    and pose_valid, "contiguous window has unmatched depth or unavailable reference pose")
            used_depth.add(depth_index)
            rgb_path = (data / row[1]).resolve()
            depth_path = (data / depth_rows[depth_index][1]).resolve()
            require(path_within(rgb_path, data) and path_within(depth_path, data),
                    "TUM path escapes frozen dataset")
            associated.append((row[0], rgb_path, depth_path, float(depth_times[depth_index]), pose))

        origin = tuple((associated[0][4][:3, 3] - 4.).tolist())
        grid = GridSpec(origin, .25, (32, 32, 32))
        library = compile_native(output / "m7.so")
        # "native" routes object evidence through the M7 object kernel's C
        # reference (the PL candidate), so the memory bridge consumes the
        # FPGA-side payload; "python" keeps the original software path.
        object_backend = None
        if args.object_backend == "native":
            object_backend = NativeObjectEvidence(compile_object_native(output / "object.so"))
        pipeline = M7Pipeline(NativeM7(library), grid, object_backend=object_backend)
        geometry_rows, inventory = [], []
        target = None
        document = audit = None
        for ordinal, (stamp_text, rgb_path, depth_path, depth_stamp, pose) in enumerate(associated, 1):
            acquired_ns = int(Decimal(stamp_text) * Decimal(1_000_000_000))
            identity = CaptureIdentity(STREAM, ordinal, acquired_ns, CLOCK,
                                       CALIBRATION, LOCAL_FRAME, POSE_EPOCH)
            raw_rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
            raw_depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
            require(raw_rgb is not None and raw_depth is not None and
                    raw_depth.dtype == np.uint16 and raw_depth.shape == raw_rgb.shape[:2],
                    "TUM RGB/depth decode, dtype or registered raster mismatch")
            depth_m = raw_depth.astype(np.float32) / 5000.
            depth_m[raw_depth == 0] = np.nan
            f = DepthFrame(identity, depth_m, np.asarray(K, dtype=np.float64),
                           PoseAtCapture(acquired_ns, CLOCK, LOCAL_FRAME, POSE_EPOCH,
                                         True, pose))
            geometry = pipeline.ingest_geometry(f, now_ns=acquired_ns + 1,
                                                now_clock_id=CLOCK)
            nav = geometry.navigation
            row = {"identity": asdict(identity), "geometry_valid": nav.valid,
                   "reason": nav.reason, "age_ns": nav.age_ns,
                   "backend": nav.backend, "frame_token": nav.frame_token,
                   "counts": {key: nav.payload.get(key) for key in
                              ("input_pixels", "valid_grid_samples", "invalid_depth_or_projection",
                               "outside_grid", "unknown_voxels", "free_voxels", "hit_voxels")},
                   "voxel_sha256": hashlib.sha256(nav.payload["voxel_flags"].tobytes()).hexdigest()
                       if nav.valid else None}
            geometry_rows.append(row)
            inventory.append({"identity": asdict(identity), "rgb": reference(rgb_path),
                              "depth": reference(depth_path), "rgb_stamp": stamp_text,
                              "depth_stamp": depth_stamp,
                              "rgb_depth_delta_s": depth_stamp - float(stamp_text),
                              "pose_source": POSE_SOURCE,
                              "reference_pose_valid": True})
            if ordinal == 4:
                require(nav.valid, "M3 target's original depth has no valid M7 geometry")
                target = (f, geometry, rgb_path, depth_path)
            if ordinal == 5:
                require(target is not None, "target geometry not captured")
                old_frame, old_geometry, target_rgb, target_depth = target
                require(not old_geometry.navigation.validate_at(pipeline, acquired_ns + 2, CLOCK).valid,
                        "old navigation geometry should be revoked by next capture")
                masks, instances, outputs = {}, [], {}
                for instance_no, det in enumerate(result["detections"], 1):
                    value, mask_path = read_instance_mask(
                        det, instance_no, "batch/image-002", batch.parent, entries,
                        old_frame.depth_m.shape)
                    key = str(det["instance_id"])
                    require(key not in masks, "duplicate frozen M3 single-frame instance")
                    masks[key] = value
                    mask_hash = hashlib.sha256(value.astype(np.uint8).tobytes()).hexdigest()
                    instances.append({"instance_id": det["instance_id"],
                                      "label": det["class_name"].lower().strip(),
                                      "score": det["confidence"], "source": "frozen_M3_yolov8n_seg",
                                      "mask_content_sha256": mask_hash,
                                      "evidence_refs": [reference(result_path), reference(run_path),
                                                        reference(mask_path, "instance_id:" + key),
                                                        reference(manifest)]})
                    outputs[key] = pipeline.ingest_semantic(SemanticPacket(
                        old_frame.identity, old_geometry.navigation.frame_token, key, value),
                        now_ns=acquired_ns + 2, now_clock_id=CLOCK)
                capture = {"identity": asdict(old_frame.identity),
                           "acquisition_id": "tum-rgb:" + target_stamp,
                           "source_id": "TUM_freiburg1_desk_original_recording",
                           "image_sha256": sha256(target_rgb),
                           "time_source": "TUM_rgb.txt:" + target_stamp + " " +
                                          target_rgb.relative_to(data).as_posix(),
                           "evidence_refs": [reference(target_rgb), reference(target_depth),
                                             reference(data / "rgb.txt"),
                                             reference(data / "depth.txt"), reference(selected_path)]}
                detection = {"identity": asdict(old_frame.identity),
                             "prediction_set": PREDICTION_SET, "status": "ok", "error": None,
                             "expected_instance_ids": list(masks), "instances": instances}
                anchor = {"map_id": MAP_ID, "map_version": MAP_VERSION,
                          "world_frame": LOCAL_FRAME, "pose_source": POSE_SOURCE,
                          "source_kind": "TUM_mocap_reference",
                          "source_ref": reference(data / "groundtruth.txt"),
                          "local_frame_id": LOCAL_FRAME, "pose_epoch": POSE_EPOCH,
                          "pose_version": POSE_VERSION, "T_map_local": np.eye(4).tolist(),
                          "evidence_refs": [reference(data / "groundtruth.txt"),
                                            reference(data / "rgb.txt")],
                          "calibration_id": CALIBRATION}
                document, audit = build_m4_document(
                    pipeline, old_geometry, outputs, capture=capture, detection=detection,
                    anchor=anchor, now_ns=acquired_ns + 2, clock_id=CLOCK)
                require(document is not None, "complete frozen M3 batch did not produce M4 input")
                write_new(output / "m4-input.json", document)
                write_new(output / "adapter-audit.json", audit)
                write_new(output / "semantic-output.json", {key: {"valid": x.valid,
                    "reason": x.reason, "candidate": x.payload.get("candidate"),
                    "mask_pixels": x.payload.get("mask_pixels"),
                    "mask_valid_pixels": x.payload.get("mask_valid_pixels"),
                    "valid_fraction": x.payload.get("valid_fraction"),
                    "kernel_counts": x.payload.get("kernel_counts"),
                    "mask_content_sha256": x.payload.get("mask_content_sha256")}
                    for key, x in outputs.items()})

        require(document is not None, "M3 mask was never consumed")
        write_new(output / "geometry-frames.json", geometry_rows)
        write_new(output / "input-inventory.json",
                  {"tum_tables": [reference(data / name) for name in
                                   ("rgb.txt", "depth.txt", "groundtruth.txt")],
                   "m3_manifest": reference(manifest), "m3_selected_inputs": reference(selected_path),
                   "m3_result": reference(result_path), "m3_run_info": reference(run_path),
                   "frames": inventory})
        store = output / "memory.json"
        def command(stage, verb):
            argv = [sys.executable, "-B", "-m", "host.offline_memory", verb,
                    "--store", str(store), "--out", str(output / (stage + ".json"))]
            if verb == "import":
                argv += ["--input", str(output / "m4-input.json")]
            return run_cli(output, stage, argv)
        imported = command("import", "import")
        reloaded = command("reload-query", "query")
        before_repeat = sha256(store)
        repeated = command("repeat", "import")
        require(imported["process_id"] != reloaded["process_id"] and
                reloaded["process_id"] != repeated["process_id"],
                "M4 commands did not run in distinct processes")
        require(all(x["status"] == "duplicate" for x in repeated["batches"]) and
                sha256(store) == before_repeat,
                "repeat import changed persisted M4 evidence")
        require(len(OfflineMemory.load(store).data["observations"]) == len(result["detections"]) and
                len(document["batches"][0]["observations"]) == len(result["detections"]),
                "M4 observation count lost detector instances")
        summary = {"ok": True, "source_commit": commit,
                   "object_backend": args.object_backend,
                   "scope": "eight adjacent original TUM RGB-D captures; mocap reference, one frozen M3 prediction batch delayed by one frame; CPU M7→M4 only",
                   "rgb_depth_frames": len(geometry_rows),
                   "geometry_valid_frames": sum(x["geometry_valid"] for x in geometry_rows),
                   "semantic_prediction_frames": 1,
                   "frozen_m3_instances": len(result["detections"]),
                   "m4_objects": len(reloaded["objects"]),
                   "m4_pending": len(reloaded["pending_observations"]),
                   "repeat_unchanged": True,
                   "pose_source": POSE_SOURCE,
                   "pose_is_estimated_localization": False,
                   "navigation_ready": False}
        write_new(output / "summary.json", summary)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    except Exception as exc:
        write_new(output / "error.json",
                  {"ok": False, "error": type(exc).__name__, "reason": str(exc)})
        raise
    finally:
        (output / "MANIFEST.sha256").write_text("".join(
            f"{sha256(path)}  {path.name}\n" for path in sorted(output.iterdir())
            if path.is_file() and path.name != "MANIFEST.sha256"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True,
                        help="fresh result directory; never overwritten")
    parser.add_argument("--object-backend", choices=("python", "native"), default="python",
                        help="object evidence path: original Python or M7 object kernel")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
