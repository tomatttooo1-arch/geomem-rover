#!/usr/bin/env python3
"""Split a prepared TUM input into fixed, disjoint mapping and query inputs."""

import argparse
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import shutil


DATASET_NAME = "rgbd_dataset_freiburg1_desk"
EXPECTED_CAMERA_MATRIX = [[517.3, 0.0, 318.6], [0.0, 516.5, 255.3], [0.0, 0.0, 1.0]]
EXPECTED_LOCAL_TRANSFORM = [[0.0, 0.0, 1.0, 0.0], [-1.0, 0.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0, 0.0]]


def path_within(path, root):
    """``PurePath.is_relative_to`` for Python 3.8, the declared reference
    interpreter; the standard-library method only exists from Python 3.9.

    This is the CPython 3.9 implementation, so it keeps the exact semantics: a
    purely lexical parts-prefix test that touches no filesystem, does not resolve
    symlinks and does not collapse ``..``, and that answers ``False`` (rather than
    raising) for a relative/absolute mix. Callers that need symlinks and ``..``
    resolved must ``resolve()`` both sides first, exactly as they must with the
    standard-library method.
    """
    try:
        Path(path).relative_to(root)
    except ValueError:
        return False
    return True


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stamp_ns(value):
    return int(Decimal(str(value)) * 1_000_000_000)


def pair_identity(row):
    return {
        "original_index": row["original_index"],
        "timestamp": row["timestamp"],
        "depth_timestamp": row["depth_timestamp"],
        "rgb_source": row["rgb_sync"]["source"],
        "depth_source": row["depth_sync"]["source"],
        "rgb_sha256": row["rgb_sync"]["input_sha256"],
        "depth_sha256": row["depth_sync"]["input_sha256"],
    }


def identity_sha256(rows):
    payload = json.dumps([pair_identity(row) for row in rows], sort_keys=True,
                         separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


def validate_calibration(path):
    import cv2
    import numpy as np
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError("cannot open RTAB-Map calibration")
    try:
        def matrix(name):
            node = storage.getNode(name)
            rows = int(node.getNode("rows").real())
            columns = int(node.getNode("cols").real())
            data = node.getNode("data")
            values = [data.at(index).real() for index in range(data.size())]
            if rows <= 0 or columns <= 0 or len(values) != rows * columns:
                raise ValueError(f"malformed calibration matrix: {name}")
            return np.asarray(values, dtype=float).reshape(rows, columns)
        name = storage.getNode("camera_name").string()
        width = int(storage.getNode("image_width").real())
        height = int(storage.getNode("image_height").real())
        camera = matrix("camera_matrix")
        local = matrix("local_transform")
    finally:
        storage.release()
    if (name != "rtabmap_calib" or (width, height) != (640, 480)
            or camera is None or camera.shape != (3, 3)
            or local is None or local.shape != (3, 4)
            or not np.allclose(camera, EXPECTED_CAMERA_MATRIX, atol=1e-9, rtol=0)
            or not np.allclose(local, EXPECTED_LOCAL_TRANSFORM, atol=1e-9, rtol=0)):
        raise ValueError("calibration does not match fixed Freiburg1 camera and optical transform")
    return {"camera_name": name, "image_size": [width, height],
            "camera_matrix": camera.tolist(), "local_transform": local.tolist()}


def validate_source(manifest_path, manifest):
    pairs = manifest.get("pairs")
    if not isinstance(pairs, list) or len(pairs) < 2:
        raise ValueError("source manifest needs at least two pairs")
    stamps = [stamp_ns(row["timestamp"]) for row in pairs]
    if any(later <= earlier for earlier, later in zip(stamps, stamps[1:])):
        raise ValueError("source timestamps are not strictly increasing")
    root = manifest_path.parent.resolve()
    for index, row in enumerate(pairs):
        if "original_index" in row:
            raise ValueError("source manifest already contains original_index")
        for folder in ("rgb_sync", "depth_sync"):
            path = (root / folder / (row["timestamp"] + ".png")).resolve()
            if not path_within(path, root) or not path.is_file():
                raise ValueError(f"missing prepared image at pair {index}: {folder}")
            if sha256(path) != row[folder]["input_sha256"]:
                raise ValueError(f"prepared image hash mismatch at pair {index}: {folder}")
    return stamps


def segment_manifest(source, rows, role, start, stop, split, source_path, source_hash,
                     calibration_path, calibration_hash, calibration_validation):
    output = {key: value for key, value in source.items() if key != "pairs"}
    output.update({
        "schema": 1,
        "role": role,
        "dataset_name": DATASET_NAME,
        "source_manifest": str(source_path.resolve()),
        "source_manifest_sha256": source_hash,
        "split_ratio": 0.6,
        "split_index": split,
        "original_range": [start, stop],
        "pairs": rows,
        "pairs_identity_sha256": identity_sha256(rows),
        "calibration": {"source": str(calibration_path.resolve()),
                        "source_sha256": calibration_hash,
                        "input_sha256": calibration_hash,
                        "validation": calibration_validation},
        "groundtruth_available_to_slam": False,
    })
    return output


def split_input(source_path, output_root, calibration_path):
    source_path = Path(source_path)
    output_root = Path(output_root)
    if output_root.exists():
        raise FileExistsError(output_root)
    calibration_path = Path(calibration_path)
    calibration_validation = validate_calibration(calibration_path)
    calibration_hash = sha256(calibration_path)
    source_bytes = source_path.read_bytes()
    source = json.loads(source_bytes)
    stamps = validate_source(source_path, source)
    count = len(source["pairs"])
    split = int(count * 0.6)
    if split <= 0 or split >= count:
        raise ValueError("invalid fixed split")
    mapping_rows = source["pairs"][:split]
    query_rows = source["pairs"][split:]
    depth_stamps = [stamp_ns(row["depth_timestamp"]) for row in source["pairs"]]
    if max(stamps[:split]) >= min(stamps[split:]) \
            or max(depth_stamps[:split]) >= min(depth_stamps[split:]):
        raise ValueError("mapping and query time ranges overlap")
    for folder in ("rgb_sync", "depth_sync"):
        map_sources = {row[folder]["source"] for row in mapping_rows}
        query_sources = {row[folder]["source"] for row in query_rows}
        if map_sources & query_sources:
            raise ValueError(f"source path overlap in {folder}")
    map_pairs = {(row["rgb_sync"]["input_sha256"], row["depth_sync"]["input_sha256"])
                 for row in mapping_rows}
    query_pairs = {(row["rgb_sync"]["input_sha256"], row["depth_sync"]["input_sha256"])
                   for row in query_rows}
    if map_pairs & query_pairs:
        raise ValueError("paired input hash overlap across split")
    source_hash = hashlib.sha256(source_bytes).hexdigest()
    prepared = {}
    for role, start, stop in (("mapping", 0, split), ("query", split, count)):
        dataset = output_root / role / DATASET_NAME
        (dataset / "rgb_sync").mkdir(parents=True)
        (dataset / "depth_sync").mkdir()
        rows = []
        for original_index in range(start, stop):
            row = json.loads(json.dumps(source["pairs"][original_index]))
            row["original_index"] = original_index
            for folder in ("rgb_sync", "depth_sync"):
                filename = row["timestamp"] + ".png"
                shutil.copyfile(source_path.parent / folder / filename, dataset / folder / filename)
                if sha256(dataset / folder / filename) != row[folder]["input_sha256"]:
                    raise ValueError(f"copied image hash mismatch at pair {original_index}: {folder}")
            rows.append(row)
        shutil.copyfile(calibration_path, dataset / "rtabmap_calib.yaml")
        if sha256(dataset / "rtabmap_calib.yaml") != calibration_hash:
            raise ValueError(f"copied calibration hash mismatch in {role}")
        manifest = segment_manifest(source, rows, role, start, stop, split,
                                    source_path, source_hash, calibration_path,
                                    calibration_hash, calibration_validation)
        manifest_path = dataset / "INPUT.json"
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
        prepared[role] = {
            "manifest": str(manifest_path),
            "manifest_sha256": sha256(manifest_path),
            "pairs": len(rows),
            "original_range": [start, stop],
            "timestamp_ns_range": [stamps[start], stamps[stop - 1]],
            "pairs_identity_sha256": manifest["pairs_identity_sha256"],
        }
    summary = {
        "schema": 1,
        "protocol": "fixed floor(0.6*N) temporal split; copied images; no truth",
        "source_manifest": str(source_path.resolve()),
        "source_manifest_sha256": source_hash,
        "source_pairs": count,
        "split_index": split,
        "mapping": prepared["mapping"],
        "query": prepared["query"],
        "time_ranges_disjoint": True,
        "rgb_source_paths_disjoint": True,
        "depth_source_paths_disjoint": True,
        "paired_input_hashes_disjoint": True,
        "groundtruth_copied": False,
        "calibration": {"source": str(calibration_path.resolve()),
                        "sha256": calibration_hash, "validation": calibration_validation},
    }
    (output_root / "split.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="existing prepared INPUT.json")
    parser.add_argument("output", type=Path, help="new split root")
    parser.add_argument("--calibration", required=True, type=Path,
                        help="frozen Freiburg1 rtabmap_calib.yaml")
    args = parser.parse_args()
    try:
        result = split_input(args.source, args.output, args.calibration)
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        parser.exit(2, f"split preparation failed: {exc}\n")
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
