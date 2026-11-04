#!/usr/bin/env python3
"""Evaluate fixed forced pairs against frozen truth-relative camera poses."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


OPTICAL_IN_TRACKED = np.asarray([
    [0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]], dtype=float)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def transform_error(actual, expected):
    delta = np.linalg.inv(expected) @ actual
    cosine = np.clip((np.trace(delta[:3, :3])-1)/2, -1, 1)
    return float(np.linalg.norm(delta[:3, 3])), float(np.degrees(np.arccos(cosine)))


def optical_relative_to_tracked(relative):
    return OPTICAL_IN_TRACKED @ relative @ np.linalg.inv(OPTICAL_IN_TRACKED)


def validate_transform(matrix, label):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"non-finite or malformed transform: {label}")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError(f"invalid homogeneous last row: {label}")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-4):
        raise ValueError(f"invalid SO3 rotation: {label}")
    return matrix


def distribution(values):
    if not values:
        return {"count": 0, "min": None, "median": None, "p90": None, "max": None}
    values = np.asarray(values)
    return {"count": len(values), "min": float(values.min()), "median": float(np.median(values)),
            "p90": float(np.percentile(values, 90)), "max": float(values.max())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("selection", type=Path)
    parser.add_argument("truth_pairs", type=Path)
    parser.add_argument("pairs", type=Path)
    parser.add_argument("observations", type=Path)
    parser.add_argument("query_manifest", type=Path)
    parser.add_argument("pair_output", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("output exists")
    selection = json.loads(args.selection.read_text())
    truth = json.loads(args.truth_pairs.read_text())
    pairs = [json.loads(line) for line in args.pairs.read_text().splitlines() if line.strip()]
    observations = [json.loads(line) for line in args.observations.read_text().splitlines() if line.strip()]
    manifest = json.loads(args.query_manifest.read_text())
    control_rows = [row for row in pairs if row["index"] == -1]
    if len(control_rows) != 4 or len({row["kind"] for row in control_rows}) != 4:
        raise ValueError("controls are not exactly four unique rows")
    controls = {row["kind"]: row for row in control_rows}
    if set(controls) != {"positive_self", "positive_neighbor", "negative_black", "negative_no_depth_both"}:
        raise ValueError("missing or duplicate controls")
    if not controls["positive_self"]["accepted"] or not controls["positive_neighbor"]["accepted"]:
        raise ValueError("positive control failed")
    if controls["negative_black"]["accepted"] or controls["negative_no_depth_both"]["accepted"]:
        raise ValueError("negative control passed")
    for kind, row in controls.items():
        if row["accepted"]:
            validate_transform(row["transform_matrix"], f"control {kind}")
        elif row["transform_matrix"] is not None:
            raise ValueError(f"rejected control has transform: {kind}")
    self_translation, self_rotation = transform_error(np.asarray(controls["positive_self"]["transform_matrix"]), np.eye(4))
    if self_translation > 1e-5 or self_rotation > 1e-3:
        raise ValueError("self control is not identity")
    selected_indexes = {row["index"] for row in selection}
    measured = [row for row in pairs if row["index"] >= 0]
    if len(measured) != 12*24:
        raise ValueError("forced-pair matrix is not 12x24")
    keys = [(row["index"], row["prior_id"]) for row in measured]
    if len(set(keys)) != len(keys) or {key[0] for key in keys} != selected_indexes:
        raise ValueError("duplicate or missing representative pair")
    for index in selected_indexes:
        if {node for i, node in keys if i == index} != set(range(1, 25)):
            raise ValueError(f"index {index} does not cover all 24 nodes")
    for row in measured:
        if row["accepted"]:
            validate_transform(row["transform_matrix"], f"index {row['index']} prior {row['prior_id']}")
        elif row["transform_matrix"] is not None:
            raise ValueError(f"rejected pair has transform at index {row['index']} prior {row['prior_id']}")
    calibration_path = args.pair_output / "prior-calibration.jsonl"
    calibrations = [json.loads(line) for line in calibration_path.read_text().splitlines() if line.strip()]
    expected_local = "xyz=0.000000,0.000000,0.000000 rpy=-1.570796,-0.000000,-1.570796"
    if len(calibrations) != 24 or [row["id"] for row in calibrations] != list(range(1, 25)):
        raise ValueError("prior calibration does not cover 24 unique nodes")
    for row in calibrations:
        if ([row[key] for key in ("fx", "fy", "cx", "cy")] != [517.3, 516.5, 318.6, 255.3]
                or row["local_transform"] != expected_local):
            raise ValueError(f"prior calibration mismatch for node {row['id']}")
    truth_by_pair = {(row["index"], row["node_id"]): row for row in truth}
    info_by_ref = {row["ref_id"]: row for row in observations if row.get("event") == "info"}
    if len(info_by_ref) != 230:
        raise ValueError("unexpected Info population")
    manifest_by_index = {index: row for index, row in enumerate(manifest["pairs"])}
    selection_by_index = {row["index"]: row for row in selection}
    enriched = []
    for row in measured:
        pair = dict(row); truth_row = truth_by_pair[(row["index"], row["prior_id"])]
        pair["truth"] = {key: truth_row[key] for key in ("distance_m", "relative_rotation_deg")}
        if row["accepted"]:
            actual = np.asarray(row["transform_matrix"], dtype=float)
            expected_optical = np.asarray(truth_row["truth_map_optical_to_query_optical"], dtype=float)
            expected_tracked = optical_relative_to_tracked(expected_optical)
            direct = transform_error(actual, expected_tracked)
            inverse = transform_error(actual, np.linalg.inv(expected_tracked))
            pair["truth_residual_assuming_prior_tracked_to_query_tracked"] = {
                "translation_m": direct[0], "rotation_deg": direct[1]}
            pair["truth_residual_opposite_direction_control"] = {
                "translation_m": inverse[0], "rotation_deg": inverse[1]}
        enriched.append(pair)
    rows = []
    for selected in selection:
        index = selected["index"]
        candidates = [row for row in enriched if row["index"] == index]
        best = max(candidates, key=lambda row: (bool(row["accepted"]), row["inliers"], row["matches"], -row["prior_id"]))
        nearest = min(candidates, key=lambda row: (row["truth"]["distance_m"], row["truth"]["relative_rotation_deg"], row["prior_id"]))
        info = info_by_ref[selected["ref_id"]]
        if len(info["stats_keys"]) != len(info["stats_values"]):
            raise ValueError("malformed selected Info stats")
        full_stats = dict(zip(info["stats_keys"], info["stats_values"]))
        source = manifest_by_index[index]
        rgb = args.query_manifest.parent / "rgb_sync" / f"{source['timestamp']}.png"
        depth = args.query_manifest.parent / "depth_sync" / f"{source['timestamp']}.png"
        if sha(rgb) != selected["rgb_input_sha256"] or sha(depth) != selected["depth_input_sha256"]:
            raise ValueError(f"selected input hash mismatch at {index}")
        rows.append({"index": index, "selection_category": selected["selection_category"],
                     "online_ref_id": selected["ref_id"], "online_loop_closure_id": info["loop_closure_id"],
                     "online_proximity_detection_id": info["proximity_detection_id"],
                     "online_stats": full_stats, "nearest_truth_node": nearest, "best_forced_pair": best,
                     "accepted_prior_ids": [row["prior_id"] for row in candidates if row["accepted"]]})
    accepted = [row for row in enriched if row["accepted"]]
    direct_t = [row["truth_residual_assuming_prior_tracked_to_query_tracked"]["translation_m"] for row in accepted]
    direct_r = [row["truth_residual_assuming_prior_tracked_to_query_tracked"]["rotation_deg"] for row in accepted]
    inverse_t = [row["truth_residual_opposite_direction_control"]["translation_m"] for row in accepted]
    inverse_r = [row["truth_residual_opposite_direction_control"]["rotation_deg"] for row in accepted]
    summary = {
        "representatives": len(rows), "forced_pairs": len(enriched), "accepted_pairs": len(accepted),
        "representatives_with_any_accepted_pair": sum(bool(row["accepted_prior_ids"]) for row in rows),
        "by_category": {category: {"representatives": sum(row["selection_category"] == category for row in rows),
                                    "with_any_accepted": sum(row["selection_category"] == category and bool(row["accepted_prior_ids"]) for row in rows),
                                    "accepted_pairs": sum(row["kind"] == category and row["accepted"] for row in enriched)}
                        for category in sorted({row["selection_category"] for row in rows})},
        "truth_residual_forward_translation_m": distribution(direct_t),
        "truth_residual_forward_rotation_deg": distribution(direct_r),
        "truth_residual_opposite_translation_m": distribution(inverse_t),
        "truth_residual_opposite_rotation_deg": distribution(inverse_r),
        "controls": controls,
        "prior_calibration": {"nodes": 24, "intrinsics": [517.3, 516.5, 318.6, 255.3],
                              "local_transform": expected_local},
        "limitations": [
            "Registration reextracts features from raw prior/query images and bypasses online retrieval and Memory::computeTransform.",
            "Historical online query features were not saved.",
            "Truth residuals are reported continuously without a post-hoc acceptance threshold.",
            "A failed forced pair does not prove absent visual coverage; a passed pair does not reproduce historical online localization.",
        ],
    }
    args.output.mkdir(parents=True)
    for filename, value in (("summary.json", summary), ("rows.json", rows), ("pairs-enriched.json", enriched)):
        with (args.output/filename).open("x") as stream:
            json.dump(value, stream, indent=2, allow_nan=False); stream.write("\n")
    receipt = {"tool_sha256": sha(Path(__file__)), "source_sha256": {str(path): sha(path) for path in
               (args.selection, args.truth_pairs, args.pairs, args.observations, args.query_manifest, calibration_path)},
               "pair_output": str(args.pair_output), "transform_interpretation":
               "Registration return compared as prior camera_link -> query camera_link; truth optical relative pose conjugated by camera_link -> optical."}
    with (args.output/"receipt.json").open("x") as stream:
        json.dump(receipt, stream, indent=2); stream.write("\n")
    make_plot(rows, manifest_by_index, args.query_manifest.parent, args.pair_output, args.output/"representative-links.png")
    print(json.dumps(summary, indent=2))


def make_plot(rows, manifest, query_root, pair_output, output):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(len(rows), 3, figsize=(12, 3*len(rows)), layout="constrained")
    for axis_row, row in zip(axes, rows):
        timestamp = manifest[row["index"]]["timestamp"]
        query = cv2.imread(str(query_root/"rgb_sync"/f"{timestamp}.png"))
        nearest_id = row["nearest_truth_node"]["prior_id"]
        best = row["best_forced_pair"]; best_id = best["prior_id"]
        nearest = cv2.imread(str(pair_output/"prior-images"/f"{nearest_id}.png"))
        best_image = cv2.imread(str(pair_output/"prior-images"/f"{best_id}.png"))
        for axis, image in zip(axis_row, (query, nearest, best_image)):
            axis.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)); axis.axis("off")
        axis_row[0].set_title(f"{row['selection_category']}\nquery index {row['index']}", fontsize=8)
        axis_row[1].set_title(f"truth-nearest prior {nearest_id}\n{row['nearest_truth_node']['truth']['distance_m']:.3f} m, {row['nearest_truth_node']['truth']['relative_rotation_deg']:.1f} deg", fontsize=8)
        axis_row[2].set_title(f"forced-best prior {best_id}: {'PASS' if best['accepted'] else 'FAIL'}\n{best['inliers']} inliers / {best['matches']} matches", fontsize=8)
    figure.suptitle("Fixed representative query, truth-nearest map node, and strongest raw forced pair", fontsize=12)
    figure.savefig(output, dpi=120); plt.close(figure)


if __name__ == "__main__":
    main()
