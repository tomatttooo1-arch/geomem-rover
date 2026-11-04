#!/usr/bin/env python3
"""Freeze representative match-gap frames and truth-relative view geometry."""
import argparse
from collections import Counter
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path
import sqlite3

import numpy as np

from tools.rtabmap_eval import associate, read_tum


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stamp_ns(row):
    return int(Decimal(str(row["timestamp"])) * 1_000_000_000)


def pose_matrix(row):
    x, y, z, qx, qy, qz, qw = row[1:]
    q = np.asarray([qx, qy, qz, qw], dtype=float)
    q /= np.linalg.norm(q)
    qx, qy, qz, qw = q
    rotation = np.asarray([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ])
    output = np.eye(4); output[:3, :3] = rotation; output[:3, 3] = [x, y, z]
    return output


def relative_metrics(left, right):
    relative = np.linalg.inv(left) @ right
    cosine = np.clip((np.trace(relative[:3, :3])-1)/2, -1, 1)
    return float(np.linalg.norm(relative[:3, 3])), float(np.degrees(np.arccos(cosine))), relative


def distribution(values):
    values = np.asarray(values, dtype=float)
    return {"count": len(values), "min": float(values.min()), "median": float(np.median(values)),
            "p90": float(np.percentile(values, 90)), "max": float(values.max())}


def representatives(rows):
    rows = sorted(rows, key=lambda row: row["index"])
    if not rows:
        raise ValueError("empty representative population")
    return [rows[position] for position in sorted(set((0, len(rows)//2, len(rows)-1)))]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage1_ledger", type=Path)
    parser.add_argument("mapping_manifest", type=Path)
    parser.add_argument("query_manifest", type=Path)
    parser.add_argument("database", type=Path)
    parser.add_argument("ground_truth", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("output exists")
    stage1 = json.loads(args.stage1_ledger.read_text())
    mapping = json.loads(args.mapping_manifest.read_text())
    query = json.loads(args.query_manifest.read_text())
    if len(stage1) != 230 or len(query["pairs"]) != 230 or len(mapping["pairs"]) != 343:
        raise ValueError("unexpected frozen populations")
    db_before = sha(args.database)
    connection = sqlite3.connect(f"file:{args.database}?mode=ro&immutable=1", uri=True)
    try:
        nodes = list(connection.execute("SELECT id, stamp FROM Node ORDER BY id"))
    finally:
        connection.close()
    if [row[0] for row in nodes] != list(range(1, 25)) or sha(args.database) != db_before:
        raise ValueError("unexpected or changed frozen database")
    mapping_times = np.asarray([float(row["timestamp"]) for row in mapping["pairs"]])
    node_manifest = []
    for node_id, node_time in nodes:
        differences = np.abs(mapping_times-node_time)
        matches = np.flatnonzero(differences < 0.000002)
        if len(matches) != 1:
            raise ValueError(f"node {node_id} does not uniquely match mapping input")
        item = mapping["pairs"][int(matches[0])]
        node_manifest.append({"node_id": node_id, "node_stamp": node_time,
                              "mapping_original_index": item["original_index"],
                              "mapping_timestamp": item["timestamp"],
                              "node_manifest_difference_s": float(differences[matches[0]])})
    truth = read_tum(args.ground_truth)
    requested_times = np.asarray([row["node_stamp"] for row in node_manifest] +
                                 [float(row["timestamp"]) for row in query["pairs"]])
    matches = associate(requested_times, truth[:, 0], .02)
    if len(matches) != len(requested_times):
        raise ValueError("not every map/query frame has unique truth association")
    truth_by_request = {i: (j, abs(requested_times[i]-truth[j, 0])) for i, j in matches}
    if max(delta for _, delta in truth_by_request.values()) >= .02:
        raise ValueError("truth association outside fixed tolerance")
    map_poses = {}
    for i, node in enumerate(node_manifest):
        truth_index, delta = truth_by_request[i]
        node["truth_index"] = truth_index; node["truth_difference_s"] = delta
        node["truth_pose"] = pose_matrix(truth[truth_index]).tolist()
        map_poses[node["node_id"]] = pose_matrix(truth[truth_index])
    all_pairs, query_rows = [], []
    stage_by_index = {row["index"]: row for row in stage1}
    for index, manifest_row in enumerate(query["pairs"]):
        stage = stage_by_index[index]
        if stage["stamp_ns"] != stamp_ns(manifest_row):
            raise ValueError(f"stage1/query stamp mismatch at {index}")
        truth_index, delta = truth_by_request[len(nodes)+index]
        query_pose = pose_matrix(truth[truth_index])
        pairs = []
        for node in node_manifest:
            distance, angle, relative = relative_metrics(map_poses[node["node_id"]], query_pose)
            pair = {"index": index, "node_id": node["node_id"], "distance_m": distance,
                    "relative_rotation_deg": angle, "truth_map_optical_to_query_optical": relative.tolist()}
            pairs.append(pair); all_pairs.append(pair)
        nearest = min(pairs, key=lambda row: (row["distance_m"], row["relative_rotation_deg"], row["node_id"]))
        query_rows.append({"index": index, "timestamp": manifest_row["timestamp"],
                           "original_index": manifest_row["original_index"],
                           "truth_index": truth_index, "truth_difference_s": delta,
                           "truth_pose": query_pose.tolist(),
                           "nearest_node_by_translation": nearest})
    categories = sorted({row["category"] for row in stage1 if row["primary_reason"] == "no_prior_match"})
    if categories != ["candidate_screen_or_no_saved_geometry_evidence", "geometry_inliers_rejected", "rejected_hypothesis_stage_unknown"]:
        raise ValueError(f"unexpected primary no-prior categories: {categories}")
    selected = []
    populations = [(category, [row for row in stage1 if row["primary_reason"] == "no_prior_match" and row["category"] == category])
                   for category in categories]
    populations.append(("approved_positive", [row for row in stage1 if row["approved"]]))
    for label, population in populations:
        for row in representatives(population):
            manifest_row = query["pairs"][row["index"]]
            selected.append({"selection_category": label, "population_size": len(population),
                             "index": row["index"], "ref_id": row["ref_id"],
                             "timestamp": manifest_row["timestamp"],
                             "original_index": manifest_row["original_index"],
                             "rgb_input_sha256": manifest_row["rgb_sync"]["input_sha256"],
                             "depth_input_sha256": manifest_row["depth_sync"]["input_sha256"],
                             "online_info": row, "truth": query_rows[row["index"]]})
    if len(selected) != 12 or len({row["index"] for row in selected}) != 12:
        raise ValueError("fixed representative selection is not 12 unique frames")
    summary = {"map_nodes": len(nodes), "query_frames": len(query_rows), "truth_pairs": len(all_pairs),
               "representatives": len(selected), "population_counts": {label: len(rows) for label, rows in populations},
               "nearest_translation_m": distribution([row["nearest_node_by_translation"]["distance_m"] for row in query_rows]),
               "nearest_relative_rotation_deg": distribution([row["nearest_node_by_translation"]["relative_rotation_deg"] for row in query_rows]),
               "selection_indexes": {label: [row["index"] for row in selected if row["selection_category"] == label]
                                     for label, _ in populations},
               "limits": ["Truth-relative distance and rotation support viewpoint comparison but are not visual-overlap ground truth.",
                          "Representative selection is fixed before forced-pair results."]}
    args.output.mkdir(parents=True)
    values = {"summary.json": summary, "map-nodes-truth.json": node_manifest,
              "query-truth.json": query_rows, "all-truth-pairs.json": all_pairs,
              "selection.json": selected,
              "receipt.json": {"tool_sha256": sha(Path(__file__)), "database_sha256_before_after": [db_before, sha(args.database)],
                               "source_sha256": {str(path): sha(path) for path in (args.stage1_ledger, args.mapping_manifest,
                                                                                  args.query_manifest, args.database, args.ground_truth)}}}
    for filename, value in values.items():
        with (args.output/filename).open("x") as stream:
            json.dump(value, stream, indent=2, allow_nan=False); stream.write("\n")
    with (args.output/"selection.tsv").open("x") as stream:
        for row in selected:
            timestamp = row["timestamp"]
            stream.write(f"{row['selection_category']} {row['index']} /query/rgb_sync/{timestamp}.png /query/depth_sync/{timestamp}.png\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
