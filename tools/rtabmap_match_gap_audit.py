#!/usr/bin/env python3
"""Join frozen RTAB-Map Info, gate and ref-addressed rejection logs."""
import argparse
from collections import Counter
import hashlib
import json
import platform
from pathlib import Path
import re
import sqlite3
import subprocess


REQUIRED_STATS = (
    "Loop/Highest_hypothesis_id/",
    "Loop/RejectedHypothesis/",
    "Loop/Visual_matches/",
    "Loop/Visual_inliers/",
)
OPTIMIZATION_RATIO = "Loop/Optimization_max_error_ratio/"
ANSI = re.compile(r"\x1b\[[0-9;]*m")
GEOMETRY = re.compile(
    r"Rejected loop closure (\d+) -> (\d+): Not enough inliers (\d+)/(\d+) \(matches=(\d+)\)")
GRAPH = re.compile(
    r"Rejecting localization \((\d+) <-> (\d+)\).*maximum graph error ratio of ([0-9.eE+-]+)")


def load_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def unique(rows, key, label):
    result = {}
    for row in rows:
        value = row[key]
        if value in result:
            raise ValueError(f"duplicate {label}: {value}")
        result[value] = row
    return result


def parse_logs(text, prior_ids):
    by_ref = {}
    for line_number, raw in enumerate(text.splitlines(), 1):
        line = ANSI.sub("", raw)
        geometry = GEOMETRY.search(line)
        graph = GRAPH.search(line)
        if not geometry and not graph:
            continue
        if geometry:
            prior, current, inliers, required, matches = map(int, geometry.groups())
            item = {"kind": "geometry_inliers_rejected", "prior_id": prior,
                    "ref_id": current, "inliers": inliers,
                    "required_inliers": required, "matches": matches}
        else:
            current, prior = map(int, graph.groups()[:2])
            item = {"kind": "graph_consistency_rejected", "prior_id": prior,
                    "ref_id": current, "maximum_graph_error_ratio": float(graph.group(3))}
        if item["prior_id"] not in prior_ids:
            raise ValueError(f"log prior outside frozen map at line {line_number}: {item['prior_id']}")
        item["line_number"] = line_number
        item["line"] = line
        previous = by_ref.get(item["ref_id"])
        if previous is not None:
            raise ValueError(f"duplicate or conflicting rejection logs for ref {item['ref_id']}")
        by_ref[item["ref_id"]] = item
    return by_ref


def stats(info):
    keys, values = info.get("stats_keys"), info.get("stats_values")
    if not isinstance(keys, list) or not isinstance(values, list) or len(keys) != len(values):
        raise ValueError(f"malformed stats for ref {info.get('ref_id')}")
    if len(set(keys)) != len(keys):
        raise ValueError(f"duplicate stats key for ref {info.get('ref_id')}")
    output = dict(zip(keys, values))
    missing = [key for key in REQUIRED_STATS if key not in output]
    if missing:
        raise ValueError(f"missing stats for ref {info.get('ref_id')}: {missing}")
    return output


def close(a, b, tolerance=1e-5):
    return a is not None and b is not None and abs(float(a)-float(b)) <= tolerance * max(1.0, abs(float(b)))


def analyze(observations, gate_events, availability, prior_ids, log_text, expected):
    publishes = unique([r for r in observations if r.get("event") == "publish_begin"], "stamp_ns", "publish stamp")
    infos = unique([r for r in observations if r.get("event") == "info"], "stamp_ns", "Info stamp")
    if set(publishes) != set(infos):
        raise ValueError("published and Info stamp sets differ")
    indexes = [row["index"] for row in publishes.values()]
    if sorted(indexes) != list(range(len(indexes))):
        raise ValueError("publish indexes are not contiguous")
    by_index = unique(list(publishes.values()), "index", "publish index")
    ledger_rows = unique(availability.get("rows", []), "index", "availability index")
    if set(ledger_rows) != set(by_index):
        raise ValueError("availability and publication indexes differ")
    approved_stamps = []
    for event in gate_events:
        for approved in event.get("approved", []):
            approved_stamps.append(approved["stamp_ns"])
    if len(approved_stamps) != len(set(approved_stamps)):
        raise ValueError("duplicate approved stamp in gate events")
    approved_stamps = set(approved_stamps)
    if not approved_stamps <= set(publishes):
        raise ValueError("gate approved an unpublished stamp")
    info_by_ref = unique(list(infos.values()), "ref_id", "Info ref_id")
    refs = sorted(info_by_ref)
    if refs != list(range(refs[0], refs[0] + len(refs))):
        raise ValueError("Info ref_ids are not contiguous")
    logs = parse_logs(log_text, prior_ids)
    if not set(logs) <= set(info_by_ref):
        raise ValueError("rejection log current ref has no Info")

    output = []
    for index in range(len(by_index)):
        publish = by_index[index]
        stamp = publish["stamp_ns"]
        info = infos[stamp]
        ledger = ledger_rows[index]
        if ledger["stamp_ns"] != stamp:
            raise ValueError(f"ledger stamp mismatch at index {index}")
        values = stats(info)
        links = [value for value in (info.get("loop_closure_id", 0), info.get("proximity_detection_id", 0))
                 if value in prior_ids]
        linked = bool(links)
        ledger_no_prior = "info:no_prior_map_match" in ledger.get("raw_health_flags", [])
        if ledger_no_prior != (not linked):
            raise ValueError(f"ledger/Info prior membership mismatch at index {index}")
        if (ledger["primary_reason"] == "approved") != (stamp in approved_stamps):
            raise ValueError(f"ledger/gate approval mismatch at index {index}")
        rejection = logs.get(info["ref_id"])
        if linked and rejection:
            raise ValueError(f"linked ref also has rejection log: {info['ref_id']}")
        if rejection:
            if int(values["Loop/Highest_hypothesis_id/"]) != rejection["prior_id"]:
                raise ValueError(f"log/highest hypothesis mismatch for ref {info['ref_id']}")
            if not bool(values["Loop/RejectedHypothesis/"]):
                raise ValueError(f"log rejection lacks rejected stats flag for ref {info['ref_id']}")
            if rejection["kind"] == "geometry_inliers_rejected":
                if int(values["Loop/Visual_matches/"]) != rejection["matches"] or int(values["Loop/Visual_inliers/"]) != rejection["inliers"]:
                    raise ValueError(f"geometry log/stats mismatch for ref {info['ref_id']}")
            else:
                if OPTIMIZATION_RATIO not in values:
                    raise ValueError(f"graph rejection missing optimization ratio for ref {info['ref_id']}")
                if not close(values[OPTIMIZATION_RATIO], rejection["maximum_graph_error_ratio"]):
                    raise ValueError(f"graph ratio log/stats mismatch for ref {info['ref_id']}")
        if linked:
            category = "prior_link_observed"
        elif rejection:
            category = rejection["kind"]
        elif bool(values["Loop/RejectedHypothesis/"]):
            category = "rejected_hypothesis_stage_unknown"
        elif float(values["Loop/Visual_matches/"] or 0) == 0 and float(values["Loop/Visual_inliers/"] or 0) == 0:
            category = "candidate_screen_or_no_saved_geometry_evidence"
        else:
            category = "unlinked_unclassified_evidence"
        output.append({
            "index": index, "stamp_ns": stamp, "ref_id": info["ref_id"],
            "primary_reason": ledger["primary_reason"], "approved": stamp in approved_stamps,
            "prior_links": links, "category": category,
            "online_stats": {key: values[key] for key in REQUIRED_STATS},
            "optimization_max_error_ratio_present": OPTIMIZATION_RATIO in values,
            "optimization_max_error_ratio": values.get(OPTIMIZATION_RATIO),
            "rejection_log": rejection,
        })

    summary = {
        "published": len(output),
        "prior_link_observed": sum(bool(r["prior_links"]) for r in output),
        "approved": sum(r["approved"] for r in output),
        "primary_no_prior_match": sum(r["primary_reason"] == "no_prior_match" for r in output),
        "raw_no_prior_link": sum(not r["prior_links"] for r in output),
        "all_frames_by_category": dict(Counter(r["category"] for r in output)),
        "primary_no_prior_match_by_category": dict(Counter(r["category"] for r in output if r["primary_reason"] == "no_prior_match")),
        "raw_no_prior_link_by_category": dict(Counter(r["category"] for r in output if not r["prior_links"])),
    }
    for key, value in expected.items():
        if summary.get(key) != value:
            raise ValueError(f"baseline mismatch {key}: {summary.get(key)} != {value}")
    if sum(summary["all_frames_by_category"].values()) != len(output):
        raise ValueError("all-frame classification does not conserve population")
    if sum(summary["primary_no_prior_match_by_category"].values()) != summary["primary_no_prior_match"]:
        raise ValueError("primary-no-prior classification does not conserve population")
    if sum(summary["raw_no_prior_link_by_category"].values()) != summary["raw_no_prior_link"]:
        raise ValueError("raw-no-prior classification does not conserve population")
    return summary, output


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("observations", type=Path)
    parser.add_argument("gate_events", type=Path)
    parser.add_argument("availability", type=Path)
    parser.add_argument("database_audit", type=Path)
    parser.add_argument("database", type=Path)
    parser.add_argument("localization_log", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("output directory already exists")
    observations = load_jsonl(args.observations)
    gate_events = load_jsonl(args.gate_events)
    availability = json.loads(args.availability.read_text())
    database_audit = json.loads(args.database_audit.read_text())
    database_before = sha(args.database)
    if database_before != database_audit["database_sha256"]:
        raise ValueError("frozen database hash differs from database audit")
    connection = sqlite3.connect(f"file:{args.database}?mode=ro", uri=True)
    try:
        prior_ids = {row[0] for row in connection.execute("SELECT id FROM Node")}
    finally:
        connection.close()
    if prior_ids != set(database_audit["prior_node_ids"]):
        raise ValueError("SQLite Node IDs differ from database audit")
    if sha(args.database) != database_before:
        raise ValueError("database changed during read-only audit")
    expected = {"published": 230, "prior_link_observed": 78, "approved": 51,
                "primary_no_prior_match": 139, "raw_no_prior_link": 152}
    summary, ledger = analyze(observations, gate_events, availability, prior_ids,
                              args.localization_log.read_text(), expected)
    args.output.mkdir(parents=True)
    tool = Path(__file__)
    sources = [args.observations, args.gate_events, args.availability,
               args.database_audit, args.database, args.localization_log]
    receipt = {
        "protocol": "RTABMAP_MATCH_GAP_PROTOCOL_20260913",
        "git_head": subprocess.run(["git", "rev-parse", "HEAD"], text=True, capture_output=True).stdout.strip(),
        "python": platform.python_version(), "tool_sha256": sha(tool),
        "source_sha256": {str(path): sha(path) for path in sources},
        "prior_node_ids": sorted(prior_ids), "expected_baseline": expected,
        "database_sha256_before_after": [database_before, sha(args.database)],
        "limits": [
            "Only ref-addressed rejection log lines receive a mechanism label.",
            "Zero saved visual matches without a ref-addressed log is candidate screening or missing evidence, not proof that geometry was not attempted.",
            "Highest hypothesis id does not prove entry into geometric verification.",
        ],
    }
    for name, value in (("summary.json", summary), ("ledger.json", ledger), ("receipt.json", receipt)):
        with (args.output / name).open("x") as stream:
            json.dump(value, stream, indent=2, allow_nan=False)
            stream.write("\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
