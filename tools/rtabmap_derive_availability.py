#!/usr/bin/env python3
"""Derive bounded approval segments and primary classifications from frozen JSON.

Read-only derivation: it re-reads frozen run artifacts and reshapes them; no
algorithm is rerun and no measurement is repeated.

Origin: runs/rtabmap-reload-coordinate-20260912/derive_availability.py
(sha256 e9d937828daa994fe3be2680c58bc7fd6ca455cb5e780245b321dd26e8e74c37),
taken into the repository unchanged in derivation logic and output bytes.
"""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def derive(root):
    score_path = root / "normal-score.json"
    ledger_path = root / "normal-availability-ledger.json"
    score = json.loads(score_path.read_text())
    ledger = json.loads(ledger_path.read_text())
    records = score["records"]
    reasons = {row["index"]: row["primary_reason"] for row in ledger["rows"]}

    def segments(approved):
        output, current = [], []
        for row in records:
            if bool(row["approved"]) == approved:
                current.append(row)
            elif current:
                output.append(segment_record(current, approved))
                current = []
        if current:
            output.append(segment_record(current, approved))
        return output

    def segment_record(rows, approved):
        return {
            "local_range": [rows[0]["local_index"], rows[-1]["local_index"] + 1],
            "original_range": [rows[0]["original_index"], rows[-1]["original_index"] + 1],
            "frames": len(rows),
            "primary_reasons": {} if approved else
                dict(Counter(reasons[row["local_index"]] for row in rows)),
        }

    def threshold(group):
        rows = [row for row in records if group(row) and row["translation_error_m"] is not None]
        return {"samples": len(rows),
                "over_0_15m": sum(row["translation_error_m"] > 0.15 for row in rows),
                "max_m": max((row["translation_error_m"] for row in rows), default=None)}

    approved_segments = segments(True)
    primary = dict(ledger["summary"]["primary_reasons"])
    approved_count = primary.pop("approved")
    result = {
        "schema": 2,
        "protocol": "read-only derived availability; no algorithm rerun; approval fraction is not task success rate",
        "published": len(records),
        "healthy_prior_localization": score["denominators"]["healthy_prior_localization"],
        "approved": approved_count,
        "unapproved": len(records) - approved_count,
        "unapproved_primary_reasons": primary,
        "primary_classification_note": "Mutually exclusive ledger labels for reporting; raw flags may overlap and labels are not mechanism attribution.",
        "raw_overlapping_flags": ledger["summary"]["raw_flags"],
        "healthy_unapproved_reasons": ledger["summary"]["healthy_unapproved_reasons"],
        "approved_segments": approved_segments,
        "longest_approved_segment": max(approved_segments, key=lambda row: row["frames"], default=None),
        "unapproved_segments": segments(False),
        "absolute_position_threshold_reference": {
            "raw_localization": threshold(lambda row: row["numeric_pose"]),
            "healthy_prior": threshold(lambda row: row["healthy_prior_localization"]),
            "approved": threshold(lambda row: row["approved"]),
        },
        "completion": score["completion"],
        "limitations": [
            "primary_reason is reconstructed from application logs, not DDS arrival timestamps",
            "retirement_after_deadline is not uniquely attributable to transport, scheduling, or callback processing",
            "no_prior_match says the current frame lacked a member link to the pinned map; it does not uniquely identify perceptual root cause",
            "approval ratio is acquisition-count coverage, not navigation uptime or task success",
        ],
        "source_sha256": {score_path.name: digest(score_path), ledger_path.name: digest(ledger_path)},
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    result = derive(args.root)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({key: result[key] for key in
        ("published", "healthy_prior_localization", "approved", "unapproved",
         "unapproved_primary_reasons", "longest_approved_segment")}, indent=2))


if __name__ == "__main__":
    main()
