#!/usr/bin/env python3
"""Query the P3 memory store by category name; prints candidates, never "currently there".

    python3 -m tools.nano_loop.query --store <run>/memory.json bottle [laptop ...]

Uses the unchanged OfflineMemory.load/query; the scope statement is copied verbatim.
"""
import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from perception.offline_memory import OfflineMemory  # noqa: E402


def summarize(memory, label):
    q = memory.query(label)
    obs = memory.data["observations"]
    rows = []
    for o in q["objects"]:
        items = [obs[k] for k in o["observation_ids"]]
        scores = sorted(x["raw"]["score"] for x in items)
        fracs = sorted(x["raw"]["geometry"].get("valid_fraction", 0) for x in items
                       if x["raw"]["geometry"]["valid"])
        rows.append({
            "object_id": o["id"], "label": o["label"], "state": o["state"], "reason": o["reason"],
            "position_m": None if o["position_m"] is None else [round(v, 3) for v in o["position_m"]],
            "frame": o["world_frame"] + " (fixed camera, identity pose; not a localized map)",
            "confidence": {"score_median": scores[len(scores) // 2], "score_max": scores[-1],
                           "valid_depth_fraction_median": fracs[len(fracs) // 2] if fracs else None},
            "last_seen_frame": o["last_seen_frame"], "last_seen_ns": o["last_seen"],
            "time_source": o["time_source"],
            "physical_observation_count": o["physical_observation_count"],
            "evidence_source": {"pose_source": o["pose_source"],
                                "prediction_set": items[-1]["batch"]["prediction_set"],
                                "detector": items[-1]["raw"]["source"],
                                "summary_method": items[-1]["raw"]["geometry"].get("summary_method"),
                                "evidence_refs_count": len(o["evidence_refs"]),
                                "latest_refs": o["evidence_refs"][-2:]}})
    pending = {}
    for p in q["pending_observations"]:
        pending[p["decision"]] = pending.get(p["decision"], 0) + 1
    return {"category": q["category"], "candidates": rows, "pending_observations_by_decision": pending,
            "scope": q["scope"]}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--store", required=True)
    ap.add_argument("labels", nargs="+")
    a = ap.parse_args(argv)
    memory = OfflineMemory.load(a.store)
    print(json.dumps([summarize(memory, x) for x in a.labels], ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
