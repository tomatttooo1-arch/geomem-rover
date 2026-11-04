#!/usr/bin/env python3
"""Summarize a nano_loop run directory (ledger, memory ledger, tegrastats) into summary.json.

Warm-up rows are excluded. Position scatter (J2) clusters support_present positions per label
(greedy, 0.05 m) and reports the p95 distance to the cluster median in centimetres.
"""
import json
import os
import re
import sys

import numpy as np

CLUSTER_M = 0.05  # two real bottles sit ~8 cm apart; per-object scatter is ~1 cm


def pct(values, q):
    v = sorted(values)
    return None if not v else round(float(v[min(len(v) - 1, int(round(q * (len(v) - 1))))]), 3)


def stats(values):
    return {"n": len(values), "p50": pct(values, .5), "p95": pct(values, .95),
            "max": None if not values else round(float(max(values)), 3)}


def count(rows):
    out = {}
    for r in rows:
        out[r] = out.get(r, 0) + 1
    return dict(sorted(out.items(), key=lambda kv: -kv[1]))


def clusters(points):
    """points: list of (label, xyz). Greedy seeds by first appearance, nearest-seed assignment."""
    seeds = []
    for label, p in points:
        p = np.asarray(p)
        if not any(s[0] == label and np.linalg.norm(p - s[1]) <= CLUSTER_M for s in seeds):
            seeds.append((label, p))
    groups = {i: [] for i in range(len(seeds))}
    for label, p in points:
        p = np.asarray(p)
        cand = [(np.linalg.norm(p - s[1]), i) for i, s in enumerate(seeds) if s[0] == label]
        d, i = min(cand)
        if d <= CLUSTER_M:
            groups[i].append(p)
    rows = []
    for i, pts in groups.items():
        if not pts:
            continue
        a = np.array(pts)
        med = np.median(a, axis=0)
        r = np.linalg.norm(a - med, axis=1) * 100
        rows.append({"label": seeds[i][0], "n": len(pts), "median_xyz_m": [round(float(v), 4) for v in med],
                     "radius_cm_p50": round(float(np.percentile(r, 50)), 2),
                     "radius_cm_p95": round(float(np.percentile(r, 95)), 2),
                     "radius_cm_max": round(float(r.max()), 2),
                     "axis_std_cm": [round(float(v), 2) for v in a.std(axis=0) * 100]})
    return sorted(rows, key=lambda x: -x["n"])


def tegra(path):
    if not os.path.exists(path):
        return None
    vin, cpu, gpu = [], [], []
    for line in open(path):
        m = re.search(r"VDD_IN (\d+)mW", line)
        if m:
            vin.append(int(m.group(1)))
        m = re.search(r"CPU \[([^\]]+)\]", line)
        if m:
            loads = [int(x.split("%")[0]) for x in m.group(1).split(",") if "%" in x]
            if loads:
                cpu.append(sum(loads) / len(loads))
        m = re.search(r"GR3D_FREQ (\d+)%", line)
        if m:
            gpu.append(int(m.group(1)))
    return {"samples": len(vin), "vdd_in_mw": stats(vin), "vdd_in_mw_mean": round(float(np.mean(vin)), 1) if vin else None,
            "cpu_avg_all_cores_pct": stats(cpu), "gpu_gr3d_pct": stats(gpu)}


def timeline(run, bin_s=2.0):
    """Per-bin view for the owner's removal/occlusion runs: supports per label and watch outcomes."""
    rows = [json.loads(x) for x in open(os.path.join(run, "ledger.jsonl"))]
    frames = {r["color_index"]: r for r in rows if r["type"] == "frame"}
    t0 = min(r["acquired_ns"] for r in frames.values())
    bins = {}
    for r in frames.values():
        b = bins.setdefault(int((r["acquired_ns"] - t0) / 1e9 // bin_s), {"frames": 0, "geometry_invalid": 0})
        b["frames"] += 1
        b["geometry_invalid"] += not r["geometry"]["valid"]
    for d in rows:
        if d["type"] != "det" or d["color_index"] not in frames:
            continue
        b = bins[int((frames[d["color_index"]]["acquired_ns"] - t0) / 1e9 // bin_s)]
        for i in d["instances"]:
            if i["candidate"] == "support_present":
                k = "support:" + i["label"]
                b[k] = b.get(k, 0) + 1
        for n in d["negatives"]:
            k = "watch:%s:%s:%s" % (n["label"], n["candidate"], n["reason"])
            b[k] = b.get(k, 0) + 1
    for k in sorted(bins):
        print("%5.0fs %s" % (k * bin_s, json.dumps(bins[k], sort_keys=True)))


def main(run):
    rows = [json.loads(x) for x in open(os.path.join(run, "ledger.jsonl"))]
    frames = [r for r in rows if r["type"] == "frame" and not r["warmup"]]
    dets = [r for r in rows if r["type"] == "det" and not r["warmup"] and r["result_age_ms"] is not None]
    inst = [i for d in dets for i in d["instances"]]
    neg = [n for d in dets for n in d["negatives"]]
    s = {"run": os.path.basename(run.rstrip("/")),
         "frames_processed": len(frames),
         "first_last_color_index": [frames[0]["color_index"], frames[-1]["color_index"]] if frames else None,
         "wall_span_s": round((frames[-1]["acquired_ns"] - frames[0]["acquired_ns"]) / 1e9, 2) if frames else None,
         "drops": {"scheduled_frames_skipped_loop_busy": sum(r["skipped_scheduled"] for r in frames),
                   "detector_skipped_busy": sum(r["det"] != "submitted" for r in frames)},
         "geometry_valid": sum(r["geometry"]["valid"] for r in frames),
         "geometry_invalid_reasons": count(r["geometry"]["reason"] for r in frames if not r["geometry"]["valid"]),
         "valid_depth_frac": stats([r["geometry"]["valid_depth_frac"] for r in frames]),
         "timing_ms": {"recv_to_take": stats([r["recv_to_take_ms"] for r in frames]),
                       "rgb_sha256": stats([r["sha_ms"] for r in frames]),
                       "geometry_ingest_cpu_reference": stats([r["geometry_ms"] for r in frames]),
                       "main_loop_per_frame": stats([r["loop_ms"] for r in frames]),
                       "det_pre": stats([d["det"]["pre_ms"] for d in dets]),
                       "det_infer": stats([d["det"]["infer_ms"] for d in dets]),
                       "det_post": stats([d["det"]["post_ms"] for d in dets]),
                       "det_queue": stats([d["det"]["queue_ms"] for d in dets]),
                       "semantic_evidence_and_m4_per_batch": stats([d["sem_ms"] for d in dets]),
                       "result_age_recv_to_evidence": stats([d["result_age_ms"] for d in dets]),
                       "overlay": stats([d["overlay_ms"] for d in dets if "overlay_ms" in d])},
         "detections": {"batches_consumed": len(dets), "late_batches": sum(d["late"] for d in dets),
                        "late_index_gap": count(d["index_gap"] for d in dets if d["late"]),
                        "instances": len(inst), "by_label": count(i["label"] for i in inst)},
         "evidence": count("%s:%s" % (i["candidate"], i["reason"]) for i in inst),
         "evidence_by_label": {lab: count("%s:%s" % (i["candidate"], i["reason"]) for i in inst if i["label"] == lab)
                               for lab in sorted(set(i["label"] for i in inst))},
         "removal_watch": count("%s:%s" % (n["candidate"], n["reason"]) for n in neg),
         "removal_watch_free_space_conflict": sum(n["candidate"] == "free_space_conflict" for n in neg),
         "m4_status": count((d["m4"] or "none").split(":")[0] + ":" + ":".join((d["m4"] or "").split(":")[1:2]) for d in dets),
         "m4_sent": sum(bool(d.get("m4_sent")) for d in dets)}
    s["position_scatter_J2"] = clusters([(i["label"], i["position_m"]) for i in inst
                                         if i["candidate"] == "support_present"])
    mem = os.path.join(run, "memory_ledger.jsonl")
    if os.path.exists(mem):
        m = [json.loads(x) for x in open(mem)]
        imp = [x for x in m if "import_ms" in x]
        s["memory"] = {"imports": len(imp), "failed": sum(not x["ok"] for x in imp),
                       "import_ms": stats([x["import_ms"] for x in imp]),
                       "import_ms_first5_last5": [x["import_ms"] for x in imp[:5]] + ["..."] + [x["import_ms"] for x in imp[-5:]],
                       "lag_ms": stats([x["lag_ms"] for x in imp]),
                       "save_ms": stats([x["save_ms"] for x in m if "save_ms" in x]),
                       "decisions": count(k for x in imp if x["ok"] for k, v in x["decisions"].items() for _ in range(v))}
    end = os.path.join(run, "run_end.json")
    if os.path.exists(end):
        s["run_end"] = json.load(open(end))
    s["tegrastats"] = tegra(os.path.join(run, "tegrastats.log"))
    json.dump(s, open(os.path.join(run, "summary.json"), "w"), indent=1)
    print(json.dumps(s, indent=1))


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[2] == "--timeline":
        timeline(sys.argv[1])
    else:
        main(sys.argv[1])
