"""Detection matching / judging for the line-3 board TRT comparison (PREREG.md section 2).

`iou`, `match`, `compare` are copied verbatim (logic) from line A's
runs/yolo-runner-20261014-results/scripts/compare_ort_vs_ultralytics.py: per class, greedy
one-to-one by IoU descending, pairs need IoU >= 0.5. `judge` is line A's judge with the score gate
as a parameter (B3: T = max(0.05, 2*d5), fixed in PREREG's revision section before the full run).
Pure python (no numpy) so it imports anywhere.
"""
CONF = 0.25
BOUNDARY = CONF + 0.02


def iou(a, b):
    w = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); h = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = w * h
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def match(ref, test):
    """Per class, greedy one-to-one by IoU (descending), pairs need IoU >= 0.5."""
    pairs, used_r, used_t = [], set(), set()
    cand = []
    for i, r in enumerate(ref):
        for j, t in enumerate(test):
            if r["name"] == t["name"]:
                v = iou(r["box"], t["box"])
                if v >= 0.5:
                    cand.append((v, i, j))
    for v, i, j in sorted(cand, key=lambda c: -c[0]):
        if i in used_r or j in used_t:
            continue
        used_r.add(i); used_t.add(j)
        pairs.append({"name": ref[i]["name"], "iou": v, "score_ref": ref[i]["score"], "score_test": test[j]["score"],
                      "dscore": abs(ref[i]["score"] - test[j]["score"]), "box_ref": ref[i]["box"], "box_test": test[j]["box"]})
    un_r = [ref[i] for i in range(len(ref)) if i not in used_r]
    un_t = [test[j] for j in range(len(test)) if j not in used_t]
    return pairs, un_r, un_t


def compare(frames, ref_all, test_all):
    per = []
    for name in frames:
        pairs, ur, ut = match(ref_all[name], test_all[name])
        per.append({"frame": name, "n_ref": len(ref_all[name]), "n_test": len(test_all[name]),
                    "pairs": pairs, "unmatched_ref": ur, "unmatched_test": ut})
    return per


def judge(per_frame, score_gate, iou_gate=0.9, boundary=BOUNDARY, min_pos_frames=20, min_pairs=40):
    """B1-B4 (line A's A1-A4 with the B3 gate as a parameter)."""
    pairs = [p for f in per_frame for p in f["pairs"]]
    unmatched = [dict(u, frame=f["frame"], side=side) for f in per_frame
                 for side, lst in (("ref", f["unmatched_ref"]), ("test", f["unmatched_test"])) for u in lst]
    boundary_list = [u for u in unmatched if u["score"] < boundary]
    hard = [u for u in unmatched if u["score"] >= boundary]
    frames_pos = sum(1 for f in per_frame if f["n_ref"] > 0)
    out = {
        "frames": len(per_frame), "ref_frames_with_detection": frames_pos,
        "ref_detections": sum(f["n_ref"] for f in per_frame), "test_detections": sum(f["n_test"] for f in per_frame),
        "pairs": len(pairs),
        "min_iou": min((p["iou"] for p in pairs), default=None),
        "max_dscore": max((p["dscore"] for p in pairs), default=None),
        "unmatched_boundary": boundary_list, "unmatched_hard": hard,
        "count_mismatch_frames": [f["frame"] for f in per_frame if f["n_ref"] != f["n_test"]],
        "score_gate": score_gate, "iou_gate": iou_gate,
    }
    out["B1_classes_same_no_unmatched"] = not hard
    out["B2_iou_ge_gate"] = bool(pairs) and out["min_iou"] >= iou_gate
    out["B3_dscore_le_gate"] = bool(pairs) and out["max_dscore"] <= score_gate
    out["B4_positive_control"] = frames_pos >= min_pos_frames and len(pairs) >= min_pairs
    out["PASS"] = all(out[k] for k in ("B1_classes_same_no_unmatched", "B2_iou_ge_gate", "B3_dscore_le_gate",
                                       "B4_positive_control"))
    return out


def brief(j):
    """judge() output without the long unmatched lists (counts instead)."""
    b = {k: v for k, v in j.items() if k not in ("unmatched_boundary", "unmatched_hard")}
    b["n_unmatched_boundary"] = len(j["unmatched_boundary"])
    b["n_unmatched_hard"] = len(j["unmatched_hard"])
    return b
