#!/usr/bin/env python3
"""书包门槛读数（10-25 验收）：列记忆里某类（默认 backpack）物体的检测分数，并在给出真假标注时求能分开的门槛区间。

    python3 tools/car_run/backpack_gate.py <run_dir | memory.json> [--label backpack] [--gate 0.45]
                                           [--truth object-0001=1,object-0002=0] [--json OUT]

读 RUN/memory_final.json，没有就 RUN/memory_snapshot.json。每条 track 的 history 只存最后 20 条
（host/memory_store.py HISTORY_KEEP），分数统计只覆盖这 20 条，history_len 是全程条数。
现行门槛 perception/fpga_replay/yolo_detector.py DEFAULT_CLASS_MIN_SCORES backpack 0.45（car_up.sh CAR_CLASS_MIN_SCORE 可改）。
--truth：id=1 真书包、id=0 假（误检）。按每条 track 的中位分数比较：真的最小中位 > 假的最大中位 时，
区间 (假最大, 真最小] 内的门槛能分开；否则「单靠门槛分不开」。只读，标准库，py3.8。
"""
import argparse
import json
import os
import sys


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))] if v else None


def median(v):
    v = sorted(v)
    if not v:
        return None
    n = len(v)
    return v[n // 2] if n % 2 else (v[n // 2 - 1] + v[n // 2]) / 2.


def load(src):
    if os.path.isdir(src):
        for name in ("memory_final.json", "memory_snapshot.json"):
            p = os.path.join(src, name)
            if os.path.isfile(p):
                src = p
                break
        else:
            return None, None
    try:
        with open(src, "r", encoding="utf-8") as f:
            return json.load(f), src
    except (OSError, IOError, ValueError):
        return None, src


def parse_truth(s):
    out = {}
    for item in (s or "").split(","):
        if "=" in item:
            k, v = item.split("=", 1)
            out[k.strip()] = v.strip() in ("1", "true", "yes", "真")
    return out


def gate(src, label="backpack", current=0.45, truth=None):
    mem, path = load(src)
    if not isinstance(mem, dict):
        return {"verdict": "NODATA", "note": "无 memory_final.json / memory_snapshot.json", "file": path}
    rows = []
    for t in mem.get("tracks") or []:
        if t.get("label") != label:
            continue
        scores = [h["score"] for h in (t.get("history") or []) if isinstance(h.get("score"), (int, float))]
        rows.append({"id": t.get("id"), "state": t.get("state"), "position_m": t.get("position_m"),
                     "history_len": t.get("history_len"), "scores_kept": len(scores),
                     "median": round(median(scores), 4) if scores else None,
                     "p90": pct(scores, .9), "max": max(scores) if scores else None,
                     "kept_at_current_gate": sum(1 for s in scores if s >= current)})
    out = {"file": path, "label": label, "current_gate": current, "tracks": rows,
           "note": "分数只覆盖每条 track 最后 20 条 history（history_len 为全程条数）"}
    if not rows:
        out["verdict"] = "NODATA"
        return out
    truth = truth or {}
    if truth:
        real = [r for r in rows if truth.get(r["id"]) is True and r["median"] is not None]
        fake = [r for r in rows if truth.get(r["id"]) is False and r["median"] is not None]
        out["unlabelled"] = [r["id"] for r in rows if r["id"] not in truth]
        if real and fake:
            lo, hi = max(r["median"] for r in fake), min(r["median"] for r in real)
            if hi > lo:
                out["separating_gate"] = [lo, hi]
                out["verdict"] = "PASS" if lo < current <= hi else "FAIL"
                out["why"] = "门槛取 (%.3f, %.3f] 能分开；现行 %.2f %s" % (
                    lo, hi, current, "在区间内" if out["verdict"] == "PASS" else "不在区间内")
            else:
                out["separating_gate"] = None
                out["verdict"] = "FAIL"
                out["why"] = "真最小中位 %.3f <= 假最大中位 %.3f：单靠门槛分不开" % (hi, lo)
        else:
            out["verdict"] = "NODATA"
            out["why"] = "真假两类至少各需一条带分数的 track"
        out["current_gate_keeps"] = [r["id"] for r in rows if r["median"] is not None and r["median"] >= current]
        out["current_gate_drops"] = [r["id"] for r in rows if r["median"] is not None and r["median"] < current]
    else:
        out["verdict"] = "INFO"
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("src")
    ap.add_argument("--label", default="backpack")
    ap.add_argument("--gate", type=float, default=0.45)
    ap.add_argument("--truth", default="")
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    r = gate(os.path.realpath(a.src), a.label, a.gate, parse_truth(a.truth))
    print("%s 门槛读数（现行 %.2f）：%s" % (a.label, a.gate, r.get("verdict")))
    for t in r.get("tracks") or []:
        print("  %s state=%s 全程%s条/留%s条 中位=%s p90=%s 最大=%s 现行门槛下保留%s条" % (
            t["id"], t["state"], t["history_len"], t["scores_kept"], t["median"], t["p90"], t["max"],
            t["kept_at_current_gate"]))
    for k in ("why", "separating_gate", "current_gate_keeps", "current_gate_drops", "unlabelled", "note", "file"):
        if k in r:
            print("  %s: %s" % (k, r[k]))
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(r, f, ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
