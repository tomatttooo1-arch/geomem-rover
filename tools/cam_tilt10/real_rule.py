#!/usr/bin/env python3
"""Camera tilt 10 deg, baton 2: the real c10_1 depth (board 10-17 14:31, 10 frames) under the shallow-pitch far range
cap of tools/car_run/ground_calib.py (range_for), against docs/design/CAM_TILT_10_20261017.md section 9.2 V2.
Reads the npz, writes OUT.json only.

  real_rule.py NPZ OUT.json
Python 3.8 compatible."""
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.car_run import ground_calib as gc  # noqa: E402

REFS = {"c10_1_range2.0": (0.39152200037918883, 7.046382820144615, 0.5897540677563523),   # same frames, old range
        "c10b_2_board_check": (0.3916232419640143, 7.030135258586666, 0.6153997151745129)}  # another capture
TOL_DEG, TOL_M = 0.3, 0.005


def row(frames, K, h, p, mode):
    r = gc.calibrate(frames, K, h, p, mode=mode)
    q = r.get("quality") or {}
    f = q.get("full") or {}
    out = {"configured": [h, p], "mode": mode, "verdict": r["verdict"], "reasons": r["reasons"],
           "range_m": r["gates"]["range_m"], "predicted": f.get("predicted_ground_samples"), "inliers": f.get("inliers"),
           "share": f.get("inlier_share"), "rms_mm": None if f.get("rms_m") is None else 1000 * f["rms_m"],
           "bend_mm": None if f.get("bend_m") is None else 1000 * f["bend_m"],
           "halves_deg": q.get("halves_delta_deg"), "halves_mm": None if q.get("halves_delta_m") is None
           else 1000 * q["halves_delta_m"], "fitted": r.get("fitted"), "adopted": r.get("adopted")}
    fi = r.get("fitted")
    if fi:
        out["vs_refs"] = {}
        for name, (rh, rp, rr) in REFS.items():
            e = {"height_mm": 1000 * abs(fi["height_m"] - rh), "pitch_deg": abs(fi["pitch_deg"] - rp),
                 "roll_deg": abs(fi["roll_deg"] - rr)}
            e["within"] = e["height_mm"] <= 1000 * TOL_M and e["pitch_deg"] <= TOL_DEG and e["roll_deg"] <= TOL_DEG
            out["vs_refs"][name] = e
    quality_ok = (f.get("inliers", 0) >= gc.MIN_INLIERS and f.get("inlier_share", 0) >= gc.MIN_INLIER_SHARE
                  and f.get("rms_m", 1) <= gc.MAX_RMS_M and f.get("bend_m", 1) <= gc.BEND_MAX_M
                  and (q.get("halves_delta_deg") or 9) <= gc.HALF_MAX_DEG and (q.get("halves_delta_m") or 9) <= gc.HALF_MAX_M)
    out["quality_gates_pass"] = bool(quality_ok)
    out["only_hard_height"] = bool(r["verdict"] == "reject" and len(r["reasons"]) == 1 and "硬限" in r["reasons"][0]
                                   and "高度" in r["reasons"][0])
    return out


def main():
    npz, out = sys.argv[1], sys.argv[2]
    z = np.load(npz)
    frames, K = list(z["frames"]), np.asarray(z["K"], dtype=np.float64)
    res = {"npz": npz, "rows": []}
    for h, p, mode in ((0.39, 7.0, "auto"), (0.39, 7.0, "check"), (0.375, 10.0, "auto"), (0.375, 10.0, "check"),
                       (0.39, 8.5, "auto"), (0.375, 7.0, "auto"), (0.39, 10.0, "auto")):
        res["rows"].append(row(frames, K, h, p, mode))
    r = {(x["configured"][0], x["configured"][1], x["mode"]): x for x in res["rows"]}
    a, b, c = r[(0.39, 7.0, "auto")], r[(0.39, 7.0, "check")], r[(0.375, 10.0, "auto")]
    res["V2"] = {
        "a": a["quality_gates_pass"] and a["share"] >= 0.5 and all(v["within"] for v in a["vs_refs"].values())
        and a["only_hard_height"],
        "b": b["verdict"] == "confirm",
        "c": c["quality_gates_pass"] and all(v["within"] for v in c["vs_refs"].values()) and c["only_hard_height"]}
    # section 9.4 V7 (hard height 0.395): auto adopts, the adopted mount within the criterion of both references
    def adopted_ok(x):
        return (x["verdict"] == "adopt" and all(v["within"] for v in x["vs_refs"].values())
                and x["adopted"]["source"] == "fit")
    res["V7"] = {"a": adopted_ok(a), "b": b["verdict"] == "confirm", "c": adopted_ok(c),
                 "hard_height_m": list(gc.HARD_HEIGHT_M)}
    with open(out, "w") as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    for x in res["rows"]:
        print(x["configured"], x["mode"], x["verdict"], x["range_m"], "pred", x["predicted"], "inl", x["inliers"],
              "share %.4f" % x["share"], "rms %.2f bend %.3f halves %.3f/%.2f" % (x["rms_mm"], x["bend_mm"],
                                                                                  x["halves_deg"], x["halves_mm"]),
              "fit", None if not x["fitted"] else [round(x["fitted"][k], 4) for k in ("height_m", "pitch_deg", "roll_deg")],
              {k: [round(v["height_mm"], 2), round(v["pitch_deg"], 3), round(v["roll_deg"], 3), v["within"]]
               for k, v in (x.get("vs_refs") or {}).items()}, [s[:40] for s in x["reasons"]])
    print("V2 (hard 0.39 era)", res["V2"])
    print("V7", res["V7"])


if __name__ == "__main__":
    main()
