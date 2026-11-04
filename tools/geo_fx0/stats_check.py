#!/usr/bin/env python3
"""GEO_FX0 gate S: the PL depth statistics (golden model columns st_*) give the same verdicts as the Nano numpy checks
(original pre-NANOSW0 formulas) on codes_to_m(codes). Real frames from data/<name>.csv + packs; synthetic frames computed
with a Python mirror of the stats unit. Usage: stats_check.py OUTJSON"""
import csv, json, struct, sys
import numpy as np
R = "${FPGA_RUN}"
G = R + "/GEO_FX0"
PACK = {"rec30": R + "/E2/packs/rec30.pack", "car570": R + "/E2/packs/car570.pack"}


def frames(path):
    with open(path, "rb") as f:
        n = struct.unpack("<I", f.read(24)[12:16])[0]
        for _ in range(n):
            h = f.read(144)
            w, hh = struct.unpack("<2I", h[:8])
            unit = np.float32(struct.unpack("<f", h[24:28])[0])
            yield unit, np.frombuffer(f.read(w * hh * 2), "<u2")


def to_m(codes, unit):
    d = codes.astype(np.float32) * unit
    d[codes == 0] = np.nan
    return d


def thresholds(unit):
    c = np.arange(65536, dtype=np.uint32)
    z = c.astype(np.float32) * unit
    ok = (c >= 1) & (z > np.float32(0.1)) & (z < np.float32(10))
    return int(c[ok].min()) if ok.any() else 65536, int(c[ok].max()) if ok.any() else 0, int(c[(c >= 1) & (z <= 20)].max())


def stats(codes, c_lo, c_hi, c20):  # python mirror of the PL unit (used for synthetic frames only)
    pos = codes >= 1
    le = pos & (codes <= c20)
    gt = codes > c20
    v = (codes >= c_lo) & (codes <= c_hi)
    return dict(npos=int(pos.sum()), le20=int(le.sum()), maxle20=int(codes[le].max()) if le.any() else 0,
                mingt20=int(codes[gt].min()) if gt.any() else 0, valid=int(v.sum()),
                minvalid=int(codes[v].min()) if v.any() else 0, maxcode=int(codes.max()) if codes.size else 0)


def verdict_stats(s, unit, c20):
    f = lambda c: np.float32(c) * unit
    n, c = s["npos"], s["le20"]
    k = n // 2
    if n == 0:
        med = False
    elif n % 2:
        med = c <= k
    elif c != k:
        med = c < k
    else:
        med = bool(np.median(np.array([f(s["maxle20"]), f(s["mingt20"])], np.float32)) > 20)
    return dict(isinf=False, gt20=s["maxcode"] > c20, all_invalid=s["valid"] == 0, median_gt20=med,
                nearest=float(f(s["minvalid"])) if s["valid"] else None)


def verdict_numpy(depth):  # original Nano formulas (pre-NANOSW0)
    with np.errstate(invalid="ignore"):
        valid = (depth > .1) & (depth < 10) & np.isfinite(depth)
        vpos = depth[np.isfinite(depth) & (depth > 0)]
        return dict(isinf=bool(np.isinf(depth).any() or np.isneginf(depth).any()),
                    gt20=bool(np.isfinite(depth).any() and np.nanmax(depth) > 20),
                    all_invalid=not bool(np.any(valid)), median_gt20=bool(vpos.size and np.median(vpos) > 20),
                    nearest=float(np.min(depth[valid])) if valid.any() else None)


def diff(a, b):
    return [k for k in a if a[k] != b[k]]


def main():
    out = {"real": {}, "synthetic": []}
    for name, pack in PACK.items():
        rows = list(csv.DictReader(open("%s/data/%s.csv" % (G, name))))
        bad, cnt = [], {"gt20": 0, "all_invalid": 0, "median_gt20": 0}
        for (unit, codes), r in zip(frames(pack), rows):
            c20 = int(r["c20"])
            s = dict(npos=int(r["st_npos"]), le20=int(r["st_le20"]), maxle20=int(r["st_maxle20"]),
                     mingt20=int(r["st_mingt20"]), valid=int(r["st_valid"]), minvalid=int(r["st_minvalid"]),
                     maxcode=int(r["st_maxcode"]))
            a, b = verdict_stats(s, unit, c20), verdict_numpy(to_m(codes, unit))
            for k in cnt:
                cnt[k] += bool(b[k])
            if diff(a, b):
                bad.append((int(r["frame"]), diff(a, b)))
        out["real"][name] = {"frames": len(rows), "diff_frames": len(bad), "examples": bad[:5], "positives": cnt}
        print(name, out["real"][name], flush=True)
    # synthetic: unit x1000, all-zero codes, median straddling 20 m (several units), only >20, odd/even sizes
    rng = np.random.RandomState(20261103)
    base = next(frames(PACK["rec30"]))[1]
    cases = [("unit_x1000", np.float32(1.0), base), ("all_zero", np.float32(0.001), np.zeros(1000, np.uint16))]
    for unit in (np.float32(0.001), np.float32(0.0005), np.float32(2 ** -10), np.float32(0.00125)):  # 0.0002: c20 = 65535, no code above 20 m
        _, _, c20 = thresholds(unit)
        for n in (10, 11, 1000, 1001):
            k = n // 2
            lo = rng.randint(1, c20 + 1, size=n).astype(np.uint16)
            hi = rng.randint(c20 + 1, 65536, size=n).astype(np.uint16)
            arr = np.concatenate([lo[:k], hi[:n - k]])           # c = k: the straddle branch
            arr2 = np.concatenate([lo[:k], [c20, c20 + 1], hi[:n - k - 2]])[:n].astype(np.uint16)
            arr3 = np.concatenate([np.full(k, c20, np.uint16), np.full(n - k, c20 + 1, np.uint16)])
            arr4 = np.concatenate([arr, np.zeros(7, np.uint16)])  # with invalid zeros
            for tag, a in (("straddle", arr), ("straddle_adj", arr2), ("straddle_tight", arr3), ("with_zeros", arr4)):
                cases.append(("%s_u%.6g_n%d" % (tag, unit, n), unit, rng.permutation(a)))
    nbad = 0
    for tag, unit, codes in cases:
        c_lo, c_hi, c20 = thresholds(unit)
        a, b = verdict_stats(stats(codes, c_lo, c_hi, c20), unit, c20), verdict_numpy(to_m(codes, unit))
        d = diff(a, b)
        nbad += bool(d)
        out["synthetic"].append({"case": tag, "diff": d, "median_gt20": b["median_gt20"], "gt20": b["gt20"],
                                 "all_invalid": b["all_invalid"]})
    out["synthetic_cases"] = len(cases)
    out["synthetic_diff"] = nbad
    out["synthetic_median_true"] = sum(1 for c in out["synthetic"] if c["median_gt20"])
    print("synthetic", len(cases), "diff", nbad, "median_true", out["synthetic_median_true"])
    out["S_pass"] = nbad == 0 and all(v["diff_frames"] == 0 for v in out["real"].values())
    json.dump(out, open(sys.argv[1], "w"), indent=1)


if __name__ == "__main__":
    main()
