#!/usr/bin/env python3
"""GEO_FX0 G7: REF vs FX (and FXC) after the rolling fusion core and the B1 costmap core (product sources at the
GEO_FX0 worktree = 59fcf44). Same harness as DLAT_LIVE scripts/dlat_live.py, B mode only (fresh map per frame,
the same frame integrated 3 times), product parameters. Usage: downstream.py OUTJSON [names]."""
import json, os, sys, tempfile, time
import numpy as np

WT = "${FPGA_SRC}"
sys.path.insert(0, WT)
from host import rolling_nav_layer as rnl  # noqa: E402
from tools.b1_costmap import nav2_ref  # noqa: E402
from tools.b1_costmap.bench import build_costmap, PlCore, band_params, body_q, V, LUT_DIM  # noqa: E402
from tools.b1_costmap.bench_int import build_roll  # noqa: E402

G = "${FPGA_RUN}/GEO_FX0"
KIND = {"rec30": "cam", "car570": "car"}
OP_RESET, OP_INTEGRATE, OP_DUMP = 0, 1, 3
GROUND_Z, ROBOT_H, BODY_R, BODY_MARGIN = 0.0, 0.31, 0.30, 0.05
T_OCC, T_FREE = 16, -16


def load(name, arm):
    a = np.fromfile("%s/data/%s.%s.flags" % (G, name, arm), np.uint8).reshape(-1, 32768)
    if KIND[name] == "car":
        return a[:, :32 * 32 * 10].copy()
    c = a.reshape(-1, 32, 32, 32)[:, :, ::-1, ::-1]   # camera axes -> body-like (as DLAT_LIVE)
    return np.ascontiguousarray(c.transpose(0, 2, 3, 1)).reshape(-1, 32768)


def tri_map(dump):
    v = np.frombuffer(np.ascontiguousarray(dump, np.uint64).tobytes(), np.int8).astype(np.int32)
    s = np.zeros(v.shape, np.uint8)
    s[v >= T_OCC] = 2
    s[v <= T_FREE] = 1
    return s


class Arm(object):
    def __init__(self, so, cm, geo):
        self.core, self.cm, self.geo, self.epoch, self.seq = build_roll(so), cm, geo, 0, 0

    def frame(self, flags, reps=3):
        self.epoch += 1
        self.seq = 0
        h, _, _, _ = self.core.call(OP_RESET, self.epoch, 0, self.geo["wb"])
        assert int(h[0]) == 0
        for _ in range(reps):
            self.seq += 1
            h, _, _, _ = self.core.call(OP_INTEGRATE, self.epoch, self.seq, self.geo["wb"], self.geo["ob"],
                                        self.geo["dims"], obs=flags.tobytes())
            assert int(h[0]) == 0
        h, _, _, dump = self.core.call(OP_DUMP)
        offs = [int(h[8 + k]) for k in range(3)]
        g = self.geo
        st, grid, aux, cost = self.cm.run(dump, offs, g["band"], g["tall_lo"], g["body"], g["lut"])
        assert st == 0
        return tri_map(dump), cost.copy()


def cmp(r, x):
    t0, c0 = r
    t1, c1 = x
    known = int(((t0 != 0) | (t1 != 0)).sum())
    mis3 = int((t0 != t1).sum())
    info = int((c0 <= 253).sum())
    return {"known3": known, "mis3": mis3, "info": info, "cost_diff": int((c0 != c1).sum()),
            "lethal_ref": int((c0 == 254).sum()), "lethal_lost": int(((c0 == 254) & (c1 != 254)).sum()),
            "lethal_extra": int(((c1 == 254) & (c0 != 254)).sum()),
            "occ3_lost": int(((t0 == 2) & (t1 != 2)).sum()), "occ3_ref": int((t0 == 2).sum())}


def judge(rows, num, den, small=1000):
    worst, small_n, small_d, fails = 0.0, 0, 0, 0
    for r in rows:
        if r[den] < small:
            small_n += r[num]; small_d += r[den]
        else:
            f = r[num] / r[den]
            worst = max(worst, f)
            fails += f > 1e-3
    sm = small_n / small_d if small_d else 0.0
    return {"frames_big": sum(1 for r in rows if r[den] >= small), "worst_frame_frac": worst, "frames_over": fails,
            "small_merged_frac": sm, "small_frames": sum(1 for r in rows if r[den] < small),
            "pass": fails == 0 and sm <= 1e-3, "total_num": sum(r[num] for r in rows), "total_den": sum(r[den] for r in rows)}


def main():
    out = sys.argv[1]
    names = sys.argv[2].split(",") if len(sys.argv) > 2 else ["rec30", "car570"]
    scratch = tempfile.mkdtemp(prefix="geofx0_ds_", dir=os.environ.get("TMPDIR"))
    cm = PlCore(build_costmap(os.path.join(scratch, "cm.so")))
    lut = nav2_ref.InflationParams(resolution=V, inflation_radius=0.45, cost_scaling_factor=3.0,
                                   robot_radius=BODY_R).pl_lut(LUT_DIM)
    band = rnl.HeightBand(ground_z_m=GROUND_Z, robot_height_m=ROBOT_H)
    res = {"worktree": WT, "mode": "B (fresh map, same frame x3)", "datasets": {}}
    for name in names:
        t0 = time.time()
        arms_data = {k: load(name, k) for k in ("ref", "fx", "fxc")}
        if KIND[name] == "car":
            wb = (-40, -40, -8)
            bp, tall_lo = band_params(wb[2], band)
            geo = dict(wb=wb, ob=(1, -16, -1), dims=(32, 32, 10), band=bp, tall_lo=tall_lo,
                       body=body_q(wb, (0.0, 0.0), BODY_R + BODY_MARGIN), lut=lut)
        else:
            geo = dict(wb=(-40, -40, 0), ob=(-32, -16, 0), dims=(32, 32, 32), band=(0, 31), tall_lo=32,
                       body=(0, 0, 0, 0), lut=lut)
        arms = {k: Arm(os.path.join(scratch, "roll_%s_%s.so" % (name, k)), cm, geo) for k in ("ref", "fx", "fxc", "ref2")}
        rows = {"fx": [], "fxc": [], "ref2": []}
        n = len(arms_data["ref"])
        for i in range(n):
            snap = {k: arms[k].frame(arms_data["ref" if k == "ref2" else k][i]) for k in arms}
            for k in rows:
                r = cmp(snap["ref"], snap[k]); r["frame"] = i; rows[k].append(r)
        d = {}
        for k, rr in rows.items():
            d[k] = {"lethal_lost": sum(r["lethal_lost"] for r in rr), "lethal_extra": sum(r["lethal_extra"] for r in rr),
                    "lethal_ref": sum(r["lethal_ref"] for r in rr),
                    "frames_lethal_lost": sum(1 for r in rr if r["lethal_lost"]),
                    "occ3_lost": sum(r["occ3_lost"] for r in rr), "occ3_ref": sum(r["occ3_ref"] for r in rr),
                    "fused3": judge(rr, "mis3", "known3"), "costmap": judge(rr, "cost_diff", "info")}
            d[k]["G7_pass"] = d[k]["lethal_lost"] == 0 and d[k]["fused3"]["pass"] and d[k]["costmap"]["pass"]
            d[k]["worst_frames_mis3"] = [(r["frame"], r["mis3"], r["known3"]) for r in sorted(rr, key=lambda r: -r["mis3"])[:5]]
        d["frames"] = n
        d["seconds"] = round(time.time() - t0, 1)
        res["datasets"][name] = d
        print(name, json.dumps({k: (v["G7_pass"], v["lethal_lost"], v["fused3"]["total_num"], v["costmap"]["total_num"])
                                for k, v in d.items() if isinstance(v, dict)}), flush=True)
        with open(out, "w") as f:
            json.dump(res, f, indent=1)


if __name__ == "__main__":
    main()
