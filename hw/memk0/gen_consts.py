#!/usr/bin/env python3
"""MEMK0: compile-time constants of the memory core from perception.fpga_replay.core.Config() (product defaults).

Writes memk0_consts.h. Every constant the PL uses is derived here from the same Config the Python memory uses;
PLMemory checks config_digest() at start and runs the plain Python memory when it differs.
  python3 hw/memk0/gen_consts.py            (from the repo root)
"""
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from perception.fpga_replay.core import Config  # noqa: E402

# Config fields baked into the core (changing any of them needs a new bitstream)
PL_FIELDS = ("association_m", "association_max_m", "drift_per_m", "drift_per_rad", "below_ground_tol_m",
             "size_margin", "dedupe_m", "dedupe_iou", "obs_sigma0_m", "obs_sigma_k", "gate_chi", "fuse_keep",
             "fuse_inlier_m", "obs_max_range_m", "create_max_range_m", "exclusion_m", "merge_max_m",
             "existence_min", "existence_max", "existence_init", "existence_window_s", "existence_hit",
             "existence_free", "invalid_below", "miss_weight", "miss_max_range_m", "miss_pd_cap",
             "miss_range_curve", "miss_decisive_pd", "miss_floor", "miss_reliable", "miss_reliable_window_s",
             "miss_reliable_min_windows", "miss_reliable_rate", "miss_reliable_recent_s", "miss_reliable_vote",
             "move_far_m", "memory_quality", "identity_v2", "existence_v3", "move_v2")
# switches the PS control (PLMemory) implements only in their product setting
PS_REQUIRED = dict(memory_quality=True, identity_v2=True, existence_v3=True, move_v2=True, miss_reliable=True,
                   miss_turn_gate=True, pose_trust_gate=True, one_per_class=True)


def config_digest(cfg):
    row = {k: getattr(cfg, k) for k in PL_FIELDS}
    return hashlib.sha256(json.dumps(row, sort_keys=True, default=str).encode()).hexdigest()[:16]


def q(x, frac):
    return int(round(float(x) * (1 << frac)))


def main():
    c = Config()
    for k, v in PS_REQUIRED.items():
        assert getattr(c, k) == v, k
    span = int(round(c.miss_reliable_window_s / c.existence_window_s))
    assert span <= 16, "hit ring holds 16 windows"
    kmax = max(k for k in range(1000) if k * c.existence_window_s <= c.miss_reliable_recent_s)
    hmin = []
    for n in range(0, 17):          # smallest h with float(h / n) >= rate (Python's own comparison)
        ok = [h for h in range(n + 1) if n and h / n >= c.miss_reliable_rate]
        hmin.append(ok[0] if ok else 99)
    xs, ys = zip(*c.miss_range_curve)
    assert len(xs) == 3
    slopes = [(ys[i + 1] - ys[i]) / (xs[i + 1] - xs[i]) for i in range(2)]
    # 0.5 * ln(1 - x), x = i / 1024, in 2^-24; P_D <= miss_pd_cap < 1 so i + 1 <= 922 (one BRAM36: 1024 x 26 bit)
    assert c.miss_pd_cap * 1024 + 1 < 1023
    ln_tab = [q(c.miss_weight * math.log(1.0 - i / 1024.0), 24) for i in range(1024)]
    L = []
    def d(name, val, comment=""):
        L.append("static const long long %s = %dLL;%s" % (name, val, ("  // " + comment) if comment else ""))
    d("K_ASSOC_M20", q(c.association_m, 20), "association_m, 2^-20 m")
    d("K_ASSOC_MAX_M20", q(c.association_max_m, 20))
    d("K_DRIFT_M24", q(c.drift_per_m, 24), "per metre path (path in 2^-22)")
    d("K_DRIFT_RAD24", q(c.drift_per_rad, 24), "per radian turn (turn in 2^-21)")
    d("K_BELOW_TOL18", q(c.below_ground_tol_m, 18))
    d("K_DEDUPE2_36", q(c.dedupe_m ** 2, 36), "dedupe_m^2, 2^-36 m^2")
    d("K_IOU_NUM", 3); d("K_IOU_DEN", 10)
    assert abs(c.dedupe_iou - 0.3) < 1e-12
    d("K_SIG0_24", q(c.obs_sigma0_m, 24))
    d("K_SIGK_32", q(c.obs_sigma_k, 32), "per m^2, 2^-32")
    d("K_R_DEFAULT20", q(1.5, 20), "_obs_sd: range unknown -> 1.5 m")
    d("K_CHI2_24", q(c.gate_chi ** 2, 24))
    d("K_FUSE_KEEP", c.fuse_keep)
    d("K_INLIER20", q(c.fuse_inlier_m, 20))
    d("K_MAD3_16", q(3 * 1.4826, 16))
    d("K_OBS_MAX_R20", q(c.obs_max_range_m, 20))
    d("K_CREATE_MAX_R20", q(c.create_max_range_m, 20))
    d("K_EXCL20", q(c.exclusion_m, 20))
    d("K_MERGE20", q(c.merge_max_m, 20))
    d("K_MOVE_FAR20", q(c.move_far_m, 20))
    d("K_EX_MIN24", q(c.existence_min, 24)); d("K_EX_MAX24", q(c.existence_max, 24))
    d("K_EX_INIT24", q(c.existence_init, 24)); d("K_EX_HIT24", q(c.existence_hit, 24))
    d("K_EX_FREE24", q(c.existence_free, 24)); d("K_INVALID24", q(c.invalid_below, 24))
    d("K_MISS_FLOOR24", q(c.miss_floor, 24)); d("K_REL_VOTE24", q(c.miss_reliable_vote, 24))
    d("K_MISS_MAXR20", q(c.miss_max_range_m, 20)); d("K_PD_CAP24", q(c.miss_pd_cap, 24))
    d("K_DECISIVE24", q(c.miss_decisive_pd, 24))
    d("K_CX0_20", q(xs[0], 20)); d("K_CX1_20", q(xs[1], 20)); d("K_CX2_20", q(xs[2], 20))
    d("K_CY0_24", q(ys[0], 24)); d("K_CY1_24", q(ys[1], 24)); d("K_CY2_24", q(ys[2], 24))
    d("K_CS0_24", q(slopes[0], 24), "slope per m"); d("K_CS1_24", q(slopes[1], 24))
    d("K_SPAN", span); d("K_MIN_WIN", c.miss_reliable_min_windows); d("K_RECENT_WIN", kmax)
    out = ["// generated by gen_consts.py from Config() -- do not edit", "#pragma once",
           "// config digest " + config_digest(c),
           "static const unsigned int K_DIGEST_LO = 0x%sU;  // result word 3" % config_digest(c)[:8]] + L
    out.append("static const unsigned char K_HMIN[17] = {%s};" % ",".join(map(str, hmin)))
    out.append("static const int LN_TAB[1024] = {%s};" % ",".join(map(str, ln_tab)))
    (Path(__file__).parent / "memk0_consts.h").write_text("\n".join(out) + "\n")
    # 18-bit seeds (one BRAM18): 1/sqrt(m) in Q0.18, m in [1,4) in 768 steps; two Newton steps reach < 2^-36
    rom = [int(round((1 << 18) / math.sqrt((1024 + 4 * k + 2) / 1024.0))) for k in range(768)]
    assert max(rom) < (1 << 18)
    (Path(__file__).parent / "memk0_rsqrt_rom.h").write_text(
        "// generated by gen_consts.py: 1/sqrt(m) Q0.18 seeds, m in [1,4) in 768 steps\n#pragma once\n"
        "static const unsigned int RSQRT_ROM[768] = {%s};\n" % ",".join(map(str, rom)))
    print("digest", config_digest(c), "span", span, "kmax", kmax, "hmin", hmin)


if __name__ == "__main__":
    main()
