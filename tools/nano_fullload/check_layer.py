#!/usr/bin/env python3
"""Live stop check between N4 layers (design doc §4 + §6.1). usage: check_layer.py <run_dir> <layer>
Prints PASS or FAIL: <reasons>; exit 0 = pass, 1 = fail. Only criteria whose inputs exist in that layer are applied."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from summarize import summarize  # noqa: E402

run, L = Path(sys.argv[1]), sys.argv[2]
res = summarize(run, 200.0)["layers"]
d, bad = res.get(L, {}), []
l0 = res.get("L0", {})
if "vo" in d:
    v = d["vo"]
    if v["lost"] > 0:
        bad.append("vo lost %d" % v["lost"])
    if v["processed_ratio"] is not None and v["processed_ratio"] < 0.9:
        bad.append("vo processed %.3f < 0.9" % v["processed_ratio"])
if "det" in d:
    x = d["det"]
    if x["late_tap_ratio"] > 0.01:
        bad.append("det late %.4f" % x["late_tap_ratio"])
    if x["skipped_ratio"] > 0.01:
        bad.append("det skipped %.4f" % x["skipped_ratio"])
    if x["fps"] < 14.5:
        bad.append("det fps %.2f" % x["fps"])
if "ram_used_mb_max" in d and d["ram_used_mb_max"] > d["ram_total_mb"] - 1024:
    bad.append("ram %d MB" % d["ram_used_mb_max"])
if "swap_mb_max" in d and l0.get("swap_mb_max") is not None and d["swap_mb_max"] - l0["swap_mb_max"] > 200:
    bad.append("swap +%d MB" % (d["swap_mb_max"] - l0["swap_mb_max"]))
if "temp_c_max" in d and max(d["temp_c_max"].values()) >= 85:
    bad.append("temp %.1f C" % max(d["temp_c_max"].values()))
print("PASS" if not bad else "FAIL: " + "; ".join(bad), {k: d.get(k) for k in ("board_cores", "cluster0_cores", "vdd_in_mw_mean")})
sys.exit(1 if bad else 0)
