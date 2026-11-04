#!/usr/bin/env python3
"""Pre-run prediction D_geom (docs/design/NAV_COLDSTART_CRAWL_20261017.md section 3).

  dgeom.py OUT.json
The cold-start frame (synthetic camera, validated in ka_host.py K2) is fused at (0, 0, 0) and then FROZEN: no new
frame, only the bridge's body seed (radius 0.30 + 0.05) follows the body to (d, 0). For each d (step 0.0125 m, so
sub-cell phases of the seed are covered) the Nav2 replica evaluates DWB's 419 samples from (d, 0, 0) at rest and at
the crawl speed. D_geom = the smallest d from which the 0.15 m/s straight stays valid for every d up to 1.0 m.
Only what the start frame saw plus the seed: new frames during the crawl see further ahead, never the side band
next to the body (it is below the camera's lower field edge), so this is the expected value, not a pessimistic one.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crawl_lib as C  # noqa: E402


def main():
    out = sys.argv[1]
    ch = C.CarChain()
    codes = C.render_codes((0.0, 0.0, 0.0), C.ROOM)
    for _ in range(40):
        ch.frame(codes, (0.0, 0.0, 0.0))
    rows = []
    n = 81
    for k in range(n):
        d = round(k * 0.0125, 4)
        g, go = ch.grid((d, 0.0, 0.0), 0.05)
        origin = C.costmap_origin(d, 0.0)
        m = C.master_from_grid(g, go, origin)
        row = {"d": d}
        for name, v0 in (("rest", 0.0), ("v0_0.0316", 0.0316)):
            _, s = C.evaluate(m, origin, start=(d, 0.0, 0.0), v0=v0)
            row[name] = {k2: s[k2] for k2 in ("valid", "rot_valid", "max_straight_vx", "straight_valid")}
        rows.append(row)
        print(d, row["rest"], row["v0_0.0316"]["max_straight_vx"], flush=True)
    fast = [abs(r["rest"]["max_straight_vx"] - 0.15) < 1e-9 for r in rows]
    first = next((r["d"] for r, f in zip(rows, fast) if f), None)
    stays = None
    for i in range(len(rows)):
        if all(fast[i:]):
            stays = rows[i]["d"]
            break
    res = {"rows": rows, "first_d_fast_valid": first, "D_geom": stays}
    json.dump(res, open(out, "w"), indent=1)
    print("first", first, "D_geom", stays)


if __name__ == "__main__":
    main()
