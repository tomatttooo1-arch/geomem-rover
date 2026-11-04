#!/usr/bin/env python3
"""A blank (all FREE) map_server map: OUT_DIR/blank.pgm + OUT_DIR/blank.yaml (2026-10-18).

Nav2's start map of the in-stack mapping run (car_up CAR_MAPPING=1, docs/design/TELEOP_NAV_ARB_20261018.md section 2):
map_server needs a map before Nav2 activates, and the mapping export only exists after "mapping-stop". FREE, not
unknown: the runner (host/ros_runtime.py on_map / FreeMap) treats unknown cells as occupied, so an unknown start map
would refuse every goto (goal_not_free). Square, centred on the map origin (the car's start, map -> odom identity),
the same cells as tools/car_run/bench_pose.py free_grid (its stand-in /map). Trinary yaml; free cells are 254
(p = 1/255 < free_thresh 0.196). Standard library only; Python 3.8 compatible.

    python3 tools/car_run/blank_map.py --out-dir DIR [--size-m 20] [--res-m 0.05]
"""
import argparse
import math
import os
import sys

FREE = 254


def write_blank(out_dir, size_m=20.0, res_m=0.05):
    """-> path of the yaml. ValueError for a size/resolution that is not a positive finite grid."""
    if not (math.isfinite(size_m) and math.isfinite(res_m) and size_m > 0 and res_m > 0):
        raise ValueError("size_m and res_m must be positive (given %r, %r)" % (size_m, res_m))
    n = int(round(size_m / res_m))
    if not 1 <= n <= 4000:
        raise ValueError("blank map of %d x %d cells refused (1..4000)" % (n, n))
    half = n * res_m / 2.0
    os.makedirs(out_dir, exist_ok=True)
    pgm, yml = os.path.join(out_dir, "blank.pgm"), os.path.join(out_dir, "blank.yaml")
    with open(pgm, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (n, n))
        f.write(bytes([FREE]) * (n * n))
    with open(yml, "w") as f:
        f.write("image: blank.pgm\nmode: trinary\nresolution: %r\norigin: [%r, %r, 0.0]\nnegate: 0\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.196\n" % (res_m, -half, -half))
    return yml


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--size-m", type=float, default=20.0)
    ap.add_argument("--res-m", type=float, default=0.05)
    a = ap.parse_args(argv)
    try:
        print(write_blank(a.out_dir, a.size_m, a.res_m))
    except ValueError as exc:
        sys.stderr.write("blank_map: %s\n" % exc)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
