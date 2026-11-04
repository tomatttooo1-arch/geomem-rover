#!/usr/bin/env python3
"""crawl_lib.render_codes (the closed loop's fast renderer) against render_codes_ref, code for code, at the mounts and
poses the 10-17 tilt runs use (room + the o5 / o10 boxes + a tall box), and the time per call. docs section 9.2 V4.

  render_check.py
Python 3.8 compatible."""
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools", "nav_coldstart_crawl"))
import crawl_lib as C  # noqa: E402
sys.path.insert(0, ROOT)
from tools.car_geo_producer.common import MOUNT_XYZ, base_camera  # noqa: E402


def main():
    bad = 0
    for h, p, r in ((0.39, 7.0, 0.0), (0.375, 10.0, 0.0), (0.3799, 27.42, 0.07), (0.3915, 7.05, 0.59)):
        T = base_camera((MOUNT_XYZ[0], MOUNT_XYZ[1], h), p, r)
        for boxes in ([], [(0.50, 0.60, -0.10, 0.10, 0.10)], [(0.50, 0.60, -0.10, 0.10, 0.05)],
                      [(0.70, 1.00, -0.50, 0.50, 0.60)]):
            world = C.ROOM + boxes
            for pose in ((0.0, 0.0, 0.0), (0.14, 0.002, -0.03), (0.30, -0.05, 0.2), (-0.2, 0.1, 1.2)):
                a = C.render_codes(pose, world, T_bc=T)
                b = C.render_codes_ref(pose, world, T_bc=T)
                n = int((a != b).sum())
                bad += n
                if n:
                    print("DIFF", h, p, r, boxes, pose, n)
        t0 = time.perf_counter()
        for _ in range(10):
            C.render_codes((0.0, 0.0, 0.0), C.ROOM + [(0.50, 0.60, -0.10, 0.10, 0.10)], T_bc=T)
        print("mount %.4f/%.2f/%.2f: box o10 render %.1f ms/call" % (h, p, r, 100 * (time.perf_counter() - t0)))
    print("total differing codes", bad)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
