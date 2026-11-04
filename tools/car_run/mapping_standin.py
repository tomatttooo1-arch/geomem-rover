#!/usr/bin/env python3
"""HOST stand-in of `rtabmap_car.sh map` for the in-stack mapping run (2026-10-18). Tests control flow ONLY.

The host simulation image has no rtabmap (rtabmap-export not found), so tools/car_run/mapping_ctl.py runs this instead
on the host (car_inner.sh PLATFORM=host, marks "STAND-IN"). It mimics the contract mapping_ctl relies on, nothing else:
  stderr "running --" once started; on SIGINT/SIGTERM: writes OUT/export/<RUN_TIME>.yaml + .pgm (a FREE square with
  a 0.2 m thick occupied wall across x = WALL_X_M, so a map swap is visible on /map and in the runner's free check),
  an empty DB_DIR/<RUN_TIME>.db, OUT/sha256.txt (sha256sum format: the database + every export file, as rtabmap_car.sh
  writes it -- host/memory_store.export_source_db reads it), logs "export done" and exits 0.
  STANDIN_FAIL=1: exits 5 without an export ("db NOT saved"), the failure path (negative control).
No camera, no SLAM, no database content: a stand-in run proves nothing about rtabmap (docs/design/TELEOP_NAV_ARB_20261018.md
section 5). Env: OUT DB_DIR RUN_TIME (as rtabmap_car.sh), WALL_X_M (1.0), STANDIN_FAIL (0).
"""
import hashlib
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from blank_map import write_blank  # noqa: E402


def write_export(out, name, wall_x_m=1.0, size_m=20.0, res_m=0.05):
    yml = write_blank(os.path.join(out, "export"), size_m, res_m)
    n = int(round(size_m / res_m))
    half = n * res_m / 2.0
    col0 = int((wall_x_m + half) / res_m)
    pgm = os.path.join(out, "export", "blank.pgm")
    with open(pgm, "rb") as f:
        raw = bytearray(f.read())
    head = len(b"P5\n%d %d\n255\n" % (n, n))
    for row in range(n):
        for col in range(max(0, col0), min(n, col0 + 4)):
            raw[head + row * n + col] = 0            # occupied
    dst_pgm, dst_yaml = os.path.join(out, "export", name + ".pgm"), os.path.join(out, "export", name + ".yaml")
    with open(dst_pgm, "wb") as f:
        f.write(bytes(raw))
    with open(yml) as f:
        text = f.read().replace("image: blank.pgm", "image: %s.pgm" % name)
    with open(dst_yaml, "w") as f:
        f.write(text)
    os.unlink(pgm)
    os.unlink(yml)
    return dst_yaml


def write_sha_table(out, db):
    """OUT/sha256.txt as `sha256sum "$DB" "$OUT/export/"*` (rtabmap_car.sh) writes it."""
    rows = []
    exp = os.path.join(out, "export")
    for path in [db] + [os.path.join(exp, n) for n in sorted(os.listdir(exp))]:
        with open(path, "rb") as f:
            rows.append("%s  %s\n" % (hashlib.sha256(f.read()).hexdigest(), path))
    with open(os.path.join(out, "sha256.txt"), "w") as f:
        f.write("".join(rows))


def main():
    out, db_dir = os.environ["OUT"], os.environ["DB_DIR"]
    name = os.environ.get("RUN_TIME") or time.strftime("%Y%m%d_%H%M%S")
    os.makedirs(out, exist_ok=True)
    got = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda s, f: got.append(s))
    sys.stderr.write("[mapping_standin] STAND-IN for rtabmap_car.sh map (no rtabmap on this host)\n")
    sys.stderr.write("[mapping_standin] running -- (stand-in; SIGINT ends it)\n")
    sys.stderr.flush()
    while not got:
        time.sleep(0.1)
    if os.environ.get("STANDIN_FAIL") == "1":
        sys.stderr.write("[mapping_standin] db NOT saved (STANDIN_FAIL=1)\n")
        return 5
    os.makedirs(db_dir, exist_ok=True)
    db = os.path.join(db_dir, name + ".db")
    open(db, "wb").close()
    yml = write_export(out, name, float(os.environ.get("WALL_X_M", "1.0")))
    write_sha_table(out, db)
    sys.stderr.write("[mapping_standin] export done: %s\n" % yml)
    return 0


if __name__ == "__main__":
    sys.exit(main())
