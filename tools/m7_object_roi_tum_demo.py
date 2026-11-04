"""Descriptive demo of the mask-free region check on TUM pioneer_360 (E1).

Picks anchor points on visible surfaces in one frame, places a synthetic
(shrunk) class box on each, projects it into every frame with the reference
trajectory and tallies the region verdicts. Description only: there are no
labelled objects, so nothing here is an accuracy figure.

Usage: PYTHONPATH=. python tools/m7_object_roi_tum_demo.py <prepared_dir> <out.json>
"""
import json
from pathlib import Path
import sys
import tempfile

import numpy as np

from host.m7_object_native import NativeObjectEvidence, compile_object_native
from host.m7_object_sizes import synthetic_bounds
from perception.fpga_replay.io import interpolate_pose


def main(prepared, output):
    prepared = Path(prepared)
    meta = json.loads((prepared / "meta.json").read_text())
    w, h, n = meta["width"], meta["height"], meta["frames"]
    unit = float(meta["depth_unit_m"])
    raw = np.memmap(prepared / meta["depth_file"], dtype=np.uint16, mode="r", shape=(n, h, w))
    k = np.asarray(meta["K"], dtype=np.float64)
    # pioneer_360 groundtruth.txt has 3 repeated timestamps; read_table
    # rightly refuses it. Keep the first row of each repeated stamp.
    truth = Path(meta["source_rec"]) / "groundtruth.txt"
    rows, last, dropped = [], None, 0
    for line in truth.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        row = line.split()
        if last is not None and float(row[0]) <= last:
            dropped += 1; continue
        rows.append(row); last = float(row[0])
    times = np.array([float(r[0]) for r in rows])
    poses = [interpolate_pose(ns / 1e9, rows, times, max_gap=.05) for ns in meta["acquired_ns"]]

    def depth_m(i):
        d = raw[i].astype(np.float32) * np.float32(unit)
        d[raw[i] == 0] = np.nan
        return d

    # Anchors: frame 0 pixels on a coarse grid with valid depth 1..4 m.
    ref = next(i for i, (_, ok) in enumerate(poses) if ok)
    d0, t0 = depth_m(ref), poses[ref][0]
    anchors = []
    for v, u in ((240, 320), (180, 160), (300, 480), (360, 320)):
        z = float(d0[v, u])
        if np.isfinite(z) and 1. <= z <= 4.:
            cam = np.linalg.inv(k) @ np.array([u, v, 1.]) * z
            # "surface": box centred on the seen surface (something is there).
            # "air": same ray 0.6 m nearer, empty space in the reference frame
            # (stands in for an object that has since been removed).
            for kind, scale in (("surface", 1.), ("air", (z - .6) / z)):
                anchors.append({"kind": kind, "pixel": [u, v], "z_m": z * scale,
                                "world": (t0[:3, :3] @ (cam * scale) + t0[:3, 3]).tolist()})
    # The camera's up direction in the reference frame picks the vertical axis.
    up_axis = int(np.argmax(np.abs(t0[:3, :3] @ np.array([0., -1., 0.]))))
    entries = [("anchor-%d-%s" % (i // 2, a["kind"]), synthetic_bounds("bottle", a["world"], up_axis))
               for i, a in enumerate(anchors)]
    with tempfile.TemporaryDirectory() as tmp:
        native = NativeObjectEvidence(compile_object_native(Path(tmp) / "object.so"))
        tally = {name: {} for name, _ in entries}
        tally["_invalid_pose_frames"] = 0
        for i in range(n):
            pose, ok = poses[i]
            if not ok:
                tally["_invalid_pose_frames"] += 1
                continue
            for (name, _), (_, evidence, _) in zip(entries, native.check_boxes(depth_m(i), k, pose, entries)):
                tally[name][evidence] = tally[name].get(evidence, 0) + 1
    result = {"dataset": meta["source_dataset"], "frames": n, "reference_frame": ref,
              "up_axis": up_axis, "groundtruth_rows_dropped_repeated_stamp": dropped, "class": "bottle", "shrink": .8,
              "anchors": anchors, "tally": tally,
              "note": "descriptive only; anchors are arbitrary surfaces, not objects"}
    Path(output).write_text(json.dumps(result, indent=1))
    print(json.dumps(tally, indent=1))


if __name__ == "__main__":
    main(*sys.argv[1:3])
