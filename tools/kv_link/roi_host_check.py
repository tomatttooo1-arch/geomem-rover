#!/usr/bin/env python3
"""Server-side (x86) recheck of board ROI answers (PC_DATAPLANE E.1, package 1).

Takes the prepared recording (same depth_sha256 as on the Nano), the memory snapshot the
Nano planned from and one or more roi.jsonl files written by nano_replay --roi-memory, and
reports, per file, every answered frame recomputed with the E2 C reference compiled on this
machine (evaluate_boxes on the same float32 depth) -- known answer: all rows equal -- plus,
for every pair of files, the rows of the frames (by source index) answered in both
(e.g. --roi-core cpu on 0x0117bc0a vs --roi-core pl on 0x07784f75: must be equal).
Positive control: one changed count in the first compared frame must be flagged.

Usage: roi_host_check.py --prep DIR --snap snap.json --jsonl NAME=roi.jsonl [...] [--out r.json]
"""
import argparse
import itertools
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from host.m7_object_native import NativeObjectEvidence, compile_object_native  # noqa: E402
from perception.fpga_replay.core import Config  # noqa: E402
from tools.geo_same_input.common import depth_u16_to_m  # noqa: E402
from tools.kv_link.nano_replay import roi_plan  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prep", required=True)
    ap.add_argument("--snap", required=True)
    ap.add_argument("--jsonl", action="append", required=True, help="NAME=path")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    meta = json.load(open(os.path.join(a.prep, "meta.json")))
    h, w = meta["height"], meta["width"]
    depth = np.memmap(os.path.join(a.prep, meta["depth_file"]), dtype="<u2", mode="r",
                      shape=(meta["frames"], h, w))
    K = np.array(meta["K"], dtype=np.float64)
    _, batch = roi_plan(json.load(open(a.snap)), (h, w), K, Config())
    boxes = [(r.row_id, r.xyxy, r.near_z, r.far_z) for r in batch.rows]
    tmp = tempfile.mkdtemp(prefix="roi_host_check_")
    native = NativeObjectEvidence(compile_object_native(os.path.join(tmp, "obj.so")))
    unit = float(meta["depth_unit_m"])
    files, report = {}, {"prep_depth_sha256": meta.get("depth_sha256"), "rows_per_frame": len(boxes),
                         "files": {}, "pairs": {}}
    control = None
    for spec in a.jsonl:
        name, path = spec.split("=", 1)
        recs = [json.loads(line) for line in open(path)]
        ok = [r for r in recs if r["ok"]]
        diffs, rows = [], 0
        by_src, cores = {}, {}
        for r in ok:
            ref = native.evaluate_boxes(depth_u16_to_m(np.asarray(depth[r["src"]]), unit), boxes, batch.cfg)
            rows += len(ref)
            if ref != r["rows"]:
                diffs.append(r["frame_id"])
            if control is None and ref:
                bad = [dict(x) for x in r["rows"]]
                bad[0]["roi_front_pixels"] = bad[0].get("roi_front_pixels", 0) + 1
                control = bad != ref
            by_src.setdefault(r["src"], r["rows"])
            cores[r.get("roi_core")] = cores.get(r.get("roi_core"), 0) + 1
        files[name] = by_src
        report["files"][name] = {"path": path, "records": len(recs), "ok": len(ok),
                                 "rows_compared": rows, "frames_different": diffs[:20],
                                 "n_frames_different": len(diffs), "roi_core": cores,
                                 "equal": bool(ok) and not diffs}
    for x, y in itertools.combinations(sorted(files), 2):
        common = sorted(set(files[x]) & set(files[y]))
        diff = [s for s in common if files[x][s] != files[y][s]]
        report["pairs"]["%s~%s" % (x, y)] = {"common_frames": len(common),
                                             "rows_compared": len(common) * len(boxes),
                                             "frames_different": diff[:20], "equal": bool(common) and not diff}
    report["control_caught"] = control
    print(json.dumps(report, indent=1))
    if a.out:
        with open(a.out, "w") as fp:
            json.dump(report, fp, indent=1)


if __name__ == "__main__":
    main()
