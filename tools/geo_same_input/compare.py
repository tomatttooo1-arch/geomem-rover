#!/usr/bin/env python3
"""Voxel-by-voxel comparison of arm dumps against the reference arm (A).

Every dump holds, per frame in input order, 3 x uint32 counts (accepted,
invalid, outside) + nx*ny*nz uint8 tri-state flags (0 unknown, 1 free, 2 hit).
"outside" counts clipped rays since M7_CLIP_SPEC_20261011 (walked, no HIT).
Dumps made before 2026-10-11 used the drop rule: do not compare them with
clipped-semantics dumps on frames whose outside count is non-zero.
Judgement: a candidate equals the reference only if every flag of every frame
and every count is identical. Otherwise it reports the number of differing
voxels/frames, the value transitions (ref -> candidate) and where they are
(z layer = distance band from the camera, x/y bounding box).

Before judging, two positive controls must hold (else exit 3): a copy of the
reference with one voxel flipped must show exactly 1 difference, and the
reference compared with itself shifted by one frame must show differences
(frames really differ, so identical dumps are not vacuous).
Usage: compare.py --ref A.dump --cand NAME=path [--cand ...] --out report.json
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.geo_same_input.common import GRID_ORIGIN_M, GRID_SHAPE, GRID_VOXEL_M, read_dump  # noqa: E402


def diff_report(ref_counts, ref_flags, counts, flags, shape=GRID_SHAPE):
    nx, ny, nz = shape
    if flags.shape != ref_flags.shape:
        return {"equal": False, "error": "shape %s vs reference %s" % (flags.shape, ref_flags.shape)}
    ne = flags != ref_flags
    count_diff = np.any(counts != ref_counts, axis=1)
    frames_voxel = np.flatnonzero(ne.any(axis=1))
    rep = {"equal": bool(not ne.any() and not count_diff.any()),
           "frames": int(ref_flags.shape[0]),
           "frames_with_voxel_diff": int(frames_voxel.size),
           "frames_with_count_diff": int(count_diff.sum()),
           "voxels_diff_total": int(ne.sum()),
           "voxels_compared": int(ne.size)}
    if ne.any():
        f_idx, v_idx = np.nonzero(ne)
        z, rem = np.divmod(v_idx, nx * ny)
        y, x = np.divmod(rem, nx)
        trans = {}
        for r, c in zip(ref_flags[f_idx, v_idx], flags[f_idx, v_idx]):
            k = "%d->%d" % (r, c)
            trans[k] = trans.get(k, 0) + 1
        per_frame = ne.sum(axis=1)[frames_voxel]
        rep.update({
            "transitions_ref_to_cand": trans,
            "per_frame_diff": {"min": int(per_frame.min()), "median": float(np.median(per_frame)),
                               "max": int(per_frame.max())},
            "z_layer_histogram": {int(k): int(v) for k, v in zip(*np.unique(z, return_counts=True))},
            "z_layer_note": "z layer k spans camera-Z %.2f..%.2f m + k*%.2f m" % (
                GRID_ORIGIN_M[2], GRID_ORIGIN_M[2] + GRID_VOXEL_M, GRID_VOXEL_M),
            "bbox_xyz": [[int(x.min()), int(x.max())], [int(y.min()), int(y.max())], [int(z.min()), int(z.max())]],
            "first_frames": [int(v) for v in frames_voxel[:10]]})
    if count_diff.any():
        i = np.flatnonzero(count_diff)[:5]
        rep["count_examples"] = [{"frame": int(k), "ref": ref_counts[k].tolist(), "cand": counts[k].tolist()}
                                 for k in i]
    return rep


def controls(ref_counts, ref_flags):
    flipped = ref_flags.copy()
    mid = flipped.shape[0] // 2
    flipped[mid, flipped.shape[1] // 2] = (flipped[mid, flipped.shape[1] // 2] + 1) % 3
    one = diff_report(ref_counts, ref_flags, ref_counts, flipped)
    shifted = diff_report(ref_counts[1:], ref_flags[1:], ref_counts[:-1], ref_flags[:-1])
    distinct = len({bytes(r) for r in ref_flags})
    ok = one["voxels_diff_total"] == 1 and not one["equal"] and shifted["frames_with_voxel_diff"] > 0
    return ok, {"one_voxel_flip_detected": one["voxels_diff_total"] == 1,
                "shift_by_one_frame_frames_differing": shifted["frames_with_voxel_diff"],
                "distinct_reference_frames": distinct}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ref", required=True)
    ap.add_argument("--cand", action="append", default=[])
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    nvox = int(np.prod(GRID_SHAPE))
    rc, rf = read_dump(a.ref, nvox)
    ok, ctl = controls(rc, rf)
    report = {"reference": a.ref, "controls": ctl, "controls_pass": ok,
              "reference_stats": {"frames": int(rf.shape[0]),
                                  "free_voxels_median": float(np.median((rf == 1).sum(axis=1))),
                                  "hit_voxels_median": float(np.median((rf == 2).sum(axis=1))),
                                  "accepted_median": float(np.median(rc[:, 0]))},
              "candidates": {}}
    for spec in a.cand:
        name, path = spec.split("=", 1)
        try:
            cc, cf = read_dump(path, nvox)
            report["candidates"][name] = diff_report(rc, rf, cc, cf)
        except (OSError, ValueError) as exc:
            report["candidates"][name] = {"equal": False, "error": str(exc)}
    json.dump(report, open(a.out, "w"), indent=1)
    print(json.dumps({"controls_pass": ok, **{k: v.get("equal") for k, v in report["candidates"].items()}}))
    if not ok:
        sys.exit(3)
    sys.exit(0 if all(v.get("equal") for v in report["candidates"].values()) else 1)


if __name__ == "__main__":
    main()
