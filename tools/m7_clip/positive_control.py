#!/usr/bin/env python3
"""Positive controls for the clipped M7 frame (docs/design/M7_CLIP_SPEC_20261011.md §6).

rec30 has zero out-of-grid pixels, so it cannot see clipping at all. This
script builds synthetic frames that do, and checks, frame by frame:
  (a) old reference != new reference on every frame whose clipped rays mark
      at least one in-grid voxel by the golden (a clipped ray that leaves the
      camera voxel straight out of the grid marks nothing, so old == new is
      then the correct answer and is reported separately); old == new on every
      frame without clipped rays; and old == golden under the OLD (drop) rule;
  (b) new reference == independent golden (every voxel, every count), the
      golden's float64 and exact-rational slab evaluations agree, and
      accepted + invalid + clipped == pixels;
  (c) the two injected faults (mut_drop, mut_trunc) differ from the golden on
      at least one frame each; the frames that catch them are listed.
Usage: positive_control.py --build <fresh dir> --out report.json [--quick]
Exit 0 only if every check holds.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.m7_clip.golden import golden_frame, golden_rays, project_frame  # noqa: E402
from tools.m7_clip.kernels import Kernel, build_all  # noqa: E402

PRODUCT = dict(grid_origin=(-0.8, -0.8, -0.05), voxel=0.05, dims=(32, 32, 32))
UNIT = dict(grid_origin=(0., 0., 0.), voxel=1.0, dims=(32, 32, 32))


def rot(axis, angle):
    c, s = np.cos(angle), np.sin(angle)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def pose(R, t):
    wc = np.zeros((3, 4), dtype=np.float32)
    wc[:, :3] = R
    wc[:, 3] = t
    return wc


def inv_k(fx, fy, cx, cy):
    return np.ascontiguousarray(np.linalg.inv(np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]])), dtype=np.float32)


def realistic_frames(rng):
    """Multi-pixel frames on the product grid (32^3 x 5 cm, origin (-0.8,-0.8,-0.05))."""
    W, H = 80, 60
    ik = inv_k(50., 50., 39.5, 29.5)          # ~77 deg horizontal FOV
    centre = (0., 0., 0.75)                   # world point at the grid centre
    frames = []

    def noisy(depth):
        depth = depth.astype(np.float32)
        flat = depth.ravel()
        idx = rng.choice(flat.size, size=flat.size // 20, replace=False)
        flat[idx[: len(idx) // 3]] = np.nan
        flat[idx[len(idx) // 3: 2 * len(idx) // 3]] = np.float32(10.0)    # invalid: not < 10
        flat[idx[2 * len(idx) // 3:]] = np.float32(0.1)                    # invalid: not > 0.1
        return flat.reshape(depth.shape)

    # Optical axis (camera +z) pointed at each of the six faces from the centre.
    aims = {"+x": rot("y", np.pi / 2), "-x": rot("y", -np.pi / 2), "+y": rot("x", -np.pi / 2),
            "-y": rot("x", np.pi / 2), "+z": np.eye(3), "-z": rot("y", np.pi)}
    for name, R in aims.items():
        d = rng.uniform(0.11, 9.9, size=(H, W))
        frames.append(dict(name="face_" + name, depth=noisy(d), ik=ik, wc=pose(R, centre), **PRODUCT))
    # Product camera position (cell (16,16,1)), pitched down 27 deg, floor 1.0 m
    # below the camera: the floor lies under the grid's +y face (y points down),
    # so floor returns exit through +y ("below the ground"); far wall at 6 m.
    R = rot("x", -np.deg2rad(27.))
    u, v = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    rays = np.stack([(u - 39.5) / 50., (v - 29.5) / 50., np.ones_like(u)], axis=-1)
    wdir = rays @ R.T
    with np.errstate(divide="ignore", invalid="ignore"):
        t_floor = np.where(wdir[..., 1] > 1e-6, 1.0 / wdir[..., 1], np.inf)
        t_wall = np.where(wdir[..., 2] > 1e-6, 6.0 / wdir[..., 2], np.inf)
    depth = np.minimum(t_floor, t_wall)            # camera-z depth (ray z component is 1)
    depth[rng.random(depth.shape) < 0.1] *= 1.3    # some returns from under the floor plane
    depth = np.where(np.isfinite(depth), depth, np.nan)
    frames.append(dict(name="tilt_down_below_floor", depth=noisy(depth), ik=ik, wc=pose(R, (0., 0., 0.)), **PRODUCT))
    # Far returns 5-10 m from the product camera position.
    frames.append(dict(name="far_5_10m", depth=noisy(rng.uniform(5.0, 9.99, size=(H, W))), ik=ik,
                       wc=pose(np.eye(3), (0., 0., 0.)), **PRODUCT))
    # Mixed near/far: most endpoints inside, 30 % far.
    d = rng.uniform(0.11, 1.5, size=(H, W))
    far = rng.random((H, W)) < 0.3
    d[far] = rng.uniform(1.6, 9.9, size=far.sum())
    frames.append(dict(name="mixed_near_far", depth=noisy(d), ik=ik, wc=pose(np.eye(3), (0., 0., 0.)), **PRODUCT))
    # Rotated generic pose (yaw+roll) near a grid corner.
    R = rot("y", 0.7) @ rot("z", 0.4) @ rot("x", -0.3)
    frames.append(dict(name="corner_generic_pose", depth=noisy(rng.uniform(0.11, 4.0, size=(H, W))), ik=ik,
                       wc=pose(R, (0.6, -0.6, 0.1)), **PRODUCT))
    # Control frame: every endpoint inside the grid (no clipping): old must equal new.
    frames.append(dict(name="all_inside_control", depth=noisy(rng.uniform(0.11, 0.7, size=(H, W))), ik=ik,
                       wc=pose(np.eye(3), centre), **PRODUCT))
    return frames


def exact_ray_frame(name, start, end):
    """One pixel whose Q8 segment is exactly start->end on the unit grid (voxel 1 m)."""
    start = np.asarray(start, dtype=np.int64)
    end = np.asarray(end, dtype=np.int64)
    ik = np.zeros((3, 3), dtype=np.float32)
    ik[:, 2] = (end - start) / 256.
    return dict(name=name, depth=np.ones((1, 1), dtype=np.float32), ik=ik,
                wc=pose(np.eye(3), start / 256.), expect_start=start, expect_end=end, **UNIT)


def exact_frames(rng, n_random):
    c = 256
    mid = 16 * c + 128
    out = []
    # Each face, three exit depths: 1 Q8 unit, exactly one cell, ~200 cells.
    for axis in range(3):
        for sign in (-1, 1):
            for tag, beyond in (("1q8", 1), ("1cell", 256), ("200cells", 200 * 256)):
                e = [mid, mid + 37, mid - 53]
                e[axis] = -beyond if sign < 0 else 32 * c - 1 + beyond
                out.append(exact_ray_frame("face_%s%s_%s" % ("-+"[sign > 0], "xyz"[axis], tag), [mid] * 3, e))
    # Truncation trap: endpoint Q8 in (-256, 0) on one axis, others inside.
    for axis in range(3):
        for x in (-1, -128, -255):
            e = [mid + 300, mid - 200, mid + 500]
            e[axis] = x
            out.append(exact_ray_frame("trunc_trap_%s_%d" % ("xyz"[axis], x), [2 * c + 128] * 3, e))
    # Ties: exact diagonals leaving through grid corners/edges/faces.
    out.append(exact_ray_frame("diag3_out_corner_hi", [mid] * 3, [40 * c + 128] * 3))
    out.append(exact_ray_frame("diag3_out_corner_lo", [mid] * 3, [-8 * c + 128] * 3))
    out.append(exact_ray_frame("diag2_out_edge", [mid, mid, mid], [40 * c + 128, 40 * c + 128, mid]))
    out.append(exact_ray_frame("through_grid_corner_exact", [30 * c + 128, 30 * c + 128, 30 * c + 128],
                               [34 * c + 128, 34 * c + 128, 34 * c + 128]))
    out.append(exact_ray_frame("exit_exactly_on_edge_line", [30 * c, 30 * c, mid], [34 * c, 34 * c, mid]))
    out.append(exact_ray_frame("start_on_plane_exit_lo", [16 * c, 16 * c, 16 * c], [-3 * c, 5 * c, 16 * c]))
    out.append(exact_ray_frame("start_on_plane_exit_hi", [16 * c, 16 * c, 16 * c], [40 * c, 16 * c, 50 * c]))
    out.append(exact_ray_frame("const_axis_on_plane_out", [10 * c + 77, 12 * c, 16 * c], [-5 * c, 12 * c, 40 * c]))
    out.append(exact_ray_frame("end_exactly_at_32", [mid] * 3, [32 * c, mid, mid]))
    out.append(exact_ray_frame("end_exactly_at_minus256", [mid] * 3, [-256, mid, mid]))
    out.append(exact_ray_frame("end_exactly_at_0_inside", [mid] * 3, [0, mid, mid]))
    out.append(exact_ray_frame("mixed_dir_tie_out", [16 * c + 128, 16 * c + 128, 128], [-16 * c + 128, 48 * c + 128, 128]))
    # Random segments: generic, tie-prone (coordinates on a 128 lattice), far.
    for i in range(n_random):
        kind = i % 3
        start = rng.integers(0, 32 * c, size=3)
        if kind == 0:
            end = rng.integers(-60 * c, 92 * c, size=3)
        elif kind == 1:
            start = rng.integers(0, 64, size=3) * 128
            end = rng.integers(-40, 104, size=3) * 128
        else:
            end = start + rng.integers(-220 * c, 220 * c, size=3)
        out.append(exact_ray_frame("rand%d_%d" % (kind, i), start, end))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--old-rev", default="361b145")
    ap.add_argument("--seed", type=int, default=20261011)
    ap.add_argument("--random", type=int, default=3000)
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    t0 = time.time()
    libs = build_all(a.build, a.old_rev)
    k = {n: Kernel(p) for n, p in libs.items()}
    rng = np.random.default_rng(a.seed)
    frames = realistic_frames(rng) + exact_frames(rng, 300 if a.quick else a.random)
    rows = []
    fail = []
    for f in frames:
        args = (f["depth"], f["ik"], f["wc"], f["grid_origin"], f["voxel"], f["dims"])
        res = {n: k[n].run(*args) for n in ("new", "old", "mut_drop", "mut_trunc")}
        gold, gcounts = golden_frame(*args, method="rational")
        gold_d, gcounts_d = golden_frame(*args, method="double")
        o, e, cls = project_frame(*args)
        gold_old = golden_rays(o, e[cls == 0], np.ones(int((cls == 0).sum()), dtype=bool), f["dims"])
        clip_marks = not np.array_equal(gold_old, gold)
        nf, nc = res["new"]
        of, oc = res["old"]
        pixels = f["depth"].size
        clipped = nc[2]
        r = {"name": f["name"], "pixels": pixels, "counts_new": nc, "counts_golden": gcounts,
             "free_new": int((nf == 1).sum()), "hit_new": int((nf == 2).sum()),
             "free_old": int((of == 1).sum()), "hit_old": int((of == 2).sum()),
             "golden_double_eq_rational": bool(np.array_equal(gold, gold_d) and gcounts == gcounts_d),
             "new_eq_golden": bool(np.array_equal(nf, gold) and nc == gcounts),
             "conserved": sum(nc) == pixels,
             "old_eq_new": bool(np.array_equal(of, nf) and oc == nc),
             "old_eq_golden_old_rule": bool(np.array_equal(of, gold_old) and oc == gcounts),
             "clipped_rays_mark_voxels": bool(clip_marks),
             "mut_drop_caught": not (np.array_equal(res["mut_drop"][0], gold) and res["mut_drop"][1] == gcounts),
             "mut_trunc_caught": not (np.array_equal(res["mut_trunc"][0], gold) and res["mut_trunc"][1] == gcounts)}
        if "expect_end" in f:   # the exact-ray construction really delivered the intended Q8 segment
            r["exact_segment_ok"] = bool(np.array_equal(o, f["expect_start"]) and np.array_equal(e[0], f["expect_end"]))
            if not r["exact_segment_ok"]:
                fail.append("%s: exact segment construction failed" % f["name"])
        if not r["golden_double_eq_rational"]:
            fail.append("%s: golden double != rational" % f["name"])
        if not r["new_eq_golden"]:
            fail.append("%s: new != golden" % f["name"])
        if not r["conserved"]:
            fail.append("%s: counts not conserved" % f["name"])
        if not r["old_eq_golden_old_rule"]:
            fail.append("%s: old != golden under the old rule" % f["name"])
        if clip_marks and r["old_eq_new"]:
            fail.append("%s: old == new although clipped rays mark voxels" % f["name"])
        if clipped > 0 and not clip_marks and not r["old_eq_new"]:
            fail.append("%s: old != new although clipped rays mark nothing" % f["name"])
        if clipped == 0 and not r["old_eq_new"]:
            fail.append("%s: old != new without clipped rays" % f["name"])
        rows.append(r)
    drop_frames = [r["name"] for r in rows if r["mut_drop_caught"]]
    trunc_frames = [r["name"] for r in rows if r["mut_trunc_caught"]]
    if not drop_frames:
        fail.append("mut_drop not caught by any frame")
    if not trunc_frames:
        fail.append("mut_trunc not caught by any frame")
    clipped_frames = [r for r in rows if r["counts_new"][2] > 0]
    summary = {
        "frames": len(rows), "realistic_frames": sum(1 for f in frames if "expect_end" not in f),
        "exact_ray_frames": sum(1 for f in frames if "expect_end" in f),
        "frames_with_clipped": len(clipped_frames),
        "old_ne_new_on_clipped_frames": sum(1 for r in clipped_frames if not r["old_eq_new"]),
        "clipped_frames_marking_voxels": sum(1 for r in clipped_frames if r["clipped_rays_mark_voxels"]),
        "clipped_frames_marking_nothing": [r["name"] for r in clipped_frames if not r["clipped_rays_mark_voxels"]],
        "old_eq_golden_old_rule": sum(r["old_eq_golden_old_rule"] for r in rows),
        "old_eq_new_on_unclipped_frames": sum(1 for r in rows if r["counts_new"][2] == 0 and r["old_eq_new"]),
        "frames_without_clipped": sum(1 for r in rows if r["counts_new"][2] == 0),
        "new_eq_golden": sum(r["new_eq_golden"] for r in rows),
        "golden_double_eq_rational": sum(r["golden_double_eq_rational"] for r in rows),
        "conserved": sum(r["conserved"] for r in rows),
        "mut_drop_caught_frames": len(drop_frames),
        "mut_trunc_caught_frames": len(trunc_frames),
        "mut_trunc_caught_names_first": trunc_frames[:40],
        "mut_drop_caught_names_first": drop_frames[:20],
        "failures": fail, "pass": not fail, "seconds": round(time.time() - t0, 1),
        "libs": libs, "old_rev": a.old_rev, "seed": a.seed,
    }
    json.dump({"summary": summary, "rows": rows}, open(a.out, "w"), indent=1)
    print(json.dumps({k2: v for k2, v in summary.items() if k2 not in ("libs",)}, indent=1))
    return 0 if not fail else 1


if __name__ == "__main__":
    sys.exit(main())
