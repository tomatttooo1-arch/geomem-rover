"""M7 same-function CPU baseline, numpy arm and repo host-pipeline arm.

Stages per frame (same semantics as hw/m7 kernels, see m7_cpu_bench.cpp):
  S0 decode, S1 validity+projection+Q8, S2 vectorised integer DDA
  (all rays advanced together, one numpy step per boundary crossing),
  S3 object evidence (masked object with median + ROI-only object).
Every numpy frame is checked against the unchanged repo C++ kernels called
through host/m7_frame_native.py and host/m7_object_native.py (built with the
repo's own -O2 flags). Any mismatch is a hard failure.

The "pipeline" arm times the repo's host/m7_pipeline.py as-is (FrameNativeM7
backend + NativeObjectEvidence object backend): ingest_geometry +
two ingest_semantic calls per frame, including all validation/copy overhead.

Usage (repo root on sys.path):
  python3 m7_numpy_bench.py --repo <root> --depth f.bin --width W --height H
      --frames N --fx .. --fy .. --cx .. --cy .. --unit-m .. --grid g1|g2
      --arm numpy|pipeline --passes P --warmup K --csv out.csv --build <fresh dir>
"""
import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np

GRIDS = {"g1": ((-2., -2., -1.), .25, (16, 16, 20)),
         "g2": ((-2.4, -2.4, -0.4), .15, (32, 32, 32)),
         "p32": ((-0.8, -0.8, -0.05), .05, (32, 32, 32))}   # tools/nano_loop/chain.py default_grid()
Q8_LIMIT = 1 << 24   # |Q8| range gate of hw/m7/frame_depth.cpp (M7_CLIP_SPEC_20261011)

CHUNK = 65536


def _prep(ends, o):
    # ends may lie outside the grid (clipped rays): // is floor division.
    cell = np.broadcast_to(o // 256, ends.shape).copy()
    target = ends // 256
    direction = np.sign(ends - o)
    delta = np.abs(ends - o)
    nxt = np.where(direction > 0, (cell + 1) * 256 - o, o - cell * 256)
    return cell, target, direction, delta, nxt


def s2_events(ends, o, dims, free):
    """Loop-free exact traversal. Axis a is crossed n_a=|target_a-cell_a| times,
    the j-th crossing at time (next_a+256j)/delta_a. The reference processes
    crossings in time order and steps every axis tied at the minimum, so after
    the crossing event at time num/den the cell is start + dir*count_a with
    count_a = #{j < n_a : (next_a+256j)/delta_a <= num/den}
            = clip(floor((num*delta_a - next_a*den) / (256*den)) + 1, 0, n_a),
    all in exact int64. Cells after an event that are not the target are FREE.
    Clip semantics (M7_CLIP_SPEC_20261011): the crossings of axis a are capped
    at the one that leaves the grid (dims[a]-cell0 going up, cell0+1 going
    down); events whose cell is outside the grid are dropped. A segment meets
    the grid box in one interval, so nothing after the exit is in the grid."""
    # int64: num <= 33*256 but a clipped endpoint has delta < 2^25, products < 2^39.
    nx, ny, nz = dims
    cell0 = (o // 256).astype(np.int64)
    e = ends.astype(np.int64)
    o64 = o.astype(np.int64)
    cols = []
    for a in range(3):
        ea = e[:, a]
        ta = ea // 256
        da = np.sign(ea - o64[a]).astype(np.int64)
        dl = np.abs(ea - o64[a])
        nt = np.where(da > 0, (cell0[a] + 1) * 256 - o64[a], o64[a] - cell0[a] * 256)
        cap = np.where(da > 0, dims[a] - cell0[a], np.where(da < 0, cell0[a] + 1, 0))
        nc = np.minimum(np.abs(ta - cell0[a]), cap)
        cols.append((ta, da, dl, nt.astype(np.int64), nc))
    ray_l, num_l, den_l = [], [], []
    m = ends.shape[0]
    for k in range(3):
        ta, da, dl, nt, nk = cols[k]
        tot = int(nk.sum())
        if tot == 0:
            continue
        ray = np.repeat(np.arange(m, dtype=np.int64), nk)
        start = np.cumsum(nk) - nk
        j = np.arange(tot, dtype=np.int64) - np.repeat(start.astype(np.int64), nk)
        ray_l.append(ray)
        num_l.append(nt[ray] + 256 * j)
        den_l.append(dl[ray])
    if not ray_l:
        return
    ray = np.concatenate(ray_l)
    num = np.concatenate(num_l)
    den = np.concatenate(den_l)
    den256 = den * 256
    inv = 1.0 / den256.astype(np.float64)
    keep = np.zeros(ray.size, dtype=bool)
    ingrid = np.ones(ray.size, dtype=bool)
    lin = np.zeros(ray.size, dtype=np.int64)
    mult = (1, nx, nx * ny)
    for a in range(3):
        ta, da, dl, nt, nc = cols[a]
        x = num * dl[ray] - nt[ray] * den          # exact int64
        q = np.floor(x * inv).astype(np.int64)    # float estimate, then exact correction
        q -= (q * den256 > x)
        q += ((q + 1) * den256 <= x)
        cnt = np.minimum(np.maximum(q + 1, 0), nc[ray])
        ca = cell0[a] + da[ray] * cnt
        keep |= ca != ta[ray]
        ingrid &= (ca >= 0) & (ca < dims[a])
        lin += ca * mult[a]
    free[lin[keep & ingrid]] = True


def s2_steploop(ends, o, dims, free):
    """Direct transcription: all rays advanced together, one numpy step per crossing.
    A ray stops at its first cell outside the grid (clip semantics)."""
    nx, ny, nz = dims
    dim = np.array(dims, dtype=np.int64)
    cell, target, direction, delta, nxt = _prep(ends, o)
    act = np.nonzero(np.any(cell != target, axis=1))[0]
    for _ in range(3 * 32):
        if act.size == 0:
            break
        c, tg, d, dl, nt = cell[act], target[act], direction[act], delta[act], nxt[act]
        elig = (d != 0) & (c != tg)
        first = np.full(act.size, -1, dtype=np.int64)
        for a in range(3):
            fi = np.maximum(first, 0)
            nf = np.take_along_axis(nt, fi[:, None], 1)[:, 0]
            df = np.take_along_axis(dl, fi[:, None], 1)[:, 0]
            better = elig[:, a] & ((first < 0) | (nt[:, a] * df < nf * dl[:, a]))
            first = np.where(better, a, first)
        ok = first >= 0
        fi = np.maximum(first, 0)
        num = np.take_along_axis(nt, fi[:, None], 1)[:, 0]
        den = np.take_along_axis(dl, fi[:, None], 1)[:, 0]
        stepm = elig & (nt * den[:, None] == num[:, None] * dl) & ok[:, None]
        c = c + np.where(stepm, d, 0)
        nt = nt + np.where(stepm, 256, 0)
        cell[act], nxt[act] = c, nt
        moving = ok & np.any(c != tg, axis=1) & np.all((c >= 0) & (c < dim), axis=1)
        mc = c[moving]
        free[(mc[:, 2] * ny + mc[:, 1]) * nx + mc[:, 0]] = True
        act = act[moving]


S2_IMPL = s2_events


def numpy_frame(raw, unit, ik32, wc32, origin, voxel, shape, u32, v32, mask, roi, ikd, lc):
    nx, ny, nz = shape
    t = time.perf_counter()
    z = raw.astype(np.float32) * np.float32(unit)
    z[raw == 0] = np.nan
    s0 = time.perf_counter() - t
    # S1
    t = time.perf_counter()
    valid = (z > np.float32(.1)) & (z < np.float32(10.))
    zv, uv, vv = z[valid], u32[valid], v32[valid]
    cam = [(ik32[3 * r] * uv + ik32[3 * r + 1] * vv + ik32[3 * r + 2]) * zv for r in range(3)]
    world = [wc32[4 * r] * cam[0] + wc32[4 * r + 1] * cam[1] + wc32[4 * r + 2] * cam[2] + wc32[4 * r + 3]
             for r in range(3)]
    finite = np.isfinite(world[0]) & np.isfinite(world[1]) & np.isfinite(world[2])
    q = np.stack([np.floor((world[a].astype(np.float64) - origin[a]) / voxel * 256) for a in range(3)], axis=1)
    dims = np.array(shape, dtype=np.float64) * 256
    walk = finite & np.all((q >= -Q8_LIMIT) & (q < Q8_LIMIT), axis=1)   # accepted + clipped
    inside = walk & np.all((q >= 0) & (q < dims), axis=1)
    ends = q[walk].astype(np.int64)
    ends_in = ends[inside[walk]]
    accepted = int(ends_in.shape[0])
    invalid = int(z.size - valid.sum() + (~walk).sum())
    outside = int((walk & ~inside).sum())   # clipped: walked, no HIT
    s1 = time.perf_counter() - t
    # S2 (see s2_events / s2_steploop): both reproduce the reference traversal exactly
    t = time.perf_counter()
    o = np.floor((wc32[[3, 7, 11]].astype(np.float64) - origin) / voxel * 256).astype(np.int64)
    n = nx * ny * nz
    free = np.zeros(n, dtype=bool)
    hit = np.zeros(n, dtype=bool)
    target = ends_in // 256
    hit[(target[:, 2] * ny + target[:, 1]) * nx + target[:, 0]] = True
    for lo in range(0, ends.shape[0], CHUNK):
        S2_IMPL(ends[lo:lo + CHUNK], o, shape, free)
    flags = np.where(hit, 2, np.where(free, 1, 0)).astype(np.uint8)
    s2 = time.perf_counter() - t
    # S3 objects
    t = time.perf_counter()
    good = valid & mask
    yy, xx = np.nonzero(good)
    zz = z[yy, xx].astype(np.float64)
    fx_, fy_ = xx.astype(np.float64), yy.astype(np.float64)
    camd = [(ikd[3 * r] * fx_ + ikd[3 * r + 1] * fy_ + ikd[3 * r + 2]) * zz for r in range(3)]
    pts = np.stack([lc[4 * r] * camd[0] + lc[4 * r + 1] * camd[1] + lc[4 * r + 2] * camd[2] + lc[4 * r + 3]
                    for r in range(3)], axis=1)
    pts = pts[np.all(np.isfinite(pts), axis=1)]
    mask_px = int(mask.sum())
    npts = pts.shape[0]
    median = np.median(pts, axis=0) if npts else np.full(3, np.nan)
    cand_m = 1 if (npts >= 9 and npts / mask_px >= .7) else 0
    x0, y0, x1, y1 = roi
    patch = z[y0:y1 + 1, x0:x1 + 1]
    pv = (patch > np.float32(.1)) & (patch < np.float32(10.))
    rp, rv = patch.size, int(pv.sum())
    front, back = np.float32(1.0 - .04), np.float32(1.2 + .04)
    d = patch[pv]
    rf, rb = int((d < front).sum()), int((d > back).sum())
    cand_r = 0
    if rv >= 9 and rv / rp >= .7:
        cand_r = 0 if rf / rv >= .5 else (2 if rb / rv >= .9 else 0)
    s3 = time.perf_counter() - t
    return dict(s0=s0, s1=s1, s2=s2, s3=s3, flags=flags, counts=(accepted, invalid, outside),
                obj_m=(mask_px, npts, cand_m), pts=pts, median=median, obj_r=(rp, rv, rf, rb, cand_r))


def main():
    ap = argparse.ArgumentParser()
    for k in ("repo", "depth", "csv", "build"):
        ap.add_argument("--" + k, required=True)
    for k in ("width", "height", "frames", "passes", "warmup"):
        ap.add_argument("--" + k, type=int, required=k in ("width", "height", "frames"))
    for k in ("fx", "fy", "cx", "cy", "unit-m"):
        ap.add_argument("--" + k, type=float, required=True)
    ap.add_argument("--grid", default="g1")
    ap.add_argument("--arm", default="numpy")
    a = ap.parse_args()
    passes = a.passes or 3
    warmup = 1 if a.warmup is None else a.warmup
    sys.path.insert(0, a.repo)
    from host.m7_frame_native import FrameNativeM7, compile_frame_native
    from host.m7_object_native import NativeObjectEvidence, compile_object_native
    from perception.fpga_replay.core import Config
    build = Path(a.build)
    build.mkdir(parents=True, exist_ok=False)
    frame_lib = FrameNativeM7(compile_frame_native(build / "frame.so"))
    obj_lib = NativeObjectEvidence(compile_object_native(build / "object.so"))
    W, H, N = a.width, a.height, a.frames
    raw = np.fromfile(a.depth, dtype=np.uint16, count=W * H * N).reshape(N, H, W)
    origin, voxel, shape = GRIDS[a.grid]
    origin = np.array(origin)
    K = np.array([[a.fx, 0, a.cx], [0, a.fy, a.cy], [0, 0, 1.]])
    ikd = np.array([1 / a.fx, 0, -a.cx / a.fx, 0, 1 / a.fy, -a.cy / a.fy, 0, 0, 1])
    ik32 = np.linalg.inv(K).astype(np.float32).ravel()  # what FrameNativeM7 passes the kernel
    T = np.eye(4)
    wc32 = T[:3].astype(np.float32).ravel()
    lc = T[:3].ravel()
    vv, uu = np.mgrid[0:H, 0:W]
    u32, v32 = uu.astype(np.float32), vv.astype(np.float32)
    mask = np.zeros((H, W), dtype=bool)
    mask[H // 2 - 24:H // 2 + 24, W // 2 - 24:W // 2 + 24] = True
    roi = (W // 4 - 32, H // 2 - 32, W // 4 + 31, H // 2 + 31)
    cfg = Config()
    rows, mismatches, compared = [], 0, 0
    # reference outputs (repo kernels via repo wrappers), untimed
    ref = []
    for f in range(N):
        z = raw[f].astype(np.float32) * np.float32(a.unit_m)
        z[raw[f] == 0] = np.nan
        ev = frame_lib.frame_from_depth(depth_m=z, k=K, camera_to_world=T, grid_origin_m=tuple(origin),
                                        voxel_size_m=voxel, shape=shape, frame_id=f, capture_time_ns=f + 1,
                                        pose_time_ns=f + 1, calibration_version="bench", pose_confirmed=True)
        om = obj_lib.evaluate(z, mask, K, T, cfg=cfg)
        orr = obj_lib.evaluate(z, np.zeros_like(mask), K, T, roi=roi, near_z=1.0, far_z=1.2, cfg=cfg)
        ref.append((ev, om, orr))
    global S2_IMPL
    if a.arm == "numpy_step":
        S2_IMPL = s2_steploop
    if a.arm in ("numpy", "numpy_step"):
        # the numpy arm uses the same float32 inverse K the C wrapper passes
        for p in range(warmup + passes):
            for f in range(N):
                t = time.perf_counter()
                r = numpy_frame(raw[f], a.unit_m, ik32, wc32, origin, voxel, shape, u32, v32, mask, roi, ikd, lc)
                e2e = time.perf_counter() - t
                ev, om, orr = ref[f]
                same = (np.array_equal(r["flags"], ev.flags.ravel()) and
                        r["counts"] == (ev.accepted_rays, ev.invalid_depth_or_projection, ev.outside_grid) and
                        r["obj_m"] == (om.counts["mask_pixels"], om.counts["projected_valid_points"], om.counts["candidate_code"]) and
                        np.array_equal(r["pts"], om.points_local_m) and
                        r["obj_r"] == (orr.counts["roi_pixels"], orr.counts["roi_valid_pixels"], orr.counts["roi_front_pixels"],
                                       orr.counts["roi_behind_pixels"], orr.counts["candidate_code"]))
                compared += 1
                mismatches += 0 if same else 1
                if p >= warmup:
                    rows.append([a.arm, a.grid, 1, p - warmup, f, r["s0"] * 1e3, r["s1"] * 1e3, r["s2"] * 1e3,
                                 (r["s1"] + r["s2"]) * 1e3, r["s3"] * 1e3, e2e * 1e3, *r["counts"],
                                 int((r["flags"] == 1).sum()), int((r["flags"] == 2).sum()), r["obj_m"][1], r["obj_r"][1],
                                 r["obj_m"][2], r["obj_r"][4]])
    elif a.arm == "pipeline":
        from host.m7_pipeline import (CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, PoseAtCapture, SemanticPacket)
        grid = GridSpec(tuple(origin), voxel, shape)
        pipe = M7Pipeline(frame_lib, grid, object_backend=obj_lib)
        # historical bounds projecting over the ROI rectangle at 1.0-1.2 m (camera frame == local frame)
        zc = 1.1
        bounds = np.array([[(roi[0] - a.cx) / a.fx * zc, (roi[1] - a.cy) / a.fy * zc, 1.0],
                           [(roi[2] - a.cx) / a.fx * zc, (roi[3] - a.cy) / a.fy * zc, 1.2]])
        empty = np.zeros_like(mask)
        fid = 0
        for p in range(warmup + passes):
            for f in range(N):
                fid += 1
                acq = fid * 33_000_000
                z = raw[f].astype(np.float32) * np.float32(a.unit_m)  # decode is part of this arm too
                t = time.perf_counter()
                z[raw[f] == 0] = np.nan
                ident = CaptureIdentity("bench", fid, acq, "c", "cal", "local", "e")
                fr = DepthFrame(ident, z, K, PoseAtCapture(acq, "c", "local", "e", True, T))
                t1 = time.perf_counter()
                g = pipe.ingest_geometry(fr, now_ns=acq + 1_000_000, now_clock_id="c")
                t2 = time.perf_counter()
                assert g.navigation.valid, g.navigation.reason
                tok = g.navigation.frame_token
                m1 = pipe.ingest_semantic(SemanticPacket(ident, tok, "obj-mask", mask, None), now_ns=acq + 2_000_000, now_clock_id="c")
                m2 = pipe.ingest_semantic(SemanticPacket(ident, tok, "obj-roi", empty, bounds), now_ns=acq + 2_000_000, now_clock_id="c")
                t3 = time.perf_counter()
                ev, om, orr = ref[f]
                same = np.array_equal(g.navigation.payload["voxel_flags"], ev.flags) and m1.valid and m2.valid
                compared += 1
                mismatches += 0 if same else 1
                if p >= warmup:
                    pl = g.navigation.payload
                    rows.append(["pipeline", a.grid, 1, p - warmup, f, (t1 - t) * 1e3, "", "", (t2 - t1) * 1e3, (t3 - t2) * 1e3,
                                 (t3 - t) * 1e3, pl["valid_grid_samples"], pl["invalid_depth_or_projection"], pl["outside_grid"],
                                 pl["free_voxels"], pl["hit_voxels"], m1.payload.get("mask_valid_pixels", ""),
                                 m2.payload.get("region_valid_pixels", ""), m1.payload["candidate"], m2.payload["candidate"]])
    else:
        raise SystemExit("arm numpy|pipeline")
    with open(a.csv, "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["arm", "grid", "threads", "pass", "frame", "s0_decode_ms", "s1_project_ms", "s2_rays_ms", "s1s2_fused_ms",
                    "s3_object_ms", "e2e_ms", "accepted", "invalid", "outside", "free_voxels", "hit_voxels", "mask_valid",
                    "roi_valid", "candidate_mask", "candidate_roi"])
        w.writerows(rows)
    print(json.dumps({"arm": a.arm, "grid": a.grid, "frames": N, "passes": passes, "warmup": warmup,
                      "frames_compared_to_repo_kernels": compared, "mismatches": mismatches,
                      "numpy": np.__version__, "python": sys.version.split()[0]}))
    sys.exit(7 if mismatches else 0)


if __name__ == "__main__":
    main()
