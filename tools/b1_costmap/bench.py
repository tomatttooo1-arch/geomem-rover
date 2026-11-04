#!/usr/bin/env python3
"""B1 bench: the PL navigation-grid C model against the Nano's software bridge, cell by cell (2026-11-02).

Input: a recorded rolling-map trace (tools/m7_rolling_pl/make_trace.py format; default the 10-26 formal trace
whose main segment is rec30, 310 car frames). Two independent routes per accepted INTEGRATE:

  software (what the car runs today)
      recorded delta words -> NavigationDelta -> host/rolling_nav_layer.RollingLayerBridge (mirror rebuilt from
      deltas, D-035 projection, body seed r 0.30 + 0.05, and in the policy arm swept clearing 300 s)
      -> /fpga/rolling_map/grid values; Nav2 local costmap = StaticLayer(trinary) + InflationLayer, replayed by
      tools/b1_costmap/nav2_ref.inflate_nav2 (Humble wavefront re-typed)
  PL (proposed)
      the same trace replayed through the g++ build of hw/m7/rolling_map.cpp (the very core in the PL, 0
      mismatches on the board 10-26) -> DUMP = the URAM words in physical (ring) order + ring offsets ->
      hw/b1_costmap/costmap.cpp -> grid / aux (strict + tall) / Nav2 cost planes

Compared per frame: final grid (body seeded), strict tri-state, tall plane, the full policy (swept clearing done
on the Nano from the PL planes), Nav2 costs (PL exact form vs Nav2 wavefront replica, and vs an exact brute-force
reference). Positive controls: one tampered obstacle cell, a wrong inflation radius, two fault builds.
The body pose is SYNTHETIC (no pose in the trace): window centre + a deterministic sub-voxel jitter; time = frame
index x 100 ms. Writes --out JSON. Python 3.8, numpy, g++.
"""
import argparse
import ctypes
import json
import math
import os
import subprocess
import sys
import tempfile
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.m7_rolling import NavigationDelta, FREE, UNKNOWN, OCCUPIED  # noqa: E402
from host import rolling_nav_layer as rnl  # noqa: E402
from tools.m7_rolling_pl.replay_trace import read_trace  # noqa: E402
from tools.b1_costmap import nav2_ref  # noqa: E402

DEFAULT_TRACE = "${FPGA_SRC}/runs/fpga-innov-20261025/fuse/traces/formal.bin"
MX, MY, MZ, V = 80, 80, 32, 0.05
Q = 65536
RMAX = 12
LUT_DIM = RMAX + 2
GROUND_Z, ROBOT_H = 0.0, 0.31          # car_inner: ground_z_m 0.0, robot_height_m CAR_ROBOT_HEIGHT (0.31)
BODY_R, BODY_MARGIN = 0.30, 0.05       # car_inner: robot_radius_m 0.30, body_margin_m 0.05
SWEPT_S = 300.0                        # car_inner SWEPT_CLEAR_S default
OP_RESET, OP_INTEGRATE, OP_DUMP = 0, 1, 3
H_BASE, H_OFFSET, H_DELTA_SEQ = 5, 8, 3

U8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
I8 = np.ctypeslib.ndpointer(dtype=np.int8, flags="C_CONTIGUOUS")
U64 = np.ctypeslib.ndpointer(dtype=np.uint64, flags="C_CONTIGUOUS")


def signed(v):
    v = int(v)
    return v - (1 << 32) if v >= 1 << 31 else v


def build_costmap(out_so, mutant=None):
    cmd = ["g++", "-O2", "-std=c++11", "-shared", "-fPIC", "-Wno-unknown-pragmas", "-Wno-unused-label",
           "-ffp-contract=off"]
    if mutant:
        cmd.append("-DFPGA_B1_MUTANT_" + mutant)
    cmd += [os.path.join(ROOT, "hw/b1_costmap/costmap.cpp"), "-o", out_so]
    subprocess.run(cmd, check=True)
    lib = ctypes.CDLL(out_so)
    f = lib.fpga_b1_costmap
    f.argtypes = [U64] + [ctypes.c_uint32] * 6 + [ctypes.c_uint32, ctypes.c_int32, ctypes.c_int32,
                                                  ctypes.c_uint32, ctypes.c_uint32, U8, I8, U8, U8]
    f.restype = ctypes.c_uint32
    return f


class PlCore(object):
    def __init__(self, fn):
        self.fn = fn

    def run(self, words, offs, band, tall_lo, body, lut, infl=True):
        grid = np.zeros(MX * MY, np.int8)
        aux = np.zeros(MX * MY, np.uint8)
        cost = np.zeros(MX * MY, np.uint8)
        be, cx, cy, r = body
        st = self.fn(np.ascontiguousarray(words, np.uint64), int(offs[0]), int(offs[1]), int(offs[2]),
                     int(band[0]), int(band[1]), int(tall_lo), int(be), int(cx), int(cy), int(r),
                     1 if infl else 0, np.ascontiguousarray(lut, np.uint8), grid, aux, cost)
        return st, grid, aux, cost


def decode_delta(header, records):
    """rolling_map.hpp delta records -> world changes (= RollingKernelHost.decode_delta)."""
    base = [signed(header[H_BASE + a]) for a in range(3)]
    off_x = int(header[H_OFFSET])
    out = []
    for rec in (int(r) for r in records):
        states, mask = rec & 0xFFFF, (rec >> 16) & 0xFF
        word, ly, lz = (rec >> 24) & 0xFF, (rec >> 32) & 0xFF, (rec >> 40) & 0xFF
        for lane in range(8):
            if mask & (1 << lane):
                lx = (8 * word + lane - off_x) % MX
                out.append((base[0] + lx, base[1] + ly, base[2] + lz, (states >> (2 * lane)) & 3))
    return tuple(out)


def band_params(base_z, band):
    layers = band.band_layers(base_z, MZ, V)
    lo, hi = int(layers[0]), int(layers[-1])
    if list(layers) != list(range(lo, hi + 1)):
        raise RuntimeError("band not contiguous")
    min_z = band.ground_z_m + rnl.SWEPT_EVIDENCE_MIN_H_M
    tall = [int(lz) for lz in layers if (int(base_z) + int(lz) + 0.5) * V >= min_z]   # = tall_occupied
    tall_lo = tall[0] if tall else hi + 1
    if tall and tall != list(range(tall_lo, hi + 1)):
        raise RuntimeError("tall layers not a suffix of the band")
    return (lo, hi), tall_lo


def body_pose(base, k):
    """Synthetic: window centre + a deterministic sub-voxel jitter (golden-ratio sequence)."""
    fx = 0.05 + 0.9 * ((k * 0.6180339887) % 1.0)
    fy = 0.05 + 0.9 * ((k * 0.7548776662 + 0.3) % 1.0)
    return ((base[0] + 40 + fx) * V, (base[1] + 40 + fy) * V, 0.0)


def body_q(base, xy, radius):
    cx = int(round((xy[0] / V - base[0]) * Q))
    cy = int(round((xy[1] / V - base[1]) * Q))
    return 1, cx, cy, int(round(radius / V * Q))


def tri_of_grid(g):
    t = np.full(g.shape, UNKNOWN, np.uint8)
    t[g == 0] = FREE
    t[g == 100] = OCCUPIED
    return t


class PlPolicy(object):
    """The bridge's _grid_body + _apply_swept, with tall_occupied replaced by the PL tall plane."""

    def __init__(self):
        self.swept = rnl.SweptCells(V)

    def grid(self, strict_tri, tall, base, body_xy, now_ns):
        dims = (MX, MY, MZ)
        mask = rnl.body_cells(base, dims, V, body_xy, BODY_R + BODY_MARGIN)
        tri = rnl.seed_body(strict_tri, mask)
        hold = int(SWEPT_S * 1e9)
        recent = self.swept.window_mask(base, dims, now_ns, hold)
        if recent.any():
            strong = recent & tall & rnl.body_cells(base, dims, V, body_xy, rnl.SWEPT_EVIDENCE_NEAR_M)
            if strong.any():
                self.swept.forget(base, strong)
        self.swept.mark(body_xy, BODY_R, now_ns)
        recent = self.swept.window_mask(base, dims, now_ns, hold)
        out = np.array(tri, dtype=np.uint8, copy=True)
        out[recent] = FREE
        return rnl.occupancy_values(out).ravel()


def new_bridge(band, swept):
    return rnl.RollingLayerBridge(dims=(MX, MY, MZ), voxel_m=V, band=band, clock_id="b1", frame_id="odom",
                                  max_map_age_ns=10 ** 15, body_radius_m=BODY_R, body_margin_m=BODY_MARGIN,
                                  swept_clear_s=SWEPT_S if swept else None)


def iou(a, b):
    u = int((a | b).sum())
    return 1.0 if u == 0 else int((a & b).sum()) / u


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trace", default=DEFAULT_TRACE)
    ap.add_argument("--out", required=True)
    ap.add_argument("--vectors", default="", help="write HLS csim vectors (a few frames) to this directory")
    ap.add_argument("--vector-frames", type=int, default=6)
    ap.add_argument("--nav2-every", type=int, default=1, help="run the Nav2 wavefront replica every N frames")
    a = ap.parse_args(argv)

    from host.kv_rolling import CpuRollingCore
    from host.m7_rolling import compile_rolling

    tmp = tempfile.mkdtemp(prefix="b1bench_")
    roll = CpuRollingCore(compile_rolling(os.path.join(tmp, "roll.so")))
    pl = PlCore(build_costmap(os.path.join(tmp, "cm.so")))
    mutants = {m: PlCore(build_costmap(os.path.join(tmp, "cm_%s.so" % m), m)) for m in ("RING_Y", "UNK_FREE")}

    band = rnl.HeightBand(ground_z_m=GROUND_Z, robot_height_m=ROBOT_H)
    params = nav2_ref.InflationParams(resolution=V, inflation_radius=0.45, cost_scaling_factor=3.0,
                                      robot_radius=BODY_R)
    lut = params.pl_lut(LUT_DIM)
    wrong = nav2_ref.InflationParams(resolution=V, inflation_radius=0.40, cost_scaling_factor=3.0,
                                     robot_radius=BODY_R)
    lut_wrong = wrong.pl_lut(LUT_DIM)

    recs = read_trace(a.trace)
    br_body = br_pol = None
    pol = None
    frames = 0
    agg = {k: 0 for k in ("grid_cells_diff", "strict_cells_diff", "tall_cells_diff", "policy_cells_diff",
                          "cost_vs_nav2_cells_diff", "cost_vs_exact_cells_diff", "nav2_vs_exact_cells_diff",
                          "replay_delta_mismatch", "frames_with_obstacles", "cost_frames", "pl_status_bad")}
    agg["min_obstacle_iou"] = 1.0
    agg["nav2_vs_exact_max_abs"] = 0
    agg["cost_vs_nav2_max_abs"] = 0
    controls = {"tamper_detected_frames": 0, "tamper_tried": 0, "wrong_radius_detected_frames": 0,
                "wrong_radius_tried": 0, "wrong_radius_observable_frames": 0,
                "wrong_radius_detected_but_not_observable": 0, "wrong_radius_observable_but_missed": 0, "RING_Y_detected_frames": 0, "UNK_FREE_detected_frames": 0,
                "mutant_tried": 0}
    times = {"sw_on_delta": [], "sw_grid_body": [], "sw_grid_swept": [], "sw_tolist": [], "pl_cmodel": [],
             "nav2_replica": []}
    vec = []
    examples = []
    rng = np.random.RandomState(20261102)

    for idx, r in enumerate(recs):
        kw = {}
        if r["obs"] is not None:
            kw["obs"] = r["obs"]
        if r["op"] == 2:
            kw["query"], kw["qc"] = r["query"], r["qc"]
        hdr, delta, _q, _d, _t = roll.call(r["op"], r["epoch"], r["frame"], r["wb"], r["ob"], r["dims"], **kw)
        if int(hdr[0]) != int(r["header"][0]):
            raise RuntimeError("replay out of sync at record %d" % idx)
        if r["op"] == OP_RESET and int(hdr[0]) == 0:
            br_body, br_pol, pol = new_bridge(band, False), new_bridge(band, True), PlPolicy()
            continue
        if r["op"] != OP_INTEGRATE or int(hdr[0]) != 0:
            continue
        if delta.size != r["delta"].size or np.any(delta != r["delta"]):
            agg["replay_delta_mismatch"] += 1
        frames += 1
        now = frames * 100_000_000
        base = tuple(signed(r["header"][H_BASE + k]) for k in range(3))
        nd = NavigationDelta(int(r["epoch"]), int(r["header"][H_DELTA_SEQ]), int(r["frame"]), now, "b1",
                             now + 10 ** 15, base, (MX, MY, MZ), V, decode_delta(r["header"], r["delta"]), "trace")
        t0 = time.perf_counter()
        ok1, why1, _ = br_body.on_delta(nd, now_ns=now)
        t1 = time.perf_counter()
        ok2, why2, _ = br_pol.on_delta(nd, now_ns=now)
        if not (ok1 and ok2):
            raise RuntimeError("bridge rejected frame %d: %s %s" % (frames, why1, why2))
        try:
            band.band_layers(base[2], MZ, V)
        except ValueError:
            agg["skipped_band_outside_window"] = agg.get("skipped_band_outside_window", 0) + 1
            roll.call(OP_DUMP, 0, 0, (0, 0, 0), (0, 0, 0), (0, 0, 0))
            continue
        xy = body_pose(base, frames)
        t2 = time.perf_counter()
        sw_grid = br_body.grid(body_xy=xy[:2], now_ns=now, body_yaw=xy[2]).data.astype(np.int8)
        t3 = time.perf_counter()
        sw_pol = br_pol.grid(body_xy=xy[:2], now_ns=now, body_yaw=xy[2]).data.astype(np.int8)
        t4 = time.perf_counter()
        _ = sw_pol.astype(int).tolist()                 # what ros_rolling_nav.grid_msg does, twice per delta
        t5 = time.perf_counter()
        sw_strict = br_body.strict_grid().data.astype(np.int8)
        sw_tall = rnl.tall_occupied(br_body.mirror.states, base[2], V, GROUND_Z + rnl.SWEPT_EVIDENCE_MIN_H_M,
                                    band.band_layers(base[2], MZ, V)).ravel()
        times["sw_on_delta"].append(t1 - t0)
        times["sw_grid_body"].append(t3 - t2)
        times["sw_grid_swept"].append(t4 - t3)
        times["sw_tolist"].append(t5 - t4)

        # PL route: the resident map exactly as in URAM
        dh, _, _, words, _ = roll.call(OP_DUMP, 0, 0, (0, 0, 0), (0, 0, 0), (0, 0, 0))
        if int(dh[0]) != 0:
            raise RuntimeError("dump failed")
        offs = [int(dh[H_OFFSET + k]) for k in range(3)]
        if tuple(signed(dh[H_BASE + k]) for k in range(3)) != base:
            raise RuntimeError("dump base differs from the integrate base")
        bp, tall_lo = band_params(base[2], band)
        body = body_q(base, xy, BODY_R + BODY_MARGIN)
        t6 = time.perf_counter()
        st, g, aux, cost = pl.run(words, offs, bp, tall_lo, body, lut)
        times["pl_cmodel"].append(time.perf_counter() - t6)
        if st != 0:
            agg["pl_status_bad"] += 1
            continue
        strict = (aux & 3).astype(np.uint8)
        tall = ((aux >> 2) & 1).astype(bool)
        d_grid = int((g != sw_grid).sum())
        d_strict = int((rnl.occupancy_values(strict) != sw_strict).sum())
        d_tall = int((tall != sw_tall).sum())
        pl_pol = pol.grid(strict.reshape(MY, MX), tall.reshape(MY, MX), base, xy[:2], now).astype(np.int8)
        d_pol = int((pl_pol != sw_pol).sum())
        agg["grid_cells_diff"] += d_grid
        agg["strict_cells_diff"] += d_strict
        agg["tall_cells_diff"] += d_tall
        agg["policy_cells_diff"] += d_pol
        occ_sw, occ_pl = sw_grid == 100, g == 100
        if occ_sw.any() or occ_pl.any():
            agg["frames_with_obstacles"] += 1
        agg["min_obstacle_iou"] = min(agg["min_obstacle_iou"], iou(occ_sw, occ_pl))
        if d_grid or d_strict or d_tall or d_pol:
            if len(examples) < 10:
                examples.append({"frame": frames, "grid": d_grid, "strict": d_strict, "tall": d_tall,
                                 "policy": d_pol})

        if frames % a.nav2_every == 0:
            master = nav2_ref.master_from_grid(sw_grid.reshape(MY, MX))
            t7 = time.perf_counter()
            nav = nav2_ref.inflate_nav2(master, params)
            times["nav2_replica"].append(time.perf_counter() - t7)
            exact = nav2_ref.inflate_exact(master, params)
            c2 = cost.reshape(MY, MX).astype(np.int32)
            agg["cost_frames"] += 1
            agg["cost_vs_nav2_cells_diff"] += int((c2 != nav).sum())
            agg["cost_vs_exact_cells_diff"] += int((c2 != exact).sum())
            agg["nav2_vs_exact_cells_diff"] += int((nav != exact).sum())
            agg["nav2_vs_exact_max_abs"] = max(agg["nav2_vs_exact_max_abs"], int(np.abs(nav - exact).max()))
            agg["cost_vs_nav2_max_abs"] = max(agg["cost_vs_nav2_max_abs"], int(np.abs(c2 - nav).max()))
            # control 2: wrong inflation radius must show up whenever there is an obstacle
            if occ_sw.any():
                controls["wrong_radius_tried"] += 1
                _, _, _, cw = pl.run(words, offs, bp, tall_lo, body, lut_wrong)
                hit = int((cw.reshape(MY, MX).astype(np.int32) != nav).sum()) > 0
                # observable = Nav2 itself would give a different master costmap with the wrong radius
                seen = bool(np.any(nav2_ref.inflate_nav2(master, wrong) != nav))
                controls["wrong_radius_detected_frames"] += int(hit)
                controls["wrong_radius_observable_frames"] += int(seen)
                controls["wrong_radius_detected_but_not_observable"] += int(hit and not seen)
                controls["wrong_radius_observable_but_missed"] += int(seen and not hit)

        # control 1: one tampered obstacle cell -- a band voxel of a strict-FREE column set to occupied
        free_cols = np.nonzero(strict == FREE)[0]
        if len(free_cols):
            controls["tamper_tried"] += 1
            ci = int(free_cols[rng.randint(len(free_cols))])
            ly, lx = divmod(ci, MX)
            lz = bp[0] + rng.randint(bp[1] - bp[0] + 1)
            px, py_, pz = (lx + offs[0]) % MX, (ly + offs[1]) % MY, (lz + offs[2]) % MZ
            wi = (pz * MY + py_) * (MX // 8) + px // 8
            tw = words.copy()
            lane = px % 8
            tw[wi] = (int(tw[wi]) & ~(0xFF << (8 * lane)) | (32 << (8 * lane))) & 0xFFFFFFFFFFFFFFFF
            _, gt, _, _ = pl.run(tw, offs, bp, tall_lo, body, lut, infl=False)
            if int((gt != sw_grid).sum()) > 0:
                controls["tamper_detected_frames"] += 1
        # control 3: fault builds
        controls["mutant_tried"] += 1
        for name, core in mutants.items():
            _, gm, _, _ = core.run(words, offs, bp, tall_lo, body, lut, infl=False)
            if int((gm != sw_grid).sum()) > 0:
                controls[name + "_detected_frames"] += 1

        if a.vectors and len(vec) < a.vector_frames and frames % 37 == 5:
            vec.append((words.copy(), offs, bp, tall_lo, body, sw_grid.copy(),
                        nav2_ref.inflate_exact(nav2_ref.master_from_grid(sw_grid.reshape(MY, MX)), params)
                        .astype(np.uint8).ravel(),
                        (tri_of_grid(sw_strict) | (sw_tall.astype(np.uint8) << 2)).astype(np.uint8)))

    def stat(v):
        if not v:
            return None
        s = sorted(v)
        return {"n": len(s), "median_ms": round(1e3 * s[len(s) // 2], 4), "p99_ms": round(1e3 * s[int(0.99 * (len(s) - 1))], 4),
                "max_ms": round(1e3 * s[-1], 4)}

    result = {"trace": a.trace, "frames_compared": frames, "aggregate": agg, "controls": controls,
              "first_differences": examples, "host_times": {k: stat(v) for k, v in times.items()},
              "params": {"ground_z_m": GROUND_Z, "robot_height_m": ROBOT_H, "body_radius_m": BODY_R,
                         "body_margin_m": BODY_MARGIN, "swept_clear_s": SWEPT_S,
                         "nav2": {"inflation_radius": params.inflation_radius,
                                  "cell_inflation_radius": params.cell_radius,
                                  "inscribed_radius": params.inscribed_radius,
                                  "cost_scaling_factor": params.cost_scaling_factor},
                         "lut_row0": [int(v) for v in lut[:LUT_DIM]],
                         "body_pose": "synthetic: window centre + sub-voxel jitter; time = frame x 100 ms"}}
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(result, f, indent=1)
    if a.vectors:
        os.makedirs(a.vectors, exist_ok=True)
        with open(os.path.join(a.vectors, "vectors.bin"), "wb") as f:
            f.write(np.array([len(vec)], np.uint32).tobytes())
            f.write(lut.astype(np.uint8).tobytes())
            for words, offs, bp, tall_lo, body, g, c, aux in vec:
                f.write(np.array(list(offs) + [bp[0], bp[1], tall_lo, body[0]], np.uint32).tobytes())
                f.write(np.array([body[1], body[2]], np.int32).tobytes())
                f.write(np.array([body[3]], np.uint32).tobytes())
                f.write(np.ascontiguousarray(words, np.uint64).tobytes())
                f.write(g.astype(np.int8).tobytes())
                f.write(aux.astype(np.uint8).tobytes())
                f.write(c.astype(np.uint8).tobytes())
    print(json.dumps({"frames": frames, "aggregate": agg, "controls": controls}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
