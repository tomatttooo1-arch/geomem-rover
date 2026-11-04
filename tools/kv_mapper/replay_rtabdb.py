#!/usr/bin/env python3
"""NN-MAP1 host replay (2026-11-04): a recorded mapping trip -> PL navigation grids -> kv_mapper -> global grid,
compared with the RTAB-Map export of the SAME trip.

    python3 tools/kv_mapper/replay_rtabdb.py --cache NODES.pkl --rtab-yaml EXPORT.yaml --out DIR
            [--arm odom|opt] [--rtab-poses POSES.txt] [--shift-x-m 0.2] [--label NAME]

Input (tools/kv_mapper/rtabdb_extract.py cache of the trip's RTAB-Map database): every node's depth (the frames
RTAB-Map itself built its grid from, about 1 per second), its odometry pose (wheel + gyro, what the car had) and the
camera mount (base_link <- optical, the trip's ground calibration). Per node, the product chain on the host:
  depth codes + K + T_odom_camera -> host/geo_producer.GeoProducer (cpu frame backend + cpu rolling kernel, the same
  C code as the KV260 cores) -> NavigationDelta -> host/rolling_nav_layer.RollingLayerBridge with the car's
  parameters (band ground 0.0 / robot 0.31 m, radius 0.30 + 0.05, swept clearing 300 s) -> the 80 x 80 grid of
  /fpga/rolling_map/grid. B1 (runs/nationals-prelim-20261102/B1/DESIGN.md section 3) showed this bridge grid and the PL
  cost-map core's grid plane are cell-for-cell equal (0 cells on 376 frames), so it stands for "the PL grid".
  -> host/kv_mapper.KvMapper.integrate.
Arms: --arm odom (the product: odometry poses, no loop closure); --arm odomnodes (control of opt: the grid rebuilt from
the kept node grids with identity corrections -- separates "nodes only" from "corrected poses"); --arm opt (the loop-closure hook: after the odom
pass, kv_mapper.apply_correction with RTAB-Map's optimized poses -- the upper bound of a future loop closure).
--shift-x-m: positive control, every odometry pose moved +x before the geometry (the whole chain sees it).
Comparison (frozen in NN-MAP1/README.md before the first run): RTAB-Map export cell centres looked up in the
kv_mapper grid; agreement = equal tri-state over cells known in both; occupied precision / recall / IoU, exact and
with 1-cell tolerance. Writes OUT/compare.json, OUT/kv/ (the kv_mapper export), OUT/overlay.png when matplotlib is
available.
"""
import argparse
import ctypes
import json
import math
import os
import pickle
import re
import subprocess
import sys
import tempfile
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host import kv_mapper as km  # noqa: E402

RVL_C = r"""
#include <stdint.h>
static const int32_t *p; static int32_t word; static int nib;
static int vle(void){ unsigned int nibble; int value = 0, bits = 29;
  do { if (!nib) { word = *p++; nib = 8; } nibble = (unsigned int)word & 0xf0000000u;
       value |= (int)((nibble << 1) >> bits); word = (int32_t)((uint32_t)word << 4); nib--; bits -= 3; }
  while (nibble & 0x80000000u); return value; }
int rvl_decode(const char *in, uint16_t *out, int n){ p = (const int32_t*)in; nib = 0; int prev = 0, left = n;
  while (left > 0) { int z = vle(); left -= z; if (left < 0) return -1; for (; z; z--) *out++ = 0;
    int nz = vle(); left -= nz; if (left < 0) return -2;
    for (; nz; nz--) { int pos = vle(); int d = (pos >> 1) ^ -(pos & 1); prev = prev + d; *out++ = (uint16_t)prev; } }
  return 0; }
"""


def rvl_lib(cache_dir):
    so = os.path.join(cache_dir, "rvl.so")
    if not os.path.isfile(so):
        src = os.path.join(cache_dir, "rvl.c")
        with open(src, "w") as f:
            f.write(RVL_C)
        subprocess.check_call(["gcc", "-O2", "-shared", "-fPIC", "-o", so, src])
    lib = ctypes.CDLL(so)
    lib.rvl_decode.argtypes = [ctypes.c_char_p, ctypes.c_void_p, ctypes.c_int]
    return lib


def decode_depth(lib, blob):
    if blob[:8] != b"DEPTHRVL":
        raise ValueError("depth blob is not RVL (%r)" % blob[:8])
    cols, rows = np.frombuffer(blob, "<i4", 2, 8)
    out = np.zeros((rows, cols), np.uint16)
    rc = lib.rvl_decode(blob[16:], out.ctypes.data, int(rows * cols))
    if rc != 0:
        raise ValueError("RVL decode failed (%d)" % rc)
    return out


def read_pgm_yaml(yml):
    with open(yml) as f:
        txt = f.read()
    img = re.search(r"image:\s*(\S+)", txt).group(1)
    res = float(re.search(r"resolution:\s*([0-9.eE+-]+)", txt).group(1))
    org = [float(v) for v in re.search(r"origin:\s*\[([^\]]+)\]", txt).group(1).split(",")]
    path = img if os.path.isabs(img) else os.path.join(os.path.dirname(yml), img)
    raw = open(path, "rb").read()
    m = re.match(rb"P5\s+(?:#.*\s+)*(\d+)\s+(\d+)\s+(\d+)\s", raw)
    w, h = int(m.group(1)), int(m.group(2))
    a = np.frombuffer(raw[m.end():m.end() + w * h], np.uint8).reshape(h, w)[::-1]     # row 0 = lowest y
    tri = np.full(a.shape, km.UNK, np.int8)
    tri[a == km.PGM_FREE] = km.FREE_V
    tri[a == km.PGM_OCC] = km.OCC
    return tri, res, org[:2]


def compare(ref_tri, ref_res, ref_org, mapper):
    """RTAB-Map export cells -> the kv_mapper cell containing their centre."""
    tri = mapper.tri()
    h, w = ref_tri.shape
    yy, xx = np.mgrid[0:h, 0:w]
    cx = ref_org[0] + (xx + 0.5) * ref_res
    cy = ref_org[1] + (yy + 0.5) * ref_res
    gx = np.floor((cx - mapper.origin[0]) / mapper.res).astype(int)
    gy = np.floor((cy - mapper.origin[1]) / mapper.res).astype(int)
    inside = (gx >= 0) & (gx < mapper.n) & (gy >= 0) & (gy < mapper.n)
    ours = np.full(ref_tri.shape, km.UNK, np.int8)
    ours[inside] = tri[gy[inside], gx[inside]]
    both = (ref_tri != km.UNK) & (ours != km.UNK)
    agree = int(((ref_tri == ours) & both).sum())
    r_occ, o_occ = ref_tri == km.OCC, ours == km.OCC

    def dil(m):
        out = m.copy()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                out |= np.roll(np.roll(m, dy, 0), dx, 1)
        return out
    inter = int((r_occ & o_occ).sum())
    union = int((r_occ | o_occ).sum())
    prec1 = float((o_occ & dil(r_occ)).sum()) / max(1, int(o_occ.sum()))
    rec1 = float((r_occ & dil(o_occ)).sum()) / max(1, int(r_occ.sum()))
    # occupied cells of ours restricted to where RTAB-Map knows the cell (its free / occupied)
    o_occ_k = o_occ & (ref_tri != km.UNK)
    ok_k, rk_k = ours != km.UNK, ref_tri != km.UNK
    # coverage-normalised: RTAB-Map occupied cells inside what kv_mapper observed, and ours inside what RTAB observed
    rec1_cov = float((r_occ & ok_k & dil(o_occ)).sum()) / max(1, int((r_occ & ok_k).sum()))
    prec1_cov = float((o_occ & rk_k & dil(r_occ)).sum()) / max(1, int((o_occ & rk_k).sum()))
    return {"ref_cells": int(h * w), "ref_known": int((ref_tri != km.UNK).sum()), "ours_known": int((ours != km.UNK).sum()),
            "both_known": int(both.sum()), "agreement": agree / max(1, int(both.sum())),
            "free_both": int(((ref_tri == km.FREE_V) & (ours == km.FREE_V)).sum()),
            "occ_ref": int(r_occ.sum()), "occ_ours": int(o_occ.sum()), "occ_ours_in_ref_known": int(o_occ_k.sum()),
            "occ_inter": inter, "occ_iou": inter / max(1, union),
            "occ_precision": inter / max(1, int(o_occ.sum())), "occ_recall": inter / max(1, int(r_occ.sum())),
            "occ_precision_tol1": prec1, "occ_recall_tol1": rec1,
            "occ_f1_tol1": 2 * prec1 * rec1 / max(1e-12, prec1 + rec1),
            "occ_ref_in_ours_known": int((r_occ & ok_k).sum()),
            "occ_recall_tol1_in_ours_known": rec1_cov, "occ_precision_tol1_in_ref_known": prec1_cov,
            "ours_occ_where_ref_free": int((o_occ & (ref_tri == km.FREE_V)).sum()),
            "ours_free_where_ref_occ": int(((ours == km.FREE_V) & r_occ).sum())}, ours


class _GridStub(object):
    """compare()'s view of a mapper, holding a given tri-state array (instrument check)."""

    def __init__(self, tri, res, org):
        self.n = max(tri.shape) + 20
        self.res = res
        self.origin = (org[0] - 10 * res, org[1] - 10 * res)
        self._t = np.full((self.n, self.n), km.UNK, np.int8)
        self._t[10:10 + tri.shape[0], 10:10 + tri.shape[1]] = tri

    def tri(self):
        return self._t


def self_check(rtab_yaml, shift_cells=4):
    """Instrument calibration (README section 0): the export against itself (must be 1.0) and against itself moved
    shift_cells along +x (must drop by the frozen margins)."""
    ref, res, org = read_pgm_yaml(rtab_yaml)
    same, _ = compare(ref, res, org, _GridStub(ref, res, org))
    moved = np.full(ref.shape, km.UNK, np.int8)
    moved[:, shift_cells:] = ref[:, :-shift_cells]
    shifted, _ = compare(ref, res, org, _GridStub(moved, res, org))
    return {"self": same, "self_shift_%d_cells" % shift_cells: shifted}


def height_cell_for(cam_z):
    from host.geo_producer import check_block_height
    for hc in range(7, 10):
        ok, _info = check_block_height(cam_z, 0.0, height_cell=hc)
        if ok:
            return hc
    raise ValueError("no height cell fits camera height %.3f" % cam_z)


def run(a):
    from host.geo_producer import GeoProducer
    from host.m7_backend import M7BackendSelector
    from host.m7_frame_native import compile_frame_native
    from host.m7_rolling import compile_rolling
    from host import rolling_nav_layer as rnl
    from host.map_align import read_opt_poses
    with open(a.cache, "rb") as f:
        cache = pickle.load(f)
    nodes = cache["nodes"]
    if a.max_nodes:
        nodes = nodes[:a.max_nodes]
    build = a.build_dir or tempfile.mkdtemp(prefix="nnmap1_build_")
    os.makedirs(build, exist_ok=True)
    lib = rvl_lib(build)
    fso, rso = os.path.join(build, "f.so"), os.path.join(build, "r.so")
    if not os.path.isfile(fso):
        compile_frame_native(fso)
    if not os.path.isfile(rso):
        compile_rolling(rso)
    sel = M7BackendSelector(cpu_frame_library=fso)
    now = [0]
    prod = None
    cal = nodes[0]["calib"]
    T_bc = np.eye(4)
    T_bc[:3, :] = np.asarray(cal["local"], float).reshape(3, 4)
    K = np.asarray(cal["K"], float).reshape(3, 3)
    hc = height_cell_for(float(T_bc[2, 3]))
    prod = GeoProducer(sel.frame_backend("cpu"), sel.rolling_backend("cpu", cpu_library=rso), depth_unit_m=0.001,
                       clock=lambda: now[0], height_cell=hc, ground_z_m=0.0, roi=False)
    dims = prod.fusion.backend.config.dims
    band = rnl.HeightBand(ground_z_m=0.0, robot_height_m=a.robot_height_m)
    bridge = rnl.RollingLayerBridge(dims=dims, voxel_m=prod.voxel_m, band=band, clock_id=prod.clock_id,
                                    body_radius_m=0.30, body_margin_m=0.05, swept_clear_s=a.swept_clear_s)
    mapper = km.KvMapper()
    dump = {"stamp": [], "origin": [], "pose": [], "data": []} if a.dump_frames else None
    counts = {"nodes": len(nodes), "grids": 0, "no_grid": 0, "resync": 0, "dropped": {}, "geometry_ms": []}
    t_int = []
    for i, nd in enumerate(nodes):
        T_ob = np.eye(4)
        T_ob[:3, :] = np.asarray(nd["pose12"], float).reshape(3, 4)
        T_ob[0, 3] += a.shift_x_m
        T_oc = T_ob @ T_bc
        codes = decode_depth(lib, nd["depth"])
        stamp_ns = int(round(nd["stamp"] * 1e9))
        now[0] = stamp_ns + 1_000_000
        t0 = time.perf_counter()
        r = prod.process(codes, K, depth_stamp_ns=stamp_ns, acquired_ns=stamp_ns, T_odom_camera=T_oc,
                         robot_position_m=tuple(T_ob[:3, 3]), calibration_id="rtabdb")
        counts["geometry_ms"].append((time.perf_counter() - t0) * 1e3)
        delta = r.get("delta")
        if delta is None:
            k = str(r.get("nav_reason"))
            counts["dropped"][k] = counts["dropped"].get(k, 0) + 1
            continue
        ok, reason, req = bridge.on_delta(delta, now_ns=now[0])
        if req is not None or not ok:
            snap = prod.fusion.snapshot(now_ns=now[0], clock_id=prod.clock_id)
            if snap.valid:
                wire = rnl.decode_snapshot(rnl.encode_snapshot(snap, acquired_ns=stamp_ns, clock_id=prod.clock_id,
                                                               voxel_m=prod.voxel_m))
                bridge.on_snapshot(wire, now_ns=now[0])
                counts["resync"] += 1
        yaw = math.atan2(T_ob[1, 0], T_ob[0, 0])
        g = bridge.grid(body_xy=(T_ob[0, 3], T_ob[1, 3]), now_ns=now[0], body_yaw=yaw)
        if g is None:
            counts["no_grid"] += 1
            continue
        frame = km.GridFrame(nd["stamp"], g.origin_xy[0], g.origin_xy[1],
                             np.asarray(g.data, np.int8).reshape(g.height, g.width), (T_ob[0, 3], T_ob[1, 3], yaw),
                             g.resolution)
        if dump is not None:                       # GMAP1: the exact GridFrame sequence kv_mapper eats
            dump["stamp"].append(frame.stamp)
            dump["origin"].append((frame.origin_x, frame.origin_y))
            dump["pose"].append(frame.pose)
            dump["data"].append(frame.data.copy())
        t1 = time.perf_counter()
        mapper.integrate(frame)
        t_int.append((time.perf_counter() - t1) * 1e3)
        counts["grids"] += 1
        if a.display_every and i % a.display_every == 0:
            snap = prod.fusion.snapshot(now_ns=now[0], clock_id=prod.clock_id)
            if snap.valid:
                lz, ly, lx = np.nonzero(snap.states == rnl.OCCUPIED)
                wb = snap.window_base
                mapper.add_voxels(np.column_stack([lx + wb[0], ly + wb[1], lz + wb[2]]), prod.voxel_m)
    if dump is not None:
        np.savez_compressed(a.dump_frames, stamp=np.array(dump["stamp"], np.float64),
                            origin=np.array(dump["origin"], np.float64).reshape(-1, 2),
                            pose=np.array(dump["pose"], np.float64).reshape(-1, 3),
                            data=np.array(dump["data"], np.int8).reshape(-1, 80, 80))
    gm = counts.pop("geometry_ms")
    counts["geometry_ms_p50"] = float(np.median(gm)) if gm else None
    counts["integrate_ms_p50"] = float(np.median(t_int)) if t_int else None
    counts["integrate_ms_p99"] = float(np.percentile(t_int, 99)) if t_int else None
    counts["height_cell"] = hc
    counts["bridge"] = dict(bridge.counts)
    arm = {"name": a.arm, "shift_x_m": a.shift_x_m}
    if a.arm == "odomnodes":                           # control of "opt": the same node-only rebuild, identity
        t2 = time.perf_counter()
        arm["rebuilt_nodes"] = mapper.apply_correction({}, name="identity_rebuild")
        arm["rebuild_ms"] = (time.perf_counter() - t2) * 1e3
    if a.arm == "opt":
        opt = read_opt_poses(a.rtab_poses)              # {id: (stamp, 4x4)} of the export (RTAB-Map node ids)
        # kv_mapper node -> the RTAB-Map node with the nearest stamp; C = optimized x odometry^-1 (map_align's C)
        rt_stamp = np.array([v[0] for v in opt.values()])
        rt_ids = list(opt.keys())
        by_id = {nd["id"]: nd for nd in nodes}
        corr = {}
        for kn in mapper.nodes:
            j = int(np.argmin(np.abs(rt_stamp - kn["stamp"])))
            rid = rt_ids[j]
            if rid not in by_id:
                continue
            T_odom = np.eye(4)
            T_odom[:3, :] = np.asarray(by_id[rid]["pose12"], float).reshape(3, 4)
            c4 = opt[rid][1] @ np.linalg.inv(T_odom)
            corr[kn["id"]] = km.se2(c4[0, 3], c4[1, 3], math.atan2(c4[1, 0], c4[0, 0]))
        t2 = time.perf_counter()
        arm["rebuilt_nodes"] = mapper.apply_correction(corr, name="rtabmap_optimized")
        arm["rebuild_ms"] = (time.perf_counter() - t2) * 1e3
        arm["corrections"] = len(corr)
    ref_tri, ref_res, ref_org = read_pgm_yaml(a.rtab_yaml)
    metrics, ours = compare(ref_tri, ref_res, ref_org, mapper)
    os.makedirs(a.out, exist_ok=True)
    exp = mapper.export(os.path.join(a.out, "kv"), a.label, os.path.join(a.out, "kv", a.label + ".db"))
    res = {"label": a.label, "cache": os.path.abspath(a.cache), "rtab_yaml": os.path.abspath(a.rtab_yaml), "arm": arm,
           "counts": counts, "mapper": mapper.status(), "metrics": metrics,
           "export": {k: exp[k] for k in ("yaml", "db", "db_sha256", "crop", "origin")}}
    np.savez_compressed(os.path.join(a.out, "tri.npz"), ref=ref_tri, ours=ours, ref_res=ref_res, ref_org=ref_org)
    with open(os.path.join(a.out, "compare.json"), "w") as f:
        json.dump(res, f, indent=1, default=str)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(10, 5))
        for k, (t, name) in enumerate(((ref_tri, "RTAB-Map export"), (ours, "kv_mapper (%s)" % a.label))):
            img = np.full(t.shape + (3,), 0.75)
            img[t == km.FREE_V] = 1.0
            img[t == km.OCC] = 0.0
            ax[k].imshow(img, origin="lower")
            ax[k].set_title(name)
        fig.savefig(os.path.join(a.out, "overlay.png"), dpi=90)
    except Exception as exc:                       # noqa: BLE001 - the figure is optional
        res["figure_error"] = repr(exc)
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cache", required=True)
    ap.add_argument("--rtab-yaml", required=True)
    ap.add_argument("--rtab-poses", default=None)
    ap.add_argument("--out", required=True)
    ap.add_argument("--label", default="kvmap")
    ap.add_argument("--arm", choices=("odom", "odomnodes", "opt"), default="odom")
    ap.add_argument("--shift-x-m", type=float, default=0.0)
    ap.add_argument("--robot-height-m", type=float, default=0.31)
    ap.add_argument("--swept-clear-s", type=float, default=300.0)
    ap.add_argument("--max-nodes", type=int, default=0)
    ap.add_argument("--display-every", type=int, default=5)
    ap.add_argument("--build-dir", default=None)
    ap.add_argument("--dump-frames", default=None, help="GMAP1: write the GridFrame sequence (npz) fed to kv_mapper")
    ap.add_argument("--self-check", action="store_true", help="only the instrument check of the comparator")
    a = ap.parse_args(argv)
    if a.self_check:
        os.makedirs(a.out, exist_ok=True)
        r = self_check(a.rtab_yaml)
        with open(os.path.join(a.out, "self_check.json"), "w") as f:
            json.dump(r, f, indent=1)
        print(json.dumps({k: {m: round(v[m], 4) for m in ("agreement", "occ_iou", "occ_f1_tol1")} for k, v in r.items()}))
        return
    if a.arm == "opt" and not a.rtab_poses:
        ap.error("--arm opt needs --rtab-poses")
    res = run(a)
    m = res["metrics"]
    print(json.dumps({"label": res["label"], "arm": res["arm"], "grids": res["counts"]["grids"],
                      "agreement": round(m["agreement"], 4), "both_known": m["both_known"],
                      "occ_iou": round(m["occ_iou"], 4), "occ_f1_tol1": round(m["occ_f1_tol1"], 4)}))


if __name__ == "__main__":
    main()
