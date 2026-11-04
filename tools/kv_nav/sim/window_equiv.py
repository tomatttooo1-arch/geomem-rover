#!/usr/bin/env python3
"""CTRLPS4 equivalence: the KV-side navigation window (host/kv_nav.KvWindowLayer via make_core + RollingGridSource)
against the Nano bridge as it runs today, frame by frame, cell by cell, on a car recording with depth + poses.

Input: car1015 (2026-10-15 drive, 1229 frames, 20 min): depth from the G1d pack (runs/nationals-prelim-20261102/G1d/
packs/car1015.pack, uint16 mm, identical to the source npz, dev2 0 px), poses + stamps from dev2's hindsight run
(car1015.npz: camera = RTAB-Map odom<-camera, body = camera x inverse(mount); offline-optimised, used here as odom).

One fusion chain, production code (as tools/kv_nav/sim/cell_offset_probe.py): M7Pipeline + FrameNativeM7
(FollowCameraGrid 32x32x10, height_cell 8) -> Nano RollingFusion (enable_grid, skip_changes False = the car's
CAR_PL_GRID_SHADOW mode: deltas AND planes) -> RemotePlRollingBackend over an in-process link -> KV RollingService +
CpuRollingCore + B1 grid C model. Then per accepted frame, all arms at the same pose (body x, y, yaw) and time:

  N1  Nano, CAR_PL_GRID=1: host/pl_grid_layer.PlGridLayer.on_planes(the ACK planes) -> grid(pose, now, yaw)
      (ros_geo_producer GridOutput.grid_update / publish_grid)
  N0  Nano, CAR_PL_GRID=0 (the car default): host/rolling_nav_layer.RollingLayerBridge fed the wire-coded deltas
      (mirror, D-035 projection, tall_occupied from the 3-D mirror) -> grid(...) -- an independent code path
      for the strict grid and the tall evidence
  KV  make_core("soft", RollingService, grid_params) -> KvNavCore.on_odom + step(now_s) -> RollingGridSource
      (RollingService.nav_planes, KvWindowLayer) -> last_window; the cost window fed to the core vs
      Inflation(N1 window)

Configurations: A = car default (swept_clear_s 300, front seed off); B = front seed 0.65 m / 35 deg (runbook value)
+ swept 300. Positive controls (must differ): P_deg = B with the KV sector at 45 deg; P_swept = A with the KV swept
clearing off. Also reported: cells the swept clearing / front seed changed (the features fire on this data).
Writes OUT/summary.json, OUT/frames.jsonl (per frame diff counts).
Run: PYTHONPATH=. python3 tools/kv_nav/sim/window_equiv.py OUT [--frames N]
"""
import argparse
import json
import math
import struct
import sys
import tempfile
import time
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from host import rolling_nav_layer as rnl  # noqa: E402
from host.kv_nav import Inflation, load_library, make_core  # noqa: E402
from host.kv_rolling import CpuRollingCore, RemotePlRollingBackend, RollingService  # noqa: E402
from host.m7_frame_native import FrameNativeM7, compile_frame_native  # noqa: E402
from host.m7_grid_policy import FollowCameraGrid  # noqa: E402
from host.m7_pipeline import CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, PoseAtCapture  # noqa: E402
from host.m7_rolling import RollingFusion, compile_rolling  # noqa: E402
from host.pl_grid_layer import PlGridLayer  # noqa: E402
from tools.kv_nav.sim.cell_offset_probe import InProcLink  # noqa: E402

PACK = "${FPGA_RUN}/G1d/packs/car1015.pack"
META = PACK + ".meta.json"
POSES = "${FPGA_SRC}/runs/hindsight-20261103/car1015/car1015.npz"
V = 0.05
CLOCK = "ctrlps4_equiv"
FH = struct.Struct("<8sIIII")
RH = struct.Struct("<6If9f12f3dd")
CONFIGS = {"A": dict(swept_clear_s=300.0, front_seed_m=None, front_seed_deg=35.0),
           "B": dict(swept_clear_s=300.0, front_seed_m=0.65, front_seed_deg=35.0)}
KV_ARMS = {"A": CONFIGS["A"], "B": CONFIGS["B"],
           "P_deg": dict(CONFIGS["B"], front_seed_deg=45.0),
           "P_swept": dict(CONFIGS["A"], swept_clear_s=0.0),
           "body_only": dict(swept_clear_s=0.0, front_seed_m=None, front_seed_deg=35.0)}
REF_OF = {"A": "A", "B": "B", "P_deg": "B", "P_swept": "A", "body_only": "A"}


def pack_frames(path):
    with open(path, "rb") as f:
        magic, ver, n, _a, _b = FH.unpack(f.read(FH.size))
        assert magic == b"FPGAGPK01" and ver == 1
        for _ in range(n):
            h = RH.unpack(f.read(RH.size))
            w, hh = h[0], h[1]
            raw = np.frombuffer(f.read(2 * w * hh), "<u2").reshape(hh, w)
            yield raw, float(h[6])


def layer_kw(cfg):
    return dict(dims=(80, 80, 32), voxel_m=V, band=rnl.HeightBand(ground_z_m=0.0, robot_height_m=0.31),
                clock_id=CLOCK, max_map_age_ns=1 << 62, body_radius_m=0.30, body_margin_m=0.05,
                front_seed_m=cfg["front_seed_m"], front_seed_deg=cfg["front_seed_deg"],
                swept_clear_s=cfg["swept_clear_s"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--frames", type=int, default=0)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    meta = json.loads(Path(META).read_text())
    K = np.asarray(meta["K"], float)
    z = np.load(POSES)
    camera, body, stamps = z["camera"], z["body"], z["stamps"]
    n = len(stamps) if not a.frames else min(a.frames, len(stamps))

    tmp = Path(tempfile.mkdtemp(prefix="ctrlps4_equiv_"))
    core = CpuRollingCore(compile_rolling(tmp / "roll.so"), grid_library=str(tmp / "grid.so"))
    svc = RollingService(core)
    fusion = RollingFusion(RemotePlRollingBackend(None, link=InProcLink(svc)), voxel_m=V)
    band = rnl.HeightBand(ground_z_m=0.0, robot_height_m=0.31)
    assert fusion.enable_grid(band, skip_changes=False)
    pipe = M7Pipeline(FrameNativeM7(compile_frame_native(tmp / "frame.so")), GridSpec((0.0, 0.0, 0.0), V, (32, 32, 10)),
                      grid_policy=FollowCameraGrid(V, (32, 32, 10), back_cells=1, height_cell=8))
    lib = load_library()
    n1 = {c: PlGridLayer(**layer_kw(CONFIGS[c])) for c in CONFIGS}
    n0 = {c: rnl.RollingLayerBridge(**layer_kw(CONFIGS[c])) for c in CONFIGS}
    kv = {k: make_core("soft", svc, lib=lib, grid_params=p) for k, p in KV_ARMS.items()}
    infl = Inflation(lib=lib)

    counts, diff_frames, diff_cells = Counter(), Counter(), Counter()
    kv_ms = {k: [] for k in KV_ARMS}
    first_diff = {}
    t_wall = time.time()
    rows = open(out / "frames.jsonl", "w")
    for k, (raw, unit) in enumerate(pack_frames(PACK)):
        if k >= n:
            break
        t_ns = 1_000_000_000 + int(round((stamps[k] - stamps[0]) * 1e6)) * 1000   # relative, us lattice
        depth = raw.astype(np.float32) * np.float32(unit)
        depth[raw == 0] = np.nan
        ident = CaptureIdentity("car1015", k + 1, t_ns, CLOCK, "cal", "cam", "e1")
        frame = DepthFrame(ident, depth, K, PoseAtCapture(t_ns, CLOCK, "cam", "e1", True, camera[k]))
        try:
            nav = pipe.ingest_geometry(frame, now_ns=t_ns + 1000, now_clock_id=CLOCK).navigation
        except ValueError as exc:      # the producer's "input_refused" (geo_producer: frame dropped, chain goes on)
            counts["input_refused:%s" % str(exc)[:40]] += 1
            continue
        if not nav.valid:
            counts["nav_invalid"] += 1
            continue
        bx, by = float(body[k, 0, 3]), float(body[k, 1, 3])
        yaw = math.atan2(body[k, 1, 0], body[k, 0, 0])
        r = fusion.ingest(nav, pipe, now_ns=t_ns + 2000, clock_id=CLOCK, robot_position_m=tuple(body[k, :3, 3]))
        if not r.accepted or r.grid is None:
            counts["fusion_not_accepted"] += 1
            continue
        d = r.delta
        now = t_ns + 5_000_000                        # the window's time, the same in every arm
        assert int(round((now / 1e9) * 1e9)) == now   # the KV's float seconds carry it exactly
        ref = {}
        for c in CONFIGS:
            ok, why = n1[c].on_planes(epoch=d.epoch, delta_seq=d.delta_seq, window_base=d.window_base,
                                      acquired_ns=d.acquired_ns, clock_id=d.clock_id, planes=r.grid)
            assert ok, why
            g1 = n1[c].grid(body_xy=(bx, by), now_ns=now, body_yaw=yaw)
            wire = rnl.decode_delta(rnl.encode_delta(d))
            ok0, why0, req = n0[c].on_delta(wire, now_ns=now)
            if req is not None:
                snap = fusion.snapshot(now_ns=now, clock_id=d.clock_id)
                w = rnl.decode_snapshot(rnl.encode_snapshot(snap, acquired_ns=d.acquired_ns, clock_id=d.clock_id,
                                                            voxel_m=V))
                ok0, why0, _ = n0[c].on_snapshot(w, now_ns=now)
                counts["n0_resync:%s" % c] += 1
            g0 = n0[c].grid(body_xy=(bx, by), now_ns=now, body_yaw=yaw) if n0[c].strict_grid() is not None else None
            ref[c] = np.asarray(g1.data, np.int8).reshape(80, 80)
            if g0 is None:
                counts["n0_no_grid:%s" % c] += 1
            else:
                nd = int((np.asarray(g0.data, np.int8).reshape(80, 80) != ref[c]).sum())
                diff_cells["N0_vs_N1:%s" % c] += nd
                diff_frames["N0_vs_N1:%s" % c] += nd > 0
        row = {"frame": k, "base": list(d.window_base[:2]), "pose": [round(bx, 3), round(by, 3), round(yaw, 3)]}
        for name, core_k in kv.items():
            core_k.on_odom({"odom_pose": (bx, by, yaw), "v": 0.0, "w": 0.0, "map_odom": (0.0, 0.0, 0.0),
                            "permit": True, "teleop": False})
            t0 = time.perf_counter()
            core_k.step(now / 1e9)
            kv_ms[name].append((time.perf_counter() - t0) * 1e3)
            src = core_k.grid
            win = src.last_window
            assert src._seen is not None and src._seen[0] is svc.nav_planes, "KV window not from this frame"
            assert tuple(src._seen[1][1:3]) == tuple(d.window_base[:2])
            nd = int((win != ref[REF_OF[name]]).sum())
            row[name] = nd
            diff_cells["KV_%s_vs_N1" % name] += nd
            diff_frames["KV_%s_vs_N1" % name] += nd > 0
            if nd and name not in first_diff:
                first_diff[name] = k
            if name in CONFIGS:
                nc = int((src._seen[1][0] != infl.costs(ref[name])).sum())
                diff_cells["KV_%s_cost_vs_N1" % name] += nc
                diff_frames["KV_%s_cost_vs_N1" % name] += nc > 0
                row[name + "_cost"] = nc
        counts["compared"] += 1
        rows.write(json.dumps(row) + "\n")
        if (k + 1) % 200 == 0:
            print("frame", k + 1, dict(diff_frames), flush=True)
    rows.close()
    feat = {}
    for name, core_k in kv.items():
        feat[name] = core_k.grid.win.describe()
    summary = {
        "input": {"pack": PACK, "poses": POSES, "frames_read": n},
        "counts": dict(counts), "diff_frames": dict(diff_frames), "diff_cells": dict(diff_cells),
        "first_diff_frame": first_diff, "kv_window_features": feat,
        "nano_features": {c: {"swept_cleared_total_N1": int(n1[c].counts["swept_cleared_cells"]),
                              "swept_cleared_total_N0": int(n0[c].counts["swept_cleared_cells"]),
                              "swept_evidence_N1": int(n1[c].counts["swept_evidence_cells"]),
                              "swept_evidence_N0": int(n0[c].counts["swept_evidence_cells"]),
                              "front_seeded_N1": int(n1[c].counts["front_seeded"])} for c in CONFIGS},
        "kv_step_ms_median": {k: round(float(np.median(v)), 3) for k, v in kv_ms.items() if v},
        "kv_window_ms_last": {k: c.grid.last_window_ms for k, c in kv.items()},
        "wall_s": round(time.time() - t_wall, 1)}
    c = summary["counts"].get("compared", 0)
    df = summary["diff_frames"]
    summary["verdict"] = {
        "KV_A_equals_N1": c > 0 and df.get("KV_A_vs_N1", 0) == 0,
        "KV_B_equals_N1": c > 0 and df.get("KV_B_vs_N1", 0) == 0,
        "KV_costs_equal": df.get("KV_A_cost_vs_N1", 0) == 0 and df.get("KV_B_cost_vs_N1", 0) == 0,
        "N0_equals_N1": df.get("N0_vs_N1:A", 0) == 0 and df.get("N0_vs_N1:B", 0) == 0,
        "positive_deg_caught": df.get("KV_P_deg_vs_N1", 0) > 0,
        "positive_swept_caught": df.get("KV_P_swept_vs_N1", 0) > 0}
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary["verdict"]), json.dumps(summary["diff_frames"]))


if __name__ == "__main__":
    main()
