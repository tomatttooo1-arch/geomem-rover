#!/usr/bin/env python3
"""GMAP2: dump the per-frame inputs and outputs of PlGridLayer (config A = car default: body 0.30 + 0.05 m, swept 300 s,
front seed off) on car1015, from the same chain as tools/kv_nav/sim/window_equiv.py (CTRLPS4, ctrlps6 c506e6b), N1 arm.
Run with PYTHONPATH = a checkout of c506e6b (this file only uses that tree's modules, never writes into it):
  PYTHONPATH=<c506e6b tree> python3 dump_window_frames.py OUT.npz [--frames N]
Per accepted frame: window base (bx, by), aux plane (u8 6400), body (x, y) odom m, now ns, the float PlGridLayer output
(int8 6400) and its per-frame swept counters. The adoption gate (gate_window.py) replays these through the integer
reference and compares.
"""
import argparse
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

from host import rolling_nav_layer as rnl
from host.kv_rolling import CpuRollingCore, RemotePlRollingBackend, RollingService
from host.m7_frame_native import FrameNativeM7, compile_frame_native
from host.m7_grid_policy import FollowCameraGrid
from host.m7_pipeline import CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, PoseAtCapture
from host.m7_rolling import RollingFusion, compile_rolling
from host.pl_grid_layer import PlGridLayer
from tools.kv_nav.sim.cell_offset_probe import InProcLink
from tools.kv_nav.sim import window_equiv as we


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--frames", type=int, default=0)
    a = ap.parse_args()
    import json
    meta = json.loads(Path(we.META).read_text())
    K = np.asarray(meta["K"], float)
    z = np.load(we.POSES)
    camera, body, stamps = z["camera"], z["body"], z["stamps"]
    n = len(stamps) if not a.frames else min(a.frames, len(stamps))
    tmp = Path(tempfile.mkdtemp(prefix="gmap2_dump_"))
    core = CpuRollingCore(compile_rolling(tmp / "roll.so"), grid_library=str(tmp / "grid.so"))
    svc = RollingService(core)
    fusion = RollingFusion(RemotePlRollingBackend(None, link=InProcLink(svc)), voxel_m=we.V)
    band = rnl.HeightBand(ground_z_m=0.0, robot_height_m=0.31)
    assert fusion.enable_grid(band, skip_changes=False)
    pipe = M7Pipeline(FrameNativeM7(compile_frame_native(tmp / "frame.so")), GridSpec((0.0, 0.0, 0.0), we.V, (32, 32, 10)),
                      grid_policy=FollowCameraGrid(we.V, (32, 32, 10), back_cells=1, height_cell=8))
    layer = PlGridLayer(**we.layer_kw(we.CONFIGS["A"]))
    rec = {k: [] for k in ("base", "aux", "body", "now", "out", "cleared", "evidence")}
    for k, (raw, unit) in enumerate(we.pack_frames(we.PACK)):
        if k >= n:
            break
        t_ns = 1_000_000_000 + int(round((stamps[k] - stamps[0]) * 1e6)) * 1000
        depth = raw.astype(np.float32) * np.float32(unit)
        depth[raw == 0] = np.nan
        ident = CaptureIdentity("car1015", k + 1, t_ns, we.CLOCK, "cal", "cam", "e1")
        frame = DepthFrame(ident, depth, K, PoseAtCapture(t_ns, we.CLOCK, "cam", "e1", True, camera[k]))
        try:
            nav = pipe.ingest_geometry(frame, now_ns=t_ns + 1000, now_clock_id=we.CLOCK).navigation
        except ValueError:
            continue
        if not nav.valid:
            continue
        bx, by = float(body[k, 0, 3]), float(body[k, 1, 3])
        yaw = math.atan2(body[k, 1, 0], body[k, 0, 0])
        r = fusion.ingest(nav, pipe, now_ns=t_ns + 2000, clock_id=we.CLOCK, robot_position_m=tuple(body[k, :3, 3]))
        if not r.accepted or r.grid is None:
            continue
        d = r.delta
        now = t_ns + 5_000_000
        ok, why = layer.on_planes(epoch=d.epoch, delta_seq=d.delta_seq, window_base=d.window_base,
                                  acquired_ns=d.acquired_ns, clock_id=d.clock_id, planes=r.grid)
        assert ok, why
        ev0 = layer.counts["swept_evidence_cells"]
        g1 = layer.grid(body_xy=(bx, by), now_ns=now, body_yaw=yaw)
        rec["base"].append(tuple(int(v) for v in d.window_base[:2]))
        rec["aux"].append(np.asarray(r.grid.aux, np.uint8).copy())
        rec["body"].append((bx, by))
        rec["now"].append(now)
        rec["out"].append(np.asarray(g1.data, np.int8).copy())
        rec["cleared"].append(int(layer.last_swept_cleared))
        rec["evidence"].append(int(layer.counts["swept_evidence_cells"] - ev0))
        if len(rec["now"]) % 200 == 0:
            print("frames", len(rec["now"]), file=sys.stderr)
    np.savez_compressed(a.out, base=np.array(rec["base"], np.int64), aux=np.array(rec["aux"]),
                        body=np.array(rec["body"], np.float64), now=np.array(rec["now"], np.int64),
                        out=np.array(rec["out"]), cleared=np.array(rec["cleared"], np.int64),
                        evidence=np.array(rec["evidence"], np.int64))
    print("frames %d cleared_total %d evidence_total %d" % (len(rec["now"]), sum(rec["cleared"]), sum(rec["evidence"])))


if __name__ == "__main__":
    main()
