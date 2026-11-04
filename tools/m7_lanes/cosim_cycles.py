#!/usr/bin/env python3
"""Per-frame cycle counts from a Vitis HLS C/RTL co-simulation log of the A2 frame
top (tb_frame_stream.cpp), compared with the A2 cycle model (lanes_model.cpp).

Measured: the cosim log prints one line per finished transaction,
  // RTL Simulation : k / n [..] @ "<time in ps>"
so transaction k took (t_k - t_{k-1}) / clock_ps cycles (s_axilite programming
by the testbench included, a few hundred cycles). Transaction order = the
testbench case order: dense40x30, smallgrid, baddims, badvoxel, empty0x0, then
the frames of the FPGA_M7_TB_FRAMES file in file order.

Fixed cost: the empty0x0 transaction of the SAME run (clear 32768 + write-out
32768 + DATAFLOW start/stop + register access) is taken as the per-frame fixed
cost; a frame's walk-region estimate is measured - fixed.

Model: tools/m7_lanes/lanes_model.cpp simulate() for the same frame (read from
the same frames file), --lanes N, policy rr, lane FIFO depth 4, s_ray depth 512,
1 pixel / cycle, 1 step / cycle, no per-ray overhead. Its walk_cycles excludes
the fixed costs. All model numbers are MODEL numbers, the cosim numbers are RTL
simulation measurements (not board measurements).
Usage: cosim_cycles.py --log fpga_m7_frame_axi.log --frames tb.bin --lanes 8
         [--clock-ns 10] [--names a,b,c] [--build DIR] [--out report.json]
"""
import argparse
import json
import os
import re
import struct
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.m7_lanes import lanes  # noqa: E402

SYN_CASES = ["dense40x30", "smallgrid", "baddims", "badvoxel", "empty0x0"]


def read_frames(path):
    out = []
    with open(path, "rb") as fp:
        while True:
            hdr = fp.read(20)
            if len(hdr) < 20:
                break
            w, h, nx, ny, nz = struct.unpack("<5I", hdr)
            ik = np.frombuffer(fp.read(36), dtype="<f4").reshape(3, 3).copy()
            wc = np.frombuffer(fp.read(48), dtype="<f4").reshape(3, 4).copy()
            org = struct.unpack("<3d", fp.read(24))
            vox = struct.unpack("<d", fp.read(8))[0]
            d = np.frombuffer(fp.read(4 * w * h), dtype="<f4").reshape(h, w).copy()
            out.append(dict(depth=d, ik=ik, wc=wc, origin=org, voxel=vox, dims=(nx, ny, nz)))
    return out


def transactions(log, clock_ps):
    stamps = {}
    for line in open(log, errors="replace"):
        m = re.search(r'// RTL Simulation : (\d+) / (\d+) \[[^\]]*\] @ "(\d+)"', line)
        if m:
            stamps[int(m.group(1))] = (int(m.group(3)), int(m.group(2)))
    if not stamps:
        raise SystemExit("no RTL Simulation transaction lines in %s" % log)
    n = max(v[1] for v in stamps.values())
    cyc = []
    for k in range(1, n + 1):
        if k in stamps and k - 1 in stamps:
            cyc.append((stamps[k][0] - stamps[k - 1][0]) / clock_ps)
        else:
            cyc.append(None)
    return cyc, n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True)
    ap.add_argument("--frames", required=True)
    ap.add_argument("--lanes", type=int, required=True)
    ap.add_argument("--clock-ns", type=float, default=10.0)
    ap.add_argument("--names", default="")
    ap.add_argument("--build", default=os.path.join(lanes.SCRATCH, "a2_impl", "model_build"))
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    cyc, n = transactions(a.log, a.clock_ns * 1000.0)
    frames = read_frames(a.frames)
    names = a.names.split(",") if a.names else ["file%d" % i for i in range(len(frames))]
    if n != len(SYN_CASES) + len(frames):
        raise SystemExit("log has %d transactions, expected %d" % (n, len(SYN_CASES) + len(frames)))
    fixed = cyc[SYN_CASES.index("empty0x0")]
    L = lanes.Lanes(lanes.build(a.build))
    rows = []
    for i, f in enumerate(frames):
        meas = cyc[len(SYN_CASES) + i]
        with L.prepare(f["depth"], f["ik"], f["wc"], f["origin"], f["voxel"], f["dims"]) as p:
            _, counts, st = p.run(lanes=a.lanes, policy="rr", fifo=4, sray=512)
        h, w = f["depth"].shape
        walk_meas = meas - fixed
        rows.append(dict(name=names[i], wh=[w, h], pixels=w * h, rays=st["rays"], steps=st["steps"],
                         counts=list(counts),
                         cosim_cycles=meas, fixed_cycles_empty_frame=fixed,
                         cosim_minus_fixed=walk_meas,
                         model_walk_cycles=st["walk_cycles"], model_lower_bound=st["lower_bound"],
                         model_conv_done=st["conv_done"], model_disp_stall=st["disp_stall"],
                         model_plus_fixed=st["walk_cycles"] + fixed,
                         deviation_walk_pct=round(100.0 * (walk_meas - st["walk_cycles"]) / st["walk_cycles"], 2),
                         deviation_total_pct=round(100.0 * (meas - st["walk_cycles"] - fixed) /
                                                   (st["walk_cycles"] + fixed), 2),
                         ms_at_100MHz=round(meas / 1e5, 3)))
    rep = dict(log=os.path.abspath(a.log), frames_file=os.path.abspath(a.frames), lanes=a.lanes,
               clock_ns=a.clock_ns, synthetic_case_cycles=dict(zip(SYN_CASES, cyc[:len(SYN_CASES)])),
               frames=rows,
               note="cosim = RTL simulation (measured in xsim, not on the board); model = lanes_model.cpp rr D=4")
    txt = json.dumps(rep, indent=1)
    if a.out:
        open(a.out, "w").write(txt)
    print(txt)


if __name__ == "__main__":
    main()
