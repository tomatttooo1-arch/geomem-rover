#!/usr/bin/env python3
"""GEO_FX3 (2026-11-04) board check of the fixed-point frame core: real PL vs its model on the A53.

Runs frames of a prepared recording (depth_u16.bin + meta.json, the nano_replay format; optional
meta "camera_to_world") through the product driver (host.m7_backend.PlFrameBackend, codes call) and
compares every frame with host.m7_frame_fx.FrameFx run on the same CPU from the same codes: flags bytes,
the three counts, the four statistics registers (ST_NPOS / ST_LE20 / ST_MM0 / ST_MM1) and DEPTH_CRC.
FrameFx is the bit-exact model of the core (GEO_FX3: 880 frames vs the golden FX arm, 0 differences;
C/RTL co-simulation PASS), so on the board any difference is a PL / driver / bitstream fault.

Refuses to compare (exit 2) when the loaded frame core is not the fixed-point core (fx_state "absent":
the driver would run the CPU reference and the check would compare the model with itself).
Positive control (default on): one more frame with two FX_K words exchanged by the driver must differ.

  PYTHONPATH=. python3 tools/kv_link/geofx3_board_check.py --prep DIR --device devmem [--frames 40]
  (host: --device mock)
Exit 0: every frame equal and the control caught. Prints one JSON summary line (also --out FILE).
"""

import argparse
import json
import os
import sys
import tempfile
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from host.m7_frame_fx import compile_frame_fx, f32_bits  # noqa: E402
from host.m7_frame_native import compile_frame_native  # noqa: E402
from tools.geo_same_input.common import load_prepared, meta_depth_unit_m  # noqa: E402


def geometry(k, t_cw):
    ik = np.ascontiguousarray(np.linalg.inv(np.asarray(k, dtype=np.float64)), dtype=np.float32)
    wc = np.ascontiguousarray(np.asarray(t_cw, dtype=np.float64)[:3, :], dtype=np.float32)
    return ik, wc


def run_pl(be, codes, unit_bits, ik, wc, origin, voxel, dims):
    h, w = codes.shape
    flags = np.full(32 ** 3, 99, np.uint8)
    counts = np.full(3, 99, np.uint32)
    status = np.full(1, 99, np.uint32)
    crc = be.lib.fpga_m7_depth_frame_codes(np.ascontiguousarray(codes, dtype=np.uint16).reshape(-1), unit_bits,
                                          ik.ravel(), wc.ravel(), origin, voxel, w, h, dims[0], dims[1], dims[2],
                                          flags, counts, status)
    return flags, [int(x) for x in counts], be.lib.last_stats, int(crc), int(status[0])


def run_model(fx, codes, unit_bits, ik, wc, origin, voxel, dims):
    h, w = codes.shape
    p = fx.prepare(w, h, dims[0], dims[1], dims[2], unit_bits, ik, wc, origin, voxel)
    if p["status"] != 0:
        return None, p["status"]
    st, flags, counts, stats, crc = fx.frame(codes, w, h, dims[0], dims[1], dims[2], p["words"], p["origin"],
                                             p["codes"], p["c20"], 0, unit_bits)
    return (flags, [int(x) for x in counts], [int(x) for x in stats], int(crc), int(st)), 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prep", required=True)
    ap.add_argument("--device", choices=("devmem", "mock"), default="devmem")
    ap.add_argument("--frames", type=int, default=40, help="frames checked, spread evenly over the recording")
    ap.add_argument("--no-control", action="store_true")
    ap.add_argument("--out")
    a = ap.parse_args()

    from host.m7_backend import PlFrameBackend
    from host.m7_frame_fx import FrameFx
    from host.m7_pl_device import DevMemPlDevice, MockPlDevice

    meta, depth = load_prepared(a.prep)
    unit, _ = meta_depth_unit_m(meta)
    unit_bits = f32_bits(unit)
    k = np.asarray(meta["K"], dtype=np.float64)
    t_cw = np.asarray(meta.get("camera_to_world", np.eye(4)), dtype=np.float64)
    origin = np.ascontiguousarray(meta["grid"]["origin_m"], dtype=np.float64)
    voxel = float(meta["grid"]["voxel_m"])
    dims = [int(x) for x in meta["grid"]["shape_xyz"]]
    ik, wc = geometry(k, t_cw)

    d = tempfile.mkdtemp(prefix="geofx3_board_")
    cpu_lib = compile_frame_native(os.path.join(d, "m7_frame.so"))
    fx_lib = compile_frame_fx(os.path.join(d, "m7_frame_fx.so"))
    fx = FrameFx(fx_lib)
    dev = (DevMemPlDevice.open() if a.device == "devmem" else
           MockPlDevice(frame_library=cpu_lib, frame_fx_library=fx_lib))
    be = PlFrameBackend(dev, cpu_library=cpu_lib, fx_library=fx_lib)
    summary = {"prep": os.path.abspath(a.prep), "device": a.device, "fx_state": be.fx_state,
               "unit_m": unit, "dims": dims, "fx_library": str(fx_lib)}
    if be.fx_state != "on":
        summary["error"] = "the loaded frame core is not the fixed-point core (FX_CODES probe failed)"
        print(json.dumps(summary), flush=True)
        return 2

    n = depth.shape[0]
    picks = sorted(set(int(round(x)) for x in np.linspace(0, n - 1, min(a.frames, n))))
    rows, t0 = [], time.monotonic()
    for i in picks:
        codes = depth[i]
        want, ps = run_model(fx, codes, unit_bits, ik, wc, origin, voxel, dims)
        if want is None:
            rows.append({"frame": i, "ps_status": ps, "skipped": "PS status != 0 (CPU reference frame)"})
            continue
        got = run_pl(be, codes, unit_bits, ik, wc, origin, voxel, dims)
        rows.append({"frame": i, "flag_bytes_diff": int(np.count_nonzero(got[0] != want[0])),
                     "counts_equal": got[1] == want[1], "stats_equal": got[2] == want[2],
                     "crc_equal": got[3] == want[3], "status": got[4], "accepted": want[1][0]})
    checked = [r for r in rows if "skipped" not in r]
    equal = [r for r in checked if r["flag_bytes_diff"] == 0 and r["counts_equal"] and r["stats_equal"] and
             r["crc_equal"] and r["status"] == 0]
    summary.update(frames_checked=len(checked), frames_equal=len(equal), frames_skipped=len(rows) - len(checked),
                   seconds=round(time.monotonic() - t0, 2), backend_counts=dict(be.counts),
                   first_differences=[r for r in checked if r not in equal][:5])
    control_ok = True
    if not a.no_control and checked:
        i = checked[0]["frame"]
        want, _ = run_model(fx, depth[i], unit_bits, ik, wc, origin, voxel, dims)
        prepare = be.lib.fx.prepare

        def swapped(*args, **kw):
            p = prepare(*args, **kw)
            p["words"] = p["words"].copy()
            p["words"][[12, 14]] = p["words"][[14, 12]]          # G0 x low <-> G0 y low
            return p
        be.lib.fx.prepare = swapped
        try:
            got = run_pl(be, depth[i], unit_bits, ik, wc, origin, voxel, dims)
        finally:
            be.lib.fx.prepare = prepare
        control_ok = int(np.count_nonzero(got[0] != want[0])) > 0 or got[1] != want[1]
        summary["control"] = {"frame": i, "caught": control_ok}
    summary["pass"] = bool(checked) and len(equal) == len(checked) and control_ok
    line = json.dumps(summary)
    print(line, flush=True)
    if a.out:
        with open(a.out, "w") as fp:
            fp.write(line + "\n")
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
