#!/usr/bin/env python3
"""Arm A (repo CPU reference) or arm C (PL backend) over a prepared recording.

Both arms go through ``host.m7_backend.M7BackendSelector.frame_backend`` and
call ``frame_from_depth`` exactly as ``M7Pipeline`` does, with the fixed-camera
identity pose and the product grid of ``tools/nano_loop/chain.py``:

  --arm a                       backend "cpu"  (FrameNativeM7, g++ -O2 of hw/m7/frame_depth.cpp)
  --arm c --device mock         backend "pl"   with MockPlDevice (host stand-in, NOT performance)
  --arm c --device devmem       backend "pl"   with DevMemPlDevice.open() (KV260 board; DMA buffer
                                provider udmabuf -> dmaheap, or FPGA_M7_DMA_PROVIDER etc., see
                                docs/design/PL_DMA_PROVIDERS_20260927.md)

No fallback: if the PL device cannot be opened the run fails.

Per frame (CSV):
  convert_ms  uint16 mm -> float32 m (NaN = no return)          [data preparation]
  core_ms     the core call ``fpga_m7_depth_frame``: CPU = the C function only;
              PL = register writes + DMA buffer copies + run + read-back
  device_ms   PL only: ap_start -> ap_done polling loop
  call_ms     whole ``frame_from_depth`` (input checks, inverse K, buffers, core, output checks)
  e2e_ms      convert_ms + call_ms
The first measured pass is dumped (counts + flags per frame) for the comparator.
The depth unit comes from meta.json ``depth_unit_m`` (Gemini 2: 0.001, TUM: 0.0002);
a meta without it is refused unless --depth-unit-m is given.
--convert native (E7, 2026-11-02; default python = as before): convert_ms is the C++ pass of
host/kv_fast_native.py (kv_server --kv-fast native: u16 -> float32 with NEON + depth crc32 in the same
pass) instead of depth_u16_to_m; the float32 depth is the same bytes (checked on the first frame of every
pass against depth_u16_to_m, the run stops otherwise). The other columns are unchanged.
Usage: py_arm.py --prep <dir> --arm a|c [--device mock|devmem] --out <fresh dir>
                 [--passes 3] [--warmup 1] [--pace-hz 15] [--depth-unit-m U] [--convert python|native]
"""
import argparse
import csv
import json
import os
import resource
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.geo_same_input.common import (depth_u16_to_m, f32_hex, inverse_k_f32,  # noqa: E402
                                         load_prepared, meta_depth_unit_m, record_bytes)


class _TimedCore:
    """Proxy for backend.lib that times the core call and forwards everything else."""

    def __init__(self, lib):
        self._lib = lib
        self.last_ms = 0.0

    def fpga_m7_depth_frame(self, *args):
        t = time.perf_counter()
        self._lib.fpga_m7_depth_frame(*args)
        self.last_ms = (time.perf_counter() - t) * 1e3

    def __getattr__(self, name):
        return getattr(self._lib, name)


def build_backend(arm, device, build_dir):
    from host.m7_backend import M7BackendSelector
    from host.m7_frame_native import compile_frame_native
    lib = compile_frame_native(os.path.join(build_dir, "m7_frame.so"))
    if arm == "a":
        sel = M7BackendSelector(cpu_frame_library=lib)
        backend = sel.frame_backend("cpu")
    else:
        from host.m7_pl_device import DevMemPlDevice, MockPlDevice
        if device == "mock":
            factory = lambda: MockPlDevice(frame_library=lib)  # noqa: E731
        elif device == "devmem":
            factory = DevMemPlDevice.open
        else:
            raise ValueError("--device mock|devmem for arm c")
        sel = M7BackendSelector(pl_device_factory=factory)
        backend = sel.frame_backend("pl")
    device_ms = [0.0]
    if arm == "c":
        run = backend.lib.run

        def timed_run():
            t = time.perf_counter()
            try:
                run()
            finally:
                device_ms[0] = (time.perf_counter() - t) * 1e3
        backend.lib.run = timed_run
    timed = _TimedCore(backend.lib)
    backend.lib = timed
    return sel, backend, timed, device_ms


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--prep", required=True)
    ap.add_argument("--arm", choices=("a", "c"), required=True)
    ap.add_argument("--device", default=None)
    ap.add_argument("--out", required=True)
    ap.add_argument("--passes", type=int, default=3)
    ap.add_argument("--warmup", type=int, default=1)
    ap.add_argument("--pace-hz", type=float, default=0.0)
    ap.add_argument("--tag", default=None)
    ap.add_argument("--convert", choices=("python", "native"), default="python")
    ap.add_argument("--depth-unit-m", type=float, default=None,
                    help="only for a prepared input whose meta.json lacks depth_unit_m")
    a = ap.parse_args()
    meta, depth = load_prepared(a.prep)
    unit_m, unit_source = meta_depth_unit_m(meta, a.depth_unit_m)
    os.makedirs(a.out, exist_ok=False)
    n, h, w = depth.shape
    K = np.asarray(meta["K"], dtype=np.float64)
    ik_hex = f32_hex(inverse_k_f32(K))
    grid = meta["grid"]
    origin = tuple(float(v) for v in grid["origin_m"])
    shape = tuple(int(v) for v in grid["shape_xyz"])
    sel, backend, timed, device_ms = build_backend(a.arm, a.device, a.out)
    fast = None
    if a.convert == "native":
        from host.kv_fast_native import KvFastNative
        fast = KvFastNative.build(a.out)
        unit_bits = int(np.array([unit_m], dtype="<f4").view("<u4")[0])
        fast_buf = np.empty(h * w, dtype=np.float32)

    def convert(codes):
        if fast is None:
            return depth_u16_to_m(codes, unit_m)
        flat = np.ascontiguousarray(codes).reshape(-1)
        if fast.convert_crc(flat, unit_bits, fast_buf) is None:
            raise SystemExit("--convert native: native path refused this frame (%s)" % dict(fast.counts))
        return fast_buf.reshape(h, w)
    pl_device = None
    if a.arm == "c":
        pl_device = backend.device.describe()
        print("pl_device: %s" % json.dumps(pl_device, sort_keys=True), file=sys.stderr, flush=True)
    tag = a.tag or os.path.basename(os.path.normpath(a.out))
    rows = []
    dump = open(os.path.join(a.out, "flags.dump"), "wb")
    late = issued = 0
    period = 1.0 / a.pace_hz if a.pace_hz > 0 else 0.0
    tick = time.perf_counter()
    ru0, wall0 = resource.getrusage(resource.RUSAGE_SELF), time.perf_counter()
    for p in range(a.warmup + a.passes):
        if p == a.warmup:
            ru0, wall0 = resource.getrusage(resource.RUSAGE_SELF), time.perf_counter()
        for f in range(n):
            if period:
                tick += period
                now = time.perf_counter()
                if now > tick:
                    late += 1
                else:
                    time.sleep(tick - now)
                issued += 1
            t0 = time.perf_counter()
            depth_m = convert(depth[f])
            t1 = time.perf_counter()
            if fast is not None and f == 0 and not np.array_equal(
                    depth_m.view(np.uint32), depth_u16_to_m(depth[f], unit_m).view(np.uint32)):
                raise SystemExit("--convert native: float32 depth differs from depth_u16_to_m (frame 0)")
            ev = backend.frame_from_depth(
                depth_m=depth_m, k=K, camera_to_world=np.eye(4), grid_origin_m=origin,
                voxel_size_m=float(grid["voxel_m"]), shape=shape, frame_id=int(meta["color_index"][f]),
                capture_time_ns=int(meta["acquired_ns"][f]), pose_time_ns=int(meta["acquired_ns"][f]),
                calibration_version=meta["calibration_id"], pose_confirmed=True)
            t2 = time.perf_counter()
            if p >= a.warmup:
                # outside = clipped rays since M7_CLIP_SPEC_20261011 (key kept)
                counts = (ev.accepted_rays, ev.invalid_depth_or_projection, ev.outside_grid)
                if p == a.warmup:
                    dump.write(record_bytes(counts, ev.flags))
                rows.append({"pass": p - a.warmup, "frame": f,
                             "convert_ms": round((t1 - t0) * 1e3, 4),
                             "core_ms": round(timed.last_ms, 4),
                             "device_ms": round(device_ms[0], 4) if a.arm == "c" else "",
                             "call_ms": round((t2 - t1) * 1e3, 4),
                             "e2e_ms": round((t2 - t0) * 1e3, 4),
                             "accepted": counts[0], "invalid": counts[1], "outside": counts[2],
                             "free_voxels": int((ev.flags == 1).sum()), "hit_voxels": int((ev.flags == 2).sum())})
    ru1, wall1 = resource.getrusage(resource.RUSAGE_SELF), time.perf_counter()
    dump.close()
    with open(os.path.join(a.out, "frames.csv"), "w", newline="") as fp:
        wr = csv.DictWriter(fp, fieldnames=list(rows[0]))
        wr.writeheader()
        wr.writerows(rows)
    summary = {"tag": tag, "arm": a.arm, "device": a.device, "backend_name": getattr(backend, "backend_name", None),
               "selector_counts": dict(sel.counts), "backend_counts": dict(getattr(backend, "counts", {})),
               "pl_device": backend.device.describe() if pl_device is not None else None,
               "frames": n, "passes": a.passes, "warmup": a.warmup, "pace_hz": a.pace_hz,
               "paced_issued": issued, "paced_late_starts": late,
               "measured_wall_s": round(wall1 - wall0, 4),
               "measured_cpu_user_s": round(ru1.ru_utime - ru0.ru_utime, 4),
               "measured_cpu_sys_s": round(ru1.ru_stime - ru0.ru_stime, 4),
               "maxrss_kb": ru1.ru_maxrss, "inverse_k_f32_hex": ik_hex,
               "inverse_k_matches_prep": ik_hex == meta["inverse_k_f32_hex"],
               "depth_unit_m": unit_m, "depth_unit_source": unit_source,
               "convert": a.convert, "kv_fast": fast.describe() if fast is not None else None,
               "depth_sha256": meta["depth_sha256"], "numpy": np.__version__, "python": sys.version.split()[0]}
    json.dump(summary, open(os.path.join(a.out, "summary.json"), "w"), indent=1)
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
