"""GEO_FX3 (2026-11-04) test helpers for the fixed-point frame core.

Since GEO_FX3 the PL frame core reads 16-bit depth codes only; its expectation is no longer the float CPU
reference but ``host.m7_frame_fx.FrameFx`` (PS constants + bit-exact model of the core, verified against the
GEO_FX0 golden FX arm on rec30 / car570, 880 frames, 0 differences: GEO_FX3/logs/ps_check_*.err).

  fx()            the FrameFx library (compiled once per process, -ffp-contract=off)
  m_to_codes()    float32 metres of the old float cases -> uint16 codes (NaN / <= 0 -> 0)
  expect()        FrameFx expectation of a codes frame (None when the PS status is not 0)
  call_codes()    the PL codes call of a PlFrameBackend (host.m7_backend._PlFrameCall)
Both return ``Out`` (flags shaped (nz, ny, nx) like FrameEvidence, counts, stats, crc, status).
"""

import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from host.m7_frame_fx import FrameFx, compile_frame_fx, f32_bits

UNIT_MM_BITS = f32_bits(0.001)          # 0x3a83126f, the Gemini 2 millimetre unit
GRID_CAPACITY = 32 ** 3
_FX = {}


def fx():
    if "fx" not in _FX:
        _FX["tmp"] = tempfile.TemporaryDirectory(prefix="geofx3_ref_")
        _FX["lib"] = compile_frame_fx(Path(_FX["tmp"].name) / "m7_frame_fx.so")
        _FX["fx"] = FrameFx(_FX["lib"])
    return _FX["fx"]


def fx_library():
    fx()
    return _FX["lib"]


def m_to_codes(depth_m, unit=0.001):
    d = np.asarray(depth_m, dtype=np.float64)
    c = np.where(np.isfinite(d) & (d > 0), np.rint(d / unit), 0)
    return np.clip(c, 0, 65535).astype(np.uint16)


def geometry(k, camera_to_world):
    """inverse_k / world_camera exactly as FrameNativeM7.frame_from_depth (and the Nano client) make them."""
    inverse_k = np.ascontiguousarray(np.linalg.inv(np.asarray(k, dtype=np.float64)), dtype=np.float32)
    world_camera = np.ascontiguousarray(np.asarray(camera_to_world, dtype=np.float64)[:3, :], dtype=np.float32)
    return inverse_k, world_camera


def _out(flags, counts, stats, crc, status, shape):
    nx, ny, nz = shape
    return SimpleNamespace(flags=np.asarray(flags)[:nx * ny * nz].reshape(nz, ny, nx).copy(),
                           counts=[int(x) for x in counts],
                           accepted_rays=int(counts[0]), invalid_depth_or_projection=int(counts[1]),
                           outside_grid=int(counts[2]),
                           stats=None if stats is None else [int(x) for x in stats],
                           crc=int(crc), status=int(status))


def expect(codes, k, camera_to_world, grid_origin_m, voxel_size_m, shape, unit_bits=UNIT_MM_BITS, dlat2=0):
    codes = np.asarray(codes, dtype=np.uint16)
    height, width = codes.shape
    nx, ny, nz = shape
    ik, wc = geometry(k, camera_to_world)
    p = fx().prepare(width, height, nx, ny, nz, unit_bits, ik, wc, np.asarray(grid_origin_m, np.float64),
                     float(voxel_size_m))
    if p["status"] != 0:
        return None
    st, flags, counts, stats, crc = fx().frame(codes, width, height, nx, ny, nz, p["words"], p["origin"],
                                               p["codes"], p["c20"], dlat2, unit_bits)
    return _out(flags, counts, stats, crc, st, shape)


def call_codes(backend, codes, k, camera_to_world, grid_origin_m, voxel_size_m, shape,
               unit_bits=UNIT_MM_BITS):
    """One frame through backend.lib.fpga_m7_depth_frame_codes (sentinels 99 as FrameNativeM7)."""
    codes = np.ascontiguousarray(codes, dtype=np.uint16)
    height, width = codes.shape
    nx, ny, nz = shape
    ik, wc = geometry(k, camera_to_world)
    flags = np.full(GRID_CAPACITY, 99, dtype=np.uint8)
    counts = np.full(3, 99, dtype=np.uint32)
    status = np.full(1, 99, dtype=np.uint32)
    crc = backend.lib.fpga_m7_depth_frame_codes(codes.reshape(-1), unit_bits, ik.reshape(-1), wc.reshape(-1),
                                               np.ascontiguousarray(grid_origin_m, dtype=np.float64),
                                               float(voxel_size_m), width, height, nx, ny, nz, flags, counts,
                                               status)
    return _out(flags, counts, backend.lib.last_stats, crc, status[0], shape)


def case_args(case):
    """(codes, k, camera_to_world, grid_origin_m, voxel_size_m, shape) of a float frame case
    (tests/test_m7_backend_switch.frame_cases)."""
    return (m_to_codes(case["depth_m"]), case["k"], case["camera_to_world"], case["grid_origin_m"],
            case["voxel_size_m"], case["shape"])


def same(a, b):
    """Field by field (flags bytes, counts, statistics, crc, status)."""
    return (a is not None and b is not None and np.array_equal(a.flags, b.flags) and a.counts == b.counts and
            a.stats == b.stats and a.crc == b.crc and a.status == b.status)


def replay_dump_equals_fx(out_dir, frames, k, grid_origin_m, voxel_size_m, shape, unit_bits=UNIT_MM_BITS):
    """tools/kv_link/nano_replay.py output (flags.dump + dump_index.csv) against the FrameFx expectation of
    the source codes ``frames[src]`` (camera at the identity pose). Returns (frames_checked, frames_equal).
    nano_replay --compare compares with the float CPU reference, which the fixed-point core is not."""
    import os
    from tools.geo_same_input.common import read_dump
    nvox = int(np.prod(shape))
    counts, flags = read_dump(os.path.join(out_dir, "flags.dump"), nvox)
    with open(os.path.join(out_dir, "dump_index.csv")) as fp:
        index = [tuple(int(x) for x in ln.split(",")) for ln in fp.read().splitlines()[1:] if ln]
    equal = 0
    for row, (_, src) in enumerate(index):
        want = expect(frames[src], k, np.eye(4), grid_origin_m, voxel_size_m, shape, unit_bits)
        equal += int(want is not None and np.array_equal(flags[row], want.flags.ravel()) and
                     [int(x) for x in counts[row]] == want.counts)
    return len(index), equal


class FxFrameBackend:
    """A local frame backend for M7Pipeline whose result is the fixed-point core's model: the float32 depth
    (made from codes as f32(code) * f32(unit)) is turned back into the codes, then FrameFx. The reference
    arm of chain comparisons against a remote PL (kv_server --core pl) since GEO_FX3."""

    backend_name = "fx_model"

    def __init__(self, unit=0.001):
        self.unit = float(unit)
        self.unit_bits = f32_bits(unit)

    def frame_from_depth(self, *, depth_m, k, camera_to_world, grid_origin_m, voxel_size_m, shape, frame_id,
                         capture_time_ns, pose_time_ns, calibration_version, pose_confirmed):
        from host.m7_native import FrameEvidence
        codes = m_to_codes(depth_m, self.unit)
        out = expect(codes, k, camera_to_world, grid_origin_m, voxel_size_m, shape, self.unit_bits)
        if out is None:
            raise ValueError("PS status != 0: this reference covers in-envelope frames only")
        return FrameEvidence(int(frame_id), int(capture_time_ns), int(pose_time_ns), calibration_version,
                             out.flags, out.accepted_rays, out.invalid_depth_or_projection, out.outside_grid,
                             float(voxel_size_m) / 256.)
