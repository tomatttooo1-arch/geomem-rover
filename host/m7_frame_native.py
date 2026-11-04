"""CPU-callable reference for the complete depth-to-frame-evidence HLS top.

This backend executes a compiled C++ library on the host. It does not run a
KV260, connect an ap_memory port, transfer camera data, or verify pose trust.
The kernel's float32 projection and float64 Q8 conversion intentionally match
the existing NativeM7 reference before any later fixed-point optimization.
Since 2026-10-11 (docs/design/M7_CLIP_SPEC_20261011.md) rays whose endpoint is
outside the grid are clipped, not dropped: their in-grid voxels become FREE and
no HIT is written; FrameEvidence.outside_grid counts them ("clipped"). NativeM7
keeps the 09-21 drop rule, so flags agree with it only when nothing is clipped.
"""

import ctypes
from pathlib import Path
import shutil
import subprocess

import numpy as np

from host.m7_native import FrameEvidence, NativeM7


ROOT = Path(__file__).resolve().parents[1]
MAX_WIDTH = 1280
MAX_HEIGHT = 800
MAX_PIXELS = MAX_WIDTH * MAX_HEIGHT
GRID_CAPACITY = 32 ** 3
F32 = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
F64 = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
U8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
U32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags="C_CONTIGUOUS")


def compile_frame_native(output: Path) -> Path:
    """Build a fresh host library from exactly the HLS top's C++ source."""
    compiler = shutil.which("g++")
    if compiler is None:
        raise RuntimeError("g++ is required for the M7 frame host reference")
    output = Path(output).resolve()
    if not output.parent.is_dir() or output.exists():
        raise ValueError("choose a fresh output path within an existing directory")
    subprocess.run([compiler, "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-ffp-contract=off", "-fPIC", "-shared",
                    str(ROOT / "hw/m7/frame_depth.cpp"), "-o", str(output)],
                   check=True, capture_output=True, text=True)
    return output


class FrameNativeM7:
    """A drop-in CPU backend for M7Pipeline.frame_from_depth."""

    def __init__(self, library: Path):
        self.lib = ctypes.CDLL(str(Path(library).resolve()))
        self.lib.fpga_m7_depth_frame.argtypes = [
            F32, F32, F32, F64, ctypes.c_double,
            ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
            ctypes.c_uint32, ctypes.c_uint32, U8, U32, U32]
        self.lib.fpga_m7_depth_frame.restype = None

    def frame_from_depth(self, *, depth_m, k, camera_to_world,
                         grid_origin_m, voxel_size_m, shape, frame_id,
                         capture_time_ns, pose_time_ns, calibration_version,
                         pose_confirmed):
        nx, ny, nz = NativeM7._dimensions(shape)
        if pose_confirmed is not True:
            raise ValueError("a trusted, time-aligned camera pose is required")
        if (not isinstance(frame_id, (int, np.integer)) or frame_id < 0 or
            not isinstance(capture_time_ns, (int, np.integer)) or capture_time_ns < 0 or
            not isinstance(pose_time_ns, (int, np.integer)) or pose_time_ns < 0 or
            not isinstance(calibration_version, str) or not calibration_version):
            raise ValueError("frame/time/calibration identifiers are required")
        if not np.isfinite(voxel_size_m) or voxel_size_m <= 0:
            raise ValueError("voxel size must be positive and finite")
        depth = np.asarray(depth_m, dtype=np.float32)
        intrinsic = np.asarray(k, dtype=np.float64)
        transform = np.asarray(camera_to_world, dtype=np.float64)
        grid_origin = np.asarray(grid_origin_m, dtype=np.float64)
        if (depth.ndim != 2 or intrinsic.shape != (3, 3) or
            transform.shape != (4, 4) or grid_origin.shape != (3,) or
            not np.all(np.isfinite(intrinsic)) or not np.all(np.isfinite(transform)) or
            not np.all(np.isfinite(grid_origin)) or
            not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8, rtol=0)):
            raise ValueError("malformed depth/calibration/pose/grid")
        height, width = depth.shape
        if width > MAX_WIDTH or height > MAX_HEIGHT or (width == 0) != (height == 0):
            raise ValueError("frame exceeds 1280x800 or has one empty axis")
        if (intrinsic[0, 0] <= 0 or intrinsic[1, 1] <= 0 or
            not np.allclose(intrinsic[2], [0, 0, 1], atol=1e-8, rtol=0) or
            abs(intrinsic[1, 0]) > 1e-8 or
            not np.allclose(transform[:3, :3] @ transform[:3, :3].T,
                            np.eye(3), atol=1e-4, rtol=0) or
            not np.isclose(np.linalg.det(transform[:3, :3]), 1., atol=1e-4,
                           rtol=0)):
            raise ValueError("nonphysical intrinsics or improper camera pose")
        inverse_k = np.ascontiguousarray(np.linalg.inv(intrinsic), dtype=np.float32)
        world_camera = np.ascontiguousarray(transform[:3, :], dtype=np.float32)
        if not np.all(np.isfinite(inverse_k)) or not np.all(np.isfinite(world_camera)):
            raise ValueError("calibration/pose overflows float32 projection")

        depth_buffer = np.zeros(MAX_PIXELS, dtype=np.float32)
        depth_buffer[:depth.size] = depth.ravel()
        origin_buffer = np.ascontiguousarray(grid_origin, dtype=np.float64)
        flags = np.full(GRID_CAPACITY, 99, dtype=np.uint8)
        counts = np.full(3, 99, dtype=np.uint32)
        status = np.full(1, 99, dtype=np.uint32)
        self.lib.fpga_m7_depth_frame(
            depth_buffer, inverse_k, world_camera, origin_buffer,
            float(voxel_size_m), width, height, nx, ny, nz, flags, counts, status)
        if status[0] != 0:
            if np.any(flags != 99) or np.any(counts != 0):
                raise RuntimeError("failed frame changed output or retained counts")
            raise ValueError(f"M7 frame rejected with status {status[0]}")
        if (int(np.sum(counts.astype(np.uint64))) != depth.size or
            not np.isin(flags[:nx*ny*nz], [0, 1, 2]).all()):
            raise RuntimeError("M7 frame returned non-conservative counts or flags")
        return FrameEvidence(
            int(frame_id), int(capture_time_ns), int(pose_time_ns),
            calibration_version, flags[:nx*ny*nz].reshape(nz, ny, nx).copy(),
            int(counts[0]), int(counts[1]), int(counts[2]),
            float(voxel_size_m) / 256.)
