"""Explicit host reference for the bounded M7 C ABI; this is not a board driver.

World coordinates are metres. The grid uses a caller-provided origin and voxel
size. Each coordinate is floored to a Q8 integer (256 units per voxel); the
quantized ray is the input to the integer traversal, not a claim about an
unquantized ray near a voxel boundary. Pose trust and time synchronization
must be established by the caller before calling ``frame_from_depth``.
The reported Q8 error bound covers the final flooring alone; float32 camera
projection, pose and calibration errors are separate and NOT included.
"""

from dataclasses import dataclass
from pathlib import Path
import ctypes
import shutil
import subprocess

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
RAY_CAPACITY = 4096
GRID_CAPACITY = 32 ** 3
Q8 = 256
I32 = np.ctypeslib.ndpointer(dtype=np.int32, flags="C_CONTIGUOUS")
U32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags="C_CONTIGUOUS")
U8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
F32 = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")


def compile_native(output: Path) -> Path:
    """Compile the unchanged old geometry reference and the new ray kernel."""
    compiler = shutil.which("g++")
    if compiler is None:
        raise RuntimeError("g++ is required for the M7 host reference")
    output = Path(output).resolve()
    if not output.parent.is_dir() or output.exists():
        raise ValueError("choose a fresh output path within an existing directory")
    subprocess.run([compiler, "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-ffp-contract=off", "-fPIC", "-shared",
                    str(ROOT / "hw/geometry/geometry.cpp"),
                    str(ROOT / "hw/m7/ray_flags.cpp"), "-o", str(output)],
                   check=True, capture_output=True, text=True)
    return output


@dataclass(frozen=True)
class FrameEvidence:
    frame_id: int
    capture_time_ns: int
    pose_time_ns: int
    calibration_version: str
    flags: np.ndarray  # [z,y,x], values 0 unknown / 1 free / 2 hit.
    accepted_rays: int
    invalid_depth_or_projection: int
    # Endpoint outside the grid. NativeM7 (two-stage, frozen 09-21 rule) drops
    # these rays; FrameNativeM7/PL (hw/m7/frame_depth.cpp, since 2026-10-11)
    # clip them: the in-grid part is FREE, no HIT (M7_CLIP_SPEC_20261011).
    # The classification, hence the count, is the same in both.
    outside_grid: int
    quantization_axis_error_bound_m: float


class NativeM7:
    def __init__(self, library: Path):
        self.lib = ctypes.CDLL(str(Path(library).resolve()))
        self.lib.fpga_m7_ray_flags.argtypes = [I32, I32, U32, ctypes.c_uint32,
                                              ctypes.c_uint32, ctypes.c_uint32,
                                              ctypes.c_uint32, U8, U32]
        self.lib.fpga_m7_ray_flags.restype = None
        self.lib.fpga_backproject.argtypes = [F32, F32, F32, ctypes.c_uint32,
                                             F32, U32, U32]
        self.lib.fpga_backproject.restype = None

    @staticmethod
    def _dimensions(shape):
        if len(shape) != 3 or any(not isinstance(n, (int, np.integer)) or
                                  n < 1 or n > 32 for n in shape):
            raise ValueError("grid shape (nx, ny, nz) requires integers in 1..32")
        return tuple(int(n) for n in shape)

    def trace(self, origins, ends, shape, ray_valid=None):
        """Trace Q8 integer coordinates; return [z,y,x] flags.

        All enabled rays must fit the grid. The C ABI rejects a malformed
        batch before touching output. The host also validates request shape.
        """
        nx, ny, nz = self._dimensions(shape)
        starts = np.asarray(origins)
        stops = np.asarray(ends)
        if starts.ndim != 2 or starts.shape[1] != 3 or stops.shape != starts.shape:
            raise ValueError("origins and ends must have matching [N,3] shape")
        count = len(starts)
        if count > RAY_CAPACITY:
            raise ValueError("trace at most 4096 rays per call")
        if not np.issubdtype(starts.dtype, np.integer) or not np.issubdtype(stops.dtype, np.integer):
            raise ValueError("Q8 inputs must be integers")
        if any(np.any(a < np.iinfo(np.int32).min) or
               np.any(a > np.iinfo(np.int32).max) for a in (starts, stops)):
            raise ValueError("Q8 input exceeds int32")
        if ray_valid is None:
            active = np.ones(count, dtype=np.uint32)
        else:
            active = np.asarray(ray_valid)
            if active.shape != (count,) or not np.issubdtype(active.dtype, np.integer) or \
                    np.any((active != 0) & (active != 1)):
                raise ValueError("ray_valid must contain exactly 0 or 1 per ray")
            active = active.astype(np.uint32)
        origin_buf = np.zeros((RAY_CAPACITY, 3), dtype=np.int32)
        end_buf = np.zeros_like(origin_buf)
        active_buf = np.zeros(RAY_CAPACITY, dtype=np.uint32)
        origin_buf[:count] = starts
        end_buf[:count] = stops
        active_buf[:count] = active
        output = np.full(GRID_CAPACITY, 99, dtype=np.uint8)
        status = np.full(1, 99, dtype=np.uint32)
        self.lib.fpga_m7_ray_flags(origin_buf, end_buf, active_buf, count, nx, ny, nz,
                                   output, status)
        if status[0] != 0:
            if np.any(output != 99):
                raise RuntimeError("C ABI failure changed output; reject corrupted frame")
            raise ValueError(f"M7 ray batch rejected with status {status[0]}")
        return output[:nx * ny * nz].reshape(nz, ny, nx).copy()

    def frame_from_depth(self, *, depth_m, k, camera_to_world,
                         grid_origin_m, voxel_size_m, shape, frame_id,
                         capture_time_ns, pose_time_ns, calibration_version,
                         pose_confirmed):
        """Project old C++ float reference, quantize, and trace in 4096-ray tiles.

        Out-of-grid endpoints are conservatively discarded and counted. No
        ray from an invalid depth, invalid projection, or outside endpoint
        contributes FREE. The camera origin MUST lie inside the requested grid.
        The returned arrays are per-frame observations, never persistent odds.
        """
        nx, ny, nz = self._dimensions(shape)
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
        dims = np.array([nx, ny, nz], dtype=np.int64)

        def quantize(points):
            # C++ projection is float32. Use float64 for the metric-to-grid
            # conversion to avoid adding unnecessary numerical error.
            return np.floor((points.astype(np.float64) - grid_origin) /
                            voxel_size_m * Q8)

        camera_q8 = quantize(world_camera[:, 3])
        if np.any(~np.isfinite(camera_q8)) or np.any(camera_q8 < 0) or \
                np.any(camera_q8 >= dims * Q8):
            raise ValueError("camera origin outside grid: no safe ray traversal")
        origin = camera_q8.astype(np.int32)
        evidence = np.zeros((nz, ny, nx), dtype=np.uint8)
        invalid = outside = accepted = 0
        flat_depth = depth.ravel()
        height, width = depth.shape
        for base in range(0, height * width, RAY_CAPACITY):
            count = min(RAY_CAPACITY, height * width - base)
            pixels = np.arange(base, base + count, dtype=np.int64)
            uvz = np.zeros((RAY_CAPACITY, 3), dtype=np.float32)
            uvz[:count, 0] = pixels % width
            uvz[:count, 1] = pixels // width
            uvz[:count, 2] = flat_depth[pixels]
            xyz = np.zeros_like(uvz)
            valid = np.zeros(RAY_CAPACITY, dtype=np.uint32)
            status = np.full(1, 99, dtype=np.uint32)
            self.lib.fpga_backproject(uvz, inverse_k, world_camera, count,
                                     xyz, valid, status)
            if status[0] != 0:
                raise RuntimeError(f"old backprojection rejected tile: {status[0]}")
            invalid += count - int(np.count_nonzero(valid[:count]))
            projected = xyz[:count][valid[:count] != 0]
            if len(projected) == 0:
                continue
            end_q8 = quantize(projected)
            inside = np.all(np.isfinite(end_q8) & (end_q8 >= 0) &
                            (end_q8 < dims * Q8), axis=1)
            outside += len(projected) - int(np.count_nonzero(inside))
            ends = end_q8[inside].astype(np.int32)
            if len(ends) == 0:
                continue
            starts = np.broadcast_to(origin, ends.shape)
            tile = self.trace(starts, ends, shape)
            np.maximum(evidence, tile, out=evidence)  # HIT > FREE > UNKNOWN.
            accepted += len(ends)
        return FrameEvidence(int(frame_id), int(capture_time_ns), int(pose_time_ns),
                             calibration_version, evidence, accepted, invalid,
                             outside, float(voxel_size_m) / Q8)
