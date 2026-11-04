"""CPU caller for the M7 PL display-pixel candidate; not a video driver.

Depth is calibrated optical-axis Z in metres. The C ABI sees quantized
millimetres, with zero for every invalid/missing floating sample. An optional
bridge revalidates the M7 consumer output at display time and checks semantic
identity/token before any object evidence reaches the pixel kernel.
"""

from dataclasses import dataclass
from pathlib import Path
import ctypes
import shutil
import subprocess
from typing import Optional

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MAX_WIDTH = 1280
MAX_HEIGHT = 800
MAX_PIXELS = MAX_WIDTH * MAX_HEIGHT
DEPTH_MIN_M = 0.1  # strict inequality, same as M7Pipeline
DEPTH_MAX_M = 10.0
UNKNOWN, PRESENT, FREE_CONFLICT = 0, 1, 2
OCCLUDED = 3  # mask-free region check only (OBJECT_ROI_SPEC s.8)
U16 = np.ctypeslib.ndpointer(dtype=np.uint16, flags="C_CONTIGUOUS")
U32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags="C_CONTIGUOUS")


def compile_native(output: Path) -> Path:
    """Build only the new display candidate into a fresh local shared object."""
    compiler = shutil.which("g++")
    if compiler is None:
        raise RuntimeError("g++ is required for the M7 display CPU reference")
    output = Path(output).resolve()
    if not output.parent.is_dir() or output.exists():
        raise ValueError("choose a fresh output path within an existing directory")
    subprocess.run([compiler, "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-fPIC", "-shared", str(ROOT / "hw/m7/display_overlay.cpp"),
                    "-o", str(output)], check=True, capture_output=True, text=True)
    return output


def quantize_optical_z(depth_m):
    """Return uint16 mm plus counts; invalid floats become zero.

    Validity is decided *before* rounding. Thus 0.1001 m remains valid even
    when rounded to 100 mm; exactly 0.1 and exactly 10 m are invalid. The
    quantization error is at most 0.5 mm except for clamping just below 10 m
    (less than 1 mm). This does not bound sensor/calibration error.
    """
    depth = np.asarray(depth_m)
    if (depth.ndim != 2 or not np.issubdtype(depth.dtype, np.floating)
            or depth.shape[0] < 1 or depth.shape[1] < 1
            or depth.shape[0] > MAX_HEIGHT or depth.shape[1] > MAX_WIDTH):
        raise ValueError("floating HxW optical-axis depth must fit 1280x800")
    valid = np.isfinite(depth) & (depth > DEPTH_MIN_M) & (depth < DEPTH_MAX_M)
    result = np.zeros(depth.shape, dtype=np.uint16)
    if valid.any():
        mm = np.floor(depth[valid].astype(np.float64) * 1000.0 + 0.5)
        result[valid] = np.clip(mm, 100, 9999).astype(np.uint16)
    count = int(np.count_nonzero(valid))
    return np.ascontiguousarray(result), count, int(depth.size-count)


@dataclass(frozen=True)
class DisplayPixels:
    rgb888: np.ndarray  # [height,width] uint32, RGB888 in low 24 bits
    depth_mm: np.ndarray  # sanitized input copied from caller
    valid_depth_pixels: int
    invalid_depth_pixels: int
    nearest_optical_z_mm: Optional[int]
    fresh: bool  # upstream and local age/pose flags, plus at least one depth
    object_evidence: int
    reason: str


class NativeM7Display:
    def __init__(self, library: Path):
        self.lib = ctypes.CDLL(str(Path(library).resolve()))
        self.lib.fpga_m7_display_overlay.argtypes = [
            U16, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
            ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
            ctypes.c_uint32, U32, U32,
        ]
        self.lib.fpga_m7_display_overlay.restype = None

    def render(self, depth_m, *, frame_valid: bool, pose_valid: bool,
               age_ms: int, max_age_ms: int, object_evidence: int = UNKNOWN,
               reason: str = "direct_parameters") -> DisplayPixels:
        if type(frame_valid) is not bool or type(pose_valid) is not bool:
            raise ValueError("frame_valid and pose_valid must be booleans")
        if (type(age_ms) is not int or type(max_age_ms) is not int
                or age_ms < 0 or max_age_ms < 0 or age_ms > 0xffffffff
                or max_age_ms > 0xffffffff):
            raise ValueError("age_ms and max_age_ms must be uint32")
        if type(object_evidence) is not int or object_evidence not in (0, 1, 2, 3):
            raise ValueError("object_evidence must be UNKNOWN, PRESENT, FREE_CONFLICT or OCCLUDED")
        mm, valid_count, invalid_count = quantize_optical_z(depth_m)
        h, w = mm.shape
        output = np.full((h, w), 0xDEADBEEF, dtype=np.uint32)
        status = np.full(1, 99, dtype=np.uint32)
        self.lib.fpga_m7_display_overlay(mm, w, h, int(frame_valid),
                                        int(pose_valid), age_ms, max_age_ms,
                                        object_evidence, output, status)
        if status[0] != 0 or np.any(output == 0xDEADBEEF):
            raise RuntimeError(f"display kernel failed or left pixels unwritten: {status[0]}")
        fresh = frame_valid and pose_valid and age_ms <= max_age_ms and valid_count > 0
        nearest = int(mm[mm != 0].min()) if fresh else None
        return DisplayPixels(output.copy(), mm.copy(), valid_count, invalid_count,
                             nearest, fresh, object_evidence if fresh else UNKNOWN,
                             reason)

    def render_consumer_output(self, pipeline, frame, display_output, *,
                               now_ns: int, clock_id: str,
                               semantic_output=None) -> DisplayPixels:
        """Use original cached depth only after live provenance/time checks.

        A still-cached older memory result remains legal for memory; it is
        intentionally UNKNOWN on a newer display frame. This prototype uses
        M7Pipeline's private cached snapshot, not a wire or PS/PL protocol.
        """
        from host.m7_pipeline import ConsumerOutput, DepthFrame, M7Pipeline

        if (not isinstance(pipeline, M7Pipeline) or not isinstance(frame, DepthFrame)
                or not isinstance(display_output, ConsumerOutput)):
            raise ValueError("M7Pipeline, DepthFrame and ConsumerOutput required")
        # Shape is also checked on failure so a bad input cannot send a stale
        # image through an unconstrained output allocation.
        fallback, _, _ = quantize_optical_z(frame.depth_m)
        same_identity = frame.identity == display_output.identity
        checked = display_output.validate_at(pipeline, now_ns=now_ns, clock_id=clock_id)
        saved = pipeline._cache.get(display_output.identity) if same_identity else None
        live = bool(checked.valid and checked.channel == "display" and same_identity
                    and saved is not None and saved.token == checked.frame_token
                    and saved.depth_m.shape == fallback.shape)
        depth = saved.depth_m if live else np.zeros(fallback.shape, dtype=np.float32)
        # Exact nanosecond check above is authoritative. Rounding age upward
        # adds a conservative local HLS guard; flooring would make an expired
        # sub-millisecond fraction look fresh.
        age_ns = now_ns - frame.identity.acquired_ns if clock_id == frame.identity.clock_id else 0
        age_ms = min(0xffffffff, max(0, (age_ns + 999_999) // 1_000_000))
        max_age_ms = min(0xffffffff, pipeline.max_frame_age_ns // 1_000_000)
        pose_valid = bool(same_identity and frame.pose.valid)
        evidence = UNKNOWN
        if live and semantic_output is not None:
            if not isinstance(semantic_output, ConsumerOutput):
                raise ValueError("semantic_output must be ConsumerOutput")
            semantic = semantic_output.validate_at(pipeline, now_ns=now_ns,
                                                   clock_id=clock_id)
            same_frame = (semantic.valid and semantic.identity == checked.identity
                          and semantic.frame_token == checked.frame_token
                          and semantic.generation == checked.generation)
            if same_frame and semantic.channel == "memory":
                # Mask path: its occlusion sub-reason stays "OBJ ?".
                evidence = {
                    "support_present": PRESENT,
                    "free_space_conflict": FREE_CONFLICT,
                }.get(semantic.payload.get("candidate"), UNKNOWN)
            elif same_frame and semantic.channel == "region":
                evidence = {
                    "occluded": OCCLUDED,
                    "free_space_conflict": FREE_CONFLICT,
                }.get(semantic.payload.get("verdict"), UNKNOWN)
        failure_reason = checked.reason if not checked.valid else "display_identity_or_token_mismatch"
        return self.render(depth, frame_valid=live, pose_valid=pose_valid,
                           age_ms=age_ms, max_age_ms=max_age_ms,
                           object_evidence=evidence,
                           reason="current_frame_display" if live else failure_reason)
