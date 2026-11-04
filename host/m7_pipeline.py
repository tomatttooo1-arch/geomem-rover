"""Bounded asynchronous M7 host contract and CPU reference consumer outputs.

The host validates capture identity, calibration, time and pose before calling a
voxel backend. It never performs ray traversal or updates long-term memory.
``backend`` is explicit: the native C++ CPU reference is the intended backend.
These records specify proposed PL/PS interfaces; they do not implement PL display.
"""

from collections import OrderedDict
from dataclasses import dataclass, field
from dataclasses import replace
import hashlib
import logging
import math
import uuid
from typing import Optional

import numpy as np

from perception.fpga_replay.core import Config


BACKEND = "cpu_reference"


def _identity_text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")


def _nanoseconds(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer in nanoseconds")


@dataclass(frozen=True)
class CaptureIdentity:
    stream_instance: str
    frame_id: int  # strictly increasing within the stream_instance
    acquired_ns: int
    clock_id: str
    calibration_id: str
    local_frame_id: str
    pose_epoch: str

    def __post_init__(self):
        for name in ("stream_instance", "clock_id", "calibration_id",
                     "local_frame_id", "pose_epoch"):
            _identity_text(getattr(self, name), name)
        _nanoseconds(self.frame_id, "frame_id")
        _nanoseconds(self.acquired_ns, "acquired_ns")

    @property
    def context(self):
        return (self.stream_instance, self.clock_id, self.calibration_id,
                self.local_frame_id, self.pose_epoch)


@dataclass(frozen=True)
class GridSpec:
    origin_m: tuple
    voxel_m: float
    shape: tuple  # nx, ny, nz; native flattened index = (z * ny + y) * nx + x

    def __post_init__(self):
        if len(self.origin_m) != 3 or not all(math.isfinite(v) for v in self.origin_m):
            raise ValueError("grid origin must be finite XYZ metres")
        if not math.isfinite(self.voxel_m) or self.voxel_m <= 0:
            raise ValueError("voxel edge must be finite positive metres")
        if len(self.shape) != 3 or any(type(n) is not int or n < 1 or n > 32 for n in self.shape):
            raise ValueError("grid dimensions must be integer values in [1, 32]")
        object.__setattr__(self, "origin_m", tuple(self.origin_m))
        object.__setattr__(self, "shape", tuple(self.shape))


@dataclass(frozen=True)
class PoseAtCapture:
    stamp_ns: int
    clock_id: str
    local_frame_id: str
    pose_epoch: str
    valid: bool
    T_local_camera: np.ndarray


@dataclass(frozen=True)
class DepthFrame:
    identity: CaptureIdentity
    depth_m: np.ndarray  # aligned optical-axis Z in metres; NaN denotes missing returns
    K: np.ndarray  # calibrated for this exact depth resolution and calibration_id
    pose: PoseAtCapture
    depth_representation: str = "optical_z_m"


@dataclass(frozen=True)
class ConsumerOutput:
    identity: CaptureIdentity
    valid: bool
    reason: str
    age_ns: Optional[int]
    expires_at_ns: Optional[int] = None
    backend: str = BACKEND
    frame_token: Optional[str] = None
    generation: str = ""
    channel: str = ""
    payload: dict = field(default_factory=dict)

    def validate_at(self, pipeline, now_ns: int, clock_id: str):
        """Revalidate with live pipeline state before using a retained result."""
        return pipeline.validate_output(self, now_ns=now_ns, clock_id=clock_id)

    def _time_valid_at(self, now_ns: int, clock_id: str):
        _nanoseconds(now_ns, "now_ns")
        _identity_text(clock_id, "clock_id")
        age = now_ns - self.identity.acquired_ns if clock_id == self.identity.clock_id else None
        if clock_id != self.identity.clock_id:
            return replace(self, valid=False, reason="clock_mismatch", age_ns=None, payload={})
        if not self.valid:
            return replace(self, age_ns=age, payload={})
        if now_ns < self.identity.acquired_ns:
            return replace(self, valid=False, reason="future_output", age_ns=age, payload={})
        if self.expires_at_ns is None or now_ns > self.expires_at_ns:
            return replace(self, valid=False, reason="expired_output", age_ns=age, payload={})
        return replace(self, age_ns=age)


@dataclass(frozen=True)
class GeometryOutputs:
    navigation: ConsumerOutput
    display: ConsumerOutput


@dataclass(frozen=True)
class SemanticPacket:
    identity: CaptureIdentity
    frame_token: str
    object_id: str
    mask: Optional[np.ndarray]  # None = missing detector result; all-False = explicit negative
    historical_bounds_local_m: Optional[np.ndarray] = None  # (2, 3) lower/upper


@dataclass(frozen=True)
class RegionCheckPacket:
    """Mask-free region check for objects with no detector mask this frame.

    entries: tuple of (object_id, historical_bounds_local_m (2, 3)). See
    docs/design/OBJECT_ROI_SPEC_20261011.md. Outputs use channel "region",
    which build_m4_document never consumes.
    """
    identity: CaptureIdentity
    frame_token: str
    entries: tuple


@dataclass
class _CachedFrame:
    depth_m: np.ndarray
    K: np.ndarray
    pose: np.ndarray
    token: str
    used_objects: set = field(default_factory=set)
    # Separate from used_objects: a mask result and a region check answer
    # different questions, so one object may receive one of each per frame.
    used_region_objects: set = field(default_factory=set)


class M7Pipeline:
    """Single-stream, fail-closed host scheduling for an explicit voxel backend.

    backend.frame_from_depth(...) is the native host reference, returning
    `flags` ([z,y,x] uint8 UNKNOWN=0/FREE=1/HIT=2) and accepted_rays.
    Backend errors invalidate this frame; no previous obstacle grid is reused.
    """

    def __init__(self, backend, grid: GridSpec, *, max_frames=4, max_pixels=1_048_576,
                 max_objects_per_frame=64,
                 max_frame_age_ns=200_000_000, max_cache_age_ns=500_000_000,
                 max_pose_age_ns=50_000_000, future_tolerance_ns=0, cfg=Config(),
                 object_backend=None, grid_policy=None, region_backend=None):
        if backend is None or not callable(getattr(backend, "frame_from_depth", None)):
            raise ValueError("explicit voxel backend with frame_from_depth() required")
        if object_backend is not None and not callable(getattr(object_backend, "candidate", None)):
            raise ValueError("object_backend must provide candidate()")
        if region_backend is not None and not callable(getattr(region_backend, "check_boxes", None)):
            raise ValueError("region_backend must provide check_boxes()")
        if not isinstance(grid, GridSpec):
            raise ValueError("GridSpec required")
        for name, value in (("max_frames", max_frames), ("max_pixels", max_pixels),
                            ("max_objects_per_frame", max_objects_per_frame),
                            ("max_frame_age_ns", max_frame_age_ns),
                            ("max_cache_age_ns", max_cache_age_ns),
                            ("max_pose_age_ns", max_pose_age_ns),
                            ("future_tolerance_ns", future_tolerance_ns)):
            _nanoseconds(value, name)
            if name in ("max_frames", "max_pixels", "max_objects_per_frame") and not value:
                raise ValueError(f"{name} must be positive")
        defaults = Config()
        if (not isinstance(cfg, Config) or not math.isfinite(cfg.depth_margin_m)
                or cfg.depth_margin_m < defaults.depth_margin_m or
                not math.isfinite(cfg.association_m) or
                not 0 < cfg.association_m <= defaults.association_m or
                not math.isfinite(cfg.min_valid_fraction) or
                not defaults.min_valid_fraction <= cfg.min_valid_fraction <= 1 or
                type(cfg.min_pixels) is not int or cfg.min_pixels < defaults.min_pixels or
                not math.isfinite(cfg.free_fraction) or
                not defaults.free_fraction <= cfg.free_fraction <= 1):
            raise ValueError("geometry evidence thresholds cannot relax existing Config")
        if grid_policy is not None:
            # Optional per-frame grid (M7_FOLLOW_GRID_20260927). ``grid`` still
            # declares voxel and shape; the policy only moves the origin.
            if not callable(getattr(grid_policy, "grid_for", None)):
                raise ValueError("grid_policy must provide grid_for(T_local_camera)")
            if (getattr(grid_policy, "voxel_m", None) != grid.voxel_m or
                    tuple(getattr(grid_policy, "shape", ())) != tuple(grid.shape)):
                raise ValueError("grid_policy voxel/shape must equal the declared grid")
        self.grid_policy = grid_policy
        self.backend, self.grid, self.cfg = backend, grid, cfg
        self.object_backend = object_backend
        self.region_backend = region_backend
        self.region_backend_label = (None if region_backend is None else
                                     getattr(region_backend, "backend_name", BACKEND))
        # Explicit backend label on every output (M7_BACKEND_SWITCH_20260927):
        # a backend without backend_name keeps the historical cpu_reference.
        self.backend_label = getattr(backend, "backend_name", BACKEND)
        self.object_backend_label = (BACKEND if object_backend is None else
                                     getattr(object_backend, "backend_name", BACKEND))
        self.max_frames, self.max_pixels = max_frames, max_pixels
        self.max_objects_per_frame = max_objects_per_frame
        self.max_frame_age_ns, self.max_cache_age_ns = max_frame_age_ns, max_cache_age_ns
        self.max_pose_age_ns, self.future_tolerance_ns = max_pose_age_ns, future_tolerance_ns
        # 10-26 #2: all-invalid depth frames whose (blank) request still went to the backend / of those, failed
        self.blank_requests = 0
        self.blank_request_failures = 0
        self._cache = OrderedDict()
        self._context = None
        self._last_ns = None
        self._last_frame_id = None
        self._calibration_signature = None
        self._generation = uuid.uuid4().hex
        self._active_geometry_token = None
        self._discontinuity = False

    def reset(self):
        """Explicit discontinuity at stream/calibration/pose-epoch changes.

        Tokens are minted independently for every admitted frame, including
        after reset; a delayed semantic packet from a previous epoch cannot
        bind to a new frame even when upstream reuses a frame_id.
        """
        self._cache.clear()
        self._context = None
        self._last_ns = None
        self._last_frame_id = None
        self._calibration_signature = None
        self._generation = uuid.uuid4().hex
        self._active_geometry_token = None
        self._discontinuity = False

    def validate_output(self, output: ConsumerOutput, *, now_ns: int, clock_id: str):
        """Recheck time and live provenance; memory may use an older cached frame.

        Navigation and board-display inputs require the latest valid geometry.
        Memory candidates retain their original-frame context and cache TTL.
        A reset/context discontinuity revokes both classes, including old objects
        that happen to reuse the same identity after a stream restart.
        """
        if not isinstance(output, ConsumerOutput):
            raise ValueError("ConsumerOutput required")
        checked = output._time_valid_at(now_ns, clock_id)
        if not checked.valid:
            return checked
        if (self._discontinuity or output.generation != self._generation or
                output.identity.context != self._context):
            return replace(checked, valid=False, reason="generation_or_context_revoked", payload={})
        if output.channel in ("navigation", "display") and output.frame_token != self._active_geometry_token:
            return replace(checked, valid=False, reason="geometry_revoked", payload={})
        if output.channel in ("memory", "region"):
            saved = self._cache.get(output.identity)
            if saved is None or saved.token != output.frame_token:
                # Same reason string for both cached-frame channels.
                return replace(checked, valid=False, reason="memory_frame_revoked", payload={})
        return checked

    @property
    def cached_frames(self):
        return len(self._cache)

    def _age(self, identity, now_ns, now_clock_id, limit):
        _nanoseconds(now_ns, "now_ns")
        _identity_text(now_clock_id, "now_clock_id")
        if now_clock_id != identity.clock_id:
            return None, "clock_mismatch"
        age = now_ns - identity.acquired_ns
        if age < -self.future_tolerance_ns:
            return age, "future_frame"
        if age > limit:
            return age, "expired_frame"
        return age, None

    def _invalid(self, identity, reason, age):
        if (self._context is not None and identity.context != self._context and
                reason != "late_geometry"):
            self._discontinuity = True
        if (self._last_frame_id is None or identity.frame_id > self._last_frame_id or
                reason in ("context_changed_reset_required",
                           "calibration_content_changed_reset_required")):
            self._active_geometry_token = None
        nav = ConsumerOutput(identity, False, reason, age, generation=self._generation,
                             channel="navigation", backend=self.backend_label)
        display = replace(nav, channel="display")
        return GeometryOutputs(nav, display)

    def _invalid_semantic(self, identity, reason, age):
        return ConsumerOutput(identity, False, reason, age, generation=self._generation,
                              channel="memory", backend=self.object_backend_label)

    def _invalid_region(self, identity, reason, age, object_id=None):
        return ConsumerOutput(identity, False, reason, age, generation=self._generation,
                              channel="region", backend=self.region_backend_label or BACKEND,
                              payload={} if object_id is None else {"object_id": object_id})

    def _frame_inputs(self, frame):
        if not isinstance(frame, DepthFrame):
            raise ValueError("DepthFrame required")
        if frame.depth_representation != "optical_z_m":
            raise ValueError("depth must declare calibrated optical-axis Z in metres")
        depth = np.asarray(frame.depth_m)
        if depth.ndim != 2 or not np.issubdtype(depth.dtype, np.floating):
            raise ValueError("depth must be floating HxW optical-axis Z")
        h, w = depth.shape
        if not h or not w or h * w > self.max_pixels:
            raise ValueError("depth image exceeds bounded per-frame pixel limit")
        if np.isinf(depth).any() or np.isneginf(depth).any():
            raise ValueError("infinite depth is not a missing return; use NaN")
        if np.isfinite(depth).any() and np.nanmax(depth) > 20:
            raise ValueError("depth appears outside indoor metre range; verify scale")
        k = np.asarray(frame.K, dtype=np.float64)
        if (k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
                or not np.allclose(k[2], [0, 0, 1], atol=1e-8, rtol=0)
                or abs(k[1, 0]) > 1e-8
                or abs(float(np.linalg.det(k))) < 1e-12):
            raise ValueError("invalid calibrated depth intrinsics")
        pose = frame.pose
        if not isinstance(pose, PoseAtCapture):
            raise ValueError("PoseAtCapture required")
        _nanoseconds(pose.stamp_ns, "pose.stamp_ns")
        t = np.asarray(pose.T_local_camera, dtype=np.float64)
        if t.shape != (4, 4) or not np.isfinite(t).all():
            raise ValueError("invalid camera-to-local rigid transform")
        r = t[:3, :3]
        if (not np.allclose(t[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
                or not np.allclose(r.T @ r, np.eye(3), atol=1e-5, rtol=0)
                or not np.isclose(np.linalg.det(r), 1, atol=1e-5, rtol=0)):
            raise ValueError("camera-to-local transform must be proper rigid SE(3)")
        return depth.copy(), k.copy(), t.copy()

    def ingest_geometry(self, frame: DepthFrame, *, now_ns: int, now_clock_id: str):
        identity = frame.identity
        try:
            depth, k, t = self._frame_inputs(frame)
        except ValueError:
            if self._last_frame_id is None or identity.frame_id > self._last_frame_id:
                self._active_geometry_token = None
            raise
        age, reason = self._age(identity, now_ns, now_clock_id, self.max_frame_age_ns)
        if reason:
            return self._invalid(identity, reason, age)
        if self._discontinuity:
            return self._invalid(identity, "context_discontinuity_reset_required", age)
        if self._context is not None and identity.context != self._context:
            return self._invalid(identity, "context_changed_reset_required", age)
        calibration_signature = (depth.shape, k.tobytes(), frame.depth_representation)
        if (self._calibration_signature is not None and
                calibration_signature != self._calibration_signature):
            self._discontinuity = True
            return self._invalid(identity, "calibration_content_changed_reset_required", age)
        if self._last_frame_id is not None and identity.frame_id <= self._last_frame_id:
            return self._invalid(identity, "duplicate_geometry", age)
        if self._last_ns is not None and identity.acquired_ns <= self._last_ns:
            return self._invalid(identity, "late_geometry", age)
        pose = frame.pose
        if (not pose.valid or pose.clock_id != identity.clock_id or
                pose.local_frame_id != identity.local_frame_id or
                pose.pose_epoch != identity.pose_epoch):
            return self._invalid(identity, "invalid_capture_pose", age)
        pose_age = identity.acquired_ns - pose.stamp_ns
        if pose_age > self.max_pose_age_ns or pose_age < -self.future_tolerance_ns:
            return self._invalid(identity, "pose_outside_capture_window", age)

        self._context = identity.context
        self._calibration_signature = calibration_signature
        self._last_ns = identity.acquired_ns
        self._last_frame_id = identity.frame_id
        # Acceptance of a newer acquisition supersedes previous live geometry,
        # even if this frame has no valid depth or the backend later fails.
        self._active_geometry_token = None
        # 10-26 owner：重成功率，放宽自「整帧无有效深度就不发请求」：the frame still goes to the backend with every
        # pixel invalid (depth NaN; a remote backend gets all-zero codes), so the PL core runs, carves no ray, leaves
        # every cell UNKNOWN and its completion edge keeps the motion permit alive -- before, the permit lapsed
        # (200 ms), the MCU latched and the car could not even be driven back by remote control. The output is
        # unchanged: no geometry, reason "no_valid_depth" (the console's "相机前方过近" prompt reads that word).
        all_invalid = not np.any(np.isfinite(depth) & (depth > .1) & (depth < 10))
        if all_invalid:
            depth = np.full_like(depth, np.nan)
            self.blank_requests += 1
            if self.blank_requests == 1 or self.blank_requests % 100 == 0:
                logging.getLogger(__name__).warning(
                    "depth frame %s has no valid pixel: blank geometry request sent (%d so far)",
                    identity.frame_id, self.blank_requests)
        grid = self.grid
        if self.grid_policy is not None:
            try:
                grid = self.grid_policy.grid_for(t.copy())
                if (not isinstance(grid, GridSpec) or grid.voxel_m != self.grid.voxel_m or
                        tuple(grid.shape) != tuple(self.grid.shape)):
                    raise ValueError("grid policy changed voxel or shape")
            except (ValueError, OverflowError) as exc:
                if all_invalid:
                    self.blank_request_failures += 1
                    return self._invalid(identity, "no_valid_depth", age)
                return self._invalid(identity, f"grid_policy_failure:{type(exc).__name__}", age)
        try:
            call = dict(
                depth_m=depth, k=k, camera_to_world=t,
                grid_origin_m=grid.origin_m, voxel_size_m=grid.voxel_m,
                shape=grid.shape, frame_id=identity.frame_id,
                capture_time_ns=identity.acquired_ns, pose_time_ns=pose.stamp_ns,
                calibration_version=identity.calibration_id, pose_confirmed=True)
            # A remote backend (host/kv_link.py) puts the whole capture
            # identity on the wire and checks the echo; local backends never
            # see this argument.
            if getattr(self.backend, "accepts_capture_identity", False) is True:
                call["identity"] = identity
                if all_invalid:     # 10-26 #2: all-zero camera codes (code 0 = no return; crc matches NaN depth)
                    call["codes"] = np.zeros(depth.shape, dtype=np.uint16)
            result = self.backend.frame_from_depth(**call)
            if all_invalid:
                return self._invalid(identity, "no_valid_depth", age)
            flags = np.asarray(result.flags)
            n = math.prod(grid.shape)
            if (flags.shape != grid.shape[::-1] or flags.size != n or flags.dtype != np.uint8 or
                    not np.isin(flags, [0, 1, 2]).all()):
                raise ValueError("voxel backend returned invalid flags")
            valid_samples = result.accepted_rays
            invalid_samples = result.invalid_depth_or_projection
            # Endpoint outside the grid; with the frame kernel these rays are
            # clipped and walked (M7_CLIP_SPEC_20261011), key name unchanged.
            outside_samples = result.outside_grid
            if (any(type(n) is not int or n < 0 or n > depth.size for n in
                    (valid_samples, invalid_samples, outside_samples)) or
                    valid_samples + invalid_samples + outside_samples != depth.size or
                    result.frame_id != identity.frame_id or
                    result.capture_time_ns != identity.acquired_ns or
                    result.pose_time_ns != pose.stamp_ns or
                    result.calibration_version != identity.calibration_id):
                raise ValueError("voxel backend returned mismatched frame or non-conservative counts")
        except (RuntimeError, ValueError, OverflowError) as exc:
            if all_invalid:
                self.blank_request_failures += 1
                return self._invalid(identity, "no_valid_depth", age)
            return self._invalid(identity, f"backend_failure:{type(exc).__name__}", age)
        if valid_samples == 0:
            return self._invalid(identity, "no_valid_grid_samples", age)
        flags = flags.copy()
        token = uuid.uuid4().hex
        self._active_geometry_token = token
        self._cache[identity] = _CachedFrame(depth, k, t, token)
        while len(self._cache) > self.max_frames:
            self._cache.popitem(last=False)
        counts = {"unknown_voxels": int(np.count_nonzero(flags == 0)),
                  "free_voxels": int(np.count_nonzero(flags == 1)),
                  "hit_voxels": int(np.count_nonzero(flags == 2)),
                  "valid_grid_samples": valid_samples,
                  "input_pixels": depth.size,
                  "invalid_depth_or_projection": invalid_samples,
                  "outside_grid": outside_samples}
        navigation = ConsumerOutput(identity, True, "geometry_ready", age,
                                    backend=self.backend_label,
                                    expires_at_ns=identity.acquired_ns + self.max_frame_age_ns,
                                    frame_token=token, generation=self._generation,
                                    channel="navigation",
                                    payload={"grid": grid, "voxel_flags": flags.copy(), **counts})
        nearest = float(np.min(depth[np.isfinite(depth) & (depth > .1) & (depth < 10)]))
        display = ConsumerOutput(identity, True, "display_input_ready", age,
                                 backend=self.backend_label,
                                 expires_at_ns=identity.acquired_ns + self.max_frame_age_ns,
                                 frame_token=token, generation=self._generation,
                                 channel="display",
                                 payload={"nearest_optical_z_m": nearest,
                                          "occupied_voxels": counts["hit_voxels"],
                                          "valid_grid_samples": valid_samples,
                                          "input_pixels": depth.size,
                                          "invalid_depth_or_projection": invalid_samples,
                                          "outside_grid": outside_samples,
                                          "voxel_flags": flags.copy(),
                                          "risk_status": "not_computed",
                                          "stage": "board_overlay_input_only"})
        return GeometryOutputs(navigation, display)

    def ingest_semantic(self, packet: SemanticPacket, *, now_ns: int, now_clock_id: str):
        if not isinstance(packet, SemanticPacket):
            raise ValueError("SemanticPacket required")
        identity = packet.identity
        _identity_text(packet.object_id, "object_id")
        _identity_text(packet.frame_token, "frame_token")
        age, reason = self._age(identity, now_ns, now_clock_id, self.max_cache_age_ns)
        if reason is not None:
            return self._invalid_semantic(identity, reason, age)
        if self._discontinuity or identity.context != self._context:
            return self._invalid_semantic(identity, "context_changed_reset_required", age)
        saved = self._cache.get(identity)
        if saved is None:
            return self._invalid_semantic(identity, "frame_not_cached", age)
        if saved.token != packet.frame_token:
            return self._invalid_semantic(identity, "frame_token_mismatch", age)
        if packet.object_id in saved.used_objects:
            return self._invalid_semantic(identity, "duplicate_object_frame", age)
        if len(saved.used_objects) >= self.max_objects_per_frame:
            return self._invalid_semantic(identity, "object_capacity_exceeded", age)
        if packet.mask is not None:
            mask = np.asarray(packet.mask)
            if mask.shape != saved.depth_m.shape or mask.dtype != np.bool_:
                return self._invalid_semantic(identity, "mask_resolution_or_type_mismatch", age)
            mask = mask.copy()
            mask_pixels = int(np.count_nonzero(mask))
            mask_content_sha256 = hashlib.sha256(mask.astype(np.uint8).tobytes()).hexdigest()
        else:
            mask = None
            mask_pixels = None
            mask_content_sha256 = None
        bounds = packet.historical_bounds_local_m
        if bounds is not None:
            bounds = np.asarray(bounds, dtype=np.float64)
            if (bounds.shape != (2, 3) or not np.isfinite(bounds).all()
                    or np.any(bounds[0] >= bounds[1])):
                return self._invalid_semantic(identity, "invalid_historical_bounds", age)

        if self.object_backend is None:
            candidate, reason, details = self._candidate(saved, mask, bounds)
        else:
            try:
                candidate, reason, details = self.object_backend.candidate(
                    saved.depth_m.copy(), saved.K.copy(), saved.pose.copy(),
                    None if mask is None else mask.copy(),
                    None if bounds is None else bounds.copy(), self.cfg)
                if (candidate not in ("indeterminate", "support_present", "free_space_conflict")
                        or not isinstance(reason, str) or not reason or
                        not isinstance(details, dict) or
                        any(key in details for key in ("object_id", "candidate", "association",
                                                    "mask_pixels", "mask_content_sha256"))):
                    raise ValueError("object backend returned an invalid candidate")
            except Exception as exc:
                return self._invalid_semantic(identity,
                    "object_backend_failure:%s" % type(exc).__name__, age)
        saved.used_objects.add(packet.object_id)
        return ConsumerOutput(identity, True, reason, age,
                              backend=self.object_backend_label,
                              expires_at_ns=identity.acquired_ns + self.max_cache_age_ns,
                              frame_token=saved.token, generation=self._generation,
                              channel="memory",
                              payload={"object_id": packet.object_id,
                                       "candidate": candidate,
                                       "association": "upstream_object_id_unverified",
                                       "mask_pixels": mask_pixels,
                                       "mask_content_sha256": mask_content_sha256,
                                       **details})

    def _candidate(self, saved, mask, bounds):
        cfg = self.cfg
        if mask is None:
            return "indeterminate", "missing_mask", {}
        valid = np.isfinite(saved.depth_m) & (saved.depth_m > .1) & (saved.depth_m < 10)
        if mask.any():
            n = int(np.count_nonzero(mask & valid))
            fraction = n / int(np.count_nonzero(mask))
            if n < cfg.min_pixels or fraction < cfg.min_valid_fraction:
                return "indeterminate", "invalid_mask_depth", {"mask_valid_pixels": n}
            v, u = np.nonzero(mask & valid)
            uvz = np.column_stack((u, v, saved.depth_m[v, u]))
            rays = np.column_stack((uvz[:, :2], np.ones(n))) @ np.linalg.inv(saved.K).T
            xyz = (rays * uvz[:, 2:]) @ saved.pose[:3, :3].T + saved.pose[:3, 3]
            median = np.median(xyz, axis=0)
            if bounds is not None and (np.any(median < bounds[0] - cfg.association_m) or
                                       np.any(median > bounds[1] + cfg.association_m)):
                return "indeterminate", "mask_not_in_historical_bounds", {
                    "mask_valid_pixels": n, "position_local_m": median.tolist()}
            return "support_present", "matched_mask_depth", {
                "mask_valid_pixels": n, "valid_fraction": fraction,
                "position_local_m": median.tolist()}
        if bounds is None:
            return "indeterminate", "no_historical_bounds", {}
        corners = np.array([[x, y, z] for x in bounds[:, 0]
                            for y in bounds[:, 1] for z in bounds[:, 2]])
        camera = (corners - saved.pose[:3, 3]) @ saved.pose[:3, :3]
        if (camera[:, 2] <= .1).any():
            return "indeterminate", "historical_region_out_of_view", {}
        projected = camera @ saved.K.T
        uv = projected[:, :2] / projected[:, 2:]
        lo = np.floor(uv.min(axis=0)).astype(int)
        hi = np.ceil(uv.max(axis=0)).astype(int)
        h, w = saved.depth_m.shape
        if lo[0] < 0 or lo[1] < 0 or hi[0] >= w or hi[1] >= h:
            return "indeterminate", "historical_region_out_of_view", {}
        if np.any(hi - lo < 3):
            return "indeterminate", "historical_region_too_small", {}
        patch = saved.depth_m[lo[1]:hi[1] + 1, lo[0]:hi[0] + 1]
        good = np.isfinite(patch) & (patch > .1) & (patch < 10)
        n = int(good.sum())
        if n < cfg.min_pixels or good.mean() < cfg.min_valid_fraction:
            return "indeterminate", "invalid_historical_region_depth", {"region_valid_pixels": n}
        d = patch[good]
        if np.mean(d < camera[:, 2].min() - cfg.depth_margin_m) >= .5:
            return "indeterminate", "historical_region_occluded", {"region_valid_pixels": n}
        if np.mean(d > camera[:, 2].max() + cfg.depth_margin_m) >= cfg.free_fraction:
            return "free_space_conflict", "conservative_free_depth", {"region_valid_pixels": n}
        return "indeterminate", "surface_or_uncertain", {"region_valid_pixels": n}

    def ingest_region_checks(self, packet: RegionCheckPacket, *, now_ns: int, now_clock_id: str):
        """Mask-free region checks; one channel="region" output per entry.

        Requires an explicit region_backend (no silent Python fallback).
        Verdicts: occluded / free_space_conflict / indeterminate, plus
        core.py's geometry_evidence word. Never emits a "candidate" key.
        """
        if self.region_backend is None:
            raise ValueError("region checks require an explicit region_backend")
        if not isinstance(packet, RegionCheckPacket):
            raise ValueError("RegionCheckPacket required")
        identity = packet.identity
        _identity_text(packet.frame_token, "frame_token")
        if not isinstance(packet.entries, tuple) or not packet.entries:
            raise ValueError("entries must be a nonempty tuple")
        age, reason = self._age(identity, now_ns, now_clock_id, self.max_cache_age_ns)
        if reason is None and (self._discontinuity or identity.context != self._context):
            reason = "context_changed_reset_required"
        saved = None
        if reason is None:
            saved = self._cache.get(identity)
            if saved is None:
                reason = "frame_not_cached"
            elif saved.token != packet.frame_token:
                reason = "frame_token_mismatch"
        if reason is not None:
            # One failure per entry, so callers can align outputs with entries.
            return [self._invalid_region(identity, reason, age,
                                         e[0] if isinstance(e, tuple) and e else None)
                    for e in packet.entries]
        outputs, accepted = [], []
        seen = set()
        for entry in packet.entries:
            if not isinstance(entry, tuple) or len(entry) != 2:
                raise ValueError("each entry is (object_id, bounds)")
            object_id, bounds = entry
            _identity_text(object_id, "object_id")
            if object_id in saved.used_region_objects or object_id in seen:
                outputs.append(self._invalid_region(identity, "duplicate_object_frame", age, object_id))
                continue
            if len(saved.used_region_objects) + len(accepted) >= self.max_objects_per_frame:
                outputs.append(self._invalid_region(identity, "object_capacity_exceeded", age, object_id))
                continue
            bounds = np.asarray(bounds, dtype=np.float64)
            if (bounds.shape != (2, 3) or not np.isfinite(bounds).all()
                    or np.any(bounds[0] >= bounds[1])):
                outputs.append(self._invalid_region(identity, "invalid_historical_bounds", age, object_id))
                continue
            seen.add(object_id)
            accepted.append((len(outputs), object_id, bounds))
            outputs.append(None)
        if not accepted:
            return outputs
        try:
            results = self.region_backend.check_boxes(
                saved.depth_m.copy(), saved.K.copy(), saved.pose.copy(),
                [(object_id, bounds.copy()) for _, object_id, bounds in accepted], self.cfg)
            if len(results) != len(accepted):
                raise ValueError("region backend returned wrong number of results")
            for verdict, evidence, details in results:
                if (verdict not in ("occluded", "free_space_conflict", "indeterminate") or
                        evidence not in ("out_of_view", "too_small", "invalid_depth", "occluded",
                                         "free", "surface_or_uncertain") or
                        {"occluded": "occluded", "free": "free_space_conflict"}
                        .get(evidence, "indeterminate") != verdict or
                        not isinstance(details, dict) or "candidate" in details):
                    raise ValueError("region backend returned an invalid verdict")
        except Exception as exc:
            reason = "region_backend_failure:%s" % type(exc).__name__
            for slot, object_id, _ in accepted:
                outputs[slot] = self._invalid_region(identity, reason, age, object_id)
            return outputs
        for (slot, object_id, _), (verdict, evidence, details) in zip(accepted, results):
            saved.used_region_objects.add(object_id)
            row = details.get("kernel_row", {})
            outputs[slot] = ConsumerOutput(
                identity, True, evidence, age, backend=self.region_backend_label,
                expires_at_ns=identity.acquired_ns + self.max_cache_age_ns,
                frame_token=saved.token, generation=self._generation, channel="region",
                payload={"object_id": object_id, "verdict": verdict,
                         "geometry_evidence": evidence,
                         "roi_xyxy": details.get("roi_xyxy"),
                         "roi_pixels": row.get("roi_pixels"),
                         "roi_valid_pixels": row.get("roi_valid_pixels"),
                         "roi_front_pixels": row.get("roi_front_pixels"),
                         "roi_behind_pixels": row.get("roi_behind_pixels")})
        return outputs
