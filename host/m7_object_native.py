"""Bounded object evidence C ABI; historical projection and median stay on CPU."""
import ctypes
from dataclasses import dataclass, field, replace
import math
from pathlib import Path
import shutil
import subprocess

import numpy as np

from perception.fpga_replay.core import Config


ROOT = Path(__file__).resolve().parents[1]
MAX_WIDTH, MAX_HEIGHT = 1280, 800
MAX_PIXELS = MAX_WIDTH * MAX_HEIGHT
F32 = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
F64 = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
U8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
U32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags="C_CONTIGUOUS")
COUNT_NAMES = ("input_pixels", "mask_pixels", "projected_valid_points",
               "roi_pixels", "roi_valid_pixels", "roi_front_pixels",
               "roi_behind_pixels", "candidate_code")
CANDIDATES = ("indeterminate", "support_present", "free_space_conflict")
# Mask-free multi-box region check (docs/design/OBJECT_ROI_SPEC_20261011.md).
ROI_MAX_BOXES = 32
ROI_FIELDS = ("object_id", "enabled", "roi_pixels", "roi_valid_pixels",
              "roi_front_pixels", "roi_behind_pixels", "verdict_code")
ROI_VERDICTS = {0: "indeterminate", 2: "free_space_conflict", 3: "occluded"}


def compile_object_native(output):
    compiler = shutil.which("g++")
    if compiler is None:
        raise RuntimeError("g++ required for object-evidence reference")
    output = Path(output).resolve()
    if not output.parent.is_dir() or output.exists():
        raise ValueError("choose a fresh library path in an existing directory")
    subprocess.run([compiler, "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-ffp-contract=off", "-fPIC", "-shared",
                    str(ROOT / "hw/m7/object_evidence.cpp"), "-o", str(output)],
                   check=True, capture_output=True, text=True)
    return output


def check_roi_row(values, row_id, enabled, rect, cfg):
    """The host's per-row check of a region-kernel result (one dict of ROI_FIELDS).

    Echoed id/enabled, verdict code in ROI_VERDICTS, monotone counts,
    PIXELS == rectangle area for an enabled box, all-zero disabled row, and the
    verdict recomputed from the returned integers. Raises RuntimeError. Used by
    ``evaluate_boxes`` (local core) and by host/kv_link.py (KV260 answers).
    """
    n, valid = values["roi_pixels"], values["roi_valid_pixels"]
    front, behind = values["roi_front_pixels"], values["roi_behind_pixels"]
    if (values["object_id"] != row_id or values["enabled"] != enabled or
            values["verdict_code"] not in ROI_VERDICTS or
            not 0 <= front + behind <= valid <= n or
            (enabled and n != (rect[2] - rect[0] + 1) * (rect[3] - rect[1] + 1)) or
            (not enabled and (n or values["verdict_code"]))):
        raise RuntimeError("region kernel returned inconsistent row")
    # Recompute the verdict from the returned integers (same Python
    # true division the software path uses); any mismatch is fatal.
    expected = 0
    if enabled and valid >= cfg.min_pixels and valid / n >= cfg.min_valid_fraction:
        if front / valid >= .5:
            expected = 3
        elif behind / valid >= cfg.free_fraction:
            expected = 2
    if expected != values["verdict_code"]:
        raise RuntimeError("region kernel verdict disagrees with its counts")


@dataclass(frozen=True)
class RawObjectEvidence:
    counts: dict
    points_local_m: np.ndarray  # compact row-major masked valid points

    @property
    def candidate(self):
        return CANDIDATES[self.counts["candidate_code"]]


class NativeObjectEvidence:
    def __init__(self, library):
        self.lib = ctypes.CDLL(str(Path(library).resolve()))
        self.lib.fpga_m7_object_evidence.argtypes = [
            F32, U8, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
            ctypes.c_uint32, U32, ctypes.c_double, ctypes.c_double,
            ctypes.c_double, F64, F64, ctypes.c_uint32, ctypes.c_double,
            ctypes.c_double, F64, U32, U32]
        self.lib.fpga_m7_object_evidence.restype = None
        self.lib.fpga_m7_object_roi_batch.argtypes = [
            F32, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, U32, U32,
            F64, F64, U32, ctypes.c_double, ctypes.c_uint32, ctypes.c_double,
            ctypes.c_double, U32, U32]
        self.lib.fpga_m7_object_roi_batch.restype = None

    def evaluate(self, depth_m, mask, k, local_camera, *, roi=None, near_z=0.,
                 far_z=0., cfg=Config()):
        """Run one original frame; float64 depth is explicitly unsupported."""
        depth = np.asarray(depth_m)
        if depth.ndim != 2 or depth.dtype != np.float32:
            raise ValueError("native object backend requires float32 HxW optical-Z metres")
        height, width = depth.shape
        if not 1 <= width <= MAX_WIDTH or not 1 <= height <= MAX_HEIGHT:
            raise ValueError("depth resolution exceeds 1280x800 capacity")
        depth = np.ascontiguousarray(depth)
        if mask is None:
            mask_present = 0
            binary_mask = np.zeros(depth.shape, dtype=np.uint8)
        else:
            image_mask = np.asarray(mask)
            if image_mask.shape != depth.shape or image_mask.dtype not in (np.bool_, np.uint8):
                raise ValueError("mask must be aligned bool/uint8 original-frame pixels")
            mask_present = 1
            binary_mask = np.ascontiguousarray(image_mask, dtype=np.uint8)
        intrinsic = np.asarray(k, dtype=np.float64)
        transform = np.asarray(local_camera, dtype=np.float64)
        if (intrinsic.shape != (3, 3) or transform.shape != (4, 4) or
                not np.isfinite(intrinsic).all() or not np.isfinite(transform).all() or
                intrinsic[0, 0] <= 0 or intrinsic[1, 1] <= 0 or
                abs(intrinsic[1, 0]) > 1e-8 or
                not np.allclose(intrinsic[2], [0, 0, 1], atol=1e-8, rtol=0) or
                not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8, rtol=0) or
                not np.allclose(transform[:3, :3].T @ transform[:3, :3],
                                np.eye(3), atol=1e-5, rtol=0) or
                not np.isclose(np.linalg.det(transform[:3, :3]), 1., atol=1e-5, rtol=0)):
            raise ValueError("invalid intrinsics or rigid local-camera pose")
        inverse_k = np.ascontiguousarray(np.linalg.inv(intrinsic), dtype=np.float64)
        t = np.ascontiguousarray(transform[:3], dtype=np.float64)
        if roi is None:
            roi_enabled = 0
            rectangle = np.zeros(4, dtype=np.uint32)
        else:
            if (len(roi) != 4 or any(isinstance(n, (bool, np.bool_)) or
                    not isinstance(n, (int, np.integer)) for n in roi)):
                raise ValueError("ROI must be four integer coordinates")
            x0, y0, x1, y1 = map(int, roi)
            if not (0 <= x0 < x1 < width and 0 <= y0 < y1 < height and
                    x1 - x0 >= 3 and y1 - y0 >= 3):
                raise ValueError("ROI must be fully visible and span at least four pixels")
            if not np.isfinite([near_z, far_z]).all() or near_z <= .1 or far_z < near_z:
                raise ValueError("invalid historical camera-Z extent")
            roi_enabled = 1
            rectangle = np.ascontiguousarray([x0, y0, x1, y1], dtype=np.uint32)
        if (not isinstance(cfg, Config) or type(cfg.min_pixels) is not int or
                not 9 <= cfg.min_pixels <= MAX_PIXELS or
                not np.isfinite([cfg.min_valid_fraction, cfg.free_fraction,
                                 cfg.depth_margin_m]).all() or
                not .7 <= cfg.min_valid_fraction <= 1. or
                not .9 <= cfg.free_fraction <= 1. or
                cfg.depth_margin_m < .04):
            raise ValueError("Config thresholds must be representable and no looser than defaults")
        points = np.full(3 * MAX_PIXELS, np.nan, dtype=np.float64)
        counts = np.full(8, np.iinfo(np.uint32).max, dtype=np.uint32)
        status = np.full(1, np.iinfo(np.uint32).max, dtype=np.uint32)
        self.lib.fpga_m7_object_evidence(
            depth, binary_mask, width, height, mask_present, roi_enabled,
            rectangle, float(near_z), float(far_z), float(cfg.depth_margin_m),
            inverse_k, t, cfg.min_pixels, float(cfg.min_valid_fraction),
            float(cfg.free_fraction), points, counts, status)
        if status[0] != 0:
            if not np.all(counts == np.iinfo(np.uint32).max) or not np.isnan(points).all():
                raise RuntimeError("rejected object batch modified outputs")
            raise ValueError("object kernel rejected request status %d" % status[0])
        values = [int(x) for x in counts]
        if (values[0] != depth.size or
                not 0 <= values[2] <= values[1] <= values[0] or
                not 0 <= values[4] <= values[3] <= values[0] or
                not 0 <= values[5] <= values[4] or
                not 0 <= values[6] <= values[4] or
                values[5] + values[6] > values[4] or
                values[7] not in (0, 1, 2)):
            raise RuntimeError("object kernel returned inconsistent counts")
        compact = points[:3 * values[2]].reshape(values[2], 3).copy()
        if not np.isfinite(compact).all():
            raise RuntimeError("object kernel returned nonfinite points")
        return RawObjectEvidence(dict(zip(COUNT_NAMES, values)), compact)

    def candidate(self, depth_m, k, pose, mask, bounds, cfg):
        """Reuse old ROI, median and association rules around the C kernel."""
        roi = None
        near_z = far_z = 0.
        geometry_reason = None
        if bounds is not None:
            corners = np.array([[x, y, z] for x in bounds[:, 0]
                                for y in bounds[:, 1] for z in bounds[:, 2]])
            camera = (corners - pose[:3, 3]) @ pose[:3, :3]
            if (camera[:, 2] <= .1).any():
                geometry_reason = "historical_region_out_of_view"
            else:
                uvz = camera @ k.T
                uv = uvz[:, :2] / uvz[:, 2:]
                lo = np.floor(uv.min(axis=0)).astype(int)
                hi = np.ceil(uv.max(axis=0)).astype(int)
                height, width = depth_m.shape
                if lo[0] < 0 or lo[1] < 0 or hi[0] >= width or hi[1] >= height:
                    geometry_reason = "historical_region_out_of_view"
                elif np.any(hi - lo < 3):
                    geometry_reason = "historical_region_too_small"
                else:
                    roi = (int(lo[0]), int(lo[1]), int(hi[0]), int(hi[1]))
                    near_z = float(camera[:, 2].min())
                    far_z = float(camera[:, 2].max())
        raw = self.evaluate(depth_m, mask, k, pose, roi=roi,
                            near_z=near_z, far_z=far_z, cfg=cfg)
        details = {"kernel_counts": raw.counts.copy(),
                   "position_summary_method": "cpu_median_from_kernel_points"}
        if mask is None:
            return "indeterminate", "missing_mask", details
        # The consumer divides mask_valid_pixels by the host's own mask pixel
        # count. The kernel's denominator must therefore be that same integer.
        if raw.counts["mask_pixels"] != int(np.count_nonzero(mask)):
            raise RuntimeError("object kernel mask count disagrees with host mask")
        if np.any(mask):
            count = raw.counts["projected_valid_points"]
            details["mask_valid_pixels"] = count
            if raw.candidate != "support_present":
                return "indeterminate", "invalid_mask_depth", details
            median = np.median(raw.points_local_m, axis=0)
            details["position_local_m"] = median.tolist()
            if bounds is not None and (np.any(median < bounds[0] - cfg.association_m) or
                                       np.any(median > bounds[1] + cfg.association_m)):
                return "indeterminate", "mask_not_in_historical_bounds", details
            # PL returns integer counts only; the ratio is formed here from the
            # same two integers with Python true division, exactly as the
            # software path and the memory consumer do. No fixed-point ratio.
            details["valid_fraction"] = count / raw.counts["mask_pixels"]
            return "support_present", "matched_mask_depth", details
        if bounds is None:
            return "indeterminate", "no_historical_bounds", details
        if geometry_reason:
            return "indeterminate", geometry_reason, details
        valid = raw.counts["roi_valid_pixels"]
        total = raw.counts["roi_pixels"]
        details["region_valid_pixels"] = valid
        if valid < cfg.min_pixels or valid / total < cfg.min_valid_fraction:
            return "indeterminate", "invalid_historical_region_depth", details
        if raw.counts["roi_front_pixels"] / valid >= .5:
            return "indeterminate", "historical_region_occluded", details
        if raw.candidate == "free_space_conflict":
            return "free_space_conflict", "conservative_free_depth", details
        return "indeterminate", "surface_or_uncertain", details

    @staticmethod
    def _check_cfg(cfg):
        if (not isinstance(cfg, Config) or type(cfg.min_pixels) is not int or
                not 9 <= cfg.min_pixels <= MAX_PIXELS or
                not np.isfinite([cfg.min_valid_fraction, cfg.free_fraction,
                                 cfg.depth_margin_m]).all() or
                not .7 <= cfg.min_valid_fraction <= 1. or
                not .9 <= cfg.free_fraction <= 1. or
                cfg.depth_margin_m < .04):
            raise ValueError("Config thresholds must be representable and no looser than defaults")

    def evaluate_boxes(self, depth_m, boxes, cfg=Config()):
        """One mask-free pass over up to ROI_MAX_BOXES boxes.

        ``boxes`` is a sequence of (row_id, roi_or_None, near_z, far_z); a
        None ROI is a host-disabled box. Returns one dict per box.
        """
        depth = np.asarray(depth_m)
        if depth.ndim != 2 or depth.dtype != np.float32:
            raise ValueError("native object backend requires float32 HxW optical-Z metres")
        height, width = depth.shape
        if not 1 <= width <= MAX_WIDTH or not 1 <= height <= MAX_HEIGHT:
            raise ValueError("depth resolution exceeds 1280x800 capacity")
        depth = np.ascontiguousarray(depth)
        if not 1 <= len(boxes) <= ROI_MAX_BOXES:
            raise ValueError("region batch must hold 1..%d boxes" % ROI_MAX_BOXES)
        self._check_cfg(cfg)
        enabled = np.zeros(ROI_MAX_BOXES, dtype=np.uint32)
        rectangles = np.zeros(4 * ROI_MAX_BOXES, dtype=np.uint32)
        near = np.zeros(ROI_MAX_BOXES, dtype=np.float64)
        far = np.zeros(ROI_MAX_BOXES, dtype=np.float64)
        ids = np.zeros(ROI_MAX_BOXES, dtype=np.uint32)
        for b, (row_id, roi, near_z, far_z) in enumerate(boxes):
            if type(row_id) is not int or not 0 <= row_id <= 0xffffffff:
                raise ValueError("row id must be uint32")
            ids[b] = row_id
            if roi is None:
                continue
            if (len(roi) != 4 or any(isinstance(n, (bool, np.bool_)) or
                    not isinstance(n, (int, np.integer)) for n in roi)):
                raise ValueError("ROI must be four integer coordinates")
            x0, y0, x1, y1 = map(int, roi)
            if not (0 <= x0 < x1 < width and 0 <= y0 < y1 < height and
                    x1 - x0 >= 3 and y1 - y0 >= 3):
                raise ValueError("ROI must be fully visible and span at least four pixels")
            if not np.isfinite([near_z, far_z]).all() or near_z <= .1 or far_z < near_z:
                raise ValueError("invalid historical camera-Z extent")
            enabled[b] = 1
            rectangles[4 * b:4 * b + 4] = (x0, y0, x1, y1)
            near[b], far[b] = float(near_z), float(far_z)
        result = np.full(len(ROI_FIELDS) * ROI_MAX_BOXES, np.iinfo(np.uint32).max,
                         dtype=np.uint32)
        status = np.full(1, np.iinfo(np.uint32).max, dtype=np.uint32)
        self.lib.fpga_m7_object_roi_batch(
            depth, width, height, len(boxes), enabled, rectangles, near, far, ids,
            float(cfg.depth_margin_m), cfg.min_pixels, float(cfg.min_valid_fraction),
            float(cfg.free_fraction), result, status)
        if status[0] != 0:
            if not np.all(result == np.iinfo(np.uint32).max):
                raise RuntimeError("rejected region batch modified outputs")
            raise ValueError("region kernel rejected request status %d" % status[0])
        rows = result.reshape(ROI_MAX_BOXES, len(ROI_FIELDS))
        if not np.all(rows[len(boxes):] == np.iinfo(np.uint32).max):
            raise RuntimeError("region kernel wrote rows past box_count")
        out = []
        for b in range(len(boxes)):
            values = dict(zip(ROI_FIELDS, (int(v) for v in rows[b])))
            check_roi_row(values, int(ids[b]), int(enabled[b]),
                          tuple(int(v) for v in rectangles[4 * b:4 * b + 4]), cfg)
            out.append(values)
        return out

    def check_boxes(self, depth_m, k, pose, entries, cfg=Config(), ground_z_m=None):
        """Project historical bounds on CPU, then one kernel pass per <=N boxes.

        ``entries`` is a sequence of (object_id, bounds(2,3)). Returns a list
        of (verdict, geometry_evidence, details) in input order, where
        geometry_evidence uses compute_geometry's vocabulary. The projection is
        ``plan_boxes``'s (its ``plans`` list, all entries in input order with the
        host-disabled ones kept); every box goes to the kernel, chunked by N.
        """
        depth = np.asarray(depth_m)
        if depth.ndim != 2:
            raise ValueError("depth must be HxW")
        plans = plan_boxes(depth.shape, k, pose, entries, cfg, ground_z_m=ground_z_m).plans
        v2 = bool(getattr(cfg, "geometry_v2", False))
        if v2:
            from perception.fpga_replay.geometry_v2 import word_from_counts
        results = []
        for start in range(0, len(plans), ROI_MAX_BOXES):
            chunk = plans[start:start + ROI_MAX_BOXES]
            rows = self.evaluate_boxes(
                depth, [(i, p[2], p[3], p[4]) for i, p in enumerate(chunk)], cfg)
            for plan, row in zip(chunk, rows):
                object_id, host_reason, roi = plan[0], plan[1], plan[2]
                verdict = ROI_VERDICTS[row["verdict_code"]]
                if host_reason is not None:
                    evidence = host_reason
                elif v2:
                    # 10-27 v2: the word from the counts (50 % rule); the kernel's code stays its 0.9 echo
                    evidence = word_from_counts(row["roi_pixels"], row["roi_valid_pixels"],
                                                row["roi_front_pixels"], row["roi_behind_pixels"], cfg)
                elif (row["roi_valid_pixels"] < cfg.min_pixels or
                      row["roi_valid_pixels"] / row["roi_pixels"] < cfg.min_valid_fraction):
                    evidence = "invalid_depth"
                elif verdict == "occluded":
                    evidence = "occluded"
                elif verdict == "free_space_conflict":
                    evidence = "free"
                else:
                    evidence = "surface_or_uncertain"
                if not v2 and ({"occluded": "occluded", "free": "free_space_conflict"}
                        .get(evidence, "indeterminate") != verdict):
                    raise RuntimeError("region verdict disagrees with host sub-reason")
                details = {"kernel_row": row, "roi_xyxy": None if roi is None else list(roi),
                           "object_id": object_id}
                results.append((verdict, evidence, details))
        return results


# ---------------------------------------------------------------- box planning
@dataclass(frozen=True)
class RoiRow:
    """One box sent to a region core. ``row_id`` is the host's row label (echoed)."""
    row_id: int
    enabled: bool
    xyxy: object          # (x0, y0, x1, y1) closed pixel rectangle, or None when disabled
    near_z: float
    far_z: float


@dataclass(frozen=True)
class RoiBatch:
    """Boxes of one frame. ``memory_revision`` is the /fpga/memory revision the
    projection used (``plan_boxes`` leaves 0; set it with ``with_revision``)."""
    memory_revision: int
    cfg: Config
    rows: list

    def with_revision(self, memory_revision):
        return replace(self, memory_revision=int(memory_revision))


@dataclass(frozen=True)
class PlanResult:
    batch: object             # RoiBatch, or None when no box is enabled
    host_resolved: dict       # track_id -> "out_of_view" | "too_small" (decided on the host)
    overflow: list            # track_ids beyond max_boxes, in priority order (never re-sent)
    row_map: dict             # row_id -> track_id for the rows in ``batch``
    # every entry in input order: (track_id, host_reason, roi, near_z, far_z); check_boxes uses it
    plans: list = field(default_factory=list)


def _project_bounds(transform, intrinsic, width, height, bounds):
    """(host_reason, roi, near_z, far_z) of one memory box at this camera pose."""
    corners = np.array([[x, y, z] for x in bounds[:, 0]
                        for y in bounds[:, 1] for z in bounds[:, 2]])
    camera = (corners - transform[:3, 3]) @ transform[:3, :3]
    if (camera[:, 2] <= .1).any():
        return "out_of_view", None, 0., 0.
    uvz = camera @ intrinsic.T
    uv = uvz[:, :2] / uvz[:, 2:]
    lo = np.floor(uv.min(axis=0)).astype(int)
    hi = np.ceil(uv.max(axis=0)).astype(int)
    if lo[0] < 0 or lo[1] < 0 or hi[0] >= width or hi[1] >= height:
        return "out_of_view", None, 0., 0.
    if np.any(hi - lo < 3):
        return "too_small", None, 0., 0.
    return (None, (int(lo[0]), int(lo[1]), int(hi[0]), int(hi[1])),
            float(camera[:, 2].min()), float(camera[:, 2].max()))


def plan_boxes(depth_shape, K, T_world_camera, entries, cfg, max_boxes=ROI_MAX_BOXES,
               priority=None, ground_z_m=None):
    """Project every memory box onto this frame's pose and choose what to send.

    ``entries``: sequence of (track_id, bounds(2,3) world lower/upper corners).
    Host-decidable boxes (out of view, too small, a corner at/behind 0.1 m) go
    to ``host_resolved`` and take no row. The rest are ordered by ``priority``
    and the first ``max_boxes`` become rows 0..n-1 of ``batch``; the others are
    listed in ``overflow`` (the caller must not send a second request).

    ``priority``: None keeps the input order; otherwise a callable
    ``priority(track_id, camera_position_world(3,)) -> sort key`` (ascending,
    stable); ``memory_priority`` builds the A.2/O11 one from /fpga/memory tracks.

    10-27 ``cfg.geometry_v2``: each box is planned by perception/fpga_replay/geometry_v2.plan_one
    (probe region + surface thresholds); an entry may then carry a third item, the track's
    position (world), and ``ground_z_m`` (None: cfg.ground_z_m) raises the probe off the ground.
    """
    if len(depth_shape) != 2:
        raise ValueError("depth must be HxW")
    height, width = int(depth_shape[0]), int(depth_shape[1])
    intrinsic = np.asarray(K, dtype=np.float64)
    transform = np.asarray(T_world_camera, dtype=np.float64)
    if intrinsic.shape != (3, 3) or transform.shape != (4, 4):
        raise ValueError("invalid intrinsics or pose shape")
    max_boxes = int(max_boxes)
    if not 1 <= max_boxes <= ROI_MAX_BOXES:
        raise ValueError("max_boxes must be in 1..%d" % ROI_MAX_BOXES)
    plans = []
    v2 = bool(getattr(cfg, "geometry_v2", False))
    if v2:
        from perception.fpga_replay import geometry_v2
        ground = cfg.ground_z_m if ground_z_m is None else float(ground_z_m)
    for entry in entries:
        object_id, bounds = entry[0], entry[1]
        bounds = np.asarray(bounds, dtype=np.float64)
        if bounds.shape != (2, 3) or not np.isfinite(bounds).all():
            raise ValueError("bounds must be finite (2,3) lower/upper")
        if v2:
            position = entry[2] if len(entry) > 2 else None
            if position is not None:
                position = np.asarray(position, dtype=np.float64)
                if position.shape != (3,) or not np.isfinite(position).all():
                    position = None              # unusable position: the box centre (geometry_v2.plan_one)
            plans.append((object_id,) + geometry_v2.plan_one(transform, intrinsic, width, height,
                                                             bounds[0], bounds[1], position, ground, cfg))
        else:
            plans.append((object_id,) + _project_bounds(transform, intrinsic, width, height, bounds))
    host_resolved = {p[0]: p[1] for p in plans if p[1] is not None}
    enabled = [p for p in plans if p[1] is None]
    if priority is not None:
        cam = transform[:3, 3].copy()
        enabled = sorted(enabled, key=lambda p: priority(p[0], cam))
    chosen, rest = enabled[:max_boxes], enabled[max_boxes:]
    rows = [RoiRow(i, True, p[2], p[3], p[4]) for i, p in enumerate(chosen)]
    batch = RoiBatch(0, cfg, rows) if rows else None
    return PlanResult(batch, host_resolved, [p[0] for p in rest],
                      {i: p[0] for i, p in enumerate(chosen)}, plans)


def memory_priority(tracks, target_label=None):
    """A.2 / O11 order for ``plan_boxes``: (1) label == current task target,
    (2) newer ``last_seen`` first, (3) ``position_m`` nearer to the camera first.

    ``tracks``: track_id -> dict with "label", "last_seen" (seconds or ns, any
    monotone number, None = never), "position_m" ([x, y, z] world or None).
    A track missing from ``tracks`` sorts last in every tier.
    """
    def key(track_id, cam):
        t = tracks.get(track_id) or {}
        target = 0 if (target_label is not None and t.get("label") == target_label) else 1
        seen = t.get("last_seen")
        seen_key = (0, -float(seen)) if seen is not None else (1, 0.)
        pos = t.get("position_m")
        dist = (math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(pos, cam)))
                if pos is not None else math.inf)
        return (target, seen_key, dist)
    return key
