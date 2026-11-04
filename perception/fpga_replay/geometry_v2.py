"""Geometry evidence v2 (Config.geometry_v2, docs/design/GEOMETRY_V2_20261027.md).

v1 (core.compute_geometry) asks whether >= 90 % of the valid pixels inside the projected track box
lie 4 cm beyond the box's farthest corner. With the camera 0.39 m above the floor and pitched ~7 deg
down, the box of a floor object holds floor (the box bottom is padded 3 cm below the ground), so
after the object is taken away only ~67 % of its pixels are "behind" -> never free; the same for an
object against a wall (DIAG_NOTES R2). v2, after Panoptic Multi-TSDFs / Khronos:

* probe region: the track box without its 3 cm observation pad, shrunk about its centre to
  ``probe_shrink`` of each extent, its bottom raised to >= ground + ``probe_ground_clear_m``; the
  full padded box must still be wholly in view (else out_of_view, as v1), so an object half out of
  the image is never judged;
* reference: the expected visible-surface depth E = optical-axis depth of the track position (the
  fused median of observed SURFACE points, process_frame -- already the visible surface, so no
  half-thickness is subtracted); behind > E + max(probe_margin_m, 3 sigma(E)), sigma(d) =
  obs_sigma0_m + obs_sigma_k d^2; front (occluder) < E - occlusion_margin_m;
* words: invalid_depth as v1; front >= 50 % of valid -> occluded; behind >= free_fraction_v2 of valid
  -> free; else surface_or_uncertain.

The KV260 region core is unchanged: its row thresholds are front = float32(near_z - margin) and
back = float32(far_z + margin) (hw/m7/object_evidence.cpp, fpga_m7_object_roi_batch), so the Nano
sends near_z = F + margin, far_z = B - margin and recomputes the word from the four returned counts
(``word_from_counts``); the kernel's own verdict code (0.9 rule) is kept only as a consistency echo.
``region_counts`` is the same count on the CPU (float32, the kernel's comparisons), so
compute_geometry and the KV path give the same word for the same depth.

No ROS, no torch; Python 3.8.
"""
import numpy as np

_F32_LO = np.float32(0.1)
_F32_HI = np.float32(10.0)
# smallest near_z the region core accepts (it refuses near_z <= 0.1, status 3 for the whole batch)
NEAR_Z_MIN = float(np.nextafter(0.1, 1.0))


def ground_of(frame, cfg):
    """Ground height as memory_gates uses it: frame.ground_z_m, else cfg.ground_z_m."""
    g = getattr(frame, "ground_z_m", None)
    return cfg.ground_z_m if g is None else float(g)


def _box_corners(lower, upper):
    return np.array([[x, y, z] for x in (lower[0], upper[0])
                     for y in (lower[1], upper[1]) for z in (lower[2], upper[2])], dtype=np.float64)


def _rect(camera, K, width, height):
    """(reason, (x0, y0, x1, y1)) of camera-frame corners, v1's floor/ceil and bounds rules."""
    if (camera[:, 2] <= 0.1).any():
        return "out_of_view", None
    uvz = camera @ np.asarray(K, dtype=np.float64).T
    uv = uvz[:, :2] / uvz[:, 2:]
    lo = np.floor(uv.min(axis=0)).astype(int)
    hi = np.ceil(uv.max(axis=0)).astype(int)
    if lo[0] < 0 or lo[1] < 0 or hi[0] >= width or hi[1] >= height:
        return "out_of_view", None
    if np.any(hi - lo < 3):
        return "too_small", None
    return None, (int(lo[0]), int(lo[1]), int(hi[0]), int(hi[1]))


def probe_box(lower, upper, ground_z_m, cfg):
    """(probe lower, probe upper) in world, or None when nothing of the object is above the
    ground clearance (a flat object on the floor: v2 cannot probe it)."""
    lower = np.asarray(lower, dtype=np.float64); upper = np.asarray(upper, dtype=np.float64)
    lo = lower + cfg.probe_pad_m; hi = upper - cfg.probe_pad_m
    mid = (lower + upper) / 2.0
    lo = np.minimum(lo, mid); hi = np.maximum(hi, mid)      # pad wider than the extent: the centre
    lo[2] = max(lo[2], float(ground_z_m) + cfg.probe_ground_clear_m)
    if hi[2] <= lo[2]:
        return None
    c = (lo + hi) / 2.0; half = (hi - lo) / 2.0 * cfg.probe_shrink
    return c - half, c + half


def thresholds(expected_m, cfg):
    """(front, behind) depth thresholds in metres around the expected surface depth."""
    sigma = cfg.obs_sigma0_m + cfg.obs_sigma_k * expected_m * expected_m
    return expected_m - cfg.occlusion_margin_m, expected_m + max(cfg.probe_margin_m, 3.0 * sigma)


def plan_one(T_world_camera, K, width, height, lower, upper, position, ground_z_m, cfg):
    """(host_reason, roi, near_z, far_z) of one track -- plan_boxes' tuple, v2 rules.

    near_z / far_z are the region core's row inputs: the core compares against
    near_z - cfg.depth_margin_m and far_z + cfg.depth_margin_m."""
    T = np.asarray(T_world_camera, dtype=np.float64)
    full = (_box_corners(lower, upper) - T[:3, 3]) @ T[:3, :3]
    reason, _ = _rect(full, K, width, height)                # whole padded box in view, as v1
    if reason is not None:
        return reason, None, 0., 0.
    probe = probe_box(lower, upper, ground_z_m, cfg)
    if probe is None:
        return "too_small", None, 0., 0.
    camera = (_box_corners(*probe) - T[:3, 3]) @ T[:3, :3]
    reason, roi = _rect(camera, K, width, height)
    if reason is not None:
        return reason, None, 0., 0.
    if position is None:
        position = (np.asarray(lower, dtype=np.float64) + np.asarray(upper, dtype=np.float64)) / 2.0
    expected = float(((np.asarray(position, dtype=np.float64) - T[:3, 3]) @ T[:3, :3])[2])
    if not expected > 0.1:
        return "out_of_view", None, 0., 0.
    front, behind = thresholds(expected, cfg)
    near_z = max(front + cfg.depth_margin_m, NEAR_Z_MIN)
    far_z = max(behind - cfg.depth_margin_m, near_z)
    return None, roi, float(near_z), float(far_z)


def region_counts(depth, roi, near_z, far_z, margin):
    """(pixels, valid, front, behind) of one row exactly as the region core counts them:
    float32 depth, valid 0.1f < z < 10f, front z < float32(near_z - margin),
    behind z > float32(far_z + margin), closed rectangle."""
    x0, y0, x1, y1 = roi
    d = np.asarray(depth)[y0:y1 + 1, x0:x1 + 1].astype(np.float32)
    front_t = np.float32(near_z - margin); back_t = np.float32(far_z + margin)
    with np.errstate(invalid="ignore"):
        valid = (d > _F32_LO) & (d < _F32_HI)
        front = valid & (d < front_t)
        behind = valid & (d > back_t)
    return int(d.size), int(valid.sum()), int(front.sum()), int(behind.sum())


def word_from_counts(pixels, valid, front, behind, cfg):
    """v2 word of one checked row (CPU and KV share it; Python true division as the core)."""
    if valid < cfg.min_pixels or valid / pixels < cfg.min_valid_fraction:
        return "invalid_depth"
    if front / valid >= .5:
        return "occluded"
    if behind / valid >= cfg.free_fraction_v2:
        return "free"
    return "surface_or_uncertain"


def compute(frame, memory, cfg):
    """compute_geometry's dict under v2 (frame already validated)."""
    evidence = {}
    h, w = frame.depth.shape
    ground = ground_of(frame, cfg)
    for t in memory.tracks.values():
        if not frame.pose_valid:
            evidence[t.id] = "invalid_pose"; continue
        reason, roi, near_z, far_z = plan_one(frame.T_world_camera, frame.K, w, h, t.lower, t.upper,
                                              t.position, ground, cfg)
        if reason is not None:
            evidence[t.id] = reason; continue
        evidence[t.id] = word_from_counts(*region_counts(frame.depth, roi, near_z, far_z, cfg.depth_margin_m),
                                          cfg)
    return evidence
