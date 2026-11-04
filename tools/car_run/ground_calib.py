#!/usr/bin/env python3
"""Line E, live car (2026-10-15): start-up ground self-calibration of the camera mount.

The camera is fixed on the car body and its height / pitch are hand-measured (owner 10-15: about 0.36 m, 27-30 deg).
A pitch error of 1 deg moves the ground by about 2.6 cm at 1.5 m: configured too shallow, the far floor rises into the
obstacle band (false obstacles, also in the rtabmap / Nav2 map); configured too steep, the floor sinks and low
obstacles are missed. So, at start-up, with the car standing still on a flat floor, this fits the ground plane in the
first depth frames and derives the camera's real height, pitch and roll (yaw and x, y cannot be seen from the floor:
kept from the configuration), then -- by the switch CAR_GROUND_CALIB of car_up.sh / GROUND_CALIB of car_inner.sh:
  auto   (live default) quality gates pass and the fitted height is within +-3 cm of the configuration -> the fitted
         values are ADOPTED (new height_cell and mount TF via car_mount.mount); otherwise the start is REFUSED
         (10-28 owner: no angle limit against the configuration any more, see AUTO_MAX; the fallback below)
  check  quality gates pass and the fit is within 1 deg / 1 cm / 1 deg -> the CONFIGURED values are kept; else refused
  off    this step does not run (car_inner.sh: the mount exactly as before)
Adopted values still pass the hard limits: height 0.31-0.395 m (0.39 until 10-17), pitch <= 35 deg, the producer block check of
car_mount.mount (and, CAR_HEIGHT_CELL given explicitly, the fitted height must fit that cell).
The plan: ${FPGA_HOME}/dev1/.claude-accounts/claude_dev13/plans/cheeky-prancing-mochi.md (owner approved 10-15).

Pipeline (pure functions, numpy only, no ROS -- importable on the host; the ROS capture is separate, rclpy imported
lazily inside capture()):
  10 depth frames (16UC1, mm; 0 = no return) -> per-pixel median (a pixel needs half of the frames valid) -> every 4th
  pixel -> back-projected with K -> kept only where the CONFIGURED mount predicts the ground at 0.3-2.0 m ahead (the
  horizontal distance from the camera; configured pitch < 20 deg: 0.3-1.4 m, range_for, 10-17) -> robust plane fit near the configured plane (fit_plane, fixed seed:
  reproducible; final normal within 10 deg of the predicted one) -> height = camera-to-plane distance, pitch / roll
  from the plane normal.
Quality gates (written before any run; constants below, tests import them): inliers >= 3000 and >= 50 % of the
predicted ground samples; inlier RMS <= 1.5 cm; frames 1-5 and 6-10 fitted separately agree within 0.3 deg and 5 mm;
and (kernel selection, controller 10-15) the residual bend along the image rows <= 1 mm (BEND_MAX_M, an estimate).

fit_plane is the KERNEL INTERFACE: (points, prior_normal, ...) -> (n, d, inlier mask). The kernel
(tools/car_run/ground_plane_kernel.py, controller 10-15 selection): Open3D SegmentPlane's RANSAC ported to numpy (MIT)
+ PCL-style hypothesis gates (normal < 8 deg, height < 5 cm from the configuration) + MSAC scoring + Tukey IRLS
refit on sigma(z)-normalised residuals + exclusion of points whose local normal (4 px) is > 10 deg off the plane.
(It replaced the first placeholder, RANSAC 2 cm + one least-squares refit, which adopted 0.44-0.76 deg off with a
wide wall 0.7-1.2 m ahead.)

CLI (car_inner.sh runs it inside the stack container, after camera_check, before the mount TF):
  ground_calib.py --mode auto|check --height-m H --pitch-deg P [--height-cell N] --out FILE.json
                  [--depth-topic T --info-topic T --frames 10 --timeout-s 15] [--save-depth FILE.npz | --from-npz FILE.npz]
    exit 0 = adopted (auto) or confirmed (check); 1 = refused (FILE.json "reasons", readable); 2 = no frames / no ROS
    10-26 owner（重成功率，放宽自「auto 质量门不过就拒启动」）: auto never refuses for the fit any more -- a failed gate,
    a fit too far from the configuration or no depth frames -> 10-28: the LAST GOOD mount (--last-good FILE, the last
    adopted fit; source last_good), without one the CONFIGURED mount (source config_fallback); the refusal text in
    "warnings", the console text in "alert", exit 0. Still refused: neither mount fits the geometry block.
  ground_calib.py --write-last-good RESULT.json LAST_GOOD.json   (car_up.sh after READY: only a source-fit result)
  ground_calib.py --read FILE.json   -> "height_cell height_m pitch_deg roll_deg source x y z qx qy qz qw" (shell)
  ground_calib.py --compare MAP.json FILE.json -> one line: this start's adopted mount against the mapping run's
Python 3.8 compatible."""
import argparse
import json
import math
import os
import sys
import time
import warnings

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.car_geo_producer.common import base_camera  # noqa: E402
from tools.car_run import ground_plane_kernel as kernel  # noqa: E402

# --- written before any run (plan "owner 已定" 10-15 + brief); do not loosen ----------------------------------------
FRAMES = 10                      # depth frames captured at start-up (two halves of 5 for the consistency gate)
STRIDE = 4                       # every 4th pixel in u and v
RANGE_M = (0.3, 2.0)             # ground ahead of the camera (horizontal distance), predicted with the configured mount
PRIOR_MAX_DEG = 10.0             # fitted normal within 10 deg of the configured one
MIN_INLIERS = 3000
MIN_INLIER_SHARE = 0.5           # of the predicted ground samples
MAX_RMS_M = 0.015
HALF_MAX_DEG = 0.3               # first 5 vs last 5 frames
HALF_MAX_M = 0.005
# auto: adopt the fit when the quality gates pass and the HEIGHT is within 3 cm of the configuration (owner 10-28: the
# angle limits +-5 deg pitch / +-5 deg roll are gone -- 10-28 14:19 a clean fit 5.7 deg / roll 5.8 deg (3596 inliers,
# rms 1.6 mm, halves 0.013 deg) was refused for the roll and the stale configured 7 deg / 0 deg raised the floor into
# a ring of false obstacles around the car). The selection prior (PRIOR_MAX_DEG, kernel HYP_MAX_DEG) still picks
# the plane; the hard limits below still refuse.
AUTO_MAX = {"height_m": 0.03}
CHECK_MAX = {"pitch_deg": 1.0, "height_m": 0.01, "roll_deg": 1.0}   # check: keep the configuration within these
# upper end 0.39 -> 0.395 (owner 10-17 approved on the controller's question; the camera re-tilted, fitted 0.3915 m):
# producer voxel 0.05 m, block 32x32x10 -- a lens anywhere in [0.35, 0.40) sits in the same cell, height_cell 8, block
# bottom -0.05 m / top 0.45 m identical; at 0.40 m it enters the next cell, needs height_cell 9 and breaks "lens >= 1
# cell below the block top" (car_mount block check refuses). 5 mm kept below 0.40 for the fit repeatability (plan
# criterion <= 2 mm). docs/design/CAM_TILT_10_20261017.md section 9.4.
HARD_HEIGHT_M = (0.31, 0.395)
HARD_PITCH_MAX_DEG = 35.0
CONFIG_ROLL_DEG = 0.0            # the configuration has no roll
SEED = 20261015                  # fixed: the same frames give the same fit
BEND_MAX_M = 0.001                # residual bend along the image rows (kernel selection 10-15; ESTIMATE, see runbook)
# --- 10-17 (controller, camera tilted to about 7 deg): which floor is SELECTED, not a quality gate ---------------
# Configured pitch < SHALLOW_PITCH_DEG: the selection range ends at SHALLOW_RANGE_MAX_M instead of RANGE_M[1]. At a
# shallow pitch the lower image edge no longer sees the floor 0.3-0.6 m ahead (car K, 7 deg: from 0.63 m), the ring
# that carries the share gate at 27 deg, and real Gemini 2 floor normals degrade with distance (gate-exceeding share
# 0.6 -> 1.4 m: 3 -> 31 % at 27 deg, 10 -> 32 % at 7 deg). >= 20 deg (the originally validated 20-33 deg): RANGE_M,
# byte for byte as before. docs/design/CAM_TILT_10_20261017.md section 9.1.
SHALLOW_PITCH_DEG = 20.0
SHALLOW_RANGE_MAX_M = 1.4

EXIT_OK, EXIT_REFUSED, EXIT_NO_INPUT = 0, 1, 2
MOVE_HINT_FMT = "把车挪到前方 %s m 有一片空平地面的地方再开；确需跳过只能显式设 CAR_GROUND_CALIB=off"


def range_for(pitch_deg):
    """the selection range (near, far) m for a CONFIGURED pitch: RANGE_M at >= SHALLOW_PITCH_DEG, else the far end
    cut to SHALLOW_RANGE_MAX_M (section 9.1)."""
    if float(pitch_deg) < SHALLOW_PITCH_DEG:
        return (RANGE_M[0], SHALLOW_RANGE_MAX_M)
    return RANGE_M


def range_text(range_m):
    """'0.3–2' for (0.3, 2.0): the wording of the refusal reasons."""
    return "%g–%g" % (range_m[0], range_m[1])


MOVE_HINT = MOVE_HINT_FMT % range_text(RANGE_M)


# --- depth -> points ---------------------------------------------------------------------------------------------
def depth_mm_to_m(depth_u16):
    """16UC1 depth in mm -> float32 m, 0 (no return) -> NaN."""
    d = np.asarray(depth_u16)
    out = d.astype(np.float32) / np.float32(1000.0)
    out[d == 0] = np.nan
    return out


def median_depth(frames_m, min_valid_share=0.5):
    """per-pixel median over frames (NaN ignored); a pixel valid in fewer than min_valid_share of them -> NaN."""
    stack = np.stack([np.asarray(f, dtype=np.float32) for f in frames_m])
    valid = np.isfinite(stack).sum(axis=0)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)       # all-NaN pixels
        med = np.nanmedian(stack, axis=0)
    med[valid < max(1, int(math.ceil(min_valid_share * stack.shape[0])))] = np.nan
    return med.astype(np.float32)


def sample_pixels(width, height, stride=STRIDE):
    """(u, v) of the sampled pixels, row-major, centred in each stride cell."""
    o = stride // 2
    v, u = np.meshgrid(np.arange(o, height, stride, dtype=np.float64), np.arange(o, width, stride, dtype=np.float64),
                       indexing="ij")
    return u.ravel(), v.ravel()


def rays(K, u, v):
    """optical-frame rays with z = 1 for pixels (u, v)."""
    return np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)], -1)


def mount_rotation(pitch_deg, roll_deg=0.0):
    return base_camera((0.0, 0.0, 0.0), pitch_deg, roll_deg)[:3, :3]


def up_normal(pitch_deg, roll_deg=0.0):
    """the ground's up normal in the camera optical frame for a camera at (pitch, roll)."""
    return mount_rotation(pitch_deg, roll_deg).T @ np.array([0.0, 0.0, 1.0])


def predicted_ground(K, u, v, height_m, pitch_deg, roll_deg=0.0, range_m=RANGE_M):
    """bool per sample: the configured mount sees the ground there, at a horizontal distance within range_m."""
    d = rays(K, u, v) @ mount_rotation(pitch_deg, roll_deg).T        # base-frame direction per unit optical z
    down = d[:, 2] < -1e-9
    t = np.full(u.shape, np.inf)
    t[down] = height_m / -d[down, 2]
    horiz = t * np.hypot(d[:, 0], d[:, 1])
    return down & (horiz >= range_m[0]) & (horiz <= range_m[1])


def horizontal_distance(points_c, pitch_deg, roll_deg=0.0):
    """horizontal distance from the camera of optical-frame points, with the configured rotation."""
    pb = points_c @ mount_rotation(pitch_deg, roll_deg).T
    return np.hypot(pb[:, 0], pb[:, 1])


def ground_candidates(depth_m, K, height_m, pitch_deg, roll_deg=0.0, stride=STRIDE, range_m=RANGE_M):
    """(points (N, 3) optical frame, n_predicted): the sampled pixels the configured mount predicts as ground, with
    a valid depth whose point lies 0.3-2.0 m ahead (horizontal distance, configured rotation)."""
    h, w = depth_m.shape
    u, v = sample_pixels(w, h, stride)
    z = depth_m[v.astype(int), u.astype(int)]
    return sample_candidates(z, u, v, K, height_m, pitch_deg, roll_deg, range_m)


def sample_candidates(z, u, v, K, height_m, pitch_deg, roll_deg=0.0, range_m=RANGE_M, shape=None):
    """ground_candidates on depths z (m, NaN = none) already taken at the sampled pixels (u, v).
    shape (rows, cols) of the sample grid given -> also the local normals (central differences one sample = STRIDE
    px apart) and the image rows of the candidates: (points, normals, rows, n_predicted); else (points, n_predicted)."""
    pred = predicted_ground(K, u, v, height_m, pitch_deg, roll_deg, range_m)
    z = np.asarray(z, dtype=np.float64)
    valid = np.isfinite(z) & (z > 0)
    ok = pred & valid
    pts = rays(K, u[ok], v[ok]) * z[ok, None]
    hd = horizontal_distance(pts, pitch_deg, roll_deg)
    keep = (hd >= range_m[0]) & (hd <= range_m[1])
    if shape is None:
        return pts[keep], int(pred.sum())
    allp = np.where(valid[:, None], rays(K, u, v) * np.where(valid, z, np.nan)[:, None], np.nan)
    N = kernel.local_normals(allp.reshape(shape[0], shape[1], 3)).reshape(-1, 3)
    return pts[keep], N[ok][keep], v[ok][keep], int(pred.sum())


def grid_shape(width, height, stride=STRIDE):
    o = stride // 2
    return len(range(o, height, stride)), len(range(o, width, stride))


# --- the plane-fit kernel (interface) ----------------------------------------------------------------------------
def fit_plane(points, prior_normal, prior_d=None, normals=None, seed=SEED):
    """Plane n . X + d = 0 through `points` (N, 3) near the prior (up normal prior_normal, distance prior_d).
    -> (n, d, inliers): n unit, oriented to the prior (n . prior > 0), within PRIOR_MAX_DEG of it; inliers a bool mask
    over points. No plane -> (None, None, all-False mask). Deterministic for a given seed.
    Kernel: tools/car_run/ground_plane_kernel.fit_plane (Open3D SegmentPlane port + gates + MSAC + Tukey IRLS)."""
    n, d, inl = kernel.fit_plane(points, prior_normal, prior_d=prior_d, normals=normals, seed=seed)
    if n is None or angle_deg(n, prior_normal) > PRIOR_MAX_DEG:
        return None, None, np.zeros(len(points), dtype=bool)
    return n, d, inl


# --- plane -> mount ----------------------------------------------------------------------------------------------
def pose_from_plane(n, d):
    """(height_m, pitch_deg, roll_deg) of the camera above the plane n . X + d = 0 (n the up normal, optical frame).
    With R = R_y(pitch) R_x(roll) LEVEL (common.base_camera) the up normal in the optical frame is
    (-sin(roll) cos(pitch), -cos(roll) cos(pitch), -sin(pitch))."""
    n = np.asarray(n, dtype=np.float64)
    n = n / np.linalg.norm(n)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -n[2]))))
    roll = math.degrees(math.atan2(-n[0], -n[1]))
    return float(d), pitch, roll


def angle_deg(a, b):
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    c = float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def fit_depth(z, u, v, K, height_m, pitch_deg, roll_deg=0.0, seed=SEED, shape=None, range_m=RANGE_M):
    """fit the median depths z at the sampled pixels (u, v) of a grid of `shape` -> dict (n, d, height/pitch/roll,
    inliers, share, rms, bend, predicted, candidates); candidates selected within range_m (calibrate: range_for)."""
    pts, nrm, rows, n_pred = sample_candidates(z, u, v, K, height_m, pitch_deg, roll_deg, range_m, shape=shape)
    n, d, inl = fit_plane(pts, up_normal(pitch_deg, roll_deg), prior_d=height_m, normals=nrm, seed=seed)
    r = {"predicted_ground_samples": n_pred, "candidates": int(len(pts)), "plane_found": n is not None,
         "inliers": int(inl.sum()), "inlier_share": (float(inl.sum()) / n_pred) if n_pred else 0.0}
    if n is None:
        return r
    res = pts[inl] @ n + d
    h, p, ro = pose_from_plane(n, d)
    r.update({"normal": [float(x) for x in n], "d": float(d), "rms_m": float(np.sqrt(np.mean(res ** 2))),
              "bend_m": float(kernel.residual_bend(pts, n, d, inl, rows)),
              "height_m": h, "pitch_deg": p, "roll_deg": ro})
    return r


# --- the decision ------------------------------------------------------------------------------------------------
def tf_string(xyz, q):
    """as car_mount.py prints it ("%.9g" each)."""
    return " ".join("%.9g" % v for v in tuple(xyz) + tuple(q))


def _mount(height_m, pitch_deg, height_cell, roll_deg):
    from tools.car_run.car_mount import mount    # host.geo_producer: imported only when a mount is computed
    return mount(height_m, pitch_deg, height_cell, roll_deg=roll_deg)


def calibrate(frames_mm, K, height_m, pitch_deg, mode="auto", height_cell=None, seed=SEED, prior=None):
    """frames_mm: FRAMES depth images (uint16 mm, 0 = none), K 3x3; the configured height / pitch (roll 0);
    mode auto|check; height_cell: given explicitly (CAR_HEIGHT_CELL) or None (derived). -> result dict (JSON):
    verdict adopt | confirm | reject, reasons, configured / fitted / adopted / deviation / quality / gates,
    height_cell and mount_tf ("x y z qx qy qz qw") of the adopted mount (reject: null).
    prior (10-28): (height_m, pitch_deg, roll_deg) that SELECTS the floor (predicted ground, plane prior, range)
    instead of the configuration; the deviation / height limit stay against the configuration. None = as before."""
    if mode not in ("auto", "check"):
        raise ValueError("mode must be auto or check")
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    nf = len(frames_mm)
    if nf:
        h, w = np.asarray(frames_mm[0]).shape
        u, v = sample_pixels(w, h)
        shape = grid_shape(w, h)
        iv, iu = v.astype(int), u.astype(int)
        # per-pixel median then every 4th pixel == every 4th pixel then per-pixel median (16x less work)
        frames_m = [depth_mm_to_m(np.asarray(f)[iv, iu]) for f in frames_mm]
    ph, pp, pr = (float(height_m), float(pitch_deg), CONFIG_ROLL_DEG) if prior is None else \
        tuple(float(x) for x in prior)
    rng = range_for(pp)
    hint = MOVE_HINT_FMT % range_text(rng)
    cfg = {"height_m": float(height_m), "pitch_deg": float(pitch_deg), "roll_deg": CONFIG_ROLL_DEG,
           "height_cell": None if height_cell is None else int(height_cell)}
    out = {"version": 1, "mode": mode, "configured": cfg, "frames": nf, "fitted": None, "adopted": None,
           "deviation": None, "verdict": "reject", "reasons": [], "height_cell": None, "mount_tf": None,
           "kernel": "open3d_segment_plane_port+pcl_gates+msac+tukey_irls+local_normals", "seed": seed,
           "gates": {"min_inliers": MIN_INLIERS, "min_inlier_share": MIN_INLIER_SHARE, "max_rms_m": MAX_RMS_M,
                     "half_max_deg": HALF_MAX_DEG, "half_max_m": HALF_MAX_M, "range_m": list(rng),
                     "prior_max_deg": PRIOR_MAX_DEG, "stride": STRIDE, "bend_max_m": BEND_MAX_M,
                     "kernel": {"hyp_max_deg": kernel.HYP_MAX_DEG, "hyp_max_dh_m": kernel.HYP_MAX_DH_M,
                                "msac_t_m": kernel.MSAC_T, "sigma_k": kernel.SIGMA_K, "sigma0_m": kernel.SIGMA0_M,
                                "tukey_c": kernel.TUKEY_C, "irls_rounds": kernel.IRLS_ROUNDS,
                                "local_normal_max_deg": kernel.LOCAL_NORMAL_MAX_DEG, "iters": kernel.RANSAC_ITERS},
                     "auto_max": AUTO_MAX, "check_max": CHECK_MAX,
                     "hard_height_m": list(HARD_HEIGHT_M), "hard_pitch_max_deg": HARD_PITCH_MAX_DEG}}
    if prior is not None:
        out["prior"] = {"height_m": ph, "pitch_deg": pp, "roll_deg": pr}
    if rng != RANGE_M:       # only when the shallow rule is active: >= 20 deg the JSON is byte for byte as before
        out["gates"]["range_rule"] = {"shallow_pitch_deg": SHALLOW_PITCH_DEG, "shallow_range_max_m": SHALLOW_RANGE_MAX_M,
                                      "range_m_default": list(RANGE_M)}
    reasons = out["reasons"]
    if nf < 2:
        reasons.append("深度帧不够（%d 帧，需 %d）：相机没出数据" % (nf, FRAMES))
        return out
    nz = float(np.mean([np.isfinite(f).mean() for f in frames_m]))
    full = fit_depth(median_depth(frames_m), u, v, K, ph, pp, pr, seed, shape, rng)
    half = nf // 2
    a = fit_depth(median_depth(frames_m[:half]), u, v, K, ph, pp, pr, seed, shape, rng)
    b = fit_depth(median_depth(frames_m[half:]), u, v, K, ph, pp, pr, seed, shape, rng)
    q = {"valid_depth_share": round(nz, 4), "full": full, "first_half": a, "second_half": b}
    out["quality"] = q
    if nz == 0.0:
        reasons.append("深度图全 0（相机近距保护：镜头前很近处有东西挡着？）；" + hint)
        return out
    if not full["plane_found"]:
        reasons.append("车前 %s m 没有找到朝向接近预期（差 ≤%g°）的平面：画面里是墙或箱子，看不到地面；%s"
                       % (range_text(rng), PRIOR_MAX_DEG, hint))
        return out
    if full["inliers"] < MIN_INLIERS or full["inlier_share"] < MIN_INLIER_SHARE:
        reasons.append("车前 %s m 需要一片空的平地面：地面内点 %d 个、占预测地面点 %.0f%%（需 ≥%d 个且 ≥%.0f%%）；%s"
                       % (range_text(rng), full["inliers"], 100 * full["inlier_share"], MIN_INLIERS,
                          100 * MIN_INLIER_SHARE, hint))
    if full["rms_m"] > MAX_RMS_M:
        reasons.append("地面拟合残差 %.1f cm > %.1f cm：地面不平或有杂物、深度噪声太大；%s"
                       % (100 * full["rms_m"], 100 * MAX_RMS_M, hint))
    if full["bend_m"] > BEND_MAX_M:
        reasons.append("地面残差沿图像行有系统弯曲 %.1f mm（限 %g mm）：前方有墙根、箱子底边或地面不平混进了地面点；%s"
                       % (1000 * full["bend_m"], 1000 * BEND_MAX_M, hint))
    if not (a["plane_found"] and b["plane_found"]):
        reasons.append("前 %d 帧与后 %d 帧有一半拟合不出地面：相机或车在动，或深度时有时无" % (half, nf - half))
    else:
        dang = angle_deg(a["normal"], b["normal"])
        dd = abs(a["d"] - b["d"])
        q["halves_delta_deg"] = dang
        q["halves_delta_m"] = dd
        if dang > HALF_MAX_DEG or dd > HALF_MAX_M:
            reasons.append("前后两半帧拟合不一致（法向差 %.2f°、高度差 %.1f mm，限 %g° / %g mm）：车或相机在动？停稳再开"
                           % (dang, 1000 * dd, HALF_MAX_DEG, 1000 * HALF_MAX_M))
    fit = {"height_m": full["height_m"], "pitch_deg": full["pitch_deg"], "roll_deg": full["roll_deg"]}
    out["fitted"] = fit
    dev = {k: fit[k] - (cfg[k] if k != "roll_deg" else CONFIG_ROLL_DEG) for k in ("height_m", "pitch_deg", "roll_deg")}
    out["deviation"] = dev
    if reasons:
        return out
    lim = AUTO_MAX if mode == "auto" else CHECK_MAX
    over = [k for k in ("pitch_deg", "height_m", "roll_deg") if k in lim and abs(dev[k]) > lim[k]]
    if over:
        if mode == "auto":
            reasons.append("装的位置和配置差太多，或者把别的平面当成了地面：拟合 高度 %.3f m / 下倾 %.1f° / 横滚 %.1f°，"
                           "配置 %.3f m / %.1f° / 0°（自动修正高度限 ±%g cm）；请重量安装再改 CAR_CAM_HEIGHT，"
                           "或把车挪到前方是空平地面的地方"
                           % (fit["height_m"], fit["pitch_deg"], fit["roll_deg"], cfg["height_m"], cfg["pitch_deg"],
                              100 * lim["height_m"]))
        else:
            reasons.append("核对不过（check）：拟合 高度 %.3f m / 下倾 %.2f° / 横滚 %.2f° 与配置 %.3f m / %.2f° / 0° 相差超过 "
                           "%g cm / %g° / %g°；改用 auto 自动修正，或重量安装"
                           % (fit["height_m"], fit["pitch_deg"], fit["roll_deg"], cfg["height_m"], cfg["pitch_deg"],
                              100 * lim["height_m"], lim["pitch_deg"], lim["roll_deg"]))
        return out
    if mode == "auto":
        ad = dict(fit, source="fit")
    else:
        ad = {"height_m": cfg["height_m"], "pitch_deg": cfg["pitch_deg"], "roll_deg": CONFIG_ROLL_DEG, "source": "config"}
    if not HARD_HEIGHT_M[0] <= ad["height_m"] <= HARD_HEIGHT_M[1]:
        reasons.append("拟合高度 %.4f m 超出硬限 %g–%g m" % (ad["height_m"], HARD_HEIGHT_M[0], HARD_HEIGHT_M[1]))
    if ad["pitch_deg"] > HARD_PITCH_MAX_DEG:
        reasons.append("拟合下倾 %.1f° 超出硬限 ≤%g°" % (ad["pitch_deg"], HARD_PITCH_MAX_DEG))
    if reasons:
        return out
    try:
        hc, xyz, qq = _mount(ad["height_m"], ad["pitch_deg"], height_cell, ad["roll_deg"])
    except ValueError as exc:
        reasons.append("高度 %.3f m 放不进几何块%s：%s" % (
            ad["height_m"], "（显式设了 CAR_HEIGHT_CELL=%d）" % height_cell if height_cell is not None else "", exc))
        return out
    out.update({"verdict": "adopt" if mode == "auto" else "confirm", "adopted": ad, "height_cell": hc,
                "mount_tf": tf_string(xyz, qq)})
    return out


# --- 10-28 prior search: the floor is not looked for only around the (possibly stale) configuration -------------
SWEEP_PITCH_DEG = tuple(float(p) for p in range(0, 36, 5))   # 0, 5, ... 35 deg (HARD_PITCH_MAX_DEG), roll 0


def calibrate_search(frames_mm, K, height_m, pitch_deg, mode="auto", height_cell=None, seed=SEED, last_good=None):
    """auto (10-28, controller: the owner re-tilts the camera between 7 and 25 deg; the kernel finds the floor only
    within about 8 deg of its prior): 1) the LAST GOOD mount as the prior (last_good = (h, p, r) or None), 2) the
    configured mount (calibrate exactly as before), 3) a sweep of pitch priors SWEEP_PITCH_DEG (configured height,
    roll 0): of the attempts that pass every gate the one with the most inliers. The first of 1) / 2) that is
    adopted wins; none adopted -> the configured attempt's result (its reasons; the fallback follows in main).
    check: calibrate as before. "search" in the result lists the attempts."""
    if mode != "auto":
        return calibrate(frames_mm, K, height_m, pitch_deg, mode, height_cell, seed)
    tried = []

    def note(kind, prior, r):
        f = (r.get("quality") or {}).get("full") or {}
        tried.append({"prior": kind, "prior_pitch_deg": None if prior is None else round(float(prior[1]), 3),
                      "verdict": r["verdict"], "inliers": f.get("inliers"),
                      "fitted_pitch_deg": None if not r.get("fitted") else round(r["fitted"]["pitch_deg"], 3)})
        r["search"] = tried
        return r

    if last_good is not None:
        r = calibrate(frames_mm, K, height_m, pitch_deg, mode, height_cell, seed, prior=last_good)
        if r["verdict"] == "adopt":
            r["adopted"]["prior"] = "last_good"
            return note("last_good", last_good, r)
        note("last_good", last_good, r)
    base = calibrate(frames_mm, K, height_m, pitch_deg, mode, height_cell, seed)
    note("config", None, base)
    if base["verdict"] == "adopt" or base.get("frames", 0) < 2 or \
            (base.get("quality") or {}).get("valid_depth_share") == 0.0:
        return base
    best = None
    for p in SWEEP_PITCH_DEG:
        r = calibrate(frames_mm, K, height_m, pitch_deg, mode, height_cell, seed, prior=(height_m, p, CONFIG_ROLL_DEG))
        note("sweep", (height_m, p, 0.0), r)
        if r["verdict"] == "adopt" and (best is None or r["quality"]["full"]["inliers"] >
                                        best["quality"]["full"]["inliers"]):
            best = r
    if best is not None:
        best["adopted"]["prior"] = "sweep"
        best["search"] = tried
        return best
    base["search"] = tried
    return base


# --- ROS capture (inside the stack container) ----------------------------------------------------------------------
class CaptureError(Exception):
    pass


def image_to_u16(msg):
    """sensor_msgs/Image 16UC1 (or mono16) -> (h, w) uint16."""
    if msg.encoding not in ("16UC1", "mono16"):
        raise CaptureError("depth encoding %s, expected 16UC1" % msg.encoding)
    dt = np.dtype(">u2" if msg.is_bigendian else "<u2")
    a = np.frombuffer(bytes(msg.data), dtype=dt)
    row = int(msg.step) // 2 if msg.step else int(msg.width)
    if a.size < row * int(msg.height):
        raise CaptureError("depth buffer %d < step x height" % a.size)
    return a[:row * int(msg.height)].reshape(int(msg.height), row)[:, :int(msg.width)].astype(np.uint16)


def capture(depth_topic, info_topic, frames=FRAMES, timeout_s=15.0):
    """(list of uint16 depth images, K 3x3, info dict) from the live topics (sensor-data QoS, as depth_probe.py)."""
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
    except ImportError as exc:
        raise CaptureError("import: %s" % exc)
    st = {"imgs": [], "info": None}

    def on_img(m):
        if len(st["imgs"]) < frames:
            st["imgs"].append(m)

    def on_info(m):
        st["info"] = m

    t0 = time.monotonic()
    rclpy.init(args=None)
    node = rclpy.create_node("fpga_ground_calib")
    node.create_subscription(Image, depth_topic, on_img, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, info_topic, on_info, qos_profile_sensor_data)
    try:
        while (len(st["imgs"]) < frames or st["info"] is None) and time.monotonic() - t0 < timeout_s:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    info = st["info"]
    if info is None:
        raise CaptureError("%s 在 %.0f s 内没有 CameraInfo" % (info_topic, timeout_s))
    if len(st["imgs"]) < frames:
        raise CaptureError("%s 在 %.0f s 内只收到 %d 帧深度图（需 %d）" % (depth_topic, timeout_s, len(st["imgs"]), frames))
    imgs = [image_to_u16(m) for m in st["imgs"]]
    K = np.array(list(info.k), dtype=np.float64).reshape(3, 3)
    h, w = imgs[0].shape
    if (int(info.width), int(info.height)) != (w, h):
        raise CaptureError("CameraInfo %dx%d 与深度图 %dx%d 不一致" % (info.width, info.height, w, h))
    meta = {"width": w, "height": h, "frame_id": st["imgs"][0].header.frame_id, "info_frame_id": info.header.frame_id,
            "K": K.tolist(), "seconds": round(time.monotonic() - t0, 3), "encoding": st["imgs"][0].encoding}
    return imgs, K, meta


# --- results for the shell ------------------------------------------------------------------------------------------
# --- last good (10-28 owner): the mount of the last start whose fit was ADOPTED, kept across runs ------------------
LAST_GOOD_NAME = "ground_calib_last_good.json"   # car_up.sh: $CAR_STATE_DIR/<this> (written after READY when source fit)
LAST_GOOD_IN_NAME = "ground_calib_last_good_in.json"   # car_up.sh copies it into the run dir; car_inner.sh --last-good
ALERT_LAST_GOOD = "相机角度测量异常，已用上次标定值，建议检查相机"
ALERT_CONFIG = "相机角度测量异常，已用默认安装值，建议检查相机"


def last_good_record(result, run=None, stamp=None):
    """the last-good record of a result whose fit was adopted (source fit), else None."""
    a = (result or {}).get("adopted") or {}
    if result is None or result.get("verdict") != "adopt" or a.get("source") != "fit":
        return None
    return {"version": 1, "height_m": float(a["height_m"]), "pitch_deg": float(a["pitch_deg"]),
            "roll_deg": float(a["roll_deg"]), "height_cell": result.get("height_cell"),
            "mount_tf": result.get("mount_tf"), "run": run, "stamp": stamp,
            "quality": {k: ((result.get("quality") or {}).get("full") or {}).get(k)
                        for k in ("inliers", "inlier_share", "rms_m", "bend_m")}}


def read_last_good(path):
    """((height_m, pitch_deg, roll_deg, record), None) of a last-good file, or (None, why) when absent / unreadable /
    outside the hard limits."""
    if not path:
        return None, "无上次标定值"
    try:
        rec = json.load(open(path))
        h, p, r = float(rec["height_m"]), float(rec["pitch_deg"]), float(rec["roll_deg"])
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return None, "上次标定值读不出（%s）" % exc
    if not all(math.isfinite(x) for x in (h, p, r)):
        return None, "上次标定值不是有限数"
    if not HARD_HEIGHT_M[0] <= h <= HARD_HEIGHT_M[1] or p > HARD_PITCH_MAX_DEG:
        return None, "上次标定值超出硬限（高度 %.3f m / 下倾 %.1f°）" % (h, p)
    return (h, p, r, rec), None


def config_fallback(res, height_cell, last_good=None):
    """10-26 owner：重成功率，放宽自 auto 拒启动: a rejected auto result -> a known mount, the reasons kept as
    "warnings". 10-28 owner: first the LAST GOOD mount (the last adopted fit, file `last_good`; source last_good),
    only without a usable one the configured mount (source config_fallback). "alert" carries the console text.
    Unchanged (still a reject) when neither mount can be built (car_mount refuses it); car_inner.sh's hard height
    check runs afterwards as before."""
    cfg = res.get("configured") or {}
    lg, why = read_last_good(last_good)
    tried, cands = [], []
    if lg is not None:
        cands.append(({"height_m": lg[0], "pitch_deg": lg[1], "roll_deg": lg[2], "source": "last_good"},
                      ALERT_LAST_GOOD))
    try:
        cands.append(({"height_m": float(cfg["height_m"]), "pitch_deg": float(cfg["pitch_deg"]),
                       "roll_deg": CONFIG_ROLL_DEG, "source": "config_fallback"}, ALERT_CONFIG))
    except (KeyError, TypeError, ValueError) as exc:
        tried.append("配置值不可用：%s" % exc)
    for ad, alert in cands:
        try:
            hc, xyz, qq = _mount(ad["height_m"], ad["pitch_deg"], height_cell, ad["roll_deg"])
        except (TypeError, ValueError) as exc:
            tried.append("%s 放不进几何块：%s" % (ad["source"], exc))
            continue
        out = dict(res)
        out.update({"verdict": "adopt", "adopted": ad, "height_cell": hc, "mount_tf": tf_string(xyz, qq),
                    "warnings": list(res.get("reasons") or []) + tried, "reasons": [], "alert": alert,
                    "last_good": {"file": last_good, "used": ad["source"] == "last_good",
                                  "why_not": None if ad["source"] == "last_good" else why,
                                  "record": lg[3] if lg is not None else None}})
        return out
    res.setdefault("reasons", []).append("退回上次标定值 / 配置值也不行：%s" % "；".join(tried))
    return res


def read_line(result):
    """"height_cell height_m pitch_deg roll_deg source x y z qx qy qz qw" of an adopt / confirm result."""
    if result.get("verdict") not in ("adopt", "confirm"):
        raise ValueError("verdict %s" % result.get("verdict"))
    a = result["adopted"]
    return "%d %.4f %.3f %.3f %s %s" % (result["height_cell"], a["height_m"], a["pitch_deg"], a["roll_deg"],
                                        a["source"], result["mount_tf"])


def compare_line(map_result, result):
    """this start's adopted mount against the one stored beside the map (mapping run) -- recorded, not judged."""
    m, a = map_result.get("adopted") or {}, result.get("adopted") or {}
    if not m or not a:
        return "ground_calib vs map: no adopted mount on one side"
    dh = a["height_m"] - m["height_m"]
    dp = a["pitch_deg"] - m["pitch_deg"]
    dr = a["roll_deg"] - m["roll_deg"]
    warn = abs(dh) > 0.01 or abs(dp) > 1.0 or abs(dr) > 1.0
    return ("ground_calib vs map: height %+.1f mm pitch %+.2f deg roll %+.2f deg (map %.3f m / %.2f / %.2f, now "
            "%.3f m / %.2f / %.2f)%s" % (1000 * dh, dp, dr, m["height_m"], m["pitch_deg"], m["roll_deg"],
                                         a["height_m"], a["pitch_deg"], a["roll_deg"],
                                         " WARNING > 1 deg / 1 cm (the current mount is used)" if warn else ""))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--read", metavar="RESULT_JSON")
    ap.add_argument("--compare", nargs=2, metavar=("MAP_JSON", "RESULT_JSON"))
    ap.add_argument("--mode", choices=("auto", "check"), default="auto")
    ap.add_argument("--height-m", type=float)
    ap.add_argument("--pitch-deg", type=float)
    ap.add_argument("--height-cell", default="", help="given explicitly (CAR_HEIGHT_CELL); empty = derived")
    ap.add_argument("--out")
    ap.add_argument("--depth-topic", default="/camera/depth/image_raw")
    ap.add_argument("--info-topic", default="/camera/depth/camera_info")
    ap.add_argument("--frames", type=int, default=FRAMES)
    ap.add_argument("--timeout-s", type=float, default=15.0)
    ap.add_argument("--save-depth", help="write the captured frames + K here (npz) for an offline re-run")
    ap.add_argument("--from-npz", help="offline: frames + K from a --save-depth file instead of the topics")
    ap.add_argument("--last-good", help="auto fallback: the last adopted fit (%s); absent = the configuration"
                    % LAST_GOOD_NAME)
    ap.add_argument("--write-last-good", nargs=2, metavar=("RESULT_JSON", "LAST_GOOD_JSON"),
                    help="RESULT adopted its fit (source fit) -> write LAST_GOOD (atomic); else leave it; exit 0")
    a = ap.parse_args(argv)
    if a.read:
        try:
            print(read_line(json.load(open(a.read))))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            sys.stderr.write("ground_calib --read: %s\n" % exc)
            return 1
        return 0
    if a.write_last_good:
        try:
            res = json.load(open(a.write_last_good[0]))
            rec = last_good_record(res, run=os.path.basename(os.path.dirname(os.path.abspath(a.write_last_good[0]))),
                                   stamp=time.strftime("%Y-%m-%dT%H:%M:%S"))
            if rec is None:
                print("ground_calib last good: not written (source %s)" % ((res.get("adopted") or {}).get("source")))
                return 0
            tmp = a.write_last_good[1] + ".tmp"
            with open(tmp, "w") as f:
                json.dump(rec, f, indent=1, ensure_ascii=False)
            os.replace(tmp, a.write_last_good[1])
            print("ground_calib last good: %.4f m / %.3f deg / roll %.3f deg -> %s"
                  % (rec["height_m"], rec["pitch_deg"], rec["roll_deg"], a.write_last_good[1]))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            print("ground_calib last good: not written (%s)" % exc)
        return 0
    if a.compare:
        try:
            print(compare_line(json.load(open(a.compare[0])), json.load(open(a.compare[1]))))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            print("ground_calib vs map: not compared (%s)" % exc)
        return 0
    if a.height_m is None or a.pitch_deg is None or not a.out:
        ap.error("--height-m, --pitch-deg and --out are required")
    hc = int(a.height_cell) if a.height_cell else None
    t0 = time.monotonic()
    meta = {}
    try:
        if a.from_npz:
            z = np.load(a.from_npz)
            frames, K = list(z["frames"]), z["K"]
            meta = {"from_npz": a.from_npz}
        else:
            frames, K, meta = capture(a.depth_topic, a.info_topic, a.frames, a.timeout_s)
    except (CaptureError, OSError, KeyError) as exc:
        res = {"version": 1, "mode": a.mode, "verdict": "reject", "reasons": ["抓不到深度图：%s" % exc],
               "configured": {"height_m": a.height_m, "pitch_deg": a.pitch_deg, "roll_deg": CONFIG_ROLL_DEG,
                              "height_cell": hc}}
        if a.mode == "auto":     # 10-26 owner：重成功率，放宽自拒启动 -- the configured mount, with a warning
            res = config_fallback(res, hc, a.last_good)
        with open(a.out, "w") as f:
            json.dump(res, f, indent=1, ensure_ascii=False)
        print(json.dumps({k: res.get(k) for k in ("verdict", "reasons", "warnings", "adopted", "alert")}, ensure_ascii=False))
        return EXIT_OK if res["verdict"] == "adopt" else EXIT_NO_INPUT
    if a.save_depth:
        np.savez_compressed(a.save_depth, frames=np.stack(frames), K=np.asarray(K))
    lg = read_last_good(a.last_good)[0] if a.mode == "auto" else None
    res = calibrate_search(frames, K, a.height_m, a.pitch_deg, a.mode, hc, last_good=None if lg is None else lg[:3])
    if a.mode == "auto" and res["verdict"] == "reject":   # 10-26 owner：重成功率，放宽自拒启动
        res = config_fallback(res, hc, a.last_good)
        if res["verdict"] == "adopt":
            sys.stderr.write("WARN ground_calib: fit refused, %s mount used: %s\n"
                             % (res["adopted"]["source"], "; ".join(res["warnings"])))
    res["capture"] = meta
    res["seconds"] = round(time.monotonic() - t0, 3)
    with open(a.out, "w") as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    print(json.dumps({k: res.get(k) for k in ("verdict", "reasons", "warnings", "configured", "fitted", "adopted",
                                               "height_cell", "mount_tf", "alert", "seconds")}, ensure_ascii=False))
    return EXIT_OK if res["verdict"] in ("adopt", "confirm") else EXIT_REFUSED


if __name__ == "__main__":
    sys.exit(main())
