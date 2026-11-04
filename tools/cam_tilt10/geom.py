#!/usr/bin/env python3
"""Camera tilt 30 -> 10 deg (owner 10-17): field-of-view geometry on a flat floor, computed per pixel.
docs/design/CAM_TILT_10_20261017.md section 3.

  geom.py OUT.json
For each intrinsics set (the board's live depth camera_info -- 640x480 HW D2C, fx 517.17, the one the car runs --,
the ground-calibration test fixture gemini2_k 640x400 91x66 deg, and the old 640x360 D2C mode fx 344.8) and each
pitch (10, 30 at 0.375 m, and the board fit 7.05 deg at 0.3915 m; x 0.10 m ahead of base_link: car_mount.mount), every pixel's ray from
common.base_camera is intersected with the floor / with horizontal planes:
  * nearest floor seen (forward x from the camera; the image's lowest rays), and the lateral half-width there
  * the upper field edge's height at 1 m / 2 m ahead (centre column) and its elevation angle
  * low obstacles 5 / 10 cm tall: from which forward distance (of the box's top front edge) on they leave the bottom
    of the view as the car approaches; and the far limit by the upper edge (only when it points below level)
  * the per-frame block reach (FollowCameraGrid 32x32x10, back_cells 1, height_cell 8: the voxels a frame can vote)
Distances are given from the camera, from the physical car front (lens 0.007 m behind it, car.json
lens_to_front_m) and from the Nav2 footprint front (circle r 0.30 m around base_link = camera + 0.20 m; + body
seed margin 0.05). Python 3.8 compatible."""
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.car_geo_producer.common import MOUNT_XYZ, base_camera  # noqa: E402
from tools.car_geo_producer.mount_sweep import gemini2_k  # noqa: E402

K_BOARD = np.array([[517.1719360351562, 0.0, 324.2197265625], [0.0, 517.1170654296875, 240.64755249023438],
                    [0.0, 0.0, 1.0]])          # board ng3 1017 /camera/depth/camera_info (HW D2C to colour 640x480)
K_360 = np.array([[344.78, 0.0, 319.5], [0.0, 344.74, 179.5], [0.0, 0.0, 1.0]])   # 640x360 D2C (camera_mount_check)
LENS_TO_FRONT_M = 0.007       # car.json camera_mount.lens_to_front_m (owner 10-16)
FOOTPRINT_R_M = 0.30          # Nav2 / bridge robot_radius_m
BODY_MARGIN_M = 0.05
HEIGHT_M = 0.375


def intrinsics():
    kg, wg, hg = gemini2_k()
    return {"board_640x480_d2c": (K_BOARD, 640, 480), "fixture_gemini2_k_640x400": (kg, wg, hg),
            "old_640x360_d2c": (K_360, 640, 360)}


def rays_base(K, w, h, pitch, height=HEIGHT_M):
    u, v = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    d = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)], -1)
    T = base_camera((MOUNT_XYZ[0], MOUNT_XYZ[1], height), pitch)
    return d @ T[:3, :3].T, T


def elev(dirs):
    return np.degrees(np.arctan2(dirs[..., 2], np.hypot(dirs[..., 0], dirs[..., 1])))


def front_refs(x_cam):
    """a forward distance from the camera -> the same from the car front and from the footprint front"""
    return {"from_camera_m": round(x_cam, 3), "from_car_front_m": round(x_cam - LENS_TO_FRONT_M, 3),
            "from_footprint_front_m": round(x_cam - (FOOTPRINT_R_M - MOUNT_XYZ[0]), 3),
            "from_seed_edge_m": round(x_cam - (FOOTPRINT_R_M + BODY_MARGIN_M - MOUNT_XYZ[0]), 3)}


def one(K, w, h, pitch, height=HEIGHT_M):
    d, T = rays_base(K, w, h, pitch, height)
    e = elev(d)
    r = {"pitch_deg": pitch, "height_m": height,
         "vfov_deg": round(float(e[:, w // 2].max() - e[:, w // 2].min()), 2),
         "hfov_deg": round(float(np.degrees(np.arctan((w - 1 - K[0, 2]) / K[0, 0]) + np.arctan(K[0, 2] / K[0, 0]))), 2),
         "lower_edge_depression_deg": round(float(-e[h - 1, w // 2]), 2),
         "upper_edge_elevation_deg": round(float(e[0, w // 2]), 2)}
    # nearest floor (any pixel), forward x from the camera
    down = d[..., 2] < -1e-12
    t = np.where(down, -height / np.where(down, d[..., 2], -1.0), np.inf)
    fx = np.where(down, t * d[..., 0], np.inf)
    fy = np.where(down, t * d[..., 1], np.nan)
    i = np.unravel_index(np.argmin(fx), fx.shape)
    near = float(fx[i])
    row = h - 1
    r["nearest_floor"] = front_refs(near)
    r["nearest_floor_centre_column_m"] = round(float(fx[row, w // 2]), 3)
    r["floor_half_width_at_bottom_row_m"] = round(float(np.nanmax(np.abs(fy[row]))), 3)
    # how wide the floor is seen at the footprint front / seed edge lines (forward x from the camera)
    for name, xl in (("footprint_front", FOOTPRINT_R_M - MOUNT_XYZ[0]), ("seed_edge", FOOTPRINT_R_M + BODY_MARGIN_M - MOUNT_XYZ[0]),
                     ("0.5m", 0.5), ("1.0m", 1.0)):
        sel = down & (np.abs(fx - xl) < 0.01)
        r["floor_seen_half_width_at_%s_m" % name] = round(float(np.abs(fy[sel]).max()), 3) if sel.any() else None
    # upper edge height at 1 / 2 m ahead (centre column top pixel)
    top = d[0, w // 2]
    slope = top[2] / math.hypot(top[0], top[1])
    r["upper_edge_height_m"] = {"%.0fm" % x: round(height + x * slope, 3) for x in (1.0, 2.0)}
    # low boxes: as the car approaches, the top front edge (height hb) leaves the view when its depression angle
    # exceeds the lower edge's; far limit only if the upper edge points below level
    low = math.radians(-e[h - 1, w // 2])
    up = math.radians(e[0, w // 2])
    boxes = {}
    for hb in (0.05, 0.10):
        near_b = (height - hb) / math.tan(low)
        far_b = (height - hb) / math.tan(-up) if up < 0 else None
        boxes["%dcm" % round(hb * 100)] = {"leaves_view_closer_than": front_refs(near_b),
                                           "upper_edge_far_limit_from_camera_m": None if far_b is None else round(far_b, 3)}
    r["low_box_top_front_edge"] = boxes
    # obstacle band top 0.36 m (robot 0.31 + 0.05): from where is a point at 0.36 m in view
    bt = 0.36
    r["band_top_0.36_visible_from_camera_m"] = {"nearer_than_leaves": round((height - bt) / math.tan(low), 3),
                                                "farthest": None if up >= 0 else round((height - bt) / math.tan(-up), 3)}
    # the share of pixels whose ray ends on the floor inside 0.3-2.0 m (ground_calib's range), and inside the block
    hd = np.where(down, t * np.hypot(d[..., 0], d[..., 1]), np.inf)
    r["pixels_floor_0.3_2.0m_share"] = round(float(((hd >= 0.3) & (hd <= 2.0)).mean()), 4)
    r["pixels_floor_any_share"] = round(float(down.mean()), 4)
    r["pixels_above_level_share"] = round(float((d[..., 2] >= 0).mean()), 4)
    return r


def block_reach(height=HEIGHT_M):
    from host.geo_producer import DEFAULT_SHAPE, VOXEL_M, check_block_height
    from host.m7_grid_policy import FollowCameraGrid
    ok, info = check_block_height(height, 0.0, height_cell=8)
    T = base_camera((MOUNT_XYZ[0], MOUNT_XYZ[1], height), 10.0)
    g = FollowCameraGrid(VOXEL_M, DEFAULT_SHAPE, back_cells=1, height_cell=8).grid_for(T)
    nx, ny, nz = DEFAULT_SHAPE
    return {"block_ok": ok, "block_bottom_m": info["block_bottom_m"], "block_top_m": info["block_top_m"],
            "x_from_camera_m": [round(g.origin_m[0] - MOUNT_XYZ[0], 3), round(g.origin_m[0] + nx * VOXEL_M - MOUNT_XYZ[0], 3)],
            "y_m": [round(g.origin_m[1], 3), round(g.origin_m[1] + ny * VOXEL_M, 3)],
            "note": "camera at base_link origin; the block follows the camera cell, so the reach varies by < 1 cell"}


def main():
    out = sys.argv[1]
    res = {"height_m": HEIGHT_M, "mount_x_m": MOUNT_XYZ[0], "block": block_reach(), "sets": {}}
    for name, (K, w, h) in intrinsics().items():
        res["sets"][name] = {"K": K.tolist(), "size": [w, h], "pitch": {str(p): one(K, w, h, p) for p in (10.0, 30.0)}}
        # the board's fit after the owner's change (another session, runs/cam10-20261017 c10_1: 0.3915 m / 7.05 deg)
        res["sets"][name]["pitch"]["7.05@0.3915"] = one(K, w, h, 7.05, 0.3915)
    with open(out, "w") as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    print(json.dumps(res, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
