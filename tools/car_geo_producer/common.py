"""Shared replay fixtures for the car geometry producer (host gate and board rig).

Prepared depth sets (rec30 / TUM: depth_u16.bin + meta.json) are fixed-camera recordings.
The rig gives them a car: a camera mounted on base_link (default: the recommended mount of
docs/team/CAMERA_MOUNT_FOR_dev2_20260927.md -- 0.35 m high, 27 deg down, 0.10 m ahead of the
base origin) and a synthetic odom trajectory that both translates and turns, so a pose that
is one frame late or a grid in the camera frame changes the odom grid (positive controls).
The depth does not match the motion (it is a recording); both arms of every comparison see
the same codes and the same poses, which is all an equivalence gate needs.
"""
import json
import math
import os

import numpy as np

MOUNT_XYZ = (0.10, 0.0, 0.35)
MOUNT_PITCH_DEG = 27.0
LEVEL = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])   # optical (z fwd, x right, y down) -> base


def load_prep(prep):
    meta = json.load(open(os.path.join(prep, "meta.json")))
    w, h, n = int(meta["width"]), int(meta["height"]), int(meta["frames"])
    depth = np.memmap(os.path.join(prep, meta["depth_file"]), dtype="<u2", mode="r")
    if depth.size != w * h * n:
        raise ValueError("depth file size disagrees with meta")
    return meta, depth.reshape(n, h, w)


def base_camera(xyz=MOUNT_XYZ, pitch_deg=MOUNT_PITCH_DEG, roll_deg=0.0):
    """base_link <- camera optical frame (4x4). pitch_deg > 0: the optical axis points that many degrees below level.

    roll_deg (2026-10-15, ground self-calibration; default 0 = the transform before, byte for byte -- the roll-free
    branch below is the old expression, not a multiplication by an identity):
    a rotation about the camera's own forward (optical z) axis, applied before the pitch, R = R_y(pitch) R_x(roll)
    LEVEL. Right-hand rule about the forward axis: roll_deg > 0 lowers the camera's RIGHT side (optical +x, base -y),
    i.e. clockwise as seen from behind the camera; the image horizon then tilts so that the ground rises on the
    image's left. Roll leaves the optical axis (and so the pitch) unchanged. Pinned by
    tests/test_ground_calib.py (render -> fit -> roll back with its sign)."""
    c, s = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
    pitch = np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])
    T = np.eye(4)
    if roll_deg:
        cr, sr = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
        roll = np.array([[1., 0., 0.], [0., cr, -sr], [0., sr, cr]])
        T[:3, :3] = pitch @ roll @ LEVEL
    else:
        T[:3, :3] = pitch @ LEVEL
    T[:3, 3] = xyz
    return T


def odom_base(k, speed_mps=0.2, hz=10.0):
    """Synthetic base pose of replay frame k: forward at speed_mps with a slow weave."""
    t = k / hz
    yaw = 0.35 * math.sin(t / 3.0)
    x = speed_mps * t
    y = 0.15 * math.sin(t / 4.0)
    T = np.eye(4)
    T[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    T[:3, 3] = (x, y, 0.0)
    return T


def yaw_of(T):
    return math.atan2(T[1, 0], T[0, 0])


def memory_message(snap, T_map_camera0, runtime_boot_id="replay-runtime", revision=1):
    """A /fpga/memory body (D.1 subset plan_frame reads) from a make_roi_memory snapshot whose
    boxes are in the camera frame of frame 0: corners mapped to map, axis-aligned hull."""
    tracks = []
    for t in snap["tracks"]:
        lo, hi = (np.asarray(c, dtype=np.float64) for c in t["bounds"])
        corners = np.array([[x, y, z, 1.0] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                            for z in (lo[2], hi[2])])
        w = (T_map_camera0 @ corners.T).T[:, :3]
        pos = (T_map_camera0 @ np.r_[np.asarray(t["position_m"], dtype=np.float64), 1.0])[:3]
        tracks.append({"id": t["id"], "label": t["label"], "bounds_m": [w.min(0).tolist(), w.max(0).tolist()],
                       "last_seen": float(t["last_seen"]), "position_m": pos.tolist()})
    return {"v": 1, "runtime_boot_id": runtime_boot_id, "revision": int(revision), "stamp_ros_s": 0.0,
            "world_frame": "map", "target_label": snap.get("target_label"), "tracks": tracks}
