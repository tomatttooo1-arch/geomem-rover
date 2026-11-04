#!/usr/bin/env python3
"""Camera mount of a TUM RGB-D robot sequence from its own groundtruth (2026-10-16, host replay fixture).

Why: the host replay of TUM fr2/pioneer_360 used the real car's mount (0.36 m, 30 deg down) for base_link -> camera,
but that Kinect looks only about 5.7 deg down. base_link was then about 24 deg off the floor, and the 3-DoF projection
of the 6-DoF visual odometry (rtabmap Reg/Force3DoF on the graph, rgbd_odometry 6-DoF) made the map->odom correction's
yaw swing 26-29 deg with 8-12 deg steps at fixed places of the sequence (gate_dig/JUMP_DIG.md). A replay of another
dataset must use that dataset's own extrinsic: this derives it from groundtruth.txt, so the fixture never carries a
hand-copied number.

Method (numpy only, Python 3.8):
  groundtruth.txt rows "t tx ty tz qx qy qz qw" = the colour camera's optical frame in the mocap world.
  The robot turns about the floor normal, so the rotation axis of the camera's motion IS base_link's z. The world
  rotation axis = mean of the unit axes of the relative rotations over WINDOW_S windows turning more than MIN_TURN_DEG
  (sign-aligned upward); expressed in the optical frame per sample, median over the sequence -> the up direction u.
  pitch, roll from u with the convention of tools/car_geo_producer/common.base_camera
  (u = (-sin(roll) cos(pitch), -cos(roll) cos(pitch), -sin(pitch)); tools/car_run/ground_calib.pose_from_plane).
  height = median groundtruth z (mocap world z; the floor is not guaranteed to be z = 0 -- height does not enter
  the 3-DoF yaw mechanism; --height-m overrides). x, y of the lens over base_link are not observable this way and
  stay common.MOUNT_XYZ (the car's convention), like the yaw of the camera about the axis (0: base x = the optical
  axis projected on the floor).
  The check that pins sign and size of both angles at once: the axis mapped through the mount's rotation must be
  base z (residual_tilt_deg; the car's 30 deg mount gives about 24 deg on pioneer_360).

CLI:
  tum_mount.py SEQ_DIR [--height-m H]          -> stdout "x y z qx qy qz qw" (rtabmap_car.sh CAM_MOUNT_TF), the
                                                  summary (pitch, roll, height, residual tilt) on stderr
  tum_mount.py SEQ_DIR --check-tf "x y z qx qy qz qw" [--max-tilt-deg 1.0]
                                               -> exit 0 when the given mount's base z is within the limit of the
                                                  dataset's rotation axis, else exit 2 (the launch's fail-closed check)
  --json FILE: the summary as JSON (adds the check's tilt with --check-tf).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.car_geo_producer.common import MOUNT_XYZ, base_camera  # noqa: E402

WINDOW_S = 0.5          # relative rotation window
MIN_TURN_DEG = 5.0      # windows turning less are too noisy for an axis
STEP = 50               # every 50th groundtruth row starts a window (the mocap runs at ~300 Hz)
MAX_TILT_DEG = 1.0      # --check-tf default: the given mount's base z vs the dataset's rotation axis
EXIT_OK, EXIT_REFUSED = 0, 2


def quat_matrix(q):
    """(x, y, z, w) -> 3x3 rotation (normalised)."""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_quat(R):
    """3x3 rotation -> (x, y, z, w), w >= 0 (Shepperd)."""
    R = np.asarray(R, dtype=np.float64)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)
    q = np.array(q)
    q = q / np.linalg.norm(q)
    return tuple(float(v) for v in (q if q[3] >= 0 else -q))


def rotvec(R):
    """3x3 rotation -> rotation vector (axis * angle)."""
    c = max(-1.0, min(1.0, (R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2))
    a = math.acos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if a < 1e-9:
        return v / 2
    if a > math.pi - 1e-6:   # near 180 deg: from the symmetric part
        k = int(np.argmax(np.diag(R)))
        ax = (R[:, k] + np.eye(3)[k]) / math.sqrt(2 * (1 + R[k, k]))
        return ax * a
    return v / (2 * math.sin(a)) * a


def load_groundtruth(path):
    """groundtruth.txt -> (t (N,), p (N, 3), Rs (N, 3, 3) world <- optical)."""
    with open(path) as f:
        rows = [l.split() for l in f if l.strip() and not l.startswith('#')]
    a = np.array(rows, dtype=np.float64)
    if a.ndim != 2 or a.shape[1] != 8 or len(a) < 10:
        raise ValueError("%s: expected rows 't tx ty tz qx qy qz qw'" % path)
    return a[:, 0], a[:, 1:4], np.array([quat_matrix(q) for q in a[:, 4:8]])


def rotation_axis(t, Rs, window_s=WINDOW_S, min_turn_deg=MIN_TURN_DEG, step=STEP):
    """-> (world axis (3,), axis in the optical frame (3,) median over all samples, windows used, p90 spread deg)."""
    axes = []
    for i in range(0, len(t), step):
        j = int(np.searchsorted(t, t[i] + window_s))
        if j >= len(t):
            break
        rv = rotvec(Rs[j] @ Rs[i].T)          # world-frame rotation from i to j
        n = float(np.linalg.norm(rv))
        if n > math.radians(min_turn_deg):
            axes.append(rv / n)
    if len(axes) < 3:
        raise ValueError("fewer than 3 windows turn more than %.1f deg: no rotation axis" % min_turn_deg)
    A = np.array(axes)
    ref = A[np.argmax(np.abs(A[:, 2]))]
    A = A * np.sign(A @ ref)[:, None]        # one hemisphere
    w = A.mean(0)
    w /= np.linalg.norm(w)
    if w[2] < 0:                             # upward in the mocap world (z up)
        w = -w
        A = -A
    spread = float(np.percentile(np.degrees(np.arccos(np.clip(A @ w, -1, 1))), 90))
    ac = np.median(np.einsum('nji,j->ni', Rs, w), axis=0)   # R^T w per sample
    ac /= np.linalg.norm(ac)
    return w, ac, len(A), spread


def angles_from_up(u):
    """the up direction u in the optical frame -> (pitch_deg, roll_deg), common.base_camera's convention."""
    u = np.asarray(u, dtype=np.float64)
    u = u / np.linalg.norm(u)
    return math.degrees(math.asin(max(-1.0, min(1.0, -u[2])))), math.degrees(math.atan2(-u[0], -u[1]))


def tf_rotation(tf):
    """"x y z qx qy qz qw" -> (xyz, R base <- optical)."""
    v = [float(x) for x in (tf.split() if isinstance(tf, str) else tf)]
    if len(v) != 7:
        raise ValueError("mount TF must be 'x y z qx qy qz qw'")
    return v[:3], quat_matrix(v[3:])


def residual_tilt_deg(R_bc, up_c):
    """angle between base z and the dataset's up axis mapped through the mount rotation base <- optical."""
    a = np.asarray(R_bc) @ np.asarray(up_c, dtype=np.float64)
    return math.degrees(math.acos(max(-1.0, min(1.0, abs(a[2]) / np.linalg.norm(a)))))


def mount_tf(height_m, pitch_deg, roll_deg):
    T = base_camera((MOUNT_XYZ[0], MOUNT_XYZ[1], height_m), pitch_deg, roll_deg)
    return tuple(float(v) for v in T[:3, 3]) + matrix_quat(T[:3, :3]), T[:3, :3]


def tf_string(tf):
    return " ".join("%.9g" % v for v in tf)


def yaw_projection_error(Rs, R_bc, up_w):
    """The fixture defect, zero free parameters: base <- world rotations B_k = R_k R_bc^T; the odometry O_k = B_0^T B_k
    projected to 3-DoF keeps its ZYX yaw. Its error against the true heading change (the rotation angle about the
    dataset's up axis) -> per-sample error (deg, unwrapped, relative to sample 0)."""
    B = np.einsum('nij,kj->nik', Rs, R_bc)          # world <- base
    out = []
    for k in range(len(Rs)):
        O = B[0].T @ B[k]
        yaw = math.atan2(O[1, 0], O[0, 0])
        true = float(rotvec(Rs[k] @ Rs[0].T) @ up_w)  # signed turn about up (world frame)
        out.append(yaw - true)
    e = np.degrees(np.unwrap(np.array(out)))
    return e - e[0]


def derive(seq_dir, height_m=None):
    t, p, Rs = load_groundtruth(os.path.join(seq_dir, 'groundtruth.txt'))
    w, ac, n, spread = rotation_axis(t, Rs)
    pitch, roll = angles_from_up(ac)
    gt_z = float(np.median(p[:, 2]))
    h = gt_z if height_m is None else float(height_m)
    tf, R_bc = mount_tf(h, pitch, roll)
    return {"seq": os.path.abspath(seq_dir), "pitch_deg": pitch, "roll_deg": roll, "height_m": h,
            "height_source": "groundtruth z median" if height_m is None else "--height-m",
            "groundtruth_z_median_m": gt_z, "axis_world": [float(x) for x in w],
            "axis_vs_world_z_deg": math.degrees(math.acos(min(1.0, abs(w[2])))),
            "up_optical": [float(x) for x in ac], "windows": n, "axis_spread_p90_deg": spread,
            "residual_tilt_deg": residual_tilt_deg(R_bc, ac), "mount_tf": tf_string(tf),
            "method": "tools/nano_ros2/tum_mount.py (rotation axis of the groundtruth camera motion)"}, (t, Rs, w, ac)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("seq_dir")
    ap.add_argument("--height-m", type=float, default=None)
    ap.add_argument("--check-tf", default=None)
    ap.add_argument("--max-tilt-deg", type=float, default=MAX_TILT_DEG)
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    try:
        info, (_t, _Rs, _w, ac) = derive(a.seq_dir, a.height_m)
    except (OSError, ValueError) as exc:
        sys.stderr.write("tum_mount: %s\n" % exc)
        return EXIT_REFUSED
    rc = EXIT_OK
    if a.check_tf is not None:
        try:
            _xyz, R = tf_rotation(a.check_tf)
        except ValueError as exc:
            sys.stderr.write("tum_mount: %s\n" % exc)
            return EXIT_REFUSED
        tilt = residual_tilt_deg(R, ac)
        info.update({"check_tf": a.check_tf, "check_tilt_deg": tilt, "max_tilt_deg": a.max_tilt_deg})
        ok = tilt <= a.max_tilt_deg
        sys.stderr.write("tum_mount: given mount's base z is %.2f deg off the dataset's rotation axis (limit %.2f): %s\n"
                         % (tilt, a.max_tilt_deg, "ok" if ok else "REFUSED"))
        rc = EXIT_OK if ok else EXIT_REFUSED
    else:
        print(info["mount_tf"])
    sys.stderr.write("tum_mount: pitch %.3f deg down, roll %.3f deg, height %.4f m (%s), residual tilt %.3f deg, "
                     "%d windows\n" % (info["pitch_deg"], info["roll_deg"], info["height_m"], info["height_source"],
                                       info["residual_tilt_deg"], info["windows"]))
    if a.json:
        with open(a.json, "w") as f:
            json.dump(info, f, indent=1, sort_keys=True)
    return rc


if __name__ == "__main__":
    sys.exit(main())
