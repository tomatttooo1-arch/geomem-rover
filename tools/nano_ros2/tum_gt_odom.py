#!/usr/bin/env python3
"""Groundtruth odometry of a TUM RGB-D robot sequence, for building a distortion-free prior library (2026-10-17).

Why (定位门第 11 棒, owner 10-17 12:31): the host gate's safety criterion S compares the poses the gate releases with
the TUM groundtruth, but the prior library itself was built with visual odometry and sits 0.10 m (p50) / 0.46 m (p99)
off groundtruth (gate_dig/host_g8 pose_dev `map_floor_nodes`), so S measured the library more than the gate. The fix
is a fixture change: build the library with the groundtruth as rtabmap's odometry (tools/car_run/rtabmap_car.sh
MODE=map ODOM=wheel takes odom -> base_link from TF; tools/nano_ros2/tum_rgbd_pub.py --gt-odom-tf publishes it from
this module), so the library's node poses are groundtruth.

Pose convention = the S instrument's (gate_dig/host_g8/pose_dev.py class GT), so that a library built from this
odometry reads ~0 against that instrument by construction:
  groundtruth.txt rows "t tx ty tz qx qy qz qw" = the colour camera's optical frame in the mocap world;
  base_link = camera x inverse(mount) with the fixture's mount TF "x y z qx qy qz qw" (base_link -> camera,
  tools/nano_ros2/tum_mount.py); projected to 3-DoF: x, y of the base origin, yaw = atan2(R[1,0], R[0,0]) of the
  base x axis; yaw unwrapped over the sequence; x, y and yaw linearly interpolated at a stamp.
  odom -> base_link at stamp t = SE2(anchor)^-1 * SE2(base(t)): the odom frame is base_link at the anchor stamp (the
  first published frame), z = 0, level (rtabmap Reg/Force3DoF on the car / the fixture).
Stamps outside the groundtruth span are not covered (no extrapolation): the publisher sends no TF for them and
rtabmap skips those frames (no odometry), exactly like a wheel odometry that has not started.
numpy only (Python 3.8).
"""
import math

import numpy as np


def quat_R(x, y, z, w):
    """Unit quaternion -> 3x3 rotation (same expression as pose_dev.py)."""
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T(R, t):
    m = np.eye(4)
    m[:3, :3] = R
    m[:3, 3] = t
    return m


def proj(m):
    """4x4 pose -> (x, y, yaw of the x axis)."""
    return np.array([m[0, 3], m[1, 3], math.atan2(m[1, 0], m[0, 0])])


def se2(p):
    c, s = math.cos(p[2]), math.sin(p[2])
    return np.array([[c, -s, p[0]], [s, c, p[1]], [0, 0, 1.]])


def se2_inv(p):
    """(x, y, yaw) of SE2(p)^-1."""
    c, s = math.cos(p[2]), math.sin(p[2])
    return np.array([-(c * p[0] + s * p[1]), s * p[0] - c * p[1], -p[2]])


def compose(a, b):
    """(x, y, yaw) of SE2(a) * SE2(b); yaw not wrapped (a + b)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def parse_tf(mount_tf):
    v = [float(x) for x in str(mount_tf).split()]
    if len(v) != 7:
        raise ValueError("mount TF must be 'x y z qx qy qz qw' (given %r)" % (mount_tf,))
    n = math.sqrt(sum(q * q for q in v[3:7]))
    if not (abs(n - 1.) < 1e-3):
        raise ValueError('mount TF quaternion not unit (norm %.6f)' % n)
    return v


def load_rows(path):
    with open(path) as f:
        rows = [l.split() for l in f if l.strip() and not l.startswith('#')]
    a = np.array(rows, dtype=np.float64)
    if a.ndim != 2 or a.shape[1] != 8 or len(a) < 2:
        raise ValueError("%s: expected rows 't tx ty tz qx qy qz qw'" % path)
    if np.any(np.diff(a[:, 0]) < 0):   # equal neighbours occur (fr2/pioneer_360: 3, 4-decimal stamps); np.interp
        raise ValueError('%s: stamps decreasing' % path)   # takes them as pose_dev.py does
    return a


class GroundtruthBase:
    """3-DoF base_link poses in the mocap world from groundtruth rows and the fixture's mount TF."""

    def __init__(self, rows, mount_tf):
        v = parse_tf(mount_tf)
        T_cam_base = np.linalg.inv(T(quat_R(*v[3:7]), v[:3]))
        rows = np.asarray(rows, dtype=np.float64)
        self.t = rows[:, 0]
        p = np.array([proj(T(quat_R(*r[4:8]), r[1:4]) @ T_cam_base) for r in rows])
        p[:, 2] = np.unwrap(p[:, 2])
        self.p = p

    @classmethod
    def from_file(cls, path, mount_tf):
        return cls(load_rows(path), mount_tf)

    def covers(self, t):
        return bool(self.t[0] <= t <= self.t[-1])

    def at(self, t):
        """(x, y, yaw) at stamp t (inside the span; raises outside)."""
        if not self.covers(t):
            raise ValueError('stamp %.6f outside groundtruth [%.6f, %.6f]' % (t, self.t[0], self.t[-1]))
        return np.array([float(np.interp(t, self.t, self.p[:, k])) for k in range(3)])


class GtOdom:
    """odom -> base_link from groundtruth, the odom frame = base_link at the anchor stamp."""

    def __init__(self, base, anchor_t):
        self.base = base
        self.anchor_t = float(anchor_t)
        self.anchor = base.at(self.anchor_t)
        self.anchor_inv = se2_inv(self.anchor)

    def covers(self, t):
        return self.base.covers(t)

    def at(self, t):
        """(x, y, yaw) of odom -> base_link at t; yaw wrapped to [-pi, pi)."""
        r = compose(self.anchor_inv, self.base.at(t))
        r[2] = wrap(r[2])
        return r


def yaw_quat(yaw):
    """(qx, qy, qz, qw) of a rotation about z."""
    return 0., 0., math.sin(yaw / 2.), math.cos(yaw / 2.)


def first_covered(stamps, base):
    """The first of the frame stamps inside the groundtruth span (None when none is)."""
    for t in stamps:
        if base.covers(t):
            return t
    return None
