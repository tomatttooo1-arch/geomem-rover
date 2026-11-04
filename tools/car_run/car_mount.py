#!/usr/bin/env python3
"""Line E, live preset (2026-10-14): the camera mount of the real car -> the base_link -> camera optical frame
transform and the producer's --height-cell.

owner 10-14 (bench, measured; the car is mounted the same way): lens centre 0.36 m above the ground, 30 deg down.
tools/car_geo_producer/common.py keeps its 0.35 m / 27 deg default (replay and host runs stay comparable); this
only builds the live car's mount from those numbers with the same base_camera() transform (x, y from
common.MOUNT_XYZ: 0.10 m ahead of the base origin, centred).

--height-cell: given, it is only checked; not given, the smallest height_cell from the producer default (7) up
whose follow block holds the ground with one voxel below it (host.geo_producer.check_block_height, the very check
the producer makes on its first frame): 0.31-0.34 m -> 7, 0.36-0.39 m -> 8, >= 0.40 m does not fit
(docs/design/CAR_GEO_PRODUCER_RESULTS_20261011.md section 4).

Prints one line "height_cell x y z qx qy qz qw" (the static transform base_link -> camera optical frame);
exit 2 with the reason on stderr when the block cannot hold the ground.

--height-m H --robot-height-m R (owner 10-16): only the start-up hard check camera_above_band -- the camera must be
>= R + 0.05 (the obstacle band's upper edge, D-035) + 0.005 m; exit 0 with a summary on stdout, else exit 2 with the
readable reason on stderr. Python 3.8 compatible."""
import argparse
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.geo_producer import DEFAULT_HEIGHT_CELL, DEFAULT_SHAPE, check_block_height  # noqa: E402
from tools.car_geo_producer.common import MOUNT_XYZ, base_camera  # noqa: E402


def quaternion(R):
    """rotation matrix -> (x, y, z, w), as tools/car_geo_producer/replay_camera.tf_msg."""
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    if w > 1e-6:
        return ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)
    x = math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0
    return (x, (R[0, 1] + R[1, 0]) / (4 * x), (R[0, 2] + R[2, 0]) / (4 * x), (R[2, 1] - R[1, 2]) / (4 * x))


def mount(height_m, pitch_deg, height_cell=None, ground_z_m=0.0, roll_deg=0.0):
    """(height_cell, (x, y, z), (qx, qy, qz, qw)) or raises ValueError (block cannot hold the ground).
    roll_deg (10-15, tools/car_run/ground_calib.py): about the camera's forward axis, > 0 = the camera's right side
    lower (common.base_camera); 0 (default) = the transform before, byte for byte."""
    if not 0.0 < height_m < 2.0 or not -90.0 < pitch_deg < 90.0:
        raise ValueError("camera height %.3f m / pitch %.1f deg out of range" % (height_m, pitch_deg))
    if not -45.0 < roll_deg < 45.0:
        raise ValueError("camera roll %.1f deg out of range" % roll_deg)
    if height_cell is None:
        hc = DEFAULT_HEIGHT_CELL
        ok, info = check_block_height(height_m, ground_z_m, shape=DEFAULT_SHAPE, height_cell=hc)
        if not ok and "need_height_cell_at_least" in info:
            hc = max(hc, info["need_height_cell_at_least"])
    else:
        hc = int(height_cell)
    ok, info = check_block_height(height_m, ground_z_m, shape=DEFAULT_SHAPE, height_cell=hc)
    if not ok:
        raise ValueError("camera %.3f m: height_cell %d does not hold the ground in the %s block (%s)"
                         % (height_m, hc, "x".join(map(str, DEFAULT_SHAPE)), info))
    T = base_camera((MOUNT_XYZ[0], MOUNT_XYZ[1], height_m), pitch_deg, roll_deg)
    return hc, tuple(float(v) for v in T[:3, 3]), tuple(float(v) for v in quaternion(T[:3, :3]))


# owner 10-16: the real car's body height (the obstacle band's upper edge = this + BAND_CLEARANCE_M, D-035;
# host/rolling_nav_layer.HeightBand clearance_m). The camera must stand CAMERA_MARGIN_M above that edge, else the band
# reaches the lens and the columns ahead can never be seen FREE (the start is refused: camera_above_band).
CAR_ROBOT_HEIGHT_M = 0.31
BAND_CLEARANCE_M = 0.05
CAMERA_MARGIN_M = 0.005
_UM = 1000000   # compared in integer micrometres (as HeightBand): 0.31 + 0.05 + 0.005 is not 0.365 in floats


def camera_above_band(cam_height_m, robot_height_m, clearance_m=BAND_CLEARANCE_M, margin_m=CAMERA_MARGIN_M):
    """Start-up hard check (owner 10-16, car_inner.sh after the ground calibration, before the mount TF): the camera
    height in use must be >= robot_height + clearance (the obstacle band's upper edge) + margin.
    -> (ok, text): text is the readable reason when refused, a one-line summary when ok.
    Raises ValueError for a non-finite / non-positive height (a configuration error, not a camera too low)."""
    for name, v in (("cam_height_m", cam_height_m), ("robot_height_m", robot_height_m),
                    ("clearance_m", clearance_m), ("margin_m", margin_m)):
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError("%s must be a finite number of metres (given %r)" % (name, v))
    if cam_height_m <= 0 or robot_height_m <= 0 or clearance_m < 0 or margin_m < 0:
        raise ValueError("heights must be positive, clearance and margin nonnegative")
    cam = int(round(cam_height_m * _UM))
    top = int(round(robot_height_m * _UM)) + int(round(clearance_m * _UM))
    need = top + int(round(margin_m * _UM))
    if cam >= need:
        return True, ("camera %.4f m >= obstacle band top %.4f m (robot height %.4f + %.3f) + margin %.3f m = %.4f m"
                      % (cam_height_m, top / _UM, robot_height_m, clearance_m, margin_m, need / _UM))
    return False, ("相机太低：障碍带上沿 %.3f m（车身高度 %.3f m + %.2f m）离镜头 %.4f m 不足 %.0f mm 余量"
                   "（要求镜头 >= %.4f m），障碍带碰到镜头，前方无法判空；抬高相机，或核对车身高度 CAR_ROBOT_HEIGHT"
                   % (top / _UM, robot_height_m, clearance_m, cam_height_m, margin_m * 1000, need / _UM))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--height-m", type=float, required=True)
    ap.add_argument("--pitch-deg", type=float, default=None, help="required except with --robot-height-m")
    ap.add_argument("--height-cell", default="", help="empty = derived from the height")
    ap.add_argument("--robot-height-m", type=float, default=None,
                    help="only the camera-above-band check (camera_above_band): exit 0 + summary, 2 + reason")
    a = ap.parse_args(argv)
    if a.robot_height_m is not None:
        try:
            ok, text = camera_above_band(a.height_m, a.robot_height_m)
        except ValueError as exc:
            sys.stderr.write("camera above band: %s\n" % exc)
            return 2
        (sys.stdout if ok else sys.stderr).write(text + "\n")
        return 0 if ok else 2
    if a.pitch_deg is None:
        ap.error("--pitch-deg is required")
    try:
        hc, xyz, q = mount(a.height_m, a.pitch_deg, int(a.height_cell) if a.height_cell else None)
    except ValueError as exc:
        sys.stderr.write("camera mount: %s\n" % exc)
        return 2
    print(" ".join([str(hc)] + ["%.9g" % v for v in xyz + q]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
