"""Analytic fixtures; oracle boxes are labeled and never used as detector scores."""
import numpy as np
from .core import Frame, Detection

HALF_M = 0.09


def scene(index, kind="present", camera_x=0.0, pose_error=0.0, suppress=False):
    h, w = 120, 160
    K = np.array([[120., 0, 80], [0, 120, 60], [0, 0, 1]])
    T = np.eye(4); T[0, 3] = camera_x+pose_error
    rgb = np.full((h, w, 3), 170, np.uint8)
    depth = np.full((h, w), 3., np.float32)
    # Known red object occupies world X/Y +/-HALF_M at Z=2m (cup-sized since 10-16: the
    # memory's class size gate rejects the earlier 0.4 m "cup").
    left, right = int(80+120*(-HALF_M-camera_x)/2), int(80+120*(HALF_M-camera_x)/2)
    top, bottom = int(60-120*HALF_M/2), int(60+120*HALF_M/2)
    bbox = (left, top, right, bottom)
    if kind == "present":
        depth[top:bottom, max(left, 0):min(right, w)] = 2
        rgb[top:bottom, max(left, 0):min(right, w)] = [230, 20, 20]
    elif kind == "occluded":
        depth[30:90, 30:130] = 1
        rgb[30:90, 30:130] = [20, 20, 230]
    elif kind == "invalid_depth":
        depth[:] = np.nan
    elif kind not in ("removed", "out_of_view", "invalid_pose"):
        raise ValueError(kind)
    if kind == "out_of_view":
        T[0, 3] = 20
    f = Frame(f"synthetic-{index}", float(index), rgb, depth, K, T,
              kind != "invalid_pose", "synthetic_known_camera", "synthetic_pinhole", float(index))
    detections = [Detection("cup", 1., bbox, "synthetic_oracle_box", f.frame_id, f.timestamp)] if kind == "present" and not suppress else []
    return f, detections


def scenarios():
    for name in ("out_of_view", "occluded", "removed", "invalid_depth", "invalid_pose", "detector_miss", "repeat_view", "reconfirm"):
        first = scene(0)
        kind = "present" if name in ("detector_miss", "reconfirm") else "removed" if name == "repeat_view" else name
        frames = [first, scene(1, kind, suppress=name=="detector_miss"),
                  scene(2, kind, camera_x=0.0 if name=="repeat_view" else 0.12, suppress=name=="detector_miss")]
        if name == "reconfirm":
            frames = [first, scene(1, "out_of_view"), scene(2, "present", camera_x=0.12)]
        yield name, frames
