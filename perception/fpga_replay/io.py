"""Canonical JSONL and TUM RGB-D adapters. Ground truth is explicit input provenance."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from .core import Frame


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1024*1024), b""):
            h.update(block)
    return h.hexdigest()


def read_table(path):
    rows = [line.split() for line in Path(path).read_text().splitlines()
            if line.strip() and not line.lstrip().startswith("#")]
    if not rows:
        raise ValueError(f"empty table: {path}")
    stamps = np.array([float(row[0]) for row in rows])
    if not np.isfinite(stamps).all() or np.any(np.diff(stamps) <= 0):
        raise ValueError(f"timestamps must be finite and strictly increasing: {path}")
    return rows, stamps


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-8:
        raise ValueError("invalid quaternion")
    x, y, z, w = q/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def interpolate_pose(stamp, rows, times, max_gap=0.05):
    index = int(np.searchsorted(times, stamp))
    if index < len(times) and times[index] == stamp:
        row = np.asarray(rows[index][1:], float)
        T = np.eye(4); T[:3, :3] = quaternion_matrix(row[3:]); T[:3, 3] = row[:3]
        return T, True
    if index == 0 or index == len(times):
        return np.eye(4), False
    a, b = np.array(rows[index-1][1:], float), np.array(rows[index][1:], float)
    if times[index]-times[index-1] > max_gap:
        return np.eye(4), False
    alpha = (stamp-times[index-1])/(times[index]-times[index-1])
    qa, qb = a[3:]/np.linalg.norm(a[3:]), b[3:]/np.linalg.norm(b[3:])
    dot = float(qa @ qb)
    if dot < 0:
        qb = -qb; dot = -dot
    if dot > 0.9995:
        q = (1-alpha)*qa+alpha*qb
    else:
        theta = np.arccos(np.clip(dot, -1, 1))
        q = (np.sin((1-alpha)*theta)*qa+np.sin(alpha*theta)*qb)/np.sin(theta)
    T = np.eye(4); T[:3, :3] = quaternion_matrix(q); T[:3, 3] = (1-alpha)*a[:3]+alpha*b[:3]
    return T, True


def tum_records(root, stride=15, limit=30):
    root = Path(root).resolve()
    if stride < 1 or limit < 1:
        raise ValueError("stride and limit must be positive")
    rgb, _ = read_table(root/"rgb.txt"); depths, dt = read_table(root/"depth.txt")
    poses, pt = read_table(root/"groundtruth.txt")
    used = set(); records = []
    for rgbrow in rgb[::stride]:
        stamp = float(rgbrow[0]); i = int(np.argmin(np.abs(dt-stamp)))
        if abs(dt[i]-stamp) > 0.02 or i in used:
            continue
        used.add(i)
        T, valid = interpolate_pose(stamp, poses, pt)
        # TUM recommends ROS defaults for the already registered, warped depth images.
        rec = {"frame_id": rgbrow[0], "timestamp": stamp, "depth_timestamp": float(dt[i]),
               "rgb": rgbrow[1], "depth": depths[i][1], "depth_scale": 5000.0,
               "depth_kind": "optical_z", "K": [[525, 0, 319.5], [0, 525, 239.5], [0, 0, 1]],
               "T_world_camera": T.tolist(), "pose_valid": valid,
               "pose_source": "TUM_mocap_interpolated_not_SLAM",
               "calibration_id": "TUM_registered_ROS_defaults_no_extra_depth_correction",
               "imu": None, "encoder": None}
        for field in ("rgb", "depth"):
            rec[field+"_sha256"] = sha256(root/rec[field])
        records.append(rec)
        if len(records) == limit:
            break
    if not records:
        raise ValueError("no associated RGB-D frames")
    return records


def load_frame(record, root):
    root = Path(root).resolve()
    paths = {}
    for name in ("rgb", "depth"):
        path = (root/record[name]).resolve()
        try:
            path.relative_to(root)
        except ValueError:
            raise ValueError("image path escapes dataset root") from None
        if record.get(name+"_sha256") != sha256(path):
            raise ValueError(f"{name} checksum mismatch or missing checksum")
        paths[name] = str(path)
    if record.get("depth_kind") != "optical_z":
        raise ValueError("adapter must convert ray range to optical Z")
    scale = float(record["depth_scale"])
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("positive finite depth scale required")
    bgr = cv2.imread(paths["rgb"], cv2.IMREAD_COLOR)
    depth = cv2.imread(paths["depth"], cv2.IMREAD_UNCHANGED)
    if bgr is None or depth is None or depth.ndim != 2:
        raise ValueError("cannot decode RGB/depth pair")
    depth = depth.astype(np.float32)/scale; depth[depth <= 0] = np.nan
    frame = Frame(str(record["frame_id"]), float(record["timestamp"]), cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB),
                  depth, np.asarray(record["K"], float), np.asarray(record["T_world_camera"], float),
                  record["pose_valid"], record["pose_source"], record["calibration_id"],
                  record["depth_timestamp"], record.get("imu"), record.get("encoder"))
    frame.validate()
    return frame


def load_records(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
