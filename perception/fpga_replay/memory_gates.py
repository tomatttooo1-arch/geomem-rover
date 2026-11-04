"""Admission gates and drift-scaled association radius for the object memory (10-16).

docs/design/MEMORY_QUALITY_20261016.md. Three problems seen on the first real-car
run (progress/PROGRESS-22.md section 6) are handled before an observation may
create or update a memory track:

* below ground: an observation whose position is more than ``below_ground_tol_m``
  under the ground plane is dropped and counted;
* implausible size: the detector box, scaled to metres at the observation's own
  depth, must fit the class's largest plausible instance times ``size_margin``;
* clipped by the image border (small classes only): the box is a lower bound
  of the object, its size cannot be checked, so it may update an existing track
  but never create one.

The association radius grows with the odometry travelled since the track was
last updated (or, when the frame carries a pose standard deviation, with that),
capped at ``association_max_m``. Table values are a-priori object sizes, not
fitted to any recording; sources are recorded per row.
"""
import math

import numpy as np


# label -> ((max apparent width m, max apparent height m), may a clipped box create a track?, source)
# Width/height are the largest plausible real instance (upright where that is the
# normal pose); the gate multiplies them by Config.size_margin.
CLASS_MAX_APPARENT_M = {
    "bottle": ((.11, .33), False, "2 L 饮料瓶直径约 11 cm、高约 33 cm（助手常识估计，上限）；比赛瓶为农夫山泉 500 ml 6.5x22 cm"),
    "cup": ((.13, .20), False, "带把马克杯宽约 13 cm、保温杯高约 20 cm（助手常识估计，上限）"),
    "cell phone": ((.17, .17), False, "手机最长边约 17 cm，朝向未知两向都取最长边（助手估计）"),
    "mouse": ((.13, .13), False, "鼠标最长边约 13 cm，朝向未知（助手估计）"),
    "book": ((.30, .30), False, "A4 书最长边约 30 cm，朝向未知（助手估计）"),
    "backpack": ((.45, .55), True, "大号双肩包约 45x55 cm（助手估计）；大物体近处常被画面截断，允许截断框建条"),
    "keyboard": ((.50, .50), True, "全尺寸键盘长约 45-50 cm（助手估计）；允许截断框建条"),
    # D-044 (2026-10-21): a few indoor classes a demo may use. Classes not in this table skip the
    # size gate; since 10-28 they may NOT create from a clipped box (see may_create).
    "laptop": ((.42, .42), True, "17 寸笔记本展开约 40x30 cm，朝向未知两向都取最长边（助手估计）；允许截断框建条"),
    "chair": ((.70, 1.10), True, "带扶手办公椅宽约 70 cm、高约 110 cm（助手估计，上限）；大物体，允许截断框建条"),
    "remote": ((.25, .25), False, "遥控器最长边约 25 cm，朝向未知（助手估计）"),
    "scissors": ((.25, .25), False, "剪刀最长边约 25 cm，朝向未知（助手估计）"),
    "teddy bear": ((.60, .70), True, "大号毛绒熊约 60x70 cm（助手估计，上限）；允许截断框建条"),
    "bowl": ((.30, .30), False, "大碗直径约 30 cm，俯视/侧视都不超过直径（助手估计）"),
    "vase": ((.30, .50), False, "落地以外的花瓶宽约 30 cm、高约 50 cm（助手估计，上限）"),
    "umbrella": ((1.20, 1.20), True, "撑开的长柄伞直径约 1.1 m，收起长约 1 m（助手估计）；允许截断框建条"),
    "handbag": ((.45, .45), True, "大号手提包约 45x40 cm（助手估计）；允许截断框建条"),
}


def is_truncated(bbox, shape, margin_px):
    """True when the detector box touches the image border within ``margin_px``."""
    h, w = shape[:2]
    x1, y1, x2, y2 = map(float, bbox)
    return bool(x1 <= margin_px or y1 <= margin_px or x2 >= w - 1 - margin_px or y2 >= h - 1 - margin_px)


def apparent_extent(bbox, depth_m, K):
    """(width, height) in metres of the detector box on the plane at camera depth ``depth_m``."""
    x1, y1, x2, y2 = map(float, bbox)
    return (float((x2 - x1) * depth_m / K[0, 0]), float((y2 - y1) * depth_m / K[1, 1]))


def admission_reason(obs, ground_z_m, cfg):
    """None when the observation may enter the memory, else the rejection reason."""
    if cfg.below_ground_tol_m is not None and float(obs.position[2]) < ground_z_m - cfg.below_ground_tol_m:
        return "below_ground"
    extent = getattr(obs, "extent_m", None)
    row = CLASS_MAX_APPARENT_M.get(obs.label)
    if cfg.size_margin is not None and extent is not None and row is not None:
        (max_w, max_h), _, _ = row
        if extent[0] > max_w * cfg.size_margin or extent[1] > max_h * cfg.size_margin:
            return "size_exceeds_class"
    return None


def may_create(obs):
    """Clipped boxes (any image edge, the top one included) can update a track but not start one. Only the large
    classes the table marks True may start from a clipped box. 10-28 (owner): classes NOT in the table no longer
    may (before: they could) -- car run 20261028-164456-live: banana is not in the table and a box glued to the top
    image edge (y1 = 0..2) became a false banana track at z 0.46 m. The enabled set's unlisted classes (key, spoon,
    banana, apple, orange) are all small."""
    if not getattr(obs, "truncated", False):
        return True
    row = CLASS_MAX_APPARENT_M.get(obs.label)
    return row is not None and row[1]


def rotation_angle(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return float(math.acos(min(1., max(-1., c))))


def association_radius(track_odo, track_sigma, odo, sigma, cfg):
    """Gate radius between a track last updated at ``track_odo`` and a frame at ``odo``.

    ``odo`` is (path metres, turned radians) accumulated by the memory from the
    frame poses. With a pose standard deviation on both sides the radius is
    r0 + gate * sqrt(s_t^2 + s_o^2); otherwise r0 + k_m * dpath + k_rad * dturn.
    """
    if track_sigma is not None and sigma is not None:
        drift = cfg.pose_sigma_gate * math.hypot(track_sigma, sigma)
    else:
        drift = (cfg.drift_per_m * abs(odo[0] - track_odo[0]) +
                 cfg.drift_per_rad * abs(odo[1] - track_odo[1]))
    return min(cfg.association_max_m, cfg.association_m + drift)


def class_slack(label):
    """Extra association/merge room for large classes: half the class's widest instance beyond 0.1 m
    (bottle, cup: 0; chair 0.25 m). A chair's depth surface jumps between seat, back and legs."""
    row = CLASS_MAX_APPARENT_M.get(label)
    return 0.0 if row is None else max(0.0, row[0][0] / 2 - 0.1)


def box_iou(a, b):
    """Image overlap of two detector boxes (zero for invalid/empty boxes)."""
    ax, ay, ar, ab = map(float, a)
    bx, by, br, bb = map(float, b)
    inter = max(0., min(ar, br) - max(ax, bx)) * max(0., min(ab, bb) - max(ay, by))
    union = max(0., ar - ax) * max(0., ab - ay) + max(0., br - bx) * max(0., bb - by) - inter
    return inter / union if union > 0 else 0.


def dedupe(observations, cfg, iou=None):
    """One observation per object per frame: same label and closer than ``dedupe_m``.

    Two boxes on one bottle (10-16 real run: 3 cm apart, IoU 0.40) would
    otherwise both claim the same track and be dropped as ambiguous, or create
    twin tracks. A complete box wins over a clipped one, then the higher score.
    10-27 (identity_v2): ``iou`` also drops a box overlapping a kept one by more than ``iou`` -- two
    boxes of one bottle with different depths must not count as two co-visible objects.
    Returns (kept, dropped).
    """
    order = sorted(range(len(observations)),
                   key=lambda i: (bool(getattr(observations[i], "truncated", False)), -observations[i].score, i))
    kept, dropped = [], []
    for i in order:
        o = observations[i]
        if any(observations[k].label == o.label and
               (np.linalg.norm(observations[k].position - o.position) < cfg.dedupe_m or
                (iou is not None and box_iou(observations[k].bbox, o.bbox) > iou)) for k in kept):
            dropped.append(o)
        else:
            kept.append(i)
    return [observations[i] for i in sorted(kept)], dropped
