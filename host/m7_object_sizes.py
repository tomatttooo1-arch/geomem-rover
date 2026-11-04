"""Per-class synthetic boxes for the mask-free region check (E1).

docs/design/OBJECT_ROI_SPEC_20261011.md section 7. A box is built around a
recorded object position when no detector mask exists for this frame.

Design rule: prefer a box that is too SMALL. A box larger than the real object
pulls in background behind the object, so an object that is still there can
be reported as "free" (dangerous). A box that is too small only makes more
results indeterminate (safe). Table sizes are therefore treated as upper
bounds and shrunk by ``shrink`` (default 0.8) before projection.
"""
import math

import numpy as np


DEFAULT_SHRINK = 0.8

# (width, depth, height) in metres. Source and confidence are recorded per row.
CLASS_SIZE_M = {
    "bottle": ((.065, .065, .22),
               "owner 10-11 口述：农夫山泉 500 ml 实物"),
    "cup": ((.075, .075, .16),
            "owner 10-11 口述：手边一只矮胖杯，比赛用杯未定（暂定）"),
    "cell phone": ((.075, .007, .16), "owner 10-11 实测口述：7.5x0.7x16 cm（宽x厚x高）"),
    "backpack": ((.30, .15, .45),
                 "owner 10-11 口述：耐克双肩包常见尺寸（估计）"),
    "book": ((.17, .03, .24), "占位：助手估计，未查证"),
    "mouse": ((.065, .11, .04), "占位：助手估计，未查证"),
    "keyboard": ((.44, .14, .03), "占位：助手估计，未查证"),
    # D-044 (2026-10-21): a few indoor classes a demo may use; typical (not largest) sizes, the
    # smaller choice where the pose varies (too small is safe). Others take DEFAULT_SIZE_M.
    "laptop": ((.30, .20, .02), "助手估计：合上的 13-14 寸本；展开时高约 22 cm，取合上（偏小）"),
    "chair": ((.35, .35, .45), "助手估计：只取座面（腿间透空，框大会吸进背景）"),
    "remote": ((.04, .15, .02), "助手估计：电视遥控器"),
    "scissors": ((.07, .17, .01), "助手估计：办公剪刀平放"),
    "teddy bear": ((.20, .15, .25), "助手估计：小号毛绒熊"),
    "bowl": ((.12, .12, .06), "助手估计：饭碗"),
    "vase": ((.10, .10, .20), "助手估计：桌面小花瓶"),
    "handbag": ((.25, .10, .20), "助手估计：小号手提包"),
}
DEFAULT_SIZE_M = ((.10, .10, .10), "占位：未知类别默认，助手估计")


def class_size(label):
    """Return ((w, d, h) metres, source note) for a label or the default."""
    return CLASS_SIZE_M.get(label, DEFAULT_SIZE_M)


def synthetic_bounds(label, position_m, up_axis, *, shrink=DEFAULT_SHRINK):
    """(2,3) lower/upper box centred on a recorded position.

    Heading is unknown, so both horizontal axes take min(width, depth) -- the
    smaller choice, per the "too small is safe" rule; the vertical axis
    (``up_axis`` 0/1/2, explicit) takes height. Every side is multiplied by
    ``shrink`` in (0, 1].
    """
    if type(up_axis) is not int or up_axis not in (0, 1, 2):
        raise ValueError("up_axis must be 0, 1 or 2")
    if (isinstance(shrink, bool) or not isinstance(shrink, (int, float)) or
            not math.isfinite(shrink) or not 0 < shrink <= 1):
        raise ValueError("shrink must be in (0, 1]; boxes may only get smaller")
    center = np.asarray(position_m, dtype=np.float64)
    if center.shape != (3,) or not np.isfinite(center).all():
        raise ValueError("position must be a finite 3-vector")
    (width, depth, height), _ = class_size(label)
    half = np.full(3, min(width, depth) / 2.)
    half[up_axis] = height / 2.
    half *= shrink
    return np.array([center - half, center + half])
