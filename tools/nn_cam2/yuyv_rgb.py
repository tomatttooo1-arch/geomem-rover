"""NN-CAM2: YUYV -> RGB888 in numpy, the arithmetic of PREP1 hw/prep/ref/yuyv_ref.c (general form; kset 0 JFIF =
libjpeg jdcolor.c, 1 BT.601 limited; ups 1 = libjpeg h2v1_fancy_upsample, 0 = chroma replicated). The UDP arm of the
NN-CAM2 equivalence test feeds this RGB to the Nano's own preprocessing (host/kv_colour.ColourSource); the local arm
does the same in C++ (tools/dpu_pipe/colour_prep.h). Checked against the C reference binary (check_prep.py)."""
import numpy as np

K = {0: (65536, 0, 91881, -22554, -46802, 116130), 1: (76309, 16, 104597, -25675, -53279, 132201)}


def _fancy(c):
    """c: [h, n] uint8 chroma -> [h, 2n] (libjpeg h2v1_fancy_upsample)."""
    c = c.astype(np.int32)
    h, n = c.shape
    out = np.empty((h, 2 * n), np.int32)
    out[:, 0] = c[:, 0]
    out[:, 1] = (c[:, 0] * 3 + c[:, 1] + 2) >> 2
    mid = c[:, 1:n - 1] * 3
    out[:, 2:2 * n - 2:2] = (mid + c[:, 0:n - 2] + 1) >> 2
    out[:, 3:2 * n - 2:2] = (mid + c[:, 2:n] + 2) >> 2
    out[:, 2 * n - 2] = (c[:, n - 1] * 3 + c[:, n - 2] + 1) >> 2
    out[:, 2 * n - 1] = c[:, n - 1]
    return out


def yuyv_to_rgb(yuyv, w, h, kset=0, ups=1):
    a = np.frombuffer(yuyv, np.uint8).reshape(h, w // 2, 4)
    y = np.empty((h, w), np.int64)
    y[:, 0::2], y[:, 1::2] = a[:, :, 0], a[:, :, 2]
    if ups:
        u, v = _fancy(a[:, :, 1]), _fancy(a[:, :, 3])
    else:
        u, v = np.repeat(a[:, :, 1], 2, axis=1), np.repeat(a[:, :, 3], 2, axis=1)
    u, v = u.astype(np.int64), v.astype(np.int64)
    ky, yoff, krv, kgu, kgv, kbu = K[kset]
    yk = ky * (y - yoff)
    rgb = np.empty((h, w, 3), np.uint8)
    rgb[..., 0] = np.clip((yk + krv * (v - 128) + 32768) >> 16, 0, 255)
    rgb[..., 1] = np.clip((yk + kgu * (u - 128) + kgv * (v - 128) + 32768) >> 16, 0, 255)
    rgb[..., 2] = np.clip((yk + kbu * (u - 128) + 32768) >> 16, 0, 255)
    return rgb
