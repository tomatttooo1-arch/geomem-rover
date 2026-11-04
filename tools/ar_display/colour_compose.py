"""Numpy reference of the PS colour composition (AR colour main view, owner 10-15; 2026-10-16).

Written from the rule, NOT by translating tools/kv_link/ar_helper/m7ar_present.cpp (compose_colour):
  1. only a fresh picture (core mode 0) or a NODATA picture (mode 4: kv_server's no-depth self pull,
     all-zero depth, no geometry; 2026-10-17) with a colour frame of the depth size is composed;
  2. PiP first: the PL-rendered depth rectangle (pseudo colour + boxes + labels, as the PL wrote it)
     decimated by ``div`` -- rows dy, dy+div, ..., columns dx, dx+div, ... -- is copied to (pip_x, pip_y);
  3. then every depth-rectangle pixel that the PL overlay does NOT own is replaced by the colour pixel.
     The overlay mask comes from the registers / descriptor the PS itself wrote (same geometry as the
     core, AR_DISPLAY_SPEC): panel rows y < 40, each box's 2 px lines and its 48 x 12 label bar
     (above the box when y0 + dy >= 52, else inside it), the mini map 130 x 130 ring + inner and its
     12-row label above. A colour key is never used (label ink and invalid depth are both 0x000000).
Mini map 2x (owner 10-29), written from the rule as well: when the helper runs with --mini-scale 2 and
the frame is a colour-mode frame laid out for it (message flags b0 and b1), the box the PL drew for the
mini map -- 130 x 141 pixels from (mini_x - 1, mini_y - 12): 12-row label, 1 px ring, 128 x 128 inner --
is enlarged nearest-neighbour by the scale from the same top-left corner (after the colour composition,
on every flipped picture of such a frame). ``scale_mini``; ``mini_fits`` = the target lies on the canvas.
Python 3.8 / numpy 1.24 compatible.
"""
import numpy as np

PANEL = 40
NAMES = ["width", "height", "depth_width", "depth_height", "depth_dx", "depth_dy"]


def overlay_mask(regs, desc, shift_x=0):
    """H x W bool: pixels the core paints over the depth image. ``shift_x``: positive-control
    mutant only (the whole mask moved right by that many pixels)."""
    W, H = int(regs[0]), int(regs[1])
    dx, dy = int(regs[4]), int(regs[5])
    mx, my = int(regs[22]), int(regs[23])
    m = np.zeros((H, W), bool)
    m[:PANEL] = True
    n = int(desc[1]) if int(desc[0]) == 0x4D374152 else 0
    for i in range(min(n, 32)):
        x0, y0, x1, y1 = (int(v) for v in desc[2 + 6 * i: 6 + 6 * i])
        X0, X1, Y0, Y1 = x0 + dx, x1 + dx, y0 + dy, y1 + dy
        m[Y0:Y1 + 1, X0:min(X0 + 2, X1 + 1)] = True
        m[Y0:Y1 + 1, max(X1 - 1, X0):X1 + 1] = True
        m[Y0:min(Y0 + 2, Y1 + 1), X0:X1 + 1] = True
        m[max(Y1 - 1, Y0):Y1 + 1, X0:X1 + 1] = True
        LY = Y0 - 12 if Y0 >= 52 else Y0 + 2
        m[LY:LY + 12, X0:X0 + 48] = True
    m[max(0, my - 1):my + 129, max(0, mx - 1):mx + 129] = True
    m[max(0, my - 12):my, max(0, mx):mx + 128] = True
    if shift_x:
        out = np.zeros_like(m)
        out[:, shift_x:] = m[:, :W - shift_x]
        m = out
    return m


def rgb_to_xrgb(rgb, wd, hd):
    a = np.frombuffer(bytes(rgb), np.uint8).reshape(hd, wd, 3).astype(np.uint32)
    return (a[..., 0] << 16) | (a[..., 1] << 8) | a[..., 2]


def compose(pl_fb, regs, desc, rgb, pip, mutant=None):
    """``pl_fb``: H x W uint32 as the PL wrote it; ``rgb``: Wd*Hd*3 bytes; ``pip``: (x, y, div).
    Returns the composed frame (a copy). ``mutant`` (positive controls of the checker):
    "mask_shift" = mask one pixel to the right, "pip_row" = PiP from the odd rows."""
    fb = np.array(pl_fb, dtype=np.uint32, copy=True)
    W, H, Wd, Hd, dx, dy = (int(regs[k]) for k in range(6))
    px, py, div = (int(v) for v in pip)
    src = pl_fb[dy:dy + Hd, dx:dx + Wd]
    rows = np.minimum(np.arange(0, Hd, div) + (1 if mutant == "pip_row" else 0), Hd - 1)
    small = src[rows][:, ::div]
    ph, pw = small.shape
    if px + pw <= W and py + ph <= H:
        fb[py:py + ph, px:px + pw] = small
    m = overlay_mask(regs, desc, 1 if mutant == "mask_shift" else 0)[dy:dy + Hd, dx:dx + Wd]
    col = rgb_to_xrgb(rgb, Wd, Hd)
    region = fb[dy:dy + Hd, dx:dx + Wd]
    region[~m] = col[~m]
    return fb


MINI_BOX_W, MINI_BOX_H = 130, 141


def mini_fits(regs, scale):
    W, H = int(regs[0]), int(regs[1])
    x0, y0 = int(regs[22]) - 1, int(regs[23]) - 12
    s = int(scale)
    return s >= 2 and x0 >= 0 and y0 >= 0 and x0 + MINI_BOX_W * s <= W and y0 + MINI_BOX_H * s <= H


def scale_mini(fb, regs, scale, mutant=None):
    """Copy of ``fb`` with the mini-map box enlarged ``scale`` times (nearest neighbour) from its own
    top-left corner. ``mutant`` "mini_row" (positive control): each pair's second target row takes the
    next source row."""
    out = np.array(fb, dtype=np.uint32, copy=True)
    s = int(scale)
    x0, y0 = int(regs[22]) - 1, int(regs[23]) - 12
    src = np.array(fb[y0:y0 + MINI_BOX_H, x0:x0 + MINI_BOX_W], dtype=np.uint32, copy=True)
    rows = np.arange(MINI_BOX_H * s) // s
    if mutant == "mini_row":
        rows[s - 1::s] = np.minimum(rows[s - 1::s] + 1, MINI_BOX_H - 1)
    cols = np.arange(MINI_BOX_W * s) // s
    out[y0:y0 + MINI_BOX_H * s, x0:x0 + MINI_BOX_W * s] = src[rows][:, cols]
    return out


def pip_size(wd, hd, div):
    return len(range(0, int(wd), int(div))), len(range(0, int(hd), int(div)))
