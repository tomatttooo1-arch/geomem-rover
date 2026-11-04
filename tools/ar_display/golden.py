"""Independent numpy golden for the AR overlay core (D, 2026-10-11).

Written from AR_DISPLAY_SPEC_20261011.md + the pinned design in
AR_DISPLAY_HANDOVER_20261011.md, NOT by translating hw/m7/ar_overlay.cpp:
whole-array painting from lowest to highest priority, Python string
formatting for the panel, ASCII-art glyphs, the LUT recomputed from the legacy
piecewise formula. Compatible with Python 3.8 / numpy 1.24 and 3.13 / 2.2.
"""
import numpy as np

REG_NAMES = [
    "width", "height", "depth_width", "depth_height", "depth_dx", "depth_dy",
    "frame_valid", "pose_valid", "age_ms", "age_valid", "max_age_ms",
    "replay", "object_evidence", "frame_id", "permit_present", "permit_i",
    "grid_valid", "slice_y", "slice_lo_mm", "slice_hi_mm", "cam_cell_x",
    "cam_cell_z", "mini_x", "mini_y",
]
MAGIC = 0x4D374152
WHITE, BLACK, BG = 0xFFFFFF, 0x000000, 0x101010

# 3 wide x 5 tall. '#' = ink.
_ART = {
    "0": "### #.# #.# #.# ###", "1": ".#. ##. .#. .#. ###",
    "2": "### ..# ### #.. ###", "3": "### ..# ### ..# ###",
    "4": "#.# #.# ### ..# ..#", "5": "### #.. ### ..# ###",
    "6": "### #.. ### #.# ###", "7": "### ..# ..# ..# ..#",
    "8": "### #.# ### #.# ###", "9": "### #.# ### ..# ###",
    "A": ".#. #.# ### #.# #.#", "B": "##. #.# ##. #.# ##.",
    "C": ".## #.. #.. #.. .##", "D": "##. #.# #.# #.# ##.",
    "E": "### #.. ##. #.. ###", "F": "### #.. ##. #.. #..",
    "G": ".## #.. #.# #.# .##", "H": "#.# #.# ### #.# #.#",
    "I": "### .#. .#. .#. ###", "J": "..# ..# ..# #.# ###",
    "K": "#.# #.# ##. #.# #.#", "L": "#.. #.. #.. #.. ###",
    "M": "#.# ### ### #.# #.#", "N": "#.# ### ### ### #.#",
    "O": "### #.# #.# #.# ###", "P": "##. #.# ##. #.. #..",
    "Q": ".#. #.# #.# ##. .##", "R": "##. #.# ##. #.# #.#",
    "S": ".## #.. ### ..# ##.", "T": "### .#. .#. .#. .#.",
    "U": "#.# #.# #.# #.# ###", "V": "#.# #.# #.# #.# .#.",
    "W": "#.# #.# ### ### #.#", "X": "#.# #.# .#. #.# #.#",
    "Y": "#.# #.# .#. .#. .#.", "Z": "### ..# .#. #.. ###",
    ":": "... .#. ... .#. ...", ".": "... ... ... ... .#.",
    "?": "### ..# .#. ... .#.", ">": "#.. .#. ..# .#. #..",
    "/": "..# ..# .#. #.. #..", "-": "... ... ### ... ...",
    "+": "... .#. ### .#. ...", "=": "... ### ... ### ...",
}
FONT = {c: np.array([[ch == "#" for ch in row] for row in art.split()], bool)
        for c, art in _ART.items()}
BLANK = np.zeros((5, 3), bool)


def _legacy_colour(z):
    if z < 100 or z > 9999:
        return 0
    if z <= 2000:
        r, g, b = 255, (z - 100) * 255 // 1900, 0
    elif z <= 5000:
        r, g, b = (5000 - z) * 255 // 3000, 255, (z - 2000) * 255 // 3000
    else:
        r, g, b = 0, (9999 - z) * 255 // 4999, 255
    return (r << 16) | (g << 8) | b


LUT = np.array([0 if i < 3 else _legacy_colour(min(32 * i + 16, 9999))
                for i in range(313)], np.uint32)


def glyph(ch):
    return FONT.get(ch, BLANK)


def _text(canvas, s, x0, y0, scale, pitch, ox, oy, ink, clip=None):
    """Glyph k at cell [x0+pitch*k, ...); ink block at (+ox, +oy), scale^2."""
    H, W = canvas.shape
    for k, ch in enumerate(s):
        g = np.kron(glyph(ch), np.ones((scale, scale), bool))
        gx, gy = x0 + pitch * k + ox, y0 + oy
        for (r, c) in zip(*np.nonzero(g)):
            yy, xx = gy + r, gx + c
            if 0 <= yy < H and 0 <= xx < W and (clip is None or clip(xx, yy)):
                canvas[yy, xx] = ink


def panel_strings(mode, R, nearest):
    fresh = mode == 0
    if fresh:
        cm = nearest // 10
        z = "Z:%02d.%02dM" % (cm // 100, cm % 100)
    else:
        z = "Z:??.??M"
    if fresh:
        m = "REPLAY" if R["replay"] else "LIVE"
    else:
        m = {1: "BADFRAME", 2: "STALE", 3: "POSEBAD", 4: "NODATA", 5: "NOAGE"}[mode]
    if not R["age_valid"]:
        a = "AGE N/A"
    elif R["age_ms"] > 999:
        a = "AGE>999MS"
    else:
        a = "AGE:%03dMS" % R["age_ms"]
    if not R["permit_present"]:
        p = "DRV N/A"
    else:
        p = "DRIVE OK" if R["permit_i"] else "STOP"
    ev = R["object_evidence"] if fresh else 0
    o = ["OBJ ?", "OBJ HIT", "OBJ FREE?", "OBJ OCCL"][ev]
    f = "F:%d" % R["frame_id"]
    ps = "POSE OK" if mode == 0 else ("POSE BAD" if mode == 3 else "POSE ?")
    return [z, m, a, p], ["RISK N/A", o, f, ps]


def slice_label(lo, hi):
    def s4(v):
        sign = "-" if v < 0 else "+"
        return sign + ("????" if abs(v) > 9999 else "%04d" % abs(v))
    return "Y" + s4(lo) + ".." + s4(hi) + "MM"


def _s32(v):
    v &= 0xFFFFFFFF
    return v - (1 << 32) if v >= (1 << 31) else v


def render(depth, grid, desc, regs):
    """depth: flat u16 (>= Wd*Hd), grid: 32768 u8, desc: 196 u32, regs: 24.
    Returns (status, fb HxW uint32 or None, info dict)."""
    R = dict(zip(REG_NAMES, [int(v) for v in regs]))
    R["slice_lo_mm"] = _s32(R["slice_lo_mm"])
    R["slice_hi_mm"] = _s32(R["slice_hi_mm"])
    W, H, Wd, Hd = R["width"], R["height"], R["depth_width"], R["depth_height"]
    dx, dy = R["depth_dx"], R["depth_dy"]
    dims_ok = (0 < W <= 1280 and 0 < H <= 800 and W % 4 == 0 and
               0 < Wd <= 1280 and 0 < Hd <= 800 and Wd % 8 == 0 and
               dx + Wd <= W and dy + Hd <= H)
    if not dims_ok:
        return 1, None, {}
    ctrl_bad = (any(R[k] > 1 for k in ("frame_valid", "pose_valid", "age_valid",
                                         "replay", "permit_present", "grid_valid"))
                or R["object_evidence"] > 3 or R["slice_y"] > 31)
    desc = [int(v) for v in desc]
    n = desc[1]
    boxes = []
    desc_bad = desc[0] != MAGIC or n > 32
    if not desc_bad:
        for i in range(n):
            x0, y0, x1, y1, v, lab = desc[2 + 6 * i: 8 + 6 * i]
            if x0 > x1 or y0 > y1 or x1 >= Wd or y1 >= Hd or v not in (0, 2, 3):
                desc_bad = True
            boxes.append((x0, y0, x1, y1, v, lab))
    status = 2 if ctrl_bad else (3 if desc_bad else 0)

    z = np.asarray(depth, np.uint16)[:Wd * Hd].reshape(Hd, Wd).astype(np.int64)
    valid = (z >= 100) & (z <= 9999)
    nearest = int(z[valid].min()) if valid.any() else None
    if status != 0 or R["frame_valid"] == 0:
        mode = 1
    elif not R["replay"] and not R["age_valid"]:
        mode = 5
    elif not R["replay"] and R["age_ms"] > R["max_age_ms"]:
        mode = 2
    elif not R["pose_valid"]:
        mode = 3
    elif nearest is None:
        mode = 4
    else:
        mode = 0
    fresh = mode == 0

    fb = np.full((H, W), BG, np.uint32)
    # depth rectangle
    if fresh:
        fb[dy:dy + Hd, dx:dx + Wd] = np.where(valid, LUT[np.clip(z, 0, 9999) // 32], 0)
    else:
        yy, xx = np.mgrid[dy:dy + Hd, dx:dx + Wd]
        fb[dy:dy + Hd, dx:dx + Wd] = np.where(((xx // 8 + yy // 8) % 2) == 1,
                                              0xA000A0, 0x180018)
    # boxes: lines, then labels; lowest index painted last (on top)
    colour_of = {0: 0xFFFFFF, 2: 0xFF00FF, 3: 0x808080}
    verdict_ch = {0: "?", 2: "F", 3: "O"}
    if fresh:
        for (x0, y0, x1, y1, v, lab) in reversed(boxes):
            X0, X1, Y0, Y1 = x0 + dx, x1 + dx, y0 + dy, y1 + dy
            c = colour_of[v]
            fb[Y0:Y1 + 1, X0:min(X0 + 2, X1 + 1)] = c
            fb[Y0:Y1 + 1, max(X1 - 1, X0):X1 + 1] = c
            fb[Y0:min(Y0 + 2, Y1 + 1), X0:X1 + 1] = c
            fb[max(Y1 - 1, Y0):Y1 + 1, X0:X1 + 1] = c
        for (x0, y0, x1, y1, v, lab) in reversed(boxes):
            X0, Y0 = x0 + dx, y0 + dy
            LY = Y0 - 12 if Y0 >= 52 else Y0 + 2
            c = colour_of[v]
            fb[LY:LY + 12, X0:X0 + 48] = c
            text = "".join(chr((lab >> (8 * k)) & 0xFF) for k in range(4)) + " " + verdict_ch[v]
            _text(fb, text, X0, LY, 2, 8, 1, 1, BLACK)
    # minimap: label, border, inner (inner wins)
    mx, my = R["mini_x"], R["mini_y"]
    lab = slice_label(R["slice_lo_mm"], R["slice_hi_mm"])
    yy, xx = np.mgrid[0:H, 0:W]
    rx, ry = xx - mx, yy - my
    in_label = (rx >= 0) & (rx < 128) & (ry >= -12) & (ry < 0)
    fb[in_label] = 0x202020
    _text(fb, lab, mx, my - 12, 2, 8, 1, 1, WHITE,
          clip=lambda x, y: 0 <= x - mx < 128)
    ring = (rx >= -1) & (rx <= 128) & (ry >= -1) & (ry <= 128)
    fb[ring] = WHITE
    inner = (rx >= 0) & (rx < 128) & (ry >= 0) & (ry < 128)
    if not fresh or R["grid_valid"] != 1:
        stripe = np.where(((xx + yy) % 8) < 2, 0xA000A0, 0x180018).astype(np.uint32)
        fb[inner] = stripe[inner]
    else:
        g = np.asarray(grid, np.uint8).reshape(32, 32, 32)  # [z][y][x]
        sl = g[:, R["slice_y"], :]                           # [z][x]
        pal = np.array([0x282828, 0x2E6B3A, 0xFF3030] + [0xFF00FF] * 253, np.uint32)
        top = np.flipud(sl)                                  # row 0 = far (z=31)
        img = pal[np.kron(top, np.ones((4, 4), np.uint8))]   # 128x128
        cx, cz = R["cam_cell_x"], R["cam_cell_z"]
        if cx < 32 and cz < 32:
            r0 = (31 - cz) * 4
            img[r0:r0 + 4, cx * 4:cx * 4 + 4] = WHITE
        ys, xs = np.nonzero(inner)
        fb[ys, xs] = img[ys - my, xs - mx]
    # panel
    row0, row1 = panel_strings(mode, R, nearest)
    ev = R["object_evidence"] if fresh else 0
    bg0 = [0x203040 if fresh else 0x604010, 0x104020 if fresh else 0x701010,
           0x303030 if not R["age_valid"] else (0x701010 if R["age_ms"] > R["max_age_ms"] else 0x104020),
           0x303030 if not R["permit_present"] else (0x104020 if R["permit_i"] else 0x701010)]
    bg1 = [0x502050, [0x303030, 0x104020, 0x605010, 0x203060][ev], 0x203040,
           0x104020 if mode == 0 else (0x701010 if mode == 3 else 0x303030)]
    ph = min(H, 40)
    seg_x = [(0, 156), (156, 312), (312, 468), (468, 636)]  # 53 columns x 12
    for s, (a, b) in enumerate(seg_x):
        fb[0:min(ph, 20), a:b] = bg0[s]
        fb[20:ph, a:b] = bg1[s]
    fb[0:ph, 636:] = BG
    for s in range(4):
        _text(fb, row0[s], 156 * s, 0, 3, 12, 2, 4, WHITE, clip=lambda x, y: y < 39 and x < 636)
        _text(fb, row1[s], 156 * s, 20, 3, 12, 2, 2, WHITE, clip=lambda x, y: y < 39 and x < 636)
    for s in range(4):
        if 156 * s < W:
            fb[0:ph, 156 * s] = WHITE
    if H > 39:
        fb[39, :] = WHITE
    info = {"mode": mode, "nearest": nearest, "panel": row0 + row1,
            "slice_label": lab, "boxes": len(boxes) if fresh else 0}
    return status, fb, info
