"""Car AR display for kv_server (``--ar drm``), 2026-10-11.

Every geometry frame answered by kv_server is handed (after the answer is on the wire)
to a resident C++ presenter, tools/kv_link/ar_helper/m7ar_present.cpp, which drives the
AR overlay core of m7_pl_merge (0xA008_0000, SPI 96) into a dma-heap framebuffer and
page-flips it on the DP output (DRM/KMS). This module builds the core's inputs from the
request/answer and talks to that process. See docs/design/CAR_AR_RESULTS_20261011.md.

Where every AR input comes from (AR_DISPLAY_SPEC section 4; no source -> N/A, never made up):

  depth (u16 mm)     the request's depth codes -> float32 m (codes_to_m, the same values the
                     geometry core used) -> host.m7_display_native.quantize_optical_z
  grid / mini map    the geometry core's voxel flags of this frame (0 unknown / 1 free / 2 hit,
                     host/m7_pipeline.py), geometry status 0 only (else grid_valid=0: hatched
                     mini map), for two grid shapes:
                     32x32x32 (rec30 replay, camera frame, y down): as is; the core's slice
                       is the camera-y layer, rows = z (far at the top), columns = x;
                     32x32xnz, nz < 32 (car producer: odom block, z up, is_odom_block -- by
                       size, an assumption, the wire names no frame): permuted into the 32^3
                       buffer as [y][z][x] (layers >= nz stay 0), so the core's slice is the
                       horizontal z layer: a plan view, odom x to the right, odom y up;
                     any other shape: grid_valid=0
  slice_y, lo/hi     the grid layer through the camera centre (world_camera translation,
                     request origin/voxel), clamped into the grid: 32^3 -> along y, printed
                     as camera-frame grid y in mm; odom block -> along z (the producer's
                     default block gives its height_cell 7: block bottom = (camera cell - 7)
                     x voxel), printed as the layer's odom z bounds in mm. The core always
                     prefixes the label with 'Y' (core text, not changed here)
  cam_cell_x/z       camera centre cell (32^3: x, z; odom block: x, y); 32 (= not drawn) when
                     outside the grid
  boxes (<= 32)      ROI request rows (closed xyxy, depth-image pixels) of enabled rows whose
                     ROI answer has status 0, in request order (lower index on top);
                     verdict = the answer row's verdict code {0,2,3}. Label: the row's class
                     name from the REQ LABEL block (feature label_ext, same row index as the
                     ROI request row) shown as its first 4 characters upper-cased (the core
                     draws exactly 4 glyphs + verdict in a 48 px bar; glyphs A-Z 0-9 : . ? > /
                     - + =; anything else -> '-'; shorter -> space-padded); no block or an
                     empty name -> "R" + 3 digits of row_id. No ROI answer -> no boxes.
  frame_valid        1 iff geometry status 0
  pose_valid         1 iff geometry status 0 and all 12 world_camera values finite (no pose
                     quality bit exists on the wire)
  frame_id           low 32 bits of the request frame id
  age                Nano elapsed capture->send (REQ age extension, Nano clock) + KV time
                     since the request's first chunk (KV clock), written by the helper right
                     before ap_start (ar_age_ms below); no extension -> age_valid=0. Live or
                     replay alike (10-12: the replay mark no longer drops the age).
  replay             1 when kv_server runs --ar-source replay OR the request carries the
                     replay mark (ext_flags bit3, feature replay_ext): where the data comes
                     from, nothing else. The helper writes replay=0 to the core when the
                     frame has no age or is older than max_age_ms, so the core's own age
                     rules give NOAGE / STALE exactly as for a live frame (the core skips
                     both age checks when replay=1); a fresh replay frame shows REPLAY and
                     its age. Its events carry both values (replay = data, replay_core).
  permit             the core samples the permit/permit_o wire itself (permit_present=1 on
                     m7_pl_merge); e-stop / timeout revoke -> STOP. Nothing written here.
  object_evidence    0 (no mask path runs in kv_server) -> OBJ ?;  RISK is always N/A.

Python 3.8 compatible.
"""

from collections import Counter, deque
import fcntl
import json
import math
import os
import struct
import subprocess
import threading
import time

import numpy as np

REG_NAMES = (
    "width", "height", "depth_width", "depth_height", "depth_dx", "depth_dy",
    "frame_valid", "pose_valid", "age_ms", "age_valid", "max_age_ms",
    "replay", "object_evidence", "frame_id", "permit_present", "permit_i",
    "grid_valid", "slice_y", "slice_lo_mm", "slice_hi_mm", "cam_cell_x",
    "cam_cell_z", "mini_x", "mini_y",
)
R = {n: i for i, n in enumerate(REG_NAMES)}
AR_MAGIC = 0x4D374152
DESC_U32 = 196
GRID_N = 32
GRID_BYTES = GRID_N ** 3
MAX_BOXES = 32
PANEL_ROWS = 40
MINI = 128
# Mini map 2x on the screen (owner 10-29): the PL still draws the 128 version (4 px per cell); the
# helper (m7ar_present --mini-scale 2) enlarges its box -- label 12 rows + 1 px ring + 128 inner, 130 x 141
# from (mini_x - 1, mini_y - 12) -- nearest neighbour from the same top-left corner. Colour main view only
# (ar_colour_layout mini_scale=2 places it; the message flag MSG_COLOUR_MINI2X tells the helper).
MINI_BOX_W, MINI_BOX_H = MINI + 2, MINI + 13
MINI_SCALES = (1, 2)
MSG_COLOUR_MINI2X = 2                       # MSG_COLOUR flags b1
MAX_W, MAX_H = 1280, 800
SOURCES = ("live", "replay")

MSG_MAGIC = b"M7AF"
MSG_HEAD = struct.Struct("<4sIII")          # magic, version, type, payload length
MSG_FIXED = struct.Struct("<QQqI")          # frame_id, first_mono_ns, age_basis_ns, depth_px
MSG_VERSION, MSG_FRAME = 1, 1
# Version 2 (AR colour main view, 2026-10-16): the version-1 payload, then MSG_COLOUR (flags b0 =
# colour mode on, pip_x, pip_y, pip_div, colour width, colour height) and width*height*3 RGB888 bytes
# (0 x 0 = colour mode on but no paired colour: the helper shows the PL picture as is). Sent ONLY while
# the colour mode is on; with it off the bytes are version 1, exactly as before.
MSG_VERSION_COLOUR = 2
MSG_COLOUR = struct.Struct("<6I")
PIP_DIV = 2
LABEL_SPILL_X, LABEL_SPILL_Y = 48, 12       # a box label bar (48 x 12) may stick out of the depth image
F_SETPIPE_SZ = 1031


class ArUnavailable(RuntimeError):
    """The AR presenter cannot run (helper refused or died at startup); kv_server refuses."""


def ar_age_ms(age_basis_ns, first_mono_ns, now_mono_ns):
    """The one age formula (m7ar_present.cpp age_ms_of mirrors it): two single-clock
    durations, Nano capture->send plus KV first chunk->now, rounded up to ms, saturated."""
    kv = max(0, int(now_mono_ns) - int(first_mono_ns))
    total = int(age_basis_ns) + kv
    if total <= 0:
        return 0
    return min(0xFFFFFFFF, -(-total // 1000000))


LABEL_GLYPHS = frozenset("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789:.?>/-+= ")


def ar_label_text(row_id, name=None):
    """The 4 characters drawn for one box: the class name (bytes/str from the LABEL block)
    upper-cased, cut to 4, unsupported glyphs -> '-', space-padded; else R + row_id."""
    if name:
        t = (name.decode("ascii") if isinstance(name, (bytes, bytearray)) else str(name)).upper()
        t = "".join(c if c in LABEL_GLYPHS else "-" for c in t[:4])
        if t.strip():
            return t.ljust(4)
    return "R%03d" % (int(row_id) % 1000)


def ar_label(row_id, name=None):
    """desc label word: 4 ASCII characters, byte 0 first."""
    b = ar_label_text(row_id, name).encode("ascii")
    return b[0] | b[1] << 8 | b[2] << 16 | b[3] << 24


def ar_layout(canvas_w, canvas_h, depth_w, depth_h):
    """Depth at (0, 40) under the panel; the mini map right of the depth image when there
    is room (canvas 1024 wide: x = 640 + centred), else in the bottom-right corner over
    the depth image. None when the depth image cannot be shown on this canvas."""
    W, H, Wd, Hd = int(canvas_w), int(canvas_h), int(depth_w), int(depth_h)
    if not (0 < W <= MAX_W and 0 < H <= MAX_H and W % 4 == 0 and 0 < Wd and Wd % 8 == 0
            and 0 < Hd and Wd <= W and PANEL_ROWS + Hd <= H):
        return None
    if W - Wd >= MINI + 8:
        mini_x, mini_y = Wd + (W - Wd - MINI) // 2, PANEL_ROWS + 12 + 8
    else:
        mini_x, mini_y = W - MINI - 4, H - MINI - 4
    return {"width": W, "height": H, "depth_dx": 0, "depth_dy": PANEL_ROWS,
            "mini_x": mini_x, "mini_y": mini_y}


TUNE_LAYOUT_KEYS = ("depth_dx", "depth_dy", "mini_x", "mini_y")
# colour main view (2026-10-16): colour_main 0/1 switches it live; its layout has its own keys so the
# depth-main layout the owner tuned (depth_dx/dy, mini_x/y) is kept for colour_main 0
TUNE_COLOUR_LAYOUT = {"colour_depth_dx": "depth_dx", "colour_depth_dy": "depth_dy", "pip_x": "pip_x",
                      "pip_y": "pip_y", "colour_mini_x": "mini_x", "colour_mini_y": "mini_y"}
TUNE_KEYS = TUNE_LAYOUT_KEYS + ("slice_y", "max_age_ms", "box_max", "box_nms_pct", "depth_stretch_pct",
                                "colour_main", "colour_wait_ms", "colour_hold_ms",
                                "nodepth_after_ms", "colour_refresh_ms") + tuple(TUNE_COLOUR_LAYOUT)
# owner 10-16: a frame whose colour is not paired shows the LAST paired colour (steady picture) as long
# as that colour is at most this old; only after that the depth main view comes back
COLOUR_HOLD_MS = 1000
# No-depth self pull (docs/design/AR_DEPTH_INVALID_STALL_20261017.md): a depth frame with no valid pixel
# never reaches the KV (the Nano refuses it before the request), so without this the screen froze and
# the helper flipped STALE after max_age+1 ms. Defaults: first pull this long after the last real frame,
# then every period; WANT tolerance / lead (one camera period) for "the newest colour frame"; give up
# (old behaviour, STALE) after max_s of pulls in a row.
NODEPTH_AFTER_MS = 150
NODEPTH_PERIOD_MS = 100
NODEPTH_TOL_MS = 67
NODEPTH_LEAD_MS = 33
NODEPTH_MAX_S = 60.0
NODEPTH_KEY_MIN = 10 ** 12          # a sensor key below this is not a ns time (nano_replay: frame index + 1)
# Colour refresh (AR_LATENCY_PLAN_20261027 plan C, owner 10-27): between two geometry frames the sender
# re-sends the LAST geometry frame's AR inputs (boxes, depth inset, panel, its own growing age) with the
# NEWEST colour frame, every colour_refresh_ms after the last picture; the background then follows the
# camera at 20-30 Hz instead of the ~10 Hz geometry rate. A real frame always wins. Runs only while the
# last real frame is younger than the no-depth hand-over (nodepth_after_ms) and its age stays below
# max_age_ms (so a refresh never turns a picture STALE). 0 = off (the code before; CLI default).
COLOUR_REFRESH_LEAD_MS = 20         # newest colour: WANT the key acquired this long before now
COLOUR_REFRESH_POLL_S = 0.004       # while a refresh waits for its colour, a real frame arriving ends the wait
COLOUR_REFRESH_AGE_MARGIN_MS = 50


def pip_size(depth_w, depth_h, div=PIP_DIV):
    """PiP of the PL depth picture decimated by div: rows/columns 0, div, 2 div, ..."""
    return -(-int(depth_w) // int(div)), -(-int(depth_h) // int(div))


def ar_colour_layout(canvas_w, canvas_h, depth_w, depth_h, div=PIP_DIV, mini_scale=1):
    """Colour main view: the depth rectangle (= colour picture, D2C aligned) at the left edge,
    vertically centred under the panel; the PiP (PL depth picture / div) top right, 48 px clear of
    the depth rectangle (box label bars can stick out by 47 px); the mini map centred under the PiP.
    1024x600 with 640x480: depth (0, 80), PiP 320x240 at (688, 48), mini map (784, 309).
    mini_scale 2 (owner 10-29): the 2x mini-map box (260 x 282) centred under the PiP, 4 px below it
    (1024x600, 640x480: box at (718, 292), mini_x/y (719, 304), bottom 574); the layout carries
    "mini_scale": 2. If the 2x box does not fit, the 1x layout (no "mini_scale" key) is returned.
    None when it does not fit."""
    W, H, Wd, Hd = int(canvas_w), int(canvas_h), int(depth_w), int(depth_h)
    if ar_layout(W, H, Wd, Hd) is None:
        return None
    pw, ph = pip_size(Wd, Hd, div)
    lay = {"width": W, "height": H, "depth_dx": 0, "depth_dy": PANEL_ROWS + (H - PANEL_ROWS - Hd) // 2,
           "pip_x": Wd + LABEL_SPILL_X, "pip_y": PANEL_ROWS + 8, "pip_div": int(div)}
    lay["mini_x"] = lay["pip_x"] + max(0, (pw - MINI) // 2)
    lay["mini_y"] = lay["pip_y"] + ph + 8 + 12 + 1
    s = int(mini_scale)
    if s not in MINI_SCALES:
        raise ValueError("mini_scale must be 1 or 2")
    if s > 1:
        bw, bh = MINI_BOX_W * s, MINI_BOX_H * s
        big = dict(lay, mini_scale=s)
        big["mini_x"] = lay["pip_x"] + max(0, (pw - bw) // 2) + 1
        big["mini_y"] = lay["pip_y"] + ph + 4 + 12
        if colour_layout_problem(big, Wd, Hd) is None:
            return big
    return lay if colour_layout_problem(lay, Wd, Hd) is None else None


def colour_layout_problem(lay, depth_w, depth_h):
    """None when the three areas -- depth/colour picture (with the label-bar spill), PiP, mini map
    (with its 1 px ring and 12-row label; times lay["mini_scale"] when present) -- lie inside the canvas
    below the panel and pairwise apart; else a short reason."""
    W, H = int(lay["width"]), int(lay["height"])
    Wd, Hd = int(depth_w), int(depth_h)
    pw, ph = pip_size(Wd, Hd, lay["pip_div"])
    dx, dy = int(lay["depth_dx"]), int(lay["depth_dy"])
    rects = {"depth": (dx, max(PANEL_ROWS, dy - LABEL_SPILL_Y), dx + Wd + LABEL_SPILL_X - 1, dy + Hd),
             "pip": (int(lay["pip_x"]), int(lay["pip_y"]), int(lay["pip_x"]) + pw, int(lay["pip_y"]) + ph),
             "mini": (int(lay["mini_x"]) - 1, int(lay["mini_y"]) - 12,
                      int(lay["mini_x"]) - 1 + MINI_BOX_W * int(lay.get("mini_scale", 1)),
                      int(lay["mini_y"]) - 12 + MINI_BOX_H * int(lay.get("mini_scale", 1)))}
    if dx < 0 or dy < PANEL_ROWS or dx + Wd > W or dy + Hd > H:
        return "depth_outside"
    for k in ("pip", "mini"):
        x0, y0, x1, y1 = rects[k]
        if x0 < 0 or y0 < PANEL_ROWS or x1 > W or y1 > H:
            return "%s_outside" % k
    names = list(rects)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = rects[names[i]], rects[names[j]]
            if a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]:
                return "%s_overlaps_%s" % (names[i], names[j])
    return None


def apply_slice_override(inp, snap, slice_y):
    """Screen bring-up: force the mini-map layer; the printed mm range follows the same axis
    rule as fill_ar_inputs (odom block: z, else camera y)."""
    regs = inp["regs"]
    dims = (int(snap["nx"]), int(snap["ny"]), int(snap["nz"]))
    odom = is_odom_block(dims)
    n = dims[2] if odom else GRID_N
    s = min(max(int(slice_y), 0), n - 1)
    ax = 2 if odom else 1
    origin, voxel = [float(v) for v in snap["origin"]], float(snap["voxel"])
    regs[R["slice_y"]] = s
    regs[R["slice_lo_mm"]] = int(round((origin[ax] + voxel * s) * 1000.0)) & 0xFFFFFFFF
    regs[R["slice_hi_mm"]] = int(round((origin[ax] + voxel * (s + 1)) * 1000.0)) & 0xFFFFFFFF


def _cell(v, o, voxel):
    return int(math.floor((float(v) - float(o)) / float(voxel)))


def is_odom_block(dims):
    """32 x 32 x nz with nz < 32: taken as a z-up block in the local odometry frame (the car
    producer's FollowCameraGrid block, host/geo_producer.py DEFAULT_SHAPE (32, 32, 10)).
    ASSUMPTION by size only: the wire carries no frame name (IDENTITY_FIELDS has none), and
    the only 32 x 32 x nz senders today are that producer; a 32^3 grid keeps the camera-frame
    (y down) reading of the rec30 replay."""
    nx, ny, nz = (int(v) for v in dims)
    return nx == GRID_N and ny == GRID_N and 1 <= nz < GRID_N


def stretch_depth_mm(depth_u16, k):
    """Display-only colour stretch (2026-10-15 proposal): f(z) = zmin + (z - zmin) * k on valid
    pixels (100..9999 mm), clipped to 9999; invalid pixels stay as they are. zmin = this frame's
    nearest valid depth, so f(zmin) = zmin and the core's own nearest-Z panel reading is unchanged."""
    d = np.asarray(depth_u16, dtype=np.int64)
    valid = (d >= 100) & (d <= 9999)
    if not valid.any() or k <= 0:
        return np.asarray(depth_u16, dtype=np.uint16)
    zmin = int(d[valid].min())
    out = d.copy()
    out[valid] = np.minimum(zmin + np.floor((d[valid] - zmin) * float(k)).astype(np.int64), 9999)
    return out.astype(np.uint16)


def _box_iou(a, b):
    """IoU of two closed-interval pixel boxes (x0, y0, x1, y1)."""
    ix = min(a[2], b[2]) - max(a[0], b[0]) + 1
    iy = min(a[3], b[3]) - max(a[1], b[1]) + 1
    if ix <= 0 or iy <= 0:
        return 0.0
    inter = ix * iy
    area = lambda r: (r[2] - r[0] + 1) * (r[3] - r[1] + 1)  # noqa: E731
    return inter / float(area(a) + area(b) - inter)


def select_display_boxes(cands, max_show=MAX_BOXES, nms_iou=None):
    """Display-only box choice (2026-10-15, owner: a clean picture; the ROI core still checks every
    row, so region verdicts / memory are untouched). ``cands``: (x0, y0, x1, y1, verdict, label_word,
    class_key) in request order = the O11 priority (target class, newest last_seen, nearest).
    Same-class greedy NMS in that order (a box is hidden when its IoU with an already kept box of the
    same class_key is > nms_iou; None = off), then the first ``max_show``. Returns (kept, hidden)."""
    kept, hidden = [], 0
    for c in cands:
        if nms_iou is not None and any(k[6] == c[6] and _box_iou(k, c) > nms_iou for k in kept):
            hidden += 1
            continue
        if len(kept) >= max_show:
            hidden += 1
            continue
        kept.append(c)
    return kept, hidden


def upsample_snapshot(snap, target):
    """Display only (2026-10-16, producer --kv-depth-stride): a frame whose depth is the camera picture
    decimated by an integer s (target = (W, H) of the camera / colour picture, W = s * width and
    H = s * height) is shown as if it had the full size: every code repeated s x s (nearest), every ROI
    rectangle [x0, x1] -> [s x0, s x1 + s - 1] (same for y). The geometry answer is untouched (this runs
    in the AR sender, after the answer left). So the colour pairs again (its size = the depth rectangle),
    the owner-tuned layout stays, the PiP (depth / pip_div 2 at s = 2) is exactly the decimated frame and
    the nearest-Z reading is the decimated frame's minimum (= the full frame's, block minimum).
    Returns (snap, s): s = 1 and the snapshot itself when target is None, already the target size, or
    not an integer multiple (counted by the caller)."""
    if target is None:
        return snap, 1
    W, H = int(snap["width"]), int(snap["height"])
    Wt, Ht = int(target[0]), int(target[1])
    if (W, H) == (Wt, Ht) or W <= 0 or H <= 0 or Wt % W or Ht % H or Wt // W != Ht // H:
        return snap, 1
    s = Wt // W
    codes = np.asarray(snap["codes"]).reshape(-1)[:W * H].reshape(H, W)
    out = dict(snap, codes=np.ascontiguousarray(np.repeat(np.repeat(codes, s, axis=0), s, axis=1).reshape(-1)),
               width=Wt, height=Ht)
    roi = snap.get("roi")
    if roi is not None and roi.get("rows") is not None:
        rows = []
        for r in roi["rows"]:
            r = list(r)
            r[2], r[3] = int(r[2]) * s, int(r[3]) * s
            r[4], r[5] = int(r[4]) * s + s - 1, int(r[5]) * s + s - 1
            rows.append(tuple(r))
        out["roi"] = dict(roi, rows=rows)
    return out, s


def depth_u16_reference(codes, unit_bits, Wd, Hd, depth_stretch):
    """The numpy depth chain (reference of host/kv_ar_native.py): request codes -> float32 m
    (codes_to_m) -> u16 mm (quantize_optical_z) -> display stretch (only when k != 1)."""
    from host.kv_link import bits_f32, codes_to_m
    from host.m7_display_native import quantize_optical_z
    depth_m = codes_to_m(codes, bits_f32(unit_bits))[:Wd * Hd].reshape(Hd, Wd)
    depth_u16 = quantize_optical_z(depth_m)[0].reshape(-1)
    if depth_stretch != 1.0:
        depth_u16 = stretch_depth_mm(depth_u16, depth_stretch)
    return depth_u16


def fill_ar_inputs(snap, layout, *, source, permit_present=1, max_age_ms=200,
                   box_max=MAX_BOXES, box_nms=None, depth_stretch=1.0, native=None):
    """``snap``: the hand-off record of one answered frame (KvLinkServer._ar_snapshot).
    Returns {"regs": [24 ints], "depth": u16[Wd*Hd], "grid": u8[32768], "desc": u32[196],
    "age_basis_ns": int (-1 = none), "first_mono_ns": int, "frame_id": int, "boxes": n,
    "skipped_boxes": n}. ``regs[age_ms]`` stays 0 here: the helper writes it at ap_start.
    ``native``: host.kv_ar_native.ArFillNative -- the depth (codes -> m -> mm -> stretch) in one
    C++ pass, bit for bit the numpy chain below (2026-10-16); None = that numpy chain (the reference)."""
    if source not in SOURCES:
        raise ValueError("source must be live|replay")
    Wd, Hd = int(snap["width"]), int(snap["height"])
    depth_u16 = None
    if native is not None:
        depth_u16 = native.depth_u16(snap["codes"], snap["unit_bits"], Wd, Hd, depth_stretch)
    if depth_u16 is None:
        depth_u16 = depth_u16_reference(snap["codes"], snap["unit_bits"], Wd, Hd, depth_stretch)
    status = int(snap["geometry_status"])
    ok = status == 0
    # no-depth self pull (AR_DEPTH_INVALID_STALL_20261017): no geometry ran for this picture -> the
    # mini map is hatched (grid_valid 0) and the camera cell is not drawn; frame_valid / pose_valid
    # stay as for status 0 so the core reaches its own mode 4 NODATA (all-zero depth)
    no_geo = bool(snap.get("no_geometry"))
    wc = np.asarray(snap["world_camera"], dtype=np.float64).reshape(-1)
    pose_ok = ok and wc.size == 12 and bool(np.all(np.isfinite(wc)))
    dims = (int(snap["nx"]), int(snap["ny"]), int(snap["nz"]))
    odom_block = is_odom_block(dims)
    grid_ok = ok and not no_geo and (dims == (GRID_N, GRID_N, GRID_N) or odom_block)
    grid = np.zeros(GRID_BYTES, dtype=np.uint8)
    origin = [float(v) for v in snap["origin"]]
    voxel = float(snap["voxel"])
    cam = (wc[3], wc[7], wc[11]) if wc.size == 12 else (float("nan"),) * 3
    slice_y, lo_mm, hi_mm, cx, cz = 0, 0, 0, GRID_N, GRID_N
    if odom_block:
        # z-up block (car producer, 32x32xnz in odom): the horizontal layer through the camera
        # centre, odom x -> mini map column, odom y -> mini map row (core: far = top), so the
        # mini map is a plan view, x right, y up. Packed into the core's 32^3 buffer with the
        # axes permuted: buffer [z'][y'][x'] = flags [y][z][x], slice_y = the z layer.
        nz = dims[2]
        if grid_ok:
            g3 = grid.reshape(GRID_N, GRID_N, GRID_N)
            g3[:, :nz, :] = np.asarray(snap["flags"], dtype=np.uint8)[:GRID_N * GRID_N * nz].reshape(
                nz, GRID_N, GRID_N).transpose(1, 0, 2)
        if voxel > 0 and all(math.isfinite(v) for v in cam):
            cxi, cyi, czi = (_cell(cam[k], origin[k], voxel) for k in range(3))
            slice_y = min(max(czi, 0), nz - 1)
            lo_mm = int(round((origin[2] + voxel * slice_y) * 1000.0))
            hi_mm = int(round((origin[2] + voxel * (slice_y + 1)) * 1000.0))
            cx = cxi if 0 <= cxi < GRID_N else GRID_N
            cz = cyi if 0 <= cyi < GRID_N else GRID_N
    else:
        if grid_ok:
            grid[:] = np.asarray(snap["flags"], dtype=np.uint8)[:GRID_BYTES]
        if voxel > 0 and all(math.isfinite(v) for v in cam):
            cxi, cyi, czi = (_cell(cam[k], origin[k], voxel) for k in range(3))
            slice_y = min(max(cyi, 0), GRID_N - 1)
            lo_mm = int(round((origin[1] + voxel * slice_y) * 1000.0))
            hi_mm = int(round((origin[1] + voxel * (slice_y + 1)) * 1000.0))
            cx = cxi if 0 <= cxi < GRID_N else GRID_N
            cz = czi if 0 <= czi < GRID_N else GRID_N
    if no_geo:
        cx, cz = GRID_N, GRID_N
    desc = np.zeros(DESC_U32, dtype=np.uint32)
    desc[0] = AR_MAGIC
    n, skipped, named, invalid = 0, 0, 0, 0
    cands = []
    roi = snap.get("roi")
    labels = snap.get("labels")
    if roi is not None and snap.get("roi_status") == 0:
        for i, (req_row, ans) in enumerate(zip(roi["rows"], snap.get("roi_rows") or [])):
            row_id, en, x0, y0, x1, y1 = (int(v) for v in req_row[:6])
            name = labels[i] if labels is not None and i < len(labels) else None
            verdict = int(ans[6])
            if not en:
                continue
            if name is not None and (name[:1] == b"~" or name[:1] == "~"):
                invalid += 1                 # 10-28: position_invalid track (geo_producer.AR_HIDE_MARK), not drawn
                continue
            if (len(cands) >= MAX_BOXES or not (0 <= x0 <= x1 < Wd and 0 <= y0 <= y1 < Hd)
                    or verdict not in (0, 2, 3)):
                skipped += 1
                continue
            word = ar_label(row_id, name)
            cands.append((x0, y0, x1, y1, verdict, word, (name or "").strip().lower() or word))
    kept, hidden = select_display_boxes(cands, max(0, min(int(box_max), MAX_BOXES)), box_nms)
    for (x0, y0, x1, y1, verdict, word, key) in kept:
        desc[2 + 6 * n: 8 + 6 * n] = (x0, y0, x1, y1, verdict, word)
        named += 1 if key != word else 0
        n += 1
    desc[1] = n
    replay = source == "replay" or bool(snap.get("replay"))
    basis = snap.get("nano_elapsed_ns")
    age_valid = 1 if basis is not None else 0
    regs = [0] * len(REG_NAMES)
    regs[R["width"]], regs[R["height"]] = layout["width"], layout["height"]
    regs[R["depth_width"]], regs[R["depth_height"]] = Wd, Hd
    regs[R["depth_dx"]], regs[R["depth_dy"]] = layout["depth_dx"], layout["depth_dy"]
    regs[R["frame_valid"]] = 1 if ok else 0
    regs[R["pose_valid"]] = 1 if pose_ok else 0
    regs[R["age_valid"]] = age_valid
    regs[R["max_age_ms"]] = int(max_age_ms)
    regs[R["replay"]] = 1 if replay else 0
    regs[R["object_evidence"]] = 0
    regs[R["frame_id"]] = int(snap["frame_id"]) & 0xFFFFFFFF
    regs[R["permit_present"]] = int(permit_present)
    regs[R["grid_valid"]] = 1 if grid_ok else 0
    regs[R["slice_y"]] = slice_y
    regs[R["slice_lo_mm"]] = lo_mm & 0xFFFFFFFF
    regs[R["slice_hi_mm"]] = hi_mm & 0xFFFFFFFF
    regs[R["cam_cell_x"]], regs[R["cam_cell_z"]] = cx, cz
    regs[R["mini_x"]], regs[R["mini_y"]] = layout["mini_x"], layout["mini_y"]
    return {"regs": regs, "depth": np.ascontiguousarray(depth_u16, dtype="<u2"),
            "grid": grid, "desc": desc, "frame_id": int(snap["frame_id"]),
            "age_basis_ns": int(basis) if age_valid else -1,
            "first_mono_ns": int(snap["first_mono_ns"]), "boxes": n, "skipped_boxes": skipped,
            "named_boxes": named, "hidden_boxes": hidden, "invalid_boxes": invalid, "replay": replay}


def encode_ar_message(inp):
    """Version 1 unless ``inp["colour"]`` is set (colour mode on): then version 2 with the colour
    block ({"pip": (x, y, div), "rgb": bytes or None, "width", "height"})."""
    depth = np.ascontiguousarray(inp["depth"], dtype="<u2")
    parts = [
        MSG_FIXED.pack(inp["frame_id"] & 0xFFFFFFFFFFFFFFFF, inp["first_mono_ns"],
                       inp["age_basis_ns"], depth.size),
        struct.pack("<24I", *(int(v) & 0xFFFFFFFF for v in inp["regs"])),
        depth.tobytes(), np.ascontiguousarray(inp["grid"], dtype=np.uint8).tobytes(),
        np.ascontiguousarray(inp["desc"], dtype="<u4").tobytes()]
    col = inp.get("colour")
    version = MSG_VERSION
    if col is not None:
        version = MSG_VERSION_COLOUR
        rgb = col.get("rgb")
        cw, ch = (int(col["width"]), int(col["height"])) if rgb is not None else (0, 0)
        if rgb is not None and len(rgb) != cw * ch * 3:
            raise ValueError("colour is %d bytes, %dx%dx3 expected" % (len(rgb), cw, ch))
        px, py, div = (int(v) for v in col["pip"])
        flags = 1 | (MSG_COLOUR_MINI2X if int(col.get("mini_scale", 1)) == 2 else 0)
        parts += [MSG_COLOUR.pack(flags, px, py, div, cw, ch), bytes(rgb) if rgb is not None else b""]
    body = b"".join(parts)
    return MSG_HEAD.pack(MSG_MAGIC, version, MSG_FRAME, len(body)) + body


def helper_path(repo_root, arch=None):
    import platform
    arch = arch or ("aarch64" if platform.machine() == "aarch64" else "host")
    return os.path.join(repo_root, "tools", "kv_link", "ar_helper", "build", arch, "m7ar_present")


class ArPresenter:
    """Owns the helper process; ``submit`` is called by the geometry worker after the answer
    is sent (non-blocking: a one-slot mailbox, newest wins); a sender thread builds the AR
    inputs and writes them to the helper; a reader thread consumes the helper's events."""

    def __init__(self, cmd, *, source, max_age_ms=200, permit_present=1, out_dir=None,
                 ready_timeout_s=15.0, popen=subprocess.Popen, tune_file=None,
                 box_max=MAX_BOXES, box_nms=None, depth_stretch=1.0, colour=None, colour_main=False,
                 colour_wait_ms=40, colour_hold_ms=COLOUR_HOLD_MS, depth_upsample_to=None,
                 fill_native=None, nodepth_after_ms=NODEPTH_AFTER_MS, nodepth_period_ms=NODEPTH_PERIOD_MS,
                 nodepth_tol_ms=NODEPTH_TOL_MS, nodepth_lead_ms=NODEPTH_LEAD_MS,
                 nodepth_max_s=NODEPTH_MAX_S, colour_refresh_ms=0.0, mini_scale=1, permit_source=None):
        from host.kv_link import LatestSlot
        if source not in SOURCES:
            raise ValueError("source must be live|replay")
        self.cmd, self.source = list(cmd), source
        self.max_age_ms, self.permit_present = int(max_age_ms), int(permit_present)
        # AR2NANO2 (2026-11-02): callable -> (permit_present, permit_i) written into every message's
        # registers (the Nano renderer has no permit_o wire; host/nano_ar.py PermitPoller reads the KV
        # TELEM permit block). None (default, the KV260) = the registers exactly as before.
        self.permit_source = permit_source
        self.tune_file, self.tune_mtime, self.tune = tune_file, None, {}
        self.box_max, self.box_nms = int(box_max), box_nms
        self.depth_stretch = float(depth_stretch)
        # 2026-10-16: host.kv_ar_native.ArFillNative (kv_server --ar-fill native) or None (numpy reference)
        self.fill_native = fill_native
        # producer --kv-depth-stride (2026-10-16): shown at this (W, H) when an integer multiple (None = off)
        self.depth_upsample_to = None if depth_upsample_to is None else (int(depth_upsample_to[0]),
                                                                         int(depth_upsample_to[1]))
        # AR colour main view (2026-10-16): ``colour`` = host.kv_colour.ColourSink (None = no bypass,
        # nothing changes); colour_main = initial state (tune colour_main 0/1 switches it live)
        self.colour, self.colour_main = colour, bool(colour_main)
        # owner 10-29: mini map 2x in the colour main view (the helper needs --mini-scale 2 as well)
        if int(mini_scale) not in MINI_SCALES:
            raise ValueError("mini_scale must be 1 or 2")
        self.mini_scale = int(mini_scale)
        self.colour_wait_ms = float(colour_wait_ms)
        self.colour_state = {"state": "off" if not colour_main else "starting", "reason": None,
                             "frame_id": None}
        self.colour_waits = deque(maxlen=256)
        # hold rule (owner 10-16): last paired colour {"rgb", "width", "height", "t", "frame_id",
        # "source_key"}; reused for an unpaired frame while now - t <= colour_hold_ms; 0 = never hold.
        # Freshness is unchanged: the helper composes fresh pictures only (a fail-safe stays one).
        self.colour_hold_ms = float(colour_hold_ms)
        self.colour_last = None
        self.colour_held_age_ms_max = None
        self.colour_held_age_ms_last = None
        self.colour_log = open(os.path.join(out_dir, "ar_colour.jsonl"), "w") if (out_dir and colour is not None) else None
        # no-depth self pull (AR_DEPTH_INVALID_STALL_20261017); after_ms 0 = off (= the code before)
        self.nodepth_after_ms = float(nodepth_after_ms)
        self.nodepth_period_ms = float(nodepth_period_ms)
        self.nodepth_tol_ms, self.nodepth_lead_ms = float(nodepth_tol_ms), float(nodepth_lead_ms)
        self.nodepth_max_s = float(nodepth_max_s)
        if self.nodepth_period_ms <= 0 or self.nodepth_tol_ms <= 0 or self.nodepth_lead_ms < 0:
            raise ValueError("nodepth period / tolerance must be > 0, lead >= 0")
        self.nd_last_real = None        # (snap as submitted, mono s when the sender took it)
        self.nd_offset_ns = None        # sensor key clock - KV CLOCK_MONOTONIC (ns), from the last real frame
        self.nd_last_pull = None        # mono s of the last pull since the last real frame (None: none yet)
        self.nd_first_pull = None       # mono s of the first pull of the current run
        self.nd_seq = 0
        self.nd_last = None             # last pull record (TELEM)
        # colour refresh (plan C 10-27); 0 = off
        self.colour_refresh_ms = float(colour_refresh_ms)
        self.rf_inp = None              # AR inputs of the last real colour-mode picture (reused by refreshes)
        self.rf_last_render = None      # mono s of the last picture sent (real or refresh)
        self.rf_shown_key = None        # sensor key of the colour on screen
        self.rf_seq = 0
        self.rf_last = None             # last refresh record (TELEM)
        self.counts = Counter()
        self.lock = threading.Lock()
        self.slot = LatestSlot()
        self.events_fp = open(os.path.join(out_dir, "ar_events.jsonl"), "w") if out_dir else None
        self.last_frame = None          # last "frame" event
        self.last_failsafe = None
        self.last_render = None         # newest of startup / frame / failsafe (what is on screen)
        self.intervals = deque(maxlen=512)
        self.ar_ns = deque(maxlen=512)
        self.ready = None
        self.exit_event = None
        self.error = None
        self._stop = threading.Event()
        self.proc = popen(self.cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                          stderr=(open(os.path.join(out_dir, "ar_helper.err"), "w")
                                  if out_dir else subprocess.DEVNULL), bufsize=0)
        self.pipe_bytes = None
        # a colour message (~1.6 MB at 640x480) fits a 4 MiB pipe in one write (root: above
        # pipe-max-size); else 1 MiB as before and the write blocks until the helper has read part
        for size in ((4 << 20, 1 << 20) if colour is not None else (1 << 20,)):
            try:
                fcntl.fcntl(self.proc.stdin.fileno(), F_SETPIPE_SZ, size)
                self.pipe_bytes = fcntl.fcntl(self.proc.stdin.fileno(), 1032)   # F_GETPIPE_SZ
                break
            except OSError:
                continue
        self._ready_evt = threading.Event()
        self._reader = threading.Thread(target=self._read_events, name="ar-events", daemon=True)
        self._reader.start()
        if not self._ready_evt.wait(ready_timeout_s) or self.ready is None:
            err = self.error or "no ready line within %.0f s" % ready_timeout_s
            self.close()
            raise ArUnavailable("AR presenter did not start: %s" % err)
        self.layout_cache = {}
        self._sender = threading.Thread(target=self._send_loop, name="ar-send", daemon=True)
        self._sender.start()

    # -- helper events
    def _read_events(self):
        for raw in iter(self.proc.stdout.readline, b""):
            line = raw.decode("utf-8", "replace").strip()
            if not line.startswith("{"):
                self.counts["helper_non_json_lines"] += 1
                continue
            try:
                ev = json.loads(line)
            except ValueError:
                self.counts["helper_bad_json"] += 1
                continue
            if self.events_fp is not None:
                self.events_fp.write(line + "\n")
                self.events_fp.flush()
            kind = ev.get("ev")
            with self.lock:
                self.counts["ev:%s" % kind] += 1
                if kind == "ready":
                    self.ready = ev
                    self._ready_evt.set()
                elif kind == "error":
                    self.error = ev.get("error")
                    self._ready_evt.set()
                elif kind in ("frame", "failsafe", "startup"):
                    if self.last_render is None or ev.get("seq", 0) >= self.last_render.get("seq", 0):
                        self.last_render = ev
                    if kind == "startup":
                        continue
                    if kind == "frame":
                        self.last_frame = ev
                        if ev.get("colour"):
                            self.counts["ev_colour_composited"] += 1
                        self.ar_ns.append(ev.get("ar_ns"))
                    else:
                        self.last_failsafe = ev
                    if ev.get("interval_ms", -1) >= 0:
                        self.intervals.append(ev["interval_ms"])
                elif kind == "exit":
                    self.exit_event = ev
        self._ready_evt.set()
        with self.lock:
            self.counts["helper_stdout_closed"] += 1

    # -- worker side
    def _read_tune(self):
        """Screen bring-up (2026-10-15): the --ar-tune-file JSON, re-read when its mtime changes;
        a bad file keeps the last good one. {} without a file."""
        if not self.tune_file:
            return {}
        try:
            m = os.stat(self.tune_file).st_mtime_ns
            if m != self.tune_mtime:
                with open(self.tune_file) as f:
                    t = json.load(f)
                self.tune = {k: int(v) for k, v in t.items() if k in TUNE_KEYS and v is not None}
                self.tune_mtime = m
                self.counts["tune_reloads"] += 1
        except (OSError, ValueError, TypeError):
            self.counts["tune_errors"] += 1
        return self.tune

    def _colour_on(self, tune=None):
        if self.colour is None:
            return False
        t = self.tune if tune is None else tune
        return bool(t.get("colour_main", 1 if self.colour_main else 0))

    def early(self, req):
        """kv_link server worker, right after a request carrying the sensor key is decoded: ask the
        Nano colour source for this frame's colour (one small datagram) while the core runs."""
        if not self._colour_on():
            return
        if self.colour.want(req["frame_id"], req["capture_ns"], req["sensor_stamp_ns"]):
            self.counts["colour_wanted"] += 1

    def _colour_layout(self, tune, Wd, Hd):
        base = ar_colour_layout(self.ready["width"], self.ready["height"], Wd, Hd, mini_scale=self.mini_scale)
        if base is None:
            return None, "layout_unfit"
        over = {v: int(tune[k]) for k, v in TUNE_COLOUR_LAYOUT.items() if k in tune}
        if over:
            lay = dict(base, **over)
            why = colour_layout_problem(lay, Wd, Hd)
            if why is None:
                return lay, None
            self.counts["colour_tune_layout_rejected:%s" % why] += 1
        return base, None

    def _colour_for(self, snap, tune, Wd, Hd):
        """(layout, colour dict) for a colour-mode frame; colour["rgb"] None = depth fallback."""
        lay, why = self._colour_layout(tune, Wd, Hd)
        if lay is None:
            return None, None, why
        pip = (lay["pip_x"], lay["pip_y"], lay["pip_div"])
        rgb, why, res = None, None, None
        if snap.get("sensor_stamp_ns") is None:
            why = "no_stamp"
        elif not self.colour.has_source():
            why = "no_source"
        else:
            t0 = time.monotonic()
            res, why = self.colour.take(snap["frame_id"], snap.get("capture_ns", 0),
                                        float(tune.get("colour_wait_ms", self.colour_wait_ms)) / 1e3)
            self.colour_waits.append((time.monotonic() - t0) * 1e3)
            if res is not None:
                if (res["width"], res["height"]) != (Wd, Hd):
                    why = "size_mismatch"
                else:
                    rgb = res["rgb"]
        now = time.monotonic()
        col = {"pip": pip, "rgb": rgb, "width": Wd, "height": Hd, "held": None,
               "source_key": res.get("source_key") if (res is not None and rgb is not None) else None}
        last = self.colour_last
        if (rgb is not None and self._rf_period_ms(tune) > 0 and last is not None
                and last.get("refresh") and (last["width"], last["height"]) == (Wd, Hd)
                and last["source_key"] is not None and col["source_key"] is not None
                and int(last["source_key"]) > int(col["source_key"])):
            # colour refresh on: the screen already shows a newer colour than this frame's own; showing
            # the paired (older) one would make the background jump back in time for one picture
            self.counts["colour_refresh_kept_newer"] += 1
            col.update(rgb=last["rgb"], source_key=last["source_key"],
                       newer={"paired_key": int(res["source_key"]), "age_ms": round((now - last["t"]) * 1e3, 3)})
            return lay, col, why
        if rgb is not None:
            self.colour_last = {"rgb": rgb, "width": Wd, "height": Hd, "t": now,
                                "frame_id": snap["frame_id"], "source_key": col["source_key"]}
        else:
            last = self.colour_last
            hold_ms = float(tune.get("colour_hold_ms", self.colour_hold_ms))
            if last is not None and (last["width"], last["height"]) == (Wd, Hd):
                age_ms = (now - last["t"]) * 1e3
                if age_ms <= hold_ms:
                    col.update(rgb=last["rgb"], source_key=last["source_key"],
                               held={"from_frame_id": last["frame_id"], "age_ms": round(age_ms, 3)})
        return lay, col, why

    def submit(self, snap):
        """Called from the geometry worker after the answer left: never blocks."""
        if self.slot.put(snap):
            self.counts["superseded_before_send"] += 1
        self.counts["submitted"] += 1

    # -- no-depth self pull (AR_DEPTH_INVALID_STALL_20261017)
    def _nd_after_ms(self, tune):
        return float(tune.get("nodepth_after_ms", self.nodepth_after_ms))

    def _nd_note_real(self, snap, t_take):
        """Anchor from a real frame: sensor key clock - KV mono at the frame's acquisition (KV first
        chunk minus the Nano's capture->send elapsed). Every real frame ends a pull run."""
        self.nd_last_real = (snap, t_take)
        self.nd_last_pull = self.nd_first_pull = None
        key, el = snap.get("sensor_stamp_ns"), snap.get("nano_elapsed_ns")
        if key is not None and el is not None and int(key) >= NODEPTH_KEY_MIN:
            self.nd_offset_ns = int(key) - (int(snap["first_mono_ns"]) - int(el))
        elif key is not None:
            self.nd_offset_ns = None

    def _nd_due(self, now, tune):
        """Seconds until the next pull is due (<= 0: due now), or None when no pull applies."""
        after = self._nd_after_ms(tune)
        if (after <= 0 or self.nd_last_real is None or self.colour is None or not self._colour_on(tune)):
            return None
        if self.nd_first_pull is not None and now - self.nd_first_pull > self.nodepth_max_s:
            return None                 # a dead geometry chain ends in STALE, as before
        if self.nd_last_pull is None:
            return self.nd_last_real[1] + after / 1e3 - now
        return self.nd_last_pull + self.nodepth_period_ms / 1e3 - now

    def _nd_count(self, key):
        with self.lock:
            self.counts[key] += 1

    def _nd_pull(self, tune):
        """One pull: WANT the newest colour frame, build an all-zero-depth / no-geometry snapshot.
        Returns (snap, colour dict, record) or None (nothing is sent; the helper's STALE stays)."""
        now = time.monotonic()
        if self.nd_first_pull is None:
            self.nd_first_pull = now
        self.nd_last_pull = now
        base, _t = self.nd_last_real
        if self.nd_offset_ns is None:
            self._nd_count("nodepth_skip:key_not_time" if base.get("sensor_stamp_ns") is not None
                           else "nodepth_skip:no_key")
            return None
        if not self.colour.has_source():
            self._nd_count("nodepth_no_colour:no_source")
            return None
        t_w = int(now * 1e9)
        target = self.nd_offset_ns + t_w - int(self.nodepth_lead_ms * 1e6)
        self.nd_seq += 1
        fid, tag = int(base["frame_id"]), self.nd_seq          # tag: never a real capture_ns (RAW ns)
        if not self.colour.want(fid, tag, target, tol_ns=int(self.nodepth_tol_ms * 1e6)):
            self._nd_count("nodepth_no_colour:want_failed")
            return None
        self._nd_count("nodepth_wanted")
        res, why = self.colour.take(fid, tag, float(tune.get("colour_wait_ms", self.colour_wait_ms)) / 1e3)
        wait_ms = (time.monotonic() - now) * 1e3
        Wd, Hd = int(base["width"]), int(base["height"])
        if res is not None and (res["width"], res["height"]) != (Wd, Hd):
            res, why = None, "size_mismatch"
        if res is None:
            self._nd_count("nodepth_no_colour:%s" % why)
            if self.colour_log is not None:     # a pull that sent nothing is on record too
                self.colour_log.write(json.dumps({"seq": tag, "frame_id": fid, "decision": "nodepth_none",
                                                  "why": why, "want_key": target,
                                                  "want_to_frame_ms": round(wait_ms, 3),
                                                  "t_mono": round(time.monotonic(), 6)}) + "\n")
                self.colour_log.flush()
            return None
        k = int(res["source_key"])
        basis = max(0, t_w - (k - self.nd_offset_ns))      # estimated age of that colour since acquisition
        nvox = int(base["nx"]) * int(base["ny"]) * int(base["nz"])
        snap = {"frame_id": fid, "codes": np.zeros(Wd * Hd, dtype="<u2"), "unit_bits": base["unit_bits"],
                "width": Wd, "height": Hd, "nx": base["nx"], "ny": base["ny"], "nz": base["nz"],
                "origin": base["origin"], "voxel": base["voxel"], "world_camera": base["world_camera"],
                "flags": np.zeros(nvox, dtype=np.uint8), "geometry_status": 0, "roi": None,
                "roi_status": None, "roi_rows": [], "nano_elapsed_ns": basis, "labels": None,
                "replay": base.get("replay"), "first_mono_ns": t_w, "capture_ns": 0,
                "sensor_stamp_ns": k, "no_geometry": True}
        rec = {"seq": tag, "frame_id": fid, "want_key": target, "source_key": k,
               "age_basis_ms": round(basis / 1e6, 3), "want_to_frame_ms": round(wait_ms, 3),
               "since_real_ms": round((now - self.nd_last_real[1]) * 1e3, 3)}
        return snap, {"rgb": res["rgb"], "source_key": k}, rec

    # -- colour refresh (plan C, AR_LATENCY_PLAN_20261027)
    def _rf_period_ms(self, tune):
        return float(tune.get("colour_refresh_ms", getattr(self, "colour_refresh_ms", 0.0)))

    def _rf_due(self, now, tune):
        """Seconds until the next colour refresh (<= 0: now), or None when no refresh applies."""
        period = self._rf_period_ms(tune)
        if (period <= 0 or self.rf_inp is None or self.nd_last_real is None or self.colour is None
                or not self._colour_on(tune) or self.rf_last_render is None):
            return None
        after = self._nd_after_ms(tune)
        if after > 0 and now - self.nd_last_real[1] >= after / 1e3:
            return None                 # the no-depth pull takes over (as before)
        inp = self.rf_inp
        if inp["age_basis_ns"] >= 0:
            age_ms = (inp["age_basis_ns"] + (int(now * 1e9) - inp["first_mono_ns"])) / 1e6
            max_age = int(tune.get("max_age_ms", self.max_age_ms))
            if age_ms + period + COLOUR_REFRESH_AGE_MARGIN_MS >= max_age:
                return None             # never push the geometry picture into STALE
        return self.rf_last_render + period / 1e3 - now

    def _rf_pull(self, tune):
        """WANT the newest colour. ("refresh", inputs dict for the helper, record), ("real", snap, None)
        when a real frame arrived during the wait (it wins at once), or None (nothing to send)."""
        now = time.monotonic()
        self.rf_last_render = now       # one try per period (start to start), sent or not
        inp = self.rf_inp
        if self.nd_offset_ns is None:
            self._nd_count("refresh_skip:no_key")
            return None
        if not self.colour.has_source():
            self._nd_count("refresh_no_colour:no_source")
            return None
        col0 = inp["colour"]
        Wd, Hd = col0["width"], col0["height"]
        t_w = int(now * 1e9)
        target = self.nd_offset_ns + t_w - int(COLOUR_REFRESH_LEAD_MS * 1e6)
        self.nd_seq += 1
        fid, tag = int(inp["frame_id"]), self.nd_seq
        if not self.colour.want(fid, tag, target, tol_ns=int(self.nodepth_tol_ms * 1e6)):
            self._nd_count("refresh_no_colour:want_failed")
            return None
        end = now + float(tune.get("colour_wait_ms", self.colour_wait_ms)) / 1e3
        while True:
            res, why = self.colour.take(fid, tag, 0)
            if res is not None or why != "timeout" or time.monotonic() >= end:
                break
            real = self.slot.take(timeout=COLOUR_REFRESH_POLL_S)
            if real is not None:
                self._nd_count("refresh_preempted")
                return "real", real, None
        wait_ms = (time.monotonic() - now) * 1e3
        if res is not None and (res["width"], res["height"]) != (Wd, Hd):
            res, why = None, "size_mismatch"
        if res is None:
            self._nd_count("refresh_no_colour:%s" % why)
            return None
        k = int(res["source_key"])
        if self.rf_shown_key is not None and k <= int(self.rf_shown_key):
            self._nd_count("refresh_skip:not_newer")
            return None
        colour = dict(col0, rgb=res["rgb"], source_key=k, held=None)
        out = dict(inp, colour=colour)
        rec = {"t_start": round(now, 6), "seq": tag, "frame_id": fid, "source_key": k,
               "colour_age_ms": round(max(0, t_w - (k - self.nd_offset_ns)) / 1e6, 3),
               "want_to_frame_ms": round(wait_ms, 3),
               "since_real_ms": round((now - self.nd_last_real[1]) * 1e3, 3), "boxes": inp["boxes"]}
        return "refresh", out, rec

    def _apply_permit(self, inp):
        """AR2NANO2: the Nano's permit (present, on) into the message registers; no-op without a source."""
        if self.permit_source is None:
            return
        try:
            present, on = self.permit_source()
        except Exception:               # noqa: BLE001 -- display only: unknown permit = DRV N/A
            present, on = 0, 0
        regs = list(inp["regs"])
        regs[R["permit_present"]] = 1 if present else 0
        regs[R["permit_i"]] = 1 if (present and on) else 0
        inp["regs"] = regs

    def _rf_send(self, fd, inp, rec):
        try:
            self._apply_permit(inp)
            msg = encode_ar_message(inp)
            view = memoryview(msg)
            while view:
                n = os.write(fd, view)
                view = view[n:]
        except (BrokenPipeError, OSError) as exc:
            self.counts["helper_write_error"] += 1
            self.error = "write to helper failed: %s" % exc
            return False
        now = time.monotonic()
        self.rf_shown_key = rec["source_key"]
        c = inp["colour"]
        self.colour_last = {"rgb": c["rgb"], "width": c["width"], "height": c["height"], "t": now,
                            "frame_id": inp["frame_id"], "source_key": rec["source_key"], "refresh": True}
        with self.lock:
            self.counts["sent"] += 1
            self.counts["refresh_sent"] += 1
            self.rf_last = dict(rec, t_mono=round(now, 6))
            self.colour_state = {"state": "main_refresh", "reason": None, "frame_id": inp["frame_id"]}
        if self.colour_log is not None:
            self.colour_log.write(json.dumps(dict(rec, decision="refresh", t_mono=round(now, 6))) + "\n")
            self.colour_log.flush()
        return True

    def _send_loop(self):
        fd = self.proc.stdin.fileno()
        while not self._stop.is_set():
            tune = self._read_tune() if self.tune_file else self.tune
            now = time.monotonic()
            due = self._nd_due(now, tune)
            rdue = self._rf_due(now, tune)
            wait = min([0.2] + [max(0.0, d) for d in (due, rdue) if d is not None])
            snap = self.slot.take(timeout=wait)
            if snap is not None:
                self._nd_note_real(snap, time.monotonic())
                if not self._render(fd, snap):
                    return
                continue
            rdue = self._rf_due(time.monotonic(), tune)
            if rdue is not None and rdue <= 0:
                try:
                    rgot = self._rf_pull(tune)
                except Exception as exc:    # a refresh fault never touches the real frames
                    self._nd_count("refresh_error:%s" % type(exc).__name__)
                    rgot = None
                if rgot is None:
                    continue
                kind, item, rrec = rgot
                if kind == "refresh":
                    real = self.slot.take(timeout=0)    # a real frame came in meanwhile: it wins
                    if real is not None:
                        self._nd_count("refresh_preempted")
                        kind, item = "real", real
                if kind == "real":
                    self._nd_note_real(item, time.monotonic())
                    if not self._render(fd, item):
                        return
                    continue
                if not self._rf_send(fd, item, rrec):
                    return
                continue
            due = self._nd_due(time.monotonic(), tune)
            if due is None or due > 0:
                continue
            try:
                got = self._nd_pull(tune)
            except Exception as exc:        # a pull fault never touches the real frames
                self._nd_count("nodepth_error:%s" % type(exc).__name__)
                continue
            if got is None:
                continue
            real = self.slot.take(timeout=0)    # a real frame came in while we waited: it wins
            if real is not None:
                self._nd_count("nodepth_preempted")
                self._nd_note_real(real, time.monotonic())
                if not self._render(fd, real):
                    return
                continue
            nsnap, col, rec = got
            if not self._render(fd, nsnap, nodepth=(col, rec)):
                return

    def _render(self, fd, snap, nodepth=None):
        """Build and write one helper message. False = the helper pipe is gone (stop sending).
        ``nodepth``: (colour {"rgb", "source_key"}, record) of a self-pulled no-depth picture."""
        t0 = time.monotonic()
        try:
            if self.depth_upsample_to is not None:
                snap, s = upsample_snapshot(snap, self.depth_upsample_to)
                self.counts["depth_upsampled_x%d" % s if s > 1 else "depth_not_upsampled"] += 1
            key = (snap["width"], snap["height"])
            lay = self.layout_cache.get(key)
            if lay is None:
                lay = ar_layout(self.ready["width"], self.ready["height"], *key)
                self.layout_cache[key] = lay
            if lay is None:
                self.counts["layout_unfit"] += 1
                return True
            tune = self._read_tune()
            colour = None
            if nodepth is not None:
                clay, cwhy = self._colour_layout(tune, *key)
                if clay is None:
                    self.counts["nodepth_no_colour:%s" % cwhy] += 1
                    return True
                lay = clay
                ncol, rec = nodepth
                colour = {"pip": (lay["pip_x"], lay["pip_y"], lay["pip_div"]), "rgb": ncol["rgb"],
                          "width": key[0], "height": key[1], "held": None, "source_key": ncol["source_key"]}
                with self.lock:
                    self.counts["nodepth_sent"] += 1
                    self.nd_last = dict(rec, t_mono=round(time.monotonic(), 6))
                    self.colour_state = {"state": "main_nodepth", "reason": "no_geometry",
                                         "frame_id": snap["frame_id"]}
                if self.colour_log is not None:
                    self.colour_log.write(json.dumps(dict(rec, decision="nodepth", why="no_geometry",
                                                          held=None, t_mono=round(time.monotonic(), 6))) + "\n")
                    self.colour_log.flush()
            elif self._colour_on(tune):
                clay, colour, cwhy = self._colour_for(snap, tune, *key)
                if clay is not None:
                    lay = clay
                held = colour.get("held") if colour is not None else None
                with self.lock:
                    if colour is not None and colour["rgb"] is not None and held is None:
                        decision = "paired_newer" if colour.get("newer") else "paired"
                        self.counts["colour_paired"] += 1
                        self.colour_state = {"state": "main", "reason": None, "frame_id": snap["frame_id"]}
                    else:
                        self.counts["colour_unpaired:%s" % cwhy] += 1
                        if held is not None:
                            decision = "held"
                            self.counts["colour_held"] += 1
                            self.colour_held_age_ms_last = held["age_ms"]
                            if (self.colour_held_age_ms_max is None or
                                    held["age_ms"] > self.colour_held_age_ms_max):
                                self.colour_held_age_ms_max = held["age_ms"]
                            self.colour_state = {"state": "main_held", "reason": cwhy,
                                                 "frame_id": snap["frame_id"]}
                        else:
                            decision = "fallback"
                            self.counts["colour_fallback:%s" % cwhy] += 1
                            self.colour_state = {"state": "depth_fallback", "reason": cwhy,
                                                 "frame_id": snap["frame_id"]}
                if self.colour_log is not None:
                    self.colour_log.write(json.dumps({
                        "frame_id": snap["frame_id"], "decision": decision, "why": cwhy,
                        "source_key": colour.get("source_key") if colour else None,
                        "held": held, "newer": colour.get("newer") if colour else None,
                        "t_mono": round(time.monotonic(), 6)}) + "\n")
                    self.colour_log.flush()
            else:
                if self.colour is not None:
                    self.colour_last = None         # an off period never feeds a later hold
                    with self.lock:
                        self.colour_state = {"state": "off", "reason": None, "frame_id": snap["frame_id"]}
                if tune:
                    lay = dict(lay, **{k: int(tune[k]) for k in TUNE_LAYOUT_KEYS if k in tune})
            inp = fill_ar_inputs(snap, lay, source=self.source,
                                 permit_present=self.permit_present,
                                 max_age_ms=int(tune.get("max_age_ms", self.max_age_ms)),
                                 box_max=int(tune.get("box_max", self.box_max)),
                                 box_nms=(self.box_nms if "box_nms_pct" not in tune else
                                          (None if tune["box_nms_pct"] < 0 else tune["box_nms_pct"] / 100.0)),
                                 depth_stretch=(tune["depth_stretch_pct"] / 100.0 if "depth_stretch_pct" in tune
                                                else self.depth_stretch),
                                 native=self.fill_native)
            if "slice_y" in tune and nodepth is None:
                apply_slice_override(inp, snap, int(tune["slice_y"]))
            if colour is not None:
                # the helper enlarges the mini map only for a frame laid out for it (flag b1)
                inp["colour"] = (colour if int(lay.get("mini_scale", 1)) == 1 else
                                 dict(colour, mini_scale=int(lay["mini_scale"])))
            self._apply_permit(inp)
            msg = encode_ar_message(inp)
            t1 = time.monotonic()
            view = memoryview(msg)
            while view:
                n = os.write(fd, view)
                view = view[n:]
            t2 = time.monotonic()
            self.rf_last_render = t0            # refresh period counts start to start
            if colour is not None and nodepth is None:
                self.rf_inp = inp               # a refresh re-sends these inputs with a newer colour
                if colour.get("rgb") is not None:
                    self.rf_shown_key = colour.get("source_key")
            else:
                self.rf_inp = None              # no-depth / colour off: nothing to refresh from
                self.rf_shown_key = colour.get("source_key") if colour is not None else None
            with self.lock:
                self.counts["sent"] += 1
                self.counts["boxes"] += inp["boxes"]
                self.counts["hidden_boxes"] += inp["hidden_boxes"]
                self.counts["skipped_boxes"] += inp["skipped_boxes"]
                self.counts["named_boxes"] += inp["named_boxes"]
                self.counts["invalid_boxes"] += inp.get("invalid_boxes", 0)
                self.last_send = {"frame_id": inp["frame_id"], "fill_s": t1 - t0,
                                  "write_s": t2 - t1, "nodepth": nodepth is not None}
        except (BrokenPipeError, OSError) as exc:
            self.counts["helper_write_error"] += 1
            self.error = "write to helper failed: %s" % exc
            return False
        except Exception as exc:        # an AR fault never touches the geometry path
            self.counts["fill_error:%s" % type(exc).__name__] += 1
        return True

    # -- reporting
    def telem(self):
        """TELEM ``ar`` block (raw values; ns durations, ms intervals from the helper)."""
        with self.lock:
            lf = dict(self.last_frame) if self.last_frame else None
            fs = dict(self.last_failsafe) if self.last_failsafe else None
            lr = dict(self.last_render) if self.last_render else {}
            iv = list(self.intervals)
            counts = dict(self.counts)
        alive = self.proc.poll() is None
        # "mode" = the AR mode of the picture on screen (PS-side mirror of the core's rule,
        # m7ar_present.cpp mode_of; not read back from the core): LIVE REPLAY BADFRAME STALE
        # POSEBAD NODATA NOAGE, None before the first picture / when the core did not render.
        # The display path ("drm") moved to "presenter" (the hub reads ar.mode as the AR mode).
        out = {"available": True, "presenter": "drm", "source": self.source,
               "mode": lr.get("mode_name"), "age_ms": lr.get("age_ms"),
               "age_valid": lr.get("age_valid"), "replay": lr.get("replay"),
               "replay_core": lr.get("replay_core"), "frame_id": lr.get("frame_id"),
               "render_kind": lr.get("ev"), "render_seq": lr.get("seq"),
               "helper_alive": alive, "helper_pid": self.proc.pid,
               "display": [self.ready.get("width"), self.ready.get("height"),
                           self.ready.get("refresh_hz")],
               "scanout": self.ready.get("scanout"), "wait": self.ready.get("wait"),
               "submitted": counts.get("submitted", 0), "sent": counts.get("sent", 0),
               "flips": counts.get("ev:frame", 0), "failsafe_flips": counts.get("ev:failsafe", 0),
               "last_flip_frame_id": lf.get("frame_id") if lf else None,
               "last_flip_seq": lf.get("seq") if lf else None,
               "ar_core_ns": lf.get("ar_ns") if lf else None,
               "flip_interval_ms": lf.get("interval_ms") if lf else None,
               "last_age_ms": lf.get("age_ms") if lf else None,
               "last_status": lf.get("status") if lf else None,
               "failsafe_active": bool(fs and (not lf or fs.get("seq", 0) > lf.get("seq", 0))),
               "interval_ms_p50": float(np.median(iv)) if iv else None}
        if not alive:
            out["helper_exit"] = self.proc.returncode
        if self.colour is not None:         # only with the colour bypass: the ar keys stay as before
            with self.lock:
                st = dict(self.colour_state)
                waits = sorted(self.colour_waits)
            st.update(on=self._colour_on(), composited=counts.get("ev_colour_composited", 0),
                      paired=counts.get("colour_paired", 0), wanted=counts.get("colour_wanted", 0),
                      # unpaired = every colour-mode frame without its own colour (reason); of those,
                      # held = shown with the last paired colour (<= colour_hold_ms old), fallback =
                      # shown as the depth main view
                      unpaired={k.split(":", 1)[1]: v for k, v in counts.items()
                                if k.startswith("colour_unpaired:")},
                      held=counts.get("colour_held", 0), held_age_ms_max=self.colour_held_age_ms_max,
                      held_age_ms_last=self.colour_held_age_ms_last, hold_ms=self.colour_hold_ms,
                      fallback={k.split(":", 1)[1]: v for k, v in counts.items()
                                if k.startswith("colour_fallback:")},
                      wait_ms_p50=waits[len(waits) // 2] if waits else None,
                      wait_ms_max=waits[-1] if waits else None,
                      last_composite_ns=lf.get("composite_ns") if lf else None,
                      sink=self.colour.describe(),
                      # no-depth self pull (AR_DEPTH_INVALID_STALL_20261017): pictures sent without any
                      # geometry (core mode 4 NODATA + colour); no_colour / skip = pulls that sent nothing
                      nodepth={"after_ms": self._nd_after_ms(self.tune), "period_ms": self.nodepth_period_ms,
                               "tol_ms": self.nodepth_tol_ms, "lead_ms": self.nodepth_lead_ms,
                               "max_s": self.nodepth_max_s, "sent": counts.get("nodepth_sent", 0),
                               "wanted": counts.get("nodepth_wanted", 0),
                               "preempted": counts.get("nodepth_preempted", 0),
                               "no_colour": {k.split(":", 1)[1]: v for k, v in counts.items()
                                             if k.startswith("nodepth_no_colour:")},
                               "skip": {k.split(":", 1)[1]: v for k, v in counts.items()
                                        if k.startswith("nodepth_skip:")},
                               "last": dict(self.nd_last) if self.nd_last else None},
                      # colour refresh (plan C 10-27): pictures re-sent with the last geometry and a newer colour
                      refresh={"period_ms": self._rf_period_ms(self.tune), "sent": counts.get("refresh_sent", 0),
                               "preempted": counts.get("refresh_preempted", 0),
                               "kept_newer": counts.get("colour_refresh_kept_newer", 0),
                               "no_colour": {k.split(":", 1)[1]: v for k, v in counts.items()
                                             if k.startswith("refresh_no_colour:")},
                               "skip": {k.split(":", 1)[1]: v for k, v in counts.items()
                                        if k.startswith("refresh_skip:")},
                               "last": dict(self.rf_last) if self.rf_last else None})
            out["colour"] = st
        return out

    def describe(self):
        return {"cmd": self.cmd, "source": self.source, "ready": self.ready,
                "depth_upsample_to": self.depth_upsample_to,
                "fill": self.fill_native.describe() if self.fill_native is not None else {"impl": "python"},
                "pipe_bytes": self.pipe_bytes, "counts": dict(self.counts),
                "error": self.error, "exit_event": self.exit_event}

    def close(self, term_timeout_s=5.0):
        self._stop.set()
        try:
            self.proc.stdin.close()
        except Exception:
            pass
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(term_timeout_s)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        if hasattr(self, "_sender"):
            self._sender.join(2.0)      # 10-27: no picture from this presenter after close
        if hasattr(self, "_reader"):
            self._reader.join(2.0)
        if self.events_fp is not None:
            self.events_fp.close()
            self.events_fp = None
        if self.colour_log is not None:
            self.colour_log.close()
            self.colour_log = None
