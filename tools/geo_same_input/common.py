"""Shared pieces of the same-input geometry comparison (P8, 2026-09-27).

Kept free of cv2 / detector / memory imports so the arm runner also works on a
bare board image (KV260 tomorrow) with only numpy + g++.
See docs/design/GEO_SAME_INPUT_20260927.md.
"""
import hashlib
import json
import os

import numpy as np

# tools/nano_loop/chain.py default_grid(): the product validation grid of the
# fixed-camera loop and of the PL frame core (32^3 capacity).
GRID_ORIGIN_M = (-0.8, -0.8, -0.05)
GRID_VOXEL_M = 0.05
GRID_SHAPE = (32, 32, 32)          # nx, ny, nz
# Gemini 2 uint16 millimetres, 0 = no return. Only the unit of Gemini 2 recordings
# (prep_rec.py writes it into meta.json); every prepared input carries its own
# meta["depth_unit_m"] (TUM: 0.0002) and consumers must read it from there.
DEPTH_UNIT_M = 0.001
RECORD_HEADER = 12                 # 3 x uint32 counts before each frame's flags


def depth_u16_to_m(depth_u16, unit_m=DEPTH_UNIT_M):
    """uint16 code -> float32 metres, NaN = no return: ``f32(code) * f32(unit_m)``.

    With the default (Gemini 2 millimetres) bit-identical to
    tools/nano_loop/chain.depth_mm_to_m (test enforces it). Prepared inputs
    pass ``unit_m=meta_depth_unit_m(meta)``; TUM is 0.0002 m, and the 0.001
    default on TUM data is off by 5x yet still passes the pipeline's
    ``> 20 m`` scale check, so never rely on the default for prepared data.
    """
    depth = np.asarray(depth_u16)
    if depth.dtype != np.uint16 or depth.ndim != 2:
        raise ValueError("depth must be HxW uint16 codes")
    unit = float(unit_m)
    if not (unit > 0 and np.isfinite(unit)):
        raise ValueError("depth unit must be a positive finite number of metres")
    out = depth.astype(np.float32) * np.float32(unit)
    out[depth == 0] = np.nan
    return out


def meta_depth_unit_m(meta, override=None):
    """The prepared input's depth unit in metres, never a silent default.

    ``override`` (an explicit CLI value) wins only if meta has no unit or agrees
    with it; a disagreement is an error. Missing both is an error.
    Returns (unit_m, source) with source "meta" or "override".
    """
    have = meta.get("depth_unit_m")
    if have is not None:
        have = float(have)
        if override is not None and float(override) != have:
            raise ValueError("depth unit override %r disagrees with meta depth_unit_m %r"
                             % (override, have))
        return have, "meta"
    if override is None:
        raise ValueError("meta.json has no depth_unit_m; pass the unit explicitly "
                         "(e.g. --depth-unit-m 0.001 for Gemini 2 millimetres)")
    return float(override), "override"


def inverse_k_f32(K):
    """The float32 inverse K exactly as FrameNativeM7 computes it."""
    return np.ascontiguousarray(np.linalg.inv(np.asarray(K, dtype=np.float64)), dtype=np.float32)


def f32_hex(values):
    return [float(v).hex() for v in np.asarray(values, dtype=np.float32).ravel()]


def record_bytes(counts, flags):
    """One dump record: uint32 accepted/invalid/outside + flags in kernel order (z*ny+y)*nx+x.

    "outside" = endpoint outside the grid; since M7_CLIP_SPEC_20261011 such
    rays are clipped (walked inside the grid, no HIT), not dropped."""
    c = np.asarray(counts, dtype="<u4")
    f = np.ascontiguousarray(flags, dtype=np.uint8).ravel()
    if c.shape != (3,):
        raise ValueError("counts must have 3 entries")
    return c.tobytes() + f.tobytes()


def read_dump(path, nvox):
    """Return (counts[N,3] uint32, flags[N,nvox] uint8) from a dump file."""
    raw = np.fromfile(path, dtype=np.uint8)
    rec = RECORD_HEADER + nvox
    if raw.size == 0 or raw.size % rec:
        raise ValueError("%s: %d bytes is not a whole number of %d-byte records" % (path, raw.size, rec))
    raw = raw.reshape(-1, rec)
    counts = raw[:, :RECORD_HEADER].copy().view("<u4").reshape(-1, 3)
    return counts, raw[:, RECORD_HEADER:].copy()


def sha256_file(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as fp:
        for block in iter(lambda: fp.read(chunk), b""):
            h.update(block)
    return h.hexdigest()


def load_prepared(prep_dir):
    meta = json.load(open(os.path.join(prep_dir, "meta.json")))
    w, h, n = meta["width"], meta["height"], meta["frames"]
    depth = np.fromfile(os.path.join(prep_dir, meta["depth_file"]), dtype="<u2")
    if depth.size != w * h * n:
        raise ValueError("depth file holds %d values, meta says %d" % (depth.size, w * h * n))
    return meta, depth.reshape(n, h, w)
