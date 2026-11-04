"""Independent golden for the clipped M7 frame (docs/design/M7_CLIP_SPEC_20261011.md).

The traversal part is NOT a DDA: for every in-grid voxel near a ray it asks
whether the Q8 segment meets the voxel's OPEN interior for a positive length
(slab test), with the half-open rule for an axis that does not move (the line
belongs to the cell floor(x/256), i.e. the positive side of a plane). Two
independent evaluations of that test are provided and must agree:

  * ``double``: the 09-21 dense-testbench slab in float64; for integer Q8
    inputs with |coordinate| < 2^24 every slab time is a ratio of integers
    below 2^25, distinct ratios differ by > 2^-50 and equal ratios round to
    the same double, so the comparisons are exact;
  * ``rational``: the same slab with every time kept as an exact integer
    fraction and compared by int64 cross products (products < 2^51).

Clip semantics: only in-grid voxels are ever marked; the camera-origin voxel
is never marked FREE; the endpoint voxel becomes HIT only when the endpoint is
inside the grid; HIT wins over FREE.

The projection part (float32 back-projection, float64 Q8 floor) re-implements
the reference arithmetic in numpy with the same operation order; it is shared
semantics, not part of what this golden independently checks.
"""
import numpy as np

Q8 = 256
Q8_LIMIT = 1 << 24


def floor_div_q8(values):
    return np.floor_divide(np.asarray(values, dtype=np.int64), Q8)


def project_frame(depth, inverse_k, world_camera, grid_origin, voxel, dims):
    """Return (origin_q8[3], ends_q8[N,3] int64, cls[N]) for every pixel.

    cls: 0 accepted (end in grid), 1 invalid, 2 clipped (end outside grid).
    Mirrors hw/m7/frame_depth.cpp operation by operation in float32/float64.
    """
    depth = np.ascontiguousarray(depth, dtype=np.float32)
    h, w = depth.shape
    ik = np.asarray(inverse_k, dtype=np.float32).ravel()
    wc = np.asarray(world_camera, dtype=np.float32).ravel()
    go = np.asarray(grid_origin, dtype=np.float64)
    dims = np.asarray(dims, dtype=np.int64)
    origin = np.floor((wc[[3, 7, 11]].astype(np.float64) - go) / voxel * Q8)
    if np.any(origin < 0) or np.any(origin >= dims * Q8):
        raise ValueError("camera origin outside grid")
    origin = origin.astype(np.int64)
    z = depth.ravel()
    pix = np.arange(z.size, dtype=np.int64)
    u = (pix % w).astype(np.float32)
    v = (pix // w).astype(np.float32)
    with np.errstate(all="ignore"):
        cam = [((ik[3 * r] * u + ik[3 * r + 1] * v) + ik[3 * r + 2]) * z for r in range(3)]
        world = [((wc[4 * r] * cam[0] + wc[4 * r + 1] * cam[1]) + wc[4 * r + 2] * cam[2]) + wc[4 * r + 3]
                 for r in range(3)]
        valid = (z > np.float32(0.1)) & (z < np.float32(10.0))
        finite = np.isfinite(world[0]) & np.isfinite(world[1]) & np.isfinite(world[2])
        q = np.stack([np.floor((world[a].astype(np.float64) - go[a]) / voxel * Q8) for a in range(3)], axis=1)
    representable = np.all((q >= -Q8_LIMIT) & (q < Q8_LIMIT), axis=1)  # NaN -> False
    good = valid & finite & representable
    ends = np.zeros((z.size, 3), dtype=np.int64)
    ends[good] = q[good].astype(np.int64)
    inside = good & np.all((ends >= 0) & (ends < dims * Q8), axis=1)
    cls = np.full(z.size, 1, dtype=np.int8)
    cls[good] = 2
    cls[inside] = 0
    return origin, ends, cls


def _candidate_cells(start, end, dims):
    lo = np.minimum(floor_div_q8(start), floor_div_q8(end))
    hi = np.maximum(floor_div_q8(start), floor_div_q8(end))
    lo = np.maximum(lo, 0)
    hi = np.minimum(hi, dims - 1)
    if np.any(lo > hi):
        return np.zeros((0, 3), dtype=np.int64)
    axes = [np.arange(lo[a], hi[a] + 1, dtype=np.int64) for a in range(3)]
    gx, gy, gz = np.meshgrid(*axes, indexing="ij")
    return np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)


def _hits_double(start, end, cells):
    low_t = np.zeros(len(cells))
    high_t = np.ones(len(cells))
    keep = np.ones(len(cells), dtype=bool)
    for a in range(3):
        d = int(end[a] - start[a])
        low = cells[:, a] * Q8
        high = low + Q8
        if d == 0:
            keep &= (int(start[a]) // Q8) == cells[:, a]
            continue
        ta = (low - start[a]).astype(np.float64) / d
        tb = (high - start[a]).astype(np.float64) / d
        low_t = np.maximum(low_t, np.minimum(ta, tb))
        high_t = np.minimum(high_t, np.maximum(ta, tb))
    return keep & (low_t < high_t)


def _hits_rational(start, end, cells):
    n = len(cells)
    ln, ld = np.zeros(n, dtype=np.int64), np.ones(n, dtype=np.int64)   # low_t = ln/ld
    hn, hd = np.ones(n, dtype=np.int64), np.ones(n, dtype=np.int64)    # high_t = hn/hd
    keep = np.ones(n, dtype=bool)
    for a in range(3):
        d = int(end[a] - start[a])
        low = cells[:, a] * Q8
        high = low + Q8
        if d == 0:
            keep &= (int(start[a]) // Q8) == cells[:, a]
            continue
        s = 1 if d > 0 else -1
        den = abs(d)
        en = (low if d > 0 else high) - start[a]   # entering plane
        lv = (high if d > 0 else low) - start[a]   # leaving plane
        en, lv = en * s, lv * s
        # low_t = max(low_t, en/den)
        bigger = en * ld > ln * den
        ln = np.where(bigger, en, ln)
        ld = np.where(bigger, den, ld)
        smaller = lv * hd < hn * den
        hn = np.where(smaller, lv, hn)
        hd = np.where(smaller, den, hd)
    return keep & (ln * hd < hn * ld)


def golden_rays(origin, ends, inside, dims, method="rational"):
    """Flags (flat, kernel order (z*ny+y)*nx+x) from Q8 segments; ends unique or not."""
    dims = np.asarray(dims, dtype=np.int64)
    nx, ny, nz = (int(v) for v in dims)
    free = np.zeros(nx * ny * nz, dtype=bool)
    hit = np.zeros_like(free)
    origin = np.asarray(origin, dtype=np.int64)
    ocell = floor_div_q8(origin)
    test = _hits_rational if method == "rational" else _hits_double
    if len(ends):
        uniq, first = np.unique(np.asarray(ends, dtype=np.int64), axis=0, return_index=True)
        uin = np.asarray(inside, dtype=bool)[first]
    else:
        uniq, uin = np.zeros((0, 3), dtype=np.int64), np.zeros(0, dtype=bool)
    for end, ins in zip(uniq, uin):
        cells = _candidate_cells(origin, end, dims)
        if len(cells) == 0:
            continue
        m = test(origin, end, cells)
        ecell = floor_div_q8(end)
        m &= ~np.all(cells == ocell, axis=1)
        if ins:
            m &= ~np.all(cells == ecell, axis=1)
            hit[(ecell[2] * ny + ecell[1]) * nx + ecell[0]] = True
        c = cells[m]
        free[(c[:, 2] * ny + c[:, 1]) * nx + c[:, 0]] = True
    return np.where(hit, 2, np.where(free, 1, 0)).astype(np.uint8)


def golden_frame(depth, inverse_k, world_camera, grid_origin, voxel, dims, method="rational"):
    origin, ends, cls = project_frame(depth, inverse_k, world_camera, grid_origin, voxel, dims)
    walk = cls != 1
    flags = golden_rays(origin, ends[walk], cls[walk] == 0, dims, method)
    counts = (int((cls == 0).sum()), int((cls == 1).sum()), int((cls == 2).sum()))
    return flags, counts
