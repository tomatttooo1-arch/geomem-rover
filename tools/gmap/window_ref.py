"""GMAP2: integer PS reference of the planner core's WINDOW operation (gmap_op = 4) -- the body seed and the swept
clearing of host/pl_grid_layer.PlGridLayer (car rules: seed 0.35 m, body 0.30 m, swept hold 300 s, tall evidence
within 1.0 m, capsule join <= 0.5 m, front sector off, cold ring off) in the integer arithmetic of
runs/nationals-prelim-20261102/GMAP0/DESIGN_ADD_PLGRID.md sections 2-3, with the GMAP3 precision change (lead 11-04:
Q8 failed the adoption gate on the cleared count, 1.45 %; positions go to 1/65536 cell, Q16):
  positions in window-local 1/65536 cells (Q16, must fit int24: |P| < 2^23), closest-point distance per axis clipped at
  21 cells, squared distances compared with r^2 (Q16^2, < 2^48), times in int32 ms from the session start,
  "never" = INT32_MIN,
  the swept-time map T is 400 x 400 on the global map lattice (window cell + (bx + 200, by + 200)).
The PS keeps the control part (A point, capsule sample points, Q16 rounding, now / thr) in ``params``; ``apply`` is the
per-cell part the PL does. ``param_block`` writes the 256-byte block the core reads (GMAP3 layout, Q16). Python 3.8, numpy only.
"""
import math
import struct

import numpy as np

V = 0.05
N, W = 400, 80
NEVER = -(1 << 31)
QS = 65536
DCLIP = 21 * QS
P24 = 1 << 23
HOLD_MS = 300000
MAX_STEP_M = 0.5
R_SEED_M, R_BODY_M, R_EVID_M = 0.35, 0.30, 1.0
MAGIC_W = 0x5747
FL_SEED, FL_SWEPT, FL_MERGE = 1, 2, 4
OFF_T, OFF_AUX, OFF_WPARAM, AREA_W = 526432, 1166432, 1172832, 1173088


def q16(m_abs, base_cells):
    """odom metres -> window-local Q16 (round half away from zero, as the PS does it)."""
    v = (float(m_abs) / V - int(base_cells)) * QS
    q = int(math.floor(v + 0.5)) if v >= 0 else -int(math.floor(-v + 0.5))
    assert -P24 <= q < P24, "position outside the int24 range of the core"
    return q


def rq2(r_m):
    q = int(round(float(r_m) / V * QS))
    return q * q


def axis_dist(P, n=W):
    a = np.arange(n, dtype=np.int64) * QS - int(P)
    b = a + QS
    d = np.where(a > 0, a, np.where(b < 0, -b, 0))
    return np.minimum(d, DCLIP)


class WindowQ16(object):
    def __init__(self, t0_ns, r_seed_m=R_SEED_M, r_body_m=R_BODY_M, r_evid_m=R_EVID_M):
        self.T = np.full((N, N), NEVER, np.int64)
        self.prev = None
        self.t0 = int(t0_ns)
        self.rs2, self.rb2, self.re2 = rq2(r_seed_m), rq2(r_body_m), rq2(r_evid_m)

    def params(self, base, body_xy, now_ns):
        """control part, PS side: returns the parameter dict and advances the capsule state."""
        bx, by = int(base[0]), int(base[1])
        xy = (float(body_xy[0]), float(body_xy[1]))
        a = self.prev if (self.prev is not None and
                          math.hypot(xy[0] - self.prev[0], xy[1] - self.prev[1]) <= MAX_STEP_M) else xy
        seg = math.hypot(xy[0] - a[0], xy[1] - a[1])
        n = max(1, int(math.ceil(seg / (V / 2.0))))
        pts = []
        for t in np.linspace(0.0, 1.0, n + 1):
            px, py = a[0] + t * (xy[0] - a[0]), a[1] + t * (xy[1] - a[1])
            pts.append((q16(px, bx), q16(py, by)))
        self.prev = xy
        now = (int(now_ns) - self.t0) // 1000000
        return dict(ix=bx + 200, iy=by + 200, cx=q16(xy[0], bx), cy=q16(xy[1], by), rs2=self.rs2, re2=self.re2,
                    rb2=self.rb2, now=now, thr=now - HOLD_MS, pts=pts)

    def apply(self, aux, p, seed=True, swept=True):
        """per-cell part (the PL): aux u8[6400] -> (frame bytes u8[6400]: 0xFF / 0x00 / 0x64, cleared, evidence)."""
        aux = np.asarray(aux, np.uint8).reshape(W, W)
        strict = (aux & 3).astype(np.int64)
        tall = ((aux >> 2) & 1).astype(bool)
        ix, iy = p["ix"], p["iy"]
        gy = iy + np.arange(W)[:, None] + np.zeros((1, W), np.int64)
        gx = ix + np.arange(W)[None, :] + np.zeros((W, 1), np.int64)
        inmap = (gx >= 0) & (gx < N) & (gy >= 0) & (gy < N)
        Tw = np.full((W, W), NEVER, np.int64)
        Tw[inmap] = self.T[gy[inmap], gx[inmap]]
        dx, dy = axis_dist(p["cx"]), axis_dist(p["cy"])
        d2 = dy[:, None] ** 2 + dx[None, :] ** 2
        seedm = d2 <= p["rs2"]
        near = d2 <= p["re2"]
        f = strict.copy()
        if seed:
            f[seedm & (f == 0)] = 1
        cleared = evidence = 0
        if swept:
            mk = np.zeros((W, W), bool)
            for (px, py) in p["pts"]:
                ex, ey = axis_dist(px), axis_dist(py)
                mk |= ey[:, None] ** 2 + ex[None, :] ** 2 <= p["rb2"]
            r0 = Tw >= p["thr"]
            strong = r0 & tall & near
            Tn = np.where(mk, p["now"], np.where(strong, NEVER, Tw))
            r1 = mk | (r0 & ~strong)
            cleared = int((r1 & (f != 1)).sum())
            evidence = int(strong.sum())
            f[r1] = 1
            self.T[gy[inmap], gx[inmap]] = Tn[inmap]
        code = np.where(f == 2, 0x64, np.where(f == 1, 0x00, 0xFF)).astype(np.uint8)
        return code.ravel(), cleared, evidence


def param_block(p, flags=FL_SEED | FL_SWEPT, rb2=None, re2=None, rs2=None):
    """256-byte WINDOW parameter block (GMAP3 Q16 layout; DESIGN_ADD_PLGRID section 2.3 widened):
    0 u16 magic 0x5747 | 2 i16 ix | 4 i16 iy | 6 u8 flags (bit0 seed, bit1 swept; bit2 reserved = 0) | 7 u8 m (<= 21)
    8 i32 now | 12 i32 thr (ms) | 16 u64 rs2 | 24 u64 re2 | 32 u64 rb2 (Q16^2, < 2^48)
    40 i32 cx | 44 i32 cy (body, Q16, int24 range; right before the points so the core reads both alike)
    48 + 8k: i32 x, i32 y of point k (Q16), k < m | rest 0 up to 256.
    rb2 / re2 / rs2 override the dict values (positive controls)."""
    pts = p["pts"]
    assert 0 <= len(pts) <= 21
    b = struct.pack("<HhhBBii", MAGIC_W, p["ix"], p["iy"], flags, len(pts), p["now"], p["thr"])
    b += struct.pack("<QQQii", p["rs2"] if rs2 is None else rs2, p["re2"] if re2 is None else re2,
                     p["rb2"] if rb2 is None else rb2, p["cx"], p["cy"])
    assert len(b) == 48
    for (x, y) in pts:
        b += struct.pack("<ii", x, y)
    return b + bytes(256 - len(b))
