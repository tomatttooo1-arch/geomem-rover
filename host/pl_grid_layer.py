"""B1 (nationals, 2026-11-02): the PL navigation grid on the Nano -- the ROS-free half.

The KV260 fusion core can return, with every INTEGRATE, the 2-D navigation planes of the map it now holds
(hw/b1_costmap, runs/nationals-prelim-20261102/B1/DESIGN.md sections 2.3-2.4):

  grid  int8  [ly*80+lx]  100 / 0 / -1, the D-035 height-band projection (+ the body seed when asked)
  aux   uint8 [ly*80+lx]  bits 0-1 strict tri-state (0 unknown, 1 free, 2 occupied), bit 2 "tall"
                          (an OCCUPIED band voxel centred >= ground + 0.15 m), bits 3-7 zero
  cost  uint8 (optional)  Nav2 costs (display / diagnosis only, never handed to Nav2)

``PlGridLayer`` is host/rolling_nav_layer.RollingLayerBridge with its INPUT side replaced: no delta
mirror, no resync -- every frame's planes are self-contained. The OUTPUT side is the bridge's own code
(strict_grid / grid / status, the body seed, the front seed, the swept clearing with the PL tall bit in
place of the mirror's tall_occupied), so /fpga/rolling_map/grid and /status keep their fields and meaning
(fresh / age_ns: the capture time of the last frame whose grid was applied, same 200 ms rule).

``dump_check`` is the coverage the bridge's resync used to give for free ("the KV map and the Nano's
mirror agree"): a DUMP of the KV map projected with the existing Python code, compared cell by cell with
the latest PL planes. A mismatch is reported, never acted on (FPGA rule: no new stop / refuse paths).

Python 3.8 compatible, numpy only.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np

from host import rolling_nav_layer as rnl
from host.m7_rolling import FREE, OCCUPIED, UNKNOWN  # noqa: F401 (re-exported for callers)

# ROLL v2 grid extension (host/kv_rolling.py; interface in B1/NANO_SIDE.md "接口约定")
GRID_F_GRID, GRID_F_BODY, GRID_F_COST = 1, 2, 4
LUT_DIM = 14                       # hw/b1_costmap/costmap.hpp B1_LUT_DIM (B1_RMAX + 2)
LUT_SIZE = LUT_DIM * LUT_DIM
Q16 = 65536                        # body circle: window-local voxels in Q16
GRID_OK, GRID_NOT_RUN = 0, 0x100
GRID_STATUS_NAMES = {0: "ok", 1: "bad_offsets", 2: "bad_band", 3: "bad_body", 4: "bad_lut",
                     GRID_NOT_RUN: "not_run"}


def band_params(band, window_base_z, mz, voxel_m, min_h_m=rnl.SWEPT_EVIDENCE_MIN_H_M):
    """(band_lo, band_hi, tall_lo) local z layers for the core, from the same HeightBand the bridge uses.

    Raises ValueError when the band is not inside the window (the bridge cannot project then either) or
    is not contiguous / the tall layers are not a suffix of it (the core's encoding needs both)."""
    layers = [int(v) for v in band.band_layers(window_base_z, mz, voxel_m)]
    lo, hi = layers[0], layers[-1]
    if layers != list(range(lo, hi + 1)):
        raise ValueError("obstacle band not contiguous")
    min_z = band.ground_z_m + float(min_h_m)
    # same test as rolling_nav_layer.tall_occupied
    tall = [lz for lz in layers if (int(window_base_z) + lz + 0.5) * float(voxel_m) >= min_z]
    tall_lo = tall[0] if tall else hi + 1
    if tall and tall != list(range(tall_lo, hi + 1)):
        raise ValueError("tall layers not a suffix of the band")
    return lo, hi, tall_lo


def body_q16(window_base, voxel_m, xy, radius_m):
    """Body circle for the core (only when the body seed runs on the PL; default it runs on the Nano)."""
    cx = int(round((float(xy[0]) / voxel_m - int(window_base[0])) * Q16))
    cy = int(round((float(xy[1]) / voxel_m - int(window_base[1])) * Q16))
    return cx, cy, int(round(float(radius_m) / voxel_m * Q16))


def default_lut(resolution=0.05, inflation_radius=0.45, cost_scaling_factor=3.0, robot_radius=0.30):
    """The 14 x 14 Nav2 cost table sent with CONFIG (car Nav2 values, config/rolling_layer_nav2.yaml);
    None when tools/b1_costmap is not importable (the table is only used when inflation is asked for)."""
    try:
        from tools.b1_costmap.nav2_ref import InflationParams
    except ImportError:
        return None
    p = InflationParams(resolution=resolution, inflation_radius=inflation_radius,
                        cost_scaling_factor=cost_scaling_factor, robot_radius=robot_radius)
    return np.ascontiguousarray(p.pl_lut(LUT_DIM), dtype=np.uint8).ravel()


@dataclass(frozen=True)
class PlGridPlanes:
    """One INTEGRATE's planes as the KV answered them (local order, row ly, column lx)."""
    status: int
    grid: np.ndarray                 # int8 [my*mx]
    aux: np.ndarray                  # uint8 [my*mx]
    cost: Optional[np.ndarray] = None
    body: bool = False               # the core seeded the body circle into ``grid``

    def strict(self, dims):
        return (self.aux & 3).astype(np.uint8).reshape(int(dims[1]), int(dims[0]))

    def tall(self, dims):
        return ((self.aux >> 2) & 1).astype(bool).reshape(int(dims[1]), int(dims[0]))


def plane_problems(planes, cells):
    """[] when the planes are well formed; else short reasons (counted by the backend, never fatal)."""
    out = []
    if planes.grid.size != cells or planes.aux.size != cells:
        return ["size"]
    if not np.isin(planes.grid, (-1, 0, 100)).all():
        out.append("grid_value")
    if np.any(planes.aux & 0xF8) or np.any((planes.aux & 3) == 3):
        out.append("aux_value")
    if not planes.body:
        strict = (planes.aux & 3).astype(np.uint8)
        if np.any(rnl.occupancy_values(strict) != planes.grid):
            out.append("grid_vs_aux")
    return out


class _PlMirror(object):
    """What RollingLayerBridge reads from its mirror, set from the PL planes (no 3-D map on the Nano)."""

    def __init__(self):
        self.epoch, self.delta_seq, self.window_base = None, 0, None
        self.states = None            # anything but None once a grid is in (status / strict_grid test it)
        self.needs_resync = False


class PlGridLayer(rnl.RollingLayerBridge):
    """RollingLayerBridge fed with PL planes instead of deltas. Same constructor arguments."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.mirror = _PlMirror()
        self._strict2d = None
        self._tall2d = None
        self.last_planes = None

    def on_planes(self, *, epoch, delta_seq, window_base, acquired_ns, clock_id, planes):
        """Adopt one INTEGRATE's planes. Returns (applied, reason)."""
        if clock_id != self.clock_id:
            self.counts["rejected:clock_mismatch"] += 1
            return False, "clock_mismatch"
        if not isinstance(planes, PlGridPlanes) or planes.status != GRID_OK:
            self.counts["rejected:no_planes"] += 1
            return False, "no_planes"
        mx, my = self.dims[0], self.dims[1]
        if planes.aux.size != mx * my:
            self.counts["rejected:geometry_mismatch"] += 1
            return False, "geometry_mismatch"
        m = self.mirror
        m.epoch, m.delta_seq = int(epoch), int(delta_seq)
        m.window_base = tuple(int(v) for v in window_base)
        m.states = True
        self._strict2d = planes.strict(self.dims)
        self._tall2d = planes.tall(self.dims)
        self.last_planes = planes
        self._acquired_ns = int(acquired_ns)
        self._grid = None
        self.counts["applied"] += 1
        return True, "applied"

    # -- the bridge's projection, from the PL ---------------------------------------------------------
    def tri2d(self):
        return None if self._strict2d is None else self._strict2d

    def tall2d(self):
        return self._tall2d

    def _apply_swept(self, tri, body_xy, now_ns):
        """RollingLayerBridge._apply_swept with the PL tall bit in place of tall_occupied(mirror.states)
        (B1 bench: tall plane 0 cells different from tall_occupied on 376 rec30 frames)."""
        now = int(now_ns) if now_ns is not None else None
        if now is None:
            self.counts["swept_no_time"] += 1
            return tri
        base, dims, v = self.mirror.window_base, self.dims, self.voxel_m
        recent = self.swept.window_mask(base, dims, now, self.swept_clear_ns)
        if recent.any():
            strong = recent & self._tall2d & rnl.body_cells(base, dims, v, body_xy, self.swept_evidence_near_m)
            if strong.any():
                self.swept.forget(base, strong)
                self.counts["swept_evidence_cells"] += int(strong.sum())
        self.swept.mark(body_xy, self.body_radius_m, now)
        recent = self.swept.window_mask(base, dims, now, self.swept_clear_ns)
        out = np.array(tri, dtype=np.uint8, copy=True)
        n = int((recent & (out != FREE)).sum())
        out[recent] = FREE
        self.counts["swept_cleared_cells"] += n
        self.last_swept_cleared = n
        return out

    def on_delta(self, delta, *, now_ns):          # pragma: no cover - wrong input for this layer
        raise TypeError("PlGridLayer takes PL planes (on_planes), not deltas")


def dump_check(snapshot, layer, band, *, min_h_m=rnl.SWEPT_EVIDENCE_MIN_H_M):
    """Compare a DUMP of the KV map (host.m7_rolling.MapSnapshot, valid) with the layer's latest PL planes.

    Returns a dict: result "match" | "mismatch" | "not_comparable" (another map state: the epoch / seq /
    window differ, e.g. a frame fused between), the cell counts that differ, and the identities."""
    m = layer.mirror
    row = {"dump": [int(snapshot.epoch), int(snapshot.delta_seq)], "grid": [m.epoch, m.delta_seq],
           "strict_cells": None, "tall_cells": None}
    if not snapshot.valid or layer._strict2d is None:
        row["result"] = "not_comparable"
        return row
    if (int(snapshot.epoch), int(snapshot.delta_seq), tuple(snapshot.window_base)) != \
            (m.epoch, m.delta_seq, m.window_base):
        row["result"] = "not_comparable"
        return row
    base_z = int(snapshot.window_base[2])
    layers = band.band_layers(base_z, layer.dims[2], layer.voxel_m)
    strict_sw = rnl.project_columns(snapshot.states, layers)
    tall_sw = rnl.tall_occupied(snapshot.states, base_z, layer.voxel_m, band.ground_z_m + float(min_h_m), layers)
    row["strict_cells"] = int((strict_sw != layer._strict2d).sum())
    row["tall_cells"] = int((tall_sw != layer._tall2d).sum())
    row["result"] = "match" if row["strict_cells"] == 0 and row["tall_cells"] == 0 else "mismatch"
    return row


def lut_problem(lut):
    """None when the table is one the PL GRID op takes, else the reason: costmap.hpp's rule (254 at [0][0],
    last row / column 0) and, since the fused core B1h-v2 (folded taps, PL reason code 4), every row
    non-increasing along |dy| (INT_SW 2026-11-02; any Nav2 table is). Shared by the KV (CONFIG v2) and
    the Nano (before sending), so neither keeps a table the PL would refuse frame after frame."""
    t = np.asarray(lut, dtype=np.uint8)
    if t.size != LUT_SIZE:
        return "size"
    t = t.reshape(LUT_DIM, LUT_DIM)
    if int(t[0, 0]) != 254:
        return "corner"
    if t[-1, :].any() or t[:, -1].any():
        return "edge"
    if (np.diff(t.astype(np.int16), axis=1) > 0).any():
        return "row_increases"
    return None


def lut_ok(lut):
    """True when ``lut_problem`` finds nothing (the PL core would accept the table)."""
    return lut_problem(lut) is None


__all__ = ["GRID_F_GRID", "GRID_F_BODY", "GRID_F_COST", "LUT_DIM", "LUT_SIZE", "Q16", "GRID_OK", "GRID_NOT_RUN",
           "GRID_STATUS_NAMES", "band_params", "body_q16", "default_lut", "PlGridPlanes", "plane_problems",
           "PlGridLayer", "dump_check", "lut_ok", "lut_problem"]

