"""10-26 probabilistic search (owner: 正式版概率搜索), on top of the search MVP (host/task_control.py).

No ROS, numpy only; the runner (host/ros_runtime.py --search-mode prob) and the preview tool
(tools/car_run/search_preview.py) share it.

Belief grid. ``cell_m`` cells over the occupancy grid (unknown / occupied / free as FreeMap: >= 50 occupied, -1
unknown), cropped to the mapping trajectory's dense nodes +- (max_range_m + 0.5 m). A cell belongs to the search
universe only when it is free (no occupied sub-cell, at least half known free), within ``max_range_m`` of a
trajectory node AND visible from that node (ray over the occupancy grid, occupied cells block). Places the car never
went near, or never could see, are not searched (owner: the car does not go where it has never been).

Prior. Remembered target: Gaussian of the distance to the remembered position (``prior_sigma_m``) x ``edge_weight``
on cells within ``edge_m`` of an occupied cell (things are moved to a wall / a furniture foot), plus ``near_mass`` of
the total as a narrow Gaussian (``near_sigma_m``) at the remembered position (a detector miss there). 10-29 (owner:
the object may be put far away): ``uniform_floor`` (0.3) of the total is spread uniformly x the edge weight over the
whole searched area, so the remembered place is still looked at first and, if the object is not there, the car goes on
to look everywhere it went. No memory: uniform x the same edge weight. Normalised to 1.

Observation. Each trusted frame (at most ``update_hz``): the cells the camera sees, within range and not hidden
behind occupied cells, are multiplied by (1 - q(d)). 10-28 (blind zone): with a camera model (``set_camera`` /
``observe(K=, image_size=, T_base_camera=)``: the runtime's intrinsics, image size and base->optical mount) a cell is
seen only when a small object standing on it is WHOLLY in the image (owner: the detector needs the whole object):
the four points of a ``object_radius_m`` x ``object_height_m`` object -- bottom near edge, bottom left / right edges
(z = 0) and top far edge (z = object_height_m) -- all project inside the frame through the real camera pose (tilt,
roll and height follow the live mount). The floor right in front of the lens (below the image's bottom edge: ~0.5 m
at a 16 deg tilt) therefore stays unsearched (q = 0) instead of "looked at, 90 % not there". Without a camera model
the old horizontal wedge (``fov_deg``) is used. q is the
piecewise-linear detection-rate table ``q_table`` (distance m -> rate; below the first entry the first rate, beyond
the last 0). Views from within ``look_move_m`` of a cell's previous view are the SAME look (consecutive frames are
not independent): only the better rate counts (miss x (1-q_new)/(1-q_old) when q_new > q_old). A cell never drops
below ``miss_floor`` of its prior (a detector can miss an object at any range).

Next look. Candidates = dense nodes x ``yaws`` headings (k x 360/yaws deg). Gain = the mass the view would remove
(the same look rule; with a camera model the same whole-object-in-frame test from the camera at that base pose, so a
node right next to the remembered place gains little and 0.9-1.2 m looks win); score = gain / (path_m + turn_cost_m_per_rad x turn + step_cost_m); path = 8-connected
Dijkstra on the coarse grid (no occupied sub-cell, ``robot_radius_m`` clearance except within ``node_clear_m`` of a
trajectory node, unknown allowed only near the trajectory); unreachable candidates are dropped, a node whose
navigation failed is not offered again, a (node, heading) already chosen is not chosen again. A candidate needs gain
>= ``min_gain`` or (10-29) to newly see >= ``explore_m2`` of area not looked at yet (in a large area the floor's share
per look drops under min_gain while there is still unseen floor).

Stop. Remaining mass < ``stop_mass`` of the initial AND (10-29) the prior mass on cells some candidate could see at
q >= 0.5 but not looked at yet (miss > 0.5) < ``stop_unseen`` -- with the floor the area is looked over before the
mass stop counts --, or no candidate -> None (the controller ends not_found/search_exhausted, the MVP's word).
10-29 stop sweep (``stop_sweep``): before that None, the (node, heading) views not chosen yet that would still newly
see >= ``sweep_min_m2`` of not-looked-at floor at q >= 0.5 are looked from, best new area per cost first, however
little mass they hold (a 2.4 m move in the host sim stopped with no view ever facing the object); views that cannot
be reached and looked at in the task's remaining time (``next_goal(time_left_s=)``) are left out.
"""

from dataclasses import dataclass, fields
import heapq
import json
import math
import time

import numpy as np

DEFAULT_Q_TABLE = ((0.6, 0.9), (0.9, 0.9), (1.2, 0.85), (1.5, 0.5), (1.6, 0.4))   # 10-26 car: upright cup-like object 100/100/100/64/1 % at 0.6-2.0 m


SWEEP_LIN_MPS, SWEEP_ANG_RADPS, SWEEP_LOOK_S = 0.10, 0.30, 3.0   # car averages incl. starts / stops; 3 s look


def sweep_eta(path_m, turn_rad):
    """Seconds to drive ``path_m``, turn ``turn_rad`` and take the look (the car's average speeds)."""
    return path_m / SWEEP_LIN_MPS + turn_rad / SWEEP_ANG_RADPS + SWEEP_LOOK_S


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def _wrap1(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class SearchConfig:
    cell_m: float = 0.1
    max_range_m: float = 1.6
    prior_sigma_m: float = 1.0
    edge_m: float = 0.3
    edge_weight: float = 3.0
    near_mass: float = 0.10
    near_sigma_m: float = 0.3
    fov_deg: float = 86.0
    q_table: tuple = DEFAULT_Q_TABLE
    update_hz: float = 5.0
    look_move_m: float = 0.5
    miss_floor: float = 0.05
    yaws: int = 8
    turn_cost_m_per_rad: float = 0.2
    step_cost_m: float = 1.0
    stop_mass: float = 0.10
    min_gain: float = 0.005
    robot_radius_m: float = 0.15
    node_clear_m: float = 0.3
    max_cells: int = 60000
    object_height_m: float = 0.10     # 10-28 frustum test: a small household object (cup, banana, key, phone)
    object_radius_m: float = 0.05
    # 10-29 (owner: 评委可能把东西放远): a remembered target's prior keeps this share spread uniformly (x the edge
    # weight) over the whole searched area, so after the remembered place the car goes on to look everywhere it went
    uniform_floor: float = 0.3
    # 10-29: the mass stop only counts once the prior mass on cells not looked at yet (miss > 0.5 although a
    # candidate view could see them at q >= 0.5) is below this; until then the search goes on (no_candidate and the
    # task budget still end it); 1 = off (the mass alone, before 10-29). 0.02 -> 0.01 with the stop sweep (owner
    # 10-29: lower the stop line a little): the gain-ordered looks go on a little longer before the sweep
    stop_unseen: float = 0.01
    # 10-29: a look that would newly see (q >= 0.5) at least this much not-yet-seen area stays a candidate even below
    # min_gain (in a large area the floor's share per look falls under min_gain; min_gain still ends re-looks)
    explore_m2: float = 0.25
    # 10-29 (owner: 判搜完前补看没朝向过的方向): when the stop rule fires (mass_below_stop / no_candidate), first
    # look from the not-yet-chosen reachable (trajectory node, heading) views that would still newly see (q >= 0.5)
    # at least ``sweep_min_m2`` of not-yet-looked-at searchable floor, however little prior mass it holds -- best
    # new area per cost first; the search ends only when no such view is left (or the task budget runs out).
    # Each such look clears what it sees and a chosen view is never chosen again, so it does not circle over
    # looked-at floor. False = the stop as before.
    stop_sweep: bool = True
    sweep_min_m2: float = 0.02

    def __post_init__(self):
        q = tuple((float(d), float(r)) for d, r in self.q_table)
        object.__setattr__(self, "q_table", q)
        if not q or any(not (math.isfinite(d) and d >= 0 and 0 <= r < 1) for d, r in q) or \
                any(q[i][0] >= q[i + 1][0] for i in range(len(q) - 1)):
            raise ValueError("q_table: increasing distances, rates in [0, 1)")
        for name in ("cell_m", "max_range_m", "prior_sigma_m", "near_sigma_m", "fov_deg", "update_hz"):
            v = getattr(self, name)
            if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0):
                raise ValueError("%s must be > 0" % name)
        for name in ("edge_m", "look_move_m", "turn_cost_m_per_rad", "step_cost_m", "robot_radius_m",
                     "node_clear_m", "min_gain", "object_height_m", "object_radius_m", "explore_m2",
                     "sweep_min_m2"):
            v = getattr(self, name)
            if not (isinstance(v, (int, float)) and math.isfinite(v) and v >= 0):
                raise ValueError("%s must be >= 0" % name)
        for name in ("near_mass", "miss_floor", "stop_mass", "uniform_floor"):
            v = getattr(self, name)
            if not (isinstance(v, (int, float)) and 0 <= v < 1):
                raise ValueError("%s must be in [0, 1)" % name)
        if not (isinstance(self.stop_unseen, (int, float)) and 0 <= self.stop_unseen <= 1):
            raise ValueError("stop_unseen must be in [0, 1]")
        if not isinstance(self.stop_sweep, bool):
            raise ValueError("stop_sweep must be true / false")
        if not (self.edge_weight >= 1 and math.isfinite(self.edge_weight)):
            raise ValueError("edge_weight must be >= 1")
        if not (isinstance(self.yaws, int) and self.yaws >= 1) or not (isinstance(self.max_cells, int)
                                                                        and self.max_cells > 0):
            raise ValueError("yaws / max_cells must be positive integers")

    @classmethod
    def from_doc(cls, doc):
        """SearchConfig from a JSON object (any subset of the fields); q_table as {"0.6": 0.9, ...} or [[d, q]]."""
        if not isinstance(doc, dict):
            raise ValueError("search config must be a JSON object")
        names = {f.name for f in fields(cls)}
        bad = sorted(set(doc) - names)
        if bad:
            raise ValueError("unknown search config key(s): %s" % ", ".join(bad))
        kw = dict(doc)
        if "q_table" in kw:
            t = kw["q_table"]
            rows = [(float(k), float(v)) for k, v in t.items()] if isinstance(t, dict) else [tuple(r) for r in t]
            kw["q_table"] = tuple(sorted(rows))
        return cls(**kw)

    @classmethod
    def from_file(cls, path):
        with open(path, encoding="utf8") as f:
            return cls.from_doc(json.load(f))

    def record(self):
        out = {f.name: getattr(self, f.name) for f in fields(self)}
        out["q_table"] = [list(r) for r in self.q_table]
        return out


def dilate(mask, r):
    """Boolean disk dilation by r cells (numpy only)."""
    if r <= 0 or not mask.any():
        return mask.copy()
    h, w = mask.shape
    pad = np.pad(mask, r)
    out = np.zeros_like(mask)
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            if di * di + dj * dj <= r * r:
                out |= pad[r + di:r + di + h, r + dj:r + dj + w]
    return out


class BeliefSearch(object):
    """Belief over where the object is + greedy next-look selection. ``free_map``: FreeMap-like (cells, resolution,
    origin_x, origin_y, origin_yaw); ``nodes``: dense trajectory nodes [(x, y[, yaw])] in the same (map) frame.
    Raises ValueError when no usable universe can be built (the caller falls back to the MVP)."""

    def __init__(self, free_map, nodes, config=None, robot=None, clock=time.perf_counter):
        t0 = clock()
        self.cfg = config or SearchConfig()
        self.clock = clock
        nodes = [(float(n[0]), float(n[1])) for n in nodes]
        nodes = list(dict.fromkeys(n for n in nodes if all(math.isfinite(v) for v in n)))
        if not nodes:
            raise ValueError("no trajectory node")
        cells = np.asarray(free_map.cells)
        self.res = float(free_map.resolution)
        self.ox, self.oy = float(free_map.origin_x), float(free_map.origin_y)
        self.oyaw = float(getattr(free_map, "origin_yaw", 0.0))
        self.c, self.s = math.cos(self.oyaw), math.sin(self.oyaw)
        self.occ = cells >= 50
        known_free = (cells >= 0) & (cells < 50)
        h, w = cells.shape
        f = max(1, int(round(self.cfg.cell_m / self.res)))
        self.f, self.cell = f, f * self.res
        nxy = np.array(nodes)
        nu, nv = self._uv(nxy[:, 0], nxy[:, 1])
        margin = (self.cfg.max_range_m + 0.5) / self.res
        u0 = max(0, int(math.floor(nu.min() - margin)) // f * f)
        v0 = max(0, int(math.floor(nv.min() - margin)) // f * f)
        u1 = min(w, int(math.ceil(nu.max() + margin)))
        v1 = min(h, int(math.ceil(nv.max() + margin)))
        nj, ni = (u1 - u0) // f, (v1 - v0) // f
        if ni <= 0 or nj <= 0:
            raise ValueError("trajectory nodes are off the occupancy grid")
        if ni * nj > self.cfg.max_cells:
            raise ValueError("belief grid %d x %d over max_cells %d" % (ni, nj, self.cfg.max_cells))
        self.u0, self.v0, self.ni, self.nj = u0, v0, ni, nj

        def block(a):
            return a[v0:v0 + ni * f, u0:u0 + nj * f].reshape(ni, f, nj, f)
        occ_b = block(self.occ).any(axis=(1, 3))
        free_cnt = block(known_free).sum(axis=(1, 3))
        free = ~occ_b & (2 * free_cnt >= f * f)
        jj, ii = np.meshgrid(np.arange(nj), np.arange(ni))
        cu, cv = u0 + (jj + .5) * f, v0 + (ii + .5) * f
        gx, gy = self._xy(cu, cv)
        self.gx, self.gy = gx, gy
        # traversable (path cost): no occupied sub-cell, clearance, unknown only near the trajectory
        near_node = np.zeros((ni, nj), bool)
        for x, y in nodes:
            near_node |= (gx - x) ** 2 + (gy - y) ** 2 <= self.cfg.node_clear_m ** 2
        clear = ~dilate(occ_b, int(self.cfg.robot_radius_m / self.cell))
        self.trav = ~occ_b & ((clear & (free_cnt > 0)) | near_node)
        # the universe: free, within range of a node and visible from it
        fi = np.flatnonzero(free.ravel())
        fx, fy = gx.ravel()[fi], gy.ravel()[fi]
        R = self.cfg.max_range_m
        per_node, seen = [], np.zeros(fi.size, bool)
        for x, y in nodes:
            d = np.hypot(fx - x, fy - y)
            sel = np.flatnonzero(d <= R)
            if sel.size:
                sel = sel[~self._occluded(x, y, fx[sel], fy[sel])]
            seen[sel] = True
            per_node.append(sel)
        ui = np.flatnonzero(seen)
        if not ui.size:
            raise ValueError("no free cell visible from the trajectory")
        remap = -np.ones(fi.size, int)
        remap[ui] = np.arange(ui.size)
        self.flat = fi[ui]                                   # flat coarse index of each universe cell
        self.ux, self.uy = fx[ui], fy[ui]
        edge = dilate(occ_b, int(round(self.cfg.edge_m / self.cell))).ravel()[self.flat]
        self.edge = edge
        self.nodes = nodes
        self.node_cells = []
        for (x, y), sel in zip(nodes, per_node):
            idx = remap[sel]
            d = np.hypot(self.ux[idx] - x, self.uy[idx] - y)
            self.node_cells.append((idx, d, np.arctan2(self.uy[idx] - y, self.ux[idx] - x)))
        self.yaw_set = [_wrap1(2 * math.pi * k / self.cfg.yaws) for k in range(self.cfg.yaws)]
        qd = np.array([r[0] for r in self.cfg.q_table])
        qv = np.array([r[1] for r in self.cfg.q_table])
        self._qd, self._qv = qd, qv
        self.build_ms = (clock() - t0) * 1e3
        self.robot = tuple(robot) if robot is not None else None
        self.K = self.image_size = self.T_base_camera = None     # 10-28 camera model (set_camera / observe)
        self.reset()

    # ---- geometry -------------------------------------------------------------------------------------------
    def _uv(self, x, y):
        dx, dy = np.asarray(x, float) - self.ox, np.asarray(y, float) - self.oy
        return (self.c * dx + self.s * dy) / self.res, (-self.s * dx + self.c * dy) / self.res

    def _xy(self, u, v):
        u, v = np.asarray(u, float) * self.res, np.asarray(v, float) * self.res
        return self.ox + self.c * u - self.s * v, self.oy + self.s * u + self.c * v

    def _occluded(self, x, y, tx, ty):
        """True where the segment (x, y) -> (tx, ty) crosses an occupied map cell (the target cell's last half
        cell and the first 0.1 m excluded; off the grid blocks)."""
        dx, dy = tx - x, ty - y
        d = np.hypot(dx, dy)
        step = .75 * self.res
        n = int(math.ceil(max(float(d.max()) if d.size else 0., step) / step))
        s = .1 + step * np.arange(n)                       # (n,)
        ok = s[None, :] < (d - .5 * self.cell)[:, None]   # (m, n)
        with np.errstate(invalid="ignore", divide="ignore"):
            ux, uy = np.where(d > 0, dx / d, 0.), np.where(d > 0, dy / d, 0.)
        px = x + ux[:, None] * s[None, :]
        py = y + uy[:, None] * s[None, :]
        u, v = self._uv(px, py)
        u, v = np.floor(u).astype(int), np.floor(v).astype(int)
        h, w = self.occ.shape
        inside = (u >= 0) & (v >= 0) & (u < w) & (v < h)
        hit = np.ones(u.shape, bool)
        hit[inside] = self.occ[v[inside], u[inside]]
        return (hit & ok).any(axis=1)

    # ---- 10-28 camera model: whole object in the image ------------------------------------------------------
    def set_camera(self, K=None, image_size=None, T_base_camera=None):
        """Camera model for the frustum test: K (3 x 3), image_size (width, height) px, T_base_camera (4 x 4,
        base_link -> optical, z forward). Arguments left None keep their current value; invalid ones raise."""
        if K is not None:
            K = np.asarray(K, float).reshape(3, 3)
            if not np.all(np.isfinite(K)) or K[0, 0] <= 0 or K[1, 1] <= 0:
                raise ValueError("bad K")
            self.K = K
        if image_size is not None:
            w, h = (int(v) for v in image_size)
            if w <= 0 or h <= 0:
                raise ValueError("bad image size")
            self.image_size = (w, h)
        if T_base_camera is not None:
            T = np.asarray(T_base_camera, float).reshape(4, 4)
            if not np.all(np.isfinite(T)):
                raise ValueError("bad T_base_camera")
            self.T_base_camera = T

    def has_camera(self):
        return self.K is not None and self.image_size is not None

    def camera_pose(self, x, y, yaw):
        """T_world_camera of the base at (x, y, yaw) (floor z = 0) through T_base_camera (None without it)."""
        if self.T_base_camera is None:
            return None
        c, s = math.cos(yaw), math.sin(yaw)
        T_wb = np.array([[c, -s, 0., x], [s, c, 0., y], [0., 0., 1., 0.], [0., 0., 0., 1.]])
        return T_wb @ self.T_base_camera

    def in_frame(self, T_world_camera, idx):
        """Boolean per universe cell ``idx``: a ``object_radius_m`` x ``object_height_m`` object standing on the
        cell is wholly in the image -- bottom near / left / right edge points (z = 0) and the top far edge point
        (z = object_height_m), all four in front of the lens and inside the frame. ``T_world_camera`` 4 x 4 ->
        (len(idx),); a stack (Y, 4, 4) -> (Y, len(idx))."""
        T = np.asarray(T_world_camera, float)
        one = T.ndim == 2
        T = T.reshape(-1, 4, 4)
        Rt = np.transpose(T[:, :3, :3], (0, 2, 1))             # world -> camera rotation (rigid inverse)
        tc = -np.einsum("yij,yj->yi", Rt, T[:, :3, 3])
        px, py = self.ux[idx][None, :], self.uy[idx][None, :]
        dx, dy = px - T[:, 0, 3][:, None], py - T[:, 1, 3][:, None]
        n = np.hypot(dx, dy)
        n = np.where(n > 1e-9, n, 1.)
        ex, ey = dx / n, dy / n                                 # camera -> cell, ground plane (Y, m)
        r, H = self.cfg.object_radius_m, self.cfg.object_height_m
        pts = ((px - r * ex, py - r * ey, 0.),                  # bottom near edge (the blind-zone bound)
               (px - r * ey, py + r * ex, 0.),                  # bottom left / right edges (the side bounds)
               (px + r * ey, py - r * ex, 0.),
               (px + r * ex, py + r * ey, H))                   # top far edge (the top bound)
        w, h = self.image_size
        K = self.K
        ok = np.ones(ex.shape, bool)
        for X, Y, Z in pts:
            c = [Rt[:, k, 0][:, None] * X + Rt[:, k, 1][:, None] * Y + Rt[:, k, 2][:, None] * Z + tc[:, k][:, None]
                 for k in range(3)]
            front = c[2] > .05
            zs = np.where(front, c[2], 1.)
            u = (K[0, 0] * c[0] + K[0, 1] * c[1]) / zs + K[0, 2]
            v = K[1, 1] * c[1] / zs + K[1, 2]
            ok &= front & (u >= 0) & (u < w) & (v >= 0) & (v < h)
        return ok[0] if one else ok

    def q(self, d):
        d = np.asarray(d, float)
        out = np.interp(d, self._qd, self._qv)
        return np.where(d > self._qd[-1], 0., out)

    # ---- belief ---------------------------------------------------------------------------------------------
    def reset(self):
        """A new task: no prior yet, every cell unobserved."""
        n = self.ux.size
        self.miss = np.ones(n)
        self.look_x = np.full(n, np.nan)
        self.look_y = np.full(n, np.nan)
        self.look_q = np.zeros(n)
        self.prior = None
        self.target_xy = None
        self.visited, self.failed_nodes = set(), set()
        self.goals = []
        self.stop = None
        self.views = self.views_skipped = 0
        self.update_ms = []
        self._last_t = -math.inf

    def begin(self, target_xy=None):
        """Build the prior (remembered position or uniform); views seen since reset() already count. 10-29: a
        remembered target's prior is (1 - uniform_floor) x the memory prior + uniform_floor x the uniform one."""
        cfg = self.cfg
        w = np.where(self.edge, cfg.edge_weight, 1.0)
        uniform = w / w.sum()
        if target_xy is not None and all(math.isfinite(float(v)) for v in target_xy[:2]):
            tx, ty = float(target_xy[0]), float(target_xy[1])
            d2 = (self.ux - tx) ** 2 + (self.uy - ty) ** 2
            main = np.exp(-d2 / (2 * cfg.prior_sigma_m ** 2)) * w
            near = np.exp(-d2 / (2 * cfg.near_sigma_m ** 2))
            main = main / main.sum() if main.sum() > 1e-12 else np.full(main.size, 1. / main.size)
            if cfg.near_mass > 0 and near.sum() > 1e-6:
                prior = (1 - cfg.near_mass) * main + cfg.near_mass * near / near.sum()
            else:
                prior = main
            prior = (1 - cfg.uniform_floor) * prior / prior.sum() + cfg.uniform_floor * uniform
            self.target_xy = (tx, ty)
        else:
            prior = uniform
            self.target_xy = None
        self.prior = prior / prior.sum()

    def began(self):
        return self.prior is not None

    def mass_left(self):
        p = self.prior if self.prior is not None else np.full(self.ux.size, 1. / self.ux.size)
        return float(np.sum(p * self.miss))

    def reachable(self):
        """10-29: boolean per universe cell -- some candidate view (any node, any heading, through the camera
        model when set) would see it at q >= 0.5. Cached per camera offset / model."""
        key = (self.has_camera() and self.T_base_camera is not None,
               None if self.T_base_camera is None else np.round(self.T_base_camera, 3).tobytes(),
               None if self.K is None else (self.K.tobytes(), self.image_size))
        cache = getattr(self, "_reach", None)
        if cache is not None and cache[0] == key:
            return cache[1]
        out = np.zeros(self.ux.size, bool)
        for n in range(len(self.nodes)):
            idx, q, _cx, _cy = self._view_q(n)
            if idx.size:
                out[idx[(q >= .5).any(axis=0)]] = True
        self._reach = (key, out)
        return out

    def unseen_mass(self):
        """10-29: prior mass on reachable cells not looked at yet (miss > 0.5)."""
        p = self.prior if self.prior is not None else np.full(self.ux.size, 1. / self.ux.size)
        return float(np.sum(p[self.reachable() & (self.miss > .5)]))

    def _after(self, idx, d, x, y):
        """New miss of cells ``idx`` (at distance d) for a view from (x, y), and its q / same-look flags."""
        q = self.q(d)
        lx, ly = self.look_x[idx], self.look_y[idx]
        same = np.isfinite(lx) & ((lx - x) ** 2 + (ly - y) ** 2 < self.cfg.look_move_m ** 2)
        lq = self.look_q[idx]
        factor = np.where(same, np.where(q > lq, (1 - q) / (1 - lq), 1.), 1 - q)
        new = np.maximum(self.miss[idx] * factor, np.minimum(self.cfg.miss_floor, self.miss[idx]))
        return new, q, same

    def apply_view(self, x, y, yaw, T_world_camera=None):
        """One view from camera position (x, y) facing ``yaw`` (map frame); returns the cells updated. 10-28: with
        ``T_world_camera`` and a camera model (K, image size), the cells are those passing ``in_frame`` (x, y =
        the camera position) instead of the horizontal ``fov_deg`` wedge."""
        t0 = self.clock()
        dx, dy = self.ux - x, self.uy - y
        d = np.hypot(dx, dy)
        if T_world_camera is not None and self.has_camera():
            sel = np.flatnonzero(d <= self.cfg.max_range_m)
            if sel.size:
                sel = sel[self.in_frame(T_world_camera, sel)]
        else:
            half = math.radians(self.cfg.fov_deg) / 2
            sel = np.flatnonzero((d <= self.cfg.max_range_m) & (np.abs(_wrap(np.arctan2(dy, dx) - yaw)) <= half))
        if sel.size:
            sel = sel[~self._occluded(x, y, self.ux[sel], self.uy[sel])]
        if sel.size:
            new, q, same = self._after(sel, d[sel], x, y)
            self.miss[sel] = new
            ind = ~same
            self.look_x[sel[ind]], self.look_y[sel[ind]], self.look_q[sel[ind]] = x, y, q[ind]
            up = same & (q > self.look_q[sel])
            self.look_q[sel[up]] = q[up]
        self.views += 1
        self.update_ms.append((self.clock() - t0) * 1e3)
        del self.update_ms[:-200]
        return int(sel.size)

    def apply_base_view(self, x, y, yaw):
        """A view from the BASE pose (x, y, yaw): through the camera model when there is one (T_base_camera, K,
        image size), else the old camera-at-the-base wedge."""
        T = self.camera_pose(x, y, yaw) if self.has_camera() else None
        if T is None:
            return self.apply_view(x, y, yaw)
        return self.apply_view(float(T[0, 3]), float(T[1, 3]), math.atan2(T[1, 2], T[0, 2]), T_world_camera=T)

    def observe(self, T_world_camera, t, base_pose=None, trusted=True, K=None, image_size=None,
                T_base_camera=None):
        """A processed frame: camera 4x4 (optical z forward) at time t. ``base_pose`` (x, y, yaw) keeps the
        robot's position for path costs. Untrusted poses and frames within 1/update_hz of the last update are
        skipped. True when the belief was updated. 10-28: ``K`` / ``image_size`` / ``T_base_camera`` (the frame's
        intrinsics and size, the live mount) refresh the camera model; with K and size the view is the frustum."""
        if K is not None or image_size is not None or T_base_camera is not None:
            self.set_camera(K, image_size, T_base_camera)
        if base_pose is not None:
            self.robot = tuple(float(v) for v in base_pose[:3])
        if not trusted:
            self.views_skipped += 1
            return False
        if t - self._last_t < 1. / self.cfg.update_hz:
            return False
        T = np.asarray(T_world_camera, float)
        fx, fy = T[0, 2], T[1, 2]
        if not np.all(np.isfinite(T)) or math.hypot(fx, fy) < .3:   # camera looking at the floor/ceiling
            self.views_skipped += 1
            return False
        self._last_t = t
        self.apply_view(float(T[0, 3]), float(T[1, 3]), math.atan2(fy, fx), T_world_camera=T)
        return True

    # ---- selection ------------------------------------------------------------------------------------------
    def _cell_of(self, x, y, radius_m=.5):
        """Nearest traversable coarse cell (i, j) to (x, y) within radius_m, or None."""
        u, v = self._uv(x, y)
        j, i = int(math.floor((u - self.u0) / self.f)), int(math.floor((v - self.v0) / self.f))
        if 0 <= i < self.ni and 0 <= j < self.nj and self.trav[i, j]:
            return i, j
        r = int(math.ceil(radius_m / self.cell))
        best = None
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                a, b = i + di, j + dj
                if 0 <= a < self.ni and 0 <= b < self.nj and self.trav[a, b] and di * di + dj * dj <= r * r:
                    k = di * di + dj * dj
                    if best is None or k < best[0]:
                        best = (k, a, b)
        return None if best is None else (best[1], best[2])

    def path_lengths(self, start_xy):
        """Path length (m) from start_xy to every node (inf = unreachable); 8-connected Dijkstra."""
        out = [math.inf] * len(self.nodes)
        src = self._cell_of(*start_xy)
        if src is None:
            return out
        targets = {}
        for n, (x, y) in enumerate(self.nodes):
            c = self._cell_of(x, y)
            if c is not None:
                targets.setdefault(c, []).append(n)
        left = len(targets)
        dist = {src: 0.}
        heap = [(0., src)]
        trav, ni, nj, cell = self.trav, self.ni, self.nj, self.cell
        steps = [(di, dj, cell * math.hypot(di, dj)) for di in (-1, 0, 1) for dj in (-1, 0, 1) if di or dj]
        done = set()
        while heap and left:
            d, (i, j) = heapq.heappop(heap)
            if (i, j) in done:
                continue
            done.add((i, j))
            if (i, j) in targets:
                for n in targets[(i, j)]:
                    out[n] = d
                left -= 1
            for di, dj, c in steps:
                a, b = i + di, j + dj
                if 0 <= a < ni and 0 <= b < nj and trav[a, b]:
                    nd = d + c
                    if nd < dist.get((a, b), math.inf):
                        dist[(a, b)] = nd
                        heapq.heappush(heap, (nd, (a, b)))
        return out

    def _view_q(self, n):
        """(idx, q (Y, m), cx, cy) for node n: the universe cells it could see, the detection rate each heading's view
        gives them (0 = out of that view) and the camera position per heading (Y, 1) -- with a camera model the
        lens ahead of the base and the whole-object-in-frame test, else the base and the ``fov_deg`` wedge."""
        x, y = self.nodes[n]
        if self.has_camera() and self.T_base_camera is not None:
            idx = self._node_cells_wide()[n]
            Ts = np.stack([self.camera_pose(x, y, yaw) for yaw in self.yaw_set])
            cx, cy = Ts[:, 0, 3][:, None], Ts[:, 1, 3][:, None]
            if not idx.size:
                return idx, np.zeros((len(self.yaw_set), 0)), cx, cy
            dc = np.hypot(self.ux[idx][None, :] - cx, self.uy[idx][None, :] - cy)       # (Y, m)
            m = (dc <= self.cfg.max_range_m) & self.in_frame(Ts, idx)
            return idx, np.where(m, self.q(dc), 0.), cx, cy
        idx, d, bearing = self.node_cells[n]
        half = math.radians(self.cfg.fov_deg) / 2
        inview = np.abs(_wrap(bearing[None, :] - np.array(self.yaw_set)[:, None])) <= half      # (yaws, m)
        Y = len(self.yaw_set)
        return idx, np.where(inview, self.q(d)[None, :], 0.), np.full((Y, 1), x), np.full((Y, 1), y)

    def gains(self, new_area=False):
        """(nodes x yaws) mass each view would remove (the same-look rule of _after, broadcast over the headings;
        10-28: with a camera model from the camera at each heading's base pose, whole object in the image).
        ``new_area``: also (nodes x yaws) m^2 the view would newly see (q >= 0.5 on cells with miss > 0.5)."""
        out = np.zeros((len(self.nodes), len(self.yaw_set)))
        area = np.zeros_like(out)
        prior = self.prior if self.prior is not None else np.full(self.ux.size, 1. / self.ux.size)
        for n in range(len(self.nodes)):
            idx, q, cx, cy = self._view_q(n)
            if not idx.size:
                continue
            lx, ly, lq, miss = self.look_x[idx], self.look_y[idx], self.look_q[idx], self.miss[idx]
            same = np.isfinite(lx)[None, :] & ((lx[None, :] - cx) ** 2 + (ly[None, :] - cy) ** 2
                                               < self.cfg.look_move_m ** 2)
            factor = np.where(same, np.where(q > lq, (1 - q) / (1 - lq), 1.), 1 - q)
            new = np.maximum(miss * factor, np.minimum(self.cfg.miss_floor, miss))
            out[n] = (np.where(q > 0, miss - new, 0.)) @ prior[idx]
            if new_area:
                area[n] = ((q >= .5) & (miss > .5)[None, :]).sum(axis=1) * self.cell ** 2
        return (out, area) if new_area else out

    def _node_cells_wide(self):
        """Per node: universe cells within max_range_m + the lens's horizontal offset from the base (the camera
        sits ahead of the node), not occluded from the node; cached per offset."""
        off = round(float(np.hypot(*self.T_base_camera[:2, 3])), 3)
        cache = getattr(self, "_wide", None)
        if cache is not None and cache[0] == off:
            return cache[1]
        R = self.cfg.max_range_m + off + 1e-6
        out = []
        for x, y in self.nodes:
            d = np.hypot(self.ux - x, self.uy - y)
            sel = np.flatnonzero(d <= R)
            if sel.size:
                sel = sel[~self._occluded(x, y, self.ux[sel], self.uy[sel])]
            out.append(sel)
        self._wide = (off, out)
        return out

    def next_goal(self, time_left_s=None):
        """(PoseGoal, info) of the best next look, or (None, info) with info["stop"] the reason. 10-29: when the
        stop rule fires, a stop sweep look (info["sweep"] = the stop it postponed) comes first while one is left;
        ``time_left_s`` (the task's remaining time, None = unknown) leaves out sweep looks that could not be reached
        and looked at in it (``sweep_eta``), so the task ends search_exhausted rather than on its time limit."""
        from host.task_control import PoseGoal
        t0 = self.clock()
        if self.prior is None:
            self.begin(None)
        mass = self.mass_left()
        info = {"mass_left": round(mass, 4)}
        unseen = self.unseen_mass()
        info["unseen"] = round(unseen, 4)
        mass_stop = mass < self.cfg.stop_mass and (unseen < self.cfg.stop_unseen or self.cfg.stop_unseen >= 1)
        if mass_stop and not self.cfg.stop_sweep:
            self.stop = info["stop"] = "mass_below_stop"
            info["select_ms"] = round((self.clock() - t0) * 1e3, 2)
            return None, info
        if self.robot is not None:
            rx, ry, ryaw = self.robot[0], self.robot[1], (self.robot[2] if len(self.robot) > 2 else None)
        else:
            ref = self.target_xy or self.nodes[0]
            rx, ry = min(self.nodes, key=lambda p: (p[0] - ref[0]) ** 2 + (p[1] - ref[1]) ** 2)
            ryaw = None
        paths = self.path_lengths((rx, ry))
        gains, new_m2 = self.gains(new_area=True)
        best, n_cand = None, 0
        sweep, n_sweep, n_late = None, 0, 0
        for n, (x, y) in enumerate(self.nodes):
            if n in self.failed_nodes or not math.isfinite(paths[n]):
                continue
            heading = math.atan2(y - ry, x - rx) if paths[n] > .2 else ryaw
            for k, yaw in enumerate(self.yaw_set):
                if (n, k) in self.visited:
                    continue
                turn = abs(_wrap1(yaw - heading)) if heading is not None else 0.
                cost = paths[n] + self.cfg.turn_cost_m_per_rad * turn + self.cfg.step_cost_m
                g = gains[n, k]
                if self.cfg.stop_sweep and new_m2[n, k] >= max(self.cfg.sweep_min_m2, 1e-9):
                    if time_left_s is not None and sweep_eta(paths[n], turn) > time_left_s:
                        n_late += 1
                    else:
                        n_sweep += 1
                        s = new_m2[n, k] / cost
                        if sweep is None or s > sweep[0] + 1e-12:
                            sweep = (s, n, k, g, paths[n], turn)
                if mass_stop or (g < self.cfg.min_gain and not
                                 (new_m2[n, k] >= self.cfg.explore_m2 > 0 and g > 0)):
                    continue
                n_cand += 1
                score = g / cost
                if best is None or score > best[0] + 1e-12:
                    best = (score, n, k, g, paths[n], turn)
        info["candidates"] = n_cand
        stop = "mass_below_stop" if mass_stop else ("no_candidate" if best is None else None)
        if stop is not None:
            info["sweep_left"] = n_sweep
            if n_late:
                info["sweep_no_time"] = n_late
            if sweep is None:
                self.stop = info["stop"] = stop
                info["select_ms"] = round((self.clock() - t0) * 1e3, 2)
                return None, info
            best = sweep
            info["sweep"] = stop
        info["select_ms"] = round((self.clock() - t0) * 1e3, 2)
        score, n, k, g, path, turn = best
        self.visited.add((n, k))
        x, y = self.nodes[n]
        yaw = self.yaw_set[k]
        info.update(n=len(self.goals) + 1, node=n, x=round(x, 3), y=round(y, 3), yaw=round(yaw, 4),
                    score=round(float(score), 5), gain=round(float(g), 4), path_m=round(path, 2),
                    turn_rad=round(turn, 2))
        if "sweep" in info:
            info["new_m2"] = round(float(new_m2[n, k]), 3)
        self.goals.append(info)
        return PoseGoal(x, y, yaw), info

    def mark_failed(self, pose):
        """Navigation to ``pose`` failed after its retries: its node is not offered again."""
        for n, (x, y) in enumerate(self.nodes):
            if abs(x - pose.x) < 1e-6 and abs(y - pose.y) < 1e-6:
                self.failed_nodes.add(n)

    # ---- records --------------------------------------------------------------------------------------------
    def summary(self, compact=False):
        """Task-row record; ``compact``: the goal count and the last goal only (every row of a running task)."""
        ms = self.update_ms
        out = {"mode": "prob", "mass_left": round(self.mass_left(), 4), "began": self.prior is not None,
               "target_xy": None if self.target_xy is None else [round(v, 3) for v in self.target_xy],
               "cells": int(self.ux.size), "nodes": len(self.nodes), "views": self.views,
               "views_skipped": self.views_skipped, "failed_nodes": sorted(self.failed_nodes),
               "goal_count": len(self.goals), "stop": self.stop, "build_ms": round(self.build_ms, 1),
               "update_ms_max": round(max(ms), 2) if ms else None,
               "update_ms_mean": round(sum(ms) / len(ms), 2) if ms else None}
        if compact:
            out["last_goal"] = self.goals[-1] if self.goals else None
        else:
            out["goals"] = list(self.goals)
        return out

    def belief_grid(self, normalise=True):
        """(ni x nj) float grid of prior x miss (NaN outside the universe), coarse-grid aligned."""
        g = np.full(self.ni * self.nj, np.nan)
        p = self.prior if self.prior is not None else np.full(self.ux.size, 1. / self.ux.size)
        g[self.flat] = p * self.miss
        g = g.reshape(self.ni, self.nj)
        if normalise and np.nanmax(g) > 0:
            g = g / np.nanmax(g)
        return g

    def belief_cells(self, step_m=0.2, relative="current"):
        """(meta, int8 array ni x nj) of the down-sampled belief, 0..100 per block (its max cell), -1 outside the
        universe; ``relative`` "current" (the current max = 100) or "prior" (10-26 display: the prior's max = 100,
        so a looked-at area fades and the rest keeps its colour). meta: frame, resolution, width, height, origin."""
        k = max(1, int(round(step_m / self.cell)))
        if relative == "prior":
            g = self.belief_grid(normalise=False)
            p = self.prior if self.prior is not None else np.full(self.ux.size, 1. / self.ux.size)
            top = float(np.max(p)) if p.size else 0.
            if top > 0:
                g = g / top
        else:
            g = self.belief_grid()
        ni, nj = -(-self.ni // k), -(-self.nj // k)
        pad = np.full((ni * k, nj * k), np.nan)
        pad[:self.ni, :self.nj] = g
        blk = pad.reshape(ni, k, nj, k)
        with np.errstate(invalid="ignore"):
            valid = np.isfinite(blk).any(axis=(1, 3))
            m = np.where(valid, np.nanmax(np.where(np.isfinite(blk), blk, -1), axis=(1, 3)), -1)
        data = np.where(valid, np.round(np.clip(m, 0, 1) * 100), -1).astype(np.int8)
        ox, oy = self._xy(self.u0, self.v0)
        return ({"frame": "map", "resolution": round(self.cell * k, 4), "width": int(nj), "height": int(ni),
                 "origin": [round(float(ox), 4), round(float(oy), 4), round(self.oyaw, 6)]}, data)

    def belief_message(self, step_m=0.2):
        """Down-sampled belief for the UI (/fpga/search/belief, JSON): row-major ``data`` 0..100 (relative to the
        max; -1 outside the universe), cell (row 0, col 0) at ``origin`` [x, y, yaw] (its lower-left corner, the
        nav_msgs/OccupancyGrid convention, map frame)."""
        meta, data = self.belief_cells(step_m)
        return dict(meta, mass_left=round(self.mass_left(), 4), goals=[[g["x"], g["y"], g["yaw"]] for g in self.goals],
                    data=data.astype(int).ravel().tolist())

    def preview(self, robot=None, max_goals=12):
        """Expected look order if the object is never seen: each chosen view applied at its node (through the camera
        model when set, else camera = base).
        Leaves the belief as after those views (call reset() + begin() to start over)."""
        if robot is not None:
            self.robot = tuple(robot)
        out = []
        for _ in range(max_goals):
            pose, info = self.next_goal()
            if pose is None:
                out.append({"stop": info.get("stop"), "mass_left": info["mass_left"]})
                break
            self.apply_base_view(pose.x, pose.y, pose.yaw)
            row = dict(info, mass_after=round(self.mass_left(), 4))
            row["cum_prob"] = round(1 - row["mass_after"], 4)
            out.append(row)
            self.robot = (pose.x, pose.y, pose.yaw)
        return out
