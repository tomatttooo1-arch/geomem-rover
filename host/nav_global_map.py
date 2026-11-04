"""Nav2's global map while mapping (2026-10-29, owner: find without pressing 结束建图; car_up CAR_NAV_LIVE_GLOBAL).

Before: in the in-stack mapping run (car_inner MAPPING=1) Nav2's global costmap (static_layer + inflation_layer) read
map_server's /map, the 20 m all-FREE blank start map, until the stop's export was loaded: every global path was a
straight line and walls / furniture were seen only by the 4 m x 4 m rolling local costmap.

Now (host/ros_nav_global_map.py, started by car_inner before Nav2; Nav2's global static_layer reads OUT_TOPIC instead
of /map through FPGA_NAV_GLOBAL_MAP_TOPIC): this module builds the map that layer gets:
  base   map_server's /map as it is (the blank start map while mapping, the export after the stop, the start map again
         after a map reset) -- before map_server is up, the same start map read from its yaml
  live   while mapping_ctl reports state "mapping" (fresh status, same session) and rtabmap's grid
         (/fpga/rtabmap/map, frame "map": with live alignment map -> odom follows rtabmap's correction, so its frame
         is Nav2's) has arrived: base + the grid's OCCUPIED cells, written as 100 into the base's own cells (size,
         resolution and origin stay the base's -- the costmap is never resized and goals keep their bounds).
         UNKNOWN and FREE live cells leave the base cell as it is (the blank map: FREE), so a plan may cross what
         rtabmap has not seen yet -- with the car's strict NavFn (allow_unknown false) an unknown cell would block it.
Not written (on the side of driving, as the owner asked: success rate first):
  * an occupied cell with no occupied 8-neighbour (MIN_NEIGHBOURS): single-cell depth / ground-fit speckles; a wall,
    a box, a sofa are runs of cells (a lone chair leg is left to the rolling local costmap, as before)
  * an occupied cell within SWEPT_CLEAR_M of a place the car's base has been (wheel odometry, moved into the map
    frame with the alignment's per-stamp correction, host/map_align.Corrections): the car drove there, so it is
    passable; 0.35 m = Nav2 robot_radius 0.30 (the inscribed radius: closer lethal cells make the centre cell
    untraversable for NavFn) + one 0.05 m cell -- the centre line the car drove stays plannable however close the
    noise is. The current pose is one of these places, so the start cell is never blocked.
Falls back to the base (logged, never stops anything): no live grid yet, mapping_ctl status stale / not mapping,
a grid in another frame, and -- PLANNER_FALLBACK_S after Nav2's planner reported a failed plan while the live
cells were in the map (the next plan then is the old straight one: never worse than before this module).

The local layer's obstacles too (2026-10-29 nav-turn-start, car_up CAR_NAV_GLOBAL_ROLLING, default on): in every mode
but the planner fallback, the rolling local layer's current OCCUPIED cells (/fpga/rolling_map/grid, odom frame, the
very grid Nav2's local costmap and DWB drive on) are written as LETHAL too, moved into the map frame with the live
map <- odom transform. Before, an obstacle the local layer had and the global map did not (rtabmap not yet / never
marking it: a lone cell, a place the car drove before, anything after the stop's export, the blank map before the
first live grid) got a global path straight through it or grazing it at the inscribed radius; DWB, which refuses
the body's outline in it, could not follow, NavFn re-planned the same path every second, and the car turned on the
spot until the goal failed (host sim: a straight path over a wall's end, the car stopped at its inscribed edge).
Each rolling cell is sampled 2 x 2, so off the map's lattice it covers every map cell under it (at most one cell
more than the local layer: the global path keeps at least the local layer's clearance). Only the window's current
cells: what leaves the 4 m window or the local layer clears leaves the map at the next tick. Not touched by the
swept clearing (the local layer refuses such a cell wherever the car drove before). A failed plan while they are in
the map falls back to the base like the live cells.

No ROS. Python 3.8 compatible.
"""
import math

import numpy as np

from host import map_align

OUT_TOPIC = "/fpga/nav/global_map"
STATUS_OUT_TOPIC = "/fpga/nav/global_map/status"
LIVE_TOPIC = "/fpga/rtabmap/map"
ROLLING_TOPIC = "/fpga/rolling_map/grid"          # host/ros_rolling_nav.py: Nav2's local costmap layer (odom frame)
ROLLING_SUBSAMPLE = 2                            # per axis, per rolling cell (module doc)
ROLLING_TF_EPS_M, ROLLING_TF_EPS_RAD = 0.005, 0.002   # a map <- odom change below this is not a new map
BASE_TOPIC = "/map"
MAPPING_STATUS_TOPIC = "/fpga/mapping/status"     # = tools/car_run/mapping_ctl.STATUS_TOPIC
FRAME = "map"
STATUS_MAX_AGE_S = 5.0        # mapping_ctl re-sends every 1 s; the runner's own gate uses 3 s -- 5 s rides out hiccups
OCCUPIED_MIN = 65             # rtabmap writes 0 / 100 / -1; >= map_server's occupied_thresh 0.65
MIN_NEIGHBOURS = 1
SWEPT_CLEAR_M = 0.35
SWEPT_STEP_M = 0.05           # a new swept point every 5 cm of travel
SWEPT_MAX = 40000             # 2 km of travel at 5 cm; oldest dropped beyond
PLANNER_FALLBACK_S = 20.0
LETHAL = 100


class Grid(object):
    """An OccupancyGrid body: data (height x width int8, row 0 at the origin), resolution, origin x / y / yaw, frame."""

    def __init__(self, data, resolution, origin_x, origin_y, origin_yaw=0.0, frame=FRAME):
        data = np.asarray(data, dtype=np.int8)
        if data.ndim != 2 or not data.size:
            raise ValueError("grid data must be a non-empty 2-D array")
        if not (resolution > 0 and all(math.isfinite(v) for v in (resolution, origin_x, origin_y, origin_yaw))):
            raise ValueError("bad grid resolution / origin")
        self.data, self.resolution = data, float(resolution)
        self.origin_x, self.origin_y, self.origin_yaw = float(origin_x), float(origin_y), float(origin_yaw)
        self.frame = frame

    @property
    def shape(self):
        return self.data.shape

    def same_geometry(self, other):
        return (other is not None and self.data.shape == other.data.shape and self.resolution == other.resolution
                and (self.origin_x, self.origin_y, self.origin_yaw) == (other.origin_x, other.origin_y,
                                                                         other.origin_yaw)
                and self.frame == other.frame)

    def cells_to_world(self, rows, cols, k=1):
        """World (x, y) of the k x k sub-samples of cells (rows, cols) -> two flat arrays."""
        offs = (np.arange(k) + 0.5) / k
        u = (cols[:, None, None] + offs[None, :, None]) * self.resolution
        v = (rows[:, None, None] + offs[None, None, :]) * self.resolution
        u, v = np.broadcast_arrays(u, v)
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        x = self.origin_x + c * u - s * v
        y = self.origin_y + s * u + c * v
        return x.ravel(), y.ravel()

    def world_to_cells(self, x, y):
        """(rows, cols, inside mask) of world points."""
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        dx, dy = np.asarray(x, dtype=float) - self.origin_x, np.asarray(y, dtype=float) - self.origin_y
        cols = np.floor((c * dx + s * dy) / self.resolution).astype(np.int64)
        rows = np.floor((-s * dx + c * dy) / self.resolution).astype(np.int64)
        h, w = self.data.shape
        inside = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
        return rows, cols, inside


def occupied_neighbours(occ):
    """Per cell: how many of its 8 neighbours are occupied (bool array in, int array out)."""
    p = np.pad(occ.astype(np.int16), 1)
    h, w = occ.shape
    n = np.zeros((h, w), dtype=np.int16)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr or dc:
                n += p[1 + dr:1 + dr + h, 1 + dc:1 + dc + w]
    return n


def disk_offsets(radius_cells):
    r = int(math.ceil(radius_cells))
    return [(dr, dc) for dr in range(-r, r + 1) for dc in range(-r, r + 1) if dr * dr + dc * dc <= radius_cells ** 2]


def swept_mask(base, swept_xy, clear_m, rows_window=None):
    """Bool mask on base's cells: within clear_m of any point of swept_xy (N x 2 map-frame array). rows_window =
    (r0, r1, c0, c1) restricts the work to that window (cells outside stay False)."""
    h, w = base.shape
    mask = np.zeros((h, w), dtype=bool)
    if swept_xy is None or not len(swept_xy) or clear_m <= 0:
        return mask
    r0, r1, c0, c1 = rows_window if rows_window is not None else (0, h, 0, w)
    rows, cols, inside = base.world_to_cells(swept_xy[:, 0], swept_xy[:, 1])
    rad = clear_m / base.resolution
    pad = int(math.ceil(rad))
    keep = inside & (rows >= r0 - pad) & (rows < r1 + pad) & (cols >= c0 - pad) & (cols < c1 + pad)
    if not np.any(keep):
        return mask
    seed = np.zeros((h, w), dtype=bool)
    seed[rows[keep], cols[keep]] = True
    win = seed[max(r0 - pad, 0):min(r1 + pad, h), max(c0 - pad, 0):min(c1 + pad, w)]
    out = np.zeros_like(win)
    wh, ww = win.shape
    for dr, dc in disk_offsets(rad):
        src = win[max(-dr, 0):wh - max(dr, 0), max(-dc, 0):ww - max(dc, 0)]
        out[max(dr, 0):wh - max(-dr, 0), max(dc, 0):ww - max(-dc, 0)] |= src
    mask[max(r0 - pad, 0):min(r1 + pad, h), max(c0 - pad, 0):min(c1 + pad, w)] = out
    full = np.zeros((h, w), dtype=bool)
    full[r0:r1, c0:c1] = mask[r0:r1, c0:c1]
    return full


def compose(base, live, swept_xy=None, clear_m=SWEPT_CLEAR_M, min_neighbours=MIN_NEIGHBOURS):
    """(data, stats): base.data with live's occupied cells written as LETHAL (module doc). Never raises for a grid
    that does not overlap the base (nothing written then)."""
    out = base.data.copy()
    occ = live.data >= OCCUPIED_MIN
    stats = {"live_occupied": int(occ.sum())}
    if min_neighbours > 0 and stats["live_occupied"]:
        keep = occ & (occupied_neighbours(occ) >= min_neighbours)
    else:
        keep = occ
    stats["speckles_dropped"] = int(stats["live_occupied"] - keep.sum())
    rows, cols = np.nonzero(keep)
    k = max(1, int(math.ceil(live.resolution / base.resolution - 1e-9)))
    x, y = live.cells_to_world(rows, cols, k)
    br, bc, inside = base.world_to_cells(x, y)
    stats["outside_base"] = int((~inside).sum() // (k * k))
    br, bc = br[inside], bc[inside]
    cells = np.zeros(base.shape, dtype=bool)
    cells[br, bc] = True
    if np.any(cells):
        rr, cc = np.nonzero(cells)
        window = (int(rr.min()), int(rr.max()) + 1, int(cc.min()), int(cc.max()) + 1)
        cleared = cells & swept_mask(base, swept_xy, clear_m, window)
    else:
        cleared = cells
    stats["swept_cleared"] = int(cleared.sum())
    lethal = cells & ~cleared
    stats["marked"] = int((lethal & (out != LETHAL)).sum())
    out[lethal] = LETHAL
    stats["lethal"] = int((out == LETHAL).sum())
    return out, stats


def stamp_rolling(out, base, rolling, map_from_grid, k=ROLLING_SUBSAMPLE):
    """Write rolling's OCCUPIED cells (>= OCCUPIED_MIN) as LETHAL into out (base's cells) -> count newly written.
    map_from_grid = (x, y, yaw): rolling's frame in base's frame (map <- odom). Each cell is sampled k x k (module
    doc). Cells off the base are skipped."""
    occ = rolling.data >= OCCUPIED_MIN
    rows, cols = np.nonzero(occ)
    if not len(rows):
        return 0
    gx, gy = rolling.cells_to_world(rows, cols, k)
    tx, ty, yaw = map_from_grid
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = tx + c * gx - s * gy, ty + s * gx + c * gy
    br, bc, inside = base.world_to_cells(x, y)
    br, bc = br[inside], bc[inside]
    if not len(br):
        return 0
    hit = np.zeros(out.shape, dtype=bool)
    hit[br, bc] = True
    new = int((hit & (out != LETHAL)).sum())
    out[hit] = LETHAL
    return new


class SweptTrack(object):
    """The car base's odometry-frame positions (stamp, x, y), one every SWEPT_STEP_M, and their map-frame positions
    under an alignment (host/map_align message dict; None = map -> odom identity, bench_pose's start)."""

    def __init__(self, step_m=SWEPT_STEP_M, max_n=SWEPT_MAX):
        self.step_m, self.max_n = step_m, max_n
        self.stamps, self.xy = [], []
        self.align = None
        self._map_xy = None              # cached map-frame array for (len, align)
        self._cache_key = None

    def __len__(self):
        return len(self.xy)

    def add(self, stamp, x, y):
        """-> True when the point was kept (moved >= step since the last kept one)."""
        if not all(math.isfinite(v) for v in (stamp, x, y)):
            return False
        if self.xy and math.hypot(x - self.xy[-1][0], y - self.xy[-1][1]) < self.step_m:
            return False
        self.stamps.append(float(stamp))
        self.xy.append((float(x), float(y)))
        if len(self.xy) > self.max_n:
            drop = len(self.xy) - self.max_n
            del self.stamps[:drop], self.xy[:drop]
            self._cache_key = None
        return True

    def set_align(self, doc):
        self.align = None if doc is None or map_align.is_identity(doc) else doc
        self._cache_key = None

    def map_xy(self):
        """N x 2 map-frame positions (C(stamp) x odometry position, the correction of the node nearest in time as
        map_align.Corrections.at; outside the graph's span: map -> odom)."""
        n = len(self.xy)
        if not n:
            return np.zeros((0, 2))
        key = (n, id(self.align), self.xy[0], self.xy[-1])
        if key == self._cache_key:
            return self._map_xy
        pts = np.asarray(self.xy, dtype=float)
        if self.align is None:
            out = pts
        else:
            corr = map_align.Corrections(self.align)
            t = np.asarray(self.stamps, dtype=float)
            mats = np.stack(corr.mats + [corr.last])           # last index = map -> odom
            last = len(corr.mats)
            if corr.stamps:
                s = np.asarray(corr.stamps, dtype=float)
                k = np.searchsorted(s, t, side="left")
                kk = np.clip(k, 1, len(s) - 1) if len(s) > 1 else np.zeros_like(k)
                if len(s) > 1:
                    nearer_next = (s[kk] - t) < (t - s[kk - 1])
                    idx = np.where(nearer_next, kk, kk - 1)
                    idx = np.where(k == 0, 0, np.where(k >= len(s), len(s) - 1, idx))
                else:
                    idx = np.zeros_like(k)
                span = (t >= s[0] - map_align.SPAN_BEFORE_S) & (t <= s[-1] + map_align.SPAN_AFTER_S)
                idx = np.where(span, idx, last)
            else:
                idx = np.full(len(t), last)
            m = mats[idx]
            out = np.einsum("nij,nj->ni", m[:, :2, :2], pts) + m[:, :2, 3]
        self._map_xy, self._cache_key = out, key
        return out


class GlobalMapCore(object):
    """What goes on OUT_TOPIC (module doc). Feed it with on_*; tick(now) -> (Grid to publish or None). Every decision
    worth a log line goes through record(event, **fields)."""

    def __init__(self, record=None, status_max_age_s=STATUS_MAX_AGE_S, fallback_s=PLANNER_FALLBACK_S,
                 clear_m=SWEPT_CLEAR_M, min_neighbours=MIN_NEIGHBOURS):
        self.record = record or (lambda event, **fields: None)
        self.rolling, self.rolling_tf = None, None   # the local layer's latest grid, map <- its frame (x, y, yaw)
        self.status_max_age_s, self.fallback_s = status_max_age_s, fallback_s
        self.clear_m, self.min_neighbours = clear_m, min_neighbours
        self.base, self.base_source = None, None
        self.live, self.live_problem = None, None
        self.status_row, self.status_at = None, None
        self.track = SweptTrack()
        self.fallback_until = None
        self.mode, self.reason = None, None          # what was last published
        self.published = None                        # the last published Grid
        self.dirty = True
        self.stats = {}
        self.publishes = 0

    # ------------------------------------------------------------------ inputs
    def on_base(self, grid, source):
        changed = not grid.same_geometry(self.base) or self.base is None or not np.array_equal(grid.data,
                                                                                               self.base.data)
        self.base, self.base_source = grid, source
        if changed:
            self.dirty = True
            self.record("base_received", source=source, width=int(grid.shape[1]), height=int(grid.shape[0]),
                        resolution=grid.resolution, origin=[grid.origin_x, grid.origin_y, grid.origin_yaw])
        return changed

    def on_live(self, grid, now):
        problem = None
        if grid.frame != FRAME:
            problem = "frame %r, not %r" % (grid.frame, FRAME)
        elif self.status_row is not None and self.status_row.get("state") != "mapping":
            problem = "mapping state %r" % self.status_row.get("state")
        if problem is not None:
            if problem != self.live_problem:
                self.live_problem = problem
                self.record("live_grid_ignored", topic=LIVE_TOPIC, reason=problem)
            return False
        first = self.live is None or self.live_problem is not None
        self.live, self.live_problem = grid, None
        self.dirty = True
        if first:
            self.record("live_grid_received", topic=LIVE_TOPIC, width=int(grid.shape[1]), height=int(grid.shape[0]),
                        resolution=grid.resolution)
        return True

    def on_rolling(self, grid):
        """The rolling local layer's latest grid (any frame; on_rolling_tf brings it into the map frame)."""
        first = self.rolling is None
        self.rolling = grid
        self.dirty = True
        if first:
            self.record("rolling_grid_received", topic=ROLLING_TOPIC, frame=grid.frame, width=int(grid.shape[1]),
                        height=int(grid.shape[0]), resolution=grid.resolution)

    def on_rolling_tf(self, map_from_grid):
        """map <- rolling frame as (x, y, yaw), or None (no transform: the rolling cells stay out, logged once)."""
        prev = self.rolling_tf
        if map_from_grid is None:
            if prev is not None:
                self.record("rolling_tf_lost", level="warn")
                self.rolling_tf = None
                self.dirty = True
            return
        x, y, yaw = (float(v) for v in map_from_grid)
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return
        if prev is None:
            self.record("rolling_tf", x=round(x, 3), y=round(y, 3), yaw=round(yaw, 4))
        elif (math.hypot(x - prev[0], y - prev[1]) < ROLLING_TF_EPS_M
              and abs(math.atan2(math.sin(yaw - prev[2]), math.cos(yaw - prev[2]))) < ROLLING_TF_EPS_RAD):
            return
        self.rolling_tf = (x, y, yaw)
        self.dirty = True

    def on_status(self, row, now):
        if not isinstance(row, dict):
            return
        prev = self.status_row
        self.status_row, self.status_at = row, now
        if prev is None:
            self.record("mapping_status", state=row.get("state"), session=row.get("session"))
            return
        if prev.get("state") != row.get("state") or prev.get("session") != row.get("session"):
            self.record("mapping_status", state=row.get("state"), session=row.get("session"),
                        previous=prev.get("state"))
            # a stop / reset / failure: the grid of that map is done with (a reset's new rtabmap sends a new one)
            if self.live is not None and (row.get("state") != "mapping" or prev.get("session") != row.get("session")
                                          or prev.get("state") != "mapping"):
                self.live = None
                self.record("live_grid_dropped", state=row.get("state"), session=row.get("session"))
            self.dirty = True

    def on_odom(self, stamp, x, y):
        if self.track.add(stamp, x, y):
            self.dirty = True

    def on_align(self, doc):
        if not isinstance(doc, dict) or "map_odom" not in doc or "nodes" not in doc:
            self.record("align_ignored", reason="not an alignment message")
            return
        try:
            map_align.Corrections(doc)
        except Exception as exc:                      # noqa: BLE001 - a bad message keeps the previous one
            self.record("align_ignored", reason=repr(exc))
            return
        self.track.set_align(doc)
        self.dirty = True

    def on_planner_failure(self, now, text=""):
        """Nav2's planner reported a failed plan. While the live or the rolling cells are in the published map: the
        base for fallback_s (the next replan is the plain one). -> True when that fallback started."""
        if self.mode != "live" and not self.stats.get("rolling_marked"):
            return False
        self.fallback_until = now + self.fallback_s
        self.record("planner_failed_fallback", seconds=self.fallback_s, message=str(text)[:200],
                    marked=self.stats.get("lethal"))
        return True

    # ------------------------------------------------------------------ decision
    def wanted(self, now):
        """("live", None) or ("base", reason)."""
        if self.fallback_until is not None:
            if now < self.fallback_until:
                return "base", "planner_failed_fallback"
            self.fallback_until = None
            self.record("planner_fallback_over")
        row, at = self.status_row, self.status_at
        if row is None or at is None:
            return "base", "no_mapping_status"
        if not 0 <= now - at <= self.status_max_age_s:
            return "base", "mapping_status_stale"
        if row.get("state") != "mapping":
            return "base", "not_mapping:%s" % row.get("state")
        if self.live is None:
            return "base", "no_live_grid"
        if self.live.frame != self.base.frame:
            return "base", "frame_mismatch"
        return "live", None

    def tick(self, now):
        """-> the Grid to publish now, or None (nothing new)."""
        if self.base is None:
            return None
        mode, reason = self.wanted(now)
        changed_mode = (mode, reason) != (self.mode, self.reason)
        if not changed_mode and not self.dirty:
            return None
        self.dirty = False
        if mode == "live":
            try:
                data, stats = compose(self.base, self.live, self.track.map_xy(), self.clear_m, self.min_neighbours)
            except Exception as exc:                  # noqa: BLE001 - never stop: the base instead
                self.record("compose_failed", reason=repr(exc))
                mode, reason = "base", "compose_failed"
                changed_mode = (mode, reason) != (self.mode, self.reason)
                data, stats = self.base.data, {}
        else:
            data, stats = self.base.data, {}
        if (self.rolling is not None and self.rolling_tf is not None
                and reason not in ("planner_failed_fallback", "compose_failed")):
            try:
                data = data.copy() if data is self.base.data else data
                stats = dict(stats, rolling_marked=stamp_rolling(data, self.base, self.rolling, self.rolling_tf))
            except Exception as exc:                  # noqa: BLE001 - never stop: without the rolling cells
                self.record("rolling_stamp_failed", reason=repr(exc))
                data = self.base.data if mode == "base" else data
        out = Grid(data, self.base.resolution, self.base.origin_x, self.base.origin_y, self.base.origin_yaw,
                   self.base.frame)
        if changed_mode:
            level = "info" if mode == "live" or reason in ("no_live_grid", "no_mapping_status", "not_mapping:exported",
                                                           "not_mapping:saving") else "warn"
            self.record("mode", mode=mode, reason=reason, previous=self.mode, previous_reason=self.reason,
                        level=level, swept_points=len(self.track), **stats)
        if (not changed_mode and self.published is not None and out.same_geometry(self.published)
                and np.array_equal(out.data, self.published.data)):
            return None
        self.mode, self.reason, self.stats = mode, reason, stats
        self.published = out
        self.publishes += 1
        return out

    def status(self, now):
        left = None if self.fallback_until is None else round(max(0.0, self.fallback_until - now), 1)
        return {"mode": self.mode, "reason": self.reason, "publishes": self.publishes,
                "base_source": self.base_source, "live_grid": self.live is not None,
                "swept_points": len(self.track), "fallback_left_s": left, **self.stats}
