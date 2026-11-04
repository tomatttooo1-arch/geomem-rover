"""Nav2 Humble local-costmap arithmetic, re-typed in Python for the B1 bench (2026-11-02).

Source read: ros-navigation/navigation2 branch humble, nav2_costmap_2d
  plugins/inflation_layer.cpp  updateCosts / enqueue / computeCaches / generateIntegerDistances
  include/nav2_costmap_2d/inflation_layer.hpp  computeCost / costLookup / distanceLookup
  src/costmap_2d.cpp  cellDistance = ceil(world / resolution)
  src/footprint.cpp   makeFootprintFromRadius (16-gon), calculateMinAndMaxDistances
(fetched 2026-11-02 into the B1d scratchpad; not vendored). StaticLayer with trinary_costmap and
track_unknown_space: -1 -> 255, >= lethal_cost_threshold (100) -> 254, else 0.

Two inflation references:
  inflate_nav2   the wavefront exactly as Nav2 runs it (distance bins, seen_, 4-neighbour enqueue
                 carrying the source cell) -- what the car's Nav2 computes
  inflate_exact  max over every lethal cell of the cached cost (the exact form the PL core computes)
No ROS. Python 3.8 compatible.
"""
import math

import numpy as np

LETHAL, INSCRIBED, UNKNOWN, FREE = 254, 253, 255, 0


def inscribed_radius_from_radius(radius):
    """Nav2 robot_radius -> inscribed radius: min distance from the centre to the 16-gon's vertices/edges."""
    pts = [(math.cos(i * 2 * math.pi / 16) * radius, math.sin(i * 2 * math.pi / 16) * radius) for i in range(16)]

    def to_line(px, py, x0, y0, x1, y1):            # footprint.cpp distanceToLine
        a, b, c, d = px - x0, py - y0, x1 - x0, y1 - y0
        dot, len_sq = a * c + b * d, c * c + d * d
        param = dot / len_sq if len_sq != 0 else -1
        if param < 0:
            xx, yy = x0, y0
        elif param > 1:
            xx, yy = x1, y1
        else:
            xx, yy = x0 + param * c, y0 + param * d
        return math.hypot(px - xx, py - yy)

    best = float("inf")
    for i in range(16):
        a, b = pts[i], pts[(i + 1) % 16]
        best = min(best, math.hypot(*a), to_line(0.0, 0.0, a[0], a[1], b[0], b[1]))
    return best


class InflationParams(object):
    def __init__(self, resolution=0.05, inflation_radius=0.45, cost_scaling_factor=3.0, robot_radius=0.30):
        self.resolution = float(resolution)
        self.inflation_radius = float(inflation_radius)
        self.cost_scaling_factor = float(cost_scaling_factor)
        self.inscribed_radius = inscribed_radius_from_radius(float(robot_radius))
        self.cell_radius = int(max(0.0, math.ceil(self.inflation_radius / self.resolution)))   # cellDistance
        self.cache_length = self.cell_radius + 2

    def compute_cost(self, distance):
        if distance == 0:
            return LETHAL
        if distance * self.resolution <= self.inscribed_radius:
            return INSCRIBED
        factor = math.exp(-1.0 * self.cost_scaling_factor * (distance * self.resolution - self.inscribed_radius))
        return int((INSCRIBED - 1) * factor)        # static_cast<unsigned char> truncates

    def cached(self):
        """(distances, costs) cache_length x cache_length, as computeCaches."""
        L = self.cache_length
        dist = np.array([[math.hypot(i, j) for j in range(L)] for i in range(L)])
        cost = np.array([[self.compute_cost(dist[i, j]) for j in range(L)] for i in range(L)], dtype=np.int32)
        return dist, cost

    def pl_lut(self, lut_dim):
        """The table handed to the PL core: lut[i][j] = cached cost when hypot(i,j) <= cell_radius (else the
        cell is never enqueued, cost 0), shape lut_dim x lut_dim, flattened uint8."""
        if self.cell_radius + 2 > lut_dim:
            raise ValueError("inflation radius %d cells exceeds the core's table" % self.cell_radius)
        dist, cost = self.cached()
        out = np.zeros((lut_dim, lut_dim), dtype=np.uint8)
        L = self.cache_length
        for i in range(L):
            for j in range(L):
                if dist[i, j] <= self.cell_radius:
                    out[i, j] = cost[i, j]
        return out.ravel()


def master_from_grid(grid):
    """OccupancyGrid int8 values (row-major [y][x]) -> StaticLayer master costs (trinary, track unknown)."""
    g = np.asarray(grid, dtype=np.int16)
    out = np.zeros(g.shape, dtype=np.int32)
    out[g == -1] = UNKNOWN
    out[g >= 100] = LETHAL
    return out


def _integer_distances(r):
    pts = [(x, y) for y in range(-r, r + 1) for x in range(-r, r + 1) if x * x + y * y <= r * r]
    pts.sort(key=lambda p: p[0] * p[0] + p[1] * p[1])     # std::sort: equal keys in any order, level only
    size = 2 * r + 1
    dm = [[0] * size for _ in range(size)]
    last, level = (0, 0), 0
    for p in pts:
        if p[0] * p[0] + p[1] * p[1] != last[0] * last[0] + last[1] * last[1]:
            level += 1
        dm[p[0] + r][p[1] + r] = level
        last = p
    return dm, level


def inflate_nav2(master, params, inflate_unknown=False):
    """InflationLayer::updateCosts over the whole grid (bounds = the grid), master [y][x] -> new array."""
    m = np.array(master, dtype=np.int32, copy=True)
    h, w = m.shape
    flat = m.ravel()
    dist, cost = params.cached()
    L, R = params.cache_length, params.cell_radius
    dm, max_level = _integer_distances(R + 2)
    rr = R + 2
    bins = [[] for _ in range(max_level + 1)]
    seen = np.zeros(h * w, dtype=bool)
    for j in range(h):                                   # seeds: row-major, as the C++ loop
        for i in range(w):
            if flat[j * w + i] == LETHAL:
                bins[0].append((j * w + i, i, j, i, j))
    for b in bins:
        k = 0
        while k < len(b):
            index, mx, my, sx, sy = b[k]
            k += 1
            if seen[index]:
                continue
            seen[index] = True
            c = int(cost[abs(mx - sx), abs(my - sy)])
            old = int(flat[index])
            if old == UNKNOWN and ((c > FREE) if inflate_unknown else (c >= INSCRIBED)):
                flat[index] = c
            else:
                flat[index] = max(old, c)
            for nb in ((index - 1, mx - 1, my, mx > 0), (index - w, mx, my - 1, my > 0),
                       (index + 1, mx + 1, my, mx < w - 1), (index + w, mx, my + 1, my < h - 1)):
                nidx, nx, ny, ok = nb
                if not ok or seen[nidx]:
                    continue
                d = dist[abs(nx - sx), abs(ny - sy)]
                if d > R:
                    continue
                bins[dm[nx - sx + rr][ny - sy + rr]].append((nidx, nx, ny, sx, sy))
    return flat.reshape(h, w)


def inflate_exact(master, params, inflate_unknown=False):
    """Same update rule, but every cell takes the largest cached cost over ALL lethal cells within the radius."""
    m = np.array(master, dtype=np.int32, copy=True)
    h, w = m.shape
    dist, cost = params.cached()
    R = params.cell_radius
    best = np.zeros((h, w), dtype=np.int32)
    ys, xs = np.nonzero(m == LETHAL)
    for dy in range(-R, R + 1):
        for dx in range(-R, R + 1):
            if dist[abs(dx), abs(dy)] > R:
                continue
            c = int(cost[abs(dx), abs(dy)])
            ty, tx = ys + dy, xs + dx
            ok = (ty >= 0) & (ty < h) & (tx >= 0) & (tx < w)
            np.maximum.at(best, (ty[ok], tx[ok]), c)
    out = m.copy()
    unk = m == UNKNOWN
    if inflate_unknown:
        take = unk & (best > FREE)
    else:
        take = unk & (best >= INSCRIBED)
    out[take] = best[take]
    rest = ~unk
    out[rest] = np.maximum(m[rest], best[rest])
    return out
