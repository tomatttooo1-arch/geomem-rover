"""Trajectory -> search goals (10-25 / 10-26, moved here from tools/car_run/mapping_ctl.py on 10-29) and the runner's
own trajectory while mapping (10-29, find while mapping).

``search_waypoints`` / ``dense_nodes`` thin a trajectory [(stamp, x, y, yaw)] (map frame) to the MVP's search goals
and the probabilistic search's dense nodes. mapping_ctl applies them to the export's optimized poses at the mapping
stop (status "search_waypoints" / "search_nodes"); before that stop there is no export, so the runner applies them to
``LiveTrack`` -- the base poses of its own processed frames -- and a find started while mapping searches the places
the car has driven through so far instead of only turning on the spot (10-27 run 205546: a find while mapping got
``no_search_nodes`` -> three quarter turns -> not_found).

LiveTrack keeps every point in the alignment the object memory is in (host/live_align.py): a live alignment that
moves the memory moves the track by the same per-stamp difference (``move``), so track, memory and TF map -> odom stay
in one frame. No ROS imports. Python 3.8 compatible.
"""
import math

import numpy as np

# search waypoints from the mapping trajectory (10-25)
SEARCH_SPACING_M = 1.0
SEARCH_MAX = 6
SEARCH_MIN_FROM_START_M = 0.5
SEARCH_CENTRE_MIN_M = 0.3   # nearer the centroid than this: keep the node's own heading
# dense trajectory nodes for the probabilistic search (10-26, host/search_belief.py): status "search_nodes"
NODES_SPACING_M = 0.5
NODES_MAX = 80


def search_waypoints(nodes, spacing=SEARCH_SPACING_M, max_n=SEARCH_MAX, min_from_start=SEARCH_MIN_FROM_START_M):
    """[[x, y, yaw]] search goals from trajectory nodes [(stamp, x, y, yaw)] (map frame; any order, sorted by stamp).

    Walk the trajectory in time order; keep a node when it is >= min_from_start from the first node and >= spacing
    from every node kept so far (a revisited place is kept once). More than max_n -> max_n spread evenly over the kept
    ones (the whole trajectory stays covered). Yaw: towards the centroid of all nodes (look into the mapped area),
    the node's own heading when it is within SEARCH_CENTRE_MIN_M of the centroid."""
    nodes = sorted((float(n[0]), float(n[1]), float(n[2]), float(n[3])) for n in nodes)
    if not nodes:
        return []
    cx = sum(n[1] for n in nodes) / len(nodes)
    cy = sum(n[2] for n in nodes) / len(nodes)
    x0, y0 = nodes[0][1], nodes[0][2]
    kept = []
    for _t, x, y, yaw in nodes:
        if math.hypot(x - x0, y - y0) < min_from_start:
            continue
        if any(math.hypot(x - k[0], y - k[1]) < spacing for k in kept):
            continue
        kept.append((x, y, yaw))
    if len(kept) > max_n:
        kept = [kept[int(round(i * (len(kept) - 1) / float(max_n - 1)))] for i in range(max_n)] if max_n > 1 \
            else kept[:max_n]
    out = []
    for x, y, yaw in kept:
        if math.hypot(cx - x, cy - y) >= SEARCH_CENTRE_MIN_M:
            yaw = math.atan2(cy - y, cx - x)
        out.append([x, y, math.atan2(math.sin(yaw), math.cos(yaw))])
    return out


def dense_nodes(nodes, spacing=NODES_SPACING_M, max_n=NODES_MAX):
    """[[x, y, yaw]] of trajectory nodes [(stamp, x, y, yaw)] thinned to >= spacing apart (time order, a revisited
    place kept once, the start included), more than max_n -> max_n spread evenly; the node's own heading."""
    nodes = sorted((float(n[0]), float(n[1]), float(n[2]), float(n[3])) for n in nodes)
    kept = []
    for _t, x, y, yaw in nodes:
        if all(math.hypot(x - k[0], y - k[1]) >= spacing for k in kept):
            kept.append((x, y, yaw))
    if len(kept) > max_n:
        kept = [kept[int(round(i * (len(kept) - 1) / float(max_n - 1)))] for i in range(max_n)] if max_n > 1 \
            else kept[:max_n]
    return [[x, y, math.atan2(math.sin(yaw), math.cos(yaw))] for x, y, yaw in kept]


# LiveTrack (10-29): a point is kept when the base moved >= TRACK_STEP_M or turned >= TRACK_TURN_RAD since the last
# kept one (5 Hz frames, ~0.15 m/s: one point every ~1 s of driving); TRACK_MAX points (a 5-8 min demo run keeps a few
# hundred), the oldest dropped first beyond it.
TRACK_STEP_M = 0.10
TRACK_TURN_RAD = math.radians(20.0)
TRACK_MAX = 20000


class LiveTrack(object):
    """The car's own trajectory while mapping: [(stamp, x, y, yaw)] in the memory's alignment."""

    def __init__(self, step_m=TRACK_STEP_M, turn_rad=TRACK_TURN_RAD, max_n=TRACK_MAX):
        self.step_m, self.turn_rad, self.max_n = step_m, turn_rad, max_n
        self.points = []
        self.moves = 0

    def __len__(self):
        return len(self.points)

    def clear(self):
        self.points = []

    def add(self, stamp, x, y, yaw):
        """True when the pose was kept (finite values, moved or turned enough since the last kept one)."""
        values = (float(stamp), float(x), float(y), float(yaw))
        if not all(math.isfinite(v) for v in values):
            return False
        if self.points:
            t0, x0, y0, yaw0 = self.points[-1]
            if values[0] <= t0:
                return False
            turned = abs(math.atan2(math.sin(values[3] - yaw0), math.cos(values[3] - yaw0)))
            if math.hypot(values[1] - x0, values[2] - y0) < self.step_m and turned < self.turn_rad:
                return False
        self.points.append(values)
        if len(self.points) > self.max_n:
            del self.points[:len(self.points) - self.max_n]
        return True

    def move(self, delta_at):
        """Move every point by ``delta_at(stamp)`` (4x4, e.g. host/map_align.Delta(new, prev).at): the same move
        the memory made at that stamp. A delta that fails for a point leaves that point where it was."""
        out = []
        for t, x, y, yaw in self.points:
            try:
                d = np.asarray(delta_at(t), dtype=float)
                p = d[:3, :3] @ np.array([x, y, 0.0]) + d[:3, 3]
                h = math.atan2(d[1, 0], d[0, 0])
                out.append((t, float(p[0]), float(p[1]), math.atan2(math.sin(yaw + h), math.cos(yaw + h))))
            except Exception:                   # noqa: BLE001 - one bad point never loses the track
                out.append((t, x, y, yaw))
        self.points = out
        self.moves += 1

    def nodes(self):
        """(dense nodes ((x, y, yaw), ...) or None, search waypoints (PoseGoal-ready [[x, y, yaw]]) ) of the track."""
        if not self.points:
            return None, []
        dense = dense_nodes(self.points)
        return (tuple(tuple(n) for n in dense) if dense else None), search_waypoints(self.points)
