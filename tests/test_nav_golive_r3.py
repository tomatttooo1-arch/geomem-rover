"""Nav go-live round 3 (2026-10-17, docs/design/REAL_CAR_NAV_GOLIVE_20261016.md section 9): the body seed margin.

Board run ng2_1017 (r2, D4): DWB rejected all 419 trajectories every cycle, 100 % "ObstacleFootprint/Trajectory Hits
Unknown Region". Cause (section 9.1): DWB puts the START pose into every trajectory, and the start pose's own outline
(Nav2's 16-gon, rasterised with Bresenham between vertex cells) touched three UNKNOWN cells at 0.30-0.304 m from the
body centre that the bridge's body seed (host/rolling_nav_layer.body_cells, radius 0.30) had left out: one by float
rounding (6 * 0.05 = 0.30000000000000004 > 0.30), one because Bresenham picks a cell 0.304 m out.
Fix: RollingLayerBridge(body_margin_m) widens the seed (UNKNOWN -> FREE, current pose only, OCCUPIED untouched) by
<= 5 cm; the car stack passes 0.05. The simulation / bench launches keep 0 (byte-identical).

The Nav2 side is a re-typed replica of Nav2 Humble 1.1.20 (makeFootprintFromRadius, Costmap2D::updateOrigin /
worldToMap / mapToWorld, nav2_util::LineIterator, StaticLayer rolling copy, StandardTrajectoryGenerator,
XYThetaIterator / OneDVelocityIterator, BaseObstacleCritic, ObstacleFootprintCritic); the InflationLayer is left out
(no lethal cell within 0.9 m of the body in the fixture, so it cannot change any verdict below). Two known answers from
the board pin the replica: 419 twists per cycle and 0 of 419 valid on the board snapshot without the margin.
Fixture: the /fpga/rolling_map/grid the board published at 01:31 (ng2_1017 ng/grid_0135.json, rows y +0.60..-0.60,
columns x -0.60..+1.20 at 5 cm; '?' unknown, '#' lethal, '.' free, 'B' the body centre cell).
No ROS, docker or GPU; Python 3.8 compatible."""
import math
import os
import random
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import host.rolling_nav_layer as L  # noqa: E402
from host.m7_rolling import FREE, OCCUPIED, NavigationDelta  # noqa: E402

MS = 1_000_000
VOX = 0.05
N, NZ, BASE = 80, 32, (-40, -40, -4)      # the board bridge: dims (80, 80, 32), 4 m window, odom origin at its centre
R = 0.30

BOARD_ART = [
    "??????????????????????????????????...",
    "??????????????????????????????#######",
    "???????????????????????????????....##",
    "?????????????????????????????........",
    "???????????????????????????..........",
    "?????????????????????????............",
    "???????????????????????..............",
    "????????........??????...............",
    "???????..........???.................",
    "??????...............................",
    "??????...............................",
    "??????...............................",
    "??????......B........................",
    "??????...............................",
    "??????...............................",
    "??????...............................",
    "??????...............................",
    "???????..........???.................",
    "????????........??????...............",
    "????????????????????????.............",
    "??????????????????????????...........",
    "???????????????????????????..........",
    "?????????????????????????????........",
    "???????????????????????????????...###",
    "?????????????????????????????????.###",
]


# ---------------------------------------------------------------- the bridge side (code under test)
def art_cells():
    """{(i, j): 'F' | 'O'} grid cells (origin -2.0 m, 5 cm) of the board snapshot; everything else UNKNOWN."""
    out = {}
    for row, k in zip(BOARD_ART, range(12, -13, -1)):
        for c, ch in zip(range(-12, 25), row):
            if ch in '.B':
                out[(c + 40, k + 40)] = 'F'
            elif ch == '#':
                out[(c + 40, k + 40)] = 'O'
    return out


def bridge_with(cells, margin, radius=R):
    band = L.HeightBand(ground_z_m=0.0, robot_height_m=0.25)
    b = L.RollingLayerBridge(dims=(N, N, NZ), voxel_m=VOX, band=band, clock_id="mono", body_radius_m=radius,
                             body_margin_m=margin)
    layers = band.band_layers(BASE[2], NZ, VOX)
    ch = []
    for (i, j), kind in cells.items():
        gx, gy = BASE[0] + i, BASE[1] + j
        for n, lz in enumerate(layers):
            state = OCCUPIED if (kind == 'O' and n == 0) else FREE
            ch.append((gx, gy, BASE[2] + int(lz), state))
    t = 5_000 * MS
    d = NavigationDelta(1, 1, 1, t, "mono", t + 200 * MS, BASE, (N, N, NZ), VOX, tuple(ch))
    changed, reason, _ = b.on_delta(d, now_ns=t)
    assert changed, reason
    return b


def published(b, body=(0.0, 0.0)):
    """the OccupancyGrid data the bridge sends to Nav2, as [j, i] (100 / 0 / -1)"""
    return np.asarray(b.grid(body_xy=body, now_ns=5_001 * MS).data).reshape(N, N)


# ---------------------------------------------------------------- the Nav2 side (re-typed replica, 1.1.20)
NO_INFO, LETHAL, INSCRIBED = 255, 254, 253
RES = 0.05                                  # Costmap2DROS resolution (double)
GRID_RES = float(np.float32(0.05))          # OccupancyGrid.info.resolution is float32 -> StaticLayer's own map
SIZE = int(4 / RES)


def costmap_origin(rx=0.0, ry=0.0):
    """Costmap2DROS::updateMap -> Costmap2D::updateOrigin from origin_x/y = 0 (rolling_layer_nav2.yaml)"""
    half = (SIZE - 1 + 0.5) * RES / 2
    return tuple(0.0 + int(((r - half) - 0.0) / RES) * RES for r in (rx, ry))


ORIGIN = costmap_origin()


def world_to_map(wx, wy, origin=ORIGIN):
    if wx < origin[0] or wy < origin[1]:
        return None
    mx, my = int((wx - origin[0]) / RES), int((wy - origin[1]) / RES)
    return (mx, my) if mx < SIZE and my < SIZE else None


def master(grid, origin=ORIGIN):
    """StaticLayer::updateCosts, rolling branch: every master cell centre looked up in the layer's own map
    (trinary: -1 -> NO_INFORMATION, 100 -> LETHAL, 0 -> FREE)"""
    m = np.full((SIZE, SIZE), NO_INFO, np.int32)
    for my in range(SIZE):
        for mx in range(SIZE):
            wx, wy = origin[0] + (mx + 0.5) * RES, origin[1] + (my + 0.5) * RES
            i, j = int((wx - (-2.0)) / GRID_RES), int((wy - (-2.0)) / GRID_RES)
            if 0 <= i < N and 0 <= j < N:
                v = grid[j, i]
                m[my, mx] = NO_INFO if v < 0 else (LETHAL if v >= 100 else 0)
    return m


FOOTPRINT = [(math.cos(i * 2 * math.pi / 16) * R, math.sin(i * 2 * math.pi / 16) * R) for i in range(16)]


def bresenham(x0, y0, x1, y1):
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    xi1 = xi2 = 1 if x1 >= x0 else -1
    yi1 = yi2 = 1 if y1 >= y0 else -1
    if dx >= dy:
        xi1, yi2, den, num, add, npx = 0, 0, dx, dx // 2, dy, dx
    else:
        xi2, yi1, den, num, add, npx = 0, 0, dy, dy // 2, dx, dy
    x, y, out = x0, y0, []
    for _ in range(npx + 1):
        out.append((x, y))
        num += add
        if num >= den:
            num -= den
            x += xi1
            y += yi1
        x += xi2
        y += yi2
    return out


def outline(pose, origin=ORIGIN):
    px, py, th = pose
    c, s = math.cos(th), math.sin(th)
    pts = [(px + fx * c - fy * s, py + fx * s + fy * c) for fx, fy in FOOTPRINT]
    cells = []
    for a, b in list(zip(pts[:-1], pts[1:])) + [(pts[-1], pts[0])]:
        ca, cb = world_to_map(*a, origin=origin), world_to_map(*b, origin=origin)
        cells += bresenham(ca[0], ca[1], cb[0], cb[1])
    return cells


# board parameters after apply_speed_limits (car) over nav2_params.yaml + dwb.yaml
DWB = dict(vx=(0.0, 0.15), vth=0.4, acc_x=0.3, dec_x=-0.3, acc_th=0.6, dec_th=-0.6, nx=20, nth=20, sim=1.7,
           lin_g=0.05, ang_g=0.025)
EPS = 1e-5


def _proj(v0, acc, dec, dt, target):
    return min(target, v0 + acc * dt) if v0 < target else max(target, v0 + dec * dt)


def _one_d(lo_lim, hi_lim, acc, dec, n):
    hi, lo = _proj(0.0, acc, dec, DWB['sim'], hi_lim), _proj(0.0, acc, dec, DWB['sim'], lo_lim)
    inc = (hi - lo) / (max(2, n) - 1)
    vals, cur, zero = [], lo, True
    while not cur > hi + EPS:
        vals.append(cur)
        if zero and cur < 0 < cur + inc <= hi + EPS:
            vals.append(0.0)
            zero = False
        cur += inc
    return vals


def twists():
    xs = _one_d(DWB['vx'][0], DWB['vx'][1], DWB['acc_x'], DWB['dec_x'], DWB['nx'])
    ths = _one_d(-DWB['vth'], DWB['vth'], DWB['acc_th'], DWB['dec_th'], DWB['nth'])
    return [(vx, vt) for vx in xs for vt in ths if not (vx == 0.0 and vt == 0.0) and vx * vx <= 0.15 ** 2 + EPS]


def trajectory(vx, vt, start=(0.0, 0.0, 0.0)):
    n = math.ceil(max(abs(vx) * DWB['sim'] / DWB['lin_g'], abs(vt) * DWB['sim'] / DWB['ang_g'])) or 1
    dt = DWB['sim'] / n
    pose, v, w, poses = start, 0.0, 0.0, [start]          # the start pose is part of every trajectory
    for _ in range(n):
        v = _proj(v, DWB['acc_x'], DWB['dec_x'], dt, vx)
        w = _proj(w, DWB['acc_th'], DWB['dec_th'], dt, vt)
        pose = (pose[0] + v * math.cos(pose[2]) * dt, pose[1] + v * math.sin(pose[2]) * dt, pose[2] + w * dt)
        poses.append(pose)
    return poses + [pose]                                   # include_last_point


def verdict(m, vx, vt):
    """None (valid) or the first critic failure, in the board's critic order (BaseObstacle, ObstacleFootprint)"""
    poses = trajectory(vx, vt)
    for p in poses:
        c = world_to_map(p[0], p[1])
        if m[c[1], c[0]] in (LETHAL, INSCRIBED, NO_INFO):
            return 'BaseObstacle'
    for p in poses:
        for mx, my in outline(p):
            if m[my, mx] == LETHAL:
                return 'ObstacleFootprint/Obstacle'
            if m[my, mx] == NO_INFO:
                return 'ObstacleFootprint/Unknown'
    return None


def closest(i, j, cx=0.0, cy=0.0):
    """closest-point distance of grid cell (i, j) (origin -2.0) to (cx, cy)"""
    x0, y0 = -2.0 + i * VOX, -2.0 + j * VOX
    return math.hypot(max(x0 - cx, cx - (x0 + VOX), 0.0), max(y0 - cy, cy - (y0 + VOX), 0.0))


class BoardKnownAnswers(unittest.TestCase):
    """The replica must reproduce what the board logged before it is allowed to judge the fix."""

    def test_419_twists_per_cycle(self):
        self.assertEqual(len(twists()), 419)          # navigation.log: "No valid trajectories out of 419!"
        self.assertEqual(sum(1 for vx, _ in twists() if vx == 0.0), 20)

    def test_board_snapshot_without_margin_rejects_everything(self):
        m = master(published(bridge_with(art_cells(), 0.0)))
        reasons = {verdict(m, vx, vt) for vx, vt in twists()}
        self.assertEqual(reasons, {'ObstacleFootprint/Unknown'})     # navigation.log: "1.00: ObstacleFootprint/..."

    def test_the_start_outline_is_the_culprit(self):
        m = master(published(bridge_with(art_cells(), 0.0)))
        bad = sorted({(round(ORIGIN[0] + (mx + .5) * RES, 3), round(ORIGIN[1] + (my + .5) * RES, 3))
                      for mx, my in outline((0.0, 0.0, 0.0)) if m[my, mx] == NO_INFO})
        self.assertEqual(bad, [(-0.025, 0.325), (0.025, 0.325), (0.075, 0.325)])
        self.assertEqual(6 * 0.05, 0.30000000000000004)               # why a cell AT 0.30 m is left out


class BodyMargin(unittest.TestCase):
    def test_board_snapshot_with_margin_unlocks_rotation(self):
        m = master(published(bridge_with(art_cells(), 0.05)))
        self.assertFalse([c for c in outline((0.0, 0.0, 0.0)) if m[c[1], c[0]] in (NO_INFO, LETHAL)])
        v = {(vx, vt): verdict(m, vx, vt) for vx, vt in twists()}
        self.assertTrue(all(v[k] is None for k in v if k[0] == 0.0), "a rotation in place is still rejected")
        self.assertGreater(sum(1 for r in v.values() if r is None), 20)
        # outside the footprint nothing was cleared: the full-speed straight still meets the side band the camera
        # has not seen (x 0.2-0.4 m, |y| 0.2-0.35 m) -> it stays rejected (owner question, section 9)
        fastest = max(k for k in v if k[1] == 0.0)
        self.assertAlmostEqual(fastest[0], 0.15, places=9)
        self.assertEqual(v[fastest], 'ObstacleFootprint/Unknown')

    def test_only_unknown_within_radius_plus_margin_changes(self):
        cells = art_cells()
        g0 = published(bridge_with(cells, 0.0))
        g5 = published(bridge_with(cells, 0.05))
        changed = np.argwhere(g0 != g5)
        self.assertGreater(len(changed), 0)
        for j, i in changed:
            self.assertEqual((g0[j, i], g5[j, i]), (-1, 0))
            self.assertLessEqual(closest(i, j), R + 0.05 + 1e-9)
        # the three culprit cells (grid i/j, origin -2.0) are FREE now
        for i, j in ((39, 46), (40, 46), (41, 46)):
            self.assertEqual((g0[j, i], g5[j, i]), (-1, 0), (i, j))
        # every UNKNOWN cell beyond 0.35 m is untouched, e.g. the side band at (0.325, 0.275) m
        far = [(i, j) for j in range(N) for i in range(N) if closest(i, j) > R + 0.05 + 1e-9 and g0[j, i] == -1]
        self.assertGreater(len(far), 4000)
        self.assertTrue(all(g5[j, i] == -1 for i, j in far))
        self.assertEqual(g5[45, 46], -1)

    def test_occupied_inside_the_margin_is_kept(self):
        cells = dict(art_cells())
        cells[(40, 46)] = 'O'                           # an obstacle 0.30 m beside the body
        g = published(bridge_with(cells, 0.05))
        self.assertEqual(g[46, 40], 100)

    def test_margin_moves_with_the_body(self):
        b = bridge_with({}, 0.05)                       # nothing observed: only the seed is FREE
        here = published(b, (0.0, 0.0))
        there = published(b, (0.5, 0.0))
        self.assertEqual(here[40, 46], 0)               # (0.325, 0.025) m: in the seed at (0, 0)
        self.assertEqual(there[40, 46], 0)              # still within 0.35 of (0.5, 0)
        self.assertEqual(there[40, 33], -1)             # (-0.325, 0.025) m: left behind -> UNKNOWN again
        self.assertEqual(here[40, 33], 0)
        free = np.argwhere(there == 0)
        self.assertTrue(all(closest(i, j, 0.5, 0.0) <= R + 0.05 + 1e-9 for j, i in free))

    def test_any_pose_outline_lies_in_the_seed(self):
        """random poses (sub-cell offsets, any yaw), nothing observed: the CURRENT outline must be all FREE"""
        rng = random.Random(17)
        b = bridge_with({}, 0.05)
        b0 = bridge_with({}, 0.0)
        misses0 = 0
        for _ in range(60):
            px, py, th = rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4), rng.uniform(-math.pi, math.pi)
            for bb, strict in ((b, True), (b0, False)):
                m = master(published(bb, (px, py)))
                bad = [c for c in outline((px, py, th)) if m[c[1], c[0]] == NO_INFO]
                if strict:
                    self.assertFalse(bad, (px, py, th))
                else:
                    misses0 += bool(bad)
        self.assertGreater(misses0, 0)                  # without the margin some current outlines hit UNKNOWN

    def test_margin_is_capped_at_5cm(self):
        band = L.HeightBand(ground_z_m=0.0, robot_height_m=0.25)
        for bad in (0.06, -0.01, float('nan'), float('inf'), True, '0.05'):
            with self.assertRaises(ValueError, msg=repr(bad)):
                L.RollingLayerBridge(dims=(N, N, NZ), voxel_m=VOX, band=band, clock_id="mono", body_radius_m=R,
                                     body_margin_m=bad)
        for ok in (0.0, 0.05, 0):
            L.RollingLayerBridge(dims=(N, N, NZ), voxel_m=VOX, band=band, clock_id="mono", body_radius_m=R,
                                 body_margin_m=ok)
        self.assertEqual(L.BODY_MARGIN_MAX_M, 0.05)

    def test_raw_grid_stays_strict(self):
        b = bridge_with(art_cells(), 0.05)
        raw = np.asarray(b.strict_grid().data).reshape(N, N)
        self.assertEqual(raw[46, 40], -1)               # grid_raw (measurement) never gets the seed


def read(rel):
    with open(os.path.join(ROOT, rel)) as f:
        return f.read()


class Wiring(unittest.TestCase):
    def test_car_stack_passes_005_and_others_keep_0(self):
        inner = read('tools/car_run/car_inner.sh')
        line = [l for l in inner.splitlines() if 'host/ros_rolling_nav.py' in l and not l.lstrip().startswith('#')]
        self.assertEqual(len(line), 1)
        self.assertIn('-p robot_radius_m:=0.30 -p body_margin_m:=0.05', line[0])
        for rel in ('tools/sim_loop/stack_inner.sh', 'tools/car_geo_producer/rig_inner.sh', 'tools/rolling_nav2_smoke.sh'):
            self.assertNotIn('body_margin_m', read(rel), rel)
        node = read('host/ros_rolling_nav.py')
        self.assertIn('("body_margin_m", 0.0)', node)
        self.assertIn('body_margin_m=p("body_margin_m")', node)


if __name__ == '__main__':
    unittest.main()
