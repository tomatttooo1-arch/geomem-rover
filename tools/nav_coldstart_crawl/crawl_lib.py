"""Cold-start "crawl then run" check (2026-10-17, docs/design/NAV_COLDSTART_CRAWL_20261017.md). Validation only.

Shared pieces, ROS-free:
  * the real car's camera (board ng3_1017: Gemini 2 depth 640x480, K from camera_info, mount fitted by
    ground_calib auto: 0.3798669 m, 27.418958 deg pitch, 0.0672 deg roll, x 0.10 m ahead of base_link)
  * a ray-cast depth renderer for a floor + axis-aligned boxes (mm codes, 0 = no return)
  * the producer side exactly as the car runs it (host.geo_producer.GeoProducer with the CPU frame backend and the
    CPU rolling backend, block 32x32x10, height_cell 8, back_cells 1, stride 1) and the bridge
    (host.rolling_nav_layer.RollingLayerBridge, dims 80x80x32, radius 0.30, body_margin_m 0.05, band 0 / 0.31)
  * the Nav2 Humble 1.1.20 DWB replica of tests/test_nav_golive_r3.py, generalised to a moving robot (any pose,
    any start velocity, any costmap origin) and to a master grid read from /local_costmap/costmap.
Python 3.8 compatible.
"""
import math
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

# ---------------------------------------------------------------- the real car (board ng3_1017, 10-17 02:11)
K_BOARD = np.array([[517.1719360351562, 0.0, 324.2197265625],
                    [0.0, 517.1170654296875, 240.64755249023438],
                    [0.0, 0.0, 1.0]])
IMG_W, IMG_H = 640, 480
CAM_HEIGHT_M = 0.3798668680624316
CAM_PITCH_DEG = 27.418957905220942
CAM_ROLL_DEG = 0.06722297120535531
# 10-17 camera tilt 30 -> 10 deg (docs/design/CAM_TILT_10_20261017.md): the mount can be overridden for a run by
# CRAWL_CAM_HEIGHT_M / CRAWL_CAM_PITCH_DEG / CRAWL_CAM_ROLL_DEG (unset = the ng3 fit above, the earlier runs unchanged)
CAM_HEIGHT_M = float(os.environ.get("CRAWL_CAM_HEIGHT_M", CAM_HEIGHT_M))
CAM_PITCH_DEG = float(os.environ.get("CRAWL_CAM_PITCH_DEG", CAM_PITCH_DEG))
CAM_ROLL_DEG = float(os.environ.get("CRAWL_CAM_ROLL_DEG", CAM_ROLL_DEG))
HEIGHT_CELL = 8
ROBOT_RADIUS_M = 0.30
ROBOT_HEIGHT_M = 0.31
BODY_MARGIN_M = 0.05
VOX = 0.05
DIMS = (80, 80, 32)
MAX_RANGE_M = 8.0          # synthetic returns farther than this are dropped (code 0)


def base_camera_T():
    from tools.car_geo_producer.common import MOUNT_XYZ, base_camera
    return base_camera((MOUNT_XYZ[0], MOUNT_XYZ[1], CAM_HEIGHT_M), CAM_PITCH_DEG, CAM_ROLL_DEG)


def odom_base_T(x, y, yaw):
    T = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:3, 3] = (x, y, 0.0)
    return T


_RAYS = {}


def _rays(K, w, h):
    key = (K.tobytes(), w, h)
    if key not in _RAYS:
        u, v = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
        pix = np.stack([u.ravel(), v.ravel(), np.ones(u.size)], 0)
        _RAYS[key] = np.linalg.inv(K) @ pix       # optical frame, z = 1
    return _RAYS[key]


def render_codes_ref(pose, boxes, K=K_BOARD, w=IMG_W, h=IMG_H, T_bc=None):
    """uint16 mm optical-z depth of the floor z=0 plus boxes (x0, x1, y0, y1, z1) seen from base pose (x, y, yaw).
    Optical z = 1 per ray, so the ray parameter is the optical depth. (The first version, used for K2 / D_geom,
    built the rays per call and divided per box; this one is the same arithmetic up to 1/d -- checked equal,
    0 of 307200 codes differ at two poses with a box.)"""
    T_bc = base_camera_T() if T_bc is None else T_bc
    T = odom_base_T(*pose) @ T_bc
    R, c = T[:3, :3], T[:3, 3]
    d = R @ _rays(K, w, h)
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / d
        best = np.where(d[2] < 0, -c[2] * inv[2], np.inf)
        for (x0, x1, y0, y1, z1) in boxes:
            lo, hi = np.array([x0, y0, 0.0]), np.array([x1, y1, z1])
            ta, tb = (lo[:, None] - c[:, None]) * inv, (hi[:, None] - c[:, None]) * inv
            tmin, tmax = np.minimum(ta, tb).max(0), np.maximum(ta, tb).min(0)
            hit = (tmax >= tmin) & (tmax > 0) & (tmin > 0) & (tmin < best)
            best = np.where(hit, tmin, best)
    best[~np.isfinite(best) | (best > MAX_RANGE_M)] = 0.0
    return np.rint(best * 1000.0).astype(np.uint16).reshape(h, w)


_BASE_RAYS = {}


def render_codes(pose, boxes, K=K_BOARD, w=IMG_W, h=IMG_H, T_bc=None):
    """Same codes as render_codes_ref (checked: 0 of 307200 differ at four poses, room + box), faster for the
    closed loop: the camera only yaws and translates with the base, so the floor depth per pixel is a constant;
    a solid is tested only on the rays whose floor hit lies at least as far (horizontally) as the solid's nearest
    point -- the others reach the floor first -- and (10-17, low pitch: the box scenes rendered at ~150 ms, under the
    8 Hz grid instrument condition) for a solid whose top is below the camera, only on the rays that come down to its
    top height no farther (horizontally) than its farthest point -- the others pass above it (upward rays never reach
    it). Exact: checked equal to render_codes_ref (tools/cam_tilt10/render_check.py)."""
    T_bc = base_camera_T() if T_bc is None else T_bc
    key = (K.tobytes(), w, h, T_bc.tobytes())
    if key not in _BASE_RAYS:
        db = T_bc[:3, :3] @ _rays(K, w, h)
        with np.errstate(divide="ignore", invalid="ignore"):
            tfl = np.where(db[2] < 0, -T_bc[2, 3] / db[2], np.inf)
        hyp = np.hypot(db[0], db[1])
        with np.errstate(divide="ignore", invalid="ignore"):
            cot = np.where(db[2] < 0, hyp / -db[2], np.inf)        # horizontal run per metre of descent
        _BASE_RAYS[key] = (db, tfl, tfl * hyp, cot)
    db, tfl, reach, cot = _BASE_RAYS[key]
    x, y, yaw = pose
    cy, sy = math.cos(yaw), math.sin(yaw)
    c = (x + T_bc[0, 3] * cy - T_bc[1, 3] * sy, y + T_bc[0, 3] * sy + T_bc[1, 3] * cy, T_bc[2, 3])
    best = tfl.copy()
    for (x0, x1, y0, y1, z1) in boxes:
        dmin = math.hypot(max(x0 - c[0], 0.0, c[0] - x1), max(y0 - c[1], 0.0, c[1] - y1))
        sel = reach >= dmin - 1e-6
        if z1 < c[2]:
            dmax = math.hypot(max(abs(x0 - c[0]), abs(x1 - c[0])), max(abs(y0 - c[1]), abs(y1 - c[1])))
            sel &= (c[2] - z1) * cot <= dmax + 1e-6
        idx = np.nonzero(sel)[0]
        if idx.size == 0:
            continue
        dx = cy * db[0, idx] - sy * db[1, idx]
        dy = sy * db[0, idx] + cy * db[1, idx]
        with np.errstate(divide="ignore", invalid="ignore"):
            tmin, tmax = np.full(idx.size, -np.inf), np.full(idx.size, np.inf)
            for dd, cc, lo, hi in ((dx, c[0], x0, x1), (dy, c[1], y0, y1), (db[2, idx], c[2], 0.0, z1)):
                iv = 1.0 / dd
                ta, tb = (lo - cc) * iv, (hi - cc) * iv
                tmin, tmax = np.maximum(tmin, np.minimum(ta, tb)), np.minimum(tmax, np.maximum(ta, tb))
        b = best[idx]
        hit = (tmax >= tmin) & (tmax > 0) & (tmin > 0) & (tmin < b)
        best[idx] = np.where(hit, tmin, b)
    best[~np.isfinite(best) | (best > MAX_RANGE_M)] = 0.0
    return np.rint(best * 1000.0).astype(np.uint16).reshape(h, w)


# a 6 m x 6 m room (walls 0.6 m high, 0.1 m thick), the car at its centre facing +x
ROOM = [(2.9, 3.0, -3.0, 3.0, 0.6), (-3.0, -2.9, -3.0, 3.0, 0.6),
        (-3.0, 3.0, 2.9, 3.0, 0.6), (-3.0, 3.0, -3.0, -2.9, 0.6)]


# ---------------------------------------------------------------- producer + bridge (the car's code, CPU backends)
def car_producer(tmp, clock=None, threads=1):
    """host.geo_producer.GeoProducer as the car builds it (block 32x32x10, height_cell 8, back 1, stride 1), with
    the CPU frame core (optionally row-interleaved on `threads`, tools/kv_link/kv_server.RowInterleavedCpuCore:
    the sim_loop fake KV's exact split, equal to the single core) and the CPU rolling backend."""
    from host.geo_producer import GeoProducer
    from host.m7_backend import M7BackendSelector
    from host.m7_frame_native import compile_frame_native
    from host.m7_rolling import compile_rolling
    sel = M7BackendSelector(cpu_frame_library=compile_frame_native(os.path.join(tmp, "f.so")))
    roll = sel.rolling_backend("cpu", cpu_library=compile_rolling(os.path.join(tmp, "r.so")))
    fb = sel.frame_backend("cpu")
    if threads > 1:
        import types
        from tools.kv_link.kv_server import RowInterleavedCpuCore
        fb.lib = types.SimpleNamespace(fpga_m7_depth_frame=RowInterleavedCpuCore(fb.lib.fpga_m7_depth_frame, threads))
    kw = {} if clock is None else {"clock": clock}
    return GeoProducer(fb, roll, depth_unit_m=0.001, height_cell=HEIGHT_CELL, roi=False, **kw)


class CarChain(object):
    def __init__(self, margin=BODY_MARGIN_M, tmp=None, threads=1):
        import host.rolling_nav_layer as L
        tmp = tmp or tempfile.mkdtemp(prefix="crawl_")
        self.now = [10 ** 12]
        self.prod = car_producer(tmp, clock=lambda: self.now[0], threads=threads)
        band = L.HeightBand(ground_z_m=0.0, robot_height_m=ROBOT_HEIGHT_M)
        from host.fpga_clock import CLOCK_ID
        self.bridges = {m: L.RollingLayerBridge(dims=DIMS, voxel_m=VOX, band=band, clock_id=CLOCK_ID,
                                                body_radius_m=ROBOT_RADIUS_M, body_margin_m=m)
                        for m in sorted({0.0, margin})}
        self.T_bc = base_camera_T()
        self.frames = 0

    def frame(self, codes, pose, dt_ns=100_000_000):
        self.now[0] += dt_ns
        self.frames += 1
        T_oc = odom_base_T(*pose) @ self.T_bc
        r = self.prod.process(codes, K_BOARD, depth_stamp_ns=self.frames, acquired_ns=self.now[0] - 1_000_000,
                              T_odom_camera=T_oc, robot_position_m=(pose[0], pose[1], 0.0), calibration_id="crawl")
        d = r.get("delta")
        if d is None:
            return False
        for b in self.bridges.values():
            ok, reason, _ = b.on_delta(d, now_ns=self.now[0])
        return True

    def grid(self, pose, margin=BODY_MARGIN_M):
        """(data [j, i] int, origin_xy) as published to Nav2 (100 / 0 / -1)."""
        g = self.bridges[margin].grid(body_xy=(pose[0], pose[1]), now_ns=self.now[0])
        return np.asarray(g.data).reshape(DIMS[1], DIMS[0]), tuple(float(v) for v in g.origin_xy)


# ---------------------------------------------------------------- Nav2 1.1.20 replica (tests/test_nav_golive_r3.py)
NO_INFO, LETHAL, INSCRIBED = 255, 254, 253
RES = 0.05
GRID_RES = float(np.float32(0.05))
SIZE = int(4 / RES)
R = ROBOT_RADIUS_M
FOOTPRINT = [(math.cos(i * 2 * math.pi / 16) * R, math.sin(i * 2 * math.pi / 16) * R) for i in range(16)]
DWB = dict(vx=(0.0, 0.15), vth=0.4, acc_x=0.3, dec_x=-0.3, acc_th=0.6, dec_th=-0.6, nx=20, nth=20, sim=1.7,
           lin_g=0.05, ang_g=0.025)
EPS = 1e-5


def costmap_origin(rx=0.0, ry=0.0):
    """Costmap2D::updateOrigin of the rolling window (origin_x/y 0, width 4 m) for the robot at (rx, ry)"""
    half = (SIZE - 1 + 0.5) * RES / 2
    return tuple(0.0 + int(((r - half) - 0.0) / RES) * RES for r in (rx, ry))


def world_to_map(wx, wy, origin):
    if wx < origin[0] or wy < origin[1]:
        return None
    mx, my = int((wx - origin[0]) / RES), int((wy - origin[1]) / RES)
    return (mx, my) if mx < SIZE and my < SIZE else None


def master_from_grid(grid, grid_origin, origin):
    """StaticLayer rolling branch: master cell centres looked up in the layer's map (trinary)"""
    m = np.full((SIZE, SIZE), NO_INFO, np.int32)
    n_y, n_x = grid.shape
    mx = np.arange(SIZE)
    wx = origin[0] + (mx + 0.5) * RES
    wy = origin[1] + (mx + 0.5) * RES
    ii = np.floor((wx - grid_origin[0]) / GRID_RES).astype(int)
    jj = np.floor((wy - grid_origin[1]) / GRID_RES).astype(int)
    okx, oky = (ii >= 0) & (ii < n_x), (jj >= 0) & (jj < n_y)
    sub = grid[np.ix_(jj[oky], ii[okx])]
    val = np.where(sub < 0, NO_INFO, np.where(sub >= 100, LETHAL, 0))
    m[np.ix_(np.where(oky)[0], np.where(okx)[0])] = val
    return m


def master_from_costmap_msg(data, w, h):
    """/local_costmap/costmap OccupancyGrid -> master costs for the validity checks (-1 NO_INFO, 100 LETHAL,
    99 INSCRIBED, anything else passes)"""
    a = np.asarray(data, np.int32).reshape(h, w)
    return np.where(a < 0, NO_INFO, np.where(a >= 100, LETHAL, np.where(a == 99, INSCRIBED, 0)))


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


def outline(pose, origin):
    px, py, th = pose
    c, s = math.cos(th), math.sin(th)
    pts = [(px + fx * c - fy * s, py + fx * s + fy * c) for fx, fy in FOOTPRINT]
    cells = []
    for a, b in list(zip(pts[:-1], pts[1:])) + [(pts[-1], pts[0])]:
        ca, cb = world_to_map(*a, origin=origin), world_to_map(*b, origin=origin)
        if ca is None or cb is None:
            return None
        cells += bresenham(ca[0], ca[1], cb[0], cb[1])
    return cells


def _proj(v0, acc, dec, dt, target):
    return min(target, v0 + acc * dt) if v0 < target else max(target, v0 + dec * dt)


def _one_d(lo_lim, hi_lim, acc, dec, n, cur=0.0):
    hi, lo = _proj(cur, acc, dec, DWB['sim'], hi_lim), _proj(cur, acc, dec, DWB['sim'], lo_lim)
    inc = (hi - lo) / (max(2, n) - 1)
    vals, v, zero = [], lo, True
    while not v > hi + EPS:
        vals.append(v)
        if zero and v < 0 < v + inc <= hi + EPS:
            vals.append(0.0)
            zero = False
        v += inc
    return vals


def twists(cur_vx=0.0, cur_th=0.0):
    xs = _one_d(DWB['vx'][0], DWB['vx'][1], DWB['acc_x'], DWB['dec_x'], DWB['nx'], cur_vx)
    ths = _one_d(-DWB['vth'], DWB['vth'], DWB['acc_th'], DWB['dec_th'], DWB['nth'], cur_th)
    return [(vx, vt) for vx in xs for vt in ths if not (vx == 0.0 and vt == 0.0) and vx * vx <= 0.15 ** 2 + EPS]


def trajectory(vx, vt, start=(0.0, 0.0, 0.0), v0=0.0, w0=0.0):
    n = math.ceil(max(abs(vx) * DWB['sim'] / DWB['lin_g'], abs(vt) * DWB['sim'] / DWB['ang_g'])) or 1
    dt = DWB['sim'] / n
    pose, v, w, poses = start, v0, w0, [start]
    for _ in range(n):
        v = _proj(v, DWB['acc_x'], DWB['dec_x'], dt, vx)
        w = _proj(w, DWB['acc_th'], DWB['dec_th'], dt, vt)
        pose = (pose[0] + v * math.cos(pose[2]) * dt, pose[1] + v * math.sin(pose[2]) * dt, pose[2] + w * dt)
        poses.append(pose)
    return poses + [pose]


def verdict(m, origin, vx, vt, start=(0.0, 0.0, 0.0), v0=0.0, w0=0.0):
    poses = trajectory(vx, vt, start, v0, w0)
    for p in poses:
        c = world_to_map(p[0], p[1], origin)
        if c is None:
            return 'OffMap'
        if m[c[1], c[0]] in (LETHAL, INSCRIBED, NO_INFO):
            return 'BaseObstacle'
    for p in poses:
        cells = outline(p, origin)
        if cells is None:
            return 'OffMap'
        for mx, my in cells:
            if m[my, mx] == LETHAL:
                return 'ObstacleFootprint/Obstacle'
            if m[my, mx] == NO_INFO:
                return 'ObstacleFootprint/Unknown'
    return None


def evaluate(m, origin, start=(0.0, 0.0, 0.0), v0=0.0, w0=0.0, cur_for_window=(0.0, 0.0)):
    """{(vx, vt): verdict} over DWB's sample set; summary dict"""
    tw = twists(*cur_for_window)
    v = {k: verdict(m, origin, k[0], k[1], start, v0, w0) for k in tw}
    valid = [k for k, r in v.items() if r is None]
    straight = [k[0] for k in valid if abs(k[1]) < 1e-12 and k[0] > 0]
    rot = [k for k in v if k[0] == 0.0]
    return v, {"n": len(tw), "valid": len(valid),
               "rot_valid": sum(1 for k in rot if v[k] is None), "rot_n": len(rot),
               "max_straight_vx": max(straight) if straight else 0.0,
               "max_vx_any": max([k[0] for k in valid], default=0.0),
               "straight_valid": len(straight),
               "reasons": dict(sorted(__import__('collections').Counter(
                   r for r in v.values() if r is not None).items()))}


def closest(i, j, grid_origin, cx, cy, vox=VOX):
    x0, y0 = grid_origin[0] + i * vox, grid_origin[1] + j * vox
    return math.hypot(max(x0 - cx, cx - (x0 + vox), 0.0), max(y0 - cy, cy - (y0 + vox), 0.0))


def classify_unknown_outline(grid, grid_origin, pose, cells_world, margin=BODY_MARGIN_M):
    """the three classes of section 9.1 for a set of outline cells (master cell centres, world metres):
    1 = unknown inside the body (closest point <= R + margin), 2 = unknown outside, 3 = lethal"""
    out = {1: set(), 2: set(), 3: set()}
    for (wx, wy) in cells_world:
        i, j = int(math.floor((wx - grid_origin[0]) / GRID_RES)), int(math.floor((wy - grid_origin[1]) / GRID_RES))
        v = grid[j, i]
        if v >= 100:
            out[3].add((i, j))
        elif v < 0:
            d = closest(i, j, grid_origin, pose[0], pose[1])
            out[1 if d <= R + margin + 1e-9 else 2].add((i, j))
    return {k: len(s) for k, s in out.items()}


def outline_world_cells(poses, origin):
    s = set()
    for p in poses:
        for mx, my in outline(p, origin):
            s.add((round(origin[0] + (mx + .5) * RES, 4), round(origin[1] + (my + .5) * RES, 4)))
    return s


def art(grid, grid_origin, x_range=(-0.60, 1.20), y_range=(0.60, -0.60), body=(0.0, 0.0)):
    """character picture like the r3 fixture: rows y top -> bottom, '?' unknown, '#' lethal, '.' free, 'B' body"""
    rows = []
    j_top = int(round((y_range[0] - grid_origin[1]) / VOX))
    j_bot = int(round((y_range[1] - grid_origin[1]) / VOX))
    i0 = int(round((x_range[0] - grid_origin[0]) / VOX))
    i1 = int(round((x_range[1] - grid_origin[0]) / VOX))
    bi = int(math.floor((body[0] - grid_origin[0]) / VOX))
    bj = int(math.floor((body[1] - grid_origin[1]) / VOX))
    for j in range(j_top, j_bot - 1, -1):
        row = ''
        for i in range(i0, i1 + 1):
            v = grid[j, i]
            row += 'B' if (i, j) == (bi, bj) else ('?' if v < 0 else ('#' if v >= 100 else '.'))
        rows.append(row)
    return rows
