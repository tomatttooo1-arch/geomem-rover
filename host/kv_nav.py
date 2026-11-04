"""KV260 navigation core for kv_server --nav soft (CTRLPS3, 2026-11-03): CTRLPS1's C++ kv_nav behind CTRLPS2's
NavService interface.

Choice (controller ruling 1): the adapter lives HERE, on the kv_nav side -- ``make_core(field, grid_source)`` returns
a ``KvNavCore`` that drives tools/kv_nav/libkv_nav.so through its C ABI (ctypes). host/kv_nav_service.py is not
changed: it calls set_goal / cancel / on_odom / on_global_path / step exactly as with the mock. Goal and route
arrive in the MAP frame; the C++ core converts them to odom with the map->odom of every NAV_ODOM (and replans on a
map->odom jump > 0.3 m, DESIGN 6.2 #7).

Costmap window (the core plans on an 80 x 80, 0.05 m, odom-lattice window of Nav2 costs):
  RollingGridSource  the KV's own fused map: the B1 grid plane (int8 100 / 0 / -1, the PL grid op with --rolling pl
                     --rolling-grid pl, or its C model with --rolling cpu --rolling-grid cmodel) that RollingService
                     keeps after every INTEGRATE the Nano asked a grid for (``nav_planes``), then on the KV the
                     Nano bridge's own output side (KvWindowLayer = host/pl_grid_layer.PlGridLayer, the very class
                     the Nano producer runs with CAR_PL_GRID=1, same parameters as car_inner.sh): body seed (UNKNOWN
                     under the body circle r 0.30 + 0.05 at the newest NAV_ODOM pose -> FREE), front-sector seed
                     (off unless front_seed_m, as CAR_FRONT_SEED_M), swept clearing (300 s, as SWEPT_CLEAR_S; the
                     tall bit of the aux plane ends it) -- CTRLPS4 -- and the Nav2 StaticLayer + Inflation
                     arithmetic of the B1 cost plane (exact form, tools/b1_costmap/nav2_ref; car values r 0.30,
                     inflation 0.45, scaling 3.0). Window base = the INTEGRATE's window base; cell i of the window
                     covers [(base + i) res, (base + i + 1) res) -> the core's cell_offset 0.5 (CTRLPS3 calibration).
  SimWorldGrid       host tests only (kv_server --nav-sim-world): the same planes cut from a known world map
                     (map yaml + boxes that appear when the car comes near; aux: known cells, every lethal one
                     tall), the same KvWindowLayer + inflation.
  None               refused at startup (a core without a window would hold still forever, reason no_map).

field "pl" (DRV5 2026-11-04): the m5 PL planner core (planner_field_dual, slot 12) builds the potential field through
hw/planner/drv/libplanner_drv.so (planner_drv_kv_nav_field, handed to libkv_nav with kv_nav_set_pl_builder, core
parameter backend=1). The core's own selfcheck (every Nth PL field recomputed in software) stays on. When the driver
library is missing or the core cannot be opened, a warning is printed and the core runs field "soft" (owner 10-26: warn,
never refuse); a hung PL core makes the hook answer NOT_CONVERGED, i.e. libkv_nav builds that field in software.
KV_NAV_PL_DRIVER=<.so> overrides the library path, KV_NAV_PL_WAIT=auto|irq|poll and KV_NAV_PL_DMA=auto|dmaheap:<heap>|dpu
the driver options.

Python 3.8, numpy + ctypes.
"""

import ctypes
import json
import math
import os
import platform
import subprocess
import threading
import time
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
KV_NAV_DIR = ROOT / "tools" / "kv_nav"
ABI_MIN = 3
W = H = 80
RES = 0.05
ANCHOR = 40                      # host/m7_rolling.RollingFusion default window anchor (mx // 2, my // 2)
# C++ kv_nav State -> host.kv_link NAV_STATES name; Reason -> NAV_REASONS name
STATE_NAMES = {0: "idle", 1: "plan", 2: "rotate_to_path", 3: "track", 4: "final_rotate", 5: "succeeded",
               6: "recover", 7: "recover", 8: "paused", 9: "aborted", 10: "canceled"}
REASON_NAMES = {0: "none", 1: "plan_failed", 2: "blocked", 3: "stuck", 4: "no_map", 5: "no_odom",
                6: "goal_rejected", 7: "detour"}
# Car values (robot_sim/fpga_sim/config/rolling_layer_nav2.yaml; host/pl_grid_layer.default_lut)
CAR_INFLATION = dict(resolution=RES, inflation_radius=0.45, cost_scaling_factor=3.0, robot_radius=0.30)
BODY_RADIUS_M, BODY_MARGIN_M = 0.30, 0.05
# CTRLPS4: the Nano bridge's window parameters (tools/car_run/car_inner.sh: swept_clear_s ${SWEPT_CLEAR_S:-300.0},
# front_seed_m only with CAR_FRONT_SEED_M -- default off, owner 10-26 --, front_seed_deg the bridge default 35)
GRID_DEFAULTS = dict(swept_clear_s=300.0, front_seed_m=None, front_seed_deg=35.0)
GRID_CLOCK = "kv_nav_window"     # PlGridLayer checks the planes' clock id; both ends of this file use this one
ROBOT_HEIGHT_M = 0.31           # HeightBand for PlGridLayer's constructor only (the 2-D path never projects)


class _Cmd(ctypes.Structure):
    _fields_ = [("v", ctypes.c_double), ("w", ctypes.c_double), ("state", ctypes.c_int), ("reason", ctypes.c_int),
                ("flags", ctypes.c_uint), ("goal_id", ctypes.c_uint), ("remaining_m", ctypes.c_double),
                ("recoveries", ctypes.c_int), ("plan_status", ctypes.c_int), ("field_source", ctypes.c_int),
                ("path_points", ctypes.c_int), ("lookahead", ctypes.c_double)]


def library_path():
    """KV_NAV_LIB, else tools/kv_nav/build/<host|aarch64-native>/libkv_nav.so -- built with the Makefile
    (``make host``, native g++ on the board too) when it is not there yet (as compile_rolling does)."""
    env = os.environ.get("KV_NAV_LIB")
    if env:
        return Path(env)
    build = Path(os.environ.get("KV_NAV_BUILD", str(KV_NAV_DIR / "build" / ("native-" + platform.machine()))))
    lib = build / "host" / "libkv_nav.so"
    if not lib.exists():
        subprocess.run(["make", "-C", str(KV_NAV_DIR), "BUILD=%s" % build, str(lib)], check=True,
                       stdout=subprocess.DEVNULL)
    return lib


def load_library(path=None):
    lib = ctypes.CDLL(str(path or library_path()))
    lib.kv_nav_abi_version.restype = ctypes.c_int
    abi = lib.kv_nav_abi_version()
    if abi < ABI_MIN:
        raise SystemExit("libkv_nav ABI %d < %d (CTRLPS3 careful mode / world_to_cell): rebuild tools/kv_nav" % (abi, ABI_MIN))
    vp, d, i, u = ctypes.c_void_p, ctypes.c_double, ctypes.c_int, ctypes.c_uint
    sig = {
        "kv_nav_create": ([], vp), "kv_nav_destroy": ([vp], None),
        "kv_nav_set_param": ([vp, ctypes.c_char_p, d], i),
        "kv_nav_set_goal": ([vp, u, d, d, d, d], None), "kv_nav_cancel": ([vp, d], None),
        "kv_nav_set_map_to_odom": ([vp, d, d, d], None),
        "kv_nav_update_costmap": ([vp, ctypes.c_char_p, i, i, d], None),
        "kv_nav_set_global_route": ([vp, ctypes.POINTER(d), i], None),
        "kv_nav_tick": ([vp, d, d, d, d, d, d, i, i, ctypes.POINTER(_Cmd)], None),
        "kv_nav_get_path": ([vp, ctypes.POINTER(d), i], i),
        "kv_nav_last_plan_ms": ([vp], d),
        "kv_nav_world_to_cell": ([vp, d, d, i, i, ctypes.POINTER(i), ctypes.POINTER(i)], None),
        "kv_nav_inflate": ([ctypes.c_char_p, i, i, ctypes.c_char_p, i, ctypes.c_char_p], i),
    }
    for name, (args, res) in sig.items():
        fn = getattr(lib, name)
        fn.argtypes, fn.restype = args, res
    return lib


# ---------------------------------------------------------------------------------------------- costmap planes
class Inflation:
    """Nav2 StaticLayer (trinary, track unknown) + InflationLayer in the exact form the B1 PL core computes
    (tools/b1_costmap/nav2_ref.inflate_exact). ``lib`` "auto": libkv_nav's kv_nav_inflate (C++, ~0.2 ms per window
    on the host); None: the numpy form (shifted maxima, ~6 ms on the host -- reference / tests only). Both are
    checked against the B1 C model's cost plane in tools/kv_nav/sim/cell_offset_probe.py."""

    def __init__(self, lib="auto", **kw):
        from tools.b1_costmap.nav2_ref import InflationParams
        p = InflationParams(**{**CAR_INFLATION, **kw})
        dist, cost = p.cached()
        R = p.cell_radius
        self.R = R
        self.offsets = [(dx, dy, int(cost[abs(dx), abs(dy)])) for dy in range(-R, R + 1) for dx in range(-R, R + 1)
                        if dist[abs(dx), abs(dy)] <= R and int(cost[abs(dx), abs(dy)]) > 0]
        tbl = np.zeros((R + 1, R + 1), np.uint8)
        for i in range(R + 1):
            for j in range(R + 1):
                tbl[i, j] = int(cost[i, j]) if dist[i, j] <= R else 0
        self.table = np.ascontiguousarray(tbl).tobytes()
        self.lib = load_library() if isinstance(lib, str) and lib == "auto" else lib

    def costs(self, grid):
        """grid int8 [H, W] (100 / 0 / -1) -> uint8 [H, W] Nav2 master costs."""
        if self.lib is not None:
            g8 = np.ascontiguousarray(np.asarray(grid, dtype=np.int8).reshape(H, W))
            out = ctypes.create_string_buffer(H * W)
            if self.lib.kv_nav_inflate(g8.tobytes(), W, H, self.table, self.R, out) != 0:
                raise ValueError("kv_nav_inflate refused the sizes")
            return np.frombuffer(out.raw, np.uint8).reshape(H, W).copy()
        return self.costs_numpy(grid)

    def costs_numpy(self, grid):
        g = np.asarray(grid, dtype=np.int16).reshape(H, W)
        lethal = g >= 100
        unk = g == -1
        best = np.zeros((H, W), np.int32)
        if lethal.any():
            R = self.R
            pad = np.zeros((H + 2 * R, W + 2 * R), bool)
            pad[R:R + H, R:R + W] = lethal
            for dx, dy, c in self.offsets:     # a cell takes c when the lethal cell (x - dx, y - dy) exists
                hit = pad[R - dy:R - dy + H, R - dx:R - dx + W]
                np.maximum(best, np.where(hit, c, 0), out=best)
        out = np.zeros((H, W), np.int32)
        out[lethal] = 254
        out[unk] = 255
        take = unk & (best >= 253)             # inflate_unknown false: unknown stays unless inscribed / lethal
        out[take] = best[take]
        rest = ~unk
        out[rest] = np.maximum(out[rest], best[rest])
        return out.astype(np.uint8)


def body_seed(grid, window_base, xy, radius_m=BODY_RADIUS_M + BODY_MARGIN_M):
    """UNKNOWN cells overlapped by the body circle at odom ``xy`` -> FREE (host/rolling_nav_layer.body_cells)."""
    from host.rolling_nav_layer import body_cells
    g = np.array(grid, dtype=np.int8).reshape(H, W)
    if xy is None:
        return g
    mask = body_cells((int(window_base[0]), int(window_base[1]), 0), (W, H, 1), RES, xy, radius_m)
    mask = np.asarray(mask).reshape(H, W)
    g[(g == -1) & mask] = 0
    return g


def aux_from_grid(grid):
    """aux plane (bits 0-1 strict tri-state 0 unknown / 1 free / 2 occupied, bit 2 tall) for a grid without one:
    every lethal cell counted tall (SimWorldGrid: the world's obstacles are walls and boxes)."""
    g = np.asarray(grid, dtype=np.int8).ravel()
    aux = np.zeros(g.size, np.uint8)
    aux[g == 0] = 1
    aux[g == 100] = 2 | 4
    return aux


class KvWindowLayer:
    """CTRLPS4: the Nano bridge's window, computed on the KV -- body seed, front-sector seed, swept clearing.

    Not a re-implementation: it IS host/pl_grid_layer.PlGridLayer (host/rolling_nav_layer.RollingLayerBridge's
    output side: grid() / _grid_body / _apply_swept with the PL tall bit, SweptCells), the class the Nano producer
    runs today with CAR_PL_GRID=1, built with car_inner.sh's parameters. ``window`` = the producer's grid_update
    (on_planes, then grid(pose, now, yaw)) for one INTEGRATE's planes. Equivalence with the Nano bridge on a car
    recording: tools/kv_nav/sim/window_equiv.py (runs/nationals-prelim-20261102/CTRLPS4)."""

    def __init__(self, *, swept_clear_s=None, front_seed_m=None, front_seed_deg=None, body_radius_m=BODY_RADIUS_M,
                 body_margin_m=BODY_MARGIN_M, dims=(W, H, 32)):
        from host.pl_grid_layer import PlGridLayer
        from host.rolling_nav_layer import HeightBand
        p = dict(GRID_DEFAULTS)
        p.update({k: v for k, v in (("swept_clear_s", swept_clear_s), ("front_seed_deg", front_seed_deg))
                  if v is not None})
        p["front_seed_m"] = front_seed_m
        self.params = dict(p, body_radius_m=body_radius_m, body_margin_m=body_margin_m)
        self.layer = PlGridLayer(dims=tuple(dims), voxel_m=RES,
                                 band=HeightBand(ground_z_m=0.0, robot_height_m=ROBOT_HEIGHT_M),
                                 clock_id=GRID_CLOCK, max_map_age_ns=1 << 62, body_radius_m=body_radius_m,
                                 body_margin_m=body_margin_m, front_seed_m=p["front_seed_m"],
                                 front_seed_deg=p["front_seed_deg"], swept_clear_s=p["swept_clear_s"])
        self.seq = 0
        self.counts = Counter()

    def window(self, base, grid, aux, now_ns, pose):
        """One INTEGRATE's planes -> int8 [H, W] window (100 / 0 / -1), or None. ``pose`` (x, y[, yaw]) odom or None
        (strict grid, as the bridge without a pose); ``now_ns`` the time of this window (swept clearing clock)."""
        from host.pl_grid_layer import PlGridPlanes
        if aux is None:
            aux = aux_from_grid(grid)
            self.counts["aux_missing"] += 1
        self.seq += 1
        planes = PlGridPlanes(0, np.asarray(grid, dtype=np.int8).ravel(), np.asarray(aux, dtype=np.uint8).ravel(),
                              None, body=False)
        ok, why = self.layer.on_planes(epoch=0, delta_seq=self.seq, window_base=tuple(int(v) for v in base),
                                       acquired_ns=int(now_ns), clock_id=GRID_CLOCK, planes=planes)
        if not ok:
            self.counts["rejected:" + why] += 1
            return None
        xy = None if pose is None else (float(pose[0]), float(pose[1]))
        yaw = float(pose[2]) if pose is not None and len(pose) > 2 and pose[2] is not None else None
        fields = self.layer.grid(body_xy=xy, now_ns=int(now_ns), body_yaw=yaw)
        if fields is None:
            self.counts["no_grid"] += 1
            return None
        self.counts["windows"] += 1
        return np.asarray(fields.data, dtype=np.int8).reshape(H, W)

    def describe(self):
        c = self.layer.counts
        return {"params": self.params, "counts": dict(self.counts),
                "swept_cleared_last": int(getattr(self.layer, "last_swept_cleared", 0)),
                "swept_cleared_total": int(c["swept_cleared_cells"]),
                "swept_evidence_total": int(c["swept_evidence_cells"]),
                "front_seeded": int(c["front_seeded"]), "front_seed_no_yaw": int(c["front_seed_no_yaw"])}


class RollingGridSource:
    """The KV's own fused map (RollingService.nav_planes, newest OK B1 grid + aux planes) -> KvWindowLayer (body
    seed, front seed, swept clearing; CTRLPS4) -> inflation."""
    name = "rolling"

    def __init__(self, service, inflation=None, grid_params=None):
        self.svc = service
        self.infl = inflation or Inflation()
        self.win = KvWindowLayer(**(grid_params or {}))
        self._seen = None
        self.counts = Counter()
        self.last_ms = 0.0
        self.last_window_ms = 0.0
        self.last_window = None     # int8 [H, W] window before inflation (tests / diagnosis)

    def latest(self, now_s, pose_xy, pose_yaw=None):
        planes = getattr(self.svc, "nav_planes", None)
        if planes is None:
            return None
        if self._seen is not None and self._seen[0] is planes:
            return self._seen[1]
        t, base, grid = planes[:3]
        aux = planes[3] if len(planes) > 3 else None
        t0 = time.monotonic()
        pose = None if pose_xy is None else (pose_xy[0], pose_xy[1], pose_yaw)
        win = self.win.window(base, grid, aux, int(round(float(now_s) * 1e9)), pose)
        t1 = time.monotonic()
        if win is None:
            self._seen = (planes, None)
            self.counts["window_refused"] += 1
            return None
        cost = self.infl.costs(win)
        self.last_window_ms = (t1 - t0) * 1e3
        self.last_ms = (time.monotonic() - t0) * 1e3
        self.last_window = win
        out = (cost, int(base[0]), int(base[1]), float(t))
        self._seen = (planes, out)
        self.counts["planes"] += 1
        return out

    def describe(self):
        return {"name": self.name, "counts": dict(self.counts), "last_inflate_ms": round(self.last_ms, 3),
                "last_window_ms": round(self.last_window_ms, 3), "window": self.win.describe()}


class SimWorldGrid:
    """HOST TESTS ONLY (kv_server --nav-sim-world PATH). JSON: {"map_yaml": path (trinary pgm, any resolution that
    is a multiple of 0.05 and lattice-aligned), "boxes": [{"x0","x1","y0","y1" (odom = map metres),
    "appear_within_m": d or absent (always), "id"}], "hz": 10}. The window follows the car like the rolling map
    (base = floor(pose / 0.05) - 40); every cell is known (the world map's own unknown stays unknown)."""

    name = "sim_world"

    def __init__(self, path, inflation=None, grid_params=None):
        spec = json.loads(Path(path).read_text())
        self.spec = spec
        self.world, self.wbase = load_world(spec["map_yaml"])   # int8 [y][x] on the 0.05 lattice, base cell
        self.boxes = [dict(b, shown_t=None) for b in spec.get("boxes", ())]
        self.period = 1.0 / float(spec.get("hz", 10.0))
        self.infl = inflation or Inflation()
        self.win = KvWindowLayer(**(grid_params or {}))
        self._last = None
        self.counts = Counter()
        self.events = []

    def world_with_boxes(self, shown_only=True):
        w = self.world.copy()
        for b in self.boxes:
            if shown_only and b["shown_t"] is None:
                continue
            x0, x1 = int(math.floor(b["x0"] / RES)) - self.wbase[0], int(math.floor(b["x1"] / RES)) - self.wbase[0]
            y0, y1 = int(math.floor(b["y0"] / RES)) - self.wbase[1], int(math.floor(b["y1"] / RES)) - self.wbase[1]
            w[max(0, y0):y1 + 1, max(0, x0):x1 + 1] = 100
        return w

    def latest(self, now_s, pose_xy, pose_yaw=None):
        if pose_xy is None:
            return None
        if self._last is not None and now_s - self._last[3] < self.period:
            return self._last
        for b in self.boxes:
            if b["shown_t"] is None:
                d = b.get("appear_within_m")
                cx, cy = 0.5 * (b["x0"] + b["x1"]), 0.5 * (b["y0"] + b["y1"])
                if d is None or math.hypot(pose_xy[0] - cx, pose_xy[1] - cy) <= float(d):
                    b["shown_t"] = now_s
                    self.events.append({"box": b.get("id"), "t": now_s, "car": [round(v, 3) for v in pose_xy]})
                    self.counts["box_shown"] += 1
        base = (int(math.floor(pose_xy[0] / RES)) - ANCHOR, int(math.floor(pose_xy[1] / RES)) - ANCHOR)
        world = self.world_with_boxes()
        g = np.full((H, W), -1, np.int8)
        hy, hx = world.shape
        ys, xs = base[1] - self.wbase[1], base[0] - self.wbase[0]
        sy0, sy1, sx0, sx1 = max(0, ys), min(hy, ys + H), max(0, xs), min(hx, xs + W)
        if sy0 < sy1 and sx0 < sx1:
            g[sy0 - ys:sy1 - ys, sx0 - xs:sx1 - xs] = world[sy0:sy1, sx0:sx1]
        win = self.win.window((base[0], base[1], 0), g, aux_from_grid(g), int(round(float(now_s) * 1e9)),
                              (pose_xy[0], pose_xy[1], pose_yaw))
        if win is None:
            return self._last
        cost = self.infl.costs(win)
        self._last = (cost, base[0], base[1], float(now_s))
        self.counts["windows"] += 1
        return self._last

    def describe(self):
        return {"name": self.name, "counts": dict(self.counts), "box_events": self.events,
                "window": self.win.describe()}


def load_world(map_yaml):
    """ROS map yaml (trinary, P2/P5 pgm) -> (int8 [y][x] 100 / 0 / -1 on the 0.05 odom lattice, (base_x, base_y))."""
    import yaml
    y = yaml.safe_load(Path(map_yaml).read_text())
    img = read_pgm(Path(map_yaml).parent / y["image"])
    res = float(y["resolution"])
    k = int(round(res / RES))
    if abs(k * RES - res) > 1e-9:
        raise ValueError("map resolution %g is not a multiple of %g" % (res, RES))
    ox, oy = float(y["origin"][0]), float(y["origin"][1])
    bx, by = int(round(ox / RES)), int(round(oy / RES))
    if abs(bx * RES - ox) > 1e-9 or abs(by * RES - oy) > 1e-9:
        raise ValueError("map origin not on the 0.05 lattice")
    p = img.astype(np.float64) / 255.0
    occ = 1.0 - p if not y.get("negate", 0) else p
    g = np.zeros(img.shape, np.int8)
    g[occ > float(y["occupied_thresh"])] = 100
    g[(occ >= float(y["free_thresh"])) & (occ <= float(y["occupied_thresh"]))] = -1
    g = np.flipud(g)                                 # image row 0 = top (max y)
    g = np.repeat(np.repeat(g, k, axis=0), k, axis=1)
    return g, (bx, by)


def read_pgm(path):
    data = Path(path).read_bytes()
    if data[:2] == b"P2":
        toks = [t for line in data.decode().splitlines() if not line.startswith("#") for t in line.split()]
        w, h, _mx = int(toks[1]), int(toks[2]), int(toks[3])
        return np.array([int(v) for v in toks[4:4 + w * h]], np.uint8).reshape(h, w)
    if data[:2] == b"P5":
        parts, i = [], 2
        while len(parts) < 3:
            while data[i:i + 1].isspace():
                i += 1
            if data[i:i + 1] == b"#":
                i = data.index(b"\n", i) + 1
                continue
            j = i
            while not data[j:j + 1].isspace():
                j += 1
            parts.append(int(data[i:j]))
            i = j
        w, h = parts[0], parts[1]
        return np.frombuffer(data[i + 1:i + 1 + w * h], np.uint8).reshape(h, w)
    raise ValueError("not a pgm: %s" % path)


# ---------------------------------------------------------------------------------------------- the core
class KvNavCore:
    """NavService core (host/kv_nav_service.py interface) over libkv_nav."""

    name = "kv_nav"

    def __init__(self, lib, *, field="soft", grid_source=None, params=None, clock=time.monotonic):
        self.lib, self.field, self.grid = lib, field, grid_source
        self.clock = clock
        self.h = lib.kv_nav_create()
        self.params = {"cell_offset": 0.5}              # B1 lattice (CTRLPS3 calibration); see module doc
        self.params.update(params or {})
        for k, v in self.params.items():
            if lib.kv_nav_set_param(self.h, k.encode(), float(v)) != 0:
                raise ValueError("kv_nav: unknown parameter %r" % k)
        self.goal_id = 0            # the wire goal id (64-bit)
        self._iid = 0               # the core's 32-bit id
        self.odom = None
        self.route_goal = None      # goal id the last route was for
        self.map_stamp = None
        self.counts = Counter()
        self._lock = threading.Lock()
        self._cmd = _Cmd()
        self.last = None

    def __del__(self):
        try:
            self.lib.kv_nav_destroy(self.h)
        except Exception:
            pass

    def set_goal(self, goal_id, x, y, yaw):
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            self.counts["goal_rejected:nonfinite"] += 1
            return False, "goal_rejected"
        self._iid = (self._iid + 1) & 0xFFFFFFFF or 1
        self.goal_id = goal_id
        self.route_goal = None
        self.lib.kv_nav_set_global_route(self.h, None, 0)
        self.lib.kv_nav_set_goal(self.h, self._iid, float(x), float(y), float(yaw), self.clock())
        self.counts["goal"] += 1
        return True, "none"

    def cancel(self, goal_id):
        if goal_id == self.goal_id:
            self.lib.kv_nav_cancel(self.h, self.clock())
            self.counts["cancel"] += 1

    def on_odom(self, odom):
        self.odom = odom
        if odom.get("map_valid", True):
            mx, my, myaw = odom["map_odom"]
            self.lib.kv_nav_set_map_to_odom(self.h, float(mx), float(my), float(myaw))

    def on_global_path(self, goal_id, path_id, points):
        if goal_id != self.goal_id:
            return
        flat = [float(c) for p in points for c in p[:2]]
        arr = (ctypes.c_double * max(1, len(flat)))(*flat)
        self.lib.kv_nav_set_global_route(self.h, arr, len(flat) // 2)
        self.route_goal = goal_id
        self.counts["route"] += 1

    def _feed_map(self, now_s):
        if self.grid is None:
            return
        o = self.odom
        ok = o is not None and o.get("odom_valid", True)
        xy = tuple(o["odom_pose"][:2]) if ok else None
        yaw = float(o["odom_pose"][2]) if ok else None
        g = self.grid.latest(now_s, xy, yaw)
        if g is None:
            return
        cost, bx, by, stamp = g
        if stamp == self.map_stamp:
            return
        self.map_stamp = stamp
        buf = np.ascontiguousarray(cost, np.uint8).tobytes()
        self.lib.kv_nav_update_costmap(self.h, buf, bx, by, stamp)
        self.counts["map"] += 1
        g3 = getattr(self, "_pln_g3", None)
        if g3 is not None:
            g3.feed(now_s, buf, bx, by)

    def step(self, now_s, hold=False):
        """hold (CTRLPS6, link lost): tick as teleop in hand -- PAUSED, zero, no stuck time, replan on release."""
        self._feed_map(now_s)
        o = self.odom
        base = {"goal_id": self.goal_id, "need_path": bool(self.goal_id) and self.route_goal != self.goal_id}
        if o is None:
            return dict(base, v=0.0, w=0.0, state="plan" if self.goal_id else "idle",
                        reason="no_odom" if self.goal_id else "none", remaining_m=0.0, path_len_m=0.0,
                        recoveries=0, path_points=0)
        x, y, yaw = o["odom_pose"]
        c = self._cmd
        self.lib.kv_nav_tick(self.h, float(now_s), float(x), float(y), float(yaw), float(o["v"]), float(o["w"]),
                             1 if o["permit"] else 0, 1 if (o.get("teleop") or hold) else 0, ctypes.byref(c))
        state = STATE_NAMES.get(c.state, "aborted")
        self.counts["state:" + state] += 1
        if hold:
            self.counts["hold_ticks"] += 1
        if c.flags & 256:
            self.counts["careful_ticks"] += 1
        if c.flags & 4:
            self.counts["blocked_ticks"] += 1
        rem = max(0.0, c.remaining_m)
        out = dict(base, v=c.v, w=c.w, state=state, reason=REASON_NAMES.get(c.reason, "plan_failed"),
                   remaining_m=rem, path_len_m=rem, recoveries=c.recoveries, path_points=c.path_points,
                   flags=c.flags, plan_status=c.plan_status)
        self.last = out
        return out

    def path_odom(self, cap=2048):
        buf = (ctypes.c_double * (2 * cap))()
        n = self.lib.kv_nav_get_path(self.h, buf, cap)
        return [(buf[2 * i], buf[2 * i + 1]) for i in range(min(n, cap))]

    def world_to_cell(self, x, y, base_x, base_y):
        cx, cy = ctypes.c_int(), ctypes.c_int()
        self.lib.kv_nav_world_to_cell(self.h, float(x), float(y), int(base_x), int(base_y), ctypes.byref(cx), ctypes.byref(cy))
        return cx.value, cy.value

    def describe(self):
        d = {"name": self.name, "field": self.field, "params": self.params, "counts": dict(self.counts),
             "last_plan_ms": round(self.lib.kv_nav_last_plan_ms(self.h), 3)}
        if self.grid is not None and hasattr(self.grid, "describe"):
            d["grid"] = self.grid.describe()
        d["pln"] = self.pln_stats() or "m5"
        return d

    def pln_stats(self):
        """PLNCHAIN0: GMAP3 planner counters (path fetch + global map), None when the m5 driver is in use."""
        g3 = getattr(self, "_pln_g3", None)
        return g3.stats() if g3 is not None else None


def grid_params_from(grid_params=None):
    """KvWindowLayer parameters: GRID_DEFAULTS < ``grid_params`` (kv_server --nav-swept-clear-s / --nav-front-seed-m;
    None = not given) < environment KV_NAV_GRID='{"swept_clear_s": 300, "front_seed_m": 0.65, "front_seed_deg": 35}'.
    swept_clear_s 0 / front_seed_m 0 or null = off."""
    p = dict(GRID_DEFAULTS)
    p.update({k: v for k, v in (grid_params or {}).items() if v is not None})
    env = os.environ.get("KV_NAV_GRID")
    if env:
        p.update(json.loads(env))
    unknown = set(p) - set(GRID_DEFAULTS)
    if unknown:
        raise SystemExit("KV_NAV_GRID / grid_params: unknown key(s) %s" % sorted(unknown))
    return p


PLANNER_DRV_DIR = ROOT / "hw" / "planner" / "drv"


PLANNER_G3_DIR = ROOT / "hw" / "planner" / "drv_gmap3_chain"
G3_STATS = ("field_calls", "field_ok", "field_fail", "path_calls", "path_found", "path_nopath", "path_other",
            "path_nomark", "path_match", "path_mismatch", "last_header", "last_npts", "last_start", "last_end",
            "gmap_enabled", "merge_calls", "merge_ok", "merge_fail", "merge_dropped", "scan_calls", "scan_ok",
            "last_bbox0", "last_bbox1", "last_path_core_ns")


class G3Planner:
    """PLNCHAIN0 (PLN=gmap3): the GMAP3 planner core (global map + in-core path fetch) through
    hw/planner/drv_gmap3_chain/libplanner_drv_gmap3.so. The field hook is planner_g3_kv_nav_field (same contract as
    the m5 hook; the path fetched in the core is compared with the PS extraction on the same field and counted);
    every new cost window is MERGEd into the 400x400 global map at most every merge_s, a SCAN every scan_s."""

    def __init__(self, dl, h, merge_s, scan_s):
        self.dl, self.h, self.merge_s, self.scan_s = dl, h, merge_s, scan_s
        self._t_merge = self._t_scan = -1e18
        self._buf = (ctypes.c_uint64 * len(G3_STATS))()

    def feed(self, now_s, cost_bytes, bx, by):
        if self.merge_s >= 0 and now_s - self._t_merge >= self.merge_s:
            self._t_merge = now_s
            self.dl.planner_g3_merge(self.h, cost_bytes, int(bx), int(by))
        if self.scan_s > 0 and now_s - self._t_scan >= self.scan_s:
            self._t_scan = now_s
            self.dl.planner_g3_scan(self.h)

    def stats(self):
        self.dl.planner_g3_stats(self.h, self._buf, len(G3_STATS))
        return dict(zip(G3_STATS, (int(v) for v in self._buf)))


def attach_g3_field(core, *, driver=None):
    """PLN=gmap3: GMAP3 driver + chain shim. Returns (True, describe) or (False, why); the caller falls back to m5."""
    path = driver or os.environ.get("KV_NAV_PL_DRIVER_GMAP3") or str(
        PLANNER_G3_DIR / "build" / ("aarch64" if platform.machine() == "aarch64" else "host") / "libplanner_drv_gmap3.so")
    try:
        dl = ctypes.CDLL(path)
    except OSError as exc:
        return False, "gmap3 planner library %s: %s" % (path, exc)
    vp, cp, i = ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int
    dl.planner_g3_open.argtypes, dl.planner_g3_open.restype = [cp, cp], vp
    dl.planner_g3_open_error.argtypes, dl.planner_g3_open_error.restype = [], cp
    dl.planner_g3_describe.argtypes, dl.planner_g3_describe.restype = [vp], cp
    dl.planner_g3_merge.argtypes, dl.planner_g3_merge.restype = [vp, cp, i, i], i
    dl.planner_g3_scan.argtypes, dl.planner_g3_scan.restype = [vp], i
    dl.planner_g3_stats.argtypes, dl.planner_g3_stats.restype = [vp, ctypes.POINTER(ctypes.c_uint64), i], i
    h = dl.planner_g3_open(os.environ.get("KV_NAV_PL_WAIT", "auto").encode(),
                           os.environ.get("KV_NAV_PL_DMA", "auto").encode())
    if not h:
        return False, "gmap3 planner core: %s" % dl.planner_g3_open_error().decode(errors="replace")
    core.lib.kv_nav_set_pl_builder.argtypes, core.lib.kv_nav_set_pl_builder.restype = [vp, vp, vp], None
    if core.lib.kv_nav_set_param(core.h, b"backend", 1.0) != 0:
        return False, "libkv_nav has no backend parameter"
    core.lib.kv_nav_set_pl_builder(core.h, ctypes.cast(dl.planner_g3_kv_nav_field, vp), h)
    core._pl_driver = (dl, h)
    core._pln_g3 = G3Planner(dl, h, float(os.environ.get("KV_NAV_PLN_MERGE_S", "0.5")),
                             float(os.environ.get("KV_NAV_PLN_SCAN_S", "5")))
    core.params["backend"] = 1
    return True, dl.planner_g3_describe(h).decode(errors="replace")


def attach_pl_field(core, *, driver=None):
    """--nav pl (DRV5): hand the PL planner core driver to the C++ core. Returns (True, describe) or (False, why)."""
    path = driver or os.environ.get("KV_NAV_PL_DRIVER") or str(
        PLANNER_DRV_DIR / "build" / ("aarch64" if platform.machine() == "aarch64" else "host") / "libplanner_drv.so")
    try:
        dl = ctypes.CDLL(path)
    except OSError as exc:
        return False, "planner driver library %s: %s" % (path, exc)
    vp, cp = ctypes.c_void_p, ctypes.c_char_p
    dl.planner_drv_open.argtypes, dl.planner_drv_open.restype = [cp, cp], vp
    dl.planner_drv_open_error.argtypes, dl.planner_drv_open_error.restype = [], cp
    dl.planner_drv_describe.argtypes, dl.planner_drv_describe.restype = [vp], cp
    h = dl.planner_drv_open(os.environ.get("KV_NAV_PL_WAIT", "auto").encode(),
                            os.environ.get("KV_NAV_PL_DMA", "auto").encode())
    if not h:
        return False, "planner core: %s" % dl.planner_drv_open_error().decode(errors="replace")
    core.lib.kv_nav_set_pl_builder.argtypes, core.lib.kv_nav_set_pl_builder.restype = [vp, vp, vp], None
    if core.lib.kv_nav_set_param(core.h, b"backend", 1.0) != 0:
        return False, "libkv_nav has no backend parameter"
    core.lib.kv_nav_set_pl_builder(core.h, ctypes.cast(dl.planner_drv_kv_nav_field, vp), h)
    core._pl_driver = (dl, h)       # keep the library and the handle alive as long as the core
    core.params["backend"] = 1
    return True, dl.planner_drv_describe(h).decode(errors="replace")


def make_core(field="soft", grid_source=None, *, params=None, lib=None, grid_params=None):
    """kv_server --nav soft --nav-core kv_nav (host/kv_nav_service.build_core). ``grid_source``: a RollingService
    (wrapped in RollingGridSource; it must answer the B1 grid op, i.e. --rolling-grid cmodel|pl), an object with
    ``latest(now_s, pose_xy)`` (SimWorldGrid); None is refused (a core without a window could never move).
    Environment KV_NAV_PARAMS='{"name": value, ...}' overrides core parameters (bench use). ``grid_params``: the
    window layer (CTRLPS4, see grid_params_from); a source object passed in (SimWorldGrid) keeps its own."""
    if field not in ("soft", "pl"):
        raise SystemExit("--nav %s: unknown potential-field source (soft | pl). Not starting." % field)
    if grid_source is None:
        raise SystemExit("--nav soft --nav-core kv_nav: no costmap window source (needs --rolling cpu --rolling-grid "
                         "cmodel or --rolling pl --rolling-grid pl, the Nano with CAR_PL_GRID=1; host tests: "
                         "--nav-sim-world). Not starting.")
    if not hasattr(grid_source, "latest"):
        if not getattr(grid_source, "grid_capable", False):
            raise SystemExit("--nav soft --nav-core kv_nav: the rolling service does not answer the B1 grid op "
                             "(needs --rolling cpu --rolling-grid cmodel or --rolling pl --rolling-grid pl on a GRID "
                             "bitstream). Not starting.")
        grid_source = RollingGridSource(grid_source, grid_params=grid_params_from(grid_params))
    p = dict(params or {})
    env = os.environ.get("KV_NAV_PARAMS")
    if env:
        p.update(json.loads(env))
    core = KvNavCore(lib or load_library(), field=field, grid_source=grid_source, params=p)
    if field == "pl":
        # PLNCHAIN0: KV_NAV_PLN=gmap3 -> GMAP3 core (global map + path fetch); any failure -> WARN, m5 driver
        pln = os.environ.get("KV_NAV_PLN", "m5")
        ok = False
        if pln == "gmap3":
            ok, note = attach_g3_field(core)
            if ok:
                print(json.dumps({"kv_nav_field": "pl", "pln": "gmap3", "driver": note}), flush=True)
                return core
            print(json.dumps({"warning": "KV_NAV_PLN=gmap3: %s; m5 planner driver instead" % note}), flush=True)
        elif pln != "m5":
            print(json.dumps({"warning": "KV_NAV_PLN=%s unknown (m5 | gmap3); m5 planner driver" % pln}), flush=True)
        ok, note = attach_pl_field(core)
        if ok:
            print(json.dumps({"kv_nav_field": "pl", "driver": note}), flush=True)
        else:
            core.field = "soft"
            print(json.dumps({"warning": "--nav pl: %s; potential field in software (--nav soft)" % note}), flush=True)
    return core


__all__ = ["make_core", "KvNavCore", "RollingGridSource", "KvWindowLayer", "GRID_DEFAULTS", "grid_params_from",
           "aux_from_grid", "SimWorldGrid", "Inflation", "body_seed", "load_library",
           "load_world", "STATE_NAMES", "REASON_NAMES"]
