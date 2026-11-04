"""NN-CAM2 (2026-11-03): kv_server's local frame source -- depth from kv_cam's shared-memory ring on this board.

With the Nano gone the camera is on the KV260 and kv_cam (tools/kv_cam, NN-CAM1) publishes every frameset into the
ring /dev/shm/kv_cam_v1. ``kv_server --frame-source kvcam`` reads it here instead of waiting for the Nano's REQ:

* LocalDepthSource (own thread): newest depth frame from the ring (tools/kv_cam/cam_ring.py, the protocol's own
  reader -- the seqlock is done in C; no second implementation), then the very request bytes the Nano would send
  (host.kv_link.encode_request with the same per-frame grid rule, host.m7_grid_policy.FollowCameraGrid), into the
  server's one-slot mailbox. The worker serves it with the unchanged KvLinkServer._serve (decode -> convert -> core),
  so the geometry core's input is made by the same code on both paths; only the answer is not sent (no Nano):
  KvLinkServer hands it to ``local_sink``.
* LocalDepthChain (``local_sink``, worker thread): the rolling map INTEGRATE (+ the B1 GRID op when the rolling
  service answers it) through host.kv_rolling.RemotePlRollingBackend over an in-process link (the Nano's own
  client: same CONFIG / RESET / HELD INTEGRATE bytes, no UDP), then stamps. The KV navigation core (kv_server --nav)
  ticks at 20 Hz on its own thread; StampedNavCore notes the first tick that used each frame's grid ("field done").
* Stamps: one JSONL row per frame, camera stamps in their own columns, every other instant CLOCK_MONOTONIC ns
  (time.monotonic_ns == kv_cam's mono_ns on one board). tools/nn_cam2/chain_report.py reads them.

Bench parameters (--local-params JSON, all optional): the camera pose and intrinsics the Nano's producer would have
used. Defaults: intrinsics from the ring header when kv_cam filled them (else the JSON must give them; a warning
and Gemini 2 nominal values otherwise), depth unit from the ring, a camera 0.30 m above the odom origin looking
along +x (car still: the KV-2 bench). The real pose source (kv_pose) is NN-INT1's.

Never refuses: no ring / writer gone / a frame that cannot be built -> stderr warning (rate-limited), counted, go on.
"""

from collections import Counter, OrderedDict
import copy
import json
import math
import os
import sys
import threading
import time

import numpy as np

from host.kv_link import (IDENTITY_FIELDS, NAV_GOAL, NAV_ODOM, NAV_OP_GOAL, NAV_ODOM_MAP_VALID,
                          NAV_ODOM_ODOM_VALID, NAV_ODOM_PERMIT, encode_nav_goal, encode_nav_odom, encode_request)
from host.m7_grid_policy import FollowCameraGrid

# = host/geo_producer.py VOXEL_M / DEFAULT_SHAPE / DEFAULT_HEIGHT_CELL (the car producer's grid; not imported:
# geo_producer pulls the Nano's runtime). tests/test_kv_local_frames.py pins the equality.
VOXEL_M = 0.05
DEFAULT_SHAPE = (32, 32, 10)
DEFAULT_HEIGHT_CELL = 7
ROBOT_HEIGHT_M = 0.31          # = host/kv_nav.ROBOT_HEIGHT_M
# camera optical axes in odom: x right -> -y, y down -> -z, z forward -> +x
DEFAULT_CAMERA_TO_WORLD = [[0.0, 0.0, 1.0, 0.0], [-1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.30],
                           [0.0, 0.0, 0.0, 1.0]]
NOMINAL_INTRINSICS = {"fx": 517.0, "fy": 517.0, "cx": 319.5, "cy": 239.5}   # used only with a warning
FLAG_HAS_DEPTH = 2
STAMP_VERSION = 1


def _warn(state, key, msg, every_s=5.0):
    now = time.monotonic()
    if now - state.get(key, -1e9) >= every_s:
        state[key] = now
        print(json.dumps({"warning": "kv_server local: " + msg}), file=sys.stderr, flush=True)


class StampLog:
    """Thread-safe JSONL writer; the first row pairs CLOCK_MONOTONIC with CLOCK_REALTIME (to place SDK host stamps)."""

    def __init__(self, path):
        self.fp = open(path, "w") if path else None
        self.lock = threading.Lock()
        self.write({"ev": "clock", "v": STAMP_VERSION, "mono_ns": time.monotonic_ns(), "realtime_ns": time.time_ns(),
                    "pid": os.getpid()})

    def write(self, row):
        if self.fp is None:
            return
        with self.lock:
            self.fp.write(json.dumps(row, separators=(",", ":")) + "\n")
            self.fp.flush()

    def close(self):
        if self.fp is not None:
            with self.lock:
                self.fp.close()
                self.fp = None


class FrameParams:
    """Per-frame request constants (what the Nano's producer would put in REQ)."""

    def __init__(self, cfg=None, ring=None):
        cfg = dict(cfg or {})
        self.notes = []
        t = np.asarray(cfg.get("camera_to_world", DEFAULT_CAMERA_TO_WORLD), dtype=np.float64)
        if t.shape != (4, 4):
            raise ValueError("camera_to_world must be 4x4")
        self.camera_to_world = t
        intr = cfg.get("intrinsics")
        if intr is None and ring is not None and ring.intrinsics() is not None:
            intr = ring.intrinsics()
            self.notes.append("intrinsics from the kv_cam ring header")
        if intr is None:
            intr = dict(NOMINAL_INTRINSICS)
            self.notes.append("WARNING nominal intrinsics (no --local-params intrinsics, ring has none)")
        self.k = np.array([[intr["fx"], 0.0, intr["cx"]], [0.0, intr["fy"], intr["cy"]], [0.0, 0.0, 1.0]],
                          dtype=np.float64)
        unit = cfg.get("depth_unit_m")
        if unit is None:
            unit = (ring.depth_scale_mm / 1000.0) if ring is not None else 0.001
        self.unit = np.float32(unit)
        self.voxel_m = float(cfg.get("voxel_m", VOXEL_M))
        self.shape = tuple(int(v) for v in cfg.get("shape", DEFAULT_SHAPE))
        self.policy = FollowCameraGrid(self.voxel_m, self.shape, back_cells=int(cfg.get("back_cells", 1)),
                                       height_cell=int(cfg.get("height_cell", DEFAULT_HEIGHT_CELL)))
        ident = {"stream_instance": "kvcam", "clock_id": "kv_mono", "calibration_id": "bench", "pose_epoch": "0"}
        ident.update(cfg.get("identity", {}))
        self.identity = {k: str(ident[k]) for k in IDENTITY_FIELDS}
        self.ground_z_m = float(cfg.get("ground_z_m", 0.0))
        self.robot_height_m = float(cfg.get("grid_robot_height_m", ROBOT_HEIGHT_M))
        self.nav_goal = cfg.get("nav_goal")            # [x, y, yaw] in odom, or None (no goal: field not built)
        # derived once (the bench pose is fixed; kv_pose will make these per frame)
        self.inverse_k = np.ascontiguousarray(np.linalg.inv(self.k), dtype=np.float32)
        self.world_camera = np.ascontiguousarray(t[:3, :], dtype=np.float32)
        self.grid = self.policy.grid_for(t)
        self.origin = np.asarray(self.grid.origin_m, dtype=np.float64)
        yaw = math.atan2(t[1, 2], t[0, 2])             # optical axis heading
        self.odom_pose = (float(t[0, 3]), float(t[1, 3]), yaw)
        self.base_camera = t                           # CHASSIS0: the cfg pose = camera in base_link at the origin

    def at_pose(self, x, y, yaw):
        """CHASSIS0: these parameters with the car at odometry pose (x, y, yaw): camera_to_world = base pose x
        the configured (base_link -> camera) pose. at_pose(0, 0, 0) gives exactly the configured values."""
        c, s = math.cos(yaw), math.sin(yaw)
        base = np.array([[c, -s, 0.0, x], [s, c, 0.0, y], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        t = base @ self.base_camera
        out = copy.copy(self)
        out.camera_to_world = t
        out.world_camera = np.ascontiguousarray(t[:3, :], dtype=np.float32)
        out.grid = self.policy.grid_for(t)
        out.origin = np.asarray(out.grid.origin_m, dtype=np.float64)
        out.odom_pose = (float(x), float(y), float(yaw))
        return out

    def request(self, frame_id, capture_ns, codes, width, height):
        nx, ny, nz = self.shape
        return encode_request(frame_id=int(frame_id), capture_ns=int(capture_ns), pose_ns=int(capture_ns),
                              identity=self.identity, width=width, height=height, nx=nx, ny=ny, nz=nz,
                              unit_f32=self.unit, voxel=self.voxel_m, origin=self.origin,
                              inverse_k=self.inverse_k, world_camera=self.world_camera, codes=codes, depth_crc=0)

    def describe(self):
        return {"camera_to_world": self.camera_to_world.tolist(), "k": self.k.tolist(), "unit_m": float(self.unit),
                "voxel_m": self.voxel_m, "shape": list(self.shape), "origin": self.origin.tolist(),
                "identity": self.identity, "ground_z_m": self.ground_z_m, "robot_height_m": self.robot_height_m,
                "nav_goal": self.nav_goal, "notes": self.notes}


class _Ring:
    """tools/kv_cam/cam_ring.py CamRing (+ the header fields this source needs)."""

    def __init__(self, name, path, lib_path=None):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        sys.path.insert(0, os.path.join(root, "tools", "kv_cam"))
        try:
            from cam_ring import CamRing
        finally:
            sys.path.pop(0)
        self.r = CamRing.open(name, path=path, lib_path=lib_path)
        import ctypes
        import struct
        hdr = ctypes.string_at(self.r._lib.kvcam_header(self.r._h), 512)
        fx, fy, cx, cy = struct.unpack_from("<4f", hdr, 176)
        self.intr_valid = struct.unpack_from("<I", hdr, 224)[0]
        self.depth_scale_mm = struct.unpack_from("<f", hdr, 232)[0] or 1.0
        self._intr = {"fx": fx, "fy": fy, "cx": cx, "cy": cy} if self.intr_valid else None

    def intrinsics(self):
        return self._intr


class LocalDepthSource:
    def __init__(self, server, *, ring_name="kv_cam_v1", ring_path=None, params_cfg=None, stamps=None,
                 lib_path=None, stale_s=3.0, pose_fn=None):
        self.pose_fn = pose_fn            # CHASSIS0: () -> (x, y, yaw, ...) or None; None = the fixed bench pose
        self.server, self.ring_name, self.ring_path, self.lib_path = server, ring_name, ring_path, lib_path
        self.params_cfg, self.stamps, self.stale_s = params_cfg, stamps, stale_s
        self.params = None
        self.ring = None
        self.counts = Counter()
        self._warned = {}
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, name="kv-local-depth", daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def _open(self):
        try:
            ring = _Ring(self.ring_name, self.ring_path, self.lib_path)
        except OSError as exc:
            self.counts["open_fail"] += 1
            _warn(self._warned, "open", "cannot open kv_cam ring (%s); retrying" % exc)
            return None
        self.counts["opens"] += 1
        if self.params is None:
            self.params = FrameParams(self.params_cfg, ring)
            for n in self.params.notes:
                print(json.dumps({"kv_server_local_params": n}), file=sys.stderr, flush=True)
        return ring

    def _run(self):
        last = 0
        while not self._stop.is_set():
            if self.ring is None:
                self.ring = self._open()
                if self.ring is None:
                    self._stop.wait(0.5)
                    continue
            r = self.ring.r
            if r.stale(self.stale_s):
                self.counts["reopen_stale"] += 1
                _warn(self._warned, "stale", "kv_cam ring heartbeat stale; reopening")
                r.close()
                self.ring = None
                self._stop.wait(0.2)
                continue
            seq = r.wait_newer(last, 200)
            if seq <= last:
                continue
            t_read = time.monotonic_ns()
            if last and seq > last + 1:
                self.counts["skipped"] += seq - last - 1
            last = seq
            st, meta, _, depth = r.read(seq, colour=False, depth=True)
            if st != "ok":
                self.counts["read:" + st] += 1
                continue
            if not meta["flags"] & FLAG_HAS_DEPTH:
                self.counts["no_depth"] += 1
                continue
            fparams = self.params
            if self.pose_fn is not None:
                p = self.pose_fn()
                if p is not None:
                    fparams = self.params.at_pose(p[0], p[1], p[2])
                    self.counts["chassis_pose"] += 1
                else:
                    self.counts["chassis_pose_missing"] += 1
            try:
                h, w = depth.shape
                payload = fparams.request(meta["depth_index"], meta["arrive_ns"], depth, w, h)
            except Exception as exc:          # counted; the next frame is tried
                self.counts["build_error:%s" % type(exc).__name__] += 1
                _warn(self._warned, "build", "cannot build the request (%r)" % (exc,))
                continue
            t_put = time.monotonic_ns()
            local = {"frame": int(meta["depth_index"]), "ring_seq": int(seq),
                     "cam_dev_ts_us": int(meta["depth_dev_ts_us"]), "cam_sys_ts_us": int(meta["depth_sys_ts_us"]),
                     "t_cam_arrive_ns": int(meta["arrive_ns"]), "t_cam_publish_ns": int(meta["publish_ns"]),
                     "t_read_ns": t_read, "t_req_ns": t_put}
            if fparams is not self.params:
                local["params"] = fparams        # CHASSIS0: the chain fuses with the pose the request was built at
            old = self.server.slot.put({"addr": None, "session": 0, "frame_id": int(meta["depth_index"]),
                                        "payload": payload, "first_t": t_read / 1e9, "done_t": t_put / 1e9,
                                        "local": local})
            self.server.last_activity = time.monotonic()
            self.counts["frames"] += 1
            if old is not None:
                self.counts["superseded"] += 1   # the worker was still on the previous frame (latest-only)
                if old.get("local") is None:
                    self.counts["superseded_udp"] += 1

    def describe(self):
        return {"source": "kvcam", "ring": self.ring_path or self.ring_name, "counts": dict(self.counts),
                "params": self.params.describe() if self.params is not None else None}


class _LocalRollLink:
    """The KvLinkClient surface RemotePlRollingBackend uses, answered in process by the server's RollingService.
    Called only from the server's worker thread (the RollingService's hold/handle thread)."""

    def __init__(self, server):
        self.server = server
        self.counts = Counter()
        self.server_info = None

    def require(self, feature, timeout_s):
        rs = self.server.rolling
        return {"features": {"rolling": 1, "rolling_replay": 1, "rolling_grid": 1 if rs.grid_capable else 0}}

    def _control_resend(self, req_type, ack_type, body, wait_s, resend_s):
        self.counts["roll_local"] += 1
        return self.server.rolling.handle(body)

    def stats(self, timeout_s=0.5):
        return {"rolling": self.server.rolling.describe()}

    def close(self):
        pass


class LocalDepthChain:
    """KvLinkServer.local_sink: after the geometry core, fuse (+ GRID) in process, write the stamp row."""

    def __init__(self, server, source, stamps, nav=None, chassis=None):
        self.server, self.source, self.stamps, self.nav = server, source, stamps, nav
        self.chassis = chassis            # CHASSIS0: a KvChassisBridge feeds NAV_ODOM itself once it has a pose
        self.counts = Counter()
        self._warned = {}
        self.backend = None
        self.epoch = 0
        self.need_reset = True
        self.delta_seq = 0
        self.anchor = None
        self.map_frames = OrderedDict()   # nav_planes time -> frame (for the field stamp)
        self._last_planes = None
        self._goal_sent = False
        self._odom_seq = 0

    # ---- rolling
    def _backend(self, params):
        if self.backend is None:
            from host.kv_rolling import RemotePlRollingBackend
            b = RemotePlRollingBackend(None, link=_LocalRollLink(self.server))
            if b.grid_capable:
                from host.rolling_nav_layer import HeightBand
                b.grid_band = HeightBand(ground_z_m=params.ground_z_m, robot_height_m=params.robot_height_m)
                b.grid_voxel_m = params.voxel_m
            b.skip_changes = True          # no consumer of the per-lane deltas on the KV (the map planes are used)
            mx, my, mz = b.config.dims
            self.anchor = (mx // 2, my // 2, mz // 4)     # = RollingFusion's default window anchor
            self.backend = b
        return self.backend

    def _window_base(self, params):
        pos = params.camera_to_world[:3, 3]
        voxel = np.floor(np.asarray(pos, np.float64) / params.voxel_m).astype(np.int64)
        return tuple(int(v - a) for v, a in zip(voxel, self.anchor))

    def _obs_base(self, params):
        return tuple(int(round(float(v) / params.voxel_m)) for v in params.origin)

    def _fuse(self, req, flags, nvox, params):
        b = self._backend(params)
        wb = self._window_base(params)
        if self.need_reset:
            self.epoch += 1
            res = b.reset(self.epoch, wb)
            if not res.ok:
                raise RuntimeError("rolling reset: %s" % res.status_name)
            self.need_reset, self.delta_seq = False, 0
            self.counts["map_reset"] += 1
        nx, ny, nz = req["nx"], req["ny"], req["nz"]
        obs = np.ascontiguousarray(flags[:nvox], dtype=np.uint8).reshape(nz, ny, nx)
        res = b.integrate(self.epoch, req["frame_id"] & 0xFFFFFFFF, wb, self._obs_base(params), obs)
        if not res.ok:
            self.counts["kernel_rejected:%s" % res.status_name] += 1
            return False, None
        if res.delta_seq != self.delta_seq + 1 or res.epoch != self.epoch:
            self.counts["sequence_diverged"] += 1
            self.need_reset = True
            return False, None
        self.delta_seq = res.delta_seq
        return True, getattr(res, "grid", None)

    # ---- nav (bench goal + still odometry through the real NAV message path)
    def _nav_feed(self, params, now_ns):
        if self.nav is None or params.nav_goal is None:
            return
        if not self._goal_sent:
            x, y, yaw = (float(v) for v in params.nav_goal)
            self.nav.handle(NAV_GOAL, encode_nav_goal(NAV_OP_GOAL, 1, now_ns, x, y, yaw))
            self._goal_sent = True
        if self.chassis is not None and self.chassis.ever_heard():
            return                        # CHASSIS0: the bridge sends the real odometry at 50 Hz
        self._odom_seq += 1
        self.nav.handle(NAV_ODOM, encode_nav_odom(
            seq=self._odom_seq, t_nano_ns=now_ns, odom_stamp_ns=now_ns, odom_pose=params.odom_pose, v=0.0, w=0.0,
            map_odom=(0.0, 0.0, 0.0),
            # permit on so the core plans (with it off the core is PAUSED and builds no field). The NAV_CMD answer is
            # dropped here: in local mode nothing forwards (v, w) to a base -- the bench car does not move.
            flags=NAV_ODOM_MAP_VALID | NAV_ODOM_ODOM_VALID | NAV_ODOM_PERMIT))

    def __call__(self, item, req, status, flags, nvox, times, t_take, t_core0, t_core1):
        t_geo = time.monotonic_ns()
        local = dict(item["local"])
        params = local.pop("params", None) or self.source.params
        row = {"chain": "depth", "src": "kvcam", "frame": local["frame"], "ring_seq": local["ring_seq"],
               "cam_dev_ts_us": local["cam_dev_ts_us"], "cam_sys_ts_us": local["cam_sys_ts_us"],
               "t_cam_arrive_ns": local["t_cam_arrive_ns"], "t_cam_publish_ns": local["t_cam_publish_ns"],
               "t_read_ns": local["t_read_ns"], "t_req_ns": local["t_req_ns"],
               "t_take_ns": int(t_take * 1e9), "t_core_start_ns": int(t_core0 * 1e9),
               "t_geo_ns": int(t_core1 * 1e9), "geo_status": int(status), "kv_times_ns": dict(times)}
        self.counts["frames"] += 1
        if status == 0 and self.server.rolling is not None and params is not None:
            try:
                ok, planes = self._fuse(req, flags, nvox, params)
                row["t_grid_ns"] = time.monotonic_ns()
                row["fuse_ok"] = int(ok)
                row["grid_ok"] = int(planes is not None)
                lt = getattr(self.backend, "last_times", None)
                if isinstance(lt, dict):
                    row["roll_times_ns"] = {str(k): int(v) for k, v in lt.items()}
                elif lt is not None:
                    row["roll_times_ns"] = [int(v) for v in lt]
                cur = getattr(self.server.rolling, "nav_planes", None)
                if cur is not None and cur is not self._last_planes:
                    self._last_planes = cur
                    self.map_frames[float(cur[0])] = row["frame"]
                    while len(self.map_frames) > 64:
                        self.map_frames.popitem(last=False)
            except Exception as exc:          # counted; the map is rebuilt from the next frame
                self.counts["fuse_error:%s" % type(exc).__name__] += 1
                self.need_reset = True
                _warn(self._warned, "fuse", "rolling fuse failed (%r); map reset next frame" % (exc,))
        try:
            self._nav_feed(params, time.monotonic_ns())
        except Exception as exc:
            self.counts["nav_feed_error:%s" % type(exc).__name__] += 1
        row["t_end_ns"] = time.monotonic_ns()
        row["geo_to_end_us"] = round((row["t_end_ns"] - t_geo) / 1e3, 1)
        self.stamps.write(row)

    def describe(self):
        return {"counts": dict(self.counts), "epoch": self.epoch, "delta_seq": self.delta_seq,
                "backend_counts": dict(self.backend.counts) if self.backend is not None else None}


class StampedNavCore:
    """Wraps the NavService core. A frame's potential field is "done" at the first replan that ran on that frame's
    grid or a newer one: after each 20 Hz step, a new map_stamp marks the frame whose grid the window now holds
    (pending); a replan in this or a later step (kv_nav_last_plan_ms changed: every replan measures itself) writes
    the "field" row for the newest pending frame. kv_nav replans every replan_period (0.5 s default) or when the path
    is blocked / left, not per grid: with the default the row includes the wait for the next replan; with
    KV_NAV_PARAMS='{"replan_period": 0}' every tick replans (the "replan every frame" reading). Grids replaced before
    any replan used them get no row (counted as field_skipped). Everything else is delegated untouched."""

    def __init__(self, core, chain_ref, stamps):
        self._core, self._chain_ref, self._stamps = core, chain_ref, stamps
        self._seen = None
        self._pending = None            # (frame, map_stamp, t_map_ns)
        self._plan_ms = None
        self.field_counts = Counter()

    def __getattr__(self, name):
        return getattr(self._core, name)

    def _last_plan_ms(self):
        lib = getattr(self._core, "lib", None)
        try:
            return float(lib.kv_nav_last_plan_ms(self._core.h))
        except Exception:
            return None

    def step(self, now_s, hold=False):
        out = self._core.step(now_s, hold=hold) if hold else self._core.step(now_s)
        t = time.monotonic_ns()
        stamp = getattr(self._core, "map_stamp", None)
        if stamp is not None and stamp != self._seen:
            self._seen = stamp
            chain = self._chain_ref()
            frame = chain.map_frames.get(float(stamp)) if chain is not None else None
            if self._pending is not None:
                self.field_counts["field_skipped"] += 1
            self._pending = (frame, float(stamp), t)
        plan_ms = self._last_plan_ms()
        replanned = plan_ms is not None and plan_ms != self._plan_ms
        self._plan_ms = plan_ms
        if replanned:
            self.field_counts["replans"] += 1
        if replanned and self._pending is not None:
            frame, mstamp, t_map = self._pending
            self._pending = None
            self.field_counts["field_rows"] += 1
            self._stamps.write({"chain": "depth", "ev": "field", "frame": frame, "t_map_ns": t_map, "t_field_ns": t,
                                "state": out.get("state"), "reason": out.get("reason"),
                                "plan_status": out.get("plan_status"), "plan_ms": round(plan_ms, 3),
                                **self._pln_row()})
        return out

    def _pln_row(self):
        """PLNCHAIN0: with PLN=gmap3 every field row carries the GMAP3 counters (path fetch / global map)."""
        f = getattr(self._core, "pln_stats", None)
        try:
            st = f() if f is not None else None
        except Exception:
            st = None
        return {"pln": st} if st else {}

    def describe(self):
        d = self._core.describe() if hasattr(self._core, "describe") else {"name": "?"}
        d["field_stamps"] = dict(self.field_counts)
        return d


def dump_core_input(directory, req, depth, use16, pixels):
    """Tests (kv_server --dump-core-input DIR): the exact geometry-core input of one frame, DIR/<frame_id>.bin =
    inverse_k f32[9] | world_camera f32[12] | origin f64[3] | voxel f64 | w h nx ny nz u32 | unit_bits u32 |
    depth f32[w*h] (D16 path: the u16 codes instead)."""
    import struct
    head = b"".join([np.ascontiguousarray(req["inverse_k"], dtype="<f4").tobytes(),
                     np.ascontiguousarray(req["world_camera"], dtype="<f4").tobytes(),
                     np.ascontiguousarray(req["origin"], dtype="<f8").tobytes(),
                     struct.pack("<d6I", float(req["voxel"]), req["width"], req["height"], req["nx"], req["ny"],
                                 req["nz"], req["unit_bits"])])
    body = (np.ascontiguousarray(req["codes"], dtype="<u2").tobytes() if use16
            else np.ascontiguousarray(depth[:pixels], dtype="<f4").tobytes())
    with open(os.path.join(directory, "%d.bin" % req["frame_id"]), "wb") as fp:
        fp.write(head + body)
