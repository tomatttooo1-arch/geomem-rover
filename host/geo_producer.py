"""Car geometry producer core: depth -> KV260 -> RollingFusion -> deltas (2026-10-11).

ROS-free; host/ros_geo_producer.py is the rclpy shell (topics, TF, threads) and the only
KV260 frame client on the car. Docs: docs/design/CAR_GEO_PRODUCER_RESULTS_20261011.md.

One call per depth frame, ``GeoProducer.process``:

  1. the frame's uint16 camera codes -> float32 metres with the reference rule
     (kv_link.codes_to_m); the codes themselves go to the KV260 (``set_next_codes``,
     no requantization; the KV260's crc of its converted depth must still match);
  2. ROI: the newest /fpga/memory boxes (world frame ``map``) projected with
     ``geometry_region_glue.plan_frame`` on T_map_camera and attached to the SAME frame
     (``set_next_roi``; set or cleared before EVERY frame, so a frame the pipeline drops
     before the backend cannot leave its batch for the next one);
  3. M7Pipeline(backend, GridSpec, grid_policy=FollowCameraGrid) on T_odom_camera at the
     acquisition (odom <- base_link x base_link <- camera, looked up by the shell at the
     depth header stamp): the per-frame block is in odom on the 5 cm lattice;
  4. RollingFusion.ingest with the odom base position -> NavigationDelta (the shell hands
     it to RollingProducerLink.publish);
  5. the per-frame body for /fpga/geometry_frame: the D.2 /fpga/region_verdicts body
     (``verdict_message``) plus a ``producer`` block (frame id, depth header stamp,
     acquisition RAW ns, geometry result, fusion result, delta epoch/seq).

Clock: every stamp compared across events is host.fpga_clock (CLOCK_MONOTONIC_RAW,
CLOCK_ID "nano_monotonic_raw_ns"); acquisition = RAW arrival of the depth message at the
producer (same definition as ros_runtime's kv path). ROS header stamps are only frame
identities (matching), never subtracted from RAW stamps.

Freshness: M7Pipeline's 200 ms frame age (D-035) is the only rule and is not relaxed here:
a frame older than 200 ms at the geometry step or at fusion is dropped (counted); its ROI
answer, if any, is withdrawn too (``failure: expired_output``).

Per-frame block shape (``DEFAULT_SHAPE``): 32 x 32 x 10 voxels of 5 cm, camera at
``height_cell`` 7 -- the block of the Nav2 smoke (robot_sim/tests/rolling_nav2_smoke.py);
the obstacle band ends at robot height + 5 cm, so layers above the camera are not needed
by Nav2 (32^3 would send 3.2x the flags and integrate 3.2x the voxels for nothing the
costmap uses). ``check_block_height`` refuses a mount whose ground is not at least one
voxel inside the block.

AR labels: each ROI row carries the class name (``label``) of its memory track, taken from
the same /fpga/memory body the boxes were planned from (the task runner's object memory,
host/memory_publish.py); the KV260 shows them on the AR boxes (REQ ext_flags bit2, sent
only when the server announces ``label_ext``). ``roi_labels=False`` sends none.

Data source (owner 10-12: replay allowed, but always marked): the node is told where its depth
comes from (``--data-source live|replay``, required). With ``replay`` every KV260 request
carries the replay mark (REQ ext_flags bit3, only to a server announcing ``replay_ext``) and
the AR panel says REPLAY. A replay stand-in (tools/car_geo_producer/replay_camera.py)
publishes a latched marker on ``SENSOR_SOURCE_TOPIC``; ``SourceMarker`` cross-checks it
against the flag, and every /fpga/kv_link/frame_stats body carries ``source``,
``source_marker``, ``source_mismatch`` (a disagreement is counted, never silent) and
``source_warning`` (v5: replay without a marker, or a mismatch; an event on each change, never blocking).

Python 3.8 compatible, no ROS import.
"""

import collections
import hashlib
import json
import math
import os

import numpy as np

from host.fpga_clock import CLOCK_ID, now_ns

VOXEL_M = 0.05
DEFAULT_SHAPE = (32, 32, 10)
DEFAULT_HEIGHT_CELL = 7
DEFAULT_BACK_CELLS = 1
MAX_FRAME_AGE_NS = 200_000_000          # D-035; never larger
LOCAL_FRAME = "odom"
POSE_EPOCH = "odom_v1"
DATA_SOURCES = ("live", "replay", "sim")   # sim (line D1, 2026-10-14): Gazebo via scripts/sim_adapter.py
MARKED_SOURCES = ("replay", "sim")          # stand-ins that always latch a marker; KV260 replay mark set
SENSOR_SOURCE_TOPIC = "/fpga/sensor_source"
RESET_REASONS = ("context_changed_reset_required", "context_discontinuity_reset_required",
                 "calibration_content_changed_reset_required")

# Out-of-range depth (docs/design/DEPTH_OOR_INVALID_20261016.md, owner 10-16). OOR_MAX_M is
# host/m7_pipeline.py's indoor range check (any pixel > 20 m refuses the frame); it is not a knob.
OOR_MODES = ("invalidate", "reject")
OOR_MAX_M = 20.0
OOR_INVALID_CODE = 0                  # the camera's "no return" code (near-range laser protection)
DEFAULT_OOR_FRAC_MAX = 0.05           # T1 10-16: real frames <= 0.82 %; owner to confirm
DEFAULT_OOR_NEAR_M = 0.8              # T1 10-16 (DEPTH_OOR_INVALID §5); owner to confirm (was 1.5)


def ground_near_pixels(pixels, width, K, T_odom_camera, ground_z_m, near_m):
    """Bool per flat pixel index: its ray reaches the ground plane z = ``ground_z_m`` (odom)
    at a horizontal distance <= ``near_m`` from the camera centre. Rays that do not point
    down never reach the ground (False). The camera at or below the ground: all True
    (cannot judge -> conservative)."""
    pixels = np.asarray(pixels, dtype=np.int64)
    T = np.asarray(T_odom_camera, dtype=np.float64)
    height = float(T[2, 3]) - float(ground_z_m)
    if not (height > 0 and np.isfinite(height)):
        return np.ones(pixels.shape, dtype=bool)
    uv1 = np.stack([(pixels % width).astype(np.float64), (pixels // width).astype(np.float64),
                    np.ones(pixels.shape, dtype=np.float64)])
    d = T[:3, :3] @ (np.linalg.inv(np.asarray(K, dtype=np.float64)) @ uv1)
    down = d[2] < 0
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(down, height / -d[2], np.inf)
        horiz = t * np.hypot(d[0], d[1])
    return down & (horiz <= float(near_m))


def screen_out_of_range(depth_m, K, T_odom_camera, *, mode="invalidate", ground_z_m=None,
                        frac_max=DEFAULT_OOR_FRAC_MAX, near_m=DEFAULT_OOR_NEAR_M):
    """Per-frame out-of-range screen. Returns (mask or None, info).

    ``mask`` (bool HxW): the pixels to mark invalid (code 0 / NaN), only in mode
    "invalidate" when neither gate fires. Otherwise None: the frame goes on unchanged and
    m7_pipeline's > 20 m check refuses it exactly as before. ``info``: the per-frame record
    fields (oor_pixels, oor_fraction, oor_near_pixels, oor_max_m, oor_gate, oor_action).
    Gates (either refuses the frame): fraction of OOR pixels over ALL pixels > frac_max;
    any OOR pixel in the near-ground band (ground_near_pixels; no ground_z_m -> "no_ground").
    The near band is not computed when the fraction gate already fired (oor_near_pixels None).
    """
    if mode not in OOR_MODES:
        raise ValueError("depth oor mode must be one of %s" % (OOR_MODES,))
    oor = depth_m > OOR_MAX_M                      # same comparison as m7_pipeline (NaN -> False)
    n = int(np.count_nonzero(oor))
    info = {"oor_pixels": n, "oor_fraction": n / float(depth_m.size), "oor_near_pixels": None,
            "oor_max_m": None, "oor_gate": None, "oor_action": "none"}
    if n == 0:
        info["oor_near_pixels"] = 0
        return None, info
    info["oor_max_m"] = float(np.max(depth_m[oor]))
    if info["oor_fraction"] > float(frac_max):
        info["oor_gate"] = "fraction"
    elif ground_z_m is None:
        info["oor_gate"] = "no_ground"
    else:
        flat = np.flatnonzero(oor)
        near = int(np.count_nonzero(ground_near_pixels(flat, depth_m.shape[1], K, T_odom_camera,
                                                       ground_z_m, near_m)))
        info["oor_near_pixels"] = near
        if near:
            info["oor_gate"] = "near"
    if mode == "reject" or info["oor_gate"] is not None:
        info["oor_action"] = "refused"
        return None, info
    info["oor_action"] = "invalidated"
    return oor, info


def check_block_height(camera_z_m, ground_z_m, *, voxel_m=VOXEL_M, shape=DEFAULT_SHAPE,
                       height_cell=DEFAULT_HEIGHT_CELL):
    """(ok, info): the follow block must hold the ground with at least one voxel below it
    (the ground hit must not sit on the block's lower face, where float32 jitter moves it
    to ``outside_grid``) and the camera one voxel inside the top."""
    cam_cell = int(math.floor(float(camera_z_m) / voxel_m))
    bottom = (cam_cell - int(height_cell)) * voxel_m
    top = bottom + shape[2] * voxel_m
    info = {"camera_z_m": float(camera_z_m), "ground_z_m": float(ground_z_m),
            "block_bottom_m": bottom, "block_top_m": top, "camera_cell": cam_cell,
            "height_cell": int(height_cell), "nz": int(shape[2])}
    ok = bottom <= float(ground_z_m) - voxel_m + 1e-9 and 1 <= height_cell <= shape[2] - 2
    if not ok:
        info["need_height_cell_at_least"] = int(math.ceil(
            cam_cell + 1 - float(ground_z_m) / voxel_m - 1e-9))
    return ok, info


def decimate_depth_codes(codes, stride):
    """``stride`` x ``stride`` block -> the block's smallest VALID uint16 code (code 0 = invalid; an
    all-invalid block stays 0). Rows/columns past the last whole block are dropped. Stays in uint16
    codes, so ``codes_to_m`` on the result is still exact (the KV260's depth crc holds end to end).
    Minimum, not the top-left sample (2026-10-16, controller): the nearest return in the block is the
    conservative one for an obstacle (never shortens free space through something seen in the block)."""
    s = int(stride)
    codes = np.asarray(codes)
    if s == 1:
        return codes
    if s < 1 or codes.ndim != 2 or codes.dtype != np.uint16:
        raise ValueError("stride >= 1 and 2D uint16 codes required")
    h, w = (codes.shape[0] // s) * s, (codes.shape[1] // s) * s
    if h == 0 or w == 0:
        raise ValueError("frame smaller than one block")
    blk = codes[:h, :w].reshape(h // s, s, w // s, s)
    # invalid (0) -> 65536 (above every uint16 code) before the min, back to 0 after: an all-invalid
    # block stays invalid and every valid code, 0xFFFF included, keeps its value
    m = np.where(blk == 0, np.int32(65536), blk.astype(np.int32)).min(axis=(1, 3))
    m[m == 65536] = 0
    return np.ascontiguousarray(m, dtype=np.uint16)


def scale_intrinsics(K, stride):
    """K of the decimated image. The geometry core casts a pixel's ray through u = column index (no
    +0.5, hw/m7/frame_depth.cpp), so output pixel u' stands for the block centre s u' + (s - 1)/2 of
    the full image: fx' = fx/s, skew' = skew/s, cx' = (cx - (s - 1)/2)/s (same for y)."""
    s = int(stride)
    K = np.asarray(K, dtype=np.float64).copy()
    if s == 1:
        return K
    off = (s - 1) / 2.0
    K[0, 0] /= s
    K[0, 1] /= s
    K[1, 1] /= s
    K[0, 2] = (K[0, 2] - off) / s
    K[1, 2] = (K[1, 2] - off) / s
    return K


def calibration_id(K, T_base_camera, width, height):
    body = json.dumps({"K": [float(v) for v in np.asarray(K, dtype=np.float64).ravel()],
                       "base_camera": [float(v) for v in np.asarray(T_base_camera, dtype=np.float64).ravel()],
                       "size": [int(width), int(height)]}, sort_keys=True)
    return "geo-" + hashlib.sha256(body.encode()).hexdigest()[:16]


SOURCE_WARNINGS = ("replay_without_marker", "sim_without_marker", "marker_mismatch")
SOURCE_WARNING_GRACE_FRAMES = 10        # ~1 s at 10 Hz: the latched marker may arrive after frame 1


class SourceMarker(object):
    """The data-side evidence of where the depth comes from: the newest JSON body on
    SENSOR_SOURCE_TOPIC ({"sensor_source": "live"|"replay", "replay_id", "publisher"}, latched
    by the replay stand-in). ``check()`` -> (marker dict or None, mismatch): mismatch when a
    marker is there and names another source than the node's --data-source. No marker is not a
    mismatch (a camera driver publishes none); it shows as ``source_marker: null``.

    Source warning (2026-10-12 v5, controller): ``source_warning`` in every frame_stats body, never
    blocking: ``marker_mismatch`` (a marker naming the other source) or ``replay_without_marker``
    (--data-source replay, and still no marker after SOURCE_WARNING_GRACE_FRAMES frames -- a
    replay stand-in always latches one, so its absence means the replay is not the one we think);
    None otherwise, in particular for live with no marker (a real camera). ``transition()`` gives
    one /fpga/status event per change (``source_warning`` / ``source_warning_cleared``)."""

    def __init__(self, data_source, grace_frames=SOURCE_WARNING_GRACE_FRAMES):
        if data_source not in DATA_SOURCES:
            raise ValueError("data source must be live, replay or sim")
        self.data_source = data_source
        self.marker = None
        self.counts = collections.Counter()
        self.grace_frames = int(grace_frames)
        self.frames = 0
        self.warning = None                 # last reported warning (for transition())

    def on_marker(self, text):
        try:
            body = json.loads(text)
            src = body.get("sensor_source") if isinstance(body, dict) else None
        except ValueError:
            src = None
        if src not in DATA_SOURCES:
            self.counts["marker_bad"] += 1
            return
        self.counts["marker_rx"] += 1
        self.marker = {k: body.get(k) for k in ("sensor_source", "replay_id", "publisher")}

    def check(self):
        m = self.marker
        mismatch = m is not None and m["sensor_source"] != self.data_source
        self.counts["frames_mismatch" if mismatch else
                    ("frames_marker_ok" if m is not None else "frames_no_marker")] += 1
        return (dict(m) if m is not None else None), mismatch

    def warning_for(self, marker, mismatch):
        if mismatch:
            return "marker_mismatch"
        if self.data_source in MARKED_SOURCES and marker is None and self.frames > self.grace_frames:
            return self.data_source + "_without_marker"
        return None

    def stats_fields(self):
        """The frame_stats keys of one frame (the hub reads them by these names)."""
        self.frames += 1
        m, mismatch = self.check()
        warning = self.warning_for(m, mismatch)
        if warning is not None:
            self.counts["frames_warning:" + warning] += 1
        return {"source": self.data_source, "source_marker": m, "source_mismatch": bool(mismatch),
                "source_warning": warning}

    def transition(self, fields):
        """Event dict when ``fields["source_warning"]`` differs from the last one reported, else None."""
        w = fields.get("source_warning")
        if w == self.warning:
            return None
        prev, self.warning = self.warning, w
        ev = {"event": "source_warning" if w is not None else "source_warning_cleared",
              "source": "geo_producer", "reason": w if w is not None else prev,
              "data_source": self.data_source, "source_marker": fields.get("source_marker"),
              "frames": self.frames}
        self.counts["warning_events"] += 1
        return ev


class KvConnection(object):
    ar_sink = None      # AR2NANO: host/nano_ar.py NanoArScreen.offer, set by the caller (None = as before)
    """HELLO (+ optional sysinfo BUILD_ID check) -> RemotePlFrameBackend. ROI is optional:
    without ``roi_ext``/roi core the producer still makes geometry (``roi_available``).
    ``data_source`` "replay": the backend sets the replay mark on every request (when the
    server announces ``replay_ext``).

    ``connect()`` (2026-10-15 fix): the new client is built and HELLOed first and replaces the old
    one only on success (the old socket is closed then). A failed re-HELLO leaves the old
    client/backend as they were -- the core still holds that backend, and closing its socket first
    made every later frame raise OSError(EBADF) out of M7Pipeline (it catches LinkError, not
    OSError). The kept session is usable: a kv_server restarted on the same port answers an old
    session, and while it is down the frames fail as ``backend_failure:Link*`` (the shell's
    link-failure count). ``last_connect_error`` = why the last attempt failed (None after success);
    ``reason`` stays None while an old session is kept."""

    def __init__(self, server, *, depth_unit_m, timeout_s=0.3, hello_timeout_s=0.5,
                 expect_build_id=None, chunk_bytes=None, client_factory=None,
                 telemetry_factory=None, data_source=None):
        from host import kv_link
        self._kv = kv_link
        if data_source not in (None,) + DATA_SOURCES:
            raise ValueError("data_source must be live, replay or sim")
        self.data_source = data_source
        self.server = (server[0], int(server[1]))
        self.depth_unit_m = float(depth_unit_m)
        self.timeout_s, self.hello_timeout_s = float(timeout_s), float(hello_timeout_s)
        self.expect_build_id = expect_build_id
        chunk = kv_link.CHUNK_FRAGMENTED if chunk_bytes is None else int(chunk_bytes)
        self._client_factory = client_factory or (lambda srv: kv_link.KvLinkClient(
            srv, chunk_bytes=chunk, timeout_s=self.timeout_s))
        self._telemetry_factory = telemetry_factory or (lambda srv: kv_link.KvTelemetryClient(
            srv, timeout_s=0.2, hello_timeout_s=self.hello_timeout_s))
        self.client = self.backend = self.info = self.build_id = None
        self.roi_available = False
        self.reason = "not_connected"
        self.last_connect_error = None
        self.counts = collections.Counter()

    def _retire(self, client):
        """Fold a client's counters into ``counts`` and close its socket."""
        self.counts.update({"link:" + k: v for k, v in client.counts.items()})
        try:
            client.close()
        except Exception:
            pass

    def close(self):
        if self.client is not None:
            self._retire(self.client)
        self.client = self.backend = None

    def connect(self):
        """HELLO a NEW client; True: it replaced the old one. False: nothing changed (the old
        client/backend, if any, stay open and in use); ``last_connect_error`` says why, and
        ``reason`` too when there is no session at all (it stays None while an old one is kept)."""
        kv = self._kv
        self.counts["connect_attempts"] += 1
        client = None
        try:
            client = self._client_factory(self.server)
            info = client.hello(self.hello_timeout_s)
            feats = info.get("features") if isinstance(info, dict) else None
            build_id = None
            if self.expect_build_id is not None:
                if not isinstance(feats, dict) or feats.get("telem") != 1:
                    raise ValueError("build_id_unchecked:no_telem")
                tel = self._telemetry_factory(self.server)
                try:
                    board = tel.poll(kv.blocks_mask(("board",))).blocks.get("board") or {}
                finally:
                    tel.close()
                build_id = board.get("build_id")
                if build_id is None or int(build_id) != int(self.expect_build_id):
                    raise ValueError("build_id_mismatch:%r" % (build_id,))
        except (kv.LinkError, OSError, ValueError, KeyError, TypeError) as exc:
            if client is not None:
                client.close()
            why = ("hello_timeout" if isinstance(exc, kv.LinkTimeout)
                   else str(exc) if isinstance(exc, ValueError)
                   else "connect_error:%s" % type(exc).__name__)
            self.last_connect_error = why
            self.counts["unavailable:" + why.split(":", 1)[0]] += 1
            if self.client is not None:          # old session kept: still connected, reason unchanged
                self.counts["kept_previous_session"] += 1
            else:
                self.reason = why
            return False
        if self.client is not None:
            self._retire(self.client)
        self.client, self.info, self.build_id = client, dict(info), build_id
        self.roi_available = (isinstance(feats, dict) and feats.get("roi_ext") == 1 and
                              info.get("roi_core") not in (None, "none") and
                              int(info.get("roi_max_boxes", 0) or 0) >= 1)
        self.backend = kv.RemotePlFrameBackend(client, depth_unit_m=self.depth_unit_m)
        self.backend.data_source = self.data_source
        if self.ar_sink is not None:     # AR2NANO (2026-11-02): car screen drawn on the Nano (default None)
            self.backend.lib.ar_sink = self.ar_sink
        self.reason = self.last_connect_error = None
        self.counts["connects"] += 1
        return True

    def describe(self):
        info = self.info or {}
        return {"server": "%s:%d" % self.server, "connected": self.backend is not None,
                "reason": self.reason, "last_connect_error": self.last_connect_error,
                "core": info.get("core"), "roi_core": info.get("roi_core"),
                "roi_available": self.roi_available, "kv_boot_id": info.get("kv_boot_id"),
                "build_id": None if self.build_id is None else "%#010x" % int(self.build_id),
                "expect_build_id": (None if self.expect_build_id is None
                                    else "%#010x" % int(self.expect_build_id)),
                "depth_unit_m": self.depth_unit_m, "timeout_s": self.timeout_s,
                "data_source": self.data_source,
                "server_replay_ext": (info.get("features") or {}).get("replay_ext")
                if isinstance(info.get("features"), dict) else None}


class KvReconnect(object):
    """Link health of the producer's KV session (the shell's rule, ROS-free since 2026-10-15):
    a frame whose nav_reason is ``backend_failure:Link*`` is a link failure, any other frame resets
    the count; at ``reconnect_after`` consecutive failures, at most one re-HELLO per ``retry_s``.
    A failed re-HELLO changes nothing (KvConnection keeps the old session), so later frames keep
    counting on the same path. ``after_frame`` -> "reconnected" | "reconnect_failed" | None."""

    def __init__(self, conn, *, reconnect_after, retry_s, roi_enabled, clock=now_ns):
        self.conn = conn
        self.reconnect_after = int(reconnect_after)
        self.retry_s = float(retry_s)
        self.roi_enabled = bool(roi_enabled)
        self.clock = clock
        self.failures = 0
        self.last_connect_ns = clock()
        self.counts = collections.Counter()

    def after_frame(self, nav_reason, set_backend):
        if str(nav_reason).startswith("backend_failure:Link"):
            self.failures += 1
        else:
            self.failures = 0
        if self.failures < self.reconnect_after or self.clock() - self.last_connect_ns <= self.retry_s * 1e9:
            return None
        self.last_connect_ns = self.clock()
        if not self.conn.connect():
            self.counts["reconnect_failed"] += 1
            return "reconnect_failed"
        self.conn.backend.roi_enabled = self.conn.roi_available and self.roi_enabled
        set_backend(self.conn.backend)
        self.counts["reconnects"] += 1
        self.failures = 0
        return "reconnected"


# 10-26 owner：重成功率，放宽自 50（连续 50 帧异常即退出、整车 fail-stop）：0 = never exits on frame errors; each one is
# counted (worker_error:*) and the link health re-HELLOs the KV260. The loop itself failing still ends the node.
WORKER_ERROR_LIMIT = 0      # consecutive frame exceptions before the node exits (0 = never)


def run_worker(stop, take, handle, on_error=None, limit=WORKER_ERROR_LIMIT, on_fatal=None):
    """The producer's worker loop (2026-10-15). ``take()`` -> next item or None; ``handle(item)``.

    One frame raising is counted (``on_error(exc, streak)``) and the loop goes on ("one bad frame
    never stops the producer"). ``limit`` consecutive frames raising (0 = never), or any exception
    of the loop itself (take, on_error), ends it: ``stop`` is set and the fatal reason returned, so
    the node exits non-zero (car fail-stop) instead of living without producing frames.
    ``on_fatal(reason)`` runs before ``stop`` is set (a waiter on ``stop`` then sees the reason).
    Returns None when ``stop`` was set by someone else."""
    fatal = None
    streak = 0
    try:
        while not stop.is_set():
            item = take()
            if item is None:
                continue
            try:
                handle(item)
            except Exception as exc:  # noqa: BLE001 - counted; fatal only when it repeats
                streak += 1
                if on_error is not None:
                    on_error(exc, streak)
                if limit and streak >= limit:
                    fatal = "worker_errors:%d:%s" % (streak, type(exc).__name__)
                    break
            else:
                streak = 0
    except BaseException as exc:  # noqa: BLE001 - the loop itself failed: fail-stop
        fatal = "worker_crashed:%s" % type(exc).__name__
    if fatal is not None:
        try:
            if on_fatal is not None:
                on_fatal(fatal)
        finally:
            stop.set()
    return fatal


class FrameGapTracker(object):
    """Why the producer sends no frame (and the KV260 permit therefore lapses), in two phases.

    ``startup``: no frame has had its pose yet since the producer started (TF not ready) --
    the permit has never been granted by this producer; ``running``: a pose-less frame after
    at least one good one (TF stream interrupted while running). A pose-less frame is never
    sent (no older pose stands in for it), so the KV260 permit lapses on its own 200 ms
    rule; this class only makes the cause visible: counters per phase and reason, and one
    event when a gap starts and one when frames flow again (the shell publishes them on
    /fpga/status, the hub forwards them as EVENT). Clock: host.fpga_clock RAW ns.
    """

    def __init__(self):
        self.ready = False
        self.gap = None                 # {"phase", "reason", "first_ns", "missed"}
        self.counts = collections.Counter()

    def missing(self, reason, now_ns, **extra):
        """A frame dropped for ``reason`` (e.g. tf_missing). Returns an event dict or None."""
        phase = "running" if self.ready else "startup"
        self.counts["%s:%s" % (reason, phase)] += 1
        if self.gap is not None:
            self.gap["missed"] += 1
            return None
        self.gap = {"phase": phase, "reason": reason, "first_ns": int(now_ns), "missed": 1}
        self.counts["gaps:%s" % phase] += 1
        ev = {"event": "geometry_gap", "source": "geo_producer", "phase": phase, "reason": reason,
              "raw_ns": int(now_ns)}
        ev.update(extra)
        return ev

    def ok(self, now_ns):
        """A frame got its pose and went to the KV260. Returns an event dict or None."""
        first = not self.ready
        self.ready = True
        if self.gap is None and not first:
            return None
        gap, self.gap = self.gap, None
        ev = {"event": "geometry_ready" if first else "geometry_resumed", "source": "geo_producer",
              "raw_ns": int(now_ns)}
        if gap is not None:
            ev.update({"phase": gap["phase"], "reason": gap["reason"], "missed": gap["missed"],
                       "gap_ms": (int(now_ns) - gap["first_ns"]) / 1e6})
        return ev

    def state(self):
        if self.gap is not None:
            return "gap_%s:%s" % (self.gap["phase"], self.gap["reason"])
        return "ready" if self.ready else "startup"


class InputGapCounter(object):
    """Depth frames the camera sent that never reached this node's subscription callback (v8,
    2026-10-12: board v7, 4 frames in 3 x 180 s, each one a withdrawn KV260 permit candidate).

    ``feed`` gets every arrival's header stamp in the callback, BEFORE the rate limit and the
    one-slot mailbox, so the three ways a frame is not processed stay apart:
      * lost before the callback (DDS transport / reader)  -> counted here (``missed_frames``)
      * arrived, skipped by --rate-hz                       -> counts["rate_skipped"]
      * arrived, overwritten in the mailbox by a newer one  -> LatestSlot.superseded
    A stamp step longer than GAP_FACTOR source periods is a gap of round(step / period) - 1
    frames. The source period is the median of the last PERIOD_WINDOW ordinary steps (the
    camera's own rate, whatever --rate-hz is); nothing is counted before MIN_STEPS steps are
    known. A step <= 0 (repeated or older stamp) is counted as ``non_monotonic`` and otherwise
    ignored. Evidence only: nothing is gated on it, and a frame that did not arrive is never
    replaced by an older one. Stamps in ns (header.stamp), ``raw_ns`` = host.fpga_clock arrival.
    """

    GAP_FACTOR = 1.5
    PERIOD_WINDOW = 31
    MIN_STEPS = 8

    def __init__(self):
        self.steps = collections.deque(maxlen=self.PERIOD_WINDOW)
        self.last_stamp_ns = None
        self.counts = collections.Counter()      # arrivals, gaps, missed_frames, non_monotonic
        self.max_step_ns = 0

    def period_ns(self):
        if len(self.steps) < self.MIN_STEPS:
            return None
        s = sorted(self.steps)
        return s[len(s) // 2]

    def feed(self, stamp_ns, raw_ns):
        """One arrival. Returns an ``input_gap`` event dict when frames are missing before it."""
        stamp_ns = int(stamp_ns)
        self.counts["arrivals"] += 1
        prev, ev = self.last_stamp_ns, None
        if prev is not None:
            step = stamp_ns - prev
            if step <= 0:
                self.counts["non_monotonic"] += 1
                return None
            period = self.period_ns()
            self.max_step_ns = max(self.max_step_ns, step)
            if period is not None and step > self.GAP_FACTOR * period:
                missed = max(1, int(round(step / float(period))) - 1)
                self.counts["gaps"] += 1
                self.counts["missed_frames"] += missed
                ev = {"event": "input_gap", "source": "geo_producer", "missed": missed,
                      "step_ms": step / 1e6, "period_ms": period / 1e6, "prev_stamp_ns": prev,
                      "stamp_ns": stamp_ns, "raw_ns": int(raw_ns)}
            else:
                self.steps.append(step)
        self.last_stamp_ns = stamp_ns
        return ev

    def summary(self):
        period = self.period_ns()
        return {"arrivals": self.counts["arrivals"], "gaps": self.counts["gaps"],
                "missed_frames": self.counts["missed_frames"], "non_monotonic": self.counts["non_monotonic"],
                "period_ms": None if period is None else period / 1e6, "max_step_ms": self.max_step_ns / 1e6}


class RateGate(object):
    """The producer's --rate-hz frame gate (2026-10-15; runs/car-bench-20261015-results/FUSED_DROP_DIG.md).

    Before: a frame was skipped when its callback ran < 0.9 / rate_hz (90 ms at 10 Hz) after the last
    released one's callback. At 30 fps the 3rd frame is due 96.9 ms later: 7 ms of margin on the
    ARRIVAL time. Board 10-15 (daylight, target in view): depth frames reached the callback in pairs
    64.6 ms apart, so the 3rd frame always came at ~65 ms, the gate released every 4th frame, 929
    fused frames / 120 s (7.75 Hz) instead of ~1165, 563 of 928 map-age gaps over 200 ms, 91
    map_stale permit withdrawals at the bridge. Self-sustaining: each release restarts the same 7 ms
    margin.

    mode "stamp" (default): the gate reads header.stamp (the camera's own clock), not the arrival.
      A frame is released when stamp - last released stamp >= threshold. Threshold (auto):
        P = the source period: InputGapCounter.period_ns() (median of the camera's last 31 ordinary
            stamp steps); before it is known, 1 / nominal_fps (default 30)
        k = the smallest whole number of source periods with k * P >= 0.9 / rate_hz (the old
            gate's 0.9 factor, now counted in camera frames)
        threshold = (k - 0.5) * P   (half a period of margin on both sides: the (k-1)th frame is
            refused, the kth released, whatever the delivery jitter, as long as the stamps are clean)
      30.96 fps (board, P 32.3 ms) at 10 Hz: k 3, threshold 80.75 ms -> every 3rd frame (10.3 Hz);
      the board period moves between 32.3 and 33.4 ms: a check reads k == 3 and threshold_ms ==
      (k - 0.5) x the measured period (summary input.period_ms), not a fixed ms range;
      the replay camera (stamp = ROS now, 10 Hz): k 1, threshold 50 ms -> every frame; 15 fps: k 2.
      k changes where 0.9 / rate_hz is a whole number of periods (10 Hz: P 30 ms, 45 ms, 90 ms; a
      30 fps camera, P 32.2-33.4 ms, is far from both). Stamp jitter must stay under a quarter period.
      min_step_ms > 0 replaces the auto threshold (fixed, in ms of stamp time).
      Stamps: a stamp equal to the last released one (``stamp_repeat``) or older (``stamp_older``)
      is never released. Liveness (beyond the 10-15 brief, for the controller to rule on): nothing
      downstream checks stamp order, and the stamps are wall clock (chrony steps it); a timeline
      that went back would otherwise freeze the producer for as long as the step. So when every
      arrival for ``rebase_s`` of RAW time (default 2 s) has been strictly older than the last
      released stamp, the gate re-anchors on the newest one (released, ``rebase``, one event). A
      repeated stamp never re-anchors; a single older frame between good ones never does either.
    mode "arrival": the old gate exactly (RAW arrival, 0.9 / rate_hz, no stamp checks). The producer's
      --rate-gate auto (default) picks stamp for --data-source live, arrival for replay / sim
      (``switch`` / ``basis`` record that choice in stats()).

    ``decide(stamp_ns, raw_ns, period_ns)`` -> (release, reason, event): reason None when released
    normally, "rebase" when released by re-anchoring, else the skip reason "rate" | "stamp_repeat" |
    "stamp_older" (arrival mode: "rate_arrival"). ``event``: a status/record dict at the first
    older stamp of an episode (``depth_stamp_regression``) and at a rebase (``depth_stamp_rebase``).
    No ROS; the mailbox after it is unchanged (one slot, the newest wins, superseded counted).
    """

    MODES = ("stamp", "arrival")
    ARRIVAL_FACTOR = 0.9

    def __init__(self, rate_hz, mode="stamp", min_step_ms=0.0, nominal_fps=30.0, rebase_s=2.0,
                 switch=None, basis=None):
        if mode not in self.MODES:
            raise ValueError("rate gate mode %r" % (mode,))
        if not (rate_hz > 0 and nominal_fps > 0 and min_step_ms >= 0 and rebase_s > 0):
            raise ValueError("rate gate: rate_hz, nominal_fps, rebase_s > 0 and min_step_ms >= 0")
        self.mode, self.min_step_ns = mode, int(round(min_step_ms * 1e6))
        self.switch = mode if switch is None else switch   # the --rate-gate value (auto | stamp | arrival)
        self.basis = basis                                  # why ``mode`` is in effect (record only)
        self.target_ns = 1e9 / float(rate_hz)
        self.nominal_period_ns = 1e9 / float(nominal_fps)
        self.rebase_ns = int(rebase_s * 1e9)
        self.last_stamp_ns = self.last_raw_ns = None
        self.regress_since_raw_ns = None
        self.k = None
        self.threshold_ns = None
        self.last_step_ns = None
        self.counts = collections.Counter()   # released, rate, stamp_repeat, stamp_older, rebase, ...

    def threshold(self, period_ns=None):
        """(k, threshold ns) for the stamp mode at this source period (None = nominal)."""
        if self.min_step_ns > 0:
            return None, self.min_step_ns
        p = float(period_ns) if period_ns else self.nominal_period_ns
        k = max(1, int(math.ceil(self.ARRIVAL_FACTOR * self.target_ns / p - 1e-9)))
        return k, (k - 0.5) * p

    def _release(self, stamp_ns, raw_ns, reason=None):
        self.last_step_ns = None if self.last_stamp_ns is None else stamp_ns - self.last_stamp_ns
        self.last_stamp_ns, self.last_raw_ns = stamp_ns, raw_ns
        self.regress_since_raw_ns = None
        self.counts["released"] += 1
        if reason is not None:
            self.counts[reason] += 1
        return True, reason, None

    def decide(self, stamp_ns, raw_ns, period_ns=None):
        stamp_ns, raw_ns = int(stamp_ns), int(raw_ns)
        if self.mode == "arrival":
            if self.last_raw_ns is not None and raw_ns - self.last_raw_ns < self.ARRIVAL_FACTOR * self.target_ns:
                self.counts["rate_arrival"] += 1
                return False, "rate_arrival", None
            return self._release(stamp_ns, raw_ns)
        self.k, self.threshold_ns = self.threshold(period_ns)
        if self.last_stamp_ns is None:
            return self._release(stamp_ns, raw_ns)
        step = stamp_ns - self.last_stamp_ns
        if step == 0:
            self.counts["stamp_repeat"] += 1
            return False, "stamp_repeat", None
        if step < 0:
            ev = None
            if self.regress_since_raw_ns is None:
                self.regress_since_raw_ns = raw_ns
                self.counts["regression_episodes"] += 1
                ev = {"event": "depth_stamp_regression", "source": "geo_producer", "step_ms": step / 1e6,
                      "stamp_ns": stamp_ns, "last_released_stamp_ns": self.last_stamp_ns, "raw_ns": raw_ns}
            elif raw_ns - self.regress_since_raw_ns >= self.rebase_ns:
                since = self.regress_since_raw_ns
                prev = self.last_stamp_ns
                out = self._release(stamp_ns, raw_ns, "rebase")
                return out[0], out[1], {"event": "depth_stamp_rebase", "source": "geo_producer",
                                        "step_ms": step / 1e6, "stamp_ns": stamp_ns,
                                        "last_released_stamp_ns": prev,
                                        "regressed_for_ms": (raw_ns - since) / 1e6, "raw_ns": raw_ns}
            self.counts["stamp_older"] += 1
            return False, "stamp_older", ev
        self.regress_since_raw_ns = None
        if step < self.threshold_ns:
            self.counts["rate"] += 1
            return False, "rate", None
        return self._release(stamp_ns, raw_ns)

    def stats(self):
        """Small dict for every /fpga/kv_link/frame_stats body and the summary (cumulative counts)."""
        return {"mode": self.mode, "switch": self.switch, "basis": self.basis, "k": self.k,
                "threshold_ms": None if self.threshold_ns is None else round(self.threshold_ns / 1e6, 3),
                "step_ms": None if self.last_step_ns is None else round(self.last_step_ns / 1e6, 3),
                "released": self.counts["released"], "rebases": self.counts["rebase"],
                "skipped": {"rate": self.counts["rate"] + self.counts["rate_arrival"],
                            "stamp_repeat": self.counts["stamp_repeat"],
                            "stamp_older": self.counts["stamp_older"]}}


TF_LISTENER_DEPTH = 100     # tf2_ros TransformListener's own default history depth


def tf_listener_qos(qos, best_effort=True, depth=TF_LISTENER_DEPTH):
    """QoS of the producer's tf2 listener for the DYNAMIC /tf topic (``qos`` = rclpy.qos).

    best_effort (default): KEEP_LAST ``depth``, VOLATILE, BEST_EFFORT. A best-effort reader
    matches a reliable /tf writer (DDS: offered RELIABLE >= requested BEST_EFFORT; VOLATILE
    requested is satisfied by any durability), so nothing on the publishing side changes.
    Why: a reliable reader delivers in order, so one sample lost on the wire (UDP receive
    buffer overflow next to the 614 KB depth images) holds back every later /tf sample until
    the writer's heartbeat-driven resend (Fast DDS default heartbeat 3 s); best effort skips
    the lost sample and keeps the following ones. Semantics are unchanged: tf2 still
    interpolates only between two real samples bracketing the depth stamp and never
    extrapolates; a frame without a bracketing pair is still dropped (tf_missing), never
    sent with an older pose.
    best_effort=False: the tf2_ros default (RELIABLE, same depth/durability).

    /tf_static is not touched: the caller passes no ``static_qos`` so tf2_ros keeps its
    default (RELIABLE + TRANSIENT_LOCAL: the latched static frames must arrive, late joiners
    included). Returns the QoSProfile for ``TransformListener(qos=...)``.
    """
    rel = qos.ReliabilityPolicy.BEST_EFFORT if best_effort else qos.ReliabilityPolicy.RELIABLE
    return qos.QoSProfile(depth=int(depth), history=qos.HistoryPolicy.KEEP_LAST,
                          durability=qos.DurabilityPolicy.VOLATILE, reliability=rel)


DEPTH_SUB_DEPTH = 5        # qos_profile_sensor_data's history depth


def depth_sub_qos(qos, reliable=False, depth=DEPTH_SUB_DEPTH):
    """QoS of the producer's depth image and depth CameraInfo subscriptions (``qos`` = rclpy.qos).

    reliable=False (default): ``qos.qos_profile_sensor_data`` itself (BEST_EFFORT, KEEP_LAST 5,
    VOLATILE), exactly as before 2026-10-15.
    reliable=True (2026-10-15, camera-drop batch 2): KEEP_LAST ``depth``, VOLATILE, RELIABLE. On the
    board every best-effort reader of the real camera lost frames that the DDS lost count matched
    one for one (5.9-33.8 %), a reliable reader next to it lost none (arm l_b1, driver only). Cost
    to watch: a reliable reader delivers in order, so a sample lost on the wire holds back the
    following ones until the writer resends it (see tf_listener_qos); the producer's frame-age gate
    still drops a frame that arrives too old.
    """
    if not reliable:
        return qos.qos_profile_sensor_data
    return qos.QoSProfile(depth=int(depth), history=qos.HistoryPolicy.KEEP_LAST,
                          durability=qos.DurabilityPolicy.VOLATILE,
                          reliability=qos.ReliabilityPolicy.RELIABLE)


# 10-28 (owner: a box stayed on the car screen after the map dropped the object): a row whose track is
# position_invalid is still checked (the existence score takes its votes) but its AR name starts with this
# mark and kv_ar does not draw it -- the car screen follows the console map, which hides those tracks.
AR_HIDE_MARK = "~"


def roi_row_labels(memory_msg, plan):
    """Class name of the memory track behind each ROI row, in row order (None for a row whose
    track has no label; kv_link.clean_wire_label turns it into an empty AR name). A position_invalid
    track's name carries AR_HIDE_MARK in front (display only)."""
    by_id = {}
    for t in (memory_msg or {}).get("tracks") or ():
        if isinstance(t, dict):
            label = t.get("label")
            if t.get("state") == "position_invalid":
                label = AR_HIDE_MARK + (label if isinstance(label, str) else "")
            by_id[t.get("id")] = label
    return [by_id.get(plan.row_map.get(r.row_id)) for r in plan.batch.rows]


class GeoProducer(object):
    """Per-frame geometry for the rolling map (and the frame's ROI answer).

    ``backend``: a frame backend (RemotePlFrameBackend for the KV260, or the CPU one for a
    reference arm); ``rolling_backend``: the rolling-map backend (CPU host kernel on the
    Nano). ``clock`` is fpga_clock.now_ns (injectable for tests).
    """

    def __init__(self, backend, rolling_backend, *, depth_unit_m, stream=None,
                 voxel_m=VOXEL_M, shape=DEFAULT_SHAPE, height_cell=DEFAULT_HEIGHT_CELL,
                 back_cells=DEFAULT_BACK_CELLS, max_frame_age_ns=MAX_FRAME_AGE_NS,
                 clock=now_ns, clock_id=CLOCK_ID, local_frame=LOCAL_FRAME,
                 pose_epoch=POSE_EPOCH, world_frame="map", cfg=None, roi=True,
                 roi_labels=True, depth_oor="invalidate", ground_z_m=None,
                 oor_frac_max=DEFAULT_OOR_FRAC_MAX, oor_near_m=DEFAULT_OOR_NEAR_M, depth_stride=1):
        from host.m7_grid_policy import FollowCameraGrid
        from host.m7_pipeline import GridSpec, M7Pipeline
        from host.m7_rolling import RollingFusion
        if cfg is None:
            from perception.fpga_replay.core import Config
            cfg = Config()
        if not (isinstance(max_frame_age_ns, int) and 0 < max_frame_age_ns <= MAX_FRAME_AGE_NS):
            raise ValueError("max_frame_age_ns must be in (0, 200 ms] (D-035; never relaxed)")
        self.depth_unit_m = float(depth_unit_m)
        self.unit_f32 = np.float32(self.depth_unit_m)
        self.voxel_m, self.shape = float(voxel_m), tuple(int(v) for v in shape)
        self.policy = FollowCameraGrid(self.voxel_m, self.shape, back_cells=back_cells,
                                       height_cell=height_cell)
        self.pipeline = M7Pipeline(backend, GridSpec((0.0, 0.0, 0.0), self.voxel_m, self.shape),
                                   grid_policy=self.policy, max_frame_age_ns=max_frame_age_ns)
        self.fusion = RollingFusion(rolling_backend, voxel_m=self.voxel_m)
        self.max_frame_age_ns = max_frame_age_ns
        self.clock, self.clock_id = clock, clock_id
        self.local_frame, self.pose_epoch = local_frame, pose_epoch
        self.world_frame = world_frame
        self.stream = stream or "geoprod-%d-%s" % (os.getpid(), os.urandom(3).hex())
        # --kv-depth-stride (2026-10-16): the KV260 gets the decimated frame (prepare_depth); the ROI boxes
        # are planned on it too (same shape and K), so the ROI core and the plan share one pixel grid. The
        # ROI pixel floor (cfg.min_pixels 9) is NOT scaled: the core refuses min_pixels < 9 (status 4,
        # hw/m7/object_evidence.cpp), so a box needs 9 decimated pixels (= 36 full ones) for a verdict
        self.depth_stride = int(depth_stride)
        if self.depth_stride < 1:
            raise ValueError("depth_stride must be >= 1")
        self.cfg = cfg
        self.roi = bool(roi)
        self.roi_labels = bool(roi_labels)
        # Out-of-range depth (DEPTH_OOR_INVALID_20261016): "invalidate" marks > 20 m pixels as no
        # return unless a gate fires; "reject" = the frame refusal as before. ground_z_m None:
        # the near band cannot be judged -> every OOR frame is refused (gate "no_ground").
        if depth_oor not in OOR_MODES:
            raise ValueError("depth_oor must be one of %s" % (OOR_MODES,))
        if not (0.0 <= float(oor_frac_max) <= 1.0) or not float(oor_near_m) >= 0.0:
            raise ValueError("oor_frac_max must be in [0, 1] and oor_near_m >= 0")
        self.depth_oor = depth_oor
        self.ground_z_m = None if ground_z_m is None else float(ground_z_m)
        self.oor_frac_max, self.oor_near_m = float(oor_frac_max), float(oor_near_m)
        self.frame_seq = 0
        self.counts = collections.Counter()

    @property
    def backend(self):
        return self.pipeline.backend

    def set_backend(self, backend):
        """After a KV reconnect: same pipeline generation, so the rolling map keeps its epoch."""
        self.pipeline.backend = backend

    def describe(self):
        return {"stream": self.stream, "voxel_m": self.voxel_m, "shape_xyz": list(self.shape),
                "height_cell": self.policy.height_cell, "back_cells": self.policy.back_cells,
                "max_frame_age_ms": self.max_frame_age_ns / 1e6, "clock_id": self.clock_id,
                "depth_stride": self.depth_stride, "roi_min_pixels": self.cfg.min_pixels,
                "local_frame": self.local_frame, "world_frame": self.world_frame,
                "rolling_dims": list(self.fusion.backend.config.dims),
                "depth_oor": {"mode": self.depth_oor, "max_m": OOR_MAX_M, "frac_max": self.oor_frac_max,
                              "near_m": self.oor_near_m, "ground_z_m": self.ground_z_m},
                "backend": getattr(self.backend, "backend_name", "cpu_reference")}

    def prepare_depth(self, codes, K):
        """(codes, K) as the KV260 is to see them: decimated by ``depth_stride`` (block minimum of
        the valid codes) with the intrinsics scaled to match; unchanged at stride 1."""
        if self.depth_stride == 1:
            return codes, K
        self.counts["depth_decimated"] += 1
        return decimate_depth_codes(codes, self.depth_stride), scale_intrinsics(K, self.depth_stride)

    def process(self, codes, K, *, depth_stamp_ns, acquired_ns, T_odom_camera, robot_position_m,
                calibration_id, T_world_camera=None, memory_msg=None, memory_reason=None):
        """One frame. Returns a dict: ``delta`` (NavigationDelta or None), ``body`` (the
        /fpga/geometry_frame JSON-able body), ``stats`` (frame_stats inputs) and stamps."""
        from host.geometry_region_glue import plan_frame, verdict_message
        from host.kv_link import codes_to_m
        from host.m7_pipeline import CaptureIdentity, DepthFrame, PoseAtCapture
        from host.region_verdicts import build_message
        self.frame_seq += 1
        fid = self.frame_seq
        self.counts["frames"] += 1
        codes = np.ascontiguousarray(codes, dtype=np.uint16)
        depth_m = codes_to_m(codes, self.unit_f32)
        K = np.asarray(K, dtype=np.float64)
        oor_mask, oor = screen_out_of_range(depth_m, K, T_odom_camera, mode=self.depth_oor,
                                            ground_z_m=self.ground_z_m, frac_max=self.oor_frac_max,
                                            near_m=self.oor_near_m)
        if oor_mask is not None:
            # codes AND depth: the KV260 converts the codes and its depth crc must match depth_m.
            # The 16UC1 codes are a read-only view of the message buffer: copy first.
            codes = codes.copy()
            codes[oor_mask] = OOR_INVALID_CODE
            depth_m[oor_mask] = codes_to_m(np.array([OOR_INVALID_CODE], np.uint16), self.unit_f32)[0]
        if oor["oor_pixels"]:
            self.counts["oor:" + oor["oor_action"]] += 1
        backend = self.backend
        remote = callable(getattr(backend, "set_next_codes", None))
        plan = None
        roi_skip = None
        if not self.roi:
            roi_skip = "roi_disabled"
        elif not remote or not getattr(backend, "roi_enabled", True):
            roi_skip = "roi_unavailable"
        elif memory_msg is None:
            roi_skip = memory_reason or "no_memory"
        elif T_world_camera is None:
            roi_skip = "no_world_pose"
        elif not np.any(np.isfinite(depth_m) & (depth_m > .1) & (depth_m < 10)):
            # 10-26 (#2 follow-up): an all-invalid frame now still sends its (blank) geometry request; its ROI
            # answer is not evidence about any object, so no ROI rides along -- the memory sees exactly what it
            # saw before 10-26 (no verdicts for this frame). Same predicate as M7Pipeline's all_invalid.
            roi_skip = "no_valid_depth"
        else:
            plan = plan_frame(memory_msg, depth_m.shape, K, np.asarray(T_world_camera, dtype=np.float64),
                              self.cfg, counts=self.counts, ground_z_m=self.ground_z_m)
        if roi_skip:
            self.counts["roi_skipped:" + roi_skip] += 1
        if remote:
            backend.last_roi = None
            batch = plan.batch if plan is not None else None
            labels = roi_row_labels(memory_msg, plan) if batch is not None and self.roi_labels else None
            if labels is not None:
                backend.set_next_roi(batch, labels=labels)
                self.counts["roi_labels_sent"] += 1
            else:
                backend.set_next_roi(batch)
            backend.set_next_codes(codes)
            if callable(getattr(backend, "set_next_sensor_key", None)):
                # AR colour bypass (host/kv_colour.py): the camera header stamp pairs the colour frame;
                # sent only when the KV announces stamp_ext, else the request bytes are unchanged
                backend.set_next_sensor_key(depth_stamp_ns if depth_stamp_ns and depth_stamp_ns > 0 else None)
            backend.lib.last = None
        ident = CaptureIdentity(self.stream, fid, int(acquired_ns), self.clock_id, calibration_id,
                                self.local_frame, self.pose_epoch)
        pose = PoseAtCapture(int(acquired_ns), self.clock_id, self.local_frame, self.pose_epoch, True,
                             np.asarray(T_odom_camera, dtype=np.float64))
        t_start = self.clock()
        try:
            out = self.pipeline.ingest_geometry(DepthFrame(ident, depth_m, K, pose), now_ns=t_start,
                                                now_clock_id=self.clock_id)
            nav = out.navigation
            nav_valid, nav_reason, grid = nav.valid, nav.reason, nav.payload.get("grid")
        except ValueError as exc:
            nav, nav_valid, nav_reason, grid = None, False, "input_refused:%s" % str(exc)[:60], None
        t_geo = self.clock()
        if remote:
            backend.set_next_roi(None)          # never left for the next frame
            backend.set_next_codes(None)
            if callable(getattr(backend, "set_next_sensor_key", None)):
                backend.set_next_sensor_key(None)
        if nav_reason in RESET_REASONS:
            self.pipeline.reset()
            self.counts["pipeline_reset"] += 1
        delta, fusion_reason, pl_grid, n_changes = None, None, None, None
        if nav is not None:
            try:
                fused = self.fusion.ingest(nav, self.pipeline, now_ns=t_geo, clock_id=self.clock_id,
                                           robot_position_m=tuple(float(v) for v in robot_position_m))
            except RuntimeError as exc:
                # 10-26 owner：重成功率（#3a 自愈）。Before, this raised out of the frame: the host bookkeeping
                # stayed out of step with the rolling map ("sequence diverged"), so every later frame raised
                # the same way -- with WORKER_ERROR_LIMIT 0 the node lived on publishing no delta, forever.
                # Now the frame is dropped and the pipeline reset (new generation): the next frame bumps
                # the map epoch and resets the rolling map (the RESET_REASONS path), so deltas resume.
                fused = None
                fusion_reason = "fusion_error:%s" % type(exc).__name__
                self.pipeline.reset()
                self.counts["pipeline_reset"] += 1
                self.counts["fusion_error_recovered"] += 1
                n = self.counts["fusion_error_recovered"]
                if n == 1 or n % 100 == 0:
                    import sys
                    sys.stderr.write("WARNING geo producer: fusion error (%r), map rebuilt on a new epoch "
                                     "(%d so far)\n" % (exc, n))
            if fused is not None:
                fusion_reason = fused.reason
                if fused.accepted:
                    delta = fused.delta
                    pl_grid = fused.grid              # B1: the PL navigation grid (None unless enable_grid)
                    n_changes = len(delta.changes)
                    if not delta.changes and fused.kernel is not None and fused.kernel.changed_cells:
                        n_changes = fused.kernel.changed_cells   # B1 "on": change list not decoded
        t_fused = self.clock()
        self.counts["nav:" + str(nav_reason).split(":", 1)[0]] += 1
        self.counts["fusion:" + str(fusion_reason).split(":", 1)[0]] += 1
        client_last = getattr(getattr(backend, "lib", None), "last", None) if remote else None
        if remote:
            backend.lib.last = None
        # ROI answer of this frame; withdrawn when the frame is out of the 200 ms budget
        answered_age = t_geo - int(acquired_ns)
        expired = (str(nav_reason).startswith("expired") or str(fusion_reason).startswith("expired")
                   or answered_age > self.max_frame_age_ns)
        T_world = (np.eye(4) if T_world_camera is None else np.asarray(T_world_camera, dtype=np.float64))
        if plan is not None:
            outcome = backend.last_roi if remote else None
            if expired:
                body = build_message(depth_stamp_ns=depth_stamp_ns, capture_nano_ns=int(acquired_ns),
                                     kv_frame_id=fid, world_frame=self.world_frame, T_world_camera=T_world,
                                     pose_valid=T_world_camera is not None,
                                     runtime_boot_id=memory_msg.get("runtime_boot_id"),
                                     memory_revision=memory_msg.get("revision"),
                                     failure="expired_output")
            elif plan.batch is not None and outcome is None:
                body = build_message(depth_stamp_ns=depth_stamp_ns, capture_nano_ns=int(acquired_ns),
                                     kv_frame_id=fid, world_frame=self.world_frame, T_world_camera=T_world,
                                     pose_valid=True, runtime_boot_id=memory_msg.get("runtime_boot_id"),
                                     memory_revision=memory_msg.get("revision"),
                                     failure="geometry_not_run:%s" % nav_reason)
            else:
                body = verdict_message(plan, outcome, memory_msg=memory_msg, depth_stamp_ns=depth_stamp_ns,
                                       capture_nano_ns=int(acquired_ns), kv_frame_id=fid,
                                       world_frame=self.world_frame, T_world_camera=T_world,
                                       pose_valid=True, cfg=self.cfg)
        else:
            body = build_message(depth_stamp_ns=depth_stamp_ns, capture_nano_ns=int(acquired_ns),
                                 kv_frame_id=fid, world_frame=self.world_frame, T_world_camera=T_world,
                                 pose_valid=T_world_camera is not None,
                                 runtime_boot_id=(memory_msg or {}).get("runtime_boot_id"),
                                 memory_revision=(memory_msg or {}).get("revision"),
                                 failure="roi_skipped:%s" % roi_skip)
        self.counts["roi:" + ("ok" if body["failure"] is None else body["failure"].split(":", 1)[0])] += 1
        body["producer"] = {
            "frame_id": fid, "stream": self.stream, "depth_stamp_ns": int(depth_stamp_ns),
            "acquired_raw_ns": int(acquired_ns), "clock_id": self.clock_id,
            "geometry_start_raw_ns": int(t_start), "geometry_done_raw_ns": int(t_geo),
            "fusion_done_raw_ns": int(t_fused), "nav_valid": bool(nav_valid), "nav_reason": nav_reason,
            "fused": delta is not None, "fusion_reason": fusion_reason,
            "epoch": None if delta is None else delta.epoch,
            "delta_seq": None if delta is None else delta.delta_seq,
            "n_changes": n_changes,
            "grid_origin_m": None if grid is None else [float(v) for v in grid.origin_m],
            "backend": getattr(backend, "backend_name", "cpu_reference"),
            "expired": bool(expired),
            # 10-27: pixel frame of the ROI rows' roi_xyxy (the console's first-person boxes scale by it)
            "depth_wh": [int(np.shape(codes)[1]), int(np.shape(codes)[0])] if np.ndim(codes) == 2 else None}
        if getattr(self.cfg, "geometry_v2", False):
            # 10-27 geometry v2 (Config.geometry_v2): the runtime checks it runs the same version; v1 adds nothing
            body["producer"]["geometry_version"] = "v2"
        result = "fused" if delta is not None else "dropped:%s" % (fusion_reason or nav_reason)
        payload = nav.payload if nav is not None else {}
        # 10-29 console FPGA panel (host/fpga_metrics.py, display only): this frame's ROI answer, and the PL times
        # of this frame's INTEGRATE (the fusion backend's last ROLL answer; None when the frame was not fused)
        roll_times = getattr(self.fusion.backend, "last_times", None) if delta is not None else None
        return {"frame_id": fid, "delta": delta, "body": body, "result": result, "oor": oor, "pl_grid": pl_grid,
                "roi_outcome": backend.last_roi if remote else None,
                "roll_times_ns": dict(roll_times) if isinstance(roll_times, dict) else None,
                "roll_core": getattr(self.fusion.backend, "kv_core", None),
                "client_last": client_last, "t_start": t_start, "t_geo": t_geo, "t_fused": t_fused,
                "nav_reason": nav_reason, "fusion_reason": fusion_reason,
                # per-frame block as the backend returned it (gates compare it cell by cell)
                "grid": grid, "voxel_flags": payload.get("voxel_flags"),
                "nav_counts": {k: payload.get(k) for k in ("valid_grid_samples", "invalid_depth_or_projection",
                                                           "outside_grid")} if grid is not None else None}
