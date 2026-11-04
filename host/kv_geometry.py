"""Car runtime geometry on the KV260 (``ros_runtime --geometry-source kv``, 2026-10-11).

``compute_geometry(frame, memory)`` (perception/fpga_replay/core.py) answers one word per
memory track from the frame's depth. ``KvGeometry.evaluate`` answers the same dict, but
the per-box depth check runs on the KV260 (region core, PL or KV CPU) through the B1
kv-link channel, together with the frame's local voxel grid (geometry core):

  1. project every track box with package 1's ``plan_boxes`` (the projection
     ``check_boxes`` uses; out_of_view / too_small are decided on the Nano, exactly as
     compute_geometry decides them);
  2. send the depth (the camera's own uint16 codes when available, no requantization)
     and <= 32 boxes in one request; window 1, bounded timeout;
  3. every checked row -> the core.py word via ``geometry_region_glue.evidence_word``
     (the same branches as ``check_boxes``).

Fail-closed: any failure (no HELLO answer, feature/roi core missing, BUILD_ID differs,
timeout, superseded, depth not representable, crc mismatch, ROI answer void) returns
``evidence=None`` with a reason; nothing here ever computes geometry on the Nano. Whether
the caller then uses CPU geometry is the caller's explicit switch (``--geometry-fallback``).

Known differences from compute_geometry, counted, never hidden:
  * > 32 in-view boxes: the extra tracks are ``overflow`` (absent from the dict, so
    ``update_memory`` treats them as "unknown"); compute_geometry checks them all.
  * NumPy >= 2 compares depth against the box's float64 front/back; the kernel compares
    in float32 (NumPy 1.x value-based casting, as on the Nano image, matches the kernel).
    tests/test_m7_object_roi.py ``test_threshold_boundary_float32_vs_float64`` shows it.

Stamps that make up the frame age are ``host.fpga_clock.now_ns`` (CLOCK_MONOTONIC_RAW);
ROS header stamps are never subtracted from them. Python 3.8 compatible, no ROS import.
"""

from collections import Counter, namedtuple
import os

import numpy as np

from host.fpga_clock import CLOCK_ID, now_ns

DEFAULT_GRID = ((-0.8, -0.8, -0.05), 0.05, (32, 32, 32))   # rec30 / TUM prepared (32^3 x 5 cm)
LOCAL_FRAME = "camera_local"            # grid in the camera frame (camera_to_world = I)
POSE_EPOCH = "camera_local_v1"

GeometryOutcome = namedtuple("GeometryOutcome", "evidence reason info")


def parse_grid(text):
    """'ox,oy,oz,voxel,nx,ny,nz' -> ((ox,oy,oz), voxel, (nx,ny,nz))."""
    parts = [p.strip() for p in str(text).split(",")]
    if len(parts) != 7:
        raise ValueError("grid needs ox,oy,oz,voxel,nx,ny,nz")
    origin = tuple(float(v) for v in parts[:3])
    voxel = float(parts[3])
    shape = tuple(int(v) for v in parts[4:])
    if not (voxel > 0 and np.isfinite(voxel)) or any(not 1 <= n <= 32 for n in shape):
        raise ValueError("voxel must be > 0 and every grid dimension in 1..32")
    return origin, voxel, shape


def parse_build_id(text):
    if text is None:
        return None
    return int(str(text), 0)


def track_entries(memory):
    """(track_id, bounds(2,3)) of every memory track, in memory order (compute_geometry's)."""
    return [(t.id, np.array([np.asarray(t.lower, dtype=np.float64),
                             np.asarray(t.upper, dtype=np.float64)])) for t in memory.tracks.values()]


def entries_digest(entries):
    import hashlib
    h = hashlib.sha256()
    for tid, b in entries:
        h.update(str(tid).encode("utf-8"))
        h.update(np.ascontiguousarray(b, dtype="<f8").tobytes())
    return h.hexdigest()


class KvGeometry(object):
    """One kv-link session for the runtime's per-frame geometry.

    ``client_factory(server) -> KvLinkClient`` and ``telemetry_factory(server) ->
    KvTelemetryClient`` are injectable (tests); ``clock`` is fpga_clock.now_ns.
    """

    def __init__(self, server, *, depth_unit_m, timeout_s=0.3, hello_timeout_s=0.5,
                 retry_s=5.0, grid=DEFAULT_GRID, expect_build_id=None, chunk_bytes=None,
                 stream_tag="runtime", clock=now_ns, client_factory=None, telemetry_factory=None,
                 reconnect_after_failures=1):
        from host import kv_link
        self._kv = kv_link
        self.server = (server[0], int(server[1]))
        self.depth_unit_m = float(depth_unit_m)
        if not (self.depth_unit_m > 0 and np.isfinite(self.depth_unit_m)):
            raise ValueError("depth unit must be a positive finite number of metres")
        self.timeout_s = float(timeout_s)
        self.hello_timeout_s = float(hello_timeout_s)
        self.retry_s = float(retry_s)
        self.grid = grid
        self.expect_build_id = expect_build_id
        self.chunk_bytes = kv_link.CHUNK_FRAGMENTED if chunk_bytes is None else int(chunk_bytes)
        self.clock = clock
        self.reconnect_after_failures = int(reconnect_after_failures)
        self._client_factory = client_factory or (lambda srv: kv_link.KvLinkClient(
            srv, chunk_bytes=self.chunk_bytes, timeout_s=self.timeout_s))
        self._telemetry_factory = telemetry_factory or (lambda srv: kv_link.KvTelemetryClient(
            srv, timeout_s=0.2, hello_timeout_s=self.hello_timeout_s))
        self.stream = "kvgeo-%s-%d-%s" % (stream_tag, os.getpid(), os.urandom(3).hex())
        self.backend = None
        self.client = None
        self.server_info = None
        self.build_id = None
        self.available = False
        self.reason = "not_connected"
        self.last_attempt_ns = None
        self.connects = 0
        self.consecutive_failures = 0
        self.seq = 0
        self.counts = Counter()

    # -- connection
    def _drop(self):
        if self.client is not None:
            self.counts.update({"link:" + k: v for k, v in self.client.counts.items()})
            try:
                self.client.close()
            except Exception:
                pass
        self.backend = self.client = None

    def close(self):
        self._drop()

    def connect(self):
        """HELLO + roi_ext + roi core present + BUILD_ID (when expected). Sets
        ``available``/``reason``; returns True when the backend is usable."""
        kv = self._kv
        self._drop()
        self.last_attempt_ns = self.clock()
        self.counts["connect_attempts"] += 1
        client = None
        stage, tel = "hello", None
        try:
            client = self._client_factory(self.server)
            info = client.hello(self.hello_timeout_s)
            feats = info.get("features") if isinstance(info, dict) else None
            if not isinstance(feats, dict) or feats.get("roi_ext") != 1:
                raise _Refused("feature_unavailable:roi_ext")
            if info.get("roi_core") in (None, "none"):
                raise _Refused("kv_roi_core_none")
            if int(info.get("roi_max_boxes", 0)) < 1:
                raise _Refused("kv_roi_max_boxes")
            build_id = None
            if self.expect_build_id is not None:
                if not isinstance(feats, dict) or feats.get("telem") != 1:
                    raise _Refused("build_id_unchecked:no_telem")
                stage = "telem"
                tel = self._telemetry_factory(self.server)
                link = getattr(tel, "link", None)
                if link is not None and getattr(link, "server_info", None) is None:
                    link.server_info = dict(info)   # same server, answered just now: no second HELLO
                try:
                    board = tel.poll(kv.blocks_mask(("board",))).blocks.get("board") or {}
                finally:
                    tel.close()
                build_id = board.get("build_id")
                if build_id is None:
                    raise _Refused("build_id_unavailable:%s" % board.get("build_id_reason"))
                if int(build_id) != int(self.expect_build_id):
                    raise _Refused("build_id_mismatch:%#010x!=%#010x" % (int(build_id),
                                                                          int(self.expect_build_id)))
        except _Refused as exc:
            if client is not None:
                client.close()
            return self._unavailable(str(exc))
        except (kv.LinkError, OSError, ValueError, KeyError, TypeError) as exc:
            if client is not None:
                client.close()
            if not isinstance(exc, kv.LinkTimeout):
                name = "connect_error:%s" % type(exc).__name__
            elif stage == "hello":
                name = "hello_timeout"
            elif (getattr(tel, "counts", None) or {}).get("telem_timeout"):
                name = "telem_timeout"              # the build-id TELEM poll went unanswered
            else:
                name = "telem_hello_timeout"        # the telemetry client's own HELLO
            return self._unavailable(name)
        self.client = client
        self.server_info = dict(info)
        self.build_id = build_id
        self.backend = kv.RemotePlFrameBackend(client, depth_unit_m=self.depth_unit_m)
        self.available, self.reason = True, None
        self.connects += 1
        self.consecutive_failures = 0
        return True

    def _unavailable(self, reason):
        self.available, self.reason = False, reason
        self.counts["unavailable:" + reason.split(":", 1)[0]] += 1
        return False

    def describe(self):
        info = self.server_info or {}
        return {"server": "%s:%d" % self.server, "available": self.available, "reason": self.reason,
                "core": info.get("core"), "roi_core": info.get("roi_core"),
                "kv_boot_id": info.get("kv_boot_id"),
                "build_id": None if self.build_id is None else "%#010x" % int(self.build_id),
                "expect_build_id": (None if self.expect_build_id is None
                                    else "%#010x" % int(self.expect_build_id)),
                "depth_unit_m": self.depth_unit_m, "timeout_s": self.timeout_s,
                "grid": {"origin_m": list(self.grid[0]), "voxel_m": self.grid[1],
                         "shape_xyz": list(self.grid[2])}}

    # -- one frame
    def evaluate(self, frame, memory, *, codes=None, rx_ns=None, cfg=None):
        """GeometryOutcome(evidence dict | None, reason | None, info dict)."""
        from host.geometry_region_glue import evidence_word
        from host.m7_object_native import plan_boxes
        from host.m7_pipeline import CaptureIdentity
        if cfg is None:
            from perception.fpga_replay.core import Config
            cfg = Config()
        kv = self._kv
        t_start = self.clock()
        info = {"rx_ns": rx_ns, "start_ns": t_start, "kv_frame_id": None, "roi_core": None,
                "rows": [], "host_resolved": 0, "overflow": [], "sent": False}
        frame.validate()
        entries = track_entries(memory)
        info["tracks"] = len(entries)
        if not frame.pose_valid:             # compute_geometry's first branch, nothing to send
            info["done_ns"] = self.clock()
            return GeometryOutcome({tid: "invalid_pose" for tid, _ in entries}, None, info)
        if self.backend is None:
            if self.last_attempt_ns is not None and t_start - self.last_attempt_ns < self.retry_s * 1e9:
                self.counts["frames_unavailable"] += 1
                info["done_ns"] = self.clock()
                return GeometryOutcome(None, self.reason or "not_connected", info)
            if not self.connect():
                self.counts["frames_unavailable"] += 1
                info["done_ns"] = self.clock()
                return GeometryOutcome(None, self.reason, info)
        plan = plan_boxes(frame.depth.shape, frame.K, frame.T_world_camera, entries, cfg)
        info["host_resolved"] = len(plan.host_resolved)
        info["overflow"] = list(plan.overflow)
        if plan.overflow:
            self.counts["overflow_tracks"] += len(plan.overflow)
        self.seq += 1
        acquired = int(rx_ns) if rx_ns is not None else t_start
        calib = frame.calibration_id or "unknown"
        ident = CaptureIdentity(self.stream, self.seq, acquired, CLOCK_ID, calib, LOCAL_FRAME, POSE_EPOCH)
        info["kv_frame_id"] = self.seq
        origin, voxel, shape = self.grid
        info["send_ns"] = self.clock()
        reason = None
        try:
            self.backend.frame_from_depth(
                depth_m=frame.depth, k=frame.K, camera_to_world=np.eye(4), grid_origin_m=origin,
                voxel_size_m=voxel, shape=shape, frame_id=self.seq, capture_time_ns=acquired,
                pose_time_ns=acquired, calibration_version=calib, pose_confirmed=True,
                identity=ident, roi=plan.batch, codes=codes)
        except kv.LinkTimeout:
            reason = "link_timeout"
        except kv.LinkSuperseded:
            reason = "superseded"
        except kv.KvFeatureUnavailable:
            reason = "feature_unavailable:roi_ext"
        except ValueError as exc:
            text = str(exc)
            if "not on the float32 grid" in text or "uint16 code range" in text or "0 m is not" in text:
                reason = "depth_not_representable"
            elif "M7 frame rejected" in text:
                reason = "kv_geometry_rejected"
            else:
                reason = "request_invalid:%s" % text[:60]
        except RuntimeError as exc:
            reason = "depth_crc_mismatch" if "crc" in str(exc) else "kv_answer_invalid"
        except OSError as exc:
            reason = "socket_error:%s" % type(exc).__name__
        info["back_ns"] = self.clock()
        last = getattr(self.backend.lib, "last", None) if self.backend is not None else None
        self.backend.lib.last = None
        info["client_last"] = _plain_last(last)
        info["sent"] = last is not None       # the request was encoded and handed to the socket
        if reason is None and plan.batch is not None:
            ro = self.backend.last_roi
            if ro is None or not ro.ok:
                reason = ro.reason if ro is not None else "region_backend_failure:no_outcome"
            else:
                info["roi_core"] = ro.roi_core
                info["roi_times_ns"] = dict(ro.times_ns or {})
                info["roi_cycles"] = ro.roi_cycles
        if reason is not None:
            self.counts["frames_failed:" + reason.split(":", 1)[0]] += 1
            self.consecutive_failures += 1
            if reason in ("link_timeout", "superseded") or reason.startswith("socket_error"):
                if self.consecutive_failures >= self.reconnect_after_failures:
                    # next frame re-HELLOs (rate limited), which also re-checks BUILD_ID
                    self._drop()
                    self._unavailable(reason)
            info["done_ns"] = self.clock()
            return GeometryOutcome(None, reason, info)
        self.consecutive_failures = 0
        self.available, self.reason = True, None
        evidence = {}
        for tid, word in plan.host_resolved.items():
            evidence[tid] = word
            info["rows"].append({"track_id": tid, "word": word, "code": None, "xyxy": None, "counts": None})
        if plan.batch is not None:
            for sent, row in zip(plan.batch.rows, self.backend.last_roi.rows):
                tid = plan.row_map[sent.row_id]
                word = evidence_word(row, cfg)
                evidence[tid] = word
                info["rows"].append({"track_id": tid, "word": word, "code": int(row["verdict_code"]),
                                     "xyxy": [int(v) for v in sent.xyxy],
                                     "near_far": [float(sent.near_z), float(sent.far_z)],
                                     "counts": [int(row["roi_pixels"]), int(row["roi_valid_pixels"]),
                                                int(row["roi_front_pixels"]), int(row["roi_behind_pixels"])]})
        # dict order = memory order (compute_geometry's), so equal dicts compare equal as lists too
        order = {tid: i for i, (tid, _) in enumerate(entries)}
        evidence = dict(sorted(evidence.items(), key=lambda kv_: order[kv_[0]]))
        self.counts["frames_ok"] += 1
        info["done_ns"] = self.clock()
        return GeometryOutcome(evidence, None, info)


class _Refused(Exception):
    pass


def _plain_last(last):
    if not last:
        return None
    out = {}
    for k, v in last.items():
        if isinstance(v, dict):
            out[k] = {kk: int(vv) for kk, vv in v.items()}
        elif isinstance(v, (int, float, str)) or v is None:
            out[k] = v
    return out


def frame_stats_body(outcome, *, memory_ns=None, client_counts=None):
    """/fpga/kv_link/frame_stats body (spec D.2 keys; hub_core reads result/frame_age_ms/...)
    plus the runtime's chain stamps, all CLOCK_MONOTONIC_RAW ns on the Nano:
    rx (image callback) -> start (poll: geometry step begins) -> send -> back -> memory."""
    i = outcome.info
    last = i.get("client_last") or {}
    kv_ms = {k: v / 1e6 for k, v in (last.get("kv_ns") or {}).items()} or None
    rx = i.get("rx_ns")
    end = memory_ns if memory_ns is not None else i.get("done_ns")
    age_ms = (end - rx) / 1e6 if (rx is not None and end is not None) else None

    def seg(a, b):
        return (i[b] - i[a]) / 1e6 if i.get(a) is not None and i.get(b) is not None else None
    chain = {"rx_to_start_ms": seg("rx_ns", "start_ns"), "start_to_send_ms": seg("start_ns", "send_ns"),
             "send_to_back_ms": seg("send_ns", "back_ns"),
             "back_to_memory_ms": ((memory_ns - i["back_ns"]) / 1e6
                                   if memory_ns is not None and i.get("back_ns") is not None else None)}
    channel = None
    if age_ms is not None and kv_ms and kv_ms.get("core_device") is not None:
        channel = age_ms - kv_ms["core_device"]
    result = "fused" if outcome.reason is None else "backend_failure:%s" % outcome.reason
    return {"v": 1, "source": "ros_runtime", "kv_frame_id": i.get("kv_frame_id"),
            "capture_nano_ns": rx, "result": result, "frame_age_ms": age_ms,
            "channel_ms": channel, "mailbox_wait_ms": None,
            "rtt_ms": last["rtt_s"] * 1e3 if last.get("rtt_s") is not None else None,
            "kv_times_ms": kv_ms, "chain_ms": chain, "roi_core": i.get("roi_core"),
            "roi_times_ms": ({k: v / 1e6 for k, v in i["roi_times_ns"].items()}
                             if i.get("roi_times_ns") else None),
            "tracks": i.get("tracks"), "boxes_sent": sum(1 for r in i.get("rows", ()) if r["code"] is not None),
            "overflow": len(i.get("overflow") or ()), "client_counts": dict(client_counts or {})}
