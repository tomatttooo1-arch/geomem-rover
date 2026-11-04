"""Nano hub core without ROS (spec B, C.4, C.5, A.4 Nano side).

Everything here is deterministic given its inputs: every entry point takes the Nano
monotonic time ``nano_ns`` explicitly and nothing in this module reads a clock. The
rclpy adapter (hub_ros.py) and the TCP server (hub_server.py) supply real times; the
tests supply scripted ones, so ages are checked to the nanosecond.

Rules carried from the spec:
  * ages are (compose time - Nano receive time) on the Nano monotonic clock; ROS header
    stamps are passed through as ``*_ros_s`` and never subtracted from that clock (B.3);
  * every group has ``src``; nothing defaults to live (B.3, C.4);
  * values without a source are null with a reason, never invented (C.4);
  * PC e-stop is a latch that only an explicit release with the right epoch clears (A.4,
    D1-D3); find/select/goto are refused while latched (O3). O2 ("it never cancels the task")
    is REVOKED by the controller 10-17 (REAL_CAR_NAV_GOLIVE_20261016.md §3 item 4): an e-stop
    (a PC assert, or a KV latch the hub adopts) also publishes /fpga/cancel, so the runner
    cancels its Nav2 goal and drops the motion permit; after the release nothing resumes --
    the guard passes velocity again only once a NEW find/select/goto has re-armed the permit.
  * control (O8, controller override 10-11): any authenticated PC may assert e-stop;
    only the control holder may release, find, select, cancel or yield.
"""

import base64
import binascii
import collections
import json
import math
import os
import re
import zlib

from host.fpga_metrics import summarize as fpga_summarize
from host.hub.fpv_boxes import FpvBoxes, hidden_ids

# "sim" (line D1, owner 10-14): Gazebo through scripts/sim_adapter.py; always marked SIM, never LIVE.
DATA_SOURCES = ("live", "replay", "sim")
SENSOR_TO_SOURCE = {"hardware": "live", "simulation": "replay", "sim": "replay", "replay": "replay"}
# Which --data-source values a runner sensor_source agrees with. simulation/sim agree with replay
# (as before: recorded simulation, the no-ROS ReplayFeeder) and, since line D1, with sim.
SENSOR_SOURCES_OK = {"hardware": ("live",), "replay": ("replay",),
                     "simulation": ("replay", "sim"), "sim": ("replay", "sim")}


def sensor_agrees(sensor, source):
    """True when the runner's sensor_source is consistent with the frame/group source."""
    return source in SENSOR_SOURCES_OK.get(sensor, ())
ROS_GROUPS = ("task", "memory", "region", "pose", "map", "nav", "odom", "geometry_nano", "perf")
GROUP_INPUTS = collections.OrderedDict([
    ("task", ("status_task", "status_frame")),
    ("memory", ("memory",)),
    ("region", ("region",)),
    ("pose", ("loc_pose", "loc_status")),
    ("map", ("rolling_status", "map_global")),
    ("nav", ("motion_permit", "cmd_vel", "plan")),
    ("odom", ("odom", "odom_adapter")),
    ("kv", ("telemetry",)),
    ("estop", ()),
    ("clock", ("clock",)),
    ("geometry_nano", ("frame_stats",)),
    ("perf", ("frame_stats",)),
    ("nano", ("tegrastats", "nvpmodel")),
    ("hub", ()),
])
# 10-29 counters host/hub/health.py turns into rates (hub_ros per-topic callback counts, runner frames)
HEALTH_COUNTERS = ("frames_processed_seen", "cb:/fpga/preview/jpeg", "cb:/fpga/kv_link/frame_stats", "cb:/odom",
                   "cb:/fpga/chassis/status")
TERMINAL_TASK = frozenset(("found", "not_found", "unable_to_confirm", "failed", "cancelled",
                           "needs_manual_approach"))   # SELECT_SEMANTICS_20261016.md section 1
# task runner 200 ms geometry age gate refusals (host/geometry_age_gate.py; event kind = reason)
GEOMETRY_AGE_EVENTS = frozenset(("geometry_stale", "geometry_future", "geometry_clock_mismatch",
                                 "geometry_age_unknown"))
MEMORY_STATES = ("current_confirmed", "temporarily_occluded", "historical_memory", "position_invalid")
OPS = ("find", "select", "cancel", "estop", "release", "yield_control", "patrol", "cold_start_confirm", "goto",
       "teleop", "mapping_stop", "map_reset")
# teleop / mapping_stop (2026-10-18, docs/design/TELEOP_NAV_ARB_20261018.md section 3):
#  teleop        {"v": m/s, "w": rad/s} or {"stop": true}; controller only; non-zero refused while the e-stop is
#                latched; published as /fpga/teleop/input (String, the teleop socket's JSON + "sent_ns" = this hub's
#                nano_ns, host.fpga_clock, same machine as the teleop node); no watcher, and NOT kept in the dedup book
#                (10-20 Hz would push find/goto acks out of it): the ACK is "forwarded" or "rejected".
#                Limits (|v| <= TELEOP_V_CAP, |w| <= TELEOP_W_CAP) are the teleop node's caps; it clamps further.
#  mapping_stop  {} -> /fpga/mapping/stop (Empty); the in-stack mapping listener saves and exports the rtabmap
#                database and reports /fpga/mapping/status (String JSON, latched): RESULT "done" with the status
#                (exported | failed) or "unknown" after MAPPING_STOP_WINDOW_NS.
#  map_reset     {} (2026-10-26「重置地图」, host/map_reset.py) -> /fpga/map_reset (String JSON {"id": cmd_id}); the
#                runner stops the car, clears the memory (new scope) and the map caches, mapping_ctl restarts the map;
#                its /fpga/map_reset/status (latched) with our id -> RESULT "done", or "rejected" reason
#                map_reset_failed + "detail" (the runner's Chinese words: which step, why); "unknown" after
#                MAP_RESET_WINDOW_S. Refused here: no mapping status (localization mode / no in-stack mapping ->
#                map_reset_no_mapping), saving / loading the export (mapping_saving), resetting or one in flight
#                (map_reset_busy). Allowed while the e-stop is latched (nothing moves).
TELEOP_V_CAP, TELEOP_W_CAP = 0.30, 1.0
MAPPING_STOP_WINDOW_S = 180
MAP_RESET_WINDOW_S = 33
# goto (10-17): one goal relative to the car through the runner (host/ros_runtime.py goto_request has the
# authoritative bounds; the hub refuses what the runner would refuse anyway, so a bad command never leaves)
GOTO_MAX_FORWARD_M = 2.0
ENCODER_COUNTERS = ("good", "bad_checksum", "seq_jump", "dropped", "duplicate", "half_frame_irq",
                    "frame_error", "sync_dropped_bytes", "link_timeout", "start_glitch", "count_jump")
RESUME_REQUIRES = ("pl: next accepted geometry completion edge (PU 3.1)",
                   "mcu: heartbeat continuous >= 100 ms and a new velocity command (PU 3.2)",
                   "task: not resumed automatically; send find again")
NS = 1000000000
# first-person preview limits (10-21; tools/car_run/preview_publisher.py and the bridge check the same)
PREVIEW_MAX_W, PREVIEW_MAX_H = 640, 480
PREVIEW_MAX_BYTES = 160 * 1024
PREVIEW_MIN_INTERVAL_NS = NS // 20          # <= 20 fps in; the publisher sends 10
# Every nano_ns handed to this module is host.fpga_clock.now_ns() (CLOCK_MONOTONIC_RAW, the
# same axis as nano_replay capture stamps). ROS header stamps are a different axis.
NANO_CLOCK_ID = "nano_monotonic_raw_ns"
# kv.link_ok goes false after 0.8 s without a TELEM reply (or 10 consecutive misses), so that
# with a 100 ms TELEM period and 100 ms STATE period the PC sees it within 1 s (E.2 smoke).
LINK_LOST_NS = 800 * 1000000
MS = 1000000


def _ms(ns):
    return ns / 1e6


# ---------------------------------------------------------------- nav.path (spec C.4, C.7)
# Budget: STATE is 3-5 KB at 10 Hz (C.7) and the stream only carries what a PC view needs;
# a route drawn on an 80x80 / 5 cm map needs a few dozen vertices. So: <= 64 points AND the
# points array <= 1024 B as compact JSON (fpgac.dumps separators), i.e. <= ~1 KB extra per STATE
# (<= 10 KB/s at 10 Hz). Start tolerance = half a 5 cm cell; doubled until both caps hold.
PATH_MAX_POINTS = 64
PATH_MAX_BYTES = 1024
PATH_EPS0_M = 0.025
PATH_REASONS = ("no_plan_received", "empty_plan", "bad_plan")
# task.runtime_info cadence (v5, controller 2026-10-12): in STATE when it changed, then every 5 s;
# omitted from the other ticks (the PC keeps the last value, pc/fpgac.py)
RUNTIME_INFO_RESEND_NS = 5 * NS
RUNNER_PRODUCER_MISMATCH = "runner_producer_mismatch"


BELIEF_PHASES = ("preview", "search", "done", "none")
BELIEF_MAX_CELLS = 200000
BELIEF_KEYS = ("phase", "task", "seq", "mass_left", "next", "seen", "result", "reason", "views", "build_ms")


def _pose3(v):
    if (isinstance(v, (list, tuple)) and len(v) == 3
            and all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v)):
        return [float(x) for x in v]
    return None


def parse_belief(text):
    """10-26 /fpga/search/belief (host/search_belief_feed.py) -> (MAP meta, int8 cell bytes) for feed_map("belief").
    The display fields ride in meta["belief"]; ValueError on anything malformed (counted by the caller)."""
    doc = json.loads(text)
    if not isinstance(doc, dict) or doc.get("v") != 1 or doc.get("phase") not in BELIEF_PHASES:
        raise ValueError("belief: version/phase")
    w, h = doc.get("width"), doc.get("height")
    if not all(isinstance(n, int) and not isinstance(n, bool) and n >= 0 for n in (w, h)) or w * h > BELIEF_MAX_CELLS:
        raise ValueError("belief: size")
    res = doc.get("resolution")
    origin = _pose3(doc.get("origin"))
    if not isinstance(res, (int, float)) or isinstance(res, bool) or not res > 0 or origin is None:
        raise ValueError("belief: resolution/origin")
    if doc.get("encoding") != "int8_b64" or not isinstance(doc.get("cells"), str):
        raise ValueError("belief: encoding")
    try:
        cells = base64.b64decode(doc["cells"].encode("ascii"), validate=True)
    except (binascii.Error, UnicodeEncodeError, ValueError):
        raise ValueError("belief: cells not base64")
    if len(cells) != w * h:
        raise ValueError("belief: %d cells != %d x %d" % (len(cells), w, h))
    extra = {k: doc.get(k) for k in BELIEF_KEYS}
    extra["next"] = _pose3(extra["next"])
    seen = doc.get("seen") if isinstance(doc.get("seen"), list) else []
    extra["seen"] = [p for p in (_pose3(x) for x in seen[:200]) if p is not None]
    seq = doc.get("seq") if isinstance(doc.get("seq"), int) else None
    meta = {"rev": seq, "width": w, "height": h, "resolution_m": float(res), "origin": origin,
            "frame_id": doc.get("frame") if isinstance(doc.get("frame"), str) else "map", "belief": extra}
    return meta, cells


def _seg_dist(p, a, b):
    """Distance from p to segment a-b (a point if a == b)."""
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    px, py = p[0] - ax, p[1] - ay
    den = dx * dx + dy * dy
    if den > 0.0:
        u = (px * dx + py * dy) / den
        if u > 1.0:
            px, py = p[0] - b[0], p[1] - b[1]
        elif u > 0.0:
            px, py = px - u * dx, py - u * dy
    return math.hypot(px, py)


def rdp(points, eps):
    """Ramer-Douglas-Peucker on [(x, y)], iterative (NavFn paths are 1000s of poses: no
    recursion depth); first and last point always kept. Distance to the chord segment."""
    n = len(points)
    if n <= 2:
        return list(points)
    keep = [False] * n
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = points[i], points[j]
        best, idx = -1.0, i
        for k in range(i + 1, j):
            d = _seg_dist(points[k], a, b)
            if d > best:
                best, idx = d, k
        if best > eps:
            keep[idx] = True
            stack.append((i, idx))
            stack.append((idx, j))
    return [p for p, k in zip(points, keep) if k]


def _points_bytes(pts):
    return len(json.dumps(pts, separators=(",", ":")))


def _farthest(xs, ys, i, j):
    """(distance, index) of the interior point of i..j farthest from segment i-j; (-1, i) if none."""
    ax, ay = xs[i], ys[i]
    dx, dy = xs[j] - ax, ys[j] - ay
    den = dx * dx + dy * dy
    best, idx = -1.0, i
    for k in range(i + 1, j):
        px, py = xs[k] - ax, ys[k] - ay
        if den > 0.0:
            u = (px * dx + py * dy) / den
            if u > 1.0:
                px, py = xs[k] - xs[j], ys[k] - ys[j]
            elif u > 0.0:
                px, py = px - u * dx, py - u * dy
        d = px * px + py * py
        if d > best:
            best, idx = d, k
    return (math.sqrt(best) if best >= 0.0 else -1.0), idx


def simplify_path(points, eps0=PATH_EPS0_M, max_points=PATH_MAX_POINTS, max_bytes=PATH_MAX_BYTES):
    """(polyline [[x, y]] rounded to 1 cm, epsilon used) == [rdp(points, eps) rounded] for the
    smallest eps = eps0 * 2**k whose polyline has <= max_points points and JSON <= max_bytes.

    Computed in one pass instead of one RDP per doubling: RDP's split point of a segment does not
    depend on eps, only whether it splits does, so a point is kept for eps iff the smallest split
    distance on its chain of splits (its "effective" distance) is > eps. Splits are expanded
    largest-effective-first and stop once nothing is above eps0 or max_points - 1 interior points
    are open (any eps below that one would keep too many), so a noisy NavFn path costs a few
    passes over its poses, not a full-depth RDP per doubling (host: 14 ms -> ~2 ms, 1000 poses)."""
    import heapq
    n = len(points)
    if n <= 2:
        return [[round(x, 2), round(y, 2)] for x, y in points], eps0
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    eff = []                                   # (effective distance, index), non-increasing
    d, k = _farthest(xs, ys, 0, n - 1)
    heap = [(-d, 0, n - 1, k)]
    while heap and len(eff) < max_points - 1:
        neg, i, j, k = heapq.heappop(heap)
        e = -neg
        if e <= eps0:
            break
        eff.append((e, k))
        for a, b in ((i, k), (k, j)):
            if b > a + 1:
                d, kk = _farthest(xs, ys, a, b)
                heapq.heappush(heap, (-min(e, d), a, b, kk))
    floor = eff[-1][0] if len(eff) >= max_points - 1 else 0.0
    eps = eps0
    for _ in range(64):
        if eps >= floor:
            keep = sorted([0, n - 1] + [k for e, k in eff if e > eps])
            pts = [[round(xs[k], 2), round(ys[k], 2)] for k in keep]
            if len(pts) <= max_points and _points_bytes(pts) <= max_bytes:
                return pts, eps
        eps *= 2.0
    return [[round(xs[0], 2), round(ys[0], 2)], [round(xs[-1], 2), round(ys[-1], 2)]], eps


def simplify_path_reference(points, eps0=PATH_EPS0_M, max_points=PATH_MAX_POINTS, max_bytes=PATH_MAX_BYTES):
    """The definition simplify_path must equal: one full RDP per doubling (tests only)."""
    eps = eps0
    for _ in range(64):
        pts = [[round(x, 2), round(y, 2)] for x, y in rdp(points, eps)]
        if len(pts) <= max_points and _points_bytes(pts) <= max_bytes:
            return pts, eps
        eps *= 2.0
    return [[round(x, 2), round(y, 2)] for x, y in (points[0], points[-1])], eps


def summarize_plan(xy, frame_id, stamp_ros_s):
    """One nav_msgs/Path (as [(x, y)] in its own frame, no transform) -> (path_len_m, path, reason).

    path_len_m is the raw length (unchanged meaning of nav.path_len_m; None when a coordinate is
    not finite -- it used to be NaN, which allow_nan=False refuses and the STATE stopped)."""
    xy = [(float(x), float(y)) for x, y in xy]
    if not xy:
        return 0.0, None, "empty_plan"
    if not all(math.isfinite(x) and math.isfinite(y) for x, y in xy):
        return None, None, "bad_plan"
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(xy, xy[1:]))
    pts, eps = simplify_path(xy)
    stamp = stamp_ros_s if stamp_ros_s is not None and math.isfinite(stamp_ros_s) else None
    return length, {"frame_id": str(frame_id), "stamp_ros_s": stamp, "points": pts,
                    "n_source_poses": len(xy), "length_m": length, "epsilon_m": eps}, None


def scrub_nonfinite(obj, path="", out=None):
    """(copy of ``obj`` with every non-finite float -> None, [paths]) -- STATE/EVENT bodies go out
    with allow_nan=False, so one NaN/inf anywhere (a NaN cmd_vel, odom twist, pose, a NaN in a JSON
    input -- json.loads accepts the NaN literal) used to make the whole frame unsendable, every tick
    (2026-10-12 v5). Paths: dotted keys, list items as [i], e.g. ``nav.cmd_v``,
    ``nav.path.points[3][0]``. Dicts/lists/tuples are rebuilt (tuples become lists, as JSON does)."""
    if out is None:
        out = []
    if isinstance(obj, float):
        if math.isfinite(obj):
            return obj, out
        out.append(path or "<root>")
        return None, out
    if isinstance(obj, dict):
        return dict((k, scrub_nonfinite(v, "%s.%s" % (path, k) if path else str(k), out)[0])
                    for k, v in obj.items()), out
    if isinstance(obj, (list, tuple)):
        return [scrub_nonfinite(v, "%s[%d]" % (path, i), out)[0] for i, v in enumerate(obj)], out
    return obj, out


def _get(d, *path):
    for p in path:
        if not isinstance(d, dict) or p not in d:
            return None
        d = d[p]
    return d


def _median(values):
    if not values:
        return None
    s = sorted(values)
    n = len(s)
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


def parse_tegrastats(line):
    """One tegrastats line -> nano group fields (missing items stay None)."""
    out = {"vdd_in_mw": None, "cpu_pct": None, "gpu_pct": None, "temp_c_max": None}
    m = re.search(r"VDD_IN (\d+)mW", line)
    if m:
        out["vdd_in_mw"] = int(m.group(1))
    m = re.search(r"CPU \[([^\]]*)\]", line)
    if m:
        cpus = []
        for item in m.group(1).split(","):
            p = re.match(r"\s*(\d+)%", item)
            cpus.append(int(p.group(1)) if p else None)
        out["cpu_pct"] = cpus
    m = re.search(r"GR3D_FREQ (\d+)%", line)
    if m:
        out["gpu_pct"] = int(m.group(1))
    temps = [float(t) for t in re.findall(r"\b[A-Za-z0-9_]+@(-?\d+(?:\.\d+)?)C\b", line)]
    temps = [t for t in temps if -40.0 < t < 150.0]
    if temps:
        out["temp_c_max"] = max(temps)
    return out


def parse_nvpmodel(text):
    m = re.search(r"NV Power Mode:\s*(\S+)", text)
    return m.group(1) if m else None


class EstopManager(object):
    """Nano side of the PC e-stop latch (A.4, C.5). Talks to KV via a KvEstopClient-like object."""

    RESEND_NS = 200 * MS
    UNCONFIRMED_NS = 1 * NS

    def __init__(self, counts):
        self.counts = counts
        self.latched = False
        self.confirmed = False
        self.epoch = None
        self.kv_result = None
        self.pl_applied = None
        self.since_ns = None
        self.kv_boot_id = None
        self.pending = None
        self.waiters = []
        self.results = collections.deque()
        self._id_base = int.from_bytes(os.urandom(4), "little") << 32
        self._id_n = 0

    def _new_kv_id(self):
        self._id_n += 1
        return self._id_base | self._id_n

    def _result(self, waiter, body):
        client_id, cmd_id, op = waiter
        out = {"v": 1, "cmd_id": cmd_id, "op": op}
        out.update(body)
        self.results.append((client_id, out))

    def snapshot(self, now_ns):
        return {"latched": self.latched, "epoch": self.epoch, "source": "pc" if self.latched else None,
                "confirmed": self.confirmed, "pl_applied": self.pl_applied, "kv_result": self.kv_result,
                "since_ms": None if self.since_ns is None or not self.latched else _ms(now_ns - self.since_ns),
                "pending": None if self.pending is None else self.pending["op"]}

    def assert_request(self, waiter, now_ns):
        if not self.latched:
            self.latched, self.since_ns = True, now_ns
            self.confirmed = False
        if self.pending is not None and self.pending["op"] == "release":
            for w in self.pending["waiters"]:
                self._result(w, {"status": "rejected", "reason": "superseded_by_estop", "final": True})
            self.pending = None
        if self.pending is None:
            self.pending = {"op": "assert", "kv_cmd_id": self._new_kv_id(), "epoch": 0,
                            "started_ns": now_ns, "last_send_ns": None, "waiters": [],
                            "unconfirmed_reported": False}
        self.pending["waiters"].append(waiter)
        return {"status": "latched_on_nano", "epoch": self.epoch}

    def release_request(self, waiter, epoch, now_ns):
        if not self.latched:
            return {"status": "rejected", "reason": "not_latched"}
        if not self.confirmed or self.epoch is None or epoch != self.epoch:
            return {"status": "rejected", "reason": "epoch_mismatch", "epoch": self.epoch}
        if self.pending is not None and self.pending["op"] == "assert":
            return {"status": "rejected", "reason": "estop_in_flight"}
        if self.pending is None:
            self.pending = {"op": "release", "kv_cmd_id": self._new_kv_id(), "epoch": epoch,
                            "started_ns": now_ns, "last_send_ns": None, "waiters": [],
                            "unconfirmed_reported": False}
        self.pending["waiters"].append(waiter)
        return {"status": "forwarded", "epoch": epoch}

    def due(self, now_ns):
        p = self.pending
        return p is not None and (p["last_send_ns"] is None or now_ns - p["last_send_ns"] >= self.RESEND_NS)

    def take_send(self, now_ns):
        """Mark the pending op as sent now and return (op, kv_cmd_id, epoch), or None if not due.

        The caller performs the blocking KV call outside any lock and then hands the ack
        to on_ack with the same kv_cmd_id.
        """
        if not self.due(now_ns):
            return None
        p = self.pending
        p["last_send_ns"] = now_ns
        self.counts["estop_sent:" + p["op"]] += 1
        return p["op"], p["kv_cmd_id"], p["epoch"]

    def send_failed(self, exc):
        self.counts["estop_no_ack:" + type(exc).__name__] += 1

    def kv_step(self, client, clock):
        """Single-threaded helper: send/resend once if due (one blocking call <= its timeout)."""
        job = self.take_send(clock())
        if job is None:
            return False
        try:
            ack = client.send(job[0], job[1], job[2], timeout_s=0.2)
        except Exception as exc:                       # LinkTimeout or a socket error: just retry
            self.send_failed(exc)
            return False
        self.on_ack(ack, clock(), job[1])
        return True

    def on_ack(self, ack, now_ns, kv_cmd_id=None):
        p = self.pending
        self.kv_boot_id = ack.kv_boot_id
        if p is None:
            return
        if kv_cmd_id is not None and p["kv_cmd_id"] != kv_cmd_id:
            self.counts["estop_stale_ack"] += 1
            return
        if p["op"] == "assert":
            if ack.result not in (0, 1, 5, 6):
                self.counts["estop_bad_result:%d" % ack.result] += 1
                return
            self.confirmed, self.epoch, self.kv_result = True, ack.epoch, ack.result
            self.pl_applied = bool(ack.permit_avail and (ack.permit_status >> 3) & 1)
            body = {"status": "latched", "kv_result": ack.result, "epoch": ack.epoch,
                    "pl_applied": self.pl_applied, "force_stop_written": bool(ack.force_stop_written),
                    "permit_avail": bool(ack.permit_avail), "kv_boot_id": ack.kv_boot_id,
                    "latency_ms": _ms(now_ns - p["started_ns"]), "final": True}
        else:
            if ack.result in (0, 3):
                self.latched, self.since_ns, self.kv_result = False, None, ack.result
                self.pl_applied = None
                body = {"status": "released", "kv_result": ack.result, "epoch": ack.epoch,
                        "resume_requires": list(RESUME_REQUIRES), "final": True}
            elif ack.result == 2:
                body = {"status": "rejected", "reason": "epoch_mismatch", "kv_result": 2,
                        "epoch": ack.epoch, "final": True}
            else:
                self.counts["estop_bad_result:%d" % ack.result] += 1
                return
        for w in p["waiters"]:
            self._result(w, body)
        self.pending = None

    def tick(self, now_ns):
        p = self.pending
        if (p is not None and p["op"] == "assert" and not p["unconfirmed_reported"]
                and now_ns - p["started_ns"] >= self.UNCONFIRMED_NS):
            p["unconfirmed_reported"] = True
            self.counts["estop_kv_unconfirmed"] += 1
            for w in p["waiters"]:
                self._result(w, {"status": "kv_unconfirmed", "final": False,
                                 "note": "latched on Nano, KV ACK not yet received; still resending"})

    def reconcile(self, block, kv_boot_id, now_ns):
        """TELEM estop block vs our latch (A.4 'two-end restart')."""
        if not isinstance(block, dict) or "latched" not in block:
            return
        kv_latched = bool(block.get("latched"))
        restarted = self.kv_boot_id is not None and kv_boot_id != self.kv_boot_id
        if self.latched and self.confirmed and not kv_latched and restarted and self.pending is None:
            self.counts["estop_reassert"] += 1
            self.confirmed = False
            self.pending = {"op": "assert", "kv_cmd_id": self._new_kv_id(), "epoch": 0,
                            "started_ns": now_ns, "last_send_ns": None, "waiters": [],
                            "unconfirmed_reported": True}
        elif kv_latched and not self.latched:
            self.counts["estop_adopted"] += 1
            self.latched, self.confirmed, self.since_ns = True, True, now_ns
            self.epoch = block.get("epoch")
            self.kv_result = block.get("last_result")
            self.pl_applied = block.get("pl_applied")
        self.kv_boot_id = kv_boot_id


class HubCore(object):
    def __init__(self, data_source, hub_boot_id=None, state_hz=10, actions=None,
                 event_rate=20, cmd_window_ns=2 * NS, dedup=256):
        if data_source not in DATA_SOURCES:
            raise ValueError("--data-source must be live, replay or sim (no default)")
        self.data_source = data_source
        self.hub_boot_id = hub_boot_id or os.urandom(8).hex()
        self.state_hz = state_hz
        self.actions = actions
        self.event_rate = event_rate
        self.cmd_window_ns = cmd_window_ns
        self.counts = collections.Counter()
        self.latest = {}                     # input name -> (value, nano_recv_ns)
        self.startup = None
        self.source_state = "unknown"        # unknown | consistent | inconsistent
        self.events = collections.deque()    # (recv_ns, body) pending EVENT frames
        self.event_times = collections.deque()
        self.frame_stats = collections.deque(maxlen=10)
        # 10-25: receive time of the first frame of the current unbroken run of producer frames dropped for
        # no_valid_depth (None when the newest frame had depth); geometry_nano.depth_invalid_s, the page's
        # "camera too close" hint (10-22 14:44: 850 s of such frames, the car stopped, nobody saw why)
        self.depth_invalid_since_ns = None
        self.task_frames = {"processed": None, "dropped": None, "last_frame_id": None,
                            "geometry_age_refused": None, "geometry_age_last_refused": None}
        # v7 (2026-10-12): record rows dropped by a process's async record writer (queue full):
        # newest record_drops "dropped" per process (runner -> task, producer -> geometry_nano);
        # None = that process never reported a drop
        self.record_drops = {"runner": None, "producer": None}
        self.query = {"text": None, "seq": 0}
        self.seq_state = 0
        self.memory_sent = (None, None)      # (revision, nano_ns) of the last MEMORY frame
        self.memory_shown = None             # ids not position_invalid in that frame (10-27 fast removal)
        self.maps = {}                       # which -> dict(meta, cells, recv, sent_ns, sent_key)
        self.preview = None
        self.preview_last_ns = None
        self.fpv = FpvBoxes()                # 10-27 console first-person boxes (host/hub/fpv_boxes.py)
        self.fpv.counts = self.counts        # fpv_* counters in the hub stats
        self.hub_stats = {"clients": 0, "superseded": {}, "cpu_pct": None}
        self.estop = EstopManager(self.counts)
        self.mapping = None            # newest /fpga/mapping/status row (10-18 mapping_stop)
        # 10-28 start-up notices for the console (STATE "notices", a list of {code, level, text}); set once by
        # hub_ros from the run dir (ground_calib.json "alert": the fit was refused, a last good / configured mount
        # is used). Absent from STATE while empty.
        self.notices = []
        # 10-29 health check (host/hub/health.py, warn only): the newest report (STATE "health", set by the health
        # thread of hub_ros), the last composed STATE it reads, and the chassis driver / wheel odometry statuses
        self.health = None
        self.last_state = None
        self.map_reset = None          # newest /fpga/map_reset/status row (10-26 map_reset)
        self.book = collections.OrderedDict()
        self.dedup = dedup
        self.watchers = []
        self.results = self.estop.results
        self.telem = {"timeouts": 0, "consecutive": 0, "last_ok_ns": None}
        # kv group age around an e-stop (spec A.3): latched in the last composed STATE, the permit
        # block's FRAMES_ACCEPTED last seen while latched and that sample's KV boot, and whether a
        # release is still waiting for the next accepted frame (estop_release_pending)
        self.kv_permit_latch = {"latched": False, "fa": None, "boot": None, "pending": False}
        # producer frame_stats.source vs data_source (B.3): unknown | consistent | inconsistent
        self.producer_source_state = "unknown"
        # runner runtime_info.sensor_source vs producer frame_stats.source (B.3, v5 controller):
        # warn only -- never touches frame_source()/_ros_src(); unknown | consistent | inconsistent
        self.runner_producer_state = "unknown"
        self.runtime_info_sent = None        # (body key, nano_ns) of the last STATE carrying it
        self.executor_info = None            # hub_ros: {"requested", "used", "fallback_reason"}

    # ------------------------------------------------------------------ inputs
    def _put(self, name, value, nano_ns):
        self.latest[name] = (value, nano_ns)

    def _event(self, body, nano_ns, src=None):
        while self.event_times and nano_ns - self.event_times[0] >= NS:
            self.event_times.popleft()
        if len(self.event_times) >= self.event_rate:
            self.counts["events_dropped"] += 1
            return
        self.event_times.append(nano_ns)
        self.events.append((nano_ns, src, body))

    def feed_status(self, text, nano_ns):
        try:
            row = json.loads(text)
        except ValueError:
            self.counts["bad_input:status"] += 1
            return
        if not isinstance(row, dict):
            self.counts["bad_input:status"] += 1
            return
        kind = row.get("event")
        if kind == "startup":
            self.startup = row
            sensor = row.get("sensor_source")
            if sensor_agrees(sensor, self.data_source):
                self.source_state = "consistent"
            else:
                self.source_state = "inconsistent"
                self._event({"event": "source_mismatch", "data_source": self.data_source,
                             "runtime_sensor_source": sensor}, nano_ns, src="unknown")
        if kind == "task":
            self._put("status_task", row, nano_ns)
            self._observe_task(row, nano_ns)
            for key, name in (("processed_frames", "processed"), ("dropped_frames", "dropped")):
                if key in row:
                    self.task_frames[name] = row[key]
            return
        if kind == "frame_processed":
            self._put("status_frame", row, nano_ns)
            self.task_frames["last_frame_id"] = row.get("frame_id")
            if "dropped_frames" in row:
                self.task_frames["dropped"] = row["dropped_frames"]
            self.counts["frames_processed_seen"] += 1
            return
        if kind in GEOMETRY_AGE_EVENTS:     # runner's 200 ms gate; still forwarded as EVENT below
            self.task_frames["geometry_age_refused"] = row.get("refused_total")
            self.task_frames["geometry_age_last_refused"] = {k: row.get(k) for k in ("frame_id", "reason", "age_ms")}
        if kind == "record_drops" and row.get("process") in self.record_drops:
            self.record_drops[row["process"]] = row.get("dropped")     # still forwarded as EVENT below
        if kind in ("query_rejected", "selection_rejected", "goto_rejected"):
            self._observe_rejection(kind, row, nano_ns)
        if kind == "goto_accepted":
            for w in list(self.watchers):
                if w["op"] == "goto":
                    self._resolve(w, {"status": "accepted", "command_id": row.get("command_id"), "pose": row.get("pose")})
                    break
        self._event(row, nano_ns)

    def feed_runtime_info(self, text, nano_ns):
        """/fpga/runtime_info (runner, latched; v5): start-up facts, kept as task.runtime_info. Its
        sensor_source is checked against the producer's frame_stats.source (B.3, controller v5):
        a disagreement is a warning only (geometry_nano.runner_source_warning + one EVENT per
        change); the frame-level source stays driven by the startup event. Since v6 the group src
        falls back to this sensor_source when the startup event never reached the hub (_ros_src_basis)."""
        try:
            row = json.loads(text)
        except ValueError:
            self.counts["bad_input:runtime_info"] += 1
            return
        if not isinstance(row, dict):
            self.counts["bad_input:runtime_info"] += 1
            return
        self._put("runtime_info", row, nano_ns)
        self._observe_runner_producer(nano_ns)

    def feed_memory(self, text, nano_ns):
        try:
            row = json.loads(text)
        except ValueError:
            self.counts["bad_input:memory"] += 1
            return
        if not isinstance(row, dict) or not isinstance(row.get("tracks"), list):
            self.counts["bad_input:memory"] += 1
            return
        # MEMORY goes out with allow_nan=False: one NaN/inf (json.loads takes the NaN/Infinity literals,
        # 1e400 -> inf) used to make the frame unsendable and, in tick(), cost that tick's STATE too
        # (2026-10-15 GeoMem integration). A track with any non-finite number is dropped whole (its
        # position cannot be trusted); other non-finite fields are nulled. Both counted.
        tracks = []
        for t in row["tracks"]:
            if scrub_nonfinite(t)[1]:
                self.counts["memory_nonfinite_tracks"] += 1
            else:
                tracks.append(t)
        if len(tracks) != len(row["tracks"]):
            row = dict(row)
            row["tracks"] = tracks
        row, bad = scrub_nonfinite(row)
        if bad:
            self.counts["memory_nonfinite_fields"] += len(bad)
        self._put("memory", row, nano_ns)

    def feed_region(self, text, nano_ns):
        try:
            self._put("region", json.loads(text), nano_ns)
        except ValueError:
            self.counts["bad_input:region"] += 1

    def feed_frame_stats(self, text, nano_ns):
        try:
            row = json.loads(text)
        except ValueError:
            self.counts["bad_input:frame_stats"] += 1
            return
        if not isinstance(row, dict):
            self.counts["bad_input:frame_stats"] += 1
            return
        self._put("frame_stats", row, nano_ns)
        self.frame_stats.append((nano_ns, row))
        if row.get("result") == "dropped:no_valid_depth":
            if self.depth_invalid_since_ns is None:
                self.depth_invalid_since_ns = nano_ns
        else:
            self.depth_invalid_since_ns = None
        self._observe_producer_source(row.get("source"), nano_ns)
        self._observe_runner_producer(nano_ns)

    def _observe_producer_source(self, producer, nano_ns):
        """Producer's per-frame live/replay mark vs --data-source (B.3, second input). Missing
        mark = no information (unknown, not a mismatch). The EVENT goes out on the edge into
        inconsistent only, not per frame."""
        if producer is None:
            state = "unknown"
        else:
            state = "consistent" if producer == self.data_source else "inconsistent"
        if state == "inconsistent" and self.producer_source_state != "inconsistent":
            self._event({"event": "source_mismatch", "origin": "frame_stats", "data_source": self.data_source,
                         "producer_source": producer}, nano_ns, src="unknown")
        self.producer_source_state = state

    def _observe_runner_producer(self, nano_ns):
        """Runner's declared source (runtime_info.sensor_source, mapped hardware->live,
        replay/simulation->replay) vs the newest producer frame_stats.source. Either side missing =
        no information (unknown, no warning); an unmapped runner value counts as a disagreement.
        Warn only: one EVENT source_warning on the edge into inconsistent, source_warning_cleared
        on the edge out (origin "runtime_info"); frame_source() is not touched (group src: see
        _ros_src_basis -- startup event first, runtime_info only without it)."""
        info = self._val("runtime_info")
        fs = self._val("frame_stats")
        runner = info.get("sensor_source") if isinstance(info, dict) else None
        producer = fs.get("source") if isinstance(fs, dict) else None
        if runner is None or producer is None:
            state = "unknown"
        else:
            state = "consistent" if sensor_agrees(runner, producer) else "inconsistent"
        body = {"origin": "runtime_info", "source": "hub", "reason": RUNNER_PRODUCER_MISMATCH,
                "runner_sensor_source": runner, "producer_source": producer, "data_source": self.data_source}
        if state == "inconsistent" and self.runner_producer_state != "inconsistent":
            self.counts["runner_producer_source_warning"] += 1
            self._event(dict(body, event="source_warning"), nano_ns, src="unknown")
        elif state != "inconsistent" and self.runner_producer_state == "inconsistent":
            self._event(dict(body, event="source_warning_cleared"), nano_ns, src="unknown")
        self.runner_producer_state = state

    def feed_motion_permit(self, allowed, nano_ns):
        self._put("motion_permit", bool(allowed), nano_ns)
        if not allowed:
            self._observe_permit_false(nano_ns)

    def feed_cmd_vel(self, v, w, nano_ns):
        self._put("cmd_vel", (float(v), float(w)), nano_ns)

    def feed_plan(self, path_len_m, nano_ns, path=None, path_reason=None):
        """``path``/``path_reason`` from summarize_plan (computed outside the lock). A feeder that
        gives only a length (no-ROS bench feeders, old callers) shows path_reason length_only."""
        if path is None and path_reason is None:
            path_reason = "length_only"
        self._put("plan", {"len": None if path_len_m is None else float(path_len_m),
                           "path": path, "reason": path_reason}, nano_ns)

    def feed_loc_status(self, text, nano_ns):
        try:
            self._put("loc_status", json.loads(text), nano_ns)
        except ValueError:
            self.counts["bad_input:loc_status"] += 1

    def feed_loc_pose(self, pose, nano_ns):
        """pose: dict x, y, yaw, cov_xy_trace, stamp_ros_s."""
        self._put("loc_pose", dict(pose), nano_ns)

    def feed_rolling_status(self, text, nano_ns):
        try:
            self._put("rolling_status", json.loads(text), nano_ns)
        except ValueError:
            self.counts["bad_input:rolling_status"] += 1

    def feed_map(self, which, meta, cells, nano_ns):
        """which: rolling|global. meta: rev,width,height,resolution_m,origin,frame_id. cells: int8 bytes."""
        if which not in ("rolling", "global", "belief"):
            raise ValueError(which)
        entry = self.maps.get(which) or {"sent_ns": None, "sent_key": None}
        entry.update({"meta": dict(meta), "cells": bytes(cells), "recv": nano_ns})
        self.maps[which] = entry
        if which == "global":
            self._put("map_global", dict(meta), nano_ns)

    def feed_belief(self, text, nano_ns):
        """10-26 /fpga/search/belief (the runner's search heat map) -> MAP which=belief (<= 4 Hz, newest only)."""
        try:
            meta, cells = parse_belief(text)
        except ValueError:
            self.counts["bad_input:belief"] += 1
            return
        self.feed_map("belief", meta, cells, nano_ns)

    def feed_odom(self, v, w, stamp_ros_s, nano_ns, x=None, y=None, yaw=None, frame_id=None):
        """/odom. Since 10-15 (GeoMem 联调第 3 棒, design H2) the pose rides along: STATE odom.x/y/yaw/frame_id
        (None when the caller gives none). Odometry only -- never a map pose; the bridge labels it so."""
        self._put("odom", {"v": float(v), "w": float(w), "stamp_ros_s": stamp_ros_s,
                           "x": None if x is None else float(x), "y": None if y is None else float(y),
                           "yaw": None if yaw is None else float(yaw), "frame_id": frame_id}, nano_ns)

    def feed_odom_adapter(self, text, nano_ns):
        try:
            value = json.loads(text)
        except ValueError:
            value = {"state": text}
        self._put("odom_adapter", value, nano_ns)

    def feed_preview(self, meta, jpeg, nano_ns):
        """10-21: limits raised from 320x240 / 30 KiB / 2 fps to PREVIEW_MAX_* (operator console drives by it)."""
        w, h = meta.get("w", 0), meta.get("h", 0)
        too_fast = self.preview_last_ns is not None and nano_ns - self.preview_last_ns < PREVIEW_MIN_INTERVAL_NS
        if w > PREVIEW_MAX_W or h > PREVIEW_MAX_H or len(jpeg) > PREVIEW_MAX_BYTES or too_fast:
            self.counts["preview_dropped"] += 1
            return
        self.preview_last_ns = nano_ns
        self.preview = {"meta": dict(meta), "jpeg": bytes(jpeg), "recv": nano_ns, "sent": False}

    def feed_geometry_frame(self, text, nano_ns):
        """10-27: /fpga/geometry_frame (the producer's per-frame ROI rows, the car screen AR's boxes) -> the
        console's first-person boxes; labels from the newest /fpga/memory."""
        mem = self.latest.get("memory")
        self.fpv.feed(text, mem[0] if mem else None)

    def feed_chassis_status(self, text, nano_ns):
        """10-29: /fpga/chassis/status (host/ros_chassis_driver.py, 2 Hz JSON) -> the health check only."""
        try:
            row = json.loads(text)
        except ValueError:
            self.counts["bad_input:chassis_status"] += 1
            return
        if isinstance(row, dict):
            self._put("chassis_status", row, nano_ns)

    def feed_wheel_odom_status(self, text, nano_ns):
        """10-29: /fpga/wheel_odom/status (host/ros_wheel_odom.py, 1 Hz JSON: heading source) -> the health check."""
        try:
            row = json.loads(text)
        except ValueError:
            self.counts["bad_input:wheel_odom_status"] += 1
            return
        if isinstance(row, dict):
            self._put("wheel_odom_status", row, nano_ns)

    def health_inputs(self, nano_ns):
        """10-29: read-only inputs of host/hub/health.py (called under the hub lock, cheap): the last composed
        STATE, the receive age of every input, the counters it turns into rates, the chassis / odometry statuses."""
        ch = self.latest.get("chassis_status")
        ages = dict((k, (nano_ns - v[1]) / 1e9) for k, v in self.latest.items())
        if self.preview_last_ns is not None:
            ages["preview"] = (nano_ns - self.preview_last_ns) / 1e9
        return {"state": self.last_state, "ages_s": ages,
                "counts": dict((k, self.counts.get(k, 0)) for k in HEALTH_COUNTERS),
                "chassis": None if ch is None else ch[0],
                "chassis_recv_ns": None if ch is None else ch[1],
                "chassis_age_s": None if ch is None else (nano_ns - ch[1]) / 1e9,
                "wheel_odom": self._val("wheel_odom_status"),
                "depth_invalid_s": (None if self.depth_invalid_since_ns is None
                                    else (nano_ns - self.depth_invalid_since_ns) / 1e9)}

    def feed_telemetry(self, tel, nano_ns):
        self._put("telemetry", tel, nano_ns)
        self.telem["consecutive"] = 0
        self.telem["last_ok_ns"] = nano_ns
        estop_block = tel.blocks.get("estop") if isinstance(tel.blocks, dict) else None
        was_latched = self.estop.latched
        self.estop.reconcile(estop_block, tel.kv_boot_id, nano_ns)
        if self.estop.latched and not was_latched:    # adopted a KV latch (10-17: cancel as a PC e-stop does)
            self._estop_cancel("estop_cancel:adopted")

    def feed_telem_timeout(self, nano_ns):
        self.telem["timeouts"] += 1
        self.telem["consecutive"] += 1
        self.counts["telem_timeout"] += 1

    def feed_clock(self, report, nano_ns):
        self._put("clock", report, nano_ns)

    def feed_tegrastats(self, line, nano_ns):
        self._put("tegrastats", parse_tegrastats(line), nano_ns)

    def feed_nvpmodel(self, text, nano_ns):
        self._put("nvpmodel", parse_nvpmodel(text), nano_ns)

    def feed_executor_info(self, info, nano_ns):
        """Which ROS executor the hub runs (hub_ros: lean, or the stock one after a failed probe of
        the rclpy internals the lean loop uses). A fallback is counted and sent as one EVENT."""
        self.executor_info = dict(info)
        if info.get("fallback_reason"):
            self.counts["executor_fallback"] += 1
            self._event({"event": "hub_executor_fallback", "requested": info.get("requested"),
                         "used": info.get("used"), "reason": info.get("fallback_reason")},
                        nano_ns, src=self.data_source)

    def feed_hub_stats(self, clients, superseded, cpu_pct, effective_sndbuf=None):
        self.hub_stats = {"clients": clients, "superseded": dict(superseded), "cpu_pct": cpu_pct,
                          "effective_sndbuf": effective_sndbuf}

    # ------------------------------------------------------------------ commands
    def command(self, client_id, is_controller, body, nano_ns):
        """Return the CMD_ACK body; side effects through ``actions`` (spec C.5)."""
        cmd_id = body.get("cmd_id") if isinstance(body, dict) else None
        op = body.get("op") if isinstance(body, dict) else None
        if not isinstance(cmd_id, str) or not cmd_id or len(cmd_id) > 128:
            self.counts["cmd_bad"] += 1
            return {"v": 1, "cmd_id": cmd_id, "op": op, "status": "rejected", "reason": "bad_args"}
        if op == "teleop":                      # high rate, no watcher, outside the dedup book (module note)
            ack = {"v": 1, "cmd_id": cmd_id, "op": op, "nano_mono_ns": nano_ns}
            ack.update(self._execute(client_id, is_controller, cmd_id, op, body.get("args") or {}, nano_ns))
            self.counts["cmd:%s:%s" % (op, ack["status"])] += 1
            return ack
        key = (client_id, cmd_id)
        if key in self.book:
            self.counts["cmd_duplicate"] += 1
            ack = dict(self.book[key])
            ack["duplicate"] = True
            return ack
        ack = {"v": 1, "cmd_id": cmd_id, "op": op, "nano_mono_ns": nano_ns}
        ack.update(self._execute(client_id, is_controller, cmd_id, op, body.get("args") or {}, nano_ns))
        self.counts["cmd:%s:%s" % (op, ack["status"])] += 1
        self.book[key] = ack
        while len(self.book) > self.dedup:
            self.book.popitem(last=False)
        return ack

    def _execute(self, client_id, is_controller, cmd_id, op, args, nano_ns):
        if op not in OPS or not isinstance(args, dict):
            return {"status": "rejected", "reason": "bad_op" if op not in OPS else "bad_args"}
        if op in ("patrol", "cold_start_confirm"):
            return {"status": "rejected", "reason": "not_enabled"}
        if op == "estop":                       # stopping is always allowed (controller 10-11)
            ack = self.estop.assert_request((client_id, cmd_id, op), nano_ns)
            self._estop_cancel("estop_cancel:pc")   # 10-17: O2 revoked (module note)
            return ack
        if not is_controller:
            return {"status": "rejected", "reason": "not_controller"}
        if op == "yield_control":
            return {"status": "yielded"}
        if op == "release":
            epoch = args.get("epoch")
            if type(epoch) is not int:
                return {"status": "rejected", "reason": "bad_args"}
            return self.estop.release_request((client_id, cmd_id, op), epoch, nano_ns)
        if op in ("find", "select", "goto") and self.estop.latched:
            return {"status": "rejected", "reason": "estop_latched"}
        if op == "teleop":
            return self._teleop(args, nano_ns)
        if op == "mapping_stop":
            if args:
                return {"status": "rejected", "reason": "bad_args"}
            state = (self.mapping or {}).get("state")
            if state is None:
                return {"status": "rejected", "reason": "no_mapping_status"}
            if state != "mapping":
                return {"status": "rejected", "reason": "mapping_" + str(state)}
            self._watch(client_id, cmd_id, op, nano_ns)
            self.watchers[-1]["deadline"] = nano_ns + MAPPING_STOP_WINDOW_S * NS
            self.actions.publish_mapping_stop()
            return {"status": "forwarded"}
        if op == "map_reset":
            if args:
                return {"status": "rejected", "reason": "bad_args"}
            row = self.mapping or {}
            state = row.get("state")
            if state is None:
                return {"status": "rejected", "reason": "map_reset_no_mapping"}
            if state == "saving" or (state == "exported" and row.get("map_loaded") is None):
                return {"status": "rejected", "reason": "mapping_saving"}
            if state == "resetting" or any(w["op"] == "map_reset" for w in self.watchers):
                return {"status": "rejected", "reason": "map_reset_busy"}
            self._watch(client_id, cmd_id, op, nano_ns)
            self.watchers[-1]["deadline"] = nano_ns + MAP_RESET_WINDOW_S * NS
            self.actions.publish_map_reset(json.dumps({"id": cmd_id, "sent_ns": int(nano_ns)}, sort_keys=True))
            return {"status": "forwarded"}
        if op == "find":
            text = args.get("text")
            if not isinstance(text, str) or not text.strip() or len(text) > 64:
                return {"status": "rejected", "reason": "bad_args"}
            self.query = {"text": text, "seq": self.query["seq"] + 1}
            self._watch(client_id, cmd_id, op, nano_ns)
            self.actions.publish_query(text)
            return {"status": "forwarded", "query_seq": self.query["seq"]}
        if op == "goto":
            body = {}
            for key in ("forward_m", "turn_rad"):
                v = args.get(key, 0.0)
                if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                    return {"status": "rejected", "reason": "bad_args"}
                body[key] = float(v)
            if (set(args) - {"forward_m", "turn_rad"} or not 0.0 <= body["forward_m"] <= GOTO_MAX_FORWARD_M
                    or abs(body["turn_rad"]) > math.pi or body == {"forward_m": 0.0, "turn_rad": 0.0}):
                return {"status": "rejected", "reason": "bad_args"}
            self._watch(client_id, cmd_id, op, nano_ns)
            self.actions.publish_goto(json.dumps(body, sort_keys=True))
            return {"status": "forwarded"}
        if op == "select":
            cid = args.get("candidate_id")
            if not isinstance(cid, str) or not cid:
                return {"status": "rejected", "reason": "bad_args"}
            self._watch(client_id, cmd_id, op, nano_ns)
            self.actions.publish_select(cid)
            return {"status": "forwarded"}
        # cancel
        self._watch(client_id, cmd_id, op, nano_ns)
        self.actions.publish_cancel()
        return {"status": "forwarded"}

    def _teleop(self, args, nano_ns):
        if args.get("stop") is True and set(args) == {"stop"}:
            v = w = 0.0
        else:
            if set(args) != {"v", "w"}:
                return {"status": "rejected", "reason": "bad_args"}
            v, w = args["v"], args["w"]
            for x in (v, w):
                if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
                    return {"status": "rejected", "reason": "bad_args"}
            if abs(v) > TELEOP_V_CAP or abs(w) > TELEOP_W_CAP:
                return {"status": "rejected", "reason": "bad_args"}
        if (v or w) and self.estop.latched:
            return {"status": "rejected", "reason": "estop_latched"}
        self.actions.publish_teleop(json.dumps({"v": float(v), "w": float(w), "sent_ns": int(nano_ns)},
                                               sort_keys=True))
        return {"status": "forwarded"}

    def feed_mapping_status(self, text, nano_ns):
        """/fpga/mapping/status (latched String JSON {"state": mapping|saving|exported|failed, ...})."""
        try:
            row = json.loads(text)
        except ValueError:
            self.counts["mapping_status_bad"] += 1
            return
        if not isinstance(row, dict) or not isinstance(row.get("state"), str):
            self.counts["mapping_status_bad"] += 1
            return
        self.mapping = row
        # final: failed, or exported once the /map_server/load_map answer is in (map_loaded true/false; the
        # sim run 10-18 arb1 resolved on the first "exported" row, before the load, and read map_loaded null)
        if row["state"] == "failed" or (row["state"] == "exported" and row.get("map_loaded") is not None):
            for w in list(self.watchers):
                if w["op"] == "mapping_stop":
                    self._resolve(w, {"status": "done", "mapping_state": row["state"],
                                      "mapping": {k: row.get(k) for k in ("yaml", "db_dir", "reason", "map_loaded",
                                                                          "load_result")}})

    def feed_map_reset_status(self, text, nano_ns):
        """/fpga/map_reset/status (runner, latched; host/map_reset.py MapResetJob.status / refused_status)."""
        try:
            row = json.loads(text)
        except ValueError:
            self.counts["map_reset_status_bad"] += 1
            return
        if not isinstance(row, dict) or not isinstance(row.get("id"), str):
            self.counts["map_reset_status_bad"] += 1
            return
        self.map_reset = row
        if row.get("state") not in ("done", "failed"):
            return
        for w in list(self.watchers):
            if w["op"] == "map_reset" and w["cmd_id"] == row["id"]:
                summary = {k: row.get(k) for k in ("state", "steps", "scope", "previous_scope", "mapping_session",
                                                   "elapsed_s", "refused")}
                if row["state"] == "done":
                    self._resolve(w, {"status": "done", "map_reset": summary})
                else:
                    self._resolve(w, {"status": "rejected", "reason": "map_reset_failed",
                                      "detail": row.get("reason_zh"), "map_reset": summary})
        if row.get("steps"):           # the reset ran (also when a step failed): drop what belonged to the old map
            self.latest.pop("plan", None)
            self.counts["map_reset_cache_cleared"] += 1

    def _estop_cancel(self, count):
        """10-17 (O2 revoked): an e-stop also cancels the runner's navigation and task (/fpga/cancel)."""
        self.counts[count] += 1
        if self.actions is not None:
            self.actions.publish_cancel()

    def _task_key(self):
        row = self.latest.get("status_task")
        if row is None:
            return None
        r = row[0]
        return (r.get("state"), r.get("reason"), r.get("target_id"))

    def _watch(self, client_id, cmd_id, op, nano_ns):
        self.watchers.append({"client_id": client_id, "cmd_id": cmd_id, "op": op,
                              "deadline": nano_ns + self.cmd_window_ns, "baseline": self._task_key(),
                              "start": nano_ns})

    def _resolve(self, w, body):
        out = {"v": 1, "cmd_id": w["cmd_id"], "op": w["op"], "final": True}
        out.update(body)
        self.results.append((w["client_id"], out))
        self.watchers.remove(w)

    def _observe_task(self, row, nano_ns):
        key = (row.get("state"), row.get("reason"), row.get("target_id"))
        # target: the runner's summary of where the remembered object is (only on needs_manual_approach,
        # SELECT_SEMANTICS_20261016.md); absent -> the RESULT keys are the same as before
        extra = {"target": row["target"]} if isinstance(row.get("target"), dict) else {}
        for w in list(self.watchers):
            # task_reason / target_id: the runner's own words for WHY (e.g. failed because
            # navigation_server_unavailable), so a terminal answer is readable without the JSONL.
            if w["op"] in ("find", "select") and key != w["baseline"]:
                self._resolve(w, {"status": "accepted", "task_state": row.get("state"),
                                  "task_reason": row.get("reason"), "target_id": row.get("target_id"), **extra})
            elif w["op"] == "cancel" and row.get("state") in TERMINAL_TASK:
                self._resolve(w, {"status": "done", "task_state": row.get("state"),
                                  "task_reason": row.get("reason"), "target_id": row.get("target_id"), **extra})

    def _observe_rejection(self, kind, row, nano_ns):
        op = {"query_rejected": "find", "selection_rejected": "select", "goto_rejected": "goto"}[kind]
        for w in list(self.watchers):
            if w["op"] == op:
                self._resolve(w, {"status": "rejected", "reason": row.get("reason")})
                break

    def _observe_permit_false(self, nano_ns):
        for w in list(self.watchers):
            if w["op"] == "cancel" and nano_ns >= w["start"]:
                self._resolve(w, {"status": "done", "motion_permit": False})

    def tick(self, nano_ns):
        for w in list(self.watchers):
            if nano_ns >= w["deadline"]:
                self._resolve(w, {"status": "unknown"})
        self.estop.tick(nano_ns)

    def pop_results(self):
        out = list(self.results)
        self.results.clear()
        return out

    def pop_events(self, nano_ns):
        out = []
        while self.events:
            recv, src, row = self.events.popleft()
            row, bad = scrub_nonfinite(row)            # a NaN event body would fail the whole tick
            ev = {"v": 1, "src": src or self._ros_src(), "age_ms": _ms(nano_ns - recv), "event": row}
            if bad:
                ev["nonfinite_fields"] = bad
                self.counts["event_nonfinite_fields"] += len(bad)
            out.append(ev)
        return out

    # ------------------------------------------------------------------ frames
    def _ros_src_basis(self):
        """(src, basis) of the ROS groups and their EVENT/MEMORY/MAP frames (B.3). The runner's
        /fpga/status ``startup`` event decides when the hub has it (basis "startup_event"); it is
        volatile, so a hub that joined after the runner never gets it -- then the latched
        /fpga/runtime_info ``sensor_source`` decides, mapped the same way (hardware->live,
        replay/simulation/sim->replay; basis "runtime_info"; v6 controller ruling). Disagreeing with
        --data-source, or a value outside the map, gives unknown; neither input = unknown, basis None.
        The startup event wins whenever present (also when runtime_info says otherwise)."""
        if self.startup is not None:
            return (self.data_source if self.source_state == "consistent" else "unknown"), "startup_event"
        info = self._val("runtime_info")
        sensor = info.get("sensor_source") if isinstance(info, dict) else None
        if sensor is None:
            return "unknown", None
        return (self.data_source if sensor_agrees(sensor, self.data_source) else "unknown"), "runtime_info"

    def _ros_src(self):
        return self._ros_src_basis()[0]

    def frame_source(self):
        """Frame-level source (B.3): inconsistent if EITHER the runner's startup sensor_source OR
        the newest producer frame_stats.source disagrees with --data-source; a missing mark is
        no information. Group ``src`` is _ros_src (startup event, else runtime_info since v6); this
        frame-level value is unchanged by runtime_info (its disagreement is the separate
        runner_source_warning)."""
        if self.source_state == "inconsistent" or self.producer_source_state == "inconsistent":
            return "inconsistent"
        return self.data_source

    def _group(self, name, nano_ns, extra_age_ms=0.0):
        inputs = GROUP_INPUTS[name]
        present = [self.latest[i][1] for i in inputs if i in self.latest]
        if inputs and not present:
            g = {"src": "unavailable", "age_ms": None, "ok": False, "reason": "no_input"}
            if name in ROS_GROUPS:
                g["src_basis"] = None
            return g
        age = None if not present else _ms(nano_ns - min(present)) + extra_age_ms
        if name not in ROS_GROUPS:
            return {"src": self.data_source, "age_ms": age, "ok": True, "reason": None}
        # ROS groups (v6): which input decided src -- "startup_event" | "runtime_info" | None
        src, basis = self._ros_src_basis()
        return {"src": src, "age_ms": age, "ok": True, "reason": None, "src_basis": basis}

    def _val(self, name):
        row = self.latest.get(name)
        return None if row is None else row[0]

    def compose_state(self, nano_ns, force_runtime_info=False):
        """One STATE body. ``force_runtime_info`` (the snapshot to a newly connected client) puts
        task.runtime_info in without moving the change/5 s schedule shared by the broadcast."""
        self.seq_state += 1
        s = {"v": 1, "seq_state": self.seq_state, "hub_boot_id": self.hub_boot_id,
             "nano_mono_ns": nano_ns, "nano_clock": NANO_CLOCK_ID, "source": self.frame_source(),
             "replay_id": None, "state_hz": self.state_hz}
        if self.data_source == "replay" and self.startup is not None:
            s["replay_id"] = self.startup.get("replay_id") or self.startup.get("dataset")
        s["task"] = self._task(nano_ns, force_runtime_info)
        s["memory"] = self._memory(nano_ns)
        s["region"] = self._region(nano_ns)
        s["pose"] = self._pose(nano_ns)
        s["map"] = self._map(nano_ns)
        s["nav"] = self._nav(nano_ns)
        s["odom"] = self._odom(nano_ns)
        s["kv"] = self._kv(nano_ns)
        s["estop"] = self._estop(nano_ns)
        s["clock"] = self._clock(nano_ns)
        s["geometry_nano"] = self._geometry_nano(nano_ns)
        s["perf"] = self._perf(nano_ns)
        s["nano"] = self._nano(nano_ns)
        s["hub"] = self._hub(nano_ns)
        if self.mapping is not None:   # 10-26: the console enables 结束建图 / 重置地图 on it (absent: no mapping)
            m, r = self.mapping, self.map_reset or {}
            s["mapping"] = {"state": m.get("state"), "session": m.get("session"),
                            "map_loaded": m.get("map_loaded"),
                            "reset": {"id": r.get("id"), "state": r.get("state"), "reason_zh": r.get("reason_zh")}
                            if r else None}
        if self.notices:
            s["notices"] = [dict(n) for n in self.notices]
        if self.health is not None:    # 10-29 health check report (host/hub/health.py); absent until the first one
            s["health"] = self.health
        # never let one non-finite field stop the STATE stream: null it and name it (v5)
        s, bad = scrub_nonfinite(s)
        s["nonfinite_fields"] = bad
        self.last_state = s
        if bad:
            self.counts["state_nonfinite_fields"] += len(bad)
        return s

    def _task(self, now, force_runtime_info=False):
        g = self._group("task", now)
        t = self._val("status_task") or {}
        g.update({"state": t.get("state"), "reason_task": t.get("reason"), "target_id": t.get("target_id"),
                  "candidates": [dict({k: c.get(k) for k in ("id", "label", "color", "position_m", "state")},
                                      **({"number": c["number"]} if c.get("number") is not None else {}))
                                 for c in (t.get("candidates") or [])],
                  "query_text": self.query["text"], "query_seq": self.query["seq"],
                  "processed_frames": self.task_frames["processed"],
                  "dropped_frames": self.task_frames["dropped"],
                  "last_frame_id": self.task_frames["last_frame_id"],
                  "geometry_age_refused": self.task_frames["geometry_age_refused"],
                  "geometry_age_last_refused": self.task_frames["geometry_age_last_refused"],
                  "record_drops": self.record_drops["runner"]})
        if isinstance(t.get("target"), dict):     # needs_manual_approach only; other STATEs keep their keys
            g["target"] = {k: t["target"].get(k) for k in ("id", "label", "color", "position_m", "state", "last_seen")}
            if t["target"].get("number") is not None:   # 10-27 display number (瓶子 2), only when the runner sent it
                g["target"]["number"] = t["target"]["number"]
        # 10-25 find-retry: the runner's navigation attempt summary (count, distinct goals, cap), only when the task
        # row carries it (the per-attempt items stay in the runner's JSONL); absent -> the task keys are as before
        if isinstance(t.get("attempts"), dict):
            g["attempts"] = {k: t["attempts"].get(k) for k in ("count", "positions", "max")}
        # 10-26 search MVP: why a remembered target went searching (only when the task row carries it)
        if isinstance(t.get("search_trigger"), str):
            g["search_trigger"] = t["search_trigger"]
        # 10-29: processed frames missing past the stream timeout (the car warns and waits); only while it lasts
        if isinstance(t.get("stream_stall"), dict):
            g["stream_stall"] = {k: t["stream_stall"].get(k) for k in ("gap_s", "n")}
        # task.runtime_info (v5): present when it changed since the last STATE that carried it (the
        # first STATE counts, null = never received) and every RUNTIME_INFO_RESEND_NS after that;
        # absent on the other ticks. Change = the runner's body, not recv_age_ms.
        info = self.latest.get("runtime_info")
        key = None if info is None else json.dumps(info[0], sort_keys=True, default=repr)
        last = self.runtime_info_sent
        due = last is None or last[0] != key or now - last[1] >= RUNTIME_INFO_RESEND_NS
        if due or force_runtime_info:
            if info is None:
                g["runtime_info"] = None
            else:
                g["runtime_info"] = dict(info[0])
                g["runtime_info"]["recv_age_ms"] = _ms(now - info[1])
            if due and not force_runtime_info:      # a snapshot never moves the broadcast schedule
                self.runtime_info_sent = (key, now)
                self.counts["runtime_info_sent"] += 1
        return g

    def _memory(self, now):
        g = self._group("memory", now)
        m = self._val("memory")
        if m is None:
            g.update({"revision": None, "n_tracks": None, "by_state": None, "digest": None,
                      "runtime_boot_id": None})
            return g
        by_state = dict((k, 0) for k in MEMORY_STATES)
        for t in m["tracks"]:
            if t.get("state") in by_state:
                by_state[t.get("state")] += 1
        g.update({"revision": m.get("revision"), "n_tracks": len(m["tracks"]), "by_state": by_state,
                  "digest": m.get("digest"), "runtime_boot_id": m.get("runtime_boot_id"),
                  "stamp_ros_s": m.get("stamp_ros_s")})
        return g

    def _region(self, now):
        g = self._group("region", now)
        r = self._val("region")
        if r is None:
            g["last"] = None
            return g
        counts = {"occluded": 0, "free": 0, "indeterminate": 0, "out_of_view": 0, "too_small": 0}
        for v in r.get("verdicts") or []:
            word = v.get("geometry_evidence")
            if word in counts:
                counts[word] += 1
            else:
                counts["indeterminate"] += 1
        unchecked = r.get("unchecked") or []
        times = r.get("roi_times_ms") or {}
        g["last"] = {"capture_stamp_ros_s": r.get("stamp_ros_s"), "frame_tag": r.get("frame_tag"),
                     "memory_revision": r.get("memory_revision"),
                     "n_sent": sum(1 for v in (r.get("verdicts") or []) if v.get("code") is not None),
                     "n_overflow": sum(1 for u in unchecked if u.get("reason") == "overflow"),
                     "verdict_counts": counts, "backend": r.get("backend"), "roi_core": r.get("roi_core"),
                     "roi_status": r.get("roi_status"), "failure": r.get("failure"),
                     "roi_device_ms": times.get("device") if isinstance(times, dict) else None,
                     "roi_cycles": r.get("roi_cycles")}
        return g

    def _pose(self, now):
        g = self._group("pose", now)
        p = self._val("loc_pose") or {}
        st = self._val("loc_status") or {}
        g.update({"x": p.get("x"), "y": p.get("y"), "yaw": p.get("yaw"), "cov_xy_trace": p.get("cov_xy_trace"),
                  "stamp_ros_s": p.get("stamp_ros_s"), "loc_state": st.get("state"),
                  "loc_reason": st.get("reason"), "epoch": st.get("epoch")})
        return g

    def _map(self, now):
        g = self._group("map", now)
        r = self._val("rolling_status") or {}
        gl = self._val("map_global") or {}
        g.update({"rolling_fresh": r.get("fresh"), "rolling_reason": r.get("reason"),
                  "rolling_age_ms": r.get("age_ms"), "epoch": r.get("epoch"), "delta_seq": r.get("delta_seq"),
                  "global_rev": gl.get("rev"), "width": gl.get("width"), "height": gl.get("height"),
                  "resolution_m": gl.get("resolution_m")})
        return g

    def _nav(self, now):
        g = self._group("nav", now)
        cmd = self._val("cmd_vel")
        plan = self._val("plan")
        g.update({"motion_permit": self._val("motion_permit"),
                  "cmd_v": None if cmd is None else cmd[0], "cmd_w": None if cmd is None else cmd[1],
                  "cmd_age_ms": None if cmd is None else _ms(now - self.latest["cmd_vel"][1]),
                  "path_len_m": None if plan is None else plan["len"]})
        if plan is None:
            g.update({"path": None, "path_reason": "no_plan_received"})
        elif plan["path"] is None:
            g.update({"path": None, "path_reason": plan["reason"]})
        else:
            path = dict(plan["path"])
            path["recv_age_ms"] = _ms(now - self.latest["plan"][1])
            g.update({"path": path, "path_reason": None})
        return g

    def _odom(self, now):
        g = self._group("odom", now)
        o = self._val("odom") or {}
        a = self._val("odom_adapter") or {}
        g.update({"v": o.get("v"), "w": o.get("w"), "stamp_ros_s": o.get("stamp_ros_s"),
                  "x": o.get("x"), "y": o.get("y"), "yaw": o.get("yaw"), "frame_id": o.get("frame_id"),
                  "adapter_state": a.get("state")})
        return g

    def _kv(self, now):
        tel = self._val("telemetry")
        link_ok = (tel is not None and self.telem["last_ok_ns"] is not None
                   and now - self.telem["last_ok_ns"] <= LINK_LOST_NS and self.telem["consecutive"] < 10)
        base = {"link_ok": link_ok, "telem_timeouts": self.telem["timeouts"],
                "telem_consecutive_timeouts": self.telem["consecutive"]}
        if tel is None:
            g = self._group("kv", now)
            g.update(base)
            g.update({"telem_rtt_ms": None, "kv_boot_id": None, "geometry": None, "permit": None,
                      "encoder": None, "ar": None, "proc": None})
            return g
        delta = _ms(now - self.latest["telemetry"][1])
        g = self._group("kv", now)              # age fixed below from the blocks it shows
        g["ok"] = link_ok
        g["reason"] = None if link_ok else "telemetry_lost"
        g["age_bound"] = "upper"
        g.update(base)
        g.update({"telem_rtt_ms": tel.rtt_ms, "kv_boot_id": tel.kv_boot_id})
        avail = set(tel.avail or ())
        blocks = tel.blocks or {}

        def age(block):
            up = (tel.age_upper_ms or {}).get(block)
            return None if up is None else up + delta

        def block(name):
            b = blocks.get(name)
            if name not in avail or not isinstance(b, dict) or b.get("available") is False:
                reason = b.get("reason") if isinstance(b, dict) else "not_reported"
                return None, {"avail": False, "reason": reason or "not_available"}
            return b, {"avail": True, "reason": None, "age_ms": age(name)}

        b, geo = block("geometry")
        if b is not None:
            last = b.get("last") or {}
            counts = last.get("counts") or [None, None, None]
            times = last.get("times_ms")
            if times is None and isinstance(last.get("times_ns"), dict):
                times = dict((k, v / 1e6) for k, v in last["times_ns"].items())
            times = times or {}
            geo.update({"core": b.get("core"), "wait": b.get("wait"), "last_frame_id": last.get("frame_id"),
                        "status": last.get("status"), "rays": counts[0], "invalid": counts[1],
                        "clipped": counts[2], "pl_device_ms": times.get("core_device"),
                        "pl_cycles": last.get("pl_cycles"), "core_total_ms": times.get("core_total"),
                        "served": b.get("served"), "slot_superseded": b.get("slot_superseded")})
        g["geometry"] = geo

        b, permit = block("permit")
        if b is not None:
            st = b.get("status")
            bit = (lambda i: None if st is None else bool((st >> i) & 1))
            # heartbeat_on: the PL drives the 100 Hz heartbeat square wave exactly while STATUS b0
            # (permit) is 1 (RTL m7_permit_axil.v:184-186), so it is b0. "pin" stays (compat) but is
            # one instantaneous read of b1: 10 Hz polling of a 100 Hz wave is phase-locked (aliased)
            # and reads a near-constant level -- debug only, never a heartbeat indicator (10-14).
            permit.update({"permit": bit(0), "heartbeat_on": bit(0), "pin": bit(1),
                           "accepted": bit(2), "estop": bit(3),
                           "fresh": bit(5), "last_cause": b.get("last_cause"),
                           "revoke_count": b.get("revoke_count"), "frames_accepted": b.get("frames_accepted"),
                           "frames_rejected": b.get("frames_rejected"), "permit_age_ms": b.get("age_ms")})
        g["permit"] = permit

        b, enc = block("encoder")
        if b is not None:
            st = b.get("status")
            seqw = b.get("seq_word")
            counters = b.get("counters")
            enc.update({"link_ok": None if st is None else bool(st & 1),
                        "v_left_mm_s": b.get("v_left_mm_s"), "v_right_mm_s": b.get("v_right_mm_s"),
                        "v_center_mm_s": b.get("v_center_mm_s"), "dist_center_m": b.get("dist_center_m"),
                        "heading_rad": b.get("heading_rad"),
                        "mcu_hb_seen": b.get("mcu_hb_seen"), "motor_enabled": b.get("motor_enabled"),
                        "seq": b.get("seq") if b.get("seq") is not None else seqw,
                        "err_counts": None if counters is None else dict(zip(ENCODER_COUNTERS, counters)),
                        "frame_age_ms": b.get("frame_age_ms")})
        if b is not None:
            # no MCU on the UART (never a frame, or LINK_OK low): not connected -- shown, but its
            # frame age is not the age of this group's data
            enc["connected"] = bool(enc.get("link_ok")) and age("encoder") is not None
        g["encoder"] = enc

        b, roi = block("roi")
        if b is not None:
            last = b.get("last") or {}
            roi.update({"core": b.get("core"), "last_frame_id": last.get("frame_id"),
                        "roi_status": last.get("roi_status"), "rows": last.get("rows"),
                        "served": b.get("served")})
        g["roi"] = roi

        b, ar = block("ar")
        if b is not None:
            ar.update({k: b.get(k) for k in ("mode", "age_valid", "replay", "replay_core", "frame_id")})
            # kv.ar.age_ms (spec K1 row name; 2026-10-12 v5, was ar_age_ms): the age of the frame the AR
            # helper last drew, at drawing time (TELEM ar.age_ms, host/kv_ar.py ar_age_ms formula).
            # The TELEM carries no sample age for the ar block (kv_status puts none in age_upper_ms),
            # so the slot holds this and ar stays out of the group age (geometry/permit/encoder only).
            ar["age_ms"] = b.get("age_ms")
        g["ar"] = ar

        b, proc = block("proc")
        if b is not None:
            proc.update({"cpu_per_wall": b.get("cpu_per_wall_1s"), "snmp_errors": b.get("snmp"),
                         "worker_error": b.get("worker_error")})
        g["proc"] = proc
        # Group age (upper bound) = TELEM sample age + the oldest block that carries live data:
        # geometry, permit, a connected encoder. Excluded, each named with its reason in
        # age_excluded (never silently): unavailable blocks; a block that has no data yet; the
        # encoder without an MCU on the UART; roi (answers only when a frame carries boxes, so
        # with no boxes in view its last answer just gets old -- its own age_ms shows that);
        # permit while the e-stop is latched (estop_latched, 急停锁存; controller 10-11 night): the
        # PL accepts no frame done-edge while forced off (FRAMES_ACCEPTED frozen), so its age is
        # the time since the last accepted frame, i.e. the latch duration, not link staleness
        # (10-11 v2 formal: 2.1 s at the end of a 2 s latch while geometry stayed ~100 ms).
        # Latched = the hub's latch (incl. one adopted from the KV) or the permit block's own
        # ESTOP status bit in this sample; permit.age_ms / permit_age_ms stay shown.
        # After the release, until the PL accepts the next frame (FRAMES_ACCEPTED moves past the value
        # it was frozen at during the latch), since_done still measures the latch, not the link: permit
        # stays out with reason "estop_release_pending" (解除待接受新帧; controller 10-11 night, v3) and
        # counts again from the first sample that shows a newly accepted frame. Unknown counter (None
        # on either side) or another KV boot: no pending (permit counts, the pre-v3 rule).
        estop_latched = bool(self.estop.latched) or bool(permit.get("avail") and permit.get("estop"))
        fa = permit.get("frames_accepted") if permit.get("avail") else None
        st = self.kv_permit_latch
        if estop_latched:
            st["latched"], st["pending"] = True, False
            if fa is not None:
                st["fa"], st["boot"] = fa, tel.kv_boot_id
        else:
            if st["latched"]:
                st["pending"] = st["fa"] is not None
                st["latched"] = False
            if st["pending"] and (fa is None or fa != st["fa"] or tel.kv_boot_id != st["boot"]):
                st["pending"] = False
            if not st["pending"]:
                st["fa"] = st["boot"] = None
        release_pending = st["pending"]
        used, excluded = {}, {}
        for name, info in (("geometry", geo), ("permit", permit), ("encoder", enc), ("roi", roi)):
            a = info.get("age_ms") if info.get("avail") else None
            if not info.get("avail"):
                excluded[name] = "unavailable"
            elif name == "roi":
                excluded[name] = "event_driven"
            elif name == "encoder" and not info.get("connected"):
                excluded[name] = "not_connected"
            elif name == "permit" and estop_latched:
                excluded[name] = "estop_latched"
            elif name == "permit" and release_pending:
                excluded[name] = "estop_release_pending"
            elif a is None:
                excluded[name] = "no_data"
            else:
                used[name] = a - delta
        g["age_ms"] = delta + (max(used.values()) if used else 0.0)
        g["age_basis"] = sorted(used)
        g["age_excluded"] = excluded
        return g

    def _estop(self, now):
        g = {"src": self.data_source, "age_ms": 0.0, "ok": True, "reason": None}
        g.update(self.estop.snapshot(now))
        return g

    def _clock(self, now):
        g = self._group("clock", now)
        r = self._val("clock")
        if r is None:
            g.update({"offset_us": None, "drift_ppm": None, "bound_us": None, "consistent": None,
                      "rejected": None, "pl_timebase": None})
            return g
        km = r.get("kv_map") or {}
        pm = r.get("pl_map") or {}
        counts = r.get("counts") or {}
        off = km.get("offset_ns_at_anchor")
        half = km.get("anchor_rtt_half_ns")
        g.update({"offset_us": None if off is None else off / 1e3, "drift_ppm": km.get("drift_ppm"),
                  "bound_us": None if half is None else half / 1e3, "consistent": km.get("consistent"),
                  "rejected": sum(v for k, v in counts.items() if "reject" in k or "stale" in k),
                  "pl_timebase": "error" not in pm, "error": km.get("error")})
        if "error" in km:
            g["ok"], g["reason"] = False, km["error"]
        return g

    def _geometry_nano(self, now):
        g = self._group("geometry_nano", now)
        f = self._val("frame_stats") or {}
        c = f.get("client_counts") or {}
        sup = sum(v for k, v in c.items() if "superseded" in k) if c else None
        g.update({"frame_age_ms": f.get("frame_age_ms"), "channel_ms": f.get("channel_ms"),
                  "mailbox_wait_ms": f.get("mailbox_wait_ms"), "result": f.get("result"),
                  "kv_frame_id": f.get("kv_frame_id"), "frame_source": f.get("source"),
                  "frame_source_mismatch": f.get("source_mismatch"),
                  # producer's own warning (v5): replay_without_marker | marker_mismatch | null;
                  # its /fpga/status source_warning(_cleared) events go out as EVENT like any other
                  "source_warning": f.get("source_warning"),
                  # hub's own runner-vs-producer check (v5 controller): runner_producer_mismatch | null
                  "runner_source_warning": (RUNNER_PRODUCER_MISMATCH
                                            if self.runner_producer_state == "inconsistent" else None),
                  "expired_output": c.get("expired_output"), "superseded_total": sup,
                  "timeouts": c.get("timeouts", c.get("timeout")), "crc_mismatch": c.get("crc_mismatch"),
                  "record_drops": self.record_drops["producer"],
                  "depth_invalid_s": (None if self.depth_invalid_since_ns is None
                                      else round((now - self.depth_invalid_since_ns) / 1e9, 1))})
        return g

    def _perf(self, now):
        g = self._group("perf", now)
        rows = list(self.frame_stats)
        dev = [(r.get("kv_times_ms") or {}).get("core_device") for _, r in rows]
        dev = [x for x in dev if x is not None]
        ages = [r.get("frame_age_ms") for _, r in rows if r.get("frame_age_ms") is not None]
        fps = None
        if len(rows) >= 2 and rows[-1][0] > rows[0][0]:
            fps = (len(rows) - 1) / ((rows[-1][0] - rows[0][0]) / 1e9)
        g.update({"pl_device_ms_med10": _median(dev), "frame_age_ms_med10": _median(ages),
                  "frames_per_s": fps,
                  # 10-29 console FPGA panel (host/fpga_metrics.py, display only): PL core times / counts of
                  # the same rows (frame_stats "fpga", producer)
                  "fpga": fpga_summarize([r for _, r in rows])})
        return g

    def _nano(self, now):
        g = self._group("nano", now)
        t = self._val("tegrastats") or {}
        g.update({"power_mode": self._val("nvpmodel"), "vdd_in_mw": t.get("vdd_in_mw"),
                  "cpu_pct": t.get("cpu_pct"), "gpu_pct": t.get("gpu_pct"), "temp_c_max": t.get("temp_c_max")})
        return g

    def _hub(self, now):
        g = {"src": self.data_source, "age_ms": 0.0, "ok": True, "reason": None}
        g.update({"clients": self.hub_stats["clients"], "events_dropped": self.counts["events_dropped"],
                  "superseded": self.hub_stats["superseded"], "cpu_pct": self.hub_stats["cpu_pct"],
                  "effective_sndbuf": self.hub_stats.get("effective_sndbuf"),
                  "executor": None if self.executor_info is None else dict(self.executor_info),
                  "counts": dict((k, v) for k, v in self.counts.items()
                                 if k.startswith(("estop_", "cmd_", "telem_", "bad_input", "state_nonfinite",
                                                  "event_nonfinite", "executor")))})
        return g

    # ------------------------------------------------------------------ MEMORY / MAP / PREVIEW
    def memory_body(self, nano_ns, force=False):
        """MEMORY body if changed (<= 2 Hz) or 5 s since the last; ``force`` on (re)connect."""
        row = self.latest.get("memory")
        if row is None:
            return None
        m, recv = row
        rev, sent_ns = self.memory_sent
        changed = m.get("revision") != rev
        # 10-27 fast removal: an object leaving / (re)appearing on the map goes out on the next tick, not after 0.5 s
        shown = frozenset(t.get("id") for t in m.get("tracks") or () if isinstance(t, dict)
                          and t.get("state") != "position_invalid")
        due = (force or (changed and (sent_ns is None or nano_ns - sent_ns >= NS // 2 or shown != self.memory_shown))
               or (sent_ns is not None and nano_ns - sent_ns >= 5 * NS))
        if not due:
            return None
        if not force:
            self.memory_sent = (m.get("revision"), nano_ns)
            self.memory_shown = shown
        body = dict(m)
        body["source"] = self.frame_source()
        body["src"] = self._ros_src()
        body["age_ms"] = _ms(nano_ns - recv)
        return body

    def map_frames(self, nano_ns, force=False):
        out = []
        for which, e in sorted(self.maps.items()):
            meta = e["meta"]
            key = (meta.get("rev"), zlib.crc32(e["cells"])) if which in ("rolling", "belief") else meta.get("rev")
            min_gap = {"rolling": NS // 2, "belief": NS // 4}.get(which, 5 * NS)
            if not force:
                if key == e["sent_key"]:
                    continue
                if e["sent_ns"] is not None and nano_ns - e["sent_ns"] < min_gap:
                    continue
                e["sent_key"], e["sent_ns"] = key, nano_ns
            m = {"which": which, "rev": meta.get("rev"), "width": meta.get("width"),
                 "height": meta.get("height"), "resolution_m": meta.get("resolution_m"),
                 "origin": meta.get("origin"), "frame_id": meta.get("frame_id"), "encoding": "int8+zlib",
                 "src": self._ros_src(), "age_ms": _ms(nano_ns - e["recv"])}
            if which == "belief":                  # 10-26 heat map display fields (phase, next look, result)
                m["belief"] = dict(meta.get("belief") or {})
            out.append((m, zlib.compress(e["cells"])))
        return out

    def preview_frame(self, nano_ns):
        p = self.preview
        if p is None or p["sent"]:
            return None
        p["sent"] = True
        meta = dict(p["meta"])
        meta.update({"src": self._ros_src(), "age_ms": _ms(nano_ns - p["recv"]), "encoding": "jpeg"})
        mem = self.latest.get("memory")                  # 10-28: boxes follow the map (no position_invalid)
        ar = self.fpv.pair(meta.get("stamp_ros_s"), hidden_ids(mem[0]) if mem else frozenset())
        # 10-27 console boxes; None = no producer frames yet
        if ar is not None:
            meta["ar"] = ar
        return meta, p["jpeg"]


def ground_calib_notices(path):
    """10-28: [{code, level, text, source}] from a run's ground_calib.json when its fit was refused and a fallback
    mount is used ("alert" set by ground_calib.config_fallback); [] when the fit was adopted / no file / unreadable."""
    try:
        with open(path) as f:
            g = json.load(f)
    except (OSError, ValueError):
        return []
    alert = g.get("alert") if isinstance(g, dict) else None
    if not isinstance(alert, str) or not alert:
        return []
    src = (g.get("adopted") or {}).get("source")
    return [{"code": "ground_calib_fallback", "level": "warn", "text": alert[:100],
             "source": src if isinstance(src, str) else None}]
