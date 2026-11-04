"""FPGAC v1: the car <-> PC protocol, its client, recorder and replayer (one file, stdlib only).

Spec: docs/design/PC_DATAPLANE_SPEC_20261011.md sections C.1-C.8. This file is the only
implementation of the protocol; the Nano hub and the Windows PC both import it, so the two
sides cannot drift. Python 3.8+, no third-party packages, no POSIX-only APIs.

Layers (bottom up):
  * framing    -- 20-byte little-endian header + body, crc32, 4 MiB cap (C.2)
  * checks     -- source/src flags must be present or the frame is refused (C.6)
  * Connection -- one TCP socket: send with a lock, receive through a select() poll
  * FpgacClient -- dials the Nano, HELLO, ping, stale/dead detection, reconnect with
                  back-off and address discovery, command resend/dedup (C.3/C.5/C.8)
  * Recorder / read_fpgarec / replay_frames / ReplayServer -- .fpgarec files (C.6)
  * probe / scan -- hotspot diagnostics (C.8)

Ages: the PC never computes a data age; it only measures its own link age (time since the
last frame arrived, PC monotonic clock). Data ages come from the Nano's ``age_ms`` fields.
"""

import binascii
import collections
import ipaddress
import json
import math
import os
import platform
import socket
import struct
import subprocess
import threading
import time

__all__ = [
    "MAGIC", "VERSION", "HEADER", "MAX_BODY", "KIND", "KIND_NAME", "STATE_GROUPS",
    "ProtocolError", "Frame", "Decoder", "SeqCheck", "encode_frame", "encode_json",
    "encode_binary", "decode_binary", "check_source_flags", "Connection", "FpgacClient",
    "Recorder", "read_fpgarec", "verify_fpgarec", "replay_frames", "force_replay",
    "ReplayServer", "probe", "scan", "local_ipv4", "set_keepalive", "percentile",
    "kv_blocks", "nav_path", "RuntimeInfoKeeper",
]

MAGIC = b"FPGAC"
VERSION = (1, 0)
HEADER = struct.Struct("<4sBBHIII")     # magic, ver_major, ver_minor, kind, seq, body_len, crc32
assert HEADER.size == 20
MAX_BODY = 4 * 1024 * 1024
STALE_FRAME_MS = 200.0           # a frame that waited this long in the link is marked stale
DEFAULT_PORT = 47410

KIND = {"HELLO": 1, "HELLO_ACK": 2, "STATE": 3, "MEMORY": 4, "EVENT": 5, "MAP": 6,
        "PREVIEW": 7, "CMD": 8, "CMD_ACK": 9, "CMD_RESULT": 10, "PING": 11, "PONG": 12,
        "BYE": 13}
KIND_NAME = {v: k for k, v in KIND.items()}
BINARY_KINDS = (KIND["MAP"], KIND["PREVIEW"])

# Every STATE frame carries all of these groups, each with its own ``src`` (C.4).
STATE_GROUPS = ("task", "memory", "region", "pose", "map", "nav", "odom", "kv", "estop",
                "clock", "geometry_nano", "perf", "nano", "hub")
FRAME_SOURCES = ("live", "replay", "sim", "inconsistent")     # sim: Gazebo simulation (line D1)
GROUP_SRCS = ("live", "replay", "sim", "unknown", "unavailable")
OPS = ("find", "select", "cancel", "estop", "release", "yield_control", "patrol",
       "cold_start_confirm", "goto", "teleop", "mapping_stop",
       "map_reset")   # teleop / mapping_stop: 10-18, map_reset: 10-26 (host/hub/hub_core.py)


class ProtocolError(Exception):
    """The byte stream can no longer be trusted (C.2): log, drop the connection, reconnect."""

    def __init__(self, reason, detail=""):
        Exception.__init__(self, reason + ((": " + detail) if detail else ""))
        self.reason = reason


def dumps(obj):
    return json.dumps(obj, ensure_ascii=False, allow_nan=False, separators=(",", ":"),
                      sort_keys=True).encode("utf-8")


def encode_frame(kind, seq, body, ver=VERSION):
    body = bytes(body)
    if len(body) > MAX_BODY:
        raise ProtocolError("too_long", str(len(body)))
    return HEADER.pack(MAGIC, ver[0], ver[1], int(kind), int(seq) & 0xFFFFFFFF, len(body),
                       binascii.crc32(body) & 0xFFFFFFFF) + body


def encode_json(kind, seq, obj, ver=VERSION):
    return encode_frame(kind, seq, dumps(obj), ver)


def binary_body(meta, payload):
    m = dumps(meta)
    return struct.pack("<I", len(m)) + m + bytes(payload)


def encode_binary(kind, seq, meta, payload, ver=VERSION):
    return encode_frame(kind, seq, binary_body(meta, payload), ver)


def decode_binary(body):
    if len(body) < 4:
        raise ProtocolError("bad_binary", "short")
    (n,) = struct.unpack_from("<I", body)
    if 4 + n > len(body):
        raise ProtocolError("bad_binary", "meta length")
    try:
        meta = json.loads(body[4:4 + n].decode("utf-8"))
    except ValueError as exc:
        raise ProtocolError("bad_json", str(exc))
    if not isinstance(meta, dict):
        raise ProtocolError("bad_json", "meta not an object")
    return meta, body[4 + n:]


class Frame(object):
    __slots__ = ("kind", "seq", "ver", "body", "raw", "_obj", "lag_ms", "stale")

    def __init__(self, kind, seq, ver, body, raw):
        self.kind, self.seq, self.ver, self.body, self.raw = kind, seq, ver, body, raw
        self._obj = None
        self.lag_ms = None       # link delay beyond the session's best (FpgacClient sets it)
        self.stale = False       # lag_ms >= stale threshold: delivered and recorded, flagged

    @property
    def name(self):
        return KIND_NAME.get(self.kind, "UNKNOWN_%d" % self.kind)

    def obj(self):
        """JSON body (dict) for JSON kinds; (meta, payload) for MAP/PREVIEW."""
        if self._obj is None:
            if self.kind in BINARY_KINDS:
                self._obj = decode_binary(self.body)
            else:
                try:
                    value = json.loads(self.body.decode("utf-8"))
                except ValueError as exc:
                    raise ProtocolError("bad_json", str(exc))
                if not isinstance(value, dict):
                    raise ProtocolError("bad_json", "body not an object")
                self._obj = value
        return self._obj


class Decoder(object):
    """Incremental stream decoder; raises ProtocolError on magic/length/crc faults."""

    def __init__(self, max_body=MAX_BODY):
        self.buf = bytearray()
        self.max_body = max_body

    def feed(self, data):
        self.buf += data
        out = []
        while len(self.buf) >= HEADER.size:
            magic, vmaj, vmin, kind, seq, n, crc = HEADER.unpack_from(self.buf)
            if magic != MAGIC:
                raise ProtocolError("bad_magic", repr(bytes(magic)))
            if n > self.max_body:
                raise ProtocolError("too_long", str(n))
            if len(self.buf) < HEADER.size + n:
                break
            raw = bytes(self.buf[:HEADER.size + n])
            body = raw[HEADER.size:]
            del self.buf[:HEADER.size + n]
            if binascii.crc32(body) & 0xFFFFFFFF != crc:
                raise ProtocolError("bad_crc", "kind %d seq %d" % (kind, seq))
            out.append(Frame(kind, seq, (vmaj, vmin), body, raw))
        return out


def decode_one(raw):
    frames = Decoder().feed(raw)
    if len(frames) != 1:
        raise ProtocolError("bad_record", "not exactly one frame")
    return frames[0]


class SeqCheck(object):
    """Per connection and direction seq starts at 1, +1 each frame (C.2)."""

    def __init__(self, counts):
        self.expect = 1
        self.counts = counts

    def check(self, seq):
        if seq == self.expect:
            ok = True
        elif seq > self.expect:
            self.counts["seq_gap"] += 1
            ok = False
        else:
            self.counts["seq_back"] += 1
            ok = False
        self.expect = (seq + 1) & 0xFFFFFFFF
        return ok


KV_BLOCKS = ("geometry", "permit", "encoder", "roi", "ar", "proc")


def kv_blocks(state):
    """Per KV260 block of a STATE frame's ``kv`` group: {name: {"state", "age_ms", "reason"}}.

    state: ``live`` (data with an age), ``not_connected`` (encoder without an MCU on the
    UART), ``unavailable`` (block not present on this bitstream/build, with the Nano's
    reason), ``no_data`` (available, nothing yet), ``event_driven`` (roi: answers only for
    frames that carry boxes), ``unknown`` (no TELEM yet). Whether a block entered the group
    ``age_ms`` is the hub's ``age_basis``/``age_excluded`` (spec A.3); this is only a reading
    aid for UIs, the PC computes no age itself. For ``ar`` the ``age_ms`` is ``kv.ar.age_ms``: the
    age of the frame the AR display last drew, at drawing time (2026-10-12 v5; the hub used to call
    it ``ar_age_ms`` and leave ``age_ms`` null); it never enters the kv group age."""
    kv = (state or {}).get("kv") or {}
    excluded = kv.get("age_excluded") or {}
    out = {}
    for name in KV_BLOCKS:
        b = kv.get(name)
        if not isinstance(b, dict):
            out[name] = {"state": "unknown", "age_ms": None, "reason": None}
            continue
        if not b.get("avail"):
            st = "unavailable"
        elif excluded.get(name) in ("not_connected", "event_driven", "no_data"):
            st = excluded[name]
        elif name == "encoder" and b.get("connected") is False:
            st = "not_connected"
        elif b.get("age_ms") is None and name in ("geometry", "permit", "encoder"):
            st = "no_data"
        else:
            st = "live"
        out[name] = {"state": st, "age_ms": b.get("age_ms"), "reason": b.get("reason")}
    return out


def kv_heartbeat(state):
    """Is the PL heartbeat wave being driven? True/False, or None (no permit block / no data).

    Reads ``kv.permit.heartbeat_on`` (= STATUS b0, permit: the RTL drives the 100 Hz wave exactly
    while permit is 1); a hub without that key falls back to ``kv.permit.permit``. Never reads
    ``kv.permit.pin``: that is one instantaneous sample of the 100 Hz wave taken at 10 Hz, phase-
    locked to the wave (aliased), so it reads a near-constant level whatever the wire does."""
    kv = (state or {}).get("kv") or {}
    p = kv.get("permit")
    if not isinstance(p, dict) or not p.get("avail"):
        return None
    v = p.get("heartbeat_on", p.get("permit"))
    return v if isinstance(v, bool) else None


PATH_MAX_POINTS = 64                 # = host/hub/hub_core.PATH_MAX_POINTS (spec C.4 nav.path)
PATH_REASONS = ("no_plan_received", "empty_plan", "bad_plan", "length_only")


def _finite_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def nav_path(state):
    """The planner's route from a STATE frame's ``nav`` group, data only (drawing is the UI's).

    Returns ``{"reason": None, "frame_id", "points": [(x, y), ...], "stamp_ros_s", "length_m",
    "n_source_poses", "epsilon_m", "recv_age_ms"}`` -- coordinates in ``frame_id`` exactly as the
    Nano got them (no transform) -- or ``{"reason": r}``: the hub's ``path_reason``
    (no_plan_received / empty_plan / bad_plan / length_only) or ``malformed`` (wrong shape, more
    than PATH_MAX_POINTS points, a non-finite or non-numeric coordinate, frame_id not a string)."""
    nav = (state or {}).get("nav")
    if not isinstance(nav, dict):
        return {"reason": "malformed"}
    path = nav.get("path")
    if path is None:
        r = nav.get("path_reason")
        return {"reason": r if r in PATH_REASONS else "malformed"}
    if not isinstance(path, dict) or not isinstance(path.get("frame_id"), str):
        return {"reason": "malformed"}
    pts = path.get("points")
    if not isinstance(pts, list) or not pts or len(pts) > PATH_MAX_POINTS:
        return {"reason": "malformed"}
    out = []
    for p in pts:
        if not isinstance(p, list) or len(p) != 2 or not all(_finite_number(v) for v in p):
            return {"reason": "malformed"}
        out.append((float(p[0]), float(p[1])))
    res = {"reason": None, "frame_id": path["frame_id"], "points": out}
    for k in ("stamp_ros_s", "length_m", "n_source_poses", "epsilon_m", "recv_age_ms"):
        res[k] = path.get(k)
    return res


class RuntimeInfoKeeper(object):
    """Last ``task.runtime_info`` seen in STATE (2026-10-12 v5). The hub sends that field only
    when it changed and then every 5 s, and leaves it out of the other ticks (spec C.4), so a UI
    reads it from here, not from the newest STATE. ``value`` is None until a STATE carried it,
    and also when the Nano had not received the runner's /fpga/runtime_info (``present`` tells the
    two apart); ``seq_state``/``nano_mono_ns``/``hub_boot_id`` name the STATE it came from. A STATE
    from another hub boot resets it, so an old hub's runtime facts never stand for a new one."""

    def __init__(self):
        self.present, self.value = False, None
        self.seq_state = self.nano_mono_ns = self.hub_boot_id = None

    def observe(self, state):
        """Feed one STATE body; returns the retained value."""
        if not isinstance(state, dict):
            return self.value
        boot = state.get("hub_boot_id")
        if self.present and boot != self.hub_boot_id:
            self.__init__()
        task = state.get("task")
        if isinstance(task, dict) and "runtime_info" in task:
            self.present, self.value = True, task["runtime_info"]
            self.seq_state, self.nano_mono_ns = state.get("seq_state"), state.get("nano_mono_ns")
            self.hub_boot_id = boot
        return self.value


def check_source_flags(kind, obj):
    """None if the frame carries its live/replay marks, else the missing-flag reason (C.6)."""
    if kind == KIND["STATE"]:
        if obj.get("source") not in FRAME_SOURCES:
            return "state.source"
        for group in STATE_GROUPS:
            g = obj.get(group)
            if not isinstance(g, dict) or g.get("src") not in GROUP_SRCS:
                return "state.%s.src" % group
        return None
    if kind == KIND["MEMORY"]:
        return None if obj.get("source") in FRAME_SOURCES else "memory.source"
    if kind == KIND["EVENT"]:
        return None if obj.get("src") in GROUP_SRCS else "event.src"
    if kind in BINARY_KINDS:
        meta = obj[0]
        return None if meta.get("src") in GROUP_SRCS else "%s.src" % KIND_NAME[kind].lower()
    return None


def percentile(values, q):
    if not values:
        return None
    s = sorted(values)
    k = (len(s) - 1) * q / 100.0
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def set_keepalive(sock, idle_s=5, interval_s=2, count=3):
    """Linux TCP_KEEPIDLE/INTVL/CNT or Windows SIO_KEEPALIVE_VALS; whatever this OS has."""
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    if hasattr(socket, "SIO_KEEPALIVE_VALS") and hasattr(sock, "ioctl"):
        sock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, int(idle_s * 1000), int(interval_s * 1000)))
        return "windows"
    done = []
    for name, value in (("TCP_KEEPIDLE", idle_s), ("TCP_KEEPINTVL", interval_s),
                        ("TCP_KEEPCNT", count)):
        if hasattr(socket, name):
            sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, name), int(value))
            done.append(name)
    return "posix:" + ",".join(done)


class Connection(object):
    """One FPGAC TCP stream. send() is thread safe; recv(timeout) polls with select."""

    def __init__(self, sock, counts=None, send_timeout_s=5.0):
        self.sock = sock
        self.counts = counts if counts is not None else collections.Counter()
        self.decoder = Decoder()
        self.tx_seq = 0
        self.rx_seq = SeqCheck(self.counts)
        self.lock = threading.Lock()
        self.closed = False
        self.pending = collections.deque()
        sock.settimeout(send_timeout_s)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def send(self, kind, obj=None, body=None, meta=None, payload=b""):
        with self.lock:
            if self.closed:
                raise OSError("connection closed")
            self.tx_seq = (self.tx_seq + 1) & 0xFFFFFFFF
            if meta is not None:
                raw = encode_binary(kind, self.tx_seq, meta, payload)
            elif body is not None:
                raw = encode_frame(kind, self.tx_seq, body)
            else:
                raw = encode_json(kind, self.tx_seq, obj)
            self.sock.sendall(raw)
            return raw

    def recv(self, timeout):
        """Next frame or None on timeout; raises ProtocolError / ConnectionError."""
        import select
        if self.pending:
            return self.pending.popleft()
        end = time.monotonic() + timeout
        while True:
            left = end - time.monotonic()
            if left < 0:
                return None
            readable, _, _ = select.select([self.sock], [], [], left)
            if not readable:
                return None
            data = self.sock.recv(65536)
            if not data:
                raise ConnectionError("peer closed")
            frames = self.decoder.feed(data)
            for f in frames:
                self.rx_seq.check(f.seq)
            if frames:
                self.pending.extend(frames[1:])
                return frames[0]

    def close(self):
        with self.lock:
            self.closed = True
        try:
            self.sock.close()
        except OSError:
            pass


# --------------------------------------------------------------------------------------------
# Client (PC side)
# --------------------------------------------------------------------------------------------

class LinkLag(object):
    """How long a frame waited in the link, without reading or mapping the Nano clock.

    d = pc_arrival_ns - frame.nano_mono_ns mixes two clocks, but its *minimum* over recent
    frames is (clock offset + fastest transit); ``d - min`` is then the extra time this frame
    spent queued (sender buffer, hotspot, receiver) -- a link delay, not a data age (data ages
    stay the Nano's ``age_ms``, spec C.3 / owner 6). The minimum is kept per 1 s bucket over
    ``window_s`` so clock drift between the boards cannot accumulate; reset on every new
    session. Error: the drift over the window plus the PC clock resolution (~16 ms on Windows).
    """

    def __init__(self, window_s=20.0):
        self.window_ns = int(window_s * 1e9)
        self.buckets = collections.OrderedDict()

    def reset(self):
        self.buckets.clear()

    def observe(self, pc_ns, nano_ns):
        d = int(pc_ns) - int(nano_ns)
        key = int(pc_ns) // 1000000000
        if key not in self.buckets or d < self.buckets[key]:
            self.buckets[key] = d
        while self.buckets and (next(iter(self.buckets)) + 1) * 1000000000 < pc_ns - self.window_ns:
            self.buckets.popitem(last=False)
        return (d - min(self.buckets.values())) / 1e6


class FpgacClient(object):
    """Dial the Nano hub, keep the link, hand validated frames to ``on_frame(frame, obj)``.

    Hosts are tried in order (C.8): ``hosts`` (mDNS name, last-success, configured IPs),
    then a /24 scan if ``scan_net`` is given. Frames lacking their live/replay marks are
    refused and counted (``missing_source_flag``), never passed on as live.
    """

    def __init__(self, port, token, hosts=(), scan_net=None, client_id=None, on_frame=None,
                 on_link=None, recorder=None, want=("state", "memory", "events", "map_rolling",
                                                    "map_global"), state_hz=10,
                 stale_s=1.5, dead_s=5.0, connect_timeout_s=3.0, hello_timeout_s=3.0,
                 ping_s=1.0, backoff_s=(0.5, 1.0, 2.0), last_file=None, expect_boot_id=None,
                 clock=time.monotonic_ns, stale_frame_ms=STALE_FRAME_MS):
        self.port, self.token = int(port), token
        self.hosts = list(hosts)
        self.scan_net = scan_net
        self.client_id = client_id or binascii.hexlify(os.urandom(6)).decode()
        self.on_frame, self.on_link, self.recorder = on_frame, on_link, recorder
        self.want, self.state_hz = list(want), state_hz
        self.stale_s, self.dead_s = stale_s, dead_s
        self.connect_timeout_s, self.hello_timeout_s = connect_timeout_s, hello_timeout_s
        self.ping_s, self.backoff_s = ping_s, tuple(backoff_s)
        self.last_file, self.expect_boot_id = last_file, expect_boot_id
        self.clock = clock
        self.stale_frame_ms = float(stale_frame_ms)
        self.lag = LinkLag()
        self.last_fresh_state = None          # newest STATE body not marked stale
        self.runtime_info = RuntimeInfoKeeper()   # task.runtime_info is not in every STATE (v5)
        self.lag_ms = []                       # per-STATE link lag (for stats)
        self.counts = collections.Counter()
        self.gaps_ms, self.rtt_ms = [], []
        self.conn = None
        self.hello_ack = None
        self.hub_boot_id = None
        self.host = None
        self.link_state = "down"
        self.acks, self.results = {}, {}
        self.cmd_n = 0
        self.unacked = collections.OrderedDict()   # cmd_id -> dict(body, first_ns, estop)
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self.connected = threading.Event()

    # ---- public -----------------------------------------------------------------------------
    def start(self):
        self._thread = threading.Thread(target=self._run, name="fpgac-client", daemon=True)
        self._thread.start()
        return self

    def stop(self, bye=True):
        self._stop.set()
        conn = self.conn
        if conn is not None and bye:
            try:
                conn.send(KIND["BYE"], {"v": 1, "reason": "client_stop"})
            except (OSError, ProtocolError):
                pass
        if self._thread is not None:
            self._thread.join(timeout=10)
        if self.conn is not None:
            self.conn.close()

    def command(self, op, args=None, deadline_ms=None):
        """Queue a command; returns its cmd_id. Estop has no deadline and resends every 0.5 s."""
        with self.lock:
            self.cmd_n += 1
            cmd_id = "%s:%d" % (self.client_id, self.cmd_n)
            if deadline_ms is None:
                deadline_ms = 0 if op == "estop" else 5000
            body = {"v": 1, "cmd_id": cmd_id, "op": op, "args": dict(args or {}),
                    "deadline_ms": deadline_ms}
            self.unacked[cmd_id] = {"body": body, "first_ns": self.clock(), "last_ns": None,
                                    "estop": op == "estop", "sent_conn": None}
        self._flush_commands()
        return cmd_id

    def wait_ack(self, cmd_id, timeout_s):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            with self.lock:
                if cmd_id in self.acks:
                    return self.acks[cmd_id]
            time.sleep(0.01)
        return None

    def wait_result(self, cmd_id, timeout_s, final=True):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            with self.lock:
                rows = self.results.get(cmd_id)
                if rows and (not final or rows[-1].get("final", True)):
                    return rows[-1]
            time.sleep(0.01)
        return None

    def stats(self):
        g = self.gaps_ms
        return {"frames": dict((k, v) for k, v in self.counts.items() if k.startswith("rx:")),
                "counts": dict((k, v) for k, v in self.counts.items() if not k.startswith("rx:")),
                "gap_ms": {"n": len(g), "p50": percentile(g, 50), "p99": percentile(g, 99),
                           "max": max(g) if g else None},
                "stall_events": sum(1 for x in g if x > 1000.0),
                "rtt_ms": {"n": len(self.rtt_ms), "p50": percentile(self.rtt_ms, 50),
                           "max": max(self.rtt_ms) if self.rtt_ms else None},
                "state_lag_ms": {"n": len(self.lag_ms), "p50": percentile(self.lag_ms, 50),
                                 "p99": percentile(self.lag_ms, 99),
                                 "max": max(self.lag_ms) if self.lag_ms else None,
                                 "stale_threshold_ms": self.stale_frame_ms},
                "host": self.host, "hub_boot_id": self.hub_boot_id, "link": self.link_state}

    # ---- internals ----------------------------------------------------------------------------
    def _link(self, state, **detail):
        if state != self.link_state:
            self.link_state = state
            self.counts["link:" + state] += 1
            if self.on_link:
                self.on_link(state, detail)

    def _candidates(self):
        hosts = list(self.hosts)
        if self.last_file and os.path.exists(self.last_file):
            try:
                with open(self.last_file, "r", encoding="utf-8") as f:
                    last = json.load(f).get("host")
                if last and last not in hosts:
                    hosts.insert(1 if hosts else 0, last)
            except (OSError, ValueError):
                pass
        return hosts

    def _remember(self, host):
        if self.last_file:
            try:
                with open(self.last_file, "w", encoding="utf-8") as f:
                    json.dump({"host": host}, f)
            except OSError:
                self.counts["last_file_write_failed"] += 1

    def _dial(self):
        for host in self._candidates():
            conn = self._handshake(host)
            if conn is not None:
                return host, conn
        if self.scan_net:
            for host in scan(self.port, self.token, self.scan_net, expect_boot_id=self.expect_boot_id):
                conn = self._handshake(host)
                if conn is not None:
                    return host, conn
        return None, None

    def _handshake(self, host):
        try:
            sock = socket.create_connection((host, self.port), timeout=self.connect_timeout_s)
        except (OSError, socket.timeout):
            self.counts["connect_failed"] += 1
            return None
        conn = Connection(sock, self.counts)
        try:
            set_keepalive(sock)
            conn.send(KIND["HELLO"], {"v": 1, "client_id": self.client_id, "token": self.token,
                                      "want": self.want, "state_hz": self.state_hz})
            frame = conn.recv(self.hello_timeout_s)
            if frame is None or frame.kind != KIND["HELLO_ACK"]:
                self.counts["hello_failed"] += 1
                conn.close()
                return None
            ack = frame.obj()
            if not ack.get("ok") or frame.ver[0] != VERSION[0]:
                self.counts["hello_rejected:%s" % ack.get("reason")] += 1
                conn.close()
                return None
            if self.expect_boot_id and ack.get("hub_boot_id") != self.expect_boot_id:
                self.counts["hello_wrong_boot_id"] += 1
                conn.close()
                return None
            self.hello_ack = ack
            if self.hub_boot_id is not None and ack.get("hub_boot_id") != self.hub_boot_id:
                self.counts["hub_restarted"] += 1
                if self.on_link:
                    self.on_link("hub_restarted", {"old": self.hub_boot_id, "new": ack.get("hub_boot_id")})
            self.hub_boot_id = ack.get("hub_boot_id")
            if self.recorder is not None:
                self.recorder.hello(ack, host)
                self.recorder.write(self.clock(), frame.raw)
            return conn
        except (OSError, ProtocolError, ValueError):
            self.counts["hello_failed"] += 1
            conn.close()
            return None

    def _flush_commands(self, reconnected=False):
        conn = self.conn
        if conn is None:
            return
        now = self.clock()
        with self.lock:
            due = []
            for cmd_id, row in list(self.unacked.items()):
                deadline = row["body"]["deadline_ms"]
                if not row["estop"] and deadline and now - row["first_ns"] > deadline * 1e6:
                    del self.unacked[cmd_id]
                    self.counts["cmd_deadline_expired"] += 1
                    self.results.setdefault(cmd_id, []).append(
                        {"status": "deadline_expired", "cmd_id": cmd_id, "local": True})
                    continue
                first_on_conn = row["sent_conn"] is not conn
                estop_resend = row["estop"] and row["last_ns"] is not None and now - row["last_ns"] >= 5e8
                if first_on_conn or estop_resend:
                    row["sent_conn"], row["last_ns"] = conn, now
                    due.append(row["body"])
        for body in due:
            try:
                conn.send(KIND["CMD"], body)
                self.counts["cmd_sent"] += 1
            except (OSError, ProtocolError):
                self.counts["cmd_send_failed"] += 1
                return

    def _deliver(self, frame, now):
        self.counts["rx:" + frame.name] += 1
        if self.recorder is not None:
            self.recorder.write(now, frame.raw)
        if frame.kind not in KIND_NAME:
            self.counts["unknown_kind"] += 1
            return
        obj = frame.obj()
        missing = check_source_flags(frame.kind, obj)
        if missing is not None:
            self.counts["missing_source_flag"] += 1
            self.counts["missing_source_flag:" + missing] += 1
            return
        nano_ns = obj.get("nano_mono_ns") if isinstance(obj, dict) else None
        if isinstance(nano_ns, int) and not isinstance(nano_ns, bool):
            frame.lag_ms = self.lag.observe(now, nano_ns)
            frame.stale = frame.lag_ms >= self.stale_frame_ms
            if frame.stale:
                self.counts["stale_frames"] += 1
                self.counts["stale_frames:" + frame.name] += 1
            if frame.kind == KIND["STATE"]:
                self.runtime_info.observe(obj)
                self.lag_ms.append(frame.lag_ms)
                if not frame.stale:
                    self.last_fresh_state = obj
        if frame.kind == KIND["PONG"]:
            sent = obj.get("pc_mono_ns")
            if isinstance(sent, int):
                self.rtt_ms.append((now - sent) / 1e6)
        elif frame.kind == KIND["CMD_ACK"]:
            with self.lock:
                self.acks[obj.get("cmd_id")] = obj
                self.unacked.pop(obj.get("cmd_id"), None)
        elif frame.kind == KIND["CMD_RESULT"]:
            with self.lock:
                self.results.setdefault(obj.get("cmd_id"), []).append(obj)
        if self.on_frame:
            self.on_frame(frame, obj)

    def _session(self, conn):
        last_rx = last_ping = self.clock()
        stale = False
        while not self._stop.is_set():
            frame = conn.recv(0.05)
            now = self.clock()
            if frame is not None:
                if frame.kind == KIND["BYE"]:
                    self.counts["peer_bye"] += 1
                    return "bye"
                self.gaps_ms.append((now - last_rx) / 1e6)
                last_rx = now
                if stale:
                    stale = False
                    self._link("up", recovered=True)
                self._deliver(frame, now)
            idle = (now - last_rx) / 1e9
            if idle > self.dead_s:
                self.counts["dead_reconnect"] += 1
                return "dead"
            if idle > self.stale_s and not stale:
                stale = True
                self.counts["link_stale"] += 1
                self._link("stale", idle_s=idle)
            if (now - last_ping) / 1e9 >= self.ping_s:
                last_ping = now
                conn.send(KIND["PING"], {"v": 1, "pc_mono_ns": now})
            self._flush_commands()
        return "stop"

    def _run(self):
        attempt = 0
        while not self._stop.is_set():
            host, conn = self._dial()
            if conn is None:
                delay = self.backoff_s[min(attempt, len(self.backoff_s) - 1)]
                attempt += 1
                self.counts["dial_failed"] += 1
                self._stop.wait(delay)
                continue
            attempt = 0
            self.host = host
            self._remember(host)
            self.conn = conn
            self.lag.reset()
            self.counts["connected"] += 1
            self._link("up", host=host)
            self.connected.set()
            self._flush_commands(reconnected=True)
            try:
                why = self._session(conn)
            except ProtocolError as exc:
                self.counts["protocol_error:" + exc.reason] += 1
                why = "protocol"
            except (OSError, ConnectionError):
                why = "stop" if self._stop.is_set() else "socket"
                if why == "socket":
                    self.counts["socket_error"] += 1
            self.connected.clear()
            self.conn = None
            conn.close()
            if why == "stop":
                break
            self._link("down", why=why)
            delay = self.backoff_s[min(attempt, len(self.backoff_s) - 1)]
            attempt += 1
            self._stop.wait(delay)
        self._link("down", why="stopped")


# --------------------------------------------------------------------------------------------
# Recording and replay (C.6)
# --------------------------------------------------------------------------------------------

REC_MAGIC = b"FPGAREC1\n"
REC_ENTRY = struct.Struct("<QI")


class Recorder(object):
    """``.fpgarec``: magic, one JSON line, then (u64 PC mono ns, u32 len, raw FPGAC frame)*.

    Refuses to overwrite: the file is created exclusively at construction.
    """

    def __init__(self, path, lib_version="fpgac-1.0"):
        self.path = path
        self.f = open(path, "xb")
        self.lib_version = lib_version
        self.header_written = False
        self.n = 0
        self.lock = threading.Lock()

    def hello(self, ack, host):
        with self.lock:
            if self.header_written:
                return
            head = {"lib": self.lib_version, "start_unix_s": time.time(), "host": host,
                    "hub_boot_id": ack.get("hub_boot_id"), "hello_ack": ack}
            self.f.write(REC_MAGIC + dumps(head) + b"\n")
            self.header_written = True

    def write(self, pc_ns, raw):
        with self.lock:
            if not self.header_written:
                raise RuntimeError("recorder header must precede frames")
            self.f.write(REC_ENTRY.pack(int(pc_ns), len(raw)) + raw)
            self.n += 1

    def close(self):
        with self.lock:
            self.f.close()


def read_fpgarec(path):
    """(header dict, [(pc_ns, raw bytes)]). Every raw frame is crc-checked here."""
    with open(path, "rb") as f:
        data = f.read()
    if not data.startswith(REC_MAGIC):
        raise ProtocolError("bad_record", "magic")
    nl = data.index(b"\n", len(REC_MAGIC))
    header = json.loads(data[len(REC_MAGIC):nl].decode("utf-8"))
    pos, rows = nl + 1, []
    while pos < len(data):
        if pos + REC_ENTRY.size > len(data):
            raise ProtocolError("bad_record", "truncated entry head")
        pc_ns, n = REC_ENTRY.unpack_from(data, pos)
        pos += REC_ENTRY.size
        raw = data[pos:pos + n]
        if len(raw) != n:
            raise ProtocolError("bad_record", "truncated frame")
        decode_one(raw)
        rows.append((pc_ns, raw))
        pos += n
    return header, rows


def verify_fpgarec(path):
    header, rows = read_fpgarec(path)
    kinds = collections.Counter(KIND_NAME.get(decode_one(raw).kind, "?") for _, raw in rows)
    return {"frames": len(rows), "kinds": dict(kinds), "hub_boot_id": header.get("hub_boot_id")}


def force_replay(kind, obj, replay_file):
    """Mark a recorded frame as replay before it reaches anyone (C.6)."""
    if kind in BINARY_KINDS:
        meta, payload = obj
        meta = dict(meta)
        if meta.get("src") in ("live", "sim"):
            meta["orig_src"], meta["src"] = meta["src"], "replay"
        meta["replay_file"] = replay_file
        return meta, payload
    obj = dict(obj)
    if kind in (KIND["STATE"], KIND["MEMORY"]):
        obj["orig_source"] = obj.get("source")
        obj["source"] = "replay"
    if kind == KIND["STATE"]:
        for group in STATE_GROUPS:
            g = obj.get(group)
            if isinstance(g, dict) and g.get("src") in ("live", "sim"):
                g = dict(g)
                g["orig_src"], g["src"] = g["src"], "replay"
                obj[group] = g
    if kind == KIND["EVENT"] and obj.get("src") in ("live", "sim"):
        obj["orig_src"], obj["src"] = obj["src"], "replay"
    obj["replay_file"] = replay_file
    return obj


def replay_frames(path, rate=1.0, sleep=time.sleep, kinds=None):
    """Yield (Frame, obj) re-encoded with source forced to replay, at recorded pace / rate."""
    header, rows = read_fpgarec(path)
    name = os.path.basename(path)
    prev = None
    seq = 0
    for pc_ns, raw in rows:
        f = decode_one(raw)
        if f.kind not in KIND_NAME or f.kind in (KIND["HELLO_ACK"], KIND["PONG"], KIND["CMD_ACK"],
                                                 KIND["CMD_RESULT"], KIND["BYE"]):
            continue
        if kinds is not None and f.kind not in kinds:
            continue
        if prev is not None and rate > 0:
            delay = (pc_ns - prev) / 1e9 / rate
            if delay > 0:
                sleep(delay)
        prev = pc_ns
        obj = force_replay(f.kind, f.obj(), name)
        seq += 1
        if f.kind in BINARY_KINDS:
            out = decode_one(encode_binary(f.kind, seq, obj[0], obj[1]))
        else:
            out = decode_one(encode_json(f.kind, seq, obj))
        missing = check_source_flags(out.kind, out.obj())
        if missing is not None:
            continue                                # a recorded frame without marks stays refused
        yield out, out.obj()


class ReplayServer(object):
    """A fake Nano that serves a .fpgarec over FPGAC (HELLO_ACK data_source=replay)."""

    def __init__(self, path, token, host="127.0.0.1", port=0, rate=1.0, loop=True):
        self.path, self.token, self.rate, self.loop = path, token, rate, loop
        self.counts = collections.Counter()
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind((host, port))
        self.listener.listen(4)
        self.port = self.listener.getsockname()[1]
        self.boot_id = "replay-" + binascii.hexlify(os.urandom(4)).decode()
        self._stop = threading.Event()
        self.threads = []

    def start(self):
        t = threading.Thread(target=self._accept, name="replay-accept", daemon=True)
        t.start()
        self.threads.append(t)
        return self

    def _accept(self):
        self.listener.settimeout(0.2)
        while not self._stop.is_set():
            try:
                sock, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            t = threading.Thread(target=self._serve, args=(sock,), daemon=True)
            t.start()
            self.threads.append(t)

    def _serve(self, sock):
        conn = Connection(sock, self.counts)
        try:
            frame = conn.recv(3.0)
            if frame is None or frame.kind != KIND["HELLO"]:
                return
            hello = frame.obj()
            ok = hello.get("token") == self.token and frame.ver[0] == VERSION[0]
            conn.send(KIND["HELLO_ACK"], {"v": 1, "ok": ok, "reason": None if ok else "bad_token",
                                          "hub_boot_id": self.boot_id, "nano_mono_ns": time.monotonic_ns(),
                                          "proto": list(VERSION), "state_hz": 10, "data_source": "replay",
                                          "estop": None, "features": ["replay"], "controller": False})
            if not ok:
                return
            while not self._stop.is_set():
                for out, obj in replay_frames(self.path, self.rate, sleep=self._sleep):
                    if self._stop.is_set():
                        return
                    self._drain(conn)
                    conn.send(out.kind, body=out.body)
                if not self.loop:
                    break
            while not self._stop.is_set():              # keep answering PING/CMD after the end
                frame = conn.recv(0.2)
                if frame is not None:
                    conn.pending.appendleft(frame)
                    self._drain(conn)
        except (OSError, ProtocolError, ConnectionError):
            self.counts["client_gone"] += 1
        finally:
            conn.close()

    def _sleep(self, s):
        self._stop.wait(s)

    def _drain(self, conn):
        while True:
            try:
                frame = conn.recv(0)
            except (OSError, ProtocolError, ConnectionError):
                raise
            if frame is None:
                return
            if frame.kind == KIND["PING"]:
                conn.send(KIND["PONG"], frame.obj())
            elif frame.kind == KIND["CMD"]:
                body = frame.obj()
                conn.send(KIND["CMD_ACK"], {"v": 1, "cmd_id": body.get("cmd_id"), "status": "rejected",
                                            "reason": "replay"})

    def close(self):
        self._stop.set()
        self.listener.close()
        for t in self.threads:
            t.join(timeout=5)


# --------------------------------------------------------------------------------------------
# Hotspot diagnostics (C.8)
# --------------------------------------------------------------------------------------------

def local_ipv4():
    """This machine's outward IPv4 (a UDP connect sends no packet)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.0.2.1", 9))           # TEST-NET-1; no traffic is generated
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def ping_once(host, timeout_s=1.0):
    if platform.system().lower().startswith("win"):
        cmd = ["ping", "-n", "1", "-w", str(int(timeout_s * 1000)), host]
    else:
        cmd = ["ping", "-c", "1", "-W", str(max(1, int(round(timeout_s)))), host]
    try:
        r = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           timeout=timeout_s + 2)
        return r.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return None


def hello_check(host, port, token, timeout_s=0.3, expect_boot_id=None):
    """True only if a FPGAC hub answers HELLO with ok (and the expected boot id)."""
    try:
        sock = socket.create_connection((host, port), timeout=timeout_s)
    except (OSError, socket.timeout):
        return None
    conn = Connection(sock, collections.Counter(), send_timeout_s=max(timeout_s, 1.0))
    try:
        conn.send(KIND["HELLO"], {"v": 1, "client_id": "probe", "token": token, "want": [],
                                  "state_hz": 1, "probe": True})
        frame = conn.recv(3.0)
        if frame is None or frame.kind != KIND["HELLO_ACK"]:
            return False
        ack = frame.obj()
        try:
            conn.send(KIND["BYE"], {"v": 1, "reason": "probe"})
        except OSError:
            pass
        if not ack.get("ok"):
            return False
        return expect_boot_id is None or ack.get("hub_boot_id") == expect_boot_id
    except (OSError, ProtocolError, ConnectionError, ValueError):
        return False
    finally:
        conn.close()


def probe(host, port, token=None, ping=ping_once):
    """Separate 'hotspot isolates clients' from 'hub not running' (C.8)."""
    pinged = ping(host)
    try:
        sock = socket.create_connection((host, port), timeout=3.0)
        sock.close()
        tcp = "open"
    except ConnectionRefusedError:
        tcp = "refused"
    except (OSError, socket.timeout):
        tcp = "timeout"
    hello = hello_check(host, port, token, timeout_s=3.0) if (tcp == "open" and token) else None
    if tcp == "open":
        verdict = "hub_ok" if hello else ("hub_rejects_token_or_not_fpgac" if token else "tcp_open")
    elif tcp == "refused":
        verdict = "host_up_service_down"
    elif pinged:
        verdict = "host_up_port_filtered"
    else:
        verdict = "unreachable_isolation_or_other_subnet"
    return {"host": host, "port": port, "ping": pinged, "tcp": tcp, "hello": hello, "verdict": verdict}


def scan(port, token, net=None, timeout_s=0.3, workers=64, expect_boot_id=None):
    """Outbound-only /24 sweep; a host counts only if HELLO with this token succeeds."""
    if net is None:
        net = str(ipaddress.ip_network(local_ipv4() + "/24", strict=False))
    hosts = [str(h) for h in ipaddress.ip_network(net, strict=False).hosts()]
    found, lock = [], threading.Lock()
    todo = collections.deque(hosts)

    def worker():
        while True:
            with lock:
                if not todo:
                    return
                host = todo.popleft()
            if hello_check(host, port, token, timeout_s, expect_boot_id):
                with lock:
                    found.append(host)

    threads = [threading.Thread(target=worker, daemon=True) for _ in range(min(workers, len(hosts)))]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return sorted(found, key=lambda h: ipaddress.ip_address(h))
