"""Nano <-> KV260 real-time frame channel for the M7 frame core (B1, 2026-10-11).

``backend="remote_pl"``: the Nano keeps M7Pipeline / RollingFusion and sends
each frame's raw depth codes to the KV260, which converts them, runs the frame
core (PL, or the g++ CPU core, or a "null" core that answers at once without
any core) and returns the voxel flags. See docs/design/KV_LINK_20261011.md.

Rules (the reason this file exists rather than a socket in a script):

* Explicit and fail-closed. The backend never falls back to a local core. A
  timeout, a superseded notice, a depth-conversion mismatch or a malformed
  answer raises, so M7Pipeline marks that frame ``backend_failure:*`` and no
  previous grid is reused. Freshness is never faked: the capture identity goes
  on the wire and only an answer echoing all of it is accepted.
* Latest frame only, everywhere. Both ends reassemble only the newest frame
  (a chunk of a newer frame discards the unfinished older one); the KV260 keeps
  completed requests in a one-slot mailbox (a newer request overwrites and the
  overwritten one gets a short "superseded" notice). Window 1: the Nano sends
  the next request only after the previous answer or failure.
* Counted, not guessed. Application-level discards (superseded, stale answer,
  identity mismatch) and network-level loss (unfinished frames overtaken,
  chunks sent vs received, kernel UDP/IP counters from /proc/net/snmp) are
  kept apart.

Transport: UDP datagrams of ``chunk_bytes`` payload plus a 32-byte chunk
header. 60000 B chunks are IP-fragmented by the kernel (one lost fragment
loses the whole chunk); 1400 B chunks fit a 1500 B Ethernet MTU.
Python 3.8 compatible (the Nano's ROS-free fixture runs on 3.8 and 3.13).
"""

from collections import Counter, OrderedDict, deque, namedtuple
import json
import os
import socket
import struct
import threading
import time
import zlib

import numpy as np

from host.m7_frame_native import FrameNativeM7, GRID_CAPACITY, MAX_PIXELS
from host.fpga_clock import now_ns   # stamps that enter ages / cross boards (RAW)

MAGIC = b"KVL1"
CHUNK = struct.Struct("<4sBBHQQIHH")   # magic type flags rsv session frame_id total idx n
assert CHUNK.size == 32
REQ, RESP, SUPERSEDED, HELLO, HELLO_ACK, STATS, STATS_ACK = 1, 2, 3, 4, 5, 6, 7
SYNC, SYNC_ACK = 8, 9                   # clock mapping (B2): host/kv_clock_sync.py
# PC data plane package 1 (docs/design/PC_DATAPLANE_SPEC_20261011.md A): 10..13, 14..31 reserved
TELEM_REQ, TELEM_ACK, ESTOP_CMD, ESTOP_ACK = 10, 11, 12, 13
# 14 NUDGE (KV260 -> Nano only, empty body, one 74 B frame): makes the KV260 NIC transmit
# again after an answer so a frame its transmit ring kept back goes out now
# (docs/design/KV_LINK_BURST_TAIL_20261011.md). Receivers drop it; a client that predates
# it counts it as unexpected:14 and goes on.
NUDGE = 14
# 15/16 ROLL (v3 L2, 2026-10-26): rolling-map command / answer for the PL fusion core
# (host/kv_rolling.py); only a server built with a rolling service announces "rolling".
ROLL_REQ, ROLL_ACK = 15, 16
# 17..20 NAV (CTRLPS2 N2, 2026-11-03; docs CTRLPS0/DESIGN.md section 3): navigation on the KV260 PS.
# NAV_GOAL (goal / cancel), NAV_ODOM (50 Hz pose + permit) and NAV_PATH (G1 global route) go Nano -> KV;
# every one of them is answered with one NAV_CMD (KV -> Nano: v, w, state). The KV only answers, never
# pushes. Only a server started with --nav soft|pl announces "nav"; others count them unexpected:<type>.
NAV_GOAL, NAV_ODOM, NAV_CMD, NAV_PATH = 17, 18, 19, 20
TYPE_NAMES = {REQ: "req", RESP: "resp", SUPERSEDED: "superseded", HELLO: "hello",
              HELLO_ACK: "hello_ack", STATS: "stats", STATS_ACK: "stats_ack",
              SYNC: "sync", SYNC_ACK: "sync_ack", TELEM_REQ: "telem_req",
              TELEM_ACK: "telem_ack", ESTOP_CMD: "estop_cmd", ESTOP_ACK: "estop_ack",
              NUDGE: "nudge", ROLL_REQ: "roll_req", ROLL_ACK: "roll_ack",
              NAV_GOAL: "nav_goal", NAV_ODOM: "nav_odom", NAV_CMD: "nav_cmd", NAV_PATH: "nav_path"}
FLAG_RESET = 1                          # STATS: reset the server's measurement window
RESP_FLAG_ROI = 2                       # RESP chunk-header flag: ROI result block follows the flags
EXT_ROI = 1                             # REQ ext_flags bit0: ROI block follows the depth codes
EXT_AGE = 2                             # REQ ext_flags bit1: AGE block is the last 20 bytes (car AR)
EXT_LABEL = 4                           # REQ ext_flags bit2: LABEL block after the ROI block (car AR)
EXT_REPLAY = 8                          # REQ ext_flags bit3: no bytes; this frame's depth is a replay (car AR)
EXT_STAMP = 16                          # REQ ext_flags bit4: STAMP block (sensor key) before the AGE block (AR colour)
EXT_VER = 1                             # ``ver`` of every extension block / new message body
FEATURES = {"roi_ext": 1, "telem": 1, "estop": 1, "sync": 1}
ROI_MAX_BOXES = 32                      # = FPGA_M7_ROI_MAX_BOXES = host.m7_object_native.ROI_MAX_BOXES
CHUNK_FRAGMENTED = 60000                # one datagram, IP fragments on a 1500 B MTU
CHUNK_MTU = 1400                        # 1400 + 32 + 8 UDP + 20 IP = 1460 <= 1500
MAX_DATAGRAM = 65507
# Control answers (HELLO/STATS/SYNC/TELEM/ESTOP ACKs) go out in MTU-sized chunks: each chunk
# is one Ethernet frame, never IP-fragmented. HELLO_ACK carries only HELLO_INFO_KEYS (one
# frame); the full server info stays in STATS_ACK["info"]. docs/design/KV_LINK_HELLO_FIX_20261011.md
CONTROL_CHUNK = CHUNK_MTU
HELLO_INFO_KEYS = ("core", "chunk_bytes", "rcvbuf", "features", "roi_core", "roi_max_boxes",
                   "kv_boot_id", "pid", "started_unix", "python", "numpy")
HELLO_RESEND_S = 0.05                   # first HELLO resend gap; doubles until the deadline
TELEM_RESEND_S = 0.05                   # first TELEM_REQ resend gap (TELEM is read-only)
# KvLinkServer: how long one sendto may block on a full send buffer before that message is dropped
# (counted send_timeout). Before 2026-10-16 the reader's 0.2 s receive poll timeout bounded every send
# on the shared socket and a timeout killed the worker (kv_server exit "worker error").
SERVER_SEND_TIMEOUT_S = 1.0
SERVER_RECV_POLL_S = 0.2
POKE_S = 0.1                            # frame call: HELLO "poke" after this long without answer
NUDGE_IDLE_S = 0.002                    # server: trailing NUDGE after this long without a send
MAX_MESSAGE = 8 << 20

# Request body after the chunk header (little endian):
#   frame_id Q, capture_ns Q, pose_ns Q, width I, height I, nx I, ny I, nz I,
#   depth unit as float32 bits I, crc32 of the uint16 codes I, crc32 of the
#   Nano's float32 depth I, reserved I, voxel d
#   then origin 3 x f64, inverse_k 9 x f32, world_camera 12 x f32,
#   four strings (u16 length + utf-8): stream_instance, clock_id,
#   calibration_id, pose_epoch; then width*height uint16 codes.
REQ_HEAD = struct.Struct("<QQQIIIIIIIIId")
# Response body: frame_id Q, capture_ns Q, pose_ns Q, status I, counts 3I,
#   crc32 of the KV260's float32 depth I, nvox I, then 9 x u64 KV260 stage
#   durations in ns (TIMING_FIELDS), then u32 kv slot superseded total, u32 kv
#   unfinished-overtaken total, the four echoed strings, the core name string,
#   then nvox uint8 flags.
RESP_HEAD = struct.Struct("<QQQIIIIII")
TIMING_FIELDS = ("recv", "slot_wait", "decode", "convert", "crc", "core_prep",
                 "core_device", "core_readback", "core_total")
RESP_TIMES = struct.Struct("<%dQ" % len(TIMING_FIELDS))
# KV260 time spent on a request after it is fully received (its own clock).
KV_SERVICE_FIELDS = ("slot_wait", "decode", "convert", "crc", "core_total")
RESP_TAIL = struct.Struct("<II")
NOTICE = struct.Struct("<QQ")           # superseded: frame_id, capture_ns of the dropped request
IDENTITY_FIELDS = ("stream_instance", "clock_id", "calibration_id", "pose_epoch")


class LinkError(RuntimeError):
    """A remote frame failed (timeout, superseded, mismatch). The frame is invalid."""


class LinkTimeout(LinkError):
    pass


class LinkSuperseded(LinkError):
    pass


class KvFeatureUnavailable(LinkError):
    """The KV260 server did not announce the feature (old server): nothing was sent."""


class BadExtension(ValueError):
    """An extension block / new message body failed its ver, length, crc or range check.
    The whole message is void (counted ``bad_ext:<reason>``), never partly used."""

    def __init__(self, reason, detail=""):
        super().__init__("bad_ext:%s%s" % (reason, (" (%s)" % detail) if detail else ""))
        self.reason = reason


def f32_bits(value):
    return struct.unpack("<I", struct.pack("<f", float(value)))[0]


def bits_f32(bits):
    return np.frombuffer(struct.pack("<I", int(bits)), dtype="<f4")[0]


def codes_to_m(codes, unit_f32):
    """The reference rule (tools/geo_same_input/common.depth_u16_to_m):
    float32(code) * float32(unit), code 0 -> NaN."""
    codes = np.asarray(codes)
    out = codes.astype(np.float32) * np.float32(unit_f32)
    out[codes == 0] = np.nan
    return out


def m_to_codes(depth_f32, unit_f32):
    """Inverse of ``codes_to_m`` that refuses anything it cannot reproduce bit for bit.

    Returns uint16 codes such that ``codes_to_m(codes, unit)`` has exactly the
    bits of ``depth_f32`` (NaN <-> 0). Raises ValueError otherwise: the channel
    never quantizes silently.
    """
    d = np.ascontiguousarray(depth_f32, dtype=np.float32)
    nan = np.isnan(d)
    with np.errstate(invalid="ignore", over="ignore"):
        q = np.rint(d.astype(np.float64) / float(np.float32(unit_f32)))
    q[nan] = 0
    if not np.isfinite(q).all() or (q < 0).any() or (q > 65535).any():
        raise ValueError("depth outside the uint16 code range of unit %r" % float(unit_f32))
    if ((q == 0) & ~nan).any():
        raise ValueError("depth 0 m is not representable (code 0 means no return)")
    codes = q.astype(np.uint16)
    back = codes_to_m(codes, unit_f32)
    same_nan = np.array_equal(np.isnan(back), nan)
    if not same_nan or not np.array_equal(back.view(np.uint32)[~nan], d.view(np.uint32)[~nan]):
        raise ValueError("depth is not on the float32 grid of unit %r (would be requantized)"
                         % float(unit_f32))
    return codes


def _d16_unit_ok(unit_bits):
    from host.m7_backend import d16_unit_ok
    return d16_unit_ok(unit_bits)


def crc32(array):
    return zlib.crc32(memoryview(np.ascontiguousarray(array)).cast("B")) & 0xFFFFFFFF


def _pack_str(text):
    raw = text.encode("utf-8")
    if len(raw) > 65535:
        raise ValueError("identity string too long")
    return struct.pack("<H", len(raw)) + raw


def _unpack_str(buf, off):
    (n,) = struct.unpack_from("<H", buf, off)
    off += 2
    if off + n > len(buf):
        raise ValueError("truncated string")
    return bytes(buf[off:off + n]).decode("utf-8"), off + n


IP_MTU = 1500                           # Ethernet MTU of the Nano <-> KV260 link (not changed by us)


def wire_frames(datagram_len, mtu=IP_MTU):
    """Ethernet frames one UDP datagram of ``datagram_len`` payload bytes becomes
    (IPv4, 20 B header, fragments carry multiples of 8 B)."""
    ip_payload = int(datagram_len) + 8
    per = (mtu - 20) // 8 * 8
    return 1 if ip_payload <= mtu - 20 else -(-ip_payload // per)


def split(msg_type, session, frame_id, payload, chunk_bytes, flags=0):
    """Datagrams of one message; every chunk carries the full header."""
    payload = memoryview(payload).cast("B") if not isinstance(payload, (bytes, bytearray)) \
        else memoryview(payload)
    total = len(payload)
    if total > MAX_MESSAGE:
        raise ValueError("message of %d bytes exceeds %d" % (total, MAX_MESSAGE))
    chunk_bytes = int(chunk_bytes)
    if not 1 <= chunk_bytes <= MAX_DATAGRAM - CHUNK.size:
        raise ValueError("chunk_bytes must be in [1, %d]" % (MAX_DATAGRAM - CHUNK.size))
    n = max(1, -(-total // chunk_bytes))
    if n > 65535:
        raise ValueError("too many chunks")
    out = []
    for i in range(n):
        head = CHUNK.pack(MAGIC, msg_type, flags, 0, session, frame_id, total, i, n)
        out.append(head + bytes(payload[i * chunk_bytes:(i + 1) * chunk_bytes]))
    return out


class Reassembler:
    """Newest-frame-only reassembly per message type.

    Counters (prefix ``rx_``): chunks, malformed, duplicate_chunks,
    old_frame_chunks (chunk of a frame older than the one in progress or
    already completed), incomplete_superseded (an unfinished frame overtaken
    by a newer one = network loss of at least one chunk, the count of missing
    chunks in incomplete_missing_chunks), session_switch, completed.
    """

    def __init__(self, counts, prefix="rx_"):
        self.counts, self.p = counts, prefix
        self._state = {}       # msg_type -> dict(session, frame_id, n, total, parts, first_t)
        self._done = {}        # msg_type -> (session, last completed frame_id)

    def _c(self, name, n=1):
        self.counts[self.p + name] += n

    def forget_done(self, mtype):
        """Let a rejected message's frame id be completed again (a forged or
        corrupt answer must not block the genuine one)."""
        self._done.pop(mtype, None)

    def pending(self):
        return {TYPE_NAMES.get(t, t): (st["frame_id"], len(st["parts"]), st["n"])
                for t, st in self._state.items()}

    def feed(self, datagram, now=None):
        """Returns (msg_type, flags, session, frame_id, payload bytes, first_chunk_time) or None."""
        now = time.monotonic() if now is None else now
        self._c("chunks")
        if len(datagram) < CHUNK.size:
            self._c("malformed")
            return None
        magic, mtype, flags, _, session, frame_id, total, idx, n = CHUNK.unpack_from(datagram)
        body = datagram[CHUNK.size:]
        if magic != MAGIC or n == 0 or idx >= n or total > MAX_MESSAGE:
            self._c("malformed")
            return None
        done = self._done.get(mtype)
        if done is not None and done[0] == session and frame_id <= done[1]:
            self._c("old_frame_chunks")
            return None
        st = self._state.get(mtype)
        if st is not None and st["session"] != session:
            self._c("session_switch")
            if st["parts"]:
                self._c("incomplete_superseded")
                self._c("incomplete_missing_chunks", st["n"] - len(st["parts"]))
            st = None
        if st is not None and frame_id < st["frame_id"]:
            self._c("old_frame_chunks")
            return None
        if st is not None and frame_id > st["frame_id"]:
            self._c("incomplete_superseded")
            self._c("incomplete_missing_chunks", st["n"] - len(st["parts"]))
            st = None
        if st is None:
            st = {"session": session, "frame_id": frame_id, "n": n, "total": total,
                  "parts": {}, "first_t": now}
            self._state[mtype] = st
        elif st["n"] != n or st["total"] != total:
            self._c("malformed")
            return None
        if idx in st["parts"]:
            self._c("duplicate_chunks")
            return None
        st["parts"][idx] = bytes(body)
        if len(st["parts"]) < n:
            return None
        payload = b"".join(st["parts"][i] for i in range(n))
        del self._state[mtype]
        self._done[mtype] = (session, frame_id)
        if len(payload) != total:
            self._c("malformed")
            return None
        self._c("completed")
        return mtype, flags, session, frame_id, payload, st["first_t"]


class LatestSlot:
    """Capacity-1 mailbox: ``put`` overwrites (and returns) the unconsumed item.
    ``put_side`` (ROLL, 2026-10-26) queues an item that is never superseded; ``take``
    returns queued side items first, in order, then the latest item."""

    def __init__(self):
        self._cond = threading.Condition()
        self._item = None
        self._side = deque()
        self._closed = False
        self.puts = self.superseded = self.takes = 0

    def put(self, item):
        with self._cond:
            old, self._item = self._item, item
            self.puts += 1
            if old is not None:
                self.superseded += 1
            self._cond.notify_all()
            return old

    def put_side(self, item):
        with self._cond:
            self._side.append(item)
            self._cond.notify_all()

    def take(self, timeout=None):
        with self._cond:
            end = None if timeout is None else time.monotonic() + timeout
            while self._item is None and not self._side and not self._closed:
                left = None if end is None else end - time.monotonic()
                if left is not None and left <= 0:
                    return None
                self._cond.wait(left)
            if self._side:
                return self._side.popleft()
            item, self._item = self._item, None
            if item is not None:
                self.takes += 1
            return item

    def close(self):
        with self._cond:
            self._closed = True
            self._cond.notify_all()


def read_snmp(path="/proc/net/snmp"):
    """{'Ip': {...}, 'Udp': {...}} integer counters, or {'error': ...}."""
    try:
        lines = open(path).read().split("\n")
    except OSError as exc:
        return {"error": str(exc)}
    out = {}
    for head, vals in zip(lines[0::2], lines[1::2]):
        if not head or ":" not in head:
            continue
        name, keys = head.split(":", 1)
        _, values = vals.split(":", 1)
        if name in ("Ip", "Udp"):
            out[name] = dict(zip(keys.split(), (int(v) for v in values.split())))
    return out


SNMP_KEYS = (("Ip", "ReasmReqds"), ("Ip", "ReasmOKs"), ("Ip", "ReasmFails"),
             ("Ip", "ReasmTimeout"), ("Udp", "InDatagrams"), ("Udp", "InErrors"),
             ("Udp", "RcvbufErrors"), ("Udp", "InCsumErrors"), ("Udp", "OutDatagrams"),
             ("Udp", "SndbufErrors"))


def snmp_delta(before, after):
    out = {}
    for group, key in SNMP_KEYS:
        a = before.get(group, {}).get(key)
        b = after.get(group, {}).get(key)
        if a is not None and b is not None:
            out["%s.%s" % (group, key)] = b - a
    return out


def set_rcvbuf(sock, nbytes):
    """SO_RCVBUFFORCE when allowed (root), else SO_RCVBUF; returns (method, effective bytes)."""
    force = getattr(socket, "SO_RCVBUFFORCE", 33)
    method = "SO_RCVBUFFORCE"
    try:
        sock.setsockopt(socket.SOL_SOCKET, force, int(nbytes))
    except OSError:
        method = "SO_RCVBUF(capped by rmem_max)"
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, int(nbytes))
    return method, sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)


# ------------------------------------------------------------------ extension blocks (A.2-A.4)
# Every block: ver u8 first, crc32 of all preceding block bytes last, little endian, no padding.
ROI_REQ_HEAD = struct.Struct("<BBHdIIddQ")      # ver rsv box_count margin min_pixels rsv2 min_valid free revision
ROI_REQ_BOX = struct.Struct("<IIIIIIdd")        # row_id enabled x0 y0 x1 y1 near_z far_z
ROI_RESP_HEAD = struct.Struct("<BBHIQ4QQ")      # ver roi_core rows roi_status revision t_prep t_device t_readback t_total roi_cycles
ROI_RESP_ROW = struct.Struct("<7I")
CRC = struct.Struct("<I")
# Car AR (2026-10-11, docs/design/CAR_AR_RESULTS_20261011.md): the Nano's own elapsed time
# from capture to the first sendto of this request (one clock, the Nano's CLOCK_MONOTONIC_RAW).
# Sent only to a server that announces feature "age_ext" (kv_server --ar drm). It is never
# combined with any clock mapping; the KV adds its own first-chunk->now duration.
AGE_REQ = struct.Struct("<B3sQ")                 # ver rsv nano_elapsed_ns
AGE_BLOCK_BYTES = AGE_REQ.size + 4
# Car AR box labels (controller 10-11): one class name per ROI request row, same order,
# <= 8 printable ASCII bytes NUL-padded (all NUL = no name: the display falls back to R+row_id).
# Only with the ROI block, placed right after it (before the AGE block, which stays last); sent
# only to a server that announces feature "label_ext" (kv_server --ar drm). Display only.
# Replay mark (owner 10-12, car AR): ext_flags bit3 carries no bytes; set by the Nano when the
# frame's depth comes from a recording (replay camera stand-in, nano_replay), so the AR panel
# says REPLAY instead of LIVE. Sent only to a server that announces feature "replay_ext"
# (kv_server --ar drm); any other server voids such a request (unknown bit -> BadExtension).
LABEL_HEAD = struct.Struct("<BBH")               # ver n rsv
# AR colour bypass (2026-10-16, host/kv_colour.py): the depth image's sensor key (live: the camera
# header stamp in ns; replay: the recording's frame key), so the KV can ask the Nano's colour source
# for the colour frame of exactly this depth frame. 16 bytes, placed right before the AGE block (AGE
# stays last). Sent only to a server that announces feature "stamp_ext" (kv_server --ar drm with the
# colour bypass); any other server voids such a request (unknown bit -> BadExtension).
STAMP_REQ = struct.Struct("<B3sQ")               # ver rsv sensor_key
STAMP_BLOCK_BYTES = STAMP_REQ.size + 4
LABEL_BYTES = 8
assert (ROI_REQ_HEAD.size, ROI_REQ_BOX.size, ROI_RESP_HEAD.size) == (44, 40, 56)
ROI_CORE_CODES = {"none": 0, "cpu": 1, "pl": 2}
ROI_CORE_NAMES = {v: k for k, v in ROI_CORE_CODES.items()}
ROI_OK, ROI_ERR_SIZE, ROI_ERR_BOXES, ROI_ERR_THRESHOLDS = 0, 1, 3, 4
ROI_NOT_CONFIGURED, ROI_GEOMETRY_FAILED, ROI_DEVICE_ERROR = 0x100, 0x101, 0x102
ROI_TIME_FIELDS = ("prep", "device", "readback", "total")


def seal(body):
    return body + CRC.pack(zlib.crc32(body) & 0xFFFFFFFF)


def unseal(buf, expected_len=None):
    """Body without its crc32; BadExtension on length or crc."""
    buf = bytes(buf)
    if len(buf) < CRC.size + 1 or (expected_len is not None and len(buf) != expected_len):
        raise BadExtension("length", "%d bytes" % len(buf))
    body, (crc,) = buf[:-CRC.size], CRC.unpack_from(buf, len(buf) - CRC.size)
    if zlib.crc32(body) & 0xFFFFFFFF != crc:
        raise BadExtension("crc")
    return body


def encode_roi_block(batch):
    """REQ ROI block of a host.m7_object_native.RoiBatch (rows already filtered:
    the remote path sends enabled rows only, A.2)."""
    rows = list(batch.rows)
    if not 1 <= len(rows) <= ROI_MAX_BOXES:
        raise ValueError("ROI batch must hold 1..%d rows" % ROI_MAX_BOXES)
    cfg = batch.cfg
    out = [ROI_REQ_HEAD.pack(EXT_VER, 0, len(rows), float(cfg.depth_margin_m), int(cfg.min_pixels), 0,
                             float(cfg.min_valid_fraction), float(cfg.free_fraction),
                             int(batch.memory_revision))]
    for r in rows:
        xyxy = r.xyxy if (r.enabled and r.xyxy is not None) else (0, 0, 0, 0)
        out.append(ROI_REQ_BOX.pack(int(r.row_id), 1 if r.enabled else 0,
                                    *(int(v) for v in xyxy), float(r.near_z), float(r.far_z)))
    return seal(b"".join(out))


def decode_roi_block(buf):
    if len(buf) < ROI_REQ_HEAD.size + CRC.size:
        raise BadExtension("length", "%d bytes" % len(buf))
    (ver, rsv, n, margin, min_pixels, rsv2, min_valid, free, revision) = ROI_REQ_HEAD.unpack_from(buf)
    if ver != EXT_VER:
        raise BadExtension("ver", str(ver))
    if not 1 <= n <= ROI_MAX_BOXES:
        raise BadExtension("box_count", str(n))
    body = unseal(buf, ROI_REQ_HEAD.size + n * ROI_REQ_BOX.size + CRC.size)
    if rsv or rsv2:
        raise BadExtension("rsv")
    rows = []
    for i in range(n):
        row = ROI_REQ_BOX.unpack_from(body, ROI_REQ_HEAD.size + i * ROI_REQ_BOX.size)
        if row[1] not in (0, 1):
            raise BadExtension("enabled", str(row[1]))
        rows.append(row)
    return {"memory_revision": revision,
            "cfg": {"depth_margin_m": margin, "min_pixels": min_pixels,
                    "min_valid_fraction": min_valid, "free_fraction": free},
            "rows": rows}


def encode_age_block(nano_elapsed_ns):
    v = int(nano_elapsed_ns)
    if not 0 <= v < (1 << 63):
        raise ValueError("nano_elapsed_ns out of range")
    return seal(AGE_REQ.pack(EXT_VER, b"\0\0\0", v))


def decode_age_block(buf):
    body = unseal(buf, AGE_BLOCK_BYTES)
    ver, rsv, v = AGE_REQ.unpack_from(body)
    if ver != EXT_VER:
        raise BadExtension("age_ver", str(ver))
    if rsv != b"\0\0\0":
        raise BadExtension("age_rsv")
    if v >= (1 << 63):
        raise BadExtension("age_value")
    return v


def encode_stamp_block(sensor_key):
    v = int(sensor_key)
    if not 0 <= v <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("sensor key out of u64 range")
    return seal(STAMP_REQ.pack(EXT_VER, b"\0\0\0", v))


def decode_stamp_block(buf):
    body = unseal(buf, STAMP_BLOCK_BYTES)
    ver, rsv, v = STAMP_REQ.unpack_from(body)
    if ver != EXT_VER:
        raise BadExtension("stamp_ver", str(ver))
    if rsv != b"\0\0\0":
        raise BadExtension("stamp_rsv")
    return v


def encode_label_block(labels):
    """``labels``: one entry per ROI request row (str/bytes/None). Each must already be
    printable ASCII <= 8 bytes (clean_wire_label makes it so); None/"" = no name."""
    labels = list(labels)
    if not 1 <= len(labels) <= ROI_MAX_BOXES:
        raise ValueError("LABEL block must hold 1..%d entries" % ROI_MAX_BOXES)
    out = [LABEL_HEAD.pack(EXT_VER, len(labels), 0)]
    for v in labels:
        b = (v.encode("ascii") if isinstance(v, str) else bytes(v or b""))
        if len(b) > LABEL_BYTES or any(not 0x20 <= c <= 0x7E for c in b):
            raise ValueError("label %r is not <= %d printable ASCII bytes" % (v, LABEL_BYTES))
        out.append(b.ljust(LABEL_BYTES, b"\0"))
    return seal(b"".join(out))


def decode_label_block(buf, rows):
    """-> list of ``rows`` bytes objects (b"" = no name). BadExtension on any defect."""
    if len(buf) < LABEL_HEAD.size + CRC.size:
        raise BadExtension("label_length", "%d bytes" % len(buf))
    ver, n, rsv = LABEL_HEAD.unpack_from(buf)
    if ver != EXT_VER:
        raise BadExtension("label_ver", str(ver))
    if n != rows:
        raise BadExtension("label_count", "%d labels for %d ROI rows" % (n, rows))
    body = unseal(buf, LABEL_HEAD.size + n * LABEL_BYTES + CRC.size)
    if rsv:
        raise BadExtension("label_rsv")
    out = []
    for i in range(n):
        e = body[LABEL_HEAD.size + i * LABEL_BYTES: LABEL_HEAD.size + (i + 1) * LABEL_BYTES]
        name = e.split(b"\0", 1)[0]
        if e[len(name):].strip(b"\0") or any(not 0x20 <= c <= 0x7E for c in name):
            raise BadExtension("label_char", repr(e))
        out.append(name)
    return out


def clean_wire_label(name):
    """Nano side: a class name -> what goes on the wire (bytes, <= 8). Non-ASCII names (e.g.
    Chinese) or empty -> b"" (the display then shows R+row_id); longer ones are cut to 8."""
    if name is None:
        return b"", "missing"
    try:
        b = str(name).strip().encode("ascii")
    except UnicodeEncodeError:
        return b"", "non_ascii"
    if any(not 0x20 <= c <= 0x7E for c in b):
        return b"", "non_printable"
    if not b:
        return b"", "missing"
    if len(b) > LABEL_BYTES:
        return b[:LABEL_BYTES], "cut_to_8"
    return b, None


def encode_roi_result(*, roi_core, roi_status, memory_revision, times_ns, roi_cycles, rows):
    rows = list(rows) if roi_status == ROI_OK else []
    head = ROI_RESP_HEAD.pack(EXT_VER, ROI_CORE_CODES[roi_core], len(rows), int(roi_status),
                              int(memory_revision),
                              *(max(0, int(times_ns.get(k, 0))) for k in ROI_TIME_FIELDS),
                              int(roi_cycles or 0))
    return seal(head + b"".join(ROI_RESP_ROW.pack(*(int(v) for v in r)) for r in rows))


def decode_roi_result(buf):
    if len(buf) < ROI_RESP_HEAD.size + CRC.size:
        raise BadExtension("length", "%d bytes" % len(buf))
    head = ROI_RESP_HEAD.unpack_from(buf)
    ver, core, rows, status, revision = head[:5]
    if ver != EXT_VER:
        raise BadExtension("ver", str(ver))
    if core not in ROI_CORE_NAMES:
        raise BadExtension("roi_core", str(core))
    if rows > ROI_MAX_BOXES or (status != ROI_OK and rows):
        raise BadExtension("rows", str(rows))
    body = unseal(buf, ROI_RESP_HEAD.size + rows * ROI_RESP_ROW.size + CRC.size)
    return {"roi_core": ROI_CORE_NAMES[core], "roi_status": status, "memory_revision": revision,
            "times_ns": dict(zip(ROI_TIME_FIELDS, head[5:9])), "roi_cycles": head[9] or None,
            "rows": [ROI_RESP_ROW.unpack_from(body, ROI_RESP_HEAD.size + i * ROI_RESP_ROW.size)
                     for i in range(rows)]}


# TELEM / ESTOP bodies (A.3 / A.4)
TELEM_BLOCKS = ("geometry", "roi", "permit", "encoder", "ar", "estop", "proc", "board")
TELEM_ALL = (1 << len(TELEM_BLOCKS)) - 1
TELEM_REQ_BODY = struct.Struct("<B3sIQ")        # ver rsv want t_req_nano_ns
TELEM_ACK_HEAD = struct.Struct("<B3sIQQI")      # ver rsv avail kv_mono_ns t_req_nano_ns json_len
ESTOP_CMD_BODY = struct.Struct("<BBBBQQQ")      # ver op source rsv cmd_id epoch t_nano_ns
ESTOP_ACK_BODY = struct.Struct("<BBBBQBBBBQIIQQ")  # ver op result rsv cmd_id latched fsw permit_avail rsv epoch permit_status rsv kv_mono_ns kv_boot_id
assert (TELEM_REQ_BODY.size, ESTOP_CMD_BODY.size, ESTOP_ACK_BODY.size) == (16, 28, 48)
ESTOP_ASSERT, ESTOP_RELEASE, ESTOP_QUERY = 1, 2, 3
ESTOP_OPS = {"assert": ESTOP_ASSERT, "release": ESTOP_RELEASE, "query": ESTOP_QUERY}
ESTOP_SOURCE_PC = 1
(ESTOP_OK, ESTOP_ALREADY, ESTOP_EPOCH_MISMATCH, ESTOP_NOT_LATCHED, ESTOP_INVALID,
 ESTOP_LATCHED_NO_PERMIT, ESTOP_WRITE_UNVERIFIED) = range(7)


def mask_blocks(mask):
    return {name for i, name in enumerate(TELEM_BLOCKS) if mask >> i & 1}


def blocks_mask(names):
    return sum(1 << TELEM_BLOCKS.index(n) for n in names)


def encode_telem_req(want, t_req_nano_ns):
    return seal(TELEM_REQ_BODY.pack(EXT_VER, b"\0\0\0", int(want), int(t_req_nano_ns)))


def decode_telem_req(buf):
    body = unseal(buf, TELEM_REQ_BODY.size + CRC.size)
    ver, rsv, want, t_req = TELEM_REQ_BODY.unpack(body)
    if ver != EXT_VER:
        raise BadExtension("ver", str(ver))
    if rsv != b"\0\0\0":
        raise BadExtension("rsv")
    if want & ~TELEM_ALL or not want:
        raise BadExtension("want", hex(want))
    return {"want": want, "t_req_nano_ns": t_req}


def encode_telem_ack(avail, kv_mono_ns, t_req_nano_ns, doc):
    raw = json.dumps(doc, sort_keys=True, allow_nan=False).encode("utf-8")
    return seal(TELEM_ACK_HEAD.pack(EXT_VER, b"\0\0\0", int(avail), int(kv_mono_ns),
                                    int(t_req_nano_ns), len(raw)) + raw)


def decode_telem_ack(buf):
    if len(buf) < TELEM_ACK_HEAD.size + CRC.size:
        raise BadExtension("length", "%d bytes" % len(buf))
    ver, rsv, avail, kv_mono, t_req, n = TELEM_ACK_HEAD.unpack_from(buf)
    if ver != EXT_VER:
        raise BadExtension("ver", str(ver))
    body = unseal(buf, TELEM_ACK_HEAD.size + n + CRC.size)
    if rsv != b"\0\0\0":
        raise BadExtension("rsv")
    if avail & ~TELEM_ALL:
        raise BadExtension("avail", hex(avail))
    try:
        doc = json.loads(body[TELEM_ACK_HEAD.size:].decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as exc:
        raise BadExtension("json", str(exc))
    return {"avail": avail, "kv_mono_ns": kv_mono, "t_req_nano_ns": t_req, "doc": doc}


def encode_estop_cmd(op, cmd_id, epoch=0, t_nano_ns=0, source=ESTOP_SOURCE_PC):
    return seal(ESTOP_CMD_BODY.pack(EXT_VER, int(op), int(source), 0, int(cmd_id), int(epoch),
                                    int(t_nano_ns)))


def decode_estop_cmd(buf):
    body = unseal(buf, ESTOP_CMD_BODY.size + CRC.size)
    ver, op, source, rsv, cmd_id, epoch, t_nano = ESTOP_CMD_BODY.unpack(body)
    if ver != EXT_VER:
        raise BadExtension("ver", str(ver))
    if op not in (ESTOP_ASSERT, ESTOP_RELEASE, ESTOP_QUERY):
        raise BadExtension("op", str(op))
    if source != ESTOP_SOURCE_PC:
        raise BadExtension("source", str(source))
    if rsv:
        raise BadExtension("rsv")
    return {"op": op, "source": source, "cmd_id": cmd_id, "epoch": epoch, "t_nano_ns": t_nano}


def encode_estop_ack(*, op, result, cmd_id, latched, force_stop_written, permit_avail, epoch,
                     permit_status, kv_mono_ns, kv_boot_id):
    return seal(ESTOP_ACK_BODY.pack(EXT_VER, int(op), int(result), 0, int(cmd_id), int(bool(latched)),
                                    int(bool(force_stop_written)), int(bool(permit_avail)), 0,
                                    int(epoch), int(permit_status) & 0xFFFFFFFF, 0, int(kv_mono_ns),
                                    int(kv_boot_id)))


def decode_estop_ack(buf):
    body = unseal(buf, ESTOP_ACK_BODY.size + CRC.size)
    (ver, op, result, rsv, cmd_id, latched, fsw, pavail, rsv2, epoch, pstatus, rsv3, kv_mono,
     boot) = ESTOP_ACK_BODY.unpack(body)
    if ver != EXT_VER:
        raise BadExtension("ver", str(ver))
    if op not in (ESTOP_ASSERT, ESTOP_RELEASE, ESTOP_QUERY) or result > ESTOP_WRITE_UNVERIFIED:
        raise BadExtension("enum", "op %d result %d" % (op, result))
    if rsv or rsv2 or rsv3 or latched > 1 or fsw > 1 or pavail > 1:
        raise BadExtension("rsv")
    return {"op": op, "result": result, "cmd_id": cmd_id, "latched": bool(latched),
            "force_stop_written": bool(fsw), "permit_avail": bool(pavail), "epoch": epoch,
            "permit_status": pstatus, "kv_mono_ns": kv_mono, "kv_boot_id": "%016x" % boot}


# NAV bodies (CTRLPS2 N2, 2026-11-03). Little endian, sealed (crc32 last), ``ver`` = EXT_VER.
# NAV_GOAL: ver op rsv goal_id t_nano_ns x y yaw (map frame, m / rad); op CANCEL ignores x y yaw (0).
NAV_GOAL_BODY = struct.Struct("<BB2sQQddd")
# NAV_ODOM: ver flags rsv seq t_nano_ns odom_stamp_ns, odom->base_link x y yaw (f64), v w (f32),
#   map->odom x y yaw (f64). flags: NAV_ODOM_* below.
NAV_ODOM_BODY = struct.Struct("<BB2sIQQdddffddd")
# NAV_CMD: ver state reason flags goal_id cmd_seq odom_seq kv_mono_ns cmd_age_us v w remaining_m
#   path_len_m recoveries path_points. ``odom_seq`` echoes the NAV_ODOM answered (0 for GOAL/PATH).
NAV_CMD_BODY = struct.Struct("<BBBBQIIQIffffHH")
# NAV_PATH: ver rsv goal_id path_id t_nano_ns n rsv, then n x (x f32, y f32), map frame.
NAV_PATH_HEAD = struct.Struct("<B3sQIQH2s")
NAV_PATH_POINT = struct.Struct("<ff")
assert (NAV_GOAL_BODY.size, NAV_ODOM_BODY.size, NAV_CMD_BODY.size, NAV_PATH_HEAD.size) == (44, 80, 52, 28)
NAV_PATH_MAX_POINTS = 1024
NAV_OP_GOAL, NAV_OP_CANCEL = 1, 2
NAV_ODOM_PERMIT, NAV_ODOM_TELEOP, NAV_ODOM_MAP_VALID, NAV_ODOM_ODOM_VALID = 1, 2, 4, 8
# CTRLPS6 (appended): the Nano had no NAV_CMD for > its hold time while a goal runs (link lost): the core holds
# (PAUSED, no stuck time) on this odometry and replans from the current pose on the next one without it
NAV_ODOM_HOLD = 16
NAV_ODOM_FLAGS = 31
# kv_nav state machine (DESIGN 2.6) and abort reasons (DESIGN 3 "结果")
NAV_STATES = ("idle", "plan", "rotate_to_path", "track", "final_rotate", "recover",
              "succeeded", "aborted", "canceled",
              "paused")             # CTRLPS3 (appended): permit off / teleop in hand, zero command, no stuck time
NAV_STATE = {name: i for i, name in enumerate(NAV_STATES)}
NAV_TERMINAL = (NAV_STATE["succeeded"], NAV_STATE["aborted"], NAV_STATE["canceled"])
NAV_REASONS = ("none", "plan_failed", "blocked", "stuck", "soft_field_failed", "link_lost",
               "goal_rejected", "no_odom", "preempted",
               "no_map", "detour")   # CTRLPS3 (appended): no fresh costmap window; detour observe (DESIGN 2.6)
NAV_REASON = {name: i for i, name in enumerate(NAV_REASONS)}
# NAV_CMD flags: bit0 the goal of goal_id is accepted, bit1 (v, w) may be driven (fresh odom, a goal
# running, not stopped), bit2 the KV wants a new global route for goal_id now (no route / dead end)
NAV_CMD_ACCEPTED, NAV_CMD_VALID, NAV_CMD_NEED_PATH = 1, 2, 4
NAV_CMD_FLAGS = 7


def _finite(*values):
    import math
    return all(math.isfinite(v) for v in values)


def encode_nav_goal(op, goal_id, t_nano_ns=0, x=0.0, y=0.0, yaw=0.0):
    return seal(NAV_GOAL_BODY.pack(EXT_VER, int(op), b"\0\0", int(goal_id), int(t_nano_ns),
                                   float(x), float(y), float(yaw)))


def decode_nav_goal(buf):
    body = unseal(buf, NAV_GOAL_BODY.size + CRC.size)
    ver, op, rsv, goal_id, t_nano, x, y, yaw = NAV_GOAL_BODY.unpack(body)
    if ver != EXT_VER:
        raise BadExtension("ver", str(ver))
    if op not in (NAV_OP_GOAL, NAV_OP_CANCEL):
        raise BadExtension("op", str(op))
    if rsv != b"\0\0":
        raise BadExtension("rsv")
    if not goal_id or not _finite(x, y, yaw):
        raise BadExtension("range", "goal_id %d" % goal_id)
    return {"op": op, "goal_id": goal_id, "t_nano_ns": t_nano, "x": x, "y": y, "yaw": yaw}


def encode_nav_odom(*, seq, t_nano_ns, odom_stamp_ns, odom_pose, v, w, map_odom, flags):
    return seal(NAV_ODOM_BODY.pack(EXT_VER, int(flags), b"\0\0", int(seq) & 0xFFFFFFFF, int(t_nano_ns),
                                   int(odom_stamp_ns), *(float(a) for a in odom_pose), float(v), float(w),
                                   *(float(a) for a in map_odom)))


def decode_nav_odom(buf):
    body = unseal(buf, NAV_ODOM_BODY.size + CRC.size)
    (ver, flags, rsv, seq, t_nano, stamp, ox, oy, oyaw, v, w, mx, my, myaw) = NAV_ODOM_BODY.unpack(body)
    if ver != EXT_VER:
        raise BadExtension("ver", str(ver))
    if rsv != b"\0\0" or flags & ~NAV_ODOM_FLAGS:
        raise BadExtension("rsv", hex(flags))
    if not _finite(ox, oy, oyaw, v, w, mx, my, myaw):
        raise BadExtension("range")
    return {"seq": seq, "t_nano_ns": t_nano, "odom_stamp_ns": stamp, "odom_pose": (ox, oy, oyaw),
            "v": v, "w": w, "map_odom": (mx, my, myaw), "flags": flags,
            "permit": bool(flags & NAV_ODOM_PERMIT), "teleop": bool(flags & NAV_ODOM_TELEOP),
            "hold": bool(flags & NAV_ODOM_HOLD),
            "map_valid": bool(flags & NAV_ODOM_MAP_VALID), "odom_valid": bool(flags & NAV_ODOM_ODOM_VALID)}


def encode_nav_cmd(*, state, reason, flags, goal_id, cmd_seq, odom_seq, kv_mono_ns, cmd_age_us, v, w,
                   remaining_m, path_len_m, recoveries, path_points):
    return seal(NAV_CMD_BODY.pack(EXT_VER, int(state), int(reason), int(flags), int(goal_id),
                                  int(cmd_seq) & 0xFFFFFFFF, int(odom_seq) & 0xFFFFFFFF, int(kv_mono_ns),
                                  min(int(cmd_age_us), 0xFFFFFFFF), float(v), float(w), float(remaining_m),
                                  float(path_len_m), min(int(recoveries), 0xFFFF),
                                  min(int(path_points), 0xFFFF)))


def decode_nav_cmd(buf):
    body = unseal(buf, NAV_CMD_BODY.size + CRC.size)
    (ver, state, reason, flags, goal_id, cmd_seq, odom_seq, kv_mono, age, v, w, rem, plen, rec,
     npts) = NAV_CMD_BODY.unpack(body)
    if ver != EXT_VER:
        raise BadExtension("ver", str(ver))
    if state >= len(NAV_STATES) or reason >= len(NAV_REASONS) or flags & ~NAV_CMD_FLAGS:
        raise BadExtension("enum", "state %d reason %d flags %d" % (state, reason, flags))
    if not _finite(v, w, rem, plen):
        raise BadExtension("range")
    return {"state": state, "state_name": NAV_STATES[state], "reason": reason,
            "reason_name": NAV_REASONS[reason], "flags": flags,
            "accepted": bool(flags & NAV_CMD_ACCEPTED), "valid": bool(flags & NAV_CMD_VALID),
            "need_path": bool(flags & NAV_CMD_NEED_PATH), "goal_id": goal_id, "cmd_seq": cmd_seq,
            "odom_seq": odom_seq, "kv_mono_ns": kv_mono, "cmd_age_us": age, "v": v, "w": w,
            "remaining_m": rem, "path_len_m": plen, "recoveries": rec, "path_points": npts}


def encode_nav_path(goal_id, path_id, t_nano_ns, points):
    pts = [(float(x), float(y)) for x, y in points]
    if not 1 <= len(pts) <= NAV_PATH_MAX_POINTS:
        raise ValueError("NAV_PATH needs 1..%d points (got %d)" % (NAV_PATH_MAX_POINTS, len(pts)))
    return seal(NAV_PATH_HEAD.pack(EXT_VER, b"\0\0\0", int(goal_id), int(path_id) & 0xFFFFFFFF,
                                   int(t_nano_ns), len(pts), b"\0\0")
                + b"".join(NAV_PATH_POINT.pack(x, y) for x, y in pts))


def decode_nav_path(buf):
    if len(buf) < NAV_PATH_HEAD.size + CRC.size:
        raise BadExtension("length", "%d bytes" % len(buf))
    ver, rsv, goal_id, path_id, t_nano, n, rsv2 = NAV_PATH_HEAD.unpack_from(buf)
    if ver != EXT_VER:
        raise BadExtension("ver", str(ver))
    if not 1 <= n <= NAV_PATH_MAX_POINTS:
        raise BadExtension("points", str(n))
    body = unseal(buf, NAV_PATH_HEAD.size + n * NAV_PATH_POINT.size + CRC.size)
    if rsv != b"\0\0\0" or rsv2 != b"\0\0":
        raise BadExtension("rsv")
    pts = [NAV_PATH_POINT.unpack_from(body, NAV_PATH_HEAD.size + i * NAV_PATH_POINT.size) for i in range(n)]
    if not _finite(*(c for p in pts for c in p)):
        raise BadExtension("range")
    return {"goal_id": goal_id, "path_id": path_id, "t_nano_ns": t_nano, "points": pts}


def chunk_loss(client_window, session_window):
    """Frame-channel chunk loss from ONE client's counters and the server's counters for
    that client's session (STATS ``session_counts``), both over the same window.
    Only frame traffic is compared (REQ one way, RESP + SUPERSEDED the other): control
    answers straddle the window edges and other sessions (SYNC, TELEM, ESTOP clients on
    the same server) are not in the session counters at all."""
    c, s = client_window or {}, session_window or {}
    back = ("resp", "superseded")
    return {"nano_to_kv": c.get("tx_chunks:req", 0) - s.get("rx_chunks:req", 0),
            "kv_to_nano": (sum(s.get("tx_chunks:%s" % t, 0) for t in back) -
                           sum(c.get("rx_chunks:%s" % t, 0) for t in back)),
            "nano_sent": c.get("tx_chunks:req", 0), "kv_received": s.get("rx_chunks:req", 0),
            "kv_sent": sum(s.get("tx_chunks:%s" % t, 0) for t in back),
            "nano_received": sum(c.get("rx_chunks:%s" % t, 0) for t in back)}


# ------------------------------------------------------------------ messages
def encode_request(*, frame_id, capture_ns, pose_ns, identity, width, height, nx, ny, nz,
                   unit_f32, voxel, origin, inverse_k, world_camera, codes, depth_crc, roi=None,
                   age_block=None, age_deferred=False, labels=None, replay=False, sensor_key=None):
    """``roi``: a RoiBatch -> ext_flags bit0 and the ROI block after the codes.
    ``age_block`` (encode_age_block bytes) -> ext_flags bit1 and the block at the very end;
    ``age_deferred``: bit1 set but the block is appended by the caller at send time
    (KvLinkClient.call ``tail``), so the stamp is taken right before the first sendto.
    ``labels`` (needs ``roi``; one per ROI row) -> ext_flags bit2 and the LABEL block right
    after the ROI block. ``replay`` -> ext_flags bit3 (no bytes: the depth is a recording).
    ``sensor_key`` (u64) -> ext_flags bit4 and the STAMP block right before the AGE block.
    Without them the bytes are those of 5da3b9d (the 12th head field, then reserved, is 0)."""
    codes = np.ascontiguousarray(codes, dtype="<u2").ravel()
    if labels is not None:
        if roi is None:
            raise ValueError("labels need the ROI block")
        if len(list(labels)) != len(roi.rows):
            raise ValueError("%d labels for %d ROI rows" % (len(list(labels)), len(roi.rows)))
    label_block = encode_label_block(labels) if labels is not None else b""
    if codes.size != int(width) * int(height):
        raise ValueError("codes size does not match width*height")
    roi_block = encode_roi_block(roi) if roi is not None else b""
    head = REQ_HEAD.pack(frame_id, capture_ns, pose_ns, width, height, nx, ny, nz,
                         f32_bits(unit_f32), crc32(codes), depth_crc,
                         (EXT_ROI if roi is not None else 0) |
                         (EXT_AGE if (age_block is not None or age_deferred) else 0) |
                         (EXT_LABEL if labels is not None else 0) |
                         (EXT_REPLAY if replay else 0) |
                         (EXT_STAMP if sensor_key is not None else 0), float(voxel))
    stamp_block = encode_stamp_block(sensor_key) if sensor_key is not None else b""
    return b"".join([head,
                     np.ascontiguousarray(origin, dtype="<f8").tobytes(),
                     np.ascontiguousarray(inverse_k, dtype="<f4").ravel().tobytes(),
                     np.ascontiguousarray(world_camera, dtype="<f4").ravel().tobytes(),
                     b"".join(_pack_str(identity[k]) for k in IDENTITY_FIELDS),
                     codes.tobytes(), roi_block, label_block, stamp_block, age_block or b""])


def decode_request(buf, allow_age=False, allow_label=False, allow_replay=False, allow_stamp=False,
                   crc_fn=None):
    """``crc_fn`` (E7 kv-fast, 2026-11-02): the crc32 of the depth codes, None = ``crc32`` (zlib);
    host/kv_fast_native gives the same value in C++. Raises ValueError for a malformed base request (old rules) and its subclass
    BadExtension for a bad extension (ext_flags, ROI block, AGE block): either way the
    whole request is void. ``allow_age``: only a server that announces ``age_ext``
    (kv_server --ar drm) accepts ext_flags bit1; ``allow_label`` likewise for bit2 (LABEL
    block, only together with the ROI block); ``allow_replay`` likewise for bit3 (replay mark,
    no bytes; ``out["replay"]``); ``allow_stamp`` likewise for bit4 (STAMP block,
    ``out["sensor_stamp_ns"]``, None without it)."""
    (frame_id, capture_ns, pose_ns, width, height, nx, ny, nz, unit_bits, codes_crc,
     depth_crc, ext_flags, voxel) = REQ_HEAD.unpack_from(buf)
    if ext_flags & ~(EXT_ROI | (EXT_AGE if allow_age else 0) | (EXT_LABEL if allow_label else 0) |
                     (EXT_REPLAY if allow_replay else 0) | (EXT_STAMP if allow_stamp else 0)):
        raise BadExtension("flags", hex(ext_flags))
    if ext_flags & EXT_LABEL and not ext_flags & EXT_ROI:
        raise BadExtension("label_without_roi")
    nano_elapsed_ns = None
    if ext_flags & EXT_AGE:                     # fixed-size block at the very end
        if len(buf) < REQ_HEAD.size + AGE_BLOCK_BYTES:
            raise BadExtension("age_length", "%d bytes" % len(buf))
        nano_elapsed_ns = decode_age_block(bytes(buf[len(buf) - AGE_BLOCK_BYTES:]))
        buf = memoryview(buf)[:len(buf) - AGE_BLOCK_BYTES]
    sensor_key = None
    if ext_flags & EXT_STAMP:                   # fixed-size block right before the AGE block
        if len(buf) < REQ_HEAD.size + STAMP_BLOCK_BYTES:
            raise BadExtension("stamp_length", "%d bytes" % len(buf))
        sensor_key = decode_stamp_block(bytes(buf[len(buf) - STAMP_BLOCK_BYTES:]))
        buf = memoryview(buf)[:len(buf) - STAMP_BLOCK_BYTES]
    off = REQ_HEAD.size
    origin = np.frombuffer(buf, dtype="<f8", count=3, offset=off).copy()
    off += 24
    inverse_k = np.frombuffer(buf, dtype="<f4", count=9, offset=off).copy()
    off += 36
    world_camera = np.frombuffer(buf, dtype="<f4", count=12, offset=off).copy()
    off += 48
    identity = {}
    for k in IDENTITY_FIELDS:
        identity[k], off = _unpack_str(buf, off)
    pixels = int(width) * int(height)
    roi = labels = None
    if ext_flags & EXT_ROI:
        if len(buf) - off < 2 * pixels:
            raise ValueError("request carries %d depth bytes, header says %d"
                             % (len(buf) - off, 2 * pixels))
        ext = bytes(buf[off + 2 * pixels:])
        if ext_flags & EXT_LABEL:       # ROI block cut at its own length, the rest is LABEL
            if len(ext) < ROI_REQ_HEAD.size:
                raise BadExtension("length", "%d bytes" % len(ext))
            n_rows = ROI_REQ_HEAD.unpack_from(ext)[2]
            roi_len = ROI_REQ_HEAD.size + n_rows * ROI_REQ_BOX.size + CRC.size
            roi = decode_roi_block(ext[:roi_len])
            labels = decode_label_block(ext[roi_len:], len(roi["rows"]))
        else:
            roi = decode_roi_block(ext)
    elif len(buf) - off != 2 * pixels:
        raise ValueError("request carries %d depth bytes, header says %d" % (len(buf) - off, 2 * pixels))
    codes = np.frombuffer(buf, dtype="<u2", count=pixels, offset=off)
    if (crc32 if crc_fn is None else crc_fn)(codes) != codes_crc:
        raise ValueError("depth code crc mismatch")
    return {"frame_id": frame_id, "capture_ns": capture_ns, "pose_ns": pose_ns,
            "width": width, "height": height, "nx": nx, "ny": ny, "nz": nz,
            "unit_bits": unit_bits, "depth_crc": depth_crc, "voxel": voxel, "origin": origin,
            "inverse_k": inverse_k, "world_camera": world_camera, "identity": identity,
            "codes": codes, "roi": roi, "nano_elapsed_ns": nano_elapsed_ns, "labels": labels,
            "replay": bool(ext_flags & EXT_REPLAY), "sensor_stamp_ns": sensor_key}


def encode_response(req, *, status, counts, depth_crc, flags, times_ns, kv_superseded,
                    kv_incomplete, core_name, roi_block=b""):
    """``roi_block`` (encode_roi_result) goes after the voxel flags; the sender then
    sets RESP_FLAG_ROI in the chunk header."""
    flags = np.ascontiguousarray(flags, dtype=np.uint8).ravel()
    head = RESP_HEAD.pack(req["frame_id"], req["capture_ns"], req["pose_ns"], int(status),
                          int(counts[0]), int(counts[1]), int(counts[2]), int(depth_crc),
                          flags.size)
    times = RESP_TIMES.pack(*(max(0, int(times_ns.get(k, 0))) for k in TIMING_FIELDS))
    return b"".join([head, times, RESP_TAIL.pack(kv_superseded & 0xFFFFFFFF,
                                                 kv_incomplete & 0xFFFFFFFF),
                     b"".join(_pack_str(req["identity"][k]) for k in IDENTITY_FIELDS),
                     _pack_str(core_name), flags.tobytes(), roi_block])


def decode_response(buf, chunk_flags=0):
    """``chunk_flags`` is the RESP chunk-header flags byte: with RESP_FLAG_ROI the ROI
    result block follows the voxel flags (``out["roi"]``), else ``out["roi"]`` is None."""
    (frame_id, capture_ns, pose_ns, status, c0, c1, c2, depth_crc, nvox) = RESP_HEAD.unpack_from(buf)
    off = RESP_HEAD.size
    times = dict(zip(TIMING_FIELDS, RESP_TIMES.unpack_from(buf, off)))
    off += RESP_TIMES.size
    kv_superseded, kv_incomplete = RESP_TAIL.unpack_from(buf, off)
    off += RESP_TAIL.size
    identity = {}
    for k in IDENTITY_FIELDS:
        identity[k], off = _unpack_str(buf, off)
    core, off = _unpack_str(buf, off)
    roi = None
    if chunk_flags & RESP_FLAG_ROI:
        if len(buf) - off < nvox:
            raise ValueError("response carries %d flag bytes, header says %d" % (len(buf) - off, nvox))
        roi = decode_roi_result(bytes(buf[off + nvox:]))
    elif len(buf) - off != nvox:
        raise ValueError("response carries %d flag bytes, header says %d" % (len(buf) - off, nvox))
    flags = np.frombuffer(buf, dtype=np.uint8, count=nvox, offset=off).copy()
    return {"frame_id": frame_id, "capture_ns": capture_ns, "pose_ns": pose_ns,
            "status": status, "counts": (c0, c1, c2), "depth_crc": depth_crc,
            "times_ns": times, "kv_superseded": kv_superseded, "kv_incomplete": kv_incomplete,
            "identity": identity, "core": core, "flags": flags, "roi": roi}


# ------------------------------------------------------------------ Nano side
class KvLinkClient:
    """Window-1 request/answer client. One instance = one session id.

    ``call`` sends a request and blocks until the matching answer, a
    superseded notice for it (LinkSuperseded) or the timeout (LinkTimeout).
    Anything else that arrives is counted and dropped.
    """

    def __init__(self, server, *, chunk_bytes=CHUNK_FRAGMENTED, timeout_s=2.0,
                 session=None, sock=None, rcvbuf=4 << 20, poke_s=POKE_S, send_timeout_s=None):
        self.server = (server[0], int(server[1]))
        # how long one sendto may block on a full send buffer (None = timeout_s); see _send
        self.send_timeout_s = float(timeout_s if send_timeout_s is None else send_timeout_s)
        # ``poke_s``: a frame call without answer after poke_s (then 2x, 4x ...) sends one HELLO
        # (answered by every server version); the KV260 transmitting that answer also sends a
        # frame its NIC kept back. 0 = never. docs/design/KV_LINK_BURST_TAIL_20261011.md
        self.poke_s = float(poke_s)
        self.chunk_bytes = int(chunk_bytes)
        self.timeout_s = float(timeout_s)
        self.session = int.from_bytes(os.urandom(8), "little") if session is None else int(session)
        self.counts = Counter()
        self.sock = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if sock is None:
            self.rcvbuf = set_rcvbuf(self.sock, rcvbuf)
            self.sock.bind(("0.0.0.0", 0))
        else:
            self.rcvbuf = ("given", None)
        self.rx = Reassembler(self.counts)
        self.server_info = None
        self.last = None              # timing record of the last call
        self._ctrl_seq = 0
        # Late answers (docs/design/KV_LINK_BURST_TAIL_20261011.md): send time of the frames that
        # timed out, and one record per such answer that still arrived later (stale_response).
        self._timed_out = OrderedDict()   # frame_id -> monotonic send time (last 64)
        self.late_answers = []            # last 256 {frame_id, late_s, during_frame, since_send_s}

    # -- low level
    def _send(self, msg_type, frame_id, payload, flags=0):
        # The socket is shared with _recv_message, whose settimeout(left) would otherwise also bound
        # this sendto: with a full send buffer (a short TX stall) the send then failed after the few
        # ms left over from the last receive, as a raw TimeoutError that M7Pipeline does not catch
        # (board 10-15 r1-r3: nano_replay died). The send gets its own bound, ``send_timeout_s``
        # (default = the call timeout), and a send that stays blocked that long is a LinkTimeout.
        n = 0
        self.sock.settimeout(self.send_timeout_s)
        for dgram in split(msg_type, self.session, frame_id, payload, self.chunk_bytes, flags):
            try:
                self.sock.sendto(dgram, self.server)
            except socket.timeout:
                self.counts["send_timeout"] += 1
                raise LinkTimeout("sendto to %s:%d blocked %.3f s (send buffer full), chunk %d"
                                  % (self.server[0], self.server[1], self.send_timeout_s, n))
            n += 1
        self.counts["tx_chunks"] += n
        self.counts["tx_chunks:%s" % TYPE_NAMES[msg_type]] += n
        self.counts["tx_messages:%s" % TYPE_NAMES[msg_type]] += 1
        return n

    def _recv_message(self, deadline):
        """Next complete message for this session, or None at the deadline."""
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            self.sock.settimeout(left)
            try:
                dgram, _ = self.sock.recvfrom(65536)
            except socket.timeout:
                return None
            if len(dgram) >= CHUNK.size and dgram[:4] == MAGIC:
                if dgram[4] == NUDGE:
                    self.counts["rx_nudge"] += 1
                    continue
                self.counts["rx_chunks:%s" % TYPE_NAMES.get(dgram[4], dgram[4])] += 1
            got = self.rx.feed(dgram)
            if got is None:
                continue
            mtype, flags, session, frame_id, payload, first_t = got
            if session != self.session:
                self.counts["foreign_session"] += 1
                continue
            return mtype, flags, frame_id, payload, first_t

    def _control(self, msg_type, ack_type, body, timeout_s, flags=0):
        payload = self._control_raw(msg_type, ack_type, body, timeout_s, flags)
        return None if payload is None else json.loads(payload.decode("utf-8"))

    def _control_raw(self, msg_type, ack_type, body, timeout_s, flags=0, frame_id=None):
        """Send one control message under a NEW chunk frame id (the reassembler drops a
        frame id it already completed) and return the matching answer's raw payload.
        An answer of the right type for an older frame id is counted ``stale:<type>``.
        ``frame_id`` forces the chunk frame id (tests only: the reuse positive control)."""
        if frame_id is None:
            self._ctrl_seq += 1
            seq = self._ctrl_seq
        else:
            seq = int(frame_id)
        self._send(msg_type, seq, body, flags)
        deadline = time.monotonic() + timeout_s
        while True:
            got = self._recv_message(deadline)
            if got is None:
                return None
            mtype, _, frame_id, payload, _ = got
            if mtype == ack_type and frame_id == seq:
                return payload
            if mtype == ack_type and frame_id < seq:
                self.counts["stale:%s" % TYPE_NAMES.get(mtype, mtype)] += 1
                continue
            self.counts["unexpected_while_control:%s" % TYPE_NAMES.get(mtype, mtype)] += 1

    def require(self, feature, hello_timeout_s=3.0):
        """HELLO once, then the server must announce ``feature`` (A.1 rule 3), else
        KvFeatureUnavailable and nothing of that feature is ever sent."""
        if self.server_info is None:
            self.hello(hello_timeout_s)
        feats = self.server_info.get("features")
        if not isinstance(feats, dict) or feats.get(feature) != 1:
            self.counts["feature_unavailable:%s" % feature] += 1
            raise KvFeatureUnavailable("KV260 server at %s:%d does not announce feature %r "
                                       "(features %r)" % (self.server[0], self.server[1], feature,
                                                         feats))
        return self.server_info

    def _control_resend(self, msg_type, ack_type, body, timeout_s, first_gap_s, flags=0):
        """Idempotent control request: resend under a NEW chunk frame id after
        ``first_gap_s``, doubling the gap, until ``timeout_s``; the answer to ANY of
        this round's frame ids is taken. (A resend must not reuse the frame id: the
        server's reassembler drops a frame id it already completed for the session.)"""
        deadline = time.monotonic() + timeout_s
        first = None
        gap = float(first_gap_s)
        name = TYPE_NAMES[msg_type]
        while True:
            now = time.monotonic()
            if now >= deadline:
                return None
            self._ctrl_seq += 1
            if first is None:
                first = self._ctrl_seq
            else:
                self.counts["resend:%s" % name] += 1
            self._send(msg_type, self._ctrl_seq, body, flags)
            until = min(deadline, now + gap) if gap > 0 else deadline
            gap *= 2
            while True:
                got = self._recv_message(until)
                if got is None:
                    break
                mtype, _, frame_id, payload, _ = got
                if mtype == ack_type and first <= frame_id <= self._ctrl_seq:
                    if frame_id != self._ctrl_seq:
                        self.counts["answered_earlier_send:%s" % name] += 1
                    return payload
                if mtype == ack_type and frame_id < first:
                    self.counts["stale:%s" % TYPE_NAMES.get(mtype, mtype)] += 1
                    continue
                self.counts["unexpected_while_control:%s" % TYPE_NAMES.get(mtype, mtype)] += 1

    def hello(self, timeout_s=3.0, resend_s=HELLO_RESEND_S):
        """HELLO, resent (new frame id each time) after ``resend_s``, 2x, 4x ... within
        ``timeout_s``; ``resend_s`` 0 = one send. HELLO has no server-side effect besides
        counters, so a resend is idempotent and any of its answers is as good."""
        payload = self._control_resend(HELLO, HELLO_ACK, b"{}", timeout_s, resend_s)
        if payload is None:
            self.counts["hello_timeout"] += 1
            raise LinkTimeout("no HELLO answer from %s:%d within %.1f s"
                              % (self.server[0], self.server[1], timeout_s))
        info = json.loads(payload.decode("utf-8"))
        self.server_info = info
        return info

    def stats(self, reset=False, timeout_s=3.0):
        return self._control(STATS, STATS_ACK, b"{}", timeout_s, FLAG_RESET if reset else 0)

    def nav_call(self, msg_type, body, timeout_s, resend_s=0.0):
        """CTRLPS2: one NAV_GOAL / NAV_ODOM / NAV_PATH body (encode_nav_*) -> its decoded NAV_CMD, or None
        at ``timeout_s``. ``resend_s`` > 0 resends under new frame ids (GOAL / PATH: idempotent by
        goal_id / path_id); 0 = one send (ODOM: the next one at 50 Hz is the resend). Call
        ``require("nav")`` first; a malformed NAV_CMD raises BadExtension (counted)."""
        if msg_type not in (NAV_GOAL, NAV_ODOM, NAV_PATH):
            raise ValueError("not a NAV request type: %r" % (msg_type,))
        if resend_s > 0:
            raw = self._control_resend(msg_type, NAV_CMD, body, timeout_s, resend_s)
        else:
            raw = self._control_raw(msg_type, NAV_CMD, body, timeout_s)
        if raw is None:
            self.counts["nav_timeout:%s" % TYPE_NAMES[msg_type]] += 1
            return None
        try:
            return decode_nav_cmd(raw)
        except BadExtension as exc:
            self.counts["bad_ext:%s" % exc.reason] += 1
            raise

    # -- the frame call
    def call(self, request_bytes, expect, tail=None):
        """Send one request; return (decoded response, timing dict). ``expect``
        holds the identity the answer must echo (frame_id, capture_ns, pose_ns,
        identity strings). ``tail``: callable returning bytes appended right before
        the first sendto (the deferred AGE block)."""
        frame_id = expect["frame_id"]
        t0 = time.monotonic()
        if tail is not None:
            request_bytes = request_bytes + tail()
        n = self._send(REQ, frame_id, request_bytes)
        t1 = time.monotonic()
        self.counts["calls"] += 1
        deadline = t1 + self.timeout_s
        gap = self.poke_s
        poke_at = t1 + gap if gap > 0 else deadline
        pokes = 0
        while True:
            got = self._recv_message(min(deadline, poke_at))
            if got is None and time.monotonic() < deadline:
                self._ctrl_seq += 1             # new chunk frame id, like every control send
                self._send(HELLO, self._ctrl_seq, b"{}")
                self.counts["poke:hello"] += 1
                pokes += 1
                gap *= 2
                poke_at = time.monotonic() + gap
                continue
            if got is None:
                self.counts["timeout"] += 1
                self._timed_out[frame_id] = t0
                while len(self._timed_out) > 64:
                    self._timed_out.popitem(last=False)
                pend = self.rx.pending()
                raise LinkTimeout("no answer for frame %d within %.3f s (chunks sent %d, "
                                  "unfinished answer %s)" % (frame_id, self.timeout_s, n, pend))
            mtype, rflags, rid, payload, first_t = got
            if mtype == SUPERSEDED:
                fid, cap = NOTICE.unpack_from(payload)
                if fid == frame_id and cap == expect["capture_ns"]:
                    self.counts["superseded_notice"] += 1
                    raise LinkSuperseded("frame %d superseded on the KV260" % frame_id)
                self.counts["stale_notice"] += 1
                continue
            if mtype == HELLO_ACK and self.counts["poke:hello"]:
                self.counts["poke_answer"] += 1
                continue
            if mtype != RESP:
                self.counts["unexpected:%s" % TYPE_NAMES.get(mtype, mtype)] += 1
                continue
            if rid < frame_id:
                self.counts["stale_response"] += 1
                sent = self._timed_out.pop(rid, None)
                if sent is not None:
                    now = time.monotonic()
                    self.late_answers.append({"frame_id": int(rid), "late_s": now - sent,
                                              "during_frame": int(frame_id),
                                              "since_send_s": now - t0})
                    del self.late_answers[:-256]
                self.rx.forget_done(RESP)
                continue
            try:
                resp = decode_response(payload, rflags)
            except BadExtension as exc:          # void as a whole (A.1 rule 4)
                self.counts["bad_ext:%s" % exc.reason] += 1
                self.counts["malformed_response"] += 1
                self.rx.forget_done(RESP)
                continue
            except (ValueError, struct.error, UnicodeDecodeError):
                self.counts["malformed_response"] += 1
                self.rx.forget_done(RESP)
                continue
            if (rid != frame_id or resp["frame_id"] != frame_id or
                    resp["capture_ns"] != expect["capture_ns"] or
                    resp["pose_ns"] != expect["pose_ns"] or
                    resp["identity"] != expect["identity"]):
                self.counts["identity_mismatch"] += 1
                self.rx.forget_done(RESP)
                continue
            t2 = time.monotonic()
            self.counts["answered"] += 1
            if pokes:
                self.counts["answered_after_poke"] += 1
            return resp, {"send_s": t1 - t0, "rtt_s": t2 - t0, "resp_first_chunk_s": first_t - t0,
                          "tx_chunks": n, "pokes": pokes}

    def close(self):
        self.sock.close()


# ROI answer of one frame (``backend.last_roi``): ok, reason (None when ok, else
# ``region_backend_failure:<why>``), rows (dicts as evaluate_boxes returns, only when
# ok), roi_core ("none"/"cpu"/"pl" that ran it, "" when no answer), times_ns
# {prep, device, readback, total} (KV-clock durations), roi_cycles (PL cycles or None).
RoiOutcome = namedtuple("RoiOutcome", "ok reason rows roi_core times_ns roi_cycles")


def _roi_failure(why, answer=None):
    a = answer or {}
    return RoiOutcome(False, "region_backend_failure:%s" % why, [], a.get("roi_core", ""),
                      dict(a.get("times_ns") or {}), a.get("roi_cycles"))


def check_roi_answer(batch, answer):
    """Nano-side check of a KV ROI answer against the sent batch (A.2): revision
    echo, status 0, one row per box, then host.m7_object_native.check_roi_row on
    every row (echoed id/enabled, monotone counts, PIXELS == area, verdict
    recomputed, code in {0,2,3}). Any failure voids the frame's ROI only."""
    from host.m7_object_native import ROI_FIELDS, check_roi_row
    if answer is None:
        return _roi_failure("no_roi_block")
    if answer["memory_revision"] != int(batch.memory_revision):
        return _roi_failure("revision_mismatch", answer)
    if answer["roi_status"] == ROI_NOT_CONFIGURED:
        return _roi_failure("kv_roi_not_configured", answer)
    if answer["roi_status"] != ROI_OK:
        return _roi_failure("roi_status_%#x" % answer["roi_status"], answer)
    if len(answer["rows"]) != len(batch.rows):
        return _roi_failure("row_count", answer)
    rows = []
    for sent, got in zip(batch.rows, answer["rows"]):
        values = dict(zip(ROI_FIELDS, (int(v) for v in got)))
        enabled = 1 if (sent.enabled and sent.xyxy is not None) else 0
        rect = tuple(int(v) for v in sent.xyxy) if enabled else (0, 0, 0, 0)
        try:
            check_roi_row(values, int(sent.row_id), enabled, rect, batch.cfg)
        except RuntimeError as exc:
            return _roi_failure("row_check:%s" % exc, answer)
        rows.append(values)
    return RoiOutcome(True, None, rows, answer["roi_core"], dict(answer["times_ns"]),
                      answer["roi_cycles"])


class _RemoteFrameCall:
    """Stands where FrameNativeM7 keeps its ctypes library: same call signature.

    The capture identity (and the optional ROI batch) comes from
    ``RemotePlFrameBackend.frame_from_depth``.
    """

    # AR2NANO (2026-11-02): callable taking the answered frame's raw record (host/nano_ar.py
    # NanoArScreen.offer); None (default) = nothing extra runs, the code before.
    ar_sink = None

    def __init__(self, client, unit_f32, counts):
        self.client, self.unit, self.counts = client, np.float32(unit_f32), counts
        self.pending = None
        self.last = None
        self.roi_outcome = None

    def fpga_m7_depth_frame(self, depth_full, inverse_k, world_camera, origin, voxel,
                           width, height, nx, ny, nz, flags, counts, status):
        if self.pending is None:
            raise RuntimeError("remote frame call without a capture identity")
        ident = self.pending
        t0 = time.monotonic()
        pixels = int(width) * int(height)
        depth = depth_full[:pixels]
        codes = ident.get("codes")
        if codes is None:
            try:
                codes = m_to_codes(depth, self.unit)
            except ValueError:
                self.counts["depth_not_representable"] += 1
                raise
        else:
            # Caller-supplied camera codes (opt-in, car runtime): no Nano-side requantization.
            # Equality with ``depth`` is still proven end to end: the KV260 returns the crc
            # of codes_to_m(codes) and it must equal crc32(depth) below, else the frame fails.
            codes = np.ascontiguousarray(codes, dtype="<u2").ravel()
            if codes.size != pixels:
                raise ValueError("supplied codes do not match width*height")
            self.counts["codes_supplied"] += 1
        depth_crc = crc32(depth)
        t1 = time.monotonic()
        feats = (self.client.server_info or {}).get("features")
        age_ext = isinstance(feats, dict) and feats.get("age_ext") == 1
        labels = ident.get("labels")
        if labels is not None and not (isinstance(feats, dict) and feats.get("label_ext") == 1
                                       and ident.get("roi") is not None):
            labels = None               # the server does not want them: send as before
        if labels is not None:
            self.counts["label_ext_sent"] += 1
        replay = bool(ident.get("replay")) and isinstance(feats, dict) and feats.get("replay_ext") == 1
        sensor_key = ident.get("sensor_key")
        if sensor_key is not None and not (isinstance(feats, dict) and feats.get("stamp_ext") == 1):
            sensor_key = None           # the server does not want it: bytes as before
        if sensor_key is not None:
            self.counts["stamp_ext_sent"] += 1
        if replay:
            self.counts["replay_ext_sent"] += 1
        elif ident.get("replay"):
            self.counts["replay_ext_unannounced"] += 1     # the server would void the bit
        req = encode_request(frame_id=ident["frame_id"], capture_ns=ident["capture_ns"],
                             pose_ns=ident["pose_ns"], identity=ident["identity"],
                             width=width, height=height, nx=nx, ny=ny, nz=nz,
                             unit_f32=self.unit, voxel=voxel, origin=origin,
                             inverse_k=inverse_k, world_camera=world_camera,
                             codes=codes, depth_crc=depth_crc, roi=ident.get("roi"),
                             age_deferred=age_ext, labels=labels, replay=replay,
                             sensor_key=sensor_key)
        t2 = time.monotonic()
        tail = None
        if age_ext:                     # Nano clock only: capture -> right before sendto
            cap = ident["capture_ns"]

            def tail():
                self.counts["age_ext_sent"] += 1
                return encode_age_block(max(0, now_ns() - cap))
        try:
            resp, net = self.client.call(req, ident, tail=tail)
        except LinkError as exc:
            self.counts["link_failure:%s" % type(exc).__name__] += 1
            self.last = {"encode_s": t2 - t0, "quantize_s": t1 - t0, "failed": type(exc).__name__}
            raise
        t3 = time.monotonic()
        if resp["depth_crc"] != depth_crc:
            self.counts["depth_crc_mismatch"] += 1
            raise RuntimeError("KV260 float32 depth differs from the Nano's (crc %08x vs %08x)"
                               % (resp["depth_crc"], depth_crc))
        nvox = int(nx) * int(ny) * int(nz)
        if resp["flags"].size != nvox:
            self.counts["bad_flag_count"] += 1
            raise RuntimeError("KV260 returned %d flags for a %d-voxel grid"
                               % (resp["flags"].size, nvox))
        flags[:nvox] = resp["flags"]
        counts[:] = resp["counts"]
        status[0] = resp["status"]
        batch = ident.get("roi")
        if batch is not None:           # ROI never fails the geometry (A.2)
            self.roi_outcome = check_roi_answer(batch, resp["roi"])
            self.counts["roi_ok" if self.roi_outcome.ok else
                        "roi_failure:%s" % self.roi_outcome.reason.split(":", 2)[1]] += 1
        elif resp["roi"] is not None:
            self.counts["roi_unrequested"] += 1
        if self.ar_sink is not None:    # AR2NANO (2026-11-02): car screen on the Nano; never fails the frame
            try:
                self.ar_sink({"ident": ident, "width": int(width), "height": int(height), "nx": int(nx),
                              "ny": int(ny), "nz": int(nz), "unit_f32": self.unit, "voxel": voxel,
                              "origin": origin, "inverse_k": inverse_k, "world_camera": world_camera,
                              "codes": codes, "depth_crc": depth_crc, "resp": resp, "sent_ns": now_ns()})
            except Exception:           # noqa: BLE001 -- display only
                self.counts["ar_sink_error"] += 1
        t4 = time.monotonic()
        kv = resp["times_ns"]
        service = sum(kv[k] for k in KV_SERVICE_FIELDS) * 1e-9
        self.last = {"quantize_s": t1 - t0, "encode_s": t2 - t0, "send_s": net["send_s"],
                     "rtt_s": net["rtt_s"], "resp_first_chunk_s": net["resp_first_chunk_s"],
                     "unpack_s": t4 - t3, "tx_chunks": net["tx_chunks"],
                     "kv_ns": dict(kv), "kv_service_s": service,
                     # RTT minus the KV260's measured service (slot wait .. core):
                     # Nano sendto calls, both wire directions, KV reassembly
                     # (kv_ns["recv"] overlaps the Nano's sending), the answer's
                     # packing/sending and the Nano's reassembly. One residual,
                     # single clock per term, no cross-board clock offset.
                     "transport_s": net["rtt_s"] - service,
                     "kv_superseded_total": resp["kv_superseded"],
                     "kv_incomplete_total": resp["kv_incomplete"], "core": resp["core"]}


class RemotePlFrameBackend(FrameNativeM7):
    """FrameNativeM7 whose core call runs on the KV260 (``backend="remote_pl"``).

    ``depth_unit_m`` is required: it is the unit of the camera's uint16 codes
    (meta.json ``depth_unit_m``) and every frame's float32 depth must be
    exactly ``f32(code) * f32(unit)``, else the frame fails.
    """

    accepts_capture_identity = True
    # "replay" / "sim" (line D1): every request carries the replay mark (ext_flags bit3) when the
    # server announces "replay_ext"; "live" / None: never (bytes as before).
    data_source = None

    def __init__(self, client, *, depth_unit_m):
        unit = float(depth_unit_m)
        if not (unit > 0 and np.isfinite(unit)):
            raise ValueError("depth_unit_m must be a positive finite number of metres")
        self.counts = Counter()
        self.client = client
        self.depth_unit_m = unit
        info = client.server_info or {}
        self.backend_name = "remote_pl:%s" % info.get("core", "unknown")
        self.lib = _RemoteFrameCall(client, np.float32(unit), self.counts)

    last_roi = None

    def set_next_roi(self, roi, labels=None):
        """ROI batch for the next frame_from_depth call that has no ``roi`` argument
        (lets a caller that goes through M7Pipeline attach boxes to the frame).
        ``labels``: optional class name per ROI row (car AR display; clean_wire_label is
        applied here), sent only when the server announces ``label_ext``."""
        self._next_roi = roi
        self._next_labels = None
        if labels is not None and roi is not None:
            labels = list(labels)
            if len(labels) != len(roi.rows):
                raise ValueError("%d labels for %d ROI rows" % (len(labels), len(roi.rows)))
            wire = []
            for name in labels:
                b, why = clean_wire_label(name)
                if why:
                    self.counts["label:%s" % why] += 1
                wire.append(b)
            self._next_labels = wire

    def set_next_sensor_key(self, key):
        """AR colour bypass: the depth frame's sensor key (live: camera header stamp ns; replay: the
        recording's frame key) for the next frame_from_depth call; sent (STAMP block) only when the
        server announces ``stamp_ext``. The caller sets or clears it before EVERY frame."""
        self._next_sensor_key = None if key is None else int(key)

    def set_next_codes(self, codes):
        """uint16 camera codes for the next frame_from_depth call that has no ``codes``
        argument (M7Pipeline has no codes channel; host/geo_producer.py sets or clears this
        before EVERY frame so a frame the pipeline drops before the backend cannot hand its
        codes to the next one -- the depth crc check would fail that frame anyway)."""
        self._next_codes = codes

    def frame_from_depth(self, *, identity=None, roi=None, codes=None, **kwargs):
        """``roi``: RoiBatch (host.m7_object_native.plan_boxes) sent with this frame's
        request; the per-box answer lands in ``self.last_roi`` (RoiOutcome; None when
        no ROI was sent). Needs the server's ``roi_ext`` feature, else
        KvFeatureUnavailable before anything is sent. ``codes``: optional uint16 camera
        codes of ``depth_m`` (skips requantization; the returned depth crc still has to
        match ``depth_m`` exactly). Without it the request bytes are unchanged."""
        if roi is None:
            roi, self._next_roi = getattr(self, "_next_roi", None), None
            labels, self._next_labels = getattr(self, "_next_labels", None), None
        else:
            labels = None
        if codes is None:
            codes, self._next_codes = getattr(self, "_next_codes", None), None
        sensor_key, self._next_sensor_key = getattr(self, "_next_sensor_key", None), None
        self.last_roi = None
        if identity is None:
            raise ValueError("remote_pl needs the capture identity (M7Pipeline passes it)")
        if (identity.frame_id != kwargs["frame_id"] or
                identity.acquired_ns != kwargs["capture_time_ns"] or
                identity.calibration_id != kwargs["calibration_version"]):
            raise ValueError("capture identity disagrees with the frame arguments")
        if roi is not None:
            info = self.client.require("roi_ext")
            if len(roi.rows) > int(info.get("roi_max_boxes", 0)):
                raise ValueError("ROI batch of %d rows exceeds the server's roi_max_boxes %r"
                                 % (len(roi.rows), info.get("roi_max_boxes")))
        self.lib.pending = {"frame_id": int(kwargs["frame_id"]),
                            "capture_ns": int(kwargs["capture_time_ns"]),
                            "pose_ns": int(kwargs["pose_time_ns"]),
                            "identity": {k: getattr(identity, k) for k in IDENTITY_FIELDS},
                            "roi": roi, "codes": codes,
                            "labels": labels if roi is not None else None,
                            "replay": self.data_source in ("replay", "sim"),
                            "sensor_key": sensor_key}
        self.lib.roi_outcome = None
        self.counts["frames"] += 1
        try:
            out = super().frame_from_depth(**kwargs)
        except Exception:
            if roi is not None:
                self.last_roi = _roi_failure("geometry_failed")
            raise
        finally:
            self.lib.pending = None
        if roi is not None:
            self.last_roi = self.lib.roi_outcome or _roi_failure("no_answer")
        return out


def remote_pl_factory(server, *, depth_unit_m, chunk_bytes=CHUNK_FRAGMENTED, timeout_s=2.0,
                      hello_timeout_s=3.0):
    """For M7BackendSelector(remote_pl_factory=...): connect, handshake, or raise."""
    from host.m7_backend import PlBackendUnavailable

    def make():
        client = KvLinkClient(server, chunk_bytes=chunk_bytes, timeout_s=timeout_s)
        try:
            client.hello(hello_timeout_s)
        except (LinkError, OSError) as exc:
            client.close()
            raise PlBackendUnavailable("backend='remote_pl': %s" % exc) from exc
        return RemotePlFrameBackend(client, depth_unit_m=depth_unit_m)
    return make


# ------------------------------------------------------------------ KV260 side
class NullCore:
    """Answers at once without running any core: every pixel counted invalid,
    every voxel unknown (so the Nano gets ``no_valid_grid_samples``, never a
    fresh-looking empty map). Measures the channel alone."""

    name = "null"

    def __call__(self, depth_full, inverse_k, world_camera, origin, voxel, width, height,
                 nx, ny, nz, flags, counts, status):
        n = int(nx) * int(ny) * int(nz)
        flags[:n] = 0
        counts[:] = [0, int(width) * int(height), 0]
        status[0] = 0


class CpuRoiCore:
    """Region check on the KV260's CPU: the E2 C reference ``fpga_m7_object_roi_batch``
    (hw/m7/object_evidence.cpp, g++). Raw: returns the kernel status and the first
    box_count rows exactly as written; the Nano does every row check."""

    name = "cpu"

    def __init__(self, library):
        from host.m7_object_native import NativeObjectEvidence
        self.lib = NativeObjectEvidence(library).lib

    def __call__(self, depth2d, roi):
        t0 = time.monotonic_ns()
        n = len(roi["rows"])
        height, width = depth2d.shape
        enabled = np.zeros(ROI_MAX_BOXES, dtype=np.uint32)
        rect = np.zeros(4 * ROI_MAX_BOXES, dtype=np.uint32)
        near = np.zeros(ROI_MAX_BOXES, dtype=np.float64)
        far = np.zeros(ROI_MAX_BOXES, dtype=np.float64)
        ids = np.zeros(ROI_MAX_BOXES, dtype=np.uint32)
        for b, (row_id, en, x0, y0, x1, y1, nz, fz) in enumerate(roi["rows"]):
            ids[b], enabled[b] = row_id, en
            rect[4 * b:4 * b + 4] = (x0, y0, x1, y1)
            near[b], far[b] = nz, fz
        result = np.full(7 * ROI_MAX_BOXES, 0xFFFFFFFF, dtype=np.uint32)
        status = np.full(1, 0xFFFFFFFF, dtype=np.uint32)
        cfg = roi["cfg"]
        self.lib.fpga_m7_object_roi_batch(
            np.ascontiguousarray(depth2d, dtype=np.float32), width, height, n, enabled, rect,
            near, far, ids, float(cfg["depth_margin_m"]), int(cfg["min_pixels"]),
            float(cfg["min_valid_fraction"]), float(cfg["free_fraction"]), result, status)
        rows = [tuple(int(v) for v in result[7 * b:7 * b + 7]) for b in range(n)] \
            if status[0] == 0 else []
        total = time.monotonic_ns() - t0
        return {"status": int(status[0]), "rows": rows,
                "times_ns": {"prep": 0, "device": total, "readback": 0, "total": total},
                "cycles": None}


class KvLinkServer:
    """UDP server: reader thread (reassembly -> latest slot; HELLO/STATS/SYNC/TELEM/ESTOP
    answered inline) + worker thread (geometry core, then the optional ROI core).

    ``roi_core``: None (= --roi-core none: ROI requests get status 0x100) or a callable
    ``(depth2d_f32, roi_dict) -> {status, rows, times_ns, cycles}`` with ``.name``.
    ``regs``: {name: host.kv_regs.PlRegBlock} opened once by the caller (shared with the
    SYNC responder); ``estop``: host.kv_estop.EstopLatch (the only CTRL writer).
    ``permit_feed``: host.kv_regs.NanoFrameFeed (or the KvCounterFrameFeed positive control)
    or None. When given, every decoded geometry request is admitted by it right before the
    frame core starts (it writes NEXT_FRAME_ID from the request's Nano frame id and
    ``identity.stream_instance``); a request it does not admit is dropped without running
    the core and without an answer (counted ``permit_drop:<verdict>``).
    """

    def __init__(self, core_call, core_name, *, bind=("0.0.0.0", 0), chunk_bytes=CHUNK_FRAGMENTED,
                 rcvbuf=8 << 20, info=None, sock=None, timing_source=None, live_info=None,
                 sync_responder=None, roi_core=None, regs=None, estop=None, telem=None,
                 geometry_desc=None, boot_id=None, send_log=None, send_log_iface=None,
                 nudge_after_burst=True, nudge_idle_s=NUDGE_IDLE_S, permit_feed=None, ar=None,
                 send_timeout_s=SERVER_SEND_TIMEOUT_S, rolling=None, roi_concurrent=False,
                 core_call_codes=None, fast=None, nav=None):
        self.core_call, self.core_name = core_call, core_name
        # CTRLPS2 (2026-11-03): host.kv_nav_service.NavService (kv_server --nav soft|pl) or None (default:
        # "nav" not announced, NAV_* messages counted unexpected:<type>, everything exactly as before).
        # NAV_GOAL / NAV_ODOM / NAV_PATH are answered inline on the reader with one NAV_CMD each.
        self.nav = nav
        # D16 (2026-11-03): callable (codes, unit_bits, inverse_k, world_camera, origin, voxel, width,
        # height, nx, ny, nz, flags, counts, status) -> depth crc, the PL frame core converting the uint16
        # codes itself (host/m7_backend._PlFrameCall.fpga_m7_depth_frame_codes; GEO_FX3: always given for
        # kv_server --core pl, the fixed-point core reads codes only). None: every frame on core_call. With it, a frame whose unit passes
        # d16_unit_ok skips the A53 conversion and float32 crc; the float32 depth is still made on the A53
        # when the ROI core needs it (that core reads float32), and the crc is made on the A53 when the
        # core returns status != 0 (it then reads no depth and reports crc 0).
        self.core_call_codes = core_call_codes
        # INT_SW (2026-11-02, SLIM_ROI README 4): True = the ROI core runs in a worker thread at the same time
        # as the geometry core (separate register windows and DMA buffers; both only READ the shared float
        # depth, which is not touched again before both are done). False (default): serial, exactly as before.
        self.roi_concurrent = bool(roi_concurrent)
        self._roi_pool = None
        if self.roi_concurrent:
            from concurrent.futures import ThreadPoolExecutor
            self._roi_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="kv-roi")
        # E7 kv-fast (2026-11-02): host.kv_fast_native.KvFastNative or None (= off, exactly as before).
        # When given, the codes crc32 and the codes -> float32 conversion + depth crc32 run in C++ (one
        # pass); same bytes as the Python path. Not used while ``convert`` is replaced (crc positive control).
        self.fast = fast
        # host.kv_rolling.RollingService (kv_server --rolling cpu|pl) or None: ROLL is then
        # announced and served by the worker, after the frame that produced the block.
        self.rolling = rolling
        self.send_timeout_s = float(send_timeout_s)
        # Diagnostics (docs/design/KV_LINK_BURST_TAIL_20261011.md): ``send_log`` = callable(dict)
        # gets one record per multi-frame answer (RESP, multi-chunk control ACKs) with KV
        # monotonic stamps of the serve stages and of the sendto calls; ``send_log_iface`` also
        # reads that NIC's sysfs tx_packets right before and after the sendto calls (local
        # reads, nothing is transmitted).
        self.send_log = send_log
        self._tx_path = (None if not send_log_iface else
                         "/sys/class/net/%s/statistics/tx_packets" % send_log_iface)
        self.ar = ar                            # host.kv_ar.ArPresenter (kv_server --ar drm) or None
        self.convert = codes_to_m               # replaced only by the crc positive control
        self.timing_source = timing_source      # object with .last_times (PL core) or None
        self.live_info = live_info              # callable -> dict, sampled at every STATS
        if sync_responder is None:              # SYNC is always answered; PL pair only if given
            from host.kv_clock_sync import SyncResponder
            sync_responder = SyncResponder(pl=None)
        self.sync_responder = sync_responder
        self.chunk_bytes = int(chunk_bytes)
        self.sock = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if sock is None:
            self.rcvbuf = set_rcvbuf(self.sock, rcvbuf)
            self.sock.bind(bind)
        else:
            self.rcvbuf = ("given", None)
        self.port = self.sock.getsockname()[1]
        self.counts = Counter()
        self.session_counts = OrderedDict()     # session -> Counter(rx_chunks, tx_chunks, ...)
        self.rx = Reassembler(self.counts)
        self.slot = LatestSlot()
        self.boot_id = int.from_bytes(os.urandom(8), "little") if boot_id is None else int(boot_id)
        self.roi_core = roi_core
        self.regs = dict(regs or {})
        self.permit_feed = permit_feed
        self.geometry_desc = dict(geometry_desc or {"core": core_name, "wait": None})
        self._rec_lock = threading.Lock()
        self.last_geometry = None               # worker's latest frame record (TELEM geometry.last)
        self.last_roi = None
        self.roi_served = 0
        self.info = dict(info or {})
        features = dict(FEATURES)
        if ar is not None:                      # only then are the Nano's AGE/LABEL blocks wanted
            features["age_ext"] = 1
            features["label_ext"] = 1
            features["replay_ext"] = 1          # and the replay mark (display only)
            if getattr(ar, "colour", None) is not None:
                features["stamp_ext"] = 1       # AR colour bypass: the sensor key of each depth frame
        if fast is not None:                    # off: info keys exactly as before
            self.info["kv_fast"] = fast.describe()
        if rolling is not None:
            features["rolling"] = 1
            features["rolling_replay"] = 1      # ROLL mode 2 REPLAY (go-live 10-26, kv_rolling)
            if getattr(rolling, "grid_capable", False):
                features["rolling_grid"] = 1    # B1 11-02: ROLL v2 grid op (kv_rolling, NANO_SIDE.md)
        if nav is not None:
            features["nav"] = 1
        self.info.update(core=core_name, chunk_bytes=self.chunk_bytes, rcvbuf=self.rcvbuf,
                         features=features,
                         roi_core=roi_core.name if roi_core is not None else "none",
                         roi_max_boxes=ROI_MAX_BOXES, kv_boot_id="%016x" % self.boot_id,
                         nudge={"after_burst": bool(nudge_after_burst),
                                "idle_ms": float(nudge_idle_s or 0.0) * 1e3})
        if estop is None:
            from host.kv_estop import EstopLatch
            estop = EstopLatch(permit=self.regs.get("permit"), boot_id=self.boot_id)
        self.estop = estop
        if telem is None:
            from host.kv_telem import TelemetryResponder
            telem = TelemetryResponder(self)
        self.telem = telem
        self._stop = threading.Event()
        self._send_lock = threading.Lock()
        # Tail release (docs/design/KV_LINK_BURST_TAIL_20261011.md): the KV260 NIC can keep the
        # last frame it was handed until its next transmission. ``nudge_after_burst``: a NUDGE
        # right after every answer of more than one Ethernet frame; ``nudge_idle_s`` > 0: a
        # NUDGE once the server has sent nothing for that long after any answer (the NIC is
        # idle by then, so that NUDGE starts it and itself goes out). Either may be off.
        self.nudge_after_burst = bool(nudge_after_burst)
        self.nudge_idle_s = float(nudge_idle_s or 0.0)
        self._nudge_cond = threading.Condition()
        self._nudge_due = None                  # (monotonic due, addr, session) or None
        self._nudge_seq = 0
        self._depth = np.zeros(MAX_PIXELS, dtype=np.float32)
        self.last_activity = time.monotonic()
        # NN-CAM2 (2026-11-03): kv_server --frame-source kvcam puts frames from kv_cam's ring into ``slot`` with
        # addr None (host/kv_local_frames.py); _serve hands their answer to ``local_sink`` instead of a Nano.
        # ``dump_core_input`` (tests): directory for the exact geometry-core input of every frame. Both None =
        # exactly as before.
        self.local_sink = None
        self.dump_core_input = None
        self.worker_error = None
        self._reset_window()
        self._threads = []

    def _session(self, session):
        c = self.session_counts.get(session)
        if c is None:
            c = self.session_counts[session] = Counter()
            while len(self.session_counts) > 256:
                self.session_counts.popitem(last=False)
        return c

    def _reset_window(self):
        import resource
        self.window = {"t0": time.monotonic(), "ru": resource.getrusage(resource.RUSAGE_SELF),
                       "worker_cpu_s": 0.0, "worker_busy_s": 0.0, "snmp": read_snmp(),
                       "counts": Counter(self.counts), "frames": 0,
                       "sessions": {k: Counter(v) for k, v in self.session_counts.items()}}

    def window_stats(self, session=None):
        """``session_counts``: chunk/message counters of ``session`` alone since the
        window reset (chunk-loss arithmetic must use these: other sessions -- SYNC,
        TELEM, ESTOP clients -- share the server's global counters)."""
        import resource
        ru = resource.getrusage(resource.RUSAGE_SELF)
        w = self.window
        wall = time.monotonic() - w["t0"]
        cpu = (ru.ru_utime - w["ru"].ru_utime) + (ru.ru_stime - w["ru"].ru_stime)
        diff = Counter(self.counts)
        diff.subtract(w["counts"])
        out = {"wall_s": wall, "process_cpu_s": cpu,
               "process_cpu_user_s": ru.ru_utime - w["ru"].ru_utime,
               "process_cpu_sys_s": ru.ru_stime - w["ru"].ru_stime,
               "process_cpu_per_wall": cpu / wall if wall > 0 else None,
               "worker_cpu_s": w["worker_cpu_s"], "worker_busy_s": w["worker_busy_s"],
               "frames": w["frames"], "counts": {k: v for k, v in diff.items() if v},
               "slot": {"puts": self.slot.puts, "superseded": self.slot.superseded,
                        "takes": self.slot.takes},
               "rx_pending": self.rx.pending(),
               "snmp_delta": snmp_delta(w["snmp"], read_snmp()),
               "worker_error": self.worker_error, "info": self.info,
               "live": self.live_info() if self.live_info else None,
               "sessions_seen": len(self.session_counts)}
        if session is not None:
            sd = Counter(self.session_counts.get(session, {}))
            sd.subtract(w["sessions"].get(session, Counter()))
            out["session"] = "%016x" % session
            out["session_counts"] = {k: v for k, v in sd.items() if v}
        return out

    def _read_tx(self):
        try:
            with open(self._tx_path) as fp:
                return int(fp.read())
        except (OSError, ValueError):
            return None

    def _sendto(self, addr, msg_type, session, frame_id, payload, flags=0, chunk_bytes=None,
                log_extra=None):
        with self._send_lock:
            sc = self._session(session)
            chunk = self.chunk_bytes if chunk_bytes is None else chunk_bytes
            dgrams = split(msg_type, session, frame_id, payload, chunk, flags)
            frames = sum(wire_frames(len(d)) for d in dgrams)
            log = self.send_log is not None and frames > 1
            tx0 = self._read_tx() if log and self._tx_path else None
            t_begin = time.monotonic()
            # The socket is shared with _reader, whose receive poll timeout (0.2 s) used to bound these
            # sends too: a send buffer full for a moment made sendto raise socket.timeout here, which
            # killed the worker (or the reader) and kv_server exited -- the client-side twin was fixed
            # in KvLinkClient._send. The send gets its own bound and a send blocked that long drops this
            # message (the client times out on it, as on a lost datagram); the threads stay up. The
            # receive poll timeout is put back before _send_lock is released (every send holds it; a
            # recvfrom the reader enters meanwhile waits at most send_timeout_s: stop latency only).
            self.sock.settimeout(self.send_timeout_s)
            try:
                for n, d in enumerate(dgrams):
                    try:
                        self.sock.sendto(d, addr)
                    except socket.timeout:
                        self.counts["send_timeout"] += 1
                        self.counts["send_timeout:%s" % TYPE_NAMES[msg_type]] += 1
                        sc["send_timeout"] += 1
                        self.counts["send_timeout_dropped_chunks"] += len(dgrams) - n
                        return False
                    self.counts["tx_chunks"] += 1
                    sc["tx_chunks"] += 1
                    sc["tx_chunks:%s" % TYPE_NAMES[msg_type]] += 1
            finally:
                self.sock.settimeout(SERVER_RECV_POLL_S)
            t_end = time.monotonic()
            self.counts["tx_messages:%s" % TYPE_NAMES[msg_type]] += 1
            sc["tx_messages:%s" % TYPE_NAMES[msg_type]] += 1
            if frames > 1:
                self.counts["tx_multiframe:%s" % TYPE_NAMES[msg_type]] += 1
                if self.nudge_after_burst:
                    self._send_nudge_locked(addr, session, "burst")
            if self.nudge_idle_s > 0:
                with self._nudge_cond:
                    self._nudge_due = (time.monotonic() + self.nudge_idle_s, addr, session)
                    self._nudge_cond.notify()
            if log:
                rec = {"type": TYPE_NAMES[msg_type], "session": "%016x" % session,
                       "frame_id": int(frame_id), "bytes": len(payload), "datagrams": len(dgrams),
                       "wire_frames": frames, "t_send_begin": t_begin, "t_send_end": t_end,
                       "tx_before": tx0,
                       "tx_after": self._read_tx() if self._tx_path else None}
                rec.update(log_extra or {})
                try:
                    self.send_log(rec)
                except Exception:
                    self.counts["send_log_error"] += 1
            return True

    def _send_nudge_locked(self, addr, session, why):
        """One NUDGE datagram (caller holds _send_lock). Counted apart from the message
        chunks (tx_chunks:* is what chunk_loss() compares with the client's rx_chunks:*)."""
        self._nudge_seq += 1
        try:
            self.sock.sendto(CHUNK.pack(MAGIC, NUDGE, 0, 0, session, self._nudge_seq, 0, 0, 1), addr)
        except OSError:
            self.counts["nudge_error"] += 1
            return
        self.counts["tx_nudge:%s" % why] += 1

    def _nudger(self):
        while not self._stop.is_set():
            with self._nudge_cond:
                if self._nudge_due is None:
                    self._nudge_cond.wait(0.2)
                    continue
                due, addr, session = self._nudge_due
                left = due - time.monotonic()
                if left > 0:
                    self._nudge_cond.wait(left)
                    continue                    # re-read: a later send moves the due time
                self._nudge_due = None
            with self._send_lock:
                with self._nudge_cond:
                    if self._nudge_due is not None:
                        continue                # something was sent meanwhile: wait for its due
                self._send_nudge_locked(addr, session, "idle")

    def _control_answer(self, addr, msg_type, session, frame_id, payload, flags=0):
        self._sendto(addr, msg_type, session, frame_id, payload, flags,
                     chunk_bytes=min(self.chunk_bytes, CONTROL_CHUNK))

    def hello_body(self):
        """HELLO_ACK payload: HELLO_INFO_KEYS of ``info`` only (one Ethernet frame); the
        rest (argv, core_detail, registers, pl_timebase, permit_feed) is in STATS_ACK."""
        short = {k: self.info[k] for k in HELLO_INFO_KEYS if k in self.info}
        short["info_rest"] = "stats"
        return json.dumps(short, sort_keys=True).encode()

    def start(self):
        threads = [(self._reader, "kv-reader"), (self._worker, "kv-worker")]
        if self.nudge_idle_s > 0:
            threads.append((self._nudger, "kv-nudger"))
        for target, name in threads:
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)
        if hasattr(self.estop, "start"):
            self.estop.start()
        if self.nav is not None:
            self.nav.start()
        return self

    def stop(self):
        self._stop.set()
        self.slot.close()
        with self._nudge_cond:
            self._nudge_cond.notify_all()
        if hasattr(self.estop, "stop"):
            self.estop.stop()
        if self.nav is not None:
            self.nav.stop()
        for t in self._threads:
            t.join(timeout=5)
        if self._roi_pool is not None:
            self._roi_pool.shutdown(wait=True)
        self.sock.close()

    def _reader(self):
        self.sock.settimeout(SERVER_RECV_POLL_S)
        while not self._stop.is_set():
            try:
                dgram, addr = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                raise
            t2_ns = self.sync_responder.clock()  # SYNC receive stamp (RAW, same clock as t3)
            now = time.monotonic()
            self.last_activity = now
            if len(dgram) >= CHUNK.size and dgram[:4] == MAGIC:
                sc = self._session(CHUNK.unpack_from(dgram)[4])
                sc["rx_chunks"] += 1
                sc["rx_chunks:%s" % TYPE_NAMES.get(dgram[4], dgram[4])] += 1
            got = self.rx.feed(dgram, now)
            if got is None:
                continue
            mtype, flags, session, frame_id, payload, first_t = got
            self._session(session)["rx_messages:%s" % TYPE_NAMES.get(mtype, mtype)] += 1
            if mtype == HELLO:
                self._control_answer(addr, HELLO_ACK, session, frame_id, self.hello_body())
            elif mtype == SYNC:                 # inline on the reader, never via the slot/worker
                body = self.sync_responder.answer(payload, t2_ns)
                if body is not None:
                    self._control_answer(addr, SYNC_ACK, session, frame_id, body)
                else:
                    self.counts["sync_malformed"] += 1
            elif mtype == STATS:
                stats = self.window_stats(session)
                stats["total_counts"] = dict(self.counts)
                if self.rolling is not None:
                    try:
                        stats["rolling"] = self.rolling.describe()
                    except RuntimeError:        # counter changed size under the worker
                        stats["rolling"] = None
                if self.nav is not None:
                    stats["nav"] = self.nav.describe()
                if flags & FLAG_RESET:
                    self._reset_window()
                self._control_answer(addr, STATS_ACK, session, frame_id,
                                     json.dumps(stats, sort_keys=True, default=str).encode())
            elif mtype == TELEM_REQ:            # inline, like SYNC (A.3)
                try:
                    body = self.telem.answer(payload)
                except BadExtension as exc:
                    self.counts["bad_ext:%s" % exc.reason] += 1
                    continue
                self._control_answer(addr, TELEM_ACK, session, frame_id, body)
            elif mtype == ESTOP_CMD:            # inline (A.4); the ACK reuses this request's frame id
                try:
                    body = self.estop.handle(payload)
                except BadExtension as exc:
                    self.counts["bad_ext:%s" % exc.reason] += 1
                    continue
                self._control_answer(addr, ESTOP_ACK, session, frame_id, body)
            elif mtype == ROLL_REQ and self.rolling is not None:   # worker, in arrival order
                self.slot.put_side({"kind": "roll", "addr": addr, "session": session,
                                    "frame_id": frame_id, "payload": payload})
            elif mtype in (NAV_GOAL, NAV_ODOM, NAV_PATH) and self.nav is not None:   # inline, like TELEM
                try:
                    body = self.nav.handle(mtype, payload)
                except BadExtension as exc:
                    self.counts["bad_ext:%s" % exc.reason] += 1
                    continue
                self._control_answer(addr, NAV_CMD, session, frame_id, body)
            elif mtype == REQ:
                old = self.slot.put({"addr": addr, "session": session, "frame_id": frame_id,
                                     "payload": payload, "first_t": first_t, "done_t": now})
                if old is not None and old.get("addr") is None:   # NN-CAM2: a local frame; nobody to notify
                    self.counts["slot_superseded_local"] += 1
                    old = None
                if old is not None:
                    self.counts["slot_superseded"] += 1
                    try:
                        cap = REQ_HEAD.unpack_from(old["payload"])[1]
                    except struct.error:
                        cap = 0
                    self._sendto(old["addr"], SUPERSEDED, old["session"], old["frame_id"],
                                 NOTICE.pack(old["frame_id"], cap))
            else:
                self.counts["unexpected:%s" % TYPE_NAMES.get(mtype, mtype)] += 1

    def _worker(self):
        try:
            import resource
            thread_ru = getattr(resource, "RUSAGE_THREAD", None)
            while not self._stop.is_set():
                item = self.slot.take(timeout=0.2)
                if item is None:
                    continue
                if item.get("kind") == "roll":
                    self._serve_roll(item)
                    continue
                c0 = resource.getrusage(thread_ru) if thread_ru is not None else None
                b0 = time.monotonic()
                self._serve(item, b0)
                self.window["worker_busy_s"] += time.monotonic() - b0
                if c0 is not None:
                    c1 = resource.getrusage(thread_ru)
                    self.window["worker_cpu_s"] += ((c1.ru_utime - c0.ru_utime) +
                                                    (c1.ru_stime - c0.ru_stime))
                self.window["frames"] += 1
        except Exception as exc:   # surfaced through stats; the Nano sees timeouts
            self.worker_error = repr(exc)
            raise

    def _serve_roll(self, item):
        c0 = time.thread_time_ns()
        body = self.rolling.handle(item["payload"])
        self.window["roll_worker_cpu_s"] = (self.window.get("roll_worker_cpu_s", 0.0) +
                                            (time.thread_time_ns() - c0) / 1e9)
        if body is None:
            self.counts["roll_bad_request"] += 1
            return
        self.counts["roll_served"] += 1
        self._control_answer(item["addr"], ROLL_ACK, item["session"], item["frame_id"], body)

    def _fast_crc(self, codes):
        crc = self.fast.crc32(codes)
        return crc32(codes) if crc is None else crc

    def _permit_timebase(self):
        blk = self.regs.get("permit")
        if blk is None or not blk.available:
            return None
        try:
            return blk.timebase()
        except Exception:
            self.counts["permit_read_error"] += 1
            return None

    def _roi_invoke(self, req, depth, width, height):
        """One ROI core call -> (out, None) or (None, exception); never raises (worker thread safe)."""
        try:
            return self.roi_core(depth[:width * height].reshape(height, width), req["roi"]), None
        except Exception as exc:
            return None, exc

    def _roi_convert_invoke(self, req, depth, width, height):
        """D16 + --roi-concurrent 1: the ROI core's float32 depth, made in the ROI worker (overlapping the
        PL geometry run), then the ROI call; never raises."""
        try:
            depth[:width * height] = self.convert(req["codes"], bits_f32(req["unit_bits"]))
        except Exception as exc:
            return None, exc
        return self._roi_invoke(req, depth, width, height)

    def _run_roi(self, req, depth, width, height, geometry_status, early=None):
        """``early`` (--roi-concurrent 1): the future of the ROI call started together with the geometry
        core; its result is used exactly as the serial call's would be (dropped when the geometry failed)."""
        roi = req["roi"]
        rec = {"frame_id": req["frame_id"], "memory_revision": roi["memory_revision"]}
        times, cycles, rows = {}, None, []
        if self.roi_core is None:
            status, core = ROI_NOT_CONFIGURED, "none"
        elif geometry_status != 0:
            status, core = ROI_GEOMETRY_FAILED, self.roi_core.name
            if early is not None:
                early.result()
                self.counts["roi_concurrent_discarded"] += 1
        else:
            core = self.roi_core.name
            out, exc = early.result() if early is not None else self._roi_invoke(req, depth, width, height)
            if exc is None:
                status, rows, times, cycles = out["status"], out["rows"], out["times_ns"], out["cycles"]
            else:                           # a ROI device error never touches the geometry
                self.counts["roi_error:%s" % type(exc).__name__] += 1
                status, rows = ROI_DEVICE_ERROR, []
        verdicts = Counter(r[6] for r in rows) if status == ROI_OK else Counter()
        rec.update(roi_status=status, rows=len(rows) if status == ROI_OK else 0,
                   verdict_counts={str(k): verdicts.get(k, 0) for k in (0, 2, 3)},
                   times_ns={k: int(times.get(k, 0)) for k in ROI_TIME_FIELDS},
                   roi_cycles=cycles, kv_done_ns=now_ns(), core=core)
        with self._rec_lock:
            self.last_roi = rec
            self.roi_served += 1
        self._roi_answer = (status, rows if status == ROI_OK else [])
        self.counts["roi_status:%#x" % status] += 1
        return encode_roi_result(roi_core=core, roi_status=status,
                                 memory_revision=roi["memory_revision"], times_ns=times,
                                 roi_cycles=cycles, rows=rows)

    def _serve(self, item, t_take):
        times = {"recv": int((item["done_t"] - item["first_t"]) * 1e9),
                 "slot_wait": int((t_take - item["done_t"]) * 1e9)}
        fast = self.fast
        if fast is not None and self.convert is not codes_to_m:
            fast = None                 # the convert hook is replaced (crc positive control): Python path
            self.counts["kv_fast_bypass:convert_hook"] += 1
        t0 = time.monotonic()
        try:
            req = decode_request(item["payload"], allow_age=self.ar is not None,
                                 allow_label=self.ar is not None, allow_replay=self.ar is not None,
                                 allow_stamp=getattr(self.ar, "colour", None) is not None,
                                 crc_fn=None if fast is None else self._fast_crc)
        except BadExtension as exc:     # the whole request is void (A.1 rule 4)
            self.counts["bad_ext:%s" % exc.reason] += 1
            return
        except (ValueError, struct.error, UnicodeDecodeError) as exc:
            self.counts["bad_request:%s" % type(exc).__name__] += 1
            return                      # the Nano times out: a malformed frame is a failed frame
        t1 = time.monotonic()
        if req["sensor_stamp_ns"] is not None:  # AR colour bypass: one small WANT datagram, never blocks
            try:
                self.ar.early(req)
            except Exception as exc:
                self.counts["ar_early_error:%s" % type(exc).__name__] += 1
        pixels = req["width"] * req["height"]
        if pixels > MAX_PIXELS or req["nx"] * req["ny"] * req["nz"] > GRID_CAPACITY:
            self.counts["bad_request:size"] += 1
            return
        depth = self._depth
        use16 = self.core_call_codes is not None and _d16_unit_ok(req["unit_bits"])
        roi_float = req["roi"] is not None and self.roi_core is not None
        if use16:                       # D16: no per-pixel A53 work for the geometry core
            self.counts["d16_frames"] += 1
            t2 = t3 = time.monotonic()
            depth_crc = None            # from the core
        else:
            if self.core_call_codes is not None:
                self.counts["d16_unit_fallback"] += 1
            depth_crc = None
            if fast is not None:        # one C++ pass: convert + crc (times: convert = both, crc = 0)
                depth_crc = fast.convert_crc(req["codes"], req["unit_bits"], depth)
            if depth_crc is None:
                depth[:pixels] = self.convert(req["codes"], bits_f32(req["unit_bits"]))
                t2 = time.monotonic()
                depth_crc = crc32(depth[:pixels])
                t3 = time.monotonic()
            else:
                t2 = t3 = time.monotonic()
        flags = np.full(GRID_CAPACITY, 99, dtype=np.uint8)
        counts = np.full(3, 99, dtype=np.uint32)
        status = np.full(1, 99, dtype=np.uint32)
        if self.permit_feed is not None:   # right before the core start: NEXT_FRAME_ID
            adm = self.permit_feed.admit(req["identity"]["stream_instance"], req["frame_id"])
            self.counts["permit_admit:%s" % adm.verdict] += 1
            if not adm.run:
                self.counts["permit_drop:%s" % adm.verdict] += 1
                return
        tb0 = self._permit_timebase()
        if self.rolling is not None:    # the frame buffer is about to change: nothing is held
            self.rolling.hold(req["frame_id"], 1, flags, 0)
        early = None
        if self._roi_pool is not None and req["roi"] is not None and self.roi_core is not None:
            if use16:                   # D16: the ROI worker makes its float32 depth while the PL runs
                early = self._roi_pool.submit(self._roi_convert_invoke, req, depth, req["width"],
                                              req["height"])
            else:
                early = self._roi_pool.submit(self._roi_invoke, req, depth, req["width"], req["height"])
            self.counts["roi_concurrent_started"] += 1
        if self.dump_core_input is not None:   # NN-CAM2 tests only
            from host.kv_local_frames import dump_core_input
            dump_core_input(self.dump_core_input, req, depth, use16, pixels)
        try:
            geo = (np.ascontiguousarray(req["inverse_k"], dtype=np.float32),
                   np.ascontiguousarray(req["world_camera"], dtype=np.float32),
                   np.ascontiguousarray(req["origin"], dtype=np.float64),
                   float(req["voxel"]), req["width"], req["height"],
                   req["nx"], req["ny"], req["nz"], flags, counts, status)
            if use16:
                depth_crc = int(self.core_call_codes(req["codes"], req["unit_bits"], *geo))
            else:
                self.core_call(depth, *geo)
        except Exception as exc:        # a device error fails this frame only
            self.counts["core_error:%s" % type(exc).__name__] += 1
            if early is not None:       # the depth buffer is reused by the next frame: wait for the ROI
                early.result()
            return
        t4 = time.monotonic()
        if use16 and int(status[0]) != 0:   # D16: the core read no depth; the old reply's crc, A53-made
            self.counts["d16_crc_on_a53"] += 1
            depth_crc = crc32(self.convert(req["codes"], bits_f32(req["unit_bits"])))
        if use16 and roi_float and early is None and int(status[0]) == 0:
            depth[:pixels] = self.convert(req["codes"], bits_f32(req["unit_bits"]))   # serial ROI below
        times.update(decode=int((t1 - t0) * 1e9), convert=int((t2 - t1) * 1e9),
                     crc=int((t3 - t2) * 1e9), core_total=int((t4 - t3) * 1e9))
        last = getattr(self.timing_source, "last_times", None)
        if last:
            times.update(core_prep=int(last["prep_s"] * 1e9),
                         core_device=int(last["device_s"] * 1e9),
                         core_readback=int(last["readback_s"] * 1e9))
        pl_cycles = None
        if tb0 is not None:             # same PL counter, single-clock difference (A.3)
            try:
                done = self.regs["permit"].permit_snapshot()["last_done_ts"]
                pl_cycles = done - tb0 if done > tb0 else None
            except Exception:
                self.counts["permit_read_error"] += 1
        nvox = req["nx"] * req["ny"] * req["nz"]
        if self.rolling is not None:
            self.rolling.hold(req["frame_id"], int(status[0]), flags, nvox)
        self.counts["served"] += 1
        with self._rec_lock:
            self.last_geometry = {
                "frame_id": req["frame_id"], "capture_ns": req["capture_ns"],
                "status": int(status[0]), "counts": [int(c) for c in counts],
                "times_ns": {k: int(times.get(k, 0)) for k in TIMING_FIELDS},
                "pl_cycles": pl_cycles, "kv_done_ns": now_ns()}
            if use16:                   # GEO_FX3: PL depth statistics [ST_NPOS, ST_LE20, ST_MM0, ST_MM1] (None:
                                        # the CPU reference ran); the Nano side does not read them yet
                self.last_geometry["fx_stats"] = getattr(self.timing_source, "last_stats", None)
        roi_block, rflags = b"", 0
        self._roi_answer = (None, [])
        if req["roi"] is not None:
            roi_block = self._run_roi(req, depth, req["width"], req["height"], int(status[0]),
                                      **({"early": early} if early is not None else {}))
            rflags = RESP_FLAG_ROI
        body = encode_response(req, status=status[0], counts=counts, depth_crc=depth_crc,
                               flags=flags[:nvox], times_ns=times,
                               kv_superseded=self.counts["slot_superseded"],
                               kv_incomplete=self.counts["rx_incomplete_superseded"],
                               core_name=self.core_name, roi_block=roi_block)
        t5 = time.monotonic()
        if item.get("addr") is None:    # NN-CAM2 local frame (kv_cam ring): no Nano to answer
            self.counts["local_served"] += 1
            if self.local_sink is not None:
                try:
                    self.local_sink(item, req, int(status[0]), flags, nvox, times, t_take, t3, t4)
                except Exception as exc:    # counted; the next frame goes on
                    self.counts["local_sink_error:%s" % type(exc).__name__] += 1
            return
        self._sendto(item["addr"], RESP, item["session"], item["frame_id"], body, rflags,
                     log_extra={"t_first_chunk": item["first_t"], "t_recv_done": item["done_t"],
                                "t_take": t_take, "t_core_begin": t3, "t_core_end": t4,
                                "t_encoded": t5} if self.send_log is not None else None)
        if self.ar is not None:         # after the answer left: AR never delays the geometry
            roi_status, roi_rows = self._roi_answer
            self.ar.submit({
                "frame_id": req["frame_id"], "codes": req["codes"], "unit_bits": req["unit_bits"],
                "width": req["width"], "height": req["height"], "nx": req["nx"],
                "ny": req["ny"], "nz": req["nz"], "origin": req["origin"], "voxel": req["voxel"],
                "world_camera": req["world_camera"], "flags": flags[:nvox],
                "geometry_status": int(status[0]), "roi": req["roi"], "roi_status": roi_status,
                "roi_rows": roi_rows, "nano_elapsed_ns": req["nano_elapsed_ns"],
                "labels": req["labels"], "replay": req["replay"],
                "first_mono_ns": int(item["first_t"] * 1e9), "capture_ns": req["capture_ns"],
                "sensor_stamp_ns": req["sensor_stamp_ns"]})


# ------------------------------------------------------------------ Nano-side clients (A.3 / A.4)
class _ExtClient:
    def __init__(self, server, *, feature, hello_timeout_s=1.0, session=None, sock=None,
                 chunk_bytes=CHUNK_FRAGMENTED):
        self.link = KvLinkClient(server, chunk_bytes=chunk_bytes, session=session, sock=sock,
                                 rcvbuf=1 << 20)
        self.counts = self.link.counts
        self.feature = feature
        self.hello_timeout_s = float(hello_timeout_s)

    def _ready(self):
        return self.link.require(self.feature, self.hello_timeout_s)

    def close(self):
        self.link.close()


class KvTelemetryClient(_ExtClient):
    """``poll(want)`` -> host.kv_status.Telemetry (one TELEM_REQ, new chunk frame id each
    time). Raises LinkTimeout (counted ``telem_timeout``) when no answer in
    ``timeout_s``; KvFeatureUnavailable (nothing sent) against a server without TELEM."""

    def __init__(self, server, *, timeout_s=0.2, resend_s=TELEM_RESEND_S, **kw):
        super().__init__(server, feature="telem", **kw)
        self.timeout_s = float(timeout_s)
        self.resend_s = float(resend_s)

    def poll(self, want=TELEM_ALL):
        from host.kv_status import decode_telemetry
        self._ready()
        t_req = now_ns()
        # resent (new chunk frame id) after 50, 100 ms ... within the timeout: TELEM is
        # read-only, every resend carries the same t_req, any answer of the round is taken
        payload = self.link._control_resend(TELEM_REQ, TELEM_ACK, encode_telem_req(want, t_req),
                                            self.timeout_s, self.resend_s)
        t_recv = now_ns()
        if payload is None:
            self.counts["telem_timeout"] += 1
            raise LinkTimeout("no TELEM answer within %.3f s" % self.timeout_s)
        try:
            ack = decode_telem_ack(payload)
        except BadExtension as exc:
            self.counts["bad_ext:%s" % exc.reason] += 1
            raise LinkError("TELEM answer void: %s" % exc)
        if ack["t_req_nano_ns"] != t_req:
            self.counts["telem_stale"] += 1
            raise LinkError("TELEM answer echoes another request")
        self.counts["telem_ok"] += 1
        return decode_telemetry(ack, t_req, t_recv)


class KvEstopClient(_ExtClient):
    """``send(op, cmd_id, epoch, timeout_s)`` -> EstopAck; one attempt under a new chunk
    frame id (the 200 ms resend loop with the same cmd_id is the caller's). Raises
    LinkTimeout when unanswered, KvFeatureUnavailable (nothing sent) without ESTOP."""

    def __init__(self, server, **kw):
        super().__init__(server, feature="estop", **kw)

    def send(self, op, cmd_id, epoch=0, timeout_s=0.2, _frame_id=None):
        if op not in ESTOP_OPS:
            raise ValueError("op must be one of %s" % sorted(ESTOP_OPS))
        self._ready()
        body = encode_estop_cmd(ESTOP_OPS[op], cmd_id, epoch, now_ns())
        payload = self.link._control_raw(ESTOP_CMD, ESTOP_ACK, body, timeout_s, frame_id=_frame_id)
        if payload is None:
            self.counts["estop_timeout"] += 1
            raise LinkTimeout("no ESTOP answer within %.3f s" % timeout_s)
        try:
            a = decode_estop_ack(payload)
        except BadExtension as exc:
            self.counts["bad_ext:%s" % exc.reason] += 1
            raise LinkError("ESTOP answer void: %s" % exc)
        if a["cmd_id"] != int(cmd_id):
            self.counts["estop_foreign_ack"] += 1
            raise LinkError("ESTOP answer for cmd %d, sent %d" % (a["cmd_id"], cmd_id))
        return EstopAck(a["result"], a["latched"], a["epoch"], a["force_stop_written"],
                        a["permit_avail"], a["permit_status"], a["kv_boot_id"])


EstopAck = namedtuple("EstopAck", "result latched epoch force_stop_written permit_avail "
                                  "permit_status kv_boot_id")
