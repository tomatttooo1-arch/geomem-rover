#!/usr/bin/env python3
"""GeoMem bridge: browser (protocol v2, WebSocket) <-> car hub (FPGAC via pc/fpgac.py).

    python ui/geomem/bridge/geomem_bridge.py --hub <car IP>:47431 --token-file <token file>
        [--map-id <sha256> --map-name 实验室] [--basemap <底图包目录> ...] [--basemap-watch <目录>]
        [--listen 127.0.0.1:8788]
        [--dump-cloud DIR] [--stats-out FILE]

Pages: / (console), /kiosk.html (exhibition page, read-only; 10-17). Base-map packages (§2.10) are
served from memory at /basemap/<library sha256>/{basemap.json,cells.bin}. 10-18: drawn on the odometry tier too
(``basis: mapping_start``), and packages that appear later under ``--basemap-watch DIR`` are loaded without a
restart (polled every 2 s, see gmb_basemap.PackageDir).

Design: ui/geomem/docs/PROTOCOL_V2_DESIGN.md §2-§3. Python 3.8+ standard library only (the laptop
may be Windows); imports ``pc.fpgac`` from the repository, never ``host.*``. Listens on 127.0.0.1 only.
Threads: FPGAC client (its own), HTTP/WS server (one per browser connection, plus one sender each),
one 5 Hz composer, one command worker.
"""

import argparse
import binascii
import collections
import json
import os
import queue
import signal
import socket
import struct
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from socketserver import ThreadingMixIn

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]                       # <repo>/ui/geomem/bridge -> <repo>
APP_ROOT = HERE.parent / "app"
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import gmb_basemap as gbm  # noqa: E402
import gmb_proto as gp  # noqa: E402
import gmb_ws as ws  # noqa: E402

TICK_S = 0.2                                 # 5 Hz snapshots (§2.3)
BASEMAP_POLL_S = 2.0                         # --basemap-watch poll period (10-18)
ROLLING_MIN_S = 0.5                          # rolling-grid cloud at most 2 Hz (§3.3)
NO_ACK_GRACE_S = 0.5                         # deadline + 0.5 s without a hub CMD_ACK -> expired
RESULT_WAIT_S = 3.0                          # CMD_ACK + 3 s without CMD_RESULT -> result unknown
MAX_SESSIONS = 8
# 10-21 first-person preview pushed over the WebSocket (was: browser polled /preview.jpg, <=2 fps 320x240)
PREVIEW_MAX_W, PREVIEW_MAX_H = 640, 480      # == host/hub/hub_core.py PREVIEW_MAX_*
PREVIEW_MAX_BYTES = 160 * 1024
PREVIEW_STALE_S = 1.5
# 10-21 RTAB-Map display cloud relayed from the Nano feed (ui/geomem/tools/competition/live_mapping.py):
# pushed only when its sha256 changes (compact q5z encoding), a header-only keep-alive otherwise.
# 10-21 frame-level flow control: at most one bulk frame (preview / rolling cloud / mapping) in flight per
# browser; the page acks each on receipt ({"type":"frame_ack"}). Without it frames queued in the ssh/TCP
# send buffers on a slow WiFi link and the picture (and the snapshots behind it) lagged by seconds.
BULK_ACK_TIMEOUT_S = 1.5
FRAME_ACK_PREFIX = '{"type":"frame_ack"'
MAPPING_POLL_S = 1.0
MAPPING_MIN_PUSH_S = 2.0
MAPPING_KEEPALIVE_S = 2.0
MAPPING_MAX_BYTES = 45 * 1024 * 1024
# 10-22 Nano gateway (--gateway): a 1.4 MB mapping frame still sat in front of every snapshot in the one ordered
# TCP stream (WS over ssh over WiFi) for >2 s. Every bulk payload over CHUNK_BYTES now goes as separate binary WS
# messages {type:'chunk', id, i, n, kind} (never WS continuation frames: nothing could be sent between those),
# acked one by one, so at most one 64 KB chunk is ahead of a snapshot. Text always goes first; bulk kinds take
# turns chunk by chunk (BULK_ROUND_ROBIN) so neither the preview nor the map starves the other.
CHUNK_BYTES = 64 * 1024
BULK_KINDS = ("preview", "cloud", "mapping")
BULK_ROUND_ROBIN = True
BULK_WINDOW = 2                                    # unacked bulk frames/chunks per page (1 = stop-and-wait)
MAPPING_RESYNC_PREFIX = '{"type":"mapping_resync"'
# voxel deltas from a /delta-capable feed: each page tracks (epoch, seq); poll twice a second, push a page at most
# once a second and only when it is not still receiving the previous mapping message (natural back-pressure)
MAPPING_DELTA_POLL_S = 0.5
MAPPING_DELTA_MIN_PUSH_S = 1.0
GATEWAY_INFO = {"kind": "nano", "rover": False, "chunks": True}

CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
    ".mjs": "text/javascript; charset=utf-8", ".css": "text/css; charset=utf-8",
    ".json": "application/json", ".webmanifest": "application/manifest+json",
    ".svg": "image/svg+xml", ".png": "image/png", ".ico": "image/x-icon",
    ".txt": "text/plain; charset=utf-8", ".woff2": "font/woff2",
}


def _text_frame(obj):
    return ws.encode_frame(ws.OP_TEXT, gp.dumps(obj).encode("utf-8"))


def encode_binary(header, payload=b""):
    """``u32 L | L bytes JSON header | zero pad to 4 | payload`` -- the cloud layout (§2.4); the header's
    ``type`` tells the page what follows (cloud / preview / mapping)."""
    head = gp.dumps(header).encode("utf-8")
    pad = (-(4 + len(head))) % 4
    return b"".join((struct.pack("<I", len(head)), head, b"\x00" * pad, payload))


# ---------------------------------------------------------------------------- one browser connection

class WsSession(object):
    """Sender slots: ack/result/pong FIFO first, then the newest hello, cloud and snapshot."""

    def __init__(self, sock, rfile, bridge, max_message=ws.DEFAULT_MAX_MESSAGE, chunked=False):
        self.sock, self.rfile, self.bridge = sock, rfile, bridge
        self.max_message = max_message
        self.chunked = chunked                     # split bulk payloads into acked 64 KB messages (10-22)
        self.streams = dict((k, collections.deque()) for k in BULK_KINDS)   # encoded WS frames still to send
        self.chunk_id = 0
        self.last_bulk = None
        # voxel-delta mapping (10-22): what this page will hold once the queued message is through
        self.mapping_have = None                   # (epoch, seq) or None -> next push is a full
        self.mapping_push_t = -1e9
        self.mapping_keep_t = -1e9
        self.cv = threading.Condition()
        self.fifo = collections.deque()
        self.hello = self.cloud = self.snapshot = self.preview = self.mapping = None
        self.belief = None                         # 10-26 search heat map (text, newest only)
        self.seq = 0
        self.closed = False
        self.send_lock = threading.Lock()
        self.sender = None
        self.sent = collections.Counter()
        self.bulk_inflight = collections.deque()   # send times of unacked bulk frames (<= BULK_WINDOW)

    # -- producers (any thread) --
    def _put(self, attr, value):
        with self.cv:
            if self.closed:
                return
            if attr == "fifo":
                self.fifo.append(value)
            else:
                setattr(self, attr, value)
            self.cv.notify()

    def put_json(self, obj):
        self._put("fifo", ("json", obj))

    def put_hello(self, obj):
        self._put("hello", obj)

    def put_cloud(self, data):
        self._put("cloud", data)

    def put_snapshot(self, obj):
        self._put("snapshot", obj)

    def put_belief(self, obj):
        """10-26 search heat map: newest only (a slow page skips intermediate maps)."""
        self._put("belief", obj)

    def put_preview(self, data):
        """Latest-only: a slow browser skips frames instead of queueing them (no growing delay)."""
        self._put("preview", data)

    def put_mapping(self, data, have=None, abort=False, now=None):
        """Legacy: newest frame wins. Delta mode passes ``have`` (the page's (epoch, seq) after this message);
        ``abort`` drops the unsent chunks of the previous mapping message (a new epoch's full replaces it)."""
        with self.cv:
            if self.closed:
                return
            if abort and (self.streams["mapping"] or self.mapping is not None):
                self.streams["mapping"].clear()
                self.bridge.count("mapping_aborted")
            self.mapping = data
            if have is not None:
                self.mapping_have = have
                self.mapping_push_t = now if now is not None else time.monotonic()
            self.cv.notify()

    def mapping_idle(self):
        with self.cv:
            return self.mapping is None and not self.streams["mapping"]

    def on_mapping_resync(self):
        """The page lost its voxel state (cleared, base mismatch, integrity check): next push is a full."""
        with self.cv:
            self.mapping_have = None
            self.mapping_push_t = -1e9

    # -- sender thread --
    def start_sender(self):
        self.sender = threading.Thread(target=self._send_loop, name="gmb-ws-send", daemon=True)
        self.sender.start()

    @property
    def bulk_sent_t(self):
        """Send time of the oldest unacked bulk frame, or None."""
        return self.bulk_inflight[0] if self.bulk_inflight else None

    @bulk_sent_t.setter
    def bulk_sent_t(self, value):
        if value is None:
            self.bulk_inflight.clear()
        elif self.bulk_inflight:
            self.bulk_inflight[0] = value
        else:
            self.bulk_inflight.append(value)

    def on_frame_ack(self):
        with self.cv:
            if self.bulk_inflight:
                self.bulk_inflight.popleft()
            self.cv.notify()

    def _bulk_wait(self, now):
        """0 when a bulk frame may go now, else seconds until the in-flight one times out."""
        if len(self.bulk_inflight) < BULK_WINDOW:
            return 0.0
        left = self.bulk_inflight[0] + BULK_ACK_TIMEOUT_S - now
        if left <= 0:
            self.bridge.count("frame_ack_timeout")
            self.bulk_inflight.clear()
            return 0.0
        return left

    def _waiting(self, kind):
        return bool(self.streams[kind]) or getattr(self, kind) is not None

    def _bulk_frame(self):
        """Next bulk WS frame (cv held): kinds by priority preview > cloud > mapping, rotated so the kind just sent
        goes last when another one waits (BULK_ROUND_ROBIN). A payload is cut into chunks when taken; a kind's
        chunks are sent in order before its next payload (the newest preview/cloud waits in its slot)."""
        waiting = [k for k in BULK_KINDS if self._waiting(k)]
        if not waiting:
            return None, None
        kind = waiting[0]
        if BULK_ROUND_ROBIN and len(waiting) > 1 and kind == self.last_bulk:
            kind = waiting[1]
        q = self.streams[kind]
        if not q:
            data = getattr(self, kind)
            setattr(self, kind, None)
            if self.chunked and len(data) > CHUNK_BYTES:
                self.chunk_id += 1
                n = (len(data) + CHUNK_BYTES - 1) // CHUNK_BYTES
                for i in range(n):
                    head = {"type": "chunk", "version": gp.VERSION, "id": self.chunk_id, "i": i, "n": n,
                            "kind": kind, "bytes": len(data)}
                    q.append(ws.encode_frame(ws.OP_BINARY,
                                             encode_binary(head, data[i * CHUNK_BYTES:(i + 1) * CHUNK_BYTES])))
            else:
                q.append(ws.encode_frame(ws.OP_BINARY, data))
        self.last_bulk = kind
        return kind, q.popleft()

    def _next_batch(self):
        with self.cv:
            while not self.closed:
                if self.fifo or self.hello or self.snapshot or self.belief:
                    break
                bulk = any(self._waiting(k) for k in BULK_KINDS)
                wait = self._bulk_wait(time.monotonic()) if bulk else 1.0
                if bulk and wait == 0.0:
                    break
                self.cv.wait(min(wait, 1.0))
            if self.closed:
                return None
            out = []
            while self.fifo:
                kind, v = self.fifo.popleft()
                out.append(("fifo", _text_frame(v) if kind == "json" else v))
            if self.hello is not None:
                out.append(("hello", _text_frame(self.hello)))
                self.hello = None
            if self.snapshot is not None:
                self.seq += 1
                snap = {"type": "snapshot", "version": gp.VERSION, "seq": self.seq}
                snap.update(self.snapshot)
                snap["seq"] = self.seq
                out.append(("snapshot", _text_frame(snap)))
                self.snapshot = None
            if self.belief is not None:
                out.append(("belief", _text_frame(self.belief)))
                self.belief = None
            if self._bulk_wait(time.monotonic()) == 0.0:   # one bulk frame (or chunk) in flight
                kind, raw = self._bulk_frame()
                if raw is not None:
                    out.append((kind, raw))
                    self.bulk_inflight.append(time.monotonic())
            return out

    def _send_loop(self):
        while True:
            batch = self._next_batch()
            if batch is None:
                return
            for kind, raw in batch:
                if not self._send_raw(raw):
                    self.close()
                    return
                self.sent[kind] += 1

    def _send_raw(self, raw, lock_timeout=None):
        got = self.send_lock.acquire() if lock_timeout is None else self.send_lock.acquire(timeout=lock_timeout)
        if not got:
            return False
        try:
            self.sock.sendall(raw)
            return True
        except (OSError, socket.timeout):
            return False
        finally:
            self.send_lock.release()

    # -- reader (the HTTP handler thread) --
    def run_reader(self):
        dec = ws.FrameDecoder(max_message=self.max_message)
        while not self.closed:
            try:
                data = self.rfile.read1(65536)
            except (OSError, ValueError, socket.timeout):
                break
            if not data:
                break
            try:
                events = dec.feed(data)
            except ws.WsError as exc:
                self.bridge.count("ws_protocol_close:%d" % exc.code)
                self.close(exc.code, exc.why[:100])
                return
            for kind, value in events:
                if kind == "text" and value.startswith(FRAME_ACK_PREFIX):
                    self.on_frame_ack()
                elif kind == "text" and value.startswith(MAPPING_RESYNC_PREFIX):
                    self.bridge.count("mapping_resync")
                    self.on_mapping_resync()
                elif kind == "text":
                    self.bridge.submit(self, value)
                elif kind == "binary":
                    self.close(ws.CLOSE_UNSUPPORTED, "binary not accepted")
                    return
                elif kind == "ping":
                    self._put("fifo", ("raw", ws.encode_frame(ws.OP_PONG, value)))
                elif kind == "close":
                    code = value[0] if value[0] is not None else ws.CLOSE_NORMAL
                    self.close(code, "")
                    return
        self.close()

    def close(self, code=None, reason=""):
        with self.cv:
            if self.closed:
                return
            self.closed = True
            self.cv.notify_all()
        if code is not None:
            self._send_raw(ws.encode_frame(ws.OP_CLOSE, ws.close_payload(code, reason)), lock_timeout=1.0)
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

    def join(self, timeout=2.0):
        if self.sender is not None:
            self.sender.join(timeout)


# ---------------------------------------------------------------------------- bridge core

HEALTH_LEVELS = ("red", "yellow", "green", "pending")


def compose_health(h):
    """10-29: hub STATE "health" -> snapshot "health" {overall, summary, checks: [{code, level, title, detail, hint}]}
    (strings capped, at most 16 checks); None when absent or malformed."""
    if not isinstance(h, dict) or h.get("overall") not in HEALTH_LEVELS:
        return None
    checks = []
    for c in h.get("checks") or []:
        if not isinstance(c, dict) or c.get("level") not in HEALTH_LEVELS or not isinstance(c.get("title"), str):
            continue
        row = {"code": str(c.get("code") or "")[:30], "level": c["level"], "title": c["title"][:20],
               "detail": str(c.get("detail") or "")[:160]}
        if isinstance(c.get("hint"), str) and c["hint"]:
            row["hint"] = c["hint"][:120]
        checks.append(row)
        if len(checks) >= 16:
            break
    return {"overall": h["overall"], "summary": str(h.get("summary") or "")[:200], "checks": checks}


class Bridge(object):
    """Holds the newest hub data, composes protocol v2 messages, routes commands.

    ``client`` is a ``fpgac.FpgacClient`` (or a test double with the same attributes:
    ``client_id, link_state, hub_boot_id, hello_ack, runtime_info.value, command()``)."""

    def __init__(self, map_id=None, map_name=None, clock=time.monotonic, dump_cloud=None, bridge_id=None,
                 log=None, basemaps=None, basemap_dir=None, mapping_feed=None, gateway=False):
        self.map_id, self.map_name = map_id, map_name
        self.gateway = gateway                     # Nano direct gateway: no rover agent, chunked bulk (10-22)
        # {library sha256: package}. Never mutated in place: the watcher swaps in a new dict (one reference
        # assignment), so tick() / the HTTP handlers read one consistent dict without a lock (10-18).
        self.basemaps = gbm.index_packages(basemaps or [])
        self.basemap_dir = gbm.PackageDir(basemap_dir, log=log) if basemap_dir else None
        self._basemap_lock = threading.Lock()
        self.clock = clock
        self.dump_cloud = dump_cloud
        self.bridge_id = bridge_id or binascii.hexlify(os.urandom(8)).decode()
        self.log = log or (lambda msg: None)
        self.client = None
        self.lock = threading.RLock()            # hub data + sessions + broadcast order
        self.cmd_lock = threading.Lock()         # pending commands (shared with on_frame acks)
        self.state = self.state_t = None
        self.preview = None
        self.preview_seq = 0
        self.mapping_feed = mapping_feed           # (host, port) of the Nano display feed, or None
        self.mapping_frame = None                  # last full mapping binary (sent to new sessions)
        self.mapping_sha = None
        self.memory = self.memory_t = None
        self.raw_maps = {"global": None, "rolling": None}
        self.belief = None                         # 10-26 (browser message, receive time) of the newest heat map
        self._grids = {"global": (None, None), "rolling": (None, None)}
        self.controller = False
        self.tracker = gp.TaskIdTracker()
        self.pending = collections.OrderedDict()
        self.sessions = set()
        self.counts = collections.Counter()
        self.issues_last, self.issues_max = {}, collections.Counter()
        self.cloud_rev, self.cloud_t = None, None
        self.cloud_bytes = gp.encode_cloud(None, None, "map", [], b"")
        self.hello = None
        self.ctx = {"source": "unknown", "sourceDetail": None, "scope": None, "task": gp.idle_task()}
        self.cmd_q = queue.Queue()
        self._stop = threading.Event()
        self._threads = []

    def attach(self, client):
        self.client = client
        return self

    def count(self, key, n=1):
        with self.lock:
            self.counts[key] += n

    # ---- FPGAC callbacks (fpgac thread; never raise into it) ----
    def on_frame(self, frame, obj):
        try:
            self._on_frame(frame.name, obj)
        except Exception:                         # noqa: BLE001 - must not kill the FPGAC thread
            self.count("on_frame_error")
            self.log("on_frame error:\n" + traceback.format_exc())

    def _on_frame(self, name, obj):
        now = self.clock()
        if name == "STATE":
            with self.lock:
                self.state, self.state_t = obj, now
        elif name == "MEMORY":
            with self.lock:
                self.memory, self.memory_t = obj, now
        elif name == "PREVIEW":
            if not isinstance(obj, tuple) or len(obj) != 2:
                self.count("preview_invalid")
                return
            meta, payload = obj
            if (not isinstance(meta, dict) or meta.get("encoding") != "jpeg"
                    or not isinstance(payload, bytes) or not 4 <= len(payload) <= PREVIEW_MAX_BYTES
                    or not payload.startswith(b"\xff\xd8") or not payload.endswith(b"\xff\xd9")
                    or not isinstance(meta.get("w"), int) or not 0 < meta["w"] <= PREVIEW_MAX_W
                    or not isinstance(meta.get("h"), int) or not 0 < meta["h"] <= PREVIEW_MAX_H
                    or not isinstance(meta.get("age_ms"), (int, float))
                    or not 0 <= meta["age_ms"] <= 1000):
                self.count("preview_invalid")
                return
            with self.lock:
                self.preview = (dict(meta), payload, now)
                self.preview_seq += 1
                if self.sessions and self._link() == "up":
                    head = {"type": "preview", "version": gp.VERSION, "seq": self.preview_seq,
                            "w": meta["w"], "h": meta["h"], "ageMs": meta["age_ms"], "encoding": "jpeg"}
                    if "ar" in meta:              # 10-27 console first-person boxes (the car screen AR's rows)
                        ar = gp.fpv_ar(meta["ar"])
                        if ar is None:
                            self.count("preview_ar_invalid")
                        else:
                            head["ar"] = ar
                    frame = encode_binary(head, payload)
                    for s in self.sessions:
                        s.put_preview(frame)
                    self.counts["preview_pushed"] += 1
        elif name == "MAP":
            meta = obj[0] if isinstance(obj, tuple) and isinstance(obj[0], dict) else {}
            which = meta.get("which")
            if which == "belief":
                self._on_belief(obj, now)
            elif which in self.raw_maps:
                with self.lock:
                    self.raw_maps[which] = obj   # decoded later in the composer, not here
            else:
                self.count("map_unknown_which")
        elif name == "EVENT":
            ev = obj.get("event") if isinstance(obj, dict) else None
            if isinstance(ev, dict) and ev.get("event") == "controller_changed":
                with self.lock:
                    self.controller = ev.get("controller_client_id") == getattr(self.client, "client_id", None)
        elif name == "CMD_ACK":
            self._on_ack(obj)
        elif name == "CMD_RESULT":
            self._on_result(obj)

    def on_link(self, state, detail=None):
        try:
            detail = detail or {}
            with self.lock:
                self.counts["link:" + state] += 1
                if state == "up" and not detail.get("recovered"):
                    ack = getattr(self.client, "hello_ack", None) or {}
                    self.controller = bool(ack.get("controller"))
        except Exception:                         # noqa: BLE001
            self.count("on_link_error")

    # ---- commands ----
    def submit(self, session, text):
        self.cmd_q.put((session, text))

    def handle_command(self, session, text):
        cid, msg = gp.parse_command(text)
        with self.lock:
            ctx = dict(self.ctx)
            ctx["link"] = getattr(self.client, "link_state", "down")
            est = (self.state or {}).get("estop") if isinstance(self.state, dict) else None
            ctx["estop_epoch"] = est.get("epoch") if isinstance(est, dict) else None
            self.counts["browser_cmd"] += 1
            if isinstance(msg, dict):
                self.counts["browser_cmd:%s" % msg.get("action")] += 1
        if msg is None:
            d = gp.reject("bad_request", "不是 JSON 对象")
        else:
            d = gp.classify_command(msg, ctx)
        if d["kind"] == "reject":
            self.count("reject:" + d["code"])
            session.put_json(gp.make_ack(cid, False, "rejected", d["code"], d["note"]))
            return
        with self.cmd_lock:                       # a CMD_ACK may arrive before command() returns
            hub_id = self.client.command(d["op"], d["args"], d["deadline_ms"])
            self.pending[hub_id] = {"session": session, "id": cid, "op": d["op"], "t": self.clock(),
                                    "deadline_ms": d["deadline_ms"], "acked": None, "note": d["note"],
                                    "result_wait_s": gp.RESULT_WAIT_S.get(d["op"], RESULT_WAIT_S)}
        self.count("forwarded:" + d["op"])

    def _on_ack(self, obj):
        hub_id = obj.get("cmd_id") if isinstance(obj, dict) else None
        with self.cmd_lock:
            p = self.pending.get(hub_id)
            if p is None:
                self.count("ack_unmatched")
                return
            if p["acked"] is not None:
                return
            if obj.get("status") in gp.ACK_TAKEN:     # estop: latched_on_nano
                p["acked"] = self.clock()
                note = p["note"]
                if obj.get("duplicate"):
                    note = ((note + "；") if note else "") + "车上判为重复命令，按第一次处理"
                p["session"].put_json(gp.make_ack(p["id"], True, "forwarded", note=note))
                if p["op"] in gp.NO_RESULT_OPS:          # teleop: the hub sends no CMD_RESULT
                    del self.pending[hub_id]
                return
            del self.pending[hub_id]
            code = obj.get("reason") or obj.get("status") or "rejected"
            self.count("reject:%s" % code)
            p["session"].put_json(gp.make_ack(p["id"], False, "rejected", code, p["note"]))

    def _on_result(self, obj):
        if not isinstance(obj, dict) or obj.get("final", True) is not True:
            return
        with self.cmd_lock:
            p = self.pending.pop(obj.get("cmd_id"), None)
            if p is None:
                self.count("result_unmatched")
                return
            if p["acked"] is None:                # result before ack: say forwarded first
                p["session"].put_json(gp.make_ack(p["id"], True, "forwarded", note=p["note"]))
            status = obj.get("status")
            if status in gp.RESULT_DONE:             # estop latched / release released (KV confirmed)
                status = "done"
            if status in ("accepted", "rejected", "done", "unknown"):
                code = obj.get("reason") if status == "rejected" else None
            else:
                status, code = "unknown", status
            self.count("result:%s" % status)
            p["session"].put_json(gp.make_result(p["id"], status, code, obj.get("detail") if code else None))
        if p["op"] == "map_reset" and status in ("done", "rejected"):
            self.on_map_reset()

    def check_timeouts(self, now):
        with self.cmd_lock:
            for hub_id, p in list(self.pending.items()):
                ack_wait = (gp.ESTOP_NO_ACK_S if p["deadline_ms"] is None
                            else p["deadline_ms"] / 1000.0 + NO_ACK_GRACE_S)
                if p["acked"] is None and now - p["t"] > ack_wait:
                    del self.pending[hub_id]
                    self.count("no_ack")
                    p["session"].put_json(gp.make_ack(p["id"], False, "expired", "no_ack"))
                    p["session"].put_json(gp.make_result(p["id"], "unknown"))
                elif p["acked"] is not None and now - p["acked"] > p.get("result_wait_s", RESULT_WAIT_S):
                    del self.pending[hub_id]
                    self.count("result_timeout")
                    p["session"].put_json(gp.make_result(p["id"], "unknown"))

    def on_map_reset(self):
        """10-26 重置地图 ran on the car: forget the cached RTAB-Map cloud (new pages would get the old map) and
        every page's voxel base (the next push is a full of the new map, or nothing until it has voxels)."""
        with self.lock:
            self.mapping_frame, self.mapping_sha = None, None
            sessions = list(self.sessions)
            self.counts["map_reset_cleared"] += 1
        for s in sessions:
            s.on_mapping_resync()

    def _on_belief(self, obj, now):
        """10-26 search heat map: small (a few KB), composed here and pushed to every page at once."""
        try:
            msg = gp.compose_belief(obj[0], obj[1])
        except Exception as exc:                  # noqa: BLE001 - ValueError, zlib.error, bad shapes
            self.count("belief_invalid")
            self.log("MAP belief rejected: %s" % exc)
            return
        with self.lock:
            self.belief = (msg, now)
            self.counts["belief_pushed"] += 1
            for s in self.sessions:
                s.put_belief(msg)

    def _belief_for_new_page(self, now):
        if self.belief is None:
            return None
        msg, t = self.belief
        out = dict(msg)
        out["ageMs"] = round((out.get("ageMs") or 0.0) + max(0.0, now - t) * 1000.0, 1)
        return out

    # ---- sessions ----
    def register(self, session):
        with self.lock:
            if len(self.sessions) >= MAX_SESSIONS:
                return False
            self.sessions.add(session)
            self.counts["ws_sessions"] += 1
            session.put_hello(self.hello or self._hello(self._link(), self.ctx["source"],
                                                        self.ctx.get("sourceDetail")))
            session.put_cloud(self.cloud_bytes)      # always once on connect (count 0 when none)
            self.counts["cloud_sent"] += 1
            if self.mapping_frame is not None:
                session.put_mapping(self.mapping_frame)
            belief = self._belief_for_new_page(self.clock())
            if belief is not None:
                session.put_belief(belief)
        return True

    def unregister(self, session):
        with self.lock:
            self.sessions.discard(session)

    def _link(self):
        return getattr(self.client, "link_state", "down") if self.client is not None else "down"

    def _hello(self, link, source, detail=None):
        control = "none" if link == "down" else ("controller" if self.controller else "observer")
        hello = gp.make_hello(self.bridge_id, getattr(self.client, "hub_boot_id", None), link, source, control,
                              detail)
        if self.gateway:                           # the page must not look for /rover/* and must reassemble chunks
            hello["gateway"] = dict(GATEWAY_INFO)
        return hello

    # ---- composer ----
    def _grid(self, which, raw):
        cached_raw, grid = self._grids[which]
        if raw is cached_raw:
            return grid
        grid = None
        if raw is not None:
            try:
                grid = gp.decode_grid(raw[0], raw[1])
            except Exception as exc:             # noqa: BLE001 - ValueError, zlib.error, bad shapes
                self.count("map_decode_error:%s" % which)
                self.log("MAP %s decode failed: %s" % (which, exc))
        self._grids[which] = (raw, grid)
        return grid

    def tick(self, now=None):
        now = self.clock() if now is None else now
        with self.lock:
            state, state_t = self.state, self.state_t
            memory, memory_t = self.memory, self.memory_t
            raw_g, raw_r = self.raw_maps["global"], self.raw_maps["rolling"]
        client = self.client
        link = self._link()
        ri = getattr(getattr(client, "runtime_info", None), "value", None)
        boot = getattr(client, "hub_boot_id", None)
        issues = {}
        view = gp.classify_view(state, ri, memory)
        g_grid, r_grid = self._grid("global", raw_g), self._grid("rolling", raw_r)
        placeholder = g_grid is not None and gp.is_placeholder(g_grid["cells"])
        pose = (state or {}).get("pose") if isinstance(state, dict) else None
        map_id = id_source = None
        bm_id = bm_source = None
        if view["tier"] in ("loc", "odom"):
            # 10-18 (owner): the base map is also drawn on the odometry tier ("mapping_start" basis); its id is
            # the car's pose.map_id when the car reports one (H1), else the operator's --map-id. map.id / the cloud
            # header stay localisation-only (§2.3): the console page compares the cloud's mapId with map.id.
            pid = pose.get("map_id") if isinstance(pose, dict) else None
            bm_id = pid if isinstance(pid, str) else self.map_id
            bm_source = "car" if isinstance(pid, str) else ("operator" if self.map_id else None)
            if view["tier"] == "loc":
                map_id, id_source = bm_id, bm_source
        # cloud: send before the snapshot that names its rev
        grid = gp.choose_cloud_grid(view, g_grid, r_grid)
        rolling_hold = (grid is not None and grid["which"] == "rolling" and self.cloud_rev is not None
                        and self.cloud_rev.startswith("rolling:") and self.cloud_t is not None
                        and now - self.cloud_t < ROLLING_MIN_S)
        if not rolling_hold:
            rev = gp.cloud_rev_for(grid) if grid is not None else None
            if rev != self.cloud_rev:
                self._new_cloud(now, rev, grid, map_id, view["frame"])
        task_group = state.get("task") if isinstance(state, dict) else None
        tid = self.tracker.update(boot, task_group)
        task = gp.compose_task(task_group, tid, issues) if task_group is not None else gp.idle_task()
        snap = gp.compose_snapshot(state, ri, memory, None if memory_t is None else now - memory_t, link,
                                   None if state_t is None else (now - state_t) * 1000.0, task,
                                   self.cloud_rev, placeholder, map_id=map_id, map_name=self.map_name,
                                   counts=issues, view=view)
        snap.pop("type", None)
        snap.pop("version", None)
        # kiosk additions (§2.10, 10-17): optional keys an older page ignores
        basemap = gbm.basemap_status(view["tier"], bm_id, bm_source, self.basemaps,
                                     configured=self.basemap_dir is not None)
        if basemap is not None:
            snap["map"]["basemap"] = basemap
            if basemap["status"] != "ok":
                issues["basemap_" + basemap["status"]] = 1
        fpga = gp.compose_fpga(state, None if state_t is None else (now - state_t) * 1000.0)
        if fpga is not None:
            snap["fpga"] = fpga
        # 10-28 start-up notices of the hub (ground calib fallback: "相机角度测量异常…"), optional key
        notices = [n for n in (state.get("notices") if isinstance(state, dict) else None) or []
                   if isinstance(n, dict) and isinstance(n.get("text"), str) and n["text"]]
        if notices:
            snap["notices"] = [{"code": str(n.get("code") or "")[:40], "level": str(n.get("level") or "warn")[:10],
                                "text": n["text"][:100]} for n in notices[:4]]
        health = compose_health(state.get("health") if isinstance(state, dict) else None)
        if health is not None:         # 10-29 health check of the car stack (host/hub/health.py), optional key
            snap["health"] = health
        qt = task_group.get("query_text") if isinstance(task_group, dict) else None
        if isinstance(qt, str) and qt.strip() and task["state"] != "idle":
            task["queryText"] = qt.strip()[:gp.MAX_FIND_TEXT]
        gp.dumps(snap)                       # fail here (counted) rather than in a sender thread
        with self.lock:
            self.issues_last = issues
            for k, v in issues.items():
                self.issues_max[k] = max(self.issues_max[k], v)
            self.ctx = {"source": snap["source"], "sourceDetail": snap["sourceDetail"],
                        "scope": snap["memory"]["scope"], "task": task}
            hello = self._hello(link, snap["source"], snap["sourceDetail"])
            if hello != self.hello:
                self.hello = hello
                self.counts["hello_sent"] += 1
                for s in self.sessions:
                    s.put_hello(hello)
            for s in self.sessions:
                s.put_snapshot(snap)
            self.counts["snapshot_ticks"] += 1
        self.check_timeouts(now)
        return snap

    def _new_cloud(self, now, rev, grid, map_id, frame):
        if grid is not None:
            xyz, rgb, step = gp.grid_to_points(grid)
            data = gp.encode_cloud(map_id, rev, frame, xyz, rgb)
            if step > 1:
                self.count("cloud_thinned")
        else:
            xyz, rgb, data = [], b"", gp.encode_cloud(None, None, frame, [], b"")
        with self.lock:
            self.cloud_rev, self.cloud_t, self.cloud_bytes = rev, now, data
            self.counts["cloud_built"] += 1
            for s in self.sessions:
                s.put_cloud(data)
                self.counts["cloud_sent"] += 1
        if self.dump_cloud and grid is not None:
            try:
                os.makedirs(self.dump_cloud, exist_ok=True)
                name = "cloud_%04d_%s.ply" % (self.counts["cloud_built"], rev.replace(":", "_"))
                with open(os.path.join(self.dump_cloud, name), "wb") as f:
                    f.write(gp.ply_bytes(xyz, rgb))
            except OSError as exc:
                self.log("dump-cloud failed: %s" % exc)

    # ---- threads ----
    def _compose_loop(self):
        while not self._stop.wait(TICK_S):
            try:
                self.tick()
            except Exception:                     # noqa: BLE001 - keep composing
                self.count("tick_error")
                self.log("tick error:\n" + traceback.format_exc())

    def _command_loop(self):
        while True:
            item = self.cmd_q.get()
            if item is None:
                return
            try:
                self.handle_command(*item)
            except Exception:                     # noqa: BLE001
                self.count("command_error")
                self.log("command error:\n" + traceback.format_exc())

    def poll_basemap_dir(self):
        """One scan of --basemap-watch (10-18); returns the library sha256s added. Called by the watch thread
        (and directly by tests)."""
        if self.basemap_dir is None:
            return []
        with self._basemap_lock:
            new = self.basemap_dir.scan(self.basemaps)
            if new:
                merged = dict(self.basemaps)
                for p in new:
                    merged[p["sha256"]] = p
                self.basemaps = merged                # atomic swap, see __init__
                self.count("basemap_loaded_runtime", len(new))
        return [p["sha256"] for p in new]

    def _basemap_loop(self):
        while True:
            try:
                self.poll_basemap_dir()
            except Exception:                     # noqa: BLE001 - keep watching
                self.count("basemap_watch_error")
                self.log("basemap watch error:\n" + traceback.format_exc())
            if self._stop.wait(BASEMAP_POLL_S):
                return

    def start(self):
        loops = [(self._compose_loop, "gmb-compose"), (self._command_loop, "gmb-command")]
        if self.basemap_dir is not None:
            loops.append((self._basemap_loop, "gmb-basemap-watch"))
        if self.mapping_feed is not None:
            loops.append((self._mapping_loop, "gmb-mapping"))
        for target, name in loops:
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def stop(self):
        self._stop.set()
        self.cmd_q.put(None)
        with self.lock:
            sessions = list(self.sessions)
        for s in sessions:
            s.close(ws.CLOSE_GOING_AWAY, "bridge stopping")
        for t in self._threads:
            t.join(2.0)

    # ---- RTAB-Map display cloud relay (10-21) ----
    def feed_get(self, path, timeout=8.0, limit=MAPPING_MAX_BYTES):
        """GET from the Nano display feed (loopback). Returns (status, content type, body)."""
        import http.client
        host, port = self.mapping_feed
        conn = http.client.HTTPConnection(host, port, timeout=timeout)
        try:
            conn.request("GET", path, headers={"Host": "%s:%d" % (host, port)})
            r = conn.getresponse()
            body = r.read(limit + 1)
            if len(body) > limit:
                raise ValueError("mapping body over limit")
            return r.status, r.getheader("Content-Type") or "application/octet-stream", body
        finally:
            conn.close()

    def _mapping_poll(self, last_push, last_keep):
        """One poll: full push when the cloud changed (and MAPPING_MIN_PUSH_S passed), else a keep-alive.
        Returns updated (last_push, last_keep)."""
        now = self.clock()
        status, _, body = self.feed_get("/status", timeout=3.0, limit=65536)
        if status != 200:
            raise ValueError("feed status %d" % status)
        meta = json.loads(body.decode("utf-8"))
        if isinstance(meta.get("epoch"), int) and isinstance(meta.get("seq"), int):
            return self._mapping_delta_poll(meta, last_push, last_keep)
        sha = meta.get("sha256")
        if meta.get("error") or not isinstance(sha, str) or not meta.get("count"):
            return last_push, last_keep                # nothing displayable yet; page keeps polling state
        if sha != self.mapping_sha and now - last_push >= MAPPING_MIN_PUSH_S:
            status, _, raw = self.feed_get("/cloud.bin?format=q5z")
            if status != 200:
                raise ValueError("feed cloud %d" % status)
            (L,) = struct.unpack_from("<I", raw, 0)
            head = json.loads(raw[4:4 + L].decode("utf-8"))
            if head.get("encoding") != "q5z":
                raise ValueError("feed is too old for q5z (restart live_mapping.py)")
            head.update({"type": "mapping", "version": gp.VERSION})
            frame = encode_binary(head, raw[4 + L:])
            with self.lock:
                self.mapping_frame, self.mapping_sha = frame, head.get("sha256")
                for s in self.sessions:
                    s.put_mapping(frame)
                self.counts["mapping_pushed"] += 1
                self.counts["mapping_bytes"] += len(frame)
            return now, now
        if now - last_keep >= MAPPING_KEEPALIVE_S:
            keep = {k: meta.get(k) for k in ("format", "source", "session", "scope", "frame", "count", "sha256",
                                             "graphNodes", "mapToOdom", "graphAgeMs", "cloudAgeMs", "revision")}
            keep.update({"type": "mapping", "version": gp.VERSION, "unchanged": sha == self.mapping_sha})
            frame = encode_binary(keep)
            with self.lock:
                for s in self.sessions:
                    s.put_mapping(frame)
                self.counts["mapping_keepalive"] += 1
            return last_push, now
        return last_push, last_keep

    def _fetch_delta(self, have):
        """GET /delta since ``have`` ((epoch, seq) or None = full) -> (header, encoded mapping frame)."""
        path = "/delta" if have is None else "/delta?epoch=%d&seq=%d" % have
        status, _, raw = self.feed_get(path)
        if status != 200:
            raise ValueError("feed delta %d" % status)
        (L,) = struct.unpack_from("<I", raw, 0)
        head = json.loads(raw[4:4 + L].decode("utf-8"))
        if head.get("encoding") != "vdz1" or head.get("mode") not in ("full", "delta"):
            raise ValueError("feed delta format")
        head.update({"type": "mapping", "version": gp.VERSION})
        return head, encode_binary(head, raw[4 + L:])

    def _mapping_delta_poll(self, meta, last_push, last_keep):
        """Per page: nothing while its previous mapping message is still going out (unless a new epoch makes
        it obsolete); else the merged changes since its (epoch, seq), a full when it has none; a header-only
        keep-alive every MAPPING_KEEPALIVE_S when it is current."""
        now = self.clock()
        cur = (meta["epoch"], meta["seq"])
        if meta.get("error") or not meta.get("voxels") or not isinstance(meta.get("mapToOdom"), dict):
            return last_push, last_keep               # page would reject it (no graph yet): do not ship a full
        with self.lock:
            sessions = list(self.sessions)
        fetched = {}
        for s in sessions:
            have = s.mapping_have
            idle = s.mapping_idle()
            if have == cur:
                if idle and now - s.mapping_keep_t >= MAPPING_KEEPALIVE_S:
                    keep = {k: meta.get(k) for k in ("format", "source", "session", "scope", "frame", "graphNodes",
                                                     "mapToOdom", "graphAgeMs", "cloudAgeMs", "revision")}
                    keep.update({"type": "mapping", "version": gp.VERSION, "mode": "keep", "epoch": cur[0],
                                 "seq": cur[1], "count": meta.get("voxels")})
                    s.mapping_keep_t = now
                    s.put_mapping(encode_binary(keep))
                    self.count("mapping_keepalive")
                continue
            abort = not idle and have is not None and have[0] != cur[0]
            if not idle and not abort:
                continue                              # back-pressure: the page is still receiving
            if not abort and have is not None and now - s.mapping_push_t < MAPPING_DELTA_MIN_PUSH_S:
                continue
            base = have if have is not None and have[0] == cur[0] else None
            if base not in fetched:
                fetched[base] = self._fetch_delta(base)
            head, frame = fetched[base]
            if base is not None and (head["epoch"], head["seq"]) == base:
                continue                              # feed status ran ahead of nothing: no change after all
            s.mapping_keep_t = now
            s.put_mapping(frame, have=(head["epoch"], head["seq"]), abort=abort, now=now)
            with self.lock:
                self.counts["mapping_%s_pushed" % head["mode"]] += 1
                self.counts["mapping_bytes"] += len(frame)
        return last_push, last_keep

    def _mapping_loop(self):
        last_push = last_keep = -1e9
        failed = False
        while not self._stop.wait(MAPPING_DELTA_POLL_S):
            with self.lock:
                idle = not self.sessions
            if idle:
                continue
            try:
                last_push, last_keep = self._mapping_poll(last_push, last_keep)
                failed = False
            except Exception as exc:              # noqa: BLE001 - feed down / restarting: retry next second
                self.count("mapping_error")
                if not failed:
                    self.log("mapping feed: %s" % exc)
                failed = True

    def basemap_file(self, path):
        """(body, content type) of a loaded base-map package file, or None (§2.10)."""
        return gbm.resolve(path, self.basemaps)

    def stats(self):
        with self.lock:
            out = {"bridge_id": self.bridge_id, "counts": dict(self.counts),
                   "rejects": dict((k[7:], v) for k, v in self.counts.items() if k.startswith("reject:")),
                   "snapshot_issues_last": dict(self.issues_last), "snapshot_issues_max": dict(self.issues_max),
                   "cloud_rev": self.cloud_rev, "sessions_open": len(self.sessions)}
        stats = getattr(self.client, "stats", None)
        if callable(stats):
            try:
                out["fpgac"] = stats()
            except Exception:                     # noqa: BLE001
                out["fpgac"] = None
        return out


# ---------------------------------------------------------------------------- HTTP / WS server

class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"             # the default 1.0 writes "HTTP/1.0 101", which Chrome refuses
    server_version = "GeoMemBridge/2"
    timeout = 60

    def log_message(self, fmt, *args):        # quiet; counts go to --stats-out
        pass

    def _port(self):
        """Ports a browser may name in Host/Origin: the bound one plus --public-port (ssh -L)."""
        extra = getattr(self.server, "public_ports", ())
        return (self.server.server_address[1],) + tuple(extra) if extra else self.server.server_address[1]

    def _plain(self, status, text, head_only=False, extra=()):
        body = (text + "\n").encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Cache-Control", "no-cache")
        for k, v in extra:
            self.send_header(k, v)
        self.end_headers()
        if not head_only:
            self.wfile.write(body)

    def do_GET(self):
        self._serve(False)

    def do_HEAD(self):
        self._serve(True)

    def _serve(self, head_only):
        bridge = self.server.bridge
        if not ws.check_host(self.headers, self._port()):
            bridge.count("http_bad_host")
            return self._plain(403, "host not allowed", head_only)
        path = self.path.split("?", 1)[0].split("#", 1)[0]
        query = self.path.split("?", 1)[1].split("#", 1)[0] if "?" in self.path else ""
        if path.startswith("/bridge/"):            # 10-21: same URLs as the PC console server (server.mjs)
            path = path[len("/bridge"):]
        if path.startswith("/rover/"):            # the old PC-side camera agent does not exist here
            return self._plain(503, "no rover agent on the gateway", head_only)
        if path in ("/mapping/status", "/mapping/cloud.bin"):
            return self._mapping(bridge, path[len("/mapping"):], query, head_only)
        if path == "/ws":
            if head_only:
                return self._plain(405, "method", True)
            return self._upgrade(bridge)
        if path == "/preview.jpg":
            with bridge.lock:
                frame = bridge.preview
                link = getattr(bridge.client, "link_state", "down")
            if (frame is None or link != "up"
                    or bridge.clock() - frame[2] + frame[0]["age_ms"] / 1000.0 > PREVIEW_STALE_S):
                return self._plain(503, "preview unavailable or stale", head_only)
            body, ctype = frame[1], "image/jpeg"
        elif path.startswith("/basemap/"):          # §2.10: only loaded packages (start or --basemap-watch), from memory
            hit = bridge.basemap_file(path)
            if hit is None:
                bridge.count("http_404")
                return self._plain(404, "not found", head_only)
            body, ctype = hit
            bridge.count("basemap_served")
        else:
            full = ws.safe_static_path(path, str(self.server.app_root))
            if full is None:
                bridge.count("http_404")
                return self._plain(404, "not found", head_only)
            try:
                with open(full, "rb") as f:
                    body = f.read()
            except OSError:
                return self._plain(404, "not found", head_only)
            ctype = CONTENT_TYPES.get(os.path.splitext(full)[1].lower(), "application/octet-stream")
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        if not head_only:
            self.wfile.write(body)

    def _mapping(self, bridge, route, query, head_only):
        """Legacy pull path for pages that poll /mapping/* (the gateway also pushes over /ws)."""
        if bridge.mapping_feed is None:
            return self._plain(503, "no mapping feed", head_only)
        path = route + ("?format=q5z" if route == "/cloud.bin" and query == "format=q5z" else "")
        try:
            status, ctype, body = bridge.feed_get(path)
        except Exception:                          # noqa: BLE001
            return self._plain(503, "mapping feed unavailable", head_only)
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if not head_only:
            self.wfile.write(body)

    def _upgrade(self, bridge):
        ok, status, why, accept = ws.check_upgrade(self.command, self.headers, self._port())
        if not ok:
            bridge.count("ws_refused:%s" % why)
            extra = (("Sec-WebSocket-Version", "13"),) if status == 426 else ()
            return self._plain(status, why, extra=extra)
        session = WsSession(self.connection, self.rfile, bridge, chunked=bridge.gateway)
        if not bridge.register(session):
            bridge.count("ws_refused:too_many")
            return self._plain(503, "too many connections")
        try:
            self.send_response(101, "Switching Protocols")
            self.send_header("Upgrade", "websocket")
            self.send_header("Connection", "Upgrade")
            self.send_header("Sec-WebSocket-Accept", accept)
            self.end_headers()
            self.wfile.flush()
            self.close_connection = True       # after the session, never parse these bytes as HTTP
            self.connection.settimeout(None)
            session.start_sender()
            session.run_reader()
        finally:
            session.close()
            bridge.unregister(session)
            session.join()


class BridgeHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = os.name != "nt"    # SO_REUSEADDR on Windows would let another process share the port

    def __init__(self, addr, bridge, app_root=APP_ROOT, public_ports=()):
        self.bridge = bridge
        self.app_root = Path(app_root)
        self.public_ports = tuple(public_ports)
        HTTPServer.__init__(self, addr, Handler)


# ---------------------------------------------------------------------------- main

def parse_hostport(text, what):
    host, sep, port = (text or "").rpartition(":")
    if not sep or not host:
        raise SystemExit("%s 须为 HOST:PORT：%r" % (what, text))
    try:
        port = int(port)
    except ValueError:
        raise SystemExit("%s 端口不是整数：%r" % (what, text))
    if not 0 <= port <= 65535:
        raise SystemExit("%s 端口越界：%r" % (what, text))
    return host, port


def build_arg_parser():
    ap = argparse.ArgumentParser(description="GeoMem 显示界面桥（浏览器协议 v2 <-> 车上 hub FPGAC）")
    ap.add_argument("--hub", required=True, help="车上 hub 地址 HOST:PORT（car.json 的 hub，默认端口 47431）")
    ap.add_argument("--token-file", required=True, help="FPGAC 口令文件（与 fpgapc.py 同一份）")
    ap.add_argument("--map-id", default=None, help="定位档地图库 sha256（64 位十六进制）")
    ap.add_argument("--map-name", default=None, help="地图名，如 实验室")
    ap.add_argument("--basemap", action="append", default=[], metavar="DIR",
                    help="展台底图包目录（make_basemap.py 的输出，可重复）；定位档或里程档且地图 id 等于包的库 sha256 时页面才画")
    ap.add_argument("--basemap-watch", default=None, metavar="DIR",
                    help="运行中加载：每 2 s 看 DIR 下新出现的 basemap-<库 sha 前 12 位>/，核对通过即加载（不重启桥）")
    ap.add_argument("--listen", default="127.0.0.1:8788", help="本机监听地址，只允许 127.0.0.1")
    ap.add_argument("--dump-cloud", default=None, help="把每次发出的点云另存 PLY 到此目录")
    ap.add_argument("--stats-out", default=None, help="退出时把统计写成 JSON")
    ap.add_argument("--app-root", default=None,
                    help="网页目录（默认 ui/geomem/app；Nano 直连网关用 ui/geomem/operator）")
    ap.add_argument("--public-port", type=int, action="append", default=[],
                    help="浏览器经 ssh -L 访问时地址栏里的端口（可重复），Host/Origin 校验放行")
    ap.add_argument("--mapping-feed", default=None, metavar="HOST:PORT",
                    help="RTAB-Map 三维显示源（live_mapping.py，通常 127.0.0.1:47802）：变了才推、压缩")
    ap.add_argument("--gateway", action="store_true",
                    help="Nano 直连网关（nano_gateway.sh）：握手告知页面无 rover 代理；大块数据切 64 KB 分片逐片回执")
    return ap


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    lhost, lport = parse_hostport(args.listen, "--listen")
    if lhost != "127.0.0.1":
        raise SystemExit("--listen 只允许 127.0.0.1（开放局域网须 owner 同意，见设计 §3.6）")
    hhost, hport = parse_hostport(args.hub, "--hub")
    try:
        with open(args.token_file, "r", encoding="utf-8") as f:
            token = f.read().strip()
    except OSError as exc:
        raise SystemExit("读不了口令文件：%s" % exc)
    if not token:
        raise SystemExit("口令文件是空的")
    if str(REPO) not in sys.path:
        sys.path.insert(0, str(REPO))
    from pc import fpgac                       # noqa: E402 - repo import, never host.*

    def log(msg):
        sys.stderr.write("[geomem-bridge] %s\n" % msg)
        sys.stderr.flush()

    try:
        basemaps = [gbm.load_package(d) for d in args.basemap]
        gbm.index_packages(basemaps)
    except gbm.BasemapError as exc:
        raise SystemExit("底图包无效：%s" % exc)
    for p in basemaps:
        log("basemap %s… %d cells from %s" % (p["sha256"][:12], p["count"], p["dir"]))
    if args.basemap_watch and not os.path.isdir(args.basemap_watch):
        raise SystemExit("--basemap-watch 不是目录：%s" % args.basemap_watch)
    if basemaps and args.map_id and args.map_id not in [p["sha256"] for p in basemaps]:
        log("注意：--map-id 与启动时的底图包库 sha256 都不符，页面暂不画底图%s"
            % ("（等 --basemap-watch 目录里出现该库的包）" if args.basemap_watch else ""))
    mapping_feed = parse_hostport(args.mapping_feed, "--mapping-feed") if args.mapping_feed else None
    if mapping_feed is not None and mapping_feed[0] != "127.0.0.1":
        raise SystemExit("--mapping-feed 只允许 127.0.0.1")
    app_root = Path(args.app_root).resolve() if args.app_root else APP_ROOT
    if not (app_root / "index.html").is_file():
        raise SystemExit("--app-root 下没有 index.html：%s" % app_root)
    bridge = Bridge(map_id=args.map_id, map_name=args.map_name, dump_cloud=args.dump_cloud, log=log,
                    basemaps=basemaps, basemap_dir=args.basemap_watch, mapping_feed=mapping_feed,
                    gateway=args.gateway)
    if args.basemap_watch:
        log("basemap watch %s (every %.0f s)" % (os.path.abspath(args.basemap_watch), BASEMAP_POLL_S))
    client = fpgac.FpgacClient(hport, token, hosts=[hhost], on_frame=bridge.on_frame, on_link=bridge.on_link,
                            want=("state", "memory", "events", "map_rolling", "map_global", "preview"))
    bridge.attach(client)
    try:
        server = BridgeHTTPServer((lhost, lport), bridge, app_root=app_root, public_ports=args.public_port)
    except OSError as exc:
        raise SystemExit("监听 %s:%d 失败：%s" % (lhost, lport, exc))
    port = server.server_address[1]

    def _term(signum, frame):
        raise KeyboardInterrupt()

    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _term)
    # 10-16: start + the "listening" line inside the try: a SIGTERM right after that line used to raise
    # KeyboardInterrupt outside it -> no cleanup, no --stats-out, rc -2 (3.8 141/200, 3.13 6/200)
    try:
        bridge.start()
        client.start()
        log("listening http://127.0.0.1:%d/  hub %s:%d  bridge %s" % (port, hhost, hport, bridge.bridge_id))
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        bridge.stop()
        client.stop()
        stats = bridge.stats()
        text = json.dumps(stats, ensure_ascii=False, indent=1, sort_keys=True, default=str)
        if args.stats_out:
            try:
                with open(args.stats_out, "w", encoding="utf-8") as f:
                    f.write(text)
            except OSError as exc:
                log("stats-out failed: %s" % exc)
        log("stats: browser_cmd=%d rejects=%s cloud_sent=%d" % (
            stats["counts"].get("browser_cmd", 0), stats["rejects"], stats["counts"].get("cloud_sent", 0)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
