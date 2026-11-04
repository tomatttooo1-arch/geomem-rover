"""Colour bypass for the car AR screen (owner 10-15: colour picture as the main AR view), 2026-10-16.

The aligned colour image (Gemini 2 hardware D2C: colour pixel (u, v) = depth pixel (u, v)) travels on
its OWN UDP socket pair, never inside the geometry request (RGB_OPTIONS.md: a 921 KB block on the REQ
would add ~7-10 ms to every geometry frame age). Pull model:

  Nano  ColourSource  --HELLO (every hello_s)-->  KV  ColourSink   (KV learns the source address)
  KV    on each decoded geometry REQ that carries the sensor key (REQ STAMP block, feature stamp_ext):
        ColourSink.want()  --WANT(want_id, key, tol)-->  Nano
  Nano  nearest colour frame within tol  --FRAME(want_id) chunks-->  KV   (or NONE(want_id, reason))
  KV    ArPresenter sender thread: ColourSink.take(frame_id, capture_ns, timeout) -> RGB888 or a reason

Why pull: the colour leaves the Nano only after the depth request is fully on the KV side (no
colour burst queued in front of a depth request on the Nano NIC), only for frames the KV really
serves (the camera runs 30 fps, geometry ~10 Hz), and the pairing is exact by the camera's own
stamp (the colour sender is a separate process: the producer's GIL never sees the colour).
Encoding: raw RGB888, width x height x 3 bytes (no decoder on the KV side, pixel-exact checking;
640x480 = 921,600 B, ~7.4 ms on 1 GbE; 10 Hz ~74 Mbit/s -- arithmetic, not measured on the board).

Messages use host.kv_link chunk framing (split / Reassembler) on this socket only; types 40-43.
  HELLO  (Nano->KV) json {"ver", "source", "width", "height", "format"[, "camera_fps"]}
         10-29: "camera_fps" (only when the camera does not run 30 fps): the sink's default WANT tolerance becomes at
         least half that camera period plus 5 ms (pair_tol_ns), so the nearest colour frame always pairs (15 fps: 39 ms)
  WANT   (KV->Nano) frame_id = want_id; WANT_BODY ver key tol_ns kv_frame_id kv_capture_ns
  FRAME  (Nano->KV) frame_id = want_id; FRAME_HEAD ver fmt source_key want_key width height crc32,
                    then width*height*3 bytes
  NONE   (Nano->KV) frame_id = want_id; NONE_BODY ver reason want_key

DPU input (DPUPIPE0, 2026-11-03; types 44-45, same socket, same HELLO). A ColourSource now serves several
receivers at once: HELLO goes to every address in ``kv_addr`` (one or a list), and every answer goes back
to the address the WANT came from (for the one-KV case that is the same address as before).
  WANT_DPU  (receiver->Nano) frame_id = want_id; DPU_WANT ver fmt size fix_point flags after_key wait_ms
            chunk_bytes kv_t_ns: "the newest frame newer than after_key, preprocessed for this DPU input,
            cut into chunk_bytes chunks"; held up to wait_ms when there is none yet (long poll).
  FRAME_DPU (Nano->receiver) frame_id = want_id; DPU_HEAD (64 B) + int8 content rows (host/dpu_prep.py:
            ultralytics letterbox + data-fix, byte-identical to dev3's evaluator). Fixed 64-byte head so a
            receiver that knows chunk_bytes can scatter every chunk straight into the DPU input buffer
            (tools/dpu_pipe/dpu_rx.cpp).
  NONE      reason no_new (4) when wait_ms ran out, error (3) when preprocessing failed.
Python 3.8 compatible.
"""
from collections import Counter, OrderedDict, deque
import json
import math
import os
import socket
import struct
import threading
import time
import zlib

from host.kv_link import CHUNK, CHUNK_FRAGMENTED, CHUNK_MTU, MAX_DATAGRAM, Reassembler, set_rcvbuf, split

CHELLO, CWANT, CFRAME, CNONE = 40, 41, 42, 43
CWANT_DPU, CFRAME_DPU = 44, 45
VER = 1
FMT_RGB8 = 1
WANT_BODY = struct.Struct("<B3sQQQQ")       # ver rsv key tol_ns kv_frame_id kv_capture_ns
FRAME_HEAD = struct.Struct("<BB2sQQIII")    # ver fmt rsv source_key want_key width height crc32(pixels)
NONE_BODY = struct.Struct("<BB2sQ")         # ver reason rsv want_key
NONE_REASONS = {1: "no_match", 2: "no_frames", 3: "error", 4: "no_new"}
FMT_DPU_I8 = 2
DPU_WANT = struct.Struct("<BBHBB2sQIIQ")   # ver fmt size fix_point flags rsv after_key wait_ms chunk_bytes kv_t_ns
DPU_HEAD = struct.Struct("<BBHBbBBQQQIIIIHHHHHHI")
# ver fmt size fix_point pad_q backend rsv | source_key after_key kv_t_ns | prep_ns hold_ns crc32 content_bytes |
# src_w src_h w h top left | seq   = 64 bytes
assert DPU_HEAD.size == 64
DPU_BACKENDS = {"numpy": 1, "cv2": 2}
DPU_WAIT_MAX_MS = 2000
MAX_W, MAX_H = 1280, 800


def encode_want(key, tol_ns, kv_frame_id, kv_capture_ns):
    return WANT_BODY.pack(VER, b"\0\0\0", int(key) & 0xFFFFFFFFFFFFFFFF, int(tol_ns),
                          int(kv_frame_id) & 0xFFFFFFFFFFFFFFFF, int(kv_capture_ns) & 0xFFFFFFFFFFFFFFFF)


def decode_want(buf):
    ver, _r, key, tol, fid, cap = WANT_BODY.unpack_from(buf)
    if ver != VER or len(buf) != WANT_BODY.size:
        raise ValueError("bad WANT")
    return {"key": key, "tol_ns": tol, "kv_frame_id": fid, "kv_capture_ns": cap}


def encode_frame(source_key, want_key, width, height, rgb):
    rgb = bytes(rgb)
    if len(rgb) != int(width) * int(height) * 3:
        raise ValueError("rgb is %d bytes, %dx%dx3 expected" % (len(rgb), width, height))
    return FRAME_HEAD.pack(VER, FMT_RGB8, b"\0\0", int(source_key), int(want_key), int(width),
                           int(height), zlib.crc32(rgb) & 0xFFFFFFFF) + rgb


def decode_frame(buf):
    ver, fmt, _r, skey, wkey, w, h, crc = FRAME_HEAD.unpack_from(buf)
    rgb = bytes(buf[FRAME_HEAD.size:])
    if ver != VER or fmt != FMT_RGB8:
        raise ValueError("bad FRAME ver/fmt")
    if not (0 < w <= MAX_W and 0 < h <= MAX_H) or len(rgb) != w * h * 3:
        raise ValueError("bad FRAME size")
    if zlib.crc32(rgb) & 0xFFFFFFFF != crc:
        raise ValueError("FRAME crc")
    return {"source_key": skey, "want_key": wkey, "width": w, "height": h, "rgb": rgb}


def pair_tol_ns(tol_ns, hello):
    """10-29: the default WANT tolerance for a source's HELLO: the configured one (kv_start.sh --ar-colour-tol-ms 17 =
    half a 30 fps period), widened when the HELLO says the camera runs slower than 30 fps to half the camera period
    plus 5 ms, rounded up to whole ms -- the runner's pair slop (host/ros_runtime.camera_pairing, 15 fps 0.038 s) or
    just above it (owner 10-29: 15 fps stamps sat 26.6 / 32.1 ms apart, random sensor phase can reach 33.3 ms):
    15 fps -> 39 ms, 10 -> 55, 5 -> 105. 30 fps and faster: half the period as before (30 -> 17 ms). A HELLO
    without a usable camera_fps (older Nano, 30 fps) leaves it unchanged."""
    try:
        fps = float((hello or {}).get("camera_fps"))
    except (TypeError, ValueError):
        return int(tol_ns)
    if not (fps == fps and 1.0 <= fps <= 120.0):
        return int(tol_ns)
    half_ms = 500.0 / fps + (5.0 if fps < 30.0 else 0.0)
    return max(int(tol_ns), int(math.ceil(half_ms - 1e-9)) * 1000000)


def encode_want_dpu(size, fix_point, after_key, wait_ms, chunk_bytes, kv_t_ns=0, flags=0):
    return DPU_WANT.pack(VER, FMT_DPU_I8, int(size), int(fix_point), int(flags), b"\0\0",
                         int(after_key) & 0xFFFFFFFFFFFFFFFF, int(wait_ms), int(chunk_bytes),
                         int(kv_t_ns) & 0xFFFFFFFFFFFFFFFF)


def decode_want_dpu(buf):
    ver, fmt, size, fp, flags, _r, after, wait_ms, chunk, kv_t = DPU_WANT.unpack_from(buf)
    if ver != VER or fmt != FMT_DPU_I8 or len(buf) != DPU_WANT.size:
        raise ValueError("bad WANT_DPU")
    if size not in (640, 512, 416, 320) or not 0 <= fp <= 7:
        raise ValueError("bad WANT_DPU size/fix_point")
    if not 1 <= chunk <= MAX_DATAGRAM - CHUNK.size:
        raise ValueError("bad WANT_DPU chunk_bytes")
    return {"size": size, "fix_point": fp, "flags": flags, "after_key": after,
            "wait_ms": min(int(wait_ms), DPU_WAIT_MAX_MS), "chunk_bytes": chunk, "kv_t_ns": kv_t}


def encode_frame_dpu(geometry, content, *, fix_point, pad_q, backend, source_key, after_key, kv_t_ns,
                     prep_ns, hold_ns, seq):
    g = geometry
    if len(content) != g.content_bytes:
        raise ValueError("content is %d bytes, %d expected" % (len(content), g.content_bytes))
    head = DPU_HEAD.pack(VER, FMT_DPU_I8, g.size, int(fix_point), int(pad_q), DPU_BACKENDS.get(backend, 0), 0,
                         int(source_key) & 0xFFFFFFFFFFFFFFFF, int(after_key) & 0xFFFFFFFFFFFFFFFF,
                         int(kv_t_ns) & 0xFFFFFFFFFFFFFFFF, min(int(prep_ns), 0xFFFFFFFF),
                         min(int(hold_ns), 0xFFFFFFFF), zlib.crc32(content) & 0xFFFFFFFF, len(content),
                         g.src_w, g.src_h, g.w, g.h, g.top, g.left, int(seq) & 0xFFFFFFFF)
    return head + content


def decode_frame_dpu(buf):
    f = DPU_HEAD.unpack_from(buf)
    keys = ("ver", "fmt", "size", "fix_point", "pad_q", "backend", "rsv", "source_key", "after_key", "kv_t_ns",
            "prep_ns", "hold_ns", "crc32", "content_bytes", "src_w", "src_h", "w", "h", "top", "left", "seq")
    d = dict(zip(keys, f))
    content = bytes(buf[DPU_HEAD.size:])
    if d["ver"] != VER or d["fmt"] != FMT_DPU_I8:
        raise ValueError("bad FRAME_DPU ver/fmt")
    if len(content) != d["content_bytes"] or len(content) != d["w"] * d["h"] * 3:
        raise ValueError("bad FRAME_DPU size")
    if zlib.crc32(content) & 0xFFFFFFFF != d["crc32"]:
        raise ValueError("FRAME_DPU crc")
    d["content"] = content
    return d


# ------------------------------------------------------------------ KV side
class ColourSink:
    """kv_server end. ``want`` is called by the geometry worker right after a request is decoded
    (one small sendto, never blocks); ``take`` by the AR sender thread (bounded wait)."""

    def __init__(self, bind=("0.0.0.0", 0), *, rcvbuf=8 << 20, tol_ns=10_000_000, keep=8, sock=None):
        self.sock = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if sock is None:
            self.rcvbuf = set_rcvbuf(self.sock, rcvbuf)
            self.sock.bind(bind)
        else:
            self.rcvbuf = ("given", None)
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]
        self.tol_ns = int(tol_ns)
        self.keep = int(keep)
        self.counts = Counter()
        self.rx = Reassembler(self.counts, prefix="colour_rx_")
        self.cond = threading.Condition()
        self.source = None              # (addr, hello dict, last hello mono)
        self.next_want = 0
        self.wants = OrderedDict()      # (frame_id, capture_ns) -> want_id
        self.results = OrderedDict()    # want_id -> dict
        self.want_t = {}                # want_id -> mono at WANT
        self.want_tol = {}              # want_id -> tolerance of that WANT (ns), for the key_far check
        self.lat_ms = deque(maxlen=256)  # WANT -> FRAME complete, ms (KV clock)
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._reader, name="colour-rx", daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(2.0)
        self.sock.close()

    def _reader(self):
        while not self._stop.is_set():
            try:
                data, addr = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                self.counts["recv_error"] += 1
                continue
            try:
                self._handle(data, addr)
            except Exception as exc:        # a bad datagram never kills the bypass
                self.counts["handle_error:%s" % type(exc).__name__] += 1

    def _handle(self, data, addr):
        got = self.rx.feed(data)
        if got is None:
            return
        mtype, _flags, _session, want_id, payload, _t0 = got
        if mtype == CHELLO:
            info = json.loads(payload.decode("utf-8"))
            with self.cond:
                if self.source is None or self.source[0] != addr:
                    self.counts["source_registered"] += 1
                self.source = (addr, info, time.monotonic())
                self.counts["hello"] += 1
            self.rx.forget_done(CHELLO)
            return
        if mtype in (CFRAME, CNONE):
            try:
                res = decode_frame(payload) if mtype == CFRAME else None
                if mtype == CNONE:
                    _v, reason, _r, wkey = NONE_BODY.unpack_from(payload)
                    res = {"none": NONE_REASONS.get(reason, "code%d" % reason), "want_key": wkey}
            except (ValueError, struct.error) as exc:
                self.counts["bad_%s:%s" % ("frame" if mtype == CFRAME else "none", exc)] += 1
                return
            with self.cond:
                t = self.want_t.pop(want_id, None)
                if t is None:
                    self.counts["late_or_unknown"] += 1
                    return
                if mtype == CFRAME:
                    self.lat_ms.append((time.monotonic() - t) * 1e3)
                    self.counts["frames"] += 1
                else:
                    self.counts["none:%s" % res["none"]] += 1
                self.results[want_id] = res
                while len(self.results) > self.keep:
                    self.results.popitem(last=False)
                self.cond.notify_all()
            return
        self.counts["unexpected_type:%d" % mtype] += 1

    def has_source(self):
        with self.cond:
            return self.source is not None

    def want(self, frame_id, capture_ns, key, tol_ns=None):
        """Ask the source for the colour of this geometry frame. False when no source is known.
        ``tol_ns``: this WANT's tolerance (default the sink's --ar-colour-tol-ms); the no-depth
        self pull of host/kv_ar.py (AR_DEPTH_INVALID_STALL_20261017) asks with a wider one. The
        bytes are the same WANT_BODY either way (the tolerance field always existed)."""
        with self.cond:
            if self.source is None:
                self.counts["want_no_source"] += 1
                return False
            tol = pair_tol_ns(self.tol_ns, self.source[1]) if tol_ns is None else int(tol_ns)
            self.next_want += 1
            wid = self.next_want
            self.wants[(int(frame_id), int(capture_ns))] = wid
            while len(self.wants) > self.keep:
                old = self.wants.popitem(last=False)[1]
                self.want_t.pop(old, None)
                self.want_tol.pop(old, None)
            self.want_t[wid] = time.monotonic()
            self.want_tol[wid] = tol
            addr = self.source[0]
        body = encode_want(key, tol, frame_id, capture_ns)
        try:
            for d in split(CWANT, 0, wid, body, CHUNK_MTU):
                self.sock.sendto(d, addr)
            self.counts["wants"] += 1
        except OSError:
            self.counts["want_send_error"] += 1
            return False
        return True

    def take(self, frame_id, capture_ns, timeout_s):
        """(result dict with "rgb", None) or (None, reason). Waits at most ``timeout_s``."""
        end = time.monotonic() + max(0.0, float(timeout_s))
        with self.cond:
            wid = self.wants.get((int(frame_id), int(capture_ns)))
            if wid is None:
                return None, "not_wanted"
            while wid not in self.results:
                left = end - time.monotonic()
                if left <= 0:
                    self.counts["take_timeout"] += 1
                    return None, "timeout"
                self.cond.wait(left)
            res = self.results.pop(wid)
            self.wants.pop((int(frame_id), int(capture_ns)), None)
            tol = self.want_tol.pop(wid, self.tol_ns)
        if "none" in res:
            return None, "none:%s" % res["none"]
        if abs(int(res["source_key"]) - int(res["want_key"])) > tol:
            self.counts["key_far"] += 1                   # the source broke the tolerance: refuse
            return None, "key_far"
        return res, None

    def describe(self):
        with self.cond:
            src = self.source
            lat = sorted(self.lat_ms)
        return {"port": self.port, "rcvbuf": list(self.rcvbuf), "tol_ns": self.tol_ns,
                "pair_tol_ns": pair_tol_ns(self.tol_ns, src[1] if src else None),   # 10-29: what WANTs use
                "source": None if src is None else {"addr": "%s:%d" % src[0], "hello": src[1],
                                                   "since_hello_s": round(time.monotonic() - src[2], 3)},
                "want_to_frame_ms_p50": lat[len(lat) // 2] if lat else None,
                "counts": dict(self.counts)}


# ------------------------------------------------------------------ Nano side
class ColourRing:
    """Last ``n`` colour frames by sensor key (live: the camera header stamp in ns)."""

    def __init__(self, n=12):
        self.lock = threading.Lock()
        self.items = deque(maxlen=int(n))     # (key, width, height, rgb bytes)
        self.counts = Counter()

    def put(self, key, width, height, rgb):
        with self.lock:
            self.items.append((int(key), int(width), int(height), bytes(rgb)))
            self.counts["put"] += 1

    def lookup(self, key, tol_ns):
        """(source_key, width, height, rgb) of the nearest frame within tol_ns, else a reason str."""
        with self.lock:
            if not self.items:
                return "no_frames"
            best = min(self.items, key=lambda it: abs(it[0] - int(key)))
        if abs(best[0] - int(key)) > int(tol_ns):
            return "no_match"
        return best

    def newest(self, after_key=0):
        """(key, width, height, rgb) of the newest frame whose key is > after_key, else None (DPU pull)."""
        with self.lock:
            if not self.items:
                return None
            best = max(self.items, key=lambda it: it[0])
        return best if best[0] > int(after_key) else None


class ColourSource:
    """Nano end: HELLO to every receiver in ``kv_addr`` (one (host, port) or a list) every ``hello_s``;
    answers each WANT, to the address it came from, with the colour frame ``provider(key, tol_ns)``
    returns ((source_key, w, h, rgb) or a reason string). With ``latest`` (callable after_key -> (key, w,
    h, rgb) or None, e.g. ColourRing.newest) it also answers WANT_DPU: newest frame, preprocessed by
    host/dpu_prep.DpuPrep for the asked size/fix_point, held up to the WANT's wait_ms (long poll)."""

    REASON_CODES = {"no_match": 1, "no_frames": 2, "error": 3, "no_new": 4}

    def __init__(self, kv_addr, provider, *, info=None, chunk_bytes=CHUNK_FRAGMENTED, hello_s=0.5,
                 sock=None, latest=None, dpu_backend="auto", bind=None):
        addrs = [kv_addr] if isinstance(kv_addr, tuple) or (kv_addr and isinstance(kv_addr[0], str)) \
            else list(kv_addr)
        self.kvs = [(a[0], int(a[1])) for a in addrs]
        self.kv = self.kvs[0]
        self.provider = provider
        self.latest = latest
        self.dpu_backend = dpu_backend
        self.info = dict(info or {}, ver=VER, format="rgb8", dpu=latest is not None)
        self.chunk_bytes = int(chunk_bytes)
        self.hello_s = float(hello_s)
        self.sock = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if bind is not None and sock is None:
            self.sock.bind(bind)
        self.sock.settimeout(0.05)
        self.session = int.from_bytes(os.urandom(8), "little")
        self.counts = Counter()
        self.rx = Reassembler(self.counts, prefix="rx_")
        self._stop = threading.Event()
        self._thread = None
        self._hello_seq = 0
        self._preps = {}                    # (size, fix_point) -> DpuPrep
        self._pending = []                  # held WANT_DPU: [deadline, addr, want_id, want dict, t_rx]
        self.dpu_seq = 0
        self.dpu_prep_ms = deque(maxlen=512)

    def start(self):
        self._thread = threading.Thread(target=self._run, name="colour-src", daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(2.0)
        self.sock.close()

    def _send(self, mtype, fid, body, chunk, addr=None):
        addr = self.kv if addr is None else addr
        for d in split(mtype, self.session, fid, body, chunk):
            # blocking send, no timeout: the socket timeout is for recvfrom only
            self.sock.settimeout(None)
            try:
                self.sock.sendto(d, addr)
            finally:
                self.sock.settimeout(0.05)

    def _send_none(self, addr, want_id, reason, key):
        self._send(CNONE, want_id, NONE_BODY.pack(VER, self.REASON_CODES.get(reason, 3), b"\0\0", int(key)),
                   CHUNK_MTU, addr)

    def _run(self):
        last_hello = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now - last_hello >= self.hello_s:
                self._hello_seq += 1
                body = json.dumps(self.info).encode()
                for kv in self.kvs:
                    try:
                        self._send(CHELLO, self._hello_seq, body, CHUNK_MTU, kv)
                        self.counts["hello_sent"] += 1
                    except OSError:
                        self.counts["hello_send_error"] += 1
                last_hello = now
            if self._pending:
                self._serve_pending()
            self.sock.settimeout(0.002 if self._pending else 0.05)
            try:
                data, addr = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                self.counts["recv_error"] += 1
                continue
            got = self.rx.feed(data)
            if got is None:
                continue
            if got[0] == CWANT_DPU:
                self.rx.forget_done(CWANT_DPU)
                self._want_dpu(addr, got[3], got[4])
                continue
            if got[0] != CWANT:
                continue
            want_id, payload = got[3], got[4]
            self.rx.forget_done(CWANT)          # a restarted KV counts want ids from 1 again
            try:
                w = decode_want(payload)
            except (ValueError, struct.error):
                self.counts["bad_want"] += 1
                continue
            self.counts["wants"] += 1
            try:
                hit = self.provider(w["key"], w["tol_ns"])
            except Exception:
                self.counts["provider_error"] += 1
                hit = "error"
            try:
                if isinstance(hit, str):
                    self.counts["none:%s" % hit] += 1
                    self._send_none(addr, want_id, hit, w["key"])
                else:
                    skey, width, height, rgb = hit
                    self._send(CFRAME, want_id, encode_frame(skey, w["key"], width, height, rgb),
                               self.chunk_bytes, addr)
                    self.counts["frames_sent"] += 1
            except (OSError, ValueError) as exc:
                self.counts["send_error:%s" % type(exc).__name__] += 1

    # ---------------------------------------------------------------- DPU input
    def _want_dpu(self, addr, want_id, payload):
        try:
            w = decode_want_dpu(payload)
        except (ValueError, struct.error):
            self.counts["bad_want_dpu"] += 1
            return
        self.counts["dpu_wants"] += 1
        if self.latest is None:
            self.counts["none:dpu_off"] += 1
            self._send_none(addr, want_id, "error", w["after_key"])
            return
        # a newer WANT from the same receiver replaces its held one
        self._pending = [p for p in self._pending if p[1] != addr]
        t = time.monotonic()
        self._pending.append([t + w["wait_ms"] / 1e3, addr, want_id, w, t])
        self._serve_pending()

    def _serve_pending(self):
        now = time.monotonic()
        keep = []
        for item in self._pending:
            deadline, addr, want_id, w, t_rx = item
            try:
                hit = self.latest(w["after_key"])
            except Exception:
                self.counts["latest_error"] += 1
                hit = None
            if hit is None:
                if now >= deadline:
                    self.counts["none:no_new"] += 1
                    try:
                        self._send_none(addr, want_id, "no_new", w["after_key"])
                    except OSError:
                        self.counts["send_error:OSError"] += 1
                else:
                    keep.append(item)
                continue
            self._send_dpu(addr, want_id, w, hit, t_rx)
        self._pending = keep

    def warm_dpu(self, size, fix_point, src_w=640, src_h=480):
        """Build the DpuPrep for (size, fix_point) and run it once (cv2 self-test, resize tables), so the
        first WANT_DPU does not pay ~0.1 s. Call before start()."""
        from host.dpu_prep import DpuPrep
        import numpy as np
        prep = self._preps.get((int(size), int(fix_point)))
        if prep is None:
            prep = self._preps[(int(size), int(fix_point))] = DpuPrep(size, fix_point, self.dpu_backend)
        _g, _c, be = prep.content(np.zeros((int(src_h), int(src_w), 3), np.uint8))
        return be

    def _send_dpu(self, addr, want_id, w, hit, t_rx):
        from host.dpu_prep import DpuPrep
        skey, width, height, rgb = hit
        try:
            prep = self._preps.get((w["size"], w["fix_point"]))
            if prep is None:
                prep = self._preps[(w["size"], w["fix_point"])] = DpuPrep(w["size"], w["fix_point"],
                                                                           self.dpu_backend)
            import numpy as np
            t0 = time.monotonic()
            img = np.frombuffer(rgb, np.uint8).reshape(int(height), int(width), 3)
            g, content, be = prep.content(img)
            prep_ns = int((time.monotonic() - t0) * 1e9)
            self.dpu_prep_ms.append(prep_ns / 1e6)
            self.dpu_seq += 1
            body = encode_frame_dpu(g, content, fix_point=w["fix_point"], pad_q=prep.pad_q, backend=be,
                                    source_key=skey, after_key=w["after_key"], kv_t_ns=w["kv_t_ns"],
                                    prep_ns=prep_ns, hold_ns=int((t0 - t_rx) * 1e9), seq=self.dpu_seq)
        except Exception as exc:
            self.counts["dpu_prep_error:%s" % type(exc).__name__] += 1
            try:
                self._send_none(addr, want_id, "error", w["after_key"])
            except OSError:
                pass
            return
        try:
            self._send(CFRAME_DPU, want_id, body, w["chunk_bytes"], addr)
            self.counts["dpu_frames_sent"] += 1
        except OSError as exc:
            self.counts["send_error:%s" % type(exc).__name__] += 1

    def describe(self):
        lat = sorted(self.dpu_prep_ms)
        return {"kv": "%s:%d" % self.kv, "kvs": ["%s:%d" % a for a in self.kvs], "info": self.info,
                "counts": dict(self.counts),
                "dpu_prep_ms_p50": lat[len(lat) // 2] if lat else None}
