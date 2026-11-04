"""RFC 6455 WebSocket server side (text + binary, no extensions) and the HTTP-side checks.

Python 3.8+ standard library only (runs on the Windows laptop, design D2). Everything here is
socket-free so it can be unit tested byte by byte:

  * ``encode_frame``  -- server frames (never masked, FIN set unless asked)
  * ``FrameDecoder``  -- incremental client-frame parser: masking required, 7/16/64-bit lengths,
                         fragmentation (control frames may interleave), size caps, UTF-8 check,
                         close codes; protocol violations raise ``WsError(close_code, why)``
  * ``check_upgrade`` -- Host / Origin / Upgrade / Connection / Version / key checks (§3.6)
  * ``accept_key``    -- Sec-WebSocket-Accept
  * ``safe_static_path`` -- whitelist + realpath containment for the app/ static files
"""

import base64
import codecs
import hashlib
import os
import struct

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

OP_CONT, OP_TEXT, OP_BINARY, OP_CLOSE, OP_PING, OP_PONG = 0x0, 0x1, 0x2, 0x8, 0x9, 0xA
DATA_OPS = (OP_TEXT, OP_BINARY)
CONTROL_OPS = (OP_CLOSE, OP_PING, OP_PONG)

# close codes (RFC 6455 7.4.1)
CLOSE_NORMAL, CLOSE_GOING_AWAY, CLOSE_PROTOCOL, CLOSE_UNSUPPORTED = 1000, 1001, 1002, 1003
CLOSE_BAD_DATA, CLOSE_POLICY, CLOSE_TOO_BIG, CLOSE_INTERNAL = 1007, 1008, 1009, 1011
# codes a peer may put on the wire (1004/1005/1006/1015 are reserved / never sent)
VALID_CLOSE_CODES = frozenset((1000, 1001, 1002, 1003, 1007, 1008, 1009, 1010, 1011))

DEFAULT_MAX_MESSAGE = 1 << 20          # inbound (browser -> bridge) cap: commands are tiny
SERVER_MAX_FRAME = 16 << 20            # outbound single-frame cap (§3.1: 单帧 <= 16 MB)

STATIC_TOP = ("index.html", "styles.css", "manifest.webmanifest", "sw.js", "src", "vendor", "assets",
              "kiosk.html", "kiosk.css", "competition.html")


class WsError(Exception):
    """A protocol violation: close the connection with ``code``."""

    def __init__(self, code, why):
        Exception.__init__(self, "%d %s" % (code, why))
        self.code, self.why = code, why


def accept_key(key):
    digest = hashlib.sha1((key + GUID).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def encode_frame(opcode, payload=b"", fin=True, mask_key=None):
    """One frame. Server frames are unmasked; ``mask_key`` (4 bytes) is for test clients."""
    payload = bytes(payload)
    n = len(payload)
    if opcode in CONTROL_OPS and (n > 125 or not fin):
        raise ValueError("control frame must be FIN and <= 125 bytes")
    if mask_key is None and n > SERVER_MAX_FRAME:
        raise ValueError("frame larger than %d bytes" % SERVER_MAX_FRAME)
    b0 = (0x80 if fin else 0) | (opcode & 0x0F)
    mbit = 0x80 if mask_key is not None else 0
    if n < 126:
        head = struct.pack("!BB", b0, mbit | n)
    elif n < 65536:
        head = struct.pack("!BBH", b0, mbit | 126, n)
    else:
        head = struct.pack("!BBQ", b0, mbit | 127, n)
    if mask_key is None:
        return head + payload
    mask_key = bytes(mask_key)
    if len(mask_key) != 4:
        raise ValueError("mask key must be 4 bytes")
    return head + mask_key + apply_mask(payload, mask_key)


def apply_mask(data, key):
    """XOR with the 4-byte key (involution). int-wide XOR keeps 1 MB under a few ms."""
    data = bytes(data)
    n = len(data)
    if n == 0:
        return b""
    reps = (n + 3) // 4
    k = int.from_bytes((key * reps)[:n], "little")
    return (int.from_bytes(data, "little") ^ k).to_bytes(n, "little")


def close_payload(code, reason=""):
    raw = reason.encode("utf-8")[:123]
    while True:                                  # never cut a UTF-8 sequence in half
        try:
            raw.decode("utf-8")
            break
        except UnicodeDecodeError:
            raw = raw[:-1]
    return struct.pack("!H", code) + raw


def parse_close(payload):
    """(code|None, reason) of a received close frame; malformed -> WsError(1002/1007)."""
    if len(payload) == 0:
        return None, ""
    if len(payload) == 1:
        raise WsError(CLOSE_PROTOCOL, "close payload of 1 byte")
    (code,) = struct.unpack("!H", payload[:2])
    if code not in VALID_CLOSE_CODES and not 3000 <= code <= 4999:
        raise WsError(CLOSE_PROTOCOL, "invalid close code %d" % code)
    try:
        reason = payload[2:].decode("utf-8")
    except UnicodeDecodeError:
        raise WsError(CLOSE_BAD_DATA, "close reason not UTF-8")
    return code, reason


class FrameDecoder(object):
    """Feed raw client bytes, get whole events back.

    ``feed(data)`` returns a list of ``(kind, value)``: ("text", str), ("binary", bytes),
    ("ping", bytes), ("pong", bytes), ("close", (code|None, reason)). After a close event or a
    raised WsError the decoder must not be fed again.
    """

    def __init__(self, max_message=DEFAULT_MAX_MESSAGE, require_mask=True):
        self.max_message = int(max_message)
        self.require_mask = require_mask
        self.buf = bytearray()
        self.frag_op = None                     # opcode of the message being reassembled
        self.frag = []
        self.frag_len = 0
        self.utf8 = None                        # incremental decoder for fragmented text
        self.closed = False

    def feed(self, data):
        if self.closed:
            raise WsError(CLOSE_PROTOCOL, "data after close")
        self.buf += data
        out = []
        while True:
            frame = self._one()
            if frame is None:
                return out
            ev = self._event(*frame)
            if ev is not None:
                out.append(ev)
                if ev[0] == "close":
                    self.closed = True
                    return out

    def _one(self):
        b = self.buf
        if len(b) < 2:
            return None
        b0, b1 = b[0], b[1]
        fin, rsv, op = bool(b0 & 0x80), b0 & 0x70, b0 & 0x0F
        masked, n = bool(b1 & 0x80), b1 & 0x7F
        if rsv:
            raise WsError(CLOSE_PROTOCOL, "RSV bits set without an extension")
        if op not in DATA_OPS + CONTROL_OPS + (OP_CONT,):
            raise WsError(CLOSE_PROTOCOL, "reserved opcode %d" % op)
        if self.require_mask and not masked:
            raise WsError(CLOSE_PROTOCOL, "client frame not masked")
        pos = 2
        if n == 126:
            if len(b) < 4:
                return None
            (n,) = struct.unpack_from("!H", b, 2)
            if n < 126:
                raise WsError(CLOSE_PROTOCOL, "non-minimal 16-bit length")
            pos = 4
        elif n == 127:
            if len(b) < 10:
                return None
            (n,) = struct.unpack_from("!Q", b, 2)
            if n >> 63:
                raise WsError(CLOSE_PROTOCOL, "64-bit length with the top bit set")
            if n < 65536:
                raise WsError(CLOSE_PROTOCOL, "non-minimal 64-bit length")
            pos = 10
        if op in CONTROL_OPS and (n > 125 or not fin):
            raise WsError(CLOSE_PROTOCOL, "control frame fragmented or > 125 bytes")
        if op not in CONTROL_OPS and n > self.max_message:
            raise WsError(CLOSE_TOO_BIG, "frame of %d bytes > %d" % (n, self.max_message))
        if op in (OP_CONT,) + DATA_OPS and self.frag_len + n > self.max_message:
            raise WsError(CLOSE_TOO_BIG, "message > %d bytes" % self.max_message)
        key = None
        if masked:
            if len(b) < pos + 4:
                return None
            key = bytes(b[pos:pos + 4])
            pos += 4
        if len(b) < pos + n:
            return None
        payload = bytes(b[pos:pos + n])
        del b[:pos + n]
        if key is not None:
            payload = apply_mask(payload, key)
        return fin, op, payload

    def _event(self, fin, op, payload):
        if op == OP_PING:
            return ("ping", payload)
        if op == OP_PONG:
            return ("pong", payload)
        if op == OP_CLOSE:
            return ("close", parse_close(payload))
        if op == OP_CONT:
            if self.frag_op is None:
                raise WsError(CLOSE_PROTOCOL, "continuation without a started message")
        else:
            if self.frag_op is not None:
                raise WsError(CLOSE_PROTOCOL, "new data frame inside a fragmented message")
            self.frag_op, self.frag, self.frag_len = op, [], 0
            self.utf8 = codecs.getincrementaldecoder("utf-8")() if op == OP_TEXT else None
        self.frag.append(payload)
        self.frag_len += len(payload)
        if self.utf8 is not None:
            try:
                self.utf8.decode(payload, final=fin)       # fail fast on invalid UTF-8
            except UnicodeDecodeError:
                raise WsError(CLOSE_BAD_DATA, "text frame not valid UTF-8")
        if not fin:
            return None
        data = b"".join(self.frag)
        kind = self.frag_op
        self.frag_op, self.frag, self.frag_len, self.utf8 = None, [], 0, None
        if kind == OP_TEXT:
            return ("text", data.decode("utf-8"))
        return ("binary", data)


# ---------------------------------------------------------------------------- HTTP side

def _ports(port):
    """One port or several (10-21: the Nano gateway is reached through ``ssh -L <public>:...:<listen>``,
    so the browser's Host carries the public port, not the one the socket is bound to)."""
    return (port,) if isinstance(port, int) else tuple(port)


def allowed_hosts(port):
    return tuple(h % p for p in _ports(port) for h in ("127.0.0.1:%d", "localhost:%d"))


def allowed_origins(port):
    return tuple("http://" + h for h in allowed_hosts(port))


def check_host(headers, port):
    """Host must be exactly 127.0.0.1:<port> or localhost:<port> (DNS rebinding guard)."""
    host = (headers.get("Host") or "").strip().lower()
    return host in allowed_hosts(port)


def _tokens(value):
    return [t.strip().lower() for t in (value or "").split(",") if t.strip()]


def check_upgrade(method, headers, port):
    """(ok, http_status, reason, accept). ``headers`` is any mapping with case-insensitive
    ``get`` (http.client.HTTPMessage) or a plain dict with canonical names."""
    if method != "GET":
        return False, 405, "method", None
    if not check_host(headers, port):
        return False, 403, "host", None
    origin = headers.get("Origin")
    if origin is None:
        return False, 403, "origin_missing", None
    origin = origin.strip()
    host = (headers.get("Host") or "").strip().lower()
    if origin not in allowed_origins(port) or origin.lower() != "http://" + host:   # same origin as the page
        return False, 403, "origin", None
    if "websocket" not in _tokens(headers.get("Upgrade")):
        return False, 400, "upgrade", None
    if "upgrade" not in _tokens(headers.get("Connection")):
        return False, 400, "connection", None
    if (headers.get("Sec-WebSocket-Version") or "").strip() != "13":
        return False, 426, "version", None
    key = (headers.get("Sec-WebSocket-Key") or "").strip()
    try:
        raw = base64.b64decode(key.encode("ascii"), validate=True)
    except (ValueError, UnicodeEncodeError):
        raw = b""
    if len(raw) != 16:
        return False, 400, "key", None
    return True, 101, None, accept_key(key)


def safe_static_path(url_path, app_root):
    """Real file path under ``app_root`` for a request path, or None (404).

    Query/fragment are stripped by the caller. Percent-decoding happens here; the first path
    segment must be one of STATIC_TOP and the resolved real path (symlinks followed) must stay
    inside the real path of that whitelisted top entry."""
    try:
        from urllib.parse import unquote
        rel = unquote(url_path, errors="strict")
    except UnicodeDecodeError:
        return None
    if "\x00" in rel or "\\" in rel:
        return None
    rel = rel.lstrip("/") or "index.html"
    first = rel.split("/", 1)[0]
    if first not in STATIC_TOP:
        return None
    root = os.path.realpath(app_root)
    top = os.path.realpath(os.path.join(root, first))
    if not (top == root or top.startswith(root + os.sep)):
        return None
    full = os.path.realpath(os.path.join(root, rel))
    if not (full == top or full.startswith(top + os.sep)):
        return None
    if not os.path.isfile(full):
        return None
    return full
