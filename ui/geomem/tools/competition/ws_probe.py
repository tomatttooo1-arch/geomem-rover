#!/usr/bin/env python3
"""Read-only probe of the Nano gateway WebSocket from the Nano itself (no network in between).

Behaves like the page: acks every binary message ({"type":"frame_ack"}), reassembles 10-22 chunk messages
({type:'chunk', id, i, n, kind}) and counts the reassembled payload under its inner type. Prints per-kind counts,
sizes and arrival gaps (snapshot gap = what the page's freshness check sees), chunks per payload, the mapping
message modes (full / delta / keep) and the snapshot link.state history.

    python3 ws_probe.py <seconds> [public_port]       (bridge on 127.0.0.1:8790; Host/Origin use public_port)
"""
import base64
import collections
import json
import os
import socket
import struct
import sys
import time


def main():
    dur = float(sys.argv[1])
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8788
    s = socket.create_connection(("127.0.0.1", 8790), timeout=5)
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall(("GET /ws HTTP/1.1\r\nHost: 127.0.0.1:%d\r\nOrigin: http://127.0.0.1:%d\r\nUpgrade: websocket\r\n"
               "Connection: Upgrade\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Key: %s\r\n\r\n"
               % (port, port, key)).encode())
    st = {"buf": b""}
    while b"\r\n\r\n" not in st["buf"]:
        st["buf"] += s.recv(4096)
    head, st["buf"] = st["buf"].split(b"\r\n\r\n", 1)
    assert b" 101 " in head.split(b"\r\n")[0], head[:200]

    def masked_text(txt):
        p = txt.encode()
        m = os.urandom(4)
        return bytes([0x81, 0x80 | len(p)]) + m + bytes(b ^ m[i % 4] for i, b in enumerate(p))

    def need(n):
        while len(st["buf"]) < n:
            d = s.recv(1 << 16)
            if not d:
                raise EOFError
            st["buf"] += d

    t0 = time.monotonic()
    last, gaps, sizes = {}, collections.defaultdict(list), collections.defaultdict(list)
    chunks = collections.defaultdict(list)          # inner kind -> chunks per completed payload
    parts = {}                                      # chunk kind -> {"id", "got", "buf"}
    dropped = collections.Counter()
    modes = collections.Counter()
    links = []
    hello = None
    s.settimeout(2.5)

    def note(kind, size, now):
        if kind in last:
            gaps[kind].append(now - last[kind])
        last[kind] = now
        sizes[kind].append(size)

    def binary(payload, now, nchunks=None):
        L = struct.unpack("<I", payload[:4])[0]
        try:
            h = json.loads(payload[4:4 + L])
        except ValueError:
            note("bin:?", len(payload), now)
            return
        kind = "bin:" + str(h.get("type", "?"))
        if h.get("type") == "chunk":
            body = payload[4 + L + ((-(4 + L)) % 4):]
            ck = h.get("kind")
            p = parts.get(ck)
            if h.get("i") == 0:
                if p:
                    dropped[ck] += 1
                p = parts[ck] = {"id": h.get("id"), "got": 0, "buf": b""}
            elif not p or p["id"] != h.get("id") or p["got"] != h.get("i"):
                dropped[ck] += 1
                parts.pop(ck, None)
                return
            p["buf"] += body
            p["got"] += 1
            if p["got"] == h.get("n"):
                del parts[ck]
                binary(p["buf"], now, h.get("n"))
            return
        if h.get("type") == "mapping":
            modes[h.get("mode") or ("q5z" if h.get("encoding") == "q5z" else "legacy-keep")] += 1
        chunks[kind].append(nchunks or 1)
        note(kind, len(payload), now)

    while time.monotonic() - t0 < dur:
        try:
            need(2)
        except socket.timeout:
            print("%.2f  NOTHING for 2.5 s" % (time.monotonic() - t0))
            continue
        b = st["buf"]
        op, n, off = b[0] & 0x0F, b[1] & 0x7F, 2
        if n == 126:
            need(4); n = struct.unpack(">H", st["buf"][2:4])[0]; off = 4
        elif n == 127:
            need(10); n = struct.unpack(">Q", st["buf"][2:10])[0]; off = 10
        need(off + n)
        payload, st["buf"] = st["buf"][off:off + n], st["buf"][off + n:]
        now = time.monotonic() - t0
        if op == 1:
            try:
                obj = json.loads(payload)
                kind = obj.get("type", "?")
            except ValueError:
                obj, kind = {}, "text?"
            if kind == "snapshot":
                state = (obj.get("link") or {}).get("state")
                if not links or links[-1][1] != state:
                    links.append((round(now, 2), state))
            if kind == "hello":
                hello = obj.get("gateway")
            note(kind, n, now)
        elif op == 2:
            s.sendall(masked_text('{"type":"frame_ack"}'))
            binary(payload, now)
        elif op == 9:
            note("ping", n, now)
        else:
            note("op%d" % op, n, now)

    def pct(v, q):
        v = sorted(v)
        return v[min(len(v) - 1, int(q * len(v)))] if v else float("nan")

    print("gateway in hello:", hello)
    print("kind              n    size_med   size_max  chunks_max   gap_med  gap_p95  gap_max  gaps>0.5s  gaps>2s")
    for k in sorted(sizes):
        g = gaps[k]
        print("%-15s %5d %10d %10d %10s %8.3f %8.3f %8.3f %8d %8d" % (
            k, len(sizes[k]), pct(sizes[k], .5), max(sizes[k]), max(chunks[k]) if chunks.get(k) else "-",
            pct(g, .5), pct(g, .95), max(g) if g else float("nan"), sum(x > .5 for x in g), sum(x > 2 for x in g)))
    if modes:
        print("mapping messages:", dict(modes))
    if dropped:
        print("incomplete chunk payloads dropped:", dict(dropped))
    print("link.state changes:", links[:20])


if __name__ == "__main__":
    main()
