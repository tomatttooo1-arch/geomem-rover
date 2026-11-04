#!/usr/bin/env python3
"""10-22 slow-link bench for the bridge's per-browser sender (WsSession): does a big mapping payload hold back the
snapshots behind it?

A real WsSession sender thread writes into one end of a socketpair; a consumer thread drains the other end at a
fixed byte rate (the WiFi/ssh path), parses WS frames, and -- like the page -- acks every binary message after one
round trip (RTT). Producers: a 1.5 KB snapshot every 0.2 s, a preview every 0.1 s (hub publishes 10 fps), and a
mapping payload of --mapping-bytes every --mapping-period s.

    python tests/bench_gateway_slow_link.py [--seconds 20] [--rate 2e6] [--rtt 0.03] [--old-bridge FILE] [--arm new]

--old-bridge: a geomem_bridge.py from another commit (e.g. `git show 18635ff:ui/geomem/bridge/geomem_bridge.py`)
for the before/after comparison; its WsSession is driven by the same harness.
"""
import argparse
import importlib.util
import json
import os
import socket
import struct
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BRIDGE_DIR = ROOT / "ui/geomem/bridge"
sys.path.insert(0, str(BRIDGE_DIR))
import geomem_bridge as gb  # noqa: E402


def load_bridge(path):
    spec = importlib.util.spec_from_file_location("old_geomem_bridge", str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))] if v else float("nan")


class Consumer(threading.Thread):
    """Rate-limited reader + page behaviour (ack each binary message after rtt, reassemble chunks)."""

    def __init__(self, sock, session, rate, rtt):
        threading.Thread.__init__(self, daemon=True)
        self.sock, self.session, self.rate, self.rtt = sock, session, rate, rtt
        self.buf = b""
        self.snap_t, self.preview, self.mapping, self.parts = [], [], [], {}
        self.bytes = 0
        self.stop = False
        self.timers = []

    def ack_later(self):
        t = threading.Timer(self.rtt, self.session.on_frame_ack)
        t.daemon = True
        t.start()

    def run(self):
        self.sock.settimeout(0.2)
        next_t = time.monotonic()
        while not self.stop:
            try:
                data = self.sock.recv(16384)
            except socket.timeout:
                continue
            except OSError:
                return
            if not data:
                return
            self.bytes += len(data)
            now = time.monotonic()
            next_t = max(next_t, now) + len(data) / self.rate
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            self.buf += data
            self.parse(time.monotonic())

    def parse(self, now):
        while len(self.buf) >= 2:
            b = self.buf
            op, n, j = b[0] & 15, b[1] & 127, 2
            if n == 126:
                if len(b) < 4:
                    return
                n = struct.unpack_from(">H", b, 2)[0]; j = 4
            elif n == 127:
                if len(b) < 10:
                    return
                n = struct.unpack_from(">Q", b, 2)[0]; j = 10
            if len(b) < j + n:
                return
            payload, self.buf = b[j:j + n], b[j + n:]
            if op == 1:
                if b'"type":"snapshot"' in payload[:200]:
                    self.snap_t.append(now)
            elif op == 2:
                self.ack_later()
                self.binary(payload, now)

    def binary(self, frame, now):
        (L,) = struct.unpack_from("<I", frame, 0)
        head = json.loads(frame[4:4 + L].decode())
        body = frame[4 + L + ((-(4 + L)) % 4):]
        if head["type"] == "chunk":
            st = self.parts.get(head["kind"])
            if head["i"] == 0:
                st = self.parts[head["kind"]] = {"id": head["id"], "got": 0, "buf": b""}
            elif st is None or st["id"] != head["id"] or st["got"] != head["i"]:
                self.parts.pop(head["kind"], None)
                return
            st["buf"] += body
            st["got"] += 1
            if st["got"] == head["n"]:
                del self.parts[head["kind"]]
                self.binary(st["buf"], now)
            return
        if head["type"] == "preview":
            self.preview.append((now, head["t"]))
        elif head["type"] == "mapping":
            self.mapping.append((now, head["t"], len(frame)))


def run_arm(mod, seconds, rate, rtt, mapping_bytes, mapping_period, preview_bytes, preview_hz, round_robin=True,
            chunked=True, window=1, mapping_first_bytes=None):
    a, b = socket.socketpair()
    sndbuf, rcvbuf = a.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF), b.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
    bridge = mod.Bridge()
    kwargs = {"chunked": chunked} if "chunked" in mod.WsSession.__init__.__code__.co_varnames else {}
    if hasattr(mod, "BULK_ROUND_ROBIN"):
        mod.BULK_ROUND_ROBIN = round_robin
        mod.BULK_WINDOW = window
    s = mod.WsSession(a, None, bridge, **kwargs)
    c = Consumer(b, s, rate, rtt)
    c.start()
    s.start_sender()
    t0 = time.monotonic()
    pad = "x" * 1400
    jpeg = b"\xff\xd8" + os.urandom(preview_bytes - 4) + b"\xff\xd9"
    blob = os.urandom(mapping_bytes)
    first = os.urandom(mapping_first_bytes) if mapping_first_bytes else blob
    next_snap = next_prev = next_map = t0
    sent = {"preview": 0, "mapping": 0}
    while True:
        now = time.monotonic()
        if now - t0 >= seconds:
            break
        if now >= next_snap:
            s.put_snapshot({"link": {"state": "up"}, "pad": pad})
            next_snap += 0.2
        if now >= next_prev:
            s.put_preview(gb.encode_binary({"type": "preview", "version": 2, "t": now}, jpeg))
            sent["preview"] += 1
            next_prev += 1.0 / preview_hz
        if now >= next_map:
            s.put_mapping(gb.encode_binary({"type": "mapping", "version": 2, "t": now}, first if not sent["mapping"] else blob))
            sent["mapping"] += 1
            next_map += mapping_period
        time.sleep(max(0.0, min(next_snap, next_prev, next_map) - time.monotonic()))
    time.sleep(0.3)
    c.stop = True
    s.close()
    a.close()
    b.close()
    st = [x - t0 for x in c.snap_t]
    gaps = [y - x for x, y in zip(st, st[1:])]
    # a gap still open at the end counts too
    pv_lat = [arr - t for arr, t in c.preview]
    mp_lat = [arr - t for arr, t, _ in c.mapping]
    span = seconds
    return {"snapshots": len(st), "gap_p50": pct(gaps, .5), "gap_p95": pct(gaps, .95), "gap_max": max(gaps or [0]),
            "gaps_over_0.5s": sum(g > .5 for g in gaps), "gaps_over_2s": sum(g > 2 for g in gaps),
            "preview_fps": len(c.preview) / span, "preview_lat_p50": pct(pv_lat, .5), "preview_lat_p95": pct(pv_lat, .95),
            "mapping_done": len(c.mapping), "mapping_sent": sent["mapping"], "mapping_lat_p50": pct(mp_lat, .5),
            "mapping_lat_max": max(mp_lat or [float("nan")]),
            "throughput_MBps": c.bytes / span / 1e6, "sndbuf": sndbuf, "rcvbuf": rcvbuf}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20)
    ap.add_argument("--rate", type=float, default=2e6, help="link bytes/s")
    ap.add_argument("--rtt", type=float, default=0.03)
    ap.add_argument("--mapping-bytes", type=int, default=1500000)
    ap.add_argument("--mapping-period", type=float, default=2.4)
    ap.add_argument("--preview-bytes", type=int, default=50000)
    ap.add_argument("--preview-hz", type=float, default=10)
    ap.add_argument("--mapping-first-bytes", type=int, default=None, help="first mapping payload (a full before deltas)")
    ap.add_argument("--old-bridge", default=None)
    ap.add_argument("--arm", action="append", default=None, help="old | new | new-literal | new-w2 (repeatable)")
    args = ap.parse_args()
    arms = args.arm or (["old", "new", "new-literal"] if args.old_bridge else ["new", "new-literal"])
    common = dict(seconds=args.seconds, rate=args.rate, rtt=args.rtt, mapping_bytes=args.mapping_bytes,
                  mapping_period=args.mapping_period, preview_bytes=args.preview_bytes, preview_hz=args.preview_hz,
                  mapping_first_bytes=args.mapping_first_bytes)
    for arm in arms:
        if arm == "old":
            res = run_arm(load_bridge(args.old_bridge), **common)
        else:
            res = run_arm(gb, round_robin=(arm != "new-literal"), window=2 if arm == "new-w2" else 1, **common)
        print(json.dumps(dict(arm=arm, **{k: (round(v, 3) if isinstance(v, float) else v) for k, v in res.items()})))
        sys.stdout.flush()


if __name__ == "__main__":
    main()
