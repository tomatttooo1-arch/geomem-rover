#!/usr/bin/env python3
"""GOLIVE0 (2026-11-04): KV260 -> Nano status uplink of the single-board product.

Follows (tail) the files the two KV data-plane processes already write and sends one small JSON datagram to the
Nano task console every PERIOD seconds (UDP, fire and forget; the KV never waits for the Nano):

  depth.jsonl   kv_server --stamp-log  depth rows (geometry -> fusion + GRID cost map done) and "field" rows
                (planner potential field done) -> map status: frames, rates over the last WINDOW s, newest state
  colour.jsonl  dpu_rx --stamp-log     one row per recognised frame (flushed per row) -> recognition rate
  boxes.jsonl   dpu_rx --out-log       one row per frame with its boxes [x1, y1, x2, y2, score, cls] (stdio
                buffered by dpu_rx: boxes arrive in ~4 KB blocks, i.e. about 1-3 s late) -> newest boxes

Never refuses: a missing or truncated file is retried; a send error is counted and logged (rate limited).
Stdlib only (python3 on the KV). Each datagram is also appended to --log (one JSON line).
"""

import argparse
import collections
import json
import os
import socket
import sys
import time

MAX_BOXES = 12


class Tail:
    """Lines appended to a file since the last call (handles the file appearing late or being recreated)."""

    def __init__(self, path):
        self.path, self.fp, self.ino, self.buf = path, None, None, ""

    def lines(self):
        try:
            st = os.stat(self.path)
        except OSError:
            return []
        if self.fp is None or st.st_ino != self.ino or st.st_size < self.fp.tell():
            if self.fp is not None:
                self.fp.close()
            self.fp, self.ino, self.buf = open(self.path, "r", errors="replace"), st.st_ino, ""
        chunk = self.fp.read()
        if not chunk:
            return []
        self.buf += chunk
        parts = self.buf.split("\n")
        self.buf = parts.pop()
        out = []
        for p in parts:
            p = p.strip()
            if not p:
                continue
            try:
                out.append(json.loads(p))
            except ValueError:
                pass
        return out


class Rate:
    def __init__(self, window_s):
        self.w, self.t, self.n = window_s, collections.deque(), 0

    def add(self, now):
        self.t.append(now)
        self.n += 1

    def fps(self, now):
        while self.t and now - self.t[0] > self.w:
            self.t.popleft()
        return round(len(self.t) / self.w, 2)


def srch_view(path):
    """SRCHCHAIN0: the KV search service's newest status, compact (None until the file exists)."""
    try:
        st = json.load(open(path))
    except (OSError, ValueError):
        return None
    last = st.get("last") or {}
    cnt = st.get("counts") or {}
    return {"impl": st.get("impl"), "reason": st.get("reason"), "episodes": st.get("episodes"),
            "core_calls": cnt.get("core_calls"), "core_fails": cnt.get("core_fails"),
            "selfcheck_same": (st.get("selfcheck") or {}).get("same"), "n": last.get("n"),
            "target_i": last.get("target_i"), "target": last.get("target"), "first": last.get("first"),
            "order": [[l[0], l[3], l[4]] for l in last.get("looks") or []], "stop": last.get("stop"),
            "ms": last.get("ms")}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--depth", required=True)
    ap.add_argument("--colour", default=None)
    ap.add_argument("--boxes", default=None)
    ap.add_argument("--names", default=None, help="tools/dpu_pipe/recog/profiles.json (class names)")
    ap.add_argument("--nano", default="198.51.100.1:47950", help="host:port of nano_console.py")
    ap.add_argument("--period", type=float, default=0.5)
    ap.add_argument("--window", type=float, default=10.0, help="rate window, s")
    ap.add_argument("--log", default=None)
    ap.add_argument("--srch-status", default=None, help="SRCHCHAIN0: srch_chain.py serve --status file (SRCH=pl)")
    ap.add_argument("--mem-state", default=None, help="MEMCHAIN0: kv_memsvc.py --out file; sent as msg['mem']")
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = until SIGTERM")
    a = ap.parse_args(argv)
    names = []
    if a.names:
        try:
            names = json.load(open(a.names)).get("names", [])
        except (OSError, ValueError) as exc:
            print(json.dumps({"warning": "names: %r" % exc}), file=sys.stderr, flush=True)
    host, port = a.nano.rsplit(":", 1)
    dst = (host, int(port))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    td, tc, tb = Tail(a.depth), Tail(a.colour) if a.colour else None, Tail(a.boxes) if a.boxes else None
    r_depth, r_grid, r_field, r_col, r_box = (Rate(a.window) for _ in range(5))
    last_field, last_depth, last_boxes = {}, {}, {"frame": None, "dets": []}
    counts = collections.Counter()
    seen = collections.Counter()           # per class, frames with that class (since start)
    logf = open(a.log, "a") if a.log else None
    seq, t0, last_warn = 0, time.monotonic(), 0.0
    nxt = t0
    while True:
        now = time.monotonic()
        for r in td.lines():
            if r.get("ev") == "field":
                r_field.add(now)
                last_field = {k: r.get(k) for k in ("frame", "state", "reason", "plan_status", "plan_ms")}
                if r.get("pln"):        # PLNCHAIN0: PLN=gmap3 counters (path fetch / global map) go up to the Nano
                    last_field["pln"] = r["pln"]
            elif r.get("chain") == "depth":
                r_depth.add(now)
                if r.get("grid_ok"):
                    r_grid.add(now)
                last_depth = {"frame": r.get("frame"), "geo_status": r.get("geo_status"),
                              "fuse_ok": r.get("fuse_ok"), "grid_ok": r.get("grid_ok")}
        if tc is not None:
            for r in tc.lines():
                if r.get("chain") == "colour":
                    r_col.add(now)
        if tb is not None:
            for r in tb.lines():
                if "dets" not in r:
                    continue
                r_box.add(now)
                dets = r.get("dets") or []
                for c in {int(d[5]) for d in dets if len(d) >= 6}:
                    seen[c] += 1
                if dets:
                    top = sorted(dets, key=lambda d: -d[4])[:MAX_BOXES]
                    last_boxes = {"frame": r.get("seq"), "t_out": r.get("t_out"),
                                  "dets": [[round(d[0]), round(d[1]), round(d[2]), round(d[3]), round(d[4], 3),
                                            int(d[5]), names[int(d[5])] if int(d[5]) < len(names) else str(d[5])]
                                           for d in top]}
        if now >= nxt:
            nxt += a.period
            seq += 1
            msg = {"v": 1, "seq": seq, "t_mono_ns": time.monotonic_ns(), "up_s": round(now - t0, 1),
                   "map": {"depth_frames": r_depth.n, "depth_fps": r_depth.fps(now), "grid_frames": r_grid.n,
                           "grid_fps": r_grid.fps(now), "field_updates": r_field.n, "field_fps": r_field.fps(now),
                           "last_depth": last_depth, "last_field": last_field},
                   "recog": {"frames": r_col.n, "fps": r_col.fps(now), "box_rows": r_box.n,
                             "box_rows_fps": r_box.fps(now),
                             "seen_frames": {names[c] if c < len(names) else str(c): n for c, n in seen.items()},
                             "last": last_boxes}}
            if a.srch_status:
                msg["srch"] = srch_view(a.srch_status)
            if a.mem_state:
                # MEMCHAIN0: the KV object memory (kv_memsvc.py); missing / partial file -> no "mem" field this time
                try:
                    with open(a.mem_state) as fp:
                        msg["mem"] = json.load(fp)
                    counts["mem_attached"] += 1
                except (OSError, ValueError):
                    counts["mem_missing"] += 1
            data = json.dumps(msg, separators=(",", ":")).encode()
            try:
                sock.sendto(data, dst)
                counts["sent"] += 1
            except OSError as exc:
                counts["send_error"] += 1
                if now - last_warn > 10:
                    last_warn = now
                    print(json.dumps({"warning": "send %s: %r" % (a.nano, exc), "counts": dict(counts)}),
                          file=sys.stderr, flush=True)
            if logf is not None:
                logf.write(data.decode() + "\n")
                logf.flush()
            if a.seconds and now - t0 > a.seconds:
                break
        time.sleep(0.05)
    print(json.dumps({"exit": "seconds", "counts": dict(counts)}), flush=True)


if __name__ == "__main__":
    main()
