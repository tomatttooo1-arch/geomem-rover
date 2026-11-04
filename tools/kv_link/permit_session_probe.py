#!/usr/bin/env python3
"""Board probe: permit NEXT_FRAME_ID from the Nano frame id, per Nano session (owner 10-11).

Against a running ``kv_server.py --core pl --device devmem --wait irq`` (merge bitstream with
the permit block). Every "Nano process" is a real child process (``send`` subcommand) with
its own stream_instance (``permitprobe-<pid>-<rand>``) and its own link session, so a
"Nano restart" here is a real process restart. A TELEM watcher (own session, --watch-hz)
records the permit block for the whole run; PL register deltas decide each check.

Phases (``run``):
  P1 normal   process A sends ids 1..N at --hz: permit stays 1 from the first grant to the
              last answer, FRAMES_ACCEPTED +N, no revoke while sending.
  P2 expiry   nothing sent: revoked; LAST_CAUSE 1; (LAST_REVOKE_TS - LAST_DONE_TS)/clk = the
              measured revoke time (spec: TIMEOUT + 1 cycle); TELEM: last answer -> first
              permit=0 sample (upper bound, includes the sample period and TELEM RTT).
  P3 restart  new process B, ids 1..M (restart from 1): the first frame grants again.
  P4 late     right after B's last frame, a new process sends stream A, id N+1 (a late packet
              of the ended session A): nano feed -> no answer, FRAMES_ACCEPTED unchanged, the
              revoke still comes TIMEOUT after B's last accepted edge. (kv-counter control:
              answered, accepted, permit extended.)
  P5a dup     a new process on stream B sends ids M+1..M+K, then M+K again on the same link
              session: the link reassembler drops it (no core, no answer, no renewal) -- both
              feeds.
  P5b dup     another process on stream B sends M+K+1..M+2K, then M+2K on a fresh link session
              (duplicate) and M+2K-3 on another (regress) while permit is up: nano feed -> both
              rejected by the PL (FRAMES_REJECTED +2, LAST_CAUSE 3, permit 0 at once).
              (kv-counter control: both accepted, permit stays.)
Prints one JSON report (also --out). ``--expect nano|kv-counter`` selects the verdicts.

Usage: permit_session_probe.py run --server HOST:PORT [--hz 20] [--n 60] [--m 30] [--k 20]
                                   [--watch-hz 100] [--expect nano] [--out report.json]
       permit_session_probe.py send --server HOST:PORT --stream S --ids A:B [--hz 20]
                                   [--extra ID,ID] [--timeout 0.5]
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from host.kv_link import (KvLinkClient, KvTelemetryClient, LinkError, LinkTimeout,  # noqa: E402
                          blocks_mask, crc32, encode_request)
from host.fpga_clock import now_ns  # noqa: E402

H, W = 48, 64
K = np.array([[50.0, 0, 31.5], [0, 50.0, 23.5], [0, 0, 1]])
UNIT = np.float32(0.001)


def build_request(stream, fid):
    codes = np.full((H, W), 1000, dtype=np.uint16)
    codes[::7, ::5] = 0
    depth = codes.astype(np.float32) * UNIT
    depth[codes == 0] = np.nan
    ident = {"stream_instance": stream, "clock_id": "permit_probe", "calibration_id": "probe",
             "pose_epoch": "e1"}
    cap = now_ns()
    body = encode_request(frame_id=fid, capture_ns=cap, pose_ns=cap, identity=ident,
                          width=W, height=H, nx=32, ny=32, nz=32, unit_f32=UNIT, voxel=0.05,
                          origin=np.array([-0.8, -0.8, -0.05]),
                          inverse_k=np.linalg.inv(K).astype(np.float32),
                          world_camera=np.eye(4, dtype=np.float32)[:3], codes=codes,
                          depth_crc=crc32(depth))
    return body, {"frame_id": fid, "capture_ns": cap, "pose_ns": cap, "identity": ident}


def parse_server(text):
    host, port = text.rsplit(":", 1)
    return host, int(port)


def cmd_send(a):
    server = parse_server(a.server)
    lo, hi = (int(x) for x in a.ids.split(":")) if a.ids else (1, 0)
    ids = [(i, False) for i in range(lo, hi + 1)]
    for x in a.extra.split(","):          # "ID" = same link session, "ID@new" = a fresh one
        if x:
            ids.append((int(x.split("@")[0]), x.endswith("@new")))
    client = KvLinkClient(server, timeout_s=a.timeout)
    if a.wait_stdin:                      # imports done, socket open: send on the parent's "go"
        print("ready", flush=True)
        sys.stdin.readline()
    rows = []
    period = 1.0 / a.hz
    t_next = time.monotonic()
    try:
        for fid, fresh in ids:
            body, expect = build_request(a.stream, fid)
            t0 = now_ns()
            link = KvLinkClient(server, timeout_s=a.timeout) if fresh else client
            try:
                link.call(body, expect)
                ok, err = True, None
            except LinkTimeout:
                ok, err = False, "timeout"
            except LinkError as exc:
                ok, err = False, "%s: %s" % (type(exc).__name__, exc)
            finally:
                if fresh:
                    link.close()
            rows.append({"fid": fid, "new_link": fresh, "ok": ok, "err": err, "t_send_ns": t0,
                         "t_end_ns": now_ns()})
            t_next += period
            dt = t_next - time.monotonic()
            if dt > 0:
                time.sleep(dt)
    finally:
        client.close()
    print(json.dumps({"stream": a.stream, "pid": os.getpid(), "rows": rows,
                      "counts": dict(client.counts)}))


class Watcher(threading.Thread):
    def __init__(self, server, hz):
        super().__init__(daemon=True)
        self.client = KvTelemetryClient(server, timeout_s=0.2, hello_timeout_s=2.0)
        self.period, self.samples = 1.0 / hz, []
        self.stop_evt = threading.Event()

    def run(self):
        want = blocks_mask(("permit", "board"))
        while not self.stop_evt.is_set():
            t0 = now_ns()
            try:
                tm = self.client.poll(want)
            except LinkError as exc:
                self.samples.append({"t_ns": t0, "outcome": type(exc).__name__})
            else:
                p = tm.blocks.get("permit") or {}
                bits = p.get("status_bits") or {}
                self.samples.append({
                    "t_ns": t0, "t_recv_ns": now_ns(), "permit": bits.get("permit"),
                    "status": p.get("status"), "cause": bits.get("revoke_cause"),
                    "accepted": p.get("frames_accepted"), "rejected": p.get("frames_rejected"),
                    "revokes": p.get("revoke_count"), "last_frame_id": p.get("last_frame_id"),
                    "feed": p.get("feed")})
            time.sleep(max(0.0, self.period - (now_ns() - t0) / 1e9))
        self.client.close()


def snapshot(server):
    """Raw permit registers + feed (one TELEM poll on its own session)."""
    tel = KvTelemetryClient(server, timeout_s=0.5, hello_timeout_s=2.0)
    try:
        tm = tel.poll(blocks_mask(("permit", "board")))
    finally:
        tel.close()
    p = dict(tm.blocks.get("permit") or {})
    b = tm.blocks.get("board") or {}
    p["clk_hz"] = b.get("pl_clk_hz")
    p["build_id"] = b.get("build_id")
    return p


def server_counts(server):
    c = KvLinkClient(server, timeout_s=1.0)
    try:
        c.hello(2.0)
        st = c.stats()
    finally:
        c.close()
    tc = st.get("total_counts") or {}
    return {k: v for k, v in tc.items() if k.startswith("permit_")}


def child_cmd(a, stream, ids=None, extra=()):
    cmd = [sys.executable, os.path.abspath(__file__), "send", "--server", a.server,
           "--stream", stream, "--hz", str(a.hz), "--timeout", str(a.timeout)]
    if ids:
        cmd += ["--ids", "%d:%d" % ids]
    if extra:
        cmd += ["--extra=" + ",".join(str(x) for x in extra)]
    return cmd


def child(a, stream, ids=None, extra=()):
    r = subprocess.run(child_cmd(a, stream, ids, extra), capture_output=True, text=True, timeout=600,
                       env=dict(os.environ, PYTHONPATH=ROOT))
    if r.returncode != 0:
        raise RuntimeError("send child failed: %s" % r.stderr[-2000:])
    return json.loads(r.stdout.strip().splitlines()[-1])


def new_stream():
    return "permitprobe-%d-%s" % (os.getpid(), os.urandom(3).hex())


def delta(after, before, key):
    return int(after[key]) - int(before[key])


def window(samples, t0, t1):
    return [s for s in samples if t0 <= s["t_ns"] <= t1 and "permit" in s]


def first_after(samples, t, pred):
    for s in samples:
        if s["t_ns"] >= t and "permit" in s and pred(s):
            return s
    return None


def cmd_run(a):
    server = parse_server(a.server)
    rep = {"argv": sys.argv[1:], "expect": a.expect, "phases": {}, "checks": {}}
    s0 = snapshot(server)
    clk = float(s0["clk_hz"] or 100e6)
    # v3fc2: the PL window is a register (0x44, power-on 1000 ms; 200 ms on older bitstreams)
    win_ms = float(s0.get("timeout_cycles") or 0.2 * clk) / clk * 1e3
    rep["start"] = {"build_id": s0.get("build_id"), "clk_hz": clk, "feed": s0.get("feed"),
                    "permit_window_ms": win_ms,
                    "server_counts": server_counts(server)}
    w = Watcher(server, a.watch_hz)
    w.start()
    time.sleep(0.3)
    exp_nano = a.expect == "nano"
    checks = rep["checks"]
    try:
        # P1 normal renewal
        sa = new_stream()
        b1 = snapshot(server)
        ra = child(a, sa, (1, a.n))
        ok_rows = [r for r in ra["rows"] if r["ok"]]
        t_first, t_last = ok_rows[0]["t_end_ns"], ok_rows[-1]["t_end_ns"]
        win = window(w.samples, t_first + int(20e6), t_last)
        # P2 expiry
        time.sleep(win_ms / 1e3 + 0.8)
        a1 = snapshot(server)
        rev_ms = (a1["last_revoke_ts"] - a1["last_done_ts"]) / clk * 1e3
        drop = first_after(w.samples, t_last, lambda s: s["permit"] is False)
        rep["phases"]["P1_P2"] = {
            "stream": sa, "answered": len(ok_rows), "sent": len(ra["rows"]),
            "accepted_delta": delta(a1, b1, "frames_accepted"),
            "revokes_while_sending": (max(s["revokes"] for s in win) - min(s["revokes"] for s in win))
            if win else None,
            "samples_while_sending": len(win),
            "samples_permit0_while_sending": sum(1 for s in win if not s["permit"]),
            "last_cause": a1["last_cause"], "revoke_after_done_ms_pl": rev_ms,
            "telem_last_answer_to_permit0_ms": ((drop["t_ns"] - t_last) / 1e6) if drop else None,
            "feed": a1.get("feed")}
        p = rep["phases"]["P1_P2"]
        checks["P1_all_answered"] = p["answered"] == a.n
        checks["P1_accepted_all"] = p["accepted_delta"] == a.n
        checks["P1_no_revoke_while_sending"] = (p["revokes_while_sending"] == 0 and
                                                p["samples_permit0_while_sending"] == 0 and
                                                p["samples_while_sending"] > 10)
        checks["P2_cause_timeout"] = p["last_cause"] == 1
        checks["P2_revoke_at_window"] = abs(rev_ms - win_ms) < 0.1
        # P3 restart (new process, ids from 1 again)
        sb = new_stream()
        b3 = snapshot(server)
        late = subprocess.Popen(child_cmd(a, sa, extra=(a.n + 1,)) + ["--wait-stdin"],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
                                env=dict(os.environ, PYTHONPATH=ROOT))
        if late.stdout.readline().strip() != "ready":
            raise RuntimeError("late-packet child did not start")
        rb = child(a, sb, (1, a.m))
        t_b_first = next(r["t_end_ns"] for r in rb["rows"] if r["ok"])
        t_b_last = [r for r in rb["rows"] if r["ok"]][-1]["t_end_ns"]
        # P4 late packet of the ended session A, ~100 ms after B's last frame (B still granted)
        b4 = snapshot(server)
        time.sleep(max(0.0, 0.1 - (now_ns() - t_b_last) / 1e9))
        late.stdin.write("go\n")
        late.stdin.flush()
        rl = json.loads(late.stdout.read().strip().splitlines()[-1])
        if late.wait(60) != 0:
            raise RuntimeError("late-packet child failed")
        time.sleep(win_ms / 1e3 + 0.8)
        a4 = snapshot(server)
        grant_b = first_after(w.samples, t_b_first, lambda s: s["permit"] is True)
        rep["phases"]["P3_P4"] = {
            "stream_b": sb, "answered_b": sum(1 for r in rb["rows"] if r["ok"]),
            "accepted_delta_b": delta(b4, b3, "frames_accepted"),
            "first_grant_after_b_first_answer_ms": ((grant_b["t_ns"] - t_b_first) / 1e6)
            if grant_b else None,
            "late_answered": rl["rows"][0]["ok"], "late_err": rl["rows"][0]["err"],
            "late_accepted_delta": delta(a4, b4, "frames_accepted"),
            "late_rejected_delta": delta(a4, b4, "frames_rejected"),
            "last_cause_after": a4["last_cause"],
            "revoke_after_last_accepted_ms_pl": (a4["last_revoke_ts"] - a4["last_done_ts"]) / clk * 1e3,
            "late_sent_after_b_last_ms": (rl["rows"][0]["t_send_ns"] - t_b_last) / 1e6,
            "feed": a4.get("feed"), "server_counts": server_counts(server)}
        p = rep["phases"]["P3_P4"]
        checks["P3_restart_all_answered"] = p["answered_b"] == a.m
        checks["P3_restart_accepted_all"] = p["accepted_delta_b"] == a.m
        checks["P3_restart_grants"] = grant_b is not None
        if exp_nano:
            checks["P4_late_sent_while_b_granted"] = p["late_sent_after_b_last_ms"] < win_ms - 20.0
            checks["P4_late_not_answered"] = p["late_answered"] is False
            checks["P4_late_not_accepted"] = p["late_accepted_delta"] == 0
            checks["P4_revoke_at_window_after_b"] = (abs(p["revoke_after_last_accepted_ms_pl"] - win_ms) < 0.1
                                                 and p["last_cause_after"] == 1)
        else:
            checks["P4_control_late_renews"] = (p["late_answered"] is True and
                                                p["late_accepted_delta"] == 1)
        # P5a duplicate on the same link session: dropped by the link reassembler
        b5 = snapshot(server)
        k1 = a.m + a.k
        rs = child(a, sb, (a.m + 1, k1), extra=(k1,))
        time.sleep(0.05)
        a5 = snapshot(server)
        same_link_dup = rs["rows"][a.k]
        rep["phases"]["P5a_same_link"] = {
            "row": {k: same_link_dup[k] for k in ("fid", "new_link", "ok", "err")},
            "accepted_delta": delta(a5, b5, "frames_accepted"),
            "rejected_delta": delta(a5, b5, "frames_rejected")}
        p = rep["phases"]["P5a_same_link"]
        checks["P5a_same_link_dup_not_answered"] = same_link_dup["ok"] is False
        checks["P5a_same_link_dup_not_accepted"] = (p["accepted_delta"] == a.k and
                                                    p["rejected_delta"] == 0)
        # P5b duplicate / regress on fresh link sessions while permit is up: the PL judges
        b6 = snapshot(server)
        k2 = k1 + a.k
        rd = child(a, sb, (k1 + 1, k2), extra=("%d@new" % k2, "%d@new" % (k2 - 3)))
        time.sleep(0.05)
        a6 = snapshot(server)
        dup_row = rd["rows"][a.k]
        before = [s for s in w.samples if "permit" in s and s["t_ns"] <= dup_row["t_send_ns"]]
        up_before = bool(before) and before[-1]["permit"] is True and \
            (dup_row["t_send_ns"] - before[-1]["t_ns"]) < 50e6
        drop6 = first_after(w.samples, dup_row["t_send_ns"], lambda s: s["permit"] is False)
        rep["phases"]["P5b_fresh_link"] = {
            "rows": [{k: r[k] for k in ("fid", "new_link", "ok", "err")} for r in rd["rows"][a.k - 2:]],
            "accepted_delta": delta(a6, b6, "frames_accepted"),
            "rejected_delta": delta(a6, b6, "frames_rejected"),
            "last_cause": a6["last_cause"], "permit_after": bool(a6["status"] & 1),
            "permit_up_just_before_dup": up_before,
            "dup_send_to_permit0_ms": ((drop6["t_ns"] - dup_row["t_send_ns"]) / 1e6) if drop6 else None,
            "feed": a6.get("feed"), "server_counts": server_counts(server)}
        p = rep["phases"]["P5b_fresh_link"]
        if exp_nano:
            checks["P5b_new_ids_accepted"] = p["accepted_delta"] == a.k
            checks["P5b_dup_and_regress_rejected"] = p["rejected_delta"] == 2
            checks["P5b_cause_replay"] = p["last_cause"] == 3
            checks["P5b_permit_revoked_at_dup"] = (up_before and drop6 is not None and
                                                   p["dup_send_to_permit0_ms"] < 50.0)
        else:
            checks["P5b_control_dup_renews"] = (p["accepted_delta"] == a.k + 2 and
                                                p["rejected_delta"] == 0)
        time.sleep(0.5)
    finally:
        w.stop_evt.set()
        w.join(2.0)
    rep["end"] = {"server_counts": server_counts(server), "snapshot": snapshot(server)}
    rep["watch"] = {"samples": len(w.samples),
                    "errors": sum(1 for s in w.samples if "outcome" in s)}
    rep["pass"] = all(checks.values())
    text = json.dumps(rep, indent=1, sort_keys=True, default=str)
    if a.out:
        with open(a.out, "w") as fp:
            fp.write(text)
        with open(a.out + ".samples.jsonl", "w") as fp:
            for s in w.samples:
                fp.write(json.dumps(s, default=str) + "\n")
    print(json.dumps({"pass": rep["pass"], "checks": checks}, sort_keys=True))
    return 0 if rep["pass"] else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--server", required=True)
    r.add_argument("--hz", type=float, default=20.0)
    r.add_argument("--n", type=int, default=60)
    r.add_argument("--m", type=int, default=30)
    r.add_argument("--k", type=int, default=20)
    r.add_argument("--timeout", type=float, default=0.5)
    r.add_argument("--watch-hz", type=float, default=100.0)
    r.add_argument("--expect", choices=("nano", "kv-counter"), default="nano")
    r.add_argument("--out")
    s = sub.add_parser("send")
    s.add_argument("--server", required=True)
    s.add_argument("--stream", required=True)
    s.add_argument("--ids", default="")
    s.add_argument("--extra", default="")
    s.add_argument("--hz", type=float, default=20.0)
    s.add_argument("--timeout", type=float, default=0.5)
    s.add_argument("--wait-stdin", action="store_true")
    a = ap.parse_args()
    if a.cmd == "send":
        cmd_send(a)
        return 0
    return cmd_run(a)


if __name__ == "__main__":
    sys.exit(main())
