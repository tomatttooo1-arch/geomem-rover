#!/usr/bin/env python3
"""Whole-chain board smoke instrument (Nano side): KV permit watcher + one PC e-stop.

Runs next to the geometry producer rig (it never sends frames: TELEM/ESTOP only, so the
permit feed sees exactly one Nano frame session -- the producer's).

  * TELEM watcher on its own kv-link session: --hz (default 20) for the whole run, --fast-hz
    (default 50) from 1 s before the e-stop until 3 s after the release. One JSON line per poll
    (Nano CLOCK_MONOTONIC_RAW stamps, host.fpga_clock): permit bit, status, cause, accepted,
    rejected, revoke_count, last_frame_id, estop bit, TELEM rtt.
  * E-stop as a PC (pc/fpgac.py FpgacClient to the Nano hub, its own connection, made at start so
    this PC is the control holder -- only the holder may release): at --estop-at-s
    send ``estop``; record send / ACK / RESULT stamps (RESULT carries the epoch and the hub->KV
    latency); hold --hold-s; send ``release --epoch <epoch>``; record ACK / RESULT.
  * At the end one full permit block (raw last_done_ts / last_revoke_ts / last_cause) and the
    board block (pl_clk_hz, build_id): (last_revoke_ts - last_done_ts) / clk = the revoke time
    after the producer stopped sending, if the last revoke is the expiry (cause 1).

Writes <out>/watch.jsonl, <out>/estop.json, <out>/final.json. Exit 0 also when a step failed
(the failure is recorded); analysis is tools/integ_car_chain/summarize_chain.py.

Usage: chain_watch.py --server HOST:PORT --hub-port P --token-file T --seconds S
                      [--estop-at-s 100] [--hold-s 2] [--hz 20] [--fast-hz 50] --out DIR
"""
import argparse
import json
import os
import signal
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.kv_link import KvTelemetryClient, LinkError, blocks_mask  # noqa: E402
from host.fpga_clock import now_ns  # noqa: E402
from pc import fpgac  # noqa: E402


class Watcher(threading.Thread):
    def __init__(self, server, hz, fast_hz, path):
        super().__init__(daemon=True)
        self.client = KvTelemetryClient(server, timeout_s=0.2, hello_timeout_s=3.0)
        self.slow, self.fast = 1.0 / hz, 1.0 / fast_hz
        self.fast_until_ns = 0
        self.fp = open(path, "x")
        self.stop_evt = threading.Event()
        self.n = 0
        self.last_permit = None

    def run(self):
        want = blocks_mask(("permit", "board", "estop"))
        while not self.stop_evt.is_set():
            t0 = now_ns()
            row = {"t_ns": t0}
            try:
                tm = self.client.poll(want)
            except LinkError as exc:
                row["outcome"] = type(exc).__name__
            else:
                t1 = now_ns()
                p = tm.blocks.get("permit") or {}
                bits = p.get("status_bits") or {}
                e = tm.blocks.get("estop") or {}
                row.update({"outcome": "ok", "rtt_ms": round((t1 - t0) / 1e6, 3),
                            "permit": bits.get("permit"), "estop_bit": bits.get("estop"),
                            "status": p.get("status"), "cause": bits.get("revoke_cause"),
                            "last_cause": p.get("last_cause"),
                            "accepted": p.get("frames_accepted"), "rejected": p.get("frames_rejected"),
                            "revokes": p.get("revoke_count"), "last_frame_id": p.get("last_frame_id"),
                            "since_done_ms": p.get("since_done_ms"),
                            "kv_estop_latched": e.get("latched"), "kv_estop_epoch": e.get("epoch")})
                self.last_permit = p
            self.fp.write(json.dumps(row) + "\n")
            self.n += 1
            period = self.fast if t0 < self.fast_until_ns else self.slow
            time.sleep(max(0.0, period - (now_ns() - t0) / 1e9))
        self.fp.close()


def final_block(server):
    tel = KvTelemetryClient(server, timeout_s=0.5, hello_timeout_s=3.0)
    try:
        tm = tel.poll(blocks_mask(("permit", "board", "estop")))
    finally:
        tel.close()
    return {"t_ns": now_ns(), "permit": tm.blocks.get("permit"), "board": tm.blocks.get("board"),
            "estop": tm.blocks.get("estop")}


def connect_pc(a, rec):
    """The e-stop PC connects first (before the listening PC), so it is the hub's control holder
    (only the holder may release; any PC may assert). ``<out>/pc_connected`` marks the moment."""
    token = open(a.token_file).read().strip()
    c = fpgac.FpgacClient(a.hub_port, token, hosts=["127.0.0.1"], want=("state",),
                        last_file=os.path.join(a.out, "fpgac_last.json")).start()
    rec["t_connect_start_ns"] = now_ns()
    if not c.connected.wait(15):
        rec["error"] = "pc client not connected"
    else:
        rec["t_connected_ns"] = now_ns()
    open(os.path.join(a.out, "pc_connected"), "w").write(str(rec.get("t_connected_ns")))
    return c


def estop_cycle(a, rec, watcher, c):
    if "t_connected_ns" not in rec:
        return
    watcher.fast_until_ns = now_ns() + int((a.hold_s + 30) * 1e9)
    time.sleep(1.0)
    rec["t_estop_send_ns"] = now_ns()
    cid = c.command("estop", {})
    ack = c.wait_ack(cid, 30)
    rec["t_estop_ack_ns"], rec["estop_ack"] = now_ns(), ack
    res = c.wait_result(cid, 10)
    rec["t_estop_result_ns"], rec["estop_result"] = now_ns(), res
    if not res or res.get("status") != "latched":
        rec["error"] = "estop not latched"
        return
    time.sleep(a.hold_s)
    epoch = res.get("epoch")
    rec["t_release_send_ns"] = now_ns()
    cid = c.command("release", {"epoch": epoch})
    ack = c.wait_ack(cid, 6)
    rec["t_release_ack_ns"], rec["release_ack"] = now_ns(), ack
    res = c.wait_result(cid, 10)
    rec["t_release_result_ns"], rec["release_result"] = now_ns(), res
    if not res or res.get("status") != "released":
        rec["error"] = "release not confirmed"
    watcher.fast_until_ns = now_ns() + int(3e9)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--server", required=True)
    ap.add_argument("--hub-port", type=int, default=fpgac.DEFAULT_PORT)
    ap.add_argument("--token-file", required=True)
    ap.add_argument("--seconds", type=float, required=True)
    ap.add_argument("--estop-at-s", type=float, default=100.0, help="<0: no e-stop")
    ap.add_argument("--hold-s", type=float, default=2.0)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--fast-hz", type=float, default=50.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    host, port = a.server.rsplit(":", 1)
    server = (host, int(port))
    t_start = now_ns()
    # 10-13: with the real camera the chain's start is not at a fixed time (driver start retries), so the
    # caller gives a generous --seconds and ends the watch with SIGINT/SIGTERM when the rig is done. A
    # background child of a non-interactive shell starts with SIGINT ignored: install the handler explicitly.
    # The handler only appends to a list (10-16): it runs on the main thread between bytecodes, and an
    # Event.set() there deadlocks for good if it lands inside the main thread's own Event.wait() (which
    # holds the Event's non-reentrant lock); a TERM that trips just before wait() blocks was also lost
    # until --seconds ran out. Stress, TERM right at the wait: 3.8 10/400, 3.13 4/400 never exited.
    # The main thread polls with time.sleep(0.1), so a signal stops the watch within ~0.1 s.
    stop_sig = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda signum, _f: stop_sig.append(signum))
    w = Watcher(server, a.hz, a.fast_hz, os.path.join(a.out, "watch.jsonl"))
    w.start()
    rec = {"argv": sys.argv[1:], "t_start_ns": t_start}
    c = connect_pc(a, rec) if a.estop_at_s >= 0 else None
    try:
        if c is not None:
            time.sleep(max(0.0, a.estop_at_s - 1.0 - (now_ns() - t_start) / 1e9))
            try:
                estop_cycle(a, rec, w, c)
            except Exception as exc:          # recorded; the watcher keeps running
                rec["error"] = "%s: %s" % (type(exc).__name__, exc)
            rec["pc_client_stats"] = c.stats()
            c.stop()
        with open(os.path.join(a.out, "estop.json"), "x") as f:
            json.dump(rec, f, indent=1, default=str)
        while not stop_sig and (now_ns() - t_start) / 1e9 < a.seconds:
            time.sleep(min(0.1, max(0.0, a.seconds - (now_ns() - t_start) / 1e9)))
        rec_stop = bool(stop_sig)
    finally:
        w.stop_evt.set()
        w.join(5)
        try:
            fin = final_block(server)
        except Exception as exc:
            fin = {"error": "%s: %s" % (type(exc).__name__, exc)}
        fin["watch_rows"] = w.n
        fin["watch_client_counts"] = dict(w.client.counts) if hasattr(w.client, "counts") else None
        with open(os.path.join(a.out, "final.json"), "x") as f:
            json.dump(fin, f, indent=1, default=str)
        w.client.close()
    print(json.dumps({"watch_rows": w.n, "estop_error": rec.get("error"), "stopped_by_signal": bool(rec_stop)}))


if __name__ == "__main__":
    main()
