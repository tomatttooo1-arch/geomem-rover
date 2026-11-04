#!/usr/bin/env python3
"""Line D2 (2026-10-14): passive TELEM recorder of the fake KV's permit model, for stop-latency timestamps.

  permit_recorder.py --kv 127.0.0.1:47811 --out PATH [--hz 100]

One JSON row per poll on its own kv-link session (never sends frames, e-stops or SYNC): the permit block
(available, reason, status, age, last frame id, accepted / rejected / revoke counts, last cause, the
last DONE / REVOKE timestamps and the current timebase in PL cycles), the e-stop block, and the poll's
send / receive times on CLOCK_MONOTONIC (the clock of the host permit model, host/kv_regs.py
RealtimePermitModel) and CLOCK_REALTIME (the axis record_motion.py and mcu_sim.py --log share). The first
row also carries the PL clock (--pl-clk-hz, default 100 MHz = the model's default).
A PL timestamp ts maps to monotonic ns as  t_mid - (tb_now - ts) * 1e9 / clk  (error <= RTT / 2).
Stops on SIGINT / SIGTERM; flushes every row.
"""
import argparse
import json
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from host.kv_link import KvTelemetryClient, LinkError, blocks_mask  # noqa: E402

PERMIT_KEYS = ("available", "reason", "status", "status_bits", "age_cycles", "last_frame_id",
               "frames_accepted", "frames_rejected", "revoke_count", "last_cause", "last_done_ts",
               "last_revoke_ts", "tb_now", "timeout_cycles")
ESTOP_KEYS = ("latched", "epoch", "latched_kv_ns", "force_stop_written", "pl_applied", "last_result")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kv", required=True, help="HOST:PORT of kv_server")
    ap.add_argument("--out", required=True)
    ap.add_argument("--hz", type=float, default=100.)
    ap.add_argument("--pl-clk-hz", type=int, default=100000000)
    a = ap.parse_args(argv)
    host, _, port = a.kv.rpartition(":")
    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.append(1))
    client = KvTelemetryClient((host, int(port)), timeout_s=0.2, hello_timeout_s=10.0)
    want = blocks_mask(("permit", "estop"))
    period = 1. / a.hz
    n = errors = 0
    with open(a.out, "x", buffering=1) as fp:
        nxt = time.monotonic()
        while not stop:
            row = {"mono_send_ns": time.monotonic_ns(), "wall_send_ns": time.time_ns()}
            try:
                tm = client.poll(want)
            except LinkError as exc:
                errors += 1
                row.update(ok=False, err=type(exc).__name__)
            else:
                row.update(ok=True, mono_recv_ns=time.monotonic_ns(), wall_recv_ns=time.time_ns())
                p = tm.blocks.get("permit") or {}
                row["permit"] = {k: p.get(k) for k in PERMIT_KEYS}
                e = tm.blocks.get("estop") or {}
                row["estop"] = {k: e.get(k) for k in ESTOP_KEYS}
            if n == 0:
                row["pl_clk_hz"] = a.pl_clk_hz
            fp.write(json.dumps(row, sort_keys=True) + "\n")
            n += 1
            nxt += period
            dt = nxt - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            else:
                nxt = time.monotonic()
    client.close()
    print(json.dumps({"polls": n, "errors": errors}), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
