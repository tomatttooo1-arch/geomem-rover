#!/usr/bin/env python3
"""E-stop (type 12/13) probe against a running kv_server.py (PC_DATAPLANE A.4, E.1).

Runs one fixed sequence and prints every EstopAck (one JSON line per step):

  query -> assert -> assert (same op, new cmd_id: idempotent, result 1) -> query
  -> [hold --hold-s] -> release with a wrong epoch (result 2, stays latched)
  -> release with the latched epoch -> [watch --after-s] -> query -> release again (3)

Each step is sent like the Nano will: one cmd_id, resent every 200 ms (new chunk frame id
each time) until answered or --tries. With --watch-hz > 0 a TELEM poller on its own
session samples the permit / estop blocks during the whole sequence, so the report shows
when STATUS b3 (estop) rises and falls, and that permit b0 comes back only after
FRAMES_ACCEPTED moves (the next accepted geometry frame), never at the release itself.

Usage: estop_probe.py --server HOST:PORT [--hold-s 2] [--after-s 3] [--watch-hz 50]
                      [--out report.json]
"""
import argparse
import json
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.kv_link import KvEstopClient, KvTelemetryClient, LinkError, LinkTimeout  # noqa: E402
from host.fpga_clock import now_ns  # noqa: E402


class Watcher(threading.Thread):
    def __init__(self, server, hz, t0):
        super().__init__(daemon=True)
        self.client = KvTelemetryClient(server, timeout_s=0.2, hello_timeout_s=2.0)
        self.period, self.t0 = 1.0 / hz, t0
        self.samples, self.stop_evt = [], threading.Event()

    def run(self):
        while not self.stop_evt.is_set():
            t = (now_ns() - self.t0) / 1e6
            try:
                tm = self.client.poll()
            except LinkError as exc:
                self.samples.append({"t_ms": t, "outcome": type(exc).__name__})
            else:
                p, e = tm.blocks.get("permit") or {}, tm.blocks.get("estop") or {}
                g = (tm.blocks.get("geometry") or {}).get("last") or {}
                s = {"t_ms": t, "outcome": "ok", "estop_latched": e.get("latched"),
                     "epoch": e.get("epoch"), "geometry_frame_id": g.get("frame_id")}
                if p.get("available"):
                    bits = p["status_bits"]
                    s.update(permit=bits["permit"], estop_bit=bits["estop"],
                             accepted=bits["accepted"], last_cause=p["last_cause"],
                             frames_accepted=p["frames_accepted"],
                             frames_rejected=p["frames_rejected"],
                             revoke_count=p["revoke_count"], last_frame_id=p["last_frame_id"])
                else:
                    s["permit_avail"] = False
                self.samples.append(s)
            self.stop_evt.wait(self.period)
        self.client.close()


def send(client, op, cmd_id, epoch, tries, t0):
    t_first = now_ns()
    for k in range(tries):
        try:
            ack = client.send(op, cmd_id, epoch=epoch, timeout_s=0.2)
        except LinkTimeout:
            continue
        t = now_ns()
        return {"op": op, "cmd_id": cmd_id, "epoch_sent": epoch, "tries": k + 1,
                "t_ms": (t_first - t0) / 1e6, "send_to_ack_ms": (t - t_first) / 1e6,
                "ack": ack._asdict()}
    return {"op": op, "cmd_id": cmd_id, "epoch_sent": epoch, "tries": tries,
            "t_ms": (t_first - t0) / 1e6, "ack": None}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--server", required=True)
    ap.add_argument("--hold-s", type=float, default=2.0)
    ap.add_argument("--after-s", type=float, default=3.0)
    ap.add_argument("--watch-hz", type=float, default=50.0)
    ap.add_argument("--tries", type=int, default=10)
    ap.add_argument("--cmd-base", type=int, default=None,
                    help="first cmd_id (default: from the clock, so reruns never collide)")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    host, port = a.server.rsplit(":", 1)
    server = (host, int(port))
    t0 = now_ns()
    base = a.cmd_base if a.cmd_base is not None else (t0 & 0xFFFFFFFF) << 8
    watch = None
    if a.watch_hz > 0:
        watch = Watcher(server, a.watch_hz, t0)
        watch.start()
        time.sleep(0.5)
    c = KvEstopClient(server, hello_timeout_s=2.0)
    steps = []

    def step(op, epoch=0):
        r = send(c, op, base + len(steps), epoch, a.tries, t0)
        steps.append(r)
        print(json.dumps(r, default=str), flush=True)
        return r
    step("query")
    s = step("assert")
    epoch = s["ack"]["epoch"] if s["ack"] else 0
    step("assert")
    step("query")
    time.sleep(a.hold_s)
    step("release", epoch + 1000)          # wrong epoch: must answer 2 and stay latched
    step("release", epoch)
    time.sleep(a.after_s)
    step("query")
    step("release", epoch)                 # nothing latched any more: 3
    if watch is not None:
        watch.stop_evt.set()
        watch.join(2.0)
    report = {"server": a.server, "steps": steps,
              "results": [x["ack"]["result"] if x["ack"] else None for x in steps],
              "client_counts": dict(c.counts)}
    if watch is not None:
        report["watch"] = watch.samples
        rel_t = steps[5]["t_ms"]
        after = [x for x in watch.samples if x.get("outcome") == "ok" and x["t_ms"] >= rel_t
                 and "permit" in x]
        before = [x for x in watch.samples if x.get("outcome") == "ok" and x["t_ms"] < rel_t
                  and "permit" in x]
        regrant = next((x for x in after if x["permit"]), None)
        report["release_analysis"] = {
            "release_t_ms": rel_t,
            "accepted_at_release": before[-1]["frames_accepted"] if before else None,
            "first_permit_after_release": regrant,
            "permit_samples_after_release_before_regrant": sum(
                1 for x in after if regrant is None or x["t_ms"] < regrant["t_ms"]),
            "regrant_needed_new_accept": (regrant is not None and before != [] and
                                          regrant["frames_accepted"] > before[-1]["frames_accepted"]),
        }
    c.close()
    if a.out:
        with open(a.out, "w") as fp:
            json.dump(report, fp, indent=1, sort_keys=True, default=str)
    print(json.dumps({"results": report["results"],
                      "release_analysis": report.get("release_analysis")}, default=str))


if __name__ == "__main__":
    main()
