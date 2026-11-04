#!/usr/bin/env python3
"""GOLIVE0 (2026-11-04): Nano task console of the single-board product (upper task layer only).

The KV260 does the sensing and the computing (camera, geometry / fusion / cost map / planner field on the PL,
recognition on the PL pre-processing core + DPU); this console receives the KV's uplink (tools/golive/kv_uplink.py)
and keeps the task-level view:

  * object memory: per class, how often and when it was last seen, best score and the newest box
  * map status: depth / cost-map / field update counts and rates, the planner state
  * a find request (--find <class>): answered from the object memory on every status line
  * liveness: no uplink for > --stale-s, or map / recognition counters that stop moving, give an ALARM line
    (and the console goes on: it never stops anything; it has no link to the chassis at all)

  * probabilistic search (SRCHCHAIN0, --srch pl|nano): where to look for the object next. pl: the KV runs it on the
    PL search core and the uplink carries the look order; the console only shows it and answers with it. nano (and
    the fallback of pl when the KV reports "fallback_nano" or sends no search for --srch-wait s): the console runs
    the product's software search itself (host/search_belief.py + scene under --srch-root, numpy; old path)

Writes every datagram to --log (JSON lines) and the current view to --status (JSON, rewritten every --every s).
Stdlib only.
"""

import argparse
import json
import os
import socket
import sys
import threading
import time


class LocalSearch(object):
    """SRCHCHAIN0: the old path, the product's software BeliefSearch on the Nano (thread, one episode per period)."""

    def __init__(self, root, period):
        self.root, self.period = root, period
        self.state = {"source": "nano_sw", "impl": "sw", "episodes": 0, "error": None}
        self.started = False

    def start(self):
        if self.started:
            return
        self.started = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            sys.path.insert(0, os.path.join(self.root, "tools", "golive"))
            import srch_chain as C
            sc = C.Scene(os.path.join(self.root, "scene.npz"))
            bs, _ = C.build_planner(sc, "sw")
        except Exception as exc:              # noqa: BLE001  (never stops the console)
            self.state["error"] = "local search unavailable: %r" % (exc,)
            print(json.dumps({"warning": self.state["error"]}), file=sys.stderr, flush=True)
            return
        n = 0
        while True:
            t = time.monotonic()
            try:
                r = C.episode(bs, sc, n)
                n += 1
                self.state.update(episodes=n, n=n, target_i=r["target_i"], target=r["target"], first=r["first"],
                                  order=[[l[0], l[3], l[4]] for l in r["looks"]], stop=r["stop"], ms=r["ms"])
            except Exception as exc:          # noqa: BLE001
                self.state["error"] = "episode: %r" % (exc,)
            time.sleep(max(0.5, self.period - (time.monotonic() - t)))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=47950)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--log", default="nano_console.jsonl")
    ap.add_argument("--status", default="nano_status.json")
    ap.add_argument("--every", type=float, default=5.0, help="status line period, s")
    ap.add_argument("--stale-s", type=float, default=3.0)
    ap.add_argument("--find", default="bottle", help="class name the task looks for")
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = until SIGTERM")
    ap.add_argument("--srch", default="pl", choices=("pl", "nano"), help="where the search runs (SRCHCHAIN0)")
    ap.add_argument("--srch-root", default="srch", help="host/search_belief.py, tools/golive/srch_chain.py, scene.npz")
    ap.add_argument("--srch-period", type=float, default=10.0)
    ap.add_argument("--srch-wait", type=float, default=60.0, help="pl: no KV search for this long -> local search")
    ap.add_argument("--mem", default="nano", choices=["nano", "pl"],
                    help="MEMCHAIN0: find answer from this console's box counts (nano) or from the KV object "
                         "memory in the uplink (pl; the box counts stay as a shadow)")
    a = ap.parse_args(argv)
    local = LocalSearch(a.srch_root, a.srch_period)
    if a.srch == "nano":
        local.start()
    kv_srch, kv_srch_t = None, None
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((a.bind, a.port))
    sock.settimeout(0.5)
    logf = open(a.log, "a")
    mem = {}            # class -> {"sightings", "first_t", "last_t", "best_score", "box"}
    last = None
    got, t0 = 0, time.monotonic()
    last_rx, nxt = None, t0 + a.every
    prev_counts, alarms = None, 0
    src = None
    mem_kv, mem_rx_t = None, None          # MEMCHAIN0: newest msg["mem"] (KV object memory) and when it came
    print(json.dumps({"console": "listening", "port": a.port, "find": a.find}), flush=True)
    while True:
        try:
            data, addr = sock.recvfrom(65535)
            now = time.monotonic()
            msg = json.loads(data.decode())
            got += 1
            last, last_rx, src = msg, now, addr[0]
            if msg.get("srch") is not None:
                kv_srch, kv_srch_t = msg["srch"], now
            if isinstance(msg.get("mem"), dict):
                mem_kv, mem_rx_t = msg["mem"], now
            logf.write(json.dumps({"rx_mono_ns": time.monotonic_ns(), "from": addr[0], "msg": msg},
                                  separators=(",", ":")) + "\n")
            boxes = (msg.get("recog") or {}).get("last") or {}
            fr = boxes.get("frame")
            for d in boxes.get("dets") or []:
                name = d[6] if len(d) > 6 else str(d[5])
                m = mem.setdefault(name, {"sightings": 0, "first_t": round(now - t0, 1), "last_frame": None})
                if m["last_frame"] == fr:          # one sighting per KV frame (the uplink repeats the newest)
                    continue
                m["sightings"] += 1
                m["last_frame"] = fr
                m["last_t"] = round(now - t0, 1)
                m["best_score"] = max(m.get("best_score", 0.0), d[4])
                m["box"] = d[:5]
        except socket.timeout:
            now = time.monotonic()
        except (ValueError, OSError) as exc:
            now = time.monotonic()
            print(json.dumps({"warning": repr(exc)}), file=sys.stderr, flush=True)
        if now >= nxt:
            nxt += a.every
            alarm = []
            if last_rx is None or now - last_rx > a.stale_s:
                alarm.append("no uplink for %.1f s" % ((now - last_rx) if last_rx else now - t0))
            mp = (last or {}).get("map", {})
            rc = (last or {}).get("recog", {})
            counts = (mp.get("depth_frames"), mp.get("field_updates"), rc.get("frames"))
            if prev_counts is not None and last is not None:
                for name, c0, c1 in zip(("depth", "field", "recog"), prev_counts, counts):
                    if c0 == c1:
                        alarm.append("%s counter not moving (%s)" % (name, c1))
            prev_counts = counts
            # SRCHCHAIN0: the search view; pl falls back to the local software search (old path) when the KV
            # reports fallback_nano or sends no search result for --srch-wait s
            if a.srch == "pl" and not local.started and (
                    (kv_srch or {}).get("impl") == "fallback_nano"
                    or (now - t0 > a.srch_wait and (kv_srch_t is None or now - kv_srch_t > a.srch_wait))):
                alarm.append("search: KV search core not available (%s): local software search started" % (
                    (kv_srch or {}).get("reason") or "no search from the KV"))
                local.start()
            if local.started:
                srch = dict(local.state, mode=a.srch)
            elif kv_srch is not None:
                srch = dict(kv_srch, source="kv_" + str(kv_srch.get("impl")), mode=a.srch)
            else:
                srch = {"mode": a.srch, "source": None}
            target = mem.get(a.find)
            find_nano = {"target": a.find, "found": target is not None,
                         **({"last_t_s": target["last_t"], "sightings": target["sightings"],
                             "best_score": round(target["best_score"], 3), "box": target["box"]}
                            if target else {})}
            kvmem = None
            if a.mem == "pl":
                if mem_rx_t is None or now - mem_rx_t > a.stale_s:
                    alarm.append("no KV memory in the uplink for %.1f s"
                                 % ((now - mem_rx_t) if mem_rx_t else now - t0))
                if mem_kv is not None:
                    if mem_kv.get("fallbacks"):
                        alarm.append("KV memory fell back to %s (%d)" % (mem_kv.get("src"), mem_kv["fallbacks"]))
                    ans = (mem_kv.get("find") or {}).get(a.find)
                    kvmem = {"src": mem_kv.get("src"), "calls": mem_kv.get("calls"),
                             "fallbacks": mem_kv.get("fallbacks"), "frames": mem_kv.get("frames"),
                             "n_objects": mem_kv.get("n_objects"),
                             "objects": [[o.get("id"), o.get("label"), o.get("state"), o.get("pos")]
                                         for o in (mem_kv.get("objects") or [])[:8]]}
            alarms += bool(alarm)
            view = {"t_s": round(now - t0, 1), "rx": got, "from": src, "kv_up_s": (last or {}).get("up_s"),
                    "map": {k: mp.get(k) for k in ("depth_frames", "depth_fps", "grid_fps", "field_updates",
                                                    "field_fps")},
                    "planner": mp.get("last_field"),
                    "recog": {k: rc.get(k) for k in ("frames", "fps", "box_rows")},
                    "find": dict(find_nano, **({} if target else {"search_next": srch.get("first"), "search_source": srch.get("source")})),
                    "search": srch,
                    "memory": {k: {"sightings": v["sightings"], "last_t_s": v["last_t"],
                                   "best": round(v["best_score"], 3)} for k, v in sorted(mem.items())},
                    "alarm": alarm, "alarm_lines": alarms}
            if a.mem == "pl":
                ans = ((mem_kv or {}).get("find") or {}).get(a.find)
                view["find"] = {"target": a.find, "from": "kv_memory", "found": bool(ans and ans.get("found")),
                                **({"items": ans.get("items")} if ans else {}),
                                "src": (mem_kv or {}).get("src"),
                                **({} if ans and ans.get("found") else
                                   {"search_next": srch.get("first"), "search_source": srch.get("source")})}
                view["find_nano_shadow"] = find_nano
                view["kv_memory"] = kvmem
            print(json.dumps(view, separators=(",", ":")), flush=True)
            tmp = a.status + ".tmp"
            with open(tmp, "w") as fp:
                json.dump(view, fp, indent=1)
            os.replace(tmp, a.status)
            if a.seconds and now - t0 > a.seconds:
                break


if __name__ == "__main__":
    main()
