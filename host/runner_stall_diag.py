"""Opt-in stall diagnostics for the task runner (host/ros_runtime.py --stall-diag PATH).

Why: on the board (2026-10-11 22:31, 1826 frames) the runner refused 8 geometry frames as
older than 200 ms (245-565 ms) while the producer's own frame age stayed <= 107 ms and the
runner never waited for the producer's answer. This module says where those milliseconds went,
frame by frame, without touching what the runner computes or publishes:

  frame timeline (RAW ns, host.fpga_clock, one clock with the producer's acquired_raw_ns):
    producer_acquired -> depth_rx (runner's depth callback) -> rx (on_images, the synchronized
    pair) -> prepared (TF/clock found, frame built) -> submitted (inference started) ->
    infer_begin/infer_end (worker thread around the detector round trip) -> queued (result
    handed to the executor) -> picked (poll takes it, after the bounded kv_topic hold) -> gate
  poll sections: wall ms and this thread's CPU ms of each step of poll() for that frame
    (result, process, geometry, gate, memory, stats, record_dumps/record_write/record_publish,
    task, submit, permit, memory_feed/memory_publish); a poll WITHOUT a frame still times its
    permit and throttled /fpga/memory publish: one poll_sections row when a section took >= 50 ms
  acquisition waits: every prepare_images retry of a pair (branch, exception, the runner's own
    latest TF stamp vs. the image stamp = TF lag) and the pairs dropped after the 0.5 s bound;
    since v6 (--tf-retry) each try names who called prepare_images (image | poll | tf) and the
    frame row carries acq_prepared_by / acq_tries_by
  /tf arrivals in this process (wrapped tf2 listener callback): gaps and the last stamp
  poll gaps (no poll for > poll_gap_ms although a 100 ms steady timer drives it), GC pauses,
  GIL watchdog overshoots (host.stall_watch, copied unchanged from integ/dev1/car-chain-20261011)

Rows go to their own JSONL file (never through record(): that publishes /fpga/status, itself a
suspect, and the hub forwards at most 20 events/s). Buffered, flushed at most once per second;
since v7 (2026-10-12) the flushed text goes through host/record_writer.RecordWriter (async by
default: queued for a background thread; ``record_write="sync"`` = the write+flush on the caller
as before), and the events file's slow writes come back here as record_write_slow rows.
The faulthandler dead-man switch of stall_watch is NOT used (it crashed the producer 4/4 in
long host runs, integ v3).

Positive control hook (hidden CLI): inject a sleep into one section, every Nth time it runs:
  SECTION:EVERY:MS:free|hold    free = time.sleep (GIL released), hold = C sleep holding the GIL
  every injection writes an inject row (section, start/end RAW ns, frame id or None)
Python 3.8 compatible; no ROS import.
"""

import collections
import json
import threading
import time

from host.fpga_clock import now_ns
from host.record_writer import RecordWriter
from host.stall_watch import EventQueue, GcPauses, GilWatchdog, free_gil_sleep, hold_gil_sleep

INJECT_SECTIONS = ("acquire", "infer", "process", "geometry", "gate", "memory", "record", "task",
                   "permit", "memory_publish")
POLL_SECTIONS = ("result", "process", "geometry", "gate", "memory", "stats", "record_dumps",
                 "record_write", "record_publish", "task", "submit", "permit", "memory_feed",
                 "memory_publish")


def parse_inject(spec):
    """'SECTION:EVERY:MS[:free|hold]' -> (section, every, ms, mode) or None."""
    if not spec:
        return None
    parts = spec.split(":")
    if len(parts) == 3:
        parts.append("free")
    if len(parts) != 4 or parts[0] not in INJECT_SECTIONS or parts[3] not in ("free", "hold"):
        raise ValueError("--stall-diag-inject SECTION:EVERY:MS[:free|hold], SECTION in %s" % (INJECT_SECTIONS,))
    every, ms = int(parts[1]), float(parts[2])
    if every < 1 or not ms > 0:
        raise ValueError("--stall-diag-inject: EVERY >= 1 and MS > 0")
    return parts[0], every, ms, parts[3]


def _writer_brief(w):
    x = w.summary()
    return {k: x[k] for k in ("file", "mode", "accepted", "written", "dropped", "dropped_bytes", "write_errors",
                              "slow_writes", "max_write_ms", "max_queue_rows", "max_queue_bytes")}


def _ms(a, b):
    return None if a is None or b is None else (b - a) / 1e6


def stamp_ns(header):
    return header.stamp.sec * 10 ** 9 + header.stamp.nanosec


class RunnerStallDiag(object):
    def __init__(self, path, inject=None, clock=now_ns, gil_period_s=0.02, gil_min_ms=30.0,
                 gc_min_ms=10.0, poll_gap_ms=200.0, tf_gap_ms=100.0, flush_s=1.0, watchdog=True,
                 poll_section_ms=50.0, record_write="async"):
        self.path = str(path)
        self.clock = clock
        self.events = EventQueue()
        self.out = RecordWriter(open(self.path, "x", encoding="utf8"), mode=record_write, name="stall_diag",
                                on_slow=self.events.put)       # own RAW clock: never the caller's
        self.fp = self.out
        self.peer_writers = []                   # e.g. the runner's events writer (summary only)
        self.inject = parse_inject(inject) if isinstance(inject, str) or inject is None else inject
        self.inject_seen = collections.Counter()
        self.injected = collections.Counter()
        self.gc = GcPauses(self.events, min_ms=gc_min_ms, clock=clock).install()
        self.watchdog = GilWatchdog(self.events, period_s=gil_period_s, min_ms=gil_min_ms, clock=clock) if watchdog else None
        if self.watchdog is not None:
            self.watchdog.start()
        self.poll_gap_ns, self.tf_gap_ns = int(poll_gap_ms * 1e6), int(tf_gap_ms * 1e6)
        self.poll_section_ms = float(poll_section_ms)
        self.poll_cur = None                     # sections of a poll that has no frame (permit, /fpga/memory)
        self.flush_ns, self.last_flush = int(flush_s * 1e9), clock()
        self.lock = threading.Lock()             # worker thread stamps frames; rows are written by the executor
        self.rows = []
        self.counts = collections.Counter()
        self.acq = collections.OrderedDict()     # pair stamp ns -> wait record
        self.arrivals = {"depth": collections.OrderedDict(), "rgb": collections.OrderedDict()}
        self.tf_last = None                      # (arrival raw, newest transform stamp ns)
        self.tf_max_gap_ns = 0
        self.last_poll = None                    # (raw, thread cpu, process cpu)
        self.max_poll_gap_ns = 0
        self.section_max = {}
        self.cur = None
        self.closed = False
        self.graph = None                        # last set of ROS node names seen (participants joining/leaving)
        self.graph_next = 0
        self.write({"event": "stall_diag_start", "raw_ns": clock(), "inject": list(self.inject) if self.inject else None,
                    "gil_period_ms": gil_period_s * 1e3, "gil_min_ms": gil_min_ms, "gc_min_ms": gc_min_ms,
                    "poll_gap_ms": poll_gap_ms, "tf_gap_ms": tf_gap_ms, "poll_section_ms": poll_section_ms})

    # ------------------------------------------------------------------ output
    def write(self, row):
        self.rows.append(row)

    def flush(self, force=False):
        now = self.clock()
        if not force and now - self.last_flush < self.flush_ns:
            return
        self.last_flush = now
        for ev in self.events.drain():
            self.write(ev)
        rows, self.rows = self.rows, []
        if self.fp is not None and rows:
            self.fp.write("".join(json.dumps(r, allow_nan=False) + "\n" for r in rows))
            took = self.clock() - now
            if took > self.counts.get("flush_max_ns", 0):
                self.counts["flush_max_ns"] = took

    # ------------------------------------------------------------------ injection (positive control)
    def maybe_inject(self, section):
        if self.inject is None or self.inject[0] != section:
            return 0.0
        self.inject_seen[section] += 1
        if self.inject_seen[section] % self.inject[1]:
            return 0.0
        self.injected[section] += 1
        t0 = self.clock()
        (hold_gil_sleep if self.inject[3] == "hold" else free_gil_sleep)(self.inject[2])
        # exact time of every injection: lets the report match it to a frame row, a frameless poll
        # row and the GIL watchdog without guessing
        self.write({"event": "inject", "section": section, "ms": self.inject[2], "mode": self.inject[3],
                    "start_raw_ns": t0, "end_raw_ns": self.clock(),
                    "in_frame": self.cur["frame"].frame_id if self.cur is not None else None})
        return self.inject[2]

    # ------------------------------------------------------------------ arrivals (wrapped rclpy callbacks)
    def wrap_subscription(self, subscription, kind):
        """Replace an rclpy Subscription's callback (rclpy's executor reads sub.callback per
        message) by one that first stamps the arrival. kind: tf | depth | rgb."""
        inner = subscription.callback

        def timed(msg, _inner=inner):
            raw = self.clock()
            try:
                if kind == "tf":
                    self.on_tf(msg, raw)
                else:
                    self.on_image(kind, msg, raw)
            except Exception as exc:        # noqa: BLE001 - diagnostics never break the runner
                self.counts["arrival_error:" + type(exc).__name__] += 1
            return _inner(msg)
        subscription.callback = timed
        return inner

    def on_tf(self, msg, raw):
        newest = max((stamp_ns(t.header) for t in msg.transforms), default=None)
        self.counts["tf_messages"] += 1
        if self.tf_last is not None:
            gap = raw - self.tf_last[0]
            if gap > self.tf_max_gap_ns:
                self.tf_max_gap_ns = gap
            if gap >= self.tf_gap_ns:
                self.write({"event": "tf_arrival_gap", "gap_ms": gap / 1e6, "end_raw_ns": raw,
                            "prev_stamp_ns": self.tf_last[1], "stamp_ns": newest})
        self.tf_last = (raw, newest if newest is not None else (self.tf_last[1] if self.tf_last else None))

    def on_image(self, kind, msg, raw):
        table = self.arrivals[kind]
        table[stamp_ns(msg.header)] = raw
        while len(table) > 32:
            table.popitem(last=False)

    # ------------------------------------------------------------------ acquisition (prepare_images)
    def acquisition_wait(self, pair_stamp_ns, rx_raw, branch, exc_name, tf_latest_ns=None, trigger=None):
        rec = self.acq.get(pair_stamp_ns)
        now = self.clock()
        if rec is None:
            rec = self.acq[pair_stamp_ns] = {"first_wait_raw": now, "rx_raw": rx_raw, "tries": 0, "branches": collections.Counter(),
                                             "by": collections.Counter()}
            while len(self.acq) > 16:
                old_stamp, old = self.acq.popitem(last=False)
                self.counts["acquisition_wait_evicted"] += 1
        rec["tries"] += 1
        rec["branches"][branch + ":" + exc_name] += 1
        rec["by"][trigger or "?"] += 1              # v6: image | poll | tf (who called prepare_images)
        rec["last_raw"] = now
        lag = None if tf_latest_ns is None else (pair_stamp_ns - tf_latest_ns) / 1e6
        if lag is not None and (rec.get("max_tf_lag_ms") is None or lag > rec["max_tf_lag_ms"]):
            rec["max_tf_lag_ms"] = lag
        self.counts["acquisition_waits"] += 1

    def acquisition_dropped(self, pair_stamp_ns, rx_raw, reason, trigger=None):
        rec = self.acq.pop(pair_stamp_ns, None) or {}
        now = self.clock()
        self.counts["acquisition_dropped:" + reason] += 1
        self.write({"event": "acquisition_dropped", "reason": reason, "stamp_ns": pair_stamp_ns, "raw_ns": now,
                    "since_rx_ms": _ms(rx_raw, now), "tries": rec.get("tries", 0),
                    "branches": dict(rec.get("branches", {})), "max_tf_lag_ms": rec.get("max_tf_lag_ms"),
                    "tries_by": dict(rec.get("by", {})), "trigger": trigger,
                    "tf_last_arrival_ago_ms": _ms(self.tf_last[0], now) if self.tf_last else None,
                    "tf_last_stamp_lag_ms": ((pair_stamp_ns - self.tf_last[1]) / 1e6
                                             if self.tf_last and self.tf_last[1] is not None else None)})

    def acquisition_done(self, frame, pair_stamp_ns, depth_stamp_ns, rx_raw, trigger=None):
        rec = self.acq.pop(pair_stamp_ns, None) or {}
        frame.stall_diag = {"rx": rx_raw, "prepared": self.clock(),
                            "depth_rx": self.arrivals["depth"].get(depth_stamp_ns),
                            "rgb_rx": self.arrivals["rgb"].get(pair_stamp_ns),
                            "acq_tries": rec.get("tries", 0), "acq_branches": dict(rec.get("branches", {})),
                            "acq_max_tf_lag_ms": rec.get("max_tf_lag_ms"),
                            "acq_prepared_by": trigger, "acq_tries_by": dict(rec.get("by", {}))}

    def superseded(self, frame):
        self.counts["pending_superseded"] += 1
        d = getattr(frame, "stall_diag", None)
        if d is not None:
            self.write({"event": "frame_superseded", "frame_id": frame.frame_id, "raw_ns": self.clock(),
                        "since_rx_ms": _ms(d.get("rx"), self.clock())})

    # ------------------------------------------------------------------ inference (worker thread)
    def submitted(self, frame):
        d = getattr(frame, "stall_diag", None)
        if d is not None:
            d["submitted"] = self.clock()

    def detect_wrapper(self, detect):
        def timed(frame):
            d = getattr(frame, "stall_diag", None)
            t0, c0 = self.clock(), time.thread_time_ns()
            self.maybe_inject("infer")
            try:
                return detect(frame)
            finally:
                if d is not None:
                    d["infer_begin"], d["infer_end"] = t0, self.clock()
                    d["infer_cpu_ms"] = (time.thread_time_ns() - c0) / 1e6
        return timed

    def queued(self, frame):
        d = getattr(frame, "stall_diag", None)
        if d is not None:
            d["queued"] = self.clock()

    # ------------------------------------------------------------------ ROS graph (who joins / leaves)
    def graph_tick(self, names_fn, period_s=0.5):
        """Rate-limited: record node names added / removed since the last look (a new DDS
        participant joining is the integ v3 publish-stall trigger)."""
        now = self.clock()
        if now < self.graph_next:
            return
        self.graph_next = now + int(period_s * 1e9)
        t0 = now
        try:
            names = set("%s/%s" % (ns.rstrip("/"), n) for n, ns in names_fn())
        except Exception as exc:        # noqa: BLE001
            self.counts["graph_error:" + type(exc).__name__] += 1
            return
        if self.graph is not None and names != self.graph:
            self.write({"event": "graph_change", "raw_ns": now, "added": sorted(names - self.graph),
                        "removed": sorted(self.graph - names), "n": len(names), "query_ms": (self.clock() - t0) / 1e6})
        self.graph = names

    # ------------------------------------------------------------------ record() of other event kinds
    def record_timing(self, kind, t0, t1, t2, min_ms=50.0):
        """A record() call outside the frame_processed row: file write t0..t1, /fpga/status
        publish t1..t2 (it may sit inside a poll section, e.g. geometry_unavailable in geometry)."""
        self.counts["records_other"] += 1
        if (t2 - t0) >= min_ms * 1e6:
            self.counts["slow_records"] += 1
            self.write({"event": "slow_record", "kind": kind, "write_ms": (t1 - t0) / 1e6,
                        "publish_ms": (t2 - t1) / 1e6, "end_raw_ns": t2,
                        "in_frame": self.cur["frame"].frame_id if self.cur is not None else None})

    # ------------------------------------------------------------------ poll
    def poll_enter(self):
        self._poll_sections_end()
        now, tc, pc = self.clock(), time.thread_time_ns(), time.process_time_ns()
        if self.last_poll is not None:
            gap = now - self.last_poll[0]
            if gap > self.max_poll_gap_ns:
                self.max_poll_gap_ns = gap
            if gap >= self.poll_gap_ns:
                self.counts["poll_gaps"] += 1
                self.write({"event": "poll_gap", "gap_ms": gap / 1e6, "end_raw_ns": now,
                            "main_thread_cpu_ms": (tc - self.last_poll[1]) / 1e6,
                            "process_cpu_ms": (pc - self.last_poll[2]) / 1e6})
        self.last_poll = (now, tc, pc)
        self.poll_cur = {"t": now, "c": tc, "sections": {}, "last": None}

    def _poll_sections_end(self):
        """Close the previous poll's frameless sections: one poll_sections row when any section
        took >= poll_section_ms (marks made while no frame was open, e.g. the permit and the
        throttled /fpga/memory publish in the poll's finally path)."""
        pc, self.poll_cur = self.poll_cur, None
        if not pc or not pc["sections"]:
            return
        if max(v[0] for v in pc["sections"].values()) < self.poll_section_ms:
            return
        self.counts["poll_sections"] += 1
        self.write({"event": "poll_sections", "end_raw_ns": pc["last"],
                    "sections_ms": {k: round(v[0], 3) for k, v in pc["sections"].items()},
                    "sections_cpu_ms": {k: round(v[1], 3) for k, v in pc["sections"].items()}})

    def frame_begin(self, frame, held_ms=None):
        now = self.clock()
        d = getattr(frame, "stall_diag", None)
        if d is None:
            d = frame.stall_diag = {}
        d["picked"] = now
        self.cur = {"frame": frame, "d": d, "sections": {}, "t": now, "c": time.thread_time_ns(),
                    "p0": time.process_time_ns(), "held_ms": held_ms}

    def mark(self, section):
        """End of ``section`` (started at the previous mark) for the current frame; with no frame
        open, for the current poll (poll_sections row, written when the next poll starts)."""
        cur = self.cur if self.cur is not None else self.poll_cur
        if cur is None:
            return
        now, c = self.clock(), time.thread_time_ns()
        wall, cpu = (now - cur["t"]) / 1e6, (c - cur["c"]) / 1e6
        prev = cur["sections"].get(section)
        cur["sections"][section] = [wall + (prev[0] if prev else 0.0), cpu + (prev[1] if prev else 0.0)]
        cur["t"], cur["c"] = now, c
        if cur is self.poll_cur:
            cur["last"] = now
        if wall > self.section_max.get(section, 0.0):
            self.section_max[section] = wall

    def skip(self):
        """Restart the section clock without attributing (time spent outside a named section)."""
        cur = self.cur if self.cur is not None else self.poll_cur
        if cur is not None:
            cur["t"], cur["c"] = self.clock(), time.thread_time_ns()

    def frame_end(self, gate=None, origin_ns=None, basis=None, tracks=None, result=None):
        cur, self.cur = self.cur, None
        if cur is None:
            return None
        frame, d = cur["frame"], cur["d"]
        end = self.clock()
        gate_now = None
        if gate is not None and gate.get("age_ms") is not None and origin_ns is not None:
            gate_now = origin_ns + int(round(gate["age_ms"] * 1e6))
        spans = {
            "producer_to_depth_rx": _ms(origin_ns if basis == "producer_acquired" else None, d.get("depth_rx")),
            "depth_rx_to_rx": _ms(d.get("depth_rx"), d.get("rx")),
            "acquire": _ms(d.get("rx"), d.get("prepared")),
            "pending": _ms(d.get("prepared"), d.get("submitted")),
            "submit_to_infer": _ms(d.get("submitted"), d.get("infer_begin")),
            "infer": _ms(d.get("infer_begin"), d.get("infer_end")),
            "infer_to_queued": _ms(d.get("infer_end"), d.get("queued")),
            "queued_to_picked": _ms(d.get("queued"), d.get("picked")),
            "picked_to_gate": _ms(d.get("picked"), gate_now),
        }
        row = {"event": "frame_timing", "frame_id": frame.frame_id, "end_raw_ns": end,
               "age_basis": basis, "origin_raw_ns": origin_ns,
               "gate_age_ms": gate.get("age_ms") if gate else None, "gate_verdict": gate.get("verdict") if gate else None,
               "gate_reason": gate.get("reason") if gate else None, "result": result,
               "spans_ms": spans, "held_ms": cur["held_ms"],
               "sections_ms": {k: round(v[0], 3) for k, v in cur["sections"].items()},
               "sections_cpu_ms": {k: round(v[1], 3) for k, v in cur["sections"].items()},
               "poll_process_cpu_ms": (time.process_time_ns() - cur["p0"]) / 1e6,
               "infer_cpu_ms": d.get("infer_cpu_ms"), "acq_tries": d.get("acq_tries"),
               "acq_branches": d.get("acq_branches"), "acq_max_tf_lag_ms": d.get("acq_max_tf_lag_ms"),
               "acq_prepared_by": d.get("acq_prepared_by"), "acq_tries_by": d.get("acq_tries_by"),
               "tracks": tracks}
        self.counts["frames"] += 1
        self.write(row)
        self.flush()
        return row

    # ------------------------------------------------------------------ close
    def summary(self):
        return {"counts": dict(self.counts), "section_max_ms": dict(self.section_max),
                "max_poll_gap_ms": self.max_poll_gap_ns / 1e6, "tf_max_gap_ms": self.tf_max_gap_ns / 1e6,
                "gc": self.gc.summary(), "gil": self.watchdog.summary() if self.watchdog else None,
                "events_dropped": self.events.dropped, "injected": dict(self.injected),
                # v7: this file's and the peer (runner events) writers, as of this row
                "record_writers": [_writer_brief(w) for w in [self.out] + list(self.peer_writers) if w is not None]}

    def close(self):
        if self.closed:
            return
        self.closed = True
        self._poll_sections_end()
        if self.watchdog is not None:
            self.watchdog.stop()
            self.watchdog.join(1.0)
        self.gc.uninstall()
        self.write({"event": "stall_diag_summary", "raw_ns": self.clock(), **self.summary()})
        self.flush(force=True)
        self.fp = None
        self.writer_summary = self.out.close()
        return self.writer_summary
