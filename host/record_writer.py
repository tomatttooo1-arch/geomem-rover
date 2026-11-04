"""Record files written off the caller's thread (2026-10-12, integration v7).

Why: on the board (v6, docs/design/INTEG_CAR_CHAIN_20261011.md "第 6 版上板结果" F2) the runner's
events.jsonl write+flush on its executor thread blocked 1430 ms with 0.29 ms CPU (SD-card
writeback on the Nano root file system, 89 % full), the producer's worker stalled 0.7 s at the
same moment on its own frames.jsonl, and the KV260 withdrew the motion permit. A record file is
evidence, never an input of a decision, so its write must not sit on a loop that publishes.

RecordWriter(fp, mode):
  async  write(text) only appends ``text`` to a bounded in-memory queue (rows and bytes) and
         returns; one daemon thread drains the queue in order, writes the rows joined and
         flushes after each drained batch. A full queue DROPS the new row (never blocks the
         caller) and counts it (``dropped``, ``dropped_bytes``); the caller makes the count
         visible (runner/producer ``record_drops`` event on /fpga/status -> hub STATE).
  sync   the behaviour before v7 (control arm): write(+flush when ``flush_each``) on the
         calling thread, serialised by a lock.
Rows reach the file in write() order and byte for byte as given, in both modes (only the
flush timing differs). close() drains everything accepted (bounded by ``close_timeout_s``;
a writer still stuck then is reported, the file is left open for it) -- the normal exit and
the SIGINT/SIGTERM paths of the runner and the producer both call it.

Every write (async batch or sync row) is timed; one that takes >= ``slow_ms`` is counted and
handed to ``on_slow(row)`` (called on the writing thread; must be cheap and thread safe, e.g.
stall_watch.EventQueue.put), so a run still shows the SD stalls even when they no longer stop
the loop. Python 3.8 compatible; no ROS import.
"""

import collections
import threading

from host.fpga_clock import now_ns

MODES = ("async", "sync")
DEFAULT_MODE = "async"
DEFAULT_MAX_ROWS = 4096             # ~400 s of the runner's ~10 rows/s
DEFAULT_MAX_BYTES = 16 << 20        # memory bound whatever the row size
DEFAULT_SLOW_MS = 100.0


class RecordWriter(object):
    def __init__(self, fp, mode=DEFAULT_MODE, name="records", flush_each=True, max_rows=DEFAULT_MAX_ROWS,
                 max_bytes=DEFAULT_MAX_BYTES, slow_ms=DEFAULT_SLOW_MS, on_slow=None, clock=now_ns,
                 close_timeout_s=10.0):
        if mode not in MODES:
            raise ValueError("record write mode must be one of %s" % (MODES,))
        if not (max_rows >= 1 and max_bytes >= 1):
            raise ValueError("queue bounds must be >= 1")
        self.fp, self.mode, self.name, self.flush_each = fp, mode, name, bool(flush_each)
        self.max_rows, self.max_bytes = int(max_rows), int(max_bytes)
        self.slow_ns, self.on_slow, self.clock = int(slow_ms * 1e6), on_slow, clock
        self.close_timeout_s = close_timeout_s
        self._cv = threading.Condition(threading.Lock())
        self._q = collections.deque()
        self._qbytes = 0
        self._stop = False
        self.closed = False
        self.counts = collections.Counter()   # rows accepted|dropped|late, bytes, written rows, writes...
        self.max_queue_rows = 0
        self.max_queue_bytes = 0
        self.max_write_ms = 0.0
        self.slow = collections.deque(maxlen=32)
        self.first_error = None
        self.close_state = None               # None | "drained" | "timeout"
        self._thread = None
        if mode == "async":
            self._thread = threading.Thread(target=self._run, name="record-writer-%s" % name, daemon=True)
            self._thread.start()

    @classmethod
    def open(cls, path, file_mode="x", encoding=None, **kw):
        return cls(open(path, file_mode, encoding=encoding) if encoding else open(path, file_mode), **kw)

    # ------------------------------------------------------------------ caller side
    @property
    def dropped(self):
        return self.counts["dropped"]

    def write(self, text):
        """Queue (async) or write (sync) ``text`` as given. True = accepted, False = dropped
        (queue full) or the writer is closed."""
        n = len(text)
        if self.mode == "sync":
            with self._cv:
                if self.closed:
                    self.counts["late"] += 1
                    return False
                self.counts["accepted"] += 1
                self.counts["bytes"] += n
                self._write([text], 1)
            return True
        with self._cv:
            if self.closed:
                self.counts["late"] += 1
                return False
            if len(self._q) >= self.max_rows or self._qbytes + n > self.max_bytes:
                self.counts["dropped"] += 1
                self.counts["dropped_bytes"] += n
                return False
            self.counts["accepted"] += 1
            self.counts["bytes"] += n
            self._q.append(text)
            self._qbytes += n
            if len(self._q) > self.max_queue_rows:
                self.max_queue_rows = len(self._q)
            if self._qbytes > self.max_queue_bytes:
                self.max_queue_bytes = self._qbytes
            self._cv.notify()
        return True

    def queued(self):
        with self._cv:
            return len(self._q), self._qbytes

    # ------------------------------------------------------------------ writing
    def _write(self, chunks, rows):
        t0 = self.clock()
        try:
            self.fp.write(chunks[0] if len(chunks) == 1 else "".join(chunks))
            if self.flush_each or self.mode == "async":
                self.fp.flush()
            self.counts["written"] += rows
        except Exception as exc:      # noqa: BLE001 - a record file never stops the loop that feeds it
            self.counts["write_errors"] += rows
            if self.first_error is None:
                self.first_error = "%s: %s" % (type(exc).__name__, str(exc)[:200])
        t1 = self.clock()
        self.counts["writes"] += 1
        ms = (t1 - t0) / 1e6
        if ms > self.max_write_ms:
            self.max_write_ms = ms
        if t1 - t0 >= self.slow_ns:
            self.counts["slow_writes"] += 1
            row = {"event": "record_write_slow", "file": self.name, "mode": self.mode, "ms": ms,
                   "rows": rows, "start_raw_ns": t0, "end_raw_ns": t1}
            self.slow.append(row)
            if self.on_slow is not None:
                try:
                    self.on_slow(row)
                except Exception:     # noqa: BLE001
                    self.counts["on_slow_errors"] += 1

    def _run(self):
        while True:
            with self._cv:
                while not self._q and not self._stop:
                    self._cv.wait()
                if not self._q:
                    return
                batch = list(self._q)
                self._q.clear()
                self._qbytes = 0
            self._write(batch, len(batch))

    # ------------------------------------------------------------------ exit
    def close(self, timeout_s=None):
        """Drain what was accepted, then close the file. Idempotent; returns summary()."""
        if self.closed:
            return self.summary()
        timeout_s = self.close_timeout_s if timeout_s is None else timeout_s
        with self._cv:
            self.closed = True
            self._stop = True
            self._cv.notify_all()
        alive = False
        if self._thread is not None:
            self._thread.join(timeout_s)
            alive = self._thread.is_alive()
        if alive:
            self.close_state = "timeout"      # still stuck in a write: leave the file to that thread
        else:
            self.close_state = "drained"
            try:
                self.fp.close()
            except Exception as exc:  # noqa: BLE001
                if self.first_error is None:
                    self.first_error = "close %s: %s" % (type(exc).__name__, str(exc)[:200])
        return self.summary()

    def summary(self):
        with self._cv:
            q_rows, q_bytes = len(self._q), self._qbytes
        return {"file": self.name, "mode": self.mode, "accepted": self.counts["accepted"],
                "written": self.counts["written"], "dropped": self.counts["dropped"],
                "dropped_bytes": self.counts["dropped_bytes"], "late": self.counts["late"],
                "write_errors": self.counts["write_errors"], "first_error": self.first_error,
                "writes": self.counts["writes"], "slow_writes": self.counts["slow_writes"],
                "max_write_ms": round(self.max_write_ms, 3), "max_queue_rows": self.max_queue_rows,
                "max_queue_bytes": self.max_queue_bytes, "queued_rows": q_rows, "queued_bytes": q_bytes,
                "max_rows": self.max_rows, "max_bytes": self.max_bytes, "close": self.close_state,
                "slow": list(self.slow)}


class DropReporter(object):
    """When to say that rows were dropped: the first drop at once, then at most every
    ``period_s`` while the count grows, and once more at close if it grew since the last
    report. ``due(total, now_s)`` -> True when a report should go out now."""

    def __init__(self, period_s=1.0):
        self.period_s = float(period_s)
        self.reported = 0
        self.last_s = None

    def due(self, total, now_s, final=False):
        if total <= self.reported:
            return False
        if final or self.last_s is None or now_s - self.last_s >= self.period_s:
            self.reported, self.last_s = total, now_s
            return True
        return False
