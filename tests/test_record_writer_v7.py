"""Integration v7 (2026-10-12): record files written by a background thread (host/record_writer.py).

Docs: docs/design/INTEG_CAR_CHAIN_20261011.md "第 7 版：记录文件后台写". Board v6: the runner's
events.jsonl write+flush blocked its executor 1430 ms (0.29 ms CPU, SD writeback); the KV260
withdrew the motion permit.

1. Known answer: the same rows give the same file bytes in async and sync mode (RecordWriter
   alone; and the production Runtime class over the 9 M5 software-replay scenarios: async, sync
   and the pre-v7 runner (6611e03, events written+flushed inline) give byte-identical
   events.jsonl files). Positive control: a writer that skips one row is caught.
2. Blocking injection: the file's write blocks 1.5 s. Async: every write()/record() returns in
   milliseconds, the queue holds exactly max_rows, the rest are dropped and counted exactly; the
   runner publishes a record_drops event and the hub shows it in STATE (task.record_drops /
   geometry_nano.record_drops). Positive control: sync mode blocks the caller >= 1.4 s.
3. Exit: close() drains everything accepted (slow file); the runner's SIGINT and SIGTERM path
   (spin_until_interrupt -> close) leaves every accepted row in the file; a writer stuck past
   the close timeout is reported, not hung on.
4. Stall diagnostics file: sync and async give the same bytes (fixed clock).
Python 3.8+.
"""
import contextlib
import itertools
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from host.record_writer import DropReporter, RecordWriter  # noqa: E402

PRE_V7 = "6611e03"          # public main before v7 (events written + flushed on the executor)
REAL_NS = types.SimpleNamespace


class BlockingFile(object):
    """File wrapper: the first ``block_writes`` write() calls sleep ``block_s`` (an SD-card
    writeback stall); ``started`` is set when the first write begins."""

    def __init__(self, fp, block_s=1.5, block_writes=1, every_s=0.0):
        self.fp, self.block_s, self.left, self.every_s = fp, block_s, block_writes, every_s
        self.started = threading.Event()
        self.calls = 0

    def write(self, text):
        self.calls += 1
        self.started.set()
        if self.left > 0:
            self.left -= 1
            time.sleep(self.block_s)
        elif self.every_s:
            time.sleep(self.every_s)
        return self.fp.write(text)

    def flush(self):
        self.fp.flush()

    def close(self):
        self.fp.close()

    @property
    def closed(self):
        return self.fp.closed


def rows(n, tag="r"):
    return [json.dumps({"event": tag, "i": i, "text": "é" * (i % 7), "pad": "x" * (i * 13 % 300)},
                       ensure_ascii=False) + "\n" for i in range(n)]


class Unit(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="recw_")

    def path(self, name):
        return os.path.join(self.dir, name)

    def test_async_equals_sync_bytes(self):
        data = rows(600)
        for mode in ("sync", "async"):
            w = RecordWriter(open(self.path(mode), "x", encoding="utf8"), mode=mode, name=mode)
            for r in data:
                self.assertTrue(w.write(r))
            s = w.close()
            self.assertEqual((s["accepted"], s["written"], s["dropped"], s["close"]), (600, 600, 0, "drained"))
        a, b = open(self.path("sync"), "rb").read(), open(self.path("async"), "rb").read()
        self.assertEqual(a, b)
        self.assertEqual(a.decode("utf8"), "".join(data))

    def test_threads_keep_their_order(self):
        w = RecordWriter(open(self.path("t"), "x"), mode="async")

        def feed(k):
            for i in range(300):
                w.write("%d %d\n" % (k, i))
        ts = [threading.Thread(target=feed, args=(k,)) for k in range(4)]
        [t.start() for t in ts]
        [t.join() for t in ts]
        w.close()
        got = [tuple(map(int, l.split())) for l in open(self.path("t"))]
        self.assertEqual(len(got), 1200)
        for k in range(4):
            self.assertEqual([i for kk, i in got if kk == k], list(range(300)))

    def test_blocked_file_never_blocks_the_caller_and_drops_are_exact(self):
        slow_rows = []
        f = BlockingFile(open(self.path("b"), "x"), block_s=1.5)
        w = RecordWriter(f, mode="async", max_rows=5, on_slow=slow_rows.append)
        data = rows(21)
        self.assertTrue(w.write(data[0]))
        self.assertTrue(f.started.wait(2.0))          # the writer thread is now stuck in write()
        took = []
        for r in data[1:]:
            t0 = time.perf_counter()
            w.write(r)
            took.append(time.perf_counter() - t0)
            time.sleep(0.01)
        self.assertLess(max(took), 0.02, took)
        s = w.summary()
        self.assertEqual(s["accepted"], 6)            # 1 being written + 5 queued
        self.assertEqual(s["dropped"], 15)
        self.assertEqual(s["dropped_bytes"], sum(len(r) for r in data[6:]))
        self.assertEqual(s["max_queue_rows"], 5)
        s = w.close()
        self.assertEqual((s["written"], s["close"], s["slow_writes"]), (6, "drained", 1))
        self.assertGreaterEqual(s["max_write_ms"], 1400)
        self.assertEqual(open(self.path("b")).read(), "".join(data[:6]))
        self.assertEqual(len(slow_rows), 1)
        self.assertEqual(slow_rows[0]["event"], "record_write_slow")

    def test_positive_control_sync_blocks_the_caller(self):
        f = BlockingFile(open(self.path("s"), "x"), block_s=1.5)
        w = RecordWriter(f, mode="sync")
        t0 = time.perf_counter()
        w.write("a\n")
        self.assertGreaterEqual(time.perf_counter() - t0, 1.4)
        w.close()

    def test_byte_bound(self):
        f = BlockingFile(open(self.path("y"), "x"), block_s=0.5)
        w = RecordWriter(f, mode="async", max_bytes=25)
        w.write("0" * 9 + "\n")
        self.assertTrue(f.started.wait(2.0))
        self.assertTrue(w.write("1" * 9 + "\n"))
        self.assertTrue(w.write("2" * 9 + "\n"))
        self.assertFalse(w.write("3" * 9 + "\n"))     # 20 + 10 > 25
        self.assertEqual((w.dropped, w.counts["dropped_bytes"]), (1, 10))
        w.close()
        self.assertEqual(open(self.path("y")).read(), "0" * 9 + "\n" + "1" * 9 + "\n" + "2" * 9 + "\n")

    def test_close_drains_slow_file_then_refuses(self):
        f = BlockingFile(open(self.path("c"), "x"), block_s=0.0, block_writes=0, every_s=0.02)
        w = RecordWriter(f, mode="async")
        data = rows(50)
        for r in data:
            w.write(r)
            time.sleep(0.001)
        s = w.close()
        self.assertEqual((s["written"], s["close"]), (50, "drained"))
        self.assertEqual(open(self.path("c")).read(), "".join(data))
        self.assertFalse(w.write("late\n"))
        self.assertEqual(w.counts["late"], 1)

    def test_close_timeout_is_reported(self):
        f = BlockingFile(open(self.path("o"), "x"), block_s=1.0)
        w = RecordWriter(f, mode="async")
        w.write("a\n")
        self.assertTrue(f.started.wait(2.0))
        t0 = time.perf_counter()
        s = w.close(timeout_s=0.2)
        self.assertLess(time.perf_counter() - t0, 0.6)
        self.assertEqual(s["close"], "timeout")
        self.assertFalse(f.closed)                    # left to the stuck writer
        w._thread.join(3.0)
        self.assertEqual(open(self.path("o")).read(), "a\n")

    def test_drop_reporter(self):
        r = DropReporter(1.0)
        self.assertFalse(r.due(0, 0.0))
        self.assertTrue(r.due(1, 0.0))
        self.assertFalse(r.due(3, 0.5))
        self.assertTrue(r.due(3, 1.0))
        self.assertFalse(r.due(3, 5.0))
        self.assertTrue(r.due(4, 1.1, final=True))


# ---------------------------------------------------------------- the production runner
def m5_events(source, mode, out_dir, module_name):
    """Run the 9 M5 software-replay scenarios with ``source``'s Runtime, events file per scenario
    (args.events; args.record_write = mode unless None). Fixed RAW clock: the age gate's
    wall-clock ages are the only wall-time fields of the events."""
    import test_region_source_none_regression as reg
    import host.fpga_clock as pc
    os.makedirs(out_dir)
    n = itertools.count()

    def factory(**kw):
        if kw.get("socket") == "offline-no-socket" and "pose_source" in kw:
            kw["events"] = os.path.join(out_dir, "%02d.jsonl" % next(n))
            if mode is not None:
                kw["record_write"] = mode
        return REAL_NS(**kw)
    # The fixed clock must reach every module that bound fpga_clock.now_ns at import (the replay's
    # raw_now_ns stamps geometry_origin_ns, the runtime ages it). Import the replay BEFORE patching
    # (a first import under the patch would bind the constant for the rest of the test process),
    # then patch each bound name; all restored on exit. (10-12: the full discover run imported the
    # replay earlier, so only pc.now_ns was patched there -> real-clock origins vs fixed-clock ages.)
    import tools.m5_software_replay  # noqa: F401
    orig = pc.now_ns
    bound = [(m, k) for m in list(sys.modules.values())
             if getattr(m, "__name__", "").startswith(("host", "tools", "perception"))
             for k, v in list(vars(m).items()) if v is orig]
    with contextlib.ExitStack() as stack:
        stack.enter_context(patch.object(types, "SimpleNamespace", factory))
        for m, k in bound:
            stack.enter_context(patch.object(m, k, lambda: 10 ** 12))
        _, passed = reg.capture(source, module_name)
    return {f: open(os.path.join(out_dir, f), "rb").read() for f in sorted(os.listdir(out_dir))}, passed


def git_show(rev, path):
    try:
        return subprocess.check_output(["git", "show", "%s:%s" % (rev, path)], cwd=str(ROOT),
                                       stderr=subprocess.DEVNULL).decode("utf-8")
    except (OSError, subprocess.CalledProcessError):
        return None


def load_tf_runtime(events, mode, name):
    """The Runtime class (reference-pose branch) on test_runner_tf_retry's scripted world, with an
    events file."""
    import test_runner_tf_retry as tr

    def factory(**kw):
        if "pose_source" in kw:
            kw.update(events=events, record_write=mode)
        return REAL_NS(**kw)
    with patch.object(tr, "NS", factory):
        node, world = tr.load_runtime(tr.current_source(), name)
    node.destroy_node = lambda: None
    deadline = time.monotonic() + 5.0          # the startup row is on disk before the test swaps the file
    while node.events.queued()[0] and time.monotonic() < deadline:
        time.sleep(0.005)
    while node.events.counts["written"] < 1 and time.monotonic() < deadline:
        time.sleep(0.005)
    return node, world


def retry_free(files):
    """Events files without the rows/keys the 10-25 find-retry added (test_car_geometry_kv.retry_normalise; the
    events file holds the /fpga/status payloads), byte lines otherwise unchanged."""
    from test_car_geometry_kv import retry_normalise
    out = {}
    for name, data in files.items():
        lines = []
        for line in data.splitlines(keepends=True):
            row = json.loads(line)
            kept = retry_normalise({"event": "publish", "topic": "/fpga/status", "payload": row})
            if kept is None:
                continue
            lines.append(line if kept["payload"] == row else
                         (json.dumps(kept["payload"], ensure_ascii=False, allow_nan=False) + "\n").encode())
        out[name] = b"".join(lines)
    return out


class RunnerKnownAnswer(unittest.TestCase):
    def test_events_bytes_async_sync_and_pre_v7(self):
        src = (ROOT / "host" / "ros_runtime.py").read_text()
        d = tempfile.mkdtemp(prefix="recw_m5_")
        got = {}
        for mode in ("async", "sync"):
            got[mode], passed = m5_events(src, mode, os.path.join(d, mode), "rt_v7_" + mode)
            self.assertTrue(all(p for p, _ in passed.values()), passed)
        self.assertEqual(len(got["async"]), 9)
        self.assertGreaterEqual(sum(len(b.splitlines()) for b in got["async"].values()), 150)
        self.assertEqual(got["async"], got["sync"])
        old = git_show(PRE_V7, "host/ros_runtime.py")
        if old is None:
            self.skipTest("no git history (board/image tree)")
        self.assertNotIn("--record-write", old)
        got["old"], _ = m5_events(old, None, os.path.join(d, "old"), "rt_pre_v7")
        self.assertEqual(retry_free(got["async"]), retry_free(got["old"]))

    def test_positive_control_skipped_row_is_caught(self):
        src = (ROOT / "host" / "ros_runtime.py").read_text()
        d = tempfile.mkdtemp(prefix="recw_m5pc_")
        good, _ = m5_events(src, "async", os.path.join(d, "a"), "rt_v7_pc_a")
        orig = RecordWriter.write
        count = itertools.count()

        def skipping(self, text):
            if next(count) == 7:
                return True               # accepted but never written
            return orig(self, text)
        with patch.object(RecordWriter, "write", skipping):
            bad, _ = m5_events(src, "async", os.path.join(d, "b"), "rt_v7_pc_b")
        self.assertNotEqual(good, bad)


class RunnerBlocking(unittest.TestCase):
    def run_arm(self, mode, n=20, max_rows=5):
        d = tempfile.mkdtemp(prefix="recw_rt_")
        path = os.path.join(d, "events.jsonl")
        node, world = load_tf_runtime(path, mode, "rt_block_" + mode)
        node.events.max_rows = max_rows
        f = BlockingFile(node.events.fp, block_s=1.5)
        node.events.fp = f
        took = []
        for i in range(n):
            t0 = time.perf_counter()
            node.record("frame_processed", frame_id="f%d" % i)
            took.append(time.perf_counter() - t0)
            if i == 0 and mode == "async":
                self.assertTrue(f.started.wait(2.0))
        node.close()
        status = [e["payload"] for e in world.events if e["event"] == "publish" and e["topic"] == "/fpga/status"]
        return node, path, took, status

    def test_async_record_does_not_wait_and_drops_are_reported(self):
        node, path, took, status = self.run_arm("async")
        self.assertLess(max(took), 0.05, took)
        s = node.events.summary()
        # startup row was written before the block; f0 is the stuck write; 5 queued; the rest dropped
        self.assertEqual(s["max_queue_rows"], 5)
        self.assertGreaterEqual(s["dropped"], 14)
        drops = [p for p in status if p["event"] == "record_drops"]
        self.assertTrue(drops)
        self.assertEqual(drops[0]["dropped"], 1)          # reported at the first drop
        # and the total at close (the report row itself may be dropped too: not in its own count)
        self.assertIn(s["dropped"] - drops[-1]["dropped"], (0, 1))
        self.assertEqual(drops[-1]["process"], "runner")
        kinds = [json.loads(l)["event"] for l in open(path)]
        self.assertEqual(kinds[0], "startup")
        written = [json.loads(l).get("frame_id") for l in open(path) if '"frame_processed"' in l]
        self.assertEqual(written, ["f%d" % i for i in range(6)])     # in order, the first 6
        from host.hub.hub_core import HubCore
        core = HubCore("replay")
        t = 10 ** 12
        self.assertIsNone(core.compose_state(t)["task"]["record_drops"])
        for p in status:
            core.feed_status(json.dumps(p), t)
        st = core.compose_state(t + 10 ** 6)
        self.assertEqual(st["task"]["record_drops"], drops[-1]["dropped"])
        self.assertIsNone(st["geometry_nano"]["record_drops"])
        core.feed_status(json.dumps({"event": "record_drops", "process": "producer", "dropped": 7}), t)
        self.assertEqual(core.compose_state(t + 2 * 10 ** 6)["geometry_nano"]["record_drops"], 7)

    def test_positive_control_sync_record_waits(self):
        node, path, took, status = self.run_arm("sync", n=3)
        self.assertGreaterEqual(took[0], 1.4)
        self.assertFalse([p for p in status if p["event"] == "record_drops"])
        self.assertEqual(len([l for l in open(path) if '"frame_processed"' in l]), 3)


class RunnerExit(unittest.TestCase):
    def setUp(self):
        self.saved = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}

    def tearDown(self):
        for s, h in self.saved.items():
            signal.signal(s, h)

    def run_signal(self, signum):
        from test_ros_runtime_shutdown import FakeRclpy, NO
        from host.ros_runtime import spin_until_interrupt
        d = tempfile.mkdtemp(prefix="recw_exit_")
        path = os.path.join(d, "events.jsonl")
        node, world = load_tf_runtime(path, "async", "rt_exit_%d" % signum)
        node.events.fp = BlockingFile(node.events.fp, block_s=0.0, block_writes=0, every_s=0.3)
        for i in range(12):
            node.record("frame_processed", frame_id="f%d" % i)
        self.assertGreater(node.events.queued()[0], 0)      # still queued when the signal comes
        node.tick = lambda i: os.kill(os.getpid(), signum) if i == 0 else None
        spin_until_interrupt(FakeRclpy(), lambda: node, [], NO)
        kinds = [json.loads(l) for l in open(path)]
        return node, kinds

    def test_sigint_and_sigterm_drain(self):
        for signum in (signal.SIGINT, signal.SIGTERM):
            node, got = self.run_signal(signum)
            frames = [r["frame_id"] for r in got if r["event"] == "frame_processed"]
            self.assertEqual(frames, ["f%d" % i for i in range(12)])
            self.assertEqual(got[-1]["event"], "geometry_close")
            s = node.events.summary()
            self.assertEqual((s["close"], s["dropped"], s["written"], s["accepted"]),
                             ("drained", 0, len(got), len(got)))


class StallDiagFile(unittest.TestCase):
    def write_arm(self, mode):
        from host.runner_stall_diag import RunnerStallDiag
        p = os.path.join(tempfile.mkdtemp(prefix="recw_sd_"), "sd.jsonl")
        tick = itertools.count(10 ** 12, 10 ** 6)
        d = RunnerStallDiag(p, clock=lambda: next(tick), watchdog=False, gc_min_ms=1e9, flush_s=0.0,
                            record_write=mode)
        for i in range(30):
            d.poll_enter()
            d.frame_begin(types.SimpleNamespace(frame_id="f%d" % i))
            d.mark("result")
            d.mark("record_write")
            d.frame_end(gate={"age_ms": 50.0, "verdict": "pass", "reason": None}, origin_ns=10 ** 12,
                        basis="producer_acquired")
        s = d.close()
        self.assertEqual((s["mode"], s["dropped"], s["close"]), (mode, 0, "drained"))
        return open(p, "rb").read()

    def test_same_bytes_both_modes(self):
        # fixed RAW clock and fixed CPU clocks (the rows carry this thread's CPU ms per section)
        with patch.object(time, "thread_time_ns", lambda: 0), patch.object(time, "process_time_ns", lambda: 0):
            out = {mode: self.write_arm(mode) for mode in ("sync", "async")}
        self.assertEqual(out["sync"].count(b"frame_timing"), 30)
        # every row but the last (stall_diag_summary names its own writer's mode) byte for byte
        self.assertEqual(out["sync"].splitlines()[:-1], out["async"].splitlines()[:-1])
        a, b = json.loads(out["sync"].splitlines()[-1]), json.loads(out["async"].splitlines()[-1])
        self.assertEqual([w["mode"] for w in a.pop("record_writers")], ["sync"])
        self.assertEqual([w["mode"] for w in b.pop("record_writers")], ["async"])
        self.assertEqual(a, b)


if __name__ == "__main__":
    unittest.main()
