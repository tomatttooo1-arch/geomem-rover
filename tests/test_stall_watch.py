"""host/stall_watch.py: the producer's publish-stall diagnostics (10-11 night).

Known answers with positive controls (each instrument must see a stall it is built for and stay
quiet for one it is not):
1. GilWatchdog: a 150 ms C sleep WITH the GIL held (ctypes.PyDLL usleep) -> one gil_stall row
   >= 100 ms; the same sleep with the GIL released (time.sleep) -> none.
2. StallDump: a section over the timeout -> faulthandler stacks of all threads in the file, the
   section footer naming label/frame/RAW times, a section_stall event; a short section -> nothing.
3. GcPauses: gc.collect() is counted per generation; a fake clock makes one collection 25 ms ->
   a gc_pause row; uninstall removes the hook.
4. CallTimer: wrap() returns the callback's value, records max and >= warn counts, rows >= event.
5. parse_debug_stall and EventQueue bound; tf_probe.thread_states parses /proc/self.
6. DeadmanDump re-armed by the watchdog: a 250 ms GIL-held C sleep -> stacks of all threads (the
   sleeping caller visible) + footer; a GIL-free sleep of the same length -> no dump.
Python 3.8+, no ROS.
"""
import gc
import os
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host import stall_watch  # noqa: E402
from tools.integ_car_chain import tf_probe  # noqa: E402


class WatchdogTests(unittest.TestCase):
    def run_with(self, sleeper):
        q = stall_watch.EventQueue()
        wd = stall_watch.GilWatchdog(q, period_s=0.01, min_ms=60.0)
        wd.start()
        time.sleep(0.1)
        sleeper(150)
        time.sleep(0.1)
        wd.stop()
        wd.join(2)
        return [e for e in q.drain() if e["event"] == "gil_stall"], wd

    def test_gil_held_sleep_is_seen(self):
        rows, wd = self.run_with(stall_watch.hold_gil_sleep)
        self.assertEqual(len(rows), 1, rows)
        self.assertGreaterEqual(rows[0]["late_ms"], 100.0)
        self.assertGreater(rows[0]["end_raw_ns"], rows[0]["start_raw_ns"])
        self.assertGreaterEqual(wd.summary()["max_overshoot_ms"], 100.0)

    def test_gil_free_sleep_is_not(self):
        rows, wd = self.run_with(stall_watch.free_gil_sleep)
        self.assertEqual(rows, [])
        self.assertGreater(wd.ticks, 10)

    def stopped_wake(self, elapsed_ms):
        """run() on this thread with a scripted clock: the wait returns "stopped" after
        ``elapsed_ms`` (stop() arrived before the watchdog could run again)."""
        q = stall_watch.EventQueue()
        t = iter([1000, 1000 + int(elapsed_ms * 1e6)])
        wd = stall_watch.GilWatchdog(q, period_s=0.01, min_ms=60.0, clock=lambda: next(t))

        class Stopped(object):
            def is_set(self):
                return False

            def wait(self, timeout):
                return True
        wd.stop_event = Stopped()
        wd.run()
        return [e for e in q.drain() if e["event"] == "gil_stall"], wd

    def test_stall_right_before_stop_is_still_recorded(self):
        # 10-12 (v7): the owner went from a 120 ms GIL-held stall straight to close(); with other
        # threads taking the GIL first the watchdog woke with the stop flag already set and left
        # without a row (full regression: 2 of 4 runs; 2/15 with 8 background sleepers)
        rows, wd = self.stopped_wake(130.0)
        self.assertEqual(len(rows), 1, rows)
        self.assertAlmostEqual(rows[0]["late_ms"], 120.0, places=6)
        self.assertEqual((rows[0]["start_raw_ns"], rows[0]["end_raw_ns"]), (1000, 1000 + 130000000))
        self.assertEqual((wd.over, wd.ticks), (1, 0))
        self.assertAlmostEqual(wd.summary()["max_overshoot_ms"], 120.0, places=6)

    def test_normal_stop_records_nothing(self):
        for elapsed in (3.0, 10.0, 60.0):          # stop inside the period / a late wake under min_ms
            rows, wd = self.stopped_wake(elapsed)
            self.assertEqual(rows, [], elapsed)
            self.assertEqual((wd.over, wd.ticks), (0, 0))


class StallDumpTests(unittest.TestCase):
    def test_long_section_dumps_all_threads(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "stacks.txt")
            q = stall_watch.EventQueue()
            sd = stall_watch.StallDump(path, timeout_s=0.05, events=q)
            other = threading.Event()
            th = threading.Thread(target=other.wait, args=(5,), name="bystander")
            th.start()
            t0 = sd.begin()
            time.sleep(0.12)
            sd.end(t0, "post_fusion_publish", frame_id=214, dpub_ms=297.0)
            t0 = sd.begin()
            sd.end(t0, "post_fusion_publish", frame_id=215)
            other.set()
            th.join()
            sd.close()
            with open(path) as fp:
                text = fp.read()
        self.assertIn("Timeout (0:00:00.050000)!", text)
        self.assertIn("test_long_section_dumps_all_threads", text)          # this thread
        self.assertIn("in wait", text)                                       # the bystander
        self.assertIn("=== ABOVE: section post_fusion_publish", text)
        self.assertIn("frame_id=214", text)
        self.assertNotIn("frame_id=215", text)
        ev = q.drain()
        self.assertEqual([e["frame_id"] for e in ev], [214])
        self.assertEqual(ev[0]["dpub_ms"], 297.0)
        self.assertEqual(sd.summary()["sections"], 2)
        self.assertEqual(sd.summary()["long"], 1)

    def test_disabled_dump_still_times(self):
        sd = stall_watch.StallDump(None, timeout_s=0.05, events=None)
        t0 = sd.begin()
        sd.end(t0, "x")
        self.assertEqual(sd.summary()["sections"], 1)


class GcTests(unittest.TestCase):
    def test_counts_and_known_pause(self):
        q = stall_watch.EventQueue()
        g = stall_watch.GcPauses(q, min_ms=10.0).install()
        try:
            gc.collect(0)
            gc.collect(2)
        finally:
            g.uninstall()
        self.assertGreaterEqual(g.n[0], 1)
        self.assertGreaterEqual(g.n[2], 1)
        self.assertNotIn(g, gc.callbacks)
        t = iter([0, 25000000])
        fake = stall_watch.GcPauses(q, min_ms=10.0, clock=lambda: next(t))
        q.drain()
        fake("start", {"generation": 2})
        fake("stop", {"generation": 2, "collected": 7, "uncollectable": 0})
        ev = q.drain()
        self.assertEqual(len(ev), 1)
        self.assertEqual((ev[0]["event"], ev[0]["generation"], ev[0]["pause_ms"]), ("gc_pause", 2, 25.0))
        self.assertEqual(fake.summary()["max_ms"][2], 25.0)


class CallTimerTests(unittest.TestCase):
    def test_wrap_records(self):
        q = stall_watch.EventQueue()
        t = iter([0, 5000000, 0, 60000000])
        ct = stall_watch.CallTimer(q, warn_ms=20.0, event_ms=50.0, clock=lambda: next(t))
        f = ct.wrap("on_depth", lambda x: x + 1)
        self.assertEqual(f(1), 2)
        self.assertEqual(f(2), 3)
        s = ct.summary()["calls"]["on_depth"]
        self.assertEqual((s["calls"], s["max_ms"], s["over_warn"]), (2, 60.0, 1))
        ev = q.drain()
        self.assertEqual([(e["name"], e["ms"]) for e in ev], [("on_depth", 60.0)])


class DeadmanTests(unittest.TestCase):
    def run_with(self, sleeper):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "stacks.txt")
            dm = stall_watch.DeadmanDump(path, timeout_s=0.1, rearm_s=0.02)
            q = stall_watch.EventQueue()
            wd = stall_watch.GilWatchdog(q, period_s=0.01, min_ms=60.0, deadman=dm)
            wd.start()
            time.sleep(0.15)
            sleeper(250)
            time.sleep(0.15)
            wd.stop()
            wd.join(2)
            dm.close()
            with open(path) as fp:
                return fp.read(), dm.summary()

    def test_gil_held_freeze_dumps(self):
        text, summ = self.run_with(stall_watch.hold_gil_sleep)
        self.assertIn("Timeout (0:00:00.100000)!", text)
        self.assertIn("hold_gil_sleep", text)
        self.assertIn("=== ABOVE: process frozen", text)
        self.assertEqual(summ["fired"], 1)

    def test_gil_free_sleep_no_dump(self):
        text, summ = self.run_with(stall_watch.free_gil_sleep)
        self.assertEqual(text, "")
        self.assertEqual(summ["fired"], 0)


class MiscTests(unittest.TestCase):
    def test_parse_debug_stall(self):
        self.assertIsNone(stall_watch.parse_debug_stall(None))
        self.assertEqual(stall_watch.parse_debug_stall("60:150:hold"), (60, 150.0, "hold"))
        with self.assertRaises(ValueError):
            stall_watch.parse_debug_stall("60:150:spin")

    def test_event_queue_bound(self):
        q = stall_watch.EventQueue(maxlen=3)
        for i in range(5):
            q.put({"i": i})
        self.assertEqual([e["i"] for e in q.drain()], [2, 3, 4])
        self.assertEqual(q.dropped, 2)
        self.assertEqual(q.drain(), [])

    def test_thread_states_of_self(self):
        th = tf_probe.thread_states(os.getpid())
        self.assertTrue(th)
        tids = [t[0] for t in th]
        self.assertIn(os.getpid(), tids)
        for tid, comm, st, ticks in th:
            self.assertIn(st, "RSDZTtWXIP")
            self.assertGreaterEqual(ticks, 0)
        self.assertEqual(tf_probe.thread_states(2 ** 22 + 12345), [])


if __name__ == "__main__":
    unittest.main()
