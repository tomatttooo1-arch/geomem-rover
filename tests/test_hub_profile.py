"""Opt-in per-thread CPU hook of the hub (host/hub/hub_profile.py): off by default, and when on
it records each thread's own CPU (a busy thread vs an idle one, known order) and task snapshots."""

import glob
import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest

from host.hub import hub_profile


def busy(seconds):
    t = time.thread_time()
    while time.thread_time() - t < seconds:
        sum(range(1000))


class HubProfileTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="hubprof-")

    def tearDown(self):
        hub_profile.uninstall()
        shutil.rmtree(self.dir, ignore_errors=True)

    def test_off_by_default_leaves_thread_run_untouched(self):
        before = threading.Thread.run
        self.assertIsNone(hub_profile.install({}))
        self.assertIs(threading.Thread.run, before)
        hub_profile.finish_main()                      # no-op, writes nothing
        self.assertEqual(os.listdir(self.dir), [])

    def test_per_thread_cpu_busy_vs_idle(self):
        before = threading.Thread.run
        out = os.path.join(self.dir, "p")
        tasks = os.path.join(out, "tasks.jsonl")
        self.assertEqual(hub_profile.install({"FPGA_HUB_PROFILE": out, "FPGA_HUB_PROFILE_TASK_S": "0.05"}), out)
        self.assertIsNot(threading.Thread.run, before)
        # No assertion here may depend on how many snapshots fit into a wall time: a snapshot cannot finish
        # while a CPU-bound Python thread runs (every /proc read of read_tasks releases the GIL and then waits
        # up to one switch interval, 5 ms, to get it back; ~50 kernel tasks x >= 3 system calls here), so it
        # ends when the busy thread does. Until 2026-10-12 this test let busy-one run 0.3 s and expected >= 2
        # snapshots at a 0.05 s period: 2 only if the first snapshot (2 ms) won the race against the start of
        # busy-one, 1 otherwise (always after test_hub_path_source's first classes, sometimes alone on 3.8).
        # busy-one therefore stays alive, idle after its 0.3 s of CPU, until two more snapshots are on file.
        hold = threading.Event()

        def busy_then_hold():
            busy(0.3)
            hold.wait(60)

        def snapshots():
            try:
                with open(tasks, "rb") as f:
                    return f.read().count(b"\n")
            except OSError:
                return 0

        a = threading.Thread(target=busy_then_hold, name="busy-one")
        b = threading.Thread(target=time.sleep, args=(0.3,), name="idle-one")
        a.start(); b.start()
        n0 = snapshots()                 # the one after the next was started with busy-one alive
        deadline = time.monotonic() + 60
        while snapshots() < n0 + 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        reached = snapshots()
        hold.set()
        a.join(); b.join()
        hub_profile.finish_main()
        hub_profile.uninstall()
        self.assertIs(threading.Thread.run, before)
        self.assertGreaterEqual(reached, n0 + 2)
        recs = {}
        for path in glob.glob(os.path.join(out, "*.json")):
            with open(path) as f:
                r = json.load(f)
            recs[r["name"]] = r
        self.assertGreaterEqual(recs["busy-one"]["cpu_s"], 0.29)
        self.assertLess(recs["busy-one"]["cpu_s"], 0.29 + 0.2)    # the hold is idle: CPU stays the busy part
        self.assertLess(recs["idle-one"]["cpu_s"], 0.05)          # positive control: idle is not busy
        self.assertIn("MainThread", recs)
        self.assertIsNotNone(recs["busy-one"]["native_id"])
        with open(tasks) as f:
            snaps = [json.loads(line) for line in f]
        self.assertGreaterEqual(len(snaps), 2)
        named = set()
        for s in snaps:
            named.update(s["py"].values())
            self.assertTrue(all(len(v) == 3 for v in s["tasks"].values()))
        self.assertIn("busy-one", named)
        # the Python thread is matched to a kernel task by native_id (no claim about its ticks at a given
        # instant: a snapshot taken beside the busy thread is read over the whole busy period, see above)
        tid = str(recs["busy-one"]["native_id"])
        self.assertTrue(any(tid in s["tasks"] and s["py"].get(tid) == "busy-one" for s in snaps))

    @unittest.skipIf(sys.version_info >= (3, 12), "cProfile is per process (sys.monitoring) on >= 3.12")
    def test_cprofile_mode_writes_prof(self):
        out = os.path.join(self.dir, "p")
        hub_profile.install({"FPGA_HUB_PROFILE": out, "FPGA_HUB_PROFILE_MODE": "cprofile"})
        t = threading.Thread(target=busy, args=(0.05,), name="prof-one")
        t.start(); t.join()
        hub_profile.uninstall()
        import pstats
        prof = glob.glob(os.path.join(out, "prof-one-*.prof"))
        self.assertEqual(len(prof), 1)
        st = pstats.Stats(prof[0])
        self.assertTrue(any(fn[2] == "busy" for fn in st.stats))


if __name__ == "__main__":
    unittest.main()
