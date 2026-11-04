"""Integration v5 (2026-10-12) controller rulings on top of the v5 small fixes (test_hub_v5_fixes):

1. B.3 source check also uses the runner's /fpga/runtime_info sensor_source: when it disagrees with the
   producer's frame_stats.source the hub sets geometry_nano.runner_source_warning and sends one EVENT
   source_warning (origin runtime_info) per change -- warn only: frame-level source and group src are
   untouched, the producer's own geometry_nano.source_warning is not overwritten.
3. task.runtime_info is not in every STATE: sent when it changed (first STATE counts), then every 5 s,
   absent otherwise; a new client's snapshot always has it; pc/fpgac.py keeps the last value.

Old-behaviour controls load the pre-ruling module from git 9a58274 (v5 fixes without the rulings).
Python 3.8+, no ROS.
"""
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from pc import fpgac  # noqa: E402
from host import ros_runtime  # noqa: E402
from host.hub import hub_core  # noqa: E402
from host.hub.hub_core import HubCore, NS, MS, RUNTIME_INFO_RESEND_NS  # noqa: E402
from host.hub.hub_server import HubServer  # noqa: E402

PRE_RULINGS = "9a58274"
TOKEN = "hub-v5-rulings-token-0123"
_tmp = tempfile.mkdtemp(prefix="v5_rulings_old_")


def old_module(path, name, commit=PRE_RULINGS):
    """``path`` as of ``commit``, imported as its own module (None without git history)."""
    if shutil.which("git") is None:
        return None
    r = subprocess.run(["git", "-C", str(ROOT), "show", "%s:%s" % (commit, path)],
                       stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    if r.returncode != 0:
        return None
    f = Path(_tmp) / (name + ".py")
    f.write_bytes(r.stdout)
    spec = importlib.util.spec_from_file_location(name, str(f))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ARGS = types.SimpleNamespace(sensor_source="replay", pose_source="reference", inference_device="cpu",
                             geometry_fallback="none", region_source="none", tf_reliable=False, stall_diag=None)


def info(sensor="replay", boot="rt-1"):
    a = types.SimpleNamespace(**dict(vars(ARGS), sensor_source=sensor))
    return json.dumps(ros_runtime.runtime_info_body(a, boot, "kv_topic", 5, 200.0, version={"commit": None}))


def stats(source, i=0, **kw):
    row = {"v": 1, "kv_frame_id": i, "source": source}
    row.update(kw)
    return json.dumps(row)


def warn_events(core, now):
    return [e for e in core.pop_events(now) if e["event"].get("origin") == "runtime_info"]


# ------------------------------------------------------------------------------------- 1
class RunnerVsProducerSource(unittest.TestCase):
    def setUp(self):
        self.core = HubCore("replay")
        self.core.feed_status(json.dumps({"event": "startup", "sensor_source": "replay"}), NS)

    def test_agreeing_sources_never_warn(self):
        self.core.feed_runtime_info(info("replay"), NS)
        for i in range(30):
            self.core.feed_frame_stats(stats("replay", i), NS + i * MS)
        s = self.core.compose_state(NS + 40 * MS)
        self.assertIsNone(s["geometry_nano"]["runner_source_warning"])
        self.assertEqual(warn_events(self.core, NS + 40 * MS), [])

    def test_runner_hardware_vs_producer_replay_warns_once_and_clears(self):
        self.core.feed_runtime_info(info("hardware"), NS)            # runner says live, producer replay
        for i in range(20):
            self.core.feed_frame_stats(stats("replay", i), NS + i * MS)
        s = self.core.compose_state(NS + 30 * MS)
        self.assertEqual(s["geometry_nano"]["runner_source_warning"], "runner_producer_mismatch")
        # warn only: frame source and group src are what they were (startup replay == --data-source)
        self.assertEqual(s["source"], "replay")
        self.assertEqual((s["geometry_nano"]["src"], s["perf"]["src"]), ("replay", "replay"))
        self.assertIsNone(s["geometry_nano"]["source_warning"])       # the producer's field is its own
        evs = warn_events(self.core, NS + 30 * MS)
        self.assertEqual(len(evs), 1)                                   # one per change, not per frame
        self.assertEqual(evs[0]["src"], "unknown")
        self.assertEqual(evs[0]["event"], {"event": "source_warning", "origin": "runtime_info", "source": "hub",
                                           "reason": "runner_producer_mismatch", "runner_sensor_source": "hardware",
                                           "producer_source": "replay", "data_source": "replay"})
        self.assertEqual(self.core.counts["runner_producer_source_warning"], 1)
        self.core.feed_runtime_info(info("replay", boot="rt-2"), NS + 50 * MS)   # runner restarted as replay
        s = self.core.compose_state(NS + 60 * MS)
        self.assertIsNone(s["geometry_nano"]["runner_source_warning"])
        evs = warn_events(self.core, NS + 60 * MS)
        self.assertEqual([(e["event"]["event"], e["event"]["runner_sensor_source"]) for e in evs],
                         [("source_warning_cleared", "replay")])

    def test_order_independent_and_producer_side_flip(self):
        self.core.feed_frame_stats(stats("live"), NS)                  # producer first
        self.assertEqual(warn_events(self.core, NS), [])                # runner unknown: no warning
        self.core.feed_runtime_info(info("replay"), NS + MS)
        self.assertEqual([e["event"]["event"] for e in warn_events(self.core, NS + MS)], ["source_warning"])
        self.core.feed_frame_stats(stats("replay", 1), NS + 2 * MS)
        self.assertEqual([e["event"]["event"] for e in warn_events(self.core, NS + 2 * MS)],
                         ["source_warning_cleared"])

    def test_missing_side_is_no_information(self):
        self.core.feed_runtime_info(info("hardware"), NS)
        self.core.feed_frame_stats(json.dumps({"v": 1, "kv_frame_id": 0}), NS)     # no source mark
        self.assertIsNone(self.core.compose_state(NS)["geometry_nano"]["runner_source_warning"])
        bare = HubCore("replay")
        bare.feed_frame_stats(stats("live"), NS)                                     # no runtime_info
        bare.feed_runtime_info(json.dumps({"v": 1}), NS)                             # body without the key
        self.assertIsNone(bare.compose_state(NS)["geometry_nano"]["runner_source_warning"])
        self.assertEqual(warn_events(bare, NS), [])

    def test_unmapped_runner_value_counts_as_disagreement(self):
        self.core.feed_runtime_info(json.dumps({"v": 1, "sensor_source": "camera9"}), NS)
        self.core.feed_frame_stats(stats("replay"), NS)
        self.assertEqual(self.core.compose_state(NS)["geometry_nano"]["runner_source_warning"],
                         "runner_producer_mismatch")

    def test_both_warnings_side_by_side(self):
        self.core.feed_runtime_info(info("hardware"), NS)
        self.core.feed_frame_stats(stats("replay", source_warning="replay_without_marker"), NS)
        g = self.core.compose_state(NS)["geometry_nano"]
        self.assertEqual((g["source_warning"], g["runner_source_warning"]),
                         ("replay_without_marker", "runner_producer_mismatch"))

    def test_old_hub_did_not_compare(self):
        old = old_module("host/hub/hub_core.py", "pre_rulings_hub_core_1")
        if old is None:
            self.skipTest("no git history")
        core = old.HubCore("replay")
        core.feed_status(json.dumps({"event": "startup", "sensor_source": "replay"}), NS)
        core.feed_runtime_info(info("hardware"), NS)
        core.feed_frame_stats(stats("replay"), NS)
        g = core.compose_state(NS)["geometry_nano"]
        self.assertNotIn("runner_source_warning", g)
        self.assertEqual([e for e in core.pop_events(NS) if e["event"].get("event", "").startswith("source_")], [])


# ------------------------------------------------------------------------------------- 3
class RuntimeInfoCadence(unittest.TestCase):
    def ticks(self, core, start, n, feeds=None, step=100 * MS):
        """compose_state every ``step``; ``feeds`` {tick index: text}; returns indices carrying the key."""
        out = []
        for i in range(n):
            t = start + i * step
            if feeds and i in feeds:
                core.feed_runtime_info(feeds[i], t - MS)
            if "runtime_info" in core.compose_state(t)["task"]:
                out.append(i)
        return out

    def test_known_schedule(self):
        core = HubCore("replay")
        # tick 0: first STATE (null = never received); tick 3: body arrives; then every 5 s = 50 ticks
        got = self.ticks(core, NS, 130, feeds={3: info()})
        self.assertEqual(got, [0, 3, 53, 103])
        self.assertEqual(RUNTIME_INFO_RESEND_NS, 5 * NS)

    def test_null_also_follows_the_schedule(self):
        core = HubCore("replay")
        self.assertEqual(self.ticks(core, NS, 101), [0, 50, 100])
        self.assertIsNone(HubCore("replay").compose_state(NS)["task"]["runtime_info"])

    def test_body_change_goes_out_at_once_recv_age_does_not(self):
        core = HubCore("replay")
        got = self.ticks(core, NS, 40, feeds={2: info(), 20: info(), 30: info(boot="rt-2")})
        self.assertEqual(got, [0, 2, 30])        # same body re-received at 20 is not a change
        t = NS + 40 * 100 * MS
        core.feed_runtime_info(info(boot="rt-3"), t)
        ri = core.compose_state(t + 250 * MS)["task"]["runtime_info"]
        self.assertEqual((ri["runtime_boot_id"], ri["recv_age_ms"]), ("rt-3", 250.0))

    def test_forced_snapshot_does_not_move_the_schedule(self):
        core = HubCore("replay")
        got = []
        for i in range(60):
            t = NS + i * 100 * MS
            if i == 1:
                core.feed_runtime_info(info(), t - MS)
            if i == 20:                                       # a client connects: snapshot
                self.assertIn("runtime_info", core.compose_state(t, force_runtime_info=True)["task"])
            if "runtime_info" in core.compose_state(t)["task"]:
                got.append(i)
        self.assertEqual(got, [0, 1, 51])

    def test_snapshot_when_due_leaves_the_change_to_the_broadcast(self):
        core = HubCore("replay")
        core.compose_state(NS)                                  # null goes out
        core.feed_runtime_info(info(), NS + 50 * MS)            # a change is now due ...
        self.assertIn("runtime_info", core.compose_state(NS + 60 * MS, force_runtime_info=True)["task"])
        # ... and a client snapshot in between must not swallow it for the already connected ones
        self.assertIn("runtime_info", core.compose_state(NS + 100 * MS)["task"])
        self.assertNotIn("runtime_info", core.compose_state(NS + 200 * MS)["task"])
        t5 = NS + 100 * MS + RUNTIME_INFO_RESEND_NS             # 5 s refresh due: same rule
        self.assertIn("runtime_info", core.compose_state(t5, force_runtime_info=True)["task"])
        self.assertIn("runtime_info", core.compose_state(t5 + MS)["task"])

    def test_old_hub_sent_it_every_tick(self):
        old = old_module("host/hub/hub_core.py", "pre_rulings_hub_core_3")
        if old is None:
            self.skipTest("no git history")
        core = old.HubCore("replay")
        core.feed_runtime_info(info(), NS)
        self.assertEqual(sum("runtime_info" in core.compose_state(NS + i * 100 * MS)["task"] for i in range(30)), 30)


class RuntimeInfoKeeperTests(unittest.TestCase):
    def state(self, seq, boot="hub-a", ri=Ellipsis):
        task = {"src": "replay"}
        if ri is not Ellipsis:
            task["runtime_info"] = ri
        return {"seq_state": seq, "hub_boot_id": boot, "nano_mono_ns": seq * NS, "task": task}

    def test_keeps_last_value_through_ticks_without_it(self):
        k = fpgac.RuntimeInfoKeeper()
        self.assertEqual((k.present, k.value), (False, None))
        k.observe(self.state(1, ri=None))
        self.assertEqual((k.present, k.value, k.seq_state), (True, None, 1))   # Nano had not received it
        k.observe(self.state(2, ri={"sensor_source": "replay"}))
        for seq in range(3, 10):
            self.assertEqual(k.observe(self.state(seq)), {"sensor_source": "replay"})
        self.assertEqual((k.seq_state, k.nano_mono_ns, k.hub_boot_id), (2, 2 * NS, "hub-a"))
        k.observe(self.state(10, boot="hub-b"))                                  # another hub boot
        self.assertEqual((k.present, k.value), (False, None))
        k.observe("not a dict")
        self.assertFalse(k.present)

    def test_client_and_snapshot_end_to_end(self):
        core = HubCore("replay", hub_boot_id="hub-e2e")
        core.feed_status(json.dumps({"event": "startup", "sensor_source": "replay"}), 1)
        core.feed_runtime_info(info(), 1)
        server = HubServer(core, TOKEN, port=0, host="127.0.0.1").start()
        clients = []
        try:
            def connect():
                rows, lock = [], threading.Lock()

                def on(f, o):
                    if f.kind == fpgac.KIND["STATE"]:
                        with lock:
                            rows.append(o)
                c = fpgac.FpgacClient(server.port, TOKEN, hosts=["127.0.0.1"], on_frame=on,
                                    backoff_s=(0.1, 0.2)).start()
                clients.append(c)
                return c, rows, lock

            c1, rows1, lock1 = connect()
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and len(rows1) < 8:
                time.sleep(0.02)
            c2, rows2, lock2 = connect()                      # joins mid-run, after the field went out
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and len(rows2) < 5:
                time.sleep(0.02)
            with lock1, lock2:
                r1, r2 = list(rows1), list(rows2)
            self.assertGreaterEqual(len(r1), 8)
            self.assertGreaterEqual(len(r2), 5)
            self.assertIn("runtime_info", r2[0]["task"])     # snapshot carries it
            self.assertTrue(all("runtime_info" not in r["task"] for r in r2[1:]))
            self.assertTrue(all("runtime_info" not in r["task"] for r in r1[2:]))   # < 5 s run
            for c in (c1, c2):
                self.assertTrue(c.runtime_info.present)
                self.assertEqual(c.runtime_info.value["runtime_boot_id"], "rt-1")
        finally:
            for c in clients:
                c.stop()
            server.close()


if __name__ == "__main__":
    unittest.main()
