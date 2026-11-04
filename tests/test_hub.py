"""Nano hub: core known answers (injected Nano clock) and loopback integration (spec B, C, E.2).

Controller override 10-11: any authenticated PC may assert e-stop; only the control holder
may release, find, select, cancel or yield.
"""

import collections
import json
import sys
import threading
import time
import types
import unittest

from pc import fpgac
from host import fpga_clock
from host.hub import hub_core
from host.hub.hub_core import HubCore, NS, MS
from host.hub.hub_server import HubServer, KvLoops, RosActionsLog
from host.hub.fake_kv import FakeKv, KvTelemetryClient, KvEstopClient

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from fpgac_relay import Relay  # noqa: E402

K = fpgac.KIND
TOKEN = "hub-test-token-0123456789"


def status(event, **kw):
    kw["event"] = event
    return json.dumps(kw)


def memory_msg(rev, n=2, state="historical_memory"):
    return json.dumps({"v": 1, "runtime_boot_id": "rt", "revision": rev, "digest": "d%d" % rev,
                       "world_frame": "map", "tracks": [{"id": "object-%04d" % i, "label": "bottle",
                                                         "state": state} for i in range(n)]})


def feed_everything(core, t0, sensor="hardware"):
    """One input of every kind at t0 + k ms (k = input index), startup first."""
    core.feed_status(status("startup", sensor_source=sensor), t0)
    core.feed_status(status("task", state="searching", reason="r", target_id=None, candidates=[],
                            processed_frames=4, dropped_frames=1), t0 + 1 * MS)
    core.feed_memory(memory_msg(3), t0 + 2 * MS)
    core.feed_region(json.dumps({"stamp_ros_s": 99.5, "frame_tag": "kvroi-1", "memory_revision": 3,
                                 "verdicts": [{"track_id": "object-0000", "geometry_evidence": "occluded",
                                               "code": 3}], "unchecked": []}), t0 + 3 * MS)
    core.feed_frame_stats(json.dumps({"frame_age_ms": 820.0, "channel_ms": 210.0, "mailbox_wait_ms": 4.0,
                                      "kv_times_ms": {"core_device": 604.0}, "client_counts": {}}), t0 + 4 * MS)
    core.feed_motion_permit(True, t0 + 5 * MS)
    core.feed_cmd_vel(0.2, 0.1, t0 + 6 * MS)
    core.feed_plan(3.5, t0 + 7 * MS)
    core.feed_loc_status(json.dumps({"state": "healthy", "reason": None, "epoch": 2}), t0 + 8 * MS)
    core.feed_loc_pose({"x": 1.0, "y": 2.0, "yaw": 0.5, "cov_xy_trace": 0.01, "stamp_ros_s": 12345.678},
                       t0 + 9 * MS)
    core.feed_rolling_status(json.dumps({"fresh": True, "epoch": 1}), t0 + 10 * MS)
    core.feed_map("global", {"rev": 1, "width": 4, "height": 4, "resolution_m": 0.05, "origin": [0, 0, 0],
                             "frame_id": "map"}, bytes(16), t0 + 11 * MS)
    core.feed_odom(0.19, 0.09, 12345.0, t0 + 12 * MS)
    core.feed_odom_adapter(json.dumps({"state": "ok"}), t0 + 13 * MS)
    core.feed_telemetry(KvTelemetryClient(FakeKv()).poll(), t0 + 14 * MS)
    core.feed_clock({"kv_map": {"offset_ns_at_anchor": 5000, "drift_ppm": 1.5, "consistent": True,
                                "anchor_rtt_half_ns": 70000}, "pl_map": {"error": "no pl"}, "counts": {}},
                    t0 + 15 * MS)
    core.feed_tegrastats("RAM 2000/7620MB CPU [10%@1510,off,30%@1510] GR3D_FREQ 45% cpu@50.5C tj@52.25C "
                         "VDD_IN 7123mW/7000mW", t0 + 16 * MS)
    core.feed_nvpmodel("NV Power Mode: MAXN_SUPER\n2\n", t0 + 17 * MS)


def age_mismatches(state, expected_ms):
    return [(g, state[g]["age_ms"], v) for g, v in expected_ms.items() if state[g]["age_ms"] != v]


class PoseAgeFromRosStamp(HubCore):
    """Variant: ages the pose group with the ROS stamp against the Nano clock (forbidden, B.3)."""

    def _pose(self, now):
        g = HubCore._pose(self, now)
        g["age_ms"] = (now / 1e9 - g["stamp_ros_s"]) * 1e3
        return g


class CoreStateTests(unittest.TestCase):
    T0 = 5 * NS
    NOW = 5 * NS + 250 * MS + 123

    def test_state_known_answer_ages_to_the_nanosecond(self):
        core = HubCore("live", hub_boot_id="hb")
        feed_everything(core, self.T0)
        s = core.compose_state(self.NOW)
        d = self.NOW - self.T0
        expected = {"task": (d - 0 * MS) / 1e6, "memory": (d - 2 * MS) / 1e6, "region": (d - 3 * MS) / 1e6,
                    "pose": (d - 8 * MS) / 1e6, "map": (d - 10 * MS) / 1e6, "nav": (d - 5 * MS) / 1e6,
                    "odom": (d - 12 * MS) / 1e6, "clock": (d - 15 * MS) / 1e6,
                    "geometry_nano": (d - 4 * MS) / 1e6, "nano": (d - 16 * MS) / 1e6,
                    "kv": (d - 14 * MS) / 1e6 + 5.0}
        # a group's age is that of its oldest present input; task: status_frame absent -> +1 ms
        expected["task"] = (d - 1 * MS) / 1e6
        self.assertEqual(age_mismatches(s, expected), [])
        self.assertEqual(s["pose"]["stamp_ros_s"], 12345.678)            # passed through, not aged
        self.assertEqual((s["source"], s["nano_clock"]), ("live", fpga_clock.CLOCK_ID))
        self.assertEqual({g: s[g]["src"] for g in fpgac.STATE_GROUPS}, {g: "live" for g in fpgac.STATE_GROUPS})
        self.assertIsNone(fpgac.check_source_flags(K["STATE"], json.loads(fpgac.dumps(s))))
        self.assertEqual(s["task"]["state"], "searching")
        self.assertEqual((s["task"]["processed_frames"], s["task"]["dropped_frames"]), (4, 1))
        self.assertEqual(s["memory"]["by_state"]["historical_memory"], 2)
        self.assertEqual(s["region"]["last"]["verdict_counts"]["occluded"], 1)
        self.assertEqual((s["nav"]["cmd_v"], s["nav"]["path_len_m"], s["nav"]["motion_permit"]), (0.2, 3.5, True))
        self.assertEqual(s["nav"]["cmd_age_ms"], (d - 6 * MS) / 1e6)
        self.assertEqual((s["kv"]["link_ok"], s["kv"]["geometry"]["pl_device_ms"], s["kv"]["geometry"]["clipped"]),
                         (True, 604.0, 35))
        self.assertEqual(s["kv"]["geometry"]["age_ms"], (d - 14 * MS) / 1e6 + 5.0)
        self.assertEqual((s["kv"]["permit"]["avail"], s["kv"]["permit"]["reason"]), (False, "no_uio_node"))
        self.assertEqual((s["clock"]["offset_us"], s["clock"]["bound_us"], s["clock"]["pl_timebase"]),
                         (5.0, 70.0, False))
        self.assertEqual(s["nano"], dict(s["nano"], power_mode="MAXN_SUPER", vdd_in_mw=7123,
                                         cpu_pct=[10, None, 30], gpu_pct=45, temp_c_max=52.25))
        self.assertEqual(s["perf"]["pl_device_ms_med10"], 604.0)

    def test_task_attempts_whitelisted_only_when_present(self):
        """10-25 find-retry: the task row's attempts -> STATE task.attempts = count/positions/max (items dropped);
        a row without attempts leaves the task group's keys exactly as before"""
        core = HubCore("live", hub_boot_id="hb")
        feed_everything(core, self.T0)
        plain = core.compose_state(self.NOW)["task"]
        self.assertNotIn("attempts", plain)
        core.feed_status(status("task", state="recovering", reason="navigation_retry_pending", target_id=None,
                                candidates=[], processed_frames=4, dropped_frames=1, goal="bottle",
                                attempts={"count": 2, "positions": 1, "max": 3,
                                          "items": [{"n": 1, "result": "failed"}, {"n": 2, "result": None}]}),
                         self.T0 + 1 * MS)
        s = core.compose_state(self.NOW)
        self.assertEqual(s["task"]["attempts"], {"count": 2, "positions": 1, "max": 3})
        self.assertEqual(set(s["task"]) - set(plain), {"attempts"})
        self.assertIsNone(fpgac.check_source_flags(K["STATE"], json.loads(fpgac.dumps(s))))
        core.feed_status(status("task", state="searching", reason="r", target_id=None, candidates=[],
                                attempts="junk"), self.T0 + 2 * MS)
        self.assertNotIn("attempts", core.compose_state(self.NOW)["task"])

    def test_positive_control_ros_stamp_age_is_caught(self):
        core = PoseAgeFromRosStamp("live")
        feed_everything(core, self.T0)
        s = core.compose_state(self.NOW)
        self.assertTrue(age_mismatches(s, {"pose": (self.NOW - self.T0 - 8 * MS) / 1e6}))

    def test_no_input_is_unavailable_not_zero(self):
        s = HubCore("live").compose_state(1)
        for g in ("task", "memory", "kv", "pose", "clock"):
            self.assertEqual((s[g]["src"], s[g]["age_ms"], s[g]["reason"]), ("unavailable", None, "no_input"))
        self.assertIsNone(s["kv"]["geometry"])
        self.assertFalse(s["kv"]["link_ok"])

    def test_replay_marks_every_group(self):
        core = HubCore("replay")
        feed_everything(core, self.T0, sensor="simulation")
        s = core.compose_state(self.NOW)
        self.assertEqual(s["source"], "replay")
        self.assertEqual({s[g]["src"] for g in fpgac.STATE_GROUPS}, {"replay"})

    def test_mismatch_is_inconsistent_with_event(self):
        core = HubCore("live")
        feed_everything(core, self.T0, sensor="simulation")
        s = core.compose_state(self.NOW)
        self.assertEqual(s["source"], "inconsistent")
        self.assertEqual(s["task"]["src"], "unknown")
        events = core.pop_events(self.NOW)
        self.assertEqual(events[0]["event"]["event"], "source_mismatch")
        self.assertEqual(events[0]["src"], "unknown")

    def test_before_startup_ros_groups_are_unknown(self):
        core = HubCore("live")
        core.feed_memory(memory_msg(1), 10)
        s = core.compose_state(20)
        self.assertEqual((s["source"], s["memory"]["src"], s["estop"]["src"]), ("live", "unknown", "live"))

    def test_data_source_has_no_default(self):
        with self.assertRaises(ValueError):
            HubCore(None)

    def test_events_rate_limited_and_task_frames_folded(self):
        core = HubCore("live")
        core.feed_status(status("startup", sensor_source="hardware"), 0)
        for i in range(30):
            core.feed_status(status("fault", reason=str(i)), 10 * MS + i * MS)
        core.feed_status(status("frame_processed", frame_id="rgb-9", dropped_frames=2), 50 * MS)
        events = core.pop_events(60 * MS)
        self.assertEqual(len(events), 20)                    # startup + 19 faults
        self.assertEqual(core.counts["events_dropped"], 11)
        self.assertEqual(core.compose_state(60 * MS)["task"]["last_frame_id"], "rgb-9")
        core.feed_status(status("fault", reason="later"), 2 * NS)
        self.assertEqual(len(core.pop_events(2 * NS)), 1)

    def test_kv_link_ok_drops_within_one_second(self):
        core = HubCore("live")
        core.feed_telemetry(KvTelemetryClient(FakeKv()).poll(), 0)
        self.assertTrue(core.compose_state(800 * MS)["kv"]["link_ok"])
        core.feed_telem_timeout(750 * MS)
        self.assertFalse(core.compose_state(801 * MS)["kv"]["link_ok"])

    def test_memory_frames_throttled_and_resent(self):
        core = HubCore("live")
        core.feed_memory(memory_msg(1), 0)
        self.assertIsNotNone(core.memory_body(10 * MS))
        core.feed_memory(memory_msg(2), 20 * MS)
        self.assertIsNone(core.memory_body(100 * MS))                 # < 0.5 s since last
        self.assertEqual(core.memory_body(520 * MS)["revision"], 2)
        self.assertIsNone(core.memory_body(1 * NS))
        self.assertEqual(core.memory_body(5600 * MS)["revision"], 2)   # 5 s full resend
        self.assertIsNotNone(core.memory_body(5601 * MS, force=True))


class Actions(RosActionsLog):
    def topics(self, name):
        return [p for p in self.published if p[0] == name]


class CoreCommandTests(unittest.TestCase):
    def setUp(self):
        self.actions = Actions()
        self.core = HubCore("live", actions=self.actions)
        self.core.feed_status(status("startup", sensor_source="hardware"), 0)
        self.kv = FakeKv()
        self.estop = KvEstopClient(self.kv)
        self.t = [1 * NS]

    def clock(self):
        return self.t[0]

    def cmd(self, op, cid, args=None, controller=True, client="pc1"):
        return self.core.command(client, controller, {"v": 1, "cmd_id": cid, "op": op, "args": args or {}},
                                 self.t[0])

    def test_dedup_publishes_once(self):
        a = self.cmd("find", "pc1:1", {"text": "找瓶子"})
        b = self.cmd("find", "pc1:1", {"text": "找瓶子"})
        self.assertEqual((a["status"], b["status"], b["duplicate"]), ("forwarded", "forwarded", True))
        self.assertEqual(len(self.actions.topics("/fpga/query")), 1)

    def test_positive_control_without_dedup_publishes_twice(self):
        core = HubCore("live", actions=self.actions, dedup=0)
        for _ in range(2):
            core.command("pc1", True, {"v": 1, "cmd_id": "pc1:1", "op": "find", "args": {"text": "x"}}, 0)
        self.assertEqual(len(self.actions.topics("/fpga/query")), 2)

    def test_control_rules_estop_from_anyone_release_only_holder(self):
        self.assertEqual(self.cmd("find", "b:1", {"text": "x"}, controller=False, client="b")["reason"],
                         "not_controller")
        self.assertEqual(self.cmd("cancel", "b:2", controller=False, client="b")["reason"], "not_controller")
        self.assertEqual(self.cmd("estop", "b:3", controller=False, client="b")["status"], "latched_on_nano")
        self.assertTrue(self.core.estop.latched)
        self.core.estop.kv_step(self.estop, self.clock)
        epoch = self.core.estop.epoch
        self.assertEqual(self.cmd("release", "b:4", {"epoch": epoch}, controller=False, client="b")["reason"],
                         "not_controller")
        self.assertTrue(self.core.estop.latched)
        self.assertEqual(self.cmd("release", "a:1", {"epoch": epoch})["status"], "forwarded")
        self.assertEqual(self.cmd("yield_control", "b:5", controller=False, client="b")["reason"], "not_controller")
        self.assertEqual(self.cmd("patrol", "a:2")["reason"], "not_enabled")
        self.assertEqual(self.cmd("cold_start_confirm", "a:3")["reason"], "not_enabled")
        self.assertEqual(self.cmd("dance", "a:4")["reason"], "bad_op")

    def test_estop_kv_silent_then_recovers(self):
        self.kv.offline = True
        ack = self.cmd("estop", "pc1:1")
        self.assertEqual(ack["status"], "latched_on_nano")
        for step in range(5):                                  # 0..0.8 s: resend every 200 ms
            self.core.estop.kv_step(self.estop, self.clock)
            self.core.tick(self.t[0])
            self.t[0] += 200 * MS
        self.assertEqual(self.core.counts["estop_sent:assert"], 5)
        self.assertEqual(self.core.pop_results(), [])
        self.core.tick(self.t[0])                              # 1.0 s after the request
        res = self.core.pop_results()
        self.assertEqual([r[1]["status"] for r in res], ["kv_unconfirmed"])
        self.assertFalse(res[0][1]["final"])
        self.assertEqual(self.cmd("find", "pc1:2", {"text": "x"})["reason"], "estop_latched")
        self.assertEqual(self.cmd("select", "pc1:3", {"candidate_id": "object-0001"})["reason"], "estop_latched")
        self.assertEqual(self.cmd("cancel", "pc1:4")["status"], "forwarded")
        self.kv.offline = False
        self.core.estop.kv_step(self.estop, self.clock)
        res = [r[1] for r in self.core.pop_results() if r[1]["op"] == "estop"]
        self.assertEqual((res[0]["status"], res[0]["kv_result"], res[0]["epoch"], res[0]["pl_applied"]),
                         ("latched", 5, 1, False))                    # today's board: permit block absent
        # the e-stop's own cancel + the explicit cancel (controller 10-17: O2 revoked, an e-stop cancels navigation)
        self.assertEqual(len(self.actions.topics("/fpga/cancel")), 2)
        self.assertEqual(self.kv.executed["assert"], 1)

    def test_release_epoch_rules_and_resume_note(self):
        self.cmd("estop", "pc1:1")
        self.core.estop.kv_step(self.estop, self.clock)
        self.core.pop_results()
        self.assertEqual(self.cmd("release", "pc1:2", {"epoch": 7})["reason"], "epoch_mismatch")
        self.assertEqual(self.cmd("release", "pc1:3", {"epoch": 1})["status"], "forwarded")
        self.t[0] += 300 * MS
        self.core.estop.kv_step(self.estop, self.clock)
        res = self.core.pop_results()[0][1]
        self.assertEqual(res["status"], "released")
        self.assertIn("next accepted geometry", res["resume_requires"][0])
        self.assertFalse(self.core.estop.latched)
        self.assertEqual(self.cmd("release", "pc1:4", {"epoch": 1})["reason"], "not_latched")

    def test_kv_restart_reasserts_and_hub_restart_adopts(self):
        self.cmd("estop", "pc1:1")
        self.core.estop.kv_step(self.estop, self.clock)
        self.core.feed_telemetry(KvTelemetryClient(self.kv).poll(), self.t[0])
        self.kv.restart()                                       # new boot id, latch lost
        self.t[0] += 100 * MS
        self.core.feed_telemetry(KvTelemetryClient(self.kv).poll(), self.t[0])
        self.assertEqual(self.core.counts["estop_reassert"], 1)
        self.core.estop.kv_step(self.estop, self.clock)
        self.assertTrue(self.kv.latched)
        fresh = HubCore("live", actions=Actions())
        fresh.feed_telemetry(KvTelemetryClient(self.kv).poll(), 0)
        self.assertEqual((fresh.estop.latched, fresh.estop.epoch, fresh.counts["estop_adopted"]),
                         (True, self.kv.epoch, 1))

    def test_find_select_cancel_results(self):
        self.core.feed_status(status("task", state="found", reason=None, target_id=None), self.t[0])
        self.cmd("find", "pc1:1", {"text": "bottle"})
        self.core.feed_status(status("task", state="searching", reason=None, target_id=None), self.t[0] + MS)
        self.cmd("find", "pc1:2", {"text": "bottle"})
        self.core.feed_status(status("query_rejected", reason="busy"), self.t[0] + 2 * MS)
        self.cmd("select", "pc1:3", {"candidate_id": "x"})
        self.core.tick(self.t[0] + 3 * NS)
        self.cmd("cancel", "pc1:4")
        self.core.feed_motion_permit(False, self.t[0] + 4 * NS)
        got = {r[1]["cmd_id"]: (r[1]["status"], r[1].get("reason")) for r in self.core.pop_results()}
        self.assertEqual(got, {"pc1:1": ("accepted", None), "pc1:2": ("rejected", "busy"),
                               "pc1:3": ("unknown", None), "pc1:4": ("done", None)})
        self.assertEqual(self.cmd("find", "pc1:5", {"text": "x" * 65})["reason"], "bad_args")


class Hub(object):
    def __init__(self, **kw):
        self.actions = Actions()
        self.core = HubCore("live", hub_boot_id="hub-test", state_hz=kw.pop("state_hz", 10), actions=self.actions,
                            dedup=kw.pop("dedup", 256))
        self.core.feed_status(status("startup", sensor_source="hardware"), fpga_clock.now_ns())
        self.core.feed_memory(memory_msg(1), fpga_clock.now_ns())
        self.server = HubServer(self.core, TOKEN, port=0, host="127.0.0.1", **kw).start()
        self.kv = None

    def with_kv(self, kv):
        self.kv = kv
        self.loops = KvLoops(self.server, KvTelemetryClient(kv), KvEstopClient(kv)).start()
        return self

    def close(self):
        if self.kv is not None:
            self.loops.stop()
        self.server.close()


class Client(object):
    def __init__(self, port, **kw):
        self.frames = []
        self.events = []
        self.lock = threading.Lock()
        self.c = fpgac.FpgacClient(port, TOKEN, hosts=["127.0.0.1"], on_frame=self._on, on_link=self._link,
                                 backoff_s=(0.1, 0.2), **kw).start()
        self.links = []

    def _on(self, f, o):
        with self.lock:
            self.frames.append((time.monotonic(), f.kind, o))
        if f.kind == K["EVENT"]:
            self.events.append(o["event"])

    def _link(self, state, detail):
        self.links.append((time.monotonic(), state))

    def wait(self, predicate, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(0.02)
        return False

    def states(self):
        with self.lock:
            return [(t, o) for t, k, o in self.frames if k == K["STATE"]]

    def stop(self):
        self.c.stop()


class ServerControlTests(unittest.TestCase):
    def test_first_client_controls_second_reads_estop_from_either(self):
        hub = Hub()
        a = Client(hub.server.port)
        try:
            self.assertTrue(a.c.connected.wait(5))
            b = Client(hub.server.port)
            try:
                self.assertTrue(b.c.connected.wait(5))
                self.assertEqual((a.c.hello_ack["controller"], b.c.hello_ack["controller"]), (True, False))
                cid = b.c.command("find", {"text": "bottle"})
                self.assertEqual(b.c.wait_ack(cid, 3)["reason"], "not_controller")
                cid = b.c.command("estop")
                self.assertEqual(b.c.wait_ack(cid, 3)["status"], "latched_on_nano")
                cid = b.c.command("release", {"epoch": 0})
                self.assertEqual(b.c.wait_ack(cid, 3)["reason"], "not_controller")
                cid = a.c.command("yield_control")
                self.assertEqual(a.c.wait_ack(cid, 3)["status"], "yielded")
                self.assertTrue(b.wait(lambda: any(e.get("event") == "controller_changed" and
                                                   e.get("controller_client_id") == b.c.client_id
                                                   for e in b.events)))
                cid = a.c.command("cancel")
                self.assertEqual(a.c.wait_ack(cid, 3)["reason"], "not_controller")
                cid = b.c.command("cancel")
                self.assertEqual(b.c.wait_ack(cid, 3)["status"], "forwarded")
                self.assertEqual(hub.actions.topics("/fpga/query"), [])
            finally:
                b.stop()
        finally:
            a.stop()
            hub.close()

    def test_disconnect_hands_control_over_and_keeps_estop_latched(self):
        hub = Hub()
        a, b = Client(hub.server.port), None
        try:
            self.assertTrue(a.c.connected.wait(5))
            b = Client(hub.server.port)
            self.assertTrue(b.c.connected.wait(5))
            cid = a.c.command("estop")
            self.assertTrue(a.c.wait_ack(cid, 3))
            a.stop()
            self.assertTrue(b.wait(lambda: any(e.get("event") == "controller_changed" for e in b.events)))
            cid = b.c.command("cancel")
            self.assertEqual(b.c.wait_ack(cid, 3)["status"], "forwarded")
            time.sleep(0.3)
            self.assertTrue(hub.core.estop.latched)          # D1: disconnect never releases
            self.assertTrue(b.states()[-1][1]["estop"]["latched"])
        finally:
            if b:
                b.stop()
            hub.close()

    def test_positive_control_both_clients_command_is_caught(self):
        class Everyone(HubServer):
            def _command(self, s, body):
                now = self.clock()
                with self.lock:
                    ack = self.core.command(s.client_id, True, body, now)
                s.offer_ack(K["CMD_ACK"], ack)

        actions = Actions()
        core = HubCore("live", actions=actions)
        server = Everyone(core, TOKEN, port=0, host="127.0.0.1").start()
        a, b = Client(server.port), Client(server.port)
        try:
            self.assertTrue(a.c.connected.wait(5) and b.c.connected.wait(5))
            cid = b.c.command("find", {"text": "bottle"})
            self.assertNotEqual(b.c.wait_ack(cid, 3).get("reason"), "not_controller")   # the bug shows
        finally:
            a.stop()
            b.stop()
            server.close()

    def test_wrong_token_and_kv_net_denied(self):
        hub = Hub(deny_nets=("127.0.0.0/8",))
        c = Client(hub.server.port)
        try:
            time.sleep(0.8)
            self.assertFalse(c.c.connected.is_set())
            self.assertGreaterEqual(hub.server.counts["denied_local_net"], 1)
        finally:
            c.stop()
            hub.close()
        hub = Hub(deny_nets=("127.0.0.0/8",), check_local=False)            # variant: no local check
        c = Client(hub.server.port)
        try:
            self.assertTrue(c.c.connected.wait(5))
        finally:
            c.stop()
            hub.close()
        hub = Hub()
        bad = fpgac.FpgacClient(hub.server.port, "not-the-token-at-all", hosts=["127.0.0.1"], backoff_s=(0.2,)).start()
        try:
            time.sleep(0.6)
            self.assertFalse(bad.connected.is_set())
            self.assertGreaterEqual(hub.server.counts["hello_rejected:bad_token"], 1)
        finally:
            bad.stop()
            hub.close()


class ServerLinkTests(unittest.TestCase):
    def test_reconnect_gets_memory_and_state_immediately(self):
        hub = Hub(state_hz=0.5)                       # ticks 2 s apart: the snapshot must not wait
        relay = Relay(hub.server.port)
        c = Client(relay.port)
        try:
            self.assertTrue(c.c.connected.wait(5))
            relay.kill()
            self.assertTrue(c.wait(lambda: c.c.counts["connected"] >= 2, 5))
            t_up = [t for t, st in c.links if st == "up"][-1]
            first = lambda kind: [t for t, k, o in c.frames if t >= t_up and k == kind]  # noqa: E731
            self.assertTrue(c.wait(lambda: first(K["MEMORY"]) and first(K["STATE"]), 1.0))
            self.assertLess(first(K["MEMORY"])[0] - t_up, 0.2)
            self.assertLess(first(K["STATE"])[0] - t_up, 0.2)
        finally:
            c.stop()
            relay.close()
            hub.close()

    def _dedup_across_reconnect(self, dedup):
        hub = Hub(dedup=dedup)
        relay = Relay(hub.server.port)
        c = Client(relay.port)
        try:
            self.assertTrue(c.c.connected.wait(5))
            relay.pause()                            # the ACK cannot come back
            cid = c.c.command("find", {"text": "bottle"})
            self.assertTrue(c.wait(lambda: len(hub.actions.topics("/fpga/query")) == 1, 3))
            relay.kill()
            ack = c.c.wait_ack(cid, 5)
            self.assertIsNotNone(ack)
            return len(hub.actions.topics("/fpga/query")), ack
        finally:
            c.stop()
            relay.close()
            hub.close()

    def test_resend_after_reconnect_publishes_once(self):
        n, ack = self._dedup_across_reconnect(256)
        self.assertEqual((n, ack.get("duplicate")), (1, True))

    def test_positive_control_no_dedup_publishes_twice(self):
        n, ack = self._dedup_across_reconnect(0)
        self.assertEqual(n, 2)

    def test_stall_3s_marks_stale_without_reconnect_6s_reconnects(self):
        hub = Hub()
        relay = Relay(hub.server.port)
        c = Client(relay.port)
        try:
            self.assertTrue(c.c.connected.wait(5))
            time.sleep(0.5)
            relay.pause()
            time.sleep(3.0)
            relay.resume()
            time.sleep(1.0)
            self.assertEqual((c.c.counts["link_stale"], c.c.counts["connected"], c.c.counts["dead_reconnect"]),
                             (1, 1, 0))
            relay.pause()
            time.sleep(6.0)
            relay.resume()
            self.assertTrue(c.wait(lambda: c.c.counts["connected"] >= 2, 5))
            self.assertEqual(c.c.counts["dead_reconnect"], 1)
        finally:
            c.stop()
            relay.close()
            hub.close()

    # ---- back-pressure gate (controller ruling 10-11, replaces spec E.2 "first frame < 200 ms") --
    # Measured quantity: frame age at arrival = arrival - nano_mono_ns, both on this host's
    # fpga_clock (valid on loopback only). Gate: after a 3 s stall, the first frame younger than
    # 200 ms arrives within 500 ms of resume, and at most 3 frames >= 200 ms old arrive.
    GATE_OLD_MS, GATE_FIRST_MS, GATE_MAX_OLD = 200.0, 500.0, 3

    def _stall_arrivals(self, latest_only, sndbuf, hz=10, pause_s=3.0):
        hub = Hub(state_hz=hz, latest_only=latest_only, sndbuf=sndbuf)
        relay = Relay(hub.server.port, rcvbuf=4096)
        rows, hello = [], {}

        def on(f, o):
            if f.kind == K["STATE"]:
                rows.append((fpga_clock.now_ns(), o["nano_mono_ns"], f.stale, f.lag_ms))

        c = fpgac.FpgacClient(relay.port, TOKEN, hosts=["127.0.0.1"], on_frame=on, backoff_s=(0.1, 0.2)).start()
        try:
            self.assertTrue(c.connected.wait(5))
            hello.update(c.hello_ack)
            time.sleep(1.0)
            relay.pause()
            time.sleep(pause_s)
            resume_ns = fpga_clock.now_ns()
            relay.resume()
            time.sleep(1.5)
            superseded = sum(x.superseded["STATE"] for x in hub.server.sessions)
            return dict(rows=list(rows), resume_ns=resume_ns, superseded=superseded,
                        sndbuf=hub.server.effective_sndbuf, hello=hello, counts=dict(c.counts))
        finally:
            c.stop()
            relay.close()
            hub.close()

    def gate(self, run):
        after = [(a, n) for a, n, _, _ in run["rows"] if a >= run["resume_ns"]]
        ages = [(a - n) / 1e6 for a, n in after]
        old = [x for x in ages if x >= self.GATE_OLD_MS]
        first = next(((a - run["resume_ns"]) / 1e6 for a, n in after if (a - n) / 1e6 < self.GATE_OLD_MS), None)
        ok = first is not None and first <= self.GATE_FIRST_MS and len(old) <= self.GATE_MAX_OLD
        return {"pass": ok, "n_old": len(old), "first_fresh_ms": first, "old_ages_ms": [round(x) for x in old],
                "n_after": len(after), "sndbuf": run["sndbuf"], "superseded": run["superseded"]}

    def _report(self, name, g):
        sys.stderr.write("\n[backpressure] %s %s\n" % (name, json.dumps(g)))

    def test_backpressure_gate_min_sndbuf_latest_only(self):
        run = self._stall_arrivals(latest_only=True, sndbuf=1)
        g = self.gate(run)
        self._report("min-sndbuf+latest-only 10Hz", g)
        self.assertTrue(g["pass"], g)
        self.assertEqual(run["hello"]["effective_sndbuf"], run["sndbuf"])
        self.assertLessEqual(run["sndbuf"], 8192)               # kernel floor, not the 64 KiB of the spec
        self.assertGreater(run["superseded"], 10)
        # (c) the PC library flags frames that waited in the link, without the Nano clock
        for a, n, stale, lag in run["rows"]:
            age = (a - n) / 1e6
            if age >= self.GATE_OLD_MS + 30:
                self.assertTrue(stale, (age, lag))
            elif age < self.GATE_OLD_MS - 50:
                self.assertFalse(stale, (age, lag))
        self.assertEqual(run["counts"].get("stale_frames:STATE", 0), g["n_old"])

    def test_positive_control_queue_all_default_sndbuf_fails_gate(self):
        g = self.gate(self._stall_arrivals(latest_only=False, sndbuf=None))
        self._report("queue-all+default-sndbuf 10Hz", g)
        self.assertFalse(g["pass"], g)
        self.assertGreater(g["n_old"], 20)

    def test_positive_control_64k_sndbuf_latest_only_fails_gate(self):
        g = self.gate(self._stall_arrivals(latest_only=True, sndbuf=32 * 1024))   # 64 KiB effective
        self._report("64KiB-sndbuf+latest-only 10Hz", g)
        self.assertFalse(g["pass"], g)
        self.assertEqual(g["sndbuf"], 64 * 1024)

    def test_backpressure_stress_50hz_reported(self):
        g = self.gate(self._stall_arrivals(latest_only=True, sndbuf=1, hz=50))
        self._report("min-sndbuf+latest-only 50Hz (stress, reported)", g)
        self.assertIsNotNone(g["first_fresh_ms"])

    def test_acks_survive_a_slow_reader(self):
        hub = Hub(state_hz=50)
        relay = Relay(hub.server.port)
        c = Client(relay.port)
        try:
            self.assertTrue(c.c.connected.wait(5))
            relay.pause()
            ids = [c.c.command("cancel") for _ in range(20)]
            time.sleep(1.0)
            relay.resume()
            self.assertTrue(c.wait(lambda: all(i in c.c.acks for i in ids), 5))
            self.assertEqual(len(hub.actions.topics("/fpga/cancel")), 20)
        finally:
            c.stop()
            relay.close()
            hub.close()

    def test_jitter_300_200ms_no_false_reconnect(self):
        import random
        rng = random.Random(7)
        hub = Hub()
        relay = Relay(hub.server.port)
        relay.delay_fn = lambda: 0.3 + rng.uniform(-0.2, 0.2)
        c = Client(relay.port)
        try:
            self.assertTrue(c.c.connected.wait(5))
            time.sleep(8.0)
            self.assertEqual((c.c.counts["connected"], c.c.counts["dead_reconnect"], c.c.counts["link_stale"]),
                             (1, 0, 0))
            self.assertGreater(len(c.states()), 60)
        finally:
            c.stop()
            relay.close()
            hub.close()


class SmokeTests(unittest.TestCase):
    def test_fake_kv_smoke_10hz_and_link_loss_within_1s(self):
        kv = FakeKv(sleep_on_timeout=True)
        hub = Hub().with_kv(kv)
        c = Client(hub.server.port)
        try:
            self.assertTrue(c.c.connected.wait(5))
            time.sleep(3.0)
            states = [o for t, o in c.states()]
            last = states[-1]
            geo = last["kv"]["geometry"]
            self.assertTrue(geo["avail"] and geo["pl_device_ms"] == 604.0 and geo["core"] == "null")
            ages = [o[g]["age_ms"] for o in states[-10:] for g in ("kv", "memory")]
            self.assertTrue(all(a is not None and 0 <= a < 1e9 for a in ages), ages)
            # the hub fixture feeds only a startup event, no `task` event: task stays unavailable
            self.assertEqual({(o["task"]["src"], o["task"]["age_ms"]) for o in states[-10:]},
                             {("unavailable", None)})
            window = [t for t, o in c.states() if t >= c.states()[-1][0] - 2.0]
            rate = (len(window) - 1) / (window[-1] - window[0])
            self.assertTrue(9.0 <= rate <= 11.0, rate)
            kv.offline = True
            t_off = time.monotonic()
            self.assertTrue(c.wait(lambda: c.states()[-1][1]["kv"]["link_ok"] is False, 2.0))
            first_false = next(t for t, o in c.states() if t >= t_off and o["kv"]["link_ok"] is False)
            self.assertLessEqual(first_false - t_off, 1.0)
        finally:
            c.stop()
            hub.close()

    def test_estop_end_to_end_over_tcp_with_fake_kv(self):
        kv = FakeKv(permit_avail=True)
        hub = Hub().with_kv(kv)
        c = Client(hub.server.port)
        try:
            self.assertTrue(c.c.connected.wait(5))
            cid = c.c.command("estop")
            self.assertEqual(c.c.wait_ack(cid, 3)["status"], "latched_on_nano")
            res = c.c.wait_result(cid, 3)
            self.assertEqual((res["status"], res["kv_result"], res["pl_applied"]), ("latched", 0, True))
            self.assertEqual(kv.ctrl_writes, 1)
            cid = c.c.command("release", {"epoch": res["epoch"]})
            self.assertEqual(c.c.wait_result(cid, 3)["status"], "released")
            self.assertFalse(kv.force_stop)
        finally:
            c.stop()
            hub.close()


class FakeKvContractTests(unittest.TestCase):
    """The fake must accept every call the real package-1 clients accept (da4e4b3 additions)."""

    def test_fake_signatures_superset_of_real(self):
        import inspect
        from host import kv_link
        from host.hub import fake_kv
        pairs = [(kv_link.KvTelemetryClient.__init__, fake_kv.KvTelemetryClient.__init__),
                 (kv_link.KvTelemetryClient.poll, fake_kv.KvTelemetryClient.poll),
                 (kv_link.KvEstopClient.__init__, fake_kv.KvEstopClient.__init__),
                 (kv_link.KvEstopClient.send, fake_kv.KvEstopClient.send),
                 (kv_link._ExtClient.__init__, fake_kv._FakeClient.__init__)]
        missing = []
        for real, fake in pairs:
            fp = inspect.signature(fake).parameters
            for name, prm in inspect.signature(real).parameters.items():
                if prm.kind in (prm.VAR_KEYWORD, prm.VAR_POSITIONAL):
                    continue
                if name == "feature" and real is kv_link._ExtClient.__init__:   # fixed by subclass
                    continue
                if name not in fp:
                    missing.append((real.__qualname__, name))
                elif prm.default is not prm.empty and fp[name].default != prm.default:
                    missing.append((real.__qualname__, name, "default"))
        self.assertEqual(missing, [])
        self.assertTrue(hasattr(fake_kv.KvTelemetryClient, "close"))

    def test_fake_telemetry_is_the_real_type_and_units(self):
        from host import kv_status, kv_link
        tel = KvTelemetryClient(FakeKv(permit_avail=True)).poll()
        self.assertIs(type(tel), kv_status.Telemetry)
        self.assertEqual(tel.blocks["geometry"]["last"]["times_ms"]["core_device"], 604.0)
        self.assertAlmostEqual(tel.age_upper_ms["geometry"], 5.0)
        self.assertAlmostEqual(tel.age_upper_ms["permit"], 2.0)
        self.assertAlmostEqual(tel.blocks["permit"]["age_ms"], 12.0)
        names = KvTelemetryClient(FakeKv()).poll(want=kv_link.blocks_mask(["geometry"]))
        self.assertEqual(set(names.blocks), {"geometry"})

    def test_fake_estop_query_and_epoch_rules_match_pkg1(self):
        kv = FakeKv()
        c = KvEstopClient(kv)
        self.assertEqual(c.send("query", 1).result, 3)
        self.assertEqual(c.send("assert", 2, epoch=5).result, 4)
        self.assertEqual(c.send("assert", 3).result, 5)
        self.assertEqual(c.send("query", 4).result, 5)
        self.assertEqual(c.send("query", 5, epoch=1).result, 4)


class FakeMsg(types.SimpleNamespace):
    pass


class RosAdapterTests(unittest.TestCase):
    def test_every_callback_feeds_core_and_no_image_topics(self):
        from host.hub import hub_ros
        subs, pubs = {}, {}
        node_kw, sub_kw, pub_kw = {}, {}, {}

        class Node(object):
            def __init__(self, name, **kw):
                node_kw.update(kw)

            def create_subscription(self, kind, topic, cb, qos, **kw):
                subs[topic] = cb
                sub_kw[topic] = kw

            def create_publisher(self, kind, topic, qos, **kw):
                pub_kw[topic] = kw
                pubs[topic] = []
                return types.SimpleNamespace(publish=pubs[topic].append)

        m = types.SimpleNamespace(Node=Node, QoSProfile=lambda **kw: kw,
                                  DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1),
                                  ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2),
                                  String=lambda data="": FakeMsg(data=data), Empty=lambda: FakeMsg(),
                                  Bool=object, Twist=object, PoseWithCovarianceStamped=object,
                                  OccupancyGrid=object, Odometry=object, Path=object, CompressedImage=object,
                                  SubscriptionEventCallbacks=lambda **kw: ("sub_ev", kw),
                                  PublisherEventCallbacks=lambda **kw: ("pub_ev", kw))
        core = HubCore("live")
        server = types.SimpleNamespace(core=core, lock=threading.RLock())
        hub_ros.build_node(server, m, preview=True, clock=lambda: 7 * NS)
        self.assertEqual(set(subs), set(hub_ros.SUBSCRIPTIONS))
        self.assertNotIn("/fpga/rgb", subs)
        # no parameter services, no QoS default event handlers (lean executor serves subs only)
        self.assertEqual(node_kw, {"start_parameter_services": False})
        self.assertEqual(set(sub_kw), set(hub_ros.SUBSCRIPTIONS))
        self.assertTrue(all(kw == {"event_callbacks": ("sub_ev", {"use_default_callbacks": False})}
                            for kw in sub_kw.values()))
        self.assertEqual(set(pub_kw), set(hub_ros.PUBLICATIONS))
        self.assertTrue(all(kw == {"event_callbacks": ("pub_ev", {"use_default_callbacks": False})}
                            for kw in pub_kw.values()))
        self.assertNotIn("/fpga/depth", subs)
        P = lambda **kw: FakeMsg(**kw)  # noqa: E731
        stamp = P(sec=12, nanosec=500000000)
        q = P(x=0.0, y=0.0, z=0.0, w=1.0)
        grid = P(header=P(frame_id="map"), info=P(width=2, height=2, resolution=0.05,
                                                  origin=P(position=P(x=1.0, y=2.0), orientation=q)),
                 data=[0, 100, -1, 50])
        subs["/fpga/status"](P(data=status("startup", sensor_source="hardware")))
        subs["/fpga/memory"](P(data=memory_msg(1)))
        subs["/cmd_vel"](P(linear=P(x=0.3), angular=P(z=-0.2)))
        subs["/fpga/localization_pose"](P(header=P(stamp=stamp), pose=P(pose=P(position=P(x=1.0, y=2.0),
                                                                                orientation=q),
                                                                         covariance=[0.1] + [0] * 6 + [0.2] + [0] * 28)))
        subs["/map"](grid)
        subs["/fpga/rolling_map/grid"](grid)
        subs["/odom"](P(header=P(stamp=stamp), twist=P(twist=P(linear=P(x=0.25), angular=P(z=0.05)))))
        subs["/plan"](P(header=P(frame_id="map", stamp=stamp),
                        poses=[P(pose=P(position=P(x=0.0, y=0.0))), P(pose=P(position=P(x=3.0, y=4.0)))]))
        subs["/fpga/motion_permit"](P(data=False))
        s = core.compose_state(8 * NS)
        self.assertEqual((s["nav"]["cmd_v"], s["nav"]["cmd_w"], s["nav"]["path_len_m"]), (0.3, -0.2, 5.0))
        self.assertEqual((s["nav"]["path"]["points"], s["nav"]["path"]["frame_id"], s["nav"]["path_reason"]),
                         ([[0.0, 0.0], [3.0, 4.0]], "map", None))
        self.assertEqual((s["pose"]["x"], s["pose"]["stamp_ros_s"], s["pose"]["age_ms"]), (1.0, 12.5, 1000.0))
        self.assertAlmostEqual(s["pose"]["cov_xy_trace"], 0.3)
        self.assertEqual((s["map"]["global_rev"], s["odom"]["v"]), (1, 0.25))
        maps = core.map_frames(8 * NS, force=True)
        self.assertEqual([m_[0]["which"] for m_ in maps], ["global", "rolling"])
        import zlib
        self.assertEqual(zlib.decompress(maps[0][1]), bytes([0, 100, 255, 50]))
        core.actions.publish_query("找瓶子")
        core.actions.publish_cancel()
        self.assertEqual(pubs["/fpga/query"][0].data, "找瓶子")
        self.assertEqual(len(pubs["/fpga/cancel"]), 1)


class DepthInvalidRun(unittest.TestCase):
    """10-25: geometry_nano.depth_invalid_s = seconds since the first frame of the newest unbroken run of producer
    frames dropped for no_valid_depth; any other result ends the run; None without such a run."""

    def feed(self, core, result, t):
        core.feed_frame_stats(json.dumps({"result": result, "client_counts": {}}), t)

    def test_run_length(self):
        core = HubCore("replay")
        t0 = 10 ** 12
        self.assertIsNone(core.compose_state(t0)["geometry_nano"]["depth_invalid_s"])
        self.feed(core, "fused", t0)
        self.assertIsNone(core.compose_state(t0)["geometry_nano"]["depth_invalid_s"])
        for k in range(5):
            self.feed(core, "dropped:no_valid_depth", t0 + (1 + k) * 10 ** 9)
        self.assertEqual(core.compose_state(t0 + 5 * 10 ** 9 + 500 * MS)["geometry_nano"]["depth_invalid_s"], 4.5)
        self.feed(core, "dropped:pose_outside_capture_window", t0 + 6 * 10 ** 9)     # another reason ends the run
        self.assertIsNone(core.compose_state(t0 + 6 * 10 ** 9)["geometry_nano"]["depth_invalid_s"])
        self.feed(core, "dropped:no_valid_depth", t0 + 7 * 10 ** 9)                # a new run starts from here
        self.assertEqual(core.compose_state(t0 + 9 * 10 ** 9)["geometry_nano"]["depth_invalid_s"], 2.0)


if __name__ == "__main__":
    unittest.main()
