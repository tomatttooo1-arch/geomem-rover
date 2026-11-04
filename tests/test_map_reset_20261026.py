"""10-26「重置地图」(owner): console button -> bridge map_reset {confirm} -> hub op map_reset -> /fpga/map_reset ->
runner (stop, new memory scope, caches, /fpga/mapping/reset -> mapping_ctl, costmaps, heat map none) -> status ->
hub CMD_RESULT -> bridge result; the 3-D feed (live_mapping.py) follows the scope and clears its voxels.

Executed here (host, no ROS): host/map_reset.py, the runner's Runtime on the offline fixture
(tests/test_runner_tf_retry.load_runtime), hub_core, gmb_proto + Bridge, mapping_ctl.MappingCore / new_run_id /
respawn_args, the mapping stand-in as processes (map -> reset by respawn -> map again -> stop/export), live_mapping
MappingFollow + VoxelFeed, search_belief_feed.BeliefFeed.clear. Not executed: rtabmap /rtabmap/reset, map_assembler
reset, Nav2 load_map / ClearEntireCostmap, bench_pose (car checks; ui/geomem/vehicle_interface_protocol.md「重置地图」).
"""
import importlib.util
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
for p in (str(ROOT), str(ROOT / "tests"), str(ROOT / "tools" / "car_run"), str(ROOT / "ui" / "geomem" / "bridge")):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402

from host import map_reset  # noqa: E402
from host.hub.hub_core import HubCore, parse_belief  # noqa: E402
from host.hub.hub_server import RosActionsLog  # noqa: E402
from host.search_belief_feed import BeliefFeed, dumps as belief_dumps  # noqa: E402
import mapping_ctl  # noqa: E402
import gmb_proto as gp  # noqa: E402
import geomem_bridge as gb  # noqa: E402

spec = importlib.util.spec_from_file_location("live_mapping_mr", ROOT / "ui/geomem/tools/competition/live_mapping.py")
live_mapping = importlib.util.module_from_spec(spec)
spec.loader.exec_module(live_mapping)

NS_ = 1000000000


# ------------------------------------------------------------------------------------------ host/map_reset.py
class PureTests(unittest.TestCase):
    def test_topics_match_mapping_ctl(self):
        self.assertEqual(map_reset.MAPPING_RESET_TOPIC, mapping_ctl.RESET_TOPIC)
        self.assertEqual(map_reset.STEPS, ("stop", "memory", "caches", "mapping", "costmaps", "display"))

    def test_parse_and_refusal(self):
        self.assertEqual(map_reset.parse_request('{"id": "c1"}')["id"], "c1")
        for bad in ('[]', '{"id": ""}', '{"id": 3}', '{}'):
            with self.assertRaises(ValueError):
                map_reset.parse_request(bad)
        row = {"state": "mapping", "session": 0}
        self.assertIsNone(map_reset.refusal(row, 0.5, False, False))
        self.assertEqual(map_reset.refusal(row, 0.5, True, False), "busy")
        self.assertEqual(map_reset.refusal(row, 0.5, False, True), "estimated_pose")    # localization mode
        self.assertEqual(map_reset.refusal(None, None, False, False), "no_mapping")
        self.assertEqual(map_reset.refusal(row, 9.0, False, False), "no_mapping")       # stale status
        self.assertEqual(map_reset.refusal({"state": "saving"}, 0.1, False, False), "mapping_saving")
        self.assertEqual(map_reset.refusal({"state": "exported", "map_loaded": None}, 0.1, False, False),
                         "mapping_saving")
        self.assertIsNone(map_reset.refusal({"state": "exported", "map_loaded": True}, 0.1, False, False))
        self.assertEqual(map_reset.refusal({"state": "resetting"}, 0.1, False, False), "busy")

    def test_job_and_mapping_answer(self):
        t = [10.0]
        job = map_reset.MapResetJob("c1", clock=lambda: t[0], wall=lambda: 5.0)
        self.assertTrue(job.holding_memory)
        job.step("stop", True)
        t[0] = 10.5
        job.step("mapping", False, "建图进程 30 秒内没有回应")
        self.assertFalse(job.holding_memory)
        self.assertEqual(job.finish(), "failed")
        self.assertIn("重置建图没有完成", job.reason_zh())
        self.assertEqual(job.align_key, "reset:c1")
        self.assertEqual(map_reset.mapping_answer({"state": "resetting", "reset": {"id": "c1"}}, "c1")[0], False)
        self.assertEqual(map_reset.mapping_answer({"state": "mapping", "session": 1, "reset": {"id": "c1", "ok": True}},
                                                  "c1"), (True, True, None, 1))
        self.assertEqual(map_reset.mapping_answer({"state": "mapping", "reset": {"id": "old", "ok": True}}, "c1")[0],
                         False)


# ------------------------------------------------------------------------------------------ the runner
class RunnerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import test_runner_tf_retry as tr
        cls.tr = tr
        cls.source = tr.current_source()

    def setUp(self):
        self.node, self.world = self.tr.load_runtime(self.source, "mr_runner_%s" % self._testMethodName)
        self.world.events.clear()

    def pub(self, topic):
        v = self.world.publisher_values.get(topic)
        return json.loads(v["data"]) if isinstance(v, dict) and "data" in v else v

    def events(self, name):
        return [e["payload"] for e in self.world.events
                if e.get("topic") == "/fpga/status" and e["payload"].get("event") == name]

    def mapping(self, **row):
        self.node.on_mapping_status(NS(data=json.dumps(dict({"state": "mapping", "session": 0}, **row))))

    def test_reset_stops_clears_memory_new_scope_caches_and_asks_mapping_ctl(self):
        n = self.node
        n.memory.tracks = [object(), object()]                # two remembered objects
        n.observation_views = {"o1": [1]}
        n.search_planner_cache = {"k": 1}
        n.mapping_waypoints, n.mapping_nodes = ["wp"], ["node"]
        self.mapping()
        old_scope, old_memory = n.runtime_boot_id, n.memory
        n.on_map_reset(NS(data=json.dumps({"id": "c1"})))
        # stop: zero velocity, permit false
        self.assertEqual(self.world.publisher_values["/fpga/motion_permit"], False)
        self.assertEqual(self.world.publisher_values["/cmd_vel"]["linear"]["x"], 0.0)
        # memory: empty, new scope, republished on /fpga/runtime_info
        self.assertIsNot(n.memory, old_memory)
        self.assertEqual(len(n.memory.tracks), 0)
        self.assertNotEqual(n.runtime_boot_id, old_scope)
        self.assertEqual(self.pub("/fpga/runtime_info")["runtime_boot_id"], n.runtime_boot_id)
        self.assertEqual(self.pub("/fpga/runtime_info")["map_reset"]["previous_runtime_boot_id"], old_scope)
        # caches
        self.assertEqual((n.observation_views, n.search_planner_cache, n.mapping_waypoints, n.mapping_nodes),
                         ({}, {}, None, None))
        self.assertIn("reset:c1", n.memory_align_done)
        # mapping_ctl asked; status running; memory held
        self.assertEqual(self.pub("/fpga/mapping/reset"), {"align_key": "reset:c1", "id": "c1"})
        self.assertEqual(self.pub("/fpga/map_reset/status")["state"], "running")
        self.assertTrue(n.map_reset_holds(NS(timestamp=1e9)))
        # find / goto refused while it runs (the map-switch gate)
        n.begin_patrol(None)
        self.assertTrue(self.events("patrol_rejected"))
        self.assertIn("map", json.dumps(self.events("patrol_rejected")[-1]).lower())
        # an old-session row (no reset answer) does not finish it, does not refill the goals
        self.mapping(state="resetting", search_waypoints={"ok": True, "waypoints": [[1, 2, 0]]})
        n.check_map_reset()
        self.assertIsNotNone(n.map_reset_job)
        self.assertIsNone(n.mapping_waypoints)
        # mapping_ctl answers: done, costmaps + heat map none, memory open again (newer frames only)
        self.mapping(session=1, reset={"id": "c1", "ok": True, "method": "rtabmap_reset"})
        n.check_map_reset()
        self.assertIsNone(n.map_reset_job)
        st = self.pub("/fpga/map_reset/status")
        self.assertEqual(st["state"], "done", st)
        self.assertEqual([s["step"] for s in st["steps"]], list(map_reset.STEPS))
        self.assertEqual((st["scope"], st["previous_scope"], st["mapping_session"]), (n.runtime_boot_id, old_scope, 1))
        belief = self.pub("/fpga/search/belief")
        self.assertEqual((belief["phase"], belief["width"], belief["height"]), ("none", 0, 0))
        meta, cells = parse_belief(json.dumps(belief))          # the hub takes it (MAP which=belief)
        self.assertEqual((meta["width"], cells), (0, b""))
        until = n.map_reset_hold_until
        self.assertTrue(n.map_reset_holds(NS(timestamp=until - 0.1)))      # acquired before the end
        self.assertFalse(n.map_reset_holds(NS(timestamp=until + 0.1)))
        self.assertEqual(len(self.events("map_reset_done")), 1)
        self.assertEqual(len(self.events("map_reset_step")), 6)

    def test_localization_mode_and_no_mapping_refused_nothing_touched(self):
        n = self.node
        scope = n.runtime_boot_id
        n.on_map_reset(NS(data=json.dumps({"id": "c2"})))                 # no mapping status at all
        st = self.pub("/fpga/map_reset/status")
        self.assertEqual((st["state"], st["refused"]), ("failed", "no_mapping"))
        self.assertIn("定位模式", st["reason_zh"])
        self.mapping()
        n.localization = object()                                          # estimated pose = a stored map
        n.on_map_reset(NS(data=json.dumps({"id": "c3"})))
        self.assertEqual(self.pub("/fpga/map_reset/status")["refused"], "estimated_pose")
        self.assertEqual(n.runtime_boot_id, scope)
        self.assertIsNone(n.map_reset_job)
        self.assertIsNone(self.world.publisher_values.get("/fpga/mapping/reset"))

    def test_mapping_ctl_silent_times_out_failed_with_reason(self):
        n = self.node
        self.mapping()
        n.on_map_reset(NS(data=json.dumps({"id": "c4"})))
        n.map_reset_job.mapping_sent_at -= map_reset.MAPPING_WAIT_S + 1
        n.check_map_reset()
        st = self.pub("/fpga/map_reset/status")
        self.assertEqual(st["state"], "failed")
        self.assertIn("没有回应", st["reason_zh"])
        self.assertEqual(self.pub("/fpga/search/belief")["phase"], "none")  # the later steps still ran

    def test_running_task_cancelled_on_reset(self):
        n = self.node
        self.mapping()
        calls = []
        n.active = lambda: True
        n.task = NS(state="navigating", cancel=lambda now: calls.append(now) or [], reason=None, target_id=None,
                    candidates=[], search_planner=None, TERMINAL=())
        n.execute = lambda actions: calls.append(("execute", actions))
        n.publish_task = lambda: None             # the stand-in task has no task-row fields
        n.on_map_reset(NS(data=json.dumps({"id": "c5"})))
        self.assertTrue(calls, "task.cancel not called")
        self.assertEqual(self.world.publisher_values["/fpga/motion_permit"], False)
        self.assertEqual(n.map_reset_job.steps[0]["detail"]["task_state"], "navigating")


# ------------------------------------------------------------------------------------------ hub
class Actions(RosActionsLog):
    def topics(self, name):
        return [p for p in self.published if p[0] == name]


class HubTests(unittest.TestCase):
    def setUp(self):
        self.actions = Actions()
        self.core = HubCore("live", actions=self.actions)
        self.t = 1 * NS_

    def cmd(self, cid, args=None):
        return self.core.command("pc1", True, {"v": 1, "cmd_id": cid, "op": "map_reset", "args": args or {}}, self.t)

    def test_refusals(self):
        self.assertEqual(self.cmd("a")["reason"], "map_reset_no_mapping")          # localization mode
        self.core.feed_mapping_status(json.dumps({"state": "saving"}), self.t)
        self.assertEqual(self.cmd("b")["reason"], "mapping_saving")
        self.core.feed_mapping_status(json.dumps({"state": "resetting"}), self.t)
        self.assertEqual(self.cmd("c")["reason"], "map_reset_busy")
        self.core.feed_mapping_status(json.dumps({"state": "mapping", "session": 0}), self.t)
        self.assertEqual(self.cmd("d", {"x": 1})["reason"], "bad_args")
        self.assertEqual(self.actions.topics("/fpga/map_reset"), [])

    def test_forward_done_failed_plan_cleared_state_group(self):
        self.core.feed_mapping_status(json.dumps({"state": "mapping", "session": 0}), self.t)
        self.assertEqual(self.cmd("e")["status"], "forwarded")
        self.assertEqual(self.cmd("f")["reason"], "map_reset_busy")                  # one in flight
        (topic, text), = self.actions.topics("/fpga/map_reset")
        self.assertEqual(json.loads(text)["id"], "e")
        self.core.latest["plan"] = ("old plan", self.t)
        self.core.feed_map_reset_status(json.dumps({"v": 1, "id": "e", "state": "running", "steps": []}), self.t)
        self.assertEqual(self.core.pop_results(), [])
        self.core.feed_map_reset_status(json.dumps({"v": 1, "id": "e", "state": "done", "steps": [{"step": "stop"}],
                                                    "scope": "s2", "previous_scope": "s1"}), self.t)
        (_, res), = self.core.pop_results()
        self.assertEqual((res["status"], res["map_reset"]["scope"]), ("done", "s2"))
        self.assertNotIn("plan", self.core.latest)
        self.core.feed_mapping_status(json.dumps({"state": "mapping", "session": 1}), self.t)
        g = self.core.compose_state(self.t)["mapping"]
        self.assertEqual((g["state"], g["session"], g["reset"]["id"]), ("mapping", 1, "e"))
        # a failed one: rejected + map_reset_failed + the runner's words
        self.assertEqual(self.cmd("g")["status"], "forwarded")
        self.core.feed_map_reset_status(json.dumps({"v": 1, "id": "g", "state": "failed", "steps": [{"step": "x"}],
                                                    "reason_zh": "重置建图没有完成（建图进程 30 秒内没有回应）"}), self.t)
        (_, res), = self.core.pop_results()
        self.assertEqual((res["status"], res["reason"]), ("rejected", "map_reset_failed"))
        self.assertIn("30 秒", res["detail"])

    def test_localization_state_has_no_mapping_group(self):
        self.assertNotIn("mapping", self.core.compose_state(self.t))


# ------------------------------------------------------------------------------------------ bridge
class FakeSession(object):
    def __init__(self):
        self.events = []
        self.resync = 0

    def put_json(self, obj):
        self.events.append(obj)

    def on_mapping_resync(self):
        self.resync += 1


class BridgeTests(unittest.TestCase):
    def ctx(self, **kw):
        c = {"link": "up", "source": "live", "scope": "rb1", "task": None}
        c.update(kw)
        return c

    def classify(self, payload, **ctx):
        return gp.classify_command({"type": "command", "version": 2, "id": "u1", "action": "map_reset",
                                    "payload": payload}, self.ctx(**ctx))

    def test_classify(self):
        self.assertEqual(self.classify({})["code"], "confirm_required")
        self.assertEqual(self.classify({"confirm": "yes"})["code"], "confirm_required")
        self.assertEqual(self.classify({"confirm": True, "x": 1})["code"], "confirm_required")
        self.assertEqual(self.classify({"confirm": True}, link="down")["code"], "link_down")
        d = self.classify({"confirm": True})
        self.assertEqual((d["kind"], d["op"], d["args"]), ("forward", "map_reset", {}))
        self.assertEqual(gp.RESULT_WAIT_S["map_reset"], 35.0)
        codes = {c for c, _, _ in gp.REJECT_CODES}
        self.assertTrue({"map_reset_no_mapping", "map_reset_busy", "map_reset_failed", "mapping_saving"} <= codes)

    def test_result_detail_and_on_map_reset(self):
        r = gp.make_result("u1", "rejected", "map_reset_failed", "重置建图没有完成（超时）")
        self.assertEqual(r["reason"], "地图重置没有全部完成：重置建图没有完成（超时）")
        b = gb.Bridge(bridge_id="bid")
        s = FakeSession()
        b.sessions.append(s) if isinstance(b.sessions, list) else b.sessions.add(s)
        b.mapping_frame, b.mapping_sha = b"frame", "sha"
        b.pending["me:1"] = {"session": s, "id": "u1", "op": "map_reset", "t": 0.0, "deadline_ms": 3000,
                             "acked": 1.0, "note": None, "result_wait_s": 35.0}
        b._on_result({"cmd_id": "me:1", "status": "done", "final": True})
        self.assertEqual(s.events[-1]["status"], "done")
        self.assertEqual((b.mapping_frame, b.mapping_sha, s.resync), (None, None, 1))


# ------------------------------------------------------------------------------------------ mapping_ctl
class MappingCtlTests(unittest.TestCase):
    def test_reset_state_machine(self):
        with tempfile.TemporaryDirectory() as d:
            t = [0.0]
            c = mapping_ctl.MappingCore(d, d, clock=lambda: t[0], db=os.path.join(d, "r1.db"))
            self.assertIsNone(c.begin_reset("c1"))
            self.assertEqual((c.state, c.reset_from), ("resetting", "mapping"))
            self.assertEqual(c.begin_reset("c2"), "busy")
            self.assertEqual(c.reset_plan(), ["rtabmap_reset", "display_reset"])
            c.finish_reset([{"step": "rtabmap_reset", "ok": True, "ms": 2400.0}], True, 2.5)
            st = c.status()
            self.assertEqual((st["state"], st["session"], st["reset"]["id"], st["reset"]["ok"],
                              st["reset"]["method"]), ("mapping", 1, "c1", True, "rtabmap_reset"))
            # after an export (Nav2 has the exported map, an alignment was published): back, then a new process
            c.request_stop()
            c.state, c.map_loaded, c.align = "exported", True, {"ok": True, "identity": False}
            c.yaml = "x.yaml"
            self.assertIsNone(c.begin_reset("c3"))
            self.assertEqual(c.reset_plan(), ["map_odom", "nav_map", "respawn", "display_reset"])
            c.finish_reset([{"step": "respawn", "ok": False, "reason_zh": "新建图进程退出"}], False, 9.0)
            st = c.status()
            self.assertEqual((st["state"], st["session"], st["reset"]["ok"], st["yaml"], st["map_loaded"]),
                             ("failed", 2, False, None, None))
            self.assertIn("建图进程没有起来", st["reset"]["reason_zh"])
            c.state = "saving"
            self.assertEqual(c.begin_reset("c4"), "mapping_saving")

    def test_run_ids(self):
        self.assertNotEqual(mapping_ctl.new_run_id("20261026_210000", clock=lambda: 0), "20261026_210000")
        same = time.strftime("%Y%m%d_%H%M%S", time.localtime(100.0))
        self.assertEqual(mapping_ctl.new_run_id(same, clock=lambda: 100.0), same + "_r2")
        cmd = ["env", "RUN_TIME=t1", "OUT=/db/run_t1", "bash", "x.sh"]
        cmd2, out2, db2 = mapping_ctl.respawn_args(cmd, "/db/run_t1", "/db/t1.db", "t1", "t2")
        self.assertEqual((cmd2[1:3], out2, db2), (["RUN_TIME=t2", "OUT=/db/run_t2"], "/db/run_t2", "/db/t2.db"))
        with self.assertRaises(ValueError):
            mapping_ctl.respawn_args(cmd, None, None, None, "t2")
        with self.assertRaises(ValueError):
            mapping_ctl.respawn_args(cmd, None, None, "zz", "t2")


class StandinFlowTests(unittest.TestCase):
    """建图 -> 重置 (respawn path, as mapping_ctl does on the host stand-in) -> 再建图 -> 结束建图 export."""

    def start(self, cmd):
        p = subprocess.Popen(cmd, start_new_session=True, stderr=subprocess.PIPE, universal_newlines=True)
        line = ""
        while "running --" not in line:
            line = p.stderr.readline()
            self.assertTrue(line, "stand-in ended before running --")
        return p

    def test_map_reset_map_again_export(self):
        standin = str(ROOT / "tools/car_run/mapping_standin.py")
        with tempfile.TemporaryDirectory() as d:
            run1 = "20261026_210000"
            cmd = ["env", "DB_DIR=" + d, "RUN_TIME=" + run1, "OUT=%s/run_%s" % (d, run1), sys.executable, standin]
            os.makedirs("%s/run_%s" % (d, run1))
            core = mapping_ctl.MappingCore("%s/run_%s" % (d, run1), d, standin=True, db="%s/%s.db" % (d, run1))
            p1 = self.start(cmd)
            self.assertIsNone(core.begin_reset("c1"))
            os.killpg(p1.pid, signal.SIGKILL)                    # the old map is discarded: no save, no export
            p1.wait(5)
            p1.stderr.close()
            run2 = mapping_ctl.new_run_id(run1)
            cmd2, out2, db2 = mapping_ctl.respawn_args(cmd, core.out, core.db, run1, run2)
            os.makedirs(out2, exist_ok=True)
            p2 = self.start(cmd2)
            core.finish_reset([{"step": "respawn", "ok": True, "ms": 300.0}], p2.poll() is None, 0.4, out=out2, db=db2)
            self.assertEqual((core.state, core.session, core.out, core.db), ("mapping", 1, out2, db2))
            self.assertTrue(core.request_stop())
            p2.send_signal(signal.SIGINT)
            err = p2.stderr.read()
            p2.wait(10)
            p2.stderr.close()
            self.assertEqual(p2.returncode, 0, err)
            core.on_child_exit(0)
            self.assertEqual(core.state, "exported", core.reason)
            self.assertTrue(core.yaml.startswith(out2))
            self.assertTrue(os.path.exists(db2))
            self.assertFalse(os.path.exists("%s/run_%s/export" % (d, run1)))   # nothing of the old map exported
            self.assertFalse(os.path.exists("%s/%s.db" % (d, run1)))


# ------------------------------------------------------------------------------------------ live_mapping.py
class FeedTests(unittest.TestCase):
    def test_scope_follows_runtime_info(self):
        f = live_mapping.MappingFollow("a" * 16)
        self.assertFalse(f.on_runtime_info(json.dumps({"runtime_boot_id": "a" * 16})))
        self.assertEqual(f.scope_source, "topic")
        self.assertTrue(f.on_runtime_info(json.dumps({"runtime_boot_id": "b" * 16})))
        self.assertEqual(f.fields()["scope"], "b" * 16)
        for bad in ("{", "[]", json.dumps({"runtime_boot_id": "XYZ"})):
            self.assertFalse(f.on_runtime_info(bad))
        self.assertEqual(f.scope, "b" * 16)

    def test_clear_on_resetting_once_and_stale_gate(self):
        t = [1000.0]
        f = live_mapping.MappingFollow("a" * 16, clock=lambda: t[0])
        self.assertFalse(f.on_mapping_status(json.dumps({"state": "mapping", "session": 0})))   # first sample
        self.assertTrue(f.accept_cloud(2e9))
        t[0] = 2e9
        self.assertTrue(f.on_mapping_status(json.dumps({"state": "resetting", "session": 0})))
        self.assertFalse(f.on_mapping_status(json.dumps({"state": "resetting", "session": 0})))  # re-sent row
        self.assertFalse(f.accept_cloud(2e9 + 5))                 # during the reset: dropped
        self.assertFalse(f.on_mapping_status(json.dumps({"state": "mapping", "session": 1})))   # already cleared
        self.assertFalse(f.accept_cloud(2e9 - 1))                 # stamped before the clear
        self.assertTrue(f.accept_cloud(2e9 + 1))
        self.assertTrue(f.accept_cloud(0))                        # no plausible stamp: never stuck
        t[0] += live_mapping.STALE_GATE_S + 1
        self.assertTrue(f.accept_cloud(2e9 - 1))                  # the gate is bounded in time
        self.assertEqual(f.resets, 1)
        # a session change whose resetting was missed (late joiner) still clears
        self.assertTrue(f.on_mapping_status(json.dumps({"state": "mapping", "session": 2})))
        self.assertEqual((f.resets, f.fields()["mappingSession"]), (2, 2))

    def test_new_voxel_feed_is_a_new_epoch_and_empty(self):
        v = live_mapping.VoxelFeed(0.03)
        xyz = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])
        v.update(xyz, np.zeros((2, 3), np.uint8))
        w = live_mapping.VoxelFeed(0.03, epoch0=(v.epoch + 1) % (1 << 31))
        self.assertEqual((len(w.keys), w.started, w.seq), (0, False, 0))
        w.update(xyz[:1], np.zeros((1, 3), np.uint8))
        self.assertEqual(w.epoch, v.epoch + 1)                    # the page's (old epoch, seq) -> a full
        head = w.delta_arrays(v.epoch, v.seq)[0]
        self.assertEqual((head["mode"], head["count"]), ("full", 1))

    def test_feed_source_wiring(self):
        src = (ROOT / "ui/geomem/tools/competition/live_mapping.py").read_text(encoding="utf-8")
        for needle in ('"/fpga/runtime_info", runtime_info, latched', '"/fpga/mapping/status", mapping_status, latched',
                       "clear_voxels()", "follow.accept_cloud("):
            self.assertIn(needle, src)
        sh = (ROOT / "ui/geomem/tools/competition/start_live_mapping.sh").read_text(encoding="utf-8")
        self.assertIn("scopeSource') == 'topic'", sh)


# ------------------------------------------------------------------------------------------ heat map
class BeliefClearTests(unittest.TestCase):
    def test_clear_publishes_none_and_ends_the_task(self):
        out = []
        f = BeliefFeed(out.append)
        task = NS(state="searching", search_planner=None, TERMINAL=("succeeded",))
        f.tick(task, 0.0)
        out.clear()
        msg = f.clear("map_reset")
        self.assertEqual((msg["phase"], msg["width"], msg["reason"]), ("none", 0, "map_reset"))
        meta, cells = parse_belief(belief_dumps(msg))
        self.assertEqual((meta["belief"]["phase"], cells), ("none", b""))
        self.assertIsNone(f.tick(task, 1.0))                  # nothing more for the old task
        self.assertEqual(len(out), 1)


if __name__ == "__main__":
    unittest.main()
