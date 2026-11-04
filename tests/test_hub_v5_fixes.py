"""Integration v5 (2026-10-12) small fixes ruled by the controller, each with an old-behaviour control
(the pre-fix module from git b2114c2, loaded under another name):

a. kv.ar.age_ms carries the AR frame age (TELEM ar.age_ms); ar_age_ms is gone; kv group age unchanged.
b. producer source warning: replay without the latched marker (after a grace) or a marker naming the
   other source -> frame_stats source_warning + one /fpga/status event per change; never for live
   without a marker; hub shows geometry_nano.source_warning and forwards the events.
c. lean executor start-up probe: missing/changed rclpy internals -> stock executor, reason in
   hub.executor, EVENT hub_executor_fallback, stdout line; old: add_node accepted, spin thread died.
d. /fpga/runtime_info: latched runner start-up facts -> STATE task.runtime_info.
e. STATE/EVENT never stop on a non-finite float: null + nonfinite_fields paths.
Python 3.8+, no ROS.
"""
import importlib.util
import io
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import types
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from pc import fpgac  # noqa: E402
from host import ros_runtime  # noqa: E402
from host.geo_producer import SourceMarker, SOURCE_WARNING_GRACE_FRAMES  # noqa: E402
from host.hub import hub_core, hub_ros  # noqa: E402
from host.hub.hub_core import HubCore, NS, MS, scrub_nonfinite  # noqa: E402
from host.hub.hub_executor import LeanExecutor, SelectingExecutor  # noqa: E402
from host.hub.fake_kv import FakeKv, KvTelemetryClient  # noqa: E402

OLD = "b2114c2"
_tmp = tempfile.mkdtemp(prefix="v5_old_")


def old_module(path, name):
    """The pre-v5 file from git, imported as its own module (None without git history)."""
    if shutil.which("git") is None:
        return None
    r = subprocess.run(["git", "-C", str(ROOT), "show", "%s:%s" % (OLD, path)],
                       stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    if r.returncode != 0:
        return None
    f = Path(_tmp) / (name + ".py")
    f.write_bytes(r.stdout)
    spec = importlib.util.spec_from_file_location(name, str(f))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def telemetry(ar=None):
    tel = KvTelemetryClient(FakeKv()).poll()
    if ar is None:
        return tel
    blocks = dict(tel.blocks)
    blocks["ar"] = ar
    return tel._replace(blocks=blocks, avail=tuple(set(tel.avail or ()) | {"ar"}))


AR = {"mode": "drm", "presenter": "drm", "age_ms": 130.0, "age_valid": True, "replay": True,
      "replay_core": True, "frame_id": 300}


# ------------------------------------------------------------------------------------- a
class ArAgeField(unittest.TestCase):
    def test_age_ms_is_the_ar_frame_age(self):
        core = HubCore("replay")
        core.feed_telemetry(telemetry(AR), NS)
        kv = core.compose_state(NS + 10 * MS)["kv"]
        self.assertEqual(kv["ar"]["age_ms"], 130.0)
        self.assertNotIn("ar_age_ms", kv["ar"])
        ref = HubCore("replay")                          # same TELEM without an ar block
        ref.feed_telemetry(telemetry(), NS)
        kv0 = ref.compose_state(NS + 10 * MS)["kv"]
        self.assertEqual((kv["age_ms"], kv["age_basis"]), (kv0["age_ms"], kv0["age_basis"]))
        self.assertNotIn("ar", kv["age_basis"])
        self.assertEqual(fpgac.kv_blocks(json.loads(fpgac.dumps(core.compose_state(NS))))["ar"]["age_ms"], 130.0)

    def test_old_hub_left_age_ms_null(self):
        old = old_module("host/hub/hub_core.py", "old_hub_core_a")
        if old is None:
            self.skipTest("no git history")
        core = old.HubCore("replay")
        core.feed_telemetry(telemetry(AR), NS)
        ar = core.compose_state(NS)["kv"]["ar"]
        self.assertEqual((ar["age_ms"], ar["ar_age_ms"]), (None, 130.0))


# ------------------------------------------------------------------------------------- b
MARKER = json.dumps({"sensor_source": "replay", "replay_id": "prep", "publisher": "replay_camera"})


class ProducerSourceWarning(unittest.TestCase):
    def run_frames(self, sm, n):
        rows, events = [], []
        for _ in range(n):
            f = sm.stats_fields()
            rows.append(f)
            ev = sm.transition(f)
            if ev is not None:
                events.append(ev)
        return rows, events

    def test_replay_without_marker_after_grace_then_cleared(self):
        sm = SourceMarker("replay")
        rows, events = self.run_frames(sm, SOURCE_WARNING_GRACE_FRAMES + 5)
        warn = [r["source_warning"] for r in rows]
        self.assertEqual(warn, [None] * SOURCE_WARNING_GRACE_FRAMES + ["replay_without_marker"] * 5)
        self.assertEqual([(e["event"], e["reason"], e["frames"]) for e in events],
                         [("source_warning", "replay_without_marker", SOURCE_WARNING_GRACE_FRAMES + 1)])
        self.assertTrue(all(r["source_mismatch"] is False for r in rows))   # not a mismatch
        sm.on_marker(MARKER)
        rows, events = self.run_frames(sm, 3)
        self.assertEqual([r["source_warning"] for r in rows], [None] * 3)
        self.assertEqual([(e["event"], e["reason"]) for e in events],
                         [("source_warning_cleared", "replay_without_marker")])
        self.assertEqual(sm.counts["frames_warning:replay_without_marker"], 5)
        json.dumps(events, allow_nan=False)

    def test_marker_within_grace_never_warns(self):
        sm = SourceMarker("replay")
        rows, events = self.run_frames(sm, 3)
        sm.on_marker(MARKER)
        rows2, events2 = self.run_frames(sm, 50)
        self.assertEqual(events + events2, [])
        self.assertTrue(all(r["source_warning"] is None for r in rows + rows2))

    def test_live_without_marker_never_warns(self):
        sm = SourceMarker("live")
        rows, events = self.run_frames(sm, 200)
        self.assertEqual((events, {r["source_warning"] for r in rows}), ([], {None}))
        self.assertEqual(sm.counts["frames_no_marker"], 200)

    def test_marker_mismatch_warns_at_once(self):
        sm = SourceMarker("live")
        sm.on_marker(MARKER)                             # a replay stand-in while we claim live
        rows, events = self.run_frames(sm, 4)
        self.assertEqual([r["source_warning"] for r in rows], ["marker_mismatch"] * 4)
        self.assertEqual([e["event"] for e in events], ["source_warning"])

    def test_old_marker_had_no_warning(self):
        old = old_module("host/geo_producer.py", "old_geo_producer_b")
        if old is None:
            self.skipTest("no git history")
        sm = old.SourceMarker("replay")
        for _ in range(SOURCE_WARNING_GRACE_FRAMES + 5):
            f = sm.stats_fields()
        self.assertEqual(f, {"source": "replay", "source_marker": None, "source_mismatch": False})
        self.assertFalse(hasattr(sm, "transition"))

    def test_hub_state_and_event(self):
        core = HubCore("replay")
        sm = SourceMarker("replay")
        for i in range(SOURCE_WARNING_GRACE_FRAMES + 1):
            f = sm.stats_fields()
            ev = sm.transition(f)
            if ev is not None:
                core.feed_status(json.dumps(ev), NS + i * MS)
            f.update({"v": 1, "kv_frame_id": i})
            core.feed_frame_stats(json.dumps(f), NS + i * MS)
        s = core.compose_state(NS + 20 * MS)
        self.assertEqual(s["geometry_nano"]["source_warning"], "replay_without_marker")
        self.assertEqual(s["geometry_nano"]["frame_source"], "replay")
        self.assertEqual(s["source"], "replay")           # a warning does not make it inconsistent
        evs = [e["event"]["event"] for e in core.pop_events(NS + 20 * MS)]
        self.assertEqual(evs, ["source_warning"])

    def test_producer_node_publishes_transition(self):
        text = (ROOT / "host/ros_geo_producer.py").read_text()
        self.assertRegex(text, r"fs\.update\(source\.stats_fields\(\)\)\n\s+self\.event\(source\.transition\(fs\)\)")


# ------------------------------------------------------------------------------------- c
class _Handle(object):
    def __init__(self, pointer):
        self.pointer = pointer
        self.taken = 0

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def take_message(self, msg_type, raw):
        self.taken += 1
        return None


def fake_node(n=3):
    subs = [types.SimpleNamespace(handle=_Handle(100 + i), msg_type=object, raw=False, callback=lambda m: None,
                                  topic_name="/t%d" % i) for i in range(n)]
    ctx = types.SimpleNamespace(handle=_Handle(1), ok=lambda: True, on_shutdown=lambda cb: None)
    return types.SimpleNamespace(subscriptions=subs, context=ctx, timers=[], services=[], clients=[],
                                 waitables=[], guards=[], publishers=[])


class _Guard(object):
    def __init__(self, ctx):
        self.handle = _Handle(2)
        self.destroyed = False

    def destroy(self):
        self.destroyed = True


class GoodWaitSet(object):
    calls = []

    def __init__(self, n_subs, n_guards, n_timers, n_clients, n_services, n_events, ctx):
        self.subs = []

    def clear_entities(self):
        self.subs = []

    def add_subscription(self, h):
        self.subs.append(h.pointer)

    def add_guard_condition(self, h):
        pass

    def wait(self, timeout):
        GoodWaitSet.calls.append(timeout)

    def get_ready_entities(self, kind):
        return list(self.subs)


class SixArgWaitSet(GoodWaitSet):
    def __init__(self, a, b, c, d, e, ctx):            # a changed constructor
        pass


class NoReadyWaitSet(GoodWaitSet):
    get_ready_entities = None


class ExecutorProbe(unittest.TestCase):
    def test_known_answers(self):
        node = fake_node()
        good = types.SimpleNamespace(WaitSet=GoodWaitSet)
        self.assertIsNone(LeanExecutor.probe(node, impl=good, guard_factory=_Guard))
        self.assertEqual(GoodWaitSet.calls[-1], 0)       # wait(0): never blocks
        self.assertEqual([s.handle.taken for s in node.subscriptions], [0, 0, 0])   # nothing taken
        self.assertEqual(LeanExecutor.probe(node, impl=types.SimpleNamespace(), guard_factory=_Guard),
                         "missing:WaitSet")
        r = LeanExecutor.probe(node, impl=types.SimpleNamespace(WaitSet=SixArgWaitSet), guard_factory=_Guard)
        self.assertTrue(r.startswith("signature:"), r)
        r = LeanExecutor.probe(node, impl=types.SimpleNamespace(WaitSet=NoReadyWaitSet), guard_factory=_Guard)
        self.assertTrue(r.startswith("signature:") or r.startswith("probe:"), r)
        bad = fake_node()
        bad.subscriptions[0].handle = types.SimpleNamespace(pointer=1)       # no take_message
        self.assertEqual(LeanExecutor.probe(bad, impl=good, guard_factory=_Guard),
                         "missing:subscription.handle.take_message")

    def test_without_rclpy_the_import_fails_and_is_named(self):
        try:
            import rclpy  # noqa: F401
            self.skipTest("rclpy present (ROS image): covered by test_hub_lean_executor")
        except ImportError:
            pass
        self.assertTrue(LeanExecutor.probe(fake_node()).startswith("import:"))

    def test_selecting_executor_falls_back_and_reports(self):
        chosen, made = [], []

        class Stock(object):
            def add_node(self, node):
                made.append(("stock", node))

            def spin(self):
                made.append("spin")

            def shutdown(self, timeout_sec=None):
                return True

        ex = SelectingExecutor(Stock, on_choice=chosen.append, probe=lambda node: "missing:WaitSet")
        node = fake_node()
        ex.add_node(node)
        ex.spin()
        self.assertEqual(chosen, [{"requested": "lean", "used": "rclpy", "fallback_reason": "missing:WaitSet"}])
        self.assertEqual(made, [("stock", node), "spin"])
        self.assertTrue(ex.shutdown(timeout_sec=1.0))
        chosen2 = []

        class Lean(Stock):
            def add_node(self, node):
                made.append(("lean", node))
        ex = SelectingExecutor(Stock, on_choice=chosen2.append, probe=lambda node: None, lean_factory=Lean)
        ex.add_node(node)
        self.assertEqual(chosen2, [{"requested": "lean", "used": "lean", "fallback_reason": None}])
        self.assertEqual(made[-1], ("lean", node))

    def test_hub_records_choice(self):
        core = HubCore("replay")
        server = types.SimpleNamespace(core=core, lock=threading.RLock())
        out = io.StringIO()
        with mock.patch.object(sys, "stdout", out):
            hub_ros.executor_chosen(server, {"requested": "lean", "used": "rclpy",
                                             "fallback_reason": "missing:WaitSet"}, clock=lambda: NS)
        self.assertEqual(out.getvalue(), "hub executor=rclpy requested=lean fallback_reason=missing:WaitSet\n")
        s = core.compose_state(NS)
        self.assertEqual(s["hub"]["executor"]["used"], "rclpy")
        self.assertEqual(s["hub"]["counts"]["executor_fallback"], 1)
        ev = core.pop_events(NS)
        self.assertEqual([(e["src"], e["event"]["event"], e["event"]["reason"]) for e in ev],
                         [("replay", "hub_executor_fallback", "missing:WaitSet")])
        ok = HubCore("replay")
        out = io.StringIO()
        with mock.patch.object(sys, "stdout", out):
            hub_ros.executor_chosen(types.SimpleNamespace(core=ok, lock=threading.RLock()),
                                    {"requested": "lean", "used": "lean", "fallback_reason": None}, clock=lambda: NS)
        self.assertEqual(out.getvalue(), "hub executor=lean\n")    # the line the board scripts grep
        self.assertEqual(ok.pop_events(NS), [])

    def test_old_executor_accepted_missing_internals_and_died_in_spin(self):
        old = old_module("host/hub/hub_executor.py", "old_hub_executor_c")
        if old is None:
            self.skipTest("no git history")
        guard_mod = types.ModuleType("rclpy.guard_condition")
        guard_mod.GuardCondition = lambda a, b, ctx: _Guard(ctx)
        impl_mod = types.ModuleType("rclpy.impl.implementation_singleton")
        impl_mod.rclpy_implementation = types.SimpleNamespace()          # no WaitSet
        mods = {"rclpy": types.ModuleType("rclpy"), "rclpy.impl": types.ModuleType("rclpy.impl"),
                "rclpy.guard_condition": guard_mod, "rclpy.impl.implementation_singleton": impl_mod}
        with mock.patch.dict(sys.modules, mods):
            ex = old.LeanExecutor()
            self.assertTrue(ex.add_node(fake_node()))   # accepted: nothing checked the internals
            self.assertFalse(hasattr(old.LeanExecutor, "probe"))
            with self.assertRaises(AttributeError):     # ... the spin thread is where it broke
                ex.spin()
            self.assertEqual(LeanExecutor.probe(fake_node()), "missing:WaitSet")   # new: caught up front


# ------------------------------------------------------------------------------------- d
class RuntimeInfo(unittest.TestCase):
    ARGS = types.SimpleNamespace(sensor_source="replay", pose_source="reference", inference_device="cpu",
                                 geometry_fallback="none", region_source="none", tf_reliable=False,
                                 stall_diag=Path("/x/stall.jsonl"))

    def test_body_known_answer(self):
        body = ros_runtime.runtime_info_body(self.ARGS, "rt-1", "kv_topic", 123, 200.0,
                                             version={"commit": "abc", "runtime_sha256": "f" * 16})
        self.assertEqual(body, {"v": 1, "runtime_boot_id": "rt-1", "start_raw_ns": 123, "sensor_source": "replay",
                                "pose_source": "reference", "inference_device": "cpu",
                                # 2026-10-14: recognition model (args without --detector: legacy, unchecked)
                                "detector": "ssdlite", "detector_checked": False,
                                "geometry_source": "kv_topic", "geometry_fallback": "none", "region_source": "none",
                                "age_gate": {"limit_ms": 200.0, "missing_origin": "refused",
                                             "origin": "producer_acquired"},
                                "tf_qos": "best_effort", "tf_retry": "arrival",
                                # 2026-10-15: camera subscription QoS (args without --camera-qos: auto -> replay
                                # is best effort)
                                "camera_qos": "sensor_data", "camera_qos_switch": "auto",
                                "stall_diag": True,
                                # v7 (2026-10-12): record write mode and gc policy (args without them: defaults)
                                "record_write": "async", "gc": "default",
                                "version": {"commit": "abc", "runtime_sha256": "f" * 16}})
        json.dumps(body, allow_nan=False)
        rel = types.SimpleNamespace(**dict(vars(self.ARGS), tf_reliable=True, stall_diag=None))
        b = ros_runtime.runtime_info_body(rel, "rt", "cpu", 1, 200.0, version={})
        self.assertEqual((b["tf_qos"], b["stall_diag"], b["age_gate"]["origin"]), ("reliable", False, "runner_arrival"))
        est = types.SimpleNamespace(**dict(vars(self.ARGS), pose_source="estimated"))
        self.assertIsNone(ros_runtime.runtime_info_body(est, "rt", "cpu", 1, 200.0, version={})["tf_qos"])
        # v6: tf_retry = the mode in effect (the runner passes it; else args; null with estimated poses)
        self.assertIsNone(ros_runtime.runtime_info_body(est, "rt", "cpu", 1, 200.0, version={})["tf_retry"])
        poll = types.SimpleNamespace(**dict(vars(self.ARGS), tf_retry="poll"))
        self.assertEqual(ros_runtime.runtime_info_body(poll, "rt", "cpu", 1, 200.0, version={})["tf_retry"], "poll")
        self.assertEqual(ros_runtime.runtime_info_body(self.ARGS, "rt", "cpu", 1, 200.0, version={},
                                                       tf_retry="poll")["tf_retry"], "poll")

    def test_code_version_reads_deployed_commit(self):
        d = Path(tempfile.mkdtemp(prefix="v5_commit_"))
        try:
            self.assertEqual(ros_runtime.code_version(d)["commit"], None)
            (d / "COMMIT").write_text("deadbeef\n")
            v = ros_runtime.code_version(d)
            self.assertEqual(v["commit"], "deadbeef")
            self.assertRegex(v["runtime_sha256"], r"^[0-9a-f]{16}$")
        finally:
            shutil.rmtree(str(d))

    def test_runner_publishes_it_latched(self):
        text = (ROOT / "host/ros_runtime.py").read_text()
        self.assertIn("self.create_publisher(String, RUNTIME_INFO_TOPIC, map_qos)", text)
        m = re.search(r"map_qos = QoSProfile\(depth=1, durability=DurabilityPolicy\.TRANSIENT_LOCAL,\s*"
                      r"reliability=ReliabilityPolicy\.RELIABLE\)", text)
        self.assertIsNotNone(m)
        self.assertLess(m.start(), text.index("RUNTIME_INFO_TOPIC, map_qos"))
        self.assertEqual(ros_runtime.RUNTIME_INFO_TOPIC, "/fpga/runtime_info")

    def test_hub_subscribes_latched_and_shows_it(self):
        subs, qos = {}, {}

        class Node(object):
            def __init__(self, name, **kw):
                pass

            def create_subscription(self, kind, topic, cb, q, **kw):
                subs[topic], qos[topic] = cb, q

            def create_publisher(self, kind, topic, q, **kw):
                return types.SimpleNamespace(publish=lambda m: None)

        m = types.SimpleNamespace(Node=Node, QoSProfile=lambda **kw: kw,
                                  DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL="tl"),
                                  ReliabilityPolicy=types.SimpleNamespace(RELIABLE="rel", BEST_EFFORT="be"),
                                  String=object, Empty=object, Bool=object, Twist=object,
                                  PoseWithCovarianceStamped=object, OccupancyGrid=object, Odometry=object,
                                  Path=object, CompressedImage=object,
                                  SubscriptionEventCallbacks=lambda **kw: kw, PublisherEventCallbacks=lambda **kw: kw)
        core = HubCore("replay")
        hub_ros.build_node(types.SimpleNamespace(core=core, lock=threading.RLock()), m, clock=lambda: NS)
        self.assertEqual(qos["/fpga/runtime_info"], {"depth": 1, "durability": "tl", "reliability": "rel"})
        # /odom best effort (10-17): matches the best-effort adapter and the reliable wheel odometry alike
        self.assertEqual(qos["/odom"], {"depth": 10, "reliability": "be"})
        self.assertIn("/fpga/runtime_info", hub_ros.SUBSCRIPTIONS)
        self.assertIsNone(core.compose_state(NS)["task"]["runtime_info"])
        body = ros_runtime.runtime_info_body(self.ARGS, "rt-1", "kv_topic", 5, 200.0, version={"commit": None})
        subs["/fpga/runtime_info"](types.SimpleNamespace(data=json.dumps(body)))
        info = core.compose_state(NS + 250 * MS)["task"]["runtime_info"]
        self.assertEqual(info["recv_age_ms"], 250.0)
        self.assertEqual({k: v for k, v in info.items() if k != "recv_age_ms"}, body)
        core.feed_runtime_info("not json", NS)
        self.assertEqual(core.counts["bad_input:runtime_info"], 1)

    def test_old_hub_had_none(self):
        old = old_module("host/hub/hub_core.py", "old_hub_core_d")
        if old is None:
            self.skipTest("no git history")
        self.assertNotIn("runtime_info", old.HubCore("replay").compose_state(NS)["task"])


# ------------------------------------------------------------------------------------- e
def feed_nonfinite(core):
    nan, inf = float("nan"), float("inf")
    core.feed_cmd_vel(nan, 0.1, NS)
    core.feed_odom(inf, -inf, nan, NS)
    core.feed_loc_pose({"x": nan, "y": 1.0, "yaw": 0.0, "cov_xy_trace": inf, "stamp_ros_s": 1.0}, NS)
    core.feed_plan(nan, NS, path={"frame_id": "map", "stamp_ros_s": nan, "points": [[0.0, 0.0], [nan, 1.0]],
                                  "n_source_poses": 2, "length_m": nan, "epsilon_m": 0.025}, path_reason=None)
    core.feed_frame_stats('{"v": 1, "frame_age_ms": NaN, "source": "replay"}', NS)
    core.feed_telemetry(telemetry(dict(AR, age_ms=nan)), NS)


NONFINITE_PATHS = {"nav.cmd_v", "nav.path_len_m", "nav.path.stamp_ros_s", "nav.path.points[1][0]",
                   "nav.path.length_m", "odom.v", "odom.w", "odom.stamp_ros_s", "pose.x", "pose.cov_xy_trace",
                   "geometry_nano.frame_age_ms", "kv.ar.age_ms", "perf.frame_age_ms_med10"}


class NonFiniteNeverStopsState(unittest.TestCase):
    def test_state_goes_out_with_nulls_and_paths(self):
        core = HubCore("replay")
        feed_nonfinite(core)
        s = core.compose_state(NS + MS)
        body = json.loads(fpgac.dumps(s))                 # allow_nan=False: must not raise
        self.assertEqual(set(body["nonfinite_fields"]), NONFINITE_PATHS)
        self.assertEqual(len(body["nonfinite_fields"]), len(NONFINITE_PATHS))
        self.assertIsNone(body["nav"]["cmd_v"])
        self.assertEqual(body["nav"]["cmd_w"], 0.1)       # the finite neighbour is kept
        self.assertEqual(body["nav"]["path"]["points"], [[0.0, 0.0], [None, 1.0]])
        self.assertEqual(body["pose"]["y"], 1.0)
        self.assertEqual(fpgac.nav_path(body)["reason"], "malformed")   # PC side refuses a null coordinate
        s2 = core.compose_state(NS + 2 * MS)
        self.assertEqual(s2["hub"]["counts"]["state_nonfinite_fields"], len(NONFINITE_PATHS))

    def test_clean_state_has_empty_list(self):
        core = HubCore("live")
        core.feed_cmd_vel(0.2, 0.0, NS)
        s = core.compose_state(NS)
        self.assertEqual(s["nonfinite_fields"], [])
        self.assertNotIn("state_nonfinite_fields", s["hub"]["counts"])

    def test_old_hub_state_could_not_be_sent(self):
        old = old_module("host/hub/hub_core.py", "old_hub_core_e")
        if old is None:
            self.skipTest("no git history")
        core = old.HubCore("replay")
        feed_nonfinite(core)
        with self.assertRaises(ValueError):
            fpgac.dumps(core.compose_state(NS + MS))

    def test_event_with_nan(self):
        core = HubCore("replay")
        core.feed_status('{"event": "fault", "value": NaN, "rows": [1.0, Infinity]}', NS)
        ev = core.pop_events(NS)
        fpgac.dumps(ev[0])
        self.assertEqual(ev[0]["nonfinite_fields"], ["value", "rows[1]"])
        self.assertEqual(ev[0]["event"], {"event": "fault", "value": None, "rows": [1.0, None]})
        old = old_module("host/hub/hub_core.py", "old_hub_core_e2")
        if old is not None:
            oc = old.HubCore("replay")
            oc.feed_status('{"event": "fault", "value": NaN}', NS)
            with self.assertRaises(ValueError):
                fpgac.dumps(oc.pop_events(NS)[0])

    def test_server_tick_keeps_sending(self):
        from host.hub.hub_server import HubServer
        core = HubCore("replay")
        feed_nonfinite(core)
        server = HubServer(core, "t" * 16, port=0, host="127.0.0.1", clock=lambda: NS + MS)
        try:
            for _ in range(3):
                server.tick()
            self.assertFalse([k for k in core.counts if k.startswith("tick_error")])
        finally:
            server.listener.close()

    def test_scrub_known_answers(self):
        self.assertEqual(scrub_nonfinite({"a": (1.5, float("nan")), "b": {"c": float("-inf"), "d": True, "e": None}}),
                         ({"a": [1.5, None], "b": {"c": None, "d": True, "e": None}}, ["a[1]", "b.c"]))
        self.assertEqual(scrub_nonfinite(float("nan")), (None, ["<root>"]))
        self.assertEqual(scrub_nonfinite([1, "x", 2.0]), ([1, "x", 2.0], []))



# ------------------------------------------------------------------------------------- f
class RelayTableBeforeThreads(unittest.TestCase):
    """tests/fpgac_relay.py: a pair is in the relay's table before its forwarding threads run, so a
    pause() right after the client's handshake (which passes through those threads) always finds it."""

    def started_at_append(self, relay_mod):
        import socket
        up = socket.socket()
        up.bind(("127.0.0.1", 0))
        up.listen(1)
        seen = []

        class Recording(list):
            def append(self, pair):
                seen.append(any(t.ident is not None for t in pair.threads))
                list.append(self, pair)

        relay = relay_mod.Relay(up.getsockname()[1])
        relay.pairs = Recording()
        c = socket.create_connection(("127.0.0.1", relay.port))
        try:
            for _ in range(200):
                if seen:
                    break
                threading.Event().wait(0.01)
            return seen
        finally:
            c.close()
            relay.close()
            up.close()

    def test_new_relay_appends_first(self):
        sys.path.insert(0, str(ROOT / "tests"))
        import fpgac_relay
        self.assertEqual(self.started_at_append(fpgac_relay), [False])

    def test_old_relay_started_threads_first(self):
        old = old_module("tests/fpgac_relay.py", "old_fpgac_relay_f")
        if old is None:
            self.skipTest("no git history")
        self.assertEqual(self.started_at_append(old), [True])


if __name__ == "__main__":
    unittest.main()
