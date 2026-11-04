"""Line D1 (2026-10-14): simulation find-object loop on the server -- the parts testable without ROS.

1. SIM data source (owner 10-14): hub --data-source sim, producer --data-source sim, FPGAC frames,
   the PC's summary line (SIM), recorded-sim force_replay; the runner's "simulation" agrees with
   sim and still with replay (the old mapping); hardware never agrees with sim.
2. Producer SourceMarker for sim: the adapter's marker agrees, a missing marker warns
   sim_without_marker after the grace, a replay marker is a mismatch.
3. sim_adapter.py: 32FC1 m -> 16UC1 mm known answers (inf / nan / out of range -> 0), optical-frame
   cloud known answer, no TF publisher in the source.
4. sim_estop_gate.latch_of and velocity_guard's optional e-stop input (rclpy stubbed): fail-closed
   on True, on a stale sample and on no sample; unchanged with the parameter empty.
5. Launch/extrinsics: the base_to_camera + camera_to_optical TF of navigation.launch.py equals the
   car mount of tools/car_geo_producer/common.base_camera(xyz=(0.13, 0, 0.35), 27 deg); bridge cloud
   filter; rolling bridge radius 0.30 in rig_inner.sh (= Nav2 robot_radius, runner free() default).
6. Run scripts and audit (tools/sim_loop, tools/verify_bottle_run.py --layout sim): shell syntax, no
   pkill, wd- container names, shared IPC/network of the stack container, radius 0.30, RPP heading
   precision; the verifier's adapter / producer / PC-recording helpers on known answers.
Python 3.8+, no ROS.
"""
import ast
import importlib.util
import json
import math
from pathlib import Path
import re
import runpy
import sys
import types
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from pc import fpgac, fpgapc  # noqa: E402
from host.hub.hub_core import HubCore, NS, MS, sensor_agrees  # noqa: E402
from host.hub import hub_server  # noqa: E402
from host.geo_producer import SourceMarker, DATA_SOURCES  # noqa: E402
from host import ros_geo_producer  # noqa: E402

SCRIPTS = ROOT / "robot_sim/fpga_sim/scripts"


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def feed(core, producer_source, t=NS):
    core.feed_status(json.dumps({"event": "task", "state": "idle", "processed_frames": 1}), t)
    core.feed_memory(json.dumps({"revision": 1, "tracks": []}), t)
    core.feed_frame_stats(json.dumps({"v": 1, "kv_frame_id": 0, "source": producer_source}), t)


class SimSource(unittest.TestCase):
    def test_hub_sim_with_simulation_runner_and_sim_producer_is_consistent(self):
        core = HubCore("sim")
        core.feed_status(json.dumps({"event": "startup", "sensor_source": "simulation"}), NS)
        core.feed_runtime_info(json.dumps({"sensor_source": "simulation"}), NS)
        feed(core, "sim")
        s = core.compose_state(NS + MS)
        self.assertEqual(s["source"], "sim")
        self.assertEqual(s["task"]["src"], "sim")
        self.assertEqual(s["estop"]["src"], "sim")       # non-ROS group: --data-source
        self.assertEqual(core.runner_producer_state, "consistent")
        self.assertIsNone(fpgac.check_source_flags(fpgac.KIND["STATE"], s))

    def test_hub_sim_rejects_hardware_runner_and_replay_producer(self):
        core = HubCore("sim")
        core.feed_status(json.dumps({"event": "startup", "sensor_source": "hardware"}), NS)
        feed(core, "replay")
        s = core.compose_state(NS + MS)
        self.assertEqual(s["source"], "inconsistent")
        self.assertEqual(s["task"]["src"], "unknown")
        self.assertEqual(core.producer_source_state, "inconsistent")

    def test_old_mapping_kept(self):
        for sensor, source, ok in (("simulation", "replay", True), ("sim", "replay", True),
                                   ("hardware", "live", True), ("replay", "replay", True),
                                   ("simulation", "sim", True), ("sim", "sim", True),
                                   ("hardware", "sim", False), ("replay", "sim", False),
                                   ("simulation", "live", False), ("bogus", "sim", False), (None, "sim", False)):
            self.assertIs(sensor_agrees(sensor, source), ok, (sensor, source))
        core = HubCore("replay")
        core.feed_status(json.dumps({"event": "startup", "sensor_source": "simulation"}), NS)
        feed(core, "replay")
        self.assertEqual(core.compose_state(NS + MS)["source"], "replay")

    def test_hub_and_producer_parsers_take_sim(self):
        p = __import__("argparse").ArgumentParser()
        hub_server.common_args(p)
        self.assertEqual(p.parse_args(["--data-source", "sim", "--token-file", "x"]).data_source, "sim")
        a = ros_geo_producer.parse_args(["--data-source", "sim", "--ground-z-m", "0", "--kv-server", "127.0.0.1:1"])[0]
        self.assertEqual(a.data_source, "sim")
        self.assertIn("sim", DATA_SOURCES)
        with self.assertRaises(ValueError):
            HubCore("simulation")

    def test_pc_shows_sim_and_recorded_sim_replays_as_replay(self):
        state = fpgapc.synthetic_state(1, "sim")
        self.assertIn(" src=SIM ", fpgapc.summary(state))
        self.assertIn(" src=replay ", fpgapc.summary(fpgapc.synthetic_state(1, "replay")))
        out = fpgac.force_replay(fpgac.KIND["STATE"], state, "x.fpgarec")
        self.assertEqual((out["source"], out["orig_source"]), ("replay", "sim"))
        self.assertEqual((out["task"]["src"], out["task"]["orig_src"]), ("replay", "sim"))
        ev = fpgac.force_replay(fpgac.KIND["EVENT"], {"src": "sim", "event": {}}, "x")
        self.assertEqual((ev["src"], ev["orig_src"]), ("replay", "sim"))
        self.assertIsNone(fpgac.check_source_flags(fpgac.KIND["MEMORY"], {"source": "sim"}))

    def test_listen_yield_control_option(self):
        src = (ROOT / "pc/fpgapc.py").read_text()
        self.assertIn('c.command("yield_control", {})', src)
        self.assertIn("--yield-control", src)


class Marker(unittest.TestCase):
    def test_sim_marker(self):
        adapter = load(SCRIPTS / "sim_adapter.py", "sim_adapter_marker")
        m = SourceMarker("sim", grace_frames=2)
        m.on_marker(json.dumps(adapter.MARKER))
        f = m.stats_fields()
        self.assertEqual((f["source"], f["source_mismatch"], f["source_warning"]), ("sim", False, None))
        m2 = SourceMarker("sim", grace_frames=2)
        warnings = [m2.stats_fields()["source_warning"] for _ in range(4)]
        self.assertEqual(warnings, [None, None, "sim_without_marker", "sim_without_marker"])
        m3 = SourceMarker("sim")
        m3.on_marker(json.dumps({"sensor_source": "replay"}))
        self.assertEqual(m3.stats_fields()["source_warning"], "marker_mismatch")
        live = SourceMarker("live", grace_frames=0)
        self.assertIsNone([live.stats_fields() for _ in range(3)][-1]["source_warning"])


class Adapter(unittest.TestCase):
    mod = load(SCRIPTS / "sim_adapter.py", "sim_adapter_t")

    def test_depth_codes_known_answer(self):
        d = np.array([[0.5, np.inf, np.nan, 0.1], [0.15, 10.0, 10.2, 1.2345]], np.float32)
        codes, ok = self.mod.depth_to_mm(d)
        self.assertEqual(codes.dtype, np.uint16)
        self.assertEqual(codes.tolist(), [[500, 0, 0, 0], [150, 10000, 0, 1234]])
        self.assertEqual(ok.tolist(), [[True, False, False, False], [True, True, False, True]])

    def test_cloud_known_answer_optical_axes(self):
        d = np.full((4, 6), 2.0, np.float32)
        d[0, 0] = np.inf
        codes, ok = self.mod.depth_to_mm(d)
        k = [2.0, 0, 3.0, 0, 4.0, 2.0, 0, 0, 1]          # fx 2 cx 3 fy 4 cy 2
        pts = self.mod.cloud_points(d, ok, k, 2)
        # stride-2 pixels (u, v) in {0,2,4} x {0,2}; (0, 0) invalid
        want = [[(u - 3.0) * 2.0 / 2.0, (v - 2.0) * 2.0 / 4.0, 2.0] for v in (0, 2) for u in (0, 2, 4)
                if (u, v) != (0, 0)]
        np.testing.assert_allclose(pts, np.array(want, np.float32))
        self.assertTrue(np.isfinite(pts).all())
        with self.assertRaises(ValueError):
            self.mod.cloud_points(d, ok, [0] * 9, 2)

    def test_no_tf_publisher(self):
        src = (SCRIPTS / "sim_adapter.py").read_text()
        code = "\n".join(l for l in src.splitlines() if not l.lstrip().startswith("#"))
        body = code.split('"""', 2)[2]                    # skip the module docstring
        for word in ("TransformBroadcaster", "StaticTransformBroadcaster", "/tf"):
            self.assertNotIn(word, body)


class _Pub(object):
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


def stub_ros():
    """Minimal rclpy / msgs stand-ins for velocity_guard.py (never imported for real here)."""
    class Twist(object):
        def __init__(self, x=0.0):
            self.linear = types.SimpleNamespace(x=x)
            self.angular = types.SimpleNamespace(z=0.0)

    class Bool(object):
        def __init__(self, data=False):
            self.data = data

    class Param(object):
        def __init__(self, value):
            self.value = value

    class Node(object):
        params = {}

        def __init__(self, name):
            self.subs = {}

        def create_publisher(self, kind, topic, depth):
            return _Pub()

        def create_subscription(self, kind, topic, cb, depth):
            self.subs[topic] = cb

        def create_timer(self, period, cb):
            pass

        def declare_parameter(self, name, default):
            return Param(Node.params.get(name, default))

        def get_logger(self):
            return types.SimpleNamespace(info=lambda *a: None)

    mods = {"rclpy": types.ModuleType("rclpy"), "rclpy.node": types.ModuleType("rclpy.node"),
            "rclpy.executors": types.ModuleType("rclpy.executors"),
            "geometry_msgs": types.ModuleType("geometry_msgs"), "geometry_msgs.msg": types.ModuleType("geometry_msgs.msg"),
            "std_msgs": types.ModuleType("std_msgs"), "std_msgs.msg": types.ModuleType("std_msgs.msg")}
    mods["rclpy.node"].Node = Node
    mods["rclpy.executors"].ExternalShutdownException = RuntimeError
    mods["geometry_msgs.msg"].Twist = Twist
    mods["std_msgs.msg"].Bool = Bool
    return mods, Node, Twist, Bool


class Guard(unittest.TestCase):
    def setUp(self):
        self.mods, self.Node, self.Twist, self.Bool = stub_ros()
        self.saved = {k: sys.modules.get(k) for k in self.mods}
        sys.modules.update(self.mods)
        self.mod = load(SCRIPTS / "velocity_guard.py", "velocity_guard_t")
        self.now = [100.0]
        self.mod.time = types.SimpleNamespace(monotonic=lambda: self.now[0])

    def tearDown(self):
        for k, v in self.saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v

    def guard(self, topic):
        self.Node.params = {"estop_topic": topic}
        g = self.mod.VelocityGuard()
        g.permit(self.Bool(True))
        return g

    def passed(self, g):
        g.output.sent.clear()
        g.command(self.Twist(0.2))
        return g.output.sent[-1].linear.x == 0.2

    def test_without_parameter_unchanged(self):
        g = self.guard("")
        self.assertTrue(self.passed(g))
        self.assertNotIn("/fpga/sim_estop", g.subs)

    def test_estop_fail_closed(self):
        g = self.guard("/fpga/sim_estop")
        self.assertFalse(self.passed(g))                     # no e-stop sample yet
        g.subs["/fpga/sim_estop"](self.Bool(False))
        self.assertTrue(self.passed(g))
        g.subs["/fpga/sim_estop"](self.Bool(True))
        self.assertEqual(g.output.sent[-1].linear.x, 0.0)   # zero published on the latch itself
        self.assertFalse(self.passed(g))
        g.subs["/fpga/sim_estop"](self.Bool(False))
        g.permit(self.Bool(True))
        self.now[0] += 0.4
        g.permit(self.Bool(True))
        self.assertTrue(self.passed(g))
        self.now[0] += 0.2                                   # e-stop sample 0.6 s old, permit fresh
        g.permit(self.Bool(True))
        self.assertFalse(self.passed(g))

    def test_estop_gate_latch_of(self):
        gate = load(SCRIPTS / "sim_estop_gate.py", "sim_estop_gate_t")
        self.assertIs(gate.latch_of({"estop": {"latched": 1, "epoch": 3}}), True)
        self.assertIs(gate.latch_of({"estop": {"latched": False}}), False)
        self.assertIsNone(gate.latch_of({"estop": {}}))
        self.assertIsNone(gate.latch_of({}))
        self.assertIsNone(gate.latch_of(None))


def rpy_matrix(xyz, rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    t = np.eye(4)
    t[:3, :3] = rz @ ry @ rx
    t[:3, 3] = xyz
    return t


class LaunchAndConfig(unittest.TestCase):
    def static_tfs(self):
        tree = ast.parse((ROOT / "robot_sim/fpga_sim/launch/navigation.launch.py").read_text())
        out = {}
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "static_tf":
                kw = {k.arg: ast.literal_eval(k.value) for k in node.keywords}
                out[node.args[0].value] = (kw.get("xyz", (0, 0, 0)), kw.get("rpy", (0, 0, 0)))
        return out

    def test_launch_tf_is_the_car_mount(self):
        from tools.car_geo_producer.common import base_camera
        tfs = self.static_tfs()
        t = rpy_matrix(*tfs["base_to_camera"]) @ rpy_matrix(*tfs["camera_to_optical"])
        np.testing.assert_allclose(t, base_camera(xyz=(0.13, 0.0, 0.35), pitch_deg=27.0), atol=1e-8)
        manifest = json.loads((ROOT / "robot_sim/fpga_sim/config/extrinsics_sim.json").read_text())
        self.assertEqual(list(tfs["base_to_camera"][0]), manifest["base_to_tracked"]["xyz"])
        self.assertEqual(list(tfs["base_to_camera"][1]), manifest["base_to_tracked"]["rpy"])

    def test_cloud_bridge_filter(self):
        prof = runpy.run_path(str(ROOT / "robot_sim/fpga_sim/config/nav_input_profile.py"))
        import yaml
        entries = yaml.safe_load((ROOT / "robot_sim/fpga_sim/config/bridge.yaml").read_text())
        self.assertEqual(prof["cloud_source"]({}), "gazebo")
        self.assertEqual(prof["cloud_source"]({"FPGA_SIM_CLOUD": "adapter"}), "adapter")
        with self.assertRaises(ValueError):
            prof["cloud_source"]({"FPGA_SIM_CLOUD": "both"})
        self.assertEqual(prof["cloud_inputs"](entries, "gazebo"), entries)
        kept = prof["cloud_inputs"](entries, "adapter")
        self.assertEqual({e["ros_topic_name"] for e in entries} - {e["ros_topic_name"] for e in kept}, {"/fpga/points"})

    def test_one_robot_radius(self):
        rig = (ROOT / "tools/car_geo_producer/rig_inner.sh").read_text()
        self.assertIn("-p robot_radius_m:=0.30", rig)
        self.assertNotIn("robot_radius_m:=0.15", rig)
        nav = (ROOT / "robot_sim/fpga_sim/config/nav2_overrides.yaml").read_text()
        self.assertEqual(re.findall(r"robot_radius: ([0-9.]+)", nav), ["0.30", "0.30"])
        runtime = (ROOT / "host/ros_runtime.py").read_text()
        self.assertIn("def free(self, x, y, radius=0.30):", runtime)


class RunScripts(unittest.TestCase):
    OUTER = ROOT / "tools/sim_loop/run_sim_loop.sh"
    INNER = ROOT / "tools/sim_loop/stack_inner.sh"

    def test_shell_syntax(self):
        import subprocess
        for path in (self.OUTER, self.INNER):
            self.assertEqual(subprocess.run(["bash", "-n", str(path)]).returncode, 0, path)

    def test_discipline(self):
        outer, inner = self.OUTER.read_text(), self.INNER.read_text()
        for text in (outer, inner):
            code = "\n".join(l for l in text.splitlines() if not l.lstrip().startswith("#"))
            self.assertNotIn("pkill", code)
        self.assertIn("GZ=wd2-sim-gz-$TAG", outer)   # D2: wd2- prefix
        self.assertIn("STACK=wd2-sim-stack-$TAG", outer)
        # SHM transport between the two containers needs one /dev/shm (and one loopback)
        self.assertIn('--network "container:$GZ" --ipc "container:$GZ"', outer)
        self.assertIn("--ipc shareable", outer)
        self.assertIn("-p robot_radius_m:=0.30", inner)
        self.assertIn("rpp_heading_precision.yaml", inner)
        self.assertIn("--geometry-source kv_topic --geometry-fallback none", inner)
        self.assertIn("--data-source sim", inner)
        # fake KV: own wd- container, only CAP_NET_ADMIN kept (SO_RCVBUFFORCE like the board), not root
        self.assertIn("KV=wd2-sim-kv-$TAG", outer)
        self.assertIn("--cap-drop ALL --cap-add NET_ADMIN", outer)
        self.assertIn('--reuid="$(id -u)"', outer)
        self.assertIn("--ambient-caps=+net_admin --bounding-set=-all,+net_admin", outer)
        self.assertIn('--network "container:$GZ" --cpuset-cpus "$KV_CPUS"', outer)
        self.assertIn('docker kill -s INT "$KV"', outer)
        self.assertIn("-e KV_EXTERNAL=1", outer)


class RowInterleavedCore(unittest.TestCase):
    """kv_server --core cpu --cpu-threads T (host stand-in): identical flags / counts / status to
    the single g++ core on known frames, including invalid, out-of-range and NaN pixels, an
    argument error, odd sizes and more threads than rows."""

    @classmethod
    def setUpClass(cls):
        import shutil, tempfile, os
        if shutil.which("g++") is None:
            raise unittest.SkipTest("g++ missing")
        from host.m7_frame_native import FrameNativeM7, compile_frame_native
        cls.kvs = load(ROOT / "tools/kv_link/kv_server.py", "kv_server_d1")
        lib = compile_frame_native(os.path.join(tempfile.mkdtemp(), "m7_frame.so"))
        cls.fn = FrameNativeM7(lib).lib.fpga_m7_depth_frame
        from host.kv_link import GRID_CAPACITY, MAX_PIXELS
        cls.cap, cls.maxpix = GRID_CAPACITY, MAX_PIXELS

    def frame(self, w, h, seed):
        rng = np.random.RandomState(seed)
        fx = w / 2.0 / math.tan(0.79)
        K = np.array([[fx, 0, w / 2.0], [0, fx, h / 2.0], [0, 0, 1]], np.float64)
        inv = np.linalg.inv(K).astype(np.float32)
        p = 0.471238898
        Ro = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
        Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
        wc = np.hstack([Ry.dot(Ro), [[0.13], [0.0], [0.35]]]).astype(np.float32)
        depth = np.zeros(self.maxpix, np.float32)
        d = rng.uniform(0.02, 11.0, w * h).astype(np.float32)
        d[rng.rand(w * h) < 0.05] = np.nan
        d[rng.rand(w * h) < 0.02] = np.inf
        depth[:w * h] = d
        return depth, inv, wc

    def run_core(self, core, depth, inv, wc, w, h, voxel=0.05):
        flags = np.full(self.cap, 99, np.uint8)
        counts = np.full(3, 99, np.uint32)
        status = np.full(1, 99, np.uint32)
        core(depth, inv, wc, np.array([-0.05, -0.8, -0.05]), voxel, w, h, 32, 32, 10, flags, counts, status)
        return flags, counts.tolist(), int(status[0])

    def test_identical_to_single_core(self):
        for (w, h, seed) in ((640, 400, 1), (321, 7, 2), (64, 3, 3)):
            depth, inv, wc = self.frame(w, h, seed)
            ref = self.run_core(self.fn, depth, inv, wc, w, h)
            self.assertEqual(ref[2], 0)
            self.assertGreater(ref[1][0] + ref[1][2], 0)
            for T in (2, 3, 4, 5):
                got = self.run_core(self.kvs.RowInterleavedCpuCore(self.fn, T), depth, inv, wc, w, h)
                self.assertTrue(np.array_equal(ref[0], got[0]), (w, h, T))
                self.assertEqual(ref[1:], got[1:], (w, h, T))

    def test_error_status_like_single_core(self):
        depth, inv, wc = self.frame(64, 8, 4)
        ref = self.run_core(self.fn, depth, inv, wc, 64, 8, voxel=-1.0)
        got = self.run_core(self.kvs.RowInterleavedCpuCore(self.fn, 3), depth, inv, wc, 64, 8, voxel=-1.0)
        self.assertNotEqual(ref[2], 0)
        self.assertEqual(ref[1:], got[1:])
        self.assertTrue(np.array_equal(ref[0], got[0]))     # flags untouched (99) by both

    def test_positive_control_detects_a_dropped_row(self):
        depth, inv, wc = self.frame(160, 40, 5)
        ref = self.run_core(self.fn, depth, inv, wc, 160, 40)
        bad = depth.copy()
        bad[:160] = 0.0                                     # one image row lost
        got = self.run_core(self.kvs.RowInterleavedCpuCore(self.fn, 4), bad, inv, wc, 160, 40)
        self.assertNotEqual(ref[1], got[1])

    def test_needs_two_threads(self):
        with self.assertRaises(ValueError):
            self.kvs.RowInterleavedCpuCore(self.fn, 1)


class VerifySim(unittest.TestCase):
    def setUp(self):
        self.v = load(ROOT / "tools/verify_bottle_run.py", "verify_bottle_run_d1")

    def test_adapter_ok(self):
        good = {"depth_out": 5, "cloud_out": 5, "nonfinite_out": 0, "bad_encoding": 0, "valid_fraction_min": 0.4}
        self.assertTrue(self.v.adapter_ok(good))
        for key, bad in (("depth_out", 0), ("cloud_out", 0), ("nonfinite_out", 1), ("bad_encoding", 2),
                         ("valid_fraction_min", 0.0), ("valid_fraction_min", None)):
            self.assertFalse(self.v.adapter_ok(dict(good, **{key: bad})), key)
        self.assertFalse(self.v.adapter_ok({}))

    def test_producer_fused(self):
        self.assertEqual(self.v.producer_fused({"counts": {"published_deltas": 561}}), 561)
        self.assertIsNone(self.v.producer_fused({"counts": {}}))
        self.assertIsNone(self.v.producer_fused({}))

    def test_pc_state_sources(self):
        import tempfile, os
        d = tempfile.mkdtemp()
        path = os.path.join(d, "pc.fpgarec")
        rec = fpgac.Recorder(path)
        rec.hello({"v": 1, "ok": True, "hub_boot_id": "t"}, "t")
        for i in range(1, 4):
            rec.write(i * 1000, fpgac.encode_json(fpgac.KIND["STATE"], i, fpgapc.synthetic_state(i, "sim")))
        rec.close()
        sources, n = self.v.pc_state_sources(path)
        self.assertEqual(sources, ["sim"] * 3)
        self.assertGreaterEqual(n, 3)


if __name__ == "__main__":
    unittest.main()
