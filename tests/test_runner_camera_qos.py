"""Runner camera subscription QoS switch (2026-10-15, camera-drop batch 2 follow-up).

1. camera_reliable / camera_qos_label: --camera-qos auto (default) = reliable for --sensor-source hardware only,
   replay / simulation best effort; reliable and sensor_data override; args without the field = auto.
2. runner_camera_qos: default = qos_profile_sensor_data itself; reliable = RELIABLE + VOLATILE + KEEP_LAST depth
   (the producer's host.geo_producer.depth_sub_qos, not a copy).
3. CLI (main()'s own argparse, captured before ROS is imported): defaults, the three values, depth, bad values
   refused, --ros-args remaps passed through to rclpy untouched.
4. runtime_info_body records camera_qos / camera_qos_switch.
5. Wiring (source): the three camera subscriptions take one profile; none is left on qos_profile_sensor_data
   directly; old-behaviour control: the same check on the runner before this change fails.
6. Launch sites: tools/car_geometry_kv/ros_arm.sh ("hardware" with a best-effort replay camera) pins sensor_data;
   the chain fixture's live runner remaps to the driver (depth CameraInfo); session v8rt arms.
7. --input-log known answer: RunnerInputLog rows read by cam_drop/bx_readout.py as receiver rx_runner against a
   publisher record with 3 stamps withheld -> missing 3, extra 0 (positive control of the whole format chain);
   message-lost records land in rx_runner.json.
8. runner_qos_readout.graph_reliability / graph_summary on ros2 topic info -v text, named (host) and board form
   (_NODE_NAME_UNKNOWN_: counted by GID participant prefix), with a best-effort control.
10. Graph snapshot after the v8rt race (2026-10-15): the four real snapshots of session v8rt 2039
   (tests/data/camera_topics_v8rt_20261015/{b1,r1,r2,b2}.txt = runs/integ-car-chain-v3-20261011-results/board/
   nano_v8rt_2039/runs/rt_2039_<arm>/rig/camera_topics.txt, verbatim) are all judged invalid by graph_check, each
   for its own reason (old-parser control: r2/b2 were read as two topics); a snapshot with two topic sections
   raises GraphSnapshotError and the readout CLI exits 3; converged variants pass, and the same converged text
   under the other switch fails; a timed-out wait record voids a converged snapshot; graph_snapshot.py polls
   until converged / until the timeout and writes the wait record the readout reads.
9. With rclpy (ROS image only; skipped on the host interpreters): qos_check_compatible says a RELIABLE reader is
   incompatible with a BEST_EFFORT writer (why auto keeps replay/sim best effort) and compatible with a
   RELIABLE writer (the Orbbec driver); a node's own subscriptions show the chosen reliability; message_filters
   Subscriber passes event_callbacks through.
Python 3.8+.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host import ros_runtime  # noqa: E402
from host.record_writer import RecordWriter  # noqa: E402

PRE_CHANGE_COMMIT = "3cd9c82"           # the producer switch merged, runner untouched


def _fake_qos():
    class Profile(object):
        def __init__(self, **kw):
            self.kw = kw
    enum = lambda *names: types.SimpleNamespace(**{n: n.lower() for n in names})  # noqa: E731
    return types.SimpleNamespace(QoSProfile=Profile, qos_profile_sensor_data=object(),
                                 ReliabilityPolicy=enum("RELIABLE", "BEST_EFFORT"),
                                 DurabilityPolicy=enum("VOLATILE", "TRANSIENT_LOCAL"),
                                 HistoryPolicy=enum("KEEP_LAST", "KEEP_ALL"))


NS = types.SimpleNamespace


class Resolution(unittest.TestCase):
    def test_auto_by_sensor_source(self):
        self.assertTrue(ros_runtime.camera_reliable(NS(sensor_source="hardware", camera_qos="auto")))
        self.assertFalse(ros_runtime.camera_reliable(NS(sensor_source="replay", camera_qos="auto")))
        self.assertFalse(ros_runtime.camera_reliable(NS(sensor_source="simulation", camera_qos="auto")))

    def test_args_without_field_are_auto(self):
        self.assertTrue(ros_runtime.camera_reliable(NS(sensor_source="hardware")))
        self.assertFalse(ros_runtime.camera_reliable(NS(sensor_source="replay")))
        self.assertEqual(ros_runtime.camera_qos_label(NS(sensor_source="hardware")), "reliable:5")

    def test_explicit_values_override(self):
        for src in ("hardware", "replay", "simulation"):
            self.assertTrue(ros_runtime.camera_reliable(NS(sensor_source=src, camera_qos="reliable")), src)
            self.assertFalse(ros_runtime.camera_reliable(NS(sensor_source=src, camera_qos="sensor_data")), src)

    def test_label(self):
        self.assertEqual(ros_runtime.camera_qos_label(NS(sensor_source="replay", camera_qos="auto")), "sensor_data")
        self.assertEqual(ros_runtime.camera_qos_label(NS(sensor_source="replay", camera_qos="reliable",
                                                         camera_qos_depth=7)), "reliable:7")
        self.assertEqual(ros_runtime.camera_qos_label(NS(sensor_source="hardware", camera_qos="sensor_data",
                                                         camera_qos_depth=7)), "sensor_data")

    def test_unknown_mode_refused(self):
        with self.assertRaises(ValueError):
            ros_runtime.camera_reliable(NS(sensor_source="hardware", camera_qos="reliable_ish"))


class Profile(unittest.TestCase):
    def test_default_is_sensor_data_itself(self):
        q = _fake_qos()
        self.assertIs(ros_runtime.runner_camera_qos(qos=q), q.qos_profile_sensor_data)
        self.assertEqual(ros_runtime.CAMERA_SUB_DEPTH, 5)

    def test_reliable(self):
        p = ros_runtime.runner_camera_qos(True, 7, qos=_fake_qos())
        self.assertEqual(p.kw, {"depth": 7, "history": "keep_last", "durability": "volatile",
                                "reliability": "reliable"})

    def test_is_the_producer_profile(self):
        from host.geo_producer import depth_sub_qos
        q = _fake_qos()
        self.assertEqual(ros_runtime.runner_camera_qos(True, 5, qos=q).kw, depth_sub_qos(q, reliable=True, depth=5).kw)


class _Parsed(Exception):
    pass


def parsed(argv):
    """main()'s own argparse result (args, ros_args), captured before main imports rclpy."""
    real = argparse.ArgumentParser.parse_known_args
    box = {}

    def capture(self, args=None, namespace=None):
        box["r"] = real(self, args, namespace)
        raise _Parsed()
    with mock.patch.object(argparse.ArgumentParser, "parse_known_args", capture):
        try:
            ros_runtime.main(argv)
        except _Parsed:
            return box["r"]
    raise AssertionError("main() did not parse")


class Cli(unittest.TestCase):
    BASE = ["--socket", "/tmp/x.sock", "--pose-source", "reference", "--sensor-source", "hardware"]

    def test_defaults(self):
        a, _ = parsed(self.BASE)
        self.assertEqual((a.camera_qos, a.camera_qos_depth, a.input_log), ("auto", 5, None))
        self.assertTrue(ros_runtime.camera_reliable(a))

    def test_values(self):
        for v in ("auto", "reliable", "sensor_data"):
            self.assertEqual(parsed(self.BASE + ["--camera-qos", v])[0].camera_qos, v)
        self.assertEqual(parsed(self.BASE + ["--camera-qos=sensor_data"])[0].camera_qos, "sensor_data")
        self.assertEqual(parsed(self.BASE + ["--camera-qos-depth", "9"])[0].camera_qos_depth, 9)
        self.assertEqual(parsed(self.BASE + ["--input-log", "/x/rx_runner.csv"])[0].input_log, Path("/x/rx_runner.csv"))

    def test_bad_values_refused(self):
        with mock.patch("sys.stderr"):
            with self.assertRaises(SystemExit):
                ros_runtime.main(self.BASE + ["--camera-qos", "best_effort"])
            with self.assertRaises(SystemExit):           # checked after parsing, still before rclpy
                ros_runtime.main(self.BASE + ["--camera-qos-depth", "0"])

    def test_remaps_pass_through(self):
        # the chain fixture's live runner: flags first, --ros-args last
        a, ros = parsed(self.BASE + ["--camera-qos=reliable", "--camera-frame", "camera_color_optical_frame",
                                     "--ros-args", "-r", "/fpga/rgb:=/camera/color/image_raw",
                                     "-r", "/fpga/depth:=/camera/depth/image_raw",
                                     "-r", "/fpga/camera_info:=/camera/depth/camera_info"])
        self.assertEqual((a.camera_qos, a.camera_frame), ("reliable", "camera_color_optical_frame"))
        self.assertEqual(ros, ["--ros-args", "-r", "/fpga/rgb:=/camera/color/image_raw",
                               "-r", "/fpga/depth:=/camera/depth/image_raw",
                               "-r", "/fpga/camera_info:=/camera/depth/camera_info"])


class RuntimeInfo(unittest.TestCase):
    def body(self, **kw):
        a = NS(sensor_source="hardware", pose_source="reference", inference_device="cpu", **kw)
        return ros_runtime.runtime_info_body(a, "rt", "kv_topic", 1, 200.0, version={})

    def test_recorded(self):
        b = self.body()
        self.assertEqual((b["camera_qos"], b["camera_qos_switch"]), ("reliable:5", "auto"))
        b = self.body(camera_qos="sensor_data")
        self.assertEqual((b["camera_qos"], b["camera_qos_switch"]), ("sensor_data", "sensor_data"))
        b = self.body(camera_qos="reliable", camera_qos_depth=3)
        self.assertEqual(b["camera_qos"], "reliable:3")
        json.dumps(b, allow_nan=False)


def wiring_problems(src):
    """What the runner's camera subscriptions get, read from the source text."""
    problems = []
    for line in ('"/fpga/camera_info", self.on_info, camera_qos',
                 'Subscriber(self, Image, "/fpga/rgb", qos_profile=camera_qos',
                 'Subscriber(self, Image, "/fpga/depth", qos_profile=camera_qos'):
        if line not in src:
            problems.append("missing: " + line)
    for line in ('self.on_info, qos_profile_sensor_data', '"/fpga/rgb", qos_profile=qos_profile_sensor_data',
                 '"/fpga/depth", qos_profile=qos_profile_sensor_data'):
        if line in src:
            problems.append("left on sensor data: " + line)
    if "runner_camera_qos(True, camera_qos_depth(args)) if camera_reliable(args)" not in src:
        problems.append("profile not switched by camera_reliable(args)")
    return problems


class Wiring(unittest.TestCase):
    SRC = (ROOT / "host" / "ros_runtime.py").read_text()

    def test_current(self):
        self.assertEqual(wiring_problems(self.SRC), [])
        self.assertIn('"camera_qos": camera_qos_label(args)', self.SRC)

    def test_old_behaviour_control(self):
        try:
            old = subprocess.run(["git", "-C", str(ROOT), "show", PRE_CHANGE_COMMIT + ":host/ros_runtime.py"],
                                 stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=True).stdout.decode()
        except (OSError, subprocess.CalledProcessError):
            self.skipTest("no git history here (board tree)")
        p = wiring_problems(old)
        self.assertIn('left on sensor data: "/fpga/rgb", qos_profile=qos_profile_sensor_data', p)
        self.assertTrue(any(x.startswith("missing:") for x in p))


class LaunchSites(unittest.TestCase):
    def read(self, rel):
        return (ROOT / rel).read_text()

    def test_ros_arm_pins_sensor_data(self):
        s = self.read("tools/car_geometry_kv/ros_arm.sh")
        self.assertIn("--sensor-source hardware --camera-qos sensor_data", s)
        # its camera is the best-effort replay stand-in (why the pin is needed)
        self.assertIn("qos_profile_sensor_data", self.read("tools/car_geometry_kv/replay_camera.py"))

    def test_best_effort_publishers_documented(self):
        for rel in ("tools/car_geo_producer/replay_camera.py", "robot_sim/fpga_sim/scripts/sim_adapter.py"):
            self.assertIn("qos_profile_sensor_data", self.read(rel), rel)

    def test_chain_fixture_live_runner(self):
        s = self.read("tools/integ_car_chain/nano_chain_inner_v2.sh")
        self.assertIn('SRC=live; RT_SRC=hardware', s)
        self.assertIn('"/fpga/camera_info:=${LIVE_INFO_TOPIC:-/camera/depth/camera_info}"', s)
        self.assertIn('"/fpga/rgb:=${LIVE_COLOR_TOPIC:-/camera/color/image_raw}"', s)
        self.assertIn('--camera-frame "${LIVE_CAMERA_FRAME:-camera_color_optical_frame}"', s)
        self.assertIn('--input-log "$OUT/runtime/rx_runner.csv"', s)
        self.assertIn('$RUNTIME_EXTRA "${RT_LIVE[@]}"', s)          # --ros-args last
        self.assertNotIn("CAMERA=live needs RUNTIME=0", s)
        n = self.read("docs/design/integ_car_chain_v3_20261011/nano_v3.sh")
        self.assertIn("LIVE_DRV_LOG=*|RUNTIME=*|RUNTIME_EXTRA=*) ;;", n)

    def test_session_arms(self):
        s = self.read("docs/design/integ_car_chain_v3_20261011/session_v3.sh")
        self.assertIn('case "$arm" in r*) RQ="--camera-qos=reliable" ;; *) RQ="--camera-qos=sensor_data" ;; esac', s)
        self.assertIn("${V8RT_ORDER:-b1 r1 r2 b2}", s)
        self.assertIn("\"'--depth-qos=reliable'\"", s)
        self.assertIn('[ "$MODE" = v8cam ] || [ "$MODE" = v8rt ]', s)      # camera lock
        for f in ("tools/integ_car_chain/nano_chain_inner_v2.sh", "docs/design/integ_car_chain_v3_20261011/session_v3.sh",
                  "docs/design/integ_car_chain_v3_20261011/nano_v3.sh", "tools/car_geo_producer/rig_inner.sh",
                  "tools/car_geometry_kv/ros_arm.sh"):
            subprocess.check_call(["bash", "-n", str(ROOT / f)])


class FakeSub(object):
    def __init__(self, topic_name, seen):
        self.topic_name, self.callback = topic_name, (lambda msg: seen.append(msg))


def msg(stamp_ns):
    return NS(header=NS(stamp=NS(sec=stamp_ns // 10 ** 9, nanosec=stamp_ns % 10 ** 9)))


class InputLogKnownAnswer(unittest.TestCase):
    def test_bx_readout_reads_it_as_a_receiver(self):
        d = Path(tempfile.mkdtemp(prefix="rx_runner_"))
        t0 = 1791995630 * 10 ** 9
        stamps = [t0 + 2 * 10 ** 9 + k * 33333333 for k in range(60)]
        withheld = {stamps[10], stamps[11], stamps[40]}
        with open(d / "replay.csv", "w") as f:                # publisher record (bx_readout's replay layout)
            f.write("stamp_ns\n" + "".join("%d\n" % s for s in stamps))
        (d / "env.txt").write_text("domain=1\n")
        (d / "window.txt").write_text("%.3f %.3f\n" % (t0 / 1e9 + 1, t0 / 1e9 + 10))
        log_path = d / "rx_runner.csv"
        il = ros_runtime.RunnerInputLog(log_path, RecordWriter(open(log_path, "x"), mode="sync", name="input",
                                                               flush_each=False),
                                        {"kind": "test", "camera_qos": "reliable:5"}, wall=lambda: t0, raw=lambda: 7)
        seen = []
        subs = {n: FakeSub(r, seen) for n, r in (("/fpga/depth", "/camera/depth/image_raw"),
                                                  ("/fpga/rgb", "/camera/color/image_raw"))}
        for n, s in subs.items():
            il.wrap(s, n)
        lost = il.lost_callback("/fpga/depth")
        lost(NS(total_count=2, total_count_change=2))
        il.lost_unsupported("/fpga/rgb", RuntimeError("no event"))
        for s in stamps:
            if s not in withheld:
                for sub in subs.values():
                    sub.callback(msg(s))
        self.assertEqual(len(seen), 2 * (len(stamps) - 3))      # the inner callbacks still ran
        summary = il.close()
        self.assertEqual(summary["_receiver"]["camera_qos"], "reliable:5")
        self.assertEqual(summary["/camera/depth/image_raw"]["rows"], len(stamps) - 3)
        self.assertEqual(summary["/camera/depth/image_raw"]["lost"]["total_count"], 2)
        self.assertFalse(summary["/camera/color/image_raw"]["lost"]["supported"])
        on_disk = json.loads((d / "rx_runner.json").read_text())
        self.assertEqual(on_disk["/camera/depth/image_raw"]["lost"]["total_count"], 2)
        with open(log_path, newline="") as f:
            rows = list(csv.reader(f))
        self.assertEqual(rows[0], ["topic", "stamp_ns", "wall_ns", "arrival_raw_ns"])
        out = subprocess.run([sys.executable, str(ROOT / "tools/integ_car_chain/cam_drop/bx_readout.py"), str(d)],
                             stdout=subprocess.PIPE, check=True).stdout
        res = json.loads(out)
        rx = res["receivers"]["rx_runner"]
        self.assertEqual(rx["qos"]["camera_qos"], "reliable:5")
        for topic in ("/camera/depth/image_raw", "/camera/color/image_raw"):
            vp = rx["topics"][topic]["vs_publisher"]
            inw = [s for s in stamps if t0 + 10 ** 9 <= s <= t0 + 10 * 10 ** 9]
            self.assertEqual(vp["published"], len(inw), topic)
            self.assertEqual(vp["missing"], 3, topic)
            self.assertEqual(vp["extra_not_in_publisher_record"], 0, topic)


class GraphParse(unittest.TestCase):
    TEXT = """camera_frame=camera_color_optical_frame driver_pid=1 launch_pid=2
Type: sensor_msgs/msg/Image

Publisher count: 1

Node name: camera
Node namespace: /camera
Topic type: sensor_msgs/msg/Image
Endpoint type: PUBLISHER
QoS profile:
  Reliability: RELIABLE

Subscription count: 1

Node name: fpga_runtime
Node namespace: /
Topic type: sensor_msgs/msg/Image
Endpoint type: SUBSCRIPTION
QoS profile:
  Reliability: BEST_EFFORT

Type: sensor_msgs/msg/CameraInfo

Publisher count: 1

Node name: camera
Node namespace: /camera
Endpoint type: PUBLISHER
QoS profile:
  Reliability: RELIABLE

Subscription count: 1

Node name: fpga_runtime
Endpoint type: SUBSCRIPTION
QoS profile:
  Reliability: RELIABLE
"""

    # board form (10-15 record cam_1630_r1/rig/camera_topics.txt, shortened): rclpy endpoints have no node name,
    # the GID's first 12 bytes name the participant
    BOARD = """camera_frame=camera_color_optical_frame driver_pid=1613 launch_pid=1584
Type: sensor_msgs/msg/Image

Publisher count: 1

Node name: camera
Node namespace: /camera
Topic type: sensor_msgs/msg/Image
Endpoint type: PUBLISHER
GID: 01.0f.b9.5b.4d.06.8f.6c.00.00.00.00.00.00.81.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE

Subscription count: 2

Node name: _NODE_NAME_UNKNOWN_
Node namespace: _NODE_NAMESPACE_UNKNOWN_
Topic type: sensor_msgs/msg/Image
Endpoint type: SUBSCRIPTION
GID: 01.0f.b9.5b.bf.06.00.91.00.00.00.00.00.00.11.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT

Node name: _NODE_NAME_UNKNOWN_
Node namespace: _NODE_NAMESPACE_UNKNOWN_
Topic type: sensor_msgs/msg/Image
Endpoint type: SUBSCRIPTION
GID: 01.0f.b9.5b.c0.06.4d.40.00.00.00.00.00.00.12.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE

Type: sensor_msgs/msg/CameraInfo

Publisher count: 1

Node name: camera
Endpoint type: PUBLISHER
GID: 01.0f.b9.5b.4d.06.8f.6c.00.00.00.00.00.00.91.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE

Subscription count: 2

Node name: _NODE_NAME_UNKNOWN_
Endpoint type: SUBSCRIPTION
GID: 01.0f.b9.5b.c0.06.4d.40.00.00.00.00.00.00.13.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE

Node name: _NODE_NAME_UNKNOWN_
Endpoint type: SUBSCRIPTION
GID: 01.0f.b9.5b.bf.06.00.91.00.00.00.00.00.00.15.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
"""

    def test_board_form_by_participant(self):
        from tools.integ_car_chain.runner_qos_readout import graph_reliability, graph_summary
        p = Path(tempfile.mkdtemp(prefix="graph_")) / "camera_topics.txt"
        p.write_text(self.BOARD)
        g = graph_reliability(str(p))
        self.assertEqual([b["node"] for b in g], [[], []])                 # names unknown: the name check is blind
        self.assertEqual([b["publishers"] for b in g], [["RELIABLE"], ["RELIABLE"]])
        sm = graph_summary(g)
        self.assertEqual(sm["reliable_subscriptions"], [1, 1])
        self.assertEqual(sm["reliable_on_all_topics"], ["01.0f.b9.5b.c0.06.4d.40.00.00.00.00"])
        # control: the best-effort participant subscribes on both topics too, but is not counted
        b = Path(tempfile.mkdtemp(prefix="graph_")) / "camera_topics.txt"
        b.write_text(self.BOARD.replace("Reliability: RELIABLE\n\nType", "Reliability: BEST_EFFORT\n\nType")
                     .replace("13.04.00.00.00.00.00.00.00.00\nQoS profile:\n  Reliability: RELIABLE",
                              "13.04.00.00.00.00.00.00.00.00\nQoS profile:\n  Reliability: BEST_EFFORT"))
        sb = graph_summary(graph_reliability(str(b)))
        self.assertEqual((sb["reliable_subscriptions"], sb["reliable_on_all_topics"]), ([0, 0], []))

    def test_parse(self):
        sys.path.insert(0, str(ROOT))
        from tools.integ_car_chain.runner_qos_readout import graph_reliability
        p = Path(tempfile.mkdtemp(prefix="graph_")) / "camera_topics.txt"
        p.write_text(self.TEXT)
        g = graph_reliability(str(p))
        self.assertEqual([b["reliable_subscriptions"] for b in g], [0, 1])
        self.assertEqual([(b["type"], b["publishers"], b["node"]) for b in g],
                         [("sensor_msgs/msg/Image", ["RELIABLE"], ["BEST_EFFORT"]),
                          ("sensor_msgs/msg/CameraInfo", ["RELIABLE"], ["RELIABLE"])])


V8RT = ROOT / "tests" / "data" / "camera_topics_v8rt_20261015"


def endpoint(node, kind, gid, rel, typ="sensor_msgs/msg/Image"):
    return ("Node name: %s\nNode namespace: /\nTopic type: %s\nEndpoint type: %s\nGID: %s\nQoS profile:\n"
            "  Reliability: %s\n  History (Depth): UNKNOWN\n  Durability: VOLATILE\n\n" % (node, typ, kind, gid, rel))


def add_to_section(text, i, extra):
    """TEXT with EXTRA appended to its I-th topic section (0-based)."""
    import re
    marks = [m.start() for m in re.finditer(r"(?m)^(?:Type: |Unknown topic )", text)] + [len(text)]
    return text[:marks[i + 1]] + extra + text[marks[i + 1]:]


RUNNER_COLOUR_REL = endpoint("_NODE_NAME_UNKNOWN_", "SUBSCRIPTION",
                             "01.0f.b9.5b.3f.00.56.cd.00.00.00.00.00.00.22.04.00.00.00.00.00.00.00.00", "RELIABLE")
DRIVER_DEPTH_PUB = endpoint("camera", "PUBLISHER",
                            "01.0f.b9.5b.c6.00.7e.10.00.00.00.00.00.00.81.03.00.00.00.00.00.00.00.00", "RELIABLE")


def rig_dir(text, wait=None):
    d = Path(tempfile.mkdtemp(prefix="rig_"))
    (d / "camera_topics.txt").write_text(text)
    if wait is not None:
        (d / "camera_topics_wait.json").write_text(json.dumps(wait))
    return str(d)


class GraphSnapshotV8rt(unittest.TestCase):
    def setUp(self):
        from tools.integ_car_chain import runner_qos_readout as R
        self.R = R
        self.txt = {a: (V8RT / ("%s.txt" % a)).read_text() for a in ("b1", "r1", "r2", "b2")}
        self.qos = {"b1": "sensor_data", "r1": "reliable:5", "r2": "reliable:5", "b2": "sensor_data"}

    def test_real_v8rt_snapshots_all_invalid(self):
        why = {"b1": "topic 1 (sensor_msgs/msg/Image) has no publisher",
               "r1": "runner RELIABLE subscriptions per topic [1, 0, 1]",
               "r2": "topic 2 (/camera/color/image_raw) unknown to the graph",
               "b2": "topic 2 (/camera/color/image_raw) unknown to the graph"}
        for arm, text in self.txt.items():
            g = self.R.graph_check(rig_dir(text), self.qos[arm])
            self.assertFalse(g["valid"], arm)
            self.assertTrue(any(why[arm] in r for r in g["reasons"]), (arm, g["reasons"]))
            self.assertIn("no wait record", g["note"])
            self.assertNotIn("error", g)                      # three sections each (one of them unknown)

    def test_old_parser_counted_two_topics(self):
        # control: what the readout did before (split at "Type: " only) -- two topics, the third silently gone
        import re
        for arm in ("r2", "b2"):
            self.assertEqual(len(re.split(r"(?m)^Type: ", self.txt[arm])[1:]), 2, arm)
            blocks = self.R.graph_reliability(str(V8RT / ("%s.txt" % arm)))
            self.assertEqual(len(blocks), 3)
            self.assertEqual([b["unknown"] for b in blocks], [False, True, False])
            self.assertEqual(blocks[1]["topic"], "/camera/color/image_raw")

    def test_two_sections_is_an_error(self):
        two = "\n".join(l for l in self.txt["r2"].splitlines() if not l.startswith("Unknown topic")) + "\n"
        blocks = self.R.graph_blocks(two)
        self.assertEqual(len(blocks), 2)
        with self.assertRaises(self.R.GraphSnapshotError):
            self.R.graph_summary(blocks, expected_topics=3)
        self.assertEqual(self.R.graph_summary(blocks)["topics"], 2)          # without the expectation: lenient
        g = self.R.graph_check(rig_dir(two), "reliable:5")
        self.assertFalse(g["valid"])
        self.assertIn("2 topic sections, expected 3", g["error"])
        self.assertIn("2 topic sections, expected 3", g["reasons"])

    def test_cli_exits_3_on_two_sections(self):
        two = "\n".join(l for l in self.txt["r2"].splitlines() if not l.startswith("Unknown topic")) + "\n"
        rig = Path(rig_dir(two))
        res = []
        with mock.patch.object(self.R, "readout", lambda run: res.append(run) or
                               {"run": "x", "validity": {"graph_check": self.R.graph_check(str(rig), "reliable:5")}}), \
                mock.patch("sys.stdout"), mock.patch("sys.stderr") as err:
            self.assertEqual(self.R.main([str(rig.parent)]), 3)
        self.assertIn("ERROR", "".join(c.args[0] for c in err.write.call_args_list))
        with mock.patch.object(self.R, "readout", lambda run: {"run": "x", "validity": {
                "graph_check": self.R.graph_check(rig_dir(self.txt["r1"]), "reliable:5")}}), \
                mock.patch("sys.stdout"), mock.patch("sys.stderr"):
            self.assertEqual(self.R.main(["y"]), 0)            # invalid (r1) but three sections: no error exit

    def test_converged_variants_pass_and_switch_mismatch_fails(self):
        r_ok = add_to_section(self.txt["r1"], 1, RUNNER_COLOUR_REL)
        g = self.R.graph_check(rig_dir(r_ok), "reliable:5")
        self.assertTrue(g["valid"], g["reasons"])
        self.assertEqual(self.R.graph_summary(self.R.graph_blocks(r_ok))["reliable_on_all_topics"],
                         ["01.0f.b9.5b.3f.00.56.cd.00.00.00.00"])
        self.assertFalse(self.R.graph_check(rig_dir(r_ok), "sensor_data")["valid"])   # b arm must see none
        b_ok = add_to_section(self.txt["b1"], 0, DRIVER_DEPTH_PUB)
        self.assertTrue(self.R.graph_check(rig_dir(b_ok), "sensor_data")["valid"])
        self.assertFalse(self.R.graph_check(rig_dir(b_ok), "reliable:5")["valid"])    # r arm needs its runner
        self.assertTrue(self.R.graph_check(rig_dir(b_ok), None)["valid"])             # no runner: publishers only

    def test_timed_out_wait_record_voids_the_arm(self):
        r_ok = add_to_section(self.txt["r1"], 1, RUNNER_COLOUR_REL)
        ok = {"ready": True, "timed_out": False, "waited_s": 4.2, "expect": "reliable", "unmet": []}
        self.assertTrue(self.R.graph_check(rig_dir(r_ok, ok), "reliable:5")["valid"])
        late = dict(ok, ready=False, timed_out=True, waited_s=30.1, unmet=["topic 2 (x) unknown to the graph"])
        g = self.R.graph_check(rig_dir(r_ok, late), "reliable:5")
        self.assertFalse(g["valid"])
        self.assertIn("snapshot timed out after 30.1 s", g["reasons"][0])
        self.assertIn("unknown to the graph", g["reasons"][0])
        g = self.R.graph_check(rig_dir(r_ok, dict(ok, expect="best_effort")), "reliable:5")
        self.assertFalse(g["valid"])                           # waited for the other switch


class GraphSnapshotPoller(unittest.TestCase):
    TOPICS = ["/camera/depth/image_raw", "/camera/color/image_raw", "/camera/depth/camera_info"]

    def setUp(self):
        sys.path.insert(0, str(ROOT / "tools" / "integ_car_chain"))
        from tools.integ_car_chain import graph_snapshot
        self.G = graph_snapshot
        self.r1 = (V8RT / "r1.txt").read_text()
        self.r_ok = add_to_section(self.r1, 1, RUNNER_COLOUR_REL)
        self.r2 = (V8RT / "r2.txt").read_text()

    def clock(self, step):
        t = [0.0]

        def now():
            t[0] += step
            return t[0]
        return now

    def test_polls_until_converged(self):
        seq = [self.r2, self.r1, self.r_ok, self.r2]
        with mock.patch.object(self.G, "take", side_effect=lambda *a: seq.pop(0)):
            text, rec = self.G.poll(self.TOPICS, "reliable", 30.0, 1.0, ["x"], clock=self.clock(1.0),
                                    sleep=lambda s: None)
        self.assertEqual(text, self.r_ok)
        self.assertEqual((rec["ready"], rec["timed_out"], rec["attempts"], rec["unmet"]), (True, False, 3, []))

    def test_timeout_keeps_last_snapshot_and_says_why(self):
        with mock.patch.object(self.G, "take", side_effect=lambda *a: self.r1):
            text, rec = self.G.poll(self.TOPICS, "reliable", 5.0, 1.0, ["x"], clock=self.clock(1.0),
                                    sleep=lambda s: None)
        self.assertEqual(text, self.r1)
        self.assertTrue(rec["timed_out"])
        self.assertFalse(rec["ready"])
        self.assertIn("[1, 0, 1]", rec["unmet"][0])
        # control: the same r1 text is "ready" for an arm without a runner (publishers are all there)
        with mock.patch.object(self.G, "take", side_effect=lambda *a: self.r1):
            self.assertTrue(self.G.poll(self.TOPICS, "none", 5.0, 1.0, ["x"], clock=self.clock(1.0),
                                        sleep=lambda s: None)[1]["ready"])

    def test_four_real_arms_never_ready_and_voided(self):
        """Each of the four real v8rt snapshots, served unchanged by the CLI until the timeout: the poller never
        calls it ready (each for that arm's own reason) and the readout voids the arm on the wait record."""
        cases = {"b1": ("best_effort", "sensor_data", "topic 1 (sensor_msgs/msg/Image) has no publisher"),
                 "r1": ("reliable", "reliable:5", "runner RELIABLE subscriptions per topic [1, 0, 1]"),
                 "r2": ("reliable", "reliable:5", "topic 1 (sensor_msgs/msg/Image) has no publisher"),
                 "b2": ("best_effort", "sensor_data", "topic 2 (/camera/color/image_raw) unknown to the graph")}
        from tools.integ_car_chain.runner_qos_readout import graph_check
        for arm, (expect, qos, why) in cases.items():
            text = (V8RT / ("%s.txt" % arm)).read_text()
            with mock.patch.object(self.G, "take", side_effect=lambda *a: text):
                last, rec = self.G.poll(self.TOPICS, expect, 3.0, 1.0, ["x"], clock=self.clock(1.0),
                                        sleep=lambda s: None)
            self.assertEqual(last, text, arm)
            self.assertEqual((rec["ready"], rec["timed_out"]), (False, True), arm)
            self.assertIn(why, rec["unmet"][0], (arm, rec["unmet"]))
            g = graph_check(rig_dir(text, rec), qos)
            self.assertFalse(g["valid"], arm)
            self.assertTrue(g["reasons"][0].startswith("snapshot timed out after"), (arm, g["reasons"]))

    def test_b_arm_converges(self):
        b1 = (V8RT / "b1.txt").read_text()
        b_ok = add_to_section(b1, 0, DRIVER_DEPTH_PUB)
        seq = [b1, b_ok]
        with mock.patch.object(self.G, "take", side_effect=lambda *a: seq.pop(0)):
            text, rec = self.G.poll(self.TOPICS, "best_effort", 30.0, 1.0, ["x"], clock=self.clock(1.0),
                                    sleep=lambda s: None)
        self.assertEqual(text, b_ok)
        self.assertEqual((rec["ready"], rec["timed_out"], rec["attempts"]), (True, False, 2))

    def test_main_end_to_end_with_fake_cli(self):
        """The real subprocess path: a fake ``ros2 topic info -v`` answering per topic, unconverged on the first
        round (r2 form), converged on the second; expectation read from runtime_info.json; the readout then
        accepts the snapshot and its wait record."""
        import re
        d = Path(tempfile.mkdtemp(prefix="gsnap_"))

        def sections(text):
            marks = [m.start() for m in re.finditer(r"(?m)^(?:Type: |Unknown topic )", text)] + [len(text)]
            return [text[a:b] for a, b in zip(marks, marks[1:])]
        for rnd, text in ((1, self.r2), (2, self.r_ok)):
            for i, sec in enumerate(sections(text)):
                (d / ("%d_%d.txt" % (rnd, i))).write_text(sec)
        fake = d / "fake_ros2.py"
        fake.write_text("import os, sys\nd, topic = sys.argv[1], sys.argv[2]\n"
                        "i = %r.index(topic)\nc = os.path.join(d, 'n_%%d' %% i)\n"
                        "n = (int(open(c).read()) if os.path.exists(c) else 0) + 1\n"
                        "open(c, 'w').write(str(n))\n"
                        "sys.stdout.write(open(os.path.join(d, '%%d_%%d.txt' %% (min(n, 2), i))).read())\n"
                        % (self.TOPICS,))
        rt = d / "runtime_info.json"
        rt.write_text(json.dumps({"camera_qos": "reliable:5"}))
        rig = d / "rig"
        rig.mkdir()
        argv = []
        for t in self.TOPICS:
            argv += ["--topic", t]
        argv += ["--out", str(rig / "camera_topics.txt"), "--wait-json", str(rig / "camera_topics_wait.json"),
                 "--runtime-info", str(rt), "--header", "camera_frame=cf driver_pid=1 launch_pid=2",
                 "--interval-s", "0", "--timeout-s", "20",
                 "--cmd", "%s %s %s" % (sys.executable, fake, d)]
        with mock.patch("sys.stdout"):
            self.assertEqual(self.G.main(argv), 0)
        rec = json.loads((rig / "camera_topics_wait.json").read_text())
        self.assertEqual((rec["ready"], rec["timed_out"], rec["attempts"], rec["expect"]), (True, False, 2, "reliable"))
        self.assertIn("camera_qos=reliable:5", rec["expect_from"])
        self.assertTrue((rig / "camera_topics.txt").read_text().startswith("camera_frame=cf driver_pid=1"))
        from tools.integ_car_chain.runner_qos_readout import graph_check
        g = graph_check(str(rig), "reliable:5")
        self.assertTrue(g["valid"], g["reasons"])
        # no runtime_info (no runner in the chain) -> expect none
        with mock.patch("sys.stdout"):
            self.G.main(argv[:argv.index("--runtime-info")] + argv[argv.index("--runtime-info") + 2:]
                        + ["--runtime-info", str(d / "absent.json")])
        self.assertEqual(json.loads((rig / "camera_topics_wait.json").read_text())["expect"], "none")

    def test_needs_three_topics(self):
        with mock.patch("sys.stderr"), self.assertRaises(SystemExit):
            self.G.main(["--topic", "a", "--topic", "b", "--out", "x", "--wait-json", "y"])


try:
    import rclpy  # noqa: F401
    import rclpy.qos  # noqa: F401
    HAVE_ROS = hasattr(rclpy.qos, "qos_check_compatible")
except ImportError:
    HAVE_ROS = False


@unittest.skipUnless(HAVE_ROS, "needs rclpy with qos_check_compatible (ROS 2 image)")
class WithRos(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        import rclpy
        rclpy.shutdown()

    def test_compatibility_known_answers(self):
        from rclpy.qos import QoSCompatibility, qos_check_compatible, qos_profile_sensor_data
        reliable = ros_runtime.runner_camera_qos(True, 5)
        writer_reliable = ros_runtime.runner_camera_qos(True, 5)
        # positive control: a reliable reader on a best-effort writer (replay camera / sim) does not match
        self.assertEqual(qos_check_compatible(qos_profile_sensor_data, reliable)[0], QoSCompatibility.ERROR)
        # the reliable reader on the driver's RELIABLE writer, and best effort on either, do (Humble answers
        # WARNING for these: "liveliness is system_default" on both sides, a can't-tell, not a reliability verdict)
        for w, r in ((writer_reliable, reliable), (writer_reliable, qos_profile_sensor_data),
                     (qos_profile_sensor_data, qos_profile_sensor_data)):
            verdict, reason = qos_check_compatible(w, r)
            self.assertNotEqual(verdict, QoSCompatibility.ERROR, reason)
            self.assertNotIn("reliab", reason.lower())

    def test_own_subscriptions_and_message_filters_kwargs(self):
        from rclpy.node import Node
        from rclpy.qos import ReliabilityPolicy
        from rclpy.qos_event import SubscriptionEventCallbacks
        from message_filters import Subscriber
        from sensor_msgs.msg import Image
        node = Node("runner_camqos_%d" % os.getpid())
        try:
            for reliable, topic in ((True, "/camqos_test/rel"), (False, "/camqos_test/be")):
                qos = ros_runtime.runner_camera_qos(reliable, 5)
                Subscriber(node, Image, topic, qos_profile=qos,
                           event_callbacks=SubscriptionEventCallbacks(message_lost=lambda s: None))
                infos = node.get_subscriptions_info_by_topic(topic)
                self.assertEqual(len(infos), 1)
                self.assertEqual(infos[0].qos_profile.reliability,
                                 ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT)
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
