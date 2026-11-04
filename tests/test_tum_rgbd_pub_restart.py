"""tum_rgbd_pub --out: a restarted publisher starts again on the same file (2026-10-15 第 5 棒续).

Host g5r_1 (gate_dig/HOST_CORRECTION_PREREG.md 23:25 record): the first camera start timed out, and
the 9 restarts by tools/car_run/rtabmap_car.sh all exited at once because --out was opened with 'x'
and the file already existed. Now --out is appended to; every session begins with a `start` row
(`session`, `pid`) and readers take the last session (last_session()).

main() runs against stub ROS / OpenCV modules: session 1 sees no subscriber (startup_timeout, as on
g5r_1), session 2 sees them and publishes two frames. Positive control: the f4a273f publisher on the
same driver raises FileExistsError at the second start. Python 3.8+.
"""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

PREV = "f4a273f"
REL = "tools/nano_ros2/tum_rgbd_pub.py"


def source_at(rev):
    try:
        return subprocess.check_output(["git", "show", "%s:%s" % (rev, REL)], cwd=str(ROOT),
                                       stderr=subprocess.DEVNULL).decode("utf-8")
    except (OSError, subprocess.CalledProcessError):
        return None


def load(source, name):
    module = types.ModuleType(name)
    module.__file__ = str(ROOT / REL)
    exec(compile(source, str(ROOT / REL), "exec"), module.__dict__)
    return module


class Msg:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0), frame_id="")


def stub_modules(subscribers):
    """sys.modules entries for rclpy / messages / cv2; `subscribers` decides startup."""
    class Pub:
        def __init__(self):
            self.sent = 0

        def get_subscription_count(self):
            return 1 if subscribers else 0

        def publish(self, message):
            self.sent += 1

    class Node:
        def create_publisher(self, *a):
            return Pub()

        def create_subscription(self, *a):
            return None

        def count_publishers(self, topic):
            return 1 if subscribers else 0

        def destroy_node(self):
            pass

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    rclpy.create_node = lambda name: Node()
    rclpy.spin_once = lambda node, timeout_sec=0.: None
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kw: None
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    rclpy.qos = qos
    mods = {"rclpy": rclpy, "rclpy.qos": qos}
    for pkg, names in (("sensor_msgs", ("Image", "CameraInfo")), ("nav_msgs", ("Odometry",)),
                       ("geometry_msgs", ("PoseWithCovarianceStamped",)), ("rtabmap_msgs", ("OdomInfo", "Info"))):
        top, msg = types.ModuleType(pkg), types.ModuleType(pkg + ".msg")
        for n in names:
            setattr(msg, n, type(n, (Msg,), {}))
        top.msg = msg
        mods[pkg], mods[pkg + ".msg"] = top, msg
    cv2 = types.ModuleType("cv2")
    cv2.IMREAD_COLOR, cv2.IMREAD_UNCHANGED = 1, -1
    cv2.imread = lambda path, flag: (np.zeros((4, 6, 3), np.uint8) if flag == 1 else np.full((4, 6), 5000, np.uint16))
    mods["cv2"] = cv2
    return mods


def make_seq(root):
    seq = Path(root) / "seq"
    seq.mkdir()
    (seq / "rgb.txt").write_text("# rgb\n100.000 rgb/a.png\n100.033 rgb/b.png\n")
    (seq / "depth.txt").write_text("# depth\n100.001 depth/a.png\n100.034 depth/b.png\n")
    return seq


def run(module, seq, out, subscribers):
    argv = ["tum_rgbd_pub.py", str(seq), "--out", str(out), "--startup-timeout", "0.05", "--drain", "0",
            "--stamp-now", "--no-subscribe"]
    with mock.patch.dict(sys.modules, stub_modules(subscribers)), mock.patch.object(sys, "argv", argv):
        return module.main()


def rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines()]


class RestartTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.seq = make_seq(self.tmp.name)
        self.out = Path(self.tmp.name) / "pub_events.jsonl"

    def test_restart_on_existing_out_starts_and_last_session_is_the_live_one(self):
        from tools.nano_ros2 import tum_rgbd_pub as pub
        self.assertEqual(run(pub, self.seq, self.out, subscribers=False), 2)     # g5r_1 first start
        self.assertEqual(run(pub, self.seq, self.out, subscribers=False), 2)     # a restart starts again
        self.assertEqual(run(pub, self.seq, self.out, subscribers=True), 0)      # and the live one runs
        everything = rows(self.out)
        starts = [r for r in everything if r["event"] == "start"]
        self.assertEqual([r["session"] for r in starts], [0, 1, 2])
        self.assertTrue(all(isinstance(r["pid"], int) for r in starts))
        self.assertEqual(sum(r["event"] == "startup_timeout" for r in everything), 2)
        last = pub.last_session(everything)
        self.assertEqual(last[0]["session"], 2)
        self.assertEqual([r["event"] for r in last if r["event"] != "stamp_now"],
                         ["start", "ready", "pub", "pub", "complete"])
        self.assertNotIn("startup_timeout", [r["event"] for r in last])

    def test_last_session_of_a_single_session_file_is_the_whole_file(self):
        from tools.nano_ros2 import tum_rgbd_pub as pub
        self.assertEqual(run(pub, self.seq, self.out, subscribers=True), 0)
        self.assertEqual(pub.last_session(rows(self.out)), rows(self.out))
        self.assertEqual(pub.last_session([{"event": "pub"}]), [{"event": "pub"}])   # no start row: all
        self.assertEqual(pub.count_sessions(self.out), 1)
        self.assertEqual(pub.count_sessions(Path(self.tmp.name) / "absent.jsonl"), 0)

    @unittest.skipIf(source_at(PREV) is None, "git history with %s not available" % PREV)
    def test_positive_control_previous_publisher_cannot_restart(self):
        old = load(source_at(PREV), "tum_rgbd_pub_prev")
        self.assertEqual(run(old, self.seq, self.out, subscribers=False), 2)
        with self.assertRaises(FileExistsError):
            run(old, self.seq, self.out, subscribers=True)


if __name__ == "__main__":
    unittest.main()
