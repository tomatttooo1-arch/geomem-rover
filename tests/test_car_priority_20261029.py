"""10-29 car stack CPU priorities (tools/car_run/car_priority.py): which processes get which nice value, every thread,
refusals logged once, the summary line car_up writes after READY. A fake /proc tree; setpriority is injected (no
real process is touched). The argv lists are the stack's own command lines (tools/car_run/car_inner.sh launch lines,
ui/geomem/tools/competition/start_live_mapping.sh, the Orbbec ros2 launch)."""
import errno
import io
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.car_run import car_priority as cp  # noqa: E402

LAUNCH = ["/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch", "orbbec_camera", "gemini2.launch.py",
          "color_width:=640", "color_fps:=15"]
CONTAINER = ["/opt/ros/humble/lib/rclcpp_components/component_container", "--ros-args", "-r",
             "__node:=camera_container", "-r", "__ns:=/camera"]
CONTAINER_BARE = ["/opt/ros/humble/lib/rclcpp_components/component_container", "--ros-args"]
STACK = {
    "runtime": ["python3", "-m", "host.ros_runtime", "--pose-source", "odom", "--camera-fps", "15"],
    "wheel_odom": ["python3", "-m", "host.ros_wheel_odom", "--source", "chassis", "--tf-topic", "/tf"],
    "chassis_driver": ["python3", "-m", "host.ros_chassis_driver", "--port", "/dev/fpga_chassis"],
    "hub": ["python3", "-m", "host.hub.hub_ros", "--data-source", "live", "--port", "8765"],
    "preview": ["python3", "tools/car_run/preview_publisher.py"],
    "colour": ["python3", "tools/kv_link/nano_colour.py", "--kv", "198.51.100.79:47801", "--topic",
               "/camera/color/image_raw", "--lazy"],
    "map_display": ["python3", "/data/fpga/rtabmap/operator-live/r1/live_mapping.py", "--session", "r1"],
    "map_assembler": ["/opt/ros/humble/lib/rtabmap_util/map_assembler", "--ros-args", "-r", "__ns:=/operator_map"],
    "map_gate": ["python3", "/data/fpga/rtabmap/operator-live/r1/map_data_gate.py"],
}
LEFT_ALONE = [
    ["python3", "host/ros_rolling_nav.py", "--ros-args", "-p", "ground_z_m:=0.0"],     # car_inner "bridge" = avoidance
    ["python3", "-m", "host.ros_geo_producer", "--rate-hz", "30"],
    ["python3", "-m", "host.ros_runtime_info_x"],
    ["python3", "tools/car_run/depth_probe.py", "--topic", "/camera/depth/image_raw"],
    ["python3", "tools/car_run/camera_check.py", "--color-topic", "/camera/color/image_raw"],
    ["/opt/ros/humble/lib/rtabmap_slam/rtabmap", "--ros-args"],
    ["/opt/ros/humble/lib/nav2_controller/controller_server", "--ros-args"],
    ["bash", "/src/tools/car_run/car_inner.sh"],
    ["python3", "-m", "host.ros_nav_global_map"],
    ["python3", "tools/car_run/mapping_ctl.py", "--status-file", "/o/mapping_status.json"],
    [],
]


class TestClassify(unittest.TestCase):
    def test_stack_roles(self):
        for role, argv in STACK.items():
            got = cp.classify(argv)
            self.assertIsNotNone(got, role)
            self.assertEqual(got[0], role)
            want = {"runtime": -5, "wheel_odom": -5, "chassis_driver": -5, "hub": 5}.get(role, 10)
            self.assertEqual(got[1], want, role)

    def test_camera(self):
        self.assertEqual(cp.classify(LAUNCH), ("camera_launch", -5))
        self.assertEqual(cp.classify(CONTAINER, [LAUNCH]), ("camera", -5))
        self.assertEqual(cp.classify(CONTAINER), ("camera", -5))                 # by its node name alone
        self.assertEqual(cp.classify(CONTAINER_BARE, [LAUNCH]), ("camera", -5))  # by its launch parent alone
        nav_launch = ["/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch", "fpga_sim", "navigation.launch.py"]
        self.assertIsNone(cp.classify(CONTAINER_BARE, [nav_launch]))            # a Nav2 container stays 0
        self.assertIsNone(cp.classify(CONTAINER_BARE))
        self.assertIsNone(cp.classify(nav_launch))

    def test_left_alone(self):
        for argv in LEFT_ALONE:
            self.assertIsNone(cp.classify(argv), argv)


class FakeProc:
    def __init__(self):
        self.root = tempfile.mkdtemp(prefix="prio_proc_")
        self.nice = {}

    def add(self, pid, argv, ppid=1, threads=1, start="100"):
        d = os.path.join(self.root, str(pid))
        os.makedirs(os.path.join(d, "task"), exist_ok=True)
        with open(os.path.join(d, "cmdline"), "wb") as f:
            f.write(b"\0".join(a.encode() for a in argv) + (b"\0" if argv else b""))
        fields = ["S", str(ppid)] + ["0"] * 17 + [start] + ["0"] * 10
        with open(os.path.join(d, "stat"), "w") as f:
            f.write("%d (x y) %s\n" % (pid, " ".join(fields)))
        tids = [pid] + [pid * 100 + i for i in range(1, threads)]
        for t in tids:
            os.makedirs(os.path.join(d, "task", str(t)), exist_ok=True)
            self.nice.setdefault(t, 0)
        return tids

    def remove(self, pid):
        shutil.rmtree(os.path.join(self.root, str(pid)))


class TestScan(unittest.TestCase):
    def setUp(self):
        self.fp = FakeProc()
        self.out = io.StringIO()
        self.refuse = set()

        def setter(tid, n):
            if tid in self.refuse and n < 0:
                raise OSError(errno.EPERM, "Operation not permitted")
            self.fp.nice[tid] = n
        self.p = cp.Prio(self.fp.root, setter=setter, getter=lambda t: self.fp.nice[t], out=self.out)

    def tearDown(self):
        shutil.rmtree(self.fp.root, ignore_errors=True)

    def rows(self, ev=None):
        rows = [json.loads(l) for l in self.out.getvalue().splitlines()]
        return [r for r in rows if ev is None or r["ev"] == ev]

    def test_every_thread_set_others_untouched_late_threads_and_restart(self):
        rt = self.fp.add(10, STACK["runtime"], threads=4)
        mp = self.fp.add(11, STACK["map_display"], threads=3)
        hub = self.fp.add(12, STACK["hub"], threads=2)
        br = self.fp.add(13, LEFT_ALONE[0], threads=3)
        la = self.fp.add(20, LAUNCH, threads=2)
        cam = self.fp.add(21, CONTAINER_BARE, ppid=20, threads=6)
        self.p.scan()
        self.assertEqual({self.fp.nice[t] for t in rt}, {-5})
        self.assertEqual({self.fp.nice[t] for t in mp}, {10})
        self.assertEqual({self.fp.nice[t] for t in hub}, {5})
        self.assertEqual({self.fp.nice[t] for t in br}, {0})               # navigation avoidance never lowered
        self.assertEqual({self.fp.nice[t] for t in la + cam}, {-5})
        applied = {r["role"]: r for r in self.rows("apply")}
        self.assertEqual(set(applied), {"runtime", "map_display", "hub", "camera_launch", "camera"})
        self.assertEqual(applied["camera"]["threads"], 6)
        n_apply = len(self.rows("apply"))
        # a second scan: nothing to change, no new rows
        sets = self.p.sets
        self.p.scan()
        self.assertEqual((self.p.sets, len(self.rows("apply"))), (sets, n_apply))
        # a late thread (created at nice 0 by a thread someone reset) is caught on the next scan, no new apply row
        os.makedirs(os.path.join(self.fp.root, "10", "task", "1099"))
        self.fp.nice[1099] = 0
        self.p.scan()
        self.assertEqual(self.fp.nice[1099], -5)
        self.assertEqual(len(self.rows("apply")), n_apply)
        # a restarted runner (same pid number, new start time) is applied again and logged
        self.fp.remove(10)
        self.fp.add(10, STACK["runtime"], threads=2, start="999")
        for t in (10, 1001):
            self.fp.nice[t] = 0
        self.p.scan()
        self.assertEqual((self.fp.nice[10], self.fp.nice[1001]), (-5, -5))
        self.assertEqual(len(self.rows("apply")), n_apply + 1)
        # a process that ended leaves the summary
        self.fp.remove(11)
        self.p.scan()
        self.p.summary()
        roles = self.rows("summary")[-1]["roles"]
        self.assertNotIn("map_display", roles)
        self.assertEqual(roles["runtime"], [{"nice": -5, "pid": 10, "threads": 2}])

    def test_refusal_logged_once_and_the_rest_goes_on(self):
        rt = self.fp.add(10, STACK["runtime"], threads=2)
        pv = self.fp.add(11, STACK["preview"], threads=2)
        self.refuse.update(rt)
        for _ in range(3):
            self.p.scan()
        errs = self.rows("error")
        self.assertEqual(len(errs), 1)
        self.assertEqual((errs[0]["role"], errs[0]["errno"]), ("runtime", errno.EPERM))
        self.assertEqual({self.fp.nice[t] for t in pv}, {10})
        self.assertEqual([r["role"] for r in self.rows("apply")], ["preview"])

    def test_summarize_line(self):
        self.fp.add(10, STACK["runtime"], threads=2)
        self.fp.add(11, STACK["hub"])
        self.fp.add(12, STACK["preview"])
        self.out.write(json.dumps({"ev": "start", "euid": 0}) + "\n")
        self.p.scan()
        self.p.summary()
        log = os.path.join(self.fp.root, "priority.log")
        with open(log, "w") as f:
            f.write(self.out.getvalue())
        line = cp.summarize(log)
        self.assertEqual(line, "priority: hub +5(pid 11); preview +10(pid 12); runtime -5(pid 10)")
        with open(log, "w") as f:
            f.write("Error response from daemon: container is not running\n")
        self.assertIn("not running (Error response", cp.summarize(log))
        self.assertIn("no log", cp.summarize(log + ".absent"))

    def test_once_on_this_machine_proc_runs(self):
        """the real /proc of the test host: one scan with a no-op setter must not raise"""
        p = cp.Prio("/proc", setter=lambda t, n: None, getter=lambda t: 0, out=io.StringIO())
        p.scan()
        p.summary()


if __name__ == "__main__":
    unittest.main()
