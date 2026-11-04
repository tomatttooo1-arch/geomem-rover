"""2026-10-26: the retry's costmap-clear service names match the car's Nav2 (no namespace). Host-only, no ROS.

host/ros_runtime.py clear_costmaps() calls /global_costmap/clear_entirely_global_costmap and
/local_costmap/clear_entirely_local_costmap (absolute). That is right only while Nav2 runs in the root namespace:
- robot_sim/fpga_sim/launch/navigation.launch.py includes Humble nav2_bringup navigation_launch.py without a
  "namespace" launch argument and without PushRosNamespace (default namespace '');
- tools/car_run/car_inner.sh launches that file (board: by path) with no namespace argument;
- robot_sim/tests/nav_startup_probe.py talks to "/<node>/get_state" (absolute, root) and on the car activated all
  eight Nav2 nodes (docs/design/REAL_CAR_NAV_GOLIVE_20261016.md D1), which it could not do under a namespace; the same
  run saw the subscriber /local_costmap/local_costmap (D3), i.e. the costmap node in /local_costmap, not /<ns>/...
These tests lock that chain: adding a namespace to the launch (or to car_inner) without changing clear_costmaps()
fails here instead of silently recording costmap_clear_skipped on the car.
"""
import re
import unittest
from pathlib import Path

from host import nav_recovery

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "robot_sim/fpga_sim/launch/navigation.launch.py"
INNER = ROOT / "tools/car_run/car_inner.sh"
RUNTIME = ROOT / "host/ros_runtime.py"
PROBE = ROOT / "robot_sim/tests/nav_startup_probe.py"
CLEAR = ("/global_costmap/clear_entirely_global_costmap", "/local_costmap/clear_entirely_local_costmap")


class CostmapClearNames(unittest.TestCase):
    def test_launch_has_no_namespace(self):
        src = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('navigation_launch.py', src)
        args = src[src.index("launch_arguments="):]
        args = args[:args.index(".items()")]
        self.assertNotIn("namespace", args)
        self.assertNotIn("PushRosNamespace", src)
        self.assertNotIn("namespace=", src)

    def test_car_inner_passes_no_namespace(self):
        src = INNER.read_text(encoding="utf-8")
        navl = [l for l in src.splitlines() if re.match(r"\s*NAVL=", l)]
        self.assertEqual(len(navl), 2, navl)   # host (package) and board (by path)
        for line in navl:
            self.assertNotIn("namespace", line)
        self.assertNotRegex(src, r"FPGA_NAV\w*NAMESPACE|namespace:=")

    def test_runtime_names_are_root_namespace_and_match_the_bt(self):
        src = RUNTIME.read_text(encoding="utf-8")
        body = src[src.index("def clear_costmaps(self):"):]
        body = body[:body.index("\n        def ", 1)]
        self.assertEqual(tuple(re.findall(r'"(/[a-z_]+/clear_entirely_[a-z_]+)"', body)), CLEAR)
        # the same services Humble's default BT (bt_navigator, root namespace) names relatively (host/nav_recovery.py)
        names = set(re.findall(r"'([a-z_]+/clear_entirely_[a-z_]+)'", Path(nav_recovery.__file__).read_text()))
        self.assertEqual({"/" + n for n in names}, set(CLEAR))

    def test_startup_probe_uses_root_namespace(self):
        src = PROBE.read_text(encoding="utf-8")
        self.assertIn('"/" + name + "/get_state"', src)
        self.assertIn('"/" + name + "/manage_nodes"', src)


if __name__ == "__main__":
    unittest.main()
