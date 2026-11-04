"""2026-10-22 find-nav dig: the runner's two per-goal clocks share one budget.

host/ros_runtime.py ended goals after 30 s twice over -- the wall-clock watchdog (navigation_wall_timeout) and the
task controller's navigating clock (TaskConfig.navigation_timeout_s, ROS time) -- and the task after 120 s. Both
per-goal clocks now read --navigation-timeout-s, the task --task-timeout-s; the defaults stay 30 / 120 (simulation,
tests); tools/car_run (board) passes 60 / 180 (tests/test_car_run.py checks the plan). Host-only: no ROS is started.
"""
import argparse
from pathlib import Path
import types
import unittest
from unittest import mock

from host import ros_runtime
from host.task_control import TaskConfig

ROOT = Path(__file__).resolve().parents[1]


class _Parsed(Exception):
    pass


def parsed(argv):
    """main()'s own argparse result (args, ros_args), captured before main imports rclpy (and before its checks)."""
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


BASE = ["--socket", "/tmp/x.sock", "--pose-source", "reference", "--sensor-source", "hardware"]


class RuntimeTimeouts(unittest.TestCase):
    def test_defaults_are_the_simulations(self):
        args, _ = parsed(BASE)
        self.assertEqual((args.navigation_timeout_s, args.task_timeout_s), (30.0, 120.0))
        self.assertEqual((TaskConfig().navigation_timeout_s, TaskConfig().task_timeout_s), (30.0, 120.0))
        config = ros_runtime.task_config(args)
        self.assertEqual((config.navigation_timeout_s, config.task_timeout_s), (30.0, 120.0))
        self.assertEqual(ros_runtime.task_config(types.SimpleNamespace()), TaskConfig())   # namespace without them

    def test_car_values_reach_the_task_config(self):
        args, _ = parsed(BASE + ["--navigation-timeout-s", "60", "--task-timeout-s", "180"])
        config = ros_runtime.task_config(args)
        self.assertEqual((config.navigation_timeout_s, config.task_timeout_s), (60.0, 180.0))
        self.assertEqual((config.observation_timeout_s, config.stream_timeout_s), (3.0, 2.0))   # others unchanged

    def test_bad_values_refused_before_anything_starts(self):
        for flag in ("--navigation-timeout-s", "--task-timeout-s"):
            for value in ("0", "-5", "nan", "inf"):
                with self.subTest(flag=flag, value=value), self.assertRaises(SystemExit) as stop, \
                        mock.patch("sys.stderr"):
                    ros_runtime.main(BASE + [flag, value])
                self.assertEqual(stop.exception.code, 2)

    def test_both_clocks_read_the_one_value(self):
        """Source check of the node (needs rclpy to run): the wall watchdog compares with the task config's
        navigation budget, the controller is built with that config; no hard-coded 30 s is left."""
        source = (ROOT / "host/ros_runtime.py").read_text()
        # 10-25 find-retry: per attempt min(navigation budget, task time left) for a task goal, else the budget
        self.assertIn('time.monotonic()-self.nav["started_wall"] > self.nav_wall_budget(self.nav)', source)
        self.assertIn("return self.task.attempt_timeout_s\n            return self.task_config.navigation_timeout_s",
                      source)
        self.assertNotIn('self.nav["started_wall"] > 30', source)
        self.assertIn("TaskController(self.memory, self.planner, search, config=self.task_config,", source)
        self.assertIn("self.task_config = task_config(args)", source)



class RuntimeRetry(unittest.TestCase):
    """10-25 find-retry: the retry options, the fault-injection switch and what the events record."""

    def test_retry_defaults_and_flags_reach_the_task_config(self):
        args, _ = parsed(BASE)
        self.assertIsNone(args.fault_nav_fail_first)       # main() resolves it from the environment after parsing
        config = ros_runtime.task_config(args)
        self.assertEqual((config.nav_retries_same_goal, config.max_nav_failures, config.min_attempt_time_s,
                          config.retry_pause_s, config.fault_nav_fail_first), (1, 3, 20.0, 1.0, 0))
        args, _ = parsed(BASE + ["--nav-retries-same-goal", "2", "--max-nav-attempts", "5",
                                 "--min-attempt-time-s", "15", "--retry-pause-s", "0.5", "--fault-nav-fail-first", "2"])
        config = ros_runtime.task_config(args)
        self.assertEqual((config.nav_retries_same_goal, config.max_nav_failures, config.min_attempt_time_s,
                          config.retry_pause_s, config.fault_nav_fail_first), (2, 5, 15.0, 0.5, 2))
        self.assertEqual(ros_runtime.task_config_record(TaskConfig(navigation_timeout_s=60, task_timeout_s=180)),
                         {"navigation_timeout_s": 60, "task_timeout_s": 180, "stream_timeout_s": 2.0,
                          "nav_retries_same_goal": 1, "max_nav_failures": 3, "min_attempt_time_s": 20.0, "retry_pause_s": 1.0,
                          "fault_nav_fail_first": 0, "not_seen_frames": 5, "observe_min_s": 6.0,
                          "en_route_target_max_distance_m": 1.6, "en_route_max_distance_m": 1.2})

    def test_failure_cap_flag_and_old_alias(self):
        """10-26: --max-nav-failures; the old --max-nav-attempts still parses (car scripts) into the same field."""
        for flag in ("--max-nav-failures", "--max-nav-attempts"):
            with self.subTest(flag=flag):
                args, _ = parsed(BASE + [flag, "5"])
                self.assertEqual(ros_runtime.task_config(args).max_nav_failures, 5)
        self.assertEqual(ros_runtime.task_config(parsed(BASE)[0]).max_nav_failures, 3)

    def test_stream_timeout_flag(self):
        """10-26: --stream-timeout-s reaches the controller and the startup record; default 2.0 (simulation)."""
        self.assertEqual(ros_runtime.task_config(parsed(BASE)[0]).stream_timeout_s, 2.0)
        config = ros_runtime.task_config(parsed(BASE + ["--stream-timeout-s", "5.0"])[0])
        self.assertEqual(config.stream_timeout_s, 5.0)
        self.assertEqual(ros_runtime.task_config_record(config)["stream_timeout_s"], 5.0)

    def test_fault_injection_env_default_off(self):
        self.assertEqual(ros_runtime.fault_nav_default({}), "0")
        self.assertEqual(ros_runtime.fault_nav_default({"FPGA_FAULT_NAV_FAIL_FIRST": " 2 "}), "2")
        with mock.patch.dict("os.environ", {"FPGA_FAULT_NAV_FAIL_FIRST": "nope"}), \
                self.assertRaises(SystemExit) as stop, mock.patch("sys.stderr"):
            ros_runtime.main(BASE)
        self.assertEqual(stop.exception.code, 2)

    def test_bad_retry_values_refused(self):
        for flag, value in (("--max-nav-attempts", "0"), ("--max-nav-failures", "0"),
                            ("--stream-timeout-s", "0"), ("--stream-timeout-s", "nan"), ("--nav-retries-same-goal", "-1"),
                            ("--min-attempt-time-s", "nan"), ("--retry-pause-s", "-1"),
                            ("--fault-nav-fail-first", "-2")):
            with self.subTest(flag=flag), self.assertRaises(SystemExit) as stop, mock.patch("sys.stderr"):
                ros_runtime.main(BASE + [flag, value])
            self.assertEqual(stop.exception.code, 2)

    def test_failure_reason_words(self):
        codes = types.SimpleNamespace(STATUS_ABORTED=6, STATUS_CANCELED=5)
        self.assertEqual([ros_runtime.nav_failure_reason(s, codes) for s in (6, 5, 4)],
                         ["navigation_aborted", "navigation_canceled_by_server", "navigation_failed"])

    def test_node_wiring(self):
        """Source pins (the node needs rclpy): startup records the effective options, task rows carry the query
        and the attempts, failures and wall timeouts of task goals go to the controller, recover is executed."""
        source = (ROOT / "host/ros_runtime.py").read_text()
        self.assertIn("**task_config_record(self.task_config))", source)
        self.assertIn("extra.update(goal=self.task.goal, attempts=self.task.attempts_snapshot())", source)
        self.assertIn('self.task.navigation_result(context["id"], bool(succeeded), self.now(), reason)', source)
        self.assertIn('self.execute(self.task.navigation_result(context["id"], False, self.now(),\n'
                      '                                                                     "navigation_wall_timeout"))',
                      source)
        self.assertIn('elif command.kind == "recover":\n                    self.recover(command)', source)
        for service in ("/global_costmap/clear_entirely_global_costmap", "/local_costmap/clear_entirely_local_costmap"):
            self.assertIn(service, source)



class MapSwitchGate(unittest.TestCase):
    """10-25 (audit nav4): no find/select while the mapping stop saves / aligns / loads the exported map."""

    def test_known_answers(self):
        f = ros_runtime.map_switch_refusal
        R = ros_runtime.MAP_SWITCH_REFUSAL
        cases = (
            (None, None, False, None),                                        # no mapping controller
            ({"state": "mapping"}, .5, False, None),                          # mapping: blank map, finding allowed
            ({"state": "saving"}, .5, False, R),
            ({"state": "exported", "map_loaded": None}, .5, False, R),         # aligning / load_map in flight
            ({"state": "exported", "map_loaded": True}, .5, False, None),
            ({"state": "exported", "map_loaded": False}, .5, False, None),     # load failed: blank map stays
            ({"state": "failed"}, .5, False, None),
            ({"state": "saving"}, 3.5, False, None),                          # controller gone (no 1 s re-send)
            ({"state": "mapping"}, .5, True, R),                              # runner memory alignment pending
            (None, None, True, R),
        )
        for row, age, pending, want in cases:
            with self.subTest(row=row, age=age, pending=pending):
                self.assertEqual(f(row, age, pending), want)
        self.assertEqual(ros_runtime.MAPPING_STATUS_TOPIC, "/fpga/mapping/status")
        source = (ROOT / "tools/car_run/mapping_ctl.py").read_text()
        self.assertIn('STATUS_TOPIC = "/fpga/mapping/status"', source)

    def test_query_and_select_check_it(self):
        source = (ROOT / "host/ros_runtime.py").read_text()
        self.assertIn('if self.map_switching():\n                self.record("query_rejected", reason=MAP_SWITCH_REFUSAL',
                      source)
        self.assertIn('if self.map_switching():\n                self.record("selection_rejected", reason=MAP_SWITCH_REFUSAL)',
                      source)
        self.assertIn("self.create_subscription(String, MAPPING_STATUS_TOPIC, self.on_mapping_status", source)

    def test_sim_loop_passes_fault_injection_only_when_set(self):
        """tools/sim_loop: FPGA_FAULT_NAV_FAIL_FIRST into the stack container and on to the runtime, both behind
        ${VAR:+...} (unset/empty = no argument at all = off)."""
        outer = (ROOT / "tools/sim_loop/run_sim_loop.sh").read_text()
        inner = (ROOT / "tools/sim_loop/stack_inner.sh").read_text()
        self.assertIn('${FPGA_FAULT_NAV_FAIL_FIRST:+-e FPGA_FAULT_NAV_FAIL_FIRST="$FPGA_FAULT_NAV_FAIL_FIRST"}', outer)
        self.assertIn('${FPGA_FAULT_NAV_FAIL_FIRST:+--fault-nav-fail-first "$FPGA_FAULT_NAV_FAIL_FIRST"}', inner)
        import subprocess
        line = 'printf "<%s>" ${FPGA_FAULT_NAV_FAIL_FIRST:+--fault-nav-fail-first "$FPGA_FAULT_NAV_FAIL_FIRST"}'
        for value, want in ((None, "<>"), ("", "<>"), ("2", "<--fault-nav-fail-first><2>")):
            env = {"PATH": "/usr/bin:/bin"}
            if value is not None:
                env["FPGA_FAULT_NAV_FAIL_FIRST"] = value
            out = subprocess.run(["bash", "-c", line], env=env, stdout=subprocess.PIPE, universal_newlines=True).stdout
            self.assertEqual(out, want, value)

    def test_one_mapping_status_handler_serves_gate_and_search_waypoints(self):
        """10-26 merge with search-waypoints: both features read /fpga/mapping/status. One subscription and one
        on_mapping_status (a second def would silently replace the first in the class body); it stores the row for
        the gate before the --search-waypoints-from-mapping early return."""
        source = (ROOT / "host/ros_runtime.py").read_text()
        self.assertEqual(source.count("def on_mapping_status("), 1)
        self.assertEqual(source.count("self.create_subscription(String, MAPPING_STATUS_TOPIC"), 1)
        self.assertEqual(source.count('\nMAPPING_STATUS_TOPIC = '), 1)
        body = source[source.index("def on_mapping_status("):source.index("def search_goals(")]
        self.assertLess(body.index("self.mapping_status = (row, time.monotonic())"),
                        body.index('if not getattr(args, "search_waypoints_from_mapping", False):'))
        self.assertIn("poses = mapping_search_waypoints(msg.data)", body)


if __name__ == '__main__':
    unittest.main()
