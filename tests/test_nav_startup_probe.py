"""Known-answer controls for probe verdicts; does not simulate DDS itself."""
import argparse
from concurrent.futures import Future
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import time
import types
import unittest
from unittest.mock import patch


class Request:
    STARTUP = 0


class Client:
    def __init__(self, node, name):
        self.node, self.srv_name, self.calls = node, name, 0

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.calls += 1
        future = Future()
        if self.node.silent:
            return future
        if self.srv_name.endswith("manage_nodes"):
            if self.node.startup_success:
                self.node.state = 3
            response = types.SimpleNamespace(success=self.node.startup_success)
        else:
            response = types.SimpleNamespace(
                current_state=types.SimpleNamespace(id=self.node.state, label="test"))
        future.set_result(response)
        return future

    def remove_pending_request(self, future):
        pass


class Node:
    # Mirror the Humble Node descriptor: the old plain fake missed the real
    # startup crash caused by assigning to this read-only base-class property.
    @property
    def clients(self):
        return iter(())

    def __init__(self, name):
        self.state, self.silent, self.startup_success = 3, False, True

    def create_client(self, service_type, name):
        return Client(self, name)


def load_probe():
    fake = {"rclpy": types.SimpleNamespace(
                spin_once=lambda node, timeout_sec: time.sleep(.001)),
            "rclpy.node": types.SimpleNamespace(Node=Node),
            "lifecycle_msgs": types.ModuleType("lifecycle_msgs"),
            "lifecycle_msgs.srv": types.SimpleNamespace(GetState=types.SimpleNamespace(Request=Request)),
            "nav2_msgs": types.ModuleType("nav2_msgs"),
            "nav2_msgs.srv": types.SimpleNamespace(ManageLifecycleNodes=types.SimpleNamespace(Request=Request))}
    path = Path(__file__).resolve().parents[1] / "robot_sim/tests/nav_startup_probe.py"
    spec = importlib.util.spec_from_file_location("nav_startup_probe_test_target", path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, fake):
        spec.loader.exec_module(module)
    return module


class NavStartupProbeTests(unittest.TestCase):
    def setUp(self):
        self.module = load_probe()
        args = argparse.Namespace(interval=.001, response_timeout=.005, rounds=2,
                                  discovery_timeout=.1, startup_timeout=.02,
                                  duration=.04, allow_inactive=False, all_services_first=False)
        self.stdout = contextlib.redirect_stdout(io.StringIO())
        self.stdout.__enter__()
        self.node = self.module.Probe(args, None)

    def tearDown(self):
        self.stdout.__exit__(None, None, None)

    def test_live_active_responses_pass_and_reuse_clients(self):
        clients = dict(self.node.state_clients)
        self.node.observe()
        self.assertFalse(self.node.failures)
        self.assertTrue(self.node.all_replied())
        self.assertEqual(clients, self.node.state_clients)

    def test_discovered_but_silent_service_fails(self):
        self.node.silent = True
        self.node.observe()
        self.assertTrue(all(self.node.ready.values()))
        self.assertIn("response_timeout", [row["reason"] for row in self.node.failures])
        self.assertFalse(self.node.all_replied())

    def test_response_completed_by_cross_deadline_spin_is_rejected(self):
        client = self.node.state_clients["controller_server"]
        future = Future()
        self.node.pending["controller_server"] = (future, 0.0, 1)
        self.node.sent["controller_server"] = 1
        clock = [0.0]

        def complete_late(node, timeout_sec):
            clock[0] = self.node.args.response_timeout + .001
            future.set_result(types.SimpleNamespace(
                current_state=types.SimpleNamespace(id=3, label="active")))

        fake_time = types.SimpleNamespace(monotonic=lambda: clock[0], time_ns=time.time_ns)
        with patch.object(self.module, "time", fake_time), \
                patch.object(self.module.rclpy, "spin_once", complete_late):
            self.node.poll(send_requests=False)
        self.assertEqual(self.node.replies["controller_server"], 0)
        self.assertEqual(self.node.failures[0]["reason"], "response_timeout")
        self.assertGreater(self.node.failures[0]["roundtrip_s"],
                           self.node.args.response_timeout)

    def test_response_first_observed_at_deadline_is_rejected(self):
        client = self.node.state_clients["controller_server"]
        future = Future()
        self.node.pending["controller_server"] = (future, 0.0, 1)
        self.node.sent["controller_server"] = 1
        clock = [0.0]

        def complete_at_deadline(node, timeout_sec):
            clock[0] = self.node.args.response_timeout
            future.set_result(types.SimpleNamespace(
                current_state=types.SimpleNamespace(id=3, label="active")))

        fake_time = types.SimpleNamespace(monotonic=lambda: clock[0], time_ns=time.time_ns)
        with patch.object(self.module, "time", fake_time), \
                patch.object(self.module.rclpy, "spin_once", complete_at_deadline):
            self.node.poll(send_requests=False)
        self.assertEqual(self.node.replies["controller_server"], 0)
        self.assertEqual(self.node.failures[0]["reason"], "response_timeout")
        self.assertEqual(self.node.failures[0]["roundtrip_s"],
                         self.node.args.response_timeout)

    def test_partial_activation_is_not_success(self):
        self.node.state = 1
        self.node.observe()
        self.assertTrue(self.node.all_replied())
        self.assertIn("not_all_active", [row["reason"] for row in self.node.failures])

    def test_manager_rejection_is_not_retried(self):
        self.node.state, self.node.startup_success = 1, False
        with self.assertRaisesRegex(RuntimeError, "STARTUP failed; no retry"):
            self.node.startup()
        self.assertEqual([client.calls for client in self.node.managers.values()], [1, 0])

    def test_manager_completion_by_cross_deadline_spin_is_rejected(self):
        self.node.state = 1
        manager = self.node.managers["lifecycle_manager_map"]
        future = Future()
        clock = [0.0]

        def request_startup(request):
            manager.calls += 1
            return future

        def complete_late(node, timeout_sec):
            if self.node.phase == "startup_lifecycle_manager_map":
                clock[0] += self.node.args.startup_timeout + .001
                self.node.state = 3
                future.set_result(types.SimpleNamespace(success=True))
            else:
                clock[0] += .001

        fake_time = types.SimpleNamespace(monotonic=lambda: clock[0], time_ns=time.time_ns)
        with patch.object(manager, "call_async", request_startup), \
                patch.object(self.module, "time", fake_time), \
                patch.object(self.module.rclpy, "spin_once", complete_late):
            with self.assertRaisesRegex(RuntimeError, "STARTUP response timed out; no retry"):
                self.node.startup()
        self.assertEqual([client.calls for client in self.node.managers.values()], [1, 0])

    def test_silent_manager_times_out_without_retrying_or_starting_next_manager(self):
        self.node.state = 1
        manager = self.node.managers["lifecycle_manager_map"]
        clock = [0.0]

        def silent_startup(request):
            manager.calls += 1
            return Future()

        def advance(node, timeout_sec):
            clock[0] += .001

        fake_time = types.SimpleNamespace(monotonic=lambda: clock[0], time_ns=time.time_ns)
        with patch.object(manager, "call_async", silent_startup), \
                patch.object(self.module, "time", fake_time), \
                patch.object(self.module.rclpy, "spin_once", advance):
            with self.assertRaisesRegex(RuntimeError, "STARTUP response timed out; no retry"):
                self.node.startup()
        self.assertEqual([client.calls for client in self.node.managers.values()], [1, 0])

    def test_startup_requires_new_active_responses(self):
        self.node.state = 1
        self.node.startup()
        self.assertTrue(self.node.all_active())
        self.assertFalse(self.node.failures)
        self.assertEqual([client.calls for client in self.node.managers.values()], [1, 1])

    def test_all_services_first_sends_nothing_until_late_manager_is_discovered(self):
        self.node.args.all_services_first = True
        self.node.state = 1
        self.node.output = io.StringIO()
        manager = self.node.managers["lifecycle_manager_navigation"]
        checks = [0]

        def ready_late():
            checks[0] += 1
            if checks[0] <= 4:
                self.assertEqual([client.calls for client in self.node.state_clients.values()], [0] * 8)
            return checks[0] >= 4

        with patch.object(manager, "service_is_ready", ready_late):
            self.node.startup()
        rows = [json.loads(line) for line in self.node.output.getvalue().splitlines()]
        events = [row["event"] for row in rows]
        self.assertLess(events.index("all_services_discovered"), events.index("get_state_request"))
        barrier = next(row for row in rows if row["event"] == "barrier_passed")
        self.assertEqual(list(barrier["replies"].values()), [self.node.args.rounds] * 8)
        self.assertEqual([client.calls for client in self.node.managers.values()], [1, 1])
        self.assertTrue(self.node.all_active())
        self.assertFalse(self.node.failures)

    def test_failed_barrier_follows_up_without_activating_or_extending_deadline(self):
        self.node.state = 1
        self.node.output = io.StringIO()
        client = self.node.state_clients["controller_server"]
        original_call = client.call_async

        def drop_first_response(request):
            future = original_call(request)
            return Future() if client.calls == 1 else future

        # Deterministic wall clock: check the unchanged deadline without relying
        # on host scheduling speed. No middleware/manager timeouts are altered.
        clock = [0.0]
        self.node.started = 0.0

        def spin_once(node, timeout_sec):
            clock[0] += .001

        fake_time = types.SimpleNamespace(monotonic=lambda: clock[0], time_ns=time.time_ns)
        with patch.object(client, "call_async", drop_first_response), \
                patch.object(self.module, "time", fake_time), \
                patch.object(self.module.rclpy, "spin_once", spin_once):
            with self.assertRaisesRegex(RuntimeError, "no activation attempted"):
                self.node.startup()
        self.assertAlmostEqual(clock[0], self.node.args.discovery_timeout)
        self.assertEqual([entry.calls for entry in self.node.managers.values()], [0, 0])
        self.assertEqual(self.node.failures[0]["reason"], "response_timeout")
        self.assertEqual(self.node.failures[0]["sequence"], 1)
        self.assertTrue(self.node.failures)  # main's verdict stays passed=False.
        self.assertGreaterEqual(self.node.replies["controller_server"], 2)
        self.assertIs(client, self.node.state_clients["controller_server"])
        rows = [json.loads(line) for line in self.node.output.getvalue().splitlines()]
        self.assertTrue(any(row["event"] == "get_state_response" and
                            row["node"] == "controller_server" and
                            row["phase"] == "diagnostic_followup" for row in rows))
        self.assertFalse(any(row["event"] == "manager_startup_request" for row in rows))


if __name__ == "__main__":
    unittest.main()
