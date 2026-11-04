#!/usr/bin/env python3
"""Observe Nav2 service discovery/roundtrips, optionally request one startup.

Clients survive every sample, including timed-out requests. Discovery alone is
never a successful response. All deadlines use wall time, independent of /clock.
The barrier checks THIS client's readers, not the lifecycle manager's readers.
No manager timeout or retry policy is changed. No velocity is published.

Coverage added over the old velocity_smoother-only readiness check: records all
eight nodes' discovery, first/repeated response latency, missing replies, and
partial activation. Existing runtime readiness and manager failures stay intact.
Failed startup barriers keep sampling only until their original deadline to
record whether the same clients recover. The failure verdict is never cleared.
"""
import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node


# Humble nav2_bringup/launch/navigation_launch.py, plus our separate map server.
NODES = ("map_server", "controller_server", "smoother_server", "planner_server",
         "behavior_server", "bt_navigator", "waypoint_follower", "velocity_smoother")
# CTRLPS2 (2026-11-03): FPGA_NAV_PROBE_NODES (comma list) = the nodes of a reduced launch (CAR_NAV_BACKEND=kv:
# map_server,planner_server); unset or empty = the full Humble list above, exactly as before
if os.environ.get("FPGA_NAV_PROBE_NODES"):
    NODES = tuple(n for n in os.environ["FPGA_NAV_PROBE_NODES"].split(",") if n)
MANAGERS = ("lifecycle_manager_map", "lifecycle_manager_navigation")


class Probe(Node):
    def __init__(self, args, output):
        super().__init__("fpga_nav_startup_probe")
        self.args, self.output = args, output
        self.started = time.monotonic()
        self.state_clients = {name: self.create_client(GetState, "/" + name + "/get_state")
                        for name in NODES}
        self.managers = {name: self.create_client(
            ManageLifecycleNodes, "/" + name + "/manage_nodes") for name in MANAGERS}
        self.ready = {}
        self.pending = {}
        self.next_query = {name: 0.0 for name in NODES}
        self.sent = {name: 0 for name in NODES}
        self.replies = {name: 0 for name in NODES}
        self.states = {}
        self.failures = []
        self.phase = "observation"
        self.emit("begin", arguments=vars(args), nodes=NODES,
                  environment={key: os.environ.get(key) for key in (
                      "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION",
                      "FASTRTPS_DEFAULT_PROFILES_FILE", "FPGA_NAV_AUTOSTART")})

    def emit(self, event, **data):
        row = {"event": event, "wall_ns": time.time_ns(),
               "elapsed_s": time.monotonic() - self.started, "phase": self.phase, **data}
        line = json.dumps(row, allow_nan=False)
        print(line, flush=True)
        if self.output:
            self.output.write(line + "\n")
            self.output.flush()

    def failure(self, reason, **data):
        self.failures.append({"reason": reason, **data})
        self.emit("failure", reason=reason, **data)

    def poll(self, send_requests=True):
        rclpy.spin_once(self, timeout_sec=0.02)
        for name, client in {**self.state_clients, **self.managers}.items():
            ready = client.service_is_ready()
            if self.ready.get(name) != ready:
                self.ready[name] = ready
                self.emit("service_discovery", node=name, service=client.srv_name,
                          ready=ready)
        for name, client in self.state_clients.items():
            if name in self.pending:
                future, sent_at, sequence = self.pending[name]
                # We cannot recover the instant at which the Future became done.
                # If this executor observes it at or beyond the wall deadline,
                # conservatively reject it even when the same spin completed it.
                observed_at = time.monotonic()
                roundtrip = observed_at - sent_at
                if roundtrip >= self.args.response_timeout:
                    client.remove_pending_request(future)
                    future.cancel()
                    del self.pending[name]
                    self.failure("response_timeout", node=name, sequence=sequence,
                                 roundtrip_s=roundtrip,
                                 service_discovered=self.ready[name])
                    self.next_query[name] = observed_at + self.args.interval
                elif future.done():
                    del self.pending[name]
                    try:
                        state = future.result().current_state
                        self.replies[name] += 1
                        self.states[name] = state.id
                        self.emit("get_state_response", node=name, sequence=sequence,
                                  first_response=self.replies[name] == 1,
                                  roundtrip_s=roundtrip, state_id=state.id,
                                  state_label=state.label)
                    except Exception as exc:
                        self.failure("response_exception", node=name, error=str(exc))
                    self.next_query[name] = observed_at + self.args.interval
            if (send_requests and name not in self.pending and self.ready.get(name)
                    and time.monotonic() >= self.next_query[name]):
                self.sent[name] += 1
                # Include time spent inside call_async in the request deadline.
                sent_at = time.monotonic()
                future = client.call_async(GetState.Request())
                self.pending[name] = (future, sent_at, self.sent[name])
                self.emit("get_state_request", node=name, sequence=self.sent[name])

    def all_replied(self):
        return all(count >= self.args.rounds for count in self.replies.values())

    def all_active(self):
        return all(self.states.get(name) == 3 for name in NODES)

    def startup(self):
        self.phase = "discovery_barrier"
        deadline = time.monotonic() + self.args.discovery_timeout
        may_request = not self.args.all_services_first
        barrier_passed = False
        while time.monotonic() < deadline:
            self.poll(send_requests=may_request)
            # poll() may consume the remainder of the whole barrier budget.
            # Do not let work first observed after that deadline authorize STARTUP.
            if time.monotonic() >= deadline:
                break
            if not may_request and all(self.ready.get(name) for name in (*NODES, *MANAGERS)):
                may_request = True
                self.emit("all_services_discovered", nodes=list(NODES), managers=list(MANAGERS))
                # Discovery uses the same deadline as the subsequent roundtrips.
                # It does not prove that either client's response reader matched.
                continue
            if self.failures:
                if self.phase != "diagnostic_followup":
                    self.phase = "diagnostic_followup"
                    self.emit("activation_inhibited", reason="initial_service_response_failed",
                              remaining_s=max(0.0, deadline - time.monotonic()))
                # Read-only follow-up, with the original deadline and clients.
                # A later reply cannot authorize STARTUP after any failure.
                continue
            if self.all_replied() and all(self.ready.get(name) for name in MANAGERS):
                barrier_passed = True
                break
        if not barrier_passed:
            if self.failures:
                raise RuntimeError("service response failed before startup; no activation attempted")
            raise RuntimeError("discovery/roundtrip barrier timed out")
        self.emit("barrier_passed", replies=self.replies)
        # Lifecycle transitions may hold a server's callback group. Do not inject
        # competing get_state requests during STARTUP and mislabel that as DDS loss.
        for name, (future, _, _) in self.pending.items():
            self.state_clients[name].remove_pending_request(future)
            future.cancel()
            self.emit("request_cancelled_at_barrier", node=name)
        self.pending.clear()
        for name in MANAGERS:
            self.phase = "startup_" + name
            request = ManageLifecycleNodes.Request()
            request.command = ManageLifecycleNodes.Request.STARTUP
            started = time.monotonic()
            future = self.managers[name].call_async(request)
            self.emit("manager_startup_request", node=name)
            deadline = started + self.args.startup_timeout
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
            completed_at = time.monotonic()
            if completed_at >= deadline or not future.done():
                self.managers[name].remove_pending_request(future)
                future.cancel()
                raise RuntimeError(name + " STARTUP response timed out; no retry")
            response = future.result()
            self.emit("manager_startup_response", node=name,
                      roundtrip_s=completed_at - started, success=response.success)
            if not response.success:
                raise RuntimeError(name + " STARTUP failed; no retry")
        self.phase = "verify_active"
        self.states.clear()  # Require fresh responses after manager completion.
        deadline = time.monotonic() + self.args.discovery_timeout
        while time.monotonic() < deadline:
            self.poll()
            if self.failures:
                raise RuntimeError("service response failed after startup")
            if time.monotonic() >= deadline:
                break
            if self.all_active():
                self.emit("all_active", states=self.states)
                return
        raise RuntimeError("not all lifecycle nodes reached active")

    def observe(self):
        deadline = self.started + self.args.duration
        while time.monotonic() < deadline:
            self.poll()
        if not self.all_replied():
            self.failure("missing_repeated_responses", replies=self.replies)
        if not self.args.allow_inactive and not self.all_active():
            self.failure("not_all_active", states=self.states)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--observe-only", action="store_true")
    mode.add_argument("--activate", action="store_true")
    parser.add_argument("--all-services-first", action="store_true",
                        help="with --activate, discover every dependency before any get_state request")
    parser.add_argument("--allow-inactive", action="store_true",
                        help="observe service roundtrips without requiring active (no TF/clock)")
    parser.add_argument("--out", help="new JSONL path; stdout always receives the same events")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--discovery-timeout", type=float, default=15.0)
    parser.add_argument("--response-timeout", type=float, default=2.0)
    parser.add_argument("--startup-timeout", type=float, default=60.0,
                        help="whole-manager operation deadline, not per-service Nav2 timeout")
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--rounds", type=int, default=2)
    args = parser.parse_args()
    if args.all_services_first and not args.activate:
        parser.error("--all-services-first requires --activate")
    if any(getattr(args, key) <= 0 for key in (
            "duration", "discovery_timeout", "response_timeout", "startup_timeout",
            "interval", "rounds")):
        parser.error("timeouts, interval and rounds must be positive")
    output = Path(args.out).open("x") if args.out else None
    rclpy.init()
    node = Probe(args, output)
    try:
        node.startup() if args.activate else node.observe()
    except (Exception, KeyboardInterrupt) as exc:
        node.failure("probe_aborted", error=str(exc))
    finally:
        passed = not node.failures
        node.emit("summary", passed=passed, states=node.states,
                  requests=node.sent, replies=node.replies, failures=node.failures)
        node.destroy_node()
        rclpy.shutdown()
        if output:
            output.close()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
