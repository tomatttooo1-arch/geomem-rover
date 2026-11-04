#!/usr/bin/env python3
"""New-ROS-participant generator for the publish-stall diagnosis (10-11 night).

Both ~300 ms producer publish stalls seen on the board (v1 frame 27, v2 frame 214) came ~2.6 s
after new DDS participants appeared (v1: the whole rig starting; v2: the measurement probe
board_probe.py starting after the warm-up). This tool makes that event frequent: every
--period-s it starts a child process that creates one node with the same subscriptions as
board_probe.py (/fpga/geometry_frame, /fpga/rolling_map/grid latched, /fpga/rolling_map/status,
/costmap/costmap), spins for --life-s, and exits. Read-only: it publishes nothing.

--log (new JSONL): one row per child: spawn RAW ns, node-ready RAW ns, first geometry frame RAW ns,
exit RAW ns, rc. RAW = host.fpga_clock (same clock as the producer's frames.jsonl).
--bare: children create a node without subscriptions (participant discovery only, no endpoint match).
--freeze-at-s A --freeze-s F (experiment): the parent SIGSTOPs each child A s after spawning it and
SIGCONTs it F s later (a participant that stops answering, e.g. its shared-memory listener).
Usage: participant_churn.py --log F --duration-s N [--period-s 6] [--life-s 4] [--freeze-at-s A --freeze-s F]
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.fpga_clock import now_ns  # noqa: E402


def child(name, life_s, bare=False):
    t_start = now_ns()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    rclpy.init()
    node = Node(name)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    first = {}

    def mark(key):
        def cb(_msg):
            first.setdefault(key, now_ns())
        return cb
    if not bare:
        node.create_subscription(String, "/fpga/geometry_frame", mark("frame"), 50)
        node.create_subscription(OccupancyGrid, "/fpga/rolling_map/grid", mark("grid"), latched)
        node.create_subscription(String, "/fpga/rolling_map/status", mark("status"), 50)
        node.create_subscription(OccupancyGrid, "/costmap/costmap", mark("costmap"), 10)
    t_ready = now_ns()
    end = time.monotonic() + life_s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps({"child_start_raw_ns": t_start, "node_ready_raw_ns": t_ready,
                      "first_raw_ns": first, "child_end_raw_ns": now_ns()}))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log")
    ap.add_argument("--duration-s", type=float, default=60.0)
    ap.add_argument("--period-s", type=float, default=6.0)
    ap.add_argument("--life-s", type=float, default=4.0)
    ap.add_argument("--freeze-at-s", type=float, default=None)
    ap.add_argument("--freeze-s", type=float, default=0.0)
    ap.add_argument("--bare", action="store_true", help="children create a node with no subscriptions")
    ap.add_argument("--child", default=None, help=argparse.SUPPRESS)
    a = ap.parse_args(argv)
    if a.child:
        return child(a.child, a.life_s, a.bare)
    if not a.log:
        ap.error("--log is required")
    out = open(a.log, "x")
    end = time.monotonic() + a.duration_s
    i = 0
    while time.monotonic() < end:
        i += 1
        t_spawn = now_ns()
        p = subprocess.Popen([sys.executable, os.path.abspath(__file__), "--child", "fpga_churn_%d" % i,
                              "--life-s", str(a.life_s)] + (["--bare"] if a.bare else []), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        next_t = time.monotonic() + a.period_s
        frozen = {}
        if a.freeze_at_s is not None:
            t_sp = time.monotonic()
            while p.poll() is None and time.monotonic() < t_sp + a.freeze_at_s:
                time.sleep(0.01)
            if p.poll() is None:
                p.send_signal(signal.SIGSTOP)
                frozen["stop_raw_ns"] = now_ns()
                time.sleep(a.freeze_s)
                p.send_signal(signal.SIGCONT)
                frozen["cont_raw_ns"] = now_ns()
        try:
            so, se = p.communicate(timeout=a.life_s + a.freeze_s + 30)
        except subprocess.TimeoutExpired:
            p.kill()
            so, se = p.communicate()
        row = {"i": i, "spawn_raw_ns": t_spawn, "rc": p.returncode, "exit_seen_raw_ns": now_ns()}
        row.update(frozen)
        try:
            row.update(json.loads(so.decode().strip().splitlines()[-1]))
        except (ValueError, IndexError):
            row["stderr_tail"] = se.decode(errors="replace")[-300:]
        out.write(json.dumps(row) + "\n")
        out.flush()
        while time.monotonic() < min(next_t, end):
            time.sleep(0.05)
    out.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
