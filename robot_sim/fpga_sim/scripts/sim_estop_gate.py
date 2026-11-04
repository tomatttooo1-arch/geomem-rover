#!/usr/bin/env python3
"""Simulation stand-in for the car's hardware stop path (line D1, 2026-10-14).

On the car an e-stop goes PC -> hub -> KV260 (host/kv_estop.py latches it, the PL withdraws the
drive permit) -> heartbeat line -> MCU, which stops the motors. The fake KV on the server has no
permit block and nothing drives Gazebo's wheels from it, so in simulation this node reads the
KV260's e-stop latch (TELEM ``estop`` block, the same poll as tools/integ_car_chain/chain_watch.py)
at --hz and publishes it as std_msgs/Bool on --topic; robot_sim/fpga_sim/scripts/velocity_guard.py
(parameter estop_topic) then zeroes /fpga/cmd_vel_safe while it is True or stale (fail-closed).
A TELEM failure publishes nothing, so the guard stops the car 0.5 s later.
Line D2 replaces this with the permit model + mcu_sim; it is a simulation-only stand-in.

Every published sample and every latch change is appended to --log (JSONL, new file).
Run from the repo root with PYTHONPATH=. inside the ROS container.
"""
import argparse
import json
import os
import signal
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def latch_of(blocks):
    """True / False from a TELEM blocks dict, None when the estop block is missing."""
    e = blocks.get("estop") if isinstance(blocks, dict) else None
    if not isinstance(e, dict) or "latched" not in e:
        return None
    return bool(e["latched"])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kv-server", required=True, help="HOST:PORT of kv_server")
    ap.add_argument("--topic", default="/fpga/sim_estop")
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--log", required=True)
    a, ros_args = ap.parse_known_args(argv)
    host, port = a.kv_server.rsplit(":", 1)

    from host.kv_link import KvTelemetryClient, LinkError, blocks_mask
    from host.fpga_clock import now_ns
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool

    log = open(a.log, "x")
    rclpy.init(args=ros_args)
    node = Node("fpga_sim_estop_gate")
    pub = node.create_publisher(Bool, a.topic, 1)
    tel = KvTelemetryClient((host, int(port)), timeout_s=0.2, hello_timeout_s=3.0)
    want = blocks_mask(("estop",))
    state = {"last": None, "n": 0, "fail": 0}

    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok():
            t0 = now_ns()
            try:
                latched = latch_of(tel.poll(want).blocks)
            except LinkError as exc:
                latched, err = None, type(exc).__name__
            else:
                err = None if latched is not None else "no_estop_block"
            if latched is not None:
                pub.publish(Bool(data=latched))
                state["n"] += 1
            else:
                state["fail"] += 1
            if latched != state["last"] or err is not None or state["n"] % 200 == 1:
                log.write(json.dumps({"t_ns": t0, "wall": time.time(), "latched": latched, "error": err,
                                      "change": latched != state["last"], "published": state["n"],
                                      "failures": state["fail"]}) + "\n")
                log.flush()
            state["last"] = latched
            rclpy.spin_once(node, timeout_sec=0)
            time.sleep(max(0.0, 1.0 / a.hz - (now_ns() - t0) / 1e9))
    except KeyboardInterrupt:
        pass
    finally:
        log.close()
        tel.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
