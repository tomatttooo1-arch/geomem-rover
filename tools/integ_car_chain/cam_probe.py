#!/usr/bin/env python3
"""Independent image subscriber for the input-drop runs (v8, 2026-10-12).

Subscribes to every --topic (sensor_msgs/Image, sensor-data QoS: best effort, keep last 5 -- what
the producer uses) and writes one CSV row per arrival: topic, header stamp ns, ROS time now ns,
host.fpga_clock RAW ns. On exit --summary gets, per topic, what was received (frame id, encoding,
size), the stamp-interval distribution, the frames missing by stamp (``summarize`` below, the same
rule as host.geo_producer.InputGapCounter: a step over 1.5 periods is round(step/period)-1
frames), the stamp -> arrival delay in ROS time (a driver that stamps with device time shows up
here as a delay far from a few ms) and the publishers' and subscriptions' QoS as the graph reports them.
A second receiver beside the producer: a frame both miss was lost on the publisher's side.
The graph is read every --graph-every-s WHILE the node runs and the last non-empty reading is kept (board
smoke 2026-10-13: read only on exit, after SIGINT had invalidated the context, it gave an RCLError and no
QoS at all; the subscriptions list is how the producer's own QoS on the image topic gets on record).
"""
import argparse
import csv
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.fpga_clock import now_ns  # noqa: E402


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))] if v else None


def summarize(stamps_ns, delays_ms=None):
    """Known-answer part (no ROS): stamps of the arrivals, in arrival order -> interval statistics
    and the frames missing by stamp. Period = median step."""
    n = len(stamps_ns)
    steps = [(b - a) / 1e6 for a, b in zip(stamps_ns, stamps_ns[1:])]
    pos = [s for s in steps if s > 0]
    out = {"arrivals": n, "non_monotonic": len(steps) - len(pos)}
    if len(pos) < 8:
        return out
    period = pct(pos, 0.5)
    gaps = [s for s in pos if s > 1.5 * period]
    out.update(period_ms=round(period, 3), rate_hz=round(1000.0 / period, 3),
               step_ms={"p01": round(pct(pos, 0.01), 3), "p50": round(period, 3), "p99": round(pct(pos, 0.99), 3),
                        "min": round(min(pos), 3), "max": round(max(pos), 3)},
               steps_over_1p5_period=len(gaps), missed_frames=sum(max(1, int(round(s / period)) - 1) for s in gaps),
               gap_steps_ms=[round(s, 1) for s in gaps[:50]])
    if delays_ms:
        out["stamp_to_arrival_ms"] = {"p01": round(pct(delays_ms, 0.01), 3), "p50": round(pct(delays_ms, 0.5), 3),
                                      "p99": round(pct(delays_ms, 0.99), 3), "min": round(min(delays_ms), 3),
                                      "max": round(max(delays_ms), 3)}
    return out


def endpoint_rows(infos):
    """Graph endpoint infos (rclpy TopicEndpointInfo) -> plain rows; no ROS import (unit-tested with stand-ins)."""
    return [{"node": i.node_name, "reliability": str(i.qos_profile.reliability).split(".")[-1],
             "durability": str(i.qos_profile.durability).split(".")[-1],
             "history": str(i.qos_profile.history).split(".")[-1], "depth": i.qos_profile.depth} for i in infos]


class GraphKeeper:
    """Keeps, per topic and per kind, the last NON-EMPTY graph reading; a failing or empty later read (the
    context is gone, the peer has left) never overwrites it. read(kind, topic) is injected."""

    def __init__(self, topics, read):
        self.topics, self.read = list(topics), read
        self.last = dict((t, {"publishers": None, "subscriptions": None}) for t in self.topics)
        self.reads = self.errors = 0

    def poll(self):
        for t in self.topics:
            for kind in ("publishers", "subscriptions"):
                try:
                    rows = endpoint_rows(self.read(kind, t))
                except Exception:  # noqa
                    self.errors += 1
                    continue
                self.reads += 1
                if rows:
                    self.last[t][kind] = rows

    def result(self, topic):
        out = dict(self.last[topic])
        out["graph_reads"] = self.reads
        out["graph_read_errors"] = self.errors
        return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", action="append", required=True)
    ap.add_argument("--graph-every-s", type=float, default=5.0)
    ap.add_argument("--out", required=True, help="new CSV")
    ap.add_argument("--summary", required=True)
    ap.add_argument("--duration-s", type=float, default=200.0)
    a, ros_args = ap.parse_known_args(argv)
    import rclpy
    import rclpy.executors
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init(args=ros_args)
    node = Node("fpga_cam_probe")
    f = open(a.out, "x", newline="")
    w = csv.writer(f)
    w.writerow(["topic", "stamp_ns", "ros_now_ns", "arrival_raw_ns"])
    seen = dict((t, {"stamps": [], "delays": [], "first": None}) for t in a.topic)

    def cb(topic):
        def on(msg):
            raw = now_ns()
            ros = node.get_clock().now().nanoseconds
            st = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            s = seen[topic]
            if s["first"] is None:
                s["first"] = {"frame_id": msg.header.frame_id, "encoding": msg.encoding, "width": msg.width,
                              "height": msg.height, "bytes": len(msg.data)}
            s["stamps"].append(st)
            s["delays"].append((ros - st) / 1e6)
            w.writerow([topic, st, ros, raw])
        return on

    for t in a.topic:
        node.create_subscription(Image, t, cb(t), qos_profile_sensor_data)
    graph = GraphKeeper(a.topic, lambda kind, t: (node.get_publishers_info_by_topic(t) if kind == "publishers"
                                                  else node.get_subscriptions_info_by_topic(t)))
    end = time.monotonic() + a.duration_s
    next_graph = time.monotonic() + min(2.0, a.graph_every_s)
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= next_graph:
                graph.poll()
                next_graph = time.monotonic() + a.graph_every_s
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        out = {}
        for t in a.topic:
            s = seen[t]
            out[t] = dict(summarize(s["stamps"], s["delays"]), first=s["first"])
            out[t].update(graph.result(t))
        f.close()
        with open(a.summary, "w") as g:
            json.dump(out, g, indent=1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
