#!/usr/bin/env python3
"""Light image receiver for the camera-drop layer experiments (2026-10-13).

The two receivers of the real-camera runs (the producer and cam_probe.py) are both rclpy subscriptions that
turn every 600-900 KB image into a Python message; cam_probe lost MORE than the producer, so "both missed
it" does not prove a publisher-side loss. This one takes the RAW serialized bytes (rclpy ``raw=True``: no
Python message is built) and reads only header.stamp out of the CDR head (``parse_head`` below), so its own
cost per frame is one copy of the bytes.

One CSV row per arrival: topic, header stamp ns, wall ns (CLOCK_REALTIME, the axis of the stamp), RAW ns
(host.fpga_clock), message bytes. --summary: per topic the stamp intervals and the frames missing by stamp
(cam_probe.summarize, the same rule as the producer's input accounting), the stamp -> arrival delay, what
the first message was, and the parse self-check: the first --check-n messages of each topic are ALSO
deserialized the normal way and the stamp / frame id / height / width / encoding / data length compared
with what parse_head read from the bytes (``parse_check``: ok / bad counts; a bad one means the offsets are
wrong and every stamp of this file is suspect).

DDS message-lost count (2026-10-14, as the host repro's receiver dds-bigmsg-host-20261014/tools/rx.py): every
subscription also takes the rmw "message lost" event (rclpy SubscriptionEventCallbacks(message_lost=...)); Fast
DDS raises it for a sample the reader knows was sent but never got whole (missing fragments of a best-effort
sample), NOT for a keep-last history overflow inside the reader. Per topic the summary gets "lost":
{"total_count", "events", "change_sum", "log": [[wall_ns, total_count_change, total_count], ...]} (the log lets
a readout count the window only; the count itself covers the whole life of the receiver) and
"incompatible_qos" (total count). The callbacks run in the same spin as the messages, nothing per frame.
If the rmw does not support the event the subscription is made without it and "lost" says
{"supported": false, "error": ...} (never a silent 0).

--reliability best_effort (default, what the producer uses) | reliable (a reliable reader of a reliable
publisher gets lost fragments resent: the "what was published" reading; it does not match a best-effort
publisher such as the replay camera). --depth: history depth (keep last).
Stops at --duration-s or on SIGINT / SIGTERM (a handler is installed: a child started with & ignores SIGINT
by default and a KeyboardInterrupt-only loop would never see it).
"""
import argparse
import json
import os
import signal
import struct
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.fpga_clock import now_ns  # noqa: E402
from tools.integ_car_chain.cam_probe import summarize  # noqa: E402


def _align4(off):
    return (off + 3) & ~3


def parse_head(raw):
    """Head of a serialized sensor_msgs/Image (CDR, 4-byte encapsulation header, alignment relative to the
    byte after it): returns {"stamp_ns", "frame_id", "height", "width", "encoding", "data_len"}.
    Raises ValueError on anything that is not a plain CDR big/little-endian encapsulation or is truncated."""
    if len(raw) < 16:
        raise ValueError("short message: %d bytes" % len(raw))
    kind = raw[1]
    if raw[0] != 0 or kind not in (0, 1):
        raise ValueError("not plain CDR: encapsulation %r" % (bytes(raw[:4]),))
    e = "<" if kind == 1 else ">"
    base = 4
    sec, nsec = struct.unpack_from(e + "iI", raw, base)

    def string_at(off):
        (n,) = struct.unpack_from(e + "I", raw, base + off)
        if n < 1 or base + off + 4 + n > len(raw):
            raise ValueError("bad string length %d at %d" % (n, off))
        s = bytes(raw[base + off + 4: base + off + 4 + n - 1]).decode("utf-8", "replace")
        return s, off + 4 + n

    frame_id, off = string_at(8)
    off = _align4(off)
    height, width = struct.unpack_from(e + "II", raw, base + off)
    encoding, off = string_at(off + 8)
    off += 1                                   # is_bigendian
    off = _align4(off)
    step, data_len = struct.unpack_from(e + "II", raw, base + off)
    return {"stamp_ns": sec * 1000000000 + nsec, "frame_id": frame_id, "height": height, "width": width,
            "encoding": encoding, "step": step, "data_len": data_len}


def parse_stamp_ns(raw):
    """Only the stamp (the per-frame path): sec, nanosec right after the encapsulation header."""
    sec, nsec = struct.unpack_from("<iI" if raw[1] == 1 else ">iI", raw, 4)
    return sec * 1000000000 + nsec


def same_as_message(head, msg):
    """parse_head result against a normally deserialized message (duck-typed: unit-tested with stand-ins)."""
    return (head["stamp_ns"] == msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            and head["frame_id"] == msg.header.frame_id and head["height"] == msg.height
            and head["width"] == msg.width and head["encoding"] == msg.encoding and head["step"] == msg.step
            and head["data_len"] == len(msg.data))


LOST_LOG_MAX = 20000


def new_lost_record():
    return {"supported": True, "total_count": 0, "events": 0, "change_sum": 0, "log": []}


def lost_callback(rec, wall=time.time_ns):
    """The message_lost event callback for one topic (duck-typed status: total_count, total_count_change;
    unit-tested with stand-ins). total_count is the rmw's running total; change_sum adds the per-event
    changes (equal to total_count when no event was coalesced away before the first one we saw)."""
    log = rec["log"]

    def on(status):
        n = int(getattr(status, "total_count", -1))
        ch = int(getattr(status, "total_count_change", -1))
        rec["total_count"] = n
        rec["events"] += 1
        rec["change_sum"] += max(ch, 0)
        if len(log) < LOST_LOG_MAX:
            log.append([wall(), ch, n])
    return on


def incompatible_callback(store, topic):
    def on(status):
        store[topic] = int(getattr(status, "total_count", -1))
    return on


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", action="append", required=True)
    ap.add_argument("--info-topic", action="append", default=[],
                    help="sensor_msgs/CameraInfo topic recorded the same way (the small-message control: the "
                         "driver sends one with every image, same stamp)")
    ap.add_argument("--out", required=True, help="new CSV")
    ap.add_argument("--summary", required=True)
    ap.add_argument("--duration-s", type=float, default=200.0)
    ap.add_argument("--reliability", choices=("best_effort", "reliable"), default="best_effort")
    ap.add_argument("--depth", type=int, default=5)
    ap.add_argument("--check-n", type=int, default=3)
    ap.add_argument("--node-name", default="fpga_rx_lite")
    a, ros_args = ap.parse_known_args(argv)
    import rclpy
    import rclpy.signals
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, Image

    stop = {"sig": None}

    def on_signal(signum, _frame):
        stop["sig"] = signum

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    rclpy.init(args=ros_args, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = Node(a.node_name)
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=a.depth, durability=DurabilityPolicy.VOLATILE,
                     reliability=(ReliabilityPolicy.RELIABLE if a.reliability == "reliable"
                                  else ReliabilityPolicy.BEST_EFFORT))
    f = open(a.out, "x")
    f.write("topic,stamp_ns,wall_ns,arrival_raw_ns,bytes\n")
    seen = dict((t, {"stamps": [], "delays": [], "first": None, "ok": 0, "bad": 0, "err": 0}) for t in a.topic)
    wall = time.time_ns
    write = f.write

    def cb(topic):
        s = seen[topic]
        stamps, delays = s["stamps"], s["delays"]

        def on(raw):
            arr = now_ns()
            w = wall()
            try:
                st = parse_stamp_ns(raw)
            except Exception:  # noqa
                s["err"] += 1
                return
            stamps.append(st)
            delays.append((w - st) / 1e6)
            write("%s,%d,%d,%d,%d\n" % (topic, st, w, arr, len(raw)))
            if s["ok"] + s["bad"] < a.check_n:
                try:
                    head = parse_head(raw)
                    good = same_as_message(head, deserialize_message(raw, Image))
                except Exception as exc:  # noqa
                    head, good = {"error": repr(exc)}, False
                s["ok" if good else "bad"] += 1
                if s["first"] is None:
                    s["first"] = dict(head, bytes=len(raw), encapsulation=list(raw[:4]))
        return on

    def info_cb(topic):
        s = seen[topic]
        stamps, delays = s["stamps"], s["delays"]

        def on(raw):
            arr = now_ns()
            w = wall()
            try:
                st = parse_stamp_ns(raw)
            except Exception:  # noqa
                s["err"] += 1
                return
            stamps.append(st)
            delays.append((w - st) / 1e6)
            write("%s,%d,%d,%d,%d\n" % (topic, st, w, arr, len(raw)))
            if s["ok"] + s["bad"] < a.check_n:
                try:
                    m = deserialize_message(raw, CameraInfo)
                    good = st == m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec
                    head = {"stamp_ns": st, "frame_id": m.header.frame_id, "height": m.height, "width": m.width}
                except Exception as exc:  # noqa
                    head, good = {"error": repr(exc)}, False
                s["ok" if good else "bad"] += 1
                if s["first"] is None:
                    s["first"] = dict(head, bytes=len(raw), encapsulation=list(raw[:4]))
        return on

    lost = {}
    incompatible = {}

    def subscribe(typ, t, callback):
        lost[t] = new_lost_record()
        incompatible[t] = 0
        try:
            from rclpy.qos_event import SubscriptionEventCallbacks
            ev = SubscriptionEventCallbacks(message_lost=lost_callback(lost[t]),
                                            incompatible_qos=incompatible_callback(incompatible, t))
            node.create_subscription(typ, t, callback, qos, raw=True, event_callbacks=ev)
        except Exception as exc:  # noqa  (ImportError on a newer rclpy, UnsupportedEventTypeError on an rmw)
            lost[t] = {"supported": False, "error": repr(exc)}
            node.create_subscription(typ, t, callback, qos, raw=True)

    for t in a.topic:
        subscribe(Image, t, cb(t))
    for t in a.info_topic:
        seen[t] = {"stamps": [], "delays": [], "first": None, "ok": 0, "bad": 0, "err": 0}
        subscribe(CameraInfo, t, info_cb(t))
    t_start = time.time()
    end = time.monotonic() + a.duration_s
    try:
        while rclpy.ok() and stop["sig"] is None and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        f.close()
        out = {"_receiver": {"kind": "rclpy raw subscription", "reliability": a.reliability, "depth": a.depth,
                             "t_start_wall": t_start, "t_end_wall": time.time(), "stopped_by_signal": stop["sig"]}}
        for t in list(a.topic) + list(a.info_topic):
            s = seen[t]
            out[t] = dict(summarize(s["stamps"], s["delays"]), first=s["first"],
                          parse_check={"ok": s["ok"], "bad": s["bad"], "stamp_errors": s["err"]},
                          lost=lost.get(t), incompatible_qos=incompatible.get(t))
        with open(a.summary, "w") as g:
            json.dump(out, g, indent=1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
