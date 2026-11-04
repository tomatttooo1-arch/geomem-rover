#!/usr/bin/env python3
"""TF arrival probe + system sampler for the whole-chain board runs (TF gap diagnosis, 10-11).

Runs INSIDE the chain container (same ROS domain / shared memory as the producer):
  --tf-csv    one row per odom->base_link sample received on /tf, subscribed with the QoS of a
              tf2 TransformListener (reliable, volatile, depth 100), in a separate process from
              the producer: arrival RAW ns (host.fpga_clock), header stamp ns. If this process
              sees the stream while the producer's lookups fail, the gap is on the producer
              side; if this one sees the same hole, the stream itself stopped.
  --sys-csv   every --sample-ms: RAW ns, wall time, /proc/net/snmp Udp (InDatagrams NoPorts InErrors
              RcvbufErrors SndbufErrors), /proc/stat cpu busy/total jiffies, loadavg 1 min,
              and utime+stime ticks of each --pid NAME=PID (per-process CPU over time); since v7
              (2026-10-12) then one column per online CPU, freq_khz_cpuN = cpufreq scaling_cur_freq
              in kHz (empty when unreadable; the frame-age fast/slow states of v6 F1).
  --threads-jsonl / --threads-of NAME (10-11 night, publish-stall diagnosis): every sample, each
              thread of that --pid: [tid, state char, utime+stime ticks] (thread names once, in
              rows with "names"); a thread blocked in a lock/condvar shows S with no new ticks.
  --graph-jsonl  ROS graph as this process sees it (node names), polled every --graph-poll-s:
              one row per change with RAW ns (when new participants appear / leave).
Usage: tf_probe.py --tf-csv F --sys-csv F --duration-s N [--pid producer=123 ...]
"""
import argparse
import json
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.fpga_clock import now_ns  # noqa: E402

UDP_FIELDS = ("InDatagrams", "NoPorts", "InErrors", "RcvbufErrors", "SndbufErrors")


def udp_counters(text):
    lines = [l.split() for l in text.splitlines() if l.startswith("Udp:")]
    if len(lines) < 2:
        return [None] * len(UDP_FIELDS)
    head, vals = lines[0][1:], lines[1][1:]
    d = dict(zip(head, vals))
    return [int(d[k]) if k in d else None for k in UDP_FIELDS]


def cpu_jiffies(text):
    f = [int(x) for x in text.splitlines()[0].split()[1:]]
    busy = f[0] + f[1] + f[2] + f[5] + f[6] + (f[7] if len(f) > 7 else 0)
    return busy, busy + f[3] + f[4]


def proc_ticks(pid):
    try:
        with open("/proc/%d/stat" % pid) as fp:
            x = fp.read().rsplit(")", 1)[1].split()
        return int(x[11]) + int(x[12])
    except (OSError, ValueError, IndexError):
        return None


def cpu_freq_paths(root="/sys/devices/system/cpu"):
    """[(column name, scaling_cur_freq path)] of every cpuN with a cpufreq directory, by N."""
    out = []
    try:
        names = os.listdir(root)
    except OSError:
        return out
    for name in names:
        if name.startswith("cpu") and name[3:].isdigit():
            path = os.path.join(root, name, "cpufreq", "scaling_cur_freq")
            if os.path.exists(path):
                out.append((int(name[3:]), "freq_khz_" + name, path))
    return [(col, path) for _, col, path in sorted(out)]


def read_freq(path):
    try:
        with open(path) as fp:
            return int(fp.read().strip())
    except (OSError, ValueError):
        return None


def thread_states(pid):
    """[(tid, comm, state, ticks)] of every thread of pid (empty when gone)."""
    out = []
    try:
        tids = os.listdir("/proc/%d/task" % pid)
    except OSError:
        return out
    for tid in tids:
        try:
            with open("/proc/%d/task/%s/stat" % (pid, tid)) as fp:
                raw = fp.read()
        except OSError:
            continue
        head, rest = raw.split(" (", 1)
        comm, tail = rest.rsplit(") ", 1)
        x = tail.split()
        try:
            out.append((int(tid), comm, x[0], int(x[11]) + int(x[12])))
        except (ValueError, IndexError):
            continue
    return sorted(out)


def sampler(path, pids, period_s, stop, threads_path=None, threads_of=None, freq_paths=None):
    names = sorted(pids)
    freq = cpu_freq_paths() if freq_paths is None else freq_paths
    fcols = "".join("," + col for col, _ in freq)
    tout = open(threads_path, "x") if threads_path and threads_of in pids else None
    seen = {}
    with open(path, "x") as out:
        out.write("raw_ns,wall,%s,cpu_busy,cpu_total,load1,%s%s\n" % (",".join(UDP_FIELDS), ",".join(names), fcols))
        while not stop.is_set():
            t, wall = now_ns(), time.time()
            try:
                udp = udp_counters(open("/proc/net/snmp").read())
                busy, total = cpu_jiffies(open("/proc/stat").read())
                load1 = open("/proc/loadavg").read().split()[0]
            except OSError:
                udp, busy, total, load1 = [None] * len(UDP_FIELDS), None, None, None
            ticks = [proc_ticks(pids[n]) for n in names]
            fv = "".join("," + ("" if v is None else str(v)) for v in (read_freq(p) for _, p in freq))
            out.write("%d,%.6f,%s,%s,%s,%s,%s%s\n" % (t, wall, ",".join("" if v is None else str(v) for v in udp),
                                                busy, total, load1,
                                                ",".join("" if v is None else str(v) for v in ticks), fv))
            out.flush()
            if tout is not None:
                ths = thread_states(pids[threads_of])
                new = dict((tid, comm) for tid, comm, _, _ in ths if seen.get(tid) != comm)
                seen.update(new)
                row = {"raw_ns": t, "th": [[tid, st, tk] for tid, _, st, tk in ths]}
                if new:
                    row["names"] = dict((str(k), v) for k, v in new.items())
                tout.write(json.dumps(row, separators=(",", ":")) + "\n")
                tout.flush()
            stop.wait(period_s)
    if tout is not None:
        tout.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tf-csv", required=True)
    ap.add_argument("--sys-csv", required=True)
    ap.add_argument("--duration-s", type=float, required=True)
    ap.add_argument("--sample-ms", type=float, default=100.0)
    ap.add_argument("--pid", action="append", default=[], help="NAME=PID, repeatable")
    ap.add_argument("--odom-frame", default="odom")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--threads-jsonl", default=None)
    ap.add_argument("--threads-of", default="producer")
    ap.add_argument("--graph-jsonl", default=None)
    ap.add_argument("--graph-poll-s", type=float, default=0.2)
    a, ros_args = ap.parse_known_args(argv)
    pids = dict((p.split("=", 1)[0], int(p.split("=", 1)[1])) for p in a.pid)
    stop = threading.Event()
    th = threading.Thread(target=sampler, args=(a.sys_csv, pids, a.sample_ms / 1e3, stop, a.threads_jsonl,
                                                a.threads_of), daemon=True)
    th.start()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage
    out = open(a.tf_csv, "x")
    out.write("arrival_raw_ns,stamp_ns\n")
    rclpy.init(args=ros_args)
    node = Node("fpga_tf_probe")
    qos = QoSProfile(depth=100, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE)        # tf2_ros DynamicListenerQoS

    def on_tf(msg):
        t = now_ns()
        for tr in msg.transforms:
            if tr.child_frame_id == a.base_frame and tr.header.frame_id == a.odom_frame:
                out.write("%d,%d\n" % (t, tr.header.stamp.sec * 1000000000 + tr.header.stamp.nanosec))
    node.create_subscription(TFMessage, "/tf", on_tf, qos)
    end = time.monotonic() + a.duration_s
    gout = open(a.graph_jsonl, "x") if a.graph_jsonl else None
    last_graph, next_graph = None, 0.0
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if gout is not None and time.monotonic() >= next_graph:
                next_graph = time.monotonic() + a.graph_poll_s
                t0 = now_ns()
                names = sorted("%s/%s" % (ns.rstrip("/"), n) for n, ns in node.get_node_names_and_namespaces())
                t1 = now_ns()
                if names != last_graph:
                    prev = last_graph or []
                    gout.write(json.dumps({"raw_ns": t1, "poll_ms": (t1 - t0) / 1e6, "n": len(names),
                                           "added": sorted(set(names) - set(prev)),
                                           "removed": sorted(set(prev) - set(names))}) + "\n")
                    gout.flush()
                    last_graph = names
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        th.join(timeout=2)
        out.close()
        if gout is not None:
            gout.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
