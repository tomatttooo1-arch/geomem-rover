#!/usr/bin/env python3
"""sysmon.py OUT_JSONL DURATION_S [PERIOD_S]: board-side sampler for the camera-drop experiments (Nano host).

Every PERIOD_S (default 1): wall time, per-cpu scaling_cur_freq, per-cpu busy/total jiffies, and for every
process whose command line contains one of PATTERNS each thread's (name, utime+stime ticks, last cpu).
First and last record also carry /proc/net/snmp Udp counters and /proc/loadavg.
Stops at DURATION_S or on SIGTERM/SIGINT (handler installed: started with & it would ignore SIGINT).
Repo copy (2026-10-14) of the sampler that ran on the Nano host in the 10-13 arms; no hard-coded paths.
"""
import glob
import json
import os
import signal
import sys
import time

PATTERNS = ("component_container", "rx_lite.py", "cam_probe.py", "replay_camera.py", "ros_geo_producer",
            "ros_rolling_nav", "fpga_cam_probe", "stress", "live_tf.py", "board_probe.py", "tf_probe.py",
            "fpga_hub", "rx_cpp")
stop = {"s": False}


def on_sig(_n, _f):
    stop["s"] = True


def freqs():
    out = []
    for p in sorted(glob.glob("/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq")):
        try:
            out.append(int(open(p).read()))
        except (OSError, ValueError):
            out.append(-1)
    return out


def cpu_stat():
    out = {}
    for line in open("/proc/stat"):
        if not line.startswith("cpu"):
            break
        f = line.split()
        v = [int(x) for x in f[1:9]]
        out[f[0]] = [v[0] + v[1] + v[2] + v[5] + v[6] + v[7], sum(v)]
    return out


def udp():
    rows = [line.split() for line in open("/proc/net/snmp") if line.startswith("Udp:")]
    return dict(zip(rows[0][1:], (int(x) for x in rows[1][1:]))) if len(rows) == 2 else {}


def scan():
    found = {}
    for d in glob.glob("/proc/[0-9]*"):
        try:
            cmd = open(d + "/cmdline", "rb").read().replace(b"\0", b" ").decode("utf-8", "replace")
        except OSError:
            continue
        for p in PATTERNS:
            if p in cmd and "sysmon.py" not in cmd:
                found[int(d[6:])] = p + "|" + cmd[:120]
                break
    return found


def threads(pid):
    out = {}
    for t in glob.glob("/proc/%d/task/[0-9]*" % pid):
        try:
            s = open(t + "/stat").read()
        except OSError:
            continue
        r = s.rfind(")")
        name = s[s.find("(") + 1:r]
        f = s[r + 2:].split()
        out[os.path.basename(t)] = [name, int(f[11]) + int(f[12]), int(f[36])]
    return out


def main():
    out, dur = sys.argv[1], float(sys.argv[2])
    period = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)
    end = time.monotonic() + dur
    procs, next_scan, n = {}, 0.0, 0
    with open(out, "w") as f:
        while not stop["s"]:
            now = time.monotonic()
            if now >= next_scan:
                procs = scan()
                next_scan = now + 3.0
            last = now + period >= end
            rec = {"t": time.time(), "freq": freqs(), "cpu": cpu_stat(),
                   "th": dict((str(p), {"cmd": c, "t": threads(p)}) for p, c in procs.items())}
            if n == 0 or last:
                rec["udp"] = udp()
                rec["loadavg"] = open("/proc/loadavg").read().strip()
                rec["clk_tck"] = os.sysconf("SC_CLK_TCK")
            f.write(json.dumps(rec, separators=(",", ":")) + "\n")
            f.flush()
            n += 1
            if last:
                break
            time.sleep(max(0.0, period - (time.monotonic() - now)))
        if stop["s"]:
            f.write(json.dumps({"t": time.time(), "udp": udp(), "stopped": True}) + "\n")


if __name__ == "__main__":
    main()
