#!/usr/bin/env python3
"""T1 (2026-10-16) host-side passive sampler for the Orin Nano (docs/design/HEARTBEAT_GAP_DIG_20261016.md §6/§9).

Runs on the Nano HOST (not in the stack container), read-only: every --period s one JSON line with wall and
CLOCK_MONOTONIC_RAW ns, thermal zones (each read guarded: cv0-2 give EAGAIN), pwm fan, tach, soctherm_oc counters,
ina3221 rails, cpu/gpu clocks, /proc/net/snmp (Ip, Udp), /proc/net/netstat IpExt, /proc/net/udp per socket
(drops, rx_queue, inode); every --slow s: `ss -uanpm`, the producer's / driver's fd -> socket inode and /dev/shm,
and `docker exec <stack> df/ls /dev/shm` (the stack container has its own /dev/shm). Stops on SIGTERM.
Standard library only.
"""
import argparse
import glob
import json
import os
import signal
import subprocess
import sys
import time

STOP = [False]


def rd(path):
    try:
        with open(path) as fh:
            return fh.read().strip()
    except Exception as exc:              # EAGAIN on cv0-2 thermal zones, missing files
        return "ERR:%s" % type(exc).__name__


def snmp(path, keep):
    out = {}
    txt = rd(path)
    if txt.startswith("ERR"):
        return {"error": txt}
    lines = txt.splitlines()
    for a, b in zip(lines[0::2], lines[1::2]):
        ka, kb = a.split(), b.split()
        if ka[0] in keep:
            out[ka[0].rstrip(":")] = dict(zip(ka[1:], (int(x) for x in kb[1:])))
    return out


def udp_sockets():
    rows = []
    for path in ("/proc/net/udp", "/proc/net/udp6"):
        txt = rd(path)
        if txt.startswith("ERR"):
            continue
        for ln in txt.splitlines()[1:]:
            f = ln.split()
            if len(f) < 13:
                continue
            tx, rx = f[4].split(":")
            rows.append({"f": path[-4:], "local": f[1], "rem": f[2], "rxq": int(rx, 16), "txq": int(tx, 16),
                         "uid": int(f[7]), "inode": int(f[9]), "drops": int(f[12])})
    return rows


def find_procs(patterns):
    out = {}
    me = os.getuid()
    for p in os.listdir("/proc"):
        if not p.isdigit():
            continue
        try:
            if os.stat("/proc/" + p).st_uid != me:
                continue
            cmd = open("/proc/%s/cmdline" % p, "rb").read().replace(b"\0", b" ").decode("utf-8", "replace")
        except Exception:
            continue
        for name, pat in patterns.items():
            if pat in cmd and "t1_sampler" not in cmd:
                out.setdefault(name, []).append({"pid": int(p), "cmd": cmd[:200]})
    return out


def fd_info(pid):
    socks, shm = [], []
    try:
        for fd in os.listdir("/proc/%d/fd" % pid):
            try:
                t = os.readlink("/proc/%d/fd/%s" % (pid, fd))
            except Exception:
                continue
            if t.startswith("socket:["):
                socks.append(int(t[8:-1]))
            elif "/dev/shm" in t:
                shm.append(t)
    except Exception as exc:
        return {"error": type(exc).__name__}
    maps = []
    try:
        for ln in open("/proc/%d/maps" % pid):
            if "/dev/shm" in ln or "fastrtps" in ln or "fastdds" in ln:
                maps.append(ln.split()[-1])
    except Exception:
        pass
    return {"sockets": sorted(socks), "shm_fds": sorted(shm), "shm_maps": sorted(set(maps))}


def run(cmd, timeout=20):
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
        return p.stdout.decode("utf-8", "replace")
    except Exception as exc:
        return "ERR:%s" % exc


def stack_name(car_json):
    try:
        return json.load(open(car_json))["containers"]["stack"]
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--period", type=float, default=5.0)
    ap.add_argument("--slow", type=float, default=60.0)
    ap.add_argument("--car-json", default="", help="RUN/car.json, read for the stack container name")
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: STOP.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: STOP.__setitem__(0, True))
    zones = sorted(glob.glob("/sys/class/thermal/thermal_zone*"))
    ztype = {z: rd(z + "/type") for z in zones}
    hw = {rd(h + "/name"): h for h in glob.glob("/sys/class/hwmon/hwmon*")}
    ina = hw.get("ina3221")
    ina_files = sorted(os.path.basename(f) for f in glob.glob(ina + "/*_input")) if ina else []
    oc = hw.get("soctherm_oc")
    pats = {"producer": "host.ros_geo_producer", "driver": "orbbec_camera", "runner": "host.ros_runtime",
            "bridge": "ros_rolling_nav"}
    fh = open(os.path.join(a.out, "samples.jsonl"), "a", buffering=1)
    sl = open(os.path.join(a.out, "slow.jsonl"), "a", buffering=1)
    meta = {"event": "start", "wall": time.time(), "raw_ns": time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW),
            "zones": ztype, "hwmon": hw, "ina_files": ina_files, "pid": os.getpid(),
            "trips": {ztype[z]: {os.path.basename(t): rd(t) + "/" + rd(t[:-5] + "_type")
                                 for t in sorted(glob.glob(z + "/trip_point_*_temp"))} for z in zones}}
    fh.write(json.dumps(meta) + "\n")
    next_slow = 0.0
    while not STOP[0]:
        t_loop = time.monotonic()
        row = {"wall": time.time(), "raw_ns": time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)}
        row["temp"] = {ztype[z]: rd(z + "/temp") for z in zones}
        row["pwm1"] = rd(hw["pwmfan"] + "/pwm1") if "pwmfan" in hw else None
        row["rpm"] = rd(hw["pwm_tach"] + "/rpm") if "pwm_tach" in hw else None
        if oc:
            row["oc"] = {os.path.basename(f): rd(f) for f in sorted(glob.glob(oc + "/oc*_event_cnt"))}
        if ina:
            row["ina"] = {f: rd(ina + "/" + f) for f in ina_files}
        row["cpu"] = {p: rd("/sys/devices/system/cpu/cpufreq/%s/scaling_cur_freq" % p) for p in ("policy0", "policy4")}
        row["gpu"] = rd("/sys/devices/platform/17000000.gpu/devfreq/17000000.gpu/cur_freq")
        row["snmp"] = snmp("/proc/net/snmp", ("Ip:", "Udp:", "UdpLite:"))
        row["netstat"] = snmp("/proc/net/netstat", ("IpExt:",))
        row["udp"] = udp_sockets()
        row["loadavg"] = rd("/proc/loadavg")
        fh.write(json.dumps(row) + "\n")
        if t_loop >= next_slow:
            next_slow = t_loop + a.slow
            s = {"wall": time.time(), "raw_ns": time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)}
            procs = find_procs(pats)
            s["procs"] = {k: [dict(p, **fd_info(p["pid"])) for p in v] for k, v in procs.items()}
            s["ss"] = run(["ss", "-uanpm"])
            st = stack_name(a.car_json) if a.car_json else None
            s["stack"] = st
            if st:
                s["stack_shm"] = run(["docker", "exec", st, "sh", "-c",
                                      "df -k /dev/shm; ls -la --time-style=full-iso /dev/shm"])
            sl.write(json.dumps(s) + "\n")
        while not STOP[0] and time.monotonic() - t_loop < a.period:
            time.sleep(0.2)
    fh.write(json.dumps({"event": "stop", "wall": time.time(),
                         "raw_ns": time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)}) + "\n")


if __name__ == "__main__":
    sys.exit(main())
