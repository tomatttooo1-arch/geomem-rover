#!/usr/bin/env python3
"""N5 stall probe (host, no sudo). docs/design/NANO_PARTITION_LOAD_20260927.md §3.

Every --interval (default 0.1 s) one JSON line:
  [mono_ns, wall_ns, procs_running, procs_blocked, iowait_ticks_total, disk, dirty_kb, writeback_kb, dtasks]
  disk   = [reads_done, sectors_read, writes_done, sectors_written, in_flight, io_ticks_ms, weighted_ms]
           of --disk (default mmcblk0) from /proc/diskstats
  dtasks = D-state tasks [[pid, comm, wchan], ...], scanned when procs_blocked > 0 and on every 10th
           row (a full /proc scan costs a few ms); null when not scanned
/proc/pressure is read too when the kernel has it (the Nano's L4T kernel has no PSI: then null).
The output file should live on tmpfs so the probe itself is not blocked by the device it watches.
A gap between consecutive rows much larger than the interval means the probe itself was not run.
"""
import argparse
import json
import os
import time


def read(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return None


def stat_fields():
    running = blocked = iowait = 0
    for line in read("/proc/stat").splitlines():
        if line.startswith("cpu "):
            iowait = int(line.split()[5])
        elif line.startswith("procs_running"):
            running = int(line.split()[1])
        elif line.startswith("procs_blocked"):
            blocked = int(line.split()[1])
    return running, blocked, iowait


def disk_fields(dev):
    for line in read("/proc/diskstats").splitlines():
        p = line.split()
        if len(p) > 13 and p[2] == dev:
            return [int(p[3]), int(p[5]), int(p[7]), int(p[9]), int(p[11]), int(p[12]), int(p[13])]
    return None


def mem_fields():
    dirty = wb = None
    for line in read("/proc/meminfo").splitlines():
        if line.startswith("Dirty:"):
            dirty = int(line.split()[1])
        elif line.startswith("Writeback:"):
            wb = int(line.split()[1])
    return dirty, wb


def d_tasks(limit=40):
    out = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        st = read("/proc/%s/stat" % pid)
        if not st:
            continue
        r = st.rfind(")")
        if st[r + 2:r + 3] != "D":
            continue
        comm = st[st.find("(") + 1:r]
        wchan = read("/proc/%s/wchan" % pid) or ""
        out.append([int(pid), comm, wchan])
        if len(out) >= limit:
            break
    return out


def psi():
    out = {}
    for k in ("cpu", "io", "memory"):
        t = read("/proc/pressure/" + k)
        if t is None:
            return None
        out[k] = t.strip()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--stop-file", required=True)
    ap.add_argument("--interval", type=float, default=0.1)
    ap.add_argument("--disk", default="mmcblk0")
    a = ap.parse_args()
    has_psi = psi() is not None
    with open(a.out, "w", buffering=1 << 16) as f:
        f.write(json.dumps({"head": 1, "interval": a.interval, "disk": a.disk, "psi": has_psi,
                            "clk_tck": os.sysconf("SC_CLK_TCK")}) + "\n")
        last_flush = time.monotonic()
        nxt = time.monotonic()
        k = 0
        while not os.path.exists(a.stop_file):
            running, blocked, iowait = stat_fields()
            dirty, wb = mem_fields()
            k += 1
            dt = d_tasks() if blocked > 0 or k % 10 == 0 else None
            row = [time.monotonic_ns(), time.time_ns(), running, blocked, iowait, disk_fields(a.disk), dirty, wb, dt]
            if has_psi:
                row.append(psi())
            f.write(json.dumps(row, separators=(",", ":")) + "\n")
            now = time.monotonic()
            if now - last_flush >= 1.0:
                f.flush()
                last_flush = now
            nxt += a.interval
            time.sleep(max(0.0, nxt - time.monotonic()))
            if time.monotonic() - nxt > 1.0:  # resynchronise after a long stall instead of bursting
                nxt = time.monotonic()


if __name__ == "__main__":
    main()
