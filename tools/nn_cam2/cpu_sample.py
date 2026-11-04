#!/usr/bin/env python3
"""NN-CAM2: A53 four-core occupancy from /proc/stat (+ chosen pids from /proc/<pid>/stat), one JSONL row per period.
No sysstat needed. Units: cores (1.0 = one core fully busy; four-core board full = 4.0).

  cpu_sample.py --out cpu.jsonl [--period 1.0] [--seconds 600] [--pid NAME=PID ...]
  cpu_sample.py --summary cpu.jsonl [--skip-s 5]      -> mean / p95 per CPU, total, per pid
  cpu_sample.py --selftest                            -> spin one core 3 s: total and the spinner ~1.0 core
Busy = user+nice+system+irq+softirq+steal (idle and iowait excluded)."""
import argparse
import json
import os
import sys
import time


def read_stat():
    out = {}
    with open("/proc/stat") as fp:
        for line in fp:
            if line.startswith("cpu"):
                f = line.split()
                v = [int(x) for x in f[1:9]]
                busy = v[0] + v[1] + v[2] + v[5] + v[6] + v[7]
                out[f[0]] = (busy, busy + v[3] + v[4])
    return out


def read_pid(pid):
    try:
        with open("/proc/%d/stat" % pid) as fp:
            f = fp.read().rsplit(")", 1)[1].split()
        return int(f[11]) + int(f[12])      # utime + stime (ticks)
    except (OSError, IndexError, ValueError):
        return None


def sample(out, period, seconds, pids):
    hz = os.sysconf("SC_CLK_TCK")
    s0, p0, t0 = read_stat(), {k: read_pid(v) for k, v in pids.items()}, time.monotonic()
    end = t0 + seconds
    with open(out, "w") as fp:
        while time.monotonic() < end:
            time.sleep(period)
            s1, p1, t1 = read_stat(), {k: read_pid(v) for k, v in pids.items()}, time.monotonic()
            row = {"t_mono_ns": time.monotonic_ns(), "dt_s": round(t1 - t0, 4)}
            for c in s1:
                if c in s0 and s1[c][1] > s0[c][1]:
                    frac = (s1[c][0] - s0[c][0]) / (s1[c][1] - s0[c][1])
                    ncpu = len(s1) - 1 if c == "cpu" else 1
                    row[c] = round(frac * ncpu, 4)   # "cpu" = all cores, in cores
            for k in pids:
                if p0.get(k) is not None and p1.get(k) is not None:
                    row["pid:" + k] = round((p1[k] - p0[k]) / hz / (t1 - t0), 4)
            fp.write(json.dumps(row) + "\n")
            fp.flush()
            s0, p0, t0 = s1, p1, t1


def summary(path, skip_s):
    rows = [json.loads(l) for l in open(path) if l.strip()]
    if not rows:
        return {}
    t_first = rows[0]["t_mono_ns"]
    rows = [r for r in rows if r["t_mono_ns"] - t_first >= skip_s * 1e9]
    keys = sorted({k for r in rows for k in r if k.startswith("cpu") or k.startswith("pid:")})
    res = {"samples": len(rows)}
    for k in keys:
        v = sorted(r[k] for r in rows if k in r)
        if v:
            res[k] = {"mean": round(sum(v) / len(v), 4), "p95": v[min(len(v) - 1, int(0.95 * (len(v) - 1) + 0.5))],
                      "max": v[-1]}
    return res


def selftest():
    import multiprocessing as mp
    import tempfile

    def spin(sec):
        e = time.monotonic() + sec
        while time.monotonic() < e:
            pass
    p = mp.Process(target=spin, args=(4.0,))
    p.start()
    f = tempfile.mktemp(suffix=".jsonl")
    sample(f, 0.5, 3.0, {"spin": p.pid})
    p.join()
    s = summary(f, 0.5)
    os.unlink(f)
    ok = abs(s["pid:spin"]["mean"] - 1.0) < 0.1
    print(json.dumps({"selftest": "pass" if ok else "FAIL", "spinner_cores": s["pid:spin"]["mean"],
                      "total_cores": s["cpu"]["mean"], "note": "total includes everything else on this machine"}))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out")
    ap.add_argument("--period", type=float, default=1.0)
    ap.add_argument("--seconds", type=float, default=600)
    ap.add_argument("--pid", action="append", default=[])
    ap.add_argument("--summary")
    ap.add_argument("--skip-s", type=float, default=5.0)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if a.summary:
        print(json.dumps(summary(a.summary, a.skip_s), indent=1))
        return 0
    pids = dict(x.split("=", 1) for x in a.pid)
    sample(a.out, a.period, a.seconds, {k: int(v) for k, v in pids.items()})
    return 0


if __name__ == "__main__":
    sys.exit(main())
