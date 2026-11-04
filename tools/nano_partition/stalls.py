#!/usr/bin/env python3
"""N5 stall events and their attribution (docs/design/NANO_PARTITION_LOAD_20260927.md §3).

usage: stalls.py <run_dir> [--windows L4,L5] [--json]
Stall event: in a window, consumer status receipt gap > 500 ms or detector completion gap > 500 ms;
gaps overlapping or < 1 s apart are merged. Attribution from probe rows in [start - 1 s, end + 1 s]:
  io     : mmcblk0 io_ticks increase >= 50 % of the event length and some row with procs_blocked >= 1
  non_io : io_ticks increase < 10 % of the event length and every row procs_blocked == 0
  undetermined otherwise, or when the probe has fewer than half of its expected rows there.
Probe validity: the IOCTL window (dd to the SD card) must show io_ticks >= 50 % busy and procs_blocked >= 1
(positive), L0 must show io_ticks < 10 % busy (negative); otherwise every "io" verdict becomes undetermined.
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nano_fullload"))
from summarize import load_phases  # noqa: E402

GAP_NS = 500_000_000
MERGE_NS = 1_000_000_000
PAD_NS = 1_000_000_000


def load_probe(run):
    rows = []
    p = run / "probe.jsonl"
    if not p.exists():
        return None, rows
    with open(p) as f:
        head = json.loads(f.readline())
        for line in f:
            try:
                rows.append(json.loads(line))
            except ValueError:
                pass
    return head, rows


def clock_offset(run):
    """wall_ns - mono_ns, from the probe (both clocks per row); falls back to the meter."""
    _, rows = load_probe(run)
    if rows:
        return rows[0][1] - rows[0][0]
    p = run / "meter" / "images.jsonl"
    for line in open(p):
        if line.startswith('["'):
            _, _, rx, rxm = json.loads(line)
            return rx - rxm
    raise SystemExit("no clock offset")


def gaps(run, off):
    """(start_wall_ns, end_wall_ns, source) for every gap > 500 ms."""
    out = []
    p = run / "consumer" / "status.jsonl"
    if p.exists():
        rx = []
        for line in open(p):
            try:
                rx.append(json.loads(line)[0] + off)
            except ValueError:
                pass
        out += [(a, b, "status") for a, b in zip(rx, rx[1:]) if b - a > GAP_NS]
    p = run / "det.jsonl"
    if p.exists():
        done = []
        for line in open(p):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if not r.get("warmup"):
                done.append(int(r["done_us"]) * 1000)
        out += [(a, b, "detector") for a, b in zip(done, done[1:]) if b - a > GAP_NS]
    return sorted(out)


def merge(gs):
    ev = []
    for a, b, src in gs:
        if ev and a - ev[-1]["end"] < MERGE_NS:
            e = ev[-1]
            e["end"] = max(e["end"], b)
            e["sources"][src] = max(e["sources"].get(src, 0), (b - a) / 1e6)
        else:
            ev.append({"start": a, "end": b, "sources": {src: (b - a) / 1e6}})
    return ev


def probe_span(rows, off, a, b, interval):
    sel = [r for r in rows if a <= r[1] <= b]
    if len(sel) < 2:
        return {"rows": len(sel), "expected": int((b - a) / 1e9 / interval)}
    d0, d1 = sel[0][5], sel[-1][5]
    span_ms = (sel[-1][1] - sel[0][1]) / 1e6
    gaps_ms = [(y[1] - x[1]) / 1e6 for x, y in zip(sel, sel[1:])]
    dnames = sorted({"%s:%s" % (t[1], t[2]) for r in sel if r[8] for t in r[8]})
    return {"rows": len(sel), "expected": int((b - a) / 1e9 / interval),
            "span_ms": round(span_ms, 1), "probe_max_gap_ms": round(max(gaps_ms), 1),
            "io_ticks_ms": (d1[5] - d0[5]) if d0 and d1 else None,
            "io_busy_frac": round((d1[5] - d0[5]) / span_ms, 3) if d0 and d1 and span_ms > 0 else None,
            "sectors_written": (d1[3] - d0[3]) if d0 and d1 else None,
            "writes": (d1[2] - d0[2]) if d0 and d1 else None,
            "in_flight_max": max(r[5][4] for r in sel if r[5]),
            "procs_blocked_max": max(r[3] for r in sel),
            "procs_blocked_rows": sum(1 for r in sel if r[3] >= 1),
            "procs_running_max": max(r[2] for r in sel),
            "dirty_kb_max": max(r[6] for r in sel), "writeback_kb_max": max(r[7] for r in sel),
            "d_tasks": dnames[:30]}


def verdict(ev_len_ms, pr, probe_ok):
    if pr.get("io_ticks_ms") is None or pr["rows"] < 0.5 * max(1, pr["expected"]):
        return "undetermined"
    frac = pr["io_ticks_ms"] / ev_len_ms
    if frac >= 0.5 and pr["procs_blocked_rows"] >= 1:
        return "io" if probe_ok else "undetermined"
    if frac < 0.1 and pr["procs_blocked_rows"] == 0:
        return "non_io"
    return "undetermined"


def analyse(run, windows):
    marks, wins = load_phases(run)
    if "start IOCTL" in marks and "end IOCTL" in marks:  # summarize's window regex only takes L* labels
        wins["IOCTL"] = (marks["start IOCTL"][0], marks["end IOCTL"][0])
    head, rows = load_probe(run)
    interval = head["interval"] if head else 0.1
    off = clock_offset(run)
    res = {"controls": {}, "windows": {}, "events": []}
    if "IOCTL" in wins:
        a, b = wins["IOCTL"]
        res["controls"]["positive_ioctl"] = pr = probe_span(rows, off, int(a * 1e9), int(b * 1e9), interval)
        res["controls"]["positive_ok"] = bool(pr.get("io_busy_frac") is not None and pr["io_busy_frac"] >= 0.5
                                              and pr["procs_blocked_rows"] >= 1)
    if "L0" in wins:
        a, b = wins["L0"]
        res["controls"]["negative_L0"] = pr = probe_span(rows, off, int(a * 1e9), int(b * 1e9), interval)
        res["controls"]["negative_ok"] = bool(pr.get("io_busy_frac") is not None and pr["io_busy_frac"] < 0.1)
    probe_ok = res["controls"].get("positive_ok", False) and res["controls"].get("negative_ok", False)
    res["controls"]["probe_ok"] = probe_ok
    evs = merge(gaps(run, off))
    for L, (a, b) in wins.items():
        if L == "IOCTL":
            continue
        a_ns, b_ns = int(a * 1e9), int(b * 1e9)
        pr = probe_span(rows, off, a_ns, b_ns, interval)
        res["windows"][L] = {k: pr.get(k) for k in ("io_busy_frac", "sectors_written", "writes", "procs_blocked_rows",
                                                     "procs_blocked_max", "in_flight_max", "dirty_kb_max",
                                                     "writeback_kb_max", "probe_max_gap_ms", "rows", "expected")}
        res["windows"][L]["stalls"] = 0
        for e in evs:
            if not (a_ns <= e["start"] < b_ns):
                continue
            res["windows"][L]["stalls"] += 1
            if windows and L not in windows:
                continue
            ln = (e["end"] - e["start"]) / 1e6
            p = probe_span(rows, off, e["start"] - PAD_NS, e["end"] + PAD_NS, interval)
            res["events"].append({"window": L, "start_wall": round(e["start"] / 1e9, 3),
                                  "len_ms": round(ln, 1), "sources_gap_ms": {k: round(v, 1) for k, v in e["sources"].items()},
                                  "probe": p, "verdict": verdict(ln, p, probe_ok)})
    res["outside_windows"] = sum(1 for e in evs if not any(int(a * 1e9) <= e["start"] < int(b * 1e9)
                                                          for L, (a, b) in wins.items() if L != "IOCTL"))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--windows", default="L4,L5")
    a = ap.parse_args()
    run = Path(a.run)
    res = analyse(run, set(a.windows.split(",")) if a.windows else None)
    (run / "stalls.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
    print(json.dumps(res, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
