#!/usr/bin/env python3
#Copyright (C) 2026, Advanced Micro Devices, Inc. All rights reserved.
#SPDX-License-Identifier: MIT
# Modified for Vitis HLS 2022.2 classic flow by the contest team, 2026-11-02
#   - new primary mode --solution: reads 2022.2 report files directly
#     (syn/report/csynth.xml + csynth.rpt, sim/report/<top>_cosim.rpt,
#      impl/report/verilog/export_impl.xml) via hls_classic.reports
#   - real latency = cosim cycles x post-route clock period (falls back to the csynth
#     estimated period, and says so in column real_period_basis)
#   - missing data is an EMPTY cell + a "缺 X" line, never 0 / -1
#   - --report (upstream mode) kept, but the summary table is parsed by its header row,
#     because the 2022.2 column order (Issue Type | Slack | Latency(cycles) | Latency(ns) |
#     Iteration Latency | Interval | ...) differs from the newer layout the upstream
#     hard-coded, and the 2022.2 top row has no trailing '*' unless it is a dataflow top.
"""
qor_log.py  —  extract key QoR metrics and append a CSV row.

Usage:
    python3 qor_log.py --solution <solution_dir|project_dir> [--label LABEL] [--csv FILE]
    python3 qor_log.py --report "<csynth.rpt content>"       [--label LABEL] [--csv FILE]

Always prints a human-readable summary; appends one CSV row (default qor_history.csv).
"""

import argparse
import csv
import re
import sys
from datetime import datetime
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent / "pylib"))

DEFAULT_CSV = Path("qor_history.csv")

FIELDNAMES = [
    "label", "timestamp", "solution_dir", "top_module",
    "target_clock_ns", "est_clock_ns", "slack_ns",
    "lat_min_cyc", "lat_max_cyc", "interval_max", "pipelined",
    "bram", "dsp", "ff", "lut", "uram",
    "cosim_status", "cosim_lat_avg_cyc", "cosim_lat_max_cyc", "cosim_total_cyc",
    "impl_cp_ns", "impl_timing_met", "impl_lut", "impl_ff", "impl_dsp", "impl_bram", "impl_uram",
    "real_period_ns", "real_period_basis", "real_lat_max_ns", "real_total_ns",
]

_NA = {"", "-", "?", "undef", "NA", None}


def _num(v):
    if v in _NA:
        return None
    try:
        return float(str(v).strip())
    except ValueError:
        return None


def _first_int(v):
    if v in _NA:
        return ""
    m = re.match(r"\s*(\d+)", str(v))
    return m.group(1) if m else ""


def parse_summary_table(report_content: str) -> dict:
    """Top row of the 'Performance & Resource Estimates' table, columns located by header."""
    lines = [l.strip() for l in report_content.splitlines()]
    tbl = [l for l in lines if l.startswith("|")]
    hdr_idx = next((i for i, l in enumerate(tbl) if "Modules" in l), None)
    if hdr_idx is None or hdr_idx + 1 >= len(tbl):
        return {}
    # header spans two rows; join per column
    h1 = [c.strip() for c in tbl[hdr_idx].split("|")[1:-1]]
    h2 = [c.strip() for c in tbl[hdr_idx + 1].split("|")[1:-1]]
    heads = [f"{a} {b}".strip() for a, b in zip(h1, h2)]
    col = {}
    for i, h in enumerate(heads):
        hl = h.lower()
        if "modules" in hl: col["name"] = i
        elif hl == "slack": col["slack"] = i
        elif "latency" in hl and "cycles" in hl: col["lat_cyc"] = i
        elif "latency" in hl and "(ns)" in hl: col["lat_ns"] = i
        elif hl == "interval": col["interval"] = i
        elif "pipelined" in hl: col["pipelined"] = i
        elif hl in ("bram", "dsp", "ff", "lut", "uram"): col[hl] = i
    for l in tbl[hdr_idx + 2:]:
        cells = [c.strip() for c in l.split("|")[1:-1]]
        if len(cells) < len(heads) or not cells[0].startswith("+"):
            continue
        g = lambda k: cells[col[k]] if k in col else ""  # noqa: E731
        return {
            "top_module": cells[0].lstrip("+ ").rstrip("*").strip(),
            "slack_ns": "" if g("slack") in _NA else g("slack"),
            "lat_max_cyc": "" if g("lat_cyc") in _NA else g("lat_cyc"),
            "interval_max": "" if g("interval") in _NA else g("interval"),
            "pipelined": g("pipelined"),
            # in this table "-" in a resource column means "none used" -> 0
            **{k: ("0" if g(k) == "-" else _first_int(g(k))) for k in ("bram", "dsp", "ff", "lut", "uram")},
        }
    return {}


def from_solution(target: str) -> tuple[dict, list[str]]:
    from hls_classic.locate import locate
    from hls_classic.reports import MissingReport, cosim_summary, impl_summary, synth_summary

    loc = locate(target)
    sol = loc.solution_dir
    row: dict = {"solution_dir": sol}
    missing: list[str] = []
    try:
        s = synth_summary(sol)
        row.update({
            "top_module": s["top"], "target_clock_ns": s["target_clock_ns"],
            "est_clock_ns": s["estimated_clock_ns"],
            "lat_min_cyc": "" if s["latency_best"] in _NA else s["latency_best"],
            "lat_max_cyc": "" if s["latency_worst"] in _NA else s["latency_worst"],
            "interval_max": "" if s["interval_max"] in _NA else s["interval_max"],
            "bram": s["resources"].get("BRAM_18K", ""), "dsp": s["resources"].get("DSP", ""),
            "ff": s["resources"].get("FF", ""), "lut": s["resources"].get("LUT", ""),
            "uram": s["resources"].get("URAM", ""),
        })
        top_row = parse_summary_table((Path(sol) / "syn" / "report" / "csynth.rpt").read_text(errors="ignore"))
        row["slack_ns"] = top_row.get("slack_ns", "")
        row["pipelined"] = top_row.get("pipelined", "")
    except MissingReport as e:
        missing.append(str(e))
    try:
        c = cosim_summary(sol)
        if c["simulated"]:
            r = c["simulated"][0]
            row.update({
                "cosim_status": r["status"],
                "cosim_lat_avg_cyc": "" if r["latency_avg"] in _NA else r["latency_avg"],
                "cosim_lat_max_cyc": "" if r["latency_max"] in _NA else r["latency_max"],
                "cosim_total_cyc": "" if r["total_execution_cycles"] in _NA else r["total_execution_cycles"],
            })
        else:
            missing.append(f"cosim 报告里所有 RTL 行都是 NA（{c['source']}），先跑 cosim")
    except MissingReport as e:
        missing.append(str(e))
    try:
        im = impl_summary(sol)
        res = im.get("resources", {})
        row.update({
            "impl_cp_ns": im.get("cp_route") or im.get("achieved_clock_ns", ""),
            "impl_timing_met": im.get("timing_met", ""),
            "impl_lut": res.get("LUT", ""), "impl_ff": res.get("FF", ""), "impl_dsp": res.get("DSP", ""),
            "impl_bram": res.get("BRAM", res.get("BRAM_18K", "")), "impl_uram": res.get("URAM", ""),
        })
    except MissingReport as e:
        missing.append(str(e))

    period, basis = _num(row.get("impl_cp_ns")), "post-route"
    if period is None:
        period, basis = _num(row.get("est_clock_ns")), "csynth-estimate(无 impl，非布线后实测)"
    if period is None:
        basis = ""
    row["real_period_ns"] = f"{period:.3f}" if period is not None else ""
    row["real_period_basis"] = basis
    for src, dst in (("cosim_lat_max_cyc", "real_lat_max_ns"), ("cosim_total_cyc", "real_total_ns")):
        cyc = _num(row.get(src))
        row[dst] = f"{cyc * period:.1f}" if (cyc is not None and period is not None) else ""
    return row, missing


def main():
    ap = argparse.ArgumentParser(description="Log HLS QoR metrics (Vitis HLS 2022.2) to CSV")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--solution", help="solution dir or classic project dir (hls.app)")
    g.add_argument("--report", help="csynth.rpt content (string), upstream mode")
    ap.add_argument("--label", default=None)
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    label = args.label or datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = Path(args.csv) if args.csv else DEFAULT_CSV
    if args.solution:
        try:
            metrics, missing = from_solution(args.solution)
        except Exception as e:  # locate errors
            print(f"error: {e}", file=sys.stderr)
            return 4
    else:
        metrics = parse_summary_table(args.report)
        missing = [] if metrics else ["--report 内容里找不到 Performance & Resource Estimates 表的顶层行"]

    row = {k: "" for k in FIELDNAMES}
    row.update({k: v for k, v in metrics.items() if k in row})
    row["label"] = label
    row["timestamp"] = datetime.now().isoformat(timespec="seconds")

    def v(k):
        return row[k] if row[k] != "" else "缺"

    print(
        f"[{label}]  {v('top_module')}\n"
        f"  csynth : clock est {v('est_clock_ns')} ns / target {v('target_clock_ns')} ns, slack {v('slack_ns')}, "
        f"latency {v('lat_min_cyc')}..{v('lat_max_cyc')} cyc, interval {v('interval_max')}, pipelined {v('pipelined')}\n"
        f"           DSP={v('dsp')} FF={v('ff')} LUT={v('lut')} BRAM_18K={v('bram')} URAM={v('uram')}\n"
        f"  cosim  : {v('cosim_status')}, latency avg {v('cosim_lat_avg_cyc')} / max {v('cosim_lat_max_cyc')} cyc, "
        f"total {v('cosim_total_cyc')} cyc\n"
        f"  impl   : post-route CP {v('impl_cp_ns')} ns, timing met {v('impl_timing_met')}, "
        f"LUT={v('impl_lut')} FF={v('impl_ff')} DSP={v('impl_dsp')} BRAM={v('impl_bram')} URAM={v('impl_uram')}\n"
        f"  real   : period {v('real_period_ns')} ns [{v('real_period_basis')}] -> "
        f"latency max {v('real_lat_max_ns')} ns, total {v('real_total_ns')} ns"
    )
    for m in missing:
        print(f"  缺: {m}")

    if not row["top_module"]:
        print("  未写 CSV：没有综合结果可记（先跑 csynth）", file=sys.stderr)
        return 3
    write_header = not csv_path.exists()
    if not write_header:
        with open(csv_path, newline="") as f:
            existing = next(csv.reader(f), [])
        if existing != FIELDNAMES:
            print(f"  未写 CSV：{csv_path} 的表头与本版本列不一致（可能是上游旧版 qor_log 写的），换个 --csv 文件", file=sys.stderr)
            return 5
    with open(csv_path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDNAMES)
        if write_header:
            w.writeheader()
        w.writerow(row)
    print(f"  → appended to {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
