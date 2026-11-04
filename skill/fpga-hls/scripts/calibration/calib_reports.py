#!/usr/bin/env python3
# Copyright (C) 2026 the contest team. SPDX-License-Identifier: MIT
# Calibration of hls_report.sh (synth / cosim) against the tool's own TEXT reports.
# hls_report.sh "summary" reads XML (csynth.xml, <module>_csynth.xml); this script reads
# the human-readable .rpt files with an independent regex parser and compares field by field.
# cosim: hls_report.sh "upstream" top row comes from .autopilot/db process/loop zips;
# compared against <top>_cosim.rpt numbers.
#
# Usage: calib_reports.py <solution_dir> <synth.json> <cosim.json|-> [--perturb]
#   --perturb : negative control - corrupt one actual value (top DSP +1) before comparing;
#               the run MUST then report a mismatch.
# Prints a markdown table; exit 0 all match, 1 any mismatch.
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

NA = {"-", "?", "undef", "", None, "NA"}


def norm(v):
    if v is None:
        return "NA"
    s = str(v).strip()
    if s in NA:
        return "NA"
    m = re.match(r"^~?(\d+)\s*\(", s)  # "43 (3%)" -> 43
    if m:
        return m.group(1)
    return s


def norm_res(v):
    n = norm(v)
    return "0" if n == "NA" else n


def table_rows(text: str, start_marker: str):
    i = text.find(start_marker)
    if i < 0:
        return []
    rows = []
    for line in text[i:].splitlines()[1:]:
        if line.strip().startswith("|"):
            rows.append(line)
        elif rows and not line.strip().startswith("+"):
            break
    return rows


def parse_top_rpt(rpt: Path) -> dict:
    t = rpt.read_text(errors="ignore")
    out = {}
    m = re.search(r"\|ap_clk\s*\|\s*([\d.]+) ns\|\s*([\d.]+) ns\|", t)
    out["target_clock_ns"], out["estimated_clock_ns"] = (m.group(1), m.group(2)) if m else ("?", "?")
    lat = re.search(r"\+ Latency:.*?\| *min *\| *max *\|.*?\n\s*\+[-+]+\+\n\s*\|([^\n]*)\|", t, re.S)
    if lat:
        c = [x.strip() for x in lat.group(1).split("|")]
        out["latency_best"], out["latency_worst"] = c[0], c[1]
        out["interval_min"], out["interval_max"] = c[4], c[5]
    tot = re.search(r"== Utilization Estimates.*?\|Total\s*\|([^\n]*)\|", t, re.S)
    if tot:
        c = [x.strip() for x in tot.group(1).split("|")]
        out["res"] = dict(zip(("BRAM_18K", "DSP", "FF", "LUT", "URAM"), c[:5]))
    return out


def parse_summary_table(rpt: Path):
    """csynth.rpt Performance & Resource Estimates table -> modules, loops keyed by (module, loop)."""
    t = rpt.read_text(errors="ignore")
    rows = [r for r in table_rows(t, "Performance & Resource Estimates")
            if len(r.split("|")) > 2 and re.match(r"\s*[+o]\s", r.split("|")[1])]
    mods, loops = {}, {}
    stack = []  # (indent, kind, name)
    for r in rows:
        cells = r.split("|")[1:-1]
        namecell = cells[0]
        indent = len(namecell) - len(namecell.lstrip(" "))
        kind = namecell.strip()[0]
        name = namecell.strip()[1:].strip().rstrip("*").strip()
        c = [x.strip() for x in cells]
        # cols: name, issue, slack, lat_cyc, lat_ns, iter_lat, interval, trip, pipelined, BRAM, DSP, FF, LUT, URAM
        while stack and stack[-1][0] >= indent:
            stack.pop()
        if kind == "+":
            mods[name] = {"latency": c[3], "interval": c[6], "res": dict(zip(("BRAM_18K", "DSP", "FF", "LUT", "URAM"), c[9:14]))}
        else:
            parent_mod = next((n for (_, k, n) in reversed(stack) if k == "+"), "?")
            loops[(parent_mod, name)] = {"latency": c[3], "iter_lat": c[5], "interval": c[6], "trip": c[7], "pipelined": c[8]}
        stack.append((indent, kind, name))
    return mods, loops


def flatten_loops(loops, out, mod):
    for l in loops:
        out[(mod, l["name"])] = l
        flatten_loops(l.get("child_loops", []), out, mod)


def main(argv):
    perturb = "--perturb" in argv
    argv = [a for a in argv if a != "--perturb"]
    sol, synth_json, cosim_json = Path(argv[1]), argv[2], argv[3]
    S = json.load(open(synth_json))["summary"]
    if perturb:
        S["resources"]["DSP"] = str(int(S["resources"]["DSP"]) + 1)
    rep = sol / "syn" / "report"
    top = S["top"]
    checks = []

    def chk(item, exp, act, f=norm):
        checks.append((item, str(exp), str(act), f(exp) == f(act)))

    tr = parse_top_rpt(rep / f"{top}_csynth.rpt")
    chk("top 目标时钟 ns", tr["target_clock_ns"], S["target_clock_ns"])
    chk("top 估计时钟 ns", tr["estimated_clock_ns"], S["estimated_clock_ns"])
    chk("top 延迟 min (cycles)", tr.get("latency_best"), S["latency_best"])
    chk("top 延迟 max (cycles)", tr.get("latency_worst"), S["latency_worst"])
    chk("top II min", tr.get("interval_min"), S["interval_min"])
    chk("top II max", tr.get("interval_max"), S["interval_max"])
    for k in ("BRAM_18K", "DSP", "FF", "LUT", "URAM"):
        chk(f"top {k}", tr["res"][k], S["resources"][k], norm_res)

    mods_rpt, loops_rpt = parse_summary_table(rep / "csynth.rpt")
    mods_act = {m["name"]: m for m in S["modules"]}
    loops_act = {}
    for m in S["modules"]:
        flatten_loops(m.get("loops", []), loops_act, m["name"])
    for name, e in mods_rpt.items():
        a = mods_act.get(name)
        if a is None:
            checks.append((f"模块 {name}", "存在", "缺", False)); continue
        chk(f"模块 {name} 延迟(cycles)", e["latency"], a["latency_worst"])
        chk(f"模块 {name} Interval", e["interval"], a["interval_max"])
        for k in ("BRAM_18K", "DSP", "FF", "LUT", "URAM"):
            chk(f"模块 {name} {k}", e["res"][k], a["resources"][k], norm_res)
    for (mod, ln), e in loops_rpt.items():
        a = loops_act.get((mod, ln))
        if a is None:
            checks.append((f"循环 {mod}/{ln}", "存在", "缺", False)); continue
        chk(f"循环 {mod}/{ln} II", e["interval"], a["pipeline_ii"])
        chk(f"循环 {mod}/{ln} 迭代延迟", e["iter_lat"], a["iteration_latency_rpt"])
        chk(f"循环 {mod}/{ln} 延迟max", e["latency"], a["latency_max"])
        chk(f"循环 {mod}/{ln} trip max", e["trip"], a["trip_count_max"])
    extra = set(loops_act) - set(loops_rpt)
    for k in sorted(extra):
        checks.append((f"循环 {k[0]}/{k[1]}", "rpt 无此行", "XML 多出", False))

    if cosim_json != "-":
        C = json.load(open(cosim_json))
        rpt = next((sol / "sim" / "report").glob("*_cosim.rpt"))
        row = None
        for line in rpt.read_text(errors="ignore").splitlines():
            c = [x.strip() for x in line.split("|")]
            if len(c) > 10 and c[1] in ("Verilog", "VHDL") and c[2] != "NA":
                row = c
        cs = C["summary"]["simulated"][0]
        chk("cosim 状态", row[2], cs["status"])
        for i, k in ((3, "latency_min"), (4, "latency_avg"), (5, "latency_max"), (6, "interval_min"), (7, "interval_avg"), (8, "interval_max"), (9, "total_execution_cycles")):
            chk(f"cosim(rpt→summary) {k}", row[i], cs[k])
        up = C["upstream"]["coSimulationReportContent"]["rows"][0]
        for i, k in ((3, "min_latency"), (4, "avg_latency"), (5, "max_latency")):
            chk(f"cosim(rpt→upstream process.zip) {k}", row[i], up[k])
        chk("cosim(rpt→upstream) total_execution_time", row[9], up["total_execution_time"])

    ok = all(c[3] for c in checks)
    print(f"| 项 | 期望（文本 .rpt） | 实际（hls_report.sh） | 结果 |\n|---|---|---|---|")
    for item, e, a, good in checks:
        print(f"| {item} | {e} | {a} | {'一致' if good else '**不一致**'} |")
    print(f"\n共 {len(checks)} 项，不一致 {sum(1 for c in checks if not c[3])} 项 → {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
