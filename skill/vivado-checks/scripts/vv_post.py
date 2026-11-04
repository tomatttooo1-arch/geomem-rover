#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# vv_post.py —— vivado-checks 后处理（本项目新写，非上游文件）。只做计数与摘要，不改写任何 Vivado 输出。
#   vv_post.py timing <timing_summary.rpt>      解析 "Design Timing Summary" 表那一行
#   vv_post.py lint   <linter.csv> <vivado.log>  按规则计数；对账 "Total of X linter message(s)"
#   vv_post.py elab   <messages.csv> <ref_dir>   按严重度/ID 计数，并标出知识库 resolution 文件
import csv
import os
import re
import sys
from collections import Counter



def timing(rpt):
    lines = open(rpt, errors="replace").read().splitlines()
    for i, l in enumerate(lines):
        if l.strip().startswith("WNS(ns)") and "TNS Failing Endpoints" in l:
            hdr = re.split(r"\s{2,}", l.strip())
            # 跳过 ----- 分隔行
            j = i + 1
            while j < len(lines) and (not lines[j].strip() or set(lines[j].strip()) <= set("- ")):
                j += 1
            vals = lines[j].split()
            d = dict(zip(hdr, vals))
            keys = [("WNS(ns)", "wns_ns"), ("TNS(ns)", "tns_ns"),
                    ("TNS Failing Endpoints", "tns_failing_endpoints"),
                    ("TNS Total Endpoints", "tns_total_endpoints"),
                    ("WHS(ns)", "whs_ns"), ("THS(ns)", "ths_ns"),
                    ("THS Failing Endpoints", "ths_failing_endpoints"),
                    ("WPWS(ns)", "wpws_ns"), ("TPWS Failing Endpoints", "tpws_failing_endpoints")]
            for k, o in keys:
                print(f"VV_SUMMARY {o}={d.get(k, 'NA')}")
            met = "met" if "All user specified timing constraints are met" in "\n".join(lines) else "NOT_met_or_unknown"
            print(f"VV_SUMMARY timing_constraints={met}")
            return 0
    print("VV_SUMMARY timing_parse=FAILED (no Design Timing Summary table)")
    return 1


def lint(csv_path, log):
    rows = [l for l in open(csv_path, errors="replace").read().splitlines() if l.strip()]
    total = None
    if os.path.exists(log):
        m = re.findall(r"Total of (\d+) linter message", open(log, errors="replace").read())
        total = int(m[-1]) if m else None
    c = Counter(re.sub(r"\s*\(T\d\)$", "", r.split(",")[0]) for r in rows)
    print(f"VV_SUMMARY lint_csv_rows={len(rows)}")
    print(f"VV_SUMMARY lint_vivado_total={total if total is not None else 'NA'}")
    if total is not None and total != len(rows):
        print("VV_SUMMARY lint_count_check=MISMATCH (见 SKILL.md：rpt 行数与 Vivado 总数不符时按上游回退规则读 rpt)")
    elif total is not None:
        print("VV_SUMMARY lint_count_check=OK")
    print("VV_SUMMARY lint_by_rule=" + (";".join(f"{k}:{v}" for k, v in sorted(c.items())) or "none"))
    return 0


def elab(csv_path, ref_dir):
    with open(csv_path, newline="", errors="replace") as f:
        rows = list(csv.DictReader(f))
    sev = Counter(r.get("Severity", "") for r in rows)
    print("VV_SUMMARY elab_by_severity=" + ";".join(f"{k}:{v}" for k, v in sorted(sev.items())))
    ids = Counter((r.get("Severity", ""), r.get("Msg_ID", "")) for r in rows if r.get("Severity") != "INFO")
    res_dir = os.path.join(ref_dir, "vivado-rtl-elaboration-analysis", "resolution")
    out = []
    for (s, i), n in sorted(ids.items()):
        rf = os.path.join(res_dir, f"8-{i}.md")
        has = "resolution/8-%s.md" % i if os.path.exists(rf) else "NO_FILE"
        out.append(f"{s}:8-{i}x{n}[{has}]")
    print("VV_SUMMARY elab_ids=" + (" ".join(out) or "none"))
    return 0


if __name__ == "__main__":
    mode = sys.argv[1]
    sys.exit({"timing": lambda: timing(sys.argv[2]),
              "lint": lambda: lint(sys.argv[2], sys.argv[3]),
              "elab": lambda: elab(sys.argv[2], sys.argv[3])}[mode]())
