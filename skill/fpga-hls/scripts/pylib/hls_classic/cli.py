# Copyright (C) 2026 the contest team. SPDX-License-Identifier: MIT
# Modified for Vitis HLS 2022.2 classic flow by the contest team, 2026-11-02
# New file (not in upstream Ross): single entry point used by hls_report.sh.
#   python3 -m hls_classic.cli <kind> <solution_dir|project_dir|component_dir>
# kinds: basic-info | synth | cosim | impl | dataflow
# Output: JSON on stdout. Exit codes: 0 ok, 2 usage, 3 report missing, 4 locate error.
from __future__ import annotations

import json
import os
import sys
from dataclasses import asdict
from pathlib import Path

from .locate import LocateError, Located, locate, parse_solution_aps
from .reports import MissingReport, cosim_summary, impl_summary, synth_summary

KINDS = ("basic-info", "synth", "cosim", "impl", "dataflow")


def _basic_info(loc: Located) -> dict:
    if loc.layout == "unified":
        from hls_component_basic_info.component_basic_info_getter import get_component_basic_info
        info = get_component_basic_info(loc.project_dir)
        return {"layout": "unified", **(asdict(info) if info else {})}
    if loc.hls_app is None:
        raise LocateError(f"{loc.solution_dir} 没有 hls.app / vitis-comp.json，无法给出工程基本信息")
    app = loc.hls_app
    aps = parse_solution_aps(str(Path(loc.solution_dir) / f"{loc.solution_name}.aps"))
    incs: list[str] = []
    for f in app.source_files + app.testbench_files:
        toks = (f["cflags"] + " " + f["csimflags"]).split()
        for i, t in enumerate(toks):
            if t == "-I" and i + 1 < len(toks):
                incs.append(toks[i + 1])
            elif t.startswith("-I") and len(t) > 2:
                incs.append(t[2:])
    root = os.getenv("XILINX_HLS") or "/tools/Xilinx/Vitis_HLS/2022.2"
    incs += [str(Path(root) / "include")]
    return {
        "layout": "classic",
        "component_name": app.name,
        "project_dir": loc.project_dir,
        "solution": loc.solution_name,
        "solution_dir": loc.solution_dir,
        "solutions": app.solutions,
        "top_function": app.top,
        "source_files": [f["name"] for f in app.source_files],
        "testbench_files": [f["name"] for f in app.testbench_files],
        "file_cflags": {f["name"]: f["cflags"] for f in app.source_files + app.testbench_files},
        "include_paths": list(dict.fromkeys(incs)),
        "part": aps["part"],
        "clock_period_ns": aps["clock_period_ns"],
        "flow_target": aps["flow_target"],
        "csim_argv": app.csim_argv,
        "cosim_argv": aps["cosim_argv"],
        "steps_done": {
            "csim": (Path(loc.solution_dir) / "csim").is_dir(),
            "csynth": (Path(loc.solution_dir) / "syn" / "report" / "csynth.rpt").exists(),
            "cosim": any((Path(loc.solution_dir) / "sim" / "report").glob("*_cosim.rpt")),
            "impl": any((Path(loc.solution_dir) / "impl" / "report").glob("*/export_impl.*")),
        },
    }


def _need_design_xml(sol: str) -> None:
    from hls_common.file_path_util import find_design_file_path
    if not find_design_file_path(sol):
        raise MissingReport("设计层次(.autopilot/db/*.design.xml)", "csynth", f"{sol}/.autopilot/db")


def run(kind: str, target: str) -> dict:
    loc = locate(target)
    sol = loc.solution_dir
    head = {"kind": kind, "solution_dir": sol, "layout": loc.layout}
    if kind == "basic-info":
        return {**head, "info": _basic_info(loc)}
    if kind == "synth":
        summary = synth_summary(sol)  # raises MissingReport
        from hls_synth_report.report_info_getter import get_report_info
        return {**head, "summary": summary, "upstream": asdict(get_report_info(sol))}
    if kind == "cosim":
        summary = cosim_summary(sol)
        if summary["status"] in ("NA", ""):
            raise MissingReport("有效协同仿真结果（报告里所有 RTL 行都是 NA）", "cosim", summary["source"])
        _need_design_xml(sol)
        from hls_cosim_report.report_info_getter import get_report_info
        ri = get_report_info(sol)
        return {
            **head,
            "summary": summary,
            "upstream": {
                "coSimulationReportContent": asdict(ri.performance_and_resource_estimates),
                "dataflowProcessAndChannelContent": [asdict(d) for d in ri.dataflow_infos],
            },
        }
    if kind == "impl":
        summary = impl_summary(sol)
        from hls_impl_report.report_info_getter import get_impl_info
        return {**head, "summary": summary, "upstream": asdict(get_impl_info(sol))}
    if kind == "dataflow":
        _need_design_xml(sol)
        from hls_dataflow.dataflow_info_getter import get_dataflow_infos
        infos = [asdict(d) for d in get_dataflow_infos(sol)]
        has_cosim = any((Path(sol) / "sim" / "report").glob("*_cosim.rpt"))
        return {
            **head,
            "note": (
                "设计中没有 dataflow 区域（design.xml 里无 dataflow 模块），这不是解析失败"
                if not infos else
                ("" if has_cosim else "未跑 cosim：通道/进程的 cosim 字段（阻塞时间、最大深度）为空，先跑 cosim 才有")
            ),
            "dataflow_infos": infos,
        }
    raise ValueError(kind)


def main(argv: list[str]) -> int:
    if len(argv) != 3 or argv[1] not in KINDS:
        print(f"Usage: hls_report.sh <{'|'.join(KINDS)}> <solution_dir|project_dir>", file=sys.stderr)
        return 2
    try:
        out = run(argv[1], argv[2])
    except MissingReport as e:
        msg = str(e)
        print(json.dumps({"kind": argv[1], "error": "missing_report", "message": msg,
                          "run_first": e.run_first, "path": e.path}, ensure_ascii=False, indent=2))
        print(msg, file=sys.stderr)
        return 3
    except LocateError as e:
        print(json.dumps({"kind": argv[1], "error": "locate", "message": str(e)}, ensure_ascii=False, indent=2))
        print(str(e), file=sys.stderr)
        return 4
    print(json.dumps(out, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
