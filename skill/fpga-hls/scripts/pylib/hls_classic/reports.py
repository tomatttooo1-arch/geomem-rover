# Copyright (C) 2026 the contest team. SPDX-License-Identifier: MIT
# Modified for Vitis HLS 2022.2 classic flow by the contest team, 2026-11-02
# New file (not in upstream Ross): structured summaries of Vitis HLS 2022.2 reports.
#   synth : syn/report/csynth.xml (top) + syn/report/<module>_csynth.xml (modules, loops)
#   cosim : sim/report/<top>_cosim.rpt
#   impl  : impl/report/verilog/export_impl.xml (+ .rpt)
# Every value is copied as the tool wrote it (strings; "undef"/"?" kept verbatim);
# nothing is defaulted to 0. Missing files raise MissingReport.
from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional


class MissingReport(Exception):
    def __init__(self, what: str, run_first: str, path: str):
        super().__init__(f"缺 {what} 报告（{path}），先跑 {run_first}")
        self.what, self.run_first, self.path = what, run_first, path


RES_KEYS = ("BRAM_18K", "DSP", "FF", "LUT", "URAM")
_LOOP_FIELD_TAGS = {
    "Slack", "TripCount", "Latency", "AbsoluteTimeLatency", "IterationLatency",
    "PipelineII", "PipelineDepth", "InstanceList", "PipelineType",
}


def _txt(el: Optional[ET.Element]) -> str:
    return (el.text or "").strip() if el is not None else ""


def _val(el: Optional[ET.Element]):
    """Scalar text, or {"min":..,"max":..} for <range>; None if absent."""
    if el is None:
        return None
    rng = el.find("range")
    if rng is not None:
        return {"min": _txt(rng.find("min")), "max": _txt(rng.find("max"))}
    return _txt(el)


def _unknown_if_negative(s):
    # 2022.2 XML writes an unknown bound (variable trip count) as "-1" (e.g. <range><min>3</min>
    # <max>-1</max>); the .rpt prints "-". Report it as "undef", never as a number.
    try:
        return "undef" if s is not None and float(s) < 0 else s
    except ValueError:
        return s


def _minmax(v):
    if v is None:
        return (None, None)
    if isinstance(v, dict):
        return (_unknown_if_negative(v.get("min")), _unknown_if_negative(v.get("max")))
    v = _unknown_if_negative(v)
    return (v, v)


def _parse_xml(path: Path) -> ET.Element:
    return ET.fromstring(path.read_text(errors="ignore"))


def _resources(root: ET.Element, tag: str = "Resources") -> dict:
    res = root.find(f"AreaEstimates/{tag}")
    return {k: _txt(res.find(k)) for k in RES_KEYS} if res is not None else {}


def _loops(parent: Optional[ET.Element]) -> list[dict]:
    out = []
    if parent is None:
        return out
    for child in parent:
        if child.tag in _LOOP_FIELD_TAGS:
            continue
        if child.find("Latency") is None and child.find("TripCount") is None:
            continue
        lat = _minmax(_val(child.find("Latency")))
        trip = _minmax(_val(child.find("TripCount")))
        it = _minmax(_val(child.find("IterationLatency")))
        out.append({
            "name": child.tag,
            "trip_count_min": trip[0], "trip_count_max": trip[1],
            "latency_min": lat[0], "latency_max": lat[1],
            "iteration_latency_min": it[0], "iteration_latency_max": it[1],
            # what csynth.rpt prints in its "Iteration Latency" column: PipelineDepth for
            # pipelined loops (2022.2 XML has no IterationLatency there), else the max.
            "iteration_latency_rpt": (
                _val(child.find("PipelineDepth")) if child.find("PipelineII") is not None else it[1]
            ),
            "pipeline_ii": _val(child.find("PipelineII")),
            "pipeline_depth": _val(child.find("PipelineDepth")),
            "slack": _val(child.find("Slack")),
            "child_loops": _loops(child),
        })
    return out


def _perf(root: ET.Element) -> dict:
    pe = root.find("PerformanceEstimates")
    ol = pe.find("SummaryOfOverallLatency") if pe is not None else None
    g = lambda t: _unknown_if_negative(_txt(ol.find(t))) if ol is not None else None  # noqa: E731
    return {
        "estimated_clock_ns": _txt(root.find("PerformanceEstimates/SummaryOfTimingAnalysis/EstimatedClockPeriod")),
        "pipeline_type": _txt(root.find("PerformanceEstimates/PipelineType")),
        "latency_best": g("Best-caseLatency"),
        "latency_avg": g("Average-caseLatency"),
        "latency_worst": g("Worst-caseLatency"),
        "latency_best_abs": g("Best-caseRealTimeLatency"),
        "latency_worst_abs": g("Worst-caseRealTimeLatency"),
        "interval_min": g("Interval-min"),
        "interval_max": g("Interval-max"),
    }


def synth_summary(solution_dir: str) -> dict:
    rep = Path(solution_dir) / "syn" / "report"
    top_xml = rep / "csynth.xml"
    if not (rep / "csynth.rpt").exists():
        raise MissingReport("综合(csynth)", "csynth", str(rep / "csynth.rpt"))
    if not top_xml.exists():
        raise MissingReport("综合 XML(csynth.xml)", "csynth", str(top_xml))
    root = _parse_xml(top_xml)
    ua = root.find("UserAssignments")
    top = _txt(ua.find("TopModelName")) if ua is not None else ""
    out = {
        "source": str(top_xml),
        "version": _txt(root.find("ReportVersion/Version")),
        "top": top,
        "part": _txt(ua.find("Part")) if ua is not None else "",
        "target_clock_ns": _txt(ua.find("TargetClockPeriod")) if ua is not None else "",
        "clock_uncertainty_ns": _txt(ua.find("ClockUncertainty")) if ua is not None else "",
        **_perf(root),
        "resources": _resources(root),
        "available": _resources(root, "AvailableResources"),
        "modules": [],
    }
    for mx in sorted(rep.glob("*_csynth.xml")):
        name = mx.name[: -len("_csynth.xml")]
        try:
            mroot = _parse_xml(mx)
        except ET.ParseError as e:
            out["modules"].append({"name": name, "error": f"XML parse error: {e}"})
            continue
        out["modules"].append({
            "name": name,
            "is_top": name == top,
            **_perf(mroot),
            "resources": _resources(mroot),
            "loops": _loops(mroot.find("PerformanceEstimates/SummaryOfLoopLatency")),
        })
    return out


def cosim_summary(solution_dir: str) -> dict:
    rdir = Path(solution_dir) / "sim" / "report"
    rpts = sorted(rdir.glob("*_cosim.rpt")) if rdir.is_dir() else []
    if not rpts:
        raise MissingReport("协同仿真(cosim)", "cosim", str(rdir / "<top>_cosim.rpt"))
    rpt = rpts[0]
    rows = []
    for line in rpt.read_text(errors="ignore").splitlines():
        cols = [c.strip() for c in line.split("|")]
        if len(cols) >= 11 and cols[1] in ("Verilog", "VHDL", "SystemC"):
            rows.append({
                "rtl": cols[1], "status": cols[2],
                "latency_min": cols[3], "latency_avg": cols[4], "latency_max": cols[5],
                "interval_min": cols[6], "interval_avg": cols[7], "interval_max": cols[8],
                "total_execution_cycles": cols[9],
            })
    ran = [r for r in rows if r["status"] not in ("NA", "")]
    return {
        "source": str(rpt),
        "rows": rows,
        "simulated": ran,
        "status": (ran[0]["status"] if ran else "NA"),
    }


def _impl_paths(solution_dir: str) -> tuple[Path, Path]:
    base = Path(solution_dir) / "impl" / "report"
    for hdl in ("verilog", "vhdl"):
        x = base / hdl / "export_impl.xml"
        r = base / hdl / "export_impl.rpt"
        if x.exists() or r.exists():
            return x, r
    return base / "verilog" / "export_impl.xml", base / "verilog" / "export_impl.rpt"


def impl_summary(solution_dir: str) -> dict:
    x, r = _impl_paths(solution_dir)
    if not x.exists() and not r.exists():
        raise MissingReport("实现(impl, export_design -flow impl)", "impl", str(r))
    out: dict = {"source_xml": str(x) if x.exists() else "", "source_rpt": str(r) if r.exists() else ""}
    if x.exists():
        root = _parse_xml(x)
        tr = root.find("TimingReport")
        ar = root.find("AreaReport/Resources")
        av = root.find("AreaReport/AvailableResources")
        out["target_clock_ns"] = _txt(tr.find("TargetClockPeriod")) if tr is not None else ""
        out["achieved_clock_ns"] = _txt(tr.find("AchievedClockPeriod")) if tr is not None else ""
        out["timing_met"] = _txt(tr.find("TIMING_MET")) if tr is not None else ""
        for k in ("CP_FINAL", "CP_ROUTE", "CP_SYNTH", "CP_TARGET", "SLACK_FINAL", "WNS_FINAL", "TNS_FINAL"):
            if tr is not None and tr.find(k) is not None:
                out[k.lower()] = _txt(tr.find(k))
        out["resources"] = {c.tag: _txt(c) for c in ar} if ar is not None else {}
        out["available"] = {c.tag: _txt(c) for c in av} if av is not None else {}
    if r.exists():
        text = r.read_text(errors="ignore")
        m = re.search(r"\|\s*Post-Route\s*\|\s*([^|]+?)\s*\|", text)
        out["rpt_post_route_cp"] = m.group(1) if m else ""
    return out
