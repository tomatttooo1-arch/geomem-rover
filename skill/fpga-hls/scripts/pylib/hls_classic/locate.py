# Copyright (C) 2026 the contest team. SPDX-License-Identifier: MIT
# Modified for Vitis HLS 2022.2 classic flow by the contest team, 2026-11-02
# New file (not in upstream Ross): solution-directory resolution for the classic
# Tcl project layout (<proj_dir>/hls.app + <proj_dir>/<solution>/), falling back to
# the upstream Vitis-Unified resolution (vitis-comp.json / hls_config.cfg -> <work_dir>/hls).
from __future__ import annotations

import html
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from hls_common.solution_folder_getter import get_solution_folder


class LocateError(Exception):
    pass


@dataclass
class HlsApp:
    path: str
    name: str = ""
    top: str = ""
    solutions: list[str] = field(default_factory=list)
    source_files: list[dict] = field(default_factory=list)
    testbench_files: list[dict] = field(default_factory=list)
    csim_argv: str = ""


@dataclass
class Located:
    solution_dir: str
    layout: str  # "classic" | "unified" | "bare"
    project_dir: str = ""
    solution_name: str = ""
    hls_app: Optional[HlsApp] = None


def _attr(tag: str, name: str) -> str:
    m = re.search(r'\b' + re.escape(name) + r'="([^"]*)"', tag)
    return html.unescape(m.group(1)) if m else ""


def parse_hls_app(path: str) -> HlsApp:
    text = Path(path).read_text(errors="ignore")
    app = HlsApp(path=path)
    m = re.search(r"<AutoPilot:project\b[^>]*>", text)
    if m:
        app.name = _attr(m.group(0), "name")
        app.top = _attr(m.group(0), "top")
    for tag in re.findall(r"<file\b[^>]*/?>", text):
        entry = {
            "name": _attr(tag, "name"),
            "cflags": _attr(tag, "cflags"),
            "csimflags": _attr(tag, "csimflags"),
            "blackbox": _attr(tag, "blackbox"),
        }
        tb = _attr(tag, "tb")
        (app.testbench_files if tb in ("1", "true") else app.source_files).append(entry)
    app.solutions = [_attr(t, "name") for t in re.findall(r"<solution\b[^>]*/?>", text)]
    sm = re.search(r"<Simulation\b[^>]*>", text)
    if sm:
        app.csim_argv = _attr(sm.group(0), "argv")
    return app


def parse_solution_aps(path: str) -> dict:
    """Part, clock period, cosim argv from <solution>/<solution>.aps (2022.2)."""
    out = {"part": "", "clock_period_ns": "", "cosim_argv": "", "flow_target": ""}
    p = Path(path)
    if not p.exists():
        return out
    text = p.read_text(errors="ignore")
    m = re.search(r'<TargetInfo value="([^"]*)"', text)
    if m:
        out["part"] = m.group(1).replace(":", "")
    m = re.search(r'<clock\b[^>]*period="([^"]*)"', text)
    if m:
        out["clock_period_ns"] = m.group(1)
    m = re.search(r"<Simulation\b[^>]*>", text)
    if m:
        out["cosim_argv"] = _attr(m.group(0), "argv")
    m = re.search(r'<flowTarget value="([^"]*)"', text)
    if m:
        out["flow_target"] = m.group(1)
    return out


def _looks_like_solution_dir(p: Path) -> bool:
    if (p / f"{p.name}.aps").exists():
        return True
    return any((p / d).is_dir() for d in ("syn", "sim", "csim", "impl", ".autopilot"))


def locate(path_str: str) -> Located:
    p = Path(path_str).resolve()
    if not p.is_dir():
        raise LocateError(f"路径不存在或不是目录: {path_str}")

    # 1) classic project dir: has hls.app
    if (p / "hls.app").exists():
        app = parse_hls_app(str(p / "hls.app"))
        sols = [s for s in app.solutions if s]
        if not sols:
            raise LocateError(f"{p}/hls.app 里没有 solution")
        if len(sols) > 1:
            raise LocateError(
                f"工程 {p} 有多个 solution: {', '.join(sols)}；请直接给 solution 目录，如 {p}/{sols[0]}"
            )
        sol = p / sols[0]
        if not sol.is_dir():
            raise LocateError(f"hls.app 列出的 solution 目录不存在: {sol}")
        return Located(str(sol), "classic", str(p), sols[0], app)

    # 2) classic solution dir: parent has hls.app listing this name, or <name>.aps present
    parent_app = p.parent / "hls.app"
    if parent_app.exists():
        app = parse_hls_app(str(parent_app))
        if p.name in app.solutions or (p / f"{p.name}.aps").exists():
            return Located(str(p), "classic", str(p.parent), p.name, app)

    # 3) upstream Vitis Unified layout (vitis-comp.json / hls_config.cfg) -> <work_dir>/hls
    try:
        uni = get_solution_folder(str(p))
    except Exception:
        uni = None
    if uni and Path(uni).is_dir():
        return Located(str(Path(uni)), "unified", str(p), "hls", None)

    # 4) a bare solution-like dir (e.g. a unified <work_dir>/hls given directly)
    if _looks_like_solution_dir(p):
        return Located(str(p), "bare", str(p.parent), p.name, None)

    if uni:
        raise LocateError(f"按 vitis-comp.json/hls_config.cfg 推出的 solution 目录不存在: {uni}（先跑 csynth）")
    raise LocateError(
        f"{p} 既不是 2022.2 工程目录（无 hls.app）、也不是 solution 目录、也不是 Vitis Unified 组件目录"
    )
