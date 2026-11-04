#Copyright (C) 2026, Advanced Micro Devices, Inc. All rights reserved.
#SPDX-License-Identifier: MIT
# Modified for Vitis HLS 2022.2 classic flow by the contest team, 2026-11-02
#   - fallback when no Vitis Unified component is found: classic Tcl project
#     (<proj>/hls.app with exactly one solution -> <proj>/<solution>), or the path is
#     itself a classic solution dir (parent has hls.app). Upstream resolution unchanged.
from __future__ import annotations

from typing import Optional

from .file_path_util import join_and_normalize
from .file_util import read_file
from .vitis_comp_getter import (
    find_hls_cfg_file,
    get_vitis_comp,
    get_work_dir,
    is_hls_component,
)

from .cfg_parser import get_cfg_dict, get_hls_top


def get_solution_folder(component_location: str) -> Optional[str]:
    work_dir = _get_hls_work_dir_path(component_location)
    if work_dir:
        return join_and_normalize(work_dir, "hls")
    return _classic_solution_folder(component_location)


def _classic_solution_folder(location: str) -> Optional[str]:
    import os
    import re

    app = os.path.join(location, "hls.app")
    if os.path.isfile(app):
        names = re.findall(r'<solution\b[^>]*\bname="([^"]*)"', read_file(app))
        if len(names) == 1 and os.path.isdir(os.path.join(location, names[0])):
            return join_and_normalize(location, names[0])
        return None  # zero or several solutions: caller must pass the solution dir
    parent_app = os.path.join(os.path.dirname(os.path.normpath(location)), "hls.app")
    if os.path.isfile(parent_app):
        return join_and_normalize(location)
    return None


def _get_hls_work_dir_path(component_location: str) -> Optional[str]:
    vitis_comp = get_vitis_comp(component_location)
    if not vitis_comp:
        return None
    if not is_hls_component(vitis_comp):
        return None
    work_dir = get_work_dir(vitis_comp)
    if work_dir:
        return join_and_normalize(component_location, work_dir)
    hls_cfg = find_hls_cfg_file(component_location, vitis_comp)
    if hls_cfg:
        top = get_hls_top(get_cfg_dict(read_file(hls_cfg)))
        if top:
            return join_and_normalize(component_location, top)
    return None
