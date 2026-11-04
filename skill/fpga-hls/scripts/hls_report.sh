#!/usr/bin/env bash
# Copyright (C) 2026 the contest team. SPDX-License-Identifier: MIT
# Replaces the per-skill run.sh wrappers of AMD Ross (hls-*/scripts/run.sh,
# Copyright (C) 2026, Advanced Micro Devices, Inc. All rights reserved. MIT).
# Modified for Vitis HLS 2022.2 classic flow by the contest team, 2026-11-02
#
# Usage: hls_report.sh <kind> <solution_dir|project_dir|vitis_component_dir>
#   kind: basic-info | synth | cosim | impl | dataflow
# - solution dir (…/<proj>/solution)       -> used directly
# - classic project dir (has hls.app)       -> its single solution (several -> error listing them)
# - Vitis Unified component (vitis-comp.json / hls_config.cfg) -> <work_dir>/hls (upstream behaviour)
# Output: JSON. "summary" = structured numbers from 2022.2 XML/RPT; "upstream" = Ross parser output.
# Exit: 0 ok | 2 usage | 3 report missing ("缺 X 报告，先跑 Y") | 4 cannot locate solution
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ $# -eq 2 ] || { echo "usage: hls_report.sh <basic-info|synth|cosim|impl|dataflow> <solution_dir|project_dir>" >&2; exit 2; }
TARGET="$(cd "$2" 2>/dev/null && pwd)" || { echo "路径不存在: $2" >&2; exit 4; }
export PYTHONDONTWRITEBYTECODE=1
cd "$HERE/pylib" && exec python3 -m hls_classic.cli "$1" "$TARGET"
