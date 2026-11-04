#!/usr/bin/env bash
# 用法：build.sh <工程名> <step>...   （step: csim csynth cosim impl；其余选项透传给 hls_run.sh）
# 向量：SRCHK1/data/trace_v1.bin trace_v2.bin（DESIGN §10.2，可用 SR_VEC 覆盖，空格分隔）。
# 构建目录：$SR_BUILD_ROOT（默认 runs/nationals-prelim-20261102/SRCHK3/build）/<工程名>；先把源码快照进构建目录。
set -e
here="$(cd "$(dirname "$0")" && pwd)"
name=$1; shift
run=${FPGA_RUN}
root=${SR_BUILD_ROOT:-$run/SRCHK5/build}
vec=${SR_VEC:-"$run/SRCHK1/data/trace_v1.bin $run/SRCHK1/data/trace_v2.bin"}
mkdir -p "$root"
snap="$root/${name}_src"; rm -rf "$snap"; mkdir -p "$snap"
cp "$here"/../srch_core.cpp "$here"/../srch_core.h "$here"/../srch_tb.cpp "$here"/run_hls.tcl "$snap/"
(cd "$here" && git rev-parse HEAD && git status --short ..) > "$snap/GIT_STATE.txt" 2>&1 || true
export SR_SRC="$snap" SR_BUILD="$root" SR_NAME="$name"
source /data1/Xilinx/Vitis_HLS/2022.2/settings64.sh >/dev/null 2>&1
exec ${FPGA_HOME}/dev1/.claude-global/skills/fpga-hls/scripts/hls_run.sh ${SR_HLSOPT} --tcl "$snap/run_hls.tcl" --argv "$vec" "$root/$name" sol1 "$@"
