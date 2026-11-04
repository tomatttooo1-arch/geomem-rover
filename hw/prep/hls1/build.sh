#!/usr/bin/env bash
# 用法：build.sh <工程名> <向量文件> <step>...   （step: csim csynth cosim impl；其余选项透传给 hls_run.sh）
# 构建目录：$PP_BUILD_ROOT（默认 runs/nationals-prelim-20261102/PREP1/build）/<工程名>。先把源码快照进构建目录再构建。
set -e
here="$(cd "$(dirname "$0")" && pwd)"
name=$1; vec=$2; shift 2
root=${PP_BUILD_ROOT:-${FPGA_RUN}/PREP1/build}
mkdir -p "$root"
snap="$root/${name}_src"; rm -rf "$snap"; mkdir -p "$snap"
cp "$here"/*.cpp "$here"/*.h "$here"/*.tcl "$vec" "$snap/"
(cd "$here" && git rev-parse HEAD && git status --short .) > "$snap/GIT_STATE.txt" 2>&1 || true
export PP_SRC="$snap" PP_BUILD="$root" PP_NAME="$name" PP_VEC="$snap/$(basename "$vec")"
source /data1/Xilinx/Vitis_HLS/2022.2/settings64.sh >/dev/null 2>&1
exec ${FPGA_HOME}/dev1/.claude-accounts/claude_dev13/skills/fpga-hls/scripts/hls_run.sh --tcl "$snap/run_hls.tcl" --argv "$PP_VEC" "$root/$name" sol1 "$@"
