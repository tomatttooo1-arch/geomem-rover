#!/usr/bin/env bash
# 用法：build.sh <工程名> <向量文件> "<额外综合源>" <step>...   （step: csim csynth cosim impl）
# 构建目录：$DP_BUILD_ROOT（默认 runs/nationals-prelim-20261102/DETPOST1/build）/<工程名>
set -e
here="$(cd "$(dirname "$0")" && pwd)"
name=$1; vec=$2; ksrc=$3; shift 3
root=${DP_BUILD_ROOT:-${FPGA_RUN}/DETPOST1/build}
mkdir -p "$root"
# 先把源码快照进构建目录再构建（构建中途改工作区源码不会混进本次结果；DETPOST1 第 0 步吃过这个亏）
snap="$root/${name}_src"; rm -rf "$snap"; mkdir -p "$snap/hls1"
cp "$here"/*.cpp "$here"/*.h "$here"/*.tcl "$snap/hls1/"; cp "$here"/../detpost.h "$here"/../detpost_ref.cpp "$here"/../detpost_host.cpp "$snap/"
cp "$vec" "$snap/"
ksrc=$(echo "$ksrc" | sed "s#$here/\.\./#$snap/#g; s#$here/#$snap/hls1/#g")
export DP_SRC="$snap/hls1" DP_BUILD="$root" DP_NAME="$name" DP_KSRC="$ksrc" DP_VEC="$snap/$(basename "$vec")"
source /data1/Xilinx/Vitis_HLS/2022.2/settings64.sh >/dev/null 2>&1
(cd "$here/.." && git rev-parse HEAD && git status --short .) > "$snap/GIT_STATE.txt" 2>&1 || true
exec ${FPGA_HOME}/dev1/.claude-accounts/claude_dev13/skills/fpga-hls/scripts/hls_run.sh --tcl "$here/run_hls.tcl" --argv "$DP_VEC" "$root/$name" sol1 "$@"
