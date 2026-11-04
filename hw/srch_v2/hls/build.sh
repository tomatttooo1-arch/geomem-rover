#!/usr/bin/env bash
# 用法：build.sh <工程名> <step>...（csim csynth cosim impl）。构建根 $SR_BUILD_ROOT（默认 runs/.../SRCHK6/build）。
set -e
here="$(cd "$(dirname "$0")" && pwd)"
name=$1; shift
run=${FPGA_RUN}
root=${SR_BUILD_ROOT:-$run/SRCHK6/build}
vec=${SR_VEC:-"$run/SRCHK1/data/trace_v1.bin $run/SRCHK1/data/trace_v2.bin"}
mkdir -p "$root"
snap="$root/${name}_src"; rm -rf "$snap"; mkdir -p "$snap/srch6" "$snap/srch"
cp "$here"/../srch6_core.cpp "$here"/../srch6_urom.h "$here"/run_hls.tcl "$snap/"
cp "$here"/../../srch/srch_core.h "$here"/../../srch/srch_tb.cpp "$snap/"
cp "$here"/../../srch/srch_core.h "$snap/srch/"
(cd "$here" && git rev-parse HEAD && git status --short ..) > "$snap/GIT_STATE.txt" 2>&1 || true
sed -i 's#"../srch/srch_core.h"#"srch_core.h"#' "$snap/srch6_core.cpp"
export SR_SRC="$snap" SR_BUILD="$root" SR_NAME="$name"
source /data1/Xilinx/Vitis_HLS/2022.2/settings64.sh >/dev/null 2>&1
exec ${FPGA_HOME}/dev1/.claude-global/skills/fpga-hls/scripts/hls_run.sh ${SR_HLSOPT} --tcl "$snap/run_hls.tcl" --argv "$vec" "$root/$name" sol1 "$@"
