#!/usr/bin/env bash
# 宿主 g++ -O2 快速自检（不替代 HLS csim 门）：编当前 detpost_k.cpp + testbench，跑向量文件，与 golden 逐字节比。
# 用法：fastsim.sh <向量文件> <golden tb_out 目录> <工作目录>
set -e
here="$(cd "$(dirname "$0")" && pwd)"
vec="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"; gold=$2; wd=$3
mkdir -p "$wd"; cd "$wd"; rm -rf tb_out
g++ -O2 -std=c++14 -DDETPOST_CAND_MAX=256 -I/data1/Xilinx/Vitis_HLS/2022.2/include -I"$here" -o fastsim.exe \
  "$here/tb_detpost.cpp" "$here/detpost_k.cpp" "$here/../detpost_host.cpp" $DP_FASTSIM_EXTRA
./fastsim.exe "$vec"
python3 ${FPGA_HOME}/dev1/.claude-accounts/claude_dev13/skills/fpga-hls/scripts/bitexact_check.py "$gold" tb_out | tail -1
