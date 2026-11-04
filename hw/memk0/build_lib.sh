#!/bin/sh
# C model shared library for PLMemory (ctypes). Usage: build_lib.sh OUT.so [extra g++ flags]
# Same source as the HLS kernel; needs the Vitis HLS ap_int headers (header-only).
set -e
here=$(cd "$(dirname "$0")" && pwd)
inc=${XILINX_HLS:-/data1/Xilinx/Vitis_HLS/2022.2}/include
out=$1; shift
g++ -O2 -std=c++14 -fPIC -shared -Wall -Wno-unused-label -Wno-unused-function -Wno-unknown-pragmas -I"$inc" "$@" -o "$out" "$here/memk0_core.cpp"
