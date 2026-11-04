#!/bin/bash
# Regenerate hw/m7/pl/hls/tb_frame_stream_fwdcov.h (forwarding-coverage frames of
# tb_frame_stream.cpp). The expected flags come from the Dlat2 reference simulator
# of G1d (runs/nationals-prelim-20261102/G1d/tools/g1d_ref.cpp, the G1c program;
# build: g++ -std=c++11 -O2 -ffp-contract=off -w -I hw/m7 g1d_ref.cpp hw/m7/frame_depth.cpp).
# usage: regen.sh G1D_REF_BINARY WORKDIR
set -e
ref=$1; wd=$2; here=$(cd "$(dirname "$0")" && pwd); top=$(cd "$here/../../.." && pwd)
mkdir -p "$wd"
python3 "$here/make_frames.py" "$wd/fwdcov"
n=$(wc -l < "$wd/fwdcov.names")
python3 -c "import sys; open(sys.argv[1],'wb').write(b'\xff'*(32768*int(sys.argv[2])))" "$wd/fwdcov.expect" $n
G1D_DUMP_DLAT2="$wd/fwdcov.expect" "$ref" "$wd/fwdcov.pack" - 0 $n fwdcov > "$wd/fwdcov.ref.csv"
python3 "$here/make_header.py" "$wd/fwdcov.pack" "$wd/fwdcov.expect" "$wd/fwdcov.names" "$top/hw/m7/pl/hls/tb_frame_stream_fwdcov.h"
