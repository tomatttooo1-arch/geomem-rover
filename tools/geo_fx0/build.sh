#!/bin/bash
# usage: build.sh SRCROOT [extra flags] -> build/fx_golden
set -e
D=${FPGA_RUN}/GEO_FX0
src=$1; shift
g++ -std=c++11 -O2 -ffp-contract=off -Wall -Wno-unused-function "$@" -I $src/hw/m7 $D/tools/fx_golden.cpp -lz -o $D/build/fx_golden
echo "$(date -Is) build fx_golden $* src=$src sha=$(sha256sum $D/tools/fx_golden.cpp | cut -c1-16)" >> $D/logs/build.log
