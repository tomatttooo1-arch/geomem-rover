#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p .memory/bin .memory/vectors .memory/logs
GOLD=.memory/ref/object_evidence_ref.cpp
g++ -std=c++14 -O3 -ffp-contract=off -fno-fast-math -Wno-unknown-pragmas -Dfpga_m7_object_roi_batch=ref_roi_batch -c "$GOLD" -o .memory/bin/ref.o
g++ -std=c++14 -O3 -ffp-contract=off -fno-fast-math cpu_equiv.cpp roi_fx_ps.cpp .memory/bin/ref.o -o .memory/bin/cpu_equiv
g++ -std=c++14 -O3 -ffp-contract=off -fno-fast-math make_vectors.cpp roi_fx_ps.cpp .memory/bin/ref.o -o .memory/bin/make_vectors
for seed in 7 1 11; do .memory/bin/cpu_equiv "$seed" | tee ".memory/logs/cpu-seed$seed.log"; done
sha256sum ${FPGA_SRC}/runs/ar-display-20261011-scratch/rec30_prep/depth_u16.bin | tee .memory/logs/rec30-depth-sha256.txt
.memory/bin/make_vectors ${FPGA_SRC}/runs/ar-display-20261011-scratch/rec30_prep/depth_u16.bin .memory/vectors | tee .memory/logs/rec30.log
