#!/bin/bash
# Whole-board build with the extension cores (memory, search, observation/verification, final planner)
# on top of the m5 configuration. Run from the repository root.
# usage: scripts/build/launch_whole_m7.sh <tag> [planner_ip] [srch_ip] [memk_ip] [obs_ip]
# Each *_ip = an HLS export.zip (or IP directory). Empty string or "-" = keep the m5 block:
#   planner -> FPGA_PLANNER_IP (m5 planner); srch / memk -> not built; obs -> not built, roi kept.
# With no core given this is exactly the m5 command set (scripts/build/launch_whole_m5.sh).
# Required environment: FPGA_IP_REPO, FPGA_DPU_IPREPO, FPGA_PREP_IP, FPGA_PLANNER_IP (see launch_whole_m5.sh).
# Optional environment:
#   FPGA_BUILD_OUT (default ./builds)
#   LPD_IC=samd|sasd (samd)   OBS_PORT=hp1|lpd (hp1: obs core on HP1 at its own m_axi width)
#   PLANNER_SHA256 / SRCH_SHA256 / MEMK_SHA256 / OBS_SHA256 (optional sha256 prefixes of the given IPs)
#   LPD_FIFO_NEW=0|1|2  data FIFO depth of the new ic_lpd SIs (S04+); unset = tool default; 1 = 32 deep, no BRAM
#   INCR=1 + INCR_DCP=<routed.dcp of an earlier build>  incremental implementation
#           (FPGA_M7_PL_INCR_WITH_C1=1, FPGA_M7_PL_INCR_DIRECTIVE=${INCR_DIRECTIVE:-TimingClosure})
#   MEMK_IF=lite|bram  memory-core shell; unset = taken from the memk IP (BRAM master port -> bram)
#   DRY=1  print the FPGA_M7_PL_* environment and the build dir, launch nothing
# Output: $FPGA_BUILD_OUT/impl-<m6|m6a>[-mk4][-incr]-<tag>/. Vivado 2022.2.
set -u
TAG=${1:?tag}
: "${FPGA_IP_REPO:?set FPGA_IP_REPO}" "${FPGA_DPU_IPREPO:?set FPGA_DPU_IPREPO}" \
  "${FPGA_PREP_IP:?set FPGA_PREP_IP}" "${FPGA_PLANNER_IP:?set FPGA_PLANNER_IP}"
OUT=${FPGA_BUILD_OUT:-$PWD/builds}
arg() { local v=${1:-}; [ "$v" = "-" ] && v=""; echo "$v"; }
PLANNER_IP=$(arg "${2:-}"); SRCH_IP=$(arg "${3:-}"); MEMK_IP=$(arg "${4:-}"); OBS_IP=$(arg "${5:-}")
for f in "$PLANNER_IP" "$SRCH_IP" "$MEMK_IP" "$OBS_IP"; do [ -z "$f" ] || [ -e "$f" ] || { echo "not found: $f" >&2; exit 2; }; done
avail=$(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo); [ "$avail" -ge 12 ] || { echo "MemAvailable ${avail} GB < 12" >&2; exit 2; }
export FPGA_M7_PL_NO_AR=1 FPGA_M7_PL_MEM_DIRECT=1 FPGA_M7_PL_IMPL_STRATEGY=c1 FPGA_M7_PL_CLK1_MHZ=166.67 \
  FPGA_M7_PL_POSTPLACE_WNS_MIN=-0.3 FPGA_M7_PL_DPU_ARCH=1024 FPGA_M7_PL_DPU_URAM=26 FPGA_M7_PL_DPU_MHZ=300 \
  FPGA_M7_PL_DPU_IPREPO=$FPGA_DPU_IPREPO \
  FPGA_M7_PL_DPU_CLK_GATE=1 FPGA_M7_PL_DETPOST=1 \
  FPGA_M7_PL_GEO_FX=1 FPGA_M7_PL_LPD=1 FPGA_M7_PL_LPD_IC=${LPD_IC:-samd} \
  FPGA_M7_PL_PREP=1 FPGA_M7_PL_PREP_IP=$FPGA_PREP_IP \
  FPGA_M7_PL_PLANNER=1
if [ -z "$PLANNER_IP" ]; then
  export FPGA_M7_PL_PLANNER_IP=$FPGA_PLANNER_IP FPGA_M7_PL_PLANNER_IP_SHA256=${FPGA_PLANNER_IP_SHA256:-}
else
  export FPGA_M7_PL_PLANNER_IP=$PLANNER_IP FPGA_M7_PL_PLANNER_IP_SHA256=${PLANNER_SHA256:-}
fi
NAME=m6
[ -n "${LPD_FIFO_NEW:-}" ] && export FPGA_M7_PL_LPD_FIFO_NEW=$LPD_FIFO_NEW
if [ -n "$SRCH_IP" ]; then export FPGA_M7_PL_SRCH=1 FPGA_M7_PL_SRCH_IP=$SRCH_IP FPGA_M7_PL_SRCH_IP_SHA256=${SRCH_SHA256:-}; fi
if [ -n "$MEMK_IP" ]; then
  export FPGA_M7_PL_MEMK=1 FPGA_M7_PL_MEMK_IP=$MEMK_IP FPGA_M7_PL_MEMK_IP_SHA256=${MEMK_SHA256:-}
  MIF=${MEMK_IF:-}
  if [ -z "$MIF" ]; then
    MIF=$(python3 -c "
import sys, zipfile, os, re
p = sys.argv[1]
x = zipfile.ZipFile(p).read('component.xml').decode() if os.path.isfile(p) else open(os.path.join(p, 'component.xml')).read()
print('bram' if re.search(r'spirit:name=\"bram\"', x) else 'lite')" "$MEMK_IP") || exit 2
  fi
  export FPGA_M7_PL_MEMK_IF=$MIF
fi
if [ -n "$OBS_IP" ]; then
  export FPGA_M7_PL_OBS=1 FPGA_M7_PL_OBS_IP=$OBS_IP FPGA_M7_PL_OBS_PORT=${OBS_PORT:-hp1} FPGA_M7_PL_OBS_IP_SHA256=${OBS_SHA256:-}
else NAME=m6a; fi
[ "${FPGA_M7_PL_MEMK_IF:-lite}" = bram ] && NAME=$NAME-mk4
[ -z "$PLANNER_IP$SRCH_IP$MEMK_IP$OBS_IP" ] && NAME=m5eq   # nothing new: the m5 command set
if [ "${INCR:-0}" = 1 ]; then
  D=${INCR_DCP:?set INCR_DCP}; [ -f "$D" ] || { echo "INCR_DCP not found: $D" >&2; exit 2; }
  export FPGA_M7_PL_INCR_DCP=$D FPGA_M7_PL_INCR_WITH_C1=1 FPGA_M7_PL_INCR_DIRECTIVE=${INCR_DIRECTIVE:-TimingClosure}
  NAME=$NAME-incr
fi
if [ "${DRY:-0}" = 1 ]; then env | grep '^FPGA_M7_PL_' | sort; echo "build dir: $OUT/impl-$NAME-$TAG"; exit 0; fi
mkdir -p "$OUT"
bash hw/m7/pl/tools/launch_build.sh "$OUT/impl-$NAME-$TAG" "$FPGA_IP_REPO" 100 8
