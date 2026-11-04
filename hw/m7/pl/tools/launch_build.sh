#!/usr/bin/env bash
# Launch one detached Vivado non-project build.
# usage: launch_build.sh <out_dir> <ip_repo> <clk_mhz> [threads]
# Optional env: FPGA_M7_PL_FROM_SYNTH=<earlier build dir> resumes from its
# post_synth.dcp (no .xsa then: the block design is not in memory).
# Exit code 3 from the Tcl script = post-place early stop
# (FPGA_M7_PL_POSTPLACE_WNS_MIN): logged as POSTPLACE_STOP, not as a failure.
# The source commit becomes BUILD_ID (sysinfo register 0x08); the build refuses
# to start from a dirty tree so BUILD_ID always names the exact sources.
set -u
OUT=${1:?out dir}; IPREPO=${2:?ip repo}; MHZ=${3:?clk MHz}; THREADS=${4:-8}
HERE=$(cd "$(dirname "$0")/.." && pwd)
REPO=$(git -C "$HERE" rev-parse --show-toplevel)
if [ -n "$(git -C "$REPO" status --porcelain -- hw/m7)" ]; then
  echo "refusing: uncommitted changes under hw/m7" >&2; exit 2
fi
SHORT=$(git -C "$REPO" rev-parse --short=7 HEAD)
LEDGER=$(dirname "$OUT")/LEDGER.txt
mkdir -p "$(dirname "$OUT")"
echo "$(date -Is) START build out=$OUT clk=${MHZ}MHz clk1=${FPGA_M7_PL_CLK1_MHZ:-off} ip_repo=$IPREPO commit=$(git -C "$REPO" rev-parse HEAD) threads=$THREADS${FPGA_M7_PL_INCR_DCP:+ incr_dcp=$FPGA_M7_PL_INCR_DCP}${FPGA_M7_PL_INCR_DIRECTIVE:+ incr_directive=$FPGA_M7_PL_INCR_DIRECTIVE}${FPGA_M7_PL_POSTPLACE_WNS_MIN:+ postplace_wns_min=$FPGA_M7_PL_POSTPLACE_WNS_MIN}${FPGA_M7_PL_NO_AR:+ no_ar=$FPGA_M7_PL_NO_AR}${FPGA_M7_PL_MEM_DIRECT:+ mem_direct=$FPGA_M7_PL_MEM_DIRECT}${FPGA_M7_PL_IMPL_STRATEGY:+ impl_strategy=$FPGA_M7_PL_IMPL_STRATEGY}${FPGA_M7_PL_DPU_ARCH:+ dpu_arch=$FPGA_M7_PL_DPU_ARCH}${FPGA_M7_PL_DPU_CLK_GATE:+ dpu_clk_gate=$FPGA_M7_PL_DPU_CLK_GATE}${FPGA_M7_PL_DETPOST:+ detpost=$FPGA_M7_PL_DETPOST}${FPGA_M7_PL_GEO_FX:+ geo_fx=$FPGA_M7_PL_GEO_FX}${FPGA_M7_PL_LPD:+ lpd=$FPGA_M7_PL_LPD}${FPGA_M7_PL_LPD_IC:+ lpd_ic=$FPGA_M7_PL_LPD_IC}${FPGA_M7_PL_PREP:+ prep=$FPGA_M7_PL_PREP prep_ip=${FPGA_M7_PL_PREP_IP:-}}${FPGA_M7_PL_PLANNER:+ planner=$FPGA_M7_PL_PLANNER planner_ip=${FPGA_M7_PL_PLANNER_IP:-}}${FPGA_M7_PL_SRCH:+ srch=$FPGA_M7_PL_SRCH srch_ip=${FPGA_M7_PL_SRCH_IP:-}}${FPGA_M7_PL_MEMK:+ memk=$FPGA_M7_PL_MEMK memk_ip=${FPGA_M7_PL_MEMK_IP:-}}${FPGA_M7_PL_OBS:+ obs=$FPGA_M7_PL_OBS obs_port=${FPGA_M7_PL_OBS_PORT:-hp1} obs_ip=${FPGA_M7_PL_OBS_IP:-}}${FPGA_M7_PL_INCR_WITH_C1:+ incr_with_c1=$FPGA_M7_PL_INCR_WITH_C1}${FPGA_M7_PL_LPD_FIFO_NEW:+ lpd_fifo_new=$FPGA_M7_PL_LPD_FIFO_NEW}" >> "$LEDGER"
setsid nohup bash -c '
  source /data1/Xilinx/Vivado/2022.2/settings64.sh >/dev/null 2>&1
  mkdir -p "$0.run" && cd "$0.run" || exit 97
  # Vivado 2022.2 segfault at the end of route_design on this host (Ubuntu 20.04,
  # libudev1 245.4): the license library dlopen()s libudev with its own
  # dependencies searched first, so libudev calls glibc malloc/realloc/
  # malloc_usable_size while glibc-internal allocations (strdup etc.) went to
  # Vivado tcmalloc -> malloc_usable_size on a tcmalloc block -> SIGSEGV.
  # Preloading the system libudev loads it in the global scope at startup, so
  # all of its allocator calls bind to tcmalloc like the rest of the process;
  # the later dlopen() just returns that already-loaded copy.
  # Details: docs/design/M7_PL_BITSTREAM_20260927.md.
  UDEV=${FPGA_M7_PL_UDEV_PRELOAD:-/lib/x86_64-linux-gnu/libudev.so.1}
  [ -e "$UDEV" ] || { echo "missing $UDEV" > "$0.log"; exit 96; }
  export LD_PRELOAD="$UDEV${LD_PRELOAD:+:$LD_PRELOAD}"
  FPGA_M7_PL_BUILD="$0" FPGA_M7_PL_IPREPO="$1" FPGA_M7_PL_CLK_MHZ="$2" \
  FPGA_M7_PL_BUILD_ID="0x$3" FPGA_M7_PL_BUILD_DATE="0x$(date +%Y%m%d)" FPGA_M7_PL_THREADS="$4" \
    vivado -mode batch -nojournal -source "$5/bd/build_bitstream.tcl" > "$0.log" 2>&1
  rc=$?
  if [ $rc = 0 ]; then echo DONE >> "$0.log"
  elif [ $rc = 3 ]; then echo "POSTPLACE_STOP exit=3 (post-place WNS below FPGA_M7_PL_POSTPLACE_WNS_MIN; see $0/POSTPLACE_STOP.txt)" >> "$0.log"
  else echo "FAILED exit=$rc" >> "$0.log"; fi
  if [ $rc = 3 ]; then
    echo "$(date -Is) EXIT build out=$0 rc=3 POSTPLACE_STOP $(tr "\n" " " < "$0/POSTPLACE_STOP.txt" 2>/dev/null)" >> "$6"
  else
    echo "$(date -Is) EXIT build out=$0 rc=$rc" >> "$6"
  fi
' "$OUT" "$IPREPO" "$MHZ" "$SHORT" "$THREADS" "$HERE" "$LEDGER" > /dev/null 2>&1 < /dev/null &
echo "launched build $OUT (log $OUT.log)"
