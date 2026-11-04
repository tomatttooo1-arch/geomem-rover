#!/usr/bin/env bash
# Launch the HLS shell exports in parallel, detached from the caller.
# usage: launch_hls.sh <run_dir> <clock_ns> <cosim 0|1> [top ...]
# Each job writes <run_dir>/<top>.log and appends start/exit lines, with
# `date` timestamps and exit codes, to <run_dir>/LEDGER.txt.
set -u
RUN=${1:?run dir}; CLK=${2:?clock ns}; COSIM=${3:?cosim 0|1}; shift 3
TOPS=("$@")
# Default = the cores in the merged block design (2026-10-11): the AR overlay
# replaced fpga_m7_display_axi; the E2 region check is new; v3a (2026-10-26)
# dropped the legacy fpga_m7_object_axi. The frame top is the
# A1 streaming implementation only with FPGA_M7_PL_FRAME_IMPL=stream exported
# into the environment (the merged build used the A1 IP built that way).
[ ${#TOPS[@]} -eq 0 ] && TOPS=(fpga_m7_frame_axi fpga_m7_object_roi_axi fpga_m7_ar_overlay_axi)
HERE=$(cd "$(dirname "$0")/.." && pwd)
PART=${FPGA_M7_PL_PART:-xck26-sfvc784-2LV-c}
mkdir -p "$RUN/ip_repo"
for top in "${TOPS[@]}"; do
  log="$RUN/$top.log"
  echo "$(date -Is) START $top clock=${CLK}ns cosim=$COSIM part=$PART log=$log" >> "$RUN/LEDGER.txt"
  setsid nohup bash -c '
    source /data1/Xilinx/Vitis_HLS/2022.2/settings64.sh >/dev/null 2>&1
    mkdir -p "$0/$1" && cd "$0/$1" || exit 97
    # export_design -flow syn starts Vivado OOC synthesis: same libudev /
    # tcmalloc SIGSEGV as launch_build.sh (2026-10-26 probe crashed there) ->
    # preload the system libudev the same way.
    UDEV=${FPGA_M7_PL_UDEV_PRELOAD:-/lib/x86_64-linux-gnu/libudev.so.1}
    [ -e "$UDEV" ] && export LD_PRELOAD="$UDEV${LD_PRELOAD:+:$LD_PRELOAD}"
    FPGA_M7_PL_TOP="$1" FPGA_M7_PL_PART="$2" FPGA_M7_PL_CLOCK_NS="$3" \
    FPGA_M7_PL_BUILD="$0/$1/hls" FPGA_M7_PL_IPREPO="$0/ip_repo" FPGA_M7_PL_COSIM="$4" \
      vitis_hls -f "$5/hls/run_export_hls.tcl" > "$0/$1.log" 2>&1
    rc=$?
    echo "$(date -Is) EXIT $1 rc=$rc" >> "$0/LEDGER.txt"
  ' "$RUN" "$top" "$PART" "$CLK" "$COSIM" "$HERE" > /dev/null 2>&1 < /dev/null &
done
echo "launched ${TOPS[*]} in $RUN"
