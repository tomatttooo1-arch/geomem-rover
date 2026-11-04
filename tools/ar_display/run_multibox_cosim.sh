#!/usr/bin/env bash
# Run one variant of the multi-box co-simulation gate (D, 2026-10-11).
#   bash tools/ar_display/run_multibox_cosim.sh <scratch> <variant>
# <scratch> is the --scratch given to multibox_cosim.py; <variant> is clean or
# a mutant name. Log: <scratch>/cosim_<variant>.log, last line
#   AR_MB_VERDICT <variant> csim=<PASS|FAIL> cosim=<PASS|FAIL|NONE>
# Verdict comes from the tool's own "C/RTL co-simulation finished" line, not
# only from the Tcl marker.
set -u
S=$(readlink -f "$1"); V=$2
HERE=$(cd "$(dirname "$0")" && pwd)
LOG=$S/cosim_$V.log
mkdir -p "$S/tmp"
(
  source /data1/Xilinx/Vitis_HLS/2022.2/settings64.sh >/dev/null 2>&1
  cd "$S" && TMPDIR=$S/tmp AR_MB_CORE=$S/core_$V AR_MB_DATA=$S/data/ar_mb_cases.bin \
    AR_MB_BUILD=$S/hls_$V vitis_hls -f "$HERE/run_multibox_cosim.tcl"
) > "$LOG" 2>&1
rc=$?
csim=FAIL; grep -q '^AR_MB_CSIM_PASS' "$LOG" && csim=PASS
cosim=NONE
grep -q 'C/RTL co-simulation finished: PASS' "$LOG" && cosim=PASS
grep -q 'C/RTL co-simulation finished: FAIL' "$LOG" && cosim=FAIL
if [ "$cosim" = NONE ] && grep -q '^AR_MB_COSIM_FAIL' "$LOG"; then cosim=FAIL; fi
echo "AR_MB_VERDICT $V csim=$csim cosim=$cosim rc=$rc" | tee -a "$LOG"
