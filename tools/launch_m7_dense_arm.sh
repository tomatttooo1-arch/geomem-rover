#!/bin/bash
# Launch one dense-probe arm fully detached under tools/m7_supervise.py.
# usage: launch_m7_dense_arm.sh <ARM> <RUN_DIR>
set -eu
ARM=$1
RUN=$2
ROOT=$(cd "$(dirname "$0")/.." && pwd)
if [ -e "$RUN" ]; then echo "run dir exists: $RUN" >&2; exit 2; fi
export FPGA_M7_DENSE_PART=xck26-sfvc784-2LV-c
export FPGA_M7_DENSE_CLOCK_NS=10
export FPGA_M7_DENSE_ARM="$ARM"
export FPGA_M7_DENSE_BUILD="$RUN/hls_k26_10ns"
mkdir -p "$(dirname "$RUN")"
setsid nohup /usr/bin/python3 "$ROOT/tools/m7_supervise.py" \
    --run-dir "$RUN" --heartbeat-seconds 60 \
    --progress-glob "$RUN/hls_k26_10ns/solution/sim/verilog/xsim.log" \
    -- bash -c "source /data1/Xilinx/Vitis_HLS/2022.2/settings64.sh && exec vitis_hls -f '$ROOT/hw/m7/run_frame_dense_arm_hls.tcl'" \
    > "$RUN.supervisor.out" 2>&1 < /dev/null &
echo "launched arm=$ARM supervisor_pid=$! run=$RUN at $(date -Is)"
