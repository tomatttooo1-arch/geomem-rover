#!/bin/bash
# Launch one object bank-probe arm fully detached under tools/m7_supervise.py.
# usage: launch_m7_object_bank_arm.sh <A|B|C|D> <RUN_DIR>
set -eu
ARM=$1
RUN=$2
ROOT=$(cd "$(dirname "$0")/.." && pwd)
if [ -e "$RUN" ]; then echo "run dir exists: $RUN" >&2; exit 2; fi
export FPGA_M7_OBJECT_BANK_ARM="$ARM"
export FPGA_M7_OBJECT_BANK_PART=xck26-sfvc784-2LV-c
export FPGA_M7_OBJECT_BANK_CLOCK_NS=10
export FPGA_M7_OBJECT_BANK_SOURCE_COMMIT=$(git -C "$ROOT" rev-parse HEAD)
export FPGA_M7_OBJECT_BANK_BUILD="$RUN/hls_k26_10ns"
mkdir -p "$(dirname "$RUN")"
setsid nohup /usr/bin/python3 "$ROOT/tools/m7_supervise.py" \
    --run-dir "$RUN" --heartbeat-seconds 60 \
    --progress-glob "$RUN/hls_k26_10ns/solution/sim/verilog/xsim.log" \
    -- bash -c "source /data1/Xilinx/Vitis_HLS/2022.2/settings64.sh && exec vitis_hls -f '$ROOT/hw/m7/run_object_bank_probe.tcl'" \
    > "$RUN.supervisor.out" 2>&1 < /dev/null &
echo "launched object arm=$ARM supervisor_pid=$! run=$RUN at $(date -Is)"
