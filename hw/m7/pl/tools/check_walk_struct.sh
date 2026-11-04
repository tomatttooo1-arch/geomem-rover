#!/bin/bash
# GEO_FX3 (2026-11-04) build check for the fixed-point frame core (m7_frame_stream.cpp).
# usage: check_walk_struct.sh VERILOG_DIR   (HLS solution/syn/verilog or solution/impl/verilog)
# The X1 saving (~3.1k LUT over 8 walk lanes, GEO_FX1/README.md section 3.3) depends on Vitis HLS
# 2022.2 building walk_loop as "state machine + pipeline": the ap_wait() before the loop keeps HLS
# from pipelining the loop in place with flow_control_loop_delay_pipe (each carried state bit then
# gets a first-iteration select, ~+400 LUT per lane, and the worst path starts at ap_loop_init).
# Any generated module named / instantiating flow_control_loop_delay_pipe means the structure came
# back (other tool version, code change): exit 3 and list the files. Exit 0 = clean, 2 = bad dir.
# Calibrated on GEO_FX1 RTL: s1b / W1 (no ap_wait) -> alarm (9 files each); X1 -> clean.
d=${1:?usage: check_walk_struct.sh VERILOG_DIR}
[ -d "$d" ] && ls "$d"/*.v >/dev/null 2>&1 || { echo "WALK_STRUCT_CHECK bad dir $d"; exit 2; }
hits=$(grep -l flow_control_loop_delay_pipe "$d"/*.v 2>/dev/null | xargs -r -n1 basename | sort)
if [ -n "$hits" ]; then
  echo "WALK_STRUCT_ALARM flow_control_loop_delay_pipe in $(echo "$hits" | wc -l) file(s) under $d:"
  echo "$hits" | sed 's/^/  /'
  exit 3
fi
echo "WALK_STRUCT_OK no flow_control_loop_delay_pipe in $(ls "$d"/*.v | wc -l) verilog files ($d)"
exit 0
