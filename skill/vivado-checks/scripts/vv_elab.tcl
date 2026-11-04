# SPDX-License-Identifier: MIT
# vv_elab.tcl —— RTL elaboration 批处理版。移植自上游 vivado-rtl-elaboration-analysis/tcl-reference.md
# Step 1–3（Copyright (C) 2026, Advanced Micro Devices, Inc., MIT）。改动：MCP → 批处理；默认器件 → KV260；
# 日志由 vv_run.sh 用 -log 固定到输出目录的 vivado.log。
source [file join [file dirname [info script]] vv_common.tcl]
vv_setup
lassign [vv_load_design] part top incs
set args [vv_synth_args $part $top $incs]
if {[vv_env VV_SKIP_IP] ne ""} { lappend args -rtl_skip_ip }
set rc [catch {synth_design {*}$args -rtl -name rtl_1} msg]
vv_kv elab_rc $rc
if {$rc} { puts "VV_ERROR synth_design -rtl failed: $msg" }
