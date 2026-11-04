# SPDX-License-Identifier: MIT
# vv_lint.tcl —— RTL lint 批处理版。移植自上游 vivado-rtl-lint/tcl-reference.md Step 1–3
# （Copyright (C) 2026, Advanced Micro Devices, Inc., MIT）。改动：MCP 交互 → 批处理脚本；
# 默认器件 xc7k70t → KV260；include_dirs 改走 synth_design 参数；2022.2 只产 .rpt（无 CSV 开关）。
source [file join [file dirname [info script]] vv_common.tcl]
vv_setup
lassign [vv_load_design] part top incs
set rpt [file join [vv_env VV_OUT .] linter.rpt]
set args [vv_synth_args $part $top $incs]
set rc [catch {synth_design {*}$args -lint -file $rpt} msg]
vv_kv lint_rc $rc
if {$rc} { puts "VV_ERROR synth_design -lint failed: $msg" }
if {![file exists $rpt] || [file size $rpt] == 0} {
    vv_kv lint_report missing
} else {
    vv_kv lint_report $rpt
}
