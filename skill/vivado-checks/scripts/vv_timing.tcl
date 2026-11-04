# SPDX-License-Identifier: MIT
# vv_timing.tcl —— 时序摘要 + 时序方法学检查（批处理）。
# 方法学部分移植自上游 vivado-timing-methodology-checks SKILL.md Step 1–3
# （Copyright (C) 2026, Advanced Micro Devices, Inc., MIT）。改动：2022.2 的 report_methodology 没有 -json，
# 改为 -file 文本报告 + 上游 scripts/write_json_report.tcl 生成 JSON；新增 report_timing_summary 与 WNS/TNS 摘要。
# 只读打开检查点：所有输出写 VV_OUT，绝不 write_checkpoint 回原目录。
source [file join [file dirname [info script]] vv_common.tcl]
vv_setup
set out [vv_env VV_OUT .]
set dcp [vv_env VV_DCP]
set xpr [vv_env VV_XPR]
if {$dcp ne ""} {
    open_checkpoint $dcp
    vv_kv design $dcp
} elseif {$xpr ne ""} {
    open_project -read_only $xpr
    set run [vv_env VV_RUN impl_1]
    open_run $run
    vv_kv design "$xpr:$run"
} else { error "VV_ERROR need --dcp or --xpr" }

# 1) 时序摘要（与常规构建脚本同一命令，便于与已有 timing_route.rpt 对账）
report_timing_summary -max_paths 20 -report_unconstrained -file [file join $out timing_summary.rpt]
# Tcl 侧独立读数（交叉核对 rpt 解析）
foreach {k d} {setup -setup hold -hold} {
    set p [get_timing_paths -max_paths 1 -nworst 1 $d]
    if {[llength $p]} { vv_kv tcl_worst_${k}_slack [get_property SLACK $p] } else { vv_kv tcl_worst_${k}_slack none }
}
vv_kv n_clocks [llength [get_clocks]]

# 2) 方法学检查
if {[vv_env VV_SKIP_METHODOLOGY] eq ""} {
    report_methodology -file [file join $out methodology.rpt]
    set vio [get_methodology_violations]
    vv_kv methodology_total [llength $vio]
    set counts [dict create]
    set rprop CHECK
    if {[llength $vio] && "CHECK" ni [list_property [lindex $vio 0]]} { set rprop RULE }
    vv_kv methodology_rule_property $rprop
    foreach v $vio {
        set key "[get_property $rprop $v]|[get_property SEVERITY $v]"
        dict incr counts $key
    }
    set fh [open [file join $out methodology_counts.csv] w]
    puts $fh "rule,severity,count"
    foreach key [lsort [dict keys $counts]] {
        lassign [split $key |] r s
        puts $fh "$r,$s,[dict get $counts $key]"
    }
    close $fh
    source [file join [file dirname [info script]] .. reference vivado-timing-methodology-checks scripts write_json_report.tcl]
    write_json_report -report methodology -file [file join $out methodology.json]
    write_xdc -force -exclude_physical [file join $out constraints.xdc]
}
