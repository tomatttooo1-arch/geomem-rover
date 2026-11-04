# SPDX-License-Identifier: MIT
# vv_common.tcl —— vivado-checks 批处理公共过程（本项目新写，非上游文件）
# 由 vv_run.sh 通过环境变量传参：
#   VV_OUT      输出目录（已由 vv_run.sh 创建并 cd 进去）
#   VV_PART     器件，默认 xck26-sfvc784-2LV-c（KV260）
#   VV_TOP      顶层模块名，可空（空则 find_top 自动猜，多候选时打印并取第一个）
#   VV_SRCS     源文件/目录/文件列表(.f) 的空白分隔列表
#   VV_INCS     include 目录（空白分隔）
#   VV_DEFS     verilog_define（空白分隔，如 FOO=1 BAR）
#   VV_XPR      工程模式：.xpr 路径（给了就忽略 VV_SRCS）
#   VV_THREADS  general.maxThreads，默认 4（共享机器）

proc vv_env {name {default ""}} {
    if {[info exists ::env($name)] && $::env($name) ne ""} { return $::env($name) }
    return $default
}

proc vv_kv {key val} {
    # 统一的机读摘要行：vv_run.sh 只把 VV_SUMMARY 行打到终端
    puts "VV_SUMMARY $key=$val"
    set fh [open [file join [vv_env VV_OUT .] summary.txt] a]
    puts $fh "$key=$val"
    close $fh
}

proc vv_setup {} {
    set_param general.maxThreads [vv_env VV_THREADS 4]
    vv_kv vivado_version [version -short]
}

# 展开源：文件、目录（递归找 .v/.sv/.vhd/.vhdl）、.f 文件列表（每行一个路径，# 注释）
proc vv_expand_srcs {items} {
    set out {}
    foreach it $items {
        if {[file isdirectory $it]} {
            foreach pat {*.v *.sv *.vhd *.vhdl} {
                foreach f [lsort [glob -nocomplain -directory $it -types f $pat]] { lappend out [file normalize $f] }
            }
            foreach sub [lsort [glob -nocomplain -directory $it -types d *]] {
                set out [concat $out [vv_expand_srcs [list $sub]]]
            }
        } elseif {[string match *.f $it]} {
            set fh [open $it r]
            foreach line [split [read $fh] "\n"] {
                set line [string trim $line]
                if {$line eq "" || [string index $line 0] eq "#"} continue
                set out [concat $out [vv_expand_srcs [list $line]]]
            }
            close $fh
        } elseif {[file exists $it]} {
            lappend out [file normalize $it]
        } else {
            puts "VV_WARN source not found: $it"
        }
    }
    return $out
}

# 读入设计；返回 {part top include_dirs}
# 非工程模式下没有 current_fileset，include_dirs / define 通过 synth_design 参数传入
# （上游 tcl-reference 用 set_property include_dirs [current_fileset]，那是工程/MCP 会话写法）。
proc vv_load_design {} {
    set part [vv_env VV_PART xck26-sfvc784-2LV-c]
    set top  [vv_env VV_TOP]
    set xpr  [vv_env VV_XPR]
    set incs {}
    foreach d [vv_env VV_INCS] { lappend incs [file normalize $d] }
    if {$xpr ne ""} {
        open_project $xpr
        set part [get_property part [current_project]]
        if {$top eq ""} { set top [get_property top [current_fileset]] }
        vv_kv mode project
    } else {
        set files [vv_expand_srcs [vv_env VV_SRCS]]
        if {[llength $files] == 0} { error "VV_ERROR no RTL files (VV_SRCS empty or not found)" }
        # 头文件目录自动加入 include_dirs（上游做法：按 .vh/.svh/.h 所在目录收集）
        foreach f $files { set d [file dirname $f]; if {$d ni $incs} { lappend incs $d } }
        foreach f $files {
            switch -- [file extension $f] {
                .v            { read_verilog $f }
                .sv           { read_verilog -sv $f }
                .vhd - .vhdl  { read_vhdl $f }
            }
        }
        vv_kv mode non-project
        vv_kv n_files [llength $files]
    }
    if {$top eq ""} {
        if {[catch {find_top} tops]} { set tops {} }
        set top [lindex $tops 0]
        if {[llength $tops] > 1} {
            puts "VV_WARN multiple top candidates: $tops ; auto-selected $top ; rerun with --top if wrong"
            vv_kv top_candidates [join $tops ,]
        }
        if {$top eq ""} { error "VV_ERROR could not auto-detect top; pass --top" }
    }
    vv_kv part $part
    vv_kv top $top
    return [list $part $top $incs]
}

proc vv_synth_args {part top incs} {
    set a [list -top $top -part $part]
    if {[llength $incs]} { lappend a -include_dirs $incs }
    set defs [vv_env VV_DEFS]
    if {[llength $defs]} { lappend a -verilog_define $defs }
    return $a
}
