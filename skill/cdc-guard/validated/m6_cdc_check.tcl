# CDC0 (2026-11-04): m6 整板构建后的跨时钟域护栏检查（只读，不写回 dcp）。
# 用法: vivado -mode batch -nojournal -source m6_cdc_check.tcl -tclargs <routed.dcp> <out_dir>
# 目的: DPU IP 自带的 S_AXI_CLK_INDEPENDENT 约束 (m7_pl_dpu_0.xdc L39-40) 是"时钟对"级 false path，
#       会覆盖 clk_pl_0 <-> clk_dpu 之间【全设计】的路径（TIMING-54 的含义）。m5 实测被它覆盖的
#       236 个终点全部在 m7_pl_i/dpu 内。本脚本在 m6 上复查：起点或终点只要有一个不在 DPU 内就 FAIL。
set_param general.maxThreads 8
lassign $argv dcp o
open_checkpoint $dcp
file mkdir $o
report_cdc -details -file $o/cdc.rpt
report_methodology -file $o/methodology.rpt
set cp [get_clocks clk_pl_0]
set cd [get_clocks -filter {NAME =~ clk_dpu_*}]
set f [open $o/cdc_guard.txt w]
set bad 0
foreach {a b} [list $cp $cd $cd $cp] {
  set p [get_timing_paths -from $a -to $b -max_paths 5000 -nworst 1 -unique_pins]
  puts $f "PAIR $a -> $b paths=[llength $p]"
  foreach x $p {
    set s [get_property STARTPOINT_PIN $x]; set e [get_property ENDPOINT_PIN $x]
    if {![string match m7_pl_i/dpu/* $e] || ![string match m7_pl_i/dpu/* $s]} {
      puts $f "  OUTSIDE_DPU from=$s to=$e"; incr bad
    }
  }
}
puts $f [expr {$bad==0 ? "CDC_GUARD PASS (all clk_pl_0<->clk_dpu paths inside m7_pl_i/dpu)" : "CDC_GUARD FAIL outside=$bad"}]
close $f
