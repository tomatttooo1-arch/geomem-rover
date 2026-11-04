# cdc_guard.tcl — 构建后跨时钟域护栏（只读打开 routed.dcp，不写回）
# 用法: vivado -mode batch -nojournal -source cdc_guard.tcl -tclargs <routed.dcp> <out_dir> [clkA] [clkB_glob] [inside_glob]
#   默认值就是我们验证过的那一版（validated/m6_cdc_check.tcl）：
#     clkA        = clk_pl_0           （PS 给的寄存器口时钟）
#     clkB_glob   = clk_dpu_*          （IP 的计算时钟，按名字通配）
#     inside_glob = m7_pl_i/dpu/*      （允许出现跨域路径的 IP 层级）
# 判据: clkA <-> clkB 两个方向的每条时序路径，起点和终点都必须落在 inside_glob 里；否则 FAIL 并逐条列出。
# 为什么需要: 某些 IP（我们遇到的是 AMD DPU 的 S_AXI 独立时钟模式）自带"时钟对"级 set_false_path，
#   它会作用到全设计这对时钟之间的所有路径（方法学 TIMING-54）。之后你自己加的任何跨这对时钟的逻辑都会被静默屏蔽，
#   时序报告和 CDC 报告都不会提醒你。这个脚本把"只有 IP 内部才允许跨"变成一个会失败的检查。
set_param general.maxThreads 8
lassign $argv dcp o clkA clkB inside
if {$clkA eq ""} {set clkA clk_pl_0}
if {$clkB eq ""} {set clkB {clk_dpu_*}}
if {$inside eq ""} {set inside {m7_pl_i/dpu/*}}
open_checkpoint $dcp
file mkdir $o
report_cdc -details -file $o/cdc.rpt
report_methodology -file $o/methodology.rpt
set cp [get_clocks $clkA]
set cd [get_clocks -filter "NAME =~ $clkB"]
if {[llength $cp] != 1 || [llength $cd] != 1} {
  set f [open $o/cdc_guard.txt w]; puts $f "CDC_GUARD ERROR clocks not unique: clkA=$cp clkB=$cd"; close $f
  error "clock lookup not unique"
}
set f [open $o/cdc_guard.txt w]
set bad 0
foreach {a b} [list $cp $cd $cd $cp] {
  set p [get_timing_paths -from $a -to $b -max_paths 5000 -nworst 1 -unique_pins]
  puts $f "PAIR $a -> $b paths=[llength $p]"
  if {[llength $p] >= 5000} { puts $f "  WARN path count hit -max_paths 5000; raise it" }
  foreach x $p {
    set s [get_property STARTPOINT_PIN $x]; set e [get_property ENDPOINT_PIN $x]
    if {![string match $inside $e] || ![string match $inside $s]} {
      puts $f "  OUTSIDE from=$s to=$e"; incr bad
    }
  }
}
puts $f [expr {$bad==0 ? "CDC_GUARD PASS (all $clkA<->$clkB paths inside $inside)" : "CDC_GUARD FAIL outside=$bad"}]
close $f
