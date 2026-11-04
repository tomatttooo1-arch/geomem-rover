# poscontrol_eco.tcl — cdc_guard.tcl 的阳性对照：在一份 routed.dcp 的副本里人为造一条越界的跨域路径
# 用法: vivado -mode batch -nojournal -source poscontrol_eco.tcl -tclargs <routed.dcp> <out_tmp.dcp> <src_cell_glob> <dst_clk_pin>
#   src_cell_glob : clkA 域里某个我方模块的寄存器（例：m7_pl_i/sysinfo/*）
#   dst_clk_pin   : 提供 clkB 的一个引脚（例：m7_pl_i/dpu/m_axi_dpu_aclk）
# 做法: 新建一个 FDRE（在 IP 层级之外），时钟接 clkB 网络，D 接 clkA 域那个寄存器的 Q。写到临时 dcp（绝不覆盖原文件）。
# 然后对临时 dcp 原样跑 cdc_guard.tcl，必须 FAIL 且报出这一条；跑完删掉临时 dcp。
set_param general.maxThreads 8
lassign $argv dcp out srcglob ckpin
open_checkpoint $dcp
set src [lindex [lsort [get_cells -hier -filter "IS_SEQUENTIAL && NAME =~ $srcglob && REF_NAME =~ FD*"]] 0]
set qn [get_nets -of_objects [get_pins $src/Q]]
set ckn [get_nets -of_objects [get_pins $ckpin]]
puts "ECO src=$src q_net=$qn clk_net=$ckn clocks_on_clk_net=[get_clocks -of_objects $ckn]"
set top [lindex [split $srcglob /] 0]
create_cell -reference FDRE $top/cdcguard_poscontrol_ff
connect_net -hier -net $qn -objects [get_pins $top/cdcguard_poscontrol_ff/D]
connect_net -hier -net $ckn -objects [get_pins $top/cdcguard_poscontrol_ff/C]
set one [lindex [get_nets -hier -filter {TYPE == POWER}] 0]; set zero [lindex [get_nets -hier -filter {TYPE == GROUND}] 0]
connect_net -hier -net $one -objects [get_pins $top/cdcguard_poscontrol_ff/CE]
connect_net -hier -net $zero -objects [get_pins $top/cdcguard_poscontrol_ff/R]
set p [get_timing_paths -to [get_pins $top/cdcguard_poscontrol_ff/D] -max_paths 1]
puts "ECO path: [get_property STARTPOINT_PIN $p] -> [get_property ENDPOINT_PIN $p] clocks [get_property STARTPOINT_CLOCK $p] -> [get_property ENDPOINT_CLOCK $p]"
write_checkpoint -force $out
puts "ECO DONE"
