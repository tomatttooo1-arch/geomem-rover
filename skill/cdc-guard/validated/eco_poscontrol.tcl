# MERGE10: positive control for CDC0/m6_cdc_check.tcl. Opens the m5 routed.dcp read-only, adds ONE flip-flop of
# ours outside m7_pl_i/dpu clocked by clk_dpu whose D is a clk_pl_0 register of sysinfo (a real clk_pl_0 -> clk_dpu
# path outside the DPU), and writes the result to a TEMPORARY dcp (never over the original). The unmodified guard is
# then run on that copy and must FAIL. usage: -tclargs <routed.dcp> <out.dcp>
set_param general.maxThreads 8
lassign $argv dcp out
open_checkpoint $dcp
set src [lindex [lsort [get_cells -hier -filter {IS_SEQUENTIAL && NAME =~ m7_pl_i/sysinfo/* && REF_NAME =~ FD*}]] 0]
set qn [get_nets -of_objects [get_pins $src/Q]]
set ckn [get_nets -of_objects [get_pins m7_pl_i/dpu/m_axi_dpu_aclk]]
puts "ECO src=$src q_net=$qn clk_net=$ckn clocks_on_clk_net=[get_clocks -of_objects $ckn]"
create_cell -reference FDRE m7_pl_i/merge10_negctl_ff
connect_net -hier -net $qn -objects [get_pins m7_pl_i/merge10_negctl_ff/D]
connect_net -hier -net $ckn -objects [get_pins m7_pl_i/merge10_negctl_ff/C]
set one [lindex [get_nets -hier -filter {TYPE == POWER}] 0]; set zero [lindex [get_nets -hier -filter {TYPE == GROUND}] 0]
connect_net -hier -net $one -objects [get_pins m7_pl_i/merge10_negctl_ff/CE]
connect_net -hier -net $zero -objects [get_pins m7_pl_i/merge10_negctl_ff/R]
set p [get_timing_paths -to [get_pins m7_pl_i/merge10_negctl_ff/D] -max_paths 1]
puts "ECO path: [get_property STARTPOINT_PIN $p] -> [get_property ENDPOINT_PIN $p] clocks [get_property STARTPOINT_CLOCK $p] -> [get_property ENDPOINT_CLOCK $p]"
write_checkpoint -force $out
puts "ECO DONE"
