# OOC synthesis of the DPU alone (resources + post-synth timing estimate).
# env: FPGA_DPU_OUT (new dir), FPGA_DPU_IPREPO, FPGA_DPU_ARCH, FPGA_DPU_URAM, FPGA_DPU_CLK_MHZ, FPGA_DPU_THREADS
set out [file normalize $::env(FPGA_DPU_OUT)]
if {[file exists $out]} { error "exists: $out" }
file mkdir $out
set here [file dirname [file normalize [info script]]]
source $here/dpu_config.tcl
set_param general.maxThreads $::env(FPGA_DPU_THREADS)
config_webtalk -user off
set part xck26-sfvc784-2LV-c
create_project -in_memory -part $part
set_property ip_repo_paths [file normalize $::env(FPGA_DPU_IPREPO)] [current_project]
update_ip_catalog
create_ip -vlnv xilinx.com:ip:dpuczdx8g:4.1 -module_name dpu_ooc -dir $out
set ip [get_ips dpu_ooc]
set_property -dict [m7_dpu_zynquplus_props] $ip
set_property -dict [m7_dpu_props $::env(FPGA_DPU_ARCH) $::env(FPGA_DPU_URAM)] $ip
set f [open $out/IPCONFIG.txt w]
foreach p [lsort [list_property $ip CONFIG.*]] { puts $f "$p=[get_property $p $ip]" }
close $f
generate_target all [get_files $out/dpu_ooc/dpu_ooc.xci]
foreach a [glob -nocomplain $out/dpu_ooc/arch.json $out/dpu_ooc/*/arch.json] { file copy -force $a $out/arch.json }
set mhz $::env(FPGA_DPU_CLK_MHZ)
set f [open $out/ooc_clocks.xdc w]
puts $f "create_clock -period [format %.4f [expr {1000.0/$mhz}]] -name clk_dpu \[get_ports m_axi_dpu_aclk\]"
puts $f "create_clock -period [format %.4f [expr {500.0/$mhz}]] -name clk_dsp \[get_ports dpu_2x_clk\]"
puts $f "create_clock -period 10.000 -name clk_s_axi \[get_ports s_axi_aclk\]"
close $f
read_xdc -mode out_of_context $out/ooc_clocks.xdc
set_property PROCESSING_ORDER EARLY [get_files $out/ooc_clocks.xdc]
synth_design -top dpu_ooc -part $part -mode out_of_context
report_utilization -file $out/utilization_ooc_synth.rpt
report_utilization -hierarchical -hierarchical_depth 3 -file $out/utilization_ooc_synth_hier.rpt
report_timing_summary -max_paths 10 -file $out/timing_ooc_synth.rpt
puts "DPU_OOC done"
