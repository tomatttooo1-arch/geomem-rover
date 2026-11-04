set source_dir [file dirname [file normalize [info script]]]
set project [file join $source_dir .memory/build/hls]
file mkdir [file dirname $project]
cd [file dirname $project]
open_project [file tail $project]
set_top fpga_m7_object_roi_fx_axi
add_files [file join $source_dir roi_fx.cpp] -cflags "-std=c++14"
add_files -tb [file join $source_dir tb_roi_fx.cpp] -cflags "-std=c++14"
open_solution solution -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 10 -name default
config_compile -pipeline_loops 0 -pragma_strict_mode=true
config_interface -m_axi_addr64=true
csim_design -argv [file join $source_dir .memory/vectors]
cosim_design -argv [file join $source_dir .memory/vectors] -rtl verilog
