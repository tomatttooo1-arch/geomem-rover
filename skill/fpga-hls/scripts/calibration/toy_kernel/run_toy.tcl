# usage: TOY_BUILD=<proj_dir> [TOY_FAULT=0|1|2] vitis_hls -f run_toy.tcl
set src [file dirname [file normalize [info script]]]
set build [file normalize $::env(TOY_BUILD)]
set fault 0
if {[info exists ::env(TOY_FAULT)]} { set fault $::env(TOY_FAULT) }
file mkdir [file dirname $build]
cd [file dirname $build]
open_project [file tail $build]
set_top toy
add_files "$src/toy.cpp" -cflags "-DTOY_FAULT=$fault"
add_files -tb "$src/tb_toy.cpp"
open_solution solution -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 10
csim_design
csynth_design
cosim_design -rtl verilog
exit
