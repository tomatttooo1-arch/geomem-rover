# Approved dual-grid hardware, Vitis HLS 2022.2 / KV260 / 100 MHz.
set here [file dirname [file normalize [info script]]]
set repo [file normalize [file join $here ../..]]
set project [file join $repo runs planner-dual-20261104 hls]
if {[info exists ::env(dev2_HLS_PROJECT)]} {set project $::env(dev2_HLS_PROJECT)}
file mkdir [file dirname $project]
cd [file dirname $project]
open_project [file tail $project]
set_top planner_field_dual
add_files [file join $here src kernel_hls.cpp] -cflags "-std=c++14 -ffp-contract=off"
add_files -tb [file join $here tb testbench.cpp] -cflags "-std=c++14 -I[file join $here src] -ffp-contract=off"
open_solution solution -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 10 -name default
config_compile -pragma_strict_mode=true
config_interface -m_axi_buffer_impl bram
config_export -format ip_catalog -rtl verilog -vendor fpga -library navigation -version 1.0
