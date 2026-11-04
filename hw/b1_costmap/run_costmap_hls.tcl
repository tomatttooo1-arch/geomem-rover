# Vitis HLS 2022.2 / K26: B1 navigation-grid core, C sim -> synthesis (no cosim, no board).
# Usage (from a fresh run directory outside the source tree):
#   FPGA_B1_PART=xck26-sfvc784-2LV-c FPGA_B1_BUILD=<new dir> FPGA_B1_CLOCK_NS=7.0 \
#   B1_VECTORS=<bench --vectors dir>/vectors.bin vitis_hls -f <src>/run_costmap_hls.tcl
#   FPGA_B1_FLOW=csim runs C simulation only.
foreach key {FPGA_B1_PART FPGA_B1_BUILD FPGA_B1_CLOCK_NS} {
    if {![info exists ::env($key)] || $::env($key) eq ""} { error "Set $key explicitly" }
}
set flow full
if {[info exists ::env(FPGA_B1_FLOW)] && $::env(FPGA_B1_FLOW) ne ""} { set flow $::env(FPGA_B1_FLOW) }
set src [file dirname [file normalize [info script]]]
set build [file normalize $::env(FPGA_B1_BUILD)]
if {[file exists $build]} { error "Build destination already exists" }
if {[string first "$src/" "$build/"] == 0} { error "Build destination must be outside the source directory" }
file mkdir [file dirname $build]
cd [file dirname $build]
open_project [file tail $build]
set_top fpga_b1_costmap
add_files "$src/costmap.cpp" -cflags "-std=c++11 -ffp-contract=off"
add_files -tb "$src/tb_costmap.cpp" -cflags "-std=c++11 -ffp-contract=off"
open_solution solution -flow_target vivado
set_part $::env(FPGA_B1_PART)
create_clock -period $::env(FPGA_B1_CLOCK_NS)
csim_design
if {$flow eq "full"} { csynth_design }
exit
