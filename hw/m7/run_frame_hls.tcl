# Vitis HLS 2022.2 C simulation -> synthesis -> Verilog co-simulation.
# A caller must select a fresh build, device, and clock explicitly.
foreach key {FPGA_M7_FRAME_PART FPGA_M7_FRAME_BUILD FPGA_M7_FRAME_CLOCK_NS} {
    if {![info exists ::env($key)] || $::env($key) eq ""} {
        error "Set $key explicitly before running Vitis HLS"
    }
}
if {![string is double -strict $::env(FPGA_M7_FRAME_CLOCK_NS)] ||
    $::env(FPGA_M7_FRAME_CLOCK_NS) <= 0} {
    error "FPGA_M7_FRAME_CLOCK_NS must be positive"
}
set src [file dirname [file normalize [info script]]]
set build [file normalize $::env(FPGA_M7_FRAME_BUILD)]
if {[file exists $build]} { error "Build destination already exists" }
if {$build eq $src || [string first "$src/" "$build/"] == 0} {
    error "Build destination must be outside source directory"
}
file mkdir [file dirname $build]
cd [file dirname $build]
open_project [file tail $build]
set_top fpga_m7_depth_frame
add_files "$src/frame_depth.cpp" -cflags "-std=c++11 -ffp-contract=off"
add_files -tb "$src/frame_testbench.cpp" -cflags "-std=c++11 -ffp-contract=off"
open_solution solution -flow_target vivado
set_part $::env(FPGA_M7_FRAME_PART)
create_clock -period $::env(FPGA_M7_FRAME_CLOCK_NS)
csim_design
csynth_design
cosim_design -rtl verilog
exit
