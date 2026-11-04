# Vitis HLS 2022.2: display pixel candidate, C sim -> synthesis -> RTL cosim.
# A fresh build outside source is mandatory; report is not a board result.
foreach key {FPGA_M7_PART FPGA_M7_DISPLAY_BUILD FPGA_M7_CLOCK_NS} {
    if {![info exists ::env($key)] || $::env($key) eq ""} {
        error "Set $key explicitly before running Vitis HLS"
    }
}
if {![string is double -strict $::env(FPGA_M7_CLOCK_NS)] ||
    $::env(FPGA_M7_CLOCK_NS) <= 0} {
    error "FPGA_M7_CLOCK_NS must be positive"
}
set src [file dirname [file normalize [info script]]]
set build [file normalize $::env(FPGA_M7_DISPLAY_BUILD)]
if {[file exists $build]} { error "Build destination already exists" }
if {$build eq $src || [string first "$src/" "$build/"] == 0} {
    error "Build destination must be outside the source directory"
}
file mkdir [file dirname $build]
cd [file dirname $build]
open_project [file tail $build]
set_top fpga_m7_display_overlay
add_files "$src/display_overlay.cpp" -cflags "-std=c++11"
add_files -tb "$src/display_testbench.cpp" -cflags "-std=c++11"
open_solution solution -flow_target vivado
set_part $::env(FPGA_M7_PART)
create_clock -period $::env(FPGA_M7_CLOCK_NS)
csim_design
csynth_design
cosim_design -rtl verilog
exit
