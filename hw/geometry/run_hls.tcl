# Batch entry for the planned Vitis HLS 2022.2 flow; not yet run with Vitis.
# No default device: the competition board has not been confirmed.
foreach key {FPGA_HLS_PART FPGA_HLS_BUILD FPGA_HLS_CLOCK_NS} {
    if {![info exists ::env($key)] || $::env($key) eq ""} {
        error "Set $key explicitly before running Vitis HLS"
    }
}
set top fpga_backproject
if {[info exists ::env(FPGA_HLS_TOP)]} { set top $::env(FPGA_HLS_TOP) }
if {$top ni {fpga_backproject fpga_visibility_counts}} { error "Unknown FPGA_HLS_TOP" }
if {![string is double -strict $::env(FPGA_HLS_CLOCK_NS)] || $::env(FPGA_HLS_CLOCK_NS) <= 0} {
    error "FPGA_HLS_CLOCK_NS must be positive"
}
set src [file dirname [file normalize [info script]]]
set build [file normalize $::env(FPGA_HLS_BUILD)]
if {[file exists $build]} { error "Build destination already exists; choose a fresh directory" }
if {$build eq $src || [string first "$src/" "$build/"] == 0} {
    error "Use a build destination outside the source directory"
}
# open_project 只接受项目名，不接受路径（2022.2 实测：路径含 '/' 直接报 HLS 200-70）。
# 因此先进到父目录，再用目录名开工程。上面的越界/复用守卫仍然作用在 $build 绝对路径上。
file mkdir [file dirname $build]
cd [file dirname $build]
open_project [file tail $build]
set_top $top
add_files "$src/geometry.cpp" -cflags "-std=c++11 -ffp-contract=off"
add_files -tb "$src/testbench.cpp" -cflags "-std=c++11 -ffp-contract=off"
open_solution solution -flow_target vivado
set_part $::env(FPGA_HLS_PART)
create_clock -period $::env(FPGA_HLS_CLOCK_NS)
csim_design
csynth_design
cosim_design -rtl verilog
exit
