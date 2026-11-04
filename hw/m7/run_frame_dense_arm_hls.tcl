# Dense frame RTL probe, one arm per fresh build (2026-09-26).
# Differs from run_frame_dense_hls.tcl only in that the design file is chosen
# from a fixed allowlist (FPGA_M7_DENSE_ARM) and an arm metadata file is written;
# the testbench, flags, part/clock handling and csim/csynth/cosim order are the
# same.  Arms other than A are diagnostics and are not production sources.
foreach key {FPGA_M7_DENSE_PART FPGA_M7_DENSE_BUILD FPGA_M7_DENSE_CLOCK_NS FPGA_M7_DENSE_ARM} {
    if {![info exists ::env($key)] || $::env($key) eq ""} {
        error "Set $key explicitly before running Vitis HLS"
    }
}
if {![string is double -strict $::env(FPGA_M7_DENSE_CLOCK_NS)] ||
    $::env(FPGA_M7_DENSE_CLOCK_NS) <= 0} {
    error "FPGA_M7_DENSE_CLOCK_NS must be positive"
}
set src [file dirname [file normalize [info script]]]
array set designs {
    A      frame_depth.cpp
    UV     probe/frame_depth_uv_counter.cpp
    QPIPE  probe/frame_depth_quant_pipeline.cpp
    MUTANT probe/frame_depth_rtl_mutant.cpp
}
set arm $::env(FPGA_M7_DENSE_ARM)
if {![info exists designs($arm)]} { error "Unknown dense arm $arm" }
set design [file join $src $designs($arm)]
set build [file normalize $::env(FPGA_M7_DENSE_BUILD)]
if {[file exists $build]} { error "Build destination already exists" }
if {$build eq $src || [string first "$src/" "$build/"] == 0} {
    error "Build destination must be outside source directory"
}
file mkdir [file dirname $build]
set metadata [file join [file dirname $build] ARM_METADATA.txt]
if {[file exists $metadata]} { error "Arm metadata already exists" }
set meta [open $metadata w]
puts $meta "probe=m7_dense_frame_arm"
puts $meta "arm=$arm"
puts $meta "part=$::env(FPGA_M7_DENSE_PART)"
puts $meta "clock_ns=$::env(FPGA_M7_DENSE_CLOCK_NS)"
foreach path [list $design "$src/frame_depth.hpp" "$src/frame_dense_testbench.cpp" [info script]] {
    puts $meta "source_[exec sha256sum [file normalize $path]]"
}
close $meta
cd [file dirname $build]
open_project [file tail $build]
set_top fpga_m7_depth_frame
add_files $design -cflags "-std=c++11 -ffp-contract=off"
add_files -tb "$src/frame_dense_testbench.cpp" -cflags "-std=c++11 -ffp-contract=off"
open_solution solution -flow_target vivado
set_part $::env(FPGA_M7_DENSE_PART)
create_clock -period $::env(FPGA_M7_DENSE_CLOCK_NS)
csim_design
csynth_design
cosim_design -rtl verilog
exit
