# Vitis HLS C sim -> synthesis for the mask-free multi-box region check
# (OBJECT_ROI_SPEC_20261011; E2 pipelined structure, spec section 11).
# Default: core top fpga_m7_object_roi_batch, csim + csynth (as in E1).
#   FPGA_M7_ROI_MUTANT=<NAME> adds -DFPGA_M7_ROI_MUTANT_<NAME> to the kernel
#   flags; csim must then fail. Use mutants with csim only (FPGA_M7_ROI_CSIM_ONLY=1).
#   Exception: LANE_SHIFT_RTL changes only the synthesized RTL (C path intact),
#   so csim passes and C/RTL co-simulation (FPGA_M7_ROI_COSIM=1) must fail.
# Optional (E2), all off unless set to the stated value:
#   FPGA_M7_ROI_TOP=axi       synthesize the AXI shell fpga_m7_object_roi_axi
#                            (pl/hls/m7_object_roi_axi.cpp, tb_object_roi_axi.cpp)
#   FPGA_M7_ROI_TB_SCENE=<n>  AXI bench scene (0 all; 1..3 one full frame each;
#                            4 forty small random batches)
#   FPGA_M7_ROI_COSIM=1       Verilog C/RTL co-simulation after csynth
#   FPGA_M7_ROI_DEBUG_KEYS=1  test-only: result front/behind carry the lane
#                            threshold keys; AXI bench names their owner box
#   FPGA_M7_ROI_VIVADO_SYN=1  Vivado out-of-context synthesis (export_design
#                            -flow syn) for post-synthesis resources; no bitstream
foreach key {FPGA_M7_OBJECT_PART FPGA_M7_OBJECT_BUILD FPGA_M7_OBJECT_CLOCK_NS} {
    if {![info exists ::env($key)] || $::env($key) eq ""} {
        error "Set $key explicitly before running Vitis HLS"
    }
}
if {![string is double -strict $::env(FPGA_M7_OBJECT_CLOCK_NS)] ||
    $::env(FPGA_M7_OBJECT_CLOCK_NS) <= 0} {
    error "FPGA_M7_OBJECT_CLOCK_NS must be positive"
}
set src [file dirname [file normalize [info script]]]
set build [file normalize $::env(FPGA_M7_OBJECT_BUILD)]
if {[file exists $build]} { error "Build destination already exists" }
if {$build eq $src || [string first "$src/" "$build/"] == 0} {
    error "Build destination must be outside the source directory"
}
set kernel_flags "-std=c++11 -ffp-contract=off"
if {[info exists ::env(FPGA_M7_ROI_MUTANT)] && $::env(FPGA_M7_ROI_MUTANT) ne ""} {
    append kernel_flags " -DFPGA_M7_ROI_MUTANT_$::env(FPGA_M7_ROI_MUTANT)"
}
proc roi_env_is {key value} {
    return [expr {[info exists ::env($key)] && $::env($key) eq $value}]
}
# Test-only flags given to kernel AND bench (diagnostics / RTL-only controls).
set shared_flags ""
if {[roi_env_is FPGA_M7_ROI_DEBUG_KEYS 1]} { append shared_flags " -DFPGA_M7_ROI_DEBUG_KEYS" }
append kernel_flags $shared_flags
set use_axi [roi_env_is FPGA_M7_ROI_TOP axi]
if {[info exists ::env(FPGA_M7_ROI_TOP)] && !$use_axi && $::env(FPGA_M7_ROI_TOP) ne "core"} {
    error "FPGA_M7_ROI_TOP must be core or axi"
}
set scene 0
if {[info exists ::env(FPGA_M7_ROI_TB_SCENE)] && $::env(FPGA_M7_ROI_TB_SCENE) ne ""} {
    set scene $::env(FPGA_M7_ROI_TB_SCENE)
    if {![string is integer -strict $scene] || $scene < 0 || $scene > 4} {
        error "FPGA_M7_ROI_TB_SCENE must be 0..4"
    }
}
file mkdir [file dirname $build]
cd [file dirname $build]
open_project [file tail $build]
if {$use_axi} {
    set_top fpga_m7_object_roi_axi
    add_files "$src/pl/hls/m7_object_roi_axi.cpp" -cflags $kernel_flags
    add_files "$src/object_evidence.cpp" -cflags $kernel_flags
    add_files -tb "$src/pl/hls/tb_object_roi_axi.cpp" \
        -cflags "-std=c++11 -ffp-contract=off -DFPGA_M7_ROI_TB_SCENE=$scene$shared_flags"
} else {
    set_top fpga_m7_object_roi_batch
    add_files "$src/object_evidence.cpp" -cflags $kernel_flags
    add_files -tb "$src/object_roi_testbench.cpp" -cflags "-std=c++11 -ffp-contract=off"
}
open_solution solution -flow_target vivado
set_part $::env(FPGA_M7_OBJECT_PART)
create_clock -period $::env(FPGA_M7_OBJECT_CLOCK_NS)
csim_design
if {[info exists ::env(FPGA_M7_ROI_CSIM_ONLY)] && $::env(FPGA_M7_ROI_CSIM_ONLY) eq "1"} { exit }
csynth_design
if {[roi_env_is FPGA_M7_ROI_COSIM 1]} { cosim_design -rtl verilog }
if {[roi_env_is FPGA_M7_ROI_VIVADO_SYN 1]} { export_design -flow syn -rtl verilog -format ip_catalog }
exit
