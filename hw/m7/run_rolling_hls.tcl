# Vitis HLS 2022.2/K26: rolling log-odds map, C sim -> synthesis -> C/RTL co-sim.
# Caller chooses a fresh build outside the source tree, device and clock.
#   FPGA_M7_ROLL_FLOW=full (default) runs csim, csynth and Verilog cosim;
#   FPGA_M7_ROLL_FLOW=csim runs C simulation only.
#   FPGA_M7_ROLL_MUTANT=<NAME> adds the test-only fault macro
#   FPGA_M7_ROLL_MUTANT_<NAME> to the kernel's -cflags; the testbench must then
#   fail. NOTE (observed in run hls-v2-mutants, csim.mk): Vitis HLS 2022.2 also
#   appends design-file -cflags to the global IFLAG, so the testbench-side
#   reference is compiled with the SAME fault; failures there come from the
#   testbench's hand-computed known answers. The kernel-only fault build
#   (kernel vs clean reference) is exercised natively by tests/test_m7_rolling.py.
#   A failing csim makes vitis_hls exit 1 (checked in hls-v2-mutants/
#   vitis_hls_SHIFT_SIGN_rc.log); still judge by the FAIL/PASS lines in the log.
foreach key {FPGA_M7_PART FPGA_M7_BUILD FPGA_M7_CLOCK_NS} {
    if {![info exists ::env($key)] || $::env($key) eq ""} {
        error "Set $key explicitly before running Vitis HLS"
    }
}
if {![string is double -strict $::env(FPGA_M7_CLOCK_NS)] ||
    $::env(FPGA_M7_CLOCK_NS) <= 0} {
    error "FPGA_M7_CLOCK_NS must be positive"
}
set flow full
if {[info exists ::env(FPGA_M7_ROLL_FLOW)] && $::env(FPGA_M7_ROLL_FLOW) ne ""} {
    set flow $::env(FPGA_M7_ROLL_FLOW)
}
if {$flow ne "full" && $flow ne "csim"} { error "FPGA_M7_ROLL_FLOW must be full or csim" }
set kernel_flags "-std=c++11 -ffp-contract=off"
if {[info exists ::env(FPGA_M7_ROLL_MUTANT)] && $::env(FPGA_M7_ROLL_MUTANT) ne ""} {
    set mutant $::env(FPGA_M7_ROLL_MUTANT)
    if {[lsearch -exact {UNKNOWN_CLEARS NO_FREE_CLEAR SHIFT_SIGN NO_EVICT IGNORE_EPOCH} $mutant] < 0} {
        error "unknown kernel mutant $mutant"
    }
    append kernel_flags " -DFPGA_M7_ROLL_MUTANT_$mutant"
}
# FPGA_M7_ROLL_CFLAGS (optional, B1h 2026-11-02): extra flags for the kernel and
#   the testbench files, e.g. -DFPGA_M7_ROLL_GRID (feature bit for op GRID).
if {[info exists ::env(FPGA_M7_ROLL_CFLAGS)] && $::env(FPGA_M7_ROLL_CFLAGS) ne ""} {
    append kernel_flags " $::env(FPGA_M7_ROLL_CFLAGS)"
    set tb_flags "-std=c++11 -ffp-contract=off $::env(FPGA_M7_ROLL_CFLAGS)"
} else {
    set tb_flags "-std=c++11 -ffp-contract=off"
}
set src [file dirname [file normalize [info script]]]
set build [file normalize $::env(FPGA_M7_BUILD)]
if {[file exists $build]} { error "Build destination already exists" }
if {$build eq $src || [string first "$src/" "$build/"] == 0} {
    error "Build destination must be outside the source directory"
}
file mkdir [file dirname $build]
cd [file dirname $build]
open_project [file tail $build]
set_top fpga_m7_rolling_map
add_files "$src/rolling_map.cpp" -cflags $kernel_flags
add_files -tb "$src/rolling_testbench.cpp" -cflags $tb_flags
add_files -tb "$src/rolling_map_ref.cpp" -cflags $tb_flags
add_files -tb "$src/ray_flags.cpp" -cflags $tb_flags
open_solution solution -flow_target vivado
set_part $::env(FPGA_M7_PART)
create_clock -period $::env(FPGA_M7_CLOCK_NS)
csim_design
if {$flow eq "full"} {
    csynth_design
    cosim_design -rtl verilog
}
exit
