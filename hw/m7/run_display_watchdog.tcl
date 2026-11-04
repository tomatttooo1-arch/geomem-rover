# Vivado 2022.2 behavioral RTL simulation and K26 synthesis for the independent
# display watchdog. This does not build a video timing generator, framebuffer,
# DMA path or board design. Caller supplies a fresh build directory.
foreach key {FPGA_M7_PART FPGA_M7_WATCHDOG_BUILD FPGA_M7_WATCHDOG_BYPASS FPGA_M7_WATCHDOG_CLOCK_NS} {
    if {![info exists ::env($key)] || $::env($key) eq ""} {
        error "Set $key explicitly before running Vivado"
    }
}
if {$::env(FPGA_M7_WATCHDOG_BYPASS) ni {0 1}} {
    error "FPGA_M7_WATCHDOG_BYPASS must be 0 or 1"
}
if {![string is double -strict $::env(FPGA_M7_WATCHDOG_CLOCK_NS)] ||
    $::env(FPGA_M7_WATCHDOG_CLOCK_NS) <= 0} {
    error "FPGA_M7_WATCHDOG_CLOCK_NS must be positive"
}
set src [file dirname [file normalize [info script]]]
set build [file normalize $::env(FPGA_M7_WATCHDOG_BUILD)]
if {[file exists $build]} { error "Build destination already exists" }
if {$build eq $src || [string first "$src/" "$build/"] == 0} {
    error "Build destination must be outside the source directory"
}
file mkdir [file dirname $build]
create_project display_watchdog $build -part $::env(FPGA_M7_PART)
add_files -norecurse "$src/display_watchdog.sv"
add_files -fileset sim_1 -norecurse "$src/display_watchdog_tb.sv"
set xdc "$build/display_watchdog_clock.xdc"
set xdc_file [open $xdc w]
puts $xdc_file [format {create_clock -name display_scan_clk -period %s [get_ports display_scan_clk]} $::env(FPGA_M7_WATCHDOG_CLOCK_NS)]
close $xdc_file
add_files -fileset constrs_1 -norecurse $xdc
set_property top m7_display_watchdog [get_filesets sources_1]
# This is an isolated RTL module with wide identity/diagnostic ports, not a
# board top. Out-of-context synthesis avoids adding I/O buffers or interpreting
# every port as a package pin.
set_property -name {STEPS.SYNTH_DESIGN.ARGS.MORE OPTIONS} -value {-mode out_of_context} -objects [get_runs synth_1]
set_property top m7_display_watchdog_tb [get_filesets sim_1]
set_property generic "BYPASS=$::env(FPGA_M7_WATCHDOG_BYPASS)" [get_filesets sim_1]
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
# launch_simulation otherwise runs its generated 1000 ns script first. The TB
# finishes at its own $finish; a second run then has only the forever clock and
# never terminates. Starting at 0 ns followed by exactly one run-all executes
# the identical TB/assertion set once. No validation coverage is removed.
set_property xsim.simulate.runtime {0ns} [get_filesets sim_1]
launch_simulation -simset sim_1 -mode behavioral
set run_failed [catch {run all} run_message]
catch {close_sim} close_message
set sim_log "$build/display_watchdog.sim/sim_1/behav/xsim/simulate.log"
if {![file isfile $sim_log]} {
    error "simulation did not create the expected log: $sim_log"
}
set sim_file [open $sim_log r]
set sim_text [read $sim_file]
close $sim_file
set pass_marker "M7 display watchdog RTL testbench PASS"
set sim_lines [split $sim_text "\n"]
set pass_indices [lsearch -all -exact $sim_lines $pass_marker]
set all_failure_indices [lsearch -all -regexp $sim_lines {^WATCHDOG TB FAIL:}]
set bypass_indices [lsearch -all -regexp $sim_lines {^WATCHDOG TB FAIL: stopped stream must hide old framebuffer pixels at [0-9]+$}]
set tool_error_indices [lsearch -all -regexp $sim_lines {^ERROR:}]
set has_fatal [expr {[string first "Fatal:" $sim_text] >= 0}]

if {$::env(FPGA_M7_WATCHDOG_BYPASS) eq "1"} {
    if {$run_failed || [llength $all_failure_indices] != 1 ||
        [llength $bypass_indices] != 1 || [llength $pass_indices] != 1 ||
        [lindex $pass_indices 0] <= [lindex $bypass_indices 0] ||
        $has_fatal || [llength $tool_error_indices] != 0} {
        error "BYPASS control must have one exact designated failure, then one PASS, and no tool failure"
    }
    puts "BYPASS_CONTROL_MATCHED designated stopped-stream assertion; remaining checks completed"
    error "BYPASS_CONTROL_EXPECTED_NONZERO_EXIT"
}
if {$run_failed || [llength $pass_indices] != 1 ||
    [llength $all_failure_indices] != 0 || $has_fatal ||
    [llength $tool_error_indices] != 0} {
    error "positive watchdog RTL simulation did not reach one clean PASS: $run_message"
}
# Do not use the project launch_runs path here. Vivado 2022.2 can spawn a
# second process and query WebTalk host information on that path; this machine
# reproduced a parent crash in libudev/libXil_lmgr even though the child OOC
# synthesis completed. Close the simulation project and synthesize the exact
# same frozen RTL directly in this process instead.
close_project
read_verilog -sv "$src/display_watchdog.sv"
read_xdc $xdc
synth_design -top m7_display_watchdog -part $::env(FPGA_M7_PART) -mode out_of_context
if {[llength [get_clocks display_scan_clk]] != 1} {
    error "synthesized watchdog is missing the requested display_scan_clk constraint"
}
set latch_cells [get_cells -hierarchical -quiet -filter {REF_NAME =~ LD*}]
puts "WATCHDOG_LATCH_COUNT [llength $latch_cells]"
if {[llength $latch_cells] != 0} {
    error "watchdog synthesis inferred latch primitives: $latch_cells"
}
report_utilization -file "$build/utilization.rpt"
report_timing_summary -report_unconstrained -file "$build/timing_summary.rpt"
report_clock_utilization -file "$build/clock_utilization.rpt"
check_timing -verbose -file "$build/check_timing.rpt"
report_drc -file "$build/drc.rpt"
write_checkpoint -force "$build/display_watchdog_synth.dcp"
puts "WATCHDOG_DIRECT_OOC_SYNTH_PASS"
exit
