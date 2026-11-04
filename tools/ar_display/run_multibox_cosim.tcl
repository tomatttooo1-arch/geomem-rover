# Multi-box co-simulation gate (D, 2026-10-11): C sim -> synthesis -> Verilog
# co-simulation of fpga_m7_ar_overlay_axi with tb_ar_overlay_multibox.cpp.
# The core source comes from AR_MB_CORE (a directory made by
# tools/ar_display/multibox_cosim.py: clean copy or a __SYNTHESIS__-only
# mutant); wrapper and testbench come from the repository. No IP export.
#   AR_MB_CORE  AR_MB_DATA (ar_mb_cases.bin)  AR_MB_BUILD (must not exist)
#   AR_MB_PART (default xck26-sfvc784-2LV-c)  AR_MB_CLOCK_NS (default 10)
# Markers: AR_MB_CSIM_PASS, AR_MB_COSIM_PASS / AR_MB_COSIM_FAIL.
foreach key {AR_MB_CORE AR_MB_DATA AR_MB_BUILD} {
    if {![info exists ::env($key)] || $::env($key) eq ""} { error "Set $key" }
}
set part xck26-sfvc784-2LV-c
if {[info exists ::env(AR_MB_PART)]} { set part $::env(AR_MB_PART) }
set clk 10
if {[info exists ::env(AR_MB_CLOCK_NS)]} { set clk $::env(AR_MB_CLOCK_NS) }
set here [file dirname [file normalize [info script]]]
set hls [file normalize "$here/../../hw/m7/pl/hls"]
set core [file normalize $::env(AR_MB_CORE)]
set data [file normalize $::env(AR_MB_DATA)]
set build [file normalize $::env(AR_MB_BUILD)]
if {[file exists $build]} { error "Build destination already exists: $build" }
set cflags "-std=c++11"
file mkdir [file dirname $build]
cd [file dirname $build]
open_project [file tail $build]
set_top fpga_m7_ar_overlay_axi
add_files "$hls/m7_ar_overlay_axi.cpp" -cflags $cflags
add_files "$core/ar_overlay.cpp" -cflags "$cflags -I$core"
add_files -tb "$hls/tb_ar_overlay_multibox.cpp" -cflags $cflags
add_files -tb $data
open_solution solution -flow_target vivado
set_part $part
create_clock -period $clk
csim_design
puts "AR_MB_CSIM_PASS"
csynth_design
if {[catch {cosim_design -rtl verilog} err]} {
    puts "AR_MB_COSIM_FAIL $err"
} else {
    puts "AR_MB_COSIM_PASS"
}
exit
