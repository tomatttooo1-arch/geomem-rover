# Vitis HLS 2022.2: C sim -> synthesis -> IP export -> (optional) C/RTL cosim
# for one AXI shell top. Caller selects everything explicitly:
#   FPGA_M7_PL_TOP     fpga_m7_frame_axi | fpga_m7_object_axi | fpga_m7_display_axi
#                     | fpga_m7_object_roi_axi (E2 region check)
#                     | fpga_m7_ar_overlay_axi (D, 2026-10-11)
#   FPGA_M7_PL_PART    e.g. xck26-sfvc784-2LV-c
#   FPGA_M7_PL_CLOCK_NS  target clock period in ns
#   FPGA_M7_PL_BUILD   fresh build directory (must not exist, outside sources)
#   FPGA_M7_PL_IPREPO  directory that receives a copy of the exported IP
#   FPGA_M7_PL_COSIM   1 to run Verilog co-simulation after export, else 0
# Optional:
#   FPGA_M7_PL_FRAME_IMPL  "stream" selects the A1 streaming frame top
#                     (m7_frame_stream.cpp; the reference frame_depth.cpp is
#                     then a testbench file only, bench tb_frame_stream.cpp)
#   FPGA_M7_TB_FRAMES  frame file read by tb_frame_stream.cpp (csim and cosim)
#   FPGA_M7_PL_TB      (stream only) testbench file in this directory instead of
#                     tb_frame_stream.cpp (GEO_FX1: tb_frame_stream_fx.cpp)
#   FPGA_M7_PL_CFLAGS  extra compile flags for every source (A2: -DFPGA_M7_LANES=N,
#                     cosim positive controls -DFPGA_M7_HWFAULT_*)
#   FPGA_M7_PL_CLOCK_UNCERTAINTY  clock uncertainty for create_clock (ns or
#                     percent such as 12.5%); unset = HLS default (27%)
#   FPGA_M7_PL_WALK_STRUCT_STRICT  (stream only) 0 = a walk-structure alarm after csynth
#                     (tools/check_walk_struct.sh) does not stop the build; default stops
#   FPGA_M7_PL_EXPORT_FLOW  "syn" also runs Vivado out-of-context synthesis in
#                     export_design (utilization / latch / timing reports under
#                     solution/impl/verilog/report); needs
#                     LD_PRELOAD=/lib/x86_64-linux-gnu/libudev.so.1 on this host
# The exported IP is copied BEFORE co-simulation so a block design build can
# start while co-simulation still runs; co-simulation never edits the IP.
foreach key {FPGA_M7_PL_TOP FPGA_M7_PL_PART FPGA_M7_PL_CLOCK_NS FPGA_M7_PL_BUILD
             FPGA_M7_PL_IPREPO FPGA_M7_PL_COSIM} {
    if {![info exists ::env($key)] || $::env($key) eq ""} {
        error "Set $key explicitly before running Vitis HLS"
    }
}
set top $::env(FPGA_M7_PL_TOP)
set clk $::env(FPGA_M7_PL_CLOCK_NS)
if {![string is double -strict $clk] || $clk <= 0} { error "bad clock $clk" }
set here [file dirname [file normalize [info script]]]
set core [file normalize "$here/../.."]
set build [file normalize $::env(FPGA_M7_PL_BUILD)]
if {[file exists $build]} { error "Build destination already exists: $build" }
if {[string first "$core/" "$build/"] == 0} { error "Build must be outside $core" }

# top -> {wrapper core testbench extra-cflags}
array set spec {
    fpga_m7_frame_axi   {m7_frame_axi.cpp   frame_depth.cpp     tb_frame_axi.cpp   {-ffp-contract=off}}
    fpga_m7_object_axi  {m7_object_axi.cpp  object_evidence.cpp tb_object_axi.cpp  {-ffp-contract=off}}
    fpga_m7_display_axi {m7_display_axi.cpp display_overlay.cpp tb_display_axi.cpp {}}
    fpga_m7_object_roi_axi {m7_object_roi_axi.cpp object_evidence.cpp tb_object_roi_axi.cpp {-ffp-contract=off}}
    fpga_m7_ar_overlay_axi {m7_ar_overlay_axi.cpp ar_overlay.cpp tb_ar_overlay_axi.cpp {}}
    fpga_m7_rolling_axi {m7_rolling_axi.cpp rolling_map.cpp tb_rolling_axi.cpp {-Wno-unknown-pragmas}}
}
if {![info exists spec($top)]} { error "unknown top $top" }
lassign $spec($top) wrapper coresrc tb extra
set impl ""
if {[info exists ::env(FPGA_M7_PL_FRAME_IMPL)]} { set impl $::env(FPGA_M7_PL_FRAME_IMPL) }
if {$impl ne "" && $impl ne "stream"} { error "unknown FPGA_M7_PL_FRAME_IMPL $impl" }
if {$impl eq "stream" && $top ne "fpga_m7_frame_axi"} { error "FPGA_M7_PL_FRAME_IMPL=stream is only for fpga_m7_frame_axi" }
set cflags "-std=c++11 $extra"
if {[info exists ::env(FPGA_M7_PL_CFLAGS)] && $::env(FPGA_M7_PL_CFLAGS) ne ""} {
    append cflags " $::env(FPGA_M7_PL_CFLAGS)"
}
set flow ""
if {[info exists ::env(FPGA_M7_PL_EXPORT_FLOW)]} { set flow $::env(FPGA_M7_PL_EXPORT_FLOW) }
# csim (2026-10-26, v3 L2): C simulation only, no synthesis / export (trace replays, mutants)
if {$flow ne "" && $flow ne "syn" && $flow ne "csim"} { error "unknown FPGA_M7_PL_EXPORT_FLOW $flow" }
puts "FPGA_M7_PL_CFLAGS_USED $cflags"

file mkdir [file dirname $build]
cd [file dirname $build]
open_project [file tail $build]
set_top $top
if {$impl eq "stream"} {
    add_files "$here/m7_frame_stream.cpp" -cflags $cflags
    add_files -tb "$core/$coresrc" -cflags $cflags
    # GEO_FX3: PS constants + host model of the fixed-point core (benches only)
    add_files -tb "$core/frame_fx.cpp" -cflags $cflags
    set tbfile tb_frame_stream.cpp
    if {[info exists ::env(FPGA_M7_PL_TB)] && $::env(FPGA_M7_PL_TB) ne ""} { set tbfile $::env(FPGA_M7_PL_TB) }
    add_files -tb "$here/$tbfile" -cflags $cflags
    puts "FPGA_M7_PL_TB_USED $tbfile"
    set desc "M7 AXI frame top $top (A1 streaming + A2 ray lanes, 2026-10-11)"
} else {
    add_files "$here/$wrapper" -cflags $cflags
    add_files "$core/$coresrc" -cflags $cflags
    add_files -tb "$here/$tb" -cflags $cflags
    set desc "M7 AXI shell $top (unchanged core, 2026-09-26)"
}
open_solution solution -flow_target vivado
set_part $::env(FPGA_M7_PL_PART)
create_clock -period $clk
if {[info exists ::env(FPGA_M7_PL_CLOCK_UNCERTAINTY)] && $::env(FPGA_M7_PL_CLOCK_UNCERTAINTY) ne ""} {
    set_clock_uncertainty $::env(FPGA_M7_PL_CLOCK_UNCERTAINTY)
    puts "FPGA_M7_PL_CLOCK_UNCERTAINTY_USED $::env(FPGA_M7_PL_CLOCK_UNCERTAINTY)"
}
csim_design
if {$flow eq "csim"} { puts "FPGA_M7_PL_CSIM_DONE $top"; exit }
csynth_design
# GEO_FX3: the fixed-point frame core's X1 walk structure (hw/m7/pl/tools/check_walk_struct.sh).
# Alarm => stop before export unless env FPGA_M7_PL_WALK_STRUCT_STRICT=0 (then only the alarm line).
if {$impl eq "stream"} {
    set wchk [file normalize "$here/../tools/check_walk_struct.sh"]
    set wrc [catch {exec bash $wchk "$build/solution/syn/verilog"} wout]
    puts $wout
    if {$wrc} {
        set strict 1
        if {[info exists ::env(FPGA_M7_PL_WALK_STRUCT_STRICT)] && $::env(FPGA_M7_PL_WALK_STRUCT_STRICT) eq "0"} { set strict 0 }
        if {$strict} { error "FPGA_M7_PL_WALK_STRUCT_ALARM: walk_loop pipelined in place (see check_walk_struct.sh)" }
        puts "FPGA_M7_PL_WALK_STRUCT_ALARM (not strict, build continues)"
    }
}
if {$flow eq "syn"} {
    export_design -flow syn -rtl verilog -format ip_catalog -vendor fpga.lab \
        -library m7 -version 1.0 -description $desc
} else {
    export_design -format ip_catalog -vendor fpga.lab -library m7 -version 1.0 \
        -description $desc
}
set ipdir [file normalize "$build/solution/impl/ip"]
set dest [file normalize "$::env(FPGA_M7_PL_IPREPO)/$top"]
file delete -force $dest
file mkdir [file dirname $dest]
file copy $ipdir $dest
puts "FPGA_M7_PL_IP_EXPORTED $top -> $dest"
if {$::env(FPGA_M7_PL_COSIM) eq "1"} {
    cosim_design -rtl verilog
    puts "FPGA_M7_PL_COSIM_DONE $top"
}
exit
