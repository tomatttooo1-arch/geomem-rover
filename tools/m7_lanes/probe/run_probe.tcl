# csynth-only resource/II probe for the A2 lanes structure (no export, no board).
# env: PROBE_LANES (1/2/4/8/16), PROBE_SRC (lanes_probe.cpp), PROBE_CLOCK_NS (default 10),
#      PROBE_EXPORT_SYN=1 to also run Vivado OOC synthesis (export_design -flow syn)
set n $::env(PROBE_LANES)
set src $::env(PROBE_SRC)
set clk 10
if {[info exists ::env(PROBE_CLOCK_NS)]} { set clk $::env(PROBE_CLOCK_NS) }
open_project -reset lanes_probe_N$n
set_top lanes_probe
set extra ""
if {[info exists ::env(PROBE_CFLAGS)]} { set extra $::env(PROBE_CFLAGS) }
add_files $src -cflags "-DPROBE_LANES=$n $extra"
open_solution -reset sol -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period $clk
csynth_design
# optional: Vivado out-of-context synthesis for post-synthesis LUT/FF/BRAM
if {[info exists ::env(PROBE_EXPORT_SYN)] && $::env(PROBE_EXPORT_SYN) == 1} {
  export_design -flow syn -rtl verilog
}
exit
