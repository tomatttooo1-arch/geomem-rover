# Out-of-context synthesis of the permit and encoder-UART blocks, for a
# resource/timing ESTIMATE only (no placement, no bitstream), 2026-10-11.
#   vivado -mode batch -source ooc_synth_permit_uart.tcl -tclargs <out_dir>
# Numbers are post-synthesis OOC estimates at 100 MHz, not post-route.
set out [file normalize [lindex $argv 0]]
file mkdir $out
set rtl [file normalize [file dirname [info script]]/../rtl]
set part xck26-sfvc784-2LV-c
foreach {top params} {
    m7_permit_axil   {TIMEOUT_CYCLES 100000000 TIMEOUT_MIN_CYCLES 10000000 TIMEOUT_MAX_CYCLES 500000000 HB_HALF_CYCLES 500000}
    m7_enc_uart_axil {CLK_HZ 100000000 BAUD 115200}
} {
    create_project -in_memory -part $part
    read_verilog [list $rtl/m7_timebase.v $rtl/m7_permit_axil.v $rtl/m7_enc_uart_axil.v]
    set gen {}
    foreach {k v} $params { lappend gen -generic $k=$v }
    synth_design -top $top -part $part -mode out_of_context {*}$gen
    create_clock -period 10.000 -name clk [get_ports s_axi_aclk]
    report_utilization -file $out/util_$top.rpt
    report_timing_summary -max_paths 5 -file $out/timing_$top.rpt
    set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
    puts "OOC_RESULT $top wns_ns=$wns"
    close_project
}
