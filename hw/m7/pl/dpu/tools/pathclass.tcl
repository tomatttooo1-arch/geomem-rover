# Classify the 200 worst setup paths per clock: logic levels, route share, crossing, fanout.
# usage: vivado -mode batch -source pathclass.tcl -tclargs <routed.dcp> <out.csv>
lassign $argv dcp csv
open_checkpoint $dcp
set f [open $csv w]
puts $f "group,slack_ns,levels,datapath_ns,logic_ns,route_ns,route_pct,src_clk,dst_clk,max_fanout,src_cell,dst_cell"
foreach g [get_path_groups] {
  foreach p [get_timing_paths -setup -group $g -max_paths 200 -nworst 1 -slack_lesser_than 0] {
    set dp [get_property DATAPATH_DELAY $p]
    set lg [get_property DATAPATH_LOGIC_DELAY $p]
    set rt [get_property DATAPATH_NET_DELAY $p]
    set pct [expr {$dp > 0 ? 100.0*$rt/$dp : 0}]
    set mf 0
    foreach n [get_nets -quiet -of_objects $p] { set fo [get_property FLAT_PIN_COUNT $n]; if {$fo > $mf} {set mf $fo} }
    set sp [get_property STARTPOINT_PIN $p]; set ep [get_property ENDPOINT_PIN $p]
    set sc [get_cells -quiet -of_objects $sp]; set ec [get_cells -quiet -of_objects $ep]
    puts $f [join [list $g [get_property SLACK $p] [get_property LOGIC_LEVELS $p] $dp $lg $rt [format %.1f $pct] \
      [get_property STARTPOINT_CLOCK $p] [get_property ENDPOINT_CLOCK $p] $mf [string range $sc 0 90] [string range $ec 0 90]] ,]
  }
}
close $f
puts "PATHCLASS done"
