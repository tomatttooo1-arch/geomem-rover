#!/usr/bin/env bash
# usage: extract.sh <build dir>  -> key numbers from the routed reports
B=$1
echo "== SUMMARY"; cat $B/SUMMARY.txt
echo "== util"; grep -E "^\| (CLB LUTs|CLB Registers|Block RAM Tile|URAM|DSPs|Register as Latch)\b" $B/utilization_route.rpt
echo "== hier"; grep -E "^\|  +(frame|roi|rolling|dpu|clk_wiz_dpu|sc_ctrl|sc_mem[0-9]|sc_dpu_instr|permit|sysinfo|ps|rst_[a-z0-9]+|timebase) " $B/utilization_route_hier.rpt | awk -F'|' '{print $2,$3,$4,$8,$9,$10,$11,$12,$13}'
echo "== intra"; grep -A14 "Intra Clock Table" $B/timing_route.rpt | grep -E "^clk|^ *clk"
echo "== inter"; grep -A20 "Inter Clock Table" $B/timing_route.rpt | grep -E "clk" | head -12
echo "== cdc"; grep -E "^(CDC|Critical|Warning|Info)" $B/cdc.rpt | head; grep -E "^\| CDC-[0-9]+ " $B/cdc.rpt | head
echo "== cong"; grep -iE "level|congestion" $B/congestion.rpt | head -20
echo "== log"; grep -cE "^CRITICAL WARNING" $B.log; grep -E "^CRITICAL WARNING" $B.log | cut -c1-160 | sort | uniq -c | head
