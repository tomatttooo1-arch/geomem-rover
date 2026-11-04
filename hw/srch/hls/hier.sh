#!/bin/bash
L=${FPGA_LOCKS}; slot=""
while [ -z "$slot" ]; do for i in 1 2 3 5; do if mkdir $L/nat_build_slot_$i 2>/dev/null; then slot=$L/nat_build_slot_$i; echo "dev1 SRCHK5 hier-report pid $$ $(date -Iseconds)" > $slot/owner; echo "$(date -Iseconds) TAKE nat_build_slot_$i dev1 SRCHK5 hier pid $$" >> $L/HISTORY.log; break; fi; done; [ -z "$slot" ] && sleep 20; done
trap 'rm -f $slot/owner; rmdir $slot; echo "$(date -Iseconds) RELEASE $(basename $slot) dev1 SRCHK5 hier pid $$" >> $L/HISTORY.log' EXIT
source /data1/Xilinx/Vivado/2022.2/settings64.sh
cd $(dirname $3) && vivado -mode batch -nojournal -nolog -source $(dirname $0)/hier.tcl -tclargs $1 $2 $3 > $3.vivado.log 2>&1
