#!/bin/bash
# usage: run.sh [names...]  (rec30 car570; dlat2 = 1) -> data/<name>.csv/.flags, logs/<name>.err
R=${FPGA_RUN}
G=$R/GEO_FX0
for n in ${@:-rec30 car570}; do
  ( /usr/bin/time -v $G/build/fx_golden $R/E2/packs/$n.pack $R/E2/golden/$n.dump $R/G1d/data/$n.dlat2.flags $n $G/data 1 > $G/logs/$n.out 2> $G/logs/$n.err; echo "$n rc=$? $(grep '^FX' $G/logs/$n.err)" ) &
done
wait
