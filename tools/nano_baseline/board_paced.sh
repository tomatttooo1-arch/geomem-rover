#!/bin/bash
# Sustained 30 Hz runs of the M7 CPU chain (board-side, detached): whole-board
# CPU / RAM / power at camera rate. Usage:
#   board_paced.sh <repo_dir> <out_dir> <depth_u16.bin> W H N fx fy cx cy unit_m <probe_binary>
set -u
R=$1; OUT=$2; DEPTH=$3; W=$4; H=$5; N=$6; FX=$7; FY=$8; CX=$9; CY=${10}; UNIT=${11}; PROBE=${12}
SEC=${SEC:-30}; HZ=${HZ:-30}
mkdir -p "$OUT/build" "$OUT/csv"
L="$OUT/ledger.jsonl"; PH="$OUT/phases.log"
ph() { echo "$(date +%s.%N) $*" >> "$PH"; }
{ date -Is; nvpmodel -q 2>/dev/null | head -1; uptime; top -bn1 | head -15; cat "$R/COMMIT"; sha256sum "$DEPTH"; } > "$OUT/env.txt" 2>&1
B="$OUT/build"; M="$R/hw/m7"; NB="$R/tools/nano_baseline"
g++ -std=c++11 -O2 -Wall -Wextra -Werror -ffp-contract=off -c "$M/frame_depth.cpp" -o "$B/fd_O2.o" &&
g++ -std=c++11 -O2 -Wall -Wextra -Werror -ffp-contract=off -c "$M/object_evidence.cpp" -o "$B/oe_O2.o" &&
g++ -std=c++14 -O3 -mcpu=native -fopenmp -ffp-contract=off -Wall -Wextra -Werror -I"$M" "$NB/m7_cpu_bench.cpp" "$B/fd_O2.o" "$B/oe_O2.o" -o "$B/bench" \
  > "$OUT/build.log" 2>&1 || { echo BUILD_FAILED >> "$PH"; exit 1; }
COMMON="--depth $DEPTH --width $W --height $H --frames $N --fx $FX --fy $FY --cx $CX --cy $CY --unit-m $UNIT"
PASSES=$(( SEC * HZ / N ))
export LD_LIBRARY_PATH=$(ls -d ~/fpga/orbbec/OrbbecSDK_v2.9.3_*_linux_arm64/lib)
tegrastats --interval 500 --logfile "$OUT/tegrastats.log" & TS=$!
ph "start idle"; sleep 20; ph "end idle"
run() {  # tag, env..., then args
  tag=$1; shift; ph "start $tag"
  env "$@"; ph "end $tag rc=$?"; sleep 4
}
for g in g1 g2; do
  run refO2_${g}_paced OMP_NUM_THREADS=1 taskset -c 3 python3 "$NB/runwrap.py" "$L" refO2_${g}_paced "$B/bench" $COMMON --grid $g --arm ref --passes $PASSES --warmup 0 --pace-hz $HZ --csv "$OUT/csv/refO2_${g}_paced.csv"
  run opt1_${g}_paced OMP_NUM_THREADS=1 taskset -c 3 python3 "$NB/runwrap.py" "$L" opt1_${g}_paced "$B/bench" $COMMON --grid $g --arm opt --passes $PASSES --warmup 0 --pace-hz $HZ --csv "$OUT/csv/opt1_${g}_paced.csv"
  run opt6p_${g}_paced OMP_NUM_THREADS=6 OMP_WAIT_POLICY=passive OMP_PROC_BIND=close OMP_PLACES=cores python3 "$NB/runwrap.py" "$L" opt6p_${g}_paced "$B/bench" $COMMON --grid $g --arm opt --passes $PASSES --warmup 0 --pace-hz $HZ --csv "$OUT/csv/opt6p_${g}_paced.csv"
  run opt6a_${g}_paced OMP_NUM_THREADS=6 OMP_PROC_BIND=close OMP_PLACES=cores python3 "$NB/runwrap.py" "$L" opt6a_${g}_paced "$B/bench" $COMMON --grid $g --arm opt --passes $PASSES --warmup 0 --pace-hz $HZ --csv "$OUT/csv/opt6a_${g}_paced.csv"
done
# camera transport alone, then camera + official arm together (load superposition; the
# chain still consumes the recorded input frames, not the live stream)
run cam_only_paced python3 "$NB/runwrap.py" "$L" cam_only_paced "$PROBE" --depth 640x400@30:Y16 --seconds $SEC
ph "start cam_plus_opt6p_g1_paced"
python3 "$NB/runwrap.py" "$L" cam_plus_opt6p_g1_paced_cam "$PROBE" --depth 640x400@30:Y16 --seconds $((SEC + 4)) & CP=$!
sleep 3
OMP_NUM_THREADS=6 OMP_WAIT_POLICY=passive OMP_PROC_BIND=close OMP_PLACES=cores python3 "$NB/runwrap.py" "$L" cam_plus_opt6p_g1_paced_chain "$B/bench" $COMMON --grid g1 --arm opt --passes $PASSES --warmup 0 --pace-hz $HZ --csv "$OUT/csv/camplus_opt6p_g1_paced.csv"
wait $CP; ph "end cam_plus_opt6p_g1_paced"
sleep 4; kill $TS; ph DONE
