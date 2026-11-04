#!/bin/bash
# M7 CPU baseline on the Orin Nano. Board-side, detached. Usage:
#   board_bench.sh <repo_dir> <out_dir> <depth_u16.bin> W H N fx fy cx cy unit_m
# Arms x grids x 3 runs (separate processes); per-frame CSV rows per run.
set -u
R=$1; OUT=$2; DEPTH=$3; W=$4; H=$5; N=$6; FX=$7; FY=$8; CX=$9; CY=${10}; UNIT=${11}
RUNS=${RUNS:-3}
mkdir -p "$OUT/build" "$OUT/csv"
L="$OUT/ledger.jsonl"; PH="$OUT/phases.log"
ph() { echo "$(date +%s.%N) $*" >> "$PH"; }
{ date -Is; nvpmodel -q 2>/dev/null | head -1; cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor;
  for c in 0 1 2 3 4 5; do echo "cpu$c $(cat /sys/devices/system/cpu/cpu$c/cpufreq/scaling_min_freq) $(cat /sys/devices/system/cpu/cpu$c/cpufreq/scaling_max_freq) $(cat /sys/devices/system/cpu/cpu$c/cpufreq/scaling_cur_freq)"; done
  g++ --version | head -1; python3 -c 'import numpy,sys;print("numpy",numpy.__version__,"python",sys.version.split()[0])';
  uptime; top -bn1 | head -15; cat "$R/COMMIT"; sha256sum "$DEPTH"; } > "$OUT/env.txt" 2>&1
B="$OUT/build"; M="$R/hw/m7"; NB="$R/tools/nano_baseline"
# reference kernels with the repo's own flags (host/m7_frame_native.py uses -O2 -ffp-contract=off)
g++ -std=c++11 -O2 -Wall -Wextra -Werror -ffp-contract=off -c "$M/frame_depth.cpp" -o "$B/fd_O2.o" &&
g++ -std=c++11 -O2 -Wall -Wextra -Werror -ffp-contract=off -c "$M/object_evidence.cpp" -o "$B/oe_O2.o" &&
g++ -std=c++11 -O3 -mcpu=native -Wall -Wextra -Werror -ffp-contract=off -c "$M/frame_depth.cpp" -o "$B/fd_O3.o" &&
g++ -std=c++11 -O3 -mcpu=native -Wall -Wextra -Werror -ffp-contract=off -c "$M/object_evidence.cpp" -o "$B/oe_O3.o" &&
g++ -std=c++14 -O3 -mcpu=native -fopenmp -ffp-contract=off -Wall -Wextra -I"$M" "$NB/m7_cpu_bench.cpp" "$B/fd_O2.o" "$B/oe_O2.o" -o "$B/bench_refO2" &&
g++ -std=c++14 -O3 -mcpu=native -fopenmp -ffp-contract=off -Wall -Wextra -I"$M" "$NB/m7_cpu_bench.cpp" "$B/fd_O3.o" "$B/oe_O3.o" -o "$B/bench_refO3" \
  > "$OUT/build.log" 2>&1 || { echo BUILD_FAILED >> "$PH"; exit 1; }
COMMON="--depth $DEPTH --width $W --height $H --frames $N --fx $FX --fy $FY --cx $CX --cy $CY --unit-m $UNIT"
tegrastats --interval 500 --logfile "$OUT/tegrastats.log" & TS=$!
sleep 5; ph idle_end
for r in $(seq 1 "$RUNS"); do
 for g in g1 g2; do
  for arm in refO2 refO3 opt1 opt6 opt4 numpy pipeline; do
    tag="${arm}_${g}_r$r"; ph "start $tag"
    case $arm in
      refO2) OMP_NUM_THREADS=1 taskset -c 3 python3 "$NB/runwrap.py" "$L" "$tag" "$B/bench_refO2" $COMMON --grid $g --arm ref --passes 20 --warmup 2 --csv "$OUT/csv/$tag.csv";;
      refO3) OMP_NUM_THREADS=1 taskset -c 3 python3 "$NB/runwrap.py" "$L" "$tag" "$B/bench_refO3" $COMMON --grid $g --arm ref --passes 20 --warmup 2 --csv "$OUT/csv/$tag.csv";;
      opt1)  OMP_NUM_THREADS=1 taskset -c 3 python3 "$NB/runwrap.py" "$L" "$tag" "$B/bench_refO2" $COMMON --grid $g --arm opt --passes 20 --warmup 2 --csv "$OUT/csv/$tag.csv";;
      opt4)  OMP_NUM_THREADS=4 OMP_PROC_BIND=close OMP_PLACES=cores taskset -c 2-5 python3 "$NB/runwrap.py" "$L" "$tag" "$B/bench_refO2" $COMMON --grid $g --arm opt --passes 20 --warmup 2 --csv "$OUT/csv/$tag.csv";;
      opt6)  OMP_NUM_THREADS=6 OMP_PROC_BIND=close OMP_PLACES=cores python3 "$NB/runwrap.py" "$L" "$tag" "$B/bench_refO2" $COMMON --grid $g --arm opt --passes 20 --warmup 2 --csv "$OUT/csv/$tag.csv";;
      numpy) OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 taskset -c 3 python3 "$NB/runwrap.py" "$L" "$tag" python3 "$NB/m7_numpy_bench.py" --repo "$R" $COMMON --grid $g --arm numpy --passes 4 --warmup 1 --csv "$OUT/csv/$tag.csv" --build "$B/np_$tag";;
      pipeline) OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 taskset -c 3 python3 "$NB/runwrap.py" "$L" "$tag" python3 "$NB/m7_numpy_bench.py" --repo "$R" $COMMON --grid $g --arm pipeline --passes 10 --warmup 1 --csv "$OUT/csv/$tag.csv" --build "$B/pl_$tag";;
    esac
    ph "end $tag rc=$?"; sleep 2
  done
 done
done
# the literal step-loop numpy transcription, once, for disclosure only
tag=numpy_step_g1_r1; ph "start $tag"
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 taskset -c 3 python3 "$NB/runwrap.py" "$L" "$tag" python3 "$NB/m7_numpy_bench.py" --repo "$R" $COMMON --grid g1 --arm numpy_step --passes 1 --warmup 0 --csv "$OUT/csv/$tag.csv" --build "$B/nps_$tag"
ph "end $tag rc=$?"
sleep 5; kill $TS; ph DONE
