#!/bin/bash
# P8 same-input geometry comparison on the Orin Nano (board-side, run detached).
#   run_board.sh <repo_dir> <prep_dir> <out_dir>
# Arms (each: WARMUP pass + PASSES passes x all prepared frames, separate process):
#   A   repo CPU reference FrameNativeM7 via M7BackendSelector("cpu")    core 3
#   B1  optimized C++ (tools/nano_baseline/m7_cpu_bench.cpp --arm opt)   core 3,   1 thread
#   B4  same, OMP 4 threads, passive wait                                cores 0-3 (cluster 0)
#   B6  same, OMP 6 threads, passive wait                                cores 0-5 (both clusters)
#   C   PL backend with MockPlDevice (flow only, NOT a performance arm)  core 3
# Two modes: p15 = paced 15 Hz (the loop's schedule; CPU/power at camera rate),
#            free = back-to-back (latency without idle gaps).
# Then compare.py: every arm's first measured pass vs A, voxel by voxel.
# Writes <out>/DONE or <out>/FAILED.
set -u
R=$1; PREP=$2; OUT=$3
PASSES=${PASSES:-3}; WARMUP=${WARMUP:-1}; HZ=${HZ:-15}
mkdir -p "$OUT/build" "$OUT/runs"
PH="$OUT/phases.log"
ph() { echo "$(date +%s.%N) $*" >> "$PH"; }
fail() { ph "FAILED $*"; echo "$*" > "$OUT/FAILED"; kill ${TS:-0} 2>/dev/null; exit 1; }
tis() { for p in 0 4; do echo "policy$p"; cat /sys/devices/system/cpu/cpufreq/policy$p/stats/time_in_state; done > "$OUT/runs/$1.tis_$2"; }
{ date -Is; nvpmodel -q 2>/dev/null; uptime; cat /proc/loadavg
  for p in 0 4; do d=/sys/devices/system/cpu/cpufreq/policy$p; echo "policy$p $(cat $d/scaling_governor) min=$(cat $d/scaling_min_freq) max=$(cat $d/scaling_max_freq) cur=$(cat $d/scaling_cur_freq) rate_limit_us=$(cat $d/../schedutil/rate_limit_us 2>/dev/null)"; done
  for c in 0 1 2 3 4 5; do echo "cpu$c cluster=$(cat /sys/devices/system/cpu/cpu$c/topology/cluster_id 2>/dev/null)"; done
  cat /sys/kernel/mm/transparent_hugepage/enabled; grep -E "MemFree|MemAvailable|^Cached|AnonHuge" /proc/meminfo
  g++ --version | head -1; python3 -c 'import numpy,sys;print("numpy",numpy.__version__,"python",sys.version.split()[0])'
  cat "$R/COMMIT"; sha256sum "$PREP/depth_u16.bin" "$PREP/meta.json"; top -bn1 | head -15; } > "$OUT/env.txt" 2>&1
B="$OUT/build"; M="$R/hw/m7"
# same flags as tools/nano_baseline/board_paced.sh (09-26 opt6p binary recipe)
{ g++ -std=c++11 -O2 -Wall -Wextra -Werror -ffp-contract=off -c "$M/frame_depth.cpp" -o "$B/fd_O2.o" &&
  g++ -std=c++11 -O2 -Wall -Wextra -Werror -ffp-contract=off -c "$M/object_evidence.cpp" -o "$B/oe_O2.o" &&
  g++ -std=c++14 -O3 -mcpu=native -fopenmp -ffp-contract=off -Wall -Wextra -Werror -I"$M" "$R/tools/nano_baseline/m7_cpu_bench.cpp" "$B/fd_O2.o" "$B/oe_O2.o" -o "$B/bench"; } \
  > "$OUT/build.log" 2>&1 || fail build
read -r W H N IK UNIT < <(python3 -c "
import json; m=json.load(open('$PREP/meta.json'))
K=m['K']; print(m['width'], m['height'], m['frames'], ','.join(m['inverse_k_f32_hex']), repr(float(m['depth_unit_m'])))")
[ -n "${UNIT:-}" ] || fail "meta.json has no depth_unit_m"
FX_ARGS=$(python3 -c "
import json; K=json.load(open('$PREP/meta.json'))['K']; print('--fx %r --fy %r --cx %r --cy %r' % (K[0][0], K[1][1], K[0][2], K[1][2]))")
BCOMMON="--depth $PREP/depth_u16.bin --width $W --height $H --frames $N $FX_ARGS --unit-m $UNIT --grid p32 --ik $IK --no-object --arm opt --passes $PASSES --warmup $WARMUP"
export PYTHONPATH="$R"
tegrastats --interval 500 --logfile "$OUT/tegrastats.log" & TS=$!
ph "start idle"; sleep 20; ph "end idle"
run() {  # tag, then command
  tag=$1; shift; tis "$tag" start; ph "start $tag"
  "$@" > "$OUT/runs/$tag.stdout" 2> "$OUT/runs/$tag.stderr"; rc=$?
  ph "end $tag rc=$rc"; tis "$tag" end; sleep 4
  return $rc
}
for mode in p15 free; do
  if [ $mode = p15 ]; then PACE=$HZ; else PACE=0; fi
  run A_$mode taskset -c 3 python3 "$R/tools/geo_same_input/py_arm.py" --prep "$PREP" --arm a --out "$OUT/runs/A_$mode" \
      --passes $PASSES --warmup $WARMUP --pace-hz $PACE || fail A_$mode
  run B1_$mode env OMP_NUM_THREADS=1 taskset -c 3 "$B/bench" $BCOMMON --pace-hz $PACE --csv "$OUT/runs/B1_$mode.csv" --dump "$OUT/runs/B1_$mode.dump" || { rc=$?; [ $rc -eq 7 ] || fail B1_$mode; }  # 7 = differs from in-binary reference: keep going, compare.py reports it
  run B4_$mode env OMP_NUM_THREADS=4 OMP_WAIT_POLICY=passive OMP_PROC_BIND=close OMP_PLACES=cores taskset -c 0-3 "$B/bench" $BCOMMON --pace-hz $PACE --csv "$OUT/runs/B4_$mode.csv" --dump "$OUT/runs/B4_$mode.dump" || { rc=$?; [ $rc -eq 7 ] || fail B4_$mode; }  # 7 = differs from in-binary reference: keep going, compare.py reports it
  run B6_$mode env OMP_NUM_THREADS=6 OMP_WAIT_POLICY=passive OMP_PROC_BIND=close OMP_PLACES=cores taskset -c 0-5 "$B/bench" $BCOMMON --pace-hz $PACE --csv "$OUT/runs/B6_$mode.csv" --dump "$OUT/runs/B6_$mode.dump" || { rc=$?; [ $rc -eq 7 ] || fail B6_$mode; }  # 7 = differs from in-binary reference: keep going, compare.py reports it
  run C_$mode taskset -c 3 python3 "$R/tools/geo_same_input/py_arm.py" --prep "$PREP" --arm c --device mock --out "$OUT/runs/C_$mode" \
      --passes $PASSES --warmup $WARMUP --pace-hz $PACE || fail C_$mode
done
kill $TS
CANDS=""
for t in A_free B1_p15 B1_free B4_p15 B4_free B6_p15 B6_free C_p15 C_free; do
  case $t in A_*|C_*) f="$OUT/runs/$t/flags.dump";; *) f="$OUT/runs/$t.dump";; esac
  CANDS="$CANDS --cand $t=$f"
done
python3 "$R/tools/geo_same_input/compare.py" --ref "$OUT/runs/A_p15/flags.dump" $CANDS --out "$OUT/compare.json" > "$OUT/compare.stdout" 2>&1
echo "compare rc=$?" >> "$OUT/compare.stdout"
ph DONE; touch "$OUT/DONE"
