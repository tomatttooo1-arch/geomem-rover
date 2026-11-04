#!/bin/bash
# P8 side check: is today's 12 % slowdown of the 09-26 geometry baseline (NANO_DETECTOR_20260927 §4) reproducible,
# and what do CPU frequency / governor / huge pages look like while it runs? Board-side, detached.
#   tum_recheck.sh <out_dir> [bench_binary] [tum_depth]
# Same binary and input as 09-26 paced_tum2 opt6p_g1_paced (31d7b60 build) and, for the back-to-back mode,
# the 09-26 bench_tum_run1 opt6 settings (6 threads, active wait, 20 passes x 5 frames, 2 warm-up passes).
set -u
OUT=$1; BIN=${2:-$HOME/fpga/nano0926/bench/tum_paced2/build/bench}; DEPTH=${3:-$HOME/fpga/nano0926/tum5/depth_u16.bin}
mkdir -p "$OUT/csv"; PH="$OUT/phases.log"
ph() { echo "$(date +%s.%N) $*" >> "$PH"; }
tis() { for p in 0 4; do echo "policy$p"; cat /sys/devices/system/cpu/cpufreq/policy$p/stats/time_in_state; done > "$OUT/$1.tis_$2"; }
{ date -Is; nvpmodel -q 2>/dev/null; uptime
  for p in 0 4; do d=/sys/devices/system/cpu/cpufreq/policy$p; echo "policy$p $(cat $d/scaling_governor) min=$(cat $d/scaling_min_freq) max=$(cat $d/scaling_max_freq) rate_limit_us=$(cat $d/../schedutil/rate_limit_us 2>/dev/null)"; done
  cat /sys/kernel/mm/transparent_hugepage/enabled /sys/kernel/mm/transparent_hugepage/defrag; cat /proc/buddyinfo
  grep -E "MemFree|MemAvailable|^Cached|AnonHuge" /proc/meminfo; grep -E "thp_fault" /proc/vmstat
  sha256sum "$BIN" "$DEPTH"; top -bn1 | head -15; } > "$OUT/env.txt" 2>&1
COMMON="--depth $DEPTH --width 640 --height 480 --frames 5 --fx 525 --fy 525 --cx 319.5 --cy 239.5 --unit-m 0.0002 --grid g1 --arm opt"
tegrastats --interval 500 --logfile "$OUT/tegrastats.log" & TS=$!
ph "start idle"; sleep 20; ph "end idle"
run() {  # tag, then command; samples the process' huge pages 3 s in
  tag=$1; shift; tis "$tag" start; grep thp_fault /proc/vmstat > "$OUT/$tag.vmstat_start"; ph "start $tag"
  "$@" > "$OUT/$tag.stdout" 2> "$OUT/$tag.stderr" & P=$!
  sleep 3; grep -E "AnonHugePages|Rss" /proc/$P/smaps_rollup > "$OUT/$tag.smaps" 2>&1
  wait $P; rc=$?; ph "end $tag rc=$rc"; tis "$tag" end; grep thp_fault /proc/vmstat > "$OUT/$tag.vmstat_end"; sleep 4
}
P6="env OMP_NUM_THREADS=6 OMP_WAIT_POLICY=passive OMP_PROC_BIND=close OMP_PLACES=cores"
A6="env OMP_NUM_THREADS=6 OMP_PROC_BIND=close OMP_PLACES=cores"
run opt6p_g1_paced_a $P6 "$BIN" $COMMON --passes 180 --warmup 0 --pace-hz 30 --csv "$OUT/csv/opt6p_g1_paced_a.csv"
for r in 1 2 3; do run opt6_g1_free_r$r $A6 "$BIN" $COMMON --passes 20 --warmup 2 --csv "$OUT/csv/opt6_g1_free_r$r.csv"; done
run opt6p_g1_paced_b $P6 "$BIN" $COMMON --passes 180 --warmup 0 --pace-hz 30 --csv "$OUT/csv/opt6p_g1_paced_b.csv"
kill $TS; ph DONE; touch "$OUT/DONE"
