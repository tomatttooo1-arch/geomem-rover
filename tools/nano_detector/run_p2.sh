#!/bin/bash
# P2 measurement matrix on the Orin Nano (board-side, detached). Detector = nano_detector.py on the
# live Gemini 2 colour stream; geometry = the 09-26 M7 CPU chain bench (opt arm, g1 grid) at 30 Hz
# on the recorded TUM depth frames.
# CPU placement (Orin Nano: cluster 0 = cores 0-3, cluster 1 = cores 4-5):
#   detector      : taskset -c 4,5 in every arm (never changes cluster between A and B)
#   geometry  c0  : taskset -c 0-3, 4 OpenMP threads (disjoint from the detector)
#   geometry  u6  : unpinned, 6 OpenMP threads = the 09-26 opt6p_g1 configuration (overlaps the detector)
# Usage: run_p2.sh <code_dir> <engine_dir> <out_dir> <bench_binary> <depth_u16.bin>
set -u
C=$1; E=$2; OUT=$3; BENCH=$4; DEPTH=$5
DETC=${DETC:-4,5}
mkdir -p "$OUT/csv" "$OUT/det"
L="$OUT/ledger.jsonl"; PH="$OUT/phases.log"
ph() { echo "$(date +%s.%N) $*" >> "$PH"; }
export PYTHONPATH=$HOME/fpga/p2/pylib
{ date -Is; nvpmodel -q 2>/dev/null | head -1; uptime; top -bn1 | head -15; sha256sum "$BENCH" "$DEPTH" "$E"/*.engine "$C"/*.py; } > "$OUT/env.txt" 2>&1
GEO="--depth $DEPTH --width 640 --height 480 --frames 5 --fx 525 --fy 525 --cx 319.5 --cy 239.5 --unit-m 0.0002 --grid g1 --arm opt --warmup 0 --pace-hz 30"
geo() {  # tag placement seconds
  local tag=$1 pl=$2 sec=$3; local passes=$(( sec * 30 / 5 ))
  if [ "$pl" = c0 ]; then
    env OMP_NUM_THREADS=4 OMP_WAIT_POLICY=passive OMP_PROC_BIND=close OMP_PLACES=cores taskset -c 0-3 \
      python3 "$C/runwrap.py" "$L" "$tag" "$BENCH" $GEO --passes $passes --csv "$OUT/csv/$tag.csv"
  else
    env OMP_NUM_THREADS=6 OMP_WAIT_POLICY=passive OMP_PROC_BIND=close OMP_PLACES=cores \
      python3 "$C/runwrap.py" "$L" "$tag" "$BENCH" $GEO --passes $passes --csv "$OUT/csv/$tag.csv"
  fi
}
det() {  # tag size mode seconds  (the detector itself writes "start/end tag" around its measured window)
  local tag=$1 sz=$2 mode=$3 sec=$4
  taskset -c $DETC python3 "$C/runwrap.py" "$L" "$tag" python3 "$C/nano_detector.py" --engine "$E/yolov8n-seg-$sz-fp16.engine" \
    --mode $mode --hz 15 --seconds $sec --warmup-s 5 --out "$OUT/det/$tag.jsonl" --summary "$OUT/det/$tag.summary.json" \
    --phase-file "$PH" --tag "$tag" ${DET_EXTRA:-}
}
solo_geo() { ph "start $1"; geo "$@"; ph "end $1 rc=$?"; sleep 4; }
combo() {  # tag size mode geo_placement
  local tag=$1 sz=$2 mode=$3 pl=$4
  det ${tag}_det $sz $mode 70 & local DP=$!
  until grep -q "start ${tag}_det" "$PH" 2>/dev/null; do sleep 0.2; done
  sleep 2; ph "start ${tag}"
  geo ${tag}_geo $pl 60; ph "end ${tag} rc=$?"
  wait $DP; sleep 4
}
tegrastats --interval 500 --logfile "$OUT/tegrastats.log" & TS=$!
( G=/sys/class/devfreq/17000000.gpu/cur_freq; while :; do echo "$(date +%s.%N) $(cat $G)"; sleep 0.5; done ) > "$OUT/gpufreq.log" 2>&1 & GF=$!
ph "start idle"; sleep 20; ph "end idle"
solo_geo geo_u6 u6 30
solo_geo geo_c0 c0 30
det det640_p15 640 paced 60; sleep 4
det det640_best 640 best 30; sleep 4
DET_EXTRA="--cam-fps 15" det det640_p15_cam15 640 paced 60; sleep 4
combo mix640_p15_c0 640 paced c0
combo mix640_best_c0 640 best c0
combo mix640_p15_u6 640 paced u6
det det320_p15 320 paced 60; sleep 4
det det320_best 320 best 30; sleep 4
solo_geo geo_u6_end u6 30
kill $TS $GF; ph DONE
