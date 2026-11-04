#!/bin/bash
# Nano side of one B1 board run, detached: environment record + nano_replay.py pinned to CORES.
#   nano_board.sh <repo> <out> <nano_replay args...>     (no sudo; does not change power mode or network)
# Writes <out>/env.txt, <out>/replay.out|err, then <out>/DONE (rc in it) when finished.
set -u
repo=$1; out=$2; shift 2
CORES=${CORES:-4,5}
mkdir -p "$(dirname "$out")"
mkdir -p "$out.meta"
{ date -Is; nvpmodel -q 2>&1; uptime; cat /proc/loadavg
  for p in 0 4; do d=/sys/devices/system/cpu/cpufreq/policy$p; echo "policy$p $(cat $d/scaling_governor) cur=$(cat $d/scaling_cur_freq) max=$(cat $d/scaling_max_freq)"; done
  cat /sys/class/net/enP8p1s0/speed 2>&1; ip -br addr show enP8p1s0 2>&1
  python3 -c 'import numpy,sys;print("numpy",numpy.__version__,"python",sys.version.split()[0])'
  cat "$repo/COMMIT" 2>/dev/null; echo "cores $CORES"; top -bn1 | head -15; } > "$out.meta/env.txt" 2>&1
export REPO="$repo" OUT="$out" CORES
setsid nohup bash -c 'env OPENBLAS_NUM_THREADS=1 PYTHONPATH="$REPO" taskset -c "$CORES" python3 "$REPO/tools/kv_link/nano_replay.py" --out "$OUT" "$@" > "$OUT.meta/replay.out" 2> "$OUT.meta/replay.err"; echo $? > "$OUT.meta/DONE"' _ "$@" < /dev/null > /dev/null 2>&1 &
echo "started $!"
