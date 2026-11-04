#!/bin/bash
# Whole-chain board smoke, run INSIDE one ROS 2 Humble container from the repo root (one container so
# Fast DDS shared memory reaches every node: hub, producer, bridge, costmap).
#   hub (host.hub.hub_ros, KV TELEM/ESTOP/SYNC) + PC receiver (pc/fpgapc.py listen over 127.0.0.1)
#   + chain_watch.py (KV permit watcher, one PC e-stop through the hub)
#   + tools/car_geo_producer/rig_inner.sh (replay camera 10 Hz -> producer -> KV -> RollingFusion -> bridge -> costmap)
# env: OUT (new dir) PREP KV EXPECT_BUILD_ID TOKEN (file) HUBPORT DUR HZ ESTOP_AT (s after watcher start)
# Never pkill -f: every child is stopped by its own pid.
cd "$(dirname "$0")/../.."
source /opt/ros/humble/setup.bash
set -u
export PYTHONPATH=$PWD:${PYTHONPATH:-} PYTHONUNBUFFERED=1
: "${OUT:?}" "${PREP:?}" "${KV:?}" "${TOKEN:?}" "${HUBPORT:=47431}" "${DUR:=180}" "${HZ:=10}" "${ESTOP_AT:=110}" "${WARM:=20}"
mkdir "$OUT" || exit 2
ticks() { awk '{printf "%.0f\n", $14+$15}' /proc/$1/stat 2>/dev/null || echo NA; }
# total watcher/listener time: rig WARM + DUR + producer tail (20 s) + 25 s for the revoke/close
TOTAL=$((WARM + DUR + 45))
# hub_ros exits by itself after --duration-s: under rclpy, SIGINT/SIGTERM only shut the rclpy
# context down (rclpy's own handlers) and run()'s loop keeps going (host smoke 10-11).
python3 -m host.hub.hub_ros --data-source replay --token-file "$TOKEN" --port "$HUBPORT" --bind 127.0.0.1 \
  --kv "$KV" --duration-s $((TOTAL + 15)) --stats-out "$OUT/hub_stats.json" > "$OUT/hub.out" 2> "$OUT/hub.err" &
HUB=$!
for i in $(seq 1 100); do grep -q 'hub listening' "$OUT/hub.out" 2>/dev/null && break; sleep 0.2; done
grep -q 'hub listening' "$OUT/hub.out" || { echo "hub not up"; kill -KILL $HUB; exit 3; }
# e-stop PC first (it becomes the control holder), then the listening PC
python3 tools/integ_car_chain/chain_watch.py --server "$KV" --hub-port "$HUBPORT" --token-file "$TOKEN" \
  --seconds "$TOTAL" --estop-at-s "$ESTOP_AT" --out "$OUT/watch" > "$OUT/watch.out" 2> "$OUT/watch.err" &
WATCH=$!
for i in $(seq 1 100); do [ -e "$OUT/watch/pc_connected" ] && break; sleep 0.2; done
python3 pc/fpgapc.py --host 127.0.0.1 --port "$HUBPORT" --token-file "$TOKEN" listen --duration-s "$TOTAL" \
  --out "$OUT/pc.fpgarec" --stats "$OUT/pc_stats.json" > "$OUT/pc_listen.out" 2> "$OUT/pc_listen.err" &
PC=$!
echo "{\"hub\": $HUB, \"pc\": $PC, \"watch\": $WATCH}" > "$OUT/pids.json"
T0=$(date +%s.%N); H0=$(ticks $HUB); P0=$(ticks $PC)
OUT="$OUT/rig" PREP="$PREP" KV="$KV" EXPECT_BUILD_ID="${EXPECT_BUILD_ID:-}" BACKEND=remote_pl DUR="$DUR" HZ="$HZ" \
  COSTMAP=1 WARM="$WARM" bash tools/car_geo_producer/rig_inner.sh > "$OUT/rig.log" 2>&1
echo "rig_rc=$?" > "$OUT/rc.txt"
T1=$(date +%s.%N); H1=$(ticks $HUB); P1=$(ticks $PC)
python3 -c "
import json,sys,os
t0,t1,h0,h1,p0,p1=sys.argv[1:7]; tck=os.sysconf('SC_CLK_TCK'); w=float(t1)-float(t0)
pc=lambda a,b: None if 'NA' in (a,b) else round((int(b)-int(a))/tck/w*100,2)
print(json.dumps({'window_s':round(w,2),'hub_pct_one_core':pc(h0,h1),'pc_listen_pct_one_core':pc(p0,p1)}))
" "$T0" "$T1" "$H0" "$H1" "$P0" "$P1" > "$OUT/cpu_hub_pc.json"
wait $WATCH; echo "watch_rc=$?" >> "$OUT/rc.txt"
for i in $(seq 1 150); do kill -0 $PC 2>/dev/null || break; sleep 0.2; done
kill -0 $PC 2>/dev/null && kill -INT $PC
wait $PC; echo "pc_rc=$?" >> "$OUT/rc.txt"
for i in $(seq 1 300); do kill -0 $HUB 2>/dev/null || break; sleep 0.2; done
if kill -0 $HUB 2>/dev/null; then
  echo "hub alive 60 s after its duration window, INT" >> "$OUT/rc.txt"; kill -INT $HUB; sleep 5
  kill -0 $HUB 2>/dev/null && { echo "hub alive after INT, KILL" >> "$OUT/rc.txt"; kill -KILL $HUB; }
fi
wait $HUB; echo "hub_rc=$?" >> "$OUT/rc.txt"
python3 pc/fpgapc.py verify --file "$OUT/pc.fpgarec" > "$OUT/pc_verify.json" 2>&1
echo DONE >> "$OUT/rc.txt"
