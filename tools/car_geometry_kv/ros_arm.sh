#!/bin/bash
# One ROS-runtime arm of the geometry-switch board runs, INSIDE the ROS image (fpga-ros:humble-nav-*,
# ros-base lacks nav2_msgs/cv2 that host/ros_runtime.py imports). Started by nano_ros_arm.sh.
#   ros_arm.sh <mode: cpu|kv|kvshadow> <out dir (new)> <prep dir> <duration s> [<kv HOST:PORT> <build id>]
# Processes: stub inference (fixed detections), ros_runtime, replay camera. CPU of each process is
# read from /proc/<pid>/stat (utime+stime) and the machine's /proc/stat line 1 at two instants inside
# the replay (after WARM s, and WARM+WIN s); /proc/stat is not namespaced, so "machine" is the Nano.
# Runtime stops with SIGINT (KeyboardInterrupt -> close -> geometry_close event), stub with SIGTERM.
set -u
MODE=$1; OUT=$2; PREP=$3; DUR=$4; KV=${5:-}; BID=${6:-}
WARM=${WARM:-15}
WIN=$(python3 -c "print(max(10, int($DUR) - $WARM - 5))")
mkdir "$OUT" || { echo "out exists"; exit 2; }
set +u; source /opt/ros/humble/setup.bash; set -u   # setup.bash reads unset vars
cd /checkout
SOCK=/tmp/fpga_stub_$$.sock
python3 -B tools/car_geometry_kv/stub_inference.py --socket "$SOCK" > "$OUT/stub.out" 2> "$OUT/stub.err" &
STUB=$!
for i in $(seq 1 100); do [ -S "$SOCK" ] && break; sleep 0.1; done
[ -S "$SOCK" ] || { echo "stub not ready"; kill -TERM $STUB; exit 3; }
# --camera-qos sensor_data (2026-10-15): this arm says "hardware" but its camera is the best-effort replay stand-in
# below; the runner's auto rule would read hardware as the live driver and subscribe RELIABLE (no match, no frames)
RT_ARGS=(--socket "$SOCK" --pose-source reference --sensor-source hardware --camera-qos sensor_data --inference-device cpu
         --events "$OUT/events.jsonl" --result-wakeup "${RESULT_WAKEUP:-event}")   # event: as docs/design/M5_HANDOFF.md
case "$MODE" in
  cpu) ;;
  kv) RT_ARGS+=(--geometry-source kv --kv-server "$KV" --kv-expect-build-id "$BID") ;;
  kvshadow) RT_ARGS+=(--geometry-source kv --kv-server "$KV" --kv-expect-build-id "$BID" --kv-shadow-check) ;;
  *) echo "bad mode"; kill -TERM $STUB; exit 2 ;;
esac
WATCH=
if [ "${WATCH_STOP:-0}" = 1 ]; then   # records /fpga/motion_permit and /cmd_vel (did the stop messages go out?)
  python3 -B tools/car_geometry_kv/stop_watch.py --out "$OUT/stop_watch.jsonl" > "$OUT/stop_watch.err" 2>&1 &
  WATCH=$!
fi
python3 -B -m host.ros_runtime "${RT_ARGS[@]}" > "$OUT/runtime.out" 2> "$OUT/runtime.err" &
RT=$!
sleep 3
python3 -B tools/car_geometry_kv/replay_camera.py --prep "$PREP" --csv "$OUT/replay.csv" --hz 10 \
    --duration-s "$DUR" > "$OUT/replay.out" 2> "$OUT/replay.err" &
RP=$!
snap() {  # label -> one JSON line with wall time, per-process utime+stime ticks, /proc/stat cpu line
  python3 - "$1" "$RT" "$STUB" "$RP" <<'EOF'
import json, os, sys, time
label, pids = sys.argv[1], dict(zip(("runtime", "stub", "replay"), map(int, sys.argv[2:5])))
def ticks(pid):
    try:
        f = open("/proc/%d/stat" % pid).read().rsplit(")", 1)[1].split()
        return int(f[11]) + int(f[12])          # utime + stime (fields 14, 15)
    except OSError:
        return None
cpu = [int(x) for x in open("/proc/stat").readline().split()[1:]]
print(json.dumps({"label": label, "wall": time.time(), "raw_ns": time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW),
                  "ticks": {k: ticks(p) for k, p in pids.items()}, "proc_stat_cpu": cpu,
                  "hz": os.sysconf("SC_CLK_TCK"), "ncpu": os.cpu_count(), "loadavg": open("/proc/loadavg").read().split()[:3]}))
EOF
}
sleep $((2 + WARM))
snap start >> "$OUT/cpu_snaps.jsonl"
sleep "$WIN"
snap end >> "$OUT/cpu_snaps.jsonl"
wait $RP; echo "replay_rc=$?" >> "$OUT/rc.txt"
sleep 2
python3 -c "import time;print('sigint_raw_ns=%d' % time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW))" >> "$OUT/rc.txt"
kill -INT $RT
for i in $(seq 1 100); do kill -0 $RT 2>/dev/null || break; sleep 0.1; done
kill -0 $RT 2>/dev/null && { echo "runtime still alive after SIGINT, TERM" >> "$OUT/rc.txt"; kill -TERM $RT; }
wait $RT; echo "runtime_rc=$?" >> "$OUT/rc.txt"
if [ -n "$WATCH" ]; then sleep 1; kill -TERM $WATCH; wait $WATCH; echo "watch_rc=$?" >> "$OUT/rc.txt"; fi
kill -TERM $STUB; wait $STUB; echo "stub_rc=$?" >> "$OUT/rc.txt"
echo DONE >> "$OUT/rc.txt"
