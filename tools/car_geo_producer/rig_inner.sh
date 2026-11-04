#!/bin/bash
# Geometry producer rig, run INSIDE a ROS 2 Humble container from the repo root.
#   replay camera (prepared depth, synthetic car) -> host/ros_geo_producer.py (KV260 or fake KV)
#   -> RollingFusion -> host/ros_rolling_nav.py bridge -> [Nav2 standalone costmap] ; probe records
# env: OUT (new dir) PREP KV (HOST:PORT | fake) DUR (s) HZ MEMBOXES COSTMAP (1: start costmap
#      here, 0: another container does it, none: skip) EXPECT_BUILD_ID (optional)
#      ROS_DOMAIN_ID (set by caller) WARM (s before the CPU window, default 20)
#      BACKEND (remote_pl default | cpu: the producer computes the grid itself, KV unused)
#      REPLAY_EXTRA / PRODUCER_EXTRA (optional, word-split extra arguments, e.g. "--rgb",
#      "--tf-listener-thread"); PROBE=1: tools/integ_car_chain/tf_probe.py (TF arrivals + system
#      sampler, PROBE_PIDS "--pid name=pid ..." for processes started by the caller; also the
#      producer's per-thread states threads.jsonl and the ROS graph changes graph.jsonl)
#      PRODUCER_WRAP (optional, host diagnosis only): command prefix for the producer, e.g.
#      "py-spy record --native --idle --threads --format raw -o F --" (then PROD is the wrapper's pid)
#      CHURN_EXTRA (optional, e.g. "--freeze-at-s 1 --freeze-s 5": freeze experiment)
#      CHURN="PERIOD:LIFE" (optional): tools/integ_car_chain/participant_churn.py during the
#      measurement window (a new read-only ROS participant every PERIOD s, alive LIFE s)
#      PROBE_JOIN (v3, 10-11 night): early (default) = the measurement probe board_probe.py joins the
#      ROS graph at rig start and records from the end of WARM (no new DDS participant inside the
#      measurement window: one joining there can stall the producer's publish ~300 ms, the v1/v2
#      formal-run stalls); late = the v1/v2 way (probe starts at the window start)
#      PUB_MODE (v3): producer --dds-publish-mode (env | sync | async; empty = producer default)
#      CAM_DDS_PROFILE (v8, 10-12): Fast DDS XML profile for the IMAGE PUBLISHER ONLY: the replay camera
#      process alone gets FASTRTPS_DEFAULT_PROFILES_FILE=<it>; every other process keeps its environment.
#      A path (absolute, or relative to the repo root): tools/nano_ros2/fastdds_shm_localhost.xml (NOT
#      fastdds_shm.xml: its UDP transport is on all interfaces, and on the Nano the loopback-only
#      participants of this chain then never discover the camera -- board 2026-10-12, 0 frames): a 32 MB
#      shared-memory segment instead of the 512 KB default, which is smaller than one RGB+depth tick
#      (1.15 MB at 640x360) so a late reader loses the frame without any error. Empty = the Fast DDS
#      default (control arm). A missing file aborts (exit 4). env.txt records the profile, its sha256
#      and the largest /dev/shm/fastrtps_* segment seen at the window start (the effect check).
#      INPUT_LOG (0/1, default 0): producer --input-log $OUT/input_log.csv (one row per depth arrival)
#      RX_FREEZE="EVERY:MS" (optional, HOST known-answer stress only, never on a board timing run):
#      tools/integ_car_chain/rx_freeze.py SIGSTOPs the producer for MS ms every EVERY s in the window
#      CAMERA (v8, 10-12): replay (default, as before) | live: the Orbbec Gemini 2 driver of the image
#      fpga-ros:humble-nav-orbbec-scipy-20261017 instead of the replay camera (the container needs the camera's USB
#      node). Then: the driver (LIVE_LAUNCH_ARGS, default 640x480 RGB + depth hardware-aligned to colour, 30
#      fps, publish_tf:=false) is the image publisher (CAM_DDS_PROFILE applies to it and to nobody else);
#      tools/integ_car_chain/live_tf.py stands in for the car's TF (at rest); tools/integ_car_chain/cam_probe.py
#      is a second receiver of depth + colour (cam_probe.csv / cam_probe.json: stamp intervals, frames missing
#      by stamp, stamp -> arrival delay, publisher QoS); the producer reads LIVE_DEPTH_TOPIC / LIVE_INFO_TOPIC
#      in the driver's frame. No replay.csv, no /fpga/memory, no /fpga/rgb: use it without the runtime.
#      camera_info.json (10-13): the CameraInfo the producer was given (LIVE_INFO_TOPIC) and the colour camera's
#      (LIVE_COLOR_INFO_TOPIC), recorded by live_tf.py: with depth aligned to colour the two must agree.
#      The caller sets DATA_SOURCE=live. A driver that publishes no CameraInfo within 60 s aborts (exit 5).
#      RX_LITE (2026-10-14, camera-drop arm l_chain32; live only, default 0): 1 = one more receiver next to the
#      producer and cam_probe: tools/integ_car_chain/rx_lite.py (raw subscription, stamp only, DDS message-lost
#      count) on depth + colour + both camera_info, best effort keep 5 -> rx_chain.csv / rx_chain.json; also
#      writes window.txt ("WIN_START WIN_END", wall s) so that tools/integ_car_chain/cam_drop/bx_readout.py reads
#      this directory. LIVE_DRV_LOG (live only, default 0): 1 = the driver's own per-frame record
#      (enable_frame_drop_log:=true frame_timestamp_csv_file:=$OUT/drv_frames.csv, as bx_inner.sh DRV_LOG=1).
#      Both off = the chain exactly as before.
#      RT_INFO (live only, 10-15, optional): the runner's runtime_info.json when a runner is in the chain, the
#      expectation of the camera-graph snapshot (tools/integ_car_chain/graph_snapshot.py -> camera_topics.txt +
#      camera_topics_wait.json + graph_snapshot.log); GRAPH_WAIT_S (default 30): its longest wait.
# Never pkill -f: every child is stopped by its own pid.
cd "$(dirname "$0")/../.."
source /opt/ros/humble/setup.bash          # not nounset-clean: source before set -u
[ "${CAMERA:-replay}" = live ] && source /opt/orbbec_ws/install/setup.bash
set -u
export PYTHONPATH=$PWD:${PYTHONPATH:-} PYTHONUNBUFFERED=1
: "${OUT:?}" "${PREP:?}" "${KV:?}" "${DUR:=180}" "${HZ:=10}" "${MEMBOXES:=24}" "${COSTMAP:=1}" "${WARM:=20}" "${BACKEND:=remote_pl}"
: "${CAMERA:=replay}" "${LIVE_DEPTH_TOPIC:=/camera/depth/image_raw}" "${LIVE_INFO_TOPIC:=/camera/depth/camera_info}" "${LIVE_COLOR_TOPIC:=/camera/color/image_raw}"
: "${LIVE_COLOR_INFO_TOPIC:=/camera/color/camera_info}"
: "${LIVE_LAUNCH_ARGS:=color_width:=640 color_height:=480 color_fps:=30 color_format:=RGB depth_width:=640 depth_height:=400 depth_fps:=30 depth_format:=Y16 enable_ir:=false enable_point_cloud:=false depth_registration:=true align_mode:=HW publish_tf:=false}"
case "$CAMERA" in replay|live) ;; *) echo "CAMERA must be replay or live" >&2; exit 2;; esac
mkdir "$OUT" || exit 2
CAM_ENV=()
if [ -n "${CAM_DDS_PROFILE:-}" ]; then
  CAM_XML=$(readlink -f "$CAM_DDS_PROFILE")
  [ -f "$CAM_XML" ] || { echo "CAM_DDS_PROFILE $CAM_DDS_PROFILE: no such file" >&2; exit 4; }
  CAM_ENV=(env "FASTRTPS_DEFAULT_PROFILES_FILE=$CAM_XML")
fi
{ echo "domain=$ROS_DOMAIN_ID backend=$BACKEND kv=$KV dur=$DUR hz=$HZ memboxes=$MEMBOXES costmap=$COSTMAP camera=$CAMERA"; python3 -c "import numpy,sys;print(sys.version.split()[0], numpy.__version__)"; } > "$OUT/env.txt"
pids=()
if [ "$BACKEND" = remote_pl ] && [ "$KV" = fake ]; then
  python3 tools/kv_link/kv_server.py --core cpu --roi-core cpu --port 0 --out "$OUT/kv_fake" > "$OUT/kv_fake.json" 2> "$OUT/kv_fake.log" & pids+=($!)
  for i in $(seq 1 100); do grep -q '"port"' "$OUT/kv_fake.json" 2>/dev/null && break; sleep 0.2; done
  KV=127.0.0.1:$(python3 -c "import json,sys;print(json.loads(open(sys.argv[1]).readline())['port'])" "$OUT/kv_fake.json")
fi
if [ "$COSTMAP" = 1 ]; then
  python3 tools/car_geo_producer/costmap_params.py --out "$OUT/costmap.yaml" > "$OUT/costmap_check.json" || exit 3
  ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args --params-file "$OUT/costmap.yaml" > "$OUT/costmap.log" 2>&1 & pids+=($!)
  ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=check_lifecycle --params-file "$OUT/costmap.yaml" > "$OUT/lifecycle.log" 2>&1 & pids+=($!)
fi
# robot_radius_m 0.30 since 2026-10-14 (line D1): one radius for Nav2 (nav2_overrides.yaml), this bridge
# and the runner (host/ros_runtime.py free()); was 0.15
python3 host/ros_rolling_nav.py --ros-args -p ground_z_m:=0.0 -p robot_height_m:=0.25 -p robot_radius_m:=0.30 > "$OUT/bridge.log" 2>&1 &
BRIDGE=$!; pids+=($BRIDGE)
stop_all() { for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null; done; sleep 3; for p in "${pids[@]}"; do kill -9 "$p" 2>/dev/null; done; }
LIVE=()
if [ "$CAMERA" = replay ]; then
"${CAM_ENV[@]}" python3 tools/car_geo_producer/replay_camera.py --prep "$PREP" --hz "$HZ" --duration-s $((DUR + 40)) \
  --memory-boxes "$MEMBOXES" --log "$OUT/replay.csv" ${REPLAY_EXTRA:-} > "$OUT/replay.log" 2>&1 & pids+=($!)
REPLAY=$!
else
  [ "${LIVE_DRV_LOG:-0}" = 1 ] && LIVE_LAUNCH_ARGS="$LIVE_LAUNCH_ARGS enable_frame_drop_log:=true frame_timestamp_csv_file:=$OUT/drv_frames.csv"
  echo "live_launch_args=$LIVE_LAUNCH_ARGS" >> "$OUT/env.txt"
  # 10-13: the driver's depth stream does not come up on about half of the starts (board 10-15, cam-start-dig:
  # every start judged by depth DATA, 12 of 23 good; the plain SDK probe 13 of 13; cause not found). A start is good
  # LIVE_DEPTH_MSGS (30, about 1-1.5 s of depth) images on LIVE_DEPTH_TOPIC within LIVE_START_WAIT_S while the driver
  # still runs (3 is too few: cam-start-dig m11 session 8 got 4 depth images at 30 fps, then none) -- NOT
  # "depth Frame" in driver.log: that line was printed on starts that then sent no depth image (10-15 l_chain32,
  # cam-start-dig m5: 4 of 6 failed starts). The probe subscribes 1 s before the launch (the driver publishes images
  # only to subscribers). A failed start is stopped and repeated after LIVE_START_GAP_S, up to LIVE_START_TRIES times;
  # its logs -> driver_failed_start_N.log / driver_probe_N.log. All of this is before the warm-up; env.txt
  # records every try.
  for try in $(seq 1 "${LIVE_START_TRIES:-10}"); do
    python3 tools/car_run/depth_probe.py --topic "$LIVE_DEPTH_TOPIC" --count "${LIVE_DEPTH_MSGS:-30}" \
      --timeout-s $((${LIVE_START_WAIT_S:-20} + 1)) > "$OUT/driver_probe_$try.log" 2>&1 &
    DPROBE=$!   # not PROBE: rig_inner.sh takes PROBE=1 from outside (the system sampler switch; 10-15 v8rt: it was overwritten here)
    sleep 1
    "${CAM_ENV[@]}" ros2 launch orbbec_camera gemini2.launch.py $LIVE_LAUNCH_ARGS > "$OUT/driver.log" 2>&1 &
    LAUNCH=$!
    wait $DPROBE; prc=$?
    started=0
    [ $prc = 0 ] && kill -0 $LAUNCH 2>/dev/null && started=1
    echo "driver_start try=$try depth_stream=$started probe_rc=$prc" >> "$OUT/env.txt"
    [ $started = 1 ] && break
    [ "$try" = "${LIVE_START_TRIES:-10}" ] && break    # leave the last one running: the CameraInfo check below reports it
    cp "$OUT/driver.log" "$OUT/driver_failed_start_$try.log"
    kids=$(pgrep -P $LAUNCH)
    kill -INT $kids $LAUNCH 2>/dev/null
    for i in $(seq 1 50); do kill -0 $LAUNCH 2>/dev/null || break; sleep 0.2; done
    kill -9 $kids $LAUNCH 2>/dev/null
    sleep "${LIVE_START_GAP_S:-10}"
  done
  pids+=($LAUNCH)
  python3 tools/integ_car_chain/live_tf.py --info-topic "$LIVE_INFO_TOPIC" --frame-out "$OUT/camera_frame.txt" \
    --info-out "$OUT/camera_info.json" --extra-info-topic "$LIVE_COLOR_INFO_TOPIC" \
    --duration-s $((WARM + DUR + 120)) > "$OUT/live_tf.log" 2>&1 & pids+=($!)
  for i in $(seq 1 300); do [ -s "$OUT/camera_frame.txt" ] && break; sleep 0.2; done
  # the driver process itself (ros2 launch's child): stopped by its own pid at the end, sampled as "replay"
  for c in $(pgrep -P $LAUNCH); do pids+=($c); done
  REPLAY=$(pgrep -P $LAUNCH | head -1); REPLAY=${REPLAY:-$LAUNCH}
  if [ ! -s "$OUT/camera_frame.txt" ]; then
    echo "no CameraInfo on $LIVE_INFO_TOPIC within 60 s" >&2; tail -20 "$OUT/driver.log" >&2; stop_all; exit 5
  fi
  # camera_topics.txt: the camera graph (publishers' QoS; the reliability of a runner already subscribed; + the depth
  # CameraInfo since 10-15). Taken HERE: after the runner subscribed (the caller starts it before this script and
  # waits for its latched /fpga/runtime_info, published after its camera subscriptions) and before the producer starts
  # below (v8rt: the producer subscribes RELIABLE, which would be counted as the runner's). 10-15 v8rt: one
  # `ros2 topic info -v` per topic right after the driver came up caught DDS discovery unconverged in all four arms;
  # tools/integ_car_chain/graph_snapshot.py repeats the same calls until converged (every topic with a publisher;
  # the runner's RELIABLE subscriptions as RT_INFO's camera_qos says) or GRAPH_WAIT_S, and writes
  # camera_topics_wait.json beside it (runner_qos_readout.graph_check voids a timed-out arm). RT_INFO (optional):
  # the runner's runtime_info.json (nano_chain_inner_v2.sh, RUNTIME=1); unset/empty or no file = no runner: only
  # the three publishers are waited for. Never fails the rig (exit 0 either way).
  # (before cam_probe / rx_lite: their --duration-s counts from their own start, so a long wait here does not
  # cut their tail inside the window; both are best effort and do not enter the counted subscriptions)
  GS=(); [ -n "${RT_INFO:-}" ] && GS=(--runtime-info "$RT_INFO")
  python3 tools/integ_car_chain/graph_snapshot.py --topic "$LIVE_DEPTH_TOPIC" --topic "$LIVE_COLOR_TOPIC" \
    --topic "$LIVE_INFO_TOPIC" --out "$OUT/camera_topics.txt" --wait-json "$OUT/camera_topics_wait.json" "${GS[@]}" \
    --header "camera_frame=$(head -1 "$OUT/camera_frame.txt") driver_pid=$REPLAY launch_pid=$LAUNCH" \
    --timeout-s "${GRAPH_WAIT_S:-30}" > "$OUT/graph_snapshot.log" 2>&1
  python3 tools/integ_car_chain/cam_probe.py --topic "$LIVE_DEPTH_TOPIC" --topic "$LIVE_COLOR_TOPIC" \
    --out "$OUT/cam_probe.csv" --summary "$OUT/cam_probe.json" --duration-s $((WARM + DUR + 22)) > "$OUT/cam_probe.log" 2>&1 & pids+=($!)
  if [ "${RX_LITE:-0}" = 1 ]; then
    python3 tools/integ_car_chain/rx_lite.py --topic "$LIVE_DEPTH_TOPIC" --topic "$LIVE_COLOR_TOPIC" \
      --info-topic "$LIVE_INFO_TOPIC" --info-topic "$LIVE_COLOR_INFO_TOPIC" --out "$OUT/rx_chain.csv" \
      --summary "$OUT/rx_chain.json" --duration-s $((WARM + DUR + 22)) --reliability best_effort --depth 5 \
      --node-name fpga_rx_lite_chain > "$OUT/rx_chain.log" 2>&1 & pids+=($!)
    echo "rx_lite chain pid=$! reliability=best_effort depth=5" >> "$OUT/env.txt"
  fi
  LIVE=(--depth-topic "$LIVE_DEPTH_TOPIC" --info-topic "$LIVE_INFO_TOPIC" --camera-frame "$(head -1 "$OUT/camera_frame.txt")")
fi
# the depth of this rig is always the replay stand-in above: the producer says so (replay mark on
# every KV260 request -> AR panel REPLAY; frame_stats source=replay). DATA_SOURCE overrides only
# for a deliberate mismatch test.
EXP=(--data-source "${DATA_SOURCE:-replay}" --backend "$BACKEND")
[ "$BACKEND" = remote_pl ] && EXP+=(--kv-server "$KV")
[ "$BACKEND" = remote_pl ] && [ -n "${EXPECT_BUILD_ID:-}" ] && EXP+=(--kv-expect-build-id "$EXPECT_BUILD_ID")
[ -n "${PUB_MODE:-}" ] && EXP+=(--dds-publish-mode "$PUB_MODE")
[ "${INPUT_LOG:-0}" = 1 ] && EXP+=(--input-log "$OUT/input_log.csv")
${PRODUCER_WRAP:-} python3 -m host.ros_geo_producer "${EXP[@]}" "${LIVE[@]}" --ground-z-m 0.0 --rate-hz "$HZ" \
  --duration-s $((DUR + 20)) --stats-out "$OUT/frames.jsonl" --summary-out "$OUT/producer_summary.json" \
  ${PRODUCER_EXTRA:-} > "$OUT/producer.log" 2>&1 &
PROD=$!; pids+=($PROD)
echo "$PROD" > "$OUT/producer.pid"
if [ "${PROBE:-0}" = 1 ]; then
  python3 tools/integ_car_chain/tf_probe.py --tf-csv "$OUT/tf_probe.csv" --sys-csv "$OUT/sys.csv" \
    --duration-s $((WARM + DUR + 25)) --pid producer=$PROD --pid bridge=$BRIDGE --pid replay=$REPLAY \
    --threads-jsonl "$OUT/threads.jsonl" --threads-of producer --graph-jsonl "$OUT/graph.jsonl" \
    ${PROBE_PIDS:-} > "$OUT/tf_probe.log" 2>&1 & pids+=($!)
fi
WIN_START=$(python3 -c "import time,sys;print('%.3f' % (time.time() + float(sys.argv[1])))" "$WARM")
if [ "${PROBE_JOIN:-early}" = early ]; then
  python3 tools/car_geo_producer/board_probe.py --start-at-wall "$WIN_START" --duration-s "$DUR" \
    --out "$OUT/probe.json" > "$OUT/probe.log" 2>&1 &
  BPROBE=$!; pids+=($BPROBE)
fi
echo "probe_join=${PROBE_JOIN:-early} win_start_wall=$WIN_START" >> "$OUT/env.txt"
[ "${RX_LITE:-0}" = 1 ] && python3 -c "import sys;print('%.3f %.3f' % (float(sys.argv[1]), float(sys.argv[1]) + float(sys.argv[2])))" \
  "$WIN_START" "$DUR" > "$OUT/window.txt"
python3 -c "import time,sys;time.sleep(max(0.0, float(sys.argv[1]) - time.time()))" "$WIN_START"
ls -la /dev/shm > "$OUT/shm_ls.txt" 2>&1
{ echo "cam_dds_profile=${CAM_XML:-default} sha256=$([ -n "${CAM_XML:-}" ] && sha256sum "$CAM_XML" | cut -c1-64)"
  echo "replay_env_profile=$(tr '\0' '\n' < /proc/$REPLAY/environ 2>/dev/null | grep '^FASTRTPS_DEFAULT_PROFILES_FILE=' || echo none)"
  echo "shm_segment_bytes_max=$(stat -c %s /dev/shm/fastrtps_* 2>/dev/null | sort -n | tail -1) dev_shm_kb=$(df -k --output=size,used /dev/shm | tail -1 | tr -s ' ' ':')"
} >> "$OUT/env.txt"
if [ -n "${RX_FREEZE:-}" ]; then
  python3 tools/integ_car_chain/rx_freeze.py "${RX_FREEZE%%:*}" "${RX_FREEZE##*:}" "$DUR" "$PROD" \
    > "$OUT/rx_freeze.csv" 2> "$OUT/rx_freeze.err" & pids+=($!)
fi
if [ -n "${CHURN:-}" ]; then
  python3 tools/integ_car_chain/participant_churn.py --log "$OUT/churn.jsonl" --duration-s "$DUR" \
    --period-s "${CHURN%%:*}" --life-s "${CHURN##*:}" ${CHURN_EXTRA:-} > "$OUT/churn.log" 2>&1 & pids+=($!)
fi
cpu() { awk '{printf "%.0f\n", $14+$15}' /proc/$1/stat 2>/dev/null || echo NA; }
sys_cpu() { awk '/^cpu /{b=$2+$3+$4+$7+$8+$9; t=b+$5+$6; printf "%.0f:%.0f\n", b, t}' /proc/stat; }
TCK=$(getconf CLK_TCK); T0=$(date +%s.%N); P0=$(cpu $PROD); B0=$(cpu $BRIDGE); S0=$(sys_cpu)
if [ "${PROBE_JOIN:-early}" = early ]; then
  wait $BPROBE
else
  python3 tools/car_geo_producer/board_probe.py --duration-s "$DUR" --out "$OUT/probe.json" > "$OUT/probe.log" 2>&1
fi
T1=$(date +%s.%N); P1=$(cpu $PROD); B1=$(cpu $BRIDGE); S1=$(sys_cpu)
python3 - "$T0" "$T1" "$P0" "$P1" "$B0" "$B1" "$TCK" "$S0" "$S1" "$(nproc)" > "$OUT/cpu.json" <<'PY'
import json, sys
t0, t1, p0, p1, b0, b1, tck, s0, s1, ncpu = sys.argv[1:]
wall = float(t1) - float(t0)
def pc(a, b):
    try:
        return round((float(b) - float(a)) / float(tck) / wall * 100, 2)
    except ValueError:
        return None
(bb0, tt0), (bb1, tt1) = (map(int, s0.split(":")), map(int, s1.split(":")))
whole = round((bb1 - bb0) / (tt1 - tt0) * 100, 2) if tt1 > tt0 else None
print(json.dumps({"window_s": round(wall, 2), "t0_wall": float(t0), "t1_wall": float(t1),
                  "producer_pct_one_core": pc(p0, p1), "bridge_pct_one_core": pc(b0, b1),
                  "machine_busy_pct_of_all_cores": whole, "machine_cores": int(ncpu),
                  "machine_busy_cores": round(whole / 100 * int(ncpu), 3) if whole is not None else None}))
PY
wait $PROD
echo "producer_rc=$?" > "$OUT/producer_rc.txt"
ros2 topic list > "$OUT/topics.txt" 2>&1
stop_all
python3 tools/car_geo_producer/summarize.py "$OUT" > "$OUT/summary.json" 2> "$OUT/summarize.log"
cat "$OUT/summary.json"
