#!/bin/bash
# Whole-chain board run, version 2 (10-11 night), INSIDE one ROS 2 Humble container from the repo root.
# Differences from nano_chain_inner.sh (v1, kept as it was for the first run):
#   * RUNTIME=1: the task runner (host.ros_runtime --geometry-source kv_topic --geometry-fallback none,
#     stub inference, pose-source reference) takes its geometry from the producer's /fpga/geometry_frame;
#     the producer stays the ONLY KV260 frame client. The replay then also publishes /fpga/rgb and no
#     fixed memory (MEMBOXES=0): /fpga/memory (and the AR class names) come from the runtime.
#   * hub stopped with SIGINT (the fix), its rc and stats recorded; runtime SIGINT rc + last event.
#   * PROBE=1: TF arrival probe + system sampler (per-process CPU, UDP counters) in the rig.
#   * HUB=0: no hub, no PC listener, no watcher (the rig-only arm of the TF-gap diagnosis).
# env: OUT (new dir) PREP KV EXPECT_BUILD_ID TOKEN HUBPORT DUR HZ WARM
#      ESTOP_AT (s after watcher start; <0 none) RUNTIME (0/1) HUB (0/1) PROBE (0/1)
#      PRODUCER_EXTRA (e.g. --tf-listener-thread)
#      RIG_MEMBOXES / RIG_REPLAY_EXTRA (optional overrides of the RUNTIME-dependent defaults;
#      e.g. RUNTIME=0 RIG_REPLAY_EXTRA=--rgb keeps the RGB topic without the runtime)
#      CHURN (passed to rig_inner.sh: new-participant generator, "PERIOD:LIFE")
#      FAKE_PLAN (0/1, default 0): SYNTHETIC /plan (tools/integ_car_chain/fake_plan.py, 1 Hz,
#      150 poses, frame map -- NOT Nav2) so the hub's STATE nav.path has data on a host run;
#      started first, because no new ROS process is started while the chain runs.
#      STALL_DIAG (0/1, default 0): runtime --stall-diag $OUT/runtime/stall_diag.jsonl (host/runner_stall_diag.py)
#      RUNTIME_EXTRA (extra runtime args, e.g. the positive-control --stall-diag-inject SECTION:EVERY:MS:free|hold,
#      or the reliable-/tf control arm --tf-reliable)
#      RECORD_WRITE (v7, 2026-10-12; async|sync, empty = the programs' default async): --record-write of BOTH the
#      runner (events / stall-diag files) and the producer (frames.jsonl); sync = the control arm (before v7)
#      GC_MODE (v7; default|freeze, empty = the programs' default freeze since 10-12): --gc of both the runner and the producer
#      CAM_DDS_PROFILE / INPUT_LOG / RX_FREEZE (v8, 2026-10-12; see rig_inner.sh): Fast DDS profile of the image
#      publisher only (empty = default, the control arm), the producer's depth arrival log, the host-only
#      receiver-freeze stress. Handed to rig_inner.sh explicitly below.
#      RX_LITE / LIVE_DRV_LOG (2026-10-14, camera-drop arm l_chain32; see rig_inner.sh): an extra light receiver with
#      the DDS message-lost count next to the producer, and the driver's per-frame record (live only; default 0 = as before)
#      CAMERA (v8; replay default | live, see rig_inner.sh): live = the real Orbbec driver instead of the replay
#      camera; then SRC below is "live" (hub, producer) and the rig sends no memory boxes.
#      CAMERA=live RUNTIME=1 (2026-10-15, runner camera-QoS A/B): the runner reads the driver's topics through
#      remaps (nobody publishes /fpga/rgb|depth|camera_info on the real camera): /fpga/rgb := LIVE_COLOR_TOPIC,
#      /fpga/depth := LIVE_DEPTH_TOPIC, /fpga/camera_info := LIVE_INFO_TOPIC (the DEPTH CameraInfo: with depth
#      hardware-aligned to colour it has the colour intrinsics, the colour frame and zero distortion; the colour
#      CameraInfo carries the lens distortion, which the runner refuses), --camera-frame LIVE_CAMERA_FRAME
#      (default camera_color_optical_frame, the board record of 10-15), sensor source RT_SRC=hardware (the
#      runner's name for live, set together with SRC=live below), --input-log $OUT/runtime/rx_runner.csv (copied to $OUT/rig/ at the end so that
#      cam_drop/bx_readout.py reads it as receiver rx_runner), /fpga/runtime_info written to
#      $OUT/runtime/runtime_info.json. The stub detector finds no index in a real image: no detections
#      (counted no_index), every frame still goes through the runner (a transport / timing arm, not a
#      recognition test). RUNTIME_EXTRA chooses the runner's --camera-qos (default auto = reliable for hardware).
# Data source (10-12, 5th version): this rig always replays recorded frames (rig_inner.sh's replay camera),
# so the runner (--sensor-source), the hub and the producer (--data-source) all say "replay" from ONE
# variable (SRC, not an env knob): STATE source is then replay, not inconsistent (it was: runner hardware).
# Never pkill -f: every child is stopped by its own pid.
cd "$(dirname "$0")/../.."
source /opt/ros/humble/setup.bash
set -u
export PYTHONPATH=$PWD:${PYTHONPATH:-} PYTHONUNBUFFERED=1 LANG=C.UTF-8 LC_ALL=C.UTF-8
: "${OUT:?}" "${PREP:?}" "${KV:?}" "${TOKEN:?}" "${HUBPORT:=47431}" "${DUR:=180}" "${HZ:=10}" "${ESTOP_AT:=110}"
: "${WARM:=20}" "${RUNTIME:=1}" "${HUB:=1}" "${PROBE:=1}" "${PRODUCER_EXTRA:=}" "${STALL_DIAG:=0}" "${RUNTIME_EXTRA:=}"
SRC=replay
: "${CAMERA:=replay}"
RT_SRC=replay; RT_LIVE=()
if [ "$CAMERA" = live ]; then
  SRC=live; RT_SRC=hardware
  RT_LIVE=(--camera-frame "${LIVE_CAMERA_FRAME:-camera_color_optical_frame}" --input-log "$OUT/runtime/rx_runner.csv"
           --ros-args -r "/fpga/rgb:=${LIVE_COLOR_TOPIC:-/camera/color/image_raw}"
           -r "/fpga/depth:=${LIVE_DEPTH_TOPIC:-/camera/depth/image_raw}"
           -r "/fpga/camera_info:=${LIVE_INFO_TOPIC:-/camera/depth/camera_info}")
fi
RW_ARGS=""; [ -n "${RECORD_WRITE:-}" ] && RW_ARGS="--record-write $RECORD_WRITE"
[ -n "${GC_MODE:-}" ] && RW_ARGS="$RW_ARGS --gc $GC_MODE"
PRODUCER_EXTRA="$PRODUCER_EXTRA $RW_ARGS"
mkdir "$OUT" || exit 2
raw() { python3 -c "from host.fpga_clock import now_ns; print(now_ns())"; }
TOTAL=$((WARM + DUR + 45))
# 10-13, real camera: the driver may need several starts (rig_inner.sh, up to LIVE_START_TRIES of about 30 s
# each) before the warm-up begins, while the hub, the watch and the PC listener below start first and ran a
# fixed time: after nine tries the watch was over before the window began. With CAMERA=live they get that
# allowance on top, and the watch is ended by signal once the rig is done (replay: unchanged).
[ "$CAMERA" = live ] && TOTAL=$((TOTAL + ${LIVE_START_TRIES:-10} * (${LIVE_START_WAIT_S:-20} + ${LIVE_START_GAP_S:-10})))
PP=""
echo "start_raw_ns=$(raw) start_wall=$(date +%s.%N)" > "$OUT/marks.txt"
if [ "${FAKE_PLAN:-0}" = 1 ]; then
  python3 tools/integ_car_chain/fake_plan.py --hz 1 --poses 150 --frame-id map \
    > "$OUT/fake_plan.out" 2> "$OUT/fake_plan.err" &
  FPLAN=$!
fi
if [ "$HUB" = 1 ]; then
  python3 -m host.hub.hub_ros --data-source "$SRC" --token-file "$TOKEN" --port "$HUBPORT" --bind 127.0.0.1 \
    --kv "$KV" --duration-s $((TOTAL + 120)) --stats-out "$OUT/hub_stats.json" > "$OUT/hub.out" 2> "$OUT/hub.err" &
  HUBP=$!
  for i in $(seq 1 100); do grep -q 'hub listening' "$OUT/hub.out" 2>/dev/null && break; sleep 0.2; done
  grep -q 'hub listening' "$OUT/hub.out" || { echo "hub not up"; kill -KILL $HUBP; exit 3; }
  python3 tools/integ_car_chain/chain_watch.py --server "$KV" --hub-port "$HUBPORT" --token-file "$TOKEN" \
    --seconds "$TOTAL" --estop-at-s "$ESTOP_AT" --out "$OUT/watch" > "$OUT/watch.out" 2> "$OUT/watch.err" &
  WATCH=$!
  if [ "$ESTOP_AT" -ge 0 ]; then
    for i in $(seq 1 100); do [ -e "$OUT/watch/pc_connected" ] && break; sleep 0.2; done
  fi
  # real camera: the listener is ended by SIGINT below, and a background child of this shell would start
  # with SIGINT ignored (fpgapc only knows KeyboardInterrupt; pc/ keeps to the Windows stdlib, no signal module)
  PCSIG=""; [ "$CAMERA" = live ] && PCSIG="env --default-signal=INT"
  $PCSIG python3 pc/fpgapc.py --host 127.0.0.1 --port "$HUBPORT" --token-file "$TOKEN" listen --duration-s "$TOTAL" \
    --out "$OUT/pc.fpgarec" --stats "$OUT/pc_stats.json" > "$OUT/pc_listen.out" 2> "$OUT/pc_listen.err" &
  PC=$!
  PP="$PP --pid hub=$HUBP --pid watch=$WATCH --pid pc_listen=$PC"
fi
if [ "$RUNTIME" = 1 ]; then
  mkdir -p "$OUT/runtime"
  SOCK=/tmp/fpga_stub_$$.sock
  python3 -B tools/car_geometry_kv/stub_inference.py --socket "$SOCK" > "$OUT/runtime/stub.out" 2> "$OUT/runtime/stub.err" &
  STUB=$!
  for i in $(seq 1 100); do [ -S "$SOCK" ] && break; sleep 0.1; done
  python3 -B tools/car_geometry_kv/stop_watch.py --out "$OUT/runtime/stop_watch.jsonl" > "$OUT/runtime/stop_watch.err" 2>&1 &
  SW=$!
  RT_DIAG=""
  [ "$STALL_DIAG" = 1 ] && RT_DIAG="--stall-diag $OUT/runtime/stall_diag.jsonl"
  # (--ros-args of RT_LIVE last: everything after it goes to rclpy)
  python3 -B -m host.ros_runtime --socket "$SOCK" --pose-source reference --sensor-source "$RT_SRC" \
    --inference-device cpu --events "$OUT/runtime/events.jsonl" --result-wakeup event \
    --geometry-source kv_topic --geometry-fallback none $RT_DIAG $RW_ARGS $RUNTIME_EXTRA "${RT_LIVE[@]}" \
    > "$OUT/runtime/runtime.out" 2> "$OUT/runtime/runtime.err" &
  RT=$!
  PP="$PP --pid runtime=$RT --pid stub=$STUB"
  sleep 3
  if [ "$CAMERA" = live ]; then   # latched start-up facts (camera_qos in effect); one short participant, before the rig
    timeout 60 python3 tools/sim_loop/latched_once.py --topic /fpga/runtime_info --out "$OUT/runtime/runtime_info.json" \
      --timeout-s 30 > "$OUT/runtime/runtime_info.log" 2>&1
  fi
  MEMBOXES=0; REPLAY_EXTRA="--rgb"
else
  MEMBOXES=24; REPLAY_EXTRA=""
fi
# CPU-attribution arms (10-11 night): override the rig's memory source / replay extras
[ -n "${RIG_MEMBOXES:-}" ] && MEMBOXES=$RIG_MEMBOXES
[ -n "${RIG_REPLAY_EXTRA:-}" ] && REPLAY_EXTRA=$RIG_REPLAY_EXTRA
# 10-15: the rig's camera-graph snapshot (live only) waits for the runner's subscriptions as its latched start-up
# facts say (camera_qos); no runner -> empty -> publishers only (rig_inner.sh, tools/integ_car_chain/graph_snapshot.py)
RT_INFO=""; [ "$RUNTIME" = 1 ] && RT_INFO="$OUT/runtime/runtime_info.json"
echo "{\"hub\": \"${HUBP:-}\", \"pc\": \"${PC:-}\", \"watch\": \"${WATCH:-}\", \"runtime\": \"${RT:-}\", \"stub\": \"${STUB:-}\"}" > "$OUT/pids.json"
echo "rig_start_raw_ns=$(raw) rig_start_wall=$(date +%s.%N)" >> "$OUT/marks.txt"
OUT="$OUT/rig" DATA_SOURCE="$SRC" PREP="$PREP" KV="$KV" EXPECT_BUILD_ID="${EXPECT_BUILD_ID:-}" BACKEND=remote_pl DUR="$DUR" HZ="$HZ" \
  COSTMAP=1 WARM="$WARM" MEMBOXES="$MEMBOXES" REPLAY_EXTRA="$REPLAY_EXTRA" PRODUCER_EXTRA="$PRODUCER_EXTRA" \
  RX_LITE="${RX_LITE:-0}" LIVE_DRV_LOG="${LIVE_DRV_LOG:-0}" \
  RT_INFO="$RT_INFO" PROBE="$PROBE" PROBE_PIDS="$PP" CAM_DDS_PROFILE="${CAM_DDS_PROFILE:-}" INPUT_LOG="${INPUT_LOG:-0}" RX_FREEZE="${RX_FREEZE:-}" CAMERA="$CAMERA" \
  bash tools/car_geo_producer/rig_inner.sh > "$OUT/rig.log" 2>&1
echo "rig_rc=$?" > "$OUT/rc.txt"
if [ "$RUNTIME" = 1 ]; then
  sleep 2
  echo "runtime_sigint_raw_ns=$(raw)" >> "$OUT/rc.txt"
  kill -INT $RT
  for i in $(seq 1 100); do kill -0 $RT 2>/dev/null || break; sleep 0.1; done
  kill -0 $RT 2>/dev/null && { echo "runtime alive 10 s after SIGINT, TERM" >> "$OUT/rc.txt"; kill -TERM $RT; }
  wait $RT; echo "runtime_rc=$?" >> "$OUT/rc.txt"
  [ "$CAMERA" = live ] && cp "$OUT/runtime/rx_runner.csv" "$OUT/runtime/rx_runner.json" "$OUT/rig/" 2>> "$OUT/rc.txt"
  sleep 1; kill -TERM $SW; wait $SW; echo "stop_watch_rc=$?" >> "$OUT/rc.txt"
  kill -TERM $STUB; wait $STUB; echo "stub_rc=$?" >> "$OUT/rc.txt"
fi
if [ "$HUB" = 1 ]; then
  if [ "$CAMERA" = live ]; then
    sleep 20; kill -INT $WATCH 2>/dev/null     # same ~20 s of tail as a replay run, then end the watch
    kill -INT $PC 2>/dev/null
  fi
  wait $WATCH; echo "watch_rc=$?" >> "$OUT/rc.txt"
  for i in $(seq 1 150); do kill -0 $PC 2>/dev/null || break; sleep 0.2; done
  kill -0 $PC 2>/dev/null && kill -INT $PC
  wait $PC; echo "pc_rc=$?" >> "$OUT/rc.txt"
  echo "hub_sigint_raw_ns=$(raw)" >> "$OUT/rc.txt"
  kill -INT $HUBP
  for i in $(seq 1 100); do kill -0 $HUBP 2>/dev/null || break; sleep 0.1; done
  if kill -0 $HUBP 2>/dev/null; then
    echo "hub alive 10 s after SIGINT, TERM" >> "$OUT/rc.txt"; kill -TERM $HUBP; sleep 5
    kill -0 $HUBP 2>/dev/null && { echo "hub alive after TERM, KILL" >> "$OUT/rc.txt"; kill -KILL $HUBP; }
  fi
  wait $HUBP; echo "hub_rc=$?" >> "$OUT/rc.txt"
  python3 pc/fpgapc.py verify --file "$OUT/pc.fpgarec" > "$OUT/pc_verify.json" 2>&1
fi
if [ -n "${FPLAN:-}" ]; then
  kill -INT $FPLAN; wait $FPLAN; echo "fake_plan_rc=$?" >> "$OUT/rc.txt"
fi
echo DONE >> "$OUT/rc.txt"
