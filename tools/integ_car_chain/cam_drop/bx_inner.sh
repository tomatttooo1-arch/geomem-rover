#!/bin/bash
# One arm of the camera-drop layer experiments, run INSIDE the driver image (started by bx.sh).
# Repo copy (2026-10-14) of the version after the START_TRIES fix (arm l_a2 onwards); no host paths, only the
# container mounts named below.
# Mounts: /repo (ro, the worktree HEAD), /out (this arm's new directory), /t (ro, these tools), /prep (ro).
# env: DUR WARM CAMERA (live | replay)
#      LAUNCH_ARGS (live: all of the driver's launch arguments; default = the chain's)  LAUNCH_EXTRA (appended)
#      DRV_LOG=1: the driver's own per-frame record (enable_frame_drop_log + frame_timestamp_csv_file=/out/drv_frames.csv)
#      RX="name:reliability:depth ..." light receivers (rx_lite.py), e.g. "be:best_effort:5 rel:reliable:30"
#      RX_INFO=1: every light receiver also records the two camera_info topics
#      PY_PROBE=1: tools/integ_car_chain/cam_probe.py (the chain's Python probe) on depth + colour
#      CAM_DDS_PROFILE: Fast DDS XML for the image publisher only (path in the container)
#      REPLAY_HZ REPLAY_EXTRA (replay: --rgb, --debug-skip-frames ...)  RX_FREEZE="EVERY:MS:NAME" (freeze receiver NAME)
#      START_TRIES START_WAIT_S START_GAP_S START_DEPTH_MSGS (live driver start retry, as rig_inner.sh; good start =
#      tools/car_run/depth_probe.py gets START_DEPTH_MSGS depth images)
# Every child is stopped by its own pid. Never pkill -f.
source /opt/ros/humble/setup.bash
[ "${CAMERA:-live}" = live ] && source /opt/orbbec_ws/install/setup.bash
set -u
cd /repo || exit 9
export PYTHONPATH=/repo:${PYTHONPATH:-} PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
: "${DUR:=60}" "${WARM:=10}" "${CAMERA:=live}" "${RX:=be:best_effort:5}" "${RX_INFO:=1}" "${PY_PROBE:=0}" "${DRV_LOG:=1}"
: "${LAUNCH_ARGS:=color_width:=640 color_height:=480 color_fps:=30 color_format:=RGB depth_width:=640 depth_height:=400 depth_fps:=30 depth_format:=Y16 enable_ir:=false enable_point_cloud:=false depth_registration:=true align_mode:=HW publish_tf:=false}"
: "${LAUNCH_EXTRA:=}" "${REPLAY_HZ:=30}" "${REPLAY_EXTRA:=}" "${RX_FREEZE:=}"
# defaults set HERE: a ${X:=v} inside $(...) is assigned in a subshell and is gone afterwards (arm l_a1 died
# on "START_TRIES: unbound variable" at its first failed driver start)
: "${START_TRIES:=10}" "${START_WAIT_S:=20}" "${START_GAP_S:=10}"
OUT=/out
E=$OUT/env.txt
{ echo "camera=$CAMERA dur=$DUR warm=$WARM rx=$RX rx_info=$RX_INFO py_probe=$PY_PROBE drv_log=$DRV_LOG"
  echo "domain=$ROS_DOMAIN_ID localhost_only=${ROS_LOCALHOST_ONLY:-} rmw=${RMW_IMPLEMENTATION:-unset} pubmode=${RMW_FASTRTPS_PUBLICATION_MODE:-unset}"
  echo "repo_commit=$(cat /repo/COMMIT 2>/dev/null)"; } > $E
CAM_ENV=()
if [ -n "${CAM_DDS_PROFILE:-}" ]; then
  [ -f "$CAM_DDS_PROFILE" ] || { echo "no profile $CAM_DDS_PROFILE" >&2; exit 4; }
  CAM_ENV=(env "FASTRTPS_DEFAULT_PROFILES_FILE=$CAM_DDS_PROFILE")
  echo "cam_dds_profile=$CAM_DDS_PROFILE" >> $E
fi
pids=()
if [ "$CAMERA" = live ]; then
  DEPTH=/camera/depth/image_raw; COLOR=/camera/color/image_raw
  INFOS="/camera/depth/camera_info /camera/color/camera_info"
  ARGS="$LAUNCH_ARGS $LAUNCH_EXTRA"
  [ "$DRV_LOG" = 1 ] && ARGS="$ARGS enable_frame_drop_log:=true frame_timestamp_csv_file:=$OUT/drv_frames.csv"
  echo "launch_args=$ARGS" >> $E
  # 10-15 (cam-start-dig): a start is good only when depth DATA arrives -- tools/car_run/depth_probe.py (copied from
  # session 8 got 4 depth images, then none) images on $DEPTH within START_WAIT_S
  # while the driver still runs; it subscribes 1 s before the launch. Not "depth Frame" in driver.log: that line was
  # printed on starts that then sent no depth image (cam-start-dig m5: 4 of 6 failed starts).
  for try in $(seq 1 "${START_TRIES:=10}"); do
    python3 tools/car_run/depth_probe.py --topic "$DEPTH" --count "${START_DEPTH_MSGS:-30}" \
      --timeout-s $((${START_WAIT_S:=20} + 1)) > $OUT/driver_probe_$try.log 2>&1 &
    DPROBE=$!   # named as in rig_inner.sh, where PROBE is the system sampler switch from outside (10-15 v8rt)
    sleep 1
    "${CAM_ENV[@]}" ros2 launch orbbec_camera gemini2.launch.py $ARGS > $OUT/driver.log 2>&1 &
    LAUNCH=$!
    wait $DPROBE; prc=$?
    started=0
    [ $prc = 0 ] && kill -0 $LAUNCH 2>/dev/null && started=1
    echo "driver_start try=$try depth_stream=$started probe_rc=$prc at=$(date +%T.%N | cut -c1-12)" >> $E
    [ $started = 1 ] && break
    cp $OUT/driver.log $OUT/driver_failed_start_$try.log
    kids=$(pgrep -P $LAUNCH)
    kill -INT $kids $LAUNCH 2>/dev/null
    for i in $(seq 1 50); do kill -0 $LAUNCH 2>/dev/null || break; sleep 0.2; done
    kill -9 $kids $LAUNCH 2>/dev/null
    [ "$try" = "$START_TRIES" ] && { echo "driver never started" >> $E; echo 5 > $OUT/rc; exit 5; }
    sleep "${START_GAP_S:=10}"
  done
  DRV=$(pgrep -P $LAUNCH | head -1)
  echo "launch_pid=$LAUNCH driver_pid=$DRV" >> $E
else
  DEPTH=/fpga/depth; COLOR=/fpga/rgb; INFOS="/fpga/camera_info"
  "${CAM_ENV[@]}" python3 tools/car_geo_producer/replay_camera.py --prep /prep --hz "$REPLAY_HZ" --duration-s $((WARM + DUR + 20)) \
    --log $OUT/replay.csv $REPLAY_EXTRA > $OUT/replay.log 2>&1 &
  LAUNCH=$!; DRV=$LAUNCH
  echo "replay_pid=$LAUNCH replay_hz=$REPLAY_HZ replay_extra=$REPLAY_EXTRA" >> $E
  for i in $(seq 1 100); do [ -s $OUT/replay.csv ] && break; sleep 0.2; done
fi
sleep 2
TOPICS="--topic $DEPTH"
case " $REPLAY_EXTRA " in *" --rgb "*) HAS_COLOR=1;; *) HAS_COLOR=0;; esac
[ "$CAMERA" = live ] && HAS_COLOR=1
case " $LAUNCH_EXTRA " in *" enable_color:=false "*) HAS_COLOR=0;; esac
[ "$HAS_COLOR" = 1 ] && TOPICS="$TOPICS --topic $COLOR"
INFO_ARGS=""
if [ "$RX_INFO" = 1 ]; then for t in $INFOS; do
  case "$t" in */color/*) [ "$HAS_COLOR" = 1 ] || continue;; esac
  INFO_ARGS="$INFO_ARGS --info-topic $t"; done; fi
# receivers live from now to window end + 4 s: the window is fixed right after they are started (nothing
# of unknown duration in between; the graph query runs in the background during the warm-up)
RXLIFE=$((3 + WARM + DUR + 4))
declare -A RXPID
for spec in $RX; do
  IFS=: read -r name rel depth <<< "$spec"
  python3 tools/integ_car_chain/rx_lite.py $TOPICS $INFO_ARGS --out $OUT/rx_$name.csv --summary $OUT/rx_$name.json \
    --duration-s $RXLIFE --reliability "$rel" --depth "$depth" --node-name fpga_rx_lite_$name > $OUT/rx_$name.log 2>&1 &
  RXPID[$name]=$!; pids+=($!)
  echo "rx $name reliability=$rel depth=$depth pid=$!" >> $E
done
if [ "$PY_PROBE" = 1 ]; then
  PT="--topic $DEPTH"; [ "$HAS_COLOR" = 1 ] && PT="$PT --topic $COLOR"
  python3 tools/integ_car_chain/cam_probe.py $PT --out $OUT/cam_probe.csv --summary $OUT/cam_probe.json \
    --duration-s $RXLIFE > $OUT/cam_probe.log 2>&1 &
  pids+=($!); echo "py_probe pid=$!" >> $E
fi
WIN_START=$(python3 -c "import time,sys;print('%.3f' % (time.time() + 3.0 + float(sys.argv[1])))" "$WARM")
{ sleep 3; for t in $DEPTH $COLOR $INFOS; do echo "=== $t"; timeout 8 ros2 topic info -v --no-daemon --spin-time 2 "$t"; done
  echo "graph_query_done=$(date +%s.%N | cut -c1-14)" >> $E; } > $OUT/topics_info.txt 2>&1 &
pids+=($!)
WIN_END=$(python3 -c "import sys;print('%.3f' % (float(sys.argv[1]) + float(sys.argv[2])))" "$WIN_START" "$DUR")
echo "$WIN_START $WIN_END" > $OUT/window.txt
echo "win_start_wall=$WIN_START win_end_wall=$WIN_END" >> $E
python3 -c "import time,sys;time.sleep(max(0.0, float(sys.argv[1]) - time.time()))" "$WIN_START"
ls -la /dev/shm > $OUT/shm_ls.txt 2>&1
if [ -n "$RX_FREEZE" ]; then
  IFS=: read -r fe fm fn <<< "$RX_FREEZE"
  python3 tools/integ_car_chain/rx_freeze.py "$fe" "$fm" "$DUR" "${RXPID[$fn]}" > $OUT/rx_freeze.csv 2> $OUT/rx_freeze.err &
  pids+=($!); echo "rx_freeze every=$fe ms=$fm target=$fn pid=${RXPID[$fn]}" >> $E
fi
python3 -c "import time,sys;time.sleep(max(0.0, float(sys.argv[1]) + 2.0 - time.time()))" "$WIN_END"
ps -eLo pid,tid,psr,pcpu,comm,args --sort=-pcpu 2>/dev/null | head -40 | cut -c1-160 > $OUT/ps_end.txt
for p in "${pids[@]}"; do for i in $(seq 1 40); do kill -0 "$p" 2>/dev/null || break; sleep 0.25; done; done
for p in "${pids[@]}"; do kill -TERM "$p" 2>/dev/null; done
kids=$(pgrep -P $LAUNCH)
kill -INT $kids $LAUNCH 2>/dev/null
for i in $(seq 1 60); do kill -0 $LAUNCH 2>/dev/null || break; sleep 0.25; done
kill -9 $kids $LAUNCH "${pids[@]}" 2>/dev/null
echo "stopped at=$(date +%T)" >> $E
echo 0 > $OUT/rc
