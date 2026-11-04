#!/bin/bash
# N4 full-load layers on the Orin Nano (host side, run detached with nohup). docs/design/NANO_FULLLOAD_20260927.md
#   run_layers.sh <n4_dir> <run_name>
# <n4_dir> holds code/ (host robot_sim tools hw perception) ; results go to <n4_dir>/runs/<run_name>.
# Layers (cumulative): L0 idle, L1 driver+meter, L2 +detector, L3 +B4+rolling producer+bridge+consumer,
# L4 +RTAB-Map on a TUM fr1_desk ping-pong replay at 0.1x (~3 pairs/s; the live camera gave ~30 features and
# no inliers in the smoke, see the design doc §6.1), L5 +Nav2 with goals.
# After every layer check_layer.py applies the pre-registered criteria; on FAIL the run stops at that layer.
# At the last layer reached B4 is stopped and the same layer is measured again without it (<L>nob4).
# Env: WIN (s, default 300), WIN5 (s, default 600), SETTLE (s, default 60), B4_PASSES (default 200)
# Writes <run>/DONE or <run>/FAILED (reason). Needs the orin-nano and camera locks (taken on the server).
set -u
N4=$1; RUN=$N4/runs/$2; RUNNAME=$2
WIN=${WIN:-300}; WIN0=${WIN0:-$WIN}; WIN5=${WIN5:-600}; SETTLE=${SETTLE:-60}; B4_PASSES=${B4_PASSES:-200}
C=$N4/code; T=$C/tools
IMAGE=fpga-ros:humble-nav-orbbec-scipy-20261017
NAME=fpga-n4-ros
TAP=/dev/shm/fpga_n4
BENCH=$RUN/build/bench
PREP=$HOME/fpga/p8/prep_rec30
ENGINE=$HOME/fpga/p2/w/yolov8n-seg-640-fp16.engine
mkdir -p "$RUN" || exit 2
test ! -e "$RUN/phases.log" || { echo "run exists" >&2; exit 2; }
PH=$RUN/phases.log
ph() { echo "$(date +%s.%N) $(awk '{print $1}' /proc/uptime) $*" >> "$PH"; }
PIDS=()
teardown() {
  ph teardown
  [ -n "${DETP:-}" ] && kill -TERM "$DETP" 2>/dev/null
  [ -n "${B4P:-}" ] && kill "$B4P" 2>/dev/null
  docker exec $NAME bash -c 'pkill -INT -f rtabmap_slam/rtabmap; pkill -INT -f rgbd_odometry; pkill -INT -f n4_consumer.py; pkill -INT -f n4_meter.py; pkill -INT -f navigation_launch; pkill -INT -f rolling_nav2_smoke.py; pkill -INT -f ros_rolling_nav.py; pkill -INT -f velocity_guard.py; pkill -INT -f tum_rgbd_pub.py; pkill -INT -f static_transform_publisher' 2>/dev/null
  sleep 6
  docker exec $NAME bash -c 'pkill -INT -f gemini2.launch; pkill -INT -f component_container' 2>/dev/null
  sleep 4
  docker rm -f $NAME > /dev/null 2>&1
  sleep 1; touch "$RUN/.stop"; kill "${PIDS[@]}" 2>/dev/null; wait 2>/dev/null
  ls -la $TAP >> "$RUN/env_end.txt" 2>&1; rm -rf $TAP
  { date -Is; df -B1 / | tail -1; free -b; docker ps -a --format '{{.Names}}'; } >> "$RUN/env_end.txt" 2>&1
}
fail() { echo "$*" > "$RUN/FAILED"; ph "FAILED $*"; teardown; exit 1; }
guard() {  # sleep $1 seconds while checking temperature, disk and the long-lived processes
  local end=$(( $(date +%s) + $1 ))
  while [ "$(date +%s)" -lt "$end" ]; do
    local tmax=$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -1)
    [ "${tmax:-0}" -ge 90000 ] && fail "overtemp ${tmax} mC"
    local avail=$(df -B1 / | awk 'NR==2{print $4}')
    [ "$avail" -lt 5000000000 ] && fail "disk ${avail} B"
    [ -n "${DETP:-}" ] && ! kill -0 "$DETP" 2>/dev/null && fail "detector exited"
    [ -n "${B4P:-}" ] && [ -z "${B4_DONE_OK:-}" ] && ! kill -0 "$B4P" 2>/dev/null && fail "B4 exited early"
    [ -n "${CPID:-}" ] && [ ! -d /proc/$CPID ] && fail "container gone"
    sleep 5
  done
}
layer() {  # tag seconds
  ph "start $1"; guard "$2"; ph "end $1"
}
LAST=L0
check() {  # layer: apply the pre-registered criteria; stop the run at this layer on FAIL
  LAST=$1; sleep 3  # let the loggers flush the window's tail
  local out; out=$(python3 $T/nano_fullload/check_layer.py "$RUN" "$1" 2>&1); local rc=$?
  ph "check $1 rc=$rc $out"
  [ $rc = 0 ] && return 0
  echo "$1 $out" > "$RUN/STOPPED_AT"; finish; exit 0
}
finish() {  # measure the last layer again without B4, then tear down
  if [ -n "${B4P:-}" ]; then
    ph "stop B4"; B4_DONE_OK=1; kill -TERM "$B4P" 2>/dev/null; wait "$B4P"; echo "b4_rc=$?" >> "$PH"; B4P=
    guard 30; layer ${LAST}nob4 "$WIN"
  fi
  docker exec $NAME bash -c "du -sb /n4/runs/$RUNNAME/rtabmap/n4.db" >> "$RUN/env_end.txt" 2>&1
  teardown; ph DONE; touch "$RUN/DONE"
}
dx() { docker exec -d $NAME bash -c "source /opt/ros/humble/setup.bash; source /opt/orbbec_ws/install/setup.bash; cd /n4/code; export PYTHONPATH=/n4/code:\$PYTHONPATH; $*"; }
mkdir -p "$RUN/build"; M=$C/hw/m7
# same recipe as tools/geo_same_input/run_board.sh (P8); this tree adds only a line-buffered csv
{ g++ -std=c++11 -O2 -Wall -Wextra -Werror -ffp-contract=off -c "$M/frame_depth.cpp" -o "$RUN/build/fd_O2.o" &&
  g++ -std=c++11 -O2 -Wall -Wextra -Werror -ffp-contract=off -c "$M/object_evidence.cpp" -o "$RUN/build/oe_O2.o" &&
  g++ -std=c++14 -O3 -mcpu=native -fopenmp -ffp-contract=off -Wall -Wextra -Werror -I"$M" "$C/tools/nano_baseline/m7_cpu_bench.cpp" "$RUN/build/fd_O2.o" "$RUN/build/oe_O2.o" -o "$BENCH"; } \
  > "$RUN/build.log" 2>&1 || { echo build > "$RUN/FAILED"; exit 1; }
{ date -Is; nvpmodel -q 2>/dev/null; uptime; df -B1 / | tail -1; free -b
  for p in 0 4; do d=/sys/devices/system/cpu/cpufreq/policy$p; echo "policy$p $(cat $d/scaling_governor) min=$(cat $d/scaling_min_freq) max=$(cat $d/scaling_max_freq)"; done
  cat /sys/class/devfreq/17000000.gpu/governor /sys/class/devfreq/17000000.gpu/max_freq 2>/dev/null
  cat $C/COMMIT; sha256sum "$BENCH" "$PREP/depth_u16.bin" "$ENGINE"; docker image inspect -f '{{.Id}}' $IMAGE
  lsusb -d 2bc5:; top -bn1 | head -15; } > "$RUN/env.txt" 2>&1
BUSDEV=$(lsusb -d 2bc5: | head -1 | awk '{printf "/dev/bus/usb/%s/%s", $2, substr($4,1,3)}')
[ -e "$BUSDEV" ] || { echo "no camera" > "$RUN/FAILED"; exit 1; }
tegrastats --interval 500 --logfile "$RUN/tegrastats.log" & PIDS+=($!)
python3 $T/nano_ros2/proc_sampler.py --out "$RUN/procs.jsonl" --stop-file "$RUN/.stop" --interval 1.0 \
  --match component_container --match n4_meter.py --match nano_detector.py --match build/bench \
  --match rolling_nav2_smoke.py --match ros_rolling_nav.py --match n4_consumer.py --match rgbd_odometry \
  --match rtabmap_slam/rtabmap --match velocity_guard.py --match controller_server --match planner_server \
  --match bt_navigator --match behavior_server --match smoother_server --match velocity_smoother \
  --match waypoint_follower --match lifecycle_manager --match "ros2 launch" --match tegrastats --match _ros2_daemon \
  --match containerd-shim --match dockerd --match tum_rgbd_pub.py --match static_transform_publisher & PIDS+=($!)
trap 'fail "signal"' INT TERM
sleep 3
# ---------------- L0
layer L0 "$WIN0"
# ---------------- L1: container + driver + meter
rm -rf $TAP; mkdir -p $TAP; chmod 777 $TAP
docker rm -f $NAME > /dev/null 2>&1
docker run -d --name $NAME --net=host --cpuset-cpus 0-3 --shm-size 1g --device "$BUSDEV" \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_DOMAIN_ID=224 -e ROS_LOCALHOST_ONLY=1 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/n4/code/tools/nano_ros2/fastdds_shm.xml -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONUNBUFFERED=1 \
  -v "$N4:/n4" -v "$TAP:/tap" -v "$HOME/fpga/n2/data:/data:ro" --entrypoint sleep $IMAGE infinity > "$RUN/docker_run.txt" 2>&1 || fail "docker run"
CPID=$(docker inspect -f '{{.State.Pid}}' $NAME); echo "container_pid=$CPID" >> "$RUN/docker_run.txt"
R=/n4/runs/$2
ph "L1 launch"
dx "ros2 launch orbbec_camera gemini2.launch.py color_width:=640 color_height:=480 color_fps:=30 color_format:=RGB depth_width:=640 depth_height:=400 depth_fps:=30 depth_format:=Y16 enable_ir:=false enable_point_cloud:=false depth_registration:=true align_mode:=HW > $R/driver.log 2>&1"
sleep 8
dx "python3 tools/nano_fullload/n4_meter.py --out $R/meter --ring /tap/color.ring > $R/meter.log 2>&1"
for i in $(seq 1 60); do [ -e $TAP/color.ring ] && break; sleep 1; done
[ -e $TAP/color.ring ] || fail "no colour frames in 60 s (driver)"
guard "$SETTLE"; layer L1 "$WIN"; check L1
# ---------------- L2: detector on cores 4-5 from the ring
ph "L2 launch"
PYTHONPATH=$HOME/fpga/p2/pylib taskset -c 4,5 python3 $T/nano_detector/nano_detector.py --engine "$ENGINE" \
  --source shm:$TAP/color.ring --mode paced --hz 15 --seconds 100000 --warmup-s 5 \
  --out "$RUN/det.jsonl" --summary "$RUN/det_summary.json" > "$RUN/det.log" 2>&1 & DETP=$!
guard "$SETTLE"; layer L2 "$WIN"; check L2
# ---------------- L3: B4 + rolling producer + bridge + consumer
ph "L3 launch"
env OMP_NUM_THREADS=4 OMP_WAIT_POLICY=passive OMP_PROC_BIND=close OMP_PLACES=cores taskset -c 0-3 stdbuf -oL "$BENCH" \
  --depth $PREP/depth_u16.bin --width 640 --height 360 --frames 310 \
  --fx 344.781311 --fy 344.74472 --cx 322.479797 --cy 180.431702 --unit-m 0.001 --grid p32 \
  --ik $(python3 -c "import json;print(','.join(json.load(open('$PREP/meta.json'))['inverse_k_f32_hex']))") \
  --no-object --arm opt --passes "$B4_PASSES" --warmup 0 --pace-hz 15 --csv "$RUN/b4.csv" > "$RUN/b4.stdout" 2> "$RUN/b4.stderr" & B4P=$!
( until grep -q comparator_selftest "$RUN/b4.stdout" 2>/dev/null; do sleep 0.2; done; ph "b4_loop_start" ) & PIDS+=($!)
dx "python3 host/ros_rolling_nav.py --ros-args -p ground_z_m:=0.0 -p robot_height_m:=0.25 -p robot_radius_m:=0.15 -p cold_start_width_m:=0.22 -p cold_start_log_path:=$R/cold_start_log.jsonl > $R/bridge.log 2>&1"
mkdir -p "$RUN/scene" "$RUN/consumer"
dx "FPGA_SCENE_NO_SNAPSHOT=1 python3 robot_sim/tests/rolling_nav2_smoke.py scene --out $R/scene --seconds 100000 > $R/scene.log 2>&1"
dx "python3 tools/nano_fullload/n4_consumer.py --out $R/consumer > $R/consumer.log 2>&1"
guard "$SETTLE"; layer L3 "$WIN"; check L3
# ---------------- L4: RTAB-Map on a TUM fr1_desk ping-pong replay (0.1x, ~3 pairs/s; DetectionRate 20 in data time = 2 Hz wall)
ph "L4 launch"
mkdir -p "$RUN/rtabmap"
cat > "$RUN/rtabmap/odom.yaml" <<'YAML'
/**:
  ros__parameters:
    frame_id: tum_base
    odom_frame_id: vo_odom
    approx_sync: false
    publish_tf: true
YAML
cat > "$RUN/rtabmap/rtabmap.yaml" <<YAML
/**:
  ros__parameters:
    frame_id: tum_base
    map_frame_id: vo_map
    subscribe_depth: true
    subscribe_rgb: true
    subscribe_odom_info: true
    approx_sync: false
    database_path: "$R/rtabmap/n4.db"
    Rtabmap/DetectionRate: "20"
    RGBD/LinearUpdate: "0"
    RGBD/AngularUpdate: "0"
    Mem/STMSize: "30"
YAML
RMAP="-r __ns:=/vo -r rgb/image:=/fpga/rgb -r depth/image:=/fpga/depth -r rgb/camera_info:=/fpga/camera_info"
dx "/opt/ros/humble/lib/tf2_ros/static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id tum_base --child-frame-id tum_optical_frame > $R/rtabmap/tf.log 2>&1"
dx "/opt/ros/humble/lib/rtabmap_odom/rgbd_odometry --ros-args $RMAP --params-file $R/rtabmap/odom.yaml > $R/rtabmap/odom.log 2>&1"
dx "/opt/ros/humble/lib/rtabmap_slam/rtabmap --ros-args $RMAP --params-file $R/rtabmap/rtabmap.yaml > $R/rtabmap/rtabmap.log 2>&1"
sleep 3
dx "python3 tools/nano_ros2/tum_rgbd_pub.py /data/rgbd_dataset_freiburg1_desk --out $R/rtabmap/replay.jsonl --speed 0.1 ${REPLAY_RANGE:---pingpong 12} --info-topic /vo/info --no-subscribe --frame-id tum_optical_frame > $R/rtabmap/replay.log 2>&1"
guard "$SETTLE"; layer L4 "$WIN"; check L4
# ---------------- L5: Nav2 + velocity guard + goals
ph "L5 launch"
dx "python3 robot_sim/tests/rolling_nav2_smoke.py params --out $R/nav2_params.yaml > $R/nav2_params.log 2>&1"
sleep 5
dx "python3 robot_sim/fpga_sim/scripts/velocity_guard.py > $R/guard.log 2>&1"
dx "ros2 launch nav2_bringup navigation_launch.py params_file:=$R/nav2_params.yaml use_sim_time:=False autostart:=True > $R/nav2.log 2>&1"
sleep 15; touch "$RUN/consumer/goals_on"; ph "goals_on"
guard $(( SETTLE - 20 )); layer L5 "$WIN5"; check L5 || true
finish
