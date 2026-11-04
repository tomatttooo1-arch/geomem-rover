#!/bin/bash
# N5 full load in the real partition (first-stage topology: camera on the Nano) on the Orin Nano.
# docs/design/NANO_PARTITION_LOAD_20260927.md. Host side, run detached with setsid nohup.
#   run_partition.sh <n5_dir> <run_name> <run_base>
# <n5_dir>/code holds the packed tree (host robot_sim tools hw perception + COMMIT) and <n5_dir>/n5_deltas.bin
# (tools/nano_partition/n5_map_replay.py record). Results go to <run_base>/<run_name>:
#   arm B: run_base on tmpfs (/dev/shm/fpga_n5/runs) -> RTAB-Map database and every log in RAM
#   arm C: run_base on the SD card (<n5_dir>/runs)   -> as in N4
# The stall probe always writes to tmpfs (/dev/shm/fpga_n5_probe) and is copied into the run at the end.
# Layers (cumulative): L0 idle, L1 driver+meter, L2 +detector, L3 +map replay (PL stand-in) + virtual base
# + bridge + consumer, L4 +RTAB-Map on the TUM fr1_desk ping-pong replay (as N4 main3), L5 +Nav2 with goals.
# Placement (design doc §2.2): container cpuset 0-5; detector, rgbd_odometry, rtabmap on cores 4-5;
# map replay + virtual base on core 3 only; everything else on cores 0-3.
# After every layer tools/nano_fullload/check_layer.py applies the N4 criteria; on FAIL the run stops there.
# Env: WIN (300) WIN0 (WIN) WIN5 (600) SETTLE (60) LOWWIN / LOWSETTLE (window / settle of L0-L3, default
# WIN / SETTLE; arm C uses short ones) IOCTL (1 = SD-card write positive control before L0).
# Writes <run>/DONE or <run>/FAILED (reason). Needs the orin-nano and camera locks (taken on the server).
set -u
N5=$1; RUNNAME=$2; RUN=$3/$2
WIN=${WIN:-300}; WIN5=${WIN5:-600}; SETTLE=${SETTLE:-60}
LOWWIN=${LOWWIN:-$WIN}; WIN0=${WIN0:-$LOWWIN}; LOWSETTLE=${LOWSETTLE:-$SETTLE}
C=$N5/code; T=$C/tools
IMAGE=fpga-ros:humble-nav-orbbec-scipy-20261017
NAME=fpga-n5-ros
TAP=/dev/shm/fpga_n5_tap
PROBEDIR=/dev/shm/fpga_n5_probe
ENGINE=$HOME/fpga/p2/w/yolov8n-seg-640-fp16.engine
C0=0-3; C1=4,5; CREP=3
mkdir -p "$RUN" "$PROBEDIR" || exit 2
test ! -e "$RUN/phases.log" || { echo "run exists" >&2; exit 2; }
PH=$RUN/phases.log
ph() { echo "$(date +%s.%N) $(awk '{print $1}' /proc/uptime) $*" >> "$PH"; }
PIDS=()
teardown() {
  ph teardown
  [ -n "${DETP:-}" ] && kill -TERM "$DETP" 2>/dev/null
  docker exec $NAME bash -c 'pkill -INT -f rtabmap_slam/rtabmap; pkill -INT -f rgbd_odometry; pkill -INT -f n4_consumer.py; pkill -INT -f n4_meter.py; pkill -INT -f navigation_launch; pkill -INT -f n5_map_replay.py; pkill -INT -f ros_rolling_nav.py; pkill -INT -f velocity_guard.py; pkill -INT -f tum_rgbd_pub.py; pkill -INT -f static_transform_publisher' 2>/dev/null
  sleep 6
  docker exec $NAME bash -c 'pkill -INT -f gemini2.launch; pkill -INT -f component_container' 2>/dev/null
  sleep 4
  docker exec $NAME bash -c "du -sb /n5/run/rtabmap/n4.db" >> "$RUN/env_end.txt" 2>&1
  docker rm -f $NAME > /dev/null 2>&1
  sleep 1; touch "$RUN/.stop" "$PROBEDIR/$RUNNAME.stop"; kill "${PIDS[@]}" 2>/dev/null; wait 2>/dev/null
  cp "$PROBEDIR/$RUNNAME.jsonl" "$RUN/probe.jsonl" 2>> "$RUN/env_end.txt"
  ls -la $TAP >> "$RUN/env_end.txt" 2>&1; rm -rf $TAP
  { date -Is; df -B1 / /dev/shm; free -b; docker ps -a --format '{{.Names}}'; } >> "$RUN/env_end.txt" 2>&1
}
fail() { echo "$*" > "$RUN/FAILED"; ph "FAILED $*"; teardown; exit 1; }
guard() {  # sleep $1 seconds while checking temperature, disk, RAM and the long-lived processes
  local end=$(( $(date +%s) + $1 ))
  while [ "$(date +%s)" -lt "$end" ]; do
    local tmax=$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -1)
    [ "${tmax:-0}" -ge 90000 ] && fail "overtemp ${tmax} mC"
    local avail=$(df -B1 / | awk 'NR==2{print $4}')
    [ "$avail" -lt 5000000000 ] && fail "disk ${avail} B"
    local used=$(awk '/^MemTotal:/{t=$2} /^MemAvailable:/{a=$2} END{print int((t-a)/1024)}' /proc/meminfo)
    [ "$used" -gt 7000 ] && fail "ram used ${used} MB > 7000"
    [ -n "${DETP:-}" ] && ! kill -0 "$DETP" 2>/dev/null && fail "detector exited"
    [ -n "${CPID:-}" ] && [ ! -d /proc/$CPID ] && fail "container gone"
    sleep 5
  done
}
layer() { ph "start $1"; guard "$2"; ph "end $1"; }
affinity() {  # record where every measured process is allowed to run (checks the pinning landed)
  { echo "== $1 $(date -Is)"
    for pat in component_container n4_meter.py nano_detector.py n5_map_replay.py ros_rolling_nav.py n4_consumer.py \
               rgbd_odometry rtabmap_slam/rtabmap tum_rgbd_pub.py velocity_guard.py controller_server planner_server \
               bt_navigator behavior_server smoother_server velocity_smoother waypoint_follower lifecycle_manager; do
      for p in $(pgrep -f "$pat"); do
        c=$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | cut -c1-90); case "$c" in *proc_sampler*|*run_partition*) continue;; esac
        echo "$pat $p $(awk '/^Cpus_allowed_list/{print $2}' /proc/$p/status 2>/dev/null) $c"; done
    done; } >> "$RUN/affinity.txt" 2>&1
}
check() {
  sleep 3; affinity "$1"
  local out; out=$(python3 $T/nano_fullload/check_layer.py "$RUN" "$1" 2>&1); local rc=$?
  ph "check $1 rc=$rc $out"
  [ $rc = 0 ] && return 0
  echo "$1 $out" > "$RUN/STOPPED_AT"; finish; exit 0
}
finish() { teardown; ph DONE; touch "$RUN/DONE"; }
dx() { local cpus=$1; shift; docker exec -d $NAME bash -c "source /opt/ros/humble/setup.bash; source /opt/orbbec_ws/install/setup.bash; cd /n5/code; export PYTHONPATH=/n5/code:\$PYTHONPATH; exec taskset -c $cpus bash -c '$*'"; }
{ date -Is; nvpmodel -q 2>/dev/null; uptime; df -B1 / /dev/shm; free -b; findmnt -T "$RUN"
  for p in 0 4; do d=/sys/devices/system/cpu/cpufreq/policy$p; echo "policy$p $(cat $d/scaling_governor) min=$(cat $d/scaling_min_freq) max=$(cat $d/scaling_max_freq)"; done
  cat /sys/class/devfreq/17000000.gpu/governor /sys/class/devfreq/17000000.gpu/max_freq 2>/dev/null
  cat $C/COMMIT; sha256sum "$ENGINE" "$N5/n5_deltas.bin"; docker image inspect -f '{{.Id}}' $IMAGE
  lsusb -d 2bc5:; ls /proc/pressure 2>&1; top -bn1 | head -15; } > "$RUN/env.txt" 2>&1
BUSDEV=$(lsusb -d 2bc5: | head -1 | awk '{printf "/dev/bus/usb/%s/%s", $2, substr($4,1,3)}')
[ -e "$BUSDEV" ] || { echo "no camera" > "$RUN/FAILED"; exit 1; }
rm -f "$PROBEDIR/$RUNNAME.stop"
python3 $T/nano_partition/stall_probe.py --out "$PROBEDIR/$RUNNAME.jsonl" --stop-file "$PROBEDIR/$RUNNAME.stop" & PIDS+=($!)
tegrastats --interval 500 --logfile "$RUN/tegrastats.log" & PIDS+=($!)
python3 $T/nano_ros2/proc_sampler.py --out "$RUN/procs.jsonl" --stop-file "$RUN/.stop" --interval 1.0 --seconds 10800 \
  --match component_container --match n4_meter.py --match nano_detector.py \
  --match "n5_map_replay.py replay" --match "n5_map_replay.py base" --match ros_rolling_nav.py --match n4_consumer.py \
  --match rgbd_odometry --match rtabmap_slam/rtabmap --match velocity_guard.py --match controller_server --match planner_server \
  --match bt_navigator --match behavior_server --match smoother_server --match velocity_smoother \
  --match waypoint_follower --match lifecycle_manager --match "ros2 launch" --match tegrastats --match _ros2_daemon \
  --match containerd-shim --match dockerd --match tum_rgbd_pub.py --match static_transform_publisher --match stall_probe.py & PIDS+=($!)
trap 'fail "signal"' INT TERM
sleep 3
# ---------------- probe positive control: 256 MB fsync'd write to the SD card (outside every window)
if [ "${IOCTL:-1}" = 1 ]; then
  ph "start IOCTL"; dd if=/dev/zero of="$N5/ioctl.bin" bs=1M count=256 conv=fsync 2> "$RUN/ioctl.txt"; rm -f "$N5/ioctl.bin"; sync; ph "end IOCTL"
  sleep 10
fi
# ---------------- L0
layer L0 "$WIN0"
# ---------------- L1: container + driver + meter (cores 0-3)
rm -rf $TAP; mkdir -p $TAP; chmod 777 $TAP
docker rm -f $NAME > /dev/null 2>&1
docker run -d --name $NAME --net=host --cpuset-cpus 0-5 --shm-size 1g --device "$BUSDEV" \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_DOMAIN_ID=225 -e ROS_LOCALHOST_ONLY=1 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/n5/code/tools/nano_ros2/fastdds_shm.xml -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONUNBUFFERED=1 \
  -v "$C:/n5/code:ro" -v "$N5/n5_deltas.bin:/n5/n5_deltas.bin:ro" -v "$RUN:/n5/run" -v "$TAP:/tap" -v "$HOME/fpga/n2/data:/data:ro" \
  --entrypoint sleep $IMAGE infinity > "$RUN/docker_run.txt" 2>&1 || fail "docker run"
CPID=$(docker inspect -f '{{.State.Pid}}' $NAME); echo "container_pid=$CPID" >> "$RUN/docker_run.txt"
R=/n5/run
ph "L1 launch"
dx $C0 "ros2 launch orbbec_camera gemini2.launch.py color_width:=640 color_height:=480 color_fps:=30 color_format:=RGB depth_width:=640 depth_height:=400 depth_fps:=30 depth_format:=Y16 enable_ir:=false enable_point_cloud:=false depth_registration:=true align_mode:=HW > $R/driver.log 2>&1"
sleep 8
dx $C0 "python3 tools/nano_fullload/n4_meter.py --out $R/meter --ring /tap/color.ring > $R/meter.log 2>&1"
for i in $(seq 1 60); do [ -e $TAP/color.ring ] && break; sleep 1; done
[ -e $TAP/color.ring ] || fail "no colour frames in 60 s (driver)"
guard "$LOWSETTLE"; layer L1 "$LOWWIN"; check L1
# ---------------- L2: detector on cores 4-5 from the ring (as N4)
ph "L2 launch"
PYTHONPATH=$HOME/fpga/p2/pylib taskset -c $C1 python3 $T/nano_detector/nano_detector.py --engine "$ENGINE" \
  --source shm:$TAP/color.ring --mode paced --hz 15 --seconds 100000 --warmup-s 5 \
  --out "$RUN/det.jsonl" --summary "$RUN/det_summary.json" > "$RUN/det.log" 2>&1 & DETP=$!
guard "$LOWSETTLE"; layer L2 "$LOWWIN"; check L2
# ---------------- L3: map replay (PL stand-in) + virtual base on core 3; bridge + consumer on cores 0-3
ph "L3 launch"
mkdir -p "$RUN/scene" "$RUN/consumer"
dx $C0 "python3 host/ros_rolling_nav.py --ros-args -p ground_z_m:=0.0 -p robot_height_m:=0.25 -p robot_radius_m:=0.15 -p cold_start_width_m:=0.22 -p cold_start_log_path:=$R/cold_start_log.jsonl > $R/bridge.log 2>&1"
dx $CREP "python3 tools/nano_partition/n5_map_replay.py base --out $R/scene > $R/base.log 2>&1"
dx $CREP "python3 tools/nano_partition/n5_map_replay.py replay --deltas /n5/n5_deltas.bin --out $R/scene > $R/scene.log 2>&1"
dx $C0 "python3 tools/nano_fullload/n4_consumer.py --out $R/consumer > $R/consumer.log 2>&1"
guard "$LOWSETTLE"; layer L3 "$LOWWIN"; check L3
# ---------------- L4: RTAB-Map (cores 4-5) on the TUM fr1_desk ping-pong replay (publisher on cores 0-3), as N4 main3
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
dx $C0 "/opt/ros/humble/lib/tf2_ros/static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id tum_base --child-frame-id tum_optical_frame > $R/rtabmap/tf.log 2>&1"
dx $C1 "/opt/ros/humble/lib/rtabmap_odom/rgbd_odometry --ros-args $RMAP --params-file $R/rtabmap/odom.yaml > $R/rtabmap/odom.log 2>&1"
dx $C1 "/opt/ros/humble/lib/rtabmap_slam/rtabmap --ros-args $RMAP --params-file $R/rtabmap/rtabmap.yaml > $R/rtabmap/rtabmap.log 2>&1"
sleep 3
dx $C0 "python3 tools/nano_ros2/tum_rgbd_pub.py /data/rgbd_dataset_freiburg1_desk --out $R/rtabmap/replay.jsonl --speed 0.1 --start 0 --stop 160 --pingpong 40 --info-topic /vo/info --no-subscribe --frame-id tum_optical_frame > $R/rtabmap/replay.log 2>&1"
guard "$SETTLE"; layer L4 "$WIN"; check L4
# ---------------- L5: Nav2 + velocity guard (cores 0-3) + goals
ph "L5 launch"
dx $C0 "python3 robot_sim/tests/rolling_nav2_smoke.py params --out $R/nav2_params.yaml > $R/nav2_params.log 2>&1"
sleep 5
dx $C0 "python3 robot_sim/fpga_sim/scripts/velocity_guard.py > $R/guard.log 2>&1"
dx $C0 "ros2 launch nav2_bringup navigation_launch.py params_file:=$R/nav2_params.yaml use_sim_time:=False autostart:=True > $R/nav2.log 2>&1"
sleep 15; touch "$RUN/consumer/goals_on"; ph "goals_on"
guard $(( SETTLE - 20 )); layer L5 "$WIN5"; check L5 || true
finish
