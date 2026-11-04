#!/usr/bin/env bash
# Host-side measured run on the Nano (run with nohup; writes $RUN/status = DONE|FAILED).
# Usage: measure_rtabmap.sh LABEL MODE SEQ DB [START STOP]
#   MODE: map | loc | none (none = no RTAB-Map, detector only)
# Env:   WITH_DET=1  also run the YOLOv8n-seg detector (needs the camera lock), pinned to cores 4-5.
#        SPEED (TUM playback speed, default 1.0), ODOM_EXTRA (extra rgbd_odometry yaml lines)
#        DDS_PROFILE (file name under tools/, e.g. fastdds_shm.xml; empty = Fast DDS defaults)
#        DET_SECONDS (default 75), IDLE_S (default 15), IMAGE (default fpga-ros:humble-nav-20260927)
# RTAB-Map container is pinned to cores 0-3 (cluster 0); detector to 4-5 (cluster 1).
set -u
label=$1; mode=$2; seq=${3:-}; db=${4:-}; start=${5:-0}; stop=${6:-}
N2=$HOME/fpga/n2
RUN=$N2/runs/$label
IMAGE=${IMAGE:-fpga-ros:humble-nav-20260927}
IDLE_S=${IDLE_S:-15}; DET_SECONDS=${DET_SECONDS:-75}
mkdir -p "$RUN"; test ! -e "$RUN/status" || exit 2
ph() { echo "$(date +%s.%N) $*" >> "$RUN/phases.txt"; }
{ date -Is; df -B1 / | tail -1; free -b; uptime; cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq | tr '\n' ' '; echo; } > "$RUN/env.txt"
tegrastats --interval 500 --logfile "$RUN/tegrastats.log" & TS=$!
python3 "$N2/tools/proc_sampler.py" --out "$RUN/procs.jsonl" --stop-file "$RUN/.stop" \
  --match rgbd_odometry --match rtabmap_slam/rtabmap --match tum_rgbd_pub.py --match nano_detector.py \
  --match static_transform_publisher --match rtabmap_sync/rgbd_sync & SP=$!
ph idle_start; sleep "$IDLE_S"; ph idle_end
DP=
if [ "${WITH_DET:-0}" = 1 ]; then
  ph det_start
  PYTHONPATH=$HOME/fpga/p2/pylib taskset -c 4,5 python3 $HOME/fpga/p2/code/nano_detector.py \
    --engine $HOME/fpga/p2/w/yolov8n-seg-640-fp16.engine --mode paced --hz 15 --seconds "$DET_SECONDS" \
    --out "$RUN/det.jsonl" --summary "$RUN/det.json" > "$RUN/det.log" 2>&1 & DP=$!
  sleep 10
fi
rc=0
if [ "$mode" != none ]; then
  ph slam_start
  docker run --rm --name "fpga-n2-$label" --net=host --cpuset-cpus 0-3 --shm-size 1g \
    ${DDS_PROFILE:+-e FASTRTPS_DEFAULT_PROFILES_FILE=/work/tools/$DDS_PROFILE} \
    -e ROS_DOMAIN_ID=215 -e ROS_LOCALHOST_ONLY=1 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e PYTHONDONTWRITEBYTECODE=1 -e HOME=/tmp -e SPEED=${SPEED:-1.0} -e SYNC=${SYNC:-separate} -e CAM="${CAM:-}" -e ODOM_EXTRA="${ODOM_EXTRA:-}" \
    -v "$N2:/work" -w /work --entrypoint bash "$IMAGE" \
    -c "/work/tools/rtabmap_tum_run.sh $mode /work/data/$seq /work/runs/$label/slam /work/$db $start $stop; chown -R $(id -u):$(id -g) /work/runs/$label /work/dbs" \
    > "$RUN/docker.log" 2>&1
  rc=$?
  ph slam_end
fi
if [ -n "$DP" ]; then wait "$DP"; ph det_end; fi
sleep 3
touch "$RUN/.stop"; wait "$SP"; kill "$TS"; wait "$TS" 2>/dev/null
df -B1 / | tail -1 >> "$RUN/env.txt"
if [ "$rc" = 0 ] && { [ "$mode" = none ] || grep -q DONE "$RUN/slam/status" 2>/dev/null; }; then
  echo DONE > "$RUN/status"; else echo FAILED > "$RUN/status"; fi
