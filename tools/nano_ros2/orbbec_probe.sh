#!/usr/bin/env bash
# Inside fpga-ros:humble-nav-orbbec-scipy-20261017: start the Gemini 2 driver, measure topics, stop.
# Usage: orbbec_probe.sh OUT_DIR [extra launch args...]
source /opt/ros/humble/setup.bash
source /opt/orbbec_ws/install/setup.bash
set -u
out=$1; shift
here=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$out"
ros2 launch orbbec_camera gemini2.launch.py "$@" > "$out/driver.log" 2>&1 &
lp=$!
python3 "$here/topic_rate_probe.py" --warmup 12 --seconds 30 --out "$out/rates.json" \
  /camera/color/image_raw:image /camera/depth/image_raw:image /camera/depth/points:points \
  /camera/color/camera_info:info /camera/depth/camera_info:info > "$out/probe.log" 2>&1
prc=$?
ros2 topic list > "$out/topics.txt" 2>&1
ros2 param dump /camera/camera > "$out/params.yaml" 2>&1
kill -INT $lp; sleep 5; pkill -INT -f component_container; sleep 2; pkill -9 -f component_container
[ "$prc" = 0 ] && echo DONE > "$out/status" || echo FAILED > "$out/status"
