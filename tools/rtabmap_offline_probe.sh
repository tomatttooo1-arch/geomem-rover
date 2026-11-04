#!/usr/bin/env bash
# Run inside the isolated ROS Humble container; input has no ground truth.
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen
mode=${1:?normal or blank}
case "$mode" in normal|blank) ;; *) exit 2;; esac
input="/$mode/rgbd_dataset_freiburg1_desk"
output="/results/$mode"
test ! -e "$output"
test ! -e "$input/groundtruth.txt"
mkdir -p "$output"
dpkg-query -W > "$output/packages.txt"
env | sort | awk '/^(ROS_|RMW_|OMP_|OPENBLAS_|QT_)/' > "$output/environment.txt"
printf '%s\n' 'rtabmap-rgbd_dataset --Rtabmap/DetectionRate 2 --RGBD/LinearUpdate 0 --RGBD/AngularUpdate 0 --Mem/STMSize 30 --RGBD/CreateOccupancyGrid true --output OUTPUT INPUT' > "$output/command.txt"
set +e
timeout --signal=INT --kill-after=30s 1200s rtabmap-rgbd_dataset \
  --Rtabmap/DetectionRate 2 --RGBD/LinearUpdate 0 --RGBD/AngularUpdate 0 \
  --Mem/STMSize 30 --RGBD/CreateOccupancyGrid true \
  --output "$output" "$input" > "$output/algorithm.log" 2>&1
status=$?
set -e
printf '%s\n' "$status" > "$output/exit-code.txt"
exit "$status"
