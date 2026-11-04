#!/bin/bash
# Inspect the driver image (no camera, no network): build tools, driver launch arguments, QoS defaults.
# Repo copy (2026-10-14); image name fpga-ros:humble-nav-orbbec-scipy-20261017 as on the Nano.
IMG=fpga-ros:humble-nav-orbbec-scipy-20261017
docker run --rm --name fpga-dev1-inspect --network none --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog \
  --entrypoint bash $IMG -c '
source /opt/ros/humble/setup.bash; source /opt/orbbec_ws/install/setup.bash
echo "--- tools"; for t in g++ gcc cmake colcon make stress stress-ng pidstat top taskset chrt python3; do printf "%s=%s " $t "$(command -v $t || echo NO)"; done; echo
echo "--- rclcpp headers"; ls -d /opt/ros/humble/include/rclcpp /opt/ros/humble/include/sensor_msgs 2>&1
echo "--- rmw"; echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"; ls /opt/ros/humble/lib | grep -E "librmw_.*\.so" | head
echo "--- orbbec ws"; ls /opt/orbbec_ws; ls /opt/orbbec_ws/install; ls /opt/orbbec_ws/src 2>&1 | head
echo "--- launch file"; f=$(find /opt/orbbec_ws/install -name gemini2.launch.py | head -1); echo $f
echo "--- launch args"; ros2 launch orbbec_camera gemini2.launch.py --show-args 2>&1 | grep -E "^    .[a-z_0-9]+.:|default:" | paste - - | sed "s/  */ /g" | head -150
echo "--- src present?"; find / -name "ob_camera_node.cpp" -not -path "*/proc/*" 2>/dev/null | head
echo "--- driver libs"; find /opt/orbbec_ws/install -name "*.so*" | head -20
'
