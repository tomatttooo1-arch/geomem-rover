#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 3 ]]; then
    echo "Usage: bash tools/run_service_response_match_control.sh ROOTFS PROJECT NEW_OUT_DIR" >&2; exit 2
fi
rootfs=$(realpath -e -- "$1"); project=$(realpath -e -- "$2")
parent=$(realpath -e -- "$(dirname -- "$3")"); out="$parent/$(basename -- "$3")"
[[ -d "$rootfs/opt/ros/humble" && -x "$rootfs/bin/bash" ]] || { echo "invalid ROS Humble rootfs" >&2; exit 2; }
[[ -f "$project/robot_sim/tests/service_response_match_control.cpp" ]] || { echo "invalid project" >&2; exit 2; }
[[ ! -e "$out" ]] || { echo "NEW_OUT_DIR exists: $out" >&2; exit 2; }
mkdir -p -- "$out/src/control"
cp -- "$project/robot_sim/tests/service_response_match_control.cpp" "$out/src/control/"
git -C "$project" rev-parse HEAD > "$out/source-revision.txt"
(cd "$out" && sha256sum src/control/service_response_match_control.cpp > source.sha256)
cat > "$out/src/control/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.8)
project(service_response_match_control)
find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_fastrtps_shared_cpp REQUIRED)
find_package(std_msgs REQUIRED)
add_executable(service_response_match_control service_response_match_control.cpp)
ament_target_dependencies(service_response_match_control lifecycle_msgs rclcpp rmw_fastrtps_shared_cpp std_msgs)
install(TARGETS service_response_match_control DESTINATION lib/${PROJECT_NAME})
ament_package()
CMAKE
cat > "$out/src/control/package.xml" <<'XML'
<?xml version="1.0"?>
<package format="3"><name>service_response_match_control</name><version>0.1.0</version>
<description>Native rmw_fastrtps response matching control.</description>
<maintainer email="dev2@example.local">dev2</maintainer><license>Apache-2.0</license>
<buildtool_depend>ament_cmake</buildtool_depend><depend>lifecycle_msgs</depend><depend>rclcpp</depend>
<depend>rmw_fastrtps_shared_cpp</depend><depend>std_msgs</depend><export><build_type>ament_cmake</build_type></export></package>
XML
export FPGA_AGX_WORKSPACE="$out" FPGA_DDS_TRANSPORT=udp ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
FPGA_SERVICE_REPLY_PROFILE=legacy bash "$project/tools/agx_rootfs.sh" "$rootfs" "$project" \
  dpkg-query -W '-f=${binary:Package}\t${Version}\n' \
  ros-humble-rclcpp ros-humble-rmw-fastrtps-cpp ros-humble-rmw-fastrtps-shared-cpp \
  ros-humble-fastrtps ros-humble-fastcdr > "$out/dependency-versions.txt"
FPGA_SERVICE_REPLY_PROFILE=legacy bash "$project/tools/agx_rootfs.sh" "$rootfs" "$project" \
  colcon --log-base /workspace/log build --base-paths /workspace/src --build-base /workspace/build \
  --install-base /workspace/install --packages-select service_response_match_control 2>&1 | tee "$out/build.log"
for mode in legacy bounded; do
    expected=100; [[ "$mode" == bounded ]] && expected=1000
    FPGA_SERVICE_REPLY_PROFILE="$mode" bash "$project/tools/agx_rootfs.sh" "$rootfs" "$project" \
      bash -c 'source /workspace/install/setup.bash; exec /workspace/install/service_response_match_control/lib/service_response_match_control/service_response_match_control "$1"' \
      control "$expected" > "$out/$mode.jsonl" 2> "$out/$mode.stderr"
done
(cd "$out" && sha256sum source-revision.txt source.sha256 dependency-versions.txt build.log \
  legacy.jsonl legacy.stderr bounded.jsonl bounded.stderr > MANIFEST.sha256)
echo "control passed: $out"
