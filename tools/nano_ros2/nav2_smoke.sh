#!/usr/bin/env bash
# Nav2 software smoke test inside fpga-ros:humble-nav-20260927 (no hardware).
# map_server (saved RTAB-Map grid) + static map->odom (stand-in for RTAB-Map localization)
# + fake_base.py (integrates cmd_vel) + nav2_bringup navigation_launch.py, then one NavigateToPose goal.
# Usage: nav2_smoke.sh MAP_YAML OUT_DIR START_X START_Y GOAL_X GOAL_Y
source /opt/ros/humble/setup.bash
set -u
map=$1; out=$2; sx=$3; sy=$4; gx=$5; gy=$6
here=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$out"
python3 "$here/nav2_smoke_params.py" /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml "$out/nav2_params.yaml" > "$out/params.log" 2>&1
/opt/ros/humble/lib/nav2_map_server/map_server --ros-args -p yaml_filename:="$map" -p use_sim_time:=false > "$out/map_server.log" 2>&1 &
p1=$!
/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager --ros-args -r __node:=lifecycle_manager_map \
  -p node_names:="['map_server']" -p autostart:=true > "$out/lcm_map.log" 2>&1 &
p2=$!
/opt/ros/humble/lib/tf2_ros/static_transform_publisher --frame-id map --child-frame-id odom > "$out/tf_map_odom.log" 2>&1 &
p3=$!
python3 "$here/fake_base.py" --x "$sx" --y "$sy" --log "$out/fake_base_cmd.jsonl" > "$out/fake_base.log" 2>&1 &
p4=$!
ros2 launch nav2_bringup navigation_launch.py params_file:="$out/nav2_params.yaml" use_sim_time:=false autostart:=true > "$out/nav2.log" 2>&1 &
p5=$!
t0=$(date +%s.%N)
python3 "$here/nav2_goal.py" --x "$gx" --y "$gy" --out "$out/goal.json" --timeout 120 > "$out/goal.log" 2>&1
grc=$?
echo "start_wall=$t0 goal_rc=$grc" > "$out/exit.txt"
ros2 node list > "$out/nodes.txt" 2>&1
kill -INT $p5 $p4 $p3 $p2 $p1 2>/dev/null
# patterns name the ROS binaries only, so this script itself is not matched
sleep 5; pkill -INT -f "/opt/ros/humble/lib/nav2_" 2>/dev/null; sleep 2
pkill -9 -f "/opt/ros/humble/lib/nav2_|fake_base.py|static_transform_publisher|ros2 launch nav2_bringup" 2>/dev/null
[ "$grc" = 0 ] && echo DONE > "$out/status" || echo FAILED > "$out/status"
