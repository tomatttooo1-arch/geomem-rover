#!/bin/bash
# CTRLPS2 mock run (host container, network none): goal -> G1 route -> KV commands -> guard -> chassis stand-in.
# Pieces: kv_server --core null --nav soft --nav-core mock (the KV), navigation.launch.py with FPGA_NAV_BACKEND=kv
# (map_server + planner_server + velocity guard; static map->odom identity), nav_startup_probe with
# FPGA_NAV_PROBE_NODES, host.ros_kv_nav_proxy --mode kv, sim_chassis.py, goal_client.py (the runner's goal).
set +u
: "${O:=/o}" "${GOAL:=2.0 -1.5 0.0}" "${GOAL_TIMEOUT_S:=120}" "${MODE:=kv}"
# MODE=shadow: the full Nav2 drives (navigation.launch.py as on the nav2 path), the proxy only records
source /opt/ros/humble/setup.bash; source /opt/fpga_ws/install/setup.bash 2>/dev/null || true
export PYTHONPATH=/src:$PYTHONPATH PYTHONUNBUFFERED=1 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/roslog HOME=/tmp
cd /src
PIDS=()
bg() { local out=$1; shift; "$@" > "$O/$out" 2>&1 & PIDS+=($!); }
cleanup() { for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done; sleep 2; for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null; done; }
trap cleanup EXIT
bg kv.json python3 tools/kv_link/kv_server.py --core null --nav soft --nav-core mock --port 47811 --max-idle-s 600 --out "$O/kv"
NB=(FPGA_NAV_BACKEND=kv); PN=(FPGA_NAV_PROBE_NODES=map_server,planner_server)
[ "$MODE" = shadow ] && { NB=(); PN=(); }
bg navigation.log env "${NB[@]}" FPGA_NAV_AUTOSTART=false FPGA_NAV_USE_SIM_TIME=false FPGA_NAV_STATIC_MAP_ODOM=true \
  FPGA_NAV_SIM_CAMERA_TF=false FPGA_NAV_VELOCITY_GUARD=true ros2 launch fpga_sim navigation.launch.py
bg chassis.log python3 tests/kv_nav_mock/sim_chassis.py "$O/chassis.jsonl"
sleep 3
env "${PN[@]}" timeout 120 python3 robot_sim/tests/nav_startup_probe.py --activate \
  --all-services-first --out "$O/startup-probe.jsonl" > "$O/startup-probe.log" 2>&1; echo "probe rc=$?"
bg kv_nav_proxy.log python3 -m host.ros_kv_nav_proxy --kv-server 127.0.0.1:47811 --mode "$MODE" --record "$O/kv_nav_cmds.jsonl"
for _ in $(seq 100); do grep -q '"event": "ready"' "$O/kv_nav_proxy.log" && break; sleep 0.2; done
grep -o '"event": "[a-z]*"' "$O/kv_nav_proxy.log" | head -2
timeout $((GOAL_TIMEOUT_S + 40)) python3 tests/kv_nav_mock/goal_client.py $GOAL "$GOAL_TIMEOUT_S" > "$O/goal_result.json" 2> "$O/goal_client.err"
echo "goal rc=$?"; cat "$O/goal_result.json"
