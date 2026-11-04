#!/usr/bin/env bash
# Host-side wrapper on the Nano: Nav2 smoke in the nav image, cores 0-3, with CPU/RSS sampling.
# Usage: run_nav2_smoke.sh LABEL MAP_YAML(relative to ~/fpga/n2) START_X START_Y GOAL_X GOAL_Y
set -u
label=$1; map=$2; sx=$3; sy=$4; gx=$5; gy=$6
N2=$HOME/fpga/n2; RUN=$N2/runs/$label
mkdir -p "$RUN"; test ! -e "$RUN/status" || exit 2
tegrastats --interval 500 --logfile "$RUN/tegrastats.log" & TS=$!
python3 "$N2/tools/proc_sampler.py" --out "$RUN/procs.jsonl" --stop-file "$RUN/.stop" \
  --match controller_server --match planner_server --match bt_navigator --match behavior_server \
  --match smoother_server --match velocity_smoother --match waypoint_follower --match map_server \
  --match fake_base.py --match nav2_goal.py & SP=$!
echo "$(date +%s.%N) start" > "$RUN/phases.txt"
docker run --rm --name "fpga-n2-$label" --net=host --cpuset-cpus 0-3 --shm-size 1g \
  -e ROS_DOMAIN_ID=216 -e ROS_LOCALHOST_ONLY=1 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e PYTHONDONTWRITEBYTECODE=1 -e HOME=/tmp -v "$N2:/work" -w /work --entrypoint bash fpga-ros:humble-nav-20260927 \
  -c "/work/tools/nav2_smoke.sh /work/$map /work/runs/$label/nav $sx $sy $gx $gy; chown -R $(id -u):$(id -g) /work/runs/$label" \
  > "$RUN/docker.log" 2>&1
echo "$(date +%s.%N) end" >> "$RUN/phases.txt"
touch "$RUN/.stop"; wait "$SP"; kill "$TS"
grep -q DONE "$RUN/nav/status" 2>/dev/null && echo DONE > "$RUN/status" || echo FAILED > "$RUN/status"
