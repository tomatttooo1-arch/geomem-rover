#!/bin/bash
# Container smoke for the rolling-map Nav2 layer (docs/design/ROLLING_NAV2_LAYER_20260927.md).
# usage: [FPGA_SMOKE_S0=1] tools/rolling_nav2_smoke.sh <out_dir> [tag]
# FPGA_SMOKE_S0=1: control run, check costmap with track_unknown_space false, stops after S0.
# Server only: ROS 2 Humble image with Nav2, --network none, no GPU/devices, removed on exit.
set -euo pipefail
OUT=$(realpath -m "$1"); TAG=${2:-smoke}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
IMAGE=${FPGA_ROLLING_SMOKE_IMAGE:-fpga-dev1-rtabmap:humble-0.23.7-20260910}
NAME=fpga-dev1-rolling-nav2-$TAG
mkdir -p "$OUT"
TAR="$OUT/src.tar"
(cd "$ROOT" && git ls-files -co --exclude-standard -z -- host robot_sim tools tests hw perception \
   | tar --null -T - -cf "$TAR")
{ echo "head $(git -C "$ROOT" rev-parse HEAD)"; git -C "$ROOT" status --porcelain;
  sha256sum "$TAR"; echo "image $IMAGE $(docker image inspect -f '{{.Id}}' "$IMAGE")"; } > "$OUT/source.txt"
cat > "$OUT/inner.sh" <<'INNER'
#!/bin/bash
set +u
mkdir -p /tmp/src && tar -xf /smoke/out/src.tar -C /tmp/src && cd /tmp/src
source /opt/ros/humble/setup.bash
export PYTHONPATH=/tmp/src:$PYTHONPATH ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=88 PYTHONUNBUFFERED=1
O=/smoke/out
python3 robot_sim/tests/rolling_nav2_smoke.py params --out /tmp/params.yaml || exit 2
cp /tmp/params.yaml $O/nav2_params_effective.yaml
grep -l -a "Map update ignored" /opt/ros/humble/lib/*.so 2>/dev/null > $O/static_layer_update_warning_present.txt
pids=()
python3 host/ros_rolling_nav.py --ros-args -p ground_z_m:=0.0 -p robot_height_m:=0.25 -p robot_radius_m:=0.15 -p cold_start_width_m:=0.22 -p cold_start_log_path:=$O/cold_start_log.jsonl > $O/bridge.log 2>&1 & pids+=($!)
python3 robot_sim/tests/rolling_nav2_smoke.py scene --out $O --seconds 400 > $O/scene.log 2>&1 & pids+=($!)
python3 robot_sim/fpga_sim/scripts/velocity_guard.py > $O/guard.log 2>&1 & pids+=($!)
ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args --params-file /tmp/params.yaml > $O/check_costmap.log 2>&1 & pids+=($!)
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=check_lifecycle --params-file /tmp/params.yaml > $O/check_lifecycle.log 2>&1 & pids+=($!)
ros2 launch nav2_bringup navigation_launch.py params_file:=/tmp/params.yaml use_sim_time:=False autostart:=True > $O/nav2.log 2>&1 & pids+=($!)
timeout 300 python3 robot_sim/tests/rolling_nav2_smoke.py driver --out $O > $O/driver.log 2>&1
rc=$?
echo "driver_rc=$rc" > $O/driver_rc.txt
ros2 topic list > $O/topics.txt 2>&1
cat /sys/fs/cgroup/cpu.stat /sys/fs/cgroup/cpu/cpu.stat > $O/cgroup_cpu_stat.txt 2>/dev/null
kill -INT "${pids[@]}" 2>/dev/null; sleep 3; kill -9 "${pids[@]}" 2>/dev/null
pkill -9 -f nav2 2>/dev/null; pkill -9 -f rolling 2>/dev/null
exit $rc
INNER
chmod +x "$OUT/inner.sh"
docker rm -f "$NAME" >/dev/null 2>&1 || true
uptime > "$OUT/host_load.txt"
set +e
docker run --rm --name "$NAME" --network none --cpus 4 --memory 6g --entrypoint bash \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e FPGA_SMOKE_S0="${FPGA_SMOKE_S0:-0}" -e FPGA_SMOKE_CLEARING="${FPGA_SMOKE_CLEARING:-0}" -e FPGA_SMOKE_CONFIRM_FIRST="${FPGA_SMOKE_CONFIRM_FIRST:-0}" \
  -v "$OUT":/smoke/out "$IMAGE" /smoke/out/inner.sh
rc=$?
set -e
echo "docker_rc=$rc" >> "$OUT/driver_rc.txt"
uptime >> "$OUT/host_load.txt"
docker ps -a --filter "name=$NAME" --format '{{.Names}}' > "$OUT/leftover_containers.txt"
exit $rc
