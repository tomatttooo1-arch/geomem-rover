#!/bin/bash
# Cold-start crawl check, one closed-loop run in a container (docs/design/NAV_COLDSTART_CRAWL_20261017.md).
#   bash tools/nav_coldstart_crawl/run_sim.sh RUN_DIR [--box x0,x1,y0,y1,z1 ...]
# WARMUP_S (5), GOAL_X (1.0), TIMEOUT_S (45), CPU_THREADS (4), CRAWL_CAM_HEIGHT_M / _PITCH_DEG / _ROLL_DEG (unset =
# the ng3 fit of crawl_lib; 10-17 tilt 10 deg runs set them).
# No hub, no board, no GPU: --network none (loopback only), so no host interface can listen on any port.
# One car on this host at a time: refuses when CAR_STATE_DIR has entries or a sim/car container runs.
# Never pkill -f: the container is removed by name, processes inside are stopped by pid.
set -u
RUN=${1:?RUN_DIR}; shift
case "$RUN" in /*) ;; *) echo "RUN_DIR must be absolute" >&2; exit 2;; esac
[ -e "$RUN" ] && { echo "RUN_DIR exists: $RUN" >&2; exit 2; }
TAG=$(basename "$RUN")
[[ "$TAG" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "bad tag" >&2; exit 2; }
REPO=$(cd "$(dirname "$0")/../.." && pwd)
: "${CPUSET:=36,37,39-47}" "${IMAGE:=fpga-dev1-sim:humble}" "${WARMUP_S:=5}" "${GOAL_X:=1.0}" "${TIMEOUT_S:=45}"
: "${CPU_THREADS:=4}" "${CAR_STATE_DIR:=${FPGA_HOME}/dev1/.t/wde}" "${CAR_HUB_BIND:=127.0.0.1}"
export CAR_HUB_BIND CAR_STATE_DIR
BOXES=()
while [ $# -gt 0 ]; do
  case "$1" in --box) BOXES+=(--box "$2"); shift 2;; *) echo "unknown arg $1" >&2; exit 2;; esac
done
# one car at a time
if [ -d "$CAR_STATE_DIR" ] && [ -n "$(ls -A "$CAR_STATE_DIR")" ]; then echo "CAR_STATE_DIR not empty: another car" >&2; exit 3; fi
RUNNING=$(docker ps --format '{{.Names}}' | grep -E '^(wd-sim-|wde-car-|fpga-dev1-rolling-|fpga-dev1-crawl-)' || true)
[ -n "$RUNNING" ] && { echo "another car/sim container runs: $RUNNING" >&2; exit 3; }
mkdir -p "$RUN"
(cd "$REPO" && git ls-files -co --exclude-standard -z -- host robot_sim tools tests perception hw \
  | tar --null -T - -cf "$RUN/src.tar")
{ echo "head $(git -C "$REPO" rev-parse HEAD)"; git -C "$REPO" status --porcelain; sha256sum "$RUN/src.tar";
  echo "image $IMAGE $(docker image inspect -f '{{.Id}}' "$IMAGE")"; echo "cpuset $CPUSET"; echo "boxes ${BOXES[*]:-none}";
  echo "date $(date '+%F %T %z')"; } > "$RUN/source.txt"
uptime > "$RUN/host_load.txt"
HEAD=$(git -C "$REPO" rev-parse HEAD)
cat > "$RUN/inner.sh" <<INNER
#!/bin/bash
set +u
mkdir -p /tmp/src && tar -xf /run_out/src.tar -C /tmp/src && cd /tmp/src
source /opt/ros/humble/setup.bash
export PYTHONPATH=/tmp/src:\$PYTHONPATH ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=61 PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
O=/run_out
mark() { echo "\$(date +%s.%N) \$*" >> \$O/marks.txt; }
pids=()
stop_all() { for p in "\${pids[@]}"; do kill -TERM "\$p" 2>/dev/null; done; sleep 4; for p in "\${pids[@]}"; do kill -KILL "\$p" 2>/dev/null; done; }
mark start
python3 robot_sim/tests/nav_controller_variants.py --output \$O/controllers --source-revision $HEAD > \$O/controller-generation.log 2>&1 || { mark controllers_failed; exit 4; }
mkdir -p \$O/map
python3 - <<'PY'
w = h = 200
open('/run_out/map/free.pgm', 'wb').write(b'P5\n%d %d\n255\n' % (w, h) + bytes([254]) * (w * h))
open('/run_out/map/free.yaml', 'w').write('image: free.pgm\nresolution: 0.05\norigin: [-5.0, -5.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n')
PY
python3 host/ros_rolling_nav.py --ros-args -p ground_z_m:=0.0 -p robot_height_m:=0.31 -p robot_radius_m:=0.30 -p body_margin_m:=0.05 > \$O/bridge.log 2>&1 & pids+=(\$!)
python3 tools/nav_coldstart_crawl/sim_scene.py --out \$O --seconds 400 --cpu-threads $CPU_THREADS ${BOXES[*]} > \$O/scene.log 2>&1 & pids+=(\$!)
env FPGA_NAV2_OVERRIDES=\$O/controllers/dwb.yaml FPGA_NAV_AUTOSTART=false FPGA_NAV_USE_SIM_TIME=false \
    FPGA_NAV_STATIC_MAP_ODOM=true FPGA_NAV_SIM_CAMERA_TF=false FPGA_NAV_SPEED_LIMITS=car FPGA_NAV_VELOCITY_GUARD=true \
    FPGA_NAV_MAP_YAML=\$O/map/free.yaml FPGA_NAV_COSTMAP=rolling \
    ros2 launch robot_sim/fpga_sim/launch/navigation.launch.py > \$O/navigation.log 2>&1 & pids+=(\$!)
python3 tools/car_run/grid_probe.py --topic /fpga/rolling_map/grid --timeout-s 90 > \$O/grid_probe.log 2>&1 || { mark no_grid; stop_all; exit 4; }
mark ready_rolling_grid
timeout 150 python3 robot_sim/tests/nav_startup_probe.py --activate --all-services-first --out \$O/startup-probe.jsonl > \$O/startup-probe.log 2>&1 || { mark nav2_activation_failed; stop_all; exit 4; }
mark ready_nav2
for n in controller_server local_costmap/local_costmap velocity_smoother; do
  timeout 30 ros2 param dump /\$n > \$O/params_\$(echo \$n | tr / _).yaml 2>&1
done
mark params_dumped
python3 tools/nav_coldstart_crawl/cmd_recorder.py --out \$O > \$O/cmd_recorder.log 2>&1 & pids+=(\$!)
timeout 200 python3 tools/nav_coldstart_crawl/sim_driver.py --out \$O --warmup-s $WARMUP_S --goal-x $GOAL_X --timeout-s $TIMEOUT_S > \$O/driver.log 2>&1
echo "driver_rc=\$?" > \$O/driver_rc.txt
mark driver_done
stop_all
mark stopped
INNER
chmod +x "$RUN/inner.sh"
NAME=fpga-dev1-crawl-$TAG
docker rm -f "$NAME" >/dev/null 2>&1 || true
# the camera mount override (crawl_lib CRAWL_CAM_*; unset = the ng3 fit) reaches the scene node only through -e
CAM_ENV=()
for v in CRAWL_CAM_HEIGHT_M CRAWL_CAM_PITCH_DEG CRAWL_CAM_ROLL_DEG; do
  [ -n "${!v:-}" ] && CAM_ENV+=(-e "$v=${!v}")
done
echo "cam_env ${CAM_ENV[*]:-none}" >> "$RUN/source.txt"
docker run --rm --name "$NAME" --network none --cpuset-cpus "$CPUSET" --memory 8g --shm-size 512m --entrypoint bash \
  --user "$(id -u):$(id -g)" -e HOME=/tmp ${CAM_ENV[@]+"${CAM_ENV[@]}"} -v "$RUN":/run_out "$IMAGE" /run_out/inner.sh
rc=$?
echo "docker_rc=$rc" >> "$RUN/driver_rc.txt"
uptime >> "$RUN/host_load.txt"
docker ps -a --filter "name=$NAME" --format '{{.Names}}' > "$RUN/leftover_containers.txt"
rm -f "$RUN/src.tar"
exit $rc
