#!/bin/bash
# Line E (2026-10-14): the car's ROS processes, started in order, each with a readiness check, then kept
# running until a signal (no time limit). Run INSIDE the stack container (host: started by car_up.sh;
# /src the source snapshot read-only, /o the run directory, /ipc the inference socket directory) or, for a
# board plan, printed by car_up.sh --dry-run (DRY_RUN=1: print the commands, start nothing, check nothing).
#
# Order (from tools/sim_loop/stack_inner.sh, D2, and tools/integ_car_chain/nano_chain_inner_v2.sh +
# tools/car_geo_producer/rig_inner.sh), each step "ready <step>" in marks.txt or "FAIL ..." and exit:
#   kv       fake KV260 port in kv.json (host; the fake KV runs in its own container) | the board's KV260
#   camera   sim: /fpga/depth + /clock from Gazebo (gazebo), then the depth adapter's /fpga/depth16 (adapter)
#            replay: replay_camera.py rows in replay.csv | live: the Orbbec driver's first depth frame (start retry),
#            then tools/car_run/camera_check.py (frame ids / sizes / zero distortion / encodings as the runner checks),
#            then (GROUND_CALIB auto|check, 10-15) tools/car_run/ground_calib.py: the ground plane in 10 depth frames ->
#            the camera's height / pitch / roll (auto: adopted within +-5 deg / +-3 cm / +-5 deg, else FAIL; check:
#            the configured mount kept within 1 deg / 1 cm, else FAIL; off: not run), then the mount TF
#   chassis  pty: mcu_sim.py --pty, then host/ros_chassis_driver.py on it gets an MCU status frame
#            uart: the chassis driver on the real port gets an MCU status frame
#            none (sim): the D1 e-stop stand-in sim_estop_gate (KV e-stop latch -> /fpga/sim_estop, guard input)
#   nav      nav2: controller configs, navigation.launch.py (no autostart), nav_startup_probe.py --activate
#            costmap: Nav2's standalone costmap + lifecycle manager, "Managed nodes are active"
#            board nav2 with FPGA_NAV_COSTMAP rolling (the board default, 10-15): this step runs AFTER hub, bridge and
#            producer (its local costmap is the bridge's /fpga/rolling_map/grid, which the bridge publishes only once it
#            has the producer's deltas), and tools/car_run/grid_probe.py must receive that grid before Nav2 is
#            activated (GRID_READY_S); everywhere else the order below is unchanged
#   pose     rtabmap (board, DRAFT): rtabmap odometry + localization on a pinned database, localization gate
#            static (board, line-4 bench, 10-15): tools/car_run/bench_pose.py -- map -> odom -> base_link identity on
#            /tf_static (the camera is fixed, no chassis: the true pose) + a free stand-in /map; NOT mapping/localization
#            odom (board, real-car drive, 10-15): bench_pose.py --no-odom-base (map -> odom identity + stand-in /map),
#            odom -> base_link from the wheel odometry below; NOT mapping/localization. With nav2 (10-17): bench_pose
#            --no-odom-base --map-m 0 (map -> odom only), /map from Nav2's map_server (MAP_YAML, required)
#   wheel_odom (WHEEL_ODOM=1; car_up: default 1 with chassis uart) host/ros_wheel_odom.py --source chassis, after the
#            chassis driver: live -> odom -> base_link on /tf + /odom (nobody else publishes that edge on the live car;
#            with POSE static the chassis is none, refused); replay/sim (pty) -> shadow topics /fpga/wheel_odom/tf and
#            /fpga/wheel_odom only (replay_camera.py / Gazebo own odom -> base_link: never two publishers of one edge)
#   teleop   (TELEOP=1, car_up CAR_TELEOP=1) host/ros_teleop_safe.py: the only /fpga/cmd_vel_safe publisher, input socket
#            $IPC/teleop.sock (tools/car_run/teleop_keys.py); not gated by the motion permit, the MCU heartbeat latch is
#            the backstop; car_validate refuses it with sim (the velocity guard publishes there) and nav2; the run is
#            marked (marks "teleop", the TELEOP file) -- not a navigation run
#   hub      "hub listening"     bridge   rolling map bridge alive 3 s     producer  its first frame row
#   stub     (stub detector) socket exists     runtime   "startup" event and the first frame_processed
# Then READY, and a 1 s supervision loop: a child that exits takes the whole stack down (fail-stop, its
# name and rc in stack_fail.txt); car_down.sh stops everything else.
# Stop: INT/TERM to this script -> every child by its own pid in reverse start order, INT -> TERM -> KILL;
# marks.txt "stop <name>" before each (the per-child stop time). The C++ nodes run as their own executables
# (ros_exe), not through `ros2 run`: that wrapper catches INT and waits for its child without passing the
# signal on (ros2run run_executable assumes a terminal Ctrl-C that hits the whole process group), so INT to
# its pid never reached the node -- 10 s, then TERM killed the wrapper (rc 143) and orphaned the node
# (line-4 bench 10-15: costmap, lifecycle, mount_tf each 10 s, stack stop 39-41 s; host replay the same).
# Never pkill -f.
#
# Test-only recorders (RECORD=1, car_up.sh --record; never needed by the car): motion recorder, permit
# recorder, single-source check, display PC listener (yields control), runtime_info grabber, runner
# --frames-dir / --stall-diag. They make a run checkable by tools/verify_bottle_run.py --layout sim.
# (no set -u before the ROS setup files: they are not nounset-clean)
: "${CAMERA:?}" "${POSE:?}" "${NAV:?}" "${DETECTOR:?}" "${CHASSIS:?}"
: "${DRY_RUN:=0}" "${RECORD:=0}" "${O:=/o}" "${SRCD:=/src}" "${IPC:=/ipc}" "${HEAD:=unknown}"
: "${INF_DEVICE:=cpu}" "${HUBPORT:=47431}" "${HUB_BIND:=127.0.0.1}" "${KV_ADDR:=127.0.0.1:47811}" "${KV_BUILD_ID:=}"
: "${PREP:=/prep}" "${HZ:=10}" "${STACK_CPUS:=}" "${RUNTIME_CPUS:=}" "${MCU_HEARTBEAT:=kv}"
# 10-27 (AR_LATENCY_PLAN_20261027 plan A, owner): the live producer takes the newest depth frame as soon as its
# worker is free (--rate-hz 30 = the camera rate, stamp gate k=1) instead of every third one: the mailbox wait
# drops ~40 -> ~15 ms (frame rate stays bounded by the worker's ~90 ms). Replay / sim keep HZ.
if [ "$CAMERA" = live ]; then : "${PRODUCER_HZ:=30}"; else : "${PRODUCER_HZ:=$HZ}"; fi
: "${GZ_WAIT_S:=240}" "${NAV_READY_S:=180}" "${PRODUCER_READY_S:=90}" "${RUNTIME_READY_S:=180}" "${CAMERA_READY_S:=60}"
: "${CHASSIS_PORT:=/dev/fpga_chassis}" "${RTABMAP_DB:=}" "${CAR_FAILSTOP:=0}"   # 10-26 owner：重成功率，放宽自 1
# rtabmap (10-15, POSE rtabmap|map; car_up.sh passes them): the odometry under rtabmap (visual|wheel), the database
# directory as this container sees it, the INT -> exit wait of rtabmap_car.sh at stop (it saves the database), the
# start wait (rtabmap_car: settle 3 s + odometry checks up to 2 x 30 s + gate), the board Nav2 map (rtabmap-export)
: "${RTABMAP_ODOM:=visual}" "${RTABMAP_DB_DIR:=/data/fpga/rtabmap}" "${RTABMAP_STOP_S:=240}" "${RTABMAP_READY_S:=120}"
: "${MAP_YAML:=}" "${NAV_COSTMAP:=}" "${GRID_READY_S:=60}"
[ "$RTABMAP_READY_S" -ge 120 ] 2>/dev/null || RTABMAP_READY_S=120
# the localization gate (rtabmap loc + visual odometry only; car_up.sh CAR_GATE_MODE / CAR_GATE_X_S): what it approves
# (correction, default since 10-15 evening | output | frame) and X, the oldest held correction it still uses (s).
# X tiers (host/localization_gate.py COUNT_MODES): measure on the board first the rtabmap localization output rate
# and the p90 stamp age at which the gate receives them; >= 3 Hz and p90 <= 1.0 s -> 2; 1-3 Hz and p90 <= 1.5 s -> 3;
# < 1 Hz -> no navigation on estimated poses (reduce the board load first); then fixed. rtabmap_car.sh checks both.
: "${RTABMAP_GATE_MODE:=correction}" "${RTABMAP_GATE_X_S:=2}"
# the gate's odometry jump check (10-17 第 10 棒; car_up.sh CAR_GATE_ODOM_JUMP / _V_MPS / _E_M): 1 | 0, V (m/s), E (m)
: "${RTABMAP_GATE_ODOM_JUMP:=1}" "${RTABMAP_GATE_ODOM_V_MPS:=0.5}" "${RTABMAP_GATE_ODOM_E_M:=0.10}"
# drive chain (10-15): remote control, wheel odometry (empty wheel parameters = the placeholders of host/wheel_odom.py)
: "${MAPPING:=0}" "${MAPPING_BLANK_M:=20}"   # 10-18 in-stack mapping (car_up CAR_MAPPING=1)
: "${LIVE_ALIGN:=1}"   # 10-27 live alignment while mapping (car_up CAR_LIVE_ALIGN; 0 = off = as before)
: "${NAV_LIVE_GLOBAL:=1}"   # 10-29 Nav2's global map while mapping (car_up CAR_NAV_LIVE_GLOBAL; 0 = off = as before)
: "${NAV_GLOBAL_ROLLING:=1}"   # 10-29 the local layer's obstacles in that global map too (car_up CAR_NAV_GLOBAL_ROLLING)
: "${TELEOP:=0}" "${WHEEL_ODOM:=0}" "${TELEOP_V_MAX:=}" "${TELEOP_W_MAX:=}" "${TELEOP_MAX_CMD_S:=}"
: "${WHEEL_DIAM_M:=}" "${COUNTS_PER_REV:=}" "${TRACK_M:=}"
: "${CAMERA_FPS:=15}"   # 10-29 owner (Nano CPU): colour and depth rate (car_up CAR_CAMERA_FPS; 30 = before)
: "${LIVE_LAUNCH_ARGS:=color_width:=640 color_height:=480 color_fps:=$CAMERA_FPS color_format:=RGB depth_width:=640 depth_height:=400 depth_fps:=$CAMERA_FPS depth_format:=Y16 enable_ir:=false enable_point_cloud:=false depth_registration:=true align_mode:=HW publish_tf:=false}"
# the rate the driver really gets is the one in LIVE_LAUNCH_ARGS (given from outside it wins); the rate-dependent rules
# (depth probe half-rate rule, runner colour/depth pairing + geometry match, rtabmap approx sync) follow it
case " $LIVE_LAUNCH_ARGS " in *" color_fps:="*) CAMERA_FPS=$(printf '%s\n' " $LIVE_LAUNCH_ARGS " | sed -n 's/.* color_fps:=\([^ ]*\) .*/\1/p');; esac
case "$CAMERA_FPS" in 5|10|15|30) ;; *) echo "CAMERA_FPS: 5|10|15|30 (got '$CAMERA_FPS')" >&2; exit 2;; esac
export CAMERA_FPS
: "${LIVE_DEPTH_TOPIC:=/camera/depth/image_raw}" "${LIVE_INFO_TOPIC:=/camera/depth/camera_info}"
: "${LIVE_COLOR_TOPIC:=/camera/color/image_raw}" "${LIVE_COLOR_INFO_TOPIC:=/camera/color/camera_info}" "${CAMERA_CHECK_S:=20}"
# runner camera subscriptions (host/ros_runtime.py --camera-qos; auto = reliable for --sensor-source hardware)
: "${RUNNER_CAMERA_QOS:=auto}"
# heartbeat-gap T2 (10-16) measurement switches, defaults = the chain as before (car_up.sh passes them only when set):
# PRODUCER_DEPTH_QOS auto (the producer's default: reliable:5 on live) | reliable | sensor_data -> --depth-qos (only
# when not auto); LIVE_DRIVER_DDS_PROFILE: a Fast DDS XML profile (relative to the source root or absolute) for the
# camera DRIVER process alone (FASTRTPS_DEFAULT_PROFILES_FILE; the sender's shared-memory segment is the one that
# carries its images), empty = Fast DDS defaults. As rig_inner.sh CAM_DDS_PROFILE (10-12).
: "${PRODUCER_DEPTH_QOS:=auto}" "${LIVE_DRIVER_DDS_PROFILE:=}"
case "$PRODUCER_DEPTH_QOS" in auto|reliable|sensor_data) ;; *) echo "PRODUCER_DEPTH_QOS: auto|reliable|sensor_data" >&2; exit 2;; esac
# POSE=static (bench): bench_pose.py --odom-hz (0 = odom -> base_link static too) and the stand-in /map size (0 = none)
# POSE=odom (10-16): stand-in 20 m by default; MAP_YAML (car_up: CAR_MAP_YAML / the RTABMAP_DB export) = that map instead
: "${BENCH_ODOM_HZ:=0}"
if [ "${POSE:-}" = odom ]; then : "${BENCH_MAP_M:=20}"; else : "${BENCH_MAP_M:=8}"; fi
# NAV_DRYRUN (10-17, car_up CAR_NAV_DRYRUN=1): the board Nav2 dry run, live static nav2 none only (car_validate)
: "${NAV_DRYRUN:=0}"
# CTRLPS2 (2026-11-03, car_up CAR_NAV_BACKEND): nav2 (default, every line as before) | kv (the KV260 drives: Nav2 keeps
# map_server + planner_server for the G1 route, host/ros_kv_nav_proxy.py --mode kv serves navigate_to_pose and
# publishes /cmd_vel into the unchanged guard / arbitration / chassis) | shadow (Nav2 drives; the proxy only records
# the KV's commands next to Nav2's). Both need --nav nav2 (car_validate) and a KV serving "nav" (kv_server --nav).
: "${NAV_BACKEND:=nav2}"
: "${LIVE_START_TRIES:=10}" "${LIVE_START_WAIT_S:=20}" "${LIVE_START_GAP_S:=10}" "${LIVE_DEPTH_MSGS:=30}"   # driver start retry
# half-rate check (10-17, car_up.sh CAR_CAMERA_RATE_CHECK, board default on; docs/design/COLOUR_RATE_NAVMODE_20261017.md
# section 7): 1 = the depth probe listens CAMERA_RATE_S (integer 1-5) more seconds and exits 3 (half-rate) / 4 (too few
# frames) -> a failed start for car_start_retry; 0 (the default here: car_up passes it only when on) = the plan of before
: "${CAMERA_RATE_CHECK:=0}" "${CAMERA_RATE_S:=3}"
case "$CAMERA_RATE_CHECK" in 0|1) ;; *) echo "CAMERA_RATE_CHECK: 0|1" >&2; exit 2;; esac
case "$CAMERA_RATE_S" in [1-5]) ;; *) echo "CAMERA_RATE_S: an integer 1-5" >&2; exit 2;; esac
# LIVE_DEPTH_MSGS 30 (was 3; 10-15): cam-start-dig showed a failed start leaking 1-4 depth images before it stops (m11
# session 8: 4 images at 30 fps, then none) -- 3 passes it, 30 (about 1 s of depth) had 0 false passes in 10 sessions;
# the line-4 bench ran its 5 cycles with 30 and every start came up on try 1 (car-bench-20261015-results RUNLOG)
# live camera mount (car_up.sh, tools/car_run/car_mount.py): HEIGHT_CELL for the producer, CAM_MOUNT_TF "x y z qx qy qz qw"
# base_link -> LIVE_CAMERA_FRAME; empty on the host (the producer's default 7, the sim/replay TF unchanged)
# LIVE_CAMERA_FRAME (10-15): ONE frame for the mount TF, the producer and the runner. The driver runs with
# depth_registration:=true align_mode:=HW, so the depth image is re-projected into the COLOUR camera: depth image,
# colour image and the depth CameraInfo all carry camera_color_optical_frame (board record 10-15, live_tf.py
# camera_frame.txt). camera_depth_optical_frame (before 10-15) is published by nobody (publish_tf:=false) and the
# runner refuses pairs whose header frame differs from its --camera-frame. camera_check.py verifies it on the board.
: "${HEIGHT_CELL:=}" "${CAM_MOUNT_TF:=}" "${CAM_HEIGHT:=}" "${CAM_PITCH_DEG:=}" "${LIVE_CAMERA_FRAME:=camera_color_optical_frame}"
# start-up ground self-calibration (10-15, live only; car_up.sh CAR_GROUND_CALIB): auto (default) | check | off;
# HEIGHT_CELL_GIVEN = car_up's CAR_HEIGHT_CELL (empty = derived: the fitted height may move the cell); GROUND_CALIB_MAP
# = the mount stored beside the pinned rtabmap database (loc: compared, printed); frames and the capture wait
: "${GROUND_CALIB:=auto}" "${HEIGHT_CELL_GIVEN:=}" "${GROUND_CALIB_MAP:=}" "${GROUND_CALIB_FRAMES:=10}" "${GROUND_CALIB_S:=15}"
# robot (car body) height of the rolling bridge's obstacle band (owner 10-16): band = ground + 0.05 .. ROBOT_HEIGHT + 0.05.
# Host sim / replay: not passed -> 0.25 (the value before 10-16, plan byte for byte unchanged); the live car: car_up.sh
# CAR_ROBOT_HEIGHT (0.31 m -> band top 0.36 m), and the camera must be >= ROBOT_HEIGHT + 0.055 m (checked below).
: "${ROBOT_HEIGHT:=0.25}"
source "$(dirname "$0")/car_common.sh"
car_validate || exit 2
# the driver's frame-drop log (10-17): live adds enable_frame_drop_log:=true unless LIVE_FRAME_DROP_LOG=false
if [ "$CAMERA" = live ]; then car_frame_drop_log || exit 2; fi
# board nav2: NAV_COSTMAP empty = not given -> rolling; 'sim' must be explicit (car_common.sh car_nav_costmap)
car_nav_costmap "$NAV_COSTMAP" || exit 2
NAV_AFTER_PRODUCER=0; [ "$CAR_NAV_COSTMAP" = rolling ] && NAV_AFTER_PRODUCER=1
# nav2 (10-17): Nav2's map_server owns /map -- bench_pose.py (static, odom) publishes no stand-in /map then
[ "$NAV" = nav2 ] && BENCH_MAP_M=0
if [ "$DRY_RUN" != 1 ]; then
  source /opt/ros/humble/setup.bash
  [ -f /opt/fpga_ws/install/setup.bash ] && source /opt/fpga_ws/install/setup.bash
  [ "$CAMERA" = live ] && [ -f /opt/orbbec_ws/install/setup.bash ] && source /opt/orbbec_ws/install/setup.bash
  cd "$SRCD" || exit 3
fi
set -u
export PYTHONPATH=$SRCD:${PYTHONPATH:-} PYTHONUNBUFFERED=1 LANG=C.UTF-8 LC_ALL=C.UTF-8

REST=""
if [ -n "$STACK_CPUS" ]; then
  REST=$(python3 -c "
import sys
s=[]
for part in sys.argv[1].split(','):
    a,_,b=part.partition('-'); s+=range(int(a),int(b or a)+1)
print(','.join(map(str,s)))" "$STACK_CPUS")
fi
# Command prefix (an array: "func ... &" would fork a subshell and $! would miss the real process).
# Background children of a non-interactive bash start with SIGINT ignored: env --default-signal=INT.
T=(env --default-signal=INT); [ -n "$REST" ] && T=(taskset -c "$REST" "${T[@]}")
TR=("${T[@]}"); [ -n "$RUNTIME_CPUS" ] && TR=(taskset -c "$RUNTIME_CPUS" env --default-signal=INT)

mark() { [ "$DRY_RUN" = 1 ] || echo "$(date +%s.%N) $*" >> "$O/marks.txt"; }
# ros_exe PKG EXE: the node's own executable, the file `ros2 run PKG EXE` would start (<prefix>/lib/PKG/EXE on
# AMENT_PREFIX_PATH; both images install ROS from apt: /opt/ros/humble). Started directly so that its pid is
# the node's and INT reaches it (see "Stop" above).
ros_exe() {
  if [ "$DRY_RUN" = 1 ]; then echo "/opt/ros/humble/lib/$1/$2"; return 0; fi
  local d IFS=:
  for d in ${AMENT_PREFIX_PATH:-/opt/ros/humble}; do [ -x "$d/lib/$1/$2" ] && { echo "$d/lib/$1/$2"; return 0; }; done
  return 1
}
fail() { mark "FAIL $1"; echo "FAIL $1" >&2; [ "$DRY_RUN" = 1 ] || echo "$1" > "$O/stack_fail.txt"; exit "${2:-3}"; }
ready() { mark "ready $*"; [ "$DRY_RUN" = 1 ] && echo "#   -> ready check: $*"; return 0; }
NAMES=(); PIDS=(); STARTS=()
# launch NAME OUTFILE CMD... : background, stdout+stderr to OUTFILE (relative to $O), tracked by pid
launch() {
  local n=$1 out=$2; shift 2
  if [ "$DRY_RUN" = 1 ]; then printf '[%s] %s > %s\n' "$n" "$*" "$O/$out"; return 0; fi
  "$@" > "$O/$out" 2>&1 &
  local p=$!
  NAMES+=("$n"); PIDS+=("$p"); STARTS+=("$(car_pid_start "$p")")
  CMDS+=("$(printf '%q ' "$@")"); OUTS+=("$out")   # 10-26: kept for the supervision restart (car_exit_action)
  echo "$n $p ${STARTS[-1]}" >> "$O/pids.txt"
}
run1() {   # one-shot step (foreground): NAME OUTFILE CMD...
  local n=$1 out=$2; shift 2
  if [ "$DRY_RUN" = 1 ]; then printf '[%s, once] %s > %s\n' "$n" "$*" "$O/$out"; return 0; fi
  "$@" > "$O/$out" 2>&1
}
alive() {   # NAME: the tracked process of that name still runs
  # "return $?", not a bare "return": inside a trap handler (cleanup runs from the EXIT trap) a bare return gives the
  # status from before the trap (130 after INT/TERM), so every alive answered "no" there (host 10-15)
  local k
  for k in "${!NAMES[@]}"; do [ "${NAMES[$k]}" = "$1" ] && { car_pid_alive "${PIDS[$k]}" "${STARTS[$k]}"; return $?; }; done
  return 1
}
# wait_for NAME SECONDS TEST... : poll TEST (every 0.2 s + its own time) until SECONDS of wall time have passed;
# fail early if process NAME (if not '-') is gone
wait_for() {
  local n=$1 s=$2 end; shift 2
  [ "$DRY_RUN" = 1 ] && return 0
  end=$((SECONDS + s))
  while [ "$SECONDS" -lt "$end" ]; do
    "$@" && return 0
    [ "$n" = - ] || alive "$n" || { sleep 0.3; "$@" && return 0; fail "$n exited before it was ready" 3; }
    sleep 0.2
  done
  "$@"
}
has_line() { grep -q -- "$2" "$O/$1" 2>/dev/null; }
check_reasons() {   # FILE (relative to $O): the "reasons" of a check's JSON result, joined
  python3 -c 'import json, sys
try:
    print("; ".join(json.load(open(sys.argv[1])).get("reasons") or ["no reasons"]))
except (OSError, ValueError):
    print("no result file")' "$O/$1"
}
topics_have() {   # TOPIC... : all present in one ros2 topic list
  "${T[@]}" ros2 topic list --no-daemon 2>/dev/null > "$O/topics_ready.txt" || return 1
  local t; for t in "$@"; do grep -qx -- "$t" "$O/topics_ready.txt" || return 1; done
}
first_frame_row() { python3 - "$O/frames.jsonl" <<'PY'
import json, sys
try:
    rows = open(sys.argv[1]).read().splitlines()
except OSError:
    sys.exit(1)
for line in rows:
    try:
        r = json.loads(line)
    except ValueError:
        continue
    if 'event' not in r:
        sys.exit(0)
sys.exit(1)
PY
}
# The standalone Nav2 costmap (NAV costmap) aborts when INT finds it ACTIVE (its update thread still joinable:
# "terminate called without an active exception", rc 134 -- line-4 bench 10-15 and every host replay rc.txt), and
# segfaults (139) when INT finds it cleaned up (lifecycle RESET / SHUTDOWN, host 10-15); INACTIVE it exits 0
# (host 10-15: never configured 0, configured 0, active 134, deactivated 0 x3, deactivated + cleaned up 139 x3).
# So the stop first asks the lifecycle manager (check_lifecycle) to PAUSE (= deactivate) it, in the background from
# the start of the stop (the ros2 CLI takes ~2 s on the host) and waited for -- at most COSTMAP_PAUSE_S -- just
# before the lifecycle manager itself is stopped; a failed or late request is marked and the stop goes on unchanged
# (still reverse start order, every child INT -> TERM -> KILL as before).
: "${COSTMAP_PAUSE_S:=8}"
COSTMAP_PAUSE_PID=""
costmap_pause_start() {
  [ "$NAV" = costmap ] && alive lifecycle && alive costmap || return 0
  mark "costmap deactivate requested (check_lifecycle manage_nodes PAUSE)"
  "${T[@]}" timeout "$COSTMAP_PAUSE_S" ros2 service call /check_lifecycle/manage_nodes \
    nav2_msgs/srv/ManageLifecycleNodes "{command: 1}" > "$O/costmap_pause.log" 2>&1 &
  COSTMAP_PAUSE_PID=$!
}
costmap_pause_wait() {
  [ -n "$COSTMAP_PAUSE_PID" ] || return 0
  local rc; wait "$COSTMAP_PAUSE_PID"; rc=$?; COSTMAP_PAUSE_PID=""
  if [ "$rc" = 0 ] && grep -q 'success=True' "$O/costmap_pause.log" 2>/dev/null; then mark "costmap deactivated"
  else mark "costmap deactivate failed rc=$rc (costmap_pause.log); stopping on"; fi
}
cleanup() {
  trap - EXIT INT TERM
  mark "cleanup"
  costmap_pause_start
  local k
  for ((k=${#PIDS[@]}-1; k>=0; k--)); do
    [ "${NAMES[$k]}" = lifecycle ] && costmap_pause_wait
    mark "stop ${NAMES[$k]}"
    if [ "${NAMES[$k]}" = rtabmap ] || [ "${NAMES[$k]}" = mapping ]; then   # rtabmap_car.sh saves the database (and exports, map) after INT
      CAR_INT_WAIT_S=$RTABMAP_STOP_S CAR_RC_FILE="$O/rc.txt" car_stop_pid "${NAMES[$k]}" "${PIDS[$k]}" "${STARTS[$k]}"
    else CAR_RC_FILE="$O/rc.txt" car_stop_pid "${NAMES[$k]}" "${PIDS[$k]}" "${STARTS[$k]}"; fi
  done
  mark "stopped"
}
if [ "$DRY_RUN" != 1 ]; then trap cleanup EXIT; trap 'exit 130' INT TERM; fi
mark "start camera=$CAMERA pose=$POSE nav=$NAV detector=$DETECTOR chassis=$CHASSIS source=$SOURCE record=$RECORD teleop=$TELEOP wheel_odom=$WHEEL_ODOM head=$HEAD"
[ "$DRY_RUN" = 1 ] && echo "# car_inner plan: camera=$CAMERA pose=$POSE nav=$NAV detector=$DETECTOR chassis=$CHASSIS source=$SOURCE platform=$PLATFORM record=$RECORD teleop=$TELEOP wheel_odom=$WHEEL_ODOM"
if [ "$TELEOP" = 1 ]; then
  if [ "$NAV" = nav2 ]; then   # 10-18 arbitration
    mark "teleop: REMOTE CONTROL + Nav2 ARBITRATION -- /fpga/cmd_vel_safe from host/ros_teleop_safe.py (remote control when in hand, else the velocity guard's /fpga/nav/cmd_vel_safe); remote control not gated by the motion permit; MCU heartbeat latch is the backstop"
    [ "$DRY_RUN" = 1 ] || echo "remote control + Nav2 arbitration run: remote control (in hand) not gated by /fpga/motion_permit; navigation through the velocity guard" > "$O/TELEOP"
  else
    mark "teleop: REMOTE CONTROL run -- NOT a navigation run; /fpga/cmd_vel_safe from host/ros_teleop_safe.py, not gated by the motion permit; MCU heartbeat latch is the backstop"
    [ "$DRY_RUN" = 1 ] || echo "remote control (teleop) run: NOT a navigation run; not gated by /fpga/motion_permit" > "$O/TELEOP"
  fi
fi

# --- kv ----------------------------------------------------------------------------------------------
if [ "$PLATFORM" = host ]; then
  [ "$DRY_RUN" = 1 ] && echo "# kv: the fake KV260 container writes $O/kv.json (port line)"
  wait_for - 60 has_line kv.json '"port"' || fail "fake KV not up (no port in kv.json in 60 s)"
  ready kv "$KV_ADDR"
else
  [ "$DRY_RUN" = 1 ] && echo "# kv: the board's KV260 at $KV_ADDR${KV_BUILD_ID:+ (build id $KV_BUILD_ID)}; judged by the producer's first frame"
  ready kv "$KV_ADDR (board, judged at the producer)"
fi

# --- camera ------------------------------------------------------------------------------------------
LIVE=()
case "$CAMERA" in
sim)
  [ "$DRY_RUN" = 1 ] && echo "# gazebo: the Gazebo container publishes /fpga/depth and /clock"
  wait_for - "$GZ_WAIT_S" topics_have /fpga/depth /clock || fail "no /fpga/depth + /clock from Gazebo in $GZ_WAIT_S s"
  ready gazebo
  launch adapter adapter.log "${T[@]}" python3 robot_sim/fpga_sim/scripts/sim_adapter.py --cloud --summary "$O/adapter_summary.json"
  wait_for adapter 60 topics_have /fpga/depth16 /fpga/camera_info || fail "depth adapter: no /fpga/depth16 in 60 s"
  ready adapter;;
replay)
  launch camera replay.log "${T[@]}" python3 tools/car_geo_producer/replay_camera.py --prep "$PREP" --hz "$HZ" \
    --duration-s 100000000 --memory-boxes 0 --rgb --log "$O/replay.csv"
  wait_for camera "$CAMERA_READY_S" python3 -c "import sys;sys.exit(0 if sum(1 for _ in open(sys.argv[1])) >= 3 else 1)" "$O/replay.csv" \
    || fail "replay camera: no frames in $CAMERA_READY_S s"
  ready camera replay;;
live)   # board, DRAFT (second class): the Orbbec Gemini 2 driver, with rig_inner.sh's start retry (CAMERA=live, 10-13):
        # the depth stream does not come up on about two starts in three. A start is good only when depth DATA arrives:
        # tools/car_run/depth_probe.py receives LIVE_DEPTH_MSGS (30) images on LIVE_DEPTH_TOPIC within LIVE_START_WAIT_S
        # (20) -- not the rig's grep of "depth Frame" in driver.log, which the board showed printed with no depth frame
        # after it (10-15, l_chain32). Else the start is stopped and repeated after LIVE_START_GAP_S (10), up to
        # LIVE_START_TRIES (10). Each try -> marks.txt "camera driver try=N depth_stream=0|1 probe_rc=R"; a failed
        # try's log -> driver_failed_start_N.log, the probe's -> driver_probe_N.log. Every try failing = stack_fail
        # (the rig leaves its last try running).
  DRV_T=("${T[@]}")
  if [ -n "$LIVE_DRIVER_DDS_PROFILE" ]; then
    DDSP=$LIVE_DRIVER_DDS_PROFILE; [ "${DDSP#/}" = "$DDSP" ] && DDSP=$SRCD/$DDSP
    [ "$DRY_RUN" = 1 ] || [ -f "$DDSP" ] || fail "LIVE_DRIVER_DDS_PROFILE $LIVE_DRIVER_DDS_PROFILE: no such file ($DDSP)"
    DRV_T=(env "FASTRTPS_DEFAULT_PROFILES_FILE=$DDSP" "${T[@]}")
    [ "$DRY_RUN" = 1 ] || { echo "$DDSP $(sha256sum "$DDSP" | cut -c1-64)" > "$O/driver_dds_profile.txt"; mark "camera driver dds profile $DDSP"; }
    # 10-17 shm8 guard (docs/design/HEARTBEAT_GAP_DIG_20261016.md section 11): a failed start can leave an unowned
    # 8 MB segment in the container's 64 MB /dev/shm; before every retry `fastdds shm clean` deletes the segments whose
    # lock file nobody holds (its source: open + flock LOCK_EX|LOCK_NB; a live participant's stays). Then, once the
    # driver runs, tools/car_run/shm_segcheck.py: the driver node's OWN segment must be >= the profile's segment_size,
    # else the start is refused (Fast DDS falls back to the 512 KB default without a word: the 32 MB profile, 10-16).
    driver_shm_clean() {
      df -k /dev/shm; ls -la /dev/shm
      command -v fastdds > /dev/null || { echo "fastdds: not on PATH, nothing cleaned"; return 127; }
      fastdds shm clean; local rc=$?
      df -k /dev/shm; return $rc
    }
    CAR_RETRY_BEFORE=(driver_shm_clean)
    DRV_CLEAN_TXT="; before every retry: fastdds shm clean"
  fi
  # 10-17 (docs/design/CAM_USB_REENUM_20261017.md): before every try, the camera's USB node as this container sees it
  # -> marks "camera driver usb try=N BBB/DDD visible|MISSING (port)": a device number that changes between tries =
  # the camera re-enumerated (board 14:36 / 16:26: in the first second of try 1); MISSING = the node is not in this
  # container's /dev/bus/usb (the --device of before). Read from /sys (no lsusb in the image). Stand-ins: CAR_SYS_USB
  # (/sys/bus/usb/devices) CAR_DEV_USB (/dev/bus/usb).
  camera_usb_note() {
    local d b n out=""
    for d in "${CAR_SYS_USB:-/sys/bus/usb/devices}"/*; do
      [ "$(cat "$d/idVendor" 2>/dev/null)" = 2bc5 ] || continue
      b=$(printf '%03d' "$(cat "$d/busnum")") n=$(printf '%03d' "$(cat "$d/devnum")")
      out="$out${out:+, }$b/$n $([ -c "${CAR_DEV_USB:-/dev/bus/usb}/$b/$n" ] && echo visible || echo MISSING) (${d##*/})"
    done
    echo "${out:-no 2bc5 device in ${CAR_SYS_USB:-/sys/bus/usb/devices}}"
  }
  CAR_RETRY_NOTE=(camera_usb_note)
  CAR_RETRY_PROBE=("${T[@]}" python3 tools/car_run/depth_probe.py --topic "$LIVE_DEPTH_TOPIC" --count "$LIVE_DEPTH_MSGS"
                   --timeout-s "$LIVE_START_WAIT_S")
  # half-rate check (10-17, CAMERA_RATE_CHECK=1): the same probe, CAMERA_RATE_S more seconds on depth + colour header
  # stamps after the depth images; exit 3 half-rate / 4 too few frames = a failed start (stop, pause, clean, again).
  # car_start_retry's wait (it kills an overrunning probe at wait + 5 s) grows by the window; the probe's depth wait
  # (--timeout-s) stays LIVE_START_WAIT_S. Order: depth probe -> rate check (inside the retry) -> shm8 segment check
  # (below, on the start that was kept; it refuses, it does not retry). Section 7 of the doc above.
  RETRY_WAIT_S=$LIVE_START_WAIT_S
  if [ "$CAMERA_RATE_CHECK" = 1 ]; then
    CAR_RETRY_PROBE+=(--rate-s "$CAMERA_RATE_S" --color-topic "$LIVE_COLOR_TOPIC")
    # 10-29: judged at the configured rate (30: the 10-17 rule, nothing added)
    [ "$CAMERA_FPS" = 30 ] || CAR_RETRY_PROBE+=(--rate-fps "$CAMERA_FPS")
    RETRY_WAIT_S=$((LIVE_START_WAIT_S + CAMERA_RATE_S))
  fi
  if [ "$DRY_RUN" = 1 ]; then
    echo "# camera usb: before every try marks 'camera driver usb try=N BBB/DDD visible|MISSING (port)' (the 2bc5 device in /sys, its node in this container's /dev/bus/usb)"
    printf '[camera] %s > %s  (start retry: up to %s tries; good = probe %s gets %s depth images within %s s, else stop, %s s pause%s, again)\n' \
      "${DRV_T[*]} ros2 launch orbbec_camera gemini2.launch.py $LIVE_LAUNCH_ARGS" "$O/driver.log" "$LIVE_START_TRIES" \
      "'${CAR_RETRY_PROBE[*]}'" "$LIVE_DEPTH_MSGS" "$LIVE_START_WAIT_S" "$LIVE_START_GAP_S" "${DRV_CLEAN_TXT:-}"
    [ "$CAMERA_RATE_CHECK" = 1 ] && echo "# camera rate check: after the depth images the probe listens $CAMERA_RATE_S s more on $LIVE_DEPTH_TOPIC + $LIVE_COLOR_TOPIC (header stamps); depth median < 32.8 ms or colour p25 > 50 ms = half-rate (probe exit 3), < 20 depth / 10 colour intervals = exit 4: both a failed start, retried as above (retry wait $RETRY_WAIT_S + 5 s); the kept start's result -> $O/camera_rate_check.json"
  else
    CAR_MARKS_FILE="$O/marks.txt" car_start_retry "$LIVE_START_TRIES" "$RETRY_WAIT_S" "$LIVE_START_GAP_S" "$O/driver.log" \
      "${DRV_T[@]}" ros2 launch orbbec_camera gemini2.launch.py $LIVE_LAUNCH_ARGS
    RETRY_RC=$?
    if [ "$CAMERA_RATE_CHECK" = 1 ]; then   # every try, one line (probe_rc 3 = half-rate, 4 = too few frames for the check)
      RATE_TRIES=$(python3 tools/car_run/depth_probe.py --summarize "$O"/driver_probe_*.log 2>&1)
      mark "camera rate check tries: $RATE_TRIES"
    fi
    if [ "$RETRY_RC" != 0 ]; then
      [ "$CAMERA_RATE_CHECK" = 1 ] \
        && fail "Orbbec driver: no good start in $LIVE_START_TRIES starts (depth images on $LIVE_DEPTH_TOPIC within $LIVE_START_WAIT_S s, then the half-rate check over $CAMERA_RATE_S s): $RATE_TRIES (driver_failed_start_*.log, driver_probe_*.log)"
      fail "Orbbec driver: no depth images on $LIVE_DEPTH_TOPIC in $LIVE_START_TRIES starts (each $LIVE_START_WAIT_S s; driver_failed_start_*.log, driver_probe_*.log)"
    fi
    NAMES+=(camera); PIDS+=("$CAR_RETRY_PID"); STARTS+=("$(car_pid_start "$CAR_RETRY_PID")")
    echo "camera $CAR_RETRY_PID ${STARTS[-1]}" >> "$O/pids.txt"
    if [ "$CAMERA_RATE_CHECK" = 1 ]; then   # the kept start's probe line -> camera_rate_check.json
      python3 - "$O/driver_probe_$CAR_RETRY_TRY.log" "$O/camera_rate_check.json" "$CAR_RETRY_TRY" <<'PY' \
        || fail "camera rate check: no result line in driver_probe_$CAR_RETRY_TRY.log" 3
import json, sys
sys.path.insert(0, '.')
from tools.car_run.depth_probe import last_json
row = last_json(sys.argv[1])
if not row or not row.get('rate') or row['rate'].get('verdict') != 'normal':
    sys.exit(1)
row['try'] = int(sys.argv[3])
json.dump(row, open(sys.argv[2], 'w'), indent=1)
PY
      ready camera_rate "$(python3 tools/car_run/depth_probe.py --summarize "$O/driver_probe_$CAR_RETRY_TRY.log")"
    fi
  fi
  if [ -n "$LIVE_DRIVER_DDS_PROFILE" ]; then   # the shm8 guard (above): read-only, no DDS participant
    run1 driver_shm_check driver_shm_check.log "${T[@]}" python3 tools/car_run/shm_segcheck.py --pid "${CAR_RETRY_PID:-<driver pid>}" \
      --profile "$DDSP" --wait-s 10 --out "$O/driver_shm_check.json" \
      || fail "camera driver shared memory: $(tail -1 "$O/driver_shm_check.log" 2>/dev/null) (driver_shm_check.json; profile $LIVE_DRIVER_DDS_PROFILE)" 3
    ready driver_shm "$([ "$DRY_RUN" = 1 ] || tail -1 "$O/driver_shm_check.log")"
  fi
  LIVE=(--depth-topic "$LIVE_DEPTH_TOPIC" --info-topic "$LIVE_INFO_TOPIC" --camera-frame "$LIVE_CAMERA_FRAME")
  ready camera live
  # the runner's acceptance rules on one message of each topic, now: a wrong frame name / CameraInfo / size fails here
  # with its reason instead of a silent runtime timeout (camera_check.json)
  run1 camera_check camera_check.log "${T[@]}" python3 tools/car_run/camera_check.py --color-topic "$LIVE_COLOR_TOPIC" \
    --depth-topic "$LIVE_DEPTH_TOPIC" --info-topic "$LIVE_INFO_TOPIC" --record-info-topic "$LIVE_COLOR_INFO_TOPIC" \
    --frame "$LIVE_CAMERA_FRAME" --timeout-s "$CAMERA_CHECK_S" --out "$O/camera_check.json" \
    || fail "camera check: the driver's topics do not match what the runner accepts ($(check_reasons camera_check.json))"
  ready camera_check "frame $LIVE_CAMERA_FRAME"
  # the ground self-calibration (10-15): the depth stream is up, nobody uses the mount TF yet -- the producer bakes the
  # mount into its calibration_id on its first frame, so the mount is settled HERE, once, before the TF starts.
  # Refused (unreadable floor, mount too far from the configuration, hard limits) = FAIL with the readable reason
  # (car_up exit 7); to start without it CAR_GROUND_CALIB=off must be given explicitly (owner 10-15).
  GC_MOUNT_NOTE=""
  case "$GROUND_CALIB" in
  off) ;;
  auto|check)
    [ -n "$CAM_HEIGHT" ] && [ -n "$CAM_PITCH_DEG" ] \
      || fail "ground calib: no configured mount (CAM_HEIGHT / CAM_PITCH_DEG; car_up.sh CAR_CAM_HEIGHT / CAR_CAM_PITCH_DEG)" 2
    GCX=(--mode "$GROUND_CALIB" --height-m "$CAM_HEIGHT" --pitch-deg "$CAM_PITCH_DEG")
    [ -n "$HEIGHT_CELL_GIVEN" ] && GCX+=(--height-cell "$HEIGHT_CELL_GIVEN")
    # 10-28: the last adopted fit (car_up.sh copied it into the run dir) -- the fallback before the configuration
    [ -s "$O/ground_calib_last_good_in.json" ] && GCX+=(--last-good "$O/ground_calib_last_good_in.json")
    [ "$DRY_RUN" = 1 ] && echo "# ground_calib ($GROUND_CALIB): the floor 0.3-2 m ahead in $GROUND_CALIB_FRAMES depth frames; $([ "$GROUND_CALIB" = auto ] \
      && echo 'mount_tf and the producer --height-cell then take the FITTED mount of ground_calib.json (this plan prints the configured one)' \
      || echo 'the configured mount is kept when the fit agrees within 1 deg / 1 cm')"
    GC_FALLBACK=0
    if ! run1 ground_calib ground_calib.log "${T[@]}" python3 tools/car_run/ground_calib.py "${GCX[@]}" \
      --depth-topic "$LIVE_DEPTH_TOPIC" --info-topic "$LIVE_INFO_TOPIC" --frames "$GROUND_CALIB_FRAMES" \
      --timeout-s "$GROUND_CALIB_S" --save-depth "$O/ground_calib_depth.npz" --out "$O/ground_calib.json"; then
      # owner 10-26: no readable floor ahead (car facing a wall / boxes) must not keep the car from starting --
      # fall back to the configured, measured mount (10-17: 0.39 m / 7 deg), loudly. GROUND_CALIB_FALLBACK=refuse
      # restores the 10-15 behaviour (refuse the start).
      GC_WHY=$(check_reasons ground_calib.json)
      [ "${GROUND_CALIB_FALLBACK:-configured}" = configured ] \
        || fail "ground calib ($GROUND_CALIB) refused the start: $GC_WHY" 3
      GC_FALLBACK=1
      mark "GROUND_CALIB_FALLBACK: ground calib ($GROUND_CALIB) found no usable floor ($GC_WHY); using the configured mount ${CAM_HEIGHT} m / ${CAM_PITCH_DEG} deg"
      echo "GROUND_CALIB_FALLBACK: 地面标定没找到地面（${GC_WHY}），改用量好的安装值 ${CAM_HEIGHT} m / ${CAM_PITCH_DEG}°" >&2
    fi
    if [ "$GC_FALLBACK" = 1 ]; then
      GC_SRC=configured_fallback
      GC_MOUNT_NOTE=" (ground_calib $GROUND_CALIB: no floor found -> configured mount)"
    elif [ "$DRY_RUN" != 1 ]; then
      GC=$(python3 tools/car_run/ground_calib.py --read "$O/ground_calib.json") || fail "ground calib: no usable result in ground_calib.json" 3
      read -r HEIGHT_CELL CAM_HEIGHT CAM_PITCH_DEG GC_ROLL GC_SRC CAM_MOUNT_TF <<< "$GC"
      GC_MOUNT_NOTE=" roll ${GC_ROLL} deg (ground_calib $GROUND_CALIB: $GC_SRC)"
      # 10-26 owner：重成功率，放宽自 auto 拒启动: a refused fit falls back (warning only); 10-28: to the last
      # adopted fit first (last_good), else the configured mount (config_fallback); the hub shows ground_calib.json
      # "alert" on the console
      case "$GC_SRC" in config_fallback|last_good)
        mark "WARN ground_calib fallback: $GC_SRC mount used (see ground_calib.json warnings)"
        echo "WARN ground calib: fit refused, the $GC_SRC mount is used (ground_calib.json warnings): 相机角度测量异常，建议检查相机" >&2;;
      esac
    else
      GC_MOUNT_NOTE=" (ground_calib $GROUND_CALIB: values of ground_calib.json)"
    fi
    ready ground_calib "$GROUND_CALIB${GC_SRC:+ -> $GC_SRC}: height ${CAM_HEIGHT} m pitch ${CAM_PITCH_DEG} deg roll ${GC_ROLL:-?} deg height_cell ${HEIGHT_CELL}"
    if [ -n "$GROUND_CALIB_MAP" ]; then   # rtabmap loc: against the mount stored beside the pinned database (recorded only)
      if [ "$DRY_RUN" = 1 ]; then echo "# ground_calib vs map: python3 tools/car_run/ground_calib.py --compare $GROUND_CALIB_MAP $O/ground_calib.json -> marks.txt (warning > 1 deg / 1 cm; the current mount is used)"
      else mark "$(python3 tools/car_run/ground_calib.py --compare "$GROUND_CALIB_MAP" "$O/ground_calib.json")"; fi
    fi;;
  *) fail "GROUND_CALIB must be auto, off or check (given '$GROUND_CALIB')" 2;;
  esac
  # hard check (owner 10-16): the camera height in use (auto: the adopted one; off / check / dry run: the configured
  # one) must be >= the obstacle band's top (ROBOT_HEIGHT + 0.05) + 5 mm, else the band reaches the lens and nothing
  # ahead can be seen free -> refused with the reason (tools/car_run/car_mount.py camera_above_band)
  [ "$DRY_RUN" = 1 ] && echo "# camera above band: python3 tools/car_run/car_mount.py --height-m $CAM_HEIGHT --robot-height-m $ROBOT_HEIGHT (camera >= robot height + 0.05 + 0.005 m, else FAIL; a real start checks the ADOPTED height)"
  CAB=$(python3 "$(dirname "$0")/car_mount.py" --height-m "$CAM_HEIGHT" --robot-height-m "$ROBOT_HEIGHT" 2>&1) \
    || fail "camera above band refused the start: $CAB" 3
  ready camera_above_band "$CAB"
  # the camera mount: base_link -> camera optical frame on /tf_static (the driver runs with publish_tf:=false)
  [ -n "$CAM_MOUNT_TF" ] || fail "live: no camera mount (CAM_MOUNT_TF; car_up.sh computes it from CAR_CAM_HEIGHT / CAR_CAM_PITCH_DEG)" 2
  read -r MX MY MZ MQX MQY MQZ MQW <<< "$CAM_MOUNT_TF"
  STP=$(ros_exe tf2_ros static_transform_publisher) || fail "no tf2_ros/static_transform_publisher on AMENT_PREFIX_PATH" 2
  launch mount_tf mount_tf.log "${T[@]}" "$STP" --x "$MX" --y "$MY" --z "$MZ" \
    --qx "$MQX" --qy "$MQY" --qz "$MQZ" --qw "$MQW" --frame-id base_link --child-frame-id "$LIVE_CAMERA_FRAME"
  wait_for mount_tf 2 false; alive mount_tf || [ "$DRY_RUN" = 1 ] || fail "camera mount TF publisher exited at start"
  ready mount_tf "height ${CAM_HEIGHT} m pitch ${CAM_PITCH_DEG} deg height_cell ${HEIGHT_CELL}${GC_MOUNT_NOTE}";;
esac

# --- chassis -----------------------------------------------------------------------------------------
case "$CHASSIS" in
pty)
  [ "$RECORD" = 1 ] && launch permit_recorder permit_recorder.log "${T[@]}" python3 tools/sim_loop/permit_recorder.py \
    --kv "$KV_ADDR" --out "$O/permit_telem.jsonl"
  HB=(--heartbeat-kv "$KV_ADDR"); [ "$MCU_HEARTBEAT" = always ] && HB=()
  TW=(); [ "$CAMERA" = sim ] && TW=(--twist-topic /fpga/chassis/sim_twist)
  if [ "$DRY_RUN" = 1 ]; then
    echo "[mcu] ${T[*]} python3 tools/chassis_uart/mcu_sim.py --pty ${HB[*]} ${TW[*]} --log $O/mcu.jsonl > $O/mcu.out 2> $O/mcu.err"
    PTY='<pty printed by mcu_sim>'
  else
    "${T[@]}" python3 tools/chassis_uart/mcu_sim.py --pty "${HB[@]}" "${TW[@]}" --log "$O/mcu.jsonl" > "$O/mcu.out" 2> "$O/mcu.err" &
    NAMES+=(mcu); PIDS+=($!); STARTS+=("$(car_pid_start $!)"); echo "mcu $! ${STARTS[-1]}" >> "$O/pids.txt"
    wait_for mcu 10 test -s "$O/mcu.out" || fail "mcu_sim: no pty in 10 s"
    PTY=$(head -n 1 "$O/mcu.out")
    [ -n "$PTY" ] && [ -e "$PTY" ] || fail "mcu_sim pty not up"
  fi
  launch chassis_driver chassis_driver.log "${T[@]}" python3 -m host.ros_chassis_driver --port "$PTY" --record "$O/chassis_record.jsonl"
  wait_for chassis_driver 10 has_line chassis_record.jsonl '"dir": "rx"' || fail "chassis driver got no MCU status frame in 10 s"
  ready chassis "pty=$PTY heartbeat=$MCU_HEARTBEAT";;
uart)   # board: the real MCU on CHASSIS_PORT (tools/chassis_uart/nano_uart_setup.sh makes /dev/fpga_chassis)
  launch chassis_driver chassis_driver.log "${T[@]}" python3 -m host.ros_chassis_driver --port "$CHASSIS_PORT" --record "$O/chassis_record.jsonl"
  wait_for chassis_driver 10 has_line chassis_record.jsonl '"dir": "rx"' || fail "chassis driver got no MCU status frame on $CHASSIS_PORT in 10 s"
  ready chassis "uart=$CHASSIS_PORT";;
none)
  if [ "$CAMERA" = sim ]; then   # D1: the velocity guard drives Gazebo directly; its e-stop input is this stand-in
    launch estop_gate estop_gate.log "${T[@]}" python3 robot_sim/fpga_sim/scripts/sim_estop_gate.py --kv-server "$KV_ADDR" \
      --log "$O/estop_gate.jsonl"
    wait_for estop_gate 3 false; alive estop_gate || [ "$DRY_RUN" = 1 ] || fail "sim_estop_gate exited at start"
    ready chassis "none (sim: velocity guard + KV e-stop stand-in)"
  else
    ready chassis none
  fi;;
esac

# --- wheel odometry (10-15) -------------------------------------------------------------------------------
if [ "$WHEEL_ODOM" = 1 ]; then
  case "$CHASSIS" in pty|uart) ;; *) fail "wheel odometry needs the chassis driver (CHASSIS=$CHASSIS)" 2;; esac
  if [ "$CAMERA" = live ]; then WO_OUT=(--tf-topic /tf --nav-odom-topic /odom); WO_TF=/tf
  else WO_OUT=(--tf-topic /fpga/wheel_odom/tf); WO_TF=/fpga/wheel_odom/tf; fi   # shadow: replay/sim own odom -> base_link
  # one publisher per edge: bench_pose (static) owns odom -> base_link there
  [ "$POSE" = static ] && [ "$WO_TF" = /tf ] && fail "pose static and wheel odometry would both publish odom -> base_link" 2
  WO_P=(); [ -n "$WHEEL_DIAM_M" ] && WO_P+=(--wheel-diam-m "$WHEEL_DIAM_M")
  [ -n "$COUNTS_PER_REV" ] && WO_P+=(--counts-per-rev "$COUNTS_PER_REV"); [ -n "$TRACK_M" ] && WO_P+=(--track-m "$TRACK_M")
  WO_DESC=$(python3 -c '
import sys
from host.wheel_odom import WheelParams
d = dict(zip(("wheel_diam_m", "counts_per_rev", "track_m"), sys.argv[1:]))
kw = {k: (int(v) if k == "counts_per_rev" else float(v)) for k, v in d.items() if v}
p = WheelParams(**kw)
print("wheel_diam_m=%g counts_per_rev=%d track_m=%g placeholder=%d%s" % (p.wheel_diam_m, p.counts_per_rev, p.track_m,
      p.placeholder, " (占位轮参数 PLACEHOLDER)" if p.placeholder else ""))' "$WHEEL_DIAM_M" "$COUNTS_PER_REV" "$TRACK_M") \
    || fail "wheel parameters refused: WHEEL_DIAM_M=$WHEEL_DIAM_M COUNTS_PER_REV=$COUNTS_PER_REV TRACK_M=$TRACK_M" 2
  WO_H=(--heading "${HEADING:-wheel}"); read -r -a WO_X <<< "${WHEEL_ODOM_ARGS:-}"
  launch wheel_odom wheel_odom.log "${T[@]}" python3 -m host.ros_wheel_odom --source chassis "${WO_P[@]}" "${WO_OUT[@]}" \
    --record "$O/wheel_odom.jsonl" "${WO_H[@]}" "${WO_X[@]}"
  wait_for wheel_odom 10 test -s "$O/wheel_odom.jsonl" || fail "wheel odometry: no sample in 10 s"
  ready wheel_odom "tf=$WO_TF $WO_DESC heading=${HEADING:-wheel}${WHEEL_ODOM_ARGS:+ args=$WHEEL_ODOM_ARGS}"
fi

# --- remote control (10-15) ---------------------------------------------------------------------------------
if [ "$TELEOP" = 1 ]; then
  TX=(--socket "$IPC/teleop.sock" --record "$O/teleop.jsonl")
  [ -n "$TELEOP_V_MAX" ] && TX+=(--v-max "$TELEOP_V_MAX"); [ -n "$TELEOP_W_MAX" ] && TX+=(--w-max "$TELEOP_W_MAX")
  [ -n "$TELEOP_MAX_CMD_S" ] && TX+=(--max-cmd-s "$TELEOP_MAX_CMD_S")
  # teleop + nav2 = arbitration (10-18): the velocity guard publishes /fpga/nav/cmd_vel_safe (board: FPGA_NAV_GUARD_OUTPUT
  # below; sim: car_up GZ_ENV FPGA_SIM_GUARD_OUTPUT), the teleop node passes it on while the remote control is not in hand
  [ "$NAV" = nav2 ] && TX+=(--nav-topic /fpga/nav/cmd_vel_safe)
  launch teleop teleop.log "${T[@]}" python3 -m host.ros_teleop_safe "${TX[@]}"
  wait_for teleop 15 test -S "$IPC/teleop.sock" || fail "teleop: no input socket $IPC/teleop.sock in 15 s"
  if [ "$NAV" = nav2 ]; then
    ready teleop "socket $IPC/teleop.sock v_max=${TELEOP_V_MAX:-0.15} w_max=${TELEOP_W_MAX:-0.6} max_cmd_s=${TELEOP_MAX_CMD_S:-3} -- ARBITRATION with Nav2 (/fpga/nav/cmd_vel_safe passes while not in hand; new navigation refused while in hand)"
  else
    ready teleop "socket $IPC/teleop.sock v_max=${TELEOP_V_MAX:-0.15} w_max=${TELEOP_W_MAX:-0.6} max_cmd_s=${TELEOP_MAX_CMD_S:-3} -- NOT a navigation run"
  fi
fi

# 10-25 (owner plan A): in the mapping run bench_pose re-sends map -> odom as the alignment mapping_ctl.py publishes at
# the mapping stop (host/map_align.py); the runner moves its memory with it (--memory-align below)
BP_ALIGN=(); [ "$MAPPING" = 1 ] && BP_ALIGN=(--map-align)
# --- in-stack mapping start map (10-18, MAPPING=1): the FREE blank map is made before the pose block, so that the
# board's MAP_YAML (map_server's start map, the pose block's check and its `ready pose` mark) is the real file and not
# car_up's placeholder (relay 3 fix; the mapping process itself starts after bench_pose, below)
if [ "$MAPPING" = 1 ]; then
  run1 blank_map blank_map.log python3 tools/car_run/blank_map.py --out-dir "$O/blank_map" --size-m "$MAPPING_BLANK_M" \
    || fail "blank start map ($O/blank_map)" 2
  [ "$PLATFORM" = board ] && MAP_YAML="$O/blank_map/blank.yaml"
fi

# --- pose (rtabmap localization / mapping: tools/car_run/rtabmap_car.sh loc|map, STANDALONE=0 in this container) --
# camera topics: live = the driver's (LIVE_*; the mount TF is car_inner's, CAM_MOUNT_TF empty there); replay (map
# only) = the replay's /fpga/rgb|depth|camera_info, best effort (QOS 2) and RTAB_PUBLISH_TF=false: the replay owns
# map -> odom -> base_link (RTABMAP_ODOM wheel: rtabmap reads odom -> base_link from TF).
# Ready: rtabmap_car.sh logs "running --" (to stderr = rtabmap.log) once its odometry start check passed; loc also
# needs the gate's /fpga/localization_status. Wait RTABMAP_READY_S (>= 120: 3 + 2 x 30 s odometry checks + the gate).
if [ "$POSE" = rtabmap ] || [ "$POSE" = map ]; then
  RT_MODE=loc; [ "$POSE" = map ] && RT_MODE=map
  if [ "$CAMERA" = live ]; then
    RTT=(COLOR_TOPIC="$LIVE_COLOR_TOPIC" DEPTH_TOPIC="$LIVE_DEPTH_TOPIC" INFO_TOPIC="$LIVE_INFO_TOPIC" CAMERA_FRAME="$LIVE_CAMERA_FRAME")
  else
    RTT=(COLOR_TOPIC=/fpga/rgb DEPTH_TOPIC=/fpga/depth INFO_TOPIC=/fpga/camera_info QOS=2 RTAB_PUBLISH_TF=false)
  fi
  if [ "$RT_MODE" = loc ]; then
    [ -n "$RTABMAP_DB" ] || [ "$DRY_RUN" = 1 ] || fail "rtabmap: RTABMAP_DB (pinned localization database) not set" 2
    MAP_ID=$([ -f "$RTABMAP_DB" ] && sha256sum "$RTABMAP_DB" | cut -c1-64 || echo '<sha256 of RTABMAP_DB>')
    RT_RUN=$(date +%Y%m%d_%H%M%S)
    RTO=(DB="${RTABMAP_DB:-<RTABMAP_DB>}" OUT="$RTABMAP_DB_DIR/run_loc_$RT_RUN")
  else
    RT_RUN=$(date +%Y%m%d_%H%M%S)   # rtabmap_car.sh map: DB_DIR/<RT_RUN>.db, OUT DB_DIR/run_<RT_RUN>
    RTO=(RUN_TIME="$RT_RUN" OUT="$RTABMAP_DB_DIR/run_$RT_RUN")
    mark "rtabmap map: db $RTABMAP_DB_DIR/$RT_RUN.db out $RTABMAP_DB_DIR/run_$RT_RUN"
    # the mount this map is built with (10-15): beside the database, for the localization runs to compare against
    if [ "$CAMERA" = live ] && [ "$GROUND_CALIB" != off ]; then
      if [ "$DRY_RUN" = 1 ]; then echo "# ground_calib: $O/ground_calib.json -> $RTABMAP_DB_DIR/$RT_RUN.ground_calib.json (beside the new database)"
      else cp "$O/ground_calib.json" "$RTABMAP_DB_DIR/$RT_RUN.ground_calib.json" && mark "ground_calib stored beside the map: $RTABMAP_DB_DIR/$RT_RUN.ground_calib.json" \
        || mark "ground_calib NOT stored beside the map ($RTABMAP_DB_DIR/$RT_RUN.ground_calib.json)"; fi
    fi
  fi
  launch rtabmap rtabmap.log "${T[@]}" env STANDALONE=0 MODE="$RT_MODE" ODOM="$RTABMAP_ODOM" DB_DIR="$RTABMAP_DB_DIR" \
    "${RTO[@]}" "${RTT[@]}" GATE_ODOM_JUMP="$RTABMAP_GATE_ODOM_JUMP" GATE_MAX_ODOM_SPEED_MPS="$RTABMAP_GATE_ODOM_V_MPS" \
    GATE_ODOM_JUMP_MARGIN_M="$RTABMAP_GATE_ODOM_E_M" CAM_MOUNT_TF= CAR_WHEEL_ODOM="$WHEEL_ODOM" SAVE_TIMEOUT_S=120 \
    GATE_COUNT_MODE="$RTABMAP_GATE_MODE" GATE_MAX_CORRECTION_AGE_S="$RTABMAP_GATE_X_S" \
    bash tools/car_run/rtabmap_car.sh "$RT_MODE"
  wait_for rtabmap "$RTABMAP_READY_S" has_line rtabmap.log 'running --' \
    || fail "rtabmap ($RT_MODE): no 'running --' in rtabmap.log in $RTABMAP_READY_S s"
  if [ "$RT_MODE" = loc ]; then
    wait_for rtabmap "$RTABMAP_READY_S" topics_have /fpga/localization_status \
      || fail "rtabmap loc: no /fpga/localization_status in $RTABMAP_READY_S s"
    ready pose "rtabmap loc db=${RTABMAP_DB:-<RTABMAP_DB>} odom=$RTABMAP_ODOM (rtabmap.log 'running --' + /fpga/localization_status, <= $RTABMAP_READY_S s)"
  else
    ready pose "rtabmap map (NEW database, mapping run -- not localization) odom=$RTABMAP_ODOM (rtabmap.log 'running --', <= $RTABMAP_READY_S s)"
  fi
elif [ "$POSE" = static ]; then   # line-4 bench: the camera is fixed and nothing can move the car
  launch bench_pose bench_pose.log "${T[@]}" python3 tools/car_run/bench_pose.py --odom-hz "$BENCH_ODOM_HZ" --map-m "$BENCH_MAP_M" \
    ${BP_ALIGN[@]+"${BP_ALIGN[@]}"}
  wait_for bench_pose 2 false; alive bench_pose || [ "$DRY_RUN" = 1 ] || fail "bench pose publisher exited at start"
  ready pose "static: bench, camera fixed, no chassis -- map->odom->base_link identity (odom_hz $BENCH_ODOM_HZ), stand-in free /map ${BENCH_MAP_M} m; NOT mapping/localization"
elif [ "$POSE" = odom ]; then   # the real car: wheel odometry from the start point
  [ "$WHEEL_ODOM" = 1 ] && [ "${WO_TF:-}" = /tf ] || fail "pose odom needs the wheel odometry on /tf (WHEEL_ODOM=1, live)" 2
  if [ "$NAV" = nav2 ]; then   # 10-17: map_server owns /map (MAP_YAML, the mapping export); bench_pose: map -> odom only
    [ -n "$MAP_YAML" ] || fail "pose odom + nav2 needs MAP_YAML (the mapping export for Nav2's map_server; car_up CAR_MAP_YAML / RTABMAP_DB)" 2
    launch bench_pose bench_pose.log "${T[@]}" python3 tools/car_run/bench_pose.py --no-odom-base --map-m 0 \
      ${BP_ALIGN[@]+"${BP_ALIGN[@]}"}
    wait_for bench_pose 2 false; alive bench_pose || [ "$DRY_RUN" = 1 ] || fail "map -> odom publisher exited at start"
    ready pose "odom + nav2: map->odom identity (start = origin) + odom->base_link from the wheel odometry ($WO_DESC); no stand-in /map -- Nav2 map_server publishes $MAP_YAML (right only when the car started at the mapping start, same heading); NOT mapping/localization"
  elif [ -n "$MAP_YAML" ]; then   # the mapping export (car_up checked it is a file)
    launch bench_pose bench_pose.log "${T[@]}" python3 tools/car_run/bench_pose.py --no-odom-base --map-yaml "$MAP_YAML" \
      ${BP_ALIGN[@]+"${BP_ALIGN[@]}"}
    wait_for bench_pose 2 false; alive bench_pose || [ "$DRY_RUN" = 1 ] || fail "map -> odom publisher exited at start (map $MAP_YAML)"
    ready pose "odom: map->odom identity (start = origin) + odom->base_link from the wheel odometry ($WO_DESC), /map = mapping export $MAP_YAML (right only when the car started at the mapping start, same heading); NOT mapping/localization"
  else
    launch bench_pose bench_pose.log "${T[@]}" python3 tools/car_run/bench_pose.py --no-odom-base --map-m "$BENCH_MAP_M" \
      ${BP_ALIGN[@]+"${BP_ALIGN[@]}"}
    wait_for bench_pose 2 false; alive bench_pose || [ "$DRY_RUN" = 1 ] || fail "map -> odom publisher exited at start"
    ready pose "odom: map->odom identity (start = origin) + odom->base_link from the wheel odometry ($WO_DESC), stand-in free /map ${BENCH_MAP_M} m; NOT mapping/localization"
  fi
fi

# --- in-stack mapping (10-18, MAPPING=1 = car_up CAR_MAPPING=1; docs/design/TELEOP_NAV_ARB_20261018.md section 2) ----
# Nav2 starts on a FREE blank map ($O/blank_map, MAP_YAML for the board map_server; the host simulation keeps its
# world map); rtabmap maps beside it (rtabmap_car.sh map, ODOM=wheel: odom -> base_link from the wheel odometry /
# the simulation, RTAB_PUBLISH_TF=false: map -> odom stays bench_pose's identity / the simulation's, MAP_NS=rtabmap:
# its grid on /fpga/rtabmap/map, map_server keeps /map). tools/car_run/mapping_ctl.py owns it: /fpga/mapping/stop
# (hub op mapping_stop) -> save + export -> /map_server/load_map; status on /fpga/mapping/status. Host: the stand-in
# tools/car_run/mapping_standin.py (no rtabmap in the host image; control flow only, marked STAND-IN).
if [ "$MAPPING" = 1 ]; then
  MP_RUN=$(date +%Y%m%d_%H%M%S)
  MP_OUT="$RTABMAP_DB_DIR/run_$MP_RUN"
  [ "$DRY_RUN" = 1 ] || mkdir -p "$MP_OUT"
  if [ "$PLATFORM" = board ]; then
    MP_CMD=(env STANDALONE=0 MODE=map ODOM=wheel RTAB_PUBLISH_TF=false MAP_NS=rtabmap DB_DIR="$RTABMAP_DB_DIR"
            RUN_TIME="$MP_RUN" OUT="$MP_OUT" COLOR_TOPIC="$LIVE_COLOR_TOPIC" DEPTH_TOPIC="$LIVE_DEPTH_TOPIC"
            INFO_TOPIC="$LIVE_INFO_TOPIC" CAMERA_FRAME="$LIVE_CAMERA_FRAME" CAM_MOUNT_TF= CAR_WHEEL_ODOM="$WHEEL_ODOM"
            SAVE_TIMEOUT_S=120 bash tools/car_run/rtabmap_car.sh map)
    MP_SI=()
  else
    MP_CMD=(env DB_DIR="$RTABMAP_DB_DIR" RUN_TIME="$MP_RUN" OUT="$MP_OUT" python3 tools/car_run/mapping_standin.py)
    MP_SI=(--standin)
    mark "mapping: STAND-IN (tools/car_run/mapping_standin.py; host image has no rtabmap) -- control flow only"
  fi
  mark "mapping: db $RTABMAP_DB_DIR/$MP_RUN.db out $MP_OUT (in-stack, beside Nav2)"
  # 10-26 map reset (console 重置地图, host/map_reset.py): --run-id lets mapping_ctl restart MP_CMD under a new run
  # name after the stop; --start-map-yaml = the map Nav2 started on (board: the blank map), loaded back then
  MP_RS=(--run-id "$MP_RUN"); [ -n "${MAP_YAML:-}" ] && MP_RS+=(--start-map-yaml "$MAP_YAML")
  # 10-27 live alignment (car_up CAR_LIVE_ALIGN, default 1): map -> odom + the memory follow loop closures while mapping
  MP_LIVE=(); [ "$LIVE_ALIGN" = 1 ] && MP_LIVE=(--live-align)
  launch mapping mapping.log "${T[@]}" python3 tools/car_run/mapping_ctl.py --status-file "$O/mapping_status.json" \
    --stop-marker "$O/mapping_stop_requested" --out "$MP_OUT" --db-dir "$RTABMAP_DB_DIR" --db "$RTABMAP_DB_DIR/$MP_RUN.db" --stop-wait-s "$RTABMAP_STOP_S" \
    --search-waypoints-out "$O/search_waypoints.json" "${MP_RS[@]}" ${MP_LIVE[@]+"${MP_LIVE[@]}"} \
    ${MP_SI[@]+"${MP_SI[@]}"} -- "${MP_CMD[@]}"
  wait_for mapping "$RTABMAP_READY_S" has_line mapping.log 'running --' \
    || fail "mapping: no 'running --' in mapping.log in $RTABMAP_READY_S s"
  ready mapping "in-stack mapping (rtabmap map, wheel odometry, grid /fpga/rtabmap/map; $([ "$PLATFORM" = board ] || echo 'STAND-IN, ')stop: hub mapping_stop -> export -> /map_server/load_map); Nav2 start map: $([ "$PLATFORM" = board ] && echo "blank $MAPPING_BLANK_M m" || echo 'the simulation world map')"
fi

# --- navigation --------------------------------------------------------------------------------------
# kv_nav_proxy_step (CTRLPS2, NAV_BACKEND kv|shadow, after Nav2 is active): host/ros_kv_nav_proxy.py. kv: its
# navigate_to_pose is the runner's, so no "ready" line in 30 s or a KV without "nav" (refused) fails the start (4);
# shadow: a WARN and the stack goes on (Nav2 drives either way). Records $O/kv_nav_cmds.jsonl.
kv_nav_proxy_step() {
  local name=kvnav mode=$NAV_BACKEND ok=1 end
  [ "$mode" = shadow ] && name=kvshadow
  launch "$name" kv_nav_proxy.log "${T[@]}" python3 -m host.ros_kv_nav_proxy --kv-server "$KV_ADDR" --mode "$mode" \
    --record "$O/kv_nav_cmds.jsonl"
  if [ "$DRY_RUN" != 1 ]; then
    ok=0; end=$((SECONDS + 30))
    while [ "$SECONDS" -lt "$end" ]; do
      has_line kv_nav_proxy.log '"event": "ready"' && { ok=1; break; }
      has_line kv_nav_proxy.log '"event": "refused"' && break
      alive "$name" || break
      sleep 0.2
    done
  fi
  if [ "$ok" = 1 ]; then
    ready "$name" "KV260 navigation ($mode) via $KV_ADDR (kv_nav_proxy.log, kv_nav_cmds.jsonl)"
  elif [ "$mode" = kv ]; then
    fail "kv navigation: proxy not ready in 30 s or the KV does not serve nav (kv_nav_proxy.log; kv_server --nav soft|pl)" 4
  else
    mark "WARN kvshadow: proxy not ready (kv_nav_proxy.log) -- Nav2 drives, no KV shadow record"
    echo "WARN kvshadow: no KV shadow record (kv_nav_proxy.log)" >&2
  fi
}
# nav_step: Nav2 (nav2), the standalone costmap (costmap) or nothing (pose map). Called here, or -- board nav2 with the
# rolling costmap (NAV_AFTER_PRODUCER=1) -- after the producer below. Same commands either way; only the place differs.
nav_step() {
if [ "$NAV" = nav2 ]; then
  # board (10-15): the launch FILE from the source snapshot (/src; the board image has no fpga_sim package), the
  # real-car switches of nav_input_profile.car_inputs (FPGA_NAV_SPEED_LIMITS=car: the car speed limits and the 0.10 m
  # xy goal tolerance), the map exported from the pinned database (MAP_YAML), FPGA_NAV_COSTMAP = CAR_NAV_COSTMAP
  # (car_nav_costmap: rolling unless NAV_COSTMAP=sim). rolling: the overrides file is the DWB one (dwb.yaml of
  # nav_controller_variants.py = config/nav2_overrides.yaml) -- the rolling merge rebuilds FollowPath as DWB anyway
  # and would drop an RPP file's controller keys; its yaw goal tolerance is nav2_overrides.yaml's, not
  # rpp_heading_precision's 0.005. Host and board sim: rpp_heading_precision.yaml as before.
  NAVL=(ros2 launch fpga_sim navigation.launch.py)
  NAVE=()
  NAV_OVR=rpp_heading_precision.yaml
  if [ "$PLATFORM" = board ]; then
    NAVL=(ros2 launch "$SRCD/robot_sim/fpga_sim/launch/navigation.launch.py")
    NAVE=(FPGA_NAV_USE_SIM_TIME=false FPGA_NAV_STATIC_MAP_ODOM=false FPGA_NAV_SIM_CAMERA_TF=false FPGA_NAV_SPEED_LIMITS=car
          FPGA_NAV_VELOCITY_GUARD=true FPGA_NAV_MAP_YAML="$MAP_YAML" FPGA_NAV_COSTMAP="$CAR_NAV_COSTMAP")
    [ "$CAR_NAV_COSTMAP" = rolling ] && NAV_OVR=dwb.yaml
    [ "$TELEOP" = 1 ] && NAVE+=(FPGA_NAV_GUARD_OUTPUT=/fpga/nav/cmd_vel_safe)   # teleop arbitration (10-18)
    # 10-26 (car_up header): the car Nav2 switches and DWB minimum speeds, each only when given (unset = the line of
    # before and the launch's defaults / nav_input_profile.py's constants)
    for _v in FPGA_NAV_ROTATION_SHIM FPGA_NAV_PROGRESS_CHECKER FPGA_NAV_CAR_PLANNER; do
      [ -n "${!_v:-}" ] && NAVE+=("$_v=${!_v}"); done
    [ -n "${DWB_MIN_SPEED_THETA:-}" ] && NAVE+=(FPGA_NAV_DWB_MIN_SPEED_THETA="$DWB_MIN_SPEED_THETA")
    [ -n "${DWB_MIN_SPEED_XY:-}" ] && NAVE+=(FPGA_NAV_DWB_MIN_SPEED_XY="$DWB_MIN_SPEED_XY")
    [ -n "${ROT_STOPPED_VELOCITY:-}" ] && NAVE+=(FPGA_NAV_ROT_STOPPED_VELOCITY="$ROT_STOPPED_VELOCITY")
    [ -n "${DWB_FORWARD_PRUNE_M:-}" ] && NAVE+=(FPGA_NAV_DWB_FORWARD_PRUNE_M="$DWB_FORWARD_PRUNE_M")   # 10-29
  elif [ "${SIM_NAV_PROFILE:-}" = car ]; then
    # 10-26 car_up CAR_SIM_NAV_PROFILE=car: the car's Nav2 switches in the simulation -- the car's controller too
    # (DWB = dwb.yaml, wrapped in the rotation shim, which refuses RPP); the simulation's map and costmap layers stay
    NAVE=(FPGA_NAV_SPEED_LIMITS=car)
    NAV_OVR=dwb.yaml
  fi
  # 10-29 find while mapping (car_up CAR_NAV_LIVE_GLOBAL, default 1; mapping run with live alignment only): Nav2's
  # GLOBAL static_layer reads /fpga/nav/global_map from host/ros_nav_global_map.py -- map_server's map with rtabmap's
  # live occupied cells on top while mapping (unknown stays plannable, places the car drove stay free, a failed plan
  # -> the plain map for 20 s), map_server's map as it is otherwise (the export after the stop). Started before Nav2
  # with the start map, so the costmap has a map at activation. Not publishing -> Nav2 stays on /map (as before) and
  # the stack goes on (WARN mark); an exit later is restarted (car_common.sh car_exit_action).
  if [ "$MAPPING" = 1 ] && [ "$LIVE_ALIGN" = 1 ] && [ "$NAV_LIVE_GLOBAL" = 1 ]; then
    NG=(--record "$O/nav_global_map.jsonl"); [ -n "${MAP_YAML:-}" ] && NG+=(--start-map-yaml "$MAP_YAML")
    # 10-29 nav-turn-start (car_up CAR_NAV_GLOBAL_ROLLING, default 1): the rolling local layer's OCCUPIED cells go in
    # too, so a global path keeps the local layer's clearance (no path through / grazing what only the local layer
    # has, which DWB cannot follow); 0 = rtabmap's cells only (as before)
    [ "$NAV_GLOBAL_ROLLING" = 1 ] || NG+=(--rolling-topic "")
    launch navmap nav_global_map.log "${T[@]}" python3 -m host.ros_nav_global_map "${NG[@]}"
    NG_OK=1
    if [ "$DRY_RUN" != 1 ]; then
      NG_OK=0; NG_END=$((SECONDS + 20))
      while [ "$SECONDS" -lt "$NG_END" ]; do
        if [ -z "${MAP_YAML:-}" ]; then sleep 2; alive navmap && NG_OK=1; break; fi   # host sim: waits for /map
        has_line nav_global_map.log '"event": "published"' && { NG_OK=1; break; }
        alive navmap || break
        sleep 0.2
      done
    fi
    if [ "$NG_OK" = 1 ]; then
      NAVE+=(FPGA_NAV_GLOBAL_MAP_TOPIC=/fpga/nav/global_map)
      ready navmap "Nav2 global static_layer on /fpga/nav/global_map (live occupied cells while mapping; map_server's map otherwise; nav_global_map.log)"
    else
      mark "WARN navmap: no global map from host/ros_nav_global_map.py in 20 s (nav_global_map.log) -- Nav2's global costmap stays on /map, the stack continues"
      echo "WARN navmap: Nav2's global costmap stays on /map (nav_global_map.log)" >&2
    fi
  fi
  # CTRLPS2: kv = Nav2 reduced to map_server + planner_server (navigation.launch.py FPGA_NAV_BACKEND=kv)
  [ "$NAV_BACKEND" = kv ] && NAVE+=(FPGA_NAV_BACKEND=kv)
  run1 nav2_configs controller-generation.log "${T[@]}" python3 robot_sim/tests/nav_controller_variants.py \
    --output "$O/controllers" --source-revision "$HEAD" || fail "Nav2 controller generation"
  launch navigation navigation.log "${T[@]}" env FPGA_NAV2_OVERRIDES="$O/controllers/$NAV_OVR" \
    FPGA_NAV_AUTOSTART=false "${NAVE[@]}" "${NAVL[@]}"
  if [ "$CAR_NAV_COSTMAP" = rolling ]; then
    # the local costmap's input: one /fpga/rolling_map/grid message (latched) before activation, else no Nav2
    run1 rolling_grid grid_probe.log "${T[@]}" python3 tools/car_run/grid_probe.py --topic /fpga/rolling_map/grid \
      --timeout-s "$GRID_READY_S" || fail "rolling costmap: no /fpga/rolling_map/grid from the bridge in $GRID_READY_S s (grid_probe.log, bridge.log, producer.log)" 4
    ready rolling_grid "/fpga/rolling_map/grid received before Nav2 activation"
  fi
  PROBE_ENV=(); [ "$NAV_BACKEND" = kv ] && PROBE_ENV=(FPGA_NAV_PROBE_NODES=map_server,planner_server)   # CTRLPS2
  run1 nav2_activate startup-probe.log "${T[@]}" "${PROBE_ENV[@]}" timeout "$NAV_READY_S" python3 robot_sim/tests/nav_startup_probe.py \
    --activate --all-services-first --out "$O/startup-probe.jsonl" || fail "Nav2 start/activation failed (nav_startup_probe, no retry)" 4
  if [ "$PLATFORM" = board ]; then ready nav2 "costmap=$CAR_NAV_COSTMAP overrides=$NAV_OVR"; else ready nav2; fi   # host mark unchanged
  [ "$NAV_BACKEND" = nav2 ] || kv_nav_proxy_step
elif [ "$POSE" = map ]; then   # mapping run: nothing to navigate on yet (car_validate refuses nav2 with map)
  mark "skipped costmap (pose map)"; [ "$DRY_RUN" = 1 ] && echo "# costmap: skipped (pose map: mapping run, no navigation)"
else
  run1 costmap_params costmap_check.json python3 tools/car_geo_producer/costmap_params.py --out "$O/costmap.yaml" \
    || fail "costmap parameters"
  CMX=$(ros_exe nav2_costmap_2d nav2_costmap_2d) || fail "no nav2_costmap_2d/nav2_costmap_2d on AMENT_PREFIX_PATH" 2
  LMX=$(ros_exe nav2_lifecycle_manager lifecycle_manager) || fail "no nav2_lifecycle_manager/lifecycle_manager on AMENT_PREFIX_PATH" 2
  launch costmap costmap.log "${T[@]}" "$CMX" --ros-args --params-file "$O/costmap.yaml"
  launch lifecycle lifecycle.log "${T[@]}" "$LMX" --ros-args \
    -r __node:=check_lifecycle --params-file "$O/costmap.yaml"
  wait_for lifecycle "$NAV_READY_S" has_line lifecycle.log "Managed nodes are active" || fail "costmap not active in $NAV_READY_S s"
  ready costmap
fi
}
if [ "$NAV_AFTER_PRODUCER" = 1 ]; then
  mark "navigation deferred: after hub, bridge, producer (rolling costmap needs /fpga/rolling_map/grid)"
  [ "$DRY_RUN" = 1 ] && echo "# navigation: after the producer (board nav2, rolling costmap: /fpga/rolling_map/grid comes from the bridge fed by the producer)"
else
  nav_step
fi
if [ "$RECORD" = 1 ] && [ "$CAMERA" = sim ]; then
  REC=(); [ "$CHASSIS" = pty ] && REC=(--odom-period-s 0 --clock-period-s 0.005 --twist-topic /fpga/chassis/sim_twist)
  launch motion motion-recorder.log "${T[@]}" python3 robot_sim/tests/record_motion.py --out "$O/motion.jsonl" --seconds 100000000 "${REC[@]}"
  if [ "$CHASSIS" = pty ] && [ "$DRY_RUN" != 1 ]; then
    { echo "== ign topic -i -t /fpga/cmd_vel"; timeout 20 ign topic -i -t /fpga/cmd_vel;
      for t in /fpga/chassis/sim_twist /fpga/cmd_vel_safe; do echo "== ros2 topic info -v $t"; timeout 30 ros2 topic info -v "$t" --no-daemon --spin-time 5 2>/dev/null || timeout 30 ros2 topic info -v "$t"; done
    } > "$O/single_source.txt" 2>&1
    mark "record single source check"
  fi
fi

# --- hub (+ test-only display PC) --------------------------------------------------------------------
if [ "$DRY_RUN" = 1 ]; then echo "# token: python3 -c secrets.token_hex(24) > $O/token.txt (umask 077)"
else (umask 077; python3 -c "import secrets;print(secrets.token_hex(24))" > "$O/token.txt"); fi
PREVIEW_ARGS=()
if [ "${PREVIEW:-0}" = 1 ]; then
  PREVIEW_ARGS=(--preview)
  launch preview preview.log "${T[@]}" python3 tools/car_run/preview_publisher.py
  wait_for preview 20 has_line preview.log 'preview ready:' || fail "preview subscriber did not start"
fi
launch hub hub.log "${T[@]}" python3 -m host.hub.hub_ros --data-source "$SOURCE" --token-file "$O/token.txt" --port "$HUBPORT" \
  --bind "$HUB_BIND" --kv "$KV_ADDR" --stats-out "$O/hub_stats.json" "${PREVIEW_ARGS[@]}"
wait_for hub 30 has_line hub.log 'hub listening' || fail "hub not listening in 30 s"
ready hub "$HUB_BIND:$HUBPORT"
if [ "$RECORD" = 1 ]; then
  launch pc_listen pc_listen.log "${T[@]}" python3 pc/fpgapc.py --host "$HUB_BIND" --port "$HUBPORT" --token-file "$O/token.txt" \
    listen --yield-control --out "$O/pc.fpgarec" --stats "$O/pc_stats.json"
  wait_for pc_listen 20 has_line pc_listen.log '^YIELD' || fail "record: display PC did not yield control"
fi

# --- rolling map bridge, geometry producer -----------------------------------------------------------
# body_margin_m 0.05 (10-17 nav go-live r3, REAL_CAR_NAV_GOLIVE_20261016.md 9): the body seed (UNKNOWN -> FREE
# around the CURRENT pose only) covers Nav2's rasterised outline; with 0 the board's DWB rejected all 419
# trajectories at the start pose (3 UNKNOWN outline cells at 0.30-0.304 m). Cap 0.05 is in the bridge.
# FRONT_SEED_M (10-26, car_up CAR_FRONT_SEED_M, empty = off = the line unchanged): front-sector seed, see the bridge.
# SWEPT_CLEAR_S (10-28, owner approved; default 300, 0 = off): cells the body covered stay FREE that long unless a
# >= 15 cm obstacle is seen within 1 m (host/rolling_nav_layer.py SweptCells), against ground-fit false obstacles.
# B1 (nationals 11-02, runs/nationals-prelim-20261102/B1/NANO_SIDE.md; car_up CAR_PL_GRID / CAR_PL_GRID_SHADOW):
# CAR_PL_GRID=0 (default): the two lines below exactly as before. =1: the producer publishes /fpga/rolling_map/grid,
# /grid_raw and /status itself from the KV260's navigation planes (same body seed, swept clearing, 200 ms freshness)
# and the bridge is not started; CAR_PL_GRID_SHADOW=1 starts the bridge anyway on the _sw topics (cell-by-cell car
# comparison, tools/b1_costmap/shadow_compare.py). A KV260 without the grid op: the producer warns and builds the same
# grid in process from its own deltas (status grid_source nano_sw).
: "${CAR_PL_GRID:=0}" "${CAR_PL_GRID_SHADOW:=0}"
if [ "$CAR_PL_GRID" != 1 ]; then
launch bridge bridge.log "${T[@]}" python3 host/ros_rolling_nav.py --ros-args -p ground_z_m:=0.0 -p swept_clear_s:="${SWEPT_CLEAR_S:-300.0}" -p robot_height_m:="$ROBOT_HEIGHT" -p robot_radius_m:=0.30 -p body_margin_m:=0.05 ${FRONT_SEED_M:+-p front_seed_m:=$FRONT_SEED_M}
wait_for bridge 3 false; alive bridge || [ "$DRY_RUN" = 1 ] || fail "rolling bridge exited at start"
ready bridge
elif [ "$CAR_PL_GRID_SHADOW" = 1 ]; then
launch bridge bridge.log "${T[@]}" python3 host/ros_rolling_nav.py --ros-args -p ground_z_m:=0.0 -p swept_clear_s:="${SWEPT_CLEAR_S:-300.0}" -p robot_height_m:="$ROBOT_HEIGHT" -p robot_radius_m:=0.30 -p body_margin_m:=0.05 ${FRONT_SEED_M:+-p front_seed_m:=$FRONT_SEED_M} \
  -p grid_topic:=/fpga/rolling_map/grid_sw -p raw_topic:=/fpga/rolling_map/grid_raw_sw -p status_topic:=/fpga/rolling_map/status_sw
wait_for bridge 3 false; alive bridge || [ "$DRY_RUN" = 1 ] || fail "rolling bridge (shadow) exited at start"
ready bridge "shadow: /fpga/rolling_map/grid_sw"
else
mark "bridge not started (CAR_PL_GRID=1: the producer publishes /fpga/rolling_map/grid from the KV260 planes)"
[ "$DRY_RUN" = 1 ] && echo "# bridge: not started (CAR_PL_GRID=1, grid + status from the producer)"
fi
PX=(--data-source "$SOURCE" --backend remote_pl --kv-server "$KV_ADDR")
[ -n "$KV_BUILD_ID" ] && PX+=(--kv-expect-build-id "$KV_BUILD_ID")
[ "${ROLLING_BACKEND:-cpu}" != cpu ] && PX+=(--rolling-backend "$ROLLING_BACKEND")   # 10-26 FPGA profile v3fc: fuse on the KV PL
[ -n "$HEIGHT_CELL" ] && PX+=(--height-cell "$HEIGHT_CELL")   # live: from the camera height (car_mount.py)
[ "$PRODUCER_DEPTH_QOS" != auto ] && PX+=(--depth-qos "$PRODUCER_DEPTH_QOS")   # T2 switch (default: not passed)
[ "$CAMERA" = sim ] && PX+=(--depth-topic /fpga/depth16 --info-topic /fpga/camera_info --camera-frame camera_optical_frame)
# 10-27 geometry v2 (car_up CAR_GEOMETRY_V2=1): the producer's ROI/thresholds/words and the runtime (below) together
[ "${GEOMETRY_V2:-0}" = 1 ] && PX+=(--geometry-v2) && mark "geometry v2 ON (producer + runtime)"
# B1: the producer's grid outputs, the bridge's parameters (CAR_PL_GRID=0: nothing added)
if [ "$CAR_PL_GRID" = 1 ]; then
  PX+=(--pl-grid "$([ "$CAR_PL_GRID_SHADOW" = 1 ] && echo shadow || echo on)" --grid-robot-height-m "$ROBOT_HEIGHT"
       --grid-robot-radius-m 0.30 --grid-body-margin-m 0.05 --grid-swept-clear-s "${SWEPT_CLEAR_S:-300.0}")
  [ -n "$FRONT_SEED_M" ] && PX+=(--grid-front-seed-m "$FRONT_SEED_M")
  mark "PL navigation grid ON (producer --pl-grid, shadow=$CAR_PL_GRID_SHADOW)"
fi
# AR2NANO2 (car_up CAR_AR_NANO=1): the producer draws the car screen on this Nano (display only, never stops it)
[ "${AR_NANO:-0}" = 1 ] && PX+=(--ar-nano --ar-nano-out "$O/ar_nano" --ar-nano-colour-port "${AR_COLOUR_PORT:-47801}") \
  && mark "car screen on this Nano (producer --ar-nano, logs ar_nano/)"
launch producer producer.log "${T[@]}" python3 -m host.ros_geo_producer "${PX[@]}" "${LIVE[@]}" --ground-z-m 0.0 --rate-hz "$PRODUCER_HZ" \
  --stats-out "$O/frames.jsonl" --summary-out "$O/producer_summary.json"
[ "$DRY_RUN" = 1 ] || echo "${PIDS[-1]}" > "$O/producer.pid"
wait_for producer "$PRODUCER_READY_S" first_frame_row || fail "producer: no frame row in $PRODUCER_READY_S s"
ready producer
# 10-26 car screen colour (car_up CAR_AR_COLOUR=1): answers the KV AR core's colour requests; it subscribes to the
# colour topic only while the KV asks (--lazy), so with the KV AR off it just sends a small HELLO every 0.5 s.
# Display only: not waited for, and its exit never fail-stops the stack (car_common.sh car_exit_action: colour = log).
if [ "${AR_COLOUR:-0}" = 1 ]; then
  # 10-29: below 30 fps the camera rate goes into the HELLO, the KV pairs with half that period (30: nothing added)
  CFX=(); [ "$CAMERA_FPS" = 30 ] || CFX=(--camera-fps "$CAMERA_FPS")
  # AR2NANO2: with AR_NANO=1 the colour goes to the producer's ColourSink on this Nano (127.0.0.1), not the KV
  CAH=${KV_ADDR%:*}; [ "${AR_NANO:-0}" = 1 ] && CAH=127.0.0.1
  launch colour colour.log "${T[@]}" python3 tools/kv_link/nano_colour.py --kv "$CAH:${AR_COLOUR_PORT:-47801}" \
    --topic "${LIVE_COLOR_TOPIC:-/camera/color/image_raw}" --lazy --stats-out "$O/colour_stats.json" "${CFX[@]}"
  mark "colour sender started -> $CAH:${AR_COLOUR_PORT:-47801} (log colour.log)"
fi
[ "$NAV_AFTER_PRODUCER" = 1 ] && nav_step   # board nav2, rolling costmap (see "navigation" above)

# --- inference (stub here; ssdlite/yolo is the host service or the board service, started outside) ----
if [ "$POSE" = map ]; then   # mapping run: no inference, no runner (the hub, bridge and producer stay: KV heartbeat)
  mark "skipped stub runtime runtime_info (pose map)"
  [ "$DRY_RUN" = 1 ] && echo "# stub, runtime, runtime_info: skipped (pose map: mapping run)"
elif [ "$DETECTOR" = stub ]; then
  launch stub stub.log "${T[@]}" python3 -B tools/car_geometry_kv/stub_inference.py --socket "$IPC/inf.sock"
  wait_for stub 20 test -S "$IPC/inf.sock" || fail "stub inference: no socket in 20 s"
  ready stub
fi

# --- task runtime (not in the mapping run) -------------------------------------------------------------
if [ "$POSE" != map ]; then
RX=(--socket "$IPC/inf.sock" --sensor-source "$SENSOR_SOURCE" --inference-device "$INF_DEVICE" --result-wakeup event
    --geometry-source kv_topic --geometry-fallback none --events "$O/runtime.jsonl")
[ "$DETECTOR" = stub ] || RX+=(--detector "$DETECTOR")
[ "${GEOMETRY_V2:-0}" = 1 ] && RX+=(--geometry-v2)   # 10-27: same switch as the producer's (runtime warns on a mismatch)
[ "${EXISTENCE_V3:-1}" = 0 ] && RX+=(--existence-v3 off) && mark "existence_v3 OFF (runtime)"
[ "${SEE_THROUGH_FILTER:-1}" = 0 ] && RX+=(--see-through-filter off) && mark "see-through filter OFF (runtime)"
[ "${ONE_PER_CLASS:-1}" = 0 ] && RX+=(--one-per-class off) && mark "one per class OFF (runtime)"
[ "${FAST_REMOVAL:-1}" = 0 ] && RX+=(--fast-removal off) && mark "fast removal OFF (runtime)"
[ -n "${FAST_REMOVAL_VOTE:-}" ] && RX+=(--fast-removal-vote "$FAST_REMOVAL_VOTE") && mark "fast removal vote $FAST_REMOVAL_VOTE (runtime)"
# static (bench): the runner's reference path -- poses read from TF (map -> camera), which bench_pose.py makes identity
# odom (real-car drive): the same reference path, map -> odom identity and odom -> base_link from the wheel odometry
if [ "$POSE" = reference ] || [ "$POSE" = static ] || [ "$POSE" = odom ]; then RX+=(--pose-source reference)
# rtabmap loc (ODOM visual): the gate tracks base_link (rtabmap_car.sh: host.ros_localization_gate --tracked-frame base_link)
else RX+=(--pose-source estimated --localization-map-id "${MAP_ID:-<sha256>}" --localization-tracked-frame base_link); fi
[ "$RECORD" = 1 ] && RX+=(--frames-dir "$O/frames" --stall-diag "$O/stall_diag.jsonl")
# object memory across runs (10-18, car_up CAR_MEMORY_*): this run's map identity, the files at stop, the file to load
[ "${MEMORY_IDENTITY:-0}" = 1 ] && RX+=(--memory-map-identity "$O/map_identity.json" --pose-mode "${MEMORY_POSE_MODE:-$POSE}")
[ "${MEMORY_SAVE:-0}" = 1 ] && RX+=(--memory-out-dir "$O" --memory-snapshot-s "${MEMORY_SNAPSHOT_S:-5}")
[ "${MEMORY_IN:-0}" = 1 ] && RX+=(--memory-in "$O/memory_in.json")
# 10-25 mapping run: the memory follows the mapping stop's map alignment (host/map_align.py)
[ "$MAPPING" = 1 ] && RX+=(--memory-align)
[ "$MAPPING" = 1 ] && [ "$LIVE_ALIGN" = 1 ] && RX+=(--live-align)   # 10-27: several alignments per run, frames tagged
# 10-25 search goals: the mapping run's own trajectory (mapping_ctl.py, latched status); the board never falls back to
# the simulation's five fixed points (unrelated to the room) -- in-place quarter turns instead (host/ros_runtime.py)
[ "$MAPPING" = 1 ] && RX+=(--search-waypoints-from-mapping)
[ "$PLATFORM" = board ] && RX+=(--search-fallback spin)
# 10-26 probabilistic search (host/search_belief.py; car_up CAR_SEARCH_MODE prob | mvp, default prob): the belief over
# the mapped, seen area picks each search look; without the mapping run's trajectory nodes the runner keeps the MVP list.
# Detection-rate table / weights: tools/car_run/search_config.json (replace q_table with the measured one)
RX+=(--search-mode "${SEARCH_MODE:-prob}")
[ -f "$SRCD/tools/car_run/search_config.json" ] && RX+=(--search-config "$SRCD/tools/car_run/search_config.json")
# board (10-22 find-nav dig, car_up CAR_NAV_TIMEOUT_S / CAR_TASK_TIMEOUT_S): per-goal 60 s on both of the runner's
# clocks, task 180 s; the host keeps the runner's defaults 30 / 120. Before --ros-args (rclpy takes all after it).
# 10-26: frame-gap watchdog 5 s on the car (Nano at ~96 % CPU stalls the processed stream ~2 s; the host keeps 2.0);
# car_up CAR_STREAM_TIMEOUT_S (2..20, checked there) arrives as STREAM_TIMEOUT_S
{ [ "$PLATFORM" = board ] || [ "${SIM_NAV_PROFILE:-}" = car ]; } \
  && RX+=(--navigation-timeout-s "${NAV_TIMEOUT_S:-90}" --task-timeout-s "${TASK_TIMEOUT_S:-300}")   # 10-26: + host CAR_SIM_NAV_PROFILE=car; 10-26 owner：重成功率，放宽自 60 / 180
[ "$PLATFORM" = board ] && RX+=(--stream-timeout-s "${STREAM_TIMEOUT_S:-5.0}")
# 10-26 owner：重成功率，失败上限放宽自 3（car_up CAR_MAX_NAV_FAILURES）；位姿可信门车上默认关（CAR_POSE_TRUST，陀螺比例标定好再开）
[ "$PLATFORM" = board ] && RX+=(--max-nav-failures "${MAX_NAV_FAILURES:-5}" --pose-trust "${POSE_TRUST:-off}")
# 10-26 owner：重成功率，运行器几何年龄门放宽自 200 ms（过期几何只少负证据/遮挡判断，不停车）
[ "$PLATFORM" = board ] && RX+=(--geometry-age-limit-ms 500)
# 10-26 owner：重成功率，放宽自「一帧 TF 过期/传感器无效/位姿无效/输入未就绪即判任务失败」：丢这一帧或走导航重试
[ "$PLATFORM" = board ] && RX+=(--input-hiccup drop)
# live (10-15): nobody publishes /fpga/rgb|depth|camera_info on the real camera -- the runner reads the driver's topics
# through remaps, the DEPTH CameraInfo (colour intrinsics + colour frame + zero distortion once hardware-aligned; the
# colour CameraInfo carries the lens distortion, which the runner refuses), --camera-frame LIVE_CAMERA_FRAME. As
# tools/integ_car_chain/nano_chain_inner_v2.sh (10-15). --ros-args LAST: everything after it goes to rclpy.
if [ "$CAMERA" = live ]; then
  # 10-29: colour/depth pairing and the producer-frame match scale with the camera period (30: nothing added)
  [ "$CAMERA_FPS" = 30 ] || RX+=(--camera-fps "$CAMERA_FPS")
  RX+=(--camera-qos "$RUNNER_CAMERA_QOS" --camera-frame "$LIVE_CAMERA_FRAME" --ros-args -r "/fpga/rgb:=$LIVE_COLOR_TOPIC"
       -r "/fpga/depth:=$LIVE_DEPTH_TOPIC" -r "/fpga/camera_info:=$LIVE_INFO_TOPIC")
fi
[ -n "${YOLO_PROFILE:-}" ] && RX+=(--yolo-profile "$YOLO_PROFILE")
launch runtime runtime.log "${TR[@]}" python3 -m host.ros_runtime "${RX[@]}"
wait_for runtime 60 has_line runtime.jsonl '"event": "startup"' || fail "runtime: no startup event in 60 s"
wait_for runtime "$RUNTIME_READY_S" has_line runtime.jsonl '"event": "frame_processed"' \
  || fail "runtime: no processed frame in $RUNTIME_READY_S s"
ready runtime
# runtime_info (latched start-up facts: camera_qos in effect, pose source ...): test runs, and always on the real camera
if { [ "$RECORD" = 1 ] && [ "$DETECTOR" != stub ]; } || [ "$CAMERA" = live ]; then
  launch runtime_info runtime_info.log "${T[@]}" python3 tools/sim_loop/latched_once.py --topic /fpga/runtime_info \
    --out "$O/runtime_info.json"
fi
fi   # POSE != map

if [ "$DRY_RUN" = 1 ]; then echo "# READY; then supervise (fail-stop=$CAR_FAILSTOP) until INT/TERM"; exit 0; fi
date -Is > "$O/READY"
mark "READY"
# supervision: no time limit. A child that exits (car_common.sh car_exit_action, CAR_FAILSTOP): hub / bridge / preview / producer
# are restarted after a backoff (10-26 owner：重成功率，放宽自 fail-stop; RESTART lines in marks.txt), any other child
# brings the stack down (fail-stop); CAR_FAILSTOP=1 stops on any child, CAR_FAILSTOP=log only logs.
# colour (car screen colour, display only) is only marked when it exits, in every CAR_FAILSTOP mode (car_exit_action).
ONESHOT=" runtime_info "
declare -A RESTARTS=() RESTART_AT=()
while :; do car_supervise_once; sleep 1; done
