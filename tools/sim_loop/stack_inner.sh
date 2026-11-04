#!/bin/bash
# Line D1 (2026-10-14): the ROS stack of one simulated run, INSIDE the stack container
# (started by tools/sim_loop/run_sim_loop.sh; /src read-only source, /o the run directory,
# /ipc the inference socket). MODE nav | bottle | estop (see run_sim_loop.sh).
#
# Start order, each step with its readiness check (a failed check stops the run, no retry):
#   fake KV (kv_server, CPU cores, its own CPU) -> /fpga/depth + /clock from Gazebo -> sim_adapter
#   (16UC1 depth, optical cloud, SIM marker) -> sim_estop_gate (KV e-stop latch -> /fpga/sim_estop)
#   -> Nav2 controller configs -> navigation.launch.py (rpp_heading_precision, no autostart)
#   -> nav_startup_probe.py --activate --all-services-first -> record_motion.py
#   nav:          nav_point_to_point.py
#   bottle/estop: hub, PC listener (yields control), rolling bridge, producer, runtime, bottle_scenario.py
# Line D2 (2026-10-14), CHASSIS=mcu: instead of sim_estop_gate, the permit recorder (tools/sim_loop/
# permit_recorder.py, 100 Hz TELEM) and the chassis chain: mcu_sim.py --pty (heartbeat = KV permit bit, or
# always with MCU_HEARTBEAT=always) publishing its executed speed on /fpga/chassis/sim_twist (the simulator's
# only wheel command), host/ros_chassis_driver.py on the pty (subscribes /fpga/cmd_vel_safe); after the
# startup probe the single-source check (single_source.json). MODE cut: the producer's pid in producer.pid.
# Every process is stopped by its own pid (INT, then TERM, then KILL). Never pkill -f.
source /opt/ros/humble/setup.bash
source /opt/fpga_ws/install/setup.bash
set -u
cd /src
export PYTHONPATH=/src:${PYTHONPATH:-} LANG=C.UTF-8 LC_ALL=C.UTF-8   # keep the ROS paths
: "${MODE:?}" "${STACK_CPUS:?}" "${INF_DEVICE:=cpu}" "${ESTOP_AFTER_S:=3}" "${SCENARIO_TIMEOUT_S:=420}" "${NAV_TIMEOUT_S:=900}" "${HEAD:=unknown}"
: "${DETECTOR:=ssdlite}" "${CHASSIS:=guard}" "${MCU_HEARTBEAT:=kv}" "${ESTOP_HOLD_S:=5}" "${ESTOP_RELEASE_AFTER_S:=}" "${CUT_AFTER_S:=3}"
O=/o
KVPORT=47811
HUBPORT=47431
# CPU split inside the container (only without KV_EXTERNAL=1): the first KV_NCPU CPUs of the set for the
# fake KV, the rest for all else. run_sim_loop.sh sets KV_EXTERNAL=1: the fake KV has its own container and
# CPUs. Smokes 06/07 (KV in here, 1 then 2 CPUs): ~80 % LinkTimeout either way -- not the CPU split but the
# 416 KB receive buffer of a non-root process (rmem_max) against 9 x 60 KB chunks per 640x400 request, and
# ~130 ms per frame for the one-thread CPU core (sil-assess smoke 3 had 45 ms at 320x240, 3 chunks)
CPUS=$(python3 -c "
import sys
s=[]
for part in sys.argv[1].split(','):
    a,_,b=part.partition('-'); s+=range(int(a),int(b or a)+1)
n=int(sys.argv[2])
print(','.join(map(str,s[:n])), ','.join(map(str,s[n:])))" "$STACK_CPUS" "${KV_NCPU:-2}")
KV_CPU=${CPUS%% *}; REST=${CPUS#* }
if [ "${KV_EXTERNAL:-0}" = 1 ]; then   # the fake KV runs in its own container (run_sim_loop.sh: wd-sim-kv-<tag>,
  KV_CPU=external; REST=$STACK_CPUS    # CAP_NET_ADMIN for the board's 8 MB receive buffer, its own CPUs)
  REST=$(python3 -c "
import sys
s=[]
for part in sys.argv[1].split(','):
    a,_,b=part.partition('-'); s+=range(int(a),int(b or a)+1)
print(','.join(map(str,s)))" "$STACK_CPUS")
fi
# Command prefix (an array, not a function: "func ... &" would fork a subshell, and $! would be that
# subshell's pid, so INT/TERM would miss the real process). Background children of a non-interactive
# bash start with SIGINT ignored: env --default-signal=INT restores it so INT stops them cleanly.
T=(taskset -c "$REST" env --default-signal=INT)
mark() { echo "$(date +%s.%N) $*" >> "$O/marks.txt"; }
fail() { mark "FAIL $*"; echo "FAIL $*" >&2; echo "$*" > "$O/stack_fail.txt"; exit "${2:-1}"; }
NAMES=(); PIDS=()
track() { NAMES+=("$1"); PIDS+=("$2"); echo "$1 $2" >> "$O/pids.txt"; }
stop_pid() {   # name pid : INT, 10 s, TERM, 5 s, KILL; records rc
  local n=$1 p=$2 i
  kill -0 "$p" 2>/dev/null || { wait "$p" 2>/dev/null; echo "${n}_rc=$? (already exited)" >> "$O/rc.txt"; return; }
  kill -INT "$p" 2>/dev/null
  for i in $(seq 1 100); do kill -0 "$p" 2>/dev/null || break; sleep 0.1; done
  if kill -0 "$p" 2>/dev/null; then
    echo "$n alive 10 s after INT, TERM" >> "$O/rc.txt"; kill -TERM "$p" 2>/dev/null
    for i in $(seq 1 50); do kill -0 "$p" 2>/dev/null || break; sleep 0.1; done
    kill -0 "$p" 2>/dev/null && { echo "$n alive after TERM, KILL" >> "$O/rc.txt"; kill -KILL "$p"; }
  fi
  wait "$p" 2>/dev/null; echo "${n}_rc=$?" >> "$O/rc.txt"
}
cleanup() {
  trap - EXIT INT TERM
  mark "cleanup"
  local k
  for ((k=${#PIDS[@]}-1; k>=0; k--)); do stop_pid "${NAMES[$k]}" "${PIDS[$k]}"; done
  mark "stopped"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
mark "start mode=$MODE head=$HEAD kv_cpu=$KV_CPU rest=$REST"
{ python3 --version; printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE IGN_PARTITION;
  dpkg-query -W ros-humble-nav2-controller ros-humble-rmw-fastrtps-cpp 2>/dev/null; } > "$O/stack_env.txt" 2>&1

# 1. fake KV260 (CPU geometry + object cores), its own CPU (or already started outside: KV_EXTERNAL=1)
if [ "${KV_EXTERNAL:-0}" != 1 ]; then
  taskset -c "$KV_CPU" env --default-signal=INT python3 tools/kv_link/kv_server.py --core cpu --roi-core cpu --port "$KVPORT" --out "$O/kv" \
    > "$O/kv.json" 2> "$O/kv.err" &
  track kv $!
fi
for i in $(seq 1 300); do grep -q '"port"' "$O/kv.json" 2>/dev/null && break; sleep 0.2; done
grep -q '"port"' "$O/kv.json" || fail "kv_server not up" 3
mark "kv up $(head -c 300 "$O/kv.json")"

# 2. Gazebo topics
for i in $(seq 1 120); do
  "${T[@]}" ros2 topic list --no-daemon 2>/dev/null > "$O/topics_initial.txt"
  grep -qx /fpga/depth "$O/topics_initial.txt" && grep -qx /clock "$O/topics_initial.txt" && break
  sleep 2
done
grep -qx /fpga/depth "$O/topics_initial.txt" || fail "no /fpga/depth from Gazebo in 240 s" 3
mark "gazebo topics"

# 3. depth adapter (16UC1 + optical cloud + SIM marker)
"${T[@]}" python3 robot_sim/fpga_sim/scripts/sim_adapter.py --cloud --summary "$O/adapter_summary.json" > "$O/adapter.log" 2>&1 &
track adapter $!
if [ "$CHASSIS" = guard ]; then
# 4. simulated hardware stop: KV e-stop latch -> /fpga/sim_estop (the guard is fail-closed on it)
"${T[@]}" python3 robot_sim/fpga_sim/scripts/sim_estop_gate.py --kv-server "127.0.0.1:$KVPORT" --log "$O/estop_gate.jsonl" \
  > "$O/estop_gate.log" 2>&1 &
track estop_gate $!
else
# 4'. (D2) permit recorder; chassis chain: MCU stand-in on a pty pair (heartbeat from the KV permit bit),
# its executed speed is the simulator's wheel command; the Nano chassis driver on the other end of the pty
"${T[@]}" python3 tools/sim_loop/permit_recorder.py --kv "127.0.0.1:$KVPORT" --out "$O/permit_telem.jsonl" \
  > "$O/permit_recorder.log" 2>&1 &
track permit_recorder $!
HB=(--heartbeat-kv "127.0.0.1:$KVPORT"); [ "$MCU_HEARTBEAT" = always ] && HB=()
"${T[@]}" python3 tools/chassis_uart/mcu_sim.py --pty "${HB[@]}" --twist-topic /fpga/chassis/sim_twist \
  --log "$O/mcu.jsonl" > "$O/mcu.out" 2> "$O/mcu.err" &
track mcu $!
for i in $(seq 1 100); do [ -s "$O/mcu.out" ] && break; sleep 0.1; done
PTY=$(head -n 1 "$O/mcu.out")
[ -n "$PTY" ] && [ -e "$PTY" ] || fail "mcu_sim pty not up" 3
"${T[@]}" python3 -m host.ros_chassis_driver --port "$PTY" --record "$O/chassis_record.jsonl" \
  > "$O/chassis_driver.log" 2>&1 &
track chassis_driver $!
for i in $(seq 1 100); do grep -q '"dir": "rx"' "$O/chassis_record.jsonl" 2>/dev/null && break; sleep 0.1; done
grep -q '"dir": "rx"' "$O/chassis_record.jsonl" 2>/dev/null || fail "chassis driver got no MCU status frame" 3
mark "chassis chain up pty=$PTY heartbeat=$MCU_HEARTBEAT"
fi

# 5. Nav2 (RPP heading precision, configuration unchanged), verified start
"${T[@]}" python3 robot_sim/tests/nav_controller_variants.py --output "$O/controllers" --source-revision "$HEAD" \
  > "$O/controller-generation.log" 2>&1 || fail "controller generation" 3
FPGA_NAV2_OVERRIDES="$O/controllers/rpp_heading_precision.yaml" FPGA_NAV_AUTOSTART=false \
  "${T[@]}" ros2 launch fpga_sim navigation.launch.py > "$O/navigation.log" 2>&1 &
track navigation $!
mark "navigation launched"
"${T[@]}" python3 robot_sim/tests/nav_startup_probe.py --activate --all-services-first --out "$O/startup-probe.jsonl" \
  > "$O/startup-probe.log" 2>&1 || fail "nav_startup_probe failed (no retry)" 4
mark "startup probe passed"
REC=(); [ "$CHASSIS" = mcu ] && REC=(--odom-period-s 0 --clock-period-s 0.005 --twist-topic /fpga/chassis/sim_twist)
"${T[@]}" python3 robot_sim/tests/record_motion.py --out "$O/motion.jsonl" --seconds 100000 "${REC[@]}" > "$O/motion-recorder.log" 2>&1 &
track motion $!
if [ "$CHASSIS" = mcu ]; then   # the simulator has exactly one wheel-command source (judged in the verifier)
  # --spin-time 5: the default 0.5 s discovery of a fresh participant missed endpoints under load (D2 smoke_02:
  # publisher _NODE_NAME_UNKNOWN_, 0 subscribers on /fpga/cmd_vel_safe although the chassis driver was running)
  { echo "== ign topic -i -t /fpga/cmd_vel"; timeout 20 ign topic -i -t /fpga/cmd_vel;
    for t in /fpga/chassis/sim_twist /fpga/cmd_vel_safe; do echo "== ros2 topic info -v $t"; timeout 30 ros2 topic info -v "$t" --no-daemon --spin-time 5 2>/dev/null || timeout 30 ros2 topic info -v "$t"; done
  } > "$O/single_source.txt" 2>&1
  mark "single source check recorded"
fi

if [ "$MODE" = nav ]; then
  mark "nav suite start"
  timeout --signal=INT "$NAV_TIMEOUT_S" taskset -c "$REST" python3 robot_sim/tests/nav_point_to_point.py \
    --out "$O/nav-suite.jsonl" > "$O/nav-suite.log" 2>&1
  RC=$?
  echo "nav_suite_rc=$RC" >> "$O/rc.txt"; mark "nav suite end rc=$RC"
  exit $RC
fi

# 6. hub + display-only PC (yields control so that the scenario's fpgapc find becomes the controller)
python3 -c "import secrets;print(secrets.token_hex(24))" > "$O/token.txt"
"${T[@]}" python3 -m host.hub.hub_ros --data-source sim --token-file "$O/token.txt" --port "$HUBPORT" --bind 127.0.0.1 \
  --kv "127.0.0.1:$KVPORT" --stats-out "$O/hub_stats.json" > "$O/hub.out" 2> "$O/hub.err" &
track hub $!
for i in $(seq 1 100); do grep -q 'hub listening' "$O/hub.out" 2>/dev/null && break; sleep 0.2; done
grep -q 'hub listening' "$O/hub.out" || fail "hub not up" 3
"${T[@]}" python3 pc/fpgapc.py --host 127.0.0.1 --port "$HUBPORT" --token-file "$O/token.txt" \
  listen --yield-control --out "$O/pc.fpgarec" --stats "$O/pc_stats.json" > "$O/pc_listen.out" 2> "$O/pc_listen.err" &
track pc_listen $!
for i in $(seq 1 100); do grep -q '^YIELD' "$O/pc_listen.out" 2>/dev/null && break; sleep 0.2; done
grep -q '^YIELD' "$O/pc_listen.out" || fail "display PC did not yield control" 3
mark "hub + pc up"

# 7. rolling map bridge (radius 0.30 = Nav2 = runtime), geometry producer (the only KV frame client)
"${T[@]}" python3 host/ros_rolling_nav.py --ros-args -p ground_z_m:=0.0 -p robot_height_m:=0.25 -p robot_radius_m:=0.30 \
  > "$O/bridge.log" 2>&1 &
track bridge $!
"${T[@]}" python3 -m host.ros_geo_producer --data-source sim --backend remote_pl --kv-server "127.0.0.1:$KVPORT" \
  --depth-topic /fpga/depth16 --info-topic /fpga/camera_info --camera-frame camera_optical_frame \
  --ground-z-m 0.0 --rate-hz 10 --stats-out "$O/frames.jsonl" --summary-out "$O/producer_summary.json" \
  > "$O/producer.log" 2>&1 &
track producer $!
echo $! > "$O/producer.pid"

# 8. task runtime (SSDLite on INF_DEVICE over the host socket, geometry from the producer's topic, no CPU fallback)
# (its own CPUs when RUNTIME_CPUS is set: smoke 09 --stall-diag, 264 ms median geometry age = 108 ms waiting
#  for the previous inference + 140 ms inference round trip + 13 ms; the 200 ms gate needs both shorter)
TR=("${T[@]}"); [ -n "${RUNTIME_CPUS:-}" ] && TR=(taskset -c "$RUNTIME_CPUS" env --default-signal=INT)
"${TR[@]}" python3 -m host.ros_runtime --socket /ipc/inf.sock --pose-source reference --sensor-source simulation \
  --inference-device "$INF_DEVICE" --detector "$DETECTOR" --geometry-source kv_topic --geometry-fallback none --result-wakeup event \
  --events "$O/runtime.jsonl" --frames-dir "$O/frames" ${STALL_DIAG:+--stall-diag "$O/stall_diag.jsonl"} \
  ${FPGA_FAULT_NAV_FAIL_FIRST:+--fault-nav-fail-first "$FPGA_FAULT_NAV_FAIL_FIRST"} \
  > "$O/runtime.log" 2>&1 &
track runtime $!
mark "runtime started"
"${T[@]}" python3 tools/sim_loop/latched_once.py --topic /fpga/runtime_info --out "$O/runtime_info.json" \
  > "$O/runtime_info.log" 2>&1 &
track runtime_info $!

# 9. scenario (user level: no velocity, permit or truth input)
SC=(--out "$O/scenario.jsonl" --timeout "$SCENARIO_TIMEOUT_S" --expect-device "$INF_DEVICE" --query-via hub
    --hub-port "$HUBPORT" --token-file "$O/token.txt" --expect-detector "$DETECTOR")
[ "$MODE" = estop ] && SC+=(--estop-during-return-s "$ESTOP_AFTER_S" --estop-hold-s "$ESTOP_HOLD_S"
                           ${ESTOP_RELEASE_AFTER_S:+--estop-release-after-s "$ESTOP_RELEASE_AFTER_S"})
[ "$MODE" = cut ] && SC+=(--cut-during-return-s "$CUT_AFTER_S" --cut-pid-file "$O/producer.pid")
# PREREG §10 (straight-driving stop): both empty = the frozen §3/§4 trigger (|v|+|w| >= 0.05)
case "$MODE" in estop|cut) SC+=(${TRIGGER_MIN_V:+--trigger-min-v "$TRIGGER_MIN_V"} ${TRIGGER_MAX_W:+--trigger-max-w "$TRIGGER_MAX_W"});; esac
"${T[@]}" python3 robot_sim/tests/bottle_scenario.py "${SC[@]}" > "$O/scenario.log" 2>&1
RC=$?
echo "scenario_rc=$RC" >> "$O/rc.txt"; mark "scenario end rc=$RC"
sleep 2
exit $RC
