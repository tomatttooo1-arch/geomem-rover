#!/bin/bash
# Line D1 (2026-10-14): one simulated find-the-object run on the server, end to end.
#
#   bash tools/sim_loop/run_sim_loop.sh MODE RUN_DIR
#     MODE   nav    : Nav2 point-to-point suite only (robot_sim/tests/nav_point_to_point.py)
#            bottle : the whole "找瓶子" scenario (robot_sim/tests/bottle_scenario.py, query via the hub)
#            estop  : the same scenario, with a PC e-stop (fpgapc estop) during the return navigation
#            cut    : (line D2) the same scenario, the geometry producer stopped (SIGTERM by pid) during the
#                     return navigation: frames stop -> permit TIMEOUT -> MCU latch -> the car stops
#     RUN_DIR new directory (absolute); every original record lands there.
#
# Line D2 (2026-10-14) switches; the defaults are the D2 configuration (D1 = DETECTOR=ssdlite INF_DEVICE=cuda
# CHASSIS=guard at commit 9d34518):
#   DETECTOR  yolo (default) | ssdlite (torch, INF_DEVICE cuda|cpu). The runtime gets --detector explicitly
#             (checked per frame). YOLO_BACKEND torch (default: YOLO_PT on torch CUDA fp32, TF32 off, ultralytics
#             8.3.40 from YOLO_UL_PATH, INF_DEVICE must be cuda, GPU INF_GPU -- hold its lock; equivalent to ort on
#             line A's 39 rec30 frames, runs/sim-loop-d2-20261014-results/gpu_equiv) | ort (YOLO_ONNX on
#             onnxruntime CPU, INF_DEVICE must be cpu, YOLO_THREADS; ~130-155 ms a frame here, too slow for the
#             runner's 200 ms geometry age gate, see D1 smokes 08-10).
#   CHASSIS   mcu (default): /fpga/cmd_vel_safe -> host/ros_chassis_driver.py -> pty -> tools/chassis_uart/
#             mcu_sim.py (heartbeat = the fake KV's permit bit, --permit-model realtime) -> its executed speed
#             /fpga/chassis/sim_twist -> the only ROS -> Gazebo wheel command (FPGA_SIM_CMD_TOPIC); the D1 e-stop
#             stand-in (sim_estop_gate + the guard's e-stop input) is not started. Needs bottle|estop|cut.
#             | guard (D1): velocity guard /fpga/cmd_vel_safe straight into Gazebo, sim_estop_gate.
#   MCU_HEARTBEAT kv (default) | always (positive control: the MCU never sees the permit drop).
#   ESTOP_RELEASE_AFTER_S (estop mode, CHASSIS=mcu: PC release this many s after the stop hold, default 1;
#             '' = no release, D1 behaviour)  ESTOP_HOLD_S (default 3 with mcu, 5 with guard)
#   CUT_AFTER_S (cut mode: s after the return navigation starts, default 3)
#   FPGA_SIM_WORLD_FILE (worlds/ file name; default room_carmount_d2.sdf = the D2 mesh wine bottle at the original
#             x 1.25 / original light; '' = the launch default room_carmount.sdf, the D1 world)
#   FPGA_FAULT_NAV_FAIL_FIRST (10-25 find-retry, FAULT INJECTION, default unset = off): passed into the stack only when
#             set; the runtime judges its first N successful task navigation results failed (--fault-nav-fail-first)
#
# Layout (all on this host, no board, no GPU):
#   host       host.inference_service (SSDLite, CPU, conda python) on a Unix socket
#   container  wd-sim-gz-<tag>    Gazebo Fortress + ros_gz bridge + velocity guard (network none, ipc shareable)
#   container  wd-sim-kv-<tag>    the fake KV260 (tools/kv_link/kv_server.py --core cpu --roi-core cpu
#              --cpu-threads KV_THREADS) in the Gazebo container's network namespace, on its own CPUs.
#              Started as root with CAP_NET_ADMIN and dropped at once to this user with only that
#              capability kept (setpriv, ambient), so its SO_RCVBUFFORCE 8 MB receive buffer takes effect
#              like on the board, where kv_server runs as root (docs/design/KV_LINK_20261011.md). As a plain
#              user it is capped at rmem_max 416 KB, less than one 640x400 request (9 x 60 KB chunks): the
#              tail chunks overflowed and 80 % of the frames timed out (smokes 06, 07).
#   container  wd-sim-stack-<tag> everything else (tools/sim_loop/stack_inner.sh); shares the Gazebo
#              container's network namespace (loopback only) AND its /dev/shm: Fast DDS's shared-memory
#              transport between two containers silently loses messages without a common /dev/shm.
# The source is a snapshot of this checkout (RUN_DIR/src, tracked + untracked-not-ignored files);
# RUN_DIR/head.txt and RUN_DIR/dirty.txt say which commit and whether it was clean. A formal run
# refuses a dirty tree unless ALLOW_DIRTY=1 (smoke runs only).
#
# env (optional): GZ_CPUS (default 36-41)  KV_CPUS (default 18,42,19,43 = both hyperthreads of two cores)
#                 KV_THREADS (default 4: the CPU core on 4 threads, rows interleaved, merged exactly; one thread
#                 takes ~100-130 ms per 640x400 frame here, too late for the runner's 200 ms age gate, smoke 07)
#                 STACK_CPUS (default 23,47,12-17)  RUNTIME_CPUS (task runner alone, default 22,46 = one core)
#                 INF_CPUS (default 20,21,44,45 = two whole cores). 48 logical CPUs = 24 cores x 2 threads,
#                 siblings N and N+24; CPUs 0-11/24-35 carry other users' jobs. Smoke 09 (--stall-diag): the
#                 runner's geometry age 264 ms median > 200 ms gate = 108 ms waiting for the previous inference
#                 + 140 ms inference round trip (SSDLite 110 ms with its CPUs' siblings busy, ~65 ms when idle).
#                 AUG (inference test-time augmentation, default from PREREG: none)
#                 INF_DEVICE (cuda|cpu, default cuda = the car's choice, an Orin GPU) INF_GPU (default 2: one host GPU,
#                 the only one the service sees). Smokes 08-10 with cpu: SSDLite 96-113 ms per frame in the run
#                 (69 ms on idle CPUs), round trip 127-140 ms, geometry age 250-281 ms median -> 90 % refused by
#                 the runner's 200 ms gate (the board runner refused 8 of 1826). cuda: ~25 ms on an idle 4090.
#                 ESTOP_AFTER_S (estop mode: seconds after the return navigation starts, default 3)
#                 SCENARIO_TIMEOUT_S (default 420, wall)  NAV_TIMEOUT_S (default 900, wall)
#                 INF_WARMUP (default 2: blank-frame detector calls before the service says ready; 0 = none,
#                 the 9df4147 behaviour) INF_WARMUP_SIZE (default 640x400 = the sim camera). D2 run_b2: every
#                 run's first request paid 2.4-10+ s of CUDA lazy init; >10 s = the runner's client timeout.
#                 TRIGGER_MIN_V / TRIGGER_MAX_W (estop|cut: fire only at odometry |v| >= V and |w| <= W, PREREG §10;
#                 unset = the frozen |v|+|w| >= 0.05)
#                 ALLOW_DIRTY=1  KEEP_SRC=1  STALL_DIAG=1 (runner --stall-diag: per-frame timing, measurement only)
# Never pkill -f: containers are removed by name, host processes stopped by pid.
set -u
MODE=${1:?MODE nav|bottle|estop|cut}
RUN=${2:?RUN_DIR}
case "$MODE" in nav|bottle|estop|cut) ;; *) echo "MODE must be nav, bottle, estop or cut" >&2; exit 2;; esac
case "$RUN" in /*) ;; *) echo "RUN_DIR must be absolute" >&2; exit 2;; esac
REPO=$(cd "$(dirname "$0")/../.." && pwd)
TAG=$(basename "$RUN")
[[ "$TAG" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "RUN_DIR basename: letters, digits, - and _ only" >&2; exit 2; }
: "${GZ_CPUS:=36-41}" "${KV_CPUS:=18,42,19,43}" "${KV_THREADS:=4}" "${STACK_CPUS:=23,47,12-17}"
: "${RUNTIME_CPUS:=22,46}" "${INF_CPUS:=20,21,44,45}" "${AUG:=none}" "${DETECTOR:=yolo}" "${CHASSIS:=mcu}"
: "${MCU_HEARTBEAT:=kv}" "${YOLO_ONNX:=${FPGA_SRC}/runs/yolo-runner-20261014-results/export/yolov8n-seg-640.onnx}"
: "${YOLO_THREADS:=4}" "${YOLO_BACKEND:=torch}" "${INF_WARMUP:=2}" "${INF_WARMUP_SIZE:=640x400}"
: "${TRIGGER_MIN_V:=}" "${TRIGGER_MAX_W:=}"
[[ "$INF_WARMUP" =~ ^[0-9]+$ ]] || { echo "INF_WARMUP: a count >= 0" >&2; exit 2; }
: "${YOLO_PT:=${FPGA_SHARED}/models/20260914-dev3-m3-single-image/yolov8n-seg.pt}"
: "${YOLO_UL_PATH:=${FPGA_HOME}/dev1/.t/ya/ul8340}"
case "$YOLO_BACKEND" in torch|ort) ;; *) echo "YOLO_BACKEND must be torch or ort" >&2; exit 2;; esac
YOLO_DEV=cpu; [ "$YOLO_BACKEND" = torch ] && YOLO_DEV=cuda
case "$DETECTOR" in yolo) : "${INF_DEVICE:=$YOLO_DEV}";; ssdlite) : "${INF_DEVICE:=cuda}";;
  *) echo "DETECTOR must be yolo or ssdlite" >&2; exit 2;; esac
: "${INF_GPU:=2}"
case "$INF_DEVICE" in cpu) INF_GPU="";; cuda) [[ "$INF_GPU" =~ ^[0-9]+$ ]] || { echo "INF_GPU: one GPU index" >&2; exit 2; };;
  *) echo "INF_DEVICE must be cpu or cuda" >&2; exit 2;; esac
[ "$DETECTOR" = yolo ] && [ "$INF_DEVICE" != "$YOLO_DEV" ] && { echo "host YOLO_BACKEND=$YOLO_BACKEND runs on INF_DEVICE=$YOLO_DEV" >&2; exit 2; }
YOLO_MODEL=""; [ "$DETECTOR" = yolo ] && { YOLO_MODEL=$YOLO_ONNX; [ "$YOLO_BACKEND" = torch ] && YOLO_MODEL=$YOLO_PT; }
case "$CHASSIS" in mcu) [ "$MODE" = nav ] && { echo "CHASSIS=mcu needs frames for the permit: bottle|estop|cut (nav: CHASSIS=guard)" >&2; exit 2; }
    : "${ESTOP_HOLD_S:=3}" "${ESTOP_RELEASE_AFTER_S=1}";;
  guard) [ "$MODE" = cut ] && { echo "cut mode needs CHASSIS=mcu" >&2; exit 2; }
    : "${ESTOP_HOLD_S:=5}" "${ESTOP_RELEASE_AFTER_S=}";;
  *) echo "CHASSIS must be mcu or guard" >&2; exit 2;; esac
case "$MCU_HEARTBEAT" in kv|always) ;; *) echo "MCU_HEARTBEAT must be kv or always" >&2; exit 2;; esac
PERMIT_MODEL=off; [ "$CHASSIS" = mcu ] && PERMIT_MODEL=realtime
: "${FPGA_SIM_WORLD_FILE=room_carmount_d2.sdf}"
[ -z "$FPGA_SIM_WORLD_FILE" ] || [ -f "$(dirname "$0")/../../robot_sim/fpga_sim/worlds/$FPGA_SIM_WORLD_FILE" ] \
  || { echo "FPGA_SIM_WORLD_FILE: no such file in robot_sim/fpga_sim/worlds/" >&2; exit 2; }
: "${ESTOP_AFTER_S:=3}" "${CUT_AFTER_S:=3}" "${SCENARIO_TIMEOUT_S:=420}" "${NAV_TIMEOUT_S:=900}"
PY_HOST=${FPGA_HOME}/miniconda3/bin/python3.13
MODELS=${FPGA_SHARED}/models/20260907-dev1-replay
IMAGE=fpga-dev1-sim:humble
GZ=wd2-sim-gz-$TAG
KV=wd2-sim-kv-$TAG
STACK=wd2-sim-stack-$TAG
IPC=${FPGA_HOME}/dev1/.t/wd2/$TAG         # short path: Unix socket length limit
mkdir "$RUN" || exit 2
git -C "$REPO" rev-parse HEAD > "$RUN/head.txt"
git -C "$REPO" status --porcelain > "$RUN/dirty.txt"
if [ -s "$RUN/dirty.txt" ] && [ "${ALLOW_DIRTY:-0}" != 1 ]; then
  echo "dirty tree (see $RUN/dirty.txt): commit first, or ALLOW_DIRTY=1 for a smoke" >&2; exit 2
fi
mkdir "$RUN/src"
(cd "$REPO" && git ls-files -z --cached --others --exclude-standard -- host robot_sim tools pc perception hw \
   | tar --null -T - -cf -) | tar -xf - -C "$RUN/src" || exit 3
{ date -Is; uptime; nproc; } > "$RUN/host_start.txt"
cat > "$RUN/run_config.json" <<EOF
{"mode": "$MODE", "tag": "$TAG", "head": "$(cat "$RUN/head.txt")", "dirty": $([ -s "$RUN/dirty.txt" ] && echo true || echo false),
 "image": "$IMAGE", "gz_container": "$GZ", "kv_container": "$KV", "stack_container": "$STACK",
 "gz_cpus": "$GZ_CPUS", "kv_cpus": "$KV_CPUS", "kv_threads": $KV_THREADS, "stack_cpus": "$STACK_CPUS",
 "inference_cpus": "$INF_CPUS", "runtime_cpus": "$RUNTIME_CPUS", "kv_rcvbuf": "SO_RCVBUFFORCE 8 MB (CAP_NET_ADMIN, like the board's root kv_server)",
 "inference": {"detector": "$DETECTOR", "device": "$INF_DEVICE", "gpu": "$INF_GPU", "augmentation": "$AUG", "models": "$MODELS",
               "warmup_frames": $INF_WARMUP, "warmup_size": "$INF_WARMUP_SIZE",
               "yolo_backend": "$([ "$DETECTOR" = yolo ] && echo "$YOLO_BACKEND")", "yolo_model": "$YOLO_MODEL",
               "yolo_model_sha256": "$([ -n "$YOLO_MODEL" ] && sha256sum "$YOLO_MODEL" | cut -c1-64)",
               "yolo_onnx": "$([ "$DETECTOR" = yolo ] && [ "$YOLO_BACKEND" = ort ] && echo "$YOLO_ONNX")",
               "yolo_threads": "$([ "$DETECTOR" = yolo ] && [ "$YOLO_BACKEND" = ort ] && echo "$YOLO_THREADS")"},
 "world_file": "${FPGA_SIM_WORLD_FILE:-room_carmount.sdf (launch default)}",
 "world_sha256": "$(sha256sum "$REPO/robot_sim/fpga_sim/worlds/${FPGA_SIM_WORLD_FILE:-room_carmount.sdf}" | cut -c1-64)",
 "chassis": "$CHASSIS", "mcu_heartbeat": "$MCU_HEARTBEAT", "permit_model": "$PERMIT_MODEL",
 "estop_hold_s": $ESTOP_HOLD_S, "estop_release_after_s": "${ESTOP_RELEASE_AFTER_S}", "cut_after_s": $CUT_AFTER_S,
 "controller": "rpp_heading_precision", "estop_after_s": $ESTOP_AFTER_S,
 "trigger_min_v": "$TRIGGER_MIN_V", "trigger_max_w": "$TRIGGER_MAX_W",
 "scenario_timeout_s": $SCENARIO_TIMEOUT_S, "nav_timeout_s": $NAV_TIMEOUT_S,
 "host_numbers_note": "宿主，不代表板上"}
EOF
mkdir -p "$IPC" && chmod 700 "$IPC" || exit 3
INF=""
cleanup() {
  trap - EXIT INT TERM
  if docker inspect "$KV" > /dev/null 2>&1; then   # INT: kv_server writes kv/summary.json and exits
    docker kill -s INT "$KV" > /dev/null 2>&1
    for i in $(seq 1 100); do [ "$(docker inspect -f '{{.State.Running}}' "$KV" 2>/dev/null)" = true ] || break; sleep 0.1; done
    echo "kv_container_exit=$(docker inspect -f '{{.State.ExitCode}}' "$KV" 2>/dev/null)" >> "$RUN/outer_rc.txt"
  fi
  for c in "$STACK" "$KV" "$GZ"; do
    if docker inspect "$c" > /dev/null 2>&1; then
      docker logs "$c" > "$RUN/docker-$c.log" 2>&1
      docker rm -f "$c" > /dev/null 2>&1
    fi
  done
  if [ -n "$INF" ] && kill -0 "$INF" 2>/dev/null; then
    kill -TERM "$INF"
    for i in $(seq 1 50); do kill -0 "$INF" 2>/dev/null || break; sleep 0.1; done
    kill -0 "$INF" 2>/dev/null && kill -KILL "$INF"
    wait "$INF" 2>/dev/null; echo "inference_rc=$?" >> "$RUN/outer_rc.txt"
  fi
  rm -f "$IPC/inf.sock"; rm -f "$IPC/ultralytics/settings.json"; rmdir "$IPC/ultralytics" "$IPC" 2>/dev/null
  { date -Is; uptime; } > "$RUN/host_end.txt"
  [ "${KEEP_SRC:-0}" = 1 ] || rm -rf "$RUN/src"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

if [ "$MODE" != nav ]; then
  # Inference service on the host (the image has no torch); INF_DEVICE, only GPU INF_GPU visible (none for cpu).
  INF_PATH="$RUN/src"
  if [ "$DETECTOR" = yolo ] && [ "$YOLO_BACKEND" = torch ]; then   # host YOLO: torch CUDA fp32 on GPU INF_GPU
    INF_ARGS=(--detector yolo --pt "$YOLO_PT")
    INF_PATH="$RUN/src:$YOLO_UL_PATH"
  elif [ "$DETECTOR" = yolo ]; then   # host YOLO: onnxruntime CPU on INF_CPUS, YOLO_THREADS intra-op threads
    INF_ARGS=(--detector yolo --onnx "$YOLO_ONNX" --threads "$YOLO_THREADS")
  else
    INF_ARGS=(--detector ssdlite --models "$MODELS" --augmentation "$AUG")
  fi
  CUDA_VISIBLE_DEVICES="$INF_GPU" TORCH_HOME="$MODELS" PYTHONPATH="$INF_PATH" PYTHONDONTWRITEBYTECODE=1 \
    YOLO_CONFIG_DIR="$IPC/ultralytics" YOLO_OFFLINE=1 \
    taskset -c "$INF_CPUS" "$PY_HOST" -m host.inference_service --socket "$IPC/inf.sock" \
    --device "$INF_DEVICE" "${INF_ARGS[@]}" --warmup-frames "$INF_WARMUP" --warmup-size "$INF_WARMUP_SIZE" \
    > "$RUN/inference.log" 2>&1 &
  INF=$!
  for i in $(seq 1 600); do
    grep -q '"ready": true' "$RUN/inference.log" 2>/dev/null && [ -S "$IPC/inf.sock" ] && break
    kill -0 "$INF" 2>/dev/null || { echo "inference service exited" >&2; exit 4; }
    sleep 0.1
  done
  grep -q '"ready": true' "$RUN/inference.log" || { echo "inference not ready in 60 s" >&2; exit 4; }
fi

SRC=$RUN/src
DDS=(-e ROS_DOMAIN_ID=93 -e ROS_LOCALHOST_ONLY=1 -e FASTRTPS_DEFAULT_PROFILES_FILE=/src/robot_sim/fpga_sim/config/fastdds_server.xml
     -e IGN_PARTITION="$TAG" -e GZ_PARTITION="$TAG" -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONUNBUFFERED=1 -e HOME=/tmp)
MNT=(-v "$SRC:/src:ro")
for name in launch config worlds maps; do
  MNT+=(-v "$SRC/robot_sim/fpga_sim/$name:/opt/fpga_ws/install/share/fpga_sim/$name:ro")
done
MNT+=(-v "$SRC/robot_sim/fpga_sim/scripts/velocity_guard.py:/opt/fpga_ws/install/lib/fpga_sim/velocity_guard.py:ro")
if [ "$CHASSIS" = mcu ]; then   # one wheel-command source: the MCU stand-in's executed speed; no e-stop stand-in
  GZ_ENV=(-e FPGA_SIM_CMD_TOPIC=/fpga/chassis/sim_twist -e FPGA_SIM_ESTOP_TOPIC=)
else
  GZ_ENV=(-e FPGA_SIM_ESTOP_TOPIC=/fpga/sim_estop)
fi
[ -n "${FPGA_SIM_WORLD_FILE:-}" ] && GZ_ENV+=(-e FPGA_SIM_WORLD="/opt/fpga_ws/install/share/fpga_sim/worlds/$FPGA_SIM_WORLD_FILE")
docker run -d --init --name "$GZ" --network none --ipc shareable --shm-size 1g --cpuset-cpus "$GZ_CPUS" \
  --user "$(id -u):$(id -g)" "${DDS[@]}" -e LIBGL_ALWAYS_SOFTWARE=1 -e FPGA_SIM_CLOUD=adapter \
  "${GZ_ENV[@]}" "${MNT[@]}" --entrypoint bash "$IMAGE" -c \
  'source /opt/ros/humble/setup.bash; source /opt/fpga_ws/install/setup.bash;
   exec xvfb-run -a -s "-screen 0 1280x720x24" ros2 launch fpga_sim simulation.launch.py' \
  > "$RUN/gz_container_id.txt" 2> "$RUN/gz_run.err" || exit 5
docker inspect "$GZ" > "$RUN/gz_inspect.json"
sleep 3
[ "$(docker inspect -f '{{.State.Running}}' "$GZ")" = true ] || { echo "Gazebo container exited at start" >&2; exit 5; }
# (the image's own entrypoint script is root-only: the setups are sourced here and in stack_inner.sh)
# fake KV260: root + CAP_NET_ADMIN only to drop at once to this user keeping that one capability
# (KILL: tini, still root, must forward docker kill -s INT to the dropped process; smoke 08 lost kv/summary.json)
docker run -d --init --name "$KV" --network "container:$GZ" --cpuset-cpus "$KV_CPUS" --user 0 \
  --cap-drop ALL --cap-add NET_ADMIN --cap-add SETUID --cap-add SETGID --cap-add SETPCAP --cap-add KILL \
  -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONUNBUFFERED=1 -v "$SRC:/src:ro" -v "$RUN:/o" \
  --entrypoint setpriv "$IMAGE" --reuid="$(id -u)" --regid="$(id -g)" --clear-groups \
  --inh-caps=-all,+net_admin --ambient-caps=+net_admin --bounding-set=-all,+net_admin \
  bash -c 'cd /src && exec python3 tools/kv_link/kv_server.py --core cpu --cpu-threads "$0" --roi-core cpu \
           --permit-model "$1" --port 47811 --out /o/kv > /o/kv.json 2> /o/kv.err' "$KV_THREADS" "$PERMIT_MODEL" \
  > "$RUN/kv_container_id.txt" 2> "$RUN/kv_run.err" || exit 5
docker run --rm --init --name "$STACK" --network "container:$GZ" --ipc "container:$GZ" --cpuset-cpus "$STACK_CPUS${RUNTIME_CPUS:+,$RUNTIME_CPUS}" \
  --user "$(id -u):$(id -g)" "${DDS[@]}" -e MODE="$MODE" -e STACK_CPUS="$STACK_CPUS" -e KV_EXTERNAL=1 -e RUNTIME_CPUS="$RUNTIME_CPUS" -e INF_DEVICE="$INF_DEVICE" ${STALL_DIAG:+-e STALL_DIAG=1} -e ESTOP_AFTER_S="$ESTOP_AFTER_S" \
  -e DETECTOR="$DETECTOR" -e CHASSIS="$CHASSIS" -e MCU_HEARTBEAT="$MCU_HEARTBEAT" -e ESTOP_HOLD_S="$ESTOP_HOLD_S" \
  -e ESTOP_RELEASE_AFTER_S="$ESTOP_RELEASE_AFTER_S" -e CUT_AFTER_S="$CUT_AFTER_S" \
  -e SCENARIO_TIMEOUT_S="$SCENARIO_TIMEOUT_S" -e NAV_TIMEOUT_S="$NAV_TIMEOUT_S" -e HEAD="$(cat "$RUN/head.txt")" \
  -e TRIGGER_MIN_V="$TRIGGER_MIN_V" -e TRIGGER_MAX_W="$TRIGGER_MAX_W" \
  ${FPGA_FAULT_NAV_FAIL_FIRST:+-e FPGA_FAULT_NAV_FAIL_FIRST="$FPGA_FAULT_NAV_FAIL_FIRST"} \
  "${MNT[@]}" -v "$RUN:/o" -v "$IPC:/ipc" --entrypoint bash "$IMAGE" /src/tools/sim_loop/stack_inner.sh \
  > "$RUN/stack.log" 2>&1
RC=$?
echo "stack_rc=$RC" >> "$RUN/outer_rc.txt"
exit $RC
