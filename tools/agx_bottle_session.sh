#!/usr/bin/env bash
# Run only this session's AGX processes, preserving existing environments/runs.
set -euo pipefail
release=$(realpath -e "$1")
session=$2
port=$3
controller=${4:-dwb}
hold=${5:-0}
augmentation=${6:-none}
startup_mode=${7:-auto}
startup_only=${8:-0}
processing_load=${9:-live}
dds_transport=${10:-udp}
result_wakeup=${11:-poll}
view_refinement=${12:-none}
detector=${13:-ssdlite}
[[ "$detector" == ssdlite ]]
[[ "$view_refinement" == none ]]
[[ "$result_wakeup" == poll || "$result_wakeup" == event ]]
[[ "$dds_transport" == udp || "$dds_transport" == shm_udp ]]
[[ "$startup_mode" == auto || "$startup_mode" == barrier || "$startup_mode" == observe || "$startup_mode" == verified ]]
[[ "$startup_only" == 0 || "$startup_only" == 1 ]]
[[ "$startup_mode" != observe || "$startup_only" == 1 ]]
[[ "$processing_load" == live || "$processing_load" == idle ]]
[[ "$processing_load" != idle || "$startup_only" == 1 ]]
[[ "$startup_only" == 0 || "$hold" == 0 ]]
nav_autostart=true
[[ "$startup_mode" == auto ]] || nav_autostart=false
[[ "$controller" == dwb || "$controller" == rpp || "$controller" == rpp_damped || "$controller" == rpp_heading_precision ]]
workspace=${14:?explicit AGX workspace required}
rootfs=${15:?explicit ROS rootfs required}
inference_python=${16:?explicit inference Python required}
models=${17:?explicit model directory required}
export FPGA_SERVICE_REPLY_PROFILE=${18:-legacy}
[[ "$FPGA_SERVICE_REPLY_PROFILE" == legacy || "$FPGA_SERVICE_REPLY_PROFILE" == bounded ]]
nav_entry=${19:-bottle}
export ROS_DOMAIN_ID=${20:-42}
nav_map_yaml=${21:--}
nav_map_id=${22:--}
[[ "$nav_entry" == bottle || "$nav_entry" == nav_suite || "$nav_entry" == estimated_inspect ]]
[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]]
nav_input_profile=reference
if [[ "$nav_entry" == nav_suite ]]; then
    [[ "$startup_only" == 1 && "$startup_mode" == verified && "$processing_load" == idle && "$hold" == 0 ]]
    [[ "$nav_map_yaml" == - && "$nav_map_id" == - ]]
elif [[ "$nav_entry" == estimated_inspect ]]; then
    nav_input_profile=estimated_inspect
    [[ "$startup_only" == 1 && "$startup_mode" == observe && "$processing_load" == idle && "$hold" == 0 ]]
    [[ "$nav_map_yaml" == /workspace/* ]]
    [[ "$nav_map_id" =~ ^[0-9a-f]{64}$ ]]
else
    [[ "$ROS_DOMAIN_ID" == 42 && "$nav_map_yaml" == - && "$nav_map_id" == - ]]
fi
[[ "$session" =~ ^bottle-[A-Za-z0-9_-]+$ ]]
[[ "$workspace" == /* && "$workspace" != / && -d "$workspace" ]]
[[ -d "$rootfs/opt/ros/humble" ]]
[[ "$nav_entry" != bottle || ( -x "$inference_python" && -d "$models" ) ]]
[[ "$release" == "$workspace/releases/"* ]]
out=$workspace/runs/$session
mkdir "$out"
echo $$ > "$out/supervisor.pid"
export FPGA_AGX_WORKSPACE=$workspace
export FPGA_DDS_TRANSPORT="$dds_transport"
export FPGA_DDS_SHM_DIR="$workspace/ipc/$session-dds"
export FPGA_NAV_INPUT_PROFILE="$nav_input_profile"
pids=()
shm_ram_created=0
snapshot_host_counters() {
    # Raw host counters, not derived claims; no network or scheduler settings change.
    cat /proc/net/snmp > "$out/proc-net-snmp.$1.txt"
    cat /proc/stat > "$out/proc-stat.$1.txt"
    date -Iseconds > "$out/host-counters.$1.time.txt"
}
cleanup() {
    trap - EXIT INT TERM
    snapshot_host_counters after || true
    for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
    if [[ "$shm_ram_created" == 1 ]]; then
        python3 - "$FPGA_DDS_SHM_BACKING" "$workspace" "$session" "$out" <<'PYRAM'
import json, os, sys
from pathlib import Path
root, workspace, session, out = map(Path, sys.argv[1:])
expected = Path('/dev/shm') / ('fpga-'+workspace.name+'-'+str(session)+'-dds')
if root != expected or root.is_symlink() or root.stat().st_uid != os.getuid():
    raise RuntimeError('refuse unrelated RAM IPC cleanup')
entries = list(root.iterdir())
if any(p.is_symlink() or not p.is_file() or not p.name.startswith(('fastrtps_', 'sem.fastrtps_')) for p in entries):
    raise RuntimeError('unexpected RAM IPC content; retained for inspection')
report = {'path':str(root),'files':[{'name':p.name,'bytes':p.stat().st_size} for p in entries]}
for p in entries: p.unlink()
root.rmdir()
report['removed_after_process_exit'] = True
(out/'dds-ram-cleanup.json').write_text(json.dumps(report,indent=2)+'\n')
PYRAM
    fi
    date -Iseconds > "$out/stopped.txt"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
# Never reuse another run's DDS segments; retain this directory as run evidence.
mkdir -m 700 "$FPGA_DDS_SHM_DIR"
echo "$FPGA_DDS_TRANSPORT" > "$out/dds-transport.txt"
echo "$FPGA_DDS_SHM_DIR" > "$out/dds-shm-dir.txt"
if [[ "$dds_transport" == shm_udp ]]; then
    export FPGA_DDS_SHM_BACKING="/dev/shm/fpga-${workspace##*/}-$session-dds"
    mkdir -m 700 "$FPGA_DDS_SHM_BACKING"
    shm_ram_created=1
    echo "$FPGA_DDS_SHM_BACKING" > "$out/dds-shm-backing.txt"
    stat -f -c %T "$FPGA_DDS_SHM_BACKING" > "$out/dds-shm-filesystem.txt"
fi
cd "$release"
git rev-parse HEAD > "$out/source-revision.txt"
uname -a > "$out/platform.txt"
if [[ "$nav_entry" == bottle ]]; then
"$inference_python" -m host.inference_service \
    --socket "$workspace/ipc/$session.sock" --models "$models" \
    --device cuda --cuda-math fp32 --augmentation "$augmentation" > "$out/inference.log" 2>&1 &
pids+=("$!")
# Do not feed a sensor frame before the model has loaded and the socket listens.
inference_pid=${pids[0]}
ready_deadline=$((SECONDS + 45))
until grep -q '"ready": true' "$out/inference.log" && [[ -S "$workspace/ipc/$session.sock" ]]; do
    kill -0 "$inference_pid"
    if (( SECONDS >= ready_deadline )); then echo 'Inference readiness deadline exceeded' >&2; exit 1; fi
    sleep .1
done
fi
ros() { bash "$release/tools/agx_rootfs.sh" "$rootfs" "$release" "$@"; }
if [[ "$nav_entry" == estimated_inspect ]]; then
    # /workspace is created by agx_rootfs.sh; it is not a host path.
    ros test -f "$nav_map_yaml"
fi
ros python3 /fpga/robot_sim/tests/nav_controller_variants.py \
    --output "/workspace/runs/$session/controllers" --source-revision "$(git rev-parse HEAD)" \
    > "$out/controller-generation.log" 2>&1
ros bash -c 'mkdir -p "/workspace/build/$1"; cd "/workspace/build/$1";
    colcon build --merge-install --base-paths /fpga/robot_sim/fpga_sim --install-base "/workspace/install/$1"' \
    fpga-build "$session" > "$out/build.log" 2>&1
ros bash -c 'python3 --version; printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION FPGA_DDS_TRANSPORT FPGA_SERVICE_REPLY_PROFILE;
    dpkg-query -W "ros-humble-nav2*" "ros-humble-rmw*"' > "$out/ros-versions.txt" 2>&1
snapshot_host_counters before
ros env RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS python3 -m host.ros_tunnel client \
    --port "$port" --domain "$ROS_DOMAIN_ID" --profile "$nav_input_profile" > "$out/relay.log" 2>&1 &
pids+=("$!")
ros bash -c 'source "/workspace/install/$1/setup.bash";
    export FPGA_NAV2_OVERRIDES="/workspace/runs/$1/controllers/$2.yaml";
    export FPGA_NAV_AUTOSTART="$3";
    export FPGA_NAV_INPUT_PROFILE="$4";
    if [[ "$4" == estimated_inspect ]]; then export FPGA_NAV_MAP_YAML="$5"; fi;
    exec ros2 launch fpga_sim navigation.launch.py' \
    fpga-nav "$session" "$controller" "$nav_autostart" "$nav_input_profile" "$nav_map_yaml" > "$out/navigation.log" 2>&1 &
pids+=("$!")
start_runtime() {
    local name=$1 frames=$1-frames
    [[ "$name" != runtime ]] || frames=frames
    # The tracked PID is bwrap itself, so termination closes its PID namespace.
    bash "$release/tools/agx_rootfs.sh" "$rootfs" "$release" \
        python3 -m host.ros_runtime --socket "/workspace/ipc/$session.sock" \
        --pose-source reference --sensor-source simulation --inference-device cuda \
        --result-wakeup "$result_wakeup" \
        --view-refinement "$view_refinement" \
        --detector "$detector" \
        --events "/workspace/runs/$session/$name.jsonl" --frames-dir "/workspace/runs/$session/$frames" \
        > "$out/$name.log" 2>&1 &
    runtime_pid=$!
    pids+=("$runtime_pid")
}
stop_runtime() {
    kill -TERM "$runtime_pid" 2>/dev/null || true
    wait "$runtime_pid" 2>/dev/null || true
    local keep=() pid
    for pid in "${pids[@]}"; do [[ "$pid" == "$runtime_pid" ]] || keep+=("$pid"); done
    pids=("${keep[@]}")
}
# Keep the same model, relay, RGB-D/TF/clock traffic in both load conditions.
# Only the runtime that consumes and processes frames is absent in idle probes.
if [[ "$processing_load" == live ]]; then start_runtime runtime; fi
if [[ "$nav_entry" == estimated_inspect ]]; then
    # This probe only observes estimated inputs and lifecycle state.  It never
    # invokes lifecycle transitions, sends a goal, or publishes motion permit.
    set +e
    ros python3 /fpga/robot_sim/tests/nav_input_inspect.py \
        --map-id "$nav_map_id" --map-yaml "$nav_map_yaml" --duration 15 \
        --out "/workspace/runs/$session/nav-input-inspect.json" \
        > "$out/nav-input-inspect.log" 2>&1
    result=$?
    set -e
    echo "$result" > "$out/nav-input-inspect-exit.txt"
    [[ "$result" == 0 ]]
    echo 'ESTIMATED_INPUT_INSPECTION_COMPLETE'
    exit 0
fi
if [[ "$startup_only" == 1 || "$startup_mode" == barrier || "$startup_mode" == verified ]]; then
    probe_args=()
    [[ "$startup_mode" != barrier ]] || probe_args+=(--activate)
    [[ "$startup_mode" != verified ]] || probe_args+=(--activate --all-services-first)
    [[ "$startup_mode" != observe ]] || probe_args+=(--observe-only --allow-inactive --duration 15)
    ros python3 /fpga/robot_sim/tests/nav_startup_probe.py \
        --out "/workspace/runs/$session/startup-probe.jsonl" "${probe_args[@]}" \
        > "$out/startup-probe.log" 2>&1
    # set -e stops the session on a failed probe; there is no activation retry.
    if [[ "$nav_entry" == nav_suite ]]; then
        ros python3 /fpga/robot_sim/tests/nav_point_to_point.py \
            --out "/workspace/runs/$session/nav-suite.jsonl" > "$out/nav-suite.log" 2>&1
        echo 'NAV_SUITE_COMPLETE'
    fi
    if [[ "$startup_only" == 1 ]]; then echo 'STARTUP_PROBE_COMPLETE'; exit 0; fi
fi
ros python3 /fpga/robot_sim/tests/bottle_scenario.py \
    --out "/workspace/runs/$session/scenario.jsonl" > "$out/scenario.log" 2>&1
# Successful scenario evidence is closed before deliberate fault traffic starts.
if (( hold > 0 )); then
    stop_runtime
    start_runtime fault-runtime
    echo 'SCENARIO_COMPLETE_HOLD'
fi
hold_deadline=$((SECONDS + hold))
while (( SECONDS < hold_deadline )) && [[ ! -f "$out/finish-hold" ]]; do
    if [[ -f "$out/restart-fault-runtime" && ! -f "$out/fault-runtime-restarted" ]]; then
        stop_runtime
        start_runtime fault-runtime-recovered
        # Acknowledges process launch only; the probe also requires fresh processed input.
        echo "$runtime_pid" > "$out/fault-runtime-restarted"
    fi
    sleep .1
done
