#!/usr/bin/env bash
# Run one command inside the frozen RTAB-Map ROS Humble image against THIS checkout.
#
# What it is for: this host has no ROS 2, so everything that imports rclpy/tf2_ros can only
# be exercised in the container. A rehearsal is expensive and takes the machine; this is the
# cheap version — one short command, no network, no writable mount, gone when it exits — so
# "does the container still import what the node needs, and does the node source still
# compile in there" stops being a question that only a full rehearsal can answer.
#
# It is NOT a rehearsal: nothing is recorded, nothing is judged, and a green run proves only
# that the command exited 0 inside the image.
#
# Conventions are the ones tools/rtabmap_reload_coordinate_run.py::common_create already
# uses (--init, --network none, --cpus/--memory, bind mounts declared with --mount), with
# the container bounded smaller because the work is trivial. The name carries the owner, so
# `docker ps` says whose it is and nothing here can touch another member's containers.
#
#   tools/rtabmap_container_smoke.sh                      # the default smoke, for real
#   tools/rtabmap_container_smoke.sh --dry-run            # print the docker argv, run nothing
#   tools/rtabmap_container_smoke.sh --rw DIR -- CMD ...  # one writable dir at /rosresults
set -euo pipefail

readonly IMAGE="fpga-dev1-rtabmap:humble-0.23.7-20260910"
readonly CHECKOUT_MOUNT="/checkout"
readonly RESULTS_MOUNT="/rosresults"

# Two questions, both cheap: does the container still have the runtime the nodes import, and
# does the source mounted from this checkout still compile against that Python.
# PYTHONPYCACHEPREFIX sends the .pyc files into the container's own writable layer; without
# it py_compile fails trying to write __pycache__ into the read-only checkout mount, which
# is a property of the mount worth keeping rather than a reason to loosen it.
readonly DEFAULT_SCRIPT='python3 -c "import rclpy, tf2_ros, scipy; print(\"ros ok\")"
PYTHONPATH=/checkout PYTHONPYCACHEPREFIX=/tmp/pycache python3 -m py_compile /checkout/host/ros_localization_gate.py /checkout/host/ros_odom_base_adapter.py /checkout/tools/tf_topic_recorder.py
echo "py_compile ok"'

usage() {
    sed -n '2,/^set -euo/p' "$0" | sed 's/^# \{0,1\}//; /^set -euo/d'
}

checkout=$(cd -- "$(dirname -- "$0")/.." && pwd)
dry_run=false
writable=""
while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run) dry_run=true; shift;;
        --rw)
            [ $# -ge 2 ] || { echo "--rw needs a directory" >&2; exit 2; }
            [ -z "$writable" ] || { echo "--rw may be given once" >&2; exit 2; }
            [ -d "$2" ] || { echo "--rw directory does not exist: $2" >&2; exit 2; }
            writable=$(cd -- "$2" && pwd)
            shift 2;;
        -h|--help) usage; exit 0;;
        --) shift; break;;
        -*) echo "unknown option: $1" >&2; exit 2;;
        *) break;;
    esac
done

# One command per container. With no command the default smoke above runs instead.
if [ $# -gt 0 ]; then
    command=("$@")
else
    command=(bash -c "set -e
$DEFAULT_SCRIPT")
fi

# fpga-member-purpose, plus the unix user and this shell's pid so two runs never collide and
user=$(id -un | tr '[:upper:]' '[:lower:]' | sed 's/[^a-z0-9]\{1,\}/-/g; s/^-\{1,\}//; s/-\{1,\}$//')
name="fpga-dev1-smoke-${user:-unknown}-$$"

# Read-only checkout, no network, no writable mount unless one was asked for by name.
argv=(docker run --rm --init --name "$name" --network none --cpus 2 --memory 4g
      --env "PYTHONPATH=$CHECKOUT_MOUNT"
      --mount "type=bind,src=$checkout,dst=$CHECKOUT_MOUNT,readonly")
if [ -n "$writable" ]; then
    argv+=(--mount "type=bind,src=$writable,dst=$RESULTS_MOUNT")
fi
argv+=("$IMAGE" -c 'source /opt/ros/humble/setup.bash; exec "$@"' bash "${command[@]}")

if [ "$dry_run" = true ]; then
    printf '%s\n' "${argv[@]}"
    exit 0
fi
"${argv[@]}"
