#!/bin/bash
# Nano host side of one ROS-runtime arm (geometry switch board runs). Login user, docker group, no sudo;
# does not change power mode, network or other processes.
#   nano_ros_arm.sh <repo> <runs dir> <label> <mode: cpu|kv|kvshadow> <prep> <duration s> [<kv HOST:PORT> <build id>]
# Writes <runs>/<label>/ (ros_arm.sh output) and <runs>/<label>.meta/{env.txt,container.*}. Foreground.
set -u
REPO=$1; RUNS=$2; LABEL=$3; MODE=$4; PREP=$5; DUR=$6; KV=${7:-}; BID=${8:-}
IMG=${IMG:-fpga-ros:humble-nav-20260927}
CPUSET=${CPUSET:-2-5}
mkdir -p "$RUNS"; M="$RUNS/$LABEL.meta"; mkdir "$M" || { echo "meta exists"; exit 2; }
{ date -Is; nvpmodel -q 2>&1; uptime; cat /proc/loadavg; cat "$REPO/COMMIT" 2>/dev/null
  for p in 0 4; do d=/sys/devices/system/cpu/cpufreq/policy$p; echo "policy$p $(cat $d/scaling_governor) cur=$(cat $d/scaling_cur_freq) max=$(cat $d/scaling_max_freq)"; done
  echo "image $IMG cpuset $CPUSET result_wakeup ${RESULT_WAKEUP:-event}"; docker image inspect "$IMG" --format '{{.Id}}'; top -bn1 | head -15; } > "$M/env.txt" 2>&1
NAME=fpga-cargeo-$LABEL
docker run --init --name "$NAME" --net=host --cpuset-cpus "$CPUSET" --memory 4g --shm-size 512m \
  -v "$REPO:/checkout:ro" -v "$RUNS:/runs" -v "$(dirname "$PREP"):/prepdir:ro" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e PYTHONPATH=/checkout -e PYTHONDONTWRITEBYTECODE=1 \
  -e ROS_DOMAIN_ID=217 -e ROS_LOCALHOST_ONLY=1 -e OPENBLAS_NUM_THREADS=1 -e RESULT_WAKEUP="${RESULT_WAKEUP:-event}" -e WATCH_STOP="${WATCH_STOP:-0}" -e LANG=C.UTF-8 -e LC_ALL=C.UTF-8 \
  --entrypoint bash "$IMG" /checkout/tools/car_geometry_kv/ros_arm.sh "$MODE" "/runs/$LABEL" "/prepdir/$(basename "$PREP")" "$DUR" "$KV" "$BID" \
  > "$M/container.stdout" 2> "$M/container.stderr"
echo "container_rc=$?" >> "$M/env.txt"
docker rm "$NAME" > /dev/null 2>&1
docker run --rm -v "$RUNS:/d" --entrypoint chown "$IMG" -R "$(id -u):$(id -g)" "/d/$LABEL" "/d/$LABEL.meta"
python3 - "$RUNS/$LABEL" <<'EOF'
import json, sys
d = sys.argv[1]
rows = [json.loads(l) for l in open(d + "/events.jsonl")]
n = sum(r["event"] == "frame_processed" for r in rows)
print(json.dumps({"label": d, "frame_processed": n, "events": len(rows),
                  "rc": open(d + "/rc.txt").read().split()}))
EOF
