#!/bin/bash
# One arm of the camera-drop layer experiments on the Nano host (hold the orin-nano and camera locks).
# Repo copy (2026-10-14) of the board tool that ran the 10-13 arms a_neg1 / l_a1 / l_a2 (unchanged below this
# copied to $B/tools, the worktree to $B/repo), PREPD=$HOME/fpga/geoprod/rec30_prep, image
# fpga-ros:humble-nav-orbbec-scipy-20261017. Another user changes B and PREPD first.
#   bx.sh NAME [VAR=VALUE ...]      (values without spaces; use '+' for a space inside LAUNCH_EXTRA / REPLAY_EXTRA / RX)
# Host-side variables: STRESS=N (N busy loops of `stress` on the host from the start of the warm-up)
# Everything else is handed to bx_inner.sh (see its head): DUR WARM CAMERA RX RX_INFO PY_PROBE DRV_LOG
# LAUNCH_EXTRA REPLAY_HZ REPLAY_EXTRA RX_FREEZE CAM_DDS_PROFILE.
set -u
B=$HOME/fpga/dev1_camdrop
IMG=fpga-ros:humble-nav-orbbec-scipy-20261017
PREPD=$HOME/fpga/geoprod/rec30_prep
NAME=$1; shift
OUT=$B/runs/$NAME
[ -e "$OUT" ] && { echo "exists: $OUT"; exit 2; }
mkdir -p "$OUT/sdklog"
STRESS=0; DUR=60; WARM=15; CAMERA=live
ENVS=()
for kv in "$@"; do
  k=${kv%%=*}; v=${kv#*=}; v=${v//+/ }
  case "$k" in
    STRESS) STRESS=$v;;
    DUR) DUR=$v; ENVS+=(-e "DUR=$v");;
    WARM) WARM=$v; ENVS+=(-e "WARM=$v");;
    CAMERA) CAMERA=$v; ENVS+=(-e "CAMERA=$v");;
    RX|RX_INFO|PY_PROBE|DRV_LOG|LAUNCH_EXTRA|LAUNCH_ARGS|REPLAY_HZ|REPLAY_EXTRA|RX_FREEZE|CAM_DDS_PROFILE|START_TRIES|START_WAIT_S|START_GAP_S|START_DEPTH_MSGS|RMW_FASTRTPS_PUBLICATION_MODE) ENVS+=(-e "$k=$v");;
    *) echo "bad variable $k"; exit 2;;
  esac
done
ENVS+=(-e "WARM=$WARM" -e "DUR=$DUR" -e "CAMERA=$CAMERA")
DEV=()
if [ "$CAMERA" = live ]; then
  BUSDEV=$(lsusb -d 2bc5: | head -1 | awk '{printf "/dev/bus/usb/%s/%s", $2, substr($4,1,3)}')
  [ -c "$BUSDEV" ] || { echo "no Orbbec camera on USB ($BUSDEV)"; exit 3; }
  DEV=(--device "$BUSDEV")
fi
{ echo "name=$NAME args=$* start=$(date -Is)"; echo "nvpmodel=$(nvpmodel -q 2>&1 | head -1)"
  echo "governor=$(cat /sys/devices/system/cpu/cpufreq/policy*/scaling_governor | tr '\n' ' ')"
  echo "loadavg=$(cat /proc/loadavg)"; echo "usb=$(lsusb -d 2bc5: | head -1) ${BUSDEV:-}"
  echo "containers_before=$(docker ps --format '{{.Names}}' | tr '\n' ' ')"; } > "$OUT/host.txt"
TOTAL=$((WARM + DUR + 150))
python3 $B/tools/sysmon.py "$OUT/sysmon.jsonl" "$TOTAL" 1 > "$OUT/sysmon.log" 2>&1 &
MON=$!
docker run -d --rm --name "fpga-dev1-cd-$NAME" --network host --user "$(id -u):$(id -g)" -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 \
  -e ROS_DOMAIN_ID=48 -e ROS_LOCALHOST_ONLY=1 -e OPENBLAS_NUM_THREADS=1 -e ROS_LOG_DIR=/tmp/roslog "${DEV[@]}" "${ENVS[@]}" \
  -v "$OUT":/out -v "$OUT/sdklog":/tmp/.ros/Log -v $B/repo:/repo:ro -v $B/tools:/t:ro -v $PREPD:/prep:ro \
  --entrypoint bash $IMG -c "bash /t/bx_inner.sh > /out/inner.log 2>&1; [ -e /out/rc ] || echo 99 > /out/rc" > "$OUT/container_id.txt" 2>&1
STRESSPID=
for i in $(seq 1 $((TOTAL))); do
  if [ -z "$STRESSPID" ] && [ "$STRESS" != 0 ] && [ -e "$OUT/window.txt" ]; then
    stress --cpu "$STRESS" --timeout $((WARM + DUR + 3)) > "$OUT/stress.log" 2>&1 &
    STRESSPID=$!; echo "stress cpu=$STRESS pid=$STRESSPID at=$(date +%s.%N | cut -c1-14)" >> "$OUT/host.txt"
  fi
  [ -e "$OUT/rc" ] && break
  sleep 1
done
[ -n "$STRESSPID" ] && kill -TERM "$STRESSPID" 2>/dev/null
for i in $(seq 1 15); do docker ps --format '{{.Names}}' | grep -q "^fpga-dev1-cd-$NAME\$" || break; sleep 1; done
docker stop -t 3 "fpga-dev1-cd-$NAME" > /dev/null 2>&1
kill -TERM $MON 2>/dev/null
{ echo "end=$(date -Is) rc=$(cat "$OUT/rc" 2>/dev/null || echo none)"
  echo "governor_after=$(cat /sys/devices/system/cpu/cpufreq/policy*/scaling_governor | tr '\n' ' ')"
  echo "containers_after=$(docker ps --format '{{.Names}}' | tr '\n' ' ')"; } >> "$OUT/host.txt"
echo "BX_DONE $NAME rc=$(cat "$OUT/rc" 2>/dev/null || echo none)"
