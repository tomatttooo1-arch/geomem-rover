#!/usr/bin/env bash
# Host wrapper on the Nano (hold the camera lock). Usage: run_orbbec_probe.sh LABEL [launch args...]
set -u
label=$1; shift
N2=$HOME/fpga/n2; RUN=$N2/runs/$label
mkdir -p "$RUN"; test ! -e "$RUN/status" || exit 2
BUSDEV=$(lsusb -d 2bc5: | head -1 | awk '{printf "/dev/bus/usb/%s/%s", $2, substr($4,1,3)}')
echo "$BUSDEV $(ls -l $BUSDEV)" > "$RUN/device.txt"
tegrastats --interval 500 --logfile "$RUN/tegrastats.log" & TS=$!
python3 "$N2/tools/proc_sampler.py" --out "$RUN/procs.jsonl" --stop-file "$RUN/.stop" \
  --match component_container --match topic_rate_probe.py & SP=$!
docker run --rm --name "fpga-n2-$label" --net=host --cpuset-cpus 0-3 --shm-size 1g --device "$BUSDEV" \
  -e ROS_DOMAIN_ID=217 -e ROS_LOCALHOST_ONLY=1 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/work/tools/fastdds_shm.xml \
  -e PYTHONDONTWRITEBYTECODE=1 -e HOME=/tmp -v "$N2:/work" -w /work --entrypoint bash fpga-ros:humble-nav-orbbec-scipy-20261017 \
  -c "/work/tools/orbbec_probe.sh /work/runs/$label/ros $*; chown -R $(id -u):$(id -g) /work/runs/$label" > "$RUN/docker.log" 2>&1
touch "$RUN/.stop"; wait "$SP"; kill "$TS"
grep -q DONE "$RUN/ros/status" 2>/dev/null && echo DONE > "$RUN/status" || echo FAILED > "$RUN/status"
