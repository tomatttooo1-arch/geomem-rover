#!/bin/bash
# Replay the 09-21 ROS 2 memory-transport acceptance (source d1fda80) on the Orin Nano.
# Runs on the Nano, in ~/fpga/p5, as the login user (docker group, no sudo).
# usage: nano_ros2_replay.sh <label> <netmode: host|none> <checkout tarball>
#   tarball = git archive d1fda80 <probe closure> + tools/nano_ros2_probe_replay.py
#   (see docs/design/NANO_ROS2_CONTAINER_20260927.md, "复现命令").
# Writes results-<label>/ and results-<label>.status (RUNNING -> DONE|FAILED).
set -u
cd ~/fpga/p5
LABEL=$1; NET=$2; TGZ=$3
IMG=fpga-ros:humble-ros-base-20260927
CK=$PWD/checkout-${TGZ%.tgz}
[ -d "$CK" ] || { mkdir "$CK" && tar -xzf "$TGZ" -C "$CK"; }
R=$PWD/results-$LABEL
mkdir "$R" "$R/probe" "$R/build" || { echo "results exist"; exit 2; }
echo RUNNING > "$R.status"
docker image inspect $IMG > "$R/image-inspect.json"
NAME=fpga-p5-ros2-$LABEL
COMMON=(--init --net="$NET" --cpus 2 --memory 4g
  -v "$CK:/checkout:ro"
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e PYTHONPATH=/checkout -e PYTHONDONTWRITEBYTECODE=1
  -e PYTHONPYCACHEPREFIX=/tmp/pycache -e ROS_DOMAIN_ID=214 -e ROS_LOCALHOST_ONLY=1
  -e LANG=C.UTF-8 -e LC_ALL=C.UTF-8 -w /checkout --entrypoint bash)
# 1) the 9 wrapper/instrument unit tests (same test file as 09-21)
docker run --rm --name "$NAME-tests" "${COMMON[@]}" $IMG -c 'source /opt/ros/humble/setup.bash; python3 -B -m unittest -v tests.test_m7_memory_ros2_probe' > "$R/unittest.log" 2>&1
echo "unittest_exit=$?" >> "$R/exit.txt"
# 2) formal replay; container kept until inspected, then removed
docker run --name "$NAME" "${COMMON[@]}" -v "$R/probe:/rosresults" -v "$R/build:/rosbuild" $IMG -c 'source /opt/ros/humble/setup.bash; exec python3 -B /checkout/tools/nano_ros2_probe_replay.py --source-commit d1fda805f831b7167c90afaf68ccd27ce6381779 --source-tag team-continuous-ros2-transport-20260921-prerun-v2' > "$R/container.stdout.log" 2> "$R/container.stderr.log"
RC=$?
echo "probe_exit=$RC" >> "$R/exit.txt"
docker inspect "$NAME" > "$R/container-after.json"
docker rm "$NAME" > "$R/container-remove.txt" 2>&1
docker ps -a --filter name="$NAME" --format '{{.Names}}' > "$R/no-residual.txt"
# container ran as root: hand files back to the login user (no sudo)
docker run --rm -v "$R:/d" --entrypoint chown $IMG -R "$(id -u):$(id -g)" /d
if [ $RC -eq 0 ]; then echo DONE > "$R.status"; else echo FAILED > "$R.status"; fi
