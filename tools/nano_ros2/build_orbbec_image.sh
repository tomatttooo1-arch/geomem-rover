#!/usr/bin/env bash
# Build the Orbbec driver image on the Nano in the background; writes <out>/build.status.
# Usage: nohup ./build_orbbec_image.sh ~/fpga/n2/src/OrbbecSDK_ROS2 ~/fpga/n2/build-orbbec-v1 &
set -u
ctx=${1:?checkout}; out=${2:?output dir}; tag=${3:-fpga-ros:humble-nav-orbbec-20260927}
here=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$out"; rm -f "$out/build.status"
cp "$here/orbbec.dockerignore" "$ctx/.dockerignore"
{ date -Is; git -C "$ctx" log -1 --format='%H %cd'; df -B1 / | tail -1; docker system df; } > "$out/before.txt" 2>&1
start=$(date +%s)
docker build --progress=plain -f "$here/Dockerfile.orbbec" -t "$tag" "$ctx" > "$out/build.log" 2>&1
rc=$?
{ date -Is; echo "rc=$rc seconds=$(( $(date +%s)-start ))"; df -B1 / | tail -1; docker system df
  docker image inspect "$tag" --format '{{.Id}} {{.Size}}'; } > "$out/after.txt" 2>&1
[ "$rc" = 0 ] && echo DONE > "$out/build.status" || echo FAILED > "$out/build.status"
