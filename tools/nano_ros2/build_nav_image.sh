#!/usr/bin/env bash
# Build the nav image on the Nano in the background; writes <dir>/build.status = DONE|FAILED.
# Usage (on the Nano): nohup ./build_nav_image.sh ~/fpga/n2/build-v1 >/dev/null 2>&1 &
set -u
out=${1:?output dir}
tag=${2:-fpga-ros:humble-nav-20260927}
here=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$out"
rm -f "$out/build.status"
{
  date -Is; df -B1 / | tail -1; docker system df
} > "$out/before.txt" 2>&1
start=$(date +%s)
docker build --progress=plain -f "$here/Dockerfile.nav" -t "$tag" "$here" > "$out/build.log" 2>&1
rc=$?
end=$(date +%s)
{
  date -Is; echo "rc=$rc seconds=$((end-start))"; df -B1 / | tail -1; docker system df
  docker image inspect "$tag" --format '{{.Id}} {{.Size}}' 2>&1
} > "$out/after.txt" 2>&1
if [ "$rc" = 0 ]; then
  docker run --rm --entrypoint cat "$tag" /opt/fpga/packages.tsv > "$out/packages.tsv" 2>&1
  echo DONE > "$out/build.status"
else
  echo FAILED > "$out/build.status"
fi
