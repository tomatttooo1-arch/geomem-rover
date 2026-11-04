#!/usr/bin/env bash
# Tar only simulator sources: works with both legacy builder and BuildKit.
set -euo pipefail
case "${1:-}" in
  simulator) fpga_image=fpga-dev1-sim:humble ;;
  agx_ros) fpga_image=fpga-dev1-ros:humble ;;
  *) echo 'Usage: bash robot_sim/docker/build.sh simulator|agx_ros' >&2; exit 2 ;;
esac
fpga_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
tar -C "$fpga_repo" --exclude=__pycache__ --exclude='*.pyc' -cf - \
  robot_sim/fpga_sim robot_sim/docker/Dockerfile robot_sim/docker/entrypoint.sh |
  docker build --build-arg "ROS_BASE_IMAGE=${FPGA_ROS_BASE_IMAGE:-public.ecr.aws/docker/library/ros:humble-ros-base-jammy}" --target "$1" -f robot_sim/docker/Dockerfile -t "$fpga_image" -
