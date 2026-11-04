#!/usr/bin/env bash
# Run inside the nav image: export optimized poses (TUM-style, ROS frame) and the 2D grid of a DB.
# Usage: export_db.sh DB OUTDIR NAME
source /opt/ros/humble/setup.bash
set -u
db=$1; out=$2; name=$3
mkdir -p "$out"
rtabmap-info "$db" > "$out/$name.info.txt" 2>&1
rtabmap-export --poses --poses_format 10 --output_dir "$out" --output "$name" "$db" > "$out/$name.export_poses.log" 2>&1
rtabmap-export --map --output_dir "$out" --output "$name" "$db" > "$out/$name.export_map.log" 2>&1
ls -la "$out"
