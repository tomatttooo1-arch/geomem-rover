#!/usr/bin/env bash
# Run inside fpga-ros:humble-nav-20260927: static TF + rgbd_odometry + rtabmap, then stream a
# TUM sequence with tum_rgbd_pub.py. Writes $out/{events.jsonl,*.log,status}.
# Env: SPEED (playback speed factor, default 1.0 = recorded 30 Hz pace); ODOM_EXTRA (extra odom yaml lines); SYNC (rgbd|separate); CAM (e.g. "--fx 520.9 --fy 521.0 --cx 325.1 --cy 249.7" for freiburg2)
# Usage: rtabmap_tum_run.sh map|loc SEQ_DIR OUT_DIR DB_PATH [START] [STOP]
#   map: new database, incremental mapping.  loc: existing database, localization only
#        (Mem/IncrementalMemory=false, Mem/InitWMWithAllNodes=true), run on a copy of the DB.
mode=$1; seq=$2; out=$3; db=$4; start=${5:-0}; stop=${6:-}
here=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/humble/setup.bash
set -u
mkdir -p "$out"
case "$mode" in
  map) inc=true; initwm=false; test ! -e "$db" || { echo "db exists" > "$out/status"; exit 2; } ;;
  loc) inc=false; initwm=true; test -e "$db" || { echo "db missing" > "$out/status"; exit 2; }
       # localization mode still writes to the DB on shutdown, so work on a copy
       sha256sum "$db" > "$out/db_source.sha256"; cp "$db" "$out/loc_copy.db"; db="$out/loc_copy.db" ;;
  *) exit 2 ;;
esac
cat > "$out/rtabmap.yaml" <<YAML
/**:
  ros__parameters:
    frame_id: camera_link
    map_frame_id: map
    subscribe_depth: true
    subscribe_rgb: true
    subscribe_odom_info: true
    approx_sync: false
    database_path: "$db"
    Rtabmap/DetectionRate: "2"
    RGBD/LinearUpdate: "0"
    RGBD/AngularUpdate: "0"
    Mem/STMSize: "30"
    Mem/IncrementalMemory: "$inc"
    Mem/InitWMWithAllNodes: "$initwm"
YAML
cat > "$out/odom.yaml" <<YAML
/**:
  ros__parameters:
    frame_id: camera_link
    odom_frame_id: odom
    approx_sync: false
    publish_tf: true
${ODOM_EXTRA:-}
YAML
# SYNC=rgbd: rgbd_sync bundles RGB+depth+camera_info into one RGBDImage topic that both
# rgbd_odometry and rtabmap subscribe to (layout of the upstream RTAB-Map ROS 2 examples).
# SYNC=separate (default): odometry and rtabmap each subscribe to the three raw topics (exact sync).
R="-r rgb/image:=/fpga/rgb -r depth/image:=/fpga/depth -r rgb/camera_info:=/fpga/camera_info -r odom:=/fpga/odom -r odom_info:=/fpga/odom_info -r rgbd_image:=/fpga/rgbd_image"
if [ "${SYNC:-separate}" = rgbd ]; then
  printf '    subscribe_rgbd: true\n' >> "$out/odom.yaml"
  sed -i 's/^    subscribe_depth: true/    subscribe_rgbd: true/; /^    subscribe_rgb: true/d' "$out/rtabmap.yaml"
  /opt/ros/humble/lib/rtabmap_sync/rgbd_sync --ros-args $R -p approx_sync:=false > "$out/sync.log" 2>&1 &
  syp=$!
fi
/opt/ros/humble/lib/tf2_ros/static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 \
  --frame-id camera_link --child-frame-id camera_optical_frame > "$out/tf.log" 2>&1 &
tfp=$!
/opt/ros/humble/lib/rtabmap_odom/rgbd_odometry --ros-args $R --params-file "$out/odom.yaml" > "$out/odom.log" 2>&1 &
odp=$!
/opt/ros/humble/lib/rtabmap_slam/rtabmap --ros-args $R -r info:=/fpga/info -r localization_pose:=/fpga/localization_pose \
  --params-file "$out/rtabmap.yaml" > "$out/rtabmap.log" 2>&1 &
rtp=$!
echo "tf=$tfp odom=$odp rtabmap=$rtp" > "$out/pids.txt"
python3 "$here/tum_rgbd_pub.py" "$seq" --out "$out/events.jsonl" --start "$start" ${stop:+--stop "$stop"} --speed "${SPEED:-1.0}" ${CAM:-} > "$out/pub.log" 2>&1
prc=$?
# SIGINT lets rtabmap close and save its database.
kill -INT $rtp $odp 2>/dev/null
for i in $(seq 1 120); do kill -0 $rtp 2>/dev/null || break; sleep 0.5; done
kill -INT $tfp ${syp:-} 2>/dev/null; wait $tfp ${syp:-} 2>/dev/null
wait $rtp; rrc=$?
wait $odp 2>/dev/null
echo "pub=$prc rtabmap=$rrc" > "$out/exit.txt"
if [ "$prc" = 0 ]; then echo DONE > "$out/status"; else echo FAILED > "$out/status"; fi
