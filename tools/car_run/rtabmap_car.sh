#!/bin/bash
# Real car (2026-10-15): rtabmap mapping / localization on the real camera.  Usage:  rtabmap_car.sh map|loc
#
#   map  a new database DB_DIR/<YYYYmmdd_HHMMSS>.db, incremental mapping. End: Ctrl-C (SIGINT) -> the camera and the
#        odometry stop, rtabmap gets SIGINT and saves its database ("db saved <path> <bytes>"), then rtabmap-info,
#        rtabmap-export --map (pgm/yaml for Nav2's map_server) and --poses, and the readout of
#        docs/design/PREREG_HANDHELD_RTABMAP_20261015.md section 4 (OUT/readout.json, "export done").
#   loc  localization in an existing database DB (rtabmap works on a copy in OUT: localization still writes the db
#        on shutdown; DB itself is only hashed); rtabmap's own grid goes to /fpga/rtabmap/map so that map_server keeps
#        /map. Topics under /fpga/rtabmap (odom, odom_info, info, localization_pose). With ODOM=visual the rehearsed
#        gate layout (tools/rtabmap_nav2_rehearsal_run.py, WP4b): rgbd_odometry and rtabmap publish their TF on
#        /fpga/localization_raw_tf; host.ros_odom_base_adapter relays odom -> base_link and /odom (tracked frame =
#        base_link: identity); host.ros_localization_gate (--tracked-frame base_link, reliable RGB on COLOR_TOPIC,
#        holds map -> odom at 20 Hz on /tf) publishes /fpga/localization_status; the runner then takes
#        --localization-tracked-frame base_link. With ODOM=wheel there is no gate (it needs odom_info): rtabmap
#        publishes map -> odom on /tf itself -- the runner cannot use --pose-source estimated (owner's decision).
#        Gate options (loc + ODOM=visual only; map mode and ODOM=wheel start no gate, so none of these touch them):
#        GATE_COUNT_MODE (correction | output | frame; default correction, owner 10-15 evening: approve the map->odom
#        correction of 3 consecutive linked rtabmap outputs, then one pose per odometry pair -- no camera topic read;
#        output = count consecutive rtabmap outputs per camera frame (10-15 daytime); frame = the old
#        every-camera-frame count, which never validates on this camera -- gate_dig/DIG.md);
#        GATE_MAX_CORRECTION_AGE_S (default 2, a positive number; correction mode only): X, the oldest held correction
#        the gate still uses. X tiers (host/localization_gate.py COUNT_MODES): measure on the board first the rtabmap
#        localization output rate and the p90 stamp age at which the gate receives them; >= 3 Hz and p90 <= 1.0 s
#        -> 2; 1-3 Hz and p90 <= 1.5 s -> 3; < 1 Hz -> no navigation on estimated poses (reduce the load first).
#        GATE_ODOM_JUMP (1 | 0; default 1), GATE_MAX_ODOM_SPEED_MPS (V, default 0.5, a positive number) and
#        GATE_ODOM_JUMP_MARGIN_M (E, default 0.10, a number >= 0); correction mode only (10-17 第 10 棒): a healthy
#        odometry step longer than V x dt + E revokes (valid, odometry_jump) or drops the entry candidates; V is the
#        platform's top speed (per platform); GATE_ODOM_JUMP=0 is the 4edc10f gate exactly.
#        GATE_ACQ / COLOR_INFO_TOPIC / --rgb-reliable apply to output and frame only (correction reads no camera topic).
#        GATE_ACQ (rgb | camera_info; default rgb): the gate's acquisition stamp from COLOR_TOPIC or from the COLOUR
#        CameraInfo COLOR_INFO_TOPIC (/camera/color/camera_info; never INFO_TOPIC, the depth one). camera_info is a
#        small message the gate receives far sooner than the image, but it is only correct if the Orbbec driver
#        stamps the colour CameraInfo identically to the colour Image: NOT verified on the board yet -- record
#        100 pairs and require every stamp equal before switching a car run to camera_info. --rgb-reliable
#        applies to whichever topic is chosen (check the driver's camera_info QoS too).
#        BUNDLE (default empty): a map bundle directory; the gate reads only BUNDLE/bundle.json navigation_ready
#        (true only when written explicitly) and reports it in /fpga/localization_status. It changes no approval.
#        Empty = no --bundle = navigation_ready false, which host/nav_localization.py (the Nav2-side consumer)
#        refuses: set BUNDLE for a navigation run. Who writes navigation_ready true for a car map, and when, is
#        not decided here (tools/rtabmap_bundle_map.py build always writes false; joint acceptance flips it).
#
# Layers.  STANDALONE=1 (the bench / hand-held mapping trial, no car stack): run on the board HOST; this script
#   starts ONE container of BOARD_IMAGE (name RT_CONTAINER) with the camera's USB node and runs itself inside it
#   (RTABMAP_CAR_LAYER=inner): the Orbbec driver (car_inner's LIVE_LAUNCH_ARGS and depth-data start retry), the
#   mount TF base_link -> CAMERA_FRAME (from CAM_HEIGHT_M / CAM_PITCH_DEG through car_mount.py), rgbd_odometry,
#   rtabmap and the watcher, all in one container (no cross-container Fast DDS shared memory; handover fact 9).
#   The container runs in the foreground: Ctrl-C reaches it through docker's signal proxy (PID 1 tini -> this
#   script's trap); the host side then records lsusb again, stops tegrastats and removes the container.
#   STANDALONE=0: run where ROS and the camera topics already are (docker exec into the car stack's container,
#   a replay container); starts only the odometry, rtabmap and the watcher (+ the mount TF when CAM_MOUNT_TF given).
#
# Env (PREREG names): MODE (map|loc, must equal the argument when set) ODOM (visual | wheel) STANDALONE (0|1)
#   FORCE_3DOF (1: Reg/Force3DoF true, the pushed bench / the car; 0: hand-held) CAM_HEIGHT_M (0.36, the 10-14 bench;
#   the car's camera measured 0.39 m since 10-17: pass CAM_HEIGHT_M=0.39 for it) CAM_PITCH_DEG (7.0: the car's camera
#   re-tilted by the owner 10-17, measured 7.05 deg; was 30) ODOM_3DOF (0: rgbd_odometry stays 6-DoF; FORCE_3DOF constrains rtabmap's graph only -- host
#   replay 10-15, fr2_pioneer_360 at 1/4 pace with the 0.36 m / 30 deg mount: odometry 3-DoF lost 61-78 % of frames
#   whatever the mount pitch, 6-DoF lost 0.13 %; a 3-DoF odometry needs the mount TF exact) DB_DIR (/data/fpga/rtabmap) OUT (the run directory; default DB_DIR/run_<time>)
#   DB (loc: the database; map: always DB_DIR/<time>.db)
#   ODOM=visual: rgbd_odometry (frame base_link, publishes odom -> base_link), rtabmap subscribes its odom_info.
#   ODOM=wheel: no visual odometry; rtabmap takes odom -> base_link from TF (odom_frame_id odom; the wheel
#     odometry node publishes it). ODOM_TF_LIN_VAR / ODOM_TF_ANG_VAR (default 0.001 / 0.001, positive numbers): the
#     variance rtabmap gives those TF odometry links (odom_tf_linear/angular_variance). 10-17 第 11 棒: a host replay
#     library built on the dataset's groundtruth as this odometry uses 0.000001 (the groundtruth is the truth: with
#     0.001 the graph optimisation bent its nodes 3-5 cm towards the visual loop closures; the stored odometry poses
#     were exact). The localization gate needs odom_info, which only rgbd_odometry gives: wheel
#     cannot pass the gate (owner's decision); the gate is not changed here.
#   ODOM=visual refuses (exit 3) when CAR_WHEEL_ODOM is set and not 0, or /odom or /fpga/odom already has a publisher.
# Topics: COLOR_TOPIC (/camera/color/image_raw) DEPTH_TOPIC (/camera/depth/image_raw) INFO_TOPIC
#   (/camera/depth/camera_info: the depth CameraInfo, zero distortion; HW-aligned to colour) CAMERA_FRAME
#   (camera_color_optical_frame); outputs (map) /fpga/odom /fpga/odom_info /fpga/info /fpga/localization_pose /map,
#   (loc) the same under /fpga/rtabmap/ + the gate's /fpga/localization_status and /fpga/localization_pose.
# Tuning: APPROX_SYNC (true) SYNC_MAX_S (0.02) DETECTION_RATE (map 1, loc 0) ODOM_RESET (Odom/ResetCountdown, 0)
#   GRID_MAX_OBSTACLE_M (1.5) GRID_RANGE_MAX_M (4.0) GRID_MAX_GROUND_M (0.08) RTAB_EXTRA / ODOM_EXTRA (extra yaml
#   lines, 4-space indented "    Key: \"value\"") QOS (1: reliable; 2: best effort, the car stack's replay topics)
#   RTAB_PUBLISH_TF (true; false: rtabmap does not publish map -> odom, e.g. a replay that publishes its own TF)
#   MAP_NS (empty; "rtabmap": map mode under /fpga/rtabmap and its grid on /fpga/rtabmap/map as loc does -- the
#   in-stack mapping of car_up CAR_MAPPING=1 (10-18), where Nav2's map_server owns /map; empty = unchanged)
#   SAVE_TIMEOUT_S (120) DURATION_S (0 = until Ctrl-C or the camera ends)
#   ODOM_SETTLE_S (3: camera up -> odometry start) ODOM_CHECK_N (20) ODOM_CHECK_TAIL (10) ODOM_CHECK_S (30): the
#   odometry start check (below; exit 6 after two failed starts) LSUSB_DELAY_S (5: the after-run lsusb waits for the
#   driver's shutdown USB reset; dmesg disconnects cross-checked)
# Board/host: BOARD_IMAGE (fpga-ros:humble-nav-orbbec-scipy-20261017) RT_CONTAINER (fpga-rtabmap-<mode>) DOCKER_NET (host)
#   ROS_DOMAIN_ID (48) DDS_PROFILE (tools/nano_ros2/fastdds_shm.xml; empty = Fast DDS defaults) CAMDEV (from lsusb
#   -d 2bc5:; "none" = no device) CAM_MOUNT_TF ("x y z qx qy qz qw"; default from car_mount.py) CAMERA_CMD (instead
#   of the Orbbec launch; a test stand-in) LIVE_LAUNCH_ARGS LIVE_FRAME_DROP_LOG (true: + enable_frame_drop_log:=true) LIVE_START_TRIES (10) LIVE_START_WAIT_S (20)
#   LIVE_START_GAP_S (10) LIVE_DEPTH_MSGS (30) SKIP_STACK_CHECK (0: refuse while a wde-car-* container runs)
#   DOCKER_EXTRA (extra docker run arguments, e.g. a test's data mount) CAR_PY_HOST (python3, for car_mount.py)
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$(cd "$HERE/../.." && pwd)
source "$HERE/car_common.sh"

ARG=${1:-}
: "${MODE:=$ARG}"
[ -n "$ARG" ] && [ "$ARG" != "$MODE" ] && { echo "rtabmap_car: MODE=$MODE but argument '$ARG'" >&2; exit 2; }
case "$MODE" in map|loc) ;; *) echo "usage: rtabmap_car.sh map|loc   (MODE=$MODE)" >&2; exit 2;; esac
: "${ODOM:=visual}" "${STANDALONE:=0}" "${FORCE_3DOF:=1}" "${CAM_HEIGHT_M:=0.36}" "${CAM_PITCH_DEG:=7.0}"
: "${DB_DIR:=/data/fpga/rtabmap}" "${RTABMAP_CAR_LAYER:=}" "${RUN_TIME:=$(date +%Y%m%d_%H%M%S)}"
: "${OUT:=$DB_DIR/run_$RUN_TIME}" "${DB:=}"
: "${COLOR_TOPIC:=/camera/color/image_raw}" "${DEPTH_TOPIC:=/camera/depth/image_raw}"
: "${INFO_TOPIC:=/camera/depth/camera_info}" "${CAMERA_FRAME:=camera_color_optical_frame}"
# 10-29: the colour/depth approx sync covers half a camera period + 5 ms below 30 fps (car_inner.sh exports CAMERA_FPS,
# car_up CAR_CAMERA_FPS; = host/ros_runtime.camera_pairing; 15 fps -> 0.038 s: the 30 fps stamps sat ~12 ms apart, the
# 15 fps ones 26.6 / 32.1 ms in two starts -- random sensor phase, up to half a period 33.3 ms)
case "${CAMERA_FPS:-30}" in 15) : "${SYNC_MAX_S:=0.038}";; 10) : "${SYNC_MAX_S:=0.055}";; 5) : "${SYNC_MAX_S:=0.105}";; esac
: "${APPROX_SYNC:=true}" "${SYNC_MAX_S:=0.02}" "${ODOM_RESET:=0}" "${RTAB_EXTRA:=}" "${ODOM_EXTRA:=}"
: "${GRID_MAX_OBSTACLE_M:=1.5}" "${GRID_RANGE_MAX_M:=4.0}" "${GRID_MAX_GROUND_M:=0.08}"
: "${SAVE_TIMEOUT_S:=120}" "${DURATION_S:=0}" "${QOS:=1}" "${RTAB_PUBLISH_TF:=true}"
: "${GATE_COUNT_MODE:=correction}" "${GATE_ACQ:=rgb}" "${COLOR_INFO_TOPIC:=/camera/color/camera_info}" "${BUNDLE:=}"
: "${GATE_MAX_CORRECTION_AGE_S:=2}"
: "${GATE_ODOM_JUMP:=1}" "${GATE_MAX_ODOM_SPEED_MPS:=0.5}" "${GATE_ODOM_JUMP_MARGIN_M:=0.10}"
: "${ODOM_TF_LIN_VAR:=0.001}" "${ODOM_TF_ANG_VAR:=0.001}"
for _v in ODOM_TF_LIN_VAR ODOM_TF_ANG_VAR; do
  awk -v x="${!_v}" 'BEGIN { exit !(x ~ /^([0-9]+\.?[0-9]*|\.[0-9]+)([eE][-+]?[0-9]+)?$/ && x + 0 > 0) }' \
    || { echo "$_v must be a positive number (given '${!_v}')" >&2; exit 2; }
done
case "$GATE_COUNT_MODE" in correction|output|frame) ;; *) echo "GATE_COUNT_MODE must be correction, output or frame" >&2; exit 2;; esac
case "$GATE_MAX_CORRECTION_AGE_S" in ''|.|*[!0-9.]*|*.*.*) GATE_X_OK=0;; *[1-9]*) GATE_X_OK=1;; *) GATE_X_OK=0;; esac
[ "$GATE_X_OK" = 1 ] || { echo "GATE_MAX_CORRECTION_AGE_S must be a positive number of seconds (given '$GATE_MAX_CORRECTION_AGE_S')" >&2; exit 2; }
case "$GATE_ODOM_JUMP" in 0|1) ;; *) echo "GATE_ODOM_JUMP must be 0 or 1" >&2; exit 2;; esac
case "$GATE_MAX_ODOM_SPEED_MPS" in ''|.|*[!0-9.]*|*.*.*) GATE_V_OK=0;; *[1-9]*) GATE_V_OK=1;; *) GATE_V_OK=0;; esac
[ "$GATE_V_OK" = 1 ] || { echo "GATE_MAX_ODOM_SPEED_MPS must be a positive number of m/s (given '$GATE_MAX_ODOM_SPEED_MPS')" >&2; exit 2; }
case "$GATE_ODOM_JUMP_MARGIN_M" in ''|.|*[!0-9.]*|*.*.*) GATE_E_OK=0;; *) GATE_E_OK=1;; esac
[ "$GATE_E_OK" = 1 ] || { echo "GATE_ODOM_JUMP_MARGIN_M must be a number of m >= 0 (given '$GATE_ODOM_JUMP_MARGIN_M')" >&2; exit 2; }
case "$GATE_ACQ" in rgb|camera_info) ;; *) echo "GATE_ACQ must be rgb or camera_info" >&2; exit 2;; esac
case "$QOS" in 0|1|2) ;; *) echo "QOS must be 0, 1 or 2 (rtabmap_ros qos: system default/reliable/best effort)" >&2; exit 2;; esac
case "$RTAB_PUBLISH_TF" in true|false) ;; *) echo "RTAB_PUBLISH_TF must be true or false" >&2; exit 2;; esac
: "${ODOM_SETTLE_S:=3}" "${ODOM_CHECK_N:=20}" "${ODOM_CHECK_TAIL:=10}" "${ODOM_CHECK_S:=30}" "${LSUSB_DELAY_S:=5}"
: "${BOARD_IMAGE:=fpga-ros:humble-nav-orbbec-scipy-20261017}" "${RT_CONTAINER:=fpga-rtabmap-$MODE}" "${DOCKER_NET:=host}"
: "${ROS_DOMAIN_ID:=48}" "${DDS_PROFILE=tools/nano_ros2/fastdds_shm.xml}" "${CAMDEV:=}" "${CAM_MOUNT_TF:=}"
: "${CAMERA_CMD:=}" "${SKIP_STACK_CHECK:=0}" "${DOCKER_EXTRA:=}" "${CAR_PY_HOST:=python3}"
: "${LIVE_LAUNCH_ARGS:=color_width:=640 color_height:=480 color_fps:=30 color_format:=RGB depth_width:=640 depth_height:=400 depth_fps:=30 depth_format:=Y16 enable_ir:=false enable_point_cloud:=false depth_registration:=true align_mode:=HW publish_tf:=false}"
# the driver's frame-drop log (10-17, car_common.sh): enable_frame_drop_log:=true unless LIVE_FRAME_DROP_LOG=false
car_frame_drop_log || exit 2
: "${LIVE_START_TRIES:=10}" "${LIVE_START_WAIT_S:=20}" "${LIVE_START_GAP_S:=10}" "${LIVE_DEPTH_MSGS:=30}"
case "$ODOM" in visual|wheel) ;; *) echo "ODOM must be visual or wheel" >&2; exit 2;; esac
case "$STANDALONE$FORCE_3DOF" in [01][01]) ;; *) echo "STANDALONE and FORCE_3DOF must be 0 or 1" >&2; exit 2;; esac
if [ "$MODE" = map ]; then DB="$DB_DIR/$RUN_TIME.db"; else
  [ -n "$DB" ] && [ -f "$DB" ] || { echo "loc: DB (an existing database) not given or missing: '$DB'" >&2; exit 2; }; fi
# BUNDLE is read by the gate only (loc + visual); elsewhere it is not used and not checked
if [ "$MODE" = loc ] && [ "$ODOM" = visual ] && [ -n "$BUNDLE" ]; then
  [ -f "$BUNDLE/bundle.json" ] || { echo "loc: BUNDLE given but $BUNDLE/bundle.json missing" >&2; exit 2; }; fi
F3=false; [ "$FORCE_3DOF" = 1 ] && F3=true
: "${ODOM_3DOF:=0}"; case "$ODOM_3DOF" in 0) OF3=false;; 1) OF3=true;; *) echo "ODOM_3DOF must be 0 or 1" >&2; exit 2;; esac
# every variable the inner layer reads (the outer layer passes them into the container)
PASS=(MODE ODOM STANDALONE FORCE_3DOF CAM_HEIGHT_M CAM_PITCH_DEG DB_DIR OUT DB RUN_TIME COLOR_TOPIC DEPTH_TOPIC INFO_TOPIC
      CAMERA_FRAME APPROX_SYNC SYNC_MAX_S ODOM_RESET RTAB_EXTRA ODOM_EXTRA GRID_MAX_OBSTACLE_M GRID_RANGE_MAX_M
      GRID_MAX_GROUND_M SAVE_TIMEOUT_S DURATION_S CAM_MOUNT_TF CAMERA_CMD LIVE_LAUNCH_ARGS LIVE_FRAME_DROP_LOG LIVE_START_TRIES
      LIVE_START_WAIT_S LIVE_START_GAP_S LIVE_DEPTH_MSGS DETECTION_RATE CAR_WHEEL_ODOM ODOM_3DOF
      ODOM_SETTLE_S ODOM_CHECK_N ODOM_CHECK_TAIL ODOM_CHECK_S QOS RTAB_PUBLISH_TF
      GATE_COUNT_MODE GATE_ACQ COLOR_INFO_TOPIC BUNDLE GATE_MAX_CORRECTION_AGE_S GATE_ODOM_JUMP GATE_MAX_ODOM_SPEED_MPS
      GATE_ODOM_JUMP_MARGIN_M ODOM_TF_LIN_VAR ODOM_TF_ANG_VAR MAP_NS)

# ============================================== outer: board host, STANDALONE=1 ======================================
if [ "$STANDALONE" = 1 ] && [ "$RTABMAP_CAR_LAYER" != inner ]; then
  if [ "$SKIP_STACK_CHECK" != 1 ]; then
    running=$(docker ps --format '{{.Names}}' | grep -E '^wde-car-' || true)
    [ -z "$running" ] || { echo "a car stack is running ($running): stop it (car_down.sh) first -- one camera, one rtabmap" >&2; exit 3; }
  fi
  docker inspect "$RT_CONTAINER" > /dev/null 2>&1 && { echo "container $RT_CONTAINER exists: docker rm it first" >&2; exit 3; }
  if [ -z "$CAMDEV" ]; then CAMDEV=$(car_orbbec_busdev) || { echo "no Orbbec camera (lsusb -d 2bc5:)" >&2; exit 3; }; fi
  if [ -z "$CAM_MOUNT_TF" ]; then
    M=$("$CAR_PY_HOST" "$HERE/car_mount.py" --height-m "$CAM_HEIGHT_M" --pitch-deg "$CAM_PITCH_DEG") \
      || { echo "car_mount.py refused height $CAM_HEIGHT_M pitch $CAM_PITCH_DEG" >&2; exit 2; }
    read -r _cell CAM_MOUNT_TF <<< "$M"
  fi
  mkdir -p "$OUT" "$DB_DIR" || exit 3
  [ -w "$DB_DIR" ] || { echo "DB_DIR $DB_DIR not writable" >&2; exit 3; }
  { echo "mode=$MODE odom=$ODOM force_3dof=$FORCE_3DOF height=$CAM_HEIGHT_M pitch=$CAM_PITCH_DEG mount_tf=$CAM_MOUNT_TF"
    echo "db=$DB camdev=$CAMDEV image=$BOARD_IMAGE container=$RT_CONTAINER src=$SRC"
    git -C "$SRC" rev-parse HEAD 2>/dev/null || head -1 "$SRC/COMMIT" 2>/dev/null; } > "$OUT/run.txt"
  ${CAR_LSUSB:-lsusb} -d 2bc5: > "$OUT/lsusb_before.txt" 2>&1
  UP0=$(cut -d' ' -f1 /proc/uptime)   # dmesg time base: USB disconnects from here on are counted after the run
  TS=""
  if command -v tegrastats > /dev/null; then
    setsid tegrastats --interval 1000 > "$OUT/tegrastats.log" 2>&1 < /dev/null & TS=$!
  fi
  ENVS=(-e RTABMAP_CAR_LAYER=inner -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e ROS_LOCALHOST_ONLY=1 -e HOME=/tmp
        -e ROS_LOG_DIR=/tmp/roslog -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONUNBUFFERED=1)
  [ -n "$DDS_PROFILE" ] && ENVS+=(-e FASTRTPS_DEFAULT_PROFILES_FILE="/src/$DDS_PROFILE")
  for v in "${PASS[@]}"; do [ -n "${!v+x}" ] && ENVS+=(-e "$v=${!v}"); done
  DEV=(); [ "$CAMDEV" != none ] && DEV=(--device "$CAMDEV")
  [ "$MODE" = loc ] && DEV+=(-v "$DB:$DB:ro")   # the pinned database, read only (rtabmap works on a copy in OUT)
  [ "$MODE" = loc ] && [ -n "$BUNDLE" ] && DEV+=(-v "$BUNDLE:$BUNDLE:ro")   # the gate reads bundle.json only
  # Ctrl-C: docker run (foreground, signal proxy) forwards SIGINT into the container; this shell only notes it
  trap 'echo "[rtabmap_car] SIGINT forwarded to the container; waiting for the db to be saved and exported" >&2' INT
  # shellcheck disable=SC2086
  docker run --init --name "$RT_CONTAINER" --network "$DOCKER_NET" --shm-size 1g --user "$(id -u):$(id -g)" \
    "${ENVS[@]}" "${DEV[@]}" -v "$SRC:/src:ro" -v "$OUT:$OUT" -v "$DB_DIR:$DB_DIR" $DOCKER_EXTRA \
    --entrypoint bash "$BOARD_IMAGE" /src/tools/car_run/rtabmap_car.sh "$MODE"
  rc=$?
  while docker inspect -f '{{.State.Running}}' "$RT_CONTAINER" 2>/dev/null | grep -q true; do
    echo "[rtabmap_car] container still running (rtabmap saving?); waiting -- do not kill -9" >&2; sleep 5
    docker wait "$RT_CONTAINER" > /dev/null 2>&1; rc=$(docker inspect -f '{{.State.ExitCode}}' "$RT_CONTAINER")
  done
  trap - INT
  # board static smoke 10-15: an lsusb right after the run read "Device 000" -- the driver's shutdown resets the
  # camera's USB 1-2 s later (dmesg: reset, same device number), not a drop-out. So: wait LSUSB_DELAY_S, then read,
  # and count dmesg "USB disconnect" lines since the run's start (the drop-out record; a reset is not one).
  UP1=$(cut -d' ' -f1 /proc/uptime)
  sleep "$LSUSB_DELAY_S"
  ${CAR_LSUSB:-lsusb} -d 2bc5: > "$OUT/lsusb_after.txt" 2>&1
  { ${CAR_DMESG:-dmesg} 2>/dev/null || sudo -n dmesg 2>/dev/null; } > "$OUT/dmesg_tail.txt"
  awk -v t0="$UP0" '{ t = $0; sub(/^\[ */, "", t); sub(/\].*/, "", t); if (t + 0 >= t0 + 0) print }' "$OUT/dmesg_tail.txt" \
    > "$OUT/dmesg_run.txt"
  NDISC=$(grep -ci "usb disconnect" "$OUT/dmesg_run.txt")
  if [ -s "$OUT/dmesg_tail.txt" ]; then DM="dmesg: $NDISC USB disconnect line(s) since the run's start (uptime $UP0 s; container ended at $UP1 s)"
  else DM="dmesg: not readable (no USB disconnect cross-check)"; fi
  if cmp -s "$OUT/lsusb_before.txt" "$OUT/lsusb_after.txt"; then echo "lsusb same (read ${LSUSB_DELAY_S} s after the run); $DM" >> "$OUT/run.txt"
  else echo "lsusb CHANGED (read ${LSUSB_DELAY_S} s after the run; camera re-enumerated? PREREG validity); $DM" | tee -a "$OUT/run.txt" >&2; fi
  [ "${NDISC:-0}" -gt 0 ] && grep -i "usb disconnect" "$OUT/dmesg_run.txt" | tee -a "$OUT/run.txt" >&2
  [ -n "$TS" ] && kill "$TS" 2>/dev/null
  docker rm "$RT_CONTAINER" > /dev/null && echo "container $RT_CONTAINER removed" >&2
  echo "[rtabmap_car] container exit $rc; run directory $OUT" >&2
  exit "$rc"
fi

# ============================================== inner: where ROS runs ===============================================
set +u   # the ROS setup scripts read unset variables
source /opt/ros/humble/setup.bash
[ "$STANDALONE" = 1 ] && [ -z "$CAMERA_CMD" ] && [ -f /opt/orbbec_ws/install/setup.bash ] && source /opt/orbbec_ws/install/setup.bash
set -u
cd "$SRC" || exit 3
mkdir -p "$OUT" || exit 3
[ "$MODE" = map ] && [ -e "$DB" ] && { echo "db $DB exists" >&2; exit 2; }
T=(env --default-signal=INT)
NAMES=() PIDS=()
log() { echo "[rtabmap_car $(date +%H:%M:%S)] $*" | tee -a "$OUT/marks.txt" >&2; }
start() {   # NAME LOGFILE CMD...  (background, SIGINT not ignored)
  local name=$1 lf=$2; shift 2
  "${T[@]}" "$@" > "$OUT/$lf" 2>&1 &
  NAMES+=("$name"); PIDS+=($!); echo "$name $!" >> "$OUT/pids.txt"
}
pid_of() { local i; for i in "${!NAMES[@]}"; do [ "${NAMES[$i]}" = "$1" ] && echo "${PIDS[$i]}"; done; }
alive() { local p; p=$(pid_of "$1"); [ -n "$p" ] && kill -0 "$p" 2>/dev/null; }
stop_one() {   # NAME TIMEOUT_S: INT (and its direct children), wait, then KILL
  local p kids i; p=$(pid_of "$1"); [ -n "$p" ] || return 0
  kids=$(pgrep -P "$p" 2>/dev/null)
  kill -INT $kids "$p" 2>/dev/null
  for ((i=0; i<$2*5; i++)); do kill -0 "$p" 2>/dev/null || break; sleep 0.2; done
  if kill -0 "$p" 2>/dev/null; then log "$1 did not stop in $2 s: KILL"; kill -KILL $kids "$p" 2>/dev/null; fi
  wait "$p" 2>/dev/null
}

if [ "$MODE" = map ]; then INC=true; INITWM=false; RO=false; : "${DETECTION_RATE:=1}"; RT_DB=$DB
else INC=false; INITWM=true; RO=true; : "${DETECTION_RATE:=0}"
  sha256sum "$DB" > "$OUT/db_source.sha256"; RT_DB="$OUT/loc_copy.db"; cp "$DB" "$RT_DB" || exit 3; fi
SUB_OI=false; [ "$ODOM" = visual ] && SUB_OI=true
SYNC_YAML="    approx_sync: $APPROX_SYNC
    approx_sync_max_interval: $SYNC_MAX_S
    qos: $QOS
    qos_camera_info: $QOS"
cat > "$OUT/rtabmap.yaml" <<YAML
/**:
  ros__parameters:
    frame_id: base_link
    map_frame_id: map
    publish_tf: $RTAB_PUBLISH_TF
    subscribe_depth: true
    subscribe_rgb: true
    subscribe_odom_info: $SUB_OI
$SYNC_YAML
    database_path: "$RT_DB"
$([ "$ODOM" = wheel ] && printf '    odom_frame_id: odom\n    odom_tf_linear_variance: %s\n    odom_tf_angular_variance: %s' "$ODOM_TF_LIN_VAR" "$ODOM_TF_ANG_VAR")
    Rtabmap/DetectionRate: "$DETECTION_RATE"
    Reg/Force3DoF: "$F3"
    Mem/IncrementalMemory: "$INC"
    Mem/InitWMWithAllNodes: "$INITWM"
    Mem/LocalizationReadOnly: "$RO"
    RGBD/StartAtOrigin: "$RO"
    Grid/Sensor: "1"
    Grid/3D: "false"
    Grid/RayTracing: "true"
    Grid/CellSize: "0.05"
    Grid/RangeMax: "$GRID_RANGE_MAX_M"
    Grid/MaxObstacleHeight: "$GRID_MAX_OBSTACLE_M"
    Grid/NormalsSegmentation: "false"
    Grid/MaxGroundHeight: "$GRID_MAX_GROUND_M"
$RTAB_EXTRA
YAML
cat > "$OUT/odom.yaml" <<YAML
/**:
  ros__parameters:
    frame_id: base_link
    odom_frame_id: odom
    publish_tf: true
$SYNC_YAML
    Reg/Force3DoF: "$OF3"
    Odom/ResetCountdown: "$ODOM_RESET"
$ODOM_EXTRA
YAML
# map: outputs under /fpga (as tools/nano_ros2/rtabmap_tum_run.sh); loc: under /fpga/rtabmap, the gate's --prefix
# (the gate itself publishes /fpga/localization_pose)
: "${MAP_NS:=}"; case "$MAP_NS" in ''|rtabmap) ;; *) echo "MAP_NS must be empty or rtabmap" >&2; exit 2;; esac
PX=/fpga; { [ "$MODE" = loc ] || [ "$MAP_NS" = rtabmap ]; } && PX=/fpga/rtabmap
GATE=0; [ "$MODE" = loc ] && [ "$ODOM" = visual ] && GATE=1
R=(-r "rgb/image:=$COLOR_TOPIC" -r "depth/image:=$DEPTH_TOPIC" -r "rgb/camera_info:=$INFO_TOPIC"
   -r "odom:=$PX/odom" -r "odom_info:=$PX/odom_info")
[ "$GATE" = 1 ] && R+=(-r /tf:=/fpga/localization_raw_tf)
RM=(-r "info:=$PX/info" -r "localization_pose:=$PX/localization_pose")
MAP_TOPIC=/map; { [ "$MODE" = loc ] || [ "$MAP_NS" = rtabmap ]; } && { RM+=(-r map:=/fpga/rtabmap/map); MAP_TOPIC=/fpga/rtabmap/map; }
log "mode=$MODE odom=$ODOM standalone=$STANDALONE force_3dof=$FORCE_3DOF odom_3dof=$ODOM_3DOF db=$RT_DB out=$OUT sync_max_s=$SYNC_MAX_S camera_fps=${CAMERA_FPS:-30}"
# ODOM=visual: rgbd_odometry is the one publisher of odom -> base_link; the wheel odometry (drive branch, on by
# default with live + uart; CAR_WHEEL_ODOM=0 turns it off) must not run too -- refuse when it is on or /odom has a
# publisher already (ODOM=wheel: it is the wheel odometry's edge, nothing to check here)
if [ "$ODOM" = visual ]; then
  [ "${CAR_WHEEL_ODOM:-${WHEEL_ODOM:-0}}" = 0 ] || { log "ODOM=visual needs CAR_WHEEL_ODOM=0 (car_inner: WHEEL_ODOM=0; one publisher of odom -> base_link)"; exit 3; }
  python3 tools/car_run/rtabmap_watch.py publishers --topic /odom --topic /fpga/odom > "$OUT/odom_guard.json" 2>&1 \
    || { log "ODOM=visual: /odom or /fpga/odom already has a publisher ($(cat "$OUT/odom_guard.json")) -- wheel odometry running?"; exit 3; }
fi

STOPPING=0; GOT_INT=0
trap 'GOT_INT=1' INT TERM
# 1. mount TF (when given); 2. watcher, rtabmap (+ gate) -- subscribed before the first frame; 3. the camera (a replay
# stand-in waits for rtabmap's subscriptions); 4. after ODOM_SETTLE_S the visual odometry, then its start check.
# Board 10-15 (smoke_1015_132253): odometry started before the driver initialised on its first frame (79 features,
# the camera still adjusting), then matched 0/20 inliers against it for the whole run (360 of 361 frames lost;
# Odom/ResetCountdown 0 never resets). Hence the odometry starts on a settled image, and its first ODOM_CHECK_N
# records are checked: ending in ODOM_CHECK_TAIL consecutive lost frames (or fewer than ODOM_CHECK_N records in
# ODOM_CHECK_S) = a failed start -> the odometry is restarted once (marks.txt), a second failure ends the run (exit 6).
if [ -n "$CAM_MOUNT_TF" ]; then
  read -r MX MY MZ MQX MQY MQZ MQW <<< "$CAM_MOUNT_TF"
  start mount_tf mount_tf.log /opt/ros/humble/lib/tf2_ros/static_transform_publisher --x "$MX" --y "$MY" --z "$MZ" \
    --qx "$MQX" --qy "$MQY" --qz "$MQZ" --qw "$MQW" --frame-id base_link --child-frame-id "$CAMERA_FRAME"
fi
start watch watch.out python3 tools/car_run/rtabmap_watch.py watch --map "$MAP_TOPIC" --odom-info "$PX/odom_info" \
  --info "$PX/info" --status /fpga/localization_status --log "$OUT/watch.jsonl" --summary "$OUT/watch_summary.json"
start rtabmap rtabmap.log /opt/ros/humble/lib/rtabmap_slam/rtabmap --ros-args "${R[@]}" "${RM[@]}" \
  --params-file "$OUT/rtabmap.yaml"
if [ "$GATE" = 1 ]; then
  start adapter adapter.log python3 -m host.ros_odom_base_adapter --prefix "$PX" --tracked-frame base_link \
    --base-frame base_link --odom-frame odom
  GA=(--count-mode "$GATE_COUNT_MODE" --acquisition-topic "$GATE_ACQ" --max-correction-age-s "$GATE_MAX_CORRECTION_AGE_S")
  GA+=(--max-odom-speed-mps "$GATE_MAX_ODOM_SPEED_MPS" --odom-jump-margin-m "$GATE_ODOM_JUMP_MARGIN_M")
  if [ "$GATE_ODOM_JUMP" = 1 ]; then GA+=(--odom-jump-check); else GA+=(--no-odom-jump-check); fi
  if [ -n "$BUNDLE" ]; then GA+=(--bundle "$BUNDLE")
  else log "gate: BUNDLE not set -> navigation_ready false (host/nav_localization.py refuses it; fine without Nav2)"; fi
  log "gate: count_mode=$GATE_COUNT_MODE max_correction_age_s=$GATE_MAX_CORRECTION_AGE_S acquisition=$GATE_ACQ bundle=${BUNDLE:-none}"
  log "gate: odom_jump_check=$GATE_ODOM_JUMP max_odom_speed_mps=$GATE_MAX_ODOM_SPEED_MPS odom_jump_margin_m=$GATE_ODOM_JUMP_MARGIN_M"
  start gate gate.log python3 -m host.ros_localization_gate --database "$DB" --map-id "$(cut -c1-64 "$OUT/db_source.sha256")" \
    --prefix "$PX" --tracked-frame base_link --rgb-reliable --no-publish-tracked-frame --map-odom-hold-hz 20 \
    --map-odom-future-s 0.1 --events "$OUT/gate-events.jsonl" "${GA[@]}" --ros-args -r /fpga/localization_tf:=/tf \
    -r "$PX/rgb:=$COLOR_TOPIC" -r "$PX/camera_info:=$COLOR_INFO_TOPIC"
fi
sleep 3
for n in watch rtabmap $([ "$GATE" = 1 ] && echo adapter gate); do alive "$n" || { log "$n exited at start"; GOT_INT=1; }; done
if [ "$STANDALONE" = 1 ] && [ "$GOT_INT" = 0 ]; then
  CAR_RETRY_PROBE=("${T[@]}" python3 tools/car_run/depth_probe.py --topic "$DEPTH_TOPIC" --count "$LIVE_DEPTH_MSGS"
                   --timeout-s "$LIVE_START_WAIT_S")
  if [ -n "$CAMERA_CMD" ]; then CAM=(bash -c "$CAMERA_CMD"); else CAM=(ros2 launch orbbec_camera gemini2.launch.py $LIVE_LAUNCH_ARGS); fi
  CAR_MARKS_FILE="$OUT/marks.txt" car_start_retry "$LIVE_START_TRIES" "$LIVE_START_WAIT_S" "$LIVE_START_GAP_S" \
    "$OUT/driver.log" "${T[@]}" "${CAM[@]}" || { log "camera: no depth images on $DEPTH_TOPIC in $LIVE_START_TRIES starts"; GOT_INT=1; }
  if [ -n "${CAR_RETRY_PID:-}" ]; then NAMES+=(camera); PIDS+=("$CAR_RETRY_PID"); echo "camera $CAR_RETRY_PID" >> "$OUT/pids.txt"; fi
fi
ODOM_RC=0
if [ "$ODOM" = visual ] && [ "$GOT_INT" = 0 ]; then
  log "camera up; odometry starts after ${ODOM_SETTLE_S} s (exposure settling)"
  sleep "$ODOM_SETTLE_S"
  for try in 1 2; do
    [ "$try" = 2 ] && { stop_one odom 10; NAMES=("${NAMES[@]/#odom/odom_failed_1}"); sleep "$ODOM_SETTLE_S"; }
    since=$(date +%s.%N)
    start odom "odom$([ "$try" = 2 ] && echo _2).log" /opt/ros/humble/lib/rtabmap_odom/rgbd_odometry --ros-args "${R[@]}" \
      --params-file "$OUT/odom.yaml"
    python3 tools/car_run/rtabmap_watch.py odomcheck --log "$OUT/watch.jsonl" --since "$since" --n "$ODOM_CHECK_N" \
      --tail "$ODOM_CHECK_TAIL" --timeout-s "$ODOM_CHECK_S" > "$OUT/odom_check_$try.json" 2>&1
    rc=$?
    log "odometry start check try=$try rc=$rc $(tail -c 300 "$OUT/odom_check_$try.json")"
    [ "$rc" = 0 ] && break
    [ "$GOT_INT" = 1 ] && break
    if [ "$try" = 2 ]; then log "ERROR: visual odometry failed to start twice -- ending the run"; ODOM_RC=6; GOT_INT=1; fi
  done
fi
if [ "$GOT_INT" = 0 ]; then
  log "running -- walk now; Ctrl-C to end (rtabmap then saves the db)"
  t0=$SECONDS
  while [ "$GOT_INT" = 0 ]; do
    for n in rtabmap $([ "$ODOM" = visual ] && echo odom) $([ "$STANDALONE" = 1 ] && echo camera) $([ "$GATE" = 1 ] && echo adapter gate); do
      alive "$n" || { log "$n exited (end of run)"; GOT_INT=1; }
    done
    [ "$DURATION_S" != 0 ] && [ $((SECONDS - t0)) -ge "$DURATION_S" ] && { log "DURATION_S $DURATION_S reached"; GOT_INT=1; }
    sleep 1
  done
  log "end after $((SECONDS - t0)) s"
fi
trap '' INT TERM   # from here on: finish the shutdown whatever arrives

# ---- shutdown: source first, then the odometry, then rtabmap (it saves the db) --------------------------------------
P=$(pid_of rtabmap)
[ -n "$P" ] && grep -E 'VmHWM|VmRSS' "/proc/$P/status" > "$OUT/rtabmap_mem.txt" 2>/dev/null
alive camera && stop_one camera 10
alive odom && stop_one odom 10
alive gate && stop_one gate 10
alive adapter && stop_one adapter 10
RRC=none
if [ -n "$P" ]; then
  kill -INT "$P" 2>/dev/null
  for ((i=0; i<SAVE_TIMEOUT_S*5; i++)); do kill -0 "$P" 2>/dev/null || break; sleep 0.2; done
  if kill -0 "$P" 2>/dev/null; then log "rtabmap still saving after $SAVE_TIMEOUT_S s -- left running (no KILL: the db would be lost)"
  else wait "$P"; RRC=$?; fi
fi
stop_one watch 10
alive mount_tf && stop_one mount_tf 5
log "rtabmap exit $RRC"
[ "$MODE" = loc ] && { log "loc done (copy $RT_DB)"; exit "$ODOM_RC"; }
if [ "$RRC" = 0 ] && [ -s "$DB" ] && rtabmap-info "$DB" > "$OUT/rtabmap_info.txt" 2>&1; then
  log "db saved $DB $(stat -c %s "$DB") bytes"
else
  log "db NOT saved ($DB; rtabmap exit $RRC)"; exit 5
fi
mkdir -p "$OUT/export"
NAME=$(basename "$DB" .db)
# given explicitly only when this rtabmap-export lists the option (the board image's help is not checked yet)
# 10-25: the poses export takes the same --opt -- host/map_align.py derives the map -> odom correction from these
# poses, which must be the graph the exported grid was drawn from
EXPORT_OPT=(); rtabmap-export --help 2>&1 | grep -q -- '--opt' && EXPORT_OPT=(--opt 0)
log "export: rtabmap-export --map ${EXPORT_OPT[*]:-(no --opt in the help of this rtabmap-export: its default)}"
rtabmap-export --map ${EXPORT_OPT[@]+"${EXPORT_OPT[@]}"} --output_dir "$OUT/export" --output "$NAME" "$DB" > "$OUT/export_map.log" 2>&1
rtabmap-export --poses --poses_format 11 ${EXPORT_OPT[@]+"${EXPORT_OPT[@]}"} --output_dir "$OUT/export" --output "$NAME" "$DB" > "$OUT/export_poses.log" 2>&1
POSES=$(ls "$OUT/export/"*poses*.txt 2>/dev/null | head -1)
python3 tools/car_run/rtabmap_watch.py readout --out "$OUT" --summary "$OUT/watch_summary.json" --db "$DB" \
  --info-txt "$OUT/rtabmap_info.txt" --poses "${POSES:-}" --driver-log "$OUT/driver.log" --judge-loop "$FORCE_3DOF" \
  --readout "$OUT/readout.json" | tee -a "$OUT/marks.txt"
sha256sum "$DB" "$OUT/export/"* > "$OUT/sha256.txt" 2>/dev/null
log "export done: $(ls "$OUT/export" | tr '\n' ' ')"
exit "$ODOM_RC"
