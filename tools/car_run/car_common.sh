#!/bin/bash
# Line E (2026-10-14): shared pieces of the one-command car start/stop (sourced, not run).
#   car_validate            the switch combination check (CAMERA POSE NAV DETECTOR CHASSIS YOLO_BACKEND, TELEOP, NAV_DRYRUN)
#                           (POSE odom + TELEOP, 10-15: the real-car drive chain -- wheel odometry pose, remote control)
#   car_default_dirs [PLATFORM]  CAR_STATE_DIR / CAR_RUNS_DIR defaults: board /data/fpga/car, host /data1 (unchanged)
#                           (POSE static, 10-15: the line-4 bench -- live camera fixed, no chassis, costmap)
#                           (POSE map, 10-15: rtabmap MAPPING run -- tools/car_run/rtabmap_car.sh map; live or replay,
#                            no navigation, no inference/runner; teleop to drive the car round)
#   car_stop_pid NAME PID   INT -> TERM -> KILL with timeouts, by pid only (never pkill -f)
#   car_pid_start PID       the process start time (/proc/PID/stat field 22): pid + start = identity
#   car_orbbec_busdev       the Orbbec camera's USB node from lsusb (board, live)
#   car_frame_drop_log      add the driver's enable_frame_drop_log:=true to LIVE_LAUNCH_ARGS (live; LIVE_FRAME_DROP_LOG)
#   car_start_retry ...     start a program again until a probe sees its data (the Orbbec driver's depth frames)
# Kept free of ROS and docker so that it also runs in a unit test and in a dry run.

CAR_CAMERAS="sim replay live"
CAR_POSES="reference rtabmap static odom map"
CAR_NAVS="costmap nav2"
CAR_DETECTORS="stub ssdlite yolo"
CAR_CHASSIS_KINDS="none pty uart"

# Run / state directory defaults (10-15): the Nano has no /data1. On the board (CAR_ARCH or uname -m = aarch64, or
# PLATFORM board = the live camera) /data/fpga/car/runs and /data/fpga/car/state; on the host unchanged. Given values win.
car_default_dirs() {   # [PLATFORM]
  if [ "${CAR_ARCH:-$(uname -m)}" = aarch64 ] || [ "${1:-}" = board ]; then
    : "${CAR_STATE_DIR:=/data/fpga/car/state}" "${CAR_RUNS_DIR:=/data/fpga/car/runs}"
  else
    : "${CAR_STATE_DIR:=${FPGA_HOME}/dev1/.t/wde}" "${CAR_RUNS_DIR:=${FPGA_SRC}/runs/car-runs}"
  fi
}

car_in() { local x=$1 w; shift; for w in "$@"; do [ "$x" = "$w" ] && return 0; done; return 1; }

# Presets: the three combinations named in the plan (switches given after a preset override it).
car_preset() {
  case "$1" in
    sim)    CAMERA=sim;    POSE=reference; NAV=nav2;    DETECTOR=yolo; CHASSIS=pty;;
    replay) CAMERA=replay; POSE=reference; NAV=costmap; DETECTOR=stub; CHASSIS=none;;
    live)   CAMERA=live;   POSE=rtabmap;   NAV=nav2;    DETECTOR=yolo; CHASSIS=uart;;
    *) return 1;;
  esac
}

# Derived values (after car_validate): SOURCE (data-source marker = the camera), PLATFORM (host|board),
# INF_DEVICE (cpu|cuda), NEED_GPU_LOCK (1 on the host when inference runs on a host GPU).
car_derive() {
  SOURCE=$CAMERA                      # sim -> sim, replay -> replay, live -> live (one variable, never a switch)
  PLATFORM=host; [ "$CAMERA" = live ] && PLATFORM=board
  case "$DETECTOR" in
    stub) INF_DEVICE=cpu;;
    ssdlite) INF_DEVICE=${INF_DEVICE:-cuda};;
    yolo) case "$YOLO_BACKEND" in torch|trt) INF_DEVICE=cuda;; ort) INF_DEVICE=cpu;; esac;;
  esac
  NEED_GPU_LOCK=0
  [ "$PLATFORM" = host ] && [ "$INF_DEVICE" = cuda ] && NEED_GPU_LOCK=1
  case "$SOURCE" in sim) SENSOR_SOURCE=simulation;; replay) SENSOR_SOURCE=replay;; live) SENSOR_SOURCE=hardware;; esac
  return 0
}

# Board Nav2 costmap (10-15; navigation.launch.py FPGA_NAV_COSTMAP, merged from real-car-nav2ctl b8a2bb5/9f30533).
# Board + nav2: an EMPTY value means "not given" (car_up.sh: FPGA_NAV_COSTMAP unset or empty) -> rolling: the live car
# has no /fpga/points publisher, so the simulation's obstacle layers would see nothing; rolling = the rolling-map local
# costmap (StaticLayer on /fpga/rolling_map/grid from host/ros_rolling_nav.py) + DWB (nav_controller_variants dwb.yaml).
# The board's behaviour before 10-15 (RPP heading precision, obstacle layers on /fpga/points) needs an explicit 'sim':
# an empty string cannot ask for it. Anywhere else (host Nav2, costmap runs) the value is not used: CAR_NAV_COSTMAP ''
# (the host launch keeps its own default sim, byte-identical). Sets CAR_NAV_COSTMAP; returns 2 for anything but
# rolling|sim on the board.
car_nav_costmap() {   # VALUE  (after car_validate / car_derive: NAV, PLATFORM)
  CAR_NAV_COSTMAP=""
  [ "$NAV" = nav2 ] && [ "$PLATFORM" = board ] || return 0
  CAR_NAV_COSTMAP=${1:-rolling}
  case "$CAR_NAV_COSTMAP" in rolling|sim) return 0;; esac
  echo "FPGA_NAV_COSTMAP 只能是 rolling|sim（给的是 '$1'；空＝板上默认 rolling）" >&2; CAR_NAV_COSTMAP=""; return 2
}

# One message per refused rule, on stderr; returns 2 if any rule refuses the combination.
car_validate() {
  local bad=0
  car_in "$CAMERA" $CAR_CAMERAS || { echo "相机 --camera 只能是 sim|replay|live（给的是 '$CAMERA'）" >&2; return 2; }
  car_in "$POSE" $CAR_POSES || { echo "位姿 --pose 只能是 reference|rtabmap|static|odom|map（给的是 '$POSE'）" >&2; return 2; }
  car_in "$NAV" $CAR_NAVS || { echo "导航 --nav 只能是 costmap|nav2（给的是 '$NAV'）" >&2; return 2; }
  car_in "$DETECTOR" $CAR_DETECTORS || { echo "识别 --detector 只能是 stub|ssdlite|yolo（给的是 '$DETECTOR'）" >&2; return 2; }
  car_in "$CHASSIS" $CAR_CHASSIS_KINDS || { echo "底盘 --chassis 只能是 none|pty|uart（给的是 '$CHASSIS'）" >&2; return 2; }
  if [ -z "${YOLO_BACKEND:-}" ]; then YOLO_BACKEND=torch; [ "$CAMERA" = live ] && YOLO_BACKEND=trt; fi
  car_in "$YOLO_BACKEND" torch ort trt || { echo "--yolo-backend 只能是 torch|ort|trt（给的是 '$YOLO_BACKEND'）" >&2; return 2; }
  # pose
  if [ "$CAMERA" = live ] && [ "$POSE" = reference ]; then
    echo "不合法：live 相机配 reference 位姿——真车没有参考位姿（reference 只来自仿真/回放），live 须用 rtabmap（相机固定在台架、无底盘时用 static；接底盘遥控时可用 odom）" >&2; bad=1; fi
  if [ "$CAMERA" != live ] && [ "$POSE" = rtabmap ]; then
    echo "不合法：$CAMERA 相机配 rtabmap 位姿——rtabmap 定位只接了真相机（live）；仿真/回放用 reference" >&2; bad=1; fi
  # static (2026-10-15, line 4 bench): the real camera fixed on a bench, no chassis -- map -> odom -> base_link is a
  # constant identity and that IS the true pose. Not mapping, not localization; only where the car cannot move.
  if [ "$POSE" = static ] && [ "$CAMERA" != live ]; then
    echo "不合法：$CAMERA 相机配 static 位姿——static 是真相机台架（车不动）的静态位姿；仿真/回放用 reference" >&2; bad=1; fi
  if [ "$POSE" = static ] && [ "$CHASSIS" != none ]; then
    echo "不合法：static 位姿配 $CHASSIS 底盘——接了底盘车会动，恒等位姿就不是真位姿；台架用 --chassis none" >&2; bad=1; fi
  # NAV_DRYRUN (car_up: CAR_NAV_DRYRUN=1; controller 10-17, REAL_CAR_NAV_GOLIVE_20261016.md §3 item 1 (b)): the board
  # Nav2 DRY RUN -- live --pose static --nav nav2 --chassis none and nothing else: the car cannot move (no chassis),
  # so Nav2 / the velocity guard / the runner's goals are checked with /fpga/cmd_vel_safe going nowhere. Not set: the
  # refusal below is word for word the one before. Set with any other combination: refused (it opens nothing else).
  # CTRLPS2 (2026-11-03): CAR_NAV_BACKEND nav2 (default) | kv | shadow; kv and shadow only with --nav nav2
  case "${NAV_BACKEND:-nav2}" in nav2|kv|shadow) ;; *) echo "CAR_NAV_BACKEND 只能是 nav2|kv|shadow（给的是 '${NAV_BACKEND}'）" >&2; return 2;; esac
  if [ "${NAV_BACKEND:-nav2}" != nav2 ] && [ "$NAV" != nav2 ]; then
    echo "不合法：CAR_NAV_BACKEND=${NAV_BACKEND} 需要 --nav nav2（kv 保留 Nav2 的 map_server + planner_server，shadow 让 Nav2 照常开车）" >&2; bad=1; fi
  case "${NAV_DRYRUN:-0}" in 0|1) ;; *) echo "CAR_NAV_DRYRUN 只能是 0 或 1（给的是 '${NAV_DRYRUN}'）" >&2; return 2;; esac
  if [ "${NAV_DRYRUN:-0}" = 1 ] && ! { [ "$CAMERA" = live ] && [ "$POSE" = static ] && [ "$NAV" = nav2 ] && [ "$CHASSIS" = none ]; }; then
    echo "不合法：CAR_NAV_DRYRUN=1 只用于板上 Nav2 空跑 live --pose static --nav nav2 --chassis none（给的是 $CAMERA/$POSE/$NAV/$CHASSIS）" >&2; bad=1; fi
  if [ "$POSE" = static ] && [ "$NAV" = nav2 ] && [ "${NAV_DRYRUN:-0}" != 1 ]; then
    echo "不合法：static 位姿配 nav2——台架车不动，Nav2 无从验证（板上 Nav2 入口也未定）；台架用 costmap" >&2; bad=1; fi
  # odom (2026-10-15, real-car drive chain): map -> odom identity (start = origin) + odom -> base_link from the wheel
  # odometry of the chassis link (host/ros_wheel_odom.py --tf-topic /tf). Not mapping, not localization. Only the live
  # car with the real chassis: replay and sim own odom -> base_link themselves (two publishers of one edge).
  if [ "$POSE" = odom ] && [ "$CAMERA" != live ]; then
    echo "不合法：$CAMERA 相机配 odom 位姿——odom 是真车轮式里程位姿；回放/仿真自己发 odom->base_link，再起轮式里程会双发同一条 TF" >&2; bad=1; fi
  if [ "$POSE" = odom ] && [ "$CHASSIS" != uart ]; then
    echo "不合法：odom 位姿配 $CHASSIS 底盘——轮式里程来自真底盘串口，须 --chassis uart" >&2; bad=1; fi
  # odom + nav2 (10-17, REAL_CAR_NAV_GOLIVE_20261016.md section 12): allowed -- the board Nav2 entry is in main (nav
  # go-live rounds 1-3). The two rules above already pin it to live + chassis uart (the wheel odometry needs the real
  # chassis); teleop + nav2 and CAR_NAV_DRYRUN stay refused below / above. The map: Nav2's map_server needs the mapping
  # export (CAR_MAP_YAML or the one <RTABMAP_DB dir>/run_<stem>/export/*.yaml) -- known only after car_up derives it,
  # so car_up.sh (exit 2, dry run included) and car_inner.sh refuse odom + nav2 without it, not this switch check.
  # map (2026-10-15, real car): an rtabmap MAPPING run (rtabmap_car.sh map) -- a new database, not localization. Live
  # (the real camera) or replay (the host test); never with Nav2 (there is no map to navigate on yet)
  if [ "$POSE" = map ] && [ "$CAMERA" = sim ]; then
    echo "不合法：sim 相机配 map 位姿——建图档只接真相机（live）与回放（replay，宿主验证）" >&2; bad=1; fi
  if [ "$POSE" = map ] && [ "$NAV" = nav2 ]; then
    echo "不合法：map 位姿配 nav2——建图趟还没有地图可导航；建图用 --nav costmap（建图档不起导航）" >&2; bad=1; fi
  # teleop (2026-10-15): TELEOP=1 (car_up: CAR_TELEOP=1) starts host/ros_teleop_safe.py, the only /fpga/cmd_vel_safe
  # publisher; it is mutually exclusive with the velocity guard (sim: in the Gazebo container) and with navigation runs
  case "${TELEOP:-0}" in 0|1) ;; *) echo "CAR_TELEOP 只能是 0 或 1（给的是 '${TELEOP}'）" >&2; return 2;; esac
  if [ "${TELEOP:-0}" = 1 ]; then
    if [ "$CHASSIS" != uart ] && [ "$CHASSIS" != pty ]; then
      echo "不合法：遥控配 $CHASSIS 底盘——遥控发 /fpga/cmd_vel_safe 给底盘驱动，须 --chassis uart（真车）或 pty（假单片机）" >&2; bad=1; fi
    # teleop + nav2 (2026-10-18, docs/design/TELEOP_NAV_ARB_20261018.md): the ARBITRATION mode -- the velocity guard
    # (board Nav2 launch / sim Gazebo container) publishes /fpga/nav/cmd_vel_safe and host/ros_teleop_safe.py --nav-topic
    # stays the only /fpga/cmd_vel_safe publisher. Only this combination opens: the two refusals below are word for word
    # the ones before, for every other combination.
    if [ "$CAMERA" = sim ] && [ "$NAV" != nav2 ]; then
      echo "不合法：遥控配 sim——仿真里速度守卫（Gazebo 容器）已发 /fpga/cmd_vel_safe，二者互斥" >&2; bad=1; fi
    if [ "$NAV" = nav2 ] && [ "${NAV_DRYRUN:-0}" = 1 ]; then
      echo "不合法：遥控配 nav2——遥控趟不是导航趟（不经运动许可），二者互斥；遥控用 --nav costmap" >&2; bad=1; fi
  fi
  # in-stack mapping (2026-10-18, MAPPING=1 = car_up CAR_MAPPING=1; docs/design/TELEOP_NAV_ARB_20261018.md): rtabmap
  # maps (wheel odometry, its grid on /fpga/rtabmap/map) BESIDE Nav2, which starts on a blank map and is handed the
  # export by "mapping-stop". Live: --pose odom (wheel odometry, chassis uart); sim: the host control-flow check with
  # the mapping stand-in (no rtabmap in the host image). Never with --pose map/rtabmap (they run rtabmap themselves).
  case "${MAPPING:-0}" in 0|1) ;; *) echo "CAR_MAPPING 只能是 0 或 1（给的是 '${MAPPING}'）" >&2; return 2;; esac
  if [ "${MAPPING:-0}" = 1 ]; then
    if [ "$NAV" != nav2 ]; then
      echo "不合法：CAR_MAPPING=1 配 $NAV——并栈建图是给 Nav2 同趟换图用的，须 --nav nav2；只建图用 --pose map" >&2; bad=1; fi
    if [ "$CAMERA" = replay ]; then
      echo "不合法：CAR_MAPPING=1 配 replay——并栈建图只接真车（live --pose odom）与宿主仿真替身（sim）" >&2; bad=1; fi
    if [ "$CAMERA" = live ] && [ "$POSE" != odom ]; then
      echo "不合法：CAR_MAPPING=1 配 $POSE 位姿——并栈建图用轮式里程位姿 --pose odom（map/rtabmap 档自己起 rtabmap）" >&2; bad=1; fi
  fi
  # navigation
  if [ "$CAMERA" = replay ] && [ "$NAV" = nav2 ]; then
    echo "不合法：replay 相机配 nav2——回放的车沿录好的轨迹走，Nav2 开不动它；回放用 costmap" >&2; bad=1; fi
  if [ "$CAMERA" = sim ] && [ "$NAV" = costmap ]; then
    echo "不合法：sim 相机配 costmap——仿真车只由 Nav2 驱动（单独代价地图未接仿真）；只看建图用 replay" >&2; bad=1; fi
  # detector
  if [ "$DETECTOR" = stub ] && [ "$CAMERA" != replay ]; then
    echo "不合法：stub 识别配 $CAMERA 相机——stub 读回放相机写进 RGB 的帧号，只能配 replay" >&2; bad=1; fi
  if [ "$CAMERA" = replay ] && [ "$DETECTOR" != stub ]; then
    echo "不合法：replay 相机配 $DETECTOR 识别——回放的 RGB 只带帧号、没有图像，真识别器认不出东西；replay 用 stub" >&2; bad=1; fi
  if [ "$CAMERA" = live ] && [ "$DETECTOR" = ssdlite ]; then
    echo "不合法：live 配 ssdlite——板上识别只有 YOLO（TensorRT）" >&2; bad=1; fi
  if [ "$DETECTOR" = yolo ]; then
    if [ "$CAMERA" = live ] && [ "$YOLO_BACKEND" != trt ]; then
      echo "不合法：板上（live）YOLO 只走 trt（给的是 $YOLO_BACKEND）" >&2; bad=1; fi
    if [ "$CAMERA" != live ] && [ "$YOLO_BACKEND" = trt ]; then
      echo "不合法：宿主（$CAMERA）YOLO 只能 torch（GPU）或 ort（CPU），trt 是板上的" >&2; bad=1; fi
  fi
  # chassis
  if [ "$CHASSIS" = uart ] && [ "$CAMERA" != live ]; then
    echo "不合法：$CAMERA 相机配 uart 底盘——真串口接的是真车，仿真/回放的车不会跟着动；用 pty（mcu_sim 链）或 none" >&2; bad=1; fi
  if [ "$CHASSIS" = pty ] && [ "$CAMERA" = live ]; then
    echo "不合法：live 相机配 pty 底盘——pty 另一端是假单片机，真车不会动；用 uart 或 none" >&2; bad=1; fi
  [ "$bad" = 0 ] || return 2
  car_derive
}

# /proc/PID/stat after "PID (comm) " (comm may hold spaces): field 1 = state, field 20 = start time
# 10-26 owner：重成功率，放宽自「就绪后任一子进程退出即整栈停」(D-038 第 2 条)。CAR_FAILSTOP:
#   0 (default) -- hub / bridge / preview / producer / navmap (10-29, Nav2's live global map; Nav2 keeps the last
#                  map it got meanwhile) are restarted with a backoff, every restart in marks.txt
#                  (the producer since 10-26 2nd pass: its relaunch re-loads roll.so and re-HELLOs the KV, same
#                  command line); any other child exiting still stops the stack (cleanup INTs rtabmap, which saves its db)
#   1           -- the old fail-stop: any child exiting stops the stack
#   log         -- the old CAR_FAILSTOP=0: an exit is only logged (the cut test that stops the producer)
car_exit_action() {   # FAILSTOP NAME -> restart | stop | log
  # colour (car screen colour sender, display only, comp-integ 10-26): an exit is only logged in every mode
  case "$2" in colour) echo log; return;; esac
  case "$1" in
    1) echo stop;;
    log) echo log;;
    *) case "$2" in hub|bridge|preview|producer|navmap|kvnav) echo restart;; kvshadow) echo log;; *) echo stop;; esac;;
  esac
}
car_restart_backoff() {   # N (1-based restart count) -> seconds before the restart: 1 2 4 8 16 30 30 ... (no cap on N)
  local n=$1 d=1
  while [ "$n" -gt 1 ] && [ "$d" -lt 30 ]; do d=$((d * 2)); n=$((n - 1)); done
  [ "$d" -gt 30 ] && d=30
  echo "$d"
}
car_supervise_once() {   # one pass of car_inner.sh's supervision loop (10-26: a function so tests run the real code)
  # globals: NAMES PIDS STARTS CMDS OUTS (car_inner launch), ONESHOT, RESTARTS RESTART_AT (declare -A), O, CAR_FAILSTOP;
  # the caller defines mark and fail
  local k rc d
  for k in "${!NAMES[@]}"; do
    case "$ONESHOT" in *" ${NAMES[$k]} "*) continue;; esac
    if [ -n "${RESTART_AT[$k]:-}" ]; then   # an exited child waiting for its restart time
      if [ "$(date +%s)" -ge "${RESTART_AT[$k]}" ]; then
        unset "RESTART_AT[$k]"
        eval "exec ${CMDS[$k]}" >> "$O/${OUTS[$k]}" 2>&1 &   # exec: the tracked pid IS the child (INT at stop reaches it)
        PIDS[$k]=$!; STARTS[$k]="$(car_pid_start "${PIDS[$k]}")"
        echo "${NAMES[$k]} ${PIDS[$k]} ${STARTS[$k]}" >> "$O/pids.txt"
        [ -f "$O/${NAMES[$k]}.pid" ] && echo "${PIDS[$k]}" > "$O/${NAMES[$k]}.pid"   # producer.pid follows the restart
        mark "RESTARTED ${NAMES[$k]} n=${RESTARTS[$k]} pid=${PIDS[$k]}"
      fi
      continue
    fi
    if ! car_pid_alive "${PIDS[$k]}" "${STARTS[$k]}" || [ "$(car_pid_state "${PIDS[$k]}")" = Z ]; then
      wait "${PIDS[$k]}" 2>/dev/null; rc=$?
      mark "EXITED ${NAMES[$k]} rc=$rc"
      case "$(car_exit_action "$CAR_FAILSTOP" "${NAMES[$k]}")" in
        stop) fail "${NAMES[$k]} exited after READY (rc=$rc): fail-stop" 5;;
        restart)
          RESTARTS[$k]=$(( ${RESTARTS[$k]:-0} + 1 )); d=$(car_restart_backoff "${RESTARTS[$k]}")
          RESTART_AT[$k]=$(( $(date +%s) + d ))
          mark "RESTART ${NAMES[$k]} n=${RESTARTS[$k]} in ${d}s (rc=$rc)"
          echo "WARN ${NAMES[$k]} exited after READY (rc=$rc): restart #${RESTARTS[$k]} in ${d} s" >&2;;
        *) [ "${NAMES[$k]}" = colour ] && mark "display-only colour exited: car screen colour stops, the stack continues"
           NAMES[$k]="${NAMES[$k]}(exited)"; ONESHOT="$ONESHOT${NAMES[$k]} ";;
      esac
    fi
  done
}
car_pid_stat() { local l; l=$(cat "/proc/$1/stat" 2>/dev/null) || return 1; echo "${l##*) }"; }
car_pid_start() { local l; l=$(car_pid_stat "$1") || return 1; set -- $l; echo "${20}"; }

# alive by identity: the pid exists AND it is the same process (start time unchanged; pids get reused)
car_pid_alive() {   # PID [START]
  local s; s=$(car_pid_start "$1") || return 1
  [ -n "$s" ] || return 1
  [ -z "${2:-}" ] || [ "$s" = "$2" ]
}

# INT, CAR_INT_WAIT_S (10), TERM, CAR_TERM_WAIT_S (5), KILL. Appends to CAR_RC_FILE (if set) what it had to do.
# Works on processes that are not children of this shell too (no wait on them: the rc is the child's own business).
car_stop_pid() {   # NAME PID [START]
  local n=$1 p=$2 st=${3:-} i log=${CAR_RC_FILE:-/dev/null}
  local iw=${CAR_INT_WAIT_S:-10} tw=${CAR_TERM_WAIT_S:-5}
  if ! car_pid_alive "$p" "$st"; then
    wait "$p" 2>/dev/null; echo "${n}_rc=$? (already exited)" >> "$log"; return 0; fi
  kill -INT "$p" 2>/dev/null
  for ((i=0; i<iw*10; i++)); do car_pid_alive "$p" "$st" && [ "$(car_pid_state "$p")" != Z ] || break; sleep 0.1; done
  if car_pid_alive "$p" "$st" && [ "$(car_pid_state "$p")" != Z ]; then
    echo "$n alive ${iw} s after INT, TERM" >> "$log"; kill -TERM "$p" 2>/dev/null
    for ((i=0; i<tw*10; i++)); do car_pid_alive "$p" "$st" && [ "$(car_pid_state "$p")" != Z ] || break; sleep 0.1; done
    if car_pid_alive "$p" "$st" && [ "$(car_pid_state "$p")" != Z ]; then
      echo "$n alive after TERM, KILL" >> "$log"; kill -KILL "$p" 2>/dev/null
      for ((i=0; i<50; i++)); do car_pid_alive "$p" "$st" && [ "$(car_pid_state "$p")" != Z ] || break; sleep 0.1; done
    fi
  fi
  local rc; wait "$p" 2>/dev/null; rc=$?
  if [ "$rc" = 127 ]; then echo "${n}_stopped (not a child of this shell: rc not visible)" >> "$log"
  else echo "${n}_rc=$rc" >> "$log"; fi
}

car_pid_state() { local l; l=$(car_pid_stat "$1") || return 1; set -- $l; echo "$1"; }

# The Orbbec camera's USB node /dev/bus/usb/BBB/DDD (first device of USB vendor 2bc5), as the real-camera board
# scripts resolve it (docs/design/integ_car_chain_v3_20261011/nano_v3.sh livechain, tools/nano_ros2/run_orbbec_probe.sh):
# lsusb -d 2bc5: | head -1, bus = field 2, device = field 4 without its ':'. Those scripts pass exactly that node
# with docker --device (no --privileged, no -v /dev/bus/usb). car_up.sh used to as well; since 10-17 it uses this
# node only as the presence check and mounts the whole USB root with a 'c 189:* rw' device cgroup rule instead (the
# camera can re-enumerate with the next device number in a driver start: docs/design/CAM_USB_REENUM_20261017.md).
# Prints nothing and returns 1 when no Orbbec device is listed. Stand-ins: CAR_LSUSB (lsusb), CAR_USB_ROOT (/dev/bus/usb).
car_orbbec_busdev() {
  local d
  d=$(${CAR_LSUSB:-lsusb} -d 2bc5: 2>/dev/null | head -1 \
      | awk -v root="${CAR_USB_ROOT:-/dev/bus/usb}" '{printf "%s/%s/%s", root, $2, substr($4,1,3)}')
  [[ "$d" =~ /[0-9]{3}/[0-9]{3}$ ]] || return 1
  echo "$d"
}

# The Orbbec driver's frame-drop log (10-17, colour-rate dig 10-16 advice): real-camera starts add
# enable_frame_drop_log:=true to LIVE_LAUNCH_ARGS so that the driver log (driver.log) shows afterwards where frames
# were dropped ("Frame drop detected: stage=SDK_RECEIVE|ROS_PUBLISH"). The parameter is declared by the board image's
# gemini2.launch.py (fpga-ros:humble-nav-orbbec-20260927 and its scipy derivative, default false; checked 10-17). LIVE_FRAME_DROP_LOG (true)
# = false leaves LIVE_LAUNCH_ARGS as given; an enable_frame_drop_log:= already in LIVE_LAUNCH_ARGS is kept as given.
# Only the live camera paths call this (replay/sim start no driver). Returns 1 on a bad LIVE_FRAME_DROP_LOG.
car_frame_drop_log() {
  : "${LIVE_FRAME_DROP_LOG:=true}"
  case "$LIVE_FRAME_DROP_LOG" in
    true) case " $LIVE_LAUNCH_ARGS " in
            *" enable_frame_drop_log:="*) ;;
            *) LIVE_LAUNCH_ARGS="$LIVE_LAUNCH_ARGS enable_frame_drop_log:=true";;
          esac;;
    false) ;;
    *) echo "LIVE_FRAME_DROP_LOG must be true or false (given '$LIVE_FRAME_DROP_LOG')" >&2; return 1;;
  esac
}

# Start a program whose start is unreliable and start it again until a probe sees its data (the Orbbec driver:
# tools/car_geo_producer/rig_inner.sh 10-13, the depth stream does not come up on about two starts in three -- 10 good
# of 28 on the board). Judged by DATA, not by a log line: board 10-15 (integration l_chain32) the driver printed its
# depth profile line "depth Frame ..." and then sent no depth frame at all, so the rig's grep counted a dead start
# as good (controller 10-15 00:03).
#   CAR_RETRY_PROBE=(probe command ...)   car_start_retry TRIES WAIT_S GAP_S LOG CMD...
# Each try: CMD in the background (stdout+stderr to LOG), then the probe in the background (stdout+stderr to LOG minus
# .log plus _probe_N.log; it must exit 0 once the data arrived, non-zero otherwise, and bound its own wait to about
# WAIT_S). Success = the probe exited 0 within WAIT_S + 5 s while CMD still runs. Otherwise (probe failed or overran,
# or CMD exited) the try is stopped: the probe (KILL), CMD and its direct children (pgrep -P: by parent pid, never
# pkill -f) INT, up to 10 s, then KILL; LOG is kept as LOG minus .log plus _failed_start_N.log; GAP_S pause.
# Every try appends "<time> camera driver try=N depth_stream=0|1 probe_rc=R" to CAR_MARKS_FILE (if set).
# CAR_RETRY_BEFORE=(command ...) (optional, 10-17): run before every try from the 2nd on (after the pause), output to
# LOG minus .log plus _before_N.log, "<time> camera driver before try=N rc=R" to CAR_MARKS_FILE; its exit status does
# not stop the retry (car_inner.sh: fastdds shm clean, the unowned shared-memory segments a failed start left behind).
# CAR_RETRY_NOTE=(command ...) (optional, 10-17): run before EVERY try (the 1st too, after CAR_RETRY_BEFORE), the first
# line of its stdout -> "<time> camera driver usb try=N <line>" in CAR_MARKS_FILE (car_inner.sh: the camera's USB
# node bus/dev as the container sees it, so that a re-enumeration between tries is on record).
# Success: CAR_RETRY_PID = the running process, CAR_RETRY_TRY = its try, return 0. All TRIES failed: return 1,
# nothing left running. Call it directly (not in $(...)): the background process must be a child of this shell.
car_start_retry() {
  local tries=$1 wait_s=$2 gap_s=$3 log=$4 try i p q prc started kids
  shift 4
  [ "${#CAR_RETRY_PROBE[@]}" -gt 0 ] || { echo "car_start_retry: CAR_RETRY_PROBE not set" >&2; return 2; }
  CAR_RETRY_PID="" CAR_RETRY_TRY=""
  for ((try=1; try<=tries; try++)); do
    if [ "$try" -gt 1 ] && [ -n "${CAR_RETRY_BEFORE+x}" ]; then
      "${CAR_RETRY_BEFORE[@]}" > "${log%.log}_before_$try.log" 2>&1
      i=$?
      [ -n "${CAR_MARKS_FILE:-}" ] && echo "$(date +%s.%N) camera driver before try=$try rc=$i" >> "$CAR_MARKS_FILE"
    fi
    if [ -n "${CAR_RETRY_NOTE+x}" ]; then
      i=$("${CAR_RETRY_NOTE[@]}" 2>&1 | head -1)
      [ -n "${CAR_MARKS_FILE:-}" ] && echo "$(date +%s.%N) camera driver usb try=$try ${i:-<no output>}" >> "$CAR_MARKS_FILE"
    fi
    "$@" > "$log" 2>&1 &
    p=$!
    "${CAR_RETRY_PROBE[@]}" > "${log%.log}_probe_$try.log" 2>&1 &
    q=$!
    started=0 prc=timeout
    for ((i=0; i<(wait_s+5)*5; i++)); do
      if ! car_pid_alive "$q" || [ "$(car_pid_state "$q")" = Z ]; then
        wait "$q"; prc=$?
        [ "$prc" = 0 ] && car_pid_alive "$p" && [ "$(car_pid_state "$p")" != Z ] && started=1
        break
      fi
      car_pid_alive "$p" && [ "$(car_pid_state "$p")" != Z ] || { prc=driver_exited; break; }
      sleep 0.2
    done
    if car_pid_alive "$q"; then kill -KILL "$q" 2>/dev/null; wait "$q" 2>/dev/null; fi
    [ -n "${CAR_MARKS_FILE:-}" ] && echo "$(date +%s.%N) camera driver try=$try depth_stream=$started probe_rc=$prc" >> "$CAR_MARKS_FILE"
    if [ "$started" = 1 ]; then CAR_RETRY_PID=$p CAR_RETRY_TRY=$try; return 0; fi
    cp "$log" "${log%.log}_failed_start_$try.log" 2>/dev/null
    kids=$(pgrep -P "$p" 2>/dev/null)
    kill -INT $kids "$p" 2>/dev/null
    for ((i=0; i<50; i++)); do car_pid_alive "$p" && [ "$(car_pid_state "$p")" != Z ] || break; sleep 0.2; done
    kill -KILL $kids "$p" 2>/dev/null
    wait "$p" 2>/dev/null
    [ "$try" -lt "$tries" ] && sleep "$gap_s"
  done
  return 1
}

# The hub address a PC command must connect to, read from the run (2026-10-15): car.json "hub" {bind, port}
# (car_up.sh writes it before the stack starts), else the marks.txt line "ready hub ADDR:PORT" (car_inner.sh, written
# once the hub said "hub listening"; runs from before car.json had the key). Both present and different -> refused.
# Never a 127.0.0.1 default: the board hub binds only the car's LAN address (CAR_HUB_BIND), so a guessed loopback
# address would leave the e-stop unconnected. CAR_HUB_HOST / HUBPORT (env) override the recorded address / port.
# Prints "HOST PORT"; returns 2 with the reason on stderr when there is nothing to read.
car_hub_addr() {   # RUN
  python3 - "$1" "${CAR_HUB_HOST:-}" "${HUBPORT:-}" <<'PY'
import json, os, sys
run, host_env, port_env = sys.argv[1], sys.argv[2], sys.argv[3]
rec = {}
try:
    hub = json.load(open(os.path.join(run, 'car.json'))).get('hub') or {}
    if hub.get('bind') and hub.get('port'):
        rec['car.json'] = (str(hub['bind']), int(hub['port']))
except (OSError, ValueError, TypeError) as exc:
    sys.stderr.write('car_hub_addr: %s/car.json: %s\n' % (run, exc))
try:
    marks = [l.split()[3] for l in open(os.path.join(run, 'marks.txt')) if l.split()[1:3] == ['ready', 'hub']]
    if marks:
        h, _, p = marks[-1].rpartition(':')
        rec['marks.txt'] = (h, int(p))
except (OSError, ValueError, IndexError):
    pass
if len(set(rec.values())) > 1:
    sys.stderr.write('car_hub_addr: %s: car.json hub %s:%d and marks.txt ready hub %s:%d disagree\n'
                     % ((run,) + rec['car.json'] + rec['marks.txt']))
    sys.exit(2)
bind, port = next(iter(rec.values())) if rec else (None, None)
host = host_env or bind
port = int(port_env) if port_env else port
if not host or not port:
    sys.stderr.write('car_hub_addr: %s: no hub address recorded (car.json "hub" / marks.txt "ready hub"); '
                     'set CAR_HUB_HOST=<the hub address> (and HUBPORT) -- not guessing 127.0.0.1\n' % run)
    sys.exit(2)
print(host, port)
PY
}
