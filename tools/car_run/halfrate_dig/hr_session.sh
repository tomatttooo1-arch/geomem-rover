#!/bin/bash
# Camera half-rate dig, 10-17 (docs/design/COLOUR_RATE_NAVMODE_20261017.md section 7.3, prereg E1-E4). Session tool of
# the run (derived from runs/nav-golive-r3-20261017-results/tools/ng_session.sh; KV steps the same).
#   bash hr_session.sh run          (server; detached with setsid nohup): waits for kv260 + orin-nano + camera (never
#       steals), deploys HEAD of W by git archive, Nano checks, then
#       E1/E2 driver-only cold starts x N_DRV (half-rate -> the manual-exposure switch in the same start),
#       E3 x N_AE starts with color_ae_max_exposure:=30000,
#       KV260 A2 + one kv_server, E4 full chain car_up live --pose static --chassis none --nav costmap (the A check on,
#       frame log on) up to N_CHAIN starts, stop at the first chain start that restarted a half-rate try and got READY.
#       EXIT trap: driver container / car_down -> kv_server stop -> m7_pl restore (read back 0x0117bc0a) -> pull ->
#       locks released (by token).
#   bash hr_session.sh nano <sub> ...   (Orin Nano)     bash hr_session.sh kv <sub> ...   (KV260, root via sudo -S)
# 10-17 second baton (doc section 8.1): MODE and ARM.
#   MODE  (dig | a space-separated subset of "e2chain drvinf"; default dig = the flow above, unchanged)
#         e2chain: KV260 A2 + kv_server, up to N_CHAIN (10) full-chain starts with CAR_CAMERA_RATE_CHECK=0; in each
#                  stack (nano chk): A's probe + a 10 s state window; half-rate -> the manual-exposure switch
#                  20000 / 33000 / 30000 in the same start; the first normal one -> a 20000 control; stop after the
#                  first half-rate switch.
#         drvinf:  N_INF (12) driver-only cold starts, each right after a fresh inference service (TensorRT YOLO,
#                  the board command of car_up) is ready; the service stopped by pid after the driver.
#   ARM   (Nano runs dir $NB/runs_<ARM>, state dir $NB/st_<ARM>; refused when runs_<ARM> exists; empty = runs/st)
# Never pkill -f: processes by pid, containers by name. No chassis: --chassis none always.
set -u
NB=/data/fpga/hrdig
KB=/home/ubuntu/fpga/hrdig
A2_TEST=/home/ubuntu/fpga/pl-merge-a2-0da5041/pkg/m7pl_test
BASE_TEST=/home/ubuntu/fpga/package-100mhz/m7pl_test
A2_BID=0x00da5041; BASE_BID=0x0117bc0a
KV_PORT=47800
SARGS="--core pl --device devmem --wait irq --roi-core pl --port $KV_PORT --max-idle-s 900"
IMG=fpga-ros:humble-nav-orbbec-scipy-20261017
DC=hrdig-cam
LLA="color_width:=640 color_height:=480 color_fps:=30 color_format:=RGB depth_width:=640 depth_height:=400 depth_fps:=30 depth_format:=Y16 enable_ir:=false enable_point_cloud:=false depth_registration:=true align_mode:=HW publish_tf:=false"
SRC="source /opt/ros/humble/setup.bash && source /opt/orbbec_ws/install/setup.bash"
wall() { date +%s.%N; }

# =============================================================================================== Nano side
nano_main() {
  local sub=$1; shift
  local RUNS=$NB/runs${ARM:+_$ARM} ST=$NB/st${ARM:+_$ARM} REPO=$NB/repo
  export CAR_STATE_DIR=$ST CAR_RUNS_DIR=$RUNS
  stack_of() { python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['containers']['stack'])" "$RUNS/$1/car.json" 2>/dev/null; }
  case "$sub" in
  env)   # FILE
    { date -Is; nvpmodel -q 2>&1; uptime; free -m | head -2
      echo "listen 47431: $(ss -ltn 2>/dev/null | grep -c ':47431 ')"
      echo "orbbec: $(lsusb -d 2bc5: 2>/dev/null | head -1)"
      echo "car.current: $(readlink $ST/car.current 2>/dev/null || echo none)"
      echo "car.current(default state): $(readlink /data/fpga/car/state/car.current 2>/dev/null || echo none)"
      docker ps -a --format '{{.Names}} {{.Image}} {{.Status}}'; top -bn1 | head -12; } > "$1" 2>&1 ;;
  drv)   # NAME I EXPO(0|1) [extra launch args...]: one driver-only cold start (E1; E2 when EXPO=1; E3 with extras)
    local name=$1 i=$2 expo=$3; shift 3
    local R=$RUNS/$name/s$i in=/w/runs${ARM:+_$ARM}/$name/s$i busdev prc
    mkdir -p "$R/sdklog"
    busdev=$(lsusb -d 2bc5: | head -1 | awk '{printf "/dev/bus/usb/%s/%s", $2, substr($4,1,3)}')
    [ -c "$busdev" ] || { echo "no Orbbec camera ($busdev)" | tee "$R/error.txt"; return 3; }
    docker rm -f "$DC" > /dev/null 2>&1
    { echo "start $(date -Is) $(wall) busdev=$busdev extra=$*"; cat /proc/loadavg; } > "$R/marks.txt"
    docker run -d --init --name "$DC" --network host --user "$(id -u):$(id -g)" -e ROS_DOMAIN_ID=63 -e ROS_LOCALHOST_ONLY=1 \
      -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog -e PYTHONDONTWRITEBYTECODE=1 --device "$busdev" -v "$NB:/w" \
      -v "$R/sdklog:/tmp/.ros/Log" --entrypoint bash "$IMG" -c "$SRC && FASTRTPS_DEFAULT_PROFILES_FILE=/w/repo/tools/nano_ros2/fastdds_shm8_localhost.xml exec ros2 launch orbbec_camera gemini2.launch.py $LLA enable_frame_drop_log:=true frame_timestamp_csv_file:=$in/drv_frames.csv $* > $in/driver.log 2>&1" \
      > "$R/cid.txt" 2>&1 || { echo "docker run failed: $(cat "$R/cid.txt")"; return 3; }
    docker exec "$DC" bash -c "$SRC && python3 /w/repo/tools/car_run/depth_probe.py --topic /camera/depth/image_raw --count 30 --timeout-s 20 --rate-s 3 --color-topic /camera/color/image_raw" \
      > "$R/rate_probe.log" 2>&1
    prc=$?; echo "probe_rc=$prc $(date -Is) $(wall)" >> "$R/marks.txt"
    local P="$SRC && python3 /w/repo/tools/car_run/halfrate_dig/crdig_probe.py"
    if [ "$prc" != 1 ]; then
      docker exec "$DC" bash -c "$P --label s$i --out $in/state.json --warm 0 --seconds 10" >> "$R/probe.log" 2>&1
      echo "state done $(date -Is)" >> "$R/marks.txt"
      if [ "$expo" = 1 ] && [ "$prc" = 3 ]; then   # E2 in a half-rate start: 20000, then 33000 (reverse control)
        docker exec "$DC" bash -c "$P --label s${i}_m20000 --out $in/manual20000.json --set-ae 0 --set-exp 20000 --warm 3 --seconds 30" >> "$R/probe.log" 2>&1
        docker exec "$DC" bash -c "$P --label s${i}_m33000 --out $in/manual33000.json --set-exp 33000 --warm 3 --seconds 30" >> "$R/probe.log" 2>&1
        echo "manual done (half) $(date -Is)" >> "$R/marks.txt"
      elif [ "$expo" = 1 ] && [ "$prc" = 0 ] && [ ! -e "$RUNS/$name/normal_control_done" ]; then   # E2 control, once
        docker exec "$DC" bash -c "$P --label s${i}_m20000 --out $in/manual20000.json --set-ae 0 --set-exp 20000 --warm 3 --seconds 30" >> "$R/probe.log" 2>&1
        touch "$RUNS/$name/normal_control_done"; echo "manual done (normal control) $(date -Is)" >> "$R/marks.txt"
      fi
    fi
    docker stop -t 15 "$DC" > /dev/null 2>&1; docker rm "$DC" > /dev/null 2>&1
    echo "stopped $(date -Is) $(wall)" >> "$R/marks.txt"
    echo "drv $name s$i probe_rc=$prc $(tail -1 "$R/probe.log" 2>/dev/null | cut -c1-200)" ;;
  up)   # NAME [RATE_CHECK 0|1]: car_up (E4; E2c with 0), foreground; car_up's own rollback on a refusal
    local name=$1; mkdir -p "$RUNS" "$ST"
    [ -n "${2:-}" ] && export CAR_CAMERA_RATE_CHECK=$2
    export CAR_BOARD_DRAFT=1 CAR_HUB_BIND=127.0.0.1 LIVE_LAUNCH_ARGS="$LLA enable_frame_drop_log:=true frame_timestamp_csv_file:=/o/drv_frames.csv"
    echo "up $(date -Is) $(wall)" > "$RUNS/$name.up_t"
    bash "$REPO/tools/car_run/car_up.sh" live --pose static --chassis none --nav costmap --run-dir "$RUNS/$name" \
      > "$RUNS/$name.up.log" 2>&1 < /dev/null
    local rc=$?; echo "rc=$rc $(date -Is) $(wall)" >> "$RUNS/$name.up_t"; echo "up $name rc=$rc"; return $rc ;;
  measure)   # NAME: exposure / gain / brightness / camera_info Hz in the stack (10 s) + the SDK log, before car_down
    local name=$1 s R; s=$(stack_of "$name"); R=$RUNS/$name; mkdir -p "$R/hr"
    cp "$REPO/tools/car_run/halfrate_dig/crdig_probe.py" "$R/hr/"
    docker exec -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog "$s" bash -c "$SRC && python3 /o/hr/crdig_probe.py --label $name --out /o/hr/state.json --warm 0 --seconds 10" > "$R/hr/probe.log" 2>&1
    docker cp "$s:/tmp/.ros/Log" "$R/hr/sdklog" > /dev/null 2>&1 || docker cp "$s:/tmp/roslog" "$R/hr/roslog" > /dev/null 2>&1
    ls -R "$R/hr" | head -20; tail -1 "$R/hr/probe.log" ;;
  chk)   # NAME: E2c (section 8.1) in the chain's stack: A's probe (3 s) + 10 s state window -> half | normal | undecided;
         # half -> manual 20000 / 33000 / 30000 (30 s each, same start); the first normal of this ARM -> a 20000 control
    local name=$1 s R prc v e; s=$(stack_of "$name"); R=$RUNS/$name; mkdir -p "$R/hr"
    [ -n "$s" ] && [ "$(docker inspect -f '{{.State.Running}}' "$s" 2>/dev/null)" = true ] || { echo "chk $name: no running stack ($s)"; return 3; }
    cp "$REPO/tools/car_run/halfrate_dig/crdig_probe.py" "$REPO/tools/car_run/depth_probe.py" "$R/hr/"
    local X=(docker exec -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog "$s" bash -c)
    local P="$SRC && python3 /o/hr/crdig_probe.py"
    echo "chk start $(date -Is) $(wall)" > "$R/hr/marks.txt"
    "${X[@]}" "$SRC && python3 /o/hr/depth_probe.py --topic /camera/depth/image_raw --count 30 --timeout-s 20 --rate-s 3 --color-topic /camera/color/image_raw" \
      > "$R/hr/rate_probe.log" 2>&1
    prc=$?
    "${X[@]}" "$P --label $name --out /o/hr/state.json --warm 0 --seconds 10" > "$R/hr/probe.log" 2>&1
    v=$(python3 -c '
import json, sys
try:
    s = json.load(open(sys.argv[1])); c = s["color"].get("hz"); d = s["depth"].get("hz")
except Exception:
    print("undecided"); sys.exit()
rc = sys.argv[2]
if rc == "3" and c is not None and d is not None and c <= 16.5 and d >= 30.5: print("half")
elif rc == "0" and c is not None and c >= 29.5: print("normal")
else: print("undecided")' "$R/hr/state.json" "$prc")
    echo "probe_rc=$prc verdict=$v $(date -Is) $(wall)" >> "$R/hr/marks.txt"
    if [ "$v" = half ]; then
      "${X[@]}" "$P --label ${name}_m20000 --out /o/hr/manual20000.json --set-ae 0 --set-exp 20000 --warm 3 --seconds 30" >> "$R/hr/probe.log" 2>&1
      for e in 33000 30000; do
        "${X[@]}" "$P --label ${name}_m$e --out /o/hr/manual$e.json --set-exp $e --warm 3 --seconds 30" >> "$R/hr/probe.log" 2>&1
      done
      echo "manual done (half) $(date -Is)" >> "$R/hr/marks.txt"
    elif [ "$v" = normal ] && [ ! -e "$RUNS/e2c_control_done" ]; then
      "${X[@]}" "$P --label ${name}_m20000 --out /o/hr/manual20000.json --set-ae 0 --set-exp 20000 --warm 3 --seconds 30" >> "$R/hr/probe.log" 2>&1
      touch "$RUNS/e2c_control_done"; echo "manual done (normal control) $(date -Is)" >> "$R/hr/marks.txt"
    fi
    docker cp "$s:/tmp/.ros/Log" "$R/hr/sdklog" > /dev/null 2>&1
    echo "chk $name probe_rc=$prc verdict=$v | $(grep -v '^$' "$R/hr/probe.log" | grep -E '^[^ ]+: colour' | tr '\n' ' ' | cut -c1-900)" ;;
  infup)   # NAME I: the board inference service (car_up's command), setsid, wait for "ready": true (<= 120 s)
    local R=$RUNS/$1/s$2 INF i; mkdir -p "$R"
    : "${YOLO_ENGINE:=/data/fpga/yolo_engine_20261015/yolov8n-seg-640-fp16.engine}" "${NANO_PYLIB:=/home/robot/fpga/p2/pylib}"
    setsid python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
      env PYTHONPATH="$REPO:$NANO_PYLIB" python3 -m host.inference_service --socket "$R/inf.sock" --device cuda \
      --detector yolo --engine "$YOLO_ENGINE" --warmup-frames 2 --warmup-size 640x480 > "$R/inference.log" 2>&1 < /dev/null &
    INF=$!; echo "$INF" > "$R/inf.pid"; echo "infup $(date -Is) $(wall) pid=$INF" > "$R/inf_marks.txt"
    for ((i=0; i<1200; i++)); do
      grep -q '"ready": true' "$R/inference.log" 2>/dev/null && [ -S "$R/inf.sock" ] && break
      kill -0 "$INF" 2>/dev/null || break; sleep 0.1
    done
    if grep -q '"ready": true' "$R/inference.log" && [ -S "$R/inf.sock" ]; then
      echo "ready $(date -Is) $(wall)" >> "$R/inf_marks.txt"; echo "infup $1 s$2 ready pid=$INF"
    else echo "not ready $(date -Is)" >> "$R/inf_marks.txt"; echo "infup $1 s$2 NOT ready: $(tail -1 "$R/inference.log")"; return 4; fi ;;
  infdown)   # NAME I: stop that inference service by pid (INT, TERM after 10 s, KILL after 20 s)
    local R=$RUNS/$1/s$2 INF i; INF=$(cat "$R/inf.pid" 2>/dev/null)
    [ -n "$INF" ] || { echo "infdown $1 s$2: no pid"; return 0; }
    kill -INT "$INF" 2>/dev/null
    for ((i=0; i<200; i++)); do kill -0 "$INF" 2>/dev/null || break; [ $i = 100 ] && kill -TERM "$INF" 2>/dev/null; sleep 0.1; done
    kill -0 "$INF" 2>/dev/null && kill -KILL "$INF" 2>/dev/null
    echo "infdown $(date -Is) $(wall) alive=$(kill -0 "$INF" 2>/dev/null && echo 1 || echo 0)" >> "$R/inf_marks.txt"
    echo "infdown $1 s$2 $(tail -1 "$R/inf_marks.txt")" ;;
  sdk)   # NAME: the SDK log only (a refused start: the stack container may still exist)
    local name=$1 s R; s=$(stack_of "$name"); R=$RUNS/$name; mkdir -p "$R/hr"
    [ -n "$s" ] && docker cp "$s:/tmp/.ros/Log" "$R/hr/sdklog" > /dev/null 2>&1; echo "sdk copy rc=$?" ;;
  down)   # NAME
    bash "$REPO/tools/car_run/car_down.sh" --run "$RUNS/$1" > "$RUNS/$1.down.log" 2>&1 < /dev/null; echo "down rc=$?"
    echo "wde-car: $(docker ps -a --format '{{.Names}}' | grep '^wde-car-' | tr '\n' ' ') pointer: $(readlink "$ST/car.current" 2>/dev/null || echo none)" ;;
  marks)   # NAME: the camera lines of a chain start
    grep -E "camera driver|camera rate|camera_rate|READY|FAIL" "$RUNS/$1/marks.txt" 2>/dev/null; cat "$RUNS/$1/stack_fail.txt" 2>/dev/null ;;
  stopall)
    docker rm -f "$DC" > /dev/null 2>&1
    for f in "$RUNS"/*/s*/inf.pid; do [ -s "$f" ] && kill -0 "$(cat "$f")" 2>/dev/null && kill -TERM "$(cat "$f")" 2>/dev/null; done
    [ -L "$ST/car.current" ] && bash "$REPO/tools/car_run/car_down.sh" < /dev/null
    echo "stopall done; containers: $(docker ps -a --format '{{.Names}}' | grep -E '^(wde-car-|hrdig-)' | tr '\n' ' ')" ;;
  *) echo "nano: unknown $sub"; return 2 ;;
  esac
}

# =============================================================================================== KV260 side (root)
kv_main() {
  local sub=$1; shift
  case "$sub" in
  clean)   rm -rf "$KB/repo" "$KB/runs"; mkdir -p "$KB/runs" "$KB/repo"; chown ubuntu:ubuntu "$KB/runs" "$KB/repo" ;;
  prep)    xmutil unloadapp >/dev/null 2>&1; xmutil loadapp m7_pl_merge_a2 2>&1 | tail -1; sleep 2
           bash "$KB/repo/tools/kv_link/kv_board.sh" check "$A2_TEST" | grep -E 'identity|BUILD_ID|m7_roi|m7_frame' ;;
  start)   local out=$1; shift; rm -rf "$out"
           env PYTHONDONTWRITEBYTECODE=1 FPGA_M7_DMA_PROVIDER=dmaheap FPGA_M7_DMA_HEAP=reserved \
             bash "$KB/repo/tools/kv_link/kv_board.sh" start "$KB/repo" "$out" "$@" ;;
  stop)    [ -s "$1/pid" ] || { echo "no server pid in $1"; return 0; }
           bash "$KB/repo/tools/kv_link/kv_board.sh" stop "$1" | tail -2; chown -R ubuntu:ubuntu "$1" ;;
  restore) xmutil desktop_enable >/dev/null 2>&1; echo "desktop_enable rc=$?"
           xmutil unloadapp >/dev/null 2>&1; xmutil loadapp m7_pl 2>&1 | tail -1; sleep 2
           "$BASE_TEST" --heap reserved info 2>&1 | grep -o -E 'BUILD_ID=0x[0-9a-fA-F]+|SLOT_MAP=0x[0-9a-fA-F]+' | head -2 ;;
  residue) ps -eo pid,args | grep -E 'kv_server.py|m7ar_present' | grep -v grep; echo "residue-check done" ;;
  *) echo "kv: unknown $sub"; return 2 ;;
  esac
}

# =============================================================================================== server side
run_main() {
  HERE=$(cd "$(dirname "$0")" && pwd)
  W=${W:-${FPGA_SRC}}
  R=${R:-${FPGA_SRC}/runs/halfrate-dig-20261017}
  : "${LOCK_POLL_S:=30}" "${NANO_SSH:=jetson}" "${KV_SSH:=kv260}" "${SSH_TRIES:=12}"
  : "${N_DRV:=20}" "${N_AE:=2}" "${N_CHAIN:=6}" "${GAP_S:=10}" "${MODE:=dig}" "${ARM:=}" "${N_INF:=12}"
  KV_SECRET=${FPGA_HOME}/dev1/.secrets/kv260_sudo
  LOCK="bash ${FPGA_HOME}/tools/lock.sh"
  T=$(date +%m%d_%H%M)
  L=$R/board_$T; mkdir -p "$L/ctl" || exit 2
  log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$L/session.log"; }
  nano() { local n rc; for ((n=1; n<=SSH_TRIES; n++)); do ssh "$NANO_SSH" "ARM='$ARM' bash $NB/hr_session.sh nano $*"; rc=$?
    [ $rc -ne 255 ] && return $rc; log "ssh transport to the Nano failed (try $n): nano $1"; sleep 20; done; return 255; }
  ksu() { local n rc; for ((n=1; n<=SSH_TRIES; n++)); do ssh "$KV_SSH" "sudo -S -p '' bash $KB/hr_session.sh kv $*" < "$KV_SECRET"; rc=$?
    [ $rc -ne 255 ] && return $rc; log "ssh transport to the KV failed (try $n): kv $1"; sleep 20; done; return 255; }
  lock_token() { local o; o=$($LOCK acquire "$1" "$2" -u dev1 2>&1) || { echo "lock $1 not acquired: $o" >&2; return 1; }
    echo "$o" | grep -o -E '"token": "[0-9a-f]{32}"' | grep -o -E '[0-9a-f]{32}' | head -1; }
  TOK="$L/tokens" KV_LOADED=0 SRV="" PULLED=0 DEPLOYED=0
  COMMIT=$(git -C "$W" rev-parse HEAD) || exit 2
  [ -z "$(git -C "$W" status --porcelain --untracked-files=no)" ] || { echo "W has uncommitted tracked changes" >&2; exit 2; }
  : > "$TOK"; echo "$$" > "$L/session.pid"
  case "$ARM" in *[!a-z0-9]*) echo "ARM: [a-z0-9] only" >&2; exit 2;; esac
  log "half-rate dig session commit $COMMIT -> $L (MODE=$MODE ARM=$ARM N_DRV=$N_DRV N_AE=$N_AE N_CHAIN=$N_CHAIN N_INF=$N_INF)"
  pull() {
    [ "$PULLED" = 1 ] && return 0
    mkdir -p "$L/nano" "$L/kv"; local n
    for ((n=1; n<=SSH_TRIES; n++)); do
      ssh "$NANO_SSH" "tar czf - -C $NB runs${ARM:+_$ARM}" | tar xzf - -C "$L/nano" && break; log "nano pull incomplete (try $n)"; sleep 20; done
    if [ "$KV_LOADED" = 1 ]; then for ((n=1; n<=SSH_TRIES; n++)); do
      ssh "$KV_SSH" "tar czf - -C $KB runs" | tar xzf - -C "$L/kv" && break; log "kv pull incomplete (try $n)"; sleep 20; done; fi
    PULLED=1; log "pulled to $L/nano"
  }
  cleanup() {
    trap - EXIT INT TERM HUP
    log "cleanup start"
    if [ -s "$TOK" ]; then
      [ "$DEPLOYED" = 1 ] && nano stopall >> "$L/session.log" 2>&1
      [ -n "$SRV" ] && { ksu stop "$SRV" >> "$L/session.log" 2>&1; SRV=""; }
      if [ "$KV_LOADED" = 1 ]; then
        ksu restore > "$L/kv_restore.log" 2>&1; cat "$L/kv_restore.log" >> "$L/session.log"
        grep -q "$BASE_BID" "$L/kv_restore.log" && log "m7_pl $BASE_BID read back" || log "WARNING: m7_pl $BASE_BID not read back after restore"
        ksu residue >> "$L/session.log" 2>&1
      fi
      [ "$DEPLOYED" = 1 ] && { nano env "$NB/runs${ARM:+_$ARM}/env_end_$T.txt" >> "$L/session.log" 2>&1; pull; }
    fi
    while read -r res tok; do [ -n "$tok" ] && $LOCK release "$res" --token "$tok" -u dev1 >> "$L/session.log" 2>&1 && log "released $res"; done < "$TOK"
    $LOCK status > "$L/lock_status_end.txt" 2>&1
    log "cleanup done"; echo CLEANUP_DONE >> "$L/session.log"; touch "$L/CLEANUP_DONE"
  }
  kv_up() {   # KV260 A2 + one kv_server (as E4)
    ssh "$KV_SSH" "mkdir -p $KB" && scp -q "$HERE/hr_session.sh" "$KV_SSH:$KB/hr_session.sh" && ksu clean || return 1
    ssh "$KV_SSH" "tar xf - -C $KB/repo" < "$L/repo.tar" || return 1
    KV_LOADED=1
    ksu prep > "$L/kv_prep.log" 2>&1; cat "$L/kv_prep.log" >> "$L/session.log"
    grep -q "$A2_BID" "$L/kv_prep.log" || { log "A2 BUILD_ID $A2_BID not read"; return 1; }
    SRV=$KB/runs/srv_hr
    ksu start "$SRV" $SARGS > "$L/kv_start.log" 2>&1; cat "$L/kv_start.log" >> "$L/session.log"
    grep -q '"port"' "$L/kv_start.log" || { log "kv_server not ready"; return 1; }
  }
  second_baton() {   # doc section 8.1
    local k m c i
    if echo " $MODE " | grep -q ' e2chain '; then
      kv_up || return 1
      for ((k=1; k<=N_CHAIN; k++)); do
        [ -e "$L/ctl/ABORT" ] && { log "abort"; return 1; }
        log "$(nano up h$k 0)"
        m=$(nano marks h$k); echo "== h$k" >> "$L/chain_marks.txt"; echo "$m" >> "$L/chain_marks.txt"
        log "h$k: $(echo "$m" | grep -E 'camera driver try|READY|FAIL' | tr '\n' ' ' | cut -c1-400)"
        c=$(nano chk h$k); log "$c"
        log "$(nano down h$k | tr '\n' ' ')"
        if echo "$c" | grep -q 'verdict=half'; then log "E2c: half-rate start h$k switched"; break; fi
        sleep "$GAP_S"
      done
      [ "$k" -gt "$N_CHAIN" ] && log "E2c: no half-rate start in $N_CHAIN chain starts"
      [ -n "$SRV" ] && { ksu stop "$SRV" >> "$L/session.log" 2>&1; SRV=""; }
    fi
    if echo " $MODE " | grep -q ' drvinf '; then
      [ -e "$L/ctl/SKIP_INF" ] && { log "drvinf skipped (ctl/SKIP_INF)"; return 0; }
      for ((i=1; i<=N_INF; i++)); do
        [ -e "$L/ctl/ABORT" ] && { log "abort"; return 1; }
        c=$(nano infup e5 "$i"); log "$c"
        if echo "$c" | grep -q ' ready '; then c=$(nano drv e5 "$i" 1); log "$c"; else c=""; log "e5 s$i: inference not ready, driver not started"; fi
        log "$(nano infdown e5 "$i")"
        echo "$c" | grep -q -E 'is not running|docker run failed|probe_rc=13[0-9]' && { log "e5 s$i: driver container did not run; stop"; return 1; }
        sleep "$GAP_S"
      done
    fi
    log "second baton steps done"
  }
  trap cleanup EXIT
  trap 'log "signal"; exit 1' INT TERM HUP

  local WHY="相机半速态查因（只起驱动冷启动 x$N_DRV + 整车链 --chassis none 验证起流检测重启；A2 比特流；约 60 min），dev1 子代理"
  log "waiting for locks kv260 orin-nano camera"
  while true; do
    local st got=1 r t
    st=$($LOCK status 2>/dev/null)
    if ! echo "$st" | grep -q -E '"resource": "(kv260|orin-nano|camera)"'; then
      for r in kv260 orin-nano camera; do
        t=$(lock_token "$r" "$WHY" 2>> "$L/session.log")
        if [ -n "$t" ]; then echo "$r $t" >> "$TOK"; else got=0; break; fi
      done
      [ "$got" = 1 ] && break
      while read -r r t; do $LOCK release "$r" --token "$t" -u dev1 >> "$L/session.log" 2>&1; done < "$TOK"; : > "$TOK"
    fi
    [ -e "$L/ctl/ABORT" ] && { log "abort while waiting"; exit 1; }
    sleep "$LOCK_POLL_S"
  done
  log "acquired kv260 orin-nano camera"; touch "$L/LOCKED"

  git -C "$W" archive --format=tar -o "$L/repo.tar" "$COMMIT" host robot_sim tools tests hw perception pc || exit 1
  echo "$COMMIT" > "$L/COMMIT"
  if [ -n "$ARM" ]; then ssh "$NANO_SSH" "[ ! -e $NB/runs_$ARM ] && [ ! -e $NB/st_$ARM ]" || { log "Nano $NB/runs_$ARM or st_$ARM exists; refused"; exit 1; }; fi
  ssh "$NANO_SSH" "mkdir -p $NB/runs${ARM:+_$ARM} $NB/st${ARM:+_$ARM} && rm -rf $NB/repo && mkdir -p $NB/repo" || exit 1
  scp -q "$HERE/hr_session.sh" "$NANO_SSH:$NB/" || exit 1
  ssh "$NANO_SSH" "tar xf - -C $NB/repo" < "$L/repo.tar" || exit 1
  scp -q "$L/COMMIT" "$NANO_SSH:$NB/repo/COMMIT" || exit 1
  DEPLOYED=1
  log "deployed $COMMIT to $NANO_SSH:$NB/repo"
  local NR=$NB/runs${ARM:+_$ARM}
  nano env "$NR/env_start_$T.txt"; ssh "$NANO_SSH" "cat $NR/env_start_$T.txt" > "$L/nano_env_start.txt"
  grep -q 'MAXN_SUPER' "$L/nano_env_start.txt" || { log "Nano not in MAXN_SUPER; abort"; exit 1; }
  grep -q 'orbbec: Bus' "$L/nano_env_start.txt" || { log "no Orbbec camera on the Nano's USB"; exit 1; }
  grep -q 'car.current: none' "$L/nano_env_start.txt" || { log "a car is already running on the Nano"; exit 1; }
  grep -q 'car.current(default state): none' "$L/nano_env_start.txt" || { log "a car of the default state dir is running"; exit 1; }
  grep -q -E '^(wde-car-|hrdig-)' "$L/nano_env_start.txt" && { log "a wde-car/hrdig container exists on the Nano"; exit 1; }

  [ "$MODE" = dig ] || { second_baton; exit $?; }
  # ---- E1 / E2: driver-only cold starts; E3: AE maximum
  local i
  for ((i=1; i<=N_DRV; i++)); do
    [ -e "$L/ctl/ABORT" ] && { log "abort"; exit 1; }
    log "$(nano drv e1 "$i" 1)"; sleep "$GAP_S"
  done
  for ((i=1; i<=N_AE; i++)); do
    log "$(nano drv e3 "$i" 1 color_ae_max_exposure:=30000)"; sleep "$GAP_S"
  done
  ssh "$NANO_SSH" "grep -h -E 'AE max exposure|ae_max' $NB/runs${ARM:+_$ARM}/e3/s*/driver.log" > "$L/e3_aemax.txt" 2>&1
  log "E3 driver lines: $(head -c 400 "$L/e3_aemax.txt" | tr '\n' ' ')"

  # ---- E4: the full chain with the A check
  [ -e "$L/ctl/SKIP_CHAIN" ] && { log "chain skipped (ctl/SKIP_CHAIN)"; exit 0; }
  ssh "$KV_SSH" "mkdir -p $KB" && scp -q "$HERE/hr_session.sh" "$KV_SSH:$KB/hr_session.sh" && ksu clean || exit 1
  ssh "$KV_SSH" "tar xf - -C $KB/repo" < "$L/repo.tar" || exit 1
  KV_LOADED=1
  ksu prep > "$L/kv_prep.log" 2>&1; cat "$L/kv_prep.log" >> "$L/session.log"
  grep -q "$A2_BID" "$L/kv_prep.log" || { log "A2 BUILD_ID $A2_BID not read"; exit 1; }
  SRV=$KB/runs/srv_hr
  ksu start "$SRV" $SARGS > "$L/kv_start.log" 2>&1; cat "$L/kv_start.log" >> "$L/session.log"
  grep -q '"port"' "$L/kv_start.log" || { log "kv_server not ready"; exit 1; }
  local k m
  for ((k=1; k<=N_CHAIN; k++)); do
    [ -e "$L/ctl/ABORT" ] && { log "abort"; exit 1; }
    log "$(nano up c$k)"
    m=$(nano marks c$k); echo "== c$k" >> "$L/chain_marks.txt"; echo "$m" >> "$L/chain_marks.txt"
    log "c$k: $(echo "$m" | grep -E 'camera driver try|READY|FAIL' | tr '\n' ' ' | cut -c1-400)"
    if echo "$m" | grep -q '^.* READY$'; then log "$(nano measure c$k | tail -1)"; else nano sdk c$k >> "$L/session.log" 2>&1; fi
    log "$(nano down c$k | tr '\n' ' ')"
    if echo "$m" | grep -q 'probe_rc=3' && echo "$m" | grep -q ' READY$'; then log "E4: a half-rate try restarted and READY in c$k"; break; fi
    sleep "$GAP_S"
  done
  log "session steps done"
}

case "${1:-}" in
  nano) shift; nano_main "$@";;
  kv) shift; kv_main "$@";;
  run) shift; run_main "$@";;
  *) sed -n '2,13p' "$0"; exit 2;;
esac
