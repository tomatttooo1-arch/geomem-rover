#!/bin/bash
# shm8 guard board check (2026-10-17, docs/design/HEARTBEAT_GAP_DIG_20261016.md section 11). Derived from
# tools/hb_gap_dig/t2/t2_session.sh (same lock parsing, deploy + sha table, KV A2 prep/start/stop/restore, EXIT trap),
# without the samplers. Car still, camera on the floor, NO chassis (--chassis none): nothing can move.
# One session, three locks held once, the arms in order (ARMS, default "P N R"), each a short car_up:
#   P : LIVE_DRIVER_DDS_PROFILE unset (the board default, 8 MB)    -> the guard must pass, READY, WIN_S window
#   N : LIVE_DRIVER_DDS_PROFILE=tools/nano_ros2/fastdds_shm_localhost.xml (32 MB; falls back silently, 10-16)
#       -> the guard must refuse the start (stack_fail, car_up rc 7) and car_up must roll back clean
#   R : unset (8 MB) + an induced failed first start: the try-1 depth probe is stopped (TERM, by pid) as soon as it
#       runs, and once try 1's driver is gone an UNOWNED placeholder segment (fastrtps_00000000deadbeef + its _el,
#       created as the container's user) fills the stack container's /dev/shm to WARN_FREE_KB free -> the pre-retry
#       fastdds shm clean must delete it and try 2's driver node must get its 8 MB segment (guard passes, READY)
#
#   bash shm8g_session.sh run [--dry-run]    (server)
#   bash shm8g_session.sh nano <sub> ...      (Orin Nano, robot)
#   bash shm8g_session.sh kv <sub> ...        (KV260, root via sudo -S)
# Never pkill -f: processes by pid, containers by name, locks by token.
set -u
NB=/data/fpga/shm8g
KB=/home/ubuntu/fpga/shm8g
A2_TEST=/home/ubuntu/fpga/pl-merge-a2-0da5041/pkg/m7pl_test
BASE_TEST=/home/ubuntu/fpga/package-100mhz/m7pl_test
A2_BID=0x00da5041; BASE_BID=0x0117bc0a
KV_PORT=47800
SARGS="--core pl --device devmem --wait irq --roi-core pl --port $KV_PORT"
PLACE=fastrtps_00000000deadbeef
: "${LEAVE_FREE_KB:=4096}"
wall() { date +%s.%N; }
lock_token() { local o; o=$($LOCK acquire "$1" "$2" -u dev1 2>&1) || { echo "lock $1 not acquired: $o" >&2; return 1; }
  echo "$o" | grep -o -E '"token": "[0-9a-f]{32}"' | grep -o -E '[0-9a-f]{32}' | head -1; }

# =============================================================================================== Nano side
nano_main() {
  local sub=$1; shift
  local RUNS=$NB/runs ST=$NB/st REPO=$NB/repo
  export CAR_STATE_DIR=$ST CAR_RUNS_DIR=$RUNS
  stack_of() { python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['containers']['stack'])" "$RUNS/$1/car.json" 2>/dev/null; }
  case "$sub" in
  env)
    { date -Is; nvpmodel -q 2>&1 | head -1; uptime; free -m | head -2
      echo "listen 47431: $(ss -ltn 2>/dev/null | grep -c ':47431 ')"
      echo "orbbec: $(lsusb -d 2bc5: 2>/dev/null | head -1)"
      echo "car.current: $(readlink $ST/car.current 2>/dev/null || echo none)"
      echo "containers: $(docker ps -a --format '{{.Names}}' | tr '\n' ' ')"
      echo "image fastdds: $(docker run --rm --entrypoint bash fpga-ros:humble-nav-orbbec-scipy-20261017 -c 'ls /opt/ros/humble/bin/fastdds /opt/ros/humble/tools/fastdds/shm/clean.py' 2>&1 | tr '\n' ' ')"
    } > "$1" 2>&1 ;;
  sha) (cd "$REPO" && find . -type f ! -name COMMIT -print0 | LC_ALL=C sort -z | xargs -0 sha256sum) ;;
  up)    # NAME ARM_ENV_FILE
    local name=$1 envf=$2; mkdir -p "$RUNS" "$ST"
    source "$envf"
    { echo "LIVE_DRIVER_DDS_PROFILE=${LIVE_DRIVER_DDS_PROFILE-<unset>}"; } > "$RUNS/$name.arm_env"
    wall > "$RUNS/$name.up_t0"
    CAR_BOARD_DRAFT=1 CAR_HUB_BIND=127.0.0.1 \
      bash "$REPO/tools/car_run/car_up.sh" live --pose static --chassis none --nav costmap --run-dir "$RUNS/$name" \
      > "$RUNS/$name.up.log" 2>&1 < /dev/null
    local rc=$?; echo $rc > "$RUNS/$name.up_rc"; wall > "$RUNS/$name.up_t1"
    echo "up $name rc=$rc"; return $rc ;;
  check)   # NAME
    local name=$1 stack; stack=$(stack_of "$name")
    echo "t=$(date +%T) stack=$(docker inspect -f '{{.State.Running}}' "$stack" 2>/dev/null) shm=$(docker exec "$stack" df -k /dev/shm 2>/dev/null | tail -1 | tr -s ' ')" ;;
  shmstat)   # NAME TAG: the stack container's /dev/shm and each ROS process's held segments (read-only)
    local name=$1 tag=$2 stack pp; stack=$(stack_of "$name")
    { echo "== $(date -Is) stack=$stack"; docker exec "$stack" sh -c 'df -k /dev/shm; ls -la --time-style=full-iso /dev/shm'
      for pp in $(docker top "$stack" -eo pid,args 2>/dev/null | awk 'NR > 1 && /component_container|ros2 launch|ros_geo_producer|ros_runtime/ {print $1}'); do
        echo "pid $pp $(tr '\0' ' ' < /proc/$pp/cmdline | cut -c1-90) holds: $(ls -l /proc/$pp/fd 2>/dev/null | grep -o 'fastrtps_[0-9a-f]*_el' | sort -u | tr '\n' ' ')"
      done; } > "$RUNS/$name/shmstat_$tag.txt" 2>&1
    head -3 "$RUNS/$name/shmstat_$tag.txt" | tail -1 ;;
  inject)   # NAME: the R arm's induced failed first start (detached; log $RUNS/NAME.inject.log)
    local name=$1 stack="" pid="" i u free n
    exec > "$RUNS/$name.inject.log" 2>&1
    echo "inject start $(date -Is)"
    for ((i=0; i<1200; i++)); do stack=$(stack_of "$name"); [ -n "$stack" ] && docker inspect "$stack" > /dev/null 2>&1 && break; sleep 0.1; done
    echo "stack=$stack ($(date +%T.%N))"
    # 1. the try-1 depth probe: stopped as soon as it runs (before it can count 30 depth images)
    for ((i=0; i<1200; i++)); do
      pid=$(docker top "$stack" -eo pid,args 2>/dev/null | awk 'NR > 1 && /depth_probe.py/ {print $1; exit}')
      [ -n "$pid" ] && break; sleep 0.1
    done
    [ -n "$pid" ] || { echo "no depth probe seen in 120 s: nothing done"; exit 1; }
    kill -TERM "$pid"; echo "probe pid $pid TERM rc=$? ($(date +%T.%N)) driver_probe_1: $(ls $RUNS/$name/driver_probe_1.log 2>&1)"
    # 2. try 1 judged and its driver gone
    for ((i=0; i<600; i++)); do grep -q 'camera driver try=1 ' "$RUNS/$name/marks.txt" 2>/dev/null && break; sleep 0.1; done
    grep 'camera driver try=1 ' "$RUNS/$name/marks.txt"
    for ((i=0; i<300; i++)); do
      n=$(docker top "$stack" -eo pid,args 2>/dev/null | grep -c -E 'component_container|ros2 launch orbbec')
      [ "$n" = 0 ] && break; sleep 0.1
    done
    echo "try-1 driver processes left: $n ($(date +%T.%N))"
    # 3. the placeholder: unowned (nobody holds its _el), the container's user, /dev/shm left with LEAVE_FREE_KB
    u=$(docker inspect -f '{{.Config.User}}' "$stack")
    free=$(docker exec "$stack" df -k /dev/shm | awk 'NR == 2 {print $4}')
    n=$((free - LEAVE_FREE_KB))
    docker exec --user "$u" "$stack" sh -c "dd if=/dev/zero of=/dev/shm/$PLACE bs=1024 count=$n 2>&1 | tail -1; : > /dev/shm/${PLACE}_el; df -k /dev/shm; ls -la /dev/shm"
    echo "placeholder ${n} KB as $u (free before $free KB) ($(date +%T.%N))"
    grep 'camera driver before try=2' "$RUNS/$name/marks.txt" && echo "WARNING: the pre-retry clean already ran before the placeholder"
    echo "inject done $(date -Is)" ;;
  inject_start)   # NAME
    setsid nohup bash "$NB/shm8g_session.sh" nano inject "$1" > /dev/null 2>&1 < /dev/null &
    echo "injector pid $!" ;;
  postfail)   # NAME: what a refused start left
    local name=$1
    echo "containers: $(docker ps -a --format '{{.Names}}' | grep '^wde-car-' | tr '\n' ' ')"
    echo "pointer: $(readlink $ST/car.current 2>/dev/null || echo none)"
    echo "stack_fail: $(cat $RUNS/$name/stack_fail.txt 2>/dev/null)"
    echo "down.json: $(head -c 500 $RUNS/$name/down.json 2>/dev/null)" ;;
  down)   # NAME
    local name=$1
    bash "$REPO/tools/car_run/car_down.sh" --run "$RUNS/$name" > "$RUNS/$name.down.log" 2>&1 < /dev/null; echo "down rc=$?"
    echo "containers: $(docker ps -a --format '{{.Names}}' | grep '^wde-car-' | tr '\n' ' ')"
    echo "pointer: $(readlink $ST/car.current 2>/dev/null)" ;;
  stopall)
    [ -L "$ST/car.current" ] && bash "$REPO/tools/car_run/car_down.sh" < /dev/null
    echo "stopall done" ;;
  *) echo "nano: unknown $sub"; return 2 ;;
  esac
}

# =============================================================================================== KV260 side (root)
kv_main() {
  local sub=$1; shift
  case "$sub" in
  clean)   rm -rf "$KB/repo" "$KB/runs"; mkdir -p "$KB/runs"; chown ubuntu:ubuntu "$KB/runs"; ls -la --time-style=full-iso /dev/shm ;;
  sha)     (cd "$KB/repo" && find . -type f -print0 | LC_ALL=C sort -z | xargs -0 sha256sum) ;;
  prep)
    uptime; xmutil unloadapp >/dev/null 2>&1; xmutil loadapp m7_pl_merge_a2 2>&1 | tail -1; sleep 2
    bash "$KB/repo/tools/kv_link/kv_board.sh" check "$A2_TEST" | grep -E 'identity|BUILD_ID|m7_roi|m7_frame' ;;
  start)   local out=$1; shift; rm -rf "$out"
    env PYTHONDONTWRITEBYTECODE=1 FPGA_M7_DMA_PROVIDER=dmaheap FPGA_M7_DMA_HEAP=reserved \
      bash "$KB/repo/tools/kv_link/kv_board.sh" start "$KB/repo" "$out" "$@" ;;
  stop)    bash "$KB/repo/tools/kv_link/kv_board.sh" stop "$1" | tail -2; chown -R ubuntu:ubuntu "$1" ;;
  restore)
    xmutil desktop_enable >/dev/null 2>&1; echo "desktop_enable rc=$?"
    xmutil unloadapp >/dev/null 2>&1; xmutil loadapp m7_pl 2>&1 | tail -1; sleep 2
    "$BASE_TEST" --heap reserved info 2>&1 | grep -o -E 'BUILD_ID=0x[0-9a-fA-F]+|SLOT_MAP=0x[0-9a-fA-F]+' | head -2 ;;
  residue) ps -eo pid,args | grep -E 'kv_server.py|m7ar_present' | grep -v grep; ls -la --time-style=full-iso /dev/shm; echo "residue-check done" ;;
  *) echo "kv: unknown $sub"; return 2 ;;
  esac
}

# =============================================================================================== server side
run_main() {
  DRY=0; [ "${1:-}" = --dry-run ] && DRY=1
  W=${W:-${FPGA_SRC}}
  R=${R:-${FPGA_SRC}/runs/shm8g-20261017}
  : "${DEPLOY:=HEAD}" "${WIN_S:=180}" "${ARMS:=P N R}" "${LOCK_POLL_S:=60}" "${LOCK_WAIT_MAX_S:=7200}" "${ARM_GAP_S:=20}"
  : "${NANO_SSH:=jetson}" "${KV_SSH:=kv260}" "${SSH_TRIES:=12}"
  KV_SECRET=${FPGA_HOME}/dev1/.secrets/kv260_sudo
  LOCK="bash ${FPGA_HOME}/tools/lock.sh"
  T=$(date +%m%d_%H%M)
  if [ "$DRY" = 1 ]; then L=$R/dryrun_$T; else L=$R/log_$T; fi
  mkdir -p "$L" || exit 2
  log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$L/session.log"; }
  X() { if [ "$DRY" = 1 ]; then echo "[dry] $*" >> "$L/session.log"; echo "[dry] $*"; return 0; fi; "$@"; }
  nano() { local n rc; for ((n=1; n<=SSH_TRIES; n++)); do X ssh "$NANO_SSH" "bash $NB/shm8g_session.sh nano $*"; rc=$?
    [ $rc -ne 255 ] && return $rc; log "ssh to Nano failed (try $n): nano $1"; sleep 20; done; return 255; }
  ksu() { local n rc; for ((n=1; n<=SSH_TRIES; n++)); do X ssh "$KV_SSH" "sudo -S -p '' bash $KB/shm8g_session.sh kv $*" < "$KV_SECRET"; rc=$?
    [ $rc -ne 255 ] && return $rc; log "ssh to KV failed (try $n): kv $1"; sleep 20; done; return 255; }
  pause() { [ "$DRY" = 1 ] && { echo "[dry] sleep $1"; return 0; }; sleep "$1"; }
  CLEAN_FAIL=0 TOK="$L/tokens" KV_LOADED=0 SRV="" PULLED=0 UPNAME=""
  COMMIT=$(git -C "$W" rev-parse "$DEPLOY^{commit}") || exit 2
  : > "$TOK"
  { echo "deploy_commit=$COMMIT"; echo "arms=$ARMS"; echo "win_s=$WIN_S"; echo "leave_free_kb=$LEAVE_FREE_KB"; } > "$L/config.txt"
  log "shm8g session (dry=$DRY) commit $COMMIT arms '$ARMS' -> $L"
  pull() {
    [ "$PULLED" = 1 ] && return 0
    mkdir -p "$L/nano"
    if [ "$DRY" = 1 ]; then X "pull"; else
      local n; for ((n=1; n<=SSH_TRIES; n++)); do
        ssh "$NANO_SSH" "tar czf - -C $NB runs" | tar xzf - -C "$L/nano" && break; log "nano pull incomplete (try $n)"; sleep 20; done
    fi
    PULLED=1; log "pulled to $L/nano"
  }
  cleanup() {
    trap - EXIT INT TERM HUP
    log "cleanup start"
    if [ -s "$TOK" ] || [ "$DRY" = 1 ]; then
      [ -n "$UPNAME" ] && { nano down "$UPNAME" >> "$L/session.log" 2>&1; [ $? = 255 ] && CLEAN_FAIL=1; UPNAME=""; }
      nano stopall >> "$L/session.log" 2>&1; [ $? = 255 ] && CLEAN_FAIL=1
      [ -n "$SRV" ] && { ksu stop "$SRV" >> "$L/session.log" 2>&1; SRV=""; }
      if [ "$KV_LOADED" = 1 ]; then
        ksu restore > "$L/kv_restore.log" 2>&1; [ $? = 255 ] && CLEAN_FAIL=1; cat "$L/kv_restore.log" >> "$L/session.log"
        grep -q "$BASE_BID" "$L/kv_restore.log" || [ "$DRY" = 1 ] || { log "WARNING: m7_pl $BASE_BID not read back after restore"; CLEAN_FAIL=1; }
        ksu residue > "$L/kv_residue.txt" 2>&1; cat "$L/kv_residue.txt" >> "$L/session.log"
      fi
      nano env "$NB/runs/env_end_$T.txt" >> "$L/session.log" 2>&1
      [ "$DRY" = 1 ] || ssh "$NANO_SSH" "cat $NB/runs/env_end_$T.txt" > "$L/nano_env_end.txt" 2>&1
      pull
    fi
    if [ "$CLEAN_FAIL" = 1 ]; then log "cleanup incomplete: locks KEPT (tokens in $TOK); finish by hand"; cp "$TOK" "$R/tokens_held_$T.txt"; : > "$TOK"; fi
    while read -r res tok; do [ -n "$tok" ] && X $LOCK release "$res" --token "$tok" -u dev1 >> "$L/session.log" 2>&1 && log "released $res"; done < "$TOK"
    log "cleanup done"; echo CLEANUP_DONE >> "$L/session.log"
  }
  trap cleanup EXIT
  trap 'log "signal"; exit 1' INT TERM HUP

  # --- locks: all three or none, wait (bounded), never steal
  local WHY="shm8 守卫板上验证（车静止、--chassis none 不接底盘、3 段短趟约 25 min；A2 比特流），dev1 子代理"
  if [ "$DRY" = 1 ]; then X $LOCK acquire kv260 "$WHY" -u dev1
  else
    local waited=0
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
      [ "$waited" -ge "$LOCK_WAIT_MAX_S" ] && { log "locks busy for $waited s: giving up"; echo "$st" > "$L/lock_status_busy.txt"; exit 3; }
      log "locks busy, waiting"; sleep "$LOCK_POLL_S"; waited=$((waited + LOCK_POLL_S))
    done
    log "acquired kv260 orin-nano camera"
  fi

  # --- deploy + sha table
  X git -C "$W" archive --format=tar -o "$L/repo.tar" "$COMMIT" host robot_sim tools tests hw perception pc || exit 1
  echo "$COMMIT" > "$L/COMMIT"
  mkdir -p "$L/ref" && tar xf "$L/repo.tar" -C "$L/ref" && (cd "$L/ref" && find . -type f -print0 | LC_ALL=C sort -z | xargs -0 sha256sum) > "$L/sha_host.txt"
  local ME="$W/tools/hb_gap_dig/shm8g/shm8g_session.sh"
  X ssh "$NANO_SSH" "mkdir -p $NB/old && { [ ! -d $NB/runs ] || mv $NB/runs $NB/old/runs_\$(date +%s); } && mkdir -p $NB/runs $NB/st && rm -rf $NB/repo && mkdir -p $NB/repo" || exit 1
  X scp -q "$ME" "$NANO_SSH:$NB/" || exit 1
  if [ "$DRY" = 1 ]; then X "ssh nano tar x"; else ssh "$NANO_SSH" "tar xf - -C $NB/repo" < "$L/repo.tar" || exit 1; fi
  X scp -q "$L/COMMIT" "$NANO_SSH:$NB/repo/COMMIT" || exit 1
  X ssh "$KV_SSH" "mkdir -p $KB" && X scp -q "$ME" "$KV_SSH:$KB/shm8g_session.sh" && ksu clean > "$L/kv_shm_before.txt" 2>&1 || exit 1
  if [ "$DRY" = 1 ]; then X "ssh kv tar x"; else ssh "$KV_SSH" "mkdir -p $KB/repo && tar xf - -C $KB/repo" < "$L/repo.tar" || exit 1; fi
  if [ "$DRY" != 1 ]; then
    nano sha > "$L/sha_nano.txt"; ksu sha > "$L/sha_kv.txt"
    if cmp -s "$L/sha_host.txt" "$L/sha_nano.txt" && cmp -s "$L/sha_host.txt" "$L/sha_kv.txt"; then
      log "sha table: $(wc -l < "$L/sha_host.txt") files identical on Nano and KV"
    else log "sha table MISMATCH (diff in $L/sha_*.txt)"; exit 1; fi
  fi
  log "deployed $COMMIT"
  # --- Nano checks
  nano env "$NB/runs/env_start_$T.txt"
  [ "$DRY" = 1 ] || ssh "$NANO_SSH" "cat $NB/runs/env_start_$T.txt" > "$L/nano_env_start.txt"
  if [ "$DRY" != 1 ]; then
    cat "$L/nano_env_start.txt" >> "$L/session.log"
    grep -q 'listen 47431: 0' "$L/nano_env_start.txt" || { log "port 47431 busy on the Nano"; exit 1; }
    grep -q 'orbbec: Bus' "$L/nano_env_start.txt" || { log "no Orbbec camera on the Nano's USB"; exit 1; }
    grep -q 'car.current: none' "$L/nano_env_start.txt" || { log "a car is already running on the Nano"; exit 1; }
    grep -q 'image fastdds: /opt/ros/humble/bin/fastdds /opt/ros/humble/tools/fastdds/shm/clean.py' "$L/nano_env_start.txt" \
      || { log "board image: no fastdds shm clean"; exit 1; }
  fi
  # --- KV260: A2 + kv_server
  KV_LOADED=1
  ksu prep > "$L/kv_prep.log" 2>&1; cat "$L/kv_prep.log" >> "$L/session.log"
  [ "$DRY" = 1 ] || grep -q "$A2_BID" "$L/kv_prep.log" || { log "A2 BUILD_ID $A2_BID not read"; exit 1; }
  SRV=$KB/runs/srv_shm8g_$T
  ksu start "$SRV" $SARGS > "$L/kv_start.log" 2>&1; cat "$L/kv_start.log" >> "$L/session.log"
  [ "$DRY" = 1 ] || grep -q '"port"' "$L/kv_start.log" || { log "kv_server not ready"; exit 1; }
  pause 2

  local arm name envf rc el t_ready first=1
  for arm in $ARMS; do
    [ "$first" = 1 ] || pause "$ARM_GAP_S"; first=0
    name=shm8g_${arm}_$T envf=$L/up_env_$arm.sh
    case "$arm" in
      P|R) echo "unset LIVE_DRIVER_DDS_PROFILE" > "$envf" ;;
      N) echo "export LIVE_DRIVER_DDS_PROFILE=tools/nano_ros2/fastdds_shm_localhost.xml" > "$envf" ;;
      *) log "unknown arm $arm"; exit 2 ;;
    esac
    X scp -q "$envf" "$NANO_SSH:$NB/up_env_$arm.sh" || exit 1
    [ "$arm" = R ] && { nano inject_start "$name" >> "$L/session.log" 2>&1; log "R: injector started"; }
    log "arm $arm: car_up live --pose static --chassis none --nav costmap ($(cat "$envf"))"
    UPNAME=$name
    nano up "$name" "$NB/up_env_$arm.sh" >> "$L/session.log" 2>&1; rc=$?
    log "arm $arm: car_up rc=$rc"
    if [ "$DRY" != 1 ]; then
      ssh "$NANO_SSH" "grep -E 'camera driver|driver_shm|READY|FAIL' $NB/runs/$name/marks.txt; cat $NB/runs/$name/driver_shm_check.log; tail -3 $NB/runs/$name.up.log" > "$L/marks_$arm.txt" 2>&1
      sed 's/^/    /' "$L/marks_$arm.txt" | tee -a "$L/session.log" > /dev/null
    fi
    if [ "$rc" != 0 ]; then
      UPNAME=""
      nano postfail "$name" > "$L/postfail_$arm.txt" 2>&1; cat "$L/postfail_$arm.txt" >> "$L/session.log"
      [ "$arm" = N ] || log "arm $arm: car_up FAILED (not expected)"
      continue
    fi
    [ "$arm" = N ] && log "arm N: car_up SUCCEEDED (not expected: the guard should have refused)"
    nano shmstat "$name" early >> "$L/session.log" 2>&1
    t_ready=$(date +%s)
    while true; do
      el=$(( $(date +%s) - t_ready )); [ "$el" -ge "$WIN_S" ] && break
      X nano check "$name" >> "$L/checks_$arm.txt" 2>&1
      tail -1 "$L/checks_$arm.txt" | grep -q 'stack=true' || [ "$DRY" = 1 ] || { log "arm $arm: stack not running at ${el}s"; break; }
      [ "$DRY" = 1 ] && break
      pause 60
    done
    nano shmstat "$name" late >> "$L/session.log" 2>&1
    nano down "$name" >> "$L/session.log" 2>&1; UPNAME=""
    log "arm $arm done"
  done
  log "all arms done"
}

case "${1:-}" in
  nano) shift; nano_main "$@";;
  kv) shift; kv_main "$@";;
  run) shift; run_main "$@";;
  *) sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'; exit 2;;
esac
