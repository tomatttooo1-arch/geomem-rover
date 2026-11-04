#!/bin/bash
# T1 (2026-10-16): heartbeat-gap on-board repro + side sampling, docs/design/HEARTBEAT_GAP_DIG_20261016.md §6.
# Derived from tools/car_run/bench_line4.sh (same lock parsing, KV prep/start/stop/restore, Nano up/down, EXIT trap).
# Car still, camera on free floor, NO chassis (--chassis none): nothing can move.
#
#   bash t1_session.sh run [--dry-run]      (server)
#   bash t1_session.sh nano <sub> ...        (Orin Nano, robot)
#   bash t1_session.sh kv <sub> ...          (KV260, root via sudo -S)
#
# run: three locks (wait, never steal) -> deploy commit DEPLOY (git archive) to both boards + sha256 table check
#   -> Nano checks -> KV A2 (0x00da5041) + kv_server (no --max-idle-s) -> sampler + tegrastats (Nano host)
#   -> car_up live --pose static --chassis none --nav costmap (LIVE_LAUNCH_ARGS + the driver's per-frame record)
#   -> drv_frames.csv must grow -> CLEAN_S clean window (topic info once ~3 min) -> SIDE_S with hb_sidecar.py
#   -> topic info again -> car_down -> stop sampler/tegrastats -> kv stop/restore m7_pl (0x0117bc0a) -> pull -> locks.
# Never pkill -f: processes by pid, containers by name, locks by token.
set -u
NB=/data/fpga/t1
KB=/home/ubuntu/fpga/t1
A2_TEST=/home/ubuntu/fpga/pl-merge-a2-0da5041/pkg/m7pl_test
BASE_TEST=/home/ubuntu/fpga/package-100mhz/m7pl_test
A2_BID=0x00da5041; BASE_BID=0x0117bc0a
KV_PORT=47800
SARGS="--core pl --device devmem --wait irq --roi-core pl --port $KV_PORT"
DEF_LAUNCH="color_width:=640 color_height:=480 color_fps:=30 color_format:=RGB depth_width:=640 depth_height:=400 depth_fps:=30 depth_format:=Y16 enable_ir:=false enable_point_cloud:=false depth_registration:=true align_mode:=HW publish_tf:=false"
raw_ns() { python3 -c 'import time; print(time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW))'; }
wall() { date +%s.%N; }
lock_token() { local o; o=$($LOCK acquire "$1" "$2" -u dev1 2>&1) || { echo "lock $1 not acquired: $o" >&2; return 1; }
  echo "$o" | grep -o -E '"token": "[0-9a-f]{32}"' | grep -o -E '[0-9a-f]{32}' | head -1; }

# =============================================================================================== Nano side
nano_main() {
  local sub=$1; shift
  local RUNS=$NB/runs ST=$NB/st REPO=$NB/repo
  export CAR_STATE_DIR=$ST CAR_RUNS_DIR=$RUNS
  case "$sub" in
  env)
    { date -Is; nvpmodel -q 2>&1; uptime; cat /proc/loadavg; free -m | head -2
      echo "listen 47431: $(ss -ltn 2>/dev/null | grep -c ':47431 ')"
      echo "orbbec: $(lsusb -d 2bc5: 2>/dev/null | head -1)"
      echo "car.current: $(readlink $ST/car.current 2>/dev/null || echo none)"
      echo "host_shm:"; ls -la --time-style=full-iso /dev/shm
      docker ps -a --format '{{.Names}} {{.Image}} {{.Status}}'; top -bn1 | head -15; } > "$1" 2>&1 ;;
  sha)   # the deployed tree's sha256 table (relative paths, sorted)
    (cd "$REPO" && find . -type f ! -name COMMIT -print0 | LC_ALL=C sort -z | xargs -0 sha256sum) ;;
  sample_start)   # OUT: sampler + tegrastats, detached, pids recorded
    local out=$1; mkdir -p "$out"
    setsid nohup python3 "$NB/t1_sampler.py" --out "$out" --car-json "$out/../car_pointer.json" > "$out/sampler.log" 2>&1 < /dev/null &
    echo $! > "$out/sampler.pid"
    setsid nohup tegrastats --interval 1000 --logfile "$out/tegrastats.log" > "$out/tegrastats.out" 2>&1 < /dev/null &
    echo $! > "$out/tegrastats.pid"
    sleep 2; for p in sampler tegrastats; do echo "$p pid=$(cat $out/$p.pid) alive=$(kill -0 $(cat $out/$p.pid) 2>/dev/null && echo 1 || echo 0)"; done ;;
  sample_stop)   # OUT
    local out=$1 p pid
    for p in sampler tegrastats; do pid=$(cat "$out/$p.pid" 2>/dev/null) || continue
      kill -TERM "$pid" 2>/dev/null; sleep 3; kill -0 "$pid" 2>/dev/null && { kill -KILL "$pid"; echo "$p killed"; }
      echo "$p $pid stopped"; done ;;
  up)    # NAME
    local name=$1; mkdir -p "$RUNS" "$ST"
    source "$NB/up_env.sh"
    wall > "$RUNS/$name.up_t0"
    CAR_BOARD_DRAFT=1 CAR_HUB_BIND=127.0.0.1 \
      bash "$REPO/tools/car_run/car_up.sh" live --pose static --chassis none --nav costmap --run-dir "$RUNS/$name" \
      > "$RUNS/$name.up.log" 2>&1 < /dev/null
    local rc=$?; echo $rc > "$RUNS/$name.up_rc"; wall > "$RUNS/$name.up_t1"
    # the sampler reads the stack name from here (written after READY)
    [ -f "$RUNS/$name/car.json" ] && cp "$RUNS/$name/car.json" "$RUNS/car_pointer.json"
    echo "up $name rc=$rc"; return $rc ;;
  check)   # NAME: one heartbeat line
    local name=$1; local stack r=$RUNS/$name s
    stack=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['containers']['stack'])" "$r/car.json" 2>/dev/null)
    s="t=$(date +%T) stack=$(docker inspect -f '{{.State.Running}}' "$stack" 2>/dev/null) frames=$(wc -l < $r/frames.jsonl 2>/dev/null)"
    s="$s drv=$(wc -l < $r/drv_frames.csv 2>/dev/null) gaps=$(grep -c '"input_gap"' $r/frames.jsonl 2>/dev/null)"
    s="$s fail=$(head -c 200 $r/stack_fail.txt 2>/dev/null | tr '\n' ' ')"
    s="$s sampler=$(kill -0 $(cat $RUNS/t1side/sampler.pid 2>/dev/null) 2>/dev/null && echo 1 || echo 0)"
    s="$s teg=$(kill -0 $(cat $RUNS/t1side/tegrastats.pid 2>/dev/null) 2>/dev/null && echo 1 || echo 0)"
    [ -f "$r/t1/sidecar.pid" ] && s="$s sidecar=$(docker exec "$stack" sh -c "kill -0 \$(cat /o/t1/sidecar.pid) 2>/dev/null && echo 1 || echo 0") side_lines=$(wc -l < $r/t1/side_frames.csv) over=$(wc -l < $r/t1/side_over.jsonl)"
    s="$s temp_tj=$(cat /sys/class/thermal/thermal_zone8/temp) pwm=$(cat /sys/class/hwmon/hwmon0/pwm1)"
    echo "$s" ;;
  topicinfo)   # NAME TAG
    local name=$1 tag=$2; local stack r=$RUNS/$name
    stack=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['containers']['stack'])" "$r/car.json")
    docker exec "$stack" bash -c 'source /opt/ros/humble/setup.bash; [ -f /opt/orbbec_ws/install/setup.bash ] && source /opt/orbbec_ws/install/setup.bash
      for t in /camera/depth/image_raw /camera/color/image_raw /camera/depth/camera_info; do echo "== ros2 topic info -v $t ($(date +%s.%N))"
        timeout 40 ros2 topic info -v "$t" --no-daemon --spin-time 5 2>&1 || timeout 40 ros2 topic info -v "$t" 2>&1; done
      echo "== /dev/shm"; df -k /dev/shm; ls -la --time-style=full-iso /dev/shm' > "$r/topicinfo_$tag.txt" 2>&1
    echo "topicinfo $tag rc=$?"; grep -E 'Reliability|Node name|Endpoint type|History|Depth' "$r/topicinfo_$tag.txt" | head -40 ;;
  side_start)   # NAME
    local name=$1; local stack r=$RUNS/$name
    stack=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['containers']['stack'])" "$r/car.json")
    mkdir -p "$r/t1"; cp "$NB/hb_sidecar.py" "$r/t1/hb_sidecar.py"
    docker exec -d "$stack" bash -c 'source /opt/ros/humble/setup.bash; exec python3 /o/t1/hb_sidecar.py --out /o/t1 > /o/t1/sidecar.log 2>&1'
    sleep 15; cat "$r/t1/side_heartbeat.jsonl" 2>/dev/null | tail -2; wc -l < "$r/t1/side_frames.csv" 2>/dev/null ;;
  side_stop)   # NAME
    local name=$1; local stack r=$RUNS/$name
    stack=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['containers']['stack'])" "$r/car.json")
    docker exec "$stack" sh -c 'kill -TERM $(cat /o/t1/sidecar.pid)'; echo "side_stop rc=$?"; sleep 3
    tail -1 "$r/t1/side_heartbeat.jsonl" ;;
  down)   # NAME
    local name=$1
    bash "$REPO/tools/car_run/car_down.sh" --run "$RUNS/$name" > "$RUNS/$name.down.log" 2>&1 < /dev/null; echo "down rc=$?"
    echo "containers: $(docker ps -a --format '{{.Names}}' | grep '^wde-car-' | tr '\n' ' ')"
    echo "pointer: $(readlink $ST/car.current 2>/dev/null)"
    cat "$RUNS/$name/down.json" 2>/dev/null | head -c 600; echo ;;
  stopall)
    if [ -L "$ST/car.current" ]; then bash "$REPO/tools/car_run/car_down.sh" < /dev/null; fi
    bash "$REPO/tools/car_run/car_down.sh" < /dev/null
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
    uptime; xmutil listapps 2>&1 | grep -E 'm7_pl|Active' | head -8
    xmutil unloadapp >/dev/null 2>&1; xmutil loadapp m7_pl_merge_a2 2>&1 | tail -1; sleep 2
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
  HERE=$(cd "$(dirname "$0")" && pwd)
  W=${W:-${FPGA_SRC}}
  R=${R:-${FPGA_SRC}/runs/hbgap-t1-20261016}
  : "${DEPLOY:=9aab163}" "${CLEAN_S:=2400}" "${SIDE_S:=1200}" "${LOCK_POLL_S:=120}" "${LOCK_WAIT_MAX_S:=1800}"
  : "${NANO_SSH:=jetson}" "${KV_SSH:=kv260}" "${SSH_TRIES:=12}"
  KV_SECRET=${FPGA_HOME}/dev1/.secrets/kv260_sudo
  LOCK="bash ${FPGA_HOME}/tools/lock.sh"
  T=$(date -u +%m%d_%H%M); NAME=t1_$T
  if [ "$DRY" = 1 ]; then L=$R/dryrun_$T; else L=$R/log_$T; fi
  mkdir -p "$L" || exit 2
  log() { echo "[$(date -u +%H:%M:%S)Z $(date +%H:%M:%S)] $*" | tee -a "$L/session.log"; }
  X() { if [ "$DRY" = 1 ]; then echo "[dry] $*" >> "$L/session.log"; echo "[dry] $*"; return 0; fi; "$@"; }
  nano() { local n rc; for ((n=1; n<=SSH_TRIES; n++)); do X ssh "$NANO_SSH" "bash $NB/t1_session.sh nano $*"; rc=$?
    [ $rc -ne 255 ] && return $rc; log "ssh to Nano failed (try $n): nano $1"; sleep 20; done; return 255; }
  ksu() { local n rc; for ((n=1; n<=SSH_TRIES; n++)); do X ssh "$KV_SSH" "sudo -S -p '' bash $KB/t1_session.sh kv $*" < "$KV_SECRET"; rc=$?
    [ $rc -ne 255 ] && return $rc; log "ssh to KV failed (try $n): kv $1"; sleep 20; done; return 255; }
  pause() { [ "$DRY" = 1 ] && { echo "[dry] sleep $1"; return 0; }; sleep "$1"; }
  LINK_LOST=0 CLEAN_FAIL=0 TOK="$L/tokens" KV_LOADED=0 SRV="" PULLED=0 UP=0 SAMPLING=0
  COMMIT=$(git -C "$W" rev-parse "$DEPLOY^{commit}") || exit 2
  : > "$TOK"
  { echo "deploy_commit=$COMMIT"; echo "name=$NAME"; echo "clean_s=$CLEAN_S"; echo "side_s=$SIDE_S"; } > "$L/config.txt"
  log "T1 session (dry=$DRY) commit $COMMIT name $NAME -> $L"

  pull() {
    [ "$PULLED" = 1 ] && return 0
    mkdir -p "$L/nano" "$L/kv"
    if [ "$DRY" = 1 ]; then X "pull"; else
      local n; for ((n=1; n<=SSH_TRIES; n++)); do
        ssh "$NANO_SSH" "tar czf - -C $NB runs" | tar xzf - -C "$L/nano" && break; log "nano pull incomplete (try $n)"; sleep 20; done
      for ((n=1; n<=SSH_TRIES; n++)); do
        ssh "$KV_SSH" "sudo -S -p '' tar czf - -C $KB runs" < "$KV_SECRET" | tar xzf - -C "$L/kv" && break; log "kv pull incomplete (try $n)"; sleep 20; done
    fi
    PULLED=1; log "pulled to $L/nano $L/kv"
  }
  cleanup() {
    trap - EXIT INT TERM HUP
    log "cleanup start"
    if [ -s "$TOK" ] || [ "$DRY" = 1 ]; then
      [ "$UP" = 1 ] && { nano down "$NAME" >> "$L/session.log" 2>&1; [ $? = 255 ] && CLEAN_FAIL=1; UP=0; }
      nano stopall >> "$L/session.log" 2>&1; [ $? = 255 ] && CLEAN_FAIL=1
      [ "$SAMPLING" = 1 ] && { nano sample_stop "$NB/runs/t1side" >> "$L/session.log" 2>&1; SAMPLING=0; }
      [ -n "$SRV" ] && { ksu stop "$SRV" >> "$L/session.log" 2>&1; SRV=""; }
      if [ "$KV_LOADED" = 1 ]; then
        ksu restore > "$L/kv_restore.log" 2>&1; [ $? = 255 ] && CLEAN_FAIL=1; cat "$L/kv_restore.log" >> "$L/session.log"
        grep -q "$BASE_BID" "$L/kv_restore.log" || [ "$DRY" = 1 ] || log "WARNING: m7_pl $BASE_BID not read back after restore"
        ksu residue > "$L/kv_residue.txt" 2>&1; cat "$L/kv_residue.txt" >> "$L/session.log"
      fi
      nano env "$NB/runs/env_end_$T.txt" >> "$L/session.log" 2>&1
      pull
    fi
    if [ "$CLEAN_FAIL" = 1 ]; then log "a board was unreachable during cleanup: locks KEPT (tokens in $TOK); finish by hand"; cp "$TOK" "$R/tokens_held_$T.txt"; : > "$TOK"; fi
    while read -r res tok; do [ -n "$tok" ] && X $LOCK release "$res" --token "$tok" -u dev1 >> "$L/session.log" 2>&1 && log "released $res"; done < "$TOK"
    log "cleanup done"; echo CLEANUP_DONE >> "$L/session.log"
  }
  # 10-16: an ssh failure (rc 255 after SSH_TRIES) is NOT a stack failure -- the stack runs on the Nano without us.
  # Link lost: log, keep waiting (no cleanup); only an explicit stack=false/absent answer fails the run.
  hb_check() {
    local rc
    nano check "$NAME" >> "$L/checks.txt" 2>&1; rc=$?
    if [ "$rc" = 255 ]; then log "link to the Nano lost during $1 (ssh 255); not a stack failure, waiting"; LINK_LOST=$((LINK_LOST + 1)); return 0; fi
    tail -1 "$L/checks.txt" | grep -q 'stack=true' && return 0
    log "stack not running ($1): $(tail -1 $L/checks.txt)"; return 1
  }
  trap cleanup EXIT
  trap 'log "signal"; exit 1' INT TERM HUP

  # --- locks: all three or none, wait (bounded), never steal
  local WHY="T1 心跳空档上板补测（车静止对空地、--chassis none 不接底盘、≥40 min＋20 min 旁路；A2 比特流），dev1 子代理，owner 10-16 同意"
  if [ "$DRY" = 1 ]; then X $LOCK acquire kv260 "$WHY" -u dev1
  elif [ -n "${REUSE_TOKENS:-}" ]; then   # 10-16: the three locks are already ours (kept across a power loss)
    cp "$REUSE_TOKENS" "$TOK"; [ "$(wc -l < "$TOK")" = 3 ] || { log "REUSE_TOKENS must hold 3 lines"; : > "$TOK"; exit 3; }
    for r in kv260 orin-nano camera; do t=$(awk -v r=$r '$1 == r {print $2}' "$TOK")
      $LOCK status 2>/dev/null | grep -q "\"token\": \"$t\"" || { log "token for $r not held"; : > "$TOK"; exit 3; }; done
    log "reusing our held locks kv260 orin-nano camera ($REUSE_TOKENS)"
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
      [ "$waited" -ge "$LOCK_WAIT_MAX_S" ] && { log "locks busy for $waited s: giving up (report to controller)"; echo "$st" > "$L/lock_status_busy.txt"; exit 3; }
      log "locks busy, waiting"; sleep "$LOCK_POLL_S"; waited=$((waited + LOCK_POLL_S))
    done
    log "acquired kv260 orin-nano camera"
  fi

  # --- deploy + sha table
  X git -C "$W" archive --format=tar -o "$L/repo.tar" "$COMMIT" host robot_sim tools tests hw perception pc || exit 1
  echo "$COMMIT" > "$L/COMMIT"
  mkdir -p "$L/ref" && tar xf "$L/repo.tar" -C "$L/ref" && (cd "$L/ref" && find . -type f -print0 | LC_ALL=C sort -z | xargs -0 sha256sum) > "$L/sha_host.txt"
  local ME="$W/tools/hb_gap_dig/t1/t1_session.sh"
  X ssh "$NANO_SSH" "mkdir -p $NB/runs $NB/st && rm -rf $NB/repo && mkdir -p $NB/repo" || exit 1
  X scp -q "$ME" "$W/tools/hb_gap_dig/t1/t1_sampler.py" "$W/tools/hb_gap_dig/t1/hb_sidecar.py" "$NANO_SSH:$NB/" || exit 1
  if [ "$DRY" = 1 ]; then X "ssh nano tar x"; else ssh "$NANO_SSH" "tar xf - -C $NB/repo" < "$L/repo.tar" || exit 1; fi
  X scp -q "$L/COMMIT" "$NANO_SSH:$NB/repo/COMMIT" || exit 1
  printf 'export LIVE_LAUNCH_ARGS=%q\n' "$DEF_LAUNCH enable_frame_drop_log:=true frame_timestamp_csv_file:=/o/drv_frames.csv" > "$L/up_env.sh"
  X scp -q "$L/up_env.sh" "$NANO_SSH:$NB/up_env.sh" || exit 1
  X ssh "$KV_SSH" "mkdir -p $KB" && X scp -q "$ME" "$KV_SSH:$KB/t1_session.sh" && ksu clean > "$L/kv_shm_before.txt" 2>&1 || exit 1
  if [ "$DRY" = 1 ]; then X "ssh kv tar x"; else ssh "$KV_SSH" "mkdir -p $KB/repo && tar xf - -C $KB/repo" < "$L/repo.tar" || exit 1; fi
  if [ "$DRY" != 1 ]; then
    nano sha > "$L/sha_nano.txt"; ksu sha > "$L/sha_kv.txt"
    if cmp -s "$L/sha_host.txt" "$L/sha_nano.txt" && cmp -s "$L/sha_host.txt" "$L/sha_kv.txt"; then
      log "sha table: $(wc -l < "$L/sha_host.txt") files identical on Nano and KV"
    else log "sha table MISMATCH (diff in $L/sha_*.txt)"; diff "$L/sha_host.txt" "$L/sha_nano.txt" | head > "$L/sha_diff.txt"; exit 1; fi
  fi
  log "deployed $COMMIT"

  # --- Nano checks
  nano env "$NB/runs/env_start_$T.txt"
  [ "$DRY" = 1 ] || ssh "$NANO_SSH" "cat $NB/runs/env_start_$T.txt" > "$L/nano_env_start.txt"
  if [ "$DRY" != 1 ]; then
    grep -q 'MAXN_SUPER' "$L/nano_env_start.txt" || { log "Nano not in MAXN_SUPER; abort"; exit 1; }
    grep -q 'listen 47431: 0' "$L/nano_env_start.txt" || { log "port 47431 busy on the Nano"; exit 1; }
    grep -q 'orbbec: Bus' "$L/nano_env_start.txt" || { log "no Orbbec camera on the Nano's USB"; exit 1; }
    grep -q 'car.current: none' "$L/nano_env_start.txt" || { log "a car is already running on the Nano"; exit 1; }
  fi
  # --- KV260: A2 + kv_server
  KV_LOADED=1
  ksu prep > "$L/kv_prep.log" 2>&1; cat "$L/kv_prep.log" >> "$L/session.log"
  [ "$DRY" = 1 ] || grep -q "$A2_BID" "$L/kv_prep.log" || { log "A2 BUILD_ID $A2_BID not read"; exit 1; }
  SRV=$KB/runs/srv_$NAME
  ksu start "$SRV" $SARGS > "$L/kv_start.log" 2>&1; cat "$L/kv_start.log" >> "$L/session.log"
  [ "$DRY" = 1 ] || grep -q '"port"' "$L/kv_start.log" || { log "kv_server not ready"; exit 1; }
  pause 2
  # --- sampler before the car (baseline)
  nano sample_start "$NB/runs/t1side" >> "$L/session.log" 2>&1 && SAMPLING=1
  pause 20
  # --- car_up
  log "car_up live --pose static --chassis none --nav costmap (LIVE_LAUNCH_ARGS + driver frame log)"
  UP=1
  nano up "$NAME" >> "$L/session.log" 2>&1; local rc=$?
  if [ "$rc" != 0 ]; then log "car_up rc=$rc (car_up rolled back); stack_fail: $(ssh $NANO_SSH "cat $NB/runs/$NAME/stack_fail.txt" 2>/dev/null | head -3)"; UP=0; exit 1; fi
  local t_ready; t_ready=$(date +%s); echo "ready_wall=$(wall)" >> "$L/config.txt"
  log "READY"
  ssh "$NANO_SSH" "grep -E 'camera driver try|ground|ready' $NB/runs/$NAME/marks.txt" > "$L/marks_head.txt" 2>&1
  # self-test of the side receiver path (~20 s right after READY, long before the 180 s criterion buckets), moved aside
  echo "selftest_side_start_wall=$(wall)" >> "$L/config.txt"
  nano side_start "$NAME" > "$L/selftest_side.txt" 2>&1; nano check "$NAME" >> "$L/selftest_side.txt" 2>&1
  nano side_stop "$NAME" >> "$L/selftest_side.txt" 2>&1
  echo "selftest_side_end_wall=$(wall)" >> "$L/config.txt"
  ssh "$NANO_SSH" "mv $NB/runs/$NAME/t1 $NB/runs/$NAME/t1_selftest"
  grep -q 'sidecar=1' "$L/selftest_side.txt" && grep -q '"event": "stop"' "$L/selftest_side.txt" \
    || { log "side receiver self-test failed: $(tr '\n' ' ' < $L/selftest_side.txt | head -c 400)"; exit 1; }
  log "side receiver self-test ok: $(grep -o 'side_lines=[0-9]* over=[0-9]*' $L/selftest_side.txt)"
  # --- drv_frames.csv must grow
  pause 60; local a b; a=$(ssh "$NANO_SSH" "wc -l < $NB/runs/$NAME/drv_frames.csv" 2>/dev/null); pause 10
  b=$(ssh "$NANO_SSH" "wc -l < $NB/runs/$NAME/drv_frames.csv" 2>/dev/null)
  log "drv_frames.csv lines $a -> $b"
  if [ "$DRY" != 1 ] && { [ -z "$b" ] || [ "${b:-0}" -le "${a:-0}" ]; }; then log "driver frame record not growing: stop (no formal window)"; exit 1; fi
  # --- clean window
  local el tinfo=0
  while true; do
    el=$(( $(date +%s) - t_ready ))
    [ "$el" -ge "$CLEAN_S" ] && break
    if [ "$tinfo" = 0 ] && [ "$el" -ge 180 ]; then nano topicinfo "$NAME" early >> "$L/session.log" 2>&1; tinfo=1; fi
    hb_check "clean window at ${el}s" || exit 1
    pause 60
  done
  log "clean window done (${el}s after READY)"; echo "clean_end_wall=$(wall)" >> "$L/config.txt"
  # --- side window
  nano side_start "$NAME" >> "$L/session.log" 2>&1; echo "side_start_wall=$(wall)" >> "$L/config.txt"
  local t_side; t_side=$(date +%s)
  while [ $(( $(date +%s) - t_side )) -lt "$SIDE_S" ]; do
    hb_check "side window" || exit 1
    pause 60
  done
  nano topicinfo "$NAME" late >> "$L/session.log" 2>&1
  nano side_stop "$NAME" >> "$L/session.log" 2>&1; echo "side_end_wall=$(wall)" >> "$L/config.txt"
  log "side window done"
  nano down "$NAME" >> "$L/session.log" 2>&1; UP=0
  nano sample_stop "$NB/runs/t1side" >> "$L/session.log" 2>&1; SAMPLING=0
  log "T1 complete"
}

case "${1:-}" in
  nano) shift; nano_main "$@";;
  kv) shift; kv_main "$@";;
  run) shift; run_main "$@";;
  *) sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'; exit 2;;
esac
