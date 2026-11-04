#!/bin/bash
# Line 4 (2026-10-15): the live car on the bench, one command -- the two boards + the real camera fixed in the car's
# mount (0.36 m, 30 deg down), NO chassis (the car cannot move). Driven from the SERVER, like
# docs/design/integ_car_chain_v3_20261011/session_v3.sh: the lock tool, the KV260's root password and the ssh routes
# to both boards live here; this same file is copied to the Nano and the KV260 and run there as "nano"/"kv".
#
#   bash tools/car_run/bench_line4.sh run [--dry-run]     (server; --dry-run: print every step, touch nothing)
#   bash tools/car_run/bench_line4.sh nano <sub> ...       (Orin Nano, user robot; called by run)
#   bash tools/car_run/bench_line4.sh kv <sub> ...         (KV260, root via sudo -S; called by run)
#
# run: locks kv260 + orin-nano + camera (wait, never steal; tokens kept, released by token at exit) -> deploy the
#   HEAD commit of W (git archive; W must have no uncommitted tracked change) to both boards -> Nano checks
#   (nvpmodel MAXN_SUPER, Orbbec on USB, no car running) -> KV260: load the A2 merged bitstream m7_pl_merge_a2
#   (BUILD_ID 0x00da5041) -> [TRY_RTABMAP=1] one car_up with --pose rtabmap (DRAFT) -- only when RTABMAP_DB is given:
#   without it car_up refuses at once with 2 (the expected result, controller 10-15), so the attempt is skipped with
#   one line of reason (cycles.txt "rtabmap_skipped") and costs no board time; if it reaches READY the formal cycles
#   use rtabmap, else they fall back to --pose static (owner 10-15: rtabmap is then its own next item; the static run
#   does NOT show real mapping) -> CYCLES (3) formal (fail-stop) cycles, each on its own kv_server:
#     car_up.sh live --pose static|rtabmap --chassis none --nav costmap (hub on 127.0.0.1; CAR_BOARD_DRAFT=1)
#     -> WARM_S (20) -> a WINDOW_S (120) s window (CLOCK_MONOTONIC_RAW bounds in window.txt, as the rigs)
#     -> the PC side through the hub: car_pc.sh listen PCMEM_S (8) s (MEMORY frames = what the PC is told the car
#        remembers) and car_pc.sh find FIND_TEXT (瓶子)
#     -> fail-stop injection: FAILSTOP_SIGNAL (KILL: a crash) to the stack child FAILSTOP_NODE (producer) by its pid
#        (pids.txt, inside the stack container: docker exec kill); car_inner.sh's 1 s supervision must stop the whole
#        stack (stack_fail.txt "... exited after READY ...: fail-stop") -> car_down.sh -> residue check
#   -> [NORMAL_STOP=1] one normal-stop cycle (controller 10-15): the same up / window / PC side / query, then
#      car_down.sh directly, no injection (judged: no residue, every child stopped in order, rc normal -- PREREG C6)
#   -> [JETSON_CLOCKS_ARM=1] one more cycle with jetson_clocks locked (record only; --store before, --restore after)
#   -> pull the Nano runs and the KV server dirs -> tools/car_run/bench_line4_readout.py (judged per PREREG).
#   A failed car_up, a stack that died in the window or a car_down that left residue ends the cycles: cleanup and pull
#   keep the evidence. EXIT trap: stop any car on the Nano, stop kv_server, restore m7_pl (0x0117bc0a), restore the
#   clocks if they were locked, release the three locks by token, pull.
# env: W (tree to deploy; default this script's repo)  R (results dir)  CYCLES (3)  WARM_S (20)  WINDOW_S (120)
#   GAP_S (75: between driver starts; a driver restarted within seconds once hung at "open device")  PCMEM_S (8)
#   FIND_TEXT (瓶子)  BENCH_OBJ ("<label> <x> <y>": the object as placed, metres in base_link = map: x forward from
#   the base origin (0.10 m behind the lens), y left; measured by the controller; goes to the readout)
#   (no BENCH_OBJ: the query still runs and its answer is recorded; criterion 5 is "not judged", readout exit 3)
#   SITE_NOTE (what the site lacked, e.g. 现场无光照: into the C5 "not judged" note)
#   FIND_ONLY (0; 1 = the find-only rerun: CYCLES=0 NORMAL_STOP=1 TRY_RTABMAP=0 JETSON_CLOCKS_ARM=0 -- up once, window,
#   PC side + query, normal stop; the readout judges C5 on that cycle (and C6), C1-C4 left out)
#   CAR_HEIGHT_CELL (8, given explicitly -- owner 10-15: the height is hand-measured, 0.35-0.37 m; cell 8 holds the
#   ground over 0.34-0.38 m, while the derivation would give 7 at 0.35; CAR_CAM_HEIGHT stays car_up's default, 0.375 since
#   owner 10-16, 0.39 m / 7.0 deg measured since 10-17). Bench only (the line-4 bench rig): the real car does NOT pass CAR_HEIGHT_CELL -- the ground self-calibration
#   picks the cell from the fitted height (owner 10-16, DECISIONS D-041; a given cell refuses the start when the fitted
#   height does not fit it)
#   FAILSTOP_NODE (producer)  FAILSTOP_SIGNAL (KILL)  FAILSTOP_WAIT_S (90)  NORMAL_STOP (1)
#   TRY_RTABMAP (1; tried only with RTABMAP_DB)  RTABMAP_DB (Nano path)  JETSON_CLOCKS_ARM (0)  LOCK_POLL_S (120)  NANO_SSH (jetson)  KV_SSH (kv260)  SSH_TRIES (12)
#   car_up pass-through (only those set here reach the Nano, in up_env.sh): LIVE_LAUNCH_ARGS LIVE_START_TRIES
#   LIVE_START_WAIT_S LIVE_START_GAP_S LIVE_DEPTH_MSGS LIVE_CAMERA_FRAME LIVE_COLOR_TOPIC LIVE_DEPTH_TOPIC
#   LIVE_INFO_TOPIC LIVE_COLOR_INFO_TOPIC CAMERA_CHECK_S RUNNER_CAMERA_QOS BENCH_ODOM_HZ BENCH_MAP_M CAR_CAM_HEIGHT
#   CAR_CAM_PITCH_DEG CAR_HEIGHT_CELL YOLO_ENGINE NANO_PYLIB CAR_READY_S INF_READY_S BOARD_IMAGE CAR_HUB_BIND
#   CAR_ROBOT_HEIGHT (10-16; unset = car_up's default 0.31 m, the car body height of the obstacle band)
#   CAR_GROUND_CALIB (10-15; unset = car_up's default auto: the bench camera must then see 0.3-2 m of free floor)
#   (LIVE_LAUNCH_ARGS: the driver's launch arguments, car_up's default when unset -- the camera start-up fix may
#   change them; nothing here hard-codes them)
# Never pkill -f: processes by pid, containers by name, locks by token.
set -u
# Nano base: repo/ runs/ st/. On /data (NVMe, 168 G free 10-15), not ~ (root partition mmcblk0p1: 88 % used, 6.7 G
# free on 10-15; the line-4 session put 29 M there). Kept short for the Unix socket limit (sun_path 108 bytes incl.
# the NUL, so 107): the longest socket is car_up's inference socket CAR_STATE_DIR/<tag>/inf.sock =
# /data/fpga/b4/st/b4_MMDD_HHMM_rtab/inf.sock = 42 bytes (cycle tags b4_MMDD_HHMM_c<N> / _ns / _jc are shorter;
# the old ~/fpga/bench4 gave 53). Inside the stack container the socket is /ipc/inf.sock.
NB=/data/fpga/b4
KB=/home/ubuntu/fpga/bench4                 # KV260 base: repo/ runs/
A2_TEST=/home/ubuntu/fpga/pl-merge-a2-0da5041/pkg/m7pl_test
BASE_TEST=/home/ubuntu/fpga/package-100mhz/m7pl_test
A2_BID=0x00da5041; BASE_BID=0x0117bc0a
KV_PORT=47800
SARGS="--core pl --device devmem --wait irq --roi-core pl --port $KV_PORT --max-idle-s 900"
PASS_VARS="LIVE_LAUNCH_ARGS LIVE_FRAME_DROP_LOG LIVE_START_TRIES LIVE_START_WAIT_S LIVE_START_GAP_S LIVE_DEPTH_MSGS LIVE_CAMERA_FRAME
  LIVE_COLOR_TOPIC LIVE_DEPTH_TOPIC LIVE_INFO_TOPIC LIVE_COLOR_INFO_TOPIC CAMERA_CHECK_S RUNNER_CAMERA_QOS BENCH_ODOM_HZ
  BENCH_MAP_M CAR_CAM_HEIGHT CAR_CAM_PITCH_DEG CAR_HEIGHT_CELL YOLO_ENGINE NANO_PYLIB CAR_READY_S INF_READY_S BOARD_IMAGE
  CAR_HUB_BIND CAR_GROUND_CALIB CAR_ROBOT_HEIGHT"
raw_ns() { python3 -c 'import time; print(time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW))'; }
wall() { date +%s.%N; }
# acquire RESOURCE WHY ($LOCK) -> our token on stdout; nothing and rc 1 when lock.sh refused (its answer on stderr).
# 10-15, as session_v3.sh acq() (board-fixes 38ae200): a refusal ("错误: 资源已被占用: {..., "token": "<theirs>", ...}",
# rc 1) names the HOLDER's lease, and grep -o '[0-9a-f]{32}' | tail -1 on the merged output read that token as ours
# (then "all three acquired" and the holder's lock released by us at exit). rc first, then only the "token" field.
lock_token() { local o; o=$($LOCK acquire "$1" "$2" -u dev1 2>&1) || { echo "lock $1 not acquired: $o" >&2; return 1; }
  echo "$o" | grep -o -E '"token": "[0-9a-f]{32}"' | grep -o -E '[0-9a-f]{32}' | head -1; }

# =============================================================================================== Nano side
nano_main() {
  local sub=$1; shift
  local RUNS=$NB/runs ST=$NB/st REPO=$NB/repo
  export CAR_STATE_DIR=$ST CAR_RUNS_DIR=$RUNS
  side() { echo "$RUNS/$1.side"; }
  case "$sub" in
  env)   # FILE
    { date -Is; nvpmodel -q 2>&1; uptime; cat /proc/loadavg; free -m | head -2
      for p in 0 4; do d=/sys/devices/system/cpu/cpufreq/policy$p; echo "policy$p $(cat $d/scaling_governor 2>/dev/null) cur=$(cat $d/scaling_cur_freq 2>/dev/null) max=$(cat $d/scaling_max_freq 2>/dev/null)"; done
      echo "gpu cur=$(cat /sys/devices/platform/17000000.gpu/devfreq/17000000.gpu/cur_freq 2>/dev/null) min=$(cat /sys/devices/platform/17000000.gpu/devfreq/17000000.gpu/min_freq 2>/dev/null)"
      echo "listen 47431: $(ss -ltn 2>/dev/null | grep -c ':47431 ')"
      echo "orbbec: $(lsusb -d 2bc5: 2>/dev/null | head -1)"
      echo "car.current: $(readlink $ST/car.current 2>/dev/null || echo none)"
      echo "rtabmap_db: ${RTABMAP_DB:-<none>} $([ -n "${RTABMAP_DB:-}" ] && [ -f "$RTABMAP_DB" ] && echo exists || echo absent)"
      docker ps -a --format '{{.Names}} {{.Image}} {{.Status}}'; top -bn1 | head -15; } > "$1" 2>&1 ;;
  up)    # NAME POSE: car_up for the bench; side/{up_t0,up_t1,up_rc,up.log}
    local name=$1 pose=$2 sd; sd=$(side "$name"); mkdir -p "$RUNS" "$ST" "$sd"
    [ -f "$NB/up_env.sh" ] && source "$NB/up_env.sh"
    wall > "$sd/up_t0"
    CAR_BOARD_DRAFT=1 CAR_HUB_BIND=${CAR_HUB_BIND:-127.0.0.1} RTABMAP_DB=${RTABMAP_DB:-} \
      bash "$REPO/tools/car_run/car_up.sh" live --pose "$pose" --chassis none --nav costmap --run-dir "$RUNS/$name" \
      > "$sd/up.log" 2>&1 < /dev/null
    local rc=$?; echo $rc > "$sd/up_rc"; wall > "$sd/up_t1"; echo "up $name pose=$pose rc=$rc"; return $rc ;;
  window_start)   # NAME WARM WIN: detached (an ssh drop must not end the window); writes side/window.txt at the end
    local name=$1 sd; sd=$(side "$name")
    setsid nohup bash "$NB/bench_line4.sh" nano window "$@" > "$sd/window.log" 2>&1 < /dev/null &
    echo "window started" ;;
  window)   # NAME WARM WIN (foreground; run by window_start)
    local name=$1 warm=$2 win=$3 sd stack; sd=$(side "$name")
    stack=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['containers']['stack'])" "$RUNS/$name/car.json")
    sleep "$warm"
    local s_raw s_wall e_raw e_wall run_s run_e
    run_s=$(docker inspect -f '{{.State.Running}}' "$stack" 2>/dev/null)
    s_raw=$(raw_ns); s_wall=$(wall)
    sleep "$win"
    e_raw=$(raw_ns); e_wall=$(wall)
    run_e=$(docker inspect -f '{{.State.Running}}' "$stack" 2>/dev/null)
    { echo "window_start_raw_ns=$s_raw"; echo "window_end_raw_ns=$e_raw"; echo "window_start_wall=$s_wall"
      echo "window_end_wall=$e_wall"; echo "warm_s=$warm"; echo "window_s=$win"
      echo "stack_running_at_start=${run_s:-false}"; echo "stack_running_at_end=${run_e:-false}"
      echo "stack_fail_at_end=$(cat "$RUNS/$name/stack_fail.txt" 2>/dev/null | head -1)"; } > "$sd/window.tmp"
    mv "$sd/window.tmp" "$sd/window.txt" ;;
  waitfile)   # NAME FILE TIMEOUT_S
    local sd i; sd=$(side "$1")
    for ((i=0; i<$3; i++)); do [ -e "$sd/$2" ] && { echo "have $2"; return 0; }; sleep 1; done
    echo "no $2 in $3 s"; return 1 ;;
  pcmem)   # NAME SECS: the PC side's view of the car's memory, through the hub (display client, yields control)
    local name=$1 sd; sd=$(side "$name")
    CAR_RUN=$RUNS/$name timeout $(($2 + 30)) bash "$REPO/tools/car_run/car_pc.sh" listen --yield-control --duration-s "$2" \
      --out /o/pc_mem.fpgarec --stats /o/pc_mem_stats.json > "$sd/pcmem.txt" 2>&1 < /dev/null
    echo "pcmem rc=$?" | tee -a "$sd/pcmem.txt" ;;
  find)   # NAME TEXT: car_pc.sh find (ACK / RESULT lines) + the send time (wall, the runner's event clock on hardware)
    local name=$1 sd; sd=$(side "$name")
    wall > "$sd/find_sent"
    CAR_RUN=$RUNS/$name timeout 60 bash "$REPO/tools/car_run/car_pc.sh" find "$2" > "$sd/find.txt" 2>&1 < /dev/null
    echo "find_rc=$?" >> "$sd/find.txt"; sleep 5; cat "$sd/find.txt" ;;
  failstop)   # NAME NODE SIGNAL WAIT_S: kill one supervised stack child by pid; time the stack's own fail-stop
    local name=$1 node=$2 sig=$3 waits=$4 sd stack pid rc t_kill t_fail="" t_exit="" i; sd=$(side "$name")
    stack=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['containers']['stack'])" "$RUNS/$name/car.json")
    pid=$(awk -v n="$node" '$1 == n {p = $2} END {print p}' "$RUNS/$name/pids.txt" 2>/dev/null)
    [ -n "$pid" ] || { echo "{\"error\": \"no pid for $node in pids.txt\"}" > "$sd/failstop.json"; echo "no pid for $node"; return 1; }
    t_kill=$(wall)
    docker exec "$stack" bash -c 'kill -s "$0" "$1"' "$sig" "$pid" > "$sd/failstop_kill.txt" 2>&1; rc=$?
    for ((i=0; i<waits*5; i++)); do
      [ -z "$t_fail" ] && [ -s "$RUNS/$name/stack_fail.txt" ] && t_fail=$(wall)
      [ -z "$t_exit" ] && [ "$(docker inspect -f '{{.State.Running}}' "$stack" 2>/dev/null)" != true ] && t_exit=$(wall)
      [ -n "$t_fail" ] && [ -n "$t_exit" ] && break
      sleep 0.2
    done
    python3 - "$sd/failstop.json" "$node" "$pid" "$sig" "$rc" "$t_kill" "$t_fail" "$t_exit" "$waits" \
      "$(cat "$RUNS/$name/stack_fail.txt" 2>/dev/null)" "$(docker inspect -f '{{.State.FinishedAt}} {{.State.ExitCode}}' "$stack" 2>/dev/null)" <<'PY'
import json, sys
out, node, pid, sig, rc, tk, tf, te, waits, fail, fin = sys.argv[1:]
f = lambda v: float(v) if v else None
d = {"node": node, "pid_in_container": int(pid), "signal": sig, "kill_rc": int(rc), "t_kill_wall": f(tk),
     "t_stack_fail_seen_wall": f(tf), "t_stack_exited_seen_wall": f(te), "wait_s": int(waits),
     "stack_fail": fail, "docker_finished_at_exit_code": fin, "poll_s": 0.2}
json.dump(d, open(out, "w"), indent=1, ensure_ascii=False)
print(json.dumps(d, ensure_ascii=False))
PY
    ;;
  down)   # NAME: car_down + an independent residue check (side/residue.json)
    local name=$1 sd; sd=$(side "$name")
    wall > "$sd/down_t0"
    bash "$REPO/tools/car_run/car_down.sh" --run "$RUNS/$name" > "$sd/down.log" 2>&1 < /dev/null; echo $? > "$sd/down_rc"
    wall > "$sd/down_t1"
    local cont ptr infp="" inf_alive=false
    cont=$(docker ps -a --format '{{.Names}}' | grep '^wde-car-' | tr '\n' ' ')
    ptr=$(readlink "$ST/car.current" 2>/dev/null)
    if [ -s "$RUNS/$name/host_pids.txt" ]; then
      while read -r n p s; do
        [ -r "/proc/$p/stat" ] && [ "$(awk '{print $22}' /proc/$p/stat 2>/dev/null)" = "$s" ] && { inf_alive=true; infp="$infp $n:$p"; }
      done < "$RUNS/$name/host_pids.txt"
    fi
    python3 - "$sd/residue.json" "$cont" "$ptr" "$inf_alive" "$infp" "$(cat "$sd/down_rc")" <<'PY'
import json, sys
out, cont, ptr, alive, procs, rc = sys.argv[1:]
d = {"car_down_rc": int(rc), "wde_car_containers": cont.split(), "pointer": ptr or None,
     "host_processes_alive": alive == "true", "host_processes": procs.split()}
d["clean"] = d["car_down_rc"] == 0 and not d["wde_car_containers"] and not d["pointer"] and not d["host_processes_alive"]
json.dump(d, open(out, "w"), indent=1)
print(json.dumps(d))
PY
    ;;
  stopall)   # cleanup: the current car (if any), then any stray wde-car-* container (car_down's own no-pointer path)
    if [ -L "$ST/car.current" ]; then bash "$REPO/tools/car_run/car_down.sh" < /dev/null; fi
    bash "$REPO/tools/car_run/car_down.sh" < /dev/null
    echo "stopall done" ;;
  clocks)   # store|on|restore|show (sudo -S: the password comes on stdin from the server)
    case "$1" in
      store)   sudo -S -p '' jetson_clocks --store "$NB/jetson_clocks.store" && echo stored ;;
      on)      sudo -S -p '' jetson_clocks && echo locked ;;
      restore) sudo -S -p '' jetson_clocks --restore "$NB/jetson_clocks.store" && echo restored ;;
      show)    sudo -S -p '' jetson_clocks --show ;;
      *) echo "clocks store|on|restore|show"; return 2 ;;
    esac ;;
  *) echo "nano: unknown $sub"; return 2 ;;
  esac
}

# =============================================================================================== KV260 side (root)
kv_main() {
  local sub=$1; shift
  case "$sub" in
  clean)   rm -rf "$KB/repo" "$KB/runs"; mkdir -p "$KB/runs"; chown ubuntu:ubuntu "$KB/runs" ;;
  prep)    # the A2 merged bitstream; identity = its own m7pl_test's sysinfo BUILD_ID (kv_board.sh check)
    xmutil unloadapp >/dev/null 2>&1; xmutil loadapp m7_pl_merge_a2 2>&1 | tail -1; sleep 2
    bash "$KB/repo/tools/kv_link/kv_board.sh" check "$A2_TEST" | grep -E 'identity|BUILD_ID|m7_roi|m7_frame' ;;
  start)   # OUT kv_server args...
    local out=$1; shift; rm -rf "$out"
    env PYTHONDONTWRITEBYTECODE=1 FPGA_M7_DMA_PROVIDER=dmaheap FPGA_M7_DMA_HEAP=reserved \
      bash "$KB/repo/tools/kv_link/kv_board.sh" start "$KB/repo" "$out" "$@" ;;
  stop)    bash "$KB/repo/tools/kv_link/kv_board.sh" stop "$1" | tail -2; chown -R ubuntu:ubuntu "$1" ;;
  restore) # back to the base bitstream m7_pl (0x0117bc0a), as session_v3 / kv_v2.sh
    xmutil desktop_enable >/dev/null 2>&1; echo "desktop_enable rc=$?"
    xmutil unloadapp >/dev/null 2>&1; xmutil loadapp m7_pl 2>&1 | tail -1; sleep 2
    "$BASE_TEST" --heap reserved info 2>&1 | grep -o -E 'BUILD_ID=0x[0-9a-fA-F]+|SLOT_MAP=0x[0-9a-fA-F]+' | head -2 ;;
  residue) ps -eo pid,args | grep -E 'kv_server.py|m7ar_present' | grep -v grep; echo "residue-check done" ;;
  *) echo "kv: unknown $sub"; return 2 ;;
  esac
}

# =============================================================================================== server side
run_main() {
  # (no "local" for the session state: the EXIT trap's cleanup reads it after run_main has returned)
  DRY=0
  [ "${1:-}" = --dry-run ] && DRY=1
  HERE=$(cd "$(dirname "$0")" && pwd)
  W=${W:-$(cd "$HERE/../.." && pwd)}
  R=${R:-${FPGA_SRC}/runs/car-bench-20261015-results}
  : "${CYCLES:=3}" "${WARM_S:=20}" "${WINDOW_S:=120}" "${GAP_S:=75}" "${PCMEM_S:=8}" "${FIND_TEXT:=瓶子}" "${BENCH_OBJ:=}"
  : "${FAILSTOP_NODE:=producer}" "${FAILSTOP_SIGNAL:=KILL}" "${FAILSTOP_WAIT_S:=90}" "${TRY_RTABMAP:=1}" "${RTABMAP_DB:=}"
  : "${NORMAL_STOP:=1}" "${SITE_NOTE:=}" "${FIND_ONLY:=0}" "${CAR_HEIGHT_CELL:=8}"
  if [ "$FIND_ONLY" = 1 ]; then CYCLES=0 NORMAL_STOP=1 TRY_RTABMAP=0 JETSON_CLOCKS_ARM=0; fi
  : "${JETSON_CLOCKS_ARM:=0}" "${LOCK_POLL_S:=120}" "${NANO_SSH:=jetson}" "${KV_SSH:=kv260}" "${SSH_TRIES:=12}"
  KV_SECRET=${FPGA_HOME}/dev1/.secrets/kv260_sudo NANO_SECRET=${FPGA_HOME}/dev1/.secrets/jetson_sudo
  LOCK="bash ${FPGA_HOME}/tools/lock.sh"
  T=$(date -u +%m%d_%H%M)
  if [ "$DRY" = 1 ]; then L=$R/dryrun_$T; else L=$R/log_$T; fi
  mkdir -p "$L" || exit 2
  log() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a "$L/session.log"; }
  # every remote / lock action goes through X: printed in a dry run, executed otherwise
  X() { if [ "$DRY" = 1 ]; then echo "[dry] $*" >> "$L/session.log"; echo "[dry] $*"; return 0; fi; "$@"; }
  nano() { local n rc; for ((n=1; n<=SSH_TRIES; n++)); do X ssh "$NANO_SSH" "bash $NB/bench_line4.sh nano $*"; rc=$?
    [ $rc -ne 255 ] && return $rc; log "ssh transport to the Nano failed (try $n): nano $1"; sleep 20; done; return 255; }
  nanosu() { local n rc; for ((n=1; n<=SSH_TRIES; n++)); do X ssh "$NANO_SSH" "bash $NB/bench_line4.sh nano $*" < "$NANO_SECRET"; rc=$?
    [ $rc -ne 255 ] && return $rc; log "ssh transport to the Nano failed (try $n): nano $1"; sleep 20; done; return 255; }
  ksu() { local n rc; for ((n=1; n<=SSH_TRIES; n++)); do X ssh "$KV_SSH" "sudo -S -p '' bash $KB/bench_line4.sh kv $*" < "$KV_SECRET"; rc=$?
    [ $rc -ne 255 ] && return $rc; log "ssh transport to the KV failed (try $n): kv $1"; sleep 20; done; return 255; }
  pause() { [ "$DRY" = 1 ] && { echo "[dry] sleep $1"; return 0; }; sleep "$1"; }
  TOK="$L/tokens" KV_LOADED=0 SRV="" CLOCKS_STORED=0 PULLED=0 POSE_FORMAL=static
  COMMIT=$(git -C "$W" rev-parse HEAD) || { echo "W=$W is not a git tree" >&2; exit 2; }
  if [ -n "$(git -C "$W" status --porcelain --untracked-files=no)" ]; then
    echo "W=$W has uncommitted tracked changes: the boards get the HEAD commit only (git archive); commit first" >&2
    [ "$DRY" = 1 ] || exit 2; fi
  : > "$TOK"; : > "$L/cycles.txt"
  { echo "commit=$COMMIT"; echo "w=$W"; for v in CYCLES WARM_S WINDOW_S GAP_S PCMEM_S FIND_TEXT BENCH_OBJ FAILSTOP_NODE \
      FAILSTOP_SIGNAL FAILSTOP_WAIT_S NORMAL_STOP SITE_NOTE FIND_ONLY CAR_HEIGHT_CELL TRY_RTABMAP RTABMAP_DB JETSON_CLOCKS_ARM; do echo "$v=${!v}"; done; } > "$L/config.txt"
  log "bench line 4 (dry=$DRY) commit $COMMIT -> $L"

  pull() {
    [ "$PULLED" = 1 ] && return 0
    mkdir -p "$L/nano" "$L/kv"
    if [ "$DRY" = 1 ]; then X "ssh $NANO_SSH tar czf - -C $NB runs | tar xzf - -C $L/nano"; X "ssh $KV_SSH tar czf - -C $KB runs | tar xzf - -C $L/kv"
    else
      local n; for ((n=1; n<=SSH_TRIES; n++)); do
        ssh "$NANO_SSH" "tar czf - -C $NB runs" | tar xzf - -C "$L/nano" && break; log "nano pull incomplete (try $n)"; sleep 20; done
      for ((n=1; n<=SSH_TRIES; n++)); do
        ssh "$KV_SSH" "tar czf - -C $KB runs" | tar xzf - -C "$L/kv" && break; log "kv pull incomplete (try $n)"; sleep 20; done
    fi
    PULLED=1; log "pulled to $L/nano $L/kv"
  }
  cleanup() {
    trap - EXIT INT TERM HUP
    log "cleanup start"
    if [ -s "$TOK" ] || [ "$DRY" = 1 ]; then
      nano stopall >> "$L/session.log" 2>&1
      [ -n "$SRV" ] && { ksu stop "$SRV" >> "$L/session.log" 2>&1; SRV=""; }
      if [ "$KV_LOADED" = 1 ]; then
        ksu restore > "$L/kv_restore.log" 2>&1; cat "$L/kv_restore.log" >> "$L/session.log"
        grep -q "$BASE_BID" "$L/kv_restore.log" || [ "$DRY" = 1 ] || log "WARNING: m7_pl $BASE_BID not read back after restore"
        ksu residue >> "$L/session.log" 2>&1
      fi
      if [ "$CLOCKS_STORED" = 1 ]; then nanosu clocks restore >> "$L/session.log" 2>&1 && CLOCKS_STORED=0; fi
      nano env "$NB/runs/env_end_$T.txt" >> "$L/session.log" 2>&1
      pull
    fi
    while read -r res tok; do [ -n "$tok" ] && X $LOCK release "$res" --token "$tok" -u dev1 >> "$L/session.log" 2>&1 && log "released $res"; done < "$TOK"
    log "cleanup done"; echo CLEANUP_DONE >> "$L/session.log"
  }
  trap cleanup EXIT
  trap 'log "signal"; exit 1' INT TERM HUP

  # --- locks: all three or none, wait, never steal
  local WHY="线4 台架整车$([ "$FIND_ONLY" = 1 ] && echo '（只补跑寻物）')（真相机 Gemini 2 固定装车姿态、无底盘；car_up live 一键起停×$CYCLES、120 s 帧龄窗口、寻物、fail-stop；A2 比特流），dev1 子代理"
  if [ "$DRY" = 1 ]; then X $LOCK acquire kv260 "$WHY" -u dev1; X $LOCK acquire orin-nano "$WHY" -u dev1; X $LOCK acquire camera "$WHY" -u dev1
  else
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
      sleep "$LOCK_POLL_S"
    done
    log "acquired kv260 orin-nano camera"
  fi

  # --- deploy the HEAD commit (+ COMMIT file: car_up's snapshot of a tree without .git) and this script
  X git -C "$W" archive --format=tar -o "$L/repo.tar" "$COMMIT" host robot_sim tools tests hw perception pc || exit 1
  echo "$COMMIT" > "$L/COMMIT"
  local ME="$W/tools/car_run/bench_line4.sh"
  X ssh "$NANO_SSH" "mkdir -p $NB/runs $NB/st && rm -rf $NB/repo && mkdir -p $NB/repo" || exit 1
  X scp -q "$ME" "$NANO_SSH:$NB/bench_line4.sh" || exit 1
  if [ "$DRY" = 1 ]; then X "ssh $NANO_SSH tar xf - -C $NB/repo < $L/repo.tar"; else ssh "$NANO_SSH" "tar xf - -C $NB/repo" < "$L/repo.tar" || exit 1; fi
  X scp -q "$L/COMMIT" "$NANO_SSH:$NB/repo/COMMIT" || exit 1
  { local v; for v in $PASS_VARS; do [ -n "${!v+x}" ] && printf 'export %s=%q\n' "$v" "${!v}"; done; } > "$L/up_env.sh"
  [ -n "$RTABMAP_DB" ] && printf 'export RTABMAP_DB=%q\n' "$RTABMAP_DB" >> "$L/up_env.sh"
  X scp -q "$L/up_env.sh" "$NANO_SSH:$NB/up_env.sh" || exit 1
  X ssh "$KV_SSH" "mkdir -p $KB" && X scp -q "$ME" "$KV_SSH:$KB/bench_line4.sh" && ksu clean || exit 1
  if [ "$DRY" = 1 ]; then X "ssh $KV_SSH mkdir -p $KB/repo && tar xf - -C $KB/repo < $L/repo.tar"
  else ssh "$KV_SSH" "mkdir -p $KB/repo && tar xf - -C $KB/repo" < "$L/repo.tar" || exit 1; fi
  log "deployed $COMMIT to $NANO_SSH:$NB/repo and $KV_SSH:$KB/repo"

  # --- Nano checks: power mode (owner 10-09: formal timing in MAXN_SUPER; not changed here), camera, no car
  nano env "$NB/runs/env_start_$T.txt"
  [ "$DRY" = 1 ] || ssh "$NANO_SSH" "cat $NB/runs/env_start_$T.txt" > "$L/nano_env_start.txt"
  if [ "$DRY" != 1 ]; then
    grep -q 'MAXN_SUPER' "$L/nano_env_start.txt" || { log "Nano not in MAXN_SUPER; not touching it, abort"; exit 1; }
    grep -q 'listen 47431: 0' "$L/nano_env_start.txt" || { log "port 47431 busy on the Nano"; exit 1; }
    grep -q 'orbbec: Bus' "$L/nano_env_start.txt" || { log "no Orbbec camera on the Nano's USB"; exit 1; }
    grep -q 'car.current: none' "$L/nano_env_start.txt" || { log "a car is already running on the Nano"; exit 1; }
  fi
  # --- KV260: A2
  KV_LOADED=1
  ksu prep > "$L/kv_prep.log" 2>&1; cat "$L/kv_prep.log" >> "$L/session.log"
  [ "$DRY" = 1 ] || grep -q "$A2_BID" "$L/kv_prep.log" || { log "A2 BUILD_ID $A2_BID not read"; exit 1; }
  start_srv() { SRV=$KB/runs/srv_$1
    ksu start "$SRV" $SARGS > "$L/start_$1.log" 2>&1; cat "$L/start_$1.log" >> "$L/session.log"
    [ "$DRY" = 1 ] || grep -q '"port"' "$L/start_$1.log" || { log "kv_server $1 not ready"; return 1; }; pause 2; }
  stop_srv() { [ -n "$SRV" ] && ksu stop "$SRV" >> "$L/session.log" 2>&1; SRV=""; }

  # one cycle: NAME KIND POSE -> 0 = the next cycle may run; 1 = stop (evidence kept)
  # KIND formal | jetson_clocks: fail-stop injection before car_down; normal_stop: car_down directly (no injection)
  cycle() {
    local name=$1 kind=$2 pose=$3 rc
    echo "$name $kind $pose" >> "$L/cycles.txt"
    start_srv "$name" || return 1
    log "$name ($kind): car_up live --pose $pose --chassis none --nav costmap"
    nano up "$name" "$pose" >> "$L/session.log" 2>&1; rc=$?
    if [ "$rc" != 0 ]; then log "$name: car_up rc=$rc (rolled back by car_up)"; stop_srv; return 1; fi
    nano window_start "$name" "$WARM_S" "$WINDOW_S" >> "$L/session.log" 2>&1
    nano waitfile "$name" window.txt $((WARM_S + WINDOW_S + 120)) >> "$L/session.log" 2>&1 \
      || { log "$name: no window.txt"; stop_x "$name"; return 1; }
    [ "$DRY" = 1 ] || ssh "$NANO_SSH" "cat $NB/runs/$name.side/window.txt" > "$L/$name.window.txt"
    if [ "$DRY" != 1 ] && ! grep -q 'stack_running_at_end=true' "$L/$name.window.txt"; then
      log "$name: the stack was not running at the end of the window ($(grep stack_fail "$L/$name.window.txt"))"; stop_x "$name"; return 1; fi
    nano pcmem "$name" "$PCMEM_S" >> "$L/session.log" 2>&1
    nano find "$name" "$FIND_TEXT" >> "$L/session.log" 2>&1
    if [ "$kind" = normal_stop ]; then log "$name: normal stop (no injection): car_down"
    else nano failstop "$name" "$FAILSTOP_NODE" "$FAILSTOP_SIGNAL" "$FAILSTOP_WAIT_S" >> "$L/session.log" 2>&1; fi
    nano down "$name" >> "$L/session.log" 2>&1; rc=$?
    stop_srv
    [ "$DRY" = 1 ] || ssh "$NANO_SSH" "cat $NB/runs/$name.side/residue.json" > "$L/$name.residue.json"
    if [ "$DRY" != 1 ] && ! grep -q '"clean": true' "$L/$name.residue.json"; then log "$name: residue after car_down"; return 1; fi
    log "$name done"
    return 0
  }
  stop_x() { nano down "$1" >> "$L/session.log" 2>&1; stop_srv; }

  local c ok=1
  if [ "$TRY_RTABMAP" = 1 ] && [ -z "$RTABMAP_DB" ]; then
    # controller 10-15: without a pinned localization database car_up refuses with 2 before anything (the expected
    # result); do not spend a kv_server start and a car_up on it
    echo "b4_${T}_rtab rtabmap_skipped rtabmap no RTABMAP_DB (car_up would exit 2 before anything: expected)" >> "$L/cycles.txt"
    echo "rtabmap=skipped_no_db" >> "$L/config.txt"
    log "rtabmap attempt skipped: no RTABMAP_DB (car_up --pose rtabmap would refuse with exit 2 at once -- the expected result); formal pose static (NOT real mapping; rtabmap is its own next item)"
  elif [ "$TRY_RTABMAP" = 1 ]; then
    local rn=b4_${T}_rtab
    echo "$rn rtabmap_try rtabmap" >> "$L/cycles.txt"
    start_srv rtab && { nano up "$rn" rtabmap >> "$L/session.log" 2>&1; local rrc=$?
      log "rtabmap attempt: car_up rc=$rrc"
      if [ "$rrc" = 0 ] && [ "$DRY" != 1 ]; then POSE_FORMAL=rtabmap; nano down "$rn" >> "$L/session.log" 2>&1; fi
      stop_srv; }
    [ "$DRY" = 1 ] && log "(dry run: the rtabmap attempt's result decides; printed with the static fallback)"
    log "formal pose: $POSE_FORMAL$([ "$POSE_FORMAL" = static ] && echo ' (rtabmap did not reach READY: static fallback, NOT real mapping; rtabmap is its own next item)')"
    pause "$GAP_S"
  fi
  echo "formal_pose=$POSE_FORMAL" >> "$L/config.txt"
  for ((c=1; c<=CYCLES; c++)); do
    cycle "b4_${T}_c$c" formal "$POSE_FORMAL" || { ok=0; log "cycle $c failed: stopping the cycles (evidence kept)"; break; }
    [ "$c" -lt "$CYCLES" ] && pause "$GAP_S"
  done
  if [ "$ok" = 1 ] && [ "$NORMAL_STOP" = 1 ]; then
    [ "$CYCLES" -gt 0 ] && pause "$GAP_S"
    cycle "b4_${T}_ns" normal_stop "$POSE_FORMAL" || { ok=0; log "normal-stop cycle failed (evidence kept)"; }
  fi
  if [ "$ok" = 1 ] && [ "$JETSON_CLOCKS_ARM" = 1 ]; then
    pause "$GAP_S"
    nanosu clocks show > "$L/clocks_before.txt" 2>&1
    nanosu clocks store >> "$L/session.log" 2>&1 && CLOCKS_STORED=1
    if [ "$CLOCKS_STORED" = 1 ] || [ "$DRY" = 1 ]; then
      nanosu clocks on >> "$L/session.log" 2>&1; nanosu clocks show > "$L/clocks_locked.txt" 2>&1
      cycle "b4_${T}_jc" jetson_clocks "$POSE_FORMAL" || log "jetson_clocks cycle failed (record only)"
      nanosu clocks restore >> "$L/session.log" 2>&1 && CLOCKS_STORED=0
      nanosu clocks show > "$L/clocks_after.txt" 2>&1
    fi
  fi
  pull
  local OBJ=(); [ -n "$BENCH_OBJ" ] && OBJ=(--expect "$BENCH_OBJ")
  [ -n "$BENCH_OBJ" ] || log "no BENCH_OBJ: criterion 5 will be 'not judged' (the query runs and its answer is recorded)"
  OBJ+=(--cycles "$CYCLES" --normal-stops "$([ "$NORMAL_STOP" = 1 ] && echo 1 || echo 0)")
  [ -n "$SITE_NOTE" ] && OBJ+=(--site-note "$SITE_NOTE")
  if [ "$DRY" = 1 ]; then echo "[dry] python3 $W/tools/car_run/bench_line4_readout.py $L --find-text $FIND_TEXT ${OBJ[*]} --json $L/readout.json"
  else python3 "$W/tools/car_run/bench_line4_readout.py" "$L" --find-text "$FIND_TEXT" "${OBJ[@]}" --json "$L/readout.json" \
    > "$L/readout.txt" 2>&1; log "readout rc=$? -> $L/readout.json"; fi
}

case "${1:-}" in
  nano) shift; nano_main "$@";;
  kv) shift; kv_main "$@";;
  run) shift; run_main "$@";;
  *) sed -n '2,55p' "$0" | sed 's/^# \{0,1\}//'; exit 2;;
esac
