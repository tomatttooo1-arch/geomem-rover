#!/bin/bash
# Line E (2026-10-14): stop the car started by car_up.sh, in reverse order, and check that nothing is left.
#
#   bash tools/car_run/car_down.sh [--run RUN_DIR] [--rollback REASON]
# Without --run: the car CAR_STATE_DIR/car.current points to. With no car recorded, containers named
# wde-car-* (only those) are still removed, so a lost pointer cannot leave a car running.
# Order: each container of containers.txt from the last one (stack, then the fake KV, then Gazebo):
#   docker kill -s INT (the stack's car_inner.sh then stops every child by pid, INT -> TERM -> KILL; the fake KV
#   writes kv/summary.json), wait; then TERM, wait; docker logs to the run directory; docker rm -f.
#   Then the host processes of host_pids.txt (inference service): INT -> TERM -> KILL by pid (+ start time).
#   A mapping run (RUN/mapping_status.json): the memory file is bound to the new database (mapping_memory_bind.py).
#   Then the GPU lock (token in gpu_lock.json), the IPC socket directory, the pointer, the source snapshot.
# Residue check -> RUN/down.json: no recorded container, no recorded process (pid + start time), no IPC
# directory, the lock token no longer held, no pointer. Exit 0 only when all of that holds, else 1.
# Never pkill -f.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/car_common.sh"
car_default_dirs   # board /data/fpga/car/state, host /data1/... (car_common.sh)
: "${CAR_DOCKER:=docker}" "${CAR_LOCK:=bash ${FPGA_HOME}/tools/lock.sh}"
: "${CAR_CONTAINER_STOP_S:=15}"   # CAR_STACK_STOP_S: explicit env > $RUN/stack_stop_s.txt (car_up, rtabmap|map) > 150
DK() { $CAR_DOCKER "$@"; }
POINTER=$CAR_STATE_DIR/car.current
RUN="" REASON=""
while [ $# -gt 0 ]; do
  case "$1" in
    --run) RUN=${2:-}; shift 2;; --rollback) REASON=${2:-}; shift 2;;
    *) echo "unknown argument '$1'" >&2; exit 2;;
  esac
done
if [ -z "$RUN" ]; then
  if [ -L "$POINTER" ]; then RUN=$(readlink "$POINTER")
  else
    LEFT=$(DK ps -a --format '{{.Names}}' 2>/dev/null | grep '^wde-car-')
    if [ -n "$LEFT" ]; then
      echo "no car recorded, but containers $(echo $LEFT) exist: removing them" >&2
      for c in $LEFT; do DK kill -s INT "$c" > /dev/null 2>&1; done; sleep 5
      for c in $LEFT; do DK rm -f "$c" > /dev/null 2>&1; done
    else echo "no car running"; fi
    exit 0
  fi
fi
[ -d "$RUN" ] || { echo "no run directory $RUN" >&2; [ -L "$POINTER" ] && [ "$(readlink "$POINTER")" = "$RUN" ] && rm -f "$POINTER"; exit 1; }
dmark() { echo "$(date +%s.%N) $*" >> "$RUN/down_marks.txt"; }
# the stack's stop wait: car_up writes stack_stop_s.txt when rtabmap runs inside (it saves its database on INT,
# rtabmap_car.sh: up to CAR_RTABMAP_STOP_S + the other children); an explicit CAR_STACK_STOP_S still wins
STOP_SRC=env
if [ -z "${CAR_STACK_STOP_S:-}" ] && [ -s "$RUN/stack_stop_s.txt" ]; then
  CAR_STACK_STOP_S=$(tr -dc '0-9' < "$RUN/stack_stop_s.txt"); STOP_SRC=stack_stop_s.txt
fi
[ -n "${CAR_STACK_STOP_S:-}" ] || { CAR_STACK_STOP_S=150; STOP_SRC=default; }
dmark "stack stop wait $CAR_STACK_STOP_S s ($STOP_SRC)"
dmark "car_down${REASON:+ rollback: $REASON}"
running() { [ "$(DK inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" = true ]; }
exists() { DK inspect "$1" > /dev/null 2>&1; }

# 1. containers, last started first
if [ -s "$RUN/containers.txt" ]; then
  mapfile -t CS < <(tac "$RUN/containers.txt")
  for c in "${CS[@]}"; do
    exists "$c" || { dmark "$c: not there"; continue; }
    limit=$CAR_CONTAINER_STOP_S; case "$c" in wde-car-stack-*) limit=$CAR_STACK_STOP_S;; esac
    if running "$c"; then
      DK kill -s INT "$c" > /dev/null 2>&1; dmark "$c: INT"
      for ((i=0; i<limit*5; i++)); do running "$c" || break; sleep 0.2; done
      if running "$c"; then
        DK kill -s TERM "$c" > /dev/null 2>&1; dmark "$c: still running ${limit} s after INT, TERM"
        for ((i=0; i<50; i++)); do running "$c" || break; sleep 0.2; done
        running "$c" && dmark "$c: still running after TERM, removed with KILL"
      fi
    fi
    echo "$c exit=$(DK inspect -f '{{.State.ExitCode}}' "$c" 2>/dev/null)" >> "$RUN/down_rc.txt"
    DK logs "$c" > "$RUN/docker-$c.log" 2>&1
    DK rm -f "$c" > /dev/null 2>&1; dmark "$c: removed"
  done
fi
# 2. host processes, last started first
if [ -s "$RUN/host_pids.txt" ]; then
  mapfile -t HP < <(tac "$RUN/host_pids.txt")
  for line in "${HP[@]}"; do
    set -- $line
    CAR_RC_FILE="$RUN/down_rc.txt" car_stop_pid "$1" "$2" "${3:-}"; dmark "$1 pid $2: stopped"
  done
fi
# 2b. in-stack mapping run (10-18, CAR_MAPPING=1): the stack has exited, so the runner's memory_final.json (map identity
# mapping_pending) and the saved database + export are on disk -> bind the memory file to that database
# (tools/car_run/mapping_memory_bind.py; docs/design/TELEOP_NAV_ARB_20261018.md section 2.4). Marks only: a failed
# bind never fails car_down (the pending file stays; memory_store bind by hand later).
if [ -f "$RUN/mapping_status.json" ]; then
  BIND_OUT=$(python3 "$HERE/mapping_memory_bind.py" "$RUN" 2>&1); BIND_RC=$?
  dmark "memory bind rc=$BIND_RC: $(echo "$BIND_OUT" | tr '\n' ' ')"
fi
# 3. GPU lock
LOCK_TOKEN=""
if [ -s "$RUN/gpu_lock.json" ]; then
  read -r LOCK_RES LOCK_TOKEN LOCK_USER < <(python3 -c "import json,sys;d=json.load(open(sys.argv[1]));print(d['resource'],d['token'],d['user'])" "$RUN/gpu_lock.json")
  if [ ! -e "$RUN/gpu_lock_released" ]; then
    if $CAR_LOCK release "$LOCK_RES" --token "$LOCK_TOKEN" -u "$LOCK_USER" > "$RUN/gpu_lock_release.txt" 2>&1; then
      date -Is > "$RUN/gpu_lock_released"; dmark "gpu lock $LOCK_RES released"
    else dmark "gpu lock $LOCK_RES release FAILED: $(tail -1 "$RUN/gpu_lock_release.txt")"; fi
  fi
fi
# 4. IPC directory (only our own: CAR_STATE_DIR/<tag>), pointer, source snapshot
IPC=$(cat "$RUN/ipc.txt" 2>/dev/null)
if [ -n "$IPC" ] && [ "$(dirname "$IPC")" = "$CAR_STATE_DIR" ] && [ "$(basename "$IPC")" = "$(basename "$RUN")" ] && [ -d "$IPC" ]; then
  rm -rf "$IPC"; dmark "ipc $IPC removed"
fi
if [ -L "$POINTER" ] && [ "$(readlink "$POINTER")" = "$RUN" ]; then rm -f "$POINTER"; dmark "pointer removed"; fi
[ "${KEEP_SRC:-0}" = 1 ] || rm -rf "$RUN/src"

# 5. residue check
CLEFT=(); PLEFT=()
if [ -s "$RUN/containers.txt" ]; then while read -r c; do exists "$c" && CLEFT+=("$c"); done < "$RUN/containers.txt"; fi
for f in "$RUN/host_pids.txt"; do   # (pids.txt: inside the stack container, gone with it)
  [ -s "$f" ] || continue
  while read -r n p s; do car_pid_alive "$p" "$s" && [ "$(car_pid_state "$p")" != Z ] && PLEFT+=("$n:$p"); done < "$f"
done
LOCK_HELD=false
if [ -n "$LOCK_TOKEN" ]; then $CAR_LOCK status 2>/dev/null | grep -q "$LOCK_TOKEN" && LOCK_HELD=true; fi
IPC_LEFT=false; [ -n "$IPC" ] && [ -e "$IPC" ] && IPC_LEFT=true
PTR_LEFT=false; [ -L "$POINTER" ] && [ "$(readlink "$POINTER")" = "$RUN" ] && PTR_LEFT=true
python3 - "$RUN/down.json" "${#CLEFT[@]}" "${#PLEFT[@]}" "$LOCK_HELD" "$IPC_LEFT" "$PTR_LEFT" "${REASON}" "${CLEFT[*]:-}" "${PLEFT[*]:-}" <<'PY'
import json, sys, time
out, nc, np_, lock, ipc, ptr, reason, cl, pl = sys.argv[1:]
d = {"time": time.strftime('%Y-%m-%dT%H:%M:%S%z'), "rollback_reason": reason or None,
     "containers_left": cl.split() if cl else [], "processes_left": pl.split() if pl else [],
     "gpu_lock_still_held": lock == 'true', "ipc_left": ipc == 'true', "pointer_left": ptr == 'true'}
d["clean"] = not (d["containers_left"] or d["processes_left"] or d["gpu_lock_still_held"] or d["ipc_left"] or d["pointer_left"])
json.dump(d, open(out, 'w'), indent=1)
print(json.dumps(d))
PY
python3 -c "import json,sys;sys.exit(0 if json.load(open(sys.argv[1]))['clean'] else 1)" "$RUN/down.json"
