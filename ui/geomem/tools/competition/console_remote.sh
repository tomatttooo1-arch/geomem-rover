#!/usr/bin/env bash
# Nano-side helper for the one-click laptop launcher (start-directconsole.ps1), 10-21.
#   console_remote.sh status   -> "STACK READY <run>" | "STACK DOWN [stale <run>]", and "GATEWAY UP|DOWN"
#   console_remote.sh up       -> start the car stack (detached nano_up.sh, mapping mode) if it is not
#                                 running, wait for it (<= 6 min, progress lines), make sure the gateway runs.
#                                 A stale session (pointer left after a power cut) is car_down'ed first.
#   console_remote.sh down     -> car_down.sh for the current stack, stop the gateway
# Simple argv only, so the Windows side never has to quote a remote shell command.
set -u
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../../../.." && pwd)
current=/data/fpga/car/state/car.current
logdir=/home/robot/fpga/dev1
gw="$here/nano_gateway.sh"
hub_port=47801

# READY survives a power cut, so ask check_current.py (container running + hub answering) instead (10-22).
stack_ready() { python3 "$here/check_current.py" --current "$current" --hub-port "$hub_port" >/dev/null 2>&1; }
stack_recorded() { [ -e "$current" ] || [ -L "$current" ]; }
gateway_up() { bash "$gw" status 2>/dev/null | grep -q '^bridge: [0-9]'; }

case "${1:-status}" in
  status)
    if stack_ready; then echo "STACK READY $(basename "$(readlink -f "$current")")"
    elif stack_recorded; then echo "STACK DOWN stale $(basename "$(readlink "$current")") (cleaned up on start)"
    else echo "STACK DOWN"; fi
    if gateway_up; then echo "GATEWAY UP"; else echo "GATEWAY DOWN"; fi ;;
  up)
    if ! stack_ready; then
      if pgrep -f "competition/nano_up.sh" >/dev/null; then
        echo "a car stack start is already in progress; waiting for it"
        log=$(cat "$logdir/nano_up.latest" 2>/dev/null)
      else
        if stack_recorded; then
          echo "cleaning up the stale car session $(basename "$(readlink "$current")") first"
          ( cd "$repo" && timeout 150 bash tools/car_run/car_down.sh ) 2>&1 | tail -2 | cut -c1-160
          stack_recorded && { echo "STALE SESSION NOT CLEANED; see the run directory's down.json"; exit 1; }
        fi
        mkdir -p "$logdir"
        log="$logdir/nano_up-$(date +%Y%m%d-%H%M%S).log"
        echo "$log" >"$logdir/nano_up.latest"
        setsid nohup bash "$here/nano_up.sh" "$repo" mapping "$hub_port" /home/robot/.ssh/geomem-kv.conf \
          >"$log" 2>&1 </dev/null &
        echo "starting the car stack (log $log)"
      fi
      shown=0
      for _ in $(seq 1 180); do
        sleep 2
        if [ -n "${log:-}" ] && [ -f "$log" ]; then
          n=$(wc -l <"$log")
          if [ "$n" -gt "$shown" ]; then
            sed -n "$((shown + 1)),${n}p" "$log" | grep -v '^ *#' | grep -E '^ *ready |fail|FAIL|error|Error|refused|^AR_' | cut -c1-110
            shown=$n
          fi
          grep -q 'Car and display feed ready' "$log" && break
        fi
        if ! pgrep -f "competition/nano_up.sh" >/dev/null && ! stack_ready; then
          echo "CAR STACK FAILED TO START; last lines:"; tail -15 "${log:-/dev/null}" | cut -c1-160; exit 1
        fi
      done
      stack_ready || { echo "CAR STACK NOT READY after 6 min"; exit 1; }
    fi
    gateway_up || bash "$gw" start
    for _ in 1 2 3 4 5; do gateway_up && break; sleep 1; done
    gateway_up || { echo "GATEWAY FAILED; see /data/fpga/car/gateway/supervisor.log"; exit 1; }
    echo "STACK READY $(basename "$(readlink -f "$current")")" ;;
  down)
    if stack_ready || [ -e "$current" ]; then
      ( cd "$repo" && timeout 150 bash tools/car_run/car_down.sh ) 2>&1 | tail -2 | cut -c1-160
    else
      echo "no car stack running"
    fi
    bash "$gw" stop ;;
  *) echo "usage: $0 status|up|down" >&2; exit 2 ;;
esac
