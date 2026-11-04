#!/usr/bin/env bash
# Nano direct-LAN operator gateway (10-21). Read-only towards the car stack: it never starts/stops the
# stack and sends no motion command by itself; the page's buttons go through the hub as before.
#
#   bash nano_gateway.sh start|stop|status      (start is idempotent; nano_up.sh calls it)
#   bash nano_gateway.sh run                     (foreground supervisor loop)
#
# Follows /data/fpga/car/state/car.current: when READY, runs ui/geomem/bridge/geomem_bridge.py on
# 127.0.0.1:8790 with the session token, serving the operator page (ui/geomem/operator), pushing
# preview frames and the RTAB-Map display cloud over its WebSocket. Restarts it on a new session,
# stops it when the stack is down. Laptop side: one `ssh -N -L 127.0.0.1:8788:127.0.0.1:8790 robot@<nano>`
# then http://127.0.0.1:8788/#remote (operator/portable-team/start-directconsole.ps1 does this).
set -u
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../../../.." && pwd)
state=${GEOMEM_GATEWAY_DIR:-/data/fpga/car/gateway}
current=/data/fpga/car/state/car.current
listen_port=${GEOMEM_GATEWAY_PORT:-8790}
public_port=${GEOMEM_GATEWAY_PUBLIC_PORT:-8788}
mapping_feed=${GEOMEM_MAPPING_FEED:-127.0.0.1:47802}
mkdir -p "$state/basemaps"
umask 077

log() { echo "$(date '+%F %T') $*" >>"$state/supervisor.log"; }
cmd_of() { tr '\0' ' ' <"/proc/$1/cmdline" 2>/dev/null; }
supervisor_alive() {
  local p; p=$(cat "$state/supervisor.pid" 2>/dev/null) || return 1
  cmd_of "$p" | grep -q "nano_gateway.sh run"
}
bridge_alive() {
  local p; p=$(cat "$state/bridge.pid" 2>/dev/null) || return 1
  cmd_of "$p" | grep -q geomem_bridge.py
}
stop_bridge() {
  if bridge_alive; then kill "$(cat "$state/bridge.pid")"; log "bridge stopped ($1)"; fi
  rm -f "$state/bridge.pid" "$state/session.txt"
}

# sleep in the background and wait: bash runs the TERM trap only after a foreground child exits, so a plain
# `sleep 10` kept a stopped supervisor (and its pid file) alive for up to 10 s (10-22: stop then start said
# "already running" and nothing came up)
nap() { sleep "$1" & wait $!; }

run() {
  echo $$ >"$state/supervisor.pid"
  trap 'stop_bridge "supervisor exit"; rm -f "$state/supervisor.pid"; exit 0' INT TERM
  log "supervisor start pid $$ repo $repo"
  while true; do
    if [ -f "$current/READY" ] && [ -f "$current/token.txt" ]; then
      run_name=$(basename "$(readlink -f "$current")")
      if [ "$(cat "$state/session.txt" 2>/dev/null)" != "$run_name" ] || ! bridge_alive; then
        stop_bridge "session $run_name"
        cp "$current/token.txt" "$state/token.txt"
        # exec chain: $! is the python process itself (a forking wrapper here made the check below fail)
        ( cd "$repo" && exec python3 ui/geomem/bridge/geomem_bridge.py --hub "127.0.0.1:47801" \
            --token-file "$state/token.txt" --listen "127.0.0.1:$listen_port" --public-port "$public_port" \
            --app-root ui/geomem/operator --mapping-feed "$mapping_feed" --basemap-watch "$state/basemaps" --gateway \
            --stats-out "$state/bridge-stats-$run_name.json" >>"$state/bridge.log" 2>&1 </dev/null ) &
        echo $! >"$state/bridge.pid"
        nap 2
        if bridge_alive; then
          echo "$run_name" >"$state/session.txt"
          log "bridge started for $run_name pid $(cat "$state/bridge.pid")"
        else
          log "bridge failed to start for $run_name (see bridge.log); retry in 10 s"
          rm -f "$state/bridge.pid"
          nap 10
          continue
        fi
      fi
    elif [ -f "$state/session.txt" ]; then
      stop_bridge "stack down"
    fi
    nap 2
  done
}

case "${1:-status}" in
  run) run ;;
  start)
    if supervisor_alive; then echo "gateway already running (pid $(cat "$state/supervisor.pid"))"; exit 0; fi
    # 10-26: start the supervisor with SIGTERM/SIGINT at their defaults and nothing blocked. bash cannot
    # trap a signal that was already ignored when it started, so a caller that ignores SIGTERM (a Python
    # parent with SIG_IGN, `nohup`-style wrappers, some ssh/service launchers) left a supervisor that
    # `stop` could only SIGKILL after 15 s (its exit trap never ran; seen in the full unittest run).
    # python3 is already required for the bridge.
    setsid nohup python3 -c 'import os, signal, sys
for s in (signal.SIGTERM, signal.SIGINT):
    signal.signal(s, signal.SIG_DFL)
signal.pthread_sigmask(signal.SIG_SETMASK, [])
os.execvp("bash", ["bash"] + sys.argv[1:])' "$here/nano_gateway.sh" run >/dev/null 2>&1 </dev/null &
    for _ in 1 2 3 4 5 6; do sleep 1; bridge_alive && break; done
    if supervisor_alive; then
      echo "gateway started (pid $(cat "$state/supervisor.pid")); bridge: $(bridge_alive && echo up || echo 'waiting for READY')"
    else
      echo "gateway failed to start; see $state/supervisor.log" >&2; exit 1
    fi ;;
  stop)
    # synchronous: returns only when supervisor and bridge are gone, so `stop; start` really restarts
    if ! supervisor_alive && ! bridge_alive; then echo "gateway not running"; exit 0; fi
    sup=$(supervisor_alive && cat "$state/supervisor.pid"); br=$(bridge_alive && cat "$state/bridge.pid")
    [ -n "$sup" ] && kill "$sup" 2>/dev/null
    [ -z "$sup" ] && [ -n "$br" ] && kill "$br" 2>/dev/null   # orphaned bridge without a supervisor
    for _ in $(seq 1 ${GEOMEM_GATEWAY_STOP_WAIT_S:-15}0); do
      supervisor_alive || bridge_alive || break; sleep 0.1
    done
    if supervisor_alive || bridge_alive; then
      for p in $sup $br; do kill -KILL "$p" 2>/dev/null; done
      log "stop: SIGKILL after ${GEOMEM_GATEWAY_STOP_WAIT_S:-15} s"
    fi
    rm -f "$state/supervisor.pid" "$state/bridge.pid" "$state/session.txt"
    echo "gateway stopped" ;;
  status)
    echo "supervisor: $(supervisor_alive && cat "$state/supervisor.pid" || echo down)"
    echo "bridge: $(bridge_alive && cat "$state/bridge.pid" || echo down)  session: $(cat "$state/session.txt" 2>/dev/null || echo -)"
    tail -3 "$state/supervisor.log" 2>/dev/null ;;
  *) echo "usage: $0 start|stop|status|run" >&2; exit 2 ;;
esac
