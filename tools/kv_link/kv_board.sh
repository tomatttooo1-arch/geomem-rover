#!/bin/bash
# KV260 side of the B1 board runs (tools/kv_link). Run as root (sudo): /dev/mem, dma_heap,
# the UIO node and SO_RCVBUFFORCE all need it.
#   kv_board.sh check [<m7pl_test>]        -> BUILD_ID via that package's m7pl_test info + UIO names
#   kv_board.sh start <repo> <out> <kv_server args...>   -> detached server, writes <out>/pid and <out>/ready.json
#   kv_board.sh stop <out>                 -> SIGTERM the pid in <out>/pid (only that pid), wait for summary.json
# The server exits by itself after --max-idle-s without traffic, so a lost controller leaves nothing behind.
set -u
cmd=${1:-}; shift || true
case "$cmd" in
check)
  # Identity = sysinfo BUILD_ID as printed by the m7pl_test that belongs to the loaded
  # bitstream (controller 10-11: `xmutil listapps` is not reliable). Pass the package's own
  # binary: the merge-build m7pl_test also reads PERMIT/ENCODER MAGIC, and on 0x0117bc0a
  # nothing answers at those addresses (an unassigned AXI read can hang the bus).
  #   kv_board.sh check [<m7pl_test>]   -> "BUILD_ID=0x........" line + UIO names
  m7pl=${1:-}
  if [ -n "$m7pl" ]; then
    out=$("$m7pl" --heap reserved info 2>&1)
    echo "$out" | grep -E 'sysinfo MAGIC=|BUILD_ID' | head -2
    bid=$(echo "$out" | grep -o 'BUILD_ID=0x[0-9a-fA-F]*' | head -1)
    echo "identity ${bid:-BUILD_ID=unread}"
  else
    echo "identity not read (no m7pl_test given)"
  fi
  for u in /sys/class/uio/uio*; do [ -e "$u/name" ] && echo "$u $(cat $u/name) $(cat $u/maps/map0/addr 2>/dev/null)"; done
  ;;
start)
  repo=$1; out=$2; shift 2
  mkdir -p "$out"
  cd "$out" || exit 1
  env OPENBLAS_NUM_THREADS=1 PYTHONPATH="$repo" setsid nohup python3 "$repo/tools/kv_link/kv_server.py" --out "$out" "$@" \
      > "$out/server.out" 2> "$out/server.err" < /dev/null &
  echo $! > "$out/pid"
  for i in $(seq 1 100); do
    if [ -s "$out/server.out" ]; then head -1 "$out/server.out" > "$out/ready.json"; cat "$out/ready.json"; exit 0; fi
    if ! kill -0 "$(cat "$out/pid")" 2>/dev/null; then echo "server died:"; tail -20 "$out/server.err"; exit 1; fi
    sleep 0.2
  done
  echo "server not ready after 20 s"; tail -20 "$out/server.err"; exit 1
  ;;
stop)
  out=$1
  # the server's own pid (printed by kv_server.py), not $! of the launcher
  pid=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['pid'])" "$out/ready.json")
  if kill -0 "$pid" 2>/dev/null; then kill -TERM "$pid"; fi
  for i in $(seq 1 50); do kill -0 "$pid" 2>/dev/null || break; sleep 0.2; done
  kill -0 "$pid" 2>/dev/null && { echo "pid $pid still alive"; exit 1; }
  ls -l "$out/summary.json" && tail -2 "$out/server.out"
  ;;
*)
  echo "usage: kv_board.sh check | start <repo> <out> <args...> | stop <out>"; exit 2;;
esac
