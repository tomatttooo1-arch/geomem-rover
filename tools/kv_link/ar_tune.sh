#!/bin/bash
# Host side of the AR screen bring-up (2026-10-15). Layout knobs (A-tier: registers / PS only) live in
# the board's tune.json, which kv_server re-reads on change (no restart). Every call then dumps one
# frame and renders it to PNG on the host.
#   ar_tune.sh set <out> key=val ...   keys: depth_dx depth_dy mini_x mini_y slice_y max_age_ms
#                                      box_max (display only) box_nms_pct (IoU x100, -1 = off)
#                                      depth_stretch_pct (colour stretch k x100, 100 = off)
#                                      colour main view (bypass open): colour_main 0/1 colour_wait_ms
#                                      colour_hold_ms colour_depth_dx/dy pip_x/y colour_mini_x/y
#                                      (key=  removes it -> back to the default layout)
#   ar_tune.sh show                    -> current tune.json
#   ar_tune.sh shot <out> [tag]        -> dump one frame, pull, ar_dump_check --png-out
# Canvas width/height = the display mode: restart with ar_screen.sh start <out> <mode>.
# Env: KVSSH (default "ssh kv260"), SECRET, RESULTS.
set -eu
KVSSH=${KVSSH:-ssh kv260}
SECRET=${SECRET:-${FPGA_HOME}/dev1/.secrets/kv260_sudo}
RESULTS=${RESULTS:-${FPGA_SRC}/runs/ar-screen-20261015}
HERE=$(cd "$(dirname "$0")" && pwd)
B=/home/ubuntu/fpga/arscreen
cmd=${1:-}; shift || true
case "$cmd" in
show) $KVSSH "cat $B/tune.json" ;;
set)
  out=$1; shift
  cur=$($KVSSH "cat $B/tune.json 2>/dev/null || echo {}")
  new=$(python3 - "$cur" "$@" <<'PY'
import json, sys
t = json.loads(sys.argv[1] or "{}")
keys = ("depth_dx", "depth_dy", "mini_x", "mini_y", "slice_y", "max_age_ms", "box_max", "box_nms_pct", "depth_stretch_pct",
        "colour_main", "colour_wait_ms", "colour_hold_ms", "colour_depth_dx", "colour_depth_dy", "pip_x", "pip_y",
        "colour_mini_x", "colour_mini_y")
for kv in sys.argv[2:]:
    k, _, v = kv.partition("=")
    if k not in keys:
        sys.exit("unknown key %s (allowed: %s)" % (k, " ".join(keys)))
    if v == "":
        t.pop(k, None)
    else:
        t[k] = int(v)
print(json.dumps(t, sort_keys=True))
PY
)
  echo "tune.json <- $new"
  printf '%s\n' "$new" | $KVSSH "cat > $B/tune.json.tmp && mv $B/tune.json.tmp $B/tune.json"
  sleep 1.5
  "$0" shot "$out" "$(echo "$new" | tr -d '{}" ' | tr ':,' '=_')" ;;
shot)
  out=$1; tag=${2:-shot}
  ts=$(date +%H%M%S)
  $KVSSH "sudo -S -p '' bash $B/repo/tools/kv_link/ar_screen.sh dump $out" < "$SECRET"
  sleep 1.5
  d="$RESULTS/${ts}_${tag:0:60}"; mkdir -p "$d/dump" "$d/png"
  newest=$($KVSSH "ls -t $out/dump/*.json 2>/dev/null | head -1")
  [ -n "$newest" ] || { echo "no dump yet"; exit 1; }
  base=${newest%.json}
  for e in json fb depth grid desc; do $KVSSH "cat $base.$e" > "$d/dump/$(basename "$base").$e"; done
  # colour main view: a composited dump also has .fbpl (PL picture) and .rgb (colour), both needed by the checker
  for e in fbpl rgb; do $KVSSH "test -f $base.$e && cat $base.$e" > "$d/dump/$(basename "$base").$e" || rm -f "$d/dump/$(basename "$base").$e"; done
  python3 "$HERE/ar_dump_check.py" --dump "$d/dump" --png-out "$d/png" --json "$d/check.json" | tail -3 || true
  ls "$d/png"/*.png ;;
*) echo "usage: ar_tune.sh set <out> k=v ... | show | shot <out> [tag]"; exit 2 ;;
esac
