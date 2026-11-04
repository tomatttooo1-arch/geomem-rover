#!/bin/bash
# KV260 side of the AR screen bring-up (2026-10-15, Waveshare 7" HDMI, no EDID). Run as root (sudo -S).
#   prep                          -> loadapp m7_pl_merge_a2 (8-lane geometry core, BUILD_ID 0x00da5041, the
#                                    car chain's bitstream: car_up.sh KV_BUILD_ID), identity read back and
#                                    REFUSED (rc 3) unless it is 0x00da5041; m7_ar / m7_roi UIO listed.
#                                    2026-10-16: prep loaded m7_pl_merge (A1 single lane, 0x07784f75) before;
#                                    on a full-screen ground view its core took ~87 ms/frame (A2 ~11 ms est.)
#                                    and the AR picture went STALE (AR_COLOUR_MAIN_20261016.md section 10)
#   start <out> <mode> [src]      -> kv_server --ar drm --expect-build-id 0x00da5041 on <mode> (2026-10-16: the
#                                    server reads sysinfo BUILD_ID itself and refuses another bitstream; rc 3
#                                    REFUSED with its reason); mode = WxH from the connector list
#                                    or a CVT name below (1024x600_60, 1024x600R, 1024x600_50, 800x480_60) or
#                                    1024x600_edid (the panel's EDID timing forced, -hsync -vsync);
#                                    src = live|replay (default live); tune file = $B/tune.json
#   dump <out>                    -> SIGUSR1 to the helper: dump the next flipped render into <out>/dump
#   stop <out>                    -> SIGTERM kv_server (its pid only), desktop_enable is done by kv_server
#   restore                       -> desktop_enable, loadapp m7_pl, baseline identity
#   residue                       -> kv_server / m7ar_present left?
set -u
B=${AR_SCREEN_BASE:-/home/ubuntu/fpga/arscreen}     # override: tests only
REPO=$B/repo
APP=m7_pl_merge_a2
WANT_BUILD_ID=0x00da5041
# the A2 package's m7pl_test (byte-identical to the A1 merge package's one, PL_MERGE_A2_20261011.md section 4)
MERGE_TEST=${AR_SCREEN_M7PL_TEST:-$(find /home/ubuntu/fpga/pl-merge-a2-0da5041/pkg /home/ubuntu/fpga/pl-merge-7784f75/pkg \
  -name m7pl_test -type f 2>/dev/null | head -1)}
BASE_TEST=/home/ubuntu/fpga/package-100mhz/m7pl_test
modeline_of() {
  case "$1" in
    1024x600_60) echo "49000 1024 1072 1168 1312 600 603 613 624 -hsync +vsync" ;;
    # the Waveshare panel's own EDID timing (AR_SCREEN_BRINGUP section 3), forced: for a converter that reports
    # SINK_COUNT=0 / no EDID after a power cycle (2026-10-16); same canvas and timing as --ar-mode 1024x600 with EDID
    1024x600_edid) echo "49000 1024 1072 1168 1312 600 603 613 624 -hsync -vsync" ;;
    1024x600R)   echo "43750 1024 1072 1104 1184 600 603 613 619 +hsync -vsync" ;;
    1024x600_50) echo "39500 1024 1056 1152 1280 600 603 613 621 -hsync +vsync" ;;
    800x480_60)  echo "29500 800 824 896 992 480 483 493 500 -hsync +vsync" ;;
    *) echo "" ;;
  esac
}
cmd=${1:-}; shift || true
case "$cmd" in
prep)
  xmutil unloadapp >/dev/null 2>&1; xmutil loadapp $APP 2>&1 | tail -1; sleep 2
  chk=$(bash $REPO/tools/kv_link/kv_board.sh check "$MERGE_TEST")
  echo "$chk" | grep -E 'identity|m7_ar|m7_roi'
  bid=$(echo "$chk" | grep -o 'identity BUILD_ID=0x[0-9a-fA-F]*' | head -1 | grep -o '0x[0-9a-fA-F]*' | tr 'A-F' 'a-f')
  if [ "$bid" != "$WANT_BUILD_ID" ]; then
    echo "REFUSED: loaded bitstream BUILD_ID ${bid:-unread}, the AR screen needs $WANT_BUILD_ID ($APP)." \
         "Fix: is $APP installed (xmutil listapps; package ~/fpga/pl-merge-a2-0da5041/pkg)? m7pl_test found:" \
         "${MERGE_TEST:-none}. Then: xmutil unloadapp; xmutil loadapp $APP; rerun prep." >&2
    exit 3
  fi
  echo "DP $(cat /sys/class/drm/card0-DP-1/status 2>/dev/null)" ;;
start)
  out=$1; mode=$2; src=${3:-live}; shift 3 2>/dev/null || shift $#
  rm -rf "$out"; mkdir -p "$out/dump"
  ml=$(modeline_of "$mode")
  if [ -n "$ml" ]; then marg=(--ar-modeline "$ml"); else marg=(--ar-mode "$mode"); fi
  [ -f $B/tune.json ] || echo '{}' > $B/tune.json
  env FPGA_M7_DMA_PROVIDER=dmaheap FPGA_M7_DMA_HEAP=reserved bash $REPO/tools/kv_link/kv_board.sh start $REPO "$out" \
    --core pl --device devmem --wait irq --roi-core pl --port 47800 --max-idle-s 3600 --core-timeout-s 3 \
    --ar drm --ar-source "$src" "${marg[@]}" --ar-tune-file $B/tune.json \
    --ar-dump-dir "$out/dump" --ar-dump-seq 1 --expect-build-id $WANT_BUILD_ID "$@"
  rc=$?
  if [ $rc -ne 0 ] && grep -q -- '--expect-build-id' "$out/server.err" 2>/dev/null; then
    echo "REFUSED: kv_server did not start: $(grep -- '--expect-build-id' "$out/server.err" | tail -1)" \
         "Run ar_screen.sh prep (loads $APP) first." >&2
    exit 3
  fi
  exit $rc ;;
dump)
  out=$1
  hp=$(python3 -c "import json,sys;print([json.loads(l) for l in open(sys.argv[1]) if '\"ready\"' in l][0]['pid'])" "$out/ar_helper_events.jsonl")
  kill -USR1 "$hp" && echo "dump requested from helper $hp" ;;
stop)
  out=$1; bash $REPO/tools/kv_link/kv_board.sh stop "$out" | tail -1; chown -R ubuntu:ubuntu "$out" ;;
restore)
  xmutil desktop_enable >/dev/null 2>&1; echo "desktop_enable rc=$?"
  xmutil unloadapp >/dev/null 2>&1; xmutil loadapp m7_pl 2>&1 | tail -1; sleep 2
  "$BASE_TEST" --heap reserved info 2>&1 | grep -o -E 'BUILD_ID=0x[0-9a-fA-F]+|SLOT_MAP=0x[0-9a-fA-F]+' | head -2 ;;
residue)
  ps -eo pid,args | grep -E 'kv_server.py|m7ar_present|nano_replay' | grep -v grep; echo "residue-check done" ;;
*) echo "usage: ar_screen.sh prep|start|dump|stop|restore|residue"; exit 2 ;;
esac
