#!/bin/bash
# GOLIVE0 (2026-11-04): the whole single-board product on the KV260, one script, runs as root from the staged dir
#
#   kv_golive.sh start     load the bitstream (unless that app is already active), then start, each with its own pid:
#                            kv_cam    KV's own RGB-D camera (Gemini 2 on the KV USB3) -> shared-memory ring
#                            kv_server depth chain on the PL: geometry core -> fusion + GRID cost map -> planner field
#                            dpu_rx    recognition: PL pre-processing core -> DPU (VART) -> post-processing -> boxes
#                            kv_uplink boxes + map/field status -> the Nano task console (UDP, 2 Hz)
#   kv_golive.sh stop      stop them by pid (no pkill), give the display back (gdm) unless GDM_RESTORE=0, chain report
#   kv_golive.sh status    one status line per process + the newest uplink line
#
# Nothing here refuses to run (FPGA rule): a missing piece is WARN and the chain goes on with the next best thing.
# NO native compile of our programs on the board (09:28 hang was while linking dpu_rx on the board): dpu_rx,
# libkv_nav, libplanner_drv come prebuilt in xbin/ (host cross build) or from the board's earlier build. kv_server
# still compiles its small one-file helpers (m7_frame.so etc.) at start, as every car start since September.
#
# Env (set by start_all.sh; defaults in brackets):
#   APP        bitstream app name               [m7_pl_m5_166; start_all.sh default BIT=m7pre passes m7_pl_m7s17]   (package in pkg/, BUILD_ID in pkg/SELFCHECK.txt)
#   PREP       soft | pl                         [pl]    NAV  kv_server nav args  [--nav pl --nav-core kv_nav]
#   RECOG_MODEL y8s640 | y8s512                  [y8s640]  needs xbin/dpu_rx_640; else falls back to y8s512 + DPU512_BIN
#   POST_ARGS  extra dpu_rx args, e.g. "--post pl" when the binary has the PL post-processing core path  []
#   PLN        m5 | gmap3  [gmap3 since GOLIVE1]  PLNCHAIN0: gmap3 = kv_nav's planner hook uses the GMAP3 core (in-core path fetch,
#              400x400 global map MERGE/SCAN) via xbin/libplanner_drv_gmap3.so, only when the loaded BUILD_ID is in
#              PLN_IDS; otherwise / library missing / core self-test failing: WARN and the m5 driver (as before)
#   PLN_IDS    BUILD_IDs whose planner is GMAP3 v9 (impl-m6a-mk4-setp-m7s17-63ba99b, impl-m6a-mk4-m7pre0-6661def)  [0x063ba99b 0x06661def]
#   NANO       Nano address:port for the uplink  [198.51.100.1:47950]
#   RUN_S      seconds the samplers run (0 = 3600); the chain itself runs until stop (GOLIVE0 run 1: a timed dpu_rx
#              ended its own run at 300 s before the stop)  [0]
#   LOAD_MODE  bringup (package board_bringup.sh, default) | xmutil (app already installed on the board; start_all.sh default BIT=m7pre passes xmutil)
#   WANT_ID    expected BUILD_ID when the package's SELFCHECK.txt is not the loaded app's
#   MEASURE    1 = also CPU (1 Hz) and SOM power (INA260, 10 Hz) samplers for RUN_S or 3600 s  [1]
#   SRCH       pl | nano  probabilistic search (SRCHCHAIN0) [pl]: pl = kv_srch (tools/golive/srch_chain.py serve) runs
#              the search on the PL search core m7_srch (xbin/libsrch_drv.so, scene srch_scene.npz) and the uplink
#              carries its look order to the Nano; no m7_srch node / core error / self-check mismatch -> WARN and
#              the Nano console runs the software search (old path). nano = kv_srch not started.
#   SRCH_PERIOD  seconds between search episodes  [10]
#   MEM        nano | pl  [pl since GOLIVE1]  MEMCHAIN0: pl = the object memory runs on the KV (tools/golive/kv_memsvc.py), on
#              the PL memory core (slot 14) when the loaded BUILD_ID is in MEMK_IDS, else (WARN) on the core's C model
#              (xbin/libmemk0.so, host cross build); kv_uplink sends it to the Nano as msg["mem"]
#   MEMK_IDS   BUILD_IDs that carry the memory core (MEMK4, bram layout)  [0x063ba99b 0x06661def]
#   CHASSIS    off | kv | nano  [off]  off = bench run; kv = KV260 chassis bridge: kv_server opens CHASSIS_PORT
#              [/dev/fpga_chassis], sends velocity over USB serial and reads odometry back (host/kv_chassis_bridge.py;
#              a missing port is a WARN and the pose stays static); nano = compatibility with the earlier Nano-side
#              chassis stack. CHASSIS_ODOM_ARGS overrides
#              kv_server --chassis-odom-args. CHASSIS_V_LIMIT / CHASSIS_W_LIMIT [150 mm/s / 500 mrad/s: the first
#              on-car tests] clip every command frame. LOCAL_PARAMS [tools/nn_cam2/params_car_bench.json, goal 2 m
#              ahead]; first lifted-wheel test: tools/kv_chassis/params_lift_test.json (goal 0.3 m ahead).
set -u
B=${B:-$(cd "$(dirname "$0")/.." && pwd)}
O=$B/out; mkdir -p "$O"
P=$O/progress.txt
APP=${APP:-m7_pl_m5_166}; PREP=${PREP:-pl}; NAV=${NAV:---nav pl --nav-core kv_nav}
RECOG_MODEL=${RECOG_MODEL:-y8s640}; POST_ARGS=${POST_ARGS:---post pl}; NANO=${NANO:-198.51.100.1:47950}
RUN_S=${RUN_S:-0}; MEASURE=${MEASURE:-1}; GDM_RESTORE=${GDM_RESTORE:-1}
SRCH=${SRCH:-pl}; SRCH_PERIOD=${SRCH_PERIOD:-10}
MEM=${MEM:-pl}; MEMK_IDS=${MEMK_IDS:-"0x063ba99b 0x06661def"}; FIND=${FIND:-bottle}
PROCS="kv_cam kv_server dpu_rx kv_memsvc kv_srch kv_uplink"
CHASSIS=${CHASSIS:-off}; CHASSIS_PORT=${CHASSIS_PORT:-/dev/fpga_chassis}
case "$CHASSIS" in off|kv|nano) ;; *) echo "WARN CHASSIS='$CHASSIS' is not off|kv|nano -> off" >&2; CHASSIS=off;; esac
CHASSIS_ARGS=(--chassis "$CHASSIS")
[ "$CHASSIS" = kv ] && CHASSIS_ARGS+=(--chassis-port "$CHASSIS_PORT" --chassis-record "$O/chassis_record.jsonl")
[ "$CHASSIS" = kv ] && [ -n "${CHASSIS_ODOM_ARGS:-}" ] && CHASSIS_ARGS+=(--chassis-odom-args "$CHASSIS_ODOM_ARGS")
[ "$CHASSIS" = kv ] && CHASSIS_ARGS+=(--chassis-v-limit-mm-s "${CHASSIS_V_LIMIT:-150}" --chassis-w-limit-mrad-s "${CHASSIS_W_LIMIT:-500}")
LOCAL_PARAMS=${LOCAL_PARAMS:-tools/nn_cam2/params_car_bench.json}   # its nav_goal is 2 m ahead: with CHASSIS=kv the car DRIVES there
DPU512_BIN=${DPU512_BIN:-/home/ubuntu/smoke5_dev1/build/dpu_pipe/dpu_rx}; DPU640_BIN=${DPU640_BIN:-$B/xbin/dpu_rx_640}
R=$B/repo; CAMBIN=$B/kvcam1/bin; RING=kv_cam_v1; PY=python3
export LD_LIBRARY_PATH=$B/kvcam1/lib:${LD_LIBRARY_PATH:-}
export KVCAM_LIB=$B/kvcam1/lib/libcamring.so   # kv_server ring reader (E2E0 try 1: not found -> retried forever)
NAVP=${KV_NAV_PARAMS:-'{"stuck_window": 1e9}'}   # bench: no stuck detection; '{"stuck_window": 1e9, "replan_period": 0}' = replan every tick
PLN=${PLN:-gmap3}; PLN_IDS=${PLN_IDS:-"0x063ba99b 0x06661def"}
WORKERS=${WORKERS:-1}   # product default: 1 inference thread (controller 11-04: 640 157 ms / 6.8 fps vs 2 threads 286 ms / 7.5 fps)
ph() { echo "== PHASE $1 $(awk '{print $1}' /proc/uptime) $(date +%FT%T%z)" | tee -a "$P"; sync; }
warn() { echo "WARN: $*" | tee -a "$P" >&2; sync; }
note() { echo "$*" | tee -a "$P"; }
alive() { [ -n "${1:-}" ] && [ -d "/proc/$1" ]; }   # works for a non-root status reader too
pidof_f() { cat "$O/$1.pid" 2>/dev/null; }
SECS=$RUN_S; [ "$SECS" = 0 ] && SECS=10000000

do_start() {
  : > "$P"
  ph snap
  note "boot $(uptime -s)"
  { uptime; free -m; ps -eo pid,comm,args | grep -E "kv_server|dpu_rx|kv_cam|kv_uplink" | grep -v grep; lsusb | grep -i -E "orbbec|2bc5";
    xmutil listapps; ls -l /dev/dpu; cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null; } > "$O/snap.txt" 2>&1
  for n in $PROCS; do alive "$(pidof_f $n)" && warn "$n already running (pid $(pidof_f $n)): run stop first"; done
  [ -f "$B/SHA256SUMS" ] && { (cd "$B" && sha256sum -c --quiet SHA256SUMS) > "$O/sha_check.txt" 2>&1 || warn "stage sha mismatch (sha_check.txt)"; }

  ph unpack
  rm -rf "$R" "$B/kvcam1"; tar xf "$B/repo.tar" -C "$B"; tar xzf "$B/kvcam1.tar.gz" -C "$B"
  PD=$R/hw/planner/drv/build/aarch64; mkdir -p "$PD" "$R/tools/kv_nav/build/native-aarch64/host" "$R/tools/dpu_pipe/build"
  cp "$B/xbin/libplanner_drv.so" "$PD/" && cp "$B/xbin/libkv_nav.so" "$R/tools/kv_nav/build/native-aarch64/host/" \
    && note "BIN_OK libplanner_drv libkv_nav (host cross build)" || warn "cross libs missing (planner falls back to soft)"
  SD=$R/hw/srch/drv/build/aarch64; mkdir -p "$SD"
  [ "$SRCH" = pl ] && { cp "$B/xbin/libsrch_drv.so" "$SD/" && note "BIN_OK libsrch_drv sha=$(sha256sum "$SD/libsrch_drv.so" | cut -c1-16)" \
    || warn "xbin/libsrch_drv.so missing (search falls back to the Nano)"; }
  if [ "$PLN" = gmap3 ]; then   # PLNCHAIN0: the GMAP3 chain library next to the m5 one
    G3D=$R/hw/planner/drv_gmap3_chain/build/aarch64; mkdir -p "$G3D"
    cp "$B/xbin/libplanner_drv_gmap3.so" "$G3D/" && note "BIN_OK libplanner_drv_gmap3 sha=$(sha256sum "$G3D/libplanner_drv_gmap3.so" | cut -c1-16)" \
      || warn "PLN=gmap3 but no xbin/libplanner_drv_gmap3.so: m5 planner driver"
  fi
  if [ "$RECOG_MODEL" = y8s640 ] && [ ! -x "$DPU640_BIN" ]; then
    warn "no 640 dpu_rx ($DPU640_BIN): recognition falls back to y8s512 with $DPU512_BIN"; RECOG_MODEL=y8s512
  fi
  BIN=$DPU512_BIN; [ "$RECOG_MODEL" = y8s640 ] && BIN=$DPU640_BIN
  if [ -x "$BIN" ]; then cp "$BIN" "$R/tools/dpu_pipe/build/dpu_rx"
    note "BIN dpu_rx=$BIN $(stat -c %s "$BIN") B sha=$(sha256sum "$BIN" | cut -c1-16) not_found=$(ldd "$BIN" | grep -c 'not found')"
  else warn "no dpu_rx binary at $BIN: recognition not started"; fi
  systemctl stop gdm 2>/dev/null; sleep 2; note "gdm: $(systemctl is-active gdm 2>&1)"

  ph load
  WANT=$(sed -n 's/^expected sysinfo BUILD_ID on the board: \(0x[0-9a-fA-F]*\).*/\1/p' "$B/pkg/SELFCHECK.txt" | head -1)
  # an app already active is not reloaded (fewer loads = fewer chances to hang); the active app is read from
  # xmutil (no PL register read before we know which bitstream is in)
  [ -n "${WANT_ID:-}" ] && WANT=$WANT_ID
  # Is the wanted bitstream already in? xmutil listapps shows Active_slot -1 even right after board_bringup's load
  # (seen 11-04 on m5), so it cannot tell. The sysinfo UIO node exists only while one of our overlays is in; only
  # then is the BUILD_ID register read (a PL read with no overlay loaded could hang the bus).
  GOT0=
  grep -qx m7_sysinfo /sys/class/uio/*/name 2>/dev/null && \
    GOT0=$("$B/pkg/m7pl_test" --heap reserved info 2>&1 | sed -n 's/.*BUILD_ID=\(0x[0-9a-fA-F]*\).*/\1/p' | head -1)
  if [ -n "$GOT0" ] && [ -n "$WANT" ] && [ "$((GOT0))" = "$((WANT))" ]; then
    note "BUILD_ID $GOT0 already loaded: not reloaded"
  elif [ "${LOAD_MODE:-bringup}" = xmutil ]; then     # an app already installed on the board (fallback, e.g. v3fc2)
    { xmutil unloadapp; sleep 2; xmutil loadapp "$APP"; } > "$O/loadapp.txt" 2>&1; tail -2 "$O/loadapp.txt" >> "$P"
    # MEMCHAIN0: the first xmutil load after m5 can report "load Error: -1" (BOARD3, M7BOARD0): one retry
    if grep -q "load Error" "$O/loadapp.txt"; then
      warn "xmutil loadapp $APP failed once: one retry"
      { xmutil unloadapp; sleep 2; xmutil loadapp "$APP"; } >> "$O/loadapp.txt" 2>&1; tail -2 "$O/loadapp.txt" >> "$P"
    fi
    # S_AXI_LPD prerequisite as the m5 package's board_bringup does (M7BOARD0 ran it on m7pre0c1: no change needed)
    [ -f "$B/pkg/m5_lpd_afi.py" ] && $PY "$B/pkg/m5_lpd_afi.py" --json "$O/lpd_afi.json" >> "$P" 2>&1
  else
    (cd "$B/pkg" && M7PL_APP=$APP ./board_bringup.sh 0 1 > "$O/bringup.txt" 2>&1); cp -r "$B"/pkg/bringup-* "$O/" 2>/dev/null
    tail -3 "$O/bringup.txt" >> "$P"
  fi
  GOT=$("$B/pkg/m7pl_test" --heap reserved info 2>&1 | sed -n 's/.*BUILD_ID=\(0x[0-9a-fA-F]*\).*/\1/p' | head -1)
  note "BUILD_ID want=$WANT got=$GOT app=$APP bit.bin.sha256=$(sha256sum /lib/firmware/xilinx/$APP/$APP.bit.bin 2>/dev/null | cut -c1-16)"
  [ -n "$GOT" ] && [ "$((GOT))" = "$((WANT))" ] || warn "BUILD_ID mismatch or unreadable: chain started anyway, numbers are NOT $APP numbers"
  [ -e /dev/dpu ] || warn "/dev/dpu missing (bitstream without DPU?): recognition will fail to open the DPU"

  ph cam
  timeout 20 "$CAMBIN/kv_cam" --list > "$O/kv_cam_list.txt" 2>&1
  nohup "$CAMBIN/kv_cam" --name "$RING" --cpu 1 --nice -5 --alarm-log "$O/kv_cam_alarms.jsonl" \
    > "$O/kv_cam.out" 2> "$O/kv_cam.err" < /dev/null &
  echo $! > "$O/kv_cam.pid"; sleep 5
  alive "$(pidof_f kv_cam)" && note "kv_cam pid $(pidof_f kv_cam)" || warn "kv_cam exited: $(tail -2 "$O/kv_cam.err" | tr '\n' ' ')"

  ph depth
  # shellcheck disable=SC2086
  # cd first, then start the server itself in the background: "(cd R && nohup X &)" backgrounds the whole list,
  # $! is then a subshell and the python server outlives a stop by pid (bench runs 1-3, 11-04)
  # PLNCHAIN0: the GMAP3 driver only on a bitstream whose planner is GMAP3 (writes to 0x90/0x98 and map ops)
  KPLN=m5
  if [ "$PLN" = gmap3 ]; then
    for id in $PLN_IDS; do [ -n "$GOT" ] && [ "$((GOT))" = "$((id))" ] && KPLN=gmap3; done
    [ "$KPLN" = gmap3 ] && note "PLN=gmap3 (BUILD_ID $GOT)" || warn "PLN=gmap3 but BUILD_ID $GOT not in PLN_IDS=$PLN_IDS: m5 planner driver"
  fi
  cd "$R"; KV_NAV_PARAMS="$NAVP" KV_NAV_PLN=$KPLN nohup $PY tools/kv_link/kv_server.py \
     --core pl --device devmem --wait irq --roi-core pl --rolling pl --rolling-grid pl --kv-fast native $NAV \
     --port 47800 --max-idle-s 864000 --frame-source kvcam --kvcam-name "$RING" \
     --local-params "$LOCAL_PARAMS" --stamp-log "$O/depth.jsonl" --out "$O/kv_server" \
     "${CHASSIS_ARGS[@]}" \
     > "$O/kv_server.out" 2> "$O/kv_server.err" < /dev/null &
  echo $! > "$O/kv_server.pid"; cd "$B"
  mkdir -p "$O/kv_server"; sleep 10
  alive "$(pidof_f kv_server)" && note "kv_server pid $(pidof_f kv_server): $(head -c 300 "$O/kv_server.out" | tr '\n' ' ')" \
    || warn "kv_server exited: $(tail -3 "$O/kv_server.err" | tr '\n' ' ')"

  ph recog
  eval "$(cd "$R" && RECOG_MODEL=$RECOG_MODEL RECOG_MODEL_DIR=$B/models $PY tools/dpu_pipe/recog/recog_args.py --format env)" \
    || warn "recog_args.py"
  # the board's earlier 512 build (10ec896, before RECOG640) has no --nc / --model-tag: legacy output args only
  [ "$BIN" = "$DPU512_BIN" ] && RECOG_POST_ARGS=
  note "recog model=$RECOG_MODEL xmodel=${RECOG_XMODEL:-} size=${RECOG_SIZE:-} fix=${RECOG_FIX:-} prep=$PREP post_args='${RECOG_POST_ARGS:-} $POST_ARGS'"
  if [ -x "$R/tools/dpu_pipe/build/dpu_rx" ]; then
    # shellcheck disable=SC2086
    nohup "$R/tools/dpu_pipe/build/dpu_rx" --frame-source kvcam --kvcam-name "$RING" --engine vart \
      --xmodel "${RECOG_XMODEL:-}" ${RECOG_POST_ARGS:-} $POST_ARGS --size "${RECOG_SIZE:-512}" --fix "${RECOG_FIX:-7}" \
      --workers "$WORKERS" --inflight "$WORKERS" --prep "$PREP" --seconds 10000000 --warm-s 5 --stamp-log "$O/colour.jsonl" \
      --out-log "$O/boxes.jsonl" > "$O/dpu_rx.json" 2> "$O/dpu_rx.err" < /dev/null &
    echo $! > "$O/dpu_rx.pid"; sleep 8
    alive "$(pidof_f dpu_rx)" && note "dpu_rx pid $(pidof_f dpu_rx)" || warn "dpu_rx exited: $(tail -3 "$O/dpu_rx.err" | tr '\n' ' ')"
  fi

  SRCH_UP=
  if [ "$SRCH" = pl ]; then
    ph srch
    # SRCHCHAIN0: search episodes on the PL search core; never refuses (no core -> status impl fallback_nano)
    nohup nice -n 10 $PY "$R/tools/golive/srch_chain.py" serve --scene "$B/srch_scene.npz" --impl pl \
      --drv "$SD/libsrch_drv.so" --period "$SRCH_PERIOD" --out "$O/srch.jsonl" --status "$O/srch_status.json" \
      > "$O/kv_srch.out" 2> "$O/kv_srch.err" < /dev/null &
    echo $! > "$O/kv_srch.pid"; SRCH_UP="--srch-status $O/srch_status.json"; sleep 3
    alive "$(pidof_f kv_srch)" && note "kv_srch pid $(pidof_f kv_srch): $(head -c 200 "$O/kv_srch.out" | tr '\n' ' ')" \
      || warn "kv_srch exited: $(tail -2 "$O/kv_srch.err" | tr '\n' ' ') (Nano console takes the search over)"
    grep -h WARN "$O/kv_srch.err" 2>/dev/null | head -2 | while read -r l; do warn "kv_srch: $l"; done
  fi
  MEMARGS=()
  if [ "$MEM" = pl ]; then
    ph mem
    # the core is touched only when the loaded bitstream is one that has it (a /dev/mem access to an empty slot can
    # hang the bus); otherwise the C model on the CPU (WARN, never refuse)
    MB=cmodel
    for id in $MEMK_IDS; do [ -n "$GOT" ] && [ "$((GOT))" = "$((id))" ] && MB=pl; done
    [ "$MB" = pl ] || warn "BUILD_ID $GOT has no memory core (MEMK_IDS=$MEMK_IDS): object memory on the C model"
    mkdir -p "$R/build/memk0"; cp "$B/xbin/libmemk0.so" "$R/build/memk0/" 2>/dev/null || warn "no xbin/libmemk0.so (C model fallback unavailable)"
    cd "$R"; MEMK0_LIB_DIR=$R/build/memk0 nohup $PY tools/golive/kv_memsvc.py --backend $MB --feed ring \
      --boxes "$O/boxes.jsonl" --names "$R/tools/dpu_pipe/recog/profiles.json" --ring-name "$RING" \
      ${MEM_CAM_PARAMS:+--cam-params "$MEM_CAM_PARAMS"} --find "$FIND" --out "$O/mem.json" \
      > "$O/kv_memsvc.out" 2> "$O/kv_memsvc.err" < /dev/null &
    echo $! > "$O/kv_memsvc.pid"; cd "$B"; sleep 3
    alive "$(pidof_f kv_memsvc)" && note "kv_memsvc pid $(pidof_f kv_memsvc) backend=$MB: $(head -c 200 "$O/kv_memsvc.out")" \
      || warn "kv_memsvc exited: $(tail -2 "$O/kv_memsvc.err" | tr '\n' ' ')"
    MEMARGS=(--mem-state "$O/mem.json")
  fi

  ph uplink
  # shellcheck disable=SC2086
  nohup $PY "$R/tools/golive/kv_uplink.py" --depth "$O/depth.jsonl" --boxes "$O/boxes.jsonl" \
    --colour "$O/colour.jsonl" --names "$R/tools/dpu_pipe/recog/profiles.json" --nano "$NANO" --log "$O/uplink.jsonl" $SRCH_UP \
    "${MEMARGS[@]}" \
    > "$O/kv_uplink.out" 2> "$O/kv_uplink.err" < /dev/null &
  echo $! > "$O/kv_uplink.pid"; sleep 2
  alive "$(pidof_f kv_uplink)" && note "kv_uplink pid $(pidof_f kv_uplink) -> $NANO" || warn "kv_uplink exited: $(tail -2 "$O/kv_uplink.err")"

  if [ "$MEASURE" = 1 ]; then
    MS=$RUN_S; [ "$MS" = 0 ] && MS=3600
    NAMES=(); for n in $PROCS; do p=$(pidof_f $n); [ -n "$p" ] && NAMES+=(--pid "$n=$p"); done
    KP=$(pgrep -P "$(pidof_f kv_server)" -n 2>/dev/null); [ -n "$KP" ] && NAMES+=(--pid "kv_server_child=$KP")
    nohup $PY "$R/tools/nn_cam2/cpu_sample.py" --out "$O/cpu.jsonl" --period 1 --seconds "$MS" "${NAMES[@]}" \
      > /dev/null 2> "$O/cpu_sample.err" < /dev/null & echo $! > "$O/cpu_sample.pid"
    [ -f "$B/pwr_sample.py" ] && { nohup $PY "$B/pwr_sample.py" golive "$MS" 10 "$O/pwr.csv" > "$O/pwr.out" 2>&1 < /dev/null &
                                   echo $! > "$O/pwr_sample.pid"; }
  fi
  ph running
}

do_stop() {
  ph stop
  note "boot $(uptime -s)"
  for n in kv_uplink kv_memsvc kv_srch dpu_rx kv_server kv_cam cpu_sample pwr_sample; do
    p=$(pidof_f $n); [ -n "$p" ] && for c in $(pgrep -P "$p" 2>/dev/null); do kill -TERM "$c"; note "TERM $n child pid $c"; done
    alive "$p" && { kill -TERM "$p"; note "TERM $n pid $p"; }
  done
  sleep 6
  for n in kv_uplink kv_memsvc kv_srch dpu_rx kv_server kv_cam cpu_sample pwr_sample; do
    p=$(pidof_f $n); alive "$p" && { kill -KILL "$p"; warn "KILL $n pid $p (no exit after TERM)"; }; rm -f "$O/$n.pid"
  done
  [ "$GDM_RESTORE" = 1 ] && { systemctl start gdm 2>/dev/null; note "gdm: $(systemctl is-active gdm 2>&1)"; }
  ST=(); [ -s "$O/depth.jsonl" ] && ST+=("$O/depth.jsonl"); [ -s "$O/colour.jsonl" ] && ST+=("$O/colour.jsonl")
  [ ${#ST[@]} -gt 0 ] && $PY "$R/tools/nn_cam2/chain_report.py" "${ST[@]}" --warm-s 10 --json "$O/chain.json" > "$O/chain_report.txt" 2>&1
  [ -s "$O/cpu.jsonl" ] && $PY "$R/tools/nn_cam2/cpu_sample.py" --summary "$O/cpu.jsonl" --skip-s 10 > "$O/cpu_summary.json" 2>&1
  dmesg | tail -60 > "$O/dmesg_tail.txt"
  ph stopped; touch "$O/STOPPED"
}

do_idle() {     # idle power: bitstream loaded, our processes stopped
  ph "idle ${1:-60}"; $PY "$B/pwr_sample.py" idle "${1:-60}" 10 "$O/pwr_idle.csv" > "$O/pwr_idle.out" 2>&1 || warn "idle power sampler"
}

do_recog() {    # recognition chain alone for a fixed time (A53 / PL comparisons): recog TAG SECONDS PREP [extra args]
  TAG=$1; S=$2; PR=$3; shift 3; D=$O/$TAG; mkdir -p "$D"
  CP=$(pidof_f kv_cam); OWN=0
  if ! alive "$CP"; then
    nohup "$CAMBIN/kv_cam" --name "$RING" --cpu 1 --nice -5 > "$D/kv_cam.out" 2> "$D/kv_cam.err" < /dev/null &
    CP=$!; OWN=1; sleep 5
  fi
  eval "$(cd "$R" && RECOG_MODEL=$RECOG_MODEL RECOG_MODEL_DIR=$B/models $PY tools/dpu_pipe/recog/recog_args.py --format env)"
  ph "recog:$TAG prep=$PR $*"
  # shellcheck disable=SC2086
  "$R/tools/dpu_pipe/build/dpu_rx" --frame-source kvcam --kvcam-name "$RING" --engine vart --xmodel "$RECOG_XMODEL" \
    ${RECOG_POST_ARGS:-} "$@" --size "$RECOG_SIZE" --fix "$RECOG_FIX" --workers "$WORKERS" --inflight "$WORKERS" --prep "$PR" \
    --seconds "$S" --warm-s 5 --stamp-log "$D/colour.jsonl" --out-log "$D/boxes.jsonl" > "$D/dpu_rx.json" 2> "$D/dpu_rx.err"
  note "recog:$TAG rc=$? $(grep -c . "$D/colour.jsonl" 2>/dev/null) rows; warn lines $(grep -ci warn "$D/dpu_rx.err")"
  [ "$OWN" = 1 ] && kill -TERM "$CP"
  $PY "$R/tools/nn_cam2/chain_report.py" "$D/colour.jsonl" --warm-s 10 --json "$D/chain.json" > "$D/chain_report.txt" 2>&1
}

do_status() {
  for n in $PROCS cpu_sample pwr_sample; do
    p=$(pidof_f $n); if alive "$p"; then echo "$n RUN pid $p"; else echo "$n DOWN"; fi
  done
  tail -1 "$O/uplink.jsonl" 2>/dev/null | cut -c1-600; grep -c . "$O/depth.jsonl" "$O/colour.jsonl" 2>/dev/null
  [ -f "$O/mem.json" ] && head -c 400 "$O/mem.json" && echo
  [ -f "$O/srch_status.json" ] && cut -c1-300 "$O/srch_status.json" && echo
  grep WARN "$P" | tail -5
}

case ${1:-} in
  start) do_start ;; stop) do_stop ;; status) do_status ;;
  idle) shift; do_idle "$@" ;; recog) shift; do_recog "$@" ;;
  *) sed -n 2,26p "$0" ;;
esac
