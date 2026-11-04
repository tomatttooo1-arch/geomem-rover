#!/bin/bash
# GOLIVE0 (2026-11-04): ONE command for the whole product on the bench, run on the server.
#
#   bash tools/golive/start_all.sh            (defaults BIT=m7pre MEM=pl SRCH=pl PLN=gmap3) stage (if not staged) -> push -> KV260 start (root, detached) -> Nano console
#   BIT=m5 bash tools/golive/start_all.sh     same with another bitstream preset (see the table below)
#   bash tools/golive/stop_all.sh             stop both ends by pid, chain report, pull the logs to the server
#   bash tools/golive/start_all.sh status     status of both ends (also: stage | push | kv | nano)
#
# KV260 (everything that senses and computes): bitstream load, kv_cam (camera on the KV), kv_server (geometry ->
# fusion + GRID cost map -> planner field, on the PL), dpu_rx (PL pre-processing -> DPU -> post-processing),
# kv_uplink (boxes + map status -> Nano). Nano: tools/golive/nano_console.py only (task console: find request input,
# result display, map status, liveness alarms); object memory runs on the PL memory core, search on the PL search core.
# CHASSIS selects the chassis mode: off = whole-system run on the bench (default) | kv = the KV260 chassis bridge
# (kv_server) sends velocity commands over the chassis USB serial and reads odometry back as the pose
# (host/kv_chassis_bridge.py) | nano = compatible with the earlier chassis stack.
#
# Before: kv260 + camera locks (tools/lock.sh), camera on the KV USB3 port.
# BIT presets   PKG (package dir)                    APP              PREP  NAV
#   m7pre (default)  installed app                    m7_pl_m7s17      pl    pl     BUILD_ID 0x063ba99b (impl-m6a-mk4-setp-m7s17-63ba99b:
#                planner GMAP3, memory core, search core SRCHK6 m17; xmutil load, SELFCHECK of the m5 package + WANT_ID;
#                the previous whole board m7_pl_m7pre0c1 / 0x06661def: BIT=app:m7_pl_m7pre0c1 WANT_ID=0x06661def)
#   m5           RUN/MERGE7/builds/pkg-m5_166         m7_pl_m5_166     pl    pl     BUILD_ID 0x0f35f07f
#   m4           RUN/MERGE4/builds/pkg-m4_166         m7_pl_m4_166     soft  soft   0x08880589 (no DPU: no recognition)
#   m3           RUN/MERGE3/builds/pkg-m3             m7_pl_m3         soft  soft   0x003e0b9f (DPU)
#   app:<name>   (an app already installed on the KV, e.g. app:m7_pl_v3fc2_166; xmutil load, WANT_ID=0x...)
# Env: RECOG_MODEL (y8s640 | y8s512), POST_ARGS (e.g. "--post pl"), WORKERS (1), KV_NAV_PARAMS (JSON), RUN_S (0 = until stop), MEASURE (1),
#      FIND (class the Nano console looks for, default bottle), GOLIVE_SSH_F (ssh_config, Tailscale path),
#      CODE_REF (git ref archived for the KV, default HEAD of this worktree), RESTAGE=1 (rebuild the stage).
#      SRCH (pl | nano, default pl; SRCHCHAIN0): probabilistic search on the KV's PL search core (needs a bitstream
#      with m7_srch, e.g. the m6a-mk4 build; without it the KV warns and the Nano console searches in software) or
#      on the Nano in software (the old path). SRCH_PERIOD (10 s), SRCH_XBIN (libsrch_drv.so), SRCH_SCENE (scene.npz),
#      CODE_REF (git ref archived for the KV, default HEAD of this worktree), RESTAGE=1 (rebuild the stage),
#      MEM (nano | pl, default pl; MEMCHAIN0: pl = object memory on the KV via the PL memory core, see kv_golive.sh),
#      MEMXBIN (host cross-built libmemk0.so for the C-model fallback, default $XBIN = RUN/GOLIVE1/xbin).
set -u
RUN=${FPGA_RUN}
G=${GOLIVE_DIR:-$RUN/GOLIVE0}; ST=$G/stage; KB=/home/ubuntu/golive_dev1; NB=/home/robot/fpga/dev1/golive0
WT=$(cd "$(dirname "$0")/../.." && pwd)
CODE_REF=${CODE_REF:-HEAD}
M512=${FPGA_SHARED}/models/20261103-dev3-recog-quant2/dpu/512/compiled/y8s_six_512_b1024.xmodel
M512_SHA=67b7c328db081d8cd2a81416f4809ce52e56f02e9e565774414d11f5d6ab11b3
M640=${FPGA_SHARED}/models/20261103-dev3-recog-quant4/full/models/640/compiled-selected/q4_y8s_640_b1024.xmodel
M640_SHA=b04fd7c1c7862eb965cd13fce49663e308bdaa1e6409ab9aa027113be2b884cd
KVCAM=$RUN/NN-CAM1/pkg/kvcam1.tar.gz; PWR=$RUN/BOARD5/stage/scripts/pwr_sample.py
XBIN=${XBIN:-$RUN/GOLIVE1/xbin}          # GOLIVE1 upgrade: one dir with every host cross build (dpu_rx_640, libkv_nav, libplanner_drv[_gmap3], libsrch_drv, libmemk0, srch_scene.npz; SHA256SUMS). m5-only: XBIN=$RUN/GOLIVE0/xbin_final
MEM=${MEM:-pl}; MEMXBIN=${MEMXBIN:-$XBIN}   # GOLIVE1 upgrade: default pl (was nano)
SUDO=${FPGA_HOME}/dev1/.secrets/kv260_sudo
SRCH=${SRCH:-pl}; SRCH_XBIN=${SRCH_XBIN:-$XBIN/libsrch_drv.so}; SRCH_SCENE=${SRCH_SCENE:-$XBIN/srch_scene.npz}
BIT=${BIT:-m7pre}; LOAD_MODE=bringup; WANT_ID=${WANT_ID:-}   # GOLIVE1 upgrade: default the intermediate build (was m5)
# PLNCHAIN0: PLN (m5 | gmap3, default gmap3): gmap3 = kv_nav's planner hook on the GMAP3 core (in-core path fetch +
# 400x400 global map), see kv_golive.sh; PLN_IDS (BUILD_IDs with the GMAP3 planner, default "0x063ba99b 0x06661def");
# PLNXBIN (host cross-built libplanner_drv_gmap3.so, staged always, default $XBIN = RUN/GOLIVE1/xbin; RESTAGE=1 after changes)
PLNXBIN=${PLNXBIN:-$XBIN}; PLN=${PLN:-gmap3}   # GOLIVE1 upgrade: default gmap3 (was m5)
case $BIT in
  m7pre) PKG=$RUN/MERGE7/builds/pkg-m5_166; APP=m7_pl_m7s17; LOAD_MODE=xmutil; WANT_ID=${WANT_ID:-0x063ba99b}; DPREP=pl; DNAV="--nav pl --nav-core kv_nav" ;;
  m5) PKG=$RUN/MERGE7/builds/pkg-m5_166; APP=m7_pl_m5_166; DPREP=pl; DNAV="--nav pl --nav-core kv_nav" ;;
  m4) PKG=$RUN/MERGE4/builds/pkg-m4_166; APP=m7_pl_m4_166; DPREP=soft; DNAV="--nav soft --nav-core kv_nav" ;;
  m3) PKG=$RUN/MERGE3/builds/pkg-m3; APP=m7_pl_m3; DPREP=soft; DNAV="--nav soft --nav-core kv_nav" ;;
  app:*) PKG=$RUN/MERGE7/builds/pkg-m5_166; APP=${BIT#app:}; LOAD_MODE=xmutil; DPREP=soft; DNAV="--nav soft --nav-core kv_nav" ;;
  *) echo "BIT=$BIT unknown (m7pre | m5 | m4 | m3 | app:<installed app>)"; exit 2 ;;
esac
PKG=${PKG_DIR:-$PKG}; PREP=${PREP:-$DPREP}; NAV=${NAV:-$DNAV}
SSH=(ssh ${GOLIVE_SSH_F:+-F $GOLIVE_SSH_F} -o ConnectTimeout=30 -o ServerAliveInterval=30)

do_stage() {
  rm -rf "$ST"; mkdir -p "$ST/models" "$ST/board" "$ST/xbin"
  [ -f "$PKG/SHA256SUMS" ] && (cd "$PKG" && sha256sum -c --quiet SHA256SUMS) || echo "WARN package sha check $PKG"
  cp -r "$PKG" "$ST/pkg"; rm -rf "$ST"/pkg/bringup-*
  echo "$M512_SHA  $M512" | sha256sum -c --quiet && cp "$M512" "$ST/models/" || echo "WARN 512 xmodel sha"
  echo "$M640_SHA  $M640" | sha256sum -c --quiet && cp "$M640" "$ST/models/" || echo "WARN 640 xmodel sha"
  git -C "$WT" archive --prefix=repo/ -o "$ST/repo.tar" "$CODE_REF" || { echo "git archive $CODE_REF failed"; return 2; }
  cp "$WT/tools/golive/kv_golive.sh" "$ST/board/"; cp "$KVCAM" "$PWR" "$ST/"
  cp "$XBIN"/* "$ST/xbin/" 2>/dev/null
  cp "$SRCH_XBIN" "$ST/xbin/libsrch_drv.so" || echo "WARN no $SRCH_XBIN (SRCH=pl falls back to the Nano)"
  cp "$SRCH_SCENE" "$ST/srch_scene.npz" || echo "WARN no $SRCH_SCENE (search has no scene)"
  [ "$MEM" = pl ] && { cp "$MEMXBIN/libmemk0.so" "$ST/xbin/" || echo "WARN no $MEMXBIN/libmemk0.so"; }
  cp "$PLNXBIN/libplanner_drv_gmap3.so" "$ST/xbin/" 2>/dev/null && echo "staged libplanner_drv_gmap3.so (PLNCHAIN0)" \
    || echo "WARN no $PLNXBIN/libplanner_drv_gmap3.so (PLN=gmap3 -> m5 driver)"
  ls "$ST/xbin" | tr '\n' ' '; echo
  printf 'BIT=%s APP=%s PKG=%s CODE_REF=%s(%s) PREP=%s NAV=%s SRCH=%s MEM=%s PLN=%s\n' "$BIT" "$APP" "$PKG" "$CODE_REF" \
    "$(git -C "$WT" rev-parse --short "$CODE_REF")" "$PREP" "$NAV" "$SRCH" "$MEM" "${PLN:-m5}" > "$ST/STAGED.txt"
  (cd "$ST" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS)
  cat "$ST/STAGED.txt"; du -sh "$ST" | cut -f1
}
do_push() {
  "${SSH[@]}" kv260 "mkdir -p $KB" && rsync -az --delete --exclude /out --exclude "/out_*" --exclude /repo --exclude /kvcam1 --exclude /extra -e "${SSH[*]}" "$ST/" kv260:$KB/; echo "push kv rc=$?"
  "${SSH[@]}" jetson "mkdir -p $NB" && scp -q ${GOLIVE_SSH_F:+-F $GOLIVE_SSH_F} "$WT/tools/golive/nano_console.py" jetson:$NB/ \
    && echo "push nano ok"
  # SRCHCHAIN0: the Nano's software search (SRCH=nano, and the fallback of SRCH=pl): product BeliefSearch + scene
  "${SSH[@]}" jetson "mkdir -p $NB/srch/host $NB/srch/tools/golive" && \
    scp -q ${GOLIVE_SSH_F:+-F $GOLIVE_SSH_F} "$WT/host/search_belief.py" "$WT/host/task_control.py" jetson:$NB/srch/host/ && \
    scp -q ${GOLIVE_SSH_F:+-F $GOLIVE_SSH_F} "$WT/tools/golive/srch_chain.py" jetson:$NB/srch/tools/golive/ && \
    scp -q ${GOLIVE_SSH_F:+-F $GOLIVE_SSH_F} "$SRCH_SCENE" jetson:$NB/srch/scene.npz && echo "push nano srch ok" \
    || echo "WARN push nano srch failed (Nano console runs without its own search)"
}
do_kv() {
  local NV=; [ -n "${KV_NAV_PARAMS:-}" ] && NV="KV_NAV_PARAMS=$(printf %q "$KV_NAV_PARAMS")"   # e.g. tick replanning
  NV="$NV PLN=${PLN:-m5}${PLN_IDS:+ PLN_IDS=\"$PLN_IDS\"}"   # PLNCHAIN0 planner switch (m5 | gmap3)
  timeout 60 "${SSH[@]}" kv260 "sudo -S -p '' bash -c 'cd $KB && rm -rf out && mkdir -p out && APP=$APP LOAD_MODE=$LOAD_MODE \
    WANT_ID=$WANT_ID PREP=$PREP NAV=\"$NAV\" RECOG_MODEL=${RECOG_MODEL:-y8s640} POST_ARGS=\"${POST_ARGS:---post pl}\" \
    NANO=${NANO:-198.51.100.1:47950} SRCH=$SRCH MEM=$MEM FIND=${FIND:-bottle} SRCH_PERIOD=${SRCH_PERIOD:-10} RUN_S=${RUN_S:-0} MEASURE=${MEASURE:-1} WORKERS=${WORKERS:-1} CHASSIS=${CHASSIS:-off} \
    CHASSIS_V_LIMIT=${CHASSIS_V_LIMIT:-150} CHASSIS_W_LIMIT=${CHASSIS_W_LIMIT:-500} ${LOCAL_PARAMS:+LOCAL_PARAMS=$LOCAL_PARAMS} $NV \
    setsid nohup bash board/kv_golive.sh start > out/nohup.txt 2>&1 < /dev/null & sleep 1; echo kv started'" < "$SUDO"
}
do_nano() {
  timeout 30 "${SSH[@]}" jetson "cd $NB && { [ -f console.pid ] && kill -0 \$(cat console.pid) 2>/dev/null && echo 'console already running pid' \$(cat console.pid); } \
    || { rm -f nano_console.jsonl nano_status.json; setsid nohup python3 nano_console.py --port 47950 --find ${FIND:-bottle} --srch $SRCH --srch-root $NB/srch --mem $MEM \
         --log nano_console.jsonl --status nano_status.json > console.out 2> console.err < /dev/null & echo \$! > console.pid; \
         sleep 1; echo nano console pid \$(cat console.pid); }"
}
do_wait_running() {   # the KV start takes ~1-2 min (load + camera + kv_server helper compiles)
  for i in $(seq 1 30); do
    out=$(timeout 30 "${SSH[@]}" kv260 "tail -1 $KB/out/progress.txt" 2>&1) || out="ssh fail"
    echo "$(date +%T) $out"; echo "$out" | grep -q 'PHASE running' && return 0; sleep 10
  done; echo "WARN: KV not at 'running' after 5 min (see status)"
}
do_status() {
  echo "--- KV260"; timeout 30 "${SSH[@]}" kv260 "bash $KB/board/kv_golive.sh status" 2>&1 | tail -12
  echo "--- Nano"; timeout 30 "${SSH[@]}" jetson "cd $NB && { kill -0 \$(cat console.pid) 2>/dev/null && echo console RUN pid \$(cat console.pid) || echo console DOWN; }; tail -1 console.out | cut -c1-700" 2>&1
}
case ${1:-all} in
  stage) do_stage ;; push) do_push ;; kv) do_kv ;; nano) do_nano ;; status) do_status ;;
  all) { [ -f "$ST/STAGED.txt" ] && [ "${RESTAGE:-0}" != 1 ] && grep -q "APP=$APP " "$ST/STAGED.txt" \
         && grep -q "($(git -C "$WT" rev-parse --short "$CODE_REF"))" "$ST/STAGED.txt" \
         && { [ ! -e "$XBIN/dpu_rx_640" ] || [ -e "$ST/xbin/dpu_rx_640" ]; }; } || do_stage || exit 2
       do_push && do_nano && do_kv && do_wait_running; do_status ;;
  *) sed -n 2,22p "$0" ;;
esac
