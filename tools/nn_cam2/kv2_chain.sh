#!/bin/bash
# NN-CAM2 board helper for the KV-2 session (KV260 + camera, car still). Runbook: RUN/NN-CAM2/board_runbook.md.
# Every process it starts is stopped by its own pid (no pkill). Nothing here refuses to run: a missing piece is
# printed as WARN and the step goes on.
#
#   kv2_chain.sh build                     native build on the KV: dpu_rx (VART), kv_nav lib
#   kv2_chain.sh check                     instrument gates: cpu_sample --selftest, 5 s power read, ring info
#   kv2_chain.sh cam-start | cam-stop      kv_cam (product defaults, cpu1, nice -5), pid in $O/kv_cam.pid
#   kv2_chain.sh idle TAG SECONDS          only the samplers (CPU 1 Hz, power 10 Hz)
#   kv2_chain.sh run TAG SECONDS [depth|colour|both]
#                                          kv_server and/or dpu_rx on the ring + samplers, then report
#   kv2_chain.sh report TAG                chain_report + CPU + power summaries -> $O/TAG/report.json
#
# Environment (defaults in brackets):
#   R       repo checkout on the KV                       [~/nncam2/repo]
#   O       output root                                   [~/nncam2/out]
#   CAMBIN  kv_cam binary dir (NN-CAM1 pkg, arm build)    [~/nncam2/kvcam1/bin]
#   PWR     pwr_sample.py (BOARD5/stage/scripts)          [~/nncam2/pwr_sample.py]
#   PARAMS  --local-params JSON                           [$R/tools/nn_cam2/params_car_bench.json]
#   RECOG_MODEL  recognition model switch (RECOG640, 11-04)  [y8s640]  y8s640 | y8s512 (previous) | y8s640r (reserved)
#           -> SIZE / FIX / XMODEL / output processing from tools/dpu_pipe/recog/profiles.json (recog_args.py);
#           RECOG_MODEL_DIR [~/recog/models], RECOG_THR_MEDBOX (medicine box threshold), RECOG_CLS_THR (all six)
#   XMODEL  DPU model for SIZE (explicit = wins)          [from RECOG_MODEL]
#   SIZE FIX WORKERS                                      [from RECOG_MODEL: 640 6; 2]
#   PREP    soft | pl                                     [soft]
#   POST    pl | cpu  output processing (RECOG641)       [pl]  pl = PL detection post-processing core, falls back to cpu with a warning
#   KVS     kv_server geometry/fusion args                [--core pl --device devmem --wait irq --roi-core pl
#                                                          --rolling pl --rolling-grid pl --kv-fast native]
#   NAV     kv_server nav args                            [--nav soft --nav-core kv_nav]
#   SUDO    prefix for kv_server (devmem / UIO)           [sudo]
#   RING    kv_cam ring name                              [kv_cam_v1]
#   KV_NAV_PARAMS  kv_nav bench parameters (passed through sudo with env)  [{"stuck_window": 1e9}]
set -u
R=${R:-$HOME/nncam2/repo}
O=${O:-$HOME/nncam2/out}
CAMBIN=${CAMBIN:-$HOME/nncam2/kvcam1/bin}
PWR=${PWR:-$HOME/nncam2/pwr_sample.py}
PARAMS=${PARAMS:-$R/tools/nn_cam2/params_car_bench.json}
PY=${PY:-python3}
# RECOG640: one switch. Explicit SIZE / FIX / XMODEL still win; RECOG_POST_ARGS = output processing of the model
# (y8s512: empty = DPUPIPE1 behaviour). recog_args.py never refuses (WARN + default profile).
RECOG_MODEL=${RECOG_MODEL:-y8s640}; export RECOG_MODEL
eval "$($PY "$R/tools/dpu_pipe/recog/recog_args.py" --format env)" || echo "WARN: recog_args.py" >&2
XMODEL=${XMODEL:-${RECOG_XMODEL:-}}
SIZE=${SIZE:-${RECOG_SIZE:-640}}; FIX=${FIX:-${RECOG_FIX:-6}}; WORKERS=${WORKERS:-2}; PREP=${PREP:-soft}; POST=${POST:-pl}
[ -n "$XMODEL" ] && [ ! -f "$XMODEL" ] && { echo "WARN: XMODEL $XMODEL missing: stub engine" >&2; XMODEL=; }
KVS=${KVS:---core pl --device devmem --wait irq --roi-core pl --rolling pl --rolling-grid pl --kv-fast native}
NAV=${NAV:---nav soft --nav-core kv_nav}
SUDO=${SUDO-sudo}          # empty = no prefix (host dry run)
RING=${RING:-kv_cam_v1}
# Bench navigation parameters (host/kv_nav.make_core reads KV_NAV_PARAMS). The car cannot move on the bench, so the
# 20 s stuck detection would put kv_nav into recovery and stop its replans (host dry run: 743 of 1658 ticks in
# "recover", one 35 s gap without a field) -- off here. Downside: the bench does not exercise stuck / recovery.
# Every-tick replanning (S4): KV_NAV_PARAMS='{"stuck_window": 1e9, "replan_period": 0}'.
KV_NAV_PARAMS=${KV_NAV_PARAMS:-'{"stuck_window": 1e9}'}
mkdir -p "$O"
warn() { echo "WARN: $*" >&2; }

cmd=${1:-}; shift || true
case "$cmd" in
build)
  (cd "$R/tools/dpu_pipe" && make OUT=build GLOG_SHIM=1 VART=1 2>&1 | tail -3) || warn "dpu_rx build"
  (cd "$R/tools/kv_nav" && make 2>&1 | tail -2) || warn "kv_nav build"
  ls -la "$R/tools/dpu_pipe/build/dpu_rx" "$CAMBIN/kv_cam" 2>&1
  ;;
check)
  $PY "$R/tools/nn_cam2/cpu_sample.py" --selftest || warn "cpu sampler selftest"
  $PY "$PWR" check 5 10 "$O/check_pwr.csv" && $PY - "$O/check_pwr.csv" <<'EOF' || warn "power read"
import csv, sys
p = [int(r["power_uW"]) for r in csv.DictReader(open(sys.argv[1]))]
print({"power_W_mean": round(sum(p) / len(p) / 1e6, 4), "samples": len(p)})
EOF
  "$CAMBIN/kv_cam_read" --name "$RING" --info 2>&1 | head -20 || warn "kv_cam_read --info (kv_cam running?)"
  ;;
cam-start)
  nohup "$CAMBIN/kv_cam" --name "$RING" --cpu 1 --nice -5 --alarm-log "$O/kv_cam_alarms.jsonl" > "$O/kv_cam.out" 2> "$O/kv_cam.err" &
  echo $! > "$O/kv_cam.pid"; sleep 5; echo "kv_cam pid $(cat "$O/kv_cam.pid")"; tail -3 "$O/kv_cam.err"
  ;;
cam-stop)
  [ -f "$O/kv_cam.pid" ] && kill -TERM "$(cat "$O/kv_cam.pid")" && sleep 2; rm -f "$O/kv_cam.pid"
  ;;
idle|run)
  TAG=$1; SECS=$2; WHAT=${3:-both}; D="$O/$TAG"; mkdir -p "$D"
  PIDS=(); NAMES=()
  CAMPID=$(cat "$O/kv_cam.pid" 2>/dev/null || true)
  [ -n "$CAMPID" ] && NAMES+=("--pid" "kv_cam=$CAMPID")
  if [ "$cmd" = run ] && [ "$WHAT" != colour ]; then
    # shellcheck disable=SC2086
    $SUDO env KV_NAV_PARAMS="$KV_NAV_PARAMS" $PY "$R/tools/kv_link/kv_server.py" $KVS $NAV --port 47800 --max-idle-s 3600 --frame-source kvcam --kvcam-name "$RING" \
      --local-params "$PARAMS" --stamp-log "$D/depth.jsonl" --out "$D" > "$D/kv_server.out" 2> "$D/kv_server.err" &
    SP=$!; PIDS+=("$SP"); sleep 8
    KP=$(pgrep -P "$SP" -n 2>/dev/null || echo "$SP")       # the python child when started through sudo
    NAMES+=("--pid" "kv_server=$KP"); head -c 400 "$D/kv_server.out"; echo
  fi
  if [ "$cmd" = run ] && [ "$WHAT" != depth ]; then
    [ -n "$XMODEL" ] || warn "XMODEL not set: dpu_rx runs the stub engine (no DPU)"
    ENG="--engine stub --infer-ms 50"; [ -n "$XMODEL" ] && ENG="--engine vart --xmodel $XMODEL ${RECOG_POST_ARGS:-}"
    # shellcheck disable=SC2086
    "$R/tools/dpu_pipe/build/dpu_rx" --frame-source kvcam --kvcam-name "$RING" $ENG --size "$SIZE" --fix "$FIX" --workers "$WORKERS" \
      --inflight "$WORKERS" --prep "$PREP" --post "$POST" --seconds "$((SECS + 5))" --warm-s 5 --stamp-log "$D/colour.jsonl" \
      --out-log "$D/boxes.jsonl" > "$D/dpu_rx.json" 2> "$D/dpu_rx.err" &
    RP=$!; PIDS+=("$RP"); NAMES+=("--pid" "dpu_rx=$RP"); sleep 3
  fi
  $PY "$R/tools/nn_cam2/cpu_sample.py" --out "$D/cpu.jsonl" --period 1 --seconds "$SECS" "${NAMES[@]}" &
  CP=$!
  $PY "$PWR" "$TAG" "$SECS" 10 "$D/pwr.csv" || warn "power sampler"
  wait "$CP"
  for p in "${PIDS[@]}"; do
    if [ "$p" = "${SP:-}" ]; then $SUDO kill -TERM "$KP" 2>/dev/null; fi
    kill -TERM "$p" 2>/dev/null
  done
  for p in "${PIDS[@]}"; do wait "$p" 2>/dev/null; done
  "$0" report "$TAG"
  ;;
report)
  TAG=$1; D="$O/$TAG"
  ST=(); [ -f "$D/depth.jsonl" ] && ST+=("$D/depth.jsonl"); [ -f "$D/colour.jsonl" ] && ST+=("$D/colour.jsonl")
  [ ${#ST[@]} -gt 0 ] && $PY "$R/tools/nn_cam2/chain_report.py" "${ST[@]}" --warm-s 10 --json "$D/chain.json"
  $PY "$R/tools/nn_cam2/cpu_sample.py" --summary "$D/cpu.jsonl" --skip-s 10 > "$D/cpu_summary.json"
  $PY - "$D" <<'EOF'
import csv, json, os, sys
d = sys.argv[1]
out = {"tag": os.path.basename(d)}
try:
    rows = list(csv.DictReader(open(os.path.join(d, "pwr.csv"))))
    t = [float(r["t_s"]) for r in rows]
    p = [int(r["power_uW"]) / 1e6 for r, ti in zip(rows, t) if ti >= 10.0]
    s = sorted(p)
    out["power_W"] = {"mean": round(sum(p) / len(p), 4), "p95": s[int(0.95 * (len(s) - 1))], "samples": len(p)}
except Exception as exc:
    out["power_W"] = {"error": repr(exc)}
out["cpu"] = json.load(open(os.path.join(d, "cpu_summary.json")))
if os.path.exists(os.path.join(d, "chain.json")):
    out["chain"] = json.load(open(os.path.join(d, "chain.json")))
json.dump(out, open(os.path.join(d, "report.json"), "w"), indent=1)
c = out["cpu"]
print(json.dumps({"power_W_mean": out["power_W"].get("mean"), "cpu_total_cores_mean": c.get("cpu", {}).get("mean"),
                  "per_cpu_mean": {k: v["mean"] for k, v in c.items() if k.startswith("cpu") and k != "cpu"},
                  "per_pid_mean": {k: v["mean"] for k, v in c.items() if k.startswith("pid:")}}))
EOF
  ;;
*)
  sed -n 2,30p "$0"
  ;;
esac
