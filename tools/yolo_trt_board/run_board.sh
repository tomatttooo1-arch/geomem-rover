#!/bin/bash
# Board entry for line 3 (runs/yolo-trt-board-20261014-results/PREREG.md). On the Orin Nano host system:
#   bash <deploy>/src/tools/yolo_trt_board/run_board.sh <deploy root> <engine> <d5|full|smoke>
#   d5    : first 5 of the 39 frames, 1 pass   -> d5 for the B3 gate (PREREG section 2)
#   full  : 39 frames x 5 passes, first 10 calls = warm-up -> B1-B4 + latency (sections 2-3)
#   smoke : host.inference_service --detector yolo --engine ... + InferenceClient, 5 frames (section 4)
# Output: <deploy root>/out/<mode>/ (refuses to overwrite). Hard stops (exit 3): nvpmodel not MAXN_SUPER,
# engine sha256 differs, smoke socket path already exists. No jetson_clocks, no clock locking, no taskset.
# SKIP_BOARD_CHECKS=1 / EXTRA_PYTHONPATH / PY exist only for the host dry run (recorded in shell_env.txt).
# 2026-10-15 engine rebuild (runs/yolo-engine-rebuild-20261015-results): EXPECT_ENGINE_SHA pins another
# engine (default: the 09-27 engine), OUT_NAME writes to out/<OUT_NAME> instead of out/<mode>, CPU_PIN
# runs the d5/full python under `taskset -c CPU_PIN` (A/B arms on one CPU cluster). All recorded in shell_env.txt.
set -u
ROOT=$(cd "$1" && pwd); ENGINE=$(readlink -f "$2"); MODE=$3
ENGINE_SHA=${EXPECT_ENGINE_SHA:-0a111806558c33df20c656818c7b486d8f9193f643c248e47962116cbc288fd6}
PY=${PY:-python3}
case "$MODE" in d5|full|smoke) ;; *) echo "mode must be d5|full|smoke"; exit 2;; esac
OUT=$ROOT/out/${OUT_NAME:-$MODE}
PIN=()
[ -n "${CPU_PIN:-}" ] && PIN=(taskset -c "$CPU_PIN")
[ -e "$OUT" ] && { echo "REFUSE: $OUT exists"; exit 2; }
mkdir -p "$OUT"
export PYTHONPATH="${EXTRA_PYTHONPATH:+$EXTRA_PYTHONPATH:}$ROOT/src:$HOME/fpga/p2/pylib"
export PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
# python -m puts the cwd first on sys.path: run from the package's own src so nothing in the
# caller's directory (e.g. a tools/ or host/ under $HOME) can shadow the deployed modules.
cd "$ROOT/src" || exit 2
{ date -Is; echo "mode=$MODE engine=$ENGINE PY=$PY"
  echo "SKIP_BOARD_CHECKS=${SKIP_BOARD_CHECKS:-0} EXTRA_PYTHONPATH=${EXTRA_PYTHONPATH:-} FPGA_FAKE_TRT=${FPGA_FAKE_TRT:-0}"
  echo "EXPECT_ENGINE_SHA=$ENGINE_SHA OUT_NAME=${OUT_NAME:-} CPU_PIN=${CPU_PIN:-}"
  echo "PYTHONPATH=$PYTHONPATH"; nvpmodel -q 2>&1; uptime; cat /etc/nv_tegra_release 2>/dev/null
  sha256sum "$ENGINE"; cat "$ROOT/PACKAGE_INFO.json" 2>/dev/null | head -5; } > "$OUT/shell_env.txt" 2>&1
if [ "${SKIP_BOARD_CHECKS:-0}" != 1 ]; then
  nvpmodel -q 2>/dev/null | grep -q "MAXN_SUPER" || { echo "STOP: nvpmodel is not MAXN_SUPER"; exit 3; }
  echo "$ENGINE_SHA  $ENGINE" | sha256sum -c --quiet || { echo "STOP: engine sha256 differs"; exit 3; }
fi
ACTUAL_SHA=$(sha256sum "$ENGINE" | cut -d' ' -f1)
RUN=(-m tools.yolo_trt_board.board_run --engine "$ENGINE" --frames-dir "$ROOT/frames"
     --manifest "$ROOT/frames_manifest.json" --out "$OUT" --package-info "$ROOT/PACKAGE_INFO.json")
case "$MODE" in
  d5)   "${PIN[@]}" "$PY" "${RUN[@]}" --n-frames 5 --passes 1 --warmup 0 --tag d5 > "$OUT/run.stdout" 2> "$OUT/run.stderr"
        RC=$?; echo "rc=$RC"; tail -3 "$OUT/run.stderr"; exit $RC;;
  full) "${PIN[@]}" "$PY" "${RUN[@]}" --n-frames 39 --passes 5 --warmup 10 --tag full > "$OUT/run.stdout" 2> "$OUT/run.stderr"
        RC=$?; echo "rc=$RC"; tail -3 "$OUT/run.stderr"; exit $RC;;
esac
# smoke
SOCK=${SMOKE_SOCK:-/tmp/fpga_yolo_trt_20261014.sock}
[ -e "$SOCK" ] && { echo "STOP: $SOCK exists (another launch?)"; exit 3; }
"$PY" -m host.inference_service --detector yolo --engine "$ENGINE" --device cuda --socket "$SOCK" \
  --warmup-frames 1 --warmup-size 640x360 > "$OUT/service.stdout" 2> "$OUT/service.stderr" &
SPID=$!
echo "$SPID" > "$OUT/service.pid"
trap 'kill -TERM $SPID 2>/dev/null' EXIT
for _ in $(seq 1 240); do
  grep -q '"ready": true' "$OUT/service.stdout" 2>/dev/null && break
  kill -0 "$SPID" 2>/dev/null || break
  sleep 0.5
done
if ! grep -q '"ready": true' "$OUT/service.stdout" 2>/dev/null; then
  echo "STOP: service not ready"; tail -5 "$OUT/service.stderr"; exit 4
fi
"$PY" -m tools.yolo_trt_board.board_smoke "$SOCK" "$ROOT/frames" "$ROOT/ref_board_onnx_ort.json" "$ACTUAL_SHA" \
  "$OUT/smoke.json" > "$OUT/smoke.stdout" 2> "$OUT/smoke.stderr"
RC=$?
kill -TERM "$SPID"; wait "$SPID"; SRC=$?
trap - EXIT
[ -e "$SOCK" ] && LEFT=true || LEFT=false
echo "{\"smoke_rc\": $RC, \"service_rc\": $SRC, \"socket_left\": $LEFT}" | tee "$OUT/smoke_shell.json"
tail -3 "$OUT/smoke.stderr"
exit $RC
