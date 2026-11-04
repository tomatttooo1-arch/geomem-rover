#!/bin/bash
# Host dry run of the line-3 board chain with the onnxruntime stand-in for TensorRT (fake_trt.py):
#   bash selftest_host.sh <worktree> <package dir (has fpga_yolo_trt_20261014.tar)> <work dir> <board onnx> <results dir>
# Extracts the real deploy tarball, runs run_board.sh d5 / full / smoke exactly as on the board
# (SKIP_BOARD_CHECKS=1, PY=python3.13, sitecustomize from this directory), then compare_board d5/full
# and two known-different perturbations of the board file (box +20 px -> B2 must fail; score +0.1 -> B3 must fail).
set -u
W=$1; P=$2; X=$3; ONNX=$4; R=$5
SELF=$(cd "$(dirname "$0")" && pwd)
[ -e "$X" ] && { echo "REFUSE: $X exists"; exit 2; }
mkdir -p "$X" && tar -xf "$P/fpga_yolo_trt_20261014.tar" -C "$X"
D=$X/fpga_yolo_trt_20261014
[ -e "$D/src/tools/yolo_trt_board/selftest" ] && { echo "FAIL: selftest dir leaked into the package"; exit 1; }
export SKIP_BOARD_CHECKS=1 PY=python3.13 EXTRA_PYTHONPATH=$SELF FPGA_FAKE_TRT=1 FPGA_FAKE_ONNX=$ONNX
export SMOKE_SOCK=$X/smoke.sock
ENG=$ONNX      # stand-in "engine": any existing file; its sha is recorded and is not the real engine's
for m in d5 full smoke; do echo "== run_board $m"; bash "$D/src/tools/yolo_trt_board/run_board.sh" "$D" "$ENG" "$m"; echo "rc=$?"; done
unset FPGA_FAKE_TRT EXTRA_PYTHONPATH
C="python3.13 -m tools.yolo_trt_board.compare_board"
export PYTHONPATH=$W PYTHONDONTWRITEBYTECODE=1
REF=$R/onnx_equiv/ref_board_onnx_ort.json
echo "== compare d5 (no --allow-fake: must be unusable)"; $C d5 "$REF" "$D/out/d5/board_detections.json" "$X/judge_d5_nofake.json" | grep -E '"usable"|"d5"|"T"'
echo "== compare d5"; $C d5 "$REF" "$D/out/d5/board_detections.json" "$X/judge_d5.json" --allow-fake | grep -E '"usable"|"d5"|"T"'
T=$(python3.13 -c "import json; print(json.load(open('$X/judge_d5.json'))['T'])")
echo "== compare full (T=$T)"
$C full "$REF" "$D/out/full/board_detections.json" "$X/judge_full.json" --gate "$T" --d5 "$X/judge_d5.json" \
   --latency "$D/out/full/latency_summary.json" --env "$D/out/full/board_env.json" --allow-fake | grep -E 'VERDICT|instrument_ok|gate_matches|equals_d5'
python3.13 - "$D/out/full/board_detections.json" "$X" <<'PY'
import json, sys
src, X = sys.argv[1:3]
b = json.load(open(src))
s = dict(b, detections={n: [dict(d, box=[d["box"][0] + 20, d["box"][1], d["box"][2] + 20, d["box"][3]]) for d in v] for n, v in b["detections"].items()})
json.dump(s, open(X + "/kd_shift20.json", "w"))
s = dict(b, detections={n: [dict(d, score=d["score"] + 0.1) for d in v] for n, v in b["detections"].items()})
json.dump(s, open(X + "/kd_score01.json", "w"))
PY
for k in shift20 score01; do
  echo "== known-different $k (must FAIL)"
  $C full "$REF" "$X/kd_$k.json" "$X/judge_kd_$k.json" --gate "$T" --allow-fake | grep -E 'VERDICT'
  python3.13 -c "import json; m=json.load(open('$X/judge_kd_$k.json'))['main']; print({k: m[k] for k in ('B1_classes_same_no_unmatched','B2_iou_ge_gate','B3_dscore_le_gate','B4_positive_control','PASS')})"
done
