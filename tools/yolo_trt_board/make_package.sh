#!/bin/bash
# Host: build the line-3 deploy package from a committed worktree.
#   bash make_package.sh <worktree> <results dir> <rec dir> <out dir>
# <out dir>/fpga_yolo_trt_20261014/{src/ (git archive HEAD: perception host tools/nano_detector
# tools/yolo_trt_board, selftest/ excluded), frames/ (the 39 manifest frames), frames_manifest.json,
# ref_board_onnx_ort.json, PACKAGE_INFO.json} and <out dir>/fpga_yolo_trt_20261014.tar (+ .sha256).
set -eu
W=$1; R=$2; REC=$3; O=$4
[ -z "$(git -C "$W" status --porcelain)" ] || { echo "worktree not clean; commit first"; exit 2; }
S=$O/fpga_yolo_trt_20261014
[ -e "$S" ] && { echo "REFUSE: $S exists"; exit 2; }
mkdir -p "$S/frames"
git -C "$W" archive --format=tar --prefix=src/ HEAD perception host tools/nano_detector tools/yolo_trt_board \
  ':(exclude)tools/yolo_trt_board/selftest' | tar -x -C "$S"
cp "$R/onnx_equiv/frames_manifest.json" "$R/onnx_equiv/ref_board_onnx_ort.json" "$S/"
python3 - "$S" "$REC" "$W" <<'PY'
import hashlib, json, os, shutil, subprocess, sys, time
S, rec, W = sys.argv[1:4]
m = json.load(open(os.path.join(S, "frames_manifest.json")))
for n in m["frames"]:
    shutil.copy2(os.path.join(rec, "color", n), os.path.join(S, "frames", n))
    assert hashlib.sha256(open(os.path.join(S, "frames", n), "rb").read()).hexdigest() == m["sha256"][n], n
info = {"commit": subprocess.check_output(["git", "-C", W, "rev-parse", "HEAD"], text=True).strip(),
        "branch": subprocess.check_output(["git", "-C", W, "rev-parse", "--abbrev-ref", "HEAD"], text=True).strip(),
        "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "frames": len(m["frames"])}
json.dump(info, open(os.path.join(S, "PACKAGE_INFO.json"), "w"), indent=1)
print(json.dumps(info))
PY
tar -cf "$O/fpga_yolo_trt_20261014.tar" -C "$O" fpga_yolo_trt_20261014
( cd "$O" && sha256sum fpga_yolo_trt_20261014.tar > fpga_yolo_trt_20261014.tar.sha256 && cat fpga_yolo_trt_20261014.tar.sha256 )
du -sh "$O/fpga_yolo_trt_20261014.tar"
