#!/bin/bash
# One-time board setup for nano_detector (Orin Nano, JetPack 6.2, no sudo). Run detached:
#   nohup bash board_setup.sh ~/fpga/p2 > setup.log 2>&1 &
# - Python packages go to <dir>/pylib with --no-deps (python3-venv/ensurepip is not installed on the
#   board, and --no-deps guarantees the JetPack torch in ~/.local is never replaced). Use with
#   PYTHONPATH=<dir>/pylib.
# - Weights: COCO-pretrained yolov8n-seg.pt from the ultralytics GitHub release (the board's GitHub
#   access is intermittent; the ghfast.top mirror is the fallback), checked against the sha256 seen
#   on 2026-09-27 from both sources.
# - ONNX export on CPU, opset 17, static 1x3xSxS, no onnxslim; then trtexec FP16 engines (~8-10 min each).
set -eu
D=${1:-$HOME/fpga/p2}; mkdir -p "$D/w"; cd "$D"
python3 -m pip install --no-deps --target "$D/pylib" --default-timeout=120 \
  "ultralytics==8.3.40" "ultralytics-thop==2.0.12" tqdm psutil py-cpuinfo "onnx==1.17.0" "protobuf==4.25.5" "pyorbbecsdk2==2.1.2"
export PYTHONPATH="$D/pylib"
python3 -c "import torch, ultralytics, onnx, pyorbbecsdk; print(torch.__version__, torch.__file__, torch.cuda.is_available(), ultralytics.__version__)"
cd w
U=https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n-seg.pt
SHA=a7cd8f929e1903d78a12a48efecab430209f18dc46cb96c3599a5980c63c423c
[ -f yolov8n-seg.pt ] || curl -sSL --connect-timeout 20 -o yolov8n-seg.pt $U || curl -sSL --connect-timeout 20 -o yolov8n-seg.pt https://ghfast.top/$U
echo "$SHA  yolov8n-seg.pt" | sha256sum -c
python3 - <<'PY'
import shutil
from ultralytics import YOLO
from ultralytics.utils import SETTINGS
SETTINGS.update({"sync": False})
for sz in (640, 320):
    p = YOLO("yolov8n-seg.pt").export(format="onnx", imgsz=sz, opset=17, simplify=False, dynamic=False, half=False, device="cpu")
    shutil.move(p, f"yolov8n-seg-{sz}.onnx")
PY
for s in 640 320; do
  /usr/src/tensorrt/bin/trtexec --onnx=yolov8n-seg-$s.onnx --fp16 --saveEngine=yolov8n-seg-$s-fp16.engine \
    --memPoolSize=workspace:1024 --skipInference --profilingVerbosity=detailed > build_$s.log 2>&1
  grep -E "Engine built in|PASSED" build_$s.log
done
sha256sum *.onnx *.engine
echo SETUP_DONE
