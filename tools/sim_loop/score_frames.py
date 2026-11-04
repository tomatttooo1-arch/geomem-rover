#!/usr/bin/env python3
"""Line D2 (2026-10-14): score rendered probe frames with the host YOLO (all 80 classes, conf floor 0.01,
otherwise the service's settings; --pt = torch CUDA fp32, the loop's backend, or --onnx = onnxruntime CPU)
and SSDLite (--ssd-device, score floor 0); one JSON row per frame.

  CUDA_VISIBLE_DEVICES=<held GPU> TORCH_HOME=<models> PYTHONPATH=<ultralytics 8.3.40> \
    python3.13 tools/sim_loop/score_frames.py --pt YOLO.pt --models DIR FRAME.png...

Reads the same detector classes the inference service builds; thresholds of the run (YOLO 0.25,
SSDLite 0.4) are NOT applied here, so the table shows how far each frame is from them.
"""
import argparse
import json
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--onnx')
    src.add_argument('--pt')
    ap.add_argument('--models', required=True)
    ap.add_argument('--ssd-device', choices=('cpu', 'cuda'), default='cpu')
    ap.add_argument('--threads', type=int, default=4)
    ap.add_argument('frames', nargs='+')
    a = ap.parse_args(argv)
    from perception.fpga_replay.yolo_detector import YoloSegDetector
    from perception.fpga_replay.detector import SSDDetector
    if a.pt:
        yolo = YoloSegDetector(a.pt, 'torch', classes=None, conf=0.01)
    else:
        yolo = YoloSegDetector(a.onnx, 'ort', classes=None, conf=0.01, threads=a.threads)
    ssd = SSDDetector(a.models, score=0.0, device=a.ssd_device)

    class Frame:
        pass
    for path in a.frames:
        rgb = np.asarray(Image.open(path).convert('RGB'))
        f = Frame()
        f.rgb, f.frame_id, f.timestamp = rgb, os.path.basename(path), 0.0
        raw = yolo.raw_detections(rgb)
        bottles = [d for d in raw if d['name'] == 'bottle']
        best = max(bottles, key=lambda d: d['score'], default=None)
        ssd_dets = ssd(f)
        sb = max([d.score for d in ssd_dets if d.label == 'bottle'], default=None)
        print(json.dumps({'frame': path, 'yolo_backend': yolo.backend, 'ssd_device': a.ssd_device,
                          'yolo_bottle': None if best is None else best['score'],
                          'yolo_bottle_box': None if best is None else best['box'],
                          'yolo_top3': [(d['name'], d['score']) for d in sorted(raw, key=lambda d: -d['score'])[:3]],
                          'ssdlite_bottle': None if sb is None else round(float(sb), 3)}), flush=True)


if __name__ == '__main__':
    main()
