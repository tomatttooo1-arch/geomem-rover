"""Fixed-image Nano profile check; saves evidence, never opens the camera or moves the car."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np

from perception.fpga_replay.yolo_detector import YoloSegDetector
from tools.nano_detector.nano_detector import annotate


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--root', required=True, type=Path)
    p.add_argument('--profile', choices=('household9', 'coco-seg'))
    a = p.parse_args()
    out = a.root / 'fixed-validation'
    out.mkdir(exist_ok=True)
    if not a.profile:
        report = {'tests': {}, 'motion_commands': 0}
        for profile in ('household9', 'coco-seg'):
            subprocess.run([sys.executable, __file__, '--root', str(a.root), '--profile', profile], check=True)
            part = json.loads((out/(profile+'-validation.json')).read_text())
            report['input_shape'] = part['input_shape']
            report['tests'].update(part['tests'])
        (out/'fixed-validation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        return
    rgb = cv2.cvtColor(cv2.imread(str(a.root/'model/user-floor-phone.jpg')), cv2.COLOR_BGR2RGB)
    report = {'input_shape': list(rgb.shape), 'tests': {}, 'motion_commands': 0}
    for profile, model in [('household9', a.root/'model/best-fp16.engine'),
                           ('coco-seg', a.root/'baseline/yolov8n-seg-640-fp16.engine')]:
        if profile != a.profile:
            continue
        d = YoloSegDetector(str(model), 'trt', profile=profile)
        ds = d.raw_detections(rgb)
        times = []
        for _ in range(20):
            t = time.perf_counter()
            ds = d.raw_detections(rgb)
            times.append((time.perf_counter()-t)*1000)
        annotate(rgb, ds, d._geometry, str(out/(profile+'.jpg')), width=960, quality=90)
        entry = {'execution': d.execution, 'detections': ds, 'warmed_total_ms': times,
                 'median_ms': float(np.median(times))}
        if profile == 'household9':
            raw = d._board.m.buf[d._board.out_det].cpu().numpy()
            x = d._board.m.inp.cpu().numpy()
            np.save(out/'trt-output.npy', raw)
            np.save(out/'trt-input.npy', x)
            assert raw.shape == (1, 13, 8400) and np.isfinite(raw).all()
            assert any(v['name'] == 'cell phone' and v['score'] > .8 for v in ds), ds
            entry['output_shape'] = list(raw.shape)
        report['tests'][profile] = entry
        del d
    (out/(a.profile+'-validation.json')).write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
