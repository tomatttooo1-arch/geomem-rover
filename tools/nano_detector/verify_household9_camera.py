"""Isolated stationary RGB-D smoke test, without KV/ROS or navigation. Not full-stack acceptance."""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import time

import cv2
import numpy as np

from perception.fpga_replay.core import Config, Frame, Memory, compute_geometry, process_frame
from perception.fpga_replay.household_names import NAMES, CLASS_ZH
from perception.fpga_replay.yolo_detector import YoloSegDetector
from tools.nano_detector.nano_detector import annotate
from tools.nano_loop.nano_loop import open_camera


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--root', required=True, type=Path)
    a = p.parse_args()
    out = a.root/'camera-validation'
    out.mkdir(exist_ok=True)
    cfg = replace(Config(), query_classes=NAMES)
    memory = Memory(cfg=cfg)
    detector = YoloSegDetector(str(a.root/'model/best-fp16.engine'), 'trt', profile='household9')
    pipe, meta = open_camera(attempts=3, wait_s=4.)
    rows = []
    try:
        deadline = time.monotonic()+25
        with (out/'frames.jsonl').open('a') as log:
            while len(rows) < 30 and time.monotonic() < deadline:
                fs = pipe.wait_for_frames(500)
                if fs is None:
                    continue
                cf, df = fs.get_color_frame(), fs.get_depth_frame()
                if cf is None or df is None:
                    continue
                rgb = np.frombuffer(cf.get_data(), np.uint8).reshape(cf.get_height(), cf.get_width(), 3).copy()
                depth = np.frombuffer(df.get_data(), np.uint16).reshape(df.get_height(), df.get_width()).astype(np.float32)
                depth *= df.get_depth_scale()/1000.
                assert depth.shape == rgb.shape[:2], 'HW aligned depth shape mismatch'
                ts = cf.get_timestamp_us()/1e6
                f = Frame('camera-%d' % cf.get_index(), ts, rgb, depth, np.asarray(meta['K']), np.eye(4),
                          pose_source='stationary_camera_identity', depth_timestamp=df.get_timestamp_us()/1e6)
                ds = detector(f)
                obs = process_frame(f, ds, cfg)
                memory.update_memory(obs, compute_geometry(f, memory, cfg), f)
                valid = np.isfinite(depth) & (depth > .1)
                row = {'frame_id': f.frame_id, 'timestamp': ts, 'depth_valid_fraction': float(valid.mean()),
                       'detections': [asdict(d) for d in ds],
                       'observations': [{'label': o.label, 'position_camera_m': o.position.tolist()} for o in obs],
                       'memory_tracks': len(memory.tracks)}
                rows.append(row)
                log.write(json.dumps(row)+'\n'); log.flush()
                if len(rows) in (1, 15, 30):
                    idx = '%02d' % len(rows)
                    cv2.imwrite(str(out/(idx+'-rgb.jpg')), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                    np.save(out/(idx+'-depth-m.npy'), depth)
                    raw = [{'name': d.label, 'score': d.score, 'box': list(d.bbox)} for d in ds]
                    annotate(rgb, raw, detector._geometry, str(out/(idx+'-annotated.jpg')), width=960)
                time.sleep(.12)
    finally:
        pipe.stop()
    assert len(rows) >= 10, 'insufficient live frames'
    report = {'scope': 'isolated camera + detector + CPU depth/memory smoke; fixed identity pose, not localization or deployed full stack',
              'motion_commands': 0, 'camera': meta, 'frames': len(rows),
              'execution': detector.execution, 'detected_classes': sorted({d['label'] for r in rows for d in r['detections']}),
              'observations': sum(len(r['observations']) for r in rows), 'memory_tracks': len(memory.tracks),
              'queries': {CLASS_ZH[n]: len(memory.query_memory('找'+CLASS_ZH[n])) for n in NAMES},
              'mean_valid_depth_fraction': float(np.mean([r['depth_valid_fraction'] for r in rows]))}
    (out/'camera-validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
