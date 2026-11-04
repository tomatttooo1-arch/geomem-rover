#!/usr/bin/env python3
"""Fixed-detection stand-in for the inference service (board runs of the geometry switch).

Serves host.inference_service's socket protocol with a detector that loads no model and uses
no GPU: it reads the source index the replay camera wrote into the rgb image
(tools/car_geometry_kv/replay_camera.encode_index) and returns a fixed schedule of boxes for
that source frame. Same source frame -> same detections in every run, whatever each run
dropped, so cpu and kv runs see identical observations and the memory has tracks whose
geometry is judged on the frames between detections.

Default schedule (rec30, 640x360; boxes checked with process_frame on rec30 depth):
  chair  (260,120,380,260)  source idx % 5 == 0
  cup    (100,80,300,280)   source idx % 7 == 0
  bottle (340,80,540,280)   source idx % 10 == 0 and idx < 100
Frames without an encoded index get no detections (counted).
Usage: stub_inference.py --socket PATH [--device cpu]
"""
import argparse
import json
import os
import signal
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from perception.fpga_replay.core import Detection  # noqa: E402
from tools.car_geometry_kv.replay_camera import decode_index  # noqa: E402

SCHEDULE = (
    ("chair", (260.0, 120.0, 380.0, 260.0), lambda i: i % 5 == 0),
    ("cup", (100.0, 80.0, 300.0, 280.0), lambda i: i % 7 == 0),
    ("bottle", (340.0, 80.0, 540.0, 280.0), lambda i: i % 10 == 0 and i < 100),
)


class FixedDetector(object):
    detector = "stub_fixed_schedule"

    def __init__(self, device="cpu", schedule=SCHEDULE):
        self.execution = {"device": device, "model": None}
        self.schedule = schedule
        self.counts = {"frames": 0, "no_index": 0, "detections": 0}

    def boxes(self, idx):
        return [(label, bbox) for label, bbox, when in self.schedule if when(idx)]

    def __call__(self, frame):
        self.counts["frames"] += 1
        idx = decode_index(frame.rgb)
        if idx is None:
            self.counts["no_index"] += 1
            return []
        out = [Detection(label, 0.9, bbox, "detector", frame.frame_id, frame.timestamp)
               for label, bbox in self.boxes(idx)]
        self.counts["detections"] += len(out)
        return out


def main(argv=None):
    from host.inference_service import serve
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--socket", required=True)
    ap.add_argument("--device", choices=("cpu", "cuda"), default="cpu",
                    help="declared device only (must equal the runtime's --inference-device)")
    a = ap.parse_args(argv)
    det = FixedDetector(a.device)

    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        serve(a.socket, det, a.device)
    except KeyboardInterrupt:
        pass
    print(json.dumps({"stub_counts": det.counts}), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
