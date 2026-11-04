"""Replay through a local inference service using the ROS-side Python environment."""
import argparse
import json
from pathlib import Path
import platform
import subprocess
import sys
import cv2
import numpy as np
from host.inference_service import InferenceClient
from host.replay import dump, run_sequence
from perception.fpga_replay.io import load_frame, load_records, sha256


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data',required=True);parser.add_argument('--manifest',required=True)
    parser.add_argument('--socket',required=True);parser.add_argument('--out',required=True)
    parser.add_argument('--query',default='keyboard');parser.add_argument('--device',choices=('cpu','cuda'),required=True)
    args=parser.parse_args();out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
    records=load_records(args.manifest)
    (out/'input.jsonl').write_bytes(Path(args.manifest).read_bytes())
    client=InferenceClient(args.socket,expected_device=args.device)
    execution=[]
    def frames():
        for record in records:
            frame=load_frame(record,args.data)
            detections=client.detect(frame)
            execution.append(client.last_metadata)
            yield frame,detections
    summary=run_sequence(frames(),out,args.query)
    dump(out/'summary.json',summary)
    dump(out/'execution.json',execution)
    dump(out/'run.json',{'command':sys.argv,'python':sys.version,'platform':platform.platform(),
                         'git_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
                         'numpy':np.__version__,'cv2':cv2.__version__,
                         'requested_inference_device':args.device,'geometry_device':'CPU in client environment',
                         'input_manifest_sha256':sha256(args.manifest),'performance_claim':False})
    dump(out/'manifest.json',{str(path.relative_to(out)):sha256(path) for path in sorted(out.iterdir()) if path.is_file()})
    print(json.dumps({key:summary[key] for key in ('frames','detections','observations')}))


if __name__=='__main__':main()
