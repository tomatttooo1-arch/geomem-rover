"""Actual detector on captured simulation pixels; no object truth or fake pose."""
import argparse
import json
from pathlib import Path
import subprocess
import cv2
import numpy as np
from perception.fpga_replay.core import Frame
from perception.fpga_replay.detector import SSDDetector
from perception.fpga_replay.io import sha256


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',required=True);p.add_argument('--models',required=True)
    p.add_argument('--device',choices=('cpu','cuda'),default='cpu');p.add_argument('--out',required=True)
    args=p.parse_args();root=Path(args.input);out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
    info=json.loads((root/'camera_info.json').read_text())
    for name,digest in info['sha256'].items():
        if sha256(root/name)!=digest:raise ValueError('capture checksum mismatch')
    stamp=info['timestamp']['sec']+info['timestamp']['nanosec']/1e9
    rgb=cv2.cvtColor(cv2.imread(str(root/'rgb.png')),cv2.COLOR_BGR2RGB)
    frame=Frame('capture-'+str(stamp),stamp,rgb,np.load(root/'depth_optical_z_m.npy',allow_pickle=False),
                np.asarray(info['K']).reshape(3,3),np.eye(4),False,
                'not_available_detection_only_no_memory_update','capture-camera-info',stamp)
    detector=SSDDetector(args.models,device=args.device)
    boxes=detector(frame)
    image=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
    for box in boxes:
        x1,y1,x2,y2=map(int,box.bbox)
        cv2.rectangle(image,(x1,y1),(x2,y2),(0,255,0),1)
        cv2.putText(image,box.label+' %.3f'%box.score,(x1,max(12,y1)),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,0),1)
    if not cv2.imwrite(str(out/'detections.png'),image):raise IOError('evidence image write failed')
    result={'source':str(root),'source_rgb_sha256':info['sha256']['rgb.png'],
            'git_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
            'execution':detector.execution,'detections':[box.__dict__ for box in boxes],
            'scope':'image detection only; no navigation, pose or instance truth supplied'}
    (out/'detections.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result))


if __name__=='__main__':main()
