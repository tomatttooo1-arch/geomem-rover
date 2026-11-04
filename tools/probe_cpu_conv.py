"""Known-answer convolution and thread/layout factorial; diagnostic only."""
import argparse
import json
from pathlib import Path
import torch
import numpy as np
from perception.fpga_replay.detector import SSDDetector
from perception.fpga_replay.io import load_frame,load_records
from torchvision.transforms.functional import to_tensor


def main():
    p=argparse.ArgumentParser();p.add_argument('--data',required=True);p.add_argument('--models',required=True);p.add_argument('--out',required=True)
    args=p.parse_args();out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
    f=load_frame(load_records(Path(args.data)/'input.jsonl')[0],args.data)
    d=SSDDetector(args.models)
    with torch.inference_mode():
        images,_=d.model.transform([d.transform(to_tensor(f.rgb))],None)
        x=images.tensors;w=d.model.backbone.features[0][0][0].weight.detach()
        # Independent NumPy direct spatial sum, no torch conv implementation.
        a=np.pad(x.numpy(),((0,0),(0,0),(1,1),(1,1)))
        reference=np.zeros((1,w.shape[0],160,160),np.float32)
        for i in range(3):
            for j in range(3):
                for c in range(3):
                    reference+=a[:,c:c+1,i:i+320:2,j:j+320:2]*w.numpy()[None,:,c,i,j,None,None]
        rows=[]
        for threads in (1,2):
            torch.set_num_threads(threads)
            for layout in ('contiguous','channels_last'):
                memory_format=torch.contiguous_format if layout=='contiguous' else torch.channels_last
                xx=x.contiguous(memory_format=memory_format)
                ww=w.contiguous(memory_format=memory_format)
                y=torch.nn.functional.conv2d(xx,ww,stride=2,padding=1)
                yy=y.numpy();finite=np.isfinite(yy)
                ones=torch.nn.functional.conv2d(torch.ones(1,3,8,8).contiguous(memory_format=memory_format),torch.ones(2,3,3,3).contiguous(memory_format=memory_format))
                row={'threads':threads,'layout':layout,'finite':int(finite.sum()),'elements':int(yy.size),
                     'max_error_on_finite':float(np.abs(yy[finite]-reference[finite]).max()) if finite.any() else None,
                     'known_ones_correct':bool(torch.all(ones==27))}
                rows.append(row)
        result={'torch':torch.__version__,'input_finite':bool(torch.isfinite(x).all()),'weights_finite':bool(torch.isfinite(w).all()),
                'reference_finite':bool(np.isfinite(reference).all()),'cases':rows}
    (out/'probe.json').write_text(json.dumps(result,indent=2,allow_nan=False));print(json.dumps(result))


if __name__=='__main__':main()
