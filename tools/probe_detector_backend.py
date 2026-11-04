"""One-frame differentiating probe for CPU/CUDA deployment failures, not timing."""
import argparse
import json
from pathlib import Path
import subprocess
import torch
from perception.fpga_replay.detector import SSDDetector
from perception.fpga_replay.io import load_frame, load_records


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data",required=True);parser.add_argument("--models",required=True)
    parser.add_argument("--device",choices=("cpu","cuda"),required=True)
    parser.add_argument("--out",required=True)
    parser.add_argument("--channels-last",action="store_true")
    parser.add_argument("--strict-fp32",action="store_true")
    args=parser.parse_args()
    out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
    frame=load_frame(load_records(Path(args.data)/"input.jsonl")[0],args.data)
    detector=SSDDetector(args.models,device=args.device)
    if args.channels_last:
        detector.model.to(memory_format=torch.channels_last)
    if args.strict_fp32:
        torch.backends.cudnn.allow_tf32=False
        torch.backends.cuda.matmul.allow_tf32=False
    rows=[]
    def describe(tensor):
        t=tensor.detach()
        finite=torch.isfinite(t)
        values=t[finite].float()
        return {"shape":list(t.shape),"finite":int(finite.sum()),"elements":t.numel(),
                "min":float(values.min()) if values.numel() else None,
                "max":float(values.max()) if values.numel() else None,
                "mean":float(values.mean()) if values.numel() else None}
    def hook(name):
        def capture(module,inputs,output):
            if isinstance(output,torch.Tensor):
                rows.append({"module":name,"type":type(module).__name__,**describe(output)})
        return capture
    handles=[module.register_forward_hook(hook(name)) for name,module in detector.model.named_modules()
             if name and not list(module.children())]
    detections=detector(frame)
    for h in handles:h.remove()
    result={"git_commit":subprocess.check_output(["git","rev-parse","HEAD"],text=True).strip(),
            "torch":torch.__version__,"device":args.device,"frame_id":frame.frame_id,
            "channels_last":args.channels_last,"strict_fp32":args.strict_fp32,
            "layers":rows,"detections":[d.__dict__ for d in detections]}
    (out/"probe.json").write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps({"device":args.device,"detections":len(detections),
                      "first_nonfinite":next((r for r in rows if r['finite']!=r['elements']),None)}))


if __name__=="__main__":main()
