"""背景量：DPU 输出头本身的 int8 量化（fix_point=2，步长 0.25）对后处理结果的影响。
与检测后处理核无关（核吃的就是量化后的字节），只用来说明「定点后处理误差」相对「头量化误差」的量级。
输入 gen_inputs.py model 生成的 *_float.npz（未量化浮点头）与同名 .bin（量化后）。"""
import glob, json, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import detpost_float as d


def run_float_heads(npz, nc, conf=0.25, iou=0.7, max_det=100):
    z = np.load(npz)
    q = np.concatenate([z["p3"], z["p4"], z["p5"]]).astype(np.float32)
    _, anc, st = d.gather_heads(open(npz.replace("_float.npz", ".bin"), "rb").read(), nc)
    # decode() 先乘 2^-fp；这里传 fp=0 直接用浮点值
    xywh, cls = d.decode(q, anc, st, 0)
    j = cls.argmax(1); sc = cls[np.arange(len(j)), j]
    k = np.nonzero(sc > conf)[0]; k = k[np.argsort(-sc[k], kind="stable")]
    b = xywh[k]; xyxy = np.concatenate([b[:, :2] - b[:, 2:] / 2, b[:, :2] + b[:, 2:] / 2], 1)
    keep = d.nms_numpy(xyxy + j[k].astype(np.float32)[:, None] * 7680, sc[k], iou)[:max_det] if len(k) else np.zeros(0, int)
    return dict(boxes=xyxy[keep], cls=j[k][keep], score=sc[k][keep], anchor=k[keep])


def main(frames, nc=33):
    tot = dict(frames=0, n_float_heads=0, n_quant_heads=0, same_anchor=0, matched_iou50=0, coord_err=[])
    for npz in sorted(glob.glob(os.path.join(frames, "*_float.npz"))):
        a = run_float_heads(npz, nc)
        b = d.postprocess(open(npz.replace("_float.npz", ".bin"), "rb").read(), nc)
        tot["frames"] += 1; tot["n_float_heads"] += len(a["anchor"]); tot["n_quant_heads"] += len(b["anchor"])
        tot["same_anchor"] += len(set(a["anchor"].tolist()) & set(b["anchor"].tolist()))
        for bb, cc in zip(b["boxes"], b["cls"]):  # 按同类最大 IoU 配对
            m = a["cls"] == cc
            if not m.any():
                continue
            A = a["boxes"][m]
            iw = np.clip(np.minimum(A[:, 2], bb[2]) - np.maximum(A[:, 0], bb[0]), 0, None)
            ih = np.clip(np.minimum(A[:, 3], bb[3]) - np.maximum(A[:, 1], bb[1]), 0, None)
            inter = iw * ih
            iou = inter / ((A[:, 2] - A[:, 0]) * (A[:, 3] - A[:, 1]) + (bb[2] - bb[0]) * (bb[3] - bb[1]) - inter)
            i = int(iou.argmax())
            if iou[i] >= 0.5:
                tot["matched_iou50"] += 1
                tot["coord_err"].append(float(np.abs(A[i] - bb).max()))
    ce = np.asarray(tot.pop("coord_err") or [0.0])
    tot.update(coord_err_median=float(np.median(ce)), coord_err_p90=float(np.percentile(ce, 90)), coord_err_max=float(ce.max()))
    print(json.dumps(tot))


if __name__ == "__main__":
    main(sys.argv[1])
