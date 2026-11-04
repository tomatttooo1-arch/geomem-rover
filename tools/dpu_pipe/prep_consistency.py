#!/usr/bin/env python3
"""DPUPIPE0 判据 T5：Nano 端预处理（host/dpu_prep.py）对 dev3 评测器口径逐字节核对。

参考端 = dev3 评测器取像素的同一段代码：ultralytics DetectionValidator(rect=False, imgsz=size).get_dataloader
（dev3 source/evaluate.py make_validator 的参数），得到 3 x size x size 的 uint8 RGB；再按 dev3 的写法
pixels.astype(float32)/255. 归一化，按 xmodel 输入 data-fix（fix_point 来自编译好的 xmodel）量化成 int8：
DPU_ROUND = floor(x*2^fp + 0.5)，同时用 numpy rint（四舍六入五成双）做第二种舍入，两者都要与被测端一致。
被测端 = host/dpu_prep.DpuPrep(size, fp, backend) 的 full_tensor（cv2.imread 读图后转 RGB，模拟相机 RGB 帧）。

阴性对照（证明比较有判别力，必须报出非零差）：
  neg_round: 缩放系数用浮点直算（不走 OpenCV 定点）——应有大量像素差；
  neg_pad:   灰边按 0 填（不按 114）——应有差；
  neg_fp:    fix_point 错一位——应有差。
用法：python3 prep_consistency.py --out results/prep_consistency.json IMG [IMG ...]
"""
import argparse
import json
import os
import sys
import tempfile
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from host import dpu_prep  # noqa: E402


def reference_pixels(images, size, workdir):
    """{path: size x size x 3 uint8 RGB} from the ultralytics validator dataloader (dev3's parameters)."""
    import torch
    from types import SimpleNamespace
    from ultralytics.models.yolo.detect import DetectionValidator
    from ultralytics.data.utils import check_det_dataset
    lst = os.path.join(workdir, "list_%d.txt" % size)
    with open(lst, "w") as f:
        f.write("\n".join(images) + "\n")
    yaml = os.path.join(workdir, "data_%d.yaml" % size)
    with open(yaml, "w") as f:
        f.write("path: %s\ntrain: %s\nval: %s\ntest: %s\nnames: {0: a, 1: b, 2: c, 3: d, 4: e, 5: f}\n"
                % (workdir, lst, lst, lst))
    v = DetectionValidator(args=dict(data=yaml, split="test", imgsz=size, batch=8, device="cpu", workers=0,
                                     plots=False, rect=False, conf=.001, iou=.7, max_det=300, half=False,
                                     augment=False, task="detect", save_json=False,
                                     project=os.path.join(workdir, "val"), name="p%d" % size, exist_ok=True))
    v.device = torch.device("cpu"); v.stride = 32
    v.data = check_det_dataset(yaml)
    v.init_metrics(SimpleNamespace(names=dict(enumerate("abcdef")), end2end=False))
    out = {}
    for batch in v.get_dataloader(lst, batch_size=8):
        for path, t in zip(batch["im_file"], batch["img"]):
            px = t.numpy()
            assert px.shape == (3, size, size) and px.dtype == np.uint8
            out[os.path.abspath(path)] = np.ascontiguousarray(px.transpose(1, 2, 0))
    return out


def ref_quant(px_hwc, fp, mode):
    x = px_hwc.astype(np.float32) / 255.
    y = x * np.float32(2 ** fp)
    q = np.floor(y + np.float32(0.5)) if mode == "dpu_round" else np.rint(y)
    return np.clip(q, -128, 127).astype(np.int8)


def float_bilinear(img, w, h):
    """阴性对照：half-pixel 双线性，浮点系数，四舍五入（不是 OpenCV 定点）。"""
    sh, sw = img.shape[:2]
    def axis(dn, sn):
        f = (np.arange(dn) + 0.5) * (sn / dn) - 0.5
        s = np.clip(np.floor(f).astype(int), 0, sn - 1); f = np.clip(f - s, 0, 1)
        return s, np.minimum(s + 1, sn - 1), f
    x0, x1, fx = axis(w, sw); y0, y1, fy = axis(h, sh)
    I = img.astype(np.float64)
    H = I[:, x0] * (1 - fx)[None, :, None] + I[:, x1] * fx[None, :, None]
    O = H[y0] * (1 - fy)[:, None, None] + H[y1] * fy[:, None, None]
    return np.clip(np.floor(O + 0.5), 0, 255).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--sizes", default="640,512,416")
    ap.add_argument("--fix", default="", help="size:fp,...（默认按 xmodel：640:6,512:7,416:7）")
    ap.add_argument("--backends", default="numpy,cv2")
    ap.add_argument("images", nargs="+")
    a = ap.parse_args()
    import cv2
    fixes = dict(dpu_prep.DEFAULT_FIX_POINT)
    for kv in filter(None, a.fix.split(",")):
        s, f = kv.split(":"); fixes[int(s)] = int(f)
    images = [os.path.abspath(p) for p in a.images]
    rgb = {p: cv2.imread(p, cv2.IMREAD_COLOR)[..., ::-1].copy() for p in images}
    report = {"images": len(images), "cv2": cv2.__version__, "numpy": np.__version__, "sizes": {}}
    try:
        import ultralytics
        report["ultralytics"] = ultralytics.__version__
    except Exception:
        pass
    with tempfile.TemporaryDirectory() as wd:
        for size in [int(s) for s in a.sizes.split(",")]:
            fp = fixes[size]
            ref = reference_pixels(images, size, wd)
            assert set(ref) == set(images), "dataloader lost images"
            row = {"fix_point": fp, "pad_q": int(dpu_prep.quant_lut(fp)[114]), "src_shapes": {}}
            for be in a.backends.split(","):
                prep = dpu_prep.DpuPrep(size, fp, backend=be)
                diff_px = diff_q = diff_q_rint = 0; t = []
                for p in images:
                    t0 = time.perf_counter(); full = prep.full_tensor(rgb[p]); t.append(time.perf_counter() - t0)
                    g, im = dpu_prep.resized_uint8(rgb[p], size, be)
                    pre = np.full((size, size, 3), 114, np.uint8); pre[g.top:g.top + g.h, g.left:g.left + g.w] = im
                    diff_px += int((pre != ref[p]).sum())
                    diff_q += int((full != ref_quant(ref[p], fp, "dpu_round")).sum())
                    diff_q_rint += int((full != ref_quant(ref[p], fp, "rint")).sum())
                    row["src_shapes"]["%dx%d" % (rgb[p].shape[1], rgb[p].shape[0])] = g.as_dict()
                row[be] = {"diff_uint8_bytes": diff_px, "diff_int8_bytes_dpu_round": diff_q,
                           "diff_int8_bytes_rint": diff_q_rint, "bytes_compared": len(images) * size * size * 3,
                           "prep_ms_p50_host": round(1e3 * float(np.median(t)), 3)}
            # 阴性对照
            neg = {"neg_round": 0, "neg_pad": 0, "neg_fp": 0}
            lut_bad = dpu_prep.quant_lut(fp - 1)
            for p in images:
                g = dpu_prep.LetterboxGeometry(rgb[p].shape[1], rgb[p].shape[0], size)
                im = rgb[p]
                if (g.load_w, g.load_h) != (im.shape[1], im.shape[0]):
                    im = float_bilinear(im, g.load_w, g.load_h)
                pre = np.full((size, size, 3), 114, np.uint8); pre[g.top:g.top + g.h, g.left:g.left + g.w] = im
                neg["neg_round"] += int((pre != ref[p]).sum())
                _, ok = dpu_prep.resized_uint8(rgb[p], size, "numpy")
                pre0 = np.zeros((size, size, 3), np.uint8); pre0[g.top:g.top + g.h, g.left:g.left + g.w] = ok
                neg["neg_pad"] += int((pre0 != ref[p]).sum())
                full_bad = dpu_prep.place_content(lut_bad[ok].tobytes(), g, int(lut_bad[114]))
                neg["neg_fp"] += int((full_bad != ref_quant(ref[p], fp, "dpu_round")).sum())
            row["negative_controls"] = neg
            report["sizes"][str(size)] = row
            print(json.dumps({str(size): {k: v for k, v in row.items() if k != "src_shapes"}}), flush=True)
    with open(a.out, "w") as f:
        json.dump(report, f, indent=1)


if __name__ == "__main__":
    main()
