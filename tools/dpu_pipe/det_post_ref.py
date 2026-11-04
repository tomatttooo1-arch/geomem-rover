"""DPUPIPE1 (2026-11-03): Python reference of the CPU output processing = DPUB1 full1.py (runs/.../DPUB1/scripts)
dfl_decode_f + nms, copied verbatim except: the input size H is a parameter, and nms takes the argsort kind
(numpy's default quicksort is not stable, so on equal scores its tie order is implementation-defined; the C++
port uses a stable sort, and the exact comparison uses kind="stable"; the DPUB1 default is kept as kind=None).

    python3 det_post_ref.py --size 512 --heads 64x64x70x2,32x32x70x2,16x16x70x2 --in outs.bin [--kind stable]
prints {"dets": [[x1, y1, x2, y2, score, cls], ...]} with float32 values written exactly (repr of float).
"""
import argparse
import json

import numpy as np


def dfl_decode_f(fl, H):   # same result, threshold first: class logit > logit(0.25) on raw logits, DFL only on survivors
    res = []; th = np.log(0.25 / 0.75)
    for f in fl:
        _, h, w, C = f.shape; nc = C - 64; stride = H // h; f2 = f[0].reshape(h * w, C)
        lg = f2[:, 64:]; mx = lg.max(1); idx = np.nonzero(mx > th)[0]
        if idx.size == 0: continue
        box = f2[idx, :64].reshape(-1, 4, 16); box = np.exp(box - box.max(-1, keepdims=True)); box /= box.sum(-1, keepdims=True)
        d = (box * np.arange(16, dtype=np.float32)).sum(-1); gy = (idx // w).astype(np.float32) + 0.5; gx = (idx % w).astype(np.float32) + 0.5
        sc = 1 / (1 + np.exp(-mx[idx])); ci = lg[idx].argmax(1).astype(np.float32)
        res.append(np.stack([(gx - d[:, 0]) * stride, (gy - d[:, 1]) * stride, (gx + d[:, 2]) * stride, (gy + d[:, 3]) * stride, sc, ci], 1))
    return (np.concatenate(res, 0) if res else np.zeros((0, 6), np.float32)), (fl[0].shape[3] - 64 if fl else 0)


def nms(d, thr=0.7, kind=None):   # greedy per-class NMS in numpy (class offset trick)
    if len(d) == 0: return d
    off = d[:, 5:6] * 4096.0; b = d[:, :4] + off; o = np.argsort(-d[:, 4], kind=kind); keep = []
    area = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    while o.size:
        i = o[0]; keep.append(i)
        xx1 = np.maximum(b[i, 0], b[o[1:], 0]); yy1 = np.maximum(b[i, 1], b[o[1:], 1])
        xx2 = np.minimum(b[i, 2], b[o[1:], 2]); yy2 = np.minimum(b[i, 3], b[o[1:], 3])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        iou = inter / (area[i] + area[o[1:]] - inter + 1e-9); o = o[1:][iou <= thr]
    return d[keep]


def parse_heads(spec):
    out = []
    for item in spec.split(","):
        h, w, c, fp = (int(x) for x in item.split("x"))
        out.append((h, w, c, fp))
    return out


def heads_from_bytes(buf, heads):
    outs, off = [], 0
    for h, w, c, fp in heads:
        n = h * w * c
        outs.append((np.frombuffer(buf, np.int8, n, off).reshape(1, h, w, c), fp))
        off += n
    assert off == len(buf), (off, len(buf))
    return outs


def run(outs, H, kind=None):
    fl = [o.astype(np.float32) * (2.0 ** -fp) for o, fp in outs]
    d, _ = dfl_decode_f(fl, H)
    return d, nms(d, kind=kind)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", type=int, required=True)
    ap.add_argument("--heads", required=True)
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--kind", default=None)
    a = ap.parse_args()
    outs = heads_from_bytes(open(a.inp, "rb").read(), parse_heads(a.heads))
    d, det = run(outs, a.size, a.kind)
    print(json.dumps({"candidates": int(len(d)), "dets": [[float(x) for x in row] for row in det]}))


if __name__ == "__main__":
    main()
