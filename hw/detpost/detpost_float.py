"""检测后处理浮点参考（numpy，按 ultralytics YOLOv8 predict 的后处理语义）。

输入与定点模型完全相同：一帧 DPU 输出原样字节（int8，三个头按 DDR 偏移排列，NHWC，
每格 64 个 DFL logit + nc 个类别 logit）。先按 fix_point 反量化成浮点，再走 ultralytics 的步骤：
  Detect 头解码：DFL（16 个 bin 做 softmax 取期望）→ dist2bbox（xywh）× stride；类别 sigmoid
  non_max_suppression：每格取最大类别（同分取下标小的）→ 分数 > conf → 按分数降序取前 max_nms
  → 按类 NMS（torchvision 语义：IoU > iou 抑制；本文件用稳定排序，同分按锚点序）→ 前 max_det
  车上附加：NMS 后再按类别白名单与按类下限（score >= floor）过滤（perception/fpga_replay/yolo_detector.py）。

参照的上游实现：ultralytics 8.3.40 nn/modules/block.py DFL、utils/tal.py make_anchors / dist2bbox、
utils/ops.py non_max_suppression；车上 numpy 版 nms_numpy / batched_nms_numpy。
只依赖 numpy。
"""
import numpy as np

REG_MAX = 16
GRIDS = (80, 40, 20)
STRIDES = (8, 16, 32)


def default_offsets(nc):
    ch = 4 * REG_MAX + nc
    s20, s40 = 20 * 20 * ch, 40 * 40 * ch
    return (s20 + s40, s20, 0)  # 80x80 / 40x40 / 20x20 在 DDR 缓冲里的字节偏移


def gather_heads(buf, nc, offsets=None):
    """按 ultralytics 锚点顺序（P3 80x80 → P4 → P5，行优先）取出 (8400, ch) 的 int8 矩阵。"""
    ch = 4 * REG_MAX + nc
    offsets = default_offsets(nc) if offsets is None else offsets
    raw = np.frombuffer(buf, dtype=np.int8)
    parts, anc, strd = [], [], []
    for g, s, off in zip(GRIDS, STRIDES, offsets):
        n = g * g
        parts.append(raw[off:off + n * ch].reshape(n, ch))
        ys, xs = np.meshgrid(np.arange(g, dtype=np.float32), np.arange(g, dtype=np.float32), indexing="ij")
        anc.append(np.stack([xs.ravel() + 0.5, ys.ravel() + 0.5], 1))
        strd.append(np.full((n, 1), s, np.float32))
    return np.concatenate(parts), np.concatenate(anc), np.concatenate(strd)


def decode(q, anchors, strides, fp):
    """ultralytics Detect 推理解码（float32）：返回 (N,4) xywh（letterbox 像素）与 (N,nc) sigmoid 分数。"""
    x = q.astype(np.float32) * np.float32(2.0 ** -fp)
    box = x[:, :4 * REG_MAX].reshape(-1, 4, REG_MAX)
    box = box - box.max(-1, keepdims=True)          # softmax 数值稳定化（不改结果）
    e = np.exp(box)
    p = e / e.sum(-1, keepdims=True)
    dist = (p * np.arange(REG_MAX, dtype=np.float32)).sum(-1)   # DFL：softmax 期望
    lt, rb = dist[:, :2], dist[:, 2:]
    x1y1, x2y2 = anchors - lt, anchors + rb                      # dist2bbox
    xywh = np.concatenate([(x1y1 + x2y2) / 2, x2y2 - x1y1], 1) * strides
    cls = 1.0 / (1.0 + np.exp(-x[:, 4 * REG_MAX:]))
    return xywh.astype(np.float32), cls.astype(np.float32)


def nms_numpy(boxes, scores, iou):
    """贪心 NMS，torchvision.ops.nms 语义（IoU > iou 抑制），稳定降序。与车上同名函数一致。"""
    order = np.argsort(-scores, kind="stable")
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    area = (x2 - x1) * (y2 - y1)
    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        rest = order[1:]
        w = np.clip(np.minimum(x2[i], x2[rest]) - np.maximum(x1[i], x1[rest]), 0, None)
        h = np.clip(np.minimum(y2[i], y2[rest]) - np.maximum(y1[i], y1[rest]), 0, None)
        inter = w * h
        overlap = inter / (area[i] + area[rest] - inter)
        order = rest[~(overlap > iou)]
    return np.asarray(keep, dtype=np.int64)


def postprocess(buf, nc, fp=2, conf=0.25, iou=0.7, max_det=100, offsets=None,
                min_scores=None, enable=None, max_nms=30000, offset_trick=True, rank_by_level=False):
    """返回 dict：boxes (K,4) xyxy letterbox 像素、cls、score、anchor，顺序 = 分数降序。"""
    q, anchors, strides = gather_heads(buf, nc, offsets)
    xywh, cls = decode(q, anchors, strides, fp)
    # 同分取下标小的（与 torch.max 一致）。float32 sigmoid 饱和时多个类别都等于 1.0，取到的未必是 logit 最大的；
    # rank_by_level=True 时按 int8 logit 取最大类别（定点模型的做法）
    j = q[:, 4 * REG_MAX:].argmax(1) if rank_by_level else cls.argmax(1)
    sc = cls[np.arange(len(j)), j]
    k = np.nonzero(sc > conf)[0]
    # float32 的 sigmoid 在 logit >= ~16.6 时饱和成 1.0，ultralytics 在这些框之间按锚点序排；
    # rank_by_level=True 时改按 int8 logit 原值排（定点模型的排序键），用来区分这一项差别
    key = q[np.arange(len(j)), 4 * REG_MAX + j].astype(np.float32) if rank_by_level else sc
    k = k[np.argsort(-key[k], kind="stable")][:max_nms]
    b = xywh[k]
    xyxy = np.concatenate([b[:, :2] - b[:, 2:] / 2, b[:, :2] + b[:, 2:] / 2], 1)
    if len(k):
        # ultralytics 用「类别 × 7680」平移坐标来做按类 NMS；float32 下大类别号会损失约 1/64 像素精度，
        # offset_trick=False 时改为数学上等价、无精度损失的逐类 NMS（用来区分这一项误差）
        off = j[k].astype(np.float32)[:, None] * np.float32(7680.0 if offset_trick else 0.0)
        if offset_trick:
            keep = nms_numpy(xyxy + off, key[k], iou)[:max_det]
        else:
            kk = []
            for c in np.unique(j[k]):
                idx = np.nonzero(j[k] == c)[0]
                kk.extend(idx[nms_numpy(xyxy[idx], key[k][idx], iou)].tolist())
            kk = np.asarray(kk, np.int64)
            keep = kk
            # 同分时按锚点序：k 已按（分数降序、锚点序）排好，kk 是 k 内位置，按（分数降序、位置）排
            keep = np.asarray(sorted(keep.tolist(), key=lambda t: (-key[k][t], t)), np.int64)[:max_det]
    else:
        keep = np.zeros(0, np.int64)
    res = dict(boxes=xyxy[keep], cls=j[k][keep], score=sc[k][keep], anchor=k[keep], n_cand=int(len(k)))
    sel = np.ones(len(keep), bool)
    if enable is not None:
        sel &= np.isin(res["cls"], np.asarray(list(enable)))
    if min_scores:
        for c, f in min_scores.items():
            sel &= ~((res["cls"] == c) & (res["score"] < np.float32(f)))
    for key in ("boxes", "cls", "score", "anchor"):
        res[key] = res[key][sel]
    return res
