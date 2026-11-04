"""生成检测后处理的测试输入（一帧 = 一个 .bin，按 DPU 输出缓冲的原样字节排列）。

三种来源（README §7.1）：
  random  ：int8 均匀随机。几乎每格都过阈值，专门压「候选溢出」路径，不代表真实场景。
  scene   ：合成场景。背景低分；按物体大小选尺度，物体中心附近若干格给高类别分和以真实距离为峰的
            DFL 分布，跨尺度也放重复框，专门压 NMS。
  model   ：真实图片过 dev3 的浮点 YOLOv8n-LeakyReLU（raw-graph，与编进 xmodel 的是同一张图），
            三个头按 xmodel 的 fix_point 量化成 int8、按 xmodel 的 DDR 偏移排好。
            这是「仿真 DPU 输出」：DPU 里每层都量化，真实输出会再有差别；真实输出要在板上用
            bench/dump_dpu_out.py 抓。
用法：
  python3 gen_inputs.py random --n 4 --out DIR
  python3 gen_inputs.py scene  --n 40 --out DIR
  python3 gen_inputs.py model  --images a.png b.jpg ... --raw-graph DIR --out DIR   （需要 torch、opencv）
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from detpost_float import GRIDS, REG_MAX, STRIDES, default_offsets  # noqa: E402


def _write(out, name, buf, meta, manifest):
    path = os.path.join(out, name + ".bin")
    with open(path, "wb") as f:
        f.write(buf.tobytes())
    meta = dict(meta, file=name + ".bin", bytes=int(buf.size))
    manifest.append(meta)


def _assemble(heads, nc):
    """heads: 三个 (g*g, ch) int8（80/40/20 顺序）→ 按 DDR 偏移拼成一个缓冲。"""
    ch = 4 * REG_MAX + nc
    offs = default_offsets(nc)
    total = max(o + g * g * ch for o, g in zip(offs, GRIDS))
    buf = np.zeros(total, np.int8)
    for h, o in zip(heads, offs):
        buf[o:o + h.size] = h.ravel()
    return buf


def gen_random(rng, nc):
    ch = 4 * REG_MAX + nc
    return _assemble([rng.integers(-128, 128, (g * g, ch), dtype=np.int16).astype(np.int8) for g in GRIDS], nc)


def gen_scene(rng, nc, fp, n_obj):
    ch = 4 * REG_MAX + nc
    sc = 2.0 ** fp
    heads = []
    for g in GRIDS:
        h = np.empty((g * g, ch), np.float32)
        h[:, :4 * REG_MAX] = rng.normal(0.0, 1.0, (g * g, 4 * REG_MAX))
        h[:, 4 * REG_MAX:] = rng.normal(-9.0, 1.5, (g * g, nc))   # 背景：sigmoid 约 1e-4
        heads.append(h)
    objs = []
    for _ in range(n_obj):
        w, hh = rng.uniform(20, 400), rng.uniform(20, 400)
        cx, cy = rng.uniform(w / 2, 640 - w / 2), rng.uniform(hh / 2, 640 - hh / 2)
        c = int(rng.integers(0, nc))
        peak = rng.uniform(-0.5, 5.0)            # 中心格的类别 logit（浮点）
        lv = 0 if max(w, hh) < 96 else (1 if max(w, hh) < 224 else 2)
        levels = [lv] + ([lv + 1] if lv < 2 and rng.random() < 0.5 else [])
        for l in levels:
            g, s = GRIDS[l], STRIDES[l]
            gx, gy = cx / s, cy / s
            r = 1.6 if l == lv else 0.9
            for y in range(max(0, int(gy - 2)), min(g, int(gy + 3))):
                for x in range(max(0, int(gx - 2)), min(g, int(gx + 3))):
                    ax, ay = x + 0.5, y + 0.5
                    dd = np.hypot(ax - gx, ay - gy)
                    if dd > r:
                        continue
                    a = y * g + x
                    tgt = np.array([ax - (cx - w / 2) / s, ay - (cy - hh / 2) / s,
                                    (cx + w / 2) / s - ax, (cy + hh / 2) / s - ay])
                    tgt = np.clip(tgt + rng.normal(0, 0.15, 4), 0, REG_MAX - 1.01)
                    sharp = rng.uniform(0.6, 3.0)
                    bins = np.arange(REG_MAX)
                    for t in range(4):
                        heads[l][a, t * REG_MAX:(t + 1) * REG_MAX] = 4.0 - sharp * (bins - tgt[t]) ** 2
                    heads[l][a, 4 * REG_MAX + c] = max(heads[l][a, 4 * REG_MAX + c],
                                                       peak - 2.0 * dd - (0 if l == lv else 1.0))
                    if rng.random() < 0.2:      # 偶尔第二类别也较高（考验取最大类别）
                        c2 = int(rng.integers(0, nc))
                        heads[l][a, 4 * REG_MAX + c2] = max(heads[l][a, 4 * REG_MAX + c2], peak - 2.5 * dd - 0.5)
        objs.append(dict(cx=cx, cy=cy, w=w, h=hh, cls=c, level=lv))
    q = [np.clip(np.floor(h * sc + 0.5), -128, 127).astype(np.int8) for h in heads]
    return _assemble(q, nc), objs


def gen_model(images, raw_graph, nc, fp, threads):
    """真实图片 → dev3 的浮点 RawYOLO（tools/raw_yolo_load.py，与 xmodel 同一张图）→ 量化成 int8 头。
    letterbox 用 dev3 的 preprocess（她已断言与 ultralytics LetterBox 逐像素相同）。"""
    import torch
    torch.set_num_threads(threads)
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "tools"))
    from raw_yolo_load import load_raw, preprocess
    model, _spec = load_raw(raw_graph)
    sc = 2.0 ** fp
    for path in images:
        x = torch.from_numpy(preprocess(path))
        with torch.inference_mode():
            outs = model(x)
        fheads = [o[0].permute(1, 2, 0).reshape(-1, o.shape[1]).numpy().copy() for o in outs]
        if fheads[0].shape[1] != 4 * REG_MAX + nc:
            raise SystemExit("head channels %d != 64+nc" % fheads[0].shape[1])
        # DPU_ROUND 按「加 0.5 向下取整」近似，饱和到 int8
        q = [np.clip(np.floor(h * sc + 0.5), -128, 127).astype(np.int8) for h in fheads]
        yield path, _assemble(q, nc), fheads


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("kind", choices=["random", "scene", "model"])
    ap.add_argument("--n", type=int, default=4)
    ap.add_argument("--nc", type=int, default=33)
    ap.add_argument("--fp", type=int, default=2)
    ap.add_argument("--seed", type=int, default=20261103)
    ap.add_argument("--out", required=True)
    ap.add_argument("--images", nargs="*", default=[])
    ap.add_argument("--raw-graph", default="${FPGA_SHARED}/models/20261103-dev3-recog-quant/dpu/raw-graph")
    ap.add_argument("--threads", type=int, default=4)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rng = np.random.default_rng(a.seed)
    manifest = []
    if a.kind == "random":
        for i in range(a.n):
            _write(a.out, "random_%03d" % i, gen_random(rng, a.nc), dict(kind="random", seed=a.seed, idx=i), manifest)
    elif a.kind == "scene":
        for i in range(a.n):
            n_obj = int(rng.integers(1, 25))
            buf, objs = gen_scene(rng, a.nc, a.fp, n_obj)
            _write(a.out, "scene_%03d" % i, buf, dict(kind="scene", seed=a.seed, idx=i, objects=objs), manifest)
    else:
        for i, (path, buf, fheads) in enumerate(gen_model(a.images, a.raw_graph, a.nc, a.fp, a.threads)):
            name = "model_%03d" % i
            np.savez_compressed(os.path.join(a.out, name + "_float.npz"), p3=fheads[0], p4=fheads[1], p5=fheads[2])
            _write(a.out, name, buf, dict(kind="model", image=path), manifest)
    with open(os.path.join(a.out, "manifest_%s.json" % a.kind), "w") as f:
        json.dump(dict(nc=a.nc, fix_point=a.fp, offsets=default_offsets(a.nc), frames=manifest), f, indent=1)
    print("wrote %d frames to %s" % (len(manifest), a.out))


if __name__ == "__main__":
    main()
