"""KV260 上：用 VART 跑编译好的 xmodel，抓「真实 DPU 输出」字节并量 DPU 推理时延（价值闸的分母）。
未在板上跑过（本包零板端）；第一次上板先用 --n 1 试，核对打印的张量名/形状/fix_point 与 README §3 一致。

产物（--out 目录）：
  dpu_000.bin ...    每帧三个输出头按 xmodel 的 DDR 排列拼成的原样字节（与 gen_inputs.py 同格式，可直接喂
                     detpost_cli / bench_detpost / compare.py）
  dpu_meta.json      输出张量名、dims、fix_point、ddr_addr（若可读），输入 fix_point
  dpu_timing.json    每帧 execute_async+wait 墙钟（ms）与分位数——T_dpu
用法：python3 dump_dpu_out.py --xmodel y8n_33_b1024.xmodel --images a.jpg b.jpg ... --out DIR [--repeat 50]
依赖：板上的 xir、vart Python 绑定，opencv-python。
"""
import argparse
import json
import os
import time

import numpy as np


def letterbox(path, size=640):
    import cv2
    image = cv2.imread(path)
    if image is None:
        raise SystemExit("unreadable image: " + path)
    h, w = image.shape[:2]
    r = min(size / h, size / w)
    unpad = (round(w * r), round(h * r))
    if (w, h) != unpad:
        image = cv2.resize(image, unpad, interpolation=cv2.INTER_LINEAR)
    dw, dh = (size - unpad[0]) / 2, (size - unpad[1]) / 2
    image = cv2.copyMakeBorder(image, round(dh - .1), round(dh + .1), round(dw - .1), round(dw + .1),
                               cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return image[:, :, ::-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xmodel", required=True)
    ap.add_argument("--images", nargs="+", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--repeat", type=int, default=50, help="计时用：对第一张图重复执行的次数")
    a = ap.parse_args()
    import vart
    import xir
    os.makedirs(a.out, exist_ok=True)
    g = xir.Graph.deserialize(a.xmodel)
    subs = [s for s in g.get_root_subgraph().toposort_child_subgraph()
            if s.has_attr("device") and s.get_attr("device").upper() == "DPU"]
    assert len(subs) == 1, "expect exactly one DPU subgraph, got %d" % len(subs)
    runner = vart.Runner.create_runner(subs[0], "run")
    it, ots = runner.get_input_tensors(), runner.get_output_tensors()
    in_fp = it[0].get_attr("fix_point")
    meta = dict(input=dict(name=it[0].name, dims=list(it[0].dims), fix_point=in_fp), outputs=[])
    for t in ots:
        m = dict(name=t.name, dims=list(t.dims), fix_point=t.get_attr("fix_point"))
        for k in ("ddr_addr", "reg_id"):
            if t.has_attr(k):
                m[k] = t.get_attr(k)
        meta["outputs"].append(m)
    print(json.dumps(meta, indent=1))
    # 按格子数排成 20x20 / 40x40 / 80x80，并按 README §3 的 DDR 偏移拼（与 gen_inputs.default_offsets 一致）
    order = sorted(range(len(ots)), key=lambda i: ots[i].dims[1])
    ch = ots[0].dims[3]
    sizes = [ots[i].dims[1] * ots[i].dims[2] * ch for i in order]
    offs = [0, sizes[0], sizes[0] + sizes[1]]
    meta["concat_offsets_20_40_80"] = offs
    inp = [np.empty(tuple(it[0].dims), dtype=np.int8, order="C")]
    outs = [np.empty(tuple(t.dims), dtype=np.int8, order="C") for t in ots]
    scale_in = 2.0 ** in_fp
    times = []
    for idx, path in enumerate(a.images):
        x = letterbox(path).astype(np.float32) / 255.0
        inp[0][0] = np.clip(np.floor(x * scale_in + 0.5), -128, 127).astype(np.int8)
        t0 = time.perf_counter()
        job = runner.execute_async(inp, outs)
        runner.wait(job)
        times.append((time.perf_counter() - t0) * 1e3)
        buf = np.concatenate([outs[i].reshape(-1) for i in order])
        buf.tofile(os.path.join(a.out, "dpu_%03d.bin" % idx))
    for _ in range(a.repeat):
        t0 = time.perf_counter()
        job = runner.execute_async(inp, outs)
        runner.wait(job)
        times.append((time.perf_counter() - t0) * 1e3)
    t = np.asarray(times[len(a.images):] or times)
    timing = dict(n=int(t.size), ms_p50=float(np.percentile(t, 50)), ms_p90=float(np.percentile(t, 90)),
                  ms_p99=float(np.percentile(t, 99)), ms_max=float(t.max()), all_ms=[round(v, 3) for v in times])
    meta["images"] = a.images
    json.dump(meta, open(os.path.join(a.out, "dpu_meta.json"), "w"), indent=1)
    json.dump(timing, open(os.path.join(a.out, "dpu_timing.json"), "w"), indent=1)
    print(json.dumps({k: v for k, v in timing.items() if k != "all_ms"}))


if __name__ == "__main__":
    main()
