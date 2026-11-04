"""造非 640 排列的稀疏帧（网格与偏移不按 16 字节对齐，如 416 输入 52/26/13），只为覆盖核的地址与对齐逻辑。
背景类别 logit 远低于阈值；随机放若干「物体」格（含相邻格重复框，触发 NMS）。固定种子。
用法：python3 gen_layout.py --grid 52,26,13 --nc 6 --n 3 --out DIR
"""
import argparse, os
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--grid", default="52,26,13")
ap.add_argument("--nc", type=int, default=6)
ap.add_argument("--n", type=int, default=3)
ap.add_argument("--seed", type=int, default=20261103)
ap.add_argument("--out", required=True)
a = ap.parse_args()
g = [int(v) for v in a.grid.split(",")]
ch = 64 + a.nc
rng = np.random.default_rng(a.seed)
os.makedirs(a.out, exist_ok=True)
offs = [g[2] * g[2] * ch + g[1] * g[1] * ch, g[2] * g[2] * ch, 0]
total = offs[0] + g[0] * g[0] * ch
for k in range(a.n):
    buf = np.zeros(total, np.int8)
    for l in range(3):
        h = np.empty((g[l] * g[l], ch), np.int16)
        h[:, :64] = rng.integers(-40, 41, (g[l] * g[l], 64))
        h[:, 64:] = rng.integers(-60, -20, (g[l] * g[l], a.nc))
        for _ in range(int(rng.integers(5, 25))):
            x, y = int(rng.integers(0, g[l])), int(rng.integers(0, g[l]))
            c = int(rng.integers(0, a.nc)); pk = int(rng.integers(-8, 60))
            base = rng.integers(-30, 60, 64)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    xx, yy = x + dx, y + dy
                    if 0 <= xx < g[l] and 0 <= yy < g[l] and rng.random() < 0.7:
                        cell = yy * g[l] + xx
                        h[cell, :64] = np.clip(base + rng.integers(-6, 7, 64), -128, 127)
                        h[cell, 64 + c] = max(h[cell, 64 + c], pk - 6 * (abs(dx) + abs(dy)))
                        if rng.random() < 0.3:
                            h[cell, 64 + int(rng.integers(0, a.nc))] = pk - 3
        buf[offs[l]:offs[l] + h.size] = np.clip(h, -128, 127).astype(np.int8).ravel()
    with open(os.path.join(a.out, "layout_%s_nc%d_%03d.bin" % (a.grid.replace(",", "x"), a.nc, k)), "wb") as f:
        f.write(buf.tobytes())
print("offsets", offs, "total", total)
