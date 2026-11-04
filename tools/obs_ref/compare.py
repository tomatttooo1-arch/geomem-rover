# C 参考 vs Python process_frame：逐 (帧,框) 比"是否出观测 / 位置 / 范围 / 颜色 / 截断"
# usage: python3 compare.py GOLD_PY C_OUT C_CSV INDEX_JSON
import sys, json, struct, csv
import numpy as np
COLORS = ["black", "white", "gray", "red", "yellow", "green", "blue", "purple"]
FMT = "<IIiiii" + "d" * 13


def load(p):
    b = open(p, "rb").read(); assert b[:8] == b"OBSG0001"
    n = struct.unpack_from("<I", b, 8)[0]
    return [struct.unpack_from(FMT, b, 12 + 128 * i) for i in range(n)]


g, c = load(sys.argv[1]), load(sys.argv[2])
diag = list(csv.DictReader(open(sys.argv[3])))
idx = json.load(open(sys.argv[4]))
assert len(g) == len(c) == len(diag) == len(idx)
bad, maxpos, maxrng, maxoth, n_obs = [], 0.0, 0.0, 0.0, 0
for i, (a, b) in enumerate(zip(g, c)):
    assert a[:2] == b[:2]
    why = []
    if a[2] != b[2]:
        why.append(f"出观测不同 py={a[2]} c={b[2]} (c status={diag[i]['status']})")
    elif a[2]:
        n_obs += 1
        va, vb = np.array(a[6:]), np.array(b[6:])
        d = np.abs(va - vb)
        maxpos, maxrng, maxoth = max(maxpos, d[:3].max()), max(maxrng, d[3:9].max()), max(maxoth, d[9:].max())
        if d[:9].max() > 1e-3: why.append(f"位置/范围差 {d[:9].max()*1e3:.3f} mm")
        if a[3] != b[3]: why.append(f"颜色 py={COLORS[a[3]]} c={COLORS[b[3]]} (hsv中位 {diag[i]['h_med']},{diag[i]['s_med']},{diag[i]['v_med']})")
        if a[4] != b[4]: why.append("截断标志不同")
        if d[9:].max() > 1e-6: why.append(f"有效比例/外形/距离差 {d[9:].max():.3g}")
    if why:
        bad.append((i, idx[i], why))
print(f"对数 {len(g)}，Python 出观测 {sum(a[2] for a in g)}，C 出观测 {sum(b[2] for b in c)}，双方都出 {n_obs}")
print(f"最大差：位置 {maxpos:.3g} m，范围 {maxrng:.3g} m，有效比例/外形/距离 {maxoth:.3g}")
from collections import Counter
print("C 不出观测的原因分布（status：1 内缩后空 2 有效深度不足 3 无前景 4 最大连通域过小）:",
      dict(Counter(d['status'] for d in diag if d['status'] != '0')))
print("颜色分布(C):", dict(Counter(COLORS[b[3]] for b in c if b[2])))
print("位姿分布:", dict(Counter(x['pose_kind'] for x in idx)))
print(f"不一致 {len(bad)} 条")
for i, ix, why in bad:
    print(f"  #{i} 帧{ix['rec_frame']} 框{ix['box']} {ix['label']} {ix['bbox']}: " + "; ".join(why))
sys.exit(1 if bad else 0)
