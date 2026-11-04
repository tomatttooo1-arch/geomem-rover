"""TSDF 底图细结构补回（椅腿/桌腿）——便宜后处理。

思路：TSDF（截断 8 cm、w≥2）把约 2 cm 粗的竖直细杆平均/掏空掉（路过的射线把细杆体素写成“空”）。
原始逐帧点云里这些杆子其实在，只是混着飞点。所以：
  1) 融合循环里顺手把每个关键帧深度反投影到 1 cm 格，记录每格被几个**不同关键帧**看到（KfSupport）；
  2) thin_fill：取 ≥min_kf 个关键帧支持、离 TSDF 已有格 > clear_r 格、高于地面的格作候选；
     候选按 26 连通（先 1 格膨胀桥接）分块，只留“竖直细长、够大”的块（z 跨度 ≥ min_zext、
     水平跨度 ≤ max_xyext、格数 ≥ min_cells），补进 TSDF 格集合。
     不满足形状的块（飞点拖尾、地面/墙面厚层碎片）一律不补，所以不带回现行方法的漂浮碎块。

纯函数：thin_fill(...) 只吃数组吐数组；KfSupport 是融合循环里的累加器（make_basemap 唯一要多做的事）。

最小用法（挂到 make_basemap 的 TSDF 循环）：
    sup = KfSupport(voxel=0.01, depth_max=3.0)
    for 每个关键帧:
        ... vbg.integrate(...)                 # 原样
        sup.add(depth_raw_mm, rgb, K, T_map_cam)  # 原始深度（**不要**用二阶差分剔过的，剔点会把细杆削掉）
    sidx, srgb, snkf = sup.result()
    idx, rgb, counts = build_cells(...)          # TSDF 格（原样）
    idx, rgb, info = thin_fill(idx, rgb, sidx, srgb, snkf, traj_xy=traj)
"""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

_OFF = np.int64(1 << 20)   # 格索引平移到正数后打包成 int64 键（每轴 21 位，够 ±10 km@1 cm）


def _key(idx):
    i = idx.astype(np.int64) + _OFF
    return (i[:, 0] << 42) | (i[:, 1] << 21) | i[:, 2]


def _unkey(k):
    m = (1 << 21) - 1
    return np.stack([(k >> 42) & m, (k >> 21) & m, k & m], 1) - _OFF


class KfSupport:
    """逐关键帧累加：每帧深度（uint16 mm，已与彩色对齐）反投影到 map 系 1 cm 格，去重后记“这帧看到过这格”。
    result() -> (idx int64 N×3 格索引 floor(xyz/voxel), rgb uint8 N×3 格内平均色, nkf int32 N 不同关键帧数)。
    stride=2 隔像素取（1 cm 格、3 m 内 2 px≈1.2 cm，不漏 2 cm 杆）；只留 z>z_min 的格（地面不需要补）。"""

    def __init__(self, voxel=0.01, depth_max=3.0, stride=2, z_min=0.03, z_max=1.5):
        self.v, self.dmax, self.s, self.zmin, self.zmax = voxel, depth_max, stride, z_min, z_max
        self.keys, self.cols = [], []
        self._grid = None

    def add(self, depth_mm, rgb, K, T_map_cam):
        s = self.s
        d = depth_mm[::s, ::s].astype(np.float32) * 1e-3
        if self._grid is None or self._grid[0].shape != d.shape:
            h, w = depth_mm.shape
            v, u = np.mgrid[0:h:s, 0:w:s].astype(np.float32)
            self._grid = (u, v)
        u, v = self._grid
        m = (d > 0) & (d < self.dmax)
        z = d[m]
        x = (u[m] - K[0, 2]) / K[0, 0] * z
        y = (v[m] - K[1, 2]) / K[1, 1] * z
        P = np.stack([x, y, z], 1) @ T_map_cam[:3, :3].T.astype(np.float32) + T_map_cam[:3, 3].astype(np.float32)
        ok = (P[:, 2] > self.zmin) & (P[:, 2] < self.zmax)
        P = P[ok]
        c = rgb[::s, ::s][m][ok]
        k = _key(np.floor(P / self.v).astype(np.int64))
        uk, inv = np.unique(k, return_inverse=True)
        acc = np.stack([np.bincount(inv, c[:, j].astype(np.float32), len(uk)) for j in range(3)], 1)
        cnt = np.bincount(inv, minlength=len(uk))[:, None]
        self.keys.append(uk); self.cols.append((acc / cnt).astype(np.float32))

    def result(self):
        if not self.keys:
            return np.zeros((0, 3), np.int64), np.zeros((0, 3), np.uint8), np.zeros(0, np.int32)
        k = np.concatenate(self.keys); c = np.concatenate(self.cols)
        uk, inv, n = np.unique(k, return_inverse=True, return_counts=True)
        acc = np.stack([np.bincount(inv, c[:, j], len(uk)) for j in range(3)], 1)
        rgb = np.floor(acc / n[:, None] + 0.5).clip(0, 255).astype(np.uint8)
        return _unkey(uk), rgb, n.astype(np.int32)


def thin_fill(tsdf_idx, tsdf_rgb, sup_idx, sup_rgb, sup_nkf, traj_xy=None, crop_m=2.0, voxel=0.01,
              min_kf=1, clear_r=3, z_min=0.03, bridge=1, min_cells=30, min_zext=0.12, max_xyext=0.15,
              min_elong=2.0, col_xy=2, min_run=0.10, run_gap=2, min_piece=40):
    """纯函数。输入 TSDF 格（idx N×3 int，rgb N×3 uint8）＋关键帧支持格（KfSupport.result()）。
    输出 (idx int16 M×3, rgb uint8 M×3, info dict)，M = N + 补回格数；原 TSDF 格顺序与内容不变，补回格追加在后。
    参数：min_kf 最少不同关键帧；clear_r 候选离任一 TSDF 格的最小距离（格，>clear_r 才算 TSDF 漏掉的）；
    z_min 只补高于地面的；bridge 分块前膨胀格数（桥接 1 格断口）；块筛：格数≥min_cells、z 跨度≥min_zext m、
    水平外接框长边≤max_xyext m、z 跨度/水平长边≥min_elong（竖直细长）；min_piece 补回格自身 26 连通片的最小格数。
    竖直段预筛（min_run>0 时）：按 col_xy×col_xy 格的竖柱，z 方向允许 run_gap 格断口连成段，只留段高≥min_run m 的格——
    地面噪声层/斜射线拖尾每柱只占 1–3 层，竖杆每柱占满一段，这一步把二者分开，再分块。"""
    tsdf_idx = np.asarray(tsdf_idx); info = {}
    m = (sup_nkf >= min_kf) & ((sup_idx[:, 2] + 0.5) * voxel > z_min)
    ci, cc = sup_idx[m], sup_rgb[m]
    info["cand_kf"] = int(m.sum())
    if traj_xy is not None and len(ci):
        d, _ = cKDTree(np.asarray(traj_xy)[:, :2]).query((ci[:, :2] + 0.5) * voxel, k=1, workers=-1)
        ci, cc = ci[d < crop_m], cc[d < crop_m]
    if len(ci):   # 离 TSDF 已有格 > clear_r 格（切比雪夫邻域判断用 kd 树欧氏距离近似）
        d, _ = cKDTree(tsdf_idx.astype(np.float32), balanced_tree=False, compact_nodes=False).query(
            ci.astype(np.float32), k=1, distance_upper_bound=clear_r + 0.5, workers=-1)
        far = ~np.isfinite(d)
        ci, cc = ci[far], cc[far]
    info["cand_far"] = int(len(ci))
    if min_run > 0 and len(ci):
        c2 = np.floor_divide(ci[:, :2], col_xy)
        o = np.lexsort((ci[:, 2], c2[:, 1], c2[:, 0])); s2, sz = c2[o], ci[o, 2]
        brk = np.r_[True, (s2[1:] != s2[:-1]).any(1) | (np.diff(sz) > run_gap + 1)]
        rid = np.cumsum(brk) - 1
        lo_z = sz[brk]; hi_z = np.maximum.reduceat(sz, np.where(brk)[0])
        okr = (hi_z - lo_z + 1) * voxel >= min_run
        sel = np.zeros(len(ci), bool); sel[o] = okr[rid]
        ci, cc = ci[sel], cc[sel]
    info["cand_run"] = int(len(ci))
    if len(ci) == 0:
        info["added"] = 0
        return tsdf_idx.astype(np.int16), np.asarray(tsdf_rgb, np.uint8), info
    # 连通分块：kd 树找 ≤(bridge+1) 格切比雪夫邻居（欧氏 √3·(bridge+1) 近似），稀疏图连通分量；比稠密网格膨胀快得多
    pr = cKDTree(ci).query_pairs(np.sqrt(3) * (bridge + 1) + 1e-3, output_type="ndarray")
    n, L = connected_components(coo_matrix((np.ones(len(pr), np.int8), (pr[:, 0], pr[:, 1])), shape=(len(ci), len(ci))),
                                directed=False)
    cnt = np.bincount(L, minlength=n)
    mn = np.full((n, 3), np.iinfo(np.int64).max); mx = np.full((n, 3), np.iinfo(np.int64).min)
    for j in range(3):
        np.minimum.at(mn[:, j], L, ci[:, j]); np.maximum.at(mx[:, j], L, ci[:, j])
    ext = (mx - mn + 1) * voxel
    xy = ext[:, :2].max(1); zz = ext[:, 2]
    good = (cnt >= min_cells) & (zz >= min_zext) & (xy <= max_xyext) & (zz >= min_elong * xy)
    keep = good[L]
    if min_piece > 1 and keep.any():   # 收尾：补回格自身 26 连通小于 min_piece 的碎片（桥接才连上的）丢掉，不新增小碎块
        k = np.where(keep)[0]; q = ci[k]
        pr = cKDTree(q).query_pairs(np.sqrt(3) + 1e-3, output_type="ndarray")
        _, L2 = connected_components(coo_matrix((np.ones(len(pr), np.int8), (pr[:, 0], pr[:, 1])), shape=(len(q), len(q))),
                                     directed=False)
        keep[k[np.bincount(L2)[L2] < min_piece]] = False
    info.update(blocks=int(n), blocks_kept=int(good.sum()), added=int(keep.sum()))
    idx = np.concatenate([tsdf_idx.astype(np.int64), ci[keep]]).astype(np.int16)
    rgb = np.concatenate([np.asarray(tsdf_rgb, np.uint8), cc[keep]])
    return idx, rgb, info
