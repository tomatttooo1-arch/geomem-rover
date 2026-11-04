"""make_basemap.py --method tsdf 的实现（10-21，owner 批）：RTAB-Map 库里的关键帧 RGB/深度 + 全局优化位姿
-> TSDF 加权融合（Open3D VoxelBlockGrid）-> 过零面体素 -> 交回 make_basemap.build_cells 出 1 cm 格。

原型与参数来源：work/dev1/runs/map3d-up-20261021/A_tsdf（REPORT §2/§6）；外参修正：同目录 C_pose_depth（REPORT §2.3）。
本模块的取数/解码/外参拟合只用 numpy + 标准库（sqlite3）+ Pillow（open3d 的 pip 依赖，JPEG/PNG 解码）；
只有 fuse() 要 open3d，且在函数里才导入（--method pointcloud 与单测不需要它）。Python 3.8 语法。
"""

import io
import math
import os
import sqlite3
import time

import numpy as np

# A 轴推荐配置（REPORT §0/§9）：体素 = 格（1 cm）、截断 8 体素（8 cm）、深度截断 3 m、二阶差分剔飞点、
# 权重 >= 2、融合坐标平移半体素、过零边两端体素都出格（双层，无透光缝）。
TSDF_DEFAULTS = {"trunc_voxels": 8.0, "depth_max_m": 3.0, "weight_min": 2.0, "preprocess": "curv",
                 "extract": "both", "block_resolution": 16, "block_count": 16000, "min_valid_px": 2000}
MOUNT_FIX_LIMIT_DEG = 3.0       # 修正量超过它就不修（拟合多半错了），json 里报警
MOUNT_FIX_MIN_FRAMES = 10       # 能拟合地面的关键帧少于它就不修


# ---------------------------------------------------------------------------- 库读取

def open_db(path):
    """只读打开（URI；Windows 盘符路径也行）。"""
    p = os.path.abspath(path).replace("\\", "/")
    if not p.startswith("/"):
        p = "/" + p                       # file:/C:/...
    from urllib.parse import quote
    return sqlite3.connect("file:%s?mode=ro" % quote(p, safe="/:"), uri=True)


def m34(blob):
    m = np.eye(4)
    m[:3] = np.frombuffer(blob, np.float32, count=12).reshape(3, 4)
    return m


def q2m(x, y, z, qx, qy, qz, qw):
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                 [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                 [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]]
    m[:3, 3] = [x, y, z]
    return m


def read_pose_matrices(path):
    """格式 11 轨迹（stamp x y z qx qy qz qw id）-> {节点 id: 4×4 base_link in map}."""
    out = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            v = [float(a) for a in s.split()]
            if len(v) < 9:
                raise ValueError("轨迹缺节点 id 列（须 --poses_format 11）：%s" % path)
            out[int(v[8])] = q2m(*v[1:8])
    return out


def _cv_blob(blob, dtype):
    """RTAB-Map Admin 表的 cv::Mat 存法：zlib 压缩数据 + 末尾 12 字节 <iii (rows, cols, cv type)."""
    import zlib
    rows, cols, typ = (int(v) for v in np.frombuffer(blob[-12:], "<i4"))
    a = np.frombuffer(zlib.decompress(bytes(blob[:-12])), dtype)
    if a.size != rows * cols:
        raise ValueError("Admin blob 长度不符（%d != %d×%d，type %d）" % (a.size, rows, cols, typ))
    return a


def read_db_poses(db):
    """库里保存的全局优化位姿（rtabmap 关图时写进 Admin.opt_ids / opt_poses，与 rtabmap-export --opt 0 --poses 同源；
    10-16 库逐节点比平移差 ≤ 2e-6 m、角度 ≤ 0.015°，float32 精度）。
    -> ({id: 4×4 base_link in map}, [(stamp, x, y, yaw, id)] 按 stamp 排序)；库里没有就返回 (None, 原因)。"""
    c = open_db(db)
    try:
        try:
            row = c.execute("select opt_ids, opt_poses from Admin").fetchone()
        except sqlite3.Error as exc:
            return None, "Admin 表读不了：%s" % exc
        if row is None or row[0] is None or row[1] is None:
            return None, "Admin.opt_ids/opt_poses 为空（建图没正常关图？）"
        ids = _cv_blob(row[0], "<i4")
        mats = _cv_blob(row[1], "<f4").reshape(-1, 3, 4)
        if len(ids) != len(mats) or len(ids) < 2:
            return None, "opt_ids %d 与 opt_poses %d 不配" % (len(ids), len(mats))
        nodes = dict(c.execute("select id, stamp from Node").fetchall())
        missing = [int(i) for i in ids if int(i) not in nodes]
        if missing:
            return None, "opt_ids 里 %d 个节点不在 Node 表" % len(missing)
        poses, traj = {}, []
        for i, m in zip(ids, mats):
            t = np.eye(4)
            t[:3] = m.astype(np.float64)
            poses[int(i)] = t
            yaw = math.atan2(t[1, 0], t[0, 0])
            traj.append((float(nodes[int(i)]), float(t[0, 3]), float(t[1, 3]), yaw, int(i)))
        traj.sort(key=lambda p: (p[0], p[4]))
        return poses, traj
    finally:
        c.close()


def parse_calibration(blob):
    """RTAB-Map 0.23 CameraModel::serialize：11 个 int32 头 [ver×3, type, w, h, K 长, D 长, R 长, P 长, local 长]，
    之后 K/D/R/P（double）与 local_transform（12 float）。-> (K 3×3, local 4×4, (w, h))."""
    h = np.frombuffer(blob, np.int32, count=11)
    if h[3] != 0:
        raise ValueError("只支持单目/RGB-D 标定（type=%d）" % h[3])
    nk, nd, nr, npj, nl = (int(x) for x in h[6:11])
    if nk != 9 or nl != 12:
        raise ValueError("标定布局不认识（K %d, local %d）" % (nk, nl))
    off = 44
    k = np.frombuffer(blob, np.float64, count=9, offset=off).reshape(3, 3).copy()
    off += 8 * (nk + nd + nr + npj)
    if off + 48 != len(blob):
        raise ValueError("标定长度不符（%d != %d）" % (off + 48, len(blob)))
    return k, m34(blob[off:off + 48]), (int(h[4]), int(h[5]))


def rvl_decode(blob):
    """RTAB-Map 的 RVL 深度（"DEPTHRVL" + int32 宽 + int32 高 + RVL 码流；Wilson 2017）-> uint16 H×W（mm）。
    纯 numpy：码流按 uint32 小端字取 4 位一组（高位先），每组低 3 位是数据、最高位是续位；先把所有可变长整数一次
    解出来，再只在「零段长 / 非零段长」上走一遍 Python 循环（段数为几千），差分值整体 cumsum。"""
    if blob[:8] != b"DEPTHRVL":
        raise ValueError("不是 RVL 深度")
    w, h = (int(v) for v in np.frombuffer(blob, np.int32, count=2, offset=8))
    npx = w * h
    pay = bytes(blob[16:])
    pay += b"\0" * (-len(pay) % 4)
    words = np.frombuffer(pay, "<u4")
    nib = ((words[:, None] >> np.arange(28, -1, -4, dtype=np.uint32)[None, :]) & 0xF).astype(np.uint8).reshape(-1)
    end = (nib & 8) == 0
    ends = np.flatnonzero(end)
    # 每个可变长整数 = sum((nib & 7) << 3k)，k 为组内序号
    gid = np.zeros(len(nib), np.int64)
    gid[ends[:-1] + 1] = 1
    gid = np.cumsum(gid)
    start = np.concatenate([[0], ends[:-1] + 1])
    k = np.arange(len(nib)) - start[gid]
    vals = np.zeros(len(ends) + 1, np.int64)
    np.add.at(vals, gid, (nib & 7).astype(np.int64) << (3 * k))
    vals = vals[:len(ends)]
    out = np.zeros(npx, np.uint16)
    deltas, pos = [], []
    p = 0
    left = npx
    pix = 0
    nv = len(vals)
    while left > 0:
        if p + 1 >= nv:
            raise ValueError("RVL 码流提前结束")
        z = int(vals[p])
        nz = int(vals[p + 1])
        p += 2
        if z + nz > left:
            raise ValueError("RVL 段长越界")
        pix += z
        if nz:
            deltas.append(vals[p:p + nz])
            pos.append((pix, nz))
            p += nz
            pix += nz
        left -= z + nz
    if deltas:
        d = np.concatenate(deltas)
        d = (d >> 1) ^ -(d & 1)                       # zigzag
        v = (np.cumsum(d) & 0xFFFF).astype(np.uint16)  # prev 跨零段延续，uint16 回绕
        idx = np.concatenate([np.arange(a, a + n) for a, n in pos])
        out[idx] = v
    return out.reshape(h, w)


def decode_depth(blob):
    if blob[:8] == b"DEPTHRVL":
        return rvl_decode(blob)
    from PIL import Image          # 旧库：PNG 16 位
    return np.asarray(Image.open(io.BytesIO(blob)), dtype=np.uint16)


def decode_rgb(blob):
    from PIL import Image
    im = Image.open(io.BytesIO(blob))
    return np.ascontiguousarray(np.asarray(im.convert("RGB"), dtype=np.uint8))




def iter_frames(db, ids):
    """按 id 升序逐帧 -> (id, rgb H×W×3 uint8 或 None, depth uint16 mm, K, local)."""
    c = open_db(db)
    try:
        for i in sorted(ids):
            row = c.execute("select image, depth, calibration from Data where id=?", (int(i),)).fetchone()
            if row is None or row[1] is None or row[2] is None:
                yield i, None, None, None, None
                continue
            k, local, _ = parse_calibration(row[2])
            yield i, row[0], decode_depth(row[1]), k, local
    finally:
        c.close()


# ---------------------------------------------------------------------------- 深度预处理

def _dilate3(m):
    p = np.pad(m, 1)
    out = np.zeros_like(m)
    for dy in (0, 1, 2):
        for dx in (0, 1, 2):
            out |= p[dy:dy + m.shape[0], dx:dx + m.shape[1]]
    return out


def remove_flying(d):
    """A 轴「curv」：横/竖二阶差分 |z(-1) + z(+1) - 2z| > max(15 mm, 3 % z)（三点都有效）判深度断裂，膨胀 1 px 置 0。
    斜看地面一阶差大而二阶差约 0，不误删远处地面（REPORT §3）。与原型逐像素相同（原型用 cv2.dilate 3×3）。"""
    z = d.astype(np.float32)
    valid = z > 0
    bad = np.zeros_like(valid)
    for ax in (0, 1):
        zp = np.roll(z, 1, ax)
        zn = np.roll(z, -1, ax)
        ok = valid & (zp > 0) & (zn > 0)
        bad |= ok & (np.abs(zp + zn - 2 * z) > np.maximum(15.0, 0.03 * z))
    out = d.copy()
    out[_dilate3(bad)] = 0
    return out


# ---------------------------------------------------------------------------- 外参修正（C §2.3）

R_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0.]])   # 光学系 -> 零姿态 base：光轴 z -> x，x -> -y，y -> -z


def mount_rpy_deg(local):
    """local_transform（光学系 -> base_link）对应的安装 roll/pitch/yaw（度，ROS 习惯，rtabmap -cam_tf 同义）."""
    m = local[:3, :3] @ R_OPT.T
    return [math.degrees(math.atan2(m[2, 1], m[2, 2])), math.degrees(math.asin(max(-1.0, min(1.0, -m[2, 0])))),
            math.degrees(math.atan2(m[1, 0], m[0, 0]))]


def ground_slope_deg(depth, k, local, stride=2, band_m=0.05, range_m=2.5, min_pts=2000):
    """单帧：深度反投影到 base_link，取 |z| < band、光轴深度 < range 的点拟平面 z = a x + b y + c（3 次 1 cm 内点重拟），
    -> (沿 x 坡度°, 沿 y 坡度°, 内点数) 或 None。"""
    d = depth[::stride, ::stride].astype(np.float64) / 1000.0
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    m = (d > 0) & (d < range_m)
    z = d[m]
    p = np.stack([(u[m] - k[0, 2]) * z / k[0, 0], (v[m] - k[1, 2]) * z / k[1, 1], z], 1)
    pb = p @ local[:3, :3].T + local[:3, 3]
    g = np.abs(pb[:, 2]) < band_m
    if g.sum() < min_pts:
        return None
    a = np.c_[pb[g, 0], pb[g, 1], np.ones(int(g.sum()))]
    zz = pb[g, 2]
    c = np.linalg.lstsq(a, zz, rcond=None)[0]
    for _ in range(3):
        keep = np.abs(zz - a @ c) < 0.01
        if keep.sum() < min_pts // 2:
            return None
        c = np.linalg.lstsq(a[keep], zz[keep], rcond=None)[0]
    return math.degrees(math.atan(c[0])), math.degrees(math.atan(c[1])), int(keep.sum())


def rotation_to_level(slope_x_deg, slope_y_deg):
    """地面法向 (-a, -b, 1) 转到 +z 的最小旋转（base 系，左乘到 local 的旋转部分上）。"""
    n = np.array([-math.tan(math.radians(slope_x_deg)), -math.tan(math.radians(slope_y_deg)), 1.0])
    n /= np.linalg.norm(n)
    zax = np.array([0, 0, 1.0])
    v = np.cross(n, zax)
    s = np.linalg.norm(v)
    if s < 1e-12:
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - float(n @ zax)) / s ** 2)


def fit_mount_fix(samples, limit_deg=MOUNT_FIX_LIMIT_DEG, min_frames=MOUNT_FIX_MIN_FRAMES):
    """samples: [(depth, K, local)]（各关键帧）。逐帧拟地面坡度，取中位数（稳健：车身晃动/地上物体的帧不左右结果），
    求把地面摆平的旋转 R，新 local = R · local（平移不动，与 C 的 rp_camtf 同法），再用新 local 复拟一遍核坡度。
    -> (R 3×3 或 None, 报告 dict)。"""
    rows = [ground_slope_deg(d, k, l) for d, k, l in samples]
    rows = [r for r in rows if r is not None]
    rep = {"frames_total": len(samples), "frames_fitted": len(rows), "limit_deg": limit_deg,
           "statistic": "median of per-keyframe ground slopes (base_link)", "applied": False}
    if len(rows) < min_frames:
        rep["warning"] = "能拟合地面的关键帧只有 %d 个（< %d），不修" % (len(rows), min_frames)
        return None, rep
    sx = np.array([r[0] for r in rows])
    sy = np.array([r[1] for r in rows])
    mx, my = float(np.median(sx)), float(np.median(sy))
    rep["slope_before_deg"] = {"x_median": round(mx, 3), "y_median": round(my, 3), "x_mean": round(float(sx.mean()), 3),
                               "y_mean": round(float(sy.mean()), 3), "x_std": round(float(sx.std()), 3),
                               "y_std": round(float(sy.std()), 3)}
    r = rotation_to_level(mx, my)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2))))
    rep["rotation_deg"] = round(ang, 3)
    l0 = samples[0][2]
    l1 = l0.copy()
    l1[:3, :3] = r @ l0[:3, :3]
    rpy0, rpy1 = mount_rpy_deg(l0), mount_rpy_deg(l1)
    rep["mount_rpy_before_deg"] = [round(a, 3) for a in rpy0]
    rep["mount_rpy_after_deg"] = [round(a, 3) for a in rpy1]
    rep["delta_roll_deg"] = round(rpy1[0] - rpy0[0], 3)
    rep["delta_pitch_deg"] = round(rpy1[1] - rpy0[1], 3)
    if not np.isfinite(ang) or ang > limit_deg:
        rep["warning"] = "修正量 %.2f° 超过 %.1f°，疑似拟合错误，不修" % (ang, limit_deg)
        return None, rep
    # 修正后坡度：各帧拟合的地面法向直接乘 R 再算坡度（不重拟合；平移不动，只差选点带 ±5 cm 的边界效应）
    n = np.stack([-np.tan(np.radians(sx)), -np.tan(np.radians(sy)), np.ones(len(sx))], 1) @ r.T
    ax_ = np.degrees(np.arctan(-n[:, 0] / n[:, 2]))
    ay_ = np.degrees(np.arctan(-n[:, 1] / n[:, 2]))
    rep["slope_after_deg"] = {"x_mean": round(float(ax_.mean()), 3), "y_mean": round(float(ay_.mean()), 3),
                              "x_median": round(float(np.median(ax_)), 3), "y_median": round(float(np.median(ay_)), 3),
                              "frames": len(sx)}
    rep["applied"] = True
    return r, rep


# ---------------------------------------------------------------------------- 融合

def pick_device(o3c, want):
    cuda_ok = False
    try:
        cuda_ok = bool(o3c.cuda.is_available())
    except Exception:  # noqa: BLE001  (no CUDA build)
        cuda_ok = False
    if want == "cuda" and not cuda_ok:
        raise SystemExit("--device cuda 但 open3d 没有可用 CUDA（open3d 版本/驱动/--gpus all？）；可改 --device cpu")
    if want == "cpu" or not cuda_ok:
        return o3c.Device("CPU:0"), "CPU:0"
    return o3c.Device("CUDA:0"), "CUDA:0"


OPEN3D_STUBBED = []


def _stub_unused_open3d_deps():
    """open3d 的 pip 依赖（plotly、dash、scikit-learn、pandas、pyyaml…）只给可视化/ML 子包用，import open3d 时却会
    被 open3d.visualization.draw_plotly 与 open3d.ml 顺带导入。为了 `pip install --no-deps open3d` + numpy 就能跑
    （Windows 便携包少装几百 MB），缺哪个就在 import open3d 之前塞一个空壳模块；我们只用 core 与 t.geometry。
    装全了依赖时什么都不做。（宿主 Linux py3.11 + open3d 0.19 --no-deps 实测：只缺 plotly/dash 与 ml 子包的依赖。）"""
    import importlib.util
    import sys
    import types

    class _Stub(types.ModuleType):
        def __getattr__(self, k):
            if k.startswith("__"):
                raise AttributeError(k)
            return _Stub(self.__name__ + "." + k)

    if "open3d" in sys.modules:
        return
    if any(importlib.util.find_spec(n) is None for n in ("sklearn", "yaml", "pandas", "tqdm", "addict")):
        sys.modules["open3d.ml"] = _Stub("open3d.ml")
        OPEN3D_STUBBED.append("open3d.ml")
    for n, subs in (("plotly", ("graph_objects",)), ("dash", ())):
        if importlib.util.find_spec(n) is None:
            sys.modules[n] = _Stub(n)
            for sub in subs:
                sys.modules[n + "." + sub] = _Stub(n + "." + sub)
            OPEN3D_STUBBED.append(n)


def import_open3d():
    _stub_unused_open3d_deps()
    try:
        import open3d as o3d
        import open3d.core as o3c
    except ImportError as exc:
        raise SystemExit("--method tsdf 需要 open3d（pip install open3d==0.19.0，Python 3.8–3.12）：%s\n"
                         "没有 open3d 的机器用 --method pointcloud（旧的逐帧点云叠加）。" % exc)
    return o3d, o3c


def zero_crossings(vbg, voxel, wmin, both=True):
    """沿 x/y/z 找相邻两体素（weight >= wmin）tsdf 变号的边。both：两端体素都出（体素中心，双层、6 连通无缝）；
    否则出插值零点（单层）。按块做（numpy）：Open3D 块内存布局 attr[块, z, y, x]、体素整数坐标 = 块键 × R + (x, y, z)
    （实测核过）；块内邻居直接错位比较，跨块只在块面上按块键查邻块。块边界不漏（Open3D 自带 extract_point_cloud
    在 16 格块边界漏点，A REPORT §2）。GPU/CPU 两档同一段代码；原型 CPU 档用 o3c.HashMap 要 23 s。"""
    import open3d.core as o3c
    hm = vbg.hashmap()
    ab = hm.active_buf_indices().to(o3c.int64)
    keys = hm.key_tensor()[ab].cpu().numpy().astype(np.int64)
    nb = len(keys)
    if nb == 0:
        return np.zeros((0, 3)), np.zeros((0, 3))
    on_cpu = ab.device.get_type() == o3c.Device.DeviceType.CPU
    ab_np = ab.cpu().numpy()

    def view(name):
        # CPU：零拷贝 .numpy() 视图（整个预分配容量），再按需用 numpy 取（open3d 张量高级索引在 CPU 上很慢，实测 18 s）；
        # GPU：设备上按活跃块取完再拷回
        if on_cpu:
            return vbg.attribute(name).numpy(), ab_np
        return vbg.attribute(name)[ab].cpu().numpy(), np.arange(nb)

    tsv, tsi = view("tsdf")
    wv, wi = view("weight")
    r = tsv.shape[1]
    # 符号图 int8：+1/-1，权重不够或恰为 0 记 0；变号 <=> 两端乘积 < 0（与原型 a*b<0 且两端 w>=wmin 相同）
    sg = np.sign(tsv[tsi].reshape(nb, r, r, r)).astype(np.int8)
    sg[wv[wi].reshape(nb, r, r, r) < wmin] = 0
    need_ts = not both
    ts = tsv[tsi].reshape(nb, r, r, r) if need_ts else None
    colv, coli = view("color")
    colv = colv.reshape(colv.shape[0], r, r, r, 3)
    # 邻块查找：块键线性化后排序 + searchsorted
    lo = keys.min(0) - 1
    span = keys.max(0) - lo + 2
    g = keys - lo
    lin = (g[:, 0] * span[1] + g[:, 1]) * span[2] + g[:, 2]
    order = np.argsort(lin)
    lin_s = lin[order]
    outp, outc = [], []

    def emit(bi, zi, yi, xi, bj, zj, yj, xj):
        if both:
            for b_, z_, y_, x_ in ((bi, zi, yi, xi), (bj, zj, yj, xj)):
                outp.append((keys[b_] * r + np.stack([x_, y_, z_], 1)).astype(np.float64) * voxel)
                outc.append(colv[coli[b_], z_, y_, x_].astype(np.float64))
        else:
            ta, tb = ts[bi, zi, yi, xi], ts[bj, zj, yj, xj]
            pa = (keys[bi] * r + np.stack([xi, yi, zi], 1)).astype(np.float64) * voxel
            pb = (keys[bj] * r + np.stack([xj, yj, zj], 1)).astype(np.float64) * voxel
            t = (ta / (ta - tb)).astype(np.float64)[:, None]
            outp.append(pa + t * (pb - pa))
            ca = colv[coli[bi], zi, yi, xi].astype(np.float64)
            cb = colv[coli[bj], zj, yj, xj].astype(np.float64)
            outc.append(ca + t * (cb - ca))

    for ax, step in ((3, span[1] * span[2]), (2, span[2]), (1, 1)):   # 数组轴 3/2/1 = x/y/z；step = 块键 +1 的线性增量
        # 块内
        sa = [slice(None)] * 4
        sb = [slice(None)] * 4
        sa[ax] = slice(0, r - 1)
        sb[ax] = slice(1, r)
        m = (sg[tuple(sa)] * sg[tuple(sb)]) < 0
        bi, zi, yi, xi = np.nonzero(m)
        d = [0, 0, 0, 0]
        d[ax] = 1
        emit(bi, zi, yi, xi, bi, zi + d[1], yi + d[2], xi + d[3])
        # 跨块：本块最后一层 vs 邻块第一层
        tgt = lin + step
        pos = np.minimum(np.searchsorted(lin_s, tgt), nb - 1)
        has = lin_s[pos] == tgt
        src = np.flatnonzero(has)
        dst = order[pos[has]]
        if len(src) == 0:
            continue
        fa = [slice(None)] * 3
        fb = [slice(None)] * 3
        fa[ax - 1] = r - 1
        fb[ax - 1] = 0
        m = (sg[src][(slice(None),) + tuple(fa)] * sg[dst][(slice(None),) + tuple(fb)]) < 0
        k, u, v = np.nonzero(m)                  # 面内两维（按 z/y/x 去掉 ax 后的顺序）
        full = []
        it = iter((u, v))
        for a_ in (1, 2, 3):
            full.append(np.full(len(k), r - 1) if a_ == ax else next(it))
        zi, yi, xi = full
        zj, yj, xj = [np.zeros(len(k), np.int64) if a_ == ax else f for a_, f in zip((1, 2, 3), full)]
        emit(src[k], zi, yi, xi, dst[k], zj, yj, xj)
    if not outp:
        return np.zeros((0, 3)), np.zeros((0, 3))
    c = np.concatenate(outc)
    if len(c) and c.max() <= 1.0:
        c = c * 255.0
    return np.concatenate(outp), c


def fuse(db, poses, voxel, device="auto", mount_fix="auto", params=None, timing=None, log=None, support=None):
    """-> (xyz float64 N×3 map 系, rgb uint8 N×3, info dict)。poses：{id: 4×4 base_link in map}（全局优化）。
    support：可选 basemap_thin.KfSupport，逐融合帧喂剔点前原始深度与 T_map_cam（不减半体素；--thin-fill on）。"""
    p = dict(TSDF_DEFAULTS, **(params or {}))
    timing = {} if timing is None else timing
    log = log or (lambda s: None)
    o3d, o3c = import_open3d()
    dev, dev_name = pick_device(o3c, device)
    info = {"method": "tsdf", "open3d": o3d.__version__, "device": dev_name, "open3d_stubbed": list(OPEN3D_STUBBED),
            "omp_num_threads": os.environ.get("OMP_NUM_THREADS"), "voxel_m": voxel, "params": p}
    # 1) 读库 + 解码 + 剔飞点（关键帧 = 轨迹里的节点）
    t = time.perf_counter()
    frames = []
    missing = 0
    for i, rgb_blob, d, k, local in iter_frames(db, poses.keys()):
        if d is None:
            missing += 1
            continue
        frames.append([i, rgb_blob, d, k, local, d if support is not None else None])
    timing["tsdf_read_decode"] = time.perf_counter() - t
    t = time.perf_counter()
    for f in frames:
        if p["preprocess"] == "curv":
            f[2] = remove_flying(f[2])
    timing["tsdf_preprocess"] = time.perf_counter() - t
    info["frames_in_trajectory"] = len(poses)
    info["frames_missing_data"] = missing
    # 2) 外参修正（地面拟合用剔飞点后的深度）
    t = time.perf_counter()
    if mount_fix == "auto":
        r, rep = fit_mount_fix([(f[2], f[3], f[4]) for f in frames])
        if r is not None:
            for f in frames:
                ln = f[4].copy()
                ln[:3, :3] = r @ f[4][:3, :3]
                f[4] = ln
        if "warning" in rep:
            log("mount-fix 警告：%s" % rep["warning"])
    else:
        rep = {"applied": False, "mode": "off"}
    rep["mode"] = mount_fix
    info["mount_fix"] = rep
    timing["mount_fix"] = time.perf_counter() - t
    # 3) 融合
    t = time.perf_counter()
    half = 0.5 * voxel
    trunc = p["trunc_voxels"]
    dmax = p["depth_max_m"]
    vbg = o3d.t.geometry.VoxelBlockGrid(("tsdf", "weight", "color"), (o3c.float32, o3c.float32, o3c.float32),
                                        ((1), (1), (3)), voxel, int(p["block_resolution"]), int(p["block_count"]), dev)
    integrated = skipped = 0
    t_sup = 0.0
    for i, rgb_blob, d, k, local, d_raw in frames:
        if np.count_nonzero((d > 0) & (d < dmax * 1000)) < p["min_valid_px"]:
            skipped += 1
            continue
        rgb = decode_rgb(rgb_blob)
        tw = poses[i] @ local
        tw[:3, 3] -= half          # 融合系 = map 系平移 -半体素：体素采样点 i·voxel 落在 map 系格心 (i+½)·voxel
        ext = o3c.Tensor(np.linalg.inv(tw))
        kt = o3c.Tensor(k)
        dt = o3d.t.geometry.Image(o3c.Tensor(np.ascontiguousarray(d))).to(dev)
        ct = o3d.t.geometry.Image(o3c.Tensor(rgb)).to(dev)
        bc = vbg.compute_unique_block_coordinates(dt, kt, ext, 1000.0, dmax, trunc)
        vbg.integrate(bc, dt, ct, kt, kt, ext, 1000.0, dmax, trunc)
        integrated += 1
        if support is not None:
            ts0 = time.perf_counter()
            support.add(d_raw, rgb, k, poses[i] @ local)
            t_sup += time.perf_counter() - ts0
    if dev_name.startswith("CUDA"):
        o3c.cuda.synchronize()
    timing["tsdf_integrate"] = time.perf_counter() - t - t_sup
    if support is not None:
        timing["thin_support"] = t_sup
    info["frames_integrated"] = integrated
    info["frames_skipped_empty"] = skipped
    info["active_blocks"] = int(vbg.hashmap().size())
    # 4) 过零面
    t = time.perf_counter()
    xyz, col = zero_crossings(vbg, voxel, p["weight_min"], p["extract"] == "both")
    xyz = xyz + half
    rgb8 = (np.clip(col, 0, 255) + 0.5).astype(np.uint8)
    info["surface_points"] = int(len(xyz))
    timing["tsdf_extract"] = time.perf_counter() - t
    return xyz, rgb8, info
