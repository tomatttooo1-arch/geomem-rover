#!/usr/bin/env python3
"""GeoMem 展台底图包：RTAB-Map 地图库 -> 1 cm 真彩体素格（F 风格效果稿的数据来源，10-17；10-17 起默认 1 cm）。

    python ui/geomem/tools/basemap/make_basemap.py --db <地图库.db> --out-dir <输出目录>
        [--ply 已导出的 gm_cloud.ply --poses gm_poses.txt]   # 跳过导出（单测、已在别处导出）
        [--export native [--rtabmap-export 可执行文件]]       # 10-18：不用 docker，直接调本机 rtabmap-export
                                                             # （Nano 车栈镜像里、装了 RTAB-Map 的笔记本）

10-21 起默认 ``--method tsdf``（关键帧深度 TSDF 加权融合，basemap_tsdf.py；要 open3d 0.19、Pillow，Python 3.8–3.12）
与 ``--mount-fix auto``（逐关键帧拟合地面修相机 roll/pitch）；位姿默认取库 Admin 表（不要 rtabmap/docker），
旧路径 ``--method pointcloud``。参数与依赖见 README.md「10-21」节。

pointcloud 路径需要 numpy、scipy（≥ 1.6，cKDTree.query 的 workers 参数；Nano 车栈镜像 apt 的 1.21.5 / 1.8.0 可用）；
Python 3.8+。桥只读成品包，仍是纯标准库。步骤与理由见同目录 README.md，
包格式见 ui/geomem/docs/PROTOCOL_V2_DESIGN.md §2.10。输出 ``<out-dir>/basemap-<库 sha256 前 12 位>/``：

  cells.bin      N×3 int16 LE 格索引（格心 = (索引 + 0.5) × cell_m，map 系 x 前 y 左 z 上）
                 紧跟 N×3 uint8 RGB（格内各点颜色的平均，四舍五入）
  basemap.json   来源库 sha256/字节数、参数、各步点数、包围盒、建图轨迹、cells.bin 的 sha256

确定性：输入点先按 (x, y, z, r, g, b) 排成规范顺序，之后每一步都不依赖导出点的先后；同一库两次输出
cells.bin 逐字节相同，basemap.json 去掉 ``generated_at`` 与 ``timing_s`` 后逐字节相同（单测与验收核）。
"""

import argparse
import datetime
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # 便携/嵌入式 Python 不自动加脚本目录

import numpy as np
from scipy.spatial import cKDTree

FORMAT = "geomem-basemap"
VERSION = 1
IMAGE = "fpga-dev1-rtabmap:humble-0.23.7-20260910"
# 与效果稿 F（runs/map3d-look-20261017/export/hq.log）同一组导出参数；--opt 0 与
# tools/car_run/rtabmap_car.sh 给 Nav2 导 --map 的默认一致（全局优化，map 系）。
EXPORT_ARGS = ["--opt", "0", "--cloud", "--poses", "--decimation", "1", "--max_range", "3.0",
               "--voxel", "0.004", "--edge_bleeding_error", "0.02"]
# 10-21 --method tsdf（默认）只要轨迹：rtabmap-export 只导位姿（格式 11 带节点 id，融合按 id 取库里的图像/深度）
POSES_EXPORT_ARGS = ["--opt", "0", "--poses", "--poses_format", "11"]
TSDF_MIN_POINTS = 1          # 融合面每格已是加权平均，1 个过零体素就成格（A REPORT §6）
SURFACE_POSTPROCESS = None   # 挂接点：f(xyz, rgb, info) -> (xyz, rgb)，作用在 TSDF 表面点上、分格之前（细结构保留等）
DEFAULTS = {"cell_m": 0.01, "min_points": 2, "crop_m": 2.0, "z_min_m": -0.15, "sor_k": 8, "sor_std": 1.5}
INT16_MAX = 32767
QUERY_CHUNK = 1 << 20     # k-d 树查询按 100 万点一块：每点结果与不分块相同，峰值内存小（Nano 8 GB 与车栈共用，10-18）


def sha256_file(path, chunk=4 << 20):
    h = hashlib.sha256()
    n = 0
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
            n += len(b)
    return h.hexdigest(), n


# ---------------------------------------------------------------------------- inputs

PLY_TYPES = {"float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8", "uchar": "u1", "uint8": "u1",
             "char": "i1", "int8": "i1", "ushort": "<u2", "uint16": "<u2", "short": "<i2", "int16": "<i2",
             "uint": "<u4", "uint32": "<u4", "int": "<i4", "int32": "<i4"}


def read_ply(path):
    """(xyz float64 N×3, rgb uint8 N×3) from a binary little-endian PLY; only the vertex element is read
    (rtabmap-export writes ``element face 0`` and ``element camera 1`` after it)."""
    with open(path, "rb") as f:
        if f.readline().strip() != b"ply":
            raise ValueError("不是 PLY 文件：%s" % path)
        fmt, elements, cur = None, [], None
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PLY 头没有 end_header")
            tok = line.decode("ascii", "replace").split()
            if not tok:
                continue
            if tok[0] == "format":
                fmt = tok[1]
            elif tok[0] == "element":
                cur = {"name": tok[1], "count": int(tok[2]), "props": []}
                elements.append(cur)
            elif tok[0] == "property":
                if tok[1] == "list":                    # only legal outside the vertex element
                    cur["props"].append((tok[-1], None))
                else:
                    cur["props"].append((tok[2], PLY_TYPES.get(tok[1])))
            elif tok[0] == "end_header":
                break
        if fmt != "binary_little_endian":
            raise ValueError("只支持 binary_little_endian PLY（收到 %s）" % fmt)
        if not elements or elements[0]["name"] != "vertex":
            raise ValueError("PLY 第一个元素须为 vertex")
        if any(t is None for _, t in elements[0]["props"]):
            raise ValueError("vertex 元素含 list 或未知类型属性")
        dt = np.dtype(elements[0]["props"])
        n = elements[0]["count"]
        raw = f.read(n * dt.itemsize)
    if len(raw) != n * dt.itemsize:
        raise ValueError("PLY 顶点数据不完整")
    v = np.frombuffer(raw, dtype=dt, count=n)
    for k in ("x", "y", "z", "red", "green", "blue"):
        if k not in dt.names:
            raise ValueError("PLY 缺字段 %s" % k)
    xyz = np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float64)
    rgb = np.stack([v["red"], v["green"], v["blue"]], 1).astype(np.uint8)
    return xyz, rgb


def read_poses(path):
    """rtabmap-export --poses 格式 11（``stamp x y z qx qy qz qw id``）-> 按 stamp 排好的 [(stamp, x, y, yaw, id)]."""
    out = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            t, x, y, z, qx, qy, qz, qw = (float(a) for a in s.split()[:8])
            nid = int(float(s.split()[8])) if len(s.split()) > 8 else None
            yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            out.append((t, x, y, yaw, nid))
    out.sort(key=lambda p: (p[0], p[4] if p[4] is not None else -1))
    if not out:
        raise ValueError("轨迹文件里没有位姿：%s" % path)
    return out


def export_native(db, work, exe="rtabmap-export", export_args=None):
    """本机 rtabmap-export（10-18：Nano 车栈镜像里 /opt/ros/humble/bin 有，与服务器镜像同为 0.23.7；装了 RTAB-Map
    的 Windows 笔记本给 rtabmap-export.exe 的路径）。参数与 docker 档相同，另显式 --poses_format 11（read_poses 读的
    格式；rtabmap_car.sh 导轨迹也这样给）。返回 (ply, poses, 命令行)."""
    extra = ["--poses_format", "11"] if export_args is None else []
    cmd = [exe] + (EXPORT_ARGS if export_args is None else export_args) + extra + ["--output_dir", os.path.abspath(work), "--output", "gm",
                                 os.path.abspath(db)]
    with open(os.path.join(work, "export.log"), "wb") as log:
        try:
            rc = subprocess.call(cmd, stdout=log, stderr=subprocess.STDOUT)
        except OSError as exc:
            raise SystemExit("起不了 %s：%s（Nano 镜像里先 source /opt/ros/humble/setup.bash）" % (exe, exc))
    if rc != 0:
        raise SystemExit("rtabmap-export 失败（rc=%d），日志 %s" % (rc, os.path.join(work, "export.log")))
    return os.path.join(work, "gm_cloud.ply"), os.path.join(work, "gm_poses.txt"), " ".join(
        ["rtabmap-export"] + cmd[1:-5] + ["--output_dir", "<work>", "--output", "gm", os.path.basename(db)])


def export_db(db, work, image=IMAGE, docker="docker", export_args=None):
    """docker 里 rtabmap-export（库目录只读挂载）；返回 (ply, poses, 命令行)."""
    db = os.path.abspath(db)
    uid = "%d:%d" % (os.getuid(), os.getgid()) if hasattr(os, "getuid") else None
    inner = "source /opt/ros/humble/setup.bash && rtabmap-export %s --output_dir /out --output gm /in/%s" % (
        " ".join(EXPORT_ARGS if export_args is None else export_args), os.path.basename(db))
    cmd = [docker, "run", "--rm"] + (["--user", uid] if uid else []) + [
        "-v", "%s:/in:ro" % os.path.dirname(db), "-v", "%s:/out" % os.path.abspath(work), image, "-c", inner]
    with open(os.path.join(work, "export.log"), "wb") as log:
        rc = subprocess.call(cmd, stdout=log, stderr=subprocess.STDOUT)
    if rc != 0:
        raise SystemExit("rtabmap-export 失败（rc=%d），日志 %s" % (rc, os.path.join(work, "export.log")))
    return os.path.join(work, "gm_cloud.ply"), os.path.join(work, "gm_poses.txt"), inner


# ---------------------------------------------------------------------------- processing

def canonical_order(xyz, rgb):
    """按 (x, y, z, r, g, b) 字典序重排：之后的均值/方差/分格都与导出点的先后无关。"""
    keys = (rgb[:, 2], rgb[:, 1], rgb[:, 0], xyz[:, 2], xyz[:, 1], xyz[:, 0])   # lexsort: last key primary
    order = np.lexsort(keys)
    return xyz[order], rgb[order]


def build_cells(xyz, rgb, traj_xy, cell_m=DEFAULTS["cell_m"], min_points=DEFAULTS["min_points"],
                crop_m=DEFAULTS["crop_m"], z_min_m=DEFAULTS["z_min_m"], sor_k=DEFAULTS["sor_k"],
                sor_std=DEFAULTS["sor_std"], workers=-1, timing=None):
    """(idx int16 M×3, rgb uint8 M×3, counts dict). 步骤见 README；效果稿 hq.py 的同一套，只把裁剪换成 k-d 树、
    把分格放在 map 系（原型在 three 系分格，格集合与格心相同，只差恰在格边界上的点）。"""
    timing = {} if timing is None else timing
    counts = {"read": int(len(xyz))}
    t = time.perf_counter()
    xyz, rgb = canonical_order(np.asarray(xyz, np.float64), np.asarray(rgb, np.uint8))
    ok = np.isfinite(xyz).all(1) & (xyz[:, 2] > z_min_m)
    xyz, rgb = xyz[ok], rgb[ok]
    counts["finite_above_z_min"] = int(len(xyz))
    timing["filter"] = time.perf_counter() - t
    # 1) 只留建图轨迹 crop_m 以内的点（xy 距离）；原型逐点暴力广播，这里 k-d 树
    t = time.perf_counter()
    if len(xyz):
        tree = cKDTree(np.asarray(traj_xy, np.float64))
        d = np.concatenate([tree.query(xyz[i:i + QUERY_CHUNK, :2], k=1, workers=workers)[0]
                            for i in range(0, len(xyz), QUERY_CHUNK)])
        keep = d < crop_m
        xyz, rgb = xyz[keep], rgb[keep]
    counts["cropped"] = int(len(xyz))
    timing["crop"] = time.perf_counter() - t
    # 2) 统计离群：到 sor_k 个近邻的平均距离 < 全体均值 + sor_std × 标准差
    t = time.perf_counter()
    if sor_k and len(xyz) > sor_k:          # sor_k = 0: 不做（--method tsdf：融合已去噪，SOR 会削掉细结构）
        tree = cKDTree(xyz)
        md = np.concatenate([tree.query(xyz[i:i + QUERY_CHUNK], k=sor_k + 1, workers=workers)[0][:, 1:].mean(1)
                             for i in range(0, len(xyz), QUERY_CHUNK)])
        keep = md < md.mean() + sor_std * md.std()
        xyz, rgb = xyz[keep], rgb[keep]
    counts["after_outliers"] = int(len(xyz))   # points kept after the statistical outlier removal (10-17: was misnamed outliers_removed)
    timing["outliers"] = time.perf_counter() - t
    # 3) cell_m 体素：格内 >= min_points 点才留，颜色取平均
    t = time.perf_counter()
    if len(xyz) == 0:
        counts.update({"cells_all": 0, "cells": 0})
        timing["voxel"] = time.perf_counter() - t
        return np.zeros((0, 3), np.int16), np.zeros((0, 3), np.uint8), counts
    k = np.floor(xyz / cell_m).astype(np.int64)
    lo = k.min(0)
    span = k.max(0) - lo + 1
    lin = ((k[:, 0] - lo[0]) * span[1] + (k[:, 1] - lo[1])) * span[2] + (k[:, 2] - lo[2])
    u, inv, cnt = np.unique(lin, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    acc = np.stack([np.bincount(inv, weights=rgb[:, c].astype(np.float64), minlength=len(u)) for c in range(3)], 1)
    m = cnt >= min_points
    counts["cells_all"] = int(len(u))
    u, acc, cnt = u[m], acc[m], cnt[m]
    col = np.floor(acc / cnt[:, None] + 0.5).clip(0, 255).astype(np.uint8)
    iz = u % span[2]
    iy = (u // span[2]) % span[1]
    ix = u // (span[2] * span[1])
    idx = np.stack([ix + lo[0], iy + lo[1], iz + lo[2]], 1)
    if len(idx) and (idx.min() < -INT16_MAX or idx.max() > INT16_MAX):
        raise ValueError("格索引超出 int16（地图跨度超过 ±%.0f m）" % (INT16_MAX * cell_m))
    counts["cells"] = int(len(idx))
    timing["voxel"] = time.perf_counter() - t
    return idx.astype(np.int16), col, counts


def cells_bytes(idx, rgb):
    return np.ascontiguousarray(idx, dtype="<i2").tobytes() + np.ascontiguousarray(rgb, dtype=np.uint8).tobytes()


def parse_cells(buf):
    """cells.bin -> (idx int16 N×3, rgb uint8 N×3)；桥/页面同一解析（单测往返用）。"""
    if len(buf) % 9:
        raise ValueError("cells.bin 长度不是 9 的倍数")
    n = len(buf) // 9
    idx = np.frombuffer(buf, dtype="<i2", count=3 * n).reshape(n, 3)
    rgb = np.frombuffer(buf, dtype=np.uint8, count=3 * n, offset=6 * n).reshape(n, 3)
    return idx, rgb


def git_commit():
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        out = subprocess.check_output(["git", "-C", here, "rev-parse", "HEAD"], stderr=subprocess.DEVNULL)
        st = subprocess.check_output(["git", "-C", here, "status", "--porcelain", "--", "."],
                                     stderr=subprocess.DEVNULL)
        return out.decode().strip() + ("+dirty" if st.strip() else "")
    except (OSError, subprocess.CalledProcessError):
        return None


def r6(v):
    return float(round(float(v), 6))


def make_manifest(db_name, db_sha, db_bytes, params, counts, idx, traj, cells_sha, cells_len, export_cmd,
                  timing, generated_at, export_mode="docker", method="pointcloud", tsdf_info=None):
    cell = params["cell_m"]
    if len(idx):
        c0 = (idx.min(0).astype(np.float64) + 0.5) * cell
        c1 = (idx.max(0).astype(np.float64) + 0.5) * cell
        bbox = {"min": [r6(v) for v in c0], "max": [r6(v) for v in c1]}
    else:
        bbox = None
    man = {
        "format": FORMAT, "version": VERSION, "method": method,
        "source_db": {"name": db_name, "sha256": db_sha, "bytes": db_bytes},
        "frame": "map", "axes": "RTAB-Map map 系 x 前 y 左 z 上，米",
        "cell_m": cell, "min_points": params["min_points"],
        "params": {k: params[k] for k in sorted(params)},
        "counts": counts,
        "bbox_cell_centers_m": bbox,
        "trajectory": [[r6(x), r6(y), r6(yaw)] for (_t, x, y, yaw, _i) in traj],
        "export": {"image": IMAGE if export_mode == "docker" else None, "mode": export_mode, "command": export_cmd},
        "files": {"cells": {"name": "cells.bin", "bytes": cells_len, "sha256": cells_sha, "count": int(len(idx)),
                            "layout": "N*3 int16 LE cell index (centre = (index + 0.5) * cell_m, map frame), "
                                      "then N*3 uint8 RGB"}},
        "generator": {"script": "ui/geomem/tools/basemap/make_basemap.py", "git": git_commit()},
        "generated_at": generated_at,
        "timing_s": {k: round(v, 3) for k, v in sorted(timing.items())},
    }
    if tsdf_info is not None:     # 10-21：只加字段，已有字段不改名不删（桥/控制台在读）
        info = dict(tsdf_info)
        man["mount_fix"] = info.pop("mount_fix", None)
        man["tsdf"] = info
    return man


def manifest_bytes(obj):
    return (json.dumps(obj, ensure_ascii=False, indent=1, sort_keys=True) + "\n").encode("utf-8")


# ---------------------------------------------------------------------------- main

def build_arg_parser():
    ap = argparse.ArgumentParser(description="GeoMem 展台底图包（RTAB-Map 库 -> 1 cm 真彩体素格）")
    ap.add_argument("--db", required=True, help="RTAB-Map 地图库 .db（算 sha256；不给 --ply 时也用它导出）")
    ap.add_argument("--out-dir", required=True, help="包目录的上级；包名 basemap-<sha 前 12 位>")
    ap.add_argument("--ply", default=None, help="已导出的点云（跳过 docker 导出；须与 --poses 同给）")
    ap.add_argument("--poses", default=None, help="已导出的轨迹（格式 11）")
    ap.add_argument("--work-dir", default=None, help="导出临时目录（默认在 --out-dir 下建、用完删）")
    ap.add_argument("--keep-export", action="store_true", help="保留导出的 PLY/轨迹（放进 work-dir）")
    ap.add_argument("--image", default=IMAGE, help="rtabmap 镜像（--export docker）")
    ap.add_argument("--export", choices=("docker", "native"), default="docker",
                    help="导出方式：docker（服务器，默认）/ native（本机 rtabmap-export：Nano 车栈镜像、笔记本）")
    ap.add_argument("--rtabmap-export", default="rtabmap-export", help="--export native 时的可执行文件")
    ap.add_argument("--method", choices=("tsdf", "pointcloud"), default=None,
                    help="tsdf（默认，10-21）：关键帧深度 TSDF 加权融合，要 open3d；pointcloud：旧的 rtabmap-export 逐帧点云"
                         "叠加（给了 --ply 时默认它）")
    ap.add_argument("--mount-fix", choices=("auto", "off"), default="auto",
                    help="tsdf：逐关键帧拟合地面，修相机安装 roll/pitch（默认 auto；超过 3° 不修并在 json 报警）")
    ap.add_argument("--poses-from", choices=("auto", "db", "export"), default="auto",
                    help="tsdf 的位姿：db = 库 Admin 表里存的全局优化位姿（不要 rtabmap/docker/ROS）；export = "
                         "rtabmap-export 导出（--export docker|native）；auto（默认）= 库里有就用 db，否则 export")
    ap.add_argument("--thin-fill", choices=("on", "off"), default="off",
                    help="tsdf：细结构补回（G_thin：TSDF 掏空的约 2 cm 椅腿/桌腿按关键帧原始深度补回竖直细长块；默认 off）")
    ap.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto",
                    help="tsdf 融合设备：auto = open3d 有 CUDA 用 GPU，否则 CPU")
    ap.add_argument("--threads", type=int, default=8, help="tsdf：OMP_NUM_THREADS（固定线程数；默认 8）")
    ap.add_argument("--cell", type=float, default=DEFAULTS["cell_m"])
    ap.add_argument("--min-points", type=int, default=None,
                    help="格内最少点数（默认 pointcloud %d、tsdf %d）" % (DEFAULTS["min_points"], TSDF_MIN_POINTS))
    ap.add_argument("--crop", type=float, default=DEFAULTS["crop_m"])
    ap.add_argument("--z-min", type=float, default=DEFAULTS["z_min_m"])
    ap.add_argument("--sor-k", type=int, default=DEFAULTS["sor_k"])
    ap.add_argument("--sor-std", type=float, default=DEFAULTS["sor_std"])
    ap.add_argument("--workers", type=int, default=-1, help="k-d 树查询线程（-1 全部）")
    ap.add_argument("--force", action="store_true", help="包目录已存在时覆盖")
    return ap


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    if args.method is None:
        args.method = "pointcloud" if args.ply is not None else "tsdf"
    tsdf = args.method == "tsdf"
    if tsdf:
        if args.ply is not None:
            raise SystemExit("--method tsdf 不用 --ply（直接读库里的图像/深度）；只给 --poses 可跳过导出")
        os.environ["OMP_NUM_THREADS"] = str(args.threads)      # 须在 import open3d 之前（A REPORT §2.3）
    elif (args.ply is None) != (args.poses is None):
        raise SystemExit("--ply 与 --poses 须同时给或同时不给")
    if args.min_points is None:
        args.min_points = TSDF_MIN_POINTS if tsdf else DEFAULTS["min_points"]
    if tsdf:
        args.sor_k = 0
    t_all = time.perf_counter()
    timing = {}
    t = time.perf_counter()
    db_sha, db_bytes = sha256_file(args.db)
    timing["sha256"] = time.perf_counter() - t
    pkg = os.path.join(args.out_dir, "basemap-%s" % db_sha[:12])
    if os.path.exists(pkg) and not args.force:
        raise SystemExit("包目录已存在：%s（--force 覆盖）" % pkg)
    os.makedirs(args.out_dir, exist_ok=True)
    work, own_work = args.work_dir, False
    try:
        tsdf_info = None
        pose_mats = traj = None
        poses_source = None
        if tsdf:
            import basemap_tsdf
            t = time.perf_counter()
            basemap_tsdf.import_open3d()          # 先核 open3d（没有就立刻报错，别等导出完）
            timing["import_open3d"] = time.perf_counter() - t
            if args.poses is None and args.poses_from in ("auto", "db"):
                t = time.perf_counter()
                pose_mats, why = basemap_tsdf.read_db_poses(args.db)
                timing["poses_db"] = time.perf_counter() - t
                if pose_mats is not None:
                    traj, poses_source, export_cmd = why, "db", None
                elif args.poses_from == "db":
                    raise SystemExit("--poses-from db：%s" % why)
                else:
                    sys.stderr.write("库里没有可用的优化位姿（%s），回落 rtabmap-export 导位姿\n" % why)
        if tsdf and pose_mats is None and args.poses is None:
            if work is None:
                work, own_work = tempfile.mkdtemp(prefix="basemap-export-", dir=args.out_dir), True
            os.makedirs(work, exist_ok=True)
            t = time.perf_counter()
            if args.export == "native":
                _, poses, export_cmd = export_native(args.db, work, args.rtabmap_export, POSES_EXPORT_ARGS)
            else:
                _, poses, export_cmd = export_db(args.db, work, args.image, export_args=POSES_EXPORT_ARGS)
            timing["export"] = time.perf_counter() - t
            poses_source = "export"
        elif tsdf and pose_mats is None:
            poses, export_cmd, poses_source = args.poses, None, "file"
        elif not tsdf and args.ply is None:
            if work is None:
                work, own_work = tempfile.mkdtemp(prefix="basemap-export-", dir=args.out_dir), True
            os.makedirs(work, exist_ok=True)
            t = time.perf_counter()
            if args.export == "native":
                ply, poses, export_cmd = export_native(args.db, work, args.rtabmap_export)
            else:
                ply, poses, export_cmd = export_db(args.db, work, args.image)
            timing["export"] = time.perf_counter() - t
        else:
            ply, poses, export_cmd = args.ply, args.poses, None
        if tsdf:
            export_mode = {"db": "db", "file": "poses"}.get(poses_source, args.export)
            if pose_mats is None:
                traj = read_poses(poses)
                pose_mats = basemap_tsdf.read_pose_matrices(poses)
            support = None
            if args.thin_fill == "on":
                import basemap_thin
                support = basemap_thin.KfSupport(voxel=args.cell)
            xyz, rgb, tsdf_info = basemap_tsdf.fuse(
                args.db, pose_mats, args.cell, device=args.device, support=support,
                mount_fix=args.mount_fix, timing=timing, log=lambda m: sys.stderr.write(m + "\n"))
            tsdf_info["poses_source"] = poses_source
            if SURFACE_POSTPROCESS is not None:
                t = time.perf_counter()
                xyz, rgb = SURFACE_POSTPROCESS(xyz, rgb, tsdf_info)
                timing["surface_postprocess"] = time.perf_counter() - t
                tsdf_info["surface_postprocess"] = getattr(SURFACE_POSTPROCESS, "__name__", "custom")
        else:
            export_mode = "ply" if args.ply is not None else args.export
            t = time.perf_counter()
            xyz, rgb = read_ply(ply)
            traj = read_poses(poses)
            timing["read"] = time.perf_counter() - t
        params = {"cell_m": args.cell, "min_points": args.min_points, "crop_m": args.crop, "z_min_m": args.z_min,
                  "sor_k": args.sor_k, "sor_std": args.sor_std}
        idx, col, counts = build_cells(xyz, rgb, [(p[1], p[2]) for p in traj], workers=args.workers, timing=timing,
                                       **{k: params[k] for k in ("cell_m", "min_points", "crop_m", "z_min_m",
                                                                 "sor_k", "sor_std")})
        if tsdf and args.thin_fill == "on":
            t = time.perf_counter()
            idx, col, thin_info = basemap_thin.thin_fill(idx, col, *support.result(),
                                                          traj_xy=[(p[1], p[2]) for p in traj], crop_m=args.crop,
                                                          voxel=args.cell)
            o = np.lexsort((idx[:, 2], idx[:, 1], idx[:, 0]))       # cells.bin 按格索引排序（补回格原在末尾）
            idx, col = idx[o], col[o]
            counts["cells_tsdf"] = counts["cells"]
            counts["thin_fill_added"] = int(thin_info.get("added", 0))
            counts["cells"] = int(len(idx))
            tsdf_info["thin_fill"] = thin_info
            timing["thin_fill"] = time.perf_counter() - t
        elif tsdf:
            tsdf_info["thin_fill"] = None
        counts["trajectory_poses"] = len(traj)
        data = cells_bytes(idx, col)
        cells_sha = hashlib.sha256(data).hexdigest()
        tmp_pkg = pkg + ".tmp-%d" % os.getpid()
        os.makedirs(tmp_pkg)
        with open(os.path.join(tmp_pkg, "cells.bin"), "wb") as f:
            f.write(data)
        timing["total"] = time.perf_counter() - t_all
        man = make_manifest(os.path.basename(args.db), db_sha, db_bytes, params, counts, idx, traj, cells_sha,
                            len(data), export_cmd, timing,
                            datetime.datetime.now(datetime.timezone.utc).astimezone().isoformat(timespec="seconds"),
                            export_mode=export_mode, method=args.method, tsdf_info=tsdf_info)
        with open(os.path.join(tmp_pkg, "basemap.json"), "wb") as f:
            f.write(manifest_bytes(man))
        if os.path.exists(pkg):
            shutil.rmtree(pkg)
        os.rename(tmp_pkg, pkg)
    finally:
        if own_work and not args.keep_export:
            shutil.rmtree(work, ignore_errors=True)
    sys.stdout.write("basemap %s\n  cells %d (%d bytes, sha256 %s)\n  counts %s\n  timing_s %s\n" % (
        pkg, counts["cells"], len(data), cells_sha, json.dumps(counts, sort_keys=True),
        json.dumps(man["timing_s"], sort_keys=True)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
