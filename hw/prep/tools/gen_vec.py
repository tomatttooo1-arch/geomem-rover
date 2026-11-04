"""PREP1：生成 csim/cosim 输入帧、向量表与 golden（固定种子，可重生）。2026-11-03

golden 的来源（两路互核，任一不一致就退出）：
  * RGB/BGR 输入：host/dpu_prep.py 的 cv2 后端（dev3 评测同一函数），并与 PREP0 定点 C 模型 prep_model 比，必须 0 字节差。
  * YUYV/UYVY 输入：hw/prep/ref/yuyv_ref（libjpeg 查表式色彩 + h2v1 三角上采样）先转 RGB，再走上面两路。
用法：CUDA_VISIBLE_DEVICES= python3 gen_vec.py <dpu_prep 所在 host 目录> <prep_model 可执行> <yuyv_ref 可执行> <run 目录> <hls1 目录>
产物：<run>/vec/in/*.raw、<run>/golden/{csim,cosim}/tb_out/{<id>.bin, meta.txt}、<hls1>/vec_{csim,cosim}.txt
"""
import glob
import os
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image

HOST, MODEL, YREF, RUN, HLS = sys.argv[1:6]
sys.path.insert(0, HOST)
import dpu_prep as dp  # noqa: E402

DATA = "${FPGA_RUN}/DPUPIPE1/data/mix16"
IN = os.path.join(RUN, "vec", "in")
os.makedirs(IN, exist_ok=True)
VERSION8 = 0x02   # STATUS[15:8]


def ceil16(x):
    return (x + 15) // 16 * 16


def save(name, arr):
    p = os.path.join(IN, name + ".raw")
    np.ascontiguousarray(arr).tofile(p)
    return p


def rgb_to_yuyv(rgb, uyvy=False):
    """JFIF 全范围 RGB->YCbCr（浮点），4:2:2 两像素平均色度，打包成 YUYV/UYVY 字节。任何字节都是合法输入，这里只为造出像样的画面。"""
    f = rgb.astype(np.float64)
    y = 0.299 * f[..., 0] + 0.587 * f[..., 1] + 0.114 * f[..., 2]
    cb = -0.168736 * f[..., 0] - 0.331264 * f[..., 1] + 0.5 * f[..., 2] + 128
    cr = 0.5 * f[..., 0] - 0.418688 * f[..., 1] - 0.081312 * f[..., 2] + 128
    cb = (cb[:, 0::2] + cb[:, 1::2]) / 2
    cr = (cr[:, 0::2] + cr[:, 1::2]) / 2
    c = lambda a: np.clip(np.rint(a), 0, 255).astype(np.uint8)
    y, cb, cr = c(y), c(cb), c(cr)
    h, w = y.shape
    out = np.empty((h, w // 2, 4), np.uint8)
    if uyvy:
        out[..., 0], out[..., 1], out[..., 2], out[..., 3] = cb, y[:, 0::2], cr, y[:, 1::2]
    else:
        out[..., 0], out[..., 1], out[..., 2], out[..., 3] = y[:, 0::2], cb, y[:, 1::2], cr
    return out.reshape(h, w * 2)


def yuyv_to_rgb(raw_path, w, h, fmt, ups, kset):
    with tempfile.TemporaryDirectory() as td:
        o = os.path.join(td, "o.rgb")
        subprocess.run([YREF, raw_path, str(w), str(h), str(fmt), str(ups), str(kset), o], check=True)
        return np.fromfile(o, np.uint8).reshape(h, w, 3)


def run_model(img, tw, th, ow, oh, top, left, fp):
    with tempfile.TemporaryDirectory() as td:
        s, d = os.path.join(td, "s.rgb"), os.path.join(td, "o.bin")
        np.ascontiguousarray(img).tofile(s)
        subprocess.run([MODEL, s, str(img.shape[1]), str(img.shape[0]), str(tw), str(th), str(ow), str(oh),
                        str(top), str(left), str(fp), d], check=True)
        return np.fromfile(d, np.int8)


def golden(rgb, cfg):
    """返回 (张量字节, 核寄存器几何)。cfg = ('sq', S) 或 ('rect', TW, TH)。"""
    h0, w0 = rgb.shape[:2]
    if cfg[0] == "sq":
        s = cfg[1]
        fp = dp.DEFAULT_FIX_POINT[s]
        g = dp.LetterboxGeometry(w0, h0, s)
        ref = dp.DpuPrep(s, fp, backend="cv2").full_tensor(rgb).reshape(-1)
        geo = dict(tw=s, th=s, ow=g.w, oh=g.h, top=g.top, left=g.left, fp=fp)
    else:
        tw, th = cfg[1], cfg[2]
        fp = 6 if tw == 640 else 7
        _, im = dp.resized_uint8(rgb, tw, "cv2")
        assert im.shape == (th, tw, 3)
        ref = dp.quant_lut(fp)[im].reshape(-1)
        geo = dict(tw=tw, th=th, ow=tw, oh=th, top=0, left=0, fp=fp)
    m = run_model(rgb, geo["tw"], geo["th"], geo["ow"], geo["oh"], geo["top"], geo["left"], geo["fp"])
    nd = int((m != ref).sum())
    if nd:
        sys.exit("C model vs dpu_prep differ by %d bytes (%s)" % (nd, cfg))
    geo["bypass"] = int(w0 == geo["ow"] and h0 == geo["oh"])
    return ref.view(np.uint8), geo


CFGS = [("sq", 640), ("sq", 512), ("sq", 416), ("rect", 640, 480), ("rect", 512, 384)]
CFGN = {CFGS[0]: "sq640", CFGS[1]: "sq512", CFGS[2]: "sq416", CFGS[3]: "r640x480", CFGS[4]: "r512x384"}


class Set:
    def __init__(self, name):
        self.name, self.lines, self.meta, self.frames = name, [], [], 0
        self.gdir = os.path.join(RUN, "golden", name, "tb_out")
        os.makedirs(self.gdir, exist_ok=True)
        for f in glob.glob(self.gdir + "/*"):
            os.remove(f)

    def add(self, cid, path, src_img_rgb, w0, h0, fmt, cfg, ups=0, kset=0, stride=None):
        t, geo = golden(src_img_rgb, cfg)
        bpp = 2 if fmt >= 2 else 3
        stride = stride or ceil16(w0 * bpp)
        self.lines.append("%s %s %d %d %d %d %d %d %d %d %d %d %d %d %d %d" % (
            cid, path, fmt, ups, geo["bypass"], w0, h0, stride, geo["tw"], geo["th"], geo["ow"], geo["oh"],
            geo["top"], geo["left"], geo["fp"], kset))
        t.tofile(os.path.join(self.gdir, cid + ".bin"))
        self.frames += 1
        self.meta.append("%s status=%08x untouched_outside=1" % (cid, (self.frames << 16) | (VERSION8 << 8)))

    def add_bad(self, cid, path, w0, h0, ow, oh):
        """非法几何（OW > W0）：期望 STATUS 置帧头错误位、整块目的缓冲一个字节都不动。"""
        self.lines.append("%s %s 0 0 0 %d %d %d 512 512 %d %d 0 0 7 0" % (cid, path, w0, h0, ceil16(w0 * 3), ow, oh))
        self.meta.append("%s status=%08x untouched_outside=1" % (cid, (self.frames << 16) | (VERSION8 << 8) | 1))

    def write(self):
        open(os.path.join(HLS, "vec_%s.txt" % self.name), "w").write(
            "# id infile fmt ups bypass w0 h0 stride tw th ow oh top left fp kset（gen_vec.py 生成）\n" + "\n".join(self.lines) + "\n")
        open(os.path.join(self.gdir, "meta.txt"), "w").write("\n".join(self.meta) + "\n")
        print(self.name, len(self.lines), "cases")


rng = np.random.RandomState(20261103)
real = sorted(glob.glob(DATA + "/*.png"))
pick = [real[i] for i in (2, 4, 6, 8, 0, 10, 12, 14)]   # 4 张 TUM fr1/desk + 4 张六类物体
reals = [(os.path.basename(p)[:-4].replace(".", "_"), np.asarray(Image.open(p).convert("RGB"))) for p in pick]
randA = rng.randint(0, 256, (480, 640, 3)).astype(np.uint8)
randB = rng.randint(0, 256, (480, 640, 3)).astype(np.uint8)
rand720 = rng.randint(0, 256, (720, 1280, 3)).astype(np.uint8)
randP = rng.randint(0, 256, (640, 480, 3)).astype(np.uint8)
rand1000 = rng.randint(0, 256, (750, 1000, 3)).astype(np.uint8)
ry1 = rng.randint(0, 256, (480, 1280)).astype(np.uint8)   # 随机 YUYV 字节
ry2 = rng.randint(0, 256, (480, 1280)).astype(np.uint8)
ry3 = rng.randint(0, 256, (480, 1280)).astype(np.uint8)

cs = Set("csim")
# 5 种配置按配置分组，组内几何相同 -> 只有每组第一帧 PARAM_RELOAD=1（也测了不重装路径）
for cfg in CFGS:
    for nm, img in reals:
        cs.add("%s_%s" % (CFGN[cfg], nm), save(nm, img), img, 640, 480, 0, cfg)
    cs.add("%s_randA" % CFGN[cfg], save("randA", randA), randA, 640, 480, 0, cfg)
    cs.add("%s_randB" % CFGN[cfg], save("randB", randB), randB, 640, 480, 0, cfg)
# 其他几何：16:9 2 倍缩小、竖图（左右灰边，640 档为旁路+左右灰边）、非整数比且行字节非 16 倍数（行步长 3008）
for nm, img in (("rand1280x720", rand720), ("rand480x640", randP), ("rand1000x750", rand1000)):
    p = save(nm, img)
    for s in (640, 512, 416):
        cs.add("sq%d_%s" % (s, nm), p, img, img.shape[1], img.shape[0], 0, ("sq", s))
# 行步长大于行字节（行间 64 字节空隙填 0xA5）
cs.add("sq512_randA_stride1984", save("randA", randA), randA, 640, 480, 0, ("sq", 512), stride=1984)
# BGR888
nm, img = reals[1]
cs.add("sq512_bgr_%s" % nm, save(nm + "_bgr", img[..., ::-1]), img, 640, 480, 1, ("sq", 512))
cs.add("sq416_bgr_randB", save("randB_bgr", randB[..., ::-1]), randB, 640, 480, 1, ("sq", 416))
# YUYV / UYVY：期望值 = yuyv_ref 转 RGB 后的参考前处理
yuv_cases = [
    ("sq512_yuyv_fancy_%s" % reals[0][0], rgb_to_yuyv(reals[0][1]), 2, 1, 0, ("sq", 512)),
    ("sq640_yuyv_copy_%s" % reals[4][0], rgb_to_yuyv(reals[4][1]), 2, 0, 0, ("sq", 640)),
    ("sq416_yuyv_copy_rand1", ry1, 2, 0, 0, ("sq", 416)),
    ("sq640_uyvy_fancy_rand2", ry2, 3, 1, 0, ("sq", 640)),
    ("r512x384_yuyv_fancy_bt601l_rand3", ry3, 2, 1, 1, ("rect", 512, 384)),
    ("sq512_yuyv_fancy_rand1", ry1, 2, 1, 0, ("sq", 512)),
]
for cid, raw, fmt, ups, kset, cfg in yuv_cases:
    p = save(cid, raw)
    rgb = yuyv_to_rgb(p, 640, 480, fmt, ups, kset)
    cs.add(cid, p, rgb, 640, 480, fmt, cfg, ups=ups, kset=kset)
cs.add_bad("bad_ow_gt_w0", save("randA", randA), 640, 480, 700, 384)
cs.add("sq512_after_bad_randA", save("randA", randA), randA, 640, 480, 0, ("sq", 512))
cs.write()

co = Set("cosim")
co.add("sq512_randA", save("randA", randA), randA, 640, 480, 0, ("sq", 512))
p = save("sq416_yuyv_fancy_rand1", ry1)
co.add("sq416_yuyv_fancy_rand1", p, yuyv_to_rgb(p, 640, 480, 2, 1, 0), 640, 480, 2, ("sq", 416), ups=1)
co.write()
