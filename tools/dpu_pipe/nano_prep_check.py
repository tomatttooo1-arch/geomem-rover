#!/usr/bin/env python3
"""DPUPIPE0 判据 T5 的板上一步：在 Nano（或任何机器）上用 host/dpu_prep.py 重算一组图的 DPU 输入张量，
与宿主上已对 dev3 评测器（ultralytics 8.4.69 验证器像素＋xmodel data-fix）逐字节核过的 sha256 比对。
  make:  python3 nano_prep_check.py --make expected.json IMG...      （宿主：生成期望值，numpy 后端）
  check: python3 nano_prep_check.py --check expected.json --imgdir D  （板上：numpy、cv2、auto 三种后端都核）
输出 JSON：每个后端的 checked / mismatched、cv2 版本与自检结果、每档预处理耗时中位（板上数）。
图片按文件名对应（--imgdir 下同名文件）；另含两张合成图（不依赖文件）。
"""
import argparse
import hashlib
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
import numpy as np  # noqa: E402
from host import dpu_prep  # noqa: E402

SIZES = ((640, 6), (512, 7), (416, 7))


def synth(w, h, seed):
    r = np.random.RandomState(seed)
    y, x = np.mgrid[0:h, 0:w]
    im = np.stack([(x * 255) // (w - 1), (y * 255) // (h - 1), ((x // 7 + y // 5) * 37) % 256], -1).astype(np.int32)
    return np.clip(im + r.randint(-20, 21, (h, w, 3)), 0, 255).astype(np.uint8)


def load(path):
    import cv2
    return cv2.imread(path, cv2.IMREAD_COLOR)[..., ::-1].copy()


def images(paths):
    out = {"synth_640x480": synth(640, 480, 7), "synth_1280x720": synth(1280, 720, 7)}
    for p in paths:
        out[os.path.basename(p)] = load(p)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--make", default=None)
    ap.add_argument("--check", default=None)
    ap.add_argument("--imgdir", default=".")
    ap.add_argument("--out", default=None)
    ap.add_argument("--reps", type=int, default=20, help="timing repetitions per size (640x480 frames)")
    ap.add_argument("imgs", nargs="*")
    a = ap.parse_args()
    if a.make:
        exp = {}
        for name, im in images(a.imgs).items():
            for size, fp in SIZES:
                t = dpu_prep.DpuPrep(size, fp, "numpy").full_tensor(im)
                exp["%s|%d|%d" % (name, size, fp)] = hashlib.sha256(t.tobytes()).hexdigest()
        json.dump(exp, open(a.make, "w"), indent=1, sort_keys=True)
        print(json.dumps({"made": len(exp)}))
        return 0
    exp = json.load(open(a.check))
    names = sorted({k.split("|")[0] for k in exp})
    files = [os.path.join(a.imgdir, n) for n in names if not n.startswith("synth_")]
    ims = images(files)
    rep = {"python": sys.version.split()[0], "numpy_version": np.__version__}
    try:
        import cv2
        rep["cv2_version"] = cv2.__version__
    except Exception:
        rep["cv2_version"] = None
    rep["cv2_selftest_640x480"] = {s: dpu_prep.selftest_cv2(640, 480, s) for s, _ in SIZES}
    ok = True
    for be in ("numpy", "cv2", "auto"):
        if be == "cv2" and rep["cv2_version"] is None:
            continue
        chk = mism = 0
        bad = []
        for key, sha in sorted(exp.items()):
            name, size, fp = key.split("|")
            t = dpu_prep.DpuPrep(int(size), int(fp), be).full_tensor(ims[name])
            chk += 1
            if hashlib.sha256(t.tobytes()).hexdigest() != sha:
                mism += 1
                bad.append(key)
        rep[be] = {"checked": chk, "mismatched": mism, "first_bad": bad[:5]}
        ok = ok and (mism == 0 or be == "cv2")    # cv2 forced may differ; auto must not
    cam = [im for n, im in ims.items() if im.shape[:2] == (480, 640)]
    timing = {}
    for size, fp in SIZES:
        for be in ("numpy", "auto"):
            p = dpu_prep.DpuPrep(size, fp, be)
            p.content(cam[0])
            ts = []
            for i in range(a.reps):
                t0 = time.perf_counter()
                p.content(cam[i % len(cam)])
                ts.append(1e3 * (time.perf_counter() - t0))
            timing["%d_%s" % (size, be)] = round(sorted(ts)[len(ts) // 2], 3)
    rep["prep_ms_p50"] = timing
    rep["T5_pass"] = ok
    print(json.dumps(rep))
    if a.out:
        json.dump(rep, open(a.out, "w"), indent=1)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
