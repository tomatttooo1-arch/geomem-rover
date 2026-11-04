"""穷举所有 int8 logit（256 个值），回答：sigmoid 查表的分数误差会不会让任何框在阈值附近翻转。

核里所有「留/丢」判定都不用查表分数：
  - conf 阈值：类别 logit q >= cls_thr_q（主机 conf_to_q 换算，整数比较）
  - 车上按类下限：q >= cls_min_q（主机 floor_to_q 换算）
  - 排序与 NMS：按 q 与整数坐标
查表分数 sig_lut[q+128]（UQ0.16）只写进框表给下游看。所以要证两件事：
  A. 整数判定与浮点参考（float32 sigmoid 后与 float32 阈值比较，ultralytics/torch 的写法）在每个 q 上逐一相同；
     对 conf 取车上值和一整片扫描值都成立。
  B. 若下游拿查表分数再和某个阈值比（例如主机侧再按 0.35 过滤），会在哪些阈值上与浮点分数比较的结果不同——
     穷举 256 个 q 的「查表值与真值之间的区间」，列出最宽的，并核车上实际用到的阈值是否落进任何区间。
用法：python3 score_threshold_exhaustive.py [--out result.json]
"""
import argparse
import json
import math

import numpy as np


def sig32(x):
    x = np.float32(x)
    return np.float32(1.0) / (np.float32(1.0) + np.exp(-x, dtype=np.float32))


def conf_to_q(conf, fp):  # 与 detpost_host.cpp conf_to_q 逐行相同
    s = math.ldexp(1.0, -fp)
    L = math.log(conf / (1.0 - conf)) / s
    q = math.floor(L) + 1
    while q > -128 and sig32((q - 1) * s) > np.float32(conf):
        q -= 1
    while q < 128 and not (sig32(q * s) > np.float32(conf)):
        q += 1
    return q


def floor_to_q(f, fp):  # 与 tb/cli 的 floor_to_q 相同：score >= f
    s = math.ldexp(1.0, -fp)
    for q in range(-128, 128):
        if sig32(q * s) >= np.float32(f):
            return q
    return 127


def sig_lut(fp):  # 与 detpost_host.cpp make_luts 相同
    return [int(math.floor(65535.0 / (1.0 + math.exp(-(i - 128) * math.ldexp(1.0, -fp))) + 0.5)) for i in range(256)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out")
    a = ap.parse_args()
    res = {}
    qs = np.arange(-128, 128)
    car_thr = [0.25, 0.35, 0.45]  # conf 0.25；书包下限 0.35（33 类）/ 0.45（COCO）
    for fp in range(0, 8):
        s = math.ldexp(1.0, -fp)
        true32 = np.array([sig32(q * s) for q in qs], np.float32)
        true64 = np.array([1.0 / (1.0 + math.exp(-q * s)) for q in qs])
        lut = np.array(sig_lut(fp), np.float64) / 65535.0
        # A：整数判定 vs 浮点判定。conf 扫描：车上值 + 0.0001..0.9999 步长 1e-4 + 每个 sigmoid 点上下各一个 float32 ulp
        confs = set(car_thr) | {round(k * 1e-4, 4) for k in range(1, 10000)}
        for v in true32:
            if 0.0 < v < 1.0:
                confs.add(float(np.nextafter(v, np.float32(0))))
                confs.add(float(v))
                confs.add(float(np.nextafter(v, np.float32(1))))
        flips_gt, flips_ge = [], []
        for c in sorted(confs):
            if not (0.0 < c < 1.0):
                continue
            t = conf_to_q(c, fp)
            hw = qs >= t
            ref = true32 > np.float32(c)
            if (hw != ref).any():
                flips_gt.append((c, [int(q) for q in qs[hw != ref]]))
            m = true32 >= np.float32(c)  # floor_to_q 的向量写法：第一个满足 sig32 >= f 的 q（逐值同 C 循环）
            tf = int(qs[m.argmax()]) if m.any() else 127
            hw2 = qs >= tf
            ref2 = true32 >= np.float32(c)
            if (hw2 != ref2).any():
                flips_ge.append((c, [int(q) for q in qs[hw2 != ref2]]))
        # B：查表分数 vs 真值（float64 与 float32）——区间 [min, max) 内的阈值会翻转
        err64 = np.abs(lut - true64)
        err32 = np.abs(lut - true32.astype(np.float64))
        lo = np.minimum(lut, true32.astype(np.float64))
        hi = np.maximum(lut, true32.astype(np.float64))
        car_hit = {str(t): [int(q) for q in qs[(lo <= t) & (t <= hi)]] for t in car_thr}
        # 每个车上阈值到最近的 sigmoid 点（float32）的距离：比查表误差大多少
        car_margin = {str(t): float(np.min(np.abs(true32.astype(np.float64) - t))) for t in car_thr}
        res["fp%d" % fp] = dict(
            n_conf_tested=len(confs), flips_conf_gt=len(flips_gt), flips_floor_ge=len(flips_ge),
            flip_examples=(flips_gt[:3] + flips_ge[:3]),
            lut_err_max_vs_f64=float(err64.max()), lut_err_max_vs_f32=float(err32.max()),
            lut_err_argmax_q=int(qs[err32.argmax()]),
            car_threshold_inside_lut_interval=car_hit, car_threshold_nearest_sigmoid_point=car_margin,
            cls_thr_q_at_0p25=conf_to_q(0.25, fp))
    for k, v in res.items():
        print(k, "confs=%d flips(>conf)=%d flips(>=floor)=%d lut_err_max=%.2e car_hits=%s nearest=%s thr_q=%d" % (
            v["n_conf_tested"], v["flips_conf_gt"], v["flips_floor_ge"], v["lut_err_max_vs_f32"],
            v["car_threshold_inside_lut_interval"], {t: "%.4f" % m for t, m in v["car_threshold_nearest_sigmoid_point"].items()},
            v["cls_thr_q_at_0p25"]))
    if a.out:
        json.dump(res, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
