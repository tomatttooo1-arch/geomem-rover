"""定点参考模型（detpost_cli）对浮点参考（detpost_float.py）的逐帧比较。

两边吃同一个 .bin。对齐方式：先按锚点序号配对（两边都是从同一个锚点出框，这是最严的对照），
配上的比类别、坐标、分数；配不上的列出来并判读原因（分数同档顺序、阈值边界、IoU 边界）。
另带阳性对照：把定点输出人为改坏（某框挪 1 像素 / 删一个框 / 改类别），比较器必须报出来。

用法：python3 compare.py --cli ./build/detpost_cli --frames DIR [--nc 33] [--out report.json]
"""
import argparse
import glob
import json
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import detpost_float as dfl  # noqa: E402


def run_fixed(cli, path, nc, conf, iou_frac, max_det, extra=()):
    out = subprocess.run([cli, path, "--nc", str(nc), "--conf", str(conf), "--iou", iou_frac,
                          "--max-det", str(max_det), *extra], check=True, capture_output=True, text=True).stdout
    d = json.loads(out)
    b = np.asarray(d["boxes"], np.float64).reshape(-1, 8)
    return dict(boxes=b[:, :4], cls=b[:, 4].astype(int), level=b[:, 5].astype(int), score=b[:, 6],
                anchor=b[:, 7].astype(int), meta={k: v for k, v in d.items() if k != "boxes"})


def compare_frame(fx, fl):
    a_fx = {int(a): i for i, a in enumerate(fx["anchor"])}
    a_fl = {int(a): i for i, a in enumerate(fl["anchor"])}
    common = sorted(set(a_fx) & set(a_fl), key=lambda a: a_fl[a])
    only_fx = sorted(set(a_fx) - set(a_fl))
    only_fl = sorted(set(a_fl) - set(a_fx))
    r = dict(n_fixed=len(a_fx), n_float=len(a_fl), n_common=len(common), only_fixed=only_fx, only_float=only_fl)
    if common:
        i_fx = np.array([a_fx[a] for a in common])
        i_fl = np.array([a_fl[a] for a in common])
        d = np.abs(fx["boxes"][i_fx] - fl["boxes"][i_fl])
        r.update(cls_agree=int((fx["cls"][i_fx] == fl["cls"][i_fl]).sum()),
                 coord_err_max=float(d.max()), coord_err_mean=float(d.mean()),
                 coord_err_p99=float(np.percentile(d, 99)),
                 score_err_max=float(np.abs(fx["score"][i_fx] - fl["score"][i_fl]).max()),
                 order_same=bool(list(fx["anchor"]) == list(fl["anchor"])))
    else:
        r.update(cls_agree=0, coord_err_max=0.0, coord_err_mean=0.0, coord_err_p99=0.0, score_err_max=0.0,
                 order_same=bool(len(a_fx) == len(a_fl) == 0))
    r["identical_set"] = not only_fx and not only_fl
    return r


def positive_controls(fx, fl):
    """把定点结果人为改坏，确认比较器能报出（仪器判别力）。"""
    out = {}
    if len(fx["anchor"]) == 0:
        return dict(skipped="no boxes")
    g = {k: (v.copy() if isinstance(v, np.ndarray) else v) for k, v in fx.items()}
    g["boxes"][0, 0] += 1.0
    out["shift_1px_detected"] = compare_frame(g, fl)["coord_err_max"] >= 0.5
    g = {k: (v[1:] if isinstance(v, np.ndarray) else v) for k, v in fx.items()}
    out["drop_box_detected"] = not compare_frame(g, fl)["identical_set"]
    g = {k: (v.copy() if isinstance(v, np.ndarray) else v) for k, v in fx.items()}
    g["cls"][0] = (g["cls"][0] + 1) % 1000
    out["cls_flip_detected"] = compare_frame(g, fl)["cls_agree"] < compare_frame(fx, fl)["cls_agree"] or \
        compare_frame(fx, fl)["cls_agree"] == 0
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cli", required=True)
    ap.add_argument("--frames", required=True, help="目录，取其中全部 .bin")
    ap.add_argument("--nc", type=int, default=33)
    ap.add_argument("--fp", type=int, default=2)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", default="7/10")
    ap.add_argument("--max-det", type=int, default=100)
    ap.add_argument("--min-score", default="", help="cls=floor,...（车上：书包下限）")
    ap.add_argument("--no-offset-trick", action="store_true", help="浮点参考用逐类 NMS 代替 ultralytics 的坐标平移")
    ap.add_argument("--rank-by-level", action="store_true", help="浮点参考按 int8 logit 排序（去掉 float32 sigmoid 饱和同分）")
    ap.add_argument("--max-nms", type=int, default=30000, help="浮点参考 NMS 前候选上限（ultralytics 30000；定点 CAND_MAX=1024）")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    num, den = (int(v) for v in a.iou.split("/"))
    mins = {int(k): float(v) for k, v in (p.split("=") for p in a.min_score.split(",") if p)}
    extra = ("--min-score", a.min_score) if a.min_score else ()
    files = sorted(glob.glob(os.path.join(a.frames, "*.bin")))
    rows, agg = [], dict(frames=0, identical_set=0, order_same=0, boxes_fixed=0, boxes_float=0, common=0,
                         cls_agree=0, coord_err_max=0.0, score_err_max=0.0, overflow_frames=0)
    pcs = []
    for p in files:
        buf = open(p, "rb").read()
        fx = run_fixed(a.cli, p, a.nc, a.conf, a.iou, a.max_det, extra)
        fl = dfl.postprocess(buf, a.nc, a.fp, a.conf, num / den, a.max_det, min_scores=mins,
                             offset_trick=not a.no_offset_trick, rank_by_level=a.rank_by_level, max_nms=a.max_nms)
        r = compare_frame(fx, fl)
        r.update(frame=os.path.basename(p), n_cand_fixed=fx["meta"]["n_cand"], n_cand_float=fl["n_cand"],
                 overflow=fx["meta"]["overflow"])
        rows.append(r)
        agg["frames"] += 1
        agg["identical_set"] += r["identical_set"]
        agg["order_same"] += r["order_same"]
        agg["boxes_fixed"] += r["n_fixed"]
        agg["boxes_float"] += r["n_float"]
        agg["common"] += r["n_common"]
        agg["cls_agree"] += r["cls_agree"]
        agg["coord_err_max"] = max(agg["coord_err_max"], r["coord_err_max"])
        agg["score_err_max"] = max(agg["score_err_max"], r["score_err_max"])
        agg["overflow_frames"] += r["overflow"]
        if len(pcs) < 5 and r["n_fixed"]:
            pcs.append(dict(frame=r["frame"], **positive_controls(fx, fl)))
    res = dict(args=vars(a), summary=agg, positive_controls=pcs, frames=rows)
    print(json.dumps(agg))
    print("positive controls:", json.dumps(pcs))
    for r in rows:
        if not r["identical_set"] or not r["order_same"]:
            print("DIFF", r["frame"], "fixed", r["n_fixed"], "float", r["n_float"], "only_fixed", r["only_fixed"][:8],
                  "only_float", r["only_float"][:8], "order_same", r["order_same"], "overflow", r["overflow"])
    if a.out:
        with open(a.out, "w") as f:
            json.dump(res, f, indent=1)


if __name__ == "__main__":
    main()
