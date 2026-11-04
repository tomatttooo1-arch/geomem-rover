#!/usr/bin/env python3
"""DPUPIPE1 (2026-11-03) board correctness check of the C++ VART back end (run on the KV260 after
dpu_infer_check). Criteria in runs/.../DPUPIPE1/README.md P1:
  C1 raw outputs: C++ VartEngine (dpu_infer_check *.out.bin) == Python VART (DPUB1 full1.py way) on the same
     input tensor, byte for byte, every head;
  C2 boxes: C++ det_post on those raw outputs == det_post_ref (DPUB1 dfl_decode_f + nms, argsort kind="stable")
     with this board's numpy, every float32 value;
  C3 (report only): DPUB1's default (unstable) argsort, set equality;
  C4 discrimination: one input byte changed -> raw output checksum changes; one raw output byte of a surviving
     cell changed -> the C2 comparator reports a difference.

  python3 board_check.py --xmodel M --indir IN --cxx cxx.jsonl --cxxdir OUTDIR --out check.json
"""
import argparse
import ctypes
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import det_post_ref as R  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xmodel", required=True)
    ap.add_argument("--indir", required=True)
    ap.add_argument("--cxx", required=True, help="dpu_infer_check stdout (JSON lines)")
    ap.add_argument("--cxxdir", required=True, help="dpu_infer_check --out-dir")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    import vart
    import xir
    g = xir.Graph.deserialize(a.xmodel)
    sg = [s for s in g.get_root_subgraph().toposort_child_subgraph()
          if s.has_attr("device") and s.get_attr("device") == "DPU"][0]
    r = vart.Runner.create_runner(sg, "run")
    it = r.get_input_tensors()[0]
    ots = r.get_output_tensors()
    H = it.dims[1]
    ofp = [t.get_attr("fix_point") for t in ots]
    outs = [np.zeros(tuple(t.dims), np.int8) for t in ots]

    def py_vart(x):
        r.wait(r.execute_async([x.reshape(tuple(it.dims))], outs))
        return [o.copy() for o in outs]

    lines = [json.loads(l) for l in open(a.cxx) if l.strip()]
    eng = lines[0]["engine"]
    rows = lines[1:]
    res = {"xmodel": os.path.basename(a.xmodel), "engine": eng, "numpy": np.__version__, "items": []}
    # diagnostic: does this numpy's float32 exp equal glibc expf?
    lm = ctypes.CDLL("libm.so.6")
    lm.expf.restype = ctypes.c_float
    lm.expf.argtypes = [ctypes.c_float]
    xs = np.linspace(-30, 0, 5001, dtype=np.float32)
    res["numpy_exp_vs_glibc_differ_frac"] = float(np.mean(np.exp(xs) != np.array([lm.expf(float(v)) for v in xs], np.float32)))
    c1_bad = c2_bad = c3_diff = 0
    n_dets = 0
    for row in rows:
        name = row["name"]
        x = np.fromfile(os.path.join(a.indir, name + ".in.bin"), np.int8)
        py = py_vart(x)
        cx_raw = open(os.path.join(a.cxxdir, name + ".out.bin"), "rb").read()
        heads = R.parse_heads(row["heads"])
        cx = R.heads_from_bytes(cx_raw, heads)
        # C1: match heads by order (both are the runner's output order); shapes must agree
        c1 = len(py) == len(cx) and all(p.shape == c.shape and np.array_equal(p, c) for p, (c, _) in zip(py, cx))
        py_sum = [int(o.astype(np.int64).sum()) for o in py]
        # C2: reference on the C++ raw outputs
        _, det_s = R.run(cx, H, "stable")
        _, det_d = R.run(cx, H, None)
        cd = np.array(row["dets"], np.float32).reshape(-1, 6)
        ref = det_s.astype(np.float32)
        c2 = cd.shape == ref.shape and np.array_equal(cd, ref)
        maxdiff = float(np.abs(cd - ref).max()) if (cd.shape == ref.shape and len(cd)) else (0.0 if cd.shape == ref.shape else None)
        c3 = set(map(tuple, det_d.astype(np.float32).tolist())) == set(map(tuple, ref.tolist()))
        c1_bad += not c1
        c2_bad += not c2
        c3_diff += not c3
        n_dets += len(cd)
        res["items"].append({"name": name, "c1_raw_equal": bool(c1), "py_out_sum": py_sum, "cxx_out_sum": row["out_sum"],
                             "c2_boxes_equal": bool(c2), "c2_max_abs_diff": maxdiff, "n_dets": int(len(cd)),
                             "n_dets_ref": int(len(ref)), "candidates": row["candidates"],
                             "c3_default_sort_same_set": bool(c3), "cxx_stage_ms": row["stage_ms"],
                             "cxx_infer_ms": row["infer_ms"], "cxx_post_us": row["post_us"]})
    # C4a: one input byte changed -> raw output changes
    row = rows[0]
    x = np.fromfile(os.path.join(a.indir, row["name"] + ".in.bin"), np.int8).copy()
    base = [int(o.astype(np.int64).sum()) for o in py_vart(x)]
    k = x.size // 2
    x[k] = np.int8(127 if x[k] < 0 else -128)
    pert = [int(o.astype(np.int64).sum()) for o in py_vart(x)]
    c4a = base != pert
    # C4b: comparator discrimination on an item with boxes
    c4b = None
    for row in rows:
        if not row["dets"]:
            continue
        heads = R.parse_heads(row["heads"])
        raw = bytearray(open(os.path.join(a.cxxdir, row["name"] + ".out.bin"), "rb").read())
        # perturb the highest-score cell over all heads: it is always kept by NMS (first box), so changing one of its
        # box-logit bytes must change the output. (The first above-threshold cell may be NMS-suppressed -> no change.)
        off = 0
        best = None
        for h, w, c, fp in heads:
            arr = np.frombuffer(bytes(raw[off:off + h * w * c]), np.int8).reshape(h * w, c)
            sc = arr[:, 64:].max(1) * (2.0 ** -fp)
            i = int(np.argmax(sc))
            if best is None or sc[i] > best[0]:
                best = (float(sc[i]), off + i * c + 3)
            off += h * w * c
        done = best is not None and best[0] > np.log(1 / 3)
        if done:
            p = best[1]
            raw[p] = (raw[p] + 40) % 256
            res["c4b_perturbed_byte"] = p
        if done:
            _, det = R.run(R.heads_from_bytes(bytes(raw), heads), H, "stable")
            cd = np.array(row["dets"], np.float32).reshape(-1, 6)
            c4b = not (cd.shape == det.shape and np.array_equal(cd, det.astype(np.float32)))
            break
    res.update({"n_items": len(rows), "c1_bad": c1_bad, "c2_bad": c2_bad, "c3_default_sort_set_differs": c3_diff,
                "dets_total": n_dets, "c4a_input_byte_changes_output": bool(c4a), "c4b_comparator_detects_change": c4b,
                "PASS": c1_bad == 0 and c2_bad == 0 and bool(c4a) and c4b is not False})
    json.dump(res, open(a.out, "w"), indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != "items"}))


if __name__ == "__main__":
    main()
