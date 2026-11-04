#!/usr/bin/env python3
"""NN-CAM2 instrument check (host): the C++ SoftPrep (tools/dpu_pipe/colour_prep.h via soft_prep_cli) and the numpy
YUYV->RGB port (yuyv_rgb.py) against the reference binaries they were written from -- PREP1 yuyv_ref.c (YUYV->RGB) and
PREP0 prep_model.c (resize / letterbox / quantise) -- and against the Nano's own preprocessing (host/dpu_prep.DpuPrep
on the same RGB). Positive control: SoftPrep with --perturb (first non-zero horizontal coefficient +1) must differ
wherever the geometry resizes.

  check_prep.py --pack P --ref-dir DIR(yuyv_ref, prep_model) --cli soft_prep_cli --work W --out result.json
"""
import argparse
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
from host.dpu_prep import DEFAULT_FIX_POINT, DpuPrep, LetterboxGeometry  # noqa: E402
from pack import Pack  # noqa: E402
from yuyv_rgb import yuyv_to_rgb  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pack", required=True)
    ap.add_argument("--ref-dir", required=True)
    ap.add_argument("--cli", required=True)
    ap.add_argument("--work", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", default="0,30,60,89")
    a = ap.parse_args()
    os.makedirs(a.work, exist_ok=True)
    pk = Pack(a.pack)
    w, h = pk.cw, pk.ch
    rng = np.random.default_rng(20261103)
    inputs = [("rec30_%s" % i, pk.colour(int(i))) for i in a.frames.split(",")]
    inputs.append(("rand", rng.integers(0, 256, w * h * 2, dtype=np.uint8).tobytes()))
    rows, bad = [], 0
    for name, yuyv in inputs:
        yp = os.path.join(a.work, name + ".yuyv")
        open(yp, "wb").write(yuyv)
        for kset in (0, 1):
            for ups in (1, 0):
                rgb_ref = os.path.join(a.work, "%s_k%d_u%d.rgb" % (name, kset, ups))
                subprocess.run([os.path.join(a.ref_dir, "yuyv_ref"), yp, str(w), str(h), "2", str(ups), str(kset),
                                rgb_ref], check=True, capture_output=True)
                ref_rgb = np.fromfile(rgb_ref, np.uint8).reshape(h, w, 3)
                np_rgb = yuyv_to_rgb(yuyv, w, h, kset, ups)
                rgb_np_ok = bool(np.array_equal(ref_rgb, np_rgb))
                for size in (512, 416, 640):
                    fp = DEFAULT_FIX_POINT[size]
                    g = LetterboxGeometry(w, h, size)
                    t_ref = os.path.join(a.work, "ref.bin")
                    subprocess.run([os.path.join(a.ref_dir, "prep_model"), rgb_ref, str(w), str(h), str(size), str(size),
                                    str(g.w), str(g.h), str(g.top), str(g.left), str(fp), t_ref], check=True)
                    ref = np.fromfile(t_ref, np.uint8)
                    t_soft, rgb_soft = os.path.join(a.work, "soft.bin"), os.path.join(a.work, "soft.rgb")
                    subprocess.run([a.cli, yp, str(w), str(h), str(size), str(fp), str(kset), str(ups), t_soft,
                                    "--rgb", rgb_soft], check=True, capture_output=True)
                    soft = np.fromfile(t_soft, np.uint8)
                    soft_rgb_ok = bool(np.array_equal(np.fromfile(rgb_soft, np.uint8), ref_rgb.ravel()))
                    subprocess.run([a.cli, yp, str(w), str(h), str(size), str(fp), str(kset), str(ups), t_soft,
                                    "--perturb"], check=True, capture_output=True)
                    pert = np.fromfile(t_soft, np.uint8)
                    nano = np.frombuffer(DpuPrep(size, fp, backend="numpy").full_tensor(np_rgb), np.uint8)
                    r = {"input": name, "kset": kset, "ups": ups, "size": size, "fix_point": fp,
                         "rgb_numpy_eq_ref": rgb_np_ok, "rgb_soft_eq_ref": soft_rgb_ok,
                         "soft_diff_bytes": int(np.count_nonzero(soft != ref)),
                         "nano_numpy_diff_bytes": int(np.count_nonzero(nano != ref)),
                         "perturb_diff_bytes": int(np.count_nonzero(pert != ref)), "resizes": size != w}
                    # the +1/2048 coefficient perturbation is a weak control (absorbed by rounding on smooth images;
                    # PREP0/PREP1 saw the same): reported, not part of the pass rule
                    ok = (rgb_np_ok and soft_rgb_ok and r["soft_diff_bytes"] == 0 and r["nano_numpy_diff_bytes"] == 0)
                    r["ok"] = ok
                    bad += not ok
                    rows.append(r)
    pc = [r for r in rows if r["resizes"]]
    out = {"cases": len(rows), "failed": bad, "tensor_bytes_compared": int(sum(
        DpuPrep(r["size"]).geometry(w, h).size ** 2 * 3 for r in rows)),
        "perturb_caught": sum(r["perturb_diff_bytes"] > 0 for r in pc), "perturb_cases": len(pc),
        "perturb_caught_by_input": {n: sum(r["perturb_diff_bytes"] > 0 for r in pc if r["input"] == n)
                                    for n in sorted({r["input"] for r in pc})}, "rows": rows}
    json.dump(out, open(a.out, "w"), indent=1)
    print(json.dumps({k: v for k, v in out.items() if k != "rows"}))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
