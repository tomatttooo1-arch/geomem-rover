#!/usr/bin/env python3
"""RECOG640 (2026-11-04): one switch for the recognition model of dpu_rx / kv2_chain.sh.

    RECOG_MODEL=y8s640|y8s640r|y8s512   (default: profiles.json "default" = y8s640)
    RECOG_MODEL_DIR   directory holding the xmodel files      [~/recog/models]
    RECOG_XMODEL      explicit xmodel path (wins over RECOG_MODEL_DIR)
    RECOG_THR_MEDBOX  medicine box threshold override (default: the model's threshold-selection.json value)
    RECOG_CLS_THR     all six thresholds "t0,...,t5" (cup,bottle,banana,laptop,backpack,medicine box); RECOG_THR_MEDBOX still wins for class 5

    recog_args.py [--model M] [--format args|env|json] [--check-xmodel]
      args: dpu_rx flags (--size --fix --xmodel --nc --model-tag + output processing)
      env : RECOG_* shell assignments for eval (kv2_chain.sh)
Never refuses: problems (unknown / not-ready profile, sha mismatch, missing file) are printed as WARN on stderr;
an unknown or not-ready profile falls back to the default one (y8s640r is reserved until dev3 delivers it).
Stdlib only (runs on the KV260)."""
import argparse, hashlib, json, os, shlex, sys

HERE = os.path.dirname(os.path.abspath(__file__))


def warn(m):
    print("WARN recog_args: " + m, file=sys.stderr)


def sha256(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def resolve(model=None, env=os.environ, check_xmodel=False):
    P = json.load(open(os.path.join(HERE, "profiles.json")))
    names = P["names"]
    want = model or env.get("RECOG_MODEL") or P["default"]
    prof = P["profiles"].get(want)
    if prof is None:
        warn("unknown RECOG_MODEL=%s, using %s" % (want, P["default"])); want = P["default"]; prof = P["profiles"][want]
    elif not prof.get("ready"):
        warn("RECOG_MODEL=%s is reserved (not delivered yet), using %s" % (want, P["default"]))
        want = P["default"]; prof = P["profiles"][want]
    mdir = os.path.expanduser(env.get("RECOG_MODEL_DIR", "~/recog/models"))
    xmodel = env.get("RECOG_XMODEL") or os.path.join(mdir, prof["xmodel"])
    if not os.path.exists(xmodel):
        warn("xmodel %s not found" % xmodel)
    elif check_xmodel and sha256(xmodel) != prof["xmodel_sha256"]:
        warn("xmodel %s sha256 differs from the profile's %s..." % (xmodel, prof["xmodel_sha256"][:12]))
    r = {"model": want, "size": prof["tensor_w"], "tensor_w": prof["tensor_w"], "tensor_h": prof["tensor_h"],
         "fix": prof["fix"], "nc": prof["nc"], "xmodel": xmodel, "names": names, "post": dict(prof["post"]),
         "cls_thr": None, "thr_source": None}
    if not prof["post"].get("legacy"):
        tj = os.path.join(HERE, prof["thresholds_json"])
        if sha256(tj) != prof["thresholds_sha256"]:
            warn("%s sha256 differs from the profile (edited?)" % tj)
        t = json.load(open(tj))["thresholds"]
        thr = [float(t[n]) for n in names]
        src = {n: "threshold-selection.json" for n in names}
        if env.get("RECOG_CLS_THR"):
            v = [float(x) for x in env["RECOG_CLS_THR"].split(",")]
            if len(v) == len(names): thr = v; src = {n: "RECOG_CLS_THR" for n in names}
            else: warn("RECOG_CLS_THR needs %d values, ignored" % len(names))
        if env.get("RECOG_THR_MEDBOX"):
            thr[names.index("medicine box")] = float(env["RECOG_THR_MEDBOX"]); src["medicine box"] = "RECOG_THR_MEDBOX"
        r["cls_thr"] = thr; r["thr_source"] = src
    return r


def dpu_args(r):
    a = ["--size", str(r["size"]), "--fix", str(r["fix"]), "--xmodel", r["xmodel"], "--nc", str(r["nc"]),
         "--model-tag", r["model"]]
    return a + post_args(r)


def post_args(r):
    p = r["post"]
    if p.get("legacy"): return []
    a = ["--conf", repr(p["conf"]), "--iou", repr(p["iou"]), "--max-det", str(p["max_det"]),
         "--cls-thr", ",".join(repr(x) for x in r["cls_thr"])]
    return a + (["--sort-heads"] if p.get("sort_heads") else [])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model")
    ap.add_argument("--format", default="args", choices=["args", "env", "json"])
    ap.add_argument("--check-xmodel", action="store_true")
    a = ap.parse_args()
    r = resolve(a.model, check_xmodel=a.check_xmodel)
    if a.format == "args": print(" ".join(shlex.quote(x) for x in dpu_args(r)))
    elif a.format == "json": print(json.dumps(r, indent=1))
    else:
        for k, v in (("RECOG_MODEL", r["model"]), ("RECOG_SIZE", r["size"]), ("RECOG_FIX", r["fix"]),
                     ("RECOG_XMODEL", r["xmodel"]), ("RECOG_NC", r["nc"]),
                     ("RECOG_POST_ARGS", " ".join(shlex.quote(x) for x in post_args(r) + ["--nc", str(r["nc"]), "--model-tag", r["model"]]))):
            print("%s=%s" % (k, shlex.quote(str(v))))


if __name__ == "__main__":
    main()
