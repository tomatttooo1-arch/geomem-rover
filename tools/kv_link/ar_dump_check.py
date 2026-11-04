#!/usr/bin/env python3
"""Check AR presenter dumps (m7ar_present --dump-dir) pixel by pixel against the numpy golden.

Each dump ``ar_<seq>.{json,depth,grid,desc,fb}`` holds the exact inputs the AR core got
(the 24 registers as written, including the age computed at ap_start) and the framebuffer
read back from the scanned buffer after the flip (cache invalidated). The core samples the
permit wire itself: ``permit_before``/``permit_after`` are PERMIT STATUS b0 read right before
ap_start and right after ap_done; if they differ the wire value is unknown and the dump is
void (counted, never guessed). The golden (tools/ar_display/golden.py, an independent numpy
implementation of AR_DISPLAY_SPEC) renders the same inputs with permit_i = that value.
Optionally the g++ C reference (tools/ar_display/ar_ref_adapter.cpp + hw/m7/ar_overlay.cpp)
is a third witness (--c-ref). --png-out writes framebuffer PNGs (needs Pillow).

Colour main view (2026-10-16): a dump whose meta "colour".composited is 1 also holds ``.fbpl`` (the
framebuffer as the PL wrote it, copied before the PS composition) and ``.rgb`` (the RGB888 colour the
helper got). Then two checks, both pixel by pixel: ``.fbpl`` == the numpy golden (the old criterion,
unchanged) and ``.fb`` == tools/ar_display/colour_compose.compose(golden, colour, PiP geometry) (PiP of
the PL depth picture + colour outside the PL overlay mask); the C reference is compared with ``.fbpl``.
Not composited (colour off, unpaired, not fresh): ``.fb`` == golden as before.
2026-10-17 (AR_DEPTH_INVALID_STALL_20261017): composition is allowed for golden mode 0 (fresh) and mode 4
(NODATA, the no-depth self pull); a composited dump in any other mode fails (``composed_mode_ok``).

Mini map 2x (owner 10-29): a dump whose meta has "mini" (helper --mini-scale 2) is also checked for the
enlargement rule. Expected: enlarged iff the scale is > 1, the frame is in colour mode (meta colour.mode_on),
laid out for it (meta colour.mini2x, the message flag b1) and the target fits the canvas
(colour_compose.mini_fits); ``mini_rule_ok`` = the helper's "applied" equals that. An enlarged dump also
holds ``.fbpl`` (the PL picture before any CPU step): ``.fbpl`` == golden, ``.fb`` ==
colour_compose.scale_mini(compose(golden) or golden).

Usage: ar_dump_check.py --dump DIR [--png-out DIR] [--c-ref] [--json OUT.json]
Exit 0 only when every non-void dump equals the golden (status and every pixel).
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools", "ar_display"))
import golden  # noqa: E402
import colour_compose  # noqa: E402

MODES = {0: "LIVE/REPLAY", 1: "BADFRAME", 2: "STALE", 3: "POSEBAD", 4: "NODATA", 5: "NOAGE"}


def load_dump(base):
    meta = json.load(open(base + ".json"))
    W, H = meta["width"], meta["height"]
    d = {"meta": meta,
         "depth": np.fromfile(base + ".depth", dtype="<u2"),
         "grid": np.fromfile(base + ".grid", dtype=np.uint8),
         "desc": np.fromfile(base + ".desc", dtype="<u4"),
         "fb": np.fromfile(base + ".fb", dtype="<u4").reshape(H, W)}
    col = meta.get("colour") or {}
    if col.get("composited") or (meta.get("mini") or {}).get("applied"):
        d["fbpl"] = np.fromfile(base + ".fbpl", dtype="<u4").reshape(H, W)
    if col.get("composited"):
        d["rgb"] = np.fromfile(base + ".rgb", dtype=np.uint8).tobytes()
    return d


def c_reference(scratch):
    sys.path.insert(0, os.path.join(ROOT, "tools", "ar_display"))
    import check_golden
    return check_golden.build(scratch, "car_ar_ref")


def check_one(base, lib=None):
    d = load_dump(base)
    m = d["meta"]
    out = {"seq": m["seq"], "kind": m["kind"], "frame_id": m["frame_id"], "status": m["status"],
           "permit_before": m["permit_before"], "permit_after": m["permit_after"]}
    if m["permit_before"] < 0 or m["permit_before"] != m["permit_after"]:
        out.update(void="permit wire changed during the render (or unreadable)", equal=None)
        return out, d
    regs = [int(v) for v in m["regs"]]
    regs[15] = int(m["permit_before"])
    st, fb, info = golden.render(d["depth"], d["grid"], d["desc"], regs)
    out.update(golden_status=st, mode=info.get("mode"), mode_name=MODES.get(info.get("mode")),
               panel=info.get("panel"), boxes=info.get("boxes"), age_ms=regs[8],
               age_valid=regs[9], replay=regs[11])
    if fb is None:
        out.update(equal=(st == m["status"]), differing_pixels=None)
        return out, d
    col = m.get("colour") or {}
    mini = m.get("mini")
    mini_on = False
    if mini is not None:
        scale = int(mini["scale"])
        mini_on = bool(scale > 1 and col.get("mode_on") and col.get("mini2x")
                       and colour_compose.mini_fits(regs, scale))
        out.update(mini_scale=scale, mini_applied=int(mini["applied"]), mini_expected=int(mini_on),
                   mini_rule_ok=int(mini["applied"]) == int(mini_on))
    pl = d["fb"]
    if mini_on and not col.get("composited"):
        pl = d["fbpl"]
        pdiff = pl != fb
        npl = int(pdiff.sum())
        want = colour_compose.scale_mini(fb, regs, scale)
        cdiff = d["fb"] != want
        nc = int(cdiff.sum())
        out.update(colour="mode_on_not_composited", pl_differing_pixels=npl,
                   pl_first_diff=None if npl == 0 else [int(v) for v in np.argwhere(pdiff)[0]],
                   mini_differing_pixels=nc, differing_pixels=npl + nc,
                   equal=(npl == 0 and nc == 0 and st == m["status"] and out["mini_rule_ok"]),
                   first_diff=None if nc == 0 else [int(v) for v in np.argwhere(cdiff)[0]])
    elif col.get("composited"):
        pl = d["fbpl"]
        pdiff = pl != fb
        npl = int(pdiff.sum())
        want = colour_compose.compose(fb, regs, d["desc"], d["rgb"], col["pip"])
        if mini_on:
            want = colour_compose.scale_mini(want, regs, scale)
        cdiff = d["fb"] != want
        nc = int(cdiff.sum())
        Wd, Hd, dx, dy = regs[2], regs[3], regs[4], regs[5]
        mask = colour_compose.overlay_mask(regs, d["desc"])[dy:dy + Hd, dx:dx + Wd]
        out.update(colour="composited", pl_differing_pixels=npl,
                   pl_first_diff=None if npl == 0 else [int(v) for v in np.argwhere(pdiff)[0]],
                   colour_pixels=int((~mask).sum()), overlay_pixels_in_depth=int(mask.sum()),
                   pip=list(col["pip"]), composite_differing_pixels=nc,
                   composed_mode_ok=info.get("mode") in (0, 4),
                   differing_pixels=npl + nc, equal=(npl == 0 and nc == 0 and st == m["status"]
                                                     and info.get("mode") in (0, 4)
                                                     and out.get("mini_rule_ok", True)),
                   first_diff=None if nc == 0 else [int(v) for v in np.argwhere(cdiff)[0]])
    else:
        if col:
            out["colour"] = "mode_on_not_composited" if col.get("mode_on") else "off"
        diff = d["fb"] != fb
        n = int(diff.sum())
        out.update(differing_pixels=n, equal=(n == 0 and st == m["status"] and out.get("mini_rule_ok", True)),
                   first_diff=None if n == 0 else [int(v) for v in np.argwhere(diff)[0]])
    if lib is not None:
        import check_golden
        cst, cfb = check_golden.run_c(lib, {"depth": d["depth"], "grid": d["grid"], "desc": d["desc"],
                                            "regs": regs})
        W, H = regs[0], regs[1]
        cimg = cfb[:W * H].reshape(H, W)
        out.update(c_ref_status=cst, c_ref_equal_fb=bool(np.array_equal(cimg, pl)),
                   c_ref_sentinel_left=int((cfb[W * H:] != 0xDEADBEEF).sum()))
    return out, d


def write_png(fb, path):
    from PIL import Image
    rgb = np.stack([(fb >> 16) & 0xFF, (fb >> 8) & 0xFF, fb & 0xFF], axis=-1).astype(np.uint8)
    Image.fromarray(rgb).save(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dump", required=True)
    ap.add_argument("--png-out", default=None)
    ap.add_argument("--c-ref", action="store_true")
    ap.add_argument("--scratch", default=None)
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    bases = sorted(os.path.join(a.dump, f[:-5]) for f in os.listdir(a.dump) if f.endswith(".json"))
    lib = None
    if a.c_ref:
        import tempfile
        lib = c_reference(a.scratch or tempfile.mkdtemp(prefix="ar_dump_check_"))
    if a.png_out:
        os.makedirs(a.png_out, exist_ok=True)
    rows, ok = [], True
    for b in bases:
        r, d = check_one(b, lib)
        if a.png_out:
            p = os.path.join(a.png_out, os.path.basename(b) + ".png")
            write_png(d["fb"], p)
            r["png"] = p
        rows.append(r)
        if r.get("void") is None and not r["equal"]:
            ok = False
        if lib is not None and r.get("void") is None and not (r.get("c_ref_equal_fb") and
                                                              r.get("c_ref_status") == r["status"]):
            ok = False
        print(json.dumps(r, sort_keys=True))
    summary = {"dumps": len(rows), "void": sum(1 for r in rows if r.get("void")),
               "equal": sum(1 for r in rows if r.get("equal")), "all_equal": ok and bool(rows)}
    print(json.dumps(summary))
    if a.json:
        json.dump({"rows": rows, "summary": summary}, open(a.json, "w"), indent=1, sort_keys=True)
    return 0 if summary["all_equal"] else 1


if __name__ == "__main__":
    sys.exit(main())
