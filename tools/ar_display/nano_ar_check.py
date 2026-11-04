"""AR2NANO (nationals 2026-11-02): the car screen moves from the PL AR core to the Jetson Orin Nano.
The Nano renderer is tools/kv_link/ar_helper/m7ar_present --provider cpu (hw/m7/ar_overlay.cpp, unchanged,
through the same HLS AXI shell, run on the CPU). This checker proves, per case, that the picture the
presenter puts in its scan-out buffer equals, byte for byte,
  (a) the independent numpy golden (tools/ar_display/golden.py, written from the spec, not from the C), and
  (b) the C reference (hw/m7/ar_overlay.cpp via tools/ar_display/ar_ref_adapter.cpp, x86 g++),
for four builds: host (x86, Xilinx headers), hostfast (x86, fastsim headers), aarch64 (KV build) and nano
(aarch64, fastsim headers: the binary that goes to the Nano), the aarch64 ones under qemu-aarch64-static.

Cases: every synthetic case of check_golden.py (panel strings, modes, 0 / 32 boxes, descriptor errors,
bad dimensions, minimap clipping, slice labels ...), the 4 multibox_cosim.py cases (11 overlapping boxes,
boxes on all four depth edges, label clipped by the canvas edge, 1-px and 2-px boxes), and car-layout
cases (1024x600 canvas, 640x360 depth at kv_ar.ar_layout, 32 boxes: full descriptor, overlaps, every edge,
empty box 0x0 of the descriptor beyond count).
Positive controls: (1) one box's verdict changed 2 -> 3 (magenta -> grey) in the message only: the
presenter output must differ from the golden; (2) permit_i flipped in the message only: must differ
(proves the cpu provider really takes permit_i from the message).

  PYTHONHASHSEED=0 PYTHONPATH=. python3 tools/ar_display/nano_ar_check.py --scratch <dir> [--build <dir>]
     [--arch host,hostfast,aarch64,nano] [--bench N]
Exit 0 only when every case matches and every positive control differs.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)
import golden  # noqa: E402
import check_golden as cg  # noqa: E402
import multibox_cosim as mb  # noqa: E402
from host import kv_ar  # noqa: E402

R = {n: i for i, n in enumerate(golden.REG_NAMES)}
QEMU = shutil.which("qemu-aarch64-static") or shutil.which("qemu-aarch64")


def car_cases():
    """1024x600 car screen, 640x360 depth (car Gemini 2 depth size), kv_ar.ar_layout."""
    lay = kv_ar.ar_layout(1024, 600, 640, 360)
    Wd, Hd = 640, 360
    rows = []
    # 32 boxes: edges, overlaps, duplicates, thin, single pixel, all three verdicts, label glyph set
    edge = [[0, 0, 639, 359], [0, 0, 40, 30], [600, 0, 639, 25], [0, 330, 50, 359], [590, 320, 639, 359],
            [300, 0, 340, 359], [0, 170, 639, 190], [100, 100, 100, 100], [200, 200, 201, 260],
            [120, 60, 220, 160], [150, 90, 250, 190], [120, 60, 220, 160]]
    labels = ["EDGE", "TL01", "TR02", "BL03", "BR04", "VBAR", "HBAR", "PT07", "TH08", "OV09", "OV10", "DUP1"]
    for i in range(32):
        if i < len(edge):
            x0, y0, x1, y1 = edge[i]
            lb = labels[i]
        else:
            k = i - len(edge)
            x0, y0 = 20 + 29 * k, 40 + 13 * k
            x1, y1 = x0 + 35 + (k % 5) * 7, y0 + 25 + (k % 3) * 9
            lb = "%c%03d" % ("ABCDEFGHIJKLMNOPQRSTUVWXYZ:.?>/-+="[k % 34], i)
        rows.append([x0, y0, x1, y1, (0, 2, 3)[i % 3], cg.lab(lb[:4])])
    depth = cg.big_depth(640, 440)[:Wd * Hd]          # contains 100 (nearest), 0, 10000, 9999
    grid = cg.formula_grid(9)
    base = dict(width=lay["width"], height=lay["height"], depth_width=Wd, depth_height=Hd,
                depth_dx=lay["depth_dx"], depth_dy=lay["depth_dy"], frame_valid=1, pose_valid=1, age_ms=143,
                age_valid=1, max_age_ms=1000, replay=0, object_evidence=0, frame_id=987654,
                permit_present=1, permit_i=1, grid_valid=1, slice_y=7, slice_lo_mm=-350, slice_hi_mm=-300,
                cam_cell_x=16, cam_cell_z=0, mini_x=lay["mini_x"], mini_y=lay["mini_y"])
    out = []
    for name, kw, nbox in (("car_live_32", {}, 32), ("car_live_0", {}, 0), ("car_live_1", {}, 1),
                           ("car_stop_32", dict(permit_i=0), 32), ("car_stale", dict(age_ms=1001), 32),
                           ("car_drv_na", dict(permit_present=0), 5), ("car_grid_off", dict(grid_valid=0), 3)):
        r = dict(base, **kw)
        out.append(dict(name=name, regs=cg.regs_of(r), depth=depth, grid=grid,
                        desc=cg.make_desc(rows[:nbox]), rows=rows[:nbox]))
    return out


def all_cases():
    cs = [dict(c, name="syn_" + c["name"]) for c in cg.synthetic_cases()]
    for c in mb.cases():
        cs.append(dict(c, name="mb_" + c["name"]))
    cs += car_cases()
    return cs


def presenter_regs(regs):
    """The presenter's own register rule (m7ar_present present(), same on the KV): age_basis = -1 keeps
    age_ms; a replay frame with no age or older than max_age_ms is written with replay = 0."""
    r = list(regs)
    if r[R["replay"]] == 1 and (r[R["age_valid"]] == 0 or r[R["age_ms"]] > r[R["max_age_ms"]]):
        r[R["replay"]] = 0
    return r


def canvas_of(regs):
    W, H = regs[0], regs[1]
    if 0 < W <= 1280 and 0 < H <= 800 and W % 4 == 0 and H > 40 and W >= 8:
        return W, H
    if 0 < W <= 1280 and 0 < H <= 800 and W % 4 == 0:
        return max(W, 64), max(H, 48)
    return 640, 480


IMPL = ["--cpu-impl", "fast"]


def stop(p):
    """SIGTERM, then SIGKILL after 20 s: under qemu-user a SIGTERM is now and then not delivered (one hang in
    4 x 325 runs, AR2NANO2 e2e/full.log); the frame event and the dump were read before, so nothing is lost."""
    p.terminate()
    try:
        p.wait(timeout=20)
    except subprocess.TimeoutExpired:
        p.kill()
        p.wait(timeout=20)   # AR2NANO2: --impl fast|ref (the presenter's --cpu-impl)


def run_presenter(binary, arch, case_regs, depth, grid, desc, scratch, tag):
    W, H = canvas_of(case_regs)
    d = os.path.join(scratch, "dump_" + arch, tag)
    if os.path.isdir(d):
        shutil.rmtree(d)
    os.makedirs(d)
    cmd = ([QEMU] if arch in ("aarch64", "nano") else []) + [
        binary, "--provider", "cpu"] + IMPL + ["--display", "none", "--mock-size", "%dx%d" % (W, H),
        "--stale-ms", "100000000", "--refresh-ms", "100000000", "--dump-dir", d, "--dump-seq", "2"]
    msg = kv_ar.encode_ar_message({"regs": case_regs, "depth": np.asarray(depth, np.uint16),
                                   "grid": np.asarray(grid, np.uint8), "desc": np.asarray(desc, np.uint32),
                                   "frame_id": case_regs[R["frame_id"]], "first_mono_ns": 0,
                                   "age_basis_ns": -1})
    # the presenter keeps showing (fail-safe) after stdin EOF until SIGTERM, by design: read its events
    # until the one frame event, then terminate it
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    p.stdin.write(msg)
    p.stdin.flush()
    frames = []
    while not frames:
        line = p.stdout.readline()
        if not line:
            break
        e = json.loads(line) if line.startswith(b"{") else {}
        if e.get("ev") == "frame":
            frames.append(e)
        elif e.get("ev") in ("error", "protocol_error"):
            break
    p.stdin.close()
    stop(p)
    if len(frames) != 1:
        raise RuntimeError("%s %s: no frame event (rc %s)" % (arch, tag, p.returncode))
    ev = frames[0]
    fbp = os.path.join(d, "ar_000002.fb")
    fb = np.fromfile(fbp, "<u4") if os.path.exists(fbp) else None
    return ev, fb, (W, H)


def check_one(binary, arch, case, lib, scratch, tag, regs=None, desc=None, want_equal=True):
    regs_msg = list(case["regs"]) if regs is None else regs
    desc_msg = case["desc"] if desc is None else desc
    eff = presenter_regs(case["regs"])            # what the core must see (golden input)
    st_g, fb_g, _ = golden.render(case["depth"], case["grid"], case["desc"], eff)
    st_c, fb_c = cg.run_c(lib, dict(case, regs=eff))
    ev, fb, (W, H) = run_presenter(binary, arch, presenter_regs(regs_msg), case["depth"], case["grid"],
                                   desc_msg, scratch, tag)
    w, h = eff[0], eff[1]
    res = {"case": tag, "arch": arch, "status_golden": st_g, "status_c": st_c, "status_nano": ev["status"],
           "mode_name": ev.get("mode_name"), "ar_ms": ev["ar_ns"] / 1e6}
    if st_g == 1:
        res["equal"] = ev["status"] == 1 and st_c == 1 and fb is None and ev["flipped"] == 0
        res["kind"] = "status1_no_picture"
    else:
        img = fb[:w * h].reshape(h, w)
        rest = fb[w * h:]
        c_img = fb_c[:w * h].reshape(h, w)
        res["ndiff_golden"] = int((img != fb_g).sum())
        res["ndiff_c"] = int((img != c_img).sum())
        res["rest_zero"] = bool(np.all(rest == 0))
        res["equal"] = (ev["status"] == st_g == st_c and res["ndiff_golden"] == 0 and res["ndiff_c"] == 0
                        and res["rest_zero"])
        res["sha256_nano"] = __import__("hashlib").sha256(img.astype("<u4").tobytes()).hexdigest()[:16]
        res["sha256_golden"] = __import__("hashlib").sha256(fb_g.astype("<u4").tobytes()).hexdigest()[:16]
    res["pass"] = res["equal"] == want_equal
    return res


def bench(binary, arch, case, scratch, n):
    """n frames through one presenter process (frames paced so none is superseded)."""
    W, H = canvas_of(case["regs"])
    cmd = ([QEMU] if arch in ("aarch64", "nano") else []) + [
        binary, "--provider", "cpu"] + IMPL + ["--display", "none", "--mock-size", "%dx%d" % (W, H),
        "--stale-ms", "100000000", "--refresh-ms", "100000000"]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    ready = None
    while ready is None:
        l = p.stdout.readline()
        if not l:
            raise RuntimeError("bench: presenter died")
        e = json.loads(l)
        if e.get("ev") == "ready":
            ready = e
    ar, prep = [], []
    for i in range(n):
        regs = list(case["regs"])
        regs[R["frame_id"]] = 1000 + i
        msg = kv_ar.encode_ar_message({"regs": regs, "depth": case["depth"], "grid": case["grid"],
                                       "desc": case["desc"], "frame_id": 1000 + i, "first_mono_ns": 0,
                                       "age_basis_ns": -1})
        p.stdin.write(msg)
        p.stdin.flush()
        while True:
            e = json.loads(p.stdout.readline())
            if e.get("ev") == "frame":
                ar.append(e["ar_ns"] / 1e6)
                prep.append(e["prep_ns"] / 1e6)
                break
    p.stdin.close()
    stop(p)
    a = np.array(ar)
    return {"arch": arch, "case": case["name"], "n": n, "canvas": [W, H],
            "core_ms_median": float(np.median(a)), "core_ms_p90": float(np.percentile(a, 90)),
            "core_ms_max": float(a.max()), "prep_ms_median": float(np.median(prep))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scratch", required=True)
    ap.add_argument("--build", default=None, help="m7ar_present build dir (make BUILD=...); default <scratch>/build")
    ap.add_argument("--arch", default="host,hostfast,aarch64,nano")
    ap.add_argument("--bench", type=int, default=0)
    ap.add_argument("--only", default=None, help="comma list of case names (debug)")
    ap.add_argument("--impl", default="fast", choices=["fast", "ref"], help="presenter --cpu-impl (AR2NANO2)")
    ap.add_argument("--random", type=int, default=0, help="+N seeded random cases (fast_ar_check.random_case)")
    ap.add_argument("--seed", type=int, default=20261102)
    a = ap.parse_args()
    IMPL[1] = a.impl
    os.makedirs(a.scratch, exist_ok=True)
    build = a.build or os.path.join(a.scratch, "build")
    archs = a.arch.split(",")
    subprocess.check_call(["make", "-s", "-C", os.path.join(ROOT, "tools/kv_link/ar_helper"), "BUILD=" + build]
                          + archs)
    lib = cg.build(a.scratch, "clean")
    cases = all_cases()
    if a.random:
        import fast_ar_check as fac
        rng = np.random.default_rng(a.seed)
        cases += [fac.random_case(rng, k) for k in range(a.random)]
    if a.only:
        keep = set(a.only.split(","))
        cases = [c for c in cases if c["name"] in keep]
    results = []
    for arch in archs:
        if arch in ("aarch64", "nano") and not QEMU:
            raise SystemExit("qemu-aarch64-static not found")
        binary = os.path.join(build, arch, "m7ar_present")
        for c in cases:
            results.append(check_one(binary, arch, c, lib, a.scratch, c["name"]))
        # positive controls on the full car case
        car = [c for c in cases if c["name"] == "car_live_32"]
        if car:
            c = car[0]
            desc = np.array(c["desc"], np.uint32).copy()
            k = next(i for i, r in enumerate(c["rows"]) if r[4] == 2)
            desc[6 + 6 * k] = 3                          # verdict 2 -> 3: magenta -> grey, label F -> O
            r1 = check_one(binary, arch, c, lib, a.scratch, "pc_box_colour", desc=desc, want_equal=False)
            r1["control"] = "box %d verdict 2->3 in the message only" % k
            regs = list(c["regs"])
            regs[R["permit_i"]] ^= 1
            r2 = check_one(binary, arch, c, lib, a.scratch, "pc_permit", regs=regs, want_equal=False)
            r2["control"] = "permit_i flipped in the message only"
            results += [r1, r2]
    benches = []
    if a.bench:
        car = [c for c in all_cases() if c["name"] == "car_live_32"][0]
        for arch in archs:
            benches.append(bench(os.path.join(build, arch, "m7ar_present"), arch, car, a.scratch,
                                 a.bench if arch in ("host", "hostfast") else max(3, a.bench // 10)))
    ok = all(r["pass"] for r in results)
    summary = {"pass": ok, "impl": a.impl, "random": a.random, "seed": a.seed, "cases": len(results),
               "failed": [r["case"] + "/" + r["arch"] for r in results if not r["pass"]],
               "controls": [r for r in results if r["case"].startswith("pc_")],
               "bench": benches, "qemu": QEMU}
    with open(os.path.join(a.scratch, "nano_ar_check.json"), "w") as fp:
        json.dump({"summary": summary, "results": results}, fp, indent=1)
    print(json.dumps(summary, indent=1))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
