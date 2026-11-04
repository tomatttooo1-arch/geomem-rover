#!/usr/bin/env python3
"""SRCHCHAIN0 (2026-11-04): probabilistic search ("the object was moved: where to look next") in the single-board
product chain, switch SRCH=pl|nano.

What it computes (the product's own search, host/search_belief.BeliefSearch, 10-26): the object's last known
position gives the prior, the views already taken (the mapping trajectory) are observed, then the planner chooses
the next look (node + yaw) again and again as if the object is never seen -> the search ORDER (BeliefSearch.preview,
12 looks). The bench has no moving car and no mapping run, so the map, the trajectory and the last known position
come from a fixed scene file (scene.npz, made on the server by ``srch_chain.py scene``; default the real mapping run
20261016_160423, 25 nodes, the map the core's co-simulation vectors V1 come from).

Implementations (same episode code, only the belief/visibility arithmetic differs):
  pl      KV: tools/srch/srch_ps.FX (product BeliefSearch subclass; visibility, views, observe, prior, gains on the
          search core) + the PL core m7_srch @ 0xA00D0000 through libsrch_drv.so (hw/srch/drv, plio access layer,
          host cross build). next_goal / scoring / stop rule stay the product's Python.
  mock    server: same as pl, libsrch_drv.so host build = the same driver protocol over a MockBus that runs the
          fixed-point C model (hw/srch/srch_core.cpp -DSRCH_SW) on ap_start. Protocol check, not board evidence.
  cmodel  server: FX + tools/srch/srch_ps.Core (C model called directly, no driver).
  sw      the product BeliefSearch in pure Python/numpy (the old path; Nano in SRCH=nano, KV fallback in SRCH=pl).

Service (``srch_chain.py serve``, KV in SRCH=pl): runs one episode every --period s, cycling the scene's targets,
appends one JSON line per episode to --out and rewrites --status (kv_uplink.py forwards the newest to the Nano).
FPGA rule: nothing refuses. No m7_srch node in the loaded bitstream, a driver/open error, a core timeout/error, or a
self-check mismatch against the software path (run once, first episode) -> WARN, the KV stops searching and reports
impl "fallback_nano": the Nano console then runs the software search itself (the old path).

Usage:
  srch_chain.py scene  --out scene.npz [--yaml X.yaml]          (server; needs tools/car_run)
  srch_chain.py run    --scene scene.npz --impl sw|cmodel|mock [--drv libsrch_drv.so] [--targets 0,1,2] --json out
  srch_chain.py compare --scene scene.npz --drv build/host/libsrch_drv.so --out cmp.json   (key check: mock vs sw)
  srch_chain.py serve  --scene scene.npz --impl pl --drv xbin/libsrch_drv.so --out srch.jsonl --status srch_status.json
"""
import argparse
import ctypes
import json
import math
import os
import sys
import time

WT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path[:0] = [WT, os.path.join(WT, "tools", "srch")]
import numpy as np  # noqa: E402

DEF_YAML = ("${FPGA_SRC}/runs/real-car-integ-20261015-results/gate_dig/host_g6/map_tum/"
            "export/20261016_160423.yaml")   # 25 nodes; the core co-simulation vectors V1 come from this map
# camera as in tools/srch/srch_harness.py (SRCHK1 key check): Gemini 2 640x480, base_camera((0.10, 0, 0.39), 16 deg)
K = np.array([[517.17, 0, 320.0], [0, 517.17, 240.65], [0, 0, 1.]])
IMG = (640, 480)
MAX_GOALS = 12
SRCH_BASE = 0xA00D0000


class _Map(object):
    def __init__(self, cells, resolution, origin_x, origin_y, origin_yaw):
        self.cells, self.resolution = cells, resolution
        self.origin_x, self.origin_y, self.origin_yaw = origin_x, origin_y, origin_yaw


def make_scene(yaml_path, out):
    sys.path.insert(0, os.path.join(WT, "tools", "car_run"))
    import search_preview as sp
    import mapping_ctl
    from tools.car_geo_producer.common import base_camera
    from host.search_belief import BeliefSearch, SearchConfig
    fm = sp.load_map(yaml_path)
    poses = yaml_path[:-5] + "_poses.txt"
    nodes = np.array(sp.poses_to_nodes(poses), float)
    rows = sorted(mapping_ctl.poses_nodes(poses))
    frames = np.array([(float(t), float(x), float(y), float(yaw)) for t, x, y, yaw in rows], float)
    cfg_text = open(os.path.join(WT, "tools/car_run/search_config.json")).read()
    P = BeliefSearch(fm, [tuple(n) for n in nodes], SearchConfig.from_doc(json.loads(cfg_text)))
    U = P.ux.size
    targets = np.array([(np.nan, np.nan), (P.ux[U // 3], P.uy[U // 3]), (P.ux[2 * U // 3], P.uy[2 * U // 3])], float)
    Tbc = np.asarray(base_camera((0.10, 0.0, 0.39), 16.0), float)
    np.savez_compressed(out, cells=np.asarray(fm.cells, np.int8), resolution=fm.resolution, origin=np.array(
        [fm.origin_x, fm.origin_y, fm.origin_yaw]), nodes=nodes, frames=frames, targets=targets, Tbc=Tbc,
        K=K, img=np.array(IMG), config=np.array(cfg_text), source=np.array(os.path.basename(yaml_path)))
    print(json.dumps({"scene": out, "source": yaml_path, "nodes": len(nodes), "frames": len(frames), "U": int(U),
                      "targets": targets.tolist()}))


class Scene(object):
    def __init__(self, path):
        z = np.load(path, allow_pickle=False)
        o = z["origin"]
        self.map = _Map(z["cells"], float(z["resolution"]), float(o[0]), float(o[1]), float(o[2]))
        self.nodes = [tuple(map(float, n)) for n in z["nodes"]]
        self.frames = [(float(r[0]), (float(r[1]), float(r[2]), float(r[3]))) for r in z["frames"]]
        self.targets = [None if not np.isfinite(t).all() else (float(t[0]), float(t[1])) for t in z["targets"]]
        self.Tbc, self.K, self.img = z["Tbc"], z["K"], tuple(int(v) for v in z["img"])
        self.config_doc = json.loads(str(z["config"]))
        self.source = str(z["source"])


class DrvCore(object):
    """srch_ps.Core-compatible core over libsrch_drv.so (board: /dev/mem + DMA heap; host: MockBus + C model).
    FX writes inputs into self.ddr and relative offsets into the parameter block; the address fields get pa here."""

    def __init__(self, lib_path, mock, timeout_s=2.0, provider="auto", wait="poll"):
        import srch_ps as S
        self.S = S
        lib = self.lib = ctypes.CDLL(lib_path)
        lib.srch_drv_open.restype = ctypes.c_void_p
        lib.srch_drv_open.argtypes = [ctypes.c_int, ctypes.c_uint64, ctypes.c_char_p, ctypes.c_char_p,
                                      ctypes.c_uint64, ctypes.c_char_p, ctypes.c_int]
        for n in ("srch_drv_va",):
            getattr(lib, n).restype, getattr(lib, n).argtypes = ctypes.c_void_p, [ctypes.c_void_p]
        for n in ("srch_drv_pa", "srch_drv_size", "srch_drv_calls", "srch_drv_fails", "srch_drv_ns_sum"):
            getattr(lib, n).restype, getattr(lib, n).argtypes = ctypes.c_uint64, [ctypes.c_void_p]
        lib.srch_drv_describe.restype, lib.srch_drv_describe.argtypes = ctypes.c_char_p, [ctypes.c_void_p]
        lib.srch_drv_run.restype = ctypes.c_int
        lib.srch_drv_run.argtypes = [ctypes.c_void_p, ctypes.c_uint64, ctypes.c_int, ctypes.POINTER(ctypes.c_uint32),
                                     ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_uint64)]
        lib.srch_drv_close.argtypes = [ctypes.c_void_p]
        why = ctypes.create_string_buffer(512)
        h = lib.srch_drv_open(int(bool(mock)), SRCH_BASE, wait.encode(), provider.encode(), S.DDR_BYTES, why, 512)
        if not h:
            raise RuntimeError("srch_drv_open: " + why.value.decode(errors="replace"))
        self.h = h
        self.phys = int(lib.srch_drv_pa(h))
        size = int(lib.srch_drv_size(h))
        assert size >= S.DDR_BYTES and self.phys % 16 == 0 and self.phys + S.DDR_BYTES <= 1 << 31
        va = lib.srch_drv_va(h)
        self.ddr = np.ctypeslib.as_array((ctypes.c_uint8 * S.DDR_BYTES).from_address(va))
        self.timeout_ms = int(timeout_s * 1000)
        self.trace, self.calls = None, []
        self.describe = lib.srch_drv_describe(h).decode(errors="replace")

    def u64(self, addr, n):
        return self.ddr[addr:addr + 8 * n].view(np.uint64)

    def run(self, op, prm_u64):
        S = self.S
        p = np.zeros(38, np.uint64)
        prm_u64 = dict(prm_u64)
        nseg = int(prm_u64.pop("nseg", 0))
        for k, v in prm_u64.items():
            v = int(v)
            if k in S.ADDR_FIELDS:
                v += self.phys
            p[k] = np.uint64(v & 0xFFFFFFFFFFFFFFFF)
        p[0] = np.uint64(S.MAGIC | (S.VER << 16) | (op << 24) | (nseg << 32))
        self.ddr[S.A_PRM:S.A_PRM + 38 * 8] = p.view(np.uint8)
        st, vld, ns = ctypes.c_uint32(0), ctypes.c_int(0), ctypes.c_uint64(0)
        rc = self.lib.srch_drv_run(self.h, self.phys + S.A_PRM, self.timeout_ms, ctypes.byref(st), ctypes.byref(vld),
                                   ctypes.byref(ns))
        if rc == -1:
            raise RuntimeError("core op %d: busy" % op)
        if rc == -2:
            raise RuntimeError("core op %d: timeout" % op)
        s = st.value if vld.value else int(self.u64(S.A_RES, 1)[0]) & 0xFFFFFFFF
        if s & 0xFF:
            raise RuntimeError("core op %d error 0x%x" % (op, s & 0xFF))
        return self.u64(S.A_RES, 96 + 1280)

    def counts(self):
        return {"core_calls": int(self.lib.srch_drv_calls(self.h)), "core_fails": int(self.lib.srch_drv_fails(self.h)),
                "core_ms": round(self.lib.srch_drv_ns_sum(self.h) / 1e6, 3)}

    def close(self):
        if self.h:
            self.lib.srch_drv_close(self.h)
            self.h = None


class CountCore(object):
    """srch_ps.Core (C model, no driver) with the same counts()."""

    def __init__(self, lib_path):
        import srch_ps as S
        self.c = S.Core(lib_path)
        self.n, self.ns = 0, 0
        self.describe = "cmodel " + lib_path

    def __getattr__(self, k):
        return getattr(self.c, k)

    def run(self, op, prm):
        t = time.monotonic_ns()
        try:
            return self.c.run(op, prm)
        finally:
            self.n += 1
            self.ns += time.monotonic_ns() - t

    def counts(self):
        return {"core_calls": self.n, "core_fails": 0, "core_ms": round(self.ns / 1e6, 3)}


def build_planner(scene, impl, drv=None, provider="auto"):
    """(planner, core or None). impl: sw | cmodel | mock | pl."""
    from host.search_belief import BeliefSearch, SearchConfig
    cfg = SearchConfig.from_doc(scene.config_doc)
    if impl == "sw":
        P = BeliefSearch(scene.map, scene.nodes, cfg)
        P.set_camera(scene.K, scene.img, scene.Tbc)
        return P, None
    import srch_ps as S
    if impl == "cmodel":
        core = CountCore(drv)
    else:
        core = DrvCore(drv, mock=(impl == "mock"), provider=provider)
    F = S.make_fx(scene.map, scene.nodes, cfg, scene.K, scene.img, scene.Tbc, core)
    F.path_lengths = BeliefSearch.path_lengths.__get__(F)    # product path lengths (as SRCHK1 criteria 3/4)
    F.trav = F.trav_c
    return F, core


def episode(bs, scene, target_i):
    """One search episode: object last seen at scene.targets[target_i] (None = no prior position), the mapping
    trajectory's views observed, then the product's look order (12 looks max). Returns a JSON-able dict."""
    target = scene.targets[target_i % len(scene.targets)]
    t0 = time.monotonic()
    bs.reset()
    bs.begin(target)
    for t, bp in scene.frames:
        bs.observe(bs.camera_pose(*bp), t, base_pose=bp)
    ref = target or bs.nodes[0]
    n0 = min(bs.nodes, key=lambda p: (p[0] - ref[0]) ** 2 + (p[1] - ref[1]) ** 2)
    order = bs.preview(robot=(n0[0], n0[1], 0.), max_goals=MAX_GOALS)
    looks = [[g["node"], g["x"], g["y"], g["yaw"], g["cum_prob"]] for g in order if "x" in g]
    stop = [g.get("stop") for g in order if "stop" in g]
    return {"target_i": target_i % len(scene.targets), "target": target, "looks": looks,
            "stop": stop[0] if stop else None, "first": looks[0][:4] if looks else None,
            "mass_left": round(float(bs.mass_left()), 6), "ms": round((time.monotonic() - t0) * 1e3, 1)}


def same_order(a, b):
    """Same look sequence: node and yaw (rounded as the product reports them) and the stop reason."""
    return [(l[0], l[3]) for l in a["looks"]] == [(l[0], l[3]) for l in b["looks"]] and a["stop"] == b["stop"]


def cmd_run(a):
    sc = Scene(a.scene)
    bs, core = build_planner(sc, a.impl, a.drv)
    out = []
    for ti in [int(v) for v in a.targets.split(",")]:
        r = episode(bs, sc, ti)
        if core is not None:
            r.update(core.counts())
        out.append(r)
        print(json.dumps(dict({k: r[k] for k in ("target_i", "first", "stop", "ms")}, calls=r.get("core_calls"))))
    if a.json:
        json.dump({"impl": a.impl, "scene": sc.source, "episodes": out}, open(a.json, "w"), indent=1)


def cmd_compare(a):
    """Key check (server): the pl path through the driver (mock = C model behind the same register/buffer protocol)
    against the old software path, same scene, every target; plus a positive control (a deliberately wrong prior
    must change the order, otherwise the check cannot tell)."""
    sc = Scene(a.scene)
    P, _ = build_planner(sc, "sw")
    F, core = build_planner(sc, a.impl, a.drv)
    res = {"scene": sc.source, "impl": a.impl, "driver": core.describe, "runs": []}
    allsame = True
    for ti in range(len(sc.targets)):
        rp, rf = episode(P, sc, ti), episode(F, sc, ti)
        same = same_order(rp, rf)
        allsame &= same
        res["runs"].append({"target_i": ti, "same": same, "sw": rp, a.impl: rf})
        print(json.dumps({"target_i": ti, "same": same, "n_looks": len(rp["looks"]), "sw_first": rp["first"],
                          "pl_first": rf["first"], "stop": [rp["stop"], rf["stop"]], "ms": [rp["ms"], rf["ms"]],
                          "mass": [rp["mass_left"], rf["mass_left"]]}))
    # positive control: sw episode with the target moved to another target's place must differ from the pl order
    ti, tj = 1, 2
    sc2 = Scene(a.scene)
    sc2.targets = list(sc.targets)
    sc2.targets[ti] = sc.targets[tj]
    rneg = episode(P, sc2, ti)
    rf1 = [r for r in res["runs"] if r["target_i"] == ti][0][a.impl]
    res["positive_control"] = {"what": "sw with target %d placed at target %d's position vs %s target %d" % (
        ti, tj, a.impl, ti), "differs": not same_order(rneg, rf1)}
    res["core"] = core.counts()
    res["verdict"] = {"all_same": bool(allsame), "control_ok": res["positive_control"]["differs"]}
    print(json.dumps({"verdict": res["verdict"], "positive_control": res["positive_control"], "core": res["core"]}))
    json.dump(res, open(a.out, "w"), indent=1)
    return 0 if allsame and res["positive_control"]["differs"] else 1


def has_srch_node():
    """The loaded overlay has the search core (its device-tree node m7_srch bound to generic-uio). Checked without
    any PL bus access: on a bitstream without the core (m5) the 0xA00D0000 window is unmapped and a read could hang."""
    import glob
    for p in glob.glob("/sys/class/uio/uio*/name"):
        try:
            if open(p).read().strip() == "m7_srch":
                return True
        except OSError:
            pass
    return False


def cmd_serve(a):
    """KV service (SRCH=pl). Fallback = the old path: on any failure the KV stops searching and reports
    impl "fallback_nano" in its status; the Nano console (--srch pl) then runs the software search itself."""
    sc = Scene(a.scene)
    outf = open(a.out, "a")
    warns = []
    state = {"impl": a.impl, "reason": None}

    def log(rec):
        outf.write(json.dumps(rec, separators=(",", ":")) + "\n")
        outf.flush()

    def warn(msg):
        warns.append(msg)
        print("WARN: " + msg, file=sys.stderr, flush=True)
        log({"ev": "warn", "t_mono_ns": time.monotonic_ns(), "msg": msg})

    def fall(msg):
        warn(msg + ": search falls back to the Nano (old path)")
        state.update(impl="fallback_nano", reason=msg)

    bs = core = None
    t_init = time.monotonic()
    if a.impl == "pl" and not a.force_node and not has_srch_node():
        fall("no m7_srch node in the loaded bitstream (no search core)")
    elif a.impl != "sw":
        try:
            bs, core = build_planner(sc, a.impl, a.drv, provider=a.provider)
        except Exception as exc:          # noqa: BLE001  (FPGA rule: never refuse, fall back)
            fall("search core init failed (%r)" % (exc,))
    else:
        bs, _ = build_planner(sc, "sw")
    init_ms = round((time.monotonic() - t_init) * 1e3, 1)
    log({"ev": "init", "t_mono_ns": time.monotonic_ns(), "impl_req": a.impl, "impl": state["impl"],
         "scene": sc.source, "init_ms": init_ms, "driver": getattr(core, "describe", None),
         "counts": core.counts() if core is not None else None})
    print(json.dumps({"srch": "init", "impl": state["impl"], "init_ms": init_ms}), flush=True)
    n, nxt, t0 = 0, time.monotonic(), time.monotonic()
    selfcheck = {"done": False}
    last = None
    while True:
        now = time.monotonic()
        if now < nxt:
            time.sleep(min(0.2, nxt - now))
            continue
        nxt = now + a.period
        if state["impl"] != "fallback_nano":
            ti = n % len(sc.targets)
            rec = {"ev": "episode", "t_mono_ns": time.monotonic_ns(), "n": n + 1, "impl": state["impl"]}
            try:
                rec.update(episode(bs, sc, ti))
                if core is not None:
                    rec.update(core.counts())
            except Exception as exc:      # noqa: BLE001
                rec = None
                fall("episode %d failed (%r)" % (n + 1, exc))
            if rec is not None and core is not None and a.selfcheck and not selfcheck["done"]:
                # once: the first core episode against the software path on the KV (same scene, same target)
                try:
                    sw, _ = build_planner(sc, "sw")
                    rs = episode(sw, sc, ti)
                    ok = same_order(rs, rec)
                    selfcheck.update(done=True, same=ok, sw_ms=rs["ms"], pl_ms=rec["ms"], target_i=ti,
                                     sw_first=rs["first"], pl_first=rec["first"])
                    del sw
                except Exception as exc:  # noqa: BLE001
                    ok = True
                    selfcheck.update(done=True, same=None, error=repr(exc))
                rec["selfcheck"] = dict(selfcheck)
                if not ok:
                    fall("self-check: core look order differs from the software order (target %d)" % ti)
            if rec is not None:
                log(rec)
                last = rec
                n += 1
        st = {"t_s": round(now - t0, 1), "impl_req": a.impl, "impl": state["impl"], "reason": state["reason"],
              "episodes": n, "last": last, "selfcheck": selfcheck, "warnings": warns[-5:], "init_ms": init_ms,
              "counts": core.counts() if core is not None else None, "period_s": a.period}
        tmp = a.status + ".tmp"
        with open(tmp, "w") as fp:
            json.dump(st, fp)
        os.replace(tmp, a.status)
        if a.seconds and now - t0 > a.seconds:
            break


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("scene")
    p.add_argument("--yaml", default=DEF_YAML)
    p.add_argument("--out", required=True)
    p = sub.add_parser("run")
    p.add_argument("--scene", required=True)
    p.add_argument("--impl", default="sw", choices=("sw", "cmodel", "mock", "pl"))
    p.add_argument("--drv", default=None)
    p.add_argument("--targets", default="0,1,2")
    p.add_argument("--json", default=None)
    p = sub.add_parser("compare")
    p.add_argument("--scene", required=True)
    p.add_argument("--impl", default="mock", choices=("cmodel", "mock", "pl"))
    p.add_argument("--drv", required=True)
    p.add_argument("--out", required=True)
    p = sub.add_parser("serve")
    p.add_argument("--scene", required=True)
    p.add_argument("--impl", default="pl", choices=("sw", "cmodel", "mock", "pl"))
    p.add_argument("--drv", default=None)
    p.add_argument("--provider", default="auto")
    p.add_argument("--period", type=float, default=10.0)
    p.add_argument("--selfcheck", type=int, default=1)
    p.add_argument("--force-node", action="store_true", help="skip the m7_srch node check (host mock runs)")
    p.add_argument("--out", required=True)
    p.add_argument("--status", required=True)
    p.add_argument("--seconds", type=float, default=0.0)
    a = ap.parse_args(argv)
    if a.cmd == "scene":
        make_scene(a.yaml, a.out)
    elif a.cmd == "run":
        cmd_run(a)
    elif a.cmd == "compare":
        sys.exit(cmd_compare(a))
    else:
        cmd_serve(a)


if __name__ == "__main__":
    main()
