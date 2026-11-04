#!/usr/bin/env python3
"""MEMCHAIN0 (2026-11-04): KV260 object-memory service -- the object memory on the KV, numbers from the PL memory core.

runs/nationals-prelim-20261102/MEMCHAIN0/DESIGN.md. One process on the KV keeps the object memory
(perception/fpga_replay/memory_pl.PLMemory: association, update, dedupe, query; every number from the core) and writes
its state and the find answers to a small JSON file (--out, atomic) that kv_uplink.py sends to the Nano.

Backends (--backend):
  pl      the core on the board: MMIOBackend(DevMemWindow(0xA00E0000), "bram") (slot 14, MERGE10 driver)
  plsim   the same driver on a memory-simulated slot (host check; tools/memk0/mmio_sim_check.SimWindow, --sim-lib)
  cmodel  the core's C model on the CPU (memory_pl.CModel; on the board MEMK0_LIB_DIR -> host cross-compiled .so)
  py      the plain Python core.Memory (no core at all)
Failure policy (FPGA rule: warn, never refuse): if pl / plsim cannot start or a core call fails, print a WARN line,
count it, switch to cmodel and replay every frame seen so far into the new memory (frames are light: pose +
observations), so the memory continues where it was; cmodel failing -> py. Never switches back.

Feeds (--feed):
  replay:<stream.json>  a recorded observation stream (tools/memory_quality/replay_stream.load_stream format, e.g.
                        tests/data/memq_realcar_20261016_compact.json), paced by --speed; --emit-boxes writes the
                        same stream as dpu_rx-style box rows so the old chain (Nano counts boxes) sees the same input
  ring                  bench live feed: dpu_rx --out-log rows (--boxes) + the newest kv_cam depth frame; median
                        depth of the box's central part, back-projected with --cam-params K, fixed camera pose. Box
                        rows arrive 1-3 s late (dpu_rx stdio buffer), so box and depth are not the same frame:
                        valid only on a static bench with a static scene.
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from perception.fpga_replay import core, memory_gates, memory_pl  # noqa: E402


def warn(msg, **kw):
    print(json.dumps(dict({"WARN": msg}, **kw)), file=sys.stderr, flush=True)


class SimWindow:
    """Memory-simulated slot 14 (as tools/memk0/mmio_sim_check.SimWindow; own lib copy so its C state is private)."""

    def __init__(self, lib_path, layout="bram"):
        from tools.memk0 import mmio_sim_check as msc
        self._w = msc.SimWindow(msc.load_copy(lib_path, "memsvc"), layout)

    def __getattr__(self, k):
        return getattr(self._w, k)


def make_memory(kind, sim_lib=None, timeout_s=1.0):
    if kind == "py":
        return core.Memory("geometry", core.Config())
    if kind == "cmodel":
        return memory_pl.PLMemory("geometry", core.Config())
    if kind == "pl":
        be = memory_pl.MMIOBackend(memory_pl.DevMemWindow(), "bram", timeout_s=timeout_s)
    elif kind == "plsim":
        be = memory_pl.MMIOBackend(SimWindow(sim_lib, "bram"), "bram", timeout_s=timeout_s)
    else:
        raise ValueError(kind)
    return memory_pl.PLMemory("geometry", core.Config(), backend=be)   # __init__ does the INIT call


class MemService:
    CHAIN = {"pl": "cmodel", "plsim": "cmodel", "cmodel": "py", "py": None}

    def __init__(self, backend, sim_lib=None, fail_after=None):
        self.kind, self.sim_lib = backend, sim_lib
        self.fallbacks, self.warnings, self.log = 0, [], []      # log: frames for a replay after a fallback
        self.fail_after = fail_after                                # negative control: fake a core failure
        self.frames = 0
        self.mem = None
        self.dead_calls = {}
        while self.mem is None:
            try:
                self.mem = make_memory(self.kind, sim_lib)
            except Exception as exc:                                # noqa: BLE001 (warn, never refuse)
                self._fall("start %s: %r" % (self.kind, exc))

    def _fall(self, why):
        nxt = self.CHAIN[self.kind]
        self.fallbacks += 1
        self.warnings.append({"t": round(time.time(), 1), "from": self.kind, "to": nxt, "why": why[:200]})
        warn("memory backend failed, falling back", frm=self.kind, to=nxt, why=why[:200])
        if nxt is None:
            raise SystemExit("no memory backend left")
        if self.mem is not None and hasattr(self.mem, "calls"):
            self.dead_calls = dict(self.mem.calls)
        self.kind, self.mem = nxt, None

    def _apply(self, mem, rec):
        t, T_wc, obs = rec
        fid = "f%06d" % rec_index(rec)
        frame = core.Frame(fid, t, _RGB, _DEPTH, rec_K(rec), T_wc, True, "kv_memsvc", "kv")
        frame.ground_z_m = 0.0
        observations = [core.Observation(o["label"], o["score"], o["p"], o["p"] - .05, o["p"] + .05, o["color"], 1.0,
                                         fid, t, tuple(o["bbox"]), "kv", extent_m=o["extent"],
                                         truncated=o["trunc"], range_m=o["range"]) for o in obs]
        mem.update_memory(observations, {}, frame)

    def update(self, rec):
        self.log.append(rec)
        self.frames += 1
        while True:
            try:
                if self.fail_after is not None and self.kind in ("pl", "plsim") and self.frames > self.fail_after:
                    raise memory_pl.Unsupported("negative control: injected core failure")
                if self.mem is None:
                    self.mem = make_memory(self.kind, self.sim_lib)
                    for r in self.log[:-1]:
                        self._apply(self.mem, r)
                self._apply(self.mem, rec)
                return
            except SystemExit:
                raise
            except Exception as exc:                                # noqa: BLE001
                self._fall("update frame %d: %r" % (self.frames, exc))

    def answer(self, targets):
        out = {}
        for g in targets:
            try:
                res = self.mem.query_memory(g)
            except Exception as exc:                                # noqa: BLE001
                self._fall("query: %r" % exc)
                self.mem = make_memory(self.kind, self.sim_lib)
                for r in self.log:
                    self._apply(self.mem, r)
                res = self.mem.query_memory(g)
            items = [{"id": r.get("id"), "label": r.get("label"), "state": r.get("state"),
                      "pos": [round(float(v), 3) for v in r.get("position_m", [])],
                      "last_seen": round(float(r.get("last_seen", 0.0)), 2)} for r in res]
            # found: the memory holds a usable position for the target (seen now or remembered); found_now: seen
            # in the current view (current_confirmed). position_invalid tracks are not an answer.
            out[g] = {"found": any(i["state"] != "position_invalid" for i in items),
                      "found_now": any(i["state"] == "current_confirmed" for i in items), "n": len(items),
                      "items": items[:4]}
        return out

    def state(self, targets):
        find = self.answer(targets)
        snap = self.mem.snapshot()
        objs = [{"id": t["id"], "label": t["label"], "state": t["state"],
                 "pos": [round(float(v), 3) for v in t["position_m"]], "last_seen": round(float(t["last_seen"]), 2)}
                for t in snap]
        return {"src": self.kind, "calls": dict(getattr(self.mem, "calls", {})), "calls_before_fallback":
                self.dead_calls, "fallbacks": self.fallbacks, "warnings": self.warnings[-3:], "frames": self.frames,
                "n_objects": len(objs), "objects": objs[:16], "find": find}


_RGB, _DEPTH = np.zeros((2, 2, 3), np.uint8), np.full((2, 2), np.nan, np.float32)
_K = {}


def rec_index(rec):
    return rec.i


def rec_K(rec):
    return _K["K"]


class Rec(tuple):
    pass


def make_rec(i, t, T_wc, obs, K, image):
    w, h = image
    out = []
    for o in obs:
        p = np.array(o["position_m"], dtype=float)
        z = float(((p - T_wc[:3, 3]) @ T_wc[:3, :3])[2])
        out.append({"label": o["label"], "score": float(o["score"]), "p": p, "color": o.get("color", "unknown"),
                    "bbox": o["bbox"], "extent": memory_gates.apparent_extent(o["bbox"], z, K),
                    "trunc": memory_gates.is_truncated(o["bbox"], (h, w), core.Config().edge_margin_px),
                    "range": z})
    r = Rec((t, T_wc, out))
    r.i = i
    return r


def base_matrix(b):
    T = np.eye(4)
    c, s = math.cos(b[2]), math.sin(b[2])
    T[:2, :2] = [[c, -s], [s, c]]
    T[0, 3], T[1, 3] = b[0], b[1]
    return T


def write_json(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as fp:
        json.dump(obj, fp, separators=(",", ":"))
    os.replace(tmp, path)


def replay_feed(a, svc, targets):
    from tools.memory_quality import replay_stream as rs
    K, T_bc, image, frames, _ = rs.load_stream(a.feed.split(":", 1)[1])
    _K["K"] = K
    if a.max_frames:
        frames = frames[:a.max_frames]
    names = sorted({o["label"] for _, _, obs in frames for o in obs})
    boxf = None
    if a.emit_boxes:
        boxf = open(a.emit_boxes, "w")
        if a.emit_names:
            write_json(a.emit_names, {"names": names})
    t_first, w0 = frames[0][0], time.monotonic()
    nxt = w0
    for i, (t, base, obs) in enumerate(frames):
        if a.speed > 0:
            dt = (t - t_first) / a.speed - (time.monotonic() - w0)
            if dt > 0:
                time.sleep(dt)
        if boxf is not None and obs:
            dets = [list(o["bbox"]) + [o["score"], names.index(o["label"])] for o in obs]
            boxf.write(json.dumps({"seq": i, "t_out": t, "dets": dets}) + "\n")
            boxf.flush()
        svc.update(make_rec(i, t, base_matrix(base) @ T_bc, obs, K, image))
        now = time.monotonic()
        if now >= nxt:
            nxt = now + a.period
            write_json(a.out, dict(svc.state(targets), feed="replay", stream_t=round(t - t_first, 2), stream_i=i))
    st = dict(svc.state(targets), feed="replay", stream_t=round(frames[-1][0] - t_first, 2),
              stream_i=len(frames) - 1, done=True)
    write_json(a.out, st)
    return st


def ring_feed(a, svc, targets):
    """Bench live feed (see module doc for its limits)."""
    from tools.golive.kv_uplink import Tail
    from tools.kv_cam.cam_ring import CamRing
    cp = json.load(open(a.cam_params)) if a.cam_params else {}
    K = np.array(cp.get("K", [[380.0, 0, 320.0], [0, 380.0, 240.0], [0, 0, 1]]), float)
    T_wc = np.array(cp.get("T_world_camera", [[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0.3], [0, 0, 0, 1]]), float)
    if not a.cam_params:
        warn("no --cam-params: K from the ring header (else nominal), camera pose = the bench pose of "
             "tools/nn_cam2/params_car_bench.json")
    _K["K"] = K
    names = []
    if a.names:
        try:
            names = json.load(open(a.names)).get("names", [])
        except (OSError, ValueError) as exc:
            warn("names: %r" % exc)
    ring = None
    tb = Tail(a.boxes)
    i, t0, nxt = 0, time.monotonic(), time.monotonic()
    last_ring_warn = 0.0
    while True:
        now = time.monotonic()
        rows = tb.lines()
        depth = None
        if rows:
            try:
                if ring is None:
                    ring = CamRing.open(a.ring_name)
                    if "K" not in cp:      # intrinsics from the ring header, as host/kv_local_frames._Ring does
                        try:
                            import ctypes
                            import struct
                            hdr = ctypes.string_at(ring._lib.kvcam_header(ring._h), 512)
                            fx, fy, cx, cy = struct.unpack_from("<4f", hdr, 176)
                            if struct.unpack_from("<I", hdr, 224)[0] and fx > 0 and fy > 0:
                                K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], float)
                                _K["K"] = K
                                print(json.dumps({"memsvc": "ring intrinsics", "fx": fx, "fy": fy, "cx": cx,
                                                  "cy": cy}), flush=True)
                        except Exception as exc:                    # noqa: BLE001
                            warn("ring intrinsics: %r (nominal K kept)" % exc)
                st, meta, _, d = ring.read(ring.latest(), colour=False, depth=True)
                if st == "ok":
                    depth = d.astype(np.float32) * (float(meta.get("depth_scale_mm") or 1.0) / 1000.0)
            except Exception as exc:                                # noqa: BLE001
                ring = None
                if now - last_ring_warn > 10:
                    last_ring_warn = now
                    warn("ring: %r" % exc)
        for r in rows:
            dets = r.get("dets") or []
            obs = []
            if depth is not None:
                for d in dets:
                    x1, y1, x2, y2, sc, c = d[:6]
                    cx, cy, hw, hh = (x1 + x2) / 2, (y1 + y2) / 2, max(1, (x2 - x1) / 4), max(1, (y2 - y1) / 4)
                    sx = depth.shape[1] / float(getattr(ring, "colour_w", 640) or 640)   # box: colour pixels
                    sy = depth.shape[0] / float(getattr(ring, "colour_h", 480) or 480)
                    patch = depth[int(max(0, (cy - hh) * sy)):int((cy + hh) * sy) + 1,
                                  int(max(0, (cx - hw) * sx)):int((cx + hw) * sx) + 1]
                    z = patch[(patch > 0.1) & (patch < 5)]
                    if z.size < 9:
                        continue
                    zc = float(np.median(z))
                    pc = np.linalg.inv(K) @ np.array([cx, cy, 1.0]) * zc
                    p = T_wc[:3, :3] @ pc + T_wc[:3, 3]
                    obs.append({"label": names[int(c)] if int(c) < len(names) else str(int(c)), "score": sc,
                                "position_m": p.tolist(), "bbox": [x1, y1, x2, y2], "color": "unknown"})
            i += 1
            svc.update(make_rec(i, t0 + (now - t0) + i * 1e-6, T_wc, obs, K, (640, 480)))
        if now >= nxt:
            nxt = now + a.period
            write_json(a.out, dict(svc.state(targets), feed="ring", up_s=round(now - t0, 1)))
        if a.seconds and now - t0 > a.seconds:
            return
        time.sleep(0.05)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--backend", default="pl", choices=["pl", "plsim", "cmodel", "py"])
    ap.add_argument("--sim-lib", default=None, help="plsim: core C model .so (e.g. MERGE10/drv/lib/libmemk4.so)")
    ap.add_argument("--feed", default="ring")
    ap.add_argument("--speed", type=float, default=0.0, help="replay: x real time (0 = as fast as possible)")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--emit-boxes", default=None)
    ap.add_argument("--emit-names", default=None)
    ap.add_argument("--boxes", default=None)
    ap.add_argument("--names", default=None)
    ap.add_argument("--ring-name", default="kv_cam_v1")
    ap.add_argument("--cam-params", default=None, help="JSON {K, T_world_camera}")
    ap.add_argument("--find", default="bottle", help="comma-separated targets answered in every state")
    ap.add_argument("--out", default="mem.json")
    ap.add_argument("--period", type=float, default=0.5)
    ap.add_argument("--seconds", type=float, default=0.0)
    ap.add_argument("--fail-after", type=int, default=None, help="negative control: fake a core failure")
    a = ap.parse_args(argv)
    targets = [s.strip() for s in a.find.split(",") if s.strip()]
    svc = MemService(a.backend, a.sim_lib, a.fail_after)
    print(json.dumps({"memsvc": "up", "requested": a.backend, "src": svc.kind, "fallbacks": svc.fallbacks}),
          flush=True)
    if a.feed.startswith("replay:"):
        st = replay_feed(a, svc, targets)
        print(json.dumps({"memsvc": "done", "src": st["src"], "calls": st["calls"], "fallbacks": st["fallbacks"],
                          "frames": st["frames"], "find": {k: v["found"] for k, v in st["find"].items()}}), flush=True)
    else:
        ring_feed(a, svc, targets)


if __name__ == "__main__":
    main()
