#!/usr/bin/env python3
"""Equivalence gate: runtime geometry cpu (compute_geometry) vs kv (host/kv_geometry.py).

One prepared depth sequence (rec30 / TUM prepared: depth_u16.bin + meta.json), fixed camera
(world = camera, as the prepared sets are), one memory of N boxes made from frame 0 by
tools/kv_link/nano_replay.make_roi_memory. Two arms run frame by frame on the SAME depth:

  cpu arm: ev = compute_geometry(frame, mem_cpu); mem_cpu.update_memory([], ev, frame)
  kv  arm: ev = KvGeometry.evaluate(frame, mem_kv, codes=...); mem_kv.update_memory([], ev, frame)

Per frame it compares (1) the evidence dicts, (2) the full Memory snapshots after the update
(the downstream state), (3) every KV box row's four counts against the local C reference
``NativeObjectEvidence.evaluate_boxes`` on the Nano's float32 depth (catches a one-pixel or
one-frame error that leaves the word unchanged). Depth goes through the exact ros_runtime
16UC1 conversion (astype float32, *= scale, <= 0 -> NaN).

KV end: ``--server HOST:PORT`` (real kv_server, e.g. the KV260) or ``--fake cpu|shift1|prevframe``
(in-process KvLinkServer on loopback with the g++ frame core and the E2 C ROI core;
shift1/prevframe are the positive controls: every box evaluated one column to the right /
on the previous request's depth, rows still self-consistent so only this gate can catch them).

Writes <out>/summary.json (+ mismatches.jsonl). Exit 0 only when every frame is equal (or, for
a positive-control variant, when the gate caught it).
Usage: equivalence.py --prep DIR --out NEWDIR (--server H:P | --fake cpu|shift1|prevframe)
                      [--frames N] [--boxes 24] [--expect-build-id 0x...] [--timeout-s 1.0]
"""
import argparse
import json
import os
import sys
import tempfile
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.kv_geometry import DEFAULT_GRID, KvGeometry, parse_build_id, parse_grid  # noqa: E402
from host.fpga_clock import now_ns  # noqa: E402
from perception.fpga_replay.core import Config, Frame, Memory, Track, compute_geometry  # noqa: E402


def runtime_depth(codes, scale):
    """host/ros_runtime.py prepare_images, 16UC1 branch, verbatim."""
    depth_array = np.asarray(codes).astype(np.float32)
    depth_array *= scale
    depth_array[depth_array <= 0] = np.nan
    return depth_array


def load(prep):
    meta = json.load(open(os.path.join(prep, "meta.json")))
    w, h, n = int(meta["width"]), int(meta["height"]), int(meta["frames"])
    depth = np.memmap(os.path.join(prep, meta["depth_file"]), dtype="<u2", mode="r")
    if depth.size != w * h * n:
        raise ValueError("depth file size disagrees with meta")
    return meta, depth.reshape(n, h, w)


def memory_from_snapshot(snap):
    mem = Memory()
    for t in snap["tracks"]:
        lo, hi = (np.asarray(c, dtype=np.float64) for c in t["bounds"])
        mem.tracks[t["id"]] = Track(t["id"], t["label"], "unknown", (lo + hi) / 2, lo, hi,
                                    float(t["last_seen"]) - 1e3)
    return mem


class _ShiftRoi(object):
    """Positive control: each enabled box evaluated one column over (area kept)."""

    def __init__(self, inner):
        self.inner, self.name = inner, inner.name

    def __call__(self, depth2d, roi):
        w = depth2d.shape[1]
        rows = []
        for (rid, en, x0, y0, x1, y1, nz, fz) in roi["rows"]:
            if en:
                d = 1 if x1 + 1 < w else -1
                x0, x1 = x0 + d, x1 + d
            rows.append((rid, en, x0, y0, x1, y1, nz, fz))
        return self.inner(depth2d, dict(roi, rows=rows))


class _PrevFrameRoi(object):
    """Positive control: boxes evaluated on the previous request's depth."""

    def __init__(self, inner):
        self.inner, self.name, self.prev = inner, inner.name, None

    def __call__(self, depth2d, roi):
        cur = np.array(depth2d, copy=True)
        use = cur if self.prev is None else self.prev
        self.prev = cur
        return self.inner(use, roi)


def fake_server(kind, build):
    from host.kv_link import CpuRoiCore, KvLinkServer
    from host.m7_frame_native import FrameNativeM7, compile_frame_native
    from host.m7_object_native import compile_object_native
    frame_lib = compile_frame_native(os.path.join(build, "frame.so"))
    roi = CpuRoiCore(compile_object_native(os.path.join(build, "object_kv.so")))
    if kind == "shift1":
        roi = _ShiftRoi(roi)
    elif kind == "prevframe":
        roi = _PrevFrameRoi(roi)
    server = KvLinkServer(FrameNativeM7(frame_lib).lib.fpga_m7_depth_frame, "cpu",
                          bind=("127.0.0.1", 0), roi_core=roi)
    server.start()
    return server


class FakeServerProcess(object):
    def __init__(self, kind):
        import subprocess
        self.proc = subprocess.Popen([sys.executable, os.path.abspath(__file__), "--serve-fake", kind],
                                     stdout=subprocess.PIPE, stdin=subprocess.PIPE, cwd=ROOT)
        line = self.proc.stdout.readline()
        self.port = json.loads(line.decode())["port"]
        self.counts = {}

    def stop(self):
        self.proc.stdin.close()            # child: EOF on stdin -> print counts, exit
        out = self.proc.stdout.read()
        self.proc.wait(timeout=30)
        try:
            self.counts = json.loads(out.decode().strip().splitlines()[-1])
        except (ValueError, IndexError):
            self.counts = {"unreadable": out[-200:].decode("utf-8", "replace")}


def serve_fake(kind):
    build = tempfile.mkdtemp(prefix="cargeo_fake_")
    server = fake_server(kind, build)
    print(json.dumps({"port": server.port, "rcvbuf": server.rcvbuf}), flush=True)
    sys.stdin.read()
    server.stop()
    print(json.dumps(dict(server.counts)), flush=True)
    return 0


def main(argv=None):
    if argv is None and len(sys.argv) == 3 and sys.argv[1] == "--serve-fake":
        return serve_fake(sys.argv[2])
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prep", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--server", default=None)
    ap.add_argument("--fake", choices=("cpu", "shift1", "prevframe"), default=None)
    ap.add_argument("--frames", type=int, default=0, help="first N frames (0 = all)")
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--boxes", type=int, default=24)
    ap.add_argument("--roi-frame", type=int, default=0)
    ap.add_argument("--expect-build-id", default=None)
    ap.add_argument("--timeout-s", type=float, default=1.0)
    ap.add_argument("--pace-ms", type=float, default=None,
                    help="pause between request chunks (default 2 with --fake, else 0)")
    ap.add_argument("--grid", default=None, help="ox,oy,oz,voxel,nx,ny,nz (default rec30 32^3 x 5 cm)")
    a = ap.parse_args(argv)
    if (a.server is None) == (a.fake is None):
        ap.error("exactly one of --server / --fake")
    os.makedirs(a.out, exist_ok=False)
    meta, depth = load(a.prep)
    unit = float(meta["depth_unit_m"])
    K = np.asarray(meta["K"], dtype=np.float64)
    n_src = depth.shape[0]
    idx = list(range(0, n_src if not a.frames else min(n_src, a.frames), max(1, a.stride)))
    build = tempfile.mkdtemp(prefix="cargeo_eq_")
    from host.m7_object_native import NativeObjectEvidence, compile_object_native
    from tools.kv_link.nano_replay import make_roi_memory
    ref = NativeObjectEvidence(compile_object_native(os.path.join(build, "object_ref.so")))
    server = None
    if a.fake:
        # separate process: an in-process server shares the GIL with the client and its
        # reader falls behind a 0.5-0.6 MB request burst when SO_RCVBUF is capped (host
        # rmem_max -> 416 KB effective): chunks dropped, LinkTimeout (seen 10-11, TUM).
        server = FakeServerProcess(a.fake)
        addr = ("127.0.0.1", server.port)
    else:
        host, port = a.server.rsplit(":", 1)
        addr = (host, int(port))
    snap = make_roi_memory(runtime_depth(depth[a.roi_frame], unit), K, a.boxes)
    mem_cpu, mem_kv = memory_from_snapshot(snap), memory_from_snapshot(snap)
    grid = parse_grid(a.grid) if a.grid else DEFAULT_GRID
    pace = a.pace_ms if a.pace_ms is not None else (2.0 if a.fake else 0.0)
    factory = None
    if pace > 0:
        from host.kv_link import KvLinkClient

        class PacedClient(KvLinkClient):
            """Host fixture only: a pause between request chunks, because this sandbox caps
            SO_RCVBUF at 416 KB (no SO_RCVBUFFORCE) and an 11-chunk TUM request overruns it."""
            def _send(self, msg_type, frame_id, payload, flags=0):
                from host.kv_link import TYPE_NAMES, split
                n = 0
                for dgram in split(msg_type, self.session, frame_id, payload, self.chunk_bytes, flags):
                    if n:
                        time.sleep(pace / 1e3)
                    self.sock.sendto(dgram, self.server)
                    n += 1
                self.counts["tx_chunks"] += n
                self.counts["tx_chunks:%s" % TYPE_NAMES[msg_type]] += n
                self.counts["tx_messages:%s" % TYPE_NAMES[msg_type]] += 1
                return n
        factory = lambda srv: PacedClient(srv, timeout_s=a.timeout_s)  # noqa: E731
    geo = KvGeometry(addr, depth_unit_m=unit, timeout_s=a.timeout_s, hello_timeout_s=2.0, grid=grid,
                     expect_build_id=parse_build_id(a.expect_build_id), stream_tag="equivalence",
                     client_factory=factory)
    if not geo.connect():
        raise SystemExit("KV not usable: %s" % geo.reason)
    cfg = Config()
    rgb = np.zeros(depth.shape[1:] + (3,), dtype=np.uint8)
    mism = open(os.path.join(a.out, "mismatches.jsonl"), "w")
    tally = {"frames": 0, "evidence_equal": 0, "snapshot_equal": 0, "kv_failed": 0,
             "rows": 0, "rows_counts_equal": 0, "words": {}, "overflow_tracks": 0, "roi_core": set()}
    rtts, sends = [], []
    t0 = time.monotonic()
    for k, i in enumerate(idx):
        codes = np.asarray(depth[i])
        d = runtime_depth(codes, unit)
        frame = Frame("eq-%d" % i, 1000.0 + k, rgb, d, K, np.eye(4), True, "replay",
                      str(meta.get("calibration_id", "unknown")), None)
        ev_cpu = compute_geometry(frame, mem_cpu, cfg)
        out = geo.evaluate(frame, mem_kv, codes=codes, rx_ns=now_ns(), cfg=cfg)
        tally["frames"] += 1
        bad = {}
        if out.evidence is None:
            tally["kv_failed"] += 1
            bad["kv_failure"] = out.reason
            ev_kv = {}
        else:
            ev_kv = out.evidence
            tally["roi_core"].add(out.info.get("roi_core"))
            last = out.info.get("client_last") or {}
            if last.get("rtt_s") is not None:
                rtts.append(last["rtt_s"] * 1e3)
            sends.append((out.info["back_ns"] - out.info["send_ns"]) / 1e6)
        tally["overflow_tracks"] += len(out.info.get("overflow") or ())
        if list(ev_cpu.items()) == list(ev_kv.items()):
            tally["evidence_equal"] += 1
        else:
            bad["evidence"] = {t: [ev_cpu.get(t), ev_kv.get(t)] for t in set(ev_cpu) | set(ev_kv)
                               if ev_cpu.get(t) != ev_kv.get(t)}
        for word in ev_kv.values():
            tally["words"][word] = tally["words"].get(word, 0) + 1
        rows = [r for r in out.info.get("rows", ()) if r["code"] is not None]
        if rows:
            refrows = ref.evaluate_boxes(d, [(j, tuple(r["xyxy"]), r["near_far"][0], r["near_far"][1])
                                             for j, r in enumerate(rows)], cfg)
            for r, rr in zip(rows, refrows):
                tally["rows"] += 1
                want = [rr["roi_pixels"], rr["roi_valid_pixels"], rr["roi_front_pixels"], rr["roi_behind_pixels"]]
                if want == r["counts"] and rr["verdict_code"] == r["code"]:
                    tally["rows_counts_equal"] += 1
                else:
                    bad.setdefault("rows", []).append({"track": r["track_id"], "kv": r["counts"], "ref": want})
        mem_cpu.update_memory([], ev_cpu, frame)
        mem_kv.update_memory([], ev_kv if out.evidence is not None else {}, frame)
        s_cpu = json.dumps(mem_cpu.snapshot(), sort_keys=True)
        s_kv = json.dumps(mem_kv.snapshot(), sort_keys=True)
        if s_cpu == s_kv:
            tally["snapshot_equal"] += 1
        else:
            bad["snapshot"] = True
        if bad:
            bad["frame"] = i
            mism.write(json.dumps(bad) + "\n")
    wall = time.monotonic() - t0
    mism.close()
    geo.close()
    if server is not None:
        server.stop()
    server_counts = dict(server.counts) if server is not None else None

    def pct(v, q):
        return round(float(np.percentile(v, q)), 3) if v else None
    tally["roi_core"] = sorted(x for x in tally["roi_core"] if x)
    n = tally["frames"]
    all_equal = (tally["evidence_equal"] == n and tally["snapshot_equal"] == n and tally["kv_failed"] == 0
                 and tally["rows_counts_equal"] == tally["rows"])
    summary = {"prep": os.path.abspath(a.prep), "depth_sha256": meta.get("depth_sha256"),
               "frames_idx": [idx[0], idx[-1], a.stride], "boxes": len(snap["tracks"]),
               "kv": geo.describe(), "fake": a.fake, "pace_ms": pace, "numpy": np.__version__,
               "python": sys.version.split()[0], "tally": tally, "all_equal": all_equal,
               "rtt_ms": {"median": pct(rtts, 50), "p95": pct(rtts, 95), "n": len(rtts)},
               "send_to_back_ms": {"median": pct(sends, 50), "p95": pct(sends, 95)},
               "kv_counts": dict(geo.counts), "fake_server_counts": server_counts,
               "wall_s": round(wall, 3)}
    with open(os.path.join(a.out, "summary.json"), "w") as fp:
        json.dump(summary, fp, indent=1, sort_keys=True)
    print(json.dumps({"all_equal": all_equal, "frames": n, "evidence_equal": tally["evidence_equal"],
                      "snapshot_equal": tally["snapshot_equal"], "rows": tally["rows"],
                      "rows_counts_equal": tally["rows_counts_equal"], "kv_failed": tally["kv_failed"],
                      "roi_core": tally["roi_core"], "words": tally["words"], "numpy": np.__version__,
                      "rtt_ms_median": summary["rtt_ms"]["median"]}))
    if a.fake in ("shift1", "prevframe"):
        return 0 if not all_equal else 3          # the control must be caught
    return 0 if all_equal else 2


if __name__ == "__main__":
    sys.exit(main())
