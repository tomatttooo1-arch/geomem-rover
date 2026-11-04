#!/usr/bin/env python3
"""NN-CAM2 host test: the local frame source gives the cores the same input bytes as the UDP path, frame by frame.

One recording (a kv_cam replay pack), two ways in:
  U  (today's path)  a Nano stand-in sends it over UDP on loopback with the Nano's own client code:
       depth  host.kv_link.RemotePlFrameBackend.frame_from_depth (codes supplied, window 1) -> kv_server --frame-source udp
       colour host.kv_colour.ColourSource (the Nano's DPU preprocessing on yuyv_rgb.yuyv_to_rgb of the pack's YUYV,
              key = frame index + 1)                                                      -> dpu_rx --frame-source udp
  L  (new path)      kv_cam --source replay writes the pack into a shared-memory ring; kv_server and dpu_rx, both
       --frame-source kvcam, read the same ring.
Each process dumps the exact buffer it hands to the core: kv_server --dump-core-input (geometry-core input: inverse_k,
world_camera, origin, voxel, sizes, unit, float32 depth), dpu_rx --dump-tensors (the int8 tensor staged for the DPU).
The comparison is byte for byte per camera frame index. Positive controls: L with kv_cam --inject-misalign (from
frame K on, each frame carries frame K+1's bytes under its own index) for depth and colour; one flipped byte in a copy
of the L dumps. Every process this script starts is stopped by its own pid.

  run_equiv.py --pack P --params J --work W --out result.json [--fps 10] [--size 512] [--misalign-from 45]
"""
import argparse
import hashlib
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
from pack import Pack  # noqa: E402
from yuyv_rgb import yuyv_to_rgb  # noqa: E402

PY = sys.executable
KV_SERVER = os.path.join(ROOT, "tools", "kv_link", "kv_server.py")
KV_CAM = os.path.join(ROOT, "tools", "kv_cam", "build", "host", "kv_cam")
DPU_RX = os.path.join(ROOT, "tools", "dpu_pipe", "build", "dpu_rx")


def free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def stop(proc, timeout=20):
    if proc.poll() is None:
        os.kill(proc.pid, signal.SIGTERM)
    try:
        proc.wait(timeout)
    except subprocess.TimeoutExpired:
        os.kill(proc.pid, signal.SIGKILL)
        proc.wait()
    return proc.returncode


def start_kv_server(out, extra, log):
    os.makedirs(out, exist_ok=True)
    p = subprocess.Popen([PY, KV_SERVER, "--core", "cpu", "--port", "0", "--max-idle-s", "3600", "--out", out] + extra,
                         stdout=subprocess.PIPE, stderr=open(log, "w"), cwd=ROOT, text=True)
    ready = json.loads(p.stdout.readline())
    threading.Thread(target=p.stdout.read, daemon=True).start()
    return p, ready


def start_dpu_rx(extra, log, out_json):
    return subprocess.Popen([DPU_RX, "--engine", "stub", "--infer-ms", "2", "--seconds", "3600"] + extra,
                            stdout=open(out_json, "w"), stderr=open(log, "w"), cwd=ROOT)


def depth_udp(pk, params, n, dump, work):
    from host.kv_link import codes_to_m, remote_pl_factory
    srv, ready = start_kv_server(os.path.join(work, "dU_srv"), ["--dump-core-input", dump],
                                 os.path.join(work, "dU_srv.err"))
    from host.kv_link import LinkError
    be = remote_pl_factory(("127.0.0.1", ready["port"]), depth_unit_m=float(params.unit), timeout_s=3.0)()
    ok, retries, failed = 0, 0, []
    try:
        for i in range(n):
            codes = pk.depth(i)
            ident = types.SimpleNamespace(frame_id=i, acquired_ns=1000 + i, **params.identity)
            # the host's SO_RCVBUF is capped at rmem_max (425 KB, no root) < one 614 KB request: a lagging receive
            # thread drops chunks -> LinkTimeout. Resend the same frame (same bytes; the dump is per frame id).
            for attempt in range(30):
                try:
                    be.frame_from_depth(identity=ident, codes=codes,
                                        depth_m=codes_to_m(codes, params.unit).reshape(codes.shape),
                                        k=params.k, camera_to_world=params.camera_to_world,
                                        grid_origin_m=params.grid.origin_m, voxel_size_m=params.voxel_m,
                                        shape=params.shape, frame_id=i, capture_time_ns=1000 + i,
                                        pose_time_ns=1000 + i, calibration_version=params.identity["calibration_id"],
                                        pose_confirmed=True)
                    ok += 1
                    break
                except LinkError:
                    retries += 1
                    time.sleep(0.05 * min(attempt + 1, 10))
            else:
                failed.append(i)
    finally:
        be.client.close()
        stop(srv)
    return {"sent": n, "answered": ok, "retries": retries, "failed": failed}


class _ColourFeeder:
    """Nano stand-in for the colour channel: frames into a ColourRing with key = index + 1 at --fps."""

    def __init__(self, pk, n, fps, kv_port, size, fp):
        from host.kv_colour import ColourRing, ColourSource
        self.ring = ColourRing(12)
        self.src = ColourSource([("127.0.0.1", kv_port)], self.ring.lookup, info={"source": "nn_cam2_equiv"},
                                latest=self.ring.newest, hello_s=0.2, dpu_backend="auto")
        self.src.warm_dpu(size, fp, pk.cw, pk.ch)
        self.pk, self.n, self.fps = pk, n, fps

    def run(self):
        self.src.start()
        t = time.monotonic()
        for i in range(self.n):
            rgb = yuyv_to_rgb(self.pk.colour(i), self.pk.cw, self.pk.ch, 0, 1)
            self.ring.put(i + 1, self.pk.cw, self.pk.ch, rgb.tobytes())
            t += 1.0 / self.fps
            time.sleep(max(0.0, t - time.monotonic()))
        time.sleep(1.5)
        self.src.stop()
        return self.src.describe()


def colour_udp(pk, n, fps, size, fp, dump, work):
    port = free_port()
    rx = start_dpu_rx(["--frame-source", "udp", "--port", str(port), "--size", str(size), "--fix", str(fp),
                       "--src-wh", "%dx%d" % (pk.cw, pk.ch), "--dump-tensors", dump],
                      os.path.join(work, "cU_rx.err"), os.path.join(work, "cU_rx.json"))
    time.sleep(0.5)
    feeder = _ColourFeeder(pk, n, fps, port, size, fp)
    d = feeder.run()
    stop(rx)
    # dumps are named by the Nano key (= index + 1): rename to the camera frame index
    for f in os.listdir(dump):
        k = int(f[:-4])
        os.rename(os.path.join(dump, f), os.path.join(dump, "i%d.bin" % (k - 1)))
    for f in os.listdir(dump):
        os.rename(os.path.join(dump, f), os.path.join(dump, f[1:]))
    return {"sender": {k: d.get(k) for k in ("counts", "dpu_prep_ms_p50") if k in d}}


def local_arm(tag, pk_path, params_path, n, fps, size, fp, work, inject=None):
    dd, cd = os.path.join(work, tag + "_depth"), os.path.join(work, tag + "_colour")
    for d in (dd, cd):
        os.makedirs(d, exist_ok=True)
    ring = "nncam2_eq_%d_%s" % (os.getpid(), tag)
    srv, _ready = start_kv_server(os.path.join(work, tag + "_srv"),
                                  ["--frame-source", "kvcam", "--kvcam-name", ring, "--local-params", params_path,
                                   "--dump-core-input", dd, "--stamp-log", os.path.join(work, tag + "_depth.jsonl")],
                                  os.path.join(work, tag + "_srv.err"))
    rx = start_dpu_rx(["--frame-source", "kvcam", "--kvcam-name", ring, "--size", str(size), "--fix", str(fp),
                       "--dump-tensors", cd, "--stamp-log", os.path.join(work, tag + "_colour.jsonl")],
                      os.path.join(work, tag + "_rx.err"), os.path.join(work, tag + "_rx.json"))
    time.sleep(1.0)
    cmd = [KV_CAM, "--source", "replay", "--pack", pk_path, "--name", ring, "--replay-fps", str(fps),
           "--frames", str(n), "--loops", "1"]
    if inject:
        cmd += ["--inject-misalign", inject]
    cam = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=600)
    time.sleep(2.0)
    rc = {"kv_server": stop(srv), "dpu_rx": stop(rx), "kv_cam": cam.returncode}
    try:
        os.unlink("/dev/shm/" + ring)
    except OSError:
        pass
    open(os.path.join(work, tag + "_cam.err"), "w").write(cam.stderr[-20000:])
    return dd, cd, rc


def compare(dir_l, dir_u, n):
    rows, mism, missing_u, have = [], [], [], 0
    for i in range(n):
        fl, fu = os.path.join(dir_l, "%d.bin" % i), os.path.join(dir_u, "%d.bin" % i)
        if not os.path.exists(fl):
            continue
        have += 1
        if not os.path.exists(fu):
            missing_u.append(i)
            continue
        a, b = open(fl, "rb").read(), open(fu, "rb").read()
        if a != b:
            diff = (int(np.count_nonzero(np.frombuffer(a, np.uint8) != np.frombuffer(b, np.uint8)))
                    if len(a) == len(b) else -1)
            mism.append({"frame": i, "diff_bytes": diff})
    nb = sum(os.path.getsize(os.path.join(dir_l, "%d.bin" % i)) for i in range(n)
             if os.path.exists(os.path.join(dir_l, "%d.bin" % i)) and os.path.exists(os.path.join(dir_u, "%d.bin" % i)))
    return {"local_frames": have, "udp_frames": len([f for f in os.listdir(dir_u)]), "compared": have - len(missing_u),
            "bytes_compared": nb, "mismatch_frames": len(mism), "mismatch_first": mism[:5],
            "mismatch_frame_ids": [m["frame"] for m in mism], "missing_in_udp": missing_u}


def flip_copy(src, dst, frame, offset_from_end=1000):
    if os.path.exists(dst):
        shutil.rmtree(dst)
    shutil.copytree(src, dst)
    p = os.path.join(dst, "%d.bin" % frame)
    b = bytearray(open(p, "rb").read())
    pos = len(b) - offset_from_end
    b[pos] ^= 0x01
    open(p, "wb").write(bytes(b))
    return {"frame": frame, "byte": pos}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pack", required=True)
    ap.add_argument("--params", required=True)
    ap.add_argument("--work", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", type=int, default=0)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--colour-fps", type=float, default=5.0)
    ap.add_argument("--size", type=int, default=512)
    ap.add_argument("--misalign-from", type=int, default=45)
    a = ap.parse_args()
    from host.dpu_prep import DEFAULT_FIX_POINT
    from host.kv_local_frames import FrameParams
    os.makedirs(a.work, exist_ok=True)
    pk = Pack(a.pack)
    n = a.frames or pk.n
    fp = DEFAULT_FIX_POINT[a.size]
    params = FrameParams(json.load(open(a.params)))
    res = {"pack": a.pack, "pack_sha256": hashlib.sha256(open(a.pack, "rb").read()).hexdigest(), "frames": n,
           "size": a.size, "fix_point": fp, "params": params.describe(), "python": sys.version.split()[0]}
    t0 = time.monotonic()
    dU, cU = os.path.join(a.work, "U_depth"), os.path.join(a.work, "U_colour")
    for d in (dU, cU):
        shutil.rmtree(d, ignore_errors=True)
        os.makedirs(d)
    res["udp_depth"] = depth_udp(pk, params, n, dU, a.work)
    res["udp_colour"] = colour_udp(pk, n, a.colour_fps, a.size, fp, cU, a.work)
    dL, cL, rc = local_arm("L", a.pack, a.params, n, a.fps, a.size, fp, a.work)
    res["local_rc"] = rc
    res["depth"] = compare(dL, dU, n)
    res["colour"] = compare(cL, cU, n)
    k = a.misalign_from
    dM, _, rcm = local_arm("Ldm", a.pack, a.params, n, a.fps, a.size, fp, a.work, inject="depth:%d" % k)
    _, cM, rcc = local_arm("Lcm", a.pack, a.params, n, a.fps, a.size, fp, a.work, inject="colour:%d" % k)
    res["pc_misalign_depth"] = dict(compare(dM, dU, n), rc=rcm, inject_from=k)
    res["pc_misalign_colour"] = dict(compare(cM, cU, n), rc=rcc, inject_from=k)
    fd = min(int(f[:-4]) for f in os.listdir(dL))
    fc = min(int(f[:-4]) for f in os.listdir(cL))
    res["pc_flip_depth"] = dict(compare(flip_copy(dL, dL + "_flip", fd) and dL + "_flip", dU, n),
                                flipped={"frame": fd})
    res["pc_flip_colour"] = dict(compare(flip_copy(cL, cL + "_flip", fc, 300000) and cL + "_flip", cU, n),
                                 flipped={"frame": fc})
    res["wall_s"] = round(time.monotonic() - t0, 1)
    d, c = res["depth"], res["colour"]

    for key, dirl in (("pc_misalign_depth", dM), ("pc_misalign_colour", cM)):
        r = res[key]
        have = {i for i in range(n) if os.path.exists(os.path.join(dirl, "%d.bin" % i))}
        expect = {i for i in have if i >= k and os.path.exists(os.path.join(dU if "depth" in key else cU, "%d.bin" % i))}
        got = set(r["mismatch_frame_ids"])
        r["expected_mismatch"] = len(expect)
        r["caught_all"] = expect <= got
        r["false_alarms_before_k"] = sorted(got - expect)
    res["verdict"] = {
        "depth_equal": d["mismatch_frames"] == 0 and not d["missing_in_udp"] and d["compared"] > 0,
        "colour_equal": c["mismatch_frames"] == 0 and c["compared"] > 0,
        "pc_misalign_depth_caught": res["pc_misalign_depth"]["caught_all"]
        and not res["pc_misalign_depth"]["false_alarms_before_k"],
        "pc_misalign_colour_caught": res["pc_misalign_colour"]["caught_all"]
        and not res["pc_misalign_colour"]["false_alarms_before_k"],
        "pc_flip_depth_caught": res["pc_flip_depth"]["mismatch_frames"] == 1,
        "pc_flip_colour_caught": res["pc_flip_colour"]["mismatch_frames"] == 1}
    json.dump(res, open(a.out, "w"), indent=1, default=str)
    print(json.dumps({"verdict": res["verdict"], "depth": {k2: d[k2] for k2 in ("local_frames", "compared",
                      "bytes_compared", "mismatch_frames")}, "colour": {k2: c[k2] for k2 in (
                      "local_frames", "udp_frames", "compared", "bytes_compared", "mismatch_frames")},
                      "wall_s": res["wall_s"]}))
    return 0 if all(res["verdict"].values()) else 1


if __name__ == "__main__":
    sys.exit(main())
