#!/usr/bin/env python3
"""NN-CAM1 host tests of the kv_cam shared-memory protocol (two key tests, each with a positive control).

T1 byte equality: kv_cam --source replay (a real recording packed by make_replay_pack.py) at 30 fps; two independent
   readers run at the same time - kv_cam_read (C++, memcmp against the pack) and the Python reader (cam_ring.py via
   libcamring.so, numpy compare against the pack). Every delivered frame must equal the pack frame named by its
   meta (colour_index / depth_index), byte for byte.
   Positive control T1+: the same run with --inject-misalign depth:K (from frame K on, depth bytes come from the
   next pack frame while the meta keeps the old index = a sequence misalignment). Both readers must catch it.
T2 overwrite safety: kv_cam --source synth at 1000 frames/s (8 slots wrap every 8 ms), a reader that sleeps a random
   0-12 ms between the colour and the depth copy, while the writer is still running. With the seqlock re-check: frames are reported "overwritten", none delivered
   with wrong bytes. Positive control T2+: the same reader without the re-check (--no-seqlock) must deliver torn
   frames (proves the test really produces overwrites mid-copy, i.e. the seqlock is what keeps T2 clean).
Smoke S1 (not a gate): start retry - two simulated failed starts, then streaming, alarms logged.

    python3 run_ring_tests.py --pack PACK --out DIR      (builds with `make host` first)
"""
import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.path.join(HERE, "build", "host")


def py_reader(name, pack, frames, timeout_s, out):
    """Second, independent consumer: Python + numpy, verifies against the pack file directly."""
    import numpy as np
    sys.path.insert(0, HERE)
    from cam_ring import CamRing, FLAG_HAS_COLOUR, FLAG_HAS_DEPTH
    raw = np.memmap(pack, np.uint8, "r")
    hdr = raw[:64].view("<u4")
    n, cb, db = int(hdr[3]), int(hdr[7]), int(hdr[11])
    stride = 32 + cb + db
    t_end = time.monotonic() + timeout_s
    ring = None
    while ring is None:
        try:
            ring = CamRing.open(name)
        except OSError:
            if time.monotonic() > t_end:
                raise
            time.sleep(0.02)
    res = dict(ok=0, overwritten=0, corrupt=0, mismatch_colour=0, mismatch_depth=0, first_bad="")
    nxt, idle_end = 1, time.monotonic() + timeout_s
    while res["ok"] + res["overwritten"] < frames:
        lat = ring.wait_newer(nxt - 1, 500)
        if lat < nxt:
            if time.monotonic() > idle_end:
                break
            continue
        idle_end = time.monotonic() + timeout_s
        st, m, c, d = ring.read(nxt)
        if st == "not_yet":
            continue
        if st == "overwritten":
            res["overwritten"] += 1
            nxt = max(nxt + 1, ring.latest() - ring.nslots + 2)
            continue
        if st != "ok":
            res["corrupt"] += 1
            nxt += 1
            continue
        res["ok"] += 1
        nxt += 1
        ci, di = m["colour_index"] % n, m["depth_index"] % n
        if m["flags"] & FLAG_HAS_COLOUR:
            ec = raw[64 + ci * stride + 32: 64 + ci * stride + 32 + cb]
            if not np.array_equal(c.reshape(-1), ec):
                res["mismatch_colour"] += 1
                res["first_bad"] = res["first_bad"] or "seq %d colour_index %d" % (m["seq"], m["colour_index"])
        if m["flags"] & FLAG_HAS_DEPTH:
            ed = raw[64 + di * stride + 32 + cb: 64 + di * stride + 32 + cb + db]
            if not np.array_equal(d.view(np.uint8).reshape(-1), ed):
                res["mismatch_depth"] += 1
                res["first_bad"] = res["first_bad"] or "seq %d depth_index %d" % (m["seq"], m["depth_index"])
    ring.close()
    res["pass"] = not (res["mismatch_colour"] or res["mismatch_depth"] or res["corrupt"])
    json.dump(res, open(out, "w"))
    print(json.dumps(res))


def run_arm(tag, out, writer_args, readers):
    """Start readers first (they wait for the ring), then the writer; collect JSON."""
    name = "kvcam_test_%s_%d" % (tag, os.getpid())
    procs = []
    for rname, rargs in readers:
        js = os.path.join(out, "%s_%s.json" % (tag, rname))
        if rname == "py":
            cmd = [sys.executable, __file__, "--py-reader", "--name", name, "--json", js] + rargs
        else:
            cmd = [os.path.join(BIN, "kv_cam_read"), "--name", name, "--json", js] + rargs
        procs.append((rname, js, subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)))
    time.sleep(0.3)
    wlog = os.path.join(out, "%s_writer.log" % tag)
    stats = os.path.join(out, "%s_writer_stats.json" % tag)
    with open(wlog, "w") as lf:
        rc_w = subprocess.call([os.path.join(BIN, "kv_cam"), "--name", name, "--stats-out", stats,
                                "--status-every-s", "0"] + writer_args, stdout=lf, stderr=subprocess.STDOUT)
    res = {"writer_rc": rc_w, "writer": json.load(open(stats)) if os.path.exists(stats) else None}
    for rname, js, p in procs:
        try:
            _, err = p.communicate(timeout=120)
        except subprocess.TimeoutExpired:
            p.kill()
            _, err = p.communicate()
        res[rname] = json.load(open(js)) if os.path.exists(js) else {"error": err.decode()[-400:]}
        res[rname]["rc"] = p.returncode
    try:
        os.unlink("/dev/shm/" + name)
    except FileNotFoundError:
        pass
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pack")
    ap.add_argument("--out")
    ap.add_argument("--py-reader", action="store_true")
    ap.add_argument("--name")
    ap.add_argument("--json")
    ap.add_argument("--frames", type=int, default=0)
    ap.add_argument("--timeout-s", type=float, default=15)
    a = ap.parse_args()
    if a.py_reader:
        return py_reader(a.name, a.pack, a.frames, a.timeout_s, a.json)
    os.makedirs(a.out, exist_ok=True)
    subprocess.check_call(["make", "-s", "-C", HERE, "host"])
    npack = int.from_bytes(open(a.pack, "rb").read(16)[12:16], "little")
    loops = 2
    total = npack * loops
    R = {"pack": a.pack, "pack_frames": npack, "loops": loops}
    common = ["--source", "replay", "--pack", a.pack, "--replay-fps", "30", "--loops", str(loops), "--start-wait-s", "5"]
    rd = [("cpp", ["--frames", str(total), "--verify", "pack:" + a.pack, "--timeout-s", "15"]),
          ("py", ["--frames", str(total), "--pack", a.pack, "--timeout-s", "15"])]
    t1 = run_arm("T1", a.out, common, rd)
    k = npack // 2
    t1p = run_arm("T1pos", a.out, common + ["--inject-misalign", "depth:%d" % k], rd)
    # 1000 frames/s into 8 slots = one ring wrap every 8 ms; the reader sleeps a random 0-12 ms between its colour
    # and depth copy, so some reads finish inside a wrap and some are overrun while the writer is still running
    syn = ["--source", "synth", "--replay-fps", "1000", "--frames", "12000", "--start-wait-s", "5", "--no-rate-check"]
    t2r = ["--frames", "1000", "--verify", "synth", "--mid-copy-delay-ms", "12", "--jitter", "--timeout-s", "10"]
    t2 = run_arm("T2", a.out, syn, [("cpp", t2r)])
    t2p = run_arm("T2pos", a.out, syn, [("cpp", t2r + ["--no-seqlock"])])
    s1 = run_arm("S1", a.out, ["--source", "synth", "--replay-fps", "30", "--frames", "150", "--fail-starts", "2",
                               "--start-wait-s", "1", "--retry-gap-s", "0.2", "--alarm-log",
                               os.path.join(a.out, "S1_alarms.jsonl")],
                 [("cpp", ["--frames", "60", "--verify", "synth", "--timeout-s", "15"])])

    def clean(r, rn):
        x = r[rn]
        return x.get("mismatch_colour", 1) == 0 and x.get("mismatch_depth", 1) == 0 and x.get("corrupt", 1) == 0

    verdict = {
        "T1_byte_equal": all(clean(t1, rn) and t1[rn]["ok"] >= 0.95 * total for rn in ("cpp", "py"))
        and t1["writer"]["published"] == total,
        "T1pos_misalign_caught": all(t1p[rn].get("mismatch_depth", 0) > 0 and t1p[rn].get("mismatch_colour", 1) == 0
                                     for rn in ("cpp", "py")),
        "T2_no_torn_frame_delivered": clean(t2, "cpp") and t2["cpp"]["overwritten"] >= 20 and t2["cpp"]["ok"] >= 20
        and t2["cpp"]["last_seq"] < t2["writer"]["published"],
        "T2pos_torn_without_seqlock": t2p["cpp"].get("mismatch_depth", 0) > 0,
        "S1_start_retry_smoke": s1["writer"]["tries"] == 3 and clean(s1, "cpp") and s1["cpp"]["ok"] >= 60,
    }
    R.update(T1=t1, T1pos=t1p, T2=t2, T2pos=t2p, S1=s1, verdict=verdict, all_pass=all(verdict.values()))
    json.dump(R, open(os.path.join(a.out, "ring_tests.json"), "w"), indent=1)
    for kx, v in verdict.items():
        print("%-30s %s" % (kx, "PASS" if v else "FAIL"))
    for tag, r in (("T1", t1), ("T1pos", t1p), ("T2", t2), ("T2pos", t2p), ("S1", s1)):
        rs = {rn: {kk: r[rn].get(kk) for kk in ("ok", "overwritten", "corrupt", "mismatch_colour", "mismatch_depth")}
              for rn in r if rn not in ("writer", "writer_rc")}
        print(tag, "writer", {kk: r["writer"].get(kk) for kk in ("published", "tries", "alarms", "cpu_cores")}, rs)
    return 0 if R["all_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
