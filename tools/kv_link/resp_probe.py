#!/usr/bin/env python3
"""Frame-answer (RESP) lateness probe (2026-10-11, docs/design/KV_LINK_BURST_TAIL_20261011.md).

Sends real frame requests (rec30 depth codes, the car geometry producer's 32x32x10 block,
voxel 0.05 m, camera 0.35 m / 27 deg down) to kv_server.py one at a time (window 1),
back to back or at ``--hz``, and records per frame the send time and when the answer
came. The discriminating part, as in hello_probe.py: a frame whose answer has not come
within ``--timeout-s`` is NOT given up. The probe keeps receiving on the same socket for
``--linger-s`` WITHOUT sending anything (does the answer arrive late on its own?), then
``--kick``s: one HELLO from a separate socket, which makes the KV260 transmit one small
frame, and keeps receiving for ``--after-kick-s`` (does the late answer arrive right after
the KV260 transmits something else?). Around a timed-out frame it snapshots the Nano NIC
counters (sysfs) and /proc/net/snmp (Ip Reasm*, Udp) at timeout, end of linger and after
the kick: a datagram held on the Nano would show as NIC rx moving without delivery.

The client is host.kv_link.KvLinkClient (its reassembler and receive path); the frame
call is driven here so the linger/kick can happen in place. ``--poke-s`` is passed to the
client's resend/poke knob when the tree has one (fixed tree) and 0 disables it.
One json line per frame (--out), a summary line at the end. Python 3.8 compatible.
"""
import argparse
import json
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np  # noqa: E402

from host import kv_link as kv  # noqa: E402
from tools.car_geo_producer.common import base_camera, load_prep  # noqa: E402


def nic(iface):
    out = {}
    if not iface:
        return out
    base = "/sys/class/net/%s/statistics" % iface
    for name in ("rx_packets", "rx_dropped", "rx_errors", "rx_missed_errors", "rx_over_errors",
                 "rx_fifo_errors"):
        try:
            out[name] = int(open(os.path.join(base, name)).read())
        except (OSError, ValueError):
            pass
    return out


def snmp():
    s = kv.read_snmp()
    out = {}
    for grp, keys in (("Ip", ("ReasmReqds", "ReasmOKs", "ReasmFails", "ReasmTimeout")),
                      ("Udp", ("InDatagrams", "NoPorts", "InErrors", "RcvbufErrors"))):
        for k in keys:
            v = (s.get(grp) or {}).get(k)
            if v is not None:
                out["%s.%s" % (grp, k)] = v
    return out


def view(iface):
    return {"t": time.monotonic(), "nic": nic(iface), "snmp": snmp()}


def build_requests(prep, frames, shape=(32, 32, 10), voxel=0.05):
    meta, depth = load_prep(prep)
    unit = np.float32(meta["depth_unit_m"])
    K = np.asarray(meta["K"], dtype=np.float64)
    n, h, w = depth.shape
    inverse_k = np.ascontiguousarray(np.linalg.inv(K), dtype=np.float32)
    world_camera = np.ascontiguousarray(base_camera()[:3, :], dtype=np.float32)
    origin = np.array([0.05, -0.8, -0.05], dtype=np.float64)   # eq-gate frame 0 block
    reqs = []
    for i in range(min(n, frames)):
        codes = np.ascontiguousarray(depth[i], dtype="<u2").ravel()
        dcrc = kv.crc32(kv.codes_to_m(codes, unit))
        reqs.append({"codes": codes, "depth_crc": dcrc})
    geom = {"width": w, "height": h, "nx": shape[0], "ny": shape[1], "nz": shape[2],
            "unit_f32": unit, "voxel": voxel, "origin": origin, "inverse_k": inverse_k,
            "world_camera": world_camera}
    return reqs, geom


def kick(server, timeout_s=0.3):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", 0))
    session = int.from_bytes(os.urandom(8), "little")
    t = time.monotonic()
    for d in kv.split(kv.HELLO, session, 1, b"{}", kv.CHUNK_FRAGMENTED):
        s.sendto(d, server)
    s.settimeout(timeout_s)
    got = None
    try:
        s.recvfrom(65536)
        got = time.monotonic()
    except socket.timeout:
        pass
    s.close()
    return t, got


class _PacedSock(object):
    def __init__(self, sock, sendto):
        self._s, self.sendto = sock, sendto

    def __getattr__(self, name):
        return getattr(self._s, name)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--server", required=True)
    ap.add_argument("--prep", required=True)
    ap.add_argument("--frames", type=int, default=1000, help="requests to send (cycles the recording)")
    ap.add_argument("--hz", type=float, default=0.0, help="0 = back to back")
    ap.add_argument("--timeout-s", type=float, default=0.3)
    ap.add_argument("--linger-s", type=float, default=2.0)
    ap.add_argument("--kick", action="store_true")
    ap.add_argument("--after-kick-s", type=float, default=0.5)
    ap.add_argument("--poke-s", type=float, default=None,
                    help="client poke/resend knob (fixed tree); 0 = off; default: the tree's default")
    ap.add_argument("--iface", default="", help="Nano NIC facing the KV260 (sysfs counters)")
    ap.add_argument("--pace-ms", type=float, default=0.0,
                    help="host fixture only: gap between request chunks (sandbox SO_RCVBUF cap)")
    ap.add_argument("--chunk-bytes", type=int, default=kv.CHUNK_FRAGMENTED)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    host, port = a.server.rsplit(":", 1)
    server = (host, int(port))
    reqs, geom = build_requests(a.prep, 310)
    kw = {}
    if a.poke_s is not None:
        kw["poke_s"] = a.poke_s
    client = kv.KvLinkClient(server, timeout_s=a.timeout_s, chunk_bytes=a.chunk_bytes, **kw)
    if a.pace_ms > 0:
        raw_sendto = client.sock.sendto

        def paced(d, addr):
            time.sleep(a.pace_ms / 1e3)
            return raw_sendto(d, addr)
        client.sock = _PacedSock(client.sock, paced)
    info = client.hello(3.0)
    ident = {"stream_instance": "resp-probe-%d" % os.getpid(), "clock_id": "probe",
             "calibration_id": "probe", "pose_epoch": "probe"}
    out = open(a.out, "w")
    out.write(json.dumps({"start": True, "server": a.server, "info_core": info.get("core"),
                          "args": vars(a), "has_poke": hasattr(client, "poke_s"),
                          "t": time.monotonic()}) + "\n")
    period = 1.0 / a.hz if a.hz > 0 else 0.0
    next_t = time.monotonic()
    tally = {"answered": 0, "late_on_own": 0, "after_kick": 0, "never": 0}
    lat = []
    for i in range(a.frames):
        if period:
            now = time.monotonic()
            if now < next_t:
                time.sleep(next_t - now)
            next_t += period
        r = reqs[i % len(reqs)]
        fid = i + 1
        cap = time.monotonic_ns()
        body = kv.encode_request(frame_id=fid, capture_ns=cap, pose_ns=cap, identity=ident,
                                 codes=r["codes"], depth_crc=r["depth_crc"], **geom)
        expect = {"frame_id": fid, "capture_ns": cap, "pose_ns": cap, "identity": ident}
        rec = {"i": i, "frame_id": fid}
        t0 = time.monotonic()
        rec["t_send"] = t0
        try:
            resp, net = client.call(body, expect)
            rec.update(ok=True, rtt_ms=net["rtt_s"] * 1e3, first_chunk_ms=net["resp_first_chunk_s"] * 1e3,
                       kv_core_ms=resp["times_ns"]["core_total"] / 1e6, pokes=net.get("pokes", 0))
            if net.get("pokes"):
                tally["answered_after_poke"] = tally.get("answered_after_poke", 0) + 1
            tally["answered"] += 1
            lat.append(net["rtt_s"] * 1e3)
        except kv.LinkTimeout:
            rec["ok"] = False
            rec["at_timeout"] = view(a.iface)
            # linger: same socket, nothing sent
            arrived = None
            deadline = time.monotonic() + a.linger_s
            while arrived is None:
                got = client._recv_message(deadline)
                if got is None:
                    break
                mtype, _, rid, _, _ = got
                if mtype == kv.RESP and rid == fid:
                    arrived = time.monotonic()
                    client.rx.forget_done(kv.RESP)
                else:
                    rec.setdefault("linger_other", []).append([kv.TYPE_NAMES.get(mtype, mtype), rid])
            rec["linger_end"] = view(a.iface)
            if arrived is not None:
                rec["late_on_own_ms"] = (arrived - t0) * 1e3
                tally["late_on_own"] += 1
            elif a.kick:
                tk, tk_ans = kick(server)
                rec["kick_t"] = tk
                rec["kick_answered_ms"] = None if tk_ans is None else (tk_ans - tk) * 1e3
                deadline = time.monotonic() + a.after_kick_s
                while arrived is None:
                    got = client._recv_message(deadline)
                    if got is None:
                        break
                    mtype, _, rid, _, _ = got
                    if mtype == kv.RESP and rid == fid:
                        arrived = time.monotonic()
                        client.rx.forget_done(kv.RESP)
                rec["after_kick"] = view(a.iface)
                if arrived is not None:
                    rec["after_kick_ms"] = (arrived - tk) * 1e3
                    rec["late_total_ms"] = (arrived - t0) * 1e3
                    tally["after_kick"] += 1
                else:
                    tally["never"] += 1
            else:
                tally["never"] += 1
            rec["pending"] = {k: list(v) for k, v in client.rx.pending().items()}
        out.write(json.dumps(rec) + "\n")
        if not rec["ok"]:
            out.flush()
    lat.sort()
    pct = (lambda q: lat[min(len(lat) - 1, int(q * len(lat)))] if lat else None)
    out.write(json.dumps({"summary": True, "frames": a.frames, "tally": tally,
                          "rtt_ms_p50": pct(0.5), "rtt_ms_p99": pct(0.99), "rtt_ms_max": lat[-1] if lat else None,
                          "client_counts": dict(client.counts), "t": time.monotonic()}) + "\n")
    out.close()
    print(json.dumps({"tally": tally, "rtt_ms_p50": pct(0.5)}))
    client.close()


if __name__ == "__main__":
    main()
