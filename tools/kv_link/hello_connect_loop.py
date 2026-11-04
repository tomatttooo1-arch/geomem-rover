#!/usr/bin/env python3
"""New-session handshake loop: ``--trials`` fresh ``KvGeometry.connect()`` calls (main HELLO,
roi/feature checks, BUILD_ID via TELEM when ``--expect-build-id``), each from new sockets,
then close. Writes one JSON line per trial (ok, reason, ms, client counters incl. HELLO
resends) and prints a summary with the latency distribution and the /proc/net/snmp delta
(Udp NoPorts). docs/design/KV_LINK_HELLO_FIX_20261011.md. Python 3.8 compatible."""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from host import kv_link as kv  # noqa: E402
from host.kv_geometry import KvGeometry  # noqa: E402


def pct(xs, q):
    xs = sorted(xs)
    return None if not xs else xs[min(len(xs) - 1, int(round(q / 100.0 * (len(xs) - 1))))]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--server", required=True)
    ap.add_argument("--trials", type=int, default=100)
    ap.add_argument("--hello-timeout-s", type=float, default=0.5)
    ap.add_argument("--expect-build-id", default=None, help="e.g. 0x0117bc0a")
    ap.add_argument("--gap-s", type=float, default=0.2)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    host, port = a.server.rsplit(":", 1)
    bid = None if a.expect_build_id is None else int(a.expect_build_id, 0)
    s0 = kv.read_snmp()
    ms, reasons, resends = [], {}, 0
    with open(a.out, "w") as f:
        for i in range(a.trials):
            g = KvGeometry((host, int(port)), depth_unit_m=2.0 ** -10,
                           hello_timeout_s=a.hello_timeout_s, retry_s=0.0, expect_build_id=bid)
            t = time.monotonic()
            ok = g.connect()
            dt = (time.monotonic() - t) * 1e3
            counts = dict(g.client.counts) if g.client is not None else {}
            g.close()
            reasons[g.reason or "ok"] = reasons.get(g.reason or "ok", 0) + 1
            resends += counts.get("resend:hello", 0)
            if ok:
                ms.append(dt)
            f.write(json.dumps({"trial": i, "ok": ok, "reason": g.reason, "ms": round(dt, 3),
                                "build_id": g.build_id, "client_counts": counts}, sort_keys=True) + "\n")
            f.flush()
            time.sleep(a.gap_s)
    out = {"trials": a.trials, "reasons": reasons, "hello_resends": resends,
           "ms": {"min": pct(ms, 0), "p50": pct(ms, 50), "p90": pct(ms, 90), "p99": pct(ms, 99),
                  "max": pct(ms, 100)},
           "snmp_delta": kv.snmp_delta(s0, kv.read_snmp()),
           "noports": (kv.read_snmp().get("Udp", {}).get("NoPorts", 0) - s0.get("Udp", {}).get("NoPorts", 0))}
    print(json.dumps(out, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
