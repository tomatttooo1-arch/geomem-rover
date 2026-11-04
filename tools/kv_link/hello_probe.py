#!/usr/bin/env python3
"""New-session HELLO probe (2026-10-11, docs/design/KV_LINK_HELLO_FIX_20261011.md).

Each trial replays what ``KvGeometry.connect`` does on the wire -- main client
HELLO, then a second (telemetry) client HELLO + one TELEM_REQ -- from fresh
sockets, and records per step: the local port, whether/when the answer came,
and the deltas of /proc/net/snmp (Udp NoPorts, Ip Reasm*) and of the Nano NIC
statistics around the trial.

The discriminating part: a step that times out does NOT close its socket. It
keeps receiving for ``--linger-s`` without sending anything (does the answer
arrive late on its own?), then optionally sends a ``--kick`` (one TELEM_REQ
from another socket, which makes the KV260 transmit one small frame) and keeps
receiving for ``--after-kick-s`` (does the late answer arrive right after the
KV260 transmits something else?). Late arrival = the answer was delayed, not
misaddressed; arrival only after the kick = it waited for the next transmission.

``--mode single`` does only the main HELLO (one socket per trial).
Sends the legacy empty HELLO body and one attempt per step (no resend), so it
runs unchanged against old and new servers. Python 3.8 compatible.
"""
import argparse
import json
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from host import kv_link as kv  # noqa: E402


def nic_stats(iface):
    if not iface:
        return {}
    base = "/sys/class/net/%s/statistics" % iface
    out = {}
    for name in ("rx_packets", "rx_dropped", "rx_errors", "rx_missed_errors", "rx_over_errors",
                 "rx_fifo_errors", "tx_packets"):
        try:
            out[name] = int(open(os.path.join(base, name)).read())
        except (OSError, ValueError):
            pass
    return out


def ethtool_stats(iface):
    """``ethtool -S`` counters with rx/tx in the name (the NIC's own view), {} when unavailable."""
    if not iface:
        return {}
    import subprocess
    try:
        out = subprocess.run(["ethtool", "-S", iface], stdout=subprocess.PIPE,
                             stderr=subprocess.DEVNULL, timeout=2).stdout.decode()
    except (OSError, subprocess.SubprocessError):
        return {}
    res = {}
    for line in out.splitlines():
        if ":" in line:
            k, v = line.split(":", 1)
            k, v = k.strip(), v.strip()
            if v.isdigit() and ("rx" in k or "tx" in k):
                res[k] = int(v)
    return res


def nic_view(iface):
    return {"sys": nic_stats(iface), "ethtool": ethtool_stats(iface)}


def snmp_flat():
    s = kv.read_snmp()
    out = {}
    for grp in ("Ip", "Udp"):
        for k, v in (s.get(grp) or {}).items():
            out["%s.%s" % (grp, k)] = v
    return out


def delta(a, b):
    return {k: b[k] - a[k] for k in b if k in a and b[k] != a[k]}


class Step(object):
    def __init__(self, server, name, msg_type, ack_type, body):
        self.server, self.name = server, name
        self.msg_type, self.ack_type, self.body = msg_type, ack_type, body
        self.client = kv.KvLinkClient(server, rcvbuf=1 << 20)
        self.port = self.client.sock.getsockname()[1]
        self.seq = None
        self.t_send = None

    def send(self, client=None):
        c = client or self.client
        c._ctrl_seq += 1
        self.seq = c._ctrl_seq
        self.t_send = time.monotonic()
        c._send(self.msg_type, self.seq, self.body)

    def wait(self, until, client=None):
        """Seconds after send at which the matching answer completed, or None."""
        c = client or self.client
        while True:
            got = c._recv_message(until)
            if got is None:
                return None
            mtype, _, frame_id, payload, _ = got
            if mtype == self.ack_type and frame_id == self.seq:
                self.answer_bytes = len(payload)
                return time.monotonic() - self.t_send


IFACE = [""]


def run_step(server, step, timeout_s, linger_s, kick, after_kick_s, rec, client=None):
    step.send(client)
    t = step.wait(step.t_send + timeout_s, client)
    r = {"port": step.port, "ok": t is not None, "ms": None if t is None else round(t * 1e3, 3),
         "answer_bytes": getattr(step, "answer_bytes", None)}
    if t is None:
        snap = snmp_flat()
        r["nic_at_timeout"] = nic_view(IFACE[0])
        late = step.wait(time.monotonic() + linger_s, client) if linger_s > 0 else None
        r["late_ms"] = None if late is None else round(late * 1e3, 3)
        r["snmp_during_linger"] = delta(snap, snmp_flat())
        if late is None and kick:
            r["nic_before_kick"] = nic_view(IFACE[0])
            k = kv.KvLinkClient(server, rcvbuf=1 << 20)
            t_k = time.monotonic()
            k._send(kv.TELEM_REQ, 1, kv.encode_telem_req(kv.blocks_mask(("estop",)), 0))
            after = step.wait(time.monotonic() + after_kick_s, client)
            r["kick_port"] = k.sock.getsockname()[1]
            r["after_kick_ms"] = None if after is None else round(after * 1e3, 3)
            r["after_kick_since_kick_ms"] = (None if after is None else
                                             round((step.t_send + after - t_k) * 1e3, 3))
            k.close()
            r["nic_after_kick"] = nic_view(IFACE[0])
    rec[step.name] = r
    return t is not None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--server", required=True, help="host:port of kv_server")
    ap.add_argument("--trials", type=int, default=25)
    ap.add_argument("--mode", choices=("flow", "single"), default="flow")
    ap.add_argument("--hello-timeout-s", type=float, default=0.5)
    ap.add_argument("--telem-timeout-s", type=float, default=0.2)
    ap.add_argument("--linger-s", type=float, default=2.0)
    ap.add_argument("--kick", action="store_true")
    ap.add_argument("--after-kick-s", type=float, default=1.0)
    ap.add_argument("--gap-s", type=float, default=0.2)
    ap.add_argument("--iface", default="", help="NIC whose /sys statistics are recorded")
    ap.add_argument("--out", required=True, help="jsonl")
    a = ap.parse_args(argv)
    IFACE[0] = a.iface
    host, port = a.server.rsplit(":", 1)
    server = (host, int(port))
    summary = {"trials": 0, "fail_steps": {}, "fail_trials": 0, "noports_total": 0}
    with open(a.out, "w") as f:
        for i in range(a.trials):
            rec = {"trial": i, "t_unix": time.time()}
            s0, n0 = snmp_flat(), nic_stats(a.iface)
            steps = [Step(server, "main_hello", kv.HELLO, kv.HELLO_ACK, b"{}")]
            ok = run_step(server, steps[0], a.hello_timeout_s, a.linger_s, a.kick, a.after_kick_s, rec)
            if ok and a.mode == "flow":
                tel = Step(server, "telem_hello", kv.HELLO, kv.HELLO_ACK, b"{}")
                steps.append(tel)
                ok = run_step(server, tel, a.hello_timeout_s, a.linger_s, a.kick, a.after_kick_s, rec)
                if ok:
                    tq = Step.__new__(Step)
                    tq.server, tq.name = server, "telem_poll"
                    tq.msg_type, tq.ack_type = kv.TELEM_REQ, kv.TELEM_ACK
                    tq.body = kv.encode_telem_req(kv.blocks_mask(("board",)), kv.now_ns())
                    tq.client, tq.port = tel.client, tel.port
                    run_step(server, tq, a.telem_timeout_s, a.linger_s, a.kick, a.after_kick_s,
                             rec, client=tel.client)
            for st in steps:
                st.client.close()
            time.sleep(a.gap_s)        # the window runs to the next trial: late arrivals count here
            rec["snmp"] = delta(s0, snmp_flat())
            rec["nic"] = delta(n0, nic_stats(a.iface))
            f.write(json.dumps(rec, sort_keys=True) + "\n")
            f.flush()
            summary["trials"] += 1
            bad = [k for k, v in rec.items() if isinstance(v, dict) and v.get("ok") is False]
            for k in bad:
                summary["fail_steps"][k] = summary["fail_steps"].get(k, 0) + 1
            summary["fail_trials"] += 1 if bad else 0
            summary["noports_total"] += rec["snmp"].get("Udp.NoPorts", 0)
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
