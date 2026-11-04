#!/usr/bin/env python3
"""Summarise burst-tail board runs (docs/design/KV_LINK_BURST_TAIL_20261011.md).

  burst_tail_report.py probe <probe.jsonl> [--send-log KV.send.jsonl --session-index N]
                             [--sampler tx.jsonl]
Per probe: answered / timed out; RTT p50/p99/max; for each timed-out frame whether the
answer came while lingering (nothing sent), only after the kick (KV260 transmitted
something else), or never; Nano NIC rx and Udp/Reasm deltas between timeout -> end of
linger -> after kick. With the KV send log (records in send order; ``--session-index``
picks the N-th client session seen in it): the KV-side stamps of each timed-out frame
(request complete -> core -> sendto returned) and sysfs tx_packets around its send.
With the KV tx sampler: frames handed to the driver (Ip OutRequests - FragOKs +
FragCreates) minus frames the MAC counted (tx_packets), right after each send and after
the kick: a deficit of exactly one that clears only when the KV260 sends again = the
last frame kept back. Clocks: KV records are KV monotonic, probe records Nano monotonic;
only durations within one board are formed.
"""
import argparse
import bisect
import json


def load(path):
    return [json.loads(l) for l in open(path) if l.strip()]


def pct(xs, q):
    if not xs:
        return None
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))]


def delta(a, b, grp):
    return {k: b[grp].get(k, 0) - a[grp].get(k, 0) for k in b[grp]
            if b[grp].get(k, 0) != a[grp].get(k, 0)}


def handed(ip):
    out_req, frag_oks, frag_creates = ip
    return out_req - frag_oks + frag_creates


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("probe")
    ap.add_argument("--send-log")
    ap.add_argument("--session-index", type=int, default=0)
    ap.add_argument("--sampler")
    a = ap.parse_args()
    rows = load(a.probe)
    frames = [r for r in rows if "frame_id" in r and "ok" in r]
    summ = [r for r in rows if r.get("summary")]
    ok = [r for r in frames if r["ok"]]
    bad = [r for r in frames if not r["ok"]]
    rtt = [r["rtt_ms"] for r in ok]
    out = {"frames": len(frames), "answered_in_time": len(ok), "timed_out": len(bad),
           "rtt_ms": {"p50": pct(rtt, .5), "p99": pct(rtt, .99), "p999": pct(rtt, .999),
                      "max": max(rtt) if rtt else None},
           "answered_after_poke": sum(1 for r in ok if r.get("pokes")),
           "rtt_over_50ms": sum(1 for r in rtt if r > 50), "rtt_over_100ms": sum(1 for r in rtt if r > 100),
           "summary": summ[-1] if summ else None}
    sends = []
    if a.send_log:
        recs = [r for r in load(a.send_log) if r.get("type") == "resp"]
        order = []
        for r in recs:
            if r["session"] not in order:
                order.append(r["session"])
        sess = order[a.session_index] if a.session_index < len(order) else None
        sends = {r["frame_id"]: r for r in recs if r["session"] == sess}
        out["kv_session"] = sess
        out["kv_resp_sends"] = len(sends)
        out["kv_tx_delta_hist"] = {}
        for r in sends.values():
            if r.get("tx_before") is not None and r.get("tx_after") is not None:
                k = str(r["tx_after"] - r["tx_before"])
                out["kv_tx_delta_hist"][k] = out["kv_tx_delta_hist"].get(k, 0) + 1
    samp = []
    if a.sampler:
        samp = [r for r in load(a.sampler) if "t" in r and r.get("ip") and r.get("tx") is not None]
        st = [r["t"] for r in samp]

    def at(t):          # last sample at or before t
        i = bisect.bisect_right(st, t) - 1
        return samp[i] if i >= 0 else None
    events = []
    for r in bad:
        e = {"frame_id": r["frame_id"], "late_on_own_ms": r.get("late_on_own_ms"),
             "after_kick_ms": r.get("after_kick_ms"), "late_total_ms": r.get("late_total_ms"),
             "kick_answered_ms": r.get("kick_answered_ms")}
        if "at_timeout" in r and "linger_end" in r:
            e["nano_linger"] = {"nic": delta(r["at_timeout"], r["linger_end"], "nic"),
                                "snmp": delta(r["at_timeout"], r["linger_end"], "snmp")}
        if "after_kick" in r:
            e["nano_kick"] = {"nic": delta(r["linger_end"], r["after_kick"], "nic"),
                              "snmp": delta(r["linger_end"], r["after_kick"], "snmp")}
        s = sends.get(r["frame_id"]) if sends else None
        if s:
            e["kv_ms"] = {"recv": (s["t_recv_done"] - s["t_first_chunk"]) * 1e3,
                          "slot_wait": (s["t_take"] - s["t_recv_done"]) * 1e3,
                          "core": (s["t_core_end"] - s["t_core_begin"]) * 1e3,
                          "encode": (s["t_encoded"] - s["t_core_end"]) * 1e3,
                          "sendto": (s["t_send_end"] - s["t_send_begin"]) * 1e3,
                          "req_done_to_sent": (s["t_send_end"] - s["t_recv_done"]) * 1e3}
            e["kv_wire_frames"] = s["wire_frames"]
            if s.get("tx_before") is not None:
                e["kv_sysfs_tx_around_send"] = s["tx_after"] - s["tx_before"]
            if samp:
                b0 = at(s["t_send_begin"] - 1e-4)
                b1 = at(s["t_send_end"] + 0.020)
                b2 = at(s["t_send_end"] + 0.250)
                if b0 and b1 and b2:
                    def deficit(x):
                        return (handed(x["ip"]) - handed(b0["ip"])) - (x["tx"] - b0["tx"])
                    e["kv_deficit_20ms"] = deficit(b1)
                    e["kv_deficit_250ms"] = deficit(b2)
                    # when did the deficit clear (first sample after the send with deficit <= 0)
                    i = bisect.bisect_right(st, s["t_send_end"])
                    clear = None
                    for x in samp[i:i + 20000]:
                        if deficit(x) <= 0:
                            clear = x
                            break
                    if clear is not None:
                        e["kv_deficit_cleared_after_ms"] = (clear["t"] - s["t_send_end"]) * 1e3
                        e["kv_cleared_with_handed"] = handed(clear["ip"]) - handed(b0["ip"])
                        e["kv_eth_at_clear"] = clear.get("eth")
        events.append(e)
    out["timeouts"] = events
    kinds = {"late_on_own": 0, "after_kick": 0, "never": 0}
    for e in events:
        if e["late_on_own_ms"] is not None:
            kinds["late_on_own"] += 1
        elif e["after_kick_ms"] is not None:
            kinds["after_kick"] += 1
        else:
            kinds["never"] += 1
    out["timeout_kinds"] = kinds
    if sends:
        okd = [sends[r["frame_id"]] for r in ok if r["frame_id"] in sends]
        out["kv_ok_req_done_to_sent_ms_p50_p99"] = [pct([(s["t_send_end"] - s["t_recv_done"]) * 1e3 for s in okd], q)
                                                     for q in (.5, .99)]
    print(json.dumps(out, indent=1, sort_keys=True))


if __name__ == "__main__":
    main()
