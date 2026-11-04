#!/usr/bin/env python3
"""Local TX-path sampler for the HELLO investigation (runs on the KV260, writes only
to a local file, never to the network: reading it over ssh would itself transmit).

Every ``--period-ms`` it reads, and logs a line only when something changed:
  * /proc/net/snmp Ip.OutRequests, Ip.FragOKs, Ip.FragCreates (kernel IP layer:
    datagrams/fragments handed down),
  * /sys/class/net/<iface>/statistics/tx_packets (on the Cadence GEM this comes from
    the MAC's own frame counter = frames that left the MAC),
  * /sys/class/net/<iface>/queues/tx-*/byte_queue_limits/inflight (bytes handed to
    the driver and not yet completed, when the driver keeps BQL),
  * the interrupt count of the NIC line in /proc/interrupts.
A fragment handed down but not leaving the MAC until some later transmission shows
as FragCreates moving now and tx_packets catching up only later (inflight > 0
meanwhile). Python 3.8 compatible; stdlib only.
"""
import argparse
import glob
import json
import os
import time


def snmp_ip():
    lines = open("/proc/net/snmp").read().split("\n")
    for head, vals in zip(lines[0::2], lines[1::2]):
        if head.startswith("Ip:"):
            d = dict(zip(head.split()[1:], (int(v) for v in vals.split()[1:])))
            return (d.get("OutRequests"), d.get("FragOKs"), d.get("FragCreates"))
    return (None, None, None)


def read_int(path):
    try:
        return int(open(path).read())
    except (OSError, ValueError):
        return None


def irq_count(pattern):
    if not pattern:
        return None
    try:
        for line in open("/proc/interrupts"):
            if pattern in line:
                parts = line.split()
                return sum(int(p) for p in parts[1:] if p.isdigit())
    except OSError:
        pass
    return None


def ethtool_tx(iface):
    import subprocess
    try:
        out = subprocess.run(["ethtool", "-S", iface], stdout=subprocess.PIPE,
                             stderr=subprocess.DEVNULL, timeout=2).stdout.decode()
    except (OSError, subprocess.SubprocessError):
        return None
    res = []
    for line in out.splitlines():
        k, _, v = line.partition(":")
        k = k.strip()
        if k in ("tx_frames", "q0_tx_packets", "q1_tx_packets"):
            res.append(int(v))
    return tuple(res)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--irq", default="", help="substring of the NIC line in /proc/interrupts")
    ap.add_argument("--period-ms", type=float, default=1.0)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--ethtool-every", type=int, default=0,
                    help="also read `ethtool -S` (GEM: MAC tx_frames, per-queue completions) every N samples")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    inflight = sorted(glob.glob("/sys/class/net/%s/queues/tx-*/byte_queue_limits/inflight" % a.iface))
    txp = "/sys/class/net/%s/statistics/tx_packets" % a.iface
    end = time.monotonic() + a.seconds
    last = None
    n = 0
    with open(a.out, "w") as f:
        f.write(json.dumps({"start_unix": time.time(), "start_mono": time.monotonic(),
                            "inflight_files": inflight, "pid": os.getpid()}) + "\n")
        k = 0
        eth = None
        while time.monotonic() < end:
            if a.ethtool_every and k % a.ethtool_every == 0:
                eth = ethtool_tx(a.iface)
            k += 1
            cur = (snmp_ip(), read_int(txp), tuple(read_int(p) for p in inflight), irq_count(a.irq), eth)
            if cur != last:
                f.write(json.dumps({"t": round(time.monotonic(), 6), "ip": cur[0], "tx": cur[1],
                                    "inflight": cur[2], "irq": cur[3], "eth": cur[4]}) + "\n")
                last = cur
                n += 1
                if n % 200 == 0:
                    f.flush()
            time.sleep(a.period_ms / 1e3)
    return 0


if __name__ == "__main__":
    main()
