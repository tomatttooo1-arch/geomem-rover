"""fpgapc: command-line PC end of the car link (no UI). Python 3.8+, standard library only.

Needs only this file and fpgac.py in the same folder. Configuration (JSON, default
``fpgapc.json`` next to this file):
    {"token": "<random string from the Nano's token file>", "port": 47410,
     "hosts": ["<nano>.local", "198.18.43.5"], "scan_net": "198.18.43.0/24"}

Commands:
    selftest                         loopback self check (no car needed)
    probe [--host H]                 isolation vs service-down diagnosis
    scan [--net CIDR]                find hubs on the /24 (outbound TCP only)
    listen [--duration-s N] [--out F.fpgarec] [--stats F.json]
    record --out F.fpgarec [--duration-s N]
    verify --file F.fpgarec           crc-check every recorded frame
    replay --file F.fpgarec [--rate R]
    serve-replay --file F.fpgarec [--port P]   fake Nano for UI work
    find TEXT | select ID | cancel | estop | release --epoch N | yield
    goto [--turn-deg D] [--forward-m M]   one goal relative to the car through the runner (10-17
                                     test entry: turn D first, then M along the new heading;
                                     forward 0..2 m, turn -180..180 deg; same permit/guard as find)
    teleop --v V --w W --seconds S [--hz 10]   remote control through the hub (10-18; controller only):
                                     V m/s, W rad/s for S s, then one stop. The car's teleop node clamps
                                     (0.15 m/s, 0.6 rad/s by default) and stops 0.3 s after the last command.
                                     With --nav nav2 (arbitration) the first command cancels navigation.
    teleop --keys [--v 0.15 --w 0.6] keyboard, only through pc/fpgapc_keys.py (Linux/macOS/Windows): w/s/a/d/q/e drive, space/x stop,
                                     Ctrl-C/Esc stop and quit; a held key auto-repeats, and the last key is
                                     re-sent for --hold-s (0.4 s) to bridge the auto-repeat delay.
    mapping-stop                     end the in-stack mapping (CAR_MAPPING=1): save + export the map, Nav2
                                     loads it (RESULT after up to 180 s: exported | failed)
"""

import argparse
import json
import os
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import fpgac  # noqa: E402

K = fpgac.KIND


def utf8_stdout():
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass


def load_config(args):
    path = args.config or os.path.join(HERE, "fpgapc.json")
    cfg = {}
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
    token = args.token or os.environ.get("FPGAC_TOKEN") or cfg.get("token")
    if args.token_file:
        with open(args.token_file, "r", encoding="utf-8") as f:
            token = f.read().strip()
    hosts = [args.host] if args.host else list(cfg.get("hosts") or [])
    return {"token": token, "port": args.port or cfg.get("port") or fpgac.DEFAULT_PORT,
            "hosts": hosts, "scan_net": args.net or cfg.get("scan_net"),
            "last_file": os.path.join(os.path.expanduser("~"), ".fpgapc_last.json")}


def client(cfg, **kw):
    if not cfg["token"]:
        raise SystemExit("no token: put it in fpgapc.json, --token-file or FPGAC_TOKEN")
    return fpgac.FpgacClient(cfg["port"], cfg["token"], hosts=cfg["hosts"], scan_net=cfg["scan_net"],
                           last_file=cfg["last_file"], **kw)


def summary(obj):
    def g(group, *keys):
        d = obj.get(group) or {}
        for k in keys:
            if not isinstance(d, dict):
                return None
            d = d.get(k)
        return d
    idle = ",".join("%s:%s" % (k, v["state"]) for k, v in sorted(fpgac.kv_blocks(obj).items())
                    if v["state"] not in ("live", "unknown")) or "-"
    return ("seq=%s src=%s task=%s mem=%s/%s kv_link=%s kv_age_ms=%s kv_not_live=%s estop=%s epoch=%s "
            "pose_age_ms=%s fps=%s kv_hb=%s" % (
                obj.get("seq_state"), source_label(obj.get("source")), g("task", "state"), g("memory", "n_tracks"),
                g("memory", "revision"), g("kv", "link_ok"), _r(g("kv", "age_ms")), idle,
                g("estop", "latched"), g("estop", "epoch"), _r(g("pose", "age_ms")),
                _r(g("perf", "frames_per_s")), fpgac.kv_heartbeat(obj)))


def source_label(source):
    """Frame source as shown: a simulated stream (line D1, owner 10-14) always reads SIM."""
    return "SIM" if source == "sim" else source


def _r(x):
    return None if x is None else round(x, 1)


class DisplayYield(object):
    """listen --yield-control: a display-only PC never keeps the hub's control. It yields right after
    connecting and again whenever the hub hands control back to it -- spec O8: a holder's disconnect
    hands control to the earliest connected client, i.e. back to this display PC (line D2 smoke_04: after
    `fpgapc find` exited, a later `fpgapc release` was rejected not_controller). At most one re-yield per
    min_interval_s, so two display-only PCs cannot pass control back and forth at full speed.
    on_event runs on the client's reader thread and only sets a flag; poll (main loop) sends the yield."""

    def __init__(self, min_interval_s=1.0):
        self.client = None
        self.pending = threading.Event()
        self.last = None
        self.min_interval_s = float(min_interval_s)

    def on_event(self, event):
        if (self.client is not None and isinstance(event, dict) and event.get("event") == "controller_changed"
                and event.get("controller_client_id") == self.client.client_id):
            self.pending.set()

    def poll(self, now=None):
        if not self.pending.is_set():
            return None
        now = time.monotonic() if now is None else now
        if self.last is not None and now - self.last < self.min_interval_s:
            return None
        self.pending.clear()
        self.last = now
        cid = self.client.command("yield_control", {})
        return self.client.wait_ack(cid, 6)


def cmd_listen(cfg, args, record_only=False):
    rec = fpgac.Recorder(args.out) if args.out else None
    last_print = [0.0]
    yielder = DisplayYield() if getattr(args, "yield_control", False) else None

    def on_frame(frame, obj):
        if yielder is not None and frame.kind == K["EVENT"]:
            yielder.on_event(obj.get("event"))
        if record_only:
            return
        if frame.kind == K["STATE"] and frame.stale:
            return                      # recorded and counted, never shown as current
        if frame.kind == K["STATE"] and time.monotonic() - last_print[0] >= 1.0:
            last_print[0] = time.monotonic()
            print(summary(obj))
        elif frame.kind == K["EVENT"]:
            print("EVENT", json.dumps(obj.get("event"), ensure_ascii=False)[:200])

    def on_link(state, detail):
        print("LINK", state, json.dumps(detail, ensure_ascii=False))

    c = client(cfg, on_frame=on_frame, on_link=on_link, recorder=rec).start()
    if yielder is not None:
        yielder.client = c
        # display-only PC: hand the hub's control back at once, so that a later command PC
        # (fpgapc find / select / cancel / release) becomes the holder; any PC may still e-stop
        if c.connected.wait(15):
            cid = c.command("yield_control", {})
            print("YIELD", json.dumps(c.wait_ack(cid, 6), ensure_ascii=False))
    t0 = time.monotonic()
    try:
        while args.duration_s <= 0 or time.monotonic() - t0 < args.duration_s:
            time.sleep(0.2)
            ack = yielder.poll() if yielder is not None else None
            if ack is not None:     # control was handed back to this display-only PC: given up again
                print("YIELD", json.dumps(ack, ensure_ascii=False), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        c.stop()
        if rec:
            rec.close()
    stats = c.stats()
    stats["wall_s"] = time.monotonic() - t0
    if rec:
        stats["recorded_frames"] = rec.n
    print(json.dumps(stats, ensure_ascii=False))
    if args.stats:
        with open(args.stats, "x", encoding="utf-8") as f:
            json.dump(stats, f, ensure_ascii=False, indent=1)
    return 0


def cmd_command(cfg, args, op, cmd_args):
    c = client(cfg).start()
    if not c.connected.wait(15):
        c.stop()
        print(json.dumps({"error": "not connected", "stats": c.stats()}))
        return 2
    cid = c.command(op, cmd_args)
    ack = c.wait_ack(cid, 30 if op == "estop" else 6)
    print("ACK", json.dumps(ack, ensure_ascii=False))
    wait = 185 if op == "mapping_stop" else 4
    res = c.wait_result(cid, wait) if ack is not None and ack.get("status") != "rejected" else None
    if res is not None:
        print("RESULT", json.dumps(res, ensure_ascii=False))
    c.stop()
    return 0 if ack is not None and ack.get("status") != "rejected" else 1


def synthetic_state(i, source="replay"):
    s = {"v": 1, "seq_state": i, "hub_boot_id": "selftest", "nano_mono_ns": i * 100000000,
         "source": source, "replay_id": "selftest", "state_hz": 10}
    for g in fpgac.STATE_GROUPS:
        s[g] = {"src": source, "age_ms": 1.0, "ok": True, "reason": None}
    s["task"]["state"] = "searching"
    s["memory"].update({"revision": i, "n_tracks": 1})
    return s


def cmd_selftest(args):
    """Write a small .fpgarec, serve it over loopback, receive it, record it, verify it."""
    tmp = tempfile.mkdtemp(prefix="fpgapc-selftest-")
    src = os.path.join(tmp, "source.fpgarec")
    rec = fpgac.Recorder(src)
    rec.hello({"v": 1, "ok": True, "hub_boot_id": "selftest"}, "selftest")
    for i in range(1, 31):
        rec.write(i * 20000000, fpgac.encode_json(K["STATE"], i, synthetic_state(i, "live")))
    rec.close()
    server = fpgac.ReplayServer(src, "selftest-token", rate=1.0, loop=False).start()
    got = []
    out = os.path.join(tmp, "received.fpgarec")
    rec2 = fpgac.Recorder(out)
    c = fpgac.FpgacClient(server.port, "selftest-token", hosts=["127.0.0.1"], recorder=rec2,
                        on_frame=lambda f, o: got.append(o) if f.kind == K["STATE"] else None).start()
    t0 = time.monotonic()
    while len(got) < 30 and time.monotonic() - t0 < 10:
        time.sleep(0.05)
    c.stop()
    rec2.close()
    server.close()
    check = fpgac.verify_fpgarec(out)
    import shutil
    shutil.rmtree(tmp, ignore_errors=True)
    ok = (len(got) == 30 and all(o["source"] == "replay" and o["orig_source"] == "live" for o in got)
          and check["kinds"].get("STATE") == 30)
    print("SELFTEST %s frames=%d recorded=%d python=%s platform=%s keepalive=%s" % (
        "OK" if ok else "FAIL", len(got), check["frames"], sys.version.split()[0], sys.platform,
        _keepalive_kind()))
    return 0 if ok else 1


def _keepalive_kind():
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        return fpgac.set_keepalive(s)
    finally:
        s.close()


TELEOP_KEYS = {"w": (1, 0), "s": (-1, 0), "a": (0, 1), "d": (0, -1), "q": (1, 1), "e": (1, -1),
               " ": (0, 0), "x": (0, 0)}       # tools/car_run/teleop_keys.py


def cmd_teleop(cfg, args):
    """Remote control through the hub's teleop op (no result; one ACK per command, printed when rejected)."""
    c = client(cfg).start()
    if not c.connected.wait(15):
        c.stop()
        print(json.dumps({"error": "not connected", "stats": c.stats()}))
        return 2
    sent, rejected = [0], []

    def send(body):
        cid = c.command("teleop", body)
        sent[0] += 1
        return cid

    def check(cid):
        ack = c.wait_ack(cid, 2)
        if ack is None or ack.get("status") == "rejected":
            rejected.append(ack)
            print("ACK", json.dumps(ack, ensure_ascii=False))
            return False
        return True

    rc = 0
    try:
        if args.keys:
            rc = KEYS_LOOP(send, check, args)
        else:
            period, end = 1.0 / args.hz, time.monotonic() + args.seconds
            first = send({"v": args.v, "w": args.w})
            if not check(first):
                rc = 1
            while rc == 0 and time.monotonic() < end:
                time.sleep(period)
                send({"v": args.v, "w": args.w})
    finally:
        check(send({"stop": True}))
        print(json.dumps({"teleop_sent": sent[0], "rejected": len(rejected)}))
        c.stop()
    return rc or (1 if rejected else 0)


# Keyboard mode (teleop --keys) reads raw keys, which needs a POSIX terminal or msvcrt -- outside this file's
# Windows-stdlib-only rule. pc/fpgapc_keys.py sets KEYS_LOOP and then runs main(); plain fpgapc refuses --keys.
KEYS_LOOP = None


def main(argv=None):
    utf8_stdout()
    p = argparse.ArgumentParser(description="FPGA car link, PC command line (no UI)")
    p.add_argument("--config")
    p.add_argument("--host")
    p.add_argument("--port", type=int)
    p.add_argument("--token")
    p.add_argument("--token-file")
    p.add_argument("--net")
    sub = p.add_subparsers(dest="cmd")
    sub.required = True
    sub.add_parser("selftest")
    sub.add_parser("probe")
    sub.add_parser("scan")
    for name in ("listen", "record"):
        q = sub.add_parser(name)
        q.add_argument("--out", required=(name == "record"))
        q.add_argument("--duration-s", type=float, default=0.0)
        q.add_argument("--stats")
        q.add_argument("--yield-control", action="store_true",
                       help="display only: give up the hub's control right after connecting, and again "
                            "whenever it is handed back")
    q = sub.add_parser("verify")
    q.add_argument("--file", required=True)
    q = sub.add_parser("replay")
    q.add_argument("--file", required=True)
    q.add_argument("--rate", type=float, default=1.0)
    q = sub.add_parser("serve-replay")
    q.add_argument("--file", required=True)
    q.add_argument("--rate", type=float, default=1.0)
    q.add_argument("--bind", default="127.0.0.1")
    q = sub.add_parser("find")
    q.add_argument("text")
    q = sub.add_parser("select")
    q.add_argument("candidate_id")
    sub.add_parser("cancel")
    sub.add_parser("estop")
    q = sub.add_parser("release")
    q.add_argument("--epoch", type=int, required=True)
    sub.add_parser("yield")
    q = sub.add_parser("goto")
    q.add_argument("--forward-m", type=float, default=0.0)
    q.add_argument("--turn-deg", type=float, default=0.0)
    q = sub.add_parser("teleop")
    q.add_argument("--keys", action="store_true")
    q.add_argument("--v", type=float, default=0.15)
    q.add_argument("--w", type=float, default=0.0)
    q.add_argument("--seconds", type=float, default=1.0)
    q.add_argument("--hz", type=float, default=10.0)
    q.add_argument("--hold-s", type=float, default=0.4)
    sub.add_parser("mapping-stop")
    args = p.parse_args(argv)
    if args.cmd == "teleop" and args.keys and KEYS_LOOP is None:
        p.error("teleop --keys: run pc/fpgapc_keys.py (same arguments; it reads the keyboard)")
    if args.cmd == "teleop" and not args.keys and not (0 < args.seconds <= 30 and 4 <= args.hz <= 50):
        p.error("teleop: --seconds within (0, 30], --hz within [4, 50] (the car's dead man is 0.3 s)")
    if args.cmd == "selftest":
        return cmd_selftest(args)
    if args.cmd == "verify":
        print(json.dumps(fpgac.verify_fpgarec(args.file)))
        return 0
    if args.cmd == "replay":
        for frame, obj in fpgac.replay_frames(args.file, args.rate):
            line = summary(obj) if frame.kind == K["STATE"] else json.dumps(obj, ensure_ascii=False)[:200]
            print(frame.name, line)
        return 0
    cfg = load_config(args)
    if args.cmd == "serve-replay":
        server = fpgac.ReplayServer(args.file, cfg["token"] or "replay", host=args.bind,
                                   port=cfg["port"], rate=args.rate).start()
        print("serving %s on %s:%d (Ctrl+C to stop)" % (args.file, args.bind, server.port))
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            server.close()
        return 0
    if args.cmd == "probe":
        hosts = cfg["hosts"] or [None]
        for h in hosts:
            if h is None:
                print(json.dumps({"error": "no host: give --host or hosts in fpgapc.json"}))
                return 2
            print(json.dumps(fpgac.probe(h, cfg["port"], cfg["token"])))
        return 0
    if args.cmd == "scan":
        found = fpgac.scan(cfg["port"], cfg["token"], cfg["scan_net"])
        print(json.dumps({"net": cfg["scan_net"] or "local /24", "found": found}))
        return 0 if found else 1
    if args.cmd == "listen":
        return cmd_listen(cfg, args)
    if args.cmd == "record":
        return cmd_listen(cfg, args, record_only=True)
    if args.cmd == "find":
        return cmd_command(cfg, args, "find", {"text": args.text})
    if args.cmd == "select":
        return cmd_command(cfg, args, "select", {"candidate_id": args.candidate_id})
    if args.cmd == "cancel":
        return cmd_command(cfg, args, "cancel", {})
    if args.cmd == "estop":
        return cmd_command(cfg, args, "estop", {})
    if args.cmd == "release":
        return cmd_command(cfg, args, "release", {"epoch": args.epoch})
    if args.cmd == "goto":
        import math
        return cmd_command(cfg, args, "goto", {"forward_m": args.forward_m, "turn_rad": math.radians(args.turn_deg)})
    if args.cmd == "yield":
        return cmd_command(cfg, args, "yield_control", {})
    if args.cmd == "teleop":
        if args.keys:
            args.w = args.w or 0.6
        return cmd_teleop(cfg, args)
    if args.cmd == "mapping-stop":
        return cmd_command(cfg, args, "mapping_stop", {})
    return 2


if __name__ == "__main__":
    sys.exit(main())
