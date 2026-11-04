"""Nano hub TCP service for PCs (spec C.3) + KV loops + a no-ROS replay feeder.

Threads: accept, one reader + one sender per client, a state ticker at state_hz, and
(optionally) the KV telemetry, KV e-stop and clock-sync loops. All HubCore calls hold
``server.lock``; blocking KV calls are made outside it.

Back-pressure (C.3): per client, STATE/MEMORY/MAP(rolling)/MAP(global)/PREVIEW each keep
only the newest copy (overwrites counted ``superseded:<kind>``); EVENT is a bounded queue
(256, oldest dropped); CMD_ACK/CMD_RESULT/PONG are never dropped -- if that queue overflows
the client is disconnected. SO_SNDBUF is requested at the kernel minimum (controller ruling
10-11, replacing the spec's 64 KiB: Linux 5.4 clamps it to 4608 B effective; the value read
back is reported as ``effective_sndbuf`` in HELLO_ACK, STATE ``hub`` and the exit stats) so a
stalled hotspot backs up into the newest-only slots, not into the kernel, and at most ~3
old STATE frames reach the PC after a stall. A send blocked for 5 s disconnects the client.

Run without ROS (replay data, e.g. the Nano bench test):
    python3 -m host.hub.hub_server --data-source replay --token-file T --feed synthetic
"""

import argparse
import collections
import ipaddress
import json
import os
import socket
import subprocess
import sys
import threading
import time

from pc import fpgac
from host import fpga_clock
from host.hub import hub_profile
from host.hub.hub_core import HubCore

K = fpgac.KIND
LATEST_ORDER = ("STATE", "MEMORY", "MAP:rolling", "MAP:global", "MAP:belief", "PREVIEW")   # belief: 10-26 heat map
# Requested SO_SNDBUF: 1 byte -> the kernel's floor (SOCK_MIN_SNDBUF, 4608 B effective on
# Linux 5.4). None leaves the kernel default/autotuning (the positive-control variant).
SNDBUF_MIN_REQUEST = 1


class Session(object):
    def __init__(self, server, sock, addr, counts):
        self.server, self.addr = server, addr
        self.conn = fpgac.Connection(sock, counts, send_timeout_s=server.send_timeout_s)
        self.client_id = None
        self.order = None
        self.probe = False
        self.want = set()
        self.slots = collections.OrderedDict()
        self.fifo = collections.deque()
        self.events = collections.deque()
        self.acks = collections.deque()
        self.superseded = collections.Counter()
        self.cv = threading.Condition()
        self.alive = True
        self.handshaked = False
        self.sent = collections.Counter()
        self.blocked_ns = 0

    # producers --------------------------------------------------------------------------
    def offer_latest(self, key, kind, body=None, meta=None, payload=b""):
        with self.cv:
            if not self.server.latest_only:          # positive-control variant: queue everything
                self.fifo.append((kind, body, meta, payload))
                self.cv.notify()
                return
            if key in self.slots:
                self.superseded[key] += 1
            self.slots[key] = (kind, body, meta, payload)
            self.cv.notify()

    def offer_event(self, body):
        with self.cv:
            if len(self.events) >= 256:
                self.events.popleft()
                self.superseded["EVENT_dropped"] += 1
            self.events.append(body)
            self.cv.notify()

    def offer_ack(self, kind, obj):
        with self.cv:
            if len(self.acks) >= 256:
                self.server.counts["ack_overflow_disconnect"] += 1
                self.alive = False
                self.cv.notify()
                return
            self.acks.append((kind, obj))
            self.cv.notify()

    # sender ---------------------------------------------------------------------------
    def _next(self):
        if self.acks:
            kind, obj = self.acks.popleft()
            return kind, fpgac.dumps(obj), None, b""
        if self.events:
            return K["EVENT"], self.events.popleft(), None, b""
        for key in LATEST_ORDER:
            if key in self.slots:
                return self.slots.pop(key)
        if self.fifo:
            return self.fifo.popleft()
        return None

    def send_loop(self):
        try:
            while True:
                with self.cv:
                    while self.alive and not (self.acks or self.events or self.slots or self.fifo):
                        self.cv.wait(0.5)
                    if not self.alive:
                        return
                    item = self._next()
                kind, body, meta, payload = item
                t0 = time.monotonic_ns()
                if meta is not None:
                    self.conn.send(kind, meta=meta, payload=payload)
                else:
                    self.conn.send(kind, body=body)
                self.blocked_ns = max(self.blocked_ns, time.monotonic_ns() - t0)
                self.sent[fpgac.KIND_NAME.get(kind, "?")] += 1
        except (OSError, fpgac.ProtocolError) as exc:
            self.server.counts["send_error:" + type(exc).__name__] += 1
        finally:
            self.close()

    def close(self):
        with self.cv:
            self.alive = False
            self.cv.notify_all()
        self.conn.close()


class HubServer(object):
    def __init__(self, core, token, port=fpgac.DEFAULT_PORT, host="0.0.0.0",
                 deny_nets=("198.51.100.0/24",), max_clients=4, sndbuf=SNDBUF_MIN_REQUEST,
                 clock=fpga_clock.now_ns, idle_timeout_s=10.0, hello_timeout_s=3.0,
                 send_timeout_s=5.0, check_local=True, latest_only=True, features=()):
        self.core, self.token = core, token
        self.lock = threading.RLock()
        self.counts = core.counts
        self.clock = clock
        self.deny_nets = [ipaddress.ip_network(n, strict=False) for n in deny_nets]
        self.max_clients, self.sndbuf = max_clients, sndbuf
        self.idle_timeout_s, self.hello_timeout_s = idle_timeout_s, hello_timeout_s
        self.send_timeout_s = send_timeout_s
        self.check_local, self.latest_only = check_local, latest_only
        self.features = list(features)
        self.sessions = []
        self.controller = None
        self.order = 0
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind((host, port))
        self.listener.listen(8)
        self.port = self.listener.getsockname()[1]
        self._stop = threading.Event()
        self.threads = []
        self.state_hz = core.state_hz
        self.tick_times = collections.deque(maxlen=4096)
        self.effective_sndbuf = None
        self._cpu = (time.monotonic(), sum(os.times()[:2]))

    # lifecycle --------------------------------------------------------------------------
    def _thread(self, target, name, *args):
        t = threading.Thread(target=target, args=args, name=name, daemon=True)
        t.start()
        self.threads.append(t)
        return t

    def start(self):
        self._thread(self._accept_loop, "hub-accept")
        self._thread(self._tick_loop, "hub-tick")
        return self

    def close(self):
        self._stop.set()
        try:
            self.listener.close()
        except OSError:
            pass
        for s in list(self.sessions):
            s.close()
        for t in list(self.threads):
            t.join(timeout=5)

    # accept / handshake ------------------------------------------------------------------
    def _accept_loop(self):
        self.listener.settimeout(0.2)
        while not self._stop.is_set():
            try:
                sock, addr = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            local = sock.getsockname()[0]
            if self.check_local and any(ipaddress.ip_address(local) in n for n in self.deny_nets):
                self.counts["denied_local_net"] += 1
                sock.close()
                continue
            with self.lock:
                if len(self.sessions) >= self.max_clients:
                    self.counts["too_many_clients"] += 1
                    sock.close()
                    continue
                if self.sndbuf is not None:
                    self.effective_sndbuf = limit_sndbuf(sock, self.sndbuf)
                else:
                    self.effective_sndbuf = sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
                try:
                    fpgac.set_keepalive(sock)
                except OSError:
                    self.counts["keepalive_unavailable"] += 1
                s = Session(self, sock, addr, self.counts)
                self.sessions.append(s)
            self._thread(self._reader, "hub-reader", s)

    def _hello(self, s, frame):
        hello = frame.obj()
        ok, reason = True, None
        if frame.ver[0] != fpgac.VERSION[0]:
            ok, reason = False, "version"
        elif not isinstance(hello.get("token"), str) or not _same(hello.get("token"), self.token):
            ok, reason = False, "bad_token"
        elif not isinstance(hello.get("client_id"), str) or not hello.get("client_id"):
            ok, reason = False, "bad_client_id"
        now = self.clock()
        with self.lock:
            if ok:
                s.client_id = hello["client_id"]
                s.probe = bool(hello.get("probe"))
                s.want = set(hello.get("want") or ())
                self.order += 1
                s.order = self.order
                if self.controller is None and not s.probe:
                    self.controller = s
                    self.counts["controller_assigned"] += 1
                s.handshaked = True
            ack = {"v": 1, "ok": ok, "reason": reason, "hub_boot_id": self.core.hub_boot_id,
                   "nano_mono_ns": now, "nano_clock": fpga_clock.CLOCK_ID, "proto": list(fpgac.VERSION), "state_hz": self.core.state_hz,
                   "data_source": self.core.data_source, "estop": self.core.estop.snapshot(now),
                   "features": self.features, "controller": self.controller is s,
                   "effective_sndbuf": self.effective_sndbuf, "latest_only": self.latest_only}
        if not ok:
            self.counts["hello_rejected:" + reason] += 1
            s.conn.send(K["HELLO_ACK"], ack)
            s.conn.send(K["BYE"], {"v": 1, "reason": reason})
            return False
        s.conn.send(K["HELLO_ACK"], ack)
        if not s.probe:
            self._snapshot_to(s, now)
        self._thread(s.send_loop, "hub-sender", )  # noqa
        return True

    def _memory_bytes(self, mem):
        """Encoded MEMORY or None. A body that cannot be encoded loses only itself (counted): the
        STATE of the same tick still goes out (feed_memory already drops non-finite tracks; this is
        the backstop for anything it misses)."""
        if mem is None:
            return None
        try:
            return fpgac.dumps(mem)
        except (ValueError, TypeError) as exc:
            self.counts["memory_unsendable:" + type(exc).__name__] += 1
            return None

    def _snapshot_to(self, s, now):
        """Right after HELLO_ACK: full MEMORY + newest STATE (+ maps) without waiting (C.3)."""
        with self.lock:
            mem = self.core.memory_body(now, force=True)
            state = self.core.compose_state(now, force_runtime_info=True)   # v5: field not every tick
            maps = self.core.map_frames(now, force=True)
        mem = self._memory_bytes(mem)
        if mem is not None:
            s.offer_latest("MEMORY", K["MEMORY"], mem)
        s.offer_latest("STATE", K["STATE"], fpgac.dumps(state))
        for meta, payload in maps:
            s.offer_latest("MAP:" + meta["which"], K["MAP"], meta=meta, payload=payload)

    def _reader(self, s):
        try:
            frame = s.conn.recv(self.hello_timeout_s)
            if frame is None or frame.kind != K["HELLO"]:
                self.counts["no_hello"] += 1
                return
            if not self._hello(s, frame):
                return
            last_rx = time.monotonic()
            while not self._stop.is_set() and s.alive:
                frame = s.conn.recv(0.5)
                if frame is None:
                    if time.monotonic() - last_rx > self.idle_timeout_s:
                        self.counts["client_idle_timeout"] += 1
                        return
                    continue
                last_rx = time.monotonic()
                if frame.kind == K["PING"]:
                    s.offer_ack(K["PONG"], frame.obj())
                elif frame.kind == K["CMD"]:
                    self._command(s, frame.obj())
                elif frame.kind == K["BYE"]:
                    return
                elif frame.kind not in fpgac.KIND_NAME:
                    self.counts["unknown_kind"] += 1
        except fpgac.ProtocolError as exc:
            self.counts["protocol_error:" + exc.reason] += 1
        except (OSError, ConnectionError, ValueError) as exc:
            self.counts["reader_error:" + type(exc).__name__] += 1
        finally:
            self._drop(s)

    def _command(self, s, body):
        now = self.clock()
        with self.lock:
            ack = self.core.command(s.client_id, self.controller is s, body, now)
            if ack.get("status") == "yielded" and not ack.get("duplicate"):
                self._transfer(exclude=s)
        s.offer_ack(K["CMD_ACK"], ack)
        self._route_results()

    def _transfer(self, exclude):
        """Hand control to the earliest handshaked other client, else leave it free."""
        others = [x for x in self.sessions if x is not exclude and x.handshaked and not x.probe and x.alive]
        self.controller = min(others, key=lambda x: x.order) if others else None
        self.counts["controller_changed"] += 1
        body = fpgac.dumps({"v": 1, "src": self.core.data_source, "age_ms": 0.0,
                           "event": {"event": "controller_changed",
                                     "controller_client_id": self.controller.client_id if self.controller else None}})
        for x in self.sessions:
            if x.handshaked and x.alive:
                x.offer_event(body)

    def _drop(self, s):
        s.close()
        with self.lock:
            if s in self.sessions:
                self.sessions.remove(s)
            if self.controller is s:
                self._transfer(exclude=s)

    # ticking -------------------------------------------------------------------------------
    def push_preview(self, now):
        """Hand a just-fed preview frame to the clients at once instead of on the next state tick
        (10-21: up to one tick, 100 ms, of extra latency and tick/frame aliasing). Caller holds self.lock."""
        prev = self.core.preview_frame(now)
        if prev is None:
            return
        for x in self.sessions:
            if x.handshaked and not x.probe and x.alive and "preview" in x.want:
                x.offer_latest("PREVIEW", K["PREVIEW"], meta=prev[0], payload=prev[1])

    def _route_results(self):
        with self.lock:
            results = self.core.pop_results()
            targets = list(self.sessions)
        for client_id, body in results:
            delivered = False
            for x in targets:
                if x.client_id == client_id and x.handshaked:
                    x.offer_ack(K["CMD_RESULT"], body)
                    delivered = True
            if not delivered:
                self.counts["result_undeliverable"] += 1

    def tick(self):
        now = self.clock()
        with self.lock:
            sup = collections.Counter()
            for x in self.sessions:
                sup.update(x.superseded)
            t, c = time.monotonic(), sum(os.times()[:2])
            cpu = None if t <= self._cpu[0] else 100.0 * (c - self._cpu[1]) / (t - self._cpu[0])
            self._cpu = (t, c)
            self.core.feed_hub_stats(sum(1 for x in self.sessions if x.handshaked), dict(sup), cpu,
                                     self.effective_sndbuf)
            self.core.tick(now)
            state = fpgac.dumps(self.core.compose_state(now))
            events = [fpgac.dumps(e) for e in self.core.pop_events(now)]
            mem = self.core.memory_body(now)
            maps = self.core.map_frames(now)
            prev = self.core.preview_frame(now)
            targets = [x for x in self.sessions if x.handshaked and not x.probe and x.alive]
        self.tick_times.append(now)
        mem = self._memory_bytes(mem)                       # once, not once per client
        for x in targets:
            for e in events:
                x.offer_event(e)
            if mem is not None:
                x.offer_latest("MEMORY", K["MEMORY"], mem)
            for meta, payload in maps:
                x.offer_latest("MAP:" + meta["which"], K["MAP"], meta=meta, payload=payload)
            if prev is not None and "preview" in x.want:
                x.offer_latest("PREVIEW", K["PREVIEW"], meta=prev[0], payload=prev[1])
            x.offer_latest("STATE", K["STATE"], state)
        self._route_results()

    def _tick_loop(self):
        period = 1.0 / self.state_hz
        nxt = time.monotonic()
        while not self._stop.is_set():
            nxt += period
            try:
                self.tick()
            except Exception as exc:                      # visible, never silent
                self.counts["tick_error:" + type(exc).__name__] += 1
            delay = nxt - time.monotonic()
            if delay < 0:
                self.counts["tick_overrun"] += 1
                nxt = time.monotonic()
                delay = 0
            self._stop.wait(delay)


def limit_sndbuf(sock, wanted):
    """Request ``wanted`` bytes of SO_SNDBUF and return what the kernel actually set (Linux
    doubles the request and clamps it at SOCK_MIN_SNDBUF; the read-back is the truth)."""
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, int(wanted))
    return sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)


def _same(a, b):
    import hmac
    return hmac.compare_digest(a.encode("utf-8"), b.encode("utf-8"))


class KvLoops(object):
    """TELEM poll (10 Hz) and e-stop sender threads around package-1 clients (contract F)."""

    def __init__(self, server, telem_client=None, estop_client=None, sync_client=None,
                 hz=10.0, estop_poll_s=0.02, clock=fpga_clock.now_ns):
        self.server, self.core = server, server.core
        self.telem, self.estop_client, self.sync = telem_client, estop_client, sync_client
        self.hz, self.estop_poll_s, self.clock = hz, estop_poll_s, clock
        self._stop = threading.Event()
        self.threads = []

    def start(self):
        for target, name, on in ((self._telem_loop, "kv-telem", self.telem),
                                 (self._estop_loop, "kv-estop", self.estop_client),
                                 (self._sync_loop, "kv-sync", self.sync)):
            if on is not None:
                t = threading.Thread(target=target, name=name, daemon=True)
                t.start()
                self.threads.append(t)
        return self

    def stop(self):
        self._stop.set()
        for t in self.threads:
            t.join(timeout=5)

    def _telem_loop(self):
        period = 1.0 / self.hz
        nxt = time.monotonic()
        while not self._stop.is_set():
            nxt += period
            try:
                tel = self.telem.poll()
            except Exception as exc:
                with self.server.lock:
                    self.core.feed_telem_timeout(self.clock())
                    if type(exc).__name__ != "LinkTimeout":
                        self.core.counts["telem_error:" + type(exc).__name__] += 1
            else:
                with self.server.lock:
                    self.core.feed_telemetry(tel, self.clock())
            self._stop.wait(max(0.0, nxt - time.monotonic()))
            if time.monotonic() - nxt > period:
                nxt = time.monotonic()

    def _estop_loop(self):
        while not self._stop.is_set():
            with self.server.lock:
                job = self.core.estop.take_send(self.clock())
            if job is not None:
                try:
                    ack = self.estop_client.send(job[0], job[1], job[2], timeout_s=0.2)
                except Exception as exc:
                    with self.server.lock:
                        self.core.estop.send_failed(exc)
                else:
                    with self.server.lock:
                        self.core.estop.on_ack(ack, self.clock(), job[1])
                self.server._route_results()
            self._stop.wait(self.estop_poll_s)

    def _sync_loop(self):
        self.sync.start(10)
        while not self._stop.wait(1.0):
            with self.server.lock:
                self.core.feed_clock(self.sync.report(), self.clock())
        self.sync.stop()


class RosActionsLog(object):
    """Actions for the no-ROS mode: record what would be published (counted, visible)."""

    def __init__(self):
        self.published = []

    def publish_query(self, text):
        self.published.append(("/fpga/query", text))

    def publish_select(self, cid):
        self.published.append(("/fpga/select_candidate", cid))

    def publish_cancel(self):
        self.published.append(("/fpga/cancel", None))

    def publish_goto(self, text):
        self.published.append(("/fpga/goto", text))

    def publish_teleop(self, text):
        self.published.append(("/fpga/teleop/input", text))

    def publish_mapping_stop(self):
        self.published.append(("/fpga/mapping/stop", None))

    def publish_map_reset(self, text):
        self.published.append(("/fpga/map_reset", text))


class ReplayFeeder(object):
    """No-ROS input for bench runs: replays /fpga/status events from a runtime JSONL (e.g. the
    m5 software replay's events) plus scripted memory/frame-stats/map inputs. Always declares
    sensor_source=simulation, so it can only be run with --data-source replay."""

    def __init__(self, server, status_jsonl=None, hz=10.0, clock=fpga_clock.now_ns):
        self.server, self.core, self.hz, self.clock = server, server.core, hz, clock
        self.rows = []
        if status_jsonl:
            with open(status_jsonl, "r", encoding="utf-8") as f:
                for line in f:
                    row = json.loads(line)
                    if row.get("event") == "publish" and row.get("topic") == "/fpga/status":
                        row = row["payload"]
                    if isinstance(row, dict) and "event" in row and row.get("event") != "startup":
                        self.rows.append(row)
        self._stop = threading.Event()
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._loop, name="replay-feeder", daemon=True)
        self.thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self.thread:
            self.thread.join(timeout=5)

    def _loop(self):
        lock, core = self.server.lock, self.core
        with lock:
            core.feed_status(json.dumps({"event": "startup", "sensor_source": "simulation",
                                         "replay_id": "hub-replay-feeder"}), self.clock())
        i, period = 0, 1.0 / self.hz
        rev = 0
        while not self._stop.wait(period):
            now = self.clock()
            i += 1
            with lock:
                core.feed_frame_stats(json.dumps({"v": 1, "kv_frame_id": i, "capture_nano_ns": now - 820 * 1000000,
                                                  "result": "fused", "frame_age_ms": 820.0, "channel_ms": 210.0,
                                                  "mailbox_wait_ms": 5.0, "rtt_ms": 600.0,
                                                  "kv_times_ms": {"core_device": 604.0, "core_total": 610.0},
                                                  "client_counts": {"timeouts": 0}}), now)
                core.feed_motion_permit(False, now)
                if self.rows:
                    core.feed_status(json.dumps(self.rows[i % len(self.rows)]), now)
                if i % 5 == 0:
                    rev += 1 if i % 20 == 0 else 0
                    tracks = [{"id": "object-%04d" % k, "label": "bottle", "color": "blue",
                               "position_m": [1.0 + k, 0.5, 0.2], "state": "historical_memory",
                               "history_len": 3} for k in range(1 + rev % 4)]
                    core.feed_memory(json.dumps({"v": 1, "runtime_boot_id": "feeder", "revision": rev,
                                                 "world_frame": "map", "sensor_source": "simulation",
                                                 "digest": "rev%d" % rev, "tracks": tracks}), now)
                    core.feed_map("rolling", {"rev": i, "width": 80, "height": 80, "resolution_m": 0.05,
                                              "origin": [0, 0, 0], "frame_id": "odom"},
                                  bytes((i + k) % 100 for k in range(6400)), now)


def start_tegrastats(server, interval_ms=1000):
    """Optional H1: one tegrastats child, parsed into the nano group."""
    try:
        proc = subprocess.Popen(["tegrastats", "--interval", str(interval_ms)], stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, universal_newlines=True)
    except OSError:
        server.counts["tegrastats_unavailable"] += 1
        return None

    def loop():
        for line in proc.stdout:
            with server.lock:
                server.core.feed_tegrastats(line, server.clock())

    threading.Thread(target=loop, name="tegrastats", daemon=True).start()
    return proc


def read_nvpmodel(server):
    try:
        out = subprocess.run(["nvpmodel", "-q"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             universal_newlines=True, timeout=10).stdout
    except (OSError, subprocess.SubprocessError):
        server.counts["nvpmodel_unavailable"] += 1
        return
    with server.lock:
        server.core.feed_nvpmodel(out, server.clock())


def read_token(path):
    with open(path, "r", encoding="utf-8") as f:
        token = f.read().strip()
    if len(token) < 16:
        raise SystemExit("token file must hold a random string of >= 16 characters")
    return token


def common_args(parser):
    parser.add_argument("--data-source", choices=("live", "replay", "sim"), required=True,
                        help="sim: Gazebo simulation (line D1), shown as SIM on the PC")
    parser.add_argument("--token-file", required=True)
    parser.add_argument("--port", type=int, default=fpgac.DEFAULT_PORT)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--deny-local-net", action="append", default=None,
                        help="local-address networks whose connections are closed (default 198.51.100.0/24)")
    parser.add_argument("--state-hz", type=float, default=10.0)
    parser.add_argument("--kv", default="none", help="host:port of kv_server, or none")
    parser.add_argument("--tegrastats", action="store_true")
    parser.add_argument("--stats-out", help="new JSON file written on exit with hub counters")
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--sndbuf", type=int, default=SNDBUF_MIN_REQUEST,
                        help="requested SO_SNDBUF per client (default 1 = kernel minimum; the "
                             "effective value is reported); raise only if the hotspot cannot carry "
                             "10 Hz STATE (throughput ceiling ~ effective/RTT)")


def make_kv(spec):
    if spec == "none":
        return None, None, None
    host, port = spec.rsplit(":", 1)
    server = (host, int(port))
    from host.kv_link import KvTelemetryClient, KvEstopClient   # package 1 (contract F)
    from host.kv_clock_sync import ClockSyncClient
    return KvTelemetryClient(server, timeout_s=0.2), KvEstopClient(server), ClockSyncClient(server)


def run(server, args, extra_stop=()):
    kv = make_kv(args.kv)
    loops = KvLoops(server, *kv).start() if kv[0] is not None else None
    tegra = start_tegrastats(server) if args.tegrastats else None
    read_nvpmodel(server)
    server.start()
    sys.stdout.write("hub listening on %s:%d boot=%s data_source=%s\n" % (
        args.bind, server.port, server.core.hub_boot_id, server.core.data_source))
    sys.stdout.flush()
    t0 = time.monotonic()
    try:
        while args.duration_s <= 0 or time.monotonic() - t0 < args.duration_s:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        for x in extra_stop:
            x.stop()
        if loops:
            loops.stop()
        if tegra:
            tegra.terminate()
            tegra.wait(timeout=5)
        ticks = list(server.tick_times)
        gaps = [(b - a) / 1e6 for a, b in zip(ticks, ticks[1:])]
        stats = {"counts": dict(server.counts), "ticks": len(ticks),
                 "effective_sndbuf": server.effective_sndbuf,
                 "tick_gap_ms": {"p50": fpgac.percentile(gaps, 50), "p99": fpgac.percentile(gaps, 99),
                                 "max": max(gaps) if gaps else None},
                 "cpu_times_s": list(os.times()[:2]), "wall_s": time.monotonic() - t0}
        server.close()
        if args.stats_out:
            with open(args.stats_out, "x", encoding="utf-8") as f:
                json.dump(stats, f, indent=1)
        sys.stdout.write(json.dumps(stats) + "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    common_args(parser)
    parser.add_argument("--feed", choices=("synthetic",), required=True,
                        help="no-ROS input; use host.hub.hub_ros for the ROS graph")
    parser.add_argument("--status-jsonl", help="runtime events to replay (m5 replay events.jsonl)")
    args = parser.parse_args(argv)
    hub_profile.install()              # FPGA_HUB_PROFILE=<dir>: per-thread CPU (default off)
    if args.data_source != "replay":
        parser.error("the no-ROS feeder is replay data; --data-source must be replay")
    core = HubCore(args.data_source, state_hz=args.state_hz, actions=RosActionsLog())
    server = HubServer(core, read_token(args.token_file), port=args.port, host=args.bind,
                       deny_nets=args.deny_local_net or ("198.51.100.0/24",), sndbuf=args.sndbuf)
    feeder = ReplayFeeder(server, args.status_jsonl).start()
    run(server, args, extra_stop=(feeder,))
    hub_profile.finish_main()


if __name__ == "__main__":
    main()
