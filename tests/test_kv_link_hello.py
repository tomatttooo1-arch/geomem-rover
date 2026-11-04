"""New-session HELLO hardening (docs/design/KV_LINK_HELLO_FIX_20261011.md).

The board fault: a HELLO_ACK that does not reach the client within the HELLO timeout
(1847 B, IP-fragmented). Injected here by a server socket wrapper that either drops the
first HELLO_ACK or holds it until the server's next transmission ("held" = what a
transmit path that keeps the last frame until the next one is queued looks like).

1. Repro: the b790f1a client (loaded from git) and the new client with resends off both
   time out against a server that drops/holds the first HELLO_ACK; the new client passes.
2. Resend is idempotent: the server answers each resend (new frame id) with no other
   state; one answer is taken; frames and STATS work afterwards on the same client.
3. HELLO_ACK is one Ethernet frame with the real KV260 server info (tests/data); every
   control answer datagram fits CHUNK_MTU, also a multi-KB STATS_ACK.
4. Compatibility both ways: b790f1a client <-> new server, new client <-> b790f1a server.
5. KvGeometry.connect names the failing step: hello_timeout / telem_hello_timeout /
   telem_timeout; the telemetry client reuses the HELLO answer (no second HELLO).
Run: PYTHONPATH=. python3 -m unittest discover -s tests -p test_kv_link_hello.py
(also with /usr/bin/python3.8).
"""
import json
from pathlib import Path
import socket
import subprocess
import sys
import threading
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host import kv_link  # noqa: E402
from host.kv_link import (CHUNK, CHUNK_MTU, HELLO_ACK, KvLinkClient, KvLinkServer,  # noqa: E402
                          LinkTimeout, NullCore)

BASELINE = "b790f1a"
INFO = json.loads((ROOT / "tests" / "data" / "kv_server_info_20261011.json").read_text())


def legacy_module():
    """host/kv_link.py as of b790f1a (before this fix), or None without git history."""
    try:
        src = subprocess.check_output(["git", "show", "%s:host/kv_link.py" % BASELINE],
                                      cwd=str(ROOT), stderr=subprocess.DEVNULL).decode("utf-8")
    except (OSError, subprocess.CalledProcessError):
        return None
    mod = types.ModuleType("kv_link_legacy_%s" % BASELINE)
    mod.__file__ = str(ROOT / "host" / "kv_link.py")
    exec(compile(src, "kv_link@%s" % BASELINE, "exec"), mod.__dict__)
    return mod


LEGACY = legacy_module()


class FaultySock(object):
    """Server socket wrapper. mode "drop": the first ``n`` HELLO_ACK datagrams vanish.
    mode "hold": the first HELLO_ACK is kept and sent right before the next datagram the
    server sends (to anyone). Records every datagram size sent."""

    def __init__(self, mode, n=1):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.mode, self.n = mode, n
        self.held = []
        self.sizes = []
        self.dropped = 0
        self.lock = threading.Lock()

    def __getattr__(self, name):
        return getattr(self.sock, name)

    def sendto(self, data, addr):
        with self.lock:
            self.sizes.append((data[4], len(data)))
            is_ack = len(data) >= CHUNK.size and data[:4] == kv_link.MAGIC and data[4] == HELLO_ACK
            if self.held:
                for d, a in self.held:
                    self.sock.sendto(d, a)
                self.held = []
            if is_ack and self.n > 0:
                self.n -= 1
                if self.mode == "drop":
                    self.dropped += 1
                    return len(data)
                self.held.append((bytes(data), addr))
                return len(data)
            return self.sock.sendto(data, addr)


def server(mod=kv_link, sock=None, info=None, nudge=True):
    """``nudge=False``: this tree's server without the NUDGE tail release (KV_LINK_BURST_TAIL,
    later than this fix), i.e. the wire behaviour the HELLO resend was built against -- the
    server's own NUDGE would release a held HELLO_ACK 2 ms later and hide the client side."""
    kw = {} if nudge or mod is not kv_link else {"nudge_after_burst": False, "nudge_idle_s": 0.0}
    return mod.KvLinkServer(mod.NullCore(), "null", bind=("127.0.0.1", 0), sock=sock,
                            info=info, **kw).start()


def port_of(srv):
    return srv.sock.getsockname()[1]


def settled_count(srv, key, want, timeout_s=2.0, counts=None):
    """The server bumps its tx_messages counters after sendto returns (and, since 2026-10-16, after it
    restores the socket's receive timeout), so the client can hold the reply before the counter moves.
    Wait for the counter to reach ``want`` (bounded), then return it for the exact assertion.
    ``counts``: another of the server's counter dicts (a session's, ``srv.session_counts[s]``)."""
    import time
    counts = srv.counts if counts is None else counts
    end = time.monotonic() + timeout_s
    while counts[key] < want and time.monotonic() < end:
        time.sleep(0.005)
    return counts[key]


class ReproTests(unittest.TestCase):
    def check_fault(self, mode):
        for label, make in (("new_no_resend", lambda p: (KvLinkClient(("127.0.0.1", p)), 0.0)),
                            ("legacy", lambda p: (LEGACY.KvLinkClient(("127.0.0.1", p)), None))):
            if label == "legacy" and LEGACY is None:
                continue
            fs = FaultySock(mode)
            srv = server(sock=fs, info=INFO, nudge=False)
            try:
                c, resend = make(port_of(srv))
                with self.assertRaises(Exception) as cm:
                    c.hello(0.4) if resend is None else c.hello(0.4, resend_s=resend)
                self.assertEqual(type(cm.exception).__name__, "LinkTimeout", label)
                c.close()
            finally:
                srv.stop()
        fs = FaultySock(mode)
        srv = server(sock=fs, info=INFO, nudge=False)
        try:
            c = KvLinkClient(("127.0.0.1", port_of(srv)))
            info = c.hello(0.4)
            self.assertEqual(info["features"], kv_link.FEATURES)
            self.assertGreaterEqual(c.counts["resend:hello"], 1)
            c.close()
        finally:
            srv.stop()
        return c, srv

    def test_first_ack_dropped(self):
        c, srv = self.check_fault("drop")
        self.assertEqual(c.counts["resend:hello"], 1)
        self.assertEqual(srv.counts["tx_messages:hello_ack"], 2)

    def test_first_ack_held_until_next_send(self):
        c, srv = self.check_fault("hold")
        # the resend makes the server transmit, which releases the held answer: either may win
        self.assertEqual(c.counts["resend:hello"], 1)


class IdempotenceTests(unittest.TestCase):
    def test_resends_answered_alike_and_link_usable_after(self):
        fs = FaultySock("drop", n=2)
        srv = server(sock=fs, info=INFO)
        try:
            c = KvLinkClient(("127.0.0.1", port_of(srv)), chunk_bytes=CHUNK_MTU, timeout_s=1.0)
            info = c.hello(1.0)
            self.assertEqual(c.counts["resend:hello"], 2)
            sc = srv.session_counts[c.session]
            self.assertEqual(sc["rx_messages:hello"], 3)
            self.assertEqual(settled_count(srv, "tx_messages:hello_ack", 3, counts=sc), 3)   # (counted after sendto)
            self.assertEqual(fs.dropped, 2)
            self.assertEqual(info, json.loads(srv.hello_body().decode()))
            st = c.stats()                      # control after the round still matches
            self.assertEqual(st["info"]["argv"], INFO["argv"])
            self.assertEqual(sc["rx_messages:hello"], 3)
            c.close()
        finally:
            srv.stop()

    def test_one_send_when_answered_fast(self):
        srv = server(info=INFO)
        try:
            c = KvLinkClient(("127.0.0.1", port_of(srv)))
            c.hello(1.0)
            self.assertEqual(c.counts["resend:hello"], 0)
            self.assertEqual(settled_count(srv, "tx_messages:hello_ack", 1), 1)
            c.close()
        finally:
            srv.stop()


class SingleFrameTests(unittest.TestCase):
    def test_hello_ack_fits_one_ethernet_frame_with_real_info(self):
        fs = FaultySock("drop", n=0)
        srv = server(sock=fs, info=INFO)
        try:
            body = srv.hello_body()
            self.assertLessEqual(len(body) + CHUNK.size + 8 + 20, 1500)
            for k in ("core", "features", "roi_core", "roi_max_boxes", "kv_boot_id", "chunk_bytes"):
                self.assertIn(k, json.loads(body.decode()))
            c = KvLinkClient(("127.0.0.1", port_of(srv)))
            c.hello(1.0)
            c.stats()
            c.close()
            acks = [n for t, n in fs.sizes if t == HELLO_ACK]
            self.assertEqual(len(acks), 1)
            stats = [n for t, n in fs.sizes if t == kv_link.STATS_ACK]
            self.assertGreater(len(stats), 1)                       # multi-chunk, not IP-fragmented
            self.assertTrue(all(n <= CHUNK_MTU + CHUNK.size for t, n in fs.sizes), fs.sizes)
            # legacy HELLO_ACK of the same info: one datagram > 1472 B (IP-fragmented on the wire)
            self.assertGreater(len(json.dumps(srv.info, sort_keys=True)) + CHUNK.size, 1472)
        finally:
            srv.stop()


@unittest.skipIf(LEGACY is None, "git history with %s not available" % BASELINE)
class CompatTests(unittest.TestCase):
    def test_legacy_client_new_server(self):
        srv = server(info=INFO)
        try:
            c = LEGACY.KvLinkClient(("127.0.0.1", port_of(srv)))
            info = c.hello(1.0)
            self.assertEqual(info["roi_max_boxes"], kv_link.ROI_MAX_BOXES)
            self.assertEqual(c.require("telem")["features"]["telem"], 1)
            self.assertIn("core_detail", c.stats()["info"])
            c.close()
        finally:
            srv.stop()

    def test_new_client_legacy_server_with_lost_ack(self):
        fs = FaultySock("drop")
        srv = server(mod=LEGACY, sock=fs, info=INFO)
        try:
            c = KvLinkClient(("127.0.0.1", port_of(srv)))
            info = c.hello(1.0)
            self.assertEqual(info["argv"], INFO["argv"])          # legacy: full info in HELLO_ACK
            self.assertEqual(c.counts["resend:hello"], 1)
            c.close()
        finally:
            srv.stop()


class ConnectReasonTests(unittest.TestCase):
    def geo(self, port, **kw):
        from host.kv_geometry import KvGeometry
        return KvGeometry(("127.0.0.1", port), depth_unit_m=2.0 ** -10, hello_timeout_s=0.3,
                          retry_s=0.0, expect_build_id=0x0117BC0A, **kw)

    def roi_server(self, telem_answers=True):
        from host.kv_regs import mock_sysinfo
        blk, _ = mock_sysinfo(build_id=0x0117BC0A)
        roi = types.SimpleNamespace(name="cpu")
        srv = KvLinkServer(NullCore(), "null", bind=("127.0.0.1", 0), roi_core=roi,
                           regs={"sysinfo": blk})
        if not telem_answers:
            srv.telem = types.SimpleNamespace(answer=lambda payload: (_ for _ in ()).throw(
                kv_link.BadExtension("test", "no telem answer")))
        return srv.start()

    def test_ok_and_single_hello(self):
        srv = self.roi_server()
        try:
            g = self.geo(srv.port)
            self.assertTrue(g.connect(), g.reason)
            self.assertEqual(settled_count(srv, "tx_messages:hello_ack", 1), 1)   # telemetry reused it
            self.assertEqual(settled_count(srv, "tx_messages:telem_ack", 1), 1)
            g.close()
        finally:
            srv.stop()

    def test_telem_timeout(self):
        srv = self.roi_server(telem_answers=False)
        try:
            g = self.geo(srv.port)
            self.assertFalse(g.connect())
            self.assertEqual(g.reason, "telem_timeout")
            g.close()
        finally:
            srv.stop()

    def test_telem_hello_timeout(self):
        srv = self.roi_server()

        class Tel(object):                        # a telemetry client whose own HELLO fails
            counts = {}
            link = None

            def poll(self, want):
                raise LinkTimeout("no HELLO answer")

            def close(self):
                pass
        try:
            g = self.geo(srv.port, telemetry_factory=lambda s: Tel())
            self.assertFalse(g.connect())
            self.assertEqual(g.reason, "telem_hello_timeout")
        finally:
            srv.stop()

    def test_hello_timeout(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))                  # bound, never answers
        try:
            g = self.geo(s.getsockname()[1])
            self.assertFalse(g.connect())
            self.assertEqual(g.reason, "hello_timeout")
        finally:
            s.close()


if __name__ == "__main__":
    unittest.main()
