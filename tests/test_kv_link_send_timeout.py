"""kv_link client: the receive timeout must not leak into the send (board 10-15, r1-r3).

KvLinkClient receives and sends on ONE socket; ``_recv_message`` does ``settimeout(left)``. Before
the fix that timeout stayed on the socket, so the next ``sendto`` was bounded by the few ms left over
from the last receive: a send buffer that is full for a moment (a short TX stall) made sendto raise
a raw TimeoutError, which M7Pipeline does not catch -> nano_replay died (replay.err of r1-r3:
``kv_link.py _send -> sock.sendto -> TimeoutError: timed out``).

Real kernel, no network: an AF_UNIX datagram socketpair whose peer queue is filled makes a blocking
send really block (the same "send buffer full" state); a thin adapter gives it the (data, addr)
sendto/recvfrom shape KvLinkClient uses. Known answers:
  * leak: after a receive that timed out, a send into a full queue that drains after 150 ms must go
    through (the fixed client waits up to send_timeout_s). The code before the fix raises
    socket.timeout after ~20 ms (the positive control runs the old module: must fail that way);
  * bound: a queue that never drains -> LinkTimeout (a LinkError, which M7Pipeline counts as a link
    failure) after about send_timeout_s, counted ``send_timeout``; never an uncaught OSError;
  * the send bound does not change the receive: _recv_message still returns None at its deadline.
Run: cd tests && PYTHONPATH=.. python3 -m unittest discover -s . -p test_kv_link_send_timeout.py
"""
import importlib.util
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host import kv_link  # noqa: E402


class UnixAdapter:
    """(data, addr) shaped socket over a connected AF_UNIX datagram socket; timeouts are the real
    kernel socket's (settimeout/gettimeout pass through)."""

    def __init__(self, sock):
        self.s = sock

    def settimeout(self, t):
        self.s.settimeout(t)

    def gettimeout(self):
        return self.s.gettimeout()

    def sendto(self, data, _addr):
        return self.s.send(data)

    def recvfrom(self, n):
        return self.s.recv(n), ("peer", 0)

    def close(self):
        self.s.close()


def full_pair():
    """(client adapter, peer socket) with the peer's receive queue full: a blocking send blocks."""
    a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
    a.setblocking(False)
    n = 0
    while True:
        try:
            a.send(b"x" * 1024)
            n += 1
        except BlockingIOError:
            break
    a.setblocking(True)
    assert n > 0
    return UnixAdapter(a), b


def drain(sock, delay_s):
    def run():
        time.sleep(delay_s)
        sock.setblocking(False)
        try:
            while True:
                sock.recv(65536)
        except BlockingIOError:
            pass
    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class SendTimeoutTests(unittest.TestCase):
    def client(self, mod, sock, **kw):
        return mod.KvLinkClient(("peer", 1), sock=sock, timeout_s=2.0, **kw)

    def leak_case(self, mod):
        """Receive times out after 20 ms, then a send into a queue that drains after 150 ms."""
        sock, peer = full_pair()
        try:
            c = self.client(mod, sock)
            t0 = time.monotonic()
            self.assertIsNone(c._recv_message(time.monotonic() + 0.02))
            self.assertLess(time.monotonic() - t0, 0.5)
            th = drain(peer, 0.15)
            t1 = time.monotonic()
            try:
                c._send(mod.HELLO, 1, b"{}")
                return None, time.monotonic() - t1
            except Exception as exc:        # noqa: BLE001  (the outcome is the answer)
                return exc, time.monotonic() - t1
            finally:
                th.join(2)
        finally:
            sock.close()
            peer.close()

    def test_short_stall_rides_through(self):
        exc, dt = self.leak_case(kv_link)
        self.assertIsNone(exc, "send failed after %.3f s: %r" % (dt, exc))
        self.assertGreaterEqual(dt, 0.1)        # it really blocked until the drain

    def test_never_drains_is_a_link_timeout(self):
        sock, peer = full_pair()
        try:
            c = self.client(kv_link, sock, send_timeout_s=0.3)
            c._recv_message(time.monotonic() + 0.02)
            t0 = time.monotonic()
            with self.assertRaises(kv_link.LinkTimeout):
                c._send(kv_link.HELLO, 1, b"{}")
            dt = time.monotonic() - t0
            self.assertTrue(0.25 <= dt < 1.5, dt)
            self.assertEqual(c.counts["send_timeout"], 1)
            self.assertTrue(issubclass(kv_link.LinkTimeout, kv_link.LinkError))
            self.assertTrue(issubclass(kv_link.LinkTimeout, RuntimeError))   # M7Pipeline catches it
            # the default bound is the call timeout
            self.assertEqual(self.client(kv_link, UnixAdapter(socket.socket(socket.AF_UNIX,
                                                                             socket.SOCK_DGRAM))).send_timeout_s, 2.0)
        finally:
            sock.close()
            peer.close()

    def test_receive_deadline_unchanged(self):
        a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            c = self.client(kv_link, UnixAdapter(a))
            t0 = time.monotonic()
            self.assertIsNone(c._recv_message(time.monotonic() + 0.08))
            self.assertTrue(0.06 <= time.monotonic() - t0 < 0.5)
        finally:
            a.close()
            b.close()

    def test_positive_control_old_code_fails(self):
        """The module as of a85db90 (before the fix) fails the leak case with socket.timeout
        within the 20 ms left from the receive -- the defect seen on the board."""
        try:
            src = subprocess.run(["git", "-C", str(ROOT), "show", "a85db90:host/kv_link.py"],
                                 check=True, stdout=subprocess.PIPE).stdout
        except (OSError, subprocess.CalledProcessError):
            self.skipTest("git history not available")
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "kv_link_a85db90.py")
            with open(p, "wb") as fp:
                fp.write(src)
            old = load_module(p, "kv_link_a85db90")
            self.assertFalse(hasattr(old.KvLinkClient(("peer", 1), sock=UnixAdapter(
                socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM))), "send_timeout_s"))
            exc, dt = self.leak_case(old)
        self.assertIsInstance(exc, socket.timeout)
        self.assertLess(dt, 0.12)


# ------------------------------------------------------------------ server side (2026-10-16)
class ServerSock:
    """KvLinkServer's one socket, split for the test: receives from ``rx`` (a socketpair end the
    test writes requests into), sends into ``tx`` (a socketpair end whose peer queue the test
    fills). settimeout applies to both, as on the real shared socket; every call is recorded."""

    def __init__(self, rx, tx):
        self.rx, self.tx = rx, tx
        self.timeouts = []

    def settimeout(self, t):
        self.timeouts.append(t)
        self.rx.settimeout(t)
        self.tx.settimeout(t)

    def sendto(self, data, _addr):
        return self.tx.send(data)

    def recvfrom(self, n):
        return self.rx.recv(n), ("peer", 0)

    def getsockname(self):
        return ("test", 0)

    def close(self):
        self.rx.close()
        self.tx.close()


class ServerSendTimeoutTests(unittest.TestCase):
    """KvLinkServer._reader sets the shared socket's timeout to 0.2 s for its recvfrom; before the
    fix every sendto (worker RESP, reader HELLO_ACK / SYNC_ACK / TELEM_ACK) inherited it, and a
    send buffer full for > 0.2 s raised socket.timeout in that thread: the worker set worker_error
    (kv_server main exits "worker error"), the reader thread died. Known answers: a 0.5 s stall is
    ridden through; a stall that never ends drops that one message after send_timeout_s, counted,
    no exception, the receive poll timeout put back; the reader keeps answering afterwards. The
    positive control runs the module before the fix (8cc9c5e) on the same stall."""

    def make(self, mod, **kw):
        rx_srv, rx_peer = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        tx, peer = full_pair()
        tx = tx.s
        sock = ServerSock(rx_srv, tx)
        srv = mod.KvLinkServer(mod.NullCore(), "null", sock=sock, nudge_after_burst=False,
                               nudge_idle_s=0.0, **kw)
        self.addCleanup(peer.close)
        self.addCleanup(rx_peer.close)
        return srv, sock, rx_peer, peer

    def stall_case(self, mod, **kw):
        srv, sock, _rx, peer = self.make(mod, **kw)
        sock.settimeout(0.2)                    # what _reader leaves on the socket
        th = drain(peer, 0.5)
        t0 = time.monotonic()
        try:
            out = srv._sendto(("peer", 0), mod.HELLO_ACK, 7, 1, b"{}")
            return out, None, time.monotonic() - t0, srv
        except Exception as exc:                # noqa: BLE001  (the outcome is the answer)
            return None, exc, time.monotonic() - t0, srv
        finally:
            th.join(2)
            sock.close()

    def test_server_short_stall_rides_through(self):
        out, exc, dt, srv = self.stall_case(kv_link)
        self.assertIsNone(exc, "server send failed after %.3f s: %r" % (dt, exc))
        self.assertTrue(out)
        self.assertGreaterEqual(dt, 0.4)        # it really blocked until the drain
        self.assertEqual(srv.counts["send_timeout"], 0)
        self.assertEqual(srv.counts["tx_messages:hello_ack"], 1)
        self.assertEqual(kv_link.SERVER_SEND_TIMEOUT_S, 1.0)
        self.assertEqual(srv.send_timeout_s, 1.0)

    def test_server_never_drains_drops_one_message_counted(self):
        srv, sock, _rx, _peer = self.make(kv_link, send_timeout_s=0.3)
        sock.settimeout(0.2)
        t0 = time.monotonic()
        out = srv._sendto(("peer", 0), kv_link.RESP, 7, 1, b"x" * 5000, chunk_bytes=1400)
        dt = time.monotonic() - t0
        sock.close()
        self.assertFalse(out)
        self.assertTrue(0.25 <= dt < 1.5, dt)
        self.assertEqual(srv.counts["send_timeout"], 1)
        self.assertEqual(srv.counts["send_timeout:resp"], 1)
        self.assertEqual(srv.counts["send_timeout_dropped_chunks"], 4)   # 5000 B in 1400 B chunks
        self.assertEqual(srv.counts["tx_messages:resp"], 0)
        self.assertEqual(sock.timeouts[-2:], [0.3, kv_link.SERVER_RECV_POLL_S])  # receive poll put back

    def test_server_reader_survives_and_answers_again(self):
        srv, sock, rx_peer, peer = self.make(kv_link, send_timeout_s=0.3)
        srv.start()
        try:
            for d in kv_link.split(kv_link.HELLO, 9, 1, b"{}", 1400):
                rx_peer.send(d)
            t0 = time.monotonic()
            while srv.counts["send_timeout"] < 1 and time.monotonic() - t0 < 3:
                time.sleep(0.02)
            self.assertEqual(srv.counts["send_timeout:hello_ack"], 1)
            reader = [t for t in srv._threads if t.name == "kv-reader"][0]
            self.assertTrue(reader.is_alive())
            drain(peer, 0.0).join(2)
            for d in kv_link.split(kv_link.HELLO, 9, 2, b"{}", 1400):
                rx_peer.send(d)
            peer.settimeout(2.0)
            got = peer.recv(65536)
            self.assertEqual(kv_link.CHUNK.unpack_from(got)[1], kv_link.HELLO_ACK)
            self.assertIsNone(srv.worker_error)
        finally:
            srv.stop()

    def test_positive_control_server_before_fix_raises(self):
        try:
            src = subprocess.run(["git", "-C", str(ROOT), "show", "8cc9c5e:host/kv_link.py"],
                                 check=True, stdout=subprocess.PIPE).stdout
        except (OSError, subprocess.CalledProcessError):
            self.skipTest("git history not available")
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "kv_link_8cc9c5e.py")
            with open(p, "wb") as fp:
                fp.write(src)
            old = load_module(p, "kv_link_8cc9c5e")
            self.assertFalse(hasattr(old, "SERVER_SEND_TIMEOUT_S"))
            out, exc, dt, _srv = self.stall_case(old)
        self.assertIsInstance(exc, socket.timeout)
        self.assertLess(dt, 0.45)               # the 0.2 s receive timeout, not the 0.5 s stall


if __name__ == "__main__":
    unittest.main()
