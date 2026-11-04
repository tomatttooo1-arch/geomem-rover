"""KV260 transmit-tail release (docs/design/KV_LINK_BURST_TAIL_20261011.md).

The board fault: the KV260 NIC keeps the last Ethernet frame of an answer until the
KV260 transmits anything else. Injected here by a server socket wrapper that holds the
last datagram of a message until the server's next sendto (to anyone) -- on loopback a
datagram is the unit, on the board it is the last IP fragment / last chunk.

1. Repro (positive control): the server and client as of 2678454 (before this fix,
   loaded from git) time out on every held frame answer; so does this tree's server with
   --nudge off and a client without poke. Fixed: this tree's server (NUDGE after a
   multi-frame answer and after 2 ms idle; each alone is enough) answers every frame,
   also to the 2678454 client; this tree's client gets its answer from a 2678454 server
   through the HELLO poke.
2. Harsh model: EVERY message's last datagram is held (so the NUDGE itself is held too):
   all frame answers still arrive; the held NUDGE is harmless.
3. STATS_ACK (multi-chunk) and TELEM_ACK with a held tail: released by the NUDGE; against
   a server without NUDGE, TELEM's resend gets it (resend 0 = old single send times out).
4. Accounting: NUDGEs are counted apart (tx_nudge:*, rx_nudge), chunk_loss() unchanged;
   ClockSyncClient ignores them; the idle NUDGE fires once per quiet period.
5. Late-answer record: a held answer released by the next call is logged with its lateness.
Run: PYTHONPATH=. python3 tests/test_kv_link_burst_tail.py (also /usr/bin/python3.8).
"""
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import types
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host import kv_link  # noqa: E402
from host.kv_link import (CHUNK, HELLO_ACK, NUDGE, RESP, STATS_ACK, TELEM_ACK,  # noqa: E402
                          KvLinkClient, KvLinkServer, KvTelemetryClient, LinkTimeout, NullCore,
                          chunk_loss, wire_frames)

BASELINE = "2678454"     # merge of kv-link-hello into car-geo-producer, before this fix
H, W = 48, 64
K = np.array([[60.0, 0.0, 31.5], [0.0, 60.0, 23.5], [0.0, 0.0, 1.0]])
UNIT = 0.001
IDENT = {"stream_instance": "s", "clock_id": "c", "calibration_id": "cal", "pose_epoch": "e"}


def legacy_module():
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


class TailHoldSock(object):
    """Server socket wrapper: the last datagram of a message whose type is in ``types``
    (None = every type, NUDGE included) and that is more than one Ethernet frame in all
    (``multi_only``) is kept and sent right before the next datagram the server sends."""

    def __init__(self, types=(RESP,), multi_only=True):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.types, self.multi_only = types, multi_only
        self.held = []
        self.held_count = {}
        self.sent = []          # (type, len)
        self.lock = threading.Lock()
        self._frames = {}       # (type, session, frame_id) -> wire frames so far

    def __getattr__(self, name):
        return getattr(self.sock, name)

    def sendto(self, data, addr):
        with self.lock:
            for d, a in self.held:
                self.sock.sendto(d, a)
            self.held = []
            data = bytes(data)
            hold = False
            if len(data) >= CHUNK.size and data[:4] == kv_link.MAGIC:
                _, mtype, _, _, session, fid, _, idx, n = CHUNK.unpack_from(data)
                self.sent.append((mtype, len(data)))
                key = (mtype, session, fid)
                frames = self._frames.get(key, 0) + wire_frames(len(data))
                self._frames[key] = frames
                last = idx == n - 1
                if last:
                    self._frames.pop(key, None)
                if last and (self.types is None or mtype in self.types) and \
                        (frames > 1 or not self.multi_only):
                    hold = True
                    self.held_count[mtype] = self.held_count.get(mtype, 0) + 1
            if hold:
                self.held.append((data, addr))
                return len(data)
            return self.sock.sendto(data, addr)


def request(fid, nz=10):
    c = (600 + (np.arange(H * W) % 97)).astype(np.uint16).reshape(H, W)
    d = c.astype(np.float32) * np.float32(UNIT)
    body = kv_link.encode_request(
        frame_id=fid, capture_ns=1000 + fid, pose_ns=1000 + fid, identity=IDENT, width=W,
        height=H, nx=32, ny=32, nz=nz, unit_f32=np.float32(UNIT), voxel=0.05,
        origin=np.array([-0.8, -0.8, -0.05]), inverse_k=np.linalg.inv(K).astype(np.float32),
        world_camera=np.eye(4)[:3].astype(np.float32), codes=c, depth_crc=kv_link.crc32(d))
    return body, {"frame_id": fid, "capture_ns": 1000 + fid, "pose_ns": 1000 + fid,
                  "identity": IDENT}


def new_server(sock=None, **kw):
    return KvLinkServer(NullCore(), "null", bind=("127.0.0.1", 0), sock=sock, **kw).start()


def legacy_server(sock=None):
    return LEGACY.KvLinkServer(LEGACY.NullCore(), "null", bind=("127.0.0.1", 0), sock=sock).start()


def port(srv):
    return srv.sock.getsockname()[1]


def run_calls(client, n, first=1):
    """(answered frame ids, timed-out frame ids) of n window-1 calls."""
    ok, lost = [], []
    for fid in range(first, first + n):
        body, expect = request(fid)
        try:
            client.call(body, expect)
            ok.append(fid)
        except Exception as exc:            # this tree's or the git baseline's LinkTimeout
            if type(exc).__name__ != "LinkTimeout":
                raise
            lost.append(fid)
    return ok, lost


class ReproTests(unittest.TestCase):
    def test_resp_is_multi_frame(self):
        body, _ = request(1)
        srv = new_server()
        try:
            c = KvLinkClient(("127.0.0.1", port(srv)), timeout_s=1.0, poke_s=0)
            c.hello(1.0)
            c.call(body, request(1)[1])
            self.assertGreater(srv.counts["tx_multiframe:resp"], 0)   # the shape that gets held
            c.close()
        finally:
            srv.stop()

    @unittest.skipIf(LEGACY is None, "no git history")
    def test_before_fix_every_held_answer_times_out(self):
        # 2678454 server + 2678454 client
        fs = TailHoldSock()
        srv = legacy_server(fs)
        try:
            c = LEGACY.KvLinkClient(("127.0.0.1", port(srv)), timeout_s=0.25)
            c.hello(1.0)
            ok, lost = run_calls(c, 3)
            self.assertEqual(ok, [])
            self.assertEqual(lost, [1, 2, 3])
            self.assertGreaterEqual(c.counts["stale_response"], 2)   # each came with the next call
            c.close()
        finally:
            srv.stop()
        # this tree with NUDGE off and no poke = the same wire behaviour
        fs = TailHoldSock()
        srv = new_server(fs, nudge_after_burst=False, nudge_idle_s=0.0)
        try:
            c = KvLinkClient(("127.0.0.1", port(srv)), timeout_s=0.25, poke_s=0)
            c.hello(1.0)
            ok, lost = run_calls(c, 3)
            self.assertEqual(lost, [1, 2, 3])
            # the late-answer record: frames 1 and 2 arrived during the next call, >= timeout late
            self.assertEqual([r["frame_id"] for r in c.late_answers], [1, 2])
            for r in c.late_answers:
                self.assertGreaterEqual(r["late_s"], 0.25)
                self.assertLess(r["since_send_s"], 0.2)
            c.close()
        finally:
            srv.stop()

    def check_fixed_server(self, client_mod, **server_kw):
        fs = TailHoldSock()
        srv = new_server(fs, **server_kw)
        try:
            kw = {"poke_s": 0} if client_mod is kv_link else {}
            c = client_mod.KvLinkClient(("127.0.0.1", port(srv)), timeout_s=0.25, **kw)
            c.hello(1.0)
            t0 = time.monotonic()
            ok, lost = run_calls(c, 10)
            self.assertEqual(lost, [])
            self.assertLess(time.monotonic() - t0, 2.0)
            self.assertEqual(fs.held_count.get(RESP), 10)                # every answer was held
            c.close()
            return c, srv
        finally:
            srv.stop()

    def test_fixed_server_new_client(self):
        c, srv = self.check_fixed_server(kv_link)
        self.assertGreaterEqual(c.counts["rx_nudge"], 9)     # the last one is read by no later call
        self.assertEqual(c.counts["poke:hello"], 0)
        self.assertEqual(srv.counts["tx_nudge:burst"], 10)

    @unittest.skipIf(LEGACY is None, "no git history")
    def test_fixed_server_legacy_client(self):
        c, srv = self.check_fixed_server(LEGACY)
        self.assertGreaterEqual(c.counts["unexpected:14"] + c.counts["rx_old_frame_chunks"], 9)
        self.assertEqual(c.counts["answered"], 10)

    def test_burst_nudge_alone_and_idle_nudge_alone_each_suffice(self):
        c, srv = self.check_fixed_server(kv_link, nudge_after_burst=True, nudge_idle_s=0.0)
        self.assertEqual(srv.counts["tx_nudge:idle"], 0)
        c, srv = self.check_fixed_server(kv_link, nudge_after_burst=False, nudge_idle_s=0.002)
        self.assertEqual(srv.counts["tx_nudge:burst"], 0)
        self.assertGreaterEqual(srv.counts["tx_nudge:idle"], 10)

    @unittest.skipIf(LEGACY is None, "no git history")
    def test_new_client_pokes_a_legacy_server(self):
        fs = TailHoldSock()
        srv = legacy_server(fs)
        try:
            c = KvLinkClient(("127.0.0.1", port(srv)), timeout_s=0.5, poke_s=0.05)
            c.hello(1.0)
            ok, lost = run_calls(c, 5)
            self.assertEqual(lost, [])
            self.assertEqual(c.counts["answered_after_poke"], 5)
            self.assertGreaterEqual(c.counts["poke:hello"], 5)
            self.assertEqual(c.counts["timeout"], 0)
            c.close()
        finally:
            srv.stop()

    def test_poke_off_against_a_slow_but_healthy_server_is_silent(self):
        srv = new_server()
        try:
            c = KvLinkClient(("127.0.0.1", port(srv)), timeout_s=1.0)
            c.hello(1.0)
            ok, lost = run_calls(c, 5)
            self.assertEqual(lost, [])
            self.assertEqual(c.counts["poke:hello"], 0)     # answered well within POKE_S
            c.close()
        finally:
            srv.stop()


class HarshModelTests(unittest.TestCase):
    def test_every_last_datagram_held_including_the_nudge(self):
        fs = TailHoldSock(types=None, multi_only=False)
        srv = new_server(fs)
        try:
            c = KvLinkClient(("127.0.0.1", port(srv)), timeout_s=0.25, poke_s=0)
            c.hello(1.0)                    # HELLO_ACK held too; the idle NUDGE releases it
            ok, lost = run_calls(c, 10)
            self.assertEqual(lost, [])
            self.assertGreater(fs.held_count.get(NUDGE, 0), 0)    # NUDGEs were held, harmlessly
            self.assertEqual(c.counts["stale_response"], 0)
            c.close()
        finally:
            srv.stop()


class ControlAnswerTests(unittest.TestCase):
    def test_multi_chunk_stats_ack_tail(self):
        for nudge, expect_ok in ((True, True), (False, False)):
            fs = TailHoldSock(types=(STATS_ACK,))
            kw = {} if nudge else {"nudge_after_burst": False, "nudge_idle_s": 0.0}
            srv = new_server(fs, info={"pad": "x" * 6000}, **kw)
            try:
                c = KvLinkClient(("127.0.0.1", port(srv)), timeout_s=0.25, poke_s=0)
                c.hello(1.0)
                st = c.stats(timeout_s=0.3)
                self.assertEqual(st is not None, expect_ok, nudge)
                self.assertGreaterEqual(fs.held_count.get(STATS_ACK, 0), 1)
                c.close()
            finally:
                srv.stop()

    def telem(self, nudge, resend_s):
        fs = TailHoldSock(types=(TELEM_ACK,), multi_only=False)
        kw = {} if nudge else {"nudge_after_burst": False, "nudge_idle_s": 0.0}
        srv = new_server(fs, **kw)
        try:
            t = KvTelemetryClient(("127.0.0.1", port(srv)), timeout_s=0.2, resend_s=resend_s)
            try:
                tm = t.poll()
                ok = tm is not None
            except LinkTimeout:
                ok = False
            counts = dict(t.counts)
            t.close()
            return ok, counts
        finally:
            srv.stop()

    def test_telem_ack_tail(self):
        ok, _ = self.telem(nudge=True, resend_s=0.0)
        self.assertTrue(ok)
        ok, _ = self.telem(nudge=False, resend_s=0.0)          # positive control
        self.assertFalse(ok)
        ok, counts = self.telem(nudge=False, resend_s=0.05)    # resend makes the KV transmit
        self.assertTrue(ok)
        self.assertGreaterEqual(counts.get("resend:telem_req", 0), 1)


class AccountingTests(unittest.TestCase):
    def test_nudges_counted_apart_and_chunk_loss_unchanged(self):
        srv = new_server()
        try:
            c = KvLinkClient(("127.0.0.1", port(srv)), timeout_s=1.0, poke_s=0)
            c.hello(1.0)
            sess = c.session
            run_calls(c, 5)
            time.sleep(0.05)
            st = srv.window_stats(sess)
            loss = chunk_loss(dict(c.counts), st["session_counts"])
            self.assertEqual(loss["kv_to_nano"], 0)
            self.assertEqual(loss["nano_to_kv"], 0)
            self.assertNotIn("tx_chunks:nudge", st["session_counts"])
            self.assertEqual(srv.counts["tx_nudge:burst"], 5)
            # one idle NUDGE per quiet period (after HELLO_ACK and after each answer), never re-armed by itself
            idle = srv.counts["tx_nudge:idle"]
            time.sleep(0.05)
            self.assertEqual(srv.counts["tx_nudge:idle"], idle)
            self.assertLessEqual(idle, 6)
            self.assertGreaterEqual(c.counts["rx_nudge"], 4)
            self.assertEqual(srv.info["nudge"], {"after_burst": True, "idle_ms": 2.0})
            c.close()
        finally:
            srv.stop()

    def test_clock_sync_client_ignores_nudges(self):
        from host.kv_clock_sync import ClockSyncClient
        srv = new_server()
        try:
            client = ClockSyncClient(("127.0.0.1", port(srv)))
            acks = [client.exchange() for _ in range(5)]
            for _ in range(5):
                acks.append(client.exchange())
                time.sleep(0.01)            # let the idle NUDGE land between exchanges
            self.assertTrue(all(a is not None for a in acks))
            self.assertEqual(client.sync.counts.get("unexpected", 0), 0)
            self.assertGreaterEqual(client.sync.counts.get("nudge", 0), 1)
        finally:
            srv.stop()


if __name__ == "__main__":
    unittest.main()
