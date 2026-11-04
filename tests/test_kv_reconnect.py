"""Producer KV260 re-HELLO and worker-thread fail-stop (2026-10-15).

Defect (host-reproduced 10-15): ``KvConnection.connect()`` closed the old client's socket before
the new HELLO. When the re-HELLO failed (kv_server not up yet), the producer core still held the
old RemotePlFrameBackend on a CLOSED socket; the next frame raised ``OSError: [Errno 9] Bad file
descriptor`` out of M7Pipeline (it catches RuntimeError/ValueError/OverflowError only). The shell's
worker swallowed it per frame, and the link-health block (after core.process) was never reached
again: a live process that never produced a frame and never retried.

Fixed: connect() builds and HELLOs the new client first and swaps only on success; on failure the
old session stays as it was (kv_server restarted on the same port answers an old session, so the
next frame either works or counts as a link failure on the existing path). The worker loop
(host.geo_producer.run_worker) keeps "one bad frame never stops the producer" but fails stop
after WORKER_ERROR_LIMIT consecutive frame exceptions or any exception of the loop itself; the
node then exits non-zero (rc 4) so tools/car_run/car_inner.sh's fail-stop takes the stack down.

Known answers, in-process KvLinkServer (CPU frame core) on 127.0.0.1:
  A  re-HELLO with kv_server down: no OSError; next frame backend_failure:LinkTimeout; the old
     client is kept (open socket); the failed new client's socket is closed; server back on the
     same port -> the old session's next frame is geometry_ready without any HELLO.
  B  re-HELLO succeeds (server restarted on the same port): new client swapped in, old socket
     closed, its counters folded into conn.counts; next frame geometry_ready.
  C  same-port restart, no re-HELLO at all: the old session keeps working.
  D  KvReconnect (the shell's link-health rule): 3 link failures -> one re-HELLO per retry period;
     a failed one leaves frames on the counting path; recovery resets the count.
  E  run_worker: one bad frame -> continue; N consecutive -> stop + fatal; take() raising -> fatal.
  W  wiring of host/ros_geo_producer.py (no rclpy on the host interpreters).

Run: PYTHONPATH=. python3 -m unittest discover -s tests -p 'test_kv_reconnect.py' (also /usr/bin/python3.8).
"""
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for p in (str(ROOT), str(ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

from host.kv_link import KvLinkClient, KvLinkServer  # noqa: E402
from host.m7_backend import M7BackendSelector  # noqa: E402
from host.m7_frame_native import FrameNativeM7, compile_frame_native  # noqa: E402
from host.m7_rolling import compile_rolling  # noqa: E402
from host.fpga_clock import now_ns  # noqa: E402

UNIT = 0.001
HD, WD = 64, 96
K = np.array([[80.0, 0.0, 47.5], [0.0, 80.0, 31.5], [0.0, 0.0, 1.0]])


class Env:
    _dir = None

    @classmethod
    def get(cls):
        if cls._dir is None:
            cls._tmp = tempfile.TemporaryDirectory(prefix="test_kv_reconnect_")
            cls._dir = Path(cls._tmp.name)
            cls.frame_lib = compile_frame_native(cls._dir / "frame.so")
            cls._n = 0
        return cls

    @classmethod
    def rolling(cls):
        cls.get()._n += 1
        return M7BackendSelector().rolling_backend(
            "cpu", cpu_library=compile_rolling(cls._dir / ("roll%d.so" % cls._n)))


def start_server(port=0):
    """CPU-core kv_server on 127.0.0.1:port (a restart rebinds the same port; retried)."""
    core = FrameNativeM7(Env.get().frame_lib).lib.fpga_m7_depth_frame
    for i in range(20):
        try:
            return KvLinkServer(core, "cpu", bind=("127.0.0.1", port)).start()
        except OSError:
            if port == 0 or i == 19:
                raise
            time.sleep(0.1)


def rig_camera():
    import test_kv_ar_slice as sl
    return sl.rig_camera()


class Rig(object):
    """KvConnection + GeoProducer as the shell builds them (roi off: geometry only)."""

    def __init__(self, server_port, client_log=None):
        from host.geo_producer import GeoProducer, KvConnection
        self.made = [] if client_log is None else client_log

        def factory(srv):
            c = KvLinkClient(srv, timeout_s=0.2)
            self.made.append(c)
            return c
        self.conn = KvConnection(("127.0.0.1", server_port), depth_unit_m=UNIT, timeout_s=0.2,
                                 hello_timeout_s=0.3, client_factory=factory, data_source="live")
        assert self.conn.connect(), self.conn.reason
        self.core = GeoProducer(self.conn.backend, Env.rolling(), depth_unit_m=UNIT, roi=False)
        self.T = rig_camera()
        self.n = 0

    def frame(self):
        """nav_reason of one frame (an exception propagates: that is the defect)."""
        self.n += 1
        codes = np.full((HD, WD), 1500, np.uint16)
        res = self.core.process(codes, K, depth_stamp_ns=self.n, acquired_ns=now_ns(), T_odom_camera=self.T,
                                robot_position_m=tuple(self.T[:3, 3]), calibration_id="c1")
        return res["nav_reason"]

    def close(self):
        self.conn.close()


def sock_closed(client):
    return client.sock.fileno() == -1


# ------------------------------------------------------------------ A/B/C: KvConnection
class ReHello(unittest.TestCase):
    def setUp(self):
        self.srv = start_server()
        self.port = self.srv.port
        self.rig = Rig(self.port)
        self.assertEqual(self.rig.frame(), "geometry_ready")

    def tearDown(self):
        self.rig.close()
        if self.srv is not None:
            self.srv.stop()

    def restart(self):
        self.srv = start_server(self.port)

    def test_a_failed_rehello_keeps_session_no_oserror(self):
        conn, core = self.rig.conn, self.rig.core
        self.srv.stop()
        self.srv = None
        self.assertTrue(self.rig.frame().startswith("backend_failure:Link"))
        old_client, old_backend = conn.client, conn.backend
        self.assertFalse(conn.connect())                         # kv_server not up
        self.assertIsNone(conn.reason)                          # old session kept: still connected
        self.assertEqual(conn.last_connect_error, "hello_timeout")
        self.assertEqual(conn.counts["unavailable:hello_timeout"], 1)
        self.assertEqual(conn.counts["kept_previous_session"], 1)
        # next frame: the existing link-failure path, not OSError(EBADF)
        self.assertTrue(self.rig.frame().startswith("backend_failure:Link"))
        self.assertIs(conn.client, old_client)
        self.assertIs(conn.backend, old_backend)
        self.assertIs(core.backend, old_backend)
        self.assertFalse(sock_closed(old_client))
        self.assertEqual(len(self.rig.made), 2)
        self.assertTrue(sock_closed(self.rig.made[1]))          # the failed new client leaks no fd
        d = conn.describe()
        self.assertEqual((d["connected"], d["reason"], d["last_connect_error"]), (True, None, "hello_timeout"))
        # server back on the same port: the old session answers, no HELLO needed
        self.restart()
        self.assertEqual(self.rig.frame(), "geometry_ready")
        self.assertEqual(len(self.rig.made), 2)

    def test_b_rehello_success_swaps_and_closes_old(self):
        conn, core = self.rig.conn, self.rig.core
        self.srv.stop()
        self.srv = None
        self.assertTrue(self.rig.frame().startswith("backend_failure:Link"))
        old_client = conn.client
        old_answered = old_client.counts["answered"]
        self.assertEqual(old_answered, 1)
        self.restart()
        self.assertTrue(conn.connect(), conn.reason)
        self.assertIsNot(conn.client, old_client)
        self.assertTrue(sock_closed(old_client))
        self.assertFalse(sock_closed(conn.client))
        self.assertEqual(conn.counts["link:answered"], old_answered)   # old counters kept
        self.assertEqual(conn.counts["connects"], 2)
        self.assertIsNone(conn.reason)
        self.assertIsNone(conn.describe()["last_connect_error"])
        core.set_backend(conn.backend)
        self.assertEqual(self.rig.frame(), "geometry_ready")
        self.assertEqual(conn.client.counts["answered"], 1)

    def test_c_same_port_restart_old_session_answers(self):
        self.srv.stop()
        self.srv = None
        self.assertTrue(self.rig.frame().startswith("backend_failure:Link"))
        self.restart()
        self.assertEqual(self.rig.frame(), "geometry_ready")
        self.assertEqual(len(self.rig.made), 1)                 # no HELLO since the first


class FirstConnect(unittest.TestCase):
    def test_no_server_at_start(self):
        from host.geo_producer import KvConnection
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()                                               # nothing listens there
        conn = KvConnection(("127.0.0.1", port), depth_unit_m=UNIT, hello_timeout_s=0.2)
        self.assertFalse(conn.connect())
        self.assertIsNone(conn.client)
        self.assertIsNone(conn.backend)
        self.assertEqual((conn.reason, conn.last_connect_error), ("hello_timeout", "hello_timeout"))
        self.assertFalse(conn.describe()["connected"])


# ------------------------------------------------------------------ D: link-health rule
class FakeClock(object):
    def __init__(self):
        self.t = 10 ** 12

    def __call__(self):
        return self.t


class LinkHealth(unittest.TestCase):
    def setUp(self):
        self.srv = start_server()
        self.port = self.srv.port
        self.rig = Rig(self.port)
        self.assertEqual(self.rig.frame(), "geometry_ready")

    def tearDown(self):
        self.rig.close()
        if self.srv is not None:
            self.srv.stop()

    def health(self, clock):
        from host.geo_producer import KvReconnect
        return KvReconnect(self.rig.conn, reconnect_after=3, retry_s=5.0, roi_enabled=True, clock=clock)

    def step(self, h):
        reason = self.rig.frame()
        return reason, h.after_frame(reason, self.rig.core.set_backend)

    def test_failed_rehello_then_same_port_recovery(self):
        clock = FakeClock()
        h = self.health(clock)
        self.srv.stop()
        self.srv = None
        clock.t += int(6e9)                                     # past one retry period
        for i in range(2):
            r, act = self.step(h)
            self.assertTrue(r.startswith("backend_failure:Link"))
            self.assertIsNone(act)
        r, act = self.step(h)                                   # 3rd failure -> re-HELLO, fails
        self.assertEqual(act, "reconnect_failed")
        self.assertEqual(h.failures, 3)
        for i in range(3):                                      # rate limited: no new HELLO
            r, act = self.step(h)
            self.assertTrue(r.startswith("backend_failure:Link"), r)    # never OSError
            self.assertIsNone(act)
        self.assertEqual(len(self.rig.made), 2)
        self.restart_same_port()
        r, act = self.step(h)
        self.assertEqual((r, act), ("geometry_ready", None))    # old session, failures reset
        self.assertEqual(h.failures, 0)
        self.assertEqual(dict(h.counts), {"reconnect_failed": 1})

    def test_rehello_success_after_restart(self):
        clock = FakeClock()
        h = self.health(clock)
        old_client = self.rig.conn.client
        self.srv.stop()
        self.srv = None
        for i in range(3):                                      # within the retry period: no HELLO
            r, act = self.step(h)
            self.assertTrue(r.startswith("backend_failure:Link"), r)
            self.assertIsNone(act)
        self.assertEqual(h.failures, 3)
        self.restart_same_port()
        clock.t += int(6e9)
        # the reconnect branch on one more link failure (a real frame would be answered by the
        # restarted server on the old session, so it is fed as a reason)
        act = h.after_frame("backend_failure:LinkTimeout", self.rig.core.set_backend)
        self.assertEqual(act, "reconnected")
        self.assertEqual(h.failures, 0)
        self.assertIsNot(self.rig.conn.client, old_client)
        self.assertTrue(sock_closed(old_client))
        self.assertIs(self.rig.core.backend, self.rig.conn.backend)
        self.assertFalse(self.rig.conn.backend.roi_enabled)     # no roi_ext on this server
        self.assertEqual(self.step(h), ("geometry_ready", None))
        self.assertEqual(dict(h.counts), {"reconnects": 1})

    def restart_same_port(self):
        self.srv = start_server(self.port)


# ------------------------------------------------------------------ E: worker loop
class WorkerLoop(unittest.TestCase):
    def run_loop(self, items, handle, limit=3, take_raises_at=None):
        from host.geo_producer import run_worker
        stop = threading.Event()
        it = iter(items)
        n = [0]
        errors = []

        def take():
            n[0] += 1
            if take_raises_at is not None and n[0] == take_raises_at:
                raise OSError(9, "Bad file descriptor")
            try:
                return next(it)
            except StopIteration:
                stop.set()
                return None
        seen = []

        def on_fatal(why):
            seen.append((why, stop.is_set()))               # the reason is out before stop is set
        fatal = run_worker(stop, take, handle, on_error=lambda exc, streak: errors.append((type(exc).__name__, streak)),
                           limit=limit, on_fatal=on_fatal)
        self.assertEqual(seen, [] if fatal is None else [(fatal, False)])
        return fatal, stop.is_set(), errors

    def test_one_bad_frame_continues(self):
        done = []

        def handle(x):
            if x in (2, 4):
                raise OSError(9, "Bad file descriptor")
            done.append(x)
        fatal, stopped, errors = self.run_loop([1, 2, 3, 4, 5], handle)
        self.assertIsNone(fatal)
        self.assertEqual(done, [1, 3, 5])
        self.assertEqual(errors, [("OSError", 1), ("OSError", 1)])

    def test_consecutive_errors_fail_stop(self):
        def handle(x):
            if x >= 2:
                raise OSError(9, "Bad file descriptor")
        fatal, stopped, errors = self.run_loop(list(range(10)), handle, limit=3)
        self.assertEqual(fatal, "worker_errors:3:OSError")
        self.assertTrue(stopped)
        self.assertEqual([s for _, s in errors], [1, 2, 3])

    def test_loop_exception_is_fatal(self):
        fatal, stopped, errors = self.run_loop([1, 2, 3], lambda x: None, take_raises_at=2)
        self.assertEqual(fatal, "worker_crashed:OSError")
        self.assertTrue(stopped)

    def test_error_handler_raising_is_fatal(self):
        from host.geo_producer import run_worker
        stop = threading.Event()

        def bad_handler(exc, streak):
            raise RuntimeError("logger gone")
        fatal = run_worker(stop, lambda: 1, lambda x: 1 / 0, on_error=bad_handler, limit=50)
        self.assertEqual(fatal, "worker_crashed:RuntimeError")
        self.assertTrue(stop.is_set())

    def test_limit_zero_never_stops_on_frames(self):
        n = [0]
        stop = threading.Event()
        from host.geo_producer import run_worker

        def handle(x):
            n[0] += 1
            if n[0] >= 200:
                stop.set()
            raise ValueError("x")
        self.assertIsNone(run_worker(stop, lambda: 1, handle, limit=0))
        self.assertEqual(n[0], 200)

    def test_default_limit(self):
        from host.geo_producer import WORKER_ERROR_LIMIT
        self.assertEqual(WORKER_ERROR_LIMIT, 0)    # 10-26 owner: never exits on frame errors (was 50)


# ------------------------------------------------------------------ W: shell wiring
class ShellWiring(unittest.TestCase):
    SRC = (ROOT / "host" / "ros_geo_producer.py").read_text()

    def test_worker_delegates_and_exit_code(self):
        self.assertIn("run_worker(self.stop, lambda: self.slot.take(timeout=0.1), self.one", self.SRC)
        self.assertIn("node.worker.is_alive()", self.SRC)
        self.assertIn("if node.fatal is not None:\n        return 4", self.SRC)
        self.assertIn('"fatal": node.fatal', self.SRC)

    def test_link_health_delegates(self):
        self.assertIn("self.health.after_frame(res[\"nav_reason\"], core.set_backend)", self.SRC)
        self.assertNotIn("conn.connect():\n                        conn.backend.roi_enabled", self.SRC)


if __name__ == "__main__":
    unittest.main()
