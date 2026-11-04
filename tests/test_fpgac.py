"""FPGAC v1 codec, client checks, recording/replay, discovery and Windows portability (spec C, E.2)."""

import ast
import binascii
import collections
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

from pc import fpgac

ROOT = Path(__file__).resolve().parents[1]
K = fpgac.KIND
TOKEN = "t" * 24


def full_state(i=1, source="live", src="live"):
    s = {"v": 1, "seq_state": i, "hub_boot_id": "boot", "nano_mono_ns": i, "source": source,
         "replay_id": None, "state_hz": 10}
    for g in fpgac.STATE_GROUPS:
        s[g] = {"src": src, "age_ms": 1.0, "ok": True, "reason": None}
    return s


class ScriptedHub(object):
    """A raw FPGAC server that answers HELLO then sends a scripted list of raw frames."""

    def __init__(self, frames_after_hello, ack_ver=(1, 0), ack_ok=True, host="127.0.0.1", port=0,
                 token=TOKEN, boot="scripted", hold_s=3.0):
        self.frames, self.ack_ver, self.ack_ok, self.token = frames_after_hello, ack_ver, ack_ok, token
        self.boot, self.hold_s = boot, hold_s
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind((host, port))
        self.listener.listen(8)
        self.port = self.listener.getsockname()[1]
        self.sent_raw = []
        self.connections = 0
        self.received = []
        self._stop = threading.Event()
        self.threads = [threading.Thread(target=self._accept, daemon=True)]
        self.threads[0].start()

    def _accept(self):
        self.listener.settimeout(0.05)
        while not self._stop.is_set():
            try:
                sock, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self.connections += 1
            t = threading.Thread(target=self._serve, args=(sock, self.connections), daemon=True)
            t.start()
            self.threads.append(t)

    def _serve(self, sock, n):
        dec = fpgac.Decoder()
        try:
            sock.settimeout(3)
            hello = None
            while hello is None:
                # 10-17: a peer that closes before its HELLO (fpgac.probe's TCP check) gives b"" at once, every
                # time -- without this return the thread spun at ~1 CPU forever, outliving close() (join 5 s) and
                # starving the GIL of every later test in the same unittest process (LOAD_TIMING_TESTS_20261017)
                data = sock.recv(65536)
                if not data:
                    return
                frames = dec.feed(data)
                hello = frames[0] if frames else None
            ok = self.ack_ok and hello.obj().get("token") == self.token
            ack = fpgac.encode_json(K["HELLO_ACK"], 1, {"v": 1, "ok": ok, "hub_boot_id": self.boot,
                                                      "data_source": "live"}, ver=self.ack_ver)
            sock.sendall(ack)
            if not ok:
                return
            frames = self.frames(n) if callable(self.frames) else self.frames
            for raw in frames:
                sock.sendall(raw)
                self.sent_raw.append(raw)
            sock.settimeout(0.1)
            end = time.monotonic() + self.hold_s
            while time.monotonic() < end and not self._stop.is_set():
                try:
                    data = sock.recv(65536)
                    if not data:
                        return
                    self.received.extend(dec.feed(data))
                except socket.timeout:
                    pass
        except OSError:
            pass
        finally:
            sock.close()

    def close(self):
        self._stop.set()
        self.listener.close()
        for t in self.threads:
            t.join(timeout=5)


def run_client(port, seconds, **kw):
    got = []
    c = fpgac.FpgacClient(port, kw.pop("token", TOKEN), hosts=kw.pop("hosts", ["127.0.0.1"]),
                        on_frame=lambda f, o: got.append((f.kind, o)), backoff_s=(0.1,), **kw).start()
    time.sleep(seconds)
    c.stop()
    return c, got


class FramingTests(unittest.TestCase):
    def test_header_known_answer(self):
        raw = fpgac.encode_json(K["STATE"], 7, {"v": 1})
        body = b'{"v":1}'
        self.assertEqual(raw[:20], struct.pack("<4sBBHIII", b"FPGAC", 1, 0, 3, 7, len(body),
                                               binascii.crc32(body) & 0xFFFFFFFF))
        self.assertEqual(raw[20:], body)
        self.assertEqual(fpgac.HEADER.size, 20)

    def test_byte_by_byte_feed_and_binary(self):
        raws = [fpgac.encode_json(K["EVENT"], 1, {"v": 1, "src": "live", "event": {"event": "x"}}),
                fpgac.encode_binary(K["MAP"], 2, {"which": "rolling", "src": "replay"}, b"\x00\x01\xff")]
        dec, out = fpgac.Decoder(), []
        for b in b"".join(raws):
            out.extend(dec.feed(bytes([b])))
        self.assertEqual([f.kind for f in out], [K["EVENT"], K["MAP"]])
        meta, payload = out[1].obj()
        self.assertEqual((meta["which"], payload), ("rolling", b"\x00\x01\xff"))
        self.assertEqual(out[0].raw, raws[0])

    def test_corruptions_raise(self):
        raw = bytearray(fpgac.encode_json(K["STATE"], 1, full_state()))
        bad_crc = bytearray(raw)
        bad_crc[-2] ^= 0x01
        with self.assertRaises(fpgac.ProtocolError) as cm:
            fpgac.Decoder().feed(bytes(bad_crc))
        self.assertEqual(cm.exception.reason, "bad_crc")
        bad_magic = bytearray(raw)
        bad_magic[0:4] = b"FPGAX"
        with self.assertRaises(fpgac.ProtocolError) as cm:
            fpgac.Decoder().feed(bytes(bad_magic))
        self.assertEqual(cm.exception.reason, "bad_magic")
        too_long = struct.pack("<4sBBHIII", b"FPGAC", 1, 0, 3, 1, fpgac.MAX_BODY + 1, 0)
        with self.assertRaises(fpgac.ProtocolError) as cm:
            fpgac.Decoder().feed(too_long)
        self.assertEqual(cm.exception.reason, "too_long")
        with self.assertRaises(ValueError):
            fpgac.dumps({"x": float("nan")})

    def test_seq_check(self):
        counts = collections.Counter()
        s = fpgac.SeqCheck(counts)
        for n in (1, 2, 4, 3):
            s.check(n)
        self.assertEqual((counts["seq_gap"], counts["seq_back"]), (1, 1))


class SourceFlagTests(unittest.TestCase):
    def test_known_good_and_each_missing_flag(self):
        self.assertIsNone(fpgac.check_source_flags(K["STATE"], full_state()))
        s = full_state()
        del s["source"]
        self.assertEqual(fpgac.check_source_flags(K["STATE"], s), "state.source")
        for group in fpgac.STATE_GROUPS:
            s = full_state()
            del s[group]["src"]
            self.assertEqual(fpgac.check_source_flags(K["STATE"], s), "state.%s.src" % group)
            s = full_state()
            del s[group]
            self.assertEqual(fpgac.check_source_flags(K["STATE"], s), "state.%s.src" % group)
        s = full_state(source="maybe")
        self.assertEqual(fpgac.check_source_flags(K["STATE"], s), "state.source")
        self.assertEqual(fpgac.check_source_flags(K["MEMORY"], {"tracks": []}), "memory.source")
        self.assertEqual(fpgac.check_source_flags(K["EVENT"], {"event": {}}), "event.src")
        self.assertEqual(fpgac.check_source_flags(K["MAP"], ({"which": "global"}, b"")), "map.src")

    def test_client_refuses_frames_without_marks(self):
        bad1 = full_state(2)
        del bad1["source"]
        bad2 = full_state(3)
        del bad2["kv"]["src"]
        frames = [fpgac.encode_json(K["STATE"], 1, full_state(1)), fpgac.encode_json(K["STATE"], 2, bad1),
                  fpgac.encode_json(K["STATE"], 3, bad2), fpgac.encode_json(K["EVENT"], 4, {"v": 1, "event": {}}),
                  fpgac.encode_json(77, 5, {"v": 1})]
        hub = ScriptedHub(frames)
        try:
            c, got = run_client(hub.port, 0.6)
        finally:
            hub.close()
        self.assertEqual([o["seq_state"] for k, o in got if k == K["STATE"]], [1])
        self.assertEqual(c.counts["missing_source_flag"], 3)
        self.assertEqual(c.counts["missing_source_flag:state.source"], 1)
        self.assertEqual(c.counts["missing_source_flag:state.kv.src"], 1)
        self.assertEqual(c.counts["unknown_kind"], 1)


class LinkIntegrityTests(unittest.TestCase):
    def _corrupt_session(self, mutate, reason):
        good = fpgac.encode_json(K["STATE"], 1, full_state(1))
        bad = bytearray(fpgac.encode_json(K["STATE"], 2, full_state(2)))
        mutate(bad)
        hub = ScriptedHub(lambda n: [good, bytes(bad)], hold_s=0.2)
        try:
            c, got = run_client(hub.port, 1.0)
        finally:
            hub.close()
        self.assertGreaterEqual(c.counts["protocol_error:" + reason], 1)
        self.assertGreaterEqual(c.counts["connected"], 2, "must reconnect after an untrusted stream")
        self.assertNotIn(2, [o.get("seq_state") for k, o in got if k == K["STATE"]])

    def test_crc_error_disconnects_and_reconnects(self):
        def flip(b):
            b[-3] ^= 0x40
        self._corrupt_session(flip, "bad_crc")

    def test_magic_error_disconnects_and_reconnects(self):
        def magic(b):
            b[0:4] = b"XXXX"
        self._corrupt_session(magic, "bad_magic")

    def test_major_version_rejected_at_hello(self):
        hub = ScriptedHub([fpgac.encode_json(K["STATE"], 1, full_state(1))], ack_ver=(2, 0))
        try:
            c, got = run_client(hub.port, 0.5)
        finally:
            hub.close()
        self.assertEqual(got, [])
        self.assertEqual(c.counts["connected"], 0)
        self.assertGreaterEqual(c.counts["hello_rejected:None"], 1)

    def test_seq_gap_from_variant_server_is_reported(self):
        frames = [fpgac.encode_json(K["STATE"], n, full_state(n)) for n in (2, 3, 5)]   # HELLO_ACK was 1
        hub = ScriptedHub(frames)
        try:
            c, got = run_client(hub.port, 0.5)
        finally:
            hub.close()
        self.assertEqual(c.counts["seq_gap"], 1)
        self.assertEqual(len(got), 3)

    def test_wrong_token_never_connects(self):
        hub = ScriptedHub([fpgac.encode_json(K["STATE"], 2, full_state(1))])
        try:
            c, got = run_client(hub.port, 0.4, token="wrong-token-wrong-token")
        finally:
            hub.close()
        self.assertEqual((c.counts["connected"], got), (0, []))
        self.assertGreaterEqual(c.counts["hello_rejected:None"], 1)


class RecordReplayTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def _record(self, frames):
        path = str(self.dir / "run.fpgarec")
        rec = fpgac.Recorder(path)
        hub = ScriptedHub(frames)
        try:
            c, got = run_client(hub.port, 0.6, recorder=rec)
        finally:
            hub.close()
            rec.close()
        return path, hub, got

    def test_recording_is_byte_exact_and_crc_verifiable(self):
        frames = [fpgac.encode_json(K["STATE"], n + 1, full_state(n)) for n in range(1, 21)]
        frames.append(fpgac.encode_binary(K["MAP"], 22, {"which": "rolling", "src": "live"}, bytes(range(200))))
        path, hub, got = self._record(frames)
        header, rows = fpgac.read_fpgarec(path)
        self.assertEqual(header["hub_boot_id"], "scripted")
        self.assertEqual(header["hello_ack"]["data_source"], "live")
        raws = [r for _, r in rows]
        self.assertEqual(fpgac.decode_one(raws[0]).kind, K["HELLO_ACK"])
        self.assertEqual(raws[1:], hub.sent_raw)                          # byte for byte
        self.assertEqual(fpgac.verify_fpgarec(path)["kinds"], {"HELLO_ACK": 1, "STATE": 20, "MAP": 1})
        stamps = [t for t, _ in rows]
        self.assertEqual(stamps, sorted(stamps))
        with self.assertRaises(FileExistsError):
            fpgac.Recorder(path)

    def test_replay_forces_replay_and_keeps_orig_even_for_hand_edited_live(self):
        frames = [fpgac.encode_json(K["STATE"], n + 1, full_state(n, source="replay", src="replay"))
                  for n in range(1, 6)]
        path, hub, got = self._record(frames)
        # Hand-edit frame 3 to claim source=live, recomputing its crc (tests forcing, not crc).
        header, rows = fpgac.read_fpgarec(path)
        edited = []
        for i, (t, raw) in enumerate(rows):
            f = fpgac.decode_one(raw)
            if f.kind == K["STATE"] and f.obj()["seq_state"] == 3:
                obj = f.obj()
                obj["source"] = "live"
                obj["kv"]["src"] = "live"
                raw = fpgac.encode_json(K["STATE"], f.seq, obj)
            edited.append((t, raw))
        out = str(self.dir / "edited.fpgarec")
        with open(out, "wb") as fo:
            fo.write(fpgac.REC_MAGIC + fpgac.dumps(header) + b"\n")
            for t, raw in edited:
                fo.write(fpgac.REC_ENTRY.pack(t, len(raw)) + raw)
        replayed = [o for f, o in fpgac.replay_frames(out, rate=0) if f.kind == K["STATE"]]
        self.assertEqual([o["seq_state"] for o in replayed], [1, 2, 3, 4, 5])
        self.assertTrue(all(o["source"] == "replay" and o["replay_file"] == "edited.fpgarec" for o in replayed))
        third = replayed[2]
        self.assertEqual((third["orig_source"], third["kv"]["src"], third["kv"]["orig_src"]),
                         ("live", "replay", "live"))
        self.assertEqual(replayed[0]["orig_source"], "replay")
        # Replay of the unedited file reproduces every recorded STATE body except the replay marks.
        plain = [o for f, o in fpgac.replay_frames(path, rate=0) if f.kind == K["STATE"]]
        for o, raw in zip(plain, hub.sent_raw):
            original = fpgac.decode_one(raw).obj()
            stripped = {k: v for k, v in o.items() if k not in ("orig_source", "replay_file")}
            self.assertEqual(stripped, original)

    def test_replay_server_marks_replay_and_refuses_commands(self):
        frames = [fpgac.encode_json(K["STATE"], n + 1, full_state(n)) for n in range(1, 11)]
        path, hub, got = self._record(frames)
        server = fpgac.ReplayServer(path, TOKEN, rate=0, loop=False).start()
        try:
            got2 = []
            c = fpgac.FpgacClient(server.port, TOKEN, hosts=["127.0.0.1"],
                                on_frame=lambda f, o: got2.append((f.kind, o))).start()
            self.assertTrue(c.connected.wait(5))
            self.assertEqual(c.hello_ack["data_source"], "replay")
            cid = c.command("find", {"text": "bottle"})
            time.sleep(0.6)
            c.stop()
        finally:
            server.close()
        states = [o for k, o in got2 if k == K["STATE"]]
        self.assertEqual(len(states), 10)
        self.assertTrue(all(o["source"] == "replay" and o["orig_source"] == "live" for o in states))
        self.assertTrue(all(o[g]["src"] == "replay" for o in states for g in fpgac.STATE_GROUPS))
        self.assertEqual((c.acks[cid]["status"], c.acks[cid]["reason"]), ("rejected", "replay"))


class DiscoveryTests(unittest.TestCase):
    def _free_port(self, host):
        s = socket.socket()
        s.bind((host, 0))
        port = s.getsockname()[1]
        s.close()
        return port

    def test_scan_finds_only_the_hub_with_this_token_and_boot_id(self):
        port = self._free_port("127.0.0.5")
        good = ScriptedHub([], host="127.0.0.5", port=port, boot="real", hold_s=0.2)
        imposter = ScriptedHub([], host="127.0.0.6", port=port, token="another-token-entirely", hold_s=0.2)
        wrong_boot = ScriptedHub([], host="127.0.0.7", port=port, boot="other", hold_s=0.2)
        try:
            self.assertEqual(fpgac.scan(port, TOKEN, "127.0.0.0/28"), ["127.0.0.5", "127.0.0.7"])
            self.assertEqual(fpgac.scan(port, TOKEN, "127.0.0.0/28", expect_boot_id="real"), ["127.0.0.5"])
        finally:
            for h in (good, imposter, wrong_boot):
                h.close()

    def test_address_change_rediscovered_and_remembered(self):
        port = self._free_port("127.0.0.9")
        state = lambda n: [fpgac.encode_json(K["STATE"], 2, full_state(n))]   # noqa: E731
        first = ScriptedHub(state, host="127.0.0.9", port=port, hold_s=30)
        with tempfile.TemporaryDirectory() as tmp:
            last = os.path.join(tmp, "last.json")
            c = fpgac.FpgacClient(port, TOKEN, hosts=["127.0.0.3"], scan_net="127.0.0.8/29", last_file=last,
                                backoff_s=(0.1,), dead_s=1.0, stale_s=0.5).start()
            try:
                self.assertTrue(c.connected.wait(10))
                self.assertEqual(c.host, "127.0.0.9")
                with open(last) as f:
                    self.assertEqual(json.load(f)["host"], "127.0.0.9")
                first.close()
                second = ScriptedHub(state, host="127.0.0.12", port=port, hold_s=30)
                try:
                    end = time.monotonic() + 15
                    # the client sets .host, writes the last-host file, THEN counts the connection:
                    # wait for both (integration v5: polled between the two under a loaded regression)
                    while ((c.host != "127.0.0.12" or c.counts["connected"] < 2)
                           and time.monotonic() < end):
                        time.sleep(0.05)
                    self.assertEqual(c.host, "127.0.0.12")
                    self.assertGreaterEqual(c.counts["connected"], 2)
                finally:
                    c.stop()
                    second.close()
            finally:
                c.stop()

    def test_probe_distinguishes_isolation_from_service_down(self):
        port = self._free_port("127.0.0.1")
        r = fpgac.probe("127.0.0.1", port, TOKEN, ping=lambda h: True)
        self.assertEqual((r["tcp"], r["verdict"]), ("refused", "host_up_service_down"))
        hub = ScriptedHub([], hold_s=0.2)
        try:
            self.assertEqual(fpgac.probe("127.0.0.1", hub.port, TOKEN, ping=lambda h: True)["verdict"], "hub_ok")
            self.assertEqual(fpgac.probe("127.0.0.1", hub.port, "bad-token-bad-token", ping=lambda h: True)["verdict"],
                             "hub_rejects_token_or_not_fpgac")
        finally:
            hub.close()
        with mock.patch.object(fpgac.socket, "create_connection", side_effect=socket.timeout()):
            r = fpgac.probe("192.0.2.1", 1, TOKEN, ping=lambda h: False)
        self.assertEqual(r["verdict"], "unreachable_isolation_or_other_subnet")

    def test_fixture_hub_ends_a_connection_closed_before_hello(self):
        """10-17 fixture regression: probe's bare TCP check (connect, close) left ScriptedHub's serve thread spinning
        on recv() == b"" for the rest of the unittest process. It must end on its own, well before close()."""
        hub = ScriptedHub([], hold_s=0.2)
        try:
            fpgac.probe("127.0.0.1", hub.port, None, ping=lambda h: True)     # the TCP check only (no token)
            end = time.monotonic() + 10
            while len(hub.threads) < 2 and time.monotonic() < end:
                time.sleep(0.01)
            self.assertEqual(len(hub.threads), 2, "the hub never accepted the probe")
            hub.threads[1].join(timeout=max(0.0, end - time.monotonic()))
            self.assertFalse(hub.threads[1].is_alive(), "serve thread still running after the peer closed")
        finally:
            hub.close()
        self.assertFalse([t for t in hub.threads if t.is_alive()])


POSIX_ONLY = ("os.fork", "signal.SIGALRM", "signal.alarm", "select.poll", "select.epoll", "os.getuid",
              "AF_UNIX", "MSG_DONTWAIT", "SO_REUSEPORT", "os.setsid", "os.killpg", "fcntl", "termios",
              "pwd", "grp", "resource", "os.uname", "/dev/", "/tmp/", "os.geteuid")
WINDOWS_STDLIB = {"argparse", "binascii", "collections", "ipaddress", "json", "math", "os", "platform", "socket",
                  "struct", "subprocess", "sys", "tempfile", "threading", "time", "select", "shutil", "fpgac"}


class PortabilityTests(unittest.TestCase):
    FILES = (ROOT / "pc" / "fpgac.py", ROOT / "pc" / "fpgapc.py")

    def test_only_windows_stdlib_imports_and_no_posix_only_api(self):
        for path in self.FILES:
            src = path.read_text(encoding="utf-8")
            tree = ast.parse(src)
            names = set()
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    names.update(a.name.split(".")[0] for a in node.names)
                elif isinstance(node, ast.ImportFrom):
                    names.add(node.module.split(".")[0])
            self.assertLessEqual(names, WINDOWS_STDLIB, path.name)
            for needle in POSIX_ONLY:
                self.assertNotIn(needle, src, "%s uses %s" % (path.name, needle))

    def test_python38_isolated_import_is_stdlib_only(self):
        code = ("import sys, sysconfig; before=set(sys.modules); sys.path.insert(0, %r); import fpgac, fpgapc; "
                "std=sysconfig.get_paths()['stdlib']; new=set(sys.modules)-before; "
                "bad=[m for m in new if getattr(sys.modules[m],'__file__',None) and "
                "not sys.modules[m].__file__.startswith(std) and m not in ('fpgac','fpgapc')]; "
                "print(len(bad))" % str(ROOT / "pc"))
        out = subprocess.run([sys.executable, "-I", "-S", "-c", code], stdout=subprocess.PIPE,
                             universal_newlines=True, cwd=tempfile.gettempdir(), check=True).stdout
        self.assertEqual(out.strip(), "0")

    def test_keepalive_windows_branch(self):
        class FakeSock(object):
            def __init__(self):
                self.calls = []

            def setsockopt(self, *a):
                self.calls.append(("setsockopt",) + a)

            def ioctl(self, *a):
                self.calls.append(("ioctl",) + a)

        s = FakeSock()
        with mock.patch.object(fpgac.socket, "SIO_KEEPALIVE_VALS", 0x98000004, create=True):
            self.assertEqual(fpgac.set_keepalive(s), "windows")
        self.assertEqual(s.calls[-1], ("ioctl", 0x98000004, (1, 5000, 2000)))
        real = socket.socket()
        try:
            self.assertTrue(fpgac.set_keepalive(real))
            self.assertEqual(real.getsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE) != 0, True)
        finally:
            real.close()

    def test_ping_command_per_platform(self):
        seen = []

        def fake_run(cmd, **kw):
            seen.append(cmd)
            return mock.Mock(returncode=0)

        with mock.patch.object(fpgac.subprocess, "run", fake_run):
            with mock.patch.object(fpgac.platform, "system", return_value="Windows"):
                fpgac.ping_once("203.0.113.1")
            with mock.patch.object(fpgac.platform, "system", return_value="Linux"):
                fpgac.ping_once("203.0.113.1")
        self.assertEqual(seen[0][:3], ["ping", "-n", "1"])
        self.assertEqual(seen[1][:3], ["ping", "-c", "1"])

    def test_cli_selftest_and_config_paths(self):
        out = subprocess.run([sys.executable, "-I", "-S", str(ROOT / "pc" / "fpgapc.py"), "selftest"],
                             stdout=subprocess.PIPE, universal_newlines=True, cwd=tempfile.gettempdir(),
                             timeout=60)
        self.assertEqual(out.returncode, 0, out.stdout)
        self.assertTrue(out.stdout.startswith("SELFTEST OK frames=30"), out.stdout)
        src = (ROOT / "pc" / "fpgapc.py").read_text(encoding="utf-8")
        self.assertIn('os.path.expanduser("~")', src)
        self.assertIn("os.path.join(HERE, \"fpgapc.json\")", src)


class LinkLagTests(unittest.TestCase):
    """Stale-frame flag (controller ruling 10-11 (c)): link delay from the minimum of
    pc_arrival - nano_mono_ns, never from a Nano clock reading on the PC."""

    S = 1000000000

    def test_known_answer_offset_cancels_and_queued_frame_has_its_wait(self):
        lag = fpgac.LinkLag()
        off = 123456789 * self.S                      # arbitrary PC-Nano clock offset
        nano = [k * 100000000 for k in range(30)]     # 10 Hz compose times
        out = [lag.observe(n + off + 2000000, n) for n in nano[:20]]      # 2 ms transit
        self.assertEqual(out, [0.0] * 20)
        # frame 20 composed on time but held 2.5 s in a buffer
        self.assertAlmostEqual(lag.observe(nano[20] + off + 2000000 + 2500000000, nano[20]), 2500.0)

    def test_drift_does_not_accumulate_beyond_the_window(self):
        lag = fpgac.LinkLag(window_s=20.0)
        worst = 0.0
        for k in range(6000):                         # 10 min at 10 Hz, PC clock 100 ppm fast
            n = k * 100000000
            pc = int(n * (1 + 100e-6)) + 7 * self.S
            worst = max(worst, lag.observe(pc, n))
        self.assertLess(worst, 3.0)                   # ~100 ppm x 21 s = 2.1 ms

    def test_positive_control_session_wide_minimum_accumulates_drift(self):
        lag = fpgac.LinkLag(window_s=1e9)              # no window: the drift piles up
        worst = 0.0
        for k in range(6000):
            n = k * 100000000
            worst = max(worst, lag.observe(int(n * (1 + 100e-6)) + 7 * self.S, n))
        self.assertGreater(worst, 50.0)


if __name__ == "__main__":
    unittest.main()
