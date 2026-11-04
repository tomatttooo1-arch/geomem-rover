"""10-21 direct-LAN console: pushed preview, q5z mapping relay, ssh -L Host/Origin ports. No sockets/ROS."""
import json
import struct
import sys
import types
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ui/geomem/bridge"))
sys.path.insert(0, str(ROOT / "ui/geomem/tools/competition"))
import geomem_bridge as gb  # noqa: E402
import gmb_ws as ws  # noqa: E402

try:
    import numpy as np
    import live_mapping as lm
except ImportError:
    np = lm = None


def split(frame):
    (L,) = struct.unpack_from("<I", frame, 0)
    head = json.loads(frame[4:4 + L].decode("utf-8"))
    return head, frame[4 + L + ((-(4 + L)) % 4):]


class Sess(object):
    def __init__(self):
        self.preview, self.mapping = [], []

    def put_preview(self, d):
        self.preview.append(d)

    def put_mapping(self, d):
        self.mapping.append(d)


class ConsoleDirectTests(unittest.TestCase):
    def test_preview_is_pushed_with_its_header(self):
        b = gb.Bridge(clock=lambda: 10.0)
        b.client = types.SimpleNamespace(link_state="up")
        s = Sess()
        b.sessions.add(s)
        jpeg = b"\xff\xd8" + b"x" * 50000 + b"\xff\xd9"            # over the old 30 KiB cap
        b._on_frame("PREVIEW", ({"encoding": "jpeg", "w": 640, "h": 480, "age_ms": 12}, jpeg))
        head, body = split(s.preview[0])
        self.assertEqual((head["type"], head["w"], head["h"], head["seq"]), ("preview", 640, 480, 1))
        self.assertEqual(body, jpeg)
        b.client.link_state = "down"                                # no push while the hub link is down
        b._on_frame("PREVIEW", ({"encoding": "jpeg", "w": 640, "h": 480, "age_ms": 12}, jpeg))
        self.assertEqual(len(s.preview), 1)

    @unittest.skipIf(np is None, "numpy unavailable")
    def test_q5z_roundtrip_and_mapping_relay(self):
        rs = np.random.RandomState(1)
        xyz = (rs.rand(5000, 3) * [8, 6, 2] - [4, 3, 0.2]).astype("<f4")
        rgb = rs.randint(0, 255, (5000, 3)).astype("u1")
        body = xyz.tobytes() + rgb.tobytes()
        head, packed = lm.pack_q5z(body, 5000)
        raw = zlib.decompress(packed)
        self.assertEqual(len(raw), head["rawBytes"])
        q = np.frombuffer(raw, "<u2", 15000).reshape(3, 5000).T.astype(np.int64) + head["origin"]
        self.assertLess(float(np.abs(q * head["scale"] - xyz).max()), 0.0026)
        self.assertTrue((np.frombuffer(raw, "u1", 15000, 30000).reshape(3, 5000).T == rgb).all())
        self.assertLess(len(packed), head["rawBytes"] * 1.01)        # random data: 9 B/pt bound (vs 15 raw)

        status = {"format": "geomem-live-mapping", "version": 1, "source": "rtabmap", "session": "r1", "scope": "s1",
                  "frame": "map", "count": 5000, "sha256": "a" * 64, "graphNodes": 3, "graphAgeMs": 5.0,
                  "cloudAgeMs": 5.0, "revision": 4, "mapToOdom": {"position": [0, 0, 0], "quaternion": [0, 0, 0, 1]}}
        now = [100.0]
        b = gb.Bridge(clock=lambda: now[0], mapping_feed=("127.0.0.1", 47802))
        calls = []

        def feed_get(path, timeout=8.0, limit=0):
            calls.append(path)
            if path == "/status":
                return 200, "application/json", json.dumps(status).encode()
            h = json.dumps(dict(status, **head)).encode()
            return 200, "application/octet-stream", struct.pack("<I", len(h)) + h + packed
        b.feed_get = feed_get
        s = Sess()
        b.sessions.add(s)
        t = b._mapping_poll(-1e9, -1e9)
        mh, mbody = split(s.mapping[0])
        self.assertEqual((mh["type"], mh["encoding"], mbody), ("mapping", "q5z", packed))
        now[0] += 2.5                                               # unchanged cloud: header-only keep-alive
        b._mapping_poll(*t)
        kh, kbody = split(s.mapping[1])
        self.assertEqual((kbody, kh["unchanged"]), (b"", True))
        self.assertEqual(calls.count("/cloud.bin?format=q5z"), 1)   # the full cloud went over the wire once
        late = Sess()
        b.register = gb.Bridge.register.__get__(b)
        late.put_hello = late.put_cloud = lambda *_: None
        b.register(late)                                            # a new page gets the cached cloud at once
        self.assertEqual(late.mapping[0], s.mapping[0])

    def test_bulk_window_until_acked(self):
        """at most BULK_WINDOW unacked bulk frames per page (1 = stop-and-wait; 2 since 92ffcd5, 10-22);
        state frames are never held back; a lost ack never wedges the stream. Both windows checked."""
        kinds = lambda batch: [k for k, _ in batch]                  # noqa: E731
        saved = gb.BULK_WINDOW
        try:
            for window in (1, 2):
                gb.BULK_WINDOW = window
                b = gb.Bridge(clock=lambda: 10.0)
                s = gb.WsSession(None, None, b)
                for k in range(window):                              # fill the window
                    s.put_preview(b"w%d" % k)
                    self.assertEqual(kinds(s._next_batch()), ["preview"], window)
                s.put_preview(b"p2")
                s.put_preview(b"p3")                                 # latest only: p2 is dropped
                s.put_snapshot({"x": 1})
                self.assertEqual(kinds(s._next_batch()), ["snapshot"], window)   # state flows, preview waits
                s.on_frame_ack()
                batch = s._next_batch()
                self.assertEqual(kinds(batch), ["preview"], window)
                self.assertTrue(batch[0][1].endswith(b"p3"))
                s.bulk_sent_t -= gb.BULK_ACK_TIMEOUT_S + 0.1         # a lost ack never wedges the stream
                s.put_mapping(b"m")
                self.assertEqual(kinds(s._next_batch()), ["mapping"], window)
        finally:
            gb.BULK_WINDOW = saved

    def test_ssh_forwarded_port_is_accepted_same_origin_only(self):
        def h(host, origin):
            return {"Host": host, "Origin": origin, "Upgrade": "websocket", "Connection": "Upgrade",
                    "Sec-WebSocket-Version": "13", "Sec-WebSocket-Key": "dGhlIHNhbXBsZSBub25jZQ=="}
        ports = (8790, 8788)
        self.assertTrue(ws.check_upgrade("GET", h("127.0.0.1:8788", "http://127.0.0.1:8788"), ports)[0])
        self.assertTrue(ws.check_upgrade("GET", h("127.0.0.1:8790", "http://127.0.0.1:8790"), ports)[0])
        self.assertFalse(ws.check_upgrade("GET", h("127.0.0.1:8788", "http://127.0.0.1:8790"), ports)[0])
        self.assertFalse(ws.check_upgrade("GET", h("127.0.0.1:9999", "http://127.0.0.1:9999"), ports)[0])
        self.assertFalse(ws.check_upgrade("GET", h("127.0.0.1:8788", "http://127.0.0.1:8788"), 8790)[0])


if __name__ == "__main__":
    unittest.main()
