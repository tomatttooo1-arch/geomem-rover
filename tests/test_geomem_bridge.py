"""GeoMem bridge (ui/geomem/bridge): RFC 6455 codec, HTTP-side checks, protocol v2 framing and the
bridge core (design ui/geomem/docs/PROTOCOL_V2_DESIGN.md §2-§3). No listening sockets: pure functions,
socket.socketpair() and an injected fake FPGAC client."""

import base64
import json
import math
import os
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ui" / "geomem" / "bridge"))

import gmb_proto as gp  # noqa: E402
import gmb_ws as ws  # noqa: E402

MASK = b"\x11\x22\x33\x44"


def cframe(op, payload=b"", fin=True, mask=MASK):
    return ws.encode_frame(op, payload, fin=fin, mask_key=mask)


class WsFrameTests(unittest.TestCase):
    def feed_all(self, data, **kw):
        return ws.FrameDecoder(**kw).feed(data)

    def test_mask_is_involution_and_known_answer(self):
        self.assertEqual(ws.apply_mask(b"\x00\x00\x00\x00\x00", MASK), MASK + b"\x11")
        data = os.urandom(1001)
        self.assertEqual(ws.apply_mask(ws.apply_mask(data, MASK), MASK), data)
        self.assertEqual(ws.apply_mask(b"", MASK), b"")

    def test_rfc_example_masked_hello(self):
        # RFC 6455 §5.7: masked "Hello"
        raw = bytes([0x81, 0x85, 0x37, 0xfa, 0x21, 0x3d, 0x7f, 0x9f, 0x4d, 0x51, 0x58])
        self.assertEqual(self.feed_all(raw), [("text", "Hello")])
        # and the unmasked server frame
        self.assertEqual(ws.encode_frame(ws.OP_TEXT, b"Hello"), bytes([0x81, 0x05]) + b"Hello")

    def test_three_length_encodings_roundtrip(self):
        for n, head_len in ((0, 2), (125, 2), (126, 4), (65535, 4), (65536, 10), (70000, 10)):
            payload = bytes(range(256)) * (n // 256) + bytes(range(n % 256))
            server = ws.encode_frame(ws.OP_BINARY, payload)
            self.assertEqual(len(server), head_len + n, n)
            client = cframe(ws.OP_BINARY, payload)
            self.assertEqual(len(client), head_len + 4 + n, n)
            self.assertEqual(self.feed_all(client), [("binary", payload)], n)

    def test_byte_by_byte_feed(self):
        raw = cframe(ws.OP_TEXT, "杯子".encode("utf-8")) + cframe(ws.OP_PING, b"p")
        dec = ws.FrameDecoder()
        events = []
        for i in range(len(raw)):
            events += dec.feed(raw[i:i + 1])
        self.assertEqual(events, [("text", "杯子"), ("ping", b"p")])

    def test_non_minimal_lengths_rejected(self):
        b0 = 0x82
        sixteen = struct.pack("!BBH", b0, 0x80 | 126, 125) + MASK + b"x" * 125
        with self.assertRaises(ws.WsError) as cm:
            self.feed_all(sixteen)
        self.assertEqual(cm.exception.code, ws.CLOSE_PROTOCOL)
        sixty4 = struct.pack("!BBQ", b0, 0x80 | 127, 65535) + MASK
        with self.assertRaises(ws.WsError) as cm:
            self.feed_all(sixty4)
        self.assertEqual(cm.exception.code, ws.CLOSE_PROTOCOL)
        top = struct.pack("!BBQ", b0, 0x80 | 127, 1 << 63) + MASK
        with self.assertRaises(ws.WsError) as cm:
            self.feed_all(top)
        self.assertEqual(cm.exception.code, ws.CLOSE_PROTOCOL)

    def test_unmasked_client_frame_1002(self):
        with self.assertRaises(ws.WsError) as cm:
            self.feed_all(ws.encode_frame(ws.OP_TEXT, b"hi"))
        self.assertEqual(cm.exception.code, ws.CLOSE_PROTOCOL)

    def test_fragmentation_with_interleaved_ping(self):
        raw = (cframe(ws.OP_TEXT, b"ab", fin=False) + cframe(ws.OP_PING, b"1")
               + cframe(ws.OP_CONT, b"cd", fin=False) + cframe(ws.OP_PONG, b"2")
               + cframe(ws.OP_CONT, b"ef", fin=True))
        self.assertEqual(self.feed_all(raw), [("ping", b"1"), ("pong", b"2"), ("text", "abcdef")])
        raw = cframe(ws.OP_BINARY, b"\x00", fin=False) + cframe(ws.OP_CONT, b"\xff")
        self.assertEqual(self.feed_all(raw), [("binary", b"\x00\xff")])

    def test_fragmentation_errors(self):
        with self.assertRaises(ws.WsError) as cm:
            self.feed_all(cframe(ws.OP_CONT, b"x"))
        self.assertEqual(cm.exception.code, ws.CLOSE_PROTOCOL)
        with self.assertRaises(ws.WsError) as cm:
            self.feed_all(cframe(ws.OP_TEXT, b"a", fin=False) + cframe(ws.OP_TEXT, b"b"))
        self.assertEqual(cm.exception.code, ws.CLOSE_PROTOCOL)

    def test_control_frame_limits(self):
        # fragmented ping (FIN clear) and > 125 byte ping, hand-built since encode_frame refuses
        raw = struct.pack("!BB", 0x09, 0x80 | 1) + MASK + ws.apply_mask(b"x", MASK)
        with self.assertRaises(ws.WsError) as cm:
            self.feed_all(raw)
        self.assertEqual(cm.exception.code, ws.CLOSE_PROTOCOL)
        raw = struct.pack("!BBH", 0x89, 0x80 | 126, 126) + MASK + b"x" * 126
        with self.assertRaises(ws.WsError) as cm:
            self.feed_all(raw)
        self.assertEqual(cm.exception.code, ws.CLOSE_PROTOCOL)
        with self.assertRaises(ValueError):
            ws.encode_frame(ws.OP_PING, b"x" * 126)
        with self.assertRaises(ValueError):
            ws.encode_frame(ws.OP_CLOSE, b"", fin=False)

    def test_reserved_opcodes_and_rsv_1002(self):
        for op in (0x3, 0x4, 0x5, 0x6, 0x7, 0xB, 0xC, 0xD, 0xE, 0xF):
            raw = struct.pack("!BB", 0x80 | op, 0x80) + MASK
            with self.assertRaises(ws.WsError) as cm:
                self.feed_all(raw)
            self.assertEqual(cm.exception.code, ws.CLOSE_PROTOCOL, op)
        for rsv in (0x40, 0x20, 0x10):
            raw = struct.pack("!BB", 0x80 | rsv | ws.OP_TEXT, 0x80) + MASK
            with self.assertRaises(ws.WsError) as cm:
                self.feed_all(raw)
            self.assertEqual(cm.exception.code, ws.CLOSE_PROTOCOL, rsv)

    def test_oversize_frame_and_message_1009(self):
        # positive control: exactly at the cap passes
        self.assertEqual(self.feed_all(cframe(ws.OP_BINARY, b"x" * 100), max_message=100),
                         [("binary", b"x" * 100)])
        # a single frame over the cap is refused from the header alone (no payload needed)
        head = struct.pack("!BBQ", 0x82, 0x80 | 127, (1 << 20) + 1) + MASK
        with self.assertRaises(ws.WsError) as cm:
            self.feed_all(head)
        self.assertEqual(cm.exception.code, ws.CLOSE_TOO_BIG)
        # fragments summing over the cap
        raw = cframe(ws.OP_TEXT, b"a" * 60, fin=False) + cframe(ws.OP_CONT, b"a" * 41)
        with self.assertRaises(ws.WsError) as cm:
            self.feed_all(raw, max_message=100)
        self.assertEqual(cm.exception.code, ws.CLOSE_TOO_BIG)

    def test_invalid_utf8_1007_single_and_across_fragments(self):
        with self.assertRaises(ws.WsError) as cm:
            self.feed_all(cframe(ws.OP_TEXT, b"\xff\xfe"))
        self.assertEqual(cm.exception.code, ws.CLOSE_BAD_DATA)
        good = "杯".encode("utf-8")              # 3 bytes split across fragments: valid
        raw = cframe(ws.OP_TEXT, good[:1], fin=False) + cframe(ws.OP_CONT, good[1:])
        self.assertEqual(self.feed_all(raw), [("text", "杯")])
        bad = cframe(ws.OP_TEXT, good[:2], fin=False) + cframe(ws.OP_CONT, b"A")
        with self.assertRaises(ws.WsError) as cm:
            self.feed_all(bad)
        self.assertEqual(cm.exception.code, ws.CLOSE_BAD_DATA)
        truncated = cframe(ws.OP_TEXT, good[:2], fin=False) + cframe(ws.OP_CONT, b"", fin=True)
        with self.assertRaises(ws.WsError) as cm:
            self.feed_all(truncated)
        self.assertEqual(cm.exception.code, ws.CLOSE_BAD_DATA)

    def test_close_frames(self):
        self.assertEqual(self.feed_all(cframe(ws.OP_CLOSE, b"")), [("close", (None, ""))])
        self.assertEqual(self.feed_all(cframe(ws.OP_CLOSE, ws.close_payload(1000, "bye"))),
                         [("close", (1000, "bye"))])
        for code in (3000, 4999, 1011):
            self.assertEqual(self.feed_all(cframe(ws.OP_CLOSE, struct.pack("!H", code)))[0][1][0], code)
        for code in (999, 1004, 1005, 1006, 1015, 1016, 2999, 5000):
            with self.assertRaises(ws.WsError) as cm:
                self.feed_all(cframe(ws.OP_CLOSE, struct.pack("!H", code)))
            self.assertEqual(cm.exception.code, ws.CLOSE_PROTOCOL, code)
        with self.assertRaises(ws.WsError) as cm:
            self.feed_all(cframe(ws.OP_CLOSE, b"\x03"))
        self.assertEqual(cm.exception.code, ws.CLOSE_PROTOCOL)
        with self.assertRaises(ws.WsError) as cm:
            self.feed_all(cframe(ws.OP_CLOSE, struct.pack("!H", 1000) + b"\xff"))
        self.assertEqual(cm.exception.code, ws.CLOSE_BAD_DATA)

    def test_nothing_after_close(self):
        dec = ws.FrameDecoder()
        out = dec.feed(cframe(ws.OP_CLOSE, b"") + cframe(ws.OP_TEXT, b"late"))
        self.assertEqual(out, [("close", (None, ""))])
        with self.assertRaises(ws.WsError):
            dec.feed(b"")

    def test_close_payload_truncates_on_utf8_boundary(self):
        p = ws.close_payload(1008, "拒" * 100)
        self.assertLessEqual(len(p), 125)
        self.assertEqual(ws.parse_close(p)[0], 1008)
        p[2:].decode("utf-8")

    def test_server_frame_cap(self):
        with self.assertRaises(ValueError):
            ws.encode_frame(ws.OP_BINARY, b"\x00" * (ws.SERVER_MAX_FRAME + 1))


class _H(dict):
    """Case-insensitive header mapping like http.client.HTTPMessage."""

    def get(self, k, d=None):
        for key, v in self.items():
            if key.lower() == k.lower():
                return v
        return d


def upgrade_headers(port=8788, **over):
    h = {"Host": "127.0.0.1:%d" % port, "Origin": "http://127.0.0.1:%d" % port,
         "Upgrade": "websocket", "Connection": "keep-alive, Upgrade",
         "Sec-WebSocket-Version": "13", "Sec-WebSocket-Key": "dGhlIHNhbXBsZSBub25jZQ=="}
    for k, v in over.items():
        k = k.replace("_", "-")
        if v is None:
            h.pop(k, None)
        else:
            h[k] = v
    return _H(h)


class WsHttpCheckTests(unittest.TestCase):
    def test_accept_key_rfc_example(self):
        self.assertEqual(ws.accept_key("dGhlIHNhbXBsZSBub25jZQ=="), "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=")

    def test_upgrade_positive(self):
        for host, origin in (("127.0.0.1:8788", "http://127.0.0.1:8788"),
                             ("localhost:8788", "http://localhost:8788"),
                             ("LOCALHOST:8788", "http://localhost:8788")):
            ok, status, why, acc = ws.check_upgrade("GET", upgrade_headers(Host=host, Origin=origin), 8788)
            self.assertEqual((ok, status, why), (True, 101, None), host)
            self.assertEqual(acc, "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=")
        ok = ws.check_upgrade("GET", upgrade_headers(Upgrade="WebSocket", Connection="Upgrade"), 8788)
        self.assertTrue(ok[0])

    def test_upgrade_negative(self):
        cases = [
            (dict(Host="evil.example:8788"), 403, "host"),
            (dict(Host="127.0.0.1:9999"), 403, "host"),
            (dict(Host="127.0.0.1"), 403, "host"),
            (dict(Host=None), 403, "host"),
            (dict(Origin=None), 403, "origin_missing"),
            (dict(Origin="http://evil.example"), 403, "origin"),
            (dict(Origin="http://127.0.0.1:9999"), 403, "origin"),
            (dict(Origin="https://127.0.0.1:8788"), 403, "origin"),
            (dict(Origin="null"), 403, "origin"),
            (dict(Origin="http://127.0.0.1:8788.evil.example"), 403, "origin"),
            (dict(Upgrade="h2c"), 400, "upgrade"),
            (dict(Connection="keep-alive"), 400, "connection"),
            (dict(Sec_WebSocket_Version="8"), 426, "version"),
            (dict(Sec_WebSocket_Key="short"), 400, "key"),
            (dict(Sec_WebSocket_Key=base64.b64encode(b"x" * 15).decode()), 400, "key"),
            (dict(Sec_WebSocket_Key=None), 400, "key"),
        ]
        for over, status, why in cases:
            ok, st, w, acc = ws.check_upgrade("GET", upgrade_headers(**over), 8788)
            self.assertEqual((ok, st, w, acc), (False, status, why, None), over)
        self.assertEqual(ws.check_upgrade("POST", upgrade_headers(), 8788)[:3], (False, 405, "method"))

    def test_check_host(self):
        self.assertTrue(ws.check_host(_H({"Host": "localhost:80"}), 80))
        self.assertFalse(ws.check_host(_H({"Host": "lo:80"}), 80))
        self.assertFalse(ws.check_host(_H({"Host": "localhost:8080"}), 80))
        self.assertFalse(ws.check_host(_H({}), 80))


class StaticPathTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="gmb_static_")
        self.app = os.path.join(self.tmp, "app")
        os.makedirs(os.path.join(self.app, "src"))
        os.makedirs(os.path.join(self.app, "notlisted"))
        for rel in ("index.html", "src/app.js", "notlisted/x.txt"):
            with open(os.path.join(self.app, rel), "w") as f:
                f.write("x")
        with open(os.path.join(self.tmp, "secret.txt"), "w") as f:
            f.write("s")

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_positive(self):
        self.assertEqual(ws.safe_static_path("/", self.app), os.path.realpath(os.path.join(self.app, "index.html")))
        self.assertEqual(ws.safe_static_path("/src/app.js", self.app),
                         os.path.realpath(os.path.join(self.app, "src", "app.js")))
        self.assertEqual(ws.safe_static_path("/src/%61pp.js", self.app),
                         os.path.realpath(os.path.join(self.app, "src", "app.js")))

    def test_traversal_and_whitelist(self):
        for p in ("/../secret.txt", "/src/../../secret.txt", "/src/%2e%2e/%2e%2e/secret.txt",
                  "/src/%2E%2E/%2E%2E/secret.txt", "/notlisted/x.txt", "/src/..%5c..%5csecret.txt",
                  "/src/app.js%00", "/src", "/src/missing.js", "/%ff", "/secret.txt"):
            self.assertIsNone(ws.safe_static_path(p, self.app), p)

    @unittest.skipIf(os.name == "nt", "symlinks need privileges on Windows")
    def test_symlink_out_of_tree(self):
        os.symlink(os.path.join(self.tmp, "secret.txt"), os.path.join(self.app, "src", "leak.txt"))
        os.symlink(self.tmp, os.path.join(self.app, "vendor"))
        self.assertIsNone(ws.safe_static_path("/src/leak.txt", self.app))
        self.assertIsNone(ws.safe_static_path("/vendor/secret.txt", self.app))


# ---------------------------------------------------------------------------- protocol v2 framing

def grid_meta(w, h, which="global", rev=7, res=0.05, origin=(0.0, 0.0, 0.0), frame="map"):
    return {"which": which, "rev": rev, "width": w, "height": h, "resolution_m": res,
            "origin": list(origin), "frame_id": frame, "encoding": "int8+zlib"}


def grid_payload(cells):
    return zlib.compress(struct.pack("<%db" % len(cells), *cells))


def make_grid(cells, w, h, **kw):
    return gp.decode_grid(grid_meta(w, h, **kw), grid_payload(cells))


class TranslationTests(unittest.TestCase):
    def test_coco_table_complete_and_runner_words(self):
        from perception.fpga_replay.coco_zh import COCO_NAMES, HOUSEHOLD_NAMES
        self.assertEqual(set(gp.COCO_ZH), set(COCO_NAMES) | set(HOUSEHOLD_NAMES))
        self.assertEqual(len(set(gp.COCO_ZH.values())), len(gp.COCO_ZH))
        for en, zh in (("cup", "杯子"), ("bottle", "瓶子"), ("backpack", "书包"), ("book", "书"),
                       ("mouse", "鼠标"), ("keyboard", "键盘"), ("cell phone", "手机"), ("chair", "椅子")):
            self.assertEqual(gp.class_zh(en), zh)
        self.assertEqual(gp.aliases("cup"), ["杯"])
        self.assertEqual(gp.aliases("backpack"), ["背包"])
        self.assertIsNone(gp.aliases("chair"))
        self.assertIsNone(gp.class_zh("spaceship"))
        self.assertFalse(hasattr(gp, "COLOR_ZH"))      # 10-22: no colour words in any Chinese text

    def test_object_name(self):
        # 10-22 (owner): the class name only; the colour guess is not shown anywhere
        self.assertEqual(gp.object_name("cup", "blue"), "杯子")
        self.assertEqual(gp.object_name("backpack", "gray"), "书包")
        self.assertEqual(gp.object_name("cup", None), "杯子")
        self.assertEqual(gp.object_name("cup"), "杯子")
        self.assertIsNone(gp.object_name("spaceship", "red"))
        self.assertIsNone(gp.object_name(None, "red"))

    def test_reason_table(self):
        self.assertEqual(gp.reason_zh("search_exhausted"), "搜索路线走完仍未找到")
        self.assertEqual(gp.reason_zh("cancel current operation and wait for navigation to stop"),
                         "还有任务或导航在进行，请先停止")
        self.assertEqual(gp.reason_zh("no candidate selection is pending"), "当前没有待选择的候选")
        self.assertEqual(gp.reason_zh("not_controller"), "另一台电脑持有控制权")
        self.assertEqual(gp.reason_zh("fault:x"), "车上出错：fault:x")
        self.assertEqual(gp.reason_zh("map_invalid: bad sha"), "map_invalid: bad sha")   # verbatim
        self.assertIsNone(gp.reason_zh(None))
        for code, who, zh in gp.REJECT_CODES:
            self.assertEqual(gp.reason_zh(code), zh)
            self.assertIn(who, ("hub", "bridge"))
        self.assertEqual(len(set(c for c, _, _ in gp.REJECT_CODES)), len(gp.REJECT_CODES))
        for code in ("stale_task", "not_ambiguous", "link_down", "no_ack", "scope_changed",
                     "not_supported", "no_such_candidate", "bad_request", "source_inconsistent"):
            self.assertIn(code, gp.BRIDGE_REJECT_CODES)

    def test_reject_codes_documented_in_readme(self):
        readme = (ROOT / "ui" / "geomem" / "bridge" / "README.md").read_text(encoding="utf-8")
        for code, who, zh in gp.REJECT_CODES:
            self.assertIn("`%s`" % code, readme, code)
            self.assertIn(zh, readme, code)

    def test_find_text_translation(self):
        self.assertEqual(gp.translate_find_text("蓝色杯子"), ("蓝色杯子", None))
        self.assertEqual(gp.translate_find_text("杯"), ("杯", None))
        self.assertEqual(gp.translate_find_text("找书包"), ("找书包", None))
        self.assertEqual(gp.translate_find_text("blue chair"), ("blue chair", None))
        self.assertEqual(gp.translate_find_text("椅子Z"), ("椅子Z", None))   # any ASCII letter: as is
        sent, note = gp.translate_find_text("椅子")
        self.assertEqual(sent, "chair")
        self.assertIn("椅子", note)
        self.assertEqual(gp.translate_find_text("红色椅子")[0], "chair")      # 10-22: colour words dropped
        self.assertEqual(gp.translate_find_text("酒杯")[0], "wine glass")    # longest match beats 杯
        self.assertEqual(gp.translate_find_text("白花瓶")[0], "vase")         # beats 瓶; no colour
        self.assertEqual(gp.translate_find_text("笔记本电脑")[0], "laptop")
        self.assertEqual(gp.translate_find_text("你好"), ("你好", None))
        self.assertEqual(gp.translate_find_text("杯子和椅子"), ("杯子和椅子", None))


class ViewTests(unittest.TestCase):
    def st(self, loc_state=None, source="live", **kw):
        s = {"source": source, "pose": {"x": 1.0, "y": 2.0, "yaw": 0.5, "loc_state": loc_state, "age_ms": 60},
             "odom": {"v": -0.2, "w": 0.0, "age_ms": 30}, "nav": {"motion_permit": False, "cmd_v": 0.0}}
        s.update(kw)
        return s

    def test_tiers(self):
        v = gp.classify_view(self.st(), {"pose_source": "estimated"})
        self.assertEqual((v["tier"], v["frame"]), ("loc", "map"))
        v = gp.classify_view(self.st(loc_state="lost"), None)
        self.assertEqual((v["tier"], v["frame"]), ("loc", "map"))
        v = gp.classify_view(self.st(), {"pose_source": "reference", "sensor_source": "replay"})
        self.assertEqual((v["tier"], v["frame"]), ("odom", "odom"))
        v = gp.classify_view(self.st(source="sim"), {"pose_source": "reference"})
        self.assertEqual((v["tier"], v["frame"]), ("sim", "map"))
        v = gp.classify_view(self.st(), None, {"world_frame": "odom"})
        self.assertEqual((v["tier"], v["frame"]), ("unknown", "odom"))
        v = gp.classify_view(self.st(), None, {"world_frame": "base_link"})
        self.assertEqual((v["tier"], v["frame"], v["frame_clamped"]), ("unknown", "map", True))
        self.assertEqual(gp.classify_view(None, None)["tier"], "unknown")

    def test_robot_by_tier(self):
        ri_e = {"pose_source": "estimated"}
        st = self.st(loc_state="valid")
        r = gp.compose_robot(gp.classify_view(st, ri_e), st, ri_e, "idle", False)
        self.assertEqual((r["position"], r["pose"], r["poseAgeMs"], r["speed"]), ([1.0, 2.0, 0.0], "map", 60.0, 0.2))
        st = self.st(loc_state="lost")
        r = gp.compose_robot(gp.classify_view(st, ri_e), st, ri_e, "idle", False)
        self.assertEqual(r["pose"], "none")
        st = self.st(loc_state="valid")             # loc_state valid but pose_source not estimated
        r = gp.compose_robot(gp.classify_view(st, None), st, None, "idle", False)
        self.assertEqual(r["pose"], "none")
        ri_r = {"pose_source": "reference"}
        st = self.st()                               # odom tier, STATE odom has no x/y before H2
        self.assertIsNone(gp.compose_robot(gp.classify_view(st, ri_r), st, ri_r, "idle", False))
        st = self.st(odom={"x": 3.0, "y": 4.0, "yaw": 0.0, "v": 0.1, "age_ms": 20})
        r = gp.compose_robot(gp.classify_view(st, ri_r), st, ri_r, "idle", False)
        self.assertEqual((r["position"], r["pose"], r["poseAgeMs"]), ([3.0, 4.0, 0.0], "odometry", 20.0))
        st = self.st()
        self.assertIsNone(gp.compose_robot(gp.classify_view(st, None), st, None, "idle", False))
        st = self.st(loc_state="valid", pose={"x": float("nan"), "y": 1.0, "loc_state": "valid"})
        self.assertIsNone(gp.compose_robot(gp.classify_view(st, ri_e), st, ri_e, "idle", False))

    def test_robot_status(self):
        st = self.st(nav={"motion_permit": True, "cmd_v": 0.2})
        self.assertEqual(gp.robot_status(st, "idle", True), "急停锁存")
        self.assertEqual(gp.robot_status(st, "navigating", False), "正在前往")
        self.assertEqual(gp.robot_status(st, "succeeded", False), "行驶中")
        self.assertEqual(gp.robot_status(self.st(), "idle", False), "待命")
        st = self.st(nav={"motion_permit": False, "cmd_v": 0.2})
        self.assertEqual(gp.robot_status(st, "idle", False), "待命")

    def test_robot_status_depth_blocked(self):
        """10-25: >= 3 s of producer frames without valid depth -> the camera-too-close hint (hub depth_invalid_s)"""
        for s, want in ((None, "待命"), (0.0, "待命"), (2.9, "待命"), (3.0, "相机前方过近，请后退"),
                        (850.0, "相机前方过近，请后退"), ("x", "待命")):
            st = self.st(geometry_nano={"depth_invalid_s": s})
            self.assertEqual(gp.robot_status(st, "idle", False), want, s)
        st = self.st(geometry_nano={"depth_invalid_s": 5.0})
        self.assertEqual(gp.robot_status(st, "idle", True), "急停锁存（相机前方过近，请后退）")
        self.assertEqual(gp.robot_status(st, "navigating", False), "相机前方过近，请后退")
        self.assertEqual(gp.robot_status(self.st(), "idle", True), "急停锁存")


class TaskTests(unittest.TestCase):
    def test_state_mapping(self):
        m = gp.task_protocol_state
        self.assertEqual(m(None, None), "idle")
        self.assertEqual(m("idle", None), "idle")
        self.assertEqual(m("ambiguous", "candidate_selection_required"), "ambiguous")
        self.assertEqual(m("navigating", "search"), "searching")
        self.assertEqual(m("navigating", "reobserve"), "navigating")
        self.assertEqual(m("observing", "fresh_visual_evidence_required"), "confirming")
        self.assertEqual(m("found", "fresh_matching_observation"), "succeeded")
        for s in ("not_found", "unable_to_confirm", "failed"):
            self.assertEqual(m(s, "x"), "failed")
        self.assertEqual(m("cancelled", "user_cancelled"), "cancelled")
        self.assertIsNone(m("warping", None))

    def test_idle_object(self):
        t = gp.compose_task({"state": None, "age_ms": 12}, None)
        self.assertEqual(t, {"id": None, "state": "idle", "reasonCode": None, "message": "待命",
                             "targetId": None, "ageMs": 12.0, "candidates": []})
        self.assertEqual(gp.compose_task(None, None)["state"], "idle")

    def test_ambiguous_and_messages(self):
        g = {"state": "ambiguous", "reason_task": "candidate_selection_required", "target_id": None, "age_ms": 5,
             "candidates": [{"id": "object-0001", "label": "cup", "color": "blue", "position_m": [1, 2, 0.5],
                             "state": "current_confirmed"},
                            {"id": "object-0002", "label": "cup", "color": None, "position_m": [3, 4],
                             "state": "weird"},
                            {"id": "object-0003", "label": "cup", "position_m": [float("inf"), 0, 0]}]}
        counts = {}
        t = gp.compose_task(g, "b:3", counts)
        self.assertEqual(t["state"], "ambiguous")
        self.assertEqual(t["message"], "记忆里有 2 个杯子，请选择要找哪一个")
        self.assertEqual([c["id"] for c in t["candidates"]], ["object-0001", "object-0002"])
        self.assertEqual(t["candidates"][0]["name"], "杯子")
        self.assertEqual(t["candidates"][1]["position"], [3.0, 4.0, 0.0])
        self.assertIsNone(t["candidates"][1]["state"])
        self.assertEqual(counts["candidate_dropped"], 1)
        t = gp.compose_task({"state": "navigating", "reason_task": "search", "candidates": [{"id": "x"}]}, "b:3")
        self.assertEqual((t["state"], t["message"], t["candidates"]), ("searching", "正在按搜索路线寻找", []))
        t = gp.compose_task({"state": "failed", "reason_task": "map_invalid: x"}, "b:3")
        self.assertEqual((t["state"], t["reasonCode"], t["message"]), ("failed", "map_invalid: x", "任务失败：map_invalid: x"))
        t = gp.compose_task({"state": "not_found", "reason_task": "search_exhausted", "target_id": "object-9"}, "b:3")
        self.assertEqual((t["message"], t["targetId"]), ("搜索路线走完仍未找到", "object-9"))
        t = gp.compose_task({"state": "warping", "reason_task": None}, "b:3", counts)
        self.assertEqual((t["state"], t["reasonCode"]), ("idle", "warping"))
        self.assertEqual(counts["task_state_unknown"], 1)

    def test_task_retry_states_and_attempts(self):
        """10-25 find-retry: the runner's recovering state shows as navigating; attempts -> attempts/positionsTried."""
        att = {"count": 3, "positions": 2, "max": 3, "items": []}
        t = gp.compose_task({"state": "recovering", "reason_task": "navigation_retry_pending", "attempts": att}, "b:4")
        self.assertEqual((t["state"], t["message"], t["attempts"], t["positionsTried"]),
                         ("navigating", "没能到达，已停车并清理地图，马上重试", 3, 2))
        for reason, zh in (("navigation_failed_after_retries", "导航重试几次仍没能到达"),
                           ("attempts_time_exhausted", "剩余时间不够再试一次，已停止")):
            t = gp.compose_task({"state": "failed", "reason_task": reason, "attempts": att}, "b:4")
            self.assertEqual((t["state"], t["message"], t["positionsTried"]), ("failed", zh, 2))
        t = gp.compose_task({"state": "navigating", "reason_task": "reobserve"}, "b:4")    # old rows: no keys
        self.assertNotIn("attempts", t)
        self.assertNotIn("positionsTried", t)
        t = gp.compose_task({"state": "navigating", "reason_task": "reobserve",
                             "attempts": {"count": True, "positions": "2"}}, "b:4")         # malformed: dropped
        self.assertNotIn("positionsTried", t)
        self.assertEqual(gp.reason_zh("map_switching"), "地图切换中，请稍候")

    def test_task_id_tracker(self):
        tr = gp.TaskIdTracker()
        self.assertIsNone(tr.update("B1", {"state": "idle", "query_seq": 0}))
        self.assertEqual(tr.update("B1", {"state": "navigating", "reason_task": "search", "query_seq": 1}), "B1:1")
        # a rejected query bumps query_seq but leaves the task row: id unchanged
        self.assertEqual(tr.update("B1", {"state": "navigating", "reason_task": "search", "query_seq": 2}), "B1:1")
        self.assertEqual(tr.update("B1", {"state": "found", "reason_task": "x", "query_seq": 2}), "B1:2")
        self.assertEqual(tr.update("B1", {"state": "found", "reason_task": "x", "query_seq": 2}), "B1:2")
        # hub restart: recomputed with the new boot even though the row is the same
        self.assertEqual(tr.update("B2", {"state": "found", "reason_task": "x", "query_seq": 0}), "B2:0")
        self.assertIsNone(tr.update(None, {"state": "found", "reason_task": "x", "query_seq": 0}))


def memory_body(tracks, **kw):
    m = {"runtime_boot_id": "rb1", "revision": 42, "stamp_ros_s": 1000.0, "world_frame": "map",
         "age_ms": 200, "source": "replay", "tracks": tracks}
    m.update(kw)
    return m


class SnapshotTests(unittest.TestCase):
    def test_objects_recording_shape_and_validation(self):
        tracks = [
            {"id": "t00_front", "label": "chair", "position_m": [1, 2, 0.4], "last_seen": 0.0},   # recording shape
            {"id": "object-0001", "label": "cup", "color": "blue", "position_m": [1, 2, 3],
             "last_seen": 999.5, "state": "historical_memory", "history_last": {"score": 0.81}},
            {"id": "object-0001", "label": "cup", "position_m": [9, 9, 9]},                  # duplicate id
            {"id": "object-0002", "label": "cup", "position_m": [float("nan"), 0, 0]},
            {"id": "object-0003", "label": "cup", "position_m": [0, 0, 0], "state": "bogus",
             "history_last": {"score": float("inf")}, "last_seen": float("nan")},
            {"label": "cup", "position_m": [0, 0, 0]},
        ]
        counts = {}
        objs = gp.compose_objects(memory_body(tracks), counts)
        self.assertEqual([o["id"] for o in objs], ["t00_front", "object-0001", "object-0003"])
        a, b, c = objs
        self.assertEqual((a["name"], a["className"], a["color"], a["aliases"], a["state"], a["confidence"]),
                         ("椅子", "chair", None, None, None, None))
        self.assertEqual((b["name"], b["aliases"], b["confidence"], b["lastSeen"], b["state"]),
                         ("杯子", ["杯"], 0.81, 999.5, "historical_memory"))
        self.assertEqual((c["state"], c["confidence"], c["lastSeen"]), (None, None, None))
        self.assertEqual(counts, {"object_duplicate_id": 1, "object_nonfinite_position": 1,
                                  "object_unknown_state": 1, "object_bad_id": 1})
        gp.dumps(objs)                                 # allow_nan=False must not raise

    def test_objects_cap_keeps_most_recent(self):
        tracks = [{"id": "o%d" % i, "label": "cup", "position_m": [0, 0, 0], "last_seen": float(i)}
                  for i in range(1100)]
        tracks.append({"id": "never", "label": "cup", "position_m": [0, 0, 0]})
        counts = {}
        objs = gp.compose_objects(memory_body(tracks), counts)
        self.assertEqual(len(objs), gp.MAX_OBJECTS)
        self.assertEqual(counts["object_truncated"], 101)
        ids = set(o["id"] for o in objs)
        self.assertIn("o1099", ids)
        self.assertNotIn("o99", ids)
        self.assertNotIn("never", ids)

    def test_server_time_and_path(self):
        self.assertAlmostEqual(gp.server_time(memory_body([]), 1.5), 1000.0 + 0.2 + 1.5)
        self.assertIsNone(gp.server_time(memory_body([], stamp_ros_s=None), 1.0))
        st = {"nav": {"path": {"frame_id": "map", "points": [[0, 0], [1, float("nan")], [2, 3]]}}}
        self.assertEqual(gp.compose_path(st, "map"), [[0.0, 0.0, 0.0], [2.0, 3.0, 0.0]])
        self.assertEqual(gp.compose_path(st, "odom"), [])
        self.assertEqual(gp.compose_path({"nav": {"path": None}}, "map"), [])

    def test_snapshot_unknown_tier_recording(self):
        st = {"source": "replay", "hub_boot_id": "B", "pose": {"x": None, "loc_state": None},
              "memory": {"age_ms": 180}, "task": {"state": None, "age_ms": 3}, "nav": {"path": None},
              "estop": {"latched": False}}
        mem = memory_body([{"id": "t1", "label": "chair", "position_m": [1, 1, 0]}], world_frame="map")
        snap = gp.compose_snapshot(st, None, mem, 0.5, "up", 40, gp.compose_task(st["task"], None),
                                   None, False, map_id="abc", map_name="实验室")
        self.assertEqual(snap["type"], "snapshot")
        self.assertNotIn("seq", snap)
        self.assertEqual(snap["source"], "replay")
        self.assertIsNone(snap["robot"])
        self.assertEqual(snap["map"], {"id": None, "name": None, "frame": "map", "units": "m",
                                       "cloudRev": None, "placeholder": False})
        self.assertEqual(snap["memory"], {"scope": "rb1", "revision": 42, "ageMs": 180.0})
        self.assertAlmostEqual(snap["serverTime"], 1000.7)
        self.assertEqual(snap["objects"][0]["name"], "椅子")
        self.assertEqual(snap["task"]["state"], "idle")
        self.assertEqual(snap["link"], {"state": "up", "hubAgeMs": 40.0})
        json.loads(gp.dumps(snap))

    def test_snapshot_loc_tier_and_no_memory(self):
        st = {"source": "live", "pose": {"x": 1, "y": 2, "yaw": 0, "loc_state": "valid", "age_ms": 10},
              "odom": {"v": 0.0}, "task": {"state": "idle"}, "estop": {"latched": True}}
        ri = {"pose_source": "estimated"}
        snap = gp.compose_snapshot(st, ri, None, None, "stale", None, gp.compose_task(st["task"], None),
                                   "global:7", False, map_id="abc", map_name="实验室")
        self.assertEqual((snap["map"]["id"], snap["map"]["name"], snap["map"]["cloudRev"]), ("abc", "实验室", "global:7"))
        self.assertEqual(snap["memory"], {"scope": None, "revision": None, "ageMs": None})
        self.assertEqual((snap["objects"], snap["serverTime"]), ([], None))
        self.assertEqual((snap["robot"]["pose"], snap["robot"]["status"]), ("map", "急停锁存"))
        st["pose"]["map_id"] = "from_state"
        snap = gp.compose_snapshot(st, ri, None, None, "up", 1, gp.idle_task(), None, False, map_id="abc")
        self.assertEqual(snap["map"]["id"], "from_state")
        self.assertEqual(gp.compose_snapshot(None, None, None, None, "bogus", None, gp.idle_task(), None,
                                             False)["link"]["state"], "down")

    def test_hello(self):
        h = gp.make_hello("bid", "B1", "up", "replay", "observer")
        self.assertEqual(h["capabilities"], {"findText": True, "findById": False, "select": True, "cancel": True,
                                             "pause": False, "resume": False, "estop": True, "control": "observer",
                                             "teleop": True, "mappingStop": True, "goto": True, "release": True,
                                             "mapReset": True})
        self.assertEqual(h["bridge"], {"id": "bid", "hubBootId": "B1", "link": "up"})
        self.assertEqual(gp.make_hello("b", None, "down", "weird", "none")["source"], "unknown")


class CloudTests(unittest.TestCase):
    def test_decode_grid_int8_and_errors(self):
        g = make_grid([-1, 0, 100, 50], 2, 2)
        self.assertEqual(list(g["cells"]), [-1, 0, 100, 50])
        with self.assertRaises(ValueError):
            gp.decode_grid(grid_meta(3, 2), grid_payload([0, 0, 0, 0]))
        with self.assertRaises(ValueError):
            gp.decode_grid(grid_meta(2, 2, res=0.0), grid_payload([0] * 4))
        with self.assertRaises(ValueError):
            gp.decode_grid(grid_meta(2, 2, origin=(float("nan"), 0, 0)), grid_payload([0] * 4))
        with self.assertRaises(ValueError):
            gp.decode_grid(dict(grid_meta(2, 2), width=True), grid_payload([0] * 4))

    def test_placeholder(self):
        self.assertTrue(gp.is_placeholder(make_grid([-1] * 16, 4, 4)["cells"]))
        self.assertTrue(gp.is_placeholder(make_grid([0, 64, -1, 25], 2, 2)["cells"]))
        self.assertFalse(gp.is_placeholder(make_grid([0, 65, -1, 25], 2, 2)["cells"]))

    def test_grid_to_points_known_answer(self):
        # 8x8, one occupied cell at (i=1, j=2), others free; origin (10, 20) yaw 0, res 0.5
        cells = [0] * 64
        cells[2 * 8 + 1] = 100
        cells[3 * 8 + 3] = 50                 # 26..64: drawn as nothing
        g = make_grid(cells, 8, 8, res=0.5, origin=(10.0, 20.0, 0.0))
        xyz, rgb, step = gp.grid_to_points(g)
        self.assertEqual(step, 1)
        self.assertEqual(len(xyz), 3 * (4 + 4))        # 4 occupied points + free at (0,0),(4,0),(0,4),(4,4)
        self.assertEqual(len(rgb), len(xyz))
        self.assertEqual(xyz[0:3], [10.75, 21.25, 0.05])
        self.assertEqual([xyz[3 * k + 2] for k in range(4)], [0.05, 0.15, 0.25, 0.35])
        floor = sorted((xyz[3 * k], xyz[3 * k + 1]) for k in range(4, 8))
        self.assertEqual(floor, [(10.25, 20.25), (10.25, 22.25), (12.25, 20.25), (12.25, 22.25)])
        self.assertEqual(set(xyz[3 * k + 2] for k in range(4, 8)), {0.0})

    def test_grid_rotation(self):
        cells = [100]
        g = make_grid(cells, 1, 1, res=1.0, origin=(1.0, 0.0, math.pi / 2))
        xyz, _, _ = gp.grid_to_points(g)
        # local centre (0.5, 0.5) rotated 90 deg -> (-0.5, 0.5), plus origin
        self.assertAlmostEqual(xyz[0], 0.5)
        self.assertAlmostEqual(xyz[1], 0.5)

    def test_unknown_cells_not_drawn(self):
        xyz, rgb, _ = gp.grid_to_points(make_grid([-1] * 16, 4, 4))
        self.assertEqual((xyz, len(rgb)), ([], 0))

    def test_thinning_doubles_step(self):
        g = make_grid([100] * 400, 20, 20)      # 400 occupied cells -> 1600 points
        xyz, _, step = gp.grid_to_points(g, limit=1600)
        self.assertEqual((step, len(xyz) // 3), (1, 1600))
        xyz, _, step = gp.grid_to_points(g, limit=1599)
        self.assertEqual((step, len(xyz) // 3), (2, 400))
        xyz, _, step = gp.grid_to_points(g, limit=399)
        self.assertEqual((step, len(xyz) // 3), (4, 100))

    def test_encode_cloud_layout(self):
        for n in (0, 1, 5):
            xyz = [float(k) for k in range(3 * n)]
            rgb = bytes(range(3 * n))
            for mid in (None, "a" * 64, "实验室"):
                buf = gp.encode_cloud(mid, "global:7", "map", xyz, rgb)
                (L,) = struct.unpack_from("<I", buf)
                pad = (-(4 + L)) % 4
                self.assertEqual(len(buf), 4 + L + pad + 15 * n)
                self.assertEqual((4 + L + pad) % 4, 0)
                self.assertEqual(buf[4 + L:4 + L + pad], b"\x00" * pad)
                head, xyz2, rgb2 = gp.decode_cloud(buf)
                self.assertEqual(head, {"type": "cloud", "version": 2, "mapId": mid, "rev": "global:7",
                                        "frame": "map", "count": n, "source": "grid", "encoding": "f32xyz+u8rgb"})
                self.assertEqual(list(xyz2), xyz)
                self.assertEqual(rgb2, rgb)
        empty = gp.encode_cloud(None, None, "map", [], b"")
        self.assertEqual(gp.decode_cloud(empty)[0]["count"], 0)
        with self.assertRaises(ValueError):
            gp.encode_cloud(None, None, "map", [0.0, float("nan"), 0.0], b"\x00\x00\x00")
        with self.assertRaises(ValueError):
            gp.encode_cloud(None, None, "map", [0.0, 0.0], b"\x00\x00")
        with self.assertRaises(ValueError):
            gp.encode_cloud(None, None, "map", [0.0] * 3 * (gp.MAX_CLOUD_POINTS + 1),
                            b"\x00" * 3 * (gp.MAX_CLOUD_POINTS + 1))

    def test_choose_cloud_grid_and_rev(self):
        real = make_grid([0, 100, -1, 0], 2, 2, rev=7, frame="map")
        fake = make_grid([-1] * 4, 2, 2, rev=1, frame="map")
        roll = make_grid([0, 100, -1, 0], 2, 2, which="rolling", rev=None, frame="odom")
        loc = {"tier": "loc", "frame": "map"}
        odom = {"tier": "odom", "frame": "odom"}
        unk = {"tier": "unknown", "frame": "map"}
        self.assertIs(gp.choose_cloud_grid(loc, real, roll), real)
        self.assertIsNone(gp.choose_cloud_grid(loc, fake, roll))              # placeholder global
        self.assertIsNone(gp.choose_cloud_grid(loc, make_grid([100], 1, 1, frame="odom"), None))
        self.assertIs(gp.choose_cloud_grid(odom, fake, roll), roll)
        self.assertIsNone(gp.choose_cloud_grid(odom, real, None))
        self.assertIsNone(gp.choose_cloud_grid(unk, real, roll))
        self.assertIs(gp.choose_cloud_grid({"tier": "sim", "frame": "map"}, real, roll), real)
        self.assertEqual(gp.cloud_rev_for(real), "global:7")
        r1 = gp.cloud_rev_for(roll)
        self.assertRegex(r1, r"^rolling:[0-9a-f]{8}$")
        moved = make_grid([0, 100, -1, 0], 2, 2, which="rolling", rev=None, frame="odom", origin=(1, 0, 0))
        self.assertNotEqual(gp.cloud_rev_for(moved), r1)
        self.assertEqual(gp.cloud_rev_for(make_grid([0, 100, -1, 0], 2, 2, which="rolling", rev=None,
                                                    frame="odom")), r1)

    def test_ply(self):
        b = gp.ply_bytes([1.0, 2.0, 3.0], b"\x01\x02\x03")
        self.assertTrue(b.startswith(b"ply\n"))
        self.assertEqual(len(b.split(b"end_header\n", 1)[1]), 15)


class CommandTests(unittest.TestCase):
    TASK = {"id": "B:3", "state": "ambiguous", "candidates": [{"id": "object-0001"}, {"id": "object-0002"}]}

    def ctx(self, **kw):
        c = {"link": "up", "source": "live", "scope": "rb1", "task": self.TASK}
        c.update(kw)
        return c

    def cmd(self, action, payload=None, **kw):
        m = {"type": "command", "version": 2, "id": "u-1", "action": action, "payload": payload or {}}
        m.update(kw)
        return m

    def code(self, msg, **ctx):
        d = gp.classify_command(msg, self.ctx(**ctx))
        return d["code"] if d["kind"] == "reject" else ("forward", d["op"], d["args"], d["deadline_ms"])

    def test_find(self):
        self.assertEqual(self.code(self.cmd("find", {"text": "杯子"})), ("forward", "find", {"text": "杯子"}, 3000))
        d = gp.classify_command(self.cmd("find", {"text": "椅子"}), self.ctx())
        self.assertEqual((d["args"], bool(d["note"])), ({"text": "chair"}, True))
        self.assertEqual(self.code(self.cmd("find", {"text": "  "})), "bad_request")
        self.assertEqual(self.code(self.cmd("find", {"text": "杯" * 65})), "bad_request")
        self.assertEqual(self.code(self.cmd("find", {"text": "杯" * 64}))[0], "forward")
        self.assertEqual(self.code(self.cmd("find", {"text": 3})), "bad_request")
        self.assertEqual(self.code(self.cmd("find", {"targetId": "object-0001"})), "not_supported")
        self.assertEqual(self.code(self.cmd("find", {"targetId": "object-0001", "scope": "rb1"})), "not_supported")
        self.assertEqual(self.code(self.cmd("find", {"targetId": "object-0001", "scope": "old"})), "scope_changed")
        self.assertEqual(self.code(self.cmd("find", {"text": "杯子"}), link="stale"), "link_down")
        self.assertEqual(self.code(self.cmd("find", {"text": "杯子"}), link="down"), "link_down")
        self.assertEqual(self.code(self.cmd("find", {"text": "杯子"}), source="inconsistent"), "source_inconsistent")

    def test_select(self):
        ok = self.cmd("select", {"taskId": "B:3", "candidateId": "object-0002"})
        self.assertEqual(self.code(ok), ("forward", "select", {"candidate_id": "object-0002"}, 3000))
        self.assertEqual(self.code(self.cmd("select", {"taskId": "B:3", "candidateId": "object-0002",
                                                       "scope": "rb1"}))[0], "forward")
        self.assertEqual(self.code(self.cmd("select", {"taskId": "B:3", "candidateId": "object-0002",
                                                       "scope": "rb0"})), "scope_changed")
        self.assertEqual(self.code(self.cmd("select", {"taskId": "B:3", "candidateId": "object-0002",
                                                       "scope": "rb1"}), scope=None), "scope_changed")
        self.assertEqual(self.code(self.cmd("select", {"taskId": "B:2", "candidateId": "object-0002"})), "stale_task")
        self.assertEqual(self.code(ok, task=dict(self.TASK, state="navigating")), "not_ambiguous")
        self.assertEqual(self.code(self.cmd("select", {"taskId": "B:3", "candidateId": "object-0009"})),
                         "no_such_candidate")
        self.assertEqual(self.code(self.cmd("select", {"taskId": "B:3"})), "bad_request")
        self.assertEqual(self.code(ok, link="down"), "link_down")
        self.assertEqual(self.code(ok, source="inconsistent"), "source_inconsistent")

    def test_cancel_always_forwarded(self):
        for ctx in (dict(link="down"), dict(link="stale"), dict(source="inconsistent"), dict(task=gp.idle_task())):
            self.assertEqual(self.code(self.cmd("cancel", {"taskId": "B:3"}), **ctx), ("forward", "cancel", {}, 5000))
        d = gp.classify_command(self.cmd("cancel", {"taskId": "B:1"}), self.ctx())
        self.assertIn("taskId", d["note"])
        self.assertIsNone(gp.classify_command(self.cmd("cancel", {"taskId": None}), self.ctx())["note"])

    def test_motion_ops_ruling4(self):
        """10-18 controller ruling 4: teleop / goto / mapping_stop / estop / release forwarded, shapes only."""
        # estop: always, link stale/down and inconsistent source included; fpgac's own e-stop deadline (None)
        for ctx in (dict(), dict(link="down"), dict(link="stale"), dict(source="inconsistent")):
            self.assertEqual(self.code(self.cmd("estop"), **ctx), ("forward", "estop", {}, None))
        self.assertEqual(self.code(self.cmd("estop", {"x": 1}))[:3], ("forward", "estop", {}))
        # teleop: values passed through unclamped (0.9 > the hub's 0.30 cap: the hub rejects bad_args, not the bridge)
        self.assertEqual(self.code(self.cmd("teleop", {"v": 0.1, "w": -0.2})),
                         ("forward", "teleop", {"v": 0.1, "w": -0.2}, 1000))
        self.assertEqual(self.code(self.cmd("teleop", {"v": 0.9, "w": 0})), ("forward", "teleop", {"v": 0.9, "w": 0}, 1000))
        for bad in ({"v": 0.1}, {"v": True, "w": 0}, {"v": float("nan"), "w": 0}, {"v": "0.1", "w": 0},
                    {"stop": 1}, {"stop": True, "v": 0}):
            self.assertEqual(self.code(self.cmd("teleop", bad)), "bad_request", bad)
        self.assertEqual(self.code(self.cmd("teleop", {"v": 0.1, "w": 0}), link="stale"), "link_down")
        self.assertEqual(self.code(self.cmd("teleop", {"stop": True}), link="down"),
                         ("forward", "teleop", {"stop": True}, 1000))
        # goto
        self.assertEqual(self.code(self.cmd("goto", {"forwardM": 0.5})), ("forward", "goto", {"forward_m": 0.5}, 3000))
        self.assertEqual(self.code(self.cmd("goto", {"forwardM": 0.5, "turnRad": 0.3})),
                         ("forward", "goto", {"forward_m": 0.5, "turn_rad": 0.3}, 3000))
        for bad in ({}, {"forward_m": 0.5}, {"forwardM": None}, {"forwardM": float("inf")}):
            self.assertEqual(self.code(self.cmd("goto", bad)), "bad_request", bad)
        self.assertEqual(self.code(self.cmd("goto", {"forwardM": 0.5}), link="stale"), "link_down")
        self.assertEqual(self.code(self.cmd("goto", {"forwardM": 0.5}), source="inconsistent"), "source_inconsistent")
        # mapping_stop
        self.assertEqual(self.code(self.cmd("mapping_stop")), ("forward", "mapping_stop", {}, 3000))
        self.assertEqual(self.code(self.cmd("mapping_stop", {"now": True})), "bad_request")
        self.assertEqual(self.code(self.cmd("mapping_stop"), link="down"), "link_down")
        # release: explicit confirm, epoch from the request or the hub STATE
        self.assertEqual(self.code(self.cmd("release", {"epoch": 3})), "confirm_required")
        self.assertEqual(self.code(self.cmd("release", {"confirm": "yes", "epoch": 3})), "confirm_required")
        self.assertEqual(self.code(self.cmd("release", {"confirm": True, "epoch": 3})),
                         ("forward", "release", {"epoch": 3}, 5000))
        self.assertEqual(self.code(self.cmd("release", {"confirm": True}), estop_epoch=7),
                         ("forward", "release", {"epoch": 7}, 5000))
        self.assertEqual(self.code(self.cmd("release", {"confirm": True})), "bad_request")       # no epoch anywhere
        self.assertEqual(self.code(self.cmd("release", {"confirm": True, "epoch": True})), "bad_request")
        self.assertEqual(self.code(self.cmd("release", {"confirm": True, "epoch": 3, "x": 1})), "bad_request")
        self.assertEqual(self.code(self.cmd("release", {"confirm": True, "epoch": 3}), link="stale"), "link_down")

    def test_bad_requests_and_unsupported(self):
        self.assertEqual(self.code(self.cmd("pause")), "not_supported")
        self.assertEqual(self.code(self.cmd("resume")), "not_supported")
        self.assertEqual(self.code(self.cmd("estop")), ("forward", "estop", {}, None))   # 10-18 ruling 4
        self.assertEqual(self.code(self.cmd("find", {"text": "杯子"}, version=1)), "bad_request")
        self.assertEqual(self.code(self.cmd("find", {"text": "杯子"}, type="hello")), "bad_request")
        self.assertEqual(self.code(self.cmd("find", {"text": "杯子"}, id="")), "bad_request")
        self.assertEqual(self.code(self.cmd("find", {"text": "杯子"}, id="x" * 129)), "bad_request")
        m = self.cmd("find")
        m["payload"] = [1]
        self.assertEqual(self.code(m), "bad_request")
        self.assertEqual(gp.parse_command("[1]"), (None, None))
        self.assertEqual(gp.parse_command("{"), (None, None))
        self.assertEqual(gp.parse_command('{"id": 5}')[0], None)
        self.assertEqual(gp.parse_command('{"id": "a"}')[0], "a")

    def test_ack_and_result_shapes(self):
        self.assertEqual(gp.make_ack("u", True, "forwarded"),
                         {"type": "ack", "version": 2, "id": "u", "accepted": True, "status": "forwarded"})
        a = gp.make_ack("u", False, "rejected", "stale_task")
        self.assertEqual((a["reasonCode"], a["reason"]), ("stale_task", "任务已经变了，请按最新任务重新操作"))
        r = gp.make_result("u", "rejected", "cancel current operation and wait for navigation to stop")
        self.assertEqual(r["reason"], "还有任务或导航在进行，请先停止")
        self.assertEqual(gp.make_result("u", "unknown"), {"type": "result", "version": 2, "id": "u",
                                                         "status": "unknown", "reason": None, "reasonCode": None})


# ---------------------------------------------------------------------------- bridge core and HTTP/WS shell

import geomem_bridge as gb  # noqa: E402


class FakeFrame(object):
    def __init__(self, name):
        self.name = name


class FakeRuntimeInfo(object):
    value = None


class FakeClient(object):
    def __init__(self):
        self.client_id = "me"
        self.link_state = "up"
        self.hub_boot_id = "B1"
        self.hello_ack = {"ok": True, "controller": True}
        self.runtime_info = FakeRuntimeInfo()
        self.commands = []
        self.on_command = None

    def command(self, op, args=None, deadline_ms=None):
        cid = "me:%d" % (len(self.commands) + 1)
        self.commands.append((cid, op, dict(args or {}), deadline_ms))
        if self.on_command:
            self.on_command(cid)
        return cid


class FakeSession(object):
    def __init__(self):
        self.events = []

    def put_json(self, obj):
        self.events.append(("json", obj))

    def put_hello(self, obj):
        self.events.append(("hello", obj))

    def put_cloud(self, data):
        self.events.append(("cloud", data))

    def put_snapshot(self, obj):
        self.events.append(("snapshot", obj))

    def kinds(self):
        return [k for k, _ in self.events]

    def jsons(self):
        return [v for k, v in self.events if k == "json"]

    def take(self):
        out, self.events = self.events, []
        return out


class Clock(object):
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def loc_state(**kw):
    s = {"source": "live", "hub_boot_id": "B1",
         "pose": {"x": 1.0, "y": 2.0, "yaw": 0.0, "loc_state": "valid", "age_ms": 50},
         "odom": {"v": 0.0, "age_ms": 20}, "nav": {"motion_permit": False, "cmd_v": 0.0, "path": None},
         "memory": {"age_ms": 100}, "task": {"state": "idle", "reason_task": None, "query_seq": 0, "age_ms": 5},
         "estop": {"latched": False}}
    s.update(kw)
    return s


class BridgeCoreTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.client = FakeClient()
        self.client.runtime_info.value = {"pose_source": "estimated", "sensor_source": "hardware"}
        self.b = gb.Bridge(map_id="m" * 64, map_name="实验室", clock=self.clock, bridge_id="bid").attach(self.client)
        self.b.on_link("up", {})
        self.s = FakeSession()

    def feed(self, name, obj):
        self.b.on_frame(FakeFrame(name), obj)

    def cmd(self, action, payload, cid="u1"):
        self.b.handle_command(self.s, json.dumps({"type": "command", "version": 2, "id": cid,
                                                  "action": action, "payload": payload}))

    def test_register_sends_hello_and_empty_cloud(self):
        self.assertTrue(self.b.register(self.s))
        self.assertEqual(self.s.kinds(), ["hello", "cloud"])
        hello = self.s.events[0][1]
        self.assertEqual(hello["capabilities"]["control"], "controller")
        head = gp.decode_cloud(self.s.events[1][1])[0]
        self.assertEqual((head["count"], head["rev"]), (0, None))

    def test_cloud_before_snapshot_and_placeholder(self):
        self.b.register(self.s)
        self.s.take()
        self.feed("STATE", loc_state())
        cells = [0] * 16
        cells[5] = 100
        self.feed("MAP", (grid_meta(4, 4, rev=7), grid_payload(cells)))
        snap = self.b.tick()
        kinds = self.s.kinds()
        self.assertEqual(kinds, ["cloud", "hello", "snapshot"])   # hello moved: hubBootId/source known now
        head, xyz, _ = gp.decode_cloud(self.s.events[0][1])
        self.assertEqual((head["rev"], head["mapId"], head["frame"], head["count"]), ("global:7", "m" * 64, "map", 4 + 1))
        self.assertEqual(snap["map"]["cloudRev"], "global:7")
        self.assertEqual(snap["map"]["id"], "m" * 64)
        self.assertEqual(snap["robot"]["pose"], "map")
        self.s.take()
        self.b.tick()
        self.assertEqual(self.s.kinds(), ["snapshot"])             # same rev: no cloud, no hello
        # placeholder global (all unknown) -> cloud cleared (rev null) and placeholder flag
        self.feed("MAP", (grid_meta(4, 4, rev=8), grid_payload([-1] * 16)))
        self.s.take()
        snap = self.b.tick()
        self.assertEqual(self.s.kinds(), ["cloud", "snapshot"])
        self.assertEqual(gp.decode_cloud(self.s.events[0][1])[0]["count"], 0)
        self.assertEqual((snap["map"]["cloudRev"], snap["map"]["placeholder"]), (None, True))

    def test_odom_rolling_cloud_rate_limited(self):
        self.client.runtime_info.value = {"pose_source": "reference", "sensor_source": "replay"}
        self.b.register(self.s)
        self.feed("STATE", loc_state(pose={"x": None, "loc_state": None}))
        self.feed("MAP", (grid_meta(4, 4, which="rolling", rev=None, frame="odom"), grid_payload([100] + [0] * 15)))
        self.s.take()
        snap = self.b.tick()
        self.assertTrue(snap["map"]["cloudRev"].startswith("rolling:"))
        self.assertEqual((snap["map"]["frame"], snap["map"]["id"], snap["robot"]), ("odom", None, None))
        first = snap["map"]["cloudRev"]
        self.feed("MAP", (grid_meta(4, 4, which="rolling", rev=None, frame="odom"), grid_payload([0, 100] + [0] * 14)))
        self.clock.t += 0.3
        self.s.take()
        self.assertEqual(self.b.tick()["map"]["cloudRev"], first)      # < 0.5 s: held
        self.assertNotIn("cloud", self.s.kinds())
        self.clock.t += 0.3
        self.s.take()
        self.assertNotEqual(self.b.tick()["map"]["cloudRev"], first)
        self.assertEqual(self.s.kinds()[0], "cloud")

    def test_unknown_tier_no_cloud_even_with_maps(self):
        self.client.runtime_info.value = None
        self.feed("STATE", loc_state(source="replay", pose={"x": None, "loc_state": None}))
        self.feed("MAP", (grid_meta(4, 4, rev=7), grid_payload([100] * 16)))
        self.feed("MEMORY", memory_body([{"id": "t1", "label": "chair", "position_m": [0, 0, 0]}]))
        snap = self.b.tick()
        self.assertEqual((snap["map"]["cloudRev"], snap["robot"], snap["source"]), (None, None, "replay"))
        self.assertEqual(snap["objects"][0]["name"], "椅子")

    def test_hello_control_and_link(self):
        self.b.register(self.s)
        self.b.tick()
        self.s.take()
        self.feed("EVENT", {"v": 1, "event": {"event": "controller_changed", "controller_client_id": "other"}})
        self.b.tick()
        hellos = [v for k, v in self.s.events if k == "hello"]
        self.assertEqual(hellos[-1]["capabilities"]["control"], "observer")
        self.s.take()
        self.client.link_state = "down"
        self.b.on_link("down", {"why": "dead"})
        self.b.tick()
        hellos = [v for k, v in self.s.events if k == "hello"]
        self.assertEqual((hellos[-1]["capabilities"]["control"], hellos[-1]["bridge"]["link"]), ("none", "down"))
        # reconnect to a restarted hub that hands control to us again
        self.s.take()
        self.client.link_state, self.client.hub_boot_id = "up", "B2"
        self.b.on_link("hub_restarted", {"old": "B1", "new": "B2"})
        self.b.on_link("up", {"host": "h"})
        self.b.tick()
        hellos = [v for k, v in self.s.events if k == "hello"]
        self.assertEqual((hellos[-1]["capabilities"]["control"], hellos[-1]["bridge"]["hubBootId"]), ("controller", "B2"))
        # stale -> up recovery does not re-read hello_ack (an observer stays observer)
        self.feed("EVENT", {"v": 1, "event": {"event": "controller_changed", "controller_client_id": "other"}})
        self.client.hello_ack = {"controller": True}
        self.b.on_link("up", {"recovered": True})
        self.assertFalse(self.b.controller)

    def test_replay_server_observer(self):
        self.client.hello_ack = {"ok": True, "controller": False}
        self.b.on_link("up", {})
        self.b.register(self.s)
        self.assertEqual(self.s.events[0][1]["capabilities"]["control"], "observer")

    def test_find_forward_ack_result(self):
        self.b.tick()
        self.cmd("find", {"text": "杯子"})
        self.assertEqual(self.client.commands, [("me:1", "find", {"text": "杯子"}, 3000)])
        self.assertEqual(self.s.events, [])
        self.feed("CMD_ACK", {"cmd_id": "me:1", "status": "forwarded", "query_seq": 1})
        self.feed("CMD_RESULT", {"cmd_id": "me:1", "status": "accepted", "task_state": "navigating", "final": True})
        ack, res = self.s.jsons()
        self.assertEqual(ack, {"type": "ack", "version": 2, "id": "u1", "accepted": True, "status": "forwarded"})
        self.assertEqual((res["type"], res["id"], res["status"]), ("result", "u1", "accepted"))
        self.assertEqual(self.b.pending, {})

    def test_find_rejected_by_runner_result(self):
        self.cmd("find", {"text": "杯子"})
        self.feed("CMD_ACK", {"cmd_id": "me:1", "status": "forwarded"})
        self.feed("CMD_RESULT", {"cmd_id": "me:1", "status": "rejected", "final": True,
                                 "reason": "recent processed input, TF and map required"})
        res = self.s.jsons()[-1]
        self.assertEqual((res["status"], res["reason"]), ("rejected", "小车还没准备好（画面、位置或地图缺失）"))

    def test_hub_rejects_replay_and_not_controller(self):
        for code, zh in (("replay", "回放模式下不能操作小车"), ("not_controller", "另一台电脑持有控制权")):
            self.s.take()
            self.cmd("cancel", {"taskId": None}, cid="c-" + code)
            hub_id = self.client.commands[-1][0]
            self.feed("CMD_ACK", {"cmd_id": hub_id, "status": "rejected", "reason": code})
            ack = self.s.jsons()[-1]
            self.assertEqual((ack["accepted"], ack["status"], ack["reasonCode"], ack["reason"]), (False, "rejected", code, zh))
        self.assertEqual(self.b.pending, {})
        self.assertEqual(self.b.stats()["rejects"], {"replay": 1, "not_controller": 1})

    def test_duplicate_ack_note(self):
        self.cmd("find", {"text": "杯子"})
        self.feed("CMD_ACK", {"cmd_id": "me:1", "status": "forwarded", "duplicate": True})
        ack = self.s.jsons()[-1]
        self.assertTrue(ack["accepted"])
        self.assertIn("重复", ack["note"])

    def test_no_ack_expires(self):
        self.cmd("find", {"text": "杯子"})
        self.clock.t += 3.4
        self.b.check_timeouts(self.clock())
        self.assertEqual(self.s.jsons(), [])
        self.clock.t += 0.2
        self.b.check_timeouts(self.clock())
        ack, res = self.s.jsons()
        self.assertEqual((ack["accepted"], ack["status"], ack["reasonCode"]), (False, "expired", "no_ack"))
        self.assertEqual(res["status"], "unknown")
        self.feed("CMD_ACK", {"cmd_id": "me:1", "status": "forwarded"})       # late ack: dropped
        self.assertEqual(len(self.s.jsons()), 2)

    def test_ack_without_result_gives_unknown(self):
        self.cmd("cancel", {})
        self.feed("CMD_ACK", {"cmd_id": "me:1", "status": "forwarded"})
        self.clock.t += 2.9
        self.b.check_timeouts(self.clock())
        self.assertEqual(len(self.s.jsons()), 1)
        self.clock.t += 0.2
        self.b.check_timeouts(self.clock())
        self.assertEqual(self.s.jsons()[-1]["status"], "unknown")
        self.assertEqual(self.b.pending, {})

    def test_estop_ack_latched_and_result(self):
        self.cmd("estop", {})
        self.assertEqual(self.client.commands[-1], ("me:1", "estop", {}, None))
        self.clock.t += 5.0                                                     # no 3.5 s no_ack for estop
        self.b.check_timeouts(self.clock())
        self.assertEqual(self.s.jsons(), [])
        self.feed("CMD_ACK", {"cmd_id": "me:1", "status": "latched_on_nano", "epoch": None})
        self.feed("CMD_RESULT", {"cmd_id": "me:1", "status": "kv_unconfirmed", "final": False})
        self.feed("CMD_RESULT", {"cmd_id": "me:1", "status": "latched", "epoch": 4, "final": True})
        ack, res = self.s.jsons()
        self.assertEqual((ack["accepted"], ack["status"]), (True, "forwarded"))
        self.assertEqual((res["status"], res["reasonCode"]), ("done", None))
        self.assertEqual(self.b.pending, {})

    def test_estop_without_ack_expires_after_30s(self):
        self.cmd("estop", {})
        self.clock.t += 29.0
        self.b.check_timeouts(self.clock())
        self.assertEqual(self.s.jsons(), [])
        self.clock.t += 1.5
        self.b.check_timeouts(self.clock())
        self.assertEqual(self.s.jsons()[0]["reasonCode"], "no_ack")

    def test_release_uses_state_epoch_and_hub_rejection(self):
        self.feed("STATE", loc_state(estop={"latched": True, "epoch": 5}))
        self.cmd("release", {"epoch": 5})
        self.assertEqual(self.s.jsons()[-1]["reasonCode"], "confirm_required")
        self.assertEqual(self.client.commands, [])
        self.cmd("release", {"confirm": True}, cid="r2")
        self.assertEqual(self.client.commands[-1][1:3], ("release", {"epoch": 5}))
        self.feed("CMD_ACK", {"cmd_id": "me:1", "status": "rejected", "reason": "epoch_mismatch", "epoch": 6})
        ack = self.s.jsons()[-1]
        self.assertEqual((ack["accepted"], ack["reasonCode"], ack["reason"]),
                         (False, "epoch_mismatch", "急停已更新，请按最新急停再解除"))

    def test_teleop_ack_ends_without_result(self):
        for i in range(5):
            self.cmd("teleop", {"v": 0.1, "w": 0.0}, cid="t%d" % i)
            self.feed("CMD_ACK", {"cmd_id": "me:%d" % (i + 1), "status": "forwarded"})
        self.assertEqual(self.b.pending, {})
        self.clock.t += 10.0
        self.b.check_timeouts(self.clock())
        msgs = self.s.jsons()
        self.assertEqual([m["type"] for m in msgs], ["ack"] * 5)
        self.cmd("teleop", {"v": 0.1, "w": 0.0}, cid="t9")
        self.feed("CMD_ACK", {"cmd_id": "me:6", "status": "rejected", "reason": "estop_latched"})
        self.assertEqual(self.s.jsons()[-1]["reasonCode"], "estop_latched")

    def test_mapping_stop_waits_for_the_export(self):
        self.cmd("mapping_stop", {})
        self.feed("CMD_ACK", {"cmd_id": "me:1", "status": "forwarded"})
        self.clock.t += 120.0                                                  # saving + export take minutes
        self.b.check_timeouts(self.clock())
        self.assertEqual(len(self.s.jsons()), 1)
        self.feed("CMD_RESULT", {"cmd_id": "me:1", "status": "done", "mapping_state": "exported",
                                 "map_loaded": True, "final": True})
        self.assertEqual(self.s.jsons()[-1]["status"], "done")
        self.cmd("mapping_stop", {}, cid="m2")
        self.feed("CMD_ACK", {"cmd_id": "me:2", "status": "forwarded"})
        self.clock.t += 186.0
        self.b.check_timeouts(self.clock())
        self.assertEqual(self.s.jsons()[-1]["status"], "unknown")

    def test_ruling4_ops_through_the_real_hub_core(self):
        """Bridge -> real host.hub.hub_core.HubCore.command (in process, no sockets): the hub's own ACK shapes."""
        sys.path.insert(0, str(ROOT))
        from host.hub.hub_core import HubCore
        from host.hub.hub_server import RosActionsLog
        core = HubCore("replay", actions=RosActionsLog())

        acks = {}
        for cid, action, payload in (("t-ok", "teleop", {"v": 0.1, "w": 0.0}),
                                     ("t-fast", "teleop", {"v": 0.9, "w": 0.0}),     # beyond the hub's 0.30 cap
                                     ("g", "goto", {"forwardM": 0.5}),
                                     ("m", "mapping_stop", {}),
                                     ("e", "estop", {}),
                                     ("t-est", "teleop", {"v": 0.1, "w": 0.0}),
                                     ("t-stop", "teleop", {"stop": True}),
                                     ("r-noconf", "release", {"epoch": 0}),
                                     ("r", "release", {"confirm": True, "epoch": 0})):
            n = len(self.client.commands)
            self.cmd(action, payload, cid=cid)
            if len(self.client.commands) > n:      # forwarded: the hub answers (after command() returned)
                hub_id, op, args, _ = self.client.commands[-1]
                self.feed("CMD_ACK", core.command("me", True, {"cmd_id": hub_id, "op": op, "args": args}, 1000))
            acks[cid] = [m for m in self.s.jsons() if m.get("id") == cid and m["type"] == "ack"][-1]
        code = lambda c: (acks[c]["accepted"], acks[c].get("reasonCode"))   # noqa: E731
        self.assertEqual(code("t-ok"), (True, None))
        self.assertEqual(code("t-fast"), (False, "bad_args"))        # the car's limit, not widened by the bridge
        self.assertEqual(code("g"), (True, None))
        self.assertEqual(code("m"), (False, "no_mapping_status"))
        self.assertEqual(code("e"), (True, None))                    # hub latched_on_nano
        self.assertEqual(code("t-est"), (False, "estop_latched"))
        self.assertEqual(code("t-stop"), (True, None))
        self.assertEqual(code("r-noconf"), (False, "confirm_required"))
        self.assertEqual(code("r"), (False, "epoch_mismatch"))       # not KV-confirmed in this fixture
        topics = [t for t, _ in core.actions.published]
        self.assertEqual(topics.count("/fpga/teleop/input"), 2)
        self.assertIn("/fpga/goto", topics)
        self.assertNotIn("r-noconf", [c[0] for c in self.client.commands])
        self.assertEqual([p["op"] for p in self.b.pending.values()], ["goto", "estop"])   # awaiting results

    def test_link_down_rejects_find_select_but_forwards_cancel(self):
        self.client.link_state = "down"
        self.cmd("find", {"text": "杯子"}, cid="f")
        self.cmd("select", {"taskId": None, "candidateId": "object-0001"}, cid="s")
        self.cmd("cancel", {"taskId": None}, cid="c")
        acks = self.s.jsons()
        self.assertEqual([(a["id"], a["reasonCode"]) for a in acks], [("f", "link_down"), ("s", "link_down")])
        self.assertEqual([c[1:] for c in self.client.commands], [("cancel", {}, 5000)])

    def test_select_flow_uses_composed_task_id(self):
        st = loc_state(task={"state": "ambiguous", "reason_task": "candidate_selection_required",
                             "target_id": None, "query_seq": 4, "age_ms": 1,
                             "candidates": [{"id": "object-0001", "label": "cup", "color": "blue",
                                             "position_m": [1, 1, 0], "state": "current_confirmed"},
                                            {"id": "object-0002", "label": "cup", "color": "red",
                                             "position_m": [2, 2, 0], "state": "current_confirmed"}]})
        self.feed("STATE", st)
        self.feed("MEMORY", memory_body([]))
        snap = self.b.tick()
        self.assertEqual(snap["task"]["id"], "B1:4")
        self.cmd("select", {"taskId": "B1:3", "candidateId": "object-0002"}, cid="old")
        self.cmd("select", {"taskId": "B1:4", "candidateId": "object-0002", "scope": "rb0"}, cid="scope")
        self.cmd("select", {"taskId": "B1:4", "candidateId": "object-0002", "scope": "rb1"}, cid="ok")
        self.assertEqual([(a["id"], a.get("reasonCode")) for a in self.s.jsons()],
                         [("old", "stale_task"), ("scope", "scope_changed")])
        self.assertEqual(self.client.commands[-1][1:], ("select", {"candidate_id": "object-0002"}, 3000))

    def test_bad_json_and_unsupported(self):
        self.b.handle_command(self.s, "not json")
        self.b.handle_command(self.s, json.dumps({"type": "command", "version": 2, "id": "p", "action": "pause",
                                                  "payload": {"taskId": None}}))
        a, b = self.s.jsons()
        self.assertEqual((a["id"], a["reasonCode"]), (None, "bad_request"))
        self.assertEqual((b["id"], b["reasonCode"]), ("p", "not_supported"))
        self.assertEqual(self.client.commands, [])

    def test_ack_arriving_inside_command_call_is_matched(self):
        # the FPGAC thread delivers CMD_ACK while command() is still running in the worker thread
        threads = []

        def deliver(cid):
            t = threading.Thread(target=self.feed, args=("CMD_ACK", {"cmd_id": cid, "status": "forwarded"}))
            t.start()
            threads.append(t)
            time.sleep(0.05)

        self.client.on_command = deliver
        self.cmd("find", {"text": "杯子"})
        for t in threads:
            t.join(2)
        self.assertEqual(self.s.jsons()[-1]["status"], "forwarded")
        self.assertEqual(self.b.counts["ack_unmatched"], 0)

    def test_bad_frames_never_raise(self):
        self.feed("MAP", (grid_meta(4, 4, rev=1), b"not zlib"))
        self.feed("MAP", "garbage")
        self.feed("EVENT", None)
        self.feed("CMD_ACK", None)
        self.feed("CMD_RESULT", {"cmd_id": "zz", "status": "done"})
        self.feed("STATE", loc_state(pose={"x": float("nan"), "y": 0, "loc_state": "valid"},
                                     odom={"v": float("inf")}))
        snap = self.b.tick()
        self.assertIsNone(snap["robot"])
        self.assertEqual(self.b.counts["map_decode_error:global"], 1)
        self.assertEqual(self.b.counts["on_frame_error"], 0)
        json.loads(gp.dumps(snap))


class _StubServer(object):
    def __init__(self, bridge, app_root, port=8788):
        self.bridge, self.app_root, self.server_address = bridge, Path(app_root), ("127.0.0.1", port)


class HttpShellTests(unittest.TestCase):
    """The real Handler over socket.socketpair(): no listening socket anywhere."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="gmb_http_")
        os.makedirs(os.path.join(self.tmp, "src"))
        with open(os.path.join(self.tmp, "index.html"), "wb") as f:
            f.write(b"<!doctype html>hi")
        with open(os.path.join(self.tmp, "src", "app.js"), "wb") as f:
            f.write(b"export const x = 1;")
        self.client = FakeClient()
        self.bridge = gb.Bridge(bridge_id="bid").attach(self.client)
        self.bridge.start()
        self.socks = []

    def tearDown(self):
        self.bridge.stop()
        for s in self.socks:
            s.close()
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def connect(self):
        a, b = socket.socketpair()
        self.socks += [a, b]
        stub = _StubServer(self.bridge, self.tmp)
        def serve():                      # what socketserver does around a handler
            try:
                gb.Handler(b, ("127.0.0.1", 0), stub)
            finally:
                try:
                    b.shutdown(socket.SHUT_WR)
                except OSError:
                    pass
                b.close()

        t = threading.Thread(target=serve, daemon=True)
        t.start()
        a.settimeout(5)
        return a, t

    def request(self, raw):
        a, t = self.connect()
        a.sendall(raw)
        a.shutdown(socket.SHUT_WR)
        data = b""
        while True:
            chunk = a.recv(65536)
            if not chunk:
                break
            data += chunk
        t.join(5)
        head, _, body = data.partition(b"\r\n\r\n")
        lines = head.decode("latin-1").split("\r\n")
        headers = dict((k.lower(), v.strip()) for k, _, v in (ln.partition(":") for ln in lines[1:]))
        return lines[0], headers, body

    def get(self, path, host="127.0.0.1:8788", extra=""):
        return self.request(("GET %s HTTP/1.1\r\nHost: %s\r\n%sConnection: close\r\n\r\n" % (path, host, extra))
                            .encode("latin-1"))

    def test_static_ok_and_headers(self):
        status, h, body = self.get("/")
        self.assertEqual(status, "HTTP/1.1 200 OK")
        self.assertEqual(body, b"<!doctype html>hi")
        self.assertEqual(h["content-length"], str(len(body)))
        self.assertEqual((h["x-content-type-options"], h["cache-control"]), ("nosniff", "no-cache"))
        self.assertTrue(h["content-type"].startswith("text/html"))
        status, h, body = self.get("/src/app.js?v=2", host="localhost:8788")
        self.assertEqual((status, h["content-type"]), ("HTTP/1.1 200 OK", "text/javascript; charset=utf-8"))

    def test_static_refusals(self):
        status, h, body = self.get("/", host="evil.example:8788")
        self.assertEqual(status.split()[1], "403")
        self.assertEqual(h["content-length"], str(len(body)))
        for p in ("/../secret", "/src/%2e%2e/%2e%2e/etc/passwd", "/server.mjs"):
            self.assertEqual(self.get(p)[0].split()[1], "404", p)

    def ws_request(self, origin="http://127.0.0.1:8788", host="127.0.0.1:8788"):
        lines = ["GET /ws HTTP/1.1", "Host: " + host, "Upgrade: websocket", "Connection: Upgrade",
                 "Sec-WebSocket-Version: 13", "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ=="]
        if origin is not None:
            lines.append("Origin: " + origin)
        return ("\r\n".join(lines) + "\r\n\r\n").encode("latin-1")

    def test_ws_refused_origin_and_host(self):
        for origin, host, code in (("http://evil.example", "127.0.0.1:8788", "403"),
                                   (None, "127.0.0.1:8788", "403"),
                                   ("http://127.0.0.1:8788", "evil.example", "403")):
            status, h, body = self.request(self.ws_request(origin, host))
            self.assertEqual(status.split()[1], code, (origin, host))
        self.assertEqual(self.bridge.counts["ws_sessions"], 0)

    def read_frames(self, sock, dec, n):
        out = []
        while len(out) < n:
            data = sock.recv(65536)
            if not data:
                break
            out += dec.feed(data)
        return out

    def test_ws_session_end_to_end(self):
        a, t = self.connect()
        a.sendall(self.ws_request())
        buf = b""
        while b"\r\n\r\n" not in buf:
            buf += a.recv(4096)
        head, _, rest = buf.partition(b"\r\n\r\n")
        lines = head.decode("latin-1").split("\r\n")
        self.assertEqual(lines[0], "HTTP/1.1 101 Switching Protocols")
        self.assertIn("Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=", lines)
        dec = ws.FrameDecoder(require_mask=False, max_message=ws.SERVER_MAX_FRAME)
        events = dec.feed(rest) if rest else []
        events += self.read_frames(a, dec, 2 - len(events))
        self.assertEqual(events[0][0], "text")
        self.assertEqual(json.loads(events[0][1])["type"], "hello")
        self.assertEqual(events[1][0], "binary")
        self.assertEqual(gp.decode_cloud(events[1][1])[0]["count"], 0)
        # a command while the hub link is down -> link_down ack; ping -> pong; snapshots keep coming
        self.client.link_state = "down"
        cmd = json.dumps({"type": "command", "version": 2, "id": "w1", "action": "find",
                          "payload": {"text": "杯子"}}).encode("utf-8")
        a.sendall(cframe(ws.OP_TEXT, cmd[:10], fin=False) + cframe(ws.OP_PING, b"hb")
                  + cframe(ws.OP_CONT, cmd[10:]))
        got = {}
        deadline = time.time() + 5
        while time.time() < deadline and not ("ack" in got and "pong" in got and "snapshot" in got):
            for kind, val in self.read_frames(a, dec, 1):
                if kind == "pong":
                    got["pong"] = val
                elif kind == "text":
                    m = json.loads(val)
                    got.setdefault(m["type"], m)
        self.assertEqual(got["pong"], b"hb")
        self.assertEqual((got["ack"]["id"], got["ack"]["reasonCode"]), ("w1", "link_down"))
        self.assertGreaterEqual(got["snapshot"]["seq"], 1)
        # client close -> echoed close, handler thread ends
        a.sendall(cframe(ws.OP_CLOSE, ws.close_payload(1000, "bye")))
        tail = []
        while True:
            try:
                data = a.recv(65536)
            except OSError:
                break
            if not data:
                break
            try:
                tail += dec.feed(data)
            except ws.WsError:
                break
            if any(k == "close" for k, _ in tail):
                break
        self.assertIn(("close", (1000, "")), tail)
        t.join(5)
        self.assertFalse(t.is_alive())
        self.assertEqual(len(self.bridge.sessions), 0)

    def test_ws_protocol_error_closes_1002(self):
        a, t = self.connect()
        a.sendall(self.ws_request())
        buf = b""
        while b"\r\n\r\n" not in buf:
            buf += a.recv(4096)
        a.sendall(ws.encode_frame(ws.OP_TEXT, b"unmasked"))
        dec = ws.FrameDecoder(require_mask=False, max_message=ws.SERVER_MAX_FRAME)
        rest = buf.partition(b"\r\n\r\n")[2]
        events = dec.feed(rest) if rest else []
        while not any(k == "close" for k, _ in events):
            data = a.recv(65536)
            if not data:
                break
            events += dec.feed(data)
        self.assertIn(("close", (1002, "client frame not masked")), events)
        t.join(5)
        self.assertEqual(self.bridge.counts["ws_protocol_close:1002"], 1)


class MainArgTests(unittest.TestCase):
    def test_listen_must_be_loopback(self):
        tok = tempfile.NamedTemporaryFile("w", delete=False)
        tok.write("t" * 24)
        tok.close()
        try:
            with self.assertRaises(SystemExit) as cm:
                gb.main(["--hub", "127.0.0.1:47431", "--token-file", tok.name, "--listen", "0.0.0.0:8788"])
            self.assertIn("127.0.0.1", str(cm.exception.code))
            with self.assertRaises(SystemExit):
                gb.main(["--hub", "nohostport", "--token-file", tok.name])
        finally:
            os.unlink(tok.name)
        with self.assertRaises(SystemExit) as cm:
            gb.main(["--hub", "127.0.0.1:47431", "--token-file", "/nonexistent/token"])
        self.assertIn("口令", str(cm.exception.code))

    def test_no_host_imports_and_py38_syntax(self):
        import ast
        for name in ("geomem_bridge.py", "gmb_proto.py", "gmb_ws.py", "gmb_basemap.py"):
            src = (ROOT / "ui" / "geomem" / "bridge" / name).read_text(encoding="utf-8")
            tree = ast.parse(src)
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom):
                    self.assertFalse((node.module or "").startswith("host"), name)
                if isinstance(node, ast.Import):
                    for a in node.names:
                        self.assertFalse(a.name.startswith("host"), name)
            self.assertFalse("removeprefix" in src, name)
            self.assertFalse("setsockopt" in src, name)
        self.assertEqual(gb.BridgeHTTPServer.allow_reuse_address, os.name != "nt")


# ---------------------------------------------------------------------------- kiosk additions (§2.10, 10-17)

import gmb_basemap as gbm  # noqa: E402

LIB_A, LIB_B = "a" * 64, "b" * 64


def write_pkg(root, lib_sha, cells=b"", name="lab.db", **override):
    """A base-map package as make_basemap.py writes it (stdlib only here; the tool itself is tested in
    tests/test_geomem_basemap.py, which also loads a real tool output through gmb_basemap)."""
    d = os.path.join(root, "basemap-" + lib_sha[:12])
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "cells.bin"), "wb") as f:
        f.write(cells)
    import hashlib
    man = {"format": "geomem-basemap", "version": 1, "frame": "map", "cell_m": 0.02,
           "source_db": {"name": name, "sha256": lib_sha, "bytes": 123},
           "files": {"cells": {"name": "cells.bin", "bytes": len(cells), "sha256": hashlib.sha256(cells).hexdigest(),
                               "count": len(cells) // 9}}}
    man.update(override)
    with open(os.path.join(d, "basemap.json"), "w", encoding="utf-8") as f:
        json.dump(man, f)
    return d


def cells_of(n):
    return b"".join(struct.pack("<3h", k, -k, 1) for k in range(n)) + bytes(range(3)) * n


class BasemapPackageTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="gmb_basemap_")

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_load_ok(self):
        cells = cells_of(5)
        p = gbm.load_package(write_pkg(self.tmp, LIB_A, cells))
        self.assertEqual((p["sha256"], p["count"], p["cells"], p["name"], p["cell_m"]), (LIB_A, 5, cells, "lab.db", 0.02))

    def test_tampered_or_wrong_packages_refused(self):
        d = write_pkg(self.tmp, LIB_A, cells_of(4))
        with open(os.path.join(d, "cells.bin"), "r+b") as f:     # flip one byte: sha no longer matches
            f.seek(3)
            f.write(b"\x7f")
        with self.assertRaises(gbm.BasemapError) as cm:
            gbm.load_package(d)
        self.assertIn("sha256", str(cm.exception))
        cases = [dict(format="other"), dict(version=2), dict(frame="odom"), dict(cell_m=0),
                 dict(source_db={"sha256": "A" * 64}), dict(source_db={"sha256": "a" * 63}),
                 dict(files={"cells": {"name": "x.bin"}})]
        for k, over in enumerate(cases):
            sub = os.path.join(self.tmp, "c%d" % k)
            with self.assertRaises(gbm.BasemapError, msg=over):
                gbm.load_package(write_pkg(sub, LIB_A, cells_of(2), **over))
        d = write_pkg(os.path.join(self.tmp, "len"), LIB_A, b"\x00" * 10)      # not a multiple of 9
        with self.assertRaises(gbm.BasemapError):
            gbm.load_package(d)
        with self.assertRaises(gbm.BasemapError):
            gbm.load_package(os.path.join(self.tmp, "missing"))
        a1 = gbm.load_package(write_pkg(os.path.join(self.tmp, "d1"), LIB_A, cells_of(1)))
        a2 = gbm.load_package(write_pkg(os.path.join(self.tmp, "d2"), LIB_A, cells_of(2)))
        with self.assertRaises(gbm.BasemapError):
            gbm.index_packages([a1, a2])

    def test_status_matrix(self):
        pkgs = gbm.index_packages([gbm.load_package(write_pkg(self.tmp, LIB_A, cells_of(3)))])
        self.assertIsNone(gbm.basemap_status("loc", LIB_A, "operator", {}))
        ok = gbm.basemap_status("loc", LIB_A, "operator", pkgs)
        self.assertEqual((ok["status"], ok["basis"], ok["sha256"], ok["url"], ok["cellsUrl"], ok["count"],
                          ok["idSource"], ok["loaded"]),
                         ("ok", "localized", LIB_A, "/basemap/%s/basemap.json" % LIB_A, "/basemap/%s/cells.bin" % LIB_A,
                          3, "operator", [LIB_A]))
        # 10-18 (owner): the odometry tier with the right id is drawable, basis "mapping_start"
        od = gbm.basemap_status("odom", LIB_A, "operator", pkgs)
        self.assertEqual((od["status"], od["basis"], od["sha256"], od["tier"], od["count"]),
                         ("ok", "mapping_start", LIB_A, "odom", 3))
        for tier, mid, want in (("sim", None, "not_localized"), ("sim", LIB_A, "not_localized"),
                                ("unknown", LIB_A, "not_localized"), ("loc", None, "no_map_id"),
                                ("odom", None, "no_map_id"), ("odom", LIB_A[:-1] + "b", "mismatch"),
                                ("loc", LIB_B, "mismatch"), ("loc", LIB_A[:-1] + "b", "mismatch")):
            st = gbm.basemap_status(tier, mid, "car", pkgs)
            self.assertEqual((st["status"], st["basis"], st["sha256"], st["url"], st["tier"]),
                             (want, None, None, None, tier), (tier, mid))
        # a watched directory without packages yet: the key is present, mismatch with nothing loaded
        w = gbm.basemap_status("odom", LIB_A, "operator", {}, configured=True)
        self.assertEqual((w["status"], w["loaded"]), ("mismatch", []))

    def test_resolve_only_loaded_files(self):
        cells = cells_of(2)
        pkgs = gbm.index_packages([gbm.load_package(write_pkg(self.tmp, LIB_A, cells))])
        body, ctype = gbm.resolve("/basemap/%s/cells.bin" % LIB_A, pkgs)
        self.assertEqual((body, ctype), (cells, "application/octet-stream"))
        self.assertEqual(json.loads(gbm.resolve("/basemap/%s/basemap.json" % LIB_A, pkgs)[0])["source_db"]["sha256"], LIB_A)
        for bad in ("/basemap/%s/cells.bin" % LIB_B, "/basemap/%s/../cells.bin" % LIB_A, "/basemap/%s/x" % LIB_A,
                    "/basemap/%s/cells.bin" % LIB_A.upper(), "/basemap/%s/basemap.json/" % LIB_A, "/basemap/"):
            self.assertIsNone(gbm.resolve(bad, pkgs), bad)


class KioskSnapshotTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="gmb_kiosk_")
        self.pkg = gbm.load_package(write_pkg(self.tmp, LIB_A, cells_of(7)))
        self.clock = Clock()
        self.client = FakeClient()
        self.client.runtime_info.value = {"pose_source": "estimated", "sensor_source": "hardware"}

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def bridge(self, map_id=LIB_A, basemaps=None):
        b = gb.Bridge(map_id=map_id, clock=self.clock, bridge_id="bid",
                      basemaps=[self.pkg] if basemaps is None else basemaps).attach(self.client)
        b.on_link("up", {})
        return b

    def tick(self, b, state):
        b.on_frame(FakeFrame("STATE"), state)
        return b.tick()

    def test_basemap_in_snapshot(self):
        snap = self.tick(self.bridge(), loc_state())
        bm = snap["map"]["basemap"]
        self.assertEqual((bm["status"], bm["sha256"], bm["idSource"], snap["map"]["id"]), ("ok", LIB_A, "operator", LIB_A))
        # one hex digit off on the operator side (the positive control of the screenshot check)
        snap = self.tick(self.bridge(map_id=LIB_A[:-1] + "0"), loc_state())
        self.assertEqual(snap["map"]["basemap"]["status"], "mismatch")
        # the car's own pose.map_id wins over --map-id (H1, once the car reports it)
        st = loc_state()
        st["pose"] = dict(st["pose"], map_id=LIB_B)
        snap = self.tick(self.bridge(), st)
        self.assertEqual((snap["map"]["basemap"]["status"], snap["map"]["basemap"]["idSource"]), ("mismatch", "car"))
        st["pose"]["map_id"] = LIB_A
        snap = self.tick(self.bridge(map_id=None), st)
        self.assertEqual((snap["map"]["basemap"]["status"], snap["map"]["basemap"]["idSource"]), ("ok", "car"))
        snap = self.tick(self.bridge(map_id=None), loc_state())
        self.assertEqual(snap["map"]["basemap"]["status"], "no_map_id")
        snap = self.tick(self.bridge(basemaps=[]), loc_state())
        self.assertNotIn("basemap", snap["map"])

    def odom_state(self, **kw):
        self.client.runtime_info.value = {"pose_source": "reference", "sensor_source": "hardware"}
        st = loc_state(odom={"x": 0.5, "y": -0.2, "yaw": 0.1, "v": 0.0, "age_ms": 20}, **kw)
        del st["pose"]
        return st

    def test_basemap_on_odometry_tier(self):
        """10-18 (owner): odometry tier + operator id equal to the package -> ok / mapping_start; map.id and the
        cloud header stay null on this tier (the console page compares them); one hex digit off -> mismatch;
        sim -> not_localized even with the right id."""
        b = self.bridge()
        snap = self.tick(b, self.odom_state())
        bm = snap["map"]["basemap"]
        self.assertEqual((bm["status"], bm["basis"], bm["tier"], bm["sha256"], bm["idSource"], snap["map"]["id"],
                          snap["map"]["frame"], snap["robot"]["pose"]),
                         ("ok", "mapping_start", "odom", LIB_A, "operator", None, "odom", "odometry"))
        self.assertNotIn("basemap_ok", snap.get("counts") or {})
        self.assertNotIn(LIB_A.encode(), b.cloud_bytes)          # cloud mapId not set from the basemap id
        snap = self.tick(self.bridge(map_id=LIB_A[:-1] + "0"), self.odom_state())
        self.assertEqual((snap["map"]["basemap"]["status"], snap["map"]["basemap"]["sha256"]), ("mismatch", None))
        snap = self.tick(self.bridge(map_id=None), self.odom_state())
        self.assertEqual(snap["map"]["basemap"]["status"], "no_map_id")
        st = self.odom_state()
        st["source"] = "sim"
        snap = self.tick(self.bridge(), st)
        self.assertEqual((snap["map"]["basemap"]["status"], snap["map"]["basemap"]["tier"]), ("not_localized", "sim"))
        # the loc tier keeps basis "localized"
        self.client.runtime_info.value = {"pose_source": "estimated", "sensor_source": "hardware"}
        snap = self.tick(self.bridge(), loc_state())
        self.assertEqual((snap["map"]["basemap"]["basis"], snap["map"]["id"]), ("localized", LIB_A))

    def test_runtime_load_from_watched_dir(self):
        """10-18: --basemap-watch: a package that appears later is loaded without a restart; half-copied,
        tampered, misnamed, symlinked, temp-named and duplicate packages are not; the HTTP resolver sees it."""
        watch = os.path.join(self.tmp, "watch")
        os.makedirs(watch)
        logs = []
        b = gb.Bridge(map_id=LIB_B, clock=self.clock, bridge_id="bid", basemaps=[self.pkg], basemap_dir=watch,
                      log=logs.append).attach(self.client)
        b.on_link("up", {})
        self.client.runtime_info.value = {"pose_source": "reference", "sensor_source": "hardware"}
        st = self.odom_state()
        self.assertEqual(b.poll_basemap_dir(), [])
        snap = self.tick(b, st)
        self.assertEqual((snap["map"]["basemap"]["status"], snap["map"]["basemap"]["loaded"]), ("mismatch", [LIB_A]))
        self.assertIsNone(b.basemap_file("/basemap/%s/cells.bin" % LIB_B))
        # make_basemap's temp name and an unrelated name are never looked at
        write_pkg(os.path.join(self.tmp, "x"), LIB_B, cells_of(4))
        os.rename(os.path.join(self.tmp, "x", "basemap-" + LIB_B[:12]), os.path.join(watch, "basemap-%s.tmp-123" % LIB_B[:12]))
        self.assertEqual(b.poll_basemap_dir(), [])
        # half-copied: cells.bin shorter than registered -> not loaded, retried after the file changes
        d = write_pkg(watch, LIB_B, cells_of(5))
        with open(os.path.join(d, "cells.bin"), "rb") as f:
            full = f.read()
        with open(os.path.join(d, "cells.bin"), "wb") as f:
            f.write(full[:18])
        self.assertEqual(b.poll_basemap_dir(), [])
        self.assertEqual(b.poll_basemap_dir(), [])
        n_logs = len(logs)
        self.assertEqual(b.poll_basemap_dir(), [])
        self.assertEqual(len(logs), n_logs)                       # logged once per change, not every poll
        with open(os.path.join(d, "cells.bin"), "wb") as f:
            f.write(full)
        os.utime(os.path.join(d, "cells.bin"), ns=(1, 2))         # a changed signature even on coarse clocks
        self.assertEqual(b.poll_basemap_dir(), [LIB_B])
        snap = self.tick(b, st)
        bm = snap["map"]["basemap"]
        self.assertEqual((bm["status"], bm["basis"], bm["sha256"], bm["count"], bm["loaded"]),
                         ("ok", "mapping_start", LIB_B, 5, sorted([LIB_A, LIB_B])))
        self.assertEqual(b.basemap_file("/basemap/%s/cells.bin" % LIB_B)[0], full)
        self.assertEqual(b.poll_basemap_dir(), [])                # nothing new
        # the same library regenerated in place (make_basemap --force) -> ignored, the loaded one is kept
        write_pkg(watch, LIB_B, cells_of(6))
        os.utime(os.path.join(d, "cells.bin"), ns=(3, 4))
        self.assertEqual(b.poll_basemap_dir(), [])
        self.assertTrue(any("已加载" in m for m in logs), logs)
        self.assertEqual(b.basemaps[LIB_B]["count"], 5)
        # misnamed: manifest library does not start with the directory's 12 digits
        lib_d = "d" * 64
        d2 = write_pkg(os.path.join(self.tmp, "y"), lib_d, cells_of(2))
        os.rename(d2, os.path.join(watch, "basemap-" + "e" * 12))
        self.assertEqual(b.poll_basemap_dir(), [])
        self.assertTrue(any("与目录名不符" in m for m in logs), logs)
        # symlink to a valid package -> ignored
        if hasattr(os, "symlink"):
            d3 = write_pkg(os.path.join(self.tmp, "z"), "f" * 64, cells_of(2))
            os.symlink(d3, os.path.join(watch, "basemap-" + "f" * 12))
            self.assertEqual(b.poll_basemap_dir(), [])
        self.assertEqual(sorted(b.basemaps), sorted([LIB_A, LIB_B]))

    def test_watch_limits_and_threads(self):
        watch = os.path.join(self.tmp, "watch2")
        os.makedirs(watch)
        saved = gbm.MAX_PACKAGES
        gbm.MAX_PACKAGES = 2
        try:
            b = gb.Bridge(map_id=LIB_A, clock=self.clock, bridge_id="bid", basemaps=[self.pkg],
                          basemap_dir=watch).attach(self.client)
            for lib in ("1" * 64, "2" * 64):
                write_pkg(watch, lib, cells_of(1))
            self.assertEqual(b.poll_basemap_dir(), ["1" * 64])    # sorted names; the second is over the limit
            self.assertEqual(len(b.basemaps), 2)
        finally:
            gbm.MAX_PACKAGES = saved
        # the watch thread: start() polls at once and stop() ends it
        watch3 = os.path.join(self.tmp, "watch3")
        os.makedirs(watch3)
        write_pkg(watch3, LIB_B, cells_of(3))
        b = gb.Bridge(map_id=LIB_B, clock=self.clock, bridge_id="bid", basemap_dir=watch3).attach(self.client)
        b.start()
        try:
            deadline = time.monotonic() + 5.0
            while LIB_B not in b.basemaps and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertIn(LIB_B, b.basemaps)
            self.assertIn("gmb-basemap-watch", [t.name for t in b._threads])
        finally:
            b.stop()
        self.assertFalse(any(t.is_alive() for t in b._threads if t.name == "gmb-basemap-watch"))

    def test_fpga_and_query_text(self):
        b = self.bridge(basemaps=[])
        snap = self.tick(b, loc_state())
        self.assertNotIn("fpga", snap)                       # STATE without a kv group
        self.assertNotIn("queryText", snap["task"])
        kv = {"link_ok": True, "age_ms": 80.0,
              "geometry": {"avail": True, "age_ms": 60.0, "pl_device_ms": 12.25},
              "roi": {"avail": True, "age_ms": 1500.0, "rows": 8},
              "permit": {"avail": True, "age_ms": 40.0, "heartbeat_on": False},
              "ar": {"avail": True, "mode": "replay"}}
        st = loc_state(kv=kv, task={"state": "navigating", "reason_task": "search", "query_seq": 3, "age_ms": 5,
                                    "query_text": "  水瓶 ", "target_id": None})
        snap = self.tick(b, st)
        fpga = dict(snap["fpga"])
        live = fpga.pop("live")                              # 10-29 console FPGA panel (own tests)
        self.assertEqual(fpga, {"linkOk": True, "geometryMs": 12.25, "geometryAgeMs": 60.0, "roiRows": 8,
                                "roiAgeMs": 1500.0, "heartbeatOn": False, "ageMs": 80.0})
        self.assertIsNone(live["geoMs"])                     # STATE without perf: no per-frame numbers
        self.assertEqual((snap["task"]["state"], snap["task"]["queryText"]), ("searching", "水瓶"))
        kv2 = {"link_ok": False, "geometry": {"avail": False, "reason": "telemetry_lost", "pl_device_ms": 3.0},
               "roi": {"avail": True, "rows": True}, "permit": {"avail": True, "heartbeat_on": 1}}
        snap = self.tick(b, loc_state(kv=kv2))
        fpga = dict(snap["fpga"])
        self.assertIsNone(fpga.pop("live")["permit"])        # telemetry lost: no permit shown
        self.assertEqual(fpga, {"linkOk": False, "geometryMs": None, "geometryAgeMs": None, "roiRows": None,
                                "roiAgeMs": None, "heartbeatOn": None, "ageMs": None})
        snap = self.tick(b, loc_state(kv={"link_ok": False, "geometry": None, "permit": None, "roi": None}))
        self.assertEqual(snap["fpga"]["linkOk"], False)
        self.assertIsNone(snap["fpga"]["geometryMs"])
        gp.dumps(snap)                                       # wire-encodable

    def test_old_keys_unchanged(self):
        """The kiosk additions only add keys: every v2 key keeps its value with or without a package."""
        a = self.tick(self.bridge(basemaps=[]), loc_state())
        b = self.tick(self.bridge(), loc_state())
        b["map"].pop("basemap")
        for k in ("seq", "serverTime"):
            a.pop(k, None)
            b.pop(k, None)
        self.assertEqual(a, b)


class KioskHttpTests(HttpShellTests):
    def setUp(self):
        HttpShellTests.setUp(self)
        self.bridge.stop()
        self.cells = cells_of(11)
        pkg = gbm.load_package(write_pkg(os.path.join(self.tmp, "pkgs"), LIB_A, self.cells))
        with open(os.path.join(self.tmp, "kiosk.html"), "wb") as f:
            f.write(b"<!doctype html>kiosk")
        self.bridge = gb.Bridge(bridge_id="bid", basemaps=[pkg]).attach(self.client)
        self.bridge.start()

    def test_kiosk_page_and_basemap_files(self):
        status, h, body = self.get("/kiosk.html")
        self.assertEqual((status, body), ("HTTP/1.1 200 OK", b"<!doctype html>kiosk"))
        status, h, body = self.get("/basemap/%s/cells.bin" % LIB_A)
        self.assertEqual((status, body, h["content-type"], h["content-length"]),
                         ("HTTP/1.1 200 OK", self.cells, "application/octet-stream", str(len(self.cells))))
        status, h, body = self.get("/basemap/%s/basemap.json" % LIB_A)
        self.assertEqual((status, json.loads(body)["source_db"]["sha256"]), ("HTTP/1.1 200 OK", LIB_A))
        status, h, body = self.request(("HEAD /basemap/%s/cells.bin HTTP/1.1\r\nHost: 127.0.0.1:8788\r\n"
                                        "Connection: close\r\n\r\n" % LIB_A).encode("latin-1"))
        self.assertEqual((status, body, h["content-length"]), ("HTTP/1.1 200 OK", b"", str(len(self.cells))))
        for p in ("/basemap/%s/cells.bin" % LIB_B, "/basemap/%s/%%2e%%2e/basemap.json" % LIB_A,
                  "/basemap/pkgs/basemap-aaaaaaaaaaaa/cells.bin", "/pkgs/basemap-aaaaaaaaaaaa/cells.bin"):
            self.assertEqual(self.get(p)[0].split()[1], "404", p)
        self.assertEqual(self.get("/basemap/%s/cells.bin" % LIB_A, host="evil.example:8788")[0].split()[1], "403")

    def test_main_refuses_bad_package(self):
        d = write_pkg(os.path.join(self.tmp, "bad"), LIB_B, cells_of(2), version=9)
        tok = tempfile.NamedTemporaryFile("w", delete=False)
        tok.write("t")
        tok.close()
        try:
            with self.assertRaises(SystemExit) as cm:
                gb.main(["--hub", "127.0.0.1:47431", "--token-file", tok.name, "--basemap", d])
            self.assertIn("底图包无效", str(cm.exception.code))
        finally:
            os.unlink(tok.name)


if __name__ == "__main__":
    unittest.main()
