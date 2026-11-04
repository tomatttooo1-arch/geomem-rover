"""tools/integ_car_chain/rx_lite.py (2026-10-13): the CDR head parser of the light image receiver.

Known answers built by hand (no ROS): a serialized sensor_msgs/Image head in both byte orders and with frame
ids of every length modulo 4 (the padding before height and before step moves with them); the stamp-only
path agrees with the full head; a head shifted by one byte or cut short is refused, not misread; the
comparison with a normally deserialized message notices each field. Python 3.8+.
2026-10-14: the DDS message-lost callback (stand-in status objects: the running total, the per-event change,
the wall-time log and its cap); the source subscribes with event_callbacks and falls back without them,
recorded as unsupported (never a silent 0).
"""
from pathlib import Path
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.integ_car_chain import rx_lite  # noqa: E402


def cdr_image(sec, nsec, frame_id, height, width, encoding, step, data, little=True):
    """Serialized sensor_msgs/Image as Fast CDR writes it: 4-byte encapsulation, then the body with every
    primitive aligned to its size relative to the body start."""
    e = "<" if little else ">"
    body = bytearray()

    def pad(n):
        while len(body) % n:
            body.append(0)

    def string(s):
        pad(4)
        b = s.encode() + b"\0"
        body.extend(struct.pack(e + "I", len(b)))
        body.extend(b)

    body.extend(struct.pack(e + "iI", sec, nsec))
    string(frame_id)
    pad(4)
    body.extend(struct.pack(e + "II", height, width))
    string(encoding)
    body.append(0)                              # is_bigendian
    pad(4)
    body.extend(struct.pack(e + "I", step))
    body.extend(struct.pack(e + "I", len(data)))
    body.extend(data)
    return bytes([0, 1 if little else 0, 0, 0]) + bytes(body)


class Stamp:
    def __init__(self, sec, nanosec):
        self.sec, self.nanosec = sec, nanosec


class Header:
    def __init__(self, sec, nanosec, frame_id):
        self.stamp, self.frame_id = Stamp(sec, nanosec), frame_id


class Msg:
    def __init__(self, sec, nanosec, frame_id, height, width, encoding, step, data):
        self.header = Header(sec, nanosec, frame_id)
        self.height, self.width, self.encoding, self.step, self.data = height, width, encoding, step, data


class ParseHead(unittest.TestCase):
    def test_known_answers_both_byte_orders_and_every_padding(self):
        data = bytes(range(48))
        for little in (True, False):
            for frame_id in ("", "a", "ab", "abc", "abcd", "camera_color_optical_frame"):
                for encoding in ("16UC1", "rgb8", "mono16x"):
                    raw = cdr_image(1791868606, 782171248, frame_id, 480, 640, encoding, 1280, data, little)
                    h = rx_lite.parse_head(raw)
                    self.assertEqual(h, {"stamp_ns": 1791868606782171248, "frame_id": frame_id, "height": 480,
                                         "width": 640, "encoding": encoding, "step": 1280, "data_len": 48})
                    self.assertEqual(rx_lite.parse_stamp_ns(raw), 1791868606782171248)

    def test_the_real_sizes(self):
        raw = cdr_image(5, 999999999, "camera_color_optical_frame", 480, 640, "16UC1", 1280, bytes(614400))
        h = rx_lite.parse_head(raw)
        self.assertEqual((h["stamp_ns"], h["data_len"]), (5999999999, 614400))
        self.assertEqual(len(raw) - 614400, 4 + 8 + 4 + 27 + 1 + 8 + 4 + 6 + 1 + 1 + 4 + 4)

    def test_negative_seconds_are_signed(self):
        raw = cdr_image(-1, 5, "f", 1, 1, "mono8", 1, b"\0")
        self.assertEqual(rx_lite.parse_stamp_ns(raw), -1000000000 + 5)

    def test_refuses_what_is_not_plain_cdr_or_is_cut(self):
        raw = cdr_image(1, 2, "frame", 4, 4, "mono8", 4, bytes(16))
        for bad in (raw[:10], bytes([0, 3, 0, 0]) + raw[4:], bytes([1, 1, 0, 0]) + raw[4:], raw[:20]):
            with self.assertRaises((ValueError, struct.error)):
                rx_lite.parse_head(bad)

    def test_positive_control_a_shifted_head_does_not_pass_as_the_message(self):
        raw = cdr_image(1791868606, 782171248, "camera_color_optical_frame", 480, 640, "rgb8", 1920, bytes(64))
        msg = Msg(1791868606, 782171248, "camera_color_optical_frame", 480, 640, "rgb8", 1920, bytes(64))
        self.assertTrue(rx_lite.same_as_message(rx_lite.parse_head(raw), msg))
        shifted = raw[:4] + b"\0" + raw[4:]
        try:
            ok = rx_lite.same_as_message(rx_lite.parse_head(shifted), msg)
        except (ValueError, struct.error):
            ok = False
        self.assertFalse(ok)

    def test_the_comparison_notices_each_field(self):
        raw = cdr_image(10, 20, "f", 480, 640, "rgb8", 1920, bytes(64))
        head = rx_lite.parse_head(raw)
        good = dict(sec=10, nanosec=20, frame_id="f", height=480, width=640, encoding="rgb8", step=1920,
                    data=bytes(64))
        self.assertTrue(rx_lite.same_as_message(head, Msg(**good)))
        for key, other in (("sec", 11), ("nanosec", 21), ("frame_id", "g"), ("height", 481), ("width", 641),
                           ("encoding", "bgr8"), ("step", 1921), ("data", bytes(63))):
            self.assertFalse(rx_lite.same_as_message(head, Msg(**dict(good, **{key: other}))), key)


class LostStatus:
    def __init__(self, total_count, total_count_change):
        self.total_count, self.total_count_change = total_count, total_count_change


class LostCallback(unittest.TestCase):
    def test_running_total_changes_and_log(self):
        rec = rx_lite.new_lost_record()
        clock = iter([100, 200, 300])
        on = rx_lite.lost_callback(rec, wall=lambda: next(clock))
        for st in (LostStatus(1, 1), LostStatus(4, 3), LostStatus(5, 1)):
            on(st)
        self.assertEqual((rec["total_count"], rec["events"], rec["change_sum"]), (5, 3, 5))
        self.assertEqual(rec["log"], [[100, 1, 1], [200, 3, 4], [300, 1, 5]])
        self.assertTrue(rec["supported"])

    def test_a_fresh_record_is_zero_and_the_log_is_capped(self):
        rec = rx_lite.new_lost_record()
        self.assertEqual((rec["total_count"], rec["events"], rec["change_sum"], rec["log"]), (0, 0, 0, []))
        on = rx_lite.lost_callback(rec, wall=lambda: 7)
        for i in range(rx_lite.LOST_LOG_MAX + 5):
            on(LostStatus(i + 1, 1))
        self.assertEqual(len(rec["log"]), rx_lite.LOST_LOG_MAX)
        self.assertEqual((rec["events"], rec["total_count"]), (rx_lite.LOST_LOG_MAX + 5, rx_lite.LOST_LOG_MAX + 5))

    def test_incompatible_qos_total(self):
        store = {}
        rx_lite.incompatible_callback(store, "/t")(LostStatus(2, 1))
        self.assertEqual(store, {"/t": 2})


class Source(unittest.TestCase):
    def test_lost_event_subscription_and_fallback(self):
        src = (ROOT / "tools/integ_car_chain/rx_lite.py").read_text()
        self.assertIn("SubscriptionEventCallbacks(message_lost=lost_callback(lost[t])", src)
        self.assertIn("raw=True, event_callbacks=ev)", src)
        self.assertIn('lost[t] = {"supported": False, "error": repr(exc)}', src)
        self.assertIn("lost=lost.get(t), incompatible_qos=incompatible.get(t))", src)
        # every topic (images and camera_info) goes through the same subscribe()
        self.assertEqual(src.count("node.create_subscription("), 2)
        self.assertIn("subscribe(Image, t, cb(t))", src)
        self.assertIn("subscribe(CameraInfo, t, info_cb(t))", src)

    def test_raw_subscription_and_own_signal_handler(self):
        src = (ROOT / "tools/integ_car_chain/rx_lite.py").read_text()
        self.assertIn("raw=True", src)
        self.assertIn("signal.signal(signal.SIGINT, on_signal)", src)
        self.assertIn("signal.signal(signal.SIGTERM, on_signal)", src)
        self.assertNotIn("pkill", src)


if __name__ == "__main__":
    unittest.main()
