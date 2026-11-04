import json
import socket
import struct
import tempfile
import threading
import time
import unittest
from pathlib import Path
import numpy as np

from host.inference_service import (InferenceClient, MAX_BYTES, decode_frame, encode_frame,
                                    process_request, receive_message, send_message)
from perception.fpga_replay.synthetic import scene


class ServiceTests(unittest.TestCase):
    def test_roundtrip_preserves_depth_pose_and_frame(self):
        frame, _ = scene(3, camera_x=0.12)
        frame.depth[0, 0] = np.nan
        decoded = decode_frame(encode_frame(frame))
        for key in ("rgb", "depth", "K", "T_world_camera"):
            np.testing.assert_equal(getattr(decoded, key), getattr(frame, key))
        self.assertEqual(decoded.timestamp, frame.timestamp)
        self.assertEqual(decoded.frame_id, frame.frame_id)

    def test_detector_foreign_frame_rejected(self):
        frame, _ = scene(1)
        _, boxes = scene(0)
        with self.assertRaisesRegex(ValueError, "foreign frame"):
            process_request(encode_frame(frame), lambda _: boxes, "cpu")

    def test_truncated_and_oversized_messages(self):
        for header, content, exception in ((MAX_BYTES+1, b"", ValueError), (10, b"x", ConnectionError)):
            left, right = socket.socketpair()
            with left, right:
                left.sendall(struct.pack("!I", header) + content)
                left.shutdown(socket.SHUT_WR)
                with self.assertRaises(exception):
                    receive_message(right)

    def rpc(self, response_edit=None, expected_device="cpu"):
        frame, boxes = scene(1)
        with tempfile.TemporaryDirectory() as td:
            path = str(Path(td) / "test.sock")
            with socket.socket(socket.AF_UNIX) as listener:
                listener.bind(path)
                listener.listen(1)
                errors = []
                def server():
                    try:
                        stream, _ = listener.accept()
                        with stream:
                            reply = process_request(receive_message(stream), lambda _: boxes, "cpu")
                            if response_edit:
                                response_edit(reply)
                            send_message(stream, json.dumps(reply).encode())
                    except Exception as error:
                        errors.append(error)
                worker = threading.Thread(target=server, daemon=True)
                worker.start()
                try:
                    return InferenceClient(path, timeout=2, expected_device=expected_device).detect(frame)
                finally:
                    worker.join(3)
                    self.assertFalse(worker.is_alive())
                    if errors:
                        raise errors[0]

    def test_actual_socket_roundtrip(self):
        self.assertEqual(len(self.rpc()), 1)

    def test_slow_drip_cannot_extend_total_deadline(self):
        left, right = socket.socketpair()
        def drip():
            try:
                left.sendall(struct.pack("!I", 30))
                for _ in range(30):
                    left.sendall(b"x")
                    time.sleep(0.02)
            except OSError:
                pass
        with left, right:
            right.settimeout(0.09)
            worker = threading.Thread(target=drip, daemon=True)
            worker.start()
            started = time.monotonic()
            with self.assertRaises((TimeoutError, socket.timeout)):
                receive_message(right)
            self.assertLess(time.monotonic() - started, 0.4)
            right.shutdown(socket.SHUT_RDWR)
            worker.join(1)

    def test_stale_reply_and_backend_substitution_rejected(self):
        with self.assertRaisesRegex(ValueError, "different frame"):
            self.rpc(lambda response: response.update(timestamp=-1))
        with self.assertRaisesRegex(ValueError, "backend"):
            self.rpc(expected_device="cuda")


class WarmUp(unittest.TestCase):
    """D2 run_b2: the first request paid the CUDA lazy init (2.4-10+ s); --warmup-frames pays it before ready."""
    def test_warm_up_calls_detector_on_valid_blank_frame(self):
        from host.inference_service import warm_up
        seen = []
        def detector(frame):
            frame.validate()
            seen.append(frame.rgb.shape)
            return []
        seconds = warm_up(detector, 2, (640, 400))
        self.assertEqual(seen, [(400, 640, 3)] * 2)
        self.assertEqual(len(seconds), 2)
        self.assertEqual(warm_up(detector, 0, (640, 400)), [])
        self.assertEqual(len(seen), 2)   # known-different control: 0 frames = no call

    def test_parse_size(self):
        from host.inference_service import parse_size
        self.assertEqual(parse_size("640x400"), (640, 400))
        for bad in ("640", "0x400", "ax400"):
            with self.assertRaises(ValueError):
                parse_size(bad)

    def test_ready_line_keeps_key_and_reports_warmup(self):
        from host.inference_service import serve
        import contextlib, io
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "s.sock"
            out = io.StringIO()
            def stop_soon():
                for _ in range(100):
                    if path.exists():
                        break
                    time.sleep(0.01)
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as c:
                    c.connect(str(path))
                    c.sendall(struct.pack("!I", 1) + b"x")
                    c.recv(1 << 16)
            worker = threading.Thread(target=stop_soon, daemon=True)
            worker.start()
            class Stop(Exception):
                pass
            def detector(frame):
                raise Stop()
            # serve() loops forever: break out by raising from send via a patched send_message
            import host.inference_service as svc
            orig = svc.send_message
            def send_then_stop(stream, body):
                orig(stream, body)
                raise KeyboardInterrupt
            svc.send_message = send_then_stop
            try:
                with contextlib.redirect_stdout(out):
                    with self.assertRaises(KeyboardInterrupt):
                        serve(path, detector, "cuda", ready_extra={"warmup_s": [2.3, 0.02]})
            finally:
                svc.send_message = orig
            worker.join(3)
            line = json.loads(out.getvalue().splitlines()[0])
            self.assertIn('"ready": true', out.getvalue())
            self.assertEqual(line["warmup_s"], [2.3, 0.02])
            self.assertFalse(path.exists())


if __name__ == "__main__":
    unittest.main()
