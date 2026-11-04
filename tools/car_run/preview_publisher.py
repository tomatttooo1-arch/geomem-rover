#!/usr/bin/env python3
"""Optional camera-topic subscriber. Never opens camera hardware.

10-21: first-person preview raised from 2 fps / 320x240 / 30 KiB to PREVIEW_FPS / 640x480 / 160 KiB
(the operator console now gets each frame pushed over its WebSocket; the old cap made driving blind).
Limits must stay <= the hub's feed_preview and the bridge's PREVIEW checks.

10-29 (CPU): the camera topic is taken serialized (raw) and only a frame that goes out is deserialized; the
others (1 of 3 at 15 fps) are dropped as bytes, no Image object. The pixel buffer is read in place
(numpy view of msg.data, no bytes() copy). The JPEG goes into CompressedImage.data as array.array('B'):
given bytes, the generated setter checks every byte in Python (2 x 40 000 checks per frame, the largest cost
of this process in a profile). Resolution, JPEG quality and output rate are unchanged.
"""
import array
import os
import time

PREVIEW_MAX_W, PREVIEW_MAX_H = 640, 480
PREVIEW_MAX_BYTES = 160 * 1024
PREVIEW_QUALITIES = (60, 45, 35)            # q60 640x480 ~35-40 KB: fewer bytes per frame on WiFi


def _fps(text):
    try:
        return min(15.0, max(0.5, float(text)))
    except (TypeError, ValueError):
        return 10.0


PREVIEW_FPS = _fps(os.environ.get("CAR_PREVIEW_FPS", "10"))


def preview_due(now, next_due, period):
    """(send this frame?, next due time) -- 10-29: scheduled on due times, not on the gap since the last frame.

    The old rule (skip while now - last < 0.9 period) sends every 2nd frame of a 15 fps camera = 7.5 fps instead of
    10. Here a frame goes out once it is within 0.4 period of the due time, and the due time advances by one period
    (never closer than 0.6 period ahead of now, so a stall does not cause a burst): 15 fps in -> 2 of every 3 frames
    = 10 fps out, 30 fps in -> every 3rd frame = 10 fps out (as before). ``next_due`` None = the first frame."""
    if next_due is None:
        return True, now + period
    if now < next_due - 0.4 * period:
        return False, next_due
    return True, max(next_due + period, now + 0.6 * period)


def jpeg_payload(data):
    """CompressedImage.data value for JPEG bytes: array.array('B') is taken as is by the message setter
    (bytes would be validated element by element in Python)."""
    return array.array("B", data)


def encode_preview(msg, cv2, np):
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
    n = channels.get(msg.encoding)
    if not n or msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * n:
        return None
    if len(msg.data) != msg.step * msg.height:
        return None
    try:
        flat = np.frombuffer(msg.data, dtype=np.uint8)      # array.array / bytes: a view, no copy
    except TypeError:
        flat = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    rows = flat.reshape(msg.height, msg.step)
    img = rows[:, :msg.width * n].reshape(msg.height, msg.width, n)
    scale = min(float(PREVIEW_MAX_W) / msg.width, float(PREVIEW_MAX_H) / msg.height, 1.0)
    if scale < 1.0:
        img = cv2.resize(img, (max(1, int(msg.width * scale)), max(1, int(msg.height * scale))),
                         interpolation=cv2.INTER_AREA)
    if msg.encoding == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif msg.encoding == "rgba8":
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    elif msg.encoding == "bgra8":
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    for quality in PREVIEW_QUALITIES:
        ok, data = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok and data.size <= PREVIEW_MAX_BYTES:
            return data.tobytes()
    return None


def main():
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, CompressedImage

    class Preview(Node):
        def __init__(self):
            super().__init__("geomem_preview")
            self.next_due = None
            self.pub = self.create_publisher(CompressedImage, "/fpga/preview/jpeg", 1)
            # raw=True: the callback gets the serialized message; frames that are not due are never deserialized
            self.sub = self.create_subscription(Image, "/camera/color/image_raw", self.frame, qos_profile_sensor_data,
                                                raw=True)
            self.period = 1.0 / PREVIEW_FPS
            print("preview ready: subscribed camera topic, <=%g fps, <=%dx%d"
                  % (PREVIEW_FPS, PREVIEW_MAX_W, PREVIEW_MAX_H), flush=True)

        def frame(self, raw):
            send, self.next_due = preview_due(time.monotonic(), self.next_due, self.period)
            if not send:
                return
            msg = deserialize_message(raw, Image)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            age = self.get_clock().now().nanoseconds / 1e9 - stamp
            if not 0 <= age <= 1.0:
                return
            try:
                data = encode_preview(msg, cv2, np)
                if data:
                    out = CompressedImage()
                    out.header = msg.header
                    out.format = "jpeg"
                    out.data = jpeg_payload(data)
                    self.pub.publish(out)
            except (ValueError, cv2.error):
                pass

    rclpy.init()
    node = Preview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
