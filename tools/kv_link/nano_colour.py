#!/usr/bin/env python3
"""Nano colour sender for the AR colour main view (2026-10-16, host/kv_colour.py). Its own process:
the geometry producer never subscribes to the colour (no 921 KB messages under its GIL).

Subscribes the hardware-D2C colour topic (default /camera/color/image_raw, rgb8 640x480; the car runs
the Orbbec driver with depth_registration:=true align_mode:=HW, so colour pixel (u, v) = depth pixel
(u, v)) with a RELIABLE QoS (921 KB > 512 KiB: best effort was seen to drop large messages, STATE
10-14), keeps the last --ring frames by header stamp (ns) and answers the KV's WANT (the depth frame's
header stamp, from the REQ STAMP block the producer sends) with the nearest frame within the KV's
tolerance, raw RGB888. Nothing is sent unasked except a small HELLO every 0.5 s.

Usage: nano_colour.py --kv HOST:PORT [--kv HOST2:PORT2 ...] [--dpu] [--dpu-backend auto|cv2|numpy]
                      [--topic /camera/color/image_raw] [--ring 12] [--stats-out F] [--lazy]
  HOST:PORT = a receiver: the KV's kv_server --ar-colour-port, the Nano car screen's sink (127.0.0.1:47801), or
  the KV DPU input receiver (tools/dpu_pipe/dpu_rx, DPUPIPE0 2026-11-03). Repeat --kv to serve several at
  once: each gets the HELLO, each WANT is answered to the address it came from.
  --dpu: also answer WANT_DPU (newest frame, letterboxed + int8-quantised on the Nano by host/dpu_prep.py,
  byte-identical to dev3's evaluator). Without it a WANT_DPU gets NONE(error).
  --lazy (10-26, the competition stack, car_inner.sh AR_COLOUR=1): subscribe to the colour topic only while the KV
  asks (first WANT -> subscribe; no WANT for --idle-unsubscribe-s -> unsubscribe). With the KV AR off nothing ever
  asks, so the process only sends its HELLO (no 921 KB colour messages deserialised, no error output).
  --camera-fps (10-29, car_inner.sh passes CAMERA_FPS when it is not 30): sent in the HELLO; the KV then pairs with at
  least half that camera period plus 5 ms (host/kv_colour.py pair_tol_ns; 15 fps: 39 ms instead of kv_start.sh's 17 ms).
"""
import argparse
import json
import os
import signal
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.kv_colour import ColourRing, ColourSource  # noqa: E402


def image_to_rgb(msg):
    """(width, height, RGB888 bytes) of a sensor_msgs/Image, or a reason string."""
    w, h, enc, step = int(msg.width), int(msg.height), str(msg.encoding).lower(), int(msg.step)
    data = bytes(msg.data)
    if enc not in ("rgb8", "bgr8"):
        return "encoding_%s" % enc
    if step < 3 * w or len(data) < step * h:
        return "short_data"
    if step != 3 * w:
        data = b"".join(data[r * step: r * step + 3 * w] for r in range(h))
    if enc == "bgr8":
        import numpy as np
        data = np.frombuffer(data, np.uint8).reshape(h, w, 3)[..., ::-1].tobytes()
    return w, h, data


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kv", required=True, action="append", help="receiver host:port (repeatable)")
    ap.add_argument("--dpu", action="store_true", help="serve WANT_DPU (DPU input bytes) too")
    ap.add_argument("--dpu-backend", default="auto", choices=("auto", "cv2", "numpy"))
    ap.add_argument("--dpu-warm", default="640:6,512:7,416:7",
                    help="size:fix_point list to prepare at start (cv2 self-test, tables); '' = none")
    ap.add_argument("--topic", default="/camera/color/image_raw")
    ap.add_argument("--ring", type=int, default=12)
    ap.add_argument("--stats-out", default=None)
    ap.add_argument("--lazy", action="store_true", help="subscribe only while the KV sends WANTs")
    ap.add_argument("--idle-unsubscribe-s", type=float, default=10.0)
    ap.add_argument("--camera-fps", type=float, default=None, help="the camera's configured rate, sent in the HELLO")
    a = ap.parse_args()
    kvs = [(h, int(p)) for h, p in (k.rsplit(":", 1) for k in a.kv)]
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    ring = ColourRing(a.ring)
    info = {"source": "live", "topic": a.topic}
    if a.camera_fps is not None:
        info["camera_fps"] = a.camera_fps
    src = ColourSource(kvs, ring.lookup, info=info, latest=ring.newest if a.dpu else None,
                       dpu_backend=a.dpu_backend)
    if a.dpu:
        for item in filter(None, a.dpu_warm.split(",")):
            size, fp = item.split(":")
            be = src.warm_dpu(int(size), int(fp))
            print(json.dumps({"nano_colour": "dpu_warm", "size": int(size), "fix_point": int(fp), "backend": be}),
                  flush=True)
    rclpy.init()
    node = rclpy.create_node("fpga_nano_colour")

    def on_image(msg):
        got = image_to_rgb(msg)
        if isinstance(got, str):
            ring.counts["skip:%s" % got] += 1
            return
        st = msg.header.stamp
        ring.put(st.sec * 1000000000 + st.nanosec, *got)
        src.info.update(width=got[0], height=got[1])
    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
    sub = [None if a.lazy else node.create_subscription(Image, a.topic, on_image, qos)]
    src.start()
    print(json.dumps({"nano_colour": "ready", "kv": a.kv, "topic": a.topic, "lazy": a.lazy,
                      "camera_fps": a.camera_fps}), flush=True)
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    t0 = time.monotonic()
    seen = [0, time.monotonic()]       # WANT count, when it last grew

    def lazy_step():
        n, now = src.counts["wants"] + src.counts["dpu_wants"], time.monotonic()
        if n != seen[0]:
            seen[0], seen[1] = n, now
            if sub[0] is None:
                sub[0] = node.create_subscription(Image, a.topic, on_image, qos)
                ring.counts["subscribe"] += 1
                print(json.dumps({"nano_colour": "subscribed", "wants": n, "t_s": round(now - t0, 1)}), flush=True)
        elif sub[0] is not None and now - seen[1] > a.idle_unsubscribe_s:
            node.destroy_subscription(sub[0])
            sub[0] = None
            ring.counts["unsubscribe"] += 1
            print(json.dumps({"nano_colour": "unsubscribed (KV stopped asking)", "t_s": round(now - t0, 1)}), flush=True)
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.2)
            if a.lazy:
                lazy_step()
    except KeyboardInterrupt:
        pass
    finally:
        src.stop()
        out = {"source": src.describe(), "ring": dict(ring.counts), "run_s": round(time.monotonic() - t0, 1)}
        print(json.dumps(out), flush=True)
        if a.stats_out:
            json.dump(out, open(a.stats_out, "w"), indent=1)
        node.destroy_node()
        rclpy.try_shutdown() if hasattr(rclpy, "try_shutdown") else rclpy.shutdown()


if __name__ == "__main__":
    main()
