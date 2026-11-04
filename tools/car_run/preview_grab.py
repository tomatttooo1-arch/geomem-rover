#!/usr/bin/env python3
"""Save the stack's own preview JPEGs (/fpga/preview/jpeg) for a fixed time window (10-27, detect_accuracy.py).

Runs INSIDE the car stack container (docker exec; the container's ROS env/DDS settings), read-only on the stack:
it only subscribes. Each file is named by the camera header stamp in ns (= runtime.jsonl frame_id rgb-<ns>), so
the report joins pictures to detections. Written into --out (a new directory under the run directory /o).

    python3 tools/car_run/preview_grab.py --out /o/acc_frames/r01 --seconds 8
"""
import argparse
import os
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", required=True)
    p.add_argument("--seconds", type=float, default=8.0)
    p.add_argument("--topic", default="/fpga/preview/jpeg")
    a = p.parse_args()
    os.makedirs(a.out, exist_ok=True)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    count = [0]

    class Grab(Node):
        def __init__(self):
            super().__init__("geomem_preview_grab_%d" % os.getpid())
            self.create_subscription(CompressedImage, a.topic, self.cb, qos_profile_sensor_data)

        def cb(self, msg):
            ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            with open(os.path.join(a.out, "rgb-%d.jpg" % ns), "wb") as f:
                f.write(bytes(msg.data))
            count[0] += 1

    rclpy.init()
    node = Grab()
    end = time.monotonic() + a.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("saved %d preview frames to %s" % (count[0], a.out), flush=True)


if __name__ == "__main__":
    main()
