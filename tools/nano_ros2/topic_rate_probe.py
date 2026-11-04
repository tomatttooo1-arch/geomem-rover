#!/usr/bin/env python3
"""Subscribe to camera topics for --seconds and report message rate, header-stamp spacing and format.

Rates come from receive times (host monotonic) and from header stamps; both are reported.
Uses sensor-data QoS (best effort), which matches both reliable and best-effort publishers.
"""
import argparse
import json
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds', type=float, default=30.)
    p.add_argument('--warmup', type=float, default=10.)
    p.add_argument('--out', required=True)
    p.add_argument('topics', nargs='+', help='TOPIC:TYPE with TYPE image|points|imu|info')
    a = p.parse_args()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, PointCloud2, Imu, CameraInfo
    types = {'image': Image, 'points': PointCloud2, 'imu': Imu, 'info': CameraInfo}
    rclpy.init(args=[])
    node = rclpy.create_node('topic_rate_probe')
    rec = {}
    t_start = [None]

    def cb(topic, m):
        if t_start[0] is None or time.monotonic() < t_start[0]:
            return
        r = rec.setdefault(topic, {'recv': [], 'stamp': [], 'meta': None})
        r['recv'].append(time.monotonic())
        r['stamp'].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        if r['meta'] is None:
            meta = {'frame_id': m.header.frame_id}
            if hasattr(m, 'encoding'):
                meta.update(width=m.width, height=m.height, encoding=m.encoding)
            if hasattr(m, 'point_step'):
                meta.update(width=m.width, height=m.height, point_step=m.point_step)
            r['meta'] = meta
    for spec in a.topics:
        topic, kind = spec.rsplit(':', 1)
        node.create_subscription(types[kind], topic, lambda m, t=topic: cb(t, m), qos_profile_sensor_data)
    t_start[0] = time.monotonic() + a.warmup
    end = t_start[0] + a.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    out = {'seconds': a.seconds, 'warmup': a.warmup, 'topics': {}}
    for spec in a.topics:
        topic = spec.rsplit(':', 1)[0]
        r = rec.get(topic)
        if not r or len(r['recv']) < 2:
            out['topics'][topic] = {'count': 0 if not r else len(r['recv'])}
            continue
        gaps = sorted(b - a_ for a_, b in zip(r['stamp'], r['stamp'][1:]))
        out['topics'][topic] = {
            'count': len(r['recv']),
            'rate_hz_recv': (len(r['recv']) - 1) / (r['recv'][-1] - r['recv'][0]),
            'rate_hz_stamp': (len(r['stamp']) - 1) / (r['stamp'][-1] - r['stamp'][0]),
            'stamp_gap_ms_median': gaps[len(gaps) // 2] * 1e3, 'stamp_gap_ms_max': gaps[-1] * 1e3,
            'stamp_gap_over_1.5x_median': sum(g > 1.5 * gaps[len(gaps) // 2] for g in gaps),
            'meta': r['meta']}
    json.dump(out, open(a.out, 'x'), indent=1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
