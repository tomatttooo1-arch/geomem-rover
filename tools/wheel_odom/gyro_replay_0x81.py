#!/usr/bin/env python3
"""Offline smoke of the v0.2 gyro odometry on a v0.1 recording (2026-10-21).

Turns every 0x81 rx row of a host/ros_chassis_driver.py --record file into a pseudo-0x82 wheel_counts body
and feeds OdomCore(heading='gyro'): enc_a = enc_c = left_count, enc_b = enc_d = right_count (only two wheels
were reported), gz_sum = 2 * round(gyro_z_cdps * 0.01 / 0.061035), gz_n = 2 (0 when gyro_valid is false),
az = 1 g, ax = ay = 0, imu_err = 0, mcu_ms unwrapped from u16, cmd = the last tx row before it.
Caveats (why this is a smoke, not a check): the 0x81 gyro is the MCU's processed value (bias subtracted,
dead band, static zeroing), so the Nano bias estimate comes out ~0 and slow turns are invisible; there are
no torn-read or accel effects; two counts stand for four wheels (lateral and redundancy are 0 by construction).

  gyro_replay_0x81.py RECORD.jsonl [--wheel-diam-m 0.081 --counts-per-rev 1560 --track-m 0.394] [--out X.json]
Prints the net yaw of the gyro and the wheel heading, their difference, the fallback seconds and the five
largest 1 s jumps of (gyro - wheel).
"""
import argparse
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from host.chassis_uart import GYRO_RAW_DEG_S  # noqa: E402
from host.gyro_odom import FusedParams  # noqa: E402
from host.ros_wheel_odom import OdomCore  # noqa: E402
from host.wheel_odom import WheelParams  # noqa: E402


def pseudo_bodies(path):
    cmd = None
    ms_prev = None
    ms_abs = 0
    with open(path) as f:
        for line in f:
            row = json.loads(line)
            if row.get('dir') == 'tx':
                cmd = [row.get('v_mm_s', 0), row.get('w_mrad_s', 0), bool(row.get('enable'))]
                continue
            if row.get('dir') != 'rx' or 'left_count' not in row:
                continue
            ms = row['mcu_ms']
            if ms_prev is None or row.get('after_silence'):
                ms_abs += 10 if ms_prev is not None else 0
            else:
                ms_abs += (ms - ms_prev) % 65536
            ms_prev = ms
            n = 2 if row.get('gyro_valid', True) else 0
            read = int(round(row['gyro_z_cdps'] * 0.01 / GYRO_RAW_DEG_S))
            yield {'stamp_ns': row['t_ns'], 't_ns': row['t_ns'], 'seq': row['seq'], 'mcu_ms': ms_abs,
                   'left_count': row['left_count'], 'right_count': row['right_count'],
                   'after_silence': bool(row.get('after_silence')), 'lost_before': row.get('lost_before', 0),
                   'motor_enabled': row.get('motor_enabled'), 'frame': '0x82',
                   'enc_a': row['left_count'], 'enc_b': row['right_count'], 'enc_c': row['left_count'],
                   'enc_d': row['right_count'], 'gz_sum': read * n, 'gz_n': n, 'imu_err': 0, 'gx_mean': 0,
                   'gy_mean': 0, 'ax': 0, 'ay': 0, 'az': 2048, 'cmd': cmd}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('record')
    ap.add_argument('--wheel-diam-m', type=float, default=0.081)
    ap.add_argument('--counts-per-rev', type=int, default=1560)
    ap.add_argument('--track-m', type=float, default=0.394)
    ap.add_argument('--out')
    a = ap.parse_args(argv)
    core = OdomCore('chassis', WheelParams(a.wheel_diam_m, a.counts_per_rev, a.track_m), heading='gyro',
                    fused_params=FusedParams(wheel_diam_m=a.wheel_diam_m, counts_per_rev=a.counts_per_rev))
    t0 = None
    series = []                     # (t_s, gyro yaw, wheel yaw) every 100th frame
    k = 0
    gyro = wheel = None
    for body in pseudo_bodies(a.record):
        st = core.feed_chassis(body)
        if t0 is None:
            t0 = body['t_ns']
        if st is not None:
            gyro = st['yaw']
        if core.last_wheel_state is not None:
            wheel = core.last_wheel_state['yaw']
        k += 1
        if k % 100 == 0 and gyro is not None and wheel is not None:
            series.append(((body['t_ns'] - t0) / 1e9, gyro, wheel))
    jumps = []
    for i in range(1, len(series)):
        d = (series[i][1] - series[i][2]) - (series[i - 1][1] - series[i - 1][2])
        jumps.append((abs(math.degrees(d)), round(series[i][0], 1), round(math.degrees(d), 1)))
    jumps.sort(reverse=True)
    st = core.status()
    out = {'frames': k, 'duration_s': round(series[-1][0], 1) if series else 0,
           'gyro_net_yaw_deg': round(math.degrees(gyro), 2), 'wheel_net_yaw_deg': round(math.degrees(wheel), 2),
           'gyro_minus_wheel_deg': round(math.degrees(gyro - wheel), 2),
           'fallback_s': st['gyro']['fallback_by_reason_s'], 'gyro_s': st['gyro']['gyro_s'],
           'bias_dps': st['gyro']['bias']['bias_dps'], 'static_total_s': st['gyro']['bias']['static_total_s'],
           'spikes_replaced': st['gyro']['spikes_replaced'], 'slip': st['gyro']['slip'],
           'largest_1s_jumps_deg': [{'t_s': t, 'gyro_minus_wheel_deg': d} for _, t, d in jumps[:5]]}
    print(json.dumps(out, indent=1))
    if a.out:
        Path(a.out).write_text(json.dumps(out, indent=1) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
