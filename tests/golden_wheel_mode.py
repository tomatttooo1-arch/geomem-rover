"""Wheel-mode golden outputs of the chassis -> wheel odometry chain (v0.2 Nano side, 2026-10-21).

The --heading wheel mode of host/ros_wheel_odom.py must stay bit-identical to the code before the v0.2
Nano-side change. This module builds one deterministic 0x81 input stream (straight, rotate in place,
arc, i32 wrap, a count jump, serial batches sharing one receive time, a duplicate frame, an MCU silence
with counter reset, garbage bytes, a lost frame) and runs it through the whole ROS-free chain:
  encode_status -> FrameParser -> ChassisLink.on_status -> wheel_counts_row -> JSON -> OdomCore.feed_chassis
  -> odometry_fields (covariances collected as a set), plus the --record rows, OdomCore.status() and ChassisLink.status().
The fixture tests/data/wheel_mode_golden_de10531.json was written by `python3 tests/golden_wheel_mode.py
--write` on the UNMODIFIED base (feat/dev1/imu-v02-20261021 at de10531); tests/test_gyro_odom.py re-runs
the chain on the current code and compares with ==, floats included (JSON floats round-trip exactly).
"""
import io
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from host.chassis_uart import ChassisLink, FrameParser, TYPE_STATUS, encode_status  # noqa: E402
from host.ros_chassis_driver import wheel_counts_row  # noqa: E402
from host.ros_wheel_odom import OdomCore, odometry_fields  # noqa: E402
from host.wheel_odom import WheelParams  # noqa: E402

FIXTURE = ROOT / 'tests/data/wheel_mode_golden_de10531.json'
MS = 1000000
PARAM_SETS = {'placeholder': {}, 'realcar_1016': {'wheel_diam_m': 0.081, 'counts_per_rev': 1560,
                                                   'track_m': 0.394}}


def input_batches():
    """[(receive_ns, bytes)]: the byte chunks the serial thread would read, with their receive time."""
    batches = []
    left = right = 0
    seq = 0
    ms = 0
    pending = []

    def frame(status_bits=0x3F):
        nonlocal seq, ms
        f = encode_status(seq, ms, left, right, int(round(1234 * math.sin(ms / 700.))), 0, 7400,
                          (seq * 2) & 0xFF, status_bits)
        seq = (seq + 1) & 0xFF
        ms = (ms + 10) & 0xFFFF
        return f

    def flush(t_ms, extra=b''):
        batches.append((t_ms * MS + 123457, b''.join(pending) + extra))
        del pending[:]

    t = 0
    for k in range(60):                       # straight, one frame per read
        left += 7; right += 7
        t += 10
        pending.append(frame()); flush(t)
    for k in range(80):                       # rotate in place, 3-frame serial batches (one receive time)
        left -= 5; right += 5
        t += 10
        pending.append(frame())
        if k % 3 == 2:
            flush(t)
    flush(t)
    for k in range(120):                      # arc
        left += 4; right += 9
        t += 10
        pending.append(frame()); flush(t)
    left, right = 2**31 - 30, 2**31 - 30      # big step = jump (rebaseline by the jump rule)
    t += 10
    pending.append(frame()); flush(t)
    for k in range(20):                       # i32 wrap crossing
        left = ((left + 2**31 + 6) % 2**32) - 2**31
        right = ((right + 2**31 + 3) % 2**32) - 2**31
        t += 10
        pending.append(frame()); flush(t, b'\x00\xa5' if k == 5 else b'')
    dup = frame()
    t += 10
    batches.append((t * MS + 123457, dup))    # the same frame twice (same seq, mcu_ms): skipped by key
    batches.append((t * MS + 123457, dup))
    seq = (seq + 1) & 0xFF                    # one lost frame
    ms = (ms + 10) & 0xFFFF
    for k in range(30):
        left += 3; right -= 2
        t += 10
        pending.append(frame()); flush(t)
    t += 500                                  # MCU silence 500 ms with a reset: counts restart near 0
    left, right, seq, ms = 11, 13, 0, 0
    for k in range(40):
        left += 6; right += 8
        t += 10
        pending.append(frame(0x3B if k < 20 else 0x3F)); flush(t)
    return batches


def run_chain(params_kw):
    """Everything the wheel-mode chain produces for input_batches(), as plain JSON-able data."""
    link, parser = ChassisLink(), FrameParser(accept={TYPE_STATUS})
    record = io.StringIO()
    core = OdomCore('chassis', WheelParams(**params_kw), record=record)
    rows, states, fields = [], [], []
    covariances = set()
    last_ns = 0
    for received, data in input_batches():
        ros_now = received + 1792000000 * 10**9
        for row in parser.feed(data):
            derived = link.on_status(row, received)
            body = wheel_counts_row(row, derived, ros_now - (received - derived['stamp_ns']))
            body = json.loads(json.dumps(body))
            rows.append(body)
            st = core.feed_chassis(body)
            if st is not None:
                states.append(st)
                f = odometry_fields(st)
                covariances.add(json.dumps([f.pop('pose_covariance'), f.pop('twist_covariance')]))
                fields.append(f)
        last_ns = received
    return {'counts_rows': rows, 'states': states, 'odometry_fields': fields,
            'covariances': sorted(json.loads(c) for c in covariances),
            'record': [json.loads(x) for x in record.getvalue().splitlines()],
            'odom_status': core.status(), 'link_status': link.status(last_ns),
            'parser': dict(parser.counters)}


def normalise(obj):
    """JSON round trip (tuples -> lists), so the comparison is on what a topic or file would carry."""
    return json.loads(json.dumps(obj, allow_nan=False))


def golden():
    return normalise({name: run_chain(kw) for name, kw in sorted(PARAM_SETS.items())})


if __name__ == '__main__':
    if '--write' in sys.argv:
        FIXTURE.write_text(json.dumps(golden(), sort_keys=True, separators=(',', ':')) + '\n')
        print('wrote', FIXTURE, FIXTURE.stat().st_size, 'bytes')
    else:
        print(json.dumps({k: len(v['states']) for k, v in golden().items()}))
