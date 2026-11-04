#!/usr/bin/env python3
"""Exercise the installed message_filters with delayed same-frame arrivals."""
import json
from types import SimpleNamespace
from builtin_interfaces.msg import Time
from message_filters import ApproximateTimeSynchronizer, SimpleFilter


def run(size):
    inputs = [SimpleFilter() for _ in range(3)]
    sync = ApproximateTimeSynchronizer(inputs, size, .02)
    output = []
    sync.registerCallback(lambda *rows: output.append([row.header.stamp.nanosec for row in rows]))
    # RGB arrives four frames ahead of depth/info; first frame is still only
    # 0.3 seconds old, inside the unchanged capture/receipt acceptance limits.
    for channel in range(3):
        for stamp in (0, 100000000, 200000000, 300000000):
            inputs[channel].signalMessage(SimpleNamespace(header=SimpleNamespace(stamp=Time(sec=1, nanosec=stamp))))
    assert all(len(set(row)) == 1 for row in output)
    return output


old, new = run(3), run(10)
assert [0, 0, 0] not in old
assert [0, 0, 0] in new and len(new) == 4
print(json.dumps({'queue_3': old, 'queue_10': new, 'passed': True,
                  'scope': 'installed ROS synchronizer synthetic arrival order; not device acceptance'}))
