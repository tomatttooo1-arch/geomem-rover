"""Opt-in bounded scalar snapshots; no hot-path serialization or file IO."""
import json
import math
from pathlib import Path
import threading
import time


def scalar_copy(value):
    if value is None or type(value) in (str, bool, int):
        return value
    if type(value) is float:
        return value if math.isfinite(value) else {'nonfinite': str(value)}
    if type(value) in (list, tuple):
        return [scalar_copy(v) for v in value]
    if type(value) is dict:
        return {str(k): scalar_copy(v) for k, v in value.items()}
    raise TypeError('diagnostics require scalar snapshots, got ' + type(value).__name__)


def status_snapshot(row):
    if not isinstance(row, dict):
        return {'payload_type': type(row).__name__, 'status': 'not_object'}
    fields = {k: row.get(k) if type(row.get(k)) in (str, int, float, bool, type(None))
              else {'invalid_type': type(row.get(k)).__name__}
              for k in ('instance', 'epoch', 'sequence', 'state', 'stamp_ns', 'clock_ns')}
    approved = row.get('approved')
    fields['approved_stamps'] = ([item.get('stamp_ns') if isinstance(item, dict)
        and type(item.get('stamp_ns')) is int else {'invalid_entry_type': type(item).__name__}
        for item in approved[:32]] if isinstance(approved, list) else {'invalid_type': type(approved).__name__})
    fields['approved_count'] = len(approved) if isinstance(approved, list) else None
    return fields


def lease_snapshot(lease):
    if lease is None:
        return None
    def row_snapshot(row):
        if row is None:
            return None
        return status_snapshot(row)
    return {'row': row_snapshot(lease.row), 'last_status': row_snapshot(lease.last_status),
            'generation': lease.generation, 'received': lease.received,
            'progress': lease.progress, 'last_ros': lease.last_ros}


def queue_snapshot(queue):
    def entry(row):
        return None if row is None else {'stamp': row.stamp, 'received_at': row.received_at,
                                        'deadline': row.received_at + queue.timeout}
    return {'current': entry(queue.current), 'latest': entry(queue.latest), 'timeout': queue.timeout}


class RuntimeDiagnostics:
    def __init__(self, output, capacity=20000):
        if capacity <= 0:
            raise ValueError('positive diagnostics capacity required')
        self.output, self.capacity = Path(output), capacity
        if self.output.exists():
            raise FileExistsError(self.output)
        self.rows = []
        self.total = 0
        self.closed = False
        self.static_edges = set()
        self.map_seen = self.status_seen = self.ack_sent = False

    def record(self, event, **fields):
        self.total += 1
        if len(self.rows) < self.capacity:
            self.rows.append({'sequence': self.total, 'event': event,
                              'monotonic_ns': time.monotonic_ns(), 'thread_id': threading.get_ident(),
                              **scalar_copy(fields)})

    def infrastructure_ready(self, base, tracked, camera):
        return (self.map_seen and self.status_seen and
                {(base, tracked), (tracked, camera)} <= self.static_edges)

    def flush(self):
        if self.closed:
            return
        with self.output.open('x') as stream:
            json.dump({'schema': 1, 'capacity': self.capacity, 'total_events': self.total,
                       'retained_events': len(self.rows), 'dropped_events': self.total-len(self.rows),
                       'clock_semantics': 'event monotonic_ns is observation time; explicit input_wall/input_ros_ns are actual policy arguments',
                       'events': self.rows}, stream, allow_nan=False)
        self.closed = True
