import json
from pathlib import Path
import tempfile
import unittest

from host.runtime_diagnostics import RuntimeDiagnostics, lease_snapshot, queue_snapshot, status_snapshot
from host.ros_runtime import AcquisitionQueue
from host.localization_gate import LocalizationLease, LocalizationGate


class DiagnosticsTests(unittest.TestCase):
    def test_bounded_snapshot_and_initial_nonfinite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'diagnostics.json'
            d = RuntimeDiagnostics(path, capacity=2)
            original = {'values': [1, float('inf'), -float('inf')]}
            d.record('initial', lease=lease_snapshot(LocalizationLease('map')), value=original)
            original['values'][0] = 9
            d.record('next'); d.record('overflow')
            self.assertFalse(path.exists())
            d.flush(); d.flush()
            data = json.loads(path.read_text())
            self.assertEqual((data['retained_events'], data['dropped_events']), (2, 1))
            self.assertEqual(data['events'][0]['value']['values'][0], 1)
            self.assertEqual(data['events'][0]['value']['values'][1], {'nonfinite': 'inf'})
            with self.assertRaises(FileExistsError):
                RuntimeDiagnostics(path)

    def test_recording_has_no_queue_or_lease_policy_effect(self):
        outputs = []
        for enabled in (False, True):
            with tempfile.TemporaryDirectory() as directory:
                d = RuntimeDiagnostics(Path(directory)/'log') if enabled else None
                q = AcquisitionQueue(); lease = LocalizationLease('map')
                gate = LocalizationGate('map', {1}, 'camera_link')
                rows = []
                for i, wall in enumerate((0., .1, .3, .6)):
                    q.push(10.+i, ('image',), wall)
                    status = gate.status(wall, 10**10+i)
                    lease.update(status, wall, 10**10+i)
                    if d:
                        d.record('snapshot', queue=queue_snapshot(q), lease=lease_snapshot(lease))
                    rows.append((q.expired(wall), lease.check(wall, 10**10+i), lease.generation))
                outputs.append(rows)
        self.assertEqual(outputs[0], outputs[1])
        self.assertTrue(outputs[0][-1][0])

    def test_readiness_requires_all_actual_infrastructure_but_no_valid_lease(self):
        with tempfile.TemporaryDirectory() as directory:
            d = RuntimeDiagnostics(Path(directory)/'log')
            d.static_edges.add(('base', 'tracked')); d.map_seen = d.status_seen = True
            self.assertFalse(d.infrastructure_ready('base', 'tracked', 'camera'))
            d.static_edges.add(('tracked', 'camera'))
            self.assertTrue(d.infrastructure_ready('base', 'tracked', 'camera'))

    def test_future_clock_rejection_and_valid_token_are_not_changed_by_snapshot(self):
        gate = LocalizationGate('map', {1}, 'camera_link')
        row = gate.status(1., 10**9)
        row.update(state='valid', stamp_ns=10**9, approved=[{'stamp_ns': 10**9, 'pose': {
            'pose': {'position': {'x': 0., 'y': 0., 'z': 0.},
                     'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}},
            'covariance': [float(i % 7 == 0)*.01 for i in range(36)]}}])
        for ros_ns, expected in ((0, False), (10**9, True)):
            lease = LocalizationLease('map')
            lease.update(row, 1., ros_ns)
            before = lease_snapshot(lease)
            after = lease_snapshot(lease)
            self.assertEqual(before, after)
            self.assertEqual(lease.token(10**9, 1., ros_ns) is not None, expected)

    def test_flush_in_finally_survives_shutdown_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'log'; d = RuntimeDiagnostics(path)
            with self.assertRaises(RuntimeError):
                try:
                    d.record('before_shutdown')
                    raise RuntimeError('ROS context already invalid')
                finally:
                    d.flush()
            self.assertEqual(json.loads(path.read_text())['retained_events'], 1)

    def test_queue_deadline_not_reset_by_snapshot_or_successor(self):
        q = AcquisitionQueue(); q.push(1., None, 3.); q.push(2., None, 3.2)
        before = queue_snapshot(q)
        self.assertEqual(before['current']['deadline'], 3.5)
        q.pop()
        self.assertEqual(queue_snapshot(q)['current']['deadline'], 3.7)
        self.assertEqual(before['current']['stamp'], 1.)

    def test_reject_mutable_sensor_objects(self):
        with tempfile.TemporaryDirectory() as directory:
            d = RuntimeDiagnostics(Path(directory)/'log')
            with self.assertRaises(TypeError):
                d.record('bad', image=object())

    def test_malformed_status_summary_does_not_raise_or_mutate_lease(self):
        for row in ({'approved': [{}]}, [], None, {'approved': [None, [], {'stamp_ns': []}]},
                    {'state': [], 'approved': 'invalid'}):
            with self.subTest(row=row), tempfile.TemporaryDirectory() as directory:
                lease = LocalizationLease('map')
                lease.update(row, 1., 0)
                before = lease_snapshot(lease)
                d = RuntimeDiagnostics(Path(directory)/'log')
                d.record('status_update', input_status=status_snapshot(row), lease=lease_snapshot(lease))
                d.flush()
                self.assertEqual(before, lease_snapshot(lease))


if __name__ == '__main__':
    unittest.main()
