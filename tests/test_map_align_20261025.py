"""Map alignment at the mapping stop (2026-10-25, owner plan A; host/map_align.py).

Known answer: a synthetic graph whose per-node correction C_i (translation + yaw, C_first = identity) is chosen here;
opt_i = C_i x odom_i goes to a format-11 poses file, odom_i to an rtabmap-layout Node table (12 float32 3x4 row-major
blobs). Checked: map -> odom = C_last, the car's map pose = opt_last, every object lands on C_k x its odometry-frame
position (k = the node nearest its observation), MappingCore publishes it. Failure paths: no poses file, no database,
no Node table, no common node, root check -> AlignError / align ok false with a reason and nothing to publish.
Identity control: bench_pose's transforms and the memory's saved document are unchanged byte for byte.
Not executed here: ROS (bench_pose re-send, mapping_ctl's /tf_static wait, the runner's subscription) and rtabmap.
"""
import json
import math
import os
import sqlite3
import struct
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tools', 'car_run'))

import bench_pose  # noqa: E402
import mapping_ctl  # noqa: E402
from host import map_align, memory_store  # noqa: E402
from perception.fpga_replay.core import Memory, Track  # noqa: E402

T0 = 1761000000.0
N = 11


def planar(x, y, yaw):
    m = np.eye(4)
    m[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    m[:2, 3] = (x, y)
    return m


def correction(k):
    """The known per-node correction: grows from identity (root) to (0.30, -0.15) m, 6 deg at the last node."""
    f = k / (N - 1.0)
    return planar(0.30 * f, -0.15 * f, math.radians(6.0) * f)


def odom(k):
    return planar(0.25 * k, 0.05 * k * k / N, 0.08 * k)


def write_graph(d, corr=correction, ids=None, node_table=True):
    ids = list(range(1, N + 1)) if ids is None else ids
    poses = os.path.join(d, 'export', 'run_poses.txt')
    os.makedirs(os.path.dirname(poses), exist_ok=True)
    with open(poses, 'w') as f:
        f.write('#timestamp x y z qx qy qz qw id\n')
        for k, nid in enumerate(ids):
            p = map_align.matrix_pose7(corr(k) @ odom(k))
            f.write('%.6f %s %d\n' % (T0 + k, ' '.join('%.9f' % v for v in p), nid))
    db = os.path.join(d, 'run.db')
    con = sqlite3.connect(db)
    if node_table:
        con.execute('CREATE TABLE Node (id INTEGER PRIMARY KEY, map_id INTEGER, weight INTEGER, stamp FLOAT, '
                    'pose BLOB)')
        for k in range(N):
            con.execute('INSERT INTO Node VALUES (?, 0, 0, ?, ?)',
                        (k + 1, T0 + k, struct.pack('<12f', *odom(k)[:3, :].reshape(-1))))
    else:
        con.execute('CREATE TABLE Other (id INTEGER)')
    con.commit()
    con.close()
    return poses, db


def memory_with_objects():
    """Two objects written by the runner in the odometry frame (identity map -> odom) at nodes 3 and 9, and one loaded
    from an earlier run (last_seen 0)."""
    m = Memory()
    for key, k, p in (('obj-1', 3, (1.0, 0.5, 0.2)), ('obj-2', 9, (2.5, -1.0, 0.4))):
        p = np.array(p)
        t = Track(key, 'cup', 'red', p.copy(), p - 0.05, p + 0.05, T0 + k + 0.1)
        t.history = [{'frame_id': 'f%d' % k, 'timestamp': T0 + k + 0.1, 'position_m': p.tolist()}]
        t.negative_poses = [(T0 + k + 0.1, odom(k))]
        m.tracks[key] = t
    old = Track('obj-0', 'book', 'blue', np.array([5.0, 5.0, 0.0]), np.array([4.9, 4.9, 0.0]),
                np.array([5.1, 5.1, 0.1]), 0.0)
    m.tracks['obj-0'] = old
    m._odometer_pose = odom(N - 1)
    return m


class TestKnownAnswer(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp()
        self.poses, self.db = write_graph(self.d)
        self.a = map_align.compute(self.poses, self.db)

    def test_map_odom_is_the_last_node_correction(self):
        self.assertEqual(self.a['node_id'], N)
        self.assertEqual(self.a['count'], N)
        got = map_align.pose7_matrix(*self.a['map_odom'])
        np.testing.assert_allclose(got, correction(N - 1), atol=2e-6)
        self.assertLess(self.a['root']['offset_m'], 1e-5)
        self.assertFalse(map_align.is_identity(self.a))

    def test_car_lands_on_the_optimized_pose(self):
        plan = bench_pose.plan(0, 0, 0.05, odom_base=False)
        static, _dyn = bench_pose.transforms(plan, map_odom=map_align.pose7_matrix(*self.a['map_odom']))
        self.assertEqual([(p, c) for p, c, _ in static], [('map', 'odom')])
        map_base = static[0][2] @ odom(N - 1)                  # TF chain map -> odom -> base_link
        np.testing.assert_allclose(map_base, correction(N - 1) @ odom(N - 1), atol=2e-6)

    def test_objects_land_on_their_own_nodes_correction(self):
        m = memory_with_objects()
        before = {k: t.position.copy() for k, t in m.tracks.items()}
        info = map_align.apply_memory(m, self.a)
        self.assertEqual((info['applied'], info['moved'], info['skipped_not_seen_this_run']), (True, 2, 1))
        self.assertEqual(info['out_of_span_lookups'], 0)
        for key, k in (('obj-1', 3), ('obj-2', 9)):
            t = m.tracks[key]
            c = correction(k)
            want = c[:3, :3] @ before[key] + c[:3, 3]
            np.testing.assert_allclose(t.position, want, atol=2e-6)
            np.testing.assert_allclose((t.lower + t.upper) / 2, want, atol=2e-6)
            self.assertTrue(np.all(t.upper - t.lower >= 0.1 - 1e-9))     # the enclosing box never shrinks
            np.testing.assert_allclose(t.history[0]['position_m'], want, atol=2e-6)
            np.testing.assert_allclose(t.negative_poses[0][1], c @ odom(k), atol=2e-6)
        # obj-1 (node 3) and obj-2 (node 9) got different corrections: per node, not one T
        self.assertGreater(np.linalg.norm(correction(9)[:3, 3] - correction(3)[:3, 3]), 0.1)
        np.testing.assert_array_equal(m.tracks['obj-0'].position, before['obj-0'])   # earlier run: untouched
        np.testing.assert_allclose(m._odometer_pose, correction(N - 1) @ odom(N - 1), atol=2e-6)
        doc = memory_store.dump(m, complete=True, runtime_boot_id='b', world_frame='map', pose_source='reference',
                                sensor_source='live', map_identity_doc=memory_store.map_identity())
        json.dumps(doc, allow_nan=False)                     # the memory file still serialises

    def test_stamp_outside_the_graph_gets_the_last_correction(self):
        c = map_align.Corrections(self.a)
        np.testing.assert_allclose(c.at(T0 + 99999.0), correction(N - 1), atol=2e-6)
        np.testing.assert_allclose(c.at(T0 + 4.4), correction(4), atol=2e-6)
        np.testing.assert_allclose(c.at(T0 + 4.6), correction(5), atol=2e-6)
        self.assertEqual(c.out_of_span, 1)

    def test_mapping_core_publishes_it(self):
        out = self.d
        with open(os.path.join(out, 'export', 'run.yaml'), 'w') as f:
            f.write('image: run.pgm\n')
        core = mapping_ctl.MappingCore(out, out, db=self.db)
        core.request_stop()
        core.on_child_exit(0)
        self.assertEqual(core.state, 'exported')
        msg = core.align_message()
        self.assertIsNotNone(msg)
        self.assertEqual(msg['map_odom'], self.a['map_odom'])
        self.assertEqual(len(msg['nodes']), N)
        m, key = bench_pose.align_matrix(json.dumps(msg))
        self.assertEqual(key, core.yaml)
        np.testing.assert_allclose(m, correction(N - 1), atol=2e-6)
        s = core.status()['align']
        self.assertTrue(s['ok'])
        self.assertIsNone(s['applied'])
        core.on_align_applied(True)
        self.assertTrue(core.status()['align']['applied'])
        json.dumps(core.status())


class TestNoCorrection(unittest.TestCase):
    def assert_refused(self, poses, db, words):
        with self.assertRaises(map_align.AlignError) as cm:
            map_align.compute(poses, db)
        self.assertIn(words, str(cm.exception))

    def test_failure_paths_name_the_reason(self):
        d = tempfile.mkdtemp()
        poses, db = write_graph(d)
        self.assert_refused(os.path.join(d, 'none.txt'), db, 'no exported poses file')
        self.assert_refused(poses, os.path.join(d, 'none.db'), 'missing or empty')
        d2 = tempfile.mkdtemp()
        p2, db2 = write_graph(d2, node_table=False)
        self.assert_refused(p2, db2, 'Node table not readable')
        d3 = tempfile.mkdtemp()
        p3, db3 = write_graph(d3, ids=list(range(101, 101 + N)))
        self.assert_refused(p3, db3, 'no node id in both')
        d4 = tempfile.mkdtemp()                              # root not identity: optimized poses in another frame
        p4, db4 = write_graph(d4, corr=lambda k: planar(1.0, 0.0, 0.0) @ correction(k))
        self.assert_refused(p4, db4, 'root check failed')

    def test_mapping_core_keeps_identity_and_says_why(self):
        for standin, make in ((False, lambda d: None), (True, lambda d: write_graph(d))):
            d = tempfile.mkdtemp()
            os.makedirs(os.path.join(d, 'export'), exist_ok=True)
            make(d)
            with open(os.path.join(d, 'export', 'run.yaml'), 'w') as f:
                f.write('image: run.pgm\n')
            core = mapping_ctl.MappingCore(d, d, standin=standin, db=os.path.join(d, 'run.db'))
            core.request_stop()
            core.on_child_exit(0)
            self.assertEqual(core.state, 'exported')              # the export stands
            self.assertIsNone(core.align_message())               # nothing published: map -> odom stays identity
            s = core.status()['align']
            self.assertFalse(s['ok'])
            self.assertIn('stand-in' if standin else 'poses file', s['reason'])

    def test_refused_messages(self):
        for bad in ('{}', '{"map_odom": [0, 0, 0, 0, 0, 0, 2]}', '{"map_odom": [0, 0, 0, 1]}', 'nope'):
            with self.assertRaises((ValueError, KeyError)):
                bench_pose.align_matrix(bad)


class TestFusedSamplesMove(unittest.TestCase):
    """Identity v2 (D-050) recomputes a track's position from its last 20 samples at every sighting. The samples must
    move with the alignment, or the next sighting pulls the track back into the odometry frame (10-27 defect)."""

    def setUp(self):
        d = tempfile.mkdtemp()
        self.a = map_align.compute(*write_graph(d))

    def track(self):
        m = Memory()
        p = np.array([2.0, 0.5, 0.11])
        t = Track('object-0001', 'bottle', 'red', p.copy(), p - .03, p + .03, T0 + 9.1)
        t.samples = [(T0 + 9.1 + 0.01 * j, p + 0.005 * np.array([math.sin(j), math.cos(j), 0.0]), 0.06)
                     for j in range(20)]
        m._fuse(t)
        m.tracks[t.id] = t
        return m, t

    def test_next_fusion_stays_in_the_map_frame(self):
        m, t = self.track()
        c = correction(9)
        want = c[:3, :3] @ t.position + c[:3, 3]
        self.assertGreater(np.linalg.norm(want - t.position), 0.2)            # the correction is large here
        map_align.apply_memory(m, self.a)
        np.testing.assert_allclose(t.position, want, atol=1e-6)
        m._fuse(t)                                                            # what the next sighting does
        np.testing.assert_allclose(t.position, want, atol=1e-6)
        np.testing.assert_allclose(np.median([s[1] for s in t.samples], axis=0), want, atol=0.01)

    def test_control_unmoved_samples_pull_the_track_back(self):
        m, t = self.track()
        before, samples = t.position.copy(), list(t.samples)
        map_align.apply_memory(m, self.a)
        t.samples = samples                                                   # the 10-27 deployed behaviour
        m._fuse(t)
        np.testing.assert_allclose(t.position, before, atol=1e-6)


class TestIdentityControl(unittest.TestCase):
    EYE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

    def test_bench_pose_transforms_unchanged(self):
        for args in ((0, 8.0, 0.05, True), (0, 0, 0.05, False), (10.0, 8.0, 0.05, True)):
            p = bench_pose.plan(*args[:3], odom_base=args[3])
            for m in (None, np.eye(4), map_align.pose7_matrix(*self.EYE)):
                s, dyn = bench_pose.transforms(p, map_odom=m)
                s0, d0 = bench_pose.transforms(p)
                self.assertEqual([(a, b, t.tobytes()) for a, b, t in s], [(a, b, t.tobytes()) for a, b, t in s0])
                self.assertEqual([(a, b, t.tobytes()) for a, b, t in dyn], [(a, b, t.tobytes()) for a, b, t in d0])
                self.assertEqual(s0[0][2].tobytes(), np.eye(4).tobytes())

    def test_identity_alignment_leaves_memory_byte_identical(self):
        align = {'map_odom': list(self.EYE), 'node_id': 3,
                 'nodes': [[T0 + k] + list(self.EYE) for k in range(3)]}
        self.assertTrue(map_align.is_identity(align))

        def dump(m):
            return json.dumps(memory_store.dump(
                m, complete=True, runtime_boot_id='b', world_frame='map', pose_source='reference',
                sensor_source='live', map_identity_doc=memory_store.map_identity(),
                now=__import__('datetime').datetime(2026, 10, 25, tzinfo=__import__('datetime').timezone.utc)),
                sort_keys=True).encode()
        m = memory_with_objects()
        before, odo_before = dump(m), m._odometer_pose.tobytes()
        neg_before = [p.tobytes() for t in m.tracks.values() for _s, p in t.negative_poses]
        self.assertEqual(map_align.apply_memory(m, align), {'applied': False, 'identity': True})
        self.assertEqual(dump(m), before)
        self.assertEqual(m._odometer_pose.tobytes(), odo_before)
        self.assertEqual([p.tobytes() for t in m.tracks.values() for _s, p in t.negative_poses], neg_before)
        core = mapping_ctl.MappingCore('/x', '/x')
        core.align = dict(align, ok=True, identity=True, count=3, root={})
        self.assertIsNone(core.align_message())                   # identity: nothing published, today's behaviour


class TestRealDatabaseLayout(unittest.TestCase):
    """The Node.pose blob layout on a real rtabmap database (host artifact, skipped when absent)."""
    DB = '${FPGA_SHARED}/artifacts/20260910-dev1-rtabmap-offline/normal-result/rtabmap.db'
    ODOM = '${FPGA_SHARED}/artifacts/20260910-dev1-rtabmap-offline/export/odometry_camera_poses.txt'

    @unittest.skipUnless(os.path.isfile(DB) and os.path.isfile(ODOM), 'host rtabmap artifact not present')
    def test_blob_translation_equals_exported_odometry(self):
        nodes = map_align.read_db_odom(self.DB)
        by_stamp = {round(s, 5): m for s, m in nodes.values()}
        with open(self.ODOM) as f:
            rows = [list(map(float, line.split())) for line in f if not line.startswith('#')]
        self.assertGreater(len(rows), 10)
        for r in rows:
            m = by_stamp[round(r[0], 5)]
            np.testing.assert_allclose(m[:3, 3], r[1:4], atol=1e-5)


if __name__ == '__main__':
    unittest.main()
