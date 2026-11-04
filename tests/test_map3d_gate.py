"""10-30 3D map display gate: map_data_gate.py (CDR prefix parser, GraphGate, MapDataForwarder) and
live_mapping.CloudGate -- pure parts, no ROS. Fixtures: rtabmap_msgs/MapData serialized by rclpy (Humble,
rtabmap_msgs 0.23.7) in fpga-dev1-rtabmap:humble-0.23.7: ONE = ids [3, 4, 9], node data [9]; TWO = ids [3, 4, 9, 12],
node data [12, 4], frame id of another length; EMPTY = no graph, no node. Poses: x = 0.1 k, y = -0.05 k, w = 1.
The equivalence check against the real map_assembler is the bench of runs/map3d-gate-20261030-results (not here)."""
import base64
import importlib.util
import json
import sys
import unittest
import zlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
COMP = ROOT / "ui/geomem/tools/competition"
sys.path.insert(0, str(COMP))
import map_data_gate as mg  # noqa: E402

spec = importlib.util.spec_from_file_location("live_mapping_gate", COMP / "live_mapping.py")
lm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lm)

FIX = {
    "ONE": "eJxjYGRg2OPyJKth0zM+FgYGhtzEAgw+BDywZyAJfLBnBpIgDDKHE8rGAhqINW/WTBDYCaVX7idN30mYfqL1MTEg3E8++GCPg+8AoTigtACUFoHSElBaBkorQGklKK0CpdWgtAaU1oLSOlBaD0obQGlDKG0EpY2htAmUNoXSZlDaHEpbQGlLKG0Fpa2htA2UtoXSdlDaHko7QOgGKM3gCOU7OsDSB9h1rAxkgtFwhtC4wxmY1eHhzMgAA5oOoCDPScqxZMjLSy3SS0xmIAughz8qYMTOL0Ll8wdj1ReK2xxBsCATMwsrGzsHJxc3Dy8fv0AoA06Abh4DMXwgMKJauiQNwNw7gy3ejpb2jAJI+B2nUD8pAADwqFlu",
    "TWO": "eJxjYGRg2OPyJKth0zM+bgYGhtzEgrSixNzU+AoGOxziUPDAnoEk8MGeBUgyAzGI5gRiHigbDTQQa96smSCwE0qv3E+avpMw/UTrMzEGgctQ+jDR+kB+hvmbfPDBHgffAUJxQGkBKC0CpSWgtAyUVoDSSlBaBUqrQWkNKK0FpXWgtB6UNoDShlDaCEobQ2kTKG0Kpc2gtDmUtoDSllDaCkpbQ2kbKG0Lpe2gtD2UdoDQDVCawRHKd3SApSvKwGg4Q2jc4QzLu2DVWglHGMgCo+EMoXGHMxMDIpwZGWBA04ENSOYk5RgaAekZZBcq6OGPCsD21aPxs5mKUPi49GUzFONSJwgWZGJmYWVj5+Dk4ubh5eMXSE7VweuOvJIiFEE9P1Q+Q1E+ur4Pc59Pw2UmfoA/XAgBmH9P0dieUQAJP1zpkFj9pABYVgPZmQvPd5oOrAzg/GjCkKaR+EIi3o4W7gH7szgzF4WfV0xcfsTDx5ofCbmjii8Yn1uRyw0YIJwfcAEq5MdQoCmnpKbT0p5RAAm/F9K0Sf/YAAD3xJDe",
    "EMPTY": "eJxjYGRg2OPyJKth0zM+JgYGhlwGTD4EPLBnIAl8wFAPAOKmDBY=",
}


def fixture(name):
    return zlib.decompress(base64.b64decode(FIX[name]))


def pose(x=0., y=0., yaw=0.):
    return [x, y, 0., 0., 0., np.sin(yaw / 2), np.cos(yaw / 2)]


def graph(*nodes):
    """nodes: (id, x, y, yaw) -> ids, poses"""
    return np.array([n[0] for n in nodes]), np.array([pose(*n[1:]) for n in nodes])


class ParserTests(unittest.TestCase):
    def test_one(self):
        stamp, ids, poses, nids = mg.parse_map_data_prefix(fixture("ONE"))
        self.assertAlmostEqual(stamp, 1793344700.25, places=6)
        self.assertEqual(ids.tolist(), [3, 4, 9])
        np.testing.assert_allclose(poses[:, 0], [0., .1, .2])
        np.testing.assert_allclose(poses[:, 1], [0., -.05, -.1])
        np.testing.assert_allclose(poses[:, 6], [1., 1., 1.])
        self.assertEqual(nids, [9])
        self.assertAlmostEqual(mg.cdr_stamp(fixture("ONE")), stamp, places=6)

    def test_two_nodes_and_other_frame_length(self):
        stamp, ids, poses, nids = mg.parse_map_data_prefix(fixture("TWO"))
        self.assertEqual(ids.tolist(), [3, 4, 9, 12])
        np.testing.assert_allclose(poses[:, 0], [0., .1, .2, .3])
        self.assertEqual(nids, [12, None])

    def test_empty(self):
        stamp, ids, poses, nids = mg.parse_map_data_prefix(fixture("EMPTY"))
        self.assertEqual((len(ids), poses.shape, nids), (0, (0, 7), []))

    def test_truncated_or_garbage_raise(self):
        raw = fixture("ONE")
        for bad in (raw[:40], raw[:7], b"\x01\x02" + raw[2:], b""):
            with self.assertRaises(ValueError):
                mg.parse_map_data_prefix(bad)


class GraphGateTests(unittest.TestCase):
    def setUp(self):
        self.g = mg.GraphGate()

    def d(self, nodes, now, node_ids="newest"):
        ids, poses = graph(*nodes)
        return self.g.decide(ids, poses, now, [int(ids[-1])] if node_ids == "newest" else node_ids)

    def test_parked_frames_skipped_moving_forwarded(self):
        base = [(1, 0, 0, 0), (2, .3, 0, 0)]
        self.assertEqual(self.d(base + [(10, .31, 0, .01)], 0.)[:2], (True, "start"))
        for k, t in enumerate((1., 2., 3.)):       # parked: a new temporary id every second, same place
            self.assertEqual(self.d(base + [(11 + k, .31, 0, .01)], t)[:2], (False, "parked_frame"))
        # the car moves on: RTAB-Map keeps a node 0.2 m further (newest, beyond half the 0.1 m threshold)
        self.assertEqual(self.d(base + [(20, .5, 0, 0)], 4.)[:2], (True, "new_node"))
        # turning in place by 0.06 rad is also a new node
        self.assertEqual(self.d(base + [(20, .5, 0, 0), (21, .5, 0, .06)], 5.)[:2], (True, "new_node"))

    def test_fallback_every_10_s(self):
        base = [(1, 0, 0, 0)]
        self.d(base + [(5, 0, 0, 0)], 0.)
        self.assertFalse(self.d(base + [(6, 0, 0, 0)], 9.9)[0])
        self.assertEqual(self.d(base + [(7, 0, 0, 0)], 10.)[:2], (True, "fallback"))
        self.assertFalse(self.d(base + [(8, 0, 0, 0)], 11.)[0])

    def test_pose_correction_forwarded_even_by_1_5_mm(self):
        self.d([(1, 0, 0, 0), (2, .3, 0, 0), (9, .3, 0, 0)], 0.)
        self.assertEqual(self.d([(1, 0, 0, 0), (2, .3015, 0, 0), (10, .3, 0, 0)], 1.)[:2], (True, "pose_changed"))
        self.assertEqual(self.d([(1, 0, 0, 0), (2, .3015, 0, .0015), (11, .3, 0, 0)], 2.)[:2], (True, "pose_changed"))
        self.assertFalse(self.d([(1, 0, 0, 0), (2, .3015, 0, .0015), (12, .3, 0, 0)], 3.)[0])

    def test_parked_frame_kept_later_is_forwarded_with_its_id(self):
        self.d([(1, 0, 0, 0), (2, .3, 0, 0), (9, .3, 0, 0)], 0.)
        self.assertFalse(self.d([(1, 0, 0, 0), (2, .3, 0, 0), (10, .32, 0, 0)], 1.)[0])
        fwd, why, need = self.d([(1, 0, 0, 0), (2, .3, 0, 0), (10, .32, 0, 0), (11, .32, 0, 0)], 2.)
        self.assertEqual((fwd, why, need), (True, "node_kept", {10}))

    def test_node_removed_and_shrunk_graph(self):
        self.d([(1, 0, 0, 0), (2, .3, 0, 0), (3, .6, 0, 0), (9, .6, 0, 0)], 0.)
        self.assertEqual(self.d([(1, 0, 0, 0), (3, .6, 0, 0), (10, .6, 0, 0)], 1.)[:2], (True, "node_removed"))
        self.assertEqual(self.d([(1, 0, 0, 0)], 2.)[:2], (True, "node_removed"))   # a new map (ids restart)

    def test_extra_node_data_forwarded(self):
        self.d([(1, 0, 0, 0), (9, 0, 0, 0)], 0.)
        self.assertEqual(self.d([(1, 0, 0, 0), (10, 0, 0, 0)], 1., node_ids=[10, None])[:2], (True, "node_data"))
        self.assertEqual(self.d([(1, 0, 0, 0), (11, 0, 0, 0)], 2., node_ids=[1])[:2], (True, "node_data"))
        self.assertEqual(self.d([(1, 0, 0, 0), (12, 0, 0, 0)], 3., node_ids=None)[:2], (True, "node_data"))

    def test_parked_past_half_threshold_skips_after_first_frame(self):
        base = [(1, 0, 0, 0), (2, .3, 0, 0)]
        self.d(base + [(9, .3, 0, 0)], 0.)
        # parked 7 cm past the last kept node: RTAB-Map discards the frame (< 0.1 m), the gate forwards the first
        self.assertEqual(self.d(base + [(10, .37, 0, 0)], 1.)[:2], (True, "new_node"))
        self.assertEqual(self.d(base + [(11, .37, 0, .001)], 2.)[:2], (False, "parked_frame"))
        self.assertEqual(self.d(base + [(12, .395, 0, 0)], 3.)[:2], (True, "new_node"))   # crept 2.5 cm

    def test_reset_and_malformed(self):
        self.d([(1, 0, 0, 0), (9, 0, 0, 0)], 0.)
        self.g.reset("mapping_reset")
        self.assertEqual(self.d([(1, 0, 0, 0), (10, 0, 0, 0)], 1.)[:2], (True, "mapping_reset"))
        ids, poses = graph((1, 0, 0, 0), (11, 0, 0, 0))
        poses[0, 0] = np.nan
        self.assertEqual(self.g.decide(ids, poses, 2., [11])[:2], (True, "malformed"))
        self.assertEqual(self.d([(1, 0, 0, 0), (12, 0, 0, 0)], 3.)[:2], (True, "malformed"))  # no reference after it

    def test_positive_control_switch(self):
        g = mg.GraphGate(check_poses=False, fallback_s=float("inf"))
        ids, poses = graph((1, 0, 0, 0), (9, 0, 0, 0))
        g.decide(ids, poses, 0., [9])
        ids, poses = graph((1, .2, 0, 0), (10, .2, 0, 0))
        self.assertFalse(g.decide(ids, poses, 1., [10])[0])        # a broken gate misses the correction


class ForwarderTests(unittest.TestCase):
    def test_inject_parse_error_resetting_subscriber(self):
        calls = []

        def inject(raw, cached):
            calls.append([nid for nid, _ in cached])
            return b"INJECTED"
        fw = mg.MapDataForwarder(mg.GraphGate(), inject)
        msgs = {}

        class G(mg.GraphGate):
            pass
        # drive the forwarder with a fake parser: the decision logic is GraphGate's; here only the plumbing
        seq = iter([graph((1, 0, 0, 0), (9, 0, 0, 0)), graph((1, 0, 0, 0), (10, .01, 0, 0)),
                    graph((1, 0, 0, 0), (10, .01, 0, 0), (11, .01, 0, 0))])
        orig = mg.parse_map_data_prefix
        try:
            def fake(raw):
                if raw == b"BAD":
                    raise ValueError("bad")
                ids, poses = msgs[raw]
                return 1.0, ids, poses, [int(ids[-1])]
            mg.parse_map_data_prefix = fake
            for k, raw in enumerate((b"m0", b"m1", b"m2")):
                msgs[raw] = next(seq)
            self.assertEqual(fw.handle(b"m0", 0., subs=1), (b"m0", "new_subscriber"))   # first assembler
            self.assertEqual(fw.handle(b"m1", 1., subs=1), (None, "parked_frame"))
            self.assertEqual(fw.handle(b"m2", 2., subs=1), (b"INJECTED", "node_kept+inject"))
            self.assertEqual(calls, [[10]])
            self.assertEqual(fw.handle(b"BAD", 3., subs=1), (b"BAD", "parse_error"))
            msgs[b"m3"] = graph((1, 0, 0, 0), (10, .01, 0, 0), (12, .01, 0, 0))
            self.assertEqual(fw.handle(b"m3", 4., subs=1)[1], "parse_error")      # reference dropped by the error
            msgs[b"m4"] = graph((1, 0, 0, 0), (10, .01, 0, 0), (13, .01, 0, 0))
            self.assertEqual(fw.handle(b"m4", 5., subs=1), (None, "parked_frame"))
            self.assertEqual(fw.handle(b"m4", 6., subs=2)[1], "new_subscriber")    # an assembler (re)joined
            fw.on_status(json.dumps({"state": "mapping", "session": 0}))
            self.assertIsNone(fw.handle(b"m4", 7., subs=2)[0])
            fw.on_status(json.dumps({"state": "resetting", "session": 0}))
            self.assertEqual(fw.handle(b"m4", 8., subs=2)[1], "resetting")         # fail-open while resetting
            self.assertEqual(fw.handle(b"m4", 9., subs=2)[1], "resetting")
            fw.on_status(json.dumps({"state": "mapping", "session": 1}))
            self.assertEqual(fw.handle(b"m4", 10., subs=2)[1], "mapping_reset")    # new session: fresh reference
            self.assertIsNone(fw.handle(b"m4", 11., subs=2)[0])
        finally:
            mg.parse_map_data_prefix = orig


class CloudGateTests(unittest.TestCase):
    def test_flow(self):
        c = lm.CloudGate(10.)
        self.assertTrue(c.on_cloud(1., 0.))                 # no graph seen yet: process
        c.done(1., 0.)
        ids, poses = graph((1, 0, 0, 0), (2, .3, 0, 0))
        self.assertTrue(c.on_graph(ids, poses, 2., 1.))     # first graph = a change
        self.assertTrue(c.on_cloud(2., 1.3))                # its cloud
        c.done(2., 1.3)
        self.assertFalse(c.on_cloud(3., 2.3))               # no change since: pending
        self.assertFalse(c.poll(5.))
        ids, poses = graph((1, 0, 0, 0), (2, .3, 0, 0), (7, .3, 0, 0))   # parked frame on /mapGraph: no change
        self.assertFalse(c.on_graph(ids, poses, 4., 3.))
        self.assertFalse(c.poll(3.))
        self.assertTrue(c.poll(11.3))                       # fallback: the pending cloud within 10 s
        c.done(3., 11.3)
        self.assertFalse(c.poll(12.))
        # a change whose cloud arrives after the graph message
        ids, poses = graph((1, 0, 0, 0), (2, .31, 0, 0), (8, .3, 0, 0))
        self.assertTrue(c.on_graph(ids, poses, 5., 12.))
        self.assertTrue(c.on_cloud(5., 12.2))
        c.done(5., 12.2)
        # a change whose cloud is older than it: wait for the newer one; a pending cloud >= the change: process
        ids, poses = graph((1, 0, 0, 0), (2, .32, 0, 0), (9, .3, 0, 0))
        self.assertFalse(c.on_cloud(6., 13.))
        self.assertTrue(c.on_graph(ids, poses, 7., 13.1))
        self.assertFalse(c.poll(13.2))                      # pending 6 < change 7
        self.assertTrue(c.on_cloud(7., 13.3))
        c.done(7., 13.3)
        self.assertFalse(c.poll(13.4))                      # the older pending was superseded
        self.assertFalse(c.on_cloud(7.5, 14.))
        ids, poses = graph((1, 0, 0, 0), (2, .33, 0, 0), (10, .3, 0, 0))
        c.on_graph(ids, poses, 7.5, 14.1)                   # graph after its cloud
        self.assertTrue(c.poll(14.1))
        c.reset()
        self.assertTrue(c.on_cloud(1., 15.))                # after a map reset: no evidence -> process

    def test_feed_wiring(self):
        src = (COMP / "live_mapping.py").read_text(encoding="utf-8")
        for needle in ('"/operator_map/cloud_map", cloud, qos, raw=True', "gate.reset()", "create_timer(1.0, flush_pending)",
                       "follow.accept_cloud(", "gate.on_graph("):
            self.assertIn(needle, src)
        sh = (COMP / "start_live_mapping.sh").read_text(encoding="utf-8")
        for needle in ('gated=/operator_map/mapData_gated', '-r "mapData:=$gated"', 'map_data_gate.py" --in-topic /mapData',
                       "ungated map_assembler replaced", "map_data_gate.py; do docker cp"):
            self.assertIn(needle, sh)
        self.assertNotIn("-r mapData:=/mapData", sh)


if __name__ == "__main__":
    unittest.main()
