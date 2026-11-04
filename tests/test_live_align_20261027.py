"""Live map alignment while mapping (2026-10-27; host/live_align.py, docs/design/LIVE_ALIGN_20261027.md).

Known answers: synthetic graphs whose per-node corrections are chosen here (opt_i = C_i x odom_i). Checked:
  * the live source (LiveAlign): nothing while opt == odom; a loop closure -> one message with C_i per node; the
    2 cm / 0.5 deg threshold; the 2 s interval (a held change goes out later without new data); the root check and the
    map_to_odom check when the root's data was missed; rtabmap_msgs field access (duck-typed MapData)
  * the invariant: after any sequence of alignments every sample / history / negative view / box / candidate =
    C_latest(its stamp) x its odometry-frame position (and boxes do not grow with repeated small rotations)
  * frames: a frame looked up under an older (or the held) alignment is brought into the memory's current one
  * run 150138 shape: a drift duplicate made while mapping merges at the first frame after the live alignment,
    number 1 kept; the stop's export alignment then corrects nothing twice; a map reset goes back to identity
  * switch off: MappingCore status / messages as before 10-27; car_up CAR_LIVE_ALIGN reaches mapping_ctl and the runner
Not executed here: ROS (rclpy, rtabmap, TF) -- the subscription / publication glue in mapping_ctl and ros_runtime.
"""
import copy
import math
import os
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tools', 'car_run'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

import mapping_ctl  # noqa: E402
from host import live_align, map_align  # noqa: E402
from perception.fpga_replay.core import Config, Frame, Memory, Observation  # noqa: E402
import test_map_align_20261025 as tma  # noqa: E402

K = np.array([[517., 0, 320], [0, 517., 240], [0, 0, 1]])
EYE7 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]


def planar(x, y, yaw):
    return tma.planar(x, y, yaw)


def odom_pose(k):
    return planar(0.2 * k, 0.02 * k, 0.03 * k)


def ramp(scale_m=0.9, yaw_deg=5.0, n=20):
    """Per-node correction of a loop closure: identity at the root (node 1) growing to (0, -scale) m, yaw at node n."""
    def c(i):
        f = (i - 1) / float(n - 1)
        return planar(0.0, -scale_m * f, math.radians(yaw_deg) * f)
    return c


def feed(la, n, corr=None, first=1, m2o=True):
    """Every node first..n delivered (one MapData per node, the graph of nodes 1..k each time); corr None = opt == odom."""
    out = None
    for k in range(first, n + 1):
        graph = {i: (odom_pose(i) if corr is None else corr(i) @ odom_pose(i)) for i in range(1, k + 1)}
        c_last = graph[k] @ np.linalg.inv(odom_pose(k))
        out = la.on_map_data([(k, 100.0 + k, odom_pose(k))], graph, c_last if m2o else None)
    return out


def graph_msg(la, n, corr, m2o=None):
    graph = {i: corr(i) @ odom_pose(i) for i in range(1, n + 1)}
    return la.on_map_data([], graph, m2o)


class LiveSource(unittest.TestCase):
    def test_no_message_while_the_graph_equals_odometry(self):
        la = live_align.LiveAlign(run='R', clock=lambda: 0.0)
        a = feed(la, 20)
        self.assertIsNotNone(a)
        for n in a['nodes']:                              # identity up to rounding
            np.testing.assert_allclose(n[1:], EYE7, atol=1e-12)
        self.assertIsNone(la.poll(100.0))
        self.assertEqual(la.status()['nodes'], 20)

    def test_loop_closure_publishes_every_node_correction(self):
        la = live_align.LiveAlign(run='R')
        feed(la, 20)
        c = ramp()
        a = graph_msg(la, 20, c)
        self.assertEqual(a['root']['node_id'], 1)
        msg = la.poll(10.0)
        self.assertIsNotNone(msg)
        self.assertEqual((msg['key'], msg['seq'], msg['live'], msg['count'], msg['node_id']), ('live:R:0:1', 1, True, 20, 20))
        np.testing.assert_allclose(map_align.pose7_matrix(*msg['map_odom']), c(20), atol=1e-9)
        for n in msg['nodes']:
            np.testing.assert_allclose(map_align.pose7_matrix(*n[1:]), c(int(round(n[0] - 100.0))), atol=1e-9)
        moved = max(float(np.linalg.norm(c(i)[:3, :3] @ odom_pose(i)[:3, 3] + c(i)[:3, 3] - odom_pose(i)[:3, 3]))
                    for i in range(1, 21))                 # how far the loop closure moves each node
        self.assertAlmostEqual(msg['change']['m'], moved, places=9)
        self.assertIsNone(la.poll(10.1))                  # sent once

    def test_threshold_and_interval(self):
        la = live_align.LiveAlign(run='R')
        feed(la, 20)
        graph_msg(la, 20, ramp(0.9))
        self.assertIsNotNone(la.poll(10.0))
        graph_msg(la, 20, ramp(0.905))                  # 5 mm more at the last node: below 2 cm / 0.5 deg
        self.assertIsNone(la.pending)
        self.assertIsNone(la.poll(20.0))
        graph_msg(la, 20, ramp(1.0))                    # 10 cm more: a change, but within 2 s of the last message
        self.assertIsNone(la.poll(11.0))
        msg = la.poll(12.0)                             # due now, no new /mapData needed
        self.assertIsNotNone(msg)
        self.assertEqual(msg['seq'], 2)
        np.testing.assert_allclose(map_align.pose7_matrix(*msg['map_odom']), ramp(1.0)(20), atol=1e-9)

    def test_rotation_only_change_counts(self):
        la = live_align.LiveAlign(run='R')
        feed(la, 20)
        def turn_in_place(i):                           # every node turned 0.8 deg about its own position: none moves
            p = odom_pose(i)[:3, 3]
            return planar(p[0], p[1], 0.0) @ planar(0, 0, math.radians(0.8) if i > 1 else 0.0) @ planar(-p[0], -p[1], 0)
        a = graph_msg(la, 20, turn_in_place)
        self.assertLess(a['change']['m'], 1e-9)
        self.assertIsNotNone(la.poll(5.0))

    def test_root_check(self):
        la = live_align.LiveAlign(run='R')
        feed(la, 20)
        bad = lambda i: planar(0.1, 0.0, 0.0) @ ramp()(i)   # noqa: E731 - the root is moved: not one frame
        self.assertIsNone(graph_msg(la, 20, bad))
        self.assertIn('root check failed', la.reason)
        self.assertIsNone(la.poll(99.0))

    def test_missed_root_uses_map_to_odom(self):
        la = live_align.LiveAlign(run='R')
        feed(la, 20, first=3)                           # nodes 1, 2 never received
        c = ramp()
        a = graph_msg(la, 20, c, m2o=c(20))
        self.assertEqual(a['root']['checked_by'], 'map_to_odom')
        self.assertEqual(a['root']['missing_root'], 1)
        self.assertIsNotNone(la.poll(5.0))
        la2 = live_align.LiveAlign(run='R')
        feed(la2, 20, first=3)
        self.assertIsNone(graph_msg(la2, 20, c, m2o=planar(0.5, 0, 0)))
        self.assertIn('map_to_odom', la2.reason)
        self.assertIsNone(graph_msg(la2, 20, c, m2o=None))
        self.assertIsNone(la2.poll(5.0))

    def test_reset_starts_a_new_session(self):
        la = live_align.LiveAlign(run='R')
        feed(la, 20)
        graph_msg(la, 20, ramp())
        la.poll(1.0)
        self.assertTrue(la.nonidentity_sent())
        la.reset()
        self.assertEqual((la.status()['nodes'], la.published, la.session), (0, None, 1))
        feed(la, 5)
        graph_msg(la, 5, ramp(n=5))
        self.assertTrue(la.poll(2.0)['key'].startswith('live:R:1:'))

    def test_map_data_fields(self):
        """rtabmap_msgs 0.23.7: MapData.graph.{map_to_odom, poses_id, poses}, MapData.nodes[].{id, stamp, pose}."""
        def pose(m):
            p = map_align.matrix_pose7(m)
            return NS(position=NS(x=p[0], y=p[1], z=p[2]), orientation=NS(x=p[3], y=p[4], z=p[5], w=p[6]))
        c = ramp()
        tf = map_align.matrix_pose7(c(3))
        msg = NS(nodes=[NS(id=3, stamp=103.0, pose=pose(odom_pose(3)))],
                 graph=NS(poses_id=[1, 2, 3], poses=[pose(c(i) @ odom_pose(i)) for i in (1, 2, 3)],
                          map_to_odom=NS(translation=NS(x=tf[0], y=tf[1], z=tf[2]),
                                         rotation=NS(x=tf[3], y=tf[4], z=tf[5], w=tf[6]))))
        nodes, graph, m2o = live_align.tables_from_map_data(msg)
        self.assertEqual([(n[0], n[1]) for n in nodes], [(3, 103.0)])
        np.testing.assert_allclose(nodes[0][2], odom_pose(3), atol=1e-9)
        self.assertEqual(sorted(graph), [1, 2, 3])
        np.testing.assert_allclose(graph[2], c(2) @ odom_pose(2), atol=1e-9)
        np.testing.assert_allclose(m2o, c(3), atol=1e-9)

    def test_export_alignment_equals_live_on_the_same_graph(self):
        """The stop's file-based compute and the live table compute give the same corrections (float32 DB poses)."""
        d = tempfile.mkdtemp()
        poses, db = tma.write_graph(d)
        off = map_align.compute(poses, db)
        la = live_align.LiveAlign()
        odom = {i + 1: (tma.T0 + i, tma.odom(i)) for i in range(tma.N)}
        opt = {i + 1: tma.correction(i) @ tma.odom(i) for i in range(tma.N)}
        la.on_map_data([(i, s, m) for i, (s, m) in odom.items()], opt)
        live = la.poll(0.0)
        self.assertEqual(len(live['nodes']), len(off['nodes']))
        for a, b in zip(live['nodes'], off['nodes']):
            self.assertAlmostEqual(a[0], b[0], places=5)
            np.testing.assert_allclose(map_align.pose7_matrix(*a[1:]), map_align.pose7_matrix(*b[1:]), atol=2e-5)


# ------------------------------------------------------------------------------------------------ memory invariant
def align_doc(corr, stamps, key=None, seq=None, live=True):
    """An alignment message: nodes at stamps with corrections corr(k) (k = index), C_last = corr(last)."""
    nodes = sorted([s] + map_align.matrix_pose7(corr(k)) for k, s in enumerate(stamps))
    doc = {"map_odom": map_align.matrix_pose7(corr(len(stamps) - 1)), "node_id": len(stamps), "nodes": nodes,
           "count": len(stamps), "root": None, "key": key}
    if seq is not None:
        doc.update(seq=seq, live=live)
    return doc


def rand_corr(seed, n):
    rng = np.random.default_rng(seed)
    xs, ys, yaws = rng.uniform(-0.5, 0.5, n), rng.uniform(-0.5, 0.5, n), rng.uniform(-0.15, 0.15, n)
    xs[0] = ys[0] = yaws[0] = 0.0
    return lambda k: planar(xs[k], ys[k], yaws[k])


class Car:
    """Camera at (x, y) looking along +x (world = odometry frame until aligned), 5 Hz, observations with range_m."""

    def __init__(self, cfg=Config()):
        self.memory, self.t, self.i = Memory("geometry", cfg), 1000.0, 0

    def frame(self, x=0.0, y=0.0, pre=None):
        self.i += 1
        self.t += 0.2
        T = np.eye(4)
        T[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        T[0, 3], T[1, 3] = x, y
        if pre is not None:
            T = pre @ T
        f = Frame("f-%05d" % self.i, self.t, np.zeros((2, 2, 3), np.uint8), np.full((2, 2), np.nan, np.float32),
                  K, T, True, "test", "test")
        f.ground_z_m = 0.0
        return f

    def obs(self, f, p, x=0.0, y=0.0):
        p = np.array(p, dtype=float)
        rng = float(p[0] - x)
        u = 320 - 517 * (p[1] - y) / rng
        return Observation("bottle", 0.8, p, p - .03, p + .03, "red", 1.0, f.frame_id, f.timestamp,
                           (u - 10, 200, u + 10, 260), "test", extent_m=(0.06, 0.2), truncated=False, range_m=rng)

    def step(self, points, x=0.0, y=0.0, f=None):
        f = self.frame(x, y) if f is None else f
        return self.memory.update_memory([self.obs(f, p, x, y) for p in points], {}, f)

    def bottles(self):
        return sorted((t for t in self.memory.tracks.values() if t.label == "bottle"), key=lambda t: t.id)


def odom_snapshot(memory):
    snap = {}
    for kind, d in (("t", memory.tracks), ("c", getattr(memory, "candidates", {}))):
        for k, t in d.items():
            snap[(kind, k)] = dict(last_seen=t.last_seen, position=np.array(t.position, float),
                                   lower=np.array(t.lower, float), upper=np.array(t.upper, float),
                                   samples=[(s, np.array(p, float)) for s, p, _ in getattr(t, "samples", [])],
                                   history=[(h.get("timestamp"), np.array(h["position_m"], float)) for h in t.history
                                            if isinstance(h, dict) and h.get("position_m") is not None],
                                   negative=[(s, np.array(p, float)) for s, p in t.negative_poses])
    return snap


def stamps_of(car_t0, car_t1, n):
    return list(np.linspace(car_t0, car_t1, n))


class Invariant(unittest.TestCase):
    def build(self):
        car = Car()
        for k in range(15):
            car.step([(1.5, 0.0, 0.11), (1.6, 0.8, 0.11)], x=0.02 * k)
        car.t += 20.0
        for k in range(10):
            car.step([(1.4, -0.7, 0.11)], x=0.3)
        car.step([(1.2, 0.3, 0.11)], x=0.3)               # one sighting: a creation-gate candidate
        (t0,) = [t for t in car.bottles()][:1]
        t0.negative_poses.append((car.t - 1.0, car.frame(0.1).T_world_camera.copy()))
        car.t -= 0.2
        self.assertTrue(car.memory.candidates)
        return car

    def check(self, memory, snap, doc):
        corr = map_align.Corrections(doc)
        for (kind, k), o in snap.items():
            t = (memory.tracks if kind == "t" else memory.candidates)[k]
            c = corr.at(float(o["last_seen"]))
            np.testing.assert_allclose(t.position, map_align._point(c, o["position"]), atol=1e-9)
            lo, hi = map_align._box(c, o["lower"], o["upper"])
            np.testing.assert_allclose(t.lower, lo, atol=1e-9)
            np.testing.assert_allclose(t.upper, hi, atol=1e-9)
            for (s, p), (s2, p2, _sd) in zip(o["samples"], t.samples):
                np.testing.assert_allclose(p2, map_align._point(corr.at(s), p), atol=1e-9)
            hist = [h for h in t.history if isinstance(h, dict) and h.get("position_m") is not None]
            for (s, p), h in zip(o["history"], hist):
                np.testing.assert_allclose(h["position_m"], map_align._point(corr.at(float(s or 0.0)), p), atol=1e-9)
            for (s, p), (s2, p2) in zip(o["negative"], t.negative_poses):
                np.testing.assert_allclose(p2, corr.at(s) @ p, atol=1e-9)

    def test_any_sequence_of_alignments(self):
        car0 = self.build()
        st = list(stamps_of(1000.0, car0.t, 12))
        docs = {i: align_doc(rand_corr(i, 12), st) for i in (1, 2, 3)}
        for order in ([1, 2, 3], [3, 1, 2], [2, 2, 1], [1]):
            car = copy.deepcopy(car0)
            snap = odom_snapshot(car.memory)
            s = live_align.new_runner_state(NS())
            for seq, i in enumerate(order, 1):
                d = dict(docs[i], seq=seq, live=True, key="live:x:%d" % seq)
                self.assertEqual(live_align.runner_receive(s, d, d["key"], 0.0)[0], "received")
                live_align.runner_apply(s, car.memory)
                self.check(car.memory, snap, docs[i])          # = the latest one x the odometry position
            # identity at the end (the stop's export alignment found no correction): back to odometry
            ident = align_doc(lambda k: np.eye(4), st)
            live_align.runner_receive(s, dict(ident, seq=99, key="yaml"), "yaml", 0.0)
            live_align.runner_apply(s, car.memory)
            self.check(car.memory, snap, ident)

    def test_positive_control_without_the_difference_corrects_twice(self):
        car = self.build()
        st = list(stamps_of(1000.0, car.t, 12))
        a = align_doc(rand_corr(1, 12), st)
        snap = odom_snapshot(car.memory)
        map_align.apply_memory(car.memory, a)
        map_align.apply_memory(car.memory, a)                 # the 10-25 arithmetic twice: corrected twice
        with self.assertRaises(AssertionError):
            self.check(car.memory, snap, a)

    def test_stale_seq_is_dropped(self):
        car = self.build()
        st = list(stamps_of(1000.0, car.t, 12))
        s = live_align.new_runner_state(NS())
        a, b = align_doc(rand_corr(1, 12), st, seq=2), align_doc(rand_corr(2, 12), st, seq=1)
        live_align.runner_receive(s, a, "a", 0.0)
        live_align.runner_apply(s, car.memory)
        what, fields = live_align.runner_receive(s, b, "b", 0.0)
        self.assertEqual((what, fields["newest"]), ("stale", 2))
        self.assertIsNone(s.memory_align_pending)
        self.assertIn("b", s.memory_align_done)

    def test_boxes_do_not_grow_with_repeated_small_turns(self):
        car = self.build()
        st = list(stamps_of(1000.0, car.t, 12))
        s = live_align.new_runner_state(NS())
        t = car.bottles()[0]
        lo0, hi0 = np.array(t.lower), np.array(t.upper)
        for k in range(40):                                  # 40 alignments alternating +-3 deg
            d = align_doc(lambda i, k=k: planar(0, 0, math.radians(3.0 if k % 2 else -3.0)), st, seq=k + 1)
            live_align.runner_receive(s, d, "k%d" % k, 0.0)
            live_align.runner_apply(s, car.memory)
        grown = (np.array(t.upper) - np.array(t.lower)) / (hi0 - lo0)
        self.assertLess(float(grown.max()), 1.1)


class Frames(unittest.TestCase):
    def test_in_flight_frame_from_an_older_alignment(self):
        st = list(stamps_of(1000.0, 1010.0, 6))
        a1, a2 = align_doc(rand_corr(1, 6), st, seq=1), align_doc(rand_corr(2, 6), st, seq=2)
        s = live_align.new_runner_state(NS())
        mem = Memory("geometry", Config())
        for i, d in enumerate((a1, a2)):
            live_align.runner_receive(s, d, "k%d" % i, 0.0)
            live_align.runner_apply(s, mem)
        car = Car()
        car.t = 1004.9
        tag = map_align.pose7_matrix(*a1["map_odom"])
        f = car.frame(0.5, 0.2, pre=tag)                    # looked up while TF still showed a1
        T_odom = np.linalg.inv(tag) @ f.T_world_camera
        wb = tag @ planar(0.5, 0.2, 0.0)
        f.map_odom_tag, f.world_base_matrix = tag, wb
        self.assertIsNone(live_align.runner_align_frame(s, mem, f))
        c = map_align.Corrections(a2).at(f.timestamp)
        np.testing.assert_allclose(f.T_world_camera, c @ T_odom, atol=1e-9)
        self.assertAlmostEqual(f.ground_z_m, float((c @ planar(0.5, 0.2, 0.0))[2, 3]))
        self.assertEqual(s.live_align_frames["converted"], 1)

    def test_frame_with_the_held_alignment_applies_it_first(self):
        st = list(stamps_of(1000.0, 1010.0, 6))
        a1 = align_doc(rand_corr(1, 6), st, seq=1)
        s = live_align.new_runner_state(NS())
        car = Car()
        car.step([(1.5, 0.0, 0.11)])
        live_align.runner_receive(s, a1, "k1", 0.0)
        car.t = 1020.0                                      # after the last node: C_cur(stamp) = C_last = the tag
        tag = map_align.pose7_matrix(*a1["map_odom"])
        f = car.frame(0.0, 0.0, pre=tag)
        f.map_odom_tag = tag
        before = f.T_world_camera.copy()
        info = live_align.runner_align_frame(s, car.memory, f)
        self.assertIsNotNone(info)
        self.assertIs(s.memory_align_cur, a1)
        np.testing.assert_allclose(f.T_world_camera, before, atol=1e-9)   # already in a1: nothing to convert

    def test_untagged_frame_is_left_as_it_is(self):
        s = live_align.new_runner_state(NS())
        car = Car()
        f = car.frame(0.3)
        before = f.T_world_camera.copy()
        self.assertIsNone(live_align.runner_align_frame(s, car.memory, f))
        np.testing.assert_array_equal(f.T_world_camera, before)
        self.assertEqual(s.live_align_frames["untagged"], 1)


class Run150138(unittest.TestCase):
    """The same bottle seen again minutes later through 0.9 m of drift, the map never stopped."""

    FIX = planar(0.0, -0.9, 0.0)

    def drive(self):
        car = Car()
        for k in range(15):
            car.step([(1.5, 0.0, 0.11)])
        t_first = car.t
        car.t += 300.0
        for k in range(15):
            car.step([(1.5, 0.9, 0.11)])
        a, b = car.bottles()
        self.assertEqual((a.number, b.number), (1, 2))
        # the live graph: early nodes uncorrected, the revisit's nodes corrected by the loop closure
        st = [1000.0, t_first, car.t - 3.0, car.t]
        corr = lambda k: np.eye(4) if k < 2 else self.FIX     # noqa: E731
        return car, a, b, st, corr

    def test_live_alignment_merges_at_the_next_frame(self):
        car, a, b, st, corr = self.drive()
        s = live_align.new_runner_state(NS())
        live = align_doc(corr, st, key="live:R:0:1", seq=1)
        live_align.runner_receive(s, live, live["key"], 0.0)
        f = car.frame(0.0, 0.0, pre=self.FIX)               # TF has become the live alignment
        f.map_odom_tag = self.FIX
        self.assertIsNotNone(live_align.runner_align_frame(s, car.memory, f))
        r = car.memory.update_memory([], {}, f)
        (t,) = car.bottles()
        self.assertEqual((t.id, t.number), (a.id, 1))
        self.assertEqual(r["merged"][0]["merged"], b.id)
        self.assertLess(abs(t.position[1]), 0.03)
        # mapping goes on: the bottle seen from the corrected pose stays one track, number 1
        for k in range(10):
            f = car.frame(0.0, 0.0, pre=self.FIX)
            f.map_odom_tag = self.FIX
            live_align.runner_align_frame(s, car.memory, f)
            car.memory.update_memory([car.obs(f, (1.5, 0.0, 0.11))], {}, f)
        (t,) = car.bottles()
        self.assertEqual(t.number, 1)
        pos, samples = t.position.copy(), [p.copy() for _s, p, _sd in t.samples]
        # the stop: the export's alignment is the same graph -> nothing moves a second time
        final = align_doc(corr, st, key="/run/export/map.yaml", seq=2, live=False)
        info = live_align.runner_receive(s, final, final["key"], 0.0)
        live_align.runner_apply(s, car.memory)
        np.testing.assert_allclose(t.position, pos, atol=1e-9)
        for p0, (_s, p1, _sd) in zip(samples, t.samples):
            np.testing.assert_allclose(p1, p0, atol=1e-9)
        self.assertEqual(info[0], "received")
        # map reset: memory cleared by the runner, alignment back to none: an identity-tagged frame is not converted
        live_align.runner_reset(s, "reset-key")
        self.assertIsNone(s.memory_align_cur)
        f = car.frame(0.4)
        f.map_odom_tag = np.eye(4)
        before = f.T_world_camera.copy()
        live_align.runner_align_frame(s, Memory("geometry", Config()), f)
        np.testing.assert_array_equal(f.T_world_camera, before)

    def test_export_slightly_different_from_live_lands_on_export(self):
        car, a, b, st, corr = self.drive()
        s = live_align.new_runner_state(NS())
        snap = odom_snapshot(car.memory)
        live_align.runner_receive(s, align_doc(corr, st, seq=1), "l1", 0.0)
        live_align.runner_apply(s, car.memory)
        corr2 = lambda k: np.eye(4) if k < 2 else planar(0.03, -0.88, 0.01)   # noqa: E731 - the export re-optimized
        final = align_doc(corr2, st, seq=2, live=False)
        live_align.runner_receive(s, final, "yaml", 0.0)
        live_align.runner_apply(s, car.memory)
        Invariant.check(self, car.memory, snap, final)

    def test_positive_control_without_alignment_two_tracks_stay(self):
        car, a, b, st, corr = self.drive()
        car.memory.update_memory([], {}, car.frame())
        self.assertEqual(len(car.bottles()), 2)


# ------------------------------------------------------------------------------------------------ mapping_ctl core
class Core(unittest.TestCase):
    def core(self, live):
        d = tempfile.mkdtemp()
        return mapping_ctl.MappingCore(d, d, clock=lambda: 0.0,
                                       live=live_align.LiveAlign(run='R') if live else None)

    def test_off_is_as_before(self):
        c = self.core(False)
        self.assertNotIn('live_align', c.status())
        self.assertIsNone(c.live_message())
        c.align = {"ok": True, "identity": False, "map_odom": [0.1] + EYE7[1:], "node_id": 3, "nodes": [], "count": 3,
                   "root": {}, "source": {}}
        c.yaml = 'y.yaml'
        self.assertEqual(sorted(c.align_message()), sorted(["key", "map_odom", "node_id", "nodes", "count", "root",
                                                            "source"]))
        c.align = dict(c.align, identity=True)
        self.assertIsNone(c.align_message())
        c.reset_from = 'mapping'
        self.assertNotIn('map_odom', c.reset_plan())

    def test_live_messages_only_while_mapping_and_the_stop_returns_identity(self):
        c = self.core(True)
        feed(c.live, 20)
        c.on_live_map_data([], {i: ramp()(i) @ odom_pose(i) for i in range(1, 21)})
        m = c.live_message(5.0)
        self.assertEqual(m['seq'], 1)
        self.assertTrue(c.map_odom_moved())
        self.assertIn('live_align', c.status())
        c.on_live_map_data([], {i: ramp(0.5)(i) @ odom_pose(i) for i in range(1, 21)})
        self.assertIsNone(c.live_message(6.0))              # held: within 2 s of the last one
        self.assertTrue(c.live.pending)
        c.request_stop()                                    # saving: the held one is not sent any more
        self.assertIsNone(c.live_message(50.0))
        c.on_live_map_data([], {i: ramp(0.7)(i) @ odom_pose(i) for i in range(1, 21)})
        self.assertIsNone(c.live_message(60.0))
        c.align = {"ok": True, "identity": True, "map_odom": EYE7, "node_id": 20, "nodes": [], "count": 20,
                   "root": {}, "source": {}}
        c.yaml = 'y.yaml'
        msg = c.align_message()                             # identity, but map -> odom must come back from live
        self.assertIsNotNone(msg)
        self.assertEqual(msg['seq'], 2)
        c.state, c.map_loaded, c.reset_from = 'exported', True, 'exported'
        self.assertIn('map_odom', c.reset_plan())
        self.assertIsNone(c.begin_reset('r1'))
        self.assertEqual(c.live.session, 1)


class Switch(unittest.TestCase):
    def test_car_up_passes_the_switch(self):
        import test_car_run as tcr

        class T(tcr.Base):
            def runTest(self):
                pass
        t = T()
        t.setUp()
        try:
            for val, flag in (('', True), ('1', True), ('0', False)):
                env = dict(CAR_ARCH='x86_64', CAR_MAPPING='1', RTABMAP_DB='', CAR_MAP_YAML='')
                if val:
                    env['CAR_LIVE_ALIGN'] = val
                r = t.up('sim', '--dry-run', **env)
                self.assertEqual(r.returncode, 0, r.stderr)
                self.assertIn('-e LIVE_ALIGN=%s ' % (val or '1'), r.stdout)
                self.assertEqual(r.stdout.count('--live-align') == 2, flag, val)
                if not flag:
                    self.assertNotIn('--live-align', r.stdout)
            r = t.up('sim', '--dry-run', CAR_ARCH='x86_64', CAR_MAPPING='1', CAR_LIVE_ALIGN='yes')
            self.assertEqual(r.returncode, 2)
            self.assertIn('CAR_LIVE_ALIGN', r.stderr)
        finally:
            t.tearDown()


if __name__ == "__main__":
    unittest.main()
