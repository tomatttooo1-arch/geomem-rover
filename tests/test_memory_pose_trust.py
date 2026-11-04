"""Pose trust (10-17): wheel vs gyro yaw disagreement pauses memory writes.

docs/design/MEMORY_POSE_TRUST_20261017.md. Three parts:

* the detector (perception/fpga_replay/pose_trust.py): discrepancy, step (trip / clear /
  settle boundaries, missing data) and the buffering monitor;
* the memory entry (Memory.update_memory / update_geometry_only with frame.pose_trusted);
* the 10-16 real-car run: 20 Hz yaw series (tests/data/memq_realcar_20261016_yaw20hz.json,
  from chassis_record gyro + wheel_odom yaw) and the compact observation stream, criteria
  T1-T4 of the design doc, with the gate removed as negative control.
"""
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from perception.fpga_replay import pose_trust as pt
from perception.fpga_replay.core import (Config as _Config, Frame, Memory, Observation, UNGATED_CREATION,
                                        compute_geometry, process_frame)
from perception.fpga_replay.synthetic import scene


def Config(**kw):
    """core.Config with the D-044 creation gate off (core.UNGATED_CREATION). The expectations in this file
    were written for single-sighting creation; the gate has its own tests (test_memory_creation_gate.py)."""
    return _Config(**dict(UNGATED_CREATION, **kw))


ROOT = Path(__file__).resolve().parents[1]
STREAM = ROOT / "tests/data/memq_realcar_20261016_compact.json"
YAW = ROOT / "tests/data/memq_realcar_20261016_yaw20hz.json"
P = pt.DEFAULT_PARAMS
K = np.array([[500., 0, 320], [0, 500, 240], [0, 0, 1]])


def series(rate_g, rate_w, t1, dt=0.01, t0=0.0):
    """Cumulative yaw series of constant (or callable) rates on [t0, t1]."""
    times = list(np.arange(t0, t1 + dt / 2, dt))
    g, w, sg, sw = [], [], 0.0, 0.0
    for i, t in enumerate(times):
        if i:
            sg += (rate_g(t) if callable(rate_g) else rate_g) * dt
            sw += (rate_w(t) if callable(rate_w) else rate_w) * dt
        g.append(sg); w.append(sw)
    return (times, g), (times, w)


def feed(monitor, gyro_rate, wheel_rate, t0, t1, dt=0.01, state=None):
    """Feed the monitor gyro rates (integrated by the monitor) and wheel yaw from t0 to t1."""
    yaw = state if state is not None else [0.0]
    t = t0
    while t <= t1 + 1e-9:
        monitor.add_gyro_rate(t, gyro_rate(t) if callable(gyro_rate) else gyro_rate)
        yaw[0] += (wheel_rate(t) if callable(wheel_rate) else wheel_rate) * dt
        monitor.add_wheel_yaw(t, math.atan2(math.sin(yaw[0]), math.cos(yaw[0])))
        t = round(t + dt, 6)
    return yaw


class DiscrepancyTests(unittest.TestCase):
    def test_consistent_turn_is_zero(self):
        g, w = series(1.5, 1.5, 2.0)
        self.assertAlmostEqual(pt.discrepancy(g, w, 1.5, P.window_s, P.max_gap_s), 0.0, places=9)

    def test_gyro_only_turn_equals_gyro_change(self):
        g, w = series(1.0, 0.0, 2.0)
        self.assertAlmostEqual(pt.discrepancy(g, w, 1.5, P.window_s, P.max_gap_s), 0.5, places=6)
        g, w = series(0.0, 1.0, 2.0)                        # wheels turn, gyro does not: sign kept
        self.assertAlmostEqual(pt.discrepancy(g, w, 1.5, P.window_s, P.max_gap_s), -0.5, places=6)

    def test_missing_coverage_or_gap_is_none(self):
        g, w = series(1.0, 0.0, 2.0)
        self.assertIsNone(pt.discrepancy(g, w, 0.3, P.window_s, P.max_gap_s))      # window starts before data
        self.assertIsNone(pt.discrepancy(g, w, 2.5, P.window_s, P.max_gap_s))      # past the last sample
        self.assertIsNone(pt.discrepancy(([], []), w, 1.5, P.window_s, P.max_gap_s))
        times, values = g
        keep = [i for i, t in enumerate(times) if not 1.1 < t < 1.4]           # 0.3 s hole > 0.25
        hole = ([times[i] for i in keep], [values[i] for i in keep])
        self.assertIsNone(pt.discrepancy(hole, w, 1.5, P.window_s, P.max_gap_s))
        keep = [i for i, t in enumerate(times) if not 1.1 < t < 1.3]           # 0.2 s hole: allowed
        small = ([times[i] for i in keep], [values[i] for i in keep])
        self.assertIsNotNone(pt.discrepancy(small, w, 1.5, P.window_s, P.max_gap_s))


class StepTests(unittest.TestCase):
    def test_trip_boundary(self):
        self.assertTrue(pt.step(pt.INITIAL, 1.0, P.trip_rad, P).trusted)           # equal: still trusted
        s = pt.step(pt.INITIAL, 1.0, -(P.trip_rad + 1e-6), P)
        self.assertFalse(s.trusted)
        self.assertEqual(s.since, 1.0)

    def test_settle_boundary_and_reset(self):
        s = pt.step(pt.INITIAL, 0.0, 2 * P.trip_rad, P)
        s = pt.step(s, 1.0, P.clear_rad, P)                                     # equal to clear: calm
        self.assertEqual(s.calm_since, 1.0)
        s2 = pt.step(s, 1.0 + P.settle_s - 0.01, 0.0, P)
        self.assertFalse(s2.trusted)
        s3 = pt.step(s2, 1.0 + P.settle_s, 0.0, P)
        self.assertTrue(s3.trusted)
        r = pt.step(s2, 2.0, P.clear_rad + 1e-6, P)                              # above clear: restart
        self.assertFalse(r.trusted)
        self.assertIsNone(r.calm_since)

    def test_hysteresis_between_clear_and_trip(self):
        self.assertEqual((round(math.degrees(P.trip_rad), 6), round(math.degrees(P.clear_rad), 6)), (10.0, 5.0))
        s = pt.step(pt.INITIAL, 0.0, math.radians(12), P)
        s = pt.step(s, 1.0, math.radians(7), P)                                # below trip, above clear
        self.assertFalse(s.trusted)
        self.assertIsNone(s.calm_since)
        self.assertTrue(pt.step(pt.INITIAL, 1.0, math.radians(7), P).trusted)  # and it never trips

    def test_missing_data_holds_state(self):
        self.assertEqual(pt.step(pt.INITIAL, 1.0, None, P), pt.INITIAL)
        s = pt.step(pt.INITIAL, 0.0, 1.0, P)
        s = pt.step(s, 0.5, 0.0, P)
        held = pt.step(s, 0.6, None, P)
        self.assertFalse(held.trusted)
        self.assertIsNone(held.calm_since)                                      # settling restarts

    def test_unwrap(self):
        a = pt.unwrap(None, 3.1)
        b = pt.unwrap(a, -3.1)
        self.assertAlmostEqual(b - a, 2 * math.pi - 6.2, places=9)


class MonitorTests(unittest.TestCase):
    def test_consistent_turns_and_scale_error_never_trip(self):
        m = pt.PoseTrustMonitor()
        feed(m, lambda t: 2.0 * math.sin(t), lambda t: 2.0 * math.sin(t), 0.0, 20.0)
        feed(m, 0.93 * math.radians(200), math.radians(200), 20.01, 30.0, state=[0.0])   # 7 % scale, 200 deg/s
        self.assertEqual(m.transitions, [])
        self.assertTrue(m.trusted_at(25.0))

    def test_gyro_only_turn_trips_then_recovers(self):
        m = pt.PoseTrustMonitor()
        yaw = feed(m, 0.0, 0.0, 0.0, 5.0)
        feed(m, math.radians(60), 0.0, 5.01, 6.0, state=yaw)                    # bumped: 60 deg/s, wheels still
        feed(m, 0.0, 0.0, 6.01, 12.0, state=yaw)
        (t_trip, ok1, _), (t_back, ok2, _) = m.transitions
        self.assertFalse(ok1); self.assertTrue(ok2)
        self.assertGreater(t_trip, 5.0)
        self.assertLess(t_trip, 5.0 + 0.3)                                      # 10 deg at 60 deg/s = 0.17 s
        # calm once the gyro-only part left in the window is <= clear: 6.0 + W - clear/rate, then settle
        calm = 6.0 + P.window_s - P.clear_rad / math.radians(60)
        self.assertAlmostEqual(t_back, calm + P.settle_s, delta=0.03)
        self.assertTrue(m.trusted_at(4.9))
        self.assertFalse(m.trusted_at(t_trip + 0.001))
        self.assertFalse(m.trusted_at(t_back - 0.01))
        self.assertTrue(m.trusted_at(t_back))
        self.assertEqual(len(m.drain_transitions()), 2)
        self.assertEqual(m.drain_transitions(), [])

    def test_same_stamp_batch_integrates_by_frame_period(self):
        m = pt.PoseTrustMonitor()
        for k in range(3):                                                      # three frames, one stamp
            m.add_gyro_rate(1.0, 1.0)
        self.assertAlmostEqual(m.gyro[1][-1], 0.03, places=12)
        self.assertEqual(m.gyro[0], [1.0])

    def test_no_gyro_means_always_trusted(self):
        m = pt.PoseTrustMonitor()
        for k in range(500):
            m.add_wheel_yaw(k * 0.01, k * 0.01)
        self.assertEqual(m.transitions, [])
        self.assertTrue(m.trusted_at(3.0))

    def test_gyro_stops_while_untrusted_holds_untrusted(self):
        m = pt.PoseTrustMonitor()
        yaw = feed(m, 0.0, 0.0, 0.0, 2.0)
        feed(m, math.radians(90), 0.0, 2.01, 3.0, state=yaw)
        for k in range(1, 600):                                                 # wheel only from here on
            m.add_wheel_yaw(3.0 + k * 0.01, yaw[0])
        self.assertFalse(m.state.trusted)
        self.assertFalse(m.trusted_at(8.0))


class Driver:
    def __init__(self, cfg=Config()):
        self.memory, self.i = Memory("geometry", cfg), 0

    def step(self, specs=(), trusted=None, x=0.0, pose_valid=True, geometry=None):
        self.i += 1
        T = np.eye(4)
        T[0, 3] = x
        f = Frame("q-%04d" % self.i, float(self.i), np.zeros((2, 2, 3), np.uint8),
                  np.full((2, 2), np.nan, np.float32), K, T, pose_valid, "test", "test")
        if trusted is not None:
            f.pose_trusted = trusted
        obs = [Observation(lab, 0.9, np.array(p, float), np.array(p, float) - .03, np.array(p, float) + .03,
                           "red", 1.0, f.frame_id, f.timestamp, (100, 100, 130, 200), "test",
                           extent_m=(0.06, 0.2)) for lab, p in specs]
        return self.memory.update_memory(obs, geometry or {}, f)


class MemoryEntryTests(unittest.TestCase):
    def test_untrusted_frame_stores_nothing_then_recovers(self):
        d = Driver()
        r = d.step([("bottle", (1, 0, .1))], trusted=False)
        self.assertEqual(d.memory.tracks, {})
        self.assertEqual([x["reason"] for x in r["rejected"]], ["pose_untrusted"])
        self.assertEqual(d.memory.rejected, {"pose_untrusted": 1})
        d.step([("bottle", (1, 0, .1))], trusted=True)
        self.assertEqual(len(d.memory.tracks), 1)
        d.step([("bottle", (1.02, 0, .1))])                                      # attribute absent = trusted
        t = d.memory.tracks["object-0001"]                       # 10-27: newest observation; position fused
        self.assertEqual(t.history[-1]["position_m"], [1.02, 0, .1])
        self.assertEqual(len(t.history), 2)

    def test_untrusted_frame_does_not_update_or_advance_odometer(self):
        d = Driver()
        d.step([("bottle", (1, 0, .1))])
        d.step([("bottle", (1.1, 0, .1))], trusted=False, x=5.0)
        self.assertEqual(d.memory.tracks["object-0001"].position.tolist(), [1, 0, .1])
        self.assertEqual(d.memory.odometer, (0.0, 0.0))
        self.assertEqual(d.memory.tracks["object-0001"].reason, "pose_untrusted")

    def test_untrusted_frame_gives_no_negative_evidence(self):
        d = Driver(Config())
        d.step([("bottle", (1, 0, .1))])
        for k in range(4):
            d.step(trusted=False, geometry={"object-0001": "free"})
        t = d.memory.tracks["object-0001"]
        self.assertEqual(len(t.negative_poses), 0)
        self.assertNotEqual(t.state, "position_invalid")
        m = Memory("geometry", Config())
        m.tracks = d.memory.tracks
        f = Frame("g-1", 99.0, np.zeros((2, 2, 3), np.uint8), np.full((2, 2), np.nan, np.float32), K, np.eye(4))
        f.pose_trusted = False
        m.update_geometry_only({"object-0001": "free"}, f)
        self.assertEqual(t.reason, "pose_untrusted")
        self.assertEqual(len(t.negative_poses), 0)

    def test_gate_works_without_memory_quality(self):
        d = Driver(Config(memory_quality=False))
        d.step([("bottle", (1, 0, -0.5))], trusted=False)
        self.assertEqual(d.memory.tracks, {})

    def test_gate_off_ignores_the_verdict(self):
        on, off = Driver(), Driver(Config(pose_trust_gate=False))
        for drv, trusted in ((on, None), (off, False)):
            for k in range(3):
                drv.step([("bottle", (1 + .01 * k, 0, .1)), ("cup", (2, 0, .1))], trusted=trusted, x=.1 * k)
        self.assertEqual(on.memory.snapshot(), off.memory.snapshot())
        self.assertEqual(on.memory.rejected, off.memory.rejected)
        self.assertEqual(on.memory.odometer, off.memory.odometer)

    def test_invalid_pose_reason_unchanged(self):
        d = Driver()
        d.step([("bottle", (1, 0, .1))])
        d.step(pose_valid=False, trusted=False)
        self.assertEqual(d.memory.tracks["object-0001"].reason, "invalid_pose")

    def test_synthetic_scene_unchanged_without_verdict(self):
        snaps = []
        for cfg in (Config(), Config(pose_trust_gate=False)):
            m = Memory("geometry", cfg)
            for i in range(4):
                f, dets = scene(i)
                m.update_memory(process_frame(f, dets, cfg), compute_geometry(f, m), f)
            snaps.append(m.snapshot())
        self.assertEqual(snaps[0], snaps[1])


class RunnerEntryTests(unittest.TestCase):
    """host/ros_runtime.py helpers the Runtime node calls: Imu / Odometry messages in, frame verdict out."""

    @staticmethod
    def stamp(t):
        sec = int(math.floor(t))
        return SimpleNamespace(sec=sec, nanosec=int(round((t - sec) * 1e9)))

    def imu(self, t, wz):
        return SimpleNamespace(header=SimpleNamespace(stamp=self.stamp(t)), angular_velocity=SimpleNamespace(z=wz))

    def odom(self, t, yaw):
        q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
        return SimpleNamespace(header=SimpleNamespace(stamp=self.stamp(t)),
                               pose=SimpleNamespace(pose=SimpleNamespace(orientation=q)))

    def test_odometry_yaw(self):
        from host.ros_runtime import odometry_yaw
        for yaw in (-3.0, -0.5, 0.0, 1.2, 3.1):
            self.assertAlmostEqual(odometry_yaw(self.odom(0.0, yaw)), yaw, places=12)

    def test_messages_to_memory_write(self):
        from host.ros_runtime import feed_pose_trust, mark_pose_trust
        m = pt.PoseTrustMonitor()
        t0, wheel = 1792092960.0, 0.3
        k = 0
        while k <= 1200:                                     # 12 s at 100 Hz; 3.0-4.0 s gyro-only 80 deg/s
            t = t0 + k * 0.01
            wz = math.radians(80) if 300 < k <= 400 else 0.0
            feed_pose_trust(m, "imu", self.imu(t, wz))
            feed_pose_trust(m, "odom", self.odom(t, wheel))
            k += 1
        memory = Memory("geometry", Config())

        def write(t, x):
            f = Frame("f-%.2f" % t, t, np.zeros((2, 2, 3), np.uint8), np.full((2, 2), np.nan, np.float32), K,
                      np.eye(4))
            mark_pose_trust(m, f)
            o = Observation("bottle", 0.9, np.array([x, 0, .1]), np.array([x, 0, .1]) - .03,
                            np.array([x, 0, .1]) + .03, "red", 1.0, f.frame_id, t, (100, 100, 130, 200), "test",
                            extent_m=(0.06, 0.2))
            memory.update_memory([o], {}, f)
            return f.pose_trusted

        self.assertTrue(write(t0 + 2.0, 1.0))
        self.assertFalse(write(t0 + 3.5, 3.0))                   # during the gyro-only turn
        self.assertFalse(write(t0 + 5.0, 3.0))                   # settling
        self.assertTrue(write(t0 + 9.0, 1.01))                   # recovered
        self.assertEqual(memory.rejected, {"pose_untrusted": 2})
        # 10-27 identity_v2: the recovered write is the newest observation (the position is fused with 1.0)
        self.assertEqual([t.history[-1]["position_m"] for t in memory.tracks.values()], [[1.01, 0, .1]])
        f = Frame("off", t0 + 10, np.zeros((2, 2, 3), np.uint8), np.full((2, 2), np.nan, np.float32), K, np.eye(4))
        mark_pose_trust(None, f)                                 # --pose-trust off: attribute never set
        self.assertFalse(hasattr(f, "pose_trusted"))

    def test_runtime_marks_frames_right_before_the_memory_write(self):
        import ast
        src = (ROOT / "host/ros_runtime.py").read_text()
        lines = [ln.strip() for ln in src.splitlines()]
        # the one memory write; the trust mark is the last thing done to the frame before it (10-26 map-reset hold
        # check and 10-27 keep_ids sit in between: they read, never change, the frame)
        self.assertEqual(src.count("self.memory.update_memory("), 1)
        i = next(k for k, ln in enumerate(lines) if "self.memory.update_memory(observations, geometry, frame)" in ln)
        j = max(k for k in range(i) if lines[k] == "mark_pose_trust(self.pose_trust, frame)")
        self.assertLess(i - j, 12)
        between = lines[j + 1:i]
        self.assertFalse([ln for ln in between if ln.startswith("frame.") or "process_frame(" in ln
                          or "align_frame(" in ln], between)
        ast.parse(src)


class RealCarTests(unittest.TestCase):
    """Design doc section 4 on the compact stream + 20 Hz yaw fixture."""

    @classmethod
    def setUpClass(cls):
        from perception.fpga_replay import core
        from tools.memory_quality import pose_trust_replay as pr
        from tools.memory_quality import replay_stream as rs
        cls.pr, cls.rs = pr, rs
        gyro, wheel = pr.series_from_fixture(YAW)
        cls.monitor = pr.run_monitor(gyro, wheel)
        cls.episodes = pr.episodes(cls.monitor.transitions)
        K_, T_bc, image, cls.frames, _ = rs.load_stream(STREAM)
        cls.points = {"find1": 1792092486.9923933, "find2": 1792092813.9154556, "kidnap": 1792092965.0}
        cls.arms, cls.rej = {}, {}
        trust = cls.monitor.trusted_at
        for name, cfg, tr in (("new", Config(), None), ("legacy", Config(memory_quality=False), None),
                              ("new+trust", Config(), trust), ("legacy+trust", Config(memory_quality=False), trust),
                              ("gate_off+trust", Config(pose_trust_gate=False), trust)):
            cls.rej[name] = []
            cls.arms[name] = rs.run_arm(core, cfg, cls.frames, K_, T_bc, image, cls.points, True,
                                        trust=tr, rejections=cls.rej[name])
        crash = [e for e in cls.episodes if e[0] >= pr.CRASH_ONSET]
        cls.trip, cls.recovery = crash[0]

    def window(self, name):
        a, b = self.trip + self.pr.REL0, self.recovery + self.pr.REL0
        applied = [x for x in self.arms[name][2] if a <= x[0] < b]
        rejected = [why for t, why in self.rej[name] if a <= t < b]
        crash = sorted({x[1] for x in self.arms[name][2] if x[1] in self.pr.CRASH_TRACKS})
        return applied, rejected, crash

    def crash_segment_holds(self, name, quality=True):
        applied, rejected, crash = self.window(name)
        ok = not applied and not crash
        if quality:
            ok = ok and rejected.count("below_ground") == 0 and rejected.count("pose_untrusted") >= 25
        return ok

    def test_fixture_size_and_schema(self):
        self.assertLessEqual(YAW.stat().st_size, 1 << 20)
        self.assertEqual(len(self.episodes), 4)

    def test_t1_crash_trip_within_x(self):
        self.assertGreaterEqual(self.trip, 2128.0)
        self.assertLessEqual(self.trip, 2128.0 + self.pr.X_S)

    def test_t2_crash_segment_stops_writes(self):
        self.assertTrue(self.crash_segment_holds("new+trust"))
        self.assertTrue(self.crash_segment_holds("legacy+trust", quality=False))
        self.assertEqual(sum(1 for x in self.arms["legacy+trust"][2] if x[3][2] < -0.03), 0)

    def test_t2_negative_control_without_the_gate(self):
        for name in ("new", "gate_off+trust"):
            applied, rejected, _ = self.window(name)
            self.assertEqual(rejected.count("below_ground"), 25)              # only the ground gate catches them
            self.assertFalse(self.crash_segment_holds(name))
        applied, _, crash = self.window("legacy")
        self.assertGreaterEqual(len(applied), 25)
        self.assertEqual(crash, self.pr.CRASH_TRACKS)
        self.assertFalse(self.crash_segment_holds("legacy", quality=False))

    def test_t3_normal_segment_one_true_positive_only(self):
        normal = [e for e in self.episodes if e[0] <= self.pr.NORMAL_END]
        self.assertEqual(len(normal), 1)
        self.assertGreaterEqual(normal[0][0], self.pr.HAND_TURN[0])
        self.assertLessEqual(normal[0][0], self.pr.HAND_TURN[1])

    def test_t4_recovery(self):
        self.assertGreaterEqual(self.recovery, self.pr.CRASH_RECOVERY[0])
        self.assertLessEqual(self.recovery, self.pr.CRASH_RECOVERY[1])
        hand = self.episodes[0][1]
        self.assertGreaterEqual(hand, self.pr.HAND_RECOVERY[0])
        self.assertLessEqual(hand, self.pr.HAND_RECOVERY[1])
        after = [x for x in self.arms["new+trust"][2] if x[0] - self.pr.REL0 >= self.recovery]
        self.assertGreater(len(after), 0)
        milk = [x for x in self.arms["new+trust"][2] if hand <= x[0] - self.pr.REL0 < hand + 5]
        self.assertGreater(len(milk), 0)

    def test_t4_memory_quality_criteria_still_hold(self):
        ev = self.rs.evaluate({"new": self.arms["new+trust"]}, self.points["kidnap"], self.points["find1"], 0.03)["new"]
        self.assertLessEqual(len(ev["bottle_tracks_find1"]), 3)
        self.assertEqual(set(ev["find1_true_bottles"]), {"redcap", "nongfu"})
        self.assertEqual(ev["identity_full_run"]["bins"], [])
        for phys in ("milk", "nongfu", "redcap"):
            self.assertEqual(len(ev["identity_before_kidnap"][phys]), 1, phys)
        self.assertEqual(ev["mixed_tracks_before_kidnap"], {})
        self.assertEqual(ev["applied_below_tol"], 0)

    def test_gate_off_equals_no_verdict_on_real_stream(self):
        a, b = self.arms["new"], self.arms["gate_off+trust"]
        self.assertEqual(a[1]["end"], b[1]["end"])
        self.assertEqual(a[2], b[2])
        self.assertEqual(a[0].rejected, b[0].rejected)


if __name__ == "__main__":
    unittest.main()
