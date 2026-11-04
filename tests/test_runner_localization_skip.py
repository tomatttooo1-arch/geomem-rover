"""Runner: an acquisition the localization lease moved past without approving is skipped, not waited
for (2026-10-15 evening, with the gate's correction mode; GATEFIX_HANDOVER.md 第 2 棒 B/C).

The production Runtime class (estimated-pose branch) runs on the scripted world of
tools/rtabmap_runtime_probe_support, loaded through test_region_source_none_regression's
3.8-compatible loader, fed by a REAL LocalizationGate in correction mode whose every status row
goes to Runtime.on_localization. Camera 30 fps, odometry 10 Hz (every third camera stamp, pair
30 ms late), rtabmap outputs ~3.3 Hz (200 ms late), gate heartbeat 10 Hz, runner poll 10 Hz.

1. Navigation for 3 s on that input: no acquisition_clock_or_tf_wait_expired, no fault at all,
   frames processed, frames skipped (localization_skipped > 0, counted apart from the rest).
   Positive control: the runner of 0902bef on the same input faults with
   acquisition_clock_or_tf_wait_expired (RELAX_EVAL §0).
2. Backstops (no new timer; controller decision): the gate stops publishing ->
   localization_invalid_or_timeout within 1 s; the lease stays healthy but no camera stamp ever
   matches an approval -> processed_stream_wall_timeout at 2 s.
3. The `if token is None:` single-raise branch that tools/rtabmap_runtime_probe_support's
   candidate policy matches is still there exactly once (3.9+: the real candidate loader runs).
Python 3.8+.
"""
import ast
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from host.localization_gate import LocalizationGate, matrix_pose  # noqa: E402

OLD = "0902bef"            # gate-fix branch before the skip
MS = 10 ** 6
BASE = 10 ** 9             # ROS ns at wall 0
CAM_NS = 33333333          # 30 fps
COV = [0.01 if i % 7 == 0 else 0. for i in range(36)]


def source_at(rev):
    try:
        return subprocess.check_output(["git", "show", "%s:host/ros_runtime.py" % rev], cwd=str(ROOT),
                                       stderr=subprocess.DEVNULL).decode("utf-8")
    except (OSError, subprocess.CalledProcessError):
        return None


def current_source():
    return (ROOT / "host" / "ros_runtime.py").read_text(encoding="utf-8")


def header(stamp, frame):
    return {"stamp": {"sec": stamp // 10 ** 9, "nanosec": stamp % 10 ** 9}, "frame_id": frame}


def odom_pose(stamp):
    m = np.eye(4)
    m[0, 3] = .05 * (stamp - BASE) / 1e9          # slow forward motion
    return matrix_pose(m)


class Drive:
    """One Runtime + one correction-mode gate on a common scripted clock."""

    def __init__(self, source, name, cam_offset_ns=0):
        from tools import rtabmap_runtime_probe_support as probe
        import test_region_source_none_regression as reg
        from types import SimpleNamespace as NS
        self.probe = probe
        self.tmp = tempfile.TemporaryDirectory()
        module = reg.module_from_source(source, name)
        try:
            self.world = probe.World("original")
            self.node = reg.make_loader(source, module)(self.world, Path(self.tmp.name) / "diag.json")
        finally:
            sys.modules.pop(name, None)
        self.world.node = self.node
        n = self.node
        n.on_static_tf(NS(transforms=[probe.transform("base_link", "camera_link", (.2, -.1, .3), np.pi / 2),
                                      probe.transform("camera_link", "camera_optical_frame", (.1, .2, -.05),
                                                      -np.pi / 4)]))
        n.on_map(NS(info=NS(width=40, height=40, resolution=.5,
                            origin=NS(position=NS(x=-10., y=-10., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))),
                    header=NS(frame_id="map"), data=[0] * (40 * 40)))
        self.gate = LocalizationGate("a" * 64, {1}, tracked_frame="camera_link", count_mode="correction")
        self.cam_offset_ns = cam_offset_ns
        self.ref = 0
        self.gate_silent_from = math.inf
        self.status_walls = []
        self.t = 0.

    # -- clock and gate
    def clock(self, t):
        self.t = t
        self.world.clock(t, BASE + int(round(t * 1e9)))

    def publish_status(self):
        if self.t >= self.gate_silent_from:
            return
        row = self.gate.status(self.world.wall, self.world.ros_ns)
        self.status_walls.append(self.t)
        self.world.deliver_status(row)

    def ingest(self, kind, row):
        self.gate.ingest(kind, row, self.world.wall, self.world.ros_ns)
        self.publish_status()                                  # the node's heartbeat(clock) per receive

    # -- schedule
    def events(self, start, end):
        out = []
        k0 = int(math.ceil(start * 1e9 / CAM_NS))
        for k in range(k0, int(end * 1e9 / CAM_NS) + 1):
            stamp = BASE + k * CAM_NS
            if k % 3 == 0:
                out.append(((stamp - BASE) / 1e9 + .030, 0, "pair", stamp))
                if k % 9 == 0:
                    out.append(((stamp - BASE) / 1e9 + .200, 1, "rtab", stamp))
            out.append(((stamp - BASE) / 1e9 + .005, 2, "image", stamp + self.cam_offset_ns))
        t = math.ceil(start * 10) / 10.
        while t < end:
            out.append((t, 3, "heartbeat", None))
            out.append((t + .05, 4, "poll", None))
            t = round(t + .1, 6)
        return sorted(e for e in out if start <= e[0] < end)

    def run(self, start, end):
        for t, _, kind, stamp in self.events(start, end):
            self.clock(t)
            if kind == "pair":
                self.ingest("odom", {"header": header(stamp, "odom"), "child_frame_id": "camera_link",
                                     "pose": {"pose": odom_pose(stamp), "covariance": COV}})
                self.ingest("odom_info", {"header": header(stamp, "odom"), "lost": False, "covariance": COV})
            elif kind == "rtab":
                self.ref += 1
                self.ingest("info", {"header": header(stamp, "map"), "loop_closure_id": 1,
                                     "proximity_detection_id": 0, "ref_id": self.ref})
                self.ingest("localization_pose", {"header": header(stamp, "map"),
                                                  "pose": {"pose": odom_pose(stamp), "covariance": COV}})
            elif kind == "image":
                self.world.image(stamp)
            elif kind == "heartbeat":
                self.publish_status()
            else:
                self.world.poll("poll")
                if self.world.finish_detection():
                    self.world.poll("poll_result")

    def warm_and_navigate(self):
        from host.task_control import PoseGoal
        self.run(0., 1.6)
        assert self.gate.state == "valid", (self.gate.state, self.gate.reason)
        assert self.node.processed > 0, "no frame processed while warming"
        assert self.node.ready()
        self.node.navigate(PoseGoal(.5, .2, .1), None)
        self.node.actions.request.set_result(self.world.goal)
        self.world.poll("navigation_armed")
        assert self.node.nav and not self.node.nav["cancelled"]
        self.nav_start = self.t

    def faults(self):
        return [(e["wall"], e["payload"]["reason"]) for e in self.world.events
                if e["event"] == "publish" and e.get("topic") == "/fpga/status"
                and e["payload"].get("event") == "fault"]

    def close(self):
        try:
            self.node.close()
        finally:
            self.tmp.cleanup()


class SkipTests(unittest.TestCase):
    def drive(self, source, name, **kw):
        d = Drive(source, name, **kw)
        self.addCleanup(d.close)
        d.warm_and_navigate()
        return d

    def test_navigation_on_correction_mode_runs_without_wait_expired(self):
        d = self.drive(current_source(), "runtime_skip_current")
        processed, skipped = d.node.processed, d.node.localization_skipped
        d.run(d.t + 1e-6, d.nav_start + 3.)
        self.assertEqual(d.faults(), [])
        self.assertTrue(d.node.nav and not d.node.nav["cancelled"])
        # every odometry stamp is a camera stamp (10 Hz); this harness completes inference at
        # the 10 Hz poll only, so ~8 frames/s are processed
        self.assertGreaterEqual(d.node.processed - processed, 20)
        # frames between odometry stamps: skipped (the rest are replaced in the 2-slot queue)
        self.assertGreaterEqual(d.node.localization_skipped - skipped, 15)
        self.assertGreaterEqual(d.node.dropped, d.node.localization_skipped)
        self.assertEqual([e for e in d.world.events if e["event"] == "warning"], [])

    @unittest.skipIf(source_at(OLD) is None, "git history with %s not available" % OLD)
    def test_positive_control_old_runner_waits_out_every_frame(self):
        # Same input, runner of 0902bef: the frame at the head of the queue waits for an approval
        # that never comes and is dropped at the 0.5 s bound. Idle, that is this warning; during a
        # task or navigation the same branch is the fault acquisition_clock_or_tf_wait_expired, so
        # the old runner never becomes ready to navigate here (0 frames processed).
        d = Drive(source_at(OLD), "runtime_skip_old")
        self.addCleanup(d.close)
        d.run(0., 3.)
        warnings = [e["message"] for e in d.world.events if e["event"] == "warning"]
        self.assertGreaterEqual(warnings.count("Dropped acquisition after bounded wait for clock/TF"), 4)
        self.assertEqual(d.node.processed, 0)
        self.assertFalse(d.node.ready())
        new = Drive(current_source(), "runtime_skip_new_idle")
        self.addCleanup(new.close)
        new.run(0., 3.)
        self.assertEqual([e for e in new.world.events if e["event"] == "warning"], [])
        self.assertGreater(new.node.processed, 10)

    def test_backstop_gate_silent_lease_fault_within_one_second(self):
        d = self.drive(current_source(), "runtime_skip_silent")
        d.run(d.t + 1e-6, d.nav_start + 1.)
        self.assertEqual(d.faults(), [])
        d.gate_silent_from = d.t + 1e-6
        d.run(d.t + 1e-6, d.t + 2.)
        faults = d.faults()
        self.assertTrue(faults)
        wall, reason = faults[0]
        self.assertEqual(reason, "localization_invalid_or_timeout")
        self.assertLessEqual(wall - d.status_walls[-1], 1.)

    def test_backstop_no_frame_ever_approved_processed_stream_timeout(self):
        d = self.drive(current_source(), "runtime_skip_offset")
        d.cam_offset_ns = 7 * MS              # from now on no camera stamp equals an odometry stamp
        last_processed = d.node.last_processed_wall
        d.run(d.t + 1e-6, d.t + 3.)
        faults = d.faults()
        self.assertTrue(faults)
        wall, reason = faults[0]
        self.assertEqual(reason, "processed_stream_wall_timeout")
        self.assertLessEqual(wall - last_processed, 2.2)
        self.assertTrue(d.gate.state == "valid" and d.node.localization.row is not None)   # lease healthy


class SourceShapeTests(unittest.TestCase):
    source = current_source()

    def prepare(self):
        tree = ast.parse(self.source)
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
        klass = next(n for n in main.body if isinstance(n, ast.ClassDef) and n.name == "Runtime")
        return next(n for n in klass.body if isinstance(n, ast.FunctionDef) and n.name == "prepare_images")

    def test_probe_matched_branch_kept_and_skip_placed_before_it(self):
        method = self.prepare()
        ifs = [n for n in ast.walk(method) if isinstance(n, ast.If)]
        plain = [n for n in ifs if isinstance(n.test, ast.Compare) and len(n.body) == 1
                 and isinstance(n.body[0], ast.Raise)
                 and ast.dump(n.test) == ast.dump(ast.parse("token is None", mode="eval").body)]
        self.assertEqual(len(plain), 1)
        skip = [n for n in ifs if isinstance(n.test, ast.BoolOp) and "superseded" in ast.dump(n.test)]
        self.assertEqual(len(skip), 1)
        self.assertLess(skip[0].lineno, plain[0].lineno)
        self.assertIn("LocalizationSkipped", ast.dump(skip[0].body[0]))

    def test_skip_handler_precedes_the_generic_handler(self):
        method = self.prepare()
        trys = [n for n in ast.walk(method) if isinstance(n, ast.Try) and len(n.handlers) >= 3]
        self.assertEqual(len(trys), 1)
        names = [ast.dump(h.type) for h in trys[0].handlers]
        skip = next(i for i, d in enumerate(names) if "LocalizationSkipped" in d)
        generic = next(i for i, d in enumerate(names) if "'Exception'" in d)
        self.assertLess(skip, generic)

    @unittest.skipIf(sys.version_info < (3, 9), "probe loader needs ast.unparse (3.9+, D-028)")
    def test_probe_candidate_loader_still_patches_exactly_one_branch(self):
        from tools import rtabmap_runtime_probe_support as probe
        with tempfile.TemporaryDirectory() as tmp:
            for policy in ("original", "candidate"):
                world = probe.World(policy)
                node = probe.load_runtime(world, Path(tmp) / ("%s.json" % policy))
                loader = [e for e in world.events if e["event"] == "loader"][-1]
                self.assertEqual(loader["candidate_branch_insertions"], int(policy == "candidate"))
                node.close()


if __name__ == "__main__":
    unittest.main()
