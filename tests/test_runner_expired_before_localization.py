"""Runner: a frame that expires in the queue only for want of its localization approval is skipped
(LocalizationSkipped, stall-diag reason `expired_before_localization`), not a clock/TF wait_expired
(2026-10-15 第 5 棒; controller ruling on owner's rule "the search program skips frames without
localization, no fault"; host g4r/g4r_re, HOST_CORRECTION_RESULT.md "C1 不过").

Same production Runtime + real correction-mode LocalizationGate harness as
test_runner_localization_skip.py (Drive). Cases:
1. a queued frame expires while the lease waits (gate valid, odometry fresh, no approval for it yet)
   -> skipped, no fault during navigation (positive control: e443313 faults
   acquisition_clock_or_tf_wait_expired on the same input);
2. the same expiry with odometry missing at the gate (status `stale`/odometry_stale), or with the
   static base<-camera extrinsic missing, or with the frame ahead of the ROS clock -> still the
   clock/TF branch (fault acquisition_clock_or_tf_wait_expired during navigation);
3. every frame expires that way for 2 s during navigation -> processed_stream_wall_timeout still;
4. the g4r_re timing: the first approved frame stalls ~1.3 s inside the runner (stale), the frame
   queued behind it was never tried and pops expired -> 0 wait_expired, idle and in navigation.
Python 3.8+.
"""
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

import test_runner_localization_skip as base  # noqa: E402

PREV = "e443313"           # gate-fix branch before this skip (第 4 棒 code under test)
WAIT_FAULT = "acquisition_clock_or_tf_wait_expired"
WAIT_WARNING = "Dropped acquisition after bounded wait for clock/TF"


class Sdiag:
    """Records the runner's stall-diag calls (the real RunnerStallDiag starts threads)."""

    out = None

    def __init__(self):
        self.calls = []

    def detect_wrapper(self, function):
        return function

    def clock(self):
        return 0

    def __getattr__(self, name):
        def call(*args, **kwargs):
            self.calls.append((name, args, kwargs))
        return call

    def dropped(self):
        """(stamp_ns, rx, reason) per acquisition_dropped call, in order"""
        return [a for name, a, _ in self.calls if name == "acquisition_dropped"]


class Drive(base.Drive):
    """base.Drive plus: gate-input gaps by stamp, image filter, per-pair extra delay, a runner
    stall (the gate keeps running during it; the runner's callbacks queued meanwhile are delivered
    after it, ROS KEEP_LAST: newest 10 statuses, newest 5 images, one timer tick), a stall-diag
    recorder, and hand-driven pieces. The clock never runs backwards (late = now)."""

    def __init__(self, source, name, **kw):
        super().__init__(source, name, **kw)
        self.sdiag = self.node.sdiag = Sdiag()
        self.gaps = []              # (groups, from_s, until_s): gate inputs stamped in [from, until) never come
        self.image_skip = None      # f(arrival_s, stamp_ns) -> True: this image is not delivered
        self.pair_extra = {}        # stamp_ns -> extra delay (s) of that odometry pair
        self.deferred = self.stalled = None
        self.flush_order = ("poll", "status", "image")
        self.plan, self.cursor = [], 0

    def clock(self, t):
        super().clock(max(t, self.world.wall))

    # -- gate side
    def gap(self, groups, start, until=math.inf):
        self.gaps.append((groups, start, until))

    def ingest(self, kind, row):
        st = row["header"]["stamp"]
        s = (st["sec"] * 10 ** 9 + st["nanosec"] - base.BASE) / 1e9
        group = "odom" if kind in ("odom", "odom_info") else "rtab"
        if any(group in g and a <= s < u for g, a, u in self.gaps):
            return
        super().ingest(kind, row)

    def publish_status(self):
        if self.deferred is not None:          # runner stalled: the gate publishes, nobody reads yet
            if self.t < self.gate_silent_from:
                self.deferred["status"].append(self.gate.status(self.world.wall, self.world.ros_ns))
            return
        super().publish_status()

    # -- schedule
    def events(self, start, end):
        out = []
        for e in super().events(start, end + 1.):
            t, prio, kind, stamp = e
            if kind == "image" and self.image_skip is not None and self.image_skip(t, stamp):
                continue
            if kind == "pair" and stamp in self.pair_extra:
                e = (t + self.pair_extra[stamp], prio, kind, stamp)
            out.append(e)
        return sorted(e for e in out if start <= e[0] < end)

    def run(self, start, end):
        self.plan, self.cursor = self.events(start, end), 0
        while self.cursor < len(self.plan):
            e = self.plan[self.cursor]
            self.cursor += 1
            self.dispatch(e)
            self.flush()

    def dispatch(self, e):
        t, _, kind, stamp = e
        self.clock(t)
        if self.deferred is not None and kind in ("image", "poll"):
            self.deferred[kind].append(stamp)
            return
        if kind == "pair":
            self.ingest("odom", {"header": base.header(stamp, "odom"), "child_frame_id": "camera_link",
                                 "pose": {"pose": base.odom_pose(stamp), "covariance": base.COV}})
            self.ingest("odom_info", {"header": base.header(stamp, "odom"), "lost": False, "covariance": base.COV})
        elif kind == "rtab":
            self.ref += 1
            self.ingest("info", {"header": base.header(stamp, "map"), "loop_closure_id": 1,
                                 "proximity_detection_id": 0, "ref_id": self.ref})
            self.ingest("localization_pose", {"header": base.header(stamp, "map"),
                                              "pose": {"pose": base.odom_pose(stamp), "covariance": base.COV}})
        elif kind == "image":
            self.world.image(stamp)
        elif kind == "heartbeat":
            self.publish_status()
        else:
            self.poll_once()

    def poll_once(self):
        self.world.poll("poll")
        if self.world.finish_detection():
            self.world.poll("poll_result")

    def stall(self, seconds):
        """Called from inside a runner callback: the scheduled events of the next `seconds` happen
        on the gate side, the runner's share waits; the clock ends `seconds` later."""
        end = self.world.wall + seconds
        self.deferred = {"status": [], "image": [], "poll": []}
        while self.cursor < len(self.plan) and self.plan[self.cursor][0] < end:
            e = self.plan[self.cursor]
            self.cursor += 1
            self.dispatch(e)
        self.clock(end)
        self.stalled, self.deferred = self.deferred, None

    def flush(self):
        q, self.stalled = self.stalled, None
        if not q:
            return
        for kind in self.flush_order:
            if kind == "poll" and q["poll"]:
                self.poll_once()
            elif kind == "status":
                for row in q["status"][-10:]:
                    self.world.deliver_status(row)
            elif kind == "image":
                for stamp in q["image"][-5:]:
                    self.world.image(stamp)

    def stall_at(self, stamp, seconds=1.3):
        """The frame `stamp` stalls `seconds` inside prepare_images once it has its approval
        (after token and matrix, at image_array: g4r_re's 1.3 s poll gap with GIL stall and
        main-thread CPU on the first approved frame), so it is stale at the age check."""
        glb = type(self.node).prepare_images.__globals__
        original = glb["image_array"]
        state = {"done": False}

        def image_array(message):
            st = message.header.stamp
            if not state["done"] and st.sec * 10 ** 9 + st.nanosec == stamp:
                state["done"] = True
                self.stall(seconds)
            return original(message)
        glb["image_array"] = image_array
        self.stall_restore = lambda: glb.__setitem__("image_array", original)
        return state

    def close(self):
        restore = getattr(self, "stall_restore", None)
        if restore:
            restore()
        super().close()

    # -- hand-driven pieces
    def heartbeat(self, t):
        self.clock(t)
        self.publish_status()

    def image(self, t, stamp):
        self.clock(t)
        self.world.image(stamp)

    def poll(self, t):
        self.clock(t)
        self.poll_once()

    def warnings(self):
        return [e["message"] for e in self.world.events if e["event"] == "warning"]

    @staticmethod
    def ab(t0):
        """A: the first non-odometry camera stamp after BASE + t0; B: the next non-odometry one."""
        k = int(t0 * 1e9 / base.CAM_NS) + 1
        while k % 3 == 0:
            k += 1
        j = k + 1
        while j % 3 == 0:
            j += 1
        return base.BASE + k * base.CAM_NS, base.BASE + j * base.CAM_NS

    def queue_behind_expiry(self, t0, b=None):
        """Frames A and B, 10 ms apart, stamped after any odometry the gate has (nothing can approve
        them yet); no gate input stamped in the window (< 1 s, so a valid gate stays valid);
        statuses at 10 Hz keep the lease alive and retry A, which waits for its approval; no poll
        until B is 0.54 s old. A expires at a status-triggered try (no_localization_for_frame),
        B pops already expired and was never tried. Returns B's stamp."""
        a, b_default = self.ab(t0)
        b = b_default if b is None else b
        wa = (a - base.BASE) / 1e9 + .002
        self.gap(("odom", "rtab"), t0, wa + .56)
        self.image(wa, a)
        self.image(wa + .010, b)
        for i in range(1, 6):
            self.heartbeat(wa + .02 + .1 * i)        # last at wa + .52: A expired there
        self.poll(wa + .55)                          # B: 0.54 s old, never tried
        return b


def first_valid():
    """(arrival time, approved stamp) of the gate's first valid status on the default schedule;
    the gate does not depend on the runner, so any Drive gives the same answer."""
    d = Drive(base.current_source(), "exp_first_valid")
    seen = []
    deliver = d.world.deliver_status

    def hook(row):
        if row["state"] == "valid" and not seen:
            seen.append((d.t, row["approved"][-1]["stamp_ns"]))
        deliver(row)
    d.world.deliver_status = hook
    try:
        d.run(0., 2.)
    finally:
        d.close()
    return seen[0]


def current():
    return base.current_source()


def probe_runtime_fault_run(d):
    # the runtime module is loaded per case under its own name: read the constant from the node's globals
    return type(d.node).__init__.__globals__["TF_EXPIRY_FAULT_RUN"]


class ExpiredBeforeLocalization(unittest.TestCase):
    def make(self, source, name, navigate=True):
        d = Drive(source, name)
        self.addCleanup(d.close)
        if navigate:
            d.warm_and_navigate()
        else:
            d.run(0., 1.6)
            self.assertEqual(d.gate.state, "valid")
        return d

    def reason_of(self, d, stamp):
        rows = [r for r in d.sdiag.dropped() if r[0] == stamp]
        self.assertEqual(len(rows), 1, rows)
        return rows[0][2]

    def faults(self, d):
        return [r for _, r in d.faults()]

    # ---- 1. queued frame expires while the lease waits -> skipped
    def test_queued_frame_expiring_while_lease_waits_is_skipped_in_navigation(self):
        d = self.make(current(), "exp_wait_nav")
        d.run(d.t + 1e-6, d.nav_start + .5)
        skipped = d.node.localization_skipped
        b = d.queue_behind_expiry(d.t + .05)
        self.assertEqual(self.reason_of(d, b), "expired_before_localization")
        self.assertEqual(d.node.localization_skipped - skipped, 2)       # A (waited) + B (untried)
        self.assertEqual(d.faults(), [])
        self.assertTrue(d.node.nav and not d.node.nav["cancelled"])
        self.assertEqual(d.gate.state, "valid")
        self.assertNotIn(WAIT_WARNING, d.warnings())
        processed = d.node.processed                  # the run goes on: frames processed, no fault
        d.run(d.t + 1e-6, d.t + 1.)
        self.assertGreater(d.node.processed, processed)
        self.assertEqual(d.faults(), [])

    @unittest.skipIf(base.source_at(PREV) is None, "git history with %s not available" % PREV)
    def test_positive_control_previous_runner_faults_on_the_same_input(self):
        d = self.make(base.source_at(PREV), "exp_wait_prev")
        d.run(d.t + 1e-6, d.nav_start + .5)
        b = d.queue_behind_expiry(d.t + .05)
        self.assertEqual(self.reason_of(d, b), "wait_expired")
        self.assertIn(WAIT_FAULT, self.faults(d))

    # ---- 2. odometry missing at the gate / static extrinsic missing / clock -> clock/TF branch
    def odometry_missing_at_the_gate(self, source, name):
        d = self.make(source, name)
        d.run(d.t + 1e-6, d.nav_start + .5)
        d.gap(("odom",), d.t)                           # odometry stops reaching the gate
        d.run(d.t + 1e-6, d.t + 1.3)                    # rtabmap outputs come without their odometry
        self.assertEqual((d.gate.state, d.gate.reason), ("lost", "incomplete_output"))
        b = d.queue_behind_expiry(d.t + .05)
        return d, self.reason_of(d, b)

    def test_incomplete_output_at_the_gate_now_skipped(self):
        """2026-10-16 第 5 棒收口 (controller ruling): lost:incomplete_output is "no localization",
        so the frame behind is skipped; navigation is still stopped by the lease
        (localization_invalid), as before. Real gate, no status rewrite."""
        d, got = self.odometry_missing_at_the_gate(current(), "exp_odom_missing")
        self.assertEqual(got, "expired_before_localization")
        reasons = self.faults(d)
        self.assertNotIn(WAIT_FAULT, reasons)
        self.assertIn("localization_invalid", reasons)  # the lease path faulted first, unchanged

    @unittest.skipIf(base.source_at("fd7608a") is None, "git history with fd7608a not available")
    def test_incomplete_output_at_the_gate_control_fd7608a_faults(self):
        d, got = self.odometry_missing_at_the_gate(base.source_at("fd7608a"), "exp_odom_missing_5")
        self.assertEqual(got, "wait_expired")
        reasons = self.faults(d)
        self.assertIn(WAIT_FAULT, reasons)
        self.assertIn("localization_invalid", reasons)

    def test_odometry_stale_idle_still_warns_wait_expired(self):
        d = self.make(current(), "exp_odom_stale_idle", navigate=False)
        d.gap(("odom", "rtab"), d.t)                    # odometry (so rtabmap too) stops
        d.run(d.t + 1e-6, d.t + 1.3)                    # > 1 s: gate revokes odometry_stale
        self.assertEqual((d.gate.state, d.gate.reason), ("stale", "odometry_stale"))
        b = d.queue_behind_expiry(d.t + .05)
        self.assertEqual(self.reason_of(d, b), "wait_expired")
        self.assertIn(WAIT_WARNING, d.warnings())

    def test_static_extrinsic_missing_still_faults_wait_expired(self):
        d = self.make(current(), "exp_tf_missing")
        d.run(d.t + 1e-6, d.nav_start + .5)
        t0 = d.t + .05
        _, b = d.ab(t0)
        lookup = d.node.buffer.lookup_transform

        def missing(target, source, stamp):
            if target == "base_link" and stamp.nanoseconds == b:
                raise d.probe.TransformException("base_link<-camera_optical_frame absent at B")
            return lookup(target, source, stamp)
        d.node.buffer.lookup_transform = missing
        # 10-26: a task faults only on the TF_EXPIRY_FAULT_RUN-th consecutive expiry (one lost /tf sample is not fatal)
        d.node.tf_expiry_run = probe_runtime_fault_run(d) - 1
        self.assertEqual(d.queue_behind_expiry(t0), b)
        self.assertEqual(self.reason_of(d, b), "wait_expired")
        self.assertIn(WAIT_FAULT, self.faults(d))

    def test_single_tf_expiry_in_navigation_is_dropped_not_faulted(self):
        d = self.make(current(), "exp_tf_single")
        d.run(d.t + 1e-6, d.nav_start + .5)
        t0 = d.t + .05
        _, b = d.ab(t0)
        lookup = d.node.buffer.lookup_transform

        def missing(target, source, stamp):
            if target == "base_link" and stamp.nanoseconds == b:
                raise d.probe.TransformException("base_link<-camera_optical_frame absent at B")
            return lookup(target, source, stamp)
        d.node.buffer.lookup_transform = missing
        self.assertEqual(d.queue_behind_expiry(t0), b)
        self.assertEqual(self.reason_of(d, b), "wait_expired")
        self.assertNotIn(WAIT_FAULT, self.faults(d))
        self.assertEqual(d.node.tf_expiry_run, 1)
        self.assertTrue(d.node.nav and not d.node.nav["cancelled"])

    def test_frame_ahead_of_ros_clock_still_faults_wait_expired(self):
        d = self.make(current(), "exp_clock")
        d.run(d.t + 1e-6, d.nav_start + .5)
        t0 = d.t + .05
        a, _ = d.ab(t0)
        future = a + 3 * 10 ** 9                        # B stamped 3 s ahead of the ROS clock
        d.node.tf_expiry_run = probe_runtime_fault_run(d) - 1   # 10-26: the run-th consecutive expiry faults
        d.queue_behind_expiry(t0, b=future)
        self.assertEqual(self.reason_of(d, future), "wait_expired")
        self.assertIn(WAIT_FAULT, self.faults(d))

    # ---- 3. the 2 s processed-stream backstop is not eaten by the new skip
    def test_two_seconds_without_a_processed_frame_still_faults(self):
        d = self.make(current(), "exp_backstop")
        d.run(d.t + 1e-6, d.nav_start + .3)
        last, processed = d.node.last_processed_wall, d.node.processed
        d.image_skip = lambda t, stamp: True            # only the hand-driven A/B frames from now on
        for _ in range(4):                              # 0.5 s gate input, then 0.6 s A/B window
            d.run(d.t + 1e-6, d.t + .5)
            if d.faults():
                break
            d.queue_behind_expiry(d.t + .01)
            if d.faults():
                break
        self.assertEqual(d.node.processed, processed)
        reasons = self.faults(d)
        self.assertEqual(reasons[:1], ["processed_stream_wall_timeout"])
        self.assertNotIn(WAIT_FAULT, reasons)
        wall = d.faults()[0][0]
        self.assertGreater(wall - last, 2.)
        self.assertLessEqual(wall - last, 2.7)
        self.assertGreaterEqual(sum(r[2] == "expired_before_localization" for r in d.sdiag.dropped()), 2)
        self.assertEqual(sum(r[2] == "wait_expired" for r in d.sdiag.dropped()), 0)

    # ---- 4. the g4r/g4r_re timing
    def g4r_entry(self, source, name, order):
        """g4r_re: the gate's first valid status approves frame S; the runner's try of S stalls
        1.3 s (stale), and frame S+1, queued behind S and never tried, pops expired. Images are
        held back for 0.6 s before the entry so S is at the head (the frames before it expired)."""
        t_entry, s = first_valid()
        b = s + base.CAM_NS
        self.assertLess((b - base.BASE) / 1e9 + .005, t_entry)     # S+1 queued before the entry
        d = Drive(source, name)
        self.addCleanup(d.close)
        d.flush_order = order
        d.image_skip = lambda t, stamp: t_entry - .6 <= t < t_entry + .001 and stamp not in (s, b)
        state = d.stall_at(s)
        d.run(0., 4.)
        self.assertTrue(state["done"])
        return d, s, b

    def test_g4r_first_approved_frame_stall_idle_no_wait_expired(self):
        for order in (("poll", "status", "image"), ("status", "image", "poll"), ("image", "status", "poll")):
            with self.subTest(order=order):
                d, s, b = self.g4r_entry(current(), "exp_g4r_idle_" + order[0], order)
                # stale after the stall: skipped since 第 5 棒续 (f4a273f: invalid:ValueError,
                # test_runner_skip_by_reason.py keeps that control)
                self.assertEqual(self.reason_of(d, s), "stale_after_approval")
                self.assertEqual(self.reason_of(d, b), "expired_before_localization")
                self.assertNotIn("wait_expired", [r[2] for r in d.sdiag.dropped()])
                self.assertNotIn(WAIT_WARNING, d.warnings())
                self.assertEqual(d.faults(), [])
                self.assertGreater(d.node.processed, 5)                           # and it goes on

    @unittest.skipIf(base.source_at(PREV) is None, "git history with %s not available" % PREV)
    def test_g4r_positive_control_previous_runner_wait_expired(self):
        d, s, b = self.g4r_entry(base.source_at(PREV), "exp_g4r_prev", ("poll", "status", "image"))
        self.assertEqual(self.reason_of(d, s), "invalid:ValueError")
        self.assertEqual(self.reason_of(d, b), "wait_expired")
        self.assertEqual([r[2] for r in d.sdiag.dropped()].count("wait_expired"), 1)
        self.assertEqual(d.warnings().count(WAIT_WARNING), 1)

    def test_g4r_stall_during_navigation_no_wait_expired_fault(self):
        # Same shape while navigating: odometry frame A approved, A+1 queued behind it, A stalls.
        # 第 5 棒续: the stale A is skipped (stale_after_approval), no sensor_or_pose_invalid
        # (f4a273f faulted it; control in test_runner_skip_by_reason.py); with the timer first the
        # lease is > 0.5 s without a status (lease backstop, unchanged).
        for order in (("poll", "status", "image"), ("status", "image", "poll")):
            with self.subTest(order=order):
                d = Drive(current(), "exp_g4r_nav_" + order[0])
                self.addCleanup(d.close)
                d.flush_order = order
                d.warm_and_navigate()
                k = int((d.t + .3) * 1e9 / base.CAM_NS) + 1
                k += (-k) % 3                                  # an odometry stamp
                a, b = base.BASE + k * base.CAM_NS, base.BASE + (k + 1) * base.CAM_NS
                ta = (a - base.BASE) / 1e9
                d.image_skip = lambda t, stamp: ta - .2 <= t < ta + .04 and stamp not in (a, b)
                d.pair_extra = {a: .015}                       # A's approval after A+1 is queued
                state = d.stall_at(a)
                d.run(d.t + 1e-6, d.t + 1.5)
                self.assertTrue(state["done"])
                self.assertEqual(self.reason_of(d, a), "stale_after_approval")
                self.assertEqual(self.reason_of(d, b), "expired_before_localization")
                self.assertNotIn("wait_expired", [r[2] for r in d.sdiag.dropped()])
                reasons = self.faults(d)
                self.assertNotIn(WAIT_FAULT, reasons)
                self.assertFalse([r for r in reasons if r.startswith("sensor_or_pose_invalid")], reasons)
                self.assertTrue(all(r.startswith("localization_invalid") for r in reasons), reasons)


class SourceShape(unittest.TestCase):
    def test_new_label_kept_apart_from_the_c1_label(self):
        src = base.current_source()
        self.assertIn('"expired_before_localization"', src)
        self.assertEqual(src.count('"wait_expired"'), 1)
        self.assertNotIn("wait_expired", "expired_before_localization")


if __name__ == "__main__":
    unittest.main()
