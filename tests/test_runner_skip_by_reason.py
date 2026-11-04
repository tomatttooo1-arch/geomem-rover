"""Runner: frames without localization are skipped by the gate's status REASON, not its state
(2026-10-15 第 5 棒续; controller ruling on owner's rule "the search program skips frames without
localization, no fault; backstops: lease void in 1 s, no processed frame for 2 s").

Skip (stall-diag `expired_before_localization` / `stale_after_approval`, counted in
localization_skipped, no fault): waiting (valid, lease kept it), not ready (initializing /
confirming, not clock), no correction (stale:correction_too_old, lost:correction_jump, and since
2026-10-16 第 5 棒收口 lost:incomplete_output, stale:localization_stream_timeout).
Fault as before: odometry (odometry_stale, odom/odom_info lost or bad), clock (clock_rollback,
replay_discontinuity, future_stamp, invalid_discontinuity_request), every reason the ruling does not
name (incomplete_acquisition_timeout, rtabmap_output_gap ...), and any reason not in the whitelist.

Every case runs the same driver on the current runner, on f4a273f (第 5 棒, by state) and on
fd7608a (第 5 棒续, by reason before the two reasons moved) as controls: the cases that change (no correction; first stale approved frame; unknown valid /
initializing reasons) differ between the two, the rest are the same on both ("unchanged").
Same production Runtime + real correction-mode LocalizationGate harness as
test_runner_expired_before_localization.py. Python 3.8+.
"""
import ast
import fnmatch
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

import test_runner_localization_skip as base  # noqa: E402
import test_runner_expired_before_localization as ebl  # noqa: E402
from host import ros_runtime  # noqa: E402

PREV = "f4a273f"           # 第 5 棒: skip decided by state
HAVE_PREV = base.source_at(PREV) is not None
PREV5 = "fd7608a"          # 第 5 棒续: by reason, incomplete_output / localization_stream_timeout fault
HAVE_PREV5 = base.source_at(PREV5) is not None
MOVED = {("lost", "incomplete_output"), ("stale", "localization_stream_timeout")}  # 第 5 棒收口
WAIT_FAULT = ebl.WAIT_FAULT
WAIT_WARNING = ebl.WAIT_WARNING
SKIP, EXPIRED = "expired_before_localization", "wait_expired"


class Drive(ebl.Drive):
    """ebl.Drive plus a status rewrite: every status the runner gets has its state/reason replaced
    (a non-valid row carries no approval, as the gate's own do)."""

    rewrite = None

    def publish_status(self):
        if self.rewrite is None or self.deferred is not None:
            return super().publish_status()
        if self.t >= self.gate_silent_from:
            return
        row = self.gate.status(self.world.wall, self.world.ros_ns)
        state, reason = self.rewrite
        if state != "valid":
            row["approved"], row["correction_stamp_ns"] = [], 0
        row["state"], row["reason"] = state, reason
        self.status_walls.append(self.t)
        self.world.deliver_status(row)


# (state, reason, current label, f4a273f label); the gate itself stays valid underneath
CASES = [
    # waiting / not ready: skipped on both (unchanged)
    ("valid", "three_consistent_corrections", SKIP, SKIP),
    ("valid", "correction_refreshed", SKIP, SKIP),
    ("initializing", "awaiting_acquisition", SKIP, SKIP),
    ("initializing", "entry_reset:unlinked", SKIP, SKIP),
    ("confirming", "confirming_corrections", SKIP, SKIP),
    ("confirming", "confirming_after_correction_jump", SKIP, SKIP),
    # no correction: skipped now, wait_expired on f4a273f (the change)
    ("stale", "correction_too_old", SKIP, EXPIRED),
    ("lost", "correction_jump", SKIP, EXPIRED),
    ("lost", "incomplete_output", SKIP, EXPIRED),            # 第 5 棒收口
    ("stale", "localization_stream_timeout", SKIP, EXPIRED),  # 第 5 棒收口
    # odometry: fault on both (unchanged)
    ("stale", "odometry_stale", EXPIRED, EXPIRED),
    ("lost", "odom_info:odometry_lost", EXPIRED, EXPIRED),
    ("lost", "odom:invalid_covariance", EXPIRED, EXPIRED),
    # clock: fault on both (unchanged)
    ("initializing", "clock_rollback", EXPIRED, EXPIRED),
    ("initializing", "replay_discontinuity:1", EXPIRED, EXPIRED),
    ("lost", "future_stamp", EXPIRED, EXPIRED),
    ("lost", "invalid_discontinuity_request", EXPIRED, EXPIRED),
    # not named by the ruling: fault on both (unchanged)
    ("stale", "incomplete_acquisition_timeout", EXPIRED, EXPIRED),
    ("lost", "rtabmap_output_gap", EXPIRED, EXPIRED),
    # not in the whitelist: fault now; f4a273f skipped any valid / non-clock initializing
    ("valid", "some_future_reason", EXPIRED, SKIP),
    ("initializing", "some_future_reason", EXPIRED, SKIP),
]


def label_at_prev5(state, reason, now):
    """fd7608a label: same as now except the two reasons moved to skip in 第 5 棒收口."""
    return EXPIRED if (state, reason) in MOVED else now


def labels(d):
    return [r[2] for r in d.sdiag.dropped()]


class ByReason(unittest.TestCase):
    def make(self, source, name, navigate=True):
        d = Drive(source, name)
        self.addCleanup(d.close)
        if navigate:
            d.warm_and_navigate()
        else:
            d.run(0., 1.6)
            self.assertEqual(d.gate.state, "valid")
        return d

    def expire_behind(self, source, name, state, reason, navigate):
        d = self.make(source, name, navigate)
        d.run(d.t + 1e-6, (d.nav_start if navigate else d.t) + .5)
        d.rewrite = (state, reason)
        b = d.queue_behind_expiry(d.t + .05)
        rows = [r for r in d.sdiag.dropped() if r[0] == b]
        self.assertEqual(len(rows), 1, rows)
        return d, rows[0][2]

    def check(self, d, label, navigate):
        reasons = [r for _, r in d.faults()]
        if label == SKIP:
            self.assertNotIn(WAIT_FAULT, reasons)
            self.assertNotIn(WAIT_WARNING, d.warnings())
        elif navigate:
            self.assertIn(WAIT_FAULT, reasons)
        else:
            self.assertIn(WAIT_WARNING, d.warnings())

    def run_cases(self, navigate):
        for state, reason, now, before in CASES:
            with self.subTest(state=state, reason=reason):
                d, got = self.expire_behind(base.current_source(), "rsn_" + state, state, reason, navigate)
                self.assertEqual(got, now)
                self.check(d, got, navigate)
                if navigate and state != "valid":
                    # the status itself already stopped navigation (lease), as before
                    self.assertIn("localization_invalid", [r for _, r in d.faults()])
            if HAVE_PREV:
                with self.subTest(state=state, reason=reason, control=PREV):
                    d, got = self.expire_behind(base.source_at(PREV), "rsn_prev_" + state, state, reason, navigate)
                    self.assertEqual(got, before)
                    self.check(d, got, navigate)
            if HAVE_PREV5:
                with self.subTest(state=state, reason=reason, control=PREV5):
                    d, got = self.expire_behind(base.source_at(PREV5), "rsn_prev5_" + state, state, reason,
                                                navigate)
                    self.assertEqual(got, label_at_prev5(state, reason, now))
                    self.check(d, got, navigate)

    def test_every_class_navigation_current_and_control(self):
        self.run_cases(navigate=True)

    def test_every_class_idle_current_and_control(self):
        self.run_cases(navigate=False)

    def test_changed_cases_really_differ_from_the_control(self):
        # the table carries discriminating cases in both directions, so the control can fail
        self.assertTrue(any(now == SKIP and before == EXPIRED for _, _, now, before in CASES))
        self.assertTrue(any(now == EXPIRED and before == SKIP for _, _, now, before in CASES))
        # 第 5 棒收口: every moved reason has a case, and fd7608a labels it differently
        self.assertEqual(MOVED, {(s, r) for s, r, _, _ in CASES} & MOVED)
        for s, r, now, _ in CASES:
            if (s, r) in MOVED:
                self.assertEqual((now, label_at_prev5(s, r, now)), (SKIP, EXPIRED))


class RealGate(unittest.TestCase):
    """Driven by the real gate (no rewrite)."""

    def correction_too_old(self, source, name):
        d = Drive(source, name)
        self.addCleanup(d.close)
        d.warm_and_navigate()
        d.gap(("rtab",), d.t)                           # rtabmap outputs stop, odometry goes on
        d.run(d.t + 1e-6, d.t + 2.4)                    # held correction ages past X = 2 s
        self.assertEqual((d.gate.state, d.gate.reason), ("stale", "correction_too_old"))
        b = d.queue_behind_expiry(d.t + .05)
        return d, [r[2] for r in d.sdiag.dropped() if r[0] == b]

    def test_correction_too_old_navigation_skipped(self):
        d, got = self.correction_too_old(base.current_source(), "rg_old")
        self.assertEqual(got, [SKIP])
        reasons = [r for _, r in d.faults()]
        self.assertNotIn(WAIT_FAULT, reasons)
        self.assertIn("localization_invalid", reasons)  # navigation stopped by the lease, unchanged

    @unittest.skipIf(not HAVE_PREV, "git history with %s not available" % PREV)
    def test_correction_too_old_control_previous_runner_faults(self):
        d, got = self.correction_too_old(base.source_at(PREV), "rg_old_prev")
        self.assertEqual(got, [EXPIRED])
        self.assertIn(WAIT_FAULT, [r for _, r in d.faults()])

    def test_gate_publishes_a_jump_as_confirming_never_as_lost(self):
        """lost:correction_jump is in the skip list, but the gate turns it into
        confirming_after_correction_jump within the same input: observed here."""
        d = Drive(base.current_source(), "rg_jump")
        self.addCleanup(d.close)
        seen = []
        deliver = d.world.deliver_status

        def hook(row):
            seen.append((row["state"], row["reason"]))
            deliver(row)
        d.world.deliver_status = hook
        d.warm_and_navigate()

        def dispatch(e):
            # localization_pose 30 cm off odometry from now on: one jump, then a new lock
            t, _, kind, stamp = e
            if kind != "rtab":
                return ebl.Drive.dispatch(d, e)
            d.clock(t)
            d.ref += 1
            d.ingest("info", {"header": base.header(stamp, "map"), "loop_closure_id": 1,
                              "proximity_detection_id": 0, "ref_id": d.ref})
            p = base.odom_pose(stamp)
            p = dict(p, position=dict(p["position"], x=p["position"]["x"] + .3))
            d.ingest("localization_pose", {"header": base.header(stamp, "map"),
                                           "pose": {"pose": p, "covariance": base.COV}})
        d.dispatch = dispatch
        d.run(d.t + 1e-6, d.t + 1.)
        self.assertGreaterEqual(d.gate.jumps, 1)
        self.assertIn(("confirming", "confirming_after_correction_jump"), seen)
        self.assertNotIn(("lost", "correction_jump"), seen)


class StaleAfterApproval(unittest.TestCase):
    """g4r_re: the first approved frame stalls ~1.3 s inside the runner and is stale at the age
    check. Skipped now while the last status is in the skip list; f4a273f: invalid:ValueError
    (sensor_or_pose_invalid during navigation)."""

    def nav_stall(self, source, name, view=None):
        d = Drive(source, name)
        self.addCleanup(d.close)
        d.flush_order = ("status", "image", "poll")
        d.warm_and_navigate()
        k = int((d.t + .3) * 1e9 / base.CAM_NS) + 1
        k += (-k) % 3                                   # an odometry stamp: approved
        a, b = base.BASE + k * base.CAM_NS, base.BASE + (k + 1) * base.CAM_NS
        ta = (a - base.BASE) / 1e9
        d.image_skip = lambda t, stamp: ta - .2 <= t < ta + .04 and stamp not in (a, b)
        d.pair_extra = {a: .015}
        state = d.stall_at(a)
        if view is not None:
            stall = d.stall

            def stall_then_status(seconds):
                stall(seconds)
                # a status arriving during the stall (another executor thread) said `view`
                d.node.localization_view = dict(view, row={"approved": []})
            d.stall = stall_then_status
        d.run(d.t + 1e-6, d.t + 1.5)
        self.assertTrue(state["done"])
        rows = [r[2] for r in d.sdiag.dropped() if r[0] == a]
        self.assertEqual(len(rows), 1, rows)
        return d, rows[0]

    def test_stale_approved_frame_navigation_skipped(self):
        d, got = self.nav_stall(base.current_source(), "st_nav")
        self.assertEqual(got, "stale_after_approval")
        reasons = [r for _, r in d.faults()]
        self.assertFalse([r for r in reasons if r.startswith("sensor_or_pose_invalid")], reasons)
        self.assertNotIn(WAIT_FAULT, reasons)
        self.assertGreater(d.node.localization_skipped, 0)

    @unittest.skipIf(not HAVE_PREV, "git history with %s not available" % PREV)
    def test_stale_approved_frame_control_previous_runner_faults(self):
        d, got = self.nav_stall(base.source_at(PREV), "st_nav_prev")
        self.assertEqual(got, "invalid:ValueError")
        self.assertTrue([r for _, r in d.faults() if r.startswith("sensor_or_pose_invalid")])

    def test_stale_with_no_correction_status_skipped(self):
        d, got = self.nav_stall(base.current_source(), "st_toold",
                                {"state": "stale", "reason": "correction_too_old", "kept": True})
        self.assertEqual(got, "stale_after_approval")
        self.assertFalse([r for _, r in d.faults() if r.startswith("sensor_or_pose_invalid")])

    def test_stale_with_broken_output_status_skipped(self):
        """第 5 棒收口: incomplete_output / localization_stream_timeout are "no localization"."""
        for state, reason in (("lost", "incomplete_output"), ("stale", "localization_stream_timeout")):
            with self.subTest(reason=reason):
                d, got = self.nav_stall(base.current_source(), "st_bo_" + state,
                                        {"state": state, "reason": reason, "kept": True})
                self.assertEqual(got, "stale_after_approval")
                self.assertFalse([r for _, r in d.faults() if r.startswith("sensor_or_pose_invalid")])
                self.assertGreater(d.node.localization_skipped, 0)
            if HAVE_PREV:
                with self.subTest(reason=reason, control=PREV):
                    d, got = self.nav_stall(base.source_at(PREV), "st_bop_" + state,
                                            {"state": state, "reason": reason, "kept": True})
                    self.assertEqual(got, "invalid:ValueError")
                    self.assertTrue([r for _, r in d.faults() if r.startswith("sensor_or_pose_invalid")])
            if HAVE_PREV5:
                with self.subTest(reason=reason, control=PREV5):
                    d, got = self.nav_stall(base.source_at(PREV5), "st_bop5_" + state,
                                            {"state": state, "reason": reason, "kept": True})
                    self.assertEqual(got, "invalid:ValueError")
                    self.assertTrue([r for _, r in d.faults() if r.startswith("sensor_or_pose_invalid")])

    def test_stale_with_odometry_or_clock_status_still_faults(self):
        for state, reason in (("stale", "odometry_stale"), ("lost", "odom_info:odometry_lost"),
                              ("initializing", "clock_rollback"),
                              ("stale", "incomplete_acquisition_timeout")):
            with self.subTest(reason=reason):
                d, got = self.nav_stall(base.current_source(), "st_f_" + state,
                                        {"state": state, "reason": reason, "kept": True})
                self.assertEqual(got, "invalid:ValueError")
                self.assertTrue([r for _, r in d.faults() if r.startswith("sensor_or_pose_invalid")])
            if HAVE_PREV:
                with self.subTest(reason=reason, control=PREV):
                    d, got = self.nav_stall(base.source_at(PREV), "st_fp_" + state,
                                            {"state": state, "reason": reason, "kept": True})
                    self.assertEqual(got, "invalid:ValueError")

    def test_stale_with_rejected_status_still_faults(self):
        d, got = self.nav_stall(base.current_source(), "st_none", {"state": "valid",
                                "reason": "three_consistent_corrections", "kept": False})
        self.assertEqual(got, "invalid:ValueError")


class Table(unittest.TestCase):
    """Every reason literal the gate can publish is classified (skip or a fault group)."""

    @staticmethod
    def classified(state, reason):
        if (state, reason) in ros_runtime.LOCALIZATION_SKIP_REASONS:
            return "skip"
        key = state + ":" + reason
        for group, patterns in ros_runtime.LOCALIZATION_FAULT_REASONS.items():
            if any(fnmatch.fnmatchcase(key, p) for p in patterns):
                return group
        return None

    def gate_pairs(self):
        """(state, reason) of every revoke/soft_reset/assignment in the two gate sources."""
        gate = (ROOT / "host" / "localization_gate.py").read_text(encoding="utf-8")
        node = (ROOT / "host" / "ros_localization_gate.py").read_text(encoding="utf-8")
        tree = ast.parse(gate)
        returns = set()                                 # message_reason's literals
        for fn in ast.walk(tree):
            if isinstance(fn, ast.FunctionDef) and fn.name == "message_reason":
                returns |= {n.value.value for n in ast.walk(fn) if isinstance(n, ast.Return)
                            and isinstance(n.value, ast.Constant) and isinstance(n.value.value, str)}
        returns.add("missing_ref_id")                   # assigned in correction_ingest
        kinds = ("odom", "odom_info", "info", "localization_pose")
        pairs, reasons_only = set(), set()
        for src in (gate, node):
            for n in ast.walk(ast.parse(src)):
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute):
                    if n.func.attr == "revoke" and len(n.args) == 2 and isinstance(n.args[0], ast.Constant):
                        state, r = n.args[0].value, n.args[1]
                        if isinstance(r, ast.Constant):
                            pairs.add((state, r.value))
                        elif isinstance(r, ast.BinOp) and isinstance(r.left, ast.BinOp):
                            pairs |= {(state, k + ":" + why) for k in kinds for why in returns}
                        elif isinstance(r, ast.BinOp) and isinstance(r.left, ast.Constant):
                            pairs.add((state, r.left.value + "1"))      # replay_discontinuity:<n>
                        else:
                            self.fail("unparsed revoke reason: " + ast.dump(r))
                    if n.func.attr == "soft_reset" and n.args and isinstance(n.args[0], ast.Constant):
                        pairs.add(("initializing", "entry_reset:" + n.args[0].value))
                if isinstance(n, ast.Assign):
                    targets = [t for t in n.targets]
                    names = [ast.dump(t) for t in targets]
                    if any("attr='reason'" in s for s in names):
                        reasons_only |= {c.value for c in ast.walk(n.value)
                                         if isinstance(c, ast.Constant) and isinstance(c.value, str)}
        return pairs, reasons_only

    def test_every_gate_reason_is_classified(self):
        pairs, reasons_only = self.gate_pairs()
        self.assertGreater(len(pairs), 40)
        missing = sorted(p for p in pairs if self.classified(*p) is None)
        self.assertEqual(missing, [])
        table_reasons = {r for _, r in ros_runtime.LOCALIZATION_SKIP_REASONS}
        for reason in reasons_only - {"initializing", "confirming", "valid", "entry_reset:"}:
            with self.subTest(reason=reason):
                self.assertIn(reason, table_reasons)

    def test_skip_list_is_exactly_the_ruling(self):
        skip = {k for k in ros_runtime.LOCALIZATION_SKIP_REASONS}
        no_corr = {k for k, v in ros_runtime.LOCALIZATION_SKIP_REASONS.items() if v == "no_correction"}
        self.assertEqual(no_corr, {("stale", "correction_too_old"), ("lost", "correction_jump"),
                                   ("lost", "incomplete_output"),
                                   ("stale", "localization_stream_timeout")})
        self.assertEqual({s for s, _ in skip} - {"valid", "initializing", "confirming"}, {"stale", "lost"})
        for state, reason in skip:
            self.assertNotIn("odom", reason)
            self.assertNotIn("clock", reason)
            self.assertNotIn("discontinuity", reason)
        for patterns in ros_runtime.LOCALIZATION_FAULT_REASONS.values():
            for state, reason in skip:
                self.assertFalse(any(fnmatch.fnmatchcase(state + ":" + reason, p) for p in patterns))

    def test_view_classifier(self):
        f = ros_runtime.localization_skip_class
        self.assertIsNone(f(None))
        self.assertEqual(f({"state": "valid", "reason": "correction_refreshed", "kept": True}), "waiting")
        self.assertIsNone(f({"state": "valid", "reason": "correction_refreshed", "kept": False}))
        self.assertEqual(f({"state": "stale", "reason": "correction_too_old", "kept": True}), "no_correction")
        self.assertIsNone(f({"state": "stale", "reason": "odometry_stale", "kept": True}))
        self.assertEqual(f({"state": "lost", "reason": "incomplete_output", "kept": True}), "no_correction")
        self.assertEqual(f({"state": "stale", "reason": "localization_stream_timeout", "kept": True}),
                         "no_correction")
        self.assertIsNone(f({"state": "stale", "reason": "incomplete_acquisition_timeout", "kept": True}))


if __name__ == "__main__":
    unittest.main()
