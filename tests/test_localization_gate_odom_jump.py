"""count_mode='correction': the odometry jump check (2026-10-17 第 10 棒).

Design (第 9 棒, runs/real-car-integ-20261015-results/gate_dig/HOST_G5_RESULT.md "第 9 棒" and
GATEFIX_HANDOVER.md "第 9 棒"): every new healthy odom + odom_info pair is compared with the previous one the
gate kept; a planar step |dxy| > V * dt + E (V = 0.5 m/s, E = 0.10 m, dt from the two stamps) is an odometry
jump. While valid -> revoke (new epoch, reason `odometry_jump`); during entry -> the candidates are dropped
(`entry_reset:odometry_jump`). No previous pair (start, clock rollback, replay discontinuity) -> not checked.
Measured on 9 host runs (2858 consecutive released steps): 7 triggers, all real jumps, no false trigger.
odom_jump_check=False must be the gate of 4edc10f (第 8 棒) exactly.

Known answers taken from host_g9/odom_steps.out (d_odom / dt of single recorded steps). Every test drives the
real LocalizationGate (and the real lease); mutation controls (no E, no dt term, wrong revoke reason, entry
not cleared) each fail at least one scenario here. Python 3.8+.
"""
import argparse
import collections
import copy
import importlib.util
import inspect
import os
from pathlib import Path
import random
import re
import subprocess
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tests'))

from host.localization_gate import LocalizationGate, LocalizationLease, matrix_pose, pose_matrix  # noqa: E402
from host import ros_runtime  # noqa: E402
import host.ros_localization_gate as node  # noqa: E402
from test_localization_gate_correction_mode import (BASE, LOST_COV, MAP, PRIOR, TRUE_M, Gate,  # noqa: E402
                                                    planar)
from test_localization_gate_output_mode import GOOD_COV  # noqa: E402

OLD = '4edc10f'     # 第 8 棒: the gate this change is built on


def need(cond, *what):
    """assert that survives python -O (the scenarios are also run against mutants)"""
    if not cond:
        raise AssertionError(' '.join(str(w) for w in what))


def odom_at(x, y=0., yaw_deg=0.):
    return planar(x, y, yaw_deg)


class JRig:
    """Odometry pair 20 ms after its stamp, rtabmap outputs 150 ms after (as test_localization_gate_correction_mode
    .Rig), but with free stamps (seconds after BASE) and free odometry poses."""

    def __init__(self, gate_cls=Gate, **kwargs):
        self.gate = gate_cls(**kwargs)
        self.lease = LocalizationLease(MAP, 'base_link')
        self.ref = 100
        self.approved = []
        self.t = 0.

    def clock(self, t):
        self.t = max(self.t, t)
        return 100. + self.t, BASE + int(round(self.t * 1e9))

    def send(self, kind, ts, late, odom, linked=True, lost=False, correction=TRUE_M, ref=None):
        wall, ros = self.clock(ts + late)
        stamp = BASE + int(round(ts * 1e9))
        header = {'stamp': {'sec': stamp // 10**9, 'nanosec': stamp % 10**9}}
        if kind == 'odom':
            row = {'header': dict(header, frame_id='odom'), 'child_frame_id': 'base_link',
                   'pose': {'pose': matrix_pose(odom), 'covariance': GOOD_COV}}
        elif kind == 'odom_info':
            row = {'header': dict(header, frame_id='odom'), 'lost': lost, 'covariance': LOST_COV}
        elif kind == 'info':
            row = {'header': dict(header, frame_id='map'), 'loop_closure_id': 1 if linked else 0,
                   'proximity_detection_id': 0, 'ref_id': ref}
        else:
            row = {'header': dict(header, frame_id='map'),
                   'pose': {'pose': matrix_pose(correction @ odom), 'covariance': GOOD_COV}}
        out = self.gate.ingest(kind, row, wall, ros)
        self.approved += out
        return out

    def pair(self, ts, odom, late=.02, lost=False):
        return self.send('odom', ts, late, odom) + self.send('odom_info', ts, late, odom, lost=lost)

    def rtab(self, ts, odom, late=.15, linked=True, correction=TRUE_M):
        self.ref += 1
        return (self.send('info', ts, late, odom, linked=linked, ref=self.ref)
                + self.send('localization_pose', ts, late, odom, correction=correction))

    def status(self):
        wall, ros = self.clock(self.t)
        row = self.gate.status(wall, ros)
        self.lease.update(row, wall, ros)
        return row

    def frame(self, ts, odom, rtab=True, **kw):
        self.pair(ts, odom)
        if rtab:
            self.rtab(ts, odom, **kw)
        return self.status()

    def enter(self, t0=0., x0=0., v=.3):
        """3 linked consistent corrections at t0, t0+.1, t0+.2 driving at v m/s; returns the last odom x."""
        for k in range(3):
            self.frame(t0 + .1 * k, odom_at(x0 + v * .1 * k))
        need(self.gate.state == 'valid', self.gate.state, self.gate.reason)
        return x0 + v * .2


def snapshot(g):
    return (g.state, g.reason, g.epoch, g.streak, len(g.candidates), g.held and g.held[0])


# ------------------------------------------------------------------ scenarios (also run on the mutants)
def step_in_valid(gate_cls, d, dt):
    """enter (last pair at t = .2 s, x = .06), then one pair d metres further after dt seconds; the gate."""
    rig = JRig(gate_cls)
    x = rig.enter()
    rig.pair(.2 + dt, odom_at(x + d))
    return rig


def scenario_valid_revoke(gate_cls):
    """a 0.482 m step in 0.364 s while valid (host g8r_3 15.10 s; bound 0.282 m) revokes: lost/odometry_jump,
    new epoch, no held correction, no approval from that pair, the lease drops the row."""
    rig = JRig(gate_cls)
    x = rig.enter()
    epoch, n = rig.gate.epoch, len(rig.approved)
    out = rig.pair(.564, odom_at(x + .482))
    need(out == [] and len(rig.approved) == n, 'approval from the jumped pair', out)
    need((rig.gate.state, rig.gate.reason) == ('lost', 'odometry_jump'), rig.gate.state, rig.gate.reason)
    need(rig.gate.epoch == epoch + 1 and rig.gate.held is None and rig.gate.odom_jumps == 1, snapshot(rig.gate))
    need(rig.gate.revokes == collections.Counter({'odometry_jump': 1}), rig.gate.revokes)
    row = rig.status()
    need(row['state'] == 'lost' and row['approved'] == [], row['state'])
    wall, ros = rig.clock(rig.t)
    need(not rig.lease.check(wall, ros), 'lease still healthy after the revoke')
    return rig


def scenario_known_jumps_trigger(gate_cls):
    """recorded jump steps (d_odom m, dt s; odom_steps.out) revoke while valid"""
    for arm, t, d, dt in (('g8r_3', 15.1, .4824, .3641), ('g8r_2', 21.536, .1776, .0984),
                          ('g8r_2', 21.675, .4436, .1389), ('g7r_1', 21.536, .39, .0984),
                          ('g7r_1', 21.876, .2091, .2006), ('g7r_3', 21.675, .2925, .1389),
                          ('g7r_3', 47.171, .1939, .1033)):
        g = step_in_valid(gate_cls, d, dt).gate
        need((g.state, g.reason) == ('lost', 'odometry_jump'), arm, t, g.state, g.reason)


def scenario_reachable_steps_do_not_trigger(gate_cls):
    """steps inside V * dt + E do not trigger, including two real jumps the check cannot see (KNOWN LIMIT):
    g8r_3 14.505 s, 0.1954 m in 0.3362 s (bound 0.268 m; its released poses were 0.26-0.36 m off GT) and
    g6r_3 43.0 s, 0.2192 m in 0.3703 s (bound 0.285 m; resid 0.157 m). And the largest non-jump step over V * dt
    (g7r_3 21.536 s, 0.1462 m in 0.1298 s: 0.0813 m over V * dt, 1.9 cm inside E)."""
    for arm, t, d, dt in (('g8r_3', 14.505, .1954, .3362), ('g6r_3', 43.0, .2192, .3703),
                          ('g7r_3', 21.536, .1462, .1298), ('g7r_1', 21.675, .1566, .1389),
                          ('g6r_1', 46.197, .1723, .5606)):
        g = step_in_valid(gate_cls, d, dt).gate
        need((g.state, g.odom_jumps) == ('valid', 0), arm, t, g.state, g.reason)


def scenario_bound_both_sides(gate_cls):
    """1 mm either side of V * dt + E through the real ingest path, at two dt"""
    for dt in (.1, .4):
        bound = .5 * dt + .1
        g = step_in_valid(gate_cls, bound - .001, dt).gate
        need(g.state == 'valid', 'triggered below the bound', dt)
        g = step_in_valid(gate_cls, bound + .001, dt).gate
        need((g.state, g.reason) == ('lost', 'odometry_jump'), 'not triggered above the bound', dt, g.reason)


def scenario_entry_clear(gate_cls):
    """two consistent candidates, then a jumped pair: candidates dropped (entry_reset:odometry_jump), same
    epoch; three corrections of the new odometry then enter."""
    rig = JRig(gate_cls)
    rig.frame(0., odom_at(0.))
    rig.frame(.1, odom_at(.03))
    need((rig.gate.state, rig.gate.streak) == ('confirming', 2), snapshot(rig.gate))
    epoch = rig.gate.epoch
    jump = planar(0., .3, 0.)                         # odometry jumped 0.3 m sideways
    rig.pair(.2, jump @ odom_at(.06))
    need((rig.gate.state, rig.gate.reason, rig.gate.streak, rig.gate.candidates) ==
         ('initializing', 'entry_reset:odometry_jump', 0, []), snapshot(rig.gate))
    need(rig.gate.epoch == epoch and rig.gate.odom_jumps == 1, snapshot(rig.gate))
    new_m = TRUE_M @ np.linalg.inv(jump)              # rtabmap's correction of the jumped odometry
    rig.rtab(.2, jump @ odom_at(.06), correction=new_m)
    rig.frame(.3, jump @ odom_at(.09), correction=new_m)
    rig.frame(.4, jump @ odom_at(.12), correction=new_m)
    need(rig.gate.state == 'valid', 'no re-entry after the clear', snapshot(rig.gate))
    a = rig.approved[-1]
    need(np.abs(pose_matrix(a['pose']['pose']) - TRUE_M @ odom_at(.12)).max() < 1e-9, 'pose off after re-entry')


SCENARIOS = (scenario_valid_revoke, scenario_known_jumps_trigger, scenario_reachable_steps_do_not_trigger,
             scenario_bound_both_sides, scenario_entry_clear)


class CriterionTests(unittest.TestCase):
    def test_predicate_is_strict_at_the_bound(self):
        g = Gate()
        self.assertFalse(g.odom_jumped(.2, .2))             # 0.5 * 0.2 + 0.1 = 0.2 exactly: not a jump
        self.assertTrue(g.odom_jumped(.2 + 1e-9, .2))
        self.assertFalse(g.odom_jumped(.1, 0.))
        self.assertTrue(g.odom_jumped(.1 + 1e-9, 0.))
        g = Gate(max_odom_speed_mps=1., odom_jump_margin_m=0.)
        self.assertFalse(g.odom_jumped(.3, .3))
        self.assertTrue(g.odom_jumped(.31, .3))

    def test_defaults_and_validation(self):
        g = Gate()
        self.assertEqual((g.odom_jump_check, g.max_odom_speed_mps, g.odom_jump_margin_m), (True, .5, .1))
        self.assertEqual(g.odom_jumps, 0)
        for kw in ({'max_odom_speed_mps': 0}, {'max_odom_speed_mps': -1.}, {'max_odom_speed_mps': float('inf')},
                   {'max_odom_speed_mps': float('nan')}, {'max_odom_speed_mps': '0.5'},
                   {'odom_jump_margin_m': -.01}, {'odom_jump_margin_m': float('nan')}, {'odom_jump_margin_m': None},
                   {'odom_jump_check': 1}, {'odom_jump_check': None}):
            with self.subTest(kw=kw), self.assertRaises(ValueError):
                LocalizationGate(MAP, PRIOR, count_mode='correction', **kw)
        LocalizationGate(MAP, PRIOR, count_mode='correction', odom_jump_margin_m=0, max_odom_speed_mps=2)

    def test_bound_both_sides(self):
        scenario_bound_both_sides(Gate)

    def test_recorded_jumps_trigger(self):
        scenario_known_jumps_trigger(Gate)

    def test_reachable_steps_do_not_trigger_known_limit(self):
        """KNOWN LIMIT (第 9 棒): a jump within what the car could have driven (V * dt + E) is not seen, e.g.
        host g8r_3 14.505 s 0.195 m / 0.336 s; only the 0.48 m step after it (15.10 s) is caught."""
        scenario_reachable_steps_do_not_trigger(Gate)

    def test_yaw_is_not_used(self):
        rig = JRig()
        x = rig.enter()
        rig.pair(.3, odom_at(x + .03, 0., 40.))          # 40 deg in 0.1 s, no translation beyond the bound
        self.assertEqual((rig.gate.state, rig.gate.odom_jumps), ('valid', 0))


class ValidRevokeTests(unittest.TestCase):
    def test_revoke_reason_epoch_and_no_approval(self):
        scenario_valid_revoke(Gate)

    def test_first_pair_after_entry_is_checked_against_the_pre_entry_pair(self):
        """entry publishes from a pair that had already arrived; the next pair is a step from it"""
        rig = JRig()
        x = rig.enter()
        self.assertEqual(rig.gate.odom_pair['stamp'], BASE + 2 * 10**8)
        rig.pair(.3, odom_at(x + .3))                    # 0.3 m in 0.1 s
        self.assertEqual((rig.gate.state, rig.gate.reason), ('lost', 'odometry_jump'))

    def test_the_jumped_pair_is_the_new_baseline_and_its_correction_candidate_1(self):
        rig = JRig()
        x = rig.enter()
        jump = planar(.4, 0., 0.)
        new_m = TRUE_M @ np.linalg.inv(jump)
        rig.pair(.3, jump @ odom_at(x + .03))
        self.assertEqual(rig.gate.reason, 'odometry_jump')
        rig.rtab(.3, jump @ odom_at(x + .03), correction=new_m)
        self.assertEqual((rig.gate.state, rig.gate.reason, rig.gate.streak), ('confirming', 'confirming_corrections', 1))
        for k, ts in enumerate((.4, .5), 2):
            rig.frame(ts, jump @ odom_at(x + .03 * k), correction=new_m)
        self.assertEqual(rig.gate.state, 'valid')
        self.assertEqual(rig.gate.odom_jumps, 1)            # steps after the jump are normal again
        self.assertEqual(dict(rig.gate.revokes), {'odometry_jump': 1})
        self.assertLess(np.abs(pose_matrix(rig.approved[-1]['pose']['pose']) - TRUE_M @ odom_at(x + .09)).max(), 1e-9)

    def test_checked_before_the_same_stamp_correction(self):
        """rtabmap outputs of the jumped stamp arrive first; the odom_info that completes the pair also completes
        the correction: the jump is decided first (odometry_jump, not correction_jump) and that correction,
        which is rtabmap's for the jumped odometry, is candidate 1."""
        rig = JRig()
        x = rig.enter()
        jump = planar(0., -.5, 0.)
        new_m = TRUE_M @ np.linalg.inv(jump)
        odom = jump @ odom_at(x + .03)
        rig.rtab(.3, odom, late=.03, correction=new_m)
        rig.pair(.3, odom, late=.04)
        self.assertEqual(dict(rig.gate.revokes), {'odometry_jump': 1})
        self.assertEqual(rig.gate.jumps, 0)
        self.assertEqual((rig.gate.state, rig.gate.streak), ('confirming', 1))

    def test_normal_drive_never_triggers(self):
        rig = JRig()
        x = rig.enter()
        for k in range(1, 60):                            # 0.3 m/s, every third pair refreshes
            rig.frame(.2 + .1 * k, odom_at(x + .03 * k), rtab=(k % 3 == 0))
        self.assertEqual((rig.gate.state, rig.gate.odom_jumps), ('valid', 0))
        self.assertEqual(rig.gate.revokes, collections.Counter())


class EntryAndFirstStepTests(unittest.TestCase):
    def test_entry_candidates_are_dropped(self):
        scenario_entry_clear(Gate)

    def test_entry_without_candidates_changes_nothing_visible(self):
        rig = JRig()
        rig.pair(0., odom_at(0.))
        before = snapshot(rig.gate)
        rig.pair(.1, odom_at(1.))
        self.assertEqual(snapshot(rig.gate), before)
        self.assertEqual(rig.gate.odom_jumps, 1)

    def test_first_pair_ever_is_not_checked(self):
        rig = JRig()
        rig.pair(0., odom_at(100., -40.))
        self.assertEqual(rig.gate.odom_jumps, 0)
        self.assertIsNotNone(rig.gate.odom_pair)

    def test_first_pair_after_discontinuity_or_rollback_is_not_checked(self):
        rig = JRig()
        x = rig.enter()
        rig.gate.discontinuity(1)
        self.assertIsNone(rig.gate.odom_pair)
        rig.pair(.3, odom_at(x + 5.))
        self.assertEqual(rig.gate.odom_jumps, 0)
        rig.gate.status(50., BASE)                        # clocks went back: clock_rollback
        self.assertIsNone(rig.gate.odom_pair)
        rig.pair(.4, odom_at(-5.))
        self.assertEqual(rig.gate.odom_jumps, 0)

    def test_unhealthy_pair_is_not_a_step(self):
        """a lost odom_info pair is not kept, so the next healthy pair steps from the last healthy one"""
        rig = JRig()
        x = rig.enter()
        rig.pair(.3, odom_at(x + 3.), lost=True)
        self.assertEqual((rig.gate.reason, rig.gate.odom_jumps), ('odom_info:odometry_lost', 0))
        rig.pair(.4, odom_at(x + .06))
        self.assertEqual(rig.gate.odom_jumps, 0)


# ------------------------------------------------------------------ switch off == 4edc10f
def old_gate_module():
    try:
        src = subprocess.check_output(['git', 'show', '%s:host/localization_gate.py' % OLD], cwd=str(ROOT),
                                      stderr=subprocess.DEVNULL).decode('utf-8')
    except (OSError, subprocess.CalledProcessError):
        return None
    spec = importlib.util.spec_from_loader('localization_gate_' + OLD, loader=None)
    mod = importlib.util.module_from_spec(spec)
    exec(compile(src, 'localization_gate@' + OLD, 'exec'), mod.__dict__)
    return mod


OLD_MOD = old_gate_module()


def random_script(seed, n=160, jump_p=.08):
    """a random correction-mode input: drive, odometry jumps of 0.05-0.8 m, lost/late/unlinked/inconsistent
    rtabmap outputs, odometry stalls, a discontinuity; [(op, args)]"""
    r = random.Random(seed)
    ops, ts, x, y, off, m = [], 0., 0., 0., np.eye(4), TRUE_M
    for _ in range(n):
        ts += r.choice((.1, .1, .1, .05, .2, .35, 1.2 if r.random() < .05 else .1))
        x += .3 * r.random() * .1
        if r.random() < jump_p:
            off = planar(r.uniform(-.8, .8), r.uniform(-.8, .8), r.uniform(-10, 10)) @ off
            if r.random() < .6:
                m = TRUE_M @ np.linalg.inv(off)          # rtabmap follows the jumped odometry
        odom = off @ odom_at(x, y)
        ops.append(('pair', ts, odom, r.random() < .03))
        if r.random() < .6:
            corr = m if r.random() < .9 else planar(r.uniform(-.3, .3), r.uniform(-.3, .3), 0.) @ m
            ops.append(('rtab', ts, odom, r.random() < .8, corr, r.choice((.15, .15, .3, .7))))
        if r.random() < .1:
            ops.append(('status', ts + .05))
        if r.random() < .005:
            ops.append(('disc',))
    return ops


def trace(make_gate, ops):
    """every observable after every input: outputs, status rows (instance aside), corrections, counters"""
    rig = JRig(make_gate)
    out = []
    for op in ops:
        if op[0] == 'pair':
            res = rig.pair(op[1], op[2], lost=op[3])
        elif op[0] == 'rtab':
            res = rig.rtab(op[1], op[2], linked=op[3], correction=op[4], late=op[5])
        elif op[0] == 'status':
            rig.clock(op[1])
            res = rig.status()
            res = {k: v for k, v in res.items() if k != 'instance'}
        else:
            rig.gate.discontinuity(1)
            res = None
        g = rig.gate
        out.append((copy.deepcopy(res), g.state, g.reason, g.epoch, g.streak, g.sequence,
                    [c[0] for c in g.candidates], g.held and g.held[0], g.jump_led, g.adopted, g.jumps,
                    g.synthesized, g.last_complete, g.last_ref, g.drain_corrections(), dict(g.revokes)))
    return out, rig.gate


def gate_of(module, **kw):
    class G(module.LocalizationGate):
        def __init__(self):
            super().__init__(MAP, PRIOR, tracked_frame='base_link', count_mode='correction', **kw)
            self.revokes = collections.Counter()

        def revoke(self, state, reason):
            self.revokes[reason] += 1
            super().revoke(state, reason)
    return G


@unittest.skipUnless(OLD_MOD is not None, 'git history (%s) unavailable' % OLD)
class SwitchOffIsOldGateTests(unittest.TestCase):
    """odom_jump_check=False against the gate of 4edc10f (git show), same inputs, every observable."""

    def test_random_inputs_identical_with_the_check_off(self):
        old, new = gate_of(OLD_MOD), gate_of(sys.modules['host.localization_gate'], odom_jump_check=False)
        on = gate_of(sys.modules['host.localization_gate'])
        jumps = 0
        for seed in range(120):
            ops = random_script(seed)
            (a, _), (b, _) = trace(old, ops), trace(new, ops)
            self.assertEqual(len(a), len(b))
            for k, (x, y) in enumerate(zip(a, b)):
                self.assertEqual(x, y, (seed, k, ops[k][0]))
            jumps += trace(on, ops)[1].odom_jumps
        self.assertGreater(jumps, 50)        # the inputs do contain jumps the check would see

    def test_check_on_without_a_trigger_is_identical_too(self):
        old, new = gate_of(OLD_MOD), gate_of(sys.modules['host.localization_gate'])
        quiet = 0
        for seed in range(400, 520):
            ops = random_script(seed, n=60, jump_p=.01)
            b, gate = trace(new, ops)
            if gate.odom_jumps:
                continue
            quiet += 1
            self.assertEqual(trace(old, ops)[0], b, seed)
        self.assertGreater(quiet, 10)

    def test_scenarios_on_the_old_gate_and_switch_off(self):
        """positive control of the scenarios: blocking scenarios fail on 4edc10f and with the switch off"""
        for g in (gate_of(OLD_MOD), gate_of(sys.modules['host.localization_gate'], odom_jump_check=False)):
            for s in (scenario_valid_revoke, scenario_known_jumps_trigger, scenario_bound_both_sides,
                      scenario_entry_clear):
                with self.subTest(gate=g, scenario=s.__name__), self.assertRaises((AssertionError, AttributeError)):
                    s(g)
            scenario_reachable_steps_do_not_trigger_off(g)


def scenario_reachable_steps_do_not_trigger_off(gate_cls):
    """as scenario_reachable_steps_do_not_trigger, for a gate that has no odom_jumps counter"""
    for d, dt in ((.1954, .3362), (.1462, .1298)):
        need(step_in_valid(gate_cls, d, dt).gate.state == 'valid', d, dt)


# ------------------------------------------------------------------ mutation controls
class NoMargin(Gate):
    def odom_jumped(self, distance_m, dt_s):
        return distance_m > self.max_odom_speed_mps * dt_s


class NoDt(Gate):
    def odom_jumped(self, distance_m, dt_s):
        return distance_m > self.odom_jump_margin_m


class WrongReason(Gate):
    def revoke(self, state, reason):
        super().revoke(state, 'correction_jump' if reason == 'odometry_jump' else reason)


class EntryNotCleared(Gate):
    def soft_reset(self, why):
        if why != 'odometry_jump':
            super().soft_reset(why)


class MutationTests(unittest.TestCase):
    def test_every_mutant_fails_a_scenario_and_the_gate_passes_all(self):
        for s in SCENARIOS:
            s(Gate)
        expected = {NoMargin: {scenario_reachable_steps_do_not_trigger, scenario_bound_both_sides},
                    NoDt: {scenario_reachable_steps_do_not_trigger, scenario_bound_both_sides},
                    WrongReason: {scenario_valid_revoke, scenario_known_jumps_trigger, scenario_bound_both_sides},
                    EntryNotCleared: {scenario_entry_clear}}
        for mutant, failing in expected.items():
            for s in SCENARIOS:
                with self.subTest(mutant=mutant.__name__, scenario=s.__name__):
                    if s in failing:
                        with self.assertRaises(AssertionError):
                            s(mutant)
                    else:
                        s(mutant)


# ------------------------------------------------------------------ wiring: node, scripts, runner table
CAR = ROOT / 'tools' / 'car_run'


class WiringTests(unittest.TestCase):
    source = inspect.getsource(node.main)

    def test_node_options(self):
        self.assertIn("'--odom-jump-check', action=argparse.BooleanOptionalAction, default=True", self.source)
        self.assertIn("'--max-odom-speed-mps', type=float, default=.5", self.source)
        self.assertIn("'--odom-jump-margin-m', type=float, default=.1", self.source)
        for k in ('odom_jump_check', 'max_odom_speed_mps', 'odom_jump_margin_m'):
            self.assertIn('%s=args.%s' % (k, k), self.source)

    @unittest.skipUnless(hasattr(argparse, 'BooleanOptionalAction'), 'node CLI needs Python 3.9+ (board: 3.10)')
    def test_node_rejects_bad_v_and_e(self):
        for extra in (['--max-odom-speed-mps', '0'], ['--max-odom-speed-mps', 'nan'], ['--odom-jump-margin-m', '-1'],
                      ['--odom-jump-margin-m', 'inf']):
            with self.subTest(extra=extra), self.assertRaises(SystemExit) as e:
                sys.stderr, saved = open(os.devnull, 'w'), sys.stderr
                try:
                    node.main(['--database', '/nonexistent.db', '--map-id', 'x'] + extra)
                finally:
                    sys.stderr.close()
                    sys.stderr = saved
            self.assertEqual(e.exception.code, 2)

    def test_rtabmap_car_passes_and_checks(self):
        rtab = (CAR / 'rtabmap_car.sh').read_text()
        self.assertIn(': "${GATE_ODOM_JUMP:=1}" "${GATE_MAX_ODOM_SPEED_MPS:=0.5}" "${GATE_ODOM_JUMP_MARGIN_M:=0.10}"', rtab)
        self.assertRegex(rtab, r'PASS=\([^)]*\bGATE_ODOM_JUMP\b[^)]*\bGATE_MAX_ODOM_SPEED_MPS\b[^)]*'
                               r'\bGATE_ODOM_JUMP_MARGIN_M\b')
        self.assertIn('GA+=(--max-odom-speed-mps "$GATE_MAX_ODOM_SPEED_MPS" --odom-jump-margin-m '
                      '"$GATE_ODOM_JUMP_MARGIN_M")', rtab)
        self.assertIn('if [ "$GATE_ODOM_JUMP" = 1 ]; then GA+=(--odom-jump-check); else GA+=(--no-odom-jump-check); fi',
                      rtab)
        for name, cases in (('GATE_MAX_ODOM_SPEED_MPS', (('0.5', 1), ('1', 1), ('0.25', 1), ('0', 0), ('0.0', 0),
                                                         ('', 0), ('-1', 0), ('.', 0), ('1.2.3', 0), ('fast', 0))),
                            ('GATE_ODOM_JUMP_MARGIN_M', (('0.10', 1), ('0', 1), ('0.0', 1), ('1', 1), ('', 0),
                                                         ('-0.1', 0), ('.', 0), ('1.2.3', 0), ('10cm', 0)))):
            ok = {'GATE_MAX_ODOM_SPEED_MPS': 'GATE_V_OK', 'GATE_ODOM_JUMP_MARGIN_M': 'GATE_E_OK'}[name]
            line = next(l for l in rtab.splitlines() if l.startswith('case "$%s" in' % name))
            for value, want in cases:
                out = subprocess.run(['bash', '-c', '%s="$1"; %s; echo "$%s"' % (name, line, ok), '_', value],
                                     stdout=subprocess.PIPE, universal_newlines=True, check=True).stdout
                self.assertEqual(out.strip(), str(want), (name, value))
        switch = next(l for l in rtab.splitlines() if l.startswith('case "$GATE_ODOM_JUMP" in'))
        for value, code in (('1', 0), ('0', 0), ('', 2), ('yes', 2), ('2', 2)):
            r = subprocess.run(['bash', '-c', 'GATE_ODOM_JUMP="$1"; %s' % switch, '_', value],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.assertEqual(r.returncode, code, value)

    def test_car_up_and_inner_carry_the_three(self):
        up = (CAR / 'car_up.sh').read_text()
        inner = (CAR / 'car_inner.sh').read_text()
        self.assertIn(': "${CAR_GATE_ODOM_JUMP:=1}" "${CAR_GATE_ODOM_V_MPS:=0.5}" "${CAR_GATE_ODOM_E_M:=0.10}"', up)
        self.assertIn('INNER_ENV+=(RTABMAP_GATE_ODOM_JUMP="$CAR_GATE_ODOM_JUMP" RTABMAP_GATE_ODOM_V_MPS="$CAR_GATE_ODOM_V_MPS"\n'
                      '                               RTABMAP_GATE_ODOM_E_M="$CAR_GATE_ODOM_E_M")', up)
        self.assertIn(': "${RTABMAP_GATE_ODOM_JUMP:=1}" "${RTABMAP_GATE_ODOM_V_MPS:=0.5}" "${RTABMAP_GATE_ODOM_E_M:=0.10}"',
                      inner)
        self.assertIn('GATE_ODOM_JUMP="$RTABMAP_GATE_ODOM_JUMP" GATE_MAX_ODOM_SPEED_MPS="$RTABMAP_GATE_ODOM_V_MPS" \\\n'
                      '    GATE_ODOM_JUMP_MARGIN_M="$RTABMAP_GATE_ODOM_E_M" CAM_MOUNT_TF=', inner)

    def test_runner_classifies_both_reasons_as_odometry_faults(self):
        import fnmatch
        for state, reason in (('lost', 'odometry_jump'), ('initializing', 'entry_reset:odometry_jump')):
            self.assertNotIn((state, reason), ros_runtime.LOCALIZATION_SKIP_REASONS)
            self.assertIsNone(ros_runtime.localization_skip_class({'state': state, 'reason': reason, 'kept': True}))
            self.assertTrue(any(fnmatch.fnmatchcase(state + ':' + reason, p)
                                for p in ros_runtime.LOCALIZATION_FAULT_REASONS['odometry']))
        # and the gate really emits exactly these two pairs
        seen = set()
        rig = JRig()
        rig.frame(0., odom_at(0.))
        rig.pair(.1, odom_at(1.))
        seen.add((rig.gate.state, rig.gate.reason))
        rig = JRig()
        x = rig.enter()
        rig.pair(.3, odom_at(x + 1.))
        seen.add((rig.gate.state, rig.gate.reason))
        self.assertEqual(seen, {('lost', 'odometry_jump'), ('initializing', 'entry_reset:odometry_jump')})


import test_car_rtabmap_run as car_rt  # noqa: E402  (module, not its TestCases: they are not re-collected here)


class CarChainDryRunTests(car_rt.Base):
    """car_up.sh CAR_GATE_ODOM_* -> INNER_ENV -> car_inner.sh -> rtabmap_car.sh env (board dry run, live preset)"""

    setUp = car_rt.TestBoardRtabmap.setUp

    def plan(self, **env):
        self.env.update(env)
        r = self.up('live', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        return car_rt.line(r.stdout, '    [rtabmap]')

    def test_defaults_reach_rtabmap_car(self):
        self.assertIn('GATE_ODOM_JUMP=1 GATE_MAX_ODOM_SPEED_MPS=0.5 GATE_ODOM_JUMP_MARGIN_M=0.10 CAM_MOUNT_TF=',
                      self.plan())

    def test_values_reach_rtabmap_car(self):
        rt = self.plan(CAR_GATE_ODOM_JUMP='0', CAR_GATE_ODOM_V_MPS='0.3', CAR_GATE_ODOM_E_M='0.2')
        self.assertIn('GATE_ODOM_JUMP=0 GATE_MAX_ODOM_SPEED_MPS=0.3 GATE_ODOM_JUMP_MARGIN_M=0.2 CAM_MOUNT_TF=', rt)
        self.assertIn('GATE_COUNT_MODE=correction GATE_MAX_CORRECTION_AGE_S=2 bash tools/car_run/rtabmap_car.sh loc', rt)


if __name__ == '__main__':
    unittest.main()
