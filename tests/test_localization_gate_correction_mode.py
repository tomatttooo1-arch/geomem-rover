"""count_mode='correction' (owner decision 2026-10-15 evening, "scheme 2"): approve the map->odom
correction, then one pose per odometry pair.

Design and known answers: runs/real-car-integ-20261015-results/gate_dig/GATEFIX_HANDOVER.md
("第 2 棒 HANDOVER" B/C), RELAX_EVAL.md §1/§6, reference simulator relax_sim.py C2 (results in
relax_results.json). Every test drives the real LocalizationGate and the real, unchanged
LocalizationLease; ROS is not needed. Python 3.8+.
"""
import collections
import inspect
import math
from pathlib import Path
import re
import subprocess
import unittest

import numpy as np

from host.localization_gate import (COUNT_MODES, LocalizationGate, LocalizationLease, matrix_pose,
                                    pose_matrix, pose_valid, rigid_valid)
import host.ros_localization_gate as node
from test_localization_gate_output_mode import FIXTURE, GOOD_COV, build_stream, load_fixture, make_row

MAP = 'e' * 64
PRIOR = {1, 2}
BASE = 10**9            # stamp of odometry frame 0; frame i is BASE + i*10**8 (10 Hz odometry)
LOST_COV = [0.01 if i % 7 == 0 else 0. for i in range(36)]


def stamp_of(i):
    return BASE + i * 10**8


def planar(x, y, yaw_deg):
    yaw = math.radians(yaw_deg)
    m = np.eye(4)
    m[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    m[:2, 3] = x, y
    return m


TRUE_M = planar(1.5, -2., 30.)      # the map->odom the synthetic rtabmap reports


def odom_matrix(i):
    return planar(.05 * i, .01 * i, 2. * i)


class Gate(LocalizationGate):
    """The real gate in correction mode; records each revoke reason on the way through."""

    def __init__(self, prior=PRIOR, **kwargs):
        kwargs.setdefault('count_mode', 'correction')
        super().__init__(MAP, prior, tracked_frame='base_link', **kwargs)
        self.revokes = collections.Counter()

    def revoke(self, state, reason):
        self.revokes[reason] += 1
        super().revoke(state, reason)


class Rig:
    """Odometry pair 20 ms after its stamp, rtabmap outputs 150 ms after; status feeds a lease."""

    def __init__(self, **kwargs):
        self.gate = Gate(**kwargs)
        self.lease = LocalizationLease(MAP, 'base_link')
        self.ref = 100
        self.approved = []
        self.t = 0.

    def clock(self, t):
        self.t = max(self.t, t)
        return 100. + self.t, BASE + int(round(self.t * 1e9))

    def send(self, kind, i, late, **change):
        wall, ros = self.clock(i / 10. + late)
        stamp = stamp_of(i)
        header = {'stamp': {'sec': stamp // 10**9, 'nanosec': stamp % 10**9}}
        if kind == 'odom':
            row = {'header': dict(header, frame_id='odom'), 'child_frame_id': 'base_link',
                   'pose': {'pose': matrix_pose(odom_matrix(i)), 'covariance': GOOD_COV}}
        elif kind == 'odom_info':
            row = {'header': dict(header, frame_id='odom'), 'lost': change.pop('lost', False), 'covariance': LOST_COV}
        elif kind == 'info':
            linked = change.pop('linked', True)
            row = {'header': dict(header, frame_id='map'), 'loop_closure_id': 1 if linked else 0,
                   'proximity_detection_id': 0, 'ref_id': change.pop('ref')}
        else:
            m = change.pop('correction', TRUE_M) @ odom_matrix(i)
            row = {'header': dict(header, frame_id='map'), 'pose': {'pose': matrix_pose(m), 'covariance': GOOD_COV}}
        row.update(change)
        out = self.gate.ingest(kind, row, wall, ros)
        self.approved += out
        return out

    def pair(self, i, late=.02, lost=False):
        return self.send('odom', i, late) + self.send('odom_info', i, late, lost=lost)

    def rtab(self, i, late=.15, linked=True, gap=0, correction=TRUE_M):
        self.ref += 1 + gap
        return (self.send('info', i, late, ref=self.ref, linked=linked)
                + self.send('localization_pose', i, late, correction=correction))

    def status(self, t=None):
        wall, ros = self.clock(self.t if t is None else t)
        row = self.gate.status(wall, ros)
        self.lease.update(row, wall, ros)
        return row

    def frame(self, i, rtab=True, **kw):
        self.pair(i)
        if rtab:
            self.rtab(i, **kw)
        return self.status()

    def enter(self, start=0):
        for i in range(start, start + 3):
            self.frame(i)
        assert self.gate.state == 'valid', (self.gate.state, self.gate.reason)


class EntryTests(unittest.TestCase):
    def test_three_consecutive_linked_corrections_enter_and_publish_at_once(self):
        rig = Rig()
        rig.frame(0)
        rig.frame(1)
        self.assertEqual(rig.gate.state, 'confirming')
        self.assertEqual(rig.approved, [])
        rig.pair(2)
        out = rig.rtab(2)
        self.assertEqual(rig.gate.state, 'valid')
        # one approval at the newest odometry pair, published by the call that entered valid
        self.assertEqual([a['stamp_ns'] for a in out], [stamp_of(2)])
        row = rig.status()
        self.assertEqual(row['stamp_ns'], stamp_of(2))
        self.assertEqual(row['correction_stamp_ns'], stamp_of(2))
        self.assertIsNotNone(rig.lease.row)
        wall, ros = rig.clock(rig.t)
        self.assertIsNotNone(rig.lease.token(stamp_of(2), wall, ros))

    def test_entry_uses_the_newest_pair_even_when_odometry_is_ahead(self):
        rig = Rig()
        for i in range(3):
            rig.pair(2 * i)
            rig.pair(2 * i + 1)
            rig.rtab(2 * i, late=.15)
        self.assertEqual(rig.gate.state, 'valid')
        self.assertEqual(rig.approved[-1]['stamp_ns'], stamp_of(5))

    def test_gap_or_inconsistent_restarts_the_count(self):
        for name, spoil in (('ref_gap', {'gap': 1}), ('inconsistent', {'correction': planar(1.7, -2., 30.)})):
            rig = Rig()
            rig.frame(0)
            rig.frame(1)
            rig.frame(2, **spoil)
            self.assertNotEqual(rig.gate.state, 'valid', name)
            self.assertEqual(rig.gate.epoch, 0, name)          # nothing approved: no new epoch
            self.assertEqual(sum(rig.gate.revokes.values()), 0, name)
            # the spoiling one counts as 1st
            rig.frame(3, correction=spoil.get('correction', TRUE_M))
            self.assertEqual(rig.gate.state, 'confirming', name)
            rig.frame(4, correction=spoil.get('correction', TRUE_M))
            self.assertEqual(rig.gate.state, 'valid', name)

    def test_entry_candidates_age_out_after_x(self):
        rig = Rig()
        rig.frame(0)
        rig.frame(1)
        for i in range(2, 25):
            rig.frame(i, rtab=False)
        self.assertEqual(rig.gate.reason, 'entry_reset:entry_stale')
        self.assertEqual(rig.gate.streak, 0)


class EntryUnlinkedTests(unittest.TestCase):
    """第 7 棒 (2026-10-16, owner-authorized): during entry an output not linked to the prior map neither
    counts nor clears the candidates. Old rule (main 0ddb379): it cleared them (entry_reset:unlinked).
    Frame i arrives at i/10 s, X = 2 s."""

    def verdicts(self, rig):
        return [c['verdict'] for c in rig.gate.drain_corrections()]

    def test_linked_unlinked_linked_unlinked_linked_enters(self):
        rig = Rig()
        rig.frame(0)                                           # A
        rig.frame(1, linked=False)
        self.assertEqual((rig.gate.state, rig.gate.reason, rig.gate.streak),
                         ('confirming', 'confirming_corrections', 1))   # not reset, not rewritten
        rig.frame(2)                                           # B
        rig.frame(3, linked=False)
        self.assertEqual((rig.gate.state, rig.gate.streak), ('confirming', 2))
        self.assertEqual(rig.approved, [])
        rig.frame(4)                                           # C, 0.4 s after A
        self.assertEqual((rig.gate.state, rig.gate.reason), ('valid', 'three_consistent_corrections'))
        self.assertEqual(rig.gate.epoch, 0)
        self.assertEqual(rig.gate.revokes, collections.Counter())
        self.assertEqual(self.verdicts(rig), ['candidate', 'unlinked', 'candidate', 'unlinked', 'entered'])
        self.assertEqual(rig.approved[-1]['stamp_ns'], stamp_of(4))
        self.assertIsNotNone(rig.lease.token(stamp_of(4), *rig.clock(rig.t)))

    def test_old_rule_same_input_clears(self):
        # control: the same A, unlinked, B, unlinked, C on the pre-第 7 棒 rule never enters
        rig = Rig()
        rig.gate = OldEntryRuleGate()
        rig.frame(0)
        rig.frame(1, linked=False)
        self.assertEqual((rig.gate.state, rig.gate.reason, rig.gate.streak),
                         ('initializing', 'entry_reset:unlinked', 0))
        for i in (2, 3, 4):
            rig.frame(i, linked=i % 2 == 0)
        self.assertNotEqual(rig.gate.state, 'valid')
        self.assertEqual(rig.approved, [])

    def test_unlinked_output_makes_no_ref_gap_and_is_not_the_jump_basis(self):
        rig = Rig()
        rig.frame(0)
        rig.frame(1, linked=False, correction=planar(9., 9., 120.))   # wild pose, but unlinked
        rig.frame(2)
        self.assertEqual(rig.gate.streak, 2)
        rows = rig.gate.drain_corrections()
        self.assertEqual([c['verdict'] for c in rows], ['candidate', 'unlinked', 'candidate'])
        self.assertEqual(rows[-1]['translation_m'], 0.)        # measured against A, not the unlinked one

    def test_jump_between_linked_candidates_across_unlinked_does_not_enter(self):
        off = planar(1.7, -2., 30.)                            # 20 cm away from TRUE_M
        rig = Rig()
        rig.frame(0)                                           # A
        rig.frame(1, linked=False)
        rig.frame(2, correction=off)                           # B: beyond 10 cm of A
        rig.frame(3, linked=False)
        rig.frame(4)                                           # C: beyond 10 cm of B
        self.assertNotEqual(rig.gate.state, 'valid')
        self.assertEqual(rig.gate.streak, 1)                   # C restarted the count
        self.assertEqual(rig.approved, [])
        self.assertEqual(sum(rig.gate.revokes.values()), 0)
        rig.frame(5, linked=False)
        rig.frame(6)
        self.assertEqual(rig.gate.state, 'confirming')
        rig.frame(7)
        self.assertEqual(rig.gate.state, 'valid')
        self.assertLessEqual(np.abs(np.asarray(rig.approved[-1]['map_odom']) - TRUE_M).max(), 1e-9)

    def test_candidates_age_out_between_unlinked_outputs(self):
        rig = Rig()
        rig.frame(0)
        rig.frame(1)
        for i in range(2, 21):                                 # unlinked outputs only
            rig.frame(i, linked=False)
            if i < 20:                                         # frame 20's output arrives 2.05 s after B
                self.assertEqual((rig.gate.state, rig.gate.streak), ('confirming', 2), i)
        self.assertEqual((rig.gate.state, rig.gate.reason), ('initializing', 'entry_reset:entry_stale'))
        self.assertEqual(rig.gate.streak, 0)
        rig.frame(22)                                          # starts a new count
        rig.frame(23, linked=False)
        rig.frame(24)
        self.assertEqual((rig.gate.state, rig.gate.streak), ('confirming', 2))
        rig.frame(25)
        self.assertEqual(rig.gate.state, 'valid')
        self.assertEqual(rig.gate.epoch, 0)

    def test_ref_gap_still_restarts_with_unlinked_between(self):
        rig = Rig()
        rig.frame(0)
        rig.frame(1, linked=False)
        rig.frame(2, gap=1)                                    # one rtabmap output lost before it
        self.assertEqual((rig.gate.state, rig.gate.streak), ('confirming', 1))
        rig.frame(3, linked=False)
        rig.frame(4)
        self.assertEqual(rig.gate.state, 'confirming')
        rig.frame(5)
        self.assertEqual(rig.gate.state, 'valid')

    def test_no_prior_link_at_all_never_enters(self):
        rig = Rig()
        for i in range(40):
            rig.frame(i, linked=False)
        self.assertEqual((rig.gate.state, rig.gate.streak, rig.gate.adopted), ('initializing', 0, 0))
        self.assertEqual(rig.approved, [])

    def test_valid_period_unchanged(self):
        rig = Rig()
        rig.enter()
        held = rig.gate.held[0]
        rig.frame(3, linked=False)
        self.assertEqual((rig.gate.state, rig.gate.held[0]), ('valid', held))
        i = 4
        while rig.gate.state == 'valid':                       # only unlinked outputs: ages to X
            rig.frame(i, linked=False)
            i += 1
        self.assertEqual(rig.gate.reason, 'correction_too_old')
        # held stamp frame 2 (0.2 s); frame 21's unlinked output arrives at 2.25 s, the first input > X
        self.assertEqual(i - 1, 21)



class EntryJumpLedTests(unittest.TestCase):
    """第 8 棒 (2026-10-16, owner 23:40): a lock whose candidate 1 is the correction a jump just revoked on
    may not be completed across an unlinked output -- there the pre-第 7 棒 rule applies (the count
    restarts, entry_reset:unlinked). Every other lock keeps the 第 7 棒 rule. Under e5f61b3 (第 7 棒)
    test_rejected_lead_across_unlinked_restarts enters at frame 6 (the form seen on g7r_1 24.34 s)."""

    OFF = planar(1.7, -2., 30.)                                # 20 cm from TRUE_M: beyond the jump limit

    def verdicts(self, rig):
        return [c['verdict'] for c in rig.gate.drain_corrections()]

    def jumped(self):
        rig = Rig()
        rig.enter()                                            # frames 0-2 on TRUE_M
        rig.gate.drain_corrections()
        rig.frame(3, correction=self.OFF)                      # jump: revoked, candidate 1 of a new lock
        before = len(rig.approved)                             # incl. the one at frame 3's pair (still valid then)
        self.assertEqual((rig.gate.state, rig.gate.reason, rig.gate.streak),
                         ('confirming', 'confirming_after_correction_jump', 1))
        self.assertEqual((rig.gate.epoch, rig.gate.jumps), (1, 1))
        return rig, before

    def test_rejected_lead_across_unlinked_restarts(self):
        for n_unlinked in (1, 4):                              # g7r_1: 1 (44.37 s) and 4 (24.34 s)
            rig, before = self.jumped()
            for k in range(n_unlinked):
                rig.frame(4 + k, linked=False)
                self.assertEqual((rig.gate.state, rig.gate.reason, rig.gate.streak),
                                 ('initializing', 'entry_reset:unlinked', 0), (n_unlinked, k))
            i = 4 + n_unlinked
            rig.frame(i, correction=self.OFF)                  # consistent with the rejected one
            rig.frame(i + 1, correction=self.OFF)
            # the rejected one + 2 consistent linked ones across unlinked: 第 7 棒 entered here
            self.assertEqual((rig.gate.state, rig.gate.streak), ('confirming', 2), n_unlinked)
            self.assertEqual(len(rig.approved), before, n_unlinked)
            # this lock was not started by a rejected correction: it may cross unlinked (第 7 棒 rule)
            rig.frame(i + 2, linked=False)
            self.assertEqual((rig.gate.state, rig.gate.streak), ('confirming', 2), n_unlinked)
            rig.frame(i + 3, correction=self.OFF)
            self.assertEqual((rig.gate.state, rig.gate.reason), ('valid', 'three_consistent_corrections'))
            self.assertEqual((rig.gate.epoch, sum(rig.gate.revokes.values())), (1, 1), n_unlinked)
            self.assertEqual(self.verdicts(rig), ['jump'] + ['unlinked'] * n_unlinked
                             + ['candidate', 'candidate', 'unlinked', 'entered'])
            self.assertLessEqual(np.abs(np.asarray(rig.approved[-1]['map_odom']) - self.OFF).max(), 1e-9)

    def test_rejected_lead_without_unlinked_enters_as_before(self):
        rig, before = self.jumped()
        rig.frame(4, correction=self.OFF)
        self.assertEqual(rig.gate.state, 'confirming')
        rig.frame(5, correction=self.OFF)
        self.assertEqual((rig.gate.state, rig.gate.epoch), ('valid', 1))
        self.assertEqual(len(rig.approved), before + 1)
        self.assertEqual(self.verdicts(rig), ['jump', 'candidate', 'entered'])

    def test_lock_restarted_after_the_jump_may_cross_unlinked(self):
        # the rejected one is dropped by an inconsistent candidate or a ref gap: the new candidate 1
        # is not a rejected correction, so its lock keeps the 第 7 棒 rule
        for name, spoil in (('inconsistent', {}), ('ref_gap', {'gap': 1, 'correction': self.OFF})):
            rig, before = self.jumped()
            rig.frame(4, **spoil)                              # TRUE_M 20 cm from the rejected one / gap
            self.assertEqual((rig.gate.state, rig.gate.streak), ('confirming', 1), name)
            rig.frame(5, linked=False)
            self.assertEqual((rig.gate.state, rig.gate.reason, rig.gate.streak),
                             ('confirming', 'confirming_corrections', 1), name)
            rig.frame(6, correction=spoil.get('correction', TRUE_M))
            rig.frame(7, linked=False)
            rig.frame(8, correction=spoil.get('correction', TRUE_M))
            self.assertEqual((rig.gate.state, rig.gate.epoch), ('valid', 1), name)
            self.assertEqual(self.verdicts(rig)[1:], ['candidate', 'unlinked', 'candidate', 'unlinked', 'entered'], name)

    def test_lock_after_entry_stale_may_cross_unlinked(self):
        rig, before = self.jumped()
        for i in range(4, 26):                                 # no rtabmap output for > X after the jump
            rig.frame(i, rtab=False)
        self.assertEqual((rig.gate.state, rig.gate.reason, rig.gate.streak),
                         ('initializing', 'entry_reset:entry_stale', 0))
        rig.frame(26, correction=self.OFF)
        rig.frame(27, linked=False)
        self.assertEqual((rig.gate.state, rig.gate.streak), ('confirming', 1))
        rig.frame(28, correction=self.OFF)
        rig.frame(29, linked=False)
        rig.frame(30, correction=self.OFF)
        self.assertEqual((rig.gate.state, rig.gate.epoch), ('valid', 1))

    def test_lock_after_other_revokes_may_cross_unlinked(self):
        # only a jump marks the lock; a lock after correction_too_old / odometry lost keeps the 第 7 棒 rule
        for name in ('correction_too_old', 'odom_info:odometry_lost'):
            rig = Rig()
            rig.enter()
            if name == 'correction_too_old':
                i = 3
                while rig.gate.state == 'valid':
                    rig.frame(i, linked=False)
                    i += 1
            else:
                rig.pair(3, lost=True)
                rig.status()
                i = 4
            self.assertEqual((rig.gate.revokes[name], rig.gate.jumps), (1, 0), name)
            self.assertNotEqual(rig.gate.state, 'valid', name)
            rig.frame(i)
            rig.frame(i + 1, linked=False)
            self.assertEqual((rig.gate.state, rig.gate.streak), ('confirming', 1), name)
            rig.frame(i + 2)
            rig.frame(i + 3, linked=False)
            rig.frame(i + 4)
            self.assertEqual((rig.gate.state, rig.gate.epoch), ('valid', 1), name)

    def test_revoke_during_a_rejected_led_lock_clears_the_mark(self):
        # odometry reported lost while the rejected-led lock is confirming: that lock is gone, the next
        # one (not led by a rejected correction) may cross unlinked
        rig, before = self.jumped()
        rig.pair(4, lost=True)
        rig.status()
        self.assertEqual((rig.gate.state, rig.gate.revokes['odom_info:odometry_lost'], rig.gate.streak), ('lost', 1, 0))
        rig.frame(5, correction=self.OFF)
        rig.frame(6, linked=False)
        self.assertEqual((rig.gate.state, rig.gate.streak), ('confirming', 1))
        rig.frame(7, correction=self.OFF)
        rig.frame(8, correction=self.OFF)
        self.assertEqual(rig.gate.state, 'valid')

    def test_entering_clears_the_mark(self):
        # a rejected-led lock that entered: after its later leave, a lock not led by a rejected one
        # may cross unlinked again (the mark does not outlive the lock)
        rig, before = self.jumped()
        rig.frame(4, correction=self.OFF)
        rig.frame(5, correction=self.OFF)
        self.assertEqual(rig.gate.state, 'valid')
        i = 6
        while rig.gate.state == 'valid':                       # ages out on unlinked outputs only
            rig.frame(i, linked=False)
            i += 1
        self.assertEqual(rig.gate.reason, 'correction_too_old')
        rig.frame(i, correction=self.OFF)
        rig.frame(i + 1, linked=False)
        rig.frame(i + 2, correction=self.OFF)
        rig.frame(i + 3, correction=self.OFF)
        self.assertEqual((rig.gate.state, rig.gate.epoch), ('valid', 2))


class ValidTests(unittest.TestCase):
    def test_every_pair_while_valid_is_one_approval_and_the_lease_follows(self):
        rig = Rig()
        rig.enter()
        for i in range(3, 30):
            rig.frame(i, rtab=(i % 3 == 0))
            wall, ros = rig.clock(rig.t)
            self.assertIsNotNone(rig.lease.token(stamp_of(i), wall, ros), i)
        self.assertEqual([a['stamp_ns'] for a in rig.approved], [stamp_of(i) for i in range(2, 30)])
        self.assertEqual(rig.gate.revokes, collections.Counter())

    def test_synthesized_pose_is_held_correction_times_odometry(self):
        rig = Rig()
        rig.enter()
        rig.pair(3)
        a = rig.approved[-1]
        self.assertTrue(pose_valid(a['pose']['pose']))
        self.assertTrue(rigid_valid(a['map_odom']))
        expected = np.asarray(a['map_odom']) @ odom_matrix(3)
        self.assertLessEqual(np.abs(pose_matrix(a['pose']['pose']) - expected).max(), 1e-9)
        self.assertLessEqual(np.abs(pose_matrix(a['pose']['pose']) - TRUE_M @ odom_matrix(3)).max(), 1e-9)
        self.assertEqual(a['pose']['covariance'], GOOD_COV)

    def test_unlinked_info_and_one_lost_output_neither_refresh_nor_revoke(self):
        rig = Rig()
        rig.enter()
        held = rig.gate.held[0]
        rig.frame(3, linked=False)
        self.assertEqual((rig.gate.state, rig.gate.held[0]), ('valid', held))
        rig.frame(4, gap=1)                                    # one rtabmap output lost before it
        self.assertEqual((rig.gate.state, rig.gate.held[0]), ('valid', stamp_of(4)))
        self.assertEqual(rig.gate.revokes, collections.Counter())
        verdicts = [c['verdict'] for c in rig.gate.drain_corrections()]
        self.assertEqual(verdicts[-2:], ['unlinked', 'adopted_after_gap'])
        self.assertEqual(rig.gate.drain_corrections(), [])

    def test_held_correction_older_than_x_revokes(self):
        for x, frames in ((2., 20), (3., 30)):
            rig = Rig(max_correction_age_s=x)
            rig.enter()
            i = 3
            while rig.gate.state == 'valid':
                rig.frame(i, rtab=False)                       # odometry continues, rtabmap silent
                i += 1
            self.assertEqual(rig.gate.reason, 'correction_too_old')
            # held stamp is frame 2; revoked at the first status more than x after it
            self.assertEqual(i - 1, 2 + frames, x)

    def test_odometry_stop_revokes_only_after_one_second(self):
        rig = Rig()
        rig.enter()
        rig.frame(3)                                           # last pair: stamp .3 s, arrived .32 s
        rig.status(t=1.3)                                      # 1.0 s after its stamp
        self.assertEqual(rig.gate.state, 'valid')
        self.assertIsNotNone(rig.lease.row)
        rig.status(t=1.31)
        self.assertEqual((rig.gate.state, rig.gate.reason), ('stale', 'odometry_stale'))

    def test_odometry_lost_and_bad_values_revoke_immediately(self):
        for name, act in (('odometry_lost', lambda r: r.pair(3, lost=True)),
                          ('invalid_covariance', lambda r: r.send('odom', 3, .02, pose={
                              'pose': matrix_pose(odom_matrix(3)), 'covariance': [float('nan')] * 36})),
                          ('missing_ref_id', lambda r: r.send('info', 3, .1, ref=None)),
                          ('future_stamp', lambda r: r.send('odom', 9, -.5)),
                          ('wrong_frame', lambda r: r.send('localization_pose', 3, .1, header={
                              'stamp': {'sec': 1, 'nanosec': 3 * 10**8}, 'frame_id': 'odom'}))):
            rig = Rig()
            rig.enter()
            epoch = rig.gate.epoch
            act(rig)
            self.assertNotEqual(rig.gate.state, 'valid', name)
            self.assertEqual(rig.gate.epoch, epoch + 1, name)
            self.assertTrue(any(name in reason for reason in rig.gate.revokes), (name, rig.gate.revokes))
            self.assertEqual(rig.status()['approved'], [], name)

    def test_incomplete_output_revokes_but_odometry_only_frames_are_silent(self):
        rig = Rig()
        rig.enter()
        for i in range(3, 12):
            rig.frame(i, rtab=(i % 4 == 0))                    # many frames rtabmap skipped
        self.assertEqual(rig.gate.revokes, collections.Counter())
        self.assertLessEqual(len(rig.gate.pending), 4)
        rig.ref += 1
        rig.send('info', 13, .15, ref=rig.ref)                 # rtabmap output whose odometry never comes
        rig.status(t=1.3 + .15 + .49)
        self.assertEqual(rig.gate.state, 'valid')
        rig.status(t=1.3 + .15 + .5)
        self.assertEqual(rig.gate.revokes, collections.Counter({'incomplete_output': 1}))

    def test_duplicate_and_overflow(self):
        rig = Rig()
        rig.enter()
        rig.pair(3)
        rig.send('odom', 3, .03)
        self.assertEqual(rig.gate.revokes['duplicate_message'], 1)
        gate = Gate()
        for k in range(65):                                    # 65 open odometry items 10 ms apart
            stamp = BASE + k * 10**7
            gate.ingest('odom', {'header': {'stamp': {'sec': stamp // 10**9, 'nanosec': stamp % 10**9},
                                            'frame_id': 'odom'}, 'child_frame_id': 'base_link',
                                 'pose': {'pose': matrix_pose(np.eye(4)), 'covariance': GOOD_COV}},
                        100., BASE + 7 * 10**8)
            self.assertEqual(gate.pending_overflows, int(k == 64))
        rig = Rig()
        rig.gate = gate
        self.assertEqual((rig.gate.pending_overflows, rig.gate.reason), (1, 'pending_overflow'))

    def test_x_is_validated(self):
        for bad in (0, -1., float('nan'), float('inf'), '2', True, None):
            with self.assertRaises(ValueError, msg=repr(bad)):
                LocalizationGate(MAP, PRIOR, count_mode='correction', max_correction_age_s=bad)
        self.assertEqual(COUNT_MODES, ('frame', 'output', 'correction'))
        self.assertEqual(LocalizationGate(MAP, PRIOR).count_mode, 'frame')   # class default unchanged

    def test_status_floor_survives_revoke_and_discontinuity_rollback_renews_instance(self):
        rig = Rig()
        rig.enter()
        rig.frame(3)
        floor = rig.status()['stamp_ns']
        rig.gate.discontinuity(1)
        row = rig.status()
        self.assertEqual((row['state'], row['stamp_ns']), ('initializing', floor))
        instance = rig.gate.instance
        rig.gate.status(50., BASE)                             # clocks went back
        self.assertNotEqual(rig.gate.instance, instance)
        self.assertEqual(rig.gate.reason, 'clock_rollback')
        self.assertEqual(rig.gate.last_complete, 0)

    def test_acquisition_is_a_clock_tick_only(self):
        rig = Rig()
        self.assertEqual(rig.gate.acquisition(stamp_of(0), 100., BASE + 10**8), [])
        self.assertEqual((rig.gate.state, rig.gate.revokes), ('initializing', collections.Counter()))


class JumpTests(unittest.TestCase):
    """RELAX_EVAL §1 jump controls; D-035 limits 10 cm at the tracked point / 5 deg."""

    def run_with(self, offsets):
        rig = Rig()
        rig.enter()
        held = []
        for i in range(3, 40):
            rig.frame(i, correction=offsets.get(i, TRUE_M))
            if rig.gate.state == 'valid':
                held.append(rig.gate.held[0])
        return rig, held

    def test_single_outlier_is_never_adopted(self):
        for name, outlier in (('20cm', planar(1.5, -2., 30.) @ planar(.2, 0., 0.)),
                              ('8deg', TRUE_M @ planar(0., 0., 8.))):
            rig, held = self.run_with({20: outlier})
            self.assertEqual(rig.gate.jumps, 1, name)
            self.assertEqual(rig.gate.epoch, 1, name)
            self.assertNotIn(stamp_of(20), held, name)
            self.assertEqual(rig.gate.revokes, collections.Counter({'correction_jump': 1}), name)
            self.assertEqual(rig.gate.state, 'valid', name)    # relocked on the true correction

    def test_small_change_is_adopted(self):
        rig, held = self.run_with({20: TRUE_M @ planar(.06, 0., 3.)})
        self.assertEqual((rig.gate.jumps, rig.gate.epoch), (0, 0))
        self.assertIn(stamp_of(20), held)

    def test_persistent_shift_relocks_on_the_new_correction(self):
        shifted = TRUE_M @ planar(.3, 0., 0.)
        rig, held = self.run_with({i: shifted for i in range(20, 40)})
        self.assertEqual((rig.gate.jumps, rig.gate.epoch), (1, 1))
        self.assertEqual(rig.gate.state, 'valid')
        self.assertLessEqual(np.abs(rig.gate.held[1] - shifted).max(), 1e-9)
        self.assertEqual(min(s for s in held if s >= stamp_of(20)), stamp_of(22))
        self.assertTrue(any(c['verdict'] == 'jump' and c['translation_m'] >= .1
                            for c in rig.gate.drain_corrections()))


class MatrixPoseTests(unittest.TestCase):
    def test_round_trip_including_near_half_turns(self):
        rng = np.random.default_rng(20261015)
        cases = [planar(1., 2., 179.9999), planar(-3., 0., -180.), planar(0., 0., 0.)]
        for axis in np.eye(3):
            angle = math.pi - 1e-7
            k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
            m = np.eye(4)
            m[:3, :3] = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k
            cases.append(m)
        for _ in range(200):
            q = rng.normal(size=4)
            q /= np.linalg.norm(q)
            pose = {'position': dict(zip('xyz', rng.normal(size=3) * 10)),
                    'orientation': dict(zip('xyzw', q))}
            cases.append(pose_matrix(pose))
        for m in cases:
            pose = matrix_pose(m)
            self.assertTrue(pose_valid(pose))
            self.assertLessEqual(np.abs(pose_matrix(pose) - m).max(), 1e-9)


# ------------------------------------------------------------------ recorded host v6a2 run
def replay(data, x=2., tail_s=3., info=None, lost=None, keep_rtab=None, drop_kinds=(), drop_after=None,
           count_mode='correction', gate_cls=None):
    """The fixture stream of test_localization_gate_output_mode plus (like relax_sim.py) 100 ms
    ticks for tail_s past the last event; every tick's status row goes through a real lease."""
    info = data['info'] if info is None else info
    lost = data['lost'] if lost is None else lost
    stream = build_stream(data, keep_rtab, drop_kinds)
    if drop_after is not None:
        stream = [x_ for x_ in stream if x_[2] != 'msg' or x_[4] <= drop_after]
    first, last = min(e[0] for e in stream), max(e[0] for e in stream)
    offset = min(e[1] - e[0] for e in stream)
    t = first + -(-(last - first) // (100 * 10**6)) * 100 * 10**6
    while t < last + tail_s * 1e9:
        stream.append([t, t + offset, 'tick', None, None, None])
        t += 100 * 10**6
    stream.sort(key=lambda e: (e[0], {'tick': 1, 'acq': 0, 'msg': 0}[e[2]]))
    previous = 0
    for e in stream:
        e[1] = previous = max(e[1], previous)
    gate_cls = gate_cls or Gate
    gate = (gate_cls(data['prior_ids'], max_correction_age_s=x) if count_mode == 'correction'
            else gate_cls(data['prior_ids'], count_mode=count_mode))
    lease = LocalizationLease(MAP, 'base_link')
    first_output = next(e[0] for e in stream if e[2] == 'msg')
    timeline, ticks, approvals = [], [], []
    valid_rows = voided = expected_generation = 0
    last_identity = None
    for mono, ros, op, kind, stamp, _ in stream:
        wall = mono / 1e9
        if op == 'acq':
            out = gate.acquisition(stamp, wall, ros)
        elif op == 'msg':
            out = gate.ingest(kind, make_row(kind, stamp, data, info, lost), wall, ros)
        else:
            out = []
            row = gate.status(wall, ros)
            identity = (row['instance'], row['epoch'])
            expected_generation += identity != last_identity or row['state'] != 'valid'
            last_identity = identity
            lease.update(row, wall, ros)
            if row['state'] == 'valid':
                valid_rows += 1
                voided += lease.row is None
            ticks.append((mono, lease.check(wall, ros)))
        approvals += [(mono, ros, a['stamp_ns']) for a in out]
        timeline.append((mono, gate.state))
    end = timeline[-1][0]
    valid = longest = run = exits = 0
    first_valid = previous_state = None
    for (m0, s0), (m1, _) in zip(timeline, timeline[1:]):
        if s0 == 'valid':
            valid += m1 - m0
            run += m1 - m0
            longest = max(longest, run)
            first_valid = m0 if first_valid is None else first_valid
        else:
            run = 0
        exits += previous_state == 'valid' and s0 != 'valid'
        previous_state = s0
    after = [ok for m, ok in ticks if m >= first_output]
    return {'gate': gate, 'timeline': timeline, 'ticks': ticks, 'approvals': approvals,
            'adopted': gate.adopted, 'synthesized': gate.synthesized,
            'first_valid_s': None if first_valid is None else round((first_valid - first_output) / 1e9, 2),
            'valid_share': valid / (end - first_output), 'longest_s': round(longest / 1e9, 2),
            'exits': exits, 'lease_share': sum(after) / max(1, len(after)),
            'valid_rows': valid_rows, 'voided_valid_rows': voided,
            'generation_extra': lease.generation - expected_generation}


class OldEntryRuleGate(Gate):
    """The entry rule before 第 7 棒 (main 0ddb379): an unlinked output during entry cleared the
    candidates (entry_reset:unlinked). Same code path otherwise: the valid branch comes first."""

    def accept_correction(self, stamp, item):
        if self.state != 'valid' and not item['linked']:
            ref = item['messages']['info']['ref_id']
            self.last_ref = ref
            self.log_correction(stamp, ref, 'unlinked')
            self.soft_reset('unlinked')
            return []
        return super().accept_correction(stamp, item)


@unittest.skipUnless(FIXTURE.exists(), 'replay fixture missing')
class RecordedReplayTests(unittest.TestCase):
    """Known answers: relax_results.json host_as_recorded C2(X=2) and its negative controls (old entry
    rule, OldEntryRuleGate); the current rule's numbers on the same replay (第 7 棒) next to them."""

    @classmethod
    def setUpClass(cls):
        cls.data = load_fixture()
        cls.base = replay(cls.data)
        cls.old = replay(cls.data, gate_cls=OldEntryRuleGate)

    def test_v6a2_old_entry_rule_matches_the_simulator(self):
        r = self.old
        self.assertEqual(r['adopted'], 124)                    # simulator 124
        self.assertEqual(r['first_valid_s'], 3.35)             # 3.35
        self.assertAlmostEqual(r['valid_share'], .767, delta=.005)   # .767 (here .768)
        self.assertAlmostEqual(r['lease_share'], .765, delta=.005)   # .765 (here .763)
        self.assertEqual(r['longest_s'], 29.4)                 # 29.4
        self.assertEqual(r['exits'], 4)                        # 4
        self.assertEqual(r['gate'].jumps, 0)
        self.assertEqual(dict(r['gate'].revokes), {'odom_info:invalid_covariance': 3, 'odom_info:odometry_lost': 2,
                                                   'correction_too_old': 2, 'odom:invalid_covariance': 1,
                                                   'odom:invalid_pose': 1})   # simulator's, exactly
        self.assertEqual(r['gate'].pending_overflows, 0)
        # simulator 361 = one pose per odom message while valid. Here: one per healthy odom +
        # odom_info pair (360) plus one at each of the 4 entries into valid.
        self.assertEqual(r['synthesized'], 364)
        self.assertEqual(len(r['approvals']), 364)

    def test_v6a2_current_entry_rule(self):
        # 第 7 棒: unlinked outputs during entry no longer clear the candidates. Old rule -> now on the
        # same replay: adopted 124 -> 135, first valid 3.35 -> 2.15 s, valid share .768 -> .833, lease
        # .763 -> .831, longest 29.4 -> 32.02 s, synthesized 364 -> 400; exits (4), jumps (0) and every
        # revoke count unchanged (the rule touches entry only).
        r, old = self.base, self.old
        self.assertEqual(r['adopted'], 135)
        self.assertEqual(r['first_valid_s'], 2.15)
        self.assertAlmostEqual(r['valid_share'], .833, delta=.002)
        self.assertAlmostEqual(r['lease_share'], .831, delta=.002)
        self.assertEqual(r['longest_s'], 32.02)
        self.assertEqual(r['synthesized'], 400)
        self.assertEqual((r['exits'], r['gate'].jumps, r['gate'].pending_overflows), (4, 0, 0))
        self.assertEqual(dict(r['gate'].revokes), dict(old['gate'].revokes))
        self.assertGreater(r['adopted'], old['adopted'])
        self.assertLess(r['first_valid_s'], old['first_valid_s'])
        self.assertEqual(sum(1 for c in r['gate'].corrections if c['verdict'] == 'unlinked'),
                         sum(1 for c in old['gate'].corrections if c['verdict'] == 'unlinked'))

    def test_every_valid_status_is_accepted_by_the_lease(self):
        r = self.base
        self.assertGreater(r['valid_rows'], 500)
        self.assertEqual(r['voided_valid_rows'], 0)
        self.assertEqual(r['generation_extra'], 0)             # no invalidate(): only identity/state changes

    def test_positive_control_output_mode_rows_are_voided(self):
        # RELAX_EVAL: output mode's valid rows name a stamp the row does not approve (20 of 83).
        r = replay(self.data, count_mode='output')
        self.assertGreater(r['valid_rows'], 0)
        self.assertGreater(r['voided_valid_rows'], 0)
        self.assertGreater(r['generation_extra'], 0)

    def test_approvals_are_in_stamp_order_and_fresh(self):
        stamps = [s for _, _, s in self.base['approvals']]
        self.assertEqual(stamps, sorted(stamps))
        self.assertLess(max(ros - s for _, ros, s in self.base['approvals']), 10**9)

    def test_negative_controls(self):
        nolink = {s: [0, 0, v[2]] for s, v in self.data['info'].items()}
        all_lost = {s: True for s in self.data['lost']}
        for name, kw in (('no_prior_link', {'info': nolink}), ('all_odom_lost', {'lost': all_lost}),
                         ('no_odom', {'drop_kinds': ('odom',)}), ('no_odom_info', {'drop_kinds': ('odom_info',)})):
            r = replay(self.data, **kw)
            self.assertEqual((r['adopted'], r['synthesized'], r['first_valid_s']), (0, 0, None), name)

    def test_every_fourth_rtabmap_output_lost(self):
        stamps = sorted(self.data['info'])
        keep = {s for n, s in enumerate(stamps) if n % 4 != 3}
        old = replay(self.data, keep_rtab=keep, gate_cls=OldEntryRuleGate)
        self.assertEqual(old['adopted'], 97)                   # simulator 97 (old entry rule)
        self.assertAlmostEqual(old['valid_share'], .749, delta=.005)
        r = replay(self.data, keep_rtab=keep)                  # 第 7 棒 rule: 97 -> 100, .749 -> .771
        self.assertEqual(r['adopted'], 100)
        self.assertAlmostEqual(r['valid_share'], .771, delta=.002)
        for x in (old, r):
            self.assertEqual(x['gate'].revokes['rtabmap_output_gap'], 0)

    def injection(self, mode, T):
        kw = {}
        if mode == 'outputs_stop':
            kw['drop_after'] = T
        elif mode == 'leave_map':
            kw['info'] = {s: ([0, 0, v[2]] if s > T else v) for s, v in self.data['info'].items()}
        else:
            kw['lost'] = {s: (True if s > T else v) for s, v in self.data['lost'].items()}
        r = replay(self.data, **kw)
        codes = self.data['event_codes']
        wanted = {'outputs_stop': None, 'leave_map': 'received_info', 'odom_lost': 'received_odom_info'}[mode]
        arrivals = [e[1] for e in self.data['events'] if codes[e[0]].startswith('received_') and e[3] > T
                    and (wanted is None or codes[e[0]] == wanted)]
        inject = min(arrivals)
        before = [s for m, s in r['timeline'] if m < inject]
        if not before or before[-1] != 'valid':
            return None
        leave = next(m for m, s in r['timeline'] if m >= inject and s != 'valid')
        lease_off = next(m for m, ok in r['ticks'] if m >= inject and not ok)
        adopted_after = [c for c in r['gate'].corrections if c['stamp_ns'] > T and c['verdict'] != 'unlinked']
        return (leave - inject) / 1e6, (lease_off - inject) / 1e6, adopted_after

    def test_mid_run_injections(self):
        stamps = sorted(self.data['info'])
        points = [s for n, s in enumerate(stamps) if n % 6 == 0 and stamps[0] + 3e9 < s < stamps[-1] - 3e9][::3]
        limits = {'outputs_stop': 1000., 'leave_map': 2000., 'odom_lost': 100.}   # simulator max 830 / 1515 / 0
        for mode, limit in limits.items():
            seen = 0
            for T in points:
                result = self.injection(mode, T)
                if result is None:
                    continue
                seen += 1
                gate_ms, lease_ms, adopted_after = result
                self.assertLessEqual(gate_ms, limit, (mode, T))
                self.assertLessEqual(lease_ms, limit + 100., (mode, T))
                if mode != 'outputs_stop':
                    self.assertFalse([c for c in adopted_after if c['verdict'] in ('adopted', 'entered')], (mode, T))
            self.assertGreaterEqual(seen, 3, mode)


CAR = Path(__file__).resolve().parents[1] / 'tools' / 'car_run'


class NodeAndScriptWiringTests(unittest.TestCase):
    """By source (no ROS here): node options, no camera subscription, the corrections field, and the
    car_up.sh -> car_inner.sh -> rtabmap_car.sh -> gate CLI chain for the mode and X."""

    source = inspect.getsource(node.main)

    def test_node_options_and_correction_wiring(self):
        self.assertIn("'--count-mode', choices=COUNT_MODES, default='correction'", self.source)
        self.assertIn("'--max-correction-age-s', type=float, default=2.", self.source)
        self.assertIn("parser.error('--max-correction-age-s must be a positive finite number')", self.source)
        self.assertIn('max_correction_age_s=args.max_correction_age_s', self.source)
        subscribe = self.source[self.source.index("if args.count_mode != 'correction':"):]
        subscribe = subscribe[:subscribe.index('for kind, msg_type in zip(KINDS')]
        self.assertIn('self.acquisition, rgb_qos)', subscribe)
        self.assertEqual(self.source.count('self.acquisition, rgb_qos)'), 1)
        receive = re.search(r'\n        def receive\(.*?(?=\n        def )', self.source, re.S).group(0)
        self.assertIn('corrections = self.gate.drain_corrections()', receive)
        self.assertIn("**({'corrections': corrections} if corrections else {})", receive)

    def test_x_reaches_the_gate_cli_from_car_up(self):
        up = (CAR / 'car_up.sh').read_text()
        inner = (CAR / 'car_inner.sh').read_text()
        rtab = (CAR / 'rtabmap_car.sh').read_text()
        self.assertIn(': "${CAR_RTABMAP_STOP_S:=240}" "${CAR_GATE_MODE:=correction}" "${CAR_GATE_X_S:=2}"', up)
        self.assertIn('RTABMAP_GATE_MODE="$CAR_GATE_MODE" RTABMAP_GATE_X_S="$CAR_GATE_X_S")', up)
        self.assertIn(': "${RTABMAP_GATE_MODE:=correction}" "${RTABMAP_GATE_X_S:=2}"', inner)
        self.assertIn('GATE_COUNT_MODE="$RTABMAP_GATE_MODE" GATE_MAX_CORRECTION_AGE_S="$RTABMAP_GATE_X_S" \\\n'
                      '    bash tools/car_run/rtabmap_car.sh "$RT_MODE"', inner)
        self.assertIn(': "${GATE_MAX_CORRECTION_AGE_S:=2}"', rtab)
        self.assertRegex(rtab, r'PASS=\([^)]*\bGATE_MAX_CORRECTION_AGE_S\b')
        self.assertIn('--max-correction-age-s "$GATE_MAX_CORRECTION_AGE_S"', rtab)
        for text in (up, inner, rtab):                         # the owner's tier rule is written down
            self.assertIn('p90 <= 1.5 s -> 3', re.sub(r'\s*\n#\s*', ' ', text))

    def test_x_validation_in_rtabmap_car(self):
        rtab = (CAR / 'rtabmap_car.sh').read_text()
        line = next(l for l in rtab.splitlines() if l.startswith('case "$GATE_MAX_CORRECTION_AGE_S" in'))
        for value, ok in (('2', 1), ('2.5', 1), ('0.5', 1), ('10', 1), ('0', 0), ('0.0', 0), ('', 0),
                          ('.', 0), ('-1', 0), ('1.2.3', 0), ('2s', 0)):
            out = subprocess.run(['bash', '-c', 'GATE_MAX_CORRECTION_AGE_S="$1"; %s; echo "$GATE_X_OK"' % line,
                                  '_', value], stdout=subprocess.PIPE, universal_newlines=True, check=True).stdout
            self.assertEqual(out.strip(), str(ok), value)


if __name__ == '__main__':
    unittest.main()
