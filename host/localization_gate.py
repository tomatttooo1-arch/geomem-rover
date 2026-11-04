"""Fail-closed acquisition-stamped localization policy. No ROS or truth inputs.

Health is not accuracy. Only links to IDs in the pinned prior map are accepted.
An epoch revokes all in-flight consumers across any loss, timeout or restart.

An approval also carries the `map->odom` correction it implies. That is an observation,
not a verdict: the gate applies no threshold to it, derives no "the map moved" flag and
never advances an epoch because of it. Deciding what a correction means is the consumer's,
and it is not the same object as an offline keyframe revision.
"""
import collections
from collections import OrderedDict
import copy
import math
import uuid

import numpy as np

KINDS = ('odom', 'odom_info', 'info', 'localization_pose')
# The two streams rtabmap itself publishes once per frame it processed.
RTABMAP_KINDS = ('info', 'localization_pose')

# What "three consecutive healthy acquisitions" counts (owner decision 2026-10-15).
#   frame   every camera frame the gate receives must collect all four outputs within the
#           pending window, in order; an output arriving before its camera frame revokes.
#           The pre-2026-10-15 gate, verbatim. odometry and rtabmap process only a subset of
#           camera frames (host 26.5 %, board 30 fps camera vs ~10 Hz odometry), so this mode
#           can never reach valid on real input (runs/real-car-integ-20261015-results/gate_dig/DIG.md).
#   output  what is counted is each output rtabmap actually produced, consecutively:
#           - a camera frame rtabmap never answered (no info, no localization_pose) is skipped,
#             neither counted nor a failure, once rtabmap answered a newer frame or once the
#             frame times out; a skipped frame that already carries a bad output still revokes;
#           - an output arriving before its camera frame is held (bounded, same 0.5 s / 1 s
#             limits) and joined when the frame arrives; expiry or overflow still revokes;
#           - consecutive counted outputs must carry rtabmap Info.ref_id exactly +1 apart, so a
#             lost rtabmap output resets the count instead of being bridged over.
#           Every other condition is unchanged: same-stamp four healthy outputs, info linked to
#           the prior map, N=3, the 0.5 s pending window, the 1 s stamp-age and stream limits.
#   correction  (owner decision 2026-10-15 evening, "scheme 2"; runs/real-car-integ-20261015-results/
#           gate_dig/RELAX_EVAL.md, reference implementation relax_sim.py C2). What is approved is the
#           map->odom correction rtabmap implies, not every frame's pose; no camera frame is read:
#           - an rtabmap output (info + localization_pose) with its same-stamp odom + odom_info, all
#             four healthy, is one correction M = localization_pose * odom^-1;
#           - entry: 3 linked corrections (Info.ref_id +1 apart counting every rtabmap output),
#             each within the jump limits of the previous linked one; a ref_id gap, an inconsistent
#             candidate or a candidate older than X (`entry_stale`) restarts the count (no new
#             epoch). An unlinked info in between neither counts nor restarts it (2026-10-16 第 7 棒,
#             owner-authorized entry-rule change, gate_dig/GATEFIX_HANDOVER.md 第 6 棒 §5): entry
#             = 3 mutually consistent linked corrections within X, unlinked ones may sit between;
#             except a lock whose candidate 1 is the correction a jump just revoked on (2026-10-16
#             第 8 棒, owner 23:40): an unlinked info there restarts the count (`entry_reset:unlinked`,
#             the pre-第 7 棒 rule), so a rejected correction cannot complete a lock across unlinked;
#           - while valid: a linked correction within the jump limits (10 cm at the tracked point /
#             5 deg, host.nav_localization D-035, same functions) refreshes the held one; one beyond
#             them revokes (new epoch, `correction_jump`) and becomes candidate 1 of a new lock; an
#             unlinked info or one lost rtabmap output refreshes nothing and revokes nothing;
#           - every healthy odom + odom_info pair while valid yields one approval, pose = held M x
#             odom(t) at the odometry stamp t, so a consumer finds an approval at each odometry frame;
#           - revokes: the held correction older than X (`correction_too_old`), the newest healthy
#             odometry pair older than 1 s on either clock (`odometry_stale`), odometry reported lost
#             or any other bad message (immediately), an rtabmap output not completed within 0.5 s.
#           - odometry jump (2026-10-17 第 10 棒, design 第 9 棒 gate_dig/HOST_G5_RESULT.md): every new
#             healthy odom + odom_info pair is compared with the previous one the gate kept (in any
#             state; none after start, rollback or discontinuity -> not checked); a planar step
#             |dxy| > V * dt + E (dt from the two odometry stamps) means the odometry itself jumped,
#             so the held correction no longer fits it: while valid -> revoke (new epoch,
#             `odometry_jump`), before any approval from that pair; during entry -> the candidates
#             are dropped (`entry_reset:odometry_jump`), they were computed against the old odometry.
#             V = max_odom_speed_mps (default 0.5 m/s, above the fixture's ground-truth 0.418 and the
#             car's nav2 0.25-0.26 m/s limits; per platform), E = odom_jump_margin_m (default 0.10 m
#             = the 10 cm jump limit of a correction). A jump inside V * dt + E is not seen (known:
#             host g8r_3 14.50 s, 0.195 m in 0.336 s). odom_jump_check=False skips the whole check:
#             the gate of 4edc10f (第 8 棒) exactly.
#           X = max_correction_age_s (default 2 s). Tiers, measured on the board before a car run
#           (rtabmap localization output rate, p90 of the stamp age at which the gate receives them),
#           then fixed: >= 3 Hz and p90 <= 1.0 s -> X = 2 s; 1-3 Hz and p90 <= 1.5 s -> X = 3 s;
#           < 1 Hz -> no navigation on estimated poses (reduce the board load first).
COUNT_MODES = ('frame', 'output', 'correction')


def stamp_ns(row):
    stamp = row['header']['stamp']
    sec, nano = stamp['sec'], stamp['nanosec']
    if type(sec) is not int or type(nano) is not int or sec < 0 or not 0 <= nano < 10**9:
        raise ValueError('invalid stamp')
    result = sec * 10**9 + nano
    if result <= 0 or row.get('stamp_ns', result) != result:
        raise ValueError('stamp mismatch or zero')
    return result


def covariance_valid(values):
    try:
        cov = np.asarray(values, dtype=float)
        if cov.shape != (36,) or not np.isfinite(cov).all():
            return False
        cov = cov.reshape(6, 6)
        return bool(np.allclose(cov, cov.T, atol=1e-8, rtol=1e-5)
                    and np.linalg.eigvalsh((cov+cov.T)/2).min() >= -1e-8
                    and np.diag(cov).max() < 9999.)
    except (ValueError, TypeError, np.linalg.LinAlgError):
        return False


def pose_valid(pose):
    try:
        xyz = np.array([pose['position'][k] for k in ('x', 'y', 'z')], float)
        q = np.array([pose['orientation'][k] for k in ('x', 'y', 'z', 'w')], float)
        return bool(np.isfinite(xyz).all() and np.isfinite(q).all()
                    and abs(np.linalg.norm(q)-1) <= .001)
    except (KeyError, ValueError, TypeError):
        return False


def matrix_pose(matrix):
    """Inverse of `pose_matrix` for a rigid 4x4: position + unit quaternion (no scipy here).

    Shepperd's method: the branch on the largest of trace / diagonal keeps the divisor away
    from zero, including rotations near 180 degrees where the trace branch alone fails.
    """
    held = np.asarray(matrix, dtype=float)
    r = held[:3, :3]
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0:
        s = 2. * math.sqrt(trace + 1.)
        q = (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, s / 4.
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2. * math.sqrt(1. + r[0, 0] - r[1, 1] - r[2, 2])
        q = s / 4., (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2. * math.sqrt(1. + r[1, 1] - r[0, 0] - r[2, 2])
        q = (r[0, 1] + r[1, 0]) / s, s / 4., (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s
    else:
        s = 2. * math.sqrt(1. + r[2, 2] - r[0, 0] - r[1, 1])
        q = (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, s / 4., (r[1, 0] - r[0, 1]) / s
    q = np.asarray(q) / np.linalg.norm(q)
    return {'position': {k: float(v) for k, v in zip(('x', 'y', 'z'), held[:3, 3])},
            'orientation': {k: float(v) for k, v in zip(('x', 'y', 'z', 'w'), q)}}


def pose_matrix(pose):
    if not pose_valid(pose):
        raise ValueError('invalid pose')
    x, y, z, w = [pose['orientation'][k] for k in ('x', 'y', 'z', 'w')]
    x, y, z, w = np.array([x, y, z, w]) / np.linalg.norm([x, y, z, w])
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = [pose['position'][k] for k in ('x', 'y', 'z')]
    return result


# A transform composed from two validated poses in float64 is rigid to the last few ULPs,
# never exactly. Measured worst residual over the 91 recorded approvals of
# runs/rtabmap-nav2-rehearsal-20260914 and 60000 synthetic uniform-random rotations with
# translations up to 1e6 m: |R R^T - I| <= 2.0e-15, |det(R) - 1| <= 2.3e-15, bottom row 0.0
# (2026-09-21, scratchpad gatefix/measure_rigid_residual.py). The tolerance below leaves
# that honest noise a margin of ~4e5 and still rejects a distortion of one part in 1e9.
RIGID_ATOL = 1e-9


def rigid_valid(matrix, atol=RIGID_ATOL):
    """True for a finite 4x4 homogeneous rigid transform: R orthogonal, det(R)=+1, [0,0,0,1].

    A mirrored frame (det = -1) is orthogonal and must still be rejected, which is why the
    determinant is checked separately from the orthogonality of R.
    """
    try:
        held = np.asarray(matrix, dtype=float)
    except (ValueError, TypeError):
        return False
    if held.shape != (4, 4) or not np.isfinite(held).all():
        return False
    rotation = held[:3, :3]
    try:
        determinant = float(np.linalg.det(rotation))
    except np.linalg.LinAlgError:
        return False
    return bool(np.allclose(rotation @ rotation.T, np.eye(3), rtol=0., atol=atol)
                and abs(determinant - 1.) <= atol
                and np.allclose(held[3], (0., 0., 0., 1.), rtol=0., atol=atol))


class CallbackClock:
    """The one steady/ROS clock reading a node callback is allowed to take.

    The gate decides on seconds of steady time and the event log records nanoseconds. Read
    the clock a second time to write the row and the recorded timestamp disagrees with the
    decision by the callback's own duration, which is not a rounding detail: the offline
    replay of one rehearsal recording crossed the 1.0 s stream timeout at 1.00222 s because
    the row carried a post-callback read while the gate had used the entry read (drift
    analysis 2026-09-15 §3.1). One reading per callback, passed to the gate call, to the
    log row and to the trailing heartbeat alike, makes `wall == monotonic_ns/1e9` true by
    construction — exactly the relation an offline replay reconstructs.

    Unpacks as the `(wall, ros_ns)` pair every gate entry point takes, so a caller cannot
    hand the gate one reading and the log another without it being visible in the source.
    """

    __slots__ = ('monotonic_ns', 'ros_ns')

    def __init__(self, monotonic_ns, ros_ns):
        if (type(monotonic_ns) is not int or type(ros_ns) is not int or ros_ns < 0):
            raise ValueError('integer nanosecond steady and ROS clocks required')
        self.monotonic_ns, self.ros_ns = monotonic_ns, ros_ns

    @property
    def wall(self):
        """Steady seconds for the gate. Exactly the recorded nanoseconds, never re-read."""
        return self.monotonic_ns / 1e9

    def __iter__(self):
        return iter((self.wall, self.ros_ns))

    def __eq__(self, other):
        return (isinstance(other, CallbackClock) and other.monotonic_ns == self.monotonic_ns
                and other.ros_ns == self.ros_ns)

    def __hash__(self):
        return hash((self.monotonic_ns, self.ros_ns))

    def __repr__(self):
        return 'CallbackClock(%d, %d)' % (self.monotonic_ns, self.ros_ns)


def event_row(event, gate, clock, **fields):
    """One `gate-events.jsonl` row. `clock` must be the reading the decision consumed.

    Field names and order are the recorded schema; only the source of the timestamps is
    the point of this function existing.
    """
    if not isinstance(clock, CallbackClock):
        raise ValueError('the recorded clock must be the callback reading')
    return {'event': event, 'monotonic_ns': clock.monotonic_ns, 'ros_ns': clock.ros_ns,
            'state': gate.state, 'reason': gate.reason, 'epoch': gate.epoch,
            'map_id': gate.map_id, 'instance': gate.instance,
            'pending_count': len(gate.pending), 'approved_count': len(gate.approved), **fields}


class LocalizationGate:
    def __init__(self, map_id, prior_ids, tracked_frame='camera_link', world_frame='map', odom_frame='odom',
                 pose_source='estimated', navigation_ready=False, odom_source='rtabmap_rgbd_odometry',
                 count_mode='frame', max_correction_age_s=2., odom_jump_check=True,
                 max_odom_speed_mps=.5, odom_jump_margin_m=.1):
        if not map_id or not prior_ids or not all(type(i) is int and i > 0 for i in prior_ids):
            raise ValueError('pinned map identity and nonempty prior node IDs required')
        if count_mode not in COUNT_MODES:
            raise ValueError('count_mode must be one of %r' % (COUNT_MODES,))
        if (type(max_correction_age_s) not in (int, float) or not math.isfinite(max_correction_age_s)
                or max_correction_age_s <= 0):
            raise ValueError('max_correction_age_s must be a positive finite number of seconds')
        if type(odom_jump_check) is not bool:
            raise ValueError('odom_jump_check must be True or False')
        if (type(max_odom_speed_mps) not in (int, float) or not math.isfinite(max_odom_speed_mps)
                or max_odom_speed_mps <= 0):
            raise ValueError('max_odom_speed_mps must be a positive finite speed in m/s')
        if (type(odom_jump_margin_m) not in (int, float) or not math.isfinite(odom_jump_margin_m)
                or odom_jump_margin_m < 0):
            raise ValueError('odom_jump_margin_m must be a non-negative finite distance in m')
        if len({tracked_frame, world_frame, odom_frame}) != 3:
            raise ValueError('distinct frames required')
        if not pose_source or not odom_source:
            raise ValueError('provenance labels required')
        self.map_id, self.prior_ids = map_id, frozenset(prior_ids)
        self.tracked_frame, self.world_frame, self.odom_frame = tracked_frame, world_frame, odom_frame
        # Declared provenance only: these labels never change gate decisions.
        self.pose_source, self.odom_source = pose_source, odom_source
        self.navigation_ready = navigation_ready is True
        self.instance = uuid.uuid4().hex
        self.epoch = self.sequence = self.streak = 0
        self.state, self.reason = 'initializing', 'awaiting_acquisition'
        self.pending, self.approved = OrderedDict(), OrderedDict()
        self.last_input = self.last_complete = 0
        self.last_good_wall = self.last_ros = self.last_wall = None
        self.count_mode = count_mode
        # output mode only: outputs held until their camera frame arrives (stamp -> entry),
        # the newest stamp rtabmap answered, and the ref_id of the last counted output.
        self.early = OrderedDict()
        self.newest_rtab = 0
        self.last_ref = None
        # Every pending_overflow revoke (pending queue or early hold), for the run readout.
        self.pending_overflows = 0
        # correction mode only. `pending` holds one item per output stamp (no camera frame);
        # last_complete is the newest approval stamp ever published, the status stamp floor.
        self.max_correction_ns = int(round(max_correction_age_s * 10**9))
        self.held = None                # (stamp, map->odom) in use while valid
        self.candidates = []            # [(stamp, map->odom)] towards entry
        self.jump_led = False           # candidate 1 of this lock is a correction a jump revoked on
        self.odom_pair = None           # newest healthy odom + odom_info pair
        self.last_out_stamp = 0         # newest output stamp decided (complete)
        self.adopted = self.jumps = self.synthesized = 0
        # odometry jump check (correction mode; see COUNT_MODES): V, E and the triggers seen
        self.odom_jump_check = odom_jump_check
        self.max_odom_speed_mps, self.odom_jump_margin_m = float(max_odom_speed_mps), float(odom_jump_margin_m)
        self.odom_jumps = 0
        # Each completed linked-or-not correction: the node drains it into its event row.
        self.corrections = collections.deque(maxlen=256)

    def reset_output_tracking(self):
        self.early.clear()
        self.newest_rtab = 0
        self.last_ref = None
        self.held, self.candidates, self.odom_pair = None, [], None
        self.jump_led = False

    def revoke(self, state, reason):
        if self.state == 'valid' or self.approved or self.streak:
            self.epoch += 1
        self.state, self.reason, self.streak = state, reason, 0
        self.approved.clear()
        self.held, self.candidates = None, []
        self.jump_led = False

    def discontinuity(self, cycle):
        if type(cycle) is not int or cycle < 1:
            raise ValueError('positive replay cycle required')
        self.revoke('initializing', 'replay_discontinuity:' + str(cycle))
        self.pending.clear()
        self.reset_output_tracking()
        self.last_complete = max(self.last_complete, self.last_input)
        self.last_good_wall = None

    def unanswered(self, item):
        """output mode: rtabmap has not (yet) answered this camera frame."""
        return not any(kind in item['messages'] for kind in RTABMAP_KINDS)

    def tick(self, wall, ros_ns):
        if not math.isfinite(wall) or type(ros_ns) is not int or ros_ns < 0:
            raise ValueError('invalid clock')
        if self.count_mode == 'correction':
            return self.correction_tick(wall, ros_ns)
        if ((self.last_ros is not None and ros_ns < self.last_ros)
                or (self.last_wall is not None and wall < self.last_wall)):
            self.revoke('initializing', 'clock_rollback')
            self.instance = uuid.uuid4().hex
            self.pending.clear()
            self.reset_output_tracking()
            self.last_input = self.last_complete = 0
            self.last_good_wall = None
        self.last_ros, self.last_wall = ros_ns, wall
        for stamp, item in list(self.pending.items()):
            if wall-item['wall'] >= .5 or ros_ns-stamp > 10**9:
                self.pending.pop(stamp)
                self.last_complete = max(self.last_complete, stamp)
                if self.count_mode == 'output' and self.unanswered(item) and not item['bad']:
                    continue  # a frame rtabmap skipped: not an output, so not a failure either
                self.revoke('stale', 'incomplete_acquisition_timeout')
        # output mode: a held output whose camera frame did not arrive within the same limits as
        # a pending frame, or whose frame can no longer arrive (a newer frame came), fails closed.
        for held_stamp, entry in list(self.early.items()):
            if (wall - entry['wall'] >= .5 or ros_ns - held_stamp > 10**9
                    or held_stamp <= self.last_input):
                self.early.pop(held_stamp)
                self.revoke('lost', 'unexpected_or_retired_stamp')
        if self.state in ('valid', 'confirming') and (wall-self.last_good_wall >= 1. or ros_ns-self.last_complete > 10**9):
            self.revoke('stale', 'localization_stream_timeout')
        for stamp in list(self.approved):
            if ros_ns-stamp > 10**9:
                self.approved.pop(stamp)

    def acquisition(self, stamp, wall, ros_ns):
        """Open a camera frame. Returns the approvals it completes (only held outputs can)."""
        self.tick(wall, ros_ns)
        if self.count_mode == 'correction':
            return []  # no camera frame is read in this mode (the node does not subscribe one)
        if type(stamp) is not int or stamp <= self.last_input or stamp <= 0 or stamp > ros_ns or ros_ns-stamp > 10**9:
            self.revoke('lost', 'acquisition_stamp_invalid_or_duplicate')
            return []
        self.last_input = stamp
        if len(self.pending) >= 32:
            self.pending.clear()
            self.pending_overflows += 1
            self.revoke('lost', 'pending_overflow')
        item = self.pending[stamp] = {'wall': wall, 'messages': {}, 'bad': False}
        held = self.early.pop(stamp, None)
        outputs = []
        if held is not None:
            # Outputs that came first now meet their frame and take the ordinary path.
            if held['bad']:
                item['bad'] = True  # e.g. a held duplicate: the frame stays failed
            for kind, (row, reason) in held['messages'].items():
                outputs += self.attach(item, stamp, kind, row, reason, wall)
        return outputs

    def message_reason(self, kind, row):
        expected = self.odom_frame if kind in ('odom', 'odom_info') else self.world_frame
        if row['header'].get('frame_id') != expected:
            return 'wrong_frame'
        if kind == 'odom' and row.get('child_frame_id') != self.tracked_frame:
            return 'wrong_child_frame'
        if kind in ('odom', 'localization_pose'):
            if not pose_valid(row.get('pose', {}).get('pose', {})):
                return 'invalid_pose'
            if not covariance_valid(row.get('pose', {}).get('covariance', [])):
                return 'invalid_covariance'
        elif kind == 'odom_info':
            if row.get('lost') is not False:
                return 'odometry_lost'
            if not covariance_valid(row.get('covariance', [])):
                return 'invalid_covariance'
        elif kind == 'info':
            links = (row.get('loop_closure_id'), row.get('proximity_detection_id'))
            if not any(type(link) is int and link in self.prior_ids for link in links):
                return 'no_prior_map_match'
            # output mode counts consecutive rtabmap outputs by this number; without it no
            # output can be shown to follow the previous one.
            if self.count_mode == 'output' and (type(row.get('ref_id')) is not int or row['ref_id'] <= 0):
                return 'missing_ref_id'
        return None

    def ingest(self, kind, row, wall, ros_ns):
        if self.count_mode == 'correction':
            return self.correction_ingest(kind, row, wall, ros_ns)
        self.tick(wall, ros_ns)
        if kind not in KINDS:
            raise ValueError('unknown stream')
        try:
            stamp = stamp_ns(row)
            reason = self.message_reason(kind, row)
        except (KeyError, TypeError, ValueError):
            self.revoke('lost', 'malformed_message')
            return []
        item = self.pending.get(stamp)
        if (item is None and self.count_mode == 'output' and stamp > self.last_input
                and stamp > self.last_complete):
            return self.hold(stamp, kind, row, reason, wall)
        if item is None or stamp <= self.last_complete:
            self.revoke('lost', 'unexpected_or_retired_stamp')
            return []
        return self.attach(item, stamp, kind, row, reason, wall)

    def hold(self, stamp, kind, row, reason, wall):
        """output mode: an output whose camera frame has not arrived yet waits for it."""
        entry = self.early.get(stamp)
        if entry is None:
            if len(self.early) >= 32:
                self.early.clear()
                self.pending_overflows += 1
                self.revoke('lost', 'pending_overflow')
                return []
            entry = self.early[stamp] = {'wall': wall, 'messages': OrderedDict(), 'bad': False}
        if kind in entry['messages']:
            entry['bad'] = True
            self.revoke('lost', 'duplicate_message')
            return []
        entry['messages'][kind] = (copy.deepcopy(row), reason)
        if reason:
            # Known bad already: revoke now rather than when the frame arrives, and (as a
            # newer bad acquisition) keep older completions from counting meanwhile.
            entry['bad'] = True
            self.revoke('lost', kind + ':' + reason)
        return []

    def attach(self, item, stamp, kind, row, reason, wall):
        if kind in item['messages']:
            item['bad'] = True
            self.revoke('lost', 'duplicate_message')
            return []
        item['messages'][kind] = copy.deepcopy(row)
        if self.count_mode == 'output' and kind in RTABMAP_KINDS:
            self.newest_rtab = max(self.newest_rtab, stamp)
        if reason:
            item['bad'] = True
            self.revoke('lost', kind + ':' + reason)
        outputs = []
        # Never certify a newer completed acquisition past an unresolved older one.
        while self.pending:
            current, first = next(iter(self.pending.items()))
            if len(first['messages']) != len(KINDS):
                # output mode: rtabmap publishes in stamp order, so once it answered a newer
                # frame it skipped this one. Skipping is silent, but never for a bad frame.
                if not (self.count_mode == 'output' and self.unanswered(first)
                        and self.newest_rtab > current):
                    break
                self.pending.pop(current)
                self.last_complete = current
                if first['bad']:
                    self.revoke('lost', 'unhealthy_acquisition')
                continue
            self.pending.pop(current)
            self.last_complete = current
            if first['bad']:
                self.revoke('lost', 'unhealthy_acquisition')
                continue
            # A newer known bad acquisition must also prevent old completions rearming.
            if (any(entry['bad'] for entry in self.pending.values())
                    or any(entry['bad'] for entry in self.early.values())):
                self.revoke('lost', 'newer_acquisition_unhealthy')
                continue
            if self.count_mode == 'output':
                # Consecutive means consecutive rtabmap outputs: a missing one (lost in
                # transport, never seen here) breaks the count instead of being bridged.
                ref = first['messages']['info']['ref_id']
                if self.streak and ref != self.last_ref + 1:
                    self.revoke('lost', 'rtabmap_output_gap')
                self.last_ref = ref
            self.streak += 1
            self.state = 'valid' if self.streak >= 3 else 'confirming'
            self.reason = 'three_healthy_acquisitions' if self.state == 'valid' else 'confirming_healthy_acquisitions'
            self.last_good_wall = wall
            if self.state == 'valid':
                localization = first['messages']['localization_pose']['pose']
                odom_pose = first['messages']['odom']['pose']['pose']
                # The map->odom correction this approval implies, composed here from the two
                # poses of the SAME acquisition. It used to be recomputed and dropped by the
                # node (host/ros_localization_gate.py:212-213), so no consumer could observe
                # it or its order. Both poses already passed pose_valid on this branch, so
                # the composition is total. Contract only: the gate compares nothing.
                map_odom = pose_matrix(localization['pose']) @ np.linalg.inv(pose_matrix(odom_pose))
                approved = {'stamp_ns': current,
                            'pose': localization,
                            'odom_pose': odom_pose,
                            'map_odom': map_odom.tolist()}
                self.approved[current] = approved
                while len(self.approved) > 32:
                    self.approved.popitem(last=False)
                outputs.append(copy.deepcopy(approved))
        return outputs

    # ------------------------------------------------------------------ correction mode
    def correction_tick(self, wall, ros_ns):
        if ((self.last_ros is not None and ros_ns < self.last_ros)
                or (self.last_wall is not None and wall < self.last_wall)):
            self.revoke('initializing', 'clock_rollback')
            self.instance = uuid.uuid4().hex
            self.pending.clear()
            self.reset_output_tracking()
            self.last_out_stamp = self.last_input = self.last_complete = 0
        self.last_ros, self.last_wall = ros_ns, wall
        for stamp, item in list(self.pending.items()):
            if item['rtab_wall'] is None:
                # odometry of a frame rtabmap did not process (it answered a newer one, or the
                # frame is too old to ever be used): not an output, so not a failure either
                if self.newest_rtab > stamp or ros_ns - stamp > self.max_correction_ns + 5 * 10**8:
                    self.pending.pop(stamp)
            elif wall - item['rtab_wall'] >= .5:
                self.pending.pop(stamp)
                self.revoke('lost', 'incomplete_output')
        pair = self.odom_pair
        if self.state == 'valid':
            if ros_ns - self.held[0] > self.max_correction_ns:
                self.revoke('stale', 'correction_too_old')
            elif pair is None or ros_ns - pair['stamp'] > 10**9 or wall - pair['wall'] > 1.:
                # checked before the approvals are aged out below, so a valid status always
                # still carries the approval at its stamp
                self.revoke('stale', 'odometry_stale')
        elif (self.state == 'confirming' and self.candidates
              and ros_ns - self.candidates[-1][0] > self.max_correction_ns):
            self.soft_reset('entry_stale')
        for stamp in list(self.approved):
            if stamp < ros_ns - 10**9:
                del self.approved[stamp]

    def soft_reset(self, why):
        """Restart the entry count without a new epoch (nothing was approved from it)."""
        self.candidates, self.streak = [], 0
        self.jump_led = False
        if self.state == 'confirming':
            self.state, self.reason = 'initializing', 'entry_reset:' + why

    def correction_ingest(self, kind, row, wall, ros_ns):
        self.tick(wall, ros_ns)
        if kind not in KINDS:
            raise ValueError('unknown stream')
        try:
            stamp = stamp_ns(row)
            if kind == 'info' and (type(row.get('ref_id')) is not int or row['ref_id'] <= 0):
                # consecutive outputs are counted by this number (message_reason checks the
                # prior-map link first, which is neutral here, so this goes first)
                reason = 'missing_ref_id'
            else:
                reason = self.message_reason(kind, row)
        except (KeyError, TypeError, ValueError, AttributeError):
            self.revoke('lost', 'malformed_message')
            return []
        linked = reason != 'no_prior_map_match'
        if not linked:
            reason = None  # an output away from the prior map brings no correction; not a failure
        if stamp > ros_ns:
            self.revoke('lost', 'future_stamp')
            return []
        rtab = kind in RTABMAP_KINDS
        if rtab and stamp <= self.last_out_stamp:
            return []  # older than an output already decided: can never complete in order
        if reason:
            self.revoke('lost', kind + ':' + reason)
        if rtab and ros_ns - stamp > self.max_correction_ns:
            # already older than the correction age limit on arrival: never usable, not a failure
            self.newest_rtab = max(self.newest_rtab, stamp)
            if kind == 'info' and not reason:
                self.last_ref = row['ref_id']
            self.pending.pop(stamp, None)
            return []
        item = self.pending.get(stamp)
        if item is None:
            if len(self.pending) >= 64:
                self.pending.clear()
                self.pending_overflows += 1
                self.revoke('lost', 'pending_overflow')
                return []
            item = self.pending[stamp] = {'wall': wall, 'messages': {}, 'bad': False,
                                          'rtab_wall': None, 'linked': True}
        if kind in item['messages']:
            item['bad'] = True
            self.revoke('lost', 'duplicate_message')
            return []
        item['messages'][kind] = copy.deepcopy(row)
        item['bad'] = item['bad'] or bool(reason)
        item['linked'] = item['linked'] and linked
        if rtab:
            self.newest_rtab = max(self.newest_rtab, stamp)
            if item['rtab_wall'] is None:
                item['rtab_wall'] = wall
        new_pair = (kind in ('odom', 'odom_info') and not item['bad']
                    and 'odom' in item['messages'] and 'odom_info' in item['messages']
                    and (self.odom_pair is None or stamp > self.odom_pair['stamp']))
        if new_pair:
            odom = item['messages']['odom']['pose']
            if self.odom_jump_check and self.odom_pair is not None:
                self.check_odom_step(self.odom_pair, stamp, odom['pose'])
            self.odom_pair = {'stamp': stamp, 'pose': odom['pose'], 'wall': wall,
                              'covariance': [float(v) for v in odom['covariance']]}
        outputs = []
        if len(item['messages']) == len(KINDS):
            self.pending.pop(stamp)
            self.last_out_stamp = max(self.last_out_stamp, stamp)
            if item['bad']:
                ref = item['messages']['info'].get('ref_id')
                self.last_ref = ref if type(ref) is int and ref > 0 else self.last_ref
            elif ros_ns - stamp > self.max_correction_ns:
                self.last_ref = item['messages']['info']['ref_id']
            else:
                outputs += self.accept_correction(stamp, item)
        if new_pair and self.state == 'valid':
            outputs += self.synthesize(entry=False)
        return outputs

    def odom_jumped(self, distance_m, dt_s):
        """The odometry jump criterion: a planar step longer than the platform can drive in dt, plus E."""
        return distance_m > self.max_odom_speed_mps * dt_s + self.odom_jump_margin_m

    def check_odom_step(self, previous, stamp, pose):
        """A new healthy odometry pair (stamp, pose) against the previous one the gate kept."""
        a, b = previous['pose']['position'], pose['position']
        distance = math.hypot(b['x'] - a['x'], b['y'] - a['y'])
        if not self.odom_jumped(distance, (stamp - previous['stamp']) / 1e9):
            return
        self.odom_jumps += 1
        if self.state == 'valid':
            self.revoke('lost', 'odometry_jump')   # before this pair yields an approval
        elif self.candidates:
            self.soft_reset('odometry_jump')        # candidates were corrections of the old odometry

    def log_correction(self, stamp, ref, verdict, delta=None):
        translation, yaw = delta if delta is not None else (None, None)
        self.corrections.append({'stamp_ns': stamp, 'ref_id': ref, 'verdict': verdict,
                                 'translation_m': None if translation is None else round(translation, 4),
                                 'yaw_deg': None if yaw is None else round(math.degrees(yaw), 3)})

    def drain_corrections(self):
        rows = list(self.corrections)
        self.corrections.clear()
        return rows

    def accept_correction(self, stamp, item):
        from host.nav_localization import correction_delta, correction_exceeds
        messages = item['messages']
        ref = messages['info']['ref_id']
        gap = self.last_ref is not None and ref != self.last_ref + 1
        self.last_ref = ref
        odom_pose = messages['odom']['pose']['pose']
        correction = pose_matrix(messages['localization_pose']['pose']['pose']) @ np.linalg.inv(pose_matrix(odom_pose))
        point = tuple(odom_pose['position'][k] for k in ('x', 'y', 'z'))
        jumped = False
        if self.state == 'valid':
            if not item['linked']:
                self.log_correction(stamp, ref, 'unlinked')
                return []  # no new correction: the held one keeps ageing towards X
            delta = correction_delta(self.held[1], correction, point)
            if not correction_exceeds(*delta):
                # a lost rtabmap output in between (ref gap) is one missed refresh, not a failure
                self.held = (stamp, correction)
                self.adopted += 1
                self.reason = 'correction_refreshed'
                self.log_correction(stamp, ref, 'adopted_after_gap' if gap else 'adopted', delta)
                return []
            self.jumps += 1
            self.log_correction(stamp, ref, 'jump', delta)
            self.revoke('lost', 'correction_jump')
            jumped = True  # and it becomes candidate 1 of a new lock below
        if not item['linked']:
            # entry (initializing/confirming): an output away from the prior map is neither counted
            # nor a reason to drop the candidates (第 7 棒); they still expire after X (entry_stale),
            # on a ref gap (ref advanced above, so this output makes none) or an inconsistent one.
            # A lock started by a jump-rejected correction is the exception (第 8 棒): it may not be
            # completed across unlinked outputs, so the old rule applies there and the count restarts.
            self.log_correction(stamp, ref, 'unlinked')
            if self.jump_led and self.candidates:
                self.soft_reset('unlinked')
            return []
        if gap and self.candidates:
            self.soft_reset('ref_gap')
        delta = None
        if self.candidates:
            delta = correction_delta(self.candidates[-1][1], correction, point)
            if correction_exceeds(*delta):
                self.soft_reset('inconsistent')
        self.candidates.append((stamp, correction))
        if jumped:  # after revoke/soft_reset cleared it: this lock starts with the rejected one
            self.jump_led = True
        self.streak = len(self.candidates)
        if self.streak < 3:
            self.state = 'confirming'
            self.reason = 'confirming_after_correction_jump' if jumped else 'confirming_corrections'
            if not jumped:  # a jump was logged above, with its delta against the held one
                self.log_correction(stamp, ref, 'candidate', delta)
            return []
        self.state, self.reason = 'valid', 'three_consistent_corrections'
        self.held, self.candidates = (stamp, correction), []
        self.jump_led = False
        self.adopted += 1
        self.log_correction(stamp, ref, 'entered', delta)
        # Publish at once: a valid status must carry an approval at its stamp, or the
        # consumer's lease voids the row (LocalizationLease.update).
        return self.synthesize(entry=True)

    def synthesize(self, entry):
        """One approval at the newest healthy odometry stamp: held map->odom x odom(t).

        Approval stamps only advance; on entry the stamp published last may be repeated
        (the previous epoch's approvals were all revoked) so the first valid status has one.
        """
        pair = self.odom_pair
        if self.held is None or pair is None:
            return []
        if pair['stamp'] < self.last_complete or (pair['stamp'] == self.last_complete and not entry):
            return []
        held = self.held[1]
        approved = {'stamp_ns': pair['stamp'],
                    'pose': {'pose': matrix_pose(held @ pose_matrix(pair['pose'])),
                             'covariance': list(pair['covariance'])},
                    'odom_pose': copy.deepcopy(pair['pose']),
                    'map_odom': held.tolist()}
        self.approved[pair['stamp']] = approved
        if len(self.approved) > 32:
            self.approved.popitem(last=False)
        self.last_complete = pair['stamp']
        self.synthesized += 1
        return [copy.deepcopy(approved)]

    def status(self, wall, ros_ns):
        self.tick(wall, ros_ns)
        self.sequence += 1
        approved = copy.deepcopy(list(self.approved.values())) if self.state == 'valid' else []
        # Which acquisition the newest map->odom in this row was observed at, 0 when the row
        # carries no approval. The per-item stamps already order the corrections among
        # themselves; this names the front of that order without the consumer scanning for it.
        # Appended, not a new schema: schema stays 1 and no existing key changes.
        return {'schema': 1, 'instance': self.instance, 'map_id': self.map_id,
                'epoch': self.epoch, 'sequence': self.sequence, 'state': self.state,
                'reason': self.reason, 'stamp_ns': self.last_complete, 'clock_ns': ros_ns,
                'world_frame': self.world_frame, 'tracked_frame': self.tracked_frame,
                'pose_source': self.pose_source, 'navigation_ready': self.navigation_ready,
                'odom_source': self.odom_source,
                'correction_stamp_ns': max((item['stamp_ns'] for item in approved), default=0),
                'approved': approved}


WORLD_ODOM_EDGE, ODOM_TRACKED_EDGE = 'world_odom', 'odom_tracked'


def approval_edges(publish_tracked, hold_enabled):
    """Which TF edges one approval publishes. Held map->odom leaves the approval message."""
    edges = [] if hold_enabled else [WORLD_ODOM_EDGE]
    if publish_tracked:
        edges.append(ODOM_TRACKED_EDGE)
    return edges


class MapOdomHold:
    """Republish decision for the held world->odom edge. Pure: the clock is injected.

    The stamp is a future ROS time, not the acquisition stamp, so consumers can look the
    edge up at their own now without extrapolating a retired sample. Identical stamps are
    dropped, so a stepped /clock yields one transform per step and no TF_REPEATED_DATA.
    """

    def __init__(self, future_ns):
        if type(future_ns) is not int or future_ns < 0:
            raise ValueError('non-negative integer future offset required')
        self.future_ns = future_ns
        self.matrix = self.last_stamp = None

    def update(self, matrix):
        held = np.array(matrix, dtype=float)
        if held.shape != (4, 4) or not np.isfinite(held).all():
            raise ValueError('finite 4x4 transform required')
        self.matrix = held

    def tick(self, state, ros_now_ns):
        if type(ros_now_ns) is not int or ros_now_ns < 0:
            raise ValueError('invalid clock')
        if state != 'valid':
            # Leaving valid retires the held edge; only a new approval rearms it.
            self.matrix = None
            return None
        if self.matrix is None:
            return None
        stamp = ros_now_ns + self.future_ns
        if stamp == self.last_stamp:
            return None
        self.last_stamp = stamp
        return stamp, self.matrix.copy()


class LocalizationLease:
    """Consumer checks heartbeat AND distinct acquisition progress using local steady time."""
    def __init__(self, map_id, tracked_frame='camera_link'):
        if not map_id:
            raise ValueError('expected map SHA256 required')
        self.map_id, self.tracked_frame = map_id, tracked_frame
        self.row = self.last_status = None
        self.received = self.progress = -math.inf
        self.last_ros = None
        self.retired = set()
        self.generation = 0

    def invalidate(self):
        self.row = None
        self.generation += 1

    def update(self, row, wall, ros_ns):
        # Observe an elapsed local lease before a recovery message refreshes its clocks.
        self.check(wall, ros_ns)
        previous = self.last_status
        try:
            if (row['schema'] != 1 or row['map_id'] != self.map_id or row['world_frame'] != 'map'
                    or row['tracked_frame'] != self.tracked_frame or row['instance'] in self.retired
                    or not isinstance(row['instance'], str) or not row['instance']
                    or any(type(row[k]) is not int or row[k] < 0 for k in ('epoch', 'sequence', 'stamp_ns', 'clock_ns'))
                    or not 0 <= ros_ns-row['clock_ns'] <= 10**9
                    or row['state'] not in ('initializing', 'confirming', 'valid', 'lost', 'stale')):
                raise ValueError('invalid status identity or clock')
            if previous and row['instance'] == previous['instance']:
                if (row['sequence'] <= previous['sequence'] or row['epoch'] < previous['epoch']
                        or row['stamp_ns'] < previous['stamp_ns'] or row['clock_ns'] < previous['clock_ns']):
                    raise ValueError('status reordered')
            if previous and row['instance'] != previous['instance']:
                self.retired.add(previous['instance'])
            if (not previous or (row['instance'], row['epoch']) != (previous['instance'], previous['epoch'])
                    or row['state'] != 'valid'):
                self.generation += 1
            if not previous or row['stamp_ns'] != previous['stamp_ns'] or row['instance'] != previous['instance']:
                self.progress = wall
            if len(row['approved']) > 32:
                raise ValueError('too many approvals')
            stamps = set()
            for item in row['approved']:
                stamp = item['stamp_ns']
                if (type(stamp) is not int or not 0 < stamp <= row['stamp_ns'] or stamp in stamps
                        or not pose_valid(item['pose']['pose']) or not covariance_valid(item['pose']['covariance'])):
                    raise ValueError('invalid approved pose')
                # Appended 2026-09-21. Absent on a status recorded before that, which
                # tools/rtabmap_nav2_rehearsal_run.py and tools/rtabmap_frame_disposition_probe.py
                # still replay through this lease, so absence is tolerated; present and not a
                # rigid transform is a corrupt correction and fails closed like a bad pose.
                if 'map_odom' in item and not rigid_valid(item['map_odom']):
                    raise ValueError('invalid map->odom correction')
                stamps.add(stamp)
            if row['state'] == 'valid' and row['stamp_ns'] not in stamps:
                raise ValueError('valid status without current approval')
            if 'correction_stamp_ns' in row:
                # 0 means the row states no anchor, which is what a status row assembled
                # without this field looks like (tests/test_runtime_diagnostics.py builds
                # one). A STATED anchor must be the newest approval the row carries.
                anchor = row['correction_stamp_ns']
                if type(anchor) is not int or (anchor and anchor != max(stamps, default=0)):
                    raise ValueError('correction stamp does not name the newest approval')
            self.row = copy.deepcopy(row)
            self.last_status = self.row
            self.received = wall
            self.check(wall, ros_ns)
        except (KeyError, ValueError, TypeError):
            self.invalidate()

    def check(self, wall, ros_ns):
        if self.last_ros is not None and ros_ns < self.last_ros:
            if self.row:
                self.retired.add(self.row['instance'])
            self.invalidate()
        self.last_ros = ros_ns
        row = self.row
        healthy = bool(row and row['state'] == 'valid' and 0 <= wall-self.received < .5
                       and 0 <= wall-self.progress < 1. and 0 <= ros_ns-row['stamp_ns'] <= 10**9)
        if row and row['state'] == 'valid' and not healthy:
            self.invalidate()
        return healthy

    def token(self, stamp, wall, ros_ns):
        if not self.check(wall, ros_ns):
            return None
        if any(item['stamp_ns'] == stamp for item in self.row['approved']):
            return (self.generation, self.row['instance'], self.row['epoch'], stamp)
        return None

    def permits(self, token, wall, ros_ns):
        return token is not None and self.token(token[3], wall, ros_ns) == token

    def superseded(self, stamp, wall, ros_ns):
        """True when the healthy lease has moved past `stamp` without approving it.

        The gate publishes approvals in stamp order only (all count modes), so such an
        acquisition can never be approved later; waiting for it only runs into a timeout.
        Added 2026-10-15 for the runner's frame skip; no other decision of this class reads it.
        """
        if not self.check(wall, ros_ns):
            return False
        return (self.row['stamp_ns'] >= stamp
                and all(item['stamp_ns'] != stamp for item in self.row['approved']))

    def matrix(self, stamp, wall, ros_ns):
        if self.token(stamp, wall, ros_ns) is None:
            raise ValueError('acquisition not approved')
        return pose_matrix(next(item['pose']['pose'] for item in self.row['approved'] if item['stamp_ns'] == stamp))

    def correction(self, stamp, wall, ros_ns):
        """The observed map->odom of one approved acquisition, gated like `matrix`.

        Reading it says nothing about whether the map moved; comparing two of them is the
        consumer's decision and its threshold, which this class does not own.
        """
        if self.token(stamp, wall, ros_ns) is None:
            raise ValueError('acquisition not approved')
        item = next(item for item in self.row['approved'] if item['stamp_ns'] == stamp)
        if 'map_odom' not in item:
            raise ValueError('status predates the map->odom contract')
        return np.asarray(item['map_odom'], dtype=float)
