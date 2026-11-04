"""Localization authorization and action lifecycle for the Nav2 test consumer."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass

# D-035 item 1 (provisional, to be re-measured on the real car before 10-18). A same-map
# correction whose planar translation OR yaw reaches either limit revokes the goal.
# Both limits are inclusive: a correction exactly at the limit exceeds it.
TRANSLATION_LIMIT_M = 0.10
YAW_LIMIT_RAD = math.radians(5.0)
MAP_CORRECTION_EXCEEDED = 'map_correction_exceeded'
MAP_CORRECTION_UNREADABLE = 'map_correction_unreadable'
MAP_CORRECTION_REFERENCE_MISSING = 'map_correction_reference_missing'


def valid_sha256(value):
    return (isinstance(value, str) and len(value) == 64
            and all(char in '0123456789abcdef' for char in value))


def planar_yaw(matrix):
    """Yaw of the rotation block of a 4x4 transform (Z-Y-X convention)."""
    return math.atan2(float(matrix[1][0]), float(matrix[0][0]))


def shortest_yaw_difference(reference_yaw, current_yaw):
    """Absolute yaw change folded onto [0, pi]: 179 deg vs -179 deg is 2 deg."""
    return abs(math.remainder(current_yaw - reference_yaw, 2 * math.pi))


def transform_point(matrix, point):
    return tuple(sum(float(matrix[row][col]) * point[col] for col in range(3)) + float(matrix[row][3])
                 for row in range(3))


def correction_delta(reference_map_odom, current_map_odom, odom_point):
    """How far one correction moved the tracked point on the map, and its yaw change.

    The same odom-frame point (the tracked frame's position at the current approval) is
    carried to the map by both corrections; the planar distance between the two results is
    the jump the robot sees. Comparing the translation columns instead would charge a small
    rotation about a distant odom origin as a large translation.
    """
    before = transform_point(reference_map_odom, odom_point)
    after = transform_point(current_map_odom, odom_point)
    translation = math.hypot(after[0] - before[0], after[1] - before[1])
    return translation, shortest_yaw_difference(planar_yaw(reference_map_odom),
                                                planar_yaw(current_map_odom))


def correction_exceeds(translation_m, yaw_rad):
    """D-035 item 1: at or above either limit, or not a finite measurement."""
    if not (math.isfinite(translation_m) and math.isfinite(yaw_rad)):
        return True
    return translation_m >= TRANSLATION_LIMIT_M or yaw_rad >= YAW_LIMIT_RAD


@dataclass(frozen=True)
class CorrectionReference:
    """The newest approval when a goal was sent (D-035 item 2)."""
    stamp_ns: int
    map_odom: tuple


def newest_approval(row):
    approved = row['approved']
    return max(approved, key=lambda item: item['stamp_ns']) if approved else None


def lease_correction_reference(lease, wall, ros_ns):
    """The newest approval's map->odom in a LocalizationLease now, or None (D-035 item 2).

    Shared by the point-to-point consumer and the M5 Runtime (host/ros_runtime.py).
    """
    if lease is None or lease.row is None:
        return None
    newest = newest_approval(lease.row)
    if newest is None:
        return None
    try:
        matrix = lease.correction(newest['stamp_ns'], wall, ros_ns)
    except ValueError:
        return None
    return CorrectionReference(newest['stamp_ns'],
                               tuple(tuple(float(v) for v in line) for line in matrix))


def lease_correction_verdict(lease, reference, wall, ros_ns):
    """None while a goal's correction reference still holds, else a revocation reason.

    Call only after the goal's authorization identity has been checked unchanged. Every
    approval in the current row newer than the reference is compared, so a jump already
    replaced by a newer approval inside the row's window is still seen (D-035 item 1).
    Shared by the point-to-point consumer and the M5 Runtime.
    """
    if reference is None:
        return MAP_CORRECTION_REFERENCE_MISSING
    row = lease.row
    if row is None:
        return MAP_CORRECTION_UNREADABLE
    try:
        for item in row['approved']:
            if item['stamp_ns'] <= reference.stamp_ns:
                continue
            current = lease.correction(item['stamp_ns'], wall, ros_ns)
            odom = item['odom_pose']['position']
            point = (float(odom['x']), float(odom['y']), float(odom['z']))
            if not all(math.isfinite(v) for v in point):
                return MAP_CORRECTION_UNREADABLE
            if correction_exceeds(*correction_delta(reference.map_odom, current, point)):
                return MAP_CORRECTION_EXCEEDED
    except (KeyError, TypeError, ValueError, IndexError):
        return MAP_CORRECTION_UNREADABLE
    return None


class Authorization(tuple):
    """The unchanged four-part identity, carrying the goal's correction reference aside.

    Equality and hashing are those of the plain tuple (generation, instance, epoch,
    map_id); `reference` never takes part, so the authorization identity is not changed.
    """

    def __new__(cls, identity, reference):
        value = super().__new__(cls, identity)
        value.reference = reference
        return value


class NavigationLocalization:
    """Separate reference compatibility from strict estimated authorization.

    Estimated goals are bound to the lease identity (generation, instance, epoch,
    map_id) and, within that identity, to the map->odom correction of the newest
    approval at send time. A later approval whose correction moves the tracked point
    by the D-035 limits revokes the goal (`correction_verdict`).
    """

    REFERENCE_IDENTITY = ('reference',)

    def __init__(self, mode, map_id=None, navigation_map_id=None,
                 tracked_frame='camera_link'):
        if mode not in ('reference', 'estimated'):
            raise ValueError('pose mode must be reference or estimated')
        if mode == 'estimated':
            if not valid_sha256(map_id) or not valid_sha256(navigation_map_id):
                raise ValueError('estimated mode requires localization and Nav2 expected database IDs')
            if map_id != navigation_map_id:
                raise ValueError('localization and Nav2 expected database IDs differ')
        self.mode = mode
        self.map_id = map_id
        self.navigation_map_id = navigation_map_id
        if mode == 'estimated':
            # Keep the established reference executable free of the new NumPy dependency.
            from host.localization_gate import LocalizationLease
            self.lease = LocalizationLease(map_id, tracked_frame)
        else:
            self.lease = None
        self.status_contract_valid = False
        self.reason = None

    def update_json(self, payload, wall, ros_ns):
        if self.lease is None:
            return
        try:
            row = json.loads(payload)
            if (row.get('pose_source') != 'estimated'
                    or row.get('navigation_ready') is not True
                    or row.get('map_id') != self.navigation_map_id):
                raise ValueError('status provenance, readiness or map identity rejected')
            self.status_contract_valid = True
            self.lease.update(row, wall, ros_ns)
            if self.lease.row is None:
                self.status_contract_valid = False
                self.reason = 'localization_lease_invalid'
        except (ValueError, TypeError, AttributeError):
            self.status_contract_valid = False
            self.lease.invalidate()
            self.reason = 'localization_status_contract_invalid'

    def healthy(self, wall, ros_ns):
        if self.lease is None:
            self.reason = None
            return True
        if not self.status_contract_valid or not self.lease.check(wall, ros_ns):
            self.reason = 'localization_lease_invalid'
            return False
        self.reason = None
        return True

    def identity(self, wall, ros_ns):
        if not self.healthy(wall, ros_ns):
            return None
        if self.lease is None:
            return self.REFERENCE_IDENTITY
        row = self.lease.row
        return Authorization((self.lease.generation, row['instance'], row['epoch'], row['map_id']),
                             self.correction_reference(wall, ros_ns))

    def permits(self, identity, wall, ros_ns):
        return identity is not None and self.identity(wall, ros_ns) == identity

    def _newest_approval(self):
        return newest_approval(self.lease.row)

    def correction_reference(self, wall, ros_ns):
        """The newest approval's map->odom now, or None when the row carries none."""
        return lease_correction_reference(self.lease, wall, ros_ns)

    def correction_verdict(self, authorization, wall, ros_ns):
        """D-035 rule for this consumer's goal; reference mode has no correction rule."""
        if self.lease is None:
            return None
        return lease_correction_verdict(self.lease, getattr(authorization, 'reference', None),
                                        wall, ros_ns)


@dataclass
class GoalContext:
    request_id: int
    authorization: tuple
    phase: str = 'waiting_acceptance'
    handle: object = None
    result_future: object = None
    response_processed: bool = False
    transport_error: str = None
    revoked: bool = False
    cancel_sent: bool = False
    terminal_status: int = None


class GoalLifecycle:
    """Keep one action generation occupied until its terminal result arrives."""

    TERMINAL = frozenset((4, 5, 6))  # SUCCEEDED, CANCELED, ABORTED

    def __init__(self):
        self.active = None
        self.next_request_id = 1

    def start(self, authorization):
        if self.active is not None:
            raise RuntimeError('previous navigation has not terminated')
        if authorization is None:
            raise RuntimeError('navigation authorization unavailable')
        context = GoalContext(self.next_request_id, authorization)
        self.next_request_id += 1
        self.active = context
        return context

    def revoke(self, context):
        if context is not self.active:
            return False
        context.revoked = True
        return self._request_cancel(context)

    def accepted(self, context, handle):
        context.handle = handle
        context.response_processed = True
        if context is not self.active:
            return self._request_cancel(context)
        if not handle.accepted:
            context.phase = 'terminal'
            self.active = None
            return False
        context.phase = 'accepted'
        return self._request_cancel(context) if context.revoked else False

    def cancel_response(self, context):
        """A cancel acknowledgement does not free the action generation."""
        return context is self.active and context.phase != 'terminal'

    def result(self, context, status):
        if status not in self.TERMINAL:
            raise ValueError('nonterminal navigation result')
        context.phase = 'terminal'
        context.terminal_status = status
        if context is not self.active:
            return False
        self.active = None
        return not context.revoked and status == 4

    @staticmethod
    def _request_cancel(context):
        handle = context.handle
        if (context.cancel_sent or handle is None
                or not getattr(handle, 'accepted', False)):
            return None
        context.cancel_sent = True
        try:
            return handle.cancel_goal_async()
        except Exception as exc:
            # Unknown delivery is not a terminal result. Keep the slot revoked
            # while allowing the caller to publish false and finish cleanup.
            context.revoked = True
            context.transport_error = 'goal_cancel_transport_failed:' + str(exc)
            return None
