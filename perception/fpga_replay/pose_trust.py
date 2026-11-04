"""Pose trust from wheel-odometry yaw vs integrated gyro yaw (docs/design/MEMORY_POSE_TRUST_20261017.md).

When the wheels do not see a rotation the gyro sees (the car is bumped, turned by hand, or
skids), the wheel-odometry pose is wrong from then on. Over a short trailing window W the two
yaw changes are compared; a disagreement beyond ``trip_rad`` marks the pose untrusted, and it
is trusted again after the disagreement stayed within ``clear_rad`` for ``settle_s`` seconds.

Pure Python (3.8), no ROS. ``discrepancy`` and ``step`` are pure functions; ``PoseTrustMonitor``
only buffers samples and keeps the state transitions so a frame can ask about its own
acquisition time (causal: samples at or before that time).
"""
import bisect
import math
from collections import namedtuple

Params = namedtuple("Params", "window_s trip_rad clear_rad settle_s max_gap_s")
DEFAULT_PARAMS = Params(window_s=0.5, trip_rad=math.radians(10.0), clear_rad=math.radians(5.0),
                        settle_s=2.0, max_gap_s=0.25)

# trusted: bool; since: time of the last transition (None before any); calm_since: start of the
# current run of |D| <= clear while untrusted (None when not counting)
State = namedtuple("State", "trusted since calm_since")
INITIAL = State(True, None, None)


def _value_at(times, values, t):
    """Linear interpolation of a sample series at t; None when t is outside [times[0], times[-1]]."""
    if not times or t < times[0] or t > times[-1]:
        return None
    i = bisect.bisect_left(times, t)
    if times[i] == t:
        return values[i]
    a = (t - times[i-1]) / (times[i] - times[i-1])
    return values[i-1] + a * (values[i] - values[i-1])


def _covered(times, start, end, max_gap_s):
    """Samples span [start, end] with no gap above max_gap_s inside it."""
    if not times or times[0] > start or times[-1] < end:
        return False
    i = max(bisect.bisect_right(times, start) - 1, 0)
    j = bisect.bisect_left(times, end)
    for k in range(i + 1, min(j, len(times) - 1) + 1):
        if times[k] - times[k-1] > max_gap_s:
            return False
    return True


def discrepancy(gyro, wheel, t, window_s, max_gap_s):
    """Gyro yaw change minus wheel yaw change over [t - window_s, t], radians; None if not covered.

    ``gyro`` and ``wheel`` are (times, cumulative_yaw) pairs of equal-length ascending lists.
    """
    start = t - window_s
    for times, _ in (gyro, wheel):
        if not _covered(times, start, t, max_gap_s):
            return None
    dg = _value_at(gyro[0], gyro[1], t) - _value_at(gyro[0], gyro[1], start)
    dw = _value_at(wheel[0], wheel[1], t) - _value_at(wheel[0], wheel[1], start)
    return dg - dw


def step(state, t, d, params=DEFAULT_PARAMS):
    """Next state after evaluating the disagreement d (radians, or None = no data) at time t."""
    if d is None:                                   # cannot check: hold; an untrusted pose restarts its settling
        return state if state.trusted else state._replace(calm_since=None)
    if state.trusted:
        if abs(d) > params.trip_rad:
            return State(False, t, None)
        return state
    if abs(d) > params.clear_rad:
        return state._replace(calm_since=None)
    calm = t if state.calm_since is None else state.calm_since
    if t - calm >= params.settle_s:
        return State(True, t, None)
    return state._replace(calm_since=calm)


def unwrap(previous, angle):
    """Continue an unwrapped angle series with a wrapped angle sample."""
    if previous is None:
        return angle
    return previous + math.atan2(math.sin(angle - previous), math.cos(angle - previous))


class PoseTrustMonitor:
    """Buffers gyro / wheel yaw samples and evaluates ``step`` on every new common time.

    add_gyro_rate(t, rate) integrates rate * nominal_period_s per sample (the MCU status frame
    period): the serial reader stamps a batch of frames with one receive time, so stamp
    differences would integrate wrongly while the frame count does not.
    """

    def __init__(self, params=DEFAULT_PARAMS, nominal_period_s=0.01, keep_s=30.0, eval_period_s=0.01):
        self.params, self.period, self.keep_s, self.eval_period = params, nominal_period_s, keep_s, eval_period_s
        self.gyro = ([], [])
        self.wheel = ([], [])
        self._gyro_sum = 0.0
        self._wheel_yaw = None
        self.state = INITIAL
        self.evaluated_until = None
        self.transitions = []              # (t, trusted, |d| at the transition), trimmed to keep_s
        self._fresh = []                   # transitions not yet drained (for the runner's event log)
        self.unchecked = 0                 # evaluations without data coverage
        # v0.2 (2026-10-21): verdicts decided elsewhere (the odometry node's slip / crash detector,
        # host/gyro_odom.py, published only with --heading gyro) as (t, trusted) transitions. AND-ed into
        # trusted_at; none received = trusted_at exactly as before.
        self.external = []

    # -- input
    def _append(self, series, t, value):
        times, values = series
        if times and t < times[-1]:
            return False                   # out of order: dropped
        if times and t == times[-1]:
            values[-1] = value             # same stamp (serial batch): keep the latest value
        else:
            times.append(t); values.append(value)
        return True

    def add_gyro_rate(self, t, rate_rad_s):
        self._gyro_sum += rate_rad_s * self.period
        if self._append(self.gyro, t, self._gyro_sum):
            self._advance()

    def add_gyro_yaw(self, t, cumulative_yaw):
        if self._append(self.gyro, t, cumulative_yaw):
            self._advance()

    def add_wheel_yaw(self, t, yaw, wrapped=True):
        value = unwrap(self._wheel_yaw, yaw) if wrapped else yaw
        if self._append(self.wheel, t, value):
            self._wheel_yaw = value
            self._advance()

    # -- evaluation
    def _advance(self):
        if not self.gyro[0] or not self.wheel[0]:
            return
        now = min(self.gyro[0][-1], self.wheel[0][-1])
        t = now if self.evaluated_until is None else self.evaluated_until + self.eval_period
        while t <= now + 1e-9:
            self._evaluate(t)
            t += self.eval_period
        self._trim(now)

    def _evaluate(self, t):
        d = discrepancy(self.gyro, self.wheel, t, self.params.window_s, self.params.max_gap_s)
        if d is None:
            self.unchecked += 1
        new = step(self.state, t, d, self.params)
        if new.trusted != self.state.trusted:
            row = (t, new.trusted, None if d is None else abs(d))
            self.transitions.append(row); self._fresh.append(row)
        self.state, self.evaluated_until = new, t

    def _trim(self, now):
        horizon = now - max(self.keep_s, self.params.window_s + self.params.max_gap_s)
        for times, values in (self.gyro, self.wheel):
            k = bisect.bisect_left(times, horizon) - 1
            if k > 0:
                del times[:k]; del values[:k]
        k = bisect.bisect_left([x[0] for x in self.transitions], horizon) - 1
        if k > 0:
            del self.transitions[:k]

    def add_external(self, t, trusted):
        """One external verdict transition at time t (same clock as the samples); out of order is dropped."""
        if self.external and t < self.external[-1][0]:
            return False
        self.external.append((t, bool(trusted)))
        horizon = t - self.keep_s
        k = bisect.bisect_left([x[0] for x in self.external], horizon) - 1
        if k > 0:
            del self.external[:k]
        return True

    def external_trusted_at(self, t):
        if not self.external:
            return True
        k = bisect.bisect_right([x[0] for x in self.external], t)
        if k:
            return self.external[k-1][1]
        return not self.external[0][1]

    # -- query
    def drain_transitions(self):
        rows, self._fresh = self._fresh, []
        return rows

    def trusted_at(self, t):
        """State at time t: the last transition at or before t (evaluations use samples <= their time),
        AND the external verdict at t (always True while none was received)."""
        return self._own_trusted_at(t) and self.external_trusted_at(t)

    def _own_trusted_at(self, t):
        k = bisect.bisect_right([x[0] for x in self.transitions], t)
        if k:
            return self.transitions[k-1][1]
        if self.transitions:                # t before the first kept transition: the state before it
            return not self.transitions[0][1]
        return True                         # never left the initial (trusted) state
