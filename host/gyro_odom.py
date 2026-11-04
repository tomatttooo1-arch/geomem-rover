"""Gyro-heading, four-wheel odometry from the v0.2 chassis status frame 0x82 (2026-10-21).

Design: docs/design/CHASSIS_UART_V02_IMU_HANDOFF_20261021.md section 8 (heading from the gyro, translation
from the four mecanum wheels, wheel heading only as fallback); implementation notes in its section 10.
Input is one /fpga/chassis/wheel_counts body of a 0x82 frame (host/ros_chassis_driver.wheel_counts_row):
enc_a..enc_d, gz_sum, gz_n, imu_err, ax/ay/az, mcu_ms (u32), seq, cmd [v_mm_s, w_mrad_s, enable], t_ns.

  heading      rate = gz_sum / gz_n * 2000/32768 deg/s (gz_n = 0: no gyro this frame);
               dyaw = (rate - bias) * gyro_scale(sign) * dt, dt = mcu_ms difference (never gz_n * 5 ms)
  fallback     bias not estimated yet / gyro missing > gap_fallback_s / imu_err rose > imu_err_max within
               imu_err_window_s -> dyaw = four-wheel dyaw * wheel_yaw_scale; seconds counted per reason.
               A gyro gap up to gap_fallback_s integrates the last good rate (held).
  spikes       online with one frame of look-ahead and no output latency: a rate more than spike_dps from
               the median of the previous spike_history final rates is integrated provisionally as that
               median; when the next frame arrives the decision is made -- back within spike_return_dps of
               the median (or no gyro in the next frame) = spike, the median stays and spikes_replaced
               counts it; otherwise it was a real change and the yaw is corrected by the difference. The
               published yaw is therefore at most one frame of (spike - median) away from the final one,
               never delayed.
  bias         static only when, for >= static_s continuously: all four counts unchanged, the last sent
               command is stop or v = w = 0, std of the gyro rate < static_gyro_std_dps, std of |accel| <
               static_accel_std_g. NEVER wheels alone (10-17: a crash turned the car with its wheels still).
               Torn accel reads (seen on the board 10-21: 4.8 % of static 0x82 frames had (0, 0, 0) or one
               axis zeroed) would make every 1 s window "moving" and the bias would never be ready: inside
               the window, samples more than accel_outlier_g from the window median are left out of the
               std as long as they are at most accel_outlier_frac of it (more = really disturbed, not
               static). Frames with az < accel_min_z_g (impossible upright) are also kept out of the shock
               detector.
               The bias is the mean rate of the static stretch, kept current while it lasts; samples of the
               last bias_guard_s are only committed once the stretch is still static after them, so the onset
               of a motion that ends a stretch is not averaged in. First estimate needs initial_static_s of
               static (before that: fallback); every later stop re-estimates.
  translation  per-wheel distance = sign * delta counts * pi * wheel_diam / counts_per_rev; mecanum forward
               kinematics (X rollers: section 8.4 formulas; O: the mirrored mounting), redundancy
               (FL+FR)-(RL+RR); (forward, lateral) rotated by the mean of the yaw before and after the frame.
  slip/crash   wheel vs gyro rate apart > slip_rate_dps for slip_hold_s, |redundancy rate| > redundancy_mps,
               horizontal accel step > shock_g -> untrusted; trusted again after untrusted_settle_s clean.

All thresholds marked PLACEHOLDER are guesses until the T2 static recording and the section 8.7 calibration.
Pure Python (3.8), no ROS, no I/O; time is the MCU millisecond counter (unwrapped).
"""
import math
from collections import deque

from host.chassis_uart import ACCEL_RAW_G, GYRO_RAW_DEG_S
from host.wheel_odom import s32_delta

U32 = 1 << 32
POSITIONS = ('fl', 'fr', 'rl', 'rr')
ENCODERS = ('a', 'b', 'c', 'd')
STATIC_CRITERIA = ('wheels', 'command', 'gyro_std', 'accel_std')

DEFAULT_WHEEL_MAP = 'a=fl,b=fr,c=rl,d=rr'          # PLACEHOLDER (handoff section 6 question 2, no answer yet)
DEFAULT_WHEEL_SIGNS = '+1,+1,+1,+1'                 # PLACEHOLDER (per encoder a..d; F5 asks for forward = up)
DEFAULT_ROLLERS = 'X'                               # PLACEHOLDER (section 6 question 3)

# Defaults that are not measured yet (the status reports these names while they are in use).
PLACEHOLDER_DEFAULTS = {
    'wheel_map': DEFAULT_WHEEL_MAP, 'wheel_signs': DEFAULT_WHEEL_SIGNS, 'rollers': DEFAULT_ROLLERS,
    'gyro_scale_pos': 1.0, 'gyro_scale_neg': 1.0, 'wheel_yaw_scale': 1.0,
    'static_gyro_std_dps': 0.3, 'static_accel_std_g': 0.01,
    'slip_rate_dps': 15.0, 'redundancy_mps': 0.05, 'shock_g': 0.5,
}


def parse_wheel_map(text):
    """'a=fl,b=fr,c=rl,d=rr' -> {'fl': 'a', ...}; every encoder and every position exactly once."""
    out = {}
    for item in str(text).split(','):
        enc, _, pos = item.strip().partition('=')
        enc, pos = enc.strip().lower(), pos.strip().lower()
        if enc not in ENCODERS or pos not in POSITIONS or pos in out or enc in out.values():
            raise ValueError('bad wheel map %r (want e.g. %s)' % (text, DEFAULT_WHEEL_MAP))
        out[pos] = enc
    if len(out) != 4:
        raise ValueError('bad wheel map %r: needs all four encoders' % text)
    return out


def parse_signs(text):
    """'+1,+1,-1,+1' (encoders a..d) -> {'a': 1, ...}."""
    vals = [int(v) for v in str(text).split(',')]
    if len(vals) != 4 or any(v not in (1, -1) for v in vals):
        raise ValueError('bad wheel signs %r (four of +1/-1, encoders a..d)' % text)
    return dict(zip(ENCODERS, vals))


class FusedParams:
    """Every knob of FusedOdometry. Geometry: half_track = 0.205 / 2, half_wheelbase = 0.225 / 2 (dev2 10-16
    MCU config), so lx + ly = 0.215 m as in section 8.4; wheel 0.081 m, 1560 counts/rev (D-041)."""

    def __init__(self, wheel_diam_m=0.081, counts_per_rev=1560, track_m=0.205, wheelbase_m=0.225,
                 wheel_map=DEFAULT_WHEEL_MAP, wheel_signs=DEFAULT_WHEEL_SIGNS, rollers=DEFAULT_ROLLERS,
                 lateral=True, gyro_scale_pos=1.0, gyro_scale_neg=1.0, wheel_yaw_scale=1.0,
                 static_s=1.0, initial_static_s=2.0, static_gyro_std_dps=0.3, static_accel_std_g=0.01,
                 accel_outlier_g=0.05, accel_outlier_frac=0.2, accel_min_z_g=0.5,
                 bias_guard_s=0.2, bias_max_dps=None, gap_fallback_s=0.05, imu_err_window_s=1.0,
                 imu_err_max=5, spike_dps=30.0, spike_return_dps=15.0, spike_history=5,
                 slip_rate_dps=15.0, slip_hold_s=0.3, redundancy_mps=0.05, redundancy_window_s=0.2,
                 shock_g=0.5, shock_baseline_s=1.0, untrusted_settle_s=2.0, max_delta_counts=32767):
        if not (wheel_diam_m > 0 and counts_per_rev > 0 and track_m > 0 and wheelbase_m > 0):
            raise ValueError('geometry must be positive')
        if rollers not in ('X', 'O'):
            raise ValueError('rollers must be X or O')
        self.wheel_diam_m, self.counts_per_rev = float(wheel_diam_m), int(counts_per_rev)
        self.track_m, self.wheelbase_m = float(track_m), float(wheelbase_m)
        self.wheel_map_text, self.wheel_signs_text = str(wheel_map), str(wheel_signs)
        self.wheel_map, self.wheel_signs = parse_wheel_map(wheel_map), parse_signs(wheel_signs)
        self.rollers = rollers
        lx, ly = self.wheelbase_m / 2., self.track_m / 2.
        self.yaw_lever_m = lx + ly if rollers == 'X' else lx - ly
        if abs(self.yaw_lever_m) < 1e-6:
            raise ValueError('O rollers with wheelbase == track: yaw not observable from the wheels')
        self.lateral = bool(lateral)
        self.gyro_scale_pos, self.gyro_scale_neg = float(gyro_scale_pos), float(gyro_scale_neg)
        self.wheel_yaw_scale = float(wheel_yaw_scale)
        self.static_s, self.initial_static_s = float(static_s), float(initial_static_s)
        self.static_gyro_std_dps, self.static_accel_std_g = float(static_gyro_std_dps), float(static_accel_std_g)
        self.accel_outlier_g, self.accel_outlier_frac = float(accel_outlier_g), float(accel_outlier_frac)
        self.accel_min_z_g = float(accel_min_z_g)
        self.bias_guard_s = float(bias_guard_s)
        self.bias_max_dps = None if bias_max_dps is None else float(bias_max_dps)
        self.gap_fallback_s = float(gap_fallback_s)
        self.imu_err_window_s, self.imu_err_max = float(imu_err_window_s), int(imu_err_max)
        self.spike_dps, self.spike_return_dps = float(spike_dps), float(spike_return_dps)
        self.spike_history = int(spike_history)
        self.slip_rate_dps, self.slip_hold_s = float(slip_rate_dps), float(slip_hold_s)
        self.redundancy_mps, self.redundancy_window_s = float(redundancy_mps), float(redundancy_window_s)
        self.shock_g, self.shock_baseline_s = float(shock_g), float(shock_baseline_s)
        self.untrusted_settle_s = float(untrusted_settle_s)
        self.max_delta_counts = int(max_delta_counts)
        self.m_per_count = math.pi * self.wheel_diam_m / self.counts_per_rev

    def placeholders(self):
        """Names of the knobs still at their unmeasured default."""
        now = {'wheel_map': self.wheel_map_text.replace(' ', '').lower(),
               'wheel_signs': ','.join('%+d' % self.wheel_signs[e] for e in ENCODERS), 'rollers': self.rollers,
               'gyro_scale_pos': self.gyro_scale_pos, 'gyro_scale_neg': self.gyro_scale_neg,
               'wheel_yaw_scale': self.wheel_yaw_scale, 'static_gyro_std_dps': self.static_gyro_std_dps,
               'static_accel_std_g': self.static_accel_std_g, 'slip_rate_dps': self.slip_rate_dps,
               'redundancy_mps': self.redundancy_mps, 'shock_g': self.shock_g}
        return sorted(k for k, v in now.items() if v == PLACEHOLDER_DEFAULTS[k])

    def describe(self):
        return {'wheel_diam_m': self.wheel_diam_m, 'counts_per_rev': self.counts_per_rev,
                'track_m': self.track_m, 'wheelbase_m': self.wheelbase_m, 'yaw_lever_m': self.yaw_lever_m,
                'wheel_map': self.wheel_map, 'wheel_signs': self.wheel_signs, 'rollers': self.rollers,
                'lateral': self.lateral, 'gyro_scale': [self.gyro_scale_pos, self.gyro_scale_neg],
                'wheel_yaw_scale': self.wheel_yaw_scale, 'static_s': self.static_s,
                'initial_static_s': self.initial_static_s, 'static_gyro_std_dps': self.static_gyro_std_dps,
                'static_accel_std_g': self.static_accel_std_g, 'bias_guard_s': self.bias_guard_s,
                'accel_outlier': [self.accel_outlier_g, self.accel_outlier_frac], 'accel_min_z_g': self.accel_min_z_g,
                'bias_max_dps': self.bias_max_dps, 'gap_fallback_s': self.gap_fallback_s,
                'imu_err': [self.imu_err_max, self.imu_err_window_s],
                'spike': [self.spike_dps, self.spike_return_dps, self.spike_history],
                'slip_rate_dps': self.slip_rate_dps, 'slip_hold_s': self.slip_hold_s,
                'redundancy_mps': self.redundancy_mps, 'redundancy_window_s': self.redundancy_window_s,
                'shock_g': self.shock_g, 'untrusted_settle_s': self.untrusted_settle_s,
                'placeholders': self.placeholders()}


# ------------------------------------------------------------------------------------------ kinematics
def mecanum_forward(fl, fr, rl, rr, yaw_lever_m, rollers='X'):
    """Wheel distances (m, forward-rolling positive) -> (forward, lateral left, dyaw rad, redundancy m).

    X (section 8.4): forward = sum / 4, lateral = (-FL + FR + RL - RR) / 4, dyaw = (-FL + FR - RL + RR) /
    (4 (lx + ly)), redundancy = (FL + FR) - (RL + RR). O is the mirrored mounting (every roller turned by
    90 deg): lateral = (FL - FR - RL + RR) / 4 and dyaw = (FL - FR + RL - RR) / (4 (lx - ly)) -- with
    lx - ly = 0.01 m on this car such a mounting would hardly turn, so a car that spins in place is X in
    this convention whatever its photo is called."""
    forward = (fl + fr + rl + rr) / 4.
    if rollers == 'X':
        lateral = (-fl + fr + rl - rr) / 4.
        dyaw = (-fl + fr - rl + rr) / (4. * yaw_lever_m)
    else:
        lateral = (fl - fr - rl + rr) / 4.
        dyaw = (fl - fr + rl - rr) / (4. * yaw_lever_m)
    return forward, lateral, dyaw, (fl + fr) - (rl + rr)


def mecanum_inverse(forward, lateral, dyaw, yaw_lever_m, rollers='X'):
    """(forward, lateral, dyaw) -> wheel distances (fl, fr, rl, rr); exact inverse of mecanum_forward."""
    k = dyaw * yaw_lever_m
    if rollers == 'X':
        return forward - lateral - k, forward + lateral + k, forward + lateral - k, forward - lateral + k
    return forward + lateral + k, forward - lateral - k, forward - lateral + k, forward + lateral - k


def _median(values):
    s = sorted(values)
    n = len(s)
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.


def _std(values):
    n = len(values)
    if n < 2:
        return 0.
    mean = math.fsum(values) / n
    return math.sqrt(math.fsum((v - mean) ** 2 for v in values) / n)


# ------------------------------------------------------------------------------------------ bias
class StaticBias:
    """Section 8.3 static detector and bias estimate. feed() once per frame, in time order.

    criteria: the subset of STATIC_CRITERIA in force (default all four). Anything less is a deliberately
    wrong detector used only as the negative control of the tests ('wheels' alone = the 10-17 mistake)."""

    def __init__(self, params, criteria=STATIC_CRITERIA):
        unknown = set(criteria) - set(STATIC_CRITERIA)
        if unknown:
            raise ValueError('unknown static criteria %s' % sorted(unknown))
        self.p, self.criteria = params, frozenset(criteria)
        self.run = deque()                  # (t, rate, amag) of the current run of per-sample conditions
        self.segment_start = None           # t of the first sample of the confirmed static stretch
        self.pending = deque()              # stretch samples not committed yet (younger than bias_guard_s)
        self.sum = 0.
        self.n = 0
        self.first_t = None                 # first committed sample of this stretch
        self.bias_dps = None
        self.updated_t = None
        self.static_total_s = 0.
        self.static_now = False
        self.last_t = None
        self.counters = {'stretches': 0, 'updates': 0, 'rejected_bias_max': 0, 'samples_dropped_guard': 0}

    @property
    def ready(self):
        return self.bias_dps is not None

    def _end_stretch(self):
        if self.segment_start is not None:
            self.counters['samples_dropped_guard'] += len(self.pending)
        self.segment_start = None
        self.pending.clear()
        self.sum, self.n, self.first_t = 0., 0, None
        self.static_now = False

    def feed(self, t, moved, command_stop, rate_dps, accel_g):
        """One frame: wheels moved this frame?, last command stop?, final raw rate (deg/s or None = no gyro),
        |accel| in g (or None)."""
        dt = 0. if self.last_t is None else t - self.last_t
        self.last_t = t
        base_ok = (('wheels' not in self.criteria or not moved)
                   and ('command' not in self.criteria or command_stop))
        if not base_ok:
            self.run.clear()
            self._end_stretch()
            return False
        if rate_dps is None:                # no gyro reading: proves nothing either way, skip the sample
            return self.static_now
        self.run.append((t, rate_dps, accel_g))
        window = [s for s in self.run if s[0] > t - self.p.static_s - 1e-9]
        while self.run and self.run[0][0] < t - max(self.p.initial_static_s, self.p.static_s) - 1.:
            self.run.popleft()              # (the run keeps enough for the window; older samples unused)
        span = t - self.run[0][0]
        calm = span >= self.p.static_s - 1e-6
        if calm and 'gyro_std' in self.criteria:
            calm = _std([s[1] for s in window]) < self.p.static_gyro_std_dps
        if calm and 'accel_std' in self.criteria:
            acc = [s[2] for s in window if s[2] is not None]
            calm = len(acc) == len(window)
            if calm:
                med = _median(acc)
                inliers = [a for a in acc if abs(a - med) <= self.p.accel_outlier_g]
                calm = (len(acc) - len(inliers) <= self.p.accel_outlier_frac * len(acc)
                        and _std(inliers) < self.p.static_accel_std_g)
        if not calm:
            self._end_stretch()
            return False
        if self.segment_start is None:      # a new static stretch: its first window counts in full
            self.counters['stretches'] += 1
            self.segment_start = window[0][0]
            self.pending.extend(window)
            self.static_total_s += self.p.static_s
        else:
            self.pending.append((t, rate_dps, accel_g))
            self.static_total_s += dt
        self.static_now = True
        while self.pending and self.pending[0][0] <= t - self.p.bias_guard_s + 1e-9:
            s = self.pending.popleft()
            self.sum += s[1]
            self.n += 1
            if self.first_t is None:
                self.first_t = s[0]
        need = self.p.initial_static_s if self.bias_dps is None else self.p.static_s
        if self.n and t - self.segment_start >= need - 1e-6:
            mean = self.sum / self.n
            if self.p.bias_max_dps is not None and abs(mean) > self.p.bias_max_dps:
                self.counters['rejected_bias_max'] += 1
            else:
                self.bias_dps, self.updated_t = mean, t
                self.counters['updates'] += 1
        return True

    def status(self):
        return {'bias_dps': self.bias_dps, 'ready': self.ready, 'updated_t': self.updated_t,
                'static_now': self.static_now, 'static_total_s': round(self.static_total_s, 3),
                'criteria': sorted(self.criteria), **self.counters}


# ------------------------------------------------------------------------------------------ slip / crash
class SlipDetector:
    """Section 8.5: any of three flags -> untrusted; trusted again after untrusted_settle_s with no flag."""

    def __init__(self, params):
        self.p = params
        self.mismatch_since = None
        self.red = deque()                  # (t, redundancy m, dt)
        self.acc = deque()                  # (t, ax g, ay g)
        self.trusted = True
        self.clean_since = None
        self.flags = ()
        self.transitions = []               # (t, trusted, flags) not yet drained
        self.counters = {'untrusted_events': 0, 'mismatch': 0, 'redundancy': 0, 'shock': 0}

    def feed(self, t, dt, wheel_rate_dps, gyro_rate_dps, redundancy_m, ax_g, ay_g):
        p = self.p
        flags = []
        if gyro_rate_dps is not None and abs(wheel_rate_dps - gyro_rate_dps) > p.slip_rate_dps:
            if self.mismatch_since is None:
                self.mismatch_since = t - dt
            if t - self.mismatch_since >= p.slip_hold_s - 1e-9:
                flags.append('mismatch')
        else:
            self.mismatch_since = None
        self.red.append((t, redundancy_m, dt))
        while self.red and self.red[0][0] <= t - p.redundancy_window_s + 1e-9:
            self.red.popleft()
        span = math.fsum(r[2] for r in self.red)
        if span > 0 and abs(math.fsum(r[1] for r in self.red)) / span > p.redundancy_mps:
            flags.append('redundancy')
        if ax_g is not None:
            base = [(a[1], a[2]) for a in self.acc if a[0] > t - p.shock_baseline_s]
            if len(base) >= 10:
                mx = math.fsum(b[0] for b in base) / len(base)
                my = math.fsum(b[1] for b in base) / len(base)
                if math.hypot(ax_g - mx, ay_g - my) > p.shock_g:
                    flags.append('shock')
            self.acc.append((t, ax_g, ay_g))
            while self.acc and self.acc[0][0] <= t - p.shock_baseline_s:
                self.acc.popleft()
        for f in flags:
            self.counters[f] += 1
        self.flags = tuple(flags)
        if flags:
            self.clean_since = None
            if self.trusted:
                self.trusted = False
                self.counters['untrusted_events'] += 1
                self.transitions.append((t, False, tuple(flags)))
        elif not self.trusted:
            if self.clean_since is None:
                self.clean_since = t
            if t - self.clean_since >= p.untrusted_settle_s - 1e-9:
                self.trusted = True
                self.clean_since = None
                self.transitions.append((t, True, ()))
        return self.trusted

    def status(self):
        return {'trusted': self.trusted, 'flags': list(self.flags), **self.counters}


# ------------------------------------------------------------------------------------------ odometry
class FusedOdometry:
    """0x82 wheel_counts bodies in, pose (x, y, yaw unwrapped, odom frame) and body twist out."""

    def __init__(self, params=None, static_criteria=STATIC_CRITERIA):
        self.p = params or FusedParams()
        self.x = self.y = self.yaw = 0.
        self.distance = 0.
        self.v = self.vy = self.w = 0.
        self.prev = None                    # dict of the previous frame
        self.pending_rebaseline = None
        self.t = 0.                         # unwrapped MCU time, s
        self.history = deque(maxlen=self.p.spike_history)
        self.pending = None                 # previous frame's gyro decision, finalised by the next frame
        self.held_rate = None
        self.missing_s = 0.
        self.imu_err_window = deque()       # (t, increments)
        self.imu_err_total = 0
        self.imu_err_raw = None
        self.bias = StaticBias(self.p, static_criteria)
        self.slip = SlipDetector(self.p)
        self.source = None
        self.fallback_reasons = ()
        self.fallback_s = 0.
        self.fallback_by_reason = {'bias_not_ready': 0., 'gyro_gap': 0., 'imu_err': 0.}
        self.gyro_s = 0.
        self.counters = {'frames': 0, 'baselines': 0, 'rebaselines': 0, 'jumps': 0, 'duplicates': 0,
                         'integrated': 0, 'gyro_missing_frames': 0, 'spike_suspects': 0, 'spikes_replaced': 0,
                         'spike_suspects_kept': 0, 'accel_invalid': 0}
        self.stamp_ns = None

    # -- helpers
    def rebaseline(self, reason):
        self.pending_rebaseline = str(reason)

    def set_pose(self, x, y, yaw):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)

    def apply_body_delta(self, dx_body, dy_body, dyaw):
        """Advance by a motion measured elsewhere (0x81 frames between 0x82 ones), body frame of the start."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += c * dx_body - s * dy_body
        self.y += s * dx_body + c * dy_body
        self.yaw += dyaw

    def _gyro_dyaw(self, rate_dps, bias_dps, dt):
        c = rate_dps - bias_dps
        return math.radians(c * (self.p.gyro_scale_pos if c >= 0 else self.p.gyro_scale_neg)) * dt

    def _wheel_distances(self, row, prev):
        deltas = {e: s32_delta(row['enc_' + e], prev['enc_' + e]) for e in ENCODERS}
        return deltas, [self.p.wheel_signs[self.p.wheel_map[pos]] * deltas[self.p.wheel_map[pos]]
                        * self.p.m_per_count for pos in POSITIONS]

    @staticmethod
    def _command_stop(row):
        cmd = row.get('cmd')
        return cmd is None or not cmd[2] or (cmd[0] == 0 and cmd[1] == 0)

    def _finalise(self, next_rate):
        """Decide the previous frame's rate now that the next one is known; returns the yaw correction."""
        pd, self.pending = self.pending, None
        if pd is None:
            return 0.
        final = pd['raw']
        if pd['suspect']:
            if next_rate is None or abs(next_rate - pd['median']) <= self.p.spike_return_dps:
                final = pd['median']
                self.counters['spikes_replaced'] += 1
            else:
                self.counters['spike_suspects_kept'] += 1
        correction = 0.
        # gyro_dt: this frame's dt plus the dt of the empty frames after it that used its provisional rate
        gdt = pd.get('gyro_dt', pd['dt'] if pd['source'] == 'gyro' else 0.)
        if gdt > 0. and final is not None and final != pd['prov'] and pd['prov'] is not None:
            correction = self._gyro_dyaw(final, pd['bias'], gdt) - self._gyro_dyaw(pd['prov'], pd['bias'], gdt)
        if final is not None:
            self.history.append(final)
            self.held_rate = final
        self.bias.feed(pd['t'], pd['moved'], pd['stop'], final, pd['amag'])
        for f in pd.get('followers', ()):   # the empty frames after it, in time order (no gyro reading)
            self.bias.feed(f[0], f[1], f[2], None, f[3])
        return correction

    # -- main
    def update(self, row):
        """One 0x82 wheel_counts body. Returns a state dict ('event': first / rebaseline / jump / duplicate / ok)."""
        p = self.p
        self.counters['frames'] += 1
        if self.prev is None or self.pending_rebaseline is not None or row.get('after_silence'):
            event = 'first' if self.prev is None else 'rebaseline'
            self.counters['baselines' if event == 'first' else 'rebaselines'] += 1
            reason = self.pending_rebaseline or ('chassis status after silence' if row.get('after_silence') else None)
            self.pending_rebaseline = None
            self.pending = None             # no look-ahead across a break
            self.prev = row
            self.imu_err_raw = row['imu_err']
            self.stamp_ns = int(row['t_ns'])
            self.v = self.vy = self.w = 0.
            return self.state(event, reason=reason)
        prev = self.prev
        dms = (int(row['mcu_ms']) - int(prev['mcu_ms'])) % U32
        if dms == 0 or dms >= 1 << 31:
            self.counters['duplicates'] += 1
            return self.state('duplicate')
        dt = dms / 1000.
        deltas, (fl, fr, rl, rr) = self._wheel_distances(row, prev)
        if any(abs(d) > p.max_delta_counts for d in deltas.values()):
            self.counters['jumps'] += 1
            self.prev = row
            self.pending = None
            self.stamp_ns = int(row['t_ns'])
            self.v = self.vy = self.w = 0.
            return self.state('jump', delta_counts=[deltas[e] for e in ENCODERS])
        self.t += dt
        t = self.t
        forward, lateral, dyaw_wheel, redundancy = mecanum_forward(fl, fr, rl, rr, p.yaw_lever_m, p.rollers)
        dyaw_wheel *= p.wheel_yaw_scale
        # imu_err (u8, wraps)
        inc = (int(row['imu_err']) - int(self.imu_err_raw)) % 256
        self.imu_err_raw = row['imu_err']
        self.imu_err_total += inc
        self.imu_err_window.append((t, inc))
        while self.imu_err_window and self.imu_err_window[0][0] <= t - p.imu_err_window_s + 1e-9:
            self.imu_err_window.popleft()
        err_burst = sum(i for _, i in self.imu_err_window) > p.imu_err_max
        # gyro: decide the previous frame (one frame of look-ahead), then this frame provisionally
        valid = row['gz_n'] > 0
        raw = row['gz_sum'] / row['gz_n'] * GYRO_RAW_DEG_S if valid else None
        # The look-ahead needs the next frame WITH a reading. The MCU sends an empty gyro field every other
        # frame; deciding on an empty one treated every sharp change (turn start/stop) as a spike, and the
        # median then latched onto the old rate (car run 20261027-104727: gyro kept turning ~33 deg/s after
        # the wheels stopped). Empty frames now leave the pending decision open and borrow its rate.
        carry = (not valid) and self.pending is not None and self.pending['raw'] is not None
        correction = 0. if carry else self._finalise(raw)
        median = suspect = None
        if valid:
            self.missing_s = 0.
            if len(self.history) >= p.spike_history:
                median = _median(self.history)
                suspect = abs(raw - median) > p.spike_dps
            prov = median if suspect else raw
            if suspect:
                self.counters['spike_suspects'] += 1
        else:
            self.counters['gyro_missing_frames'] += 1
            self.missing_s += dt
            prov = self.pending['prov'] if carry else self.held_rate
        reasons = []
        if not self.bias.ready:
            reasons.append('bias_not_ready')
        if dt > p.gap_fallback_s + 1e-9 or self.missing_s > p.gap_fallback_s + 1e-9 or prov is None:
            reasons.append('gyro_gap')
        if err_burst:
            reasons.append('imu_err')
        bias_now = self.bias.bias_dps
        if reasons:
            self.source = 'wheel'
            dyaw = dyaw_wheel
            self.fallback_s += dt
            for r in reasons:
                self.fallback_by_reason[r] += dt
        else:
            self.source = 'gyro'
            dyaw = self._gyro_dyaw(prov, bias_now, dt)
            self.gyro_s += dt
        self.fallback_reasons = tuple(reasons)
        self.yaw += correction
        mid = self.yaw + dyaw / 2.
        lat = lateral if p.lateral else 0.
        self.x += math.cos(mid) * forward - math.sin(mid) * lat
        self.y += math.sin(mid) * forward + math.cos(mid) * lat
        self.yaw += dyaw
        self.distance += forward
        self.v, self.vy, self.w = forward / dt, lat / dt, dyaw / dt
        moved = any(deltas[e] != 0 for e in ENCODERS)
        ax, ay, az = row.get('ax'), row.get('ay'), row.get('az')
        amag = (math.sqrt(ax * ax + ay * ay + az * az) * ACCEL_RAW_G) if az is not None else None
        accel_ok = az is not None and az * ACCEL_RAW_G >= p.accel_min_z_g
        if az is not None and not accel_ok:
            self.counters['accel_invalid'] += 1
        if carry:
            pd = self.pending
            pd.setdefault('gyro_dt', pd['dt'] if pd['source'] == 'gyro' else 0.)
            if self.source == 'gyro':
                pd['gyro_dt'] += dt
            pd.setdefault('followers', []).append((t, moved, self._command_stop(row), amag))
        else:
            self.pending = {'t': t, 'dt': dt, 'raw': raw, 'prov': prov, 'median': median, 'suspect': bool(suspect),
                            'bias': bias_now, 'source': self.source, 'moved': moved,
                            'stop': self._command_stop(row), 'amag': amag}
        gyro_rate = None
        if self.source == 'gyro':
            gyro_rate = math.degrees(self._gyro_dyaw(prov, bias_now, dt) / dt)
        self.slip.feed(t, dt, math.degrees(dyaw_wheel / dt), gyro_rate, redundancy,
                       ax * ACCEL_RAW_G if accel_ok else None, ay * ACCEL_RAW_G if accel_ok else None)
        self.prev = row
        self.stamp_ns = int(row['t_ns'])
        self.counters['integrated'] += 1
        return self.state('ok', forward=forward, lateral=lateral, dyaw_wheel=dyaw_wheel, redundancy=redundancy,
                          rate_raw_dps=raw)

    def state(self, event, **extra):
        row = {'event': event, 'x': self.x, 'y': self.y, 'yaw': self.yaw, 'distance': self.distance,
               'v': self.v, 'vy': self.vy, 'w': self.w, 'stamp_ns': self.stamp_ns,
               'heading_source': self.source, 'fallback': list(self.fallback_reasons),
               'bias_dps': self.bias.bias_dps, 'pose_trusted': self.slip.trusted}
        row.update(extra)
        return row

    def drain_slip_transitions(self):
        rows, self.slip.transitions = self.slip.transitions, []
        return rows

    def status(self):
        return {'source_now': self.source, 'fallback_now': list(self.fallback_reasons),
                'fallback_s': round(self.fallback_s, 3),
                'fallback_by_reason_s': {k: round(v, 3) for k, v in self.fallback_by_reason.items()},
                'gyro_s': round(self.gyro_s, 3), 'bias': self.bias.status(),
                'gyro_scale': [self.p.gyro_scale_pos, self.p.gyro_scale_neg],
                'wheel_yaw_scale': self.p.wheel_yaw_scale, 'spikes_replaced': self.counters['spikes_replaced'],
                'imu_err_raw': self.imu_err_raw, 'imu_err_total': self.imu_err_total,
                'imu_err_last_window': sum(i for _, i in self.imu_err_window),
                'slip': self.slip.status(), 'lateral': self.p.lateral, 'counters': dict(self.counters)}
