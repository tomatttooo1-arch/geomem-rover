"""Wheel odometry from cumulative encoder counts, one conversion for both sources (2026-10-14).

Owner 10-14 (hybrid): navigation uses the counts the chassis MCU reports on the Nano command serial
link (status frame 0x81, CHASSIS_UART_PROTOCOL_20261012.md section 2); the PL keeps computing its own
odometry from the MCU -> KV260 encoder frame (PL_PERMIT_UART_SPEC_20261011.md 4.4) as competition
evidence and as a cross-check. Both reach this module as raw cumulative int32 counts, so the two
paths share every formula here; host/ros_wheel_odom.py only moves samples and messages.

  counts -> metres      d = delta_counts * pi * wheel_diam / counts_per_rev
  pose                  exact arc integration of (d_left, d_right) over track (constant curvature
                        between two samples, which is exact for constant wheel speeds)
  wrap                  delta taken modulo 2**32 as int32 (an i32 counter wrapping is not a jump)
  first sample          baseline only: no motion, no velocity
  jump                  |delta| > max_delta_counts on either wheel (PL default 32767, MAX_DELTA_COUNTS)
                        -> not integrated, new baseline (MCU reset / garbage), counted
  rebaseline(reason)    caller hint (chassis status after >100 ms silence, PL count_jumps moved, link
                        regained): next sample is a baseline, counted

PLACEHOLDER parameters (65 mm wheel, 1320 counts/rev with quadrature, 160 mm track) are the values
the PL encoder spec, its reference model (hw/m7/pl/tools/enc_uart_model.py selftest) and the fake MCU
(tools/chassis_uart/mcu_sim.py) were built with; dev2 has not confirmed them (protocol section 5 item 4).
Everything is I/O free and ROS free; time is integer nanoseconds of one clock.
"""
import math

DEFAULT_WHEEL_DIAM_M = 0.065
DEFAULT_COUNTS_PER_REV = 1320
DEFAULT_TRACK_M = 0.160
DEFAULT_MAX_DELTA_COUNTS = 32767          # PL register 0x14 reset value
PLACEHOLDER_NOTE = ('placeholder wheel parameters (65 mm / 1320 counts per rev / 160 mm track: PL spec '
                    '4.4-5, enc_uart_model selftest, mcu_sim defaults), not confirmed by dev2')

U32 = 1 << 32


def s32_delta(new, old):
    """new - old for two int32 counters, modulo 2**32 (an i32 wrap is a small step, not a jump)."""
    d = (int(new) - int(old)) % U32
    return d - U32 if d >= 1 << 31 else d


class WheelParams:
    def __init__(self, wheel_diam_m=DEFAULT_WHEEL_DIAM_M, counts_per_rev=DEFAULT_COUNTS_PER_REV,
                 track_m=DEFAULT_TRACK_M, max_delta_counts=DEFAULT_MAX_DELTA_COUNTS):
        if not (wheel_diam_m > 0 and counts_per_rev > 0 and track_m > 0 and max_delta_counts > 0):
            raise ValueError('wheel parameters must be positive')
        self.wheel_diam_m = float(wheel_diam_m)
        self.counts_per_rev = int(counts_per_rev)
        self.track_m = float(track_m)
        self.max_delta_counts = int(max_delta_counts)
        self.m_per_count = math.pi * self.wheel_diam_m / self.counts_per_rev

    @property
    def placeholder(self):
        return (self.wheel_diam_m, self.counts_per_rev, self.track_m) == (
            DEFAULT_WHEEL_DIAM_M, DEFAULT_COUNTS_PER_REV, DEFAULT_TRACK_M)

    def pl_registers(self):
        """The same constants as the PL encoder block wants them (spec 4.4: 0x08 / 0x0C / 0x10 / 0x14)."""
        return {'WHEEL_DIAM_UM': int(round(self.wheel_diam_m * 1e6)), 'COUNTS_PER_REV': self.counts_per_rev,
                'TRACK_UM': int(round(self.track_m * 1e6)), 'MAX_DELTA_COUNTS': self.max_delta_counts}

    def describe(self):
        return {'wheel_diam_m': self.wheel_diam_m, 'counts_per_rev': self.counts_per_rev,
                'track_m': self.track_m, 'max_delta_counts': self.max_delta_counts,
                'm_per_count': self.m_per_count, 'placeholder': self.placeholder}


def integrate_arc(x, y, yaw, d_left, d_right, track):
    """Pose after the wheels moved d_left / d_right metres along one constant-curvature arc."""
    ds = (d_left + d_right) / 2.
    dth = (d_right - d_left) / track
    if abs(dth) < 1e-9:
        # second-order midpoint: the arc formula's limit, error O(dth^2 * ds) < 1e-18 m here
        mid = yaw + dth / 2.
        return x + ds * math.cos(mid), y + ds * math.sin(mid), yaw + dth
    radius = ds / dth
    return (x + radius * (math.sin(yaw + dth) - math.sin(yaw)),
            y - radius * (math.cos(yaw + dth) - math.cos(yaw)), yaw + dth)


class WheelOdometry:
    """Cumulative counts in, pose (x, y, yaw unwrapped) and body velocity out."""

    def __init__(self, params=None):
        self.params = params or WheelParams()
        self.x = self.y = self.yaw = 0.
        self.distance = 0.                  # path length of the centre point (signed sum), metres
        self.v = self.w = 0.
        self.prev = None                    # (left, right, stamp_ns)
        self.stamp_ns = None
        self.counters = {'samples': 0, 'baselines': 0, 'jumps': 0, 'rebaselines': 0, 'integrated': 0,
                         'stale_or_duplicate': 0}
        self.pending_rebaseline = None

    def rebaseline(self, reason):
        self.pending_rebaseline = str(reason)

    def update(self, left, right, stamp_ns):
        """Feed one sample; returns a dict with 'event' in first / rebaseline / jump / ok / duplicate."""
        p = self.params
        self.counters['samples'] += 1
        left, right, stamp_ns = int(left), int(right), int(stamp_ns)
        event = 'ok'
        if self.prev is None or self.pending_rebaseline is not None:
            event = 'first' if self.prev is None else 'rebaseline'
            self.counters['baselines' if event == 'first' else 'rebaselines'] += 1
            reason, self.pending_rebaseline = self.pending_rebaseline, None
            self.prev = (left, right, stamp_ns)
            self.stamp_ns = stamp_ns
            self.v = self.w = 0.
            return self.state(event, reason=reason)
        pl, pr, pt = self.prev
        dl, dr = s32_delta(left, pl), s32_delta(right, pr)
        if stamp_ns <= pt:
            if (dl, dr) == (0, 0):
                self.counters['stale_or_duplicate'] += 1
                return self.state('duplicate')
            # counts moved but the clock did not: integrate the motion, no velocity estimate
        if abs(dl) > p.max_delta_counts or abs(dr) > p.max_delta_counts:
            self.counters['jumps'] += 1
            self.prev = (left, right, stamp_ns)
            self.stamp_ns = stamp_ns
            self.v = self.w = 0.
            return self.state('jump', delta_counts=(dl, dr))
        d_left, d_right = dl * p.m_per_count, dr * p.m_per_count
        self.x, self.y, self.yaw = integrate_arc(self.x, self.y, self.yaw, d_left, d_right, p.track_m)
        ds, dth = (d_left + d_right) / 2., (d_right - d_left) / p.track_m
        self.distance += ds
        if stamp_ns > pt:
            dt = (stamp_ns - pt) / 1e9
            self.v, self.w = ds / dt, dth / dt
        self.prev = (left, right, stamp_ns)
        self.stamp_ns = stamp_ns
        self.counters['integrated'] += 1
        return self.state(event, delta_counts=(dl, dr))

    def state(self, event, **extra):
        row = {'event': event, 'x': self.x, 'y': self.y, 'yaw': self.yaw, 'distance': self.distance,
               'v': self.v, 'w': self.w, 'stamp_ns': self.stamp_ns}
        row.update(extra)
        return row


def yaw_to_quaternion(yaw):
    """(x, y, z, w) of a rotation about +z."""
    return 0., 0., math.sin(yaw / 2.), math.cos(yaw / 2.)


# ---------------------------------------------------------------------------- source adapters
# Each turns one source's record into (left, right, stamp_ns, rebaseline_reason or None, key) where key
# identifies the underlying MCU frame (duplicates are skipped by the node).

def chassis_sample(row, received_ns, after_silence=False):
    """Status frame dict (host.chassis_uart decode) + its Nano stamp -> sample.

    A frame after more than 100 ms of silence (ChassisLink 'after_silence') may follow an MCU reset,
    whose counts restart at 0: a small negative step would otherwise be integrated as motion."""
    return (row['left_count'], row['right_count'], int(received_ns),
            'chassis status after silence' if after_silence else None, (row['seq'], row['mcu_ms']))


class PlSampleTracker:
    """TELEM encoder block (host.kv_status.decode_encoder output) -> sample, with the PL's own hints.

    The TELEM answer is a snapshot of the latest PL publication, polled slower than the 100 Hz MCU
    frames; counts are cumulative, so skipped frames lose no distance (the arc between two polls is
    taken as one constant-curvature segment). Same frame twice (update_count unchanged) -> None.
    Rebaseline when the PL counted a count jump (MCU reset) or the encoder link came back."""

    def __init__(self):
        self.last_update = None
        self.last_jumps = None
        self.last_link_ok = None

    def sample(self, block, poll_ns):
        if not block or not block.get('available'):
            return None
        update = block.get('update_count')
        if update is not None and update == self.last_update:
            return None
        self.last_update = update
        reasons = []
        jumps = (block.get('counters_named') or {}).get('count_jumps')
        if self.last_jumps is not None and jumps is not None and jumps != self.last_jumps:
            reasons.append('PL count_jumps %s -> %s' % (self.last_jumps, jumps))
        self.last_jumps = jumps
        link_ok = block.get('link_ok')
        if link_ok is False:                # LINK_OK=0: counts frozen at the last good frame (spec 6)
            self.last_link_ok = False
            return None
        if self.last_link_ok is False and link_ok:
            reasons.append('PL encoder link regained')
        self.last_link_ok = link_ok
        age_ms = block.get('frame_age_ms')
        stamp = int(poll_ns) - (int(round(age_ms * 1e6)) if age_ms is not None else 0)
        seq = block.get('seq') or {}
        return (block['left_count'], block['right_count'], stamp, '; '.join(reasons) or None,
                (seq.get('seq'), seq.get('mcu_ms'), update))
