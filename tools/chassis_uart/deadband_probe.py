#!/usr/bin/env python3
"""Chassis dead-band probe (2026-10-26): what is the smallest w / v the real chassis actually executes?

Why: the 10-22 find-object runs failed because DWB asked for +-0.021 rad/s and the car did not turn
(runs/findnav-audit-20261025/audit.md). This measures the dead band so DWB (car profile) min_speed_theta /
min_speed_xy can be set above it.

Plan (low to high, every level once; rotation also in both directions):
  rotate in place  w = 0.02 0.04 0.06 0.08 0.10 0.15 rad/s, +w then -w, 3 s each
  straight         v = 0.005 0.01 0.02 0.03 0.05 m/s, 2 s each (total forward 0.23 m < 0.3 m)
  1 s of zero command between levels, 1.5 s of zero command before the first (gyro bias window).
Commands go out at 50 Hz through host.chassis_uart.ChassisLink (the same protocol, sequence numbers and
upstream/link-lost rules as host/ros_chassis_driver.py) with flag b2 (0x82) by default, like the car stack.

Per level: encoder deltas (the 0x81 pair, or enc_a/enc_b of 0x82 = what the wheel odometry reads), gyro z
mean minus the bias of the zero window (0x82 gyro is raw, not bias-corrected), measured / commanded.
  steady window   from settle_s (0.5 s) after the first command of the level to its last command
  MOVED (w)       |gyro w| >= 50 % of |cmd| AND mean |encoder delta| over the whole level >= 20 counts
                  (20 counts = 3.3 mm of wheel travel at 0.081 m / 1560 counts; the 0.02 rad/s level gives
                  ~72 counts on the real car if executed). No gyro sample -> encoder w (dR-dL)/track instead.
  MOVED (v)       |encoder v| >= 50 % of cmd AND the same 20-count rule
  a level counts as moved only if every run of it moved (both directions for w)
  min effective   the smallest level from which every larger level moved (null if the largest did not)
  recommended     min effective x 1.5, rounded UP to --round-to (0.01): min_speed_theta / min_speed_xy
A wrong sign (forward reads negative, +w reads clockwise) does not change MOVED; it is flagged per row.

Safety: levels above 0.2 rad/s or 0.06 m/s are refused (exit 2), ChassisLink limits are 60 mm/s and
200 mrad/s as a second cap. Before driving: status frames must arrive, gyro calibration must be over and,
after 1.5 s of enable=1 zero commands, the MCU must report heartbeat_seen and NOT stop_latched -- otherwise
exit 2 "MCU latched: no KV260 heartbeat (start the heartbeat source), this is NOT a dead band".
During a level: no status frame for 100 ms (link lost) or stop_latched for 0.5 s on end (heartbeat really
gone) -> abort. A shorter latch (the 10-22 car saw ~30 ms heartbeat gaps) only marks that level invalid; every
invalid level is run once more at the end and the analysis takes the LAST run of each commanded value.
Ctrl-C, any exception and every abort: enable=0 zero frames for 0.5 s, then the port is closed.
The operator must type y after "car surroundings 0.5 m clear, owner present" (--yes skips; tests only).

Modes
  serial   --port /dev/fpga_chassis (real MCU, or the pty printed by mcu_sim.py --pty on the host)
  sim      in-process fake MCU (mcu_sim.McuModel with an injected dead band; virtual time, instant)
  analyze  --record FILE: re-run the analysis on a JSON-lines record (this tool's --record, or the stack's
           chassis_record.jsonl -- same row format; levels are found by their exact commanded values, last
           contiguous run; on a stack record pass --since-ns so earlier DWB commands do not match)
  --dry-run prints the plan and exits.
On the car (the MCU only drives while the KV260 PL sends its heartbeat, i.e. while a stack feeds it depth
frames; the stack must NOT hold the serial port): heartbeat source = the bench stack with --chassis none, as in
REAL_CAR_RUNBOOK_20261015.md step 3 (bench_line4.sh run, CAR_GROUND_CALIB=off WINDOW_S=1800 FIND_ONLY=1 ...);
port = /dev/fpga_chassis (udev link of nano_uart_setup.sh install; `bash tools/chassis_uart/nano_uart_setup.sh
check` names the ttyTHS of 3100000.serial if the link is missing); `fuser /dev/fpga_chassis` must be empty.
  python3 tools/chassis_uart/deadband_probe.py serial --dry-run
  python3 tools/chassis_uart/deadband_probe.py serial --port /dev/fpga_chassis --record R/deadband.jsonl --out R/deadband.json
Exit: 0 measured (at least one level of each kind moved), 1 measured but some kind never moved,
2 could not run / refused / MCU latched / link lost, 3 interrupted (Ctrl-C).
Python 3.8 compatible.
"""
import argparse
import json
import math
import os
import select
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from host.chassis_uart import (BAUD, CENTI_DEG_TO_RAD, GYRO_RAW_DEG_S, ChassisLink, FrameParser,  # noqa: E402
                               open_serial, write_all)
from host.ros_chassis_driver import record_row  # noqa: E402  (stdlib + host only at import time)

W_LEVELS = (0.02, 0.04, 0.06, 0.08, 0.10, 0.15)
V_LEVELS = (0.005, 0.01, 0.02, 0.03, 0.05)
W_CAP, V_CAP = 0.2, 0.06                 # rad/s, m/s: hard limits of this tool
FORWARD_CAP_M = 0.3
TICK_NS = 20_000_000                     # 50 Hz commands
MS = 1_000_000
# real car (tools/car_run/car_up.sh:217, PROGRESS-22): 0.081 m wheel, 1560 counts/rev, 0.394 m effective track
WHEEL_DIAM_M, COUNTS_PER_REV, TRACK_M = 0.081, 1560, 0.394
MOVED_RATIO, MIN_COUNTS = 0.5, 20
LATCH_ABORT_S = 0.5                      # continuous stop_latched this long -> abort
RUN_GAP_NS = 100 * 1_000_000             # matching commands further apart start a new run


class ProbeAbort(Exception):
    def __init__(self, reason, code=2):
        Exception.__init__(self, reason)
        self.reason, self.code = reason, code


# ---- plan ------------------------------------------------------------------------------------------------
def build_plan(w_levels=W_LEVELS, v_levels=V_LEVELS, w_step_s=3., v_step_s=2., pause_s=1.):
    """List of steps {kind, level, v, w, seconds, pause_s}; refuses anything over the caps (ValueError)."""
    for w in w_levels:
        if not 0 < w <= W_CAP:
            raise ValueError('w level %g outside (0, %g] rad/s' % (w, W_CAP))
    for v in v_levels:
        if not 0 < v <= V_CAP:
            raise ValueError('v level %g outside (0, %g] m/s' % (v, V_CAP))
    forward = sum(v * v_step_s for v in v_levels)
    if forward >= FORWARD_CAP_M:
        raise ValueError('straight levels would drive %.3f m >= %.2f m' % (forward, FORWARD_CAP_M))
    steps = []
    for w in sorted(w_levels):
        for sign in (1, -1):
            steps.append({'kind': 'w', 'level': w, 'v': 0., 'w': sign * w, 'seconds': w_step_s, 'pause_s': pause_s})
    for v in sorted(v_levels):
        steps.append({'kind': 'v', 'level': v, 'v': v, 'w': 0., 'seconds': v_step_s, 'pause_s': pause_s})
    return steps


def plan_text(plan, preroll_s):
    total = preroll_s + sum(s['seconds'] + s['pause_s'] for s in plan)
    lines = ['plan: %d steps, %.1f s, zero command %.1f s first (gyro bias)' % (len(plan), total, preroll_s)]
    for i, s in enumerate(plan):
        lines.append('  %2d  %s  v=%+.3f m/s  w=%+.3f rad/s  %.1f s, then 0 for %.1f s'
                     % (i, 'rotate  ' if s['kind'] == 'w' else 'straight', s['v'], s['w'], s['seconds'], s['pause_s']))
    lines.append('  straight total %.3f m' % sum(s['v'] * s['seconds'] for s in plan))
    return '\n'.join(lines)


def cmd_ints(step):
    return int(round(step['v'] * 1000.)), int(round(step['w'] * 1000.))


# ---- analysis (pure: record rows in, table out) ----------------------------------------------------------
def gyro_rad_s(row):
    if row.get('frame') == '0x82' or 'gz_sum' in row:
        return math.radians(row['gz_sum'] / float(row['gz_n']) * GYRO_RAW_DEG_S) if row.get('gz_n') else None
    return row['gyro_z_cdps'] * CENTI_DEG_TO_RAD if row.get('gyro_valid') else None


def ceil_to(x, q):
    return round(math.ceil(x / q - 1e-9) * q, 6)


def analyze(rows, plan, wheel_diam_m=WHEEL_DIAM_M, counts_per_rev=COUNTS_PER_REV, track_m=TRACK_M,
            settle_s=0.5, bias_window_s=1.0, moved_ratio=MOVED_RATIO, min_counts=MIN_COUNTS, round_to=0.01):
    """rows: record rows ({'dir': 'tx', t_ns, v_mm_s, w_mrad_s, enable} / {'dir': 'rx', t_ns, left_count,
    right_count, gyro fields, status bits}), any order. Every plan step is found by its exact commanded
    (v_mm_s, w_mrad_s) with enable; the first..last such command is its window."""
    m_per_count = math.pi * wheel_diam_m / counts_per_rev
    tx = sorted((r for r in rows if r.get('dir') == 'tx'), key=lambda r: r['t_ns'])
    rx = sorted((r for r in rows if r.get('dir') == 'rx'), key=lambda r: r['t_ns'])
    windows = []
    for step in plan:
        v, w = cmd_ints(step)
        hits = [r['t_ns'] for r in tx if r.get('enable') and r['v_mm_s'] == v and r['w_mrad_s'] == w]
        start = len(hits) - 1                 # last contiguous run (a re-run of an invalid level wins)
        while start > 0 and hits[start] - hits[start - 1] <= RUN_GAP_NS:
            start -= 1
        windows.append((hits[start], hits[-1]) if hits else None)
    first = min((wd[0] for wd in windows if wd), default=None)
    bias_rows = [g for g in (gyro_rad_s(r) for r in rx
                             if first is not None and first - bias_window_s * 1e9 <= r['t_ns'] < first) if g is not None]
    bias = sum(bias_rows) / len(bias_rows) if bias_rows else 0.
    table = []
    for step, wd in zip(plan, windows):
        row = {'kind': step['kind'], 'cmd_v': step['v'], 'cmd_w': step['w'], 'moved': False}
        table.append(row)
        if wd is None:
            row['invalid'] = 'not commanded'
            continue
        t0, t1 = wd
        inside = [r for r in rx if t0 <= r['t_ns'] <= t1 + 20 * MS]
        if len(inside) < 2:
            row['invalid'] = 'no status frames'
            continue
        gaps = [b['t_ns'] - a['t_ns'] for a, b in zip(inside, inside[1:])]
        latched = sum(1 for r in inside if r.get('stop_latched'))
        not_enabled = sum(1 for r in inside if r['t_ns'] > t0 + 200 * MS and not r.get('motor_enabled'))
        row.update(seconds=round((t1 - t0) / 1e9, 3), frames=len(inside), latched_frames=latched,
                   not_enabled_frames=not_enabled, max_gap_ms=round(max(gaps) / 1e6, 1))
        steady = [r for r in inside if r['t_ns'] >= t0 + settle_s * 1e9]
        if len(steady) < 2:
            steady = inside
        dl_all = inside[-1]['left_count'] - inside[0]['left_count']
        dr_all = inside[-1]['right_count'] - inside[0]['right_count']
        dt = (steady[-1]['t_ns'] - steady[0]['t_ns']) / 1e9
        dl = steady[-1]['left_count'] - steady[0]['left_count']
        dr = steady[-1]['right_count'] - steady[0]['right_count']
        enc_v = (dl + dr) / 2. * m_per_count / dt if dt > 0 else 0.
        enc_w = (dr - dl) * m_per_count / track_m / dt if dt > 0 else 0.
        g = [x for x in (gyro_rad_s(r) for r in steady) if x is not None]
        gyro_w = sum(g) / len(g) - bias if g else None
        g_all = [(r['t_ns'], gyro_rad_s(r)) for r in inside]
        angle = sum((b[0] - a[0]) / 1e9 * (a[1] - bias) for a, b in zip(g_all, g_all[1:]) if a[1] is not None)
        mean_abs_counts = (abs(dl_all) + abs(dr_all)) / 2.
        row.update(enc_left=dl_all, enc_right=dr_all, enc_v=round(enc_v, 5), enc_w=round(enc_w, 5),
                   gyro_w=None if gyro_w is None else round(gyro_w, 5), gyro_angle_rad=round(angle, 4))
        if latched or not_enabled > 2 or max(gaps) > 100 * MS:
            row['invalid'] = 'latched' if latched else 'motor not enabled' if not_enabled > 2 else 'link gap'
            continue
        if step['kind'] == 'w':
            meas = gyro_w if gyro_w is not None else enc_w
            row['source'] = 'gyro' if gyro_w is not None else 'encoder'
            cmd = step['w']
        else:
            meas, cmd, row['source'] = enc_v, step['v'], 'encoder'
        row['measured'] = round(meas, 5)
        row['ratio'] = round(meas / cmd, 3)
        row['sign_ok'] = meas * cmd > 0
        row['moved'] = abs(meas) >= moved_ratio * abs(cmd) and mean_abs_counts >= min_counts
    out = {'bias_rad_s': round(bias, 6), 'bias_samples': len(bias_rows), 'table': table,
           'criteria': {'moved_ratio': moved_ratio, 'min_mean_abs_counts': min_counts, 'settle_s': settle_s,
                        'm_per_count': m_per_count, 'track_m': track_m, 'round_to': round_to}}
    for kind, key in (('w', 'min_speed_theta'), ('v', 'min_speed_xy')):
        levels = sorted({s['level'] for s in plan if s['kind'] == kind})
        moved = {L: all(r['moved'] for s, r in zip(plan, table) if s['kind'] == kind and s['level'] == L)
                 for L in levels}
        invalid = sorted({s['level'] for s, r in zip(plan, table) if s['kind'] == kind and 'invalid' in r})
        min_eff = None
        for L in reversed(levels):
            if not moved[L]:
                break
            min_eff = L
        out[kind] = {'levels_moved': {('%g' % L): moved[L] for L in levels}, 'min_effective': min_eff,
                     'non_monotone': any(moved[L] for L in levels if min_eff is None or L < min_eff),
                     'invalid_levels': invalid,
                     'recommended_' + key: None if min_eff is None else ceil_to(min_eff * 1.5, round_to)}
    return out


def table_text(result):
    lines = ['%-8s %8s %8s %9s %6s %7s %7s %5s %s' % ('kind', 'cmd', 'meas', 'ratio', 'src', 'encL', 'encR',
                                                      'moved', 'note')]
    for r in result['table']:
        cmd = r['cmd_w'] if r['kind'] == 'w' else r['cmd_v']
        lines.append('%-8s %+8.3f %8s %9s %6s %7s %7s %5s %s' % (
            'rotate' if r['kind'] == 'w' else 'straight', cmd,
            '-' if 'measured' not in r else '%+.4f' % r['measured'], r.get('ratio', '-'), r.get('source', '-'),
            r.get('enc_left', '-'), r.get('enc_right', '-'), 'YES' if r['moved'] else 'no',
            r.get('invalid', 'SIGN reversed' if r['moved'] and not r.get('sign_ok', True) else '')))
    for kind, key, unit in (('w', 'min_speed_theta', 'rad/s'), ('v', 'min_speed_xy', 'm/s')):
        k = result[kind]
        lines.append('%s: min effective %s %s -> recommended %s = %s%s%s' % (
            'rotate' if kind == 'w' else 'straight', k['min_effective'], unit, key, k['recommended_' + key],
            '  (NON-MONOTONE: a smaller level moved but a larger one did not)' if k['non_monotone'] else '',
            '  invalid levels %s' % k['invalid_levels'] if k['invalid_levels'] else ''))
    lines.append('gyro bias %.5f rad/s from %d frames; MOVED = |meas| >= %g x |cmd| and mean |encoder delta| >= %d'
                 % (result['bias_rad_s'], result['bias_samples'], result['criteria']['moved_ratio'],
                    result['criteria']['min_mean_abs_counts']))
    return '\n'.join(lines)


# ---- transports --------------------------------------------------------------------------------------------
class SerialTransport:
    def __init__(self, fd):
        self.fd = fd

    def now_ns(self):
        return time.monotonic_ns()

    def write(self, data):
        write_all(self.fd, data)

    def wait_until(self, t_ns):
        """Read until monotonic t_ns; returns [(bytes, received_ns)]."""
        out = []
        while True:
            left = (t_ns - time.monotonic_ns()) / 1e9
            if left <= 0:
                return out
            if select.select([self.fd], [], [], left)[0]:
                try:
                    data = os.read(self.fd, 4096)
                except BlockingIOError:
                    continue
                if data:
                    out.append((data, time.monotonic_ns()))


class SimTransport:
    """In-process fake MCU on a virtual clock: model.receive on write, model.step every 10 ms.
    status_until_ms: stop delivering status frames after this MCU ms (link-lost tests)."""

    def __init__(self, model, status_until_ms=None):
        self.model, self.ms, self.status_until_ms = model, 0, status_until_ms
        self.written = []

    def now_ns(self):
        return self.ms * MS

    def write(self, data):
        self.written.append(data)
        self.model.receive(data, self.ms)

    def wait_until(self, t_ns):
        out = []
        while self.ms < t_ns // MS:
            self.ms += 1
            if self.ms % 10 == 0:
                frame = self.model.step(self.ms)
                if frame and (self.status_until_ms is None or self.ms < self.status_until_ms):
                    out.append((frame, self.ms * MS))
        self.model.advance(self.ms)
        return out


def make_deadband_mcu(deadband_w=0.04, deadband_v=0.015, heartbeat=True, heartbeat_fn=None, status2=True, **kw):
    """mcu_sim.McuModel whose wheels ignore |w| < deadband_w rad/s and |v| < deadband_v m/s (strictly less)."""
    from tools.chassis_uart.mcu_sim import McuModel, Status2Faults
    db_w, db_v = int(round(deadband_w * 1000)), int(round(deadband_v * 1000))

    class DeadbandMcu(McuModel):
        def receive(self, data, now_ms):
            McuModel.receive(self, data, now_ms)
            v, w = self.target
            self.target = (v if abs(v) >= db_v else 0, w if abs(w) >= db_w else 0)

    params = dict(track_mm=TRACK_M * 1000, wheel_diam_mm=WHEEL_DIAM_M * 1000, counts_per_rev=COUNTS_PER_REV,
                  calib_ms=500, heartbeat=heartbeat, heartbeat_fn=heartbeat_fn,
                  status2_faults=Status2Faults(lx_ly_mm=TRACK_M * 500))
    params.update(kw)
    return DeadbandMcu(**params)


# ---- driver ------------------------------------------------------------------------------------------------
class Prober:
    def __init__(self, transport, plan, status2=True, preroll_s=1.5, calib_timeout_s=8., log=print,
                 hook=None):
        self.t, self.plan, self.preroll_s, self.calib_timeout_s = transport, plan, preroll_s, calib_timeout_s
        self.link = ChassisLink(v_limit_mm_s=int(V_CAP * 1000), w_limit_mrad_s=int(W_CAP * 1000), status2=status2)
        self.parser = FrameParser()
        self.rows, self.log, self.hook = [], log, hook
        self.latched_since = None
        self.seen_status = False
        self.zero_sent = 0
        self.retried = []

    def _tick(self, v, w, phase, check=True):
        now = self.t.now_ns()
        if self.hook is not None:
            self.hook(self, phase, now)
        if v is None:
            self.link.upstream = None
        else:
            self.link.on_velocity(max(-V_CAP, min(V_CAP, v)), max(-W_CAP, min(W_CAP, w)), now)
        frame, decision = self.link.next_command(now)
        self.t.write(frame)
        decision.update(dir='tx', t_ns=now, phase=phase)
        self.rows.append(decision)
        for data, received in self.t.wait_until(now + TICK_NS):
            for st in self.parser.feed(data):
                derived = self.link.on_status(st, received)
                self.rows.append(record_row(st, derived))
                self.seen_status = True
                if not st['stop_latched']:
                    self.latched_since = None
                elif self.latched_since is None:
                    self.latched_since = received
        if check and self.seen_status:
            if not self.link.link_alive(self.t.now_ns()):
                raise ProbeAbort('link lost: no MCU status frame for 100 ms (heartbeat lost) during %s' % phase)
            if self.latched_since is not None and self.t.now_ns() - self.latched_since >= LATCH_ABORT_S * 1e9:
                raise ProbeAbort('MCU stop_latched for %.1f s during %s: KV260 heartbeat lost (not a dead band)'
                                 % (LATCH_ABORT_S, phase))

    def _hold(self, v, w, seconds, phase, check=True):
        end = self.t.now_ns() + int(seconds * 1e9)
        while self.t.now_ns() < end:
            self._tick(v, w, phase, check)

    def last_rx(self):
        return next((r for r in reversed(self.rows) if r['dir'] == 'rx'), None)

    def start(self):
        # status frames at all
        self._hold(0., 0., 1.0, 'listen', check=False)
        if not self.seen_status:
            raise ProbeAbort('no MCU status frame in 1 s: wrong port, MCU off, or the car stack holds the port')
        waited = 0.
        while not self.last_rx().get('gyro_calibrated') or self.last_rx().get('gyro_calibrating'):
            if waited >= self.calib_timeout_s:
                raise ProbeAbort('gyro calibration not finished after %.0f s (b6 set / b5 clear)' % waited)
            self._hold(0., 0., 0.5, 'calibration', check=False)
            waited += 0.5
        self._hold(0., 0., self.preroll_s, 'preroll', check=False)
        last = self.last_rx()
        if last.get('stop_latched') or not last.get('heartbeat_seen'):
            raise ProbeAbort('MCU latched / no heartbeat after %.1f s of zero commands (stop_latched=%s '
                             'heartbeat_seen=%s): start the KV260 heartbeat source first -- this is NOT a dead band'
                             % (self.preroll_s, last.get('stop_latched'), last.get('heartbeat_seen')))
        if not self.link.link_alive(self.t.now_ns()):
            raise ProbeAbort('link lost before the first level')

    def run_steps(self, analysis_kw=None):
        for i, step in enumerate(self.plan):
            self.log('step %d/%d %s v=%+.3f w=%+.3f' % (i + 1, len(self.plan), step['kind'], step['v'], step['w']))
            self._hold(step['v'], step['w'], step['seconds'], 'step%d' % i)
            self._hold(0., 0., step['pause_s'], 'pause%d' % i)
        table = analyze(self.rows, self.plan, **(analysis_kw or {}))['table']
        self.retried = [i for i, row in enumerate(table) if 'invalid' in row]
        for i in self.retried:              # once: a short heartbeat gap must not cost the whole run
            step = self.plan[i]
            self.log('retry step %d (%s) v=%+.3f w=%+.3f' % (i + 1, table[i]['invalid'], step['v'], step['w']))
            self._hold(step['v'], step['w'], step['seconds'], 'retry%d' % i)
            self._hold(0., 0., step['pause_s'], 'retrypause%d' % i)

    def stop(self, seconds=0.5):
        """enable=0 frames for `seconds` (upstream cleared -> ChassisLink sends v=w=0 enable=0). Never raises."""
        end = self.t.now_ns() + int(seconds * 1e9)
        try:
            while self.t.now_ns() < end:
                self._tick(None, None, 'stop', check=False)
                self.zero_sent += 1
        except Exception as error:      # still try single frames: the port may be gone
            self.log('stop: %s: %s' % (type(error).__name__, error))


def run_probe(transport, plan, status2=True, preroll_s=1.5, log=print, hook=None, analysis_kw=None):
    """Returns (exit code, result dict). Always ends with Prober.stop()."""
    p = Prober(transport, plan, status2=status2, preroll_s=preroll_s, log=log, hook=hook)
    result = {'aborted': None}
    code = 0
    try:
        p.start()
        p.run_steps(analysis_kw)
    except ProbeAbort as error:
        result['aborted'], code = error.reason, error.code
    except KeyboardInterrupt:
        result['aborted'], code = 'interrupted (Ctrl-C)', 3
    except Exception as error:
        result['aborted'], code = '%s: %s' % (type(error).__name__, error), 2
    finally:
        p.stop()
    result['zero_frames_at_end'] = p.zero_sent
    result['retried_steps'] = p.retried
    result['rows'] = p.rows
    result['link'] = p.link.status(transport.now_ns())
    if result['aborted'] is None or any(r.get('dir') == 'tx' and str(r.get('phase', '')).startswith(('step', 'retry'))
                                        for r in p.rows):
        result.update(analyze(p.rows, plan, **(analysis_kw or {})))
        if code == 0:
            code = 0 if result['w']['min_effective'] is not None and result['v']['min_effective'] is not None else 1
    return code, result


# ---- CLI ---------------------------------------------------------------------------------------------------
def levels(text):
    return tuple(float(x) for x in text.split(',') if x.strip())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', choices=('serial', 'sim', 'analyze'))
    ap.add_argument('--port', default='/dev/fpga_chassis')
    ap.add_argument('--baud', type=int, default=BAUD)
    ap.add_argument('--record', help='analyze: input JSON lines; serial/sim: also write every frame here')
    ap.add_argument('--out', help='result JSON (table, min effective, recommendation)')
    ap.add_argument('--yes', action='store_true', help='skip the operator confirmation (tests only)')
    ap.add_argument('--dry-run', action='store_true', help='print the plan only')
    ap.add_argument('--w-levels', type=levels, default=W_LEVELS)
    ap.add_argument('--v-levels', type=levels, default=V_LEVELS)
    ap.add_argument('--w-step-s', type=float, default=3.)
    ap.add_argument('--v-step-s', type=float, default=2.)
    ap.add_argument('--pause-s', type=float, default=1.)
    ap.add_argument('--preroll-s', type=float, default=1.5)
    ap.add_argument('--status2', choices=('on', 'off'), default='on', help='ask for 0x82 like the car stack')
    ap.add_argument('--wheel-diam-m', type=float, default=WHEEL_DIAM_M)
    ap.add_argument('--counts-per-rev', type=int, default=COUNTS_PER_REV)
    ap.add_argument('--track-m', type=float, default=TRACK_M)
    ap.add_argument('--settle-s', type=float, default=0.5)
    ap.add_argument('--round-to', type=float, default=0.01)
    ap.add_argument('--since-ns', type=int, default=0, help='analyze: ignore rows before this t_ns')
    ap.add_argument('--sim-deadband-w', type=float, default=0.04)
    ap.add_argument('--sim-deadband-v', type=float, default=0.015)
    a = ap.parse_args(argv)
    try:
        plan = build_plan(a.w_levels, a.v_levels, a.w_step_s, a.v_step_s, a.pause_s)
    except ValueError as error:
        print('refused: %s' % error, file=sys.stderr)
        return 2
    akw = dict(wheel_diam_m=a.wheel_diam_m, counts_per_rev=a.counts_per_rev, track_m=a.track_m,
               settle_s=a.settle_s, round_to=a.round_to)
    print(plan_text(plan, a.preroll_s))
    if a.dry_run:
        return 0
    if a.mode == 'analyze':
        if not a.record:
            print('analyze needs --record', file=sys.stderr)
            return 2
        rows = []
        with open(a.record) as f:
            for line in f:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                if isinstance(r, dict) and r.get('t_ns', 0) >= a.since_ns:
                    rows.append(r)
        result = analyze(rows, plan, **akw)
        code = 0 if result['w']['min_effective'] is not None and result['v']['min_effective'] is not None else 1
    else:
        if a.mode == 'serial' and not a.yes:
            print('\n!! car surroundings 0.5 m clear of obstacles, owner present, hand on the power switch?\n'
                  '!! (车周围 0.5 m 内无障碍、owner 在场、手在电源开关旁) type y to start: ', end='', flush=True)
            try:
                answer = input().strip().lower()
            except EOFError:
                answer = ''
            if answer != 'y':
                print('not confirmed, nothing sent')
                return 2
        if a.mode == 'sim':
            transport = SimTransport(make_deadband_mcu(a.sim_deadband_w, a.sim_deadband_v,
                                                       status2=a.status2 == 'on'))
            fd = None
        else:
            try:
                fd = open_serial(a.port, a.baud)
            except OSError as error:
                print('cannot open %s: %s' % (a.port, error), file=sys.stderr)
                return 2
            transport = SerialTransport(fd)
        try:
            code, result = run_probe(transport, plan, status2=a.status2 == 'on', preroll_s=a.preroll_s,
                                     log=lambda s: print(s, flush=True), analysis_kw=akw)
        finally:
            if fd is not None:
                os.close(fd)
        rows = result.pop('rows')
        if a.record:
            with open(a.record, 'w') as f:
                for r in rows:
                    f.write(json.dumps(r) + '\n')
        if result['aborted']:
            print('ABORTED: %s (enable=0 sent for 0.5 s: %d frames)' % (result['aborted'], result['zero_frames_at_end']))
    result['mode'] = a.mode
    result['exit_code'] = code
    if 'table' in result:
        print(table_text(result))
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(result, f, indent=1)
        print('result -> %s' % a.out)
    return code


if __name__ == '__main__':
    sys.exit(main())
