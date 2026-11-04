"""Line D2 (2026-10-14): stop-chain timestamps of a simulated run, pure functions (no ROS; unit-tested).

Inputs (all in the run directory):
  motion.jsonl         record_motion.py: /clock (sim time) and /odom (header stamp = sim time) with
                       wall_ns (CLOCK_REALTIME at receipt), Twist topics incl. /fpga/chassis/sim_twist
  permit_telem.jsonl   permit_recorder.py: TELEM permit block per poll, monotonic + realtime stamps
  mcu.jsonl            mcu_sim.py --log: latch / unlock (wall_ns), executed speed changes, KV permit changes,
                       commands
Axes: the host permit model counts PL cycles on CLOCK_MONOTONIC; the recorder maps a PL stamp to
monotonic via the poll midpoint (error <= RTT/2) and to realtime with the same poll's offset; simulation
time comes from the nearest /clock samples by linear interpolation of (wall_ns, sim).
"""
import bisect

CAUSE_TIMEOUT, CAUSE_FORCE = 1, 2
STILL = 0.01                       # |v| + |w| below this = still (bottle_scenario.py)


def clock_map(motion_rows):
    """Sorted (wall_ns, sim_s) pairs from /clock rows."""
    pairs = []
    for row in motion_rows:
        if row.get('topic') == '/clock':
            c = row['message']['clock']
            pairs.append((int(row['wall_ns']), c['sec'] + c['nanosec'] / 1e9))
    pairs.sort()
    return pairs


def wall_to_sim(pairs, wall_ns, max_gap_ns=50_000_000):
    """Simulation time at wall_ns (linear between the neighbouring /clock samples); None outside the
    recorded range or across a gap wider than max_gap_ns."""
    if not pairs:
        return None
    i = bisect.bisect_left(pairs, (wall_ns, float('-inf')))
    if i < len(pairs) and pairs[i][0] == wall_ns:
        return pairs[i][1]
    if i == 0 or i == len(pairs):
        return None
    (w0, s0), (w1, s1) = pairs[i - 1], pairs[i]
    if w1 - w0 > max_gap_ns:
        return None
    return s0 + (s1 - s0) * (wall_ns - w0) / (w1 - w0)


def telem_rows(rows):
    """Successful TELEM polls with the poll midpoint on both clocks."""
    out = []
    clk = None
    for row in rows:
        if clk is None and row.get('pl_clk_hz'):
            clk = int(row['pl_clk_hz'])
        if not row.get('ok') or not row.get('permit') or not row['permit'].get('available'):
            continue
        mono = (row['mono_send_ns'] + row['mono_recv_ns']) // 2
        wall = (row['wall_send_ns'] + row['wall_recv_ns']) // 2
        out.append(dict(row, mono_mid=mono, wall_mid=wall))
    return out, (clk or 100000000)


def pl_to_wall(row, ts, clk):
    """Realtime ns of PL stamp ts, read through one TELEM row."""
    return row['wall_mid'] - (int(row['permit']['tb_now']) - int(ts)) * 1000000000 // clk


def permit_bit(row):
    return bool(((row['permit'].get('status_bits') or {}).get('permit')))


def revocations(rows, clk):
    """One dict per new revocation seen (revoke_count increase): its PL stamps, cause, wall times."""
    out, last = [], None
    for row in rows:
        p = row['permit']
        count = p.get('revoke_count')
        if last is not None and count is not None and count > last:
            cause = (p.get('status_bits') or {}).get('revoke_cause')
            if cause is None:
                cause = p.get('last_cause')
            out.append({'revoke_count': count, 'missed': count - last - 1, 'cause': cause,
                        'revoke_ts': p['last_revoke_ts'], 'done_ts': p['last_done_ts'],
                        'revoke_minus_done_cycles': int(p['last_revoke_ts']) - int(p['last_done_ts']),
                        'timeout_cycles': p.get('timeout_cycles'),
                        'revoke_wall_ns': pl_to_wall(row, p['last_revoke_ts'], clk),
                        'done_wall_ns': pl_to_wall(row, p['last_done_ts'], clk),
                        'seen_wall_ns': row['wall_mid'], 'rtt_ns': row['mono_recv_ns'] - row['mono_send_ns']})
        if count is not None:
            last = count
    return out


def grants(rows, clk):
    """Permit 0 -> 1 transitions: the DONE that granted it (wall) and the poll that saw it."""
    out, prev = [], None
    for row in rows:
        bit = permit_bit(row)
        if prev is False and bit:
            out.append({'done_wall_ns': pl_to_wall(row, row['permit']['last_done_ts'], clk),
                        'seen_wall_ns': row['wall_mid'], 'frames_accepted': row['permit'].get('frames_accepted')})
        prev = bit
    return out


def permit_at(rows, wall_ns):
    """The permit bit in the last poll before wall_ns (None when there is none)."""
    before = [r for r in rows if r['wall_mid'] <= wall_ns]
    return permit_bit(before[-1]) if before else None


def mcu_view(rows):
    """Split mcu.jsonl rows: events (latch/unlock), executed-speed changes, KV permit changes, commands."""
    events = [r for r in rows if 'event' in r]
    executed = [r for r in rows if 'executed' in r]
    kv = [r for r in rows if 'kv_permit' in r]
    commands = [r for r in rows if 'last_cmd_seq' in r]
    start = next((r for r in rows if r.get('start')), None)
    return {'events': events, 'executed': executed, 'kv': kv, 'commands': commands, 'start': start}


def moving(v, w, eps=1e-9):
    return abs(v) + abs(w) > eps


def executed_at(executed, t_ms):
    """Executed (v, w) in force at MCU millisecond t_ms (last change at or before it)."""
    cur = (0.0, 0.0)
    for r in executed:
        if r['t_ms'] <= t_ms:
            cur = tuple(r['executed'])
        else:
            break
    return cur


def odom_rows(motion_rows):
    out = []
    for r in motion_rows:
        if r.get('topic') != '/odom':
            continue
        m = r['message']
        st = m['header']['stamp']
        tw = m['twist']['twist']
        out.append({'sim': st['sec'] + st['nanosec'] / 1e9, 'wall_ns': int(r['wall_ns']),
                    'speed': abs(tw['linear']['x']) + abs(tw['angular']['z']),
                    'v': tw['linear']['x'], 'w': tw['angular']['z']})
    return out


def first_still(odom, after_sim, n=5):
    """Sim stamp of the first of n consecutive still odom samples at or after after_sim."""
    run = 0
    first = None
    for r in odom:
        if r['sim'] < after_sim:
            continue
        if r['speed'] < STILL:
            run += 1
            if run == 1:
                first = r
            if run >= n:
                return first
        else:
            run = 0
    return None


def moving_after(odom, after_sim):
    """Odom samples at or after after_sim that are moving."""
    return [r for r in odom if r['sim'] >= after_sim and r['speed'] >= STILL]
