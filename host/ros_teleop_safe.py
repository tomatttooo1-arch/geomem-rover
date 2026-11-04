"""ROS node: python -m host.ros_teleop_safe --socket PATH [--v-max 0.15] [--w-max 0.6] (2026-10-15).

Low-speed remote control for the first real-car hour: the ONLY publisher of /fpga/cmd_vel_safe while it runs.

  input   Unix datagram socket --socket (car_inner.sh: $IPC/teleop.sock, directory mode 700, same uid; no network
          listener). One datagram = one JSON object {"v": m/s, "w": rad/s} (or {"stop": true}). The keyboard client is
          tools/car_run/teleop_keys.py (on the Nano, or from a PC through `ssh -t`); scripts use its `send` mode.
  output  /fpga/cmd_vel_safe (geometry_msgs/Twist) at --rate-hz (20) on a steady clock, ALWAYS (zero when idle, as the
          velocity guard does), so the chassis driver's upstream never goes stale while the node is healthy;
          /fpga/teleop/status (String JSON, 2 Hz): limits, counters, last reason, mode.

Safety rules (TeleopCore, tested without ROS):
  - speed limits |v| <= --v-max (default 0.15 m/s), |w| <= --w-max (default 0.6 rad/s); the limits themselves are
    capped at V_CAP 0.30 m/s and W_CAP 1.0 rad/s (larger values refused at start). Inputs beyond the limit are clamped,
    non-finite inputs become zero.
  - dead man: no datagram for --deadman-s (0.3 s; cap 0.5 s) -> zero. Releasing the key = no more datagrams.
  - one command at most --max-cmd-s (3 s; cap 10 s): a run of non-zero input without a release (zero input or
    dead-man expiry) longer than that -> zero, latched until the input is released.
  - exit (INT/TERM): zero commands for 0.5 s on a live ROS context, then shutdown; the chassis driver additionally
    sends enable=0 once its upstream is 200 ms old.
  - a second publisher on /fpga/cmd_vel_safe (the velocity guard, another teleop) -> zero, error, exit 3: car_inner's
    fail-stop then takes the stack down and the driver sends enable=0.

NOT gated by the motion permit: /fpga/motion_permit needs an accepted Nav2 goal (host/ros_runtime.py), so remote control
runs BESIDE the permit (parallel to the velocity guard, never through it). The MCU heartbeat latch (KV260 -> MCU,
docs/design/CHASSIS_UART_PROTOCOL_20261012.md) still stops the wheels independently of this node: it is the backstop,
and an e-stop from the PC (car_pc.sh estop) still latches the KV and thereby the MCU. A teleop run is not a navigation
run: car_inner.sh marks it (marks.txt "teleop", TELEOP file) and car.json says so.

Arbitration mode (2026-10-18, --nav-topic, car_up CAR_TELEOP=1 with --nav nav2; docs/design/TELEOP_NAV_ARB_20261018.md):
the velocity guard publishes on --nav-topic (/fpga/nav/cmd_vel_safe) instead of /fpga/cmd_vel_safe, and this node stays
the only /fpga/cmd_vel_safe publisher (the conflict check above is unchanged):
  - "in hand" = an accepted input datagram (any, a stop/zero one included) within the dead man; then the remote
    control's command is published exactly as without arbitration (limits, dead man, max duration);
  - the idle -> in-hand edge publishes ONE /fpga/cancel (the runner cancels its task and Nav2 goal, the permit drops);
  - not in hand: the guard's newest sample is passed on unchanged (the guard and Nav2 own the navigation speed
    limits; non-finite -> zero) while it is at most --nav-stale-s (0.2 s, the chassis driver's 200 ms staleness) old,
    else zero;
  - /fpga/teleop/active (std_msgs/Bool, every tick): in hand. The runner refuses NEW navigation (find/select/goto/
    patrol) with reason teleop_active while it is True and at most 0.5 s old (controller ruling 10-18: refused, not
    held back -- a car driving off on its own after the key is released is not acceptable).
Remote input over ROS (--input-topic, default /fpga/teleop/input, std_msgs/String with the datagram's JSON): the hub's
"teleop" op (controller PC only). Same rules as a socket datagram ("sent_ns" = the hub's host.fpga_clock, same machine).
Python 3.8 and 3.10 compatible.
"""
import argparse
import json
import math
import os
import select
import signal
import socket
import sys
import threading
import time

from host.fpga_clock import now_ns

TOPIC = '/fpga/cmd_vel_safe'
STATUS_TOPIC = '/fpga/teleop/status'
ACTIVE_TOPIC = '/fpga/teleop/active'
INPUT_TOPIC = '/fpga/teleop/input'
CANCEL_TOPIC = '/fpga/cancel'
NAV_STALE_S = 0.2                 # the chassis driver drops an upstream older than 200 ms
V_MAX, W_MAX = 0.15, 0.6          # defaults
V_CAP, W_CAP = 0.30, 1.0          # absolute caps of the limits
DEADMAN_S, DEADMAN_CAP = 0.3, 0.5
MAX_CMD_S, MAX_CMD_CAP = 3.0, 10.0
STOP_S = 0.5                      # zero commands on exit
MODE_NOTE = 'teleop: remote control, NOT a navigation run; not gated by /fpga/motion_permit; MCU heartbeat latch is the backstop'
ARB_NOTE = ('teleop + Nav2 arbitration: remote control in hand overrides (one /fpga/cancel on take-over), else the '
            'velocity guard output passes; new navigation refused while in hand; MCU heartbeat latch is the backstop')


def check_limits(v_max, w_max, deadman_s, max_cmd_s):
    """ValueError naming the first limit outside (0, cap]."""
    for name, value, cap in (('v_max', v_max, V_CAP), ('w_max', w_max, W_CAP), ('deadman_s', deadman_s, DEADMAN_CAP),
                             ('max_cmd_s', max_cmd_s, MAX_CMD_CAP)):
        if not (isinstance(value, (int, float)) and math.isfinite(value) and 0 < value <= cap):
            raise ValueError('%s=%r outside (0, %g]' % (name, value, cap))


def clamp(x, limit):
    return max(-limit, min(limit, x))


class TeleopCore:
    """Input datagrams in, the command to publish out. No ROS, no clock of its own (t_ns given)."""

    def __init__(self, v_max=V_MAX, w_max=W_MAX, deadman_s=DEADMAN_S, max_cmd_s=MAX_CMD_S):
        check_limits(v_max, w_max, deadman_s, max_cmd_s)
        self.v_max, self.w_max = float(v_max), float(w_max)
        self.deadman_ns, self.max_cmd_ns = int(deadman_s * 1e9), int(max_cmd_s * 1e9)
        self.last = None              # (v, w) after clamping
        self.last_ns = None
        self.episode_ns = None        # start of the current non-zero run
        self.latched = False          # max duration reached; cleared by a release
        self.reason = 'no_input'
        self.counters = {'inputs': 0, 'bad_input': 0, 'stale_input': 0, 'clamped': 0, 'non_finite': 0, 'deadman': 0,
                         'max_duration': 0, 'published': 0, 'published_nonzero': 0}
        self.arb = None               # arbitration state (enable_arbitration)

    def enable_arbitration(self, nav_stale_s=NAV_STALE_S):
        """Arbitration mode (module note): Nav2's guarded command passes while the remote control is not in hand."""
        if not (isinstance(nav_stale_s, (int, float)) and math.isfinite(nav_stale_s) and 0 < nav_stale_s <= 0.2):
            raise ValueError('nav_stale_s=%r outside (0, 0.2]' % (nav_stale_s,))
        self.arb = {'nav_stale_ns': int(nav_stale_s * 1e9), 'nav': None, 'nav_ns': None, 'in_hand': False,
                    'source': 'idle'}
        self.counters.update({'nav_samples': 0, 'nav_non_finite': 0, 'nav_passed': 0, 'nav_passed_nonzero': 0,
                              'nav_stale': 0, 'takeover': 0, 'handback': 0})

    def on_nav(self, v, w, t_ns):
        """One sample of the guard's output (arbitration mode)."""
        self.counters['nav_samples'] += 1
        v, w = float(v), float(w)
        if not (math.isfinite(v) and math.isfinite(w)):
            self.counters['nav_non_finite'] += 1
            v = w = 0.
        self.arb['nav'], self.arb['nav_ns'] = (v, w), int(t_ns)

    def in_hand(self, t_ns):
        return self.last_ns is not None and t_ns - self.last_ns <= self.deadman_ns

    def arbitrate(self, t_ns):
        """(v, w, reason, source, takeover) to publish at t_ns in arbitration mode. source: teleop | nav | idle.
        takeover: True on the idle -> in-hand edge (the caller publishes one /fpga/cancel)."""
        a = self.arb
        hand = self.in_hand(t_ns)
        v, w, reason = self.command(t_ns)        # the remote control's own command (and its dead-man bookkeeping)
        takeover = hand and not a['in_hand']
        if takeover:
            self.counters['takeover'] += 1
        elif a['in_hand'] and not hand:
            self.counters['handback'] += 1
        a['in_hand'] = hand
        if hand:
            source = 'teleop'
        elif a['nav_ns'] is not None and 0 <= t_ns - a['nav_ns'] <= a['nav_stale_ns']:
            (v, w), reason, source = a['nav'], 'nav', 'nav'
            self.counters['nav_passed'] += 1
            if v or w:
                self.counters['nav_passed_nonzero'] += 1
        else:
            v, w, source = 0., 0., 'idle'
            reason = 'nav_none' if a['nav_ns'] is None else 'nav_stale'
            if a['nav_ns'] is not None:
                self.counters['nav_stale'] += 1
        a['source'] = source
        return v, w, reason, source, takeover

    def _release(self):
        self.episode_ns = None
        self.latched = False

    def on_datagram(self, data, t_ns):
        """One datagram (bytes). A malformed one is counted and ignored: it does NOT refresh the dead man.
        "sent_ns" (optional; teleop_keys.py sends CLOCK_MONOTONIC_RAW, the clock of the Nano and its containers): a
        datagram older than the dead man on arrival (queued while this node or the board stalled) is ignored, so a
        stall cannot replay a released key."""
        try:
            row = json.loads(data.decode('utf-8') if isinstance(data, bytes) else data)
            if not isinstance(row, dict):
                raise ValueError('not an object')
            if row.get('stop'):
                v = w = 0.
            else:
                v, w = float(row['v']), float(row['w'])
            sent = row.get('sent_ns')
            sent = None if sent is None else int(sent)
        except (ValueError, KeyError, TypeError, UnicodeDecodeError, OverflowError):
            self.counters['bad_input'] += 1
            return None
        if sent is not None and t_ns - sent > self.deadman_ns and (v or w):
            self.counters['stale_input'] += 1
            return None
        return self.on_input(v, w, t_ns)

    def on_input(self, v, w, t_ns):
        self.counters['inputs'] += 1
        if not (math.isfinite(v) and math.isfinite(w)):
            self.counters['non_finite'] += 1
            v = w = 0.
        cv, cw = clamp(v, self.v_max), clamp(w, self.w_max)
        if (cv, cw) != (v, w):
            self.counters['clamped'] += 1
        if self.last_ns is not None and t_ns - self.last_ns > self.deadman_ns:
            self._release()                       # a gap longer than the dead man is a release
        if (cv, cw) == (0., 0.):
            self._release()
        elif self.episode_ns is None:
            self.episode_ns = int(t_ns)           # a non-zero run starts at its first datagram
        self.last, self.last_ns = (cv, cw), int(t_ns)
        return self.last

    def command(self, t_ns):
        """(v, w, reason) to publish at t_ns. reason: drive | no_input | deadman | zero_input | max_duration."""
        if self.last_ns is None:
            v, w, self.reason = 0., 0., 'no_input'
        elif t_ns - self.last_ns > self.deadman_ns:
            if self.reason == 'drive' or self.latched:
                self.counters['deadman'] += 1
            self._release()
            v, w, self.reason = 0., 0., 'deadman'
        elif self.last == (0., 0.):
            self._release()
            v, w, self.reason = 0., 0., 'zero_input'
        else:
            if not self.latched and t_ns - self.episode_ns > self.max_cmd_ns:
                self.latched = True
                self.counters['max_duration'] += 1
            if self.latched:
                v, w, self.reason = 0., 0., 'max_duration'
            else:
                (v, w), self.reason = self.last, 'drive'
        self.counters['published'] += 1
        if v or w:
            self.counters['published_nonzero'] += 1
        return v, w, self.reason

    def status(self):
        row = {'mode': 'teleop', 'note': MODE_NOTE, 'v_max': self.v_max, 'w_max': self.w_max,
               'deadman_s': self.deadman_ns / 1e9, 'max_cmd_s': self.max_cmd_ns / 1e9,
               'last_reason': self.reason, 'latched': self.latched, 'counters': dict(self.counters)}
        if self.arb is not None:
            row.update({'mode': 'teleop_nav_arbitration', 'note': ARB_NOTE, 'in_hand': self.arb['in_hand'],
                        'source': self.arb['source'], 'nav_stale_s': self.arb['nav_stale_ns'] / 1e9})
        return row


def open_socket(path):
    """Unix datagram socket at path (a stale file is removed), mode 600."""
    try:
        os.unlink(path)
    except FileNotFoundError:
        pass
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    s.bind(path)
    os.chmod(path, 0o600)
    return s


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--socket', required=True, help='Unix datagram socket path (car_inner: $IPC/teleop.sock)')
    parser.add_argument('--v-max', type=float, default=V_MAX, help='m/s, cap %g' % V_CAP)
    parser.add_argument('--w-max', type=float, default=W_MAX, help='rad/s, cap %g' % W_CAP)
    parser.add_argument('--deadman-s', type=float, default=DEADMAN_S, help='cap %g' % DEADMAN_CAP)
    parser.add_argument('--max-cmd-s', type=float, default=MAX_CMD_S, help='cap %g' % MAX_CMD_CAP)
    parser.add_argument('--rate-hz', type=float, default=20.)
    parser.add_argument('--topic', default=TOPIC)
    parser.add_argument('--conflict-grace-s', type=float, default=2.,
                        help='after start, check every 1 s for another publisher on --topic')
    parser.add_argument('--record', help='append inputs and published commands as JSON lines')
    parser.add_argument('--nav-topic', default='',
                        help='arbitration mode: the velocity guard output (car_inner: /fpga/nav/cmd_vel_safe); empty = off')
    parser.add_argument('--nav-stale-s', type=float, default=NAV_STALE_S, help='cap 0.2 (the chassis driver staleness)')
    parser.add_argument('--input-topic', default=INPUT_TOPIC,
                        help='std_msgs/String datagrams over ROS (the hub teleop op); empty = socket only')
    args, ros_args = parser.parse_known_args(argv)
    try:
        check_limits(args.v_max, args.w_max, args.deadman_s, args.max_cmd_s)
    except ValueError as error:
        parser.error(str(error))
    if args.nav_topic:
        try:
            TeleopCore().enable_arbitration(args.nav_stale_s)
        except ValueError as error:
            parser.error(str(error))
        if args.nav_topic == args.topic:
            parser.error('--nav-topic must differ from --topic')
    if not 5. <= args.rate_hz <= 100.:
        parser.error('--rate-hz must be within [5, 100] (the chassis driver drops an upstream older than 200 ms)')

    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, Empty, String

    class Conflict(Exception):
        pass

    class TeleopNode(Node):
        def __init__(self, sock):
            super().__init__('fpga_teleop_safe')
            self.sock = sock
            self.lock = threading.Lock()
            self.core = TeleopCore(args.v_max, args.w_max, args.deadman_s, args.max_cmd_s)
            self.record = open(args.record, 'a', buffering=1) if args.record else None
            self.pub = self.create_publisher(Twist, args.topic, 10)
            self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
            self.arb = bool(args.nav_topic)
            if self.arb:
                self.core.enable_arbitration(args.nav_stale_s)
                self.active_pub = self.create_publisher(Bool, ACTIVE_TOPIC, 10)
                self.cancel_pub = self.create_publisher(Empty, CANCEL_TOPIC, 10)
                self.create_subscription(Twist, args.nav_topic, self.on_nav, 10)
            if args.input_topic:
                self.create_subscription(String, args.input_topic, self.on_ros_input, 10)
            self.started = time.monotonic()
            self.conflict = None
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(1. / args.rate_hz, self.tick, clock=steady)
            self.create_timer(0.5, self.publish_status, clock=steady)
            self.create_timer(1., self.check_conflict, clock=steady)
            self.running = True
            self.reader = threading.Thread(target=self.read_loop, name='teleop-rx', daemon=True)
            self.reader.start()
            log = self.get_logger()
            log.info('teleop on %s: |v|<=%.2f m/s |w|<=%.2f rad/s deadman %.2f s max command %.1f s socket %s'
                     % (args.topic, args.v_max, args.w_max, args.deadman_s, args.max_cmd_s, args.socket))
            log.warn(ARB_NOTE if self.arb else MODE_NOTE)
            if self.arb:
                log.info('arbitration: nav input %s (stale after %.2f s), in-hand flag %s' % (
                    args.nav_topic, args.nav_stale_s, ACTIVE_TOPIC))

        def log_row(self, row):
            if self.record is not None:
                self.record.write(json.dumps(row, allow_nan=False) + '\n')

        def read_loop(self):
            while self.running:
                try:
                    if not select.select([self.sock], [], [], 0.05)[0]:
                        continue
                    data = self.sock.recv(4096)
                except OSError:
                    time.sleep(0.05)
                    continue
                t = now_ns()
                with self.lock:
                    got = self.core.on_datagram(data, t)
                self.log_row({'dir': 'in', 't_ns': t, 'ok': got is not None,
                              'v': None if got is None else got[0], 'w': None if got is None else got[1]})

        def on_nav(self, msg):
            t = now_ns()
            with self.lock:
                self.core.on_nav(msg.linear.x, msg.angular.z, t)

        def on_ros_input(self, msg):
            t = now_ns()
            with self.lock:
                got = self.core.on_datagram(msg.data, t)
            self.log_row({'dir': 'in', 'via': 'ros', 't_ns': t, 'ok': got is not None,
                          'v': None if got is None else got[0], 'w': None if got is None else got[1]})

        def send(self, v, w):
            msg = Twist()
            msg.linear.x, msg.angular.z = float(v), float(w)
            self.pub.publish(msg)

        def tick(self):
            if self.conflict is not None:
                raise Conflict(self.conflict)
            t = now_ns()
            if not self.arb:
                with self.lock:
                    v, w, reason = self.core.command(t)
                self.send(v, w)
                self.log_row({'dir': 'out', 't_ns': t, 'v': v, 'w': w, 'reason': reason})
                return
            with self.lock:
                v, w, reason, source, takeover = self.core.arbitrate(t)
            if takeover:      # before the command: the runner cancels the task and the Nav2 goal, the permit drops
                self.cancel_pub.publish(Empty())
                self.get_logger().warn('remote control in hand: /fpga/cancel published (navigation cancelled)')
            self.send(v, w)
            self.active_pub.publish(Bool(data=source == 'teleop'))
            self.log_row({'dir': 'out', 't_ns': t, 'v': v, 'w': w, 'reason': reason, 'src': source,
                          'takeover': takeover})

        def check_conflict(self):
            if time.monotonic() - self.started < args.conflict_grace_s:
                return
            others = self.count_publishers(args.topic) - 1
            if others > 0:
                self.conflict = '%d other publisher(s) on %s: teleop and the velocity guard / another teleop are ' \
                                'mutually exclusive' % (others, args.topic)
                self.get_logger().error(self.conflict)
                raise Conflict(self.conflict)

        def publish_status(self):
            with self.lock:
                row = self.core.status()
            row['socket'] = args.socket
            self.status_pub.publish(String(data=json.dumps(row, allow_nan=False)))

        def stop(self):
            self.running = False
            self.reader.join(timeout=1.)
            n = max(1, int(round(STOP_S * args.rate_hz)))
            for _ in range(n):
                try:
                    self.send(0., 0.)
                except Exception:      # context already gone: the driver's 200 ms staleness takes over
                    break
                self.log_row({'dir': 'out', 't_ns': now_ns(), 'v': 0., 'w': 0., 'reason': 'exit'})
                time.sleep(1. / args.rate_hz)
            if self.record is not None:
                self.record.close()

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    sock = open_socket(args.socket)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupt)
    node, rc = None, 0
    try:
        node = TeleopNode(sock)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Conflict as error:
        sys.stderr.write('teleop: %s\n' % error)
        rc = 3
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        sock.close()
        try:
            os.unlink(args.socket)
        except OSError:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
