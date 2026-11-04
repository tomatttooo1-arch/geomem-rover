#!/usr/bin/env python3
"""Keyboard / scripted client of host/ros_teleop_safe.py (2026-10-15). No ROS needed; Python 3.8 compatible.

  keys (default): a terminal; each key press sends ONE datagram to the teleop node's socket.
        w forward   s back   a turn left   d turn right   q forward+left   e forward+right
        space / x   stop (zero)            Ctrl-C / Esc   stop and quit
     Hold a key to keep driving: the terminal's key auto-repeat sends the datagrams. Release = no more datagrams ->
     the node's dead man stops the car 0.3 s after the last one. Caveat: the first auto-repeat comes 250-660 ms after
     the press (the operator's keyboard settings), longer than the 0.3 s dead man, so the start stutters once
     (drive 0.3 s, pause, then continuous). --hold-s H re-sends the last key for H s (default 0 = off) to smooth that;
     then "release to stop" becomes H + 0.3 s (at 0.15 m/s: 0.3 s = 4.5 cm; with --hold-s 0.4, 0.7 s = 10.5 cm).
  send V W --seconds S [--hz 20]: scripted input (tests, host runs): V m/s and W rad/s for S s, then nothing
     (the node's dead man stops it). Prints the send times (CLOCK_MONOTONIC_RAW ns, the clock of the node's record).

The node clamps to its own limits (0.15 m/s, 0.6 rad/s by default); --v / --w here only choose what a key asks for.
Socket: the run's IPC directory -- on the Nano host CAR_STATE_DIR/<tag>/teleop.sock (default: the car
CAR_STATE_DIR/car.current points to, its ipc.txt), inside the stack container /ipc/teleop.sock.
From a PC: ssh -t <nano> 'python3 <repo>/tools/car_run/teleop_keys.py'.
"""
import argparse
import json
import os
import select
import socket
import sys
import time

KEYS = {'w': (1, 0), 's': (-1, 0), 'a': (0, 1), 'd': (0, -1), 'q': (1, 1), 'e': (1, -1),
        ' ': (0, 0), 'x': (0, 0)}


def now_ns():
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)


def default_socket():
    if os.path.exists('/ipc/teleop.sock'):
        return '/ipc/teleop.sock'
    arch = os.uname().machine
    state = os.environ.get('CAR_STATE_DIR') or ('/data/fpga/car/state' if arch == 'aarch64' else '${FPGA_HOME}/dev1/.t/wde')
    run = os.path.realpath(os.path.join(state, 'car.current'))
    try:
        ipc = open(os.path.join(run, 'ipc.txt')).read().strip()
    except OSError:
        return None
    return os.path.join(ipc, 'teleop.sock')


class Sender:
    def __init__(self, path):
        self.path = path
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.errors = 0

    def send(self, v, w):
        try:
            self.sock.sendto(json.dumps({'v': v, 'w': w, 'sent_ns': now_ns()}).encode(), self.path)
            return True
        except OSError as error:
            self.errors += 1
            if self.errors in (1, 10, 100):
                sys.stderr.write('send to %s failed: %s\r\n' % (self.path, error))
            return False


def run_send(sender, v, w, seconds, hz):
    period = 1. / hz
    t_end = time.monotonic() + seconds
    first = last = None
    n = 0
    while time.monotonic() < t_end:
        t = now_ns()
        if sender.send(v, w):
            n += 1
            first = first if first is not None else t
            last = t
        time.sleep(period)
    print(json.dumps({'sent': n, 'v': v, 'w': w, 'first_ns': first, 'last_ns': last}))
    return 0 if n else 1


def run_keys(sender, v, w, hold_s):
    import termios
    import tty
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        sys.stderr.write('keys mode needs a terminal (ssh -t / docker exec -it); scripts use the send mode\n')
        return 2
    old = termios.tcgetattr(fd)
    sys.stdout.write('teleop: w/s forward/back  a/d turn  q/e arc  space stop  Ctrl-C/Esc quit  (socket %s)\r\n'
                     % sender.path)
    sys.stdout.flush()
    last_cmd, last_t = (0., 0.), 0.
    try:
        tty.setraw(fd)
        while True:
            ready = select.select([fd], [], [], 0.05)[0]
            if ready:
                ch = os.read(fd, 1).decode('latin-1')
                if ch in ('\x03', '\x1b', '\x04'):
                    break
                k = KEYS.get(ch.lower())
                if k is None:
                    continue
                last_cmd, last_t = (k[0] * v, k[1] * w), time.monotonic()
                sender.send(*last_cmd)
                sys.stdout.write('\rv=%+.2f w=%+.2f   ' % last_cmd)
                sys.stdout.flush()
            elif hold_s > 0 and last_cmd != (0., 0.) and time.monotonic() - last_t < hold_s:
                sender.send(*last_cmd)
    finally:
        for _ in range(3):
            sender.send(0., 0.)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        sys.stdout.write('\r\nteleop client: stopped (zero sent)\n')
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--socket', help='teleop socket (default: /ipc/teleop.sock in the container, else the running car)')
    p.add_argument('--v', type=float, default=0.15, help='m/s a key asks for (the node clamps)')
    p.add_argument('--w', type=float, default=0.6, help='rad/s a key asks for (the node clamps)')
    p.add_argument('--hold-s', type=float, default=0., help='re-send the last key for this long (0 = off, max 0.5)')
    sub = p.add_subparsers(dest='mode')
    sub.add_parser('keys')
    s = sub.add_parser('send')
    s.add_argument('sv', type=float)
    s.add_argument('sw', type=float)
    s.add_argument('--seconds', type=float, default=1.)
    s.add_argument('--hz', type=float, default=20.)
    a = p.parse_args(argv)
    if not 0. <= a.hold_s <= 0.5:
        p.error('--hold-s must be within [0, 0.5]')
    path = a.socket or default_socket()
    if not path:
        p.error('no --socket and no running car found (CAR_STATE_DIR/car.current)')
    sender = Sender(path)
    if a.mode == 'send':
        if not 1. <= a.hz <= 100.:
            p.error('--hz must be within [1, 100]')
        return run_send(sender, a.sv, a.sw, a.seconds, a.hz)
    return run_keys(sender, a.v, a.w, a.hold_s)


if __name__ == '__main__':
    sys.exit(main())
