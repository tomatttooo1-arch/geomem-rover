#!/usr/bin/env python3
"""fpgapc with the keyboard remote control (10-18): python3 pc/fpgapc_keys.py <fpgapc arguments> teleop --keys.

Same arguments as pc/fpgapc.py; it only supplies fpgapc.KEYS_LOOP, the raw-key reader that fpgapc.py itself must not
contain (fpgapc.py stays Windows-stdlib-only, tests/test_fpgac.py PortabilityTests). POSIX terminal: termios cbreak +
select; Windows console: msvcrt. w/s/a/d/q/e drive, space/x stop, Ctrl-C/Esc stop and quit; a held key auto-repeats,
the last key is re-sent for --hold-s to bridge the auto-repeat delay (tools/car_run/teleop_keys.py, same keys).
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fpgapc  # noqa: E402


def _keys_posix():
    import select
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def read(timeout):
        if select.select([fd], [], [], timeout)[0]:
            return os.read(fd, 1).decode("utf-8", "replace")
        return None

    def restore():
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return read, restore


def _keys_windows():
    import msvcrt

    def read(timeout):
        end = time.monotonic() + timeout
        while True:
            if msvcrt.kbhit():
                return msvcrt.getwch()
            if time.monotonic() >= end:
                return None
            time.sleep(0.01)
    return read, lambda: None


def keys_loop(send, check, args):
    read, restore = _keys_windows() if os.name == "nt" else _keys_posix()
    print("teleop via hub: w/s/a/d/q/e drive, space/x stop, Ctrl-C/Esc quit", file=sys.stderr)
    last, last_t = None, 0.0
    try:
        while True:
            ch = read(0.05)
            now = time.monotonic()
            if ch is not None:
                if ch in ("\x03", "\x1b"):
                    return 0
                k = fpgapc.TELEOP_KEYS.get(ch.lower())
                if k is None:
                    continue
                last, last_t = k, now
                body = {"stop": True} if k == (0, 0) else {"v": k[0] * args.v, "w": k[1] * args.w}
                if not check(send(body)):
                    return 1
            elif last is not None and last != (0, 0) and now - last_t <= args.hold_s:
                send({"v": last[0] * args.v, "w": last[1] * args.w})
    except KeyboardInterrupt:
        return 0
    finally:
        restore()


if __name__ == "__main__":
    fpgapc.KEYS_LOOP = keys_loop
    sys.exit(fpgapc.main())
