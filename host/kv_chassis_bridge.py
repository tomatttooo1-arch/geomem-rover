"""CHASSIS0 (2026-11-04): the chassis serial link on the KV260 -- velocity commands down, odometry up.

Design: runs/nationals-prelim-20261102/CHASSIS0/DESIGN.md. Until now the Nano's car stack owned the chassis port
(host/ros_chassis_driver.py -> /fpga/chassis/wheel_counts -> host/ros_wheel_odom.py -> /odom, and kv_nav's
(v, w) came back over the two-board link to the driver). With CHASSIS=kv the chassis USB serial is plugged into
the KV260 and kv_server runs this bridge in its own process, next to the NavService:

* down: every 1 / command_hz (50 Hz) the bridge hands the newest odometry to the NavService as a NAV_ODOM (the
  very message the Nano's proxy sent), takes the (v, w) of the NAV_CMD answer with the Nano proxy's rule
  (host.kv_nav_proxy.NavLink.command: valid, fresh, non-terminal -- else 0, 0), and builds the command frame
  with host.chassis_uart.ChassisLink -- the Nano driver's own policy object (50 Hz repeat, limits, enable only
  while the MCU is heard and the upstream is fresh, flag b2 for 0x82).
* up: bytes -> host.chassis_uart.FrameParser -> ChassisLink.on_status -> host.ros_chassis_driver.wheel_counts_row
  (the /fpga/chassis/wheel_counts body, JSON round trip as on the topic) -> host.ros_wheel_odom.OdomCore (the
  Nano odometry node's ROS-free core, same CLI args). The newest pose is ``pose()`` for kv_nav and the map.

No second implementation of the protocol or the odometry: every frame byte and every pose number comes from the
same functions the Nano ran (tests/test_kv_chassis_bridge.py compares them byte for byte / bit for bit).

Never refuses to start: a port that cannot be opened, a write / read error or a silent MCU -> one rate-limited
stderr warning, ``pose()`` returns None (callers fall back to their static pose), the port is retried every
``reopen_s``. The MCU's own rules stay the hard stop: 200 ms without a command frame -> stop, KV permit
heartbeat lost -> stop latched; the bridge cannot bypass them.
"""

import json
import os
import select
import sys
import threading
import time
from collections import Counter

from host.chassis_uart import BAUD, TYPE_STATUS, TYPE_STATUS2, ChassisLink, FrameParser, open_serial, write_all
from host.kv_link import (NAV_ODOM, NAV_ODOM_MAP_VALID, NAV_ODOM_ODOM_VALID, NAV_ODOM_PERMIT,
                          NAV_TERMINAL, BadExtension, decode_nav_cmd, encode_nav_odom)
from host.ros_chassis_driver import record_row, wheel_counts_row
from host.ros_wheel_odom import OdomCore, parse_args as wheel_odom_parse_args

CHASSIS_MODES = ("off", "kv", "nano")
COMMAND_HZ = 50.0
CMD_FRESH_S = 0.15        # = host.kv_nav_proxy.CMD_FRESH_S (NAV_CMD older than this -> drive 0, 0)
POSE_STALE_S = 0.3        # = ChassisLink link_lost_ns: no status frame for this long -> pose() is None
REOPEN_S = 2.0


def _warn(state, key, msg, every_s=5.0):
    now = time.monotonic()
    if now - state.get(key, -1e9) >= every_s:
        state[key] = now
        print(json.dumps({"warning": "kv_chassis: " + msg}), file=sys.stderr, flush=True)


def nav_cmd_velocity(cmd, cmd_t, now, cmd_fresh_s=CMD_FRESH_S):
    """(v, w, why) to drive from the newest decoded NAV_CMD -- host.kv_nav_proxy.NavLink.command's rule without
    the goal bookkeeping (the goal is set inside this process; the NAV_CMD carries the service's goal)."""
    if cmd is None or cmd_t is None or now - cmd_t > cmd_fresh_s:
        return 0.0, 0.0, "stale"
    if not cmd["accepted"] or cmd["state"] in NAV_TERMINAL:
        return 0.0, 0.0, "no_goal"
    if not cmd["valid"]:
        return 0.0, 0.0, "not_valid"
    return cmd["v"], cmd["w"], "ok"


class KvChassisBridge:
    """One serial port, two threads (reader, 50 Hz commander). ``on_bytes`` / ``command_tick`` are the whole
    logic and take explicit times so the host tests drive them deterministically."""

    def __init__(self, port, *, nav=None, baud=BAUD, command_hz=COMMAND_HZ, odom_args=(), status2=True,
                 v_limit_mm_s=500, w_limit_mrad_s=1500, legacy_left="a", legacy_right="b", record=None,
                 clock_ns=time.monotonic_ns, reopen_s=REOPEN_S, pose_stale_s=POSE_STALE_S):
        self.port, self.baud, self.nav = port, int(baud), nav
        self.period = 1.0 / float(command_hz)
        self.clock_ns = clock_ns
        self.reopen_s, self.pose_stale_ns = float(reopen_s), int(pose_stale_s * 1e9)
        self.legacy = (legacy_left, legacy_right)
        # the odometry node's own CLI: same defaults, same parameter checks (car_up: --heading gyro on the board)
        a, _ros, params, fused_params, _pv, _tv = wheel_odom_parse_args(["--source", "chassis", *odom_args])
        self.heading = a.heading
        self.record = open(record, "a") if record else None
        self.odom = OdomCore("chassis", params, None, heading=a.heading, fused_params=fused_params)
        self.link = ChassisLink(v_limit_mm_s=v_limit_mm_s, w_limit_mrad_s=w_limit_mrad_s, status2=status2)
        self.parser = FrameParser(accept={TYPE_STATUS, TYPE_STATUS2})
        self.lock = threading.Lock()
        self.counts = Counter()
        self._warned = {}
        self._pose = None            # (x, y, yaw, v, w, stamp_ns, t_received_ns)
        self._permit = False         # MCU status b0: it sees the KV permit heartbeat
        self._nav_cmd = None         # (decoded NAV_CMD, receive monotonic s)
        self._odom_seq = 0
        self.fd = None
        self._stop = threading.Event()
        self._threads = []

    # ---- up: status frames -> odometry
    def on_bytes(self, data, received_ns):
        """Raw serial bytes -> frames -> odometry. Returns the list of OdomCore states (None when skipped)."""
        out = []
        frames = self.parser.feed(data)
        for row in frames:
            with self.lock:
                derived = self.link.on_status(row, received_ns)
                command = self.link.last_command
            body = wheel_counts_row(row, derived, derived["stamp_ns"], command, *self.legacy)
            body = json.loads(json.dumps(body))           # exactly what the topic carried on the Nano
            st = self.odom.feed_chassis(body)
            out.append(st)
            with self.lock:
                self._permit = bool(row["heartbeat_seen"])
                if st is not None:
                    self._pose = (st["x"], st["y"], st["yaw"], st.get("v", 0.0), st.get("w", 0.0),
                                  int(st.get("ros_stamp_ns", received_ns)), int(received_ns))
            self.counts["frames"] += 1
            if self.record is not None:
                self.record.write(json.dumps(record_row(row, derived, *self.legacy)) + "\n")
        return out

    def ever_heard(self):
        """A status frame has produced a pose (from then on the static bench pose is never used again: a link
        that drops later keeps the last pose -- the MCU stops the car within 200 ms of the last command)."""
        with self.lock:
            return self._pose is not None

    def last_pose(self):
        """(x, y, yaw, v, w, stamp_ns) of the newest frame however old, or None (never heard)."""
        with self.lock:
            return None if self._pose is None else self._pose[:6]

    def pose(self, now_ns=None):
        """(x, y, yaw, v, w, stamp_ns) in the odometry frame, or None (never heard / stale > pose_stale_s)."""
        now_ns = self.clock_ns() if now_ns is None else now_ns
        with self.lock:
            p = self._pose
        if p is None or now_ns - p[6] > self.pose_stale_ns:
            return None
        return p[:6]

    # ---- down: NAV_ODOM -> NAV_CMD -> command frame
    def command_tick(self, now_ns):
        """One 50 Hz step. Returns (frame bytes, decision dict)."""
        v = w = 0.0
        why = "no_nav"
        if self.nav is not None:
            p = self.pose(now_ns)
            if p is not None:
                self._odom_seq += 1
                flags = NAV_ODOM_ODOM_VALID | NAV_ODOM_MAP_VALID | (NAV_ODOM_PERMIT if self._permit else 0)
                body = encode_nav_odom(seq=self._odom_seq, t_nano_ns=now_ns, odom_stamp_ns=max(0, p[5]),
                                       odom_pose=p[:3], v=p[3], w=p[4], map_odom=(0.0, 0.0, 0.0), flags=flags)
                try:
                    ans = self.nav.handle(NAV_ODOM, body)
                    self._nav_cmd = (decode_nav_cmd(ans), now_ns / 1e9) if ans else self._nav_cmd
                except (BadExtension, ValueError) as exc:
                    self.counts["nav_error:%s" % type(exc).__name__] += 1
            c = self._nav_cmd
            v, w, why = nav_cmd_velocity(None if c is None else c[0], None if c is None else c[1], now_ns / 1e9)
        with self.lock:
            if why == "ok":
                self.link.on_velocity(v, w, now_ns)
            else:
                # nothing to drive: enable 0 at once (the Nano chain sent enable 1 with v = w = 0 through the
                # velocity guard here; enable 0 is the stricter stop and needs no 200 ms upstream timeout)
                self.link.upstream = None
            frame, decision = self.link.next_command(now_ns)
        decision["nav"] = why
        self.counts["cmd:" + why] += 1
        return frame, decision

    def stop_frames(self, now_ns, n=3):
        """The Nano driver's shutdown: n frames with no upstream (enable 0)."""
        out = []
        for _ in range(n):
            with self.lock:
                self.link.upstream = None
                out.append(self.link.next_command(now_ns)[0])
        return out

    # ---- threads
    def _open(self):
        try:
            self.fd = open_serial(self.port, self.baud)
            self.counts["opened"] += 1
            return True
        except OSError as exc:
            self.counts["open_failed"] += 1
            _warn(self._warned, "open", "cannot open %s (%s); pose falls back to static, retry every %.0f s"
                  % (self.port, exc, self.reopen_s))
            return False

    def _drop_fd(self):
        fd, self.fd = self.fd, None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def _reader(self):
        while not self._stop.is_set():
            fd = self.fd
            if fd is None:
                self._stop.wait(0.05)
                continue
            try:
                if not select.select([fd], [], [], 0.05)[0]:
                    continue
                data = os.read(fd, 4096)
            except BlockingIOError:
                continue
            except (OSError, ValueError) as exc:
                self.counts["read_error"] += 1
                _warn(self._warned, "read", "read failed (%s); reopening" % (exc,))
                self._drop_fd()
                continue
            if not data:
                self._stop.wait(0.01)
                continue
            self.on_bytes(data, self.clock_ns())

    def _commander(self):
        nxt = time.monotonic()
        last_open = -1e9
        while not self._stop.is_set():
            if self.fd is None and time.monotonic() - last_open >= self.reopen_s:
                last_open = time.monotonic()
                self._open()
            now = self.clock_ns()
            frame, _ = self.command_tick(now)
            if self.fd is not None:
                try:
                    write_all(self.fd, frame)
                    self.counts["written"] += 1
                except (OSError, TimeoutError) as exc:
                    self.counts["write_error"] += 1
                    _warn(self._warned, "write", "write failed (%s); reopening" % (exc,))
                    self._drop_fd()
            if self.link.last_status_ns is not None and not self.link.link_alive(now):
                _warn(self._warned, "silent", "no status frame from the MCU for > 300 ms (enable held 0)", 10.0)
            nxt += self.period
            delay = nxt - time.monotonic()
            if delay < 0:
                self.counts["late"] += 1
                nxt, delay = time.monotonic(), 0
            self._stop.wait(delay)

    def start(self):
        self._open()
        for name, fn in (("kv-chassis-rx", self._reader), ("kv-chassis-tx", self._commander)):
            t = threading.Thread(target=fn, name=name, daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        if self.fd is not None:
            for f in self.stop_frames(self.clock_ns()):
                try:
                    write_all(self.fd, f)
                except (OSError, TimeoutError):
                    break
        self._drop_fd()
        if self.record is not None:
            self.record.close()

    def describe(self):
        p = self.pose()
        with self.lock:
            st = self.link.status(self.clock_ns())
        return {"port": self.port, "heading": self.heading, "open": self.fd is not None,
                "pose": None if p is None else [round(float(v), 4) for v in p[:5]],
                "counts": dict(self.counts), "parser": dict(self.parser.counters), "link": st,
                "odom": dict(self.odom.counters), "frames_by_type": dict(self.odom.frames_by_type)}
