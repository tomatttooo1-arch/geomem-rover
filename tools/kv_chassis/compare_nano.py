#!/usr/bin/env python3
"""CHASSIS0 key comparison (host only, the car does not move): KV bridge vs the Nano's existing chassis code.

Input: one real-car recording (the Nano driver's --record chassis_record.jsonl, 0x82 frames, plus the Nano odometry
node's --record wheel_odom.jsonl of the same run). Both checks replay the same events into both sides.

A. command bytes. Nano side = host/ros_chassis_driver.py main() itself (the ChassisDriver ROS node class, run
   with a stub rclpy on this host), serial = a pty; the script writes each recorded status frame into the pty at
   its recorded time (fake monotonic clock) and waits until the node's reader thread took it, and at every
   recorded command tick hands the node the recorded (v, w) and calls its 50 Hz send_command, reading the bytes
   it wrote. KV side = host.kv_chassis_bridge.KvChassisBridge: the same frames into on_bytes, and at every tick
   command_tick with a stub NavService answering a VALID NAV_CMD carrying the same (v, w). Pass = every command
   frame byte-identical, plus the 3 shutdown frames.
   Negative control: a bridge with status2 off (flag b2 not set) must differ.
B. odometry. The recorded 0x82 frames (re-encoded with host.chassis_uart.encode_status2; the re-encoding is
   checked by decoding it back to the recorded fields) into the bridge's on_bytes at their recorded receive time;
   every resulting odometry state is compared, by frame key (seq, mcu_ms), with the Nano odometry node's own
   recorded row: x, y, yaw, distance, v, w (and in gyro mode the shadow gyro_x/y/yaw) -- exact equality.
   Negative control: a bridge with a 1 % different wheel diameter must differ.

Usage: compare_nano.py RUN_DIR --out RESULT.json [--max-events N] [--odom-args "..."]
"""

import argparse
import json
import os
import pty
import select
import sys
import threading
import time
import tty
import types

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from host import ros_chassis_driver as drv                                   # noqa: E402
from host.chassis_uart import STATUS2_FIELDS, decode_frame, encode_status2   # noqa: E402
from host.kv_chassis_bridge import KvChassisBridge                           # noqa: E402
from host.kv_link import NAV_CMD_ACCEPTED, NAV_CMD_VALID, NAV_STATE, encode_nav_cmd  # noqa: E402

FAKE_NS = [0]


def fake_now():
    return FAKE_NS[0]


# ------------------------------------------------------------------ stub rclpy (only what the driver touches)
class _Pub:
    def __init__(self, sink, topic):
        self.sink, self.topic = sink, topic

    def publish(self, msg):
        self.sink.append((self.topic, getattr(msg, "data", None)))


class _Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __getattr__(self, name):            # nested message fields: header.stamp.sec, ...
        v = _Obj()
        object.__setattr__(self, name, v)
        return v

    def __setattr__(self, name, value):
        object.__setattr__(self, name, value)

    def to_msg(self):
        return _Obj()


def _msg(**kw):
    o = _Obj()
    o.__dict__.update(kw)
    return o


class _Imu(_Obj):
    def __init__(self):
        object.__setattr__(self, "orientation_covariance", [0.0] * 9)
        object.__setattr__(self, "linear_acceleration_covariance", [0.0] * 9)
        object.__setattr__(self, "angular_velocity_covariance", [0.0] * 9)


class _Clock:
    def __init__(self, *a, **kw):
        pass

    def now(self):
        return _Obj(nanoseconds=FAKE_NS[0])


STATE = {"node": None, "published": [], "script": None}


class _Node:
    def __init__(self, name):
        STATE["node"] = self

    def create_publisher(self, typ, topic, qos):
        return _Pub(STATE["published"], topic)

    def create_subscription(self, *a, **kw):
        return None

    def create_service(self, *a, **kw):
        return None

    def create_timer(self, *a, **kw):
        return None

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return _Obj(info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None)

    def destroy_node(self):
        pass


def install_stub_rclpy():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    def spin(node):
        STATE["script"](node)
        raise KeyboardInterrupt

    mod("rclpy", init=lambda **kw: None, spin=spin, ok=lambda: True, shutdown=lambda: None)
    mod("rclpy.node", Node=_Node)
    mod("rclpy.clock", Clock=_Clock, ClockType=_Obj(STEADY_TIME=1))
    mod("rclpy.qos", qos_profile_sensor_data=None)
    mod("rclpy.signals", SignalHandlerOptions=_Obj(NO=0))
    mod("geometry_msgs"), mod("geometry_msgs.msg", TransformStamped=_Obj, Twist=_Obj)
    mod("sensor_msgs"), mod("sensor_msgs.msg", BatteryState=_Obj, Imu=_Imu)
    mod("std_msgs"), mod("std_msgs.msg", String=lambda data=None: _msg(data=data))
    mod("std_srvs"), mod("std_srvs.srv", Trigger=_Obj)
    mod("tf2_ros"), mod("tf2_ros.static_transform_broadcaster",
                        StaticTransformBroadcaster=lambda node: _Obj(sendTransform=lambda t: None))


# ------------------------------------------------------------------ recording -> events
def load_events(run_dir, max_events):
    ev = []
    with open(os.path.join(run_dir, "chassis_record.jsonl")) as fp:
        for line in fp:
            r = json.loads(line)
            if r.get("dir") == "rx" and r.get("frame") == "0x82":
                fields = {k: r[k] for k in STATUS2_FIELDS if k != "temp_cdeg"}
                fields["temp_cdeg"] = -32768 if r["temp_cdeg"] is None else r["temp_cdeg"]
                frame = encode_status2(**fields)
                ev.append(("rx", r["t_ns"], frame, r))
            elif r.get("dir") == "tx":
                ev.append(("tx", r["t_ns"], (r["v_mm_s"] / 1000.0, r["w_mrad_s"] / 1000.0), r))
            if max_events and len(ev) >= max_events:
                break
    return ev


def check_reencode(events):
    bad = 0
    for kind, _t, frame, r in events:
        if kind != "rx":
            continue
        d = decode_frame(frame)
        if any(d[k] != r[k] for k in STATUS2_FIELDS if k != "temp_cdeg"):
            bad += 1
    return bad


class StubNav:
    """NavService stand-in: answers every NAV_ODOM with a VALID NAV_CMD carrying the scripted (v, w)."""

    def __init__(self):
        self.vw = (0.0, 0.0)
        self.seq = 0

    def handle(self, mtype, body):
        self.seq += 1
        return encode_nav_cmd(state=NAV_STATE["track"], reason=0, flags=NAV_CMD_ACCEPTED | NAV_CMD_VALID,
                              goal_id=1, cmd_seq=self.seq, odom_seq=0, kv_mono_ns=FAKE_NS[0], cmd_age_us=0,
                              v=self.vw[0], w=self.vw[1], remaining_m=1.0, path_len_m=1.0, recoveries=0,
                              path_points=2)


# ------------------------------------------------------------------ A. command bytes
def nano_commands(events, tmp):
    """Run the Nano driver main() on a pty; returns the list of command frames it wrote (+ 3 stop frames)."""
    install_stub_rclpy()
    drv.now_ns = fake_now                     # the driver's monotonic clock (imported name in its module)
    master, slave = pty.openpty()
    tty.setraw(master)
    slave_path = os.ttyname(slave)
    out = []
    rbuf = bytearray()

    def read_exact(n, timeout=2.0):
        end = time.monotonic() + timeout
        while len(rbuf) < n:
            if not select.select([master], [], [], max(0.0, end - time.monotonic()))[0]:
                raise TimeoutError("driver wrote nothing")
            rbuf.extend(os.read(master, 4096))
        data = bytes(rbuf[:n])
        del rbuf[:n]
        return data

    def script(node):
        for kind, t, payload, _r in events:
            if kind == "rx":
                FAKE_NS[0] = t
                before = node.link.counters["status_frames"]
                os.write(master, payload)
                end = time.monotonic() + 2.0
                while node.link.counters["status_frames"] == before:
                    if time.monotonic() > end:
                        raise TimeoutError("driver reader did not take the frame")
                    time.sleep(0.0002)
            else:
                FAKE_NS[0] = t
                node.velocity(_msg(linear=_msg(x=payload[0]), angular=_msg(z=payload[1])))
                node.send_command()
                out.append(read_exact(11))

    STATE["script"] = script
    drv.main(["--port", slave_path, "--record", os.path.join(tmp, "nano_driver_record.jsonl")])
    for _ in range(3):
        out.append(read_exact(11))
    os.close(master)
    return out


def bridge_commands(events, **kw):
    nav = StubNav()
    b = KvChassisBridge("/nonexistent", nav=nav, clock_ns=fake_now, **kw)
    out = []
    for kind, t, payload, _r in events:
        FAKE_NS[0] = t
        if kind == "rx":
            b.on_bytes(payload, t)
        else:
            nav.vw = payload
            out.append(b.command_tick(t)[0])
    out.extend(b.stop_frames(FAKE_NS[0]))
    return out, b


def compare_frames(a, b):
    n = min(len(a), len(b))
    diff = [i for i in range(n) if a[i] != b[i]]
    return {"nano_frames": len(a), "bridge_frames": len(b), "equal_frames": n - len(diff), "differ": len(diff),
            "first_diff": None if not diff else {"i": diff[0], "nano": a[diff[0]].hex(), "bridge": b[diff[0]].hex()},
            "identical": len(a) == len(b) and not diff}


# ------------------------------------------------------------------ B. odometry
ODOM_KEYS = ("x", "y", "yaw", "distance", "v", "w", "event")
GYRO_KEYS = ("gyro_x", "gyro_y", "gyro_yaw", "gyro_event")


def first_odom_key(run_dir):
    with open(os.path.join(run_dir, "wheel_odom.jsonl")) as fp:
        return tuple(json.loads(fp.readline())["key"])


def bridge_odom(events, odom_args, start_key=None, link_lost_ms=None):
    b = KvChassisBridge("/nonexistent", nav=None, clock_ns=fake_now, odom_args=odom_args)
    if link_lost_ms is not None:   # the recording's code version (head.txt) decides when a frame is "after silence"
        b.link.link_lost_ns = int(link_lost_ms * 1e6)
    rows = {}

    class Tap:
        def write(self, line):
            r = json.loads(line)
            rows[tuple(r["key"])] = r
    b.odom.record = Tap()          # OdomCore's own --record row (what the Nano node wrote)
    started = start_key is None    # the Nano odometry node starts after the driver: begin at its first frame
    for kind, t, payload, r in events:
        if kind == "rx":
            started = started or (r["seq"], r["mcu_ms"]) == start_key
            if not started:
                continue
            FAKE_NS[0] = t
            b.on_bytes(payload, t)
    return rows, b


def compare_odom(run_dir, mine, keys):
    n = same = missing = 0
    first_diff = None
    maxabs = {k: 0.0 for k in keys if k not in ("event", "gyro_event")}
    with open(os.path.join(run_dir, "wheel_odom.jsonl")) as fp:
        for line in fp:
            r = json.loads(line)
            k = tuple(r["key"])
            m = mine.get(k)
            if m is None:
                missing += 1
                continue
            n += 1
            ok = all(m.get(f) == r.get(f) for f in keys)
            same += ok
            for f in maxabs:
                if isinstance(m.get(f), float) and isinstance(r.get(f), float):
                    maxabs[f] = max(maxabs[f], abs(m[f] - r[f]))
            if not ok and first_diff is None:
                first_diff = {"key": list(k), "nano": {f: r.get(f) for f in keys}, "bridge": {f: m.get(f) for f in keys}}
    return {"compared": n, "identical": same, "nano_rows_without_bridge_row": missing, "max_abs_diff": maxabs,
            "first_diff": first_diff, "all_identical": n > 0 and same == n}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-events", type=int, default=0, help="0 = whole recording")
    ap.add_argument("--a-events", type=int, default=12000,
                    help="A (commands, slow: real pty round trips) runs on this many events starting --a-lead "
                         "events before the first moving command; 0 = all")
    ap.add_argument("--a-lead", type=int, default=1000)
    ap.add_argument("--rec-link-lost-ms", type=float, default=100.0,
                    help="B: ChassisLink link-lost of the code that made the recording (accept-20261026: head c274f1f, "
                         "100 ms; 556f9ce raised it to 300 ms afterwards). The bridge's own default stays 300 ms.")
    ap.add_argument("--odom-args", default="--heading wheel --wheel-diam-m 0.081 --counts-per-rev 1560 --track-m 0.394")
    ap.add_argument("--neg-odom-args", default="--heading wheel --wheel-diam-m 0.08181 --counts-per-rev 1560 --track-m 0.394")
    a = ap.parse_args()
    tmp = os.path.dirname(os.path.abspath(a.out))
    t0 = time.time()
    ev = load_events(a.run_dir, a.max_events)
    res = {"run_dir": a.run_dir, "events": len(ev), "rx": sum(e[0] == "rx" for e in ev),
           "tx": sum(e[0] == "tx" for e in ev),
           "tx_enabled_in_recording": sum(e[0] == "tx" and e[3]["enable"] for e in ev),
           "reencode_mismatch": check_reencode(ev)}
    ev_a = ev
    if a.a_events:
        first_move = next(i for i, e in enumerate(ev) if e[0] == "tx" and e[3]["enable"] and e[3]["v_mm_s"])
        lo = max(0, first_move - a.a_lead)
        ev_a = ev[lo:lo + a.a_events]
        res["A_window"] = {"first_event": lo, "events": len(ev_a), "t0_ns": ev_a[0][1], "t1_ns": ev_a[-1][1],
                           "tx_moving": sum(bool(e[0] == "tx" and e[3]["enable"] and (e[3]["v_mm_s"] or e[3]["w_mrad_s"]))
                                            for e in ev_a)}
    nano = nano_commands(ev_a, tmp)
    mine, b = bridge_commands(ev_a)
    res["A_commands"] = compare_frames(nano, mine)
    res["A_commands"]["enabled_frames_bridge"] = sum(bool(f[8] & 1) for f in mine)
    neg, _ = bridge_commands(ev_a, status2=False)
    res["A_negative_control_status2_off"] = compare_frames(nano, neg)
    odom_args = a.odom_args.split()
    k0 = first_odom_key(a.run_dir)
    res["B_start_key"] = list(k0)
    rows, bo = bridge_odom(ev, odom_args, k0, a.rec_link_lost_ms)
    keys = ODOM_KEYS + (GYRO_KEYS if "gyro" in a.odom_args else ())
    res["B_odom"] = compare_odom(a.run_dir, rows, keys)
    res["B_odom"]["args"] = a.odom_args
    res["B_odom"]["bridge_last_pose"] = bo.last_pose()
    nrows, _ = bridge_odom(ev, a.neg_odom_args.split(), k0, a.rec_link_lost_ms)
    res["B_negative_control"] = compare_odom(a.run_dir, nrows, keys)
    res["B_negative_control"]["args"] = a.neg_odom_args
    res["B_odom"]["link_lost_ms"] = a.rec_link_lost_ms
    crows, _ = bridge_odom(ev, odom_args, k0, None)     # informational: the bridge as shipped (300 ms)
    res["B_info_bridge_default_link_lost"] = compare_odom(a.run_dir, crows, keys)
    res["wall_s"] = round(time.time() - t0, 1)
    res["pass"] = (res["reencode_mismatch"] == 0 and res["A_commands"]["identical"]
                   and not res["A_negative_control_status2_off"]["identical"] and res["B_odom"]["all_identical"]
                   and not res["B_negative_control"]["all_identical"])
    with open(a.out, "w") as fp:
        json.dump(res, fp, indent=1, default=str)
    print(json.dumps({k: res[k] for k in ("events", "pass", "wall_s")}))
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
