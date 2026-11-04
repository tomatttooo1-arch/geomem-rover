#!/usr/bin/env python3
"""Colour-rate dig instrument (10-16). Runs INSIDE the camera container (docker exec), so it shares the driver's
pid namespace: reads /proc/<component_container>/stat itself.

One call = optional actions (AE on/off, exposure set), warm-up, one measurement window:
  - /camera/color/camera_info and /camera/depth/camera_info header stamps -> Hz from stamp INTERVALS
    ((n-1) / sum of intervals), interval histogram in frame periods (round(dt / 33.33 ms)): 1, 2, 3, 4+
  - driver CPU (utime+stime delta / wall, % of one core) and the top threads, whole-board CPU from /proc/stat
  - get_color_exposure / get_color_gain at window start and end
  - one colour image at window start and end -> mean of the rgb8 bytes (brightness proxy, 0-255)
Writes one JSON (--out) and prints a one-line summary.
"""
import argparse, json, os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool
from orbbec_camera_msgs.srv import GetInt32, SetInt32

PERIOD = 1.0 / 30.0
TCK = os.sysconf("SC_CLK_TCK")


def find_driver_pid():
    for p in os.listdir("/proc"):
        if not p.isdigit():
            continue
        try:
            if open(f"/proc/{p}/comm").read().strip().startswith("component_conta"):
                return int(p)
        except OSError:
            pass
    return None


def proc_ticks(pid):
    f = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
    return int(f[11]) + int(f[12])          # utime + stime (fields 14, 15)


def thread_ticks(pid):
    out = {}
    for t in os.listdir(f"/proc/{pid}/task"):
        try:
            s = open(f"/proc/{pid}/task/{t}/stat").read()
            comm = s[s.index("(") + 1:s.rindex(")")]
            f = s.rsplit(")", 1)[1].split()
            out[t] = (comm, int(f[11]) + int(f[12]))
        except (OSError, ValueError):
            pass
    return out


def board_cpu():
    f = [int(x) for x in open("/proc/stat").readline().split()[1:]]
    idle = f[3] + f[4]
    return sum(f), idle


class Probe(Node):
    def __init__(self, ns):
        super().__init__("crdig_probe")
        self.ns = ns
        self.stamps = {"color": [], "depth": []}
        self.recv = {"color": [], "depth": []}
        self.on = False
        for k in ("color", "depth"):
            self.create_subscription(CameraInfo, f"{ns}/{k}/camera_info", lambda m, k=k: self._info(k, m),
                                     qos_profile_sensor_data)
        self.cli = {n: self.create_client(t, f"{ns}/{n}") for n, t in (
            ("get_color_exposure", GetInt32), ("get_color_gain", GetInt32),
            ("set_color_exposure", SetInt32), ("set_color_auto_exposure", SetBool))}

    def _info(self, k, m):
        if self.on:
            self.stamps[k].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            self.recv[k].append(time.monotonic())

    def call(self, name, data=None, timeout=5.0):
        c = self.cli[name]
        if not c.wait_for_service(timeout_sec=timeout):
            return {"ok": False, "msg": "no service"}
        req = c.srv_type.Request()
        if data is not None:
            req.data = data
        fut = c.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        r = fut.result()
        if r is None:
            return {"ok": False, "msg": "timeout"}
        d = {"ok": bool(r.success), "msg": r.message}
        if hasattr(r, "data"):
            d["data"] = int(r.data)
        return d

    def exposure(self):
        return {"exposure": self.call("get_color_exposure"), "gain": self.call("get_color_gain")}

    def brightness(self, timeout=3.0):
        got = []
        sub = self.create_subscription(Image, f"{self.ns}/color/image_raw", lambda m: got.append(m),
                                       qos_profile_sensor_data)
        t0 = time.monotonic()
        while not got and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        if not got:
            return None
        m = got[0]
        b = bytes(m.data)
        step = 97
        sample = b[::step]
        return {"mean": round(sum(sample) / len(sample), 2), "encoding": m.encoding, "w": m.width, "h": m.height}


def stats(stamps):
    s = sorted(stamps)
    n = len(s)
    if n < 2:
        return {"n": n}
    dts = [b - a for a, b in zip(s, s[1:])]
    hist = {"1": 0, "2": 0, "3": 0, "4+": 0}
    for dt in dts:
        k = round(dt / PERIOD)
        hist[str(k) if 1 <= k <= 3 else ("1" if k < 1 else "4+")] += 1
    ds = sorted(dts)
    return {"n": n, "hz": round((n - 1) / sum(dts), 3), "dt_ms_p50": round(ds[len(ds) // 2] * 1e3, 2),
            "dt_ms_min": round(ds[0] * 1e3, 2), "dt_ms_max": round(ds[-1] * 1e3, 2), "gaps_in_periods": hist}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--ns", default="/camera")
    ap.add_argument("--warm", type=float, default=10.0)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--set-ae", choices=["0", "1"])
    ap.add_argument("--set-exp", type=int, help="absolute colour exposure (after --set-ae)")
    ap.add_argument("--set-exp-frac", type=float, help="exposure = this x the reading right after --set-ae 0")
    ap.add_argument("--wait-depth", type=int, default=0, help="only wait for N depth infos (start check), no window")
    a = ap.parse_args()
    rclpy.init()
    p = Probe(a.ns)
    rec = {"label": a.label, "wall_start": time.strftime("%Y-%m-%d %H:%M:%S %z")}
    if a.wait_depth:
        p.on = True
        t0 = time.monotonic()
        while len(p.stamps["depth"]) < a.wait_depth and time.monotonic() - t0 < a.seconds:
            rclpy.spin_once(p, timeout_sec=0.1)
        n = len(p.stamps["depth"])
        print(f"WAIT_DEPTH {n}/{a.wait_depth} in {time.monotonic() - t0:.1f}s colour={len(p.stamps['color'])}")
        sys.exit(0 if n >= a.wait_depth else 1)
    acts = []
    if a.set_ae is not None:
        acts.append({"set_ae": a.set_ae, "r": p.call("set_color_auto_exposure", a.set_ae == "1")})
        time.sleep(0.5)
        acts.append({"after_set_ae": p.exposure()})
    if a.set_exp is not None or a.set_exp_frac is not None:
        cur = acts[-1]["after_set_ae"]["exposure"].get("data") if acts else p.exposure()["exposure"].get("data")
        val = a.set_exp if a.set_exp is not None else max(1, int(round(cur * a.set_exp_frac)))
        acts.append({"set_exp": val, "r": p.call("set_color_exposure", val)})
        time.sleep(0.5)
        acts.append({"after_set_exp": p.exposure()})
    rec["actions"] = acts
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.warm:
        rclpy.spin_once(p, timeout_sec=0.1)
    pid = find_driver_pid()
    rec["driver_pid_in_container"] = pid
    rec["start"] = {"cam": p.exposure(), "bright": p.brightness()}
    c0, th0, b0, w0 = proc_ticks(pid), thread_ticks(pid), board_cpu(), time.monotonic()
    p.on = True
    while time.monotonic() - w0 < a.seconds:
        rclpy.spin_once(p, timeout_sec=0.05)
    p.on = False
    c1, th1, b1, w1 = proc_ticks(pid), thread_ticks(pid), board_cpu(), time.monotonic()
    wall = w1 - w0
    rec["window_s"] = round(wall, 2)
    rec["end"] = {"cam": p.exposure(), "bright": p.brightness()}
    rec["color"] = stats(p.stamps["color"])
    rec["depth"] = stats(p.stamps["depth"])
    rec["driver_cpu_pct_of_core"] = round(100.0 * (c1 - c0) / TCK / wall, 1)
    th = []
    for t, (comm, v) in th1.items():
        v0 = th0.get(t, (comm, v))[1] if t in th0 else 0
        th.append((round(100.0 * (v - v0) / TCK / wall, 1), comm, t))
    th.sort(reverse=True)
    rec["threads_top"] = [{"pct": x[0], "comm": x[1], "tid": x[2]} for x in th[:8]]
    tot, idle = b1[0] - b0[0], b1[1] - b0[1]
    rec["board_busy_pct_all_cores"] = round(100.0 * (tot - idle) / tot, 1) if tot else None
    json.dump(rec, open(a.out, "w"), indent=1)
    s = rec
    print(f"{a.label}: colour {s['color'].get('hz')} Hz gaps {s['color'].get('gaps_in_periods')} | depth "
          f"{s['depth'].get('hz')} Hz | driver {s['driver_cpu_pct_of_core']}% | board {s['board_busy_pct_all_cores']}% | "
          f"exp {s['start']['cam']['exposure'].get('data')}->{s['end']['cam']['exposure'].get('data')} gain "
          f"{s['start']['cam']['gain'].get('data')} | bright {(s['start']['bright'] or {}).get('mean')}")
    p.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
