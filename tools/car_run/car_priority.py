#!/usr/bin/env python3
"""Car stack CPU priorities (10-29, owner: the Nano CPU is saturated while mapping and finding together).

Runs INSIDE the board stack container as root (car_up.sh, CAR_PRIORITY=1: the container gets --cap-add SYS_NICE and,
once it is started, ``docker exec -u 0 <stack> python3 /src/tools/car_run/car_priority.py`` keeps this running; it
ends with the container). Every --interval-s it scans /proc (the container's PID namespace: the car stack, plus the
3D map display that the console starts there with docker exec) and sets the nice value of EVERY THREAD of the
matching processes (nice is per thread on Linux; threads created later inherit their creator's value, and a rescan
covers restarted processes and late threads). CPU time only: rates, resolutions and qualities are not touched.

  -5  task runner (host.ros_runtime), pose (host.ros_wheel_odom: odom -> base_link on /tf), chassis serial
      (host.ros_chassis_driver), camera driver (the Orbbec component_container and its ros2 launch parent)
  +5  hub (host.hub.hub_ros: it also carries the console's control commands, so only a mild step down)
  +10 display only: first-person preview (preview_publisher.py), car screen colour (nano_colour.py), 3D map display
      (live_mapping.py, rtabmap_util/map_assembler, its /mapData gate map_data_gate.py) -- the map display keeps its
      rate and quality, it only yields the CPU when the critical ones need it
  everything else stays 0 -- in particular car_inner.sh's "bridge" (host/ros_rolling_nav.py = navigation obstacle
  avoidance), Nav2, rtabmap, the producer.

No real-time classes (docker's default rt_runtime is 0). A refused setpriority (EPERM: no CAP_SYS_NICE) is logged
once per process and the others go on. Log: one JSON line per event on stdout (car_up: $RUN/priority.log):
start, apply (a process got its value, with its thread count), error, summary (every --summary-s).

  car_priority.py [--interval-s 3] [--summary-s 30] [--once]
  car_priority.py --summarize LOG     one line: what is applied (car_up puts it in up_marks.txt after READY)
"""
import argparse
import errno
import json
import os
import sys
import time

CRITICAL, HUB, DISPLAY = -5, 5, 10
CONTAINERS = ("component_container", "component_container_mt", "component_container_isolated")


def _base(arg):
    return arg.rsplit("/", 1)[-1]


def _module(argv):
    """the module of ``python3 [-X..] -m MODULE ...``, else None"""
    for i, a in enumerate(argv[:-1]):
        if a == "-m":
            return argv[i + 1]
    return None


def _script(argv):
    """basename of the first *.py argument (the script a python runs), else None"""
    for a in argv[1:4]:
        if a.endswith(".py"):
            return _base(a)
    return None


def classify(argv, ancestors=()):
    """(role, nice) of one process's argv (list of str), or None = leave it alone. ``ancestors``: the argv lists of
    its parents (nearest first), used to recognise the camera driver under its ros2 launch."""
    if not argv:
        return None
    mod, script, exe = _module(argv), _script(argv), _base(argv[0])
    if mod in ("host.ros_runtime",):
        return "runtime", CRITICAL
    if mod == "host.ros_wheel_odom":
        return "wheel_odom", CRITICAL
    if mod == "host.ros_chassis_driver":
        return "chassis_driver", CRITICAL
    if mod == "host.hub.hub_ros":
        return "hub", HUB
    if script == "preview_publisher.py":
        return "preview", DISPLAY
    if script == "nano_colour.py":
        return "colour", DISPLAY
    if script == "live_mapping.py":
        return "map_display", DISPLAY
    if script == "map_data_gate.py":                # 10-30: the display's /mapData gate (start_live_mapping.sh)
        return "map_gate", DISPLAY
    if exe == "map_assembler":
        return "map_assembler", DISPLAY
    joined = " ".join(argv)
    if "orbbec_camera" in joined and "launch" in joined and ".launch.py" in joined:
        return "camera_launch", CRITICAL            # ros2 launch orbbec_camera gemini2.launch.py (relays its output)
    if exe in CONTAINERS or exe == "orbbec_camera_node":
        if exe == "orbbec_camera_node" or "camera" in joined or any(
                "orbbec_camera" in " ".join(a) for a in ancestors):
            return "camera", CRITICAL
    return None


class Prio:
    def __init__(self, proc="/proc", setter=None, getter=None, out=None, clock=time.monotonic):
        self.proc = proc
        self.setter = setter or (lambda tid, n: os.setpriority(os.PRIO_PROCESS, tid, n))
        self.getter = getter or (lambda tid: os.getpriority(os.PRIO_PROCESS, tid))
        self.out = out or sys.stdout
        self.clock = clock
        self.applied = {}       # pid -> (role, nice, start, threads)
        self.errors = set()     # (pid, start) already reported
        self.sets = 0

    def emit(self, **row):
        row = dict({"t": round(time.time(), 3)}, **row)
        self.out.write(json.dumps(row, sort_keys=True) + "\n")
        self.out.flush()

    def _read(self, pid, name):
        with open(os.path.join(self.proc, str(pid), name), "rb") as f:
            return f.read()

    def _stat(self, pid):
        """(ppid, start time) from /proc/PID/stat (fields after the last ')')"""
        s = self._read(pid, "stat").decode("ascii", "replace")
        rest = s[s.rindex(")") + 2:].split()
        return int(rest[1]), rest[19]

    def snapshot(self):
        """{pid: (argv, ppid, start)} of the processes now readable"""
        procs = {}
        for name in os.listdir(self.proc):
            if not name.isdigit():
                continue
            pid = int(name)
            try:
                argv = [a.decode("utf-8", "replace") for a in self._read(pid, "cmdline").split(b"\0") if a]
                ppid, start = self._stat(pid)
            except (OSError, ValueError, IndexError):
                continue
            procs[pid] = (argv, ppid, start)
        return procs

    def scan(self):
        procs = self.snapshot()
        seen = set()
        for pid, (argv, ppid, start) in procs.items():
            anc, p = [], ppid
            for _ in range(4):
                if p not in procs:
                    break
                anc.append(procs[p][0])
                p = procs[p][1]
            hit = classify(argv, anc)
            if hit is None:
                continue
            role, nice = hit
            seen.add(pid)
            try:
                tids = [int(t) for t in os.listdir(os.path.join(self.proc, str(pid), "task")) if t.isdigit()]
            except OSError:
                continue
            changed, refused = 0, None
            for tid in tids:
                try:
                    if self.getter(tid) != nice:
                        self.setter(tid, nice)
                        changed += 1
                        self.sets += 1
                except OSError as exc:
                    if exc.errno == errno.ESRCH:      # the thread just ended
                        continue
                    refused = exc
            if refused is not None and (pid, start) not in self.errors:
                self.errors.add((pid, start))
                self.emit(ev="error", pid=pid, role=role, nice=nice, errno=refused.errno, error=str(refused),
                          cmd=" ".join(argv)[:160])
            if refused is not None:
                continue
            prev = self.applied.get(pid)
            if prev is None or prev[2] != start:       # first time for this process (late threads: summary "sets")
                self.emit(ev="apply", pid=pid, role=role, nice=nice, threads=len(tids), changed=changed,
                          cmd=" ".join(argv)[:160])
            self.applied[pid] = (role, nice, start, len(tids))
        for pid in list(self.applied):
            if pid not in seen:
                del self.applied[pid]

    def summary(self):
        rows = {}
        for pid, (role, nice, _s, n) in sorted(self.applied.items()):
            rows.setdefault(role, []).append({"pid": pid, "nice": nice, "threads": n})
        self.emit(ev="summary", roles=rows, sets=self.sets, errors=len(self.errors))


def summarize(path):
    """one line from a priority.log: the roles of the last summary/apply rows (pid nice), errors"""
    roles, errors, started = {}, 0, False
    try:
        with open(path) as f:
            lines = f.read().splitlines()
    except OSError as exc:
        return "priority: no log (%s)" % exc
    last_summary = None
    for l in lines:
        try:
            r = json.loads(l)
        except ValueError:
            continue
        ev = r.get("ev")
        if ev == "start":
            started = True
        elif ev == "apply":
            roles.setdefault(r["role"], {})[r["pid"]] = r["nice"]
        elif ev == "error":
            errors += 1
        elif ev == "summary":
            last_summary = r
    if last_summary is not None:
        for role, items in last_summary.get("roles", {}).items():
            roles.setdefault(role, {}).update({i["pid"]: i["nice"] for i in items})
    if not started:
        return "priority: not running (%s)" % ((lines[-1][:160] if lines else "empty log"))
    parts = ["%s %s" % (role, ",".join("%+d(pid %d)" % (n, p) for p, n in sorted(v.items())))
             for role, v in sorted(roles.items())]
    return "priority: %s%s" % ("; ".join(parts) or "nothing matched yet",
                               "; %d refused (see priority.log)" % errors if errors else "")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--interval-s", type=float, default=3.0)
    ap.add_argument("--summary-s", type=float, default=30.0)
    ap.add_argument("--once", action="store_true", help="one scan, one summary, exit")
    ap.add_argument("--proc", default="/proc")
    ap.add_argument("--summarize", metavar="LOG")
    a = ap.parse_args(argv)
    if a.summarize:
        print(summarize(a.summarize))
        return 0
    if not a.interval_s > 0:
        ap.error("--interval-s must be > 0")
    p = Prio(a.proc)
    p.emit(ev="start", euid=os.geteuid(), interval_s=a.interval_s, critical=CRITICAL, hub=HUB, display=DISPLAY)
    last = None
    while True:
        try:
            p.scan()
        except Exception as exc:          # a scan error never ends the loop (display bookkeeping of a priority)
            p.emit(ev="scan_error", error="%s: %s" % (type(exc).__name__, exc))
        now = p.clock()
        if a.once or last is None or now - last >= a.summary_s:
            p.summary()
            last = now
        if a.once:
            return 0
        time.sleep(a.interval_s)


if __name__ == "__main__":
    sys.exit(main())
