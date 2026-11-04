#!/usr/bin/env python3
"""Real-car rtabmap (2026-10-15): the instrument of tools/car_run/rtabmap_car.sh -- what rtabmap_car.sh's run judged by.

  watch    a ROS node: records the visual odometry's OdomInfo (lost or not, per processed frame), rtabmap's Info
           (loop closures) and the OccupancyGrid on the map topic, one JSON line each (--log), until SIGINT/SIGTERM;
           then writes --summary (the numbers below). It must be subscribed BEFORE rtabmap publishes anything.
  readout  after the run: the summary + rtabmap-info's text + the exported poses/map files -> one readout JSON with
           the numbers of docs/design/PREREG_HANDHELD_RTABMAP_20261015.md section 4 (a-e) and their verdicts.
           Validity (driver did not drop out, walk >= 90 s with a full loop) is partly a person's record
           (walk_notes.txt); the readout records what the files show and leaves the walk to the person.

The statistics are pure functions (no ROS import) so that the unit test checks them on known sequences;
Python 3.8 compatible.
"""
import argparse
import json
import math
import re
import signal
import sys
import time
from pathlib import Path

LOST_SHARE_MAX = 0.05      # PREREG d: lost frames <= 5 % of the processed frames
LOST_STREAK_MAX_S = 2.0    # PREREG d: longest run of lost frames <= 2 s
NODES_MIN = 20             # PREREG b
KNOWN_CELLS_MIN = 1000     # PREREG c
LOOP_DIST_MAX_M = 0.3      # PREREG e
LOOP_YAW_MAX_DEG = 15.0    # PREREG e


def lost_stats(rows):
    """rows: [(stamp_s, lost_bool)] in arrival order (OdomInfo, one per processed frame).
    A lost streak lasts from its first lost frame to the first frame after it that is not lost (the time the
    odometry gave no pose); a streak still running at the end lasts to the last frame. Returns a dict."""
    rows = sorted(rows, key=lambda r: r[0])
    n = len(rows)
    n_lost = sum(1 for _s, lost in rows if lost)
    longest_s, longest_frames, start, frames = 0.0, 0, None, 0
    for stamp, lost in rows:
        if lost:
            if start is None:
                start, frames = stamp, 0
            frames += 1
        elif start is not None:
            if stamp - start > longest_s or (stamp - start == longest_s and frames > longest_frames):
                longest_s, longest_frames = stamp - start, frames
            start = None
    if start is not None:
        span = rows[-1][0] - start
        if span > longest_s or (span == longest_s and frames > longest_frames):
            longest_s, longest_frames = span, frames
    return {"frames": n, "lost": n_lost, "lost_share": (n_lost / float(n)) if n else None,
            "longest_lost_s": round(longest_s, 6), "longest_lost_frames": longest_frames}


def odom_start_ok(lost_flags, n, tail):
    """the odometry start check (rtabmap_car.sh): lost flags of the first records of a fresh rgbd_odometry, in
    arrival order -> (ok, reason). Fails when fewer than n records came, or when the first n end with >= tail
    consecutive lost frames (initialised on a bad frame and never matched again: board smoke_1015_132253)."""
    if len(lost_flags) < n:
        return False, "only %d of %d odom_info records" % (len(lost_flags), n)
    run = 0
    for lost in reversed(lost_flags[:n]):
        if not lost:
            break
        run += 1
    if run >= tail:
        return False, "first %d records end with %d consecutive lost" % (n, run)
    return True, "ok (%d of the first %d lost, %d at the end)" % (sum(lost_flags[:n]), n, run)


def median_rate_hz(stamps):
    """median processing rate = 1 / median gap of consecutive stamps (None with < 2 stamps or no positive gap)."""
    s = sorted(stamps)
    gaps = sorted(b - a for a, b in zip(s, s[1:]) if b > a)
    if not gaps:
        return None
    m = len(gaps) // 2
    med = gaps[m] if len(gaps) % 2 else 0.5 * (gaps[m - 1] + gaps[m])
    return 1.0 / med


def grid_counts(data):
    """OccupancyGrid data (-1 unknown, 0..100) -> (known cells, occupied cells >= 65 as map_server's default)."""
    values = list(data)
    known = sum(1 for v in values if v != -1)
    occupied = sum(1 for v in values if v >= 65)
    return known, occupied


def loop_ids(info_rows):
    """Info rows [(stamp, loop_closure_id)] -> the accepted loop closures (id > 0), in order."""
    return [lid for _s, lid in sorted(info_rows, key=lambda r: r[0]) if lid > 0]


def parse_info_nodes(text):
    """rtabmap-info's text -> {"ltm": n, "wm": n} from its 'LTM:  N nodes ...' / 'WM:  N nodes ...' lines (None when
    absent; colour escape codes tolerated). LTM counts every node the database holds (WM is the subset loaded at
    save time: 0.23.7 replay, LTM 11 / WM 7 / 7 exported poses), so PREREG b ("WM+LTM >= 20") is judged on LTM --
    the stricter reading; the literal sum is recorded beside it."""
    out = {}
    for key in ("LTM", "WM"):
        m = re.search(r"^(?:\x1b\[[0-9;]*m)*\s*%s:\s*(\d+)\s+nodes" % key, text, re.M)
        out[key.lower()] = int(m.group(1)) if m else None
    m = re.search(r"^(?:\x1b\[[0-9;]*m)*\s*Global graph:\s*(\d+)\s+poses and\s+(\d+)\s+links", text, re.M)
    out["graph_poses"], out["graph_links"] = (int(m.group(1)), int(m.group(2))) if m else (None, None)
    return out


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def parse_poses(text):
    """rtabmap-export --poses --poses_format 11 (RGBD-SLAM + id: 'stamp x y z qx qy qz qw id' per line, '#' comments)
    -> [(id, x, y, yaw)] sorted by node id."""
    out = []
    for line in text.splitlines():
        f = line.split()
        if not f or f[0].startswith("#") or len(f) < 9:
            continue
        x, y, _z, qx, qy, qz, qw = map(float, f[1:8])
        out.append((int(float(f[8])), x, y, yaw_of(qx, qy, qz, qw)))
    return sorted(out)


def first_last_gap(poses):
    """the optimized graph's first and last node: planar distance (m) and yaw difference (deg, wrapped)."""
    if len(poses) < 2:
        return None, None
    (_a, x0, y0, t0), (_b, x1, y1, t1) = poses[0], poses[-1]
    d = math.hypot(x1 - x0, y1 - y0)
    dyaw = math.degrees(math.atan2(math.sin(t1 - t0), math.cos(t1 - t0)))
    return d, abs(dyaw)


def status_fields(data):
    """the watch.jsonl fields of one /fpga/localization_status message: the WHOLE string. 2026-10-15 第 5 棒: it was cut
    at 2000 characters, and a status row carrying approvals (each ~300 characters, up to 32) is longer, so every such
    row was unparseable and the lease replays reading watch.jsonl were blind exactly while the gate approved
    (host g4a_re: 968 of 1270 rows). Size: host g4r_re ~9 MB per 100 s of status."""
    return {"data": data}


# ---------------------------------------------------------------- watch (ROS) ---------------------------------------
def watch(a):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from rtabmap_msgs.msg import Info, OdomInfo

    log = open(a.log, "w")
    odom_rows, info_rows, maps, status = [], [], [], []
    t0 = time.time()

    def st(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    def write(kind, **kw):
        kw.update(kind=kind, wall=round(time.time(), 3))
        log.write(json.dumps(kw) + "\n")

    def on_odom_info(m):
        odom_rows.append((st(m.header), bool(m.lost)))
        write("odom_info", stamp=st(m.header), lost=bool(m.lost), inliers=m.inliers, features=m.features,
              time_estimation=m.time_estimation)

    def on_info(m):
        info_rows.append((st(m.header), int(m.loop_closure_id)))
        write("info", stamp=st(m.header), ref_id=int(m.ref_id), loop_closure_id=int(m.loop_closure_id),
              proximity_detection_id=int(m.proximity_detection_id))

    def on_map(m):
        known, occupied = grid_counts(m.data)
        maps.append(known)
        write("map", stamp=st(m.header), width=m.info.width, height=m.info.height,
              resolution=m.info.resolution, known=known, occupied=occupied)

    def on_status(m):
        status.append(m.data)
        write("status", **status_fields(m.data))

    rclpy.init(args=[])
    node = rclpy.create_node("rtabmap_watch")
    sensor = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OdomInfo, a.odom_info, on_odom_info, sensor)
    node.create_subscription(Info, a.info, on_info, sensor)
    # the map: reliable + transient_local matches rtabmap's latched map publisher (and map_server's)
    node.create_subscription(OccupancyGrid, a.map, on_map,
                             QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    if a.status:
        node.create_subscription(String, a.status, on_status, QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    print(json.dumps({"watching": [a.odom_info, a.info, a.map]}), flush=True)
    while not stop:
        rclpy.spin_once(node, timeout_sec=0.2)
        log.flush()
    log.close()
    summary = summarize(odom_rows, info_rows, maps)
    summary["seconds"] = round(time.time() - t0, 1)
    if a.status:
        summary["status_messages"] = len(status)
        summary["status_last"] = status[-1] if status else None
    Path(a.summary).write_text(json.dumps(summary, indent=1))
    print(json.dumps({"summary": a.summary}), flush=True)
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return 0


def summarize(odom_rows, info_rows, maps):
    s = {"odometry": lost_stats(odom_rows),
         "odom_rate_median_hz": median_rate_hz([r[0] for r in odom_rows]),
         "info_messages": len(info_rows), "loop_closures": loop_ids(info_rows),
         "map_messages": len(maps), "map_known_cells_max": max(maps) if maps else 0}
    if s["odom_rate_median_hz"] is not None:
        s["odom_rate_median_hz"] = round(s["odom_rate_median_hz"], 2)
    return s


def publishers(a):
    """print the number of publishers of each --topic after --wait-s of discovery (rtabmap_car.sh's guard: with
    ODOM=visual nothing else may publish /odom -- e.g. the wheel odometry, CAR_WHEEL_ODOM on the drive branch)."""
    import rclpy
    rclpy.init(args=[])
    node = rclpy.create_node("rtabmap_car_guard")
    end = time.monotonic() + a.wait_s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    counts = {t: node.count_publishers(t) for t in a.topic}
    print(json.dumps(counts))
    node.destroy_node()
    rclpy.shutdown()
    return 1 if any(counts.values()) else 0


def odomcheck(a):
    """wait for --n odom_info records written to --log at or after --since (wall seconds), then judge them with
    odom_start_ok. Exit 0 ok, 1 failed start, 2 fewer than --n records within --timeout-s."""
    end = time.monotonic() + a.timeout_s
    flags = []
    while True:
        flags = []
        try:
            for line in Path(a.log).read_text().splitlines():
                try:
                    d = json.loads(line)
                except ValueError:
                    continue   # a line still being written
                if d.get("kind") == "odom_info" and d.get("wall", 0) >= a.since:
                    flags.append(bool(d["lost"]))
        except OSError:
            pass
        if len(flags) >= a.n or time.monotonic() > end:
            break
        time.sleep(0.5)
    ok, reason = odom_start_ok(flags, a.n, a.tail)
    print(json.dumps({"ok": ok, "reason": reason, "records": len(flags), "lost_flags": flags[:a.n]}))
    return 0 if ok else (2 if len(flags) < a.n else 1)


# ---------------------------------------------------------------- readout -------------------------------------------
def readout(a):
    out = Path(a.out)
    summary = json.loads(Path(a.summary).read_text()) if Path(a.summary).is_file() else None
    db = Path(a.db)
    info_text = Path(a.info_txt).read_text(errors="replace") if Path(a.info_txt).is_file() else ""
    poses = parse_poses(Path(a.poses).read_text()) if a.poses and Path(a.poses).is_file() else []
    counts = parse_info_nodes(info_text)
    nodes = counts["ltm"]
    ltm_wm = {k: counts[k] for k in ("ltm", "wm")}
    map_files = sorted(str(p) for p in out.glob(a.map_glob))
    r = {"db": str(db), "db_bytes": db.stat().st_size if db.is_file() else None, "rtabmap_info_nodes": nodes, "rtabmap_info_wm": counts["wm"],
         "rtabmap_info_wm_plus_ltm": None if None in ltm_wm.values() else counts["wm"] + counts["ltm"],
         "rtabmap_info_graph_links": counts["graph_links"], "exported_poses": len(poses),
         "exported_map_files": map_files, "summary": summary}
    r["a_db"] = bool(r["db_bytes"]) and nodes is not None
    # PREREG b, tightened before the evening run (controller 10-15, board static smoke: LTM 94 nodes but 1 node in
    # memory, 0 links, nothing exportable): LTM >= 20 AND >= 20 exported (optimized, linked) poses
    r["b_nodes"] = nodes is not None and nodes >= NODES_MIN and len(poses) >= NODES_MIN
    r["c_map"] = bool(summary) and summary["map_messages"] >= 1 and summary["map_known_cells_max"] >= KNOWN_CELLS_MIN \
        and any(f.endswith(".pgm") for f in map_files) and any(f.endswith(".yaml") for f in map_files)
    if summary and summary["odometry"]["frames"]:
        o = summary["odometry"]
        r["d_odometry"] = o["lost_share"] <= LOST_SHARE_MAX and o["longest_lost_s"] <= LOST_STREAK_MAX_S
    else:
        r["d_odometry"] = None   # no OdomInfo (ODOM=wheel): not judged
    dist, dyaw = first_last_gap(poses)
    r["first_last_distance_m"] = None if dist is None else round(dist, 3)
    r["first_last_yaw_deg"] = None if dyaw is None else round(dyaw, 2)
    loops = summary["loop_closures"] if summary else []
    r["e_loop"] = bool(loops) and dist is not None and dist <= LOOP_DIST_MAX_M and dyaw <= LOOP_YAW_MAX_DEG
    r["e_judged"] = bool(a.judge_loop)
    r["driver_log_dropouts"] = None
    if a.driver_log and Path(a.driver_log).is_file():
        text = Path(a.driver_log).read_text(errors="replace")
        r["driver_log_dropouts"] = len(re.findall(r"disconnect|device removed|reopen|reconnect", text, re.I))
    passed = r["a_db"] and r["b_nodes"] and r["c_map"] and r["d_odometry"] is not False
    if a.judge_loop:
        passed = passed and r["e_loop"]
    r["verdict"] = "pass" if passed else ("chain_ok_quality_short" if r["a_db"] and r["b_nodes"] and r["c_map"]
                                          else "fail")
    Path(a.readout).write_text(json.dumps(r, indent=1))
    print(json.dumps({k: r[k] for k in ("verdict", "a_db", "b_nodes", "c_map", "d_odometry", "e_loop", "e_judged",
                                         "rtabmap_info_nodes", "first_last_distance_m", "first_last_yaw_deg")}))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    w = sub.add_parser("watch")
    w.add_argument("--odom-info", default="/fpga/odom_info")
    w.add_argument("--info", default="/fpga/info")
    w.add_argument("--map", default="/map")
    w.add_argument("--status", default="", help="also record this std_msgs/String topic (the gate's status)")
    w.add_argument("--log", required=True)
    w.add_argument("--summary", required=True)
    r = sub.add_parser("readout")
    r.add_argument("--out", required=True, help="the run directory (exported map files are globbed here)")
    r.add_argument("--summary", required=True)
    r.add_argument("--db", required=True)
    r.add_argument("--info-txt", required=True)
    r.add_argument("--poses", default="")
    r.add_argument("--map-glob", default="export/*")
    r.add_argument("--driver-log", default="")
    r.add_argument("--judge-loop", type=int, default=0, help="1 = judge PREREG e (the pushed bench, FORCE_3DOF=1)")
    r.add_argument("--readout", required=True)
    g = sub.add_parser("publishers", help="exit 1 when any --topic has a publisher")
    g.add_argument("--topic", action="append", required=True)
    g.add_argument("--wait-s", type=float, default=3.0)
    c = sub.add_parser("odomcheck", help="the odometry start check (exit 0 ok, 1 failed, 2 too few records)")
    c.add_argument("--log", required=True)
    c.add_argument("--since", type=float, required=True)
    c.add_argument("--n", type=int, default=20)
    c.add_argument("--tail", type=int, default=10)
    c.add_argument("--timeout-s", type=float, default=30.0)
    a = ap.parse_args(argv)
    return {"watch": watch, "readout": readout, "publishers": publishers, "odomcheck": odomcheck}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
