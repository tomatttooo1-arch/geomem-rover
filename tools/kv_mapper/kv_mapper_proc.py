#!/usr/bin/env python3
"""kv_mapper process (NN-MAP1, 2026-11-04): the PS mapping process that replaces `rtabmap_car.sh map` (DESIGN M-B).

    env OUT=... DB_DIR=... RUN_TIME=... python3 tools/kv_mapper/kv_mapper_proc.py [--input-sock PATH]
        [--shared-out /dev/shm/fpga_kv_map.bin] [--status-out FILE] [--size-m 20] [--ros]

Drop-in child of tools/car_run/mapping_ctl.py (same env and the same contract as rtabmap_car.sh / the host stand-in
tools/car_run/mapping_standin.py): prints "running --" on stderr once its input socket is bound; on SIGINT / SIGTERM
exports (host/kv_mapper.KvMapper.export: OUT/export/<RUN_TIME>.pgm/.yaml/_poses.txt, DB_DIR/<RUN_TIME>.db,
OUT/sha256.txt), logs "export done" and exits 0 (an export error: exit 3, mapping_ctl -> failed); SIGUSR1 = map reset
in place (the in-process equivalent of RTAB-Map's /rtabmap/reset; mapping_ctl's respawn path also works unchanged:
a new process under a new RUN_TIME).
Input: AF_UNIX datagrams (host/kv_mapper.encode_frame / encode_voxels) from the geometry producer, one per fused frame
(the grid it publishes as /fpga/rolling_map/grid + the base pose at that frame). Malformed datagrams are counted and
logged, never fatal (FPGA rule: warn and continue).
Outputs while mapping: --shared-out (host/kv_mapper.write_shared, 1 Hz, G2 / hub read it), --status-out (JSON, 1 Hz).
--ros (R1, KV with ROS 2 Humble minimal): also publishes the global grid as nav_msgs/OccupancyGrid on
--ros-map-topic (default /fpga/rtabmap/map, frame map, transient local -- the topic ros_nav_global_map reads today) and
serves std_srvs/Empty --ros-reset-service (default /rtabmap/reset, the name mapping_ctl calls). NOT run on the host
(no rclpy here); the core and the plain process are tested without ROS.
Loop closure: not in this version (owner decision pending); host/kv_mapper.KvMapper.apply_correction is the hook.
"""
import argparse
import json
import os
import select
import signal
import socket
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host import kv_mapper as km  # noqa: E402


def log(msg):
    sys.stderr.write("[kv_mapper] %s\n" % msg)
    sys.stderr.flush()


class RosShell(object):
    """R1 only: OccupancyGrid publisher + reset service on a spin thread. Imports rclpy lazily."""

    def __init__(self, a, on_reset):
        import rclpy
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_srvs.srv import Empty
        rclpy.init(args=None)
        self.rclpy, self.Grid = rclpy, OccupancyGrid
        self.node = rclpy.create_node("fpga_kv_mapper")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.node.create_publisher(OccupancyGrid, a.ros_map_topic, qos)
        self.node.create_service(Empty, a.ros_reset_service, lambda req, res: (on_reset(), res)[1])
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def publish(self, mapper):
        msg = self.Grid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.info.resolution = mapper.res
        msg.info.width = msg.info.height = mapper.n
        msg.info.origin.position.x, msg.info.origin.position.y = mapper.origin
        msg.info.origin.orientation.w = 1.0
        msg.data = mapper.tri().ravel().tolist()
        self.pub.publish(msg)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.environ.get("OUT"))
    ap.add_argument("--db-dir", default=os.environ.get("DB_DIR"))
    ap.add_argument("--run-time", default=os.environ.get("RUN_TIME"))
    ap.add_argument("--input-sock", default=os.environ.get("KV_MAPPER_SOCK", "/run/fpga/kv_mapper.sock"))
    ap.add_argument("--shared-out", default=os.environ.get("KV_MAPPER_SHARED"))
    ap.add_argument("--status-out", default=None)
    ap.add_argument("--size-m", type=float, default=20.0)
    ap.add_argument("--publish-period-s", type=float, default=1.0)
    ap.add_argument("--ros", action="store_true")
    ap.add_argument("--ros-map-topic", default="/fpga/rtabmap/map")
    ap.add_argument("--ros-reset-service", default="/rtabmap/reset")
    a = ap.parse_args(argv)
    if not (a.out and a.db_dir and a.run_time):
        ap.error("OUT, DB_DIR and RUN_TIME (env, as rtabmap_car.sh) or --out/--db-dir/--run-time are required")
    db = os.path.join(a.db_dir, a.run_time + ".db")
    if os.path.exists(db):
        log("database %s exists -- refusing to overwrite another map (rtabmap_car.sh rule)" % db)
        return 2
    os.makedirs(a.out, exist_ok=True)
    mapper = km.KvMapper(size_m=a.size_m)
    lock = threading.Lock()
    events = []

    def on_signal(signum, _frame):
        events.append("reset" if signum == signal.SIGUSR1 else "stop")

    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGUSR1):
        signal.signal(s, on_signal)

    def do_reset():
        with lock:
            sess = mapper.reset()
        log("map reset: session %d" % sess)

    ros = None
    if a.ros:
        try:
            ros = RosShell(a, do_reset)
        except Exception as exc:                   # noqa: BLE001 - warn and run without the ROS outputs
            log("WARNING ROS shell not started (%r): no %s / %s" % (exc, a.ros_map_topic, a.ros_reset_service))
    os.makedirs(os.path.dirname(os.path.abspath(a.input_sock)), exist_ok=True)
    if os.path.exists(a.input_sock):
        os.remove(a.input_sock)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(a.input_sock)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    log("running -- input %s, db %s, out %s, global %.0f m @ %.2f m%s" % (
        a.input_sock, db, a.out, a.size_m, mapper.res, ", ROS %s" % a.ros_map_topic if ros else ""))
    bad, next_pub = 0, 0.0
    try:
        while True:
            while events:
                ev = events.pop(0)
                if ev == "reset":
                    do_reset()
                else:
                    raise KeyboardInterrupt
            r, _, _ = select.select([sock], [], [], 0.2)
            if r:
                buf = sock.recv(1 << 20)
                try:
                    kind, payload = km.decode(buf)
                    with lock:
                        if kind == "frame":
                            mapper.integrate(payload)
                        else:
                            mapper.add_voxels(*payload)
                except ValueError as exc:
                    bad += 1
                    if bad <= 5 or bad % 100 == 0:
                        log("WARNING bad input datagram #%d: %s" % (bad, exc))
            now = time.monotonic()
            if now >= next_pub:
                next_pub = now + a.publish_period_s
                with lock:
                    st = dict(mapper.status(), bad_datagrams=bad)
                    if a.shared_out:
                        mapper.write_shared(a.shared_out)
                    if ros is not None:
                        ros.publish(mapper)
                if a.status_out:
                    with open(a.status_out + ".tmp", "w") as f:
                        json.dump(st, f, default=str)
                    os.replace(a.status_out + ".tmp", a.status_out)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        try:
            os.remove(a.input_sock)
        except OSError:
            pass
    log("SIGINT/SIGTERM: saving the database and exporting")
    try:
        with lock:
            summary = mapper.export(a.out, a.run_time, db)
    except Exception as exc:                       # noqa: BLE001 - mapping_ctl turns a nonzero exit into failed
        log("export FAILED: %r" % (exc,))
        return 3
    log("export done: %s (frames %d, nodes %d, occupied %d, db sha256 %s)" % (
        " ".join(sorted(os.listdir(os.path.join(a.out, "export")))), summary["frames"], summary["nodes"],
        summary["occupied_cells"], summary["db_sha256"][:16]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
