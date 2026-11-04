#!/usr/bin/env python3
"""B1 Nano side, ROS-level smoke (2026-11-02): the producer's GridOutput vs the bridge node in shadow mode.

Two steps (the host has no rclpy; the ROS half runs in a humble image, --network none):

  prepare (host)   python3 tools/b1_costmap/ros_smoke.py prepare --out DIR [--frames 120]
                   replays the recorded trace through the Nano backend -> ROLL v2 -> RollingService (CPU core +
                   the PL grid C model) and writes, per accepted INTEGRATE, the encoded delta (what the producer
                   publishes for the bridge) and the KV planes (what GridOutput takes): DIR/frames.npz + meta.json
  ros (container)  python3 tools/b1_costmap/ros_smoke.py ros --data DIR --out DIR/ros_result.json
                   one process: host.ros_rolling_nav.RollingBridgeNode with grid_topic / raw_topic / status_topic
                   = the _sw topics (CAR_PL_GRID_SHADOW=1), a "producer" node with GridOutput over a PlGridLayer,
                   a static odom->base_link (same body pose for both). Per frame: the delta goes to the bridge on
                   /fpga/rolling_map/delta, the planes to GridOutput; then both latched grids are taken and compared
                   cell by cell (/grid vs /grid_sw, /grid_raw vs /grid_raw_sw) and the status rows' fresh / reason
                   / epoch / delta_seq.

Bridge parameters = car_inner (ground 0, robot height 0.31 here, radius 0.30, margin 0.05, swept 300 s).
"""
import argparse
import json
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

TRACE = "${FPGA_SRC}/runs/fpga-innov-20261025/fuse/traces/formal.bin"
ROBOT_H = 0.31
POSE = None    # body pose: the window centre of the first frame + (0.013, -0.021) m, fixed (static TF)


def prepare(a):
    import tempfile
    from host.kv_rolling import CpuRollingCore, RemotePlRollingBackend, RollingService
    from host.m7_rolling import NavigationDelta, compile_rolling
    from host.fpga_clock import CLOCK_ID
    from host.rolling_nav_layer import HeightBand, encode_delta
    from tools.m7_rolling_pl.replay_trace import read_trace
    sys.path.insert(0, os.path.join(ROOT, "tests"))
    from test_b1_pl_grid_nano import InProcLink, integrate_rec, reset_rec

    tmp = tempfile.mkdtemp(prefix="b1smoke_")
    svc = RollingService(CpuRollingCore(compile_rolling(os.path.join(tmp, "r.so")),
                                        grid_library=os.path.join(tmp, "g.so")))
    remote = RemotePlRollingBackend(None, link=InProcLink(svc))
    remote.grid_band, remote.grid_voxel_m = HeightBand(ground_z_m=0.0, robot_height_m=ROBOT_H), 0.05
    deltas, grids, auxs, meta = [], [], [], []
    started = False
    for r in read_trace(a.trace):
        if r["op"] == 0:
            if reset_rec(remote, r):
                if started:
                    break          # one epoch is enough for the smoke
                started = True
            continue
        if r["op"] != 1 or not started:
            continue
        res = integrate_rec(remote, r)
        if res is None or not res.ok:
            continue
        if res.grid is None:
            break                  # band outside the window: the bridge could not project either; stop here
        k = len(meta)
        acq = 10 ** 9 + k * 100_000_000          # rebased onto the container's clock at run time
        d = NavigationDelta(res.epoch, res.delta_seq, int(r["frame"]), acq, CLOCK_ID, acq + 200_000_000,
                            res.window_base, (80, 80, 32), 0.05, res.changes)
        deltas.append(np.frombuffer(encode_delta(d), np.uint8))
        grids.append(res.grid.grid.copy())
        auxs.append(res.grid.aux.copy())
        meta.append({"epoch": res.epoch, "delta_seq": res.delta_seq, "window_base": list(res.window_base)})
        if len(meta) >= a.frames:
            break
    os.makedirs(a.out, exist_ok=True)
    np.savez_compressed(os.path.join(a.out, "frames.npz"), grid=np.stack(grids), aux=np.stack(auxs),
                        **{"delta_%d" % i: x for i, x in enumerate(deltas)})
    with open(os.path.join(a.out, "meta.json"), "w") as f:
        json.dump({"frames": meta, "robot_height_m": ROBOT_H}, f)
    print("prepared %d frames -> %s" % (len(meta), a.out))


def ros(a):
    import rclpy
    import rclpy.executors
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String, UInt8MultiArray
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from host.pl_grid_layer import PlGridLayer, PlGridPlanes
    from host.fpga_clock import CLOCK_ID, now_ns
    from host.rolling_nav_layer import HeightBand, decode_delta, encode_delta
    from host.ros_rolling_nav import GRID_QOS, STREAM_QOS, GridOutput, RollingBridgeNode
    from dataclasses import replace

    meta = json.load(open(os.path.join(a.data, "meta.json")))
    z = np.load(os.path.join(a.data, "frames.npz"))
    frames = meta["frames"]
    wb0 = frames[0]["window_base"]
    pose = ((wb0[0] + 40) * 0.05 + 0.013, (wb0[1] + 40) * 0.05 - 0.021, 0.3)

    rclpy.init(args=["--ros-args", "-p", "ground_z_m:=0.0", "-p", "robot_height_m:=%s" % meta["robot_height_m"],
                     "-p", "robot_radius_m:=0.30", "-p", "body_margin_m:=0.05", "-p", "swept_clear_s:=300.0",
                     "-p", "grid_topic:=/fpga/rolling_map/grid_sw", "-p", "raw_topic:=/fpga/rolling_map/grid_raw_sw",
                     "-p", "status_topic:=/fpga/rolling_map/status_sw"])
    bridge = RollingBridgeNode()
    prod = Node("fpga_geo_producer_smoke", parameter_overrides=[], use_global_arguments=False)
    tfb = StaticTransformBroadcaster(prod)
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = "odom", "base_link"
    t.transform.translation.x, t.transform.translation.y = pose[0], pose[1]
    t.transform.rotation.z, t.transform.rotation.w = float(np.sin(pose[2] / 2)), float(np.cos(pose[2] / 2))
    tfb.sendTransform(t)
    band = HeightBand(ground_z_m=0.0, robot_height_m=meta["robot_height_m"])
    layer = PlGridLayer(dims=(80, 80, 32), voxel_m=0.05, band=band, clock_id=CLOCK_ID, frame_id="odom",
                        body_radius_m=0.30, body_margin_m=0.05, swept_clear_s=300.0)
    out = GridOutput(prod, layer, pose=lambda: pose, source="pl")
    delta_pub = prod.create_publisher(UInt8MultiArray, "/fpga/rolling_map/delta", STREAM_QOS)
    got = {}

    def keep(name):
        def cb(m):
            got[name] = m
        return cb
    for name, typ, qos in (("/fpga/rolling_map/grid", OccupancyGrid, GRID_QOS),
                           ("/fpga/rolling_map/grid_sw", OccupancyGrid, GRID_QOS),
                           ("/fpga/rolling_map/grid_raw", OccupancyGrid, GRID_QOS),
                           ("/fpga/rolling_map/grid_raw_sw", OccupancyGrid, GRID_QOS),
                           ("/fpga/rolling_map/status", String, 50), ("/fpga/rolling_map/status_sw", String, 50)):
        prod.create_subscription(typ, name, keep(name), qos)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(bridge)
    ex.add_node(prod)
    t_end = time.time() + 3.0                     # discovery + TF
    while time.time() < t_end:
        ex.spin_once(timeout_sec=0.05)
    res = {"frames": 0, "grid_cells": 0, "raw_cells": 0, "status_diff": 0, "timeouts": 0, "occupied": 0,
           "first_diffs": []}
    for k, f in enumerate(frames):
        acq = now_ns() - 20_000_000              # captured 20 ms ago on this clock: fresh for both
        d = decode_delta(bytes(z["delta_%d" % k]))
        d = replace(d, acquired_ns=acq, expires_at_ns=acq + 200_000_000)
        before = {n: got.get(n) for n in ("/fpga/rolling_map/grid", "/fpga/rolling_map/grid_sw")}
        msg = UInt8MultiArray()
        msg.data = bytes(encode_delta(d))
        delta_pub.publish(msg)
        layer_ok = False
        with out.lock:
            layer_ok, _ = layer.on_planes(epoch=f["epoch"], delta_seq=f["delta_seq"], window_base=f["window_base"],
                                          acquired_ns=acq, clock_id=CLOCK_ID,
                                          planes=PlGridPlanes(0, z["grid"][k], z["aux"][k]))
        out.publish_grid()
        t_end = time.time() + 2.0
        while time.time() < t_end and any(got.get(n) is before[n] for n in before):
            ex.spin_once(timeout_sec=0.02)
        if any(got.get(n) is before[n] for n in before):
            res["timeouts"] += 1
            continue
        def seq_of(name):
            m = got.get(name)
            return None if m is None else json.loads(m.data).get("delta_seq")
        t_end = time.time() + 1.0                 # status rows of this frame (any order across topics)
        while time.time() < t_end and not (seq_of("/fpga/rolling_map/status") == f["delta_seq"] ==
                                           seq_of("/fpga/rolling_map/status_sw")):
            ex.spin_once(timeout_sec=0.01)
        res["frames"] += 1
        g = np.array(got["/fpga/rolling_map/grid"].data, np.int8)
        gs = np.array(got["/fpga/rolling_map/grid_sw"].data, np.int8)
        r_ = np.array(got["/fpga/rolling_map/grid_raw"].data, np.int8)
        rs = np.array(got["/fpga/rolling_map/grid_raw_sw"].data, np.int8)
        dg, dr = int((g != gs).sum()), int((r_ != rs).sum())
        res["grid_cells"] += dg
        res["raw_cells"] += dr
        res["occupied"] += int((g == 100).sum())
        mi = (got["/fpga/rolling_map/grid"].info, got["/fpga/rolling_map/grid_sw"].info)
        if (mi[0].origin.position.x, mi[0].origin.position.y, mi[0].width) != \
                (mi[1].origin.position.x, mi[1].origin.position.y, mi[1].width):
            res["status_diff"] += 1
        st = json.loads(got["/fpga/rolling_map/status"].data)
        ss = json.loads(got["/fpga/rolling_map/status_sw"].data)
        keys = ("fresh", "reason", "epoch", "delta_seq")
        if tuple(st.get(x) for x in keys) != tuple(ss.get(x) for x in keys):
            res["status_diff"] += 1
            if len(res["first_diffs"]) < 5:
                res["first_diffs"].append({"k": k, "pl": {x: st.get(x) for x in keys}, "sw": {x: ss.get(x) for x in keys}})
        if (dg or dr) and len(res["first_diffs"]) < 5:
            res["first_diffs"].append({"k": k, "grid": dg, "raw": dr})
    res["status_pl_has_grid_source"] = json.loads(got["/fpga/rolling_map/status"].data).get("grid_source")
    res["bridge_counts"] = dict(bridge.bridge.counts)
    res["layer_counts"] = dict(layer.counts)
    with open(a.out, "w") as fh:
        json.dump(res, fh, indent=1, default=int)
    print(json.dumps({k: v for k, v in res.items() if k not in ("bridge_counts", "layer_counts")}))
    rclpy.shutdown()
    return 0 if res["frames"] and not (res["grid_cells"] or res["raw_cells"] or res["status_diff"]) else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("prepare")
    p.add_argument("--trace", default=TRACE)
    p.add_argument("--out", required=True)
    p.add_argument("--frames", type=int, default=120)
    r = sub.add_parser("ros")
    r.add_argument("--data", required=True)
    r.add_argument("--out", required=True)
    a = ap.parse_args()
    return prepare(a) if a.cmd == "prepare" else ros(a)


if __name__ == "__main__":
    sys.exit(main() or 0)
