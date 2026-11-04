#!/usr/bin/env python3
"""NN-MAP1 (2026-11-04): pull the replay inputs out of an RTAB-Map mapping database (read only) into one cache file.

    python3 -I tools/kv_mapper/rtabdb_extract.py DB OUT.pkl

Per node (Node + Data tables): id, stamp, odometry pose (Node.pose, 12 float32, the wheel+gyro odometry RTAB-Map
received -- host/map_align.read_db_odom's reading), the depth blob as stored (RVL, "DEPTHRVL" + cols + rows, uint16
mm; decoded by replay_rtabdb.py) and the camera calibration blob (rtabmap CameraModel::serialize: 11 int header,
K/D/R/P doubles, local transform base_link <- camera optical, 12 float32). The database disk is slow (a full blob scan
takes minutes), hence the cache. Nothing is written to the database (sqlite mode=ro).
"""
import pickle
import sqlite3
import struct
import sys


def parse_calibration(blob):
    h = struct.unpack("<11i", blob[:44])
    off = 44
    vals = []
    for n in h[6:10]:
        vals.append(struct.unpack("<%dd" % n, blob[off:off + 8 * n]))
        off += 8 * n
    local = struct.unpack("<%df" % h[10], blob[off:off + 4 * h[10]]) if h[10] else None
    return {"version": h[:3], "width": h[4], "height": h[5], "K": vals[0], "D": vals[1], "local": local}


def main(db, out):
    con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
    rows = con.execute("SELECT n.id, n.stamp, n.pose, d.depth, d.calibration FROM Node n JOIN Data d ON n.id = d.id "
                       "ORDER BY n.id").fetchall()
    con.close()
    nodes = []
    for nid, stamp, pose, depth, calib in rows:
        if pose is None or len(pose) != 48 or not depth or not calib:
            continue
        nodes.append({"id": int(nid), "stamp": float(stamp), "pose12": struct.unpack("<12f", pose),
                      "depth": bytes(depth), "calib": parse_calibration(bytes(calib))})
    with open(out + ".tmp", "wb") as f:
        pickle.dump({"db": db, "nodes": nodes}, f, protocol=4)
    import os
    os.replace(out + ".tmp", out)
    print("nodes %d -> %s" % (len(nodes), out))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
