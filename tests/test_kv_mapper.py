"""NN-MAP1 kv_mapper (2026-11-04): the two key checks, each with a positive control.

1. Contract: the real process (tools/kv_mapper/kv_mapper_proc.py, as mapping_ctl starts it) fed over its socket,
   stopped with SIGINT -> exit 0 + "export done"; tools/car_run/mapping_ctl.MappingCore (the ROS-free state machine
   of the stop path) then reaches "exported" with the map identity (db_sha256 = sha256 of the kv_mapper database,
   host/memory_store), an alignment that is identity within 1e-6 m (host/map_align: optimized poses vs Node table),
   search waypoints / nodes; memory_store.map_identity of the export = kind rtabmap_db (what the second trip binds
   to). Positive control: the export yaml edited after the sha table -> the identity is refused (db_sha256 None)
   (memory_store binds by the yaml + database sha; the pgm is not part of that check, as for RTAB-Map exports).
2. Geometry: one occupied cell at a known odom point lands on the right pgm pixel (map_server convention, row 0 = top),
   a reset empties the map, and the loop-closure hook (apply_correction, +0.2 m) moves it by exactly 4 cells.
   Positive control: the same check against a pgm written WITHOUT the y flip must fail.
Run: PYTHONPATH=. python3 -m pytest -q tests/test_kv_mapper.py   (a few seconds)
"""
import math
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from host import kv_mapper as km  # noqa: E402


def frame_at(stamp, x, y, yaw=0.0, occ_xy=None):
    """80 x 80 grid around (x, y) on the 5 cm lattice: all FREE, one OCC cell at occ_xy (odom), row = y."""
    ox = math.floor(x / 0.05) * 0.05 - 2.0
    oy = math.floor(y / 0.05) * 0.05 - 2.0
    d = np.zeros((80, 80), np.int8)
    d[:5, :] = km.UNK                                       # some unknown cells too
    if occ_xy is not None:
        r, c = int(math.floor((occ_xy[1] - oy) / 0.05 + 1e-9)), int(math.floor((occ_xy[0] - ox) / 0.05 + 1e-9))
        if 0 <= r < 80 and 0 <= c < 80:                     # outside this window: not seen by this frame
            d[r, c] = km.OCC
    return km.GridFrame(stamp, ox, oy, d, (x, y, yaw))


def read_pgm(path):
    raw = open(path, "rb").read()
    head = raw.split(b"\n", 3)
    w, h = (int(v) for v in head[1].split())
    return np.frombuffer(head[3], np.uint8).reshape(h, w)


def read_yaml(path):
    out = {}
    for line in open(path):
        k, _, v = line.partition(":")
        out[k.strip()] = v.strip()
    org = [float(t) for t in out["origin"].strip("[]").split(",")]
    return float(out["resolution"]), org


def pixel_of(yaml_path, x, y):
    """map_server convention: column from origin x, row counted from the TOP (highest y)."""
    res, org = read_yaml(yaml_path)
    img = read_pgm(os.path.join(os.path.dirname(yaml_path), open(yaml_path).readline().split(":")[1].strip()))
    col = int(math.floor((x - org[0]) / res))
    row = img.shape[0] - 1 - int(math.floor((y - org[1]) / res))
    return img, row, col


class Contract(unittest.TestCase):
    def test_process_export_through_mapping_ctl_core(self):
        from tools.car_run.mapping_ctl import MappingCore
        from host import memory_store
        tmp = tempfile.mkdtemp(prefix="nnmap1_")
        run = "20261104_010203"
        db_dir, out = os.path.join(tmp, "db"), os.path.join(tmp, "db", "run_" + run)
        sock = os.path.join(tmp, "in.sock")
        env = dict(os.environ, OUT=out, DB_DIR=db_dir, RUN_TIME=run, PYTHONPATH=str(ROOT))
        p = subprocess.Popen([sys.executable, str(ROOT / "tools/kv_mapper/kv_mapper_proc.py"), "--input-sock", sock,
                              "--shared-out", os.path.join(tmp, "shared.bin")], env=env, stderr=subprocess.PIPE,
                             text=True)
        self.addCleanup(lambda: p.poll() is None and p.kill())
        line = p.stderr.readline()
        self.assertIn("running --", line)
        c = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        # a 3 m drive along x, then up y: frames every 0.5 s, 0.1 m apart
        k = 0
        for i in range(31):
            c.sendto(km.encode_frame(frame_at(100.0 + 0.5 * k, 0.1 * i, 0.0, 0.0, occ_xy=(2.0, 1.0)), k),
                     sock)
            k += 1
        for i in range(1, 16):
            c.sendto(km.encode_frame(frame_at(100.0 + 0.5 * k, 3.0, 0.1 * i, math.pi / 2, occ_xy=(2.0, 1.0)), k), sock)
            k += 1
        c.sendto(b"garbage-datagram", sock)                 # counted, never fatal
        c.sendto(km.encode_voxels([[1, 2, 3], [1, 2, 3]], 0.05), sock)
        time.sleep(1.5)
        hdr, g = km.read_shared(os.path.join(tmp, "shared.bin"))
        self.assertEqual(g.shape, (400, 400))
        self.assertGreater(int((g == km.OCC).sum()), 0)
        p.send_signal(signal.SIGINT)
        rest = p.stderr.read()
        self.assertEqual(p.wait(30), 0, rest)
        self.assertIn("export done", rest)
        self.assertIn("bad input datagram", rest)
        db = os.path.join(db_dir, run + ".db")
        core = MappingCore(out, db_dir, db=db)
        self.assertTrue(core.request_stop())
        core.on_child_exit(0)
        st = core.status()
        self.assertEqual(st["state"], "exported", st)
        self.assertEqual(st["db_sha256"], km.sha256_path(db))
        self.assertTrue(st["sha_table"].startswith("ok"), st["sha_table"])
        a = core.align
        self.assertTrue(a["ok"], a)
        from host import map_align
        c_last = map_align.pose7_matrix(*a["map_odom"])
        self.assertLess(float(np.linalg.norm(c_last[:3, 3])), 1e-6)
        self.assertLess(map_align.rotation_angle(c_last), 1e-6)
        self.assertTrue(st["search_waypoints"]["ok"], st["search_waypoints"])
        self.assertTrue(st["search_nodes"]["ok"])
        ident = memory_store.map_identity(map_yaml=core.yaml)
        self.assertEqual(ident["kind"], "rtabmap_db")
        self.assertEqual(ident["db_sha256"], km.sha256_path(db))

    def test_identity_refused_when_yaml_changed(self):
        """Positive control of the identity check: the export yaml edited after the table -> no db_sha256."""
        from host import memory_store
        tmp = tempfile.mkdtemp(prefix="nnmap1_pc_")
        m = km.KvMapper()
        for i in range(5):
            m.integrate(frame_at(10.0 + 2 * i, 0.2 * i, 0.0, occ_xy=(1.0, 0.0)))
        out, db = os.path.join(tmp, "run"), os.path.join(tmp, "x.db")
        s = m.export(out, "x", db)
        ok = memory_store.export_source_db(s["yaml"], km.sha256_path(s["yaml"]))
        self.assertEqual(ok[0], km.sha256_path(db))
        with open(s["yaml"], "a") as f:
            f.write("# edited\n")
        bad = memory_store.export_source_db(s["yaml"], km.sha256_path(s["yaml"]))
        self.assertIsNone(bad[0])


class Geometry(unittest.TestCase):
    def test_known_cell_reset_and_correction_hook(self):
        tmp = tempfile.mkdtemp(prefix="nnmap1_geo_")
        m = km.KvMapper()
        target = (1.23, -0.87)
        for i in range(3):
            m.integrate(frame_at(1.0 + 2 * i, 0.3 * i, -0.4, occ_xy=target))
        s = m.export(os.path.join(tmp, "a"), "a", os.path.join(tmp, "a.db"))
        img, row, col = pixel_of(s["yaml"], *target)
        self.assertEqual(img[row, col], km.PGM_OCC)
        self.assertEqual(int((img == km.PGM_OCC).sum()), 1)
        # positive control: the same lookup on a pgm without the y flip must miss the cell
        flipped = img[::-1]
        self.assertNotEqual(flipped[row, col], km.PGM_OCC)
        # loop-closure hook: +0.2 m x correction on every node -> the cell moves exactly 4 columns
        n = m.apply_correction({nd["id"]: km.se2(0.2, 0.0, 0.0) for nd in m.nodes}, name="test")
        self.assertEqual(n, len(m.nodes))
        s2 = m.export(os.path.join(tmp, "b"), "b", os.path.join(tmp, "b.db"))
        img2, row2, col2 = pixel_of(s2["yaml"], target[0] + 0.2, target[1])
        self.assertEqual(img2[row2, col2], km.PGM_OCC)
        self.assertEqual(int((img2 == km.PGM_OCC).sum()), 1)
        traj = m.trajectory()
        self.assertAlmostEqual(traj[0][1], 0.2, places=9)
        # reset: empty map, session + 1
        self.assertEqual(m.reset(), 1)
        self.assertEqual(m.status()["known_cells"], 0)
        self.assertEqual(m.nodes, [])


if __name__ == "__main__":
    unittest.main()
