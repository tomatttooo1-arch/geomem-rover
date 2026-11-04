"""GeoMem 展台底图包工具（ui/geomem/tools/basemap/make_basemap.py，10-17）：合成点云上的裁剪/离群/分格/平均色、
输入顺序无关的确定性、PLY 读法（尾随 face/camera 元素）与包格式往返。不起 docker（--ply/--poses 路径）。

10-18：合成场景按 2 cm 格、格内 ≥3 点设计，工具默认 10-17 改成 1 cm / ≥2 点后（146ee56、cbbb23a 改了计数键名）
这里显式传 2 cm / 3 点，并改用 ``after_outliers``；另加 k-d 树分块查询与不分块逐字节相同、``--export native``
（假 rtabmap-export）两组。"""

import hashlib
import json
import os
import shutil
import struct
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ui" / "geomem" / "tools" / "basemap"))
sys.path.insert(0, str(ROOT / "ui" / "geomem" / "bridge"))

import basemap_tsdf as bt  # noqa: E402
import gmb_basemap as gbm  # noqa: E402
import make_basemap as mb  # noqa: E402

S = 0.02
OLD = {"cell_m": 0.02, "min_points": 3}          # the synthetic scene's grid (the tool's default was 2 cm / 3 until 10-17)


def cluster(cx, cy, cz, n, rgb, rng, spread=0.006):
    """n points inside the 2 cm cell whose lower corner is (cx, cy, cz) (well away from its faces)."""
    p = np.stack([cx + 0.01 + rng.uniform(-spread, spread, n), cy + 0.01 + rng.uniform(-spread, spread, n),
                  cz + 0.01 + rng.uniform(-spread, spread, n)], 1)
    return p, np.tile(np.array(rgb, np.uint8), (n, 1))


def write_ply(path, xyz, rgb, trailing=True):
    n = len(xyz)
    head = ("ply\nformat binary_little_endian 1.0\ncomment test\nelement vertex %d\nproperty float x\nproperty float y\n"
            "property float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty float nx\n"
            "property float ny\nproperty float nz\nproperty float curvature\n" % n)
    if trailing:
        head += "element face 0\nproperty list uchar int vertex_indices\nelement camera 1\nproperty float view_px\n"
    head += "end_header\n"
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("red", "u1"), ("green", "u1"), ("blue", "u1"),
                   ("nx", "<f4"), ("ny", "<f4"), ("nz", "<f4"), ("curvature", "<f4")])
    v = np.zeros(n, dt)
    v["x"], v["y"], v["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    v["red"], v["green"], v["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    with open(path, "wb") as f:
        f.write(head.encode("ascii"))
        f.write(v.tobytes())
        if trailing:
            f.write(struct.pack("<f", 1.5))       # the camera element's one float


def write_poses(path, pts):
    with open(path, "w") as f:
        f.write("#timestamp x y z qx qy qz qw id\n")
        for k, (x, y) in enumerate(pts):
            f.write("%f %f %f 0 0 0 0 1 %d\n" % (100.0 + k, x, y, k + 1))


def scene(rng):
    """Floor cells along the trajectory (each 5 points), plus the cases each step must handle."""
    xyz, rgb = [], []
    for i in range(-5, 100):             # x in [-0.10, 2.0): floor strip, y cell -1 and 0
        for j in (-1, 0):
            p, c = cluster(i * S, j * S, -0.02, 5, (100 + (i % 7), 120, 140), rng)
            xyz.append(p)
            rgb.append(c)
    # averaged colour: three points, one cell (x cell 50, y cell 1, z cell 0), next to the floor strip
    xyz.append(np.array([[1.005, 0.025, 0.005], [1.010, 0.030, 0.010], [1.012, 0.032, 0.012]]))
    rgb.append(np.array([[10, 20, 30], [20, 30, 40], [31, 41, 51]], np.uint8))
    # two points only: dropped by min_points
    xyz.append(np.array([[1.505, 0.025, 0.005], [1.506, 0.026, 0.006]]))
    rgb.append(np.array([[1, 2, 3], [1, 2, 3]], np.uint8))
    # far from the trajectory (y = 2.5 m): cropped
    p, c = cluster(1.0, 2.5, 0.0, 5, (9, 9, 9), rng)
    xyz.append(p)
    rgb.append(c)
    # below z_min: removed; NaN: removed
    xyz.append(np.array([[1.0, 0.0, -0.3], [np.nan, 0.0, 0.0]]))
    rgb.append(np.array([[7, 7, 7], [7, 7, 7]], np.uint8))
    # a lone point 0.6 m above the floor: statistical outlier
    xyz.append(np.array([[0.5, 0.0, 0.6]]))
    rgb.append(np.array([[200, 0, 0]], np.uint8))
    return np.concatenate(xyz).astype(np.float64), np.concatenate(rgb).astype(np.uint8)


TRAJ = [(0.1 * k, 0.0) for k in range(21)]


def rd(path):
    with open(path, "rb") as f:
        return f.read()


class BuildCellsTests(unittest.TestCase):
    def setUp(self):
        self.xyz, self.rgb = scene(np.random.RandomState(7))

    def build(self, xyz=None, rgb=None, **kw):
        args = dict(OLD, **kw)
        return mb.build_cells(self.xyz if xyz is None else xyz, self.rgb if rgb is None else rgb, TRAJ, workers=1,
                              **args)

    def test_steps_and_counts(self):
        idx, col, counts = self.build()
        n_floor = 105 * 2 * 5
        self.assertEqual(counts["read"], len(self.xyz))
        self.assertEqual(counts["finite_above_z_min"], len(self.xyz) - 2)
        self.assertEqual(counts["cropped"], counts["finite_above_z_min"] - 5)
        self.assertEqual(counts["after_outliers"], counts["cropped"] - 1)       # only the lone point
        self.assertEqual(counts["cells_all"], 105 * 2 + 2)
        self.assertEqual(counts["cells"], 105 * 2 + 1)                           # the 2-point cell dropped
        self.assertEqual(counts["after_outliers"], n_floor + 3 + 2)
        keys = set(map(tuple, idx.tolist()))
        self.assertIn((-5, -1, -1), keys)               # negative coordinates floor, not truncate
        self.assertNotIn((75, 1, 0), keys)              # the 2-point cell
        self.assertNotIn((25, 0, 30), keys)             # the outlier
        self.assertFalse(any(k[1] >= 100 for k in keys))   # cropped cluster at y = 2.5 m

    def test_mean_colour_rounding(self):
        idx, col, _ = self.build()
        i = [tuple(k) for k in idx.tolist()].index((50, 1, 0))
        self.assertEqual(col[i].tolist(), [20, 30, 40])   # (20.33, 30.33, 40.33)
        two = np.array([[0.505, 0.505, 0.005], [0.506, 0.506, 0.006], [0.507, 0.507, 0.007]])
        _, c2, _ = mb.build_cells(two, np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2]], np.uint8), [(0.5, 0.5)], sor_k=1,
                                  sor_std=10.0, workers=1, **OLD)
        self.assertEqual(c2.tolist(), [[1, 1, 1]])

    def test_input_order_does_not_matter(self):
        idx, col, counts = self.build()
        ref = mb.cells_bytes(idx, col)
        for seed in (1, 2, 3):
            perm = np.random.RandomState(seed).permutation(len(self.xyz))
            i2, c2, n2 = self.build(self.xyz[perm], self.rgb[perm])
            self.assertEqual(mb.cells_bytes(i2, c2), ref, seed)
            self.assertEqual(n2, counts)

    def test_sorted_and_unique(self):
        idx, _, _ = self.build()
        rows = [tuple(k) for k in idx.tolist()]
        self.assertEqual(rows, sorted(rows))
        self.assertEqual(len(rows), len(set(rows)))

    def test_empty_after_crop(self):
        idx, col, counts = mb.build_cells(self.xyz, self.rgb, [(50.0, 50.0)], workers=1, **OLD)
        self.assertEqual((idx.shape, col.shape, counts["cells"]), ((0, 3), (0, 3), 0))

    def test_chunked_query_bit_identical(self):
        """10-18: crop/SOR k-d queries run in QUERY_CHUNK blocks (Nano memory); every chunk size gives the same
        bytes and counts as one block (also under the 1 cm defaults)."""
        for kw in (OLD, {}):
            idx, col, counts = self.build(**kw) if kw else mb.build_cells(self.xyz, self.rgb, TRAJ, workers=1)
            ref = mb.cells_bytes(idx, col)
            saved = mb.QUERY_CHUNK
            try:
                for chunk in (1, 7, 100, 1000):
                    mb.QUERY_CHUNK = chunk
                    i2, c2, n2 = mb.build_cells(self.xyz, self.rgb, TRAJ, workers=1, **kw)
                    self.assertEqual((mb.cells_bytes(i2, c2), n2), (ref, counts), (kw, chunk))
            finally:
                mb.QUERY_CHUNK = saved
            self.assertGreater(counts["cells"], 0)

    def test_int16_range_guard(self):
        far = np.array([[700.0, 0.0, 0.0]] * 4) + np.arange(4)[:, None] * 0.001
        with self.assertRaises(ValueError):
            mb.build_cells(far, np.zeros((4, 3), np.uint8), [(700.0, 0.0)], sor_k=1, sor_std=10.0, workers=1)


class FileFormatTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="gm_basemap_")
        xyz, rgb = scene(np.random.RandomState(11))
        self.ply = os.path.join(self.tmp, "gm_cloud.ply")
        self.poses = os.path.join(self.tmp, "gm_poses.txt")
        self.db = os.path.join(self.tmp, "map.db")
        write_ply(self.ply, xyz, rgb)
        write_poses(self.poses, TRAJ)
        with open(self.db, "wb") as f:
            f.write(b"not really sqlite " * 100)
        self.xyz, self.rgb = xyz, rgb

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def run_tool(self, out):
        argv = ["--db", self.db, "--out-dir", out, "--ply", self.ply, "--poses", self.poses, "--workers", "1",
                "--cell", "0.02", "--min-points", "3"]
        from io import StringIO
        old = sys.stdout
        sys.stdout = StringIO()
        try:
            self.assertEqual(mb.main(argv), 0)
        finally:
            sys.stdout = old
        db_sha = hashlib.sha256(rd(self.db)).hexdigest()
        return os.path.join(out, "basemap-" + db_sha[:12]), db_sha

    def test_read_ply_skips_trailing_elements(self):
        xyz, rgb = mb.read_ply(self.ply)
        self.assertEqual(len(xyz), len(self.xyz))
        np.testing.assert_array_equal(rgb, self.rgb)
        np.testing.assert_allclose(xyz[np.isfinite(xyz).all(1)],
                                   self.xyz[np.isfinite(self.xyz).all(1)].astype(np.float32), rtol=0, atol=0)

    def test_read_poses_sorted_with_yaw(self):
        with open(self.poses, "a") as f:
            f.write("50.0 9 9 0 0 0 0.7071068 0.7071068 99\n")
        p = mb.read_poses(self.poses)
        self.assertEqual(p[0][0], 50.0)
        self.assertAlmostEqual(p[0][3], np.pi / 2, places=5)

    def test_package_roundtrip_and_determinism(self):
        pa, db_sha = self.run_tool(os.path.join(self.tmp, "a"))
        pb, _ = self.run_tool(os.path.join(self.tmp, "b"))
        ca = rd(os.path.join(pa, "cells.bin"))
        self.assertEqual(ca, rd(os.path.join(pb, "cells.bin")))
        ja, jb = (json.loads(rd(os.path.join(p, "basemap.json")).decode("utf-8")) for p in (pa, pb))
        for j in (ja, jb):
            j.pop("generated_at")
            j.pop("timing_s")
        self.assertEqual(mb.manifest_bytes(ja), mb.manifest_bytes(jb))
        self.assertEqual((ja["format"], ja["version"], ja["frame"]), ("geomem-basemap", 1, "map"))
        self.assertEqual(ja["source_db"], {"name": "map.db", "sha256": db_sha, "bytes": os.path.getsize(self.db)})
        f = ja["files"]["cells"]
        self.assertEqual((f["bytes"], f["sha256"]), (len(ca), hashlib.sha256(ca).hexdigest()))
        idx, rgb = mb.parse_cells(ca)
        self.assertEqual(f["count"], len(idx))
        ref_idx, ref_rgb, _ = mb.build_cells(*mb.read_ply(self.ply), TRAJ, workers=1, **OLD)
        np.testing.assert_array_equal(idx, ref_idx)
        np.testing.assert_array_equal(rgb, ref_rgb)
        self.assertEqual(len(ja["trajectory"]), len(TRAJ))
        lo, hi = ja["bbox_cell_centers_m"]["min"], ja["bbox_cell_centers_m"]["max"]
        self.assertAlmostEqual(lo[0], (-5 + 0.5) * S)
        self.assertAlmostEqual(hi[0], (99 + 0.5) * S)
        self.assertIsNone(ja["export"]["command"])      # --ply given: no export
        self.assertEqual(ja["export"]["mode"], "ply")
        # the bridge (stdlib only) accepts what the tool wrote
        pkg = gbm.load_package(pa)
        self.assertEqual((pkg["sha256"], pkg["count"], pkg["cells"]), (db_sha, len(idx), ca))

    def test_existing_package_refused_without_force(self):
        out = os.path.join(self.tmp, "c")
        self.run_tool(out)
        with self.assertRaises(SystemExit):
            self.run_tool(out)

    def test_native_export_mode(self):
        """10-18 --export native: a stand-in rtabmap-export (copies the fixture PLY/poses into --output_dir as
        gm_cloud.ply / gm_poses.txt and records its argv); the package equals the --ply one byte for byte."""
        fake = os.path.join(self.tmp, "fake_export.py")
        argv_log = os.path.join(self.tmp, "argv.json")
        with open(fake, "w", encoding="utf-8") as f:
            f.write("import json, shutil, sys, os\n"
                    "a = sys.argv[1:]\n"
                    "json.dump(a, open(%r, 'w'))\n"
                    "out = a[a.index('--output_dir') + 1]\n"
                    "shutil.copy(%r, os.path.join(out, 'gm_cloud.ply'))\n"
                    "shutil.copy(%r, os.path.join(out, 'gm_poses.txt'))\n" % (argv_log, self.ply, self.poses))
        if os.name == "nt":
            self.skipTest("stand-in executable is a POSIX script")
        exe = os.path.join(self.tmp, "rtabmap-export")
        with open(exe, "w", encoding="utf-8") as f:
            f.write("#!%s\nimport runpy\nrunpy.run_path(%r, run_name='__main__')\n" % (sys.executable, fake))
        os.chmod(exe, 0o755)
        out = os.path.join(self.tmp, "n")
        from io import StringIO
        old = sys.stdout
        sys.stdout = StringIO()
        try:
            self.assertEqual(mb.main(["--db", self.db, "--out-dir", out, "--export", "native", "--rtabmap-export", exe,
                                      "--method", "pointcloud", "--workers", "1", "--cell", "0.02",
                                      "--min-points", "3"]), 0)
        finally:
            sys.stdout = old
        with open(argv_log, encoding="utf-8") as f:
            a = json.load(f)
        self.assertEqual(a[:len(mb.EXPORT_ARGS)], mb.EXPORT_ARGS)
        self.assertEqual(a[len(mb.EXPORT_ARGS):len(mb.EXPORT_ARGS) + 2], ["--poses_format", "11"])
        self.assertEqual(a[-1], os.path.abspath(self.db))
        db_sha = hashlib.sha256(rd(self.db)).hexdigest()
        pn = os.path.join(out, "basemap-" + db_sha[:12])
        pp, _ = self.run_tool(os.path.join(self.tmp, "p"))
        self.assertEqual(rd(os.path.join(pn, "cells.bin")), rd(os.path.join(pp, "cells.bin")))
        man = json.loads(rd(os.path.join(pn, "basemap.json")).decode("utf-8"))
        self.assertEqual((man["export"]["mode"], man["export"]["image"]), ("native", None))
        self.assertTrue(man["export"]["command"].startswith("rtabmap-export --opt 0 "))
        self.assertEqual([n for n in os.listdir(out)], ["basemap-" + db_sha[:12]])    # export work dir removed
        with self.assertRaises(SystemExit):
            mb.main(["--db", self.db, "--out-dir", os.path.join(self.tmp, "m"), "--export", "native",
                     "--rtabmap-export", os.path.join(self.tmp, "no-such-exe")])

    def test_parse_cells_rejects_bad_length(self):
        with self.assertRaises(ValueError):
            mb.parse_cells(b"\x00" * 10)


class TsdfPathTests(unittest.TestCase):
    """10-21 --method tsdf：pointcloud 退路与 main 0606d11 逐字节相同；外参修正的已知答案（不需要 open3d）。"""

    # main 0606d11 的 make_basemap 在 FileFormatTests 的同一夹具（scene(RandomState(11))、默认 1 cm / ≥2 点）上的 cells.bin
    MAIN_0606D11_CELLS_SHA = "0c2278a641971539cd757b66edb9b12ad1ba999901fe547fc813513c3776d8f7"

    def test_pointcloud_path_matches_main(self):
        tmp = tempfile.mkdtemp(prefix="gm_basemap_pc_")
        try:
            xyz, rgb = scene(np.random.RandomState(11))
            ply, poses, db = (os.path.join(tmp, n) for n in ("gm_cloud.ply", "gm_poses.txt", "map.db"))
            write_ply(ply, xyz, rgb)
            write_poses(poses, TRAJ)
            with open(db, "wb") as f:
                f.write(b"not really sqlite " * 100)
            from io import StringIO
            for extra in ([], ["--method", "pointcloud"]):
                out = os.path.join(tmp, "o%d" % len(extra))
                old = sys.stdout
                sys.stdout = StringIO()
                try:
                    self.assertEqual(mb.main(["--db", db, "--out-dir", out, "--ply", ply, "--poses", poses,
                                              "--workers", "1"] + extra), 0)
                finally:
                    sys.stdout = old
                pkg = os.path.join(out, os.listdir(out)[0])
                self.assertEqual(hashlib.sha256(rd(os.path.join(pkg, "cells.bin"))).hexdigest(),
                                 self.MAIN_0606D11_CELLS_SHA)
                man = json.loads(rd(os.path.join(pkg, "basemap.json")).decode("utf-8"))
                self.assertEqual((man["method"], man["min_points"], man["params"]["sor_k"]), ("pointcloud", 2, 8))
                self.assertNotIn("tsdf", man)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    @staticmethod
    def local_from_rpy(roll, pitch, yaw=0.0, t=(0.10, 0.0, 0.38)):
        r, p, y = (np.radians(v) for v in (roll, pitch, yaw))
        rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
        m = np.eye(4)
        m[:3, :3] = rz @ ry @ rx @ bt.R_OPT
        m[:3, 3] = t
        return m

    @staticmethod
    def render_floor(k, local, w=640, h=480):
        """平地 z = 0（base 系）在相机里的深度图（mm）。"""
        v, u = np.mgrid[0:h, 0:w].astype(np.float64)
        d = np.stack([(u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1], np.ones_like(u)], -1)
        dz = d @ local[:3, :3][2]
        with np.errstate(divide="ignore", invalid="ignore"):
            s = -local[2, 3] / dz
        s[(dz >= 0) | ~np.isfinite(s) | (s > 4.0)] = 0
        return np.round(s * 1000).astype(np.uint16)

    def test_mount_fix_known_answer(self):
        """真实安装比库里外参多 1° roll、少 0.4° pitch（≈ C §2.3 的量级）：auto 拟合回来，误差 < 0.05°。"""
        k = np.array([[517.17, 0, 324.22], [0, 517.12, 240.65], [0, 0, 1.0]])
        self.assertTrue(np.allclose(bt.mount_rpy_deg(self.local_from_rpy(0.0, 27.3)), [0, 27.3, 0], atol=1e-9))
        truth = self.local_from_rpy(1.0, 26.9)
        nominal = self.local_from_rpy(0.0, 27.3)
        depth = self.render_floor(k, truth)
        r, rep = bt.fit_mount_fix([(depth, k, nominal)] * 12)
        self.assertIsNotNone(r)
        self.assertTrue(rep["applied"])
        self.assertAlmostEqual(rep["delta_roll_deg"], 1.0, delta=0.05)
        self.assertAlmostEqual(rep["delta_pitch_deg"], -0.4, delta=0.05)
        self.assertLess(abs(rep["slope_after_deg"]["x_mean"]) + abs(rep["slope_after_deg"]["y_mean"]), 0.05)
        # 无误差：修正量约 0；误差 5°：超过 3° 不修并报警；帧太少：不修
        r0, rep0 = bt.fit_mount_fix([(self.render_floor(k, nominal), k, nominal)] * 12)
        self.assertLess(rep0["rotation_deg"], 0.02)
        r5, rep5 = bt.fit_mount_fix([(self.render_floor(k, self.local_from_rpy(5.0, 27.3)), k, nominal)] * 12)
        self.assertIsNone(r5)
        self.assertIn("warning", rep5)
        r1, rep1 = bt.fit_mount_fix([(depth, k, nominal)] * 3)
        self.assertIsNone(r1)
        self.assertEqual(rep1["frames_fitted"], 3)


if __name__ == "__main__":
    unittest.main()
