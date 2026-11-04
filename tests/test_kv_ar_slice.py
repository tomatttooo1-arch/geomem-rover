"""AR mini map for the car producer's 32 x 32 x nz odom block (host/kv_ar.py, 2026-10-12).

Before: fill_ar_inputs drew the mini map only for a 32^3 grid, so every car frame (producer
block 32 x 32 x 10, odom, z up) showed the hatched (fail-safe-looking) mini map. Now a
32 x 32 x nz block (nz < 32) is read as z up: the horizontal layer through the camera centre,
plan view (odom x right, odom y up), packed into the core's 32^3 buffer with permuted axes.

Checked here:
  1. 32^3 input: fill_ar_inputs of d6735da (git show) and of HEAD give identical registers,
     grid and descriptor bytes, and identical golden pictures (several frames: live, geometry
     failure, camera outside the grid, NaN pose, another grid shape, replay without age);
  2. 32 x 32 x 10: an oracle written here from the flags alone (camera layer, flipud, 4x
     upscale, white camera cell) equals the golden picture's mini map region pixel by pixel,
     and the C reference of the core (tools/ar_display/check_golden + hw/m7/ar_overlay.cpp)
     equals the golden picture as a whole; known answer layer 7 (= DEFAULT_HEIGHT_CELL) on the
     replay rig's mount (block from FollowCameraGrid, not written by hand) and on a mid-cell
     camera height; a camera in layer 3; a camera above the block (clamped to the top layer)
     and outside it in x (no camera cell drawn);
  3. positive controls: geometry failure and an over-age frame still give the hatched mini
     map; text mutants of host/kv_ar.py (branch removed, no axis permutation, x mirrored,
     layer along y, camera row from z) are each caught by the oracle of 2.

Run: PYTHONHASHSEED=0 python3 -m unittest discover -s tests -p 'test_kv_ar_slice.py'
(also /usr/bin/python3.8).
"""
import math
import os
import subprocess
import sys
import tempfile
import types
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for p in (str(ROOT), str(ROOT / "tests"), str(ROOT / "tools" / "ar_display")):
    if p not in sys.path:
        sys.path.insert(0, p)

from host import kv_ar  # noqa: E402
from host.geo_producer import DEFAULT_HEIGHT_CELL, DEFAULT_SHAPE, VOXEL_M  # noqa: E402
from host.m7_grid_policy import FollowCameraGrid  # noqa: E402
import golden  # noqa: E402
import check_golden  # noqa: E402
import test_kv_ar as base  # noqa: E402

PAL = {0: 0x282828, 1: 0x2E6B3A, 2: 0xFF3030}
STRIPE = (0xA000A0, 0x180018)


def rig_camera():
    """T_odom_camera of the producer rig (tools/car_geo_producer/common: base_camera mount,
    odom_base at k=0) -- the replay the integrated chain runs."""
    from tools.car_geo_producer.common import base_camera, odom_base
    return odom_base(0) @ base_camera()


def odom_flags(nz=10, seed=0):
    """[z][y][x] flags, every layer different (a wrong layer cannot pass), x/y asymmetric
    (a transposed or mirrored plan cannot pass), one code >= 3 (magenta)."""
    rng = np.random.RandomState(seed)
    g = rng.randint(0, 3, size=(nz, 32, 32)).astype(np.uint8)
    for z in range(nz):
        g[z, :, :3 + z] = 2                  # hit band in low x, width depends on the layer
        g[z, 25:, 10:14] = 1                 # free patch in high y only
        g[z, (4 + z) % 32, 20] = 7                  # one odd code, row depends on the layer
    return g


def odom_snap(T, *, nz=10, origin=None, status=0, flags=None, nano_elapsed_ns=4000000):
    """A producer frame: block from FollowCameraGrid unless ``origin`` is given."""
    shape = (32, 32, nz)
    if origin is None:
        origin = FollowCameraGrid(VOXEL_M, shape, height_cell=min(DEFAULT_HEIGHT_CELL, nz - 2)).grid_for(T).origin_m
    snap = base.synth_snap(dims=shape, status=status, nano_elapsed_ns=nano_elapsed_ns)
    snap["origin"] = np.array(origin, dtype=np.float64)
    snap["voxel"] = VOXEL_M
    snap["world_camera"] = np.asarray(T, dtype=np.float64)[:3, :4].astype(np.float32).ravel()
    f = odom_flags(nz) if flags is None else flags
    snap["flags"] = f.ravel() if status == 0 else np.full(32 * 32 * nz, 99, np.uint8)
    return snap, f


def cam_pose(x, y, z):
    T = rig_camera().copy()
    T[:3, 3] = (x, y, z)
    return T


def oracle_mini(snap, flags):
    """Expected 128 x 128 mini map of a fresh frame, from the flags and the request alone:
    the layer containing the camera centre (float32 translation as sent, float64 origin),
    clamped; rows = odom y with y = 31 at the top, columns = odom x; 4 px per cell; the
    camera cell white when the camera's x and y cells are inside the block."""
    wc = np.asarray(snap["world_camera"], np.float32).astype(np.float64)
    cam = (wc[3], wc[7], wc[11])
    o, v = snap["origin"], snap["voxel"]
    cell = [int(math.floor((cam[k] - o[k]) / v)) for k in range(3)]
    nz = flags.shape[0]
    layer = min(max(cell[2], 0), nz - 1)
    plan = flags[layer]                                   # [y][x]
    rgb = np.vectorize(lambda c: PAL.get(int(c), 0xFF00FF), otypes=[np.uint32])(plan)
    img = np.kron(rgb[::-1, :], np.ones((4, 4), np.uint32)).astype(np.uint32)
    if 0 <= cell[0] < 32 and 0 <= cell[1] < 32:
        r = (31 - cell[1]) * 4
        img[r:r + 4, cell[0] * 4:cell[0] * 4 + 4] = 0xFFFFFF
    lo = int(round((o[2] + v * layer) * 1000))
    hi = int(round((o[2] + v * (layer + 1)) * 1000))
    return img, layer, golden.slice_label(lo, hi)


def render_regs(inp, age_ms=7, permit=1):
    """The registers base.render hands the golden (age written, permit wire value)."""
    regs = list(inp["regs"])
    regs[8] = age_ms if regs[9] else 0
    regs[15] = permit
    return regs


def mini_of(fb, inp):
    r = dict(zip(golden.REG_NAMES, inp["regs"]))
    return fb[r["mini_y"]:r["mini_y"] + 128, r["mini_x"]:r["mini_x"] + 128]


def hatched(mini, inp):
    r = dict(zip(golden.REG_NAMES, inp["regs"]))
    yy, xx = np.mgrid[0:128, 0:128]
    want = np.where(((xx + r["mini_x"] + yy + r["mini_y"]) % 8) < 2, STRIPE[0], STRIPE[1])
    return bool(np.array_equal(mini, want))


MUTANTS = {
    # anchor follows kv_ar.py's grid_ok line (2026-10-17: + "not no_geo", AR_DEPTH_INVALID_STALL_20261017)
    "branch_removed": ("grid_ok = ok and not no_geo and (dims == (GRID_N, GRID_N, GRID_N) or odom_block)",
                       "grid_ok = ok and not no_geo and dims == (GRID_N, GRID_N, GRID_N)"),
    "no_axis_permutation": [("g3[:, :nz, :] = np.asarray", "g3[:nz, :, :] = np.asarray"),
                            ("nz, GRID_N, GRID_N).transpose(1, 0, 2)", "nz, GRID_N, GRID_N)")],
    "mirrored_x": ("nz, GRID_N, GRID_N).transpose(1, 0, 2)", "nz, GRID_N, GRID_N).transpose(1, 0, 2)[:, :, ::-1]"),
    "layer_along_y": ("slice_y = min(max(czi, 0), nz - 1)", "slice_y = min(max(cyi, 0), nz - 1)"),
    "camera_row_from_z": ("cz = cyi if 0 <= cyi < GRID_N else GRID_N", "cz = czi if 0 <= czi < GRID_N else GRID_N"),
}


def mutant(name):
    edits = MUTANTS[name]
    src = open(ROOT / "host/kv_ar.py").read()
    for old, new in (edits if isinstance(edits, list) else [edits]):
        if src.count(old) != 1:
            raise AssertionError("mutant %s: anchor hits %d times" % (name, src.count(old)))
        src = src.replace(old, new)
    mod = types.ModuleType("kv_ar_slice_" + name)
    mod.__file__ = str(ROOT / "host/kv_ar.py")
    exec(compile(src, "kv_ar[%s]" % name, "exec"), mod.__dict__)
    return mod


class Shared:
    _lib = None

    @classmethod
    def c_lib(cls):
        if cls._lib is None:
            cls._tmp = tempfile.TemporaryDirectory(prefix="test_kv_ar_slice_")
            cls._lib = check_golden.build(cls._tmp.name, "slice_ref")
        return cls._lib


class CubeUnchangedTests(unittest.TestCase):
    """1. 32^3 frames: HEAD == d6735da, bytes and pixels."""

    def test_cube_inputs_and_pictures_equal_d6735da(self):
        old = base.load_old("host/kv_ar.py", "d6735da")
        cases = [("live", {}, "live"), ("geomfail", {"status": 3}, "live"),
                 ("outside", {"wc_t": (5.0, -3.0, -0.4)}, "live"),
                 ("nan_pose", {"wc_t": (float("nan"), 0, 0)}, "live"),
                 ("cam_low", {"wc_t": (-0.79, 0.74, 0.01)}, "live"),
                 ("other_shape", {"dims": (16, 32, 32)}, "live"),
                 ("no_age", {"nano_elapsed_ns": None}, "live"),
                 ("replay_no_age", {"nano_elapsed_ns": None}, "replay")]
        for name, kw, source in cases:
            with self.subTest(case=name):
                snap = base.synth_snap(**kw)
                lay = base.layout()
                new = kv_ar.fill_ar_inputs(snap, lay, source=source, max_age_ms=200)
                was = old.fill_ar_inputs(snap, lay, source=source, max_age_ms=200)
                self.assertEqual(list(new["regs"]), list(was["regs"]))
                self.assertEqual(new["grid"].tobytes(), was["grid"].tobytes())
                self.assertEqual(new["desc"].tobytes(), was["desc"].tobytes())
                self.assertEqual(new["depth"].tobytes(), was["depth"].tobytes())
                self.assertEqual(new["age_basis_ns"], was["age_basis_ns"])
                sn, fn, _ = base.render(new)
                so, fo, _ = base.render(was)
                self.assertEqual(sn, so)
                self.assertTrue(np.array_equal(fn, fo))

    def test_cube_layer_still_camera_y(self):
        # rec30 preprocessing: origin (-0.8, -0.8, -0.05), camera at the origin of its own
        # frame -> layer 16, label +0000..+0050 (board screenshot A2_frame400.png)
        snap = base.synth_snap(wc_t=(0.0, 0.0, 0.0))
        inp = kv_ar.fill_ar_inputs(snap, base.layout(), source="live")
        r = dict(zip(golden.REG_NAMES, inp["regs"]))
        self.assertEqual((r["slice_y"], r["grid_valid"]), (16, 1))
        _s, _f, info = base.render(inp)
        self.assertEqual(info["slice_label"], "Y+0000..+0050MM")


class OdomBlockTests(unittest.TestCase):
    """2. 32 x 32 x nz: plan view through the camera layer, oracle + golden + C reference."""

    def check(self, snap, flags, module=kv_ar, max_age_ms=200):
        lay = base.layout()
        inp = module.fill_ar_inputs(snap, lay, source="live", max_age_ms=max_age_ms)
        st, fb, info = base.render(inp)
        self.assertEqual((st, info["mode"]), (0, 0))
        want, layer, label = oracle_mini(snap, flags)
        return inp, fb, info, want, layer, label

    def assert_matches(self, snap, flags):
        inp, fb, info, want, layer, label = self.check(snap, flags)
        got = mini_of(fb, inp)
        self.assertEqual(int((got != want).sum()), 0)
        self.assertEqual(info["slice_label"], label)
        cst, cfb = check_golden.run_c(Shared.c_lib(), {"depth": inp["depth"], "grid": inp["grid"],
                                                      "desc": inp["desc"], "regs": render_regs(inp)})
        W, H = inp["regs"][0], inp["regs"][1]
        self.assertEqual(cst, 0)
        self.assertTrue(np.array_equal(cfb[:W * H].reshape(H, W), fb))
        return inp, layer, info

    def test_rig_mount_layer_is_height_cell(self):
        T = rig_camera()
        snap, flags = odom_snap(T)
        self.assertEqual(DEFAULT_SHAPE, (32, 32, 10))
        inp, layer, info = self.assert_matches(snap, flags)
        r = dict(zip(golden.REG_NAMES, inp["regs"]))
        # FollowCameraGrid: camera cell z = height_cell (block bottom = (cam cell - 7) x 5 cm)
        self.assertEqual((layer, r["slice_y"], r["grid_valid"]), (DEFAULT_HEIGHT_CELL, 7, 1))
        anchor = FollowCameraGrid(VOXEL_M, DEFAULT_SHAPE, height_cell=7).anchor_for(T[:3, :3])
        self.assertEqual((r["cam_cell_x"], r["cam_cell_z"]), anchor[:2])   # x cell, y cell
        self.assertEqual(info["slice_label"], "Y+0300..+0350MM")
        # this mount puts the camera centre exactly on a 5 cm face (z = 0.35 m): float32 on the
        # wire lands it in layer 7; the mid-cell case below has no such boundary

    def test_mid_cell_camera_layer_7(self):
        T = cam_pose(1.234, -0.517, 0.37)
        snap, flags = odom_snap(T)
        inp, layer, info = self.assert_matches(snap, flags)
        self.assertEqual(layer, 7)
        self.assertEqual(info["slice_label"], "Y+0350..+0400MM")
        r = dict(zip(golden.REG_NAMES, inp["regs"]))
        self.assertEqual((r["slice_lo_mm"], r["slice_hi_mm"]), (350, 400))

    def test_camera_in_layer_3(self):
        T = cam_pose(0.42, 0.61, 0.17)
        snap, flags = odom_snap(T, origin=(0.0, 0.0, 0.0))
        inp, layer, info = self.assert_matches(snap, flags)
        r = dict(zip(golden.REG_NAMES, inp["regs"]))
        self.assertEqual((layer, r["slice_y"], r["cam_cell_x"], r["cam_cell_z"]), (3, 3, 8, 12))
        self.assertEqual(info["slice_label"], "Y+0150..+0200MM")

    def test_camera_above_and_outside_block(self):
        T = cam_pose(2.5, 0.61, 0.9)
        snap, flags = odom_snap(T, origin=(0.0, 0.0, 0.0))
        inp, layer, info = self.assert_matches(snap, flags)
        r = dict(zip(golden.REG_NAMES, inp["regs"]))
        self.assertEqual((layer, r["slice_y"], r["cam_cell_x"], r["cam_cell_z"]), (9, 9, 32, 12))
        self.assertEqual(info["slice_label"], "Y+0450..+0500MM")

    def test_other_depths(self):
        for nz in (3, 16, 31):
            with self.subTest(nz=nz):
                T = cam_pose(0.8, 0.8, 0.12)
                snap, flags = odom_snap(T, nz=nz, origin=(0.0, 0.0, 0.0))
                _inp, layer, _info = self.assert_matches(snap, flags)
                self.assertEqual(layer, min(2, nz - 1))

    def test_plan_orientation_known_cells(self):
        """One hit cell at odom (x=3, y=29) in the camera layer, everything else free: on the
        screen it is near the left edge and near the top (x right, y up -- not mirrored)."""
        T = cam_pose(0.82, 0.13, 0.37)
        flags = np.ones((10, 32, 32), np.uint8)
        flags[7, 29, 3] = 2
        snap, _ = odom_snap(T, origin=(0.0, 0.0, 0.0), flags=flags)
        inp, fb, _info, want, layer, _ = self.check(snap, flags)
        got = mini_of(fb, inp)
        self.assertEqual(layer, 7)
        ys, xs = np.nonzero(got == 0xFF3030)
        self.assertEqual((ys.min(), ys.max(), xs.min(), xs.max()), (8, 11, 12, 15))   # row 31-29, col 3
        ys, xs = np.nonzero(got == 0xFFFFFF)
        self.assertEqual((ys.min(), xs.min()), ((31 - 2) * 4, 16 * 4))                 # camera x 16, y 2
        self.assertTrue(np.array_equal(got, want))


class PositiveControls(unittest.TestCase):
    """3. fail-safe pictures keep the hatch; mutants of the new branch are caught."""

    def test_failsafe_hatched(self):
        T = rig_camera()
        lay = base.layout()
        snap, _ = odom_snap(T, status=3)
        inp = kv_ar.fill_ar_inputs(snap, lay, source="live")
        _s, fb, info = base.render(inp)
        self.assertEqual(info["mode"], 1)
        self.assertTrue(hatched(mini_of(fb, inp), inp))
        snap, _ = odom_snap(T)
        inp = kv_ar.fill_ar_inputs(snap, lay, source="live", max_age_ms=3)   # age 7 > 3
        _s, fb, info = base.render(inp)
        self.assertEqual(info["mode"], 2)
        self.assertTrue(hatched(mini_of(fb, inp), inp))
        # the fresh frame is not hatched (the check above can tell the two apart)
        inp = kv_ar.fill_ar_inputs(snap, lay, source="live")
        _s, fb, _i = base.render(inp)
        self.assertFalse(hatched(mini_of(fb, inp), inp))

    def test_mutants_caught(self):
        cases = [(rig_camera(), None), (cam_pose(0.42, 0.61, 0.17), (0.0, 0.0, 0.0))]
        for name in [None] + sorted(MUTANTS):
            with self.subTest(mutant=name):
                mod = kv_ar if name is None else mutant(name)
                caught = False
                for T, origin in cases:
                    snap, flags = odom_snap(T, origin=origin)
                    inp = mod.fill_ar_inputs(snap, base.layout(), source="live")
                    _s, fb, info = base.render(inp)
                    want, _layer, label = oracle_mini(snap, flags)
                    if not np.array_equal(mini_of(fb, inp), want) or info["slice_label"] != label:
                        caught = True
                # the unmutated module passes the same check (the check is not always true)
                self.assertEqual(caught, name is not None, "mutant %s" % name)


if __name__ == "__main__":
    unittest.main()
