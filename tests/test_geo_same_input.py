"""P8 same-input geometry comparison (tools/geo_same_input): the arms really see the same input.

Checks that the harness's depth conversion and inverse K are bit-identical to what the live loop
and FrameNativeM7 use, that the dump format round-trips, that the comparator catches a one-voxel
difference and refuses vacuous (all-identical) references, and, end to end on a small synthetic
recording, that arm A (CPU reference), arm C (PL backend on MockPlDevice) and arm B (the optimized
C++ bench, when g++ with OpenMP is available) produce identical dumps.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from host.m7_frame_native import FrameNativeM7
from tools.geo_same_input.common import (GRID_ORIGIN_M, GRID_SHAPE, GRID_VOXEL_M, depth_u16_to_m,
                                         f32_hex, inverse_k_f32, meta_depth_unit_m, read_dump,
                                         record_bytes, sha256_file)
from tools.geo_same_input.compare import controls, diff_report
from tools.nano_loop.chain import default_grid, depth_mm_to_m

ROOT = Path(__file__).resolve().parents[1]
K = np.array([[344.781311, 0.0, 322.479797], [0.0, 344.74472, 180.431702], [0.0, 0.0, 1.0]])
NVOX = int(np.prod(GRID_SHAPE))


def synthetic_prep(folder, n=4, h=48, w=64, seed=3, unit_m=0.001):
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:h, 0:w]
    frames = []
    for i in range(n):
        d = (400 + 250 * np.sin(xx / 9. + i) + 120 * np.cos(yy / 7.)).astype(np.uint16)
        d[rng.random((h, w)) < 0.25] = 0
        frames.append(d)
    np.stack(frames).astype("<u2").tofile(os.path.join(folder, "depth_u16.bin"))
    k = K.copy()
    k[0, 2], k[1, 2] = w / 2 - .5, h / 2 - .5
    meta = {"width": w, "height": h, "frames": n, "depth_file": "depth_u16.bin",
            "depth_sha256": sha256_file(os.path.join(folder, "depth_u16.bin")),
            "K": k.tolist(), "inverse_k_f32_hex": f32_hex(inverse_k_f32(k)),
            "grid": {"origin_m": list(GRID_ORIGIN_M), "voxel_m": GRID_VOXEL_M, "shape_xyz": list(GRID_SHAPE)},
            "color_index": list(range(n)), "acquired_ns": [10 ** 18 + i for i in range(n)],
            "calibration_id": "synthetic"}
    if unit_m is not None:
        meta["depth_unit_m"] = unit_m
    json.dump(meta, open(os.path.join(folder, "meta.json"), "w"))
    return meta


class _Recorder:
    """Stands in for the ctypes library and records what FrameNativeM7 passes to the core."""

    def fpga_m7_depth_frame(self, depth, inverse_k, world_camera, origin, voxel, width, height,
                           nx, ny, nz, flags, counts, status):
        self.inverse_k, self.origin, self.voxel = inverse_k.copy(), origin.copy(), voxel
        flags[:] = 0
        counts[:] = [width * height, 0, 0]
        status[0] = 0


class SameInputTest(unittest.TestCase):
    def test_depth_conversion_bit_identical_to_live_loop(self):
        rng = np.random.default_rng(0)
        d = rng.integers(0, 12000, size=(37, 53)).astype(np.uint16)
        d[rng.random(d.shape) < .3] = 0
        a, b = depth_u16_to_m(d), depth_mm_to_m(d)
        self.assertEqual(a.dtype, b.dtype)
        self.assertTrue(np.array_equal(np.isnan(a), np.isnan(b)))
        self.assertEqual(a.tobytes(), b.tobytes())

    def test_depth_unit_comes_from_meta_and_is_bitwise_f32_multiply(self):
        codes = np.arange(65536, dtype=np.uint16).reshape(256, 256)
        for unit in (0.001, 0.0002):
            got = depth_u16_to_m(codes, unit)
            want = codes.astype(np.float32) * np.float32(unit)
            want[codes == 0] = np.nan
            self.assertEqual(got.tobytes(), want.tobytes())
        # the 0.001 default on 1/5000 m data is 5x off and must be visibly different
        self.assertNotEqual(depth_u16_to_m(codes, 0.0002).tobytes(), depth_u16_to_m(codes).tobytes())
        self.assertEqual(meta_depth_unit_m({"depth_unit_m": 0.0002}), (0.0002, "meta"))
        self.assertEqual(meta_depth_unit_m({}, 0.001), (0.001, "override"))
        with self.assertRaises(ValueError):
            meta_depth_unit_m({})                       # no silent default
        with self.assertRaises(ValueError):
            meta_depth_unit_m({"depth_unit_m": 0.0002}, 0.001)   # disagreement is an error
        with self.assertRaises(ValueError):
            depth_u16_to_m(codes, 0.0)

    def test_py_arm_reads_unit_from_meta(self):
        if shutil.which("g++") is None:
            self.skipTest("g++ required")
        env = dict(os.environ, PYTHONPATH=str(ROOT))
        arm = str(ROOT / "tools/geo_same_input/py_arm.py")
        with tempfile.TemporaryDirectory() as t:
            dumps = {}
            for tag, unit in (("u1", 0.001), ("u5", 0.0002)):
                prep = os.path.join(t, "prep_" + tag)
                os.makedirs(prep)
                synthetic_prep(prep, unit_m=unit)
                subprocess.run([sys.executable, arm, "--prep", prep, "--out", os.path.join(t, tag),
                                "--arm", "a", "--passes", "1", "--warmup", "0"],
                               check=True, capture_output=True, env=env)
                s = json.load(open(os.path.join(t, tag, "summary.json")))
                self.assertEqual((s["depth_unit_m"], s["depth_unit_source"]), (unit, "meta"))
                dumps[tag] = read_dump(os.path.join(t, tag, "flags.dump"), NVOX)
            # same codes, different unit -> different grids (the unit really reached the core)
            self.assertFalse(diff_report(*dumps["u1"], *dumps["u5"])["equal"])
            prep = os.path.join(t, "prep_none")
            os.makedirs(prep)
            synthetic_prep(prep, unit_m=None)
            bad = subprocess.run([sys.executable, arm, "--prep", prep, "--out", os.path.join(t, "none"),
                                  "--arm", "a", "--passes", "1", "--warmup", "0"],
                                 capture_output=True, text=True, env=env)
            self.assertNotEqual(bad.returncode, 0)
            self.assertIn("depth_unit_m", bad.stderr)
            self.assertFalse(os.path.exists(os.path.join(t, "none")))

    def test_grid_and_inverse_k_are_what_the_product_passes(self):
        g = default_grid()
        self.assertEqual((tuple(g.origin_m), g.voxel_m, tuple(g.shape)), (GRID_ORIGIN_M, GRID_VOXEL_M, GRID_SHAPE))
        backend = FrameNativeM7.__new__(FrameNativeM7)
        backend.lib = _Recorder()
        backend.frame_from_depth(depth_m=np.full((4, 5), .5, np.float32), k=K, camera_to_world=np.eye(4),
                                 grid_origin_m=GRID_ORIGIN_M, voxel_size_m=GRID_VOXEL_M, shape=GRID_SHAPE,
                                 frame_id=1, capture_time_ns=1, pose_time_ns=1, calibration_version="c",
                                 pose_confirmed=True)
        self.assertEqual(backend.lib.inverse_k.tobytes(), inverse_k_f32(K).ravel().tobytes())
        self.assertEqual([float.fromhex(v) for v in f32_hex(inverse_k_f32(K))],
                         [float(v) for v in inverse_k_f32(K).ravel()])

    def test_dump_roundtrip_and_comparator(self):
        rng = np.random.default_rng(1)
        flags = rng.integers(0, 3, size=(5, NVOX)).astype(np.uint8)
        counts = rng.integers(0, 1000, size=(5, 3)).astype(np.uint32)
        with tempfile.TemporaryDirectory() as t:
            p = os.path.join(t, "x.dump")
            with open(p, "wb") as fp:
                for c, f in zip(counts, flags):
                    fp.write(record_bytes(c, f))
            rc, rf = read_dump(p, NVOX)
        self.assertTrue(np.array_equal(rc, counts) and np.array_equal(rf, flags))
        self.assertTrue(diff_report(counts, flags, counts, flags.copy())["equal"])
        bad = flags.copy()
        bad[2, 5 + 32 * 32 * 7] = (bad[2, 5 + 32 * 32 * 7] + 1) % 3
        rep = diff_report(counts, flags, counts, bad)
        self.assertFalse(rep["equal"])
        self.assertEqual((rep["voxels_diff_total"], rep["frames_with_voxel_diff"]), (1, 1))
        self.assertEqual(rep["z_layer_histogram"], {7: 1})
        badc = counts.copy()
        badc[4, 0] += 1
        self.assertFalse(diff_report(counts, flags, badc, flags)["equal"])
        ok, _ = controls(counts, flags)
        self.assertTrue(ok)
        same = np.repeat(flags[:1], 5, axis=0)            # every frame identical: vacuous reference
        ok, ctl = controls(counts, same)
        self.assertFalse(ok)
        self.assertEqual(ctl["distinct_reference_frames"], 1)

    def test_arms_identical_on_synthetic_recording(self):
        if shutil.which("g++") is None:
            self.skipTest("g++ required")
        with tempfile.TemporaryDirectory() as t:
            prep = os.path.join(t, "prep")
            os.makedirs(prep)
            meta = synthetic_prep(prep)
            env = dict(os.environ, PYTHONPATH=str(ROOT))
            arm = str(ROOT / "tools/geo_same_input/py_arm.py")
            for name, extra in (("A", ["--arm", "a"]), ("C", ["--arm", "c", "--device", "mock"])):
                subprocess.run([sys.executable, arm, "--prep", prep, "--out", os.path.join(t, name),
                                "--passes", "1", "--warmup", "0"] + extra,
                               check=True, capture_output=True, env=env)
            s = json.load(open(os.path.join(t, "C", "summary.json")))
            self.assertEqual(s["backend_name"], "pl:mock")
            self.assertTrue(s["inverse_k_matches_prep"])
            ac, af = read_dump(os.path.join(t, "A", "flags.dump"), NVOX)
            cc, cf = read_dump(os.path.join(t, "C", "flags.dump"), NVOX)
            self.assertEqual(af.shape[0], 4)
            self.assertGreater(int((af == 2).sum()), 0)
            self.assertTrue(diff_report(ac, af, cc, cf)["equal"])
            # arm B: the optimized C++ bench with the exact inverse K, product grid, 3 OpenMP threads
            b = os.path.join(t, "bench")
            m = ROOT / "hw/m7"
            try:
                subprocess.run(["g++", "-std=c++11", "-O2", "-ffp-contract=off", "-c", str(m / "frame_depth.cpp"),
                                "-o", b + "_fd.o"], check=True, capture_output=True)
                subprocess.run(["g++", "-std=c++11", "-O2", "-ffp-contract=off", "-c", str(m / "object_evidence.cpp"),
                                "-o", b + "_oe.o"], check=True, capture_output=True)
                subprocess.run(["g++", "-std=c++14", "-O2", "-fopenmp", "-ffp-contract=off", "-I" + str(m),
                                str(ROOT / "tools/nano_baseline/m7_cpu_bench.cpp"), b + "_fd.o", b + "_oe.o", "-o", b],
                               check=True, capture_output=True)
            except subprocess.CalledProcessError:
                self.skipTest("bench does not build here (OpenMP?)")
            k = meta["K"]
            out = subprocess.run([b, "--depth", os.path.join(prep, "depth_u16.bin"), "--width", "64", "--height", "48",
                                  "--frames", "4", "--fx", repr(k[0][0]), "--fy", repr(k[1][1]), "--cx", repr(k[0][2]),
                                  "--cy", repr(k[1][2]), "--unit-m", "0.001", "--grid", "p32",
                                  "--ik", ",".join(meta["inverse_k_f32_hex"]), "--no-object", "--arm", "opt",
                                  "--passes", "1", "--warmup", "0", "--dump", os.path.join(t, "B.dump")],
                                 capture_output=True, text=True, env=dict(os.environ, OMP_NUM_THREADS="3"))
            self.assertEqual(out.returncode, 0, out.stderr)
            bc, bf = read_dump(os.path.join(t, "B.dump"), NVOX)
            self.assertTrue(diff_report(ac, af, bc, bf)["equal"])
            self.assertIn('"opt_mismatches": 0', out.stdout)


if __name__ == "__main__":
    unittest.main()
