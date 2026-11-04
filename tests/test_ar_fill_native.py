"""The AR sender's depth chain in C++ (host/kv_ar_native.py, tools/kv_link/ar_helper/m7ar_fill.cpp,
2026-10-16; docs/design/AR_COLOUR_MAIN_20261016.md section 13.2).

  1. bit-for-bit: fill_ar_inputs(native=lib) == fill_ar_inputs() (the numpy reference) on every key
     of the result (depth, regs, grid, desc, the rest), over: float32 boundaries (exactly 0.1 m /
     10 m, 0.1001, 9.9995, dense codes around them), NaN / +-Inf / negative / overflowing units, no
     valid pixel, k in {0, 1, 4.5, 6, -1, 65535}, 640x360 and 1280x800, a stride-2 upsampled
     snapshot, real replay frames (rec30c prep, its own unit; FPGA_AR_FILL_PREP or the host default,
     skipped when absent); plus the library's valid count / nearest mm against the reference.
  2. positive controls: the three compile-time mutants (1 stretch without k, 2 truncate instead of
     round, 3 validity compared in double) each differ from the reference on this input set.
  3. outside the native domain (non-uint16 codes, too large / empty, short codes, NaN / huge k) the
     reference runs: same result or the same exception, counted in counts.
Run it on the KV260 too (board g++ and numpy): an FMA or another compare rule shows only there.
"""
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from host import kv_ar  # noqa: E402
from host.kv_ar_native import ArFillNative  # noqa: E402
from host.kv_link import f32_bits  # noqa: E402
from host.m7_display_native import quantize_optical_z  # noqa: E402
from host.kv_link import bits_f32, codes_to_m  # noqa: E402

HOST_PREP = "${FPGA_SRC}/runs/ar-screen-20261016-colour/prep_rec30c"
KS = (1.0, 6.0, 4.5, 0.0, -1.0, 65535.0)


def bits(v):
    return int(np.array([v], dtype=np.float32).view(np.uint32)[0])


def snap_of(codes, unit_bits, W, H):
    return {"frame_id": 7, "codes": codes, "unit_bits": unit_bits, "width": W, "height": H,
            "nx": 32, "ny": 32, "nz": 32, "origin": [-0.8, -0.8, -0.05], "voxel": 0.05,
            "world_camera": np.eye(4)[:3], "flags": np.random.RandomState(1).randint(0, 3, 32 ** 3).astype(np.uint8),
            "geometry_status": 0, "roi": None, "roi_status": None, "roi_rows": [],
            "nano_elapsed_ns": 5000000, "labels": None, "replay": True, "first_mono_ns": 0,
            "capture_ns": 0, "sensor_stamp_ns": None}


def same(a, b):
    """Equal result dicts: same keys; arrays equal in dtype, shape and bytes; the rest ==."""
    if set(a) != set(b):
        return False
    for k in a:
        x, y = a[k], b[k]
        if isinstance(x, np.ndarray) or isinstance(y, np.ndarray):
            x, y = np.asarray(x), np.asarray(y)
            if x.dtype != y.dtype or x.shape != y.shape or x.tobytes() != y.tobytes():
                return False
        elif x != y:
            return False
    return True


def tile(values, W, H):
    v = np.asarray(values, dtype=np.uint16)
    return np.resize(v, W * H).astype(np.uint16)


def synthetic_cases():
    """(name, codes u16[W*H], unit_bits, W, H)."""
    rs = np.random.RandomState(20261016)
    W, H = 640, 360
    u1 = bits(0.001)
    out = []
    # code 100 x 0.001f == float32(0.1) exactly; 10000 x 0.001f == 10.0f exactly
    out.append(("mm_boundaries", tile([0, 99, 100, 101, 102, 9998, 9999, 10000, 10001, 65535, 500, 1234], W, H), u1, W, H))
    out.append(("mm_random", rs.randint(0, 12000, W * H).astype(np.uint16), u1, W, H))
    # 0.1001 (unit 1e-4: code 1001) and 9.9995 (unit 2.5e-4: code 39998), dense codes around them
    out.append(("m_0p1001", tile(np.arange(990, 1012), W, H), bits(0.0001), W, H))
    out.append(("m_9p9995", tile(np.arange(39990, 40010), W, H), bits(0.00025), W, H))
    # units chosen so that given codes land next to the boundaries, many units
    for i, b in enumerate((0.1, 10.0, 0.1001, 9.9995, 0.1005, 9.9985)):
        c0 = int(rs.randint(200, 60000))
        u = bits(b / c0)
        out.append(("dense_%d" % i, tile(np.arange(c0 - 40, c0 + 40), W, H), u, W, H))
    # random units across the range
    for i in range(6):
        u = bits(10 ** rs.uniform(-5, -3))
        out.append(("unit_rand_%d" % i, rs.randint(0, 65536, W * H).astype(np.uint16), u, W, H))
    # NaN / +-Inf / negative / overflow (float32 code * unit = inf) / zero / subnormal unit
    any_codes = rs.randint(0, 65536, W * H).astype(np.uint16)
    for name, u in (("unit_nan", 0x7FC00000), ("unit_nan_payload", 0x7FA00001), ("unit_inf", 0x7F800000),
                    ("unit_ninf", 0xFF800000), ("unit_neg", bits(-0.001)), ("unit_negzero", 0x80000000),
                    ("unit_zero", 0), ("unit_huge", bits(1e37)), ("unit_subnormal", 1)):
        out.append((name, any_codes, u, W, H))
    # no valid pixel
    out.append(("all_zero", np.zeros(W * H, np.uint16), u1, W, H))
    out.append(("all_out_of_range", tile([50, 99, 100, 10000, 20000], W, H), u1, W, H))
    # one valid pixel
    one = np.zeros(W * H, np.uint16)
    one[12345] = 4321
    out.append(("one_valid", one, u1, W, H))
    # 1280 x 800 and a tiny frame
    out.append(("big_1280x800", rs.randint(0, 12000, 1280 * 800).astype(np.uint16), u1, 1280, 800))
    out.append(("tiny_1x1", np.array([777], np.uint16), u1, 1, 1))
    # codes longer than W x H (fill reads the first W x H)
    out.append(("long_codes", rs.randint(0, 12000, W * H + 999).astype(np.uint16), u1, W, H))
    return out


def upsampled_case():
    rs = np.random.RandomState(7)
    s = snap_of(rs.randint(0, 12000, 320 * 180).astype(np.uint16), bits(0.001), 320, 180)
    up, k = kv_ar.upsample_snapshot(s, (640, 360))
    assert k == 2 and (up["width"], up["height"]) == (640, 360)
    return ("stride2_upsampled", up["codes"], up["unit_bits"], 640, 360)


def real_cases(n=24):
    prep = os.environ.get("FPGA_AR_FILL_PREP", HOST_PREP)
    meta_p = os.path.join(prep, "meta.json")
    if not os.path.exists(meta_p):
        return None
    import json
    meta = json.load(open(meta_p))
    W, H, frames = int(meta["width"]), int(meta["height"]), int(meta["frames"])
    raw = np.memmap(os.path.join(prep, meta.get("depth_file", "depth_u16.bin")), dtype="<u2", mode="r")
    u = f32_bits(meta["depth_unit_m"])
    step = max(1, frames // n)
    return [("real_%03d" % i, np.array(raw[i * W * H:(i + 1) * W * H]), u, W, H)
            for i in range(0, frames, step)][:n]


class ArFillNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="ar_fill_")
        cls.lib = ArFillNative.build(cls.tmp)
        cls.mutants = {m: ArFillNative.build(cls.tmp, mutant=m, name="mut%d.so" % m) for m in (1, 2, 3)}
        cls.lay = kv_ar.ar_layout(1024, 600, 640, 360)
        cls.cases = synthetic_cases() + [upsampled_case()]
        cls.real = real_cases()

    def layout(self, W, H):
        return kv_ar.ar_layout(1280, 800, W, H) or self.lay

    def compare(self, lib, case, k):
        name, codes, u, W, H = case
        s = snap_of(codes, u, W, H)
        lay = self.layout(W, H)
        ref = kv_ar.fill_ar_inputs(s, lay, source="replay", depth_stretch=k)
        got = kv_ar.fill_ar_inputs(s, lay, source="replay", depth_stretch=k, native=lib)
        return ref, got

    def check_stats(self, case, k):
        name, codes, u, W, H = case
        before = self.lib.counts["native"]
        self.lib.depth_u16(codes, u, W, H, k)
        self.assertEqual(self.lib.counts["native"], before + 1, name)
        mm, count, _ = quantize_optical_z(codes_to_m(codes, bits_f32(u))[:W * H].reshape(H, W))
        nz = mm[mm != 0]
        self.assertEqual(self.lib.last_stats, (count, int(nz.min()) if nz.size else 0), name)

    def test_equivalent_synthetic(self):
        for case in self.cases:
            for k in KS:
                ref, got = self.compare(self.lib, case, k)
                self.assertTrue(same(ref, got), "%s k=%r" % (case[0], k))
            self.check_stats(case, 6.0)
        # the cases really span both sides of the boundaries
        name, codes, u, W, H = self.cases[0]
        d = kv_ar.fill_ar_inputs(snap_of(codes, u, W, H), self.lay, source="replay")["depth"]
        self.assertEqual(sorted(set(d[:12].tolist())), [0, 101, 102, 500, 1234, 9998, 9999])

    def test_equivalent_real_frames(self):
        if self.real is None:
            self.skipTest("no prep (FPGA_AR_FILL_PREP)")
        self.assertGreaterEqual(len(self.real), 20)
        for case in self.real:
            for k in (6.0, 1.0):
                ref, got = self.compare(self.lib, case, k)
                self.assertTrue(same(ref, got), "%s k=%r" % (case[0], k))
            self.check_stats(case, 6.0)
        # real frames hold valid pixels (not a vacuous pass)
        self.assertGreater(self.lib.last_stats[0], 1000)

    def test_mutants_caught(self):
        caught = {}
        for m, lib in self.mutants.items():
            self.assertEqual(lib.mutant, m)
            bad = []
            for case in self.cases + (self.real or []):
                for k in (6.0, 1.0):
                    ref, got = self.compare(lib, case, k)
                    if not same(ref, got):
                        bad.append((case[0], k))
            caught[m] = bad
        self.assertEqual(self.lib.mutant, 0)
        # 1: without k only the stretched frames differ; 2: rounding is everywhere;
        # 3: exactly 0.1 m (code 100 x 0.001f) is valid in double
        self.assertTrue(caught[1] and all(k == 6.0 for _, k in caught[1]), caught[1])
        self.assertIn(("m_0p1001", 1.0), caught[2])
        self.assertIn(("mm_random", 1.0), caught[2])
        self.assertIn(("mm_boundaries", 1.0), caught[3])
        self.assertIn(("dense_0", 1.0), caught[3])
        if self.real:
            self.assertIn((self.real[0][0], 1.0), caught[2])
        for m in (1, 2, 3):
            print("mutant %d differs on %d of %d (case, k) pairs" % (
                m, len(caught[m]), 2 * len(self.cases + (self.real or []))), file=sys.stderr)

    def test_outside_native_domain_runs_reference(self):
        lib = ArFillNative(self.lib.path)
        u = bits(0.001)
        codes = np.random.RandomState(3).randint(0, 12000, 640 * 360).astype(np.uint16)
        # non-uint16 codes (same values as int32 / a list): reference
        for c, why in ((codes.astype(np.int32), "codes_dtype"), (codes.astype(">u2"), "codes_dtype")):
            s = snap_of(c, u, 640, 360)
            ref = kv_ar.fill_ar_inputs(s, self.lay, source="replay", depth_stretch=6.0)
            got = kv_ar.fill_ar_inputs(s, self.lay, source="replay", depth_stretch=6.0, native=lib)
            self.assertTrue(same(ref, got))
            self.assertGreaterEqual(lib.counts["python:%s" % why], 1)
        # NaN / inf / huge k: reference
        for k in (float("nan"), float("inf"), 1e9):
            s = snap_of(codes, u, 640, 360)
            ref = kv_ar.fill_ar_inputs(s, self.lay, source="replay", depth_stretch=k)
            got = kv_ar.fill_ar_inputs(s, self.lay, source="replay", depth_stretch=k, native=lib)
            self.assertTrue(same(ref, got), k)
        self.assertEqual(lib.counts["python:stretch_k"], 3)
        # too large / empty / short codes: the same exception type either way
        for c, W, H, why in ((np.zeros(1281 * 800, np.uint16), 1281, 800, "size"),
                             (np.zeros(10, np.uint16), 0, 360, "size"),
                             (np.zeros(100, np.uint16), 640, 360, "codes_short")):
            s = snap_of(c, u, W, H)
            errs = []
            for nat in (None, lib):
                try:
                    kv_ar.fill_ar_inputs(s, self.lay, source="replay", native=nat)
                    errs.append(None)
                except Exception as exc:        # noqa: BLE001 - the type is what is compared
                    errs.append(type(exc))
            self.assertEqual(errs[0], errs[1], why)
            self.assertIsNotNone(errs[0], why)
            self.assertGreaterEqual(lib.counts["python:%s" % why], 1)
        # a bad unit pattern: the same exception
        s = snap_of(codes, 1 << 33, 640, 360)
        with self.assertRaises(Exception) as a:
            kv_ar.fill_ar_inputs(s, self.lay, source="replay")
        with self.assertRaises(type(a.exception)):
            kv_ar.fill_ar_inputs(s, self.lay, source="replay", native=lib)

    def test_describe_names_the_source(self):
        d = self.lib.describe()
        self.assertEqual(d["impl"], "native")
        self.assertEqual(len(d["source_sha256"]), 64)
        self.assertEqual(len(d["so_sha256"]), 64)
        self.assertIn("-ffp-contract=off", d["flags"])


if __name__ == "__main__":
    unittest.main()
