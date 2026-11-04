"""Explicit cpu/pl backend switch with a mock PL device (2026-09-27).

Every rule is paired with a deliberately broken variant that must make the
same check fail (docs/design/M7_BACKEND_SWITCH_20260927.md section 3).
"""

import dataclasses
from pathlib import Path
import re
import subprocess
import tempfile
import types
import unittest

import numpy as np

from host.m7_backend import (M7BackendSelector, PlBackendUnavailable, PlFrameBackend,
                             PlObjectBackend)
from host.m7_frame_native import compile_frame_native
from host.m7_object_native import compile_object_native
from host.m7_pipeline import (CaptureIdentity, DepthFrame, GridSpec, M7Pipeline,
                              PoseAtCapture, SemanticPacket)
from host.m7_pl_device import (FRAME, OBJECT, FRAME_REGS, OBJECT_REGS, DevMemPlDevice,
                               MockPlDevice, PlDeviceUnavailable)
from host.m7_rolling import RollingFusion, compile_rolling
from perception.fpga_replay.core import Config

import geofx3_ref as fxr

ROOT = Path(__file__).resolve().parents[1]
REG_HEADER = "hw/m7/pl/sw/m7pl_hls_regs.h"
REG_HEADER_COMMIT = "55c7172"
A2_REG_HEADER_COMMIT = "0da5041"  # m7_pl_merge_a2, last header with the object core


def rotation(yaw, pitch):
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    rz = np.array([[cy, -sy, 0.], [sy, cy, 0.], [0., 0., 1.]])
    ry = np.array([[cp, 0., sp], [0., 1., 0.], [-sp, 0., cp]])
    return rz @ ry


def frame_cases():
    rng = np.random.default_rng(20260927)
    cases = []
    # Non-square, skewed intrinsics, rotated pose, off-lattice grid origin,
    # NaN / too-near / too-far pixels mixed with valid ones.
    depth = rng.uniform(0.3, 5.5, size=(40, 48)).astype(np.float32)
    depth[rng.random(depth.shape) < 0.1] = np.nan
    depth[rng.random(depth.shape) < 0.05] = 0.05
    depth[rng.random(depth.shape) < 0.05] = 12.0
    pose = np.eye(4)
    pose[:3, :3] = rotation(0.3, -0.2)
    pose[:3, 3] = [0.37, -0.21, 0.55]
    k = np.array([[38., 0.7, 23.5], [0., 41., 19.5], [0., 0., 1.]])
    cases.append(dict(depth_m=depth, k=k, camera_to_world=pose,
                      grid_origin_m=(-1.3, -0.73, -0.2), voxel_size_m=0.2,
                      shape=(32, 24, 16)))
    # One pixel wide and one pixel tall frames (raster counter edges).
    column = rng.uniform(0.5, 3.0, size=(37, 1)).astype(np.float32)
    cases.append(dict(depth_m=column, k=np.array([[20., 0., 0.], [0., 20., 18.], [0., 0., 1.]]),
                      camera_to_world=np.eye(4), grid_origin_m=(-2., -2., 0.),
                      voxel_size_m=0.25, shape=(16, 16, 16)))
    row = rng.uniform(0.5, 3.0, size=(1, 64)).astype(np.float32)
    cases.append(dict(depth_m=row, k=np.array([[20., 0., 32.], [0., 20., 0.], [0., 0., 1.]]),
                      camera_to_world=np.eye(4), grid_origin_m=(-2., -2., 0.),
                      voxel_size_m=0.25, shape=(16, 16, 16)))
    # Everything invalid: zero accepted rays, all-unknown grid.
    cases.append(dict(depth_m=np.full((8, 8), np.nan, np.float32),
                      k=np.array([[10., 0., 4.], [0., 10., 4.], [0., 0., 1.]]),
                      camera_to_world=np.eye(4), grid_origin_m=(0., 0., 0.),
                      voxel_size_m=0.5, shape=(4, 4, 4)))
    return cases


def call_frame(backend, case, frame_id=7):
    return backend.frame_from_depth(frame_id=frame_id, capture_time_ns=110, pose_time_ns=109,
                                    calibration_version="cal-a", pose_confirmed=True, **case)


def evidence_equal(a, b):
    for field in dataclasses.fields(a):
        x, y = getattr(a, field.name), getattr(b, field.name)
        if isinstance(x, np.ndarray):
            if x.dtype != y.dtype or x.shape != y.shape or not np.array_equal(x, y):
                return False
        elif x != y:
            return False
    return True


def payload_equal(a, b):
    if a.keys() != b.keys():
        return False
    for key in a:
        x, y = a[key], b[key]
        if isinstance(x, np.ndarray) or isinstance(y, np.ndarray):
            if not np.array_equal(np.asarray(x), np.asarray(y)):
                return False
        elif x != y:
            return False
    return True


def parse_header(text, prefix):
    return {m.group(1): int(m.group(2), 16) for m in
            re.finditer(r"#define %s_ADDR_(\w+)\s+(0x[0-9a-fA-F]+)" % prefix, text)}


def register_mismatches(table, header, prefix):
    parsed = parse_header(header, prefix)
    return sorted(name for name, off in table.items() if parsed.get(name) != off)


def assert_pl_never_falls_back(selector, what):
    """The 'no silent fallback' check, shared by the real selector and its fault."""
    builder = getattr(selector, "%s_backend" % what)
    try:
        backend = builder("pl")
    except PlBackendUnavailable:
        return
    name = getattr(backend, "backend_name", "cpu_reference")
    if not name.startswith("pl:"):
        raise AssertionError("backend='pl' returned a %s backend" % name)


class _BackendSwitchBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="m7-backend-")
        cls.addClassCleanup(cls.tmp.cleanup)
        d = Path(cls.tmp.name)
        cls.frame_lib = compile_frame_native(d / "frame.so")
        cls.object_lib = compile_object_native(d / "object.so")
        cls.dir = d
        cls.rolling_builds = 0

    @classmethod
    def fresh_rolling_library(cls):
        # RollingKernelHost requires one freshly compiled library per instance.
        cls.rolling_builds += 1
        return compile_rolling(cls.dir / ("rolling-%d.so" % cls.rolling_builds))

    def cpu(self):
        return M7BackendSelector(cpu_frame_library=self.frame_lib,
                                 cpu_object_library=self.object_lib)

    def pl(self, **mock):
        self.device = MockPlDevice(frame_library=self.frame_lib,
                                   object_library=self.object_lib, frame_fx_library=fxr.fx_library(), **mock)
        return M7BackendSelector(pl_device_factory=lambda: self.device, pl_timeout_s=0.2,
                                 cpu_frame_library=self.frame_lib, pl_frame_fx_library=fxr.fx_library())


class FrameBackendTests(_BackendSwitchBase):
    """GEO_FX3 (2026-11-04): the PL frame core is the fixed-point core (16-bit codes only). The PL result is
    compared with its bit-exact model host.m7_frame_fx.FrameFx (tests/geofx3_ref.py), not with the float
    CPU reference; the float call of a PL backend is the CPU reference on the A53 (no PL start)."""

    def test_pl_mock_equals_fx_model_and_float_call_is_cpu_reference(self):
        cpu = self.cpu().frame_backend("cpu")
        pl = self.pl().frame_backend("pl")
        self.assertEqual((cpu.backend_name, pl.backend_name), ("cpu_reference", "pl:mock"))
        for index, case in enumerate(frame_cases()):
            with self.subTest(case=index):
                want, got = fxr.expect(*fxr.case_args(case)), fxr.call_codes(pl, *fxr.case_args(case))
                self.assertTrue(fxr.same(want, got))
                self.assertEqual(int(got.flags.size), int(np.prod(case["shape"])))
                self.assertTrue(evidence_equal(call_frame(cpu, case), call_frame(pl, case)))
        self.assertEqual(self.device.starts[FRAME], len(frame_cases()))
        self.assertEqual(pl.counts["cpu_reference_float"], len(frame_cases()))
        # The main case must be non-trivial, or equality proves nothing.
        a = fxr.expect(*fxr.case_args(frame_cases()[0]))
        self.assertGreater(a.accepted_rays, 100)
        self.assertGreater(a.invalid_depth_or_projection, 0)
        self.assertGreater(a.outside_grid, 0)
        self.assertTrue((a.flags == 1).any() and (a.flags == 2).any())

    def test_core_rejection_is_identical_at_the_call_level(self):
        cpu = self.cpu().frame_backend("cpu")
        pl = self.pl().frame_backend("pl")
        outs = []
        for backend in (cpu, pl):
            flags = np.full(32 ** 3, 99, np.uint8)
            counts = np.full(3, 99, np.uint32)
            status = np.full(1, 99, np.uint32)
            backend.lib.fpga_m7_depth_frame(
                np.zeros(1280 * 800, np.float32), np.eye(3, dtype=np.float32).ravel(),
                np.eye(3, 4, dtype=np.float32).ravel(), np.zeros(3), 0.25,
                4, 4, 0, 4, 4, flags, counts, status)  # nx = 0 is rejected
            outs.append((flags, counts, status))
        self.assertNotEqual(int(outs[0][2][0]), 0)
        for x, y in zip(*outs):
            np.testing.assert_array_equal(x, y)
        # the codes call of a rejected frame: flags untouched, status != 0, depth crc 0
        flags = np.full(32 ** 3, 99, np.uint8)
        counts = np.full(3, 99, np.uint32)
        status = np.full(1, 99, np.uint32)
        crc = pl.lib.fpga_m7_depth_frame_codes(
            np.zeros(16, np.uint16), fxr.UNIT_MM_BITS, np.eye(3, dtype=np.float32).ravel(),
            np.eye(3, 4, dtype=np.float32).ravel(), np.zeros(3), 0.25, 4, 4, 0, 4, 4, flags, counts, status)
        self.assertNotEqual(int(status[0]), 0)
        self.assertTrue((flags == 99).all())
        self.assertEqual(crc, 0)

    def test_fault_flipped_output_voxel_is_seen(self):
        args = fxr.case_args(frame_cases()[0])
        reference = fxr.expect(*args)
        free = np.flatnonzero(reference.flags.ravel() == 1)[0]  # FREE -> HIT, still legal
        pl = self.pl(corrupt_next=(FRAME, int(free))).frame_backend("pl")
        self.assertFalse(fxr.same(reference, fxr.call_codes(pl, *args)))

    def test_fault_width_height_registers_swapped_is_seen(self):
        args = fxr.case_args(frame_cases()[0])
        reference = fxr.expect(*args)
        regs = dict(FRAME_REGS)
        regs["WIDTH_DATA"], regs["HEIGHT_DATA"] = regs["HEIGHT_DATA"], regs["WIDTH_DATA"]
        pl = PlFrameBackend(MockPlDevice(frame_fx_library=fxr.fx_library()), regs=regs,
                            cpu_library=self.frame_lib, fx_library=fxr.fx_library())
        self.assertFalse(fxr.same(reference, fxr.call_codes(pl, *args)))

    def test_fault_dropped_high_word_of_64_bit_writes_is_seen(self):
        args = fxr.case_args(frame_cases()[0])
        reference = fxr.expect(*args)
        # GEO_FX3: the only 64-bit writes left are the buffer addresses (the float64 GRID_ORIGIN / VOXEL
        # registers are no longer written), so the buffers are placed above 4 GiB (KV260 DDR high window
        # 0x8_0000_0000) for the dropped high word to matter.
        def high_backend():
            dev = MockPlDevice(frame_fx_library=fxr.fx_library())
            dev._next_pa = 0x8_0000_0000
            return PlFrameBackend(dev, cpu_library=self.frame_lib, fx_library=fxr.fx_library())
        self.assertTrue(fxr.same(reference, fxr.call_codes(high_backend(), *args)))   # the window itself works
        pl = high_backend()

        def low_only(core, name, value):
            core.device.wr(core.block, core.regs[name], int(value) & 0xFFFFFFFF)
        pl.lib.wr64 = types.MethodType(low_only, pl.lib)
        try:
            broken = fxr.call_codes(pl, *args)
        except (RuntimeError, ValueError):
            return  # also a visible failure
        self.assertFalse(fxr.same(reference, broken))


def object_inputs():
    k = np.array([[12., 0., 12.], [0., 12., 12.], [0., 0., 1.]])
    pose = np.eye(4)
    pose[:3, :3] = rotation(0.1, 0.05)
    pose[:3, 3] = [0.02, -0.01, 0.0]
    present = np.full((25, 25), 3., np.float32)
    present[10:15, 10:15] = 2.
    present[11, 12] = np.nan
    mask = np.zeros((25, 25), np.bool_)
    mask[10:15, 10:15] = True
    removed = np.full((25, 25), 3., np.float32)
    occluded = np.full((25, 25), 3., np.float32)
    occluded[9:16, 9:17] = 1.
    return [
        dict(depth_m=present, mask=mask, k=k, local_camera=pose),
        dict(depth_m=removed, mask=np.zeros((25, 25), np.bool_), k=k, local_camera=np.eye(4),
             roi=(9, 10, 16, 14), near_z=1.5, far_z=2.5),
        dict(depth_m=occluded, mask=None, k=k, local_camera=np.eye(4),
             roi=(9, 10, 16, 14), near_z=1.5, far_z=2.5),
        dict(depth_m=present, mask=np.zeros((25, 25), np.bool_), k=k, local_camera=np.eye(4)),
    ]


def raw_equal(a, b):
    return a.counts == b.counts and np.array_equal(a.points_local_m, b.points_local_m)


class ObjectBackendTests(_BackendSwitchBase):
    def evaluate(self, backend, case):
        case = dict(case)
        return backend.evaluate(case.pop("depth_m"), case.pop("mask"), case.pop("k"),
                                case.pop("local_camera"), cfg=Config(), **case)

    def test_cpu_and_pl_mock_evaluate_and_candidate_are_equal(self):
        cpu = self.cpu().object_backend("cpu")
        pl = self.pl().object_backend("pl")
        candidates = set()
        for index, case in enumerate(object_inputs()):
            with self.subTest(case=index):
                a, b = self.evaluate(cpu, case), self.evaluate(pl, case)
                self.assertTrue(raw_equal(a, b))
                candidates.add(a.candidate)
        self.assertEqual(candidates, {"support_present", "free_space_conflict", "indeterminate"})
        self.assertGreater(len(self.evaluate(cpu, object_inputs()[0]).points_local_m), 20)
        bounds = np.array([[-.25, -.25, 1.5], [.25, .25, 2.5]])
        case = object_inputs()[0]
        args = (case["depth_m"], case["k"], case["local_camera"], case["mask"], bounds, Config())
        self.assertEqual(repr(cpu.candidate(*args)), repr(pl.candidate(*args)))

    def test_core_rejection_is_identical_at_the_call_level(self):
        cpu = self.cpu().object_backend("cpu")
        pl = self.pl().object_backend("pl")
        outs = []
        for backend in (cpu, pl):
            points = np.full(3 * 16, np.nan)
            counts = np.full(8, 0xFFFFFFFF, np.uint32)
            status = np.full(1, 0xFFFFFFFF, np.uint32)
            backend.lib.fpga_m7_object_evidence(
                np.ones(16, np.float32), np.zeros(16, np.uint8), 4, 4, 0, 1,
                np.array([3, 3, 1, 1], np.uint32), 1., 2., .05,
                np.eye(3).ravel(), np.eye(3, 4).ravel(), 9, .7, .9, points, counts, status)
            outs.append((points, counts, status))
        self.assertNotEqual(int(outs[0][2][0]), 0)
        for x, y in zip(*outs):
            np.testing.assert_array_equal(x, y)

    def test_fault_flipped_xyz_byte_is_seen(self):
        case = object_inputs()[0]
        reference = self.evaluate(self.cpu().object_backend("cpu"), case)
        pl = self.pl(corrupt_next=(OBJECT, 8 * 3 * 5 + 2)).object_backend("pl")
        self.assertFalse(raw_equal(reference, self.evaluate(pl, case)))

    def test_fault_roi_registers_swapped_is_seen(self):
        case = object_inputs()[2]
        reference = self.evaluate(self.cpu().object_backend("cpu"), case)
        regs = dict(OBJECT_REGS)
        regs["ROI_X1_DATA"], regs["ROI_Y1_DATA"] = regs["ROI_Y1_DATA"], regs["ROI_X1_DATA"]
        pl = PlObjectBackend(MockPlDevice(object_library=self.object_lib), regs=regs)
        self.assertFalse(raw_equal(reference, self.evaluate(pl, case)))

    def test_v3a_slot_map_without_object_slot_is_refused_before_any_write(self):
        # v3a (2026-10-26) removed the legacy object core: SLOT_MAP 0x3D3, bit 2 clear.
        device = MockPlDevice(object_library=self.object_lib)
        before = dict(device.starts)
        with self.assertRaises(PlDeviceUnavailable):
            PlObjectBackend(device, slot_map=0x3D3)
        self.assertEqual(device.starts, before)
        # A2 (0x00da5041) SLOT_MAP 0x3D7 has slot 2: accepted and usable.
        pl = PlObjectBackend(device, slot_map=0x3D7)
        case = object_inputs()[0]
        reference = self.evaluate(self.cpu().object_backend("cpu"), case)
        self.assertTrue(raw_equal(reference, self.evaluate(pl, case)))

    def test_devmem_device_reads_slot_map_and_selector_raises_unavailable(self):
        reads = []

        class V3aBackend(PlObjectBackend):
            @staticmethod
            def slot_map_reader():
                reads.append(1)
                return 0x3D3

        fake = types.SimpleNamespace(kind="devmem")
        with self.assertRaises(PlDeviceUnavailable):
            V3aBackend(fake)
        self.assertEqual(reads, [1])
        # Through the selector: same refusal becomes PlBackendUnavailable, counted.
        selector = M7BackendSelector(pl_device_factory=lambda: fake, pl_timeout_s=0.2)
        original = PlObjectBackend.slot_map_reader
        PlObjectBackend.slot_map_reader = staticmethod(lambda: 0x3D3)
        try:
            with self.assertRaises(PlBackendUnavailable):
                selector.object_backend("pl")
        finally:
            PlObjectBackend.slot_map_reader = original
        self.assertEqual(selector.counts["pl_unavailable:object"], 1)


VOXEL = 0.05
FUSION_GRID = GridSpec(origin_m=(-2 * VOXEL, -16 * VOXEL, -4 * VOXEL), voxel_m=VOXEL,
                       shape=(32, 32, 8))
FUSION_K = np.array([[40., 0., 4.], [0., 40., 4.], [0., 0., 1.]])
FUSION_T = np.eye(4)
FUSION_T[:3, :3] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]
FUSION_T[:3, 3] = (0.025, 0.025, 0.025)


class ConsumerEquivalenceTests(_BackendSwitchBase):
    def run_chain(self, selector, name, *, corrupt_frame=None, corrupt_index=5):
        pipeline = M7Pipeline(selector.frame_backend(name), FUSION_GRID,
                              object_backend=selector.object_backend(name))
        fusion = RollingFusion(self.cpu().rolling_backend("cpu", cpu_library=self.fresh_rolling_library()),
                               voxel_m=VOXEL)
        rng = np.random.default_rng(7)
        records = []
        t = 1_000_000_000
        for frame_id in range(1, 7):
            t += 10_000_000
            depth = np.full((9, 9), 1.1, np.float32)
            depth[3:6, 3:6] = 0.5 + 0.05 * frame_id
            depth[rng.random((9, 9)) < 0.1] = np.nan
            if frame_id == 4:
                depth[:] = np.nan  # no valid depth: invalid output, map untouched
            identity = CaptureIdentity("cam0", frame_id, t, "mono", "cal-1", "odom", "e1")
            pose = PoseAtCapture(t, "mono", "odom", "e1", True, FUSION_T.copy())
            if corrupt_frame == frame_id:
                self.device.corrupt_next = (FRAME, corrupt_index)
            out = pipeline.ingest_geometry(DepthFrame(identity, depth, FUSION_K, pose),
                                           now_ns=t + 1_000_000, now_clock_id="mono")
            nav, display = out.navigation, out.display
            mask = np.zeros((9, 9), np.bool_)
            mask[3:6, 3:6] = True
            semantic = pipeline.ingest_semantic(
                SemanticPacket(identity, nav.frame_token or "none", "cup", mask),
                now_ns=t + 1_500_000, now_clock_id="mono")
            fused = fusion.ingest(nav, pipeline, now_ns=t + 2_000_000, clock_id="mono",
                                  robot_position_m=(0.025, 0.025, 0.025))
            records.append((nav, display, semantic, fused))
        return records

    def assert_chains_equal(self, left, right):
        for (n1, d1, s1, f1), (n2, d2, s2, f2) in zip(left, right):
            for a, b in ((n1, n2), (d1, d2), (s1, s2)):
                self.assertEqual((a.valid, a.reason, a.channel, a.age_ns, a.expires_at_ns),
                                 (b.valid, b.reason, b.channel, b.age_ns, b.expires_at_ns))
                self.assertTrue(payload_equal(a.payload, b.payload), a.reason)
            self.assertEqual((f1.accepted, f1.reason), (f2.accepted, f2.reason))
            self.assertEqual(f1.delta, f2.delta)

    def test_pipeline_memory_and_rolling_consumers_see_the_same_content(self):
        cpu = self.run_chain(self.cpu(), "cpu")
        pl = self.run_chain(self.pl(), "pl")
        self.assert_chains_equal(cpu, pl)
        self.assertEqual({r[0].backend for r in cpu}, {"cpu_reference"})
        self.assertEqual({r[0].backend for r in pl}, {"pl:mock"})
        self.assertEqual({r[2].backend for r in pl}, {"pl:mock"})
        accepted = [r[3].accepted for r in cpu]
        self.assertEqual(accepted.count(True), 5)       # frame 4 has no valid depth
        self.assertTrue(any(r[3].delta and r[3].delta.changes for r in cpu))
        self.assertIn("support_present", {r[2].payload.get("candidate") for r in cpu})

    # GEO_FX3 (2026-11-04): test_fault_flipped_voxel_is_seen_by_the_consumer_comparison removed. M7Pipeline
    # calls the float frame_from_depth, which on a PL backend is now the CPU reference (the PL reads codes
    # only), so a device fault can no longer reach this chain; the device-level fault checks are
    # FrameBackendTests above and tests/test_geofx3_driver_20261104.py.


class NoFallbackTests(_BackendSwitchBase):
    def test_unknown_names_are_rejected(self):
        selector = self.cpu()
        for name in ("auto", "PL", "", None, "fpga"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                selector.frame_backend(name)

    def test_pl_without_device_raises_and_counts_and_never_builds_cpu(self):
        selector = self.cpu()  # has CPU libraries but no PL device factory
        for what in ("frame", "object"):
            with self.assertRaises(PlBackendUnavailable):
                getattr(selector, "%s_backend" % what)("pl")
            assert_pl_never_falls_back(selector, what)
            self.assertEqual(selector.counts["pl_unavailable:%s" % what], 2)
        self.assertEqual(selector.counts["selected:frame:cpu"], 0)

    def test_factory_failure_raises_and_counts(self):
        def factory():
            return DevMemPlDevice.open(udmabuf="p4-no-such-udmabuf")
        selector = M7BackendSelector(cpu_frame_library=self.frame_lib, pl_device_factory=factory)
        with self.assertRaises(PlBackendUnavailable) as ctx:
            selector.frame_backend("pl")
        self.assertIn("p4-no-such-udmabuf", str(ctx.exception))
        self.assertEqual(selector.counts["pl_unavailable:frame"], 1)
        with self.assertRaises(PlDeviceUnavailable):
            DevMemPlDevice.open(udmabuf="p4-no-such-udmabuf")

    def test_rolling_pl_is_explicitly_unavailable(self):
        selector = self.pl()
        with self.assertRaises(PlBackendUnavailable):
            selector.rolling_backend("pl")
        self.assertEqual(selector.counts["pl_unavailable:rolling"], 1)

    def test_fault_silent_fallback_selector_is_caught(self):
        class FallbackSelector(M7BackendSelector):
            def frame_backend(self, backend):
                try:
                    return super().frame_backend(backend)
                except PlBackendUnavailable:
                    return super().frame_backend("cpu")
        selector = FallbackSelector(cpu_frame_library=self.frame_lib)
        with self.assertRaises(AssertionError):
            assert_pl_never_falls_back(selector, "frame")

    def test_device_timeout_fails_the_codes_frame_and_does_not_fall_back(self):
        # GEO_FX3: the codes call (the only PL path) raises on a timeout; no CPU reference result instead.
        backend = self.pl(hang=True).frame_backend("pl")
        with self.assertRaises(RuntimeError):
            fxr.call_codes(backend, *fxr.case_args(frame_cases()[3]))
        self.assertEqual(backend.counts["device_timeout"], 1)
        self.assertEqual(backend.counts["cpu_reference:fx_status_0"], 0)
        self.assertEqual(self.device.starts[FRAME], 1)

    def test_busy_core_is_not_started(self):
        backend = self.pl(busy=True).frame_backend("pl")
        with self.assertRaises(RuntimeError):
            fxr.call_codes(backend, *fxr.case_args(frame_cases()[3]))
        self.assertEqual(backend.counts["device_busy"], 1)
        self.assertEqual(self.device.starts[FRAME], 0)


class RegisterTableTests(unittest.TestCase):
    @staticmethod
    def git_header(commit):
        try:
            return subprocess.run(["git", "-C", str(ROOT), "show",
                                   "%s:%s" % (commit, REG_HEADER)],
                                  check=True, capture_output=True, text=True).stdout
        except (OSError, subprocess.CalledProcessError):
            return None

    @classmethod
    def header_text(cls):
        path = ROOT / REG_HEADER
        if path.is_file():
            return path.read_text()
        return cls.git_header(REG_HEADER_COMMIT)

    def test_transcribed_offsets_match_the_generated_header(self):
        text = self.header_text()
        if text is None:
            self.skipTest("m7pl_hls_regs.h not in tree and %s not in git" % REG_HEADER_COMMIT)
        self.assertEqual(register_mismatches(FRAME_REGS, text, "XFPGA_M7_FRAME_AXI_CONTROL"), [])
        # v3a (2026-10-26) dropped the legacy object core from the tree header;
        # the object table is checked against the A2 header (0da5041), the
        # bitstream that still carries the core.
        a2_text = self.git_header(A2_REG_HEADER_COMMIT)
        if a2_text is None:
            self.skipTest("%s:%s not in git" % (A2_REG_HEADER_COMMIT, REG_HEADER))
        self.assertEqual(register_mismatches(OBJECT_REGS, a2_text, "XFPGA_M7_OBJECT_AXI_CONTROL"), [])
        # Fault: one wrong entry must be reported.
        wrong = dict(FRAME_REGS, NZ_DATA=FRAME_REGS["NZ_DATA"] + 8)
        self.assertEqual(register_mismatches(wrong, text, "XFPGA_M7_FRAME_AXI_CONTROL"),
                         ["NZ_DATA"])


if __name__ == "__main__":
    unittest.main()
