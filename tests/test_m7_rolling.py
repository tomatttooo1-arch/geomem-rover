"""M7 cross-frame rolling log-odds map: known answers, safety, rolling, epochs.

Three independent implementations are compared: the HLS top compiled for the
host (packed words + ring offsets), the plain C++ reference (byte per cell,
window move copies data) and ``PyOracle`` below (numpy, update rule and
constants re-typed from the design document, NOT read from the C++ build).
Every safety/rolling/epoch rule also has a deliberately broken build
(FPGA_M7_ROLL_MUTANT_*); each check must pass on the delivered build and fail
on the mutant it targets, which proves the check can see that fault.
"""

from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np

from host.m7_frame_native import FrameNativeM7, compile_frame_native
from host.m7_pipeline import (CaptureIdentity, DepthFrame, GridSpec, M7Pipeline,
                              PoseAtCapture)
from host.m7_rolling import (FREE, MUTANTS, OCCUPIED, UNKNOWN, RollingDeltaMirror, RollingFusion, RollingKernelHost,
                             RollingReference, compile_rolling)


ROOT = Path(__file__).resolve().parents[1]

# Constants re-typed from docs/design/M7_ROLLING_FUSION_RESULTS_20260926.md 2.2.
HIT, MISS, LMAX, LMIN, T_OCC, T_FREE = 32, 12, 96, -32, 16, -16
DIMS = (80, 80, 32)


def oracle_state(logodds):
    values = np.asarray(logodds, dtype=np.int32)
    return np.where(values >= T_OCC, OCCUPIED,
                    np.where(values <= T_FREE, FREE, UNKNOWN)).astype(np.uint8)


class PyOracle:
    """Third implementation: numpy, world-anchored window, explicit eviction."""

    def __init__(self, dims=DIMS):
        self.dims = tuple(dims)
        self.initialized = False

    def reset(self, epoch, base):
        mx, my, mz = self.dims
        self.cells = np.zeros((mz, my, mx), dtype=np.int32)
        self.base, self.epoch = tuple(base), epoch
        self.last, self.seq, self.initialized = None, 0, True

    def integrate(self, epoch, frame, window_base, obs_base, obs):
        if not self.initialized:
            return "uninitialized", ()
        if epoch != self.epoch:
            return "stale_epoch", ()
        if self.last is not None and frame <= self.last:
            return "stale_frame", ()
        if np.any(obs > 2):
            return "malformed_observation", ()
        mx, my, mz = self.dims
        # Evict: new local l holds world window_base + l = old base + (l + d).
        d = [n - o for n, o in zip(window_base, self.base)]
        moved = np.zeros_like(self.cells)
        src = [slice(max(0, k), min(m, m + k)) for k, m in zip(d, (mx, my, mz))]
        dst = [slice(max(0, -k), min(m, m - k)) for k, m in zip(d, (mx, my, mz))]
        if all(s.start < s.stop for s in src):
            moved[dst[2], dst[1], dst[0]] = self.cells[src[2], src[1], src[0]]
        self.cells, self.base = moved, tuple(window_base)
        before = oracle_state(self.cells)
        nz, ny, nx = obs.shape
        z, y, x = np.meshgrid(np.arange(nz), np.arange(ny), np.arange(nx), indexing="ij")
        lx, ly, lz = (x + obs_base[0] - self.base[0], y + obs_base[1] - self.base[1],
                      z + obs_base[2] - self.base[2])
        inside = (lx >= 0) & (lx < mx) & (ly >= 0) & (ly < my) & (lz >= 0) & (lz < mz)
        v = obs[inside]
        idx = (lz[inside], ly[inside], lx[inside])
        cur = self.cells[idx]
        cur = np.where(v == 2, np.minimum(cur + HIT, LMAX),
                       np.where(v == 1, np.maximum(cur - MISS, LMIN), cur))
        self.cells[idx] = cur
        after = oracle_state(self.cells)
        cz, cy, cx = np.nonzero(before != after)
        changes = tuple(sorted((int(a + self.base[0]), int(b + self.base[1]),
                                int(c + self.base[2]), int(after[c, b, a]))
                               for a, b, c in zip(cx, cy, cz)))
        self.last, self.seq = frame, self.seq + 1
        return "ok", changes

    def logodds(self, world):
        l = [w - b for w, b in zip(world, self.base)]
        if any(not 0 <= v < m for v, m in zip(l, self.dims)):
            return None
        return int(self.cells[l[2], l[1], l[0]])


def line_block(*, hit_at=None, free_to=None, length=32):
    """1x1xN block along x from a camera at x = 0: free 1..free_to, hit."""
    obs = np.zeros((1, 1, length), dtype=np.uint8)
    if free_to is not None:
        obs[0, 0, 1:free_to + 1] = FREE
    if hit_at is not None:
        obs[0, 0, hit_at] = OCCUPIED
    return obs


LINE_BASE = (0, 0, 0)      # block cell x is world x
WINDOW = (-40, -40, -8)    # window base used by the scenarios


def logodds_at(backend, world):
    result, (states, logodds, outside) = backend.query([world])
    if not result.ok:
        return None
    return None if outside[0] else int(logodds[0])


# ---- Scenario checks. Each returns a list of failure strings. -------------

def scenario_obstacle_leaves(backend):
    """Known answer: an obstacle clears ONLY through valid free observations."""
    fails = []
    backend.reset(1, WINDOW)
    seq = 0
    for _ in range(3):  # wall at 24, path free
        seq += 1
        backend.integrate(1, seq, WINDOW, LINE_BASE, line_block(hit_at=24, free_to=23))
    if logodds_at(backend, (24, 0, 0)) != 96:
        fails.append("wall 3 hits != 96")
    if logodds_at(backend, (12, 0, 0)) != -32:
        fails.append("path 3 frees != -32")
    for expected in (0, 32, 64):  # box appears at 12; wall behind unobserved
        seq += 1
        backend.integrate(1, seq, WINDOW, LINE_BASE, line_block(hit_at=12, free_to=11))
        if logodds_at(backend, (12, 0, 0)) != expected:
            fails.append("box log-odds while present != %d" % expected)
    for k in range(1, 8):  # box leaves: 64 - 12k
        seq += 1
        result = backend.integrate(1, seq, WINDOW, LINE_BASE, line_block(hit_at=24, free_to=23))
        expected = 64 - 12 * k
        if logodds_at(backend, (12, 0, 0)) != expected:
            fails.append("box after %d frees != %d" % (k, expected))
        state = (12, 0, 0, FREE) in result.changes
        if (k == 7) != state:
            fails.append("box FREE transition reported at wrong frame %d" % k)
    # The wall itself is removed: rays now go through 24 and hit 30.
    for k in range(1, 11):
        seq += 1
        backend.integrate(1, seq, WINDOW, LINE_BASE, line_block(hit_at=30, free_to=29))
        _, (states, _, _) = backend.query([(24, 0, 0)])
        expected = oracle_state(96 - 12 * k)
        if int(states[0]) != int(expected):
            fails.append("removed wall state after %d frees" % k)
    return fails


def scenario_never_cleared_without_free(backend):
    """Safety: occlusion / invalid depth / missing frames / looking away."""
    fails = []
    backend.reset(1, WINDOW)
    seq = 0
    for _ in range(3):
        seq += 1
        backend.integrate(1, seq, WINDOW, LINE_BASE, line_block(hit_at=24, free_to=23))
    for _ in range(50):  # occluder at 12 in front of the wall
        seq += 1
        backend.integrate(1, seq, WINDOW, LINE_BASE, line_block(hit_at=12, free_to=11))
    if logodds_at(backend, (24, 0, 0)) != 96:
        fails.append("occluded wall changed")
    for _ in range(50):  # invalid depth: the per-frame block is all UNKNOWN
        seq += 1
        backend.integrate(1, seq, WINDOW, LINE_BASE, np.zeros((1, 1, 32), dtype=np.uint8))
    if logodds_at(backend, (24, 0, 0)) != 96:
        fails.append("invalid depth changed the wall")
    seq += 1000  # 1000 missing frames: nothing is called at all
    for _ in range(50):  # the camera looks elsewhere: block far from the wall
        seq += 1
        backend.integrate(1, seq, WINDOW, (0, 20, 0), line_block(hit_at=24, free_to=23))
    if logodds_at(backend, (24, 0, 0)) != 96:
        fails.append("looking elsewhere / missing frames changed the wall")
    _, (states, _, _) = backend.query([(24, 0, 0)])
    if int(states[0]) != OCCUPIED:
        fails.append("wall not OCCUPIED")
    return fails


def scenario_rolling(backend):
    """World consistency across ring moves; evicted cells return UNKNOWN."""
    fails = []
    backend.reset(1, WINDOW)
    seq = 0
    for _ in range(3):
        seq += 1
        backend.integrate(1, seq, WINDOW, LINE_BASE, line_block(hit_at=24, free_to=23))
    empty = np.zeros((1, 1, 1), dtype=np.uint8)
    base = list(WINDOW)
    moves = [(3, 0, 0), (0, -5, 0), (0, 0, 2), (-7, 4, -1), (11, 0, 0), (0, 9, 0),
             (-2, -2, -2), (5, 5, 5)]
    for move in moves:
        base = [b + m for b, m in zip(base, move)]
        seq += 1
        backend.integrate(1, seq, base, (base[0], base[1], base[2]), empty)
        for world, expected in (((24, 0, 0), 96), ((5, 0, 0), -32), ((30, 0, 0), 0)):
            inside = all(0 <= w - b < m for w, b, m in zip(world, base, DIMS))
            got = logodds_at(backend, world)
            if inside and got != expected:
                fails.append("world %s after move %s: %r" % (world, move, got))
            if not inside and got is not None:
                fails.append("world %s outside but answered" % (world,))
    # Move so the wall leaves (window x starts at 30), then come back.
    for target in ((30, base[1], base[2]), tuple(WINDOW)):
        seq += 1
        backend.integrate(1, seq, target, target, empty)
    if logodds_at(backend, (24, 0, 0)) != 0:
        fails.append("re-entered wall is not UNKNOWN (ghost)")
    if logodds_at(backend, (5, 0, 0)) != 0:
        fails.append("re-entered path is not UNKNOWN (ghost)")
    return fails


def scenario_epoch(backend):
    """Stale epochs / frames / malformed blocks are rejected and counted."""
    fails = []
    backend.reset(5, WINDOW)
    r = backend.integrate(5, 10, WINDOW, LINE_BASE, line_block(hit_at=24, free_to=23))
    before = backend.snapshot()[1]
    r1 = backend.integrate(4, 11, WINDOW, LINE_BASE, line_block(hit_at=24, free_to=23))
    r2 = backend.integrate(5, 10, WINDOW, LINE_BASE, line_block(hit_at=24, free_to=23))
    bad = line_block(hit_at=24, free_to=23)
    bad[0, 0, 31] = 3
    r3 = backend.integrate(5, 11, WINDOW, LINE_BASE, bad)
    after = backend.snapshot()[1]
    if (r1.status_name, r2.status_name, r3.status_name) != (
            "stale_epoch", "stale_frame", "malformed_observation"):
        fails.append("rejections %s" % ((r1.status_name, r2.status_name, r3.status_name),))
    if (r3.rejected_epoch_total - r.rejected_epoch_total,
            r3.rejected_frame_total - r.rejected_frame_total,
            r3.rejected_malformed_total - r.rejected_malformed_total) != (1, 1, 1):
        fails.append("rejections not counted once each")
    if not np.array_equal(before, after) or r3.delta_seq != r.delta_seq:
        fails.append("a rejected call changed the map or the delta sequence")
    backend.reset(6, WINDOW)
    r4 = backend.integrate(5, 12, WINDOW, LINE_BASE, line_block(hit_at=24, free_to=23))
    if r4.status_name != "stale_epoch" or logodds_at(backend, (24, 0, 0)) != 0:
        fails.append("old epoch accepted after reset")
    r5 = backend.integrate(6, 1, WINDOW, LINE_BASE, line_block(hit_at=24, free_to=23))
    if not r5.ok or r5.delta_seq != 1 or logodds_at(backend, (24, 0, 0)) != 32:
        fails.append("new epoch did not restart cleanly")
    return fails


SCENARIOS = {
    "obstacle_leaves": scenario_obstacle_leaves,
    "never_cleared_without_free": scenario_never_cleared_without_free,
    "rolling": scenario_rolling,
    "epoch": scenario_epoch,
}
# Which scenario each mutant must break (it may break others as well).
MUTANT_TARGET = {
    "UNKNOWN_CLEARS": "never_cleared_without_free",
    "NO_FREE_CLEAR": "obstacle_leaves",
    "DECAY": "never_cleared_without_free",
    "SHIFT_SIGN": "rolling",
    "NO_EVICT": "rolling",
    "IGNORE_EPOCH": "epoch",
}


class RollingBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="m7-rolling-")
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.dir = Path(cls.tmp.name)
        cls.ref_lib = compile_rolling(cls.dir / "ref.so", source="reference")
        cls._n = 0

    @classmethod
    def fresh_kernel(cls, mutant=None, dims=None):
        cls._n += 1
        return RollingKernelHost(compile_rolling(cls.dir / ("k%d.so" % cls._n),
                                                 source="kernel", mutant=mutant, dims=dims))

    @classmethod
    def reference(cls, mutant=None, dims=None):
        if mutant is None and dims is None:
            return RollingReference(cls.ref_lib)
        cls._n += 1
        return RollingReference(compile_rolling(cls.dir / ("r%d.so" % cls._n),
                                                source="reference", mutant=mutant, dims=dims))


class RollingKernelTests(RollingBase):
    def test_configuration_is_the_documented_verification_config(self):
        for backend in (self.fresh_kernel(), self.reference()):
            c = backend.config
            self.assertEqual((c.dims, c.lanes, c.words, c.obs_axis, c.delta_capacity,
                              c.query_capacity, c.header_words, c.mutant_mask),
                             (DIMS, 8, 25600, 32, 5120, 1024, 24, 0))
            self.assertEqual((c.hit, c.miss, c.lmax, c.lmin, c.t_occ, c.t_free),
                             (HIT, MISS, LMAX, LMIN, T_OCC, T_FREE))
        # The five design constraints of section 2.2, as arithmetic.
        self.assertGreaterEqual(0 + HIT, T_OCC)           # unseen + 1 hit -> OCC
        self.assertGreater(LMIN + HIT, T_FREE)            # saturated free + 1 hit leaves FREE
        self.assertGreaterEqual(LMIN + 2 * HIT, T_OCC)    # ... and 2 hits -> OCC
        self.assertGreater(-MISS, T_FREE)                 # one free obs is not enough
        self.assertLessEqual(-2 * MISS, T_FREE)           # two are
        self.assertTrue(-128 <= LMIN < T_FREE < 0 < T_OCC < LMAX <= 127)

    def test_scenarios_pass_on_kernel_and_reference(self):
        for name, scenario in SCENARIOS.items():
            for label, backend in (("kernel", self.fresh_kernel()), ("reference", self.reference())):
                with self.subTest(scenario=name, backend=label):
                    self.assertEqual(scenario(backend), [])

    def test_each_mutant_fails_its_targeted_scenario(self):
        for mutant in MUTANTS:
            scenario = SCENARIOS[MUTANT_TARGET[mutant]]
            builds = [("reference", self.reference(mutant))]
            if mutant != "DECAY":  # decay fault exists in the reference only
                builds.append(("kernel", self.fresh_kernel(mutant)))
            for label, backend in builds:
                with self.subTest(mutant=mutant, backend=label):
                    self.assertNotEqual(backend.config.mutant_mask, 0)
                    self.assertNotEqual(scenario(backend), [],
                                        "%s did not detect %s" % (MUTANT_TARGET[mutant], mutant))

    def test_python_oracle_itself_detects_a_broken_rule(self):
        # Instrument check: the oracle must disagree with a reference build
        # whose free rule is broken, on the same inputs it agrees with normally.
        good, bad, oracle = self.reference(), self.reference("NO_FREE_CLEAR"), PyOracle()
        for backend in (good, bad):
            backend.reset(1, WINDOW)
        oracle.reset(1, WINDOW)
        for seq in range(1, 4):
            block = line_block(hit_at=24, free_to=23)
            for backend in (good, bad):
                backend.integrate(1, seq, WINDOW, LINE_BASE, block)
            oracle.integrate(1, seq, WINDOW, LINE_BASE, block)
        self.assertEqual(logodds_at(good, (5, 0, 0)), oracle.logodds((5, 0, 0)))
        self.assertNotEqual(logodds_at(bad, (5, 0, 0)), oracle.logodds((5, 0, 0)))

    def differential(self, dims, frames, seed):
        rng = np.random.default_rng(seed)
        kernel, reference, oracle = self.fresh_kernel(dims=dims), self.reference(dims=dims), PyOracle(dims)
        mx, my, mz = dims
        base = [-mx // 2, -my // 2, -mz // 4]
        for backend in (kernel, reference):
            self.assertTrue(backend.reset(3, base).ok)
        oracle.reset(3, base)
        shifts = 0
        for frame in range(1, frames + 1):
            shape = tuple(int(v) for v in rng.integers(1, 33, size=3))
            block = rng.choice(np.array([0, 0, 1, 2], dtype=np.uint8), size=shape)
            obs_base = [b + int(rng.integers(-24, m)) for b, m in zip(base, dims)]
            if frame % 7 == 0:
                move = [int(rng.integers(-3 * m, 3 * m)) for m in dims]  # often a full clear
            else:
                move = [int(rng.integers(-6, 7)) for _ in dims]
            base = [b + m for b, m in zip(base, move)]
            shifts += any(move)
            rk = kernel.integrate(3, frame, base, obs_base, block)
            rr = reference.integrate(3, frame, base, obs_base, block)
            status, changes = oracle.integrate(3, frame, base, obs_base, block)
            self.assertEqual((rk.status_name, rr.status_name, status), ("ok", "ok", "ok"))
            self.assertEqual(rk.changes, rr.changes)
            self.assertEqual(rk.changes, changes)
            self.assertEqual(rk.changed_cells, len(changes))
            for field in ("delta_seq", "window_base", "obs_outside", "obs_inside",
                          "shift_mask", "accepted_total"):
                self.assertEqual(getattr(rk, field), getattr(rr, field), field)
            _, local_k = kernel.snapshot()
            _, local_r = reference.snapshot()
            np.testing.assert_array_equal(local_k, local_r)
            np.testing.assert_array_equal(local_k.astype(np.int32), oracle.cells)
            queries = np.array([[base[0] + int(rng.integers(-5, mx + 5)),
                                 base[1] + int(rng.integers(-5, my + 5)),
                                 base[2] + int(rng.integers(-5, mz + 5))] for _ in range(64)])
            _, qk = kernel.query(queries)
            _, qr = reference.query(queries)
            for a, b in zip(qk, qr):
                np.testing.assert_array_equal(a, b)
        self.assertGreater(shifts, frames // 2)

    def test_random_differential_three_implementations_default_dims(self):
        self.differential(DIMS, 40, seed=20260926)

    def test_random_differential_other_dims_build(self):
        # Guards against hidden reliance on the default window dimensions.
        self.differential((16, 8, 4), 60, seed=7)
        self.differential((24, 13, 5), 60, seed=8)

    def test_power_up_bad_args_and_query_outside(self):
        kernel = self.fresh_kernel()
        block = line_block(hit_at=24, free_to=23)
        self.assertEqual(kernel.integrate(1, 1, WINDOW, LINE_BASE, block).status_name,
                         "uninitialized")
        self.assertEqual(kernel.snapshot()[0].status_name, "uninitialized")
        self.assertEqual(kernel.query([(0, 0, 0)])[0].status_name, "uninitialized")
        self.assertEqual(kernel.raw_call(9).status_name, "bad_args")
        self.assertTrue(kernel.reset(1, WINDOW).ok)
        with self.assertRaises(ValueError):
            kernel.integrate(1, 1, WINDOW, LINE_BASE, np.zeros((1, 1, 33), dtype=np.uint8))
        with self.assertRaises(ValueError):
            kernel.integrate(1, 1, (1 << 28, 0, 0), LINE_BASE, block)
        raw = kernel.raw_call(1, epoch=1, frame_seq=1, window_base=(1 << 28, 0, 0),
                              dims=(1, 1, 1))
        self.assertEqual(raw.status_name, "bad_args")
        self.assertEqual(raw.rejected_malformed_total, 1)
        self.assertTrue(kernel.integrate(1, 1, WINDOW, LINE_BASE, block).ok)
        result, (states, logodds, outside) = kernel.query(
            [(24, 0, 0), (WINDOW[0] - 1, 0, 0), (WINDOW[0] + 80, 0, 0), (0, 0, 2 ** 30)])
        self.assertTrue(result.ok)
        self.assertEqual(states.tolist(), [OCCUPIED, UNKNOWN, UNKNOWN, UNKNOWN])
        self.assertEqual(outside.tolist(), [False, True, True, True])
        self.assertEqual(logodds.tolist(), [32, 0, 0, 0])
        with self.assertRaises(ValueError):
            RollingKernelHost(kernel.lib._name)  # same file = same static map

    def test_worst_case_delta_capacity_never_overflows(self):
        # All 32 x 32 rows at the worst lane alignment (offset x = 7), every
        # cell changes: 5 records per row = the full 5120 capacity.
        kernel = self.fresh_kernel()
        kernel.reset(1, (0, 0, 0))
        block = np.full((32, 32, 32), OCCUPIED, dtype=np.uint8)
        r = kernel.integrate(1, 1, (0, 0, 0), (7, 0, 0), block)
        self.assertTrue(r.ok)
        self.assertEqual((r.changed_cells, len(r.changes), r.delta_overflow),
                         (32 ** 3, 32 ** 3, 0))
        self.assertEqual(int(r.header[11]), 5120)

    def test_cpp_testbench_passes_and_kernel_mutants_fail(self):
        flags = ["-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-Wno-unknown-pragmas",
                 "-Wno-unused-label", "-ffp-contract=off"]
        m7 = ROOT / "hw/m7"
        support = [str(m7 / "rolling_map_ref.cpp"), str(m7 / "ray_flags.cpp"),
                   str(m7 / "rolling_testbench.cpp")]
        for mutant in (None,) + tuple(m for m in MUTANTS if m != "DECAY"):
            name = mutant or "delivered"
            kernel_obj = self.dir / ("tbk-%s.o" % name)
            define = [] if mutant is None else ["-DFPGA_M7_ROLL_MUTANT_" + mutant]
            subprocess.run(["g++"] + flags + define + ["-c", str(m7 / "rolling_map.cpp"),
                                                       "-o", str(kernel_obj)],
                           check=True, capture_output=True, text=True)
            binary = self.dir / ("tb-%s" % name)
            subprocess.run(["g++"] + flags + [str(kernel_obj)] + support + ["-o", str(binary)],
                           check=True, capture_output=True, text=True)
            run = subprocess.run([str(binary)], capture_output=True, text=True)
            with self.subTest(mutant=name):
                if mutant is None:
                    self.assertEqual(run.returncode, 0, run.stdout)
                    self.assertIn("M7 rolling map testbench PASS", run.stdout)
                else:
                    self.assertNotEqual(run.returncode, 0)
                    self.assertIn("FAIL", run.stdout)
                    self.assertNotIn("PASS", run.stdout)


# ---- Host contract with the real per-frame chain ---------------------------

VOXEL = 0.05
FRAME_GRID = GridSpec(origin_m=(-2 * VOXEL, -16 * VOXEL, -4 * VOXEL), voxel_m=VOXEL,
                      shape=(32, 32, 8))
CAMERA = (0.025, 0.025, 0.025)  # centre of world voxel (0, 0, 0)
K = np.array([[40., 0., 4.], [0., 40., 4.], [0., 0., 1.]])
# Optical axis along odom +x: camera x -> -y, camera y -> -z, camera z -> +x.
T = np.eye(4)
T[:3, :3] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]
T[:3, 3] = CAMERA
WALL_CELL, BOX_CELL, PATH_CELL = (22, 0, 0), (10, 0, 0), (3, 0, 0)


def depth_image(*, box=False, nan_centre=False, wall=True):
    depth = np.full((9, 9), 1.1 if wall else np.nan, dtype=np.float32)  # wall x = 1.125
    if box:
        depth[3:6, 3:6] = 0.5  # box face x = 0.525 (voxel 10 centre)
    if nan_centre:
        depth[4, 4] = np.nan
    return depth


class FusionContractTests(RollingBase):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.frame_lib = compile_frame_native(cls.dir / "frame.so")

    def setUp(self):
        self.pipeline = M7Pipeline(FrameNativeM7(self.frame_lib), FRAME_GRID)
        self.fusion = RollingFusion(self.fresh_kernel(), voxel_m=VOXEL)
        self.mirror = RollingDeltaMirror(DIMS)
        self.frame_id = 0
        self.t = 1_000_000_000

    def frame(self, depth, *, pose_epoch="e1", pose_valid=True, advance=10_000_000):
        self.frame_id += 1
        self.t += advance
        identity = CaptureIdentity("cam0", self.frame_id, self.t, "mono", "cal-1", "odom",
                                   pose_epoch)
        pose = PoseAtCapture(self.t, "mono", "odom", pose_epoch, pose_valid, T.copy())
        out = self.pipeline.ingest_geometry(DepthFrame(identity, depth, K, pose),
                                            now_ns=self.t + 1_000_000, now_clock_id="mono")
        return out.navigation

    def feed(self, nav, *, robot=CAMERA, now=None, mirror=True):
        now = self.t + 2_000_000 if now is None else now
        result = self.fusion.ingest(nav, self.pipeline, now_ns=now, clock_id="mono",
                                    robot_position_m=robot)
        if result.accepted and mirror:
            ok, reason = self.mirror.apply(result.delta, now_ns=now)
            self.assertTrue(ok, reason)
            snap = self.fusion.snapshot(now_ns=now, clock_id="mono")
            self.assertTrue(snap.valid)
            self.assertEqual(self.mirror.window_base, snap.window_base)
            np.testing.assert_array_equal(self.mirror.states, snap.states)
        return result

    def logodds(self, *cells, now=None):
        now = self.t + 2_000_000 if now is None else now
        q = self.fusion.query_world(list(cells), now_ns=now, clock_id="mono")
        self.assertTrue(q.valid, q.reason)
        return [None if o else int(l) for l, o in zip(q.logodds, q.outside)]

    def test_end_to_end_wall_obstacle_occlusion_known_answers(self):
        for _ in range(3):
            self.assertTrue(self.feed(self.frame(depth_image())).accepted)
        self.assertEqual(self.logodds(WALL_CELL, PATH_CELL, BOX_CELL), [96, -32, -32])
        for expected in (0, 32, 64):
            self.assertTrue(self.feed(self.frame(depth_image(box=True))).accepted)
            self.assertEqual(self.logodds(BOX_CELL, WALL_CELL), [expected, 96])
        for k in range(1, 8):
            self.assertTrue(self.feed(self.frame(depth_image())).accepted)
            self.assertEqual(self.logodds(BOX_CELL), [64 - 12 * k])
        # Long occlusion and centre-invalid depth: the wall is never cleared.
        for i in range(30):
            self.assertTrue(self.feed(self.frame(depth_image(box=i < 15, nan_centre=i >= 15))).accepted)
            self.assertEqual(self.logodds(WALL_CELL), [96])
        self.assertEqual(self.fusion.counts["accepted"], 43)
        self.assertEqual(self.mirror.counts["accepted"], 43)

    def test_invalid_expired_and_missing_frames_never_clear(self):
        for _ in range(3):
            self.feed(self.frame(depth_image()))
        before = self.fusion.snapshot(now_ns=self.t + 2_000_000, clock_id="mono").logodds.copy()
        # Whole frame without valid depth: pipeline invalidates it, nothing fused.
        for _ in range(5):
            self.assertEqual(self.feed(self.frame(depth_image(wall=False))).reason, "no_valid_depth")
        # Invalid pose.
        self.assertEqual(self.feed(self.frame(depth_image(), pose_valid=False)).reason,
                         "invalid_capture_pose")
        # Expired: consumed 300 ms after acquisition (limit 200 ms).
        late = self.frame(depth_image(box=True))
        self.assertEqual(self.feed(late, now=self.t + 300_000_000).reason, "expired_output")
        # Superseded: an older output consumed after a newer frame was admitted.
        older = self.frame(depth_image(box=True))
        self.frame(depth_image(wall=False))
        self.assertEqual(self.feed(older).reason, "geometry_revoked")
        self.assertEqual(self.fusion.counts["dropped:no_valid_depth"], 5)
        self.assertEqual(self.fusion.counts["dropped:invalid_capture_pose"], 1)
        self.assertEqual(self.fusion.counts["dropped:expired_output"], 1)
        self.assertEqual(self.fusion.counts["dropped:geometry_revoked"], 1)
        # Timeout: the map reports stale, but keeps every cell.
        stale = self.fusion.query_world([WALL_CELL], now_ns=self.t + 10 ** 10, clock_id="mono")
        self.assertEqual((stale.valid, stale.reason, stale.states), (False, "map_stale", None))
        self.assertEqual(self.fusion.snapshot(now_ns=self.t + 10 ** 10, clock_id="mono").reason,
                         "map_stale")
        self.assertEqual(self.fusion.query_world([WALL_CELL], now_ns=self.t,
                                                 clock_id="other").reason, "clock_mismatch")
        # A later valid frame restores freshness; nothing was cleared meanwhile.
        self.feed(self.frame(depth_image(), advance=5_000_000_000))
        after = self.fusion.snapshot(now_ns=self.t + 2_000_000, clock_id="mono").logodds
        changed = np.argwhere(before != after)
        self.assertEqual(self.logodds(WALL_CELL, PATH_CELL), [96, -32])
        # Only cells the final frame observed may differ (same scene => none).
        self.assertEqual(len(changed), 0)

    def test_pose_epoch_change_resets_map_and_old_outputs_are_rejected(self):
        old_outputs = []
        for _ in range(3):
            nav = self.frame(depth_image())
            old_outputs.append(nav)
            result = self.feed(nav)
        old_delta = result.delta
        self.assertEqual(self.fusion.epoch, 1)
        # New pose epoch without reset: pipeline refuses, nothing fused.
        self.assertEqual(self.feed(self.frame(depth_image(), pose_epoch="e2")).reason,
                         "context_changed_reset_required")
        self.pipeline.reset()
        result = self.feed(self.frame(depth_image(), pose_epoch="e2"))
        self.assertTrue(result.accepted)
        self.assertEqual((self.fusion.epoch, result.delta.delta_seq), (2, 1))
        self.assertEqual(self.fusion.counts["map_reset"], 2)
        self.assertEqual(self.logodds(WALL_CELL, PATH_CELL), [32, -12])  # one frame only
        # Old-generation output: revoked at consumption, counted.
        self.assertEqual(self.feed(old_outputs[-1]).reason, "generation_or_context_revoked")
        self.assertEqual(self.fusion.counts["dropped:generation_or_context_revoked"], 1)
        # Old delta to the Nav2 mirror: stale epoch, counted, state untouched.
        states = self.mirror.states.copy()
        self.assertEqual(self.mirror.apply(old_delta, now_ns=self.t), (False, "stale_epoch"))
        self.assertEqual(self.mirror.counts["rejected:stale_epoch"], 1)
        np.testing.assert_array_equal(states, self.mirror.states)
        # Old epoch sent straight to the map core: rejected and counted there.
        direct = self.fusion.backend.integrate(1, 99, self.fusion.window_base_for(CAMERA),
                                               (-2, -16, -4), np.full((1, 1, 1), 2, np.uint8))
        self.assertEqual((direct.status_name, direct.rejected_epoch_total), ("stale_epoch", 1))

    def test_mirror_gap_duplicate_expired_and_resync(self):
        deltas = []
        for i in range(6):
            result = self.feed(self.frame(depth_image(box=i % 2 == 1)), mirror=False)
            deltas.append(result.delta)
        now = self.t + 2_000_000
        m = self.mirror
        self.assertEqual(m.apply(deltas[0], now_ns=now), (True, "applied"))
        self.assertEqual(m.apply(deltas[0], now_ns=now), (False, "duplicate_or_old_delta"))
        self.assertEqual(m.apply(deltas[2], now_ns=now), (False, "sequence_gap"))
        self.assertEqual(m.apply(deltas[1], now_ns=now), (False, "awaiting_resync"))
        snap = self.fusion.snapshot(now_ns=now, clock_id="mono")
        self.assertEqual(m.resync(snap), (True, "resynced"))
        np.testing.assert_array_equal(m.states, snap.states)
        # Expired delta (consumed after its deadline) forces another resync.
        result = self.feed(self.frame(depth_image()), mirror=False)
        late = result.delta.expires_at_ns + 1
        self.assertEqual(m.apply(result.delta, now_ns=late), (False, "expired_delta"))
        self.assertTrue(m.needs_resync)
        self.assertEqual(dict(m.counts), {
            "epoch_start": 1, "accepted": 1, "rejected:duplicate_or_old_delta": 1,
            "rejected:sequence_gap": 1, "rejected:awaiting_resync": 1, "resync": 1,
            "rejected:expired_delta": 1})

    def test_window_follows_robot_and_world_cells_stay_put(self):
        for _ in range(3):
            self.feed(self.frame(depth_image()))
        # The robot drives +x; the camera block stays, so the window slides.
        for step in range(1, 12):
            robot = (CAMERA[0] + step * 4 * VOXEL, CAMERA[1] - (step // 4) * VOXEL, CAMERA[2])
            result = self.feed(self.frame(depth_image()), robot=robot)
            self.assertTrue(result.accepted)
            base = self.fusion.window_base_for(robot)
            self.assertEqual(result.delta.window_base, base)
            wall = self.logodds(WALL_CELL)[0]
            inside = 0 <= WALL_CELL[0] - base[0] < DIMS[0]
            self.assertEqual(wall, 96 if inside else None)
        self.assertGreater(result.kernel.obs_outside, 0)
        self.assertEqual(self.mirror.counts["accepted"], 14)

    def test_input_contract_rejections(self):
        nav = self.frame(depth_image())
        other_voxel = RollingFusion(self.fresh_kernel(), voxel_m=0.04)
        self.assertEqual(other_voxel.ingest(nav, self.pipeline, now_ns=self.t + 2_000_000,
                                            clock_id="mono", robot_position_m=CAMERA).reason,
                         "grid_not_on_map_lattice")
        display = self.pipeline.ingest_geometry(
            DepthFrame(CaptureIdentity("cam0", 99, self.t + 5, "mono", "cal-1", "odom", "e1"),
                       depth_image(), K,
                       PoseAtCapture(self.t + 5, "mono", "odom", "e1", True, T.copy())),
            now_ns=self.t + 6, now_clock_id="mono").display
        with self.assertRaises(ValueError):
            self.fusion.ingest(display, self.pipeline, now_ns=self.t + 7, clock_id="mono",
                               robot_position_m=CAMERA)
        misaligned = M7Pipeline(FrameNativeM7(self.frame_lib),
                                GridSpec(origin_m=(-0.1 + 0.01, -0.8, -0.2), voxel_m=VOXEL,
                                         shape=(32, 32, 8)))
        fusion = RollingFusion(self.fresh_kernel(), voxel_m=VOXEL)
        identity = CaptureIdentity("cam0", 1, self.t, "mono", "cal-1", "odom", "e1")
        out = misaligned.ingest_geometry(
            DepthFrame(identity, depth_image(), K,
                       PoseAtCapture(self.t, "mono", "odom", "e1", True, T.copy())),
            now_ns=self.t + 1, now_clock_id="mono").navigation
        self.assertTrue(out.valid)
        self.assertEqual(fusion.ingest(out, misaligned, now_ns=self.t + 2, clock_id="mono",
                                       robot_position_m=CAMERA).reason, "grid_not_on_map_lattice")
        self.assertEqual(fusion.counts["dropped:grid_not_on_map_lattice"], 1)
        with self.assertRaises(ValueError):
            RollingFusion(self.fresh_kernel(), voxel_m=VOXEL, window_anchor_voxels=(80, 0, 0))
        with self.assertRaises(ValueError):
            self.fusion.window_base_for((np.nan, 0, 0))


if __name__ == "__main__":
    unittest.main()
