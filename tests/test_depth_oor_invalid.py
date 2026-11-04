"""Out-of-range depth pixels marked invalid instead of refusing the frame (2026-10-16).

Docs: docs/design/DEPTH_OOR_INVALID_20261016.md (criteria T-a .. T-f, positive controls P1 .. P3,
written before the run). Scene: the rolling Nav2 smoke renderer (64x48, mm codes) on the
replay mount (0.35 m, 27 deg down); ground z = 0. Frame core and rolling map: the HLS C++
compiled for the host (FrameNativeM7, RollingKernelHost); KV loopback: an in-process
KvLinkServer on 127.0.0.1 (ephemeral port) with the CPU core.

  T-a sparse far OOR (< frac_max, outside the near band): not refused, codes 0 at those pixels,
      the depth equals codes_to_m(codes) bit for bit, voxel flags equal the frame whose pixels
      were code 0 to begin with; rolling map: no cell FREE that the code-0 reference leaves
      not FREE.
  T-b one OOR pixel in the near band: refused (the m7_pipeline > 20 m refusal), backend not called.
  T-c fraction > frac_max: refused (gate fraction, near band not computed); == frac_max: not refused.
  T-d no ground_z_m: any OOR refused (gate no_ground).
  T-e mode reject: the T-a input refused exactly as on main, codes untouched.
  T-f KV loopback: crc passes, invalid count includes the OOR pixels, flags equal the local core.
  P1  invalid value = the longest valid range (9.9 m, inside the core's 0.1-10 m window): the
      "not free" check fails (new FREE voxels / cells).
  P2  rolling map UNKNOWN_CLEARS mutant: the "not free" map check fails.
  P3  = T-e (the old behaviour refuses the frame).
Also: ground_near_pixels known answers. Python 3.8+.
"""

import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
sys.path.insert(0, str(ROOT / "robot_sim" / "tests"))

import host.geo_producer as gp  # noqa: E402
from host.kv_link import codes_to_m  # noqa: E402

UNIT = 0.001
OOR_CODE = 25000          # 25 m at 1 mm codes: > 20 m
REFUSAL = "input_refused:depth appears outside indoor metre range"
UNKNOWN, FREE = 0, 1


class _Env(object):
    _tmp = None

    @classmethod
    def get(cls):
        if cls._tmp is None:
            from host.m7_frame_native import compile_frame_native
            cls._tmp = tempfile.TemporaryDirectory(prefix="oor_test_")
            cls.dir = Path(cls._tmp.name)
            cls.frame_lib = compile_frame_native(cls.dir / "frame.so")
            cls.n = 0
        return cls

    @classmethod
    def rolling(cls, mutant=None):
        from host.m7_rolling import RollingKernelHost, compile_rolling
        cls.get().n += 1
        return RollingKernelHost(compile_rolling(cls.dir / ("rolling%d.so" % cls.n), mutant=mutant))

    @classmethod
    def cpu(cls):
        from host.m7_frame_native import FrameNativeM7
        return FrameNativeM7(cls.get().frame_lib)


class _RecordingBackend(object):
    """Remote-style frame backend double (set_next_codes) over the CPU core; records calls."""
    backend_name = "remote_pl:fake"
    accepts_capture_identity = True

    def __init__(self, cpu):
        self.cpu, self.calls, self.lib = cpu, [], types.SimpleNamespace(last=None)
        self._next_roi = self._next_codes = None
        self.last_roi = None

    def set_next_roi(self, roi, labels=None):
        self._next_roi = roi

    def set_next_codes(self, codes):
        self._next_codes = codes

    def frame_from_depth(self, *, identity=None, **kw):
        codes, self._next_codes, self._next_roi = self._next_codes, None, None
        self.calls.append({"codes": None if codes is None else np.array(codes), "depth": np.array(kw["depth_m"])})
        return self.cpu.frame_from_depth(**kw)


def scene():
    import rolling_nav2_smoke as smoke
    from tools.car_geo_producer.common import base_camera, odom_base
    depth, _, _ = smoke.render((-0.6, 0.0, 0.0), smoke.WALLS + [smoke.BOX_A])
    codes = np.where(np.isfinite(depth), np.rint(depth / UNIT), 0).astype(np.uint16)
    return codes, smoke.intrinsics(), odom_base(0) @ base_camera()


class OorBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.codes, cls.K, cls.Toc = scene()
        cls.H, cls.W = cls.codes.shape
        near = gp.ground_near_pixels(np.arange(cls.H * cls.W), cls.W, cls.K, cls.Toc, 0.0, 1.5)
        cls.near = near.reshape(cls.H, cls.W)
        # the far patch: 8 x 6 pixels, rows 2..7 (outside the 1.5 m ground band), 48 / 3072 = 1.5625 %
        cls.patch = (slice(2, 8), slice(28, 36))
        assert not cls.near[cls.patch].any() and cls.near[-1, cls.W // 2]
        assert gp.ground_near_pixels([(cls.H - 1) * cls.W + cls.W // 2], cls.W, cls.K, cls.Toc, 0.0,
                                     gp.DEFAULT_OOR_NEAR_M)[0]

    def producer(self, rolling=None, **kw):
        now = [10 ** 12]
        backend = _RecordingBackend(_Env.cpu())
        kw.setdefault("ground_z_m", 0.0)
        prod = gp.GeoProducer(backend, rolling or _Env.rolling(), depth_unit_m=UNIT,
                              clock=lambda: now[0], roi=False, **kw)
        return prod, backend, now

    def run_frames(self, prod, now, codes, n=1):
        out = []
        for i in range(n):
            now[0] += 100_000_000
            out.append(prod.process(codes, self.K, depth_stamp_ns=i + 1, acquired_ns=now[0] - 1_000_000,
                                    T_odom_camera=self.Toc, robot_position_m=(0.0, 0.0, 0.0),
                                    calibration_id="c"))
        return out

    def with_patch(self, value, region=None):
        c = self.codes.copy()
        c[self.patch if region is None else region] = value
        return c

    def assert_not_free(self, test, ref, what):
        """No voxel/cell FREE in ``test`` that the code-0 reference ``ref`` leaves not FREE."""
        extra = int(np.count_nonzero((np.asarray(test) == FREE) & (np.asarray(ref) != FREE)))
        self.assertEqual(extra, 0, "%s: %d voxels/cells became FREE from invalid pixels" % (what, extra))


class SparseFarOor(OorBase):
    def test_a_sparse_far_oor_not_refused_and_unknown(self):
        prod, backend, now = self.producer()
        oor = self.with_patch(OOR_CODE)
        oor.setflags(write=False)                       # the 16UC1 buffer is read-only on the car
        r = self.run_frames(prod, now, oor)[0]
        self.assertEqual(r["nav_reason"], "geometry_ready")
        self.assertEqual(len(backend.calls), 1)
        self.assertEqual(r["oor"]["oor_pixels"], 48)
        self.assertEqual(r["oor"]["oor_near_pixels"], 0)
        self.assertEqual(r["oor"]["oor_action"], "invalidated")
        self.assertIsNone(r["oor"]["oor_gate"])
        self.assertEqual(r["oor"]["oor_max_m"], float(np.float32(OOR_CODE) * np.float32(UNIT)))
        self.assertEqual(int(oor[self.patch][0, 0]), OOR_CODE)   # caller's buffer untouched
        sent = backend.calls[0]["codes"]
        self.assertTrue((sent[self.patch] == 0).all())
        self.assertEqual(int(np.count_nonzero(sent != self.codes)), 48)
        self.assertEqual(codes_to_m(sent, np.float32(UNIT)).tobytes(), backend.calls[0]["depth"].tobytes())
        self.assertEqual(prod.counts["oor:invalidated"], 1)
        # reference: the same pixels code 0 (no return) from the start
        ref_prod, _, ref_now = self.producer()
        ref = self.run_frames(ref_prod, ref_now, self.with_patch(0))[0]
        self.assertEqual(ref["oor"]["oor_pixels"], 0)
        self.assertEqual(ref["voxel_flags"].tobytes(), r["voxel_flags"].tobytes())
        self.assert_not_free(r["voxel_flags"], ref["voxel_flags"], "invalidated frame")

    def test_p1_longest_valid_range_is_caught(self):
        """P1: the invalid value replaced by 9.9 m (inside 0.1-10 m) -> FREE voxels -> caught."""
        ref_prod, _, ref_now = self.producer()
        ref = self.run_frames(ref_prod, ref_now, self.with_patch(0))[0]
        far_prod, _, far_now = self.producer()
        far = self.run_frames(far_prod, far_now, self.with_patch(9900))[0]
        self.assertEqual(far["nav_reason"], "geometry_ready")
        with self.assertRaises(AssertionError):
            self.assert_not_free(far["voxel_flags"], ref["voxel_flags"], "9.9 m mutant")
        # and the same mutant inside the producer's own path (invalid code 9900 instead of 0)
        with patch.object(gp, "OOR_INVALID_CODE", 9900):
            mut_prod, mut_backend, mut_now = self.producer()
            mut = self.run_frames(mut_prod, mut_now, self.with_patch(OOR_CODE))[0]
        self.assertEqual(mut["nav_reason"], "geometry_ready")
        self.assertTrue((mut_backend.calls[0]["codes"][self.patch] == 9900).all())
        with self.assertRaises(AssertionError):
            self.assert_not_free(mut["voxel_flags"], ref["voxel_flags"], "producer 9.9 m mutant")

    def test_rolling_map_keeps_invalid_unknown_and_p2_mutant_caught(self):
        n = 12
        maps = {}
        for name, codes, mutant in (("ref", self.with_patch(0), None), ("oor", self.with_patch(OOR_CODE), None),
                                    ("far99", self.with_patch(9900), None),
                                    ("mutant", self.with_patch(OOR_CODE), "UNKNOWN_CLEARS")):
            prod, _, now = self.producer(rolling=_Env.rolling(mutant))
            rs = self.run_frames(prod, now, codes, n)
            self.assertTrue(all(r["result"] == "fused" for r in rs), (name, [r["result"] for r in rs]))
            snap = prod.fusion.snapshot(now_ns=now[0], clock_id=prod.clock_id)
            self.assertTrue(snap.valid, (name, snap.reason))
            maps[name] = snap.states
        self.assertEqual(maps["oor"].tobytes(), maps["ref"].tobytes())
        self.assert_not_free(maps["oor"], maps["ref"], "rolling map (invalidated)")
        self.assertGreater(int(np.count_nonzero(maps["ref"] == FREE)), 0)
        with self.assertRaises(AssertionError):     # P1 at the map layer
            self.assert_not_free(maps["far99"], maps["ref"], "rolling map (9.9 m)")
        with self.assertRaises(AssertionError):     # P2
            self.assert_not_free(maps["mutant"], maps["ref"], "rolling map (UNKNOWN_CLEARS)")


class Gates(OorBase):
    def test_b_near_band_refuses(self):
        prod, backend, now = self.producer()
        c = self.codes.copy()
        c[-1, self.W // 2] = OOR_CODE
        r = self.run_frames(prod, now, c)[0]
        self.assertTrue(r["nav_reason"].startswith(REFUSAL), r["nav_reason"])
        self.assertEqual(backend.calls, [])
        self.assertIsNone(r["delta"])
        self.assertEqual((r["oor"]["oor_gate"], r["oor"]["oor_action"]), ("near", "refused"))
        self.assertEqual((r["oor"]["oor_pixels"], r["oor"]["oor_near_pixels"]), (1, 1))
        # same pixel far away is fine, and the near pixel together with the far patch is not
        r2 = self.run_frames(prod, now, self.with_patch(OOR_CODE))[0]
        self.assertEqual(r2["nav_reason"], "geometry_ready")
        c3 = self.with_patch(OOR_CODE)
        c3[-1, self.W // 2] = OOR_CODE
        r3 = self.run_frames(prod, now, c3)[0]
        self.assertTrue(r3["nav_reason"].startswith(REFUSAL))
        self.assertEqual(r3["oor"]["oor_near_pixels"], 1)

    def test_c_fraction_gate_strict(self):
        prod, backend, now = self.producer()
        r = self.run_frames(prod, now, self.with_patch(OOR_CODE, (slice(0, 3), slice(None))))[0]   # 192/3072
        self.assertTrue(r["nav_reason"].startswith(REFUSAL))
        self.assertEqual(backend.calls, [])
        self.assertEqual(r["oor"]["oor_gate"], "fraction")
        self.assertIsNone(r["oor"]["oor_near_pixels"])
        self.assertAlmostEqual(r["oor"]["oor_fraction"], 0.0625)
        # exactly frac_max (48 / 3072 = 0.015625, exact in binary): not refused; 49: refused
        prod2, backend2, now2 = self.producer(oor_frac_max=48 / 3072.0)
        r2 = self.run_frames(prod2, now2, self.with_patch(OOR_CODE))[0]
        self.assertEqual(r2["nav_reason"], "geometry_ready")
        c = self.with_patch(OOR_CODE)
        c[1, 0] = OOR_CODE
        r3 = self.run_frames(prod2, now2, c)[0]
        self.assertEqual(r3["oor"]["oor_gate"], "fraction")
        self.assertEqual(len(backend2.calls), 1)

    def test_d_no_ground_refuses(self):
        prod, backend, now = self.producer(ground_z_m=None)
        r = self.run_frames(prod, now, self.with_patch(OOR_CODE))[0]
        self.assertTrue(r["nav_reason"].startswith(REFUSAL))
        self.assertEqual(r["oor"]["oor_gate"], "no_ground")
        self.assertEqual(backend.calls, [])
        # a frame without OOR pixels is unaffected
        r2 = self.run_frames(prod, now, self.codes)[0]
        self.assertEqual(r2["nav_reason"], "geometry_ready")
        self.assertEqual(r2["oor"], {"oor_pixels": 0, "oor_fraction": 0.0, "oor_near_pixels": 0,
                                     "oor_max_m": None, "oor_gate": None, "oor_action": "none"})

    def test_e_reject_mode_is_the_old_behaviour(self):
        prod, backend, now = self.producer(depth_oor="reject")
        r = self.run_frames(prod, now, self.with_patch(OOR_CODE))[0]
        self.assertTrue(r["nav_reason"].startswith(REFUSAL))
        self.assertEqual(backend.calls, [])
        self.assertIsNone(r["delta"])
        self.assertEqual(r["result"], "dropped:%s" % r["nav_reason"])
        self.assertEqual((r["oor"]["oor_action"], r["oor"]["oor_gate"]), ("refused", None))
        self.assertEqual(prod.counts["oor:refused"], 1)
        self.assertEqual(prod.describe()["depth_oor"]["mode"], "reject")
        # the M7Pipeline refusal itself (what main does with this depth)
        from host.m7_pipeline import CaptureIdentity, DepthFrame, PoseAtCapture
        ident = CaptureIdentity("s", 1, now[0], prod.clock_id, "c", "odom", "odom_v1")
        pose = PoseAtCapture(now[0], prod.clock_id, "odom", "odom_v1", True, self.Toc)
        with self.assertRaises(ValueError) as cm:
            prod.pipeline.ingest_geometry(DepthFrame(ident, codes_to_m(self.with_patch(OOR_CODE), np.float32(UNIT)),
                                                     self.K, pose), now_ns=now[0], now_clock_id=prod.clock_id)
        self.assertEqual(r["nav_reason"], "input_refused:%s" % str(cm.exception)[:60])

    def test_arguments_refused(self):
        with self.assertRaises(ValueError):
            gp.GeoProducer(None, None, depth_unit_m=UNIT, depth_oor="drop")
        with self.assertRaises(ValueError):
            gp.GeoProducer(None, None, depth_unit_m=UNIT, oor_frac_max=1.5)
        with self.assertRaises(ValueError):
            gp.GeoProducer(None, None, depth_unit_m=UNIT, oor_near_m=-1.0)


class NearBand(unittest.TestCase):
    def test_known_answers(self):
        # camera 0.5 m above the ground looking straight down / level / up (optical z = odom -z, x, z)
        K = np.array([[100.0, 0, 50], [0, 100.0, 50], [0, 0, 1]])
        down = np.eye(4)
        down[:3, :3] = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
        down[2, 3] = 0.5
        centre = 50 * 101 + 50
        self.assertTrue(gp.ground_near_pixels([centre], 101, K, down, 0.0, 0.0)[0])   # straight down: 0 m
        # pixel u=100 (x = 0.5 per unit z): ray (0.5, 0, -1) meets z=0 at t=0.5 -> 0.25 m away
        px = 50 * 101 + 100
        self.assertTrue(gp.ground_near_pixels([px], 101, K, down, 0.0, 0.25)[0])
        self.assertFalse(gp.ground_near_pixels([px], 101, K, down, 0.0, 0.2499)[0])
        level = np.eye(4)
        level[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]   # optical -> base (z forward -> x)
        level[2, 3] = 0.5
        top, bottom = 0 * 101 + 50, 100 * 101 + 50              # v=0 looks up, v=100 looks 26.6 deg down
        got = gp.ground_near_pixels([top, centre, bottom], 101, K, level, 0.0, 1.5)
        self.assertEqual(list(got), [False, False, True])        # bottom: 0.5 / tan(26.57) = 1.0 m
        self.assertFalse(gp.ground_near_pixels([bottom], 101, K, level, 0.0, 0.99)[0])
        below = level.copy()
        below[2, 3] = -0.1                                      # camera under the ground: conservative
        self.assertTrue(gp.ground_near_pixels([top], 101, K, below, 0.0, 1.5).all())


class T1Frames(unittest.TestCase):
    """The three OOR frames of T1 (10-16 board, 640x480, side_over.jsonl): their bounding boxes filled
    with 60 m on the run's adopted mount (0.380 m / 27.6 deg / roll -0.17). Default gates: none refused
    (no box pixel in the near band, fraction <= 2.6 %); the old 1.5 m band would refuse all three."""
    K = np.array([[517.1719360351562, 0.0, 324.2197265625], [0.0, 517.1170654296875, 240.64755249023438],
                  [0.0, 0.0, 1.0]])
    BOXES = ((92, 133, 472, 499), (16, 149, 26, 83), (108, 171, 0, 39))

    def test_t1_boxes_pass_default_and_old_band_refuses(self):
        from tools.car_geo_producer.common import base_camera
        T = base_camera(xyz=(0.1, 0.0, 0.3796361589154607), pitch_deg=27.6242382805504,
                        roll_deg=-0.16806484442220138)
        for y0, y1, x0, x1 in self.BOXES:
            d = np.full((480, 640), 1.0, dtype=np.float32)
            d[y0:y1 + 1, x0:x1 + 1] = 60.0
            mask, info = gp.screen_out_of_range(d, self.K, T, ground_z_m=0.0)
            self.assertEqual((info["oor_action"], info["oor_near_pixels"]), ("invalidated", 0), (y0, x0))
            self.assertEqual(int(mask.sum()), (y1 - y0 + 1) * (x1 - x0 + 1))
            _, old = gp.screen_out_of_range(d, self.K, T, ground_z_m=0.0, near_m=1.5)
            self.assertEqual(old["oor_gate"], "near")
        near = gp.ground_near_pixels(np.arange(480 * 640), 640, self.K, T, 0.0, gp.DEFAULT_OOR_NEAR_M)
        self.assertGreater(int(np.flatnonzero(near)[0]) // 640, 171)   # band starts below row 171


class KvLoopback(OorBase):
    def test_f_crc_and_counts_through_kv(self):
        from test_kv_link import cpu_core, remote, start_server
        srv = start_server(cpu_core())
        self.addCleanup(srv.stop)
        be = remote(srv.port, unit=UNIT)
        self.addCleanup(be.client.close)
        now = [10 ** 12]
        prod = gp.GeoProducer(be, _Env.rolling(), depth_unit_m=UNIT, clock=lambda: now[0], roi=False,
                              ground_z_m=0.0)
        oor = self.with_patch(OOR_CODE)
        r = self.run_frames(prod, now, oor)[0]
        self.assertEqual(r["nav_reason"], "geometry_ready", r["nav_reason"])
        self.assertEqual(r["oor"]["oor_action"], "invalidated")
        self.assertEqual(be.counts["codes_supplied"], 1)
        local_prod, _, local_now = self.producer()
        local = self.run_frames(local_prod, local_now, self.with_patch(0))[0]
        self.assertEqual(r["voxel_flags"].tobytes(), local["voxel_flags"].tobytes())
        d = codes_to_m(self.with_patch(0), np.float32(UNIT))
        expect_invalid = int(np.count_nonzero(~((d > 0.1) & (d < 10))))
        self.assertGreaterEqual(expect_invalid, 48)
        self.assertEqual(r["nav_counts"]["invalid_depth_or_projection"], expect_invalid)
        self.assertEqual(r["nav_counts"], local["nav_counts"])


if __name__ == "__main__":
    unittest.main()
