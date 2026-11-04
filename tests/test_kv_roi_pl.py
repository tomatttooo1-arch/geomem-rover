"""PL ROI driver (host/kv_roi_pl.py) on the host: mock device, gate, kv_server wiring.

Known answers: the PL path (driver + MockRoiWindow running the same g++ kernel behind the
7784f75 register map) equals the CPU path (CpuRoiCore) row for row on the same inputs;
the gate refuses on the 0x0117bc0a layout and on every partial merge layout; kv_server
refuses --roi-core pl without the node and serves it end to end with --device mock.
Positive controls: a row shifted by one slot and one box judged with a wrong threshold
must be caught (the first by the Nano row check, the second only by the same-input
comparison -- its counts stay self-consistent); the register map is re-read from the
merge branch header. Also: kv_regs.KvCounterFrameFeed (the old KV counter, now only the
positive control of tests/test_permit_nano_frameid.py) writes a strictly increasing
NEXT_FRAME_ID and nothing else.
Run: PYTHONPATH=. python3 -m unittest discover -s tests -p test_kv_roi_pl.py
"""
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from host import kv_regs, kv_roi_pl
from host.kv_link import CpuRoiCore, check_roi_answer
from host.kv_regs import KvCounterFrameFeed, mock_permit, mock_sysinfo
from host.kv_roi_pl import (AP_IDLE, ROI_REGS, MockRoiWindow, PlRoiCore, RoiDeviceError,
                            RoiUnavailable, open_roi_window)
from host.m7_object_native import (ROI_FIELDS, NativeObjectEvidence, RoiBatch, RoiRow,
                                   compile_object_native)
from host.m7_pl_device import MockPlDevice
from perception.fpga_replay.core import Config

ROOT = Path(__file__).resolve().parents[1]
MERGE_COMMIT = "7784f75"


class Env:
    _dir = None

    @classmethod
    def get(cls):
        if cls._dir is None:
            cls._tmp = tempfile.TemporaryDirectory()
            cls._dir = Path(cls._tmp.name)
            cls.obj_lib = compile_object_native(cls._dir / "obj.so")
            cls.native = NativeObjectEvidence(cls.obj_lib)
        return cls


def roi_dict(local, cfg):
    rows = []
    for row_id, roi, near, far in local:
        if roi is None:
            rows.append((row_id, 0, 0, 0, 0, 0, 0., 0.))
        else:
            rows.append((row_id, 1) + tuple(roi) + (near, far))
    return {"memory_revision": 0, "rows": rows,
            "cfg": {"depth_margin_m": cfg.depth_margin_m, "min_pixels": cfg.min_pixels,
                    "min_valid_fraction": cfg.min_valid_fraction, "free_fraction": cfg.free_fraction}}


def pl_core(mutant=None, **kw):
    env = Env.get()
    dev = MockPlDevice()
    win = MockRoiWindow(dev._memory, env.native.lib, mutant=mutant, **kw)
    return PlRoiCore(win, dev), win


def scene(rng, h, w, n, margin=.04):
    f = np.float32
    specials = np.array([f(.1), np.nextafter(f(.1), f(1)), np.nextafter(f(10.), f(0)), f(10.),
                         np.nan, np.inf, -np.inf, f(0.), f(-1.)], np.float32)
    depth = rng.uniform(.05, 3.5, size=(h, w)).astype(np.float32)
    mask = rng.random((h, w)) < .2
    depth[mask] = rng.choice(specials, size=int(mask.sum()))
    local = []
    for b in range(n):
        if rng.random() < .15:
            local.append((100 + b, None, 0., 0.))
            continue
        x0 = int(rng.integers(0, w - 4)); y0 = int(rng.integers(0, h - 4))
        x1 = int(rng.integers(x0 + 3, w)); y1 = int(rng.integers(y0 + 3, h))
        near = float(rng.uniform(.2, 3.)); far = near + float(rng.uniform(0, .6))
        local.append((100 + b, (x0, y0, x1, y1), near, far))
    return depth, local, Config(depth_margin_m=margin)


def batch_of(local, cfg):
    rows = [RoiRow(row_id=r, enabled=roi is not None, xyxy=roi, near_z=nz, far_z=fz)
            for r, roi, nz, fz in local]
    return RoiBatch(rows=rows, cfg=cfg, memory_revision=0)


def answer_of(out):
    return {"memory_revision": 0, "roi_status": out["status"], "rows": out["rows"],
            "roi_core": "pl", "times_ns": out["times_ns"], "roi_cycles": None}


class SameInputTests(unittest.TestCase):
    def test_pl_mock_equals_cpu_row_for_row(self):
        env = Env.get()
        cpu = CpuRoiCore(env.obj_lib)
        pl, win = pl_core()
        rng = np.random.default_rng(20261011)
        rows = 0
        for trial in range(150):
            h, w = int(rng.integers(8, 80)), int(rng.integers(8, 120))
            depth, local, cfg = scene(rng, h, w, int(rng.integers(1, 33)),
                                      margin=float(rng.choice([.04, .5])))
            a, b = cpu(depth, roi_dict(local, cfg)), pl(depth, roi_dict(local, cfg))
            self.assertEqual((a["status"], a["rows"]), (b["status"], b["rows"]), trial)
            self.assertEqual(b["status"], 0)
            rows += len(b["rows"])
        self.assertGreater(rows, 1500)
        self.assertEqual(win.starts, 150)
        # 640x400 rec30-sized frame, 32 boxes
        depth, local, cfg = scene(rng, 400, 640, 32)
        self.assertEqual(cpu(depth, roi_dict(local, cfg))["rows"], pl(depth, roi_dict(local, cfg))["rows"])

    def test_status_and_stale_table_rules(self):
        """status != 0 -> no rows (the PL table still holds the last call); rows past
        box_count are never read even though the PL table has them."""
        env = Env.get()
        cpu = CpuRoiCore(env.obj_lib)
        pl, win = pl_core()
        rng = np.random.default_rng(7)
        depth, local, cfg = scene(rng, 40, 60, 20)
        full = pl(depth, roi_dict(local, cfg))
        self.assertEqual(len(full["rows"]), 20)
        small = pl(depth, roi_dict(local[:3], cfg))
        self.assertEqual(small["rows"], cpu(depth, roi_dict(local[:3], cfg))["rows"])
        self.assertEqual(len(small["rows"]), 3)
        stale = win.words[ROI_REGS["RESULT_BASE"] + 4 * 7 * 10]     # row 10 of the first call
        self.assertEqual(stale, full["rows"][10][0])
        bad = roi_dict([(0, (0, 0, 2, 2), 1., 1.)], Config())        # 3-pixel-wide box -> 3
        a, b = cpu(np.ones((10, 10), np.float32), bad), pl(np.ones((10, 10), np.float32), bad)
        self.assertEqual((a["status"], a["rows"]), (3, []))
        self.assertEqual((b["status"], b["rows"]), (3, []))
        self.assertEqual(win.words[ROI_REGS["RESULT_BASE"]], full["rows"][0][0])   # untouched

    def test_positive_controls_shift_and_threshold(self):
        env = Env.get()
        cpu = CpuRoiCore(env.obj_lib)
        rng = np.random.default_rng(3)
        # boxes in a flat 1.0 m wall; box 0's near/far straddle it so a +0.25 m near_z
        # key moves its FRONT count (wall now "in front") and its verdict
        depth = np.full((60, 80), 1.0, np.float32)
        local = [(10, (2, 2, 20, 20), 1.1, 1.3), (11, (30, 5, 50, 30), .5, .7),
                 (12, (55, 30, 75, 55), 1.2, 1.4), (13, (5, 35, 25, 55), .8, 1.2)]
        cfg = Config()
        ref = cpu(depth, roi_dict(local, cfg))
        batch = batch_of(local, cfg)
        self.assertTrue(check_roi_answer(batch, answer_of(ref)).ok)
        for name, kw in (("row_shift", {}), ("threshold", {"mutant_box": 1, "mutant_dz": .9})):
            core, _ = pl_core(mutant=name, **kw)
            got = core(depth, roi_dict(local, cfg))
            self.assertEqual(got["status"], 0, name)
            self.assertNotEqual(got["rows"], ref["rows"], name)       # the mutant fires
            caught_by_row_check = not check_roi_answer(batch, answer_of(got)).ok
            if name == "row_shift":                                    # OBJECT_ID echo
                self.assertTrue(caught_by_row_check)
                self.assertIn("row_check", check_roi_answer(batch, answer_of(got)).reason)
            else:                                                      # counts consistent:
                self.assertFalse(caught_by_row_check)                  # only same-input compare
                diff = [i for i, (a, b) in enumerate(zip(ref["rows"], got["rows"])) if a != b]
                self.assertEqual(diff, [1])
                self.assertNotEqual(ref["rows"][1][6], got["rows"][1][6])  # verdict moved

    def test_device_errors(self):
        core, win = pl_core(hang=True)
        core.timeout_s = 0.05
        rng = np.random.default_rng(1)
        depth, local, cfg = scene(rng, 20, 30, 3)
        with self.assertRaises(RoiDeviceError):
            core(depth, roi_dict(local, cfg))
        with self.assertRaises(RoiDeviceError):                          # now busy
            core(depth, roi_dict(local, cfg))
        self.assertEqual(core.counts["device_timeout"], 1)
        self.assertEqual(core.counts["device_busy"], 1)

    def test_register_map_matches_merge_header(self):
        try:
            text = subprocess.run(["git", "-C", str(ROOT), "show",
                                   MERGE_COMMIT + ":hw/m7/pl/sw/m7pl_hls_regs.h"],
                                  capture_output=True, text=True, check=True).stdout
        except (OSError, subprocess.CalledProcessError):
            self.skipTest("no git / merge commit")
        hdr = dict((m.group(1), int(m.group(2), 16)) for m in re.finditer(
            r"#define XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_(\w+)\s+(0x[0-9a-fA-F]+)", text))
        self.assertGreater(len(hdr), 15)
        for name, off in ROI_REGS.items():
            self.assertEqual(hdr[name], off, name)
        # positive control: a wrong entry would be caught
        self.assertNotEqual(hdr["NEAR_Z_BASE"], ROI_REGS["NEAR_Z_BASE"] + 8)
        addr = subprocess.run(["git", "-C", str(ROOT), "show", MERGE_COMMIT +
                               ":hw/m7/pl/sw/m7pl_addrmap.h"], capture_output=True, text=True).stdout
        self.assertIn("#define M7PL_BASE_ROI 0x%XU" % kv_roi_pl.ROI_BASE, addr)
        self.assertIn("#define M7PL_IRQ_BIT_ROI 2", addr)
        dts = subprocess.run(["git", "-C", str(ROOT), "show", MERGE_COMMIT +
                              ":hw/m7/pl/overlay/m7_pl.dtso"], capture_output=True, text=True).stdout
        self.assertIn("%s: %s@a0090000" % (kv_roi_pl.ROI_UIO, kv_roi_pl.ROI_UIO), dts)


# ------------------------------------------------------------------ gate
def fake_sysfs(root, nodes):
    """nodes = {index: (name, map_addr, words{off: value})} -> (sysfs, dev)."""
    sysfs, dev = os.path.join(root, "sys"), os.path.join(root, "dev")
    os.makedirs(dev, exist_ok=True)
    os.makedirs(sysfs, exist_ok=True)
    for n, (name, addr, words) in nodes.items():
        d = os.path.join(sysfs, "uio%d" % n, "maps", "map0")
        os.makedirs(d)
        open(os.path.join(sysfs, "uio%d" % n, "name"), "w").write(name + "\n")
        open(os.path.join(d, "addr"), "w").write("0x%x\n" % addr)
        open(os.path.join(d, "size"), "w").write("0x10000\n")
        buf = bytearray(0x10000)
        for off, v in words.items():
            struct.pack_into("<I", buf, off, v)
        open(os.path.join(dev, "uio%d" % n), "wb").write(bytes(buf))
    return sysfs, dev


def sysinfo_words(build_id, slot_map):
    return {0x00: 0x4D37504C, 0x04: 0x10000, 0x08: build_id, 0x10: 100000000, 0x20: slot_map}


class Opens:
    def __init__(self):
        self.calls = []

    def __call__(self, path, flags):
        self.calls.append(path)
        return os.open(path, flags)


BASE_NODES = {0: ("m7_sysinfo", 0xA0000000, sysinfo_words(0x0117BC0A, 0x1F)),
              1: ("m7_frame", 0xA0010000, {}), 2: ("m7_object", 0xA0020000, {})}
MERGE_NODES = {0: ("m7_sysinfo", 0xA0000000, sysinfo_words(0x07784F75, 0x3D7)),
               1: ("m7_frame", 0xA0010000, {}), 7: ("m7_roi", 0xA0090000, {0: AP_IDLE})}


def _with(nodes, extra):
    out = dict(nodes)
    out.update(extra)
    return out


class GateTests(unittest.TestCase):
    def _gate(self, nodes):
        with tempfile.TemporaryDirectory() as t:
            sysfs, dev = fake_sysfs(t, nodes)
            op = Opens()
            sysinfo = kv_regs.PlRegBlock.open("sysinfo", sysfs_root=sysfs, dev_root=dev, opener=op)
            try:
                win, detail = open_roi_window(sysinfo, sysfs_root=sysfs, dev_root=dev, opener=op)
                win.close()
                return detail, None, op.calls
            except RoiUnavailable as exc:
                return None, str(exc), op.calls
            finally:
                sysinfo.close()

    def test_merge_layout_opens(self):
        detail, err, calls = self._gate(MERGE_NODES)
        self.assertIsNone(err)
        self.assertEqual((detail["build_id"], detail["slot_map"], detail["map_addr"]),
                         ("0x07784f75", "0x3d7", "0xa0090000"))
        self.assertEqual([os.path.basename(c) for c in calls], ["uio0", "uio7"])

    def test_baseline_refused_without_touching_any_other_node(self):
        detail, err, calls = self._gate(BASE_NODES)
        self.assertIn("no ROI slot", err)
        self.assertEqual([os.path.basename(c) for c in calls], ["uio0"])      # sysinfo only

    def test_partial_layouts_refused(self):
        cases = {
            "no sysinfo": ({k: v for k, v in MERGE_NODES.items() if k}, "sysinfo block unavailable"),
            "bad sysinfo magic": (_with(MERGE_NODES, {0: ("m7_sysinfo", 0xA0000000, {0: 1})}),
                                  "sysinfo block unavailable"),
            "slot bit clear": (_with(MERGE_NODES, {0: ("m7_sysinfo", 0xA0000000,
                                                         sysinfo_words(0x07784F75, 0x1D7))}),
                               "no ROI slot"),
            "no node": ({k: v for k, v in MERGE_NODES.items() if k != 7}, "no UIO node named"),
            "wrong address": (_with(MERGE_NODES, {7: ("m7_roi", 0xA0080000, {0: AP_IDLE})}),
                              "is not the ROI slot"),
            "not idle": (_with(MERGE_NODES, {7: ("m7_roi", 0xA0090000, {0: 0})}), "not idle"),
        }
        for name, (nodes, want) in cases.items():
            detail, err, _ = self._gate(nodes)
            self.assertIsNotNone(err, name)
            self.assertIn(want, err, name)

    def test_kv_server_refuses_and_rejects_combinations(self):
        env = dict(os.environ, PYTHONPATH=str(ROOT))
        tool = str(ROOT / "tools/kv_link/kv_server.py")
        with tempfile.TemporaryDirectory() as t:
            sysfs, _ = fake_sysfs(t, BASE_NODES)
            sysfs, dev = sysfs, os.path.join(t, "dev")
            r = subprocess.run([sys.executable, tool, "--core", "pl", "--device", "devmem", "--wait",
                                "poll", "--roi-core", "pl", "--uio-sysfs", sysfs, "--uio-dev", dev,
                                "--port", "0"],
                               capture_output=True, text=True, env=env, timeout=60)
            self.assertNotEqual(r.returncode, 0)
            self.assertIn("no ROI slot", r.stderr)
            self.assertIn("Not starting", r.stderr)
            self.assertNotIn("/dev/" + "mem", r.stderr)        # refused before any device open
            for bad in (["--core", "cpu", "--roi-core", "pl"],
                        ["--core", "pl", "--device", "mock", "--wait", "poll", "--roi-core", "cpu",
                         "--roi-mock-mutant", "row_shift"]):
                r = subprocess.run([sys.executable, tool, "--port", "0"] + bad, capture_output=True,
                                   text=True, env=env, timeout=60)
                self.assertEqual(r.returncode, 2, bad)


# ------------------------------------------------------------------ permit feed
class PermitFeedTests(unittest.TestCase):
    def test_strictly_increasing_next_frame_id_and_only_that_register(self):
        perm, win = mock_permit()
        win.words[kv_regs.PERMIT_LAST_FRAME_ID] = 41
        feed = KvCounterFrameFeed(perm)
        for i in range(5):
            self.assertTrue(feed.admit("s", i).run)
        self.assertEqual(win.writes, [(kv_regs.PERMIT_NEXT_FRAME_ID, 42 + i) for i in range(5)])
        self.assertEqual(feed.describe()["counts"], {"written": 5, "verdict:kv_counter": 5})
        broken = kv_regs.PlRegBlock("permit", None, reason="absent")
        with self.assertRaises(kv_regs.RegisterReadError):
            KvCounterFrameFeed(broken)

    def test_regs_source_has_exactly_two_permit_writes(self):
        src = (ROOT / "host/kv_regs.py").read_text()
        writes = re.findall(r"self\.window\.wr\(([A-Z_]+)", src)
        self.assertEqual(sorted(writes), ["PERMIT_CTRL", "PERMIT_NEXT_FRAME_ID", "PERMIT_WINDOW"])
        win_callers = sorted(str(p.relative_to(ROOT)) for p in list((ROOT / "host").glob("*.py")) +
                             list((ROOT / "tools/kv_link").glob("*.py"))
                             if "write_permit_window(" in p.read_text())
        self.assertEqual(win_callers, ["host/kv_regs.py", "tools/kv_link/kv_server.py"])  # v3fc2: start only
        callers = sorted(str(p.relative_to(ROOT)) for p in list((ROOT / "host").glob("*.py")) +
                         list((ROOT / "tools/kv_link").glob("*.py"))
                         if "write_permit_next_frame_id(" in p.read_text())
        self.assertEqual(callers, ["host/kv_regs.py"])       # only the permit feeds call it


# ------------------------------------------------------------------ kv_server end to end
def codes_for(i, h=64, w=96):
    yy, xx = np.mgrid[0:h, 0:w]
    c = (650 + 300 * np.sin(xx / 11. + i) + 150 * np.cos(yy / 7. - i)).astype(np.uint16)
    c[(xx * 7 + yy * 3 + i) % 13 == 0] = 0
    return c


class ServerEndToEndTests(unittest.TestCase):
    def _run(self, mutant):
        env = dict(os.environ, PYTHONPATH=str(ROOT))
        cmd = [sys.executable, str(ROOT / "tools/kv_link/kv_server.py"), "--bind", "127.0.0.1",
               "--port", "0", "--max-idle-s", "60", "--core", "pl", "--device", "mock",
               "--wait", "poll", "--roi-core", "pl"]
        if mutant:
            cmd += ["--roi-mock-mutant", mutant]
        srv = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
        try:
            port = json.loads(srv.stdout.readline())["port"]
            with tempfile.TemporaryDirectory() as t:
                prep = os.path.join(t, "prep")
                os.makedirs(prep)
                n = 8
                np.stack([codes_for(i) for i in range(n)]).astype("<u2").tofile(
                    os.path.join(prep, "depth_u16.bin"))
                json.dump({"width": 96, "height": 64, "frames": n, "depth_file": "depth_u16.bin",
                           "depth_unit_m": 0.001,
                           "K": [[80.0, 0.0, 47.5], [0.0, 80.0, 31.5], [0.0, 0.0, 1.0]],
                           "calibration_id": "synthetic",
                           "grid": {"origin_m": [-0.8, -0.8, -0.05], "voxel_m": 0.05,
                                    "shape_xyz": [32, 32, 32]}},
                          open(os.path.join(prep, "meta.json"), "w"))
                snap = os.path.join(t, "snap.json")
                tool = str(ROOT / "tools/kv_link/nano_replay.py")
                mk = subprocess.run([sys.executable, tool, "--prep", prep, "--make-roi-memory", snap,
                                     "--roi-boxes", "20"], capture_output=True, text=True, env=env)
                self.assertEqual(mk.returncode, 0, mk.stderr[-2000:])
                out = os.path.join(t, "run")
                done = subprocess.run([sys.executable, tool, "--prep", prep, "--out", out, "--hz", "10",
                                       "--backend", "remote_pl", "--server", "127.0.0.1:%d" % port,
                                       "--chunk-bytes", "2000", "--compare", "--roi-memory", snap],
                                      capture_output=True, text=True, env=env)
                s = json.load(open(os.path.join(out, "summary.json")))
                # GEO_FX3 (2026-11-04): the PL frame core is the fixed-point core. nano_replay --compare
                # checks the geometry against the float CPU reference, which it need not equal (exit 1 then);
                # the geometry is checked here against its bit-exact model (tests/geofx3_ref.py) instead.
                import geofx3_ref
                checked, equal = geofx3_ref.replay_dump_equals_fx(
                    out, [codes_for(i) for i in range(n)], np.array([[80.0, 0.0, 47.5], [0.0, 80.0, 31.5],
                                                                     [0.0, 0.0, 1.0]]),
                    (-0.8, -0.8, -0.05), 0.05, (32, 32, 32))
                s["fx_geometry"] = {"frames_checked": checked, "frames_equal": equal,
                                    "equal": checked > 0 and checked == equal}
                cpu_equal = s["compare"].get("equal")
                self.assertEqual(done.returncode, 1 if (mutant or not cpu_equal) else 0, done.stderr[-3000:])
                return s
        finally:
            srv.terminate()
            srv.communicate(timeout=10)

    def test_pl_mock_end_to_end_and_mutants(self):
        s = self._run(None)
        r = s["roi"]
        self.assertTrue(s["fx_geometry"]["equal"], s["fx_geometry"])
        self.assertEqual(r["frames_ok"], s["answered"])
        self.assertGreater(r["plan"]["rows_sent"], 3)
        self.assertTrue(r["compare"]["equal"])
        self.assertTrue(r["compare"]["control_caught"])
        self.assertEqual(r["compare"]["rows_compared"], r["frames_ok"] * r["plan"]["rows_sent"])
        self.assertEqual(r["roi_core_seen"], {"pl": s["answered"]})
        shifted = self._run("row_shift")["roi"]
        self.assertEqual(shifted["frames_ok"], 0)                       # Nano row check voids
        self.assertTrue(all("row_check" in k for k in shifted["reasons"]), shifted["reasons"])
        thr = self._run("threshold")["roi"]
        self.assertGreater(thr["frames_ok"], 0)                         # passes the row check
        self.assertFalse(thr["compare"]["equal"])                       # same-input compare catches


if __name__ == "__main__":
    unittest.main()
