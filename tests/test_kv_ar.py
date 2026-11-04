"""Car AR display in kv_server (--ar drm, docs/design/CAR_AR_RESULTS_20261011.md).

What is checked, each with a known answer or an independent oracle and a positive control:
  * register offsets of tools/kv_link/ar_helper/m7ar_regs.h == the exported IP header of
    m7_pl_merge (git show 7784f75:hw/m7/pl/sw/m7pl_hls_regs.h), entry by entry, and the
    helper's register order == golden.REG_NAMES == host.kv_ar.REG_NAMES;
  * the one age formula: host.kv_ar.ar_age_ms and the helper's age_ms_of (--age-selftest) on
    known answers (ns carry, KV duration negative, saturation);
  * REQ AGE / LABEL blocks: round trip, crc / version / count / padding / character / rsv
    defects -> BadExtension (whole request void), LABEL without ROI, servers that do not
    announce the feature void the bit, no-extension bytes == b43daa1;
  * fill_ar_inputs against an oracle written here from the request semantics (depth codes,
    x/z-asymmetric 32^3 flags, a translated world_camera, ROI rows with disabled /
    out-of-range / verdict 0,1,2,3 rows, class names incl. empty / long / unsupported glyphs),
    then both rendered by the numpy golden (boxes and labels really drawn);
  * positive controls: text mutants of host/kv_ar.py (anchor must hit exactly once): label
    row shift, label misalignment, label truncation, missing fallback, box off by one,
    verdict forced 0, replay polarity, frame_valid constant 1 -- each caught by the oracle,
    and its golden pixel difference lies only in that mutant's region;
  * the helper (host build, mock provider): newest-wins (3 messages in one write -> one
    render, superseded 2), fail-safe STALE on its own timer and after stdin EOF, dumps ==
    golden (and C reference);
  * kv_server: the snapshot handed to the presenter (real server path, CPU ROI core) equals
    the client's view; --ar off keeps HELLO features and the TELEM ar block of b43daa1;
    end to end kv_server --ar drm (mock) + nano_replay --roi-memory: labels on screen,
    TELEM ar block fields, fail-safe after the stream stops, dumps == golden and C ref;
    10-12: nano_replay marks every request as replay -> panel REPLAY + age, TELEM ar mode.
    Replay mark / age rules in detail: tests/test_kv_ar_replay.py; odom-block mini map:
    tests/test_kv_ar_slice.py.

Run: PYTHONHASHSEED=0 PYTHONPATH=. python3 -m unittest discover -s tests -p test_kv_ar.py
(also /usr/bin/python3.8). Builds the helper (host) with g++ + the Vitis HLS headers.
"""
import json
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest
import zlib

import numpy as np

from host import kv_ar, kv_link
from host.kv_link import (AGE_BLOCK_BYTES, LABEL_HEAD, REQ_HEAD, BadExtension, CpuRoiCore,
                          KvLinkClient, KvTelemetryClient, KvLinkServer, LinkTimeout,
                          RemotePlFrameBackend, clean_wire_label, decode_request,
                          encode_age_block, encode_label_block, encode_request, f32_bits)
from host.m7_frame_native import FrameNativeM7, compile_frame_native
from host.m7_object_native import ROI_FIELDS, RoiBatch, RoiRow, compile_object_native
from host.m7_pipeline import CaptureIdentity
from host.fpga_clock import now_ns
from perception.fpga_replay.core import Config
from tools.geo_same_input.common import depth_u16_to_m

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "ar_display"))
sys.path.insert(0, str(ROOT / "tools" / "kv_link"))
import golden  # noqa: E402
import ar_dump_check  # noqa: E402

WD, HD = 96, 64
UNIT = 0.001
ORIGIN, VOXEL = (-0.8, -0.8, -0.05), 0.05
K = np.array([[80.0, 0.0, 47.5], [0.0, 80.0, 31.5], [0.0, 0.0, 1.0]])
CLOCK, LOCAL, EPOCH = "test_raw", "cam", "e1"
CANVAS = (640, 480)
GLYPHS = set("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789:.?>/-+= ")


def load_old(path, rev):
    src = subprocess.run(["git", "show", "%s:%s" % (rev, path)], cwd=str(ROOT),
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True).stdout
    mod = types.ModuleType("old_%s_%s" % (rev, re.sub(r"\W", "_", path)))
    mod.__file__ = str(ROOT / path)
    exec(compile(src, "%s@%s" % (path, rev), "exec"), mod.__dict__)
    return mod


class Env:
    _dir = None

    @classmethod
    def get(cls):
        if cls._dir is None:
            cls._tmp = tempfile.TemporaryDirectory(prefix="test_kv_ar_")
            cls._dir = Path(cls._tmp.name)
            cls.frame_lib = compile_frame_native(cls._dir / "frame.so")
            cls.obj_lib = compile_object_native(cls._dir / "obj.so")
        return cls

    @classmethod
    def helper(cls):
        env = cls.get()
        if not hasattr(env, "helper_bin"):
            pre = os.environ.get("KV_AR_HELPER")
            if pre:
                env.helper_bin = pre
            else:
                build = env._dir / "ar_helper"
                r = subprocess.run(["make", "-C", str(ROOT / "tools/kv_link/ar_helper"),
                                    "BUILD=%s" % build, "host"], stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT)
                if r.returncode != 0:
                    raise RuntimeError("helper build failed:\n%s" % r.stdout.decode()[-3000:])
                env.helper_bin = str(build / "host" / "m7ar_present")
        return env.helper_bin

    @classmethod
    def c_ref(cls):
        env = cls.get()
        if not hasattr(env, "c_lib"):
            env.c_lib = ar_dump_check.c_reference(str(env._dir / "cref"))
        return env.c_lib


# ------------------------------------------------------------------ the oracle (from the spec)
def want_label_text(row_id, name):
    """What the box label must read: class name upper-cased, first 4, unsupported glyph '-',
    padded with spaces; nothing usable -> R + three digits of the row id."""
    if name is not None and bytes(name).strip(b" "):
        s = bytes(name).decode("ascii").upper()[:4]
        s = "".join(ch if ch in GLYPHS else "-" for ch in s)
        return s + " " * (4 - len(s))
    return "R" + str(int(row_id) % 1000).zfill(3)


def word_of(text):
    return int.from_bytes(text.encode("ascii"), "little")


def text_of(word):
    return int(word).to_bytes(4, "little").decode("ascii")


def oracle(snap, lay, source, max_age_ms):
    """Expected AR core inputs of one answered frame, by register name."""
    wd, hd = int(snap["width"]), int(snap["height"])
    unit = np.frombuffer(struct.pack("<I", snap["unit_bits"]), "<f4")[0]
    z32 = np.asarray(snap["codes"]).ravel()[:wd * hd].astype(np.float32) * unit
    # validity in float32 (the depth the geometry core used): float32(0.1) itself is "exactly
    # 0.1 m" and invalid, likewise float32(10.0); then mm from the float32 value
    keep = np.isfinite(z32) & (z32 > np.float32(0.1)) & (z32 < np.float32(10.0))
    z = z32.astype(np.float64)
    depth = np.zeros(wd * hd, np.uint16)
    depth[keep] = np.clip(np.floor(z[keep] * 1000.0 + 0.5), 100, 9999).astype(np.uint16)
    geom_ok = snap["geometry_status"] == 0
    wc = [float(v) for v in np.asarray(snap["world_camera"]).ravel()]
    pose_ok = geom_ok and len(wc) == 12 and all(np.isfinite(wc))
    is32 = (snap["nx"], snap["ny"], snap["nz"]) == (32, 32, 32)
    grid = (np.asarray(snap["flags"], np.uint8)[:32768].copy() if geom_ok and is32
            else np.zeros(32768, np.uint8))
    sy, lo, hi, cx, cz = 0, 0, 0, 32, 32
    cam = [wc[3], wc[7], wc[11]] if len(wc) == 12 else []
    if cam and all(np.isfinite(cam)):
        cells = [int(np.floor((cam[k] - snap["origin"][k]) / snap["voxel"])) for k in range(3)]
        sy = max(0, min(31, cells[1]))
        lo = int(round((snap["origin"][1] + snap["voxel"] * sy) * 1000))
        hi = int(round((snap["origin"][1] + snap["voxel"] * (sy + 1)) * 1000))
        cx = cells[0] if 0 <= cells[0] < 32 else 32
        cz = cells[2] if 0 <= cells[2] < 32 else 32
    boxes = []
    if snap.get("roi") is not None and snap.get("roi_status") == 0:
        labels = snap.get("labels")
        for i, row in enumerate(snap["roi"]["rows"]):
            rid, en, x0, y0, x1, y1 = (int(v) for v in row[:6])
            verdict = int(snap["roi_rows"][i][6])
            if en != 1:
                continue
            if x0 > x1 or y0 > y1 or x1 >= wd or y1 >= hd or verdict not in (0, 2, 3) or len(boxes) == 32:
                continue
            name = labels[i] if labels is not None else None
            boxes.append((x0, y0, x1, y1, verdict, word_of(want_label_text(rid, name))))
    desc = np.zeros(196, np.uint32)
    desc[0], desc[1] = 0x4D374152, len(boxes)
    for k, b in enumerate(boxes):
        desc[2 + 6 * k: 8 + 6 * k] = b
    replay = source == "replay" or bool(snap.get("replay"))   # 10-12: data mark, age kept
    regs = dict(width=lay["width"], height=lay["height"], depth_width=wd, depth_height=hd,
                depth_dx=lay["depth_dx"], depth_dy=lay["depth_dy"], frame_valid=int(geom_ok),
                pose_valid=int(pose_ok), age_ms=0,
                age_valid=int(snap.get("nano_elapsed_ns") is not None),
                max_age_ms=max_age_ms, replay=int(replay), object_evidence=0,
                frame_id=snap["frame_id"] & 0xFFFFFFFF, permit_present=1, permit_i=0,
                grid_valid=int(geom_ok and is32), slice_y=sy, slice_lo_mm=lo & 0xFFFFFFFF,
                slice_hi_mm=hi & 0xFFFFFFFF, cam_cell_x=cx, cam_cell_z=cz,
                mini_x=lay["mini_x"], mini_y=lay["mini_y"])
    return {"regs": [regs[n] for n in golden.REG_NAMES], "depth": depth, "grid": grid,
            "desc": desc, "boxes": boxes}


def render(inp, age_ms=7, permit=1):
    regs = list(inp["regs"])
    regs[8] = age_ms if regs[9] else 0
    regs[15] = permit
    return golden.render(inp["depth"], inp["grid"], inp["desc"], regs)


def same_inputs(test, got, want, msg=""):
    test.assertEqual(list(got["regs"]), list(want["regs"]), msg)
    test.assertTrue(np.array_equal(got["depth"], want["depth"]), msg)
    test.assertTrue(np.array_equal(got["grid"], want["grid"]), msg)
    test.assertEqual([int(v) for v in got["desc"]], [int(v) for v in want["desc"]], msg)


def differs(got, want):
    return (list(got["regs"]) != list(want["regs"]) or not np.array_equal(got["depth"], want["depth"])
            or not np.array_equal(got["grid"], want["grid"])
            or [int(v) for v in got["desc"]] != [int(v) for v in want["desc"]])


# ------------------------------------------------------------------ synthetic frames
def synth_codes(seed=0):
    yy, xx = np.mgrid[0:HD, 0:WD]
    c = (650 + 300 * np.sin(xx / 11. + seed) + 150 * np.cos(yy / 7. - seed)).astype(np.uint16)
    c[(xx * 7 + yy * 3 + seed) % 13 == 0] = 0
    c[0, :6] = 12000            # beyond 10 m -> invalid
    c[1, :6] = 100              # float32(0.1): exactly the lower limit -> invalid
    c[2, :6] = 101              # 101 mm
    c[3, :6] = 9999             # 9999 mm
    c[4, :6] = 10000            # float32(10.0): the upper limit -> invalid
    return c


def synth_flags():
    g = np.zeros((32, 32, 32), np.uint8)       # [z][y][x]
    g[:, :, :6] = 2                             # hits along low x only
    g[25:, :, 10:20] = 1                        # free cells far in z only
    g[3, :, 28] = 2
    g[:, 9, 30] = 1                             # one column in the camera's layer
    return g.ravel()


ROI_ROWS = [  # row_id, enabled, x0, y0, x1, y1, near, far   / answer verdict
    ((100, 1, 4, 30, 40, 50, .5, .9), 2),
    ((101, 0, 0, 0, 0, 0, 0., 0.), 0),          # disabled
    ((102, 1, 50, 2, 60, 20, .5, .9), 0),
    ((103, 1, 70, 10, 96, 30, .5, .9), 2),      # x1 == width -> not drawable
    ((104, 1, 20, 5, 35, 25, .5, .9), 3),
    ((105, 1, 10, 40, 30, 60, .5, .9), 1),      # verdict 1 is not a display verdict
    ((106, 1, 44, 34, 58, 62, .5, .9), 2),
]
LABELS = [b"cup", b"bottle", b"BOOK", b"x", b"", b"chair", b"tv_set"]
SHOWN = {100: "CUP ", 102: "BOOK", 104: "R104", 106: "TV-S"}


def synth_snap(*, status=0, wc_t=(0.13, -0.31, 0.27), labels=LABELS, nano_elapsed_ns=4000000,
               dims=(32, 32, 32), roi=True, frame_id=(1 << 33) + 77):
    wc = np.zeros(12, np.float32)
    wc[[0, 5, 10]] = 1.0
    wc[[3, 7, 11]] = wc_t
    return {"frame_id": frame_id, "codes": synth_codes(), "unit_bits": f32_bits(UNIT),
            "width": WD, "height": HD, "nx": dims[0], "ny": dims[1], "nz": dims[2],
            "origin": np.array(ORIGIN), "voxel": VOXEL, "world_camera": wc,
            "flags": synth_flags() if status == 0 else np.full(32768, 99, np.uint8),
            "geometry_status": status,
            "roi": {"rows": [r for r, _ in ROI_ROWS]} if roi else None,
            "roi_status": 0 if roi else None,
            "roi_rows": [(r[0], r[1], 9, 9, 0, 0, v) for r, v in ROI_ROWS] if roi else [],
            "nano_elapsed_ns": nano_elapsed_ns, "labels": labels, "first_mono_ns": 123}


def layout(w=CANVAS[0], h=CANVAS[1]):
    return kv_ar.ar_layout(w, h, WD, HD)


def fill(snap, source="live", max_age_ms=200, module=kv_ar, lay=None):
    return module.fill_ar_inputs(snap, lay or layout(), source=source, max_age_ms=max_age_ms)


# ------------------------------------------------------------------ registers
def parse_defines(text, prefix):
    return {m.group(1): int(m.group(2), 16) for m in
            re.finditer(r"#define\s+%s(\w+)\s+0x([0-9a-fA-F]+)" % prefix, text)}


def reg_table_mismatches(ours, theirs):
    bad = []
    for name, off in ours.items():
        if theirs.get(name) != off:
            bad.append((name, off, theirs.get(name)))
    return bad


class RegisterTableTests(unittest.TestCase):
    def test_offsets_equal_7784f75_ip_header(self):
        ours = parse_defines(open(ROOT / "tools/kv_link/ar_helper/m7ar_regs.h").read(), "M7AR_")
        up = subprocess.run(["git", "show", "7784f75:hw/m7/pl/sw/m7pl_hls_regs.h"], cwd=str(ROOT),
                            stdout=subprocess.PIPE, check=True).stdout.decode()
        theirs = {k[:-5] if k.endswith("_DATA") else k: v for k, v in
                  parse_defines(up, "XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_").items()}
        self.assertEqual(len(ours), 32)
        self.assertEqual(reg_table_mismatches(ours, theirs), [])
        # every scalar address of the IP is in our table (DEPTH/GRID/BOXES/FB pointers too)
        self.assertEqual(set(theirs) - set(ours), set())
        # positive control: one offset off by 8 is found
        bad = dict(ours, SLICE_Y=ours["SLICE_Y"] + 8)
        self.assertEqual(reg_table_mismatches(bad, theirs), [("SLICE_Y", ours["SLICE_Y"] + 8,
                                                              theirs["SLICE_Y"])])

    def test_helper_register_order_is_the_spec_order(self):
        text = open(ROOT / "tools/kv_link/ar_helper/m7ar_regs.h").read()
        body = re.search(r"kM7arRegOff\[24\]\s*=\s*\{(.*?)\};", text, re.S).group(1)
        toks = [t.strip() for t in body.split(",") if t.strip()]
        want = ["0xFFFFFFFFu" if n == "permit_i" else "M7AR_" + n.upper() for n in golden.REG_NAMES]
        self.assertEqual(toks, want)
        self.assertEqual(list(kv_ar.REG_NAMES), list(golden.REG_NAMES))


# ------------------------------------------------------------------ age
AGE_CASES = [  # basis_ns, first_mono_ns, now_ns -> ms
    (0, 0, 0, 0),
    (0, 0, 1, 1),                               # 1 ns rounds up to 1 ms
    (999999, 0, 1, 1),                          # exactly 1 ms
    (999999, 0, 2, 2),                          # carry from the KV part
    (1, 5, 1000000, 1),                         # 1 + 999995 ns
    (4000000, 10 ** 9, 10 ** 9 - 5, 4),         # KV clock behind the first chunk -> 0
    (2500000, 7, 7 + 1500000, 4),
    (-1, 0, 0, 0),                              # never a negative age
    (1 << 62, 0, 1 << 40, 0xFFFFFFFF),          # saturation
    (4294967295 * 1000000, 0, 0, 0xFFFFFFFF),
    (4294967294 * 1000000 + 1, 0, 0, 0xFFFFFFFF),
    (4294967294 * 1000000, 0, 0, 4294967294),
]


class AgeTests(unittest.TestCase):
    def test_python_formula_known_answers(self):
        for b, f, n, want in AGE_CASES:
            self.assertEqual(kv_ar.ar_age_ms(b, f, n), want, (b, f, n))

    def test_helper_formula_equals(self):
        exe = Env.helper()
        for b, f, n, want in AGE_CASES:
            r = subprocess.run([exe, "--age-selftest", str(b), str(f), str(n)],
                               stdout=subprocess.PIPE, check=True)
            self.assertEqual(int(r.stdout), want, (b, f, n))


# ------------------------------------------------------------------ wire
def batch_of(rows=ROI_ROWS, revision=5):
    out = []
    for (rid, en, x0, y0, x1, y1, near, far), _v in rows:
        out.append(RoiRow(rid, bool(en), (x0, y0, x1, y1) if en else None, near, far))
    return RoiBatch(revision, Config(), out)


def request(roi=True, labels=None, age=None, fid=9):
    codes = synth_codes()
    return encode_request(frame_id=fid, capture_ns=11, pose_ns=11,
                          identity={k: "x" for k in kv_link.IDENTITY_FIELDS}, width=WD, height=HD,
                          nx=32, ny=32, nz=32, unit_f32=UNIT, voxel=VOXEL, origin=ORIGIN,
                          inverse_k=np.eye(3), world_camera=np.eye(4)[:3], codes=codes,
                          depth_crc=1, roi=batch_of() if roi else None, labels=labels,
                          age_block=encode_age_block(age) if age is not None else None)


def reseal(buf, start, length):
    body = bytes(buf[start:start + length - 4])
    return buf[:start] + body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF) + buf[start + length:]


class WireTests(unittest.TestCase):
    def _label_start(self, buf, has_age):
        lab_len = LABEL_HEAD.size + len(ROI_ROWS) * 8 + 4
        end = len(buf) - (AGE_BLOCK_BYTES if has_age else 0)
        return end - lab_len, lab_len

    def test_label_and_age_round_trip(self):
        wire = [clean_wire_label(n.decode())[0] for n in LABELS]
        buf = request(labels=wire, age=3456789)
        req = decode_request(buf, allow_age=True, allow_label=True)
        self.assertEqual(req["labels"], wire)
        self.assertEqual(req["nano_elapsed_ns"], 3456789)
        self.assertEqual([tuple(r[:6]) for r in req["roi"]["rows"]], [r[:6] for r, _ in ROI_ROWS])
        self.assertTrue(np.array_equal(req["codes"], synth_codes().ravel()))
        # without AGE as well
        req = decode_request(request(labels=wire), allow_label=True)
        self.assertEqual(req["labels"], wire)
        self.assertIsNone(req["nano_elapsed_ns"])

    def test_label_block_defects_void_the_request(self):
        wire = [clean_wire_label(n.decode())[0] for n in LABELS]
        for has_age in (False, True):
            buf = request(labels=wire, age=5 if has_age else None)
            s, n = self._label_start(buf, has_age)
            flip = bytearray(buf)
            flip[s + LABEL_HEAD.size + 1] ^= 0x20       # 'u' -> 'U': crc no longer matches
            cases = {
                "crc": bytes(flip),
                "label_ver": reseal(buf[:s] + b"\x02" + buf[s + 1:], s, n),
                "label_rsv": reseal(buf[:s + 2] + b"\x01\x00" + buf[s + 4:], s, n),
                "label_count": reseal(buf[:s + 1] + bytes([len(ROI_ROWS) - 1]) + buf[s + 2:], s, n),
                "label_char": reseal(buf[:s + 4] + b"cu\x00p" + buf[s + 8:], s, n),
            }
            hi = bytearray(buf)
            hi[s + LABEL_HEAD.size] = 0xE6                # non-ASCII byte
            cases["label_char "] = reseal(bytes(hi), s, n)
            for reason, bad in cases.items():
                with self.assertRaises(BadExtension) as cm:
                    decode_request(bad, allow_age=True, allow_label=True)
                self.assertEqual(cm.exception.reason, reason.strip(), (reason, has_age))
            if not has_age:             # label block cut short (ROI block intact)
                with self.assertRaises(BadExtension) as cm:
                    decode_request(buf[:-4], allow_label=True)
                self.assertEqual(cm.exception.reason, "length")

    def test_label_needs_roi_and_the_feature(self):
        wire = [b"cup"] * len(ROI_ROWS)
        buf = request(labels=wire)
        with self.assertRaises(BadExtension) as cm:          # --ar off server
            decode_request(buf, allow_age=False, allow_label=False)
        self.assertEqual(cm.exception.reason, "flags")
        no_roi = bytearray(request(roi=False))
        fl = REQ_HEAD.unpack_from(no_roi)
        no_roi[:REQ_HEAD.size] = REQ_HEAD.pack(*(fl[:11] + (4,) + fl[12:]))
        with self.assertRaises(BadExtension) as cm:
            decode_request(bytes(no_roi), allow_label=True)
        self.assertEqual(cm.exception.reason, "label_without_roi")
        with self.assertRaises(ValueError):
            request(roi=False, labels=[b"a"])
        with self.assertRaises(ValueError):
            request(labels=[b"a"])                           # count != rows

    def test_label_encoding_rules(self):
        self.assertEqual(clean_wire_label("bottle"), (b"bottle", None))
        self.assertEqual(clean_wire_label(" cup "), (b"cup", None))
        self.assertEqual(clean_wire_label("toothbrush"), (b"toothbru", "cut_to_8"))
        self.assertEqual(clean_wire_label(u"杯子"), (b"", "non_ascii"))
        self.assertEqual(clean_wire_label(None), (b"", "missing"))
        self.assertEqual(clean_wire_label(""), (b"", "missing"))
        self.assertEqual(clean_wire_label("a\tb"), (b"", "non_printable"))
        with self.assertRaises(ValueError):
            encode_label_block([b"toothbrus"])
        with self.assertRaises(ValueError):
            encode_label_block([u"杯".encode("utf-8")])
        blk = encode_label_block([b"", b"cup"])
        self.assertEqual(len(blk), LABEL_HEAD.size + 16 + 4)
        self.assertEqual(blk[4:20], b"\0" * 8 + b"cup\0\0\0\0\0")
        # display text
        for rid, name, want in [(5, None, "R005"), (5, b"", "R005"), (5, b"   ", "R005"),
                                (1234, b"", "R234"), (5, b"cup", "CUP "), (5, b"bottle", "BOTT"),
                                (5, b"tv_set", "TV-S"), (5, b"a b", "A B "), (5, b"Q7", "Q7  ")]:
            self.assertEqual(kv_ar.ar_label_text(rid, name), want)
            self.assertEqual(want_label_text(rid, name), want)
            self.assertEqual(text_of(kv_ar.ar_label(rid, name)), want)

    def test_age_block_round_trip_and_defects(self):
        blk = encode_age_block(987654321)
        self.assertEqual(len(blk), AGE_BLOCK_BYTES)
        self.assertEqual(kv_link.decode_age_block(blk), 987654321)
        bad_crc = blk[:-1] + bytes([blk[-1] ^ 1])
        ver2 = kv_link.seal(b"\x02" + blk[1:12])
        rsv = kv_link.seal(blk[:1] + b"\x01" + blk[2:12])
        for b, reason in ((bad_crc, "crc"), (ver2, "age_ver"), (rsv, "age_rsv"),
                          (blk[:-2], "length")):
            with self.assertRaises(BadExtension) as cm:
                kv_link.decode_age_block(b)
            self.assertEqual(cm.exception.reason, reason)
        buf = request(age=5)
        with self.assertRaises(BadExtension) as cm:          # server without age_ext
            decode_request(buf)
        self.assertEqual(cm.exception.reason, "flags")
        with self.assertRaises(BadExtension):
            decode_request(buf[:-1] + bytes([buf[-1] ^ 1]), allow_age=True)
        with self.assertRaises(ValueError):
            encode_age_block(-1)

    def test_no_extension_bytes_equal_b43daa1(self):
        old = load_old("host/kv_link.py", "b43daa1")
        kw = dict(frame_id=9, capture_ns=11, pose_ns=12,
                  identity={k: "id_" + k for k in kv_link.IDENTITY_FIELDS}, width=WD, height=HD,
                  nx=32, ny=32, nz=32, unit_f32=UNIT, voxel=VOXEL, origin=ORIGIN,
                  inverse_k=np.eye(3), world_camera=np.eye(4)[:3], codes=synth_codes(), depth_crc=7)
        self.assertEqual(encode_request(**kw), old.encode_request(**kw))
        self.assertEqual(encode_request(roi=batch_of(), **kw), old.encode_request(roi=batch_of(), **kw))
        self.assertEqual(kv_link.FEATURES, old.FEATURES)


# ------------------------------------------------------------------ oracle and mutants
def regions(inp, lay, mask_of):
    """Boolean masks (HxW) of the canvas parts a mutant may touch."""
    w, h = lay["width"], lay["height"]
    m = np.zeros((h, w), bool)
    dx, dy = lay["depth_dx"], lay["depth_dy"]
    for part in mask_of:
        if part == "panel":
            m[0:40, :] = True
        elif part == "depth":
            m[dy:dy + HD, dx:dx + WD] = True
        elif part == "mini":
            mx, my = lay["mini_x"], lay["mini_y"]
            m[max(0, my - 12):my + 129, max(0, mx - 1):mx + 129] = True
        elif part in ("bars", "boxes"):
            for (x0, y0, x1, y1, _v, _l) in inp["boxes"]:
                X0, Y0, X1, Y1 = x0 + dx, y0 + dy, x1 + dx, y1 + dy
                LY = Y0 - 12 if Y0 >= 52 else Y0 + 2
                m[LY:LY + 12, X0:X0 + 48] = True
                if part == "boxes":           # box +-1 column and its moved bar
                    m[Y0:Y1 + 1, X0:X1 + 2] = True
                    m[LY:LY + 12, X0:X0 + 49] = True
    return m


MUTANTS = {  # name: (old, new, case, region parts)
    "label_row_shift": ("ar_label(row_id, name)", "ar_label(row_id + 1, name)", "live", ("bars",)),
    "label_misaligned": ("labels[i] if labels", "labels[(i + 1) % len(labels)] if labels", "live", ("bars",)),
    "label_truncated": ("for c in t[:4]", "for c in t[:3]", "live", ("bars",)),
    "label_no_fallback": ('return "R%03d" % (int(row_id) % 1000)', 'return "    "', "live", ("bars",)),
    "box_off_by_one": ("= (x0, y0, x1, y1, verdict, word)", "= (x0 + 1, y0, x1 + 1, y1, verdict, word)",
                       "live", ("boxes",)),
    "verdict_zero": ("= (x0, y0, x1, y1, verdict, word)", "= (x0, y0, x1, y1, 0, word)", "live", ("boxes",)),
    "replay_polarity": ('regs[R["replay"]] = 1 if replay else 0', 'regs[R["replay"]] = 0 if replay else 1',
                        "replay", ("panel", "depth", "boxes", "mini")),
    "replay_drops_age": ("age_valid = 1 if basis is not None else 0",
                         "age_valid = 1 if (basis is not None and not replay) else 0",
                         "replay_age", ("panel",)),
    "frame_valid_const": ('regs[R["frame_valid"]] = 1 if ok else 0', 'regs[R["frame_valid"]] = 1',
                          "geomfail", ("panel", "depth", "mini")),
}
CASES = {
    "live": dict(kw={}, source="live"),
    "replay": dict(kw={"nano_elapsed_ns": None}, source="replay"),
    "replay_age": dict(kw={}, source="replay"),
    "geomfail": dict(kw={"status": 3}, source="live"),
}


def mutant_module(name):
    old, new = MUTANTS[name][:2]
    src = open(ROOT / "host/kv_ar.py").read()
    if src.count(old) != 1:
        raise AssertionError("mutant %s: anchor hits %d times" % (name, src.count(old)))
    mod = types.ModuleType("kv_ar_" + name)
    mod.__file__ = str(ROOT / "host/kv_ar.py")
    exec(compile(src.replace(old, new), "kv_ar[%s]" % name, "exec"), mod.__dict__)
    return mod


class OracleTests(unittest.TestCase):
    def check(self, snap, source="live", max_age_ms=200, lay=None, msg=""):
        lay = lay or layout()
        got = fill(snap, source, max_age_ms, lay=lay)
        want = oracle(snap, lay, source, max_age_ms)
        same_inputs(self, got, want, msg)
        sg, fg, ig = render(got)
        sw, fw, iw = render(want)
        self.assertEqual(sg, sw)
        self.assertTrue(np.array_equal(fg, fw), msg)
        return got, want, ig

    def test_live_frame_boxes_and_labels_drawn(self):
        got, want, info = self.check(synth_snap())
        self.assertEqual(info["mode"], 0)
        self.assertEqual(info["panel"][1], "LIVE")
        self.assertEqual(info["boxes"], 4)
        self.assertEqual(got["boxes"], 4)
        self.assertEqual(got["skipped_boxes"], 2)          # out of range + verdict 1
        self.assertEqual(got["named_boxes"], 3)
        shown = {int(r[0]): text_of(got["desc"][7 + 6 * k]) for k, (r, _v) in
                 enumerate([rv for rv in ROI_ROWS if rv[0][0] in SHOWN])}
        self.assertEqual(shown, SHOWN)
        r = dict(zip(golden.REG_NAMES, got["regs"]))
        self.assertEqual((r["slice_y"], r["cam_cell_x"], r["cam_cell_z"]), (9, 18, 6))
        self.assertEqual((r["slice_lo_mm"], r["slice_hi_mm"]), ((-350) & 0xFFFFFFFF, (-300) & 0xFFFFFFFF))
        self.assertEqual(r["frame_id"], 77)
        self.assertEqual(info["slice_label"], "Y-0350..-0300MM")
        # the boxes' label bars carry ink of the expected strings: rendering the same frame
        # with other names changes exactly those bars
        other = synth_snap(labels=[b"zz"] * len(ROI_ROWS))
        _s, fa, _i = render(got)
        _s, fb, _i = render(fill(other))
        d = fa != fb
        self.assertTrue(d.any())
        self.assertFalse((d & ~regions(want, layout(), ("bars",))).any())

    def test_replay_no_labels_and_variants(self):
        _g, _w, info = self.check(synth_snap(nano_elapsed_ns=None), "replay")
        self.assertEqual((info["mode"], info["panel"][1], info["panel"][2]), (0, "REPLAY", "AGE N/A"))
        _g, _w, info = self.check(synth_snap(), "replay")              # 10-12: the age is kept
        self.assertEqual((info["mode"], info["panel"][1], info["panel"][2]), (0, "REPLAY", "AGE:007MS"))
        marked = synth_snap()
        marked["replay"] = True                                         # the request's replay mark
        _g, _w, info = self.check(marked, "live")
        self.assertEqual((info["mode"], info["panel"][1]), (0, "REPLAY"))
        got, _w, _i = self.check(synth_snap(labels=None), msg="no LABEL block")
        self.assertEqual([text_of(got["desc"][7 + 6 * k]) for k in range(4)],
                         ["R100", "R102", "R104", "R106"])
        _g, _w, info = self.check(synth_snap(nano_elapsed_ns=None), "live")
        self.assertEqual(info["mode"], 5)                    # live without age -> NOAGE
        _g, _w, info = self.check(synth_snap(status=3))
        self.assertEqual(info["mode"], 1)
        _g, _w, info = self.check(synth_snap(dims=(16, 32, 32)))
        self.assertEqual(info["mode"], 0)                    # mini map hatched only
        _g, _w, info = self.check(synth_snap(wc_t=(5.0, -3.0, -0.4)))   # camera outside the grid
        _g, _w, info = self.check(synth_snap(wc_t=(float("nan"), 0, 0)))
        self.assertEqual(info["mode"], 3)
        _g, _w, info = self.check(synth_snap(roi=False))
        self.assertEqual(info["boxes"], 0)
        _g, _w, _i = self.check(synth_snap(), max_age_ms=3)  # age 7 > 3 -> STALE
        self.assertEqual(_i["mode"], 2)
        small = kv_ar.ar_layout(320, 240, WD, HD)
        self.check(synth_snap(), lay=small)
        self.assertEqual((small["mini_x"], small["mini_y"]), (144, 60))
        corner = kv_ar.ar_layout(224, 200, WD, HD)           # no room right of the depth
        self.assertEqual((corner["mini_x"], corner["mini_y"]), (224 - 132, 200 - 132))

    def test_thirty_three_rows_cap_at_32(self):
        rows = [((200 + i, 1, i, 1, i + 3, 6, .5, .9), 2) for i in range(33)]
        snap = synth_snap()
        snap["roi"] = {"rows": [r for r, _ in rows]}
        snap["roi_rows"] = [(r[0], 1, 9, 9, 0, 0, v) for r, v in rows]
        snap["labels"] = [b"cup"] * 33
        got, _w, info = self.check(snap)
        self.assertEqual((got["boxes"], got["skipped_boxes"]), (32, 1))

    def test_positive_controls(self):
        for name, (_old, _new, case, parts) in sorted(MUTANTS.items()):
            with self.subTest(mutant=name):
                c = CASES[case]
                snap = synth_snap(**c["kw"])
                lay = layout()
                want = oracle(snap, lay, c["source"], 200)
                same_inputs(self, fill(snap, c["source"]), want, name + " (unmutated)")
                bad = fill(snap, c["source"], module=mutant_module(name))
                self.assertTrue(differs(bad, want), "oracle missed mutant %s" % name)
                sw, fw, _ = render(want)
                sb, fb, _ = render(bad)
                self.assertEqual((sw, sb), (0, 0))
                d = fw != fb
                self.assertTrue(d.any(), "mutant %s leaves every pixel equal" % name)
                outside = d & ~regions(want, lay, parts)
                self.assertFalse(outside.any(), "mutant %s changes %d pixels outside %s"
                                 % (name, int(outside.sum()), parts))
                if "panel" in parts:
                    self.assertTrue(d[:40].any())


# ------------------------------------------------------------------ helper (mock provider)
class EventReader(threading.Thread):
    def __init__(self, stream):
        super().__init__(daemon=True)
        self.stream, self.events, self.cv = stream, [], threading.Condition()

    def run(self):
        for raw in iter(self.stream.readline, b""):
            line = raw.decode("utf-8", "replace").strip()
            if line.startswith("{"):
                with self.cv:
                    self.events.append(json.loads(line))
                    self.cv.notify_all()

    def wait(self, pred, timeout=20.0):
        end = time.monotonic() + timeout
        with self.cv:
            while True:
                hit = [e for e in self.events if pred(e)]
                if hit:
                    return hit[0]
                left = end - time.monotonic()
                if left <= 0:
                    raise AssertionError("no matching helper event; got %s" % self.events[-5:])
                self.cv.wait(left)


def message(frame_id, *, lay, max_age_ms, basis_ns=2000000, labels=LABELS):
    snap = synth_snap(frame_id=frame_id, nano_elapsed_ns=basis_ns, labels=labels)
    inp = kv_ar.fill_ar_inputs(snap, lay, source="live", max_age_ms=max_age_ms)
    inp["first_mono_ns"] = time.monotonic_ns() if hasattr(time, "monotonic_ns") else int(time.monotonic() * 1e9)
    return kv_ar.encode_ar_message(inp)


class HelperMockTests(unittest.TestCase):
    def test_newest_wins_failsafe_and_dumps(self):
        exe = Env.helper()
        lay = kv_ar.ar_layout(320, 240, WD, HD)
        with tempfile.TemporaryDirectory() as d:
            p = subprocess.Popen([exe, "--provider", "mock", "--display", "none", "--mock-size", "320x240",
                                  "--mock-permit", "1", "--stale-ms", "1500", "--refresh-ms", "400",
                                  "--dump-dir", d, "--dump-seq", "2,3", "--dump-failsafe", "1",
                                  "--event-log", os.path.join(d, "events.log")],
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                import fcntl
                fcntl.fcntl(p.stdin.fileno(), kv_ar.F_SETPIPE_SZ, 1 << 20)
                ev = EventReader(p.stdout)
                ev.start()
                ready = ev.wait(lambda e: e["ev"] == "ready")
                self.assertEqual((ready["width"], ready["height"], ready["scanout"]), (320, 240, "none"))
                start = ev.wait(lambda e: e["ev"] == "startup")
                self.assertEqual(start["seq"], 1)
                three = b"".join(message(f, lay=lay, max_age_ms=1000) for f in (11, 12, 13))
                os.write(p.stdin.fileno(), three)
                f13 = ev.wait(lambda e: e["ev"] == "frame")
                self.assertEqual((f13["frame_id"], f13["seq"], f13["status"], f13["flipped"]), (13, 2, 0, 1))
                os.write(p.stdin.fileno(), message(14, lay=lay, max_age_ms=1000))
                f14 = ev.wait(lambda e: e["ev"] == "frame" and e["frame_id"] == 14)
                self.assertEqual(f14["seq"], 3)
                self.assertGreaterEqual(f14["age_ms"], 2)
                self.assertLess(f14["age_ms"], 1000)
                self.assertEqual((f14["age_valid"], f14["replay"], f14["frame_valid"]), (1, 0, 1))
                fs = ev.wait(lambda e: e["ev"] == "failsafe", timeout=10)
                self.assertGreaterEqual(fs["since_frame_ms"], 1500)
                self.assertLess(fs["since_frame_ms"], 2500)
                self.assertEqual((fs["frame_id"], fs["age_valid"], fs["replay"]), (14, 1, 0))
                self.assertGreater(fs["age_ms"], 1000)
                p.stdin.close()                              # kv_server gone
                ev.wait(lambda e: e["ev"] == "eof")
                n_eof = len(ev.events)
                later = ev.wait(lambda e: e["ev"] == "failsafe" and ev.events.index(e) >= n_eof, timeout=5)
                self.assertEqual(later["frame_id"], 14)
                p.send_signal(signal.SIGTERM)
                ex = ev.wait(lambda e: e["ev"] == "exit")
                self.assertEqual((ex["frames_rx"], ex["superseded"]), (4, 2))
                self.assertEqual([e["frame_id"] for e in ev.events if e["ev"] == "frame"], [13, 14])
                p.wait(10)
                self.assertEqual(p.returncode, 0)
                logged = [json.loads(x) for x in open(os.path.join(d, "events.log"))]
                self.assertEqual(logged, ev.events)          # incl. the events after stdin EOF
                lib = Env.c_ref()
                bases = sorted(os.path.join(d, f[:-5]) for f in os.listdir(d) if f.endswith(".json"))
                self.assertEqual([os.path.basename(b) for b in bases], ["ar_000002", "ar_000003", "ar_000004"])
                rows = [ar_dump_check.check_one(b, lib)[0] for b in bases]
                for r in rows:
                    self.assertTrue(r["equal"], r)
                    self.assertTrue(r["c_ref_equal_fb"], r)
                    self.assertEqual(r["c_ref_sentinel_left"], 0)
                self.assertEqual([r["mode_name"] for r in rows], ["LIVE/REPLAY", "LIVE/REPLAY", "STALE"])
                # the helper's own mode (PS-side mirror of the core rule) == the golden's, per dump
                by_seq = {e["seq"]: e for e in ev.events if "seq" in e}
                self.assertEqual([(by_seq[r["seq"]]["mode"], by_seq[r["seq"]]["mode_name"]) for r in rows],
                                 [(r["mode"], r["panel"][1]) for r in rows])
                self.assertEqual([r["boxes"] for r in rows], [4, 4, 0])
                self.assertEqual(rows[2]["panel"][1], "STALE")
                self.assertEqual(rows[0]["panel"][3], "DRIVE OK")
                desc = np.fromfile(bases[0] + ".desc", "<u4")
                self.assertEqual(sorted(text_of(desc[7 + 6 * k]) for k in range(desc[1])),
                                 sorted(SHOWN.values()))
            finally:
                if p.poll() is None:
                    p.kill()
                    p.wait()


# ------------------------------------------------------------------ kv_server
class FakeAr:
    def __init__(self):
        self.snaps = []

    def submit(self, snap):
        self.snaps.append(snap)

    def telem(self):
        return {"available": True, "mode": "fake"}


def ident(fid, ns):
    return CaptureIdentity("s1", fid, ns, CLOCK, "cal", LOCAL, EPOCH)


def frame_kwargs(fid, codes, ns, t=(0.0, 0.0, 0.0)):
    c2w = np.eye(4)
    c2w[:3, 3] = t
    return dict(depth_m=depth_u16_to_m(codes, UNIT), k=K, camera_to_world=c2w,
                grid_origin_m=ORIGIN, voxel_size_m=VOXEL, shape=(32, 32, 32), frame_id=fid,
                capture_time_ns=ns, pose_time_ns=ns, calibration_version="cal",
                pose_confirmed=True)


def enabled_batch():
    rows = [rv for rv in ROI_ROWS if rv[0][1] == 1 and rv[0][4] < WD]
    return batch_of(rows), rows


class ServerPathTests(unittest.TestCase):
    def _server(self, ar):
        env = Env.get()
        core = FrameNativeM7(env.frame_lib).lib.fpga_m7_depth_frame
        return KvLinkServer(core, "cpu", bind=("127.0.0.1", 0), roi_core=CpuRoiCore(env.obj_lib),
                            ar=ar).start()

    def test_snapshot_equals_client_view(self):
        ar = FakeAr()
        srv = self._server(ar)
        try:
            c = KvLinkClient(("127.0.0.1", srv.port), chunk_bytes=2000, timeout_s=1.0)
            info = c.hello(2.0)
            self.assertEqual((info["features"]["age_ext"], info["features"]["label_ext"]), (1, 1))
            be = RemotePlFrameBackend(c, depth_unit_m=UNIT)
            batch, rows = enabled_batch()
            names = ["cup", "bottle", u"杯子", "toothbrush", "tv_set"][:len(rows)]
            t = (0.13, -0.31, 0.27)
            for fid in (1, 2):
                codes = synth_codes(fid)
                ns = now_ns()
                be.set_next_roi(batch, labels=names)
                ev = be.frame_from_depth(identity=ident(fid, ns), **frame_kwargs(fid, codes, ns, t))
                t_after = time.monotonic()
                for _ in range(100):
                    if len(ar.snaps) == fid:
                        break
                    time.sleep(0.01)
                snap = ar.snaps[-1]
                ro = be.last_roi
                self.assertTrue(ro.ok, ro)
                self.assertEqual(be.counts["label_ext_sent"], fid)
                self.assertEqual(snap["labels"], [b"cup", b"bottle", b"", b"toothbru", b"tv_set"][:len(rows)])
                self.assertGreaterEqual(snap["nano_elapsed_ns"], 0)
                self.assertLess(snap["nano_elapsed_ns"], 10 ** 9)
                self.assertLessEqual(snap["first_mono_ns"], int(t_after * 1e9))
                wc = np.zeros(12)
                wc[[0, 5, 10]] = 1
                wc[[3, 7, 11]] = t
                client_view = {
                    "frame_id": fid, "codes": codes.ravel(), "unit_bits": f32_bits(UNIT),
                    "width": WD, "height": HD, "nx": 32, "ny": 32, "nz": 32,
                    "origin": list(ORIGIN), "voxel": VOXEL, "world_camera": wc.astype(np.float32),
                    "flags": ev.flags.ravel(), "geometry_status": 0,
                    "roi": {"rows": [r for r, _ in rows]}, "roi_status": 0,
                    "roi_rows": [[rw[f] for f in ROI_FIELDS] for rw in ro.rows],
                    "nano_elapsed_ns": 1,
                    "labels": [clean_wire_label(n)[0] for n in names]}
                lay = layout()
                got = kv_ar.fill_ar_inputs(snap, lay, source="live", max_age_ms=200)
                same_inputs(self, got, oracle(client_view, lay, "live", 200), "frame %d" % fid)
                st, fb, inf = render(got)
                self.assertEqual((st, inf["mode"]), (0, 0))
                self.assertEqual(inf["boxes"], got["boxes"])
                self.assertGreater(got["boxes"], 0)
        finally:
            srv.stop()

    def test_ar_off_server_unchanged_and_voids_extensions(self):
        old_telem = load_old("host/kv_telem.py", "b43daa1")
        old_link = load_old("host/kv_link.py", "b43daa1")
        srv = self._server(None)
        try:
            c = KvLinkClient(("127.0.0.1", srv.port), chunk_bytes=2000, timeout_s=0.5)
            info = c.hello(2.0)
            self.assertEqual(info["features"], old_link.FEATURES)
            self.assertNotIn("age_ext", info["features"])
            tm = KvTelemetryClient(("127.0.0.1", srv.port)).poll()
            self.assertEqual(tm.blocks["ar"], {"available": False, "reason": old_telem.AR_REASON})
            # the Nano side attaches nothing to such a server ...
            be = RemotePlFrameBackend(c, depth_unit_m=UNIT)
            batch, rows = enabled_batch()
            codes, ns = synth_codes(), now_ns()
            be.set_next_roi(batch, labels=["cup"] * len(rows))
            be.frame_from_depth(identity=ident(1, ns), **frame_kwargs(1, codes, ns))
            self.assertTrue(be.last_roi.ok)
            self.assertEqual(be.counts["label_ext_sent"], 0)
            # ... and a request that carries the blocks anyway is void (no answer)
            for fid, kw in ((50, dict(labels=[b"cup"] * len(ROI_ROWS))), (51, dict(age=5))):
                req = request(fid=fid, **kw)      # frame ids increase (older ids are dropped)
                with self.assertRaises(LinkTimeout):
                    c.call(req, {"frame_id": fid, "capture_ns": 11, "pose_ns": 11,
                                 "identity": {k: "x" for k in kv_link.IDENTITY_FIELDS}})
            self.assertEqual(srv.counts["bad_ext:flags"], 2)
        finally:
            srv.stop()

    def test_ar_server_telem_block_from_presenter(self):
        srv = self._server(FakeAr())
        try:
            tm = KvTelemetryClient(("127.0.0.1", srv.port)).poll()
            self.assertEqual(tm.blocks["ar"]["mode"], "fake")
        finally:
            srv.stop()


TELEM_AR_FIELDS = {"available", "mode", "source", "helper_alive", "helper_pid", "display", "scanout",
                   "wait", "submitted", "sent", "flips", "failsafe_flips", "last_flip_frame_id",
                   "last_flip_seq", "ar_core_ns", "flip_interval_ms", "last_age_ms", "last_status",
                   "failsafe_active", "interval_ms_p50",
                   # 10-12: the picture on screen (hub reads mode/age_valid/replay/frame_id/age_ms)
                   "presenter", "age_ms", "age_valid", "replay", "replay_core", "frame_id",
                   "render_kind", "render_seq"}


class KvServerEndToEndTests(unittest.TestCase):
    def test_kv_server_ar_drm_mock_with_nano_replay(self):
        exe = Env.helper()
        env = dict(os.environ, PYTHONPATH=str(ROOT))
        with tempfile.TemporaryDirectory() as t:
            out, dump = os.path.join(t, "kv"), os.path.join(t, "dump")
            srv = subprocess.Popen([sys.executable, str(ROOT / "tools/kv_link/kv_server.py"), "--bind",
                                    "127.0.0.1", "--port", "0", "--max-idle-s", "60", "--core", "cpu",
                                    "--roi-core", "cpu", "--ar-mock", "--ar", "drm", "--ar-helper", exe,
                                    "--ar-mock-size", "640x480", "--ar-max-age-ms", "1500",
                                    "--ar-stale-ms", "1501", "--ar-refresh-ms", "300",
                                    "--ar-dump-dir", dump, "--ar-dump-seq", "3,6",
                                    "--ar-dump-failsafe", "1", "--out", out],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
            try:
                port = json.loads(srv.stdout.readline())["port"]
                prep = os.path.join(t, "prep")
                os.makedirs(prep)
                n = 10
                np.stack([synth_codes(i) for i in range(n)]).astype("<u2").tofile(
                    os.path.join(prep, "depth_u16.bin"))
                json.dump({"width": WD, "height": HD, "frames": n, "depth_file": "depth_u16.bin",
                           "depth_unit_m": UNIT, "K": K.tolist(), "calibration_id": "synthetic",
                           "grid": {"origin_m": list(ORIGIN), "voxel_m": VOXEL, "shape_xyz": [32, 32, 32]}},
                          open(os.path.join(prep, "meta.json"), "w"))
                snapf = os.path.join(t, "snap.json")
                tool = str(ROOT / "tools/kv_link/nano_replay.py")
                mk = subprocess.run([sys.executable, tool, "--prep", prep, "--make-roi-memory", snapf,
                                     "--roi-boxes", "12"], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    env=env)
                self.assertEqual(mk.returncode, 0, mk.stderr[-2000:])
                nano = os.path.join(t, "nano")
                done = subprocess.run([sys.executable, tool, "--prep", prep, "--out", nano, "--hz", "10",
                                       "--backend", "remote_pl", "--server", "127.0.0.1:%d" % port,
                                       "--chunk-bytes", "2000", "--roi-memory", snapf],
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
                self.assertEqual(done.returncode, 0, done.stderr[-3000:])
                s = json.load(open(os.path.join(nano, "summary.json")))
                self.assertEqual(s["roi"]["plan"]["server_label_ext"], 1)
                labels = s["roi"]["plan"]["labels"]
                self.assertEqual(len(labels), s["roi"]["plan"]["rows_sent"])
                self.assertEqual(s["backend_counts"]["label_ext_sent"], s["answered"])
                self.assertEqual(s["backend_counts"]["age_ext_sent"], s["answered"])
                self.assertEqual(s["backend_counts"]["replay_ext_sent"], s["answered"])   # a recording
                self.assertEqual(s["answered"], n)
                tc = KvTelemetryClient(("127.0.0.1", port), timeout_s=1.0)
                blk = tc.poll().blocks["ar"]
                self.assertEqual(set(blk), TELEM_AR_FIELDS)
                self.assertTrue(blk["helper_alive"])
                self.assertEqual(blk["submitted"], n)
                self.assertGreater(blk["flips"], 0)
                self.assertEqual((blk["replay"], blk["presenter"]), (1, "drm"))
                self.assertIn(blk["mode"], ("REPLAY", "STALE"))
                events = os.path.join(out, "ar_events.jsonl")
                end = time.monotonic() + 10
                fs = None
                while fs is None and time.monotonic() < end:
                    for line in open(events):
                        e = json.loads(line)
                        if e["ev"] == "failsafe":
                            fs = e
                    time.sleep(0.2)
                self.assertIsNotNone(fs, "no fail-safe frame after the stream stopped")
                self.assertGreaterEqual(fs["since_frame_ms"], 1501)
                blk = tc.poll().blocks["ar"]
                self.assertTrue(blk["failsafe_active"])
                self.assertGreater(blk["failsafe_flips"], 0)
                self.assertEqual((blk["mode"], blk["replay"], blk["replay_core"], blk["age_valid"]),
                                 ("STALE", 1, 0, 1))
                self.assertGreater(blk["age_ms"], 1500)
            finally:
                srv.send_signal(signal.SIGTERM)
                so, se = srv.communicate(timeout=30)
            self.assertEqual(srv.returncode, 0, se[-3000:])
            summ = json.load(open(os.path.join(out, "summary.json")))
            relayed = [json.loads(x) for x in open(os.path.join(out, "ar_events.jsonl"))]
            own = [json.loads(x) for x in open(os.path.join(out, "ar_helper_events.jsonl"))]
            self.assertEqual(own[:len(relayed)], relayed)
            self.assertEqual(summ["ar"]["exit_event"]["ev"], "exit")
            self.assertEqual(summ["ar"]["counts"]["sent"] + summ["ar"]["counts"].get("superseded_before_send", 0),
                             n)
            chk = subprocess.run([sys.executable, str(ROOT / "tools/kv_link/ar_dump_check.py"), "--dump", dump,
                                  "--c-ref", "--scratch", os.path.join(t, "cref"), "--json",
                                  os.path.join(t, "chk.json")], stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, env=env)
            self.assertEqual(chk.returncode, 0, chk.stdout.decode()[-3000:] + chk.stderr.decode()[-2000:])
            rows = json.load(open(os.path.join(t, "chk.json")))["rows"]
            kinds = [(r["kind"], r["mode_name"]) for r in rows]
            self.assertIn(("failsafe", "STALE"), kinds)
            frames = [r for r in rows if r["kind"] == "frame"]
            self.assertTrue(frames)
            for r in frames:
                self.assertEqual(r["mode_name"], "LIVE/REPLAY")
                self.assertEqual((r["panel"][1], r["replay"], r["age_valid"]), ("REPLAY", 1, 1))   # nano_replay
                self.assertTrue(r["panel"][2].startswith("AGE:"), r["panel"])
                self.assertGreater(r["boxes"], 0)
                desc = np.fromfile(os.path.join(dump, "ar_%06d.desc" % r["seq"]), "<u4")
                drawn = {text_of(desc[7 + 6 * k]) for k in range(desc[1])}
                self.assertTrue(drawn <= {"CUP ", "BOTT", "BOOK"}, drawn)


if __name__ == "__main__":
    unittest.main()
