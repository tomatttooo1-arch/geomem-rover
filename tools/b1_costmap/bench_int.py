#!/usr/bin/env python3
"""B1h bench: op GRID integrated into the rolling fusion core, checked word for word (2026-11-02).

What it checks (one pass over a recorded rolling trace, default the 10-26 formal trace):

  regression   hw/m7/rolling_map.cpp built WITHOUT the feature bit (= pre-B1h source after preprocessing)
               and WITH -DFPGA_M7_ROLL_GRID both replay every trace record; header / delta / query / dump
               must equal the recorded words (tools/m7_rolling_pl/replay_trace.compare). The feature-on core
               additionally gets op GRID calls interleaved, so a GRID that touched state would show up as a
               mismatch on a later record.
  GRID         after each accepted INTEGRATE whose window holds the height band: op GRID on the feature-on
               core vs the standalone C model hw/b1_costmap/costmap.cpp on the same URAM words (DUMP),
               packed planes compared word for word (grid, aux, cost); header = the DUMP header except op
               and N_DELTA. Flags rotate: body+inflation every frame, body only / inflation only / none on
               every 4th frame. Also vs the Nano bridge (host/rolling_nav_layer) grid / strict / tall and the
               exact Nav2 inflation reference (tools/b1_costmap/nav2_ref.inflate_exact).
  rejections   GRID before RESET, bad band / tall / flags / table / body -> status and reason.
  controls     two fault builds of the integrated op (GRID_RING_Y, GRID_LAST_AUX) and a wrong band input
               must be caught.
Writes --out JSON and, with --trace-out, an M7RT v1 trace (formal records + GRID records whose expected
delta words come from the C model) for tb_rolling_axi.cpp (HLS csim / cosim), and --cosim-out a short one.
Python 3.8, numpy, g++.
"""
import argparse
import ctypes
import json
import os
import struct
import subprocess
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.m7_rolling import COMPILE_FLAGS, NavigationDelta  # noqa: E402
from host import rolling_nav_layer as rnl  # noqa: E402
from tools.m7_rolling_pl.replay_trace import read_trace, compare, REC_HEAD  # noqa: E402
from tools.b1_costmap import nav2_ref  # noqa: E402
from tools.b1_costmap.bench import (DEFAULT_TRACE, MX, MY, MZ, V, LUT_DIM, GROUND_Z, ROBOT_H, BODY_R,  # noqa: E402
                                    BODY_MARGIN, build_costmap, PlCore, decode_delta, band_params, body_pose,
                                    body_q, new_bridge, signed)

OP_RESET, OP_INTEGRATE, OP_QUERY, OP_DUMP, OP_GRID = 0, 1, 2, 3, 4
H_OP, H_N_DELTA, H_CHANGED = 1, 11, 12
PW = MX * MY // 8
OBS_CAP, QCAP, DCAP, WORDS = 32 ** 3, 1024, 32 * 32 * 5, MX // 8 * MY * MZ

U8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
I32 = np.ctypeslib.ndpointer(dtype=np.int32, flags="C_CONTIGUOUS")
U32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags="C_CONTIGUOUS")
U64 = np.ctypeslib.ndpointer(dtype=np.uint64, flags="C_CONTIGUOUS")


class Core(object):
    """Bare ctypes caller of fpga_m7_rolling_map (fresh .so per instance = fresh static state)."""

    def __init__(self, so):
        self.lib = ctypes.CDLL(so)
        f = self.lib.fpga_m7_rolling_map
        f.argtypes = [ctypes.c_uint32] * 3 + [I32, I32] + [ctypes.c_uint32] * 3 + [U8, ctypes.c_uint32, I32,
                                                                                    U32, U64, U64, U32]
        f.restype = None
        self.f = f

    def call(self, op, epoch=0, frame=0, wb=(0, 0, 0), ob=(0, 0, 0), dims=(0, 0, 0), obs=None, qc=0, query=None):
        obuf = np.full(OBS_CAP, 0x5A, np.uint8)
        if obs is not None:
            b = np.frombuffer(bytes(obs), np.uint8)
            obuf[:len(b)] = b
        qbuf = np.zeros(3 * QCAP, np.int32)
        if query is not None and len(query):
            qbuf[:len(query)] = query
        hdr = np.full(24, 0xDEADBEEF, np.uint32)
        qout = np.full(QCAP, 0xC3C3C3C3, np.uint32)
        delta = np.full(DCAP, 0xA5A5A5A5A5A5A5A5, np.uint64)
        dump = np.full(WORDS, 0x3C3C3C3C3C3C3C3C, np.uint64)
        self.f(op, epoch, frame, np.asarray(wb, np.int32), np.asarray(ob, np.int32), dims[0], dims[1], dims[2],
               obuf, qc, qbuf, qout, delta, dump, hdr)
        return hdr, delta, qout, dump


def build_roll(out_so, defines=()):
    subprocess.run(["g++"] + COMPILE_FLAGS + list(defines) + [os.path.join(ROOT, "hw/m7/rolling_map.cpp"),
                                                             "-o", out_so], check=True)
    return Core(out_so)


def pack(plane):
    return np.frombuffer(np.ascontiguousarray(plane).astype(np.uint8).tobytes(), "<u8").astype(np.uint64)


def grid_args(bp, tall_lo, body, flags):
    be, cx, cy, r = body
    return dict(ob=(cx, cy, int(np.uint32(r).view(np.int32))), dims=(bp[0], bp[1], tall_lo), qc=flags)


def trace_record(op, epoch, frame, wb, ob, dims, qc, obs, header, delta):
    nob = len(obs) if obs is not None else 0
    out = REC_HEAD.pack(op, epoch, frame, *[int(v) for v in wb], *[int(v) for v in ob], *[int(v) for v in dims],
                        qc, nob, len(delta), 0, 0)
    if nob:
        out += bytes(obs) + b"\0" * ((nob + 7) // 8 * 8 - nob)
    out += np.asarray(header, "<u4").tobytes() + np.asarray(delta, "<u8").tobytes()
    return out


def raw_record(r):
    """Re-serialise a read_trace record unchanged."""
    nob = len(r["obs"]) if r["obs"] is not None else 0
    out = REC_HEAD.pack(r["op"], r["epoch"], r["frame"], *[int(v) for v in r["wb"]], *[int(v) for v in r["ob"]],
                        *[int(v) for v in r["dims"]], r["qc"], nob, r["delta"].size, r["qout"].size, r["dump"].size)
    if nob:
        out += bytes(r["obs"]) + b"\0" * ((nob + 7) // 8 * 8 - nob)
    if r["op"] == OP_QUERY and r["qc"] <= QCAP:
        out += np.asarray(r["query"], "<i4").tobytes()
    out += (np.asarray(r["header"], "<u4").tobytes() + np.asarray(r["delta"], "<u8").tobytes() +
            np.asarray(r["qout"], "<u4").tobytes() + np.asarray(r["dump"], "<u8").tobytes())
    return out


def write_trace(path, chunks):
    with open(path, "wb") as f:
        f.write(b"M7RT" + struct.pack("<II", 1, len(chunks)))
        for c in chunks:
            f.write(c)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trace", default=DEFAULT_TRACE)
    ap.add_argument("--out", required=True)
    ap.add_argument("--trace-out", default="")
    ap.add_argument("--cosim-out", default="", help="short trace: first records up to --cosim-grids GRID calls")
    ap.add_argument("--cosim-grids", type=int, default=3)
    a = ap.parse_args(argv)

    tmp = tempfile.mkdtemp(prefix="b1int_")
    off = build_roll(os.path.join(tmp, "off.so"))
    on = build_roll(os.path.join(tmp, "on.so"), ["-DFPGA_M7_ROLL_GRID"])
    muts = {m: build_roll(os.path.join(tmp, "m_%s.so" % m), ["-DFPGA_M7_ROLL_GRID", "-DFPGA_M7_ROLL_MUTANT_" + m])
            for m in ("GRID_RING_Y", "GRID_LAST_AUX")}
    model = PlCore(build_costmap(os.path.join(tmp, "cm.so")))
    band = rnl.HeightBand(ground_z_m=GROUND_Z, robot_height_m=ROBOT_H)
    params = nav2_ref.InflationParams(resolution=V, inflation_radius=0.45, cost_scaling_factor=3.0,
                                      robot_radius=BODY_R)
    lut = params.pl_lut(LUT_DIM).astype(np.uint8)
    lut_b = lut.tobytes()
    # second table (wider, flatter) for the every-4th-frame extra GRID call: the integrated
    # op must match the C model for any valid table, not just the car's
    lut2 = nav2_ref.InflationParams(resolution=V, inflation_radius=0.60, cost_scaling_factor=1.0,
                                    robot_radius=BODY_R).pl_lut(LUT_DIM).astype(np.uint8)

    res = {"records": 0, "off_mismatched_records": 0, "on_mismatched_records": 0, "op_counts": {},
           "grid_calls": 0, "grid_calls_by_flags": {}, "grid_words_compared": 0, "grid_words_diff": 0,
           "grid_header_diff": 0, "grid_status_bad": 0, "model_status_bad": 0,
           "bridge_frames": 0, "bridge_grid_cells_diff": 0, "bridge_strict_cells_diff": 0,
           "bridge_tall_cells_diff": 0, "exact_cost_frames": 0, "exact_cost_cells_diff": 0,
           "skipped_band_outside_window": 0, "first_differences": []}
    ctl = {"GRID_RING_Y_detected": 0, "GRID_LAST_AUX_detected": 0, "mutant_tried": 0,
           "wrong_band_detected": 0, "wrong_band_tried": 0}
    chunks, cosim_chunks, cosim_grids = [], [], 0
    recs = read_trace(a.trace)
    br = None
    frames = 0
    rng = np.random.RandomState(20261102)
    for idx, r in enumerate(recs):
        res["records"] += 1
        res["op_counts"][str(r["op"])] = res["op_counts"].get(str(r["op"]), 0) + 1
        kw = dict(epoch=r["epoch"], frame=r["frame"], wb=r["wb"], ob=r["ob"], dims=r["dims"], obs=r["obs"],
                  qc=r["qc"], query=r["query"] if r["op"] == OP_QUERY else None)
        outs = {}
        for name, core in [("off", off), ("on", on)] + sorted(muts.items()):
            hdr, delta, qout, dump = core.call(r["op"], **kw)
            ok = int(hdr[0]) == 0
            nd = int(hdr[H_N_DELTA]) if ok and r["op"] == OP_INTEGRATE else 0
            outs[name] = (hdr, delta[:nd], qout[:r["qc"]] if ok and r["op"] == OP_QUERY else None,
                          dump if ok and r["op"] == OP_DUMP else None)
        for name in ("off", "on"):
            if compare(r, *outs[name]):
                res[name + "_mismatched_records"] += 1
        chunks.append(raw_record(r))
        if cosim_grids < a.cosim_grids:
            cosim_chunks.append(chunks[-1])
        if r["op"] == OP_RESET and int(r["header"][0]) == 0:
            br = new_bridge(band, False)
            continue
        if r["op"] != OP_INTEGRATE or int(r["header"][0]) != 0:
            continue
        frames += 1
        now = frames * 100_000_000
        base = tuple(signed(r["header"][5 + k]) for k in range(3))
        nd_ = NavigationDelta(int(r["epoch"]), int(r["header"][3]), int(r["frame"]), now, "b1", now + 10 ** 15,
                              base, (MX, MY, MZ), V, decode_delta(r["header"], r["delta"]), "trace")
        ok1, why1, _ = br.on_delta(nd_, now_ns=now)
        if not ok1:
            raise RuntimeError("bridge rejected frame %d: %s" % (frames, why1))
        try:
            band.band_layers(base[2], MZ, V)
        except ValueError:
            res["skipped_band_outside_window"] += 1
            continue
        dh, _, _, words = off.call(OP_DUMP)
        offs = [int(dh[8 + k]) for k in range(3)]
        bp, tall_lo = band_params(base[2], band)
        xy = body_pose(base, frames)
        body = body_q(base, xy, BODY_R + BODY_MARGIN)
        # body sweep (every 8th frame): random centres inside / on the edge / outside the window
        # and radii 0..40 voxels, body seed only, integrated op vs C model
        if frames % 8 == 0:
            for _ in range(12):
                cxq = int(rng.randint(-30 * 65536, 110 * 65536))
                cyq = int(rng.randint(-30 * 65536, 110 * 65536))
                rq = int(rng.choice([0, 1, 65535, 65536, int(rng.randint(0, 40 * 65536 + 1)), 40 * 65536]))
                bodyr = (1, cxq, cyq, rq)
                st, g, aux, _ = model.run(words, offs, bp, tall_lo, bodyr, lut, infl=False)
                h, dl, _, _ = on.call(OP_GRID, obs=lut_b, **grid_args(bp, tall_lo, bodyr, 1))
                want = np.concatenate([pack(g.view(np.uint8)), pack(aux)])
                res["body_sweep_calls"] = res.get("body_sweep_calls", 0) + 1
                res["body_sweep_seeded_cells"] = res.get("body_sweep_seeded_cells", 0) + int(
                    ((aux & 3) == 0).sum() - (g == -1).sum())
                if st != 0 or int(h[0]) != 0 or np.any(dl[:want.size] != want):
                    res["body_sweep_diff"] = res.get("body_sweep_diff", 0) + 1
        flag_set = [3] + ([1, 2, 0, 7] if frames % 4 == 0 else [])
        for fl in flag_set:
            flags = fl & 3
            tab_ = lut2 if fl == 7 else lut
            be = flags & 1
            infl = (flags >> 1) & 1
            st, g, aux, cost = model.run(words, offs, bp, tall_lo, (be,) + body[1:], tab_, infl=bool(infl))
            if st != 0:
                res["model_status_bad"] += 1
                continue
            want = np.concatenate([pack(g.view(np.uint8)), pack(aux)] + ([pack(cost)] if infl else []))
            ga = grid_args(bp, tall_lo, body, flags)
            hdr, delta, _, _ = on.call(OP_GRID, obs=tab_.tobytes(), **ga)
            res["grid_calls"] += 1
            key = "3_table2" if fl == 7 else str(flags)
            res["grid_calls_by_flags"][key] = res["grid_calls_by_flags"].get(key, 0) + 1
            if int(hdr[0]) != 0:
                res["grid_status_bad"] += 1
                continue
            n = int(hdr[H_N_DELTA])
            got = delta[:n]
            res["grid_words_compared"] += want.size
            d = int(np.count_nonzero(got != want)) if n == want.size else want.size
            res["grid_words_diff"] += d
            eh = dh.copy()
            eh[H_OP], eh[H_N_DELTA] = OP_GRID, want.size
            hd = int(np.count_nonzero(hdr != eh))
            res["grid_header_diff"] += hd
            if (d or hd) and len(res["first_differences"]) < 8:
                res["first_differences"].append({"frame": frames, "flags": flags, "words": d, "header": hd})
            chunks.append(trace_record(OP_GRID, 0, 0, (0, 0, 0), ga["ob"], ga["dims"], flags, tab_.tobytes(), eh,
                                       want))
            if cosim_grids < a.cosim_grids:
                cosim_chunks.append(chunks[-1])
                cosim_grids += 1 if fl == 3 else 0
            if fl == 3:
                # vs the Nano bridge and the exact Nav2 reference
                sw_grid = br.grid(body_xy=xy[:2], now_ns=now, body_yaw=0.0).data.astype(np.int8)
                sw_strict = br.strict_grid().data.astype(np.int8)
                sw_tall = rnl.tall_occupied(br.mirror.states, base[2], V, GROUND_Z + rnl.SWEPT_EVIDENCE_MIN_H_M,
                                            band.band_layers(base[2], MZ, V)).ravel()
                gb = np.frombuffer(got[:PW].astype("<u8").tobytes(), np.int8)
                ab = np.frombuffer(got[PW:2 * PW].astype("<u8").tobytes(), np.uint8)
                cb = np.frombuffer(got[2 * PW:3 * PW].astype("<u8").tobytes(), np.uint8)
                res["bridge_frames"] += 1
                res["bridge_grid_cells_diff"] += int((gb != sw_grid).sum())
                res["bridge_strict_cells_diff"] += int((rnl.occupancy_values(ab & 3) != sw_strict).sum())
                res["bridge_tall_cells_diff"] += int((((ab >> 2) & 1).astype(bool) != sw_tall).sum())
                exact = nav2_ref.inflate_exact(nav2_ref.master_from_grid(sw_grid.reshape(MY, MX)), params)
                res["exact_cost_frames"] += 1
                res["exact_cost_cells_diff"] += int((cb.reshape(MY, MX).astype(np.int32) != exact).sum())
                # controls
                ctl["mutant_tried"] += 1
                for m, core in muts.items():
                    mh, md, _, _ = core.call(OP_GRID, obs=lut_b, **ga)
                    if int(mh[0]) != 0 or np.any(md[:want.size] != want):
                        ctl[m + "_detected"] += 1
                if bp[0] + 1 <= bp[1]:
                    ctl["wrong_band_tried"] += 1
                    wa = grid_args((bp[0] + 1, bp[1]), max(tall_lo, bp[0] + 1), body, flags)
                    wh, wd, _, _ = on.call(OP_GRID, obs=lut_b, **wa)
                    if int(wh[0]) != 0 or np.any(wd[:want.size] != want):
                        ctl["wrong_band_detected"] += 1

    # rejections (fresh feature-on core; then the live one)
    rej = {}
    fresh = build_roll(os.path.join(tmp, "fresh.so"), ["-DFPGA_M7_ROLL_GRID"])
    h, _, _, _ = fresh.call(OP_GRID, obs=lut_b, ob=(0, 0, 0), dims=(5, 10, 8), qc=3)
    rej["before_reset"] = [int(h[0]), int(h[H_CHANGED])]
    dh, _, _, _ = on.call(OP_DUMP)
    good = dict(ob=(40 * 65536, 40 * 65536, 7 * 65536), dims=(5, 10, 8), qc=3)
    bad_lut = bytearray(lut_b)
    bad_lut[13] = 1
    cases = {"band_lo_gt_hi": dict(good, dims=(10, 5, 8)), "band_hi_ge_mz": dict(good, dims=(5, 32, 8)),
             "tall_below_band": dict(good, dims=(5, 10, 4)), "tall_above_band": dict(good, dims=(5, 10, 12)),
             "flags_bit2": dict(good, qc=4), "radius_too_big": dict(good, ob=(0, 0, 41 * 65536)),
             "centre_out_of_range": dict(good, ob=(1 << 28, 0, 65536))}
    for k, kw in cases.items():
        h, _, _, _ = on.call(OP_GRID, obs=lut_b, **kw)
        rej[k] = [int(h[0]), int(h[H_CHANGED])]
    h, _, _, _ = on.call(OP_GRID, obs=bytes(bad_lut), **good)
    rej["table_last_column_nonzero"] = [int(h[0]), int(h[H_CHANGED])]
    mono = bytearray(lut_b)
    mono[3 * LUT_DIM + 2], mono[3 * LUT_DIM + 1] = 200, 100      # row 3 rises along |dy|
    h, _, _, _ = on.call(OP_GRID, obs=bytes(mono), **good)
    rej["table_row_not_monotone"] = [int(h[0]), int(h[H_CHANGED])]
    t00 = bytearray(lut_b)
    t00[0] = 253
    h, _, _, _ = on.call(OP_GRID, obs=bytes(t00), **good)
    rej["table_00_not_254"] = [int(h[0]), int(h[H_CHANGED])]
    h, _, _, _ = on.call(OP_GRID, obs=bytes(bad_lut), **dict(good, qc=1))
    rej["bad_table_ignored_without_inflation"] = [int(h[0]), int(h[H_CHANGED])]
    h, _, _, _ = off.call(OP_GRID, obs=lut_b, **good)
    rej["feature_off_op4"] = [int(h[0]), int(h[H_CHANGED])]
    dh2, _, _, _ = on.call(OP_DUMP)
    rej["state_unchanged_by_rejections"] = bool(np.all(dh == dh2))
    # rejection records for the HLS trace (expected header from the C++ core: status/reason checked above)
    for k in ("band_lo_gt_hi", "flags_bit2"):
        kw = cases[k]
        h, _, _, _ = on.call(OP_GRID, obs=lut_b, **kw)
        chunks.append(trace_record(OP_GRID, 0, 0, (0, 0, 0), kw["ob"], kw["dims"], kw["qc"], lut_b, h, []))
    res["rejections"] = rej
    res["controls"] = ctl
    res["frames_integrate_accepted"] = frames
    res["params"] = {"band": "D-035 ground 0.0 m, robot height 0.31 m", "body": "synthetic pose, r 0.35 m",
                     "nav2": "inflation 0.45 m, scaling 3.0, robot radius 0.30 m", "lut_row0": [int(v) for v in lut[:LUT_DIM]]}
    exp_rej = {"before_reset": [1, 0], "band_lo_gt_hi": [2, 2], "band_hi_ge_mz": [2, 2], "tall_below_band": [2, 2],
               "tall_above_band": [2, 2], "flags_bit2": [2, 5], "radius_too_big": [2, 3], "centre_out_of_range": [2, 3],
               "table_last_column_nonzero": [2, 4], "bad_table_ignored_without_inflation": [0, 0],
               "table_row_not_monotone": [2, 4], "table_00_not_254": [2, 4],
               "feature_off_op4": [2, 0]}
    ok = (res["off_mismatched_records"] == 0 and res["on_mismatched_records"] == 0 and res["grid_words_diff"] == 0
          and res["grid_header_diff"] == 0 and res["grid_status_bad"] == 0 and res["model_status_bad"] == 0
          and res["bridge_grid_cells_diff"] == 0 and res["bridge_strict_cells_diff"] == 0
          and res["bridge_tall_cells_diff"] == 0 and res["exact_cost_cells_diff"] == 0
          and res.get("body_sweep_diff", 0) == 0 and res.get("body_sweep_calls", 0) > 0
          and all(rej[k] == v for k, v in exp_rej.items()) and rej["state_unchanged_by_rejections"]
          and ctl["GRID_RING_Y_detected"] > 0 and ctl["GRID_LAST_AUX_detected"] == ctl["mutant_tried"]
          and ctl["wrong_band_detected"] > 0)
    res["verdict"] = "PASS" if ok else "FAIL"
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(res, f, indent=1)
    if a.trace_out:
        write_trace(a.trace_out, chunks)
    if a.cosim_out:
        write_trace(a.cosim_out, cosim_chunks)
    print(json.dumps({k: res[k] for k in ("records", "off_mismatched_records", "on_mismatched_records", "grid_calls",
                                          "grid_words_compared", "grid_words_diff", "grid_header_diff",
                                          "bridge_frames", "bridge_grid_cells_diff", "exact_cost_cells_diff",
                                          "controls", "rejections", "verdict")}))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
