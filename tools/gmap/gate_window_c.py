#!/usr/bin/env python3
"""GMAP3 WINDOW, on the fixed-point C model (tools/gmap/gmap_model.c, gmap_op 4, Q16), car1015 frames dumped by
dump_window_frames.py. Criteria written before the run (DESIGN_ADD_PLGRID section 6, unchanged by the lead 11-04):
  adoption gate  C model vs today's float PlGridLayer: differing cells over all frames <= 0.01 % of all window cells
                 (<= 783 for 1,224 frames) and swept-cleared total within 0.1 % of the float total;
  check (1)      C model vs the integer PS reference (window_ref.WindowQ16): every frame's 6,400 output codes, the whole
                 400 x 400 T map and both counters identical;
  positive ctrl  (a) r_b = 0.35 m and (b) r_e = 0.9 m given to the C model only: both must report differences
                 (first frame, differing cells).
The PS control part (capsule points, Q16 rounding, now / thr) comes from window_ref.params for every arm.
  python3 gate_window_c.py FRAMES.npz OUT.json
"""
import ctypes
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import window_ref as wr  # noqa: E402

AREA = 1173088
OFF_FRAME, OFF_RESULT, OFF_T, OFF_AUX, OFF_WP = 360000, 526416, 526432, 1166432, 1172832


def build_lib():
    d = tempfile.mkdtemp(prefix="gmap3_gate_")
    so = os.path.join(d, "libgmap.so")
    subprocess.check_call(["gcc", "-O2", "-shared", "-fPIC", "-o", so, os.path.join(HERE, "gmap_model.c")])
    lib = ctypes.CDLL(so)
    lib.gmap_run.restype = ctypes.c_uint32
    lib.gmap_run.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    return lib


class CArm(object):
    """one C-model map area; T starts as 'never' (CLEAR with byte 13 bit 0)."""

    def __init__(self, lib, overrides=None):
        self.lib, self.ov = lib, overrides or {}
        self.mem = np.zeros(AREA, np.uint8)
        prm = bytes([0x4D, 0x47, 0, 0, 0, 0, 0, 0, 254, 254, 205, 0, 1, 1, 0, 0])
        self.mem[366400:366416] = np.frombuffer(prm, np.uint8)
        st = lib.gmap_run(self.mem.ctypes.data, 3)
        assert st == 0x200, hex(st)
        assert (self.mem[OFF_T:OFF_AUX].view(np.int32) == wr.NEVER).all()

    def step(self, aux, p):
        self.mem[OFF_AUX:OFF_AUX + 6400] = aux
        self.mem[OFF_WP:OFF_WP + 256] = np.frombuffer(wr.param_block(p, **self.ov), np.uint8)
        st = self.lib.gmap_run(self.mem.ctypes.data, 4)
        res = self.mem[OFF_RESULT:OFF_RESULT + 16].view(np.uint32)
        return st, self.mem[OFF_FRAME:OFF_FRAME + 6400].copy(), int(res[1]), int(res[2])

    def tmap(self):
        return self.mem[OFF_T:OFF_AUX].view(np.int32).reshape(400, 400)


def main():
    z = np.load(sys.argv[1])
    n = len(z["now"])
    lib = build_lib()
    ref = wr.WindowQ16(int(z["now"][0]))
    arms = dict(model=CArm(lib), ctl_rb035=CArm(lib, dict(rb2=wr.rq2(0.35))),
                ctl_re09=CArm(lib, dict(re2=wr.rq2(0.9))))
    gate_diff, rows, cl_c, ev_c = 0, [], 0, 0
    chk = dict(frames_code_diff=0, frames_T_diff=0, frames_count_diff=0, status_bad=0, first_bad=None)
    ctl = {k: dict(first_frame=None, diff_frames=0, diff_cells=0) for k in arms if k != "model"}
    for k in range(n):
        p = ref.params(z["base"][k], z["body"][k], z["now"][k])
        code_r, cl_r, ev_r = ref.apply(z["aux"][k], p)
        for name, arm in arms.items():
            st, code, cl, ev = arm.step(z["aux"][k], p)
            if name == "model":
                cl_c += cl
                ev_c += ev
                bad = False
                if st != 0x200:
                    chk["status_bad"] += 1
                    bad = True
                if not np.array_equal(code, code_r):
                    chk["frames_code_diff"] += 1
                    bad = True
                if (cl, ev) != (cl_r, ev_r):
                    chk["frames_count_diff"] += 1
                    bad = True
                if not np.array_equal(arm.tmap(), ref.T.astype(np.int32)):
                    chk["frames_T_diff"] += 1
                    bad = True
                if bad and chk["first_bad"] is None:
                    chk["first_bad"] = k
                fl = z["out"][k].astype(np.int16)
                fl_code = np.where(fl == 100, 0x64, np.where(fl == 0, 0x00, 0xFF))
                badc = np.nonzero(fl_code != code)[0]
                gate_diff += len(badc)
                for i in badc[:50]:
                    ly, lx = divmod(int(i), 80)
                    rows.append(dict(frame=k, lx=lx, ly=ly, float=int(fl[i]), c_model=int(code[i])))
            else:
                nd = int((code != code_r).sum()) + (0 if (cl, ev) == (cl_r, ev_r) else 1)
                if nd:
                    c = ctl[name]
                    c["diff_frames"] += 1
                    c["diff_cells"] += int((code != code_r).sum())
                    if c["first_frame"] is None:
                        c["first_frame"] = k
    cells = n * 6400
    cleared_f = int(z["cleared"].sum())
    gate = dict(frames=n, window_cells=cells, diff_cells=gate_diff, diff_limit=cells * 1e-4,
                cleared_float=cleared_f, cleared_c=cl_c, cleared_rel=abs(cl_c - cleared_f) / max(1, cleared_f),
                evidence_float=int(z["evidence"].sum()), evidence_c=ev_c)
    gate["pass"] = bool(gate_diff <= cells * 1e-4 and gate["cleared_rel"] <= 1e-3)
    chk["pass"] = bool(chk["first_bad"] is None)
    for c in ctl.values():
        c["pass"] = c["first_frame"] is not None
    res = dict(adoption_gate=gate, check1_c_vs_ps_ref=chk, positive_controls=ctl, first_gate_diffs=rows[:200])
    json.dump(res, open(sys.argv[2], "w"), indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != "first_gate_diffs"}, indent=1))


if __name__ == "__main__":
    main()
