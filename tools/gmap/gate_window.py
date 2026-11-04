#!/usr/bin/env python3
"""GMAP2 WINDOW adoption gate (DESIGN_ADD_PLGRID section 6, written before the run):
every frame's final window, integer reference (window_ref.WindowQ16, Q16 since GMAP3) vs today's float PlGridLayer (dumped by
dump_window_frames.py, car1015, config A). Pass: total differing cells <= 0.01 % of all window cells, and the
swept-cleared total within 0.1 % of the float total. Differing cells are reported with their distance to the
seed / body circles (in Q8 units) and the frame number.
  python3 gate_window.py FRAMES.npz OUT.json
"""
import json
import sys

import numpy as np

sys.path.insert(0, __import__("os").path.dirname(__file__))
import window_ref as wr  # noqa: E402


def main():
    z = np.load(sys.argv[1])
    n = len(z["now"])
    ref = wr.WindowQ16(int(z["now"][0]))
    diff_total, rows, cleared_q, evid_q = 0, [], 0, 0
    for k in range(n):
        p = ref.params(z["base"][k], z["body"][k], z["now"][k])
        code, cl, ev = ref.apply(z["aux"][k], p)
        cleared_q += cl
        evid_q += ev
        fl = z["out"][k].astype(np.int16)
        fl_code = np.where(fl == 100, 0x64, np.where(fl == 0, 0x00, 0xFF))
        bad = np.nonzero(fl_code != code)[0]
        diff_total += len(bad)
        for i in bad[:50]:
            ly, lx = divmod(int(i), 80)
            dx, dy = wr.axis_dist(p["cx"])[lx], wr.axis_dist(p["cy"])[ly]
            rows.append(dict(frame=k, lx=lx, ly=ly, float=int(fl[i]), q=int(code[i]),
                             d_body_q=float(np.sqrt(dx * dx + dy * dy)), cleared_float=int(z["cleared"][k]),
                             cleared_q=cl))
    cells = n * 6400
    cleared_f = int(z["cleared"].sum())
    res = dict(frames=n, window_cells=cells, diff_cells=diff_total, diff_limit=cells * 1e-4,
               cleared_float=cleared_f, cleared_q=cleared_q,
               cleared_rel=abs(cleared_q - cleared_f) / max(1, cleared_f),
               evidence_float=int(z["evidence"].sum()), evidence_q=evid_q,
               first_diffs=rows[:200])
    res["pass"] = bool(diff_total <= cells * 1e-4 and res["cleared_rel"] <= 1e-3)
    json.dump(res, open(sys.argv[2], "w"), indent=1)
    print({k: v for k, v in res.items() if k != "first_diffs"})


if __name__ == "__main__":
    main()
