#!/usr/bin/env python3
"""Summarize per-frame parameters of FPGAGPK01 packs (resolution, grid, voxel, unit, K^-1, [R|t], origin)."""
import struct, sys, numpy as np
R = "${FPGA_RUN}"
P = {"rec30": R+"/E2/packs/rec30.pack", "tum200": R+"/E2/packs/tum200.pack", "car570": R+"/E2/packs/car570.pack",
     "tum1209": R+"/G1d/packs/tum1209.pack", "car1015": R+"/G1d/packs/car1015.pack"}
for name, p in P.items():
    with open(p, "rb") as f:
        hdr = f.read(24); n = struct.unpack("<I", hdr[12:16])[0]
        rows = []
        for i in range(n):
            h = f.read(144)
            w, hh, nx, ny, nz, _ = struct.unpack("<6I", h[:24])
            unit = struct.unpack("<f", h[24:28])[0]
            ik = struct.unpack("<9f", h[28:64]); wc = struct.unpack("<12f", h[64:112])
            org = struct.unpack("<3d", h[112:136]); vox = struct.unpack("<d", h[136:144])[0]
            d = np.frombuffer(f.read(w*hh*2), np.uint16)
            rows.append((w, hh, nx, ny, nz, unit, ik, wc, org, vox, int(d.max()), float((d == 0).mean())))
    ws = sorted({(r[0], r[1]) for r in rows}); gs = sorted({(r[2], r[3], r[4]) for r in rows})
    print(f"== {name} frames={n} res={ws} grid={gs} vox={sorted({r[9] for r in rows})} unit={sorted({r[5] for r in rows})}")
    iks = np.array([r[6] for r in rows]); wcs = np.array([r[7] for r in rows]); orgs = np.array([r[8] for r in rows])
    print("  ik distinct:", len({tuple(x) for x in iks}), "ik0:", np.round(iks[0], 7).tolist())
    print("  wc distinct:", len({tuple(x) for x in wcs}), "wc0:", np.round(wcs[0], 5).tolist())
    print("  wc absmax per col:", np.round(np.abs(wcs).max(0), 4).tolist())
    print("  org distinct:", len({tuple(x) for x in orgs}), "org range:", orgs.min(0).tolist(), orgs.max(0).tolist())
    print("  depth code max:", max(r[10] for r in rows), " zero frac mean:", round(np.mean([r[11] for r in rows]), 4))
