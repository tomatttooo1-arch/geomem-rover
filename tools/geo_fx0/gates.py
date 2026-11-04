#!/usr/bin/env python3
"""GEO_FX0: judge the pre-registered gates I0, G1-G6, G8, P (DESIGN.md 6.1) from data/<name>.csv. G7: downstream.py; S: stats_check.py."""
import csv, json, sys
G = "${FPGA_RUN}/GEO_FX0"
names = sys.argv[1:] or ["rec30", "car570"]
out = {}
for n in names:
    rows = list(csv.DictReader(open("%s/data/%s.csv" % (G, n))))
    I = lambda k: sum(int(r[k]) for r in rows)
    F = lambda k: max(float(r[k]) for r in rows)
    valid = I("valid_ref")
    # G5 per frame with small-denominator rule
    big = [r for r in rows if int(r["known_union"]) >= 1000]
    small = [r for r in rows if int(r["known_union"]) < 1000]
    worst = max((int(r["cell_mis"]) / int(r["known_union"]) for r in big), default=0.0)
    over = sum(1 for r in big if int(r["cell_mis"]) / int(r["known_union"]) > 1e-3)
    sd = sum(int(r["known_union"]) for r in small)
    sm = sum(int(r["cell_mis"]) for r in small) / sd if sd else 0.0
    cbig = [r for r in rows if int(r["c_known_union"]) >= 1000]
    c_over = sum(1 for r in cbig if int(r["c_cell_mis"]) / int(r["c_known_union"]) > 1e-3)
    csm = [r for r in rows if int(r["c_known_union"]) < 1000]
    csd = sum(int(r["c_known_union"]) for r in csm)
    c_sm = sum(int(r["c_cell_mis"]) for r in csm) / csd if csd else 0.0
    d = {
        "frames": len(rows), "st_fx_nonzero": sum(1 for r in rows if r["st_fx"] != "0"),
        "I0_bytes": I("i0_bytes"),
        "G1_valid_diff": I("valid_diff"),
        "G2_bfx_viol": I("bfx_viol"),
        "G3_endd_gt1": I("endd_gt1"), "endd1_axis_values": I("endd1"), "valid_rays": valid,
        "G4_ep_mis": I("ep_mis"), "G4_ep_mis_frac": I("ep_mis") / valid, "G4_ep_unattr": I("ep_unattr"),
        "G5_worst_frame": worst, "G5_frames_over": over, "G5_small_frames": len(small), "G5_small_merged": sm,
        "G5_cell_mis_total": I("cell_mis"), "G5_known_total": I("known_union"),
        "G6_occ_ref": I("occ_ref"), "G6_occ_lost": I("occ_lost"), "G6_occ_lost_frac": I("occ_lost") / max(1, I("occ_ref")),
        "occ_extra": I("occ_extra"), "free_ref": I("free_ref"), "free_to_unk": I("free_to_unk"),
        "unk_to_free": I("unk_to_free"), "free_to_occ": I("free_to_occ"),
        "G8_rays_ref": I("rays_ref"), "G8_rays_fx": I("rays_fx"), "G8_steps_ref": I("steps_ref"), "G8_steps_fx": I("steps_fx"),
        "max_err_fx_q8": F("max_efx"), "max_err_ref_q8": F("max_eref"), "max_bref_q8": F("max_bref"),
        "pmax": F("pmax"), "reach": F("reach"), "c_lo_hi_c20": (rows[0]["c_lo"], rows[0]["c_hi"], rows[0]["c20"]),
        "P_ctl_bfx_viol": I("c_bfx_viol"), "P_ctl_ep_mis": I("c_ep_mis"), "P_ctl_ep_unattr": I("c_ep_unattr"),
        "P_ctl_endd_gt1": I("c_endd_gt1"), "P_ctl_G5_frames_over": c_over, "P_ctl_G5_small_merged": c_sm, "P_ctl_cell_mis": I("c_cell_mis"),
        "P_ctl_occ_lost": I("c_occ_lost"), "P_ctl_occ_lost_frac": I("c_occ_lost") / max(1, I("occ_ref")),
    }
    d["G8_rays_rel"] = (d["G8_rays_fx"] - d["G8_rays_ref"]) / d["G8_rays_ref"]
    d["G8_steps_rel"] = (d["G8_steps_fx"] - d["G8_steps_ref"]) / d["G8_steps_ref"]
    d["pass"] = {
        "I0": d["I0_bytes"] == 0 and d["st_fx_nonzero"] == 0, "G1": d["G1_valid_diff"] == 0, "G2": d["G2_bfx_viol"] == 0,
        "G3": d["G3_endd_gt1"] == 0, "G4": d["G4_ep_unattr"] == 0,
        "G5": over == 0 and sm <= 1e-3, "G6": d["G6_occ_lost_frac"] <= 1e-4 and d["G4_ep_unattr"] == 0,
        "G8": abs(d["G8_rays_rel"]) <= 1e-3 and abs(d["G8_steps_rel"]) <= 1e-3,
        "P_caught": d["P_ctl_bfx_viol"] > 0 and d["P_ctl_ep_unattr"] > 0,
        "P_info_G5_catches": c_over > 0 or c_sm > 1e-3, "P_info_G6_catches": d["P_ctl_occ_lost_frac"] > 1e-4,
    }
    out[n] = d
json.dump(out, open("%s/data/gates.json" % G, "w"), indent=1)
for n, d in out.items():
    print(n, json.dumps(d["pass"]))
    print("  ", {k: v for k, v in d.items() if k != "pass"})
