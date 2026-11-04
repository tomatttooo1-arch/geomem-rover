"""Regression: re-run process_frame + Memory on recorded simulation frames, old vs new code.

docs/design/MEMORY_QUALITY_20261016.md section 5. A run directory written by the runner
with --frames-dir (frames/<frame_id>.npz: rgb, depth, K, T_world_camera, timestamp) and
runtime.jsonl (frame_processed rows carry the detector boxes) is replayed through
  main  the Memory/process_frame of a git ref (default origin/main), and
  new   this tree's Memory/process_frame with the default Config.
Geometry evidence is recomputed on CPU with each arm's compute_geometry (both arms alike;
the run itself used the KV260 stand-in). Only reads the run directory.
"""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from perception.fpga_replay import core as new_core  # noqa: E402
from tools.memory_quality.replay_stream import core_from_git  # noqa: E402

SIM_BOTTLE_XY = (1.10, -0.45)   # robot_sim/fpga_sim/worlds/room_carmount.sdf rendered_props bottle


def replay(core, run, frame_ids):
    memory = core.Memory("geometry")
    rows = {}
    with open(run / "runtime.jsonl") as stream:
        for line in stream:
            r = json.loads(line)
            if r.get("event") == "frame_processed" and r.get("evidence"):
                rows[r["frame_id"]] = r
    trace = {}
    for fid in frame_ids:
        r = rows[fid]
        z = np.load(run / "frames" / r["evidence"]["file"])
        frame = core.Frame(fid, float(z["timestamp"]), z["rgb"], z["depth"], z["K"], z["T_world_camera"],
                           True, "reference", "replay")
        frame.ground_z_m = 0.0
        dets = [core.Detection(d["label"], d["score"], tuple(d["bbox"]), "replay", fid, frame.timestamp)
                for d in r["detections"]]
        obs = core.process_frame(frame, dets)
        memory.update_memory(obs, core.compute_geometry(frame, memory), frame)
        trace[fid] = memory.snapshot()
    return memory, trace, rows


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("runs", type=Path, nargs="+")
    p.add_argument("--main-ref", default="origin/main")
    p.add_argument("--out", type=Path)
    a = p.parse_args(argv)
    old_core = core_from_git(a.main_ref)
    report = {"main_ref": a.main_ref, "runs": {}}
    for run in a.runs:
        verify = json.loads((run / "verify.json").read_text()) if (run / "verify.json").exists() else {}
        ids = []
        with open(run / "runtime.jsonl") as stream:
            for line in stream:
                r = json.loads(line)
                if r.get("event") == "frame_processed" and r.get("evidence"):
                    ids.append(r["frame_id"])
        out = {"frames": len(ids), "target_id": verify.get("target_id"), "found_frame": verify.get("found_frame")}
        logged_found = None
        for name, core in (("main", old_core), ("new", new_core)):
            memory, trace, rows = replay(core, run, ids)
            check = verify.get("found_frame") if verify.get("found_frame") in trace else ids[-1]
            snap = trace[check]
            bottles = [t for t in snap if t["label"] == "bottle"]
            near = [t for t in bottles if math.dist(t["position_m"][:2], SIM_BOTTLE_XY) < 0.3]
            out[name] = {"checked_frame": check, "tracks": len(snap), "bottle_tracks": len(bottles),
                         "bottle_near_truth": [(t["id"], t["state"], [round(v, 3) for v in t["position_m"]])
                                               for t in near],
                         "target": next(((t["state"], [round(v, 4) for v in t["position_m"]]) for t in snap
                                         if t["id"] == verify.get("target_id")), None),
                         "rejected": dict(getattr(memory, "rejected", {}))}
            if name == "main" and check in rows:
                logged_found = {t["id"]: t["position_m"] for t in rows[check]["tracks"]}
        out["main_target_matches_logged"] = (
            logged_found is not None and out["main"]["target"] is not None and
            np.allclose(out["main"]["target"][1], logged_found.get(verify.get("target_id"), [np.nan] * 3), atol=1e-3))
        m, n = out["main"], out["new"]
        out["pass"] = bool(n["target"] is not None and n["target"][0] == "current_confirmed" and
                           m["target"] is not None and
                           math.dist(n["target"][1], m["target"][1]) <= 0.05 and
                           len(n["bottle_near_truth"]) >= len(m["bottle_near_truth"]) and
                           n["tracks"] <= m["tracks"])
        report["runs"][run.name] = out
    report["pass"] = all(r["pass"] for r in report["runs"].values())
    text = json.dumps(report, ensure_ascii=False, indent=1, default=float)
    if a.out:
        with a.out.open("x") as s:
            s.write(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
