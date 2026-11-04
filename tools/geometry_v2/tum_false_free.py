"""Usage: python3 tools/geometry_v2/tum_false_free.py <repo root> [min detection confidence]

False-free rate of geometry v2 on real RGB-D: TUM fr1/desk (static scene, mocap poses), tracks from the
frozen M3 detections (12 frames). Every object stays in place, so every 'free' is a false free."""
import json, sys, collections, dataclasses
import numpy as np
sys.path.insert(0, sys.argv[1])
from perception.fpga_replay import core
from perception.fpga_replay.io import tum_records, load_frame
DATA = "${FPGA_SHARED}/data/20260907-dev1-replay/extracted/rgbd_dataset_freiburg1_desk"
BATCH = "${FPGA_SHARED}/artifacts/20260923-dev1-m3-batch-mirror/m3-batch-20260916/batch"
minconf = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
base = dataclasses.replace(core.Config(**core.UNGATED_CREATION), ground_z_m=-100.0)
cfg1, cfg2 = base, dataclasses.replace(base, geometry_v2=True)
recs = tum_records(DATA, stride=1, limit=100000)
byid = {r["frame_id"]: r for r in recs}
dets = {}
for i in range(1, 13):
    r = json.load(open("%s/image-%03d/result.json" % (BATCH, i)))
    fid = r["image"]["timestamp_source"]["row"].split()[0]
    dets[fid] = [d for d in r["detections"] if d["confidence"] >= minconf]
m = core.Memory("geometry", cfg2)
for fid in sorted(dets):
    rec = byid.get(fid)
    if rec is None or not rec["pose_valid"]:
        print("skip build frame", fid); continue
    f = load_frame(rec, DATA)
    obs = core.process_frame(f, [core.Detection(d["class_name"], d["confidence"], tuple(d["bbox_xyxy"]),
                                                frame_id=f.frame_id, timestamp=f.timestamp) for d in dets[fid]], cfg2)
    m.update_memory(obs, {}, f)
print("tracks", len(m.tracks), collections.Counter(t.label for t in m.tracks.values()))
c1, c2 = collections.Counter(), collections.Counter()
lab_free = collections.Counter(); lab_checked = collections.Counter(); lab_free1 = collections.Counter()
frames_with_free = 0; n = 0
speeds, free_speeds = [], []
prev = None
for rec in recs:
    if not rec["pose_valid"]:
        prev = None; continue
    f = load_frame(rec, DATA); n += 1
    T = f.T_world_camera
    speed = None if prev is None else float(np.linalg.norm(T[:3, 3] - prev[1][:3, 3]) / max(f.timestamp - prev[0], 1e-3))
    prev = (f.timestamp, T)
    if speed is not None:
        speeds.append(speed)
    w1 = core.compute_geometry(f, m, cfg1); w2 = core.compute_geometry(f, m, cfg2)
    c1.update(w1.values()); c2.update(w2.values())
    anyfree = False
    for t in m.tracks.values():
        if w2[t.id] in ("free", "surface_or_uncertain", "occluded"):
            lab_checked[t.label] += 1
        if w2[t.id] == "free":
            lab_free[t.label] += 1; anyfree = True
        if w1[t.id] == "free":
            lab_free1[t.label] += 1
    frames_with_free += anyfree
    if anyfree and speed is not None:
        free_speeds.append(speed)
def rate(c):
    chk = c["free"] + c["surface_or_uncertain"] + c["occluded"]
    return "free %d / checked %d = %.4f" % (c["free"], chk, c["free"] / max(chk, 1))
print("frames", n, "minconf", minconf)
print("v1", dict(c1), rate(c1)); print("v2", dict(c2), rate(c2))
print("frames with any v2 free", frames_with_free)
print("per label v2 free/checked", {k: (lab_free[k], lab_checked[k]) for k in lab_checked}, "v1 free", dict(lab_free1))
print("camera speed m/s median all frames %.3f, frames with a v2 free %.3f" % (np.median(speeds), np.median(free_speeds)))
