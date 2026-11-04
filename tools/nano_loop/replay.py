#!/usr/bin/env python3
"""J4 offline replay: invalid / occluded depth must never yield "removed" evidence.

Feeds recorded frames (colour sha, uint16 depth, recorded detector batches) through the same
FixedCameraChain (M7Pipeline + CPU reference backends). In a window of frames the depth of a
watched object is modified and that object's detections are dropped (the detector "no longer
sees it"), so the explicit-negative path (all-False mask + historical bounds) is exercised:

  zero_all        whole depth frame 0            -> expect no free_space_conflict
  zero_region     object ROI depth 0             -> expect no free_space_conflict
  occlude_region  object ROI moved in front      -> expect no free_space_conflict
  remove_object   object ROI = background +0.5 m -> expect free_space_conflict (positive control)

The deliberately wrong variant treats missing depth as free space: 0 is replaced by 1.5 m (the far
side of the 1.6 m M7 grid; 9.9 m is also reported). The same check must then report FAIL.
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.nano_loop.chain import WATCH_ARM_OBS, WATCH_IOU, FixedCameraChain, iou  # noqa: E402

MODES = ("zero_all", "zero_region", "occlude_region", "remove_object")
EXPECT_CONFLICT = {"zero_all": False, "zero_region": False, "occlude_region": False, "remove_object": True}
WRONG_FREE_M = 1.5


VACUOUS = ("historical_region_out_of_view", "historical_region_too_small", "no_historical_bounds")


def in_view(bounds, K, shape):
    """Same projection test as the object backend: every bounds corner in front and inside the image."""
    corners = np.array([[x, y, z] for x in bounds[:, 0] for y in bounds[:, 1] for z in bounds[:, 2]])
    if (corners[:, 2] <= .1).any():
        return False
    uvz = corners @ K.T
    uv = uvz[:, :2] / uvz[:, 2:]
    lo, hi = np.floor(uv.min(axis=0)), np.ceil(uv.max(axis=0))
    h, w = shape
    return lo[0] >= 0 and lo[1] >= 0 and hi[0] < w and hi[1] < h and np.all(hi - lo >= 3)


def roi_of(bounds, K, shape, pad=4):
    corners = np.array([[x, y, z] for x in bounds[:, 0] for y in bounds[:, 1] for z in bounds[:, 2]])
    uvz = corners @ K.T
    uv = uvz[:, :2] / uvz[:, 2:]
    lo = np.floor(uv.min(axis=0)).astype(int) - pad
    hi = np.ceil(uv.max(axis=0)).astype(int) + pad
    h, w = shape
    return max(lo[0], 0), max(lo[1], 0), min(hi[0], w - 1), min(hi[1], h - 1)


def modify(depth, mode, target, K):
    d = depth.copy()
    if mode == "zero_all":
        d[:] = 0
        return d
    x0, y0, x1, y1 = roi_of(target["bounds"], K, d.shape)
    near, far = float(target["bounds"][0, 2]), float(target["bounds"][1, 2])
    if mode == "zero_region":
        d[y0:y1 + 1, x0:x1 + 1] = 0
    elif mode == "occlude_region":
        d[y0:y1 + 1, x0:x1 + 1] = int(round(max(near - 0.15, 0.12) * 1000))
    elif mode == "remove_object":
        d[y0:y1 + 1, x0:x1 + 1] = int(round((far + 0.5) * 1000))
    else:
        raise ValueError(mode)
    return d


def run_scenario(frames, dets_by_index, make_chain, mode, window, *, wrong=False, target_label=None,
                 wrong_free_m=WRONG_FREE_M):
    """frames: list of dicts(color_index, acquired_ns, rgb_sha256, depth(uint16)). Returns report."""
    chain = make_chain()
    target = None
    rows = []
    for k, f in enumerate(frames):
        ci = int(f["color_index"])
        dets = list(dets_by_index.get(ci, []))
        depth = f["depth"]
        inside = window[0] <= k < window[1]
        if inside and target is None:
            armed = [w for w in chain.watch if w["n"] >= WATCH_ARM_OBS and
                     (target_label is None or w["label"] == target_label) and
                     in_view(w["bounds"], chain.K, depth.shape)]
            if not armed:
                raise RuntimeError("no watched object armed and fully in view before the window")
            target = dict(max(armed, key=lambda w: w["n"]))
            target["bounds"] = target["bounds"].copy()
        if inside:
            depth = modify(depth, mode, target, chain.K)
            dets = [d for d in dets if not (d["name"] == target["label"] and
                                            iou(d["box"], target["box"]) >= WATCH_IOU)]
        acq = int(f["acquired_ns"])
        _, gsum = chain.ingest_frame(ci, acq, depth, acq + 1_000_000, rgb_sha256=f["rgb_sha256"],
                                     invalid_as_free_m=wrong_free_m if wrong else None)
        res, _ = chain.consume_detections(ci, dets, acq + 2_000_000)
        mine = [n for n in res["negatives"] if target is not None and n["watch"] == target["key"]]
        rows.append({"k": k, "color_index": ci, "window": inside, "geometry": gsum["reason"],
                     "target_negatives": mine,
                     "any_free_space_conflict": any(n["candidate"] == "free_space_conflict"
                                                    for n in res["negatives"])})
    win = [r for r in rows if r["window"]]
    conflicts = sum(any(n["candidate"] == "free_space_conflict" for n in r["target_negatives"]) for r in win)
    evaluated = sum(bool(r["target_negatives"]) for r in win)
    geometry_invalid = sum(r["geometry"] != "geometry_ready" for r in win)
    outcomes = {}
    for r in win:
        for n in r["target_negatives"]:
            key = "%s:%s" % (n["candidate"], n["reason"])
            outcomes[key] = outcomes.get(key, 0) + 1
        if not r["target_negatives"]:
            key = "no_negative_evaluated:geometry=" + r["geometry"]
            outcomes[key] = outcomes.get(key, 0) + 1
    # Non-vacuous: every window frame is accounted for by invalid geometry or by a negative that
    # reached the depth test (out-of-view / too-small ROI never looks at depth, so it proves nothing).
    accounted = all(r["geometry"] != "geometry_ready" or
                    any(n["reason"] not in VACUOUS for n in r["target_negatives"]) for r in win)
    if EXPECT_CONFLICT[mode]:
        passed = accounted and conflicts >= 1
    else:
        passed = accounted and conflicts == 0
    return {"mode": mode, "wrong_variant_invalid_depth_as_free_m": wrong_free_m if wrong else None, "window_frames": len(win),
            "target": None if target is None else {"key": target["key"], "label": target["label"],
                                                   "box": [round(v, 1) for v in target["box"]],
                                                   "bounds_m": np.round(target["bounds"], 3).tolist()},
            "negatives_evaluated": evaluated, "geometry_invalid_frames": geometry_invalid,
            "free_space_conflict_frames": conflicts, "outcomes": outcomes,
            "expect_conflict": EXPECT_CONFLICT[mode], "accounted": accounted,
            "check": "PASS" if passed else "FAIL"}


def load_recording(root):
    import cv2
    frames = []
    for line in open(os.path.join(root, "frames.jsonl")):
        m = json.loads(line)
        depth = cv2.imread(os.path.join(root, m["depth_png"]), cv2.IMREAD_UNCHANGED)
        if depth is None or depth.dtype != np.uint16:
            raise ValueError("bad depth png " + m["depth_png"])
        frames.append(dict(m, depth=depth))
    dets = {}
    for line in open(os.path.join(root, "detections.jsonl")):
        m = json.loads(line)
        dets[int(m["color_index"])] = m["dets"]
    calib = json.load(open(os.path.join(root, "calibration.json")))
    return frames, dets, calib


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rec", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--window-start", type=float, default=0.5, help="fraction of the recording")
    ap.add_argument("--window-frames", type=int, default=30)
    ap.add_argument("--target-label", default="bottle")
    a = ap.parse_args()
    from host.m7_frame_native import FrameNativeM7, compile_frame_native
    from host.m7_object_native import NativeObjectEvidence, compile_object_native
    from tools.nano_detector.nano_detector import Letterbox
    os.makedirs(a.out, exist_ok=False)
    fb = FrameNativeM7(compile_frame_native(os.path.join(a.out, "m7_frame.so")))
    ob = NativeObjectEvidence(compile_object_native(os.path.join(a.out, "m7_object.so")))
    frames, dets, calib = load_recording(a.rec)
    frames = [f for f in frames if int(f["color_index"]) in dets]  # frames the detector actually saw
    h, w = frames[0]["depth"].shape
    lb = Letterbox(w, h, 640)
    start = int(len(frames) * a.window_start)
    window = (start, start + a.window_frames)

    def make_chain():
        return FixedCameraChain(fb, ob, calib, "replay", letterbox=lb, prediction_set="recorded",
                                width=w, height=h)
    report = {"recording": os.path.abspath(a.rec), "frames_with_detections": len(frames), "window": window,
              "correct": [run_scenario(frames, dets, make_chain, m, window, target_label=a.target_label)
                          for m in MODES],
              "wrong": [run_scenario(frames, dets, make_chain, m, window, wrong=True,
                                     target_label=a.target_label) for m in ("zero_all", "zero_region")],
              "wrong_9p9m_info": [run_scenario(frames, dets, make_chain, m, window, wrong=True, wrong_free_m=9.9,
                                               target_label=a.target_label) for m in ("zero_all", "zero_region")]}
    report["J4"] = {"correct_chain_all_pass": all(r["check"] == "PASS" for r in report["correct"]),
                    "wrong_variant_caught": all(r["check"] == "FAIL" for r in report["wrong"])}
    json.dump(report, open(os.path.join(a.out, "replay_report.json"), "w"), indent=1)
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
