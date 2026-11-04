"""PREREG.md section 0 (host only): is the board ONNX (the engine's source graph) equivalent to the
host ONNX that line A checked against ultralytics? Also writes the reference detections for B1-B4.

Usage (repo root on PYTHONPATH, python3.13 with onnxruntime + onnx):
  python3.13 -m tools.yolo_trt_board.onnx_equiv <host.onnx> <board.onnx> <rec dir> <out dir>

Writes <out>/onnx_equiv.json (E1/E2/K1/K2 + graph diff), <out>/ref_board_onnx_ort.json and
<out>/ref_host_onnx_ort.json (39-frame detections, all 80 classes, before any whitelist),
<out>/frames_manifest.json (frame names + sha256, used by the deploy package).
"""
import copy
import hashlib
import json
import os
import sys

import cv2
import numpy as np

from perception.fpga_replay.yolo_detector import YoloSegDetector, letterbox_input, postprocess_raw
from tools.yolo_trt_board.matching import brief, compare, judge

CONF, IOU, MAX_DET = 0.25, 0.7, 100
GATES = {"box_rows_0_3": 1e-2, "score_rows_4_83": 1e-4, "coef_rows_84_115": 1e-3, "proto": 1e-3}


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def select_frames(rec):
    names = sorted(os.listdir(os.path.join(rec, "color")))
    frames = names[0:310:8]
    assert len(names) == 310 and len(frames) == 39, (len(names), len(frames))
    return frames


def graph_diff(path_a, path_b):
    import onnx
    from onnx import numpy_helper
    a, b = onnx.load(path_a), onnx.load(path_b)
    meta = {"a": {"producer": a.producer_name + " " + a.producer_version, "ir": a.ir_version,
                  "opset": [(o.domain, o.version) for o in a.opset_import]},
            "b": {"producer": b.producer_name + " " + b.producer_version, "ir": b.ir_version,
                  "opset": [(o.domain, o.version) for o in b.opset_import]}}
    ca, cb = copy.deepcopy(a), copy.deepcopy(b)
    for m in (ca, cb):
        m.producer_version = ""; m.doc_string = ""; m.graph.doc_string = ""
        del m.metadata_props[:]
    out = {"meta": meta, "bytes_equal_without_metadata": ca.SerializeToString() == cb.SerializeToString(),
           "metadata_a": {p.key: p.value for p in a.metadata_props},
           "metadata_b": {p.key: p.value for p in b.metadata_props}}
    ops_a = [n.op_type for n in a.graph.node]; ops_b = [n.op_type for n in b.graph.node]
    out["nodes"] = [len(ops_a), len(ops_b)]
    out["op_sequence_equal"] = ops_a == ops_b
    ia = {t.name: numpy_helper.to_array(t) for t in a.graph.initializer}
    ib = {t.name: numpy_helper.to_array(t) for t in b.graph.initializer}
    out["initializers"] = [len(ia), len(ib)]
    out["initializer_names_equal"] = sorted(ia) == sorted(ib)
    if out["initializer_names_equal"]:
        shapes_ok, maxdiff = True, 0.0
        for k in ia:
            if ia[k].shape != ib[k].shape or ia[k].dtype != ib[k].dtype:
                shapes_ok = False
                continue
            if ia[k].size and np.issubdtype(ia[k].dtype, np.number):
                maxdiff = max(maxdiff, float(np.max(np.abs(ia[k].astype(np.float64) - ib[k].astype(np.float64)))))
        out["initializer_shapes_equal"] = shapes_ok
        out["initializer_max_abs_diff"] = maxdiff
    else:
        out["initializer_names_only_a"] = sorted(set(ia) - set(ib))[:20]
        out["initializer_names_only_b"] = sorted(set(ib) - set(ia))[:20]
    return out


class Runner:
    """A YoloSegDetector(ort) plus direct access to both raw outputs on the same session."""
    def __init__(self, path):
        self.det = YoloSegDetector(path, backend="ort", conf=CONF, iou=IOU, classes=None, max_det=MAX_DET)
        outs = sorted(self.det.session.get_outputs(), key=lambda o: len(o.shape))
        self.names = [o.name for o in outs]                     # (1,116,N), (1,32,ph,pw)

    def run(self, x):
        return self.det.session.run(self.names, {self.det.input_name: x})


def group_diffs(oa, ob):
    d0 = np.abs(oa[0].astype(np.float64) - ob[0].astype(np.float64))[0]   # (116, N)
    dp = np.abs(oa[1].astype(np.float64) - ob[1].astype(np.float64))
    return {"box_rows_0_3": float(d0[0:4].max()), "score_rows_4_83": float(d0[4:84].max()),
            "coef_rows_84_115": float(d0[84:116].max()), "proto": float(dp.max())}


def merge_max(acc, d):
    for k, v in d.items():
        acc[k] = max(acc.get(k, 0.0), v)
    return acc


def to_simple(dets):
    return [{"name": d["name"], "score": d["score"], "box": list(d["box"])} for d in dets]


def main():
    host_onnx, board_onnx, rec, out_dir = sys.argv[1:5]
    os.makedirs(out_dir, exist_ok=True)
    frames = select_frames(rec)
    manifest = {"rec": os.path.abspath(rec), "rule": "sorted(color/*.png)[0:310:8]", "frames": frames,
                "sha256": {n: sha256_file(os.path.join(rec, "color", n)) for n in frames}}
    host, host2, board = Runner(host_onnx), Runner(host_onnx), Runner(board_onnx)
    diff_main, diff_k1, diff_k2 = {}, {}, {}
    det_h, det_h2, det_b = {}, {}, {}
    for name in frames:
        bgr = cv2.imread(os.path.join(rec, "color", name), cv2.IMREAD_COLOR)
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        lb = host.det._letterbox(rgb.shape[1], rgb.shape[0])
        x = letterbox_input(rgb, lb, cv2)
        oh, oh2, ob = host.run(x), host2.run(x), board.run(x)
        ob_shift = board.run(np.ascontiguousarray(np.roll(x, 1, axis=3)))
        merge_max(diff_main, group_diffs(oh, ob))
        merge_max(diff_k1, group_diffs(oh, oh2))
        merge_max(diff_k2, group_diffs(oh, ob_shift))
        det_h[name] = to_simple(postprocess_raw(oh[0], lb, CONF, IOU, MAX_DET, host.det.names))
        det_h2[name] = to_simple(postprocess_raw(oh2[0], lb, CONF, IOU, MAX_DET, host.det.names))
        det_b[name] = to_simple(postprocess_raw(ob[0], lb, CONF, IOU, MAX_DET, host.det.names))
    # the detector's own call path gives the same detections as the raw-output path used above
    probe = frames[0]
    bgr = cv2.imread(os.path.join(rec, "color", probe), cv2.IMREAD_COLOR)
    own = to_simple(board.det.raw_detections(np.ascontiguousarray(bgr[:, :, ::-1])))
    own_path_same = own == det_b[probe]

    def e1(d):
        return {k: {"max_abs": v, "gate": GATES[k], "ok": v <= GATES[k]} for k, v in d.items()}

    def e2(ref, test):
        per = compare(frames, ref, test)
        j = judge(per, score_gate=1e-3, iou_gate=0.99)
        ok = (j["pairs"] == j["ref_detections"] == j["test_detections"] and not j["unmatched_boundary"]
              and not j["unmatched_hard"] and not j["count_mismatch_frames"]
              and j["B2_iou_ge_gate"] and j["B3_dscore_le_gate"])
        return dict(brief(j), E2_ok=ok)

    res = {"prereg": "PREREG.md section 0", "host_onnx": host_onnx, "board_onnx": board_onnx,
           "host_onnx_sha256": sha256_file(host_onnx), "board_onnx_sha256": sha256_file(board_onnx),
           "frames": len(frames), "execution_host": host.det.execution, "execution_board": board.det.execution,
           "graph": graph_diff(host_onnx, board_onnx)}
    res["E1_main"] = e1(diff_main)
    res["E1_ok"] = all(v["ok"] for v in res["E1_main"].values())
    res["E2_main"] = e2(det_h, det_b)
    res["K1_E1"] = e1(diff_k1)
    res["K1_E2"] = e2(det_h, det_h2)
    res["K1_ok"] = all(v["max_abs"] <= 1e-6 for v in res["K1_E1"].values()) and res["K1_E2"]["E2_ok"]
    res["K2_E1"] = e1(diff_k2)
    res["K2_ok"] = not all(v["ok"] for v in res["K2_E1"].values())
    res["detector_call_path_same_as_raw_path"] = own_path_same
    res["instrument_ok"] = res["K1_ok"] and res["K2_ok"] and own_path_same
    res["EQUIVALENT"] = res["instrument_ok"] and res["E1_ok"] and res["E2_main"]["E2_ok"]
    with open(os.path.join(out_dir, "onnx_equiv.json"), "w") as f:
        json.dump(res, f, indent=1)
    for tag, dets, runner in (("ref_board_onnx_ort", det_b, board), ("ref_host_onnx_ort", det_h, host)):
        with open(os.path.join(out_dir, tag + ".json"), "w") as f:
            json.dump({"what": "host onnxruntime FP32 detections, all 80 classes, conf 0.25 iou 0.7 max_det 100",
                       "execution": runner.det.execution, "frames": frames, "frame_sha256": manifest["sha256"],
                       "detections": dets}, f, indent=1)
    with open(os.path.join(out_dir, "frames_manifest.json"), "w") as f:
        json.dump(manifest, f, indent=1)
    print(json.dumps({k: v for k, v in res.items() if k not in ("execution_host", "execution_board")}, indent=1))


if __name__ == "__main__":
    main()
