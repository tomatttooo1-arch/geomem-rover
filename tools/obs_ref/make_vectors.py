# 从 rec30 实录（真实 RGB-D + 车上 YOLO 识别框）抽观测生成向量，并用产品 Python process_frame 出黄金。
# usage: python3 make_vectors.py REPO_ROOT REC_DIR OUT_DIR [STRIDE=5]
# 输出：OUT_DIR/obs_vectors.bin（输入）、OUT_DIR/obs_golden_py.bin（Python 答案）、OUT_DIR/vectors_index.json
# 位姿：rec30 是固定相机（位姿 = 单位阵）；所选帧中奇数位的帧改用一个合成的"车载"位姿，让旋转/平移路径也被覆盖。
import sys, json, struct, hashlib
from pathlib import Path
import numpy as np
import cv2

repo, rec, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
stride = int(sys.argv[4]) if len(sys.argv) > 4 else 5
sys.path.insert(0, str(repo))
from perception.fpga_replay.core import Config, Frame, Detection, process_frame  # noqa: E402

COLORS = ["black", "white", "gray", "red", "yellow", "green", "blue", "purple"]
REC_FMT = "<IIiiii" + "d" * 13   # frame_slot, box, ok, color, truncated, pad, pos3 lo3 hi3 frac ext2 range
assert struct.calcsize(REC_FMT) == 128


def car_pose():
    # 光学系 (x 右, y 下, z 前) -> 车体系 (x 前, y 左, z 上)，再俯仰 10°、偏航 30°，平移 (1.2, -0.4, 0.25)
    base = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
    p, yw = np.deg2rad(10.0), np.deg2rad(30.0)
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(yw), -np.sin(yw), 0], [np.sin(yw), np.cos(yw), 0], [0, 0, 1]])
    T = np.eye(4); T[:3, :3] = Rz @ Ry @ base; T[:3, 3] = [1.2, -0.4, 0.25]
    return T


calib = json.loads((rec / "calibration.json").read_text())
K = np.asarray(calib["K"], float)
frames = [json.loads(l) for l in (rec / "frames.jsonl").read_text().splitlines() if l.strip()]
dets = [json.loads(l) for l in (rec / "detections.jsonl").read_text().splitlines() if l.strip()]
assert len(frames) == len(dets)
H, W = calib["height"], calib["width"]
sel = list(range(0, len(frames), stride))
cfg = Config()

vec = bytearray(); gold = bytearray(); index = []; total = 0
for slot, fi in enumerate(sel):
    fr, dd = frames[fi], dets[fi]
    assert fr["color_index"] == dd["color_index"]
    bgr = cv2.imread(str(rec / fr["color_png"]), cv2.IMREAD_COLOR)
    dmm = cv2.imread(str(rec / fr["depth_png"]), cv2.IMREAD_UNCHANGED)
    assert bgr.shape == (H, W, 3) and dmm.shape == (H, W) and dmm.dtype == np.uint16
    rgb = np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    depth = dmm.astype(np.float32) / 1000.0; depth[depth <= 0] = np.nan   # = io.load_frame
    pose_kind = slot % 2
    T = car_pose() if pose_kind else np.eye(4)
    fid, ts = str(fr["seq"]), fr["acquired_ns"] / 1e9
    frame = Frame(fid, ts, rgb, depth, K, T, True, "rec30", calib["calibration_id"])
    boxes = dd["dets"]
    vec += struct.pack("<II", fi, pose_kind) + struct.pack("<9d", *K.ravel()) + struct.pack("<16d", *T.ravel())
    vec += struct.pack("<I", len(boxes))
    for bi, d in enumerate(boxes):
        vec += struct.pack("<4dfi", *map(float, d["box"]), float(d["score"]), int(d["cls"]))
        obs = process_frame(frame, [Detection(d["name"], float(d["score"]), tuple(map(float, d["box"])),
                                              frame_id=fid, timestamp=ts)], cfg)
        if obs:
            o = obs[0]
            vals = list(o.position) + list(o.lower) + list(o.upper) + [o.valid_fraction] + list(o.extent_m) + [o.range_m]
            gold += struct.pack(REC_FMT, slot, bi, 1, COLORS.index(o.color), int(o.truncated), 0, *map(float, vals))
        else:
            gold += struct.pack(REC_FMT, slot, bi, 0, -1, 0, 0, *([0.0] * 13))
        index.append({"slot": slot, "rec_frame": fi, "color_index": fr["color_index"], "box": bi,
                      "label": d["name"], "bbox": d["box"], "pose_kind": pose_kind, "py_obs": bool(obs)})
        total += 1
    vec += dmm.astype("<u2").tobytes() + rgb.tobytes()

out.mkdir(parents=True, exist_ok=True)
(out / "obs_vectors.bin").write_bytes(b"OBSV0001" + struct.pack("<IIII", len(sel), W, H, total) + bytes(vec))
(out / "obs_golden_py.bin").write_bytes(b"OBSG0001" + struct.pack("<I", total) + bytes(gold))
(out / "vectors_index.json").write_text(json.dumps(index, ensure_ascii=False, indent=0))
n_obs = sum(r["py_obs"] for r in index)
print("frames", len(sel), "pairs", total, "py_obs", n_obs, "numpy", np.__version__, "cv2", cv2.__version__)
for p in ("obs_vectors.bin", "obs_golden_py.bin"):
    print(p, hashlib.sha256((out / p).read_bytes()).hexdigest())
