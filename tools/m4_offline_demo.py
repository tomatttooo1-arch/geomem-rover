"""Reproducible offline M4 import/reload/revision demo; never runs a detector."""
from copy import deepcopy
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

from perception.offline_memory import OfflineMemory


def write(path, value):
    with path.open("x", encoding="utf8") as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write("\n")


def synthetic_document():
    """Geometry is synthetic. No true object ID is given to the algorithm."""
    source = Path(__file__).resolve()
    ref = {"path": str(source), "sha256": hashlib.sha256(source.read_bytes()).hexdigest()}
    map_row = {"map_id": "synthetic-room", "map_version": "v1", "world_frame": "synthetic_world",
               "units": "m", "pose_source": "synthetic_known_pose", "source_kind": "synthetic",
               "source_ref": ref, "calibration_id": "synthetic-no-camera"}
    poses, batches = [], []
    for i, points in enumerate([[("cup", [1, 0, 2]), ("bottle", [2, 0, 2]), ("bottle", [2.1, 0, 2])],
                                 [("cup", [1.02, 0, 2])], []]):
        key = f"synthetic-acquisition-{i}"
        if points:
            poses.append({"map_id": map_row["map_id"], "keyframe_id": key, "version": "p1",
                          "frame_id": "synthetic_local", "T_map_local": np.eye(4).tolist(),
                          "source": "synthetic_known_pose", "evidence_refs": [ref]})
        observations = []
        for index, (label, point) in enumerate(points, 1):
            observations.append({"instance_id": index, "instance_kind": "synthetic_per_frame_index",
                                 "label": label, "score": .8, "source": "synthetic_prediction_n",
                                 "geometry": {"valid": True, "reason": None, "position_local_m": point,
                                              "keyframe_id": key, "pose_version": "p1", "frame_id": "synthetic_local",
                                              "units": "m", "summary_method": "synthetic_point_summary",
                                              "valid_fraction": 1.0}, "evidence_refs": [ref]})
        batches.append({"map_id": map_row["map_id"], "map_version": "v1",
                        "acquisition": {"id": key, "source_id": "synthetic-sequence",
                                        "image_sha256": None, "timestamp": float(i),
                                        "time_source": "synthetic_clock", "clock_domain": "synthetic_clock",
                                        "calibration_id": map_row["calibration_id"]},
                        "prediction_set": "synthetic-n", "status": "ok", "error": None,
                        "observations": observations})
    return {"schema": "dev3.m4.input.v1", "maps": [map_row], "poses": poses, "batches": batches}


def run_demo(out, replay, data):
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    dirty = subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True)
    if dirty.strip():
        raise RuntimeError("freeze and commit source before the demo; output directory retained")
    commands = []

    def command(name, module, *args):
        argv = [sys.executable, "-B", "-m", module, *map(str, args)]
        result = subprocess.run(argv, cwd=root, text=True, capture_output=True,
                                env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
        (out / f"{name}.stdout.txt").write_text(result.stdout, encoding="utf8")
        (out / f"{name}.stderr.txt").write_text(result.stderr, encoding="utf8")
        commands.append({"stage": name, "argv": argv, "returncode": result.returncode})
        write(out / f"{name}.command.json", commands[-1])
        if result.returncode:
            raise RuntimeError(f"{name} failed; see retained stderr")
        return result

    def invoke(name, verb, store, *args):
        output = out / (name + ".json")
        command(name, "host.offline_memory", verb, "--store", store, *args, "--out", output)
        return json.loads(output.read_text())

    synthetic = synthetic_document()
    write(out / "synthetic-input.json", synthetic)
    store = out / "synthetic-memory.json"
    a = invoke("synthetic-A-import", "import", store, "--input", out / "synthetic-input.json")
    b = invoke("synthetic-D-reloaded-query", "query", store, "--category", "cup")
    assert a["process_id"] != b["process_id"], "query must run in a new process"
    cup = b["objects"]
    assert len(cup) == 1 and cup[0]["physical_observation_count"] == 2
    assert np.allclose(cup[0]["position_m"], [1.02, 0, 2], atol=1e-12)
    assert len(OfflineMemory.load(store).query("bottle")["objects"]) == 2
    revised_poses = deepcopy(synthetic["poses"])
    T = [[0, -1, 0, 2], [1, 0, 0, 3], [0, 0, 1, 0], [0, 0, 0, 1]]
    for pose in revised_poses:
        pose.update(version="p2", T_map_local=T, source="synthetic_absolute_pose_revision")
    update = {"schema": "dev3.m4.revision.v1", "revision_id": "synthetic-r1",
              "map_id": "synthetic-room", "from_version": "v1", "to_version": "v2",
              "reason": "synthetic 90-degree Z rotation plus translation; not a real loop closure",
              "poses": revised_poses, "unchanged_keyframes": []}
    write(out / "synthetic-revision.json", update)
    invoke("synthetic-E-revise", "revise", store, "--input", out / "synthetic-revision.json")
    c = invoke("synthetic-E-reloaded-query", "query", store, "--category", "cup")
    assert c["objects"][0]["id"] == cup[0]["id"]
    assert np.allclose(c["objects"][0]["position_m"], [2, 4.02, 2], atol=1e-12)
    before_repeat = deepcopy(OfflineMemory.load(store).data)
    duplicate = invoke("synthetic-F-repeat", "import", store, "--input", out / "synthetic-input.json")
    assert all(x["status"] == "duplicate" for x in duplicate["batches"])
    assert OfflineMemory.load(store).data == before_repeat
    invoke("synthetic-F-final-query", "query", store)

    public_path, public_store = out / "public-tum-input.json", out / "public-tum-memory.json"
    command("public-adapt", "tools.m4_legacy_import", "--replay", replay, "--data", data, "--out", public_path)
    public = json.loads(public_path.read_text())
    imported = invoke("public-A-import", "import", public_store, "--input", public_path)
    queried = invoke("public-D-reloaded-query", "query", public_store)
    assert imported["process_id"] != queried["process_id"]
    # Independently link each adapted local summary to the original saved world
    # summary using the adapter's exact event/history selector. No identity truth.
    original_events = Path(replay, "events.jsonl").read_text().splitlines()
    poses = {p["keyframe_id"]: np.array(p["T_map_local"]) for p in public["poses"]}
    residuals = []
    for batch in public["batches"]:
        for o in batch["observations"]:
            selector = o["evidence_refs"][0]["selector"].split("/")
            row = json.loads(original_events[int(selector[0].split(":")[1])-1])
            original = row["strategies"]["geometry"]["tracks"][int(selector[4])]["history"][int(selector[6])]
            g = o["geometry"]; T = poses[g["keyframe_id"]]
            restored = T[:3, :3] @ np.array(g["position_local_m"]) + T[:3, 3]
            residuals.append(float(np.max(np.abs(restored-original["position_m"]))))
    assert residuals and max(residuals) < 1e-12
    before_public = deepcopy(OfflineMemory.load(public_store).data)
    repeat = invoke("public-F-repeat", "import", public_store, "--input", public_path)
    assert all(x["status"] == "duplicate" for x in repeat["batches"])
    assert OfflineMemory.load(public_store).data == before_public
    query = invoke("public-F-keyboard-query", "query", public_store, "--category", "keyboard")
    assert query["objects"]
    summary = {"ok": True, "source_commit": revision, "python": sys.version, "commands": commands,
               "synthetic": {"object_id": cup[0]["id"], "before_m": cup[0]["position_m"],
                             "after_m": c["objects"][0]["position_m"], "physical_observations": 2,
                             "objects": len(before_repeat["objects"]), "repeat_unchanged": True,
                             "import_pid": a["process_id"], "reload_pid": b["process_id"]},
               "public_tum": {"frames": len(public["batches"]), "exported_observations": len(residuals),
                              "not_recoverable_observations": public["adapter_report"]["not_recoverable_observations"],
                              "objects": len(queried["objects"]), "pending": len(queried["pending_observations"]),
                              "max_summary_roundtrip_error_m": max(residuals), "repeat_unchanged": True,
                              "import_pid": imported["process_id"], "reload_pid": queried["process_id"]},
               "limitations": ["public geometry is inherited software-reference output, not newly validated accuracy",
                               "pose revision and identity ground-truth assertions are synthetic only",
                               "no M3 YOLO, SLAM, FPGA, robot, disappearance or online integration validation"]}
    write(out / "summary.json", summary)
    lines = ["# M4 离线查询结果", "", f"源码：`{revision}`。只读既有数据；未运行识别或设备。", "",
             "## 合成逻辑验证", "", f"杯子 {cup[0]['id']}：{cup[0]['position_m']} → {c['objects'][0]['position_m']} 米。",
             "修订含 Z 轴旋转 90° 及平移；这是合成修订，不是真实回环。两次物理观测，重复导入不增加。", "",
             "## 公开 TUM 的旧软件参考结果", "",
             f"导出 {len(residuals)} 条既有三维摘要，未能从旧历史恢复 {summary['public_tum']['not_recoverable_observations']} 条。",
             "使用动捕位姿、旧 SSDLite 结果及原标定；不是 M3 YOLO 或 RTAB-Map 结果。", "",
             "| 长期编号 | 类别 | 位置（米） | 物理观察次数 | 状态 |", "|---|---|---|---:|---|"]
    for obj in queried["objects"]:
        lines.append(f"| {obj['id']} | {obj['label']} | {obj['position_m']} | {obj['physical_observation_count']} | {obj['state']} |")
    lines.extend(["", "全部记录均为候选；次数不等于置信度，未确认真实物体身份或当前存在。",
                  "完整原观测、证据引用、关联决定和待处理项见 memory、query JSON；详细命令及进程号见 summary.json。"])
    (out / "query-results.md").write_text("\n".join(lines) + "\n", encoding="utf8")
    manifest = []
    for p in sorted(out.iterdir()):
        if p.is_file() and not p.name.endswith(".lock"):
            manifest.append(f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.name}")
    (out / "MANIFEST.sha256").write_text("\n".join(manifest) + "\n", encoding="utf8")
    print(json.dumps({k: summary[k] for k in ("ok", "source_commit", "synthetic", "public_tum")}, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--data", type=Path, required=True)
    args = parser.parse_args()
    run_demo(args.out, args.replay, args.data)


if __name__ == "__main__":
    main()
