#!/usr/bin/env python3
"""识别准确率测试（10-27，车静止；在 detect_range.py 的摆法上扩展）：九类前五类＋对照类、按距离和角度、空地面误检、
存失败原图，出表并生成给 dev3 的反馈报告。车全程不动：本工具只读运行记录、只订阅预览图，不发任何指令。

    # 车上（Nano，比赛栈已起、车不动）：跟着提示摆，人退出画面后按回车；每轮自动存该时段的预览原图
    python3 tools/car_run/detect_accuracy.py mark --out ~/acc_20261027/marks.jsonl
    # 出表＋失败样例图（车上或宿主；画框需要 cv2，没有就只出表）
    python3 tools/car_run/detect_accuracy.py report RUN/runtime.jsonl ~/acc_20261027/marks.jsonl \
        --frames RUN/acc_frames --out ~/acc_20261027/report
    python3 tools/car_run/detect_accuracy.py feedback ~/acc_20261027/report

摆法（owner 10-26 要求少摆）：从镜头正下方地面往正前方量，在 0.6/0.9/1.2/1.5/2.0 m 各贴一条横线（沿用 detect_range
的线）。9 样东西：遥控器、药瓶、药盒、钥匙、眼镜（前五类）＋手机、饮料罐、普通瓶子、水杯（对照）。
每条线上最多两样：一样放「中」（画面正中），一样放「左边」或「右边」（预览画面里离边缘约一拳）。
每轮每样往后挪一条线（最远的挪回 0.6 m），并在中/边之间轮换；默认 5 轮（每样每个距离各测一次，中/边各覆盖到），
--full 为 10 轮（每样每个距离中、边都测）。另加 1 轮空地面（--empty-rounds）：画面里只放桌脚、椅轮、地面杂物，
不放 9 样东西，统计误检。整套默认 6 次回车。

归属：同一轮里不同东西可能被报成同名类别（例如药瓶和普通瓶子），所以不按类别、按位置归属：画面横向分左/中/右三
栏，纵向按框底边与各线在画面里的预期位置（相机高 0.39 m、俯 7°、针孔近似）就近匹配。没匹配上任何摆放物的框
记为误检。同一东西被两个类别同时报（如 bottle 和 medicine bottle，框 IoU>=0.5）单独计数。
mark 用本机 time.time() 打点：必须在 Nano 上跑（与 runtime.jsonl 的 acquisition_timestamp 同一时钟）。
"""
import argparse
import glob
import json
import math
import os
import statistics
import subprocess
import sys
import time

# 物体 key: (中文名, 期望类别, 是否前五类)
OBJECTS = {
    "remote": ("遥控器", "remote", True),
    "medicine_bottle": ("药瓶", "medicine bottle", True),
    "medicine_box": ("药盒", "medicine box", True),
    "key": ("钥匙", "key", True),
    "glasses": ("眼镜", "glasses", True),
    "phone": ("手机", "cell phone", False),
    "drink_can": ("饮料罐", "drink can", False),
    "bottle": ("普通瓶子", "bottle", False),
    "cup": ("水杯", "cup", False),
}
ORDER = ("remote", "medicine_bottle", "medicine_box", "key", "glasses", "phone", "drink_can", "bottle", "cup")
DISTANCES = (0.6, 0.9, 1.2, 1.5, 2.0)
SLOT_ZH = {"center": "中", "left": "左边", "right": "右边"}


def default_plan(objects=ORDER, full=False, empty_rounds=1):
    """第 k 轮第 i 样在第 (i % 5 + k) % 5 条线；前 5 样与后 4 样同线时一中一边，中/边按轮换。"""
    n = len(DISTANCES)
    steps = []
    rounds = 2 * n if full else n
    for k in range(rounds):
        items = []
        for i, o in enumerate(objects):
            line = (i % 5 + k) % n
            # 每条线：前五类一样＋对照一样；even 时前五类居中、对照放右边，否则前五类放左边、对照居中。
            # --full 的后 5 轮把前 5 轮的中/边对调，凑齐每样每个距离中、边各一次。
            kk = k - n if k >= n else k
            even = (kk < 3) != (line % 2 == 0)      # 5 轮里每样东西中、边都轮到
            if full and k >= n:
                even = not even
            if i < 5:
                slot = "center" if even else "left"
            else:
                slot = "right" if even else "center"
            items.append({"object": o, "distance_m": DISTANCES[line], "slot": slot})
        steps.append({"round": k + 1, "kind": "objects",
                      "items": sorted(items, key=lambda it: (it["distance_m"], it["slot"]))})
    for e in range(empty_rounds):
        steps.append({"round": rounds + e + 1, "kind": "empty", "items": []})
    return steps


def describe(step):
    if step["kind"] == "empty":
        return "第 %d 轮（空地面误检）：拿走 9 样东西，只在 0.6-2 m 范围放桌脚 / 椅子轮 / 地面杂物（纸屑、线、鞋）" % step["round"]
    lines = []
    for d in DISTANCES:
        here = [it for it in step["items"] if it["distance_m"] == d]
        if here:
            lines.append("    %.1f m 线：%s" % (d, "，".join("%s放%s" % (SLOT_ZH[it["slot"]], OBJECTS[it["object"]][0])
                                                       for it in here)))
    return "第 %d 轮：\n%s" % (step["round"], "\n".join(lines))


def parse_objects(text):
    objs = tuple(o.strip() for o in text.split(",") if o.strip())
    bad = [o for o in objs if o not in OBJECTS]
    if bad or not objs:
        raise SystemExit("--objects 只能取 %s（逗号分隔）" % ",".join(OBJECTS))
    return objs


def current_run():
    cur = "/data/fpga/car/state/car.current"
    if not os.path.exists(cur):
        return None, None
    run_dir = os.path.realpath(cur)
    return run_dir, "wde-car-stack-" + os.path.basename(run_dir)


def grab(container, rnd, seconds):
    """在车栈容器里存这段时间的预览原图（/o = 本趟运行目录）。失败只告警，不影响打点。"""
    if not container:
        return None
    out = "/o/acc_frames/r%02d" % rnd
    cmd = ["docker", "exec", container, "bash", "-c",
           "source /opt/ros/humble/setup.bash >/dev/null 2>&1; python3 /src/tools/car_run/preview_grab.py "
           "--out %s --seconds %.1f" % (out, seconds)]
    try:
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    except OSError as e:
        print("  [告警] 存图没起来：%s（本轮照常打点）" % e)
        return None


def cmd_mark(a):
    steps = default_plan(parse_objects(a.objects), a.full, a.empty_rounds)
    run_dir, container = current_run()
    if a.no_frames:
        container = None
    print("本趟：%s；存图：%s" % (run_dir or "（没找到 car.current）",
                              ("%s/acc_frames" % run_dir) if container else "关"))
    os.makedirs(os.path.dirname(os.path.abspath(a.out)) or ".", exist_ok=True)
    i = 0
    with open(a.out, "a", encoding="utf8") as out:
        print("共 %d 轮；每轮摆好、人退出画面后按回车，%.0f s 后开始记 %.0f s。输入 b 回上一轮重摆、q 退出。"
              % (len(steps), a.settle_s, a.hold_s))
        while i < len(steps):
            step = steps[i]
            cmd = input("\n[%d/%d] %s\n  放好、人退出画面后按回车 > " % (i + 1, len(steps), describe(step))).strip().lower()
            if cmd == "q":
                break
            if cmd == "b":
                i = max(0, i - 1)
                continue
            time.sleep(a.settle_s)
            t0 = time.time()
            proc = grab(container, step["round"], a.hold_s + 0.5)
            for left in range(int(a.hold_s), 0, -1):
                sys.stdout.write("\r  记录中，还剩 %2d s（别进画面）" % left)
                sys.stdout.flush()
                time.sleep(1)
            row = dict(step, t_start=t0, t_end=t0 + a.hold_s, run_dir=run_dir)
            if proc is not None:
                try:
                    msg = proc.communicate(timeout=15)[0].strip().splitlines()
                    row["frames_note"] = msg[-1] if msg else ""
                except subprocess.TimeoutExpired:
                    proc.kill()
                    row["frames_note"] = "grab timeout"
            sys.stdout.write("\r  本轮完成。%s\a%s\n" % (row.get("frames_note", ""), " " * 10))
            out.write(json.dumps(row, ensure_ascii=False) + "\n")
            out.flush()
            i += 1
    print("打点写在 %s。出表：python3 tools/car_run/detect_accuracy.py report %s/runtime.jsonl %s --frames %s/acc_frames --out <目录>"
          % (a.out, run_dir or "RUN", a.out, run_dir or "RUN"))


# ---------------------------------------------------------------- analysis
def expected_bottom_y(d, height_m, pitch_deg, fy, cy):
    """地面上距离 d 的点在画面中的纵坐标（针孔近似）。"""
    ang = math.atan2(height_m, d) - math.radians(pitch_deg)
    return cy + fy * math.tan(ang)


def column(bbox, width):
    cx = (bbox[0] + bbox[2]) / 2.0
    return "left" if cx < width / 3.0 else ("right" if cx > 2 * width / 3.0 else "center")


def iou(a, b):
    w = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    h = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    i = w * h
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i
    return i / u if u > 0 else 0.0


def load_frames(path):
    frames = []
    with open(path, encoding="utf8") as f:
        for line in f:
            if '"frame_processed"' not in line:
                continue
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get("event") != "frame_processed":
                continue
            t = r.get("acquisition_timestamp", r.get("time"))
            if isinstance(t, (int, float)):
                frames.append({"t": float(t), "frame_id": r.get("frame_id"), "dets": r.get("detections") or [],
                               "tracks": r.get("tracks") or []})
    frames.sort(key=lambda x: x["t"])
    return frames


def load_marks(path):
    last = {}
    with open(path, encoding="utf8") as f:
        for line in f:
            if line.strip():
                r = json.loads(line)
                last[r["round"]] = r          # 同一轮重摆以最后一次为准
    return [last[k] for k in sorted(last)]


def assign(dets, items, a):
    """按位置把框归给摆放物：同栏内按框底边与预期线位置就近（容差 a.match_px）；返回 {obj: [det]}, 误检列表。"""
    exp = {it["object"]: expected_bottom_y(it["distance_m"], a.cam_height, a.cam_pitch, a.fy, a.cy) for it in items}
    got = {it["object"]: [] for it in items}
    fps = []
    for d in dets:
        b = [float(v) for v in d["bbox"]]
        col = column(b, a.width)
        cands = [it for it in items if it["slot"] == col]
        best, err = None, None
        for it in cands:
            e = abs(b[3] - exp[it["object"]])
            if err is None or e < err:
                best, err = it, e
        if best is not None and err <= a.match_px:
            got[best["object"]].append(d)
        else:
            fps.append(d)
    return got, fps


def analyse(frames, marks, a):
    rows, empty, fp_examples, miss_examples, dual = [], [], [], [], []
    for m in marks:
        win = [f for f in frames if m["t_start"] <= f["t"] <= m["t_end"]]
        if m.get("kind") == "empty":
            labels = {}
            fp_frames = 0
            for f in win:
                if f["dets"]:
                    fp_frames += 1
                    fp_examples.append({"round": m["round"], "frame": f, "dets": f["dets"], "why": "空地面误检"})
                for d in f["dets"]:
                    labels[d["label"]] = labels.get(d["label"], 0) + 1
            empty.append({"round": m["round"], "frames": len(win), "fp_frames": fp_frames,
                          "fp_rate": round(fp_frames / len(win), 3) if win else None, "labels": labels})
            continue
        stats = {it["object"]: {"hits": 0, "correct": 0, "scores": [], "w": [], "h": [], "labels": {}}
                 for it in m["items"]}
        fp_count, fp_labels = 0, {}
        for f in win:
            got, fps = assign(f["dets"], m["items"], a)
            for d in fps:
                fp_count += 1
                fp_labels[d["label"]] = fp_labels.get(d["label"], 0) + 1
            if fps:
                fp_examples.append({"round": m["round"], "frame": f, "dets": f["dets"], "why": "未归属的框"})
            for it in m["items"]:
                o, mine = it["object"], got[it["object"]]
                s = stats[o]
                want = OBJECTS[o][1]
                if mine:
                    s["hits"] += 1
                    best = max(mine, key=lambda d: d["score"])
                    s["w"].append(best["bbox"][2] - best["bbox"][0])
                    s["h"].append(best["bbox"][3] - best["bbox"][1])
                    for d in mine:
                        s["labels"][d["label"]] = s["labels"].get(d["label"], 0) + 1
                    right = [d for d in mine if d["label"] == want]
                    if right:
                        s["correct"] += 1
                        s["scores"].append(max(d["score"] for d in right))
                    labs = {d["label"] for d in mine}
                    for x in mine:
                        for y in mine:
                            if x["label"] < y["label"] and iou(x["bbox"], y["bbox"]) >= 0.5:
                                dual.append({"round": m["round"], "object": o, "pair": [x["label"], y["label"]],
                                             "frame_id": f["frame_id"]})
                    if not right:
                        miss_examples.append({"round": m["round"], "frame": f, "dets": f["dets"], "object": o,
                                              "why": "%s 被认成 %s" % (OBJECTS[o][0], "/".join(sorted(labs)))})
                else:
                    miss_examples.append({"round": m["round"], "frame": f, "dets": f["dets"], "object": o,
                                          "why": "%s 漏检" % OBJECTS[o][0]})
        for it in m["items"]:
            o, s = it["object"], stats[it["object"]]
            want = OBJECTS[o][1]
            remembered = any(t.get("label") == want and t.get("state") == "current_confirmed"
                             and isinstance(t.get("last_seen"), (int, float))
                             and m["t_start"] - 1.0 <= t["last_seen"] <= m["t_end"] + 1.0
                             for f in win for t in f["tracks"])
            rows.append({
                "round": m["round"], "object": o, "name": OBJECTS[o][0], "expect": want, "priority": OBJECTS[o][2],
                "distance_m": it["distance_m"], "slot": it["slot"], "frames": len(win),
                "rate_any": round(s["hits"] / len(win), 3) if win else None,
                "rate_correct": round(s["correct"] / len(win), 3) if win else None,
                "score_mean": round(statistics.mean(s["scores"]), 3) if s["scores"] else None,
                "box_w_px": round(statistics.median(s["w"]), 1) if s["w"] else None,
                "box_h_px": round(statistics.median(s["h"]), 1) if s["h"] else None,
                "labels": s["labels"], "remembered": remembered})
        rows.append({"round": m["round"], "object": "_fp", "fp_boxes": fp_count, "fp_labels": fp_labels,
                     "frames": len(win)})
    return rows, empty, fp_examples, miss_examples, dual


def nearest_jpg(frames_dir, rnd, t):
    if not frames_dir:
        return None
    files = glob.glob(os.path.join(frames_dir, "r%02d" % rnd, "rgb-*.jpg"))
    best, err = None, None
    for p in files:
        try:
            ns = int(os.path.basename(p)[4:-4])
        except ValueError:
            continue
        e = abs(ns / 1e9 - t)
        if err is None or e < err:
            best, err = p, e
    return best if err is not None and err <= 0.08 else None


def draw(examples, frames_dir, out_dir, a, limit, tag):
    try:
        import cv2
    except ImportError:
        return []
    os.makedirs(out_dir, exist_ok=True)
    done, per_key = [], {}
    for ex in examples:
        key = (ex["round"], ex.get("object"), ex["why"])
        if per_key.get(key, 0) >= limit:
            continue
        p = nearest_jpg(frames_dir, ex["round"], ex["frame"]["t"])
        if not p:
            continue
        im = cv2.imread(p)
        if im is None:
            continue
        sx, sy = im.shape[1] / float(a.width), im.shape[0] / float(a.height)
        for d in ex["dets"]:
            b = d["bbox"]
            p1, p2 = (int(b[0] * sx), int(b[1] * sy)), (int(b[2] * sx), int(b[3] * sy))
            cv2.rectangle(im, p1, p2, (0, 0, 255), 2)
            cv2.putText(im, "%s %.2f" % (d["label"], d["score"]), (p1[0], max(12, p1[1] - 4)), 0, 0.5, (0, 0, 255), 2)
        for d in DISTANCES:
            y = int(expected_bottom_y(d, a.cam_height, a.cam_pitch, a.fy, a.cy) * sy)
            cv2.line(im, (0, y), (im.shape[1], y), (0, 200, 0), 1)
            cv2.putText(im, "%.1fm" % d, (2, y - 2), 0, 0.4, (0, 200, 0), 1)
        raw = os.path.join(out_dir, "%s_r%02d_%s_raw.jpg" % (tag, ex["round"], os.path.basename(p)[:-4]))
        anno = raw.replace("_raw.jpg", "_box.jpg")
        if not os.path.exists(raw):
            with open(p, "rb") as src, open(raw, "wb") as dst:
                dst.write(src.read())
        cv2.imwrite(anno, im)
        per_key[key] = per_key.get(key, 0) + 1
        done.append({"round": ex["round"], "object": ex.get("object"), "why": ex["why"],
                     "raw": os.path.basename(raw), "box": os.path.basename(anno),
                     "labels": [d["label"] for d in ex["dets"]]})
    return done


def cmd_report(a):
    frames = load_frames(a.runtime)
    marks = load_marks(a.marks)
    rows, empty, fpx, missx, dual = analyse(frames, marks, a)
    os.makedirs(a.out, exist_ok=True)
    fails = draw(missx, a.frames, os.path.join(a.out, "failures"), a, a.per_case, "miss")
    fails += draw(fpx, a.frames, os.path.join(a.out, "failures"), a, a.per_case, "fp")
    result = {"rows": rows, "empty": empty, "dual_label_same_object": dual, "failure_images": fails,
              "params": {k: getattr(a, k) for k in ("width", "height", "fy", "cy", "cam_height", "cam_pitch", "match_px")},
              "runtime": os.path.abspath(a.runtime), "marks": os.path.abspath(a.marks)}
    with open(os.path.join(a.out, "accuracy.json"), "w", encoding="utf8") as f:
        json.dump(result, f, ensure_ascii=False, indent=1)
    print("轮 距离 位置 物体       帧数 检出(任意类) 认对 均分  框宽x高   记忆 实际报成")
    for r in sorted((r for r in rows if r["object"] != "_fp"), key=lambda r: (ORDER.index(r["object"]), r["distance_m"])):
        box = "%sx%s" % (r["box_w_px"], r["box_h_px"]) if r["box_w_px"] is not None else "-"
        print("%-2s %-4.1f %-4s %-8s %4d %8s %6s %5s %10s %3s %s" % (
            r["round"], r["distance_m"], SLOT_ZH[r["slot"]], r["name"], r["frames"],
            "-" if r["rate_any"] is None else "%.0f%%" % (100 * r["rate_any"]),
            "-" if r["rate_correct"] is None else "%.0f%%" % (100 * r["rate_correct"]),
            "-" if r["score_mean"] is None else r["score_mean"], box, "是" if r["remembered"] else "否",
            ", ".join("%s×%d" % kv for kv in sorted(r["labels"].items(), key=lambda kv: -kv[1])) or "-"))
    for e in empty:
        print("空地面 第 %d 轮：%d 帧里 %d 帧有框（%s）；类别 %s" % (
            e["round"], e["frames"], e["fp_frames"], "-" if e["fp_rate"] is None else "%.0f%%" % (100 * e["fp_rate"]),
            e["labels"] or "-"))
    print("同一物体被两个类别同时报：%d 帧次（其中 bottle+medicine bottle %d）" % (
        len(dual), sum(1 for d in dual if d["pair"] == ["bottle", "medicine bottle"])))
    print("失败样例图 %d 张（原图＋画框）：%s" % (len(fails), os.path.join(a.out, "failures")))
    print("结果：%s" % os.path.join(a.out, "accuracy.json"))


def cmd_feedback(a):
    res = json.load(open(os.path.join(a.report, "accuracy.json"), encoding="utf8"))
    rows = [r for r in res["rows"] if r["object"] != "_fp"]
    L = ["# 识别准确率反馈（给 dev3）", "",
         "来源：车载 Nano 静止实测（相机离地 0.39 m、俯 7°，640×480，地面摆放）。运行记录 `%s`。" % res["runtime"],
         "口径：每轮约 %s 帧；「检出」= 该位置有任意类别的框，「认对」= 框的类别正确；分数为认对帧的最高分均值。" %
         (rows[0]["frames"] if rows else "-"), "",
         "## 按类别 × 距离（认对率 / 均分 / 位置）", "",
         "| 物体 | 期望类别 | " + " | ".join("%.1f m" % d for d in DISTANCES) + " | 记忆入库 |",
         "|---|---|" + "---|" * len(DISTANCES) + "---|"]
    for o in ORDER:
        mine = [r for r in rows if r["object"] == o]
        if not mine:
            continue
        cells = []
        for d in DISTANCES:
            here = [r for r in mine if r["distance_m"] == d]
            cells.append("；".join("%s%.0f%% %s" % (SLOT_ZH[r["slot"]], 100 * (r["rate_correct"] or 0),
                                                  "" if r["score_mean"] is None else "(%.2f)" % r["score_mean"])
                                  for r in here) or "-")
        L.append("| %s | %s | %s | %s |" % (OBJECTS[o][0], OBJECTS[o][1], " | ".join(cells),
                                           "%d/%d 轮" % (sum(r["remembered"] for r in mine), len(mine))))
    L += ["", "## 认错类别", ""]
    wrong = [(r, l, n) for r in rows for l, n in r["labels"].items() if l != r["expect"]]
    if wrong:
        for r, l, n in sorted(wrong, key=lambda x: -x[2]):
            L.append("- %s（%.1f m，%s）被报成 `%s`：%d 帧 / %d" % (r["name"], r["distance_m"], SLOT_ZH[r["slot"]], l, n, r["frames"]))
    else:
        L.append("- 无")
    dual = res["dual_label_same_object"]
    L += ["", "## 同一物体同时报两个类别（影响物体记忆：会记成两条）", "",
          "- 共 %d 帧次；bottle + medicine bottle：%d 帧次。" % (
              len(dual), sum(1 for d in dual if d["pair"] == ["bottle", "medicine bottle"]))]
    pairs = {}
    for d in dual:
        k = "%s（%s）" % (OBJECTS[d["object"]][0], " + ".join(d["pair"]))
        pairs[k] = pairs.get(k, 0) + 1
    L += ["- %s：%d 帧次" % kv for kv in sorted(pairs.items(), key=lambda kv: -kv[1])]
    L += ["", "## 空地面误检", ""]
    for e in res["empty"]:
        L.append("- 第 %d 轮：%d 帧中 %d 帧有框（%s），类别 %s" % (
            e["round"], e["frames"], e["fp_frames"], "-" if e["fp_rate"] is None else "%.0f%%" % (100 * e["fp_rate"]),
            ", ".join("%s×%d" % kv for kv in e["labels"].items()) or "无"))
    fpr = [r for r in res["rows"] if r["object"] == "_fp" and r["fp_boxes"]]
    for r in fpr:
        L.append("- 摆放轮 %d：未归属到任何摆放物的框 %d 个（%s）" % (
            r["round"], r["fp_boxes"], ", ".join("%s×%d" % kv for kv in r["fp_labels"].items())))
    L += ["", "## 失败样例（原图 + 画框，目录 failures/）", ""]
    for f in res["failure_images"][:60]:
        L.append("- 第 %d 轮 %s：`%s`（原图 `%s`），画面里的框：%s" % (f["round"], f["why"], f["box"], f["raw"],
                                                          ", ".join(f["labels"]) or "无"))
    if not res["failure_images"]:
        L.append("- 没有导出的图（没有失败，或测试时没开存图）")
    L += ["", "## 建议补采的数据（低机位 0.39 m、物体在地面）", ""]
    sug = 0
    for o in ORDER:
        bad = [r for r in rows if r["object"] == o and (r["rate_correct"] or 0) < a.rate]
        if not bad:
            continue
        sug += 1
        where = "、".join("%.1f m %s" % (r["distance_m"], SLOT_ZH[r["slot"]]) for r in sorted(bad, key=lambda r: r["distance_m"]))
        wrongs = sorted({l for r in bad for l in r["labels"] if l != r["expect"]})
        L.append("- **%s**（%s）：认对率 < %.0f%% 的位置：%s。%s建议在这些距离/画面位置、地面上（地毯/瓷砖/木地板各一）、"
                 "0.39 m 高度拍摄，平放/侧放各取，每个位置 ≥20 张%s。" % (
                     OBJECTS[o][0], OBJECTS[o][1], 100 * a.rate, where,
                     ("常被认成 %s；" % "、".join(wrongs)) if wrongs else "多为漏检；",
                     "，并加入易混的负样本" if wrongs else ""))
    if not sug:
        L.append("- 本次各类各位置认对率都 ≥ %.0f%%，暂无针对性补采建议。" % (100 * a.rate))
    if any(e["fp_frames"] for e in res["empty"]):
        L.append("- 空地面有误检：把失败样例中的桌脚、椅轮、杂物画面作背景负样本（不标框）加入训练。")
    L += ["", "注：数字只代表这一次、这一场地、这些实物；不是独立测试集上的精度。"]
    path = os.path.join(a.report, "FEEDBACK_dev3.md")
    with open(path, "w", encoding="utf8") as f:
        f.write("\n".join(L) + "\n")
    print("反馈报告：%s" % path)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd")
    m = sub.add_parser("mark")
    m.add_argument("--out", required=True)
    m.add_argument("--objects", default=",".join(ORDER))
    m.add_argument("--full", action="store_true", help="10 轮：每样每个距离中、边都测")
    m.add_argument("--empty-rounds", type=int, default=1)
    m.add_argument("--settle-s", type=float, default=2.0)
    m.add_argument("--hold-s", type=float, default=8.0)
    m.add_argument("--no-frames", action="store_true", help="不存预览原图")
    r = sub.add_parser("report")
    r.add_argument("runtime")
    r.add_argument("marks")
    r.add_argument("--frames", help="预览原图目录（RUN/acc_frames）")
    r.add_argument("--out", required=True)
    r.add_argument("--width", type=int, default=640)
    r.add_argument("--height", type=int, default=480)
    r.add_argument("--fy", type=float, default=450.0, help="彩色相机 fy（像素），默认按 Gemini 2 640x480 估")
    r.add_argument("--cy", type=float, default=240.0)
    r.add_argument("--cam-height", type=float, default=0.39)
    r.add_argument("--cam-pitch", type=float, default=7.0)
    r.add_argument("--match-px", type=float, default=45.0, help="框底边与预期线位置的最大偏差（像素）")
    r.add_argument("--per-case", type=int, default=2, help="每种失败每轮最多导出几张")
    fb = sub.add_parser("feedback")
    fb.add_argument("report")
    fb.add_argument("--rate", type=float, default=0.6)
    a = p.parse_args()
    if a.cmd == "mark":
        cmd_mark(a)
    elif a.cmd == "report":
        cmd_report(a)
    elif a.cmd == "feedback":
        cmd_feedback(a)
    else:
        p.print_help()


if __name__ == "__main__":
    main()
