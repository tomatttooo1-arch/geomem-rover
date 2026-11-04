#!/usr/bin/env python3
"""小物体识别距离（10-26，验收 1，简化版）：车停着不动、比赛链照常跑，摆 5 轮，事后读 runtime.jsonl 出表。

    # 车上（Nano，比赛链已起、车不动）：跟着提示摆，摆好人退出画面后按回车，每轮自动计时
    python3 tools/car_run/detect_range.py mark --out ~/range_marks_20261026.jsonl
    # 出表（车上或宿主都行）
    python3 tools/car_run/detect_range.py report RUN/runtime.jsonl ~/range_marks_20261026.jsonl [--json OUT.json]

摆法（owner 10-26：减轻摆放工作量）：从相机镜头正下方的地面往车正前方量，在 0.6 / 0.9 / 1.2 / 1.5 / 2.0 m 各贴
一条横线。手机（平放）、遥控器、小药瓶、水杯 4 样一次全摆上，每样占一条线，左右错开别互相挡；每轮每样往后挪
一条线（最远那条的挪回 0.6 m），5 轮后每样东西每个距离都测过，一共按 5 次回车。
4 样类别各不相同，按类别归属检测，不分画面左右；检出时按框中心记偏离画面中心的角度（看是不是在边上）。
mark 用本机 time.time() 打点：必须在 Nano 上跑（与 runtime.jsonl 的 acquisition_timestamp 同一时钟）。

report：每轮时间窗 [按回车 + settle, + hold] 内的 frame_processed 帧；每样东西给帧数、检出帧数、检出率、
平均 / 最高分、框宽高中位数（像素）、框中心偏角中位数、记忆有没有记下，以及窗口里出现的别的类别（误认 / 背景）。
末尾按物体给检出率 >= --rate 且更近各档也达标的最远一档距离。只用标准库，py3.8 可跑，只读运行记录。
"""
import argparse
import json
import math
import statistics
import sys
import time

# 物体：(中文名, 识别类别)
OBJECTS = {
    "phone_flat": ("手机平放", "cell phone"),
    "remote": ("遥控器", "remote"),
    "bottle": ("小药瓶", "bottle"),
    "cup": ("水杯", "cup"),
}
DISTANCES = (0.6, 0.9, 1.2, 1.5, 2.0)
ROUND_OBJECTS = ("phone_flat", "remote", "bottle", "cup")


def default_plan(objects=ROUND_OBJECTS):
    """5 轮：第 k 轮第 i 样东西在第 (i + k) mod 5 条线（每轮空一条线）。objects 可只取其中几样（10-26：遥控器、
    药瓶等新模型训好再测，先测手机和水杯；同一工具测新旧模型可直接对比）。"""
    steps = []
    n = len(DISTANCES)
    for k in range(n):
        items = [{"object": o, "distance_m": DISTANCES[(i + k) % n]} for i, o in enumerate(objects)]
        steps.append({"round": k + 1, "items": sorted(items, key=lambda it: it["distance_m"])})
    return steps


def describe(step):
    return "第 %d 轮：%s" % (step["round"], "、".join(
        "%.1f m 线放%s" % (it["distance_m"], OBJECTS[it["object"]][0]) for it in step["items"]))


def parse_objects(text):
    objs = tuple(o.strip() for o in text.split(",") if o.strip())
    bad = [o for o in objs if o not in OBJECTS]
    if bad or not objs:
        raise SystemExit("--objects 只能取 %s（逗号分隔）" % ",".join(OBJECTS))
    return objs


def cmd_mark(a):
    steps = default_plan(parse_objects(a.objects))
    i = 0
    with open(a.out, "a", encoding="utf8") as out:
        print("共 %d 轮；每轮摆好、人退出画面后按回车，%.0f s 后开始记 %.0f s。输入 b 回上一轮重摆、q 退出。"
              % (len(steps), a.settle_s, a.hold_s))
        while i < len(steps):
            step = steps[i]
            hint = "（每样往后挪一条线，最远的挪回 0.6 m）" if i else "（左右错开，别让前面的挡住后面的）"
            cmd = input("\n[%d/%d] %s\n  %s 放好按回车 > " % (i + 1, len(steps), describe(step), hint)).strip().lower()
            if cmd == "q":
                break
            if cmd == "b":
                i = max(0, i - 1)
                continue
            t0 = time.time() + a.settle_s
            row = dict(step, t_start=t0, t_end=t0 + a.hold_s)
            time.sleep(a.settle_s)
            for left in range(int(a.hold_s), 0, -1):
                sys.stdout.write("\r  记录中，还剩 %2d s（别进画面）" % left)
                sys.stdout.flush()
                time.sleep(1)
            sys.stdout.write("\r  本轮完成。\a" + " " * 20 + "\n")
            out.write(json.dumps(row, ensure_ascii=False) + "\n")
            out.flush()
            i += 1
    print("打点写在 %s，做完了告诉我。" % a.out)


def off_axis_deg(bbox, width, hfov_deg):
    """框中心偏离画面中心的水平角（度，左负右正；针孔近似）。"""
    cx = (float(bbox[0]) + float(bbox[2])) / 2
    f = (width / 2) / math.tan(math.radians(hfov_deg / 2))
    return math.degrees(math.atan((cx - width / 2) / f))


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
                frames.append((float(t), r.get("detections") or [], r.get("tracks") or []))
    frames.sort(key=lambda x: x[0])
    return frames


def load_marks(path):
    rows = []
    with open(path, encoding="utf8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    last = {}                      # 同一轮重摆时以最后一次为准
    for r in rows:
        last[r["round"]] = r
    return [last[k] for k in sorted(last)]


def analyse(frames, marks, width, hfov_deg):
    results = []
    for m in marks:
        win = [f for f in frames if m["t_start"] <= f[0] <= m["t_end"]]
        labels_here = {OBJECTS[it["object"]][1] for it in m["items"]}
        other = {}
        for _t, dets, _tr in win:
            for d in dets:
                if d.get("label") not in labels_here:
                    other[d.get("label")] = other.get(d.get("label"), 0) + 1
        for it in m["items"]:
            label = OBJECTS[it["object"]][1]
            hits, scores, ws, hs, angs = 0, [], [], [], []
            for _t, dets, _tr in win:
                mine = [d for d in dets if d.get("label") == label]
                if mine:
                    best = max(mine, key=lambda d: d["score"])
                    hits += 1
                    scores.append(float(best["score"]))
                    ws.append(float(best["bbox"][2]) - float(best["bbox"][0]))
                    hs.append(float(best["bbox"][3]) - float(best["bbox"][1]))
                    angs.append(off_axis_deg(best["bbox"], width, hfov_deg))
            remembered = any(t.get("label") == label and t.get("state") == "current_confirmed"
                             and isinstance(t.get("last_seen"), (int, float))
                             and m["t_start"] <= t["last_seen"] <= m["t_end"] + 1.0
                             for _t, _d, tracks in win for t in tracks)
            results.append({
                "round": m.get("round"), "distance_m": it["distance_m"], "object": it["object"],
                "frames": len(win), "hits": hits, "rate": round(hits / len(win), 3) if win else None,
                "score_mean": round(statistics.mean(scores), 3) if scores else None,
                "score_max": round(max(scores), 3) if scores else None,
                "box_w_px": round(statistics.median(ws), 1) if ws else None,
                "box_h_px": round(statistics.median(hs), 1) if hs else None,
                "off_axis_deg": round(statistics.median(angs), 1) if angs else None,
                "remembered": remembered, "other_labels": other})
    return results


def summary(results, rate):
    """每样东西：检出率 >= rate 且更近各档也都达标的最远一档；curve = 距离 -> 检出率。"""
    out = {}
    for obj in OBJECTS:
        rows = sorted((r for r in results if r["object"] == obj and r["rate"] is not None),
                      key=lambda r: r["distance_m"])
        if not rows:
            continue
        best = None
        for r in rows:
            if r["rate"] < rate:
                break
            best = r["distance_m"]
        out[obj] = {"farthest_ok_m": best, "curve": {str(r["distance_m"]): r["rate"] for r in rows}}
    return out


def cmd_report(a):
    frames = load_frames(a.runtime)
    marks = load_marks(a.marks)
    results = analyse(frames, marks, a.width, a.hfov_deg)
    summ = summary(results, a.rate)
    print("轮 距离   物体       帧数  检出率   均分   最高    框宽x高  偏角  记忆  窗口里别的类别")
    for r in sorted(results, key=lambda r: (r["object"], r["distance_m"])):
        box = "%sx%s" % (r["box_w_px"], r["box_h_px"]) if r["box_w_px"] is not None else "-"
        print("%-2s %-5.1f %-9s %5d %6s %6s %6s %10s %5s  %-3s %s" % (
            r["round"], r["distance_m"], OBJECTS[r["object"]][0], r["frames"],
            "-" if r["rate"] is None else "%.0f%%" % (100 * r["rate"]),
            "-" if r["score_mean"] is None else r["score_mean"],
            "-" if r["score_max"] is None else r["score_max"], box,
            "-" if r["off_axis_deg"] is None else "%+.0f°" % r["off_axis_deg"],
            "是" if r["remembered"] else "否",
            ", ".join("%s×%d" % kv for kv in sorted(r["other_labels"].items(), key=lambda kv: -kv[1])) or "-"))
    print("\n识别距离（检出率 >= %.0f%% 且更近各档也达标的最远一档）：" % (100 * a.rate))
    for obj, v in summ.items():
        print("  %s：%s   曲线 %s" % (OBJECTS[obj][0],
                                   "-" if v["farthest_ok_m"] is None else "%.1f m" % v["farthest_ok_m"], v["curve"]))
    if not marks:
        print("没有打点。")
    elif all(r["frames"] == 0 for r in results):
        print("⚠ 打点窗口里没有一帧 frame_processed：时钟不一致（mark 不在 Nano 上跑？）或运行记录选错。")
    if a.json:
        with open(a.json, "w", encoding="utf8") as f:
            json.dump({"rate_threshold": a.rate, "width_px": a.width, "hfov_deg": a.hfov_deg,
                       "results": results, "summary": summ}, f, ensure_ascii=False, indent=1)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd")
    m = sub.add_parser("mark")
    m.add_argument("--out", required=True)
    m.add_argument("--objects", default=",".join(ROUND_OBJECTS), help="只测这几样，如 phone_flat,cup")
    m.add_argument("--hold-s", type=float, default=15.0)
    m.add_argument("--settle-s", type=float, default=2.0)
    r = sub.add_parser("report")
    r.add_argument("runtime")
    r.add_argument("marks")
    r.add_argument("--width", type=int, default=640, help="彩色画面宽（像素）")
    r.add_argument("--hfov-deg", type=float, default=86.0, help="彩色水平视场角（只用于算偏角）")
    r.add_argument("--rate", type=float, default=0.8)
    r.add_argument("--json")
    pl = sub.add_parser("plan")
    pl.add_argument("--objects", default=",".join(ROUND_OBJECTS))
    a = p.parse_args(argv)
    if a.cmd == "mark":
        cmd_mark(a)
    elif a.cmd == "report":
        cmd_report(a)
    elif a.cmd == "plan":
        for s in default_plan(parse_objects(a.objects)):
            print(describe(s))
    else:
        p.print_help()
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
