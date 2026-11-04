#!/usr/bin/env python3
"""FPGA 验收读数（10-25）：读一个车栈运行目录，逐节给 PASS / FAIL / NODATA（无数据）及依据。

    python3 tools/car_run/accept_readout.py <run_dir> [--json OUT.json] [--expect-nvpmodel NAME]

--expect-nvpmodel 不给时期望 MAXN_SUPER（owner 10-26 定：测试与正式比赛统一 MAXN_SUPER）；给空串 '' 则只报不判。

只读：不写运行目录（--json 必须给到别处，给到运行目录内会被拒）。只用标准库，py3.8 可跑。
缺文件/缺字段一律 NODATA，不报错。各节：
  1 版本与配置   head.txt / dirty.txt / source_kind.txt / marks.txt / runtime.jsonl startup（时限键）
  2 建图对齐     mapping_status.json align；runtime.jsonl memory_align*；bench_pose.log
  3 寻物任务     runtime.jsonl task / navigation_* / fault / query_rejected
  4 导航配置     navigation.log 的 real_car 行（rotation_shim、yaw_goal_tolerance）；Nav2 失败计数作证据
  5 深度         frames.jsonl 连续 dropped:no_valid_depth 段（>=3.0 s 即控制台提示）
  6 识别         task 行 processed/dropped_frames；否则 frame_processed / frame_rejected
  7 负载         RUN/accept/ 或 RUN/ 下 tegrastats* / pidstat* / nvpmodel*（手动存放）
  8 1.5 cm 三项  RUN/accept/live_mapping_status*.json(l)、ws_probe*.txt、bridge-stats*.json（手动存放）
阈值是 10-25 定的假设，列在输出的 assumptions 里。
"""
import argparse
import glob
import json
import os
import re
import sys

PASS, FAIL, NODATA = "PASS", "FAIL", "NODATA"
CN = {PASS: "通过", FAIL: "不过", NODATA: "无数据"}

TERMINAL = ("found", "not_found", "unable_to_confirm", "failed", "cancelled", "needs_manual_approach")
COMMON_KEYS = ("event", "time", "pose_source", "sensor_source", "inference_device", "clock_source",
               "inference_backend")

FIELDS_ASSUMED = {
    "startup.navigation_timeout_s": "runtime.jsonl startup 事件：生效的单目标导航时限（秒）",
    "startup.task_timeout_s": "runtime.jsonl startup 事件：生效的任务时限（秒）",
    "startup.*retr*/*attempt*": "startup 中名字含 retr/attempt 的键原样列出（重试参数）",
    "task.goal": "runtime.jsonl task 事件：寻物查询文本",
    "task.attempts": "runtime.jsonl task 事件：（导航）尝试次数",
}

ALIGN_RED_M = 0.30          # 根节点修正量 >= 0.3 m：疑似 poses 为相机系（相机装高 0.39 m、前 0.1 m）
DEPTH_BLOCKED_S = 3.0       # ui/geomem/bridge/gmb_proto.py DEPTH_BLOCKED_S
DROP_RATIO_MAX = 0.20       # 识别丢帧比上限（假设）
VOXEL_MS_MAX = 800.0        # 体素比对耗时上限（10-21 实测 1.5 cm / 90 万点 0.52 s）
SNAPSHOT_GAP_MAX_S = 2.0    # 页面 2 s 判快照不新鲜
CPU_AVG_MAX = 90.0          # 负载：各核平均 CPU% 均值上限（假设）
GPU_AVG_MAX = 95.0          # 负载：GR3D 均值上限（假设）
TEMP_MAX_C = 85.0           # 负载：最高温度上限（假设）
YAW_TOL_EXPECT = 0.10
ROOT_RE = re.compile(r"correction is ([\d.]+) m / ([\d.]+) rad")

EXPECT_NVPMODEL_DEFAULT = "MAXN_SUPER"   # owner 10-26: tests and the competition both run MAXN_SUPER (CLI default)

ASSUMPTIONS = [
    "建图对齐：根节点修正 >= %.2f m 标红「疑似 poses 为相机系」" % ALIGN_RED_M,
    "深度：连续 dropped:no_valid_depth >= %.1f s 判不过（与控制台提示同阈）" % DEPTH_BLOCKED_S,
    "识别：丢帧/(处理+丢帧) > %.0f%% 判不过" % (DROP_RATIO_MAX * 100),
    "导航配置：rotation_shim 为对象且 yaw_goal_tolerance == %.2f 判通过" % YAW_TOL_EXPECT,
    "寻物：全部任务终态 found 判通过；有任务未到终态或终态非 found 判不过",
    "负载：CPU 均值 > %.0f%% / GPU 均值 > %.0f%% / 最高温 > %.0f C 判不过；nvpmodel 命令行默认期望 MAXN_SUPER（owner 10-26），--expect-nvpmodel '' 只报不判"
    % (CPU_AVG_MAX, GPU_AVG_MAX, TEMP_MAX_C),
    "1.5 cm：体素比对 voxel.ms 最大 < %.0f ms；ws_probe snapshot gap_max < %.1f s" % (VOXEL_MS_MAX, SNAPSHOT_GAP_MAX_S),
]


# ---------------------------------------------------------------------------- helpers

def path(run, *parts):
    return os.path.join(run, *parts)


def read_text(p):
    try:
        with open(p, "r", encoding="utf-8", errors="replace") as f:
            return f.read()
    except (OSError, IOError):
        return None


def read_json(p):
    text = read_text(p)
    if text is None:
        return None
    try:
        return json.loads(text)
    except ValueError:
        return None


def iter_jsonl(p):
    try:
        f = open(p, "r", encoding="utf-8", errors="replace")
    except (OSError, IOError):
        return
    with f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                o = json.loads(line)
            except ValueError:
                continue
            if isinstance(o, dict):
                yield o


def find_files(run, patterns):
    """Manual files: RUN/accept/<pat> first, then RUN/<pat>; sorted, unique."""
    out = []
    for base in (path(run, "accept"), run):
        for pat in patterns:
            for p in sorted(glob.glob(os.path.join(base, pat))):
                if os.path.isfile(p) and p not in out:
                    out.append(p)
    return out


def section(name, verdict, evidence=None, notes=None, red=None):
    return {"name": name, "verdict": verdict, "evidence": evidence or {}, "notes": list(notes or []),
            "red": list(red or [])}


def strip_common(o):
    return dict((k, v) for k, v in o.items() if k not in COMMON_KEYS)


# ---------------------------------------------------------------------------- runtime.jsonl (one pass)

class Runtime(object):
    KEEP = ("startup", "task", "navigation_started", "navigation_result", "navigation_retired", "fault",
            "query_rejected", "memory_align_received", "memory_aligned", "memory_align_skipped",
            "memory_align_refused", "goto_accepted", "goto_result", "command")

    def __init__(self, run):
        self.present = os.path.isfile(path(run, "runtime.jsonl"))
        self.events = []
        self.counts = {}
        last_task = None
        for o in iter_jsonl(path(run, "runtime.jsonl")):
            e = o.get("event")
            self.counts[e] = self.counts.get(e, 0) + 1
            if e not in self.KEEP:
                continue
            o = strip_common(dict(o, _t=o.get("time")))
            o["event"] = e
            if e == "task":
                key = json.dumps(dict((k, v) for k, v in o.items() if k != "_t"), sort_keys=True)
                if key == last_task:            # identical consecutive task rows (published twice)
                    continue
                last_task = key
            self.events.append(o)

    def of(self, *names):
        return [o for o in self.events if o["event"] in names]


# ---------------------------------------------------------------------------- 1 版本与配置

def parse_marks(run):
    text = read_text(path(run, "marks.txt"))
    if text is None:
        return None
    out = {"start": None, "ready": {}, "READY": False, "stopped": False, "mapping": False}
    for line in text.splitlines():
        parts = line.split(" ", 1)
        if len(parts) < 2:
            continue
        rest = parts[1].strip()
        if rest.startswith("start "):
            out["start"] = dict(kv.split("=", 1) for kv in rest[6:].split() if "=" in kv)
        elif rest == "READY":
            out["READY"] = True
        elif rest == "stopped":
            out["stopped"] = True
        elif rest.startswith("ready "):
            body = rest[6:]
            what = body.split(" ", 1)[0]
            out["ready"][what] = body
            if what == "mapping":
                out["mapping"] = True
    return out


def sec_version(run, rt):
    ev, notes, red = {}, [], []
    head = read_text(path(run, "head.txt"))
    ev["head"] = head.strip() if head else None
    sk = read_text(path(run, "source_kind.txt"))
    ev["source_kind"] = sk.strip() if sk else None
    dirty = read_text(path(run, "dirty.txt"))
    ev["dirty"] = None if dirty is None else bool(dirty.strip())
    marks = parse_marks(run)
    if marks:
        ev["start"] = marks["start"]
        ev["READY"] = marks["READY"]
        ev["stopped"] = marks["stopped"]
        wo = marks["ready"].get("wheel_odom")
        m = re.search(r"heading=(\S+)", wo or "")
        ev["heading"] = m.group(1) if m else ("无数据（旧版本 marks 无 heading=）" if wo else None)
        nav = marks["ready"].get("nav2")
        ev["nav2_ready"] = nav
    car = read_json(path(run, "car.json"))
    if isinstance(car, dict):
        ev["car"] = dict((k, car.get(k)) for k in ("camera", "pose", "nav", "detector", "chassis", "platform", "tag")
                         if k in car)
    rc = read_json(path(run, "run_config.json"))
    if isinstance(rc, dict):
        ev["controller"] = rc.get("controller")
        inf = rc.get("inference") if isinstance(rc.get("inference"), dict) else {}
        ev["yolo_model"] = inf.get("yolo_model")
    starts = rt.of("startup")
    st = starts[-1] if starts else {}
    for k in ("navigation_timeout_s", "task_timeout_s"):
        ev[k] = st.get(k) if k in st else ("无数据（旧版本，startup 无此键）" if starts else None)
    retry = dict((k, v) for k, v in st.items() if ("retr" in k.lower() or "attempt" in k.lower()))
    if retry:
        ev["retry_params"] = retry
    if head is None and marks is None:
        return section("版本与配置", NODATA, ev, ["无 head.txt / marks.txt"])
    verdict = PASS
    if ev["dirty"]:
        verdict = FAIL
        red.append("工作树有未提交改动（dirty.txt 非空）")
    if marks is not None and not marks["READY"]:
        verdict = FAIL
        red.append("marks.txt 无 READY：栈没起全")
    if head is None:
        notes.append("无 head.txt")
    return section("版本与配置", verdict, ev, notes, red)


# ---------------------------------------------------------------------------- 2 建图对齐

def sec_align(run, rt):
    ev, notes, red = {}, [], []
    marks = parse_marks(run) or {}
    st = read_json(path(run, "mapping_status.json"))
    mem = rt.of("memory_align_received", "memory_aligned", "memory_align_skipped", "memory_align_refused")
    if mem:
        ev["runtime_memory_align"] = [dict((k, o.get(k)) for k in ("event", "key", "applied", "identity", "moved",
                                       "skipped_not_seen_this_run", "out_of_span_lookups", "reason") if k in o)
                                      for o in mem]
    bp = read_text(path(run, "bench_pose.log")) or ""
    resent = [ln for ln in bp.splitlines() if "map align: map -> odom re-sent" in ln]
    refused = [ln for ln in bp.splitlines() if "map align refused" in ln]
    if resent:
        ev["bench_pose_resent"] = resent[-1].strip()
    if refused:
        ev["bench_pose_refused"] = refused[-1].strip()
    if st is None:
        if marks.get("mapping"):
            return section("建图对齐", FAIL, ev, notes, ["建图趟（marks 有 ready mapping）但没有 mapping_status.json"])
        return section("建图对齐", NODATA, ev, ["非建图趟（无 mapping_status.json）"])
    align = st.get("align") if isinstance(st, dict) else None
    ev["state"] = st.get("state") if isinstance(st, dict) else None
    if not isinstance(align, dict):
        return section("建图对齐", NODATA, ev, ["align 为空（对齐未运行；state=%s）" % ev["state"]])
    ev["ok"] = align.get("ok")
    if align.get("ok"):
        root = align.get("root") or {}
        ev.update({"identity": align.get("identity"), "node_count": align.get("count"),
                   "root_offset_m": root.get("offset_m"), "root_offset_rad": root.get("offset_rad"),
                   "map_odom": align.get("map_odom"), "unmatched": align.get("unmatched"),
                   "applied": align.get("applied"), "applied_reason": align.get("applied_reason")})
        verdict = PASS
        if refused and not resent:
            verdict = FAIL
            red.append("bench_pose 拒收对齐，map->odom 未改")
        return section("建图对齐", verdict, ev, notes, red)
    reason = align.get("reason") or ""
    ev["reason"] = reason
    m = ROOT_RE.search(reason)
    if m:
        off = float(m.group(1))
        ev["root_offset_m"] = off
        ev["root_offset_rad"] = float(m.group(2))
        if off >= ALIGN_RED_M:
            red.append("根节点修正 %.3f m >= %.2f m：疑似 poses 为相机系" % (off, ALIGN_RED_M))
    return section("建图对齐", FAIL, ev, notes, red)


# ---------------------------------------------------------------------------- 3 寻物任务

def split_tasks(rt):
    """Tasks from task rows (terminal/none -> non-terminal opens one, terminal closes it); then navigation legs
    and faults go to the task whose window (previous task's end, this task's end] holds their time (a leg may
    start just before the first task row of its task)."""
    tasks, cur = [], None
    for o in rt.events:
        if o["event"] != "task":
            continue
        state = o.get("state")
        if cur is None or cur["final"] is not None:
            if state in TERMINAL and cur is not None:
                cur["final"] = o                # repeated terminal row
                cur["end"] = o.get("_t")
                continue
            cur = {"start": o.get("_t"), "end": None, "rows": [], "legs": {}, "faults": [], "final": None}
            tasks.append(cur)
        cur["rows"].append(o)
        cur["end"] = o.get("_t")
        if state in TERMINAL:
            cur["final"] = o
    if not tasks:
        return tasks
    open_end = float("inf")

    def owner(t):
        if not isinstance(t, (int, float)):
            return None
        prev = float("-inf")
        for i, k in enumerate(tasks):
            end = k["end"] if (k["final"] is not None or i < len(tasks) - 1) else open_end
            if prev < t <= end or (i == 0 and t <= end):
                return k
            prev = end
        return None

    legs = {}
    for o in rt.events:
        e = o["event"]
        if e == "navigation_started":
            k = owner(o.get("_t"))
            if k is not None:
                cid = o.get("command_id") or ""
                leg = {"command_id": cid, "start": o.get("_t"), "end": None, "result": None}
                k["legs"][cid] = leg
                legs[cid] = leg
                if k["start"] is None or (isinstance(o.get("_t"), (int, float)) and o["_t"] < k["start"]):
                    k["start"] = o["_t"]
        elif e == "navigation_result":
            leg = legs.get(o.get("command_id"))
            if leg is not None and leg["end"] is None:
                leg["end"] = o.get("_t")
                leg["result"] = ("succeeded" if o.get("succeeded") else
                                 "cancelled" if o.get("cancelled") else "failed")
        elif e == "fault":
            k = owner(o.get("_t"))
            if k is not None:
                k["faults"].append(o.get("reason"))
    return tasks


def sec_find(run, rt):
    ev, notes, red = {}, [], []
    if not rt.present:
        return section("寻物任务", NODATA, ev, ["无 runtime.jsonl"])
    tasks = split_tasks(rt)
    rej = rt.of("query_rejected")
    if rej:
        ev["query_rejected"] = [o.get("reason") for o in rej]
    goto = rt.of("goto_result")
    if goto:
        ev["goto_results"] = "%d 次 goto，成功 %d" % (len(goto), sum(1 for o in goto if o.get("succeeded")))
    if not tasks:
        return section("寻物任务", NODATA, ev, ["本趟无寻物任务（无 task 行）"])
    rows = []
    all_found = True
    for i, t in enumerate(tasks, 1):
        fin = t["final"]
        last = t["rows"][-1]
        end_t = fin.get("_t") if fin else last.get("_t")
        cand = last.get("candidates") or []
        legs = list(t["legs"].values())
        r = {"index": i,
             "goal": next((x["goal"] for x in t["rows"] if "goal" in x), "无数据（旧版本）"),
             "target_id": last.get("target_id"),
             "candidate_labels": [c.get("label") for c in cand if isinstance(c, dict)],
             "final_state": fin.get("state") if fin else "未结束（最后 %s）" % last.get("state"),
             "reason": (fin or last).get("reason"),
             "duration_s": (round(end_t - t["start"], 2)
                            if isinstance(end_t, (int, float)) and isinstance(t["start"], (int, float)) else None),
             "attempts": next((x["attempts"] for x in reversed(t["rows"]) if "attempts" in x), "无数据（旧版本）"),
             "nav_legs": [{"command_id": g["command_id"], "result": g["result"] or "无结果",
                           "s": (round(g["end"] - g["start"], 2) if g["end"] is not None and g["start"] is not None
                                 else None)} for g in legs],
             "faults": t["faults"]}
        rows.append(r)
        if not fin or fin.get("state") != "found":
            all_found = False
            red.append("任务 %d：%s / %s" % (i, r["final_state"], r["reason"]))
    ev["tasks"] = rows
    ev["summary"] = "%d 个任务，found %d" % (len(rows), sum(1 for r in rows if r["final_state"] == "found"))
    return section("寻物任务", PASS if all_found else FAIL, ev, notes, red)


# ---------------------------------------------------------------------------- 4 导航配置

def sec_nav(run, rt):
    ev, notes, red = {}, [], []
    text = read_text(path(run, "navigation.log"))
    nav_s = rt.of("navigation_result")
    if nav_s:
        ev["nav_goals"] = {"succeeded": sum(1 for o in nav_s if o.get("succeeded")),
                           "cancelled": sum(1 for o in nav_s if o.get("cancelled")),
                           "failed": sum(1 for o in nav_s if not o.get("succeeded") and not o.get("cancelled"))}
    faults = rt.of("fault")
    if faults:
        ev["faults"] = [o.get("reason") for o in faults]
    if text is None:
        return section("导航配置", NODATA, ev, ["无 navigation.log"])
    ev["nav2_failed_to_make_progress"] = text.count("Failed to make progress")
    ev["follow_path_result_lost"] = text.count("Failed to get result for follow_path")
    car = None
    for line in text.splitlines():
        if '"real_car"' not in line:
            continue
        i = line.find("{")
        try:
            car = json.loads(line[i:])
        except ValueError:
            continue
    if car is None:
        return section("导航配置", NODATA, ev, ["navigation.log 无 real_car 行（仿真档）"])
    ev["real_car"] = car.get("real_car")
    if "rotation_shim" not in car and "yaw_goal_tolerance" not in car:
        return section("导航配置", NODATA, ev, ["无数据（旧版本，早于 fa542c6：real_car 行无 rotation_shim / "
                                               "yaw_goal_tolerance 键）"])
    shim, tol = car.get("rotation_shim"), car.get("yaw_goal_tolerance")
    ev["rotation_shim"], ev["yaw_goal_tolerance"] = shim, tol
    verdict = PASS
    if not isinstance(shim, dict):
        verdict = FAIL
        red.append("RotationShim 未开（rotation_shim=%r）" % (shim,))
    if not isinstance(tol, (int, float)) or abs(tol - YAW_TOL_EXPECT) > 1e-9:
        verdict = FAIL
        red.append("yaw_goal_tolerance=%r，应 %.2f" % (tol, YAW_TOL_EXPECT))
    return section("导航配置", verdict, ev, notes, red)


# ---------------------------------------------------------------------------- 5 深度

def sec_depth(run):
    ev, notes, red = {}, [], []
    p = path(run, "frames.jsonl")
    if not os.path.isfile(p):
        return section("深度", NODATA, ev, ["无 frames.jsonl"])
    counts = {}
    longest, cur_start, cur_last, n_runs = 0.0, None, None, 0
    for o in iter_jsonl(p):
        res = o.get("result")
        if res is None:
            continue
        counts[res] = counts.get(res, 0) + 1
        t = o.get("acquired_ns")
        if res == "dropped:no_valid_depth":
            if cur_start is None:
                cur_start, n_runs = t, n_runs + 1
            cur_last = t
            if isinstance(cur_start, (int, float)) and isinstance(cur_last, (int, float)):
                longest = max(longest, (cur_last - cur_start) / 1e9)
        else:
            cur_start = cur_last = None
    ev["results"] = counts
    total = sum(counts.values())
    if not total:
        return section("深度", NODATA, ev, ["frames.jsonl 无 result 行"])
    nvd = counts.get("dropped:no_valid_depth", 0)
    ev["no_valid_depth"] = nvd
    ev["no_valid_depth_runs"] = n_runs
    ev["longest_no_valid_depth_s"] = round(longest, 3)
    ps = read_json(path(run, "producer_summary.json"))
    if isinstance(ps, dict):
        notes.append("producer_summary.json 在（可交叉核对 fused/dropped 总数）")
    if longest >= DEPTH_BLOCKED_S:
        red.append("连续无有效深度 %.1f s >= %.1f s（控制台会提示深度被挡）" % (longest, DEPTH_BLOCKED_S))
        return section("深度", FAIL, ev, notes, red)
    return section("深度", PASS, ev, notes, red)


# ---------------------------------------------------------------------------- 6 识别

def sec_detect(run, rt):
    ev, notes, red = {}, [], []
    if not rt.present:
        return section("识别", NODATA, ev, ["无 runtime.jsonl"])
    tasks = rt.of("task")
    ev["frame_processed_events"] = rt.counts.get("frame_processed", 0)
    if tasks:
        last = tasks[-1]
        proc, drop = last.get("processed_frames"), last.get("dropped_frames")
        src = "最后一行 task 的 processed_frames / dropped_frames"
    elif rt.counts.get("frame_rejected"):
        proc, drop = rt.counts.get("frame_processed", 0), rt.counts.get("frame_rejected", 0)
        src = "frame_processed / frame_rejected 事件数"
    else:
        return section("识别", NODATA, ev, ["无 task 行也无 frame_rejected 事件：丢帧数无从得知（只报处理帧数）"])
    ev["source"], ev["processed"], ev["dropped"] = src, proc, drop
    if not isinstance(proc, int) or not isinstance(drop, int) or proc + drop == 0:
        return section("识别", NODATA, ev, ["计数缺失或为零"])
    ratio = drop / float(proc + drop)
    ev["drop_ratio"] = round(ratio, 4)
    if ratio > DROP_RATIO_MAX:
        red.append("丢帧比 %.1f%% > %.0f%%" % (ratio * 100, DROP_RATIO_MAX * 100))
        return section("识别", FAIL, ev, notes, red)
    return section("识别", PASS, ev, notes, red)


# ---------------------------------------------------------------------------- 7 负载

TEG_VDD = re.compile(r"VDD_IN (\d+)mW")
TEG_CPU = re.compile(r"CPU \[([^\]]*)\]")
TEG_GPU = re.compile(r"GR3D_FREQ (\d+)%")
TEG_TEMP = re.compile(r"\b[A-Za-z0-9_]+@(-?\d+(?:\.\d+)?)C\b")


def parse_tegrastats_line(line):
    """Same regexes as host/hub/hub_core.py parse_tegrastats."""
    out = {"vdd_in_mw": None, "cpu_pct": None, "gpu_pct": None, "temp_c_max": None}
    m = TEG_VDD.search(line)
    if m:
        out["vdd_in_mw"] = int(m.group(1))
    m = TEG_CPU.search(line)
    if m:
        cpus = []
        for item in m.group(1).split(","):
            p = re.match(r"\s*(\d+)%", item)
            cpus.append(int(p.group(1)) if p else None)
        out["cpu_pct"] = cpus
    m = TEG_GPU.search(line)
    if m:
        out["gpu_pct"] = int(m.group(1))
    temps = [float(t) for t in TEG_TEMP.findall(line)]
    temps = [t for t in temps if -40.0 < t < 150.0]
    if temps:
        out["temp_c_max"] = max(temps)
    return out


def mean(v):
    return sum(v) / float(len(v)) if v else None


def parse_pidstat(text):
    """pidstat -u: per Command, mean %CPU over sample lines (Average lines skipped)."""
    per = {}
    cols = None
    for line in text.splitlines():
        f = line.split()
        if not f or f[0].startswith("Average") or f[0].startswith("平均"):
            continue
        if "%CPU" in f and "Command" in f:
            cols = (f.index("%CPU"), f.index("Command"))
            continue
        if cols is None or len(f) <= max(cols):
            continue
        # header and data lines carry the same leading time columns -> same index
        try:
            cpu = float(f[cols[0]])
        except ValueError:
            continue
        per.setdefault(f[cols[1]], []).append(cpu)
    return dict((k, round(mean(v), 1)) for k, v in per.items())


def sec_load(run, expect_nvpmodel=None):
    ev, notes, red = {}, [], []
    teg = find_files(run, ["tegrastats*"])
    pid = find_files(run, ["pidstat*"])
    nvp = find_files(run, ["nvpmodel*"])
    hs = read_json(path(run, "hub_stats.json"))
    if isinstance(hs, dict) and isinstance(hs.get("counts"), dict):
        flags = [k for k in hs["counts"] if k.endswith("_unavailable")]
        if flags:
            ev["hub_stats_unavailable"] = flags
    judged = False
    verdict = PASS
    if teg:
        samples = []
        for p in teg:
            for line in (read_text(p) or "").splitlines():
                s = parse_tegrastats_line(line)
                if any(v is not None for v in s.values()):
                    samples.append(s)
        ev["tegrastats_files"] = [os.path.relpath(p, run) for p in teg]
        ev["tegrastats_samples"] = len(samples)
        if samples:
            judged = True
            cpu = [mean([c for c in s["cpu_pct"] if c is not None]) for s in samples
                   if s["cpu_pct"] and any(c is not None for c in s["cpu_pct"])]
            gpu = [s["gpu_pct"] for s in samples if s["gpu_pct"] is not None]
            tmp = [s["temp_c_max"] for s in samples if s["temp_c_max"] is not None]
            vdd = [s["vdd_in_mw"] for s in samples if s["vdd_in_mw"] is not None]
            ev["cpu_avg_pct"] = round(mean(cpu), 1) if cpu else None
            ev["cpu_max_pct"] = round(max(cpu), 1) if cpu else None
            ev["gpu_avg_pct"] = round(mean(gpu), 1) if gpu else None
            ev["gpu_max_pct"] = max(gpu) if gpu else None
            ev["temp_max_c"] = max(tmp) if tmp else None
            ev["vdd_in_avg_mw"] = round(mean(vdd)) if vdd else None
            if cpu and mean(cpu) > CPU_AVG_MAX:
                verdict = FAIL
                red.append("CPU 均值 %.0f%% > %.0f%%" % (mean(cpu), CPU_AVG_MAX))
            if gpu and mean(gpu) > GPU_AVG_MAX:
                verdict = FAIL
                red.append("GPU 均值 %.0f%% > %.0f%%" % (mean(gpu), GPU_AVG_MAX))
            if tmp and max(tmp) > TEMP_MAX_C:
                verdict = FAIL
                red.append("最高温 %.1f C > %.0f C" % (max(tmp), TEMP_MAX_C))
    if pid:
        per = {}
        for p in pid:
            per.update(parse_pidstat(read_text(p) or ""))
        top = sorted(per.items(), key=lambda kv: -kv[1])[:8]
        ev["pidstat_top_cpu"] = top
    if nvp:
        mode = None
        for p in nvp:
            m = re.search(r"NV Power Mode:\s*(\S+)", read_text(p) or "")
            if m:
                mode = m.group(1)
        ev["nvpmodel"] = mode
        if expect_nvpmodel:
            judged = True
            if mode != expect_nvpmodel:
                verdict = FAIL
                red.append("nvpmodel=%s，应 %s" % (mode, expect_nvpmodel))
        else:
            notes.append("nvpmodel 只报不判（命令行给了 --expect-nvpmodel ''）")
    if not judged:
        notes.append("无 tegrastats 文件（放 RUN/accept/tegrastats.txt）")
        return section("负载", NODATA, ev, notes)
    return section("负载", verdict, ev, notes, red)


# ---------------------------------------------------------------------------- 8 1.5 cm 三项

WS_ROW = re.compile(r"^(\S+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\S+)\s+([\d.naN]+)\s+([\d.naN]+)\s+([\d.naN]+)\s+(\d+)\s+(\d+)\s*$")


def parse_ws_probe(text):
    out = {}
    for line in text.splitlines():
        m = WS_ROW.match(line.strip())
        if not m:
            continue
        try:
            gap_max = float(m.group(8))
        except ValueError:
            gap_max = None
        out[m.group(1)] = {"n": int(m.group(2)), "gap_max_s": gap_max, "gaps_over_2s": int(m.group(10)),
                           "size_max": int(m.group(4))}
    return out


def load_status_objects(p):
    text = read_text(p) or ""
    try:
        o = json.loads(text)
        return o if isinstance(o, list) else [o]
    except ValueError:
        pass
    out = []
    for line in text.splitlines():
        try:
            o = json.loads(line)
        except ValueError:
            continue
        out.append(o)
    return out


def sec_fine(run, depth_sec):
    ev, notes, red = {}, [], []
    items = {}
    # a) 体素比对耗时
    st = find_files(run, ["live_mapping_status*.json", "live_mapping_status*.jsonl"])
    ms = []
    for p in st:
        for o in load_status_objects(p):
            v = o.get("voxel") if isinstance(o, dict) else None
            if isinstance(v, dict) and isinstance(v.get("ms"), (int, float)):
                ms.append(v["ms"])
    if ms:
        items["voxel_ms"] = PASS if max(ms) < VOXEL_MS_MAX else FAIL
        ev["voxel_ms"] = {"n": len(ms), "max": max(ms), "median": sorted(ms)[len(ms) // 2]}
        if items["voxel_ms"] == FAIL:
            red.append("体素比对最长 %d ms >= %.0f ms" % (max(ms), VOXEL_MS_MAX))
    else:
        items["voxel_ms"] = NODATA
    # b) 快照间隔
    ws = find_files(run, ["ws_probe*.txt", "ws_probe*.log"])
    snap = None
    for p in ws:
        rows = parse_ws_probe(read_text(p) or "")
        if "snapshot" in rows:
            snap = rows["snapshot"]
            ev["ws_probe_mapping"] = rows.get("mapping")
    if snap and snap["gap_max_s"] is not None:
        ev["snapshot"] = snap
        items["snapshot_gap"] = PASS if snap["gap_max_s"] < SNAPSHOT_GAP_MAX_S else FAIL
        if items["snapshot_gap"] == FAIL:
            red.append("快照最大间隔 %.2f s >= %.1f s（页面判不新鲜）" % (snap["gap_max_s"], SNAPSHOT_GAP_MAX_S))
    else:
        items["snapshot_gap"] = NODATA
    # c) 深度丢帧（第 5 节）
    items["depth"] = depth_sec["verdict"]
    # 证据：bridge-stats
    bs = find_files(run, ["bridge-stats*.json"])
    for p in bs:
        o = read_json(p)
        if isinstance(o, dict):
            ev["bridge_stats"] = {"file": os.path.relpath(p, run), "rejects": o.get("rejects"),
                                  "snapshot_issues_max": o.get("snapshot_issues_max"),
                                  "cloud_rev": o.get("cloud_rev")}
    ev["items"] = items
    vs = list(items.values())
    if FAIL in vs:
        verdict = FAIL
    elif items["voxel_ms"] == NODATA and items["snapshot_gap"] == NODATA:
        verdict = NODATA
        notes.append("体素耗时与快照间隔都没存（见 README：RUN/accept/）")
    elif NODATA in vs:
        verdict = NODATA
        notes.append("部分项无数据：%s" % ", ".join(k for k, v in items.items() if v == NODATA))
    else:
        verdict = PASS
    return section("1.5 cm 三项", verdict, ev, notes, red)


# ---------------------------------------------------------------------------- main

def readout(run, expect_nvpmodel=None):
    run = os.path.realpath(run)
    rt = Runtime(run)
    secs = [sec_version(run, rt), sec_align(run, rt), sec_find(run, rt), sec_nav(run, rt)]
    depth = sec_depth(run)
    secs += [depth, sec_detect(run, rt), sec_load(run, expect_nvpmodel), sec_fine(run, depth)]
    vs = [s["verdict"] for s in secs]
    overall = FAIL if FAIL in vs else (PASS if all(v == PASS for v in vs) else "PARTIAL")
    return {"run_dir": run, "sections": secs, "overall": overall, "fields_assumed": FIELDS_ASSUMED,
            "assumptions": ASSUMPTIONS}


def fmt(v):
    if isinstance(v, (dict, list)):
        s = json.dumps(v, ensure_ascii=False)
        return s if len(s) <= 300 else s[:297] + "..."
    return str(v)


def render(rep):
    lines = ["验收读数：%s" % rep["run_dir"]]
    for i, s in enumerate(rep["sections"], 1):
        lines.append("")
        lines.append("[%d] %s：%s" % (i, s["name"], CN[s["verdict"]] + "（" + s["verdict"] + "）"))
        for r in s["red"]:
            lines.append("    !! %s" % r)
        for k, v in s["evidence"].items():
            if k == "tasks":
                for t in v:
                    lines.append("    任务%d 目标=%s target=%s 终态=%s 原因=%s 用时=%ss 尝试=%s" % (
                        t["index"], t["goal"], t["target_id"], t["final_state"], t["reason"], t["duration_s"],
                        t["attempts"]))
                    for g in t["nav_legs"]:
                        lines.append("        导航 %s %s %ss" % (g["command_id"], g["result"], g["s"]))
                    if t["faults"]:
                        lines.append("        fault: %s" % ", ".join(str(x) for x in t["faults"]))
                continue
            lines.append("    %s: %s" % (k, fmt(v)))
        for n in s["notes"]:
            lines.append("    · %s" % n)
    lines.append("")
    lines.append("总评：%s" % {"PASS": "全部通过", "FAIL": "有不过项", "PARTIAL": "无不过项，但有无数据项"}[rep["overall"]])
    lines.append("假设字段名（待与运行时对齐）：")
    for k, v in rep["fields_assumed"].items():
        lines.append("    %s — %s" % (k, v))
    lines.append("阈值假设：")
    for a in rep["assumptions"]:
        lines.append("    %s" % a)
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description="FPGA 验收读数（只读运行目录）")
    ap.add_argument("run_dir")
    ap.add_argument("--json", default=None, help="JSON 输出路径（不得在运行目录内）；'-' 打到标准输出")
    ap.add_argument("--expect-nvpmodel", default=EXPECT_NVPMODEL_DEFAULT,
                    help="期望的 nvpmodel 模式名（默认 %(default)s，owner 10-26 定测试与比赛统一）；'' = 只报不判")
    args = ap.parse_args(argv)
    run = os.path.realpath(args.run_dir)
    if not os.path.isdir(run):
        sys.stderr.write("不是目录：%s\n" % run)
        return 2
    if args.json and args.json != "-":
        out = os.path.realpath(args.json)
        if out == run or out.startswith(run + os.sep):
            sys.stderr.write("--json 不能写进运行目录（工具只读）：%s\n" % out)
            return 2
    rep = readout(run, args.expect_nvpmodel or None)
    if args.json == "-":
        sys.stdout.write(json.dumps(rep, ensure_ascii=False, indent=1, default=str) + "\n")
        return 0
    sys.stdout.write(render(rep) + "\n")
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(rep, f, ensure_ascii=False, indent=1, default=str)
    return 0


if __name__ == "__main__":
    sys.exit(main())
