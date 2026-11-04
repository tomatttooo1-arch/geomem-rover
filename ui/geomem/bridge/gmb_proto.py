"""GeoMem protocol v2 framing: pure functions from hub (FPGAC) bodies to browser messages.

Design: ui/geomem/docs/PROTOCOL_V2_DESIGN.md §2 (protocol) and §3.2-§3.5 (field sources, cloud,
command mapping, translation tables). Python 3.8+ standard library only, no sockets, no clocks:
callers pass every time quantity in, so all of this is unit tested directly.

Conservative choices where the design is silent are listed in ui/geomem/bridge/README.md.
"""

import array
import base64
import json
import math
import re
import struct
import zlib

VERSION = 2
FRAMES = ("map", "odom")
SOURCES = ("live", "replay", "sim", "inconsistent")
MEMORY_STATES = ("current_confirmed", "historical_memory", "temporarily_occluded", "position_invalid")
MAX_OBJECTS = 1000
MAX_PATH = 5000
MAX_CLOUD_POINTS = 60000
MAX_FIND_TEXT = 64
MAX_CMD_ID = 128
OCCUPIED_MIN = 65                     # occupancy value >= 65: occupied (ROS map_server convention)
FREE_MAX = 25                         # 0..25: free
DEADLINE_MS = {"find": 3000, "select": 3000, "cancel": 5000,
               # 10-18 (controller ruling 4, docs/design/TELEOP_NAV_ARB_20261018.md section 0): the electronic control
               # interface forwards these too. estop: None = fpgac's own e-stop rule (no deadline, resent every 0.5 s
               # until the hub acks), whatever the link state.
               "goto": 3000, "mapping_stop": 3000, "teleop": 1000, "release": 5000, "estop": None,
               "map_reset": 3000}     # 10-26 重置地图 (host/map_reset.py)
# hub CMD_ACK -> CMD_RESULT wait per op (bridge; default RESULT_WAIT_S there). mapping_stop: the hub watches the
# mapping status up to 180 s (MAPPING_STOP_WINDOW_S; fpgapc waits 185); estop / release: the KV's answer.
# teleop has no CMD_RESULT (no hub watcher): its pending entry ends at the ack.
RESULT_WAIT_S = {"mapping_stop": 185.0, "estop": 30.0, "release": 30.0,
                 "map_reset": 35.0}   # 10-26: hub watches the runner's /fpga/map_reset/status up to 33 s
NO_RESULT_OPS = ("teleop",)
ESTOP_NO_ACK_S = 30.0                 # estop without a hub ack this long -> browser told expired (fpgac still resends)
# hub ACK statuses that mean "taken": estop answers latched_on_nano (the Nano latched before the KV confirms)
ACK_TAKEN = ("forwarded", "latched_on_nano")
# hub RESULT statuses of estop / release that mean done
RESULT_DONE = ("latched", "released")
MAX_GRID_CELLS = 4096 * 4096

# ---------------------------------------------------------------------------- translation tables

# COCO 80 English class -> Chinese (§3.5). The first seven are the runner query vocabulary reversed
# (perception/fpga_replay/core.py:335); their aliases are the other runner words for the same class.
COCO_ZH = {
    "person": "人", "bicycle": "自行车", "car": "汽车", "motorcycle": "摩托车", "airplane": "飞机",
    "bus": "公交车", "train": "火车", "truck": "卡车", "boat": "船", "traffic light": "红绿灯",
    "fire hydrant": "消防栓", "stop sign": "停车标志", "parking meter": "停车计时器", "bench": "长椅",
    "bird": "鸟", "cat": "猫", "dog": "狗", "horse": "马", "sheep": "羊", "cow": "牛",
    "elephant": "大象", "bear": "熊", "zebra": "斑马", "giraffe": "长颈鹿", "backpack": "书包",
    "umbrella": "雨伞", "handbag": "手提包", "tie": "领带", "suitcase": "行李箱", "frisbee": "飞盘",
    "skis": "滑雪板", "snowboard": "单板滑雪板", "sports ball": "球", "kite": "风筝",
    "baseball bat": "棒球棍", "baseball glove": "棒球手套", "skateboard": "滑板", "surfboard": "冲浪板",
    "tennis racket": "网球拍", "bottle": "瓶子", "wine glass": "酒杯", "cup": "杯子", "fork": "叉子",
    "knife": "刀", "spoon": "勺子", "bowl": "碗", "banana": "香蕉", "apple": "苹果",
    "sandwich": "三明治", "orange": "橙子", "broccoli": "西兰花", "carrot": "胡萝卜", "hot dog": "热狗",
    "pizza": "披萨", "donut": "甜甜圈", "cake": "蛋糕", "chair": "椅子", "couch": "沙发",
    "potted plant": "盆栽", "bed": "床", "dining table": "餐桌", "toilet": "马桶", "tv": "电视",
    "laptop": "笔记本电脑", "mouse": "鼠标", "remote": "遥控器", "keyboard": "键盘",
    "cell phone": "手机", "microwave": "微波炉", "oven": "烤箱", "toaster": "烤面包机", "sink": "水槽",
    "refrigerator": "冰箱", "book": "书", "clock": "时钟", "vase": "花瓶", "scissors": "剪刀",
    "teddy bear": "泰迪熊", "hair drier": "吹风机", "toothbrush": "牙刷",
    "medicine bottle": "药瓶", "medicine box": "药盒", "key": "钥匙", "glasses": "眼镜", "earbud case": "耳机充电盒",
    "waste paper": "废纸", "drink can": "饮料罐", "plastic packaging": "塑料包装",
}
ALIASES_ZH = {"cup": ["杯"], "bottle": ["瓶"], "backpack": ["背包"]}
ALIASES_ZH.update({'medicine bottle': ['药品瓶'], 'medicine box': ['药品盒'],
                   'waste paper': ['废纸团', '纸团'], 'drink can': ['易拉罐'],
                   'plastic packaging': ['塑料包装袋']})
# classes the runner understands by Chinese word (core.py:335); other Chinese names are sent in English
RUNNER_CLASSES = ("cup", "bottle", "backpack", "book", "mouse", "keyboard", "cell phone")
RUNNER_CLASSES += ('remote', 'medicine bottle', 'medicine box', 'key', 'glasses',
                   'waste paper', 'drink can', 'plastic packaging')

# 10-22 (owner): object names carry no colour any more -- the runner's HSV colour guess was often wrong
# (灰色书包 / 黑色书包 for one bag). The track's ``color`` field still goes out as data (protocol §2.3)
# but no Chinese text (name, task message, find-text translation) uses it.

# protocol task state -> interface Chinese (model.js STATES + the new ambiguous)
TASK_STATE_ZH = {"idle": "待命", "ambiguous": "请选择目标", "searching": "检索记忆", "planning": "规划路线",
                 "navigating": "正在前往", "confirming": "现场确认", "succeeded": "已确认找到",
                 "failed": "任务失败", "paused": "已暂停", "cancelled": "已停止"}
ACTIVE_TASK_STATES = ("ambiguous", "searching", "planning", "navigating", "confirming", "paused")

# Every code an ``ack`` may carry with accepted=false: (code, who decides, Chinese).
# The README table is checked against this tuple by a unit test.
REJECT_CODES = (
    ("not_controller", "hub", "另一台电脑持有控制权"),
    ("estop_latched", "hub", "急停锁存中"),
    ("bad_args", "hub", "请求格式不对"),
    ("bad_op", "hub", "车上不认识这个命令"),
    ("not_enabled", "hub", "车上没有启用这个功能"),
    ("replay", "hub", "回放模式下不能操作小车"),
    ("not_supported", "bridge", "当前版本不支持这个操作"),
    ("scope_changed", "bridge", "记忆已重置，请重新选择"),
    ("no_such_candidate", "bridge", "该候选不在当前候选列表里"),
    ("stale_task", "bridge", "任务已经变了，请按最新任务重新操作"),
    ("not_ambiguous", "bridge", "当前不需要选择候选"),
    ("link_down", "bridge", "和小车的连接断了，请等连接恢复再操作"),
    ("source_inconsistent", "bridge", "车上数据来源不一致，暂不接受找物或选择"),
    ("bad_request", "bridge", "请求格式不对"),
    ("no_ack", "bridge", "小车没有回应这条命令"),
    # 10-18 (ruling 4): release makes the car movable again -- forwarded only with payload.confirm === true
    ("confirm_required", "bridge", "解除急停会让小车重新可以动，请求里须显式确认"),
    # hub answers the new ops can carry (host/hub/hub_core.py)
    ("not_latched", "hub", "当前没有急停锁存"),
    ("epoch_mismatch", "hub", "急停已更新，请按最新急停再解除"),
    ("estop_in_flight", "hub", "急停还在下发，稍后再解除"),
    ("superseded_by_estop", "hub", "已被急停取代"),
    ("no_mapping_status", "hub", "车上没有在建图"),
    ("mapping_saving", "hub", "建图正在保存"),
    ("mapping_exported", "hub", "建图已结束并导出"),
    ("mapping_failed", "hub", "建图已失败"),
    ("teleop_active", "hub", "遥控在手，导航命令被拒；松手后重发"),
    # 10-26 重置地图 (map_reset; host/map_reset.py)
    ("map_reset_no_mapping", "hub", "定位模式（使用已存地图）或本趟没有启用建图，不能重置地图"),
    ("map_reset_busy", "hub", "上一次重置地图还没结束"),
    ("mapping_resetting", "hub", "地图正在重置"),
    ("map_reset_failed", "hub", "地图重置没有全部完成"),
)

# runner / hub reason words -> Chinese (§3.5). Words not here are shown as they are (never invented).
REASON_ZH = {
    "candidate_selection_required": "记忆里有多个符合的物品，请选择要找哪一个",
    "search_candidate_selection_required": "搜索中看到了新的候选，请确认是不是要找的那个",
    "search": "正在按搜索路线寻找",
    # 10-26 search MVP (host/task_control.py): why the search started / the single-candidate verdict
    "target_position_invalid": "原位置已空（物品被移走），正在按搜索路线寻找",
    "not_seen_at_last_location": "原位置没看到物品，正在按搜索路线寻找",
    "no_observation_pose": "原处附近无处可看，开始按搜索路线寻找",
    "search_single_candidate": "搜索中看到唯一一个同类物品，判定为目标",
    "reobserve": "正在前往目标附近",
    "fresh_visual_evidence_required": "已到观察点，正在现场确认",
    "fresh_matching_observation": "已在现场再次看到目标",
    # 10-26 find-near: confirmed while driving, within 1.2 m (TaskConfig.en_route_max_distance_m)
    "seen_en_route": "途中已在近处看到目标，已停车",
    "search_exhausted": "搜索路线走完仍未找到",
    "observation_positions_exhausted": "几个观察位置都没能确认目标",
    "visit_budget_exhausted": "几个观察位置都没能确认目标",
    "navigation_failed": "导航没能到达观察点",
    # 10-25 find-retry (host/task_control.py): bounded retry of a failed goal
    "navigation_retry_pending": "没能到达，已停车并清理地图，马上重试",
    "navigation_failed_after_retries": "导航重试几次仍没能到达",
    "navigation_attempts_exhausted": "导航次数用完仍没能确认目标",
    "attempts_time_exhausted": "剩余时间不够再试一次，已停止",
    "map_switching": "地图切换中，请稍候",
    "no_observation_pose_planned": "地图上没有能看到目标的停车位置（目标可能在地图范围外），没法开过去确认",
    "navigation_server_unavailable": "车上没开导航，不会自己开过去；需要人工开到目标附近",
    "navigation_busy_previous_goal_active": "上一个导航目标还没结束，这次导航没有发出",
    "navigation_input_unhealthy": "画面或定位不够新，这次导航没有发出",
    "navigation_timeout": "导航超时",
    "task_timeout": "任务超过时间上限",
    "stream_lost": "相机画面中断",
    # 10-29 (owner): a processed-frame gap warns and waits; the task goes on when frames come back
    "stream_stalled": "画面处理中断，等待恢复",
    "stream_resumed": "画面处理已恢复",
    "navigation_input_wait": "画面处理中断，等待恢复后继续前往",
    "no_recent_frame": "没有最新画面",
    "pose_invalid": "小车位置不可靠，已停止",
    "localization_invalid": "小车位置不可靠，已停止",
    "user_cancelled": "已按要求停止",
    # query_rejected / selection_rejected sentences (runner originals)
    "cancel current operation and wait for navigation to stop": "还有任务或导航在进行，请先停止",
    "recent processed input, TF and map required": "小车还没准备好（画面、位置或地图缺失）",
    "no candidate selection is pending": "当前没有待选择的候选",          # bridge-added (not in §3.5)
    "candidate is no longer available": "该候选已不在列表中",            # bridge-added (not in §3.5)
    # hub
    "duplicate": "重复的命令，已按第一次处理",
}
for _code, _who, _zh in REJECT_CODES:
    REASON_ZH.setdefault(_code, _zh)
BRIDGE_REJECT_CODES = tuple(c for c, who, _ in REJECT_CODES if who == "bridge")


def reason_zh(code):
    """Chinese text for a reason word; ``fault:`` words get a prefix; unknown words verbatim."""
    if code is None:
        return None
    code = str(code)
    if code in REASON_ZH:
        return REASON_ZH[code]
    if code.startswith("fault:"):
        return "车上出错：" + code
    return code


def class_zh(label):
    return COCO_ZH.get(label) if isinstance(label, str) else None


def object_name(label, color=None, number=None):
    """The Chinese class name (10-22: no colour, ``color`` is ignored), or None when the class is not in
    the table. 10-27: with the memory's class-local number -> 瓶子 2 (fixed for the object's life)."""
    name = class_zh(label)
    if name is not None and isinstance(number, int) and not isinstance(number, bool) and number > 0:
        return "%s %d" % (name, number)
    return name


FPV_MAX_BOXES = 32
FPV_STATES = ("ok", "stale", "none")


def fpv_ar(ar):
    """10-27 console first-person boxes: the hub's preview meta ``ar`` block (host/hub/fpv_boxes.py, the car
    screen AR's ROI rows paired to this preview frame) -> the WS preview header's ``ar``, with the Chinese
    name (object_name: class + class-local number). Malformed -> None (the picture still goes out)."""
    if not isinstance(ar, dict) or ar.get("state") not in FPV_STATES or not isinstance(ar.get("boxes"), list):
        return None
    num = lambda v: isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)  # noqa: E731
    is_int = lambda v: isinstance(v, int) and not isinstance(v, bool)  # noqa: E731
    wh = ar.get("depth_wh")
    out = {"state": ar["state"], "dtMs": ar.get("dt_ms") if num(ar.get("dt_ms")) else None,
           "frameId": ar.get("frame_id") if is_int(ar.get("frame_id")) else None,
           "depthWh": list(wh) if isinstance(wh, list) and len(wh) == 2 and all(is_int(v) and v > 0 for v in wh) else None,
           "hidden": ar.get("hidden") if is_int(ar.get("hidden")) else 0, "boxes": []}
    for b in ar["boxes"][:FPV_MAX_BOXES]:
        if not isinstance(b, dict):
            continue
        xy, v = b.get("xyxy"), b.get("v")
        if not (isinstance(xy, list) and len(xy) == 4 and all(is_int(c) and 0 <= c <= 8192 for c in xy)
                and xy[0] <= xy[2] and xy[1] <= xy[3]) or v not in (0, 2, 3):
            continue
        label = b.get("label") if isinstance(b.get("label"), str) else None
        number = b.get("number") if is_int(b.get("number")) else None
        out["boxes"].append({"id": b.get("id") if isinstance(b.get("id"), str) else None,
                             "name": object_name(label, None, number) or label, "label": label,
                             "number": number, "v": v, "xyxy": list(xy)})
    return out


def aliases(label):
    a = ALIASES_ZH.get(label) if isinstance(label, str) else None
    return list(a) if a else None


# runner Chinese vocabulary (every Chinese name / alias of RUNNER_CLASSES)
_RUNNER_WORDS = set()
for _c in RUNNER_CLASSES:
    _RUNNER_WORDS.add(COCO_ZH[_c])
    _RUNNER_WORDS.update(ALIASES_ZH.get(_c, ()))
_ZH_TO_CLASS = {}
for _c, _zh in COCO_ZH.items():
    _ZH_TO_CLASS[_zh] = _c
for _c, _al in ALIASES_ZH.items():
    for _a in _al:
        _ZH_TO_CLASS[_a] = _c
_ZH_MAXLEN = max(len(w) for w in _ZH_TO_CLASS)


def _scan_classes(text):
    """Greedy left-to-right longest match of Chinese class names: [(word, class), ...]."""
    out, i = [], 0
    while i < len(text):
        for n in range(min(_ZH_MAXLEN, len(text) - i), 0, -1):
            w = text[i:i + n]
            if w in _ZH_TO_CLASS:
                out.append((w, _ZH_TO_CLASS[w]))
                i += n
                break
        else:
            i += 1
    return out


def translate_find_text(text):
    """(text to send, note|None). Runner words or any ASCII letter -> unchanged; otherwise the
    first Chinese COCO name the runner does not know (longest match, e.g. 椅子, 酒杯, 花瓶) ->
    "<english class>" (10-22: no colour word any more, colour is not a match criterion);
    nothing recognised -> unchanged (§3.5)."""
    if any(("a" <= ch <= "z") or ("A" <= ch <= "Z") for ch in text):
        return text, None
    hits = _scan_classes(text)
    if not hits or any(w in _RUNNER_WORDS for w, _ in hits):
        return text, None
    word, cls = hits[0]
    sent = cls
    digits = re.search(r"[0-9]+", text)        # 10-27: keep the asked instance number (椅子 2 -> chair 2)
    if digits:
        sent = "%s %s" % (cls, digits.group(0))
    # D-044 (2026-10-21): runners from that date know all 80 Chinese names (coco_zh); the English form
    # still works with both old and new runners, so the translation stays and the note no longer says
    # the car only knows some Chinese names.
    return sent, "已把“%s”按英文“%s”发送" % (word, sent)


# ---------------------------------------------------------------------------- numbers

def fnum(v):
    """float(v) for a finite real number (bool excluded), else None."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    v = float(v)
    return v if math.isfinite(v) else None


def point3(v):
    """[x, y, z] from a 2- or 3-list of finite numbers, else None."""
    if not isinstance(v, (list, tuple)) or len(v) not in (2, 3):
        return None
    xs = [fnum(c) for c in v]
    if any(c is None for c in xs):
        return None
    return xs if len(xs) == 3 else xs + [0.0]


def dumps(obj):
    """Wire JSON: UTF-8, compact, non-finite numbers refused."""
    return json.dumps(obj, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def _d(v):
    return v if isinstance(v, dict) else {}


# ---------------------------------------------------------------------------- tier (§2.8)

def classify_view(state, runtime_info, memory=None):
    """{"tier": loc|sim|odom|unknown, "frame": map|odom, "frame_clamped": bool}.

    loc: runtime_info.pose_source == "estimated" or pose.loc_state present;
    sim: frame-level source == "sim"; odom: pose_source == "reference" (not sim);
    unknown (e.g. replayed recordings, --feed synthetic): frame from MEMORY world_frame."""
    state, ri = _d(state), _d(runtime_info)
    ps = ri.get("pose_source")
    loc_state = _d(state.get("pose")).get("loc_state")
    if ps == "estimated" or loc_state is not None:
        return {"tier": "loc", "frame": "map", "frame_clamped": False}
    if state.get("source") == "sim":
        return {"tier": "sim", "frame": "map", "frame_clamped": False}
    if ps == "reference":
        return {"tier": "odom", "frame": "odom", "frame_clamped": False}
    wf = _d(memory).get("world_frame")
    if wf is None:
        return {"tier": "unknown", "frame": "map", "frame_clamped": False}
    if wf in FRAMES:
        return {"tier": "unknown", "frame": wf, "frame_clamped": False}
    return {"tier": "unknown", "frame": "map", "frame_clamped": True}


def compose_robot(view, state, runtime_info, task_state, estop_latched):
    """``robot`` object or None (§2.3, §3.2)."""
    state, ri = _d(state), _d(runtime_info)
    tier = view["tier"]
    pose, odom = _d(state.get("pose")), _d(state.get("odom"))
    if tier in ("loc", "sim"):
        src = pose
        if tier == "loc":
            kind = "map" if (pose.get("loc_state") == "valid" and ri.get("pose_source") == "estimated") else "none"
        else:
            kind = "map"
        age = fnum(pose.get("age_ms"))
    elif tier == "odom":
        src, kind, age = odom, "odometry", fnum(odom.get("age_ms"))    # odom.x/y/yaw only after hub H2
    else:
        return None
    x, y, yaw = fnum(src.get("x")), fnum(src.get("y")), fnum(src.get("yaw"))
    if (x is None or y is None) and tier == "sim":
        # 10-15 第 3 棒（控制器裁定「里程计档允许寻物并提示」）：仿真档没有地图位姿（car_up sim 不发
        # /fpga/localization_pose）时退到 hub H2 的 odom.x/y/yaw，标 "odometry"（页面提示「仅里程计定位」）。
        # 仿真参考位姿下 map->odom 是恒等静态变换，odom 坐标直接落在 map 系。定位档不退：定位丢失照旧「未定位」。
        src, kind, age = odom, "odometry", fnum(odom.get("age_ms"))
        x, y, yaw = fnum(src.get("x")), fnum(src.get("y")), fnum(src.get("yaw"))
    if x is None or y is None or yaw is None:   # 浏览器校验要求 yaw 有限，缺 yaw 会整帧被拒（第 3 棒补）
        return None
    v = fnum(odom.get("v"))
    return {"position": [x, y, 0.0], "yaw": yaw, "speed": None if v is None else abs(v),
            "pose": kind, "poseAgeMs": age,
            "status": robot_status(state, task_state, estop_latched)}


# 10-25: producer frames without any valid depth for this long (hub geometry_nano.depth_invalid_s) = something
# right in front of the lens (the camera's near-range laser guard; 10-22 14:44 stopped the car for 850 s)
DEPTH_BLOCKED_S = 3.0
DEPTH_BLOCKED_ZH = "相机前方过近，请后退"


def depth_blocked(state):
    s = fnum(_d(_d(state).get("geometry_nano")).get("depth_invalid_s"))
    return s is not None and s >= DEPTH_BLOCKED_S


def robot_status(state, task_state, estop_latched):
    blocked = depth_blocked(state)
    if estop_latched:
        return "急停锁存（%s）" % DEPTH_BLOCKED_ZH if blocked else "急停锁存"
    if blocked:
        return DEPTH_BLOCKED_ZH
    if task_state in ACTIVE_TASK_STATES:
        return TASK_STATE_ZH[task_state]
    nav = _d(_d(state).get("nav"))
    v = fnum(nav.get("cmd_v"))
    if nav.get("motion_permit") is True and v is not None and abs(v) > 0.01:
        return "行驶中"
    return "待命"


# ---------------------------------------------------------------------------- task (§2.5)

RUNNER_TERMINAL = {"found": "succeeded", "not_found": "failed", "unable_to_confirm": "failed",
                   "failed": "failed", "cancelled": "cancelled", "needs_manual_approach": "failed"}
# SELECT_SEMANTICS_20261016.md section 2: the protocol ``state`` stays in the v2 set (a page from before
# 2026-10-16 rejects a whole frame whose task.state it does not know), so these two runner end states keep
# state "failed" and add an optional ``outcome`` the current page shows as its own badge; ``needs_manual_approach``
# also carries ``targetPosition`` (the remembered position, runner world frame, metres).
OUTCOME_ZH = {"unable_to_confirm": "无法确认", "needs_manual_approach": "需人工前往"}


def task_protocol_state(runner_state, reason):
    if runner_state in (None, "idle"):
        return "idle"
    if runner_state == "ambiguous":
        return "ambiguous"
    if runner_state == "navigating":
        return "searching" if reason == "search" else "navigating"
    if runner_state == "observing":
        return "confirming"
    if runner_state == "recovering":       # 10-25: stopped between two navigation attempts (not a v2 state)
        return "navigating"
    return RUNNER_TERMINAL.get(runner_state)          # None: a state this bridge does not know


class TaskIdTracker(object):
    """Synthesised task id ``<hub_boot_id>:<query_seq>`` (§2.5): moved only when the task row
    (state, reason, target_id) changes, recomputed when the hub boot changes."""

    def __init__(self):
        self.boot = None
        self.key = None
        self.id = None

    def update(self, hub_boot_id, task_group):
        g = _d(task_group)
        key = (g.get("state"), g.get("reason_task"), g.get("target_id"))
        if hub_boot_id != self.boot:
            self.boot, self.key = hub_boot_id, None
        if key != self.key:
            self.key = key
            seq = g.get("query_seq")
            if g.get("state") in (None, "idle") or hub_boot_id is None or seq is None:
                self.id = None
            else:
                self.id = "%s:%s" % (hub_boot_id, seq)
        return self.id


def compose_candidates(cands, counts=None):
    out = []
    for c in cands if isinstance(cands, list) else []:
        c = _d(c)
        cid, pos = c.get("id"), point3(c.get("position_m"))
        label = c.get("label")
        # className is required by the browser validator (§2.3/§2.5): a track without a class
        # would make the whole frame rejected, so it is dropped and counted (10-15 联调定).
        if not isinstance(cid, str) or not cid or pos is None or not isinstance(label, str) or not label:
            _count(counts, "candidate_dropped")
            continue
        color = c.get("color") if isinstance(c.get("color"), str) else None
        st = c.get("state") if c.get("state") in MEMORY_STATES else None
        out.append({"id": cid, "name": object_name(label, color, c.get("number")), "className": label, "color": color,
                    "position": pos, "state": st})
    return out


def idle_task(age_ms=None):
    return {"id": None, "state": "idle", "reasonCode": None, "message": "待命", "targetId": None,
            "ageMs": age_ms, "candidates": []}


def compose_task(task_group, task_id, counts=None):
    """``task`` object (§2.5). No task -> an idle object (never null)."""
    g = _d(task_group)
    age = fnum(g.get("age_ms"))
    rs, reason = g.get("state"), g.get("reason_task")
    pstate = task_protocol_state(rs, reason)
    if pstate == "idle":
        return idle_task(age)
    if pstate is None:
        _count(counts, "task_state_unknown")
        return {"id": task_id, "state": "idle", "reasonCode": str(rs),
                "message": ("车上任务状态未知：%s" % rs)[:1000], "targetId": None, "ageMs": age,
                "candidates": []}
    cands = compose_candidates(g.get("candidates"), counts) if pstate == "ambiguous" else []
    msg = None
    if pstate == "ambiguous" and reason == "candidate_selection_required" and cands:
        labels = set(c["className"] for c in cands)
        zh = class_zh(next(iter(labels))) if len(labels) == 1 else None
        if zh is not None:
            msg = "记忆里有 %d 个%s，请选择要找哪一个" % (len(cands), zh)
    trigger = g.get("search_trigger")
    if msg is None and pstate == "searching" and isinstance(trigger, str) and trigger in REASON_ZH:
        msg = REASON_ZH[trigger]           # 10-26: "moved away" vs "not seen there"
    if msg is None:
        if reason in REASON_ZH:
            msg = REASON_ZH[reason]
        elif reason:
            msg = "%s：%s" % (TASK_STATE_ZH[pstate], reason_zh(reason))
        else:
            msg = TASK_STATE_ZH[pstate]
    if isinstance(g.get("stream_stall"), dict) and pstate in ACTIVE_TASK_STATES:
        # 10-29: shown in front of whatever the task is doing (the task is not over, it waits for frames)
        stalled = REASON_ZH["stream_stalled"]
        msg = msg if msg.startswith(stalled) or msg == REASON_ZH["navigation_input_wait"] else stalled + "；" + msg
    target = g.get("target_id") if isinstance(g.get("target_id"), str) else None
    out = {"id": task_id, "state": pstate, "reasonCode": None if reason is None else str(reason),
           "message": msg[:1000], "targetId": target, "ageMs": age, "candidates": cands}
    # 10-25 find-retry: how many navigation attempts / distinct goals the runner has tried (optional keys)
    att = g.get("attempts")
    if isinstance(att, dict) and all(isinstance(att.get(k), int) and not isinstance(att.get(k), bool)
                                     and att.get(k) >= 0 for k in ("count", "positions")):
        out["attempts"] = att["count"]
        out["positionsTried"] = att["positions"]
    if pstate == "failed" and rs in OUTCOME_ZH:
        out["outcome"] = rs
        if rs == "needs_manual_approach":
            row = _d(g.get("target"))
            pos = point3(row.get("position_m"))
            name = object_name(row.get("label"), row.get("color"), row.get("number")) or (row.get("label") if isinstance(
                row.get("label"), str) and row.get("label") else "目标")
            where = "，记忆位置 (%.2f, %.2f, %.2f) m" % tuple(pos) if pos is not None else ""
            out["message"] = ("已在记忆里找到%s%s；车上没开导航，不会自己开过去，请人工前往" % (name, where))[:1000]
            out["targetPosition"] = pos
    return out


# ---------------------------------------------------------------------------- snapshot (§2.3)

def _count(counts, key, n=1):
    if counts is not None:
        counts[key] = counts.get(key, 0) + n


def compose_objects(memory, counts=None):
    tracks = _d(memory).get("tracks")
    out, seen = [], set()
    for t in tracks if isinstance(tracks, list) else []:
        t = _d(t)
        tid = t.get("id")
        if not isinstance(tid, str) or not tid:
            _count(counts, "object_bad_id")
            continue
        if tid in seen:
            _count(counts, "object_duplicate_id")
            continue
        pos = point3(t.get("position_m"))
        if pos is None:
            _count(counts, "object_nonfinite_position")
            continue
        label = t.get("label")
        if not isinstance(label, str) or not label:
            _count(counts, "object_no_label")      # className is required (see compose_candidates)
            continue
        seen.add(tid)
        color = t.get("color") if isinstance(t.get("color"), str) else None
        st = t.get("state")
        if st is not None and st not in MEMORY_STATES:
            _count(counts, "object_unknown_state")
        out.append({"id": tid, "name": object_name(label, color, t.get("number")), "className": label, "color": color,
                    "aliases": aliases(label), "location": None, "position": pos,
                    "confidence": fnum(_d(t.get("history_last")).get("score")),
                    "lastSeen": fnum(t.get("last_seen")),
                    "state": st if st in MEMORY_STATES else None})
    if len(out) > MAX_OBJECTS:
        _count(counts, "object_truncated", len(out) - MAX_OBJECTS)
        order = sorted(range(len(out)), key=lambda i: (out[i]["lastSeen"] is not None,
                                                        out[i]["lastSeen"] or 0.0), reverse=True)
        keep = set(order[:MAX_OBJECTS])
        out = [o for i, o in enumerate(out) if i in keep]
    return out


def compose_path(state, frame):
    p = _d(_d(_d(state).get("nav")).get("path"))
    if p.get("frame_id") != frame:
        return []
    out = []
    for pt in p.get("points") if isinstance(p.get("points"), list) else []:
        if isinstance(pt, (list, tuple)) and len(pt) >= 2:
            x, y = fnum(pt[0]), fnum(pt[1])
            if x is not None and y is not None:
                out.append([x, y, 0.0])
        if len(out) >= MAX_PATH:
            break
    return out


def server_time(memory, elapsed_s):
    """runner ROS-clock "now" estimate (§2.2) or None."""
    m = _d(memory)
    stamp = fnum(m.get("stamp_ros_s"))
    if stamp is None or fnum(elapsed_s) is None:
        return None
    age = fnum(m.get("age_ms")) or 0.0
    return stamp + (age / 1000.0) + float(elapsed_s)


def frame_source(state):
    s = _d(state).get("source")
    return s if s in SOURCES else "unknown"


# What kind of replay (§2.1 ``sourceDetail``, 10-15 integration stage 2). Copied, not imported (the
# bridge must not import host/): host/hub/hub_server.py ReplayFeeder tags its startup with this id,
# which HubCore.compose_state puts into STATE ``replay_id``; pc/fpgac.py force_replay stamps every
# recorded STATE/MEMORY with ``replay_file``. A unit test checks the copy against hub_server.py.
HUB_SYNTHETIC_REPLAY_ID = "hub-replay-feeder"
SOURCE_DETAILS = ("recording", "synthetic")


def frame_source_detail(state):
    """``"synthetic"`` | ``"recording"`` | None for one STATE. Synthetic is decided first: a
    recorded synthetic session played back again is still synthetic."""
    st = _d(state)
    if st.get("source") != "replay":
        return None
    if st.get("replay_id") == HUB_SYNTHETIC_REPLAY_ID:
        return "synthetic"
    rf = st.get("replay_file")
    if isinstance(rf, str) and rf:
        return "recording"
    return None


def compose_snapshot(state, runtime_info, memory, memory_elapsed_s, link_state, hub_age_ms, task,
                     cloud_rev, placeholder, map_id=None, map_name=None, counts=None, view=None):
    """Snapshot body without ``seq`` (the sender stamps it per connection)."""
    st = _d(state)
    view = view or classify_view(state, runtime_info, memory)
    if view.get("frame_clamped"):
        _count(counts, "frame_clamped")
    frame = view["frame"]
    pose = _d(st.get("pose"))
    mid = None
    if view["tier"] == "loc":
        mid = pose.get("map_id") if isinstance(pose.get("map_id"), str) else map_id
    estop = _d(st.get("estop")).get("latched") is True
    mem_age = fnum(_d(st.get("memory")).get("age_ms"))
    if memory is None:
        mem = {"scope": None, "revision": None, "ageMs": None}
    else:
        rbid = memory.get("runtime_boot_id")
        rev = memory.get("revision")
        mem = {"scope": rbid if isinstance(rbid, str) else None,
               "revision": rev if isinstance(rev, int) and not isinstance(rev, bool) else None,
               "ageMs": mem_age}
    return {
        "type": "snapshot", "version": VERSION,
        "serverTime": server_time(memory, memory_elapsed_s) if memory is not None else None,
        "source": frame_source(state),
        "sourceDetail": frame_source_detail(state),
        "link": {"state": link_state if link_state in ("up", "stale", "down") else "down",
                 "hubAgeMs": fnum(hub_age_ms)},
        "map": {"id": mid if view["tier"] == "loc" else None,
                "name": map_name if view["tier"] == "loc" else None,
                "frame": frame, "units": "m", "cloudRev": cloud_rev, "placeholder": bool(placeholder)},
        "robot": compose_robot(view, st, runtime_info, task.get("state"), estop),
        "estop": {"latched": _d(st.get("estop")).get("latched") if isinstance(_d(st.get("estop")).get("latched"), bool) else None,
                  "epoch": _d(st.get("estop")).get("epoch")},
        "mapping": _d(st.get("mapping")),
        "memory": mem,
        "objects": compose_objects(memory, counts) if memory is not None else [],
        "path": compose_path(st, frame),
        "task": task,
    }


def compose_fpga(state, hub_age_ms=None):
    """Optional ``fpga`` object for the kiosk page (§2.10, 10-17), or None when STATE has no ``kv`` group.

    Raw numbers only; the page decides how to show them (e.g. ROI rows older than 1 s -> "—"):
    geometryMs = kv.geometry.pl_device_ms (KV timer core_device), roiRows = kv.roi.rows (rows of the
    last ROI request, event driven: roiAgeMs says how old), heartbeatOn = kv.permit.heartbeat_on
    (STATUS b0; the PL drives the 100 Hz stop heartbeat exactly while it is 1). A block that is not
    available gives nulls. The AR refresh rate is not in STATE (hub does not forward kv.ar flips)."""
    kv = _d(state).get("kv") if isinstance(state, dict) else None
    if not isinstance(kv, dict):
        return None
    geo, roi, permit = _d(kv.get("geometry")), _d(kv.get("roi")), _d(kv.get("permit"))
    rows = roi.get("rows") if roi.get("avail") is True else None
    if isinstance(rows, bool) or not isinstance(rows, int) or rows < 0:
        rows = None
    hb = permit.get("heartbeat_on") if permit.get("avail") is True else None
    out = {"linkOk": kv.get("link_ok") is True,
           "geometryMs": fnum(geo.get("pl_device_ms")) if geo.get("avail") is True else None,
           "geometryAgeMs": fnum(geo.get("age_ms")) if geo.get("avail") is True else None,
           "roiRows": rows,
           "roiAgeMs": fnum(roi.get("age_ms")) if roi.get("avail") is True else None,
           "heartbeatOn": hb if isinstance(hb, bool) else None,
           "ageMs": fnum(kv.get("age_ms"))}
    out["live"] = compose_fpga_live(state, kv, permit, hub_age_ms)
    return out


FPGA_FRAMES_MAX_AGE_MS = 2000.0     # perf (per-frame numbers) older than this: the frames stopped -> all null
FPGA_CLOCK_MAX_AGE_MS = 2000.0      # clock sync report older than this: bound null (the hub syncs at 10 Hz)


def _nonneg_int(v):
    return v if isinstance(v, int) and not isinstance(v, bool) and v >= 0 else None


def compose_fpga_live(state, kv, permit, hub_age_ms=None):
    """10-29 console FPGA panel (display only): ``fpga.live``, every value a number from the running stack or
    null -- never an old value: per-frame numbers come from STATE perf (frame_stats of the last 10 frames) and go
    null when the newest frame is older than FPGA_FRAMES_MAX_AGE_MS; the permit goes null when the KV telemetry
    link is lost (hub kv.link_ok). ``hub_age_ms`` (how old this STATE is at the bridge) is added to every age, so
    a hub that stopped sending STATE turns everything null too.

      fps            perf.frames_per_s                         depth frames through the KV260 per second
      geoMs/pixels   perf.fpga.geo_device_ms / pixels          frame core: PL start -> done interrupt (median), pixels
      roiMs/roiBoxes perf.fpga.roi_device_ms / roi_boxes       evidence core (only frames with object boxes)
      rollMs/rollCells perf.fpga.roll_device_ms / roll_changes fusion core INTEGRATE, map cells it changed
      rttMs          perf.fpga.rtt_ms                          one frame Nano -> KV260 -> Nano (median)
      syncBoundMs    clock.bound_us / 1000                     two-board clock sync error bound
      permit         kv.permit.permit (STATUS b0)              drive permit on (the PL heartbeat runs) / off
      renewals       kv.permit.frames_accepted                 PL counter: frame done edges that renewed it
      estop          kv.permit.estop / STATE estop.latched
    """
    st = _d(state)
    perf = _d(st.get("perf"))
    f = _d(perf.get("fpga"))
    late = fnum(hub_age_ms) or 0.0
    age = fnum(perf.get("age_ms"))
    age = None if age is None else age + late
    fresh = age is not None and age <= FPGA_FRAMES_MAX_AGE_MS
    roi_n, roll_n = _nonneg_int(f.get("roi_frames")), _nonneg_int(f.get("roll_frames"))
    live = {"framesAgeMs": age,
            "fps": fnum(perf.get("frames_per_s")) if fresh else None,
            "geoMs": fnum(f.get("geo_device_ms")) if fresh else None,
            "pixels": _nonneg_int(f.get("pixels")) if fresh else None,
            "roiMs": fnum(f.get("roi_device_ms")) if fresh else None,
            "roiBoxes": _nonneg_int(f.get("roi_boxes")) if fresh else None,
            "roiFrames": roi_n if fresh else None,
            "rollMs": fnum(f.get("roll_device_ms")) if fresh else None,
            "rollCells": _nonneg_int(f.get("roll_changes")) if fresh else None,
            "rollFrames": roll_n if fresh else None,
            "frames": _nonneg_int(f.get("rows")) if fresh else None,
            "rttMs": fnum(f.get("rtt_ms")) if fresh else None,
            "syncBoundMs": None, "permit": None, "renewals": None, "estop": None}
    clk = _d(st.get("clock"))
    bound = fnum(clk.get("bound_us"))
    clk_age = fnum(clk.get("age_ms"))
    if bound is not None and bound >= 0 and clk.get("ok") is not False and not clk.get("error") \
            and clk_age is not None and clk_age + late <= FPGA_CLOCK_MAX_AGE_MS:
        live["syncBoundMs"] = bound / 1000.0
    # link_ok: a TELEM answer within the hub's limit; the STATE itself not older than the frames' limit
    if kv.get("link_ok") is True and permit.get("avail") is True and late <= FPGA_FRAMES_MAX_AGE_MS:
        p = permit.get("permit")
        live["permit"] = p if isinstance(p, bool) else None
        live["renewals"] = _nonneg_int(permit.get("frames_accepted"))
        e = permit.get("estop")
        latched = _d(st.get("estop")).get("latched")
        live["estop"] = (e is True or latched is True) if isinstance(e, bool) or isinstance(latched, bool) else None
    return live


def make_hello(bridge_id, hub_boot_id, link_state, source, control, source_detail=None):
    return {"type": "hello", "version": VERSION,
            "bridge": {"id": bridge_id, "hubBootId": hub_boot_id,
                       "link": link_state if link_state in ("up", "stale", "down") else "down"},
            "source": source if source in SOURCES + ("unknown",) else "unknown",
            "sourceDetail": source_detail if source_detail in SOURCE_DETAILS else None,
            "capabilities": {"findText": True, "findById": False, "select": True, "cancel": True,
                             "pause": False, "resume": False, "estop": True, "control": control,
                             # 10-18 ruling 4: forwarded for the electronic control interface (the kiosk stays
                             # read-only: model.js keeps only the keys it knows)
                             "teleop": True, "mappingStop": True, "goto": True, "release": True,
                             "mapReset": True}}   # 10-26 重置地图


# ---------------------------------------------------------------------------- grid -> cloud (§2.4, §3.3)

def decode_grid(meta, payload):
    """Validate a MAP body; returns a dict with ``cells`` (array('b'), row-major, -1 unknown)."""
    meta = _d(meta)
    w, h = meta.get("width"), meta.get("height")
    if not (isinstance(w, int) and isinstance(h, int)) or isinstance(w, bool) or isinstance(h, bool) \
            or w <= 0 or h <= 0 or w * h > MAX_GRID_CELLS:
        raise ValueError("bad grid size")
    res = fnum(meta.get("resolution_m"))
    if res is None or res <= 0:
        raise ValueError("bad resolution")
    origin = meta.get("origin")
    if not isinstance(origin, (list, tuple)) or len(origin) < 2:
        raise ValueError("bad origin")
    ox, oy = fnum(origin[0]), fnum(origin[1])
    oyaw = fnum(origin[2]) if len(origin) > 2 else 0.0
    if ox is None or oy is None or oyaw is None:
        raise ValueError("bad origin")
    if meta.get("encoding", "int8+zlib") != "int8+zlib":
        raise ValueError("bad encoding")
    raw = zlib.decompress(bytes(payload))
    if len(raw) != w * h:
        raise ValueError("cell count %d != %d" % (len(raw), w * h))
    cells = array.array("b")
    cells.frombytes(raw)                    # int8: bytes indexing would read -1 as 255
    return {"which": meta.get("which"), "rev": meta.get("rev"), "width": w, "height": h,
            "resolution": res, "origin": (ox, oy, oyaw), "frame_id": meta.get("frame_id"),
            "cells": cells, "crc": zlib.crc32(raw) & 0xFFFFFFFF}


def is_placeholder(cells):
    """No occupied cell anywhere (§2.8 placeholder /map)."""
    return len(cells) == 0 or max(cells) < OCCUPIED_MIN


OCC_Z = (0.05, 0.15, 0.25, 0.35)
OCC_RGB = ((60, 80, 130), (85, 115, 175), (120, 155, 210), (170, 200, 235))
FREE_RGB = (215, 218, 222)


def _sampled(grid, step):
    w, h, cells = grid["width"], grid["height"], grid["cells"]
    occ, free = [], []
    for j in range(0, h, step):
        row = j * w
        for i in range(0, w, step):
            v = cells[row + i]
            if v >= OCCUPIED_MIN:
                occ.append((i, j))
            elif 0 <= v <= FREE_MAX and i % (4 * step) == 0 and j % (4 * step) == 0:
                free.append((i, j))
    return occ, free


def grid_to_points(grid, limit=MAX_CLOUD_POINTS):
    """(xyz flat list, rgb bytearray, step). Occupied cells -> four points up to 0.35 m, free cells
    one floor point per 4x4, unknown none; the cell step doubles until <= ``limit`` points.
    Cell centres go through the grid origin (x, y, yaw)."""
    step = 1
    while True:
        occ, free = _sampled(grid, step)
        if len(occ) * len(OCC_Z) + len(free) <= limit:
            break
        step *= 2
    res = grid["resolution"]
    ox, oy, oyaw = grid["origin"]
    c, s = math.cos(oyaw), math.sin(oyaw)
    xyz, rgb = [], bytearray()

    def place(i, j):
        lx, ly = (i + 0.5) * res, (j + 0.5) * res
        return ox + c * lx - s * ly, oy + s * lx + c * ly

    for i, j in occ:
        x, y = place(i, j)
        for z, col in zip(OCC_Z, OCC_RGB):
            xyz += (x, y, z)
            rgb += bytes(col)
    for i, j in free:
        x, y = place(i, j)
        xyz += (x, y, 0.0)
        rgb += bytes(FREE_RGB)
    return xyz, rgb, step


def cloud_rev_for(grid):
    if grid["which"] == "global" and grid["rev"] is not None:
        return "global:%s" % grid["rev"]
    o = grid["origin"]
    key = struct.pack("<IIIddd", grid["crc"], grid["width"], grid["height"], o[0], o[1], o[2]) \
        + struct.pack("<d", grid["resolution"])
    return "%s:%08x" % (grid["which"] if grid["which"] in ("global", "rolling") else "grid",
                        zlib.crc32(key) & 0xFFFFFFFF)


def choose_cloud_grid(view, global_grid, rolling_grid):
    """Which grid feeds the cloud (§3.3), or None: loc/sim -> real global map in the view frame;
    odom -> rolling grid in odom; unknown -> none."""
    tier, frame = view["tier"], view["frame"]
    if tier in ("loc", "sim"):
        g = global_grid
        if g is not None and not is_placeholder(g["cells"]) and g["frame_id"] == frame:
            return g
        return None
    if tier == "odom":
        g = rolling_grid
        if g is not None and g["frame_id"] == "odom":
            return g
    return None


BELIEF_PHASES = ("preview", "search", "done", "none")
BELIEF_MAX_CELLS = 200000


def _pose3(v):
    if (isinstance(v, (list, tuple)) and len(v) == 3
            and all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v)):
        return [round(float(x), 4) for x in v]
    return None


def compose_belief(meta, payload, age_ms=None):
    """10-26 MAP which=belief (hub_core.parse_belief) -> the browser's ``belief`` text message, or ValueError.

    {"type": "belief", "version": 2, "phase": preview|search|done|none, "task", "seq", "frame", "resolution",
    "width", "height", "origin": [x, y, yaw] (lower-left corner of cell row 0 / col 0, OccupancyGrid convention),
    "cells": base64 int8 row-major (0..100 relative to the prior's max, -1 outside the searched area),
    "next": [x, y, yaw] | null, "seen": [[x, y, yaw]...], "massLeft", "result", "reason", "ageMs"}.
    phase "none": width = height = 0 (clear the layer)."""
    meta = _d(meta)
    b = _d(meta.get("belief"))
    phase = b.get("phase")
    if phase not in BELIEF_PHASES:
        raise ValueError("bad belief phase")
    w, h = meta.get("width"), meta.get("height")
    if not all(isinstance(n, int) and not isinstance(n, bool) and n >= 0 for n in (w, h)) or w * h > BELIEF_MAX_CELLS:
        raise ValueError("bad belief size")
    res = fnum(meta.get("resolution_m"))
    origin = _pose3(meta.get("origin"))
    if res is None or res <= 0 or origin is None:
        raise ValueError("bad belief resolution/origin")
    if meta.get("encoding", "int8+zlib") != "int8+zlib":
        raise ValueError("bad encoding")
    raw = zlib.decompress(bytes(payload)) if payload else b""
    if len(raw) != w * h:
        raise ValueError("belief cell count %d != %d" % (len(raw), w * h))
    seen = b.get("seen") if isinstance(b.get("seen"), list) else []
    ml = fnum(b.get("mass_left"))
    out = {"type": "belief", "version": VERSION, "phase": phase,
           "task": b.get("task") if isinstance(b.get("task"), int) else None,
           "seq": meta.get("rev") if isinstance(meta.get("rev"), int) else None,
           "frame": meta.get("frame_id") if meta.get("frame_id") in FRAMES else "map",
           "resolution": res, "width": w, "height": h, "origin": origin,
           "cells": base64.b64encode(raw).decode("ascii"),
           "next": _pose3(b.get("next")), "seen": [p for p in (_pose3(x) for x in seen[:200]) if p is not None],
           "massLeft": None if ml is None else round(ml, 4),
           "result": b.get("result") if isinstance(b.get("result"), str) else None,
           "reason": b.get("reason") if isinstance(b.get("reason"), str) else None,
           "ageMs": round(fnum(age_ms), 1) if fnum(age_ms) is not None else fnum(meta.get("age_ms"))}
    return out


def encode_cloud(map_id, rev, frame, xyz, rgb, source="grid"):
    """``u32 L | L bytes JSON header | zero pad to 4 | N*3 f32 LE | N*3 u8`` (§2.4)."""
    if len(xyz) % 3 or len(rgb) != len(xyz):
        raise ValueError("xyz/rgb length mismatch")
    n = len(xyz) // 3
    if n > MAX_CLOUD_POINTS:
        raise ValueError("too many points")
    for v in xyz:
        if not math.isfinite(v):
            raise ValueError("non-finite coordinate")
    head = dumps({"type": "cloud", "version": VERSION, "mapId": map_id, "rev": rev, "frame": frame,
                  "count": n, "source": source, "encoding": "f32xyz+u8rgb"}).encode("utf-8")
    pad = (-(4 + len(head))) % 4
    return b"".join((struct.pack("<I", len(head)), head, b"\x00" * pad,
                     struct.pack("<%df" % len(xyz), *xyz), bytes(rgb)))


def decode_cloud(buf):
    """Inverse of encode_cloud (tests, --dump-cloud checks): (header, xyz tuple, rgb bytes)."""
    (L,) = struct.unpack_from("<I", buf, 0)
    head = json.loads(buf[4:4 + L].decode("utf-8"))
    off = 4 + L + ((-(4 + L)) % 4)
    n = head["count"]
    if len(buf) != off + 15 * n:
        raise ValueError("cloud length %d != %d" % (len(buf), off + 15 * n))
    xyz = struct.unpack_from("<%df" % (3 * n), buf, off)
    return head, xyz, bytes(buf[off + 12 * n:])


def ply_bytes(xyz, rgb):
    n = len(xyz) // 3
    head = ("ply\nformat binary_little_endian 1.0\nelement vertex %d\nproperty float x\nproperty float y\n"
            "property float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n"
            "end_header\n" % n).encode("ascii")
    body = bytearray()
    for k in range(n):
        body += struct.pack("<fff", xyz[3 * k], xyz[3 * k + 1], xyz[3 * k + 2]) + bytes(rgb[3 * k:3 * k + 3])
    return head + bytes(body)


# ---------------------------------------------------------------------------- commands (§2.6)

def make_ack(cmd_id, accepted, status, code=None, note=None):
    out = {"type": "ack", "version": VERSION, "id": cmd_id, "accepted": bool(accepted), "status": status}
    if code is not None:
        out["reason"] = reason_zh(code)
        out["reasonCode"] = code
    if note:
        out["note"] = note
    return out


def make_result(cmd_id, status, code=None, detail=None):
    """``detail``: the car's own Chinese words after the code's (10-26 map_reset_failed: which step, why)."""
    reason = reason_zh(code)
    if detail and isinstance(detail, str):
        reason = (reason + "：" if reason else "") + detail[:300]
    out = {"type": "result", "version": VERSION, "id": cmd_id, "status": status,
           "reason": reason, "reasonCode": None if code is None else str(code)}
    return out


def reject(code, note=None):
    return {"kind": "reject", "code": code, "note": note}


def forward(op, args, note=None):
    return {"kind": "forward", "op": op, "args": args, "deadline_ms": DEADLINE_MS[op], "note": note}


def _num_ok(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _classify_motion(action, payload, ctx):
    """teleop / goto / mapping_stop / release (10-18 ruling 4). Shapes only: every limit (teleop |v| <= 0.30 m/s,
    |w| <= 1.0 rad/s, goto range, e-stop epoch) is the car's (hub) and the bridge neither widens nor clamps it."""
    link = ctx.get("link")
    if action == "teleop":
        if set(payload) == {"stop"} and payload["stop"] is True:
            return forward("teleop", {"stop": True})   # a stop is forwarded whatever the link (best effort)
        if set(payload) != {"v", "w"} or not (_num_ok(payload["v"]) and _num_ok(payload["w"])):
            return reject("bad_request", "遥控须为 {v, w}（有限数）或 {stop: true}")
        if link != "up":
            return reject("link_down")
        return forward("teleop", {"v": payload["v"], "w": payload["w"]})
    if action == "goto":
        keys = {"forwardM": "forward_m", "turnRad": "turn_rad"}
        if not payload or set(payload) - set(keys) or not all(_num_ok(v) for v in payload.values()):
            return reject("bad_request", "goto 须为 {forwardM, turnRad}（有限数，至少一个）")
        if link != "up":
            return reject("link_down")
        if ctx.get("source") == "inconsistent":
            return reject("source_inconsistent")
        return forward("goto", {keys[k]: v for k, v in payload.items()})
    if action == "mapping_stop":
        if payload:
            return reject("bad_request", "结束建图不带参数")
        if link != "up":
            return reject("link_down")
        return forward("mapping_stop", {})
    if action == "map_reset":             # 10-26: clears the map and the memory -> explicit confirmation, as release
        if payload.get("confirm") is not True or set(payload) - {"confirm"}:
            return reject("confirm_required", "重置地图会清空地图和记忆，请求里须带 {confirm: true}")
        if link != "up":
            return reject("link_down")
        return forward("map_reset", {})
    # release: makes the car movable again -> an explicit confirmation in the request itself
    if payload.get("confirm") is not True:
        return reject("confirm_required")
    if set(payload) - {"confirm", "epoch"}:
        return reject("bad_request", "解除急停只认 {confirm: true[, epoch: 整数]}")
    epoch = payload.get("epoch", ctx.get("estop_epoch"))   # absent: the epoch of the hub's latest STATE
    if type(epoch) is not int:
        return reject("bad_request", "解除急停须带急停编号 epoch（整数），车上状态里也没有")
    if link != "up":
        return reject("link_down")
    return forward("release", {"epoch": epoch})


def parse_command(text):
    """(cmd_id|None, msg dict|None). A body that is not a JSON object gives (None, None)."""
    try:
        msg = json.loads(text)
    except ValueError:
        return None, None
    if not isinstance(msg, dict):
        return None, None
    cid = msg.get("id")
    if not isinstance(cid, str) or not cid or len(cid) > MAX_CMD_ID:
        cid = None
    return cid, msg


def classify_command(msg, ctx):
    """Decide what to do with one browser command.

    ``ctx``: {"link": up|stale|down, "source": ..., "scope": runtime_boot_id|None,
              "task": the task object last sent in a snapshot, "estop_epoch": the hub STATE's estop.epoch|None}.
    Returns reject(code[, note]) or forward(op, args[, note])."""
    if not isinstance(msg, dict) or msg.get("type") != "command" or msg.get("version") != VERSION:
        return reject("bad_request", "type 须为 command、version 须为 2")
    cid = msg.get("id")
    if not isinstance(cid, str) or not cid or len(cid) > MAX_CMD_ID:
        return reject("bad_request", "id 须为 1-128 字的字符串")
    action, payload = msg.get("action"), msg.get("payload")
    if payload is None:
        payload = {}
    if not isinstance(payload, dict):
        return reject("bad_request", "payload 须为对象")
    if action in ("pause", "resume"):
        return reject("not_supported")
    if action == "estop":                 # stopping is always forwarded, link stale/down included (best effort)
        if payload:
            return forward("estop", {}, "急停不带参数，已忽略参数并转发")
        return forward("estop", {})
    if action in ("teleop", "goto", "mapping_stop", "release", "map_reset"):
        return _classify_motion(action, payload, ctx)
    if action == "cancel":
        task = _d(ctx.get("task"))
        tid = payload.get("taskId")
        note = None
        if tid is not None and tid != task.get("id"):
            note = "taskId 与当前任务不符，仍已转发停止"
        return forward("cancel", {}, note)
    if action not in ("find", "select"):
        return reject("bad_request", "未知 action")
    scope = ctx.get("scope")
    if action == "find":
        if "targetId" in payload:
            if "scope" in payload and payload.get("scope") != scope:
                return reject("scope_changed")
            return reject("not_supported", "暂不支持按物体发起寻物，请用文字找物")
        text = payload.get("text")
        if not isinstance(text, str) or not text.strip() or len(text) > MAX_FIND_TEXT:
            return reject("bad_request", "找物文字须为 1-64 字")
    else:
        cand = payload.get("candidateId")
        if not isinstance(cand, str) or not cand:
            return reject("bad_request", "缺 candidateId")
    if ctx.get("link") != "up":
        return reject("link_down")
    if ctx.get("source") == "inconsistent":
        return reject("source_inconsistent")
    if action == "find":
        sent, note = translate_find_text(payload["text"])
        return forward("find", {"text": sent}, note)
    task = _d(ctx.get("task"))
    if "scope" in payload and payload.get("scope") != scope:
        return reject("scope_changed")
    if payload.get("taskId") != task.get("id"):
        return reject("stale_task")
    if task.get("state") != "ambiguous":
        return reject("not_ambiguous")
    if payload["candidateId"] not in [c.get("id") for c in task.get("candidates") or []]:
        return reject("no_such_candidate")
    return forward("select", {"candidate_id": payload["candidateId"]})
