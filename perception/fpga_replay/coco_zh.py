"""COCO 80 class names <-> Chinese query words, for the runner's find text (D-044, 2026-10-21).

One table for the runner (perception/fpga_replay/core.py query_memory) and the bench readout
(tools/car_run/bench_line4_readout.py). No import of tools/nano_detector here (the runner must
import without it); tests/test_coco_zh.py checks that COCO_NAMES equals nano_detector.COCO and that
every Chinese name in the two interface tables (ui/geomem/app/src/model.js CLASS_ZH,
ui/geomem/bridge/gmb_proto.py COCO_ZH) resolves to the same class here.

Main names: model.js CLASS_ZH (it has 63 of the 80 classes); the other 17 from gmb_proto COCO_ZH.
Aliases: the gmb_proto names that differ from model.js, the runner's old words (杯/瓶/背包) and a
few everyday words that are not part of another class name. Bare single characters that are
pieces of several class names (车, 包, 杯 is kept because it was a runner word before) are not
added; the longest-match scan below makes 酒杯 / 剪刀 / 网球拍 / 自行车 / 书包 / 热狗 win over
their substrings.
"""
import re
from .household_names import NAMES as HOUSEHOLD_NAMES, CLASS_ZH as HOUSEHOLD_ZH, ALIASES_ZH as HOUSEHOLD_ALIASES
from .household_names import EXTRA_ZH33

COCO_NAMES = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
)

# class -> main Chinese name (model.js CLASS_ZH; the classes it lacks from gmb_proto COCO_ZH)
CLASS_ZH = {
    "person": "人", "bicycle": "自行车", "car": "汽车", "motorcycle": "摩托车", "airplane": "飞机",
    "bus": "公交车", "train": "火车", "truck": "卡车", "boat": "船", "traffic light": "红绿灯",
    "fire hydrant": "消防栓", "stop sign": "停车标志", "parking meter": "停车计时器", "bench": "长椅",
    "bird": "鸟", "cat": "猫", "dog": "狗", "horse": "马", "sheep": "羊", "cow": "牛",
    "elephant": "大象", "bear": "熊", "zebra": "斑马", "giraffe": "长颈鹿", "backpack": "书包",
    "umbrella": "雨伞", "handbag": "手提包", "tie": "领带", "suitcase": "行李箱", "frisbee": "飞盘",
    "skis": "滑雪板", "snowboard": "单板滑雪板", "sports ball": "球", "kite": "风筝",
    "baseball bat": "球棒", "baseball glove": "棒球手套", "skateboard": "滑板", "surfboard": "冲浪板",
    "tennis racket": "网球拍", "bottle": "瓶子", "wine glass": "酒杯", "cup": "杯子", "fork": "叉子",
    "knife": "刀", "spoon": "勺子", "bowl": "碗", "banana": "香蕉", "apple": "苹果",
    "sandwich": "三明治", "orange": "橙子", "broccoli": "西兰花", "carrot": "胡萝卜", "hot dog": "热狗",
    "pizza": "披萨", "donut": "甜甜圈", "cake": "蛋糕", "chair": "椅子", "couch": "沙发",
    "potted plant": "盆栽", "bed": "床", "dining table": "桌子", "toilet": "马桶", "tv": "电视",
    "laptop": "笔记本电脑", "mouse": "鼠标", "remote": "遥控器", "keyboard": "键盘",
    "cell phone": "手机", "microwave": "微波炉", "oven": "烤箱", "toaster": "烤面包机", "sink": "水槽",
    "refrigerator": "冰箱", "book": "书", "clock": "时钟", "vase": "花瓶", "scissors": "剪刀",
    "teddy bear": "玩具熊", "hair drier": "吹风机", "toothbrush": "牙刷",
}

# class -> extra Chinese words (each must not be a class name of another class)
ALIASES_ZH = {
    "baseball bat": ("棒球棍",),          # gmb_proto COCO_ZH
    "teddy bear": ("泰迪熊",),            # gmb_proto COCO_ZH
    "cup": ("杯", "水杯"),                # 杯: runner word before D-044; 水杯: kiosk name
    "bottle": ("瓶", "水瓶"),             # 瓶: runner word before D-044; 水瓶: kiosk name
    "backpack": ("背包", "双肩包"),
    "laptop": ("电脑", "笔记本"),
    "chair": ("凳子",),
    "tv": ("电视机",),
    "sports ball": ("足球", "篮球", "排球"),
    "suitcase": ("拉杆箱",),
    # D-044 revised: 书桌 / 课桌 would otherwise read as 书 (book) in 书桌上的杯子
    "dining table": ("餐桌", "书桌", "课桌", "办公桌"),
    "clock": ("闹钟", "挂钟"),
    "spoon": ("勺", "汤勺"),
    "potted plant": ("绿植",),
}

ZH_TO_CLASS = {}
for _cls, _zh in CLASS_ZH.items():
    ZH_TO_CLASS[_zh] = _cls
for _cls, _words in ALIASES_ZH.items():
    for _w in _words:
        if _w in ZH_TO_CLASS and ZH_TO_CLASS[_w] != _cls:
            raise ValueError("Chinese word %r names two classes" % _w)
        ZH_TO_CLASS[_w] = _cls

# Keep the COCO table/index contract intact; expand query vocabulary independently.
for _cls, _zh in HOUSEHOLD_ZH.items():
    ZH_TO_CLASS[_zh] = _cls
for _cls, _words in HOUSEHOLD_ALIASES.items():
    for _w in _words:
        ZH_TO_CLASS[_w] = _cls
for _cls, _zh in EXTRA_ZH33.items():
    ZH_TO_CLASS[_zh] = _cls
QUERY_NAMES = tuple(dict.fromkeys(COCO_NAMES + HOUSEHOLD_NAMES + tuple(EXTRA_ZH33)))

MULTIWORD_NAMES = tuple(n for n in COCO_NAMES if " " in n)

# D-044 revision (owner 2026-10-21): the default recognition set is the classes that can turn up at
# the defence venue (indoor classroom / lab). Board COCO order. The controller chose the names;
# yolo_detector.default_classes() and the runner's query vocabulary (Config.query_classes) use it.
# 10-22 (owner): tv taken out, 26 -> 25 classes (reason in CANCELLED_REASONS). 10-28 (owner): remote taken out too, 25 -> 24: the car no
# longer recognises remote controls. The 26-class set of
# 10-21 / 10-22 runs is VENUE_CLASSES_D044_26 (record checks of those runs compare against it).
VENUE_CLASSES = (
    "backpack", "umbrella", "handbag", "sports ball", "bottle", "wine glass", "cup", "spoon", "bowl",
    "banana", "apple", "orange", "chair", "potted plant", "dining table", "laptop", "mouse",
    "keyboard", "cell phone", "book", "clock", "vase", "scissors", "teddy bear",
)
VENUE_CLASSES_D044_26 = tuple(n for n in COCO_NAMES if n in VENUE_CLASSES or n in ("tv", "remote"))
# the 55 others and why they are off by default (--classes all turns all 80 back on)
CANCELLED_REASONS = (
    (("person",), "people walk about: new tracks, position-invalid evidence where they stood, kiosk ghosts"),
    (("bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
      "fire hydrant", "stop sign", "parking meter", "bench"), "traffic and street objects, not indoors; "
     "10-16 classroom: floor tape -> airplane, shoe / chair leg -> car, false fire hydrant"),
    (("bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe"), "animals"),
    (("tie",), "worn by the judges: a track on a person"),
    (("frisbee", "skis", "snowboard", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
      "tennis racket"), "outdoor sport"),
    (("suitcase",), "10-16 classroom: carton -> suitcase"),
    (("fork", "knife"), "not at the venue"),
    (("sandwich", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake"), "cooked food / vegetables"),
    (("couch", "bed", "toilet"), "not in a classroom"),
    (("microwave", "oven", "toaster", "sink", "refrigerator"), "kitchen / bathroom appliances; "
     "10-16 classroom: carton -> microwave, table leg -> oven / refrigerator"),
    (("hair drier", "toothbrush"), "bathroom"),
    (("remote",), "10-28 owner: remote controls are no longer recognised"),
    (("tv",), "10-22 owner: the venue's monitors, projector screens and laptop screens read as tv, "
     "and a tv is not something anyone asks the car to find"),
)


def scan_words(text, words):
    """Longest-first, non-overlapping matches of ``words`` (str -> value) in ``text``:
    [(start, end, word, value)] in text order. A longer word claims its span before any word
    it contains (酒杯 before 杯, 剪刀 before 刀, 自行车 before nothing shorter)."""
    spans = []
    for word in sorted(words, key=len, reverse=True):
        start = 0
        while True:
            start = text.find(word, start)
            if start < 0:
                break
            end = start + len(word)
            if not any(start < e and end > s for s, e, _, _ in spans):
                spans.append((start, end, word, words[word]))
            start = end
    return sorted(spans)


def english_labels(text):
    """COCO names written in English in ``text`` (lower-cased): whole single words and whole-word
    multi-word names ("cell phone", "teddy bear"), longest first and non-overlapping (so "hot dog"
    is not also "dog"), [(start, label)] in text order."""
    text = text.lower()
    spans = []
    for name in sorted(QUERY_NAMES, key=len, reverse=True):
        for m in re.finditer(r"(?<![a-z])" + re.escape(name) + r"(?![a-z])", text):
            if not any(m.start() < e and m.end() > s for s, e, _ in spans):
                spans.append((m.start(), m.end(), name))
    return sorted((s, name) for s, _, name in spans)


def labels_in_text(text):
    """COCO labels named in ``text`` (Chinese words via the longest-first scan, then English
    names), in order of first appearance, without repeats."""
    hits = [(s, v) for s, _, _, v in scan_words(text, ZH_TO_CLASS)] + english_labels(text)
    seen, out = set(), []
    for _, label in sorted(hits, key=lambda h: h[0]):
        if label not in seen:
            seen.add(label)
            out.append(label)
    return out


# words that join two named classes into one request (瓶子和杯子 = both); anything else between two
# class words (上的 / 旁边的 / 里的) makes the later one the target: Chinese puts the head noun last
CONNECTORS = ("和", "或", "、", "与", "跟", ",", "，", "and", "or")
_COLOUR_ZH = ("红色", "蓝色", "绿色", "黄色", "白色", "黑色", "灰色", "紫色", "red", "blue", "green", "yellow",
              "white", "black", "gray", "purple")
_FILLER = re.compile(r"^(一|两|几)?(个|只|本|把|台|部|件|张|瓶|杯)?(的)?$")


def _joined(gap):
    """True when the text between two class words is a connector, optionally followed by a measure
    word / colour / 的 (瓶子和红色杯子, 书和一个书包), and nothing else (椅子跟前的书包 is not joined)."""
    gap = gap.strip().lower()
    for c in sorted(CONNECTORS, key=len, reverse=True):
        if gap.startswith(c):
            rest = gap[len(c):].strip()
            for colour in _COLOUR_ZH:
                rest = rest.replace(colour, "")
            return bool(_FILLER.match(rest.strip()))
    return False


def goal_labels(text, allowed, extra_phrases=()):
    """The class(es) a find text asks for, among ``allowed`` (the enabled classes plus the labels of
    existing tracks). Class words of other classes are dropped after the longest-first scan (so 马上
    stays nothing, and 球棒 does not fall back to 球). With several class words, the last one is the
    target (桌子上的杯子 -> cup); earlier ones count only when joined to it by a connector
    (瓶子和杯子 -> bottle, cup). Returns a set."""
    text = text.lower()
    allowed = set(allowed)
    hits = [(s, e, v) for s, e, _, v in scan_words(text, ZH_TO_CLASS)]
    taken = [(s, e) for s, e, _ in hits]
    for name in sorted(set(QUERY_NAMES) | set(extra_phrases), key=len, reverse=True):
        for m in re.finditer(r"(?<![a-z])" + re.escape(name) + r"(?![a-z])", text):
            if not any(m.start() < e and m.end() > s for s, e in taken):
                hits.append((m.start(), m.end(), name))
                taken.append((m.start(), m.end()))
    hits = sorted(h for h in hits if h[2] in allowed)
    if not hits:
        return set()
    out = {hits[-1][2]}
    for i in range(len(hits) - 1, 0, -1):
        if not _joined(text[hits[i - 1][1]:hits[i][0]]):
            break
        out.add(hits[i - 1][2])
    return out
