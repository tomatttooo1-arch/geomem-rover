"""D-044 (2026-10-21): the runner's Chinese query vocabulary covers all 80 COCO classes.

perception/fpga_replay/coco_zh.py against the board's class list, the two interface tables, the
runner's query_memory and the bench readout's label guard."""
import copy
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from perception.fpga_replay import coco_zh  # noqa: E402
from dataclasses import replace  # noqa: E402
from perception.fpga_replay.core import (Config as _Config, Memory as _Memory, UNGATED_CREATION,  # noqa: E402
                                        compute_geometry, process_frame)
from perception.fpga_replay.synthetic import scene  # noqa: E402
from tools.car_run import bench_line4_readout as RO  # noqa: E402
from tools.nano_detector.nano_detector import COCO  # noqa: E402
sys.path.insert(0, str(ROOT / "ui" / "geomem" / "bridge"))
import gmb_proto  # noqa: E402


def Memory(strategy="geometry", cfg=None):
    """core.Memory with the D-044 creation gate off (core.UNGATED_CREATION). The expectations in this file
    were written for single-sighting creation; the gate has its own tests (test_memory_creation_gate.py)."""
    return _Memory(strategy, replace(_Config() if cfg is None else cfg, **UNGATED_CREATION))


def model_js_class_zh():
    text = (ROOT / "ui/geomem/app/src/model.js").read_text(encoding="utf-8")
    block = re.search(r"export const CLASS_ZH = \{(.*?)\};", text, re.S).group(1)
    return {(q or b): zh for q, b, zh in re.findall(r"(?:'([^']+)'|([A-Za-z_]+))\s*:\s*'([^']+)'", block)}


class Tables(unittest.TestCase):
    def test_same_80_classes_as_the_board(self):
        self.assertEqual(list(coco_zh.COCO_NAMES), list(COCO))
        self.assertEqual(set(coco_zh.CLASS_ZH), set(COCO))
        self.assertEqual(set(coco_zh.ALIASES_ZH) - set(COCO), set())

    def test_interface_tables_resolve_to_the_same_class(self):
        js = model_js_class_zh()
        self.assertEqual(set(js), set(COCO) | set(coco_zh.HOUSEHOLD_NAMES))
        for table in (js, gmb_proto.COCO_ZH):
            for cls, zh in table.items():
                self.assertEqual(coco_zh.ZH_TO_CLASS.get(zh), cls, (cls, zh))
                self.assertEqual(coco_zh.labels_in_text(zh), [cls], (cls, zh))
        for cls, zh in js.items():                               # model.js names are the main names
            self.assertEqual((coco_zh.CLASS_ZH if cls in coco_zh.CLASS_ZH else coco_zh.HOUSEHOLD_ZH)[cls], zh)

    def test_every_word_alone_is_its_class(self):
        for word, cls in coco_zh.ZH_TO_CLASS.items():
            self.assertEqual(coco_zh.labels_in_text(word), [cls], word)
        for name in COCO:
            self.assertEqual(coco_zh.labels_in_text("find the " + name), [name], name)


class Collisions(unittest.TestCase):
    CASES = [
        ("杯", ["cup"]), ("水杯", ["cup"]), ("酒杯", ["wine glass"]), ("杯子和酒杯", ["cup", "wine glass"]),
        ("刀", ["knife"]), ("剪刀", ["scissors"]),
        ("球", ["sports ball"]), ("球棒", ["baseball bat"]), ("网球拍", ["tennis racket"]),
        ("棒球手套", ["baseball glove"]),
        ("车", []), ("汽车", ["car"]), ("自行车", ["bicycle"]), ("摩托车", ["motorcycle"]),
        ("公交车", ["bus"]), ("卡车", ["truck"]), ("火车", ["train"]),
        ("包", []), ("书包", ["backpack"]), ("手提包", ["handbag"]), ("背包", ["backpack"]),
        ("书", ["book"]), ("一本书和一个书包", ["book", "backpack"]),
        ("热狗", ["hot dog"]), ("狗", ["dog"]),
        ("停车标志", ["stop sign"]), ("停车计时器", ["parking meter"]),
        ("马桶", ["toilet"]), ("斑马", ["zebra"]), ("马", ["horse"]),
        ("滑板", ["skateboard"]), ("滑雪板", ["skis"]), ("单板滑雪板", ["snowboard"]),
        ("玩具熊", ["teddy bear"]), ("泰迪熊", ["teddy bear"]), ("熊", ["bear"]),
        ("瓶", ["bottle"]), ("花瓶", ["vase"]), ("白花瓶", ["vase"]),
        ("笔记本电脑", ["laptop"]), ("电脑", ["laptop"]), ("长椅", ["bench"]), ("椅子", ["chair"]),
        ("橙色杯子", ["cup"]), ("橙子", ["orange"]),
    ]

    def test_longest_match_wins(self):
        for text, want in self.CASES:
            self.assertEqual(coco_zh.labels_in_text(text), want, text)

    def test_desk_is_a_table_not_a_book(self):
        # D-044 revised: 书桌 / 课桌 are table words, so 书 no longer fires inside them
        self.assertEqual(coco_zh.labels_in_text("书桌上的杯子"), ["dining table", "cup"])
        self.assertEqual(coco_zh.labels_in_text("课桌"), ["dining table"])


class VenueClasses(unittest.TestCase):
    CANCELLED = ("person bicycle car motorcycle airplane bus train truck boat traffic_light fire_hydrant stop_sign "
                 "parking_meter bench bird cat dog horse sheep cow elephant bear zebra giraffe tie frisbee skis "
                 "snowboard kite baseball_bat baseball_glove skateboard surfboard tennis_racket suitcase fork knife "
                 "sandwich broccoli carrot hot_dog pizza donut cake couch bed toilet microwave oven toaster sink "
                 "refrigerator hair_drier toothbrush tv")       # tv: 10-22 owner (screens read as tv)

    def test_25_venue_names_are_board_coco_names(self):
        venue = coco_zh.VENUE_CLASSES
        self.assertEqual(len(venue), 25)
        self.assertEqual(len(set(venue)), 25)
        self.assertNotIn("tv", venue)
        self.assertEqual([n for n in venue if n not in COCO], [])
        self.assertEqual(list(venue), [n for n in COCO if n in venue])          # board order
        cancelled = [n.replace("_", " ") for n in self.CANCELLED.split()]
        self.assertEqual(len(cancelled), 55)
        self.assertEqual(sorted(set(COCO) - set(venue)), sorted(cancelled))
        reasons = [n for names, _ in coco_zh.CANCELLED_REASONS for n in names]
        self.assertEqual(sorted(reasons), sorted(cancelled))
        # the 10-21 / 10-22 set, kept for record checks of those runs
        old = coco_zh.VENUE_CLASSES_D044_26
        self.assertEqual(len(old), 26)
        self.assertEqual(sorted(set(old) - set(venue)), ["tv"])
        self.assertEqual(list(old), [n for n in COCO if n in old])

    def test_tv_words_find_nothing_by_default(self):
        # 电视 / 电视机 still name the tv class (so they cannot fall back to another class), but tv is not
        # enabled: no target unless a tv track exists (e.g. a memory file from before 10-22)
        V = set(coco_zh.VENUE_CLASSES)
        self.assertEqual(coco_zh.goal_labels("找电视", V), set())
        self.assertEqual(coco_zh.goal_labels("电视机旁边的书包", V), {"backpack"})
        self.assertEqual(coco_zh.goal_labels("找电视", V | {"tv"}), {"tv"})


class GoalLabels(unittest.TestCase):
    """The head noun (last class word) is the target; 和/或/、/与/跟/, join a union."""
    V = set(coco_zh.VENUE_CLASSES)

    def test_head_noun_and_connectors(self):
        cases = [("找桌子上的杯子", {"cup"}), ("椅子旁边的书包", {"backpack"}), ("瓶子和杯子", {"bottle", "cup"}),
                 ("红色杯子", {"cup"}), ("书桌上的杯子", {"cup"}), ("瓶子、杯子或书包", {"bottle", "cup", "backpack"}),
                 ("瓶子和红色杯子", {"bottle", "cup"}), ("一本书和一个书包", {"book", "backpack"}),
                 ("椅子跟前的书包", {"backpack"}), ("桌子和椅子之间的书包", {"backpack"}),
                 ("find the cup on the dining table", {"dining table"}), ("bottle and cup", {"bottle", "cup"}),
                 ("找杯子", {"cup"}), ("你好", set())]
        for text, want in cases:
            self.assertEqual(coco_zh.goal_labels(text, self.V), want, text)

    def test_cancelled_class_words_are_ignored(self):
        self.assertEqual(coco_zh.goal_labels("马上找杯子", self.V), {"cup"})
        self.assertEqual(coco_zh.goal_labels("马上", self.V), set())
        self.assertEqual(coco_zh.goal_labels("找刀", self.V), set())
        self.assertEqual(coco_zh.goal_labels("球棒", self.V), set())                  # not a ball
        self.assertEqual(coco_zh.goal_labels("马上找马", self.V | {"horse"}), {"horse"})   # a horse track exists

    def test_single_character_collisions_among_venue_classes(self):
        cases = [("杯", {"cup"}), ("酒杯", {"wine glass"}), ("杯子和酒杯", {"cup", "wine glass"}),
                 ("书", {"book"}), ("书包", {"backpack"}), ("包", set()), ("手提包", {"handbag"}),
                 ("球", {"sports ball"}), ("足球", {"sports ball"}), ("瓶", {"bottle"}), ("花瓶", {"vase"}),
                 ("钟", set()), ("闹钟", {"clock"}), ("勺", {"spoon"}), ("碗", {"bowl"})]
        for text, want in cases:
            self.assertEqual(coco_zh.goal_labels(text, self.V), want, text)


def seeded():
    m = Memory("geometry")
    frame, boxes = scene(0)
    m.update_memory(process_frame(frame, boxes), compute_geometry(frame, m), frame)
    return m


class RunnerQuery(unittest.TestCase):
    def memory(self, labels):
        m = seeded()
        first = m.tracks["object-0001"]
        for i, label in enumerate(labels):
            t = copy.deepcopy(first)
            t.id = "object-1%03d" % i
            t.label = label
            t.position[0] += 1 + i
            m.tracks[t.id] = t
        return m

    def labels(self, m, goal):
        return sorted({row["label"] for row in m.query_memory(goal)})

    def test_new_classes_by_chinese_and_english(self):
        m = self.memory(["chair", "wine glass", "scissors", "teddy bear", "laptop", "knife", "cup"])
        self.assertEqual(self.labels(m, "找椅子"), ["chair"])
        self.assertEqual(self.labels(m, "凳子在哪"), ["chair"])
        self.assertEqual(self.labels(m, "找酒杯"), ["wine glass"])
        self.assertEqual(self.labels(m, "找剪刀"), ["scissors"])
        self.assertEqual(self.labels(m, "找刀"), ["knife"])
        self.assertEqual(self.labels(m, "我的电脑"), ["laptop"])
        self.assertEqual(self.labels(m, "find the teddy bear"), ["teddy bear"])   # multi-word, no track needed
        self.assertEqual(self.labels(m, "找杯子"), ["cup"])
        self.assertEqual(self.labels(m, "你好"), [])

    def test_head_noun_query_and_horse_is_not_soon(self):
        m = self.memory(["cup", "dining table", "bottle", "backpack", "chair"])
        self.assertEqual(self.labels(m, "找桌子上的杯子"), ["cup"])
        self.assertEqual(self.labels(m, "椅子旁边的书包"), ["backpack"])
        self.assertEqual(self.labels(m, "瓶子和杯子"), ["bottle", "cup"])
        self.assertEqual(self.labels(m, "马上找杯子"), ["cup"])
        self.assertEqual(m.tracks["object-0001"].color, "red")        # synthetic scene(0): a red cup
        self.assertEqual(self.labels(m, "红色杯子"), ["cup"])
        self.assertEqual(self.labels(m, "蓝色杯子"), ["cup"])     # 10-22: colour words do not filter

    def test_multiword_phrase_matches_without_a_track(self):
        m = seeded()
        self.assertEqual(m.query_memory("find the wine glass"), [])   # no track: empty, but no crash
        m = self.memory(["wine glass"])
        self.assertEqual(self.labels(m, "find the wine glass"), ["wine glass"])
        self.assertEqual(self.labels(m, "find the glass"), [])


class BenchGuard(unittest.TestCase):
    def test_any_coco_class_is_a_label(self):
        self.assertEqual(RO.label_of_text("瓶子"), "bottle")
        self.assertEqual(RO.label_of_text("找手机"), "cell phone")
        self.assertEqual(RO.label_of_text("椅子"), "chair")
        self.assertEqual(RO.label_of_text("酒杯"), "wine glass")
        self.assertEqual(RO.label_of_text("find the cell phone"), "cell phone")
        self.assertEqual(RO.label_of_text("你好"), None)
        self.assertEqual(set(RO.COCO), set(COCO))


if __name__ == "__main__":
    unittest.main()
