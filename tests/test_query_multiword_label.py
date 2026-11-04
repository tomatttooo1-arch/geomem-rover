"""query_memory must find multi-word COCO labels ("cell phone", D-036) by English and Chinese goal text.

Before the 2026-10-14 fix the English tokenizer split "cell phone" into {"cell", "phone"} and
there was no Chinese alias, so a stored "cell phone" track was never returned."""
import copy
import unittest

from dataclasses import replace
from perception.fpga_replay.core import Config as _Config, Memory as _Memory, UNGATED_CREATION, compute_geometry, process_frame
from perception.fpga_replay.synthetic import scene


def Memory(strategy="geometry", cfg=None):
    """core.Memory with the D-044 creation gate off (core.UNGATED_CREATION). The expectations in this file
    were written for single-sighting creation; the gate has its own tests (test_memory_creation_gate.py)."""
    return _Memory(strategy, replace(_Config() if cfg is None else cfg, **UNGATED_CREATION))


def seeded():
    m = Memory("geometry")
    frame, boxes = scene(0)
    m.update_memory(process_frame(frame, boxes), compute_geometry(frame, m), frame)
    return m


class MultiWordLabelQuery(unittest.TestCase):
    def memory_with_phone(self):
        m = seeded()
        first = m.tracks["object-0001"]
        phone = copy.deepcopy(first)
        phone.id = "object-0002"
        phone.label = "cell phone"
        phone.position[0] += 1
        m.tracks[phone.id] = phone
        return m, first

    def ids(self, m, goal):
        return [row["id"] for row in m.query_memory(goal)]

    def test_english_cell_phone(self):
        m, _ = self.memory_with_phone()
        self.assertEqual(self.ids(m, "find cell phone"), ["object-0002"])
        self.assertEqual(self.ids(m, "Where is my CELL PHONE?"), ["object-0002"])

    def test_chinese_phone(self):
        m, _ = self.memory_with_phone()
        self.assertEqual(self.ids(m, "找手机"), ["object-0002"])
        self.assertEqual(self.ids(m, "我的手机在哪"), ["object-0002"])

    def test_phone_colour_word_is_ignored(self):
        # 10-22 (owner): colour is not a match criterion any more (the colour guess was often wrong)
        m, _ = self.memory_with_phone()
        color = m.tracks["object-0002"].color
        other = "purple" if color != "purple" else "green"
        self.assertEqual(self.ids(m, "find %s cell phone" % color), ["object-0002"])
        self.assertEqual(self.ids(m, "find %s cell phone" % other), ["object-0002"])

    def test_partial_words_do_not_match(self):
        m, _ = self.memory_with_phone()
        # "cell" or "phone" alone, or the label glued to other letters, is not the label
        self.assertEqual(self.ids(m, "find cell"), [])
        self.assertEqual(self.ids(m, "find cellphone"), [])
        self.assertEqual(self.ids(m, "find microcell phones"), [])

    def test_existing_single_word_queries_unchanged(self):
        m, first = self.memory_with_phone()
        label = first.label
        self.assertEqual(self.ids(m, "find " + label), ["object-0001"])
        self.assertEqual(self.ids(m, "找大象"), [])


if __name__ == "__main__":
    unittest.main()
