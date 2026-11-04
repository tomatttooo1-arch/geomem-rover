"""Owner 10-29: a new find while the old one waits for a selection (state ambiguous) replaces it instead of being
refused with "cancel current operation" (car 10-29 23:2x: four finds refused while a search waited). The runtime's
query() is run as written (its source, against a stand-in node) up to the readiness check."""
import os
import sys
import textwrap
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)


def _query_fn():
    src = open(os.path.join(ROOT, "host", "ros_runtime.py"), encoding="utf-8").read()
    body = src.split("        def query(self, goal):\n", 1)[1]
    lines = ["        def query(self, goal):"]
    for line in body.splitlines():
        if line.startswith("        def ") or (line.strip() and not line.startswith("            ")):
            break
        lines.append(line)
    ns = {"TELEOP_REFUSAL": "teleop", "MAP_SWITCH_REFUSAL": "map_switch"}
    exec(textwrap.dedent("\n".join(lines)), ns)
    return ns["query"]


class Task:
    TERMINAL = ("found", "cancelled", "failed")

    def __init__(self, state):
        self.state, self.goal, self.reason = state, "杯子", "search_candidate_selection_required"
        self.cancelled = False

    def cancel(self, now):
        self.cancelled, self.state = True, "cancelled"
        return ["stop-cmd"]


class Node:
    def __init__(self, state, nav=None, patrol=()):
        self.task, self.nav, self.patrol = Task(state), nav, list(patrol)
        self.events, self.executed, self.stopped, self.published = [], [], 0, 0

    def record(self, event, **kw):
        self.events.append(dict(kw, event=event))

    def teleop_active(self):
        return False

    def map_switching(self):
        return None

    def active(self):
        return self.task is not None and self.task.state not in self.task.TERMINAL

    def now(self):
        return 1.0

    def execute(self, commands):
        self.executed.extend(commands)

    def stop(self):
        self.stopped += 1

    def publish_task(self):
        self.published += 1

    def ready(self):
        return False            # stop right after the busy check: reaching it is what is tested


class QueryReplaces(unittest.TestCase):
    def setUp(self):
        self.query = _query_fn()

    def test_waiting_selection_is_replaced(self):
        n = Node("ambiguous", nav={"id": "c:6"})
        self.query(n, "瓶子")
        self.assertTrue(n.task.cancelled)
        self.assertEqual((n.executed, n.stopped, n.published, n.nav), (["stop-cmd"], 1, 1, None))
        kinds = [e["event"] for e in n.events]
        self.assertEqual(kinds, ["navigation_retired", "query_replaced", "query_rejected"])
        self.assertEqual(n.events[1]["previous"]["goal"], "杯子")
        self.assertEqual(n.events[-1]["reason"], "recent processed input, TF and map required")   # past the busy check

    def test_driving_task_still_refused(self):
        n = Node("navigating", nav={"id": "c:1"})
        self.query(n, "瓶子")
        self.assertFalse(n.task.cancelled)
        self.assertEqual([e["event"] for e in n.events], ["query_rejected"])
        self.assertEqual(n.events[0]["reason"], "cancel current operation and wait for navigation to stop")

    def test_patrol_still_refused(self):
        n = Node("ambiguous", patrol=["p"])
        self.query(n, "瓶子")
        self.assertFalse(n.task.cancelled)
        self.assertEqual(n.events[0]["reason"], "cancel current operation and wait for navigation to stop")


if __name__ == "__main__":
    unittest.main()
