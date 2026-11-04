"""tools/car_run/e2e_find_flow.py pure functions (2026-10-26): known-answer tests."""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools", "car_run"))
import e2e_find_flow as ef  # noqa: E402


def task(state, reason="r", **kw):
    return dict({"event": "task", "state": state, "reason": reason}, **kw)


def nav(cid, reason, x=0.0, t=0.0):
    return [{"event": "command", "time": t, "command": {"kind": "navigate", "command_id": cid, "reason": reason,
                                                        "pose": {"x": x, "y": 0.0, "yaw": 0.0}}},
            {"event": "navigation_started", "time": t, "command_id": cid, "pose": {"x": x, "y": 0.0, "yaw": 0.0}}]


class TestEvents(unittest.TestCase):
    def test_read_events_slices_and_leaves_torn_line(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "runtime.jsonl")
            with open(p, "w") as f:
                f.write('{"event": "a"}\n{"event": "b"}\n\n{"event": "c"}\n{"event": "d"')
            rows, n = ef.read_events(p, 0)
            self.assertEqual([r["event"] for r in rows], ["a", "b", "c"])
            self.assertEqual(n, 4)
            rows, n2 = ef.read_events(p, 1)
            self.assertEqual([r["event"] for r in rows], ["b", "c"])
            self.assertEqual(n2, 4)
            with open(p, "a") as f:
                f.write("}\n")
            rows, n3 = ef.read_events(p, n)
            self.assertEqual([r["event"] for r in rows], ["d"])
            self.assertEqual(n3, 5)
            self.assertEqual(ef.read_events(os.path.join(d, "none.jsonl"), 3), ([], 3))

    def test_terminal_skips_ambiguous_and_navigating(self):
        rows = [task("navigating"), task("ambiguous", candidates=[{"id": "o2"}]), task("found", "x", target_id="o2")]
        self.assertEqual(ef.terminal(rows)["state"], "found")
        self.assertIsNone(ef.terminal(rows[:2]))
        self.assertEqual(ef.first_ambiguous(rows)["candidates"][0]["id"], "o2")
        self.assertEqual(ef.candidate_id({"id": "o9"}), "o9")
        self.assertEqual(ef.candidate_id("o7"), "o7")


class TestFindSlice(unittest.TestCase):
    def test_republished_previous_task_is_not_this_finds(self):
        """smoke2 10-26 find 1 (杯子): the runner republished the previous find's found/object-0001 for 0.3 s after the
        query; without the slice that row read as 'found' for the cup in 0.4 s"""
        prev = task("found", "fresh_matching_observation", target_id="object-0001")
        rows = [prev, dict(prev), {"event": "observation_plan", "target_id": "object-0003"}] + nav("c:3", "reobserve") + [
            task("navigating", "reobserve", target_id="object-0003"),
            {"event": "navigation_result", "command_id": "c:3", "succeeded": True, "cancelled": False},
            task("found", "fresh_matching_observation", target_id="object-0003")]
        self.assertEqual(ef.terminal(rows)["target_id"], "object-0001")          # the bug
        mine = ef.find_slice(rows, prev)
        self.assertEqual(mine[0]["event"], "observation_plan")
        self.assertEqual(ef.terminal(mine)["target_id"], "object-0003")
        self.assertEqual(ef.task_key(ef.last_task(rows)), ("found", "fresh_matching_observation", "object-0003"))
        # a new task with a different key starts the slice; only old rows -> nothing but rejections
        self.assertEqual(ef.find_slice([prev, task("searching", "s")], prev)[0]["state"], "searching")
        self.assertEqual(ef.find_slice([prev, {"event": "query_rejected", "reason": "x"}], prev),
                         [{"event": "query_rejected", "reason": "x"}])
        self.assertEqual(ef.find_slice([task("found", "f")], None)[0]["state"], "found")   # first find: no previous


class TestSummary(unittest.TestCase):
    def test_search_branch_with_failed_goal(self):
        rows = (nav("c:1", "search", x=0.0, t=10.0)
                + [{"event": "navigation_result", "time": 10.5, "command_id": "c:1", "succeeded": False,
                    "cancelled": False}]
                + nav("c:2", "search", x=-1.8, t=11.0)
                + [{"event": "navigation_result", "time": 40.0, "command_id": "c:2", "succeeded": True,
                    "cancelled": False},
                   {"event": "fault", "reason": "navigation_timeout"},
                   task("failed", "navigation_failed")])
        s = ef.task_summary(rows)
        self.assertEqual(s["state"], "failed")
        self.assertEqual(s["reason"], "navigation_failed")
        self.assertEqual(s["first_navigate_reason"], "search")
        self.assertEqual(s["n_navigate"], 2)
        self.assertEqual(s["nav_succeeded"], 1)
        self.assertEqual(len(s["nav_failed_goals"]), 1)
        self.assertAlmostEqual(s["nav_failed_goals"][0]["dt_s"], 0.5)
        self.assertEqual(len(s["quick_nav_rejects"]), 1)          # 0.5 s: Nav2 refused, not drove
        self.assertEqual(s["faults"], ["navigation_timeout"])
        self.assertFalse(s["observation_plan"])
        self.assertEqual(ef.classify(s, wall_x=1.0), "failed:navigation_failed")

    def test_reobserve_found(self):
        rows = ([{"event": "observation_plan"}] + nav("c:1", "reobserve", x=0.2)
                + [{"event": "navigation_result", "command_id": "c:1", "succeeded": True, "cancelled": False},
                   task("found", "fresh_matching_observation", target_id="object-0001")])
        s = ef.task_summary(rows)
        self.assertEqual((s["first_navigate_reason"], s["observation_plan"], s["target_id"]),
                         ("reobserve", True, "object-0001"))
        self.assertEqual(ef.classify(s, 1.0), "success")

    def test_classify_standin_wall_and_no_terminal(self):
        rows = nav("c:1", "reobserve", x=0.7, t=0.0) + [
            {"event": "navigation_result", "time": 30.0, "command_id": "c:1", "succeeded": False, "cancelled": False},
            task("failed", "navigation_failed")]
        s = ef.task_summary(rows)
        self.assertEqual(ef.classify(s, wall_x=1.0), "standin_wall")       # 0.7 within 1.0 - 0.45
        self.assertEqual(ef.classify(s, wall_x=2.5), "failed:navigation_failed")
        self.assertEqual(ef.classify(s, wall_x=3.0), "failed:navigation_failed")   # at the room wall: no stand-in wall
        self.assertEqual(ef.classify(ef.task_summary([task("observing")]), 1.0), "no_terminal")
        self.assertEqual(ef.classify(ef.task_summary([{"event": "query_rejected", "reason": "x"}]), 1.0),
                         "query_rejected")
        # cancelled goals are not failed goals
        rows2 = nav("c:1", "search", x=1.0) + [
            {"event": "navigation_result", "command_id": "c:1", "succeeded": False, "cancelled": True},
            task("not_found", "search_exhausted")]
        self.assertEqual(ef.classify(ef.task_summary(rows2), 1.0), "not_found:search_exhausted")

    def test_near_wall_bounds(self):
        self.assertTrue(ef.near_wall({"x": 0.55}, 1.0))
        self.assertTrue(ef.near_wall({"x": 1.65}, 1.0))
        self.assertFalse(ef.near_wall({"x": 0.54}, 1.0))
        self.assertFalse(ef.near_wall({"x": 1.66}, 1.0))
        self.assertTrue(ef.near_wall((2.2, 0.0), 2.5))
        self.assertFalse(ef.near_wall(None, 1.0))


class TestTruth(unittest.TestCase):
    def test_truth_errors_best_track_per_label(self):
        tracks = [{"id": "a", "label": "bottle", "position_m": [1.25, -0.15, 0.1], "state": "current_confirmed"},
                  {"id": "b", "label": "bottle", "position_m": [1.29, -0.42, 0.1], "state": "historical_memory"},
                  {"id": "c", "label": "cup", "position_m": [1.25, 0.4, 0.0], "state": "current_confirmed"},
                  {"id": "d", "label": "chair", "position_m": [0, 0, 0]},
                  {"id": "e", "label": "book", "position_m": None}]
        e = ef.truth_errors(tracks)
        self.assertEqual(sorted(e), ["bottle", "cup"])
        self.assertEqual(e["bottle"]["tracks"], 2)
        self.assertEqual(e["bottle"]["best_id"], "b")
        self.assertAlmostEqual(e["bottle"]["best_err_m"], 0.05, places=3)    # (0.04, 0.03) -> 0.05
        self.assertAlmostEqual(e["cup"]["best_err_m"], 0.4, places=3)
        self.assertEqual(ef.found_error(tracks, "a", "bottle"), (0.3, "bottle"))
        self.assertEqual(ef.found_error(tracks, "c", "bottle"), (0.85, "cup"))   # a cup found for 瓶子: its label says so
        self.assertEqual(ef.found_error(tracks, "zz", "bottle"), (None, None))

    def test_nav_log_window_and_aggregate(self):
        lines = ["[controller_server-1] [ERROR] [1792317700.100] [controller_server]: Failed to make progress\n",
                 "[bt_navigator-2] [WARN] [1792317800.000] [bt]: Failed to get result for follow_path\n",
                 "no stamp Goal was rejected\n"]
        self.assertEqual(ef.nav_log_issues(lines), {"Failed to make progress": 1, "Failed to get result": 1,
                                                    "Goal was rejected": 1})
        self.assertEqual(ef.nav_log_issues(lines, 1792317750.0, 1792317900.0),
                         {"Failed to get result": 1, "Goal was rejected": 1})
        agg = ef.aggregate([{"category": "success"}, {"category": "success"}, {"category": "standin_wall"},
                            {"category": "failed:navigation_failed"}])
        self.assertEqual((agg["n"], agg["success"], agg["rate"]), (4, 2, 0.5))
        self.assertEqual(agg["categories"]["standin_wall"], 1)
        self.assertEqual(ef.aggregate([])["rate"], None)
        json.dumps(agg)


if __name__ == "__main__":
    unittest.main()
