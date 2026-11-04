"""10-26 search heat map on the console: runner timing (preview / search <= 1 Hz / done), the hub's MAP which=belief,
the bridge's browser message and its cell coordinates.

    /usr/bin/python3 -m pytest tests/test_search_heatmap_20261026.py -q
"""

import base64
import json
import math
import os
import sys
import threading
import types
import unittest
import zlib

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))
sys.path.insert(0, os.path.join(ROOT, "tools", "car_run"))
sys.path.insert(0, os.path.join(ROOT, "ui", "geomem", "bridge"))

from host.search_belief import BeliefSearch  # noqa: E402
from host.search_belief_feed import BeliefFeed, dumps, preview_cells  # noqa: E402
from host.task_control import PoseGoal, TaskConfig, TaskController  # noqa: E402
from host.hub.hub_core import HubCore, NS, parse_belief  # noqa: E402
from perception.fpga_replay.core import Config, Memory, UNGATED_CREATION, compute_geometry, process_frame  # noqa: E402
from test_search_prob_20261026 import CORRIDOR, Stamp, render, room  # noqa: E402
import gmb_proto as gp  # noqa: E402
import geomem_bridge as gb  # noqa: E402
from dataclasses import replace  # noqa: E402


def cells_of(msg):
    raw = base64.b64decode(msg["cells"])
    return np.frombuffer(raw, np.int8).reshape(msg["height"], msg["width"])


def moved_cup_run(feed):
    """The test_search_prob moved-cup scenario (remembered at (2, 0), now at (2, 1.2)) with the feed ticked after
    every task update like the runner's publish_task. Returns (task, planner)."""
    mem = Memory("geometry", replace(Config(), **UNGATED_CREATION))

    def step(i, t, pose, cup):
        frame, dets = render(i, t, pose, cup)
        mem.update_memory(process_frame(frame, dets), compute_geometry(frame, mem), frame)
        return frame

    step(0, 0.0, (0.5, 0.0, 0.0), (2.0, 0.0))
    cup = (2.0, 1.2)
    planner = BeliefSearch(room(), CORRIDOR, robot=(0.5, 0.0, 0.0))
    task = TaskController(mem, lambda row: [PoseGoal(0.8, 0.0, 0.0)], (), config=TaskConfig(), search_planner=planner)
    t, i = 0.1, 1
    cmds = task.start("cup", t, Stamp("s", t))
    feed.tick(task, t)
    for _ in range(12):
        cmd = cmds[0]
        pose = (cmd.pose.x, cmd.pose.y, cmd.pose.yaw)
        t += 1.0
        task.navigation_result(cmd.command_id, True, t)
        feed.tick(task, t)
        cmds = []
        for _ in range(20):
            t += 0.2
            i += 1
            frame = step(i, t, pose, cup)
            planner.observe(frame.T_world_camera, t, base_pose=pose)
            cmds = task.observe(frame, t, robot_xy=pose[:2]) or task.tick(t)
            feed.tick(task, t)
            if cmds or task.state in task.TERMINAL:
                break
        if task.state in task.TERMINAL:
            break
    return task, planner


class FeedTimingTests(unittest.TestCase):
    def test_moved_cup_preview_search_done(self):
        sent = []
        feed = BeliefFeed(sent.append)
        task, planner = moved_cup_run(feed)
        self.assertEqual(task.state, "found")
        phases = [m["phase"] for m in sent]
        self.assertEqual(phases[0], "preview")
        self.assertEqual(phases.count("preview"), 1)
        self.assertEqual(phases[-1], "done")
        self.assertEqual(phases.count("done"), 1)
        self.assertGreaterEqual(phases.count("search"), 2)
        self.assertEqual(sent[-1]["result"], "found")
        # nothing more for this task
        self.assertIsNone(feed.tick(task, 1e6))
        # preview: the prior around the remembered (2, 0), peak 100; its peak is where the search later started
        pre = sent[0]
        self.assertEqual((pre["next"], pre["seen"], pre["mass_left"]), (None, [], 1.0))
        g = cells_of(pre)
        self.assertEqual(g.max(), 100)
        r, c = np.unravel_index(int(np.argmax(g)), g.shape)
        x = pre["origin"][0] + (c + .5) * pre["resolution"]
        y = pre["origin"][1] + (r + .5) * pre["resolution"]
        self.assertLess(math.hypot(x - 2.0, y - 0.0), 0.5)
        # search messages: <= 1 Hz unless a new look was chosen; carry the look being driven to
        search = [(i, m) for i, m in enumerate(sent) if m["phase"] == "search"]
        self.assertIsNotNone(search[0][1]["next"])
        # the belief only falls (relative to the fixed prior max): a looked-at area fades
        first, last = cells_of(search[0][1]).astype(int), cells_of(sent[-1]).astype(int)
        inside = first >= 0
        self.assertTrue(np.all(last[inside] <= first[inside]))
        self.assertLess(last[inside].sum(), first[inside].sum())
        for m in sent:
            json.loads(dumps(m))

    def test_rate_limit_and_unchanged(self):
        p = BeliefSearch(room(), CORRIDOR, robot=(0.0, 0.0, 0.0))
        task = types.SimpleNamespace(search_planner=p, state="searching", reason="search", TERMINAL=TaskController.TERMINAL,
                                     _target_row=None)
        sent = []
        feed = BeliefFeed(sent.append)
        p.begin(None)
        p.next_goal()
        self.assertEqual(feed.tick(task, 0.0)["phase"], "search")       # no memory: search from the start
        self.assertIsNone(feed.tick(task, 0.1))                          # unchanged
        p.apply_view(1.0, 0.0, 0.0)
        self.assertIsNone(feed.tick(task, 0.5))                          # changed, but < 1 s
        self.assertEqual(feed.tick(task, 1.05)["phase"], "search")
        p.apply_view(1.5, 0.0, 0.0)
        p.next_goal()                                                    # a new look: at once
        m = feed.tick(task, 1.2)
        self.assertEqual((m["phase"], len(m["seen"])), ("search", 1))
        task.state = "not_found"
        self.assertEqual(feed.tick(task, 1.3)["result"], "not_found")
        self.assertIsNone(feed.tick(task, 5.0))
        self.assertEqual([x["seq"] for x in sent], [1, 2, 3, 4])

    def test_new_task_without_planner_clears_and_preview_keeps_planner_state(self):
        sent = []
        feed = BeliefFeed(sent.append)
        mvp = TaskController(Memory("geometry"), lambda row: [], (PoseGoal(5, 0, 0),))
        mvp.start("cup", 0, Stamp("f0", 0))
        m = feed.tick(mvp, 0.0)
        self.assertEqual((m["phase"], m["width"], m["cells"], m["task"]), ("none", 0, "", 1))
        self.assertIsNone(feed.tick(mvp, 1.0))
        p = BeliefSearch(room(), CORRIDOR, robot=(0.0, 0.0, 0.0))
        before = (p.prior, p.target_xy)
        meta, cells = preview_cells(p, (2.0, 0.0))
        self.assertEqual((p.prior, p.target_xy), before)                 # the real search starts as before
        self.assertEqual(cells.max(), 100)
        # a planner that is dropped mid-task (fault -> MVP list): the page is cleared once
        t2 =types.SimpleNamespace(search_planner=p, state="searching", reason="search",
                                   TERMINAL=TaskController.TERMINAL, _target_row=None)
        p.begin(None)
        self.assertEqual(feed.tick(t2, 2.0)["phase"], "search")
        t2.search_planner = None
        self.assertEqual(feed.tick(t2, 2.1)["phase"], "none")
        self.assertIsNone(feed.tick(t2, 9.0))

    def test_size_and_build_time(self):
        """Report numbers: message bytes and build time on a 10 x 8 m room (the brief asks for them)."""
        cells = np.full((160, 200), -1, np.int8)
        cells[2:-2, 2:-2] = 0
        cells[2:4, :] = cells[-4:-2, :] = cells[:, 2:4] = cells[:, -4:-2] = 100
        fm = types.SimpleNamespace(cells=cells, resolution=0.05, origin_x=-0.5, origin_y=-4.0, origin_yaw=0.)
        nodes = [(x * 0.5, y, 0.) for y in (-2.5, 0., 2.5) for x in range(19)]
        p = BeliefSearch(fm, nodes, robot=(0., 0., 0.))
        p.begin((4.0, 1.0))
        p.next_goal()
        task = types.SimpleNamespace(search_planner=p, state="searching", reason="search",
                                     TERMINAL=TaskController.TERMINAL, _target_row=None)
        import time
        feed = BeliefFeed(lambda m: None)
        ms = []
        for k in range(5):
            p.apply_view(1.0 + k, 0.0, 0.0)
            p.next_goal()
            t0 = time.perf_counter()
            m = feed.tick(task, float(k * 2))
            text = dumps(m)
            ms.append((time.perf_counter() - t0) * 1e3)
        sys.stderr.write("\nheatmap message: %d x %d cells, %d bytes, tick+dumps %.2f ms max\n"
                         % (m["width"], m["height"], len(text), max(ms)))
        self.assertLess(len(text), 20000)


class HubTests(unittest.TestCase):
    def msg(self, phase="search", w=3, h=2, seq=1):
        cells = np.array([[-1, 0, 50], [100, 20, -1]], np.int8)[:h, :w]
        return dumps({"v": 1, "seq": seq, "task": 2, "phase": phase, "frame": "map", "resolution": 0.2,
                      "width": w, "height": h, "origin": [1.0, -2.0, 0.0], "encoding": "int8_b64",
                      "cells": base64.b64encode(cells.tobytes()).decode(), "mass_left": 0.8,
                      "next": [1.5, 0.0, 1.57], "seen": [[0.0, 0.0, 0.0]], "result": None, "reason": None,
                      "views": 4, "build_ms": 0.5})

    def test_parse_feed_frames(self):
        meta, cells = parse_belief(self.msg())
        self.assertEqual((meta["width"], meta["height"], meta["origin"], meta["frame_id"]), (3, 2, [1.0, -2.0, 0.0], "map"))
        self.assertEqual(list(np.frombuffer(cells, np.int8)), [-1, 0, 50, 100, 20, -1])
        core = HubCore("live")
        core.feed_belief(self.msg(), 1 * NS)
        frames = core.map_frames(1 * NS)
        self.assertEqual([m["which"] for m, _ in frames], ["belief"])
        m, payload = frames[0]
        self.assertEqual((m["belief"]["phase"], m["belief"]["next"], m["encoding"]), ("search", [1.5, 0.0, 1.57], "int8+zlib"))
        self.assertEqual(zlib.decompress(payload), cells)
        core.feed_belief(self.msg(seq=2), 1 * NS + NS // 10)
        self.assertEqual(core.map_frames(1 * NS + NS // 10), [])         # <= 4 Hz: waits, not lost
        self.assertEqual(len(core.map_frames(1 * NS + NS // 2)), 1)
        self.assertEqual(core.map_frames(2 * NS), [])                     # unchanged
        for bad in ("{", json.dumps({"v": 1, "phase": "x"}), self.msg().replace('"width":3', '"width":4')):
            core.feed_belief(bad, 3 * NS)
        self.assertEqual(core.counts["bad_input:belief"], 3)
        # "none" (clear): no cells
        meta, cells = parse_belief(dumps({"v": 1, "seq": 9, "task": 3, "phase": "none", "frame": "map",
                                          "resolution": 0.2, "width": 0, "height": 0, "origin": [0, 0, 0],
                                          "encoding": "int8_b64", "cells": ""}))
        self.assertEqual((meta["width"], cells, meta["belief"]["phase"]), (0, b"", "none"))

    def test_server_slot_and_ros_subscription(self):
        from host.hub import hub_server, hub_ros
        self.assertIn("MAP:belief", hub_server.LATEST_ORDER)       # else the slot is never sent (sender spins)
        self.assertIn("/fpga/search/belief", hub_ros.SUBSCRIPTIONS)
        subs = {}

        class Node(object):
            def __init__(self, name, **kw):
                pass

            def create_subscription(self, kind, topic, cb, qos, **kw):
                subs[topic] = (cb, qos)

            def create_publisher(self, kind, topic, qos, **kw):
                return types.SimpleNamespace(publish=lambda m: None)

        m = types.SimpleNamespace(Node=Node, QoSProfile=lambda **kw: kw,
                                  DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1),
                                  ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2),
                                  String=object, Empty=object, Bool=object, Twist=object,
                                  PoseWithCovarianceStamped=object, OccupancyGrid=object, Odometry=object, Path=object,
                                  CompressedImage=object, SubscriptionEventCallbacks=lambda **kw: None,
                                  PublisherEventCallbacks=lambda **kw: None)
        core = HubCore("live")
        hub_ros.build_node(types.SimpleNamespace(core=core, lock=threading.RLock()), m, clock=lambda: 5 * NS)
        cb, qos = subs["/fpga/search/belief"]
        self.assertEqual(qos["durability"], 1)                       # latched like the runner's publisher
        cb(types.SimpleNamespace(data=self.msg()))
        self.assertEqual(core.maps["belief"]["meta"]["belief"]["phase"], "search")


class FakeSession(object):
    def __init__(self):
        self.events = []

    def __getattr__(self, name):
        if name.startswith("put_"):
            return lambda obj, *a, **kw: self.events.append((name[4:], obj))
        raise AttributeError(name)


class BridgeTests(unittest.TestCase):
    def frames_from(self, msg_text, now_ns=1 * NS):
        core = HubCore("live")
        core.feed_belief(msg_text, now_ns)
        return core.map_frames(now_ns + NS // 5)

    def test_compose_and_coordinates(self):
        p = BeliefSearch(room(), CORRIDOR, robot=(0.0, 0.0, 0.0))
        p.begin((2.0, 0.0))
        p.next_goal()
        task = types.SimpleNamespace(search_planner=p, state="searching", reason="search",
                                     TERMINAL=TaskController.TERMINAL, _target_row=None)
        sent = []
        BeliefFeed(sent.append).tick(task, 0.0)
        meta, payload = self.frames_from(dumps(sent[0]))[0]
        out = gp.compose_belief(meta, payload)
        json.dumps(out, allow_nan=False)
        self.assertEqual((out["type"], out["phase"], out["frame"], out["seq"]), ("belief", "search", "map", 1))
        self.assertEqual(out["cells"], sent[0]["cells"])
        self.assertEqual(out["next"], [round(v, 4) for v in sent[0]["next"]])
        # cell centres from the message (origin + (col + .5, row + .5) * resolution, OccupancyGrid) cover the
        # planner's own universe cells: each universe cell lies in a message cell >= 0 holding its block max
        g = cells_of(out).astype(int)
        res, (ox, oy, _yaw) = out["resolution"], out["origin"]
        bel = p.prior * p.miss / p.prior.max()
        for k in range(0, p.ux.size, 37):
            col, row = int((p.ux[k] - ox) // res), int((p.uy[k] - oy) // res)
            self.assertGreaterEqual(g[row, col], 0)
            self.assertGreaterEqual(g[row, col], int(round(bel[k] * 100)) - 1)
        # outside the searched area: -1 everywhere a universe cell is not
        r_in = {(int((y - oy) // res), int((x - ox) // res)) for x, y in zip(p.ux, p.uy)}
        for (r, c), v in np.ndenumerate(g):
            self.assertEqual(v >= 0, (r, c) in r_in)

    def test_bridge_push_and_late_page(self):
        clock = [100.0]
        b = gb.Bridge(clock=lambda: clock[0], bridge_id="bid")
        s = FakeSession()
        b.sessions.add(s)
        text = HubTests().msg(phase="done")
        meta, payload = self.frames_from(text)[0]
        b.on_frame(types.SimpleNamespace(name="MAP"), (meta, payload))
        self.assertEqual([k for k, _ in s.events], ["belief"])
        self.assertEqual(s.events[0][1]["phase"], "done")
        clock[0] += 20.0                                             # a page connecting 20 s later: the old done
        late = b._belief_for_new_page(clock[0])
        self.assertGreater(late["ageMs"], 19000)                     # -> the page hides it at once (STALE_DONE_MS)
        b.on_frame(types.SimpleNamespace(name="MAP"), ({"which": "belief", "width": 2}, b"x"))
        self.assertEqual(b.counts["belief_invalid"], 1)

    def test_ws_session_sends_belief_text(self):
        src = open(os.path.join(ROOT, "ui", "geomem", "bridge", "geomem_bridge.py"), encoding="utf8").read()
        self.assertIn('out.append(("belief", _text_frame(self.belief)))', src)
        self.assertIn("if self.fifo or self.hello or self.snapshot or self.belief:", src)


class RunnerWiringTests(unittest.TestCase):
    def test_runner_ticks_the_feed(self):
        src = open(os.path.join(ROOT, "host", "ros_runtime.py"), encoding="utf8").read()
        self.assertIn("self.belief_feed.tick(self.task, time.monotonic())", src)
        self.assertIn("self.belief_feed.tick(None, time.monotonic())", src)
        self.assertIn("self.search_belief_topic.publish(String(data=text))", src)
        self.assertNotIn("planner.belief_message()", src)


if __name__ == "__main__":
    unittest.main()
