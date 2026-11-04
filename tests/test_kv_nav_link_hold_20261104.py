"""CTRLPS6 (2026-11-04) key tests: a lost link no longer aborts the goal at once.

The whole Nano <-> KV navigation path minus ROS, with the REAL core (libkv_nav via host.kv_nav.KvNavCore and the
CTRLPS3 test world SimWorldGrid(tests/kv_nav_real/world.json)): NavLink (Nano) <-> KvLinkServer + NavService (KV)
<-> a unicycle chassis integrating the published (v, w) at 50 Hz in real time, and LinkWatch called like the
action server's wait loop. The link is cut by making every nav_call of the Nano's client fail for a while once
the car has driven 0.4 m:

1. hold (default), 1.5 s cut, both directions lost        -> the goal stays active, ends SUCCEEDED; the core was
   held (PAUSED) and released (replanned) -- KV side: the odometry gap.
2. hold, 1.5 s cut, only the KV's answers lost             -> SUCCEEDED; the core saw NAV_ODOM_HOLD (the Nano's bit),
   it kept getting odometry so only the Nano bit can tell it.
3. hold, 35 s cut                                          -> ABORTED link_lost after the 30 s limit; the core was
   held the whole time (no stuck recovery: state paused, recoveries 0).
4. positive control: abort (CTRLPS2 rule, KV hold off), 1.5 s cut -> ABORTED link_lost (the old result).

Needs tools/kv_nav/build/host/libkv_nav.so (make -C tools/kv_nav build/host/libkv_nav.so) or KV_NAV_LIB.
Run: python3 -m unittest tests.test_kv_nav_link_hold_20261104 -v   (about 80 s, real time)
"""

import math
import os
import sys
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from host import kv_link as K                                                    # noqa: E402
from host.kv_nav_proxy import LINK_LOST_S, LinkWatch, NavLink                     # noqa: E402
from host.kv_nav_service import NavService                                       # noqa: E402

WORLD = os.path.join(ROOT, "tests/kv_nav_real/world.json")
GOAL = (1.0, 1.2, 1.57)          # goal 1 of the CTRLPS3/5 container run (1.47 m, ~17 s)
CUT_AFTER_M = 0.4


def make_core():
    from host import kv_nav
    return kv_nav.make_core(field="soft", grid_source=kv_nav.SimWorldGrid(WORLD))


class Cut:
    """Replaces client.nav_call: inside [t0, t0 + dur) the call fails (both: nothing reaches the KV; replies: the
    KV gets the message, the answer is lost)."""

    def __init__(self, client):
        self.orig = client.nav_call
        client.nav_call = self.call
        self.t0, self.dur, self.kind = None, 0.0, "both"

    def active(self):
        return self.t0 is not None and self.t0 <= time.monotonic() < self.t0 + self.dur

    def call(self, mtype, body, timeout_s, resend_s=0.0):
        if self.active():
            if self.kind == "both":
                time.sleep(timeout_s)
                return None
            self.orig(mtype, body, timeout_s, resend_s=resend_s)
            return None
        return self.orig(mtype, body, timeout_s, resend_s=resend_s)


def drive(mode, cut_s, kind="both", kv_hold_s=1.0, limit_s=90.0):
    nav = NavService(make_core(), odom_hold_s=kv_hold_s)
    srv = K.KvLinkServer(K.NullCore(), "null", bind=("127.0.0.1", 0), nav=nav).start()
    nav.start()
    link = None
    states = []
    try:
        c = K.KvLinkClient(("127.0.0.1", srv.port), chunk_bytes=1400, timeout_s=0.5, poke_s=0.0)
        c.require("nav")
        cut = Cut(c)
        cut.kind = kind
        link = NavLink(c, on_cmd=lambda cmd, t: states.append((t, cmd["state_name"])),
                       hold_after_s=LINK_LOST_S if mode == "hold" else None).start()
        pose = [0.0, 0.0, 0.0]
        link.set_odom(odom_pose=pose, permit=True)
        time.sleep(0.2)
        gid = link.new_goal_id()
        link.send_goal(gid, *GOAL)
        route = [(t * GOAL[0], t * GOAL[1]) for t in [i / 30 for i in range(31)]]
        watch = LinkWatch(link, mode=mode, abort_s=30.0)
        dt, t_end, sent_path = 0.02, time.monotonic() + limit_s, False
        outcome = None
        while time.monotonic() < t_end:
            link.set_odom(odom_pose=pose, permit=True)
            if cut.t0 is None and math.hypot(pose[0], pose[1]) >= CUT_AFTER_M:
                cut.t0, cut.dur = time.monotonic(), cut_s
            cmd = link.goal_status(gid)
            if cmd is not None and cmd["need_path"] and not sent_path:
                link.send_path(gid, route)
                sent_path = True
            if cmd is not None and link.terminal(cmd):
                outcome = (cmd["state_name"], cmd["reason_name"])
                break
            lost = watch.check()                       # as the action server's wait loop
            if lost is not None:
                link.send_cancel(gid)
                outcome = lost
                break
            v, w, why = link.command()
            pose[0] += v * math.cos(pose[2]) * dt
            pose[1] += v * math.sin(pose[2]) * dt
            pose[2] = math.atan2(math.sin(pose[2] + w * dt), math.cos(pose[2] + w * dt))
            time.sleep(dt)
        t_cut = cut.t0
        after = [s for t, s in states if t_cut is not None and t >= t_cut + cut_s - 0.05]
        res = dict(mode=mode, cut_s=cut_s, kind=kind, outcome=outcome, events=watch.events,
                   xy_err=round(math.hypot(pose[0] - GOAL[0], pose[1] - GOAL[1]), 3),
                   states_after_cut=sorted(set(after)), first_after_cut=after[:3],
                   nano=dict(link.counts), kv=dict(nav.counts),
                   core=dict(nav.core.counts), recoveries=(nav.core.last or {}).get("recoveries"),
                   core_state=(nav.core.last or {}).get("state"))
        print("CTRLPS6", res, flush=True)
        return res
    finally:
        nav.stop()
        if link is not None:
            link.stop()
            link.client.close()
        srv.stop()


class LinkLostHold(unittest.TestCase):
    def test_1_hold_cut_1_5s_both_ways_succeeds(self):
        r = drive("hold", 1.5, "both")
        self.assertEqual(r["outcome"], ("succeeded", "none"))
        self.assertEqual([e[0] for e in r["events"]], ["lost", "resumed"])
        self.assertGreater(r["kv"].get("hold_ticks", 0), 0)          # the core was held during the gap
        self.assertGreater(r["kv"].get("hold_odom", 0) + r["kv"].get("hold_gap_end", 0), 0)
        self.assertEqual(r["first_after_cut"][0], "paused")           # first answer after the cut: held zero
        self.assertLessEqual(r["xy_err"], 0.12)

    def test_2_hold_cut_1_5s_answers_only_succeeds(self):
        r = drive("hold", 1.5, "replies")
        self.assertEqual(r["outcome"], ("succeeded", "none"))
        self.assertEqual([e[0] for e in r["events"]], ["lost", "resumed"])
        self.assertGreater(r["kv"].get("hold_odom", 0), 0)            # the Nano's NAV_ODOM_HOLD reached the core
        self.assertEqual(r["kv"].get("hold_gap_end", 0), 0)           # the KV never missed odometry
        self.assertLessEqual(r["xy_err"], 0.12)

    def test_3_hold_cut_35s_aborts_after_limit(self):
        r = drive("hold", 35.0, "both")
        self.assertEqual(r["outcome"], ("aborted", "link_lost"))
        self.assertEqual([e[0] for e in r["events"]], ["lost", "abort"])
        self.assertGreater(r["events"][1][1], 30.0)
        self.assertEqual(r["core_state"], "paused")                   # held, not in a stuck recovery
        self.assertEqual(r["recoveries"], 0)

    def test_4_positive_control_old_rule_aborts(self):
        r = drive("abort", 1.5, "both", kv_hold_s=0)
        self.assertEqual(r["outcome"], ("aborted", "link_lost"))
        self.assertEqual(r["events"], [])
        self.assertEqual(r["kv"].get("hold_ticks", 0), 0)


class Wire(unittest.TestCase):
    def test_hold_bit_round_trip_and_old_bits(self):
        o = dict(seq=1, t_nano_ns=1, odom_stamp_ns=1, odom_pose=(0, 0, 0), v=0.0, w=0.0, map_odom=(0, 0, 0),
                 flags=K.NAV_ODOM_PERMIT | K.NAV_ODOM_HOLD)
        d = K.decode_nav_odom(K.encode_nav_odom(**o))
        self.assertTrue(d["hold"] and d["permit"] and not d["teleop"])
        self.assertFalse(K.decode_nav_odom(K.encode_nav_odom(**dict(o, flags=K.NAV_ODOM_PERMIT)))["hold"])
        with self.assertRaises(K.BadExtension):                       # bits above the hold bit still refused
            K.decode_nav_odom(K.encode_nav_odom(**dict(o, flags=32)))


if __name__ == "__main__":
    unittest.main()
