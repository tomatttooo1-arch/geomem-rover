"""CTRLPS2 (2026-11-03) key tests of the navigation messages of the two-board channel.

1. Round trip byte for byte: encode -> decode -> encode gives the same bytes for NAV_GOAL / NAV_ODOM / NAV_CMD /
   NAV_PATH, over the real UDP server too; positive controls: one flipped byte (crc), a wrong ver, an unknown op.
2. Default unchanged: a server without ``nav`` announces no "nav", answers no NAV message (counted unexpected) and
   its HELLO features are exactly the old set.
3. The mock drive (no ROS): NavLink (Nano) <-> KvLinkServer + NavService(MockNavCore) (KV) <-> a unicycle chassis
   integrating the published (v, w) at 50 Hz: goal -> route -> commands -> SUCCEEDED within the tolerances.
"""

import math
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from host import kv_link as K                                           # noqa: E402
from host.kv_nav_proxy import NavLink, sparsify                         # noqa: E402
from host.kv_nav_service import MockNavCore, NavService, compose       # noqa: E402

GOAL = dict(op=K.NAV_OP_GOAL, goal_id=0x1234_0000_0001, t_nano_ns=123456789, x=1.25, y=-0.5, yaw=0.7)
ODOM = dict(seq=77, t_nano_ns=10**15, odom_stamp_ns=10**15 - 5, odom_pose=(0.1, 0.2, -0.3), v=0.125, w=-0.25,
            map_odom=(0.5, -0.25, 0.125), flags=K.NAV_ODOM_PERMIT | K.NAV_ODOM_MAP_VALID | K.NAV_ODOM_ODOM_VALID)
CMD = dict(state=K.NAV_STATE["track"], reason=0, flags=K.NAV_CMD_ACCEPTED | K.NAV_CMD_VALID, goal_id=99, cmd_seq=5,
           odom_seq=77, kv_mono_ns=42, cmd_age_us=1500, v=0.125, w=-0.25, remaining_m=1.5, path_len_m=2.25,
           recoveries=1, path_points=9)
PATH = [(0.0, 0.0), (0.25, 0.0), (0.5, 0.25), (1.25, -0.5)]


class RoundTrip(unittest.TestCase):
    def test_goal(self):
        b = K.encode_nav_goal(**GOAL)
        d = K.decode_nav_goal(b)
        self.assertEqual(K.encode_nav_goal(**{k: d[k] for k in GOAL}), b)
        self.assertEqual(len(b), K.NAV_GOAL_BODY.size + 4)

    def test_odom(self):
        b = K.encode_nav_odom(**ODOM)
        d = K.decode_nav_odom(b)
        self.assertEqual(K.encode_nav_odom(**{k: d[k] for k in ODOM}), b)
        self.assertTrue(d["permit"] and d["map_valid"] and not d["teleop"])

    def test_cmd(self):
        b = K.encode_nav_cmd(**CMD)
        d = K.decode_nav_cmd(b)
        self.assertEqual(K.encode_nav_cmd(**{k: d[k] for k in CMD}), b)
        self.assertEqual(d["state_name"], "track")

    def test_path(self):
        b = K.encode_nav_path(7, 3, 99, PATH)
        d = K.decode_nav_path(b)
        self.assertEqual(K.encode_nav_path(d["goal_id"], d["path_id"], d["t_nano_ns"], d["points"]), b)
        self.assertEqual(d["points"], PATH)          # exact in f32 (chosen values)

    def test_positive_controls(self):
        for enc, dec in ((K.encode_nav_goal(**GOAL), K.decode_nav_goal), (K.encode_nav_odom(**ODOM), K.decode_nav_odom),
                         (K.encode_nav_cmd(**CMD), K.decode_nav_cmd), (K.encode_nav_path(7, 3, 99, PATH), K.decode_nav_path)):
            bad = bytearray(enc)
            bad[9] ^= 0x01
            with self.assertRaises(K.BadExtension):
                dec(bytes(bad))
            ver2 = bytearray(enc[:-4])
            ver2[0] = 2
            with self.assertRaises(K.BadExtension):
                dec(K.seal(bytes(ver2)))
        with self.assertRaises(K.BadExtension):
            K.decode_nav_goal(K.encode_nav_goal(**dict(GOAL, op=3)))
        with self.assertRaises(K.BadExtension):
            K.decode_nav_goal(K.encode_nav_goal(**dict(GOAL, x=float("nan"))))

    def test_sparsify(self):
        pts = [(i * 0.05, 0.0) for i in range(41)]          # 2 m at 0.05 m
        s = sparsify(pts)
        self.assertEqual(s[0], pts[0])
        self.assertEqual(s[-1], pts[-1])
        self.assertEqual(len(s), 9)                          # 0, 0.25 .. 2.0


class Server(unittest.TestCase):
    def test_default_server_unchanged(self):
        srv = K.KvLinkServer(K.NullCore(), "null", bind=("127.0.0.1", 0)).start()
        try:
            c = K.KvLinkClient(("127.0.0.1", srv.port), chunk_bytes=1400, timeout_s=0.5)
            info = c.hello()
            self.assertEqual(info["features"], dict(K.FEATURES))
            with self.assertRaises(K.KvFeatureUnavailable):
                c.require("nav")
            self.assertIsNone(c._control_raw(K.NAV_ODOM, K.NAV_CMD, K.encode_nav_odom(**ODOM), 0.3))
            time.sleep(0.05)
            self.assertEqual(srv.counts["unexpected:nav_odom"], 1)
            c.close()
        finally:
            srv.stop()

    def test_wire_round_trip(self):
        """The NAV_CMD bytes the server sent are the bytes decode -> encode gives back (over UDP)."""
        nav = NavService(MockNavCore())
        srv = K.KvLinkServer(K.NullCore(), "null", bind=("127.0.0.1", 0), nav=nav).start()
        try:
            c = K.KvLinkClient(("127.0.0.1", srv.port), chunk_bytes=1400, timeout_s=0.5)
            self.assertEqual(c.require("nav")["features"]["nav"], 1)
            raw = c._control_raw(K.NAV_GOAL, K.NAV_CMD, K.encode_nav_goal(**GOAL), 0.5)
            d = K.decode_nav_cmd(raw)
            self.assertEqual(K.encode_nav_cmd(**{k: d[k] for k in CMD}), bytes(raw))
            self.assertTrue(d["accepted"])
            self.assertEqual(d["goal_id"], GOAL["goal_id"])
            long_path = [(i * 0.25, math.sin(i / 7.0)) for i in range(400)]   # 3.2 kB: several chunks
            d2 = c.nav_call(K.NAV_PATH, K.encode_nav_path(GOAL["goal_id"], 1, 5, long_path), 0.5, resend_s=0.05)
            self.assertIsNotNone(d2)
            self.assertEqual(len(nav.core.route), 400)
            self.assertEqual(nav.core.route, [K.NAV_PATH_POINT.unpack(K.NAV_PATH_POINT.pack(*p)) for p in long_path])
            bad = bytearray(K.encode_nav_odom(**ODOM)); bad[20] ^= 1
            self.assertIsNone(c._control_raw(K.NAV_ODOM, K.NAV_CMD, bytes(bad), 0.2))
            self.assertEqual(srv.counts["bad_ext:crc"], 1)
            c.close()
        finally:
            srv.stop()


class MockDrive(unittest.TestCase):
    def test_goal_route_commands_chassis(self):
        nav = NavService(MockNavCore())
        srv = K.KvLinkServer(K.NullCore(), "null", bind=("127.0.0.1", 0), nav=nav).start()
        link = None
        try:
            c = K.KvLinkClient(("127.0.0.1", srv.port), chunk_bytes=1400, timeout_s=0.5, poke_s=0.0)
            c.require("nav")
            link = NavLink(c).start()
            map_odom = (0.3, -0.2, 0.2)             # map->odom not identity: the KV must compose it
            pose = [0.0, 0.0, 0.0]                  # odom->base_link
            goal = (1.2, 0.6, -1.0)                 # map frame, behind-left of the start heading after map_odom
            gid = link.new_goal_id()
            link.send_goal(gid, *goal)
            route = [(compose(map_odom, (0, 0, 0))[0] + t * (goal[0] - 0.3), -0.2 + t * (goal[1] + 0.2))
                     for t in [i / 20 for i in range(21)]]
            dt, t_end, sent_path, zeros_stale = 0.02, time.monotonic() + 40, False, 0
            done = None
            while time.monotonic() < t_end:
                link.set_odom(odom_pose=pose, map_odom=map_odom, permit=True)
                cmd = link.goal_status(gid)
                if cmd is not None and cmd["need_path"] and not sent_path:
                    link.send_path(gid, route)
                    sent_path = True
                if cmd is not None and link.terminal(cmd):
                    done = cmd
                    break
                v, w, why = link.command()
                pose[0] += v * math.cos(pose[2]) * dt
                pose[1] += v * math.sin(pose[2]) * dt
                pose[2] = math.atan2(math.sin(pose[2] + w * dt), math.cos(pose[2] + w * dt))
                time.sleep(dt)
            self.assertIsNotNone(done, "no terminal state in 40 s (last %r)" % (link.cmd,))
            self.assertEqual(done["state_name"], "succeeded")
            self.assertTrue(sent_path)
            x, y, th = compose(map_odom, pose)
            self.assertLessEqual(math.hypot(x - goal[0], y - goal[1]), 0.12)
            self.assertLessEqual(abs(math.atan2(math.sin(th - goal[2]), math.cos(th - goal[2]))), 0.12)
            # cancel of a new goal ends CANCELED and the command goes to zero at once
            gid2 = link.new_goal_id()
            link.send_goal(gid2, 0.0, 0.0, 0.0)
            time.sleep(0.2)
            link.send_cancel(gid2)
            t1 = time.monotonic() + 2
            while time.monotonic() < t1 and not link.terminal(link.goal_status(gid2)):
                link.set_odom(odom_pose=pose, map_odom=map_odom, permit=True)
                time.sleep(0.02)
            self.assertEqual(link.goal_status(gid2)["state_name"], "canceled")
            self.assertEqual(link.command()[:2], (0.0, 0.0))
            # stale rule: no answer for > 150 ms -> zero
            link.cmd_t -= 1.0
            self.assertEqual(link.command()[2], "stale")
            print("MOCK_DRIVE", dict(final_xy_err=round(math.hypot(x - goal[0], y - goal[1]), 3),
                                     cmds=link.counts["cmd"], timeouts=link.counts["timeout"],
                                     kv=dict(nav.counts)), flush=True)
        finally:
            if link is not None:
                link.stop()
                link.client.close()
            srv.stop()


if __name__ == "__main__":
    unittest.main()
