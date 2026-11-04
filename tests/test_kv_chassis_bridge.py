"""CHASSIS0: KV-side chassis bridge (host/kv_chassis_bridge.py) -- host only, no serial hardware.

The byte-for-byte comparison with the Nano driver on a real-car recording is tools/kv_chassis/compare_nano.py
(results in runs/nationals-prelim-20261102/CHASSIS0/); these are the behaviour checks around it.
"""

import os
import pty
import sys
import time
import tty
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np                                                         # noqa: E402

from host.chassis_uart import FLAG_ENABLE, FLAG_STATUS2, decode_frame, encode_status2  # noqa: E402
from host.kv_chassis_bridge import KvChassisBridge, nav_cmd_velocity      # noqa: E402
from host.kv_link import (NAV_CMD_ACCEPTED, NAV_CMD_VALID, NAV_ODOM, NAV_STATE, decode_nav_odom,  # noqa: E402
                          encode_nav_cmd)
from host.kv_local_frames import FrameParams                              # noqa: E402

MS = 1_000_000


def status(seq, ms, a=0, b=0, hb=True):
    return encode_status2(seq=seq, mcu_ms=ms, enc_a=a, enc_b=b, gz_n=2, status=(1 if hb else 0) | 0x30,
                          last_cmd_seq=0xFF)


class Nav:
    def __init__(self, v=0.2, w=0.1, valid=True, state="track"):
        self.v, self.w, self.valid, self.state = v, w, valid, state
        self.odoms = []

    def handle(self, mtype, body):
        assert mtype == NAV_ODOM
        self.odoms.append(decode_nav_odom(body))
        return encode_nav_cmd(state=NAV_STATE[self.state], reason=0,
                              flags=NAV_CMD_ACCEPTED | (NAV_CMD_VALID if self.valid else 0), goal_id=1, cmd_seq=1,
                              odom_seq=0, kv_mono_ns=0, cmd_age_us=0, v=self.v, w=self.w, remaining_m=1.0,
                              path_len_m=1.0, recoveries=0, path_points=2)


class BridgeLogic(unittest.TestCase):
    def test_drives_valid_command_after_mcu_heard(self):
        nav = Nav()
        b = KvChassisBridge("/nonexistent", nav=nav, clock_ns=lambda: 0)
        f, d = b.command_tick(0)                   # MCU never heard: no pose, no NAV_ODOM, enable 0
        self.assertFalse(decode_frame(f)["enable"])
        self.assertEqual(nav.odoms, [])
        b.on_bytes(status(1, 10), 5 * MS)
        b.on_bytes(status(2, 20, 100, 100), 15 * MS)
        f, d = b.command_tick(20 * MS)
        c = decode_frame(f)
        self.assertTrue(c["enable"] and c["status2"])
        self.assertEqual((c["v_mm_s"], c["w_mrad_s"]), (200, 100))
        o = nav.odoms[-1]
        self.assertTrue(o["permit"] and o["odom_valid"])
        self.assertGreater(o["odom_pose"][0], 0.0)  # 100 counts forward on both wheels

    def test_not_ok_is_enable_zero_at_once(self):
        for nav in (Nav(valid=False), Nav(state="succeeded")):
            b = KvChassisBridge("/nonexistent", nav=nav, clock_ns=lambda: 0)
            b.on_bytes(status(1, 10), 5 * MS)
            c = decode_frame(b.command_tick(10 * MS)[0])
            self.assertFalse(c["enable"])
            self.assertEqual((c["v_mm_s"], c["w_mrad_s"]), (0, 0))

    def test_silent_mcu_stops_and_pose_goes_stale(self):
        b = KvChassisBridge("/nonexistent", nav=Nav(), clock_ns=lambda: 0)
        b.on_bytes(status(1, 10), 0)
        self.assertTrue(decode_frame(b.command_tick(10 * MS)[0])["enable"])
        self.assertFalse(decode_frame(b.command_tick(400 * MS)[0])["enable"])   # > 300 ms silent
        self.assertIsNone(b.pose(400 * MS))
        self.assertIsNotNone(b.last_pose())     # the map keeps the last pose, never the static bench pose

    def test_heartbeat_missing_clears_permit(self):
        nav = Nav()
        b = KvChassisBridge("/nonexistent", nav=nav, clock_ns=lambda: 0)
        b.on_bytes(status(1, 10, hb=False), 0)
        b.command_tick(5 * MS)
        self.assertFalse(nav.odoms[-1]["permit"])

    def test_nav_cmd_rule(self):
        self.assertEqual(nav_cmd_velocity(None, None, 1.0)[2], "stale")
        cmd = {"accepted": True, "state": NAV_STATE["track"], "valid": True, "v": 0.1, "w": 0.0}
        self.assertEqual(nav_cmd_velocity(cmd, 1.0, 1.1)[2], "ok")
        self.assertEqual(nav_cmd_velocity(cmd, 1.0, 1.2)[2], "stale")      # > 150 ms


class FallbackNeverRefuses(unittest.TestCase):
    def test_missing_port_warns_and_runs(self):
        b = KvChassisBridge("/nonexistent/fpga_chassis", nav=Nav(), reopen_s=0.05).start()
        time.sleep(0.2)
        b.stop()
        self.assertGreaterEqual(b.counts["open_failed"], 2)
        self.assertIsNone(b.pose())
        self.assertGreater(b.counts["cmd:stale"], 0)

    def test_pty_round_trip_and_stop_frames(self):
        master, slave = pty.openpty()
        tty.setraw(master)
        b = KvChassisBridge(os.ttyname(slave), nav=Nav()).start()
        try:
            for i in range(10):
                os.write(master, status(i + 1, 10 * (i + 1)))
                time.sleep(0.01)
            time.sleep(0.1)
        finally:
            b.stop()
        data = b""
        while True:
            import select
            if not select.select([master], [], [], 0.2)[0]:
                break
            data += os.read(master, 4096)
        os.close(master)
        os.close(slave)
        frames = [data[i:i + 11] for i in range(0, len(data) - len(data) % 11, 11)]
        flags = [decode_frame(f)["flags"] for f in frames]
        self.assertTrue(any(fl & FLAG_ENABLE for fl in flags))
        self.assertTrue(all(fl & FLAG_STATUS2 for fl in flags))
        self.assertFalse(any(fl & FLAG_ENABLE for fl in flags[-3:]))   # shutdown: enable 0
        self.assertEqual(b.counts["frames"], 10)


class FramePose(unittest.TestCase):
    def test_at_origin_is_the_config(self):
        p = FrameParams({"intrinsics": {"fx": 500, "fy": 500, "cx": 320, "cy": 240}})
        q = p.at_pose(0.0, 0.0, 0.0)
        np.testing.assert_array_equal(q.camera_to_world, p.camera_to_world)
        np.testing.assert_array_equal(q.origin, p.origin)
        self.assertEqual(q.odom_pose[:2], p.odom_pose[:2])

    def test_moved_pose(self):
        p = FrameParams({"intrinsics": {"fx": 500, "fy": 500, "cx": 320, "cy": 240}})
        q = p.at_pose(1.0, 2.0, np.pi / 2)
        np.testing.assert_allclose(q.camera_to_world[:3, 3], [1.0, 2.0, 0.30], atol=1e-12)
        np.testing.assert_allclose(q.camera_to_world[:3, 2], [0.0, 1.0, 0.0], atol=1e-12)  # optical axis +y
        self.assertIsNot(q.grid, p.grid)
        np.testing.assert_array_equal(p.camera_to_world[:3, 3], [0.0, 0.0, 0.30])          # original untouched


if __name__ == "__main__":
    unittest.main()
