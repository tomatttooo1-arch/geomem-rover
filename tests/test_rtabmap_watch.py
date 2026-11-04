"""tools/car_run/rtabmap_watch.py: the statistics the hand-held / bench mapping trial is judged by
(docs/design/PREREG_HANDHELD_RTABMAP_20261015.md section 4), on sequences whose answer is known; and
tools/car_run/rtabmap_car.sh's argument checks (no ROS, no docker). Python 3.8 compatible."""
import json
import math
import os
from pathlib import Path
import runpy
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
W = runpy.run_path(str(ROOT / 'tools/car_run/rtabmap_watch.py'))

INFO_TEXT = ("\x1b[0m\x1b[0mMem/InitWMWithAllNodes=            false\n"
             "LTM:                11 nodes and 2156 words (dim=32 type=8U)\n"
             "WM:                 7 nodes and 2156 words\n"
             "Global graph:       11 poses and 20 links\n"
             "Nodes size:         1 KB\t(0.04%)\n")


def quat_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class Statistics(unittest.TestCase):
    def test_lost_share_and_longest_streak(self):
        # 10 frames; lost at stamps 2, 3, 4 (back at 4.5 -> a 2.5 s streak of 3 frames) and the last one (9, ends lost)
        stamps = [0, 1, 2, 3, 4, 4.5, 6, 7, 8, 9]
        lost = [False, False, True, True, True, False, False, False, False, True]
        s = W['lost_stats'](list(zip(stamps, lost)))
        self.assertEqual((s['frames'], s['lost']), (10, 4))
        self.assertAlmostEqual(s['lost_share'], 0.4)
        self.assertAlmostEqual(s['longest_lost_s'], 2.5)
        self.assertEqual(s['longest_lost_frames'], 3)

    def test_streak_running_at_the_end_and_arrival_order(self):
        rows = [(3.0, True), (0.0, False), (1.0, True), (2.0, True)]    # unordered arrival
        s = W['lost_stats'](rows)
        self.assertAlmostEqual(s['longest_lost_s'], 2.0)                # 1.0 -> last frame 3.0
        self.assertEqual(s['longest_lost_frames'], 3)
        self.assertIsNone(W['lost_stats']([])['lost_share'])

    def test_gate_d_edges(self):
        # exactly 5 % and exactly 2 s pass (<=); one more lost frame does not
        rows = [(i * 0.1, False) for i in range(100)]
        rows[10:15] = [(1.0 + i * 0.1, True) for i in range(5)]           # 1.0 .. 1.4, back at 1.5
        s = W['lost_stats'](rows)
        self.assertAlmostEqual(s['lost_share'], 0.05)
        self.assertLessEqual(s['lost_share'], W['LOST_SHARE_MAX'])
        rows[20] = (2.0, True)
        self.assertGreater(W['lost_stats'](rows)['lost_share'], W['LOST_SHARE_MAX'])

    def test_rate_grid_loops(self):
        self.assertAlmostEqual(W['median_rate_hz']([0.0, 0.1, 0.2, 0.4]), 10.0)
        self.assertIsNone(W['median_rate_hz']([1.0]))
        self.assertEqual(W['grid_counts']([-1, 0, 100, 65, 64, -1]), (4, 2))
        self.assertEqual(W['loop_ids']([(2, 0), (1, 5), (3, 7)]), [5, 7])

    def test_rtabmap_info_and_poses(self):
        self.assertEqual(W['parse_info_nodes'](INFO_TEXT), {'ltm': 11, 'wm': 7, 'graph_poses': 11, 'graph_links': 20})
        self.assertEqual(W['parse_info_nodes']('nothing'), {'ltm': None, 'wm': None, 'graph_poses': None,
                                                            'graph_links': None})
        lines = ['#timestamp x y z qx qy qz qw id']
        for nid, x, y, yaw in ((3, 0.2, 0.1, math.radians(-179)), (1, 0.0, 0.0, math.radians(179))):
            lines.append('%f %f %f 0 %f %f %f %f %d' % ((100.0 + nid, x, y) + quat_yaw(yaw) + (nid,)))
        poses = W['parse_poses']('\n'.join(lines))
        self.assertEqual([p[0] for p in poses], [1, 3])                   # sorted by node id
        d, dyaw = W['first_last_gap'](poses)
        self.assertAlmostEqual(d, math.hypot(0.2, 0.1), places=6)
        self.assertAlmostEqual(dyaw, 2.0, places=2)                       # wrapped, not 358 (%f-rounded quaternion)


class Readout(unittest.TestCase):
    def run_readout(self, summary, judge_loop, poses_far=False, nodes=25, n_poses=25):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            (d / 'export').mkdir()
            (d / 'export/m.pgm').write_text('P5')
            (d / 'export/m.yaml').write_text('image: m.pgm')
            (d / 'm.db').write_bytes(b'x' * 10)
            (d / 'info.txt').write_text(INFO_TEXT.replace('11 nodes', '%d nodes' % nodes))
            x = 1.0 if poses_far else 0.1
            lines = ['%d 0.05 0 0 0 0 0 1 %d' % (i, i) for i in range(2, n_poses)]
            (d / 'poses.txt').write_text('\n'.join(['0 0 0 0 0 0 0 1 1'] + lines + ['1 %f 0 0 0 0 0 1 999' % x]) + '\n')
            (d / 'summary.json').write_text(json.dumps(summary))
            W['main'](['readout', '--out', str(d), '--summary', str(d / 'summary.json'), '--db', str(d / 'm.db'),
                       '--info-txt', str(d / 'info.txt'), '--poses', str(d / 'poses.txt'),
                       '--judge-loop', str(judge_loop), '--readout', str(d / 'r.json')])
            return json.loads((d / 'r.json').read_text())

    def summary(self, lost=0, loops=(4,)):
        rows = [(i * 0.1, i < lost) for i in range(100)]
        return W['summarize'](rows, [(1.0, l) for l in loops], [1500])

    def test_pass_and_quality_short(self):
        r = self.run_readout(self.summary(), 1)
        self.assertEqual(r['verdict'], 'pass')
        self.assertEqual((r['rtabmap_info_nodes'], r['rtabmap_info_wm_plus_ltm']), (25, 32))
        r = self.run_readout(self.summary(lost=6), 1)                   # 6 % lost
        self.assertEqual((r['d_odometry'], r['verdict']), (False, 'chain_ok_quality_short'))
        r = self.run_readout(self.summary(), 1, poses_far=True)          # did not stop back at the start
        self.assertEqual((r['e_loop'], r['verdict']), (False, 'chain_ok_quality_short'))
        r = self.run_readout(self.summary(), 0, poses_far=True)          # hand-held: e not judged
        self.assertEqual(r['verdict'], 'pass')

    def test_b_needs_exported_poses_too(self):
        # board static smoke 10-15: LTM 94 but one node in memory, 0 links -> nothing exportable: b fails
        r = self.run_readout(self.summary(), 0, nodes=94, n_poses=2)
        self.assertEqual((r['exported_poses'], r['b_nodes'], r['verdict']), (2, False, 'fail'))
        r = self.run_readout(self.summary(), 0, nodes=94, n_poses=20)
        self.assertEqual((r['exported_poses'], r['b_nodes']), (20, True))

    def test_odometry_start_check(self):
        ok = W['odom_start_ok']
        self.assertFalse(ok([False] + [True] * 19, 20, 10)[0])          # the board smoke's shape
        self.assertTrue(ok([False] * 11 + [True] * 9, 20, 10)[0])        # 9 at the end < 10
        self.assertFalse(ok([False] * 10 + [True] * 10, 20, 10)[0])
        self.assertFalse(ok([False] * 19, 20, 10)[0])                     # too few records
        self.assertTrue(ok([True] * 5 + [False] * 15 + [True] * 30, 20, 10)[0])   # judged on the first 20 only

    def test_nodes_judged_on_ltm_not_the_sum(self):
        r = self.run_readout(self.summary(), 0, nodes=15)                 # LTM 15 + WM 7 = 22 >= 20, still fails b
        self.assertFalse(r['b_nodes'])
        self.assertEqual(r['verdict'], 'fail')


class ScriptArguments(unittest.TestCase):
    def run_script(self, args, **env):
        e = {k: v for k, v in os.environ.items() if k not in ('MODE', 'DB', 'ODOM', 'STANDALONE', 'FORCE_3DOF')}
        e.update(env)
        return subprocess.run(['bash', str(ROOT / 'tools/car_run/rtabmap_car.sh')] + args, env=e,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)

    def test_refusals(self):
        for args, env in ((['map'], {'MODE': 'loc'}), ([], {}), (['loc'], {}), (['loc'], {'DB': '/nonexistent.db'}),
                          (['map'], {'ODOM': 'gps'}), (['map'], {'FORCE_3DOF': 'yes'})):
            with self.subTest(args=args, env=env):
                p = self.run_script(args, **env)
                self.assertEqual(p.returncode, 2, p.stderr)


if __name__ == '__main__':
    unittest.main()


class StatusRecord(unittest.TestCase):
    """2026-10-15 第 5 棒: watch.jsonl kept /fpga/localization_status cut at 2000 characters; every row carrying
    approvals is longer, so the lease replays reading watch.jsonl (e.g. host_loc_v6/lease_replay.py) could not parse
    exactly the rows that mattered. The record now keeps the whole string."""

    @staticmethod
    def valid_status():
        """a real correction-mode LocalizationGate driven to valid: pairs at 10 Hz (30 ms late), rtabmap outputs
        every third pair (200 ms late), 3 s -> one status row with ~10 approvals."""
        import sys
        sys.path.insert(0, str(ROOT))
        import numpy as np
        from host.localization_gate import LocalizationGate, matrix_pose
        cov = [0.01 if i % 7 == 0 else 0. for i in range(36)]
        base = 10 ** 9

        def hdr(stamp, frame):
            return {"stamp": {"sec": stamp // 10 ** 9, "nanosec": stamp % 10 ** 9}, "frame_id": frame}

        def pose(stamp):
            m = np.eye(4)
            m[0, 3] = .05 * (stamp - base) / 1e9
            return matrix_pose(m)
        gate = LocalizationGate("a" * 64, {1}, tracked_frame="base_link", count_mode="correction")
        events = []
        for k in range(30):
            stamp = base + k * 10 ** 8
            events.append((stamp + 3 * 10 ** 7, "pair", stamp))
            if k % 3 == 0:
                events.append((stamp + 2 * 10 ** 8, "rtab", stamp))
        ref = 0
        for t, kind, stamp in sorted(events):
            wall = (t - base) / 1e9
            if kind == "pair":
                gate.ingest("odom", {"header": hdr(stamp, "odom"), "child_frame_id": "base_link",
                                     "pose": {"pose": pose(stamp), "covariance": cov}}, wall, t)
                gate.ingest("odom_info", {"header": hdr(stamp, "odom"), "lost": False, "covariance": cov}, wall, t)
            else:
                ref += 1
                gate.ingest("info", {"header": hdr(stamp, "map"), "loop_closure_id": 1, "proximity_detection_id": 0,
                                     "ref_id": ref}, wall, t)
                gate.ingest("localization_pose", {"header": hdr(stamp, "map"),
                                                  "pose": {"pose": pose(stamp), "covariance": cov}}, wall, t)
        t = max(e[0] for e in events)
        return gate.status((t - base) / 1e9, t), t

    def test_status_with_approvals_is_recorded_whole_and_replays_through_the_lease(self):
        import sys
        sys.path.insert(0, str(ROOT))
        from host.localization_gate import LocalizationLease
        row, ros_ns = self.valid_status()
        self.assertEqual(row["state"], "valid")
        self.assertGreaterEqual(len(row["approved"]), 5)
        data = json.dumps(row)
        self.assertGreater(len(data), 2000)
        # the defect, as a control: the old cut cannot be parsed
        with self.assertRaises(ValueError):
            json.loads(data[:2000])
        # watch()'s write(): json.dumps(dict(kind=..., wall=..., **fields)) per line
        line = json.dumps(dict(W['status_fields'](data), kind="status", wall=1.0))
        back = json.loads(json.loads(line)["data"])
        self.assertEqual(back, row)
        lease = LocalizationLease("a" * 64, "base_link")
        wall = 100.
        lease.update(back, wall, ros_ns)
        self.assertIsNotNone(lease.row)
        newest = back["approved"][-1]["stamp_ns"]
        self.assertIsNotNone(lease.token(newest, wall, ros_ns))

    def test_watch_source_keeps_status_whole(self):
        src = (ROOT / 'tools/car_run/rtabmap_watch.py').read_text()
        self.assertIn('write("status", **status_fields(m.data))', src)
        self.assertNotIn('[:2000]', src)
        self.assertIn('summary["status_last"] = status[-1] if status else None', src)
