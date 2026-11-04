"""Nav go-live round 2 (2026-10-17, docs/design/REAL_CAR_NAV_GOLIVE_20261016.md §8):
A. CAR_NAV_DRYRUN=1 opens ONLY live --pose static --nav nav2 --chassis none (board Nav2 dry run); without it the
   static+nav2 refusal is word for word the old one; nav2 never gets the bench stand-in /map (map_server owns /map).
B. the single-goal entry: fpgapc goto -> hub op "goto" -> /fpga/goto -> Runtime.goto -> Runtime.navigate (same
   ready(), MotionPermit.arm(), permit topic; nothing relaxed).
C. an e-stop cancels navigation (O2 revoked): the hub publishes /fpga/cancel on a PC e-stop and on an adopted KV latch;
   after the release the runner's permit stays false until a NEW goal re-arms it.
No docker, ROS or GPU. The runner end-to-end cases load the production Runtime through
tools/rtabmap_runtime_probe_support (Python 3.9+, ast.unparse; D-028): on 3.8 they assert that the loader names that
requirement, as tests/test_m5_runtime.py does. Python 3.8 compatible."""
import json
import math
import os
import subprocess
import sys
import unittest
from types import SimpleNamespace as NS

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in (HERE, ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

import test_car_run as tcr  # noqa: E402  (Base / stand-ins only: no TestCase is imported by name)
import test_hub as th  # noqa: E402
from host.hub.hub_core import HubCore, NS as SEC, MS  # noqa: E402
from host.hub.fake_kv import FakeKv, KvTelemetryClient, KvEstopClient  # noqa: E402
from host.ros_runtime import goto_pose, goto_request, GOTO_MAX_FORWARD_M  # noqa: E402
from tools.rtabmap_runtime_probe_support import REQUIRED_PYTHON  # noqa: E402

COMMON = os.path.join(ROOT, 'tools', 'car_run', 'car_common.sh')
OLD_STATIC_NAV2 = "不合法：static 位姿配 nav2——台架车不动，Nav2 无从验证（板上 Nav2 入口也未定）；台架用 costmap"


def validate(**env):
    """car_validate alone (car_common.sh sourced): -> (rc, stderr)"""
    e = {k: v for k, v in os.environ.items() if k not in ('NAV_DRYRUN', 'TELEOP')}
    e.update(env)
    r = subprocess.run(['bash', '-c', 'source "$1"; car_validate', '_', COMMON], env=e, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, universal_newlines=True, timeout=30)
    return r.returncode, r.stderr


BENCH = dict(CAMERA='live', POSE='static', NAV='nav2', DETECTOR='yolo', CHASSIS='none')


class TestValidate(unittest.TestCase):
    def test_unset_is_the_old_refusal_word_for_word(self):
        for extra in ({}, {'NAV_DRYRUN': '0'}, {'NAV_DRYRUN': ''}):
            rc, err = validate(**dict(BENCH, **extra))
            self.assertEqual(rc, 2, extra)
            self.assertEqual(err.strip(), OLD_STATIC_NAV2, extra)

    def test_switch_opens_only_the_bench_combination(self):
        rc, err = validate(NAV_DRYRUN='1', **BENCH)
        self.assertEqual((rc, err), (0, ''))
        for change, still in ((dict(POSE='rtabmap'), None), (dict(NAV='costmap'), None),
                              # 10-17 (§12): odom + nav2 itself is no longer refused; the switch still is
                              (dict(POSE='odom', CHASSIS='uart'), None),
                              (dict(CHASSIS='uart'), 'static 位姿配 uart 底盘')):
            rc, err = validate(NAV_DRYRUN='1', **dict(BENCH, **change))
            self.assertEqual(rc, 2, change)
            self.assertIn('CAR_NAV_DRYRUN=1 只用于板上 Nav2 空跑 live --pose static --nav nav2 --chassis none', err, change)
            if still:
                self.assertIn(still, err, change)      # the other refusals are untouched by the switch

    def test_bad_value(self):
        for v in ('2', 'yes', 'true'):
            rc, err = validate(NAV_DRYRUN=v, **BENCH)
            self.assertEqual(rc, 2, v)
            self.assertIn("CAR_NAV_DRYRUN 只能是 0 或 1（给的是 '%s'）" % v, err)


class TestNavDryRunCarUp(tcr.Base):
    """car_up end to end with the board stand-ins of test_car_run.TestLiveBoard (no camera, KV260, docker)."""
    ARGS = ['live', '--pose', 'static', '--chassis', 'none', '--nav', 'nav2']

    def setUp(self):
        tcr.Base.setUp(self)
        tcr.write_exec(os.path.join(self.tmp, 'lsusb'), tcr.FAKE_LSUSB)
        tcr.write_exec(os.path.join(self.tmp, 'kvcheck'), tcr.FAKE_KV_CHECK)
        self.usb = os.path.join(self.tmp, 'usb')
        self.node = os.path.join(self.usb, '002', '005')
        self.map_yaml = os.path.join(self.tmp, 'maps', 'export', 'm.yaml')
        os.makedirs(os.path.dirname(self.map_yaml))
        with open(self.map_yaml, 'w') as f:
            f.write('image: m.pgm\n')
        self.env.update(CAR_ARCH='aarch64', CAR_BOARD_DRAFT='1', CAR_HUB_BIND='198.51.100.1',
                        CAR_LSUSB=os.path.join(self.tmp, 'lsusb'), CAR_USB_ROOT=self.usb,
                        CAR_KV_CHECK=os.path.join(self.tmp, 'kvcheck'), CAR_PY_BOARD=os.path.join(self.tmp, 'py'),
                        RTABMAP_DB='', CAR_MAP_YAML=self.map_yaml)
        for k in ('CAR_NAV_DRYRUN', 'BENCH_MAP_M'):
            self.env.pop(k, None)

    def line(self, out, prefix):
        return next(l for l in out.splitlines() if l.startswith(prefix))

    def names(self, out):
        return [l.split(']')[0].strip().lstrip('[').split(',')[0] for l in out.splitlines() if l.startswith('    [')]

    def test_refused_without_the_switch_nothing_created(self):
        run = os.path.join(self.tmp, 'runs', 'noswitch')
        r = self.up(*(self.ARGS + ['--run-dir', run]), FAKE_USB='1')
        self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
        self.assertIn(OLD_STATIC_NAV2, r.stderr)
        self.assertFalse(os.path.exists(run))
        self.assertEqual(os.listdir(self.state), [])
        self.assertFalse(os.path.exists(os.path.join(self.fd, 'calls.log')))

    def test_switch_with_another_combination_refused(self):
        for args in (['live', '--pose', 'static', '--chassis', 'none', '--nav', 'costmap'],
                     ['live', '--pose', 'odom', '--nav', 'nav2']):
            run = os.path.join(self.tmp, 'runs', 'other')
            r = self.up(*(args + ['--run-dir', run]), FAKE_USB='1', CAR_NAV_DRYRUN='1', RTABMAP_DB='')
            self.assertEqual(r.returncode, 2, (args, r.stdout + r.stderr))
            self.assertIn('CAR_NAV_DRYRUN=1 只用于', r.stderr)
            self.assertFalse(os.path.exists(run))

    def test_dry_run_plan(self):
        r = self.up(*(self.ARGS + ['--dry-run']), FAKE_USB='1', CAR_NAV_DRYRUN='1', BENCH_MAP_M='8')
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        self.assertIn('# NAV DRY RUN (CAR_NAV_DRYRUN=1)', out)
        self.assertIn('bench_pose.py --odom-hz 0 --map-m 0', self.line(out, '    [bench_pose]'))   # no stand-in /map
        stack = self.line(out, '[container stack]')
        self.assertIn('-e NAV_DRYRUN=1 ', stack)
        self.assertIn('-e BENCH_MAP_M=0 ', stack)
        self.assertNotIn('--device /dev/fpga_chassis', stack)
        nav = self.line(out, '    [navigation]')
        for word in ('FPGA_NAV_SPEED_LIMITS=car', 'FPGA_NAV_VELOCITY_GUARD=true', 'FPGA_NAV_COSTMAP=rolling',
                     'FPGA_NAV_MAP_YAML=%s' % self.map_yaml, 'FPGA_NAV_STATIC_MAP_ODOM=false'):
            self.assertIn(word, nav)
        names = self.names(out)
        self.assertNotIn('rtabmap', names)
        self.assertNotIn('chassis', names)
        self.assertLess(names.index('producer'), names.index('rolling_grid'))
        self.assertLess(names.index('rolling_grid'), names.index('nav2_activate'))
        self.assertLess(names.index('nav2_activate'), names.index('runtime'))
        self.assertIn('--pose-source reference', self.line(out, '    [runtime]'))

    def test_costmap_bench_keeps_its_stand_in_map(self):
        """the line-4 bench (static + costmap) is unchanged: stand-in /map 8 m, no NAV_DRYRUN in the container"""
        r = self.up('live', '--pose', 'static', '--chassis', 'none', '--nav', 'costmap', '--dry-run', FAKE_USB='1')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('bench_pose.py --odom-hz 0 --map-m 8', r.stdout)
        self.assertNotIn('NAV_DRYRUN', r.stdout)

    def test_start_and_stop_with_standins(self):
        os.makedirs(os.path.dirname(self.node))
        os.symlink('/dev/null', self.node)
        run = os.path.join(self.tmp, 'runs', 'navdry1')
        r = self.up(*(self.ARGS + ['--run-dir', run]), FAKE_USB='1', CAR_NAV_DRYRUN='1', CAR_HEIGHT_CELL='8')
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        with open(os.path.join(self.fd, 'wde-car-stack-navdry1.json')) as f:
            argv = json.load(f)['argv']
        for word in ('NAV_DRYRUN=1', 'BENCH_MAP_M=0', 'POSE=static', 'NAV=nav2', 'CHASSIS=none'):
            self.assertIn(word, argv)
        self.assertNotIn('/dev/fpga_chassis', argv)
        with open(os.path.join(run, 'car.json')) as f:
            car = json.load(f)
        self.assertEqual((car['pose'], car['nav'], car['chassis']), ('static', 'nav2', 'none'))
        self.assertTrue(car['nav_dryrun']['enabled'])
        self.assertEqual(car['bench']['stand_in_map_m'], 0.0)
        self.assertEqual(car['nav2_map_yaml'], self.map_yaml)
        d = self.down()
        self.assertEqual(d.returncode, 0, d.stdout + d.stderr)
        self.assertEqual(self.containers(), [])


# ------------------------------------------------------------------------------------------------ B / C: hub
class TestHubGotoAndEstop(unittest.TestCase):
    def setUp(self):
        self.actions = th.Actions()
        self.core = HubCore("live", actions=self.actions)
        self.core.feed_status(th.status("startup", sensor_source="hardware"), 0)
        self.kv = FakeKv()
        self.estop = KvEstopClient(self.kv)
        self.t = [1 * SEC]

    def cmd(self, op, cid, args=None, controller=True, client="pc1"):
        return self.core.command(client, controller, {"v": 1, "cmd_id": cid, "op": op, "args": args or {}}, self.t[0])

    def test_goto_forwarded_as_json(self):
        ack = self.cmd("goto", "pc1:1", {"forward_m": 1.0})
        self.assertEqual(ack["status"], "forwarded")
        self.cmd("goto", "pc1:2", {"turn_rad": math.pi / 2})
        sent = [json.loads(p[1]) for p in self.actions.topics("/fpga/goto")]
        self.assertEqual(sent, [{"forward_m": 1.0, "turn_rad": 0.0}, {"forward_m": 0.0, "turn_rad": math.pi / 2}])
        for body in sent:     # what the hub forwards, the runner accepts
            goto_request(json.dumps(body))

    def test_goto_bad_args_never_leave_the_hub(self):
        bad = [{"forward_m": GOTO_MAX_FORWARD_M + 0.01}, {"forward_m": -0.1}, {"forward_m": float("nan")},
               {"forward_m": True}, {"forward_m": "1"}, {"turn_rad": 3.2}, {}, {"forward_m": 0, "turn_rad": 0},
               {"forward_m": 1.0, "dy": 0.5}]
        for i, args in enumerate(bad):
            self.assertEqual(self.cmd("goto", "pc1:%d" % i, args)["reason"], "bad_args", args)
            with self.assertRaises(ValueError):
                goto_request(json.dumps(args, allow_nan=True))
        self.assertEqual(self.actions.topics("/fpga/goto"), [])

    def test_goto_holder_only_and_refused_while_latched(self):
        self.assertEqual(self.cmd("goto", "b:1", {"forward_m": 1.0}, controller=False, client="b")["reason"],
                         "not_controller")
        self.cmd("estop", "b:2", controller=False, client="b")
        self.assertEqual(self.cmd("goto", "pc1:1", {"forward_m": 1.0})["reason"], "estop_latched")
        self.assertEqual(self.actions.topics("/fpga/goto"), [])

    def test_goto_result_from_the_runner(self):
        self.cmd("goto", "pc1:1", {"forward_m": 1.0})
        self.core.feed_status(th.status("goto_accepted", command_id="goto-1", pose={"x": 1.0, "y": 0.0, "yaw": 0.0}),
                              self.t[0] + MS)
        self.cmd("goto", "pc1:2", {"forward_m": 1.0})
        self.core.feed_status(th.status("goto_rejected", reason="goal_not_free"), self.t[0] + 2 * MS)
        self.cmd("goto", "pc1:3", {"forward_m": 1.0})
        self.core.tick(self.t[0] + 3 * SEC)
        got = {r[1]["cmd_id"]: (r[1]["status"], r[1].get("reason"), r[1].get("command_id")) for r in self.core.pop_results()}
        self.assertEqual(got, {"pc1:1": ("accepted", None, "goto-1"), "pc1:2": ("rejected", "goal_not_free", None),
                               "pc1:3": ("unknown", None, None)})

    def test_estop_cancels_navigation(self):
        """C: an e-stop (any PC) publishes /fpga/cancel once per command; the release publishes nothing that moves"""
        self.cmd("estop", "b:1", controller=False, client="b")
        self.assertEqual(len(self.actions.topics("/fpga/cancel")), 1)
        self.assertEqual(self.core.counts["estop_cancel:pc"], 1)
        self.cmd("estop", "b:1", controller=False, client="b")          # the same command resent: deduplicated
        self.assertEqual(len(self.actions.topics("/fpga/cancel")), 1)
        self.core.estop.kv_step(self.estop, lambda: self.t[0])
        self.assertEqual(self.cmd("release", "pc1:1", {"epoch": self.core.estop.epoch})["status"], "forwarded")
        self.t[0] += 300 * MS
        self.core.estop.kv_step(self.estop, lambda: self.t[0])
        self.assertFalse(self.core.estop.latched)
        self.assertEqual([p[0] for p in self.actions.published], ["/fpga/cancel"])   # no query/select/goto

    def test_adopted_kv_latch_cancels_too(self):
        KvEstopClient(self.kv).send("assert", 77, 0, timeout_s=0.2)      # latched on the KV, not by this hub
        self.core.feed_telemetry(KvTelemetryClient(self.kv).poll(), self.t[0])
        self.assertTrue(self.core.estop.latched)
        self.assertEqual(self.core.counts["estop_adopted"], 1)
        self.assertEqual(len(self.actions.topics("/fpga/cancel")), 1)
        self.core.feed_telemetry(KvTelemetryClient(self.kv).poll(), self.t[0] + 100 * MS)   # still latched: once
        self.assertEqual(len(self.actions.topics("/fpga/cancel")), 1)


# ------------------------------------------------------------------------------------------------ B / C: runner
class TestGotoPure(unittest.TestCase):
    def test_request_bounds(self):
        self.assertEqual(goto_request('{"forward_m": 1}'), (1.0, 0.0))
        self.assertEqual(goto_request('{"turn_rad": -1.5}'), (0.0, -1.5))
        for text in ('x', '[]', '{"forward_m": 1, "turn": 1}', '{"forward_m": 2.5}', '{"forward_m": -1}',
                     '{"turn_rad": 4}', '{"forward_m": 0}', '{"forward_m": NaN}', '{"forward_m": Infinity}',
                     '{"forward_m": true}'):
            with self.assertRaises(ValueError, msg=text):
                goto_request(text)

    def test_pose_relative_to_the_base(self):
        yaw = math.radians(30)
        wb = np.eye(4)
        wb[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
        wb[:2, 3] = [2.0, -1.0]
        p = goto_pose(wb, 1.0, 0.0)
        self.assertAlmostEqual(p.x, 2.0 + math.cos(yaw))
        self.assertAlmostEqual(p.y, -1.0 + math.sin(yaw))
        self.assertAlmostEqual(p.yaw, yaw)
        self.assertEqual(p.frame, "map")
        q = goto_pose(wb, 0.0, math.radians(170))           # in place, wrapped into (-pi, pi]
        self.assertAlmostEqual(q.x, 2.0)
        self.assertAlmostEqual(q.y, -1.0)
        self.assertAlmostEqual(q.yaw, math.radians(-160))
        r = goto_pose(wb, 1.0, math.radians(90))            # turn first, then 1 m along the new heading
        self.assertAlmostEqual(r.x, 2.0 + math.cos(math.radians(120)))
        self.assertAlmostEqual(r.y, -1.0 + math.sin(math.radians(120)))
        self.assertAlmostEqual(r.yaw, math.radians(120))


SUPPORTED = sys.version_info >= REQUIRED_PYTHON


class TestRunnerGotoAndCancel(unittest.TestCase):
    """the production Runtime (source-loaded, tools/m5_software_replay.Replay fixtures)"""

    def replay(self, name):
        from tools.m5_software_replay import Replay
        r = Replay(name)
        r.frame("present")
        return r

    def permit(self, r):
        return r.world.publisher_values.get('/fpga/motion_permit')

    def test_declared_requirement_on_38(self):
        if SUPPORTED:
            self.skipTest("3.9+: the cases below run")
        with self.assertRaises(RuntimeError) as cm:
            self.replay("goto_38")
        self.assertIn("requires Python 3.9+ (ast.unparse)", str(cm.exception))

    @unittest.skipUnless(SUPPORTED, "the Runtime loader needs Python 3.9+ (D-028); declared on 3.8 above")
    def test_goto_goes_through_navigate_and_the_permit(self):
        r = self.replay("goto_ok")
        start = r.node.frame.T_world_camera @ np.linalg.inv(r.node.base_camera)
        r.node.goto(NS(data='{"forward_m": 1.0}'))
        self.assertIsNone(r.node.task)                                  # no task behind a goto
        self.assertEqual(r.published("goto_accepted")[-1]["command_id"], "goto-1")
        x, y = r.last_action_pose()
        self.assertAlmostEqual(x, start[0, 3] + 1.0 * start[0, 0] / math.hypot(start[0, 0], start[1, 0]))
        self.assertFalse(self.permit(r))                                # not before Nav2 accepted
        request = r.accept()
        r.frame("present")
        self.assertTrue(self.permit(r))                                 # the same permit path as a task goal
        r.node.goto(NS(data='{"forward_m": 1.0}'))                      # busy: refused, no second action
        self.assertEqual(r.published("goto_rejected")[-1]["reason"],
                         "cancel current operation and wait for navigation to stop")
        self.assertEqual(len(r.world.action_requests), 1)
        r.result(request, True)
        self.assertEqual([(g["command_id"], g["succeeded"]) for g in r.published("goto_result")], [("goto-1", True)])
        self.assertIsNone(r.node.nav)
        self.assertTrue(r.node.motion.inhibited)
        self.assertFalse(self.permit(r))
        self.assertFalse(r.published("fault"))                          # not the patrol/task branch
        r.frame("present")
        self.assertFalse(self.permit(r))

    @unittest.skipUnless(SUPPORTED, "the Runtime loader needs Python 3.9+ (D-028)")
    def test_goto_refused_when_not_ready_or_not_free(self):
        r = self.replay("goto_refused")
        r.node.map.cells[:] = 100                                       # everything occupied
        r.node.goto(NS(data='{"forward_m": 1.0}'))
        self.assertEqual(r.published("goto_rejected")[-1]["reason"], "goal_not_free")
        r.node.goto(NS(data='{"forward_m": 9}'))
        self.assertTrue(r.published("goto_rejected")[-1]["reason"].startswith("bad_args"))
        r.advance(r.world.wall + 3.0, r.world.ros_ns / 1e9 + 3.0, "stale")
        r.node.goto(NS(data='{"turn_rad": 1.0}'))
        self.assertEqual(r.published("goto_rejected")[-1]["reason"], "recent processed input, TF and map required")
        self.assertEqual(r.world.action_requests, [])

    @unittest.skipUnless(SUPPORTED, "the Runtime loader needs Python 3.9+ (D-028)")
    def test_cancel_from_estop_keeps_the_permit_false_until_a_new_goal(self):
        """C: what the hub's e-stop sends (/fpga/cancel) cancels the Nav2 goal; frames keep coming (the release does
        not reach the runner at all), the permit stays false; only a NEW goal arms it again"""
        r = self.replay("goto_estop")
        r.node.goto(NS(data='{"forward_m": 1.0}'))
        request = r.accept()
        r.frame("present")
        self.assertTrue(self.permit(r))
        r.node.cancel(NS())                                             # hub: e-stop -> /fpga/cancel
        self.assertFalse(self.permit(r))
        self.assertEqual(len(request["handle"].cancels), 1)             # the Nav2 goal cancel was requested
        for _ in range(3):                                              # release, fresh frames: nothing re-arms
            r.frame("present")
            self.assertFalse(self.permit(r))
        r.result(request, False)                                        # Nav2 reports the cancelled goal
        self.assertTrue(r.published("navigation_retired"))
        self.assertIsNone(r.node.nav)
        r.frame("present")
        self.assertFalse(self.permit(r))
        r.node.goto(NS(data='{"turn_rad": 1.5}'))                       # a new permit: a new goal
        r.accept()
        r.frame("present")
        self.assertTrue(self.permit(r))
        self.assertEqual(len(r.world.action_requests), 2)


if __name__ == '__main__':
    unittest.main()
