"""Teleop / Nav2 arbitration (2026-10-18, docs/design/TELEOP_NAV_ARB_20261018.md section 4, criteria A1-A9).

TeleopCore.arbitrate is executed (pure, no ROS). The ROS wiring of host/ros_teleop_safe.py, host/ros_runtime.py and the
velocity guard is pinned in the source (this host has no ROS 2; the container run of the design doc section 5 is the
end-to-end check).
"""
import json
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from host.ros_teleop_safe import NAV_STALE_S, TeleopCore  # noqa: E402
from host import ros_runtime  # noqa: E402

MS = 1000000


def _read(path):
    with open(path) as f:
        return f.read()


def dgram(v, w, **kw):
    row = dict(v=v, w=w)
    row.update(kw)
    return json.dumps(row).encode()


class TestArbitrationCore(unittest.TestCase):
    def core(self, **kw):
        c = TeleopCore(**kw)
        c.enable_arbitration()
        return c

    def test_a1_nav_passes_unchanged_while_not_in_hand(self):
        c = self.core()
        c.on_nav(0.22, -0.4, 1000 * MS)                  # beyond the teleop limits: the guard/Nav2 own nav limits
        v, w, reason, src, take = c.arbitrate(1050 * MS)
        self.assertEqual((v, w, reason, src, take), (0.22, -0.4, 'nav', 'nav', False))

    def test_a2_nav_stale_after_200ms_is_zero(self):
        c = self.core()
        self.assertEqual(NAV_STALE_S, 0.2)
        c.on_nav(0.1, 0.0, 1000 * MS)
        self.assertEqual(c.arbitrate(1200 * MS)[:3], (0.1, 0.0, 'nav'))          # exactly 200 ms: still fresh
        self.assertEqual(c.arbitrate(1201 * MS)[:4], (0., 0., 'nav_stale', 'idle'))
        self.assertEqual(self.core().arbitrate(5 * MS)[:4], (0., 0., 'nav_none', 'idle'))   # no sample at all

    def test_a3_in_hand_overrides_nav_and_takeover_fires_once(self):
        c = self.core()
        c.on_nav(0.2, 0.0, 1000 * MS)
        c.on_datagram(dgram(0.1, 0.3), 1010 * MS)
        c.on_nav(0.2, 0.0, 1040 * MS)
        out = [c.arbitrate(t * MS) for t in (1050, 1100, 1150)]
        self.assertEqual([o[3] for o in out], ['teleop'] * 3)
        self.assertEqual([o[:2] for o in out], [(0.1, 0.3)] * 3)
        self.assertEqual([o[4] for o in out], [True, False, False])            # ONE cancel on the edge
        self.assertEqual(c.counters['takeover'], 1)

    def test_a4_stop_datagram_is_in_hand_and_zero(self):
        c = self.core()
        c.on_nav(0.2, 0.0, 1000 * MS)
        c.on_datagram(json.dumps({'stop': True}).encode(), 1000 * MS)
        v, w, reason, src, take = c.arbitrate(1010 * MS)
        self.assertEqual((v, w, src, take), (0., 0., 'teleop', True))

    def test_a5_release_hands_back_after_the_dead_man(self):
        c = self.core()
        c.on_datagram(dgram(0.1, 0.0), 1000 * MS)
        self.assertEqual(c.arbitrate(1300 * MS)[3], 'teleop')                  # 300 ms: dead man not passed
        c.on_nav(0.05, 0.1, 1290 * MS)
        v, w, reason, src, take = c.arbitrate(1301 * MS)
        self.assertEqual((v, w, src, take), (0.05, 0.1, 'nav', False))
        self.assertEqual(c.counters['handback'], 1)
        c.on_datagram(dgram(0.1, 0.0), 2000 * MS)                              # a second take-over: a second cancel
        self.assertTrue(c.arbitrate(2001 * MS)[4])
        self.assertEqual(c.counters['takeover'], 2)

    def test_a6_teleop_rules_unchanged_in_hand(self):
        c = self.core(max_cmd_s=1.0)
        c.on_nav(0.2, 0.0, 0)
        c.on_datagram(dgram(5.0, 9.0), 0)                                      # clamped
        self.assertEqual(c.arbitrate(10 * MS)[:2], (0.15, 0.6))
        t = 10
        while t < 1200:                                                        # held key: max duration latches zero
            c.on_datagram(dgram(0.1, 0.0), t * MS)
            c.on_nav(0.2, 0.0, t * MS)
            out = c.arbitrate(t * MS)
            t += 50
        self.assertEqual(out[:4], (0., 0., 'max_duration', 'teleop'))         # still in hand: nav does NOT pass
        # a stale datagram (sent_ns older than the dead man) is ignored: no take-over
        c2 = self.core()
        c2.on_datagram(dgram(0.1, 0.0, sent_ns=0), 400 * MS)
        self.assertEqual(c2.arbitrate(401 * MS)[3], 'idle')

    def test_a6b_non_finite_nav_is_zero(self):
        c = self.core()
        c.on_nav(float('nan'), 0.3, 0)
        self.assertEqual(c.arbitrate(1 * MS)[:2], (0., 0.))
        self.assertEqual(c.counters['nav_non_finite'], 1)

    def test_arbitration_off_is_unchanged(self):
        c = TeleopCore()
        self.assertIsNone(c.arb)
        self.assertEqual(c.status()['mode'], 'teleop')
        self.assertNotIn('takeover', c.counters)
        with self.assertRaises(ValueError):
            TeleopCore().enable_arbitration(0.5)                               # never staler than the driver's 200 ms


class TestRunnerRefusal(unittest.TestCase):
    def test_a7_in_hand_flag_age(self):
        f = ros_runtime.teleop_in_hand
        self.assertFalse(f(None, 10.))
        self.assertTrue(f((True, 10.), 10.5))
        self.assertFalse(f((True, 10.), 10.51))                                # stale: released
        self.assertFalse(f((False, 10.), 10.1))
        self.assertEqual(ros_runtime.TELEOP_REFUSAL, 'teleop_active')

    def test_a8_every_navigation_entry_refuses_first(self):
        src = _read(os.path.join(ROOT, 'host', 'ros_runtime.py'))
        self.assertIn('self.create_subscription(Bool, TELEOP_ACTIVE_TOPIC, self.on_teleop_active, 10)', src)
        for kind in ('query_rejected', 'selection_rejected', 'patrol_rejected', 'goto_rejected'):
            self.assertIn('if self.teleop_active():\n                self.record("%s", reason=TELEOP_REFUSAL)' % kind,
                          src, kind)
        nav = src.split('        def navigate(self, pose, command_id, goto=False):\n', 1)[1]
        self.assertTrue(nav.startswith('            if self.teleop_active():'))
        # the 10-16 refusal line is kept as it was
        self.assertIn('reason = navigation_refusal(self.nav is not None, self.actions.server_is_ready(), self.ready())\n'
                      '            if reason:\n                self.fault(reason)', src)

    def test_a8b_hub_maps_the_runner_refusal_to_the_pc(self):
        from host.hub.hub_core import HubCore
        from host.hub.hub_server import RosActionsLog
        core = HubCore('replay', actions=RosActionsLog())
        ack = core.command('pc', True, {'cmd_id': 'g1', 'op': 'goto', 'args': {'forward_m': 0.5}}, 1000)
        self.assertEqual(ack['status'], 'forwarded')
        core.feed_status(json.dumps({'event': 'goto_rejected', 'reason': 'teleop_active'}), 2000)
        res = [r for _, r in core.results if r.get('cmd_id') == 'g1']
        self.assertEqual(res[-1]['status'], 'rejected')
        self.assertEqual(res[-1]['reason'], 'teleop_active')


class TestHubOps(unittest.TestCase):
    def setUp(self):
        from host.hub.hub_core import HubCore
        from host.hub.hub_server import RosActionsLog
        self.core = HubCore('replay', actions=RosActionsLog())

    def cmd(self, cid, op, args, controller=True, t=1000):
        return self.core.command('pc', controller, {'cmd_id': cid, 'op': op, 'args': args}, t)

    def test_teleop_forwarded_with_sent_ns_and_outside_the_book(self):
        ack = self.cmd('t1', 'teleop', {'v': 0.1, 'w': -0.2}, t=777)
        self.assertEqual(ack['status'], 'forwarded')
        topic, text = self.core.actions.published[-1]
        self.assertEqual(topic, '/fpga/teleop/input')
        self.assertEqual(json.loads(text), {'v': 0.1, 'w': -0.2, 'sent_ns': 777})
        self.assertNotIn(('pc', 't1'), self.core.book)
        self.assertEqual(self.cmd('t2', 'teleop', {'stop': True})['status'], 'forwarded')
        self.assertEqual(json.loads(self.core.actions.published[-1][1])['v'], 0.0)
        self.assertEqual(self.core.watchers, [])

    def test_teleop_refusals(self):
        for args in ({'v': 0.1}, {'v': 0.4, 'w': 0}, {'v': 0, 'w': 1.5}, {'v': float('nan'), 'w': 0},
                     {'v': True, 'w': 0}, {'stop': False}, {'stop': True, 'v': 0.1}):
            self.assertEqual(self.cmd('b', 'teleop', args)['reason'], 'bad_args', args)
        self.assertEqual(self.cmd('n', 'teleop', {'v': 0.1, 'w': 0}, controller=False)['reason'], 'not_controller')
        self.core.estop.latched = True
        self.assertEqual(self.cmd('e', 'teleop', {'v': 0.1, 'w': 0})['reason'], 'estop_latched')
        self.assertEqual(self.cmd('s', 'teleop', {'stop': True})['status'], 'forwarded')   # stopping always allowed

    def test_mapping_stop(self):
        self.assertEqual(self.cmd('m0', 'mapping_stop', {})['reason'], 'no_mapping_status')
        self.core.feed_mapping_status(json.dumps({'state': 'mapping', 'db': '/o/rtabmap_db/x.db'}), 1)
        self.assertEqual(self.cmd('m1', 'mapping_stop', {})['status'], 'forwarded')
        self.assertEqual(self.core.actions.published[-1], ('/fpga/mapping/stop', None))
        self.core.feed_mapping_status(json.dumps({'state': 'saving'}), 2)
        self.assertEqual([r for _, r in self.core.results if r.get('cmd_id') == 'm1'], [])   # saving: not final
        self.core.feed_mapping_status(json.dumps({'state': 'exported', 'yaml': '/o/rtabmap_db/x.yaml',
                                                  'map_loaded': None}), 3)          # exported, load not answered yet
        self.assertEqual([r for _, r in self.core.results if r.get('cmd_id') == 'm1'], [])
        self.core.feed_mapping_status(json.dumps({'state': 'exported', 'yaml': '/o/rtabmap_db/x.yaml',
                                                  'map_loaded': True, 'load_result': 0}), 4)
        res = [r for _, r in self.core.results if r.get('cmd_id') == 'm1']
        self.assertEqual((res[-1]['status'], res[-1]['mapping_state']), ('done', 'exported'))
        self.assertTrue(res[-1]['mapping']['map_loaded'])
        self.assertEqual(self.cmd('m2', 'mapping_stop', {})['reason'], 'mapping_exported')

    def test_pc_client_knows_the_ops(self):
        sys.path.insert(0, os.path.join(ROOT, 'pc'))
        import fpgac
        self.assertIn('teleop', fpgac.OPS)
        self.assertIn('mapping_stop', fpgac.OPS)


class TestWiring(unittest.TestCase):
    def test_a9_guard_output_default_unchanged_and_launches_redirect_only_when_set(self):
        g = _read(os.path.join(ROOT, 'robot_sim/fpga_sim/scripts/velocity_guard.py'))
        self.assertIn('self.declare_parameter("output_topic", "/fpga/cmd_vel_safe")', g)
        nav = _read(os.path.join(ROOT, 'robot_sim/fpga_sim/launch/navigation.launch.py'))
        self.assertIn('(["-p", "output_topic:=" + output] if output else [])', nav)
        sim = _read(os.path.join(ROOT, 'robot_sim/fpga_sim/launch/simulation.launch.py'))
        self.assertIn('if os.environ.get("FPGA_SIM_GUARD_OUTPUT"):', sim)

    def test_teleop_node_publishes_cancel_on_takeover_and_active_every_tick(self):
        src = _read(os.path.join(ROOT, 'host', 'ros_teleop_safe.py'))
        tick = src.split('        def tick(self):\n', 1)[1].split('\n        def ', 1)[0]
        self.assertIn('if takeover:', tick)
        self.assertIn('self.cancel_pub.publish(Empty())', tick)
        self.assertLess(tick.index('self.cancel_pub.publish'), tick.index('self.send(v, w)\n            self.active_pub'))
        self.assertIn('self.active_pub.publish(Bool(data=source == \'teleop\'))', tick)
        # the conflict check (another /fpga/cmd_vel_safe publisher -> exit 3) is unchanged
        self.assertIn("others = self.count_publishers(args.topic) - 1", src)


if __name__ == '__main__':
    unittest.main()
