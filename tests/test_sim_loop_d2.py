"""Line D2 (2026-10-14): simulation loop with YOLO and the chassis chain -- the parts testable without ROS.

1. bottle_scenario readiness (D1 run_b2 diagnosis): the GetState poller times out an unanswered request
   and sends a new one (positive control: the D1 loop never re-sends after one lost answer); readiness
   rows on every change with their time.
2. Bridge: FPGA_SIM_CMD_TOPIC retargets the one ROS -> Gazebo wheel command (bridge.yaml), nothing else.
3. mcu_sim --log: wall_ns on every row; unlock with hb_good_since / last_cmd; latch and the executed
   speed going to zero in the same millisecond (over a real pty, heartbeat file switched off).
4. chain_times + verify_bottle_run --layout sim-cut / single-source parsing on a synthetic run with a known
   answer, and mutants (latch late, wrong cause, already revoked before the cut) that must fail.
5. World / mesh generators: the bottle is replaced, lights per option, outward faces of the lathe.
6. Run scripts: shell syntax, wd2- names, no pkill, the D2 switches wired through.
Python 3.8+, no ROS.
"""
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'robot_sim/tests'))
sys.path.insert(0, str(ROOT / 'tools/sim_loop'))
sys.path.insert(0, str(ROOT / 'robot_sim/fpga_sim/config'))

import scenario_readiness as sr  # noqa: E402
import chain_times as ct  # noqa: E402
import nav_input_profile as nip  # noqa: E402
import bottle_world  # noqa: E402
from tools import verify_bottle_run as vbr  # noqa: E402
from host.chassis_uart import encode_command  # noqa: E402

NS = 1_000_000_000
MSN = 1_000_000


class FakeFuture:
    def __init__(self):
        self._done = False

    def done(self):
        return self._done


class Readiness(unittest.TestCase):
    def test_timeout_and_resend(self):
        p = sr.ServiceStatePoller(period_s=.5, timeout_s=2.0)
        self.assertTrue(p.due(0.0))
        lost = FakeFuture()
        p.sent(0.0, lost)
        self.assertFalse(p.due(1.0))              # one request in flight
        self.assertIsNone(p.check(1.9))
        kind, fut = p.check(2.0)
        self.assertEqual((kind, fut), ('timeout', lost))
        self.assertTrue(p.due(2.0))                # a new request may go at once
        ok = FakeFuture()
        p.sent(2.0, ok)
        ok._done = True
        self.assertEqual(p.check(2.1), ('answer', ok))
        self.assertEqual(p.counts, {'sent': 2, 'answered': 1, 'timeouts': 1})
        self.assertFalse(p.due(2.2))
        self.assertTrue(p.due(2.5))
        with self.assertRaises(RuntimeError):
            p.sent(2.5, FakeFuture())
            p.sent(2.5, FakeFuture())

    def test_positive_control_d1_loop_never_resends(self):
        """The D1 loop: send only when no future is pending; a lost answer blocks it forever."""
        future, sent = None, 0
        for step in range(200):                   # 100 s at 0.5 s
            if future is not None and future.done():
                future = None
            if future is None:
                future, sent = FakeFuture(), sent + 1
        self.assertEqual(sent, 1)
        p, sent2 = sr.ServiceStatePoller(.5, 2.0), 0
        for step in range(200):
            now = step * .5
            p.check(now)
            if p.due(now):
                p.sent(now, FakeFuture())
                sent2 += 1
        self.assertGreaterEqual(sent2, 40)

    def test_readiness_rows_on_change_only(self):
        log = sr.ReadinessLog(('a', 'b'))
        rows = log.update(1.0, 10.0, a=True, b=False)
        self.assertEqual([(r['item'], r['value'], r['previous']) for r in rows], [('a', True, None), ('b', False, None)])
        self.assertEqual(log.update(2.0, 11.0, a=1, b=0), [])
        rows = log.update(3.0, 12.0, b=True)
        self.assertEqual(rows, [{'item': 'b', 'value': True, 'previous': False, 'elapsed_wall': 3.0, 'clock': 12.0}])
        self.assertTrue(log.all_true())
        self.assertEqual(log.snapshot(4.0)['b'], {'value': True, 'for_s': 1.0})
        with self.assertRaises(KeyError):
            log.update(5.0, a=True, c=True)

    def test_scenario_uses_poller_and_logs(self):
        text = (ROOT / 'robot_sim/tests/bottle_scenario.py').read_text()
        self.assertIn('ServiceStatePoller(period_s=.5, timeout_s=args.lifecycle_timeout_s)', text)
        self.assertIn("log('readiness', **change)", text)
        self.assertIn("log('lifecycle_query_timeout'", text)
        self.assertIn("'readiness': readiness.snapshot(now)", text)
        self.assertNotIn('lifecycle_future', text)
        import ast
        ast.parse(text)


class Bridge(unittest.TestCase):
    def entries(self):
        import yaml
        return yaml.safe_load((ROOT / 'robot_sim/fpga_sim/config/bridge.yaml').read_text())

    def test_retarget_wheel_command(self):
        before = self.entries()
        after = nip.command_input(before, '/fpga/chassis/sim_twist')
        to_gz = [e for e in after if e['direction'] == 'ROS_TO_GZ']
        self.assertEqual(len(to_gz), 1)
        self.assertEqual((to_gz[0]['ros_topic_name'], to_gz[0]['gz_topic_name']), ('/fpga/chassis/sim_twist', '/fpga/cmd_vel'))
        self.assertNotIn('/fpga/cmd_vel_safe', [e['ros_topic_name'] for e in after])
        self.assertEqual([e for e in after if e['direction'] != 'ROS_TO_GZ'],
                         [e for e in before if e['direction'] != 'ROS_TO_GZ'])
        self.assertEqual([e['ros_topic_name'] for e in before if e['direction'] == 'ROS_TO_GZ'], ['/fpga/cmd_vel_safe'])

    def test_refuses_without_target(self):
        with self.assertRaises(ValueError):
            nip.command_input(nip.bridge_inputs(self.entries(), 'estimated_inspect'), '/x')
        with self.assertRaises(ValueError):
            nip.command_input(self.entries(), 'relative')

    def test_launch_wires_env(self):
        text = (ROOT / 'robot_sim/fpga_sim/launch/simulation.launch.py').read_text()
        self.assertIn('os.environ.get("FPGA_SIM_CMD_TOPIC")', text)
        self.assertIn("inputs['command_input'](entries, cmd_topic)", text)


class McuLog(unittest.TestCase):
    """mcu_sim.py over a pty: heartbeat file on -> commands -> unlock; heartbeat off -> latch at +31 ms
    with the executed speed zero in the same millisecond; all rows carry wall_ns."""

    def test_latch_unlock_rows(self):
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d, ignore_errors=True)
        hb, log = os.path.join(d, 'hb'), os.path.join(d, 'mcu.jsonl')
        Path(hb).write_text('1')
        sim = subprocess.Popen([sys.executable, str(ROOT / 'tools/chassis_uart/mcu_sim.py'), '--pty', '--seconds', '3',
                                '--calib-s', '0', '--heartbeat-file', hb, '--log', log],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            path = sim.stdout.readline().strip()
            fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
            stop = threading.Event()

            def drain():
                while not stop.is_set():
                    try:
                        os.read(fd, 4096)
                    except OSError:
                        time.sleep(0.005)
            threading.Thread(target=drain, daemon=True).start()
            t0 = time.monotonic()
            seq = 0
            while time.monotonic() - t0 < 1.6:
                if time.monotonic() - t0 > 0.9:
                    Path(hb).write_text('0')
                os.write(fd, encode_command(seq & 0xFF, 200, 0, True))
                seq += 1
                time.sleep(0.02)
            stop.set()
            sim.wait(timeout=10)
            os.close(fd)
            sim.stdout.close()
            sim.stderr.close()
        finally:
            if sim.poll() is None:
                sim.terminate()
                sim.communicate(timeout=10)
        with open(log) as f:
            rows = [json.loads(l) for l in f]
        self.assertTrue(all('wall_ns' in r for r in rows))
        view = ct.mcu_view(rows)
        start = view['start']
        self.assertIsNotNone(start)
        events = view['events']
        unlock = next(e for e in events if e['event'] == 'unlock')
        latch = next(e for e in events if e['event'] == 'latch')
        self.assertGreaterEqual(unlock['t_ms'] - unlock['hb_good_since_ms'], 100)
        self.assertGreater(unlock['last_cmd_ms'], unlock['hb_good_since_ms'])
        self.assertEqual(latch['wall_ns'], start['wall_ns'] + latch['t_ms'] * MSN)
        moving = [r for r in view['executed'] if ct.moving(*r['executed'])]
        # the simulated MCU's receive() first runs the same millisecond's heartbeat tick (the unlock) and only then
        # handles the command, so the first motion may share the unlock's millisecond; what must not be executed is
        # the command frame that made the unlock hold (last_cmd_ms)
        self.assertTrue(moving and moving[0]['t_ms'] >= unlock['t_ms'] and moving[0]['t_ms'] > unlock['last_cmd_ms'])
        zero = next(r for r in view['executed'] if r['t_ms'] >= latch['t_ms'] and not ct.moving(*r['executed']))
        self.assertEqual(zero['t_ms'], latch['t_ms'])
        self.assertEqual(ct.executed_at(view['executed'], latch['t_ms'] - 1), (0.2, 0.0))


def synthetic_cut_run(root, latch_after_ms=35, cause=1, revoked_before_cut=False, heartbeat='kv'):
    """A cut run with a known answer: RTF 0.9; frames DONE every 100 ms until the cut; the permit model
    revokes at DONE + 20 000 000 + 1 cycles; the MCU latches latch_after_ms later, executed speed (0.2, 0)
    -> 0; the car decelerates at 0.5 m/s^2 (odom 30 Hz on simulation time)."""
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    W0, M0, clk = 1_800_000_000 * NS, 5_000 * NS, 100_000_000
    sim_of = lambda w: 100.0 + (w - W0) / NS * 0.9   # noqa: E731
    cut = W0 + 10 * NS
    done_last = cut - 40 * MSN if not revoked_before_cut else cut - 400 * MSN
    revoke = done_last + 200 * MSN + 10
    latch = revoke + latch_after_ms * MSN
    motion = []
    for i in range(0, 4000):
        w = W0 + 5 * NS + i * 5 * MSN
        s = sim_of(w)
        motion.append({'topic': '/clock', 'wall_ns': w, 'wall_monotonic': w / NS,
                       'message': {'clock': {'sec': int(s), 'nanosec': int(round((s - int(s)) * NS))}}})
    latch_sim = sim_of(latch)
    for i in range(0, 600):
        s = 105.0 + i / 30.0
        w = W0 + int((s - 100.0) / 0.9 * NS)
        v = 0.2 if s < latch_sim + 0.02 else max(0.0, 0.2 - 0.5 * (s - latch_sim - 0.02))
        motion.append({'topic': '/odom', 'wall_ns': w, 'wall_monotonic': w / NS,
                       'message': {'header': {'stamp': {'sec': int(s), 'nanosec': int(round((s - int(s)) * NS))}},
                                   'twist': {'twist': {'linear': {'x': v}, 'angular': {'z': 0.0}}}}})
    for i in range(0, 200):                               # Nav2 keeps commanding through the guard
        w = cut - NS + i * 20 * MSN
        motion.append({'topic': '/fpga/cmd_vel_safe', 'wall_ns': w, 'wall_monotonic': w / NS,
                       'message': {'linear': {'x': 0.2}, 'angular': {'z': 0.0}}})
        motion.append({'topic': '/fpga/chassis/sim_twist', 'wall_ns': w, 'wall_monotonic': w / NS,
                       'message': {'linear': {'x': 0.2 if w < latch else 0.0}, 'angular': {'z': 0.0}}})
    motion.sort(key=lambda r: r['wall_ns'])
    telem = []
    count = 3
    dones = list(range(int(W0 + 5 * NS), int(done_last), int(100 * MSN))) + [done_last]
    for i in range(0, 1500):
        w = W0 + 5 * NS + i * 10 * MSN
        mono = w - W0 + M0
        tb = (mono - M0) * clk // NS
        last_done = max([t for t in dones if t <= w] or [W0])
        permit = w < revoke if w >= last_done else False
        rc = count + (1 if w >= revoke else 0)
        row = {'ok': True, 'mono_send_ns': mono - 200_000, 'mono_recv_ns': mono + 200_000,
               'wall_send_ns': w - 200_000, 'wall_recv_ns': w + 200_000,
               'permit': {'available': True, 'status_bits': {'permit': permit, 'revoke_cause': cause if rc > count else 0},
                          'last_cause': cause if rc > count else 0, 'revoke_count': rc,
                          'last_done_ts': (min(last_done, done_last) - W0) * clk // NS,
                          'last_revoke_ts': ((revoke - W0) * clk // NS) if rc > count else 0,
                          'tb_now': tb, 'timeout_cycles': 20_000_000, 'frames_accepted': 10}}
        if i == 0:
            row['pl_clk_hz'] = clk
        telem.append(row)
    # the revoke stamp is exactly DONE + timeout + 1 cycle on the PL axis
    for row in telem:
        if row['permit']['revoke_count'] > count:
            row['permit']['last_revoke_ts'] = row['permit']['last_done_ts'] + 20_000_001
    wall0 = W0
    ms = lambda w: (w - wall0) // MSN   # noqa: E731
    mcu = [{'start': True, 't_ms': 0, 'wall_ns': wall0},
           {'t_ms': ms(W0 + 6 * NS), 'wall_ns': wall0 + ms(W0 + 6 * NS) * MSN, 'event': 'unlock',
            'hb_good_since_ms': ms(W0 + 6 * NS) - 120, 'last_cmd_ms': ms(W0 + 6 * NS) - 5},
           {'t_ms': ms(W0 + 6 * NS) + 20, 'wall_ns': wall0 + (ms(W0 + 6 * NS) + 20) * MSN, 'executed': [0.2, 0.0]}]
    if heartbeat == 'kv':
        mcu += [{'t_ms': ms(latch), 'wall_ns': wall0 + ms(latch) * MSN, 'event': 'latch'},
                {'t_ms': ms(latch), 'wall_ns': wall0 + ms(latch) * MSN, 'executed': [0.0, 0.0]}]
    files = {'motion.jsonl': motion, 'permit_telem.jsonl': telem, 'mcu.jsonl': mcu,
             'scenario.jsonl': [{'event': 'scenario_start', 'machine': 'x86_64'},
                                {'event': 'cut_requested', 'wall_ns': cut, 'clock': sim_of(cut)},
                                {'event': 'scenario_result', 'passed': heartbeat == 'kv',
                                 'reason': 'cut_stopped' if heartbeat == 'kv' else 'cut_not_stopped'}]}
    for name, rows in files.items():
        with open(root / name, 'w') as f:
            for r in rows:
                f.write(json.dumps(r) + '\n')
    (root / 'head.txt').write_text('abc\n')
    (root / 'dirty.txt').write_text('')
    (root / 'run_config.json').write_text(json.dumps({'head': 'abc', 'dirty': False, 'chassis': 'mcu',
                                                      'mcu_heartbeat': heartbeat}))
    (root / 'single_source.txt').write_text(SINGLE_SOURCE)
    return {'cut': cut, 'revoke': revoke, 'latch': latch, 'done': done_last, 'sim_of': sim_of}


SINGLE_SOURCE = """== ign topic -i -t /fpga/cmd_vel
Publishers [Address, Message Type]:
  tcp://127.0.0.1:41011, ignition.msgs.Twist
Subscribers [Address, Message Type]:
  tcp://127.0.0.1:39117, ignition.msgs.Twist
== ros2 topic info -v /fpga/chassis/sim_twist
Type: geometry_msgs/msg/Twist

Publisher count: 1

Node name: fpga_mcu_sim
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Endpoint type: PUBLISHER
GID: 01.0f

Subscription count: 2

Node name: fpga_bridge
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Endpoint type: SUBSCRIPTION

Node name: fpga_passive_motion_recorder
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Endpoint type: SUBSCRIPTION
== ros2 topic info -v /fpga/cmd_vel_safe
Type: geometry_msgs/msg/Twist

Publisher count: 1

Node name: fpga_velocity_guard
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Endpoint type: PUBLISHER

Subscription count: 2

Node name: fpga_chassis_driver
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Endpoint type: SUBSCRIPTION

Node name: fpga_passive_motion_recorder
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Endpoint type: SUBSCRIPTION
"""


class ChainVerifier(unittest.TestCase):
    def test_wall_to_sim(self):
        pairs = [(0, 10.0), (10, 11.0), (100, 20.0)]
        self.assertEqual(ct.wall_to_sim(pairs, 5), 10.5)
        self.assertEqual(ct.wall_to_sim(pairs, 10), 11.0)
        self.assertIsNone(ct.wall_to_sim(pairs, -1))
        self.assertIsNone(ct.wall_to_sim(pairs, 101))
        self.assertIsNone(ct.wall_to_sim(pairs, 50, max_gap_ns=10))

    def test_single_source_parse(self):
        src = vbr.single_source(SINGLE_SOURCE)
        self.assertEqual(src, {'gz_cmd_vel_publishers': 1, 'sim_twist_publishers': ['fpga_mcu_sim'],
                               'cmd_vel_safe_subscribers': ['fpga_chassis_driver', 'fpga_passive_motion_recorder']})
        two = SINGLE_SOURCE.replace('  tcp://127.0.0.1:41011, ignition.msgs.Twist\n',
                                    '  tcp://127.0.0.1:41011, ignition.msgs.Twist\n  tcp://127.0.0.1:41012, ignition.msgs.Twist\n')
        self.assertEqual(vbr.single_source(two)['gz_cmd_vel_publishers'], 2)

    def test_cut_known_answer(self):
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d, ignore_errors=True)
        k = synthetic_cut_run(d)
        out = vbr.verify_sim_cut(Path(d))
        self.assertTrue(out['passed'], {c: v for c, v in out['checks'].items() if not v})
        f = out['facts']
        self.assertEqual(f['revocation']['cause'], 1)
        self.assertAlmostEqual(f['latch_minus_revoke_wall_s'], 0.035, delta=0.002)
        self.assertAlmostEqual(f['chain_wall_s'], 0.235, delta=0.002)
        self.assertAlmostEqual(f['chain_sim_s'], 0.235 * 0.9, delta=0.003)
        self.assertGreater(f['nonzero_cmd_vel_safe_after_latch'], 0)
        self.assertAlmostEqual(f['revoke_sim'], k['sim_of'](k['revoke']), delta=0.002)

    def test_mutant_latch_late_fails(self):
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d, ignore_errors=True)
        synthetic_cut_run(d, latch_after_ms=80)
        out = vbr.verify_sim_cut(Path(d))
        self.assertFalse(out['passed'])
        self.assertFalse(out['checks']['latch_after_revoke_within_bound'])
        self.assertFalse(out['checks']['chain_within_bound_wall'])

    def test_mutant_wrong_cause_fails(self):
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d, ignore_errors=True)
        synthetic_cut_run(d, cause=2)
        out = vbr.verify_sim_cut(Path(d))
        self.assertFalse(out['checks']['revoked_with_cause'])

    def test_mutant_already_revoked_fails_precondition(self):
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d, ignore_errors=True)
        synthetic_cut_run(d, revoked_before_cut=True)
        out = vbr.verify_sim_cut(Path(d))
        self.assertFalse(out['checks']['pre_permit_set'])
        self.assertFalse(out['passed'])

    def test_positive_control_layout(self):
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d, ignore_errors=True)
        synthetic_cut_run(d, heartbeat='always')
        with open(os.path.join(d, 'motion.jsonl')) as f:
            motion = [json.loads(l) for l in f]
        for r in motion:                          # the control's car keeps moving
            if r['topic'] == '/odom':
                r['message']['twist']['twist']['linear']['x'] = 0.2
        with open(os.path.join(d, 'motion.jsonl'), 'w') as f:
            for r in motion:
                f.write(json.dumps(r) + '\n')
        out = vbr.verify_sim_cut(Path(d))
        self.assertTrue(out['passed'], {c: v for c, v in out['checks'].items() if not v})
        self.assertTrue(out['checks']['control_no_latch'])
        self.assertTrue(out['checks']['control_car_moving_1s_after_revoke'])

    def test_bounds_are_the_prereg_sum(self):
        self.assertAlmostEqual(vbr.CHAIN_BOUND_S, 0.251)
        self.assertAlmostEqual(vbr.LATCH_AFTER_REVOKE_S, 0.051)


class YoloProfile(unittest.TestCase):
    D036 = ['bottle', 'cup', 'backpack', 'cell phone']

    def test_torch_and_ort_profiles(self):
        torch_cfg = {'yolo_backend': 'torch', 'yolo_model_sha256': 'a' * 64}
        ex = {'device': 'cuda', 'backend': 'torch', 'precision': 'fp32', 'tf32': False, 'conf': 0.25,
              'model_sha256': 'a' * 64, 'classes': self.D036}
        self.assertTrue(vbr.yolo_profile_ok(torch_cfg, 'cuda', ex))
        for bad in ({'tf32': True}, {'tf32': None}, {'model_sha256': 'b' * 64}, {'conf': 0.2},
                    {'backend': 'ort'}, {'device': 'cpu'}, {'classes': ['bottle']}, {'precision': 'fp16'}):
            self.assertFalse(vbr.yolo_profile_ok(torch_cfg, 'cuda', dict(ex, **bad)), bad)
        self.assertFalse(vbr.yolo_profile_ok(torch_cfg, 'cpu', ex))
        self.assertFalse(vbr.yolo_profile_ok(torch_cfg, 'cuda', None))
        self.assertFalse(vbr.yolo_profile_ok({'yolo_backend': 'torch'}, 'cuda', ex))       # no sha in config
        # ort, including a config written before the torch backend existed (yolo_onnx_sha256 only)
        ort_ex = {'device': 'cpu', 'backend': 'ort', 'precision': 'fp32', 'conf': 0.25,
                  'model_sha256': 'c' * 64, 'classes': self.D036}
        self.assertTrue(vbr.yolo_profile_ok({'yolo_onnx_sha256': 'c' * 64}, 'cpu', ort_ex))
        self.assertTrue(vbr.yolo_profile_ok({'yolo_backend': 'ort', 'yolo_model_sha256': 'c' * 64}, 'cpu', ort_ex))
        self.assertFalse(vbr.yolo_profile_ok({'yolo_onnx_sha256': 'c' * 64}, 'cuda', dict(ort_ex, device='cuda')))
        # D-044 (2026-10-21): the new service default (all COCO minus person) is accepted too
        from perception.fpga_replay.coco_zh import COCO_NAMES
        d044 = [n for n in COCO_NAMES if n != 'person']
        self.assertTrue(vbr.yolo_profile_ok(torch_cfg, 'cuda', dict(ex, classes=d044)))
        self.assertFalse(vbr.yolo_profile_ok(torch_cfg, 'cuda', dict(ex, classes=list(COCO_NAMES))))


class ScenarioFp32(unittest.TestCase):
    def test_cuda_fp32_for_ssdlite_and_yolo_torch(self):
        # bottle_scenario.py imports rclpy at module level: take the helper's source alone (smoke_01 KeyError)
        import ast
        text = (ROOT / 'robot_sim/tests/bottle_scenario.py').read_text()
        node = next(n for n in ast.parse(text).body if isinstance(n, ast.FunctionDef) and n.name == 'cuda_fp32')
        space = {}
        exec(compile(ast.Module(body=[node], type_ignores=[]), 'cuda_fp32', 'exec'), space)
        f = space['cuda_fp32']
        self.assertTrue(f({'device': 'cuda', 'memory_format': 'contiguous', 'cuda_math': 'fp32'}))
        self.assertFalse(f({'device': 'cuda', 'cuda_math': 'tf32'}))
        self.assertTrue(f({'device': 'cuda', 'backend': 'torch', 'precision': 'fp32', 'tf32': False}))
        self.assertFalse(f({'device': 'cuda', 'backend': 'torch', 'precision': 'fp32', 'tf32': True}))
        self.assertFalse(f({'device': 'cuda', 'backend': 'torch', 'precision': 'fp32'}))
        self.assertFalse(f({'device': 'cuda', 'backend': 'trt', 'precision': 'fp16'}))


class StraightTrigger(unittest.TestCase):
    """PREREG §10: e-stop / cut while driving straight; default trigger unchanged (§3/§4 frozen)."""
    def trigger(self):
        import ast, types
        text = (ROOT / 'robot_sim/tests/bottle_scenario.py').read_text()
        node = next(n for n in ast.parse(text).body if isinstance(n, ast.FunctionDef) and n.name == 'trigger_ok')
        space = {}
        exec(compile(ast.Module(body=[node], type_ignores=[]), 'trigger_ok', 'exec'), space)
        return space['trigger_ok'], types.SimpleNamespace

    def test_default_is_frozen_rule(self):
        f, ns = self.trigger()
        a = ns(trigger_min_v=None, trigger_max_w=None)
        self.assertTrue(f({'speed': .6, 'v': 0.0, 'w': -.6}, a))      # rotating in place: fires (as frozen)
        self.assertFalse(f({'speed': .04, 'v': .02, 'w': .02}, a))
        self.assertFalse(f({'speed': None, 'v': None, 'w': None}, a))

    def test_straight_rule(self):
        f, ns = self.trigger()
        a = ns(trigger_min_v=.1, trigger_max_w=.2)
        self.assertFalse(f({'speed': .675, 'v': 0.0, 'w': -.675}, a))  # known different: in-place rotation
        self.assertFalse(f({'speed': .51, 'v': .21, 'w': -.3}, a))     # arc, |w| too large
        self.assertFalse(f({'speed': .17, 'v': .08, 'w': -.09}, a))    # too slow
        self.assertTrue(f({'speed': .27, 'v': .15, 'w': -.12}, a))
        self.assertFalse(f({'speed': .27, 'v': None, 'w': None}, a))

    def test_wired_through_launchers(self):
        inner = (ROOT / 'tools/sim_loop/stack_inner.sh').read_text()
        outer = (ROOT / 'tools/sim_loop/run_sim_loop.sh').read_text()
        self.assertIn('--trigger-min-v "$TRIGGER_MIN_V"', inner)
        self.assertIn('-e TRIGGER_MIN_V="$TRIGGER_MIN_V" -e TRIGGER_MAX_W="$TRIGGER_MAX_W"', outer)
        self.assertIn('--warmup-frames "$INF_WARMUP"', outer)


class DisplayYieldHub(unittest.TestCase):
    """smoke_04: after the command PC (fpgapc find) left, spec O8 handed control back to the display PC
    (listen --yield-control) and a later one-shot `fpgapc release` was rejected not_controller."""

    def run_case(self, display_reyields):
        sys.path.insert(0, str(ROOT / 'tests'))
        import test_hub as th
        from pc import fpgapc
        hub = th.Hub()
        a = b = c = None
        stop = threading.Event()
        try:
            a = th.Client(hub.server.port)
            self.assertTrue(a.c.connected.wait(5))
            self.assertTrue(a.c.hello_ack['controller'])
            y = fpgapc.DisplayYield(min_interval_s=0.2)
            y.client = a.c
            cid = a.c.command('yield_control', {})
            self.assertEqual(a.c.wait_ack(cid, 3)['status'], 'yielded')
            seen = [0]

            def pump():
                while not stop.is_set():
                    with a.lock:
                        new = a.events[seen[0]:]
                        seen[0] = len(a.events)
                    for e in new:
                        y.on_event(e)
                    if display_reyields:
                        y.poll()
                    time.sleep(0.02)
            threading.Thread(target=pump, daemon=True).start()
            b = th.Client(hub.server.port)
            self.assertTrue(b.c.connected.wait(5))
            self.assertTrue(b.c.hello_ack['controller'])          # the command PC holds control
            b.stop()
            self.assertTrue(a.wait(lambda: any(e.get('event') == 'controller_changed'
                                               and e.get('controller_client_id') == a.c.client_id
                                               for e in a.events)))   # spec O8: back to the display PC
            if display_reyields:
                self.assertTrue(a.wait(lambda: y.last is not None and not y.pending.is_set(), 5))
            time.sleep(0.2)
            c = th.Client(hub.server.port)
            self.assertTrue(c.c.connected.wait(5))
            cid = c.c.command('cancel')
            return c.c.wait_ack(cid, 3)
        finally:
            stop.set()
            for x in (c, a):
                if x is not None:
                    x.stop()
            hub.close()

    def test_display_pc_gives_returned_control_up_again(self):
        self.assertEqual(self.run_case(True)['status'], 'forwarded')

    def test_known_different_without_reyield(self):
        self.assertEqual(self.run_case(False)['reason'], 'not_controller')


class Generators(unittest.TestCase):
    BASE = ROOT / 'robot_sim/fpga_sim/worlds/room_carmount.sdf'

    def build(self, **kw):
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d, ignore_errors=True)
        out = os.path.join(d, 'w.sdf')
        args = dict(bottle='wine_green', x=1.25, y=-0.45, yaw=0.0, scale=1.0, light='orig', props_x=None)
        args.update(kw)
        bottle_world.build(str(self.BASE), out, **args)
        return ET.parse(out).getroot().find('world')

    def test_mesh_bottle_replaces_primitive(self):
        world = self.build(bottle='wine_green', props_x=1.25)
        link = world.find(".//model[@name='rendered_props']/link")
        names = [c.get('name') for c in link]
        self.assertFalse(any(n.startswith('bottle_') for n in names))
        self.assertIn('cup_visual', names)
        cup = link.find("visual[@name='cup_visual']/pose").text.split()
        self.assertEqual(cup[0], '1.2500')
        model = world.find("model[@name='bottle_prop']")
        self.assertEqual(model.find('pose').text.split()[:2], ['1.2500', '-0.4500'])
        uri = model.find('.//mesh/uri').text
        self.assertTrue(uri.endswith('props/bottle_wine_green.obj'))
        self.assertTrue((ROOT / 'robot_sim/fpga_sim/worlds/props/bottle_wine_green.obj').exists())
        self.assertEqual(model.find('.//material/diffuse').text, '1 1 1 1')   # not sdformat's black default

    def test_lights(self):
        orig = self.build(light='orig')
        self.assertIsNone(orig.find('scene'))
        self.assertEqual(orig.find("light[@name='sun']/direction").text, '-0.5 0.1 -0.9')
        bright = self.build(light='bright')
        self.assertEqual(bright.find('scene/ambient').text, '0.8 0.8 0.8 1')
        self.assertEqual(bright.find("light[@name='sun']/direction").text, '0.4 0.0 -1.0')

    def test_primitive_moves_only_the_bottle(self):
        world = self.build(bottle='primitive', x=1.0, y=-0.3)
        link = world.find(".//model[@name='rendered_props']/link")
        body = link.find("visual[@name='bottle_body_visual']/pose").text.split()
        self.assertEqual(body[:3], ['1.0000', '-0.3000', '0.11'])
        self.assertIsNone(world.find("model[@name='bottle_prop']"))

    def test_lathe_faces_outward(self):
        sys.path.insert(0, str(ROOT / 'robot_sim/fpga_sim/worlds/props'))
        import make_bottle_mesh as mbm
        prof = mbm.STYLES['wine_green']['profile']
        verts, norms, uvs, faces, h = mbm.lathe(prof, 32)
        self.assertAlmostEqual(h, 0.300)
        bad = 0
        for a, b, c in faces:
            pa, pb, pc = verts[a], verts[b], verts[c]
            u = [pb[i] - pa[i] for i in range(3)]
            v = [pc[i] - pa[i] for i in range(3)]
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            if math.sqrt(sum(x * x for x in n)) < 1e-12:
                continue
            avg = [sum(norms[i][k] for i in (a, b, c)) for k in range(3)]
            if sum(n[k] * avg[k] for k in range(3)) <= 0:
                bad += 1
        self.assertEqual(bad, 0)
        self.assertTrue(all(0 <= t[0] <= 1 and 0 <= t[1] <= 1 for t in uvs))


class Scripts(unittest.TestCase):
    OUTER = ROOT / 'tools/sim_loop/run_sim_loop.sh'
    INNER = ROOT / 'tools/sim_loop/stack_inner.sh'
    PROBE = ROOT / 'tools/sim_loop/render_probe.sh'

    def test_syntax_and_discipline(self):
        for path in (self.OUTER, self.INNER, self.PROBE):
            self.assertEqual(subprocess.run(['bash', '-n', str(path)]).returncode, 0, path)
            code = '\n'.join(l for l in path.read_text().splitlines() if not l.lstrip().startswith('#'))
            self.assertNotIn('pkill', code)
        outer, inner = self.OUTER.read_text(), self.INNER.read_text()
        self.assertIn('IPC=${FPGA_HOME}/dev1/.t/wd2/$TAG', outer)
        self.assertIn('NAME=wd2-probe-$TAG', self.PROBE.read_text())
        # D2 wiring: permit model with the MCU chain, one wheel-command source, YOLO checked
        self.assertIn('--permit-model "$1"', outer)
        self.assertIn('PERMIT_MODEL=realtime', outer)
        self.assertIn('-e FPGA_SIM_CMD_TOPIC=/fpga/chassis/sim_twist -e FPGA_SIM_ESTOP_TOPIC=', outer)
        self.assertIn('--detector yolo --onnx "$YOLO_ONNX" --threads "$YOLO_THREADS"', outer)
        self.assertIn('INF_ARGS=(--detector yolo --pt "$YOLO_PT")', outer)
        self.assertIn('"${YOLO_BACKEND:=torch}"', outer)
        self.assertIn('--detector "$DETECTOR"', inner)
        self.assertIn('--expect-detector "$DETECTOR"', inner)
        self.assertIn('--heartbeat-kv "127.0.0.1:$KVPORT"', inner)
        self.assertIn('--twist-topic /fpga/chassis/sim_twist', inner)
        self.assertIn('python3 -m host.ros_chassis_driver --port "$PTY"', inner)
        self.assertIn('echo $! > "$O/producer.pid"', inner)
        self.assertIn('if [ "$CHASSIS" = guard ]; then', inner)   # sim_estop_gate only with the D1 guard

    def test_defaults_refuse_bad_combinations(self):
        env = dict(os.environ)
        for extra, mode, why in (({'DETECTOR': 'yolo', 'YOLO_BACKEND': 'ort', 'INF_DEVICE': 'cuda'}, 'bottle', 'INF_DEVICE=cpu'),
                                 ({'DETECTOR': 'yolo', 'INF_DEVICE': 'cpu'}, 'bottle', 'INF_DEVICE=cuda'),
                                 ({'YOLO_BACKEND': 'trt'}, 'bottle', 'YOLO_BACKEND must be'),
                                 ({'CHASSIS': 'mcu'}, 'nav', 'CHASSIS=mcu needs frames'),
                                 ({'CHASSIS': 'guard'}, 'cut', 'cut mode needs CHASSIS=mcu'),
                                 ({'MCU_HEARTBEAT': 'sometimes'}, 'bottle', 'MCU_HEARTBEAT must be')):
            r = subprocess.run(['bash', str(self.OUTER), mode, '/nonexistent/x'], env=dict(env, **extra),
                               capture_output=True, text=True, timeout=30)
            self.assertEqual(r.returncode, 2, (extra, r.stderr))
            self.assertIn(why, r.stderr)
            self.assertFalse(os.path.exists('/nonexistent/x'))


if __name__ == '__main__':
    unittest.main()
