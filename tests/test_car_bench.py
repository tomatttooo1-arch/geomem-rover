"""Line 4 bench (2026-10-15): tools/car_run/camera_check.py judge(), tools/car_run/bench_pose.py (edges, identity,
stand-in map), tools/car_run/bench_line4_readout.py (criteria C1-C6, "not judged", the find-only rerun) and
tools/car_run/bench_line4.sh run --dry-run (no ROS, no docker, no board). Python 3.8 compatible.

Fixture provenance (the readout's inputs are built the way their writers write them, not from the readout):
  runtime.jsonl  host/ros_runtime.py record() (event, time + fields): "command" = {**Command.__dict__, "pose":
                 PoseGoal.__dict__} (execute(), ~l.1386; Command / PoseGoal from host/task_control.py, imported here),
                 "task" = state/reason/target_id/candidates[id,label,color,position_m,state] (publish_task(), ~l.1488),
                 "frame_processed" tracks/detections (~l.1608), "fault" reason (fault(), ~l.1010),
                 "query_rejected" reason (query(), ~l.1341)
  find.txt       pc/fpgapc.py cmd_command: 'ACK <json>' / 'RESULT <json>' (~l.186); bodies as host/hub/hub_core.py
                 (ACK {"status": "forwarded", "query_seq"} ~l.790, RESULT {"status": "accepted", "task_state"} ~l.825)
  pc_mem.fpgarec  pc/fpgac.py Recorder + encode_json(KIND["MEMORY"], ...); the body as host/memory_publish.py MemoryFeed
  frames.jsonl   host/ros_geo_producer.py stats row (frame_id, acquired_ns, age_ms; ~l.534)
  rc.txt         produced by the real car_common.sh car_stop_pid on stand-in processes (as car_inner.sh's cleanup)
  down_rc.txt    car_down.sh: "<container> exit=<code>"; marks "<epoch> text" as car_inner.sh mark()
"""
import json
import os
import re
import shlex
import shutil
import stat
import subprocess
import sys
import tempfile
import textwrap
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
CAR = os.path.join(ROOT, 'tools', 'car_run')
READOUT = os.path.join(CAR, 'bench_line4_readout.py')
BENCH = os.path.join(CAR, 'bench_line4.sh')
COMMON = os.path.join(CAR, 'car_common.sh')

from host.task_control import Command, PoseGoal  # noqa: E402
from pc import fpgac  # noqa: E402
from tools.car_run import bench_pose, camera_check  # noqa: E402
from tools.car_run import bench_line4_readout as RO  # noqa: E402


# ------------------------------------------------------------------------------------------------ camera_check
class TestCameraCheck(unittest.TestCase):
    F = 'camera_color_optical_frame'

    def rows(self):
        color = {'frame_id': self.F, 'width': 640, 'height': 480, 'encoding': 'rgb8'}
        depth = {'frame_id': self.F, 'width': 640, 'height': 480, 'encoding': '16UC1'}
        info = {'frame_id': self.F, 'width': 640, 'height': 480, 'distortion_model': 'plumb_bob',
                'd': [0.0] * 5, 'k': [500.0, 0, 320, 0, 500, 240, 0, 0, 1]}
        return color, depth, info

    def test_accepts_the_aligned_depth_wiring(self):
        self.assertEqual(camera_check.judge(*self.rows(), frame=self.F), [])
        c, d, i = self.rows()
        c['encoding'], d['encoding'] = 'bgr8', '32FC1'
        self.assertEqual(camera_check.judge(c, d, i, self.F), [])

    def test_each_rule_refuses(self):
        cases = []
        c, d, i = self.rows(); d['frame_id'] = 'camera_depth_optical_frame'; cases.append(((c, d, i), 'depth frame_id'))
        c, d, i = self.rows(); i['frame_id'] = 'camera_link'; cases.append(((c, d, i), 'info frame_id'))
        c, d, i = self.rows(); i['d'] = [0.1, -0.05, 0, 0, 0]; cases.append(((c, d, i), 'distortion'))   # colour info
        c, d, i = self.rows(); i['width'], i['height'] = 1280, 720; cases.append(((c, d, i), 'CameraInfo 1280x720'))
        c, d, i = self.rows(); d['height'] = 400; cases.append(((c, d, i), 'not aligned'))
        c, d, i = self.rows(); c['encoding'] = 'yuyv'; cases.append(((c, d, i), 'color encoding'))
        c, d, i = self.rows(); d['encoding'] = 'mono8'; cases.append(((c, d, i), 'depth encoding'))
        for (c, d, i), word in cases:
            bad = camera_check.judge(c, d, i, self.F)
            self.assertEqual(len(bad), 1, (word, bad))
            self.assertIn(word, bad[0])
        # the frame the pre-10-15 default named: all three topics refused
        self.assertEqual(len(camera_check.judge(*self.rows(), frame='camera_depth_optical_frame')), 3)

    def test_no_ros_is_exit_2(self):
        try:
            import rclpy  # noqa: F401
            self.skipTest('rclpy present')
        except ImportError:
            pass
        self.assertEqual(camera_check.main(['--color-topic', 'a', '--depth-topic', 'b', '--info-topic', 'c',
                                            '--frame', self.F, '--timeout-s', '0.1']), 2)


# ------------------------------------------------------------------------------------------------ bench_pose
class TestBenchPose(unittest.TestCase):
    def test_static_edges_identity_no_camera_edge(self):
        p = bench_pose.plan(0, 8.0, 0.05)
        self.assertEqual(p['static_edges'], [['map', 'odom'], ['odom', 'base_link']])
        self.assertEqual(p['dynamic_edges'], [])
        static, dynamic = bench_pose.transforms(p)
        self.assertEqual(dynamic, [])
        for parent, child, T in static:
            self.assertEqual(T.tolist(), [[1.0 if r == c else 0.0 for c in range(4)] for r in range(4)])
        # the camera mount edge is car_inner.sh's static_transform_publisher, never this node's (one publisher per edge)
        frames = {f for e in p['static_edges'] for f in e}
        self.assertFalse(any('camera' in f for f in frames))
        self.assertIn('不是建图定位', p['note'])

    def test_message_is_identity(self):
        """through the very tf_msg the node uses: zero translation, unit quaternion"""
        from tools.car_geo_producer.replay_camera import tf_msg

        class NS(object):
            pass

        def TransformStamped():
            m = NS()
            m.header, m.transform = NS(), NS()
            m.transform.translation, m.transform.rotation = NS(), NS()
            return m
        static, _ = bench_pose.transforms(bench_pose.plan(0, 8.0, 0.05))
        for parent, child, T in static:
            m = tf_msg(TransformStamped, parent, child, T, 'now')
            t, q = m.transform.translation, m.transform.rotation
            self.assertEqual((m.header.frame_id, m.child_frame_id), (parent, child))
            self.assertEqual((t.x, t.y, t.z), (0.0, 0.0, 0.0))
            self.assertEqual((q.x, q.y, q.z, q.w), (0.0, 0.0, 0.0, 1.0))

    def test_dynamic_odom_and_map(self):
        p = bench_pose.plan(30.0, 8.0, 0.05)
        self.assertEqual(p['static_edges'], [['map', 'odom']])
        self.assertEqual(p['dynamic_edges'], [['odom', 'base_link']])
        self.assertEqual(p['map'], {'frame': 'map', 'width': 160, 'height': 160, 'resolution': 0.05,
                                    'origin': [-4.0, -4.0], 'free': True, 'stand_in': True})
        w, h, ox, oy, data = bench_pose.free_grid(8.0, 0.05)
        self.assertEqual((w, h, ox, oy, len(data), set(data)), (160, 160, -4.0, -4.0, 25600, {0}))
        self.assertIsNone(bench_pose.plan(0, 0, 0.05)['map'])
        self.assertIsNone(bench_pose.free_grid(0, 0.05))
        self.assertRaises(ValueError, bench_pose.free_grid, 8.0, 0)
        self.assertRaises(ValueError, bench_pose.free_grid, 0.01, 0.05)


# ------------------------------------------------------------------------------------------------ readout fixture
T0 = 1760000000.0
RAW0 = 10 ** 12
EXPECT = (1.0, 0.1)
STARTED = ['camera', 'mount_tf', 'bench_pose', 'costmap', 'lifecycle', 'hub', 'bridge', 'producer', 'runtime', 'runtime_info']


def stop_rc_text(tmp, names, stubborn=None):
    """rc.txt as car_inner.sh's cleanup writes it: the real car_stop_pid, children stopped in reverse start order.
    stubborn: a child that ignores INT and TERM (needs KILL)."""
    # 10-17 (docs/design/LOAD_TIMING_TESTS_20261017.md): a `&` child of this non-interactive bash starts with SIGINT
    # ignored, so an INT sent before its signal.signal() is lost. Instead of a fixed 0.5 s each stand-in creates a
    # marker right after setting its handlers and the script waits for every marker (bounded, rc 3) first.
    ready = '\nopen(sys.argv[1], "w").close()\nwhile True: time.sleep(1)'
    code = 'import signal,sys,time\nsignal.signal(signal.SIGINT, signal.default_int_handler)' + ready
    hard = ('import signal,sys,time\nsignal.signal(signal.SIGINT, signal.SIG_IGN)\nsignal.signal(signal.SIGTERM, signal.SIG_IGN)'
            + ready)
    rc = os.path.join(tmp, 'rc_%s.txt' % (stubborn or 'plain'))
    mark = os.path.join(tmp, 'ready_%s_' % (stubborn or 'plain'))
    lines = ['source %s' % COMMON, 'export CAR_RC_FILE=%s CAR_INT_WAIT_S=1 CAR_TERM_WAIT_S=1' % rc, 'P=(); S=()']
    for n in names:
        lines.append('python3 -c %s %s%s & P+=($!)' % (shlex.quote(hard if n == stubborn else code), mark, n))
    lines.append('for f in %s; do for ((i=0; i<1500; i++)); do [ -e "$f" ] && break; sleep 0.02; done; '
                 '[ -e "$f" ] || { echo "stand-in not ready: $f" >&2; kill -KILL "${P[@]}"; exit 3; }; done'
                 % ' '.join(mark + n for n in names))
    lines.append('N=(%s)' % ' '.join(names))
    lines.append('for ((k=${#P[@]}-1; k>=0; k--)); do car_stop_pid "${N[$k]}" "${P[$k]}" "$(car_pid_start ${P[$k]})"; done')
    r = subprocess.run(['bash', '-c', '\n'.join(lines)], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                       universal_newlines=True, timeout=120)
    assert r.returncode == 0, r.stderr
    with open(rc) as f:
        return f.read()


def track(tid, label, xyz):
    return {'id': tid, 'label': label, 'color': None, 'position_m': list(xyz), 'state': 'current_confirmed',
            'reason': None, 'last_seen': T0 + 258.0, 'last_seen_frame': 'f2580'}


def runtime_row(event, t, **fields):
    row = {'event': event, 'time': t, 'pose_source': 'reference', 'sensor_source': 'hardware',
           'inference_device': 'cuda', 'clock_source': 'system_ros', 'inference_backend': None}
    row.update(fields)
    return row


def command_row(t, kind, target_id, x=0.6, y=0.1):
    c = Command(kind, 'sess:1', PoseGoal(x, y, 0.0) if kind == 'navigate' else None, target_id, 'reobserve')
    d = dict(c.__dict__)
    d['pose'] = c.pose.__dict__ if c.pose else None
    return runtime_row('command', t, command=d)


def write(path, text):
    d = os.path.dirname(path)
    if not os.path.isdir(d):
        os.makedirs(d)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)


def jdump(path, obj):
    write(path, json.dumps(obj, ensure_ascii=False))


class Session(object):
    """a pulled bench session directory, every file as its writer writes it; overrides break one thing each."""

    def __init__(self, root, rc_plain, rc_killed):
        self.root, self.rc_plain, self.rc_killed = root, rc_plain, rc_killed
        self.cycles = []

    def cycle(self, name, kind='formal', pose='static', **o):
        self.cycles.append('%s %s %s' % (name, kind, pose))
        write(os.path.join(self.root, 'cycles.txt'), '\n'.join(self.cycles) + '\n')
        run = os.path.join(self.root, 'nano', 'runs', name)
        side = run + '.side'
        stack = 'wde-car-stack-' + name
        jdump(os.path.join(run, 'car.json'), {'pose': pose, 'containers': {'stack': stack, 'gz': '', 'kv': ''},
                                               'pose_note': '台架静态位姿'})
        # C1 / C2
        write(os.path.join(side, 'up_rc'), o.get('up_rc', '0') + '\n')
        write(os.path.join(side, 'up_t0'), '%.6f\n' % T0)
        write(os.path.join(run, 'up_marks.txt'), '%.6f car_up camera=live\n%.6f READY\n' % (T0 + 1, T0 + o.get('ready_s', 100.0)))
        write(os.path.join(side, 'down_rc'), o.get('down_rc', '0') + '\n')
        jdump(os.path.join(run, 'down.json'), {'clean': o.get('down_clean', True), 'containers_left': []})
        jdump(os.path.join(side, 'residue.json'), {'car_down_rc': 0, 'wde_car_containers': [], 'pointer': None,
                                                    'host_processes_alive': False, 'clean': o.get('residue_clean', True)})
        # C3: 10 Hz window of 120 s, every frame fused, age 120 ms (+ one over 200 when asked)
        s_wall, e_wall = T0 + 130.0, T0 + 250.0
        write(os.path.join(side, 'window.txt'), 'window_start_raw_ns=%d\nwindow_end_raw_ns=%d\nwindow_start_wall=%.6f\n'
              'window_end_wall=%.6f\nwarm_s=20\nwindow_s=120\nstack_running_at_start=true\nstack_running_at_end=true\n'
              'stack_fail_at_end=\n' % (RAW0, RAW0 + 120 * 10 ** 9, s_wall, e_wall))
        frames = [json.dumps({'event': 'start', 'data_source': 'live'})]
        n = o.get('frames', 1200)
        for k in range(n):
            age = 250.0 if (o.get('over200') and k == 600) else 120.0
            frames.append(json.dumps({'frame_id': 'f%d' % k, 'depth_stamp_ns': k, 'acquired_ns': RAW0 + k * 10 ** 8 + 1,
                                      'waited_ms': 1.0, 'age_ms': age}))
        write(os.path.join(run, 'frames.jsonl'), '\n'.join(frames) + '\n')
        # runtime events: frames in the window (with detections unless dark), then the query and its answer
        rt = []
        dets = [] if o.get('dark') else [{'label': 'bottle', 'score': 0.6, 'bbox': [1, 2, 3, 4]}]
        trk = [track('t1', 'bottle', o.get('runner_xyz', (1.05, 0.12, 0.08)))]
        for k in range(0, 1200, 10):
            rt.append(runtime_row('frame_processed', s_wall + k / 10.0, frame_id='f%d' % k, detections=dets,
                                  tracks=trk, geometry={'frame_age_ms': 130.0}))
        t_q = T0 + 260.0
        write(os.path.join(side, 'find_sent'), '%.6f\n' % t_q)
        rt.append(runtime_row('frame_processed', t_q - 0.3, frame_id='f9999', detections=dets, tracks=trk))
        answer = o.get('answer', 'hit')
        if answer == 'rejected':
            rt.append(runtime_row('query_rejected', t_q + 0.05, reason='recent processed input, TF and map required'))
        elif answer == 'search':        # no memory hit: the task searches (navigate with no target)
            rt.append(command_row(t_q + 0.05, 'navigate', None))
        elif answer == 'ambiguous':
            rt.append(runtime_row('task', t_q + 0.05, state='ambiguous', reason='candidate_selection_required',
                                  target_id=None, candidates=[dict((k, v) for k, v in trk[0].items() if k in
                                                                   ('id', 'label', 'color', 'position_m', 'state')),
                                                              dict(track('t2', 'bottle', (3.0, -1.0, 0.1)),
                                                                   state='current_confirmed')],
                                  processed_frames=1300, dropped_frames=0))
        else:   # one remembered bottle: navigate to view it -> no NavigateToPose server on the bench -> fault, task failed
            rt.append(command_row(t_q + 0.05, 'navigate', 't1'))
            rt.append(command_row(t_q + 0.06, 'stop', 't1'))
            rt.append(runtime_row('fault', t_q + 0.06, reason='navigation_server_unavailable'))
            rt.append(runtime_row('task', t_q + 0.07, state='failed',
                                  reason='navigation_server_unavailable', target_id='t1',
                                  candidates=[], processed_frames=1300, dropped_frames=0))
        write(os.path.join(run, 'runtime.jsonl'), '\n'.join(json.dumps(r, ensure_ascii=False) for r in rt) + '\n')
        res = ({'v': 1, 'cmd_id': 7, 'op': 'find', 'final': True, 'status': 'rejected',
                'reason': 'recent processed input, TF and map required'} if answer == 'rejected' else
               {'v': 1, 'cmd_id': 7, 'op': 'find', 'final': True, 'status': 'accepted', 'task_state': 'navigating'})
        write(os.path.join(side, 'find.txt'), 'ACK %s\nRESULT %s\nfind_rc=0\n' % (
            json.dumps({'v': 1, 'cmd_id': 7, 'op': 'find', 'nano_mono_ns': 5, 'status': 'forwarded', 'query_seq': 1}),
            json.dumps(res, ensure_ascii=False)))
        # the PC side's MEMORY frames
        rec = fpgac.Recorder(os.path.join(run, 'pc_mem.fpgarec'))
        rec.hello({'hub_boot_id': 'b'}, '127.0.0.1')
        for seq, xyz in enumerate([(9.0, 9.0, 0.0), o.get('pc_xyz', (1.04, 0.11, 0.08))]):
            body = {'v': 1, 'runtime_boot_id': 'r', 'revision': seq, 'stamp_ros_s': T0 + 255 + seq, 'world_frame': 'map',
                    'sensor_source': 'hardware', 'strategy': 'x', 'target_label': None, 'cfg': {}, 'digest': 'd',
                    'tracks': [track('t1', 'bottle', xyz)]}
            rec.write(seq, fpgac.encode_json(fpgac.KIND['MEMORY'], seq, body))
        rec.close()
        # supervision / stop records
        pids = ''.join('%s %d 1%03d\n' % (nm, 100 + k, k) for k, nm in enumerate(STARTED))
        write(os.path.join(run, 'pids.txt'), pids)
        mk = ['%.6f start camera=live pose=%s' % (T0 + 2, pose), '%.6f ready producer' % (T0 + 80), '%.6f READY' % (T0 + 99)]
        if kind == 'normal_stop':
            mk += ['%.6f cleanup' % (T0 + 280), '%.6f stopped' % (T0 + 292)]
            if o.get('died'):
                mk.insert(3, '%.6f EXITED bridge rc=1' % (T0 + 200))
            write(os.path.join(run, 'rc.txt'), o.get('rc_txt', self.rc_plain))
            write(os.path.join(run, 'down_rc.txt'), '%s exit=%s\n' % (stack, o.get('stack_exit', '130')))
            write(os.path.join(run, 'down_marks.txt'), '%.6f car_down\n%.6f %s: INT\n%.6f %s: removed\n' % (
                T0 + 280, T0 + 280, stack, T0 + 293, stack))
            write(os.path.join(side, 'down_t0'), '%.6f\n' % (T0 + 280))
            write(os.path.join(side, 'down_t1'), '%.6f\n' % (T0 + 294))
        else:
            t_kill = T0 + 270.0
            detect = o.get('detect_s', 1.1)
            fail_text = o.get('fail_text', 'producer exited after READY (rc=137): fail-stop')
            mk += ['%.6f EXITED producer rc=137' % (t_kill + detect - 0.01), '%.6f FAIL %s' % (t_kill + detect, fail_text),
                   '%.6f cleanup' % (t_kill + detect), '%.6f stopped' % (t_kill + 9)]
            write(os.path.join(run, 'stack_fail.txt'), fail_text + '\n')
            jdump(os.path.join(side, 'failstop.json'), {
                'node': 'producer', 'pid_in_container': 107, 'signal': 'KILL', 'kill_rc': o.get('kill_rc', 0),
                't_kill_wall': t_kill, 't_stack_fail_seen_wall': t_kill + detect + 0.1,
                't_stack_exited_seen_wall': t_kill + o.get('stop_s', 9.5), 'wait_s': 90,
                'stack_fail': fail_text, 'docker_finished_at_exit_code': 'x 5', 'poll_s': 0.2})
            write(os.path.join(run, 'rc.txt'), self.rc_plain)
        write(os.path.join(run, 'marks.txt'), '\n'.join(mk) + '\n')
        return run, side


class TestReadout(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base = tempfile.mkdtemp(prefix='bench4_')
        cls.rc_plain = stop_rc_text(cls.base, STARTED)
        cls.rc_killed = stop_rc_text(cls.base, STARTED, stubborn='hub')

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.base, ignore_errors=True)

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='s_', dir=self.base)
        self.s = Session(self.tmp, self.rc_plain, self.rc_killed)

    def full(self, formal=None, normal=None, n=3):
        for k in range(1, n + 1):
            self.s.cycle('b4_c%d' % k, **((formal or {}) if k == 2 else {}))   # the override hits the middle cycle
        self.s.cycle('b4_ns', 'normal_stop', **(normal or {}))

    def readout(self, *extra, expect=True):
        out = os.path.join(self.tmp, 'readout.json')
        args = [self.tmp, '--find-text', '瓶子', '--json', out] + list(extra)
        if expect:
            args += ['--expect', 'bottle %s %s' % EXPECT]
        from io import StringIO
        import contextlib
        buf = StringIO()
        with contextlib.redirect_stdout(buf):
            rc = RO.main(args)
        with open(out, encoding='utf-8') as f:
            return rc, json.load(f)

    def test_rc_fixture_is_real(self):
        """positive control of the fixture: the real car_stop_pid wrote one final line per child, reverse order"""
        order, lines = RO.rc_children(self.rc_plain)
        self.assertEqual(order, STARTED[::-1])
        self.assertTrue(all(any('_rc=' in l for l in lines[n]) for n in STARTED))
        self.assertNotIn('already exited', self.rc_plain)          # every stand-in was alive and stopped by INT
        self.assertNotIn('TERM', self.rc_plain)
        order, lines = RO.rc_children(self.rc_killed)
        self.assertTrue(any('KILL' in l for l in lines['hub']))

    def test_all_pass(self):
        self.full()
        rc, d = self.readout()
        self.assertEqual(rc, 0, json.dumps(d['verdict']))
        self.assertTrue(d['all_pass'])
        self.assertEqual(set(d['verdict']), {'c1_start_stop', 'c2_ready', 'c3_frame_age', 'c4_failstop', 'c5_find',
                                             'c6_normal_stop', 'complete'})
        c = d['cycles']['b4_c1']
        self.assertEqual(c['c3_frame_age']['fused'], 1200)
        self.assertEqual(c['c5_find']['b_memory_hit']['hit']['via'], 'command_target')
        self.assertEqual(c['c5_find']['fault_after_query'], 'navigation_server_unavailable')
        self.assertNotIn('c4_failstop', d['normal_stop_cycles']['b4_ns'])
        self.assertEqual(d['normal_stop_cycles']['b4_ns']['c6_normal_stop']['down_s'], 14.0)
        self.assertEqual(c['window_detections'], {'frames': 120, 'frames_with_detection': 120, 'detections': 120,
                                                  'by_label': {'bottle': 120}})

    def fails_only(self, key, formal=None, normal=None, n=3):
        self.full(formal, normal, n)
        rc, d = self.readout()
        self.assertEqual(rc, 1)
        bad = sorted(k for k, v in d['verdict'].items() if v is not True and k != 'complete')
        self.assertEqual(bad, [key], json.dumps(d['verdict']))
        return d

    def test_c1_negatives(self):
        self.fails_only('c1_start_stop', formal={'down_clean': False})

    def test_c1_residue_negative(self):
        self.fails_only('c1_start_stop', formal={'residue_clean': False})

    def test_c2_over_budget(self):
        d = self.fails_only('c2_ready', formal={'ready_s': 241.0})
        self.assertEqual(d['cycles']['b4_c2']['c2_ready']['ready_s'], 241.0)

    def test_c3_one_frame_over_200(self):
        d = self.fails_only('c3_frame_age', formal={'over200': True})
        self.assertEqual(d['cycles']['b4_c2']['c3_frame_age']['over_200'], 1)
        self.assertTrue(d['cycles']['b4_c2']['c3_frame_age']['valid'])

    def test_c3_too_few_frames_is_invalid_not_zero_over(self):
        d = self.fails_only('c3_frame_age', formal={'frames': 1000})
        r = d['cycles']['b4_c2']['c3_frame_age']
        self.assertEqual((r['valid'], r['over_200']), (False, 0))

    def test_c4_negatives(self):
        self.fails_only('c4_failstop', formal={'fail_text': 'runtime: no processed frame in 180 s'})

    def test_c4_slow_detection(self):
        self.fails_only('c4_failstop', formal={'detect_s': 3.5})

    def test_c5_query_rejected(self):
        d = self.fails_only('c5_find', formal={'answer': 'rejected'})
        c5 = d['cycles']['b4_c2']['c5_find']
        self.assertFalse(c5['a_hub_answer']['pass'])
        self.assertEqual(c5['b_memory_hit']['first_runner_answer']['event'], 'query_rejected')

    def test_c5_search_is_no_memory_hit(self):
        d = self.fails_only('c5_find', formal={'answer': 'search'})
        self.assertFalse(d['cycles']['b4_c2']['c5_find']['b_memory_hit']['pass'])

    def test_c5_position_off(self):
        d = self.fails_only('c5_find', formal={'runner_xyz': (1.35, 0.1, 0.0)})     # 0.35 m off in x
        c5 = d['cycles']['b4_c2']['c5_find']
        self.assertEqual((c5['b_memory_hit']['pass'], c5['c_position']['pass']), (True, False))

    def test_c5_pc_side_off(self):
        d = self.fails_only('c5_find', formal={'pc_xyz': (1.0, 0.5, 0.0)})
        self.assertFalse(d['cycles']['b4_c2']['c5_find']['d_pc_memory']['pass'])

    def test_c5_ambiguous_hit_passes(self):
        self.full(formal={'answer': 'ambiguous'})
        rc, d = self.readout()
        self.assertEqual(rc, 0, json.dumps(d['verdict']))
        self.assertEqual(d['cycles']['b4_c2']['c5_find']['b_memory_hit']['hit']['via'], 'ambiguous')

    def test_c5_not_judged_without_measured_position(self):
        """no BENCH_OBJ: the query runs and is recorded; C5 is neither pass nor fail, all_pass is not true (a
        non-empty string must never count as a pass), exit 3; the dark-room note and detection count recorded"""
        for k in range(1, 4):
            self.s.cycle('b4_c%d' % k, dark=True, answer='search')
        self.s.cycle('b4_ns', 'normal_stop', dark=True, answer='search')
        rc, d = self.readout('--site-note', '现场无光照', expect=False)
        self.assertEqual(rc, 3)
        self.assertEqual(d['verdict']['c5_find'], 'not_judged')
        self.assertFalse(d['all_pass'])
        self.assertFalse(d['any_fail'])
        self.assertEqual(d['not_judged'], ['c5_find'])
        for k in ('c1_start_stop', 'c2_ready', 'c3_frame_age', 'c4_failstop', 'c6_normal_stop'):
            self.assertIs(d['verdict'][k], True, k)
        c5 = d['cycles']['b4_c1']['c5_find']
        self.assertEqual(c5['pass'], 'not_judged')
        self.assertTrue(c5['note'].startswith('未判（缺量得位置；现场无光照）'), c5['note'])
        self.assertIsNotNone(c5['b_memory_hit']['first_runner_answer'])          # the answer is recorded
        self.assertEqual(d['cycles']['b4_c1']['window_detections']['detections'], 0)
        # a failing criterion still makes it a failure (exit 1), not "not judged"
        write(os.path.join(self.tmp, 'nano', 'runs', 'b4_c2', 'up_marks.txt'), '%.6f READY\n' % (T0 + 300.0))
        rc, d = self.readout(expect=False)
        self.assertEqual(rc, 1)
        self.assertIs(d['verdict']['c2_ready'], False)

    def test_missing_cycle_fails(self):
        self.full(n=2)
        rc, d = self.readout()
        self.assertEqual(rc, 1)
        self.assertFalse(d['verdict']['complete'])
        self.assertIs(d['verdict']['c1_start_stop'], False)

    def test_c6_wrong_order(self):
        lines = self.rc_plain.splitlines()
        swapped = '\n'.join(lines[1:2] + lines[:1] + lines[2:]) + '\n'
        d = self.fails_only('c6_normal_stop', normal={'rc_txt': swapped})
        self.assertFalse(d['normal_stop_cycles']['b4_ns']['c6_normal_stop']['c_in_order']['pass'])

    def test_c6_kill_needed(self):
        d = self.fails_only('c6_normal_stop', normal={'rc_txt': self.rc_killed})
        self.assertEqual(d['normal_stop_cycles']['b4_ns']['c6_normal_stop']['c_in_order']['killed'], ['hub'])

    def test_c6_child_missing(self):
        lines = [l for l in self.rc_plain.splitlines() if not l.startswith('bridge')]
        self.fails_only('c6_normal_stop', normal={'rc_txt': '\n'.join(lines) + '\n'})

    def test_c6_rc_not_normal(self):
        self.fails_only('c6_normal_stop', normal={'stack_exit': '137'})

    def test_c6_died_before_stop(self):
        self.fails_only('c6_normal_stop', normal={'died': True})

    def test_c6_residue(self):
        self.fails_only('c6_normal_stop', normal={'down_clean': False})

    def test_c6_missing_cycle_and_switched_off(self):
        for k in range(1, 4):
            self.s.cycle('b4_c%d' % k)
        rc, d = self.readout()
        self.assertEqual(rc, 1)
        self.assertIs(d['verdict']['c6_normal_stop'], False)
        rc, d = self.readout('--normal-stops', '0')
        self.assertEqual(rc, 0)
        self.assertNotIn('c6_normal_stop', d['verdict'])

    def test_find_only_rerun(self):
        """--cycles 0: one normal-stop cycle; C5 judged on it, C6 as usual, C1-C4 not in the verdict (never vacuous)"""
        self.s.cycle('b4_rtab', 'rtabmap_skipped', 'rtabmap')
        self.s.cycle('b4_ns', 'normal_stop')
        rc, d = self.readout('--cycles', '0')
        self.assertEqual(rc, 0, json.dumps(d['verdict']))
        self.assertEqual(sorted(d['verdict']), ['c5_find', 'c6_normal_stop', 'complete'])
        self.assertTrue(d['find_only'])
        self.assertIn('rtabmap_skipped', d['reported']['b4_rtab']['kind'])
        rc, d = self.readout('--cycles', '0', expect=False)
        self.assertEqual((rc, d['verdict']['c5_find']), (3, 'not_judged'))
        self.s2 = Session(os.path.join(self.tmp, 'x'), self.rc_plain, self.rc_killed)
        self.s2.cycle('b4_ns', 'normal_stop', runner_xyz=(2.0, 0.1, 0.0))
        rc, d = RO.main([self.s2.root, '--cycles', '0', '--expect', 'bottle 1.0 0.1']), None
        self.assertEqual(rc, 1)

    def test_bad_input(self):
        self.assertEqual(RO.main([self.tmp]), 2)                                   # no cycles.txt
        self.s.cycle('b4_c1')
        self.assertEqual(RO.main([self.tmp, '--expect', 'cup 1 0']), 2)            # label != 瓶子
        self.assertEqual(RO.main([self.tmp, '--find-text', '你好']), 2)            # names no COCO class


# ------------------------------------------------------------------------------------------------ bench_line4.sh dry run
FAKE = textwrap.dedent('''\
    #!/bin/sh
    echo "$0 $*" >> "$FAKE_CALLS"
    exit 0
    ''')


class TestBenchDryRun(unittest.TestCase):
    """bench_line4.sh run --dry-run prints every step and runs nothing remote: stand-in ssh / scp on PATH record any
    call that escaped the X wrapper (positive control: calling the stand-in directly is recorded)."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='b4dry_')
        self.bin = os.path.join(self.tmp, 'bin')
        os.mkdir(self.bin)
        self.calls = os.path.join(self.tmp, 'calls.log')
        for n in ('ssh', 'scp'):
            p = os.path.join(self.bin, n)
            with open(p, 'w') as f:
                f.write(FAKE)
            os.chmod(p, os.stat(p).st_mode | stat.S_IXUSR)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def run_dry(self, **env):
        r_dir = os.path.join(self.tmp, 'R%d' % len(os.listdir(self.tmp)))
        e = dict(os.environ, PATH=self.bin + os.pathsep + os.environ['PATH'], FAKE_CALLS=self.calls, R=r_dir, **env)
        for k in ('BENCH_OBJ', 'RTABMAP_DB', 'FIND_ONLY', 'NORMAL_STOP', 'CYCLES', 'SITE_NOTE', 'CAR_HEIGHT_CELL'):
            if k not in env:
                e.pop(k, None)
        r = subprocess.run(['bash', BENCH, 'run', '--dry-run'], env=e, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=120)
        self.assertEqual(r.returncode, 0, r.stderr)
        (L,) = [os.path.join(r_dir, d) for d in os.listdir(r_dir) if d.startswith('dryrun_')]
        with open(os.path.join(L, 'session.log'), encoding='utf-8') as f:
            log = f.read()
        # stdout: log lines + unredirected steps + the readout line; session.log: every [dry] step incl. redirected ones
        return r.stdout + log, L

    def test_positive_control(self):
        subprocess.run([os.path.join(self.bin, 'ssh'), 'x'], env=dict(os.environ, FAKE_CALLS=self.calls), check=True)
        self.assertTrue(os.path.exists(self.calls))

    def test_nano_base_on_data_and_socket_fits(self):
        """10-15: the Nano base moved off the root partition (6.7 G free) to /data; the longest Unix socket car_up makes
        there (CAR_STATE_DIR/<tag>/inf.sock, CAR_STATE_DIR = <base>/st) must fit sun_path (107 bytes + NUL)."""
        out, L = self.run_dry(SITE_NOTE='x', RTABMAP_DB='/x.db')
        self.assertIn('mkdir -p /data/fpga/b4/runs /data/fpga/b4/st', out)
        self.assertNotIn('/home/robot/fpga/bench4', out)
        tags = set(re.findall(r'\bb4_\d{4}_\d{4}_[a-z0-9]+\b', out))
        self.assertTrue({t.rsplit('_', 1)[1] for t in tags} >= {'c1', 'ns', 'rtab'}, tags)
        longest = max(len(('/data/fpga/b4/st/%s/inf.sock' % t).encode()) for t in tags | {'b4_1231_2359_c99'})
        self.assertLessEqual(longest, 107)

    def test_default_plan(self):
        out, L = self.run_dry(SITE_NOTE='现场无光照')
        self.assertFalse(os.path.exists(self.calls), open(self.calls).read() if os.path.exists(self.calls) else '')
        for r in ('kv260', 'orin-nano', 'camera'):
            self.assertIn('lock.sh acquire %s ' % r, out)
        self.assertIn('kv restore', out)                                # m7_pl restored at exit
        self.assertIn('rtabmap attempt skipped: no RTABMAP_DB', out)
        self.assertFalse([l for l in out.splitlines() if ' nano up ' in l and l.rstrip().endswith('rtabmap')])
        with open(os.path.join(L, 'cycles.txt')) as f:
            kinds = [l.split()[1] for l in f.read().splitlines()]
        self.assertEqual(kinds, ['rtabmap_skipped', 'formal', 'formal', 'formal', 'normal_stop'])
        fs = [l for l in out.splitlines() if l.startswith('[dry] ssh nano') and ' nano failstop ' in l]
        self.assertEqual(len(set(fs)), 3)                                 # no injection in the normal-stop cycle
        self.assertFalse(any('b4_' in l and '_ns ' in l for l in fs))
        self.assertIn('normal stop (no injection): car_down', out)
        self.assertIn("criterion 5 will be 'not judged'", out)
        ro = [l for l in out.splitlines() if 'bench_line4_readout.py' in l][0]
        self.assertIn('--cycles 3 --normal-stops 1 --site-note 现场无光照', ro)
        self.assertNotIn('--expect', ro)
        with open(os.path.join(L, 'up_env.sh')) as f:
            self.assertIn('export CAR_HEIGHT_CELL=8', f.read())
        self.assertIn('CLEANUP_DONE', out)

    def test_with_db_and_object(self):
        out, L = self.run_dry(RTABMAP_DB='/home/robot/fpga/maps/x.db', BENCH_OBJ='bottle 1.2 -0.1',
                              LIVE_LAUNCH_ARGS='color_fps:=15')
        self.assertFalse(os.path.exists(self.calls))
        self.assertTrue([l for l in out.splitlines() if ' nano up ' in l and l.rstrip().endswith('_rtab rtabmap')])
        ro = [l for l in out.splitlines() if 'bench_line4_readout.py' in l][0]
        self.assertIn('--expect bottle 1.2 -0.1', ro)
        with open(os.path.join(L, 'up_env.sh')) as f:
            env = f.read()
        self.assertIn("export LIVE_LAUNCH_ARGS=color_fps:=15", env)
        self.assertIn('export RTABMAP_DB=/home/robot/fpga/maps/x.db', env)
        with open(os.path.join(L, 'cycles.txt')) as f:
            self.assertEqual(f.read().splitlines()[0].split()[1], 'rtabmap_try')

    def test_find_only(self):
        out, L = self.run_dry(FIND_ONLY='1', BENCH_OBJ='bottle 1.0 0.0')
        self.assertFalse(os.path.exists(self.calls))
        with open(os.path.join(L, 'cycles.txt')) as f:
            self.assertEqual([l.split()[1] for l in f.read().splitlines()], ['normal_stop'])
        self.assertNotIn('nano failstop', out)
        self.assertNotIn('rtabmap attempt', out)
        self.assertFalse([l for l in out.splitlines() if ' nano up ' in l and l.rstrip().endswith('rtabmap')])
        ro = [l for l in out.splitlines() if 'bench_line4_readout.py' in l][0]
        self.assertIn('--cycles 0 --normal-stops 1', ro)
        self.assertIn('只补跑寻物', out)



# 10-15: bench_line4.sh took its lock token with grep -o '[0-9a-f]{32}' | tail -1 on lock.sh's merged output, no rc
# check. lock.sh (tools/admin/workspace.py) answers an acquire with one JSON line {"who", "token", "since", "why"}, rc 0,
# and a refusal with '错误: 资源已被占用: {..."token": "<holder's>"...}' on stderr, rc 1 -- the holder's token was read
# The wait loop is cut out of bench_line4.sh and EXECUTED with a stand-in lock.sh in the workspace.py answer format:
# camera refused once (held by another session, token H), granted the next round. Known answer: after the loop the
# token file holds our three tokens, never H; the refused round released our kv260/orin-nano and never used H.
# Control: the same run with the old one-liner put back reads H as the camera token and stops on the first round.
HOLDER = 'f' * 32
FAKE_LOCK = r"""#!/bin/bash
# stand-in for lock.sh, workspace.py's answer format; call log in $LOCK_DIR/calls
echo "$*" >> "$LOCK_DIR/calls"
case "$1" in
status) echo '[]';;
acquire)
  c="$LOCK_DIR/n_$2"; n=$(( $(cat "$c" 2>/dev/null || echo 0) + 1 )); echo $n > "$c"
  if [ "$2" = camera ] && [ $n = 1 ]; then
    echo '错误: 资源已被占用: {"who": "dev1", "token": "HOLDER", "since": "2026-10-15T10:00:00", "why": "x"}' >&2; exit 1
  fi
  printf '{"who": "dev1", "token": "%s", "since": "2026-10-15T10:00:01", "why": "%s"}\n' \
    "$(printf '%s%s' "$2" "$n" | md5sum | cut -c1-32)" "$3";;
release) echo '{"released": "'"$2"'"}';;
esac
""".replace('HOLDER', HOLDER)


class TestBenchLockToken(unittest.TestCase):
    @staticmethod
    def cut(text):
        lines = text.splitlines()
        f0 = next(i for i, x in enumerate(lines) if x.startswith('lock_token() {'))
        f1 = next(i for i in range(f0, len(lines)) if lines[i].rstrip().endswith('head -1; }'))
        a = next(i for i, x in enumerate(lines) if x == '    while true; do' and '$LOCK status' in lines[i + 2])
        b = next(i for i in range(a, len(lines)) if lines[i] == '    done')
        return '\n'.join(lines[f0:f1 + 1]), '\n'.join(lines[a:b + 1])

    def run_loop(self, text):
        d = tempfile.mkdtemp(prefix='b4lock_')
        lock = os.path.join(d, 'lock.sh')
        with open(lock, 'w') as f:
            f.write(FAKE_LOCK)
        func, loop = self.cut(text)
        script = '\n'.join(['set -u', 'L=%s TOK=%s/tokens LOCK="bash %s" LOCK_POLL_S=0 WHY=w' % (d, d, lock),
                            ': > "$TOK"; : > "$L/session.log"', 'log() { echo "LOG $*"; }', func, 'run() {', loop, '}', 'run',
                            'echo LOOP_DONE'])
        r = subprocess.run(['bash', '-c', script], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           env=dict(os.environ, LOCK_DIR=d), universal_newlines=True, timeout=60)
        self.assertIn('LOOP_DONE', r.stdout, r.stdout)

        def read(n):
            with open(os.path.join(d, n), encoding='utf-8') as f:
                return f.read()
        return dict(l.split() for l in read('tokens').splitlines()), read('calls').splitlines(), read('session.log')

    def test_refusal_is_not_our_token(self):
        with open(BENCH, encoding='utf-8') as f:
            text = f.read()
        tok, calls, slog = self.run_loop(text)
        self.assertEqual(sorted(tok), ['camera', 'kv260', 'orin-nano'])
        self.assertNotIn(HOLDER, tok.values())
        self.assertTrue(all(re.match(r'^[0-9a-f]{32}$', t) for t in tok.values()), tok)
        self.assertEqual([c.split()[1] for c in calls if c.startswith('acquire')],
                         ['kv260', 'orin-nano', 'camera', 'kv260', 'orin-nano', 'camera'])
        rel = [c for c in calls if c.startswith('release')]
        self.assertEqual([c.split()[1] for c in rel], ['kv260', 'orin-nano'])      # ours, after the refused round
        self.assertFalse([c for c in calls if HOLDER in c], calls)
        self.assertIn('lock camera not acquired', slog)
        self.assertNotIn("grep -o -E '[0-9a-f]{32}' | tail -1", text)
        # control: the old one-liner on the same answers takes the holder's token and stops at once
        old = text.replace('t=$(lock_token "$r" "$WHY" 2>> "$L/session.log")',
                           """o=$($LOCK acquire "$r" "$WHY" -u dev1 2>&1); t=$(echo "$o" | grep -o -E '[0-9a-f]{32}' | tail -1)""")
        self.assertNotEqual(old, text)
        tok, calls, _ = self.run_loop(old.replace('local st got=1 r t', 'local st got=1 r o t'))
        self.assertEqual(tok['camera'], HOLDER)
        self.assertEqual(len([c for c in calls if c.startswith('acquire')]), 3)


if __name__ == '__main__':
    unittest.main()
