"""10-17 (docs/design/CAM_USB_REENUM_20261017.md): the Gemini 2 re-enumerates (USB disconnect, next device number) in
the first second of a driver start (board 14:36 002/003->004, 16:26 002/004->005). car_up.sh passed the node of
before with docker --device, so every start retry inside the stack container opened a dead node (10 of 10 with 0
depth frames, car_up 7). Fix: the whole USB root bind-mounted + device cgroup rule 'c 189:* rw'.

Here, with stand-ins (no docker, no camera): the board start's stack-container argv (dry run and real start), and the
start retry (the real car_start_retry) against a model of what a container sees through those switches -- a --device
node is a copy fixed at docker run (one major:minor), a bind-mounted directory is live and a cgroup rule gates the
majors:minors that may be opened (both checked against the host's docker in
docs/design/cam_usb_reenum_20261017/host_docker_check.sh). The stand-in driver re-enumerates the camera on its first
start and sends depth only when the camera's current node can be opened in the container.

CAR_REENUM_TREE=<tree> runs the same tests against another tree's tools/car_run (the negative control: origin/main
before the fix must fail). Python 3.8 compatible."""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import textwrap
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_car_run as tcr  # noqa: E402

TREE = os.path.abspath(os.environ.get('CAR_REENUM_TREE') or os.path.dirname(HERE))
CAR = os.path.join(TREE, 'tools', 'car_run')
UP = os.path.join(CAR, 'car_up.sh')
DOWN = os.path.join(CAR, 'car_down.sh')
COMMON = os.path.join(CAR, 'car_common.sh')
INNER = os.path.join(CAR, 'car_inner.sh')
USB_MAJOR = 189


# ---- the container model ------------------------------------------------------------------------------------------
def node_id(path):
    """the stand-in USB root names nodes BBB/DDD like /dev/bus/usb: usbfs major 189, minor (bus-1)*128 + dev-1"""
    bus, dev = int(os.path.basename(os.path.dirname(path))), int(os.path.basename(path))
    return (USB_MAJOR, (bus - 1) * 128 + dev - 1)


def container_view(argv):
    """what docker run ARGV gives the container, taken at docker run: --device copies (container path -> the host
    node's id at that moment; a missing source fails docker run), -v binds (host dir, container dir), cgroup rules"""
    view = {'copies': {}, 'binds': [], 'rules': []}
    for i, a in enumerate(argv[:-1]):
        v = argv[i + 1]
        if a == '--device':
            host = v.split(':')[0]
            if not re.search(r'/[0-9]{3}/[0-9]{3}$', host):   # not a USB node (the chassis tty): not modelled
                continue
            if not os.path.exists(host):
                raise RuntimeError('docker run: error gathering device information: %s: no such file' % host)
            view['copies'][v.split(':')[1] if ':' in v else host] = list(node_id(host))
        elif a == '-v':
            parts = v.split(':')
            view['binds'].append([os.path.abspath(parts[0]), parts[1]])
        elif a == '--device-cgroup-rule':
            view['rules'].append(v)
    return view


def rule_allows(rules, ident):
    for r in rules:
        kind, mm, perms = r.split()
        maj, mnr = mm.split(':')
        if kind == 'c' and maj in ('*', str(ident[0])) and mnr in ('*', str(ident[1])) and 'r' in perms and 'w' in perms:
            return True
    return False


def can_open(view, host_node):
    """can the container open HOST_NODE (exists on the host now) read-write: through a --device copy of the same
    device, or through a bind of a directory holding it with a cgroup rule allowing its major:minor (docker's default
    allow list has no 189)"""
    ident = list(node_id(host_node))
    if ident in view['copies'].values():
        return True
    for src, _dst in view['binds']:
        if host_node.startswith(src.rstrip('/') + '/') and rule_allows(view['rules'], tuple(ident)):
            return True
    return False


# ---- stand-ins ----------------------------------------------------------------------------------------------------
# lsusb: the camera's node from the stand-in USB root (whatever device number it has now), nothing without one
LSUSB = textwrap.dedent('''\
    #!/usr/bin/env python3
    import os, sys
    root = os.environ['CAR_USB_ROOT']
    if sys.argv[1:3] == ['-d', '2bc5:']:
        for bus in sorted(os.listdir(root)):
            for dev in sorted(os.listdir(os.path.join(root, bus))):
                print('Bus %s Device %s: ID 2bc5:0670 Orbbec 3D Technology International, Inc Gemini 2' % (bus, dev))
    ''')
# driver: like ros2 launch, a parent with one child. A start "sends depth" (data_<n>) only when the camera's node of
# NOW opens in the container (the model, on the view saved from the stack container's argv). REENUM=1: start 1 opens
# the node and then the camera re-enumerates under it (BBB/DDD -> BBB/DDD+1, as the board's 2-1.1 did 1 s into
# try 1): no depth from that start whatever the switches
DRIVER = textwrap.dedent('''\
    import json, os, subprocess, sys, time
    sys.path.insert(0, %(tests)r)
    from test_cam_usb_reenum import can_open
    st, root = sys.argv[1], sys.argv[2]
    view = json.load(open(os.path.join(st, 'view.json')))
    n = len([f for f in os.listdir(st) if f.startswith('start_')]) + 1
    child = subprocess.Popen(['sleep', '300'])
    open(os.path.join(st, 'start_%%d' %% n), 'w').write('%%d %%d' %% (os.getpid(), child.pid))
    def node():
        return [os.path.join(root, b, d) for b in sorted(os.listdir(root)) for d in os.listdir(os.path.join(root, b))][0]
    cur = node()
    ok = can_open(view, cur)
    if n == 1 and os.environ.get('REENUM', '1') == '1':   # opened (or not), then the camera dropped off under it
        new = os.path.join(os.path.dirname(cur), '%%03d' %% (int(os.path.basename(cur)) + 1))
        os.rename(cur, new)
        print('USB disconnect %%s, new device %%s' %% (cur, new), flush=True)
        ok = False
    print('try %%d node %%s %%s' %% (n, cur, 'opened' if ok else 'cannot open'), flush=True)
    if ok:
        open(os.path.join(st, 'data_%%d' %% n), 'w').write('depth')
    while True:
        time.sleep(1)
    ''')


class StackArgv(tcr.Base):
    """the board start of TREE with tcr's stand-ins (docker, lock, KV check, inference) and a stand-in USB root
    holding the camera at 002/005"""

    def setUp(self):
        tcr.Base.setUp(self)
        self.usb = os.path.join(self.tmp, 'usb')
        self.node = os.path.join(self.usb, '002', '005')
        os.makedirs(os.path.dirname(self.node))
        os.symlink('/dev/null', self.node)   # a character device at the resolved path
        tcr.write_exec(os.path.join(self.tmp, 'lsusb'), LSUSB)
        tcr.write_exec(os.path.join(self.tmp, 'kvcheck'), tcr.FAKE_KV_CHECK)
        db = os.path.join(self.tmp, 'pinned.db')
        open(db, 'w').write('db')
        map_yaml = os.path.join(self.tmp, 'run_pinned', 'export', 'pinned.yaml')
        os.makedirs(os.path.dirname(map_yaml))
        open(map_yaml, 'w').write('image: pinned.pgm\n')
        self.env.update(CAR_ARCH='aarch64', CAR_BOARD_DRAFT='1', CAR_HUB_BIND='198.51.100.1',
                        CAR_LSUSB=os.path.join(self.tmp, 'lsusb'), CAR_USB_ROOT=self.usb,
                        CAR_KV_CHECK=os.path.join(self.tmp, 'kvcheck'), CAR_PY_BOARD=os.path.join(self.tmp, 'py'),
                        RTABMAP_DB=db, RTABMAP_DB_DIR=os.path.join(self.tmp, 'rtabmap_dbdir'))

    def up(self, *args, **env):
        return subprocess.run(['bash', UP] + list(args), env=dict(self.env, **env), stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, universal_newlines=True, timeout=180)

    def down(self, *args):
        return subprocess.run(['bash', DOWN] + list(args), env=self.env, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, universal_newlines=True, timeout=180)

    def real_start_argv(self, name='board1'):
        run = os.path.join(self.tmp, 'runs', name)
        r = self.up('live', '--run-dir', run)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        argv = json.load(open(os.path.join(self.fd, 'wde-car-stack-%s.json' % name)))['argv']
        d = self.down()
        self.assertEqual(d.returncode, 0, d.stdout + d.stderr)
        return argv


class TestStackSwitches(StackArgv):
    def check(self, argv):
        rules = [argv[i + 1] for i, a in enumerate(argv[:-1]) if a == '--device-cgroup-rule']
        self.assertEqual(rules, ['c 189:* rw'])                                   # exactly one, usbfs only
        binds = [argv[i + 1] for i, a in enumerate(argv[:-1]) if a == '-v']
        self.assertIn('%s:/dev/bus/usb' % self.usb, binds)
        self.assertFalse([b for b in binds if b.split(':')[1] == '/dev'], binds)   # not the whole /dev
        devices = [argv[i + 1] for i, a in enumerate(argv[:-1]) if a == '--device']
        self.assertEqual(devices, ['/dev/fpga_chassis'])                           # live preset = uart; no camera node
        self.assertNotIn('--privileged', argv)

    def test_dry_run(self):
        r = self.up('live', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        stack = next(l for l in r.stdout.splitlines() if l.startswith('[container stack]'))
        self.assertIn(' -v %s:/dev/bus/usb --device-cgroup-rule c 189:* rw --device /dev/fpga_chassis ' % self.usb, stack)
        self.assertNotIn('--device %s' % self.node, stack)
        self.assertNotIn('--privileged', stack)
        self.assertIn('# camera device: %s (from ' % self.node, r.stdout)        # still resolved: presence + record
        self.assertIn("# camera usb: the whole %s -> /dev/bus/usb + device cgroup rule 'c 189:* rw'" % self.usb, r.stdout)

    def test_real_start(self):
        self.check(self.real_start_argv())

    def test_chassis_none_no_device_at_all(self):
        run = os.path.join(self.tmp, 'runs', 'b2')
        r = self.up('live', '--run-dir', run, '--pose', 'static', '--chassis', 'none', '--nav', 'costmap')
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        argv = json.load(open(os.path.join(self.fd, 'wde-car-stack-b2.json')))['argv']
        self.assertNotIn('--device', argv)
        self.assertIn('%s:/dev/bus/usb' % self.usb, argv)
        self.assertEqual(self.down().returncode, 0)

    def test_no_camera_still_refused_3(self):
        os.remove(self.node)
        r = self.up('live', '--run-dir', os.path.join(self.tmp, 'runs', 'b3'))
        self.assertEqual(r.returncode, 3, r.stdout + r.stderr)
        self.assertIn('no Orbbec camera on USB', r.stderr)


class TestModel(unittest.TestCase):
    """the model can say no: a --device copy of the node of before does not open the re-enumerated camera"""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='reenum_model_')
        self.usb = os.path.join(self.tmp, 'usb')
        os.makedirs(os.path.join(self.usb, '002'))
        self.n5, self.n6 = os.path.join(self.usb, '002', '005'), os.path.join(self.usb, '002', '006')
        os.symlink('/dev/null', self.n5)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_device_copy_is_fixed_at_run(self):
        view = container_view(['run', '--device', self.n5, 'img'])
        self.assertTrue(can_open(view, self.n5))
        os.rename(self.n5, self.n6)
        self.assertFalse(can_open(view, self.n6))
        with self.assertRaises(RuntimeError):            # the node gone before docker run: docker run itself fails
            container_view(['run', '--device', self.n5, 'img'])

    def test_bind_needs_the_rule_and_only_189(self):
        bind = ['-v', '%s:/dev/bus/usb' % self.usb]
        os.rename(self.n5, self.n6)
        self.assertTrue(can_open(container_view(['run'] + bind + ['--device-cgroup-rule', 'c 189:* rw', 'img']), self.n6))
        self.assertFalse(can_open(container_view(['run'] + bind + ['img']), self.n6))
        self.assertFalse(can_open(container_view(['run'] + bind + ['--device-cgroup-rule', 'c 10:* rw', 'img']), self.n6))
        self.assertFalse(can_open(container_view(['run'] + bind + ['--device-cgroup-rule', 'c 189:* r', 'img']), self.n6))
        self.assertFalse(can_open(container_view(['run', '-v', '%s/001:/dev/bus/usb/001' % self.usb,
                                                  '--device-cgroup-rule', 'c 189:* rw', 'img']), self.n6))


class TestRetryAcrossReenum(StackArgv):
    """the real car_start_retry with the stand-in driver on the view of the stack container car_up made: the camera
    re-enumerates in try 1; a later try must open the new node"""

    def retry(self, argv, tries=4, reenum='1'):
        st = os.path.join(self.tmp, 'drv')
        os.mkdir(st)
        json.dump(container_view(argv), open(os.path.join(st, 'view.json'), 'w'))
        tcr.write_exec(os.path.join(self.tmp, 'drv.py'), DRIVER % {'tests': HERE})
        tcr.write_exec(os.path.join(self.tmp, 'probe.py'), tcr.TestStartRetry.PROBE)
        script = ('source %s\n' % COMMON
                  + 'CAR_RETRY_PROBE=(python3 %s/probe.py %s 1.5)\n' % (self.tmp, st)
                  + 'CAR_MARKS_FILE=%s/marks.txt car_start_retry %d 1 0 %s/driver.log python3 %s/drv.py %s %s\n'
                  % (st, tries, st, self.tmp, st, self.usb)
                  + 'rc=$?; echo "rc=$rc try=$CAR_RETRY_TRY"\n'
                  '[ -n "$CAR_RETRY_PID" ] && kill -KILL $(pgrep -P $CAR_RETRY_PID) $CAR_RETRY_PID\nexit 0\n')
        try:
            r = subprocess.run(['bash', '-c', script], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               universal_newlines=True, timeout=120, env=dict(self.env, REENUM=reenum))
        finally:
            for f in os.listdir(st):
                if f.startswith('start_'):
                    for pid in map(int, open(os.path.join(st, f)).read().split()):
                        tcr.kill_if_alive(pid)
        self.assertEqual(r.returncode, 0, r.stderr)
        marks = [l.split(' ', 1)[1] for l in open(os.path.join(st, 'marks.txt')).read().splitlines()]
        return r.stdout.strip(), [m for m in marks if m.startswith('camera driver try=')], st

    def test_reenum_in_try_1_recovered_in_try_2(self):
        argv = self.real_start_argv()
        out, tries, st = self.retry(argv)
        self.assertEqual(out, 'rc=0 try=2', (tries, open(os.path.join(st, 'driver_failed_start_1.log')).read()))
        self.assertEqual(tries, ['camera driver try=1 depth_stream=0 probe_rc=1',
                                 'camera driver try=2 depth_stream=1 probe_rc=0'])
        self.assertIn('USB disconnect', open(os.path.join(st, 'driver_failed_start_1.log')).read())
        self.assertIn('002/006 opened', open(os.path.join(st, 'driver.log')).read())

    def test_without_reenum_try_1_good(self):   # control: the stand-in starts fine when nothing re-enumerates
        out, tries, st = self.retry(self.real_start_argv(), reenum='0')
        self.assertEqual(out, 'rc=0 try=1')

    def test_device_of_before_never_recovers(self):
        """positive control of the simulation: the switches of before (--device of the resolved node) fail every try"""
        argv = self.real_start_argv()
        old = list(argv)
        for pair in (['-v', '%s:/dev/bus/usb' % self.usb], ['--device-cgroup-rule', 'c 189:* rw']):
            for i in [i for i in range(len(old) - 1) if old[i:i + 2] == pair][:1]:
                del old[i:i + 2]
        if self.node not in old:                           # car_up.sh of before: lsusb's node with --device
            old[1:1] = ['--device', self.node]
        out, tries, st = self.retry(old, tries=3)
        self.assertEqual(out, 'rc=1 try=')
        self.assertEqual(tries, ['camera driver try=%d depth_stream=0 probe_rc=1' % n for n in (1, 2, 3)])


class TestUsbNoteMarks(unittest.TestCase):
    """car_start_retry CAR_RETRY_NOTE: a line before EVERY try (the 1st too) -> 'camera driver usb try=N ...'"""

    def test_note_before_every_try(self):
        tmp = tempfile.mkdtemp(prefix='reenum_note_')
        try:
            tcr.write_exec(os.path.join(tmp, 'drv.py'), tcr.TestStartRetry.DRIVER)
            tcr.write_exec(os.path.join(tmp, 'probe.py'), tcr.TestStartRetry.PROBE)
            script = ('source %s\nCAR_RETRY_PROBE=(python3 %s/probe.py %s 1.5)\n' % (COMMON, tmp, tmp)
                      + 'note() { echo "node $(ls %s | grep -c ^start_)"; echo second line; }\nCAR_RETRY_NOTE=(note)\n' % tmp
                      + 'CAR_MARKS_FILE=%s/marks.txt car_start_retry 3 1 0 %s/driver.log python3 %s/drv.py %s 2\n'
                      % (tmp, tmp, tmp, tmp)
                      + '[ -n "$CAR_RETRY_PID" ] && kill -KILL $(pgrep -P $CAR_RETRY_PID) $CAR_RETRY_PID\nexit 0\n')
            r = subprocess.run(['bash', '-c', script], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               universal_newlines=True, timeout=120)
            self.assertEqual(r.returncode, 0, r.stderr)
            marks = [l.split(' ', 1)[1] for l in open(os.path.join(tmp, 'marks.txt')).read().splitlines()]
            self.assertEqual(marks, ['camera driver usb try=1 node 0', 'camera driver try=1 depth_stream=0 probe_rc=1',
                                     'camera driver usb try=2 node 1', 'camera driver try=2 depth_stream=1 probe_rc=0'])
        finally:
            for f in os.listdir(tmp):
                if f.startswith('start_'):
                    for pid in map(int, open(os.path.join(tmp, f)).read().split()):
                        tcr.kill_if_alive(pid)
            shutil.rmtree(tmp, ignore_errors=True)

    def test_inner_marks_the_node_each_try(self):
        """car_inner.sh live: the 2bc5 device from a stand-in /sys, visible|MISSING in a stand-in /dev/bus/usb"""
        import test_camera_halfrate as thr
        from pathlib import Path
        t = thr.TestInnerExecuted('test_depth_failures_still_reported')
        t.setUp()
        inner_before, thr.INNER = thr.INNER, Path(INNER)   # TREE's car_inner.sh (CAR_REENUM_TREE)
        try:
            sysd = os.path.join(t.tmp, 'sys', '2-1.1')
            os.makedirs(sysd)
            for k, v in (('idVendor', '2bc5'), ('busnum', '2'), ('devnum', '5')):
                open(os.path.join(sysd, k), 'w').write(v + '\n')
            os.makedirs(os.path.join(t.tmp, 'sys', 'usb2'))
            open(os.path.join(t.tmp, 'sys', 'usb2', 'idVendor'), 'w').write('1d6b\n')
            devu = os.path.join(t.tmp, 'devusb')
            os.makedirs(os.path.join(devu, '002'))
            os.environ['CAR_SYS_USB'], os.environ['CAR_DEV_USB'] = os.path.join(t.tmp, 'sys'), devu
            try:
                r, marks, argv, fail = t.run_inner('nodepth nodepth', tries='2')
                usb = [m for m in marks if m.startswith('camera driver usb try=')]
                self.assertEqual(usb, ['camera driver usb try=1 002/005 MISSING (2-1.1)',
                                       'camera driver usb try=2 002/005 MISSING (2-1.1)'], r.stdout)
                shutil.rmtree(t.o); os.makedirs(t.o)
                os.symlink('/dev/null', os.path.join(devu, '002', '005'))   # the node there: visible
                r, marks, argv, fail = t.run_inner('nodepth', tries='1')
                usb = [m for m in marks if m.startswith('camera driver usb try=')]
                self.assertEqual(usb, ['camera driver usb try=1 002/005 visible (2-1.1)'], r.stdout)
            finally:
                del os.environ['CAR_SYS_USB'], os.environ['CAR_DEV_USB']
        finally:
            thr.INNER = inner_before
            t.tearDown()


if __name__ == '__main__':
    unittest.main()
