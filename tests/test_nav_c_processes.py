import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('nav_c_processes', Path(__file__).resolve().parents[1] / 'tools/nav_c_processes.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class CProcessTests(unittest.TestCase):
    def test_wrappers_and_other_sessions_never_receive_signal(self):
        with tempfile.TemporaryDirectory() as directory:
            proc = Path(directory)
            worker = ['python3', '/fpga/robot_sim/tests/nav_point_to_point.py', '--c-worker']
            wrapper = ['bwrap', '--bind', '/project/ipc/current-dds', '/dev/shm'] + worker
            def put(pid, parent, args):
                path = proc / str(pid)
                path.mkdir(exist_ok=True)
                (path / 'cmdline').write_bytes(('\0'.join(args) + '\0').encode())
                (path / 'status').write_text('PPid:\t%d\n' % parent)
            put(10, 0, wrapper)
            put(11, 10, wrapper)
            put(12, 11, worker)
            put(20, 0, ['bwrap', '/project/ipc/current-other-dds'] + worker)
            put(21, 20, worker)
            calls = []
            result = m.signal_role('current', 'worker', 19, proc=proc, send=lambda *x: calls.append(x))
            self.assertEqual(result['pid'], 12)
            self.assertEqual(calls, [(12, 19)])
            put(13, 11, worker)
            with self.assertRaises(RuntimeError):
                m.signal_role('current', 'worker', 9, True, proc, lambda *x: calls.append(x))
            self.assertEqual(calls, [(12, 19)])
            with self.assertRaises(RuntimeError):
                m.signal_role('missing', 'worker', 19, proc=proc, send=lambda *x: calls.append(x))
            self.assertTrue(m.signal_role('missing', 'worker', 9, True, proc)['missing'])
            self.assertEqual(calls, [(12, 19)])

    def test_relay_and_smoother_require_real_executable(self):
        relay = ['python3', '-m', 'host.ros_tunnel', 'client']
        smoother = ['/opt/ros/humble/lib/nav2_velocity_smoother/velocity_smoother']
        for role, argv in [('relay', relay), ('smoother', smoother)]:
            self.assertTrue(m.matches_role(argv, role))
            self.assertFalse(m.matches_role(['bwrap'] + argv, role))


if __name__ == '__main__':
    unittest.main()
