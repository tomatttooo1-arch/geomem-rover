"""The container smoke helper's docker argv, read from its own --dry-run.

Docker is never invoked here: --dry-run prints the argv and exits, which is the only part
of the helper that can be judged without taking the machine. What is pinned is the shape
that makes the helper safe to run at any time — no network, nothing writable unless asked
for by name, a container name that says who owns it, and never another member's prefix.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

CHECKOUT = Path(__file__).resolve().parents[1]
SCRIPT = CHECKOUT / 'tools' / 'rtabmap_container_smoke.sh'
IMAGE = 'fpga-dev1-rtabmap:humble-0.23.7-20260910'


def dry_run(*extra):
    done = subprocess.run(['bash', str(SCRIPT), '--dry-run', *extra],
                          capture_output=True, text=True, cwd=str(CHECKOUT))
    return done


class DryRunArgvTests(unittest.TestCase):
    def argv(self, *extra):
        done = dry_run(*extra)
        self.assertEqual(done.returncode, 0, done.stderr)
        return done.stdout.rstrip('\n').split('\n')

    def mounts(self, argv):
        return [argv[index + 1] for index, item in enumerate(argv) if item == '--mount']

    def test_it_runs_the_frozen_image_with_no_network_and_a_bounded_container(self):
        argv = self.argv()
        self.assertEqual(argv[:3], ['docker', 'run', '--rm'])
        self.assertIn('--init', argv)
        self.assertIn(IMAGE, argv)
        for flag, value in (('--network', 'none'), ('--cpus', '2'), ('--memory', '4g'),
                            ('--env', 'PYTHONPATH=/checkout')):
            self.assertIn(flag, argv)
            self.assertEqual(argv[argv.index(flag) + 1], value, flag)

    def test_the_checkout_is_mounted_read_only_and_nothing_else_is_mounted(self):
        argv = self.argv()
        self.assertEqual(self.mounts(argv),
                         ['type=bind,src=%s,dst=/checkout,readonly' % CHECKOUT])

    def test_the_container_name_says_who_owns_it_and_never_another_members_prefix(self):
        argv = self.argv()
        name = argv[argv.index('--name') + 1]
        self.assertTrue(name.startswith('fpga-dev1-smoke-'), name)
        self.assertNotIn('fpga-dev2', '\n'.join(argv))
        self.assertNotIn('fpga-dev2', SCRIPT.read_text())
        # The pid tail is what keeps two concurrent runs from colliding on the name.
        self.assertRegex(name, r'^fpga-dev1-smoke-[a-z0-9-]+-\d+$')

    def test_the_default_command_answers_both_questions_the_host_cannot(self):
        tail = '\n'.join(self.argv()[self.argv().index(IMAGE):])
        self.assertIn('source /opt/ros/humble/setup.bash', tail)
        self.assertIn('import rclpy, tf2_ros, scipy', tail)
        for module in ('host/ros_localization_gate.py', 'host/ros_odom_base_adapter.py',
                       'tools/tf_topic_recorder.py'):
            self.assertIn('/checkout/' + module, tail)
        self.assertIn('py_compile', tail)

    def test_a_given_command_replaces_the_default_and_nothing_else(self):
        argv = self.argv('--', 'python3', '-c', 'print(1)')
        self.assertEqual(argv[-3:], ['python3', '-c', 'print(1)'])
        self.assertNotIn('py_compile', '\n'.join(argv))
        self.assertEqual(self.mounts(argv),
                         ['type=bind,src=%s,dst=/checkout,readonly' % CHECKOUT])

    def test_rw_adds_exactly_one_writable_mount_at_rosresults(self):
        with tempfile.TemporaryDirectory() as directory:
            argv = self.argv('--rw', directory)
            mounts = self.mounts(argv)
            self.assertEqual(len(mounts), 2)
            self.assertTrue(mounts[0].endswith(',readonly'), mounts[0])
            self.assertEqual(mounts[1],
                             'type=bind,src=%s,dst=/rosresults' % Path(directory).resolve())
            self.assertFalse(mounts[1].endswith(',readonly'))

    def test_a_writable_mount_is_never_silently_dropped_or_doubled(self):
        with tempfile.TemporaryDirectory() as directory:
            for extra in (('--rw',),
                          ('--rw', str(Path(directory) / 'absent')),
                          ('--rw', directory, '--rw', directory),
                          ('--unknown',)):
                done = dry_run(*extra)
                self.assertEqual(done.returncode, 2, extra)
                self.assertTrue(done.stderr.strip(), extra)

    def test_dry_run_prints_the_argv_and_never_reaches_docker(self):
        # The helper is safe to call from a test only because --dry-run exits before the
        # docker call; a run without it is the caller's own decision, never a test's.
        source = SCRIPT.read_text()
        self.assertIn('if [ "$dry_run" = true ]; then', source)
        self.assertIn("printf '%s\\n' \"${argv[@]}\"\n    exit 0", source)
        self.assertIn('set -euo pipefail', source)
        self.assertEqual(subprocess.run(['bash', '-n', str(SCRIPT)]).returncode, 0)


if __name__ == '__main__':
    unittest.main()
