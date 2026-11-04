"""Bounded runtime metadata waits without Docker, ROS, hardware or real sleeps."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('competition_runtime_info', ROOT / 'ui/geomem/tools/competition/wait_runtime_info.py')
runtime = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runtime)
SCOPE = '0123456789abcdef'


class RuntimeInfoWaitTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.run = Path(self.tmp.name) / '20261021-live'
        self.run.mkdir()
        self.info = self.run / 'runtime_info.json'
        self.now = 0.0
        self.on_sleep = lambda: None

        def sleep(seconds):
            self.now += seconds
            self.on_sleep()

        self.clock = patch.object(runtime.time, 'monotonic', side_effect=lambda: self.now).start()
        self.sleep = patch.object(runtime.time, 'sleep', side_effect=sleep).start()
        self.docker = patch.object(runtime.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, 'true\n')).start()
        self.addCleanup(patch.stopall)

    def write_valid(self):
        self.info.write_text(json.dumps({'runtime_boot_id': SCOPE}), encoding='utf-8')

    def wait(self):
        return runtime.wait_runtime_info(self.run, self.run, timeout_s=1.0)

    def test_delayed_metadata_is_waited_for_without_restarting_the_stack(self):
        self.on_sleep = self.write_valid
        self.assertEqual(self.wait(), SCOPE)
        self.assertEqual(self.now, 0.25)
        self.assertEqual(self.docker.call_count, 2)
        for call in self.docker.call_args_list:
            self.assertEqual(call.args[0], ['docker', 'inspect', '--format', '{{.State.Running}}',
                                           'wde-car-stack-' + self.run.name])

    def test_partial_json_can_finish_before_the_deadline(self):
        self.info.write_text('{"runtime_boot_id":', encoding='utf-8')
        self.on_sleep = self.write_valid
        self.assertEqual(self.wait(), SCOPE)
        self.assertEqual(self.now, 0.25)

    def test_bad_json_times_out_without_altering_evidence(self):
        self.info.write_text('{"runtime_boot_id":', encoding='utf-8')
        before = self.info.read_bytes()
        with self.assertRaisesRegex(RuntimeError, 'Timed out.*not complete/readable'):
            self.wait()
        self.assertEqual(self.now, 1.0)
        self.assertEqual(self.info.read_bytes(), before)

    def test_invalid_runtime_boot_ids_are_not_used_as_scope(self):
        for scope in (None, '', 'rb1', '0123456789ABCDEF', 123, {'id': SCOPE}):
            with self.subTest(scope=scope):
                self.now = 0.0
                self.info.write_text(json.dumps({'runtime_boot_id': scope}), encoding='utf-8')
                with self.assertRaisesRegex(RuntimeError, 'Timed out.*runtime_boot_id'):
                    self.wait()

    def test_missing_metadata_times_out_with_retry_guidance_and_nonzero_exit(self):
        output = io.StringIO()
        with contextlib.redirect_stderr(output):
            code = runtime.main(['--current', str(self.run), '--run-dir', str(self.run), '--timeout-seconds', '1'])
        self.assertEqual(code, 1)
        self.assertEqual(self.now, 1.0)
        self.assertIn('retry attaching', output.getvalue())
        self.assertFalse(self.info.exists())

    def test_changed_session_during_wait_is_refused_before_another_docker_check(self):
        pointer = self.run.parent / 'car.current'
        original_resolve = Path.resolve

        def resolve(path, *args, **kwargs):
            if path == pointer:
                return self.run if self.now == 0 else self.run.parent / 'another-run'
            return original_resolve(path, *args, **kwargs)

        with patch.object(Path, 'resolve', resolve):
            with self.assertRaisesRegex(RuntimeError, 'car.current changed'):
                runtime.wait_runtime_info(pointer, self.run, timeout_s=1)
        self.assertEqual(self.docker.call_count, 1)
        self.assertEqual(self.now, 0.25)

    def test_session_switch_while_reading_valid_metadata_is_also_refused(self):
        self.write_valid()
        pointer = self.run.parent / 'car.current'
        original_resolve, original_read = Path.resolve, Path.read_text
        switched = [False]

        def resolve(path, *args, **kwargs):
            if path == pointer:
                return self.run.parent / 'another-run' if switched[0] else self.run
            return original_resolve(path, *args, **kwargs)

        def read(path, *args, **kwargs):
            data = original_read(path, *args, **kwargs)
            if path == self.info:
                switched[0] = True
            return data

        with patch.object(Path, 'resolve', resolve), patch.object(Path, 'read_text', read):
            with self.assertRaisesRegex(RuntimeError, 'car.current changed'):
                runtime.wait_runtime_info(pointer, self.run, timeout_s=1)

    def test_container_exit_during_wait_stops_immediately(self):
        self.docker.side_effect = [subprocess.CompletedProcess([], 0, 'true\n'),
                                   subprocess.CompletedProcess([], 0, 'false\n')]
        with self.assertRaisesRegex(RuntimeError, 'stack container stopped'):
            self.wait()
        self.assertEqual(self.now, 0.25)
        self.assertEqual(self.docker.call_count, 2)

    def test_unverifiable_container_is_not_assumed_running(self):
        self.docker.side_effect = subprocess.TimeoutExpired('docker', 0.1)
        with self.assertRaisesRegex(RuntimeError, 'Cannot verify the stack container'):
            self.wait()
        self.sleep.assert_not_called()


if __name__ == '__main__':
    unittest.main()
