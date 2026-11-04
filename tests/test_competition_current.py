"""Reuse checks with recorded run fixtures and mocked Docker/TCP; no hardware."""
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
SPEC = importlib.util.spec_from_file_location('competition_current', ROOT / 'ui/geomem/tools/competition/check_current.py')
current = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(current)


class CurrentSessionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.run = Path(self.tmp.name) / '20261019-live'
        self.run.mkdir()
        self.stack = 'wde-car-stack-' + self.run.name
        self.container_id = 'a' * 64
        (self.run / 'READY').write_text('ready')
        (self.run / 'token.txt').write_text('private-test-token')
        self.car = {'tag': self.run.name, 'containers': {'stack': self.stack},
                    'hub': {'bind': '127.0.0.1', 'port': 47801}}
        (self.run / 'car.json').write_text(json.dumps(self.car))
        (self.run / ('container-' + self.stack + '.id')).write_text(self.container_id + '\n')
        self.info = {'Id': self.container_id, 'Name': '/' + self.stack,
                     'State': {'Running': True, 'Paused': False, 'Restarting': False, 'Status': 'running'},
                     'HostConfig': {'NetworkMode': 'host'},
                     'Mounts': [{'Destination': '/o', 'Source': str(self.run)}]}

    def inspect_result(self):
        return subprocess.CompletedProcess([], 0, json.dumps(self.info))

    def snapshot(self):
        return {path.name: path.read_bytes() for path in self.run.iterdir()}

    def test_running_recorded_container_and_reachable_hub_can_be_reused(self):
        before = self.snapshot()
        with patch.object(current.subprocess, 'run', return_value=self.inspect_result()) as docker, \
                patch.object(current.socket, 'create_connection') as connect:
            result = current.check_current(self.run, 47801)
        self.assertEqual(result['container'], self.stack)
        docker.assert_called_once_with(['docker', 'inspect', '--format', '{{json .}}', self.stack], check=True,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=8)
        connect.assert_called_once_with(('127.0.0.1', 47801), timeout=2)
        self.assertNotIn('token', json.dumps(result))
        self.assertEqual(self.snapshot(), before)

    def test_old_ready_after_reboot_does_not_reuse_exited_container_or_remove_evidence(self):
        self.info['State'] = {'Running': False, 'Status': 'exited', 'ExitCode': 255}
        before = self.snapshot()
        with patch.object(current.subprocess, 'run', return_value=self.inspect_result()), \
                patch.object(current.socket, 'create_connection') as connect:
            with self.assertRaisesRegex(RuntimeError, 'not running.*exited'):
                current.check_current(self.run, 47801)
        connect.assert_not_called()
        self.assertEqual(self.snapshot(), before)

    def test_running_container_without_hub_is_refused_and_preserved(self):
        before = self.snapshot()
        with patch.object(current.subprocess, 'run', return_value=self.inspect_result()), \
                patch.object(current.socket, 'create_connection', side_effect=ConnectionRefusedError('refused')):
            with self.assertRaisesRegex(RuntimeError, 'hub is unreachable'):
                current.check_current(self.run, 47801)
        self.assertEqual(self.snapshot(), before)

    def test_replaced_container_and_wrong_run_mount_are_refused_before_connecting(self):
        for change, message in (({'Id': 'b' * 64}, 'container identity'),
                                ({'Mounts': [{'Destination': '/o', 'Source': str(self.run.parent)}]}, '/o mount')):
            with self.subTest(change=change):
                info = dict(self.info, **change)
                with patch.object(current.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, json.dumps(info))), \
                        patch.object(current.socket, 'create_connection') as connect:
                    with self.assertRaisesRegex(RuntimeError, message):
                        current.check_current(self.run, 47801)
                connect.assert_not_called()

    def test_paused_or_restarting_container_is_refused(self):
        for state in ({'Paused': True}, {'Restarting': True}):
            with self.subTest(state=state):
                self.info['State'] = dict(Running=True, **state)
                with patch.object(current.subprocess, 'run', return_value=self.inspect_result()), \
                        patch.object(current.socket, 'create_connection') as connect:
                    with self.assertRaisesRegex(RuntimeError, 'not running'):
                        current.check_current(self.run, 47801)
                connect.assert_not_called()

    def test_other_hub_address_or_port_is_refused_before_docker(self):
        for hub in ({'bind': '198.18.200.4', 'port': 47801}, {'bind': '127.0.0.1', 'port': 47431}):
            with self.subTest(hub=hub):
                self.car['hub'] = hub
                (self.run / 'car.json').write_text(json.dumps(self.car))
                with patch.object(current.subprocess, 'run') as docker:
                    with self.assertRaisesRegex(RuntimeError, 'direct loopback tunnel'):
                        current.check_current(self.run, 47801)
                docker.assert_not_called()

    def test_missing_ready_token_or_run_identity_is_refused_before_docker(self):
        for filename in ('READY', 'token.txt', 'car.json', 'container-' + self.stack + '.id'):
            with self.subTest(filename=filename):
                path = self.run / filename
                original = path.read_bytes()
                path.unlink()
                with patch.object(current.subprocess, 'run') as docker:
                    with self.assertRaisesRegex(RuntimeError, 'stale or incomplete'):
                        current.check_current(self.run, 47801)
                docker.assert_not_called()
                path.write_bytes(original)

    def test_docker_inspection_failure_has_actionable_error_and_nonzero_exit(self):
        output = io.StringIO()
        before = self.snapshot()
        with patch.object(current.subprocess, 'run', side_effect=subprocess.TimeoutExpired('docker', 8)), \
                patch.object(current.socket, 'create_connection') as connect, contextlib.redirect_stderr(output):
            code = current.main(['--current', str(self.run), '--hub-port', '47801'])
        self.assertEqual(code, 4)
        self.assertIn('Evidence was preserved', output.getvalue())
        self.assertNotIn('private-test-token', output.getvalue())
        connect.assert_not_called()
        self.assertEqual(self.snapshot(), before)


if __name__ == '__main__':
    unittest.main()
