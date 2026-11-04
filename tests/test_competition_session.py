"""Laptop orchestration failure boundaries, with no SSH, Docker or hardware calls."""
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
SPEC = importlib.util.spec_from_file_location("competition_session", ROOT / "ui/geomem/tools/competition/session.py")
session = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(session)


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.config = Path(self.tmp.name) / "session.json"
        self.config.write_text((ROOT / "ui/geomem/tools/competition/session.example.json").read_text(), encoding="utf-8")

    def invoke(self, action, execute=False):
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            return session.main([action, "--config", str(self.config)] + (["--execute"] if execute else []))

    def test_plan_never_executes_or_creates_session_data(self):
        with patch.object(session.subprocess, "run") as run:
            self.assertEqual(self.invoke("mapping"), 0)
            run.assert_not_called()
        self.assertFalse((self.config.parent / "session-data").exists())

    def test_failed_nano_start_cannot_open_bridge(self):
        with patch.object(session.subprocess, "run", side_effect=subprocess.CalledProcessError(2, "ssh")) as run:
            with self.assertRaises(subprocess.CalledProcessError):
                self.invoke("mapping", True)
            self.assertEqual(run.call_count, 1)
            self.assertIn("nano_up.sh", run.call_args[0][0][-1])

    def test_precheck_stops_on_remote_failure(self):
        with patch.object(session.subprocess, "run", side_effect=subprocess.CalledProcessError(1, "ssh")) as run:
            with self.assertRaises(subprocess.CalledProcessError):
                self.invoke("check", True)
            self.assertEqual(run.call_count, 1)
            remote = run.call_args[0][0][-1]
            self.assertTrue(remote.startswith("set -e;"))
            self.assertIn("sudo -n -l /usr/local/sbin/geomem-kv-start", remote)

    def test_demo_requires_first_run_artifacts_before_ssh(self):
        with patch.object(session.subprocess, "run") as run:
            with self.assertRaises(SystemExit) as error:
                self.invoke("demo", True)
            self.assertEqual(error.exception.code, 2)
            run.assert_not_called()

    def test_invalid_map_hash_is_refused_before_ssh_or_local_data_creation(self):
        for digest in ('short', 'A' * 64, 123, 0, None, False):
            with self.subTest(digest=digest):
                c = json.loads(self.config.read_text())
                c.update(map_yaml='/map/export/map.yaml', memory_file='/run/memory_final.json', map_sha256=digest)
                self.config.write_text(json.dumps(c))
                with patch.object(session.subprocess, 'run') as run:
                    with self.assertRaises(SystemExit) as error:
                        self.invoke('demo', True)
                    self.assertEqual(error.exception.code, 2)
                    run.assert_not_called()
                self.assertFalse((self.config.parent / 'session-data').exists())

    def test_demo_passes_full_database_hash_to_nano_preflight(self):
        c = json.loads(self.config.read_text())
        c.update(map_yaml='/map/export/map.yaml', memory_file='/run/memory_final.json', map_sha256='a' * 64)
        self.config.write_text(json.dumps(c))
        with patch.object(session.subprocess, 'run', side_effect=subprocess.CalledProcessError(6, 'demo preflight')) as run:
            with self.assertRaises(subprocess.CalledProcessError):
                self.invoke('demo', True)
        run.assert_called_once()
        self.assertTrue(run.call_args.args[0][-1].endswith('a' * 64))
        command = run.call_args.args[0][-1]
        self.assertTrue(command.startswith('set -e;'))
        self.assertIn('test -L /data/fpga/car/state/car.current', command)
        self.assertLess(command.index('check_demo.py'), command.index('nano_up.sh'))

    def test_direct_plan_uses_loopback_tunnel_and_no_lab_jump(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output), patch.object(session.subprocess, 'run') as run:
            session.main(['bridge', '--config', str(self.config)])
        run.assert_not_called()
        plan = output.getvalue()
        self.assertIn('127.0.0.1:18791:127.0.0.1:47801', plan)
        self.assertIn('127.0.0.1:8792:127.0.0.1:47802', plan)
        self.assertIn('--hub 127.0.0.1:18791', plan)
        self.assertNotIn('srtp-server', plan)
        self.assertNotIn('ProxyJump', plan)

    def test_web_start_failure_closes_owned_tunnel_and_bridge(self):
        from unittest.mock import MagicMock
        tunnel, bridge = MagicMock(), MagicMock()
        tunnel.poll.return_value = bridge.poll.return_value = None
        c = json.loads(self.config.read_text()); c['operator_root'] = str(ROOT / 'ui/geomem/operator')
        self.config.write_text(json.dumps(c))
        with patch.object(session.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0)), \
                patch.object(session.subprocess, 'Popen', side_effect=[tunnel, bridge, OSError('node unavailable')]), \
                patch.object(session.socket, 'socket'), patch.object(session.socket, 'create_connection'), \
                patch.object(session.os, 'chmod'):
            with self.assertRaisesRegex(OSError, 'node unavailable'):
                self.invoke('bridge', True)
        for proc in (tunnel, bridge):
            proc.terminate.assert_called_once_with()
            proc.wait.assert_called_once_with(timeout=10)

    def test_occupied_ports_never_spawn_or_copy_token(self):
        with patch.object(session.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0)) as run, \
                patch.object(session.subprocess, 'Popen') as popen, \
                patch.object(session.socket, 'socket') as sock:
            sock.return_value.__enter__.return_value.bind.side_effect = OSError('occupied')
            with self.assertRaisesRegex(RuntimeError, 'occupied'):
                self.invoke('bridge', True)
        popen.assert_not_called()
        self.assertFalse(any(call.args[0][0] == 'scp' for call in run.call_args_list))

    def test_session_becoming_stale_before_token_copy_cannot_open_laptop_services(self):
        with patch.object(session.subprocess, 'run', side_effect=[subprocess.CompletedProcess([], 0),
                           subprocess.CalledProcessError(4, 'check_current.py')]) as run, \
                patch.object(session.subprocess, 'Popen') as popen, patch.object(session.socket, 'socket'):
            with self.assertRaises(subprocess.CalledProcessError):
                self.invoke('bridge', True)
        self.assertEqual(run.call_count, 2)
        self.assertIn('check_current.py', run.call_args_list[1].args[0][-1])
        self.assertFalse(any(call.args[0][0] == 'scp' for call in run.call_args_list))
        popen.assert_not_called()


if __name__ == "__main__":
    unittest.main()
