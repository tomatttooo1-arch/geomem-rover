"""D-035 item 4: dev2's B and D navigation entries on main's M5 runner; A and C stay out.

B = --nav-suite (reference point-to-point after verified startup), D = --estimated-inspect
(motionless estimated-input inspection). The orchestration checks reuse the mocked harness
of tests/test_bottle_member_entry.py; the session checks execute the real shell entry with
a fake rootfs wrapper, adapted from dev2's SessionBoundaryTests (e284cdf) to main's
positional layout (18 = service reply profile, 19 = entry, 20 = domain, 21/22 = map).
"""
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

import test_bottle_member_entry as member_entry  # module, so its classes are not collected twice

MAP_ID = '2a450fd79970fe4c145af5afab6b76a9420b2880c8bbb70581e9aafa1bfab8c7'
SESSION_SCRIPT = Path(__file__).resolve().parents[1]/'tools/agx_bottle_session.sh'


class NavEntryOrchestrationTests(unittest.TestCase):
    setUp = member_entry.MemberEntryTests.setUp
    git = member_entry.MemberEntryTests.git
    command = member_entry.MemberEntryTests.command
    invoke = member_entry.MemberEntryTests.invoke

    def commands(self):
        return [json.loads(line)['argv'] for line in (self.out/'commands.jsonl').read_text().splitlines()]

    def session_launch(self):
        return next(shlex.split(c[-1]) for c in self.commands()
                    if c[0] == 'ssh' and '/tools/agx_bottle_session.sh' in c[-1])

    def external_map(self, image='room.pgm'):
        map_dir = self.repo/'external-map'
        map_dir.mkdir()
        (map_dir/'room.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
        (map_dir/'room.yaml').write_text('image: '+image+'\nresolution: 0.05\norigin: [0, 0, 0]\n')
        return map_dir/'room.yaml'

    def test_nav_suite_is_b_with_isolated_domains_and_no_inference(self):
        self.invoke('--nav-suite', '--remote-python', '/missing/python', '--remote-models', '/missing/models')
        config = json.loads((self.out/'run-config.json').read_text())
        self.assertEqual((config['purpose'], config['nav_entry']), ('nav_point_to_point', 'nav_suite'))
        self.assertEqual((config['startup_only'], config['startup_mode'], config['processing_load']),
                         (True, 'verified', 'idle'))
        launch = self.session_launch()
        self.assertEqual(launch[19], 'bounded')          # main's argument 18 is kept
        self.assertEqual(launch[20:], ['nav_suite', '85', '-', '-'])
        commands = self.commands()
        recorder = next(c for c in commands if '/fpga/robot_sim/tests/record_motion.py' in c)
        self.assertEqual(recorder[-2:], ['--seconds', '750'])
        docker = next(c for c in commands if c[:2] == ['docker', 'run'])
        self.assertIn('ROS_DOMAIN_ID=84', docker)
        self.assertNotIn('FPGA_NAV_INPUT_PROFILE=estimated_inspect', docker)
        relay = next(c for c in commands if 'host.ros_tunnel' in c)
        self.assertNotIn('--profile', relay)
        checks = next(c[-1] for c in commands if c[0] == 'ssh' and c[-1].startswith('set -eu; test -d'))
        self.assertNotIn('/missing/', checks)
        acquire = next(c for c in self.calls if c[1:3] == ['tools/lock.sh', 'acquire'])
        self.assertIn('Nav2 B', acquire[4])
        result = json.loads((self.out/'completed.json').read_text())
        self.assertIsNone(result['inference'])
        self.assertTrue(result['nav_suite_passed'])
        self.assertEqual(result['pose_source'], 'reference')

    def test_estimated_inspect_is_d_and_packages_and_rechecks_the_fixed_map(self):
        map_yaml = self.external_map()
        self.invoke('--estimated-inspect', '--nav-map-yaml', str(map_yaml), '--map-id', MAP_ID,
                    '--remote-python', '/missing/python', '--remote-models', '/missing/models')
        config = json.loads((self.out/'run-config.json').read_text())
        self.assertEqual(config['purpose'], 'estimated_input_inspection')
        self.assertEqual(config['nav_input_profile'], 'estimated_inspect')
        self.assertEqual(config['startup_mode'], 'observe')
        self.assertFalse(config['map_contract']['coordinate_alignment_verified'])
        self.assertEqual((self.out/'nav-map/map.yaml').read_bytes(), map_yaml.read_bytes())
        commands = self.commands()
        relay = next(c for c in commands if c[:2] == ['docker', 'exec'] and 'host.ros_tunnel' in c)
        self.assertEqual(relay[-2:], ['--profile', 'estimated_inspect'])
        docker = next(c for c in commands if c[:2] == ['docker', 'run'])
        self.assertIn('FPGA_NAV_INPUT_PROFILE=estimated_inspect', docker)
        self.assertEqual(self.session_launch()[20:], [
            'estimated_inspect', '85', '/workspace/downloads/'+config['session']+'-nav-map/map.yaml', MAP_ID])
        digest = config['map_contract']['nav_map_yaml_sha256']
        self.assertTrue(any(c[0] == 'ssh' and digest in c[-1] and 'sha256sum' in c[-1] for c in commands))
        transfer = [c for c in commands if c[0] == 'scp' and str(self.out/'nav-map') in c]
        self.assertEqual(len(transfer), 1)
        result = json.loads((self.out/'completed.json').read_text())
        self.assertEqual(result['pose_source'], 'estimated')
        self.assertIsNone(result['inference'])

    def test_bottle_task_launch_is_unchanged(self):
        self.invoke()
        launch = self.session_launch()
        self.assertEqual(len(launch), 20)
        self.assertEqual(launch[19], 'bounded')
        config = json.loads((self.out/'run-config.json').read_text())
        self.assertIsNone(config['nav_entry'])
        self.assertEqual((config['server_domain'], config['agx_domain']), (73, 42))

    def test_invalid_entry_combinations_are_rejected_before_any_device_action(self):
        map_yaml = self.external_map()
        for flags in (('--estimated-inspect',),
                      ('--nav-map-yaml', str(map_yaml)),
                      ('--map-id', MAP_ID),
                      ('--nav-suite', '--map-id', MAP_ID),
                      ('--nav-suite', '--estimated-inspect', '--nav-map-yaml', str(map_yaml), '--map-id', MAP_ID),
                      ('--nav-suite', '--startup-only'),
                      ('--nav-suite', '--hold-after-scenario', '5'),
                      ('--estimated-inspect', '--nav-map-yaml', str(map_yaml), '--map-id', 'A'*64)):
            with self.subTest(flags=flags), self.assertRaises(SystemExit):
                self.invoke('--plan', *flags)
        self.assertEqual(self.calls, [])

    def test_map_image_may_not_shadow_the_package_files(self):
        map_yaml = self.external_map(image='contract.json')
        (map_yaml.parent/'contract.json').write_text('{}')
        with self.assertRaises(SystemExit):
            self.invoke('--plan', '--estimated-inspect', '--nav-map-yaml', str(map_yaml), '--map-id', MAP_ID)

    def test_a_and_c_entries_are_not_carried_over(self):
        for flags in (('--nav-only',), ('--nav-c', 'baseline'), ('--relay-profile-calls',)):
            with self.subTest(flags=flags), self.assertRaises(SystemExit):
                self.invoke('--plan', *flags)
        self.assertEqual(self.calls, [])


class NavEntrySessionTests(unittest.TestCase):
    """Run the real session script; the fake rootfs wrapper logs each ROS call, exits 77."""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.workspace = self.root/'workspace'; self.release = self.workspace/'releases/revision'
        self.rootfs = self.root/'rootfs'; self.fake_bin = self.root/'bin'
        for path in (self.release/'tools', self.rootfs/'opt/ros/humble', self.fake_bin,
                     self.workspace/'downloads/bottle-test-nav-map', self.workspace/'ipc',
                     self.workspace/'runs'):
            path.mkdir(parents=True)
        (self.workspace/'downloads/bottle-test-nav-map/map.yaml').write_text('image: map.pgm\n')
        fake_rootfs = self.release/'tools/agx_rootfs.sh'
        fake_rootfs.write_text(
            '#!/usr/bin/env bash\nset -eu\nshift 2\nprintf "%s\\n" "$*" >> "$FPGA_AGX_WORKSPACE/ros-calls.txt"\n'
            'if [[ "$1" == test && "$2" == -f && "$3" == /workspace/* ]]; then\n'
            '  test -f "$FPGA_AGX_WORKSPACE/${3#/workspace/}"\n  exit $?\nfi\nexit 77\n')
        fake_rootfs.chmod(0o755)
        fake_git = self.fake_bin/'git'; fake_git.write_text('#!/bin/sh\necho revision\n'); fake_git.chmod(0o755)

    def run_session(self, startup_mode, entry_args):
        command = ['bash', str(SESSION_SCRIPT), str(self.release), 'bottle-test', '19999', 'dwb', '0', 'none',
                   startup_mode, '1', 'idle', 'udp', 'poll', 'none', 'ssdlite', str(self.workspace),
                   str(self.rootfs), '/missing/python', '/missing/models', 'bounded', *entry_args]
        return subprocess.run(command, env={**os.environ, 'PATH': str(self.fake_bin)+':'+os.environ['PATH']},
                              capture_output=True, text=True)

    def calls(self):
        path = self.workspace/'ros-calls.txt'
        return path.read_text().splitlines() if path.exists() else []

    def test_inspect_map_is_checked_inside_rootfs_before_build(self):
        result = self.run_session('observe', ['estimated_inspect', '85',
                                              '/workspace/downloads/bottle-test-nav-map/map.yaml', MAP_ID])
        self.assertEqual(result.returncode, 77, result.stderr)
        self.assertEqual(self.calls()[0], 'test -f /workspace/downloads/bottle-test-nav-map/map.yaml')
        self.assertIn('nav_controller_variants.py', self.calls()[1])

    def test_inspect_missing_map_stops_before_build_or_ros_nodes(self):
        result = self.run_session('observe', ['estimated_inspect', '85',
                                              '/workspace/downloads/bottle-test-nav-map/missing.yaml', MAP_ID])
        self.assertNotIn(result.returncode, (0, 77))
        self.assertEqual(self.calls(), ['test -f /workspace/downloads/bottle-test-nav-map/missing.yaml'])

    def test_nav_suite_skips_inference_and_reaches_the_ros_build(self):
        result = self.run_session('verified', ['nav_suite', '85', '-', '-'])
        self.assertEqual(result.returncode, 77, result.stderr)
        self.assertIn('nav_controller_variants.py', self.calls()[0])
        self.assertFalse((self.workspace/'runs/bottle-test/inference.log').exists())

    def test_mismatched_entry_modes_stop_before_any_ros_call(self):
        for mode, args in (('observe', ['nav_suite', '85', '-', '-']),
                           ('verified', ['estimated_inspect', '85',
                                         '/workspace/downloads/bottle-test-nav-map/map.yaml', MAP_ID]),
                           ('observe', ['estimated_inspect', '85', '/elsewhere/map.yaml', MAP_ID]),
                           ('observe', ['nav_only', '85', '-', '-'])):
            with self.subTest(mode=mode, args=args):
                result = self.run_session(mode, args)
                self.assertNotIn(result.returncode, (0, 77))
                self.assertEqual(self.calls(), [])


if __name__ == '__main__':
    unittest.main()
