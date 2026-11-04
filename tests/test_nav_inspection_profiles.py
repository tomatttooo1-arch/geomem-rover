"""dev1's producer output into dev2's no-motion inspector, without ROS or hardware.

The positive data are constructed by the production LocalizationGate and
OdomBaseAdapter. Publisher/TF subscription facts are synthetic and explicitly
checked separately; this test does not certify a live DDS graph or navigation.
"""
from contextlib import redirect_stderr
from copy import deepcopy
import io
import itertools
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

from host.localization_gate import KINDS, LocalizationGate
from host.odom_base_adapter import OdomBaseAdapter, load_extrinsics_manifest
from robot_sim.tests import nav_input_inspect as inspect


MAP_ID = 'a' * 64  # Synthetic map identity, not a certified navigation map.
EXTRINSICS = Path(__file__).resolve().parents[1] / 'robot_sim/fpga_sim/config/extrinsics_sim.json'


def raw_odom(stamp, tracked):
    sec, nano = divmod(stamp, 10**9)
    return {
        'header': {'stamp': {'sec': sec, 'nanosec': nano}, 'frame_id': 'odom'},
        'child_frame_id': tracked,
        'pose': {'pose': {'position': {'x': 1., 'y': 2., 'z': 3.},
                          'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}},
                 'covariance': [0.] * 36},
        'twist': {'twist': {'linear': {'x': .1, 'y': 0., 'z': 0.},
                            'angular': {'x': 0., 'y': 0., 'z': .01}},
                  'covariance': [0.] * 36},
    }


def gate_inputs(raw):
    world_header = deepcopy(raw['header'])
    world_header['frame_id'] = 'map'
    return {
        'odom': raw,
        'odom_info': {'header': raw['header'], 'lost': False, 'covariance': [0.] * 36},
        'info': {'header': world_header, 'loop_closure_id': 1,
                 'proximity_detection_id': 0},
        'localization_pose': {'header': world_header, 'pose': deepcopy(raw['pose'])},
    }


def produced_inputs(tracked, transient_fault=None):
    """Produce actual gate acquisitions and adapter output; optionally recover after a fault."""
    if tracked == 'camera_link':
        extrinsic, provenance = load_extrinsics_manifest(EXTRINSICS)
    else:
        extrinsic, provenance = np.eye(4), 'identity'
    adapter = OdomBaseAdapter(extrinsic, tracked_frame=tracked,
                              extrinsics_provenance=provenance)
    gate = LocalizationGate(MAP_ID, {1}, tracked_frame=tracked,
                            navigation_ready=False)
    status_rows, odom_rows, approvals = [], [], []
    output = None
    def healthy_acquisition(index):
        nonlocal output
        stamp, wall = 10**9 + index * 10**8, index * .1
        row = raw_odom(stamp, tracked)
        output = adapter.process(row)
        assert output is not None
        gate.acquisition(stamp, wall, stamp)
        for kind, message in gate_inputs(row).items():
            approvals.extend(gate.ingest(kind, message, wall, stamp))
        status = gate.status(wall, stamp)
        status['source'] = 'gate-publisher'  # Modelled publisher identity.
        status_rows.append(status)
        odom_rows.append({'frame_id': output['odom']['frame_id'],
                          'child_frame_id': output['odom']['child_frame_id'],
                          'source': 'adapter-publisher'})

    for index in range(1, 4):
        healthy_acquisition(index)
    if transient_fault is not None:
        if transient_fault == 'lost':
            gate.revoke('lost', 'synthetic_tracking_loss')
        elif transient_fault == 'wrong_frame':
            wrong_stamp = 1_350_000_000
            gate.acquisition(wrong_stamp, .35, wrong_stamp)
            wrong_odom = raw_odom(wrong_stamp, tracked)
            wrong_odom['child_frame_id'] = 'wrong_frame'
            gate.ingest('odom', wrong_odom, .35, wrong_stamp)
            assert gate.state == 'lost'
        elif transient_fault == 'healthy_rollover':
            pass
        else:
            raise ValueError('unknown transient fault')
        if transient_fault != 'healthy_rollover':
            invalid = gate.status(.35, 1_350_000_000)
            invalid['source'] = 'gate-publisher'
            status_rows.append(invalid)
            if transient_fault == 'wrong_frame':
                gate.discontinuity(1)  # Only clear the incomplete bad frame for recovery.
        # The inspector retains 64 tail rows. Recover with >64 healthy rows so
        # only its callback fault latch can preserve the earlier violation.
        for index in range(4, 85):
            healthy_acquisition(index)

    assert len(approvals) >= 1
    assert adapter.status()['published'] == len(odom_rows)
    assert status_rows[-1]['state'] == 'valid'
    assert status_rows[-1]['navigation_ready'] is False

    def edge(parent, child, source):
        return {'parent': parent, 'child': child, 'source': source,
                'stamp_sec': 1, 'stamp_nanosec': 300000000}
    transforms = {
        '/tf': [edge('map', 'odom', 'gate-publisher'),
                edge(output['tf']['parent'], output['tf']['child'], 'adapter-publisher')],
        '/tf_static': [edge('base_link', 'camera_link', 'external-extrinsics'),
                       edge('camera_link', 'camera_optical_frame', 'external-extrinsics')],
    }
    return status_rows, odom_rows, transforms


def model_graph():
    publishers = {topic: [{'source': topic}] for topic in inspect.INPUT_TOPICS}
    commands = {topic: [] for topic in inspect.COMMAND_TOPICS}
    lifecycle = {name: 'unconfigured' for name in inspect.LIFECYCLE_NODES}
    return publishers, commands, lifecycle


def evaluate(tracked, expected=None, **changes):
    statuses, odoms, transforms = produced_inputs(tracked)
    publishers, commands, lifecycle = model_graph()
    params = dict(map_id=MAP_ID, status_rows=statuses, odom_rows=odoms,
                  transforms=transforms, publishers=publishers,
                  command_publishers=commands, lifecycle=lifecycle)
    params.update(changes)
    if expected is not None:
        params['tracked_frame'] = expected
    return inspect.evaluate_observation(**params)


class ProfileMatrixTests(unittest.TestCase):
    def test_two_by_two_requires_explicit_matching_configuration(self):
        for provided in inspect.TRACKED_FRAMES:
            for expected in inspect.TRACKED_FRAMES:
                with self.subTest(gate=provided, expected=expected):
                    result = evaluate(provided, expected)
                    self.assertEqual(result['passed'], provided == expected)
                    self.assertFalse(result['navigation_ready_declared'])
                    failed = {name for name, passed in result['checks'].items() if not passed}
                    self.assertEqual(failed, set() if provided == expected else {
                        'latest_status_valid_for_map', 'status_stable_after_first_valid'})
        # Old two-argument/pure-function calls retain camera_link semantics.
        self.assertTrue(evaluate('camera_link')['passed'])
        self.assertFalse(evaluate('base_link')['passed'])
        with self.assertRaises(ValueError):
            evaluate('camera_link', 'optical_frame')

    def test_wrong_map_loss_duplicate_authority_and_motion_still_fail_closed(self):
        for tracked in inspect.TRACKED_FRAMES:
            statuses, odoms, transforms = produced_inputs(tracked)
            publishers, commands, lifecycle = model_graph()
            cases = {
                'wrong_map': dict(status_rows=statuses[:-1] + [
                    {**statuses[-1], 'map_id': 'b' * 64}]),
                'invalid_status': dict(status_rows=statuses[:-1] + [
                    {**statuses[-1], 'state': 'lost'}], status_fault_after_valid=True),
                'duplicate_publisher': dict(publishers={**publishers,
                    '/odom': [{'source': 'one'}, {'source': 'two'}]},
                    observed_input_sources={'/odom': {'one', 'two'},
                                            '/fpga/localization_status': {'gate-publisher'}}),
                'faulty_odom': dict(odom_rows=odoms[:-1] + [
                    {**odoms[-1], 'child_frame_id': 'camera_link'}], odom_fault_seen=True),
                'command_publisher': dict(command_publishers={**commands,
                    '/cmd_vel': [{'source': 'motion'}]}),
                'active_lifecycle': dict(lifecycle={**lifecycle,
                    'controller_server': 'active'}),
            }
            for name, override in cases.items():
                with self.subTest(profile=tracked, fault=name):
                    params = dict(map_id=MAP_ID, status_rows=statuses,
                                  odom_rows=odoms, transforms=transforms,
                                  publishers=publishers, command_publishers=commands,
                                  lifecycle=lifecycle, tracked_frame=tracked)
                    params.update(override)
                    self.assertFalse(inspect.evaluate_observation(**params)['passed'])
        self.assertTrue(evaluate('camera_link', status_rows=[{
            **produced_inputs('camera_link')[0][-1], 'navigation_ready': True
        }])['navigation_ready_declared'])  # Instrument positive only; no bundle changed.


class MainCallbackTests(unittest.TestCase):
    """Run actual main callbacks and final verdict under inert ROS module stand-ins."""

    def run_main(self, provided, expected=None, transient_fault=None,
                 duplicate_source=None):
        statuses, odoms, transforms = produced_inputs(provided, transient_fault)
        registry = {}

        class Client:
            def service_is_ready(self): return True
            def call_async(self, request):
                return types.SimpleNamespace(
                    done=lambda: True, exception=lambda: None,
                    result=lambda: types.SimpleNamespace(
                        current_state=types.SimpleNamespace(label='unconfigured')))

        class Node:
            def __init__(self):
                self.callbacks = {}
                self.delivered = False
            def create_subscription(self, kind, topic, callback, qos):
                self.callbacks[topic] = callback
                return object()
            def create_client(self, kind, topic): return Client()
            def get_publishers_info_by_topic(self, topic):
                if topic in inspect.COMMAND_TOPICS:
                    return []
                return [types.SimpleNamespace(node_name='synthetic_provider',
                    node_namespace='/', topic_type='synthetic', endpoint_gid=bytes([1] * 16))]
            def destroy_node(self): pass

        node = Node()
        provider = types.SimpleNamespace(publisher_gid=bytes([1] * 16))
        other_provider = types.SimpleNamespace(publisher_gid=bytes([2] * 16))

        def stamp(row):
            return types.SimpleNamespace(sec=row['stamp_sec'], nanosec=row['stamp_nanosec'])

        def deliver(*unused, **kwargs):
            if node.delivered:
                return
            node.delivered = True
            for index, row in enumerate(odoms):
                msg = types.SimpleNamespace(
                    header=types.SimpleNamespace(
                        stamp=types.SimpleNamespace(sec=1, nanosec=300000000),
                        frame_id=row['frame_id']), child_frame_id=row['child_frame_id'])
                source = (other_provider if duplicate_source == '/odom' and index == 3
                          else provider)
                node.callbacks['/odom'](msg, source)
            for index, status in enumerate(statuses):
                source = (other_provider if duplicate_source == '/fpga/localization_status'
                          and index == 3 else provider)
                node.callbacks['/fpga/localization_status'](
                    types.SimpleNamespace(data=json.dumps(status)), source)
            for topic in ('/tf', '/tf_static'):
                messages = []
                for row in transforms[topic]:
                    t = types.SimpleNamespace(x=0., y=0., z=0.)
                    q = types.SimpleNamespace(x=0., y=0., z=0., w=1.)
                    messages.append(types.SimpleNamespace(
                        header=types.SimpleNamespace(stamp=stamp(row), frame_id=row['parent']),
                        child_frame_id=row['child'],
                        transform=types.SimpleNamespace(translation=t, rotation=q)))
                node.callbacks[topic](types.SimpleNamespace(transforms=messages), provider)

        class Qos:
            def __init__(self, **kwargs): self.__dict__.update(kwargs)

        def module(name, **attrs):
            result = types.ModuleType(name)
            result.__dict__.update(attrs)
            return result

        class GetState:
            class Request: pass

        fake = {
            'rclpy': module('rclpy', init=lambda **kwargs: None,
                            create_node=lambda name: node, spin_once=deliver,
                            shutdown=lambda: None),
            'rclpy.qos': module('rclpy.qos',
                DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL='transient'),
                ReliabilityPolicy=types.SimpleNamespace(RELIABLE='reliable'),
                QoSProfile=Qos, qos_profile_sensor_data=object()),
            'lifecycle_msgs.srv': module('lifecycle_msgs.srv', GetState=GetState),
            'nav_msgs.msg': module('nav_msgs.msg', Odometry=object),
            'std_msgs.msg': module('std_msgs.msg', String=object),
            'tf2_msgs.msg': module('tf2_msgs.msg', TFMessage=object),
        }
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            yaml = base / 'map.yaml'
            yaml.write_text('image: map.pgm\n')
            out = base / 'probe.json'
            args = ['--duration', '1', '--map-id', MAP_ID,
                    '--map-yaml', str(yaml), '--out', str(out)]
            if expected is not None:
                args += ['--tracked-frame', expected]
            with patch.dict(sys.modules, fake), patch.object(inspect.time, 'monotonic',
                    side_effect=itertools.chain([0., 0., 0.], itertools.repeat(2.))):
                code = inspect.main(args)
            result = json.loads(out.read_text())
        return code, result, node

    def test_main_uses_expected_frame_in_callback_and_final_evaluation(self):
        for provided in inspect.TRACKED_FRAMES:
            for expected in inspect.TRACKED_FRAMES:
                with self.subTest(gate=provided, expected=expected):
                    code, result, node = self.run_main(provided, expected)
                    self.assertEqual(code, 0 if provided == expected else 3)
                    self.assertEqual(result['passed'], provided == expected)
                    self.assertEqual(result['expected_tracked_frame'], expected)
                    self.assertEqual(result['navigation_ready_declared'], False)
                    self.assertEqual(result['motionless_contract'], True)
                    self.assertEqual(result['checks']['no_command_publishers'], True)
                    self.assertEqual(result['checks']['all_nav_lifecycle_nonactive'], True)
                    # For base data with an expected camera frame, the callback
                    # must NOT latch an apparently valid status before the verdict.
                    if provided == 'camera_link' and expected == 'base_link':
                        self.assertFalse(result['checks']['latest_status_valid_for_map'])
                    self.assertTrue(node.delivered)
        self.assertEqual(self.run_main('camera_link')[0], 0)
        self.assertEqual(self.run_main('base_link')[0], 3)

    def test_invalid_cli_profile_rejected_before_ros_import_or_output(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            yaml = base / 'map.yaml'; yaml.write_text('image: map.pgm\n')
            out = base / 'probe.json'
            args = ['--duration', '1', '--map-id', MAP_ID,
                    '--map-yaml', str(yaml), '--out', str(out),
                    '--tracked-frame', 'not_a_frame']
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
                inspect.main(args)
            self.assertEqual(exc.exception.code, 2)
            self.assertFalse(out.exists())

    def test_callback_latches_fault_after_valid_even_after_rollover(self):
        for tracked, expected in (('camera_link', None), ('base_link', 'base_link')):
            for fault in ('lost', 'wrong_frame'):
                with self.subTest(tracked=tracked, transient_fault=fault):
                    code, result, _ = self.run_main(tracked, expected, fault)
                    self.assertEqual(code, 3)
                    self.assertFalse(result['passed'])
                    self.assertTrue(result['checks']['latest_status_valid_for_map'])
                    self.assertFalse(result['checks']['status_stable_after_first_valid'])
                    tail = result['bounded_samples']['/fpga/localization_status']
                    self.assertEqual(len(tail), 64)
                    self.assertTrue(all(row['state'] == 'valid' and
                        row['tracked_frame'] == tracked for row in tail))

    def test_original_source_authority_check_survives_bounded_rollover(self):
        for tracked, expected in (('camera_link', None), ('base_link', 'base_link')):
            for topic in ('/odom', '/fpga/localization_status'):
                with self.subTest(tracked=tracked, second_source=topic):
                    code, result, _ = self.run_main(
                        tracked, expected, 'healthy_rollover', duplicate_source=topic)
                    self.assertEqual(code, 3)
                    self.assertFalse(result['checks']['single_authority_per_input'])
                    self.assertTrue(result['checks']['latest_status_valid_for_map'])
                    self.assertTrue(result['checks']['status_stable_after_first_valid'])
                    self.assertEqual(len(result['bounded_samples'][topic]),
                                     32 if topic == '/odom' else 64)
                    self.assertEqual(len({row['source'] for row in
                        result['bounded_samples'][topic]}), 1)
                    self.assertEqual(len(result['observed_input_sources'][topic]), 2)


if __name__ == '__main__':
    unittest.main()
