"""Host-only behavior checks for the motionless estimated inspection entry."""
import contextlib
import ast
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from host import ros_tunnel
from robot_sim.tests import nav_input_inspect as inspect


MAP_ID = 'a' * 64


def valid_inputs(ready=False):
    status = {'schema': 1, 'map_id': MAP_ID, 'pose_source': 'estimated',
              'world_frame': 'map', 'tracked_frame': 'camera_link',
              'state': 'valid', 'navigation_ready': ready}
    transforms = {'/tf': [
        {'parent': 'map', 'child': 'odom', 'source': 'map-provider', 'stamp_sec': 1, 'stamp_nanosec': 0},
        {'parent': 'odom', 'child': 'base_link', 'source': 'odom-provider', 'stamp_sec': 1, 'stamp_nanosec': 0}],
        '/tf_static': [
        {'parent': 'base_link', 'child': 'camera_link', 'source': 'extrinsics', 'stamp_sec': 0, 'stamp_nanosec': 0},
        {'parent': 'camera_link', 'child': 'camera_optical_frame', 'source': 'extrinsics', 'stamp_sec': 0, 'stamp_nanosec': 0}]}
    publishers = {topic: [{'source': topic}] for topic in inspect.INPUT_TOPICS}
    lifecycle = {name: 'unconfigured' for name in inspect.LIFECYCLE_NODES}
    return status, transforms, publishers, lifecycle


class VerdictTests(unittest.TestCase):
    def test_real_callbacks_retain_source_conflict_beyond_sample_tails(self):
        tree = ast.parse(Path(inspect.__file__).read_text())
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        functions = [n for n in main.body if isinstance(n, ast.FunctionDef)
                     and n.name in ('source_id', 'append', 'odom', 'status')]
        verdict = next(n for n in ast.walk(main) if isinstance(n, ast.Assign)
                       and any(isinstance(t, ast.Name) and t.id == 'verdict' for t in n.targets))
        callbacks = compile(ast.fix_missing_locations(ast.Module(body=functions, type_ignores=[])),
                            inspect.__file__, 'exec')
        evaluate = compile(ast.fix_missing_locations(ast.Module(body=[verdict], type_ignores=[])),
                           inspect.__file__, 'exec')
        for conflict_topic in (None, '/odom', '/fpga/localization_status'):
            with self.subTest(conflict_topic=conflict_topic):
                row, transforms, publishers, lifecycle = valid_inputs()
                state = dict(status_seen_valid=False, status_fault_after_valid=False, odom_fault_seen=False)
                samples = {topic: [] for topic in inspect.INPUT_TOPICS}
                # `args` stands in for the parsed CLI; --tracked-frame defaults to camera_link
                # (added on the team-integration branch), so carry that default here too.
                env = dict(inspect.__dict__, args=types.SimpleNamespace(map_id=MAP_ID, tracked_frame='camera_link'),
                           samples=samples, state=state,
                           observed_input_sources={'/odom': set(), '/fpga/localization_status': set()},
                           observed_transforms=transforms, publishers=publishers,
                           command_publishers={topic: [] for topic in inspect.COMMAND_TOPICS},
                           lifecycle=lifecycle)
                exec(callbacks, env)
                message = types.SimpleNamespace(
                    header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=1, nanosec=0), frame_id='odom'),
                    child_frame_id='base_link')
                status = types.SimpleNamespace(data=json.dumps(row))
                first = types.SimpleNamespace(publisher_gid=[1]*16)
                second = types.SimpleNamespace(publisher_gid=[2]*16)
                env['odom'](message, first)
                env['status'](status, first)
                for _ in range(80):
                    env['odom'](message, second if conflict_topic == '/odom' else first)
                    env['status'](status, second if conflict_topic == '/fpga/localization_status' else first)
                self.assertEqual(len(samples['/odom']), 32)
                self.assertEqual(len(samples['/fpga/localization_status']), 64)
                # All retained rows are now from one source, including the bad run.
                for topic in ('/odom', '/fpga/localization_status'):
                    self.assertEqual(len({item['source'] for item in samples[topic]}), 1)
                exec(evaluate, env)
                self.assertEqual(env['verdict']['passed'], conflict_topic is None)
                self.assertEqual(env['verdict']['checks']['single_authority_per_input'],
                                 conflict_topic is None)

    def evaluate(self, statuses=None, odom=None, transforms=None, publishers=None,
                 commands=None, lifecycle=None, **kwargs):
        status, default_tf, default_publishers, default_lifecycle = valid_inputs()
        return inspect.evaluate_observation(
            MAP_ID, statuses if statuses is not None else [status],
            odom if odom is not None else [{'frame_id': 'odom', 'child_frame_id': 'base_link'}],
            transforms if transforms is not None else default_tf,
            publishers if publishers is not None else default_publishers,
            commands if commands is not None else {topic: [] for topic in inspect.COMMAND_TOPICS},
            lifecycle if lifecycle is not None else default_lifecycle, **kwargs)

    def test_valid_motionless_wiring_can_observe_navigation_ready_false(self):
        result = self.evaluate()
        self.assertTrue(result['passed'])
        self.assertFalse(result['navigation_ready_declared'])

    def test_later_status_or_odom_failure_cannot_be_erased_by_earlier_success(self):
        valid, _, _, _ = valid_inputs()
        failures = ({'malformed': True}, {**valid, 'map_id': 'b'*64},
                    {**valid, 'tracked_frame': 'base_link'}, {**valid, 'state': 'lost'})
        for failure in failures:
            with self.subTest(failure=failure):
                self.assertFalse(self.evaluate(statuses=[valid, failure])['passed'])
        self.assertFalse(self.evaluate(
            odom=[{'frame_id': 'odom', 'child_frame_id': 'base_link'},
                  {'frame_id': 'map', 'child_frame_id': 'base_link'}])['passed'])
        self.assertFalse(self.evaluate(status_fault_after_valid=True)['passed'])
        self.assertFalse(self.evaluate(odom_fault_seen=True)['passed'])

    def test_tf_channels_parents_sources_and_topic_publishers_are_enforced(self):
        _, base, publishers, lifecycle = valid_inputs()
        cases = []
        wrong_channel = {key: list(value) for key, value in base.items()}
        wrong_channel['/tf'].append(wrong_channel['/tf_static'].pop())
        cases.append(('wrong_channel', wrong_channel, publishers, lifecycle))
        multi_parent = {key: list(value) for key, value in base.items()}
        multi_parent['/tf'].append({'parent': 'world', 'child': 'base_link', 'source': 'bad'})
        cases.append(('multi_parent', multi_parent, publishers, lifecycle))
        multi_source = {key: list(value) for key, value in base.items()}
        multi_source['/tf'].append({**multi_source['/tf'][0], 'source': 'second-map-provider'})
        cases.append(('multi_source', multi_source, publishers, lifecycle))
        duplicate_publishers = {key: list(value) for key, value in publishers.items()}
        duplicate_publishers['/odom'].append({'source': 'second-odom'})
        cases.append(('duplicate_odom_publisher', base, duplicate_publishers, lifecycle))
        missing_lifecycle = dict(lifecycle); missing_lifecycle.pop('map_server')
        cases.append(('missing_lifecycle', base, publishers, missing_lifecycle))
        for name, transforms, pubs, states in cases:
            with self.subTest(name=name):
                self.assertFalse(self.evaluate(transforms=transforms, publishers=pubs,
                                               lifecycle=states)['passed'])

    def test_repeated_same_stamp_is_diagnostic_not_multi_source_evidence(self):
        _, transforms, _, _ = valid_inputs()
        repeated = {key: list(value) for key, value in transforms.items()}
        repeated['/tf'].append(dict(repeated['/tf'][0]))
        result = self.evaluate(transforms=repeated)
        self.assertTrue(result['passed'])
        self.assertTrue(result['duplicate_tf_edge_stamps'])


class RosConstructionTests(unittest.TestCase):
    def relay_modules(self, registry):
        class Logger:
            def info(self, *_): pass
            def error(self, *_): pass
        class Node:
            def __init__(self, name): self.name = name
            def create_publisher(self, kind, topic, qos):
                registry['publishers'].append(topic)
                return types.SimpleNamespace(publish=lambda message: None)
            def create_subscription(self, kind, topic, callback, qos):
                registry['subscriptions'].append(topic); return object()
            def create_timer(self, *args, **kwargs): return object()
            def get_logger(self): return Logger()
            def destroy_node(self): pass
        class Subscriber:
            def __init__(self, node, kind, topic, **kwargs): registry['subscriptions'].append(topic)
            def registerCallback(self, callback): pass
        class Filter:
            def signalMessage(self, message): pass
        class Sync:
            def __init__(self, filters, *args):
                self.queues = [[] for _ in filters]
                self.lock = contextlib.nullcontext()
            def registerCallback(self, callback): pass
        class Dummy: pass
        modules = {
            'rclpy': {'init': lambda **kwargs: None, 'spin': lambda node: (_ for _ in ()).throw(KeyboardInterrupt()),
                      'ok': lambda: False, 'shutdown': lambda: None},
            'rclpy.clock': {'Clock': lambda **kwargs: object(), 'ClockType': types.SimpleNamespace(STEADY_TIME=1)},
            'rclpy.node': {'Node': Node},
            'rclpy.qos': {'QoSProfile': lambda **kwargs: object(),
                          'ReliabilityPolicy': types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2),
                          'qos_profile_sensor_data': object()},
            'rclpy.serialization': {'serialize_message': lambda x: b'x', 'deserialize_message': lambda *x: object()},
            'rosidl_runtime_py.utilities': {'get_message': lambda kind: Dummy},
            'message_filters': {'Subscriber': Subscriber, 'SimpleFilter': Filter,
                                'ApproximateTimeSynchronizer': Sync},
            'std_msgs.msg': {'Bool': lambda **kwargs: object()},
            'geometry_msgs.msg': {'Twist': Dummy}}
        result = {}
        for name, attrs in modules.items():
            module = types.ModuleType(name); module.__dict__.update(attrs); result[name] = module
        return result

    def construct(self, mode, profile):
        registry = {'publishers': [], 'subscriptions': []}
        modules = self.relay_modules(registry)
        link = types.SimpleNamespace(start=lambda: None, close=lambda: None, consume_stop=lambda: False,
                                     connected=False, generation=None, error=None)
        with patch.dict(sys.modules, modules), patch.dict(os.environ, {}, clear=True), \
                patch.object(ros_tunnel, 'Tunnel', return_value=link):
            ros_tunnel.main([mode, '--profile', profile])
        return registry

    def test_relay_node_endpoint_construction_for_both_profiles_and_sides(self):
        estimated_server = self.construct('server', 'estimated_inspect')
        estimated_client = self.construct('client', 'estimated_inspect')
        self.assertFalse({'/odom', '/tf', *inspect.COMMAND_TOPICS}
                         & set(estimated_server['subscriptions'] + estimated_server['publishers']))
        self.assertFalse({'/odom', '/tf', *inspect.COMMAND_TOPICS}
                         & set(estimated_client['subscriptions'] + estimated_client['publishers']))
        reference_server = self.construct('server', 'reference')
        reference_client = self.construct('client', 'reference')
        self.assertTrue({'/odom', '/tf'} <= set(reference_server['subscriptions']))
        self.assertTrue(set(inspect.COMMAND_TOPICS) <= set(reference_server['publishers']))
        self.assertTrue({'/odom', '/tf'} <= set(reference_client['publishers']))
        self.assertTrue(set(inspect.COMMAND_TOPICS) <= set(reference_client['subscriptions']))

    def test_probe_constructs_only_readers_with_compatible_and_latched_qos(self):
        registry = {'subscriptions': []}
        sensor_qos = object()

        class Qos:
            def __init__(self, **kwargs): self.__dict__.update(kwargs)
        class Client:
            def service_is_ready(self): return False
        class Node:
            def create_subscription(self, kind, topic, callback, qos):
                registry['subscriptions'].append((topic, callback, qos)); return object()
            def create_client(self, kind, topic): return Client()
            def get_publishers_info_by_topic(self, topic): return []
            def destroy_node(self): pass
        class Dummy: pass
        modules = {
            'rclpy': {'init': lambda **kwargs: None, 'create_node': lambda name: Node(),
                      'spin_once': lambda *args, **kwargs: None, 'shutdown': lambda: None},
            'lifecycle_msgs.srv': {'GetState': Dummy}, 'nav_msgs.msg': {'Odometry': Dummy},
            'rclpy.qos': {'DurabilityPolicy': types.SimpleNamespace(TRANSIENT_LOCAL='transient'),
                          'ReliabilityPolicy': types.SimpleNamespace(RELIABLE='reliable'),
                          'QoSProfile': Qos, 'qos_profile_sensor_data': sensor_qos},
            'std_msgs.msg': {'String': Dummy}, 'tf2_msgs.msg': {'TFMessage': Dummy}}
        loaded = {}
        for name, attrs in modules.items():
            module = types.ModuleType(name); module.__dict__.update(attrs); loaded[name] = module
        ticks = iter(range(1, 100))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); map_yaml = root/'map.yaml'; output = root/'result.json'
            map_yaml.write_text('image: map.pgm\n')
            with patch.dict(sys.modules, loaded), patch.object(inspect.time, 'monotonic', side_effect=ticks):
                result = inspect.main(['--duration', '1', '--map-id', MAP_ID,
                                       '--map-yaml', str(map_yaml), '--out', str(output)])
        self.assertEqual(result, 3)
        topics = {topic: qos for topic, callback, qos in registry['subscriptions']}
        self.assertEqual(set(topics), set(inspect.INPUT_TOPICS))
        self.assertIs(topics['/odom'], sensor_qos); self.assertIs(topics['/tf'], sensor_qos)
        self.assertEqual(topics['/tf_static'].durability, 'transient')
        self.assertEqual(topics['/tf_static'].reliability, 'reliable')


if __name__ == '__main__':
    unittest.main()
