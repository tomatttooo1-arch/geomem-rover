"""External-I/O fixtures for the source-loaded Runtime offline probe.

No ROS imports, sockets, threads, device operations, or replacement business methods.
"""
import ast
import builtins
import copy
from concurrent.futures import Future
import hashlib
import json
import math
import symtable
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

from host import ros_runtime as production
from host.localization_gate import LocalizationGate
from perception.fpga_replay.core import Frame, Memory, process_frame, compute_geometry
from tools.rtabmap_frame_disposition_probe import disposition, observation

ROOT = Path(__file__).resolve().parents[1]

# This probe needs Python 3.9+: it reads and rebuilds the Runtime class through
# ast.unparse, which 3.8 does not have. It is a diagnostic probe, not a product path,
# and D-028 decided not to backport it. Say so by name when it is reached on 3.8,
# instead of letting the missing attribute surface as an unexplained AttributeError.
REQUIRED_PYTHON = (3, 9)


def unparse(node):
    if sys.version_info < REQUIRED_PYTHON:
        raise RuntimeError(
            'tools/rtabmap_runtime_probe_support.py requires Python {}.{}+ (ast.unparse); '
            'running on {}.{}.{} -- not backported, see D-028'.format(
                *REQUIRED_PYTHON, *sys.version_info[:3]))
    return ast.unparse(node)


def scalar(value):
    if hasattr(value, '__dict__'):
        return {k: scalar(v) for k, v in vars(value).items()}
    if isinstance(value, dict):
        return {str(k): scalar(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [scalar(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


class Time:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = nanoseconds

    @classmethod
    def from_msg(cls, msg):
        return cls(msg.sec*10**9+msg.nanosec)

    def to_msg(self):
        return NS(sec=self.nanoseconds//10**9, nanosec=self.nanoseconds % 10**9)


class TransformException(Exception):
    pass


def transform(parent, child, xyz=(0., 0., 0.), yaw=0.):
    return NS(header=NS(frame_id=parent), child_frame_id=child,
              transform=NS(translation=NS(**dict(zip(('x', 'y', 'z'), xyz))),
                           rotation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2))))


def matrix_transform(parent, child, matrix):
    # The fixture deliberately supports only its registered yaw rotations.
    if not np.allclose(matrix[:3, 2], [0., 0., 1.]) or not np.allclose(matrix[2, :3], [0., 0., 1.]):
        raise TransformException('fixture supports registered yaw rotations only')
    return transform(parent, child, matrix[:3, 3], math.atan2(matrix[1, 0], matrix[0, 0]))


class World:
    def __init__(self, policy):
        self.policy = policy
        self.wall, self.ros_ns = 0., 10**9
        self.events = []
        self.identities = {}
        self.instance_sidecar = {}
        self.node = None
        self.goal = None
        self.pool = None
        self.publisher_values = {}
        self.message_filters = []
        self.action_requests = []
        self.waypoints = []

    def emit(self, event, **fields):
        row = dict(sequence=len(self.events)+1, wall=self.wall, ros_ns=self.ros_ns,
                   event=event, **scalar(fields))
        self.events.append(row)

    def clock(self, wall, ros_ns=None):
        assert wall >= self.wall
        if ros_ns is not None:
            assert ros_ns >= self.ros_ns
            self.ros_ns = ros_ns
        self.wall = wall
        self.emit('clock')

    def now(self):
        return Time(self.ros_ns)

    def monotonic(self):
        return self.wall

    def monotonic_ns(self):
        return round(self.wall*10**9)

    def candidate_drop(self, lease, stamp, token, wall, ros_ns):
        # Identical original token result and original call arguments; no added check.
        before = copy.deepcopy(lease.__dict__)
        label = disposition(lease, stamp, self.instance_sidecar.get(stamp), token, wall, ros_ns)
        assert before == lease.__dict__
        self.emit('candidate_classification', stamp_ns=stamp, label=label,
                  captured_instance=self.instance_sidecar.get(stamp),
                  current_instance=lease.row['instance'] if lease.row else None)
        return label.startswith('completed_')

    def state(self, label):
        n = self.node
        nav = n.nav
        self.emit('state', label=label, processed=n.processed, dropped=n.dropped,
                  submitted=n.last_submitted if np.isfinite(n.last_submitted) else None,
                  current=n.acquisitions.current.stamp if n.acquisitions.current else None,
                  current_received=n.acquisitions.current.received_at if n.acquisitions.current else None,
                  latest=n.acquisitions.latest.stamp if n.acquisitions.latest else None,
                  pending=n.pending.frame_id if n.pending else None,
                  inference=n.inference is not None, memory_seen=sorted(n.memory.seen_frames),
                  nav_present=nav is not None, nav_cancelled=nav['cancelled'] if nav else None,
                  handle_accepted=bool(nav and nav.get('handle') and nav['handle'].accepted),
                  motion_inhibited=n.motion.inhibited, inference_healthy=n.inference_healthy,
                  permit=self.publisher_values.get('/fpga/motion_permit'),
                  task_state=n.task.state if n.task else None)

    def finish_detection(self):
        pending = [r for r in self.pool.jobs if not r['future'].done()]
        if not pending:
            self.emit('detector_completion_absent')
            return False
        job = pending[0]
        self.emit('detector_complete', frame_id=job['frame'].frame_id)
        try:
            job['future'].set_result(job['function'](job['frame']))
        except Exception as exc:
            job['future'].set_exception(exc)
        return True

    def image(self, stamp, depth_offset_ns=0):
        n = self.node
        row = n.localization.row
        self.instance_sidecar.setdefault(stamp, row['instance'] if row else None)
        rgb, depth, info = images(stamp, depth_offset_ns)
        n.on_info(info)
        self.emit('rgb_delivery', stamp_ns=stamp, captured_instance=self.instance_sidecar[stamp])
        n.on_images(rgb, depth)
        self.state('after_images')

    def deliver_status(self, row):
        self.emit('localization_delivery', row=row)
        self.node.on_localization(NS(data=json.dumps(row)))
        self.state('after_localization')

    def poll(self, label='poll'):
        self.emit('poll_begin', label=label)
        self.node.poll()
        self.state(label)


class Publisher:
    def __init__(self, world, topic):
        self.world, self.topic = world, topic

    def publish(self, message):
        payload = scalar(message)
        if self.topic == '/fpga/status':
            payload = json.loads(message.data)
        elif self.topic == '/fpga/motion_permit':
            payload = message.data
        self.world.publisher_values[self.topic] = payload
        self.world.emit('publish', topic=self.topic, payload=payload)


def bindings(world):
    class Node:
        def __init__(self, name):
            self.name = name
            self.parameters = {'use_sim_time': False}
            world.node = self

        def get_parameter(self, name):
            return NS(value=self.parameters[name])

        def set_parameters(self, params):
            for p in params:
                self.parameters[p.name] = p.value
            return [NS(successful=True) for _ in params]

        def create_publisher(self, _kind, topic, _qos):
            return Publisher(world, topic)

        def create_subscription(self, kind, topic, callback, qos):
            return NS(topic=topic, callback=callback)

        def create_timer(self, period, callback, clock=None):
            return NS(period=period, callback=callback, clock=clock)

        def get_clock(self):
            return world

        def get_logger(self):
            return NS(warning=lambda message: world.emit('warning', message=message))

    class Parameter:
        Type = NS(BOOL=bool)
        def __init__(self, name, kind, value):
            self.name, self.value = name, value

    class Buffer:
        def __init__(self):
            self.edges = {}

        def set_transform_static(self, value, authority):
            self.edges[(value.header.frame_id, value.child_frame_id)] = copy.deepcopy(value)

        def lookup_transform(self, target, source, stamp):
            world.emit('tf_lookup', target=target, source=source, stamp_ns=stamp.nanoseconds)
            if stamp.nanoseconds <= 0:
                raise TransformException('zero/latest lookup forbidden')
            graph = {}
            for (parent, child), edge in self.edges.items():
                matrix = production.transform_matrix(edge.transform)
                graph.setdefault(parent, []).append((child, matrix))
                graph.setdefault(child, []).append((parent, np.linalg.inv(matrix)))
            todo, seen = [(target, np.eye(4))], set()
            while todo:
                frame, accumulated = todo.pop(0)
                if frame == source:
                    return matrix_transform(target, source, accumulated)
                if frame in seen:
                    continue
                seen.add(frame)
                todo.extend((other, accumulated @ matrix) for other, matrix in graph.get(frame, []))
            raise TransformException('exact registered static transform absent')

    class Executor:
        def __init__(self, max_workers):
            assert max_workers == 1
            self.jobs = []
            world.pool = self

        def submit(self, function, frame):
            future = Future()
            self.jobs.append(dict(function=function, frame=frame, future=future))
            world.emit('executor_submit', frame_id=frame.frame_id, token=frame.localization_token)
            return future

        def shutdown(self, wait=True):
            world.emit('executor_shutdown', unresolved=sum(not j['future'].done() for j in self.jobs))

    class Client:
        def __init__(self, socket, timeout, expected_device, expected_detector=None):
            self.last_metadata = {'backend': 'fixed_offline_fixture', 'device': 'none'}

        def detect(self, frame):
            world.emit('fixture_detect', frame_id=frame.frame_id)
            return []

    class GoalHandle:
        accepted = True
        def __init__(self, request_id):
            self.request_id = request_id
            self.result_future = Future()
            self.cancels = []
            self.cancel_exception = None

        def get_result_async(self):
            return self.result_future

        def cancel_goal_async(self):
            if self.cancel_exception:
                world.emit('action_cancel_exception', request_id=self.request_id)
                raise self.cancel_exception
            result = Future()
            self.cancels.append(result)
            world.emit('action_cancel_request', request_id=self.request_id)
            return result

    class ActionClient:
        def __init__(self, *args):
            self.request = None
            self.send_exception = None

        def server_is_ready(self):
            return True

        def send_goal_async(self, message):
            if self.send_exception:
                world.emit('action_send_exception')
                raise self.send_exception
            self.request = Future()
            world.goal = GoalHandle(len(world.action_requests)+1)
            world.action_requests.append(dict(future=self.request, handle=world.goal))
            world.emit('action_send', request_id=world.goal.request_id, message=message)
            return self.request

    class Subscriber:
        def __init__(self, *args, **kwargs):
            pass

    class Synchronizer:
        def __init__(self, *args):
            pass
        def registerCallback(self, callback):
            world.message_filters.append(callback)

    class Twist:
        def __init__(self):
            self.linear, self.angular = NS(x=0., y=0., z=0.), NS(x=0., y=0., z=0.)

    def goal_message():
        return NS(pose=NS(header=NS(frame_id='', stamp=None), pose=NS(position=NS(x=0., y=0., z=0.),
                     orientation=NS(x=0., y=0., z=0., w=1.))))

    return dict(Node=Node, Parameter=Parameter, Buffer=Buffer, ThreadPoolExecutor=Executor,
                InferenceClient=Client, ActionClient=ActionClient, Time=Time, TransformException=TransformException,
                Subscriber=Subscriber, ApproximateTimeSynchronizer=Synchronizer, Twist=Twist,
                Bool=lambda data=False: NS(data=data), String=lambda data='': NS(data=data),
                Clock=lambda **kw: NS(**kw), ClockType=NS(STEADY_TIME=1),
                QoSProfile=lambda **kw: NS(**kw), DurabilityPolicy=NS(TRANSIENT_LOCAL=1),
                ReliabilityPolicy=NS(RELIABLE=1), qos_profile_sensor_data=1,
                TFMessage=NS, CameraInfo=NS, Image=NS, OccupancyGrid=NS, Empty=NS,
                NavigateToPose=NS(Goal=goal_message),
                GoalStatus=NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6),
                Frame=Frame, Memory=Memory, process_frame=process_frame, compute_geometry=compute_geometry,
                time=world, _candidate_drop=world.candidate_drop)


def load_runtime(world, diagnostic_path):
    path = ROOT/'host/ros_runtime.py'
    source = path.read_text()
    tree = ast.parse(source, filename=str(path))
    main = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main']
    assert len(main) == 1
    classes = [n for n in main[0].body if isinstance(n, ast.ClassDef) and n.name == 'Runtime']
    assert len(classes) == 1
    original = classes[0]
    klass = copy.deepcopy(original)
    changes = 0
    if world.policy == 'candidate':
        method = next(n for n in klass.body if isinstance(n, ast.FunctionDef) and n.name == 'prepare_images')
        for node in ast.walk(method):
            if (isinstance(node, ast.If) and unparse(node.test) == 'token is None'
                    and len(node.body) == 1 and isinstance(node.body[0], ast.Raise)):
                injected = ast.parse('if _candidate_drop(self.localization, stamp.nanoseconds, token, wall, ros_ns):\n    self.dropped += 1\n    return').body[0]
                node.body.insert(0, injected)
                changes += 1
        assert changes == 1, 'source token-empty branch changed; refuse approximate patch'
    else:
        assert world.policy == 'original'
    # All unmodified methods must remain exactly the source AST; candidate changes one method only.
    for a, b in zip(original.body, klass.body):
        if not (world.policy == 'candidate' and isinstance(a, ast.FunctionDef) and a.name == 'prepare_images'):
            assert ast.dump(a) == ast.dump(b)
    if world.policy == 'candidate':
        restored = copy.deepcopy(klass)
        method = next(n for n in restored.body if isinstance(n, ast.FunctionDef) and n.name == 'prepare_images')
        reversed_changes = 0
        for node in ast.walk(method):
            if (isinstance(node, ast.If) and unparse(node.test) == 'token is None'
                    and len(node.body) == 2 and isinstance(node.body[0], ast.If)
                    and isinstance(node.body[0].test, ast.Call)
                    and isinstance(node.body[0].test.func, ast.Name)
                    and node.body[0].test.func.id == '_candidate_drop'):
                node.body.pop(0)
                reversed_changes += 1
        assert reversed_changes == 1 and ast.dump(restored) == ast.dump(original), 'candidate touched other source logic'
    namespace = vars(production).copy()
    namespace.update(bindings(world))
    namespace['args'] = NS(pose_source='estimated', sensor_source='simulation', inference_device='cpu',
                          diagnostics=Path(diagnostic_path), frames_dir=None, events=None,
                          localization_map_id='a'*64, localization_tracked_frame='camera_link',
                          camera_frame='camera_optical_frame', base_frame='base_link', world_frame='map',
                          uint16_depth_scale=.001, socket='offline-no-socket')
    namespace['waypoints'] = world.waypoints
    def forbidden_reference_listener(*args, **kwargs):
        raise RuntimeError('offline fixture only supports the estimated-pose branch')
    namespace['TransformListener'] = forbidden_reference_listener
    symbols = symtable.symtable(unparse(klass), str(path), 'exec')
    required = set()
    def visit(table):
        required.update(s.get_name() for s in table.get_symbols() if s.is_global() and s.is_referenced())
        for child in table.get_children():
            visit(child)
    visit(symbols)
    missing = required-set(namespace)-set(vars(builtins))
    assert not missing, 'unprovided Runtime global dependencies: '+str(sorted(missing))
    module = ast.fix_missing_locations(ast.Module(body=[klass], type_ignores=[]))
    exec(compile(module, str(path), 'exec'), namespace)
    world.emit('loader', production_sha256=hashlib.sha256(source.encode()).hexdigest(),
               source_class_ast_sha256=hashlib.sha256(ast.dump(original).encode()).hexdigest(),
               loaded_class_ast_sha256=hashlib.sha256(ast.dump(klass).encode()).hexdigest(),
               candidate_branch_insertions=changes, required_globals=sorted(required),
               explicit_main_bindings=sorted(bindings(world))+['args', 'waypoints', 'TransformListener'],
               preserved_methods=[n.name for n in original.body if isinstance(n, ast.FunctionDef)
                                  and not (world.policy == 'candidate' and n.name == 'prepare_images')])
    node = namespace['Runtime']()
    assert isinstance(node.memory, Memory)
    return node


def images(stamp, depth_offset_ns=0):
    header = NS(stamp=Time(stamp).to_msg(), frame_id='camera_optical_frame')
    rgb = np.zeros((16, 16, 3), dtype=np.uint8); rgb[:, :, 0] = 220
    depth = np.ones((16, 16), dtype=np.float32)
    color = NS(header=header, encoding='rgb8', is_bigendian=False, width=16, height=16, step=48, data=rgb.tobytes())
    depth_msg = NS(header=NS(stamp=Time(stamp+depth_offset_ns).to_msg(), frame_id='camera_optical_frame'),
                   encoding='32FC1', is_bigendian=False, width=16, height=16, step=64, data=depth.tobytes())
    info = NS(header=header, width=16, height=16, k=[20., 0., 8., 0., 20., 8., 0., 0., 1.], d=[0.]*5)
    return color, depth_msg, info


def gate(world, identity):
    result = LocalizationGate('a'*64, {1})
    # Stable fixture identity only; gate state transitions and approvals remain production code.
    result.instance = identity
    world.emit('gate_created', instance=identity, identity_override='deterministic_fixture_only')
    return result


def healthy(world, current_gate, stamp):
    current_gate.acquisition(stamp, world.wall, world.ros_ns)
    outputs = []
    from host.localization_gate import KINDS
    for kind in KINDS:
        outputs += current_gate.ingest(kind, observation(kind, stamp), world.wall, world.ros_ns)
    world.emit('gate_complete', instance=current_gate.instance, stamp_ns=stamp,
               watermark=current_gate.last_complete, approved=[r['stamp_ns'] for r in outputs])
    return current_gate.status(world.wall, world.ros_ns)
