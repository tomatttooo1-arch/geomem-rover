"""Finite source-loaded Runtime controls. No ROS graph or real detector is run.

Fixed empty detections exercise actual result callbacks, geometry and Memory.
Candidate changes only the in-memory AST token-empty branch, never production files.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import traceback
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
import numpy as np

from host.localization_gate import KINDS
from host.ros_runtime import PoseGoal, transform_matrix
from tools.rtabmap_runtime_probe_support import (World, load_runtime, gate, healthy,
    transform, Time, TransformException, observation)

POLICIES = ('original', 'candidate')
# Concrete event table is part of the pre-run code, not an adaptive scheduler.
MAIN_EVENTS = [
    {'wall': .35, 'ros_ns': 1300000000, 'event': 'A_fault'},
    {'wall': .4, 'ros_ns': 1400000000, 'event': 'A_target_confirmation_1_and_only_target_RGB'},
    {'wall': .45, 'ros_ns': 1500000000, 'event': 'A_confirmation_2'},
    {'wall': .50, 'ros_ns': 1600000000, 'event': 'same_A_confirmation_3_or_B_reprocess_1200000000'},
    {'wall': .52, 'ros_ns': 1600000000, 'event': 'B_reprocess_1300000000_if_restart'},
    {'wall': .54, 'ros_ns': 1600000000, 'event': 'B_reprocess_target_1400000000_if_restart'},
    {'wall': .6, 'ros_ns': 1700000000, 'event': 'common_healthy_RGB_1700000000'},
    {'wall': .7, 'ros_ns': 1800000000, 'event': 'common_healthy_RGB_1800000000'},
    {'wall': .901, 'ros_ns': 1800000000, 'event': 'original_target_deadline_passed_poll_twice'},
    {'wall': .95, 'ros_ns': 1800000000, 'event': 'action_cancel_receipts_and_result'},
]


class Session:
    current = None

    def __init__(self, root, name, policy):
        self.path = Path(root)/name
        self.path.mkdir(parents=True, exist_ok=False)
        self.name, self.policy = name, policy
        self.w = World(policy)
        self.episode_sequence = None
        self.finished = False
        Session.current = self
        self.n = load_runtime(self.w, self.path/'runtime-diagnostics.json')
        self.gate = gate(self.w, 'fixture-A')
        # Nonidentity two-edge extrinsics, composed by the boundary adapter.
        self.n.on_static_tf(NS(transforms=[transform('base_link', 'camera_link', (.2, -.1, .3), np.pi/2),
                                                   transform('camera_link', 'camera_optical_frame', (.1, .2, -.05), -np.pi/4)]))
        self.n.on_map(NS(info=NS(width=40, height=40, resolution=.5,
                        origin=NS(position=NS(x=-10., y=-10., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))),
                         header=NS(frame_id='map'), data=[0]*(40*40)))
        self.episode_sequence = None

    def status(self, row, delivery='complete'):
        delivered = delivery == 'complete' or row['state'] == 'valid'
        self.w.emit('status_schedule', delivered=delivered, row=row,
                    omission_fault_injection=delivery == 'omit_intermediate')
        if delivered:
            self.w.deliver_status(row)

    def warm(self, navigation=False):
        for i in (1, 2, 3):
            stamp = 10**9+i*10**8
            self.w.clock(i*.1, stamp)
            self.status(healthy(self.w, self.gate, stamp))
        self.w.image(1300000000)
        assert self.w.finish_detection(), 'positive fixture never submitted'
        self.w.poll('warm_result')
        assert self.n.processed == 1 and self.n.memory.seen_frames == {'rgb-1300000000'}
        assert self.n.memory.last_timestamp == 1.3
        if navigation:
            self.w.emit('explicit_ready_calibration')
            assert self.n.ready()
            self.n.navigate(PoseGoal(.5, .2, .1), None)
            assert self.n.actions.request is not None
            self.w.emit('action_accept_delivery')
            self.n.actions.request.set_result(self.w.goal)
            self.w.poll('navigation_armed')
            assert self.w.publisher_values['/fpga/motion_permit'] is True
            assert not self.n.nav['cancelled'] and not self.n.motion.inhibited
        self.w.emit('episode_start', navigation=navigation)
        self.episode_sequence = len(self.w.events)

    def service(self, label):
        # Fixed external completion rule, shared by both policies: one poll, at
        # most one queued inference completion, then one poll to receive it.
        self.w.poll(label)
        if self.w.finish_detection():
            self.w.poll(label+'_result')

    def finish(self, error=None):
        self.w.state('episode_end')
        end_sequence = len(self.w.events)
        episode = [e for e in self.w.events if (self.episode_sequence or 0) < e['sequence'] <= end_sequence]
        pubs = [e for e in episode if e['event'] == 'publish']
        def first(predicate):
            return next(({'sequence': e['sequence'], 'wall': e['wall']} for e in episode if predicate(e)), None)
        events = [e['payload'] for e in pubs if e['topic'] == '/fpga/status']
        summary = dict(name=self.name, policy=self.policy, outcome='failed' if error else 'passed', error=error,
            episode_start_sequence=self.episode_sequence,
            episode_end_sequence=end_sequence, processed=self.n.processed,
            memory_seen=sorted(self.n.memory.seen_frames), memory_last_timestamp=self.n.memory.last_timestamp if np.isfinite(self.n.memory.last_timestamp) else None,
            dropped=self.n.dropped, submitted=[e['frame_id'] for e in self.w.events if e['event'] == 'executor_submit'],
            fault_reasons=[e['reason'] for e in events if e['event'] == 'fault'],
            first_permit_false=first(lambda e: e['event'] == 'publish' and e['topic'] == '/fpga/motion_permit' and e['payload'] is False),
            first_zero_command=first(lambda e: e['event'] == 'publish' and e['topic'] == '/cmd_vel'),
            first_cancel_request=first(lambda e: e['event'] == 'action_cancel_request'),
            permit_timeline=[{'sequence': e['sequence'], 'wall': e['wall'], 'value': e['payload']} for e in pubs if e['topic'] == '/fpga/motion_permit'],
            final_motion_inhibited=self.n.motion.inhibited,
            final_nav_cancelled=self.n.nav['cancelled'] if self.n.nav else None,
            scope='actual Runtime methods with fixed empty detector and deterministic I/O; not ROS, physical stop or real detection')
        self.w.emit('cleanup_start')
        try:
            self.n.close()
        except Exception:
            summary['cleanup_error'] = traceback.format_exc()
            summary['outcome'] = 'failed'
        finally:
            try:
                self.n.diagnostics.flush()
            finally:
                with (self.path/'events.json').open('x') as stream:
                    json.dump(self.w.events, stream, indent=2, allow_nan=False)
                with (self.path/'summary.json').open('x') as stream:
                    json.dump(summary, stream, indent=2, allow_nan=False)
                self.finished = True
        return summary

    def action_receipts(self):
        handle = self.w.goal
        if not handle:
            return
        for future in handle.cancels:
            if not future.done():
                self.w.emit('action_cancel_receipt')
                future.set_result(NS(goals_canceling=['fixture-goal']))
        if not handle.result_future.done():
            self.w.emit('action_result_delivery', status=5 if self.n.nav and self.n.nav['cancelled'] else 4)
            handle.result_future.set_result(NS(status=5 if self.n.nav and self.n.nav['cancelled'] else 4))


def main_cell(root, policy, evolution, navigation, delivery):
    name = '__'.join((policy, evolution, 'nav' if navigation else 'idle', delivery))
    s = Session(root, name, policy); s.warm(navigation)
    w = s.w
    w.emit('fixed_event_table', table=MAIN_EVENTS)
    w.clock(.35, 1300000000)
    s.gate.revoke('lost', 'fixed_localization_fault')
    s.status(s.gate.status(w.wall, w.ros_ns), delivery)
    # No intermediary poll when a status is deliberately omitted: it must not
    # secretly query an unreceived gate state through an oracle.
    w.clock(.4, 1400000000)
    s.status(healthy(w, s.gate, 1400000000), delivery)
    w.image(1400000000); s.service('target_confirmation')
    w.clock(.45, 1500000000)
    s.status(healthy(w, s.gate, 1500000000), delivery); s.service('second_confirmation')
    w.clock(.50, 1600000000)
    if evolution == 'same_instance':
        s.status(healthy(w, s.gate, 1600000000), delivery)
        s.service('first_valid_after_fault')
        w.clock(.52); s.service('fixed_poll_52')
        w.clock(.54); s.service('fixed_poll_54')
    else:
        assert evolution == 'restart'
        s.gate = gate(w, 'fixture-B')
        for wall, stamp in ((.50, 1200000000), (.52, 1300000000), (.54, 1400000000)):
            w.clock(wall)
            s.status(healthy(w, s.gate, stamp), delivery)
            s.service('replacement_progress')
    for wall, stamp in ((.6, 1700000000), (.7, 1800000000)):
        w.clock(wall, stamp)
        s.status(healthy(w, s.gate, stamp), delivery)
        w.image(stamp); s.service('common_successor')
    w.clock(.901, 1800000000)
    s.service('target_original_deadline'); s.service('promoted_successor')
    w.clock(.95); s.action_receipts()
    return s.finish()


def calibration(root, name, policy='original'):
    s = Session(root, policy+'__'+name, policy); w, n = s.w, s.n
    s.warm(navigation=name == 'navigation_fault')
    if name == 'navigation_fault':
        w.clock(.4, 1400000000)
        s.gate.revoke('lost', 'calibration_revoke')
        s.status(s.gate.status(w.wall, w.ros_ns)); w.poll('fault_after_armed')
        assert n.nav['cancelled'] and n.motion.inhibited
        assert w.publisher_values['/fpga/motion_permit'] is False
        assert w.goal.cancels
        s.action_receipts()
    elif name == 'static_transform_calibration':
        forward = transform_matrix(n.buffer.lookup_transform('base_link', 'camera_optical_frame', Time(w.ros_ns)).transform)
        reverse = transform_matrix(n.buffer.lookup_transform('camera_optical_frame', 'base_link', Time(w.ros_ns)).transform)
        expected = np.array([[2**-.5, -2**-.5, 0., 0.], [2**-.5, 2**-.5, 0., 0.], [0., 0., 1., .25], [0., 0., 0., 1.]])
        np.testing.assert_allclose(forward, expected, atol=1e-12)
        np.testing.assert_allclose(reverse @ forward, np.eye(4), atol=1e-12)
        assert not np.allclose(forward, np.eye(4))
        for target, source, stamp in [('missing', 'camera_optical_frame', Time(w.ros_ns)), ('base_link', 'camera_optical_frame', Time(0))]:
            try:
                n.buffer.lookup_transform(target, source, stamp)
            except TransformException:
                w.emit('tf_rejection_calibration', target=target, source=source, stamp_ns=stamp.nanoseconds)
            else:
                raise AssertionError('missing/zero TF lookup accepted')
        w.emit('tf_calibration', forward=forward, reverse=reverse)
    else:
        w.clock(.4, 1400000000)
        s.status(healthy(w, s.gate, 1400000000))
        if name == 'missing_static_tf':
            del n.buffer.edges[('camera_link', 'camera_optical_frame')]
            w.emit('remove_static_edge', edge=['camera_link', 'camera_optical_frame'])
        w.image(1400000000, depth_offset_ns=30000000 if name == 'depth_timestamp_invalid' else 0)
        if name in ('normal_result', 'depth_timestamp_invalid'):
            if name == 'normal_result':
                assert n.inference is not None
                np.testing.assert_allclose(w.pool.jobs[-1]['frame'].T_world_camera[:3, 3], [1.1, 2.2, 2.95])
                assert w.finish_detection(); w.poll('result')
                assert 'rgb-1400000000' in n.memory.seen_frames and n.memory.last_timestamp == 1.4
            else:
                assert n.inference is None and n.last_submitted == 1.3
                assert 'rgb-1400000000' not in n.memory.seen_frames
                errors = [r for r in n.diagnostics.rows if r['event'] == 'prepare_error']
                assert errors and 'RGB/depth timestamp mismatch' in errors[-1]['detail']
        elif name in ('late_future_epoch_revoke', 'late_future_instance_revoke'):
            assert n.inference is not None
            w.clock(.45)
            if name == 'late_future_epoch_revoke':
                s.gate.revoke('lost', 'calibration_revoke')
            else:
                s.gate = gate(w, 'fixture-B')
            s.status(s.gate.status(w.wall, w.ros_ns))
            w.clock(.5); assert w.finish_detection(); w.poll('late_result')
            assert n.memory.seen_frames == {'rgb-1300000000'} and n.memory.last_timestamp == 1.3
            assert any(e['event'] == 'publish' and e['topic'] == '/fpga/status'
                       and e['payload'].get('reason') == 'localization_approval_revoked_before_memory_write' for e in w.events)
        elif name == 'missing_static_tf':
            assert n.localization.row and n.localization.row['state'] == 'valid'
            assert any(r['stamp_ns'] == 1400000000 for r in n.localization.row['approved'])
            waits = [r for r in n.diagnostics.rows if r['event'] == 'prepare_wait']
            assert waits and waits[-1]['branch'] == 'base_camera_static_tf'
            assert n.acquisitions.current is not None and n.inference is None
            w.clock(.901); w.poll('missing_tf_original_deadline')
            assert n.acquisitions.current is None and n.last_submitted == 1.3
            assert any(r['event'] == 'prepare_expiry' and r['expired'] for r in n.diagnostics.rows)
        else:
            raise ValueError(name)
    return s.finish()


def inflight(root, policy, cause):
    s = Session(root, policy+'__'+cause, policy); s.warm()
    w, n = s.w, s.n
    w.clock(.4, 1400000000)
    s.gate.acquisition(1400000000, w.wall, w.ros_ns)
    if cause == 'status_in_transit':
        for kind in KINDS:
            s.gate.ingest(kind, observation(kind, 1400000000), w.wall, w.ros_ns)
        row = s.gate.status(w.wall, w.ros_ns)
        w.emit('status_withheld_in_transit', row=row)
    w.image(1400000000); w.poll('inflight_prefix')
    assert n.inference is None and n.acquisitions.current is not None
    w.clock(.55)
    if cause == 'gate_pending':
        for kind in KINDS:
            s.gate.ingest(kind, observation(kind, 1400000000), w.wall, w.ros_ns)
        row = s.gate.status(w.wall, w.ros_ns)
    s.status(row); s.service('inflight_released')
    assert n.memory.seen_frames == {'rgb-1300000000', 'rgb-1400000000'}
    return s.finish()


def deadline_control(root, policy):
    s = Session(root, policy+'__late_approval_original_deadline', policy); s.warm()
    w, n = s.w, s.n
    w.clock(.4, 1400000000); w.image(1400000000)
    # Gate acquires later than consumer, allowing its legitimate completion after
    # the unchanged consumer deadline. This is delayed acquisition delivery.
    w.clock(.6); s.gate.acquisition(1400000000, w.wall, w.ros_ns)
    w.clock(.7, 1500000000); w.image(1500000000)
    w.clock(.901)
    for kind in KINDS:
        s.gate.ingest(kind, observation(kind, 1400000000), w.wall, w.ros_ns)
    s.status(s.gate.status(w.wall, w.ros_ns))
    assert n.localization.row and n.localization.row['state'] == 'valid'
    assert any(r['stamp_ns'] == 1400000000 for r in n.localization.row['approved'])
    w.emit('late_exact_approval_present_before_expiry', stamp_ns=1400000000,
           consumer_received=n.acquisitions.current.received_at,
           original_deadline=n.acquisitions.current.received_at+n.acquisitions.timeout,
           approved_stamps=[r['stamp_ns'] for r in n.localization.row['approved']])
    w.poll('late_approval_deadline')
    assert n.last_submitted == 1.3 and n.acquisitions.current.stamp == 1.5
    assert n.acquisitions.current.received_at == .7
    w.clock(1.201); w.poll('successor_original_deadline')
    assert n.acquisitions.current is None
    return s.finish()


def run_case(function, *args):
    Session.current = None
    try:
        return function(*args)
    except Exception:
        failure = traceback.format_exc()
        session = Session.current
        if session and not session.finished:
            if hasattr(session, 'n'):
                session.finish(error=failure)
            else:
                # Initialization failure still preserves its observed prefix.
                with (session.path/'events.json').open('x') as stream:
                    json.dump(session.w.events, stream, indent=2, allow_nan=False)
                with (session.path/'failure.json').open('x') as stream:
                    json.dump({'outcome': 'failed', 'error': failure}, stream, indent=2)
                node = session.w.node
                if node and getattr(node, 'diagnostics', None):
                    node.diagnostics.flush()
        raise


def source_manifest():
    import cv2
    sources = ['host/ros_runtime.py', 'host/localization_gate.py', 'host/runtime_diagnostics.py',
               'perception/fpga_replay/core.py', 'host/task_control.py',
               'tools/rtabmap_frame_disposition_probe.py', 'tools/rtabmap_runtime_probe_support.py',
               'tools/rtabmap_runtime_behavior_probe.py', 'tests/test_rtabmap_runtime_behavior_probe.py']
    return {'command': sys.argv, 'executable': sys.executable, 'python': sys.version,
            'platform': platform.platform(), 'numpy': np.__version__, 'opencv': cv2.__version__,
            'commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
            'git_status': subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True),
            'sources': {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources},
            'main_scope_excluded': ['argument parsing', 'ROS init/spin/shutdown', 'real DDS/TF scheduling', 'real inference', 'physical robot'],
            'fixed_main_event_table': MAIN_EVENTS}


def run(root, phase, calibration_evidence=None):
    root = Path(root); root.mkdir(parents=True, exist_ok=False)
    manifest = source_manifest()
    manifest['phase'] = phase
    with (root/'manifest.json').open('x') as stream:
        json.dump(manifest, stream, indent=2)
    if phase == 'calibration':
        calibrations = [run_case(calibration, root/'calibration', name, policy) for policy in POLICIES
                        for name in ('normal_result', 'depth_timestamp_invalid', 'late_future_epoch_revoke',
                                     'late_future_instance_revoke', 'navigation_fault', 'static_transform_calibration', 'missing_static_tf')]
        cells, extras = [], []
    else:
        assert phase == 'experiment' and calibration_evidence is not None
        calibration_evidence = Path(calibration_evidence).resolve()
        prior = json.loads((calibration_evidence/'manifest.json').read_text())
        report = json.loads((calibration_evidence/'summary.json').read_text())
        assert prior['phase'] == 'calibration' and prior['sources'] == manifest['sources'], 'calibration code differs'
        calibrations = report['calibration']
        assert len(calibrations) == 14 and all(r['outcome'] == 'passed' for r in calibrations), 'incomplete calibration'
        cells = [run_case(main_cell, root/'matrix', policy, evolution, navigation, delivery)
                 for policy in POLICIES for evolution in ('same_instance', 'restart')
                 for navigation in (False, True) for delivery in ('complete', 'omit_intermediate')]
        extras = [run_case(inflight, root/'controls', policy, cause) for policy in POLICIES
                  for cause in ('gate_pending', 'status_in_transit')]
        extras += [run_case(deadline_control, root/'controls', policy) for policy in POLICIES]
    result = {'calibration': calibrations, 'matrix': cells, 'controls': extras,
              'calibration_evidence': str(Path(calibration_evidence).resolve()) if calibration_evidence else str(root.resolve()),
              'fixed_main_event_table': MAIN_EVENTS,
              'definitions': {'processed': 'actual Runtime.poll accepted fixed empty detection and ran actual Memory.update_memory',
                              'stop': 'published inhibit/zero command and requested action cancel; no physical robot',
                              'omission': 'explicit fault injection withholding all intermediate non-valid localization statuses',
                              'dropped': 'Runtime counter, including queue replacement and preparation/revocation rejection'}}
    (root/'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--phase', choices=('calibration', 'experiment'), required=True)
    parser.add_argument('--calibration-evidence', type=Path)
    args = parser.parse_args()
    assert not args.output.exists(), 'exclusive new output required'
    result = run(args.output, args.phase, args.calibration_evidence)
    print(json.dumps({k: len(result[k]) for k in ('calibration', 'matrix', 'controls')}, indent=2))


if __name__ == '__main__':
    main()
