"""Offline causal controls using production gate/lease/queue, never ROS or devices.

The conservative policy is a diagnostic candidate, not a runtime replacement.
Only exact-token eligibility is measured; downstream transforms/inference are absent.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from host.localization_gate import KINDS, LocalizationGate, LocalizationLease
from host.ros_runtime import AcquisitionQueue

POLICIES = ('wait', 'discard_all_empty', 'conservative')


def disposition(lease, stamp, instance, token, wall, ros_ns):
    """Pure current-row observation after the policy's one scheduled token call.

    A completed, absent stamp cannot subsequently be approved in this instance.
    The label says nothing about a different gate instance or downstream readiness.
    """
    if token is not None:
        return 'approved_exact'
    row = lease.row
    if not row or not instance or row['instance'] != instance:
        return 'unknown_wait'
    fresh = (0 <= wall-lease.received < .5 and 0 <= wall-lease.progress < 1.
             and 0 <= ros_ns-row['clock_ns'] <= 10**9
             and 0 <= ros_ns-row['stamp_ns'] <= 10**9)
    if not fresh or row['state'] not in ('confirming', 'valid'):
        return 'unknown_wait'
    if row['stamp_ns'] < stamp:
        return 'possibly_inflight_wait'
    if any(item['stamp_ns'] == stamp for item in row['approved']):
        return 'unknown_wait'
    if row['state'] == 'confirming' and row['reason'] == 'confirming_healthy_acquisitions' and row['stamp_ns'] == stamp:
        return 'completed_confirmation_no_reapproval_same_instance'
    return 'completed_without_approval_same_instance'


def observation(kind, stamp):
    pose = {'position': dict(x=1., y=2., z=3.), 'orientation': dict(x=0., y=0., z=0., w=1.)}
    cov = [float(i % 7 == 0)*.01 for i in range(36)]
    return {'header': {'stamp': dict(sec=stamp//10**9, nanosec=stamp % 10**9),
                       'frame_id': 'odom' if kind in KINDS[:2] else 'map'},
            'child_frame_id': 'camera_link', 'pose': {'pose': pose, 'covariance': cov},
            'covariance': cov, 'lost': False, 'loop_closure_id': 1, 'proximity_detection_id': 0}


class Trace:
    def __init__(self, name):
        self.name = name
        self.gate = LocalizationGate('offline-map', {1})
        self.consumers = {p: {'lease': LocalizationLease('offline-map'), 'queue': AcquisitionQueue(),
                             'eligible': [], 'dropped': []} for p in POLICIES}
        self.events = []
        self.wall, self.ros = 0., 10**9

    def clock(self, wall, stamp=None):
        self.wall = wall
        if stamp is not None:
            self.ros = stamp

    def record(self, event, **fields):
        self.events.append(dict(event=event, wall=self.wall, ros_ns=self.ros, **fields))

    def acquire(self, stamp, queued=False):
        self.gate.acquisition(stamp, self.wall, self.ros)
        changes = {}
        if queued:
            for policy, c in self.consumers.items():
                row = c['lease'].row
                q = c['queue']
                old_latest = q.latest.payload['stamp_ns'] if q.latest else None
                dropped = q.push(stamp/10**9, {'stamp_ns': stamp, 'instance': row['instance'] if row else None}, self.wall)
                changes[policy] = {'push_dropped': dropped, 'previous_latest': old_latest,
                                   'current': q.current.payload['stamp_ns'],
                                   'latest': q.latest.payload['stamp_ns'] if q.latest else None}
        self.record('acquire', stamp_ns=stamp, queued=queued, queues=changes)

    def complete(self, stamp, kinds=KINDS):
        outputs = []
        for kind in kinds:
            outputs += self.gate.ingest(kind, observation(kind, stamp), self.wall, self.ros)
        self.record('components', stamp_ns=stamp, kinds=list(kinds), outputs=[r['stamp_ns'] for r in outputs],
                    pending=list(self.gate.pending), watermark=self.gate.last_complete)

    def status(self, deliver=True):
        row = self.gate.status(self.wall, self.ros)
        self.record('status_emit', row=row, delivered=deliver)
        if deliver:
            self.deliver(row)
        return row

    def deliver(self, row):
        for c in self.consumers.values():
            c['lease'].update(copy.deepcopy(row), self.wall, self.ros)
        self.record('status_delivery', row=row)

    def poll(self):
        for policy, c in self.consumers.items():
            q, lease = c['queue'], c['lease']
            if not q.current:
                self.record('poll', policy=policy, action='empty')
                continue
            entry = q.current
            stamp = entry.payload['stamp_ns']
            expired = q.expired(self.wall)
            # No extra token/check calls in recording or the classifier.
            token = None if expired else lease.token(stamp, self.wall, self.ros)
            before = copy.deepcopy(lease.__dict__)
            label = disposition(lease, stamp, entry.payload['instance'], token, self.wall, self.ros)
            assert lease.__dict__ == before, 'classifier mutated lease'
            action = 'eligible' if token else 'wait'
            if expired:
                action = 'expired'
            elif not token and (policy == 'discard_all_empty' or (policy == 'conservative' and label.startswith('completed_'))):
                action = 'discard'
            self.record('poll', policy=policy, stamp_ns=stamp, received_at=entry.received_at,
                        deadline=entry.received_at+q.timeout, action=action, disposition=label,
                        token=token, row=copy.deepcopy(lease.row), generation=lease.generation,
                        latest_stamp_ns=q.latest.payload['stamp_ns'] if q.latest else None)
            if action != 'wait':
                c['eligible' if action == 'eligible' else 'dropped'].append(stamp)
                q.pop()

    def warm(self):
        for i in (1, 2, 3):
            stamp = 10**9+i*10**8
            self.clock(i*.1, stamp); self.acquire(stamp); self.complete(stamp); self.status()

    def result(self):
        return {'name': self.name, 'events': self.events,
                'outcomes': {p: {k: c[k] for k in ('eligible', 'dropped')} for p, c in self.consumers.items()}}


def controls():
    results = []
    t = Trace('confirmation_old_blocks_approved_successor')
    t.status()  # bind the known gate instance before receiving images
    for i in (1, 2, 3, 4):
        stamp = 10**9+i*10**8
        t.clock(i*.1, stamp); t.acquire(stamp, queued=True); t.complete(stamp); t.status(); t.poll()
    t.poll()
    t.clock(.61); t.poll(); t.poll()
    results.append(t.result())

    # Identical consumer prefix, two distinct gate-side causes of absent token.
    for name, delayed in [('pending_completes_before_deadline', False), ('approved_status_in_transit', True)]:
        t = Trace(name); t.warm(); t.clock(.4, 1400000000); t.acquire(1400000000, True)
        if delayed:
            t.complete(1400000000); row = t.status(False)
        t.poll()
        t.clock(.55)
        if delayed:
            t.deliver(row)
        else:
            t.complete(1400000000); t.status()
        t.poll(); results.append(t.result())

    t = Trace('newer_components_wait_for_older_pending'); t.warm()
    t.clock(.4, 1400000000); t.acquire(1400000000)
    t.clock(.45, 1500000000); t.acquire(1500000000, True); t.complete(1500000000); t.status(); t.poll()
    t.clock(.55); t.complete(1400000000); t.status(); t.poll(); results.append(t.result())

    t = Trace('restart_reapproves_same_stamp_availability_cost')
    t.status(); t.clock(.1, 1300000000); t.acquire(1300000000, True)
    t.complete(1300000000); t.status(); t.poll()
    t.gate = LocalizationGate('offline-map', {1})
    # Keep ROS time monotonic while the replacement processes earlier stamps.
    for i in (1, 2, 3):
        t.clock(.2+i*.05); stamp = 10**9+i*10**8
        t.acquire(stamp); t.complete(stamp); t.status()
    t.poll(); results.append(t.result())

    for fault in ('epoch_revokes_old_token', 'restart_revokes_old_token'):
        t = Trace(fault); t.warm()
        tokens = {p: c['lease'].token(1300000000, t.wall, t.ros) for p, c in t.consumers.items()}
        assert all(tokens.values())
        t.clock(.35)
        if fault == 'epoch_revokes_old_token':
            t.gate.revoke('lost', 'offline_fault')
        else:
            t.gate = LocalizationGate('offline-map', {1})
        t.status()
        permitted = {p: c['lease'].permits(tokens[p], t.wall, t.ros) for p, c in t.consumers.items()}
        t.record('old_token_validation', tokens=tokens, permitted=permitted)
        assert not any(permitted.values())
        results.append(t.result())

    for fault in ('heartbeat_expired', 'queue_expired', 'revoked', 'restart', 'invalid_status', 'reordered_status', 'confirming_stale'):
        t = Trace(fault)
        if fault == 'confirming_stale':
            t.status(); t.clock(.1, 1100000000); t.acquire(1100000000, True); t.complete(1100000000); t.status()
            t.clock(.61)
            # Direct pure observation: queue already expired but confirming row remains.
            for c in t.consumers.values():
                assert disposition(c['lease'], 1100000000, t.gate.instance, None, t.wall, t.ros) == 'unknown_wait'
        else:
            t.warm(); row = copy.deepcopy(t.consumers['wait']['lease'].row)
            t.clock(.4, 1400000000); t.acquire(1400000000, True)
            if fault == 'heartbeat_expired':
                t.clock(.81)
            elif fault == 'queue_expired':
                t.complete(1400000000); t.clock(.91); t.status()
            elif fault == 'revoked':
                t.gate.revoke('lost', 'offline_fault'); t.status()
            elif fault == 'restart':
                t.gate = LocalizationGate('offline-map', {1}); t.status()
            elif fault == 'invalid_status':
                row['map_id'] = 'wrong-map'; t.deliver(row)
            elif fault == 'reordered_status':
                t.deliver(row)
        t.poll(); results.append(t.result())
    return results


def run_controls():
    # Gate implementation stays real; replace nondeterministic identity only.
    identities = iter('offline-instance-%02d' % i for i in range(100))
    class Identity:
        def __init__(self):
            self.hex = next(identities)
    with patch('host.localization_gate.uuid.uuid4', side_effect=Identity):
        return controls()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    sources = ['host/localization_gate.py', 'host/ros_runtime.py', __file__, 'tests/test_rtabmap_frame_disposition_probe.py']
    manifest = {'command': sys.argv, 'python': sys.version, 'platform': platform.platform(),
                'commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                'git_status': subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True),
                'clock': 'deterministic virtual seconds; no performance measurements',
                'scope': 'exact-token eligibility only; no ROS, transform, inference, geometry or memory writes; does not reproduce prepare/poll/navigation fault semantics',
                'sources': {str(Path(s).relative_to(ROOT) if Path(s).is_absolute() else s): hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources}}
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    results = run_controls()
    (args.output/'traces.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps({r['name']: r['outcomes'] for r in results}, indent=2))


if __name__ == '__main__':
    main()
