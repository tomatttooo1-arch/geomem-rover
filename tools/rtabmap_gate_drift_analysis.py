"""Offline, deterministic drift analysis for LocalizationGate rehearsal recordings.

No ROS, no docker, no reruns: the only inputs are a run's ``observations.jsonl``
(replay-side message content) and ``gate-events.jsonl`` (gate-side event log).

Three products per invocation:

1. ``per-frame`` table: one row per published replay stamp per run with the
   gate-relevant content fields, the acquisition->last-message wall latency and
   exactly one mechanically attributed cause for non-approval.
2. ``drift-summary.json``: per-run cause counts plus, for every pair of runs,
   the symmetric difference of the approved sets partitioned by cause.
3. ``idealized-replay.json``: the same recordings replayed through the real
   ``host.localization_gate.LocalizationGate`` with wall time replaced by an
   idealized clock, which removes wall-clock starvation and leaves only the
   content and ordering semantics.

The faithful replay ("known answer") is a hard prerequisite for reading (3):
if the replay of a run does not reproduce that run's recorded approved list
exactly, the run is reported as ``known_answer: FAIL`` and its idealized result
must not be used for attribution.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import sqlite3
import sys

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from host.localization_gate import KINDS, LocalizationGate, covariance_valid  # noqa: E402

PENDING_TIMEOUT_S = .5
STREAM_TIMEOUT_S = 1.
WALL_REASONS = ('incomplete_acquisition_timeout', 'localization_stream_timeout')
CONTENT_SUFFIXES = ('invalid_covariance', 'invalid_pose', 'no_prior_map_match',
                    'odometry_lost', 'wrong_frame', 'wrong_child_frame')
# A revoke fired inside tick() can be overwritten in the same call by a healthy
# completion; the surviving reason is then a healthy one. Only tick() revokes
# (the two wall-clock timeouts) can precede a healthy completion in one call,
# because a content revoke marks the acquisition bad and blocks completion.
HEALTHY_REASONS = ('three_healthy_acquisitions', 'confirming_healthy_acquisitions')
CAUSES = ('approved', 'content-bad', 'wall-timeout-direct', 'wall-timeout-cascade',
          'streak-not-yet-3', 'missing-acquisition', 'other')


def content_class(reason):
    """WALL / CONTENT / INHERIT / OTHER for one revoke reason."""
    if reason in WALL_REASONS:
        return 'WALL'
    if reason in HEALTHY_REASONS:
        return 'WALL'  # hidden tick() revoke, see HEALTHY_REASONS
    if reason in ('unhealthy_acquisition', 'newer_acquisition_unhealthy'):
        return 'CONTENT'
    if ':' in reason and reason.split(':', 1)[1] in CONTENT_SUFFIXES:
        return 'CONTENT'
    if reason in ('unexpected_or_retired_stamp', 'duplicate_message', 'malformed_message',
                  'pending_overflow', 'acquisition_stamp_invalid_or_duplicate', 'clock_rollback'):
        return 'INHERIT'
    return 'OTHER'


class Run:
    """One recording. ``from_dir`` reads the files; tests build the fields directly."""

    def __init__(self, label, events, content, published, feedback=None, slam_feedback=None, path=None):
        self.label, self.events, self.content = label, events, content
        self.published = published                      # {stamp_ns: index}
        self.feedback = feedback or {}
        self.slam_feedback = slam_feedback or {}
        self.path = path

    @classmethod
    def from_dir(cls, label, path):
        path = Path(path)
        content, published, feedback, slam_feedback = {}, {}, {}, {}
        with (path/'observations.jsonl').open() as handle:
            for line in handle:
                row = json.loads(line)
                event = row['event']
                if event in KINDS:
                    content[(event, row['stamp_ns'])] = row
                elif event == 'published':
                    published[row['stamp_ns']] = row['index']
                elif event == 'frame_feedback':
                    feedback[row['stamp_ns']] = row['received']
                elif event == 'frame_slam_feedback':
                    slam_feedback[row['stamp_ns']] = row['received']
        with (path/'gate-events.jsonl').open() as handle:
            events = [json.loads(line) for line in handle]
        return cls(label, events, content, published, feedback, slam_feedback, path)

    def map_id(self):
        return self.events[0]['map_id'] if self.events else None

    def recorded_approved(self):
        return [item['stamp_ns'] for row in self.events for item in row.get('approved', ())]

    def message_reason(self, stamp, prior_ids):
        """The gate's own content verdict for this stamp's four recorded messages."""
        probe = LocalizationGate(self.map_id() or 'x'*64, prior_ids, 'camera_link', 'map', 'odom')
        for kind in KINDS:
            row = self.content.get((kind, stamp))
            if row is None:
                return kind + ':absent'
            reason = probe.message_reason(kind, row)
            if reason:
                return kind + ':' + reason
        return None


def prior_ids_from_db(path):
    uri = 'file:' + os.fspath(Path(path).resolve()) + '?mode=ro&immutable=1'
    with sqlite3.connect(uri, uri=True) as db:
        return sorted(row[0] for row in db.execute('SELECT id FROM Node'))


# --------------------------------------------------------------------------- replay


def _wall(mode, row, index):
    if mode == 'real':
        return row['monotonic_ns']/1e9
    if mode == 'ros':
        return row['ros_ns']/1e9
    if mode == 'tiny':
        return index*1e-6
    raise ValueError('unknown clock mode ' + repr(mode))


class _Instrumented(LocalizationGate):
    """The real gate, with hooks that expose which call sealed which stamp.

    Nothing in the decision path is overridden: the hooks only observe.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.timed_out, self.revokes, self.streak_revokes = set(), [], []

    def tick(self, wall, ros_ns):
        before = set(self.pending)
        super().tick(wall, ros_ns)
        self.timed_out |= before - set(self.pending)

    def revoke(self, state, reason):
        resets_streak = bool(self.state == 'valid' or self.approved or self.streak)
        self.revokes.append(reason)
        if resets_streak:
            self.streak_revokes.append(reason)
        super().revoke(state, reason)


def replay(run, prior_ids, mode='real', margins=False, attribution=False):
    """Drive the real gate with the recorded call sequence under clock ``mode``.

    Returns {'approved', 'mismatches', 'revokes', 'min_margin_ms', 'sealed'}.
    ``mismatches`` compares the post-call (state, reason, epoch, pending_count,
    approved_count, emitted approvals) against the recorded event; it is only
    meaningful for mode == 'real'.  ``sealed`` maps stamp -> (cause, detail).
    """
    gate = _Instrumented(run.map_id(), prior_ids, 'camera_link', 'map', 'odom')
    approved, mismatches, revokes = [], [], {}
    margin = {'pending_s': None, 'stream_s': None}
    sealed, seen_pending, last_streak_revoke = {}, set(), None
    epoch = 0
    for index, row in enumerate(run.events):
        event, ros = row['event'], row['ros_ns']
        wall = _wall(mode, row, index)
        if margins and mode == 'real':
            for item in gate.pending.values():
                gap = abs(wall-item['wall']-PENDING_TIMEOUT_S)
                margin['pending_s'] = gap if margin['pending_s'] is None else min(margin['pending_s'], gap)
            if gate.state in ('valid', 'confirming') and gate.last_good_wall is not None:
                gap = abs(wall-gate.last_good_wall-STREAM_TIMEOUT_S)
                margin['stream_s'] = gap if margin['stream_s'] is None else min(margin['stream_s'], gap)
        gate.timed_out, gate.revokes, gate.streak_revokes = set(), [], []
        before = set(gate.pending)
        emitted = []
        if event == 'acquisition':
            gate.acquisition(row['stamp_ns'], wall, ros)
        elif event.startswith('received_'):
            kind = event[len('received_'):]
            message = run.content.get((kind, row['stamp_ns']))
            if message is None:
                raise KeyError('no recorded %s content for stamp %d in run %s'
                               % (kind, row['stamp_ns'], run.label))
            emitted = [item['stamp_ns'] for item in gate.ingest(kind, message, wall, ros)]
        else:
            gate.status(wall, ros)
        approved += emitted
        if attribution:
            seen_pending |= set(gate.pending)
            gone = before - set(gate.pending)
            emitted_set = set(emitted)
            for stamp in sorted(gate.timed_out & gone):
                sealed.setdefault(stamp, ('wall-timeout-direct', 'incomplete_acquisition_timeout'))
            for stamp in sorted(gone - gate.timed_out):
                if stamp in emitted_set:
                    sealed.setdefault(stamp, ('approved', ''))
                elif run.message_reason(stamp, prior_ids) is not None:
                    sealed.setdefault(stamp, ('content-bad', run.message_reason(stamp, prior_ids)))
                elif event == 'acquisition':
                    sealed.setdefault(stamp, ('other', 'pending cleared by ' + (gate.revokes[-1] if gate.revokes else 'acquisition')))
                elif last_streak_revoke and content_class(last_streak_revoke) == 'WALL':
                    sealed.setdefault(stamp, ('wall-timeout-cascade', last_streak_revoke))
                else:
                    sealed.setdefault(stamp, ('streak-not-yet-3', last_streak_revoke or 'run_start'))
            if gate.streak_revokes:
                last_streak_revoke = gate.streak_revokes[-1]
        gate.status(wall, ros)          # the node's trailing heartbeat
        if gate.epoch != epoch:
            revokes[gate.reason] = revokes.get(gate.reason, 0) + 1
            epoch = gate.epoch
        observed = (gate.state, gate.reason, gate.epoch, len(gate.pending), len(gate.approved), emitted)
        expected = (row['state'], row['reason'], row['epoch'], row['pending_count'],
                    row['approved_count'], [item['stamp_ns'] for item in row.get('approved', ())])
        if observed != expected:
            mismatches.append({'index': index, 'event': event, 'stamp_ns': row.get('stamp_ns'),
                               'recorded': expected, 'replayed': observed})
    return {'approved': approved, 'mismatches': mismatches, 'revokes': revokes, 'sealed': sealed,
            'never_pending': sorted(set(run.published) - seen_pending) if attribution else [],
            'still_pending': sorted(gate.pending),
            'min_margin_ms': {k: (None if v is None else round(v*1e3, 6)) for k, v in margin.items()}}


def known_answer(run, prior_ids):
    """Hard prerequisite: the faithful replay must reproduce the recorded approvals."""
    result = replay(run, prior_ids, 'real', margins=True, attribution=True)
    recorded = run.recorded_approved()
    exact = result['approved'] == recorded
    first = result['mismatches'][0] if result['mismatches'] else None
    return result, {
        'verdict': 'PASS' if exact and not result['mismatches'] else
                   ('PASS_APPROVALS_ONLY' if exact else 'FAIL'),
        'recorded_approved': len(recorded), 'replayed_approved': len(result['approved']),
        'approved_list_identical': exact, 'event_mismatches': len(result['mismatches']),
        'first_mismatch': first, 'min_margin_ms': result['min_margin_ms'],
        'revokes': result['revokes']}


def attribute(run, prior_ids, result):
    """One mechanical cause per published stamp, from the validated gate replay."""
    acquisition, received = {}, {}
    for index, row in enumerate(run.events):
        if row['event'] == 'acquisition':
            acquisition.setdefault(row['stamp_ns'], index)
        elif row['event'].startswith('received_'):
            received.setdefault(row['stamp_ns'], []).append(row)
    out = {}
    for stamp in sorted(run.published):
        rows = received.get(stamp, ())
        if stamp not in acquisition:
            cause, detail = 'missing-acquisition', 'no acquisition event for a published stamp'
        elif stamp in result['sealed']:
            cause, detail = result['sealed'][stamp]
        elif stamp in result['still_pending']:
            cause, detail = 'other', 'still pending when the recording ended'
        elif stamp in result['never_pending']:
            cause, detail = 'other', 'acquisition rejected (stamp never entered pending)'
        else:
            cause, detail = 'other', 'unsealed'
        latency = None
        if stamp in acquisition and rows:
            latency = (rows[-1]['monotonic_ns']-run.events[acquisition[stamp]]['monotonic_ns'])/1e6
        out[stamp] = {'cause': cause, 'detail': detail, 'latency_ms': latency,
                      'acquired': stamp in acquisition, 'messages_at_gate': len(rows),
                      'own_content_reason': run.message_reason(stamp, prior_ids) or ''}
    return out


def content_fields(run, stamp, prior_ids):
    prior = frozenset(prior_ids)
    info = run.content.get(('info', stamp))
    pose = run.content.get(('localization_pose', stamp))
    odom = run.content.get(('odom', stamp))
    odom_info = run.content.get(('odom_info', stamp))
    links = () if info is None else (info.get('loop_closure_id'), info.get('proximity_detection_id'))
    return {
        'loop_closure_id': None if info is None else info.get('loop_closure_id'),
        'proximity_detection_id': None if info is None else info.get('proximity_detection_id'),
        'ref_id': None if info is None else info.get('ref_id'),
        'link_in_prior_map': None if info is None else any(type(x) is int and x in prior for x in links),
        'localization_pose_covariance_valid':
            None if pose is None else covariance_valid(pose['pose']['covariance']),
        'odom_covariance_valid':
            None if odom is None else covariance_valid(odom['pose']['covariance']),
        'odom_info_covariance_valid':
            None if odom_info is None else covariance_valid(odom_info['covariance']),
        'odometry_lost': None if odom_info is None else odom_info.get('lost'),
    }


# ---------------------------------------------------------------------- driver


def analyse(runs, prior_ids, ideal_modes=('ros', 'tiny')):
    labels = [run.label for run in runs]
    per_run, ideal = {}, {}
    for run in runs:
        faithful, verdict = known_answer(run, prior_ids)
        per_run[run.label] = {'known_answer': verdict,
                              'attribution': attribute(run, prior_ids, faithful),
                              'approved': run.recorded_approved(),
                              'published': len(run.published),
                              'acquisitions': sum(1 for r in run.events if r['event'] == 'acquisition')}
        ideal[run.label] = {}
        for mode in ideal_modes:
            result = replay(run, prior_ids, mode)
            ideal[run.label][mode] = {'approved': sorted(set(result['approved'])),
                                      'count': len(set(result['approved'])),
                                      'revokes': result['revokes']}
    stamps = sorted({s for run in runs for s in run.published})
    rows = []
    for stamp in stamps:
        for run in runs:
            record = {'stamp_ns': stamp, 'run': run.label,
                      'frame_index': run.published.get(stamp),
                      'published': stamp in run.published}
            record.update(per_run[run.label]['attribution'].get(
                stamp, {'cause': 'not-published', 'detail': '', 'latency_ms': None,
                        'acquired': False, 'messages_at_gate': 0}))
            record.update(content_fields(run, stamp, prior_ids))
            record['approved'] = stamp in set(per_run[run.label]['approved'])
            record['attribution_validated'] = (
                per_run[run.label]['known_answer']['verdict'] != 'FAIL')
            for mode in ideal_modes:
                record['approved_ideal_'+mode] = stamp in set(ideal[run.label][mode]['approved'])
            rows.append(record)
    summary = {'schema': 1, 'runs': labels, 'prior_ids': sorted(prior_ids),
               'per_run': {}, 'pairs': {}, 'content_identity': {}}
    for run in runs:
        counts = {}
        for value in per_run[run.label]['attribution'].values():
            counts[value['cause']] = counts.get(value['cause'], 0) + 1
        cross = {}
        for value in per_run[run.label]['attribution'].values():
            key = value['cause'] + '|own_content_' + ('bad' if value['own_content_reason'] else 'healthy')
            cross[key] = cross.get(key, 0) + 1
        summary['per_run'][run.label] = {
            'cause_vs_own_content': cross,
            'attribution_validated': per_run[run.label]['known_answer']['verdict'] != 'FAIL',
            'published': per_run[run.label]['published'],
            'acquisitions': per_run[run.label]['acquisitions'],
            'approved': len(set(per_run[run.label]['approved'])),
            'cause_counts': counts,
            'known_answer': per_run[run.label]['known_answer'],
            'idealized': {m: {'count': ideal[run.label][m]['count'],
                              'revokes': ideal[run.label][m]['revokes']} for m in ideal_modes}}
    for i, a in enumerate(runs):
        for b in runs[i+1:]:
            sa, sb = set(a.recorded_approved()), set(b.recorded_approved())
            entry = {'symmetric_difference': len(sa ^ sb),
                     'only_'+a.label: {}, 'only_'+b.label: {}}
            for stamp in sorted(sa-sb):
                cause = per_run[b.label]['attribution'].get(stamp, {}).get('cause', 'not-published')
                entry['only_'+a.label][cause] = entry['only_'+a.label].get(cause, 0)+1
            for stamp in sorted(sb-sa):
                cause = per_run[a.label]['attribution'].get(stamp, {}).get('cause', 'not-published')
                entry['only_'+b.label][cause] = entry['only_'+b.label].get(cause, 0)+1
            for mode in ideal_modes:
                ia = set(ideal[a.label][mode]['approved'])
                ib = set(ideal[b.label][mode]['approved'])
                entry['idealized_'+mode+'_symmetric_difference'] = len(ia ^ ib)
            summary['pairs'][a.label+'|'+b.label] = entry
    keys = ('loop_closure_id', 'proximity_detection_id', 'link_in_prior_map',
            'localization_pose_covariance_valid', 'odom_covariance_valid',
            'odom_info_covariance_valid', 'odometry_lost')
    identical = 0
    differing = {k: 0 for k in keys}
    for stamp in stamps:
        values = [content_fields(run, stamp, prior_ids) for run in runs]
        same = True
        for key in keys:
            if len({repr(v[key]) for v in values}) > 1:
                differing[key] += 1
                same = False
        identical += same
    summary['content_identity'] = {'frames': len(stamps), 'identical_across_all_runs': identical,
                                   'differing_frames_per_field': differing}
    return rows, summary, ideal


CSV_COLUMNS = ('stamp_ns', 'run', 'frame_index', 'published', 'acquired', 'messages_at_gate',
               'own_content_reason', 'attribution_validated',
               'latency_ms', 'loop_closure_id', 'proximity_detection_id', 'ref_id',
               'link_in_prior_map', 'localization_pose_covariance_valid', 'odom_covariance_valid',
               'odom_info_covariance_valid', 'odometry_lost', 'approved', 'cause', 'detail')


def write_outputs(out_dir, rows, summary, ideal, ideal_modes):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    columns = list(CSV_COLUMNS) + ['approved_ideal_'+m for m in ideal_modes]
    with (out_dir/'per-frame.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    (out_dir/'per-frame.json').write_text(json.dumps(rows, indent=1, sort_keys=True)+'\n')
    (out_dir/'drift-summary.json').write_text(json.dumps(summary, indent=1, sort_keys=True)+'\n')
    (out_dir/'idealized-replay.json').write_text(json.dumps(ideal, indent=1, sort_keys=True)+'\n')
    return out_dir


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--run', action='append', required=True, metavar='LABEL=PATH',
                        help='a run directory (the "normal" product directory); repeatable')
    parser.add_argument('--out', required=True, type=Path, help='output directory (created)')
    parser.add_argument('--prior-db', type=Path,
                        help='pinned prior map database; defaults to map.db inside the first run')
    parser.add_argument('--ideal-mode', action='append', default=None, choices=('ros', 'tiny'),
                        help='idealized clock variants (default: both)')
    args = parser.parse_args(argv)
    runs = []
    for spec in args.run:
        if '=' not in spec:
            parser.error('--run expects LABEL=PATH, got ' + repr(spec))
        label, path = spec.split('=', 1)
        runs.append(Run.from_dir(label, path))
    if len({run.map_id() for run in runs}) != 1:
        parser.error('runs do not share one pinned map identity: %s'
                     % sorted({run.map_id() for run in runs}))
    db = args.prior_db or (runs[0].path/'map.db')
    prior_ids = prior_ids_from_db(db)
    modes = tuple(args.ideal_mode) if args.ideal_mode else ('ros', 'tiny')
    rows, summary, ideal = analyse(runs, prior_ids, modes)
    summary['prior_db'] = os.fspath(Path(db).resolve())
    out = write_outputs(args.out, rows, summary, ideal, modes)
    for label in summary['runs']:
        entry = summary['per_run'][label]
        print('%-10s approved=%-3d acquisitions=%-3d known_answer=%-18s ideal=%s causes=%s'
              % (label, entry['approved'], entry['acquisitions'],
                 entry['known_answer']['verdict'],
                 {m: entry['idealized'][m]['count'] for m in modes},
                 {k: entry['cause_counts'][k] for k in CAUSES if k in entry['cause_counts']}))
    for pair, entry in summary['pairs'].items():
        print('%-22s symdiff(real)=%-3d %s' % (pair, entry['symmetric_difference'],
              {m: entry['idealized_'+m+'_symmetric_difference'] for m in modes}))
    print('content frames identical across all runs: %d/%d'
          % (summary['content_identity']['identical_across_all_runs'],
             summary['content_identity']['frames']))
    print('written to', out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
