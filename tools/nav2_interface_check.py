#!/usr/bin/env python3
"""Static interface health check over a `tools/tf_topic_recorder.py` recording.

Pure Python, no ROS: it reads the JSONL recording (plus, optionally, the node logs and the
`ros2 topic info` snapshot taken next to it) and answers the Nav2 handoff questions from
`docs/design/M2B_NAV2_HANDOFF_PLAN_20260914.md` §3 / `LOCALIZATION_HANDOFF_INTERFACE.md` §9
one named boolean at a time. It decides nothing about the rehearsal protocol: every
threshold is a CLI parameter so WP4b can pre-register the numbers instead of inheriting
whatever happened to be hard-coded here.

Checks (each reported with its evidence):

  single_parent_per_dynamic_child   (a) no dynamic child frame has two parents
  no_child_both_static_and_dynamic  (b) no child frame appears in both TF buffers — the
                                        silent worst case, where a dynamic edge is filed in
                                        the never-expiring static cache
  required_edges_present            (c) map->odom and odom->base_link dynamic,
                                        base_link->camera_link and
                                        camera_link->camera_optical_frame static
  odom_message_frames               (d) /odom carries frame_id=odom, child_frame_id=base_link
  status_fields_present             (e) both status topics carry their declared labels
  stamp_clock_relation              (f) held map->odom stamps are ahead of /clock and
                                        non-decreasing; odom->base_link stamps are the
                                        /odom stamps
  tf_repeated_data_absent           (g) zero repeated (parent, child, stamp) triples, from
                                        the recorder's own count AND from grepping the logs
  motion_permit_publisher_unique    (h) at most one /fpga/motion_permit publisher
  odom_message_count_matches_tf     (i) #/odom == #odom->base_link transforms
  lookup_samples_follow_status      (j) while localization is valid the consumer-side lookup
                                        resolves and is fresh; after it stops being valid the
                                        lookup goes stale and stops resolving, within windows.
                                        The judged criterion is `c2_at_odom`, can_transform at
                                        the newest odom->base_link stamp; `c2_at_clock` is
                                        reported but NEVER judged, because every odometry
                                        stamp in this replay is behind the clock and that
                                        query is therefore false throughout regardless of
                                        whether map->odom is live. The first
                                        --c2-grace-samples samples after entering valid are
                                        exempt from "c2_at_odom must be true": tf2 needs two
                                        held samples bracketing the query time, so the edge
                                        cannot resolve until enough have accumulated. A
                                        transition whose window reaches past the end of the
                                        recording is reported unevaluable, never failed: the
                                        window was never given the chance to fill
  approved_frame_count              (k) only with --expect-approved: distinct approved stamps

A check whose input is missing is reported `not_evaluable` and fails the run, unless it is
named in --allow-missing. Silence is never taken for success.
"""
import argparse
from collections import Counter
import datetime
import json
from pathlib import Path
import sys

if __package__:
    from .nav2_interface_core import (ADAPTER_STATUS_KEYS, ADAPTER_STATUS_TOPIC, BASE_FRAME,
                                      LOCALIZATION_STATUS_KEYS, LOCALIZATION_STATUS_TOPIC,
                                      MOTION_PERMIT_TOPIC, ODOM_FRAME, ODOM_TOPIC, OPTICAL_FRAME,
                                      SAMPLE, SUMMARY, TRACKED_FRAME, WORLD_FRAME)
    from .tf_topic_recorder import TOPIC_INFO_SUFFIX
else:
    from nav2_interface_core import (ADAPTER_STATUS_KEYS, ADAPTER_STATUS_TOPIC, BASE_FRAME,
                                     LOCALIZATION_STATUS_KEYS, LOCALIZATION_STATUS_TOPIC,
                                     MOTION_PERMIT_TOPIC, ODOM_FRAME, ODOM_TOPIC, OPTICAL_FRAME,
                                     SAMPLE, SUMMARY, TRACKED_FRAME, WORLD_FRAME)
    TOPIC_INFO_SUFFIX = '.topic-info.txt'

SCHEMA = 1
REPEATED_MARKER = 'TF_REPEATED_DATA'
VALID = 'valid'

CHECKS = ('single_parent_per_dynamic_child', 'no_child_both_static_and_dynamic',
          'required_edges_present', 'odom_message_frames', 'status_fields_present',
          'stamp_clock_relation', 'tf_repeated_data_absent', 'motion_permit_publisher_unique',
          'odom_message_count_matches_tf', 'lookup_samples_follow_status', 'approved_frame_count')


class Report:
    """Named booleans with evidence. `not_evaluable` is a failure with a different reason."""

    def __init__(self, allow_missing=()):
        self.allow_missing = set(allow_missing)
        self.checks = {}

    def add(self, name, passed, **evidence):
        self.checks[name] = {'pass': bool(passed), 'not_evaluable': False,
                             'allowed_missing': False, 'reason': None, 'evidence': evidence}

    def missing(self, name, reason, **evidence):
        allowed = name in self.allow_missing
        self.checks[name] = {'pass': True if allowed else False, 'not_evaluable': True,
                             'allowed_missing': allowed, 'reason': reason, 'evidence': evidence}

    def verdict(self):
        failed = sorted(n for n, c in self.checks.items() if not c['pass'])
        unevaluable = sorted(n for n, c in self.checks.items() if c['not_evaluable'])
        return {'pass': not failed, 'failed': failed, 'not_evaluable': unevaluable,
                'checks': self.checks}


def load_records(path):
    records = []
    with Path(path).open() as handle:
        for number, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except ValueError as exc:
                raise ValueError('%s:%d is not JSON: %s' % (path, number, exc))
            if not isinstance(row, dict) or 'topic' not in row:
                raise ValueError('%s:%d is not a record row' % (path, number))
            records.append(row)
    return records


def transforms_of(records, tf_topics):
    """Every transform, with the record it came from. `static` comes from the row, not the
    topic: a static-flagged edge arriving on /tf is exactly what check (b) is looking for."""
    for row in records:
        if row.get('topic') in tf_topics:
            for transform in row.get('transforms') or ():
                yield row, transform


def edge_key(transform):
    return (transform.get('parent'), transform.get('child'))


def check_single_parent(report, dynamic):
    parents = {}
    for _, transform in dynamic:
        parents.setdefault(transform.get('child'), set()).add(transform.get('parent'))
    if not parents:
        return report.missing('single_parent_per_dynamic_child', 'no dynamic transforms recorded')
    bad = {child: sorted(found) for child, found in parents.items() if len(found) > 1}
    report.add('single_parent_per_dynamic_child', not bad, children_with_multiple_parents=bad,
               dynamic_children={child: sorted(found) for child, found in sorted(parents.items())})


def check_static_dynamic_overlap(report, dynamic, static):
    dynamic_children = {t.get('child') for _, t in dynamic}
    static_children = {t.get('child') for _, t in static}
    if not dynamic_children and not static_children:
        return report.missing('no_child_both_static_and_dynamic', 'no transforms recorded')
    overlap = sorted(dynamic_children & static_children)
    report.add('no_child_both_static_and_dynamic', not overlap, children_in_both=overlap,
               dynamic_children=sorted(dynamic_children), static_children=sorted(static_children))


def check_required_edges(report, dynamic, static, options):
    required = [(options.world_frame, options.odom_frame, False),
                (options.odom_frame, options.base_frame, False),
                (options.base_frame, options.tracked_frame, True),
                (options.tracked_frame, options.optical_frame, True)]
    counts = Counter(edge_key(t) for _, t in dynamic), Counter(edge_key(t) for _, t in static)
    found, missing = {}, []
    for parent, child, is_static in required:
        seen = counts[1 if is_static else 0][(parent, child)]
        found['%s->%s (%s)' % (parent, child, 'static' if is_static else 'dynamic')] = seen
        if not seen:
            missing.append('%s->%s (%s)' % (parent, child, 'static' if is_static else 'dynamic'))
    report.add('required_edges_present', not missing, missing=missing, counts=found)


def check_odom_frames(report, odom_rows, options):
    if not odom_rows:
        return report.missing('odom_message_frames', 'no %s messages recorded' % options.odom_topic)
    bad = [{'stamp_ns': row.get('stamp_ns'), 'frame_id': row.get('frame_id'),
            'child_frame_id': row.get('child_frame_id')} for row in odom_rows
           if row.get('frame_id') != options.odom_frame
           or row.get('child_frame_id') != options.base_frame]
    report.add('odom_message_frames', not bad, messages=len(odom_rows),
               expected={'frame_id': options.odom_frame, 'child_frame_id': options.base_frame},
               offending=bad[:8], offending_count=len(bad))


def check_status_fields(report, status_rows, options):
    expected = {options.localization_status_topic: LOCALIZATION_STATUS_KEYS,
                options.adapter_status_topic: ADAPTER_STATUS_KEYS}
    evidence, missing_inputs, violations = {}, [], {}
    for topic, keys in expected.items():
        rows = [row for row in status_rows.get(topic, []) if isinstance(row.get('payload'), dict)]
        evidence[topic] = {'messages': len(status_rows.get(topic, [])), 'parsed': len(rows),
                           'required_keys': list(keys)}
        if not rows:
            missing_inputs.append(topic)
            continue
        absent = sorted({key for row in rows for key in keys if key not in row['payload']})
        evidence[topic]['absent_keys'] = absent
        evidence[topic]['last_values'] = {key: rows[-1]['payload'].get(key) for key in keys}
        if absent:
            violations[topic] = absent
    if missing_inputs:
        return report.missing('status_fields_present',
                              'no parsed status messages on ' + ', '.join(missing_inputs),
                              topics=evidence)
    report.add('status_fields_present', not violations, topics=evidence, absent_keys=violations)


def check_stamp_clock(report, dynamic, odom_rows, options):
    world_odom = [(row, t) for row, t in dynamic
                  if edge_key(t) == (options.world_frame, options.odom_frame)]
    odom_base = [(row, t) for row, t in dynamic
                 if edge_key(t) == (options.odom_frame, options.base_frame)]
    if not world_odom:
        return report.missing('stamp_clock_relation', 'no %s->%s transforms recorded'
                              % (options.world_frame, options.odom_frame))
    not_future, previous, not_monotonic = [], None, []
    for row, transform in world_odom:
        clock_ns, stamp = row.get('clock_ns'), transform.get('stamp_ns')
        if options.map_odom_mode == 'hold':
            if clock_ns is None:
                not_future.append({'stamp_ns': stamp, 'clock_ns': None})
            elif stamp < clock_ns:
                not_future.append({'stamp_ns': stamp, 'clock_ns': clock_ns,
                                   'lag_s': (clock_ns - stamp) / 1e9})
        if previous is not None and stamp < previous:
            not_monotonic.append({'stamp_ns': stamp, 'previous_stamp_ns': previous})
        previous = stamp
    tf_stamps = {t.get('stamp_ns') for _, t in odom_base}
    odom_stamps = {row.get('stamp_ns') for row in odom_rows}
    stamp_mismatch = None
    if not odom_base and not odom_rows:
        stamp_mismatch = 'no %s->%s transforms and no %s messages' % (
            options.odom_frame, options.base_frame, options.odom_topic)
    elif tf_stamps != odom_stamps:
        stamp_mismatch = {'only_in_tf': sorted(tf_stamps - odom_stamps)[:8],
                          'only_in_odom': sorted(odom_stamps - tf_stamps)[:8],
                          'tf_stamps': len(tf_stamps), 'odom_stamps': len(odom_stamps)}
    report.add('stamp_clock_relation',
               not not_future and not not_monotonic and stamp_mismatch is None,
               mode=options.map_odom_mode, world_odom_transforms=len(world_odom),
               stamps_not_ahead_of_clock=not_future[:8],
               stamps_not_ahead_of_clock_count=len(not_future),
               stamps_not_monotonic=not_monotonic[:8], odom_stamp_mismatch=stamp_mismatch)


def grep_logs(paths):
    hits, unreadable = {}, {}
    for path in paths:
        try:
            text = Path(path).read_text(errors='replace')
        except OSError as exc:
            unreadable[str(path)] = '%s: %s' % (type(exc).__name__, exc)
            continue
        hits[str(path)] = sum(1 for line in text.splitlines() if REPEATED_MARKER in line)
    return hits, unreadable


def check_repeated_data(report, summary, logs):
    hits, unreadable = grep_logs(logs)
    if summary is None:
        return report.missing('tf_repeated_data_absent',
                              'the recording has no summary row (recorder did not shut down '
                              'cleanly), so its repeated-stamp count is unknown',
                              log_hits=hits, unreadable_logs=unreadable)
    if unreadable:
        return report.missing('tf_repeated_data_absent', 'log files could not be read',
                              log_hits=hits, unreadable_logs=unreadable)
    recorded = summary.get('repeated_stamp_count')
    if not isinstance(recorded, int):
        return report.missing('tf_repeated_data_absent',
                              'summary row carries no repeated_stamp_count',
                              log_hits=hits)
    report.add('tf_repeated_data_absent', recorded == 0 and not any(hits.values()),
               repeated_stamp_count=recorded,
               repeated_stamp_examples=summary.get('repeated_stamp_examples', []),
               log_hits=hits, logs_checked=sorted(hits))


def parse_topic_info(text):
    """Split a recorder snapshot into {topic: {'publishers': n|None, 'text': ...}}."""
    sections, topic, lines = {}, None, []
    for line in text.splitlines():
        if line.startswith('=== topic: ') and line.endswith(' ==='):
            if topic is not None:
                sections[topic] = lines
            topic, lines = line[len('=== topic: '):-len(' ===')], []
        elif topic is not None:
            lines.append(line)
    if topic is not None:
        sections[topic] = lines
    parsed = {}
    for topic, lines in sections.items():
        publishers = None
        for line in lines:
            if line.strip().startswith('Publisher count:'):
                try:
                    publishers = int(line.split(':', 1)[1].strip())
                except ValueError:
                    publishers = None
        parsed[topic] = {'publishers': publishers, 'lines': len(lines)}
    return parsed


def check_motion_permit(report, path, options):
    name = 'motion_permit_publisher_unique'
    if path is None or not Path(path).is_file():
        return report.missing(name, 'no topic-info snapshot at %s' % path, path=str(path))
    try:
        parsed = parse_topic_info(Path(path).read_text(errors='replace'))
    except OSError as exc:
        return report.missing(name, '%s: %s' % (type(exc).__name__, exc), path=str(path))
    section = parsed.get(options.motion_permit_topic)
    if section is None or section['publishers'] is None:
        return report.missing(name, 'snapshot has no publisher count for ' + options.motion_permit_topic,
                              path=str(path), topics=sorted(parsed))
    report.add(name, section['publishers'] <= 1, path=str(path),
               publishers=section['publishers'], topic=options.motion_permit_topic)


def check_count_invariant(report, dynamic, odom_rows, options):
    odom_base = [t for _, t in dynamic if edge_key(t) == (options.odom_frame, options.base_frame)]
    if not odom_base and not odom_rows:
        return report.missing('odom_message_count_matches_tf',
                              'no %s messages and no %s->%s transforms recorded'
                              % (options.odom_topic, options.odom_frame, options.base_frame))
    report.add('odom_message_count_matches_tf', len(odom_base) == len(odom_rows),
               odom_messages=len(odom_rows), odom_base_transforms=len(odom_base))


def check_samples(report, records, options):
    """C1/C2 against the localization status timeline.

    The windows are measured on the SIMULATED clock, not on wall time: C1 is an age against
    /clock, and during replay the simulated clock advances far slower than the wall, so a
    wall-time window would make the staleness criterion unfalsifiable.
    """
    name = 'lookup_samples_follow_status'
    samples, statuses, latest, valid_index = [], [], None, 0
    for row in records:
        if row.get('topic') == options.localization_status_topic:
            payload = row.get('payload')
            if not isinstance(payload, dict):
                continue
            latest = payload.get('state')
            if latest != VALID:
                valid_index = 0          # a new valid run restarts the grace window
            statuses.append({'clock_ns': row.get('clock_ns'), 'state': latest,
                             'recv_monotonic_ns': row.get('recv_monotonic_ns')})
        elif row.get('topic') == SAMPLE:
            if latest == VALID:
                valid_index += 1
            samples.append(dict(row, latest_state=latest,
                                valid_sample_index=valid_index if latest == VALID else 0))
    if not samples:
        return report.missing(name, 'no sample rows in the recording')
    if not statuses:
        return report.missing(name, 'no parsed %s messages' % options.localization_status_topic)

    valid_samples = [s for s in samples if s['latest_state'] == VALID]
    graced = [s for s in valid_samples if s['valid_sample_index'] <= options.c2_grace_samples]
    judged = [s for s in valid_samples if s['valid_sample_index'] > options.c2_grace_samples]
    valid_violations = []
    for sample in valid_samples:
        # Inside the grace window C2 is not required and a lookup that raises is tolerated:
        # neither can be satisfied before enough held samples exist to bracket the lookup
        # time. The C1 bounds still apply whenever a stamp came back at all.
        grace = sample['valid_sample_index'] <= options.c2_grace_samples
        reasons = []
        if not grace and sample.get('c2_at_odom') is not True:
            reasons.append('c2_at_odom=%r error=%r'
                           % (sample.get('c2_at_odom'), sample.get('c2_at_odom_error')))
        age = sample.get('c1_age_s')
        if sample.get('c1_error') is not None:
            if not grace:
                reasons.append('c1 raised ' + str(sample.get('c1_error')))
        elif age is None:
            reasons.append('c1 age missing')
        elif age > options.c1_max_age_s:
            reasons.append('c1 age %r > %s' % (age, options.c1_max_age_s))
        elif age < options.c1_min_age_s:
            reasons.append('c1 age %r < %s' % (age, options.c1_min_age_s))
        if reasons:
            valid_violations.append({'clock_ns': sample.get('clock_ns'), 'reasons': reasons,
                                     'valid_sample_index': sample['valid_sample_index'],
                                     'in_grace_window': grace,
                                     'c2_at_clock': sample.get('c2_at_clock')})

    # Where the recording itself ends, on the simulated clock. A window that reaches past
    # this was never given the chance to fill: judging it would report the end of the
    # recording as a criterion violation.
    sample_clocks = [s['clock_ns'] for s in samples if s.get('clock_ns') is not None]
    record_end_clock_ns = sample_clocks[-1] if sample_clocks else None

    transitions, unevaluable = [], []
    for index in range(1, len(statuses)):
        if statuses[index]['state'] == VALID or statuses[index - 1]['state'] != VALID:
            continue
        start = statuses[index]['clock_ns']
        if start is None:
            unevaluable.append({'index': index, 'reason': 'no /clock seen at the transition'})
            continue
        back = next((s['clock_ns'] for s in statuses[index:] if s['state'] == VALID), None)
        entry = {'left_valid_clock_ns': start, 'state': statuses[index]['state'],
                 'returned_to_valid_clock_ns': back}
        window_end = start + int(options.c1_stale_s * 1e9)
        entry['record_end_clock_ns'] = record_end_clock_ns
        if back is not None and back <= window_end:
            entry['skipped'] = 'returned to valid inside the C1 window'
            unevaluable.append(entry)
            continue
        if record_end_clock_ns is not None and record_end_clock_ns < window_end:
            # The recording stopped before the window closed, so neither "did it stay
            # stale" nor "did it come back" was ever observable. Not evaluable, not a
            # failure: an early stop must not read as a criterion violation.
            entry['skipped'] = 'record ended inside the C1 window'
            entry['c1_window_end_clock_ns'] = window_end
            unevaluable.append(entry)
            continue
        after = [s for s in samples if s.get('clock_ns') is not None and s['clock_ns'] > start
                 and (back is None or s['clock_ns'] < back)]
        in_c1 = [s for s in after if s['clock_ns'] <= window_end]
        if not in_c1:
            entry['skipped'] = 'no sample inside the C1 window'
            unevaluable.append(entry)
            continue
        last = in_c1[-1]
        stale = last.get('c1_error') is not None or (
            last.get('c1_age_s') is not None and last['c1_age_s'] > options.c1_stale_age_s)
        entry['c1'] = {'pass': bool(stale), 'window_end_clock_ns': window_end,
                       'samples_in_window': len(in_c1), 'last_age_s': last.get('c1_age_s'),
                       'last_error': last.get('c1_error')}
        after_drop = [s for s in after if s['clock_ns'] > start + int(options.c2_drop_s * 1e9)]
        if not after_drop:
            # The analogous "record ended inside the C2 drop window" case needs no branch of
            # its own: C2 is judged on samples PAST the window, so a recording that stopped
            # inside it has none and lands here already, as not evaluable rather than as a
            # failure. `record_end_clock_ns` above says which of the two it was.
            entry['skipped'] = 'no sample past the C2 window'
            unevaluable.append(entry)
            continue
        still_true = [{'clock_ns': s['clock_ns'], 'odom_base_stamp_ns': s.get('odom_base_stamp_ns')}
                      for s in after_drop if s.get('c2_at_odom') is True]
        entry['c2'] = {'pass': not still_true, 'samples_past_window': len(after_drop),
                       'still_resolvable': still_true[:8],
                       'c2_at_clock_still_true': sum(1 for s in after_drop
                                                     if s.get('c2_at_clock') is True)}
        entry['pass'] = entry['c1']['pass'] and entry['c2']['pass']
        transitions.append(entry)

    evidence = {'samples': len(samples), 'valid_samples': len(valid_samples),
                'valid_samples_in_grace': len(graced), 'valid_samples_judged': len(judged),
                'judged_criterion': 'c2_at_odom',
                'c2_at_clock_true_samples': sum(1 for s in samples
                                                if s.get('c2_at_clock') is True),
                'c2_at_odom_true_samples': sum(1 for s in samples
                                               if s.get('c2_at_odom') is True),
                'valid_violations': valid_violations[:8],
                'valid_violation_count': len(valid_violations),
                'transitions': transitions, 'unevaluable_transitions': unevaluable,
                'record_end_clock_ns': record_end_clock_ns,
                'thresholds': {'c1_max_age_s': options.c1_max_age_s,
                               'c1_min_age_s': options.c1_min_age_s,
                               'c1_stale_s': options.c1_stale_s,
                               'c1_stale_age_s': options.c1_stale_age_s,
                               'c2_drop_s': options.c2_drop_s,
                               'c2_grace_samples': options.c2_grace_samples}}
    if not valid_samples:
        return report.missing(name, 'no sample was taken while localization reported valid',
                              **evidence)
    if not judged:
        return report.missing(name, 'every valid sample fell inside the C2 grace window',
                              **evidence)
    if not transitions:
        return report.missing(name, 'localization never left valid with samples in both windows',
                              **evidence)
    report.add(name, not valid_violations and all(t['pass'] for t in transitions), **evidence)


def check_approved(report, status_rows, options):
    name = 'approved_frame_count'
    rows = [row for row in status_rows.get(options.localization_status_topic, [])
            if isinstance(row.get('payload'), dict)]
    if not rows:
        return report.missing(name, 'no parsed %s messages' % options.localization_status_topic)
    stamps = set()
    for row in rows:
        for approved in row['payload'].get('approved') or ():
            if isinstance(approved, dict) and 'stamp_ns' in approved:
                stamps.add(approved['stamp_ns'])
    report.add(name, len(stamps) == options.expect_approved, expected=options.expect_approved,
               distinct_approved_stamps=len(stamps),
               first_stamp_ns=min(stamps) if stamps else None,
               last_stamp_ns=max(stamps) if stamps else None)


def evaluate(records, options):
    report = Report(options.allow_missing)
    tf_topics = {options.tf_topic, options.tf_static_topic}
    pairs = list(transforms_of(records, tf_topics))
    dynamic = [(row, t) for row, t in pairs if not t.get('static')]
    static = [(row, t) for row, t in pairs if t.get('static')]
    odom_rows = [row for row in records if row.get('topic') == options.odom_topic]
    status_rows = {}
    for row in records:
        if row.get('topic') in (options.localization_status_topic, options.adapter_status_topic):
            status_rows.setdefault(row['topic'], []).append(row)
    summaries = [row for row in records if row.get('topic') == SUMMARY]

    check_single_parent(report, dynamic)
    check_static_dynamic_overlap(report, dynamic, static)
    check_required_edges(report, dynamic, static, options)
    check_odom_frames(report, odom_rows, options)
    check_status_fields(report, status_rows, options)
    check_stamp_clock(report, dynamic, odom_rows, options)
    check_repeated_data(report, summaries[-1] if summaries else None, options.log or [])
    check_motion_permit(report, options.topic_info, options)
    check_count_invariant(report, dynamic, odom_rows, options)
    check_samples(report, records, options)
    if options.expect_approved is not None:
        check_approved(report, status_rows, options)

    verdict = report.verdict()
    verdict.update(schema=SCHEMA, record=str(options.record),
                   generated=datetime.datetime.now(datetime.timezone.utc)
                   .replace(microsecond=0).isoformat(),
                   records=len(records), transforms=len(pairs),
                   options={'c1_max_age_s': options.c1_max_age_s,
                            'c1_min_age_s': options.c1_min_age_s,
                            'c2_grace_samples': options.c2_grace_samples,
                            'c1_stale_s': options.c1_stale_s,
                            'c1_stale_age_s': options.c1_stale_age_s,
                            'c2_drop_s': options.c2_drop_s,
                            'map_odom_mode': options.map_odom_mode,
                            'expect_approved': options.expect_approved,
                            'allow_missing': sorted(options.allow_missing),
                            'logs': [str(path) for path in options.log or []],
                            'topic_info': None if options.topic_info is None else str(options.topic_info),
                            'frames': {'world': options.world_frame, 'odom': options.odom_frame,
                                       'base': options.base_frame, 'tracked': options.tracked_frame,
                                       'optical': options.optical_frame}})
    return verdict


def parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='command', required=True)
    check = sub.add_parser('check', help='judge one recording')
    check.add_argument('record', type=Path)
    check.add_argument('--out', type=Path, required=True, help='verdict JSON destination')
    check.add_argument('--log', '--gate-log', dest='log', action='append', type=Path, default=[],
                       help='node log to grep for ' + REPEATED_MARKER + ' (repeatable)')
    check.add_argument('--topic-info', type=Path,
                       help='ros2 topic info snapshot; default <record>' + TOPIC_INFO_SUFFIX)
    check.add_argument('--expect-approved', type=int,
                       help='enable the approved-frame count check with this expected value')
    check.add_argument('--allow-missing', action='append', default=[], choices=list(CHECKS),
                       metavar='CHECK', help='tolerate this check being not_evaluable (repeatable)')
    check.add_argument('--c1-max-age-s', type=float, default=.3,
                       help='max Time(0) lookup age while localization is valid')
    check.add_argument('--c1-min-age-s', type=float, default=-.167,
                       help='min Time(0) lookup age while valid; the held edge may sit ahead of '
                            'the clock by at most the future offset plus two clock steps '
                            '(default -(0.1 + 2 x 0.033))')
    check.add_argument('--c2-grace-samples', type=int, default=6,
                       help='samples after entering valid that are exempt from requiring '
                            'c2_at_odom, while tf2 accumulates two held samples bracketing the '
                            'newest odometry stamp (5 at a 33.3 ms step and a 0.1 s future '
                            'offset, plus one for jitter)')
    check.add_argument('--c1-stale-s', type=float, default=1.2,
                       help='clock window after leaving valid within which C1 must be stale')
    check.add_argument('--c1-stale-age-s', type=float, default=1.,
                       help='age C1 must exceed inside that window')
    check.add_argument('--c2-drop-s', type=float, default=.15,
                       help='clock window after leaving valid within which c2_at_odom must go '
                            'false and stay false')
    check.add_argument('--map-odom-mode', choices=('hold', 'approval'), default='hold',
                       help='hold also requires map->odom stamps to be ahead of /clock')
    check.add_argument('--world-frame', default=WORLD_FRAME)
    check.add_argument('--odom-frame', default=ODOM_FRAME)
    check.add_argument('--base-frame', default=BASE_FRAME)
    check.add_argument('--tracked-frame', default=TRACKED_FRAME)
    check.add_argument('--optical-frame', default=OPTICAL_FRAME)
    check.add_argument('--tf-topic', default='/tf')
    check.add_argument('--tf-static-topic', default='/tf_static')
    check.add_argument('--odom-topic', default=ODOM_TOPIC)
    check.add_argument('--localization-status-topic', default=LOCALIZATION_STATUS_TOPIC)
    check.add_argument('--adapter-status-topic', default=ADAPTER_STATUS_TOPIC)
    check.add_argument('--motion-permit-topic', default=MOTION_PERMIT_TOPIC)
    return p


def main(argv=None, stream=None):
    options = parser().parse_args(argv)
    for name in ('c1_max_age_s', 'c1_stale_s', 'c1_stale_age_s', 'c2_drop_s',
                 'c2_grace_samples'):
        if not getattr(options, name) >= 0:
            parser().error('--%s must be non-negative' % name.replace('_', '-'))
    if options.c1_min_age_s > options.c1_max_age_s:
        parser().error('--c1-min-age-s must not exceed --c1-max-age-s')
    if options.topic_info is None:
        options.topic_info = Path(str(options.record) + TOPIC_INFO_SUFFIX)
    verdict = evaluate(load_records(options.record), options)
    options.out.parent.mkdir(parents=True, exist_ok=True)
    options.out.write_text(json.dumps(verdict, indent=2, sort_keys=True) + '\n')
    print(json.dumps({'pass': verdict['pass'], 'failed': verdict['failed'],
                      'not_evaluable': verdict['not_evaluable'], 'out': str(options.out)}),
          file=stream or sys.stdout)
    return 0 if verdict['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
