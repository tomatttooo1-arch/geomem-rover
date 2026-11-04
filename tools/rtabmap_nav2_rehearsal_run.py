#!/usr/bin/env python3
"""WP4b: run the four pre-registered Nav2 rehearsal containers and judge them mechanically.

The protocol is `docs/design/M2B_NAV2_REHEARSAL_PROTOCOL_20260914.md`. It was written and
committed before any container ran; this runner executes it and does not reinterpret it.
Every threshold, run name, argv and verdict rule below comes from that document.

Two things are worth saying out loud about the layout, because both are load-bearing:

* The probe's run NAME inside every container is `normal`, not the rehearsal run label.
  The probe writes `slam.yaml` with `database_path: /rosresults/<name>/map.db`, so §2's
  requirement that `odom.yaml`/`slam.yaml` be BYTE-IDENTICAL to the frozen query's is only
  satisfiable with `<name> == normal`; naming the probe run `loaded` would make a
  pre-registered criterion fail for a reason that has nothing to do with the thing under
  test. Runs stay separated on the host instead: `<ROOT>/<run>` is mounted at `/rosresults`
  and the container's products land in `<ROOT>/<run>/normal/`. A side benefit is that
  `baseline`'s `commands.json` is then directly byte-comparable with the frozen run's.
  Everything DERIVED on the host afterwards (`approved-stamps.json`, `verdict*.json`,
  `permit-curve.json`) is written one level up, in `<ROOT>/<run>/`: the container writes its
  own directory as a remapped uid, so that directory is not writable by this account, and a
  derived file placed there would abort the round for a reason that has nothing to do with
  the thing under test.
* `run_container` and `common_create` are imported from `rtabmap_reload_coordinate_run`
  rather than copied, so the container discipline of the frozen query (owned name, no
  network, recorded create argv, recorded inspect, forced removal) is literally the same
  code and cannot drift between the two.

Nothing here decides anything a human has to be trusted for: `verdict_level` is a pure
function of recorded booleans, and it is unit tested.
"""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

CHECKOUT = Path(__file__).resolve().parents[1]
if str(CHECKOUT) not in sys.path:
    sys.path.insert(0, str(CHECKOUT))

from tools.rtabmap_reload_coordinate_run import (  # noqa: E402
    DATASET_NAME, IMAGE, common_create, container_prefix, docker, image_id, run_container, sha256)

RC = Path('${FPGA_SRC}/runs/rtabmap-reload-coordinate-20260912')
SOURCE_DATABASE = RC / 'mapping-stage-v4' / 'normal' / 'rtabmap.db'
DATABASE_SHA256 = '2a450fd79970fe4c145af5afab6b76a9420b2880c8bbb70581e9aafa1bfab8c7'
DATASET = RC / 'prepared-v2' / 'query' / DATASET_NAME
BUNDLE = RC / 'map-bundle-v2'
FROZEN_RUN = RC / 'query-normal-stage-v1' / 'normal'
FROZEN_APPROVED_COUNT = 51

CONTAINER_PREFIX = 'fpga-dev1-nav2-rehearsal'
PROBE_NAME = 'normal'
CONTAINER_TIMEOUT_S = 630
RUNS = ('baseline', 'loaded', 'neg-static-map-odom', 'neg-no-adapter')
NEGATIVE_STOP_INDEX = 140
LOAD1_LIMIT = 16.
LOAD1_POLL_S = 60.
LOAD1_MAX_WAIT_S = 600.

EXTRINSICS_IN_CONTAINER = '/checkout/robot_sim/fpga_sim/config/extrinsics_sim.json'
# Protocol §4 table, transcribed. `--gate-arg=--flag` form for values that start with a
# dash is the probe's documented escape; the bare ones are the flags' values.
LOADED_EXTRA = ['--gate-arg=--no-publish-tracked-frame',
                '--gate-arg=--map-odom-hold-hz', '--gate-arg', '20',
                '--gate-arg=--map-odom-future-s', '--gate-arg', '0.1',
                '--gate-arg=--bundle', '--gate-arg', '/bundle',
                '--gate-remap', '/fpga/localization_tf:=/tf',
                '--adapter', '--static-extrinsics', EXTRINSICS_IN_CONTAINER, '--recorder']

# §5.4: which named checks each negative control MUST fail for the instrument to have any
# discriminating power. T2 -> (b); T3 -> (c); T4 -> (d) and (i), either of which failing
# means the /odom contract is broken.
NEGATIVE_REQUIRED_FAILURES = {
    'neg-static-map-odom': [['no_child_both_static_and_dynamic']],
    'neg-no-adapter': [['required_edges_present'],
                       ['odom_message_frames', 'odom_message_count_matches_tf']],
}

SOURCE_ROLES = {
    'runner': 'tools/rtabmap_nav2_rehearsal_run.py',
    'reload_runner': 'tools/rtabmap_reload_coordinate_run.py',
    'probe': 'tools/rtabmap_ros_probe.py',
    'replay': 'tools/rtabmap_ros_replay.py',
    'gate_node': 'host/ros_localization_gate.py',
    'gate_core': 'host/localization_gate.py',
    'adapter_node': 'host/ros_odom_base_adapter.py',
    'adapter_core': 'host/odom_base_adapter.py',
    'recorder': 'tools/tf_topic_recorder.py',
    'checker': 'tools/nav2_interface_check.py',
    'checker_core': 'tools/nav2_interface_core.py',
}


# --------------------------------------------------------------------------- pure helpers

def stop_index(run):
    """§4: the two negative controls may stop at 140 frames; the judged pair runs 230."""
    return NEGATIVE_STOP_INDEX if run.startswith('neg-') else 230


def probe_args(run):
    """The probe argv for one run. Pure, so the whole plan can be pinned in a unit test.

    The first block is frozen: it is the frozen query's own invocation (§2), not the
    probe's defaults, and it is identical for all four runs apart from --stop-index.
    """
    if run not in RUNS:
        raise ValueError('unknown rehearsal run: ' + repr(run))
    argv = [PROBE_NAME, '--mode', 'normal', '--reset-countdown', '10', '--odom-tf-wait', '0',
            '--input', '/query/' + DATASET_NAME, '--start-index', '0',
            '--stop-index', str(stop_index(run)), '--timeout-seconds', '600',
            '--online-gate', '--gate-rgb-reliable']
    if run == 'baseline':
        return argv
    extra = list(LOADED_EXTRA)
    if run == 'neg-no-adapter':
        extra = [item for item in extra if item != '--adapter']
    if run == 'neg-static-map-odom':
        extra += ['--extra-static-identity', 'map:odom']
    return argv + extra


def approved_stamps(events_path):
    """§5.1: the approved stamp set, deduplicated and sorted, from `received_*` events."""
    stamps = set()
    for line in Path(events_path).read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        event = json.loads(line)
        if str(event.get('event', '')).startswith('received_'):
            for item in event.get('approved') or []:
                stamps.add(item['stamp_ns'])
    return sorted(stamps)


def acquisition_stamps(events_path):
    """Frame stamps in replay order, one per `acquisition` event."""
    stamps = []
    for line in Path(events_path).read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        event = json.loads(line)
        if event.get('event') == 'acquisition':
            stamps.append(event['stamp_ns'])
    return stamps


def stamp_comparison(observed, reference):
    """Elementwise comparison of two approved-stamp sets, with the difference named."""
    observed, reference = list(observed), list(reference)
    observed_set, reference_set = set(observed), set(reference)
    return {'equal': observed == reference,
            'observed_count': len(observed), 'reference_count': len(reference),
            'only_in_observed': sorted(observed_set - reference_set),
            'only_in_reference': sorted(reference_set - observed_set)}


def negative_is_discriminating(run, verdict):
    """§5.4: a negative control has to FAIL, and to fail at the checks it was built to trip.

    `not_evaluable` counts as "did not pass": removing the adapter removes /odom entirely,
    so the odom checks have no input, which is exactly the failure the control models.
    """
    if not verdict:
        return False
    if verdict.get('pass'):
        return False
    not_passing = set(verdict.get('failed') or []) | set(verdict.get('not_evaluable') or [])
    return all(any(name in not_passing for name in group)
               for group in NEGATIVE_REQUIRED_FAILURES[run])


def verdict_level(state):
    """Protocol §6, mechanically. Returns (level, reason). Pure; unit tested.

    Order of precedence, fixed here before any run: a broken instrument or a missing product
    first (nothing else can be read), then a non-discriminating negative control (§5.4 makes
    the whole round INSTRUMENT_ERROR), then G1 drift (§5.1 forbids evaluating `loaded` at
    all), then the judgments about the design: G2, then the T/C checks on `loaded`.
    """
    if not state.get('all_containers_clean', False):
        return 'INSTRUMENT_ERROR', 'a container failed, timed out or left no products'
    if not state.get('databases_unchanged', False):
        return 'INSTRUMENT_ERROR', 'the read-only database hash changed across a run'
    if not state.get('negative_controls_discriminating', False):
        return 'INSTRUMENT_ERROR', 'a negative control did not fail the checks it models'
    if not state.get('config_identity_ok', False):
        return 'INSTRUMENT_DRIFT', 'odom.yaml/slam.yaml differ from the frozen query'
    if not state.get('g1', False):
        return 'INSTRUMENT_DRIFT', 'G1: baseline approvals differ from the frozen 51'
    if not state.get('g2', False):
        return 'FAIL', 'G2: loaded approvals differ from baseline'
    if not state.get('loaded_checks_pass', False):
        return 'FAIL', 'the interface check failed on loaded'
    return 'PASS', 'G1, G2, T1-T7, C-valid, C-leave, C-count, hygiene and both negatives hold'


# --------------------------------------------------------------------------- side effects

def load1():
    return float(Path('/proc/loadavg').read_text().split()[0])


def wait_for_load(record):
    """The gate has wall-clock timeouts and G1 is a pacing-sensitive hard gate."""
    waited = 0.
    while load1() > LOAD1_LIMIT and waited < LOAD1_MAX_WAIT_S:
        record.append({'load1': load1(), 'waited_s': waited})
        time.sleep(LOAD1_POLL_S)
        waited += LOAD1_POLL_S
    return load1()


def stage_database(root):
    staging = root / 'database-input'
    if staging.exists():
        raise FileExistsError(staging)
    (staging / 'normal').mkdir(parents=True)
    staged = staging / 'normal' / 'rtabmap.db'
    shutil.copyfile(SOURCE_DATABASE, staged)
    for path in (SOURCE_DATABASE, staged):
        if sha256(path) != DATABASE_SHA256:
            raise RuntimeError('database identity mismatch: ' + str(path))
    return staged


def bundle_check(root):
    command = [sys.executable, str(CHECKOUT / 'tools' / 'rtabmap_bundle_map.py'), 'check',
               str(BUNDLE), '--map-id', DATABASE_SHA256, '--require-not-ready']
    completed = subprocess.run(command, cwd=str(CHECKOUT), text=True, timeout=300,
                               env=dict(os.environ, PYTHONPATH=str(CHECKOUT)),
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    record = {'argv': command, 'returncode': completed.returncode, 'stdout': completed.stdout}
    (root / 'bundle-check.json').write_text(json.dumps(record, indent=2) + '\n')
    if completed.returncode != 0:
        raise RuntimeError('map bundle check failed before the first run')
    return record


def source_hashes():
    return {role: sha256(CHECKOUT / relative) for role, relative in SOURCE_ROLES.items()}


def head_commit():
    return subprocess.run(['git', '-C', str(CHECKOUT), 'rev-parse', 'HEAD'], text=True,
                          check=True, stdout=subprocess.PIPE).stdout.strip()


def create_command(name, run, stage):
    return [*common_create(name),
            '--env', 'PYTHONPATH=/checkout',
            '--tmpfs', '/normal:ro,nosuid,nodev,noexec,size=4096',
            '--tmpfs', '/blank:ro,nosuid,nodev,noexec,size=4096',
            '--mount', f'type=bind,src={DATASET},dst=/query/{DATASET_NAME},readonly',
            '--mount', f'type=bind,src={stage.parent / "database-input"},dst=/results,readonly',
            '--mount', f'type=bind,src={stage},dst=/rosresults',
            '--mount', f'type=bind,src={CHECKOUT / "tools"},dst=/probe_tools,readonly',
            '--mount', f'type=bind,src={CHECKOUT / "host"},dst=/checkout/host,readonly',
            '--mount', f'type=bind,src={BUNDLE},dst=/bundle,readonly',
            '--mount', (f'type=bind,src={CHECKOUT / "robot_sim" / "fpga_sim" / "config"},'
                        'dst=/checkout/robot_sim/fpga_sim/config,readonly'),
            IMAGE, '-c', 'source /opt/ros/humble/setup.bash; exec "$@"', 'bash',
            'python3', '/probe_tools/rtabmap_ros_probe.py', *probe_args(run)]


def config_identity(products):
    """§2: the two generated parameter files must be byte-identical to the frozen query's."""
    identity = {}
    for name in ('odom.yaml', 'slam.yaml'):
        produced, frozen = products / name, FROZEN_RUN / name
        identity[name] = {
            'produced_sha256': sha256(produced) if produced.is_file() else None,
            'frozen_sha256': sha256(frozen),
            'identical': produced.is_file() and produced.read_bytes() == frozen.read_bytes()}
    return identity


def run_checker(products, destination, run, expect_approved, allow_missing=None):
    logs = sorted(str(path) for path in products.glob('*.log'))
    command = [sys.executable, str(CHECKOUT / 'tools' / 'nav2_interface_check.py'), 'check',
               str(products / 'tf-record.jsonl'), '--out', str(destination),
               '--c2-grace-samples', '6', '--c1-min-age-s', '-0.167', '--c1-max-age-s', '0.3',
               '--c2-drop-s', '0.2', '--c1-stale-s', '1.2', '--c1-stale-age-s', '1.0',
               '--map-odom-mode', 'hold']
    if expect_approved is not None:
        command += ['--expect-approved', str(expect_approved)]
    for name in allow_missing or []:
        command += ['--allow-missing', name]
    for log in logs:
        command += ['--log', log]
    completed = subprocess.run(command, cwd=str(CHECKOUT), text=True, timeout=900,
                               env=dict(os.environ, PYTHONPATH=str(CHECKOUT)),
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    verdict = json.loads(destination.read_text()) if destination.is_file() else None
    return {'run': run, 'argv': command, 'returncode': completed.returncode,
            'stdout_tail': completed.stdout[-4000:],
            'verdict_path': str(destination) if verdict else None, 'verdict': verdict}


def permit_curve(products, destination):
    """Offline replay of the recorded status timeline through the consumer's own lease.

    Report-only (§7). `wall` is the recorder's monotonic receive time and `ros_ns` the /clock
    value stamped on the same row, which is what the real consumer would have had.
    """
    from host.localization_gate import LocalizationLease
    rows = []
    for line in (products / 'tf-record.jsonl').read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        row = json.loads(line)
        if row.get('topic') == '/fpga/localization_status' and row.get('payload'):
            rows.append(row)
    curve = {'schema': 1, 'source': str(products / 'tf-record.jsonl'), 'status_rows': len(rows),
             'samples': [], 'permits_true': 0}
    lease = None
    for row in rows:
        payload = row['payload']
        if lease is None:
            lease = LocalizationLease(payload['map_id'], payload.get('tracked_frame', 'camera_link'))
        wall, ros_ns = row['recv_monotonic_ns'] / 1e9, row['clock_ns']
        lease.update(payload, wall, ros_ns)
        permits = bool(lease.check(wall, ros_ns))
        curve['permits_true'] += permits
        curve['samples'].append({'clock_ns': ros_ns, 'recv_monotonic_ns': row['recv_monotonic_ns'],
                                 'state': payload.get('state'), 'sequence': payload.get('sequence'),
                                 'approved_count': len(payload.get('approved') or []),
                                 'permits': permits})
    destination.write_text(json.dumps(curve, indent=2) + '\n')
    return {'path': str(destination), 'status_rows': len(rows),
            'permits_true': curve['permits_true']}


def execute(root, runs, prefix):
    identity = {'head_commit': head_commit(), 'image': IMAGE, 'image_id': image_id(),
                'source_sha256': source_hashes(), 'probe_run_name': PROBE_NAME,
                'products_layout': '<ROOT>/<run>/' + PROBE_NAME,
                'database_source': str(SOURCE_DATABASE), 'database_sha256': DATABASE_SHA256,
                'dataset': str(DATASET), 'bundle': str(BUNDLE), 'frozen_run': str(FROZEN_RUN)}
    summary = {'schema': 1, 'protocol': 'docs/design/M2B_NAV2_REHEARSAL_PROTOCOL_20260914.md',
               'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
               'identity': identity, 'runs': {}, 'order': list(runs)}
    summary['bundle_check'] = bundle_check(root)
    staged = stage_database(root)
    summary['database_input'] = {'path': str(staged), 'sha256': sha256(staged)}

    frozen_stamps = approved_stamps(FROZEN_RUN / 'gate-events.jsonl')
    summary['frozen_approved_count'] = len(frozen_stamps)
    baseline_stamps = None
    baseline_expected_140 = None

    for run in runs:
        stage = root / run
        products = stage / PROBE_NAME
        name = f'{prefix}-{run}'
        load_waits = []
        record = {'run': run, 'container': name, 'stop_index': stop_index(run),
                  'probe_args': probe_args(run), 'load1_waits': load_waits}
        record['load1_at_start'] = wait_for_load(load_waits)
        record['database_sha256_before'] = sha256(staged)
        host_identity = dict(identity, run=run, probe_args=probe_args(run))
        started = time.time()
        try:
            record['container_result'] = run_container(name, create_command(name, run, stage),
                                                       stage, CONTAINER_TIMEOUT_S, host_identity)
            record['container_error'] = None
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
            record['container_result'] = None
            record['container_error'] = f'{type(exc).__name__}: {exc}'
        record['wall_seconds'] = time.time() - started
        result_file = stage / 'container-result.json'
        if record['container_result'] is None and result_file.is_file():
            record['container_result'] = json.loads(result_file.read_text())
        record['database_sha256_after'] = sha256(staged)
        record['database_unchanged'] = record['database_sha256_before'] == record['database_sha256_after']
        record['container_exit_code'] = (record['container_result'] or {}).get('container_exit_code')
        record['timed_out'] = (record['container_result'] or {}).get('timed_out')

        runner_result = products / 'runner-result.json'
        record['runner_result'] = json.loads(runner_result.read_text()) if runner_result.is_file() else None
        record['products'] = str(products)
        record['products_present'] = products.is_dir()
        record['config_identity'] = config_identity(products) if products.is_dir() else None
        record['config_identity_ok'] = bool(record['config_identity']) and all(
            entry['identical'] for entry in record['config_identity'].values())

        events = products / 'gate-events.jsonl'
        if events.is_file():
            stamps = approved_stamps(events)
            record['approved_count'] = len(stamps)
            record['approved_stamps_path'] = str(stage / 'approved-stamps.json')
            (stage / 'approved-stamps.json').write_text(json.dumps(
                {'schema': 1, 'run': run, 'source': str(events), 'count': len(stamps),
                 'stamps_ns': stamps}, indent=2) + '\n')
            record['g1'] = None
            record['g2'] = None
            if run == 'baseline':
                baseline_stamps = stamps
                record['g1_comparison'] = stamp_comparison(stamps, frozen_stamps)
                record['g1'] = record['g1_comparison']['equal']
                acquisitions = acquisition_stamps(events)
                early = set(acquisitions[:NEGATIVE_STOP_INDEX])
                baseline_expected_140 = len([s for s in stamps if s in early])
                record['approved_within_first_140_frames'] = baseline_expected_140
            else:
                record['frozen_comparison'] = stamp_comparison(stamps, frozen_stamps)
                if baseline_stamps is not None:
                    comparison = stamp_comparison(stamps, baseline_stamps)
                    record['baseline_comparison'] = comparison
                    if run == 'loaded':
                        record['g2'] = comparison['equal']
        else:
            record['approved_count'] = None

        record['checker'] = None
        record['checker_allow_missing'] = None
        if run != 'baseline' and (products / 'tf-record.jsonl').is_file():
            expected = FROZEN_APPROVED_COUNT if run == 'loaded' else baseline_expected_140
            record['expect_approved'] = expected
            record['expect_approved_basis'] = (
                'protocol §5.1 frozen 51/230' if run == 'loaded' else
                'baseline approvals within the first 140 replayed frames (informational for '
                'negatives: the verdict rule only uses which checks they fail)')
            record['checker'] = run_checker(products, stage / 'verdict.json', run, expected)
            verdict = record['checker']['verdict'] or {}
            not_evaluable = list(verdict.get('not_evaluable') or [])
            if not_evaluable == ['motion_permit_publisher_unique']:
                record['checker_allow_missing'] = run_checker(
                    products, stage / 'verdict-allow-missing.json', run, expected,
                    allow_missing=['motion_permit_publisher_unique'])
        elif run != 'baseline':
            record['checker'] = {'run': run, 'verdict': None,
                                 'error': 'tf-record.jsonl absent; the recorder produced nothing'}

        if run == 'loaded' and (products / 'tf-record.jsonl').is_file():
            try:
                record['permit_curve'] = permit_curve(products, stage / 'permit-curve.json')
            except (OSError, ValueError, KeyError, TypeError) as exc:
                record['permit_curve'] = {'error': f'{type(exc).__name__}: {exc}'}

        summary['runs'][run] = record

    summary['verdict_inputs'] = verdict_inputs(summary, runs)
    level, reason = verdict_level(summary['verdict_inputs'])
    summary['verdict_level'] = level
    summary['verdict_reason'] = reason
    summary['finished_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
    summary['leftover_containers'] = docker(
        'ps', '-a', '--filter', f'name={prefix}', '--format', '{{.Names}}',
        check=False).stdout.strip().splitlines()
    (root / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    return summary, level


def chosen_verdict(record):
    """The verdict that counts: the --allow-missing rerun when the protocol's T7 licensed it."""
    relaxed = record.get('checker_allow_missing') or {}
    if relaxed.get('verdict'):
        return relaxed['verdict']
    return (record.get('checker') or {}).get('verdict')


def verdict_inputs(summary, runs):
    """Collect the booleans `verdict_level` reads. Everything here is already recorded."""
    records = [summary['runs'][run] for run in runs]
    clean = all(record.get('container_exit_code') == 0 and not record.get('timed_out')
                and record.get('products_present')
                and (record.get('runner_result') or {}).get('all_owned_processes_exited') is True
                and (record.get('runner_result') or {}).get('replay_exit_code') == 0
                for record in records) and set(runs) == set(RUNS)
    loaded = summary['runs'].get('loaded', {})
    loaded_verdict = chosen_verdict(loaded) if loaded else None
    negatives = {run: negative_is_discriminating(run, chosen_verdict(summary['runs'][run]))
                 for run in runs if run in NEGATIVE_REQUIRED_FAILURES}
    return {
        'runs_executed': list(runs),
        'all_containers_clean': clean,
        'databases_unchanged': all(record.get('database_unchanged') for record in records),
        'config_identity_ok': all(record.get('config_identity_ok') for record in records),
        'g1': bool(summary['runs'].get('baseline', {}).get('g1')),
        'g2': bool(loaded.get('g2')),
        'loaded_checks_pass': bool((loaded_verdict or {}).get('pass')),
        'loaded_failed_names': list((loaded_verdict or {}).get('failed') or []),
        'loaded_not_evaluable_names': list((loaded_verdict or {}).get('not_evaluable') or []),
        'negative_controls': negatives,
        'negative_controls_discriminating': bool(negatives) and all(negatives.values()),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--runs', default=','.join(RUNS),
                        help='comma-separated subset of ' + ','.join(RUNS))
    parser.add_argument('--container-prefix', type=container_prefix, default=CONTAINER_PREFIX)
    args = parser.parse_args()
    runs = [run.strip() for run in args.runs.split(',') if run.strip()]
    unknown = [run for run in runs if run not in RUNS]
    if unknown:
        parser.error('unknown runs: ' + ','.join(unknown))
    root = args.output.resolve()
    if root.exists():
        parser.error('output root already exists: ' + str(root))
    root.mkdir(parents=True)
    summary, level = execute(root, runs, args.container_prefix)
    manifest = subprocess.run(['bash', str(CHECKOUT / 'tools' / 'manifest.sh'), str(root)],
                              text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              timeout=1800)
    print(manifest.stdout.strip())
    print(json.dumps({'verdict_level': level, 'verdict_reason': summary['verdict_reason'],
                      'verdict_inputs': summary['verdict_inputs']}, indent=2, ensure_ascii=False))
    print('REHEARSAL_DONE ' + level)


if __name__ == '__main__':
    main()
