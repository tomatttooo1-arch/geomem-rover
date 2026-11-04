#!/usr/bin/env python3
"""Run the pre-registered arms that locate RTAB-Map's non-reproducibility on identical input.

The protocol is `docs/design/RTABMAP_RNG_SOURCE_PROTOCOL_20260915.md`, written and frozen
before the first container; its sha256 goes into `summary.json` so the read-out can be tied
to the document it was judged against. Nothing here reinterprets it: the arm table, the
conditions under which C and B run, and the verdict are transcribed, and the verdict itself
is `rtabmap_content_compare.verdict`, a pure function with its own tests.

Two implementation points are load-bearing:

* `OMP_NUM_THREADS` is set BOTH on `docker create` and through the probe's own
  `--omp-num-threads`. The probe writes that variable into every child's environment itself,
  so the container-level `--env` alone would be a dead key: the number would appear in
  `docker inspect` and never reach rtabmap.
* The frozen argv comes from `rtabmap_nav2_rehearsal_run.probe_args('baseline')` rather than
  being retyped, so "same probe argv as yesterday's baseline" is enforced by imports instead
  of by proofreading.
"""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

CHECKOUT = Path(__file__).resolve().parents[1]
if str(CHECKOUT) not in sys.path:
    sys.path.insert(0, str(CHECKOUT))

from tools import rtabmap_content_compare as compare  # noqa: E402
from tools.rtabmap_nav2_rehearsal_run import (  # noqa: E402
    BUNDLE, DATABASE_SHA256, DATASET, DATASET_NAME, FROZEN_RUN, IMAGE, PROBE_NAME,
    approved_stamps, common_create, config_identity, container_prefix, docker, head_commit,
    image_id, probe_args, run_container, sha256, source_hashes, stage_database, wait_for_load)

PROTOCOL = 'docs/design/RTABMAP_RNG_SOURCE_PROTOCOL_20260915.md'
CONTAINER_PREFIX = 'fpga-dev1-rng'
CONTAINER_TIMEOUT_S = 630
FROZEN_OMP_NUM_THREADS = 2

# §3, transcribed. `when`: 'always', 'a-differs' (C arms), 'time-permitting' (B1).
ARMS = {
    'a1': {'omp': 1, 'params': [], 'when': 'always'},
    'a2': {'omp': 1, 'params': [], 'when': 'always'},
    'c1': {'omp': 1, 'when': 'a-differs',
           'params': ['--odom-param', 'Vis/CorNNType=3',
                      '--slam-param', 'Vis/CorNNType=3', '--slam-param', 'Kp/NNStrategy=3']},
    'c2': {'omp': 1, 'when': 'a-differs',
           'params': ['--odom-param', 'Vis/CorNNType=3',
                      '--slam-param', 'Vis/CorNNType=3', '--slam-param', 'Kp/NNStrategy=3']},
    'b1': {'omp': FROZEN_OMP_NUM_THREADS, 'params': [], 'when': 'time-permitting'},
}
ARM_ORDER = ('a1', 'a2', 'c1', 'c2', 'b1')
B1_ELAPSED_LIMIT_S = 20 * 60.
ROUND_BUDGET_S = 30 * 60.

# Yesterday's runs, read-only. They were produced with the frozen OMP_NUM_THREADS=2, so the
# A arms are EXPECTED to differ from them; the comparison is reported, never judged.
REHEARSAL = Path('${FPGA_SRC}/runs/rtabmap-nav2-rehearsal-20260914')
REFERENCES = {'yesterday-baseline': REHEARSAL / 'baseline' / PROBE_NAME / 'observations.jsonl',
              'yesterday-loaded': REHEARSAL / 'loaded' / PROBE_NAME / 'observations.jsonl'}


# --------------------------------------------------------------------------- pure helpers

def arm_argv(arm):
    """The probe argv for one arm: the frozen baseline argv plus this arm's overrides."""
    spec = ARMS[arm]
    return [*probe_args('baseline'), '--omp-num-threads', str(spec['omp']), *spec['params']]


def arm_environment(arm):
    """The container-level environment overrides for one arm.

    The frozen arm (B1) gets none: its container has to differ from yesterday's baseline in
    nothing at all, and the probe already writes OMP_NUM_THREADS=2 by itself.
    """
    spec = ARMS[arm]
    if spec['omp'] == FROZEN_OMP_NUM_THREADS:
        return []
    return ['--env', 'OMP_NUM_THREADS=' + str(spec['omp'])]


def create_command(name, arm, stage):
    """The `docker create` argv. Same mounts and bounds as yesterday's rehearsal."""
    return [*common_create(name),
            *arm_environment(arm),
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
            'python3', '/probe_tools/rtabmap_ros_probe.py', *arm_argv(arm)]


def arms_to_run(executed, elapsed_s, a_identical):
    """§3 and §6: which arm, if any, runs next. Pure, so the schedule is unit testable.

    A skipped arm never blocks a later one: if the C arms are not licensed (A1 == A2, or the
    A pair is not comparable at all) B1 is still considered, because it answers a different
    question.
    """
    if elapsed_s >= ROUND_BUDGET_S:
        return None
    for arm in ARM_ORDER:
        if arm in executed:
            continue
        when = ARMS[arm]['when']
        if when == 'always':
            return arm
        if when == 'a-differs' and a_identical is False:
            return arm
        if when == 'time-permitting' and elapsed_s < B1_ELAPSED_LIMIT_S:
            return arm
    return None


def peak_threads(telemetry_path):
    """Highest thread count each probe child ever reached: did the OMP knob bite?"""
    peaks = {}
    for row in compare.read_events(telemetry_path):
        for entry in row.get('processes') or []:
            name = entry.get('name')
            peaks[name] = max(peaks.get(name, 0), int(entry.get('threads') or 0))
    return peaks


def timing_readout(stream_path):
    """RTAB-Map's own per-frame registration time. Informational only (§4)."""
    values = [row['time_estimation'] for row in compare.read_events(stream_path)
              if row.get('event') == 'odom_info' and row.get('time_estimation') is not None]
    return compare.percentiles(values)


def pair_names(executed):
    """Every unordered pair of executed arms, in ARM_ORDER."""
    ordered = [arm for arm in ARM_ORDER if arm in executed]
    return [(left, right) for index, left in enumerate(ordered) for right in ordered[index + 1:]]


# --------------------------------------------------------------------------- side effects

def load1():
    return float(Path('/proc/loadavg').read_text().split()[0])


def observe_arm(arm, stage, record):
    """Everything read off one finished arm's products. No judgement here."""
    products = stage / PROBE_NAME
    record['products'] = str(products)
    record['products_present'] = products.is_dir()
    runner_result = products / 'runner-result.json'
    record['runner_result'] = (json.loads(runner_result.read_text())
                               if runner_result.is_file() else None)
    record['config_identity'] = config_identity(products) if products.is_dir() else None
    record['config_identity_matches_frozen'] = bool(record['config_identity']) and all(
        entry['identical'] for entry in record['config_identity'].values())
    record['config_identity_expected_to_match'] = not ARMS[arm]['params']
    environment = products / 'environment.json'
    record['environment'] = json.loads(environment.read_text()) if environment.is_file() else None
    telemetry = products / 'process-telemetry.jsonl'
    record['peak_threads'] = peak_threads(telemetry) if telemetry.is_file() else None
    observations = products / 'observations.jsonl'
    record['observations'] = str(observations) if observations.is_file() else None
    record['time_estimation_s'] = timing_readout(observations) if observations.is_file() else None
    events = products / 'gate-events.jsonl'
    record['approved_count'] = len(approved_stamps(events)) if events.is_file() else None
    if ARMS[arm]['params']:
        record['parameter_overrides'] = ARMS[arm]['params']
        record['parameter_files_sha256'] = {
            name: sha256(products / name) for name in ('odom.yaml', 'slam.yaml')
            if (products / name).is_file()}
    return record


def run_arm(arm, root, prefix, summary):
    stage = root / arm
    name = f'{prefix}-{arm}'
    staged = root / 'database-input' / PROBE_NAME / 'rtabmap.db'
    waits = []
    record = {'arm': arm, 'container': name, 'probe_args': arm_argv(arm),
              'container_env': arm_environment(arm), 'load1_waits': waits}
    record['load1_at_start'] = wait_for_load(waits)
    record['database_sha256_before'] = sha256(staged)
    host_identity = dict(summary['identity'], arm=arm, probe_args=arm_argv(arm))
    started = time.time()
    try:
        record['container_result'] = run_container(name, create_command(name, arm, stage),
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
    record['database_unchanged'] = (record['database_sha256_before']
                                    == record['database_sha256_after'])
    record['container_exit_code'] = (record['container_result'] or {}).get('container_exit_code')
    record['timed_out'] = (record['container_result'] or {}).get('timed_out')
    observe_arm(arm, stage, record)
    record['comparable'] = bool(record['observations']) and record['container_exit_code'] == 0
    return record


def execute(root, prefix):
    identity = {'head_commit': head_commit(), 'image': IMAGE, 'image_id': image_id(),
                'source_sha256': dict(source_hashes(),
                                      rng_runner=sha256(CHECKOUT / 'tools'
                                                        / 'rtabmap_rng_source_run.py'),
                                      content_compare=sha256(CHECKOUT / 'tools'
                                                             / 'rtabmap_content_compare.py')),
                'protocol': PROTOCOL, 'protocol_sha256': sha256(CHECKOUT / PROTOCOL),
                'probe_run_name': PROBE_NAME, 'products_layout': '<ROOT>/<arm>/' + PROBE_NAME,
                'database_sha256': DATABASE_SHA256, 'dataset': str(DATASET),
                'frozen_run': str(FROZEN_RUN), 'frozen_baseline_argv': probe_args('baseline'),
                'references': {key: str(path) for key, path in REFERENCES.items()}}
    summary = {'schema': 1, 'protocol': PROTOCOL,
               'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
               'load1_at_round_start': load1(), 'identity': identity, 'arms': {}, 'order': []}
    staged = stage_database(root)
    summary['database_input'] = {'path': str(staged), 'sha256': sha256(staged)}

    streams, executed = {}, []
    a_identical = None
    round_started = time.time()
    while True:
        arm = arms_to_run(executed, time.time() - round_started, a_identical)
        if arm is None:
            break
        record = run_arm(arm, root, prefix, summary)
        summary['arms'][arm] = record
        summary['order'].append(arm)
        executed.append(arm)
        if record['observations']:
            streams[arm] = compare.load_stream(record['observations'])
        if {'a1', 'a2'} <= set(streams):
            a_identical = compare.compare_streams(streams['a1'], streams['a2'])['content_identical']
        (root / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False) + '\n')
    summary['elapsed_seconds'] = time.time() - round_started

    for key, path in REFERENCES.items():
        if path.is_file():
            streams[key] = compare.load_stream(path)
    pairs = {}
    for left, right in pair_names(executed):
        if left in streams and right in streams:
            pairs[f'{left}|{right}'] = compare.compare_streams(streams[left], streams[right])
    for arm in executed:
        for key in REFERENCES:
            if arm in streams and key in streams:
                pairs[f'{arm}|{key}'] = compare.compare_streams(streams[arm], streams[key])
    summary['pairs'] = pairs

    def identical(left, right):
        entry = pairs.get(f'{left}|{right}')
        return None if entry is None else entry['content_identical']

    within = {key: entry for key, entry in pairs.items()
              if not any(key.endswith('|' + reference) for reference in REFERENCES)}
    prerequisite_ok = bool(within) and all(entry['input_identical'] for entry in within.values())
    summary['prerequisite'] = {
        'rule': '§4: every within-round pair must have identical published image hashes',
        'pairs_checked': len(within), 'ok': prerequisite_ok,
        'failed_pairs': sorted(key for key, entry in within.items()
                               if not entry['input_identical'])}
    summary['a_pair_identical'] = identical('a1', 'a2')
    summary['c_pair_identical'] = identical('c1', 'c2')
    code, candidates, reason = compare.verdict(summary['a_pair_identical'],
                                               summary['c_pair_identical'], prerequisite_ok)
    summary['verdict'] = {'code': code, 'candidates': candidates, 'reason': reason}
    summary['finished_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
    summary['load1_at_round_end'] = load1()
    summary['leftover_containers'] = docker(
        'ps', '-a', '--filter', f'name={prefix}', '--format', '{{.Names}}',
        check=False).stdout.strip().splitlines()
    (root / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False) + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--container-prefix', type=container_prefix, default=CONTAINER_PREFIX)
    parser.add_argument('--print-plan', action='store_true',
                        help='print each arm\'s docker argv and exit; starts nothing')
    args = parser.parse_args()
    root = args.output.resolve()
    if args.print_plan:
        for arm in ARM_ORDER:
            print(json.dumps({'arm': arm, 'when': ARMS[arm]['when'],
                              'argv': create_command(f'{args.container_prefix}-{arm}', arm,
                                                     root / arm)}, indent=2))
        return
    if root.exists():
        parser.error('output root already exists: ' + str(root))
    root.mkdir(parents=True)
    summary = execute(root, args.container_prefix)
    manifest = subprocess.run(['bash', str(CHECKOUT / 'tools' / 'manifest.sh'), str(root)],
                              text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              timeout=1800, env=dict(os.environ))
    print(manifest.stdout.strip())
    print(json.dumps(summary['verdict'], indent=2, ensure_ascii=False))
    print('RNG_SOURCE_DONE ' + summary['verdict']['code'])


if __name__ == '__main__':
    main()
