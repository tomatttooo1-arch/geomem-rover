#!/usr/bin/env python3
"""Wait briefly for this running car session's latched runtime metadata."""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time


def wait_runtime_info(current, run, timeout_s=15.0, poll_s=0.25):
    if not math.isfinite(timeout_s) or timeout_s <= 0 or not math.isfinite(poll_s) or poll_s <= 0:
        raise ValueError('wait timeout and polling interval must be positive finite numbers')
    current, run = Path(current), Path(run).resolve(strict=True)
    if not re.fullmatch(r'[A-Za-z0-9_-]+', run.name):
        raise RuntimeError('Invalid car run directory name')
    container = 'wde-car-stack-' + run.name
    deadline = time.monotonic() + timeout_s
    reason = 'runtime_info.json has not been written'

    def same_session():
        try:
            matches = current.resolve(strict=True) == run
        except OSError:
            matches = False
        if not matches:
            raise RuntimeError('car.current changed or disappeared while waiting for runtime_info.json; no display feed was started')

    while True:
        same_session()
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError('Timed out after %.1f s waiting for runtime_info.json: %s. '
                               'The car stack was not restarted; inspect %s/runtime_info.log and retry attaching.' %
                               (timeout_s, reason, run))
        try:
            result = subprocess.run(['docker', 'inspect', '--format', '{{.State.Running}}', container],
                                    check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True, timeout=min(2.0, remaining))
        except (OSError, subprocess.SubprocessError) as exc:
            raise RuntimeError('Cannot verify the stack container while waiting for runtime_info.json: ' + str(exc)) from exc
        if result.stdout.strip() != 'true':
            raise RuntimeError('The stack container stopped while waiting for runtime_info.json; no display feed was started')
        try:
            data = json.loads((run / 'runtime_info.json').read_text(encoding='utf-8'))
            scope = data.get('runtime_boot_id') if isinstance(data, dict) else None
            # host.memory_publish.new_boot_id() returns os.urandom(8).hex().
            if isinstance(scope, str) and re.fullmatch(r'[0-9a-f]{16}', scope):
                same_session()
                if time.monotonic() < deadline:
                    return scope
                reason = 'runtime metadata arrived after the wait deadline'
            else:
                reason = 'runtime_boot_id is missing or is not a 16-character lowercase hexadecimal ID'
        except (OSError, ValueError) as exc:
            reason = 'runtime_info.json is not complete/readable yet (' + str(exc) + ')'
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(min(poll_s, remaining))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--current', required=True)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--timeout-seconds', type=float, default=15.0)
    args = parser.parse_args(argv)
    try:
        print(wait_runtime_info(args.current, args.run_dir, args.timeout_seconds))
    except (OSError, ValueError, RuntimeError) as exc:
        print('Runtime metadata wait stopped: ' + str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
