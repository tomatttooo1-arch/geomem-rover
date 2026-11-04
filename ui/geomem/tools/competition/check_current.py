#!/usr/bin/env python3
"""Read-only liveness and identity check for a reusable Nano car session."""
import argparse
import json
from pathlib import Path
import re
import socket
import subprocess
import sys


def check_current(current, hub_port):
    """Inspect Docker and open a TCP connection; never start or stop anything."""
    current = Path(current)
    try:
        run = current.resolve(strict=True)
        if not (run / 'READY').is_file():
            raise RuntimeError('READY is missing')
        token = (run / 'token.txt').read_bytes()
        if not token.strip():
            raise RuntimeError('token.txt is empty')
        car = json.loads((run / 'car.json').read_text(encoding='utf-8'))
        stack = 'wde-car-stack-' + run.name
        if not re.fullmatch(r'[A-Za-z0-9_-]+', run.name) or car.get('tag') != run.name:
            raise RuntimeError('car.json run identity does not match the current directory')
        if car.get('containers', {}).get('stack') != stack:
            raise RuntimeError('car.json stack identity does not match the current run')
        hub = car.get('hub', {})
        if hub.get('bind') != '127.0.0.1' or type(hub.get('port')) is not int or hub['port'] != hub_port:
            raise RuntimeError('the recorded hub does not match the direct loopback tunnel on port %d' % hub_port)
        container_id = (run / ('container-' + stack + '.id')).read_text().strip()
        if not re.fullmatch(r'[0-9a-f]{64}', container_id):
            raise RuntimeError('the recorded container ID is missing or invalid')
        result = subprocess.run(['docker', 'inspect', '--format', '{{json .}}', stack], check=True,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=8)
        info = json.loads(result.stdout)
        if info.get('Id') != container_id or info.get('Name') != '/' + stack:
            raise RuntimeError('Docker container identity differs from the recorded run')
        state = info.get('State', {})
        if state.get('Running') is not True or state.get('Paused') or state.get('Restarting'):
            raise RuntimeError('the recorded stack container is not running (status: %s)' % state.get('Status', 'unknown'))
        if info.get('HostConfig', {}).get('NetworkMode') != 'host':
            raise RuntimeError('the stack is not using the Nano host network')
        mounts = [mount for mount in info.get('Mounts', []) if mount.get('Destination') == '/o']
        if len(mounts) != 1 or Path(mounts[0].get('Source', '')).resolve() != run:
            raise RuntimeError('the stack /o mount is not the current run directory')
        try:
            with socket.create_connection(('127.0.0.1', hub_port), timeout=2):
                pass
        except OSError as exc:
            raise RuntimeError('the recorded stack hub is unreachable: ' + str(exc)) from exc
        if current.resolve(strict=True) != run or (run / 'token.txt').read_bytes() != token:
            raise RuntimeError('the current session changed during the check; retry after inspecting it')
        return {'run': str(run), 'container': stack, 'hub': '127.0.0.1:%d' % hub_port}
    except (OSError, ValueError, AttributeError, TypeError, subprocess.SubprocessError, RuntimeError) as exc:
        raise RuntimeError('Existing car session is stale or incomplete: %s. '
                           'Evidence was preserved; inspect the recorded run and explicitly stop/collect it before starting again.' % exc) from exc


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--current', default='/data/fpga/car/state/car.current')
    parser.add_argument('--hub-port', type=int, required=True)
    args = parser.parse_args(argv)
    if not 1024 <= args.hub_port <= 65535:
        parser.error('hub port must be between 1024 and 65535')
    try:
        print(json.dumps(check_current(args.current, args.hub_port), sort_keys=True))
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 4
    return 0


if __name__ == '__main__':
    sys.exit(main())
