"""Select actual session-owned processes, never the bwrap command wrapper."""
import json
import os
from pathlib import Path


def read_argv(proc, pid):
    return (proc / str(pid) / 'cmdline').read_bytes().decode().rstrip('\0').split('\0')


def matches_role(argv, role):
    if not argv:
        return False
    python = Path(argv[0]).name in ('python3', 'python3.8', 'python3.10')
    if role == 'worker':
        return (python and len(argv) > 2 and
                argv[1] == '/fpga/robot_sim/tests/nav_point_to_point.py' and
                '--c-worker' in argv[2:])
    if role == 'relay':
        return python and argv[1:3] == ['-m', 'host.ros_tunnel'] and 'client' in argv[3:]
    if role == 'smoother':
        return argv[0].endswith('/nav2_velocity_smoother/velocity_smoother')
    raise ValueError('Unknown C process role: ' + role)


def owned_by_session(proc, pid, session):
    for _ in range(6):
        argv = read_argv(proc, pid)
        # Exact bound IPC directory, not a session substring in arbitrary arguments.
        if Path(argv[0]).name == 'bwrap' and any(
                x.endswith('/ipc/' + session + '-dds') for x in argv):
            return True
        status = (proc / str(pid) / 'status').read_text()
        pid = int(status.split('PPid:')[1].split()[0])
        if pid == 0:
            return False
    return False


def signal_role(session, role, sig, allow_missing=False, proc=Path('/proc'), send=os.kill):
    found = []
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        try:
            argv = read_argv(proc, pid)
            if matches_role(argv, role) and owned_by_session(proc, pid, session):
                found.append((pid, argv))
        except (OSError, ValueError, IndexError, UnicodeDecodeError):
            continue
    if not found and allow_missing:
        return {'missing': True, 'role': role, 'session': session}
    if len(found) != 1:
        raise RuntimeError('Expected one session-owned %s, found %r' % (role, found))
    pid, argv = found[0]
    if read_argv(proc, pid) != argv or not owned_by_session(proc, pid, session):
        raise RuntimeError('C process identity changed before signal')
    send(pid, sig)
    return {'pid': pid, 'argv': argv, 'signal': sig, 'session': session}
