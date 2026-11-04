#!/usr/bin/env python3
"""Installed root-owned KV startup helper. Never replaces a running server.

sudo /usr/local/sbin/geomem-kv-start [v3fc2|v3fc2_166|v3fc3_187|v3fc|a2|m1_166|m1|m2_166|m2|stop]   (go-live 2026-10-26; m1_166/m1 SW_M1, m2_166/m2 SW_M2 2026-11-03)
  no argument = a2 (m7_pl_merge_a2, exactly what every earlier install did);
  v3fc2 / v3fc2_166 / v3fc3_187 / v3fc / a2 / m1_166 / m1 / m2_166 / m2 = that FPGA profile (tools/car_run/fpga_profiles.env in the installed snapshot);
  stop        = stop the running server of either profile (kv_board.sh stop <its run>).
  v3fc3_187 is paused (10-26, V3FC3_187_USE=v3fc2 in the env file): kv_start.sh's fpga_profile warns and loads
  m7_pl_v3fc2; the printed "profile" stays v3fc3_187, the served BUILD_ID is v3fc2's 0x03c087c9.
A running server of the other profile is never replaced: stop it first.

10-26: the car screen AR (D-042). Whether kv_start.sh is asked for the AR core comes ONLY from fixed root-owned
files, never from the caller: this entry point is run as ``sudo /usr/local/sbin/geomem-kv-start`` with no
arguments other than the FPGA profile word (sudoers pins the allowed argument lists, sudo resets the environment), so a caller cannot pass a switch.
  1. /etc/geomem-dev3-kv/ar.conf (administrator override without a redeploy), used only when it is a regular file
     owned by root and not group/world writable;
  2. else <snapshot>/ui/geomem/tools/competition/kv_ar.conf (part of the deployed commit);
  3. else drm.
Only the words ``drm`` and ``off`` are accepted; anything else is ignored with a warning (next source).
The printed JSON carries ``ar`` (what the running server really does), ``ar_requested`` and ``ar_reason``.
"""
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import time

REPO = Path('/opt/geomem-dev3-kv/source')
RUNS = Path('/var/lib/geomem-dev3-kv/runs')
BUILD = '0x00da5041'          # profile a2 (also the fallback when the snapshot has no profile file)
PROFILES_ENV = REPO / 'tools/car_run/fpga_profiles.env'
ETC_AR = Path('/etc/geomem-dev3-kv/ar.conf')
AR_WORDS = ('drm', 'off')


def _ar_word(path, need_root_owned):
    """('drm'|'off', None) from one switch file, or (None, why) when it is absent/untrusted/invalid."""
    try:
        st = os.lstat(str(path))
    except OSError:
        return None, None
    if not stat.S_ISREG(st.st_mode):
        return None, '%s is not a regular file' % path
    if need_root_owned and (st.st_uid != 0 or st.st_mode & 0o022):
        return None, '%s is not root-owned or is writable by others' % path
    try:
        words = [w for line in Path(path).read_text().splitlines()
                 for w in [line.split('#', 1)[0].strip()] if w]
    except (OSError, UnicodeError) as exc:
        return None, '%s unreadable: %s' % (path, exc)
    if len(words) != 1 or words[0] not in AR_WORDS:
        return None, '%s must hold exactly one of %s' % (path, '/'.join(AR_WORDS))
    return words[0], None


def ar_switch(repo=REPO, etc=ETC_AR):
    """(mode, source, warnings)."""
    warnings = []
    for path, root_only in ((etc, True), (Path(repo) / 'ui/geomem/tools/competition/kv_ar.conf', False)):
        word, why = _ar_word(path, root_only)
        if why:
            warnings.append(why)
        if word:
            return word, str(path), warnings
    return 'drm', 'default', warnings


def _ar_of(args, out):
    """What a running server really does: its own argv (--ar drm), plus kv_start.sh's ar_status.json."""
    ar = 'drm' if any(a == '--ar' and b == 'drm' for a, b in zip(args, args[1:])) else 'off'
    info = {'ar': ar}
    try:
        st = json.loads((out / 'ar_status.json').read_text())
        info['ar_requested'] = st.get('requested')
        info['ar_reason'] = st.get('reason')
        info['ar_mode'] = st.get('mode')
        info['ar_screen'] = st.get('screen')   # 10-27: "sink_count=N edid_bytes=M" (kv_start.sh), None before
    except (OSError, ValueError, AttributeError):
        info['ar_requested'] = None
        info['ar_reason'] = None if ar == 'drm' else 'no ar_status.json (server started before 10-26 or by hand)'
    return info


def load_profiles(path=PROFILES_ENV):
    """{'a2': {'app':..., 'build': '0x........', 'kv_rolling':...}, 'v3fc': {...}} from the env file."""
    kv = {}
    try:
        lines = Path(path).read_text().splitlines()
    except OSError:
        lines = []
    for line in lines:
        line = line.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, val = line.split('=', 1)
        kv[key.strip()] = val.strip()
    out = {'a2': {'app': 'm7_pl_merge_a2', 'build': BUILD, 'kv_rolling': 'none'}}
    for name in ('v3fc2', 'v3fc2_166', 'v3fc3_187', 'v3fc', 'a2', 'm1_166', 'm1', 'm2_166', 'm2'):
        p = name.upper()
        if p + '_BUILD_ID' in kv:
            out[name] = {'app': kv[p + '_APP'], 'build': '%#010x' % int(kv[p + '_BUILD_ID'], 16),
                         'kv_rolling': kv[p + '_KV_ROLLING']}
    return out


def running_server(proc=Path('/proc'), builds=None):
    """The one ready kv_server (its ready.json + run dir) or None. ``builds``: BUILD_IDs that
    count as ours (default the a2 one); any other identity is left untouched (RuntimeError)."""
    builds = {BUILD} if builds is None else set(builds)
    servers = []
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = (entry / 'cmdline').read_bytes().decode().strip('\0').split('\0')
        except (OSError, UnicodeError):
            continue
        if not any(a.endswith('/tools/kv_link/kv_server.py') for a in args):
            continue
        try:
            out = Path(args[args.index('--out') + 1])
            ready = json.loads((out / 'ready.json').read_text())
            if ready.get('pid') != int(entry.name) or ready.get('port') != 47800 or ready.get('build_id') not in builds:
                raise ValueError('KV readiness identity does not match the running process')
            servers.append(dict(ready, run=str(out), **_ar_of(args, out)))
        except (OSError, ValueError, IndexError) as exc:
            raise RuntimeError('An existing KV process is not ready; leaving it untouched: ' + str(exc))
    if len(servers) > 1:
        raise RuntimeError('Multiple KV servers exist; refusing to alter hardware')
    return servers[0] if servers else None


def announce(ready):
    """The visible AR line (stdout, before the JSON) and the JSON line nano_up.sh reads."""
    if ready.get('ar') == 'drm':
        print('AR_ENABLED run=%s colour_port=%s' % (ready.get('run'), ready.get('ar_colour_port')))
        scr = ready.get('ar_screen') or ''
        if 'sink_count=0' in scr or 'edid_bytes=0' in scr:   # 10-27: warning only (AR keeps running)
            print('AR_SCREEN_WARNING %s: the KV sees no screen behind the DP->HDMI converter, the panel likely '
                  'stays dark -- power the screen, replug its HDMI at the KV, restart the KV service' % scr)
    elif str(ready.get('ar_reason') or '').startswith('ar_core_absent'):
        # AR2NANO 2026-11-02: bitstream without the AR core -- the Nano draws the car screen (not a failure)
        print('AR_ON_NANO reason=%s' % ready.get('ar_reason'))
    else:
        print('AR_DISABLED reason=%s' % (ready.get('ar_reason') or 'unknown'))
    print(json.dumps(ready), flush=True)


def main():
    import fcntl
    arg = sys.argv[1] if len(sys.argv) == 2 else ''
    if len(sys.argv) > 2 or arg not in ('', 'v3fc2', 'v3fc2_166', 'v3fc3_187', 'v3fc', 'a2', 'm1_166', 'm1', 'm2_166', 'm2', 'stop') or os.geteuid() != 0:
        raise RuntimeError('Use sudo /usr/local/sbin/geomem-kv-start [v3fc2|v3fc2_166|v3fc3_187|v3fc|a2|m1_166|m1|m2_166|m2|stop] (no argument = a2)')
    profiles = load_profiles()
    builds = {p['build'] for p in profiles.values()}
    with open('/var/lib/geomem-dev3-kv/start.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        ready = running_server(builds=builds)
        if arg == 'stop':
            if ready:
                subprocess.run(['bash', str(REPO / 'tools/kv_link/kv_board.sh'), 'stop', ready['run']], check=True)
            print(json.dumps({'stopped': ready['run'] if ready else None}))
            return
        name = arg or 'a2'
        if name not in profiles:
            raise RuntimeError('FPGA profile %s is not in the installed snapshot (%s); reinstall with '
                               'install_kv_service.sh --replace' % (name, PROFILES_ENV))
        want = profiles[name]['build']
        if ready:
            # owner 10-26 (no new refusals): a running server of any registered profile is reused as it is;
            # the Nano precheck accepts every registered BUILD_ID. Switching = geomem-kv-start stop first.
            if ready.get('build_id') != want:
                print('KV server already running with BUILD_ID %s, not %s (profile %s): reusing it; '
                      'to switch: sudo /usr/local/sbin/geomem-kv-start stop' % (ready.get('build_id'), want, name),
                      file=sys.stderr)
            announce(dict(ready, reused=True, profile=name))
            return
        if not (REPO / 'COMMIT').is_file():
            raise RuntimeError('Installed source snapshot is missing')
        mode, source, warnings = ar_switch()
        for w in warnings:
            print('AR switch warning: ' + w, file=sys.stderr)
        RUNS.mkdir(parents=True, exist_ok=True)
        out = RUNS / (time.strftime('%Y%m%d-%H%M%S') + '-' + str(os.getpid()))
        sys.stdout.flush()
        cmd = ['bash', str(REPO / 'ui/geomem/tools/competition/kv_start.sh'), str(REPO), str(out), mode]
        subprocess.run(cmd + ([name] if arg else []), check=True)
        ready = running_server(builds=builds)     # kv_start may have fallen back to a2
        if not ready:
            raise RuntimeError('KV startup returned without a ready process; inspect ' + str(out))
        commit = (REPO / 'COMMIT').read_text().split()[0] if (REPO / 'COMMIT').is_file() else None
        announce(dict(ready, reused=False, profile=name, ar_switch=mode, ar_switch_source=source, commit=commit))


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print('KV startup stopped: ' + str(exc), file=sys.stderr)
        sys.exit(1)
