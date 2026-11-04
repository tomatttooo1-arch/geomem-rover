#!/usr/bin/env python3
"""Container-only bounded ROS probe. Every run owns its processes and DB copy.

The argv of every child is built by `plan_commands`, which is pure: it reads the parsed
arguments and returns the process names, their command lines and the replay command, with
no filesystem writes. `main` keeps the side effects. That split exists so the frozen
51/230 configuration can be pinned byte-for-byte against its recorded `commands.json` on a
host with no ROS and no container.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import yaml

# The joint-rehearsal extras all publish onto the REAL /tf. Both rtabmap nodes keep
# `-r /tf:=/fpga/localization_raw_tf`, so nothing published here can flow back into them.
STATIC_TRANSFORM_PUBLISHER = '/opt/ros/humble/lib/tf2_ros/static_transform_publisher'
EXTRINSICS_PROVENANCES = ('modelled_sim', 'measured')
RECORDER_SCRIPT = '/probe_tools/tf_topic_recorder.py'
RECORDER_OUTPUT = 'tf-record.jsonl'


def load_static_extrinsics(path):
    """Read a base->tracked extrinsics manifest for the extra static_transform_publisher.

    Provenance is validated here rather than trusted: an unlabelled manifest would put
    numbers of unknown origin on /tf_static, where the adapter reads them as the one true
    extrinsic.
    """
    manifest = json.loads(Path(path).read_text())
    if manifest.get('schema') != 1:
        raise ValueError('unsupported extrinsics manifest schema')
    if manifest.get('provenance') not in EXTRINSICS_PROVENANCES:
        raise ValueError('extrinsics provenance must be one of ' + repr(EXTRINSICS_PROVENANCES))
    entry = manifest['base_to_tracked']
    xyz, rpy = [float(v) for v in entry['xyz']], [float(v) for v in entry['rpy']]
    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError('base_to_tracked needs three-element xyz and rpy')
    frames = manifest.get('frames') or {}
    return xyz, rpy, frames.get('base', 'base_link'), frames.get('tracked', 'camera_link')


def static_extrinsics_command(path):
    """argv publishing base->tracked on /tf_static, taken only from the manifest."""
    xyz, rpy, base, tracked = load_static_extrinsics(path)
    argv = [STATIC_TRANSFORM_PUBLISHER]
    for flag, value in zip(('--x', '--y', '--z', '--roll', '--pitch', '--yaw'), xyz + rpy):
        argv += [flag, repr(value)]
    return argv + ['--frame-id', base, '--child-frame-id', tracked]


def identity_static_command(spec):
    """argv publishing an all-zero PARENT:CHILD edge on /tf_static.

    A negative-control aid only: a static identity for an edge that should be dynamic is
    exactly the silent failure the interface check has to catch, so the rehearsal needs a
    way to produce one on purpose.
    """
    parent, _, child = spec.partition(':')
    if not parent or not child or ':' in child:
        raise ValueError('extra static identity must be PARENT:CHILD')
    return ([STATIC_TRANSFORM_PUBLISHER, '--x', '0', '--y', '0', '--z', '0',
             '--roll', '0', '--pitch', '0', '--yaw', '0',
             '--frame-id', parent, '--child-frame-id', child],
            'static_tf_extra_%s_%s' % (frame_label(parent), frame_label(child)))


def frame_label(frame):
    """Frame id reduced to something that can be a process name and a log filename."""
    return ''.join(character if character.isalnum() else '_' for character in frame.strip('/'))


def parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('name')
    p.add_argument('--mode', choices=['normal', 'occluded', 'empty', 'empty-writable', 'sync', 'smoke'], default='normal')
    p.add_argument('--reset-countdown',type=int,default=0)
    p.add_argument('--odom-tf-wait',type=float,default=.1)
    p.add_argument('--online-gate', action='store_true', help='isolate raw TF and run acquisition gate against the copied prior')
    p.add_argument('--gate-rgb-reliable', action='store_true', help='explicit reliable acquisition subscriber for the gate')
    p.add_argument('--input', default='/normal/rgbd_dataset_freiburg1_desk')
    p.add_argument('--start-index', type=int, default=150)
    p.add_argument('--stop-index', type=int, default=330)
    p.add_argument('--timeout-seconds', type=float, default=600)
    p.add_argument('--repeat-duration-seconds', type=float, default=0.)
    p.add_argument('--repeat-cycles', type=int, default=0)
    p.add_argument('--allocator-probe', type=Path, help='explicit diagnostic-only preload library')
    p.add_argument('--heaptrack', action='store_true', help='trace only the localization process allocations')
    p.add_argument('--memory-pause-after-cycle', type=int, default=7)
    p.add_argument('--gate-diagnostics', action='store_true')
    p.add_argument('--gate-arg', action='append', default=[], metavar='ARG',
                   help='extra gate argument inserted before its --ros-args; repeatable. Use '
                        "--gate-arg=--flag form for arguments that start with a dash")
    p.add_argument('--gate-remap', action='append', default=[], metavar='FROM:=TO',
                   help='gate remapping appended as -r FROM:=TO after --ros-args; repeatable')
    p.add_argument('--adapter', action='store_true',
                   help='run host.ros_odom_base_adapter, the only odom->base_link and /odom publisher')
    p.add_argument('--static-extrinsics', type=Path,
                   help='extrinsics manifest; publishes base->tracked on /tf_static for the adapter')
    p.add_argument('--recorder', action='store_true',
                   help='run the TF/odom/clock/status recorder for the static interface check')
    p.add_argument('--extra-static-identity', action='append', default=[], metavar='PARENT:CHILD',
                   help='negative control: publish an all-zero PARENT:CHILD edge on /tf_static; '
                        'repeatable')
    p.add_argument('--omp-num-threads', type=int, default=2, metavar='N',
                   help="OMP_NUM_THREADS for every child process. The default 2 is the frozen "
                        "query's value; a container-level --env cannot reach the children "
                        'because this probe writes the variable itself')
    p.add_argument('--odom-param', action='append', default=[], metavar='KEY=VALUE',
                   help='diagnostic-only override written into odom.yaml; repeatable. Any use '
                        'makes the produced odom.yaml differ from the frozen query by design')
    p.add_argument('--slam-param', action='append', default=[], metavar='KEY=VALUE',
                   help='diagnostic-only override written into slam.yaml; repeatable')
    return p


def parse_param_override(spec):
    """`Group/Name=value` -> (key, value). RTAB-Map parameters are strings in the yaml.

    Validated rather than trusted: a silently malformed key would land in the parameter file
    as a dead entry that neither rtabmap nor this probe complains about.
    """
    key, sep, value = str(spec).partition('=')
    if not sep or not key or not value:
        raise ValueError('parameter override must be KEY=VALUE: ' + repr(spec))
    if not all(part and part.replace('_', '').isalnum() for part in key.split('/')):
        raise ValueError('parameter override key must be Group/Name: ' + repr(key))
    return key, value


def apply_param_overrides(params, specs):
    """Return `params` with every `KEY=VALUE` override applied, as strings. Pure."""
    result = dict(params)
    for spec in specs:
        key, value = parse_param_override(spec)
        result[key] = value
    return result


def parameter_sets(a, db):
    """The two ros__parameters dicts written as odom.yaml / slam.yaml. Pure.

    Split out of `main` so the frozen query's parameter files can be reproduced byte-for-byte
    in a unit test without a container: that is the known answer proving the new overrides
    are a no-op when nobody asks for them.
    """
    common = {'use_sim_time': True, 'frame_id': 'camera_link', 'approx_sync': False,
              'queue_size': 10, 'sync_queue_size': 10, 'topic_queue_size': 10,
              'qos': 1, 'qos_camera_info': 1}
    odom = dict(common, odom_frame_id='odom', publish_tf=True, publish_null_when_lost=True)
    odom['Odom/ResetCountdown'] = str(a.reset_countdown)
    odom['wait_for_transform'] = a.odom_tf_wait
    slam = dict(common, database_path=str(db), subscribe_depth=True,
                subscribe_odom_info=True, publish_tf=True, map_frame_id='map')
    slam.update({'Mem/IncrementalMemory': 'false', 'Mem/InitWMWithAllNodes': 'true',
                 'Mem/LocalizationReadOnly': 'true', 'RGBD/StartAtOrigin': 'true',
                 'Rtabmap/DetectionRate': '0'})
    if a.mode == 'empty-writable':
        slam['Mem/LocalizationReadOnly'] = 'false'
    return (apply_param_overrides(odom, a.odom_param),
            apply_param_overrides(slam, a.slam_param))


def child_environment(a, base):
    """The environment every child process gets, from `base` (normally os.environ). Pure."""
    environment = dict(base)
    environment.update(ROS_DOMAIN_ID='82', ROS_LOCALHOST_ONLY='1',
                       OMP_NUM_THREADS=str(a.omp_num_threads), OPENBLAS_NUM_THREADS='1')
    return environment


def validate(a):
    if a.repeat_duration_seconds and (not a.online_gate or not 0 < a.repeat_duration_seconds <= 1800
                                    or a.timeout_seconds < a.repeat_duration_seconds+60):
        raise ValueError('repeat requires gate, duration <=1800s and timeout >= duration+60s')
    if not 0 <= a.start_index < a.stop_index or not 0 < a.timeout_seconds <= 3600:
        raise ValueError('invalid playback bounds or timeout')
    if a.gate_rgb_reliable and not a.online_gate:
        raise ValueError('--gate-rgb-reliable requires --online-gate')
    if (a.repeat_cycles and not a.repeat_duration_seconds) or a.repeat_cycles<0:
        raise ValueError('cycle limit requires repeat duration')
    if a.allocator_probe and (not a.allocator_probe.is_file() or not 0 < a.memory_pause_after_cycle < a.repeat_cycles):
        raise ValueError('invalid allocator protocol')
    if a.heaptrack and not a.allocator_probe:
        raise ValueError('heaptrack requires memory accounting for attribution')
    if a.reset_countdown<0: raise ValueError('negative reset countdown')
    if not 0 <= a.odom_tf_wait <= 1: raise ValueError('invalid TF wait')
    if not a.name.replace('-', '').replace('_', '').isalnum():
        raise ValueError('invalid run name')
    for flag, value in (('--gate-arg', a.gate_arg), ('--gate-remap', a.gate_remap),
                        ('--adapter', a.adapter), ('--static-extrinsics', a.static_extrinsics),
                        ('--recorder', a.recorder),
                        ('--extra-static-identity', a.extra_static_identity)):
        if value and not a.online_gate:
            raise ValueError(flag+' requires --online-gate')
    for spec in a.extra_static_identity:
        identity_static_command(spec)
    for remap in a.gate_remap:
        parts = remap.split(':=')
        if len(parts) != 2 or not all(part and not part.isspace() for part in parts):
            raise ValueError('gate remap must be FROM:=TO')
    if not 1 <= a.omp_num_threads <= 64:
        raise ValueError('OMP_NUM_THREADS must be between 1 and 64')
    for spec in list(a.odom_param) + list(a.slam_param):
        parse_param_override(spec)


def plan_commands(a, out, db, db_hash):
    """Pure argv construction: (names, commands, replay).

    Nothing is written and nothing but an explicitly requested extrinsics manifest is read,
    so the whole command plan can be pinned in a unit test without ROS or a container.
    """
    out, db = Path(out), Path(db)
    remaps = ['-r', 'rgb/image:=/probe/rgb', '-r', 'depth/image:=/probe/depth',
              '-r', 'rgb/camera_info:=/probe/camera_info', '-r', 'odom:=/probe/odom',
              '-r', 'odom_info:=/probe/odom_info']
    if a.online_gate:
        remaps += ['-r', '/tf:=/fpga/localization_raw_tf']
    commands = [
      ['/opt/ros/humble/lib/tf2_ros/static_transform_publisher', '--x', '0', '--y', '0', '--z', '0',
       '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
       '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame'],
      ['/opt/ros/humble/lib/rtabmap_odom/rgbd_odometry', '--ros-args', '--params-file', str(out/'odom.yaml'), *remaps],
      ['/opt/ros/humble/lib/rtabmap_slam/rtabmap', '--ros-args', '--params-file', str(out/'slam.yaml'),
       *remaps, '-r', 'info:=/probe/info', '-r', 'localization_pose:=/probe/localization_pose'],
    ]
    names = ['static_tf', 'odometry', 'localization']
    if a.online_gate:
        if a.mode in ('empty', 'empty-writable'):
            raise ValueError('online prior gate requires a nonempty pinned read-only map')
        commands.append(['python3', '-m', 'host.ros_localization_gate', '--database', str(db),
                         '--map-id', db_hash, '--events', str(out/'gate-events.jsonl'),
                         '--ros-args', '-p', 'use_sim_time:=true'])
        if a.gate_rgb_reliable:
            commands[-1].insert(commands[-1].index('--ros-args'), '--rgb-reliable')
        if a.gate_diagnostics:
            commands[-1].insert(commands[-1].index('--ros-args'), '--diagnostics')
        for argument in a.gate_arg:
            commands[-1].insert(commands[-1].index('--ros-args'), argument)
        for remap in a.gate_remap:
            commands[-1] += ['-r', remap]
        names.append('gate')
        # Joint-rehearsal extras. They publish on the real /tf, which both rtabmap nodes
        # have remapped away, so nothing here can feed back into the provider.
        if a.static_extrinsics:
            commands.append(static_extrinsics_command(a.static_extrinsics))
            names.append('static_tf_base_camera')
        for spec in a.extra_static_identity:
            command, name = identity_static_command(spec)
            commands.append(command)
            names.append(name)
        if a.adapter:
            adapter = ['python3', '-m', 'host.ros_odom_base_adapter', '--prefix', '/probe',
                       '--tracked-frame', 'camera_link', '--base-frame', 'base_link',
                       '--odom-frame', 'odom']
            if a.static_extrinsics:
                adapter += ['--extrinsics-manifest', str(a.static_extrinsics)]
            commands.append(adapter + ['--ros-args', '-p', 'use_sim_time:=true'])
            names.append('odom_adapter')
        if a.recorder:
            commands.append(['python3', RECORDER_SCRIPT, '--output', str(out/RECORDER_OUTPUT)])
            names.append('recorder')
    replay = ['python3', '/probe_tools/rtabmap_ros_replay.py', a.input,
              str(out/'observations.jsonl'), '--start', str(a.start_index), '--stop', str(a.stop_index), '--publish-clock']
    if a.online_gate:
        replay += ['--tf-topic', '/fpga/localization_raw_tf']
    if a.repeat_duration_seconds:
        replay += ['--repeat-duration-seconds', str(a.repeat_duration_seconds)]
    if a.repeat_cycles:
        replay += ['--repeat-cycles', str(a.repeat_cycles)]
    if a.allocator_probe:
        replay += ['--allocator-socket', str(out/'allocator/control.sock'),
                   '--memory-pause-after-cycle',str(a.memory_pause_after_cycle)]
    if a.heaptrack:
        commands[names.index('localization')]=['heaptrack','-o',str(out/'heaptrack'),*commands[names.index('localization')]]
    if a.mode in ('sync', 'smoke'):
        replay[replay.index('--stop')+1] = '155'
    if a.mode == 'sync':
        replay += ['--depth-stamp-offset-s', '0.1', '--feedback-timeout', '1', '--no-wait-slam']
    if a.mode == 'occluded':
        replay += ['--blank-start', '240', '--blank-stop', '270']
    if a.mode in ('empty','empty-writable'):
        replay += ['--no-wait-slam']
    return names, commands, replay


def main():
    a = parser().parse_args()
    validate(a)
    out = Path('/rosresults')/a.name
    out.mkdir(parents=True, exist_ok=False)
    db = out/'map.db'
    source = Path('/results/normal/rtabmap.db')
    if a.mode in ('empty','empty-writable'):
        source = Path('/results/blank/rtabmap.db')
    shutil.copyfile(source, db)
    db_hash = hashlib.sha256(db.read_bytes()).hexdigest()
    odom, slam = parameter_sets(a, db)
    for name, params in [('odom', odom), ('slam', slam)]:
        (out/f'{name}.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    environment = child_environment(a, os.environ)
    (out/'environment.json').write_text(json.dumps({k:environment[k] for k in
        ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','OMP_NUM_THREADS','OPENBLAS_NUM_THREADS']}, indent=2))
    if a.allocator_probe:
        (out/'allocator').mkdir()
        (out/'memory-maps').mkdir()
    names, commands, replay = plan_commands(a, out, db, db_hash)
    (out/'commands.json').write_text(json.dumps({'nodes':commands,'replay':replay}, indent=2))
    processes, logs = [], []
    result = {'mode':a.mode, 'input_database_sha256':db_hash}
    try:
        actual_commands=[]
        for name, cmd in zip(names, commands):
            stream = (out/f'{name}.log').open('w'); logs.append(stream)
            process_environment=environment.copy()
            if name=='localization' and a.allocator_probe:
                # heaptrack prepends its library to an existing LD_PRELOAD. Its
                # child must be the ELF target, not a shell that execs afterward.
                process_environment.update(LD_PRELOAD=str(a.allocator_probe),
                    FPGA_ALLOCATOR_DIR=str(out/'allocator'),
                    FPGA_ALLOCATOR_EXECUTABLE='/opt/ros/humble/lib/rtabmap_slam/rtabmap')
            actual_commands.append({'name':name,'argv':cmd,'diagnostic_environment':
                {k:process_environment[k] for k in ('LD_PRELOAD','FPGA_ALLOCATOR_DIR','FPGA_ALLOCATOR_EXECUTABLE') if k in process_environment}})
            processes.append(subprocess.Popen(cmd, stdout=stream, stderr=subprocess.STDOUT,
                                              env=process_environment, start_new_session=True))
        (out/'actual-commands.json').write_text(json.dumps(actual_commands,indent=2)+'\n')
        time.sleep(2)
        with (out/'replay.log').open('w') as stream:
            replay_process = subprocess.Popen(replay, stdout=stream, stderr=subprocess.STDOUT,
                                              env=environment, start_new_session=True)
            processes.append(replay_process)
            deadline=time.monotonic()+a.timeout_seconds
            telemetry = (out/'process-telemetry.jsonl').open('x', buffering=1)
            logs.append(telemetry)
            next_sample = 0.
            memory_sample=0
            if a.allocator_probe:
                detailed=(out/'memory-telemetry.jsonl').open('x',buffering=1); logs.append(detailed)
            while replay_process.poll() is None:
                if time.monotonic() >= next_sample:
                    sample = []
                    for name, proc in zip(names+['replay'], processes):
                        try:
                            fields = dict(line.split(':', 1) for line in Path(f'/proc/{proc.pid}/status').read_text().splitlines() if ':' in line)
                            sample.append({'name': name, 'pid': proc.pid, 'rss_kib': int(fields.get('VmRSS', '0 kB').split()[0]),
                                           'threads': int(fields.get('Threads', '0'))})
                        except FileNotFoundError:
                            sample.append({'name': name, 'pid': proc.pid, 'exited': True})
                    telemetry.write(json.dumps({'monotonic_ns': time.monotonic_ns(), 'processes': sample})+'\n')
                    if a.allocator_probe:
                        if __package__:
                            from .rtabmap_memory_probe import telemetry as memory_telemetry
                        else:
                            from rtabmap_memory_probe import telemetry as memory_telemetry
                        owned=list(zip(names+['replay'],processes))
                        if a.heaptrack:
                            # The allocator thread exposes actual target PID via
                            # its listening process; obtain it from /proc children.
                            for root in list(Path('/proc').glob('[0-9]*')):
                                try:
                                    cmdline=(root/'cmdline').read_bytes().split(b'\0')
                                    if cmdline and cmdline[0]==b'/opt/ros/humble/lib/rtabmap_slam/rtabmap' and str(out/'slam.yaml').encode() in cmdline:
                                        maps=(root/'maps').read_text()
                                        if 'libheaptrack_preload.so' not in maps:
                                            raise RuntimeError('heaptrack absent from actual localization target')
                                        from types import SimpleNamespace
                                        owned.append(('localization_target',SimpleNamespace(pid=int(root.name))))
                                except (FileNotFoundError,ProcessLookupError,PermissionError):pass
                        detailed.write(json.dumps(memory_telemetry(owned,out/'memory-maps',memory_sample))+'\n')
                        memory_sample+=1
                    next_sample = time.monotonic()+1.
                if any(proc.poll() is not None for proc in processes[:-1]):
                    result['node_exited_during_replay']=True
                    break
                if time.monotonic()>deadline:
                    result['replay_timeout']=True
                    break
                time.sleep(.2)
            result['replay_exit_code']=replay_process.poll()
    except BaseException as exc:
        result['exception']={'type':type(exc).__name__,'message':str(exc)}
        raise
    finally:
        result['before_stop_returncodes'] = [proc.poll() for proc in processes]
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in processes:
            try: proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL); proc.wait(timeout=5)
        for stream in logs: stream.close()
        result['all_owned_processes_exited'] = all(proc.poll() is not None for proc in processes)
        result['database_sha256_after'] = hashlib.sha256(db.read_bytes()).hexdigest()
        result['readonly_database_unchanged'] = result['database_sha256_after'] == db_hash
        (out/'runner-result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
