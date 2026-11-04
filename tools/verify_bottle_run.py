#!/usr/bin/env python3
"""Offline audit of actual AGX scenario, acquisition hashes and simulator motion.

--layout sim (line D1, 2026-10-14): the server-only simulated run of tools/sim_loop/run_sim_loop.sh
(one run directory: scenario.jsonl, runtime.jsonl, frames/, motion.jsonl, pc.fpgarec, adapter and
producer summaries). Same scenario/evidence checks as the AGX layout, minus AGX, plus: x86_64,
the run's inference device (run_config.json; cuda = one host GPU, fp32, or cpu) on every processed
frame, query forwarded by the hub, SIM data source end to end, adapter output non-empty and
finite, producer fused frames. The AGX layout (default) is unchanged.

Line D2 (2026-10-14): --layout sim reads the detector from run_config.json (ssdlite: score >= 0.4 and the
D1 execution profile; yolo: the YOLO execution profile -- the run config's yolo_backend (torch = CUDA fp32
with TF32 off, ort = onnxruntime CPU), its model sha256, fp32, D-036 classes -- and score >= its conf 0.25) and requires runtime_info.json (latched /fpga/runtime_info) to say
that detector with detector_checked = true. With chassis "mcu" the simulator's command is the MCU stand-in's
executed speed (/fpga/chassis/sim_twist): non-zero during the run, zero at the end, and single_source.txt
shows exactly one publisher on Gazebo's /fpga/cmd_vel and on /fpga/chassis/sim_twist and no bridge on
/fpga/cmd_vel_safe. --layout sim-cut / sim-estop (chassis mcu): the permit / MCU stop chain from
permit_telem.jsonl, mcu.jsonl and motion.jsonl (tools/sim_loop/chain_times.py), bounds in the D2 PREREG.
"""
import argparse
import hashlib
import json
import math
import tarfile
from pathlib import Path

import numpy as np

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent / 'sim_loop'))
import chain_times as ct  # noqa: E402

# D2 stop-chain bounds (PREREG of sim-loop-d2-20261014, written before the formal runs)
PERMIT_TIMEOUT_S = 0.200       # host permit model: REVOKE = DONE + timeout + 1 PL cycle
MCU_HEARTBEAT_S = 0.031        # MCU guide section 2: no heartbeat for 30 ms -> latched in the 31st ms
COMMAND_PERIOD_S = 0.020       # one chassis command period (50 Hz); covers the heartbeat poll staleness
UNLOCK_LATE_S = 0.0            # B line: the unlock is a frame late -> only the resume, not the stop
CHAIN_BOUND_S = PERMIT_TIMEOUT_S + MCU_HEARTBEAT_S + COMMAND_PERIOD_S + UNLOCK_LATE_S   # 0.251
LATCH_AFTER_REVOKE_S = MCU_HEARTBEAT_S + COMMAND_PERIOD_S                              # 0.051
DIFF_DRIVE_ACCEL = (0.5, 1.0)  # room_carmount.sdf DiffDrive max linear / angular deceleration
STILL_ALLOWANCE_S = 0.1        # two 30 Hz odom periods + bridge / transport

FRCNN_CHECKPOINT_SHA256 = '258fb6c638b15964ddcdd1ae0748c5eef1be9e732750120cc857feed3faac384'
SOURCE_EVIDENCE_PATHS = (
    'tools/run_agx_bottle.py', 'host/ros_runtime.py',
    'robot_sim/fpga_sim/config/scanned_bottle_asset.json',
    'robot_sim/fpga_sim/worlds/room.sdf', 'tools/make_scanned_bottle_world.py',
)


def source_evidence(run):
    """Use saved source only; never follow provenance paths back to old hosts."""
    archived, extracted = {}, {}
    if (run/'source.tar').exists():
        with tarfile.open(run/'source.tar') as archive:
            for name in SOURCE_EVIDENCE_PATHS:
                try:
                    member = archive.getmember(name)
                except KeyError:
                    continue
                if member.isfile():
                    archived[name] = archive.extractfile(member).read()
    for name in SOURCE_EVIDENCE_PATHS:
        path = run/'source'/name
        if path.is_file():
            extracted[name] = path.read_bytes()
    return archived, extracted


# the service's default class whitelist: D-036's four (runs before 2026-10-21), D-044's first version (all
# COCO minus person, branch-only), D-044 revised (the 26 venue classes, runs of 10-21 / 10-22) or the venue
# classes without tv (25, from 10-22), in tools/nano_detector COCO order as YoloSegDetector reports it
YOLO_CLASS_SETS = (['bottle', 'cup', 'backpack', 'cell phone'],)


def _d044_default():
    root = str(Path(__file__).resolve().parents[1])
    if root not in sys.path:
        sys.path.append(root)
    from perception.fpga_replay.coco_zh import COCO_NAMES, VENUE_CLASSES, VENUE_CLASSES_D044_26   # == nano_detector.COCO
    return ([n for n in COCO_NAMES if n != 'person'], list(VENUE_CLASSES_D044_26),
            [n for n in COCO_NAMES if n in VENUE_CLASSES])


YOLO_CLASS_SETS += _d044_default()


def yolo_profile_ok(inference, device, execution):
    """D2: the YOLO execution the service reported matches the run config -- backend (torch = CUDA fp32 with
    TF32 off; ort = onnxruntime CPU), model sha256, fp32, conf 0.25, the default classes (D-036's four
    for runs before 2026-10-21, D-044's sets after)."""
    backend = inference.get('yolo_backend') or 'ort'          # D2 configs before the torch backend: ort
    model_sha = inference.get('yolo_model_sha256') or inference.get('yolo_onnx_sha256')
    execution = execution or {}
    backend_ok = ((backend == 'ort' and device == 'cpu') or
                  (backend == 'torch' and device == 'cuda' and execution.get('tf32') is False))
    return bool(execution and backend_ok and execution.get('device') == device
                and execution.get('backend') == backend and execution.get('precision') == 'fp32'
                and execution.get('conf') == 0.25 and bool(model_sha) and execution.get('model_sha256') == model_sha
                and execution.get('classes') in YOLO_CLASS_SETS)


def profile_checks(run, completed, config, runtime, processed, execution):
    archived, extracted = source_evidence(run)
    sources = list(archived.values())+list(extracted.values())
    explicit = ('detector' in completed or 'detector' in config
                or any('detector' in (row.get('inference_backend') or {}) for row in processed)
                or any(b'--detector' in source for source in sources))
    detector = completed.get('detector', 'ssdlite')
    checks = {
        'detector_profile_declared': (not explicit or (
            completed.get('detector') in ('ssdlite', 'fasterrcnn_resnet50_800')
            and config.get('detector') == completed.get('detector'))),
        'all_processed_detector_matches': bool(processed) and all(
            (row.get('inference_backend') or {}).get('detector') == detector
            if explicit else (row.get('inference_backend') or {}).get('detector', 'ssdlite') == 'ssdlite'
            for row in processed),
        'all_processed_cuda_profile': bool(processed) and all(
            (row.get('inference_backend') or {}).get('execution') == execution for row in processed),
        'detector_checkpoint_resize': detector != 'fasterrcnn_resnet50_800' or all(
            (row.get('inference_backend') or {}).get('checkpoint_sha256') == FRCNN_CHECKPOINT_SHA256
            and (row.get('inference_backend') or {}).get('resize') == {'min_size': 800, 'max_size': 1333}
            for row in processed),
        'detector_augmentation_compatible': detector != 'fasterrcnn_resnet50_800' or (
            completed.get('augmentation', 'none') == 'none' and config.get('augmentation', 'none') == 'none'),
        'saved_source_copies_match': all(archived[name] == extracted[name]
                                       for name in archived.keys() & extracted.keys()),
    }
    starts = [row for row in runtime if row.get('event') == 'startup']
    startup_required = (explicit or any(b'memory_tracks' in source for source in sources)
                        or any('memory_tracks' in row or 'task_active' in row or 'patrol' in row for row in starts))
    checks['startup_empty_memory_and_task'] = not startup_required or (
        len(starts) == 1 and starts[0].get('memory_tracks') == []
        and starts[0].get('task_active') is False and starts[0].get('patrol') == []
        and runtime.index(starts[0]) < next((i for i, row in enumerate(runtime)
                                           if row.get('event') == 'frame_processed'), -1))
    provenance_path = run/'scene-provenance.json'
    provenance = json.loads(provenance_path.read_text()) if provenance_path.exists() else {}
    scanned = (completed.get('scene') == 'scanned_bottle' or config.get('scene') == 'scanned_bottle'
               or provenance.get('scene') == 'scanned_bottle' or (run/'generated-room.sdf').exists())
    if scanned:
        manifest_name = 'robot_sim/fpga_sim/config/scanned_bottle_asset.json'
        manifest_bytes = archived.get(manifest_name)
        manifest = json.loads(manifest_bytes) if manifest_bytes else {}
        sha256 = lambda data: hashlib.sha256(data).hexdigest()
        world = run/'generated-room.sdf'
        checks['scanned_scene_declared'] = (completed.get('scene') == config.get('scene')
                                           == provenance.get('scene') == 'scanned_bottle')
        checks['scanned_world_hash_matches'] = (world.is_file()
            and provenance.get('output_world_sha256') == sha256(world.read_bytes()))
        checks['scanned_manifest_hash_matches_source'] = bool(manifest_bytes) and (
            sha256(manifest_bytes) == provenance.get('asset_manifest_sha256')
            == config.get('bottle_asset_manifest_sha256') == completed.get('bottle_asset_manifest_sha256'))
        checks['scanned_source_world_and_generator_match'] = all(
            name in archived and provenance.get(field) == sha256(archived[name])
            for name, field in [('robot_sim/fpga_sim/worlds/room.sdf', 'source_world_sha256'),
                                ('tools/make_scanned_bottle_world.py', 'generator_sha256')])
        expected_files = manifest.get('required_files_sha256')
        checks['scanned_asset_hash_map_matches'] = (isinstance(expected_files, dict) and bool(expected_files)
            and provenance.get('asset_files_sha256') == expected_files)
        checks['scanned_asset_identity_matches'] = bool(manifest) and all(
            provenance.get(field) == manifest.get(field) and field in manifest
            for field in ('asset_id', 'source_url', 'archive_sha256', 'license'))
    return checks, detector, not explicit


def read_rows(path):
    # 10-29: runtime rows carry the backend metadata in full only on first use / change (host/backend_log.py)
    root = str(Path(__file__).resolve().parents[1])
    if root not in sys.path:
        sys.path.append(root)
    from host.backend_log import expand_rows
    return expand_rows([json.loads(line) for line in path.read_text().splitlines()])


def verify(run):
    scenario = read_rows(run/'agx/scenario.jsonl')
    runtime = read_rows(run/'agx/runtime.jsonl')
    motion = read_rows(run/'motion.jsonl')
    checks = {}
    stages = {}
    for event in ('remembered', 'departed_out_of_view', 'query_requested', 'found_evidence', 'scenario_result'):
        matches = [r for r in scenario if r['event']==event]
        checks['one_'+event] = len(matches)==1
        if matches: stages[event]=matches[0]
    if not all(checks.values()):
        return {'passed': False, 'checks': checks}
    remembered, departed, query, found, result = (stages[k] for k in stages)
    target = query['target_id']
    processed = [r for r in runtime if r['event']=='frame_processed']
    frames = {r['frame_id']: r for r in processed}
    checks['unique_processed_frame_ids'] = len(frames) == len(processed)
    checks['ordered_acquisition_times'] = all(a['acquisition_timestamp'] < b['acquisition_timestamp']
                                            for a, b in zip(processed, processed[1:]))
    initial = remembered['frame']; outside = departed['frame']; final = found['frame']
    checks['stage_order'] = remembered['elapsed_wall'] < departed['elapsed_wall'] < query['elapsed_wall'] < found['elapsed_wall']
    checks['scenario_pass_and_stop'] = result['passed'] is True and result['stopped'] is True
    checks['actual_agx'] = scenario[0]['machine']=='aarch64'
    checks['same_target'] = found['task']['target_id']==target and any(t['id']==target and t['label']=='bottle' for t in initial['tracks'])
    checks['out_of_view'] = any(t['id']==target and t['state']=='historical_memory' and t['reason']=='out_of_view' for t in outside['tracks']) and not any(d['label']=='bottle' for d in outside['detections'])
    checks['fresh_after_arrival'] = final['acquisition_timestamp'] > found['arrived'] > outside['acquisition_timestamp']
    checks['fresh_same_track'] = any(t['id']==target and t['state']=='current_confirmed'
                                   and t['last_seen_frame']==final['frame_id']
                                   and t['last_seen']==final['acquisition_timestamp'] for t in final['tracks'])
    completed = json.loads((run/'completed.json').read_text())
    config = json.loads((run/'run-config.json').read_text()) if (run/'run-config.json').exists() else {}
    declared = completed.get('augmentation', 'none')
    revision = (run/'agx/source-revision.txt').read_text().strip()
    checks['source_revision_matches'] = completed.get('source_revision') == revision and bool(revision)
    execution = {'device':'cuda','memory_format':'contiguous','cuda_math':'fp32'}
    if declared != 'none': execution['augmentation'] = declared
    extra_checks, detector, legacy_detector = profile_checks(run, completed, config, runtime, processed, execution)
    checks.update(extra_checks)
    checks['real_cuda_detections'] = all(f['inference_backend']['execution']==
                                       execution
                                       and any(d['label']=='bottle' and d['score']>=.4 for d in f['detections']) for f in (initial,final))
    checks['scenario_frames_match_runtime'] = all(frames.get(f['frame_id'])==f for f in (initial,outside,final))
    checks['all_processed_frame_hashes'] = all(r.get('evidence') and
        hashlib.sha256((run/'agx/frames'/r['evidence']['file']).read_bytes()).hexdigest()==r['evidence']['sha256']
        for r in frames.values())
    raw_valid, raw_times, inference_times = [], [], []
    for row in processed:
        with np.load(run/'agx/frames'/row['evidence']['file'], allow_pickle=False) as data:
            rgb, depth = data['rgb'], data['depth']
            raw_valid.append(rgb.ndim == 3 and rgb.shape[2] == 3 and rgb.dtype == np.uint8
                             and depth.shape == rgb.shape[:2] and data['K'].shape == (3, 3)
                             and data['T_world_camera'].shape == (4, 4)
                             and np.isfinite(data['K']).all() and np.isfinite(data['T_world_camera']).all())
            raw_times.append(float(data['timestamp']) == row['acquisition_timestamp'])
        backend = row['inference_backend']
        inference_times.append(backend.get('frame_id') == row['frame_id']
                               and backend.get('timestamp') == row['acquisition_timestamp'])
    checks['raw_rgbd_geometry_valid'] = bool(raw_valid) and all(raw_valid)
    checks['raw_acquisition_times_match'] = bool(raw_times) and all(raw_times)
    checks['inference_frame_times_match'] = bool(inference_times) and all(inference_times)
    arrivals = [r for r in runtime if r['event']=='navigation_result' and r.get('command_id') and r.get('succeeded')]
    checks['actual_navigation_result'] = any(r['time']==found['arrived'] for r in arrivals)
    checks['navigation_start_result_link'] = any(
        r['time'] == found['arrived'] and not r.get('cancelled', False)
        and any(s['event'] == 'navigation_started' and s.get('command_id') == r['command_id']
                and outside['time'] < s['time'] < r['time'] for s in runtime)
        for r in arrivals)
    odometry = [r for r in motion if r['topic']=='/odom']
    xy = [(r['message']['pose']['pose']['position']['x'],r['message']['pose']['pose']['position']['y']) for r in odometry]
    displacement = max(math.dist(p,xy[0]) for p in xy)
    checks['simulator_actual_departure'] = displacement>=.65
    def odom_time(row):
        stamp = row['message']['header']['stamp']
        return stamp['sec'] + stamp['nanosec'] / 1e9
    departure_xy = [p for r, p in zip(odometry, xy)
                    if initial['acquisition_timestamp'] <= odom_time(r) <= outside['time']]
    checks['simulator_departure_before_query'] = bool(departure_xy) and max(
        math.dist(p, xy[0]) for p in departure_xy) >= .65
    safe = [r for r in motion if r['topic']=='/fpga/cmd_vel_safe']
    def moving(row):
        return abs(row['message']['linear']['x'])+abs(row['message']['angular']['z'])>.001
    checks['simulator_received_nonzero_commands'] = any(moving(r) for r in safe)
    tail = [r for r in safe if r['wall_monotonic'] >= safe[-1]['wall_monotonic']-1]
    checks['simulator_final_zero_commands'] = len(tail)>=5 and not any(moving(r) for r in tail)
    last_odom = odometry[-1]['message']['twist']['twist']
    checks['simulator_final_stopped'] = abs(last_odom['linear']['x'])+abs(last_odom['angular']['z'])<.01
    checks['no_runtime_fault'] = not any(r['event']=='fault' for r in runtime)
    return {'passed': all(checks.values()), 'checks': checks, 'source_revision': revision,
            'detector': detector, 'legacy_detector_evidence': legacy_detector,
            'target_id': target, 'initial_frame': initial['frame_id'], 'outside_frame': outside['frame_id'],
            'found_frame': final['frame_id'], 'arrival_sim_time': found['arrived'],
            'found_acquisition_sim_time': final['acquisition_timestamp'],
            'scenario_wall_seconds': result['elapsed_wall'], 'processed_frames': len(frames),
            'max_simulator_displacement_m': displacement,
            'scope': 'one scripted scene; AGX CUDA FP32; known map and simulated reference pose; no accuracy/performance claim'}


def pc_state_sources(path):
    """Source fields of every STATE frame in a PC recording (pc/fpgac.py), and the frame count."""
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'pc'))
    import fpgac
    _, rows = fpgac.read_fpgarec(str(path))
    sources = []
    for _, raw in rows:
        frame = fpgac.decode_one(raw)
        if frame.kind == fpgac.KIND['STATE']:
            sources.append(frame.obj().get('source'))
    return sources, len(rows)


def verify_sim(run):
    """Server simulation run (tools/sim_loop/run_sim_loop.sh bottle); one run directory."""
    scenario = read_rows(run/'scenario.jsonl')
    runtime = read_rows(run/'runtime.jsonl')
    motion = read_rows(run/'motion.jsonl')
    config = json.loads((run/'run_config.json').read_text())
    checks, stages = {}, {}
    for event in ('remembered', 'departed_out_of_view', 'query_requested', 'found_evidence', 'scenario_result'):
        matches = [r for r in scenario if r['event'] == event]
        checks['one_'+event] = len(matches) == 1
        if matches: stages[event] = matches[0]
    if not all(checks.values()):
        return {'passed': False, 'checks': checks, 'layout': 'sim'}
    remembered, departed, query, found, result = (stages[k] for k in stages)
    target = query['target_id']
    processed = [r for r in runtime if r['event'] == 'frame_processed']
    frames = {r['frame_id']: r for r in processed}
    checks['unique_processed_frame_ids'] = len(frames) == len(processed)
    checks['ordered_acquisition_times'] = all(a['acquisition_timestamp'] < b['acquisition_timestamp']
                                            for a, b in zip(processed, processed[1:]))
    initial = remembered['frame']; outside = departed['frame']; final = found['frame']
    checks['stage_order'] = remembered['elapsed_wall'] < departed['elapsed_wall'] < query['elapsed_wall'] < found['elapsed_wall']
    checks['scenario_pass_and_stop'] = result['passed'] is True and result['stopped'] is True
    start = scenario[0]
    checks['server_x86_64'] = start['event'] == 'scenario_start' and start['machine'] == 'x86_64'
    device = config['inference']['device']          # D1: cuda (host GPU, smoke-period change) or cpu
    checks['scenario_declared_device_hub'] = (device in ('cpu', 'cuda') and start.get('expect_device') == device
                                              and start.get('query_via') == 'hub')
    checks['same_target'] = found['task']['target_id'] == target and any(t['id'] == target and t['label'] == 'bottle' for t in initial['tracks'])
    checks['out_of_view'] = any(t['id'] == target and t['state'] == 'historical_memory' and t['reason'] == 'out_of_view' for t in outside['tracks']) and not any(d['label'] == 'bottle' for d in outside['detections'])
    checks['fresh_after_arrival'] = final['acquisition_timestamp'] > found['arrived'] > outside['acquisition_timestamp']
    checks['fresh_same_track'] = any(t['id'] == target and t['state'] == 'current_confirmed'
                                   and t['last_seen_frame'] == final['frame_id']
                                   and t['last_seen'] == final['acquisition_timestamp'] for t in final['tracks'])
    revision = (run/'head.txt').read_text().strip()
    checks['source_revision_clean'] = (bool(revision) and config.get('head') == revision
                                       and config.get('dirty') is False and (run/'dirty.txt').read_text() == '')
    detector = config['inference'].get('detector', 'ssdlite')
    augmentation = config['inference']['augmentation']
    if detector == 'yolo':
        execution = (processed[0].get('inference_backend') or {}).get('execution') if processed else None
        score_gate = (execution or {}).get('conf')
        checks['yolo_execution_profile'] = yolo_profile_ok(config['inference'], device, execution)
    else:
        execution = {'device': device, 'memory_format': 'contiguous', 'cuda_math': 'fp32' if device == 'cuda' else None}
        if augmentation != 'none':
            execution['augmentation'] = augmentation
        score_gate = .4
    checks['all_processed_device_profile'] = bool(processed) and all(
        (r.get('inference_backend') or {}).get('execution') == execution
        and (r.get('inference_backend') or {}).get('detector') == detector
        and r.get('inference_device') == device for r in processed)
    info_path = run/'runtime_info.json'
    info = json.loads(info_path.read_text()) if info_path.exists() else {}
    checks['runtime_info_detector_checked'] = (detector == 'ssdlite' and not info_path.exists()) or (
        info.get('detector') == detector and info.get('detector_checked') is True)
    checks['all_processed_sensor_simulation'] = bool(processed) and all(r.get('sensor_source') == 'simulation' for r in processed)
    starts = [r for r in runtime if r.get('event') == 'startup']
    checks['startup_empty_memory_and_task'] = (len(starts) == 1 and starts[0].get('memory_tracks') == []
        and starts[0].get('task_active') is False and starts[0].get('patrol') == []
        and runtime.index(starts[0]) < next((i for i, r in enumerate(runtime) if r.get('event') == 'frame_processed'), -1))
    checks['real_detections'] = all(f['inference_backend']['execution'] == execution
                                    and any(d['label'] == 'bottle' and d['score'] >= score_gate for d in f['detections'])
                                    for f in (initial, final))
    checks['scenario_frames_match_runtime'] = all(frames.get(f['frame_id']) == f for f in (initial, outside, final))
    checks['all_processed_frame_hashes'] = all(r.get('evidence') and
        hashlib.sha256((run/'frames'/r['evidence']['file']).read_bytes()).hexdigest() == r['evidence']['sha256']
        for r in frames.values())
    raw_valid, raw_times, inference_times = [], [], []
    for row in processed:
        with np.load(run/'frames'/row['evidence']['file'], allow_pickle=False) as data:
            rgb, depth = data['rgb'], data['depth']
            raw_valid.append(rgb.ndim == 3 and rgb.shape[2] == 3 and rgb.dtype == np.uint8
                             and depth.shape == rgb.shape[:2] and data['K'].shape == (3, 3)
                             and data['T_world_camera'].shape == (4, 4)
                             and np.isfinite(data['K']).all() and np.isfinite(data['T_world_camera']).all())
            raw_times.append(float(data['timestamp']) == row['acquisition_timestamp'])
        backend = row['inference_backend']
        inference_times.append(backend.get('frame_id') == row['frame_id']
                               and backend.get('timestamp') == row['acquisition_timestamp'])
    checks['raw_rgbd_geometry_valid'] = bool(raw_valid) and all(raw_valid)
    checks['raw_acquisition_times_match'] = bool(raw_times) and all(raw_times)
    checks['inference_frame_times_match'] = bool(inference_times) and all(inference_times)
    arrivals = [r for r in runtime if r['event'] == 'navigation_result' and r.get('command_id') and r.get('succeeded')]
    checks['actual_navigation_result'] = any(r['time'] == found['arrived'] for r in arrivals)
    checks['navigation_start_result_link'] = any(
        r['time'] == found['arrived'] and not r.get('cancelled', False)
        and any(s['event'] == 'navigation_started' and s.get('command_id') == r['command_id']
                and outside['time'] < s['time'] < r['time'] for s in runtime)
        for r in arrivals)
    odometry = [r for r in motion if r['topic'] == '/odom']
    xy = [(r['message']['pose']['pose']['position']['x'], r['message']['pose']['pose']['position']['y']) for r in odometry]
    displacement = max(math.dist(p, xy[0]) for p in xy) if xy else 0.0
    checks['simulator_actual_departure'] = displacement >= .65
    def odom_time(row):
        stamp = row['message']['header']['stamp']
        return stamp['sec'] + stamp['nanosec'] / 1e9
    departure_xy = [p for r, p in zip(odometry, xy)
                    if initial['acquisition_timestamp'] <= odom_time(r) <= outside['time']]
    checks['simulator_departure_before_query'] = bool(departure_xy) and max(
        math.dist(p, xy[0]) for p in departure_xy) >= .65
    safe = [r for r in motion if r['topic'] == '/fpga/cmd_vel_safe']
    def moving(row):
        return abs(row['message']['linear']['x'])+abs(row['message']['angular']['z']) > .001
    checks['simulator_received_nonzero_commands'] = any(moving(r) for r in safe)
    tail = [r for r in safe if r['wall_monotonic'] >= safe[-1]['wall_monotonic']-1] if safe else []
    checks['simulator_final_zero_commands'] = len(tail) >= 5 and not any(moving(r) for r in tail)
    last_odom = odometry[-1]['message']['twist']['twist'] if odometry else None
    checks['simulator_final_stopped'] = last_odom is not None and abs(last_odom['linear']['x'])+abs(last_odom['angular']['z']) < .01
    checks['no_runtime_fault'] = not any(r['event'] == 'fault' for r in runtime)
    # line D1 additions
    pcq = [r for r in scenario if r['event'] == 'query_pc_command']
    checks['query_forwarded_by_hub'] = (len(pcq) == 1 and pcq[0]['returncode'] == 0
                                        and (pcq[0].get('ack') or {}).get('status') == 'forwarded'
                                        and query['elapsed_wall'] <= pcq[0]['elapsed_wall'] < found['elapsed_wall'])
    sources, pc_frames = pc_state_sources(run/'pc.fpgarec')
    checks['pc_state_source_sim'] = bool(sources) and all(v == 'sim' for v in sources)
    adapter = json.loads((run/'adapter_summary.json').read_text())
    checks['adapter_nonempty_finite'] = adapter_ok(adapter)
    producer = json.loads((run/'producer_summary.json').read_text())
    fused = producer_fused(producer)
    checks['producer_fused_frames'] = fused is not None and fused > 0
    checks['producer_data_source_sim'] = producer.get('data_source') == 'sim'
    chassis = config.get('chassis', 'guard')
    if chassis == 'mcu':
        checks.update(chassis_checks(run, motion))
    return {'passed': all(checks.values()), 'checks': checks, 'layout': 'sim', 'source_revision': revision,
            'detector': detector, 'augmentation': augmentation, 'chassis': chassis,
            'runtime_info': {k: info.get(k) for k in ('detector', 'detector_checked')},
            'target_id': target, 'initial_frame': initial['frame_id'], 'outside_frame': outside['frame_id'],
            'found_frame': final['frame_id'], 'arrival_sim_time': found['arrived'],
            'found_acquisition_sim_time': final['acquisition_timestamp'],
            'scenario_wall_seconds': result['elapsed_wall'], 'processed_frames': len(frames),
            'max_simulator_displacement_m': displacement, 'pc_recorded_frames': pc_frames,
            'pc_state_frames': len(sources), 'producer_fused_frames': fused,
            'inference_device': device,
            'scope': 'one scripted scene; server simulation, ' + detector + ' on the host (' + device + '); '
                     'known map and simulated reference pose; '
                     'host numbers (宿主，不代表板上); no accuracy/performance claim'}


SIM_TWIST = '/fpga/chassis/sim_twist'


def single_source(text):
    """single_source.txt (stack_inner.sh): Gazebo /fpga/cmd_vel publishers, ROS publishers of the MCU twist,
    ROS subscribers of /fpga/cmd_vel_safe (node names)."""
    sections, cur = {}, None
    for line in text.splitlines():
        if line.startswith('== '):
            cur = line[3:].strip()
            sections[cur] = []
        elif cur is not None:
            sections[cur].append(line)
    gz = sections.get('ign topic -i -t /fpga/cmd_vel', [])
    gz_pubs, in_pubs = 0, False
    for line in gz:
        if line.strip().startswith('Publishers'):
            in_pubs = True
            continue
        if line.strip().startswith('Subscribers'):
            in_pubs = False
        if in_pubs and ('tcp://' in line or 'udp://' in line or 'inproc' in line):
            gz_pubs += 1

    def ros(topic):
        lines = sections.get('ros2 topic info -v ' + topic, [])
        pubs, subs, kind, node = [], [], None, None
        for line in lines:
            t = line.strip()
            if t.startswith('Node name:'):
                node = t.split(':', 1)[1].strip()
            if t.startswith('Endpoint type:'):
                kind = t.split(':', 1)[1].strip()
                (pubs if kind == 'PUBLISHER' else subs).append(node)
        return pubs, subs
    twist_pubs, _ = ros(SIM_TWIST)
    _, safe_subs = ros('/fpga/cmd_vel_safe')
    return {'gz_cmd_vel_publishers': gz_pubs, 'sim_twist_publishers': twist_pubs, 'cmd_vel_safe_subscribers': safe_subs}


def chassis_checks(run, motion):
    """CHASSIS=mcu: the simulator's single wheel command is the MCU stand-in's executed speed."""
    checks = {}
    src_path = run/'single_source.txt'
    src = single_source(src_path.read_text()) if src_path.exists() else {}
    checks['gazebo_one_command_publisher'] = src.get('gz_cmd_vel_publishers') == 1
    checks['sim_twist_only_from_mcu'] = src.get('sim_twist_publishers') == ['fpga_mcu_sim']
    checks['cmd_vel_safe_not_bridged'] = bool(src.get('cmd_vel_safe_subscribers')) and not any(
        'bridge' in n for n in src.get('cmd_vel_safe_subscribers', []))
    twist = [r for r in motion if r['topic'] == SIM_TWIST]
    def moving(row):
        return abs(row['message']['linear']['x'])+abs(row['message']['angular']['z']) > .001
    checks['simulator_received_nonzero_mcu_speed'] = any(moving(r) for r in twist)
    tail = [r for r in twist if r['wall_monotonic'] >= twist[-1]['wall_monotonic']-1] if twist else []
    checks['simulator_final_zero_mcu_speed'] = len(tail) >= 5 and not any(moving(r) for r in tail)
    return checks


def stop_chain(run, cause, t_cmd_wall_ns):
    """D2 stop chain after a command at t_cmd_wall_ns (cut: SIGTERM to the producer; estop: fpgapc estop):
    the first revocation after it, the MCU latch after that, the executed speed at the latch, the car still.
    Returns (checks, facts); bounds are the module constants (PREREG)."""
    motion = read_rows(run/'motion.jsonl')
    telem, clk = ct.telem_rows(read_rows(run/'permit_telem.jsonl'))
    mcu = ct.mcu_view(read_rows(run/'mcu.jsonl'))
    pairs = ct.clock_map(motion)
    odom = ct.odom_rows(motion)
    checks, facts = {}, {}
    sim = lambda w: ct.wall_to_sim(pairs, w)   # noqa: E731
    # precondition: permit set and the MCU unlocked and moving when the command was given
    permit_before = ct.permit_at(telem, t_cmd_wall_ns)
    events_before = [e for e in mcu['events'] if e['wall_ns'] <= t_cmd_wall_ns]
    executed_before = [r for r in mcu['executed'] if r['wall_ns'] <= t_cmd_wall_ns]
    checks['pre_permit_set'] = permit_before is True
    checks['pre_mcu_unlocked'] = bool(events_before) and events_before[-1]['event'] == 'unlock'
    checks['pre_mcu_executing_motion'] = bool(executed_before) and ct.moving(*executed_before[-1]['executed'])
    revs = [r for r in ct.revocations(telem, clk) if r['revoke_wall_ns'] >= t_cmd_wall_ns - 2_000_000]
    rev = revs[0] if revs else None
    checks['revoked_with_cause'] = rev is not None and rev['cause'] == cause and rev['missed'] == 0
    if cause == ct.CAUSE_TIMEOUT:
        checks['revoke_is_done_plus_timeout'] = (rev is not None and rev['timeout_cycles'] is not None
                                                 and rev['revoke_minus_done_cycles'] == rev['timeout_cycles'] + 1)
    latches = [e for e in mcu['events'] if e['event'] == 'latch' and rev is not None
               and e['wall_ns'] >= rev['revoke_wall_ns'] - 2_000_000]
    latch = latches[0] if latches else None
    checks['mcu_latched'] = latch is not None
    lat_s = None if latch is None or rev is None else (latch['wall_ns'] - rev['revoke_wall_ns']) / 1e9
    checks['latch_after_revoke_within_bound'] = lat_s is not None and -0.002 <= lat_s <= LATCH_AFTER_REVOKE_S
    zero = None
    if latch is not None:
        zero = next((r for r in mcu['executed'] if r['t_ms'] >= latch['t_ms'] - 0 and not ct.moving(*r['executed'])), None)
        later = [r for r in mcu['executed'] if r['t_ms'] > latch['t_ms']]
        unlock_after = [e for e in mcu['events'] if e['event'] == 'unlock' and e['t_ms'] > latch['t_ms']]
        end_ms = unlock_after[0]['t_ms'] if unlock_after else float('inf')
        checks['executed_zero_at_latch'] = zero is not None and zero['t_ms'] == latch['t_ms']
        checks['executed_stays_zero_while_latched'] = not any(
            ct.moving(*r['executed']) for r in later if r['t_ms'] < end_ms)
        facts['unlock_after_latch_ms'] = None if not unlock_after else unlock_after[0]['t_ms'] - latch['t_ms']
    start_wall = rev['done_wall_ns'] if cause == ct.CAUSE_TIMEOUT and rev is not None else (
        rev['revoke_wall_ns'] if rev is not None else None)
    chain_wall = None if latch is None or start_wall is None else (latch['wall_ns'] - start_wall) / 1e9
    s_start, s_latch = (sim(start_wall) if start_wall else None), (sim(latch['wall_ns']) if latch else None)
    chain_sim = None if s_start is None or s_latch is None else s_latch - s_start
    bound = CHAIN_BOUND_S if cause == ct.CAUSE_TIMEOUT else LATCH_AFTER_REVOKE_S
    checks['chain_within_bound_sim'] = chain_sim is not None and 0 <= chain_sim <= bound
    checks['chain_within_bound_wall'] = chain_wall is not None and 0 <= chain_wall <= bound
    v0 = w0 = None
    still = None
    if latch is not None and s_latch is not None:
        prior = [r for r in mcu['executed'] if r['t_ms'] < latch['t_ms']]
        v0, w0 = prior[-1]['executed'] if prior else (0.0, 0.0)
        decel = max(abs(v0) / DIFF_DRIVE_ACCEL[0], abs(w0) / DIFF_DRIVE_ACCEL[1])
        still = ct.first_still(odom, s_latch)
        facts['decel_bound_s'] = decel + STILL_ALLOWANCE_S
        checks['car_still_within_bound_sim'] = still is not None and still['sim'] - s_latch <= decel + STILL_ALLOWANCE_S
    else:
        checks['car_still_within_bound_sim'] = False
    safe = [r for r in motion if r['topic'] == '/fpga/cmd_vel_safe' and latch is not None
            and latch['wall_ns'] < r['wall_ns'] and (still is None or r['wall_ns'] <= still['wall_ns'] + 500_000_000)]
    facts.update({
        'pl_clk_hz': clk, 'command_wall_ns': t_cmd_wall_ns, 'command_sim': sim(t_cmd_wall_ns),
        'revocation': rev, 'revoke_sim': sim(rev['revoke_wall_ns']) if rev else None,
        'last_done_sim': sim(rev['done_wall_ns']) if rev else None,
        'latch_wall_ns': latch['wall_ns'] if latch else None, 'latch_sim': s_latch,
        'executed_zero_wall_ns': zero['wall_ns'] if zero else None,
        'executed_zero_sim': sim(zero['wall_ns']) if zero else None,
        'speed_zero_odom_sim': still['sim'] if still else None,
        'latch_minus_revoke_wall_s': lat_s, 'chain_wall_s': chain_wall, 'chain_sim_s': chain_sim,
        'chain_bound_s': bound, 'executed_before_latch': [v0, w0],
        'still_minus_latch_sim_s': None if still is None or s_latch is None else still['sim'] - s_latch,
        'nonzero_cmd_vel_safe_after_latch': sum(
            1 for r in safe if abs(r['message']['linear']['x']) + abs(r['message']['angular']['z']) > .001),
    })
    return checks, facts


def verify_sim_cut(run):
    """D2 cut run (run_sim_loop.sh cut, CHASSIS=mcu): producer stopped while the car moves -> permit TIMEOUT
    -> MCU latch -> executed speed zero -> car still. With mcu_heartbeat 'always' (positive control) the
    expected outcome is the opposite (control_* facts)."""
    scenario = read_rows(run/'scenario.jsonl')
    config = json.loads((run/'run_config.json').read_text())
    results = [r for r in scenario if r['event'] == 'scenario_result']
    cuts = [r for r in scenario if r['event'] == 'cut_requested']
    checks = {'one_scenario_result': len(results) == 1, 'one_cut': len(cuts) == 1,
              'chassis_mcu': config.get('chassis') == 'mcu'}
    if not all(checks.values()):
        return {'passed': False, 'checks': checks, 'layout': 'sim-cut'}
    result, cut = results[0], cuts[0]
    chain_checks, facts = stop_chain(run, ct.CAUSE_TIMEOUT, cut['wall_ns'])
    revision = (run/'head.txt').read_text().strip()
    common = {'source_revision_clean': (bool(revision) and config.get('head') == revision
                                        and config.get('dirty') is False and (run/'dirty.txt').read_text() == ''),
              'server_x86_64': scenario[0].get('machine') == 'x86_64'}
    common.update(chassis_checks(run, read_rows(run/'motion.jsonl')))
    common.pop('simulator_final_zero_mcu_speed', None)
    out = {'layout': 'sim-cut', 'source_revision': revision, 'mcu_heartbeat': config.get('mcu_heartbeat'),
           'scenario_result': result, 'facts': facts,
           'scope': 'server simulation of the permit / MCU stop chain; host numbers (宿主，不代表板上)'}
    if config.get('mcu_heartbeat') == 'always':
        # positive control: the same revocation, but no latch, and the car keeps moving
        motion = read_rows(run/'motion.jsonl')
        odom = ct.odom_rows(motion)
        rev = facts.get('revocation')
        rs = facts.get('revoke_sim')
        control = {'control_precondition_permit_set': chain_checks['pre_permit_set'],
                   'control_revoked_timeout': chain_checks['revoked_with_cause'],
                   'control_no_latch': not chain_checks['mcu_latched'],
                   'control_car_moving_1s_after_revoke': rs is not None and bool(ct.moving_after(odom, rs + 1.0)),
                   'control_scenario_not_stopped': result['reason'] == 'cut_not_stopped'}
        control.update(common)
        out.update(passed=all(control.values()), checks=control)
        return out
    checks.update(chain_checks)
    checks.update(common)
    checks['scenario_cut_stopped'] = result['passed'] is True and result['reason'] == 'cut_stopped'
    out.update(passed=all(checks.values()), checks=checks)
    return out


def verify_sim_estop_mcu(run, scenario, config):
    """D2 e-stop (CHASSIS=mcu): FORCE revocation -> MCU latch -> car still; after the PC release a new frame
    (permit grant), >= 100 ms of heartbeat and a command after it unlock the MCU; motion only after that."""
    results = [r for r in scenario if r['event'] == 'scenario_result']
    reqs = [r for r in scenario if r['event'] == 'estop_requested']
    rels = [r for r in scenario if r['event'] == 'release_requested']
    checks = {'one_scenario_result': len(results) == 1, 'one_estop': len(reqs) == 1, 'one_release': len(rels) == 1}
    if not all(checks.values()):
        return {'passed': False, 'checks': checks, 'layout': 'sim-estop'}
    result, req, rel = results[0], reqs[0], rels[0]
    chain_checks, facts = stop_chain(run, ct.CAUSE_FORCE, req['wall_ns'])
    checks.update(chain_checks)
    checks['scenario_estop_released_resumed'] = (result['passed'] is True
                                                 and result['reason'] == 'estop_stopped_released_resumed')
    motion = read_rows(run/'motion.jsonl')
    telem, clk = ct.telem_rows(read_rows(run/'permit_telem.jsonl'))
    mcu = ct.mcu_view(read_rows(run/'mcu.jsonl'))
    t_rel = rel['wall_ns']
    latch_wall = facts.get('latch_wall_ns')
    # while latched (latch .. release): permit never granted, the MCU never unlocked
    held = [r for r in telem if latch_wall is not None and latch_wall <= r['wall_mid'] <= t_rel]
    checks['no_permit_while_estop'] = bool(held) and not any(ct.permit_bit(r) for r in held)
    checks['no_unlock_before_release'] = latch_wall is not None and not any(
        e['event'] == 'unlock' and latch_wall < e['wall_ns'] < t_rel for e in mcu['events'])
    grant = next((g for g in ct.grants(telem, clk) if g['seen_wall_ns'] > t_rel), None)
    unlock = next((e for e in mcu['events'] if e['event'] == 'unlock' and e['wall_ns'] > t_rel), None)
    checks['new_frame_grant_after_release'] = grant is not None and grant['done_wall_ns'] > t_rel
    start = mcu['start']
    wall_of = (lambda ms: start['wall_ns'] + ms * 1000000) if start else (lambda ms: None)
    ok_unlock = bool(unlock and unlock.get('hb_good_since_ms') is not None and unlock.get('last_cmd_ms') is not None
                     and unlock['t_ms'] - unlock['hb_good_since_ms'] >= 100
                     and unlock['last_cmd_ms'] > unlock['hb_good_since_ms'])
    checks['unlock_after_100ms_heartbeat_and_new_command'] = ok_unlock
    checks['heartbeat_back_after_grant'] = bool(unlock and grant and unlock.get('hb_good_since_ms') is not None
                                                and wall_of(unlock['hb_good_since_ms']) >= grant['done_wall_ns'] - 2_000_000)
    first_move = next((r for r in mcu['executed'] if r['wall_ns'] > t_rel and ct.moving(*r['executed'])), None)
    checks['first_motion_after_unlock'] = bool(first_move and unlock and first_move['t_ms'] > unlock['t_ms'])
    pairs = ct.clock_map(motion)
    odom = ct.odom_rows(motion)
    s_move = ct.wall_to_sim(pairs, first_move['wall_ns']) if first_move else None
    s_latch = facts.get('latch_sim')
    early = [r for r in odom if s_latch is not None and s_move is not None
             and facts.get('speed_zero_odom_sim') is not None
             and facts['speed_zero_odom_sim'] <= r['sim'] < s_move and r['speed'] >= ct.STILL]
    checks['car_still_until_first_motion'] = s_move is not None and not early
    revision = (run/'head.txt').read_text().strip()
    checks['source_revision_clean'] = (bool(revision) and config.get('head') == revision
                                       and config.get('dirty') is False and (run/'dirty.txt').read_text() == '')
    checks['server_x86_64'] = scenario[0].get('machine') == 'x86_64'
    checks.update({k: v for k, v in chassis_checks(run, motion).items() if k != 'simulator_final_zero_mcu_speed'})
    sim = lambda w: ct.wall_to_sim(pairs, w)   # noqa: E731
    facts.update({'release_wall_ns': t_rel, 'release_sim': sim(t_rel),
                  'grant_done_sim': sim(grant['done_wall_ns']) if grant else None,
                  'heartbeat_back_sim': sim(wall_of(unlock['hb_good_since_ms'])) if unlock and unlock.get('hb_good_since_ms') is not None else None,
                  'unlock_sim': sim(unlock['wall_ns']) if unlock else None,
                  'unlock_minus_heartbeat_ms': (unlock['t_ms'] - unlock['hb_good_since_ms']) if ok_unlock else None,
                  'first_motion_sim': s_move, 'task_after_resume': result.get('task_after_resume'),
                  'estop_pc': {k: req.get(k) for k in ('clock', 'speed')}})
    return {'passed': all(checks.values()), 'checks': checks, 'layout': 'sim-estop', 'chassis': 'mcu',
            'source_revision': revision, 'scenario_result': result, 'facts': facts,
            'scope': 'server simulation of the PC e-stop -> permit FORCE -> MCU latch chain; '
                     'host numbers (宿主，不代表板上)'}


def verify_sim_estop(run):
    """Server simulation e-stop run (run_sim_loop.sh estop): the scenario's own verdict plus the stop chain
    read from motion.jsonl (wall ns of the PC command -> /fpga/sim_estop True -> zero /fpga/cmd_vel_safe)."""
    scenario = read_rows(run/'scenario.jsonl')
    motion = read_rows(run/'motion.jsonl')
    config = json.loads((run/'run_config.json').read_text())
    if config.get('chassis') == 'mcu':
        return verify_sim_estop_mcu(run, scenario, config)
    results = [r for r in scenario if r['event'] == 'scenario_result']
    evidence = [r for r in scenario if r['event'] == 'estop_evidence']
    checks = {'one_scenario_result': len(results) == 1, 'one_estop_evidence': len(evidence) == 1}
    if not all(checks.values()):
        return {'passed': False, 'checks': checks, 'layout': 'sim-estop'}
    result, e = results[0], evidence[0]
    t0 = e['command_wall_ns']
    checks['scenario_estop_stopped'] = result['passed'] is True and result['reason'] == 'estop_stopped'
    checks['pc_estop_latched'] = e.get('returncode') == 0 and (e.get('ack') or {}).get('status') == 'latched_on_nano'
    gate = [r for r in motion if r['topic'] == '/fpga/sim_estop' and r['wall_ns'] >= t0 and r['message']['data']]
    safe = [r for r in motion if r['topic'] == '/fpga/cmd_vel_safe' and r['wall_ns'] >= t0]
    def moving(row):
        return abs(row['message']['linear']['x'])+abs(row['message']['angular']['z']) > .001
    first_zero = next((r for r in safe if not moving(r)), None)
    checks['sim_estop_true_after_command'] = bool(gate)
    checks['zero_command_after_estop'] = first_zero is not None and bool(gate) and not any(
        moving(r) for r in safe if r['wall_ns'] >= max(first_zero['wall_ns'], gate[0]['wall_ns']))
    revision = (run/'head.txt').read_text().strip()
    checks['source_revision_clean'] = (bool(revision) and config.get('head') == revision
                                       and config.get('dirty') is False and (run/'dirty.txt').read_text() == '')
    checks['server_x86_64'] = scenario[0].get('machine') == 'x86_64'
    return {'passed': all(checks.values()), 'checks': checks, 'layout': 'sim-estop', 'source_revision': revision,
            'command_wall_ns': t0,
            'sim_estop_true_after_s_wall': (gate[0]['wall_ns']-t0)/1e9 if gate else None,
            'first_zero_cmd_after_s_wall': (first_zero['wall_ns']-t0)/1e9 if first_zero else None,
            'still_after_s_sim': result.get('estop_to_still_sim_s'),
            'still_after_s_wall': result.get('estop_to_still_wall_s'),
            'speed_at_command': result.get('speed_at_command'),
            'scope': 'server simulation stand-in of the stop path (KV e-stop latch -> velocity guard), '
                     'not the PL permit / MCU path (line D2); host numbers (宿主，不代表板上)'}


def adapter_ok(summary):
    """sim_adapter.py --summary: frames converted, outputs non-empty and finite."""
    return bool(summary.get('depth_out', 0) > 0 and summary.get('cloud_out', 0) > 0
                and summary.get('nonfinite_out', 1) == 0 and summary.get('bad_encoding', 1) == 0
                and (summary.get('valid_fraction_min') or 0) > 0)


def producer_fused(summary):
    """Fused frames = published rolling-map deltas (host/ros_geo_producer.py summary counts)."""
    value = (summary.get('counts') or {}).get('published_deltas')
    return value if isinstance(value, int) else None


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path); p.add_argument('--out',type=Path,required=True)
    p.add_argument('--layout',choices=('agx','sim','sim-estop','sim-cut'),default='agx')
    args=p.parse_args()
    result={'agx':verify,'sim':verify_sim,'sim-estop':verify_sim_estop,'sim-cut':verify_sim_cut}[args.layout](args.run)
    with args.out.open('x') as stream: json.dump(result,stream,ensure_ascii=False,indent=2);stream.write('\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()
