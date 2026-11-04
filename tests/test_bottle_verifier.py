"""Synthetic mechanism checks; these do not replace real-run calibration."""
import hashlib
import io
import json
import sys
import tarfile
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from calibrate_bottle_verifier import calibrate, write_rows
from verify_bottle_run import read_rows, verify, FRCNN_CHECKPOINT_SHA256


def fixture(run):
    (run/'agx/frames').mkdir(parents=True)
    revision = 'a'*40
    (run/'agx/source-revision.txt').write_text(revision)
    (run/'completed.json').write_text(json.dumps({'source_revision': revision}))
    frames = []
    for timestamp in (3, 5, 10):
        frame_id = 'rgb-'+str(timestamp*10**9)
        path = run/'agx/frames'/(frame_id+'.npz')
        np.savez_compressed(path, rgb=np.full((2, 2, 3), timestamp, np.uint8),
                            depth=np.ones((2, 2)), K=np.eye(3), T_world_camera=np.eye(4), timestamp=timestamp)
        frames.append({'event': 'frame_processed', 'frame_id': frame_id, 'time': timestamp,
                       'acquisition_timestamp': timestamp,
                       'evidence': {'file': path.name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()},
                       'inference_backend': {'execution': {'device': 'cuda', 'memory_format': 'contiguous',
                                                          'cuda_math': 'fp32'},
                                             'frame_id': frame_id, 'timestamp': timestamp},
                       'detections': [] if timestamp == 5 else [{'label': 'bottle', 'score': .8}],
                       'tracks': [{'id': 'bottle-1', 'label': 'bottle',
                                   'state': 'historical_memory' if timestamp == 5 else 'current_confirmed',
                                   'reason': 'out_of_view' if timestamp == 5 else 'new_visual_observation',
                                   'last_seen_frame': frame_id, 'last_seen': timestamp}]})
    write_rows(run/'agx/runtime.jsonl', [frames[0], frames[1],
               {'event': 'navigation_started', 'time': 6, 'command_id': 'query:1'},
               {'event': 'navigation_result', 'time': 9, 'command_id': 'query:1', 'succeeded': True}, frames[2]])
    write_rows(run/'agx/scenario.jsonl', [
        {'event': 'scenario_start', 'machine': 'aarch64'},
        {'event': 'remembered', 'elapsed_wall': 3, 'frame': frames[0]},
        {'event': 'departed_out_of_view', 'elapsed_wall': 5, 'frame': frames[1]},
        {'event': 'query_requested', 'elapsed_wall': 6, 'target_id': 'bottle-1'},
        {'event': 'found_evidence', 'elapsed_wall': 10, 'frame': frames[2], 'arrived': 9,
         'task': {'target_id': 'bottle-1'}},
        {'event': 'scenario_result', 'elapsed_wall': 12, 'passed': True, 'stopped': True}])
    def velocity(speed):
        return {'linear': {'x': speed, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': 0}}
    motion = [{'topic': '/odom', 'wall_monotonic': t,
               'message': {'header': {'stamp': {'sec': t, 'nanosec': 0}},
                           'pose': {'pose': {'position': {'x': x, 'y': 0, 'z': 0}}},
                           'twist': {'twist': velocity(0)}}} for t, x in ((0, 0), (3, 0), (5, .7), (9, 0), (11, 0))]
    motion += [{'topic': '/fpga/cmd_vel_safe', 'wall_monotonic': t, 'message': velocity(speed)}
               for t, speed in [(4, .1)] + [(10 + i*.1, 0) for i in range(11)]]
    write_rows(run/'motion.jsonl', motion)


class BottleVerifierTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name)/'run'
        fixture(self.run)

    def modern_fixture(self, detector='fasterrcnn_resnet50_800', scanned=False):
        completed = json.loads((self.run/'completed.json').read_text())
        completed.update(detector=detector, augmentation='none',
                         scene='scanned_bottle' if scanned else 'primitive')
        rows = read_rows(self.run/'agx/runtime.jsonl')
        for row in rows:
            if row['event'] == 'frame_processed':
                row['inference_backend']['detector'] = detector
                if detector == 'fasterrcnn_resnet50_800':
                    row['inference_backend'].update(checkpoint_sha256=FRCNN_CHECKPOINT_SHA256,
                                                   resize={'min_size': 800, 'max_size': 1333})
        rows.insert(0, {'event': 'startup', 'time': 0, 'memory_tracks': [], 'task_active': False, 'patrol': []})
        self.write_runtime_and_stage_frames(rows)
        sources = {'tools/run_agx_bottle.py': b"parser.add_argument('--detector')\n",
                   'host/ros_runtime.py': b"self.record('startup', memory_tracks=self.memory.snapshot())\n"}
        if scanned:
            # Original external asset paths intentionally do not exist: archived
            # verification must use the saved source manifest/provenance only.
            manifest = {'asset_id': 'calibration-bottle', 'source_url': 'https://example.invalid/asset',
                        'archive_sha256': 'b'*64, 'license': {'identifier': 'CC-BY-4.0'},
                        'required_files_sha256': {'meshes/model.obj': 'c'*64, 'texture.png': 'd'*64}}
            manifest_bytes = json.dumps(manifest).encode()
            sources.update({'robot_sim/fpga_sim/config/scanned_bottle_asset.json': manifest_bytes,
                            'robot_sim/fpga_sim/worlds/room.sdf': b'<sdf>original</sdf>',
                            'tools/make_scanned_bottle_world.py': b'# fixed generator\n'})
            generated = b'<sdf>scanned bottle</sdf>'
            (self.run/'generated-room.sdf').write_bytes(generated)
            digest = lambda data: hashlib.sha256(data).hexdigest()
            completed['bottle_asset_manifest_sha256'] = digest(manifest_bytes)
            provenance = {key: manifest[key] for key in ('asset_id', 'source_url', 'archive_sha256', 'license')}
            provenance.update(scene='scanned_bottle', asset_dir='/missing/original/asset',
                              asset_manifest='/missing/original/manifest',
                              asset_manifest_sha256=digest(manifest_bytes),
                              output_world_sha256=digest(generated),
                              source_world_sha256=digest(sources['robot_sim/fpga_sim/worlds/room.sdf']),
                              generator_sha256=digest(sources['tools/make_scanned_bottle_world.py']),
                              asset_files_sha256=manifest['required_files_sha256'])
            (self.run/'scene-provenance.json').write_text(json.dumps(provenance))
        with tarfile.open(self.run/'source.tar', 'w') as archive:
            for name, data in sources.items():
                member = tarfile.TarInfo(name)
                member.size = len(data)
                archive.addfile(member, io.BytesIO(data))
        (self.run/'completed.json').write_text(json.dumps(completed))
        (self.run/'run-config.json').write_text(json.dumps(completed))

    def write_runtime_and_stage_frames(self, rows):
        write_rows(self.run/'agx/runtime.jsonl', rows)
        frames = {row['frame_id']: row for row in rows if row['event'] == 'frame_processed'}
        stages = read_rows(self.run/'agx/scenario.jsonl')
        for stage in stages:
            if 'frame' in stage:
                stage['frame'] = frames[stage['frame']['frame_id']]
        write_rows(self.run/'agx/scenario.jsonl', stages)

    def test_positive_and_two_independent_negative_controls(self):
        original = {str(p.relative_to(self.run)): p.read_bytes() for p in self.run.rglob('*') if p.is_file()}
        self.assertTrue(verify(self.run)['passed'])
        result = calibrate(self.run, Path(self.temp.name)/'calibration')
        self.assertTrue(result['passed'])
        self.assertEqual(result['cases']['old_frame']['failed_checks'], ['raw_acquisition_times_match'])
        self.assertEqual(original, {str(p.relative_to(self.run)): p.read_bytes() for p in self.run.rglob('*') if p.is_file()})

    def test_movement_only_after_departure_does_not_count(self):
        rows = read_rows(self.run/'motion.jsonl')
        for row in rows:
            if row['topic'] == '/odom':
                row['message']['pose']['pose']['position']['x'] = .7 if row['wall_monotonic'] == 9 else 0
        write_rows(self.run/'motion.jsonl', rows)
        result = verify(self.run)
        self.assertTrue(result['checks']['simulator_actual_departure'])
        self.assertFalse(result['checks']['simulator_departure_before_query'])

    def test_duplicate_frame_is_rejected(self):
        rows = read_rows(self.run/'agx/runtime.jsonl')
        write_rows(self.run/'agx/runtime.jsonl', rows + [rows[-1]])
        self.assertFalse(verify(self.run)['checks']['unique_processed_frame_ids'])

    def test_unmatched_navigation_result_is_rejected(self):
        rows = read_rows(self.run/'agx/runtime.jsonl')
        rows = [row for row in rows if row['event'] != 'navigation_started']
        write_rows(self.run/'agx/runtime.jsonl', rows)
        self.assertFalse(verify(self.run)['checks']['navigation_start_result_link'])

    def test_runtime_with_backend_by_reference_still_passes(self):
        # 10-29 host/backend_log.py: the runtime writes the backend metadata in full once, then short rows
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from host.backend_log import BackendLog
        baseline = verify(self.run)
        log = BackendLog()
        rows = read_rows(self.run/'agx/runtime.jsonl')
        for row in rows:
            if 'inference_backend' in row:
                row['inference_backend'] = log.field(row['inference_backend'], row['time'])
        self.assertEqual(sum('execution' in r.get('inference_backend', {}) for r in rows), 1)
        write_rows(self.run/'agx/runtime.jsonl', rows)
        result = verify(self.run)
        self.assertTrue(result['passed'])
        self.assertEqual(result['checks'], baseline['checks'])

    def test_cannot_calibrate_in_original_run(self):
        with self.assertRaises(ValueError):
            calibrate(self.run, self.run/'derived')

    def test_explicit_detector_profiles_and_legacy_positive(self):
        legacy = verify(self.run)
        self.assertTrue(legacy['passed'])
        self.assertTrue(legacy['legacy_detector_evidence'])
        for detector in ('ssdlite', 'fasterrcnn_resnet50_800'):
            with self.subTest(detector=detector):
                self.modern_fixture(detector)
                result = verify(self.run)
                self.assertTrue(result['passed'], result['checks'])
                self.assertFalse(result['legacy_detector_evidence'])
                self.assertEqual(result['detector'], detector)
                # Rebuild original rows so startup remains singular in the next subcase.
                rows = [row for row in read_rows(self.run/'agx/runtime.jsonl') if row['event'] != 'startup']
                write_rows(self.run/'agx/runtime.jsonl', rows)

    def test_wrong_architecture_on_nonterminal_frame_is_rejected(self):
        self.modern_fixture()
        rows = read_rows(self.run/'agx/runtime.jsonl')
        next(row for row in rows if row.get('frame_id') == 'rgb-5000000000')['inference_backend']['detector'] = 'ssdlite'
        self.write_runtime_and_stage_frames(rows)
        result = verify(self.run)
        self.assertTrue(result['checks']['real_cuda_detections'])
        self.assertFalse(result['checks']['all_processed_detector_matches'])

    def test_wrong_checkpoint_or_resize_is_rejected(self):
        self.modern_fixture()
        rows = read_rows(self.run/'agx/runtime.jsonl')
        backend = next(row for row in rows if row.get('frame_id') == 'rgb-5000000000')['inference_backend']
        backend['checkpoint_sha256'] = '0'*64
        self.write_runtime_and_stage_frames(rows)
        self.assertFalse(verify(self.run)['checks']['detector_checkpoint_resize'])
        backend['checkpoint_sha256'] = FRCNN_CHECKPOINT_SHA256
        backend['resize']['min_size'] = 320
        self.write_runtime_and_stage_frames(rows)
        self.assertFalse(verify(self.run)['checks']['detector_checkpoint_resize'])

    def test_nonempty_or_missing_startup_memory_is_rejected(self):
        self.modern_fixture()
        rows = read_rows(self.run/'agx/runtime.jsonl')
        rows[0]['memory_tracks'] = [{'id': 'old-bottle'}]
        self.write_runtime_and_stage_frames(rows)
        self.assertFalse(verify(self.run)['checks']['startup_empty_memory_and_task'])
        del rows[0]['memory_tracks']
        self.write_runtime_and_stage_frames(rows)
        self.assertFalse(verify(self.run)['checks']['startup_empty_memory_and_task'])

    def test_new_archived_source_cannot_fall_back_to_legacy_when_fields_missing(self):
        self.modern_fixture()
        for name in ('completed.json', 'run-config.json'):
            value = json.loads((self.run/name).read_text())
            del value['detector']
            (self.run/name).write_text(json.dumps(value))
        rows = read_rows(self.run/'agx/runtime.jsonl')
        for row in rows:
            if row['event'] == 'frame_processed':
                del row['inference_backend']['detector']
        self.write_runtime_and_stage_frames(rows)
        result = verify(self.run)
        self.assertFalse(result['legacy_detector_evidence'])
        self.assertFalse(result['checks']['detector_profile_declared'])
        self.assertFalse(result['checks']['all_processed_detector_matches'])

    def test_scanned_scene_verifies_and_calibrates_without_external_asset(self):
        self.modern_fixture(scanned=True)
        result = verify(self.run)
        self.assertTrue(result['passed'], result['checks'])
        self.assertTrue(calibrate(self.run, Path(self.temp.name)/'modern-calibration')['passed'])
        (self.run/'generated-room.sdf').write_text('<sdf>wrong world</sdf>')
        self.assertFalse(verify(self.run)['checks']['scanned_world_hash_matches'])

    def test_scanned_scene_rejects_asset_map_and_manifest_mismatch(self):
        self.modern_fixture(scanned=True)
        path = self.run/'scene-provenance.json'
        provenance = json.loads(path.read_text())
        provenance['asset_files_sha256']['meshes/model.obj'] = '0'*64
        provenance['asset_manifest_sha256'] = '0'*64
        path.write_text(json.dumps(provenance))
        checks = verify(self.run)['checks']
        self.assertFalse(checks['scanned_asset_hash_map_matches'])
        self.assertFalse(checks['scanned_manifest_hash_matches_source'])


if __name__ == '__main__':
    unittest.main()
