"""Same-frame dual model regression: bottle retained, household added, exclusions honoured."""
from dataclasses import replace
from pathlib import Path
import os
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
from perception.fpga_replay import yolo_detector as yd, coco_zh
from perception.fpga_replay.household_names import NAMES
from perception.fpga_replay.core import Detection
from host import inference_service as svc
from host.ros_runtime import memory_config
from test_yolo_detector import PureHelpers, HAVE_ORT, constant_onnx, head, frame

ROOT = Path(__file__).resolve().parents[1]

class CombinedTests(unittest.TestCase):
    def test_vocabulary_and_legacy_profiles(self):
        names = yd.parse_classes(None, profile='combined')
        self.assertEqual(len(names), 22)  # 15 COCO + 9 household - 2 shared (umbrella off, owner 10-27 evening)
        self.assertNotIn('umbrella', names)
        self.assertEqual(coco_zh.goal_labels('找伞', names), set())
        self.assertEqual(coco_zh.goal_labels('找雨伞', names), set())
        self.assertEqual(set(names), set(yd.default_classes()) | set(NAMES))
        self.assertTrue({'bottle', 'cup', 'medicine bottle', 'key', 'glasses'} <= set(names))
        self.assertFalse(set(yd.DEFAULT_EXCLUDE_CLASSES) & set(names))
        self.assertEqual(yd.parse_classes(None, profile='household9'), NAMES)
        self.assertEqual(len(yd.parse_classes(None, (), 'combined')), 32)
        self.assertEqual(memory_config(SimpleNamespace(yolo_profile='combined')).query_classes, names)
        for query, expected in [('找瓶子', 'bottle'), ('找药瓶', 'medicine bottle'), ('找眼镜', 'glasses')]:
            self.assertEqual(coco_zh.goal_labels(query, names), {expected})

    def test_same_frame_merge_keeps_different_classes_and_distinct_phones(self):
        f = frame()
        def d(label, score, box=(10, 10, 30, 30)):
            return Detection(label, score, box, frame_id=f.frame_id, timestamp=f.timestamp)
        def model(items):
            m = mock.Mock(return_value=items)
            m.execution = {'device': 'cpu'}
            return m
        primary = model([d('cell phone', .9), d('key', .8)])
        original = model([d('cell phone', .7), d('bottle', .85), d('cell phone', .6, (50, 50, 70, 70))])
        merged = yd.CombinedDetector([primary, original])
        ds = merged(f)
        self.assertEqual([x.label for x in ds], ['cell phone', 'bottle', 'key', 'cell phone'])
        primary.assert_called_once_with(f)
        original.assert_called_once_with(f)
        self.assertEqual(len(merged.execution['model_seconds']), 2)
        original.return_value = [replace(d('bottle', .9), frame_id='old')]
        with self.assertRaisesRegex(ValueError, 'foreign frame'):
            merged(f)

    def test_startup_uses_two_model_paths(self):
        env = dict(os.environ, YOLO_PROFILE='combined', YOLO_COCO_MODEL='/models/original.onnx', CAR_ARCH='x86_64')
        r = subprocess.run(['bash', 'tools/car_run/car_up.sh', 'sim', '--dry-run'], cwd=str(ROOT),
                           env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        self.assertEqual(r.returncode, 0, r.stdout+r.stderr)
        self.assertIn('--coco-model /models/original.onnx', r.stdout)
        self.assertIn('--yolo-profile combined', r.stdout)
        self.assertIn('YOLO_PROFILE=combined', r.stdout)

    @unittest.skipUnless(HAVE_ORT, 'ONNX/ORT unavailable')
    def test_both_real_onnx_sessions_and_ipc_reply(self):
        import onnx
        with tempfile.TemporaryDirectory() as tmp:
            # Original model outputs bottle, duplicate phone, and all nine disabled classes.
            coco_rows = [(100, 300, 40, 80, yd.coco_names().index('bottle'), .9),
                         (300, 300, 40, 80, yd.coco_names().index('cell phone'), .7)]
            coco_rows += [(400, 300, 30, 60, yd.coco_names().index(n), .95) for n in yd.DEFAULT_EXCLUDE_CLASSES]
            original = constant_onnx(os.path.join(tmp, 'original.onnx'), head(coco_rows))
            raw = np.zeros((1, 13, 11), np.float32)
            raw[0, :4, 0] = [300, 300, 40, 80]; raw[0, 12, 0] = .9  # phone
            raw[0, :4, 1] = [200, 300, 40, 80]; raw[0, 7, 1] = .85  # key
            household = constant_onnx(os.path.join(tmp, 'household.onnx'), raw)
            graph = onnx.load(household)
            del graph.graph.output[1]
            onnx.save(graph, household)
            args = PureHelpers.service_args('--detector', 'yolo', '--onnx', household,
                                             '--coco-model', original, '--yolo-profile', 'combined')
            detector = svc.build_detector(args)
            f = frame()
            reply = svc.process_request(svc.encode_frame(f), detector, 'cpu')
            labels = [d['label'] for d in reply['detections']]
            self.assertEqual(set(labels), {'bottle', 'key', 'cell phone'})
            self.assertEqual(labels.count('cell phone'), 1)
            self.assertEqual(reply['execution']['profile'], 'combined')
            self.assertEqual(len(reply['execution']['models']), 2)
            self.assertTrue(all(d['frame_id'] == f.frame_id for d in reply['detections']))

if __name__ == '__main__':
    unittest.main()
