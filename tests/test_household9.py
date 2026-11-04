"""Explicit household detector profile; no hardware or motion required."""
import contextlib
from dataclasses import replace
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

from perception.fpga_replay import yolo_detector as yd, coco_zh
from perception.fpga_replay.household_names import NAMES, CLASS_ZH
from perception.fpga_replay.core import Config, Detection, Frame, process_frame
from host.ros_runtime import memory_config
from host import inference_service as svc
from test_yolo_detector import PureHelpers, HAVE_ORT, HAVE_TV, _CpuTorch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ui/geomem/bridge'))
import gmb_proto


def head():
    a = np.zeros((1, 13, 11), np.float32)
    for i in range(9):
        a[0, :4, i] = [40 + 60 * i, 300, 20, 40]
        a[0, 4+i, i] = .9
    a[0, :4, 9:] = a[0, :4, :1]
    a[0, 4, 9] = .8  # same class and position: suppressed
    a[0, 5, 10] = .7  # different class, same position: retained
    return a


class HouseholdProfile(unittest.TestCase):
    def test_class_order_boxes_and_class_aware_nms(self):
        lb = yd._board_module().Letterbox(640, 360, 640)
        ds = yd.postprocess_raw(head(), lb, .25, .7, 100, NAMES, 0)
        self.assertEqual([d['name'] for d in ds[:9]], list(NAMES))
        self.assertEqual(ds[0]['box'], [30., 140., 50., 180.])
        self.assertEqual(len(ds), 10)
        self.assertEqual(ds[-1]['name'], 'medicine bottle')
        with self.assertRaises(ValueError):
            yd.postprocess_raw(head(), lb, .25, .7, 100, yd.coco_names())
        bad = head(); bad[0, 12, 0] = np.nan
        with self.assertRaises(RuntimeError):
            yd.postprocess_raw(bad, lb, .25, .7, 100, NAMES, 0)

    @unittest.skipUnless(HAVE_TV, 'torch/torchvision unavailable')
    def test_board_postprocess_matches_numpy_without_protos(self):
        import torch, torchvision
        nd = yd._board_module()
        d = nd.Detector.__new__(nd.Detector)
        d.torch, d.tv = _CpuTorch(), torchvision
        d.m = SimpleNamespace(buf={'output0': torch.from_numpy(head())}, stream=None)
        d.lb = nd.Letterbox(640, 360, 640)
        d.conf, d.iou, d.max_det = .25, .7, 100
        d.names, d.mask_channels = list(NAMES), 0
        d.out_det, d.out_proto, d.proto_stride = 'output0', None, 0.
        self.assertEqual(d.postprocess(masks=False), yd.postprocess_raw(head(), d.lb, .25, .7, 100, NAMES, 0))
        with self.assertRaisesRegex(ValueError, 'no segmentation masks'):
            d.postprocess(masks=True)

    def test_profile_and_class_filter_reach_service(self):
        args = PureHelpers.service_args('--detector', 'yolo', '--onnx', 'm.onnx', '--yolo-profile', 'household9')
        with mock.patch.object(yd, 'YoloSegDetector') as ctor:
            svc.build_detector(args)
            self.assertEqual(ctor.call_args[1]['profile'], 'household9')
            self.assertEqual(ctor.call_args[1]['classes'], NAMES)
        self.assertEqual(yd.parse_classes(None, profile='household9'), NAMES)
        with self.assertRaises(ValueError):
            yd.parse_classes('bottle', profile='household9')
        self.assertEqual(yd.parse_classes(None), yd.default_classes())

    def test_chinese_queries_reach_new_labels_without_generic_bottle(self):
        cases = {'找药瓶': {'medicine bottle'}, '椅子旁边的药盒': {'medicine box'},
                 '找废纸团': {'waste paper'}, '钥匙和眼镜': {'key', 'glasses'},
                 '塑料包装袋': {'plastic packaging'}, '易拉罐': {'drink can'}}
        for phrase, labels in cases.items():
            sent, _ = gmb_proto.translate_find_text(phrase)
            self.assertEqual(coco_zh.goal_labels(sent, NAMES), labels, (phrase, sent))
        self.assertEqual(coco_zh.labels_in_text('药瓶'), ['medicine bottle'])
        for name, zh in CLASS_ZH.items():
            self.assertEqual(gmb_proto.class_zh(name), zh)
        cfg = memory_config(SimpleNamespace(yolo_profile='household9'))
        self.assertEqual(cfg.query_classes, NAMES)
        self.assertEqual(cfg.create_min_hits, Config().create_min_hits)
        self.assertEqual(memory_config(SimpleNamespace()), Config())

    def test_foreground_depth_and_invalid_depth_for_new_classes(self):
        depth = np.full((480, 640), 3., np.float32)
        depth[150:330, 300:340] = 2.30
        depth[200:250, 300:340] = 2.05
        f = Frame('h1', 1., np.zeros((480, 640, 3), np.uint8), depth,
                  np.array([[500., 0, 320], [0, 500., 240], [0, 0, 1]]), np.eye(4))
        for label in NAMES:
            d = Detection(label, .9, (300., 150., 340., 330.), frame_id='h1', timestamp=1.)
            obs = process_frame(f, [d])
            self.assertEqual(len(obs), 1)
            self.assertAlmostEqual(float(obs[0].position[2]), 2.05, delta=.03)
        f.depth[:] = np.nan
        self.assertEqual(process_frame(f, [d]), [])

    @unittest.skipUnless(HAVE_ORT, 'onnx/onnxruntime unavailable')
    def test_real_final_onnx_profile_and_phone(self):
        model = os.environ.get('HOUSEHOLD9_TEST_ONNX')
        photo = os.environ.get('HOUSEHOLD9_TEST_PHOTO')
        if not model or not photo:
            self.skipTest('real final model/phone fixture not configured')
        import cv2
        d = yd.YoloSegDetector(model, 'ort', profile='household9')
        rgb = cv2.cvtColor(cv2.imread(photo), cv2.COLOR_BGR2RGB)
        ds = d.raw_detections(rgb)
        phones = [x for x in ds if x['name'] == 'cell phone']
        self.assertTrue(phones)
        self.assertGreater(max(x['score'] for x in phones), .8)
        self.assertEqual(d.execution['names'], list(NAMES))
        self.assertEqual(d.class_min_scores, {})
        with self.assertRaises(ValueError):
            yd.YoloSegDetector(model, 'ort')

    def test_car_dry_run_passes_profile_to_both_consumers(self):
        env = dict(os.environ, YOLO_PROFILE='household9', CAR_ARCH='x86_64')
        r = subprocess.run(['bash', str(ROOT / 'tools/car_run/car_up.sh'), 'sim', '--dry-run'],
                           env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn('--yolo-profile household9', r.stdout)
        self.assertIn('YOLO_PROFILE=household9', r.stdout)


if __name__ == '__main__':
    unittest.main()
