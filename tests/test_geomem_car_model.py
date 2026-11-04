"""实车外观模型（10-21）：两份副本一致、三个三维页面都接上、控制台离线缓存清单含新文件。

视觉核对用无头 Chrome 截图人工看过（展台跟车/近景、控制台全局/近景），这里只守住接线不被改坏。
"""
import os
import re
import shutil
import subprocess
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
G = os.path.join(ROOT, 'ui', 'geomem')
APP_MODEL = os.path.join(G, 'app', 'src', 'car_model.js')
OP_MODEL = os.path.join(G, 'operator', 'src', 'car_model.js')


def read(p):
    with open(p, encoding='utf-8') as f:
        return f.read()


class CarModelTest(unittest.TestCase):
    def test_copies_identical(self):
        with open(APP_MODEL, 'rb') as a, open(OP_MODEL, 'rb') as b:
            self.assertEqual(a.read(), b.read(), '两份 car_model.js 必须同字节（各页面只托管自己的目录）')

    def test_pages_use_model(self):
        for rel in ('app/src/kiosk_scene.js', 'app/src/scene.js', 'operator/src/scene.js'):
            s = read(os.path.join(G, rel))
            self.assertRegex(s, r"import \{[^}]*buildCarModel[^}]*\} from '\./car_model\.js'", rel)
            self.assertIn('buildCarModel(', s, rel)
            self.assertNotIn('CylinderGeometry(.10,.10,.07,16)', s, rel + ' 旧方块车轮应已删除')

    def test_real_dimensions(self):
        s = read(APP_MODEL)
        self.assertIn('const R = 0.0405', s)                       # 轮径 0.081（D-041）
        self.assertIn('[0.1, 0.1025]', s)                          # 左右轮心距 0.205
        m = re.search(r'CAR_CAMERA = \{ x: ([0-9.]+), h: ([0-9.]+), pitchDeg: ([0-9.]+) \}', s)
        self.assertIsNotNone(m)
        self.assertEqual((float(m.group(2)), float(m.group(3))), (0.39, 7.0))   # 10-17 实测相机

    def test_operator_sw_caches_model(self):
        s = read(os.path.join(G, 'operator', 'sw.js'))
        self.assertIn("'./src/car_model.js'", s)

    @unittest.skipUnless(shutil.which('node'), 'node 不在 PATH')
    def test_syntax(self):
        for p in (APP_MODEL, os.path.join(G, 'app', 'src', 'kiosk_scene.js'), os.path.join(G, 'operator', 'src', 'scene.js')):
            r = subprocess.run(['node', '--check', p], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self.assertEqual(r.returncode, 0, r.stdout.decode('utf-8', 'replace'))


if __name__ == '__main__':
    unittest.main()
