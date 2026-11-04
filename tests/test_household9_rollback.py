"""Execute rollback preflight in an isolated filesystem; no SSH, camera or service changes."""
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class RollbackPreflight(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        self.root = self.base / 'deployment'
        self.root.mkdir()
        self.link = self.base / 'current'
        self.engine = self.base / 'original.engine'
        self.engine.write_bytes(b'original-engine')
        self.old = self.backup('baseline', 'old')
        self.new = self.backup('baselines/new-release', 'new')
        self.link.symlink_to(self.old)
        (self.root/'staged-release.txt').write_text(str(self.base/'candidate'))
        source = (ROOT/'tools/nano_detector/rollback_household9.sh').read_text()
        source = source.replace('root=/data/fpga/dev3/household9-20261027', 'root='+str(self.root))
        source = source.replace('link=/home/robot/fpga/dev1/console-current', 'link='+str(self.link))
        source = source.replace('/data/fpga/yolo_engine_20261015/yolov8n-seg-640-fp16.engine', str(self.engine))
        source = source.replace('/data/fpga/car/state/car.current', str(self.base/'car.current'))
        self.script = self.base/'rollback.sh'
        self.script.write_text(source)

    def backup(self, relative, commit):
        folder = self.root/relative
        folder.mkdir(parents=True)
        release = self.base/('release-'+commit)
        release.mkdir()
        (release/'COMMIT').write_text(commit)
        (folder/'COMMIT').write_text(commit)
        (folder/'previous-release.txt').write_text(str(release))
        (folder/'yolov8n-seg-640-fp16.engine').write_bytes(b'original-engine')
        files = [{'path':p.name, 'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
                 for p in folder.iterdir() if p.is_file()]
        (folder/'backup-complete.json').write_text(json.dumps({'files':files}))
        return release

    def check(self):
        before = self.link.resolve()
        result = subprocess.run(['bash', str(self.script), '--check'], stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, universal_newlines=True, timeout=10)
        self.assertEqual(self.link.resolve(), before, 'preflight must not switch releases')
        return result

    def test_original_backup_fallback(self):
        r = self.check()
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_new_default_uses_new_immutable_backup(self):
        (self.root/'active-baseline.txt').write_text(str(self.root/'baselines/new-release'))
        self.link.unlink(); self.link.symlink_to(self.new)
        r = self.check()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual((self.root/'baseline/COMMIT').read_text(), 'old')

    def test_unrelated_default_is_never_stopped(self):
        self.link.unlink(); self.link.symlink_to(self.new)
        self.assertEqual(self.check().returncode, 8)

    def test_outside_backup_path_is_refused(self):
        (self.root/'active-baseline.txt').write_text(str(self.base))
        self.assertEqual(self.check().returncode, 8)

    def test_corrupt_backup_cannot_pass(self):
        (self.root/'baseline/yolov8n-seg-640-fp16.engine').write_bytes(b'corrupt')
        self.assertNotEqual(self.check().returncode, 0)


if __name__ == '__main__':
    unittest.main()
