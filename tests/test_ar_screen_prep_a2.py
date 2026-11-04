"""ar_screen.sh prep (2026-10-16, controller): loads m7_pl_merge_a2 and refuses to go on unless the read-back
BUILD_ID is 0x00da5041 (the car chain's bitstream, car_up.sh KV_BUILD_ID). Stubs: xmutil (records its argv)
and an m7pl_test that prints a chosen BUILD_ID; AR_SCREEN_BASE points $B/repo at this tree."""

import os
import stat
import subprocess
import tempfile
import shutil
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "kv_link" / "ar_screen.sh"


def _exe(path, text):
    path.write_text(text)
    path.chmod(path.stat().st_mode | stat.S_IXUSR)


class Prep(unittest.TestCase):
    def run_prep(self, build_id):
        tmp = Path(tempfile.mkdtemp(prefix="arprep_"))
        self.addCleanup(shutil.rmtree, str(tmp), True)
        (tmp / "base").mkdir()
        os.symlink(str(ROOT), str(tmp / "base" / "repo"))
        (tmp / "bin").mkdir()
        log = tmp / "xmutil.log"
        _exe(tmp / "bin" / "xmutil", '#!/bin/bash\necho "$*" >> %s\necho ok\n' % log)
        _exe(tmp / "m7pl_test", '#!/bin/bash\necho "sysinfo MAGIC=0x4d37504c VERSION=0x00010000 BUILD_ID=%s '
                                 'SLOT_MAP=0x3d7"\n' % build_id)
        env = dict(os.environ, PATH="%s:%s" % (tmp / "bin", os.environ["PATH"]),
                   AR_SCREEN_BASE=str(tmp / "base"), AR_SCREEN_M7PL_TEST=str(tmp / "m7pl_test"))
        r = subprocess.run(["bash", str(SCRIPT), "prep"], env=env, stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE, universal_newlines=True, timeout=60)
        return r, log.read_text().split("\n")

    def test_loads_a2_and_accepts_its_build_id(self):
        r, calls = self.run_prep("0x00da5041")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("loadapp m7_pl_merge_a2", calls)
        self.assertNotIn("loadapp m7_pl_merge", calls)
        self.assertIn("identity BUILD_ID=0x00da5041", r.stdout)
        self.assertIn("DP ", r.stdout)

    def test_refuses_a1_and_unread(self):
        for bid in ("0x07784f75", "0x0117bc0a", ""):
            r, _ = self.run_prep(bid)
            self.assertEqual(r.returncode, 3, (bid, r.stdout, r.stderr))
            self.assertIn("REFUSED", r.stderr)
            self.assertIn("xmutil loadapp m7_pl_merge_a2", r.stderr)
            self.assertNotIn("DP ", r.stdout)

    def test_upper_case_hex_accepted(self):
        r, _ = self.run_prep("0x00DA5041")
        self.assertEqual(r.returncode, 0, r.stderr)


if __name__ == "__main__":
    unittest.main()
