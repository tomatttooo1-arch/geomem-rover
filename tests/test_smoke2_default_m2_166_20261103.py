#!/usr/bin/env python3
"""SMOKE2 (2026-11-03): the car's default FPGA profile moves v3fc2 -> m2_166 after the in-place whole-chain smoke
(camera + Nano + m2_166, car not moving; runs/nationals-prelim-20261102/SMOKE2/README.md).

Pins three things (the {} subtests of the SW_M1 / SW_M2 plan-identity tests were dropped because the default moved
on purpose; these replace them):
  1. fpga_profiles.env: FPGA_PROFILE_DEFAULT=m2_166, the m2_166 entries unchanged;
  2. car_up with no FPGA_PROFILE plans exactly what FPGA_PROFILE=m2_166 plans (stdout and stderr);
  3. the old default stays one variable away: the pre-SMOKE2 tree's {} plan == this tree's FPGA_PROFILE=v3fc2 plan.
Positive control in-test: this tree's {} plan differs from its v3fc2 plan (so 2 would catch a default left at v3fc2).
Run from tests/ (python3 test_smoke2_default_m2_166_20261103.py)."""
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_swm1_profiles_20261103 as S1  # noqa: E402

ROOT = Path(HERE).parent
PRE_TAG = "nat-smoke2-pre-20261103"


class ProfileFile(unittest.TestCase):
    def test_default_is_m2_166(self):
        kv = S1.env_file()
        self.assertEqual(kv["FPGA_PROFILE_DEFAULT"], "m2_166")
        self.assertEqual((kv["M2_166_APP"], kv["M2_166_BUILD_ID"], kv["M2_166_KV_ROLLING"], kv["M2_166_NANO_ROLLING"],
                          kv["M2_166_AR"]), ("m7_pl_m2_166", "0x07da3d4a", "pl", "remote_pl", "nano"))
        self.assertEqual((kv["V3FC2_APP"], kv["V3FC2_BUILD_ID"]), ("m7_pl_v3fc2", "0x03c087c9"))   # rollback still there


class CarUpPlan(S1.CarUpPlan):
    def test_m1_profiles(self):
        self.skipTest("SW_M1 test")

    def test_older_profiles_plan_identical_to_pre_change(self):
        self.skipTest("SW_M1 test")

    def test_default_plan_is_m2_166(self):
        d, m = self.plan(), self.plan(FPGA_PROFILE="m2_166")
        self.assertEqual((d.returncode, m.returncode), (0, 0), d.stderr + m.stderr)
        self.assertEqual(S1.norm(d.stdout, ROOT), S1.norm(m.stdout, ROOT))
        self.assertEqual(S1.norm(d.stderr, ROOT), S1.norm(m.stderr, ROOT))
        self.assertIn("--kv-expect-build-id 0x07da3d4a", self.t.producer(d.stdout))
        self.assertIn("--ar-nano", self.t.producer(d.stdout))
        # positive control: the v3fc2 plan is different, so a default left at v3fc2 fails the check above
        v = self.plan(FPGA_PROFILE="v3fc2")
        self.assertEqual(v.returncode, 0, v.stderr)
        self.assertNotEqual(S1.norm(d.stdout, ROOT), S1.norm(v.stdout, ROOT))

    def test_old_default_is_v3fc2_explicitly(self):
        have = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "-q", "--verify", PRE_TAG + "^{commit}"],
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, universal_newlines=True)
        if have.returncode != 0:
            self.skipTest("tag %s not in this clone" % PRE_TAG)
        pre = os.path.join(tempfile.mkdtemp(prefix="smoke2pre_"), "tree")
        self.addCleanup(shutil.rmtree, os.path.dirname(pre), True)
        add = subprocess.run(["git", "-C", str(ROOT), "worktree", "add", "-q", "--detach", pre, PRE_TAG],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        self.assertEqual(add.returncode, 0, add.stdout)
        self.addCleanup(subprocess.run, ["git", "-C", str(ROOT), "worktree", "remove", "--force", pre],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        pre_up = os.path.join(pre, "tools/car_run/car_up.sh")
        a, b = self.plan(up=pre_up), self.plan(FPGA_PROFILE="v3fc2")
        self.assertEqual((a.returncode, b.returncode), (0, 0), a.stderr + b.stderr)
        self.assertEqual(S1.norm(a.stdout, pre), S1.norm(b.stdout, ROOT))
        self.assertEqual(S1.norm(a.stderr, pre), S1.norm(b.stderr, ROOT))
        # and the pre-SMOKE2 default plan is NOT this tree's default plan (the switch really changes something)
        self.assertNotEqual(S1.norm(a.stdout, pre), S1.norm(self.plan().stdout, ROOT))


if __name__ == "__main__":
    unittest.main()
