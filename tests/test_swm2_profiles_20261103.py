"""SW_M2 (nationals 2026-11-03): the MERGE2 bitstream's two profiles m2_166 / m2 go into the car start chain.

* fpga_profiles.env: M2_166 / M2 (m7_pl_m2_166 / m7_pl_m2, BUILD_ID 0x07da3d4a, KV rolling pl, Nano remote_pl,
  <P>_AR=nano, as M1_166 / M1); the default stays v3fc2.
* fpga_profile.sh: m2_166 / m2 export their app / build / AR=nano; BUILD_ID 0x07da3d4a -> m2.
* car_up (board dry run): m2_166 / m2 -> KV precheck 0x07da3d4a, producer remote_pl + --ar-nano; every profile that
  existed before (default, v3fc2, v3fc2_166, a2, v3fc, m1_166, m1, with and without the screen switches) plans
  byte-identical to nat-swm1-post-20261103 (when that tag is in this clone).
* kv_start.sh: m2_166 / m2 load their own app and serve --rolling pl, a drm request goes to the Nano.
* kv_service.py / install_kv_service.sh: the two words are accepted (sudoers too), load_profiles has them.
* nano_up.sh: the KV helper gets "m2_166"; the car screen comes on on the Nano.

Run: PYTHONPATH=. python3 -m unittest discover -s tests -p test_swm2_profiles_20261103.py"""
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = Path(HERE).parent
sys.path.insert(0, HERE)
sys.path.insert(0, str(ROOT))
import test_car_run as T  # noqa: E402
import test_competition_nano_modes as nm  # noqa: E402
import test_swm1_profiles_20261103 as S1  # noqa: E402

POST1_TAG = "nat-swm1-post-20261103"
PROF_SH = ROOT / "tools/car_run/fpga_profile.sh"
M2 = (("m2_166", "m7_pl_m2_166"), ("m2", "m7_pl_m2"))


class ProfileFile(unittest.TestCase):
    def test_entries_and_default(self):
        kv = S1.env_file()
        self.assertEqual(kv["FPGA_PROFILE_DEFAULT"], "m2_166")         # SW_M2 left v3fc2; SMOKE2 (11-03) switched it
        for p, app in (("M2_166", "m7_pl_m2_166"), ("M2", "m7_pl_m2")):
            self.assertEqual((kv[p + "_APP"], kv[p + "_BUILD_ID"], kv[p + "_KV_ROLLING"], kv[p + "_NANO_ROLLING"],
                              kv[p + "_AR"]), (app, "0x07da3d4a", "pl", "remote_pl", "nano"))
        self.assertEqual([k for k in kv if k.endswith("_AR")], ["M1_AR", "M1_166_AR", "M2_AR", "M2_166_AR"])

    def test_profile_sh_exports(self):
        r = subprocess.run(["bash", "-c", 'source "$1"; for p in m2_166 m2 m1_166 v3fc2; do '
                            'fpga_profile $p; echo "$FPGA_PROFILE $FPGA_APP $FPGA_BUILD_ID '
                            '$FPGA_KV_ROLLING $FPGA_NANO_ROLLING $FPGA_AR"; done; '
                            'fpga_profile_of_build 0x07da3d4a; fpga_profile_of_build 0x08051f22; '
                            'fpga_profile; echo "$FPGA_PROFILE"', "_", str(PROF_SH)],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=30,
                           env={k: v for k, v in os.environ.items() if k != "FPGA_PROFILE"})
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(r.stderr, "")
        self.assertEqual(r.stdout.splitlines(), [
            "m2_166 m7_pl_m2_166 0x07da3d4a pl remote_pl nano",
            "m2 m7_pl_m2 0x07da3d4a pl remote_pl nano",
            "m1_166 m7_pl_m1_166 0x08051f22 pl remote_pl nano",
            "v3fc2 m7_pl_v3fc2 0x03c087c9 pl remote_pl kv",
            "m2", "m1", "v3fc2"])


class CarUpPlan(S1.CarUpPlan):
    def test_m1_profiles(self):          # covered in test_swm1_profiles; here the m2 ones
        self.skipTest("SW_M1 test")

    def test_older_profiles_plan_identical_to_pre_change(self):
        self.skipTest("SW_M1 test")

    def test_m2_profiles(self):
        for prof, _app in M2:
            with self.subTest(prof=prof):
                r = self.plan(FPGA_PROFILE=prof)
                self.assertEqual(r.returncode, 0, r.stderr)
                prod = self.t.producer(r.stdout)
                self.assertIn("--rolling-backend remote_pl", prod)
                self.assertIn("--kv-expect-build-id 0x07da3d4a", prod)
                self.assertIn("--ar-nano", prod)
                self.assertIn("--expect-build-id 0x07da3d4a", self.t.line(r.stdout, "[kv precheck]"))
                stack = self.t.line(r.stdout, "[container stack]")
                self.assertIn("-e ROLLING_BACKEND=remote_pl", stack)
                self.assertIn("--device /dev/dri", stack)
                # same plan as m1 apart from the build id (same backends, same Nano screen)
                m1 = self.plan(FPGA_PROFILE=prof.replace("m2", "m1"))
                self.assertEqual(S1.norm(r.stdout), S1.norm(m1.stdout).replace("0x08051f22", "0x07da3d4a"))

    def test_existing_profiles_plan_identical_to_swm1_post(self):
        have = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "-q", "--verify", POST1_TAG + "^{commit}"],
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, universal_newlines=True)
        if have.returncode != 0:
            self.skipTest("tag %s not in this clone" % POST1_TAG)
        pre = os.path.join(tempfile.mkdtemp(prefix="swm2pre_"), "tree")
        self.addCleanup(shutil.rmtree, os.path.dirname(pre), True)
        add = subprocess.run(["git", "-C", str(ROOT), "worktree", "add", "-q", "--detach", pre, POST1_TAG],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        self.assertEqual(add.returncode, 0, add.stdout)
        self.addCleanup(subprocess.run, ["git", "-C", str(ROOT), "worktree", "remove", "--force", pre],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        pre_up = os.path.join(pre, "tools/car_run/car_up.sh")
        # {} left out since SMOKE2 (11-03): the default is m2_166 now (pinned in test_smoke2_default_m2_166)
        for env in (dict(FPGA_PROFILE="v3fc2"), dict(FPGA_PROFILE="v3fc2_166"),
                    dict(FPGA_PROFILE="a2"), dict(FPGA_PROFILE="v3fc"),
                    dict(FPGA_PROFILE="m1_166"), dict(FPGA_PROFILE="m1"),
                    dict(FPGA_PROFILE="v3fc2", CAR_AR_COLOUR="1"), dict(FPGA_PROFILE="v3fc2", CAR_AR_NANO="1"),
                    dict(FPGA_PROFILE="m1_166", CAR_AR_NANO="0")):
            with self.subTest(**env):
                a, b = self.plan(up=pre_up, **env), self.plan(**env)
                self.assertEqual((a.returncode, b.returncode), (0, 0), a.stderr + b.stderr)
                self.assertEqual(S1.norm(a.stdout, pre), S1.norm(b.stdout, ROOT))
                self.assertEqual(S1.norm(a.stderr, pre), S1.norm(b.stderr, ROOT))


class KvStart(S1.KvStart):
    def run_start(self, *extra, slot_map=None):
        from test_competition_kv_service import KvStartTests
        t = KvStartTests("test_no_argument_is_old_a2_command")
        orig = subprocess.run

        def run(cmd, *a, **kw):
            if "kv_start.sh" in " ".join(map(str, cmd)):
                fw = Path(kw["env"]["FPGA_FW_DIR"])
                for _p, app in M2:
                    (fw / app).mkdir(parents=True, exist_ok=True)
            return orig(cmd, *a, **kw)
        subprocess.run = run
        try:
            r, calls, old = t.run_start(*extra)
        finally:
            subprocess.run = orig
        t.doCleanups()
        return r, calls, old

    def test_m1_loads_its_app_and_serves_rolling(self):
        self.skipTest("SW_M1 test")

    def test_v3fc2_drm_unchanged(self):
        self.skipTest("SW_M1 test")

    def test_m2_loads_its_app_and_serves_rolling(self):
        for prof, app in M2:
            for ar in ("off", "drm"):
                with self.subTest(prof=prof, ar=ar):
                    r, calls, old = self.run_start(prof, ar)
                    self.assertEqual(r.returncode, 0, r.stderr)
                    self.assertEqual(calls, ["xmutil unloadapp", "xmutil loadapp " + app,
                                             "kv_board " + old + " --rolling pl"])     # never --ar drm
                    self.assertEqual("AR_ON_NANO profile=%s" % prof in r.stdout, ar == "drm", r.stdout)
                    self.assertNotIn("AR_DISABLED", r.stdout + r.stderr)


class KvService(unittest.TestCase):
    def test_profiles_and_words(self):
        sys.path.insert(0, str(ROOT / "ui/geomem/tools/competition"))
        import kv_service
        p = kv_service.load_profiles(ROOT / "tools/car_run/fpga_profiles.env")
        self.assertEqual(p["m2_166"], {"app": "m7_pl_m2_166", "build": "0x07da3d4a", "kv_rolling": "pl"})
        self.assertEqual(p["m2"], {"app": "m7_pl_m2", "build": "0x07da3d4a", "kv_rolling": "pl"})
        src = (ROOT / "ui/geomem/tools/competition/kv_service.py").read_text()
        self.assertIn("'m1_166', 'm1', 'm2_166', 'm2', 'stop')", src)
        sudo = (ROOT / "ui/geomem/tools/competition/install_kv_service.sh").read_text()
        self.assertIn("%s m1_166, %s m1, %s m2_166, %s m2, %s stop", sudo)

    def test_sudoers_line_has_one_helper_per_word(self):
        # the printf in install_kv_service.sh: as many "$helper" arguments as %s words
        sudo = (ROOT / "ui/geomem/tools/competition/install_kv_service.sh").read_text().splitlines()
        i = next(n for n, l in enumerate(sudo) if "NOPASSWD:" in l and "printf" in l)
        self.assertEqual(sudo[i].count("%s"), sudo[i + 1].count('"$helper"'))


@unittest.skipUnless(nm.find_bash(), "bash")
class NanoUp(S1.NanoUp):
    def test_m1_profile_word_and_absent_reason(self):
        self.skipTest("SW_M1 test")

    def test_reused_m1_server_by_build(self):
        self.skipTest("SW_M1 test")

    def test_switched_off_drm_and_old_builds_stay_off(self):
        self.skipTest("SW_M1 test")

    def test_m2_profile_word(self):
        kv = '{"ar": "off", "ar_reason": "ar_core_absent: profile m2_166 has no AR core (car screen on the Nano)", ' \
             '"build_id": "0x07da3d4a"}'
        r, ups = self.run_up(kv, FPGA_PROFILE="m2_166")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertTrue(self.ssh_lines()[0].endswith("geomem-kv-start m2_166"), self.ssh_lines())
        self.assertTrue(all(c.startswith("car_up|nano=1|colour=1|") for c in ups), ups)

    def test_reused_m2_server_by_build(self):
        kv = '{"ar": "off", "ar_reason": "ar_start_failed rc=1: x", "build_id": "0x07da3d4a", "reused": true}'
        r, ups = self.run_up(kv, FPGA_PROFILE="m2_166")
        self.assertIn("AR_STATUS nano: the KV serves BUILD_ID 0x07da3d4a (profile m2", r.stdout)
        self.assertTrue(all(c.startswith("car_up|nano=1|colour=1|") for c in ups), ups)


if __name__ == "__main__":
    unittest.main()
