"""SW_M1 (nationals 2026-11-03): the next bitstream's two profiles m1_166 / m1 go into the car start chain.

* fpga_profiles.env: M1_166 / M1 (m7_pl_m1_166 / m7_pl_m1, BUILD_ID 0x08051f22, KV rolling pl, Nano remote_pl,
  <P>_AR=nano); the default stays v3fc2.
* fpga_profile.sh exports FPGA_AR (nano for m1*, kv for every older profile).
* car_up (board dry run): m1_166 / m1 -> KV precheck 0x08051f22, producer remote_pl + --ar-nano, stack /dev/dri;
  CAR_AR_NANO=0 wins; v3fc2 / default / a2 / v3fc2_166 plans byte-identical to the pre-change tree (when the
  pre tag nat-swm1-pre-20261103 is in this clone).
* kv_start.sh: m1_166 / m1 load their own app and serve --rolling pl; a drm request never tries the KV AR start
  (AR_ON_NANO, reason ar_core_absent) even with an unreadable SLOT_MAP; v3fc2 + drm unchanged.
* kv_service.py: the two words are accepted, load_profiles has them.
* nano_up.sh: the Nano screen also comes on when the KV serves a build registered as AR=nano (reused server with
  another reason), never when the KV runs drm or the screen was switched off; the KV helper gets "m1_166".

Run: PYTHONPATH=. python3 -m unittest discover -s tests -p test_swm1_profiles_20261103.py"""
import json
import os
import re
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
import test_ar2nano2_20261102 as a2n  # noqa: E402
import test_car_run as T  # noqa: E402
import test_competition_nano_modes as nm  # noqa: E402

PRE_TAG = "nat-swm1-pre-20261103"
PROF_SH = ROOT / "tools/car_run/fpga_profile.sh"


def norm(s, tree=None):
    """Run directories and time stamps differ between two dry runs (AR2NANO2 dryrun_cmp.py rules)."""
    if tree:
        s = s.replace(str(tree), "<TREE>")
    s = re.sub(r"[0-9a-f]{40}", "<SHA>", s)
    s = re.sub(r"\d{8}[-_]\d{6}(-\d+)?", "<TS>", s)
    return re.sub(r"[0-9][0-9-]*-live", "RUN-live", s)


def env_file():
    kv = {}
    for line in (ROOT / "tools/car_run/fpga_profiles.env").read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            k, v = line.split("=", 1)
            kv[k] = v
    return kv


class ProfileFile(unittest.TestCase):
    def test_entries_and_default(self):
        kv = env_file()
        self.assertEqual(kv["FPGA_PROFILE_DEFAULT"], "m2_166")         # SW_M1 left v3fc2; SMOKE2 (11-03) switched it
        for p, app in (("M1_166", "m7_pl_m1_166"), ("M1", "m7_pl_m1")):
            self.assertEqual((kv[p + "_APP"], kv[p + "_BUILD_ID"], kv[p + "_KV_ROLLING"], kv[p + "_NANO_ROLLING"],
                              kv[p + "_AR"]), (app, "0x08051f22", "pl", "remote_pl", "nano"))
        self.assertEqual([k for k in kv if k.endswith("_AR")][:2], ["M1_AR", "M1_166_AR"])   # no older profile has it (SW_M2 adds M2*)

    def test_profile_sh_exports(self):
        r = subprocess.run(["bash", "-c", 'source "$1"; for p in m1_166 m1 v3fc2 v3fc2_166 v3fc a2; do '
                            'fpga_profile $p; echo "$FPGA_PROFILE $FPGA_APP $FPGA_BUILD_ID '
                            '$FPGA_KV_ROLLING $FPGA_NANO_ROLLING $FPGA_AR"; done; '
                            'fpga_profile_of_build 0x08051f22', "_", str(PROF_SH)],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=30)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(r.stderr, "")
        self.assertEqual(r.stdout.splitlines(), [
            "m1_166 m7_pl_m1_166 0x08051f22 pl remote_pl nano",
            "m1 m7_pl_m1 0x08051f22 pl remote_pl nano",
            "v3fc2 m7_pl_v3fc2 0x03c087c9 pl remote_pl kv",
            "v3fc2_166 m7_pl_v3fc2_166 0x03c087c9 pl remote_pl kv",
            "v3fc m7_pl_v3fc 0x001256b1 pl remote_pl kv",
            "a2 m7_pl_merge_a2 0x00da5041 none cpu kv",
            "m1"])


class CarUpPlan(unittest.TestCase):
    """Board live dry runs (the TestFpgaProfile set-up of test_car_run)."""

    def setUp(self):
        self.t = T.TestFpgaProfile("test_unknown_profile_warns_and_uses_default")
        self.t.setUp()
        self.addCleanup(self.t.tearDown)

    def plan(self, up=None, **env):
        old = T.UP
        if up:
            T.UP = up
        try:
            return self.t.plan(**env)
        finally:
            T.UP = old

    def test_m1_profiles(self):
        for prof in ("m1_166", "m1"):
            with self.subTest(prof=prof):
                r = self.plan(FPGA_PROFILE=prof)
                self.assertEqual(r.returncode, 0, r.stderr)
                prod = self.t.producer(r.stdout)
                self.assertIn("--rolling-backend remote_pl", prod)
                self.assertIn("--kv-expect-build-id 0x08051f22", prod)
                self.assertIn("--ar-nano", prod)
                self.assertIn("--expect-build-id 0x08051f22", self.t.line(r.stdout, "[kv precheck]"))
                stack = self.t.line(r.stdout, "[container stack]")
                self.assertIn("-e ROLLING_BACKEND=remote_pl", stack)
                self.assertIn("--device /dev/dri", stack)
                self.assertIn("# car screen on this Nano (CAR_AR_NANO=1)", r.stdout)
                off = self.plan(FPGA_PROFILE=prof, CAR_AR_NANO="0")          # the caller wins
                self.assertEqual(off.returncode, 0, off.stderr)
                self.assertNotIn("--ar-nano", self.t.producer(off.stdout))
                self.assertNotIn("/dev/dri", off.stdout)
                self.assertNotIn("# car screen on this Nano", off.stdout)
                # with the Nano screen forced off, an m1 plan differs from v3fc2 only in the build id
                v3 = self.plan(FPGA_PROFILE="v3fc2")
                self.assertEqual(norm(off.stdout).replace("0x08051f22", "0x03c087c9"), norm(v3.stdout))

    def test_older_profiles_plan_identical_to_pre_change(self):
        have = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "-q", "--verify", PRE_TAG + "^{commit}"],
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, universal_newlines=True)
        if have.returncode != 0:
            self.skipTest("pre-change tag %s not in this clone" % PRE_TAG)
        pre = os.path.join(tempfile.mkdtemp(prefix="swm1pre_"), "tree")    # a git work tree, as the current one
        self.addCleanup(shutil.rmtree, os.path.dirname(pre), True)
        add = subprocess.run(["git", "-C", str(ROOT), "worktree", "add", "-q", "--detach", pre, PRE_TAG],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        self.assertEqual(add.returncode, 0, add.stdout)
        self.addCleanup(subprocess.run, ["git", "-C", str(ROOT), "worktree", "remove", "--force", pre],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        pre_up = os.path.join(pre, "tools/car_run/car_up.sh")
        # {} (the file default) left out since SMOKE2 (11-03) moved the default to m2_166 on purpose: the old
        # default plan is pinned by test_smoke2_default_m2_166 (pre-SMOKE2 {} == now FPGA_PROFILE=v3fc2)
        for env in (dict(FPGA_PROFILE="v3fc2"), dict(FPGA_PROFILE="v3fc2_166"),
                    dict(FPGA_PROFILE="a2"), dict(FPGA_PROFILE="v3fc"),
                    dict(FPGA_PROFILE="v3fc2", CAR_AR_COLOUR="1"), dict(FPGA_PROFILE="v3fc2", CAR_AR_NANO="1")):
            with self.subTest(**env):
                a, b = self.plan(up=pre_up, **env), self.plan(**env)
                self.assertEqual((a.returncode, b.returncode), (0, 0), a.stderr + b.stderr)
                self.assertEqual(norm(a.stdout, pre), norm(b.stdout, ROOT))
                self.assertEqual(norm(a.stderr, pre), norm(b.stderr, ROOT))


class KvStart(unittest.TestCase):
    def run_start(self, *extra, slot_map=None):
        from test_competition_kv_service import KvStartTests
        t = KvStartTests("test_no_argument_is_old_a2_command")
        orig = subprocess.run

        def run(cmd, *a, **kw):
            if "kv_start.sh" in " ".join(map(str, cmd)):
                fw = Path(kw["env"]["FPGA_FW_DIR"])
                for app in ("m7_pl_m1_166", "m7_pl_m1"):
                    (fw / app).mkdir(parents=True, exist_ok=True)
                kw["env"] = dict(kw["env"], **({"GEOMEM_KV_SLOT_MAP": slot_map} if slot_map else {}))
            return orig(cmd, *a, **kw)
        subprocess.run = run
        try:
            r, calls, old = t.run_start(*extra)
        finally:
            subprocess.run = orig
        t.doCleanups()
        return r, calls, old

    def test_m1_loads_its_app_and_serves_rolling(self):
        for prof, app in (("m1_166", "m7_pl_m1_166"), ("m1", "m7_pl_m1")):
            for ar in ("off", "drm"):
                with self.subTest(prof=prof, ar=ar):
                    r, calls, old = self.run_start(prof, ar)
                    self.assertEqual(r.returncode, 0, r.stderr)
                    self.assertEqual(calls, ["xmutil unloadapp", "xmutil loadapp " + app,
                                             "kv_board " + old + " --rolling pl"])     # never --ar drm
                    self.assertEqual("AR_ON_NANO profile=%s" % prof in r.stdout, ar == "drm", r.stdout)
                    self.assertNotIn("AR_DISABLED", r.stdout + r.stderr)

    def test_v3fc2_drm_unchanged(self):
        r, calls, old = self.run_start("v3fc2", "drm")                 # no helper in the fake repo: old fallback
        self.assertIn("AR_DISABLED reason=helper_missing", r.stdout)
        self.assertNotIn("AR_ON_NANO", r.stdout)
        self.assertEqual(calls[-1], "kv_board " + old.replace("run1", "run1-ar-off") + " --rolling pl")
        r, calls, old = self.run_start("v3fc2", "drm", slot_map="0x263")  # the AR2NANO SLOT_MAP rule as before
        self.assertIn("AR_ON_NANO slot_map=0x263", r.stdout)


class KvService(unittest.TestCase):
    def test_profiles_and_words(self):
        sys.path.insert(0, str(ROOT / "ui/geomem/tools/competition"))
        import kv_service
        p = kv_service.load_profiles(ROOT / "tools/car_run/fpga_profiles.env")
        self.assertEqual(p["m1_166"], {"app": "m7_pl_m1_166", "build": "0x08051f22", "kv_rolling": "pl"})
        self.assertEqual(p["m1"], {"app": "m7_pl_m1", "build": "0x08051f22", "kv_rolling": "pl"})
        src = (ROOT / "ui/geomem/tools/competition/kv_service.py").read_text()
        self.assertIn("'m1_166', 'm1', ", src)
        sudo = (ROOT / "ui/geomem/tools/competition/install_kv_service.sh").read_text()
        self.assertIn("%s m1_166, %s m1, ", sudo)


@unittest.skipUnless(nm.find_bash(), "bash")
class NanoUp(a2n.NanoUpArNano):
    def setUp(self):
        super().setUp()
        d = self.root / "tools/car_run"
        d.mkdir(parents=True)
        for f in ("fpga_profile.sh", "fpga_profiles.env"):
            shutil.copy(str(ROOT / "tools/car_run" / f), str(d / f))

    def ssh_lines(self):
        return [c for c in self.log.read_text().splitlines() if c.startswith("ssh|")]

    def test_m1_profile_word_and_absent_reason(self):
        kv = '{"ar": "off", "ar_reason": "ar_core_absent: profile m1_166 has no AR core (car screen on the Nano)", ' \
             '"build_id": "0x08051f22"}'
        r, ups = self.run_up(kv, FPGA_PROFILE="m1_166")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertTrue(self.ssh_lines()[0].endswith("geomem-kv-start m1_166"), self.ssh_lines())
        self.assertEqual(len(ups), 2)
        self.assertTrue(all(c.startswith("car_up|nano=1|colour=1|") for c in ups), ups)

    def test_reused_m1_server_by_build(self):
        kv = '{"ar": "off", "ar_reason": "ar_start_failed rc=1: x", "build_id": "0x08051f22", "reused": true}'
        r, ups = self.run_up(kv, FPGA_PROFILE="m1_166")
        self.assertIn("AR_STATUS nano: the KV serves BUILD_ID 0x08051f22 (profile m1", r.stdout)
        self.assertTrue(all(c.startswith("car_up|nano=1|colour=1|") for c in ups), ups)

    def test_switched_off_drm_and_old_builds_stay_off(self):
        for kv in ('{"ar": "off", "ar_reason": "switched off (kv_ar.conf or /etc/geomem-dev3-kv/ar.conf)", '
                   '"build_id": "0x08051f22"}',
                   '{"ar": "off", "ar_reason": "helper_missing: x", "build_id": "0x03c087c9"}',
                   '{"ar": "drm", "ar_reason": null, "build_id": "0x03c087c9"}'):
            with self.subTest(kv=kv):
                r, ups = self.run_up(kv, FPGA_PROFILE="v3fc2")
                self.assertEqual(r.returncode, 0, r.stderr)
                self.assertTrue(all(c.startswith("car_up|nano=unset|") for c in ups), ups)
                self.assertNotIn("AR_NANO", r.stdout)
                self.assertTrue(self.ssh_lines()[0].endswith("geomem-kv-start v3fc2"), self.ssh_lines())


if __name__ == "__main__":
    unittest.main()
