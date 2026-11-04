"""10-25 tools/car_run/deploy_nano.sh + board_preflight.sh, with fake ssh/docker/nvpmodel/... on PATH.

The fake ssh logs its argv and runs the remote command string locally with bash (fake bin dir still on PATH),
so a temporary directory stands in for the Nano file system. No network, no real board.
"""
import json
import os
import shutil
import stat
import subprocess
import tempfile
import textwrap
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEPLOY = ROOT / "tools/car_run/deploy_nano.sh"
PREFLIGHT = ROOT / "tools/car_run/board_preflight.sh"

FAKES = {
    "ssh": r"""
        echo "$*" >> "$FAKE_SSH_LOG"
        args=("$@"); i=0; host=""
        while [ $i -lt ${#args[@]} ]; do
          a=${args[$i]}
          case "$a" in -o|-F|-p|-i|-l|-J) i=$((i+2));; -*) i=$((i+1));; *) host=$a; i=$((i+1)); break;; esac
        done
        cmd="${args[*]:$i}"
        case " ${FAKE_SSH_DOWN:-} " in *" $host "*) cat >/dev/null; exit 255;; esac
        if [ -n "${FAKE_SSH_FAIL_MATCH:-}" ] && [[ "$cmd" == *"$FAKE_SSH_FAIL_MATCH"* ]]; then cat >/dev/null; exit 1; fi
        exec bash -c "$cmd"
    """,
    "docker": r"""
        if [ "$1 $2" = "image inspect" ]; then exit "${FAKE_DOCKER_IMAGE_RC:-0}"; fi
        if [ "$1" = run ]; then [ -n "${FAKE_SHIM_LINE:-}" ] && echo "$FAKE_SHIM_LINE"; exit 0; fi
        exit 1
    """,
    "nvpmodel": r"""
        echo "NV Power Mode: ${FAKE_NVP:-MAXN_SUPER}"; echo 2
    """,
    "jetson_clocks": r"""
        echo "cpu0: Online=1 Governor=schedutil MinFreq=729600 MaxFreq=1728000 CurrentFreq=1728000 IdleStates: C1=1"
        echo "cpu1: Online=1 Governor=schedutil MinFreq=729600 MaxFreq=1728000 CurrentFreq=1728000 IdleStates: C1=1"
        echo "GPU MinFreq=306000000 MaxFreq=1020000000 CurrentFreq=1020000000"
        echo "EMC MinFreq=204000000 MaxFreq=3199000000 CurrentFreq=3199000000 FreqOverride=0"
    """,
    "sudo": r"""
        [ "$1" = -n ] && shift
        exec "$@"
    """,
    "lsusb": r"""
        [ -n "${FAKE_LSUSB:-}" ] && echo "$FAKE_LSUSB"; exit 0
    """,
    "ping": r"""
        exit "${FAKE_PING_RC:-0}"
    """,
}

REPO_FILES = {
    "tools/car_run/car_up.sh": "echo car_up\n",
    "ui/geomem/tools/competition/console_remote.sh":
        'echo "console_remote $1" >> "$FAKE_ACT_LOG"\n'
        '[ "$1" = status ] && echo "STACK DOWN" && echo "GATEWAY DOWN"\n'
        '[ "$1" = up ] && exit "${FAKE_UP_RC:-0}"\nexit 0\n',
    "ui/geomem/tools/competition/nano_gateway.sh": 'echo "nano_gateway $1" >> "$FAKE_ACT_LOG"\n',
    "ui/geomem/tools/competition/check_current.py":
        "import os, sys\nsys.exit(int(os.environ.get('FAKE_CC_RC', '0')))\n",
    "README.md": "fake\n",
}


def git(repo, *args):
    return subprocess.run(["git", "-C", str(repo)] + list(args), check=True, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, universal_newlines=True).stdout.strip()


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="board_tools_"))
        self.bin = self.tmp / "bin"
        self.bin.mkdir()
        for name, body in FAKES.items():
            p = self.bin / name
            p.write_text("#!/usr/bin/env bash\n" + textwrap.dedent(body).lstrip())
            p.chmod(p.stat().st_mode | stat.S_IEXEC)
        self.repo = self.tmp / "repo"
        self.repo.mkdir()
        git(self.repo, "init", "-q")
        for rel, body in REPO_FILES.items():
            f = self.repo / rel
            f.parent.mkdir(parents=True, exist_ok=True)
            f.write_text(body)
        git(self.repo, "add", "-A")
        git(self.repo, "-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "-m", "one")
        self.sha1 = git(self.repo, "rev-parse", "HEAD")
        (self.repo / "README.md").write_text("two\n")
        git(self.repo, "-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "-am", "two")
        self.sha2 = git(self.repo, "rev-parse", "HEAD")
        self.nroot = self.tmp / "nano/fpga/dev1"
        self.ssh_log = self.tmp / "ssh.log"
        self.act_log = self.tmp / "act.log"
        self.env = dict(os.environ)
        self.env.update(PATH=str(self.bin) + os.pathsep + os.environ["PATH"], FAKE_SSH_LOG=str(self.ssh_log),
                        FAKE_ACT_LOG=str(self.act_log), FPGA_LOCK_SH="/x/lock.sh")

    def tearDown(self):
        shutil.rmtree(str(self.tmp), ignore_errors=True)

    def run_script(self, script, *args, **env):
        e = dict(self.env)
        e.update(env)
        return subprocess.run(["bash", str(script)] + list(args), env=e, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, universal_newlines=True, timeout=60)

    def deploy(self, *args, **env):
        return self.run_script(DEPLOY, "--repo", str(self.repo), "--root", str(self.nroot), *args, **env)

    def link(self):
        p = self.nroot / "console-current"
        return os.readlink(str(p)) if p.is_symlink() else None

    def acts(self):
        return self.act_log.read_text().splitlines() if self.act_log.exists() else []


class DeployTest(Base):
    def test_dry_run_prints_sequence_and_touches_nothing(self):
        r = self.deploy("--ref", self.sha1, "--dry-run", "--restart", "stack")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertFalse(self.ssh_log.exists(), "dry-run must not call ssh")
        self.assertFalse(self.nroot.exists())
        out = r.stdout
        short = self.sha1[:7]
        self.assertIn("lock.sh acquire orin-nano", out)
        self.assertIn("lock.sh acquire kv260", out)
        order = ["1. ", "archive --format=tar " + self.sha1, "/COMMIT; mv -T", "3. ", "ln -sfn console-direct-" + short,
                 "console_remote.sh down", "console_remote.sh up", "console_remote.sh status"]
        pos = [out.find(s) for s in order]
        self.assertTrue(all(p >= 0 for p in pos), list(zip(order, pos)))
        self.assertEqual(pos, sorted(pos), out)

    def test_fresh_deploy_writes_commit_and_switches_link(self):
        r = self.deploy("--ref", self.sha1)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        d = self.nroot / ("console-direct-" + self.sha1[:7])
        self.assertEqual((d / "COMMIT").read_text(), self.sha1 + "\n")
        self.assertTrue((d / "ui/geomem/tools/competition/console_remote.sh").is_file())
        self.assertEqual(self.link(), d.name)
        self.assertEqual(self.acts(), ["console_remote status"])  # --restart none: no down/up/gateway
        self.assertFalse(list(self.nroot.glob(".tmp-*")))
        self.assertFalse(list(self.nroot.glob("console-current.new*")))

    def test_same_sha_skips_transfer(self):
        self.assertEqual(self.deploy("--ref", self.sha1).returncode, 0)
        self.ssh_log.unlink()
        marker = self.nroot / ("console-direct-" + self.sha1[:7]) / "marker"
        marker.write_text("kept")
        r = self.deploy("--ref", self.sha1)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("跳过传输", r.stdout)
        self.assertNotIn("tar -x", self.ssh_log.read_text())
        self.assertTrue(marker.exists())

    def test_existing_dir_with_wrong_commit_refused(self):
        d = self.nroot / ("console-direct-" + self.sha1[:7])
        d.mkdir(parents=True)
        (d / "COMMIT").write_text("deadbeef\n")
        r = self.deploy("--ref", self.sha1)
        self.assertEqual(r.returncode, 5, r.stdout + r.stderr)
        self.assertIsNone(self.link())

    def test_transfer_failure_keeps_old_link(self):
        self.assertEqual(self.deploy("--ref", self.sha1).returncode, 0)
        old = self.link()
        r = self.deploy("--ref", self.sha2, FAKE_SSH_FAIL_MATCH="tar -x")
        self.assertEqual(r.returncode, 6, r.stdout + r.stderr)
        self.assertEqual(self.link(), old)
        self.assertFalse((self.nroot / ("console-direct-" + self.sha2[:7])).exists())

    def test_verify_failure_keeps_old_link(self):
        self.assertEqual(self.deploy("--ref", self.sha1).returncode, 0)
        old = self.link()
        r = self.deploy("--ref", self.sha2, FAKE_SSH_FAIL_MATCH="test -f")
        self.assertEqual(r.returncode, 7, r.stdout + r.stderr)
        self.assertEqual(self.link(), old)

    def test_unreachable_changes_nothing(self):
        r = self.deploy("--ref", self.sha1, "--host", "deadnano", FAKE_SSH_DOWN="deadnano")
        self.assertEqual(r.returncode, 4, r.stdout + r.stderr)
        self.assertFalse(self.nroot.exists())

    def test_switch_prints_rollback_to_old_target(self):
        self.assertEqual(self.deploy("--ref", self.sha1).returncode, 0)
        r = self.deploy("--ref", self.sha2)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(self.link(), "console-direct-" + self.sha2[:7])
        self.assertIn("回滚", r.stdout)
        self.assertIn("console-direct-" + self.sha1[:7], r.stdout)

    def test_restart_gateway(self):
        r = self.deploy("--ref", self.sha1, "--restart", "gateway")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(self.acts(), ["nano_gateway stop", "nano_gateway start", "console_remote status"])

    def test_restart_stack(self):
        r = self.deploy("--ref", self.sha1, "--restart", "stack")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(self.acts(), ["console_remote down", "console_remote up", "console_remote status"])

    def test_restart_stack_failure_exit_9(self):
        r = self.deploy("--ref", self.sha1, "--restart", "stack", FAKE_UP_RC="1")
        self.assertEqual(r.returncode, 9, r.stdout + r.stderr)

    def test_bad_args(self):
        self.assertEqual(self.deploy("--restart", "reboot").returncode, 2)
        self.assertEqual(self.deploy("--ref", "no-such-ref").returncode, 3)


class PreflightTest(Base):
    def setUp(self):
        super(PreflightTest, self).setUp()
        self.assertEqual(self.deploy("--ref", self.sha1).returncode, 0)
        self.data = self.tmp / "data"
        self.data.mkdir()
        self.kvconf = self.tmp / "geomem-kv.conf"
        self.kvconf.write_text("Host geomem-kv260\n")
        self.current = self.tmp / "car.current"
        self.kvsrc = self.tmp / "kvsrc"          # 10-26: the KV's installed competition snapshot
        helper = self.kvsrc / "tools/kv_link/ar_helper/build/aarch64/m7ar_present"
        helper.parent.mkdir(parents=True)
        helper.write_text("#!/bin/sh\n")
        helper.chmod(0o755)
        (self.kvsrc / "COMMIT").write_text(self.sha1 + "\n")
        self.good = dict(FPGA_KV_SOURCE=str(self.kvsrc), FAKE_SHIM_LINE="fpga-rotation-shim\t1.0", FAKE_LSUSB="Bus 002 Device 003: ID 2bc5:0670 Orbbec",
                         FPGA_KV_SSH_CONF=str(self.kvconf), FPGA_CAR_CURRENT=str(self.current),
                         FPGA_DISK_PATHS=str(self.data) + " " + str(self.tmp))

    def pf(self, *args, **env):
        e = dict(self.good)
        e.update(env)
        return self.run_script(PREFLIGHT, "--repo", str(self.repo), "--root", str(self.nroot), "--ref", self.sha1,
                               "--min-free-gb", "0", *args, **e)

    def lines(self, r, kind):
        return [l for l in r.stdout.splitlines() if l.startswith(kind + " ")]

    def test_all_pass(self):
        r = self.pf()
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertEqual(self.lines(r, "FAIL"), [])
        self.assertTrue(any("console-direct-" + self.sha1[:7] in l for l in self.lines(r, "PASS")))
        self.assertTrue(any("rotation-shim" in l for l in self.lines(r, "PASS")))
        self.assertTrue(any("MAXN_SUPER" in l for l in self.lines(r, "PASS")))
        self.assertTrue(any("GPU 1020000000" in l for l in self.lines(r, "INFO")))
        self.assertTrue(any("车栈未起" in l for l in self.lines(r, "INFO")))
        self.assertIn("FAIL 0", r.stdout)

    def test_each_fail_counts(self):
        r = self.pf("--ref", self.sha2, FAKE_SHIM_LINE="", FAKE_NVP="25W", FAKE_PING_RC="1", FAKE_LSUSB="")
        fails = self.lines(r, "FAIL")
        self.assertEqual(r.returncode, len(fails), r.stdout)
        self.assertEqual(len(fails), 6, r.stdout)   # 10-26: + the KV snapshot COMMIT (sha1) is not the expected sha2
        joined = "\n".join(fails)
        for s in ("期望 " + self.sha2, "FPGA_NAV_ROTATION_SHIM=false", "nvpmodel：25W", "ping 不通", "Orbbec", "deploy_kv.sh"):
            self.assertIn(s, joined)

    def test_missing_image_and_kv_ssh(self):
        r = self.pf(FAKE_DOCKER_IMAGE_RC="1", FPGA_KV_SSH_CONF=str(self.tmp / "nope.conf"))
        fails = self.lines(r, "FAIL")
        self.assertEqual(r.returncode, 2, r.stdout)
        self.assertTrue(any("镜像不在" in l for l in fails))
        self.assertTrue(any("ssh 不通" in l for l in fails))

    def test_disk_threshold_and_missing_link(self):
        os.unlink(str(self.nroot / "console-current"))
        r = self.pf("--min-free-gb", "999999999")
        fails = self.lines(r, "FAIL")
        self.assertEqual(r.returncode, 3, r.stdout)  # no link + two disk paths
        self.assertTrue(any("没有" in l and "console-current" in l for l in fails))

    def test_stack_state_is_info(self):
        run = self.tmp / "runs/20261025-live"
        run.mkdir(parents=True)
        os.symlink(str(run), str(self.current))
        r = self.pf(FAKE_CC_RC="0")
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertTrue(any("车栈在跑：20261025-live" in l for l in self.lines(r, "INFO")))
        r = self.pf(FAKE_CC_RC="4")
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertTrue(any("过期" in l for l in self.lines(r, "INFO")))

    def test_stack_health_is_info(self):
        """10-29: the running stack's health.json (host/hub/health.py) shows as one INFO line, never a FAIL."""
        run = self.tmp / "runs/20261029-live"
        run.mkdir(parents=True)
        os.symlink(str(run), str(self.current))
        (run / "health.json").write_text(json.dumps({"overall": "red", "summary": "有 1 项会让建图或寻物失败", "checks": [
            {"code": "ground_calib", "level": "red", "title": "地面标定", "detail": "拟合结果被拒",
             "hint": "检查相机支架"}, {"code": "disk", "level": "green", "title": "磁盘余量", "detail": "剩 80 GB"}]},
            ensure_ascii=False), encoding="utf-8")
        r = self.pf(FAKE_CC_RC="0")
        self.assertEqual(r.returncode, 0, r.stdout)
        info = [l for l in self.lines(r, "INFO") if "车栈体检" in l]
        self.assertEqual(len(info), 1, r.stdout)
        self.assertIn("red", info[0])
        self.assertIn("地面标定（拟合结果被拒；检查相机支架）", info[0])
        self.assertNotIn("磁盘余量", info[0])

    def test_commit_dir_mismatch_is_fail(self):
        (self.nroot / ("console-direct-" + self.sha1[:7]) / "COMMIT").write_text(self.sha2 + "\n")
        r = self.pf()
        self.assertEqual(r.returncode, 1, r.stdout)

    def test_unreachable(self):
        r = self.pf("--host", "deadnano", FAKE_SSH_DOWN="deadnano")
        self.assertEqual(r.returncode, 1, r.stdout)
        self.assertIn("Nano 不可达", r.stdout)
        self.assertEqual(len(self.lines(r, "PASS")), 0)

    def test_kv_snapshot_commit_and_helper(self):
        r = self.pf()
        self.assertTrue(any("KV 已装快照 COMMIT=" + self.sha1 in l for l in self.lines(r, "PASS")), r.stdout)
        (self.kvsrc / "COMMIT").write_text(self.sha2 + "\n")
        r = self.pf()
        self.assertEqual(r.returncode, 1, r.stdout)
        self.assertTrue(any("deploy_kv.sh" in l for l in self.lines(r, "FAIL")))

    def test_preflight_is_read_only(self):
        before = sorted(str(p) for p in self.tmp.rglob("*") if p != self.ssh_log and p != self.act_log)
        self.pf()
        after = sorted(str(p) for p in self.tmp.rglob("*") if p != self.ssh_log and p != self.act_log)
        self.assertEqual(before, after)


if __name__ == "__main__":
    unittest.main()
