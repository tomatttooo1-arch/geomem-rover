"""10-26 competition chain car-screen AR: kv_start.sh AR / fallback / switch, kv_service AR fields and switch,
install_kv_service.sh --update, deploy_kv.sh --dry-run. Fake binaries on PATH and temporary roots only:
no FPGA, no network, no root."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import tempfile
import textwrap
import unittest

ROOT = Path(__file__).resolve().parents[1]
COMP = ROOT / "ui/geomem/tools/competition"
SPEC = importlib.util.spec_from_file_location("kv_service_ar", str(COMP / "kv_service.py"))
kv = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(kv)

# kv_board.sh stand-in: logs its argv as one JSON line; "start" fails when --ar is given and FAKE_AR_FAIL=1
FAKE_BOARD = r"""
import json, os, sys
log = os.environ["FAKE_BOARD_LOG"]
with open(log, "a") as fp:
    fp.write(json.dumps(sys.argv[1:]) + "\n")
out = sys.argv[3]
os.makedirs(out, exist_ok=True)
if "--ar" in sys.argv and os.environ.get("FAKE_AR_FAIL") == "1":
    with open(os.path.join(out, "server.err"), "w") as fp:
        fp.write('{"ar_desktop": 1}\n--ar drm: AR presenter did not start: drmModeSetCrtc failed (cmd x). Not starting.\n')
    print("server died:")
    sys.exit(1)
print(json.dumps({"port": 47800, "pid": 1}))
"""


def write_exe(path, body):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(body)
    path.chmod(path.stat().st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="comp_ar_"))
        self.addCleanup(shutil.rmtree, str(self.tmp), True)
        self.bin = self.tmp / "bin"
        self.env = dict(os.environ, PATH=str(self.bin) + os.pathsep + os.environ["PATH"])

    def fake(self, name, body):
        write_exe(self.bin / name, "#!/usr/bin/env bash\n" + textwrap.dedent(body).lstrip())


class KvStartTest(Base):
    def setUp(self):
        super(KvStartTest, self).setUp()
        self.repo = self.tmp / "repo"
        board = self.repo / "tools/kv_link/kv_board.sh"
        write_exe(board, 'exec python3 "$(dirname "$0")/fake_board.py" "$@"\n')
        (board.parent / "fake_board.py").write_text(FAKE_BOARD)
        comp = self.repo / "ui/geomem/tools/competition"
        comp.mkdir(parents=True)
        shutil.copy(str(COMP / "kv_start.sh"), str(comp / "kv_start.sh"))
        shutil.copy(str(COMP / "kv_ar_tune.json"), str(comp / "kv_ar_tune.json"))
        self.helper = self.repo / "tools/kv_link/ar_helper/build/aarch64/m7ar_present"
        write_exe(self.helper, "#!/bin/sh\n")
        self.fake("pgrep", "exit 1")
        self.fake("pkill", "exit 1")
        self.fake("xmutil", 'echo "xmutil $*" >> "$FAKE_BOARD_LOG.xm"')
        self.drm = self.tmp / "drm"
        self.drm.mkdir()
        self.log = self.tmp / "board.log"
        self.out = self.tmp / "runs/r1"
        self.env.update(FAKE_BOARD_LOG=str(self.log), GEOMEM_KV_DRM_SYSFS=str(self.drm),
                        GEOMEM_KV_BUILD_ID_CMD='printf "%s\\n" "$FPGA_BUILD_ID"')   # SMOKE1: load read back OK

    def start(self, mode, **env):
        e = dict(self.env, **env)
        r = subprocess.run(["bash", str(self.repo / "ui/geomem/tools/competition/kv_start.sh"), str(self.repo),
                            str(self.out), mode], env=e, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=60)
        calls = [json.loads(l) for l in self.log.read_text().splitlines()] if self.log.exists() else []
        return r, calls

    BASE = ["--core", "pl", "--device", "devmem", "--wait", "irq", "--roi-core", "pl", "--port", "47800"]

    def test_ar_starts_with_screen_colour_tune_and_build_gate(self):
        (self.drm / "edid").write_bytes(b"\x00\xff\xff")
        r, calls = self.start("drm")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(len(calls), 1)
        a = calls[0]
        self.assertEqual(a[:3], ["start", str(self.repo), str(self.out)])
        joined = " ".join(a)
        for s in ("--ar drm", "--ar-source live", "--ar-mode 1024x600", "--ar-colour main", "--ar-colour-port 47801", "--ar-colour-tol-ms 17", "--ar-max-age-ms 1000",
                  "--expect-build-id 0x00da5041", "--ar-tune-file " + str(self.repo / "ui/geomem/tools/competition/kv_ar_tune.json")):
            self.assertIn(s, joined)
        self.assertNotIn("--ar-dump", joined)
        self.assertIn("AR_ENABLED mode=1024x600", r.stdout)
        st = json.loads((self.out / "ar_status.json").read_text())
        self.assertEqual((st["ar"], st["requested"]), ("drm", "drm"))
        tune = json.loads((COMP / "kv_ar_tune.json").read_text())
        self.assertEqual((tune["colour_main"], tune["depth_stretch_pct"], tune["max_age_ms"]), (1, 250, 1000))

    def test_no_edid_uses_forced_panel_modeline(self):
        r, calls = self.start("drm")
        self.assertEqual(r.returncode, 0, r.stderr)
        a = calls[0]
        self.assertEqual(a[a.index("--ar-modeline") + 1], "49000 1024 1072 1168 1312 600 603 613 624 -hsync -vsync")
        self.assertNotIn("--ar-mode", a)

    def test_no_screen_sink_warns_and_still_starts_ar(self):   # 10-27: warning only, no new refusal
        aux = self.tmp / "aux"
        aux.write_bytes(bytes(0x300))   # DPCD 0x200 SINK_COUNT = 0
        r, calls = self.start("drm", GEOMEM_KV_DP_AUX=str(aux))
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(len(calls), 1)
        self.assertIn("--ar", calls[0])
        self.assertIn("AR_SCREEN_WARNING sink_count=0 edid_bytes=0", r.stdout)
        st = json.loads((self.out / "ar_status.json").read_text())
        self.assertEqual((st["ar"], st["screen"]), ("drm", "sink_count=0 edid_bytes=0"))

    def test_screen_present_no_warning(self):
        (self.drm / "edid").write_bytes(b"\x00\xff\xff")
        aux = self.tmp / "aux"
        aux.write_bytes(bytes(0x200) + b"\x41" + bytes(0xff))   # SINK_COUNT 1 (+CP_READY)
        r, calls = self.start("drm", GEOMEM_KV_DP_AUX=str(aux))
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn("AR_SCREEN_WARNING", r.stdout)
        self.assertEqual(json.loads((self.out / "ar_status.json").read_text())["screen"], "sink_count=1 edid_bytes=3")

    def test_ar_failure_falls_back_to_off_visibly(self):
        r, calls = self.start("drm", FAKE_AR_FAIL="1")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(len(calls), 2)
        self.assertIn("--ar", calls[0])
        off = str(self.out) + "-ar-off"
        self.assertEqual(calls[1], ["start", str(self.repo), off] + self.BASE)   # the pre-10-26 line exactly
        line = [l for l in r.stdout.splitlines() if l.startswith("AR_DISABLED reason=")]
        self.assertEqual(len(line), 1, r.stdout)
        self.assertIn("Not starting", line[0])
        st = json.loads(Path(off, "ar_status.json").read_text())
        self.assertEqual((st["ar"], st["requested"]), ("off", "drm"))
        self.assertIn("drmModeSetCrtc", st["reason"])

    def test_missing_helper_falls_back_without_trying(self):
        self.helper.unlink()
        r, calls = self.start("drm")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(len(calls), 1)
        self.assertNotIn("--ar", calls[0])
        self.assertIn("AR_DISABLED reason=helper_missing", r.stdout)

    def test_switch_off_is_the_old_line(self):
        r, calls = self.start("off")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(calls, [["start", str(self.repo), str(self.out)] + self.BASE])
        self.assertEqual(json.loads((self.out / "ar_status.json").read_text())["ar"], "off")

    def test_bad_mode_refused(self):
        r, calls = self.start("yes")
        self.assertEqual(r.returncode, 2)
        self.assertEqual(calls, [])


class KvServiceArTest(Base):
    def test_switch_sources_and_whitelist(self):
        repo = self.tmp / "src"
        conf = repo / "ui/geomem/tools/competition/kv_ar.conf"
        conf.parent.mkdir(parents=True)
        etc = self.tmp / "etc_ar.conf"
        self.assertEqual(kv.ar_switch(repo, etc)[:2], ("drm", "default"))
        conf.write_text("# comment\noff\n")
        self.assertEqual(kv.ar_switch(repo, etc)[:2], ("off", str(conf)))
        etc.write_text("drm\n")             # owned by the (non-root) test user: never trusted
        mode, source, warn = kv.ar_switch(repo, etc)
        self.assertEqual((mode, source), ("off", str(conf)))
        self.assertTrue(any("root-owned" in w for w in warn))
        conf.write_text("on please\n")
        mode, source, warn = kv.ar_switch(repo, etc)
        self.assertEqual((mode, source), ("drm", "default"))
        self.assertTrue(any("exactly one of drm/off" in w for w in warn))
        self.assertEqual(kv._ar_word(COMP / "kv_ar.conf", False), ("drm", None))   # shipped switch: on

    def test_running_server_reports_ar(self):
        proc = self.tmp / "proc"
        for pid, extra, status in ((11, ["--ar", "drm"], None),
                                   (12, [], {"ar": "off", "requested": "drm", "reason": "ar_start_failed rc=1"})):
            (proc / str(pid)).mkdir(parents=True)
            run = self.tmp / ("run%d" % pid)
            run.mkdir()
            (proc / str(pid) / "cmdline").write_bytes("\0".join(
                ["python3", "/opt/s/tools/kv_link/kv_server.py", "--out", str(run)] + extra).encode() + b"\0")
            (run / "ready.json").write_text(json.dumps({"pid": pid, "port": 47800, "build_id": kv.BUILD}))
            if status:
                (run / "ar_status.json").write_text(json.dumps(status))
            got = kv.running_server(proc)
            self.assertEqual(got["pid"], pid)
            if extra:
                self.assertEqual(got["ar"], "drm")
            else:
                self.assertEqual((got["ar"], got["ar_reason"]), ("off", "ar_start_failed rc=1"))
            shutil.rmtree(str(proc / str(pid)))


class InstallUpdateTest(Base):
    def setUp(self):
        super(InstallUpdateTest, self).setUp()
        self.fake("id", 'echo 0')
        self.fake("chown", 'exit 0')
        self.fake("visudo", 'exit 0')
        self.fake("pgrep", 'exit "${FAKE_PGREP_RC:-1}"')
        self.fake("install", r'''
            while [ $# -gt 2 ]; do case "$1" in -o|-g|-m) shift 2;; *) shift;; esac; done
            cp "$1" "$2"
        ''')
        self.root = self.tmp / "root"
        self.env.update(GEOMEM_KV_TEST_ROOT=str(self.root))

    def snapshot(self, name, commit):
        d = self.tmp / name
        for rel in ("tools/kv_link/kv_server.py", "ui/geomem/tools/competition/kv_service.py"):
            (d / rel).parent.mkdir(parents=True, exist_ok=True)
            (d / rel).write_text("# " + commit + "\n")
        (d / "COMMIT").write_text(commit + "\n")
        return d

    def install(self, *args, **env):
        return subprocess.run(["bash", str(COMP / "install_kv_service.sh")] + list(args), env=dict(self.env, **env),
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=60)

    def src(self):
        return self.root / "opt/geomem-dev3-kv/source"

    def test_update_swaps_and_keeps_previous_for_rollback(self):
        r = self.install(str(self.snapshot("a", "a" * 40)))
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(self.install(str(self.snapshot("b", "b" * 40))).returncode, 3)   # no --update: refused
        r = self.install("--update", str(self.tmp / "b"))
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("INSTALLED COMMIT=" + "b" * 40, r.stdout)
        self.assertEqual((self.src() / "COMMIT").read_text().strip(), "b" * 40)
        self.assertFalse(self.src().is_symlink())
        prev = list((self.root / "opt/geomem-dev3-kv/releases").glob("*-prev-aaaaaaa"))
        self.assertEqual(len(prev), 1)
        self.assertEqual((prev[0] / "COMMIT").read_text().strip(), "a" * 40)
        self.assertEqual((self.root / "usr/local/sbin/geomem-kv-start").read_text().strip(), "# " + "b" * 40)
        self.assertTrue((self.root / "etc/sudoers.d/geomem-dev3-kv").read_text().endswith('geomem-kv-start ""\n'))
        r = self.install("--update", str(prev[0]))                                     # rollback
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual((self.src() / "COMMIT").read_text().strip(), "a" * 40)

    def test_update_refused_while_server_runs(self):
        self.assertEqual(self.install(str(self.snapshot("a", "a" * 40))).returncode, 0)
        r = self.install("--update", str(self.snapshot("b", "b" * 40)), FAKE_PGREP_RC="0")
        self.assertEqual(r.returncode, 4, r.stdout + r.stderr)
        self.assertIn("kv_server is running", r.stderr)
        self.assertEqual((self.src() / "COMMIT").read_text().strip(), "a" * 40)
        self.assertEqual(list((self.root / "opt/geomem-dev3-kv/releases").iterdir()), [])


class DeployKvDryRunTest(Base):
    def test_dry_run_prints_sequence_and_touches_nothing(self):
        self.fake("ssh", 'echo "$*" >> "$FAKE_SSH_LOG"; exit 1')
        self.fake("make", 'echo make >> "$FAKE_SSH_LOG"; exit 1')
        repo = self.tmp / "repo"
        repo.mkdir()
        g = ["git", "-C", str(repo)]
        subprocess.run(g + ["init", "-q"], check=True)
        (repo / "x").write_text("x\n")
        subprocess.run(g + ["add", "x"], check=True)
        subprocess.run(g + ["-c", "user.name=t", "-c", "user.email=t@t", "commit", "-qm", "x"], check=True)
        sha = subprocess.run(g + ["rev-parse", "HEAD"], stdout=subprocess.PIPE, universal_newlines=True).stdout.strip()
        log = self.tmp / "ssh.log"
        r = subprocess.run(["bash", str(ROOT / "tools/car_run/deploy_kv.sh"), "--repo", str(repo), "--ref", sha,
                            "--dry-run", "--sudo-stdin", "--stop-kv"],
                           env=dict(self.env, FAKE_SSH_LOG=str(log), FPGA_LOCK_SH="/x/lock.sh"), stdin=subprocess.DEVNULL,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertFalse(log.exists(), "dry-run must not call ssh or make")
        out = r.stdout
        order = ["lock.sh acquire kv260", "1. ", "archive --format=tar " + sha, "BUILD=build aarch64",
                 "geomem-" + sha[:7], "4. ", "sudo -S", "TERM", "5. ", "dry-run 结束"]
        pos = [out.find(s) for s in order]
        self.assertTrue(all(p >= 0 for p in pos), list(zip(order, pos)))
        self.assertEqual(pos, sorted(pos), out)
        self.assertIn("geomem-kv260", out)
        self.assertIn("install_kv_service.sh\\ --update", out.split("4. ", 1)[1].split("\n", 2)[2])   # in the sudo command


if __name__ == "__main__":
    unittest.main()
