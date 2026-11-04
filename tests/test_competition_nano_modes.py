"""Execute Nano launcher branches with isolated shell stubs; no network or hardware."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / 'ui/geomem/tools/competition/nano_up.sh'


def find_bash():
    """Allow a bundled Git sh (which is Bash) on Windows; ignore the WSL shim."""
    candidates = [os.environ.get('COMPETITION_TEST_BASH')]
    if os.name == 'nt':
        git = shutil.which('git')
        if git:
            candidates.extend(str(Path(git).parent.parent / 'usr/bin' / name)
                              for name in ('bash.exe', 'sh.exe'))
    else:
        candidates.append(shutil.which('bash'))
    return next((str(path) for path in candidates if path and Path(path).is_file()), None)


def shell_path(path):
    path = Path(path).resolve().as_posix()
    return '/' + path[0].lower() + path[2:] if os.name == 'nt' else path


# Only current-pointer tests are simulated. Map and memory use real temporary files.
# Every external launcher dependency is a shell function, including env, so this
# harness cannot accidentally contact SSH, start a car stack or open a feed.
HARNESS = r'''
dirname() { printf '%s\n' "${1%/*}"; }
test() {
  if [[ "${2:-}" = /data/fpga/car/state/car.current ]]; then
    case "$1:$FIXTURE_CURRENT" in
      -e:ready|-L:dangling) return 0;;
      *) return 1;;
    esac
  fi
  builtin test "$@"
}
python3() {
  printf 'python|%s\n' "$*" >> "$FIXTURE_LOG"
  case "$1" in
    */check_current.py) return "${FIXTURE_CHECK_RC:-0}";;
    */check_demo.py) return "${FIXTURE_DEMO_RC:-0}";;
    *) return 99;;
  esac
}
ssh() { printf 'ssh|%s\n' "$*" >> "$FIXTURE_LOG"; return 0; }
bash() {
  case "$1" in
    tools/car_run/car_up.sh|*/tools/car_run/car_up.sh)
      printf 'car_up|mapping=%s|map=%s|memory=%s|%s\n' "${CAR_MAPPING:-}" \
        "${CAR_MAP_YAML:-}" "${CAR_MEMORY_IN:-}" "$*" >> "$FIXTURE_LOG"
      return "${FIXTURE_START_RC:-0}";;
    */start_live_mapping.sh)
      printf 'feed|%s\n' "$*" >> "$FIXTURE_LOG"; return 0;;
    *) return 99;;
  esac
}
env() (
  while [[ "${1:-}" = *=* ]]; do export "$1"; shift; done
  "$@"
)
source "$0" "$@"
'''


@unittest.skipUnless(find_bash(), 'Bash is required for the isolated launcher branch tests')
class NanoModeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.log = self.root / 'calls.txt'
        self.map = self.root / 'map.yaml'
        self.memory = self.root / 'memory_final.json'
        self.map.write_text('image: map.pgm\n', encoding='utf-8')
        self.memory.write_text('{}', encoding='utf-8')

    def invoke(self, mode, current='missing', **environment):
        env = dict(os.environ, FIXTURE_CURRENT=current, FIXTURE_LOG=shell_path(self.log), **environment)
        result = subprocess.run([find_bash(), '-c', HARNESS, shell_path(LAUNCHER),
                                 shell_path(self.root), mode, '47801', '/unused/kv.conf',
                                 shell_path(self.map), shell_path(self.memory), 'a' * 64],
                                env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                                encoding='utf-8', errors='replace', timeout=10)
        calls = self.log.read_text(encoding='utf-8').splitlines() if self.log.exists() else []
        return result, calls

    def test_demo_refuses_ready_or_dangling_current_before_any_dependency(self):
        for current in ('ready', 'dangling'):
            with self.subTest(current=current):
                result, calls = self.invoke('demo', current)
                self.assertEqual(result.returncode, 7, result.stderr)
                self.assertIn('Stop and collect run 1 first', result.stderr)
                self.assertNotIn('ready;', result.stdout)
                self.assertEqual(calls, [])

    def test_mapping_and_attach_still_reuse_verified_current(self):
        for mode in ('mapping', 'attach'):
            with self.subTest(mode=mode):
                self.log.unlink(missing_ok=True)
                result, calls = self.invoke(mode, 'ready')
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(len(calls), 2)
                self.assertIn('check_current.py', calls[0])
                self.assertTrue(calls[1].startswith('feed|'))
                self.assertIn('Reusing the verified running car stack', result.stdout)

    def test_failed_reuse_validation_does_not_start_stack_or_feed(self):
        result, calls = self.invoke('mapping', 'ready', FIXTURE_CHECK_RC='4')
        self.assertEqual(result.returncode, 4, result.stderr)
        self.assertEqual(len(calls), 1)
        self.assertIn('check_current.py', calls[0])

    def test_attach_requires_a_recorded_session(self):
        result, calls = self.invoke('attach')
        self.assertEqual(result.returncode, 5, result.stderr)
        self.assertEqual(calls, [])

    def test_new_demo_passes_requested_map_and_memory_to_both_start_checks(self):
        result, calls = self.invoke('demo')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('check_demo.py', calls[0])
        self.assertIn('--map-yaml ' + shell_path(self.map), calls[0])
        self.assertIn('--memory ' + shell_path(self.memory), calls[0])
        self.assertIn('--db-sha256 ' + 'a' * 64, calls[0])
        self.assertTrue(calls[1].startswith('ssh|'))
        startups = [line for line in calls if line.startswith('car_up|')]
        self.assertEqual(len(startups), 2)
        self.assertTrue(startups[0].endswith('--pose odom --nav nav2 --dry-run'))
        self.assertTrue(startups[1].endswith('--pose odom --nav nav2'))
        for call in startups:
            self.assertIn('|mapping=|map=' + shell_path(self.map), call)
            self.assertIn('|memory=' + shell_path(self.memory), call)
        self.assertTrue(calls[-1].startswith('feed|'))

    def test_failed_demo_artifact_preflight_cannot_start_kv_stack_or_feed(self):
        result, calls = self.invoke('demo', FIXTURE_DEMO_RC='6')
        self.assertEqual(result.returncode, 6, result.stderr)
        self.assertEqual(len(calls), 1)
        self.assertIn('check_demo.py', calls[0])

    def test_new_mapping_starts_without_second_run_inputs(self):
        result, calls = self.invoke('mapping')
        self.assertEqual(result.returncode, 0, result.stderr)
        startups = [line for line in calls if line.startswith('car_up|')]
        self.assertEqual(len(startups), 2)
        self.assertTrue(all('|mapping=1|map=|memory=|' in call for call in startups))

    def test_dry_run_failure_prevents_real_start_and_feed(self):
        result, calls = self.invoke('demo', FIXTURE_START_RC='2')
        self.assertEqual(result.returncode, 2, result.stderr)
        startups = [line for line in calls if line.startswith('car_up|')]
        self.assertEqual(len(startups), 1)
        self.assertTrue(startups[0].endswith('--dry-run'))
        self.assertFalse(any(line.startswith('feed|') for line in calls))


if __name__ == '__main__':
    unittest.main()
