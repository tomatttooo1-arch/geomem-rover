"""AR2NANO2 (nationals 2026-11-02): the car screen drawn on the Nano goes live.

* nano_up.sh: CAR_AR_NANO defaults to 1 exactly when the KV says ar_core_absent (its bitstream has no AR
  core), the colour sender then defaults on; any other KV state leaves the car_up line as before.
* host/nano_ar.PermitPoller: KV TELEM permit STATUS bit 0 -> (permit_present, permit_i); stale / no block /
  link error = (0, 0) = DRV N/A, never a stale DRIVE OK.
* host/kv_ar.ArPresenter permit_source: written into the message registers; None = untouched.
* host/ros_geo_producer.start_ar_nano: a failing screen never stops the producer.
Fake shells and fake clients only (no network, no DRM).

Run: PYTHONHASHSEED=0 PYTHONPATH=. python3 -m unittest discover -s tests -p test_ar2nano2_20261102.py"""
import io
import os
import subprocess
import sys
import tempfile
import types
import unittest
from contextlib import redirect_stderr
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
import test_competition_nano_modes as nm  # noqa: E402
from host import kv_ar  # noqa: E402
from host import nano_ar  # noqa: E402

HARNESS = nm.HARNESS.replace(
    '''ssh() { printf 'ssh|%s\\n' "$*" >> "$FIXTURE_LOG"; return 0; }''',
    '''ssh() { printf 'ssh|%s\\n' "$*" >> "$FIXTURE_LOG"; printf '%s\\n' "${FIXTURE_KV_JSON:-}"; return 0; }
systemctl() { return "${FIXTURE_GDM_RC:-3}"; }''').replace(
    '''printf 'car_up|mapping=%s|map=%s|memory=%s|%s\\n' "${CAR_MAPPING:-}" \\''',
    '''printf 'car_up|nano=%s|colour=%s|mapping=%s|map=%s|memory=%s|%s\\n' "${CAR_AR_NANO-unset}" "${CAR_AR_COLOUR:-}" "${CAR_MAPPING:-}" \\''')
assert HARNESS != nm.HARNESS and "FIXTURE_KV_JSON" in HARNESS and "nano=%s" in HARNESS

ABSENT = '{"ar": "off", "ar_reason": "ar_core_absent: SLOT_MAP 0x263 (car screen on the Nano)"}'
DRM = '{"ar": "drm", "ar_reason": ""}'
OFF = '{"ar": "off", "ar_reason": "helper_missing: x"}'


@unittest.skipUnless(nm.find_bash(), "bash")
class NanoUpArNano(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.log = self.root / "calls.txt"

    def run_up(self, kv_json, **env):
        self.log.unlink(missing_ok=True)
        e = dict(os.environ, FIXTURE_CURRENT="missing", FIXTURE_LOG=str(self.log), FIXTURE_KV_JSON=kv_json, **env)
        e.pop("CAR_AR_NANO", None) if "CAR_AR_NANO" not in env else None
        e.pop("CAR_AR_COLOUR", None) if "CAR_AR_COLOUR" not in env else None
        r = subprocess.run([nm.find_bash(), "-c", HARNESS, str(nm.LAUNCHER), str(self.root), "mapping", "47801",
                            "/unused/kv.conf"], env=e, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                           timeout=20)
        calls = self.log.read_text().splitlines() if self.log.exists() else []
        return r, [c for c in calls if c.startswith("car_up|")]

    def test_ar_core_absent_turns_the_nano_screen_on(self):
        r, ups = self.run_up(ABSENT)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(len(ups), 2)
        for c in ups:
            self.assertTrue(c.startswith("car_up|nano=1|colour=1|"), c)
        self.assertIn("AR_STATUS nano", r.stdout)
        self.assertIn("AR_NANO on", r.stdout)
        self.assertNotIn("AR_NANO_WARNING", r.stderr)

    def test_desktop_running_is_a_warning_only(self):
        r, ups = self.run_up(ABSENT, FIXTURE_GDM_RC="0")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("AR_NANO_WARNING", r.stderr)
        self.assertEqual(len(ups), 2)

    def test_other_kv_states_leave_the_line_as_before(self):
        for kv_json, colour in ((DRM, "1"), (OFF, "0"), ("", "0")):
            with self.subTest(kv=kv_json):
                r, ups = self.run_up(kv_json)
                self.assertEqual(r.returncode, 0, r.stderr)
                for c in ups:
                    self.assertTrue(c.startswith("car_up|nano=unset|colour=%s|" % colour), c)
                self.assertNotIn("AR_NANO", r.stdout)

    def test_caller_wins(self):
        r, ups = self.run_up(ABSENT, CAR_AR_NANO="0")
        self.assertTrue(all(c.startswith("car_up|nano=0|colour=0|") for c in ups), ups)   # 0 = off in car_up
        r, ups = self.run_up(DRM, CAR_AR_NANO="1", CAR_AR_COLOUR="0")
        self.assertTrue(all(c.startswith("car_up|nano=1|colour=0|") for c in ups), ups)


class FakeTelem(object):
    def __init__(self, script):
        self.script = list(script)
        self.closed = 0

    def poll(self, want):
        self.want = want
        item = self.script.pop(0)
        if isinstance(item, Exception):
            raise item
        return types.SimpleNamespace(blocks={"permit": item})

    def close(self):
        self.closed += 1


class PermitPollerTests(unittest.TestCase):
    def poller(self, script, t):
        made = []

        def factory(srv):
            c = FakeTelem(script)
            made.append(c)
            return c
        clock = types.SimpleNamespace(t=t)
        p = nano_ar.PermitPoller(("kv", 1), client_factory=factory, clock=lambda: clock.t)
        return p, clock, made

    def test_states(self):
        on = {"available": True, "status_bits": {"permit": True}}
        off = {"available": True, "status_bits": {"permit": False}}
        absent = {"available": False, "reason": "no permit block"}
        p, clock, made = self.poller([on, off, absent, RuntimeError("timeout"), on], 100.0)
        self.assertEqual(p(), (0, 0))                     # nothing polled yet: DRV N/A
        p.poll_once()
        self.assertEqual(p(), (1, 1))                     # DRIVE OK
        self.assertEqual(made[0].want, 1 << 2)            # the permit block only
        p.poll_once()
        self.assertEqual(p(), (1, 0))                     # STOP
        p.poll_once()
        self.assertEqual(p(), (0, 0))                     # bitstream without the block: DRV N/A
        p.poll_once()                                     # link error: last answer kept, client rebuilt
        self.assertEqual(p.counts["errors"], 1)
        self.assertEqual(made[0].closed, 1)
        p.poll_once()
        self.assertEqual(len(made), 2)
        self.assertEqual(p(), (1, 1))
        clock.t += nano_ar.PERMIT_FRESH_S + 0.01           # no answer for > 1 s: never a stale DRIVE OK
        self.assertEqual(p(), (0, 0))


class ApplyPermit(unittest.TestCase):
    def inp(self):
        regs = [0] * 24
        regs[kv_ar.R["permit_present"]] = 0
        return {"regs": regs}

    def test_source_written_and_none_untouched(self):
        a = kv_ar.ArPresenter.__new__(kv_ar.ArPresenter)
        a.permit_source = None
        x = self.inp()
        before = list(x["regs"])
        a._apply_permit(x)
        self.assertEqual(x["regs"], before)
        for src, want in (((1, 1), (1, 1)), ((1, 0), (1, 0)), ((0, 1), (0, 0))):
            a.permit_source = lambda s=src: s
            x = self.inp()
            a._apply_permit(x)
            self.assertEqual((x["regs"][kv_ar.R["permit_present"]], x["regs"][kv_ar.R["permit_i"]]), want)

        def boom():
            raise RuntimeError("x")
        a.permit_source = boom
        x = self.inp()
        a._apply_permit(x)
        self.assertEqual(x["regs"][kv_ar.R["permit_present"]], 0)

    def test_helper_cmd(self):
        c = nano_ar.helper_cmd("/r", mini_scale=2)
        self.assertEqual(c[:3], ["/r/tools/kv_link/ar_helper/build/nano/m7ar_present", "--provider", "cpu"])
        self.assertIn("--mini-scale", c)
        self.assertNotIn("--cpu-impl", c)                 # fast is the helper's default
        self.assertIn("ref", nano_ar.helper_cmd("/r", cpu_impl="ref"))


class ProducerNeverStops(unittest.TestCase):
    def test_failed_screen_returns_none(self):
        from host import ros_geo_producer as gp
        orig = nano_ar.NanoArScreen.start

        def fail(*a, **k):
            raise kv_ar.ArUnavailable("no DRM master")
        nano_ar.NanoArScreen.start = classmethod(lambda cls, *a, **k: fail())
        try:
            a = types.SimpleNamespace(ar_nano_tune_file=None, ar_nano_out=None, ar_nano_mode="1024x600",
                                      ar_nano_modeline=None, ar_nano_colour_port=47801)
            err = io.StringIO()
            with redirect_stderr(err):
                self.assertIsNone(gp.start_ar_nano(a, ("kv", 1)))
            self.assertIn("AR_NANO_FAILED ArUnavailable: no DRM master", err.getvalue())
        finally:
            nano_ar.NanoArScreen.start = orig


if __name__ == "__main__":
    unittest.main()
