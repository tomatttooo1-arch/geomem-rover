"""AR2NANO (nationals 2026-11-02): the car screen moves to the Nano. KV side: kv_start.sh skips a drm
request quietly when the loaded bitstream has no AR core (SLOT_MAP bit 8 clear), kv_service / nano_up say
AR_ON_NANO instead of AR_DISABLED. Fake binaries only (tests/test_comp_ar_20261026.py harness).

Run: PYTHONHASHSEED=0 PYTHONPATH=. python3 -m unittest discover -s tests -p test_ar2nano_20261102.py"""
import io
import json
import os
import sys
import unittest
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_comp_ar_20261026 as base  # noqa: E402


class KvStartNoArCore(base.KvStartTest):
    def test_slot_map_without_ar_skips_quietly(self):
        (self.drm / "edid").write_bytes(b"\x00\xff\xff")
        r, calls = self.start("drm", GEOMEM_KV_SLOT_MAP="0x263")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(calls, [["start", str(self.repo), str(self.out)] + self.BASE])   # the off line, once
        self.assertIn("AR_ON_NANO slot_map=0x263", r.stdout)
        self.assertNotIn("AR_DISABLED", r.stdout + r.stderr)
        self.assertFalse((self.tmp / "board.log.xm").exists() and
                         "desktop" in (self.tmp / "board.log.xm").read_text())        # no desktop toggle
        st = json.loads((self.out / "ar_status.json").read_text())
        self.assertEqual((st["ar"], st["requested"]), ("off", "drm"))
        self.assertTrue(st["reason"].startswith("ar_core_absent: SLOT_MAP 0x263"))

    def test_slot_map_with_ar_still_starts_ar(self):            # control: v3fc2 0x363 -> exactly as before
        (self.drm / "edid").write_bytes(b"\x00\xff\xff")
        r, calls = self.start("drm", GEOMEM_KV_SLOT_MAP="0x363")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(len(calls), 1)
        self.assertIn("--ar", calls[0])
        self.assertIn("AR_ENABLED mode=1024x600", r.stdout)

    def test_unreadable_slot_map_changes_nothing(self):
        for v in ("garbage", "0x262"):                          # not hex / bit 0 (sysinfo) clear: implausible
            self.log.unlink() if self.log.exists() else None
            out = self.out
            self.out = self.tmp / ("runs/" + v)
            r, calls = self.start("drm", GEOMEM_KV_SLOT_MAP=v)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertIn("--ar", calls[0], v)
            self.out = out

    def test_off_request_ignores_slot_map(self):
        r, calls = self.start("off", GEOMEM_KV_SLOT_MAP="0x263")
        self.assertEqual(calls, [["start", str(self.repo), str(self.out)] + self.BASE])
        self.assertNotIn("AR_ON_NANO", r.stdout)


class AnnounceNoArCore(unittest.TestCase):
    def test_ar_on_nano_line(self):
        buf = io.StringIO()
        with redirect_stdout(buf):
            base.kv.announce({"ar": "off", "ar_reason": "ar_core_absent: SLOT_MAP 0x263 (car screen on the Nano)"})
        self.assertTrue(buf.getvalue().startswith("AR_ON_NANO reason=ar_core_absent"))
        buf = io.StringIO()
        with redirect_stdout(buf):
            base.kv.announce({"ar": "off", "ar_reason": "helper_missing: x"})
        self.assertTrue(buf.getvalue().startswith("AR_DISABLED reason=helper_missing"))


if __name__ == "__main__":
    unittest.main()
