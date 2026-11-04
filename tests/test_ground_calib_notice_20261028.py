"""Ground-calibration fallback notice on the console (2026-10-28, owner approved).

ground_calib.json "alert" (set when the fit was refused and the last good / configured mount is used) -> hub STATE
"notices" -> bridge snapshot "notices" -> the console's red #car-notice bar. Absent while the fit was adopted.
Python 3.8.
"""
import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))

from host.hub.hub_core import HubCore, ground_calib_notices   # noqa: E402
from tools.car_run import ground_calib as gc   # noqa: E402


class TestNotice(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="gcn_")

    def write(self, obj):
        p = os.path.join(self.tmp, "ground_calib.json")
        with open(p, "w") as f:
            f.write(obj if isinstance(obj, str) else json.dumps(obj, ensure_ascii=False))
        return p

    def test_notices_from_ground_calib(self):
        p = self.write({"verdict": "adopt", "adopted": {"source": "last_good"}, "alert": gc.ALERT_LAST_GOOD})
        n = ground_calib_notices(p)
        self.assertEqual(n, [{"code": "ground_calib_fallback", "level": "warn", "text": gc.ALERT_LAST_GOOD,
                              "source": "last_good"}])
        self.assertEqual(gc.ALERT_LAST_GOOD, "相机角度测量异常，已用上次标定值，建议检查相机")
        self.assertEqual(ground_calib_notices(self.write({"verdict": "adopt", "adopted": {"source": "fit"}})), [])
        self.assertEqual(ground_calib_notices(self.write("{broken")), [])
        self.assertEqual(ground_calib_notices(os.path.join(self.tmp, "none.json")), [])

    def test_state_carries_notices_only_when_set(self):
        core = HubCore("live")
        self.assertNotIn("notices", core.compose_state(1))
        core.notices = ground_calib_notices(self.write({"adopted": {"source": "config_fallback"},
                                                        "alert": gc.ALERT_CONFIG}))
        s = core.compose_state(2)
        self.assertEqual(s["notices"][0]["text"], gc.ALERT_CONFIG)
        json.dumps(s)

    def test_bridge_snapshot(self):
        import test_geomem_bridge as tgb
        t = tgb.KioskSnapshotTests("test_basemap_in_snapshot")
        t.setUp()
        try:
            st = tgb.loc_state()
            self.assertNotIn("notices", t.tick(t.bridge(), st))
            st["notices"] = [{"code": "ground_calib_fallback", "level": "warn", "text": gc.ALERT_LAST_GOOD},
                             {"text": 3}, "x"]
            snap = t.tick(t.bridge(), st)
            self.assertEqual(snap["notices"], [{"code": "ground_calib_fallback", "level": "warn",
                                                "text": gc.ALERT_LAST_GOOD}])
        finally:
            t.tearDown()

    def test_page_and_launch_wiring(self):
        def read(*p):
            with open(os.path.join(ROOT, *p), encoding="utf-8") as f:
                return f.read()
        html = read("ui", "geomem", "operator", "index.html")
        self.assertIn('id="car-notice"', html)
        self.assertIn('id="car-notice-text"', html)
        self.assertIn("snapshot.notices", read("ui", "geomem", "operator", "src", "remote.js"))
        self.assertIn("showCarNotices(", read("ui", "geomem", "operator", "src", "app.js"))
        self.assertIn('ground_calib_notices(os.path.join(os.environ["O"], "ground_calib.json"))',
                      read("host", "hub", "hub_ros.py"))
        inner = read("tools", "car_run", "car_inner.sh")
        self.assertIn('GCX+=(--last-good "$O/ground_calib_last_good_in.json")', inner)


if __name__ == "__main__":
    unittest.main()
