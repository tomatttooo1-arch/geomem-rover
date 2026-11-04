"""Competition additions: unknown estop, preview freshness/limits, optional image publisher.
No hub listener or hardware access. HTTP handler tested with an in-memory response writer.
"""
import io
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ui/geomem/bridge"))
import geomem_bridge as gb
import gmb_proto as gp
import gmb_ws as ws


class CompetitionTests(unittest.TestCase):
    def test_preview_source_limits_and_expiry(self):
        now = [100.0]
        b = gb.Bridge(clock=lambda: now[0])
        b.client = types.SimpleNamespace(link_state="up")
        jpeg = b"\xff\xd8test\xff\xd9"
        meta = {"encoding": "jpeg", "w": 640, "h": 480, "age_ms": 100}
        b._on_frame("PREVIEW", (meta, jpeg))
        self.assertEqual(b.preview[1], jpeg)
        for invalid in ({"w": 641}, {"h": 0}, {"age_ms": float("nan")}, {"age_ms": -1}, {"encoding": "png"}):
            b._on_frame("PREVIEW", (dict(meta, **invalid), jpeg))
        b._on_frame("PREVIEW", (meta, jpeg + b"x" * (160 * 1024)))
        self.assertEqual(b.counts["preview_invalid"], 6)

        def response():
            h = object.__new__(gb.Handler)
            h.server = types.SimpleNamespace(bridge=b, server_address=("127.0.0.1", 8790))
            h.headers = {"Host": "127.0.0.1:8790"}
            h.path = "/preview.jpg"
            h.wfile = io.BytesIO()
            statuses = []
            h.send_response = statuses.append
            h.send_header = lambda *args: None
            h.end_headers = lambda: None
            h._serve(False)
            return statuses[-1], h.wfile.getvalue()
        self.assertEqual(response(), (200, jpeg))
        now[0] = 101.5
        self.assertEqual(response()[0], 503)
        now[0] = 100.1
        b.client.link_state = "down"
        self.assertEqual(response()[0], 503)

    def test_snapshot_never_invents_released_estop(self):
        import inspect
        # Use the real composer's required args explicitly so future interface changes fail visibly.
        kwargs = dict(state=None, memory=None, runtime_info=None, task=gp.idle_task(), link_state="down")
        sig = inspect.signature(gp.compose_snapshot)
        for name, param in sig.parameters.items():
            if name not in kwargs and param.default is inspect.Parameter.empty:
                kwargs[name] = None
        s = gp.compose_snapshot(**kwargs)
        self.assertIsNone(s["estop"]["latched"])
        kwargs["state"] = {"estop": {"latched": True, "epoch": 7}}
        self.assertEqual(gp.compose_snapshot(**kwargs)["estop"], {"latched": True, "epoch": 7})

    def test_only_console_is_added_to_static_allowlist(self):
        app = str(ROOT / "ui/geomem/app")
        self.assertIsNotNone(ws.safe_static_path("/competition.html", app))
        self.assertIsNone(ws.safe_static_path("/../tools/competition/session.example.json", app))
        self.assertIsNone(ws.safe_static_path("/src/../../tools/competition/session.example.json", app))


if __name__ == "__main__":
    unittest.main()
