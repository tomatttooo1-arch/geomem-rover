"""hub MEMORY with non-finite numbers (2026-10-15 GeoMem integration, stage 2 fix b).

Before: MEMORY went out with allow_nan=False and was not scrubbed, so one NaN/inf in a track made
the frame unsendable; in HubServer.tick that raised after STATE was composed and cost the tick's
STATE too (``tick_error:ValueError``). Now feed_memory drops tracks holding a non-finite number
(``memory_nonfinite_tracks``), nulls other non-finite fields (``memory_nonfinite_fields``), and the
sender backstop drops only an unencodable MEMORY (``memory_unsendable:*``) while STATE goes on.
Loopback only (port 0 on 127.0.0.1), as in test_hub.py.
"""

import json
import sys
import time
import unittest

from pc import fpgac
from host import fpga_clock
from host.hub.hub_core import HubCore, NS

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from test_hub import Hub, Client, K  # noqa: E402


def track(i, **kw):
    t = {"id": "object-%04d" % i, "label": "bottle", "state": "historical_memory",
         "position_m": [1.0, 2.0, 0.3], "last_seen": 10.0}
    t.update(kw)
    return t


BAD_TEXT = ('{"v":1,"runtime_boot_id":"rt","revision":%d,"world_frame":"map","tracks":['
            '{"id":"object-0001","label":"bottle","position_m":[NaN,2.0,0.3]},'
            '{"id":"object-0002","label":"bottle","position_m":[1.0,2.0,0.3]},'
            '{"id":"object-0003","label":"cup","position_m":[1.0,Infinity,0.3]},'
            '{"id":"object-0004","label":"cup","position_m":[1.0,2.0,0.3],"history_last":{"score":1e400}}]}')


class CoreTests(unittest.TestCase):
    def test_nonfinite_tracks_dropped_and_counted(self):
        core = HubCore("live")
        core.feed_memory(BAD_TEXT % 1, 0)
        body = core.memory_body(1, force=True)
        self.assertEqual([t["id"] for t in body["tracks"]], ["object-0002"])
        self.assertEqual(core.counts["memory_nonfinite_tracks"], 3)
        self.assertEqual(core.counts["memory_nonfinite_fields"], 0)
        fpgac.dumps(body)                                   # encodable now

    def test_instrument_raw_body_is_unencodable(self):
        # positive control for the fix: the same input taken verbatim cannot be encoded
        row = json.loads(BAD_TEXT % 1)
        with self.assertRaises(ValueError):
            fpgac.dumps(row)

    def test_other_nonfinite_fields_nulled(self):
        core = HubCore("live")
        core.feed_memory(json.dumps({"revision": 1, "stamp_ros_s": float("nan"), "tracks": [track(1)]}), 0)
        body = core.memory_body(1, force=True)
        self.assertIsNone(body["stamp_ros_s"])
        self.assertEqual(len(body["tracks"]), 1)
        self.assertEqual(core.counts["memory_nonfinite_fields"], 1)
        self.assertEqual(core.counts["memory_nonfinite_tracks"], 0)

    def test_finite_memory_untouched(self):
        core = HubCore("live")
        text = json.dumps({"revision": 1, "tracks": [track(1), track(2)]})
        core.feed_memory(text, 0)
        body = core.memory_body(1, force=True)
        self.assertEqual(body["tracks"], json.loads(text)["tracks"])
        self.assertNotIn("memory_nonfinite_tracks", core.counts)
        self.assertNotIn("memory_nonfinite_fields", core.counts)


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.hub = Hub()
        self.c = Client(self.hub.server.port)
        self.assertTrue(self.c.c.connected.wait(5))

    def tearDown(self):
        self.c.stop()
        self.hub.close()

    def memories(self):
        with self.c.lock:
            return [(t, o) for t, k, o in self.c.frames if k == K["MEMORY"]]

    def test_bad_memory_reaches_client_without_bad_tracks_and_state_continues(self):
        with self.hub.server.lock:
            self.hub.core.feed_memory(BAD_TEXT % 2, fpga_clock.now_ns())
        self.assertTrue(self.c.wait(lambda: any(o.get("revision") == 2 for _, o in self.memories())))
        t2, m2 = [x for x in self.memories() if x[1].get("revision") == 2][0]
        self.assertEqual([t["id"] for t in m2["tracks"]], ["object-0002"])
        # STATE keeps coming around and after the bad MEMORY
        self.assertTrue(self.c.wait(lambda: sum(1 for t, _ in self.c.states() if t > t2) >= 3))
        self.assertEqual(self.hub.core.counts["memory_nonfinite_tracks"], 3)
        self.assertFalse([k for k in self.hub.core.counts if k.startswith("tick_error")])

    def test_backstop_drops_only_memory_state_still_sent(self):
        # something feed_memory did not catch (injected past it): MEMORY dropped, STATE every tick
        before = len(self.c.states())
        with self.hub.server.lock:
            self.hub.core._put("memory", {"revision": 3, "tracks": [track(1, position_m=[float("nan"), 0, 0])]},
                               fpga_clock.now_ns())
        self.assertTrue(self.c.wait(lambda: self.hub.core.counts["memory_unsendable:ValueError"] >= 1))
        self.assertTrue(self.c.wait(lambda: len(self.c.states()) >= before + 5))
        self.assertFalse(any(o.get("revision") == 3 for _, o in self.memories()))
        self.assertFalse([k for k in self.hub.core.counts if k.startswith("tick_error")])
        # a client joining now still gets its snapshot STATE (the _snapshot_to backstop)
        late = Client(self.hub.server.port)
        try:
            self.assertTrue(late.c.connected.wait(5))
            self.assertTrue(late.wait(lambda: len(late.states()) >= 1))
            self.assertFalse(any(k == K["MEMORY"] and o.get("revision") == 3 for _, k, o in late.frames))
        finally:
            late.stop()
        self.assertFalse([k for k in self.hub.core.counts if k.startswith("reader_error")])


if __name__ == "__main__":
    unittest.main()
