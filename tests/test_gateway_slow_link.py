"""10-22 acceptance on the host: with a 1.5 MB mapping payload going out over a 2 MB/s, 30 ms RTT link (socketpair +
rate-limited acking reader, tests/bench_gateway_slow_link.py), the chunked gateway sender keeps every snapshot gap
<= 0.5 s and the preview flowing. ~10 s wall time (real threads and sockets)."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bench_gateway_slow_link as bench  # noqa: E402


class SlowLinkGateway(unittest.TestCase):
    def test_snapshots_not_held_by_mapping_full(self):
        r = bench.run_arm(bench.gb, seconds=10, rate=2e6, rtt=0.03, mapping_bytes=1500000, mapping_period=2.4,
                          preview_bytes=50000, preview_hz=10)
        msg = str(r)
        self.assertGreaterEqual(r["snapshots"], 45, msg)
        self.assertLessEqual(r["gap_max"], 0.5, msg)
        self.assertGreaterEqual(r["preview_fps"], 5, msg)          # preview not starved by the map
        self.assertGreaterEqual(r["mapping_done"], 1, msg)         # nor the map by the preview


if __name__ == "__main__":
    unittest.main()
