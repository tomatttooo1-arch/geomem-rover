"""10-22 live mapping voxel deltas: feed (live_mapping.VoxelFeed) -> /delta body -> bridge (chunked per-page relay)
-> a reference store; every step must equal the feed's current voxel set key by key (known-answer test).

    python tests/test_mapping_delta.py                 # run the tests
    python tests/test_mapping_delta.py --write-fixture # regenerate ui/geomem/operator/tests/fixtures/mapping-delta.json
                                                       # (replayed by the page's VoxelStore in voxel-store.test.mjs)
"""
import base64
import json
import struct
import sys
import types
import unittest
from collections import OrderedDict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ui/geomem/bridge"))
sys.path.insert(0, str(ROOT / "ui/geomem/tools/competition"))
import geomem_bridge as gb  # noqa: E402
import live_mapping as lm  # noqa: E402

FIXTURE = ROOT / "ui/geomem/operator/tests/fixtures/mapping-delta.json"
STATUS = {"format": "geomem-live-mapping", "version": 1, "source": "rtabmap", "session": "run-a", "scope": "boot-a",
          "frame": "map", "graphNodes": 5, "graphAgeMs": 40.0, "cloudAgeMs": 60.0, "revision": 1,
          "mapToOdom": {"position": [0.0, 0.0, 0.0], "quaternion": [0.0, 0.0, 0.0, 1.0]}}


def room(n=6000, seed=0, shift=(0.0, 0.0, 0.0), jitter=0.0, drop=0.03, recolor=None, extent=(6.0, 4.0, 1.5)):
    """Deterministic points on a few surfaces; per-run jitter/dropout imitates the assembler's regenerated grid."""
    base = np.random.RandomState(1234)
    pts = base.rand(n, 3) * extent
    pts[: n // 3, 2] = 0.0137                                # floor (off the cell boundaries)
    pts[n // 3: n // 2, 0] = 0.0137                          # wall
    col = base.randint(0, 256, (n, 3)).astype("u1")
    if recolor is not None:                                 # (fraction, delta) on the first fraction of points
        k = int(n * recolor[0])
        col[:k] = np.clip(col[:k].astype(int) + recolor[1], 0, 255)
    r = np.random.RandomState(seed)
    pts = pts + r.normal(0, jitter, pts.shape) + shift
    keep = r.rand(n) >= drop
    return pts[keep].astype("<f4"), col[keep]


def feed_state(f):
    return dict(zip(f.keys.tolist(), f.rgb.tolist()))


def parse_body(body):
    (L,) = struct.unpack_from("<I", body, 0)
    return json.loads(body[4:4 + L].decode()), body[4 + L:]


class RefStore(object):
    """Python mirror of the page's VoxelStore (operator/src/voxel-store.js): same base checks, upsert/remove."""

    def __init__(self):
        self.epoch = self.seq = None
        self.vox = {}

    def apply(self, head, payload):
        import zlib
        if head["mode"] == "keep":
            return "keep"
        if head["mode"] == "delta" and (head["epoch"] != self.epoch or head["baseSeq"] != self.seq):
            raise LookupError("resync")
        raw = zlib.decompress(payload) if payload else b""
        if len(raw) != head["rawBytes"]:
            raise ValueError("size")
        ak, ac, rk = lm.decode_vdz(raw, head["adds"], head["removes"])
        if head["mode"] == "full":
            self.vox = {}
        for k in rk.tolist():
            self.vox.pop(k, None)
        for k, c in zip(ak.tolist(), ac.tolist()):
            self.vox[k] = c
        self.epoch, self.seq = head["epoch"], head["seq"]
        if len(self.vox) != head["count"] or lm.key_hash(sorted(self.vox)) != head["keyHash"]:
            raise ValueError("integrity")
        return head["mode"]


class FeedKnownAnswer(unittest.TestCase):
    def test_sequence_three_clients(self):
        """Jitter, recolour below/above tolerance, whole-map shift, out-of-range jump, ring overflow, a reconnect
        and an unchanged cloud. Client A polls every step, B every third step, C connects at step 5, D polls at
        steps 7 and 12 only (5 revisions apart, ring of 4: its base is gone -> full in the same epoch)."""
        f = lm.VoxelFeed(0.05, ring=4, span=4096, epoch0=7)    # span 4096 x 5 cm = 204.8 m
        steps = [room(), room(seed=1, jitter=.002), room(seed=2, jitter=.002, recolor=(.5, 10)),
                 room(seed=3, jitter=.002, recolor=(.5, 90)), room(seed=4, jitter=.002, shift=(.12, .07, 0)),
                 room(seed=5, jitter=.002, shift=(.12, .07, 0)), room(seed=6, shift=(250.0, 0, 0)),
                 ] + [room(seed=7 + i, jitter=.002, shift=(250.0, 0, 0)) for i in range(6)]
        steps.append(steps[-1])                               # identical cloud -> no new revision
        modes = []
        clients = {"A": RefStore(), "B": RefStore()}
        cache = OrderedDict()
        got = {"A": [], "B": [], "C": [], "D": []}
        d_epochs = []
        for i, (xyz, rgb) in enumerate(steps):
            modes.append(f.update(xyz, rgb))
            if i == 5:
                clients["C"] = RefStore()
            if i == 7:
                clients["D"] = RefStore()
            for name, c in clients.items():
                if (name == "B" and i % 3) or (name == "D" and i not in (7, 12)):
                    continue
                head, ak, ac, rk = f.delta_arrays(c.epoch, c.seq)
                h, payload = parse_body(lm.delta_body(head, ak, ac, rk, STATUS, cache))
                got[name].append(c.apply(h, payload))
                if name == "D":
                    d_epochs.append(h["epoch"])
                self.assertEqual(c.vox, feed_state(f), "client %s step %d" % (name, i))
                self.assertEqual((h["count"], h["keyHash"]), (len(f.keys), lm.key_hash(f.keys)))
        self.assertEqual([m["mode"] for m in modes],
                         ["epoch", "delta", "delta", "delta", "epoch", "delta", "epoch"] + ["delta"] * 6 + ["same"])
        self.assertEqual(modes[4]["reason"], "large_change")
        self.assertEqual(modes[6]["reason"], "out_of_range")
        # +10 per channel is display jitter (tol 24): only the usual representative-point churn of a plain step
        self.assertLess(modes[2]["recolors"], 2 * modes[1]["recolors"] + 20)
        self.assertGreater(modes[3]["recolors"], 1000)         # +90 is a real colour change and is re-sent
        self.assertEqual(got["A"], ["full", "delta", "delta", "delta", "full", "delta", "full"] + ["delta"] * 7)
        # B (every 3rd step): full, merged delta (3 revisions), full (new epoch), merged deltas
        self.assertEqual(got["B"], ["full", "delta", "full", "delta", "delta"])
        self.assertEqual(got["D"], ["full", "full"])
        self.assertEqual(d_epochs[0], d_epochs[1])
        self.assertEqual(got["C"][0], "full")
        self.assertEqual(f.seq, 13)

    def test_delta_since_edges(self):
        f = lm.VoxelFeed(0.05, ring=3, epoch0=1)
        f.update(*room())
        e, s = f.epoch, f.seq
        self.assertEqual(f.delta_arrays(e, s)[0]["mode"], "delta")          # current: empty delta
        self.assertEqual(len(f.delta_arrays(e, s)[1]), 0)
        for bad in [(e + 1, s), (e, s + 1), (e, s - 1), (None, None), (e, None)]:
            self.assertEqual(f.delta_arrays(*bad)[0]["mode"], "full", bad)
        with self.assertRaises(ValueError):
            lm.VoxelFeed(0.05, span=100).update(*room())                # cloud wider than the uint16 window

    def test_vdz_roundtrip_and_hash_order_free(self):
        rs = np.random.RandomState(3)
        keys = np.unique(rs.randint(0, 1 << 47, 5000).astype(np.int64))
        keys = keys[(keys >> 32) < 65536]
        cols = rs.randint(0, 1 << 24, len(keys)).astype(np.uint32)
        rm = np.unique(rs.randint(0, 1 << 47, 700).astype(np.int64))
        n, z = lm.encode_vdz(keys, cols, rm)
        import zlib
        ak, ac, rk = lm.decode_vdz(zlib.decompress(z), len(keys), len(rm))
        self.assertTrue((ak == keys).all() and (ac == cols).all() and (rk == rm).all())
        self.assertEqual(lm.key_hash(keys), lm.key_hash(keys[::-1]))

    def test_read_cell_size_from_repo_yaml(self):
        cell = lm.read_cell_size(str(ROOT / "ui/geomem/tools/competition/live_mapping.yaml"))
        self.assertTrue(0.005 <= cell <= 0.1)


# ---------------------------------------------------------------------------- bridge end to end

def ws_frames(raw):
    """Unmasked server frames -> [(opcode, payload)]."""
    out, i = [], 0
    while i < len(raw):
        op, n = raw[i] & 15, raw[i + 1] & 127
        j = i + 2
        if n == 126:
            n = struct.unpack_from(">H", raw, j)[0]; j += 2
        elif n == 127:
            n = struct.unpack_from(">Q", raw, j)[0]; j += 8
        out.append((op, raw[j:j + n]))
        i = j + n
    return out


class Page(object):
    """Drains one WsSession like the page: reassembles chunks, acks every binary message, applies mapping."""

    def __init__(self, session):
        self.s, self.store, self.parts, self.log = session, RefStore(), {}, []
        self.max_binary = 0

    def drain(self, limit=10000):
        s = self.s
        for _ in range(limit):
            with s.cv:
                ready = s.fifo or s.hello or s.snapshot or any(s._waiting(k) for k in gb.BULK_KINDS)
            if not ready:
                return
            for kind, raw in s._next_batch():
                for op, payload in ws_frames(raw):
                    if op == 2:
                        self.max_binary = max(self.max_binary, len(payload))
                        self.binary(payload)
                        s.on_frame_ack()
        raise AssertionError("session never drained")

    def binary(self, frame):
        head, body = gb_split(frame)
        if head["type"] == "chunk":
            st = self.parts.get(head["kind"])
            if head["i"] == 0:
                st = self.parts[head["kind"]] = {"id": head["id"], "got": 0, "buf": b""}
            elif st is None or st["id"] != head["id"] or st["got"] != head["i"]:
                self.parts.pop(head["kind"], None)
                return
            st["buf"] += body
            st["got"] += 1
            if st["got"] < head["n"]:
                return
            del self.parts[head["kind"]]
            self.log.append(("chunked", head["n"]))
            assert len(st["buf"]) == head["bytes"]
            return self.binary(st["buf"])
        if head["type"] == "mapping":
            try:
                self.log.append(self.store.apply(head, body))
            except LookupError:
                self.log.append("resync")
                self.s.on_mapping_resync()


def gb_split(frame):
    (L,) = struct.unpack_from("<I", frame, 0)
    return json.loads(frame[4:4 + L].decode()), frame[4 + L + ((-(4 + L)) % 4):]


class BridgeRelay(unittest.TestCase):
    def setUp(self):
        self.now = [100.0]
        self.f = lm.VoxelFeed(0.03, ring=8, epoch0=50)
        self.cache = OrderedDict()
        self.b = gb.Bridge(clock=lambda: self.now[0], mapping_feed=("127.0.0.1", 47802), gateway=True)
        self.b.client = types.SimpleNamespace(link_state="up")
        self.requests = []

        def feed_get(path, timeout=8.0, limit=0):
            self.requests.append(path)
            if path == "/status":
                return 200, "application/json", json.dumps(dict(STATUS, epoch=self.f.epoch, seq=self.f.seq,
                                                                 voxels=len(self.f.keys))).encode()
            q = dict(kv.split("=") for kv in path.partition("?")[2].split("&") if kv)
            head, ak, ac, rk = self.f.delta_arrays(int(q["epoch"]) if q else None, int(q["seq"]) if q else None)
            return 200, "application/octet-stream", lm.delta_body(head, ak, ac, rk, STATUS, self.cache)
        self.b.feed_get = feed_get

    def page(self):
        s = gb.WsSession(None, None, self.b, chunked=True)
        self.b.sessions.add(s)
        return Page(s)

    def step(self, cloud, pages, dt=1.1):
        if cloud is not None:
            self.f.update(*cloud)
        self.now[0] += dt
        self.b._mapping_poll(-1e9, -1e9)
        for p in pages:
            p.drain()

    def test_pages_track_feed_key_by_key(self):
        a = self.page()
        self.step(room(n=40000, extent=(9, 7, 2)), [a])
        self.assertEqual(a.store.vox, feed_state(self.f))
        self.assertIn(("chunked", 3), a.log)                  # the ~150 KB full went as three 64 KB messages
        self.assertLessEqual(a.max_binary, gb.CHUNK_BYTES + 200)
        b = None
        for i in range(1, 9):
            if i == 3:
                b = self.page()                                # a page that connects later gets a full
            cloud = room(n=40000, extent=(9, 7, 2), seed=i, jitter=.0012,
                         shift=(.15, .1, 0) if i >= 6 else (0, 0, 0))   # step 6: whole-map shift -> new epoch
            pages = [a] if b is None else [a, b]
            self.step(cloud, pages)
            for p in pages:
                self.assertEqual(p.store.vox, feed_state(self.f), "step %d" % i)
        self.assertEqual([x for x in a.log if isinstance(x, str)].count("full"), 2)
        self.assertEqual(b.log[0][0] if isinstance(b.log[0], tuple) else b.log[0], "chunked")
        self.assertEqual([x for x in b.log if isinstance(x, str)][0], "full")
        self.now[0] += gb.MAPPING_KEEPALIVE_S                  # unchanged: header-only keep-alive, no /delta
        n = len(self.requests)
        self.step(None, [a, b], dt=0)
        self.assertEqual(self.requests[n:], ["/status"])
        self.assertEqual(a.log[-1], "keep")

    def test_resync_and_new_epoch_aborts_unsent_chunks(self):
        a = self.page()
        self.step(room(n=40000, extent=(9, 7, 2)), [a])
        a.s.on_mapping_resync()                                # page lost its store -> next push is a full
        self.step(room(n=40000, extent=(9, 7, 2), seed=1, jitter=.0012), [a])
        self.assertEqual(a.log[-1], "full")
        self.assertEqual(a.store.vox, feed_state(self.f))
        # page is slow: a delta is queued but not drained; meanwhile the map jumps (new epoch)
        self.step(room(n=40000, extent=(9, 7, 2), seed=2, jitter=.0012), [])
        self.assertFalse(a.s.mapping_idle())
        self.step(room(n=40000, extent=(9, 7, 2), seed=3, shift=(.3, 0, 0)), [])
        self.assertEqual(self.b.counts["mapping_aborted"], 1)
        a.drain()
        self.assertEqual(a.log[-1], "full")
        self.assertEqual(a.store.vox, feed_state(self.f))

    def test_busy_page_waits_then_gets_merged_delta(self):
        a = self.page()
        self.step(room(n=40000, extent=(9, 7, 2)), [a])
        self.step(room(n=40000, extent=(9, 7, 2), seed=1, jitter=.0012), [])          # queued, not sent
        self.step(room(n=40000, extent=(9, 7, 2), seed=2, jitter=.0012), [])          # page busy: skipped
        self.step(room(n=40000, extent=(9, 7, 2), seed=3, jitter=.0012), [a])         # drains the first delta
        self.step(None, [a])                                                          # merged 2 -> 3
        self.assertEqual(a.store.vox, feed_state(self.f))
        self.assertEqual(a.store.seq, self.f.seq)


# ---------------------------------------------------------------------------- fixture for the page test

def build_fixture():
    """One page's message sequence with expected voxel sets: full, delta, merged delta, keep, a delta on a wrong
    base (resync, store unchanged), a delta with a bad keyHash (integrity error, store reset), full, delta,
    new-epoch full, delta."""
    f = lm.VoxelFeed(0.05, ring=8, span=4096, epoch0=900)
    small = dict(n=500, extent=(2.0, 1.5, 0.6))
    out, cache = [], OrderedDict()

    def emit(head, ak, ac, rk, expect, error=None, tamper=None):
        h, payload = parse_body(lm.delta_body(head, ak, ac, rk, STATUS, cache))
        h.update({"type": "mapping", "version": 2})
        if tamper:
            h.update(tamper)
        out.append({"header": h, "payload": base64.b64encode(payload).decode(), "error": error,
                    "expect": None if expect is None else sorted([k, (c >> 16) & 255, (c >> 8) & 255, c & 255]
                                                                 for k, c in expect.items())})
    f.update(*room(seed=0, **small))
    emit(*f.delta_arrays(), expect=feed_state(f))
    e0, s0 = f.epoch, f.seq
    f.update(*room(seed=1, jitter=.002, **small))
    emit(*f.delta_arrays(e0, s0), expect=feed_state(f))
    e1, s1 = f.epoch, f.seq
    f.update(*room(seed=2, jitter=.002, recolor=(.3, 100), **small))
    f.update(*room(seed=3, jitter=.002, **small))
    emit(*f.delta_arrays(e1, s1), expect=feed_state(f))                       # merged over two revisions
    keep = dict(STATUS, type="mapping", version=2, mode="keep", epoch=f.epoch, seq=f.seq, count=len(f.keys))
    out.append({"header": keep, "payload": "", "error": None, "expect": [[k, (c >> 16) & 255, (c >> 8) & 255, c & 255]
                                                                         for k, c in sorted(feed_state(f).items())]})
    good = feed_state(f)
    e2, s2 = f.epoch, f.seq
    f.update(*room(seed=4, jitter=.002, **small))
    emit(*f.delta_arrays(e1, s1), expect=good, error="resync")                # base s1, page holds s2
    head, ak, ac, rk = f.delta_arrays(e2, s2)
    emit(head, ak, ac, rk, expect={}, error="integrity", tamper={"keyHash": (head["keyHash"] + 1) % (1 << 32)})
    emit(*f.delta_arrays(), expect=feed_state(f))                              # the full after resync
    e3, s3 = f.epoch, f.seq
    f.update(*room(seed=5, jitter=.002, **small))
    emit(*f.delta_arrays(e3, s3), expect=feed_state(f))
    e4, s4 = f.epoch, f.seq
    f.update(*room(seed=6, shift=(.4, 0, 0), **small))                         # new epoch
    assert f.last["mode"] == "epoch"
    emit(*f.delta_arrays(e4, s4), expect=feed_state(f))
    e5, s5 = f.epoch, f.seq
    f.update(*room(seed=7, jitter=.002, shift=(.4, 0, 0), **small))
    emit(*f.delta_arrays(e5, s5), expect=feed_state(f))
    return {"format": "geomem-mapping-delta-fixture", "hash123": lm.key_hash([1 | 2 << 16 | 3 << 32]), "steps": out}


class FixtureConsistent(unittest.TestCase):
    def test_committed_fixture_replays_in_python_reference(self):
        fx = json.loads(FIXTURE.read_text())
        st = RefStore()
        modes = []
        for step in fx["steps"]:
            payload = base64.b64decode(step["payload"])
            try:
                modes.append(st.apply(step["header"], payload))
            except LookupError:
                modes.append("resync")
            except ValueError:
                modes.append("integrity")
                st = RefStore()
            if step["expect"] is not None and step["header"]["mode"] != "keep":
                self.assertEqual(sorted([k, (c >> 16) & 255, (c >> 8) & 255, c & 255] for k, c in st.vox.items()),
                                 step["expect"])
        self.assertEqual(modes, ["full", "delta", "delta", "keep", "resync", "integrity", "full", "delta", "full",
                                 "delta"])


if __name__ == "__main__":
    if "--write-fixture" in sys.argv:
        FIXTURE.parent.mkdir(parents=True, exist_ok=True)
        FIXTURE.write_text(json.dumps(build_fixture(), separators=(",", ":")) + "\n")
        print("wrote", FIXTURE, FIXTURE.stat().st_size, "bytes")
    else:
        unittest.main()
