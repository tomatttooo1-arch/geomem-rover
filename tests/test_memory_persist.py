"""10-18 object memory across runs (host/memory_store.py; docs/design/MEMORY_PERSIST_20261018.md)."""
import hashlib
import io
import json
import os
import contextlib
import tempfile
import unittest

import numpy as np

from host import memory_store as ms
from perception.fpga_replay import memory_gates
from perception.fpga_replay.core import Config, Frame, Memory, Observation, Track


def _sha(path):
    with open(path, "rb") as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def _load(path):
    with open(path) as stream:
        return json.load(stream)


def _put(path, data, mode="w"):
    with open(path, mode) as stream:
        stream.write(data)


def _frame(fid, stamp, x=0.0):
    T = np.eye(4)
    T[0, 3] = x
    K = np.array([[2.0, 0, 2], [0, 2.0, 2], [0, 0, 1]])
    return Frame(fid, stamp, np.zeros((4, 4, 3), np.uint8), np.ones((4, 4)), K, T)


def _track(key, label, pos, state="current_confirmed", last_seen=300.0, odo=(4.0, 1.0), negatives=0):
    p = np.array(pos, dtype=float)
    t = Track(key, label, "green", p, p - 0.05, p + 0.05, last_seen)
    t.state, t.last_seen_frame, t.odo = state, "rgb-%d" % int(last_seen * 1e9), odo
    t.history = [{"frame_id": "rgb-1", "timestamp": last_seen, "bbox": [1, 2, 3, 4], "score": 0.9,
                  "source": "detector", "position_m": list(pos), "valid_depth_fraction": 1.0}] * 30
    t.negative_poses = [(last_seen - 1, np.eye(4))] * negatives
    return t


def _memory():
    m = Memory()
    for t in (_track("object-0001", "bottle", [1.25, -0.45, 0.15]),
              _track("object-0002", "cup", [2.0, 1.0, 0.4], state="position_invalid", negatives=2),
              _track("object-0003", "book", [-1.0, 0.5, 0.0], state="temporarily_occluded"),
              _track("object-0004", "mouse", [0.5, 0.5, 0.7], state="historical_memory")):
        m.tracks[t.id] = t
    m.odometer = (10.0, 2.0)
    m.rejected = {"pose_untrusted": 3}
    return m


DB_IDENT = {"v": 1, "kind": "rtabmap_db", "db_sha256": "a" * 64, "db_path": "/data/fpga/rtabmap/x.db",
            "map_yaml": "/x.yaml", "map_yaml_sha256": "b" * 64, "map_image": "/x.pgm", "map_image_sha256": "c" * 64,
            "sim_world": None, "sim_world_sha256": None, "note": None}


def _doc(memory=None, ident=DB_IDENT, complete=True):
    return ms.dump(memory or _memory(), complete=complete, runtime_boot_id="0123456789abcdef", world_frame="map",
                   pose_source="reference", sensor_source="hardware", map_identity_doc=dict(ident),
                   pose_mode="odom", code_commit="deadbee", written_ros_s=1.7e9)


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = self.tmp.name

    def tearDown(self):
        self.tmp.cleanup()

    def write(self, doc, name="m.json"):
        path = os.path.join(self.dir, name)
        if isinstance(doc, str):
            _put(path, doc)
        else:
            ms.save_atomic(doc, path)
        return path


class RoundTrip(Base):
    def test_fields_and_atomic_file(self):
        path = self.write(_doc())
        self.assertEqual(sorted(os.listdir(self.dir)), ["m.json"])       # no tmp left behind
        doc = _load(path)
        self.assertEqual((doc["format"], doc["v"], doc["complete"]), (ms.FORMAT, ms.VERSION, True))
        for key in ("written_wall", "code_commit", "runtime_boot_id", "map_identity", "pose_mode", "world_frame"):
            self.assertIn(key, doc)
        row = doc["tracks"][0]
        self.assertEqual(row["position_m"], [1.25, -0.45, 0.15])
        self.assertEqual((row["history_len"], len(row["history"])), (30, ms.HISTORY_KEEP))
        self.assertEqual(doc["tracks"][1]["negative_views"], 2)
        self.assertEqual(doc["memory"]["odometer"], [10.0, 2.0])

    def test_load_restores_positions_ids_and_state_semantics(self):
        path = self.write(_doc())
        m = Memory()
        info = ms.load_into(m, path, dict(DB_IDENT))
        self.assertEqual(list(m.tracks), ["object-0001", "object-0002", "object-0003", "object-0004"])
        b = m.tracks["object-0001"]
        np.testing.assert_allclose(b.position, [1.25, -0.45, 0.15])
        np.testing.assert_allclose(b.lower, [1.20, -0.50, 0.10])
        states = {k: t.state for k, t in m.tracks.items()}
        self.assertEqual(states, {"object-0001": "historical_memory", "object-0002": "position_invalid",
                                  "object-0003": "historical_memory", "object-0004": "historical_memory"})
        self.assertTrue(all(t.reason == ms.LOADED_REASON for t in m.tracks.values()))
        self.assertTrue(all(t.last_seen == 0.0 and not t.negative_poses for t in m.tracks.values()))
        self.assertEqual(b.prior["last_seen"], 300.0)
        self.assertEqual(m.tracks["object-0002"].prior["negative_views"], 2)
        self.assertEqual(info["tracks"], 4)
        self.assertEqual(info["cfg_differs"], [])
        # a loaded memory saved again carries prior, and loads again
        again = self.write(_doc(m), "again.json")
        self.assertEqual(_load(again)["tracks"][0]["prior"]["last_seen"], 300.0)
        ms.load_into(Memory(), again, dict(DB_IDENT))

    def test_query_after_load_is_not_confirmed_on_a_restarted_clock(self):
        m = Memory()
        ms.load_into(m, self.write(_doc()), dict(DB_IDENT))
        rows = m.query_memory("找瓶子", after=50.0)       # simulator clock restarted: now 50 < old 300
        self.assertEqual([r["id"] for r in rows], ["object-0001"])
        self.assertFalse(rows[0]["confirmed_after_query"])
        self.assertEqual(rows[0]["state"], "historical_memory")

    def test_association_radius_continues_from_the_earlier_drift(self):
        old = _memory()
        before = old._radius(old.tracks["object-0001"], None)
        self.assertAlmostEqual(before, 0.25 + 0.03 * 6 + 0.05 * 1)
        m = Memory()
        ms.load_into(m, self.write(_doc(old)), dict(DB_IDENT))
        self.assertEqual(m.odometer, (0.0, 0.0))
        self.assertAlmostEqual(m._radius(m.tracks["object-0001"], None), before)
        m.odometer = (1.0, 0.0)                     # one more metre in the new run
        self.assertAlmostEqual(m._radius(m.tracks["object-0001"], None), before + 0.03)

    def test_new_track_after_load_gets_the_next_id(self):
        m = Memory(cfg=Config(memory_quality=False))
        ms.load_into(m, self.write(_doc()), dict(DB_IDENT))
        f = _frame("rgb-1", 1.0)
        o = Observation("keyboard", 0.9, np.array([3.0, 3.0, 0.2]), np.array([2.9, 2.9, 0.1]),
                        np.array([3.1, 3.1, 0.3]), "black", 1.0, "rgb-1", 1.0, (0, 0, 2, 2), "detector")
        m.update_memory([o], {}, f)
        self.assertEqual(list(m.tracks)[-1], "object-0005")
        self.assertEqual(m.tracks["object-0001"].label, "bottle")

    def test_reobserved_after_load_associates_to_the_same_id(self):
        m = Memory(cfg=Config(memory_quality=False))
        ms.load_into(m, self.write(_doc()), dict(DB_IDENT))
        p = np.array([1.30, -0.45, 0.15])
        o = Observation("bottle", 0.9, p, p - 0.05, p + 0.05, "green", 1.0, "rgb-1", 1.0, (0, 0, 2, 2), "detector")
        m.update_memory([o], {}, _frame("rgb-1", 1.0))
        self.assertEqual(len(m.tracks), 4)
        b = m.tracks["object-0001"]
        self.assertEqual((b.state, b.reason, b.last_seen), ("current_confirmed", "new_visual_observation", 1.0))
        self.assertEqual(m.query_memory("bottle", after=0.5)[0]["confirmed_after_query"], True)

    def test_loaded_track_seen_empty_becomes_position_invalid_as_usual(self):
        m = Memory(cfg=Config(existence_v3=False))   # 10-27: legacy two-view rule (existence_v3=False); v3 counterpart in tests/test_memory_existence_20261027.py
        ms.load_into(m, self.write(_doc()), dict(DB_IDENT))
        ev = {"object-0001": "free"}
        m.update_memory([], ev, _frame("rgb-1", 1.0, 0.0))
        self.assertEqual(m.tracks["object-0001"].state, "historical_memory")
        m.update_memory([], ev, _frame("rgb-2", 2.0, 0.2))      # independent second view
        self.assertEqual(m.tracks["object-0001"].state, "position_invalid")
        self.assertEqual(m.tracks["object-0001"].reason, "free")


class Refusals(Base):
    def refused(self, doc, code, ident=DB_IDENT):
        path = self.write(doc)
        m = Memory()
        with self.assertRaises(ms.MemoryStoreError) as cm:
            ms.load_into(m, path, dict(ident))
        self.assertEqual(cm.exception.code, code, str(cm.exception))
        self.assertEqual(m.tracks, {})                  # never a half memory
        return str(cm.exception)

    def test_bad_json(self):
        self.refused('{"format": "fpga-object-memory", "v": 1, ', "bad_json")

    def test_wrong_version_and_format(self):
        d = _doc(); d["v"] = 2
        self.refused(d, "wrong_version")
        d = _doc(); d["format"] = "other"
        self.refused(d, "wrong_format")

    def test_missing_fields(self):
        d = _doc(); del d["map_identity"]
        self.refused(d, "missing_field")
        d = _doc(); del d["tracks"][1]["odo"]
        self.assertIn("odo", self.refused(d, "missing_field"))

    def test_bad_values(self):
        d = _doc(); d["tracks"][0]["state"] = "gone"
        self.refused(d, "bad_field")
        d = _doc(); d["tracks"][0]["position_m"] = [1, 2]
        self.refused(d, "bad_field")
        d = _doc(); d["world_frame"] = "odom"
        self.refused(d, "bad_field")

    def test_ids_may_have_gaps_but_not_repeat_and_new_ids_go_above(self):
        # 10-27 identity_v2: merged tracks leave gaps (object-0002 merged away); the next new id is above every
        # id held, so a gap can no longer make a new track overwrite a loaded one. Duplicates/odd ids refused.
        d = _doc(); d["tracks"] = [d["tracks"][0], d["tracks"][2]]
        m = Memory(); ms.load_into(m, self.write(d), dict(DB_IDENT))
        self.assertEqual(sorted(m.tracks), ["object-0001", "object-0003"])
        self.assertEqual(m._new_id(), "object-0004")
        self.assertEqual(sorted(t.number for t in m.tracks.values() if t.label == m.tracks["object-0001"].label),
                         sorted({t.number for t in m.tracks.values() if t.label == m.tracks["object-0001"].label}))
        d = _doc(); d["tracks"] = [d["tracks"][0], d["tracks"][0]]
        self.refused(d, "duplicate_track_id")
        d = _doc(); d["tracks"][0] = dict(d["tracks"][0], id="bottle-1")
        self.refused(d, "bad_track_id")

    def test_snapshot_needs_promote(self):
        self.refused(_doc(complete=False), "incomplete")

    def test_map_mismatch_none_and_pending(self):
        other = dict(DB_IDENT, db_sha256="f" * 64)
        self.assertIn("database differs", self.refused(_doc(), "map_mismatch", other))
        none = ms.map_identity()
        self.refused(_doc(ident=none), "map_mismatch", none)
        pending = ms.map_identity(mapping_db_dir="/data/fpga/rtabmap")
        self.refused(_doc(ident=pending), "map_mismatch", DB_IDENT)
        sim = dict(DB_IDENT, kind="sim_map", db_sha256=None)
        self.assertIn("simulator", self.refused(_doc(), "map_mismatch", sim))


class Identity(Base):
    def export(self, db_bytes=b"db-1"):
        """An rtabmap_car.sh layout: <dir>/<stem>.db, <dir>/run_<stem>/{export/<stem>.yaml,.pgm, sha256.txt}."""
        db = os.path.join(self.dir, "20261018-100000.db")
        _put(db, db_bytes, "wb")
        exp = os.path.join(self.dir, "run_20261018-100000", "export")
        os.makedirs(exp)
        yaml = os.path.join(exp, "20261018-100000.yaml")
        _put(os.path.join(exp, "20261018-100000.pgm"), b"P5 1 1 255 x", "wb")
        _put(yaml, "image: 20261018-100000.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n")
        with open(os.path.join(self.dir, "run_20261018-100000", "sha256.txt"), "w") as s:
            for p in (db, yaml, os.path.join(exp, "20261018-100000.pgm")):
                s.write("%s  %s\n" % (_sha(p), p))
        return db, yaml

    def test_export_resolves_to_its_database(self):
        db, yaml = self.export()
        a = ms.map_identity(map_yaml=yaml)
        self.assertEqual((a["kind"], a["db_sha256"]), ("rtabmap_db", _sha(db)))
        b = ms.map_identity(rtabmap_db=db)
        self.assertEqual(ms.identity_match(a, b)[0], True)
        self.assertEqual(ms.identity_match(b, a)[0], True)

    def test_changed_export_falls_back_to_file_hashes(self):
        db, yaml = self.export()
        a = ms.map_identity(map_yaml=yaml)
        _put(yaml, "# edited\n", "a")
        c = ms.map_identity(map_yaml=yaml)
        self.assertEqual(c["kind"], "map_export")
        self.assertIn("changed", c["note"])
        self.assertFalse(ms.identity_match(a, c)[0])

    def test_sim_identity(self):
        _, yaml = self.export()
        world = os.path.join(self.dir, "w.sdf")
        _put(world, "<sdf/>")
        a = ms.map_identity(sim_map_yaml=yaml, sim_world=world)
        self.assertEqual((a["kind"], a["sim_world"]), ("sim_map", "w.sdf"))
        self.assertTrue(ms.identity_match(a, ms.map_identity(sim_map_yaml=yaml))[0])

    def test_bind_pending_after_mapping(self):
        db, yaml = self.export()
        doc = _doc(ident=ms.map_identity(mapping_db_dir=self.dir))
        bound = ms.bind(doc, yaml)
        self.assertEqual(bound["map_identity"]["db_sha256"], _sha(db))
        self.assertEqual(bound["map_identity"]["bound_from"]["pending"]["kind"], "mapping_pending")
        path = self.write(bound, "bound.json")
        ms.load_into(Memory(), path, ms.map_identity(map_yaml=yaml))
        with self.assertRaises(ms.MemoryStoreError) as cm:
            ms.bind(_doc(ident=ms.map_identity(mapping_db_dir="/elsewhere")), yaml)
        self.assertEqual(cm.exception.code, "bind_wrong_dir")
        with self.assertRaises(ms.MemoryStoreError) as cm:
            ms.bind(_doc(), yaml)
        self.assertEqual(cm.exception.code, "not_pending")

    def test_cli_check_identity_and_exit_codes(self):
        db, yaml = self.export()
        ident = ms.map_identity(map_yaml=yaml)
        path = self.write(_doc(ident=ident))
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(ms.main(["check", "--memory", path, "--map-yaml", yaml]), 0)
        self.assertEqual(json.loads(out.getvalue())["labels"], ["book", "bottle", "cup", "mouse"])
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            self.assertEqual(ms.main(["check", "--memory", path]), 3)          # this run: no map
        self.assertEqual(json.loads(err.getvalue())["code"], "map_mismatch")
        idf = os.path.join(self.dir, "ident.json")
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(ms.main(["identity", "--rtabmap-db", db, "--out", idf]), 0)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(ms.main(["check", "--memory", path, "--identity", idf]), 0)
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(ms.main(["identity", "--rtabmap-db", db, "--out", idf]), 3)   # never overwritten

    def test_cli_promote_snapshot(self):
        snap = self.write(_doc(complete=False), "snap.json")
        out = os.path.join(self.dir, "promoted.json")
        self.assertEqual(ms.main(["promote", "--memory", snap, "--out", out]), 0)
        ms.load_into(Memory(), out, dict(DB_IDENT))


class RunnerSwitches(Base):
    """host/ros_runtime.main refuses a bad --memory-in before rclpy is imported (exit 2, nothing started)."""

    def main(self, *extra):
        import importlib.util
        from host import ros_runtime
        argv = ["--socket", "/nonexistent.sock", "--pose-source", "reference", "--sensor-source", "simulation"]
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            try:
                ros_runtime.main(argv + list(extra))
            except SystemExit as exc:
                return exc.code, err.getvalue()
            except ImportError:
                if importlib.util.find_spec("rclpy") is None:
                    return "passed_to_rclpy", err.getvalue()
                raise
        return None, err.getvalue()

    def ident(self, ident):
        path = os.path.join(self.dir, "ident.json")
        _put(path, json.dumps(ident))
        return path

    def test_refusals_exit_2_with_reason(self):
        good = self.write(_doc())
        code, err = self.main("--memory-in", good)                       # no identity given: none
        self.assertEqual(code, 2)
        self.assertIn("map_mismatch", err)
        code, err = self.main("--memory-in", good, "--memory-map-identity", self.ident(dict(DB_IDENT, db_sha256="e" * 64)))
        self.assertEqual(code, 2)
        self.assertIn("database differs", err)
        bad = self.write("{", "bad.json")
        code, err = self.main("--memory-in", bad, "--memory-map-identity", self.ident(DB_IDENT))
        self.assertEqual(code, 2)
        self.assertIn("bad_json", err)
        code, err = self.main("--memory-out-dir", os.path.join(self.dir, "nope"))
        self.assertEqual(code, 2)

    def test_matching_file_passes_the_check(self):
        import importlib.util
        if importlib.util.find_spec("rclpy") is not None:
            self.skipTest("rclpy installed: main would start a node")
        code, _ = self.main("--memory-in", self.write(_doc()), "--memory-map-identity", self.ident(DB_IDENT),
                            "--memory-out-dir", self.dir)
        self.assertEqual(code, "passed_to_rclpy")

    def test_feed_rows_mark_loaded_tracks(self):
        from host.memory_publish import track_rows
        m = Memory()
        ms.load_into(m, self.write(_doc()), dict(DB_IDENT))
        row = track_rows(m)[0]
        self.assertEqual(row["prior"]["last_seen"], 300.0)
        self.assertEqual(row["reason"], ms.LOADED_REASON)
        self.assertNotIn("prior", track_rows(_memory())[0])

    def test_hub_and_bridge_take_loaded_rows(self):
        """The rows with "prior" pass the hub (only non-finite scrubbing) and the GeoMem bridge (t.get per key):
        the loaded object reaches the UI with its id, state and lastSeen 0, nothing counted as bad."""
        import sys
        from host.memory_publish import MemoryFeed
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                        "ui", "geomem", "bridge"))
        import gmb_proto as gp
        from host.hub.hub_core import HubCore
        m = Memory()
        ms.load_into(m, self.write(_doc()), dict(DB_IDENT))
        msg = MemoryFeed("feedfeedfeedfeed", "map", "simulation").step(m, 0.0, 12.0)
        hub = HubCore.__new__(HubCore)
        import collections
        hub.counts, puts = collections.Counter(), []
        hub._put = lambda kind, row, ns: puts.append(row)
        hub.feed_memory(json.dumps(msg), 1)
        self.assertEqual(len(puts[0]["tracks"]), 4)
        self.assertEqual(dict(hub.counts), {})
        counts = {}
        objs = gp.compose_objects(puts[0], counts)
        self.assertEqual([o["id"] for o in objs], ["object-0001", "object-0002", "object-0003", "object-0004"])
        self.assertEqual(objs[0]["state"], "historical_memory")
        self.assertEqual(objs[0]["lastSeen"], 0.0)
        self.assertEqual(counts, {})


class Snapshot(unittest.TestCase):
    def test_only_on_change_and_rate_limited(self):
        p = ms.SnapshotPolicy(5.0)
        self.assertTrue(p.due("a", 0.0)); p.done("a", 0.0)
        self.assertFalse(p.due("a", 100.0))        # unchanged
        self.assertFalse(p.due("b", 3.0))          # changed, too soon
        self.assertTrue(p.due("b", 5.0))


if __name__ == "__main__":
    unittest.main()
