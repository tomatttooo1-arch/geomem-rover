import contextlib
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from tools.rtabmap_bundle_map import BUNDLE_FILES, DATABASE_NAME, main


CHECKOUT = Path(__file__).resolve().parents[1]
FROZEN_ROOT = Path(__file__).resolve().parents[2] / "rtabmap-reload-coordinate-20260912"
FROZEN_BUNDLE = FROZEN_ROOT / "map-bundle-v2"
FROZEN_MAP_ID = "2a450fd79970fe4c145af5afab6b76a9420b2880c8bbb70581e9aafa1bfab8c7"

# Deliberately tiny stand-ins: the tool copies and hashes bytes, it never parses them.
CONTENTS = {
    "rtabmap.db": b"SQLite format 3\x00 synthetic rtabmap database\n",
    "optimized.pgm": b"P5\n4 4\n255\n" + bytes(range(16)),
    "optimized.yaml": b"image: optimized.pgm\nresolution: 0.05\n",
    "optimized_cloud.ply": b"ply\nformat ascii 1.0\nelement vertex 0\nend_header\n",
    "optimized_camera_poses.txt": b"1 1 0 0 0 1 0 0 0 0 1 0\n",
    "rtabmap_calib.yaml": b"%YAML:1.0\n---\ncamera_name: synthetic\n",
}


def run(argv):
    """Run the CLI, returning (exit code, parsed stdout JSON)."""
    stream = io.StringIO()
    with contextlib.redirect_stdout(stream):
        code = main(argv)
    text = stream.getvalue()
    return code, (json.loads(text) if text.strip() else None)


class BundleMapTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / "run-root"
        self.bundle = Path(self.temporary.name) / "map-bundle-v2"
        for name, relative in BUNDLE_FILES:
            target = self.root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(CONTENTS[name])
        self.map_id = hashlib.sha256(CONTENTS[DATABASE_NAME]).hexdigest()
        (self.root / "stage-summary-v2.json").write_text(json.dumps({
            "schema": 2,
            "experiment_id": "synthetic-run",
            "source_commit": "0" * 40,
        }))
        (self.root / "export-stage-v1" / "export-result.json").write_text(json.dumps({
            "schema": 1,
            "database_sha256_before": self.map_id,
            "database_sha256_after": self.map_id,
            "database_unchanged": True,
        }))

    def build(self, out=None):
        code, result = run(["build", "--run-root", str(self.root), "--out", str(out or self.bundle),
                            "--map-id", self.map_id, "--checkout", str(CHECKOUT)])
        self.assertEqual(code, 0)
        return result

    def check(self, extra=()):
        return run(["check", str(self.bundle), "--map-id", self.map_id, *extra])

    def read_bundle(self):
        return json.loads((self.bundle / "bundle.json").read_text())

    def test_build_then_check_passes(self):
        summary = self.build()
        self.assertEqual(summary["map_id"], self.map_id)
        bundle = self.read_bundle()
        self.assertEqual(list(bundle), [
            "schema", "map_id", "files", "navigation_ready", "map_frame_gravity_aligned",
            "world_frame", "tracked_frame", "pose_source", "source_run", "built_from_commit",
            "gate_constants", "limitations", "built_at"])
        self.assertEqual(bundle["schema"], 2)
        self.assertIs(bundle["navigation_ready"], False)
        self.assertIs(bundle["map_frame_gravity_aligned"], False)
        self.assertEqual(bundle["pose_source"], "estimated")
        self.assertEqual((bundle["world_frame"], bundle["tracked_frame"]), ("map", "camera_link"))
        self.assertEqual(bundle["source_run"]["experiment_id"], "synthetic-run")
        self.assertEqual(bundle["source_run"]["export_result"]["database_unchanged"], True)
        self.assertEqual(len(bundle["built_from_commit"]), 40)
        for name, relative in BUNDLE_FILES:
            record = bundle["files"][name]
            self.assertEqual(record["source"], relative)
            self.assertEqual(record["bytes"], len(CONTENTS[name]))
            self.assertEqual(record["sha256"], hashlib.sha256(CONTENTS[name]).hexdigest())
            self.assertEqual((self.bundle / name).read_bytes(), CONTENTS[name])
        self.assertTrue(bundle["limitations"])
        names = {entry["name"] for entry in bundle["gate_constants"]["constants"]}
        self.assertIn("healthy_acquisition_streak", names)
        # bundle.json is deliberately not self-listed in the manifest.
        manifest = (self.bundle / "MANIFEST.sha256").read_text()
        self.assertNotIn("./bundle.json", manifest)
        self.assertIn("./rtabmap.db", manifest)
        code, verdict = self.check()
        self.assertEqual(code, 0)
        self.assertTrue(verdict["pass"])
        self.assertEqual(verdict["failed_names"], [])
        self.assertEqual(verdict["checked"], len(BUNDLE_FILES))

    def test_tampered_payload_byte_fails(self):
        self.build()
        target = self.bundle / "optimized.pgm"
        data = bytearray(target.read_bytes())
        data[-1] ^= 0x01
        target.write_bytes(bytes(data))
        code, verdict = self.check()
        self.assertEqual(code, 1)
        self.assertFalse(verdict["pass"])
        # The on-disk bytes are compared against bundle.json, and the manifest body is
        # compared against bundle.json too, so a payload edit is named once, precisely.
        self.assertEqual(verdict["failed_names"], ["optimized.pgm"])

    def test_missing_file_fails(self):
        self.build()
        (self.bundle / "rtabmap_calib.yaml").unlink()
        code, verdict = self.check()
        self.assertEqual(code, 1)
        self.assertIn("rtabmap_calib.yaml", verdict["failed_names"])
        self.assertIn("missing", [item["problem"] for item in verdict["failures"]])

    def test_navigation_ready_flipped_needs_require_not_ready(self):
        self.build()
        bundle = self.read_bundle()
        bundle["navigation_ready"] = True
        (self.bundle / "bundle.json").write_text(json.dumps(bundle, indent=2) + "\n")
        # bundle.json is not manifest-listed and no payload changed, so the plain check passes.
        code, verdict = self.check()
        self.assertEqual(code, 0)
        self.assertTrue(verdict["pass"])
        code, verdict = self.check(["--require-not-ready"])
        self.assertEqual(code, 1)
        self.assertEqual(verdict["failed_names"], ["navigation_ready"])

    def test_declared_hash_mismatch_fails(self):
        """A `files` entry edited in bundle.json is refused even though bytes are intact."""
        self.build()
        bundle = self.read_bundle()
        bundle["files"]["optimized.yaml"]["sha256"] = "f" * 64
        (self.bundle / "bundle.json").write_text(json.dumps(bundle, indent=2) + "\n")
        code, verdict = self.check()
        self.assertEqual(code, 1)
        self.assertIn("optimized.yaml", verdict["failed_names"])

    def test_consistently_relabelled_payload_is_caught_by_the_manifest(self):
        """Editing the payload and its bundle.json hash together still fails the manifest."""
        self.build()
        target = self.bundle / "optimized_camera_poses.txt"
        target.write_bytes(b"0 0 0 0 0 0 0 0 0 0 0 0\n")
        bundle = self.read_bundle()
        bundle["files"]["optimized_camera_poses.txt"]["sha256"] = hashlib.sha256(
            target.read_bytes()).hexdigest()
        bundle["files"]["optimized_camera_poses.txt"]["bytes"] = target.stat().st_size
        (self.bundle / "bundle.json").write_text(json.dumps(bundle, indent=2) + "\n")
        code, verdict = self.check()
        self.assertEqual(code, 1)
        self.assertEqual(verdict["failed_names"], ["MANIFEST.sha256"])

    def test_wrong_map_id_fails(self):
        self.build()
        code, verdict = run(["check", str(self.bundle), "--map-id", "a" * 64])
        self.assertEqual(code, 1)
        self.assertIn("map_id", verdict["failed_names"])

    def test_build_refuses_existing_output(self):
        self.build()
        before = {path.name: path.read_bytes() for path in sorted(self.bundle.iterdir())}
        with self.assertRaises(SystemExit) as caught:
            run(["build", "--run-root", str(self.root), "--out", str(self.bundle),
                 "--map-id", self.map_id, "--checkout", str(CHECKOUT)])
        self.assertNotEqual(caught.exception.code, 0)
        self.assertIn("already exists", str(caught.exception))
        after = {path.name: path.read_bytes() for path in sorted(self.bundle.iterdir())}
        self.assertEqual(before, after)

    def test_build_refuses_wrong_expected_map_id(self):
        with self.assertRaises(SystemExit) as caught:
            run(["build", "--run-root", str(self.root), "--out", str(self.bundle),
                 "--map-id", "b" * 64, "--checkout", str(CHECKOUT)])
        self.assertIn("map identity mismatch", str(caught.exception))
        self.assertFalse(self.bundle.exists())

    def test_build_refuses_missing_input(self):
        (self.root / "export-stage-v1" / "optimized_cloud.ply").unlink()
        with self.assertRaises(SystemExit) as caught:
            run(["build", "--run-root", str(self.root), "--out", str(self.bundle),
                 "--map-id", self.map_id, "--checkout", str(CHECKOUT)])
        self.assertIn("missing frozen input", str(caught.exception))
        self.assertFalse(self.bundle.exists())


@unittest.skipUnless((FROZEN_BUNDLE / "bundle.json").is_file(),
                     "frozen map bundle not present next to this checkout")
class FrozenBundleTests(unittest.TestCase):
    def test_frozen_bundle_checks_out(self):
        code, verdict = run(["check", str(FROZEN_BUNDLE), "--map-id", FROZEN_MAP_ID,
                             "--require-not-ready"])
        self.assertEqual(code, 0, verdict)
        self.assertTrue(verdict["pass"])
        self.assertEqual(verdict["map_id"], FROZEN_MAP_ID)
        self.assertIs(verdict["navigation_ready"], False)


if __name__ == "__main__":
    unittest.main()
