#!/usr/bin/env python3
"""Assemble and verify the RTAB-Map map bundle handed to the Nav2 integration.

The bundle is a copy-only package: it re-hashes frozen run artifacts and copies
them next to a machine-readable `bundle.json`. No algorithm is rerun, no source
run file is written, and nothing here makes the map navigable — `navigation_ready`
stays false until joint acceptance says otherwise.

Sub-commands
  build  copy the frozen artifacts into a new bundle directory and write
         `bundle.json` + `MANIFEST.sha256` (refuses to touch an existing directory)
  check  re-hash every listed file, re-verify the map identity and the manifest
         body, and print a JSON verdict (exit 0 only when everything matches)
"""

import argparse
import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys


SCHEMA = 2

# name in the bundle -> path relative to the frozen run root
BUNDLE_FILES = (
    ("rtabmap.db", "mapping-stage-v4/normal/rtabmap.db"),
    ("optimized.pgm", "export-stage-v1/optimized.pgm"),
    ("optimized.yaml", "export-stage-v1/optimized.yaml"),
    ("optimized_cloud.ply", "export-stage-v1/optimized_cloud.ply"),
    ("optimized_camera_poses.txt", "export-stage-v1/optimized_camera_poses.txt"),
    ("rtabmap_calib.yaml", "prepared-v2/mapping/rgbd_dataset_freiburg1_desk/rtabmap_calib.yaml"),
)
DATABASE_NAME = "rtabmap.db"
SUMMARY_NAME = "stage-summary-v2.json"
EXPORT_RESULT = "export-stage-v1/export-result.json"
EXPORT_KEYS = ("database_sha256_before", "database_sha256_after", "database_unchanged")

BUNDLE_JSON = "bundle.json"
MANIFEST_NAME = "MANIFEST.sha256"

GATE_SOURCE = "host/localization_gate.py"

# The gate's policy numbers are inline literals in the source, not named constants.
# Each entry therefore cites the enclosing symbol plus the verbatim source line it
# was read from; `build` refuses to run unless that line still occurs exactly once,
# so a silent policy change cannot be recorded under an old value.
GATE_CONSTANTS = (
    {
        "name": "healthy_acquisition_streak",
        "value": 3,
        "unit": "consecutive healthy acquisitions",
        "meaning": "acquisitions required before the gate state becomes valid",
        "symbol": "LocalizationGate.ingest",
        "source_line": "self.state = 'valid' if self.streak >= 3 else 'confirming'",
    },
    {
        "name": "incomplete_acquisition_wall_timeout_s",
        "value": 0.5,
        "unit": "s (wall clock)",
        "meaning": "a pending acquisition missing any of the four streams is retired and revokes",
        "symbol": "LocalizationGate.tick",
        "source_line": "if wall-item['wall'] >= .5 or ros_ns-stamp > 10**9:",
    },
    {
        "name": "pending_acquisition_stamp_age_limit_ns",
        "value": 1000000000,
        "unit": "ns (ROS clock)",
        "meaning": "same line: a pending acquisition older than this against the ROS clock is retired",
        "symbol": "LocalizationGate.tick",
        "source_line": "if wall-item['wall'] >= .5 or ros_ns-stamp > 10**9:",
    },
    {
        "name": "localization_stream_wall_timeout_s",
        "value": 1.0,
        "unit": "s (wall clock)",
        "meaning": "valid/confirming goes stale when no healthy acquisition completed within this window",
        "symbol": "LocalizationGate.tick",
        "source_line": "if self.state in ('valid', 'confirming') and (wall-self.last_good_wall >= 1. or ros_ns-self.last_complete > 10**9):",
    },
    {
        "name": "approved_pose_retirement_age_ns",
        "value": 1000000000,
        "unit": "ns (ROS clock)",
        "meaning": "an approved pose older than this against the ROS clock is dropped from the status",
        "symbol": "LocalizationGate.tick",
        "source_line": "if ros_ns-stamp > 10**9:",
    },
    {
        "name": "acquisition_stamp_age_limit_ns",
        "value": 1000000000,
        "unit": "ns (ROS clock)",
        "meaning": "an acquisition stamp further than this behind the ROS clock is rejected",
        "symbol": "LocalizationGate.acquisition",
        "source_line": "if type(stamp) is not int or stamp <= self.last_input or stamp <= 0 or stamp > ros_ns or ros_ns-stamp > 10**9:",
    },
    {
        "name": "pending_acquisition_capacity",
        "value": 32,
        "unit": "acquisitions",
        "meaning": "pending overflow clears the queue and revokes",
        "symbol": "LocalizationGate.acquisition",
        "source_line": "if len(self.pending) >= 32:",
    },
    {
        "name": "approved_pose_capacity",
        "value": 32,
        "unit": "poses",
        "meaning": "oldest approved poses are dropped past this bound",
        "symbol": "LocalizationGate.ingest",
        "source_line": "while len(self.approved) > 32:",
    },
    {
        "name": "covariance_diagonal_sentinel",
        "value": 9999.0,
        "unit": "covariance units",
        "meaning": "RTAB-Map's lost sentinel; a diagonal at or above it fails the covariance check",
        "symbol": "covariance_valid",
        "source_line": "and np.diag(cov).max() < 9999.)",
    },
    {
        "name": "lease_status_heartbeat_s",
        "value": 0.5,
        "unit": "s (consumer steady clock)",
        "meaning": "consumer lease expires without a fresher status inside this window",
        "symbol": "LocalizationLease.check",
        "source_line": "healthy = bool(row and row['state'] == 'valid' and 0 <= wall-self.received < .5",
    },
    {
        "name": "lease_acquisition_progress_s",
        "value": 1.0,
        "unit": "s (consumer steady clock)",
        "meaning": "same line: lease also requires a distinct acquisition stamp inside this window, and a status stamp no older than 1e9 ns",
        "symbol": "LocalizationLease.check",
        "source_line": "and 0 <= wall-self.progress < 1. and 0 <= ros_ns-row['stamp_ns'] <= 10**9)",
    },
)

LIMITATIONS = [
    "the recording is TUM RGB-D fr1_desk, a handheld camera sequence, not a wheeled robot run",
    "the map frame is not gravity-aligned, so optimized.pgm is an occupancy projection and not a Nav2 costmap",
    "51 of 230 approved query frames in the frozen replay is acquisition-count coverage, not navigation uptime",
    "no real robot extrinsics are contained here; T_base_camera has no measured source yet",
    "navigation_ready stays false until joint acceptance with the Nav2 side says otherwise",
]


def digest_and_size(path):
    hasher = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            hasher.update(block)
            size += len(block)
    return hasher.hexdigest(), size


def read_gate_constants(checkout):
    """Read the gate's policy numbers, pinned to their verbatim source lines."""
    source = checkout / GATE_SOURCE
    text = source.read_text()
    stripped = [line.strip() for line in text.splitlines()]
    recorded = []
    for entry in GATE_CONSTANTS:
        hits = [index + 1 for index, line in enumerate(stripped) if line == entry["source_line"]]
        if len(hits) != 1:
            raise SystemExit(
                "gate constant %s: expected exactly one source line in %s, found %d\n  %s"
                % (entry["name"], GATE_SOURCE, len(hits), entry["source_line"]))
        recorded.append(dict(entry, source_line_number=hits[0]))
    return {
        "source_file": GATE_SOURCE,
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "note": "inline literals in the gate source, not named constants; each entry cites the verbatim line it was read from",
        "constants": recorded,
    }


def git_head(checkout):
    result = subprocess.run(["git", "-C", str(checkout), "rev-parse", "HEAD"],
                            capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit("git rev-parse HEAD failed in %s: %s" % (checkout, result.stderr.strip()))
    return result.stdout.strip()


def write_manifest(bundle_dir, checkout):
    """Generate MANIFEST.sha256 with tools/manifest.sh conventions (body `sha  ./name`)."""
    script = checkout / "tools" / "manifest.sh"
    result = subprocess.run(["bash", str(script), str(bundle_dir)], capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit("manifest generation failed: %s%s" % (result.stdout, result.stderr))
    return result.stdout.strip()


def manifest_body(path):
    """Parse a MANIFEST.sha256 into {relative path: sha256}; `#` header lines are ignored."""
    entries = {}
    for line in path.read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        entries[line[66:]] = line[:64]
    return entries


def build(args):
    run_root, bundle_dir, checkout = args.run_root, args.out, args.checkout
    sources = {}
    for name, relative in BUNDLE_FILES:
        source = run_root / relative
        if not source.is_file():
            raise SystemExit("missing frozen input: %s" % source)
        sha256, size = digest_and_size(source)
        sources[name] = {"sha256": sha256, "bytes": size, "source": relative, "path": source}

    database = sources[DATABASE_NAME]
    if database["sha256"] != args.map_id:
        raise SystemExit("map identity mismatch: %s is %s, expected %s"
                         % (DATABASE_NAME, database["sha256"], args.map_id))

    summary = json.loads((run_root / SUMMARY_NAME).read_text())
    export = json.loads((run_root / EXPORT_RESULT).read_text())
    gate_constants = read_gate_constants(checkout)
    built_from_commit = git_head(checkout)

    # Refuse an existing output directory before anything is written.
    try:
        bundle_dir.mkdir(parents=True)
    except FileExistsError:
        raise SystemExit("output directory already exists, refusing to overwrite: %s" % bundle_dir)

    files = {}
    for name, _ in BUNDLE_FILES:
        record = sources[name]
        destination = bundle_dir / name
        shutil.copyfile(record["path"], destination)
        sha256, size = digest_and_size(destination)
        if (sha256, size) != (record["sha256"], record["bytes"]):
            raise SystemExit("copy differs from source: %s" % name)
        files[name] = {"sha256": sha256, "bytes": size, "source": record["source"]}

    # The manifest covers the copied payload only; bundle.json is written afterwards
    # and is verified through `check`, not through the manifest.
    manifest_line = write_manifest(bundle_dir, checkout)

    bundle = {
        "schema": SCHEMA,
        "map_id": files[DATABASE_NAME]["sha256"],
        "files": files,
        "navigation_ready": False,
        "map_frame_gravity_aligned": False,
        "world_frame": "map",
        "tracked_frame": "camera_link",
        "pose_source": "estimated",
        "source_run": {
            "experiment_id": summary["experiment_id"],
            "source_commit": summary["source_commit"],
            "export_result": {key: export[key] for key in EXPORT_KEYS},
        },
        "built_from_commit": built_from_commit,
        "gate_constants": gate_constants,
        "limitations": list(LIMITATIONS),
        "built_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
    }
    with (bundle_dir / BUNDLE_JSON).open("x") as stream:
        json.dump(bundle, stream, indent=2, allow_nan=False)
        stream.write("\n")

    print(json.dumps({"built": str(bundle_dir), "map_id": bundle["map_id"],
                      "files": len(files), "navigation_ready": bundle["navigation_ready"],
                      "built_from_commit": built_from_commit, "manifest": manifest_line}, indent=2))
    return 0


def check(args):
    bundle_dir = args.bundle_dir
    failures = []
    bundle_path = bundle_dir / BUNDLE_JSON
    if not bundle_path.is_file():
        print(json.dumps({"schema": SCHEMA, "bundle_dir": str(bundle_dir), "pass": False,
                          "checked": 0, "failures": [{"name": BUNDLE_JSON, "problem": "missing"}]},
                         indent=2))
        return 1
    try:
        bundle = json.loads(bundle_path.read_text())
    except ValueError:
        print(json.dumps({"schema": SCHEMA, "bundle_dir": str(bundle_dir), "pass": False,
                          "checked": 0, "failures": [{"name": BUNDLE_JSON, "problem": "unparsable"}]},
                         indent=2))
        return 1

    if bundle.get("schema") != SCHEMA:
        failures.append({"name": BUNDLE_JSON, "problem": "unexpected schema %r" % bundle.get("schema")})

    files = bundle.get("files")
    if not isinstance(files, dict) or not files:
        failures.append({"name": BUNDLE_JSON, "problem": "no files listed"})
        files = {}

    observed = {}
    for name in sorted(files):
        record = files[name]
        target = bundle_dir / name
        if not target.is_file():
            failures.append({"name": name, "problem": "missing"})
            continue
        sha256, size = digest_and_size(target)
        observed[name] = sha256
        if sha256 != record.get("sha256"):
            failures.append({"name": name, "problem": "sha256 mismatch",
                             "expected": record.get("sha256"), "observed": sha256})
        if size != record.get("bytes"):
            failures.append({"name": name, "problem": "size mismatch",
                             "expected": record.get("bytes"), "observed": size})

    database = files.get(DATABASE_NAME)
    if database is None:
        failures.append({"name": DATABASE_NAME, "problem": "not listed in bundle.json"})
    else:
        if bundle.get("map_id") != database.get("sha256"):
            failures.append({"name": "map_id", "problem": "map_id does not match the database sha256",
                             "expected": database.get("sha256"), "observed": bundle.get("map_id")})
        if DATABASE_NAME in observed and observed[DATABASE_NAME] != bundle.get("map_id"):
            failures.append({"name": "map_id", "problem": "map_id does not match the database on disk",
                             "expected": observed[DATABASE_NAME], "observed": bundle.get("map_id")})
    if bundle.get("map_id") != args.map_id:
        failures.append({"name": "map_id", "problem": "map_id does not match --map-id",
                         "expected": args.map_id, "observed": bundle.get("map_id")})

    manifest_path = bundle_dir / MANIFEST_NAME
    if not manifest_path.is_file():
        failures.append({"name": MANIFEST_NAME, "problem": "missing"})
    else:
        body = manifest_body(manifest_path)
        expected = {"./" + name: record.get("sha256") for name, record in files.items()}
        for key in sorted(set(body) | set(expected)):
            if key not in body:
                failures.append({"name": MANIFEST_NAME, "problem": "entry missing from manifest: %s" % key})
            elif key not in expected:
                failures.append({"name": MANIFEST_NAME, "problem": "unexpected manifest entry: %s" % key})
            elif body[key] != expected[key]:
                failures.append({"name": MANIFEST_NAME, "problem": "manifest sha256 mismatch: %s" % key,
                                 "expected": expected[key], "observed": body[key]})

    if args.require_not_ready and bundle.get("navigation_ready") is not False:
        failures.append({"name": "navigation_ready",
                         "problem": "navigation_ready is %r, joint acceptance has not been recorded here"
                                    % bundle.get("navigation_ready")})

    verdict = {
        "schema": SCHEMA,
        "bundle_dir": str(bundle_dir),
        "map_id": bundle.get("map_id"),
        "navigation_ready": bundle.get("navigation_ready"),
        "checked": len(files),
        "pass": not failures,
        "failed_names": sorted({item["name"] for item in failures}),
        "failures": failures,
    }
    print(json.dumps(verdict, indent=2))
    return 0 if not failures else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    default_checkout = Path(__file__).resolve().parents[1]
    sub = parser.add_subparsers(dest="command", required=True)

    builder = sub.add_parser("build", help="copy the frozen artifacts into a new bundle directory")
    builder.add_argument("--run-root", type=Path, required=True, help="frozen run root (read-only)")
    builder.add_argument("--out", type=Path, required=True, help="bundle directory to create; must not exist")
    builder.add_argument("--map-id", required=True, help="expected sha256 of rtabmap.db")
    builder.add_argument("--checkout", type=Path, default=default_checkout,
                         help="repository checkout the gate constants and commit are read from")
    builder.set_defaults(handler=build)

    checker = sub.add_parser("check", help="re-verify an existing bundle directory")
    checker.add_argument("bundle_dir", type=Path)
    checker.add_argument("--map-id", required=True, help="expected map identity")
    checker.add_argument("--require-not-ready", action="store_true",
                         help="also fail when navigation_ready is not false")
    checker.set_defaults(handler=check)

    args = parser.parse_args(argv)
    return args.handler(args)


if __name__ == "__main__":
    sys.exit(main())
