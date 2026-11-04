#!/usr/bin/env python3
"""Run the fixed RTAB-Map mapping, export, or reload stage in one owned container."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import time


IMAGE = "fpga-dev1-rtabmap:humble-0.23.7-20260910"
CONTAINER_PREFIX = "fpga-dev1-rtabmap-reload-coordinate-20260912"
DATASET_NAME = "rgbd_dataset_freiburg1_desk"


def container_prefix(value):
    """Require a bounded fpga-member-purpose prefix safe for Docker names."""
    if (not isinstance(value, str) or len(value) > 200
            or re.fullmatch(r"fpga-[a-z0-9]+-[a-z0-9]+(?:-[a-z0-9]+)*", value) is None):
        raise argparse.ArgumentTypeError(
            "container prefix must be a lowercase fpga-member-purpose name")
    return value


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def docker(*args, check=True, capture=True, timeout=None):
    return subprocess.run(["docker", *args], check=check, text=True,
                          stdout=subprocess.PIPE if capture else None,
                          stderr=subprocess.STDOUT if capture else None,
                          timeout=timeout)


def image_id():
    return docker("image", "inspect", "--format", "{{.Id}}", IMAGE).stdout.strip()


def ensure_absent_container(name):
    check = docker("inspect", name, check=False)
    if check.returncode == 0:
        raise ValueError(f"container already exists: {name}")


def run_container(name, command, record_dir, timeout, host_identity=None):
    ensure_absent_container(name)
    record_dir = Path(record_dir)
    record_dir.mkdir(parents=True, exist_ok=False)
    started = time.time()
    record = {"schema": 1, "container": name, "image": IMAGE, "image_id": image_id(),
              "argv": ["docker", *command], "timeout_seconds": timeout,
              "started_unix_s": started, "host_identity": host_identity}
    (record_dir / "docker-command.json").write_text(json.dumps(record, indent=2) + "\n")
    created = False
    timed_out = False
    try:
        created_result = docker(*command)
        created = True
        (record_dir / "container-create.txt").write_text(created_result.stdout)
        container_id = json.loads(docker("inspect", name).stdout)[0]["Id"]
        with (record_dir / "container.log").open("x") as stream:
            try:
                completed = subprocess.run(["docker", "start", "--attach", name], text=True,
                                           stdout=stream, stderr=subprocess.STDOUT,
                                           timeout=timeout)
                start_returncode = completed.returncode
            except subprocess.TimeoutExpired:
                timed_out = True
                docker("stop", "--time", "10", name, check=False)
                start_returncode = None
        inspection = json.loads(docker("inspect", name).stdout)[0]
        (record_dir / "docker-inspect.json").write_text(json.dumps(inspection, indent=2) + "\n")
        result = {"schema": 1, "container": name, "container_id": container_id,
                  "docker_start_returncode": start_returncode,
                  "container_exit_code": inspection["State"]["ExitCode"],
                  "timed_out": timed_out, "running_after": inspection["State"]["Running"],
                  "finished_unix_s": time.time(), "wall_seconds": time.time() - started}
        (record_dir / "container-result.json").write_text(json.dumps(result, indent=2) + "\n")
        if timed_out or inspection["State"]["ExitCode"] != 0 or inspection["State"]["Running"]:
            raise RuntimeError(f"container stage failed: {result}")
        return result
    finally:
        if created:
            final_check = docker("inspect", name, check=False)
            if final_check.returncode == 0:
                final_inspection = json.loads(final_check.stdout)[0]
                if final_inspection["State"]["Running"]:
                    stopped = docker("stop", "--time", "10", name, check=False)
                    (record_dir / "container-final-stop.txt").write_text(
                        f"returncode={stopped.returncode}\n{stopped.stdout}")
                    final_check = docker("inspect", name, check=False)
                    if final_check.returncode == 0 and json.loads(final_check.stdout)[0]["State"]["Running"]:
                        killed = docker("kill", name, check=False)
                        (record_dir / "container-final-kill.txt").write_text(
                            f"returncode={killed.returncode}\n{killed.stdout}")
                        final_check = docker("inspect", name, check=False)
                if final_check.returncode == 0:
                    (record_dir / "docker-final-inspect.json").write_text(
                        json.dumps(json.loads(final_check.stdout)[0], indent=2) + "\n")
            removed = docker("rm", name, check=False)
            (record_dir / "container-removal.txt").write_text(
                f"returncode={removed.returncode}\n{removed.stdout}")


def common_create(name):
    return ["create", "--name", name, "--init", "--network", "none", "--cpus", "4",
            "--memory", "8g"]


def mapping(args):
    dataset = args.input.resolve()
    if dataset.name != DATASET_NAME or not (dataset / "INPUT.json").is_file():
        raise ValueError("mapping input must retain the freiburg1 dataset directory name")
    manifest_path = dataset / "INPUT.json"
    calibration_path = dataset / "rtabmap_calib.yaml"
    manifest = json.loads(manifest_path.read_text())
    calibration_present = calibration_path.is_file()
    calibration_hash = sha256(calibration_path) if calibration_present else None
    if (not args.diagnostic_calibration_control
            and (not calibration_present
                 or manifest.get("calibration", {}).get("input_sha256") != calibration_hash)):
        raise ValueError("mapping calibration file/manifest identity mismatch")
    if (calibration_present
            and manifest.get("calibration", {}).get("input_sha256") != calibration_hash):
        raise ValueError("present mapping calibration does not match manifest")
    stage = args.output.resolve()
    name = args.container_prefix + "-mapping"
    command = [*common_create(name),
        "--tmpfs", "/blank:ro,nosuid,nodev,noexec,size=4096",
        "--mount", f"type=bind,src={dataset},dst=/normal/{DATASET_NAME},readonly",
        "--mount", f"type=bind,src={stage},dst=/results",
        "--mount", f"type=bind,src={args.offline_probe.resolve()},dst=/probe_tools/rtabmap_offline_probe.sh,readonly",
        IMAGE, "/probe_tools/rtabmap_offline_probe.sh", "normal"]
    host_identity = {"input_manifest_path": str(manifest_path),
                     "input_manifest_sha256": sha256(manifest_path),
                     "diagnostic_calibration_control": args.diagnostic_calibration_control,
                     "calibration_path": str(calibration_path) if calibration_present else None,
                     "calibration_sha256": calibration_hash}
    result = run_container(name, command, stage, 1230, host_identity)
    database = stage / "normal" / "rtabmap.db"
    if not database.is_file():
        raise RuntimeError("mapping did not create rtabmap.db")
    result["database"] = str(database)
    result["database_sha256"] = sha256(database)
    (stage / "mapping-result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def export(args):
    database = args.database.resolve()
    if not database.is_file():
        raise ValueError("mapping database is missing")
    before = sha256(database)
    stage = args.output.resolve()
    name = args.container_prefix + "-export"
    command = [*common_create(name),
        "--tmpfs", "/normal:ro,nosuid,nodev,noexec,size=4096",
        "--tmpfs", "/blank:ro,nosuid,nodev,noexec,size=4096",
        "--tmpfs", "/results:ro,nosuid,nodev,noexec,size=4096",
        "--mount", f"type=bind,src={database},dst=/mapping/rtabmap.db,readonly",
        "--mount", f"type=bind,src={stage},dst=/export",
        IMAGE, "-c", "source /opt/ros/humble/setup.bash; exec \"$@\"", "bash",
        "timeout", "--signal=INT", "--kill-after=30s", "300s", "rtabmap-export",
        "--cloud", "--map", "--poses_camera",
        "--poses_format", "10", "--opt", "2", "--output", "optimized",
        "--output_dir", "/export", "/mapping/rtabmap.db"]
    result = run_container(name, command, stage, 330)
    after = sha256(database)
    result.update({"database_sha256_before": before, "database_sha256_after": after,
                   "database_unchanged": before == after})
    if before != after or not (stage / "optimized_camera_poses.txt").is_file():
        raise RuntimeError("export changed the database or omitted camera poses")
    (stage / "export-result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def query(args):
    dataset = args.input.resolve()
    database = args.database.resolve()
    checkout = Path(__file__).resolve().parents[1]
    reference = args.reference_identity.resolve()
    if dataset.name != DATASET_NAME or not (dataset / "INPUT.json").is_file():
        raise ValueError("query input must retain the freiburg1 dataset directory name")
    if not database.is_file():
        raise ValueError("query database is missing")
    if not reference.is_file():
        raise ValueError("frozen reference identity is missing")
    query_manifest = json.loads((dataset / "INPUT.json").read_text())
    calibration_path = dataset / "rtabmap_calib.yaml"
    if (not calibration_path.is_file()
            or query_manifest.get("calibration", {}).get("input_sha256") != sha256(calibration_path)):
        raise ValueError("query calibration file/manifest identity mismatch")
    stage = args.output.resolve()
    mode = args.mode
    name = args.container_prefix + "-query-" + mode
    database_role = "normal" if mode == "normal" else "blank"
    database_mount = stage.with_name(stage.name + "-database-input")
    if database_mount.exists():
        raise FileExistsError(database_mount)
    (database_mount / database_role).mkdir(parents=True)
    mounted_database = database_mount / database_role / "rtabmap.db"
    shutil.copyfile(database, mounted_database)
    if sha256(mounted_database) != sha256(database):
        raise RuntimeError("query database staging copy hash mismatch")
    probe_args = ["python3", "/probe_tools/rtabmap_ros_probe.py", mode,
                  "--mode", "normal" if mode == "normal" else "empty-writable",
                  "--reset-countdown", "10", "--odom-tf-wait", "0",
                  "--input", f"/query/{DATASET_NAME}", "--start-index", "0",
                  "--stop-index", "230", "--timeout-seconds", "600"]
    if mode == "normal":
        probe_args += ["--online-gate", "--gate-rgb-reliable"]
    command = [*common_create(name),
        "--env", "PYTHONPATH=/checkout",
        "--tmpfs", "/normal:ro,nosuid,nodev,noexec,size=4096",
        "--tmpfs", "/blank:ro,nosuid,nodev,noexec,size=4096",
        "--mount", f"type=bind,src={dataset},dst=/query/{DATASET_NAME},readonly",
        "--mount", f"type=bind,src={database_mount},dst=/results,readonly",
        "--mount", f"type=bind,src={stage},dst=/rosresults",
        "--mount", f"type=bind,src={(checkout / 'tools').resolve()},dst=/probe_tools,readonly",
        "--mount", f"type=bind,src={(checkout / 'host').resolve()},dst=/checkout/host,readonly",
        IMAGE, "-c", "source /opt/ros/humble/setup.bash; exec \"$@\"", "bash",
        *probe_args]
    reference_record = json.loads(reference.read_text())
    host_identity = {
        "reference_identity_path": str(reference),
        "reference_identity_sha256": sha256(reference),
        "reference_code_commit": reference_record.get("code_commit"),
        "database_path": str(database),
        "database_sha256": sha256(database),
        "database_mount_path": str(database_mount),
        "database_mount_sha256": sha256(mounted_database),
        "query_manifest_path": str(dataset / "INPUT.json"),
        "query_manifest_sha256": sha256(dataset / "INPUT.json"),
        "calibration_path": str(calibration_path),
        "calibration_sha256": sha256(calibration_path),
        "mounted_source_hashes": {
            "runner": sha256(Path(__file__)),
            "probe": sha256(checkout / "tools/rtabmap_ros_probe.py"),
            "replay": sha256(checkout / "tools/rtabmap_ros_replay.py"),
            "gate": sha256(checkout / "host/ros_localization_gate.py"),
        },
        "reference_was_mounted": False,
    }
    result = run_container(name, command, stage, 630, host_identity)
    run = stage / mode
    if not (run / "runner-result.json").is_file():
        raise RuntimeError("query did not create runner result")
    result["run"] = str(run)
    result["input_database_sha256"] = sha256(database)
    (stage / "query-result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="stage", required=True)
    mapping_parser = sub.add_parser("mapping")
    mapping_parser.add_argument("--container-prefix", type=container_prefix,
                                default=CONTAINER_PREFIX)
    mapping_parser.add_argument("--input", required=True, type=Path)
    mapping_parser.add_argument("--output", required=True, type=Path)
    mapping_parser.add_argument("--offline-probe", type=Path,
                                default=Path(__file__).with_name("rtabmap_offline_probe.sh"))
    mapping_parser.add_argument("--diagnostic-calibration-control", action="store_true",
                                help="bounded absent/present calibration control only")
    export_parser = sub.add_parser("export")
    export_parser.add_argument("--container-prefix", type=container_prefix,
                               default=CONTAINER_PREFIX)
    export_parser.add_argument("--database", required=True, type=Path)
    export_parser.add_argument("--output", required=True, type=Path)
    query_parser = sub.add_parser("query")
    query_parser.add_argument("--container-prefix", type=container_prefix,
                              default=CONTAINER_PREFIX)
    query_parser.add_argument("--mode", choices=("normal", "empty"), required=True)
    query_parser.add_argument("--input", required=True, type=Path)
    query_parser.add_argument("--database", required=True, type=Path)
    query_parser.add_argument("--reference-identity", required=True, type=Path,
                              help="host-side hash record only; never mounted in the query container")
    query_parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        if args.output.exists():
            raise FileExistsError(args.output)
        result = {"mapping": mapping, "export": export, "query": query}[args.stage](args)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError, json.JSONDecodeError) as exc:
        parser.exit(2, f"reload coordinate stage failed: {exc}\n")
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
