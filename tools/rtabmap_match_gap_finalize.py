#!/usr/bin/env python3
"""Create the final bounded-run receipt for the match-gap diagnosis."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess


EXPECTED_IMAGE = "sha256:c96783dbd884583a013eabfc393344f42473bfb12e51fc46979f172fed83ace7"
EXPECTED_DATABASE = "2a450fd79970fe4c145af5afab6b76a9420b2880c8bbb70581e9aafa1bfab8c7"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parse_script_times(path):
    text = Path(path).read_text(errors="replace")
    starts = re.findall(r"Script started on ([^\[]+)", text)
    ends = re.findall(r"Script done on ([^\[]+)", text)
    if len(starts) != 1 or len(ends) != 1:
        raise ValueError(f"missing script timestamps: {path}")
    start = datetime.fromisoformat(starts[0].strip())
    end = datetime.fromisoformat(ends[0].strip())
    return start, end, (end-start).total_seconds()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("container_inspect", type=Path)
    parser.add_argument("database", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("output exists")
    inspect = json.loads(args.container_inspect.read_text())
    if len(inspect) != 1:
        raise ValueError("unexpected inspect population")
    container = inspect[0]; host = container["HostConfig"]
    if (container["Image"] != EXPECTED_IMAGE or container["State"]["ExitCode"] != 0
            or container["State"]["Running"] or host["NetworkMode"] != "none"
            or host["NanoCpus"] != 4_000_000_000 or host["Memory"] != 8*1024**3
            or host["Devices"] or host["DeviceRequests"]):
        raise ValueError("container isolation/exit check failed")
    mounts = container["Mounts"]
    if not all(mount["RW"] is False for mount in mounts if mount["Destination"] != "/out"):
        raise ValueError("algorithm input mount is writable")
    started = datetime.fromisoformat(container["State"]["StartedAt"].replace("Z", "+00:00"))
    finished = datetime.fromisoformat(container["State"]["FinishedAt"].replace("Z", "+00:00"))
    algorithm_wall = (finished-started).total_seconds()
    transcripts = [args.root/"pair-build-v1/transcript.txt", args.root/"pair-build-v2/transcript.txt",
                   args.root/"pair-build-v3/transcript.txt"]
    build_times = [parse_script_times(path) for path in transcripts]
    earliest = min([time[0] for time in build_times] + [parse_script_times(args.root/"failure-stage2-prepare-v1/transcript.txt")[0]])
    finalized = datetime.now().astimezone()
    upper_bound = (finalized-earliest).total_seconds()
    if algorithm_wall >= 1800 or upper_bound >= 3600:
        raise ValueError("preregistered wall budget exceeded")
    database_hash = sha(args.database)
    if database_hash != EXPECTED_DATABASE:
        raise ValueError("frozen database hash mismatch")
    pairs = args.root/"stage2-pair-v2/pairs/pairs.jsonl"
    if len(pairs.read_text().splitlines()) != 292:
        raise ValueError("unexpected pair output count")
    live = subprocess.run(["docker", "ps", "-a", "--filter", "name=fpga-dev1-match-gap", "--filter", "status=running",
                           "--format", "{{.Names}}"], text=True, capture_output=True, check=True).stdout.splitlines()
    if live:
        raise ValueError(f"match-gap container still running: {live}")
    tracked = [
        args.root/"stage1-v2/summary.json", args.root/"stage1-v2/ledger.json",
        args.root/"stage2-prepare-v2/summary.json", args.root/"stage2-prepare-v2/selection.json",
        pairs, args.root/"stage2-eval-v2/summary.json", args.root/"stage2-eval-v2/rows.json",
        args.root/"stage2-eval-v2/pairs-enriched.json", args.root/"stage2-eval-v2/representative-links.png",
        args.container_inspect,
    ] + transcripts
    result = {
        "git_head": subprocess.run(["git", "rev-parse", "HEAD"], text=True, capture_output=True, check=True).stdout.strip(),
        "fixed_image": EXPECTED_IMAGE, "database_sha256_before_after": [database_hash, sha(args.database)],
        "algorithm_container": {"id": container["Id"], "exit_code": container["State"]["ExitCode"],
            "network": host["NetworkMode"], "cpus": host["NanoCpus"]/1e9, "memory_bytes": host["Memory"],
            "devices": host["Devices"], "device_requests": host["DeviceRequests"],
            "started_at": container["State"]["StartedAt"], "finished_at": container["State"]["FinishedAt"],
            "wall_seconds": algorithm_wall, "command": container["Config"]["Cmd"],
            "mounts": [{key: mount[key] for key in ("Source", "Destination", "RW")} for mount in mounts]},
        "build_runs": [{"path": str(path), "start": start.isoformat(), "end": end.isoformat(), "wall_seconds": wall}
                       for path, (start, end, wall) in zip(transcripts, build_times)],
        "stage2_wall_upper_bound": {"start": earliest.isoformat(), "finalized": finalized.isoformat(),
                                    "seconds": upper_bound, "limit_seconds": 3600},
        "algorithm_wall_limit_seconds": 1800,
        "source_sha256": {str(path): sha(path) for path in tracked},
        "live_match_gap_containers": live,
        "limits": ["The stage2 wall value is a conservative session-envelope upper bound including pauses.",
                   "Build v1 failed before compilation; build v2 exposed a control label issue; only v3 binary ran the 12x24 algorithm once.",
                   "No performance claim is made from these wall times."],
    }
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False); stream.write("\n")
    print(json.dumps({"algorithm_wall_seconds": algorithm_wall,
                      "stage2_wall_upper_bound_seconds": upper_bound,
                      "live_match_gap_containers": live}, indent=2))


if __name__ == "__main__":
    main()
