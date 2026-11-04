"""Replay the accepted 09-21 ROS 2 memory-transport probe on an arm64 board.

The 09-21 formal run (source d1fda80, tools/m7_memory_ros2_probe.py) copies a
frozen x86-64 ``native-m7.so`` built by the earlier Unix-socket probe.  That
shared object cannot be loaded on the Jetson (aarch64).  This wrapper changes
exactly one thing: it compiles the *same* two C++ sources (hash-checked against
the probe's own FROZEN_CPP_SHA256) with the *same* compile_native() flags on
the board, writes a one-row manifest for the new library, points the probe's
three FROZEN_* constants at it, and then calls the unchanged probe ``run``.

Everything else -- scenarios, QoS checks, fixtures, pass criteria -- is the
unchanged d1fda80 probe code.  Run it inside the ROS 2 Humble container with
the d1fda80 source subset mounted read-only at /checkout, a fresh empty results
directory mounted at /rosresults (the probe writes there directly, exactly as on
09-21, so absolute paths embedded in the memory store are the same) and a
separate fresh directory mounted at /rosbuild for the board-local library.
"""

import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
from pathlib import Path


def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkout", type=Path, default=Path("/checkout"))
    parser.add_argument("--results", type=Path, default=Path("/rosresults"))
    parser.add_argument("--build-dir", type=Path, default=Path("/rosbuild"))
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--source-tag", action="append", required=True)
    args = parser.parse_args(argv)

    checkout = args.checkout.resolve()
    sys.path.insert(0, str(checkout))
    import tools.m7_memory_ros2_probe as probe
    from host.m7_native import compile_native

    for relative, expected in probe.FROZEN_CPP_SHA256.items():
        if file_sha(checkout / relative) != expected:
            raise RuntimeError(f"{relative} differs from the source of the frozen library")

    build = args.build_dir / "native-build"
    build.mkdir(parents=True, exist_ok=False)
    library = compile_native(build / "native-m7.so")
    library_sha = file_sha(library)
    gxx = subprocess.run(["g++", "--version"], capture_output=True, text=True,
                         check=True).stdout.splitlines()[0]
    manifest = {
        "note": "board-local rebuild replacing the x86-64 frozen library; same sources and flags",
        "machine": platform.machine(),
        "compiler": gxx,
        "cpp_sha256": {name: file_sha(checkout / name) for name in probe.FROZEN_CPP_SHA256},
        "replaced_frozen_library_sha256": probe.FROZEN_LIBRARY_SHA256,
        "files": [{"path": "native-m7.so", "sha256": library_sha,
                   "size": library.stat().st_size}],
    }
    manifest_path = build / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=1, sort_keys=True) + "\n",
                             encoding="utf8")

    probe.FROZEN_LIBRARY = library
    probe.FROZEN_MANIFEST = manifest_path
    probe.FROZEN_LIBRARY_SHA256 = library_sha

    probe_argv = ["run", "--out", str(args.results),
                  "--source-commit", args.source_commit]
    for tag in args.source_tag:
        probe_argv += ["--source-tag", tag]
    probe.main(probe_argv)
    print(json.dumps({"ok": True, "machine": platform.machine(),
                      "library_sha256": library_sha, "pid": os.getpid()}))


if __name__ == "__main__":
    main()
