#!/usr/bin/env python3
"""Host-side sensitivity check for the 09-26 dense frame arms (2026-09-27).

The frozen dense testbench scales the image row by inverse_k[4] = 2^-20, so a
wrong row index moves every endpoint by far less than one Q8 step and cannot
change any voxel.  It therefore cannot observe a defect in the UV arm's row
counter.  This tool runs every arm, plus two deliberately broken UV variants,
through three host checks built with the same g++ flags as the existing tests:

  dense_tb   hw/m7/frame_dense_testbench.cpp (the cosim testbench, C side)
  frame_tb   hw/m7/frame_testbench.cpp       (the production C++ testbench)
  host_suite tests/test_m7_frame_native.py   (CPU ABI vs the frozen two-stage
                                              reference, real skew/rotation K)

Faults (each substitution must apply exactly once, or the tool stops):
  UV_ROW_STUCK        the row counter never advances (v is always 0)
  UV_SKIP_INVALID     the counters advance only for pixels with valid depth

Expected answer: A, UV and QPIPE pass all three; each fault fails at least one
check, and UV_ROW_STUCK passes dense_tb (showing that testbench's blind spot).
QPIPE differs from A only by a synthesis pragma, so host checks cannot tell
them apart; its evidence is the RTL cosim, not this tool.

2026-10-11: A now implements the clip semantics (M7_CLIP_SPEC_20261011) and
host_suite checks them (it imports tools/m7_clip/golden.py, copied below).
The UV/QPIPE probes are frozen 09-27 evidence that still DROP out-of-grid
rays, so their host_suite verdict is expected to be FAIL from now on.
"""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
ARMS = {
    "A": "hw/m7/frame_depth.cpp",
    "UV": "hw/m7/probe/frame_depth_uv_counter.cpp",
    "QPIPE": "hw/m7/probe/frame_depth_quant_pipeline.cpp",
}
ADVANCE = ("        if (++column == width) {\n"
           "            column = 0;\n"
           "            ++row_index;\n"
           "        }\n")
INVALID_EXIT = ("        if (!(z > 0.1f && z < 10.0f)) {\n"
                "            ++invalid;\n"
                "            continue;\n"
                "        }\n")


def _replace_once(text, old, new, label):
    if text.count(old) != 1:
        raise SystemExit("fault %s: pattern found %d times, expected 1"
                         % (label, text.count(old)))
    return text.replace(old, new)


def variants():
    out = {}
    for arm, rel in ARMS.items():
        out[arm] = (ROOT / rel).read_text().replace('#include "../frame_depth.hpp"',
                                                    '#include "frame_depth.hpp"')
    # Host g++ runs with -Werror; production guards HLS pragmas with
    # __SYNTHESIS__, the QPIPE probe did not.  Guarding it is a no-op for the
    # host build (g++ ignores the pragma either way) and is recorded here.
    out["QPIPE"] = _replace_once(out["QPIPE"], "#pragma HLS PIPELINE II=1\n",
                                 "#ifdef __SYNTHESIS__\n#pragma HLS PIPELINE II=1\n#endif\n",
                                 "QPIPE/guard")
    uv = out["UV"]
    out["UV_ROW_STUCK"] = _replace_once(uv, "            ++row_index;\n",
                                        "            (void)row_index;\n", "UV_ROW_STUCK")
    moved = _replace_once(uv, ADVANCE, "", "UV_SKIP_INVALID/remove")
    out["UV_SKIP_INVALID"] = _replace_once(moved, INVALID_EXIT, INVALID_EXIT + ADVANCE,
                                           "UV_SKIP_INVALID/insert")
    return out


def _run(cmd, cwd, env=None, timeout=900):
    proc = subprocess.run(cmd, cwd=str(cwd), env=env, capture_output=True,
                          text=True, timeout=timeout)
    return proc.returncode, (proc.stdout + proc.stderr)[-4000:]


def check_variant(name, source, workdir, python):
    tree = workdir / name
    for sub in ("host", "hw", "tests", "tools/m7_clip"):
        shutil.copytree(ROOT / sub, tree / sub,
                        ignore=shutil.ignore_patterns("__pycache__"))
    (tree / "hw/m7/frame_depth.cpp").write_text(source)
    result = {}
    flags = ["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
             "-ffp-contract=off", "hw/m7/frame_depth.cpp"]
    for label, tb in (("dense_tb", "hw/m7/frame_dense_testbench.cpp"),
                      ("frame_tb", "hw/m7/frame_testbench.cpp")):
        rc, log = _run(flags + [tb, "-o", label], tree)
        if rc != 0:
            result[label] = {"verdict": "BUILD_ERROR", "log": log}
            continue
        rc, log = _run(["./" + label], tree)
        result[label] = {"verdict": "PASS" if rc == 0 and "PASS" in log else "FAIL",
                         "exit_code": rc, "tail": log.strip().splitlines()[-1:] }
    env = dict(os.environ, PYTHONHASHSEED="0", PYTHONPATH=".")
    rc, log = _run([python, "-m", "unittest", "discover", "-s", "tests",
                    "-p", "test_m7_frame_native.py"], tree, env)
    summary = [line for line in log.splitlines() if line.startswith(("Ran ", "OK", "FAILED"))]
    result["host_suite"] = {"verdict": "PASS" if rc == 0 else "FAIL",
                            "exit_code": rc, "summary": summary,
                            "tail": log[-1500:] if rc else ""}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--python", default=sys.executable)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit("output exists; choose a fresh path")
    results = {}
    with tempfile.TemporaryDirectory(prefix="m7-dense-sens-") as tmp:
        for name, source in variants().items():
            results[name] = check_variant(name, source, Path(tmp), args.python)
            print(name, {k: v["verdict"] for k, v in results[name].items()}, flush=True)
    expected_pass = all(results[a][c]["verdict"] == "PASS"
                        for a in ARMS for c in ("dense_tb", "frame_tb", "host_suite"))
    faults_caught = all(any(results[f][c]["verdict"] != "PASS"
                            for c in ("dense_tb", "frame_tb", "host_suite"))
                        for f in ("UV_ROW_STUCK", "UV_SKIP_INVALID"))
    report = {"python": args.python, "results": results,
              "arms_all_pass": expected_pass, "every_fault_caught": faults_caught,
              "dense_tb_blind_to_row_stuck":
                  results["UV_ROW_STUCK"]["dense_tb"]["verdict"] == "PASS"}
    args.output.write_text(json.dumps(report, indent=1, sort_keys=True) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "results"}))
    return 0 if expected_pass and faults_caught else 1


if __name__ == "__main__":
    sys.exit(main())
