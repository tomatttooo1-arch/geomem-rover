#!/usr/bin/env python3
"""Strict parser for the 2026-09-26 object xyz_local store-bank probe (arms A-D).

Reuses the calibrated readers of parse_m7_object_port_probe.py (transaction
rows, main-loop II, resources, physical xyz ports, cosim summary, PASS markers
and printed transaction indexes).  Only the metadata check differs, because the
arms deliberately use different source files.  Every arm must also carry a
supervisor exit record with exit code 0.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location(
    "parse_m7_object_port_probe", ROOT / "tools/parse_m7_object_port_probe.py")
OP = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(OP)

DIRECTIVES = {
    "A": "none_original_flat_xyz_local",
    "B": "array_partition_cyclic_factor3_dim1_xyz_local",
    "C": "explicit_bank_view_n3_partition_complete_dim2_xyz_local",
    "D": "array_partition_cyclic_factor3_dim1_xyz_local_index64",
}


def _metadata(run_dir, arm):
    values, hashes = {}, {}
    for line in OP._read(run_dir / "PROBE_METADATA.txt").splitlines():
        match = re.fullmatch(r"source_([0-9a-f]{64})\s+(.+)", line)
        if match:
            if match.group(2) in hashes:
                raise ValueError("duplicate source hash path")
            hashes[match.group(2)] = match.group(1)
        elif "=" in line:
            key, value = line.split("=", 1)
            if key in values:
                raise ValueError("duplicate metadata key %s" % key)
            values[key] = value
    if set(values) != {"probe", "arm", "source_commit", "part", "clock_ns", "directive"}:
        raise ValueError("incomplete probe metadata")
    if values["probe"] != "object_xyz_bank_probe" or values["arm"] != arm:
        raise ValueError("probe name or arm mismatch")
    if values["directive"] != DIRECTIVES[arm]:
        raise ValueError("directive does not match arm %s" % arm)
    if len(hashes) != 4:
        raise ValueError("expected design, header, testbench and runner hashes")
    values["source_hashes"] = dict(sorted(hashes.items()))
    return values


def parse_arm(run_dir, arm):
    run_dir = Path(run_dir).resolve()
    exit_record = json.loads(OP._read(run_dir / "exit.json"))
    if exit_record.get("exit_code") != 0:
        raise ValueError("arm %s child did not exit 0: %r" % (arm, exit_record))
    solution = run_dir / "hls_k26_10ns/solution"
    csim = OP._read(solution / "csim/report/fpga_m7_object_evidence_csim.log")
    rtl = OP._read(solution / "sim/report/verilog/fpga_m7_object_evidence.log")
    cosim = OP._read(solution / "sim/report/fpga_m7_object_evidence_cosim.rpt")
    marker = "M7 object port probe C++ testbench PASS transactions=%d" % len(OP.TRANSACTIONS)
    if "CSim done with 0 errors" not in csim or marker not in csim:
        raise ValueError("arm %s: C simulation PASS markers missing" % arm)
    if ("C/RTL co-simulation finished: PASS" not in rtl or marker not in rtl or
            not re.search(r"\|\s*Verilog\|\s*Pass\|", cosim)):
        raise ValueError("arm %s: RTL co-simulation PASS markers missing" % arm)
    OP._marked_transaction_indexes(csim, "csim")
    OP._marked_transaction_indexes(rtl, "rtl")
    return {"metadata": _metadata(run_dir, arm), "exit": exit_record,
            "main_pixel_loop": OP._main_loop(solution),
            "transactions": OP._transactions(solution),
            "cosim_cycles": OP._cosim_summary(cosim), **OP._top_report(solution)}


def compare(arms):
    parsed = {arm: parse_arm(path, arm) for arm, path in arms.items()}
    reference = parsed["A"]["metadata"]
    for arm, record in parsed.items():
        meta = record["metadata"]
        if (meta["source_commit"], meta["part"], meta["clock_ns"]) != (
                reference["source_commit"], reference["part"], reference["clock_ns"]):
            raise ValueError("arm %s differs in commit, part or clock" % arm)
    summary = {}
    for arm, record in parsed.items():
        summary[arm] = {
            "achieved_ii": record["main_pixel_loop"]["achieved_ii"],
            "xyz_bank_count": record["xyz_bank_count"],
            "total_execution_cycles": record["cosim_cycles"]["total_execution_cycles"],
            "dense_576_latency": record["transactions"]["dense_24x24_576_points"]["latency_cycles"],
            "sparse_72_latency": record["transactions"]["sparse_24x24_72_points"]["latency_cycles"],
            "resources": record["resources"],
            "latency_delta_vs_A": {name: record["transactions"][name]["latency_cycles"] -
                                   parsed["A"]["transactions"][name]["latency_cycles"]
                                   for name in OP.TRANSACTIONS},
        }
    return {"arms": parsed, "summary": summary}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for arm in "ABCD":
        parser.add_argument("--arm-%s" % arm.lower(), type=Path, required=(arm == "A"))
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    arms = {arm: getattr(args, "arm_%s" % arm.lower()) for arm in "ABCD"
            if getattr(args, "arm_%s" % arm.lower()) is not None}
    result = compare(arms)
    with open(args.output, "x") as stream:
        stream.write(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result["summary"], indent=1, sort_keys=True))


if __name__ == "__main__":
    main()
