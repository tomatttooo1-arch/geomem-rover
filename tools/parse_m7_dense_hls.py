#!/usr/bin/env python3
"""Strictly join dense-probe metadata to Vitis RTL transaction cycles."""

import argparse
import csv
import json
from pathlib import Path
import re


CASE_RE = re.compile(
    r"^DENSE_CASE index=(\d+) name=([a-z0-9_]+) pixels=(\d+) valid=(\d+) "
    r"unique=(\d+) dda_steps=(\d+) accepted=(\d+) invalid=(\d+) outside=(\d+)$",
    re.MULTILINE)
TX_RE = re.compile(r"^transaction\s+(\d+):\s+(\d+)\s+(\d+|x)\s*$", re.MULTILINE)
COSIM_PASS = "C/RTL co-simulation finished: PASS"
TB_PASS = "M7 dense frame testbench PASS"
EXPECTED_CASES = (
    ("invalid", 0),
    ("same_short_64", 64), ("same_short_512", 512),
    ("same_short_4096", 4096),
    ("vary_short_64", 64), ("vary_short_512", 512),
    ("vary_short_4096", 4096),
    ("same_long_64", 64), ("same_long_512", 512),
    ("same_long_4096", 4096),
    ("vary_long_64", 64), ("vary_long_512", 512),
    ("vary_long_4096", 4096),
)


def _transactions(text):
    matches = TX_RE.findall(text)
    if not matches:
        raise ValueError("no RTL transactions found")
    result = {}
    for raw_index, raw_latency, raw_interval in matches:
        index = int(raw_index)
        if index in result:
            raise ValueError("duplicate RTL transaction index")
        result[index] = (int(raw_latency), None if raw_interval == "x" else int(raw_interval))
    if sorted(result) != list(range(len(result))):
        raise ValueError("RTL transaction indexes must be contiguous from zero")
    return result


def _last_case_set(text):
    raw = CASE_RE.findall(text)
    count = len(EXPECTED_CASES)
    if not raw or len(raw) % count:
        raise ValueError("dense case log contains a truncated group")
    groups = []
    for base in range(0, len(raw), count):
        records = []
        for expected_index, fields in enumerate(raw[base:base+count]):
            values = {
                "index": int(fields[0]), "name": fields[1], "pixels": int(fields[2]),
                "valid": int(fields[3]), "unique": int(fields[4]),
                "dda_steps": int(fields[5]), "accepted": int(fields[6]),
                "invalid": int(fields[7]), "outside": int(fields[8]),
            }
            expected_name, expected_valid = EXPECTED_CASES[expected_index]
            if (values["index"] != expected_index or values["name"] != expected_name or
                    values["valid"] != expected_valid):
                raise ValueError("dense case group has wrong name, order, index, or valid matrix")
            if values["pixels"] != 4096:
                raise ValueError("dense case pixel extent changed")
            if values["accepted"] != values["valid"]:
                raise ValueError("dense metadata accepted/valid mismatch")
            if values["accepted"] + values["invalid"] + values["outside"] != values["pixels"]:
                raise ValueError("dense metadata count conservation failed")
            records.append(values)
        groups.append(records)
    return groups[-1]


def parse_dense(log_text, transaction_text):
    if COSIM_PASS not in log_text:
        raise ValueError("HLS log lacks C/RTL co-simulation PASS")
    if TB_PASS not in log_text:
        raise ValueError("HLS log lacks dense testbench PASS")
    transactions = _transactions(transaction_text)
    if len(transactions) != len(EXPECTED_CASES):
        raise ValueError("RTL transaction count does not match fixed dense matrix")
    cases = _last_case_set(log_text)
    baselines = [item for item in cases if item["name"] == "invalid" and item["valid"] == 0]
    if len(baselines) != 1:
        raise ValueError("exactly one all-invalid baseline is required")
    baseline_latency = transactions[baselines[0]["index"]][0]
    output = []
    for item in cases:
        latency, interval = transactions[item["index"]]
        delta = latency - baseline_latency
        record = dict(item)
        record.update({
            "latency_cycles": latency,
            "interval_cycles": interval,
            "baseline_latency_cycles": baseline_latency,
            "incremental_cycles": delta,
            "incremental_cycles_per_valid": None if item["valid"] == 0 else delta / item["valid"],
        })
        output.append(record)
    return output


def write_outputs(records, json_path, csv_path):
    json_path, csv_path = Path(json_path), Path(csv_path)
    if json_path.exists() or csv_path.exists():
        raise FileExistsError("derived output paths must be fresh")
    with json_path.open("x") as stream:
        stream.write(json.dumps(records, indent=2, sort_keys=True) + "\n")
    fields = list(records[0])
    with csv_path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", required=True)
    parser.add_argument("--transaction-report", required=True)
    parser.add_argument("--json", required=True)
    parser.add_argument("--csv", required=True)
    args = parser.parse_args()
    records = parse_dense(Path(args.log).read_text(errors="replace"),
                          Path(args.transaction_report).read_text(errors="replace"))
    write_outputs(records, args.json, args.csv)
    print("parsed %d RTL transactions" % len(records))


if __name__ == "__main__":
    main()
