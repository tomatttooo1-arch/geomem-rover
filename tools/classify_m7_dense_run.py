#!/usr/bin/env python3
"""Classify one supervised dense-probe HLS run and record every case (2026-09-26).

Verdicts and process exit codes:
  0 PASS              child exited 0, C simulation, cosim C-side capture and
                      RTL post-check each report all 13 fixed cases with the
                      testbench PASS line, cosim reports PASS, the transaction
                      report has exactly 13 rows and parse_m7_dense_hls accepts it.
  1 FAIL              the design itself was rejected by a golden check: either
                      C simulation stopped before 13 cases, or the RTL ran to
                      completion and post-check stopped before 13 cases.  The
                      frozen testbench stops at the first failing case, so later
                      cases are NOT_CHECKED, never PASS.
  2 INSTRUMENT_ERROR  anything else: missing supervisor exit record, missing or
                      inconsistent reports, tool errors that do not identify a
                      golden-check failure.
  3 SIGNALED          the supervised child was terminated by a signal.

Case metadata (unique endpoints, DDA steps, counts) must be identical across
the three testbench executions, because they describe the same inputs.
"""

import argparse
import importlib.util
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location("parse_m7_dense_hls",
                                               ROOT / "tools/parse_m7_dense_hls.py")
DENSE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(DENSE)

VERDICT_CODES = {"PASS": 0, "FAIL": 1, "INSTRUMENT_ERROR": 2, "SIGNALED": 3}
CSIM_START = "*************** CSIM start ***************"
CSIM_DONE = "CSim done with 0 errors."
CAPTURE_START = "[COSIM 212-302] Starting C TB testing"
CAPTURE_END = "[COSIM 212-333] Generating C post check test bench"
POSTCHECK_START = "[COSIM 212-316] Starting C post checking"
COSIM_PASS = "C/RTL co-simulation finished: PASS"
COSIM_FAIL = "C/RTL co-simulation finished: FAIL"
PROGRESS_RE = re.compile(r'^// RTL Simulation : (\d+) / (\d+) \[[^\]]*\] @ "(\d+)"', re.MULTILINE)
CASE_FIELDS = ("index", "name", "pixels", "valid", "unique", "dda_steps",
               "accepted", "invalid", "outside")


class InstrumentError(Exception):
    pass


def _cases(text):
    rows = []
    for fields in DENSE.CASE_RE.findall(text):
        row = dict(zip(CASE_FIELDS, fields))
        for key in CASE_FIELDS:
            if key != "name":
                row[key] = int(row[key])
        rows.append(row)
    return rows


def _between(text, start, end=None):
    """Text after the single occurrence of start (and before end if given)."""
    if text.count(start) > 1:
        raise InstrumentError("phase marker occurs more than once: %s" % start)
    head, found, tail = text.partition(start)
    if not found:
        return None
    if end is not None:
        tail = tail.partition(end)[0]
    return tail


def _phase(text, start, end):
    section = _between(text, start, end)
    if section is None:
        return None
    rows = _cases(section)
    expected = DENSE.EXPECTED_CASES
    for position, row in enumerate(rows):
        if position >= len(expected) or row["index"] != position or \
                (row["name"], row["valid"]) != expected[position]:
            raise InstrumentError("case order/name/valid mismatch in phase %s" % start)
    return {"cases": rows, "tb_pass": DENSE.TB_PASS in section}


def _progress(text):
    rows = [(int(done), int(total), int(time_ps)) for done, total, time_ps in
            PROGRESS_RE.findall(text)]
    if any(total != len(DENSE.EXPECTED_CASES) for _, total, _ in rows):
        raise InstrumentError("RTL progress total is not the fixed 13 transactions")
    return rows


def _transactions(path):
    if not path.is_file():
        return None
    try:
        return DENSE._transactions(path.read_text(errors="replace"))
    except ValueError:
        return None


def classify(run_dir):
    run_dir = Path(run_dir)
    result = {"run_dir": str(run_dir.resolve()), "reasons": []}
    exit_path = run_dir / "exit.json"
    if not exit_path.is_file():
        raise InstrumentError("no supervisor exit.json: run unfinished or unsupervised")
    exit_record = json.loads(exit_path.read_text())
    result["exit"] = {key: exit_record.get(key) for key in
                      ("popen_returncode", "exit_code", "terminated_by_signal",
                       "elapsed_seconds", "start", "end")}
    log_path = run_dir / "vitis_hls.log"
    if not log_path.is_file():
        raise InstrumentError("vitis_hls.log missing")
    text = log_path.read_text(errors="replace")
    metadata = (run_dir / "ARM_METADATA.txt")
    result["arm_metadata"] = metadata.read_text().splitlines() if metadata.is_file() else None
    progress = _progress(text)
    result["rtl_progress"] = progress[-1][:2] if progress else None

    csim = _phase(text, CSIM_START, "CSIM finish")
    capture = _phase(text, CAPTURE_START, CAPTURE_END)
    post = _phase(text, POSTCHECK_START, None)
    solution = run_dir / "hls_k26_10ns/solution"
    tx = _transactions(solution / "sim/report/verilog/result.transaction.rpt")
    result["phases"] = {name: (None if phase is None else
                               {"cases": len(phase["cases"]), "tb_pass": phase["tb_pass"]})
                        for name, phase in (("csim", csim), ("cosim_capture", capture),
                                            ("rtl_postcheck", post))}
    result["transaction_rows"] = None if tx is None else len(tx)
    result["cosim_pass_marker"] = COSIM_PASS in text
    result["cosim_fail_marker"] = COSIM_FAIL in text

    # Metadata of the same inputs must agree across the three executions.
    reference = csim["cases"] if csim else []
    for name, phase in (("cosim_capture", capture), ("rtl_postcheck", post)):
        if phase:
            for row in phase["cases"]:
                if row["index"] < len(reference) and row != reference[row["index"]]:
                    raise InstrumentError("case metadata differs between csim and %s" % name)

    cases = []
    starts = {done: time_ps for done, _, time_ps in progress}
    for index, (name, valid) in enumerate(DENSE.EXPECTED_CASES):
        entry = {"index": index, "name": name, "valid": valid}
        entry["csim"] = ("PASS" if csim and index < len(csim["cases"]) else
                         "FIRST_FAIL" if csim and index == len(csim["cases"]) and
                         CSIM_DONE not in text else "NOT_CHECKED")
        if post is None:
            entry["rtl_postcheck"] = "NOT_REACHED"
        elif index < len(post["cases"]):
            entry["rtl_postcheck"] = "PASS"
        elif index == len(post["cases"]) and not post["tb_pass"]:
            entry["rtl_postcheck"] = "FIRST_FAIL"
        else:
            entry["rtl_postcheck"] = "NOT_CHECKED"
        if tx and index in tx:
            entry["latency_cycles"], entry["interval_cycles"] = tx[index]
        if index in starts and index + 1 in starts:
            entry["progress_sim_time_ps"] = starts[index + 1] - starts[index]
        if index < len(reference):
            entry["unique"] = reference[index]["unique"]
            entry["dda_steps"] = reference[index]["dda_steps"]
        cases.append(entry)
    result["cases"] = cases

    signal = exit_record.get("terminated_by_signal")
    exit_code = exit_record.get("exit_code")
    all_cases = len(DENSE.EXPECTED_CASES)
    if signal:
        result["verdict"] = "SIGNALED"
        result["reasons"].append("child terminated by %s" % signal)
        return result
    if csim is None:
        raise InstrumentError("C simulation phase not found")
    if len(csim["cases"]) < all_cases and not csim["tb_pass"] and CSIM_DONE not in text:
        result["verdict"] = "FAIL"
        result["reasons"].append("C simulation stopped at case %d" % len(csim["cases"]))
        return result
    if len(csim["cases"]) != all_cases or not csim["tb_pass"] or CSIM_DONE not in text:
        raise InstrumentError("C simulation incomplete without a testbench failure")
    if capture is None or len(capture["cases"]) != all_cases or not capture["tb_pass"]:
        raise InstrumentError("cosim C-side capture incomplete")
    if post is None:
        raise InstrumentError("RTL post-check never started")
    rtl_complete = (tx is not None and len(tx) == all_cases) or \
        (progress and progress[-1][0] == all_cases)
    if len(post["cases"]) < all_cases and not post["tb_pass"]:
        if not rtl_complete:
            raise InstrumentError("post-check failed but RTL simulation is incomplete")
        if COSIM_PASS in text:
            raise InstrumentError("post-check stopped yet cosim claims PASS")
        result["verdict"] = "FAIL"
        result["reasons"].append("RTL post-check first failure at case %d" % len(post["cases"]))
        return result
    if exit_code != 0:
        raise InstrumentError("post-check complete but child exit code %r" % exit_code)
    if len(post["cases"]) != all_cases or not post["tb_pass"] or COSIM_PASS not in text:
        raise InstrumentError("post-check incomplete or cosim PASS marker missing")
    try:
        result["cycle_table"] = DENSE.parse_dense(
            text, (solution / "sim/report/verilog/result.transaction.rpt").read_text())
    except (OSError, ValueError) as error:
        raise InstrumentError("strict dense parser rejected PASS run: %s" % error)
    result["verdict"] = "PASS"
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path,
                        help="fresh JSON path for the verdict record")
    args = parser.parse_args()
    try:
        result = classify(args.run_dir)
    except (InstrumentError, OSError, ValueError, KeyError) as error:
        result = {"run_dir": str(args.run_dir), "verdict": "INSTRUMENT_ERROR",
                  "reasons": [str(error)]}
    with open(args.output, "x") as stream:
        stream.write(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print("%s %s" % (result["verdict"], "; ".join(result.get("reasons", []))))
    sys.exit(VERDICT_CODES[result["verdict"]])


if __name__ == "__main__":
    main()
